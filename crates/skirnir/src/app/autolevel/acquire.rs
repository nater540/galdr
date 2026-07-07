//! Height-map acquisition: the serpentine grid-probe wizard that fills a [`Mesh`] with probed Z deltas.
//!
//! This is Part B2/B3 of the probing plan, mirroring OpenCNCPilot / ioSender's `HeightMapControl` (MIT). It walks
//! the mesh nodes in a boustrophedon (serpentine) raster — so the tool never rapids all the way back across the
//! stock between rows — retracting to a safe machine-Z clearance between points, and probes each node's surface
//! with the SAME two-stage `G38.3` Z touch the datum finder uses ([`super::super::datum::touch_lines`]). The FIRST
//! accepted point becomes the reference `Z₀`; every stored value is `Z − Z₀`, so the mesh holds DELTAS from the
//! first point (a flat surface → all-zero → the correction is the identity). A missed point (`G38.3`'s software
//! `:0`) aborts acquisition fail-closed rather than storing a bogus reading.
//!
//! Pure — no egui, no I/O. Each point resolves through the existing DOC-11 probe latch
//! ([`super::super::view_state::ProbeOp`]); the shell ([`super::super::shell`]) sends the lines this module builds
//! and folds each resolved [`ProbeOutcome`] back in. Host-tested without a window or hardware.

use super::super::datum::{ProbeParams as DatumProbeParams, Touch, touch_lines};
use super::super::intent::{Axis, Dir};
use super::super::view_state::ProbeOutcome;
use super::mesh::Mesh;

/// Conservative default machine-Z (mm) the probe retracts to between points (a `G53` move). grbl machine-Z is
/// typically negative below the home/top; a small negative keeps the probe clear of the stock while positioning.
pub const DEFAULT_CLEARANCE_Z_MM: f64 = -2.0;

/// Default fast Z search feed (mm/min) for the grid touch.
pub const DEFAULT_PROBE_FEED: f64 = 200.0;

/// Default maximum fast Z travel (mm) seeking the surface from the clearance height before giving up (a miss).
pub const DEFAULT_PROBE_DEPTH_MM: f64 = 25.0;

/// Default retract (mm) between the fast and slow Z passes — the latch.
pub const DEFAULT_LATCH_DISTANCE_MM: f64 = 2.0;

/// Default slow latch feed (mm/min) — the precise, KEPT Z reading's feed.
pub const DEFAULT_LATCH_FEED: f64 = 50.0;

/// Default positioning rapids feed (mm/min). The `G0` moves ignore it, but a machine configured for `G1`-paced
/// positioning (or a later `G1` positioning option) reads it; surfaced so the operator can tune it.
pub const DEFAULT_RAPIDS_FEED: f64 = 1000.0;

/// The bench-tuned grid-probe parameters, surfaced in the mesh-probe panel and threaded into each point's lines.
/// `serde` so it can persist alongside the mesh if desired; `Copy` because it is a handful of scalars.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct GridProbeParams {
  /// Machine-Z (mm) retracted to between points (`G53 G0 Z<clearance_z>`). See [`DEFAULT_CLEARANCE_Z_MM`].
  pub clearance_z: f64,
  /// Fast Z search feed (mm/min). See [`DEFAULT_PROBE_FEED`].
  pub probe_feed: f64,
  /// Maximum fast Z travel (mm) seeking the surface. See [`DEFAULT_PROBE_DEPTH_MM`].
  pub probe_depth: f64,
  /// Retract (mm) between the fast and slow Z passes. See [`DEFAULT_LATCH_DISTANCE_MM`].
  pub latch_distance: f64,
  /// Slow latch feed (mm/min) — the KEPT Z reading. See [`DEFAULT_LATCH_FEED`].
  pub latch_feed: f64,
  /// Positioning rapids feed (mm/min). See [`DEFAULT_RAPIDS_FEED`].
  pub rapids_feed: f64,
  /// The spindle→probe X offset (mm): if the probe is mounted beside the spindle, the spindle is commanded to
  /// `node_x − probe_offset_x` so the PROBE lands on the node. `0` when the probe is the tool (in-spindle).
  pub probe_offset_x: f64,
  /// The spindle→probe Y offset (mm). See [`Self::probe_offset_x`].
  pub probe_offset_y: f64,
}

impl Default for GridProbeParams {
  fn default() -> Self {
    GridProbeParams {
      clearance_z: DEFAULT_CLEARANCE_Z_MM,
      probe_feed: DEFAULT_PROBE_FEED,
      probe_depth: DEFAULT_PROBE_DEPTH_MM,
      latch_distance: DEFAULT_LATCH_DISTANCE_MM,
      latch_feed: DEFAULT_LATCH_FEED,
      rapids_feed: DEFAULT_RAPIDS_FEED,
      probe_offset_x: 0.0,
      probe_offset_y: 0.0,
    }
  }
}

/// The node visit order for a serpentine (boustrophedon) raster over `mesh`, COLUMN-major: column 0 bottom→top,
/// column 1 top→bottom, and so on, so the tool snakes up and down each column rather than rapiding across the
/// stock at the end of every column. Returns `(ix, iy)` pairs in visit order.
pub fn grid_points_serpentine(mesh: &Mesh) -> Vec<(usize, usize)> {
  let mut order = Vec::with_capacity(mesh.nx * mesh.ny);
  for ix in 0..mesh.nx {
    // Even columns run bottom→top, odd columns top→bottom — the snake that avoids a long return rapid per column.
    if ix % 2 == 0 {
      for iy in 0..mesh.ny {
        order.push((ix, iy));
      }
    } else {
      for iy in (0..mesh.ny).rev() {
        order.push((ix, iy));
      }
    }
  }
  order
}

/// The exact line sequence to probe one node whose PROBE-tip target is `probe_xy` (work-mm, the node minus the
/// spindle→probe offset), using the shared grid params. In send order:
///
/// 0. `G21` — establish MILLIMETRES first. Every following value (the `clearance_z` machine-Z, the work-XY node,
///    the two-stage probe distances) is in mm, but a machine left in `G20` (inch) would interpret them as inches
///    and mis-position the AUTOMATED grid move 25.4× — a probe crash. Asserting `G21` up front makes the whole
///    sequence unit-safe regardless of the prior modal units. `G21` is idempotent.
/// 1. `G53 G0 Z<clearance_z>` — retract to the safe machine-Z clearance before moving in XY.
/// 2. `G90 G0 X<x> Y<y>` — rapid to the node in WORK coordinates (the mesh is a work-coordinate grid, so the
///    firmware applies the active WCS; a `G53` machine move would need a WCO this pure builder does not have).
/// 3. The two-stage `G38.3` Z-down touch (fast → retract → slow-kept → `G90`), reused verbatim from the datum
///    finder so the acquisition and the datum ops share one latch/touch contract.
/// 4. `G53 G0 Z<clearance_z>` — retract back to the clearance so the next point's positioning is safe.
pub fn point_probe_lines(probe_xy: (f64, f64), params: &GridProbeParams) -> Vec<String> {
  // Map the grid params onto the datum two-stage Z-touch tuning (only these four fields drive a Z `touch_lines`).
  let touch_params = DatumProbeParams {
    probe_distance: params.probe_depth,
    latch_distance: params.latch_distance,
    probe_feed: params.probe_feed,
    latch_feed: params.latch_feed,
    ..DatumProbeParams::default()
  };
  let mut lines = vec![
    // Establish mm before any unit-sensitive move (the machine-Z clearance, the work-XY node, the probe distance).
    "G21".to_string(),
    format!("G53 G0 Z{:.3}", params.clearance_z),
    format!("G90 G0 X{:.3} Y{:.3}", probe_xy.0, probe_xy.1),
  ];
  lines.extend(touch_lines(Touch { axis: Axis::Z, dir: Dir::Neg }, &touch_params));
  lines.push(format!("G53 G0 Z{:.3}", params.clearance_z));
  lines
}

/// The acquisition wizard's step. Ready = awaiting the operator to trigger the next point; Probing = a point is in
/// flight; Done = every node captured; Aborted = a miss/failure stopped the run.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeshProbeStep {
  /// Awaiting the operator to trigger the next point's probe.
  Ready,
  /// A point has been issued; awaiting its result.
  Probing,
  /// Every node has been captured — the mesh is complete and can be saved.
  Done,
  /// A point failed (miss / alarm) or the run was cancelled. Terminal.
  Aborted,
}

/// The live acquisition state: the mesh being filled, the params, the serpentine visit order + cursor, the
/// reference `Z₀`, and the step. Pure — the shell drives it (sending each point's lines, feeding back each
/// [`ProbeOutcome`]) and the view renders it. `Z₀` is the first ACCEPTED reading; every stored value is `Z − Z₀`.
#[derive(Debug, Clone, PartialEq)]
pub struct MeshProbeState {
  /// The mesh being filled with probed deltas.
  mesh: Mesh,
  /// The bench-tuned grid-probe params shared across the run.
  params: GridProbeParams,
  /// The serpentine node visit order.
  order: Vec<(usize, usize)>,
  /// The index into `order` of the node currently being probed (`Probing`) or due next (`Ready`). Equals the
  /// number of nodes already captured.
  cursor: usize,
  /// The first accepted machine-Z reading — the reference every delta subtracts. `None` until the first point.
  z0: Option<f64>,
  /// The current step.
  pub step: MeshProbeStep,
  /// The reason the run aborted, set with [`MeshProbeStep::Aborted`].
  pub abort_reason: Option<String>,
}

impl MeshProbeState {
  /// Start a fresh acquisition over `mesh` (its `z`/`max_height` are reset to a clean slate) with `params`. An
  /// empty grid (no nodes) starts already `Done`.
  pub fn new(mut mesh: Mesh, params: GridProbeParams) -> Self {
    // Clear any prior deltas so a re-probe starts from a clean flat mesh rather than accumulating onto old values.
    for z in mesh.z.iter_mut() {
      *z = 0.0;
    }
    mesh.max_height = 0.0;
    let order = grid_points_serpentine(&mesh);
    let step = if order.is_empty() { MeshProbeStep::Done } else { MeshProbeStep::Ready };
    MeshProbeState { mesh, params, order, cursor: 0, z0: None, step, abort_reason: None }
  }

  /// Whether a point is currently awaiting its result (so the shell gates the latch / disables advance).
  pub fn is_probing(&self) -> bool {
    self.step == MeshProbeStep::Probing
  }

  /// Whether every node has been captured.
  pub fn is_done(&self) -> bool {
    self.step == MeshProbeStep::Done
  }

  /// Acquisition progress as `(captured, total)` node counts, for a progress bar.
  pub fn progress(&self) -> (usize, usize) {
    (self.cursor, self.order.len())
  }

  /// The nodes already captured, in visit order — the first `cursor` of the serpentine order. Used by the panel's
  /// grid preview to shade probed nodes (by their stored delta) distinctly from the not-yet-probed ones.
  pub fn probed(&self) -> &[(usize, usize)] {
    let n = self.cursor.min(self.order.len());
    &self.order[..n]
  }

  /// The node `(ix, iy)` due to be probed next (or in flight), or `None` when the run is finished/terminal.
  pub fn current_node(&self) -> Option<(usize, usize)> {
    match self.step {
      MeshProbeStep::Ready | MeshProbeStep::Probing => self.order.get(self.cursor).copied(),
      _ => None,
    }
  }

  /// The completed mesh, for the shell to persist once the run is [`MeshProbeStep::Done`].
  pub fn mesh(&self) -> &Mesh {
    &self.mesh
  }

  /// Begin the next point's probe: from `Ready` into `Probing`, returning the exact line sequence to send (the
  /// clearance retract, the work-XY rapid to the node minus the probe offset, and the two-stage `G38.3` touch), or
  /// `None` if no point is due (a guard against a double-advance or a finished run). The shell sends the lines and
  /// arms the Phase 0 latch before the SLOW pass.
  pub fn begin_next_point(&mut self) -> Option<Vec<String>> {
    if self.step != MeshProbeStep::Ready {
      return None;
    }
    let (ix, iy) = *self.order.get(self.cursor)?;
    self.step = MeshProbeStep::Probing;
    let (nx, ny) = self.mesh.point_xy(ix, iy);
    // Command the spindle so the PROBE tip (offset from the spindle) lands on the node.
    let probe_xy = (nx - self.params.probe_offset_x, ny - self.params.probe_offset_y);
    Some(point_probe_lines(probe_xy, &self.params))
  }

  /// Fold one resolved probe outcome into the run. A failure (a `G38.3` miss `:0`, or an alarm) aborts fail-closed
  /// — no partial mesh is presented. A success reads the machine-Z (index 2): the first accepted reading becomes
  /// `Z₀`, and each stored value is `Z − Z₀`. The cursor advances; when every node is captured the run is `Done`.
  pub fn on_probe_result(&mut self, outcome: &ProbeOutcome) {
    if self.step != MeshProbeStep::Probing {
      return; // a result on a non-probing step is unexpected; ignore rather than corrupt state.
    }
    let position = match outcome {
      ProbeOutcome::Success { position } => position,
      ProbeOutcome::Failure { reason } => {
        self.abort(format!("probe failed: {reason}"));
        return;
      }
    };
    let Some(&z) = position.get(Axis::Z.index()) else {
      self.abort("probe result has no Z axis".to_string());
      return;
    };
    // The first accepted reading is the reference; every stored value is the deviation from it.
    let z0 = *self.z0.get_or_insert(z);
    if let Some(&(ix, iy)) = self.order.get(self.cursor) {
      self.mesh.set_delta(ix, iy, z - z0);
    }
    self.cursor += 1;
    self.step = if self.cursor >= self.order.len() { MeshProbeStep::Done } else { MeshProbeStep::Ready };
  }

  /// Abort the run with a reason — terminal. No partial mesh is presented for saving.
  pub fn abort(&mut self, reason: impl Into<String>) {
    self.step = MeshProbeStep::Aborted;
    self.abort_reason = Some(reason.into());
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  fn success(z: f64) -> ProbeOutcome {
    // A `[PRB:]` machine position with the Z at the conventional index 2.
    ProbeOutcome::Success { position: vec![0.0, 0.0, z] }
  }

  fn mesh_2x2() -> Mesh {
    // A 2×2 grid over [0,10]² (spacing 10 → 2 nodes/axis, grid 10).
    Mesh::from_spacing((0.0, 0.0), (10.0, 10.0), (10.0, 10.0))
  }

  #[test]
  fn serpentine_order_sweeps_back_and_forth() {
    // A 3×2 grid, column-major snake: column 0 bottom→top, column 1 top→bottom, column 2 bottom→top.
    let mesh = Mesh::from_spacing((0.0, 0.0), (20.0, 10.0), (10.0, 10.0)); // nx=3, ny=2.
    assert_eq!((mesh.nx, mesh.ny), (3, 2));
    let order = grid_points_serpentine(&mesh);
    assert_eq!(order, vec![(0, 0), (0, 1), (1, 1), (1, 0), (2, 0), (2, 1)]);
  }

  #[test]
  fn point_probe_lines_retracts_positions_touches_and_retracts() {
    let params = GridProbeParams { clearance_z: -1.5, probe_offset_x: 0.0, probe_offset_y: 0.0, ..Default::default() };
    let lines = point_probe_lines((5.0, 7.0), &params);
    // The sequence establishes mm FIRST (unit-safe under a G20 machine), then the machine-Z clearance retract,
    // then a WORK-coord XY rapid to the node.
    assert_eq!(lines[0], "G21", "the grid probe must establish mm before any unit-sensitive move; got {lines:?}");
    assert_eq!(lines[1], "G53 G0 Z-1.500");
    assert_eq!(lines[2], "G90 G0 X5.000 Y7.000");
    // The middle is the two-stage no-alarm Z touch (exactly two G38.3 passes, never G38.2).
    let probes: Vec<&String> = lines.iter().filter(|l| l.contains("G38")).collect();
    assert_eq!(probes.len(), 2, "a grid touch is a two-stage probe; got {lines:?}");
    for p in &probes {
      assert!(p.contains("G38.3") && !p.contains("G38.2"), "a grid touch must be the no-alarm G38.3; got {p:?}");
      assert!(p.contains('Z'), "the grid touch probes Z; got {p:?}");
    }
    // And a final retract back to the clearance so the next point is safe to position.
    assert_eq!(lines.last().unwrap(), "G53 G0 Z-1.500");
  }

  #[test]
  fn the_probe_offset_shifts_the_commanded_xy_so_the_probe_lands_on_the_node() {
    // With a probe mounted +3 X / −2 Y from the spindle, probing node (5,7) commands the spindle to (2,9).
    let mut state = MeshProbeState::new(mesh_2x2(), GridProbeParams { probe_offset_x: 3.0, probe_offset_y: -2.0, ..Default::default() });
    // The first serpentine node is (0,0) → work (0,0); with the offset the spindle goes to (-3, 2).
    let lines = state.begin_next_point().expect("a point is due");
    assert!(lines.iter().any(|l| l == "G90 G0 X-3.000 Y2.000"), "the offset must shift the commanded XY; got {lines:?}");
  }

  #[test]
  fn a_full_run_stores_deltas_from_the_first_point_and_finishes_done() {
    // Four nodes; the machine-Z readings are 5.0, 5.2, 4.9, 5.1. Z₀ = 5.0, so deltas are 0, +0.2, −0.1, +0.1 in
    // COLUMN-major serpentine order (0,0),(0,1),(1,1),(1,0).
    let mut state = MeshProbeState::new(mesh_2x2(), GridProbeParams::default());
    assert_eq!(state.progress(), (0, 4));
    for z in [5.0, 5.2, 4.9, 5.1] {
      assert!(state.begin_next_point().is_some(), "a point is due until Done");
      state.on_probe_result(&success(z));
    }
    assert!(state.is_done(), "every node captured → Done");
    assert_eq!(state.progress(), (4, 4));
    let m = state.mesh();
    // Deltas by node index (order was (0,0),(0,1),(1,1),(1,0)).
    assert!((m.z[m.index(0, 0)] - 0.0).abs() < 1e-9, "the first point is the reference (delta 0)");
    assert!((m.z[m.index(0, 1)] - 0.2).abs() < 1e-9);
    assert!((m.z[m.index(1, 1)] + 0.1).abs() < 1e-9);
    assert!((m.z[m.index(1, 0)] - 0.1).abs() < 1e-9);
    // max_height is the greatest delta (+0.2), the out-of-grid lift.
    assert!((m.max_height - 0.2).abs() < 1e-9);
  }

  #[test]
  fn the_first_point_is_always_the_zero_reference() {
    let mut state = MeshProbeState::new(mesh_2x2(), GridProbeParams::default());
    state.begin_next_point();
    // Even a non-zero first machine-Z reads as delta 0 (it defines Z₀).
    state.on_probe_result(&success(-12.345));
    assert!((state.mesh().z[0] - 0.0).abs() < 1e-9, "the first point defines Z₀, so its own delta is 0");
  }

  #[test]
  fn a_missed_point_aborts_the_run_fail_closed() {
    let mut state = MeshProbeState::new(mesh_2x2(), GridProbeParams::default());
    state.begin_next_point();
    state.on_probe_result(&success(5.0)); // first point ok.
    state.begin_next_point();
    // The second point misses (G38.3 → `:0`, surfaced as a Failure).
    state.on_probe_result(&ProbeOutcome::Failure { reason: "probe did not contact (flag :0)".to_string() });
    assert_eq!(state.step, MeshProbeStep::Aborted);
    assert!(state.abort_reason.as_deref().unwrap().contains(":0"));
    // No further points are issued after an abort.
    assert_eq!(state.begin_next_point(), None, "an aborted run issues no more points");
  }

  #[test]
  fn a_short_reading_aborts_rather_than_panicking() {
    let mut state = MeshProbeState::new(mesh_2x2(), GridProbeParams::default());
    state.begin_next_point();
    state.on_probe_result(&ProbeOutcome::Success { position: vec![] });
    assert_eq!(state.step, MeshProbeStep::Aborted);
  }

  #[test]
  fn begin_next_point_is_guarded_against_a_double_advance() {
    let mut state = MeshProbeState::new(mesh_2x2(), GridProbeParams::default());
    assert!(state.begin_next_point().is_some());
    // A second begin while a point is in flight (Probing) must be refused.
    assert_eq!(state.begin_next_point(), None, "no double-advance while probing");
  }
}
