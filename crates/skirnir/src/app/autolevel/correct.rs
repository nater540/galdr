//! Pre-stream height-map correction: a pure, whole-program rewrite that shifts Z to follow a probed surface.
//!
//! This is Part C of the probing plan, mirroring OpenCNCPilot / ioSender's autolevel (MIT), reimplemented
//! clean-room and HARDENED past ioSender's known blind spots. Given a loaded program, a probed [`Mesh`], and a
//! [`CorrectionConfig`], [`correct_program`] returns a new program whose every cutting move carries a Z shifted
//! by `mesh(x, y)` so the tool tracks the real surface — warped stock, un-tram'd copper — at a constant cut
//! depth relative to it. The datum the operator set is untouched; only the *deviation* from flat is followed.
//!
//! **Driven by the shared [`cnc_kinematics::gcode::Parser`]** — the SAME parser the firmware runs and [`crate::eta`]
//! already drives over a whole program — so there is no second G-code interpreter to drift. Each line decodes to a
//! typed [`PlannerCommand`] while the parser tracks the live modal state (units, distance, plane, feed-mode). That
//! is how we beat ioSender's G91/G20 blind spot: incremental/inch inputs are resolved to absolute-mm INTERNALLY and
//! re-emitted absolute, with a canonical `G90 G21 G94` header and every corrected motion line re-asserting `G90 G21`
//! so a passed-through source modal word can never re-interpret a corrected coordinate.
//!
//! **What is corrected vs passed through:**
//! - **G1 feed moves** are subdivided (`ceil(planar_len / seg)`, `seg = min(grid_x, grid_y)`); each sub-endpoint's
//!   Z is the programmed Z linearly interpolated along the move PLUS `mesh(x, y)`.
//! - **G0 rapids** are Z-corrected but NOT subdivided (a rapid needs no surface fidelity); the `correct_rapids`
//!   toggle (default on) can leave their Z uncorrected.
//! - **G2/G3 arcs** stay real arcs: split into short sub-arcs along the sweep with **I/J recomputed from the true
//!   centre per sub-arc**, Z ramped along the helix plus `mesh` at each sub-endpoint. XY-plane only.
//! - **Pure-XY moves** (no Z word) get Z injected (`last_programmed_z + mesh`) so they follow the surface — EXCEPT
//!   a motion line with NO axis words at all (an F-only / S-only line) passes through untouched (ioSender bug #451).
//! - **Everything else** — comments, M/S/T, dwell, spindle, coolant, `$`-settings, non-frame coordinate ops —
//!   passes through verbatim.
//!
//! **Fail-closed hazards** (return an `Err` rather than emit a half-corrected program): a `G53` machine move
//! passes through but invalidates the tracked position, so a following *incremental* move is [`CorrectionError::PositionUnknown`];
//! a mid-body `G10`/`G92`/`G92.1` is [`CorrectionError::FrameShiftMidProgram`] (a *leading* preamble one passes
//! through — it defines the frame the mesh was probed in); `G93` inverse-time on a corrected move is
//! [`CorrectionError::InverseTimeFeed`]; a G18/G19 arc is [`CorrectionError::NonXyArc`] and an R-form (no I/J) arc
//! is [`CorrectionError::ArcRadiusForm`]. Pure and fully host-tested — no hardware, no window.

use cnc_kinematics::gcode::{AxisWords, CoordinateOp, DistanceMode, FeedMode, Parser, PlannerCommand, Plane, Units};

use super::mesh::Mesh;
use super::segment::{ArcSpan, planar_distance, subdivision_count};

/// Options for the correction pass. `seg` is fixed at `min(grid_x, grid_y)` (ioSender's rule), so the only knob
/// is whether rapids are Z-corrected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CorrectionConfig {
  /// Whether G0 rapids have their Z corrected by the mesh (default `true`). Off leaves a rapid's Z at the
  /// programmed value — some operators prefer un-corrected clearance/travel rapids.
  pub correct_rapids: bool,
}

impl Default for CorrectionConfig {
  fn default() -> Self {
    CorrectionConfig { correct_rapids: true }
  }
}

/// Why a program could not be safely corrected. Every variant means "do not emit a half-corrected program" — the
/// caller streams the source verbatim (autolevel off) or surfaces the reason and refuses to stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum CorrectionError {
  /// A move needs the current position (an incremental move, or an arc start) but it is unknown — typically after
  /// a `G53` machine move invalidated the tracked work position.
  #[error("position unknown (an incremental move or arc follows a machine-frame move)")]
  PositionUnknown,
  /// A `G10`/`G92`/`G92.1` frame shift appeared MID-program (after cutting began); it would move the frame the
  /// mesh was probed against out from under the correction.
  #[error("a coordinate-frame shift (G10/G92) appeared mid-program")]
  FrameShiftMidProgram,
  /// A corrected move is under `G93` inverse-time feed; subdividing it would break the per-move inverse-time `F`.
  #[error("G93 inverse-time feed cannot be corrected (subdividing breaks the per-move feed)")]
  InverseTimeFeed,
  /// A G2/G3 arc is in the G18/G19 plane; the correction handles XY-plane (G17) arcs only.
  #[error("a non-XY-plane arc (G18/G19) cannot be corrected")]
  NonXyArc,
  /// A G2/G3 arc carries no I/J centre offsets (an R-form arc); the correction needs the IJK centre geometry.
  #[error("an R-form arc (no I/J centre) cannot be corrected")]
  ArcRadiusForm,
}

/// Correct `program` against `mesh` per `cfg`, returning the rewritten program or the first hazard encountered.
/// Pure: a whole-program up-front rewrite, no I/O. The output opens with a canonical `G90 G21 G94` header.
pub fn correct_program(program: &[String], mesh: &Mesh, cfg: &CorrectionConfig) -> Result<Vec<String>, CorrectionError> {
  let mut c = Corrector::new(mesh, cfg);
  c.out.push("G90 G21 G94".to_string());
  let mut parser = Parser::new();
  for line in program {
    match parser.parse_line(line.as_bytes()) {
      // A typed command: correct a motion, fail-close a hazard, or pass a non-motion command through verbatim.
      Ok(Some(command)) => c.handle_command(command, line)?,
      // No command (comment, blank, `$`-setting, or a pure modal change like G90/G20): pass through verbatim. A
      // corrected motion line re-asserts G90/G21, so a passed-through modal word cannot corrupt a later coordinate.
      Ok(None) => c.out.push(line.clone()),
      // A line the shared parser rejects is not something the correction understands; pass it through unchanged so
      // the firmware sees exactly what it would without autolevel (the parser leaves modal state intact on error).
      Err(_) => c.out.push(line.clone()),
    }
  }
  Ok(c.out)
}

/// The pure correction state machine: the mesh/config, the subdivision segment length, the tracked WORK-mm
/// position (`cx`/`cy` and the programmed Z `pz`), whether cutting has begun (for the leading-vs-mid-body frame
/// rule), and the emitted output.
struct Corrector<'a> {
  mesh: &'a Mesh,
  cfg: &'a CorrectionConfig,
  /// The subdivision segment length: `min(grid_x, grid_y)`.
  seg: f64,
  /// Current work-mm X, or `None` when unknown (start, or after a machine-frame move).
  cx: Option<f64>,
  /// Current work-mm Y, or `None` when unknown.
  cy: Option<f64>,
  /// The last PROGRAMMED (pre-mesh) absolute work Z — the base a pure-XY move keeps and the start of a Z ramp.
  /// `None` until the first Z word (opening pure-XY moves pass through un-injected).
  pz: Option<f64>,
  /// The current absolute A (rotary) position, so a subdivided move can interpolate A across its sub-moves rather
  /// than jumping it at the end. `None` until the first A word.
  ca: Option<f64>,
  /// Whether a corrected work move has been emitted yet, so a `G10`/`G92` before any cut is a leading preamble
  /// (passed through) and one after is a mid-body frame shift (an error).
  started_motion: bool,
  /// The active work-coordinate system index (`G54`=0 … `G59`=5), updated by a `G54`…`G59` select. A mid-body
  /// `G10 L2/L20` that targets a DIFFERENT WCS than this one does not shift the frame the mesh was probed in, so it
  /// passes through rather than failing closed; only a `G10` targeting the ACTIVE WCS (or any `G92`) is a hazard.
  active_wcs: usize,
  out: Vec<String>,
}

impl<'a> Corrector<'a> {
  fn new(mesh: &'a Mesh, cfg: &'a CorrectionConfig) -> Self {
    Corrector {
      mesh,
      cfg,
      seg: mesh.grid_x.min(mesh.grid_y),
      cx: None,
      cy: None,
      pz: None,
      ca: None,
      started_motion: false,
      active_wcs: 0, // grbl's power-on active WCS is G54.
      out: Vec::new(),
    }
  }

  /// Invalidate the tracked work position (a machine-frame move landed us somewhere we cannot express in work-mm).
  fn invalidate_position(&mut self) {
    self.cx = None;
    self.cy = None;
    self.pz = None;
    self.ca = None;
  }

  /// Dispatch one typed command: correct a motion, fail-close a hazard, or pass a non-motion command through.
  fn handle_command(&mut self, command: PlannerCommand, src: &str) -> Result<(), CorrectionError> {
    match command {
      PlannerCommand::Move { rapid, axes, units, distance, feed, feed_mode, machine_coords } => {
        self.handle_move(rapid, axes, units, distance, feed, feed_mode, machine_coords, src)
      }
      PlannerCommand::Arc { cw, axes, i, j, plane, units, distance, feed, feed_mode, machine_coords, .. } => {
        self.handle_arc(cw, axes, i, j, plane, units, distance, feed, feed_mode, machine_coords, src)
      }
      PlannerCommand::Coordinate(op) => self.handle_coordinate(op, src),
      // A probe or a go-to-predefined move lands the tool at a point we cannot express in work-mm; pass the line
      // through verbatim and invalidate the tracked position so a following incremental move fails closed.
      PlannerCommand::Probe { .. } | PlannerCommand::GoToPredefined { .. } => {
        self.out.push(src.to_string());
        self.invalidate_position();
        Ok(())
      }
      // Dwell, spindle, coolant, program end/pause: no geometry to correct — pass through verbatim.
      _ => {
        self.out.push(src.to_string());
        Ok(())
      }
    }
  }

  /// A coordinate-system op. A frame shift that moves the ACTIVE work frame mid-program is a hazard (it slides the
  /// surface the mesh was probed against out from under the correction); a leading one (before any cut) passes
  /// through, defining the frame the mesh was probed in. Crucially a `G10 L2/L20` that targets a DIFFERENT WCS than
  /// the active one does NOT shift the active frame, so it is not a hazard — rejecting it was over-eager. A `G92`
  /// (dynamic offset on top of whatever WCS is active) and a `G92.1` (clearing it) always shift the active frame.
  /// A WCS select (`G54`…`G59`) updates the tracked active WCS and passes through.
  fn handle_coordinate(&mut self, op: CoordinateOp, src: &str) -> Result<(), CorrectionError> {
    let frame_shift = match op {
      // A `G10 L2/L20 P<n>` shifts the active frame only when `P<n>` IS the active WCS.
      CoordinateOp::SetWcsOffset { index, .. } | CoordinateOp::SetWcsOffsetToPosition { index, .. } => {
        index == self.active_wcs
      }
      // `G92` / `G92.1` apply on top of whatever WCS is active — always a shift of the active frame.
      CoordinateOp::SetG92ToPosition { .. } | CoordinateOp::ClearG92 => true,
      // Selecting a WCS, storing a predefined position, or a TLO apply/cancel do not move the active frame's origin.
      _ => false,
    };
    if frame_shift && self.started_motion {
      return Err(CorrectionError::FrameShiftMidProgram);
    }
    // Track the active WCS so a later `G10 L2` can be judged against it.
    if let CoordinateOp::SelectWcs { index } = op {
      self.active_wcs = index;
    }
    self.out.push(src.to_string());
    Ok(())
  }

  /// Correct one linear move (G0/G1).
  #[allow(clippy::too_many_arguments)]
  fn handle_move(
    &mut self, rapid: bool, axes: AxisWords, units: Units, distance: DistanceMode, feed: f32, feed_mode: FeedMode,
    machine_coords: bool, src: &str,
  ) -> Result<(), CorrectionError> {
    // A motion line with NO axis words (an F-only / S-only line) sets modal state but moves nothing: pass it
    // through untouched. Injecting a Z here is exactly ioSender bug #451.
    if axes.x.is_none() && axes.y.is_none() && axes.z.is_none() && axes.a.is_none() {
      self.out.push(src.to_string());
      return Ok(());
    }
    // A `G53` one-shot machine move: pass through verbatim and invalidate the tracked work position.
    if machine_coords {
      self.out.push(src.to_string());
      self.invalidate_position();
      return Ok(());
    }
    // `G93` inverse-time on a move we would subdivide breaks the per-move inverse-time F — fail closed.
    if feed_mode == FeedMode::InverseTime {
      return Err(CorrectionError::InverseTimeFeed);
    }
    let incremental = distance == DistanceMode::Incremental;
    if incremental && (self.cx.is_none() || self.cy.is_none()) {
      return Err(CorrectionError::PositionUnknown);
    }
    // The programmed absolute work Z the move goes to: a Z word sets it (an incremental Z needs the prior Z — a
    // fail-closed `PositionUnknown` if unknown); an absent Z keeps the last programmed Z (surface following).
    // `z_start` is the Z before this line, for the ramp. Resolved BEFORE the XY placement so it can gate
    // `started_motion` even when the move itself cannot be placed.
    let z_start = self.pz;
    let base_z = if axes.z.is_some() {
      match resolve(axes.z, self.pz, incremental, units) {
        Some(z) => Some(z),
        None => return Err(CorrectionError::PositionUnknown), // an incremental Z with no known prior Z.
      }
    } else {
      self.pz
    };
    // Cutting has begun ONLY when a NON-RAPID feed move commits to a known Z (a real cut) — NOT on a `G0` rapid nor
    // on a pre-first-Z re-anchor (`base_z` still `None`). This gates the leading-vs-mid-body frame-shift rule so a
    // leading `G0` opening move + `G92`/`G10` preamble is accepted while a genuine MID-CUT `G92` still fails closed.
    if !rapid && base_z.is_some() {
      self.started_motion = true;
    }
    // Resolve the absolute work-mm XY target. If a component cannot be placed (an absolute move with an absent
    // axis and no prior position), we cannot index the mesh: pass through verbatim, updating what we do know.
    let (xa, ya) = match (resolve(axes.x, self.cx, incremental, units), resolve(axes.y, self.cy, incremental, units)) {
      (Some(x), Some(y)) => (x, y),
      (rx, ry) => {
        self.out.push(src.to_string());
        if let Some(x) = rx {
          self.cx = Some(x);
        }
        if let Some(y) = ry {
          self.cy = Some(y);
        }
        return Ok(());
      }
    };
    let feed_out = to_mm(feed, units);
    // Resolve the A word to an ABSOLUTE value (rotary A is degrees — not unit-scaled). Emitting the raw incremental
    // word under our canonical `G90` output would have been wrong; resolving it here makes A absolute like X/Y/Z.
    let a_end = if axes.a.is_some() { resolve_a(axes.a, self.ca, incremental) } else { None };

    match base_z {
      // First-Z-unknown pure-XY move: emit canonically with NO Z (the surface is not yet defined — an opening
      // clearance rapid must not be pinned to a guessed surface). Not a cut; just re-anchor XY — so it does NOT set
      // `started_motion` (a leading rapid + `G92`/`G10` preamble must stay accepted, not read as mid-program).
      None => {
        let mut line = format!("G90 G21 {} X{} Y{}", motion_word(rapid), num(xa), num(ya));
        if !rapid {
          line.push_str(&format!(" F{}", num(feed_out)));
        }
        push_a(&mut line, a_end);
        self.out.push(line);
      }
      Some(bz) => {
        if rapid {
          // A rapid is Z-corrected (unless the operator opted out) but never subdivided. A rapid is positioning,
          // NOT a cut, so it does not set `started_motion` — a leading `G0` opening move before a `G92`/`G10`
          // preamble must not make that preamble read as a mid-program frame shift.
          let z = if self.cfg.correct_rapids { bz + self.mesh.interpolate(xa, ya) } else { bz };
          self.out.push(self.linear_line(true, xa, ya, z, None, a_end));
        } else if let (Some(sx), Some(sy), Some(sz)) = (self.cx, self.cy, z_start) {
          // A G1 feed move from a fully-known start: subdivide on the planar distance, ramp the base Z linearly,
          // and add the mesh at each sub-endpoint. The last sub-endpoint lands exactly on the commanded target.
          let n = subdivision_count(planar_distance((sx, sy), (xa, ya)), self.seg);
          for k in 1..=n {
            let f = k as f64 / n as f64;
            let px = sx + (xa - sx) * f;
            let py = sy + (ya - sy) * f;
            let pz = sz + (bz - sz) * f + self.mesh.interpolate(px, py);
            // Interpolate A ACROSS the sub-moves so a rotary axis sweeps coordinately with XY, rather than jumping
            // to its target on the last sub-move (the mis-sequencing bug). When the start A is unknown (this move
            // establishes A), A is placed only on the final sub-endpoint since there is nothing to interpolate from.
            let a = match (self.ca, a_end) {
              (Some(a0), Some(a1)) => Some(a0 + (a1 - a0) * f),
              (None, Some(a1)) => if k == n { Some(a1) } else { None },
              _ => None,
            };
            self.out.push(self.linear_line(false, px, py, pz, Some(feed_out), a));
          }
        } else {
          // The start is not fully known (this move establishes the cut start): a single corrected G1 cut.
          let z = bz + self.mesh.interpolate(xa, ya);
          self.out.push(self.linear_line(false, xa, ya, z, Some(feed_out), a_end));
        }
      }
    }
    self.cx = Some(xa);
    self.cy = Some(ya);
    if base_z.is_some() {
      self.pz = base_z;
    }
    if a_end.is_some() {
      self.ca = a_end;
    }
    Ok(())
  }

  /// Correct one arc (G2/G3), keeping it a real arc: split into sub-arcs with I/J recomputed per sub-arc, Z ramped
  /// along the helix plus the mesh at each sub-endpoint.
  #[allow(clippy::too_many_arguments)]
  fn handle_arc(
    &mut self, cw: bool, axes: AxisWords, i: Option<f32>, j: Option<f32>, plane: Plane, units: Units,
    distance: DistanceMode, feed: f32, feed_mode: FeedMode, machine_coords: bool, src: &str,
  ) -> Result<(), CorrectionError> {
    if machine_coords {
      self.out.push(src.to_string());
      self.invalidate_position();
      return Ok(());
    }
    if plane != Plane::XY {
      return Err(CorrectionError::NonXyArc);
    }
    if feed_mode == FeedMode::InverseTime {
      return Err(CorrectionError::InverseTimeFeed);
    }
    // An arc with no I/J centre offset is an R-form (or malformed) arc; the correction needs the IJK geometry.
    if i.is_none() && j.is_none() {
      return Err(CorrectionError::ArcRadiusForm);
    }
    // An arc needs a known start (its centre is defined relative to it).
    let (sx, sy) = match (self.cx, self.cy) {
      (Some(x), Some(y)) => (x, y),
      _ => return Err(CorrectionError::PositionUnknown),
    };
    let incremental = distance == DistanceMode::Incremental;
    // The endpoint (absent axis retains the start), and the centre from the I/J offsets (always relative to start).
    let ex = resolve(axes.x, Some(sx), incremental, units).unwrap_or(sx);
    let ey = resolve(axes.y, Some(sy), incremental, units).unwrap_or(sy);
    let center = (sx + to_mm(i.unwrap_or(0.0), units), sy + to_mm(j.unwrap_or(0.0), units));
    // The base Z ramp across the helix. Resolve the arc's programmed end Z; an INCREMENTAL Z with no known prior Z
    // must fail CLOSED (`PositionUnknown`), exactly like the linear path — NOT silently emit a Z-less (flat) arc at
    // an unknown machine Z. An ABSOLUTE Z always resolves, so the `(None, Some)` "arc establishes the first Z" case
    // still emits a (flat) Z rather than dropping it.
    let z_start = self.pz;
    let z_end = if axes.z.is_some() {
      match resolve(axes.z, self.pz, incremental, units) {
        Some(z) => Some(z),
        None => return Err(CorrectionError::PositionUnknown),
      }
    } else {
      self.pz
    };
    let span = ArcSpan::from_endpoints((sx, sy), (ex, ey), center, cw);
    let n = subdivision_count(span.length(), self.seg);
    let feed_out = to_mm(feed, units);
    let word = if cw { "G2" } else { "G3" };
    // An arc is a feed cut, so it marks that cutting has begun — unless it is a pre-first-Z arc (no Z established),
    // a re-anchor case treated like the linear one so a leading preamble stays accepted.
    if z_end.is_some() {
      self.started_motion = true;
    }

    let mut prev = (sx, sy);
    for k in 1..=n {
      let f = k as f64 / n as f64;
      // The last sub-endpoint lands exactly on the commanded endpoint (no float drift into the next move).
      let pt = if k == n { (ex, ey) } else { span.point_at(f) };
      // I/J are the vector from THIS sub-arc's start to the (unchanged) true centre — recomputed per sub-arc.
      let (sub_i, sub_j) = (center.0 - prev.0, center.1 - prev.1);
      let mut line = format!("G90 G21 G17 {word} X{} Y{}", num(pt.0), num(pt.1));
      // Z: ramp from start to end across the helix when both are known; if only the END Z is known (this arc
      // carries a Z word but no prior Z was established), emit it FLAT at the end value (we cannot ramp from an
      // unknown start) — crucially we must NOT drop the Z, or a helical cut would silently flatten (fail-open).
      // Only a pure-XY arc with no Z at all (both `None`) emits no Z word. The mesh correction is added regardless.
      match (z_start, z_end) {
        (Some(zs), Some(ze)) => {
          let pz = zs + (ze - zs) * f + self.mesh.interpolate(pt.0, pt.1);
          line.push_str(&format!(" Z{}", num(pz)));
        }
        (None, Some(ze)) => {
          let pz = ze + self.mesh.interpolate(pt.0, pt.1);
          line.push_str(&format!(" Z{}", num(pz)));
        }
        _ => {}
      }
      line.push_str(&format!(" I{} J{} F{}", num(sub_i), num(sub_j), num(feed_out)));
      self.out.push(line);
      prev = pt;
    }
    self.cx = Some(ex);
    self.cy = Some(ey);
    if z_end.is_some() {
      self.pz = z_end;
    }
    Ok(())
  }

  /// Build one canonical linear move line (`G90 G0/G1 X Y Z [F] [A]`).
  fn linear_line(&self, rapid: bool, x: f64, y: f64, z: f64, feed: Option<f64>, a: Option<f64>) -> String {
    let mut line = format!("G90 G21 {} X{} Y{} Z{}", motion_word(rapid), num(x), num(y), num(z));
    if let Some(f) = feed {
      line.push_str(&format!(" F{}", num(f)));
    }
    push_a(&mut line, a);
    line
  }
}

/// Resolve one axis word to an absolute work-mm value: an absent word retains `current`; a present word is the
/// absolute value (converted to mm) under G90, or `current + word` under G91. `None` when the value cannot be
/// determined (an incremental word with no known current).
fn resolve(word: Option<f32>, current: Option<f64>, incremental: bool, units: Units) -> Option<f64> {
  match word {
    None => current,
    Some(w) => {
      let v = to_mm(w, units);
      if incremental { current.map(|c| c + v) } else { Some(v) }
    }
  }
}

/// Resolve the A (rotary) word to an absolute value: like [`resolve`] but with NO unit conversion (rotary A is
/// degrees, not scaled by G20/G21). `None` when an incremental A word has no known current A.
fn resolve_a(word: Option<f32>, current: Option<f64>, incremental: bool) -> Option<f64> {
  match word {
    None => current,
    Some(w) => {
      let v = f64::from(w);
      if incremental { current.map(|c| c + v) } else { Some(v) }
    }
  }
}

/// Convert a value from the active `units` to millimetres (inch → ×25.4).
fn to_mm(v: f32, units: Units) -> f64 {
  let v = f64::from(v);
  if units == Units::Inch { v * 25.4 } else { v }
}

/// `G0` for a rapid, `G1` for a feed move.
fn motion_word(rapid: bool) -> &'static str {
  if rapid { "G0" } else { "G1" }
}

/// Append a raw `A` word (degrees / linear per the firmware's rotary handling — never mesh-shifted) if present.
fn push_a(line: &mut String, a: Option<f64>) {
  if let Some(a) = a {
    line.push_str(&format!(" A{}", num(a)));
  }
}

/// Format a coordinate/feed at grbl's 3-decimal precision.
fn num(v: f64) -> String {
  format!("{v:.3}")
}

#[cfg(test)]
mod tests {
  use super::*;

  /// A flat (all-zero) mesh over a generous box with 5 mm spacing — the identity correction.
  fn flat_mesh() -> Mesh {
    Mesh::from_spacing((0.0, 0.0), (100.0, 100.0), (5.0, 5.0))
  }

  /// A mesh with every node offset by a uniform `delta` (a constant Z shift everywhere inside the grid).
  fn uniform_mesh(delta: f64) -> Mesh {
    let mut m = flat_mesh();
    for iy in 0..m.ny {
      for ix in 0..m.nx {
        m.set_delta(ix, iy, delta);
      }
    }
    m
  }

  /// Correct a program and return the output lines (panicking on a hazard — tests that expect success use this).
  fn correct(program: &[&str], mesh: &Mesh, cfg: &CorrectionConfig) -> Vec<String> {
    let prog: Vec<String> = program.iter().map(|s| s.to_string()).collect();
    correct_program(&prog, mesh, cfg).expect("the program corrects without a hazard")
  }

  /// The Z value (mm) parsed out of a corrected line, or `None` if it carries no Z word.
  fn z_of(line: &str) -> Option<f64> {
    line.split_whitespace().find_map(|w| w.strip_prefix('Z').and_then(|v| v.parse::<f64>().ok()))
  }

  fn xy_of(line: &str) -> (Option<f64>, Option<f64>) {
    let x = line.split_whitespace().find_map(|w| w.strip_prefix('X').and_then(|v| v.parse::<f64>().ok()));
    let y = line.split_whitespace().find_map(|w| w.strip_prefix('Y').and_then(|v| v.parse::<f64>().ok()));
    (x, y)
  }

  fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6
  }

  /// A looser comparison for geometric quantities recovered by PARSING a 3-decimal-formatted coordinate back out
  /// of an emitted line: each coordinate carries up to ~5e-4 of rounding, so a derived radius can drift ~1e-3.
  fn close_coarse(a: f64, b: f64) -> bool {
    (a - b).abs() < 2e-3
  }

  #[test]
  fn the_output_opens_with_a_canonical_header() {
    let out = correct(&["G0 X0 Y0 Z5"], &flat_mesh(), &CorrectionConfig::default());
    assert_eq!(out[0], "G90 G21 G94", "the correction must canonicalize the modal frame up front");
  }

  #[test]
  fn a_flat_mesh_is_the_identity_on_the_z_profile() {
    // Establish position, plunge, then cut: over a flat (all-zero) mesh every corrected Z equals the programmed Z.
    let out = correct(
      &["G0 X0 Y0 Z5", "G1 Z-1 F100", "G1 X20 Y0 F100"],
      &flat_mesh(),
      &CorrectionConfig::default(),
    );
    // Every emitted Z must be either 5 (the rapid) or -1 (the plunge + the cut), never shifted.
    for line in out.iter().filter(|l| l.contains('Z')) {
      let z = z_of(line).expect("a Z word");
      assert!(close(z, 5.0) || close(z, -1.0), "a flat mesh must not shift Z; saw {line:?}");
    }
  }

  #[test]
  fn a_uniform_mesh_shifts_every_corrected_z_by_the_constant() {
    // A uniform +0.5 mesh shifts the cut Z by exactly 0.5 everywhere inside the grid.
    let out = correct(
      &["G0 X0 Y0 Z5", "G1 Z-1 F100", "G1 X20 Y0 F100"],
      &uniform_mesh(0.5),
      &CorrectionConfig::default(),
    );
    let cut_lines: Vec<&String> = out.iter().filter(|l| l.contains("G1") && l.contains('Z')).collect();
    assert!(!cut_lines.is_empty());
    for line in cut_lines {
      // The plunge target -1 and the cut all become -0.5 (−1 + 0.5). (The rapid is a G0, excluded here.)
      assert!(close(z_of(line).unwrap(), -0.5), "a uniform mesh must shift the cut Z by the constant; saw {line:?}");
    }
  }

  #[test]
  fn a_tilted_mesh_ramps_the_correction_linearly_along_x() {
    // A mesh that ramps +0.1 mm per column (2.5 mm spacing → +0.04 mm/mm in X, 0 in Y). At X the delta is
    // 0.04·X, so a cut at Z-1 lands at -1 + 0.04·X. Check a node and a mid-cell point.
    let mut mesh = flat_mesh(); // 21×21 over [0,100], grid 5.
    for iy in 0..mesh.ny {
      for ix in 0..mesh.nx {
        mesh.set_delta(ix, iy, 0.02 * (ix as f64) * 5.0); // delta = 0.02·x at node x = ix·5.
      }
    }
    // delta(x) = 0.02·x. A single cut across X so we can read intermediate sub-endpoints.
    let out = correct(&["G0 X0 Y0 Z5", "G1 Z-1 F100", "G1 X10 Y0 F100"], &mesh, &CorrectionConfig::default());
    for line in out.iter().filter(|l| l.contains("G1") && l.contains('Z')) {
      let (x, _) = xy_of(line);
      let x = x.unwrap();
      let expected = -1.0 + 0.02 * x;
      assert!(close(z_of(line).unwrap(), expected), "tilt must ramp Z linearly; at X{x} saw {line:?}");
    }
  }

  #[test]
  fn a_g1_feed_move_is_subdivided_by_ceil_len_over_seg() {
    // seg = min(grid) = 5. A 20 mm cut → ceil(20/5) = 4 sub-moves.
    let out = correct(&["G0 X0 Y0 Z-1", "G1 X20 Y0 F100"], &flat_mesh(), &CorrectionConfig::default());
    let g1_cuts = out.iter().filter(|l| l.starts_with("G90 G21 G1") && l.contains("X")).count();
    assert_eq!(g1_cuts, 4, "a 20 mm cut at seg 5 must split into 4 sub-moves; got {out:?}");
    // The final sub-move lands exactly on the commanded endpoint.
    let last = out.iter().rev().find(|l| l.starts_with("G90 G21 G1")).unwrap();
    assert_eq!(xy_of(last), (Some(20.0), Some(0.0)));
  }

  #[test]
  fn a_g0_rapid_is_z_corrected_but_not_subdivided() {
    let out = correct(&["G0 X0 Y0 Z-1", "G0 X20 Y0"], &uniform_mesh(0.5), &CorrectionConfig::default());
    let rapids: Vec<&String> = out.iter().filter(|l| l.starts_with("G90 G21 G0")).collect();
    // Two rapids total, each a single line (not subdivided), and the travel rapid's Z is corrected (-1 + 0.5).
    let travel = rapids.iter().find(|l| xy_of(l).0 == Some(20.0)).expect("the travel rapid");
    assert!(close(z_of(travel).unwrap(), -0.5), "a rapid must be Z-corrected; saw {travel:?}");
    assert_eq!(rapids.iter().filter(|l| xy_of(l).0 == Some(20.0)).count(), 1, "a rapid must not be subdivided");
  }

  #[test]
  fn correct_rapids_off_leaves_the_rapid_z_uncorrected() {
    let cfg = CorrectionConfig { correct_rapids: false };
    let out = correct(&["G0 X0 Y0 Z-1", "G0 X20 Y0"], &uniform_mesh(0.5), &cfg);
    let travel = out.iter().find(|l| l.starts_with("G90 G21 G0") && xy_of(l).0 == Some(20.0)).unwrap();
    assert!(close(z_of(travel).unwrap(), -1.0), "with correct_rapids off the rapid Z stays programmed; saw {travel:?}");
  }

  #[test]
  fn a_pure_xy_move_gets_the_last_z_plus_mesh_injected() {
    // After a plunge to -1, a pure-XY cut (no Z word) must follow the surface: Z = -1 + mesh(x,y).
    let out = correct(&["G0 X0 Y0 Z5", "G1 Z-1 F100", "G1 X20 Y0 F100"], &uniform_mesh(0.3), &CorrectionConfig::default());
    // The pure-XY cut sub-moves all carry an injected Z of -1 + 0.3 = -0.7.
    let cut_moves: Vec<&String> = out.iter().filter(|l| l.starts_with("G90 G21 G1") && xy_of(l).0.unwrap_or(0.0) > 0.0).collect();
    assert!(!cut_moves.is_empty(), "the pure-XY cut must be emitted");
    for line in cut_moves {
      assert!(close(z_of(line).unwrap(), -0.7), "a pure-XY cut must inject last_z + mesh; saw {line:?}");
    }
  }

  #[test]
  fn a_feed_only_move_with_no_axis_words_passes_through_untouched() {
    // ioSender bug #451: a G1 line with only an F word moves nothing and must NOT get a Z injected.
    let out = correct(&["G0 X0 Y0 Z-1", "G1 F250"], &uniform_mesh(0.5), &CorrectionConfig::default());
    assert!(out.iter().any(|l| l == "G1 F250"), "an F-only line must pass through verbatim; got {out:?}");
    assert!(!out.iter().any(|l| l == "G1 F250" && l.contains('Z')), "an F-only line must not get a Z");
  }

  #[test]
  fn a_non_motion_line_passes_through_verbatim() {
    let out = correct(&["(a comment)", "M3 S1000", "G0 X0 Y0 Z-1", "G4 P0.5", "M5"], &flat_mesh(), &CorrectionConfig::default());
    for token in ["(a comment)", "M3 S1000", "G4 P0.5", "M5"] {
      assert!(out.iter().any(|l| l == token), "{token:?} must pass through verbatim; got {out:?}");
    }
  }

  #[test]
  fn retract_and_re_plunge_at_the_same_xy_preserve_the_relative_z_distances() {
    // A plunge to -1, retract to +2, re-plunge to -1, all at the same XY: the corrected Zs shift by the SAME mesh
    // offset, so the relative distances (the 3 mm retract, the 3 mm plunge) are preserved exactly.
    let out = correct(
      &["G0 X10 Y10 Z5", "G1 Z-1 F100", "G0 Z2", "G1 Z-1 F100"],
      &uniform_mesh(0.4),
      &CorrectionConfig::default(),
    );
    let zs: Vec<f64> = out.iter().filter(|l| l.starts_with("G90 G")).filter_map(|l| z_of(l)).collect();
    // rapid to 5→5.4, plunge -1→-0.6, retract 2→2.4, plunge -1→-0.6. Distances: 5.4→-0.6 = 6.0; -0.6→2.4 = 3.0; etc.
    assert!(close(zs[1], -0.6) && close(zs[2], 2.4) && close(zs[3], -0.6), "same-XY offset must be constant; got {zs:?}");
    assert!(close(zs[2] - zs[1], 3.0), "the 3 mm retract distance must be preserved");
    assert!(close(zs[2] - zs[3], 3.0), "the 3 mm re-plunge distance must be preserved");
  }

  #[test]
  fn incremental_and_absolute_programs_correct_to_the_same_absolute_output() {
    // The G91 blind-spot fix: an incremental program (after establishing an absolute start) must correct to the
    // SAME absolute-mm output as its G90 equivalent.
    let mesh = uniform_mesh(0.25);
    let abs = correct(&["G0 X0 Y0 Z-1", "G1 X10 Y0 F100", "G1 X10 Y10 F100"], &mesh, &CorrectionConfig::default());
    let inc = correct(&["G0 X0 Y0 Z-1", "G91", "G1 X10 F100", "G1 Y10 F100"], &mesh, &CorrectionConfig::default());
    // Compare only the emitted corrected motion lines (the incremental version has an extra passed-through `G91`).
    let motions = |v: &[String]| -> Vec<String> { v.iter().filter(|l| l.starts_with("G90 G21 G1")).cloned().collect() };
    assert_eq!(motions(&abs), motions(&inc), "incremental input must normalize to the same absolute output");
  }

  #[test]
  fn inch_and_millimeter_programs_correct_to_the_same_millimeter_output() {
    // A G20 (inch) program must resolve to the same mm output as its G21 equivalent. 0.5 in = 12.7 mm.
    let mesh = uniform_mesh(0.1);
    let mm = correct(&["G0 X0 Y0 Z-1", "G1 X12.7 Y0 F100"], &mesh, &CorrectionConfig::default());
    let inch = correct(&["G20", "G0 X0 Y0 Z-0.03937007874", "G1 X0.5 Y0 F100"], &mesh, &CorrectionConfig::default());
    let last_mm = mm.iter().rev().find(|l| l.starts_with("G90 G21 G1")).unwrap();
    let last_in = inch.iter().rev().find(|l| l.starts_with("G90 G21 G1")).unwrap();
    assert!(close(xy_of(last_mm).0.unwrap(), 12.7) && close(xy_of(last_in).0.unwrap(), 12.7),
      "inch input must convert to mm; got {last_in:?}");
  }

  #[test]
  fn every_corrected_coordinate_line_re_asserts_g90_g21_so_a_stray_g20_cannot_govern() {
    // HARDWARE-SAFETY REGRESSION: an inch (`G20`) program passes its `G20` through verbatim; if a corrected motion
    // line re-asserted only `G90` (not `G21`), the mm-valued coordinate that follows would be read as INCHES →
    // 25.4× scale → crash. So EVERY line that carries a coordinate must re-assert `G90 G21` before the coordinate,
    // leaving no window where the surviving `G20` could still govern.
    let out = correct(&["G20", "G0 X0 Y0 Z-0.03937", "G1 X0.5 Y0 F100"], &flat_mesh(), &CorrectionConfig::default());
    for line in &out {
      // Any emitted line that carries a coordinate word must lead with the canonical absolute-mm frame.
      if line.contains('X') || line.contains('Y') || line.contains('Z') {
        assert!(
          line.starts_with("G90 G21"),
          "a corrected coordinate line must re-assert G90 G21 (a stray G20 must not govern); saw {line:?}",
        );
      }
    }
    // And the value is the mm conversion, not the raw inch number (0.5 in → 12.7 mm).
    let last = out.iter().rev().find(|l| l.starts_with("G90 G21 G1")).unwrap();
    assert!(close(xy_of(last).0.unwrap(), 12.7), "the inch coordinate must be converted to mm; got {last:?}");
  }

  #[test]
  fn a_leading_coordinate_preamble_passes_through_but_a_mid_body_one_fails_closed() {
    // A leading G10 (before any cut) defines the frame the mesh was probed in → passes through.
    let out = correct(&["G10 L2 P1 X0 Y0", "G0 X0 Y0 Z-1"], &flat_mesh(), &CorrectionConfig::default());
    assert!(out.iter().any(|l| l == "G10 L2 P1 X0 Y0"), "a leading G10 must pass through; got {out:?}");
    // A mid-body G10 targeting the ACTIVE WCS (P1 = G54, the default) is a hazard.
    let prog: Vec<String> = ["G0 X0 Y0 Z-1", "G1 X10 F100", "G10 L2 P1 X5 Y5"].iter().map(|s| s.to_string()).collect();
    assert_eq!(correct_program(&prog, &flat_mesh(), &CorrectionConfig::default()), Err(CorrectionError::FrameShiftMidProgram));
  }

  #[test]
  fn a_mid_body_g10_targeting_a_non_active_wcs_passes_through_not_fails_closed() {
    // The over-eager-rejection fix: a `G10 L2 P2` (sets the G55 offset) mid-body while the job runs under G54 does
    // NOT move the active frame the mesh was probed in, so it must PASS THROUGH rather than fail closed.
    let out = correct(&["G0 X0 Y0 Z-1", "G1 X10 F100", "G10 L2 P2 X5 Y5"], &flat_mesh(), &CorrectionConfig::default());
    assert!(out.iter().any(|l| l == "G10 L2 P2 X5 Y5"), "a G10 targeting a non-active WCS must pass through; got {out:?}");
    // But once the job SELECTS G55, a `G10 L2 P2` targeting that now-active WCS IS a mid-body frame shift.
    let prog: Vec<String> =
      ["G0 X0 Y0 Z-1", "G1 X10 F100", "G55", "G1 X20 F100", "G10 L2 P2 X5 Y5"].iter().map(|s| s.to_string()).collect();
    assert_eq!(correct_program(&prog, &flat_mesh(), &CorrectionConfig::default()), Err(CorrectionError::FrameShiftMidProgram));
    // A mid-body `G92` shifts the active frame regardless of WCS — always a hazard.
    let g92: Vec<String> = ["G0 X0 Y0 Z-1", "G1 X10 F100", "G92 X0 Y0"].iter().map(|s| s.to_string()).collect();
    assert_eq!(correct_program(&g92, &flat_mesh(), &CorrectionConfig::default()), Err(CorrectionError::FrameShiftMidProgram));
  }

  #[test]
  fn a_leading_rapid_then_a_frame_shift_preamble_is_accepted_but_a_mid_cut_one_fails_closed() {
    // The `started_motion` fix: an opening `G0` rapid is POSITIONING, not a cut, so it must not make a following
    // `G92`/`G10` preamble read as mid-program. A leading `G0 Z5` + `G92` preamble corrects successfully.
    let out = correct(&["G0 Z5", "G92 X0 Y0 Z0", "G1 X10 Z-1 F100"], &flat_mesh(), &CorrectionConfig::default());
    assert!(out.iter().any(|l| l == "G92 X0 Y0 Z0"), "a leading G0 + G92 preamble must be accepted; got {out:?}");
    // But a `G92` AFTER a real G1 cut is a genuine mid-program frame shift — still fails closed.
    let mid: Vec<String> = ["G1 X10 Z-1 F100", "G92 X0", "G1 X20"].iter().map(|s| s.to_string()).collect();
    assert_eq!(correct_program(&mid, &flat_mesh(), &CorrectionConfig::default()), Err(CorrectionError::FrameShiftMidProgram));
  }

  #[test]
  fn an_incremental_arc_z_with_no_prior_z_fails_closed_like_the_linear_path() {
    // Arc Z-drop fail-OPEN fix: an INCREMENTAL Z on an arc with no known prior Z cannot be resolved. The linear
    // path fails closed here (PositionUnknown); the arc path must MATCH, not silently emit a Z-less (flat) arc.
    let prog: Vec<String> =
      ["G0 X10 Y0", "G91", "G3 X0 Y10 Z-1 I-10 J0 F100"].iter().map(|s| s.to_string()).collect();
    assert_eq!(correct_program(&prog, &flat_mesh(), &CorrectionConfig::default()), Err(CorrectionError::PositionUnknown));
  }

  #[test]
  fn an_arc_with_a_z_word_but_no_prior_z_keeps_its_z_rather_than_dropping_it() {
    // Fail-open regression: a helical arc whose Z establishes the first Z (no prior Z to ramp from) must still emit
    // a Z word (flat at the end value + mesh) — dropping it would silently flatten the cut. Uniform +0.5 mesh → the
    // arc's Z is -1 + 0.5 = -0.5 at every sub-arc endpoint.
    let out = correct(
      &["G0 X10 Y0", "G3 X0 Y10 Z-1 I-10 J0 F100"],
      &uniform_mesh(0.5),
      &CorrectionConfig::default(),
    );
    let arcs: Vec<&String> = out.iter().filter(|l| l.contains("G3")).collect();
    assert!(!arcs.is_empty(), "the arc must be emitted");
    for line in &arcs {
      let z = z_of(line).unwrap_or_else(|| panic!("the arc must keep a Z word (not drop it); saw {line:?}"));
      assert!(close(z, -0.5), "the arc Z is the end value + mesh; saw {line:?}");
    }
  }

  #[test]
  fn a_subdivided_move_interpolates_the_a_axis_coordinately_across_the_sub_moves() {
    // A-word mis-sequencing fix: a rotary A must sweep WITH the subdivided XY, not sit still then jump on the last
    // sub-move. Establish A0, then a 20 mm cut to A10 at seg 5 → 4 sub-moves with A = 2.5, 5.0, 7.5, 10.0.
    let out = correct(&["G0 X0 Y0 Z-1 A0", "G1 X20 Y0 A10 F100"], &flat_mesh(), &CorrectionConfig::default());
    let a_of = |l: &str| l.split_whitespace().find_map(|w| w.strip_prefix('A').and_then(|v| v.parse::<f64>().ok()));
    let cut_as: Vec<f64> = out.iter().filter(|l| l.starts_with("G90 G21 G1")).filter_map(|l| a_of(l)).collect();
    assert_eq!(cut_as.len(), 4, "every sub-move must carry an interpolated A word; got {out:?}");
    assert!(
      close(cut_as[0], 2.5) && close(cut_as[1], 5.0) && close(cut_as[2], 7.5) && close(cut_as[3], 10.0),
      "A must interpolate coordinately (2.5/5/7.5/10), not jump on the last sub-move; got {cut_as:?}",
    );
  }

  #[test]
  fn g93_inverse_time_feed_fails_closed() {
    let prog: Vec<String> = ["G0 X0 Y0 Z-1", "G93", "G1 X10 F0.5"].iter().map(|s| s.to_string()).collect();
    assert_eq!(correct_program(&prog, &flat_mesh(), &CorrectionConfig::default()), Err(CorrectionError::InverseTimeFeed));
  }

  #[test]
  fn a_machine_move_then_an_incremental_move_is_position_unknown() {
    // A `G53` move invalidates the tracked work position; a following incremental move cannot be placed.
    let prog: Vec<String> = ["G0 X0 Y0 Z-1", "G53 G0 Z0", "G91", "G1 X10 F100"].iter().map(|s| s.to_string()).collect();
    assert_eq!(correct_program(&prog, &flat_mesh(), &CorrectionConfig::default()), Err(CorrectionError::PositionUnknown));
  }

  #[test]
  fn a_non_xy_plane_arc_fails_closed() {
    let prog: Vec<String> = ["G0 X0 Y0 Z-1", "G18", "G2 X10 Z-1 I5 K0 F100"].iter().map(|s| s.to_string()).collect();
    assert_eq!(correct_program(&prog, &flat_mesh(), &CorrectionConfig::default()), Err(CorrectionError::NonXyArc));
  }

  #[test]
  fn an_arc_stays_an_arc_split_into_sub_arcs_with_recomputed_ijk_and_a_z_ramp() {
    // A CCW quarter arc from (10,0) to (0,10) about the origin (radius 10, length ≈ 15.7). seg = 5 → ceil(15.7/5) = 4
    // sub-arcs. Each stays a G3, carries a Z, and its recomputed I/J point at the true centre (|I,J| = radius 10).
    let out = correct(
      &["G0 X10 Y0 Z5", "G1 Z-1 F100", "G3 X0 Y10 I-10 J0 F100"],
      &uniform_mesh(0.2),
      &CorrectionConfig::default(),
    );
    let arcs: Vec<&String> = out.iter().filter(|l| l.starts_with("G90 G21 G17 G3")).collect();
    assert_eq!(arcs.len(), 4, "the quarter arc must split into 4 sub-arcs; got {out:?}");
    for line in &arcs {
      let i = line.split_whitespace().find_map(|w| w.strip_prefix('I').and_then(|v| v.parse::<f64>().ok())).unwrap();
      let j = line.split_whitespace().find_map(|w| w.strip_prefix('J').and_then(|v| v.parse::<f64>().ok())).unwrap();
      assert!(close_coarse((i * i + j * j).sqrt(), 10.0), "each sub-arc's I/J must point at the centre (radius 10); saw {line:?}");
      // The endpoint lies on the circle of radius 10, and the Z is the ramped base (-1) plus the uniform mesh (0.2).
      let (x, y) = xy_of(line);
      assert!(close_coarse((x.unwrap() * x.unwrap() + y.unwrap() * y.unwrap()).sqrt(), 10.0), "the endpoint stays on the arc");
      assert!(close(z_of(line).unwrap(), -0.8), "the helix Z is the ramp + mesh; saw {line:?}");
    }
    // The final sub-arc lands exactly on the commanded endpoint.
    assert_eq!(xy_of(arcs[3]), (Some(0.0), Some(10.0)));
  }

  #[test]
  fn a_clockwise_arc_sweeps_the_opposite_way_from_a_counter_clockwise_one() {
    // G2 (cw) and G3 (ccw) between the same endpoints trace opposite intermediate points. From (10,0) to (-10,0)
    // about the origin: CCW passes over the top (+Y), CW under the bottom (−Y).
    let ccw = correct(&["G0 X10 Y0 Z-1", "G3 X-10 Y0 I-10 J0 F100"], &flat_mesh(), &CorrectionConfig::default());
    let cw = correct(&["G0 X10 Y0 Z-1", "G2 X-10 Y0 I-10 J0 F100"], &flat_mesh(), &CorrectionConfig::default());
    let mid_y = |out: &[String], word: &str| -> f64 {
      let arcs: Vec<&String> = out.iter().filter(|l| l.contains(word)).collect();
      // The midpoint sub-arc endpoint's Y sign distinguishes the sweep direction.
      xy_of(arcs[arcs.len() / 2 - 1]).1.unwrap()
    };
    assert!(mid_y(&ccw, "G3") > 0.0, "a CCW arc must pass over the top (+Y); got {ccw:?}");
    assert!(mid_y(&cw, "G2") < 0.0, "a CW arc must pass under the bottom (−Y); got {cw:?}");
  }

  #[test]
  fn an_out_of_grid_move_lifts_by_the_mesh_max_height() {
    // A cut that runs OUTSIDE the probed grid must lift by max_height (the fail-safe), never plunge into unprobed
    // stock. A small grid [0,10] with a +0.5 peak; a cut to X50 (outside) lifts by 0.5.
    let mut mesh = Mesh::from_spacing((0.0, 0.0), (10.0, 10.0), (5.0, 5.0));
    mesh.set_delta(1, 1, 0.5); // a peak inside the grid → max_height 0.5.
    let out = correct(&["G0 X0 Y0 Z-1", "G1 X50 Y0 F100"], &mesh, &CorrectionConfig::default());
    let last = out.iter().rev().find(|l| l.starts_with("G90 G21 G1")).unwrap();
    // The final endpoint X50 is well outside the grid → mesh returns max_height 0.5 → Z = -1 + 0.5 = -0.5.
    assert!(close(z_of(last).unwrap(), -0.5), "an out-of-grid endpoint must lift by max_height; saw {last:?}");
  }
}
