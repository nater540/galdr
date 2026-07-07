//! The datum-finder wizard: an ordered, pure state machine for single-edge and corner work-zero probing.
//!
//! This is the non-rotary analogue of [`crate::app::rotary_center::WizardState`] — an explicit step enum whose
//! ordering makes an illegal read unrepresentable (you cannot compute a corner before both faces are captured).
//! Two shapes share the type:
//!
//! - **Single edge:** `EnterParams → ProbeEdge → Review`, then a one-axis `G10 L2 P0` write. The operator picks
//!   the axis and the approach direction; the compensated edge is `contact + (Ø/2)·af` (X/Y), or the raw contact
//!   for a Z surface touch-off (Z is never compensated).
//! - **Corner (in + out, all 4):** `EnterParams → ProbeFaceX → ReadyFaceY → ProbeFaceY → Review`, then an
//!   X&Y `G10 L2 P0` write. A [`Corner`] fixes both approach signs and the comp signs — ioSender's A/B/C/D map,
//!   `A=(+X,+Y) … D=(+X,−Y)` for an OUTSIDE corner, with an inside (pocket) corner inverting both (the probe
//!   approaches from the opposite side). Both faces are touched at one Z plunge; the operator jogs to each face's
//!   approach between touches, exactly as the rotary wizard has the operator jog to each side.
//!
//! **Failure is total:** a `success:false` (`G38.3`'s software miss) or an intervening alarm surfaces as
//! [`ProbeOutcome::Failure`], which drives the wizard to [`DatumStep::Aborted`] with a reason — never a partial
//! compute off a bad reading, and never an offset written from half a corner.
//!
//! Pure — no egui, no I/O. The shell drives it (sends each touch's [`super::touch::touch_lines`], feeds back each
//! resolved [`ProbeOutcome`]) and the view renders [`DatumState`]. Credit: ioSender / OpenCNCPilot (MIT).

use super::super::intent::{Axis, Dir, machine_offset_line};
use super::super::view_state::ProbeOutcome;
use super::comp::edge_coord;
use super::touch::Touch;

/// Which of the four rectangular corners, inside or outside, a corner datum targets. `af_x`/`af_y` are the
/// OUTSIDE approach signs (ioSender's A/B/C/D map); an `inside` corner inverts both, because inside a pocket the
/// probe approaches each wall from the opposite side. The effective approach (and tip-comp) sign per axis comes
/// from [`Self::approach_x`] / [`Self::approach_y`], so callers never re-derive the inside flip.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Corner {
  /// The OUTSIDE approach sign along X (`+1.0` toward +X, `−1.0` toward −X).
  pub af_x: f64,
  /// The OUTSIDE approach sign along Y.
  pub af_y: f64,
  /// Whether this is an inside (pocket) corner, which inverts both approach signs.
  pub inside: bool,
}

impl Corner {
  /// ioSender corner **A** — the `(+X, +Y)` outside corner (front-left of a part in the +X/+Y quadrant).
  pub const A: Corner = Corner { af_x: 1.0, af_y: 1.0, inside: false };
  /// ioSender corner **B** — the `(−X, +Y)` outside corner.
  pub const B: Corner = Corner { af_x: -1.0, af_y: 1.0, inside: false };
  /// ioSender corner **C** — the `(−X, −Y)` outside corner.
  pub const C: Corner = Corner { af_x: -1.0, af_y: -1.0, inside: false };
  /// ioSender corner **D** — the `(+X, −Y)` outside corner.
  pub const D: Corner = Corner { af_x: 1.0, af_y: -1.0, inside: false };

  /// This corner as an INSIDE (pocket) corner — the same location, approached from inside, both signs inverted.
  pub fn inside(self) -> Corner {
    Corner { inside: true, ..self }
  }

  /// The effective approach/comp sign along X: the outside sign, inverted when this is an inside corner.
  pub fn approach_x(self) -> f64 {
    if self.inside { -self.af_x } else { self.af_x }
  }

  /// The effective approach/comp sign along Y.
  pub fn approach_y(self) -> f64 {
    if self.inside { -self.af_y } else { self.af_y }
  }

  /// The X-face touch: probe X in the effective approach direction.
  fn touch_x(self) -> Touch {
    Touch { axis: Axis::X, dir: dir_of(self.approach_x()) }
  }

  /// The Y-face touch: probe Y in the effective approach direction.
  fn touch_y(self) -> Touch {
    Touch { axis: Axis::Y, dir: dir_of(self.approach_y()) }
  }
}

/// A [`Dir`] from an approach sign (`≥ 0` → [`Dir::Pos`], negative → [`Dir::Neg`]).
fn dir_of(sign: f64) -> Dir {
  if sign >= 0.0 { Dir::Pos } else { Dir::Neg }
}

/// What a datum run targets: a single edge along one axis/direction, or a rectangular corner.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DatumTarget {
  /// A single-edge touch-off along `axis` in `dir` (the approach direction). Z writes the raw surface; X/Y are
  /// tip-comped.
  Edge { axis: Axis, dir: Dir },
  /// A rectangular corner (X then Y).
  Corner(Corner),
}

/// The datum wizard's step. The ordering encodes the mandatory sequence; a compute step is unreachable until the
/// readings it needs are captured, so the view never has to disambiguate a half-finished run.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DatumStep {
  /// Awaiting the operator to jog to the approach and trigger the (first) touch.
  EnterParams,
  /// A single-edge touch has been issued; awaiting its result.
  ProbeEdge,
  /// The corner's X-face touch has been issued; awaiting its result.
  ProbeFaceX,
  /// The X face is captured; awaiting the operator to jog to the Y-face approach and trigger it.
  ReadyFaceY,
  /// The corner's Y-face touch has been issued; awaiting its result.
  ProbeFaceY,
  /// All required touches captured and the datum computed; the operator reviews and may write the WCS.
  Review,
  /// A touch failed (or the operator cancelled). Terminal — carries the reason; no partial result is exposed.
  Aborted,
}

/// The live datum-finder state: the step, the target, the tip diameter, and the captured RAW machine contacts.
/// Pure — the shell mutates it via the methods below and renders it through the view. Tip compensation and the
/// `G10 L2` line are computed on demand from the raw contacts, so the readings stay auditable.
#[derive(Debug, Clone, PartialEq)]
pub struct DatumState {
  /// The current step.
  pub step: DatumStep,
  /// What this run targets (a single edge or a corner).
  pub target: DatumTarget,
  /// The probe tip/ball diameter (mm) used for lateral tip-radius compensation. Snapshotted at start so a later
  /// bench-param edit cannot retroactively change a captured reading's comp.
  pub probe_diameter: f64,
  /// The RAW machine contact along the single edge's axis, once `ProbeEdge` resolves (edge target only).
  pub edge_contact: Option<f64>,
  /// The RAW machine-X contact of the corner's X face, once `ProbeFaceX` resolves (corner target only).
  pub x_contact: Option<f64>,
  /// The RAW machine-Y contact of the corner's Y face, once `ProbeFaceY` resolves (corner target only).
  pub y_contact: Option<f64>,
  /// The reason the wizard aborted, set with [`DatumStep::Aborted`].
  pub abort_reason: Option<String>,
}

impl DatumState {
  /// Start a single-edge run along `axis` in `dir` (the approach direction), with `probe_diameter` for tip comp.
  pub fn new_edge(axis: Axis, dir: Dir, probe_diameter: f64) -> Self {
    DatumState {
      step: DatumStep::EnterParams,
      target: DatumTarget::Edge { axis, dir },
      probe_diameter,
      edge_contact: None,
      x_contact: None,
      y_contact: None,
      abort_reason: None,
    }
  }

  /// Start a corner run for `corner` (in/out, one of four), with `probe_diameter` for tip comp. The operator jogs
  /// to each face's approach manually (like the rotary wizard); the corner-slide AUTO-positioning — and with it the
  /// inside-corner standoff clamp (`ProbeParams::xy_clearance ≤ offset`) — is DEFERRED, so those params are
  /// reserved but not yet applied to any emitted g-code.
  pub fn new_corner(corner: Corner, probe_diameter: f64) -> Self {
    DatumState {
      step: DatumStep::EnterParams,
      target: DatumTarget::Corner(corner),
      probe_diameter,
      edge_contact: None,
      x_contact: None,
      y_contact: None,
      abort_reason: None,
    }
  }

  /// Whether a touch is currently awaiting its result (so the shell gates the latch / disables advance).
  pub fn is_probing(&self) -> bool {
    matches!(self.step, DatumStep::ProbeEdge | DatumStep::ProbeFaceX | DatumStep::ProbeFaceY)
  }

  /// Begin the touch due in the current step, advancing into the matching probing step and returning the [`Touch`]
  /// to issue — or `None` if no touch is due here (a guard against a double-advance). The shell turns the touch
  /// into lines ([`super::touch::touch_lines`]), sends them, and arms the probe latch before the SLOW pass.
  pub fn begin_probe(&mut self) -> Option<Touch> {
    match (self.step, self.target) {
      (DatumStep::EnterParams, DatumTarget::Edge { axis, dir }) => {
        self.step = DatumStep::ProbeEdge;
        Some(Touch { axis, dir })
      }
      (DatumStep::EnterParams, DatumTarget::Corner(corner)) => {
        self.step = DatumStep::ProbeFaceX;
        Some(corner.touch_x())
      }
      (DatumStep::ReadyFaceY, DatumTarget::Corner(corner)) => {
        self.step = DatumStep::ProbeFaceY;
        Some(corner.touch_y())
      }
      _ => None,
    }
  }

  /// Fold one resolved probe outcome into the wizard. A failure aborts (no partial compute). A success captures
  /// the relevant RAW contact and advances: a single edge goes to `Review`; the corner's X face goes to
  /// `ReadyFaceY` (awaiting the operator to jog to the Y approach), and its Y face goes to `Review`.
  ///
  /// `position` is the machine-coordinate `[PRB:]` reading; the probed axis's component is read at its
  /// conventional index (X=0, Y=1, Z=2). A reading too short to carry the needed axis aborts rather than indexing
  /// out of bounds.
  pub fn on_probe_result(&mut self, outcome: &ProbeOutcome) {
    let position = match outcome {
      ProbeOutcome::Success { position } => position,
      ProbeOutcome::Failure { reason } => {
        self.abort(format!("probe failed: {reason}"));
        return;
      }
    };
    match (self.step, self.target) {
      (DatumStep::ProbeEdge, DatumTarget::Edge { axis, .. }) => match position.get(axis.index()) {
        Some(&contact) => {
          self.edge_contact = Some(contact);
          self.step = DatumStep::Review;
        }
        None => self.abort(format!("probe result has no {} axis", axis.letter())),
      },
      (DatumStep::ProbeFaceX, DatumTarget::Corner(_)) => match position.get(Axis::X.index()) {
        Some(&x) => {
          self.x_contact = Some(x);
          self.step = DatumStep::ReadyFaceY;
        }
        None => self.abort("probe result has no X axis".to_string()),
      },
      (DatumStep::ProbeFaceY, DatumTarget::Corner(_)) => match position.get(Axis::Y.index()) {
        Some(&y) => {
          self.y_contact = Some(y);
          self.step = DatumStep::Review;
        }
        None => self.abort("probe result has no Y axis".to_string()),
      },
      // A result arriving on a non-probing step is unexpected (the shell only resolves a touch it issued); ignore
      // it rather than corrupt state.
      _ => {}
    }
  }

  /// Abort the wizard with a reason — terminal. No partial result is exposed; the operator restarts.
  pub fn abort(&mut self, reason: impl Into<String>) {
    self.step = DatumStep::Aborted;
    self.abort_reason = Some(reason.into());
  }

  /// The compensated single-edge datum value (machine coordinate the WCS origin should land on), or `None` until
  /// the edge is captured. X/Y are tip-comped (`contact + (Ø/2)·af`); Z is the raw surface (never comped).
  pub fn edge_value(&self) -> Option<f64> {
    let (axis, dir) = match self.target {
      DatumTarget::Edge { axis, dir } => (axis, dir),
      _ => return None,
    };
    let contact = self.edge_contact?;
    Some(match axis {
      Axis::Z => contact,
      _ => edge_coord(contact, dir.sign(), self.probe_diameter),
    })
  }

  /// The compensated corner `(X, Y)` datum (machine coordinates the WCS origin should land on), or `None` until
  /// both faces are captured. Each axis is tip-comped along its effective approach direction.
  pub fn corner_xy(&self) -> Option<(f64, f64)> {
    let corner = match self.target {
      DatumTarget::Corner(corner) => corner,
      _ => return None,
    };
    let x = self.x_contact?;
    let y = self.y_contact?;
    Some((
      edge_coord(x, corner.approach_x(), self.probe_diameter),
      edge_coord(y, corner.approach_y(), self.probe_diameter),
    ))
  }

  /// The `G10 L2 P0` line that writes the found datum to the active WCS, or `None` until the datum is computed.
  /// A single edge writes ONE axis (its compensated value); a corner writes X and Y. Position-independent (`L2`),
  /// built from the machine-coordinate contacts — exactly like [`super::super::probe_flow::zero_z_line`].
  pub fn offer_g10(&self) -> Option<String> {
    match self.target {
      DatumTarget::Edge { axis, .. } => {
        let value = self.edge_value()?;
        Some(machine_offset_line(&[(axis, value)]))
      }
      DatumTarget::Corner(_) => {
        let (x, y) = self.corner_xy()?;
        Some(machine_offset_line(&[(Axis::X, x), (Axis::Y, y)]))
      }
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  fn success(position: Vec<f64>) -> ProbeOutcome {
    ProbeOutcome::Success { position }
  }

  // ── Corner sign map ─────────────────────────────────────────────────────────────────────────────────────────

  #[test]
  fn the_outside_corner_map_matches_iosenders_abcd() {
    // A=(+X,+Y), B=(−X,+Y), C=(−X,−Y), D=(+X,−Y) — the outside approach signs.
    assert_eq!((Corner::A.approach_x(), Corner::A.approach_y()), (1.0, 1.0));
    assert_eq!((Corner::B.approach_x(), Corner::B.approach_y()), (-1.0, 1.0));
    assert_eq!((Corner::C.approach_x(), Corner::C.approach_y()), (-1.0, -1.0));
    assert_eq!((Corner::D.approach_x(), Corner::D.approach_y()), (1.0, -1.0));
  }

  #[test]
  fn an_inside_corner_inverts_both_approach_signs() {
    // Inside a pocket the probe approaches each wall from the opposite side, so both signs flip.
    let inside_a = Corner::A.inside();
    assert!(inside_a.inside);
    assert_eq!((inside_a.approach_x(), inside_a.approach_y()), (-1.0, -1.0));
    let inside_d = Corner::D.inside();
    assert_eq!((inside_d.approach_x(), inside_d.approach_y()), (-1.0, 1.0));
  }

  // ── Single edge ─────────────────────────────────────────────────────────────────────────────────────────────

  #[test]
  fn a_single_x_edge_touch_comps_the_reading_and_writes_one_axis() {
    // Approaching +X with a 4 mm tip, contact at machine-X 10 → edge at 12. The write is a one-axis G10 L2.
    let mut w = DatumState::new_edge(Axis::X, Dir::Pos, 4.0);
    let touch = w.begin_probe().expect("the edge touch starts from EnterParams");
    assert_eq!(touch, Touch { axis: Axis::X, dir: Dir::Pos });
    assert_eq!(w.step, DatumStep::ProbeEdge);
    w.on_probe_result(&success(vec![10.0, 0.0, 0.0]));
    assert_eq!(w.step, DatumStep::Review);
    assert_eq!(w.edge_value(), Some(12.0));
    assert_eq!(w.offer_g10().as_deref(), Some("G10 L2 P0 X12.000"));
  }

  #[test]
  fn a_negative_x_edge_comps_the_other_way() {
    // Approaching −X, the edge is below the contact by the tip radius: contact 10, Ø 4 → 8.
    let mut w = DatumState::new_edge(Axis::X, Dir::Neg, 4.0);
    w.begin_probe();
    w.on_probe_result(&success(vec![10.0, 0.0, 0.0]));
    assert_eq!(w.offer_g10().as_deref(), Some("G10 L2 P0 X8.000"));
  }

  #[test]
  fn a_z_surface_touch_is_never_tip_compensated() {
    // Z is a surface touch-off: the tip reads the surface directly, so no radius is applied. Contact −3.5 stays −3.5.
    let mut w = DatumState::new_edge(Axis::Z, Dir::Neg, 6.0);
    w.begin_probe();
    w.on_probe_result(&success(vec![0.0, 0.0, -3.5]));
    assert_eq!(w.edge_value(), Some(-3.5), "Z must not be tip-comped");
    assert_eq!(w.offer_g10().as_deref(), Some("G10 L2 P0 Z-3.500"));
  }

  // ── Corner ──────────────────────────────────────────────────────────────────────────────────────────────────

  #[test]
  fn a_corner_captures_two_faces_and_writes_comped_x_and_y() {
    // Outside corner A (+X,+Y), Ø 2 (radius 1). X face contact 5 → edge 6; Y face contact 8 → edge 9.
    let mut w = DatumState::new_corner(Corner::A, 2.0);
    let tx = w.begin_probe().expect("the X face starts from EnterParams");
    assert_eq!(tx, Touch { axis: Axis::X, dir: Dir::Pos });
    assert_eq!(w.step, DatumStep::ProbeFaceX);
    w.on_probe_result(&success(vec![5.0, 0.0, 0.0]));
    // X captured → ReadyFaceY (the operator jogs to the Y approach before triggering the Y touch).
    assert_eq!(w.step, DatumStep::ReadyFaceY);
    let ty = w.begin_probe().expect("the Y face starts from ReadyFaceY");
    assert_eq!(ty, Touch { axis: Axis::Y, dir: Dir::Pos });
    assert_eq!(w.step, DatumStep::ProbeFaceY);
    w.on_probe_result(&success(vec![0.0, 8.0, 0.0]));
    assert_eq!(w.step, DatumStep::Review);
    assert_eq!(w.corner_xy(), Some((6.0, 9.0)));
    assert_eq!(w.offer_g10().as_deref(), Some("G10 L2 P0 X6.000 Y9.000"));
  }

  #[test]
  fn an_inside_corner_comps_toward_the_pocket_walls() {
    // Inside corner A (approach −X,−Y), Ø 2. X face contact 5 → edge 4; Y face contact 8 → edge 7.
    let mut w = DatumState::new_corner(Corner::A.inside(), 2.0);
    let tx = w.begin_probe().expect("X face");
    assert_eq!(tx.dir, Dir::Neg, "an inside corner approaches −X for the A location");
    w.on_probe_result(&success(vec![5.0, 0.0, 0.0]));
    w.begin_probe();
    w.on_probe_result(&success(vec![0.0, 8.0, 0.0]));
    assert_eq!(w.corner_xy(), Some((4.0, 7.0)));
    assert_eq!(w.offer_g10().as_deref(), Some("G10 L2 P0 X4.000 Y7.000"));
  }

  #[test]
  fn the_y_face_is_unreachable_before_the_x_face_is_captured() {
    // The ordering guard: begin_probe on the corner issues X first, and cannot issue Y until X resolves (ReadyFaceY).
    let mut w = DatumState::new_corner(Corner::A, 2.0);
    w.begin_probe(); // → ProbeFaceX
    // A second begin_probe while awaiting the X result is refused (idempotent guard).
    assert_eq!(w.begin_probe(), None, "a second begin while probing X must be refused");
    assert_eq!(w.step, DatumStep::ProbeFaceX);
    // No corner datum yet — only one face captured.
    w.on_probe_result(&success(vec![5.0, 0.0, 0.0]));
    assert_eq!(w.corner_xy(), None, "one face is not a corner");
    assert_eq!(w.offer_g10(), None, "no G10 offered from half a corner");
  }

  // ── Failure ─────────────────────────────────────────────────────────────────────────────────────────────────

  #[test]
  fn a_miss_on_the_first_face_aborts_without_a_write() {
    // A G38.3 miss surfaces as a Failure (the `:0` flag). It must abort the wizard and expose no datum/offset.
    let mut w = DatumState::new_corner(Corner::A, 2.0);
    w.begin_probe();
    w.on_probe_result(&ProbeOutcome::Failure { reason: "probe did not contact (flag :0)".to_string() });
    assert_eq!(w.step, DatumStep::Aborted);
    assert!(w.abort_reason.as_deref().unwrap().contains(":0"));
    assert_eq!(w.corner_xy(), None);
    assert_eq!(w.offer_g10(), None, "an aborted run must never offer a write");
  }

  #[test]
  fn a_miss_on_the_second_face_aborts_after_the_first_is_captured() {
    let mut w = DatumState::new_corner(Corner::A, 2.0);
    w.begin_probe();
    w.on_probe_result(&success(vec![5.0, 0.0, 0.0]));
    w.begin_probe();
    w.on_probe_result(&ProbeOutcome::Failure { reason: "no contact".to_string() });
    assert_eq!(w.step, DatumStep::Aborted);
    // X was captured, but the wizard does NOT expose a partial corner off a failed Y touch.
    assert_eq!(w.corner_xy(), None);
    assert_eq!(w.offer_g10(), None);
  }

  #[test]
  fn a_short_reading_aborts_rather_than_panicking() {
    // A malformed/short `[PRB:]` must abort, never index out of bounds.
    let mut w = DatumState::new_edge(Axis::Z, Dir::Neg, 6.0);
    w.begin_probe();
    w.on_probe_result(&success(vec![]));
    assert_eq!(w.step, DatumStep::Aborted);
  }

  #[test]
  fn no_g10_is_offered_before_a_datum_is_known() {
    let mut w = DatumState::new_edge(Axis::X, Dir::Pos, 2.0);
    assert_eq!(w.offer_g10(), None, "no offer before any touch");
    w.begin_probe();
    assert_eq!(w.offer_g10(), None, "no offer while awaiting the touch");
  }

}
