//! The rotary center-finder wizard: a pure, multi-step state machine that locates the A centerline.
//!
//! This finds the rotary axis's centerline — its machine-coordinate Y and Z — relative to the spindle, using a
//! known-diameter dowel/gauge clamped concentric with A. It is the first genuinely MULTI-STEP probing flow (Phase
//! 0 was a single touch), so it is modelled as a pure state machine in the spirit of [`super::probe_flow::decide`]:
//! the shell drives it (sending the lines each step yields, feeding back each [`ProbeOutcome`]) and [`views`] just
//! renders [`WizardState`]. All the math and step-sequencing is egui-free and unit-tested without a window.
//!
//! **Math (DOC-11 §1.2, deep-research-confirmed — the symmetric two-sided formulas, not a single-value shortcut):**
//! - **Y center FIRST:** probe both sides of the dowel at center height → `Y_c = (Y_left + Y_right) / 2`. The
//!   tool/probe radius CANCELS in the midpoint (the two touches are equal-and-opposite), so no tip-radius term is
//!   needed for Y.
//! - **Then Z center:** move to the true `Y_c`, probe the top → `Z_c = Z_top − D/2` (D = dowel diameter, operator
//!   input). The order is MANDATORY — a top probe off the Y center reads a chord, not the diameter — so the state
//!   machine forces a `MoveToYc` step between the Y touches and the Z-top touch; `ProbeZTop` is unreachable
//!   without it.
//!
//! **Failure handling:** a failed/aborted touch (`success:false` or an alarm, surfaced as
//! [`ProbeOutcome::Failure`]) stops the wizard at [`WizardStep::Aborted`] with a clear reason — never a partial
//! compute off a bad reading.
//!
//! **WCS write:** the wizard offers a `G10 L2` line carrying ONLY the Y and Z words (never an `A` word, so the
//! rotary datum is untouched). The Y word is always the axis centerline; the Z word follows an operator-selectable
//! [`ZDatum`] — the rotary axis centerline (`Z_top − D/2`, the default) or the probed top surface (`Z_top`). See
//! [`WizardState::offer_g10`] for the L2-vs-L20 rationale.

use super::intent::{Axis, Dir};
use super::rotary_probe::{RotaryProbeParams, RotaryTouch};
use super::view_state::ProbeOutcome;

/// Which work-coordinate system the offered `G10 L2` line targets. `P0` = the active WCS; this is the only one
/// the first version offers (the operator selects the active WCS in the firmware), kept as an enum so a later
/// G54–G59 picker slots in without changing the line builder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Wcs {
  /// `P0` — the currently active work-coordinate system.
  Active,
}

/// Which physical feature the operator wants work-Z0 to land on when the found center is written to the WCS. A
/// product choice: wrap machining conventionally zeroes on the rotary AXIS, but some workflows zero on the
/// probed TOP SURFACE. The Y datum is always the axis centerline (`Y_c`); only Z is selectable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ZDatum {
  /// Work-Z0 = the rotary axis centerline (`Z_c = Z_top − D/2`). The default — standard for wrap machining.
  #[default]
  AxisCenterline,
  /// Work-Z0 = the probed cylinder top surface (`Z_top`); the rotary axis then sits at work-Z = −D/2.
  TopSurface,
}

impl Wcs {
  /// The `Pn` selector digit for this WCS (`P0` = active).
  fn selector(self) -> u32 {
    match self {
      Wcs::Active => 0,
    }
  }
}

/// The wizard's step — an explicit state machine. Each step has a SINGLE meaning (so the view never has to
/// disambiguate by inspecting the readings). The order encodes the mandatory sequence: left Y touch, right Y
/// touch, THEN a move to the computed Y center, THEN the Z-top touch, THEN review. `ProbeZTop` is unreachable
/// until `MoveToYc` has run, which is what guarantees the top probe reads the diameter (not a chord).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WizardStep {
  /// Awaiting the operator's dowel diameter `D` and a jog to the left-touch approach. Advancing issues the first
  /// Y probe.
  EnterDowel,
  /// The first Y touch (the −Y face) has been issued; awaiting its result.
  ProbeYLeft,
  /// The left reading is captured; awaiting the operator to jog to the +Y approach and trigger the right touch.
  ReadyYRight,
  /// The second Y touch (the +Y face) has been issued; awaiting its result.
  ProbeYRight,
  /// Both Y touches captured; the operator must send the positioning move to the computed `Y_c` (at clearance)
  /// before the top probe. Advancing (sending the move) transitions to `MovedToYc`.
  MoveToYc,
  /// The move to `Y_c` has been SENT; the tool is on the centerline and the top probe is now allowed. This split
  /// (separate from `MoveToYc`) is what guarantees the top probe cannot fire before the move was actually sent —
  /// the type, not just the UI, enforces the order so the top reads the diameter, not a chord.
  MovedToYc,
  /// The Z-top touch has been issued at the true `Y_c`; awaiting its result.
  ProbeZTop,
  /// All three touches captured and `(Y_c, Z_c)` computed; the operator reviews and may write the WCS.
  Review,
  /// A touch failed (or the operator cancelled). Terminal — carries the reason; no partial result is exposed.
  Aborted,
}

/// The live wizard state: the current step, the operator inputs, the captured readings, and (in `Review`) the
/// computed center. Pure — no egui, no I/O. The shell mutates it via the methods below and renders it through the
/// view. `(Y_c, Z_c, D)` live here as host state: the firmware has no pivot concept, so the center is a skirnir
/// value projected into the firmware only as a `G10` offset. (Persistence across sessions is a flagged follow-up
/// — see DOC-11 §1.3; for now the value lives in app state for the duration of the run.)
#[derive(Debug, Clone, PartialEq)]
pub struct WizardState {
  /// The current step.
  pub step: WizardStep,
  /// The operator-entered dowel/gauge diameter `D` (mm). Used only for `Z_c = Z_top − D/2`.
  pub dowel_diameter: f64,
  /// The A angle (degrees) every touch indexes to and holds. The dowel is round, so the angle is mostly about
  /// holding A still during the touch; a single angle for the run keeps the geometry simple.
  pub index_angle_deg: f64,
  /// The captured `Y_left` machine-Y reading (the −Y face), once `ProbeYLeft` resolves.
  pub y_left: Option<f64>,
  /// The captured `Y_right` machine-Y reading (the +Y face), once `ProbeYRight` resolves.
  pub y_right: Option<f64>,
  /// The captured `Z_top` machine-Z reading, once `ProbeZTop` resolves.
  pub z_top: Option<f64>,
  /// Which feature work-Z0 should land on when the WCS is written: the rotary axis centerline (default) or the
  /// probed top surface. Operator-set; only affects the Z word of [`Self::offer_g10`]. `Y_c` (the axis) and the
  /// `z_center()` math are unaffected — both remain the axis values shown in Review.
  pub z_datum: ZDatum,
  /// The reason the wizard aborted, set with [`WizardStep::Aborted`].
  pub abort_reason: Option<String>,
}

impl Default for WizardState {
  fn default() -> Self {
    WizardState {
      step: WizardStep::EnterDowel,
      dowel_diameter: 6.0,
      index_angle_deg: 0.0,
      y_left: None,
      y_right: None,
      z_top: None,
      z_datum: ZDatum::default(),
      abort_reason: None,
    }
  }
}

impl WizardState {
  /// Start a fresh wizard run for a dowel of diameter `dowel_diameter` (mm) indexed at `index_angle_deg`.
  pub fn new(dowel_diameter: f64, index_angle_deg: f64) -> Self {
    WizardState { dowel_diameter, index_angle_deg, ..WizardState::default() }
  }

  /// Whether the wizard is awaiting a probe result this step (so the shell gates the latch / disables advance).
  pub fn is_probing(&self) -> bool {
    matches!(self.step, WizardStep::ProbeYLeft | WizardStep::ProbeYRight | WizardStep::ProbeZTop)
  }

  /// The Y-coordinate the dowel's two sides average to, once both Y touches are captured. `Y_c = (Y_left +
  /// Y_right)/2`; the tool radius cancels in the midpoint. `None` until both readings exist.
  pub fn y_center(&self) -> Option<f64> {
    match (self.y_left, self.y_right) {
      (Some(l), Some(r)) => Some((l + r) / 2.0),
      _ => None,
    }
  }

  /// The Z-coordinate of the rotary centerline, once the top is probed at the true `Y_c`. `Z_c = Z_top − D/2`.
  /// `None` until the top reading exists. This is always the AXIS math (independent of [`Self::z_datum`]) and is
  /// what Review shows; the WCS-written Z follows the selected datum via [`Self::z_datum_value`].
  pub fn z_center(&self) -> Option<f64> {
    self.z_top.map(|z| z - self.dowel_diameter / 2.0)
  }

  /// The machine-Z value the WCS write should put at work-Z0, per the selected [`Self::z_datum`]:
  /// `AxisCenterline → z_center()` (`Z_top − D/2`), `TopSurface → z_top` (the raw probed top). `None` until the
  /// top is probed.
  pub fn z_datum_value(&self) -> Option<f64> {
    match self.z_datum {
      ZDatum::AxisCenterline => self.z_center(),
      ZDatum::TopSurface => self.z_top,
    }
  }

  /// Begin the left (first) Y touch: move from `EnterDowel` into `ProbeYLeft` (awaiting its result). Returns the
  /// touch to probe, or `None` if called off-step (a guard against a double-advance). The shell turns the touch
  /// into lines, sends them, and arms the Phase 0 latch.
  pub fn begin_y_left(&mut self) -> Option<RotaryTouch> {
    if self.step != WizardStep::EnterDowel {
      return None;
    }
    self.step = WizardStep::ProbeYLeft;
    Some(RotaryTouch { angle_deg: self.index_angle_deg, axis: Axis::Y, dir: Dir::Pos })
  }

  /// Begin the right (second) Y touch: from `ReadyYRight` (the left reading is captured) into `ProbeYRight`.
  /// Returns the touch, or `None` off-step.
  pub fn begin_y_right(&mut self) -> Option<RotaryTouch> {
    if self.step != WizardStep::ReadyYRight {
      return None;
    }
    self.step = WizardStep::ProbeYRight;
    Some(RotaryTouch { angle_deg: self.index_angle_deg, axis: Axis::Y, dir: Dir::Neg })
  }

  /// Record that the move to `Y_c` has been SENT: from `MoveToYc` into `MovedToYc`, which is the only step
  /// `begin_z_top` accepts. Returns whether it advanced (so the shell can confirm it was due). This is what makes
  /// "move before top probe" a type invariant: the top probe is unreachable until the move has actually been sent,
  /// not merely until the wizard reached the move step.
  pub fn mark_moved_to_yc(&mut self) -> bool {
    if self.step != WizardStep::MoveToYc {
      return false;
    }
    self.step = WizardStep::MovedToYc;
    true
  }

  /// Begin the Z-top touch: from `MovedToYc` (the move to the true `Y_c` has been SENT) into `ProbeZTop`. Returns
  /// the touch, or `None` off-step — in particular this is unreachable until [`Self::mark_moved_to_yc`] has run,
  /// which is what enforces "move to Y_c before the top probe" so the top reads the diameter, not a chord.
  pub fn begin_z_top(&mut self) -> Option<RotaryTouch> {
    if self.step != WizardStep::MovedToYc {
      return None;
    }
    self.step = WizardStep::ProbeZTop;
    Some(RotaryTouch { angle_deg: self.index_angle_deg, axis: Axis::Z, dir: Dir::Neg })
  }

  /// Fold one resolved probe outcome into the wizard. A failure aborts (no partial compute). A success captures
  /// the relevant reading and advances: after the right Y touch the wizard moves to `MoveToYc` (awaiting the
  /// operator to send the positioning move); after the Z-top touch it moves to `Review` with `(Y_c, Z_c)`
  /// computed. Called by the shell when the Phase 0 latch resolves the touch it issued.
  ///
  /// `position` is the machine-coordinate `[PRB:]` reading; we read the Y component for the Y touches and the Z
  /// component for the Z touch, at the conventional indices (X=0, Y=1, Z=2). A reading too short to carry the
  /// needed axis aborts rather than indexing out of bounds.
  pub fn on_probe_result(&mut self, outcome: &ProbeOutcome) {
    let position = match outcome {
      ProbeOutcome::Success { position } => position,
      ProbeOutcome::Failure { reason } => {
        self.abort(format!("probe failed: {reason}"));
        return;
      }
    };
    match self.step {
      WizardStep::ProbeYLeft => match position.get(1) {
        Some(&y) => {
          self.y_left = Some(y);
          // Left captured: await the operator to position for the right (+Y face) touch.
          self.step = WizardStep::ReadyYRight;
        }
        None => self.abort("probe result has no Y axis".to_string()),
      },
      WizardStep::ProbeYRight => match position.get(1) {
        Some(&y) => {
          self.y_right = Some(y);
          // Both Y touches captured: the next operator action is the move to the computed Y center.
          self.step = WizardStep::MoveToYc;
        }
        None => self.abort("probe result has no Y axis".to_string()),
      },
      WizardStep::ProbeZTop => match position.get(2) {
        Some(&z) => {
          self.z_top = Some(z);
          self.step = WizardStep::Review;
        }
        None => self.abort("probe result has no Z axis".to_string()),
      },
      // A result arriving on a non-probing step is unexpected; ignore it rather than corrupt state. The shell
      // only resolves a touch it issued, so this is defensive.
      _ => {}
    }
  }

  /// Abort the wizard with a reason — terminal. No partial result is exposed; the operator restarts.
  pub fn abort(&mut self, reason: impl Into<String>) {
    self.step = WizardStep::Aborted;
    self.abort_reason = Some(reason.into());
  }

  /// The positioning move that takes the tool to the true `Y_c` (at the rotary-safe Z clearance) before the
  /// top probe, using the shared bench `params` for the clearance. Returns the two lines (retract, then the Y
  /// move) or `None` if `Y_c` is not yet known. The retract is a `G53` machine-coordinate move (as in the
  /// rotary-safe primitive); the Y move is an absolute machine-coordinate move to `Y_c` so it lands on the
  /// computed centerline regardless of the active WCS.
  pub fn move_to_yc_lines(&self, params: RotaryProbeParams) -> Option<Vec<String>> {
    let yc = self.y_center()?;
    Some(vec![
      format!("G53 G0 Z{:.3}", params.clearance_mm),
      format!("G53 G0 Y{:.3}", yc),
    ])
  }

  /// The `G10 L2` line that writes the found center into `wcs`, or `None` until both the Y center and the Z datum
  /// value are known.
  ///
  /// The Y word is always the axis centerline (`Y_c`). The Z word follows the operator-selected [`Self::z_datum`]:
  /// `AxisCenterline` (default) makes work-Z0 the rotary axis (`Z_top − D/2`), the wrap-machining convention;
  /// `TopSurface` makes work-Z0 the probed cylinder top (`Z_top`), with the axis then at work-Z = −D/2.
  ///
  /// **L2, not L20 — rationale:** `Y_c` and the chosen Z are MACHINE coordinates we computed from the
  /// machine-coordinate `[PRB:]` readings. `G10 L2 Pn` sets the WCS origin directly from absolute machine coords,
  /// so work-Y0 lands exactly on the centerline and work-Z0 on the selected feature — independent of where the
  /// tool currently sits. `G10 L20 Pn` instead makes the *current position* read the given value, which would
  /// require the tool to be physically at that point (it is not — it is wherever the last probe left it). L2 is
  /// the correct primitive when the target is already known in machine coordinates.
  ///
  /// The line carries **only the Y and Z words** — never an `A` word — so the rotary datum is left untouched.
  pub fn offer_g10(&self, wcs: Wcs) -> Option<String> {
    let yc = self.y_center()?;
    let z = self.z_datum_value()?;
    Some(format!("G10 L2 P{} Y{:.3} Z{:.3}", wcs.selector(), yc, z))
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  fn success(position: Vec<f64>) -> ProbeOutcome {
    ProbeOutcome::Success { position }
  }

  /// Drive a full happy-path run: enter dowel, both Y touches, move-to-Yc, Z-top touch → Review.
  fn run_to_review(d: f64, y_left: f64, y_right: f64, z_top: f64) -> WizardState {
    let mut w = WizardState::new(d, 0.0);
    w.begin_y_left().expect("left touch starts from EnterDowel");
    w.on_probe_result(&success(vec![0.0, y_left, 0.0]));
    w.begin_y_right().expect("right touch starts from ReadyYRight");
    w.on_probe_result(&success(vec![0.0, y_right, 0.0]));
    assert_eq!(w.step, WizardStep::MoveToYc, "after both Y touches the wizard must move to Yc before the top probe");
    assert!(w.mark_moved_to_yc(), "the move to Yc must be sent before the top probe is allowed");
    w.begin_z_top().expect("z-top touch starts from MovedToYc");
    w.on_probe_result(&success(vec![0.0, 0.0, z_top]));
    w
  }

  #[test]
  fn computes_y_center_as_the_midpoint_so_tool_radius_cancels() {
    // Y_c = (Y_left + Y_right)/2. With sides at -3 and +5 the midpoint is +1 regardless of tool radius.
    let mut w = WizardState::new(6.0, 0.0);
    w.begin_y_left();
    w.on_probe_result(&success(vec![0.0, -3.0, 0.0]));
    w.begin_y_right();
    w.on_probe_result(&success(vec![0.0, 5.0, 0.0]));
    assert_eq!(w.y_center(), Some(1.0));
  }

  #[test]
  fn computes_z_center_as_top_minus_half_diameter() {
    // Z_c = Z_top - D/2. A 6 mm dowel whose top reads -10 has its center at -13.
    let w = run_to_review(6.0, -1.0, 1.0, -10.0);
    assert_eq!(w.step, WizardStep::Review);
    assert_eq!(w.z_center(), Some(-13.0));
    assert_eq!(w.y_center(), Some(0.0));
  }

  #[test]
  fn z_top_is_unreachable_before_moving_to_yc_enforcing_the_mandatory_order() {
    // The defining ordering guard: you cannot probe the top until the move-to-Yc step. Right after the Y touches
    // the step is MoveToYc, and begin_z_top only works from there — not from any earlier step.
    let mut w = WizardState::new(6.0, 0.0);
    w.begin_y_left();
    w.on_probe_result(&success(vec![0.0, -1.0, 0.0]));
    // After the first Y touch the step is ReadyYRight (awaiting the right touch), not yet MoveToYc.
    assert_eq!(w.step, WizardStep::ReadyYRight);
    // Try to jump straight to the top probe after only the first Y touch — must be refused.
    assert_eq!(w.begin_z_top(), None, "the top probe must be unreachable before both Y touches + move-to-Yc");
    assert_eq!(w.step, WizardStep::ReadyYRight, "a refused begin_z_top must not change the step");
    // Even after the right touch, begin_z_top is refused until MoveToYc (which it now is) — but the operator must
    // still pass through MoveToYc explicitly; that is the step the wizard lands on, enforcing the order.
    w.begin_y_right();
    w.on_probe_result(&success(vec![0.0, 1.0, 0.0]));
    assert_eq!(w.step, WizardStep::MoveToYc, "both Y touches land on MoveToYc, the gate before the top probe");
    // CRITICAL: even AT MoveToYc, the top probe is refused until the move has actually been SENT. This is the bug
    // the split fixes — gating on the step alone let the operator probe before moving (reading a chord).
    assert_eq!(w.begin_z_top(), None, "the top probe must be refused until the move is actually sent");
    assert_eq!(w.step, WizardStep::MoveToYc, "a refused begin_z_top must not change the step");
    assert!(w.mark_moved_to_yc(), "sending the move advances to MovedToYc");
    assert_eq!(w.step, WizardStep::MovedToYc);
    assert!(w.begin_z_top().is_some(), "the top probe is allowed only after the move was sent");
  }

  #[test]
  fn mark_moved_to_yc_is_refused_off_step() {
    // Advancing the move out of order must be a no-op (it only advances from MoveToYc).
    let mut w = WizardState::new(6.0, 0.0);
    assert!(!w.mark_moved_to_yc(), "the move cannot be marked sent before both Y touches");
    assert_eq!(w.step, WizardStep::EnterDowel);
  }

  #[test]
  fn a_failed_y_touch_aborts_with_no_partial_compute() {
    let mut w = WizardState::new(6.0, 0.0);
    w.begin_y_left();
    w.on_probe_result(&ProbeOutcome::Failure { reason: "ALARM:5 during probe".to_string() });
    assert_eq!(w.step, WizardStep::Aborted);
    assert!(w.abort_reason.as_deref().unwrap().contains("ALARM:5"));
    // No partial result is exposed.
    assert_eq!(w.y_center(), None);
    assert_eq!(w.y_left, None, "a failed touch captures no reading");
  }

  #[test]
  fn a_failed_z_touch_aborts_after_the_y_touches() {
    let mut w = WizardState::new(6.0, 0.0);
    w.begin_y_left();
    w.on_probe_result(&success(vec![0.0, -1.0, 0.0]));
    w.begin_y_right();
    w.on_probe_result(&success(vec![0.0, 1.0, 0.0]));
    w.mark_moved_to_yc();
    w.begin_z_top();
    w.on_probe_result(&ProbeOutcome::Failure { reason: "no contact (flag :0)".to_string() });
    assert_eq!(w.step, WizardStep::Aborted);
    // Y center was found, but the wizard does NOT expose a Z center off a failed top touch.
    assert_eq!(w.z_center(), None);
  }

  #[test]
  fn the_offered_g10_line_carries_only_y_and_z_never_a() {
    let w = run_to_review(6.0, -1.0, 1.0, -10.0);
    let line = w.offer_g10(Wcs::Active).expect("a center was found");
    assert_eq!(line, "G10 L2 P0 Y0.000 Z-13.000");
    // The defining safety property: the WCS write must never carry an A word (leave the rotary datum alone).
    assert!(!line.contains('A'), "the G10 line must not carry an A word; got {line:?}");
    // And it must be L2 (machine-coord offset), not L20.
    assert!(line.contains("L2 ") && !line.contains("L20"), "the offer must be G10 L2, not L20; got {line:?}");
  }

  #[test]
  fn the_z_datum_defaults_to_the_axis_centerline() {
    // A fresh run zeroes on the rotary axis (wrap-machining convention) unless the operator changes it.
    let w = WizardState::new(6.0, 0.0);
    assert_eq!(w.z_datum, ZDatum::AxisCenterline);
  }

  #[test]
  fn the_axis_centerline_datum_writes_z_top_minus_half_diameter() {
    // The default datum: the G10 Z word is the axis (Z_top − D/2). For a 6 mm dowel topping out at -10 → -13.
    let mut w = run_to_review(6.0, -1.0, 1.0, -10.0);
    w.z_datum = ZDatum::AxisCenterline;
    assert_eq!(w.z_datum_value(), w.z_center(), "the axis datum's Z equals z_center");
    let line = w.offer_g10(Wcs::Active).expect("a center was found");
    assert_eq!(line, "G10 L2 P0 Y0.000 Z-13.000");
  }

  #[test]
  fn the_top_surface_datum_writes_the_raw_probed_top() {
    // The alternate datum: the G10 Z word is the raw probed top (Z_top), here -10. Y is unchanged (the axis).
    let mut w = run_to_review(6.0, -1.0, 1.0, -10.0);
    w.z_datum = ZDatum::TopSurface;
    assert_eq!(w.z_datum_value(), w.z_top, "the top-surface datum's Z equals the raw z_top");
    let line = w.offer_g10(Wcs::Active).expect("a center was found");
    assert_eq!(line, "G10 L2 P0 Y0.000 Z-10.000");
    // z_center is still the axis math, unaffected by the datum selection — it is shown in Review either way.
    assert_eq!(w.z_center(), Some(-13.0), "z_center stays the axis math regardless of the chosen datum");
  }

  #[test]
  fn both_z_datums_emit_only_y_and_z_as_g10_l2() {
    // The safety properties hold for BOTH datums: only Y/Z words, never A, always G10 L2.
    for datum in [ZDatum::AxisCenterline, ZDatum::TopSurface] {
      let mut w = run_to_review(8.0, 2.0, 6.0, -5.0);
      w.z_datum = datum;
      let line = w.offer_g10(Wcs::Active).expect("a center was found");
      assert!(!line.contains('A'), "{datum:?}: the G10 line must not carry an A word; got {line:?}");
      assert!(line.contains("L2 ") && !line.contains("L20"), "{datum:?}: must be G10 L2; got {line:?}");
      assert!(line.contains('Y') && line.contains('Z'), "{datum:?}: must carry Y and Z; got {line:?}");
    }
  }

  #[test]
  fn no_g10_is_offered_before_the_center_is_known() {
    let mut w = WizardState::new(6.0, 0.0);
    assert_eq!(w.offer_g10(Wcs::Active), None, "no offer before any probe");
    w.begin_y_left();
    w.on_probe_result(&success(vec![0.0, -1.0, 0.0]));
    assert_eq!(w.offer_g10(Wcs::Active), None, "no offer with only one Y touch");
  }

  #[test]
  fn the_move_to_yc_lines_retract_then_move_to_the_computed_center() {
    let mut w = WizardState::new(6.0, 0.0);
    w.begin_y_left();
    w.on_probe_result(&success(vec![0.0, 2.0, 0.0]));
    w.begin_y_right();
    w.on_probe_result(&success(vec![0.0, 4.0, 0.0]));
    let params = RotaryProbeParams { clearance_mm: -2.0, settle_secs: 0.5, feed: 50.0, depth_mm: 10.0 };
    let lines = w.move_to_yc_lines(params).expect("Yc is known after both touches");
    assert_eq!(lines, vec!["G53 G0 Z-2.000".to_string(), "G53 G0 Y3.000".to_string()]);
  }

  #[test]
  fn a_probe_result_with_no_y_axis_aborts_rather_than_panicking() {
    // A malformed/short reading must abort, never index out of bounds.
    let mut w = WizardState::new(6.0, 0.0);
    w.begin_y_left();
    w.on_probe_result(&success(vec![]));
    assert_eq!(w.step, WizardStep::Aborted);
  }

  #[test]
  fn begin_guards_refuse_a_double_advance() {
    let mut w = WizardState::new(6.0, 0.0);
    assert!(w.begin_y_left().is_some());
    // A second begin_y_left while already probing the left touch must be refused (idempotent guard).
    assert_eq!(w.begin_y_left(), None);
    // begin_y_right is refused until the left reading is captured.
    assert_eq!(w.begin_y_right(), None, "right touch needs the left reading first");
  }
}
