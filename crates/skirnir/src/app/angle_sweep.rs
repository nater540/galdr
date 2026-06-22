//! A shared multi-touch "probe at a list of A angles, collect readings" engine.
//!
//! Both Phase 2 wizards — the 180°-flip center-verify ([`super::flip_verify`]) and the runout report
//! ([`super::runout`]) — are the same shape: index A to each of a fixed list of angles, run an identical
//! rotary-safe probe at each, and collect one radial reading per angle. (The Phase 1 center-finder is NOT this
//! shape — it has heterogeneous touches and an interleaved move step — so it keeps its own state machine; this
//! engine deliberately covers only the homogeneous sweep.) Factoring the sweep here means one tested
//! begin→await→fold loop and a single kind-dispatched shell pump drive both wizards, instead of two more bespoke
//! state machines.
//!
//! Pure and egui-free: it sequences the touches and captures the readings; the per-wizard math
//! (`error = (r2−r1)/2`, `TIR = max−min`, …) lives in the wizard modules that consume the completed readings.
//! Each touch composes [`super::rotary_probe::rotary_safe_probe_lines`] and resolves through the Phase 0 latch;
//! the shell adds the shared lost-push fallback ([`super::probe_flow::await_action`]).

use super::intent::{Axis, Dir};
use super::rotary_probe::RotaryTouch;
use super::view_state::ProbeOutcome;

/// The sweep's step. A single meaning per variant so the shell/view never disambiguate by inspecting readings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SweepStep {
  /// Ready for the operator to trigger the next touch (the approach has been jogged). `Idle` between touches.
  Ready,
  /// A touch has been issued for the current angle; awaiting its `[PRB:]` result.
  Probing,
  /// Every angle has been probed; the readings are complete and the wizard may compute its result.
  Done,
  /// A touch failed (or the operator cancelled). Terminal — no partial readings are exposed for compute.
  Aborted,
}

/// Generate `n` evenly-spaced angles starting at `start_deg`: `start, start + 360/n, …`. Returns an empty vec
/// for `n == 0`. Used by the runout report ("probe at N angles around the part"); the flip-verify builds its own
/// two-angle list (θ, θ+180) directly.
pub fn evenly_spaced_angles(n: usize, start_deg: f64) -> Vec<f64> {
  if n == 0 {
    return Vec::new();
  }
  let step = 360.0 / n as f64;
  (0..n).map(|i| start_deg + step * i as f64).collect()
}

/// The shared sweep state: the angle list, the fixed linear probe axis/direction, and the readings collected so
/// far. Pure — the shell drives it (issuing each touch, folding each [`ProbeOutcome`]) and the wizard modules
/// read [`Self::readings`] once [`Self::is_done`]. The radial reading captured per touch is the probed `axis`
/// component of the `[PRB:]` machine-coordinate position.
#[derive(Debug, Clone, PartialEq)]
pub struct AngleSweep {
  /// The A angles (degrees) to probe, in order.
  angles: Vec<f64>,
  /// The linear axis every touch probes along — never A. Also selects which position component is the reading.
  axis: Axis,
  /// The direction along that axis the probe advances.
  dir: Dir,
  /// The readings captured so far, one per completed angle, in angle order.
  readings: Vec<f64>,
  /// The current step.
  step: SweepStep,
  /// The reason the sweep aborted, set with [`SweepStep::Aborted`].
  abort_reason: Option<String>,
}

impl AngleSweep {
  /// Begin a sweep over `angles`, probing along `axis` in `dir`. A sweep with no angles is immediately `Done`
  /// (nothing to probe) so a degenerate `N = 0` cannot strand the wizard awaiting a touch that never issues.
  pub fn new(angles: Vec<f64>, axis: Axis, dir: Dir) -> Self {
    let step = if angles.is_empty() { SweepStep::Done } else { SweepStep::Ready };
    AngleSweep { angles, axis, dir, readings: Vec::new(), step, abort_reason: None }
  }

  /// The current step.
  pub fn step(&self) -> SweepStep {
    self.step
  }

  /// The readings captured so far (complete once [`Self::is_done`]). Borrowed for the wizard's compute.
  pub fn readings(&self) -> &[f64] {
    &self.readings
  }

  /// The full angle list, for the view (e.g. a per-angle readings table).
  pub fn angles(&self) -> &[f64] {
    &self.angles
  }

  /// The linear axis every touch probes along (also the axis a flip-verify correction would adjust).
  pub fn probe_axis(&self) -> Axis {
    self.axis
  }

  /// The 1-based index of the touch currently due or in flight (`readings.len() + 1`), for operator guidance
  /// ("probe 2 of 4"). Equals the count once done.
  pub fn current_touch_number(&self) -> usize {
    self.readings.len() + 1
  }

  /// The total number of touches in the sweep.
  pub fn total_touches(&self) -> usize {
    self.angles.len()
  }

  /// Whether a touch is currently in flight (awaiting its result), so the shell gates the latch / disables advance.
  pub fn is_probing(&self) -> bool {
    self.step == SweepStep::Probing
  }

  /// Whether every angle has been probed and the readings are ready to compute.
  pub fn is_done(&self) -> bool {
    self.step == SweepStep::Done
  }

  /// The reason the sweep aborted, if it did.
  pub fn abort_reason(&self) -> Option<&str> {
    self.abort_reason.as_deref()
  }

  /// Begin the next touch: from `Ready` into `Probing`, returning the rotary-safe touch for the current angle
  /// (the angle at index `readings.len()`). Returns `None` off-step (not `Ready`) or when no angle remains — a
  /// guard against a double-advance. The shell turns the touch into lines and arms the latch.
  pub fn begin_next_touch(&mut self) -> Option<RotaryTouch> {
    if self.step != SweepStep::Ready {
      return None;
    }
    let angle = *self.angles.get(self.readings.len())?;
    self.step = SweepStep::Probing;
    Some(RotaryTouch { angle_deg: angle, axis: self.axis, dir: self.dir })
  }

  /// Fold one resolved probe outcome into the sweep. A failure aborts (no partial readings exposed). A success
  /// captures the probed-axis component of the reading and advances: back to `Ready` for the next angle, or
  /// `Done` once the last angle is in. A reading too short to carry the probed axis aborts rather than indexing
  /// out of bounds.
  pub fn on_probe_result(&mut self, outcome: &ProbeOutcome) {
    if self.step != SweepStep::Probing {
      // A result on a non-probing step is unexpected (the shell only resolves a touch it issued); ignore it.
      return;
    }
    let position = match outcome {
      ProbeOutcome::Success { position } => position,
      ProbeOutcome::Failure { reason } => {
        self.abort(format!("probe failed: {reason}"));
        return;
      }
    };
    match position.get(self.axis.index()) {
      Some(&reading) => {
        self.readings.push(reading);
        self.step = if self.readings.len() == self.angles.len() { SweepStep::Done } else { SweepStep::Ready };
      }
      None => self.abort(format!("probe result has no {} axis", self.axis.letter())),
    }
  }

  /// Abort the sweep with a reason — terminal. No partial readings are exposed for compute.
  pub fn abort(&mut self, reason: impl Into<String>) {
    self.step = SweepStep::Aborted;
    self.abort_reason = Some(reason.into());
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  fn success(position: Vec<f64>) -> ProbeOutcome {
    ProbeOutcome::Success { position }
  }

  #[test]
  fn evenly_spaced_angles_divides_the_circle() {
    assert_eq!(evenly_spaced_angles(4, 0.0), vec![0.0, 90.0, 180.0, 270.0]);
    assert_eq!(evenly_spaced_angles(3, 0.0), vec![0.0, 120.0, 240.0]);
    assert_eq!(evenly_spaced_angles(1, 0.0), vec![0.0]);
    assert_eq!(evenly_spaced_angles(0, 0.0), Vec::<f64>::new());
    // A non-zero start offsets every angle.
    assert_eq!(evenly_spaced_angles(2, 45.0), vec![45.0, 225.0]);
  }

  #[test]
  fn a_sweep_issues_a_touch_per_angle_and_collects_the_probed_axis_reading() {
    // Probe along Y at two angles; the reading is the Y component (index 1) of each `[PRB:]` position.
    let mut s = AngleSweep::new(vec![0.0, 180.0], Axis::Y, Dir::Neg);
    assert_eq!(s.step(), SweepStep::Ready);
    let t1 = s.begin_next_touch().expect("first touch");
    assert_eq!(t1, RotaryTouch { angle_deg: 0.0, axis: Axis::Y, dir: Dir::Neg });
    assert!(s.is_probing());
    s.on_probe_result(&success(vec![0.0, 2.0, 0.0]));
    assert_eq!(s.step(), SweepStep::Ready, "after the first of two, back to Ready for the next angle");
    let t2 = s.begin_next_touch().expect("second touch");
    assert_eq!(t2.angle_deg, 180.0);
    s.on_probe_result(&success(vec![0.0, 6.0, 0.0]));
    assert!(s.is_done());
    assert_eq!(s.readings(), &[2.0, 6.0]);
  }

  #[test]
  fn a_sweep_collects_the_x_reading_when_probing_along_x() {
    let mut s = AngleSweep::new(vec![0.0], Axis::X, Dir::Pos);
    s.begin_next_touch();
    s.on_probe_result(&success(vec![7.5, 99.0, 99.0]));
    assert_eq!(s.readings(), &[7.5], "the reading must be the X component when probing along X");
  }

  #[test]
  fn begin_next_touch_is_refused_while_probing_and_when_done() {
    let mut s = AngleSweep::new(vec![0.0], Axis::Y, Dir::Neg);
    assert!(s.begin_next_touch().is_some());
    assert_eq!(s.begin_next_touch(), None, "a second begin while probing must be refused");
    s.on_probe_result(&success(vec![0.0, 1.0, 0.0]));
    assert!(s.is_done());
    assert_eq!(s.begin_next_touch(), None, "no touch is due once done");
  }

  #[test]
  fn a_failed_touch_aborts_with_no_partial_readings() {
    let mut s = AngleSweep::new(vec![0.0, 120.0, 240.0], Axis::Y, Dir::Neg);
    s.begin_next_touch();
    s.on_probe_result(&success(vec![0.0, 1.0, 0.0]));
    s.begin_next_touch();
    s.on_probe_result(&ProbeOutcome::Failure { reason: "ALARM:5 during probe".to_string() });
    assert_eq!(s.step(), SweepStep::Aborted);
    assert!(s.abort_reason().unwrap().contains("ALARM:5"));
    assert!(!s.is_done(), "an aborted sweep is never done");
  }

  #[test]
  fn a_reading_missing_the_probed_axis_aborts_rather_than_panicking() {
    // Probing along Z but the reading has no Z component (index 2): abort, never index out of bounds.
    let mut s = AngleSweep::new(vec![0.0], Axis::Z, Dir::Neg);
    s.begin_next_touch();
    s.on_probe_result(&success(vec![0.0, 0.0]));
    assert_eq!(s.step(), SweepStep::Aborted);
  }

  #[test]
  fn an_empty_sweep_is_immediately_done() {
    // A degenerate N=0 must not strand the wizard awaiting a touch that never issues.
    let s = AngleSweep::new(Vec::new(), Axis::Y, Dir::Neg);
    assert!(s.is_done());
    assert_eq!(s.readings(), &[] as &[f64]);
  }

  #[test]
  fn touch_numbering_tracks_progress() {
    let mut s = AngleSweep::new(vec![0.0, 90.0, 180.0], Axis::Y, Dir::Neg);
    assert_eq!((s.current_touch_number(), s.total_touches()), (1, 3));
    s.begin_next_touch();
    s.on_probe_result(&success(vec![0.0, 1.0, 0.0]));
    assert_eq!(s.current_touch_number(), 2, "after one reading we are on touch 2");
  }
}
