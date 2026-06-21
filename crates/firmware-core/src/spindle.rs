//! Spindle controller (DOC-07): the host-tested M3/M4/M5 sequencing, RPM→duty mapping, and the M3↔M4
//! direction-reversal interlock for the WS55-220 VFD spindle.
//!
//! All hardware access goes through the [`PwmSink`] (speed) and two [`DigitalOut`] (ENABLE, DIRECTION) traits,
//! so every transition is unit-tested off-target with recording mocks; only the `firmware` binary maps these to
//! the LEDC channel + the SPIN_EN/SPIN_DIR GPIO. The controller is PURE and SYNCHRONOUS — it never blocks on a
//! timer. The two delays DOC-07 defines are owned elsewhere so the async firmware task can `.await` them without
//! the controller needing a runtime:
//! - The **spin-up delay** (`$392`) — the dwell inserted before the FIRST cutting move after an M3/M4 — is the
//!   PLANNER's job (see [`crate::planner::SpinUpGate`]); the controller energizes the spindle immediately and
//!   does not gate motion.
//! - The **reverse dwell** (`$393`) — the spin-down pause when reversing a running spindle — is surfaced as data
//!   on the [`SpindleAction::SpinDownThenReverse`] result: the controller forces the spindle to a stop NOW and
//!   tells the caller how long to wait before calling [`SpindleController::complete_reverse`] to bring the new
//!   direction up. The caller reads `$393` from settings and hands it in, so the controller stays settings-free.
//!
//! ## Logical-level conventions (the firmware impl maps polarity, see [`DigitalOut`])
//! - ENABLE: logical `true` = RUN. The firmware drives SPIN_EN (GPIO14) LOW for `true` (the WS55-220 runs when
//!   its EN terminal is pulled to GND, i.e. active-low).
//! - DIRECTION: logical `true` = CW (M3), logical `false` = CCW (M4). The firmware maps that to the F/R input.

use crate::gcode::SpindleState;
use crate::hal_traits::{DigitalOut, DigitalOutError, PwmError, PwmSink};

/// The logical DIRECTION level for M3 (clockwise). M4 (counter-clockwise) is the opposite level. The firmware
/// [`DigitalOut`] impl maps this logical level to whichever physical SPIN_DIR pin level the F/R input expects.
const DIR_CLOCKWISE: bool = true;
/// The logical ENABLE level that RUNS the spindle. The firmware impl inverts this for the active-low SPIN_EN pin.
const ENABLE_RUN: bool = true;
/// The minimum coast-down enforced before re-energizing a reversed spindle, in seconds — a safety floor on the
/// `$393` reverse dwell. DOC-07 mandates "Never reverse a running spindle": a 0 s dwell (a legacy/zero `$393`)
/// would re-enable the opposite direction while the motor still spins, so the scheduled dwell is floored here
/// regardless of the setting. The configured `$393` still governs whenever it exceeds this floor.
const MIN_REVERSE_DWELL_S: f32 = 0.5;

/// An error driving the spindle outputs. Wraps the underlying sink errors so the firmware task can surface a
/// hardware fault rather than panicking, per the firmware-core no-`unwrap` rule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpindleError {
  /// The PWM (speed) sink failed.
  Pwm(PwmError),
  /// The ENABLE output failed.
  Enable(DigitalOutError),
  /// The DIRECTION output failed.
  Direction(DigitalOutError),
}

impl From<PwmError> for SpindleError {
  fn from(e: PwmError) -> Self {
    SpindleError::Pwm(e)
  }
}

/// What [`SpindleController::apply`] did, and what (if anything) the caller must still do.
///
/// A normal command (off→on, M5 stop, or a same-direction speed change) completes in one step and returns
/// [`SpindleAction::Applied`]. A direction REVERSAL of a RUNNING spindle (M3↔M4) cannot be applied in one step —
/// reversing a spinning VFD is forbidden (DOC-07) — so the controller forces the spindle to a STOP immediately
/// (ENABLE off, duty 0) and returns [`SpindleAction::SpinDownThenReverse`], leaving the new direction un-energized.
/// The async firmware task then awaits `dwell_s` (the `$393` spin-down time) and calls
/// [`SpindleController::complete_reverse`] with the SAME state/rpm to bring the opposite direction up.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpindleAction {
  /// The command was fully applied to the outputs; nothing further is required.
  Applied,
  /// A reversal was requested while the spindle was running. The controller has ALREADY forced a full stop
  /// (ENABLE off, duty 0). The caller must wait `dwell_s` seconds, then call
  /// [`complete_reverse`](SpindleController::complete_reverse) with the same `(state, rpm)` to energize the new
  /// direction.
  SpinDownThenReverse {
    /// The configured `$393` reverse spin-down dwell in seconds the caller must await before completing the
    /// reversal. Echoed back from the value the caller passed into [`apply`](SpindleController::apply).
    dwell_s: f32,
  },
}

/// The host-tested spindle controller (DOC-07). Generic over the three output sinks so it is exercised with
/// recording mocks off-target; the firmware binary instantiates it over the LEDC channel + SPIN_EN/SPIN_DIR GPIO.
///
/// Tracks the last applied [`SpindleState`] (to detect a direction reversal) and the last applied duty (so a
/// pure speed change re-drives only what changed). The controller owns NO settings: the RPM range and the reverse
/// dwell are passed in by the caller, which reads them from the live [`crate::settings::Settings`].
pub struct SpindleController<P: PwmSink, En: DigitalOut, Dir: DigitalOut> {
  pwm: P,
  enable: En,
  direction: Dir,
  /// The last state actually driven onto the outputs. `Stop` after construction, an `emergency_stop`, or the
  /// stop half of a scheduled reversal — so the next reversal check compares against a settled direction.
  last_state: SpindleState,
  /// The last duty fraction actually driven (0.0 when stopped). Lets a same-direction speed change be detected.
  last_duty: f32,
}

impl<P: PwmSink, En: DigitalOut, Dir: DigitalOut> SpindleController<P, En, Dir> {
  /// Build a controller over the three output sinks. The controller starts in the STOPPED state with duty 0 to
  /// match the firmware's power-on (the spindle must be off until an M3/M4); the caller is expected to have left
  /// the physical outputs de-asserted, so no I/O is performed here.
  pub fn new(pwm: P, enable: En, direction: Dir) -> Self {
    SpindleController { pwm, enable, direction, last_state: SpindleState::Stop, last_duty: 0.0 }
  }

  /// Map a commanded `rpm` to a normalized `0.0..=1.0` PWM duty against the `$31`/`$30` range, per DOC-07:
  /// `duty = clamp((rpm - rpm_min) / (rpm_max - rpm_min), 0, 1)`. A non-positive span (`rpm_max <= rpm_min`, e.g.
  /// a mis-configured / disabled spindle) yields duty 0 so the spindle stays off rather than dividing by ≤0.
  pub fn rpm_to_duty(rpm: f32, rpm_min: f32, rpm_max: f32) -> f32 {
    let span = rpm_max - rpm_min;
    // A positive span is required; a non-positive or NaN span (mis-configured / disabled spindle) yields duty 0.
    // Written as the positive `span > 0.0` (NaN makes it false) to avoid the `neg_cmp_op_on_partial_ord` lint.
    if span > 0.0 {
      ((rpm - rpm_min) / span).clamp(0.0, 1.0)
    } else {
      0.0
    }
  }

  /// Apply an M3/M4/M5 spindle command, sequencing the outputs per DOC-07. `rpm` is the commanded S word;
  /// `rpm_min`/`rpm_max` are `$31`/`$30`; `reverse_dwell_s` is `$393` (echoed back on a scheduled reversal so the
  /// caller knows how long to dwell). The caller reads all three from the live settings.
  ///
  /// Sequencing:
  /// - **M5 / Stop / S0** (`state == Stop`, or `rpm == 0` regardless of state — S0 implies stop, DOC-07):
  ///   de-assert ENABLE and set duty 0.
  /// - **M3 (CW) / M4 (CCW) from stopped or already in the same direction**: set DIRECTION, set duty, assert
  ///   ENABLE (in that order so the direction and speed are settled before the spindle is energized).
  /// - **M3↔M4 reversal of a RUNNING spindle**: force a STOP now (ENABLE off, duty 0) and return
  ///   [`SpindleAction::SpinDownThenReverse`]; the caller awaits the dwell and calls
  ///   [`complete_reverse`](Self::complete_reverse).
  pub fn apply(
    &mut self,
    state: SpindleState,
    rpm: f32,
    rpm_min: f32,
    rpm_max: f32,
    reverse_dwell_s: f32,
  ) -> Result<SpindleAction, SpindleError> {
    // S0 means stop regardless of the modal M3/M4 direction (grbl/DOC-07): a zero-speed spindle is off.
    let effective = if rpm <= 0.0 { SpindleState::Stop } else { state };
    match effective {
      SpindleState::Stop => {
        self.drive_stop()?;
        Ok(SpindleAction::Applied)
      }
      SpindleState::Clockwise | SpindleState::CounterClockwise => {
        let running = self.last_state != SpindleState::Stop;
        let reversing = running && self.last_state != effective;
        if reversing {
          // Never reverse a spinning spindle: force a full stop NOW and defer the new direction to the caller's
          // post-dwell `complete_reverse`. The pending direction is intentionally NOT energized here. The dwell is
          // floored to [`MIN_REVERSE_DWELL_S`] so a 0 s / legacy `$393` can never produce an instant reversal.
          self.drive_stop()?;
          Ok(SpindleAction::SpinDownThenReverse { dwell_s: reverse_dwell_s.max(MIN_REVERSE_DWELL_S) })
        } else {
          // Off→on or a same-direction speed change: settle direction + duty, then energize.
          let duty = Self::rpm_to_duty(rpm, rpm_min, rpm_max);
          self.drive_run(effective, duty)?;
          Ok(SpindleAction::Applied)
        }
      }
    }
  }

  /// Bring up the spindle in the new direction after the reverse dwell awaited by the caller. Call with the SAME
  /// `(state, rpm)` that produced a [`SpindleAction::SpinDownThenReverse`]. This is just the energize half of a
  /// normal start (DIRECTION, duty, ENABLE), split out so the controller never blocks on the dwell itself. A
  /// `state == Stop` or `rpm == 0` here is honored as a stop (the reversal was cancelled before completion).
  pub fn complete_reverse(
    &mut self,
    state: SpindleState,
    rpm: f32,
    rpm_min: f32,
    rpm_max: f32,
  ) -> Result<(), SpindleError> {
    let effective = if rpm <= 0.0 { SpindleState::Stop } else { state };
    match effective {
      SpindleState::Stop => self.drive_stop(),
      SpindleState::Clockwise | SpindleState::CounterClockwise => {
        let duty = Self::rpm_to_duty(rpm, rpm_min, rpm_max);
        self.drive_run(effective, duty)
      }
    }
  }

  /// Unconditionally stop the spindle: de-assert ENABLE and zero the duty. For ALARM / soft-reset (`0x18`) /
  /// hard-limit / sleep — independent of motion state and the last command. Idempotent: a second call from an
  /// already-stopped controller still re-drives the safe outputs (defense in depth), which is harmless.
  pub fn emergency_stop(&mut self) -> Result<(), SpindleError> {
    self.drive_stop()
  }

  /// The last [`SpindleState`] driven onto the outputs (`Stop` when the spindle is off / stopped for a reversal).
  pub fn state(&self) -> SpindleState {
    self.last_state
  }

  /// The last normalized duty fraction driven onto the PWM sink (0.0 when stopped).
  pub fn duty(&self) -> f32 {
    self.last_duty
  }

  /// Drive the outputs to a full stop: ENABLE off FIRST (cut power before zeroing the reference so the VFD never
  /// sees a live-enable-with-zero-speed transient), then duty 0. Records the stopped state.
  fn drive_stop(&mut self) -> Result<(), SpindleError> {
    self.enable.set(!ENABLE_RUN).map_err(SpindleError::Enable)?;
    self.pwm.set_duty(0.0)?;
    self.last_state = SpindleState::Stop;
    self.last_duty = 0.0;
    Ok(())
  }

  /// Drive the outputs to RUN in `state` at `duty`: set DIRECTION, set duty, THEN assert ENABLE — so the F/R line
  /// and the speed reference are settled before the spindle is energized (DOC-07 ordering). Records the state/duty.
  fn drive_run(&mut self, state: SpindleState, duty: f32) -> Result<(), SpindleError> {
    let dir_level = matches!(state, SpindleState::Clockwise) == DIR_CLOCKWISE;
    self.direction.set(dir_level).map_err(SpindleError::Direction)?;
    self.pwm.set_duty(duty)?;
    self.enable.set(ENABLE_RUN).map_err(SpindleError::Enable)?;
    self.last_state = state;
    self.last_duty = duty;
    Ok(())
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::hal_traits::{DigitalOut, DigitalOutError, PwmError, PwmSink};

  /// A recording [`PwmSink`] capturing every duty written, mirroring the `RecordingSink` pattern used for
  /// [`StepSink`](crate::hal_traits::StepSink) elsewhere. Never errors.
  #[derive(Default)]
  struct RecordingPwm {
    duties: heapless::Vec<f32, 16>,
  }

  impl PwmSink for RecordingPwm {
    fn set_duty(&mut self, frac: f32) -> Result<(), PwmError> {
      self.duties.push(frac).expect("test pwm buffer overflow");
      Ok(())
    }
  }

  impl RecordingPwm {
    fn last(&self) -> f32 {
      *self.duties.last().expect("at least one duty written")
    }
  }

  /// A recording [`DigitalOut`] capturing every logical level written. Never errors.
  #[derive(Default)]
  struct RecordingOut {
    levels: heapless::Vec<bool, 16>,
  }

  impl DigitalOut for RecordingOut {
    fn set(&mut self, level: bool) -> Result<(), DigitalOutError> {
      self.levels.push(level).expect("test out buffer overflow");
      Ok(())
    }
  }

  impl RecordingOut {
    fn last(&self) -> bool {
      *self.levels.last().expect("at least one level written")
    }
  }

  /// A [`DigitalOut`] that always fails, to prove the controller surfaces a transport error rather than panicking.
  struct FailingOut;

  impl DigitalOut for FailingOut {
    fn set(&mut self, _level: bool) -> Result<(), DigitalOutError> {
      Err(DigitalOutError::Transport)
    }
  }

  type Ctrl = SpindleController<RecordingPwm, RecordingOut, RecordingOut>;

  fn controller() -> Ctrl {
    SpindleController::new(RecordingPwm::default(), RecordingOut::default(), RecordingOut::default())
  }

  // The standard test range: $31 = 0, $30 = 10000 RPM, so an S5000 maps to exactly 0.5 duty.
  const MIN: f32 = 0.0;
  const MAX: f32 = 10_000.0;
  const DWELL: f32 = 1.5;

  #[test]
  fn rpm_to_duty_maps_linearly_and_clamps() {
    assert_eq!(Ctrl::rpm_to_duty(0.0, MIN, MAX), 0.0);
    assert_eq!(Ctrl::rpm_to_duty(5_000.0, MIN, MAX), 0.5);
    assert_eq!(Ctrl::rpm_to_duty(10_000.0, MIN, MAX), 1.0);
    // Below the min clamps to 0, above the max clamps to 1.
    assert_eq!(Ctrl::rpm_to_duty(20_000.0, MIN, MAX), 1.0);
    // A non-zero $31 shifts the bottom of the range.
    assert_eq!(Ctrl::rpm_to_duty(1_000.0, 1_000.0, 11_000.0), 0.0);
    assert_eq!(Ctrl::rpm_to_duty(6_000.0, 1_000.0, 11_000.0), 0.5);
    // A degenerate range (max <= min) disables the spindle: duty 0, no divide-by-non-positive.
    assert_eq!(Ctrl::rpm_to_duty(5_000.0, 10_000.0, 10_000.0), 0.0);
    assert_eq!(Ctrl::rpm_to_duty(5_000.0, 10_000.0, 1_000.0), 0.0);
  }

  #[test]
  fn off_to_cw_sets_direction_duty_then_enables() {
    let mut c = controller();
    let action = c.apply(SpindleState::Clockwise, 5_000.0, MIN, MAX, DWELL).expect("apply");
    assert_eq!(action, SpindleAction::Applied);
    assert_eq!(c.duty(), 0.5);
    assert_eq!(c.state(), SpindleState::Clockwise);
    assert_eq!(c.direction.last(), DIR_CLOCKWISE, "CW = DIR_CLOCKWISE level");
    assert_eq!(c.pwm.last(), 0.5);
    assert_eq!(c.enable.last(), ENABLE_RUN, "ENABLE asserted (run)");
    // Ordering: direction and duty are driven before enable is asserted.
    assert_eq!(c.direction.levels.len(), 1);
    assert_eq!(c.enable.levels, &[ENABLE_RUN][..]);
  }

  #[test]
  fn off_to_ccw_sets_opposite_direction() {
    let mut c = controller();
    c.apply(SpindleState::CounterClockwise, 10_000.0, MIN, MAX, DWELL).expect("apply");
    assert_eq!(c.state(), SpindleState::CounterClockwise);
    assert_eq!(c.direction.last(), !DIR_CLOCKWISE, "CCW = opposite of DIR_CLOCKWISE");
    assert_eq!(c.duty(), 1.0);
    assert_eq!(c.enable.last(), ENABLE_RUN);
  }

  #[test]
  fn same_direction_speed_change_re_drives_duty_without_dwell() {
    let mut c = controller();
    c.apply(SpindleState::Clockwise, 2_500.0, MIN, MAX, DWELL).expect("start");
    assert_eq!(c.duty(), 0.25);
    let action = c.apply(SpindleState::Clockwise, 7_500.0, MIN, MAX, DWELL).expect("speed change");
    // A speed change in the SAME direction is applied immediately — no reversal, no dwell.
    assert_eq!(action, SpindleAction::Applied);
    assert_eq!(c.duty(), 0.75);
    assert_eq!(c.state(), SpindleState::Clockwise);
    // Still enabled (run) — the spindle never stopped for a same-direction change.
    assert_eq!(c.enable.last(), ENABLE_RUN);
  }

  #[test]
  fn cw_to_ccw_reversal_forces_stop_and_schedules_completion() {
    let mut c = controller();
    c.apply(SpindleState::Clockwise, 5_000.0, MIN, MAX, DWELL).expect("start cw");
    let action = c.apply(SpindleState::CounterClockwise, 5_000.0, MIN, MAX, DWELL).expect("reverse");
    // The reversal must NOT energize the new direction in one step: the controller stops NOW and schedules.
    assert_eq!(action, SpindleAction::SpinDownThenReverse { dwell_s: DWELL });
    assert_eq!(c.state(), SpindleState::Stop, "spindle parked at a stop during the dwell");
    assert_eq!(c.duty(), 0.0);
    assert_eq!(c.enable.last(), !ENABLE_RUN, "ENABLE de-asserted for the spin-down");
    // After the caller's dwell, completing the reverse brings the new (CCW) direction up.
    c.complete_reverse(SpindleState::CounterClockwise, 5_000.0, MIN, MAX).expect("complete");
    assert_eq!(c.state(), SpindleState::CounterClockwise);
    assert_eq!(c.direction.last(), !DIR_CLOCKWISE);
    assert_eq!(c.duty(), 0.5);
    assert_eq!(c.enable.last(), ENABLE_RUN);
  }

  #[test]
  fn ccw_to_cw_reversal_also_schedules() {
    let mut c = controller();
    c.apply(SpindleState::CounterClockwise, 8_000.0, MIN, MAX, DWELL).expect("start ccw");
    let action = c.apply(SpindleState::Clockwise, 8_000.0, MIN, MAX, 2.0).expect("reverse");
    assert_eq!(action, SpindleAction::SpinDownThenReverse { dwell_s: 2.0 });
    assert_eq!(c.state(), SpindleState::Stop);
  }

  #[test]
  fn reversal_dwell_is_floored_so_a_zero_setting_never_instant_reverses() {
    // DOC-07 "never reverse a running spindle": a 0 s `$393` (a legacy decode or an explicit zero) must NOT yield
    // an instant reversal — the scheduled dwell is floored to MIN_REVERSE_DWELL_S so the motor coasts down first.
    let mut c = controller();
    c.apply(SpindleState::Clockwise, 5_000.0, MIN, MAX, 0.0).expect("start cw");
    let action = c.apply(SpindleState::CounterClockwise, 5_000.0, MIN, MAX, 0.0).expect("reverse");
    assert_eq!(action, SpindleAction::SpinDownThenReverse { dwell_s: MIN_REVERSE_DWELL_S });
    // A configured dwell ABOVE the floor is honored unchanged.
    let mut c2 = controller();
    c2.apply(SpindleState::Clockwise, 5_000.0, MIN, MAX, 3.0).expect("start cw");
    let action2 = c2.apply(SpindleState::CounterClockwise, 5_000.0, MIN, MAX, 3.0).expect("reverse");
    assert_eq!(action2, SpindleAction::SpinDownThenReverse { dwell_s: 3.0 });
  }

  #[test]
  fn stop_from_running_de_asserts_enable_and_zeros_duty() {
    let mut c = controller();
    c.apply(SpindleState::Clockwise, 5_000.0, MIN, MAX, DWELL).expect("start");
    let action = c.apply(SpindleState::Stop, 0.0, MIN, MAX, DWELL).expect("stop");
    assert_eq!(action, SpindleAction::Applied);
    assert_eq!(c.state(), SpindleState::Stop);
    assert_eq!(c.duty(), 0.0);
    assert_eq!(c.pwm.last(), 0.0);
    assert_eq!(c.enable.last(), !ENABLE_RUN);
  }

  #[test]
  fn zero_rpm_is_treated_as_stop_regardless_of_state() {
    let mut c = controller();
    c.apply(SpindleState::Clockwise, 5_000.0, MIN, MAX, DWELL).expect("start");
    // An M3 S0 (spindle on, zero speed) is a stop, not a run-at-zero.
    let action = c.apply(SpindleState::Clockwise, 0.0, MIN, MAX, DWELL).expect("m3 s0");
    assert_eq!(action, SpindleAction::Applied);
    assert_eq!(c.state(), SpindleState::Stop);
    assert_eq!(c.enable.last(), !ENABLE_RUN);
  }

  #[test]
  fn s0_to_other_direction_is_not_a_reversal() {
    // M3 S0 parks at a stop; a following M4 must be a clean off→on start (no dwell), because the spindle is not
    // actually spinning — the reversal interlock only applies to a RUNNING spindle.
    let mut c = controller();
    c.apply(SpindleState::Clockwise, 0.0, MIN, MAX, DWELL).expect("m3 s0 = stop");
    assert_eq!(c.state(), SpindleState::Stop);
    let action = c.apply(SpindleState::CounterClockwise, 5_000.0, MIN, MAX, DWELL).expect("m4");
    assert_eq!(action, SpindleAction::Applied, "from a stop, the opposite direction is a plain start");
    assert_eq!(c.state(), SpindleState::CounterClockwise);
  }

  #[test]
  fn emergency_stop_is_idempotent_and_always_de_asserts() {
    let mut c = controller();
    c.apply(SpindleState::Clockwise, 9_000.0, MIN, MAX, DWELL).expect("start");
    c.emergency_stop().expect("estop");
    assert_eq!(c.state(), SpindleState::Stop);
    assert_eq!(c.duty(), 0.0);
    assert_eq!(c.enable.last(), !ENABLE_RUN);
    // A second e-stop from an already-stopped controller still drives the safe outputs (defense in depth).
    let before = c.enable.levels.len();
    c.emergency_stop().expect("estop again");
    assert!(c.enable.levels.len() > before, "idempotent e-stop still re-drives the safe level");
    assert_eq!(c.state(), SpindleState::Stop);
  }

  #[test]
  fn enable_transport_error_is_surfaced_not_panicked() {
    // A failing ENABLE output must propagate as a SpindleError, never an unwrap/panic in firmware-core.
    let mut c = SpindleController::new(RecordingPwm::default(), FailingOut, RecordingOut::default());
    let err = c.apply(SpindleState::Stop, 0.0, MIN, MAX, DWELL).expect_err("enable fails");
    assert_eq!(err, SpindleError::Enable(DigitalOutError::Transport));
  }
}
