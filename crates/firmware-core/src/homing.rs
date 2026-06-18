//! Homing cycle state machine (DOC-06).
//!
//! The homing cycle establishes machine position by driving each axis into its limit switch, then setting
//! machine zero. This module is the PURE, host-tested core: it owns the per-axis phase sequencing
//! (seek → pull-off → locate → final pull-off), the no-contact fail bound, and the post-homing
//! set-machine-zero math. It is `no_std`, allocation-free, and has no esp-hal dependency, so the whole state
//! machine runs under `cargo test` with a recording [`StepSink`](crate::hal_traits::StepSink) and a scripted
//! mock [`DigitalIn`](crate::hal_traits::DigitalIn).
//!
//! ## What lives here vs the firmware bin
//! - **Here (pure):** [`HomingConfig`] (the `$`-settings the cycle needs), the single-axis primitive
//!   [`home_axis`] that runs all four phases on one axis through the [`ProbeStepper`](crate::motion::ProbeStepper)
//!   reused from the probe path, the cycle ORDER ([`HOMING_GROUPS`]), and the
//!   [`machine_zero_steps`] position math.
//! - **The bin (wiring, untestable on host):** the actual RMT channels, the limit ISR + `$26` debounce, and the
//!   X+Y CONCURRENCY (two axes home together on independent RMT channels — the bin runs their [`home_axis`]
//!   calls concurrently and stops each axis's channel as ITS switch latches, per research finding #10).
//!
//! ## Why reuse the probe stepper
//! A homing seek is mechanically a probe TOWARD a limit: walk one tick per burst, sample the switch between
//! bursts, stop on the trigger edge, count the steps actually emitted. The [`ProbeStepper`] already realizes
//! exactly that with exact-integer Bresenham; homing supplies a single-axis block and a `limit_triggered`
//! predicate. The slow locate pass (`$24`) re-approaches for a repeatable zero after the fast `$25` seek
//! overshoots — the same fast/slow split grbl uses (research findings #3/#8/#9).

use crate::hal_traits::{limit_triggered, DigitalIn, LimitConfig, StepSink};
use crate::motion::{MotionConfig, ProbeStepper};
use crate::planner::{Block, AXES};

/// The per-axis homing direction. Galdr's working envelope is `[-max_travel, 0]` in machine coordinates (machine
/// coords are non-positive), with the home switch at the machine-zero reference (the positive / top-right end,
/// spindle-up for Z). `$23=0` homes toward POSITIVE; a set `$23` bit reverses that axis to home toward negative.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HomeDirection {
  /// Home toward the positive end (the default, `$23` bit clear): the seek moves the axis in the `+` step
  /// direction until the switch trips. Post-homing machine zero sits at the switch (force-origin) or one
  /// pull-off below it.
  Positive,
  /// Home toward the negative end (`$23` bit set): the seek moves in the `−` step direction.
  Negative,
}

impl HomeDirection {
  /// The sign of the seek/locate step travel for this direction: `+1` toward positive, `−1` toward negative.
  fn sign(self) -> i32 {
    match self {
      HomeDirection::Positive => 1,
      HomeDirection::Negative => -1,
    }
  }
}

/// The cycle ORDER: each inner slice is a group of axes that home together (concurrently, on the bin's
/// independent RMT channels). grbl's default order, mirrored by DOC-06: Z first (lifts the tool clear of the
/// workpiece), then X and Y together. The firmware bin walks these groups in order, running [`home_axis`] for
/// every axis in a group before advancing to the next group; within a group the axes run concurrently and each
/// stops independently as its own switch latches.
pub const HOMING_GROUPS: &[&[usize]] = &[&[2], &[0, 1]];

/// The grbl `HOMING_AXIS_SEARCH_SCALAR`: a seek/locate may travel at most this multiple of the axis max travel
/// (`$130–$132`) before declaring no-contact and failing the cycle (research finding #7). Bounds the seek
/// distance so a never-tripping switch (or a wiring fault) aborts rather than driving the axis indefinitely.
pub const HOMING_SEARCH_SCALAR: f32 = 1.5;

/// Errors the homing cycle can raise. The firmware bin maps these onto grblHAL alarms / responses (a
/// no-contact fail becomes the homing-fail alarm + a forced reset).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HomingError {
  /// A seek or locate phase reached its 1.5×-max-travel search bound without the switch tripping
  /// (`EXEC_ALARM_HOMING_FAIL_APPROACH`). The axis that failed is carried so the bin can report which one.
  NoContact { axis: usize },
  /// The configuration is unusable for homing (a non-positive `steps_per_mm`, `max_travel`, or rate on the
  /// homed axis), so no valid seek travel can be computed. The bin validates settings before homing; a host
  /// test can still hit this with a degenerate config.
  InvalidConfig { axis: usize },
  /// The downstream [`StepSink`] failed mid-cycle (on target, an RMT transmit error). The bin treats this like
  /// a no-contact abort (position is suspect) rather than fabricating a homed state.
  Sink,
}

/// The homing `$`-settings and timing the pure cycle consumes, folded into one struct so the firmware bin
/// builds it once from [`Settings`](crate::settings::Settings) and the host tests construct it directly. All
/// per-axis fields are indexed `[X, Y, Z]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HomingConfig {
  /// `$100–$102` steps per millimeter, `[X, Y, Z]` — converts the seek/pull-off mm distances into step counts.
  pub steps_per_mm: [f32; AXES],
  /// `$130–$132` maximum travel per axis in mm, `[X, Y, Z]` — the seek/locate search distance is bounded at
  /// [`HOMING_SEARCH_SCALAR`] times this.
  pub max_travel_mm: [f32; AXES],
  /// `$25` homing SEEK rate (the fast first approach), mm/min.
  pub seek_mm_min: f32,
  /// `$24` homing FEED rate (the slow locate re-approach), mm/min.
  pub feed_mm_min: f32,
  /// `$27` homing pull-off distance, mm — backed off after each trigger to release the switch and applied once
  /// more at the end so the axis ends clear of the switch (research finding #4).
  pub pulloff_mm: f32,
  /// Per-axis home direction from the `$23` invert mask, `[X, Y, Z]`. `$23=0` homes every axis toward positive.
  pub direction: [HomeDirection; AXES],
  /// `$22` bit3 (HOMING_FORCE_SET_ORIGIN): when `true`, machine zero is `0` on every axis after homing; when
  /// `false`, each axis's zero is derived from its direction, max travel, and pull-off (research finding #5).
  pub force_set_origin: bool,
  /// The motion timing the [`ProbeStepper`] uses to encode each single-tick burst (the firmware RMT tick rate,
  /// `$0` step-pulse width). Built from `Settings::motion_config(tick_hz)`.
  pub motion: MotionConfig,
  /// `$5` limit-pin invert, folded into the trigger read so the seek/locate sampling sees the live sense.
  pub limit: LimitConfig,
}

impl HomingConfig {
  /// Convert a homing rate in mm/min into a single-axis dominant-step period in timer ticks for `axis`. A homing
  /// move is single-axis (each axis homes on its own RMT channel), so the homed axis IS the dominant axis and its
  /// step rate directly sets the period. A non-positive rate or `steps_per_mm` yields `u32::MAX` (the slowest
  /// representable period — the [`ProbeStepper`] clamps it into range), so a degenerate setting cannot produce an
  /// unencodable burst rather than dividing by zero.
  fn axis_step_period_ticks(&self, axis: usize, rate_mm_min: f32) -> u32 {
    let rate_mm_s = rate_mm_min / 60.0;
    let steps_per_mm = self.steps_per_mm[axis];
    if rate_mm_s <= 0.0 || steps_per_mm <= 0.0 || self.motion.tick_hz <= 0.0 {
      return u32::MAX;
    }
    let step_rate_hz = rate_mm_s * steps_per_mm;
    let period = self.motion.tick_hz / step_rate_hz.max(f32::MIN_POSITIVE);
    if period.is_finite() && period > 0.0 {
      libm::roundf(period) as u32
    } else {
      u32::MAX
    }
  }

  /// The seek/locate search distance bound for `axis` in steps: [`HOMING_SEARCH_SCALAR`] × `$130` × `$100`. A
  /// seek that emits this many steps without the switch tripping is a no-contact fail. Rounds (and saturates,
  /// finding #3) via [`mm_to_steps_safe`] so the bound is a whole, non-overflowing step count.
  fn search_steps(&self, axis: usize) -> u32 {
    let mm = HOMING_SEARCH_SCALAR * self.max_travel_mm[axis];
    mm_to_steps_safe(mm, self.steps_per_mm[axis])
  }

  /// The pull-off distance for `axis` in steps: `$27` × `$100`, rounded (and saturated, finding #3) via
  /// [`mm_to_steps_safe`]. Zero when `$27 = 0` (no pull-off) or any degenerate factor.
  fn pulloff_steps(&self, axis: usize) -> u32 {
    mm_to_steps_safe(self.pulloff_mm, self.steps_per_mm[axis])
  }
}

/// Convert a millimeter distance into a whole, NON-NEGATIVE step count: `round(mm × steps_per_mm)`, with one
/// single-sourced degenerate-fallback / saturation policy (findings #3 and #7). A non-finite or non-positive
/// product yields `0` (a degenerate / zero distance), and an enormous product (overflowing `i32`) SATURATES to
/// `i32::MAX as u32` rather than wrapping via a raw `as` cast. Capping at `i32::MAX` (not `u32::MAX`) keeps the
/// result safe to later cast to `i32` for a signed step delta: the magnitude can always be negated without the
/// `−1 × i32::MIN` overflow. Used by both the search/pull-off step bounds and the machine-zero travel term.
fn mm_to_steps_safe(mm: f32, steps_per_mm: f32) -> u32 {
  let steps = libm::roundf(mm * steps_per_mm);
  if steps.is_finite() && steps > 0.0 {
    // Saturate to the `i32` ceiling: anything at or above it (including `+inf`-adjacent rounded values) clamps,
    // so the magnitude is always representable as a positive `i32` and safe to negate.
    if steps >= i32::MAX as f32 {
      i32::MAX as u32
    } else {
      steps as u32
    }
  } else {
    0
  }
}

/// The outcome of homing ONE axis: the cycle completed all four phases and the axis ended clear of its switch
/// at the final pull-off position. Carries the machine-zero step position this axis should be set to, which the
/// bin syncs into the planner / parser commanded position (research finding #5).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HomeAxisOutcome {
  /// The axis that was homed, `0..AXES`.
  pub axis: usize,
  /// The machine-zero STEP position this axis should be set to after homing (the value the bin passes to
  /// [`Planner::sync_position`](crate::planner::Planner::sync_position) for this axis). Derived by
  /// [`machine_zero_steps`] from the direction, pull-off, and force-origin settings.
  pub zero_steps: i32,
}

/// The machine-zero STEP position for `axis` after a completed homing cycle (research finding #5). With
/// `force_set_origin` set (`$22` bit3) every axis zeroes to `0`. Otherwise the axis ends one pull-off clear of
/// the switch on the working side: homing toward positive (Galdr's default, working envelope below the switch at
/// `[-max_travel, 0]`) leaves the axis at `-pulloff_steps`; homing toward negative leaves it at
/// `+(max_travel + pulloff)` steps, mirroring grbl's `lround((max_travel + pulloff) * steps_per_mm)`. Pure and
/// host-tested so the position math is verified off-target.
pub fn machine_zero_steps(config: &HomingConfig, axis: usize) -> i32 {
  if config.force_set_origin {
    return 0;
  }
  // The pull-off magnitude is already saturated to `i32::MAX as u32` by `mm_to_steps_safe`, so this cast can
  // never wrap and the value can always be negated (no `−1 × i32::MIN`).
  let pulloff = config.pulloff_steps(axis) as i32;
  match config.direction[axis] {
    // Homed toward positive: the switch is the machine-zero reference at the top, and after the final pull-off
    // the tool sits one pull-off BELOW it on the working (negative) side — machine position `−pulloff`.
    HomeDirection::Positive => -pulloff,
    // Homed toward negative: zero sits at the far (negative) switch, so the post-home position is the full max
    // travel plus the pull-off, in the positive direction — grbl's `(max_travel + pulloff) * steps_per_mm`.
    HomeDirection::Negative => {
      let travel = mm_to_steps_safe(config.max_travel_mm[axis], config.steps_per_mm[axis]) as i32;
      // Both terms are ≤ `i32::MAX`; their sum can still exceed it, so add saturating rather than wrapping.
      travel.saturating_add(pulloff)
    }
  }
}

/// Decide whether a hard-limit ALARM should be raised right now, given the raw level of each limit input, the
/// `$5` invert, whether `$21` hard limits are enabled, and whether a `$H` homing cycle is currently active
/// (DOC-06). Returns the per-axis "this axis's switch is triggered" mask paired with a single `alarm` flag that
/// is `true` only when hard limits are enabled, homing is NOT active, and at least one axis is triggered.
///
/// This is the pure, host-tested core of the hard-limit path. The shared-pin rule (research finding #17) lives
/// here: while homing is active the limit switches are EXPECTED to trip, so `alarm` is forced `false` even
/// though the per-axis mask still reports the triggered switches. The firmware bin samples the limit inputs
/// (between motion bursts and on the ISR signal), calls this, and on `alarm` halts motion + raises `ALARM:1`.
pub fn hard_limit_alarm(raw_high: [bool; AXES], config: &LimitConfig, hard_limits_enabled: bool, homing_active: bool) -> HardLimitDecision {
  let mut triggered = [false; AXES];
  let mut any = false;
  for axis in 0..AXES {
    triggered[axis] = limit_triggered(raw_high[axis], config);
    any |= triggered[axis];
  }
  // The shared-pin rule: a limit trip is an alarm ONLY when `$21` is on and we are NOT mid-homing. During a
  // homing cycle the switches are supposed to trip, so the alarm is suppressed (re-armed once the cycle ends).
  let alarm = hard_limits_enabled && !homing_active && any;
  HardLimitDecision { triggered, alarm }
}

/// The result of [`hard_limit_alarm`]: which axes' switches are currently triggered, and whether that should
/// raise the hard-limit alarm now (gated by `$21` + the not-homing shared-pin rule).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HardLimitDecision {
  /// Per-axis "this axis's limit switch is triggered" mask, `[X, Y, Z]` (after the `$5` invert).
  pub triggered: [bool; AXES],
  /// `true` when a hard-limit alarm should be raised right now: `$21` enabled, NOT mid-homing, and at least one
  /// axis triggered. `false` while homing (the shared-pin rule) or when `$21` is disabled.
  pub alarm: bool,
}

/// Decide the hard-limit ALARM with EDGE-ARMING, given the raw level of each limit input, the `$5` invert, `$21`
/// enable, the homing-active flag, AND the per-axis "was this switch triggered at the PREVIOUS sample" state the
/// caller carries across calls (DOC-06). The alarm fires only on a NEW assertion — an axis that transitions
/// not-triggered -> triggered — so a switch the machine is merely PARKED on (already triggered at the previous
/// sample) never re-fires `ALARM:1`. This is the principled fix for the post-aborted-homing `error:9` wedge: a
/// `$H` seek/abort/fail leaves the axis parked against an engaged switch with no pull-off, so the level stays
/// asserted; without edge-arming the first block-boundary sample after `HOMING_ACTIVE` clears would latch a STALE
/// hard-limit trip that survives the soft reset and re-locks the machine into `ALARM:1` after `$X`.
///
/// Genuine over-travel is preserved: a switch that newly trips DURING a normal non-homing move is a fresh
/// not-triggered -> triggered transition and still raises the alarm at the block boundary. Only a level already
/// asserted at the last sample is suppressed.
///
/// The published `triggered` mask is purely LEVEL-based (post-`$5`), identical to [`hard_limit_alarm`], so the
/// host's `Pn:` endstop view still reflects a held switch — only the `alarm` decision is edge-armed. The returned
/// `next_armed` is the arming state the caller stores for the next call: it tracks the current level (so a press
/// arms, a release disarms and re-enables a later fresh-edge alarm). The caller SEEDS `prev_triggered` with the
/// settled levels at the arming reset points (homing start / `MOTION_RESET`) so a switch held after a cycle is
/// treated as "already known, not a new trip", while a real new edge during later motion still fires.
pub fn hard_limit_alarm_armed(
  raw_high: [bool; AXES],
  config: &LimitConfig,
  hard_limits_enabled: bool,
  homing_active: bool,
  prev_triggered: [bool; AXES],
) -> ArmedHardLimitDecision {
  let mut triggered = [false; AXES];
  let mut fresh = false;
  for axis in 0..AXES {
    triggered[axis] = limit_triggered(raw_high[axis], config);
    // A FRESH assertion is a not-triggered -> triggered transition on this axis. A persistently-held level
    // (`prev` already triggered) contributes nothing, so a parked-on switch cannot re-fire the alarm.
    fresh |= triggered[axis] && !prev_triggered[axis];
  }
  // Same gating as the level-based path: an alarm needs `$21` on and NOT mid-homing (the shared-pin rule), but
  // now keyed on a FRESH edge rather than any held level.
  let alarm = hard_limits_enabled && !homing_active && fresh;
  // The arming state carried to the next call IS the current level: a press arms (so the next held sample is
  // suppressed), a release disarms (so a subsequent re-press is once again a fresh, alarming edge).
  ArmedHardLimitDecision { triggered, alarm, next_armed: triggered }
}

/// The result of [`hard_limit_alarm_armed`]: the level-based `triggered` mask (for the `Pn:` publish), the
/// EDGE-ARMED `alarm` flag, and the `next_armed` per-axis arming state the caller stores for the next call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ArmedHardLimitDecision {
  /// Per-axis "this axis's limit switch is triggered" mask, `[X, Y, Z]` (after the `$5` invert). Purely
  /// level-based, so the published `Pn:` view still shows a held switch.
  pub triggered: [bool; AXES],
  /// `true` only when a NEW over-travel should raise `ALARM:1` now: `$21` enabled, NOT mid-homing, and at least
  /// one axis made a not-triggered -> triggered transition since the previous sample. A held level never fires.
  pub alarm: bool,
  /// The per-axis arming state to carry into the next [`hard_limit_alarm_armed`] call: the current level, so a
  /// press arms and a release disarms.
  pub next_armed: [bool; AXES],
}

/// Pack a per-axis logical-triggered array into the published `Pn:` limit bitmask: `bit0 = X`, `bit1 = Y`,
/// `bit2 = Z`. This is the SINGLE definition of that bit layout's encode side; the firmware decodes it with the
/// mirrored shift in `comms::limit_levels()`. Pass the post-`$5` logical state — typically
/// [`HardLimitDecision::triggered`] from the same sample that drove the alarm — so the published mask and the
/// alarm decision are one coherent sample of the switches rather than two independent reads.
pub fn pack_limit_mask(triggered: [bool; AXES]) -> u8 {
  let mut mask = 0u8;
  for (axis, &t) in triggered.iter().enumerate() {
    if t {
      mask |= 1 << axis;
    }
  }
  mask
}

/// Run the full single-axis homing cycle for `axis`: fast SEEK (`$25`) into the switch, PULL-OFF (`$27`) to
/// release it, slow LOCATE (`$24`) re-approach for the repeatable trigger, then a FINAL PULL-OFF so the axis
/// ends clear of the switch (research findings #3/#4). Each seek/locate walks one tick per burst through the
/// [`ProbeStepper`], sampling `limit` between bursts via the host-tested
/// [`limit_triggered`](crate::hal_traits::limit_triggered) so the `$5` sense is honored in one place.
///
/// `sink` receives the step bursts (one RMT channel on target, a recording buffer in tests); `limit` is the
/// axis's switch input. The walk is PER-AXIS independent — only `axis` ever steps, so the bin can run several
/// axes' [`home_axis`] calls concurrently on independent channels and each stops as its own switch latches
/// (research finding #10), no cross-axis Bresenham coordination needed.
///
/// Returns [`HomeAxisOutcome`] with the machine-zero step position on success. A seek or locate that reaches its
/// 1.5×-max-travel bound without the switch tripping aborts the whole cycle with [`HomingError::NoContact`]
/// (the bin raises the homing-fail alarm + reset). A sink error surfaces as [`HomingError::Sink`].
pub fn home_axis<S: StepSink, L: DigitalIn>(
  config: &HomingConfig,
  axis: usize,
  sink: &mut S,
  limit: &L,
) -> Result<HomeAxisOutcome, HomingError> {
  if axis >= AXES {
    return Err(HomingError::InvalidConfig { axis });
  }
  let search = config.search_steps(axis);
  if search == 0 {
    // A zero search bound means a non-positive max-travel / steps-per-mm on the homed axis — no valid seek.
    return Err(HomingError::InvalidConfig { axis });
  }
  let pulloff = config.pulloff_steps(axis);
  let dir_sign = config.direction[axis].sign();
  let stepper = ProbeStepper::new(config.motion);

  let seek_period = config.axis_step_period_ticks(axis, config.seek_mm_min);
  let locate_period = config.axis_step_period_ticks(axis, config.feed_mm_min);

  // Phase 1 — fast SEEK toward the switch until it trips, bounded by the 1.5× search distance.
  seek_into_switch(&stepper, config, axis, search, dir_sign, seek_period, sink, limit)?;
  // Phase 2 — PULL-OFF away from the switch to release it before the slow re-approach.
  pull_off(&stepper, config, axis, pulloff, -dir_sign, sink)?;
  // Phase 3 — slow LOCATE re-approach for the precise, repeatable trigger point. The pull-off guarantees the
  // switch is released here, so the locate starts off-switch and re-trips it slowly (the repeatable zero).
  seek_into_switch(&stepper, config, axis, search, dir_sign, locate_period, sink, limit)?;
  // Phase 4 — FINAL PULL-OFF so the axis ends clear of the switch (so a shared hard-limit pin does not
  // immediately re-raise after the cycle, research finding #4).
  pull_off(&stepper, config, axis, pulloff, -dir_sign, sink)?;

  Ok(HomeAxisOutcome { axis, zero_steps: machine_zero_steps(config, axis) })
}

/// Walk `axis` toward its switch through the [`ProbeStepper`], stopping the instant the switch trips, bounded at
/// `search` steps. A run that emits all `search` steps without a trip is a no-contact fail. Single-axis: the
/// block steps only `axis`, in the `dir_sign` direction.
#[allow(clippy::too_many_arguments)]
fn seek_into_switch<S: StepSink, L: DigitalIn>(
  stepper: &ProbeStepper,
  config: &HomingConfig,
  axis: usize,
  search: u32,
  dir_sign: i32,
  period: u32,
  sink: &mut S,
  limit: &L,
) -> Result<(), HomingError> {
  // `search` is saturated to ≤ `i32::MAX` by `mm_to_steps_safe`, so `as i32` cannot wrap and the ±1 `dir_sign`
  // product is at worst `−i32::MAX` (never the un-negatable `i32::MIN`).
  let block = single_axis_block(axis, dir_sign * search as i32);
  let limit_cfg = config.limit;
  let mut tripped = || limit_triggered(limit.is_high(), &limit_cfg);
  match stepper.run_probe(&block, period, &mut tripped, sink) {
    Ok(outcome) => {
      if outcome.triggered {
        Ok(())
      } else {
        // Reached the 1.5×-max-travel bound without the switch tripping — the no-contact homing-fail path.
        Err(HomingError::NoContact { axis })
      }
    }
    Err(_) => Err(HomingError::Sink),
  }
}

/// Back `axis` off its switch by `pulloff` steps in the `dir_sign` direction (the OPPOSITE of the seek). A
/// zero-step pull-off (`$27 = 0`) is a no-op. The pull-off is a fixed-distance move with no switch watching — it
/// drives the full distance so the switch is reliably released. Uses the slow locate period so the release is
/// gentle and the switch's mechanical hysteresis is cleared.
fn pull_off<S: StepSink>(
  stepper: &ProbeStepper,
  config: &HomingConfig,
  axis: usize,
  pulloff: u32,
  dir_sign: i32,
  sink: &mut S,
) -> Result<(), HomingError> {
  if pulloff == 0 {
    return Ok(());
  }
  // `pulloff` is saturated to ≤ `i32::MAX` by `mm_to_steps_safe`, so `as i32` and the ±1 sign cannot overflow.
  let block = single_axis_block(axis, dir_sign * pulloff as i32);
  // A pull-off never watches the switch — drive the whole distance — so the stop predicate is always false.
  let period = config.axis_step_period_ticks(axis, config.feed_mm_min);
  match stepper.run_probe(&block, period, || false, sink) {
    Ok(_) => Ok(()),
    Err(_) => Err(HomingError::Sink),
  }
}

/// Build a single-axis fixed-period [`Block`] that moves `axis` by `delta_steps` (signed). Delegates to the
/// shared [`Block::placeholder`] constructor (finding #4) so the "benign placeholder trapezoid" contract lives
/// in ONE place, shared with the probe path; here we only place the single non-zero axis delta.
fn single_axis_block(axis: usize, delta_steps: i32) -> Block {
  let mut steps = [0i32; AXES];
  steps[axis] = delta_steps;
  Block::placeholder(steps)
}

#[cfg(test)]
mod tests {
  extern crate std;
  use super::*;
  use crate::hal_traits::{DirState, StepError, StepEvent};
  use crate::motion::StepCounter;
  use core::cell::Cell;

  /// A test config: 1 µs tick, 100 steps/mm on every axis, 200 mm travel, grbl-ish seek/feed/pull-off, homing
  /// toward positive on every axis (`$23=0`), no force-origin. Pull-off 1 mm = 100 steps.
  fn test_config() -> HomingConfig {
    HomingConfig {
      steps_per_mm: [100.0; AXES],
      max_travel_mm: [200.0; AXES],
      seek_mm_min: 500.0,
      feed_mm_min: 100.0,
      pulloff_mm: 1.0,
      direction: [HomeDirection::Positive; AXES],
      force_set_origin: false,
      motion: MotionConfig::default(),
      limit: LimitConfig::default(),
    }
  }

  #[test]
  fn home_axis_full_cycle_with_position_aware_switch() {
    // A realistic switch: triggered whenever the live axis position is at or beyond the trip point in the homing
    // direction. The seek drives Z positive until pos >= trip; the pull-off drives it back below trip (releasing
    // the switch); the slow locate drives it back up to trip again; the final pull-off releases it once more.
    let config = test_config();
    let axis = 2;
    let trip_steps = 50; // the switch trips when Z reaches +50 steps.
    let position = Cell::new(0i32);

    struct PosLimit<'a> {
      position: &'a Cell<i32>,
      trip_steps: i32,
    }
    impl DigitalIn for PosLimit<'_> {
      fn is_high(&self) -> bool {
        // Triggered (HIGH under NC `$5=0`) whenever the axis is at or beyond the trip point in the + direction.
        self.position.get() >= self.trip_steps
      }
    }

    // A sink that mirrors the emitted Z steps into the shared `position` cell so the limit reacts to live motion.
    struct PosSink<'a> {
      position: &'a Cell<i32>,
      counter: StepCounter,
    }
    impl StepSink for PosSink<'_> {
      fn set_direction(&mut self, dir: DirState) -> Result<(), StepError> {
        self.counter.set_direction(dir);
        Ok(())
      }
      fn emit_burst(&mut self, ticks: &[StepEvent]) -> Result<(), StepError> {
        for ev in ticks {
          self.counter.advance(ev);
          self.position.set(self.counter.position_steps()[2]);
        }
        Ok(())
      }
    }

    let limit = PosLimit { position: &position, trip_steps };
    let mut sink = PosSink { position: &position, counter: StepCounter::new() };

    let outcome = home_axis(&config, axis, &mut sink, &limit).expect("home Z succeeds");
    assert_eq!(outcome.axis, axis);
    // Machine zero: homed positive, 1 mm pull-off at 100 steps/mm => -100 steps.
    assert_eq!(outcome.zero_steps, -100);

    // The live position must end one pull-off below the trip point: seek up to +50, pull-off down to -50, locate
    // back up to +50, final pull-off down to -50. So the final Z position is trip(50) - pulloff(100) = -50 steps.
    assert_eq!(position.get(), trip_steps - 100, "final position = trip - pulloff (one pull-off clear)");
  }

  #[test]
  fn home_axis_fails_on_no_contact_within_search_bound() {
    // A switch that never trips: the seek must abort at the 1.5× search bound with NoContact for the homed axis.
    let config = test_config();
    let position = Cell::new(0i32);
    struct NeverLimit;
    impl DigitalIn for NeverLimit {
      fn is_high(&self) -> bool {
        false // NC: low => never triggered.
      }
    }
    struct PosSink<'a> {
      position: &'a Cell<i32>,
      counter: StepCounter,
    }
    impl StepSink for PosSink<'_> {
      fn set_direction(&mut self, dir: DirState) -> Result<(), StepError> {
        self.counter.set_direction(dir);
        Ok(())
      }
      fn emit_burst(&mut self, ticks: &[StepEvent]) -> Result<(), StepError> {
        for ev in ticks {
          self.counter.advance(ev);
          self.position.set(self.counter.position_steps()[2]);
        }
        Ok(())
      }
    }
    let mut sink = PosSink { position: &position, counter: StepCounter::new() };
    let err = home_axis(&config, 2, &mut sink, &NeverLimit).expect_err("no contact must fail");
    assert_eq!(err, HomingError::NoContact { axis: 2 });
    // The seek drove the full 1.5 × 200 mm × 100 steps/mm = 30000-step search bound before giving up.
    assert_eq!(position.get(), 30_000, "seek drove the full search bound");
  }

  #[test]
  fn machine_zero_force_origin_is_zero_on_every_axis() {
    let mut config = test_config();
    config.force_set_origin = true;
    for axis in 0..AXES {
      assert_eq!(machine_zero_steps(&config, axis), 0, "force-origin zeroes axis {axis}");
    }
  }

  #[test]
  fn machine_zero_positive_home_is_minus_pulloff() {
    // Homing toward positive (Galdr default): zero sits one pull-off below the switch on the working side.
    let config = test_config(); // pull-off 1 mm, 100 steps/mm => 100 steps.
    assert_eq!(machine_zero_steps(&config, 0), -100);
  }

  #[test]
  fn machine_zero_negative_home_is_max_travel_plus_pulloff() {
    // Homing toward negative: zero sits at the far end, so the post-home position is the full travel + pull-off.
    let mut config = test_config();
    config.direction = [HomeDirection::Negative; AXES];
    // 200 mm travel × 100 steps/mm = 20000 + 100 pull-off = 20100.
    assert_eq!(machine_zero_steps(&config, 1), 20_100);
  }

  #[test]
  fn dir_invert_mask_reverses_the_seek_direction() {
    // With `$23` set for an axis (Negative), the seek must move in the − direction. A position-aware switch that
    // trips when the axis reaches −50 confirms the seek went negative.
    let mut config = test_config();
    config.direction[2] = HomeDirection::Negative;
    let position = Cell::new(0i32);
    struct PosLimit<'a> {
      position: &'a Cell<i32>,
    }
    impl DigitalIn for PosLimit<'_> {
      fn is_high(&self) -> bool {
        self.position.get() <= -50
      }
    }
    struct PosSink<'a> {
      position: &'a Cell<i32>,
      counter: StepCounter,
    }
    impl StepSink for PosSink<'_> {
      fn set_direction(&mut self, dir: DirState) -> Result<(), StepError> {
        self.counter.set_direction(dir);
        Ok(())
      }
      fn emit_burst(&mut self, ticks: &[StepEvent]) -> Result<(), StepError> {
        for ev in ticks {
          self.counter.advance(ev);
          self.position.set(self.counter.position_steps()[2]);
        }
        Ok(())
      }
    }
    let mut sink = PosSink { position: &position, counter: StepCounter::new() };
    let limit = PosLimit { position: &position };
    home_axis(&config, 2, &mut sink, &limit).expect("home Z negative");
    // Final position: trip(−50) then pull-off back toward + by 100 => +50.
    assert_eq!(position.get(), -50 + 100, "negative-home final position = trip + pulloff");
  }

  #[test]
  fn hard_limit_alarm_respects_enable_and_shared_pin_rule() {
    let cfg = LimitConfig::default(); // NC: HIGH = triggered.
    // Z switch triggered (high), `$21` enabled, NOT homing => alarm, with the Z mask bit set.
    let d = hard_limit_alarm([false, false, true], &cfg, true, false);
    assert_eq!(d.triggered, [false, false, true]);
    assert!(d.alarm, "an enabled hard limit trips the alarm when not homing");
    // Same trip, but `$21` DISABLED => no alarm (the mask still reports the switch).
    let d = hard_limit_alarm([false, false, true], &cfg, false, false);
    assert!(!d.alarm, "a disabled hard limit raises no alarm");
    assert_eq!(d.triggered, [false, false, true], "but the triggered mask still reflects the switch");
    // Same trip, `$21` enabled, but HOMING ACTIVE => suppressed (the shared-pin rule, research finding #17).
    let d = hard_limit_alarm([false, false, true], &cfg, true, true);
    assert!(!d.alarm, "a limit trip during homing must NOT raise the hard-limit alarm");
    // No switch triggered => no alarm.
    let d = hard_limit_alarm([false, false, false], &cfg, true, false);
    assert!(!d.alarm);
  }

  #[test]
  fn edge_armed_alarm_suppresses_a_held_level() {
    // The core fix: a switch that was ALREADY triggered at the previous sample (e.g. parked-on after an aborted
    // homing seek that left no pull-off) must NOT re-fire a fresh `ALARM:1`. With `prev = [_, _, true]` and the
    // level STILL `true`, no axis makes a not-triggered -> triggered transition, so `alarm` is false — while the
    // published `triggered` mask STILL reports the held switch so the host's `Pn:` endstop view stays correct.
    let cfg = LimitConfig::default(); // NC: HIGH = triggered.
    let d = hard_limit_alarm_armed([false, false, true], &cfg, true, false, [false, false, true]);
    assert!(!d.alarm, "a switch already triggered at the previous sample must not re-fire the hard-limit alarm");
    assert_eq!(d.triggered, [false, false, true], "the level mask still reports the held switch for Pn:");
    assert_eq!(d.next_armed, [false, false, true], "the carried arming state tracks the current level");
  }

  #[test]
  fn edge_armed_alarm_fires_on_a_fresh_assertion() {
    // A genuine over-travel during normal motion: the switch was NOT triggered at the previous sample and now IS.
    // That not-triggered -> triggered transition MUST raise the alarm — the fix only suppresses a persistently-held
    // level, never a real new trip.
    let cfg = LimitConfig::default();
    let d = hard_limit_alarm_armed([false, false, true], &cfg, true, false, [false, false, false]);
    assert!(d.alarm, "a fresh not-triggered -> triggered transition raises the hard-limit alarm");
    assert_eq!(d.triggered, [false, false, true]);
    assert_eq!(d.next_armed, [false, false, true]);
  }

  #[test]
  fn edge_armed_alarm_fires_on_a_new_axis_while_another_is_held() {
    // A held axis must not mask a NEW trip on a different axis: X held (prev=true) while Y freshly trips. Only Y's
    // transition counts, so the alarm fires — but the held X is not what triggered it.
    let cfg = LimitConfig::default();
    let d = hard_limit_alarm_armed([true, true, false], &cfg, true, false, [true, false, false]);
    assert!(d.alarm, "a fresh trip on Y still alarms even though X is held from before");
    assert_eq!(d.triggered, [true, true, false]);
    assert_eq!(d.next_armed, [true, true, false]);
  }

  #[test]
  fn edge_armed_alarm_respects_enable_and_shared_pin_rule() {
    // The gating rules survive the edge-arming: `$21` off and homing-active both suppress even a FRESH assertion
    // (prev all-false). A disabled limit or an expected in-cycle trip is never an over-travel alarm.
    let cfg = LimitConfig::default();
    let d = hard_limit_alarm_armed([false, false, true], &cfg, false, false, [false, false, false]);
    assert!(!d.alarm, "a disabled `$21` raises no alarm even on a fresh trip");
    assert_eq!(d.triggered, [false, false, true], "but the mask still reports the switch");
    let d = hard_limit_alarm_armed([false, false, true], &cfg, true, true, [false, false, false]);
    assert!(!d.alarm, "a fresh trip DURING homing is suppressed by the shared-pin rule");
    assert_eq!(d.triggered, [false, false, true]);
  }

  #[test]
  fn edge_armed_alarm_keeps_the_mask_level_based_under_invert() {
    // The published `triggered` mask is purely level-based (post-`$5`), independent of the arming state: under
    // `$5=1` an all-low read is all-triggered regardless of `prev`, so the host `Pn:` view never depends on edges.
    let inverted = LimitConfig { invert: true };
    let d = hard_limit_alarm_armed([false, false, false], &inverted, true, false, [true, true, true]);
    assert_eq!(d.triggered, [true, true, true], "$5=1 makes all-low read as all-triggered regardless of arming");
    assert!(!d.alarm, "all axes were already armed, so the held (inverted) level raises no fresh alarm");
    assert_eq!(d.next_armed, [true, true, true]);
  }

  #[test]
  fn edge_armed_alarm_clears_arming_on_release() {
    // A switch that RELEASES (triggered -> not) clears its arming bit, so a later RE-press is once again a fresh
    // edge that alarms. This proves the arming state is not a one-way latch: prev=true, now=false yields no alarm
    // and clears the bit; feeding that back with a new press alarms again.
    let cfg = LimitConfig::default();
    let released = hard_limit_alarm_armed([false, false, false], &cfg, true, false, [false, false, true]);
    assert!(!released.alarm, "a release is not a trip");
    assert_eq!(released.next_armed, [false, false, false], "the released axis disarms");
    let repressed = hard_limit_alarm_armed([false, false, true], &cfg, true, false, released.next_armed);
    assert!(repressed.alarm, "a fresh press after a release alarms again");
  }

  #[test]
  fn hard_limit_alarm_honors_the_dollar5_invert() {
    // With `$5=1` the sense flips: a LOW pin is now triggered. The mask + alarm must follow the inverted sense.
    let inverted = LimitConfig { invert: true };
    let d = hard_limit_alarm([false, false, false], &inverted, true, false);
    assert_eq!(d.triggered, [true, true, true], "$5=1 makes a low pin read triggered");
    assert!(d.alarm);
  }

  #[test]
  fn pack_limit_mask_uses_bit0_x_bit1_y_bit2_z() {
    // The published `Pn:` layout is bit0 = X, bit1 = Y, bit2 = Z; assert each axis maps to its own bit.
    assert_eq!(pack_limit_mask([false, false, false]), 0b000, "nothing triggered = 0");
    assert_eq!(pack_limit_mask([true, false, false]), 0b001, "X => bit0");
    assert_eq!(pack_limit_mask([false, true, false]), 0b010, "Y => bit1");
    assert_eq!(pack_limit_mask([false, false, true]), 0b100, "Z => bit2");
    assert_eq!(pack_limit_mask([true, true, true]), 0b111, "all three set");
  }

  #[test]
  fn pack_limit_mask_packs_the_hard_limit_decision_triggered_array() {
    // The single-sample coherence guarantee: the same `raw_high` that drives the alarm also drives the published
    // mask, by packing `HardLimitDecision::triggered` (the post-`$5` logical state) straight into the bitmask.
    let cfg = LimitConfig::default(); // NC: HIGH = triggered.
    let d = hard_limit_alarm([false, true, false], &cfg, true, false);
    assert_eq!(pack_limit_mask(d.triggered), 0b010, "Y trip packs to bit1");
    // Under `$5=1` the logical sense flips, and the packed mask must follow that inverted (logical) state.
    let inverted = LimitConfig { invert: true };
    let d = hard_limit_alarm([false, false, false], &inverted, true, false);
    assert_eq!(pack_limit_mask(d.triggered), 0b111, "$5=1 makes all-low read as all-triggered");
  }

  #[test]
  fn homing_groups_are_z_first_then_xy_together() {
    // The cycle order must be Z alone, then X and Y together (DOC-06 / research finding #2).
    assert_eq!(HOMING_GROUPS, &[&[2usize][..], &[0usize, 1usize][..]]);
  }

  #[test]
  fn extreme_settings_saturate_without_overflow() {
    // Non-physical `$100`/`$130`/`$27` push the mm→step products well past `i32::MAX`. The conversions must
    // SATURATE to a finite bound rather than wrapping (release) or panicking (debug `as`/`*` overflow). This
    // guards finding #3: `search_steps`, `pulloff_steps`, and the `machine_zero_steps` travel term all clamp.
    // Large-but-FINITE non-physical settings whose mm→step products exceed `i32::MAX` (≈2.1e9) without
    // overflowing `f32` to `+inf`: 1e7 steps/mm × 1e6 mm = 1e13 steps, well past the `i32` ceiling.
    let mut config = test_config();
    config.steps_per_mm = [1.0e7; AXES];
    config.max_travel_mm = [1.0e6; AXES];
    config.pulloff_mm = 1.0e6;
    // The search bound saturates to a finite, positive `u32` (no panic, no wrap to 0/negative).
    let search = config.search_steps(0);
    assert!(search > 0, "an enormous search bound saturates to a positive step count");
    // Negative-home machine zero is `travel + pulloff`; with both saturated it stays at the `i32` ceiling
    // rather than overflowing `travel as i32` or `travel + pulloff`.
    config.direction = [HomeDirection::Negative; AXES];
    let zero = machine_zero_steps(&config, 0);
    assert_eq!(zero, i32::MAX, "saturated travel + pulloff pins machine zero at i32::MAX");
    // Positive-home machine zero is `-pulloff`; a saturated pull-off stays at `-i32::MAX` (not the wrapping
    // `-i32::MIN` that would overflow), so negating it is always representable.
    config.direction = [HomeDirection::Positive; AXES];
    assert_eq!(machine_zero_steps(&config, 0), -i32::MAX, "saturated pull-off negates without overflow");
  }

  #[test]
  fn invalid_config_rejects_degenerate_axis() {
    // A zero max-travel (degenerate search bound) on the homed axis is rejected rather than seeking forever.
    let mut config = test_config();
    config.max_travel_mm[2] = 0.0;
    let position = Cell::new(0i32);
    struct NeverLimit;
    impl DigitalIn for NeverLimit {
      fn is_high(&self) -> bool {
        false
      }
    }
    struct NullSink;
    impl StepSink for NullSink {
      fn set_direction(&mut self, _dir: DirState) -> Result<(), StepError> {
        Ok(())
      }
      fn emit_burst(&mut self, _ticks: &[StepEvent]) -> Result<(), StepError> {
        Ok(())
      }
    }
    let _ = &position;
    let err = home_axis(&config, 2, &mut NullSink, &NeverLimit).expect_err("degenerate config rejected");
    assert_eq!(err, HomingError::InvalidConfig { axis: 2 });
  }
}
