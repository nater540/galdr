//! Motion model / segment generator (DOC-02).
//!
//! This is the final stage of the GCode pipeline (parser → planner → motion). The planner solves only
//! each block's optimal *entry speed*; this module realizes the full trapezoidal velocity profile at
//! execution time and coordinates the three axes in step space, emitting synchronized step ticks
//! through the [`StepSink`](crate::hal_traits::StepSink) trait. It is pure synchronous logic with no
//! runtime or esp-hal dependency, so it is fully host-testable with a recording sink.
//!
//! ## What this module computes (host-testable) vs. what the firmware bin does
//! - **Here:** classify the trapezoid (accel / cruise / decel breakpoints) from `entry_speed_sq`,
//!   `nominal_speed_sq`, and the *exit* speed (the next block's entry speed), then walk the block one
//!   DDA tick at a time. The dominant axis steps every tick; subordinate axes use Bresenham error
//!   accumulation. Each tick's instantaneous velocity gives a step *period in timer ticks*. Ticks are
//!   batched into bursts of at most [`MAX_SYMBOLS_PER_BURST`] and pushed to the sink.
//! - **Firmware bin (NOT here):** turning each [`StepEvent`] into the RMT `PulseCode` array per channel
//!   (`[level2|len2|level1|len1]`, `$0` HIGH width, `end_marker()`), the `join3` await across ch0/1/2,
//!   the dual-core `InterruptExecutor`, and the `FEED_HOLD`/stop signal checks at burst boundaries.
//!
//! ## Velocity model (grbl, squared speeds)
//! Like the planner, velocities are reasoned about as squares to keep the kinematic relation
//! `v² = v₀² + 2·a·d` `sqrt`-free in the hot path. A `sqrt` is paid once per tick to turn the
//! instantaneous `v²` into a step rate (and thus a period). All travel distances are in millimeters and
//! map to per-axis step counts via the block's geometry; the dominant axis advances one step per tick,
//! so its travel-per-tick is `millimeters / step_event_count`.
//!
//! ## Allocation
//! `#![no_std]`, allocation-free. A burst is staged in a fixed [`heapless::Vec`] of
//! [`MAX_SYMBOLS_PER_BURST`] events; `libm` supplies `sqrtf`. No `unwrap`/`expect`: every fallible path
//! returns a [`MotionError`].

use crate::hal_traits::{DirState, StepError, StepEvent, StepSink, MAX_SYMBOLS_PER_BURST};
use crate::planner::{Block, AXES};
use heapless::Vec;

/// Timing/configuration the segment generator needs that the planner does not carry. DOC-02's `$0`
/// step-pulse width and `$29` direction setup delay are firmware settings (loaded from `esp-storage` by
/// the firmware binary), not part of `PlannerConfig`, so they enter the host-testable timing math here.
/// All durations are expressed in *timer ticks* at [`tick_hz`](MotionConfig::tick_hz).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionConfig {
  /// The step-generator timer frequency in Hz (RMT source-clock ticks per second). With grbl-friendly
  /// `clk_divider = 80` on the 80 MHz APB clock this is 1_000_000 (1 tick = 1 µs); with `clk_divider = 1`
  /// it is 80_000_000 (1 tick = 12.5 ns). The period math is frequency-agnostic; tests pick 1 MHz so a
  /// tick is exactly 1 µs and hand-computed vectors are integers.
  pub tick_hz: f32,
  /// `$0` step-pulse HIGH time in timer ticks. Each step's PulseCode is HIGH for this many ticks; the
  /// remainder of the period is LOW. The minimum achievable step period is `step_pulse_ticks +
  /// min_low_ticks`, which caps the maximum step rate.
  pub step_pulse_ticks: u32,
  /// The minimum LOW time in ticks after a step pulse before the next may begin. Together with
  /// `step_pulse_ticks` it bounds the fastest step period; DOC-02 cites a 2 µs practical minimum LOW.
  pub min_low_ticks: u32,
}

impl MotionConfig {
  /// The minimum step period in ticks (fastest achievable step rate). A computed period is clamped up
  /// to this so a burst never asks the hardware for a pulse train it cannot emit.
  pub fn min_period_ticks(&self) -> u32 {
    self.step_pulse_ticks + self.min_low_ticks
  }

  /// The maximum step rate in steps/second permitted by the pulse timing. Velocities that would exceed
  /// this are clamped (the period floors at [`min_period_ticks`](MotionConfig::min_period_ticks)).
  pub fn max_step_rate_hz(&self) -> f32 {
    self.tick_hz / self.min_period_ticks() as f32
  }
}

impl Default for MotionConfig {
  /// A 1 MHz tick (1 µs) with grbl-default `$0` = 10 µs and a 2 µs minimum LOW, giving a maximum step
  /// rate of `1 / 12 µs ≈ 83.3 kHz` per axis — the DOC-02 headroom figure for PCB milling.
  fn default() -> Self {
    MotionConfig { tick_hz: 1_000_000.0, step_pulse_ticks: 10, min_low_ticks: 2 }
  }
}

/// Errors the segment generator can return. All are recoverable; the motion executor decides whether to
/// alarm or retry. The generator never panics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotionError {
  /// The configuration is unusable (a non-positive tick rate, or a zero minimum period) so no valid
  /// step timing can be produced. The firmware binary validates settings before constructing blocks; a
  /// host test can still hit this with a degenerate config.
  InvalidConfig,
  /// The downstream [`StepSink`] failed. Carries the originating [`StepError`].
  Sink(StepError),
}

impl From<StepError> for MotionError {
  fn from(err: StepError) -> Self {
    MotionError::Sink(err)
  }
}

/// The segment generator. Stateless beyond its [`MotionConfig`]; one instance can realize any number of
/// blocks. Construct it once in the motion executor and call [`run_block`](SegmentGenerator::run_block)
/// for each popped planner block, supplying the next block's entry speed as the exit speed.
pub struct SegmentGenerator {
  config: MotionConfig,
}

impl SegmentGenerator {
  /// Create a segment generator with the given timing configuration.
  pub fn new(config: MotionConfig) -> Self {
    SegmentGenerator { config }
  }

  /// Realize one planner `block` as synchronized step bursts on `sink`. `exit_speed_sq` is the squared
  /// speed (mm/s)² the block must reach at its end — the *entry speed of the next queued block*, or 0
  /// when the queue is empty or this is the final block (so the machine stops at rest). The direction is
  /// latched once from the block's step signs, then the trapezoid is walked tick-by-tick with Bresenham
  /// coordinating subordinate axes. Returns the total number of ticks emitted (one per dominant-axis
  /// step), which equals `block.step_event_count` for any moving block, or 0 for a zero-length block.
  ///
  /// ## Invariants (asserted by the host tests)
  /// - **Step conservation:** across all emitted ticks each axis steps exactly `|block.steps[axis]|`
  ///   times — Bresenham neither drops nor invents steps.
  /// - **Dominant axis every tick:** the axis with the largest step count steps on every emitted tick.
  /// - **Burst cap:** no burst exceeds [`MAX_SYMBOLS_PER_BURST`] events.
  pub fn run_block(&self, block: &Block, exit_speed_sq: f32, sink: &mut impl StepSink) -> Result<u32, MotionError> {
    // The unscaled path: a 100 % feed/rapid override (`scale = 1.0`) and no extra speed ceiling beyond the
    // pulse-timing limits (`max_speed_sq = +∞`). The realized motion is identical to the pre-override behavior,
    // so every existing caller and host test is unchanged.
    self.run_block_scaled(block, exit_speed_sq, 1.0, f32::INFINITY, sink)
  }

  /// Realize one planner `block` with a LIVE feed/rapid override applied (Phase E, DOC-08). `override_scale` is
  /// the override fraction (e.g. `1.5` for a 150 % feed override, `0.5` for a 50 % rapid override) the executor
  /// reads from the shared [`Overrides`](crate::protocol::Overrides) per block; it scales the trapezoid's
  /// entry/cruise/exit speeds so a change takes effect on the currently-executing motion WITHOUT re-planning the
  /// queue (grbl applies overrides in the stepper, not the planner). `max_speed_sq` is the squared mm/s ceiling
  /// along the block — the axis max-rate (`$110-112`) projected onto the block — so scaling UP can never exceed
  /// the configured rate limit: each scaled speed is clamped to it. `exit_speed_sq` is the next block's entry
  /// speed (also scaled here so junctions stay consistent). Acceleration is NOT scaled — grbl keeps the accel
  /// limits — so a higher override just lengthens the ramp to a higher cruise, never exceeding `max_speed_sq`.
  ///
  /// The unscaled [`run_block`](SegmentGenerator::run_block) delegates here with `override_scale = 1.0` and
  /// `max_speed_sq = +∞`, so the two share one code path and the scaling is the only added behavior.
  pub fn run_block_scaled(
    &self,
    block: &Block,
    exit_speed_sq: f32,
    override_scale: f32,
    max_speed_sq: f32,
    sink: &mut impl StepSink,
  ) -> Result<u32, MotionError> {
    if self.config.tick_hz <= 0.0 || self.config.min_period_ticks() == 0 {
      return Err(MotionError::InvalidConfig);
    }
    // A zero-length block carries no motion (the planner never enqueues one, but the executor may pop a
    // flushed/degenerate block); it is a clean no-op with no direction change and no bursts.
    if block.step_event_count == 0 {
      return Ok(0);
    }

    // Latch direction from the per-axis step signs. A zero delta keeps the axis "positive"; it never
    // steps, so the latched value is immaterial for that axis.
    let dir = DirState { dir: [block.steps[0] >= 0, block.steps[1] >= 0, block.steps[2] >= 0] };
    sink.set_direction(dir)?;

    // Apply the override to the squared speeds: a speed scales by `override_scale²` (since the stored quantity is
    // v²), then clamps to the squared max-rate ceiling so a feed boost never exceeds `$110-112`. A non-positive or
    // non-finite scale is treated as 1.0 defensively so a degenerate override cannot freeze or runaway motion.
    let scale = if override_scale.is_finite() && override_scale > 0.0 { override_scale } else { 1.0 };
    let scale_sq = scale * scale;
    let ceiling_sq = if max_speed_sq.is_finite() { max_speed_sq.max(0.0) } else { f32::INFINITY };
    let scaled = ScaledBlock {
      entry_speed_sq: clamp_speed_sq(block.entry_speed_sq * scale_sq, ceiling_sq),
      nominal_speed_sq: clamp_speed_sq(block.nominal_speed_sq * scale_sq, ceiling_sq),
    };
    let scaled_exit_sq = clamp_speed_sq(exit_speed_sq * scale_sq, ceiling_sq);

    let profile = TrapezoidProfile::plan_scaled(block, &scaled, scaled_exit_sq);
    self.emit_profile(block, &profile, sink)
  }

  /// Walk the block one dominant-axis step (one tick) at a time, computing each tick's period from the
  /// trapezoid's instantaneous velocity and deciding the subordinate-axis steps with Bresenham, batching
  /// ticks into bursts of at most [`MAX_SYMBOLS_PER_BURST`].
  fn emit_profile(&self, block: &Block, profile: &TrapezoidProfile, sink: &mut impl StepSink) -> Result<u32, MotionError> {
    let total = block.step_event_count;
    let dominant = dominant_axis(&block.steps);
    // Bresenham error accumulators per subordinate axis (the classic 2·|d| integer DDA, run in f32 only
    // for the velocity; the step decision itself is exact integer comparison so no step is ever lost).
    let mut error = [0i64; AXES];
    let abs_steps = [
      (block.steps[0].unsigned_abs()) as i64,
      (block.steps[1].unsigned_abs()) as i64,
      (block.steps[2].unsigned_abs()) as i64,
    ];
    let dom_count = abs_steps[dominant];

    // Hoist every per-block, loop-invariant timing quantity out of the hot loop: the dominant-axis travel
    // per step, the pulse-timing rate/period limits, and the tick rate. These were previously recomputed
    // (two divisions plus the min-period add) on every tick inside `period_ticks`; computing them once here
    // leaves only the per-tick sqrt + one divide in the loop. Behavior is identical — the same formula, the
    // same clamps, the same rounding — only the redundant recomputation is removed.
    let timing = StepTiming::for_block(&self.config, block, dominant);

    let mut burst: Vec<StepEvent, MAX_SYMBOLS_PER_BURST> = Vec::new();
    let mut emitted = 0u32;

    for tick in 0..total {
      // The dominant axis steps every tick. Subordinate axes use the standard Bresenham update: add the
      // axis step count to the error, and when it reaches the dominant count, step and subtract. This is
      // exact integer arithmetic, so the per-axis step total is conserved precisely.
      let mut step = [false; AXES];
      for axis in 0..AXES {
        if axis == dominant {
          step[axis] = abs_steps[axis] != 0;
          continue;
        }
        error[axis] += abs_steps[axis];
        if error[axis] >= dom_count {
          error[axis] -= dom_count;
          step[axis] = true;
        }
      }

      // Instantaneous velocity at the *midpoint* of this step gives the period. Using the midpoint of the
      // dominant-axis travel keeps the discretized ramp centered on the continuous profile (grbl uses the
      // segment's average rate); travel is measured along the block in mm.
      let traveled_mm = profile.travel_at_step(tick, total);
      let v_sq = profile.velocity_sq_at(traveled_mm);
      let period = timing.period_ticks(v_sq);
      let event = StepEvent { step, period_ticks: period };

      // `push` only fails when the Vec is full; we flush before pushing into a full burst, so the push is
      // infallible here. The explicit flush keeps every burst within the one-RMT-block symbol cap.
      if burst.is_full() {
        sink.emit_burst(&burst)?;
        burst.clear();
      }
      let _ = burst.push(event);
      emitted += 1;
    }

    if !burst.is_empty() {
      sink.emit_burst(&burst)?;
    }
    Ok(emitted)
  }
}

/// The loop-invariant timing constants for one block's step generation, computed once before the per-tick
/// loop so the hot path does not recompute them. Holds the dominant-axis travel per step, the maximum step
/// rate and minimum/slowest periods imposed by the pulse timing, and the tick rate — everything
/// [`period_ticks`](StepTiming::period_ticks) needs to turn an instantaneous velocity into a step period.
struct StepTiming {
  /// Millimeters of block travel per dominant-axis step (`block.millimeters / |dom steps|`).
  dom_mm_per_step: f32,
  /// The timer tick rate (ticks/second).
  tick_hz: f32,
  /// The maximum step rate (steps/second) the pulse timing permits.
  max_rate_hz: f32,
  /// The minimum (fastest) step period in ticks; every computed period floors at this.
  min_period_ticks: u32,
  /// The maximum (slowest) representable step period in ticks; every computed period caps at this so the
  /// LOW half of the encoded step symbol (`period − $0`) never overflows the 15-bit RMT field. A feed slow
  /// enough to want a longer period runs at this floor rate instead (see [`StepTiming::for_block`]).
  max_period_ticks: u32,
  /// The period used when velocity collapses to zero: the slowest representable step, but never faster
  /// than the minimum period.
  floor_period_ticks: u32,
}

impl StepTiming {
  /// Derive the timing constants for `block`'s dominant axis under `config`. The dominant axis has ≥ 1
  /// step for any block that reaches the generator (`step_event_count > 0`), so `dom_mm_per_step` is a
  /// well-defined finite value.
  fn for_block(config: &MotionConfig, block: &Block, dominant: usize) -> Self {
    let dom_steps = block.steps[dominant].unsigned_abs();
    let dom_mm_per_step = block.millimeters / dom_steps as f32;
    let min_period_ticks = config.min_period_ticks();
    // The slowest representable period: the step symbol's LOW half is `period − $0` in a single 15-bit RMT
    // field, so `period − $0 ≤ RMT_MAX_FIELD_LEN`, i.e. `period ≤ RMT_MAX_FIELD_LEN + $0`. Never let the
    // cap fall below the minimum period (a degenerate `$0` larger than the field would otherwise invert the
    // bounds); `max(min_period_ticks)` keeps the `[min, max]` interval well-formed. TODO(DOC-02): feeds
    // slower than `tick_hz / max_period_ticks` step rate currently floor at this rate — true sub-floor slow
    // stepping (needed for very slow Z-probing) requires multi-symbol periods, which is out of scope now.
    let max_period_ticks = (RMT_MAX_FIELD_LEN + config.step_pulse_ticks).max(min_period_ticks);
    Self {
      dom_mm_per_step,
      tick_hz: config.tick_hz,
      max_rate_hz: config.max_step_rate_hz(),
      min_period_ticks,
      max_period_ticks,
      floor_period_ticks: SLOWEST_PERIOD_TICKS.clamp(min_period_ticks, max_period_ticks),
    }
  }

  /// Convert an instantaneous squared velocity (mm/s)² into the dominant-axis step period in timer ticks.
  /// The dominant axis emits one step per `dom_mm_per_step` of travel, so its step rate is
  /// `v / dom_mm_per_step` steps/s; the period is `tick_hz / step_rate`, clamped to the pulse-timing
  /// limits. A non-positive velocity floors the rate at the slowest representable step (the period
  /// saturates rather than dividing by zero). Identical math to the previous per-tick computation, now
  /// over the pre-hoisted constants.
  fn period_ticks(&self, v_sq: f32) -> u32 {
    let v = libm::sqrtf(v_sq.max(0.0));
    let step_rate = if self.dom_mm_per_step > 0.0 { v / self.dom_mm_per_step } else { 0.0 };
    let clamped = step_rate.min(self.max_rate_hz);
    if clamped <= 0.0 {
      // Velocity collapsed to (or below) zero; emit the slowest representable step rather than diverge.
      return self.floor_period_ticks;
    }
    let period = self.tick_hz / clamped;
    // Round to the nearest whole tick, then clamp into the representable `[min, max]` interval: the floor
    // respects the fastest pulse timing, the ceiling keeps the step symbol's LOW half (`period − $0`) inside
    // the 15-bit RMT field so a very slow feed cannot overflow it and silently run faster than commanded.
    let rounded = libm::roundf(period) as u32;
    rounded.clamp(self.min_period_ticks, self.max_period_ticks)
  }
}

/// The result of running a [`ProbeStepper`] cycle: whether the expected probe edge was seen and the number of
/// dominant-axis steps actually emitted before stopping (so the executor can reconstruct the exact stop position
/// from the live step counter). `triggered` is `true` when the watched edge occurred within travel and `false`
/// when the probe reached the no-contact end of the block; the consumer turns `false` into `[PRB:..:0]` and, for
/// an alarming mode (G38.2/.4), ALARM:4/5.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ProbeOutcome {
  /// `true` if the expected edge (trigger for a toward probe, release for an away probe) was seen within travel.
  pub triggered: bool,
  /// The number of dominant-axis steps emitted before stopping (≤ the block's `step_event_count`). On a trigger
  /// this is where the probe stopped; on no-contact it equals the full block length.
  pub steps_emitted: u32,
}

/// Walks a probe `Block` one tick at a time, sampling the probe state BEFORE each tick and stopping the instant
/// the expected edge appears, so the latched stop position is the finest the architecture allows. Unlike
/// [`SegmentGenerator`] (which batches up to [`MAX_SYMBOLS_PER_BURST`] ticks per burst, so the probe could only
/// be sampled at burst boundaries), the probe stepper emits exactly ONE tick per [`StepSink::emit_burst`] call,
/// so the firmware bin's probe-watching sink can read the probe input between every single step.
///
/// grbl probes at a constant feed (no trapezoid: a probe seeks slowly and stops on contact, never ramping), so
/// each tick uses a single fixed step period derived from the probe feed — there is no entry/cruise/exit shaping.
/// The probe predicate is passed in (so the `$6`-adjusted read stays in the firmware bin / host test), keeping
/// this walker pure: it owns step generation + Bresenham coordination, not the electrical read.
///
/// ## Sampling-resolution limit (hardware boundary, DOC-02)
/// grbl samples the probe in the step ISR, stopping mid-step on contact. Galdr's RMT design transmits whole
/// bursts that cannot be preempted, so the FINEST the probe can be sampled is ONE STEP (between single-tick
/// bursts) — realized here. Over-travel past the trigger point is therefore bounded by one step plus the
/// deceleration of the in-flight single-step burst; keep the probe feed low (25–100 mm/min) to bound it, exactly
/// as `docs/tlo-offsets.md` Finding #10 prescribes. This is the documented approximation versus grbl's per-ISR
/// sampling.
pub struct ProbeStepper {
  config: MotionConfig,
}

impl ProbeStepper {
  /// Create a probe stepper with the given timing configuration (the same [`MotionConfig`] the generator uses).
  pub fn new(config: MotionConfig) -> Self {
    ProbeStepper { config }
  }

  /// Run the probe block on `sink`, sampling `is_at_stop_edge` before each tick. `is_at_stop_edge` returns `true`
  /// when the probe is at the edge that should STOP the cycle (for a TOWARD probe, "triggered"; for an AWAY
  /// probe, "released") — the firmware bin composes it from the live [`crate::hal_traits::probe_triggered`] read
  /// and the toward/away sense. The walk stops the instant the predicate is true; if it is true BEFORE the first
  /// tick the cycle stops immediately with zero steps (the firmware bin must reject a toward-probe already at the
  /// edge as ALARM:4 *before* calling this — see the consumer). Returns the [`ProbeOutcome`].
  ///
  /// `step_period_ticks` is the fixed probe-feed step period (the firmware bin derives it from the probe `F` word
  /// and `$100..102`); it is clamped into the representable `[min, max]` interval so a degenerate feed cannot
  /// produce an unencodable burst.
  pub fn run_probe<S: StepSink>(
    &self,
    block: &Block,
    step_period_ticks: u32,
    mut is_at_stop_edge: impl FnMut() -> bool,
    sink: &mut S,
  ) -> Result<ProbeOutcome, MotionError> {
    if self.config.tick_hz <= 0.0 || self.config.min_period_ticks() == 0 {
      return Err(MotionError::InvalidConfig);
    }
    let total = block.step_event_count;
    if total == 0 {
      // A zero-length probe block: sample once so an already-at-edge probe reports a trigger with no motion.
      return Ok(ProbeOutcome { triggered: is_at_stop_edge(), steps_emitted: 0 });
    }

    let dir = DirState { dir: [block.steps[0] >= 0, block.steps[1] >= 0, block.steps[2] >= 0] };
    sink.set_direction(dir)?;

    // Clamp the probe period into the representable interval, mirroring `StepTiming` (probes are slow, so the
    // period typically sits near the max; the floor guards a degenerate fast feed).
    let max_period = (RMT_MAX_FIELD_LEN + self.config.step_pulse_ticks).max(self.config.min_period_ticks());
    let period = step_period_ticks.clamp(self.config.min_period_ticks(), max_period);

    let dominant = dominant_axis(&block.steps);
    let abs_steps = [
      block.steps[0].unsigned_abs() as i64,
      block.steps[1].unsigned_abs() as i64,
      block.steps[2].unsigned_abs() as i64,
    ];
    let dom_count = abs_steps[dominant];
    let mut error = [0i64; AXES];
    let mut emitted = 0u32;

    for _ in 0..total {
      // Sample the probe BEFORE emitting the next step: if the stop edge is already present, stop without taking
      // another step so the latched position is the last position actually reached.
      if is_at_stop_edge() {
        return Ok(ProbeOutcome { triggered: true, steps_emitted: emitted });
      }
      // Decide this tick's per-axis step mask with the same exact-integer Bresenham the generator uses.
      let mut step = [false; AXES];
      for axis in 0..AXES {
        if axis == dominant {
          step[axis] = abs_steps[axis] != 0;
          continue;
        }
        error[axis] += abs_steps[axis];
        if error[axis] >= dom_count {
          error[axis] -= dom_count;
          step[axis] = true;
        }
      }
      // Emit exactly one tick so the next loop iteration can re-sample the probe after a single step.
      sink.emit_burst(&[StepEvent { step, period_ticks: period }])?;
      emitted += 1;
    }

    // Reached the end of travel without the stop edge: sample once more so a trigger on the final step still
    // counts (the loop checks BEFORE each step, so the very last position is not otherwise sampled).
    Ok(ProbeOutcome { triggered: is_at_stop_edge(), steps_emitted: emitted })
  }
}

/// Live machine position in step space, advanced one [`StepEvent`] at a time as the motion executor
/// drives the [`StepSink`], so the status reporter can publish a *live* (interpolated) MPos rather than
/// the planner's end-of-look-ahead position. It is the firmware bin's counterpart to the generator: the
/// generator decides the per-tick step mask and the executor latches one [`DirState`] per block, then
/// feeds both here so each stepping axis advances `+1` or `−1` per its latched direction.
///
/// This is pure, allocation-free, and host-tested — the executor that owns it lives in the (Xtensa-only)
/// firmware bin, so the conversion logic is decoupled here where it can run under `cargo test`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StepCounter {
  /// Signed live position in whole steps per axis, `[X, Y, Z]`. Advanced by [`advance`](StepCounter::advance);
  /// reset to the origin by [`reset`](StepCounter::reset) on a soft reset.
  position: [i32; AXES],
  /// The current per-axis step direction, latched from the active block's [`DirState`]; `true` = positive
  /// (a stepping axis advances `+1`), `false` = negative (`−1`). Set by [`set_direction`](StepCounter::set_direction).
  dir: [bool; AXES],
}

impl StepCounter {
  /// A counter at the origin with both axes latched positive. The first block's `set_direction` overrides
  /// the latched direction before any step is counted, so the initial direction is immaterial.
  pub const fn new() -> Self {
    StepCounter { position: [0; AXES], dir: [true; AXES] }
  }

  /// Latch the per-axis direction from the active block's [`DirState`], so subsequent [`advance`](StepCounter::advance)
  /// calls move each stepping axis the correct way. Called once per block, mirroring `StepSink::set_direction`.
  pub fn set_direction(&mut self, dir: DirState) {
    self.dir = dir.dir;
  }

  /// Advance the live position by one [`StepEvent`]: each axis whose `step` mask is set moves `+1` in the
  /// latched positive direction or `−1` in the negative direction. Axes that do not step are unchanged.
  /// Uses saturating arithmetic so a pathological step train can never wrap the position (it pins at
  /// `i32::MAX`/`MIN` instead), keeping the published MPos monotone rather than aliasing.
  pub fn advance(&mut self, event: &StepEvent) {
    for axis in 0..AXES {
      if event.step[axis] {
        let delta = if self.dir[axis] { 1 } else { -1 };
        self.position[axis] = self.position[axis].saturating_add(delta);
      }
    }
  }

  /// The live position in whole steps per axis, `[X, Y, Z]`.
  pub fn position_steps(&self) -> [i32; AXES] {
    self.position
  }

  /// The live position converted to millimeters per axis using `steps_per_mm` (the same `$100..102`
  /// resolution the planner rounds with). A non-positive `steps_per_mm[axis]` yields `0.0` for that axis
  /// rather than a NaN/inf, so a degenerate setting cannot poison the status report.
  pub fn position_mm(&self, steps_per_mm: &[f32; AXES]) -> [f32; AXES] {
    steps_to_mm(&self.position, steps_per_mm)
  }

  /// Reset the live position to the origin (steps zeroed) on a soft reset / pipeline reset, leaving the
  /// latched direction untouched — the next block re-latches it before stepping.
  pub fn reset(&mut self) {
    self.position = [0; AXES];
  }
}

impl Default for StepCounter {
  fn default() -> Self {
    Self::new()
  }
}

/// Convert a live step position to millimeters per axis using `steps_per_mm` (the `$100..102` resolution the
/// planner rounds with). A non-positive `steps_per_mm[axis]` yields `0.0` for that axis rather than a
/// NaN/inf, so a degenerate setting cannot poison the status report. Shared by [`StepCounter::position_mm`]
/// and the firmware bin's status reporter, which reads the live position from cross-core atomics (Finding
/// #5) and converts it here so the steps→mm math stays in one host-tested place.
pub fn steps_to_mm(position: &[i32; AXES], steps_per_mm: &[f32; AXES]) -> [f32; AXES] {
  let mut out = [0.0f32; AXES];
  for axis in 0..AXES {
    if steps_per_mm[axis] > 0.0 {
      out[axis] = position[axis] as f32 / steps_per_mm[axis];
    }
  }
  out
}

/// The maximum value an RMT pulse-length field can hold: the duration is a single 15-bit field, so one
/// LOW (or HIGH) sub-interval cannot exceed `0x7FFF` ticks. The firmware bin encodes a stepping tick as a
/// HIGH(`$0`)/LOW(`period − $0`) pair, so the binding limit on a step period is that its LOW half stay
/// within this field — see [`StepTiming::max_period_ticks`]. Single-sourced here so the host-tested clamp
/// and the on-target PulseCode encoder agree on the hardware limit (the bin re-exposes it as
/// `PulseCode::MAX_LEN`, which is the same `0x7FFF`).
pub const RMT_MAX_FIELD_LEN: u32 = 0x7FFF;

/// A ceiling on the step period (in ticks) for a velocity that has collapsed to zero, so a momentary
/// `v = 0` at the boundary of a rest-to-rest block emits a finite, very slow step instead of dividing by
/// zero or saturating the 15-bit RMT duration field. One tick of motion at this rate is harmless because
/// such ticks only occur at the extreme ends of a ramp. Chosen well below the RMT 15-bit max (32767).
const SLOWEST_PERIOD_TICKS: u32 = 30_000;

/// Split a silent (no-step) tick's full LOW period into two non-zero RMT sub-interval lengths `(a, b)` with
/// `a + b == period`, each within the 15-bit field. The firmware bin encodes a silent axis as
/// `LOW(a) / LOW(b)`: BOTH halves MUST be non-zero, because an RMT pulse code with either length zero is an
/// END MARKER that stops the channel mid-burst — which would drop every remaining step on any axis idle on a
/// tick (Finding #1, the showstopper). Splitting near the middle keeps both halves well inside the field for
/// any period the generator emits (`period ≤ RMT_MAX_FIELD_LEN + $0`, so each half ≤ ~16 K). For a degenerate
/// `period < 2` the function still returns `(1, 1)`, never a zero half; the generator never emits such a
/// period (its minimum is `step_pulse_ticks + min_low_ticks ≥ 2`), so the floor is purely defensive.
///
/// This pure split is host-tested here so the critical no-end-marker invariant is guarded under `cargo test`;
/// the bin clamps each half into the hardware field with `PulseCode::new_clamped` as a final guard.
pub fn silent_symbol_halves(period: u32) -> (u32, u32) {
  let p = period.max(2);
  let a = (p / 2).max(1);
  let b = p - a;
  (a, b)
}

/// The index of the dominant axis: the axis with the largest absolute step count. Ties resolve to the
/// lowest index (X before Y before Z), which is deterministic and matches grbl's stable selection.
fn dominant_axis(steps: &[i32; AXES]) -> usize {
  let mut best = 0usize;
  let mut best_abs = steps[0].unsigned_abs();
  for (axis, &delta) in steps.iter().enumerate().skip(1) {
    let a = delta.unsigned_abs();
    if a > best_abs {
      best_abs = a;
      best = axis;
    }
  }
  best
}

/// The override-scaled entry and nominal (cruise) squared speeds for one block (Phase E). The live feed/rapid
/// override scales these before the trapezoid is classified, while the block's geometry (acceleration, length)
/// is untouched — grbl applies the override to the velocity, not the accel limits. Kept as a tiny `Copy` struct
/// so [`TrapezoidProfile::plan_scaled`] takes the scaled speeds explicitly without mutating the planner `Block`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct ScaledBlock {
  /// The override-scaled, max-rate-clamped squared entry speed (mm/s)².
  entry_speed_sq: f32,
  /// The override-scaled, max-rate-clamped squared nominal (cruise) speed (mm/s)².
  nominal_speed_sq: f32,
}

/// Clamp a (possibly override-scaled) squared speed into `[0, ceiling_sq]`, so a feed/rapid override that
/// scales UP can never exceed the axis max-rate ceiling (`$110-112` projected onto the block). A non-finite or
/// negative input is floored at zero; an infinite ceiling (the unscaled path) leaves the value unclamped.
fn clamp_speed_sq(speed_sq: f32, ceiling_sq: f32) -> f32 {
  let floored = if speed_sq.is_finite() { speed_sq.max(0.0) } else { 0.0 };
  if ceiling_sq.is_finite() {
    floored.min(ceiling_sq)
  } else {
    floored
  }
}

/// The classified trapezoidal velocity profile for one block, expressed as the squared entry/cruise/exit
/// speeds and the two breakpoint distances (end of acceleration, start of deceleration) along the block
/// in mm. Covers every shape grbl distinguishes: cruise-only, accel-cruise, cruise-decel, full
/// trapezoid, accel-only, decel-only, and the degenerate triangle where cruise is never reached.
#[derive(Debug, Clone, Copy, PartialEq)]
struct TrapezoidProfile {
  /// Squared entry speed (mm/s)² — the planner's `block.entry_speed_sq`.
  entry_sq: f32,
  /// Squared cruise (peak) speed (mm/s)² actually reached: the block nominal, or a lower triangle peak
  /// when the block is too short to reach nominal.
  cruise_sq: f32,
  /// Squared exit speed (mm/s)² — the next block's entry speed (or 0 at a stop).
  exit_sq: f32,
  /// Twice the block acceleration magnitude (`2·a`) in mm/s², precomputed because every `v² = v₀² ± 2·a·d`
  /// evaluation in [`velocity_sq_at`](TrapezoidProfile::velocity_sq_at) needs it; hoisting it out of the
  /// per-tick velocity sampling avoids a multiply on every step.
  two_a: f32,
  /// Total block travel in mm.
  length: f32,
  /// Distance from the start at which acceleration ends (cruise begins), in mm.
  accel_end: f32,
  /// Distance from the start at which deceleration begins (cruise ends), in mm.
  decel_start: f32,
}

impl TrapezoidProfile {
  /// Classify the block's trapezoid from its (possibly override-SCALED) entry/nominal speeds plus the required
  /// exit speed, reusing the block only for its geometry (acceleration, length). Distances are derived from
  /// `v² = v₀² + 2·a·d`, rearranged to `d = (v² − v₀²) / (2·a)`. When the accel and decel ramps would overlap
  /// before reaching nominal, the block is a triangle and the peak is found from the ramp-intersection distance.
  /// The unscaled [`run_block`](SegmentGenerator::run_block) passes the block's own speeds (`scale = 1.0`) and the
  /// live-override path passes the scaled-and-clamped speeds, so the classification math lives in one place.
  fn plan_scaled(block: &Block, scaled: &ScaledBlock, exit_speed_sq: f32) -> Self {
    let entry_sq = scaled.entry_speed_sq.max(0.0);
    let nominal_sq = scaled.nominal_speed_sq.max(0.0);
    let exit_sq = exit_speed_sq.max(0.0);
    let accel = block.acceleration.max(0.0);
    let length = block.millimeters.max(0.0);

    // Distance to accelerate from entry to nominal, and distance to decelerate from nominal to exit. With
    // a non-positive acceleration (a degenerate config) the ramps collapse to zero and the block cruises.
    let two_a = 2.0 * accel;
    let (accel_dist, decel_dist) = if two_a > 0.0 {
      (((nominal_sq - entry_sq) / two_a).max(0.0), ((nominal_sq - exit_sq) / two_a).max(0.0))
    } else {
      (0.0, 0.0)
    };

    if accel_dist + decel_dist <= length {
      // The block is long enough to reach (and hold) nominal: a true trapezoid (or accel-only /
      // decel-only / cruise-only as the ramps shrink to zero).
      TrapezoidProfile {
        entry_sq,
        cruise_sq: nominal_sq,
        exit_sq,
        two_a,
        length,
        accel_end: accel_dist,
        decel_start: length - decel_dist,
      }
    } else {
      // Triangle: nominal is never reached. The accel ramp meets the decel ramp at `accel_end`, where the
      // velocity peaks. Solving `entry² + 2a·x = exit² + 2a·(L − x)` gives the crossover distance below;
      // the peak speed² follows from the accel ramp. Clamp into `[0, length]` for f32 round-off safety.
      let accel_end = if two_a > 0.0 {
        (((exit_sq - entry_sq) / two_a + length) * 0.5).clamp(0.0, length)
      } else {
        0.0
      };
      let peak_sq = (entry_sq + two_a * accel_end).max(entry_sq.max(exit_sq));
      TrapezoidProfile {
        entry_sq,
        cruise_sq: peak_sq,
        exit_sq,
        two_a,
        length,
        accel_end,
        // No cruise plateau: deceleration begins immediately where acceleration ends.
        decel_start: accel_end,
      }
    }
  }

  /// The travel in mm from the block start to the midpoint of dominant-axis step `tick` (0-based) of
  /// `total`. Using the step midpoint centers the discrete velocity sample on the continuous ramp, so
  /// the realized profile tracks the planner's trapezoid rather than lagging half a step behind.
  fn travel_at_step(&self, tick: u32, total: u32) -> f32 {
    // `total` ≥ 1 here (callers guard `step_event_count > 0`); the midpoint of step `tick` is at
    // fractional progress `(tick + 0.5) / total` along the block length.
    let frac = (tick as f32 + 0.5) / total as f32;
    frac * self.length
  }

  /// The squared instantaneous velocity (mm/s)² at distance `d` mm from the block start, following the
  /// trapezoid: accelerate from `entry_sq` until `accel_end`, hold `cruise_sq` until `decel_start`, then
  /// decelerate toward `exit_sq`. Each region uses `v² = v₀² ± 2·a·Δd`. The result is clamped to the
  /// region's bounds so f32 round-off cannot push a sample past cruise or below exit.
  fn velocity_sq_at(&self, d: f32) -> f32 {
    // `two_a` is precomputed in `plan`; the per-step velocity sampling reuses it rather than recomputing
    // `2·a` on every call.
    if d <= self.accel_end {
      // Accelerating: rises from entry, capped at the cruise peak.
      (self.entry_sq + self.two_a * d).min(self.cruise_sq).max(0.0)
    } else if d <= self.decel_start {
      // Cruising at the peak speed.
      self.cruise_sq
    } else {
      // Decelerating: falls from cruise toward exit over the remaining travel, floored at exit.
      let remaining = (self.length - d).max(0.0);
      (self.exit_sq + self.two_a * remaining).min(self.cruise_sq).max(self.exit_sq.min(self.cruise_sq))
    }
  }
}

#[cfg(test)]
mod tests {
  // firmware-core is `#![no_std]`; the host test build links `std` so the recording sink can use a
  // growable `Vec`. Production code stays allocation-free (`heapless`); only this test module uses `std`.
  extern crate std;
  use super::*;
  use std::vec::Vec;

  /// A recording [`StepSink`]: it captures every direction latch and every emitted burst so tests can
  /// assert per-axis step conservation, dominant-axis stepping, burst sizing, and period progression.
  /// It never fails, mirroring the on-target RMT sink's success path.
  struct RecordingSink {
    /// Every direction latched, in order. One entry per `set_direction` call.
    directions: Vec<DirState>,
    /// Every burst emitted, in order, as a vector of its ticks. Lengths must each be ≤ cap.
    bursts: Vec<Vec<StepEvent>>,
  }

  impl RecordingSink {
    fn new() -> Self {
      RecordingSink { directions: Vec::new(), bursts: Vec::new() }
    }

    /// Flatten every emitted tick across all bursts, in emission order.
    fn all_ticks(&self) -> Vec<StepEvent> {
      let mut out = Vec::new();
      for burst in &self.bursts {
        for ev in burst {
          out.push(*ev);
        }
      }
      out
    }

    /// Per-axis total step count across every emitted tick.
    fn step_totals(&self) -> [u32; AXES] {
      let mut totals = [0u32; AXES];
      for ev in self.all_ticks() {
        for (axis, total) in totals.iter_mut().enumerate() {
          if ev.step[axis] {
            *total += 1;
          }
        }
      }
      totals
    }
  }

  impl StepSink for RecordingSink {
    fn set_direction(&mut self, dir: DirState) -> Result<(), StepError> {
      self.directions.push(dir);
      Ok(())
    }

    fn emit_burst(&mut self, ticks: &[StepEvent]) -> Result<(), StepError> {
      if ticks.len() > MAX_SYMBOLS_PER_BURST {
        return Err(StepError::BurstTooLong);
      }
      self.bursts.push(ticks.to_vec());
      Ok(())
    }
  }

  /// A [`StepSink`] that fails its first `emit_burst`, to prove error propagation through the generator.
  struct FailingSink;

  impl StepSink for FailingSink {
    fn set_direction(&mut self, _dir: DirState) -> Result<(), StepError> {
      Ok(())
    }

    fn emit_burst(&mut self, _ticks: &[StepEvent]) -> Result<(), StepError> {
      Err(StepError::Transport)
    }
  }

  /// A 1 MHz / 1 µs tick config with a 10 µs pulse and 2 µs minimum LOW (min period 12 µs → max rate
  /// ≈ 83.3 kHz). Integer ticks make period vectors exact.
  fn test_config() -> MotionConfig {
    MotionConfig { tick_hz: 1_000_000.0, step_pulse_ticks: 10, min_low_ticks: 2 }
  }

  /// Build a block directly (bypassing the planner) so tests pin exact step deltas and squared speeds.
  /// `length_mm` is the Euclidean travel; the unit vector is derived from the step deltas for realism.
  fn make_block(steps: [i32; AXES], length_mm: f32, accel: f32, entry_sq: f32, nominal_sq: f32) -> Block {
    let sec = steps.iter().map(|s| s.unsigned_abs()).max().unwrap_or(0);
    let norm = {
      let sumsq = steps.iter().map(|&s| (s as f32) * (s as f32)).sum::<f32>();
      libm::sqrtf(sumsq).max(1.0)
    };
    Block {
      steps,
      step_event_count: sec,
      unit_vec: [steps[0] as f32 / norm, steps[1] as f32 / norm, steps[2] as f32 / norm],
      millimeters: length_mm,
      acceleration: accel,
      nominal_speed_sq: nominal_sq,
      max_entry_speed_sq: nominal_sq,
      entry_speed_sq: entry_sq,
      rapid: false,
      jog: false,
    }
  }

  // ---- Step conservation & Bresenham coordination -----------------------------------------------

  #[test]
  fn multi_axis_block_conserves_steps_exactly() {
    // A 3-axis block with distinct counts. Bresenham must emit exactly |steps[axis]| per axis and one
    // tick per dominant-axis step — never dropping or adding a step. This is the load-bearing invariant.
    let generator = SegmentGenerator::new(test_config());
    let block = make_block([400, 300, 100], 5.0, 100.0, 0.0, 400.0);
    let mut sink = RecordingSink::new();
    let emitted = generator.run_block(&block, 0.0, &mut sink).expect("runs");
    assert_eq!(emitted, 400, "one tick per dominant-axis (X) step");
    assert_eq!(sink.step_totals(), [400, 300, 100], "each axis steps exactly its delta");
  }

  #[test]
  fn dominant_axis_steps_on_every_tick() {
    // Y dominates here (600 > 250 > 0). Every emitted tick must carry a Y step; X steps on a subset.
    let generator = SegmentGenerator::new(test_config());
    let block = make_block([250, 600, 0], 6.0, 100.0, 0.0, 400.0);
    let mut sink = RecordingSink::new();
    generator.run_block(&block, 0.0, &mut sink).expect("runs");
    let ticks = sink.all_ticks();
    assert_eq!(ticks.len(), 600);
    assert!(ticks.iter().all(|ev| ev.step[1]), "dominant axis Y steps every tick");
    assert!(!ticks.iter().all(|ev| ev.step[0]), "subordinate X does not step every tick");
    assert_eq!(sink.step_totals(), [250, 600, 0]);
  }

  #[test]
  fn negative_deltas_latch_negative_direction_and_conserve_steps() {
    // Negative step deltas must latch a negative DIR for those axes and still step |delta| times.
    let generator = SegmentGenerator::new(test_config());
    let block = make_block([-300, 200, -50], 4.0, 100.0, 0.0, 400.0);
    let mut sink = RecordingSink::new();
    generator.run_block(&block, 0.0, &mut sink).expect("runs");
    assert_eq!(sink.directions.len(), 1, "direction latched once per block");
    assert_eq!(sink.directions[0].dir, [false, true, false], "signs map to DIR per axis");
    assert_eq!(sink.step_totals(), [300, 200, 50], "magnitudes conserved regardless of sign");
  }

  #[test]
  fn pure_diagonal_steps_both_axes_every_tick() {
    // Equal X and Y counts: a 45° line. Bresenham must step both axes on every tick (1:1 ratio).
    let generator = SegmentGenerator::new(test_config());
    let block = make_block([100, 100, 0], 1.414, 100.0, 0.0, 400.0);
    let mut sink = RecordingSink::new();
    generator.run_block(&block, 0.0, &mut sink).expect("runs");
    let ticks = sink.all_ticks();
    assert_eq!(ticks.len(), 100);
    assert!(ticks.iter().all(|ev| ev.step[0] && ev.step[1]), "1:1 diagonal steps both axes each tick");
  }

  // ---- Burst sizing / ≤48-symbol cap ------------------------------------------------------------

  #[test]
  fn long_block_splits_into_capped_bursts() {
    // 500 dominant steps must fan out into ceil(500/48) = 11 bursts, none exceeding the 48-symbol cap, and
    // the tick total must still equal the step count exactly.
    let generator = SegmentGenerator::new(test_config());
    let block = make_block([500, 0, 0], 5.0, 100.0, 0.0, 400.0);
    let mut sink = RecordingSink::new();
    let emitted = generator.run_block(&block, 0.0, &mut sink).expect("runs");
    assert_eq!(emitted, 500);
    assert!(sink.bursts.iter().all(|b| b.len() <= MAX_SYMBOLS_PER_BURST), "no burst exceeds the cap");
    assert_eq!(sink.bursts.len(), 500_usize.div_ceil(MAX_SYMBOLS_PER_BURST));
    let total: usize = sink.bursts.iter().map(|b| b.len()).sum();
    assert_eq!(total, 500, "tick total across bursts is conserved");
  }

  #[test]
  fn exactly_one_full_burst_emits_single_burst() {
    // Precisely 48 steps must produce one full burst, no empty trailing burst.
    let generator = SegmentGenerator::new(test_config());
    let block = make_block([MAX_SYMBOLS_PER_BURST as i32, 0, 0], 1.0, 100.0, 200.0, 200.0);
    let mut sink = RecordingSink::new();
    generator.run_block(&block, 200.0, &mut sink).expect("runs");
    assert_eq!(sink.bursts.len(), 1);
    assert_eq!(sink.bursts[0].len(), MAX_SYMBOLS_PER_BURST);
  }

  // ---- Trapezoid shaping: accel / decel / cruise / full / triangle ------------------------------

  /// The instantaneous velocity² at each tick midpoint, recovered from the emitted period: a sink only
  /// sees periods, so we invert the period→velocity math the same way the firmware would for a sanity
  /// check. Returns one velocity per tick in emission order.
  fn velocities_from_periods(sink: &RecordingSink, block: &Block, cfg: &MotionConfig) -> Vec<f32> {
    let dom = dominant_axis(&block.steps);
    let dom_mm_per_step = block.millimeters / block.steps[dom].unsigned_abs() as f32;
    sink
      .all_ticks()
      .iter()
      .map(|ev| (cfg.tick_hz / ev.period_ticks as f32) * dom_mm_per_step)
      .collect()
  }

  #[test]
  fn accel_only_profile_velocity_rises_monotonically() {
    // Entry 0, exit == nominal: the block only accelerates (then holds at the end). Velocity must be
    // non-decreasing across ticks.
    let cfg = test_config();
    let generator = SegmentGenerator::new(cfg);
    // Long enough to reach nominal: nominal v = 20 mm/s → v²=400; accel 100; accel_dist = 400/200 = 2 mm.
    let block = make_block([1000, 0, 0], 10.0, 100.0, 0.0, 400.0);
    let mut sink = RecordingSink::new();
    generator.run_block(&block, 400.0, &mut sink).expect("runs");
    let v = velocities_from_periods(&sink, &block, &cfg);
    for w in v.windows(2) {
      assert!(w[1] >= w[0] - 0.5, "velocity must not fall in an accel-only block: {} -> {}", w[0], w[1]);
    }
    // The final velocity should be near nominal (20 mm/s) within the period-rounding tolerance.
    assert!((v[v.len() - 1] - 20.0).abs() < 1.5, "ends near nominal, got {}", v[v.len() - 1]);
  }

  #[test]
  fn decel_only_profile_velocity_falls_monotonically() {
    // Entry == nominal, exit 0: the block only decelerates. Velocity must be non-increasing.
    let cfg = test_config();
    let generator = SegmentGenerator::new(cfg);
    let block = make_block([1000, 0, 0], 10.0, 100.0, 400.0, 400.0);
    let mut sink = RecordingSink::new();
    generator.run_block(&block, 0.0, &mut sink).expect("runs");
    let v = velocities_from_periods(&sink, &block, &cfg);
    for w in v.windows(2) {
      assert!(w[1] <= w[0] + 0.5, "velocity must not rise in a decel-only block: {} -> {}", w[0], w[1]);
    }
    assert!((v[0] - 20.0).abs() < 1.5, "starts near nominal, got {}", v[0]);
  }

  #[test]
  fn full_trapezoid_rises_holds_then_falls() {
    // Rest → cruise → rest over a long block: the velocity rises, plateaus near nominal, then falls. We
    // check the peak is near nominal and the endpoints are well below it (symmetric ramp).
    let cfg = test_config();
    let generator = SegmentGenerator::new(cfg);
    // nominal v = 20 mm/s (v²=400), accel 100, accel/decel dist = 2 mm each; 10 mm block leaves a 6 mm
    // cruise plateau, so nominal is genuinely reached and held.
    let block = make_block([1000, 0, 0], 10.0, 100.0, 0.0, 400.0);
    let mut sink = RecordingSink::new();
    generator.run_block(&block, 0.0, &mut sink).expect("runs");
    let v = velocities_from_periods(&sink, &block, &cfg);
    let peak = v.iter().cloned().fold(0.0_f32, f32::max);
    assert!((peak - 20.0).abs() < 1.5, "peak reaches nominal, got {peak}");
    assert!(v[0] < peak * 0.6, "starts well below peak (rest), got {}", v[0]);
    assert!(v[v.len() - 1] < peak * 0.6, "ends well below peak (rest), got {}", v[v.len() - 1]);
  }

  #[test]
  fn cruise_only_profile_holds_constant_velocity() {
    // Entry == nominal == exit: a continuous-motion block in the middle of a long straight line. Every
    // tick should carry essentially the same period (constant velocity), so periods are near-uniform.
    let cfg = test_config();
    let generator = SegmentGenerator::new(cfg);
    let block = make_block([1000, 0, 0], 10.0, 100.0, 400.0, 400.0);
    let mut sink = RecordingSink::new();
    generator.run_block(&block, 400.0, &mut sink).expect("runs");
    let periods: Vec<u32> = sink.all_ticks().iter().map(|e| e.period_ticks).collect();
    let first = periods[0];
    assert!(periods.iter().all(|&p| p.abs_diff(first) <= 1), "cruise holds a constant period ~{first}");
  }

  #[test]
  fn triangle_profile_peaks_below_nominal_when_block_is_short() {
    // A short rest-to-rest block that cannot reach nominal: the peak is the ramp-intersection speed,
    // strictly below nominal. nominal v=100 (v²=10000) but the block is only 1 mm with accel 100, so the
    // reachable peak² = a·L = 100·1 = 100 → v_peak = 10 mm/s, far under nominal.
    let cfg = test_config();
    let generator = SegmentGenerator::new(cfg);
    let block = make_block([100, 0, 0], 1.0, 100.0, 0.0, 10_000.0);
    let mut sink = RecordingSink::new();
    generator.run_block(&block, 0.0, &mut sink).expect("runs");
    let v = velocities_from_periods(&sink, &block, &cfg);
    let peak = v.iter().cloned().fold(0.0_f32, f32::max);
    assert!(peak < 30.0, "triangle peak stays well below nominal (100 mm/s), got {peak}");
    assert!(peak > 5.0, "triangle still accelerates meaningfully, got {peak}");
  }

  // ---- Exit speed shapes the deceleration ramp --------------------------------------------------

  #[test]
  fn exit_speed_changes_final_velocity() {
    // Same block, two different exit speeds (next block's entry). A non-zero exit must leave the final
    // tick moving faster than a zero exit — the ramp is shaped by the supplied exit speed.
    let cfg = test_config();
    let generator = SegmentGenerator::new(cfg);
    let block = make_block([1000, 0, 0], 10.0, 100.0, 400.0, 400.0);

    let mut stop = RecordingSink::new();
    generator.run_block(&block, 0.0, &mut stop).expect("runs");
    let v_stop = velocities_from_periods(&stop, &block, &cfg);

    let mut keep = RecordingSink::new();
    generator.run_block(&block, 400.0, &mut keep).expect("runs");
    let v_keep = velocities_from_periods(&keep, &block, &cfg);

    assert!(
      v_keep[v_keep.len() - 1] > v_stop[v_stop.len() - 1] + 2.0,
      "non-zero exit keeps speed up: keep {} vs stop {}",
      v_keep[v_keep.len() - 1],
      v_stop[v_stop.len() - 1]
    );
  }

  // ---- Period math from velocity ---------------------------------------------------------------

  #[test]
  fn period_math_matches_known_velocity() {
    // Cruise at exactly 10 mm/s with 100 steps/mm-equivalent geometry: 1000 dominant steps over 10 mm is
    // 100 steps/mm, so at 10 mm/s the step rate is 1000 steps/s → period 1000 µs = 1000 ticks at 1 MHz.
    let cfg = test_config();
    let generator = SegmentGenerator::new(cfg);
    // v=10 mm/s → v²=100; entry==nominal==exit keeps it constant so the midpoint sample is exactly 100.
    let block = make_block([1000, 0, 0], 10.0, 100.0, 100.0, 100.0);
    let mut sink = RecordingSink::new();
    generator.run_block(&block, 100.0, &mut sink).expect("runs");
    let p = sink.all_ticks()[0].period_ticks;
    assert!(p.abs_diff(1000) <= 1, "10 mm/s at 100 steps/mm is a 1000-tick period, got {p}");
  }

  #[test]
  fn period_floors_at_min_period_for_excessive_velocity() {
    // A velocity that exceeds the max step rate must clamp the period to the minimum (12 ticks here),
    // never below the pulse timing. Request a huge cruise speed on a fine-resolution block.
    let cfg = test_config();
    let generator = SegmentGenerator::new(cfg);
    // 10000 steps over 1 mm = 10000 steps/mm; nominal v=1000 mm/s would be 10 MHz steps, far over the
    // 83.3 kHz ceiling, so every cruise tick clamps to the 12-tick minimum period.
    let block = make_block([10_000, 0, 0], 1.0, 1.0e9, 1.0e6, 1.0e6);
    let mut sink = RecordingSink::new();
    generator.run_block(&block, 1.0e6, &mut sink).expect("runs");
    let min = cfg.min_period_ticks();
    assert!(sink.all_ticks().iter().all(|e| e.period_ticks >= min), "never faster than min period");
    assert_eq!(sink.all_ticks()[0].period_ticks, min, "saturates at the min period under the cap");
  }

  #[test]
  fn period_caps_at_max_representable_for_a_very_slow_feed() {
    // A feed slow enough to want a period longer than the 15-bit RMT field must clamp to the representable
    // maximum, never overflow it: the step symbol's LOW half is `period − $0`, so the period must stay
    // within `RMT_MAX_FIELD_LEN + $0`. Without the cap a tiny velocity would round to a huge period that, on
    // target, truncates to a 15-bit value and steps FASTER than commanded — a silent over-speed (Finding #2).
    let cfg = test_config();
    let generator = SegmentGenerator::new(cfg);
    // 100 steps over a very long 1000 mm block at a crawling nominal speed (v = 0.01 mm/s, v² = 1e-4). The
    // dominant-axis step rate is far below `tick_hz / max_period_ticks`, so every period saturates the cap.
    let block = make_block([100, 0, 0], 1000.0, 1.0, 1.0e-4, 1.0e-4);
    let mut sink = RecordingSink::new();
    generator.run_block(&block, 1.0e-4, &mut sink).expect("runs");
    let max_period = RMT_MAX_FIELD_LEN + cfg.step_pulse_ticks;
    assert!(
      sink.all_ticks().iter().all(|e| e.period_ticks <= max_period),
      "no period exceeds the representable max ({max_period} ticks)",
    );
    // The LOW half of the encoded step symbol (`period − $0`) must fit the 15-bit field on every tick.
    assert!(
      sink.all_ticks().iter().all(|e| e.period_ticks - cfg.step_pulse_ticks <= RMT_MAX_FIELD_LEN),
      "the step symbol's LOW half stays within the 15-bit RMT field",
    );
    // At this crawl the period genuinely saturates the cap (the floor would be unreachable otherwise).
    assert_eq!(sink.all_ticks()[0].period_ticks, max_period, "saturates at the max period for a slow feed");
  }

  // ---- Degenerate cases -------------------------------------------------------------------------

  #[test]
  fn zero_length_block_emits_nothing() {
    // A block with no dominant steps is a no-op: no direction latch, no bursts, zero ticks.
    let generator = SegmentGenerator::new(test_config());
    let block = make_block([0, 0, 0], 0.0, 100.0, 0.0, 0.0);
    let mut sink = RecordingSink::new();
    let emitted = generator.run_block(&block, 0.0, &mut sink).expect("runs");
    assert_eq!(emitted, 0);
    assert!(sink.bursts.is_empty(), "no bursts for a zero-length block");
    assert!(sink.directions.is_empty(), "no direction latch for a no-op block");
  }

  #[test]
  fn single_step_block_emits_exactly_one_tick() {
    // The smallest moving block: one step on one axis. Exactly one tick, one burst, that axis steps once.
    let generator = SegmentGenerator::new(test_config());
    let block = make_block([1, 0, 0], 0.01, 100.0, 0.0, 400.0);
    let mut sink = RecordingSink::new();
    let emitted = generator.run_block(&block, 0.0, &mut sink).expect("runs");
    assert_eq!(emitted, 1);
    assert_eq!(sink.bursts.len(), 1);
    assert_eq!(sink.bursts[0].len(), 1);
    assert_eq!(sink.step_totals(), [1, 0, 0]);
  }

  // ---- Config validation & error propagation ----------------------------------------------------

  #[test]
  fn invalid_config_is_rejected() {
    // A non-positive tick rate cannot produce valid timing; the generator reports it rather than panic.
    let generator = SegmentGenerator::new(MotionConfig { tick_hz: 0.0, step_pulse_ticks: 10, min_low_ticks: 2 });
    let block = make_block([100, 0, 0], 1.0, 100.0, 0.0, 400.0);
    let mut sink = RecordingSink::new();
    assert_eq!(generator.run_block(&block, 0.0, &mut sink), Err(MotionError::InvalidConfig));
  }

  #[test]
  fn sink_error_propagates_as_motion_error() {
    // A failing sink surfaces as MotionError::Sink, never a panic — firmware-core's no-unwrap contract.
    let generator = SegmentGenerator::new(test_config());
    let block = make_block([100, 0, 0], 1.0, 100.0, 0.0, 400.0);
    let mut sink = FailingSink;
    assert_eq!(generator.run_block(&block, 0.0, &mut sink), Err(MotionError::Sink(StepError::Transport)));
  }

  // ---- Integration: planner block flows straight through ----------------------------------------

  #[test]
  fn planner_block_realizes_through_the_generator() {
    // Drive a real planner-produced block through the generator to confirm the pipeline composes: the
    // planner solves a lone block (entry 0, exit 0), and the generator conserves its steps exactly.
    use crate::gcode::{AxisWords, DistanceMode, PlannerCommand, Units};
    use crate::planner::{Planner, PlannerConfig};
    let mut planner = Planner::new(PlannerConfig {
      steps_per_mm: [100.0; AXES],
      max_rate_mm_min: [6000.0; AXES],
      accel_mm_s2: [1000.0; AXES],
      junction_deviation_mm: 0.01,
      arc_tolerance_mm: 0.002,
    });
    planner
      .plan_command(&PlannerCommand::Move {
        rapid: false,
        axes: AxisWords { x: Some(3.0), y: Some(4.0), z: None },
        units: Units::Millimeter,
        distance: DistanceMode::Absolute,
        feed: 600.0,
        machine_coords: false,
      })
      .expect("queued");
    let block = *planner.peek_block().expect("a block");
    // A lone block stops at rest, so exit speed is 0 (no next block).
    let generator = SegmentGenerator::new(test_config());
    let mut sink = RecordingSink::new();
    let emitted = generator.run_block(&block, 0.0, &mut sink).expect("runs");
    // X3 Y4 at 100 steps/mm: 300 X steps, 400 Y steps; Y dominates so 400 ticks, steps conserved.
    assert_eq!(emitted, 400);
    assert_eq!(sink.step_totals(), [300, 400, 0]);
  }

  // ---- Phase E: feed/rapid override scaling -----------------------------------------------------

  #[test]
  fn override_scale_one_matches_unscaled() {
    // `run_block_scaled` with scale 1.0 and an infinite ceiling must be byte-for-byte identical to `run_block`.
    let generator = SegmentGenerator::new(test_config());
    let block = make_block([1000, 0, 0], 10.0, 100.0, 400.0, 400.0);
    let mut plain = RecordingSink::new();
    generator.run_block(&block, 400.0, &mut plain).expect("runs");
    let mut scaled = RecordingSink::new();
    generator.run_block_scaled(&block, 400.0, 1.0, f32::INFINITY, &mut scaled).expect("runs");
    assert_eq!(plain.all_ticks(), scaled.all_ticks(), "scale 1.0 is identical to the unscaled path");
  }

  #[test]
  fn feed_override_above_100_speeds_up_cruise_until_max_rate() {
    // A cruise-only block at v=20 mm/s (v²=400). A 150% feed override should raise the cruise to ~30 mm/s when
    // the max-rate ceiling allows it (period shrinks), but a ceiling at 20 mm/s (v²=400) must hold it at 20.
    let cfg = test_config();
    let generator = SegmentGenerator::new(cfg);
    let block = make_block([1000, 0, 0], 10.0, 100.0, 400.0, 400.0);

    // 150% with a generous ceiling (v=40 mm/s, v²=1600): cruise should rise toward 30 mm/s.
    let mut up = RecordingSink::new();
    generator.run_block_scaled(&block, 400.0, 1.5, 1600.0, &mut up).expect("runs");
    let v_up = velocities_from_periods(&up, &block, &cfg);
    let peak_up = v_up.iter().cloned().fold(0.0_f32, f32::max);
    assert!((peak_up - 30.0).abs() < 2.0, "150% feed raises cruise to ~30 mm/s, got {peak_up}");

    // 150% but with the ceiling AT the original 20 mm/s (v²=400): the scaled feed is clamped, cruise stays ~20.
    let mut capped = RecordingSink::new();
    generator.run_block_scaled(&block, 400.0, 1.5, 400.0, &mut capped).expect("runs");
    let v_cap = velocities_from_periods(&capped, &block, &cfg);
    let peak_cap = v_cap.iter().cloned().fold(0.0_f32, f32::max);
    assert!((peak_cap - 20.0).abs() < 2.0, "feed clamps to the max-rate ceiling (~20 mm/s), got {peak_cap}");
  }

  #[test]
  fn feed_override_below_100_slows_down_cruise() {
    // A 50% feed override on a v=20 mm/s cruise block must halve the cruise to ~10 mm/s (period doubles).
    let cfg = test_config();
    let generator = SegmentGenerator::new(cfg);
    let block = make_block([1000, 0, 0], 10.0, 100.0, 400.0, 400.0);
    let mut sink = RecordingSink::new();
    generator.run_block_scaled(&block, 400.0, 0.5, f32::INFINITY, &mut sink).expect("runs");
    let v = velocities_from_periods(&sink, &block, &cfg);
    let peak = v.iter().cloned().fold(0.0_f32, f32::max);
    assert!((peak - 10.0).abs() < 1.5, "50% feed halves cruise to ~10 mm/s, got {peak}");
  }

  #[test]
  fn override_scaling_conserves_steps() {
    // Scaling the feed must never drop or invent a step — only the periods change, not the step counts.
    let generator = SegmentGenerator::new(test_config());
    let block = make_block([400, 300, 100], 5.0, 100.0, 0.0, 400.0);
    let mut sink = RecordingSink::new();
    let emitted = generator.run_block_scaled(&block, 0.0, 1.75, f32::INFINITY, &mut sink).expect("runs");
    assert_eq!(emitted, 400, "one tick per dominant-axis step regardless of the override");
    assert_eq!(sink.step_totals(), [400, 300, 100], "each axis steps exactly its delta under scaling");
  }

  #[test]
  fn degenerate_override_scale_falls_back_to_unscaled() {
    // A non-positive / non-finite override scale must not freeze or runaway motion: it is treated as 1.0.
    let generator = SegmentGenerator::new(test_config());
    let block = make_block([500, 0, 0], 5.0, 100.0, 400.0, 400.0);
    let mut plain = RecordingSink::new();
    generator.run_block(&block, 400.0, &mut plain).expect("runs");
    for bad in [0.0_f32, -1.0, f32::NAN, f32::INFINITY] {
      let mut sink = RecordingSink::new();
      generator.run_block_scaled(&block, 400.0, bad, f32::INFINITY, &mut sink).expect("runs");
      assert_eq!(sink.all_ticks(), plain.all_ticks(), "degenerate scale {bad} falls back to unscaled");
    }
  }

  // ---- StepCounter: live position tracking ------------------------------------------------------

  /// A single positive event advances each stepping axis by `+1` and leaves silent axes untouched.
  #[test]
  fn step_counter_advances_positive_axes() {
    let mut counter = StepCounter::new();
    counter.set_direction(DirState { dir: [true, true, true] });
    counter.advance(&StepEvent { step: [true, false, true], period_ticks: 12 });
    assert_eq!(counter.position_steps(), [1, 0, 1]);
  }

  /// A negative latched direction makes a stepping axis advance `−1`; mixed signs are honored per axis.
  #[test]
  fn step_counter_honors_latched_direction_sign() {
    let mut counter = StepCounter::new();
    counter.set_direction(DirState { dir: [false, true, false] });
    counter.advance(&StepEvent { step: [true, true, true], period_ticks: 12 });
    assert_eq!(counter.position_steps(), [-1, 1, -1]);
  }

  /// Re-latching direction mid-stream (as the executor does once per block) changes the sign of
  /// subsequent steps without disturbing the accumulated position.
  #[test]
  fn step_counter_relatches_direction_per_block() {
    let mut counter = StepCounter::new();
    counter.set_direction(DirState { dir: [true, true, true] });
    for _ in 0..5 {
      counter.advance(&StepEvent { step: [true, false, false], period_ticks: 12 });
    }
    assert_eq!(counter.position_steps(), [5, 0, 0]);
    // Next block reverses X: three steps back toward the origin.
    counter.set_direction(DirState { dir: [false, true, true] });
    for _ in 0..3 {
      counter.advance(&StepEvent { step: [true, false, false], period_ticks: 12 });
    }
    assert_eq!(counter.position_steps(), [2, 0, 0]);
  }

  /// `position_mm` divides the live step count by `steps_per_mm` per axis (the same resolution the
  /// planner rounds with), and a non-positive `steps_per_mm` yields a finite `0.0`, never NaN/inf.
  #[test]
  fn step_counter_converts_steps_to_mm() {
    let mut counter = StepCounter::new();
    counter.set_direction(DirState { dir: [true, false, true] });
    for _ in 0..250 {
      counter.advance(&StepEvent { step: [true, true, false], period_ticks: 12 });
    }
    // X +250 steps at 250 steps/mm = +1.0 mm; Y −250 steps at 100 steps/mm = −2.5 mm; Z untouched.
    let mm = counter.position_mm(&[250.0, 100.0, 0.0]);
    assert!((mm[0] - 1.0).abs() < 1e-6);
    assert!((mm[1] + 2.5).abs() < 1e-6);
    // A zero steps/mm axis is reported as 0.0, not a division blow-up.
    assert_eq!(mm[2], 0.0);
  }

  /// A reset zeroes the live step position (so MPos returns to the origin) without disturbing the
  /// latched direction — the next block re-latches it before any step.
  #[test]
  fn step_counter_reset_returns_to_origin() {
    let mut counter = StepCounter::new();
    counter.set_direction(DirState { dir: [false, false, false] });
    counter.advance(&StepEvent { step: [true, true, true], period_ticks: 12 });
    assert_eq!(counter.position_steps(), [-1, -1, -1]);
    counter.reset();
    assert_eq!(counter.position_steps(), [0, 0, 0]);
    // Direction is retained: a subsequent step still goes negative until a block re-latches it.
    counter.advance(&StepEvent { step: [true, false, false], period_ticks: 12 });
    assert_eq!(counter.position_steps(), [-1, 0, 0]);
  }

  /// `steps_to_mm` (the standalone conversion the bin's status reporter uses against the live atomics)
  /// divides each axis by its `$100..102` resolution, and a non-positive resolution yields a finite `0.0`.
  #[test]
  fn steps_to_mm_converts_with_degenerate_axis_safe() {
    let mm = steps_to_mm(&[250, -250, 7], &[250.0, 100.0, 0.0]);
    assert!((mm[0] - 1.0).abs() < 1e-6);
    assert!((mm[1] + 2.5).abs() < 1e-6);
    // A zero steps/mm axis is reported as 0.0, never a division blow-up.
    assert_eq!(mm[2], 0.0);
  }

  // ---- Probe stepper: per-step sampling, stop-on-edge, no-contact (Phase C) ---------------------

  /// A probe block straight down Z by 5 mm at 100 steps/mm → 500 Z steps. Constant feed, no trapezoid.
  fn probe_block() -> Block {
    make_block([0, 0, -500], 5.0, 100.0, 0.0, 100.0)
  }

  #[test]
  fn probe_stops_on_the_trigger_edge_and_reports_steps() {
    // The probe triggers after 120 steps of travel: the stepper must emit exactly 120 steps then stop, reporting
    // a trigger. The single-tick bursts give per-step sampling granularity (the architecture's finest).
    let stepper = ProbeStepper::new(test_config());
    let block = probe_block();
    let mut count = 0u32;
    let mut sink = RecordingSink::new();
    let outcome = stepper
      .run_probe(&block, 1000, || { let trip = count >= 120; count += 1; trip }, &mut sink)
      .expect("probe runs");
    assert!(outcome.triggered, "the probe saw its trigger edge");
    assert_eq!(outcome.steps_emitted, 120, "stopped exactly at the trigger step");
    // Each burst is a single tick (per-step sampling), and the dominant Z axis steps every tick.
    assert!(sink.bursts.iter().all(|b| b.len() == 1), "probe bursts are single-tick for per-step sampling");
    assert_eq!(sink.bursts.len(), 120);
    assert!(sink.all_ticks().iter().all(|ev| ev.step[2]), "Z steps on every probe tick");
  }

  #[test]
  fn probe_reaching_target_without_contact_reports_no_trigger() {
    // The probe never trips: the stepper runs the full 500-step block and reports no trigger (the consumer turns
    // this into ALARM:4/5 for G38.2/.4, or a silent finish for G38.3/.5).
    let stepper = ProbeStepper::new(test_config());
    let block = probe_block();
    let mut sink = RecordingSink::new();
    let outcome = stepper.run_probe(&block, 1000, || false, &mut sink).expect("probe runs");
    assert!(!outcome.triggered, "no contact within travel");
    assert_eq!(outcome.steps_emitted, 500, "ran the full block to the no-contact end of travel");
    assert_eq!(sink.step_totals(), [0, 0, 500]);
  }

  #[test]
  fn probe_already_at_edge_stops_immediately_with_no_steps() {
    // If the stop edge is present before the first tick, the stepper takes no step (the firmware bin rejects a
    // toward-probe already triggered as ALARM:4 before calling this, but the walker is still safe).
    let stepper = ProbeStepper::new(test_config());
    let block = probe_block();
    let mut sink = RecordingSink::new();
    let outcome = stepper.run_probe(&block, 1000, || true, &mut sink).expect("probe runs");
    assert!(outcome.triggered);
    assert_eq!(outcome.steps_emitted, 0, "no step taken when already at the edge");
    assert!(sink.bursts.is_empty(), "no bursts emitted");
  }

  #[test]
  fn probe_trigger_on_the_final_step_is_detected() {
    // A trigger on the very last step of travel must still report a trigger (the loop samples before each step
    // and once more after the final step, so the last position is not missed).
    let stepper = ProbeStepper::new(test_config());
    let block = probe_block();
    let mut count = 0u32;
    let mut sink = RecordingSink::new();
    let outcome = stepper
      .run_probe(&block, 1000, || { let trip = count >= 500; count += 1; trip }, &mut sink)
      .expect("probe runs");
    assert!(outcome.triggered, "a trigger on the final step is detected by the post-loop sample");
    assert_eq!(outcome.steps_emitted, 500);
  }

  #[test]
  fn probe_cycle_latches_machine_position_and_renders_prb_and_wpos() {
    // The end-to-end simulated probe → Z-zero workflow, crossing the probe stepper, the live step counter, the
    // `[PRB:]` formatter, and the coordinate model — the exact pipeline the firmware bin wires, but host-tested.
    use crate::coords::CoordinateSystems;
    use crate::hal_traits::{probe_triggered, ProbeConfig};
    use crate::protocol::ResponseWriter;
    use std::string::String as StdString;

    // A probe block straight down Z by 5 mm at 100 steps/mm, starting at machine Z = 0 (no prior moves). The
    // touch plate (with `$6=1`, NO-plate) trips after 4.2 mm of travel → 420 Z steps, i.e. machine Z = -4.2 mm.
    let cfg = test_config();
    let prober = ProbeStepper::new(cfg);
    let block = make_block([0, 0, -500], 5.0, 100.0, 0.0, 100.0);

    // A scripted mock probe: idles high (untouched), goes low after 420 steps of travel. Under the base sense
    // (`$6=0`) a high pin reads not-triggered and a low pin (grounded by contact) reads triggered — the right
    // config for an idle-high touch-plate input on this electrical model (`probe_triggered` is unit-tested in
    // `hal_traits`). `steps_taken` is a shared `Cell` so the counting sink and the probe predicate can both touch
    // it without an aliasing borrow conflict.
    let probe_cfg = ProbeConfig { invert: false, pullup_disable: false };
    let steps_taken = std::cell::Cell::new(0u32);
    let raw_high = std::cell::Cell::new(true);
    let is_at_edge = || {
      // The plate trips (goes low) once 420 steps have been emitted.
      if steps_taken.get() >= 420 {
        raw_high.set(false);
      }
      probe_triggered(raw_high.get(), &probe_cfg)
    };

    // Advance a live step counter through the same ticks the prober emits, exactly as the firmware's CountingSink
    // does, so the latched position is derived from the emitted steps.
    let mut counter = StepCounter::new();
    counter.set_direction(DirState { dir: [block.steps[0] >= 0, block.steps[1] >= 0, block.steps[2] >= 0] });

    struct CountingRecorder<'a> {
      counter: &'a mut StepCounter,
      steps_taken: &'a std::cell::Cell<u32>,
    }
    impl StepSink for CountingRecorder<'_> {
      fn set_direction(&mut self, _dir: DirState) -> Result<(), StepError> {
        Ok(())
      }
      fn emit_burst(&mut self, ticks: &[StepEvent]) -> Result<(), StepError> {
        for ev in ticks {
          self.counter.advance(ev);
          self.steps_taken.set(self.steps_taken.get() + 1);
        }
        Ok(())
      }
    }
    let mut sink = CountingRecorder { counter: &mut counter, steps_taken: &steps_taken };
    let outcome = prober.run_probe(&block, 1000, is_at_edge, &mut sink).expect("probe runs");

    assert!(outcome.triggered, "the NO plate tripped within travel");
    // The latched machine position: 420 Z steps in the negative direction → Z = -4.2 mm at 100 steps/mm.
    let stop_steps = counter.position_steps();
    assert_eq!(stop_steps, [0, 0, -420], "latched at the trigger step");
    let steps_per_mm = [100.0, 100.0, 100.0];
    let probe_mm = steps_to_mm(&stop_steps, &steps_per_mm);
    assert!((probe_mm[2] + 4.2).abs() < 1e-4, "probe machine Z is -4.2 mm, got {}", probe_mm[2]);

    // The immediate `[PRB:]` push reports the triggered machine position with flag 1.
    let mut prb: StdString = StdString::new();
    {
      let mut s = heapless::String::<64>::new();
      ResponseWriter::probe_report(&mut s, &probe_mm, outcome.triggered).expect("prb");
      prb.push_str(s.as_str());
    }
    assert_eq!(prb, "[PRB:0.000,0.000,-4.200:1]\r\n");

    // Z-zero: the plate is 1.0 mm thick, so `G10 L20 P1 Z1.0` makes the probed point read work Z = 1.0, putting
    // the copper top (1 mm below the plate top the probe touched) at WPos Z = 0.
    let mut cs = CoordinateSystems::new();
    cs.set_wcs_offset_to_position(0, probe_mm, [0.0, 0.0, 1.0], [false, false, true]);
    let copper_top = [probe_mm[0], probe_mm[1], probe_mm[2] - 1.0];
    let wpos = cs.machine_to_work(copper_top);
    assert!(wpos[2].abs() < 1e-4, "copper top reads WPos Z = 0, got {}", wpos[2]);
  }

  #[test]
  fn probe_period_clamps_into_representable_interval() {
    // A wildly slow requested period must clamp to the representable max so the single-tick burst is encodable;
    // the probe still runs and reports correctly.
    let cfg = test_config();
    let stepper = ProbeStepper::new(cfg);
    let block = probe_block();
    let mut sink = RecordingSink::new();
    stepper.run_probe(&block, u32::MAX, || false, &mut sink).expect("probe runs");
    let max_period = RMT_MAX_FIELD_LEN + cfg.step_pulse_ticks;
    assert!(sink.all_ticks().iter().all(|e| e.period_ticks <= max_period), "probe period stays representable");
    assert!(sink.all_ticks().iter().all(|e| e.period_ticks >= cfg.min_period_ticks()), "and above the min");
  }

  // ---- Silent-symbol half split: never an RMT end marker (Finding #1) ---------------------------

  /// The silent (no-step) symbol must split a full LOW period into two NON-ZERO halves: a zero-length RMT
  /// field is an end marker that would terminate the channel mid-burst and drop a subordinate axis's
  /// remaining steps. Across every representable period (and a degenerate tiny one), both halves are ≥ 1, fit
  /// the 15-bit field, and (for non-degenerate periods) sum to the period exactly.
  #[test]
  fn silent_symbol_halves_are_never_an_end_marker() {
    // Sweep the representable period range plus the degenerate edges. `max_period` is the slowest period the
    // generator can emit (`RMT_MAX_FIELD_LEN + $0`) with the default 10-tick pulse.
    let max_period = RMT_MAX_FIELD_LEN + 10;
    for period in [2u32, 3, 12, 13, 1000, 30_000, max_period - 1, max_period] {
      let (a, b) = silent_symbol_halves(period);
      assert!(a >= 1 && b >= 1, "neither half may be zero (period {period}): got ({a}, {b})");
      assert!(a <= RMT_MAX_FIELD_LEN && b <= RMT_MAX_FIELD_LEN, "both halves fit the 15-bit field");
      assert_eq!(a + b, period, "the two halves reconstruct the full period exactly (period {period})");
    }
    // Degenerate sub-minimum periods still never produce a zero half (defensive floor, never hit in practice).
    for period in [0u32, 1] {
      let (a, b) = silent_symbol_halves(period);
      assert!(a >= 1 && b >= 1, "a degenerate period still yields two non-zero halves: ({a}, {b})");
    }
  }

  /// Saturating arithmetic pins the position at `i32::MAX` rather than wrapping, so a pathological step
  /// train keeps the published MPos monotone instead of aliasing to a negative value.
  #[test]
  fn step_counter_saturates_at_i32_bounds() {
    let mut counter = StepCounter::new();
    counter.set_direction(DirState { dir: [true, true, true] });
    // Seed near the ceiling, then step past it; the axis must clamp, not wrap.
    for _ in 0..3 {
      counter.advance(&StepEvent { step: [true, false, false], period_ticks: 12 });
    }
    // Manually drive X to the ceiling via a direct check: advance cannot reach i32::MAX in a test loop,
    // so assert the saturating contract at the boundary by constructing the edge case in steps.
    let mut edge = StepCounter::new();
    edge.set_direction(DirState { dir: [true, true, true] });
    edge.position = [i32::MAX, 0, 0];
    edge.advance(&StepEvent { step: [true, false, false], period_ticks: 12 });
    assert_eq!(edge.position_steps()[0], i32::MAX);
  }
}
