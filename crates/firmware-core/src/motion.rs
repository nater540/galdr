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

    let profile = TrapezoidProfile::plan(block, exit_speed_sq);
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
      let period = self.period_ticks(v_sq, block, dominant);
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

  /// Convert an instantaneous squared velocity (mm/s)² into the dominant-axis step period in timer
  /// ticks. The dominant axis emits one step per `dom_mm_per_step` of travel, so its step rate is
  /// `v / dom_mm_per_step` steps/s; the period is `tick_hz / step_rate`, floored at the minimum period
  /// so the pulse timing is always realizable. A non-positive velocity floors the rate at the slowest
  /// representable step (the period saturates rather than dividing by zero).
  fn period_ticks(&self, v_sq: f32, block: &Block, dominant: usize) -> u32 {
    let dom_steps = block.steps[dominant].unsigned_abs();
    // `dom_steps` is ≥ 1 for any block that reaches here (step_event_count > 0 and `dominant` is the max
    // axis), so this division is safe; mm-per-dominant-step is the block length over the dominant count.
    let dom_mm_per_step = block.millimeters / dom_steps as f32;
    let v = libm::sqrtf(v_sq.max(0.0));
    let step_rate = if dom_mm_per_step > 0.0 { v / dom_mm_per_step } else { 0.0 };
    let max_rate = self.config.max_step_rate_hz();
    let clamped = step_rate.min(max_rate);
    if clamped <= 0.0 {
      // Velocity collapsed to (or below) zero; emit the slowest representable step rather than diverge.
      return SLOWEST_PERIOD_TICKS.max(self.config.min_period_ticks());
    }
    let period = self.config.tick_hz / clamped;
    // Round to the nearest whole tick, then floor at the minimum period to respect the pulse timing.
    let rounded = libm::roundf(period) as u32;
    rounded.max(self.config.min_period_ticks())
  }
}

/// A ceiling on the step period (in ticks) for a velocity that has collapsed to zero, so a momentary
/// `v = 0` at the boundary of a rest-to-rest block emits a finite, very slow step instead of dividing by
/// zero or saturating the 15-bit RMT duration field. One tick of motion at this rate is harmless because
/// such ticks only occur at the extreme ends of a ramp. Chosen well below the RMT 15-bit max (32767).
const SLOWEST_PERIOD_TICKS: u32 = 30_000;

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
  /// Block acceleration magnitude in mm/s² (same for accel and decel; grbl's symmetric model).
  accel: f32,
  /// Total block travel in mm.
  length: f32,
  /// Distance from the start at which acceleration ends (cruise begins), in mm.
  accel_end: f32,
  /// Distance from the start at which deceleration begins (cruise ends), in mm.
  decel_start: f32,
}

impl TrapezoidProfile {
  /// Classify the block's trapezoid from its planner-solved entry speed, its nominal cruise speed, and
  /// the required exit speed. Distances are derived from `v² = v₀² + 2·a·d`, rearranged to
  /// `d = (v² − v₀²) / (2·a)`. When the accel and decel ramps would overlap before reaching nominal, the
  /// block is a triangle and the peak is found from the ramp-intersection distance.
  fn plan(block: &Block, exit_speed_sq: f32) -> Self {
    let entry_sq = block.entry_speed_sq.max(0.0);
    let nominal_sq = block.nominal_speed_sq.max(0.0);
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
        accel,
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
        accel,
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
    let two_a = 2.0 * self.accel;
    if d <= self.accel_end {
      // Accelerating: rises from entry, capped at the cruise peak.
      (self.entry_sq + two_a * d).min(self.cruise_sq).max(0.0)
    } else if d <= self.decel_start {
      // Cruising at the peak speed.
      self.cruise_sq
    } else {
      // Decelerating: falls from cruise toward exit over the remaining travel, floored at exit.
      let remaining = (self.length - d).max(0.0);
      (self.exit_sq + two_a * remaining).min(self.cruise_sq).max(self.exit_sq.min(self.cruise_sq))
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
}
