//! Hardware abstraction traits (DOC-09).
//!
//! Every hardware interaction in firmware-core is expressed through one of these traits so the
//! planner, parser, and driver logic stay host-testable with recording/mock implementations. Only
//! the `firmware` binary implements them against esp-hal peripherals.
//!
//! Status: trait surface is sketched per DOC-09. Concrete error types and method bodies land with
//! their owning subsystems. The TMC2209 register codec in [`crate::drivers::tmc2209`] does not
//! depend on a `TmcBus` trait; the transport trait is wired up when the `tmc_manager` task is built.

use crate::planner::AXES;

/// The maximum number of [`StepEvent`]s (RMT PulseCode symbols) a single burst may carry. DOC-02: the
/// ESP32-S3 RMT block holds 48 symbols (`SOC_RMT_MEM_WORDS_PER_CHANNEL`); a burst larger than one block
/// forces the driver to borrow the adjacent channel's memory and breaks the all-three-axes-at-once
/// allocation. Bursts are capped at one block so the interrupt ping-pong refill path is avoided.
pub const MAX_SYMBOLS_PER_BURST: usize = 48;

/// The direction state latched onto the three axis DIR outputs before a burst. One `bool` per axis,
/// indexed `[X, Y, Z]`: `true` is the positive (increasing-step) direction, `false` is negative. The
/// segment generator derives this from the signs of `Block::steps`; it is constant for a whole planner
/// block (a straight-line move never reverses an axis mid-block), so it is set once per block, never
/// per tick. On target the implementation drives the DIR GPIOs and then honors the `$29` direction
/// setup delay before the first step pulse of the following burst.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DirState {
  /// Per-axis direction, `[X, Y, Z]`; `true` = positive, `false` = negative.
  pub dir: [bool; AXES],
}

/// A single DDA tick: the synchronized step decision for all three axes on one step-generator tick,
/// plus the full step period that tick occupies. This is the unit the RMT layer turns into exactly one
/// PulseCode per channel: an axis that steps emits a HIGH-for-`$0`-ticks / LOW-for-remainder pulse, and
/// an axis that does not step emits a full-period-LOW (silent) symbol so all three channel bursts stay
/// the same length and remain sample-aligned under the motion executor's `join3` await (DOC-02).
///
/// The dominant axis (largest step count in the block) steps on every tick; subordinate axes step only
/// when their Bresenham error accumulator crosses, so `step` is a per-tick mask, not a constant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StepEvent {
  /// Per-axis step mask for this tick, `[X, Y, Z]`; `true` means the axis emits a step pulse this tick.
  pub step: [bool; AXES],
  /// The full step period of this tick in [`MotionConfig`](crate::motion::MotionConfig) timer ticks
  /// (one HIGH+LOW pulse). The HIGH width is the `$0` step-pulse time; the LOW width is the remainder.
  /// A larger period is a slower instantaneous velocity; varying it across ticks realizes the ramp.
  pub period_ticks: u32,
}

/// Errors a [`StepSink`] may return. All are recoverable by the caller (the motion executor): the
/// segment generator surfaces them rather than panicking, per the firmware-core no-`unwrap` rule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StepError {
  /// A burst exceeded [`MAX_SYMBOLS_PER_BURST`] events. The segment generator never emits such a burst
  /// (it flushes at the cap); this guards a sink against a mis-sized burst from any other producer.
  BurstTooLong,
  /// The underlying step transport failed (on target: an RMT transmit error). Host recorders never
  /// return this.
  Transport,
}

/// Sink for coordinated, axis-synchronized step bursts. Implemented over the three RMT TX channels on
/// target (one burst becomes one PulseCode array per channel, transmitted concurrently and awaited
/// together) and as a recording buffer in host tests.
///
/// ## Contract
/// - [`set_direction`](StepSink::set_direction) is called once before the burst(s) of a block whenever
///   the direction changes; the implementation latches the DIR outputs and observes the `$29` setup
///   delay before the next emitted pulse. Direction never changes within a planner block.
/// - [`emit_burst`](StepSink::emit_burst) submits one constant-rate-region slice of up to
///   [`MAX_SYMBOLS_PER_BURST`] [`StepEvent`]s. Every event is one synchronized tick across all axes;
///   the slice length is the symbol count for each channel, so all channels stay aligned. A slice
///   longer than the cap must be rejected with [`StepError::BurstTooLong`].
/// - The sink is purely a consumer of decided ticks; all trapezoid/DDA/timing math lives in the
///   segment generator so it stays host-testable.
pub trait StepSink {
  /// Latch the per-axis direction outputs ahead of the next burst, honoring the direction setup delay.
  fn set_direction(&mut self, dir: DirState) -> Result<(), StepError>;

  /// Emit one burst of up to [`MAX_SYMBOLS_PER_BURST`] synchronized step ticks. Returns
  /// [`StepError::BurstTooLong`] if the slice exceeds the cap, or [`StepError::Transport`] on a
  /// hardware transmit failure.
  fn emit_burst(&mut self, ticks: &[StepEvent]) -> Result<(), StepError>;
}

// PwmSink: normalized 0.0..=1.0 spindle duty sink (LEDC PWM on target).
// TODO(DOC-05): pub trait PwmSink { fn set_duty(&mut self, frac: f32) -> Result<(), PwmError>; }

// DigitalIn: limit / control digital input (NC limit switches, feed-hold, cycle-start).
// TODO(DOC-06): pub trait DigitalIn { fn is_active(&self) -> bool; }

// DigitalOut: digital output (stepper enable, spindle enable/direction).
// TODO(DOC-05): pub trait DigitalOut { fn set(&mut self, level: bool) -> Result<(), ()>; }

// TmcBus: half-duplex TMC2209 single-wire UART transport; implemented over UART1 on target, byte
// buffer in host tests. The byte-level datagram encode/decode it relies on lives in
// `crate::drivers::tmc2209`.
// TODO(DOC-03): pub trait TmcBus { fn write_reg(&mut self, node: u8, reg: u8, val: u32) -> Result<(), TmcError>;
//                                   fn read_reg(&mut self, node: u8, reg: u8) -> Result<u32, TmcError>; }
