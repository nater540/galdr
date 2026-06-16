//! Hardware abstraction traits (DOC-09).
//!
//! Every hardware interaction in firmware-core is expressed through one of these traits so the
//! planner, parser, and driver logic stay host-testable with recording/mock implementations. Only
//! the `firmware` binary implements them against esp-hal peripherals.
//!
//! Status: [`StepSink`] (DOC-02) and [`TmcBus`] (DOC-03) are implemented; the remaining traits land
//! with their owning subsystems. [`TmcBus`] carries the TMC2209 single-wire datagrams whose byte-level
//! codec lives in [`crate::drivers::tmc2209`] and whose register orchestration lives in
//! [`crate::drivers::tmc2209::manager`].

use crate::drivers::tmc2209::TmcError;
use crate::planner::AXES;

/// The maximum number of [`StepEvent`]s (RMT PulseCode symbols) a single burst may carry. The ESP32-S3 RMT
/// memory block holds 48 symbols (`SOC_RMT_MEM_WORDS_PER_CHANNEL`), but the firmware bin appends one
/// mandatory `end_marker` symbol after the events, so an N-event burst encodes to N+1 symbols. Capping at
/// 47 events makes a full burst exactly 47 + 1 = 48 symbols — precisely one memory block (`memsize = 1`).
/// This keeps every burst inside a single block so the driver never borrows the adjacent channel's memory
/// and the interrupt-priority streaming-refill (ping-pong) path is never relied upon (DOC-02). A burst
/// larger than this is rejected with [`StepError::BurstTooLong`].
pub const MAX_SYMBOLS_PER_BURST: usize = 47;

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

/// Half-duplex TMC2209 single-wire UART transport (DOC-03). Implemented over UART1 on target (one bus
/// shared by all three driver nodes) and as a byte-buffer mock in host tests. The byte-level datagram
/// encode/decode this relies on lives in [`crate::drivers::tmc2209`]; the register-level orchestration
/// that drives it lives in [`crate::drivers::tmc2209::manager`].
///
/// ## Contract
/// - [`write_reg`](TmcBus::write_reg) sends one write-access datagram and consumes the half-duplex echo
///   of the bytes it just drove onto the shared line. It returns once the write is on the wire; the
///   TMC2209 sends no reply to a write, so acceptance is confirmed out-of-band by reading `IFCNT`.
/// - [`read_reg`](TmcBus::read_reg) sends one read-request datagram, consumes the request echo, then
///   reads and validates the 8-byte reply, returning the 32-bit register value. A node that never
///   answers (absent / standalone VREF mode / broken bus) surfaces as [`TmcError::Timeout`].
/// - `node` is the 0..=3 driver address; `reg` is the 7-bit register address (the write flag is added
///   by the codec). Implementations must discard the single-wire loopback echo so it is never mistaken
///   for a reply.
///
/// The methods are `async` so the on-target UART1 transport can `.await` the half-duplex exchange (drive the
/// frame, await the echo/reply) directly instead of busy-spinning a `delay_micros` poll loop, which on the
/// single core-0 executor would stall every other task for the bus turn-around window. firmware-core itself
/// needs no async runtime: the [`manager`](crate::drivers::tmc2209::manager) only `.await`s these futures from
/// its own `async fn` init/poll methods, and the host mock's futures are immediately ready.
///
/// The `async fn`-in-trait lint (auto-trait bounds like `Send` cannot be named on the returned future) is
/// deliberately allowed, mirroring [`SettingsStore`]: firmware-core only ever drives these futures via static
/// dispatch (`TmcManager<B: TmcBus>`) from its own single-task `async fn` methods — exactly the "use the trait
/// only in your own code" case the lint calls out, so no boxing or `Send` bound is needed.
#[allow(async_fn_in_trait)]
pub trait TmcBus {
  /// Send a write-access datagram setting `reg = val` on `node`, discarding the half-duplex echo.
  async fn write_reg(&mut self, node: u8, reg: u8, val: u32) -> Result<(), TmcError>;

  /// Send a read-request datagram for `reg` on `node` and return the decoded 32-bit register value.
  async fn read_reg(&mut self, node: u8, reg: u8) -> Result<u32, TmcError>;
}

/// Errors a [`SettingsStore`] may return. The store deals only in opaque framed bytes (DOC-04 persistence);
/// the [`crate::settings::wire`] layer owns the framing/CRC/version, so these are purely about the storage
/// medium, not the record contents.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StoreError {
  /// No settings record has been persisted yet (a fresh device). The loader treats this as "use defaults".
  NotFound,
  /// The underlying storage medium failed (on target: a flash read/write/erase error).
  Io,
  /// The record is larger than the caller's buffer (load) or the store's capacity (save).
  TooLarge,
}

/// Persistence backend for the settings record (DOC-04). Implemented over `sequential-storage` on the NVS
/// flash partition on target, and as an in-memory buffer in host tests — mirroring how [`TmcBus`]/[`StepSink`]
/// abstract their hardware. The store moves only opaque framed bytes: all protobuf encode/decode, the
/// magic/version header, and the CRC live in [`crate::settings::wire`], so a store impl carries zero settings
/// knowledge and the whole load/save/versioning policy stays host-testable.
///
/// The methods are `async` so the firmware can `.await` the (interrupt-driven, erase-before-write) flash
/// transport directly instead of busy-spinning a `block_on`, which on a single-executor target risks a
/// same-executor deadlock. firmware-core itself needs no async runtime: it only `.await`s these futures from
/// its own `async fn` loaders, and the host mock's futures are immediately ready.
///
/// The `async fn`-in-trait lint (auto-trait bounds like `Send` cannot be named on the returned future) is
/// deliberately allowed: firmware-core only ever drives these futures via static dispatch from its own
/// single-task `async fn` loaders — exactly the "use the trait only in your own code" case the lint calls out.
#[allow(async_fn_in_trait)]
pub trait SettingsStore {
  /// Read the persisted settings frame into `buf`, returning its length. [`StoreError::NotFound`] if nothing
  /// has been stored yet; [`StoreError::TooLarge`] if the record does not fit `buf`.
  async fn load(&mut self, buf: &mut [u8]) -> Result<usize, StoreError>;

  /// Persist `frame` (a complete, already-framed settings record) to the backing store, replacing any prior
  /// record.
  async fn save(&mut self, frame: &[u8]) -> Result<(), StoreError>;
}

#[cfg(test)]
mod tests {
  use super::*;

  /// The ESP32-S3 RMT memory block holds 48 symbols. The firmware bin appends one `end_marker` after the
  /// events of a burst, so a full burst of [`MAX_SYMBOLS_PER_BURST`] events encodes to
  /// `MAX_SYMBOLS_PER_BURST + 1` symbols, which must fit one block exactly — otherwise the burst spills into
  /// the adjacent channel's memory or relies on the interrupt-priority streaming refill (Finding #7).
  #[test]
  fn full_burst_plus_end_marker_fits_one_rmt_block() {
    const RMT_BLOCK_SYMBOLS: usize = 48;
    assert_eq!(MAX_SYMBOLS_PER_BURST + 1, RMT_BLOCK_SYMBOLS, "events + end marker must equal one RMT block");
  }
}
