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

/// The runtime configuration of the probe input that the host-tested probe logic needs (DOC-09, Phase C). It
/// folds the two grblHAL probe `$`-settings the probe READ depends on:
/// - `$6` (`invert`): a Normally-Open touch plate sits open (pulled high) until contact, so `$6=1` inverts the
///   raw pin so an UNTOUCHED plate reads "not triggered" and contact reads "triggered" (see `docs/tlo-offsets.md`
///   Finding #6). The host-tested [`probe_triggered`] applies this, so the trigger sense is unit-tested rather
///   than buried in the GPIO wiring.
/// - `$19` (`pullup_disable`): whether the firmware enables the input's internal pull-up. It is a PIN-CONFIG
///   concern (the GPIO layer applies it at bring-up), not a per-read transform, so it is carried here only so the
///   firmware bin can read it from one place; [`probe_triggered`] does not consult it. A passive plate needs the
///   pull-up, so the default is `false` (pull-up enabled), matching grbl's `$19=0`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ProbeConfig {
  /// `$6` probe-pin invert. When `true`, the raw electrical level is inverted before the trigger decision, so a
  /// Normally-Open plate (open/high when untouched) reads "not triggered" until contact. Default `false`.
  pub invert: bool,
  /// `$19` probe-pin pull-up DISABLE. `true` disables the input's internal pull-up; a passive touch plate needs
  /// it enabled, so the default is `false`. Applied by the GPIO layer at pin config, not by [`probe_triggered`].
  pub pullup_disable: bool,
}

impl Default for ProbeConfig {
  /// grbl's probe defaults: pull-up enabled (`$19=0`) and no invert (`$6=0`). A Normally-Open plate then needs
  /// `$6=1` set explicitly, which `docs/tlo-offsets.md` documents as the common touch-plate configuration.
  fn default() -> Self {
    ProbeConfig { invert: false, pullup_disable: false }
  }
}

/// Decide whether the probe is currently TRIGGERED (contact made) from its RAW electrical level and the probe
/// config, applying the `$6` invert. This is the single host-tested place the invert is honored so the firmware
/// bin's GPIO read and the executor's probe watch agree on the trigger sense. `raw_high` is the pin level
/// directly off the input (`true` = electrically high); with `$6=0` a high pin is "not triggered" and a low pin
/// is "triggered" (grbl's NC philosophy), and `$6=1` flips that for a Normally-Open plate.
pub fn probe_triggered(raw_high: bool, config: &ProbeConfig) -> bool {
  // The base (`$6=0`) sense follows grbl's documented "pin low/grounded = triggered, pin high = not triggered":
  // an active touch-plate input idles high (held by the pull-up) and is pulled low on contact. `$6=1` is a pure
  // electrical INVERT of that decision, so a board whose probe circuit idles low (e.g. an opto-isolated or
  // hardware-inverted input) reads "not triggered" until the level flips. `docs/tlo-offsets.md` Finding #6 is
  // explicit that the correct `$6` is hardware-specific and must be verified empirically (the `Pn:P` flag must
  // be ABSENT with the plate untouched); this keeps `$6` a single, unambiguous invert so that calibration works.
  let triggered_when_low = !raw_high;
  triggered_when_low ^ config.invert
}

/// A probe digital input (DOC-09, Phase C). Implemented over a single GPIO with a pull-up on target (a dedicated
/// pin separate from the limit switches, per `docs/tlo-offsets.md` Finding #7) and as a scripted recording mock
/// in host tests. The trait exposes only the RAW electrical level; the `$6` invert is applied by the host-tested
/// [`probe_triggered`] so the firmware impl carries zero settings knowledge and the trigger sense stays unit-
/// tested. The probe-cycle executor (in the firmware bin's motion layer) samples this between step bursts.
pub trait ProbeInput {
  /// The raw electrical level of the probe pin: `true` = high, `false` = low. The `$6`-adjusted trigger decision
  /// is made by [`probe_triggered`]; this reader is deliberately invert-agnostic so the policy lives in one place.
  fn is_high(&self) -> bool;
}

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
/// deliberately allowed, mirroring [`RecordStore`]: firmware-core only ever drives these futures via static
/// dispatch (`TmcManager<B: TmcBus>`) from its own single-task `async fn` methods — exactly the "use the trait
/// only in your own code" case the lint calls out, so no boxing or `Send` bound is needed.
#[allow(async_fn_in_trait)]
pub trait TmcBus {
  /// Send a write-access datagram setting `reg = val` on `node`, discarding the half-duplex echo.
  async fn write_reg(&mut self, node: u8, reg: u8, val: u32) -> Result<(), TmcError>;

  /// Send a read-request datagram for `reg` on `node` and return the decoded 32-bit register value.
  async fn read_reg(&mut self, node: u8, reg: u8) -> Result<u32, TmcError>;
}

/// Errors a [`RecordStore`] may return. The store deals only in opaque framed bytes (DOC-04 persistence); the
/// per-record `wire` layer ([`crate::settings::wire`] / [`crate::coords::wire`]) owns the framing/CRC/version,
/// so these are purely about the storage medium, not the record contents.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StoreError {
  /// No record has been persisted yet (a fresh device). The loader treats this as "use defaults".
  NotFound,
  /// The underlying storage medium failed (on target: a flash read/write/erase error).
  Io,
  /// The record is larger than the caller's buffer (load) or the store's capacity (save).
  TooLarge,
}

/// Persistence backend for ONE framed record (DOC-04 / Phase B). A single trait serves every persisted blob —
/// the `$`-settings frame ([`crate::settings::wire`]) and the coordinate frame ([`crate::coords::wire`]) — so the
/// flash plumbing lives once: the firmware bin's `FlashRecordStore` carries the per-record NVS key and backs both
/// the settings loader and the coordinate loader with the same code. Implemented over `sequential-storage` on the
/// NVS flash partition on target, and as an in-memory buffer in host tests — mirroring how [`TmcBus`]/[`StepSink`]
/// abstract their hardware.
///
/// The store moves only opaque framed bytes: all protobuf encode/decode, the magic/version header, and the CRC
/// live in the record's own `wire` module, so a store impl carries zero record knowledge and the whole
/// load/save/versioning policy stays host-testable. Each store instance targets exactly one record (the firmware
/// bin builds one per NVS key), so the trait itself stays key-less — the record's identity is the store's, not a
/// per-call argument.
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
pub trait RecordStore {
  /// Read the persisted record frame into `buf`, returning its length. [`StoreError::NotFound`] if nothing has
  /// been stored yet; [`StoreError::TooLarge`] if the record does not fit `buf`.
  async fn load(&mut self, buf: &mut [u8]) -> Result<usize, StoreError>;

  /// Persist `frame` (a complete, already-framed record) to the backing store, replacing any prior record.
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

  // ---- Probe input: `$6` invert / `$19` pull-up semantics ----------------------------------------

  #[test]
  fn probe_default_config_is_low_triggered_pullup_enabled() {
    // grbl defaults: no invert (`$6=0`), pull-up enabled (`$19=0`). The base sense is "pin low = triggered": a
    // touch-plate input idles HIGH (held by the pull-up) and is pulled low on contact, so high reads
    // not-triggered and low reads triggered.
    let cfg = ProbeConfig::default();
    assert!(!cfg.invert);
    assert!(!cfg.pullup_disable);
    assert!(!probe_triggered(true, &cfg), "untouched (high) reads not-triggered under $6=0");
    assert!(probe_triggered(false, &cfg), "contact (low) reads triggered under $6=0");
  }

  #[test]
  fn probe_invert_flips_the_trigger_sense() {
    // `$6=1` is a pure electrical invert: it flips both levels' trigger decision so a board whose probe input
    // idles LOW (an opto-isolated / hardware-inverted circuit) reads not-triggered until the level rises. The
    // correct `$6` for a given plate is hardware-specific (verified empirically per `docs/tlo-offsets.md`); what
    // is unit-tested here is that the bit cleanly inverts the base decision.
    let inverted = ProbeConfig { invert: true, pullup_disable: false };
    let base = ProbeConfig::default();
    assert_eq!(probe_triggered(true, &inverted), !probe_triggered(true, &base), "$6 inverts the high decision");
    assert_eq!(probe_triggered(false, &inverted), !probe_triggered(false, &base), "$6 inverts the low decision");
    // Concretely: under `$6=1`, a high pin is triggered and a low pin is not.
    assert!(probe_triggered(true, &inverted));
    assert!(!probe_triggered(false, &inverted));
  }

  #[test]
  fn probe_pullup_disable_does_not_affect_the_trigger_decision() {
    // `$19` is a pin-config concern (the GPIO layer enables/disables the pull-up); it must NOT change the
    // trigger decision, which depends only on the raw level and `$6`.
    let with = ProbeConfig { invert: false, pullup_disable: true };
    let without = ProbeConfig { invert: false, pullup_disable: false };
    assert_eq!(probe_triggered(true, &with), probe_triggered(true, &without));
    assert_eq!(probe_triggered(false, &with), probe_triggered(false, &without));
  }
}
