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

// The coordinated step-output contract (`StepSink`/`StepEvent`/`DirState`/`StepError` +
// `MAX_SYMBOLS_PER_BURST`) lives in the shared `cnc-kinematics` crate alongside the segment generator
// that produces the events. Re-export it here so the firmware bin keeps its familiar
// `firmware_core::hal_traits::StepSink` paths and so this module stays the one hardware-trait surface.
pub use cnc_kinematics::step::{DirState, StepError, StepEvent, StepSink, MAX_SYMBOLS_PER_BURST};

/// Errors a [`PwmSink`] may return. Like [`StepError`] these are recoverable by the caller (the spindle task):
/// the [`SpindleController`](crate::spindle::SpindleController) surfaces them rather than panicking, per the
/// firmware-core no-`unwrap` rule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PwmError {
  /// The underlying PWM transport failed (on target: an LEDC duty-set error). Host recorders never return this.
  Transport,
}

/// Sink for a normalized analog level driven as a PWM duty cycle (the WS55-220 spindle speed line on target,
/// DOC-07). Implemented over LEDC timer0/channel0 on GPIO13 on target (the duty conditioned to 0–10 V by an
/// external RC + op-amp stage) and as a recording buffer in host tests.
///
/// ## Contract
/// - [`set_duty`](PwmSink::set_duty) takes a normalized `frac` in `0.0..=1.0`: `0.0` is full off (0 V, spindle
///   stopped) and `1.0` is full scale (10 V, max RPM). The caller (the spindle controller) is responsible for
///   CLAMPING `frac` into range before calling, so an implementation always receives a valid duty and never has
///   to reject one; a defensive impl may still clamp, but must not error on an out-of-range value.
/// - The sink is purely a consumer of a decided duty; the RPM→duty mapping and all M3/M4/M5 sequencing live in
///   the host-tested [`SpindleController`](crate::spindle::SpindleController) so they stay off-target testable.
pub trait PwmSink {
  /// Set the PWM duty from a normalized `frac` in `0.0..=1.0`. Returns [`PwmError::Transport`] on a hardware
  /// failure. The caller guarantees `frac` is already clamped into range.
  fn set_duty(&mut self, frac: f32) -> Result<(), PwmError>;
}

/// The runtime configuration of a limit input that the host-tested homing/hard-limit logic needs (DOC-06). It
/// folds the one grblHAL limit `$`-setting the trigger READ depends on:
/// - `$5` (`invert`): Galdr wires Normally-Closed (NC) micro-switches to GND with an internal pull-up, so an
///   intact untriggered switch holds the pin LOW and opening it (or a broken wire) lets the pull-up raise it
///   HIGH = triggered (the documented broken-wire fail-safe, DOC-06). `$5=1` inverts that decision so a board
///   wired Normally-Open — or a limit GPIO jumpered to GND during bench bring-up — also reads correctly.
///
/// Unlike [`ProbeConfig`] there is no pull-up-disable knob: limit inputs always enable the internal pull-up
/// (the NC fail-safe depends on it), so this carries only the `$5` invert. The trigger decision lives in the
/// host-tested [`limit_triggered`] so the firmware bin's GPIO read and the ISR/seek paths agree on one sense.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LimitConfig {
  /// `$5` limit-pin invert. With `$5=0` (default) a HIGH pin reads triggered (NC switch open / broken wire);
  /// `$5=1` inverts that for a Normally-Open wiring or the bring-up jumper-to-GND case (DOC-06 bring-up note).
  pub invert: bool,
}

impl Default for LimitConfig {
  /// grbl's limit default: no invert (`$5=0`), i.e. the NC convention where an open switch / broken wire reads
  /// HIGH = triggered. A Normally-Open wiring (or a bring-up GND jumper) needs `$5=1` set explicitly.
  fn default() -> Self {
    LimitConfig { invert: false }
  }
}

/// Decide whether a limit switch is currently TRIGGERED from its RAW electrical level and the limit config,
/// applying the `$5` invert. This is the single host-tested place the invert is honored so the firmware bin's
/// GPIO read, the rising-edge limit ISR, and the homing seek/locate sampling all agree on the trigger sense.
/// `raw_high` is the pin level directly off the input (`true` = electrically high).
///
/// The base (`$5=0`) sense is Galdr's NC convention: a closed (intact, untriggered) switch grounds the pin LOW,
/// so a HIGH pin means the switch opened (axis at the limit) OR the wire broke — both must read TRIGGERED, which
/// is the broken-wire fail-safe (DOC-06). This is the OPPOSITE base polarity from [`probe_triggered`] (the probe
/// idles high), so the two must not be conflated even though both apply their invert in one place. `$5=1` is a
/// pure electrical invert of the decision for a Normally-Open wiring or the bench jumper-to-GND bring-up case.
pub fn limit_triggered(raw_high: bool, config: &LimitConfig) -> bool {
  // NC base: HIGH = triggered (switch open or broken wire), LOW = not triggered (intact closed switch). `$5`
  // inverts the whole decision so a NO wiring (idles low, rises on trigger only via an external pull-down) or a
  // GND-jumpered bring-up pin reads correctly. DOC-06's bring-up note documents `$5=1` as the jumper workaround.
  raw_high ^ config.invert
}

/// A limit / control digital input (DOC-06). Implemented over a single GPIO with an internal pull-up and a
/// rising-edge interrupt on target (X/Y/Z limit = GPIO10/11/12 per the DOC-00 manifest) and as a scripted
/// recording mock in host tests. Like [`ProbeInput`], the trait exposes ONLY the RAW electrical level; the `$5`
/// invert is applied by the host-tested [`limit_triggered`] so the firmware impl carries zero settings knowledge
/// and the trigger sense stays unit-tested. The homing seek/locate walker samples this BETWEEN single-step
/// bursts (mirroring the probe cycle), and the hard-limit ISR turns its rising edge into a `LIMIT_TRIGGERED`
/// Signal that the firmware bin resamples after the `$26` debounce window before accepting.
pub trait DigitalIn {
  /// The raw electrical level of the input pin: `true` = high, `false` = low. The `$5`-adjusted trigger decision
  /// is made by [`limit_triggered`]; this reader is deliberately invert-agnostic so the policy lives in one place.
  fn is_high(&self) -> bool;
}

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

/// Errors a [`DigitalOut`] may return. Like [`StepError`]/[`PwmError`] these are recoverable by the caller (the
/// spindle task), surfaced rather than panicked, per the firmware-core no-`unwrap` rule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DigitalOutError {
  /// The underlying output transport failed (on target: a GPIO set error). Host recorders never return this.
  Transport,
}

/// A digital output line (the stepper-enable line and the spindle ENABLE / DIRECTION lines, DOC-07). Implemented
/// over a single `esp_hal::gpio::Output` on target and as a recording mock in host tests.
///
/// ## Logical-level contract
/// The trait deals in the LOGICAL level only: `true` means ASSERTED (the function is active), `false` means
/// de-asserted. The PHYSICAL polarity is the firmware implementation's job, NOT the controller's — e.g. the
/// spindle ENABLE line (`SPIN_EN`, GPIO14) is active-LOW (the WS55-220 runs when its EN terminal is pulled to
/// GND), so the firmware impl drives the pin LOW for a logical `true` (run). For the DIRECTION line a logical
/// `true` is the controller's chosen CW convention (see [`SpindleController`](crate::spindle::SpindleController));
/// the firmware impl maps that to whichever pin level the F/R input expects. Keeping the trait invert-agnostic
/// means the host-tested controller carries zero board-polarity knowledge, exactly as [`DigitalIn`] does for inputs.
pub trait DigitalOut {
  /// Drive the output to the given LOGICAL level (`true` = asserted/active). Returns [`DigitalOutError::Transport`]
  /// on a hardware failure. Physical inversion (e.g. active-low enable) is applied by the implementation.
  fn set(&mut self, level: bool) -> Result<(), DigitalOutError>;
}

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

  // The `MAX_SYMBOLS_PER_BURST` never-completely-full-block invariant is unit-tested in
  // `cnc_kinematics::step`, where the constant is now defined.

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

  // ---- Limit input: `$5` invert / NC fail-safe semantics (DOC-06) ---------------------------------

  #[test]
  fn limit_default_config_is_nc_high_triggered() {
    // Galdr wires NC switches to GND with an internal pull-up: an INTACT, untriggered switch holds the pin
    // LOW; opening the switch OR a broken wire lets the pull-up pull the pin HIGH = triggered. So with `$5=0`
    // a HIGH pin reads triggered and a LOW pin reads not-triggered — the OPPOSITE of the probe's base sense,
    // because the probe idles high while a NC limit idles low. This is the documented broken-wire fail-safe.
    let cfg = LimitConfig::default();
    assert!(!cfg.invert);
    assert!(limit_triggered(true, &cfg), "open switch / broken wire (high) reads triggered under $5=0");
    assert!(!limit_triggered(false, &cfg), "intact closed NC switch (low) reads not-triggered under $5=0");
  }

  #[test]
  fn limit_invert_flips_the_trigger_sense() {
    // `$5=1` is a pure electrical invert of the trigger decision, so a board wired Normally-Open (or jumpered
    // to GND during bring-up, per DOC-06's bring-up note) reads correctly. It must cleanly flip BOTH levels.
    let inverted = LimitConfig { invert: true };
    let base = LimitConfig::default();
    assert_eq!(limit_triggered(true, &inverted), !limit_triggered(true, &base), "$5 inverts the high decision");
    assert_eq!(limit_triggered(false, &inverted), !limit_triggered(false, &base), "$5 inverts the low decision");
    // Concretely: under `$5=1`, a low pin is triggered and a high pin is not (the bring-up jumper-to-GND case).
    assert!(limit_triggered(false, &inverted));
    assert!(!limit_triggered(true, &inverted));
  }

  #[test]
  fn limit_and_probe_idle_senses_are_opposite_under_no_invert() {
    // Sanity check that the two inputs genuinely differ at rest: with no invert, the probe idles HIGH and
    // reads not-triggered high, while the NC limit idles LOW and reads not-triggered low. They share the
    // single-place-invert pattern but NOT the base polarity — a future author must not copy one onto the other.
    let limit = LimitConfig::default();
    let probe = ProbeConfig::default();
    assert_ne!(limit_triggered(true, &limit), probe_triggered(true, &probe));
    assert_ne!(limit_triggered(false, &limit), probe_triggered(false, &probe));
  }
}
