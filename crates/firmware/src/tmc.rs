//! TMC2209 single-wire UART transport + manager task (DOC-03): the esp-hal wiring that carries the
//! host-tested [`firmware_core::drivers::tmc2209`] datagrams over UART1.
//!
//! This is the thin, non-host-testable adapter for the TMC2209 subsystem, mirroring how [`crate::motion`]
//! adapts the step generator and [`crate::comms`] adapts the streaming engine: ALL register logic, current
//! scaling, the init sequence, write verification, and status decoding live in firmware-core's
//! [`TmcManager`](firmware_core::drivers::tmc2209::manager::TmcManager) and are exercised by host tests; here
//! we only move datagram bytes between the UART1 peripheral and that logic through the
//! [`TmcBus`](firmware_core::hal_traits::TmcBus) trait, and run the bring-up / polling task.
//!
//! ## Single-wire half-duplex on GPIO9 (DOC-03 bus topology)
//! All three drivers share one PDN_UART node driven by UART1 on GPIO9. There is exactly one physical wire, so
//! the same GPIO carries both TX and RX: the pin is degraded to an [`AnyPin`](esp_hal::gpio::AnyPin) and two
//! further handles are cloned — one routed to the UART RX signal, one kept by the bus to switch the pad's drive
//! mode per exchange (an esp-hal boundary use of `unsafe`, permitted by CLAUDE.md — only one pin is ever
//! physically present, and the handles merely route the one line to the UART TX/RX signals through the GPIO
//! matrix and configure the shared pad).
//!
//! The pad drive is **half-duplex push-pull**: [`Uart1TmcBus::write_all`] drives GPIO9 push-pull for exactly
//! the duration of each transmit, then releases it to open-drain + pull-up before the reply window opens. Both
//! earlier static schemes failed on the bench and each failure informs this hybrid:
//! - **Always-push-pull** blocked every reply: the pad actively holds the line high during the reply window,
//!   and the driver — sourcing through the 1 kΩ series resistor (DOC-03, which also caps contention current) —
//!   cannot win the pad, so `[MSG:TMC-PROBE …]` read `no-reply` on every node (echo looped back, nothing
//!   answered). Hence the release to open-drain BEFORE the reply window: the pad then drives only the low
//!   level and floats high, so the pull-up returns the line to idle and a driver can assert its low.
//! - **Always-open-drain** transmitted with RC-limited rising edges (pull-up τ against bus capacitance) that
//!   glitched/framed even our own fixed-rate self-echo at 115200. The TMC2209 re-measures its baud from the
//!   sync byte (`0x05`) of EVERY request, so those soft edges plausibly defeat its auto-baud — it decodes
//!   garbage and never replies (`no-reply(fifo:0)` on all nodes; see
//!   `docs/tmc-uart-silent-driver-investigation.md`, leading hypothesis). The proven-working RAMPS reference
//!   config transmitted push-pull. Hence push-pull DURING the transmit: sharp edges both directions for the
//!   auto-baud, exactly like RAMPS, while still releasing the wire for the reply.
//! The swap points are safe: the UART TX signal idles high in both modes (driven high vs pulled high, no edge),
//! and `flush()` returns only after the TX FSM goes idle (last stop bit fully on the wire) while the driver's
//! `SENDDELAY` holds the reply off for ≥ 8 bit-times (~70 µs at 115200) — ample time for one pad-register write.
//! The internal pull-up (~45 kΩ) alone is weak for 115200 against breadboard capacitance, but it no longer
//! shapes the transmit (push-pull drives both edges); the external ~4.7 kΩ bus pull-up still matters for the
//! DRIVER's open-drain reply edges (see [`init`]). Because TX and RX share the wire, every transmitted byte is
//! echoed back and MUST be discarded before a reply is read: [`Uart1TmcBus::drain_and_clear`] drains the
//! looped-back bytes AND clears any latched glitch/framing flag after each transmit, leaving only the driver's
//! reply to be read and judged for line quality.
//!
//! ## Async, non-stalling transport (the turn-around is awaited, not busy-spun)
//! The [`TmcBus`] trait is `async` (DOC-09), so [`Uart1TmcBus`] uses the non-blocking UART FIFO API and a
//! short poll loop that AWAITS an [`embassy_time::Timer`] between FIFO reads to bound how long it waits for a
//! reply. A driver that never answers (standalone VREF mode / unwired) yields [`TmcError::Timeout`] after the
//! 5 ms budget rather than hanging. Awaiting the inter-poll timer (instead of busy-spinning a `delay_micros`)
//! yields the single core-0 thread executor during the bus turn-around, so the comms / parser / planner tasks
//! keep running while a datagram round-trip is in flight — an absent driver no longer stalls the executor for
//! up to 5 ms. The exchanges are infrequent (a few round-trips at init and one `DRV_STATUS` read per axis per
//! poll interval), entirely off the real-time path; the core-1 step generation is wholly unaffected.
//!
//! ## Breadboard bring-up (BTT / stepstick drivers)
//! The default build targets the Adafruit 6121 breakout (0.05 Ω sense). When bringing the hardware up on a
//! breadboard with BTT / Watterott / FYSETC TMC2209 **stepstick** modules (0.11 Ω sense), flip the
//! `BREADBOARD_STEPSTICKS` toggle in [`TmcConfig::default`](firmware_core::drivers::tmc2209::manager::TmcConfig)
//! so the current-scale math uses the right sense resistor — a wrong value mis-scales `IRUN`/`IHOLD` by ≈ 2.2×
//! and can overheat the motor. The wiring differences (VIO → 3.3 V, MS1/MS2 address straps, the single 1 kΩ
//! UART series resistor, and keeping motor-coil current OFF the breadboard rails) are in
//! `docs/breadboard-bringup.md`. Note `tmc_r_sense_ohms` is a persisted setting: the compile-time default only
//! seeds a fresh/erased flash, so on a board with settings already stored, push the value over the `$PBX`
//! host-sync channel (or erase flash) rather than relying on the rebuild alone.

// Under the `tmc-tx-diag` / `tmc-scope-diag` diagnostic builds, `tmc_manager` is replaced by the continuous-TX
// or repeating-request loop and some/all of the normal path (init/presence/poll plus its helpers and imports) is
// compiled out, leaving those items unreferenced. Suppress the EXPECTED dead-code / unused-import warnings for
// THOSE builds only — the default build keeps full strictness.
#![cfg_attr(any(feature = "tmc-tx-diag", feature = "tmc-scope-diag"), allow(dead_code, unused_imports))]

use embassy_time::{Duration, Timer};
use esp_hal::gpio::interconnect::OutputSignal;
use esp_hal::gpio::{DriveMode, OutputConfig, Pin, Pull};
use esp_hal::uart::{Config, RxError, Uart};
use esp_hal::Blocking;

use firmware_core::drivers::tmc2209::manager::{classify_ioin, AxisReport, TmcConfig, TmcManager, AXIS_COUNT};
use firmware_core::drivers::tmc2209::{decode_read_reply, encode_read_request, encode_write, TmcError, READ_REPLY_LEN};
use firmware_core::hal_traits::TmcBus;
use firmware_core::protocol::{
  BusErrorKind, BusStats, DriverProbe, DriverStatus, InitFailure, IoStage, IoinRawCapture, LoopbackReport,
  RxErrorKind, TmcProbeStage,
};

use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};
// Xtensa has no native 64-bit atomics; `portable-atomic` (already a firmware dep) supplies `AtomicU64` for the
// two 64-bit `IOIN` diagnostic snapshots (packed per-axis outcomes and the raw reply). Its `Ordering` is a
// re-export of `core`'s, so it mixes freely with the `core` atomics above.
use portable_atomic::AtomicU64;

/// Live per-axis TMC2209 bus health, published by [`tmc_manager`] after the init burst and REFRESHED every poll
/// round, so the comms layer can render the `$I+` `[DRIVER:]` report without re-touching the UART bus (a read
/// from the comms task would race the manager's polling and the half-duplex turn-around). Bit `i` set ⇒ axis
/// `i`'s driver is currently communicating AND not reporting a hard `DRV_STATUS` fault; the poll loop clears the
/// bit on an over-temp / short / open-load fault or a lost reply, and re-sets it on the next clean read, so the
/// report tracks live health rather than a frozen boot snapshot. A single `u8` covers all [`AXIS_COUNT`] axes
/// (≤ 4 today) — see the guard below. The mask is stored `Release` and loaded `Acquire`, paired with
/// [`TMC_INIT_DONE`] (stored last, loaded first) so a reader that sees init done also sees the populated bits.
static TMC_ONLINE_MASK: AtomicU8 = AtomicU8::new(0);

// The `u8` mask has one bit per axis, so it only covers AXIS_COUNT <= 8 axes. AXIS_LETTERS advertises axis
// changes as a "one-line change"; couple that promise to this second site with a compile-time guard so bumping
// AXIS_COUNT past 8 fails to build here (a loud error) instead of silently overflowing `1 << axis`.
const _: () = assert!(AXIS_COUNT <= 8, "TMC_ONLINE_MASK is a u8; widen it if AXIS_COUNT exceeds 8 axes");

/// `true` once [`tmc_manager`]'s init pass has populated [`TMC_ONLINE_MASK`]. Until then a `$I+` query reports
/// `init pending` instead of a misleading "all absent". Stored (`Release`) AFTER the mask so a reader that sees
/// `true` (via an `Acquire` load) is guaranteed to see the populated bits.
static TMC_INIT_DONE: AtomicBool = AtomicBool::new(false);

/// Live per-axis TMC2209 `IOIN` presence-read outcome for the `$I+` diagnostic. Sixteen bits per axis pack a
/// [`TmcProbeStage`] via [`to_bits`](TmcProbeStage::to_bits) (a discriminant tag plus the version byte for a
/// mismatch); see the guard below. Populated by [`tmc_manager`]'s init pass and REFRESHED by the poll loop's
/// live re-probe of absent nodes, so `[MSG:TMC-PROBE]` tracks a driver appearing mid-session (a present node's
/// outcome stays its last probe result — the `[DRIVER:]` mask is the live health signal for those). Stored
/// `Release` before [`TMC_INIT_DONE`] so a reader that sees init done also sees the packed outcomes.
static TMC_IOIN_STAGES: AtomicU64 = AtomicU64::new(0);

// TMC_IOIN_STAGES packs 16 bits per axis into a u64, so it only covers AXIS_COUNT <= 4 axes. Guard it so bumping
// AXIS_COUNT past 4 fails to build here (a loud error) instead of silently truncating the packed outcomes.
const _: () = assert!(AXIS_COUNT <= 4, "TMC_IOIN_STAGES packs 16 bits/axis into a u64; widen it past 4 axes");

// The raw-reply capture packs the 8-byte IOIN datagram into a u64, so the reply length must be exactly 8.
const _: () = assert!(READ_REPLY_LEN == 8, "TMC_IOIN_RAW packs the reply into a u64; it must be 8 bytes");

/// The raw 8-byte `IOIN` reply of the first node whose presence read hit a [`TmcProbeStage::DecodeError`], packed
/// little-endian into a u64 for the `$I+` `[MSG:TMC-IOIN …]` framing dump. Meaningful only when
/// [`TMC_IOIN_RAW_AXIS`] is not [`NO_RAW_CAPTURE`]. Published `Release` alongside the stages.
static TMC_IOIN_RAW: AtomicU64 = AtomicU64::new(0);

/// The axis index whose raw reply is held in [`TMC_IOIN_RAW`], or [`NO_RAW_CAPTURE`] when no node had a decode
/// error (the common case). Published `Release` alongside the stages.
static TMC_IOIN_RAW_AXIS: AtomicU8 = AtomicU8::new(NO_RAW_CAPTURE);

/// Sentinel for [`TMC_IOIN_RAW_AXIS`] meaning "no decode-error reply was captured this init pass".
const NO_RAW_CAPTURE: u8 = 0xFF;

/// The failing bus operation of the first node whose init hit a bare `InitError`, packed via
/// [`InitFailure::to_bits`] (axis + register + read/write + error kind) for the `$I+` `err:<w|r><REG>:<kind>`
/// diagnostic. `0` (validity bit clear) means no node was a bare `InitError`. Published `Release` alongside the
/// other snapshots so a reader that sees init done also sees it.
static TMC_INIT_FAIL: AtomicU32 = AtomicU32::new(0);

/// The boot loopback self-test result, packed via [`LoopbackReport::to_bits`] (ran flag + echoed/matched byte
/// counts + first RX-error variant) for the `$I+` `[MSG:TMC-LOOPBACK …]` line. `0` (ran bit clear) means the
/// test has not run yet. Published `Release` before [`TMC_INIT_DONE`].
static TMC_LOOPBACK: AtomicU32 = AtomicU32::new(0);

/// Per-axis FAILED bus-exchange counts for the `$I+` `[MSG:TMC-BUS …]` margin meter, 16 saturating bits per
/// axis (the same u64 packing — and the same `AXIS_COUNT <= 4` guard — as [`TMC_IOIN_STAGES`]). The poll loop
/// owns the live values and republishes both counters each round; see [`BusStats`] for what counts as a
/// failure and why the meter exists.
static TMC_BUS_FAILS: AtomicU64 = AtomicU64::new(0);

/// Per-axis ATTEMPTED bus-exchange counts, the denominator paired with [`TMC_BUS_FAILS`]. Same packing.
static TMC_BUS_TOTALS: AtomicU64 = AtomicU64::new(0);

/// Snapshot the live per-axis TMC2209 bus health for the `$I+` `[DRIVER:]` report. Lock-free: reads the two
/// atomics the manager publishes, so the comms task never touches the UART bus directly. The `Acquire` load of
/// [`TMC_INIT_DONE`] pairs with the manager's `Release` store so seeing `initialized` implies seeing the mask.
pub fn driver_status() -> DriverStatus {
  let initialized = TMC_INIT_DONE.load(Ordering::Acquire);
  let mask = TMC_ONLINE_MASK.load(Ordering::Acquire);
  // Decode the per-axis bits with the same `from_fn` idiom as `comms::limit_levels`, so the bit layout
  // (`bit0 = X`, `bit1 = Y`, …) has one shared decoder shape across both reports.
  let online: [bool; AXIS_COUNT] = core::array::from_fn(|axis| mask & (1 << axis) != 0);
  DriverStatus { online, initialized }
}

/// Snapshot the per-axis TMC2209 `IOIN` presence-read outcomes (plus the raw framing dump for a decode-error
/// node) for the `$I+` diagnostic. Lock-free like [`driver_status`]: it reads the atoms the manager publishes so
/// the comms task never touches the UART bus. The `Acquire` load of [`TMC_INIT_DONE`] pairs with the manager's
/// `Release` store, so observing `initialized` implies observing the packed outcomes and the raw capture.
pub fn driver_probe() -> DriverProbe {
  let initialized = TMC_INIT_DONE.load(Ordering::Acquire);
  let packed = TMC_IOIN_STAGES.load(Ordering::Acquire);
  let stages = core::array::from_fn(|axis| TmcProbeStage::from_bits((packed >> (axis * 16)) as u16));
  // The raw framing dump is present only when a node hit a decode error; decode the captured axis + bytes.
  let raw_axis = TMC_IOIN_RAW_AXIS.load(Ordering::Acquire);
  let raw_ioin = if raw_axis == NO_RAW_CAPTURE {
    None
  } else {
    Some(IoinRawCapture { axis: raw_axis as usize, bytes: TMC_IOIN_RAW.load(Ordering::Acquire).to_le_bytes() })
  };
  // The enriched init-failure detail for a bare-`err` node (validity bit clear ⇒ `None`).
  let init_failure = InitFailure::from_bits(TMC_INIT_FAIL.load(Ordering::Acquire));
  DriverProbe { stages, initialized, raw_ioin, init_failure }
}

/// Snapshot the boot loopback self-test result for the `$I+` `[MSG:TMC-LOOPBACK …]` line. Lock-free, like the
/// other TMC snapshots.
pub fn loopback_report() -> LoopbackReport {
  LoopbackReport::from_bits(TMC_LOOPBACK.load(Ordering::Acquire))
}

/// Snapshot the per-axis bus-exchange counters for the `$I+` `[MSG:TMC-BUS …]` margin meter. Lock-free, like
/// the other TMC snapshots. The two u64s are loaded independently, so a snapshot taken mid-round can be one
/// exchange out of step between `fail` and `total` — harmless for a diagnostic rate readout.
pub fn bus_stats() -> BusStats {
  let fails = TMC_BUS_FAILS.load(Ordering::Acquire);
  let totals = TMC_BUS_TOTALS.load(Ordering::Acquire);
  BusStats {
    fail: core::array::from_fn(|axis| (fails >> (axis * 16)) as u16),
    total: core::array::from_fn(|axis| (totals >> (axis * 16)) as u16),
  }
}

/// Per-datagram timeout in microseconds: how long [`Uart1TmcBus`] busy-polls for a reply before declaring the
/// node silent. A full 8-byte reply at 115200 baud streams in ≈ 700 µs, and the driver inserts a `SENDDELAY`
/// turn-around on top, so 5 ms is comfortable headroom while still failing an absent node fast.
const BUS_TIMEOUT_US: u32 = 5_000;

/// Busy-poll granularity in microseconds between non-blocking FIFO reads while filling a datagram buffer.
/// Small enough that a reply is picked up promptly, large enough that the poll is not a tight spin.
const POLL_STEP_US: u32 = 50;

/// How often the manager task re-reads `DRV_STATUS` on every present driver to surface over-temperature /
/// short / open-load conditions. One second is responsive for thermal/fault reporting without loading the bus.
const STATUS_POLL_INTERVAL: Duration = Duration::from_secs(1);

/// The UART1-backed [`TmcBus`]: an async single-wire transport for the shared TMC2209 bus. It frames each
/// datagram through the host-tested codec, drives the bytes onto the line, discards the half-duplex echo, and
/// (for reads) collects and validates the reply, awaiting an [`embassy_time::Timer`] between FIFO polls so the
/// bus turn-around yields the executor rather than busy-spinning.
pub struct Uart1TmcBus {
  /// The blocking UART1 driver. Both its TX and RX signals route to the single GPIO9 line (see module docs).
  /// Its FIFO ops are non-blocking (`write`/`read_buffered` move whatever fits and return), so the reply wait
  /// is realized by awaiting a timer between polls — never by spinning a blocking delay.
  uart: Uart<'static, Blocking>,
  /// The third GPIO9 handle, used only to swap the shared pad between push-pull (during a transmit, for sharp
  /// auto-baud edges) and open-drain + pull-up (the released idle/reply state). See the module docs for why the
  /// drive mode is per-exchange rather than static. `apply_output_config` touches only the pad drive-mode /
  /// pull registers, never the TX/RX signal routing established at [`init`].
  pad: OutputSignal<'static>,
  /// The outcome the most recent [`read_reg`](Uart1TmcBus::read_reg) exchange reached, at the transport layer:
  /// `Responded` on a clean decode, `EchoTimeout`/`ReplyTimeout` on a stage timeout, or `DecodeError` when a
  /// full reply arrived but failed to decode. Reset at the start of every read and set on failure, so the init
  /// task can sample it after each per-axis presence read to feed [`classify_ioin`] for the `$I+` diagnostic.
  /// Observe-only; it never affects the bus's `Result`. Version-mismatch is NOT visible here (the transport
  /// decodes but does not check `VERSION`) — the manager folds that in.
  last_probe_stage: TmcProbeStage,
  /// The raw reply datagram of the most recent read that filled a full reply buffer, captured before decode so
  /// the init task can surface a decode-error node's exact bytes on `$I+`. Only meaningful for a `DecodeError`
  /// outcome (where a full reply was read); stale otherwise, but never surfaced in that case.
  last_reply: [u8; READ_REPLY_LEN],
  /// The register byte the most recent bus operation targeted. Recorded at the start of every read/write so the
  /// init task can name the failing register on a bare `InitError` (`err:wGSTAT:to`). Observe-only.
  last_op_reg: u8,
  /// `true` if the most recent bus operation was a register write, `false` if a read (its request transmit).
  last_op_was_write: bool,
  /// The error kind of the most recent bus operation, or `None` if it succeeded. Set on every failure path of
  /// read/write so the init task can name the failure kind (`to`/`io`/`dec`) for a bare `InitError`.
  last_error_kind: Option<BusErrorKind>,
  /// The specific UART RX-error variant of the most recent `read_filling` failure, or `None` if the last read
  /// had no RX error (cleared at the start of each op). Only meaningful alongside a [`BusErrorKind::Io`]; lets
  /// the init task append `@<stage><variant>` so an `io` names glitch / framing / overflow / parity.
  last_rx_error: Option<RxErrorKind>,
  /// Which half-duplex read stage (echo vs reply) the most recent read was in, so an `Io` can be attributed to
  /// the echo or the reply — different faults with different fixes. Set immediately before each `read_filling`.
  last_io_stage: IoStage,
  /// The first RX glitch/framing variant the TOLERANT reply fill tripped this read, or `None` if the reply came
  /// in clean. Reset at the start of each read; `read_filling` records it without aborting, and `read_reg` folds
  /// it into the outcome after decode (`ok(<variant>)` if the datagram still verified, `crc(<variant>)` if not),
  /// so reliance on RX tolerance is surfaced on `$I+`, never silently swallowed.
  reply_glitched: Option<RxErrorKind>,
}

impl Uart1TmcBus {
  /// Wrap a configured blocking UART1 instance as a TMC bus. `pad` is the spare GPIO9 handle whose drive mode
  /// the bus swaps around each transmit; [`init`] hands it over already released (open-drain + pull-up).
  pub fn new(uart: Uart<'static, Blocking>, pad: OutputSignal<'static>) -> Self {
    Uart1TmcBus {
      uart,
      pad,
      last_probe_stage: TmcProbeStage::Responded,
      last_reply: [0u8; READ_REPLY_LEN],
      last_op_reg: 0,
      last_op_was_write: false,
      last_error_kind: None,
      last_rx_error: None,
      last_io_stage: IoStage::Echo,
      reply_glitched: None,
    }
  }

  /// The transport-layer outcome the most recent read reached (`Responded` on a clean decode). Sampled by the
  /// init task after each per-axis presence probe, then folded with the manager's report by [`classify_ioin`].
  pub fn last_probe_stage(&self) -> TmcProbeStage {
    self.last_probe_stage
  }

  /// The raw 8-byte reply of the most recent read that filled a full reply buffer. Sampled by the init task for
  /// a node whose [`last_probe_stage`](Self::last_probe_stage) is [`TmcProbeStage::DecodeError`], to publish the
  /// framing dump.
  pub fn last_reply_bytes(&self) -> [u8; READ_REPLY_LEN] {
    self.last_reply
  }

  /// The register byte the most recent bus operation targeted. Sampled by the init task to name a bare
  /// `InitError` node's failing register.
  pub fn last_op_reg(&self) -> u8 {
    self.last_op_reg
  }

  /// Whether the most recent bus operation was a write (`true`) or a read (`false`).
  pub fn last_op_was_write(&self) -> bool {
    self.last_op_was_write
  }

  /// The error kind of the most recent bus operation (`None` if it succeeded). Sampled by the init task to name
  /// a bare `InitError` node's failure kind.
  pub fn last_error_kind(&self) -> Option<BusErrorKind> {
    self.last_error_kind
  }

  /// The UART RX-error variant of the most recent read (`None` if it had no RX error). Sampled by the init task
  /// alongside [`last_error_kind`](Self::last_error_kind) to append the `@<stage><variant>` detail on an `Io`.
  pub fn last_rx_error(&self) -> Option<RxErrorKind> {
    self.last_rx_error
  }

  /// The half-duplex read stage (echo vs reply) the most recent read was in. Sampled by the init task to
  /// attribute an RX `Io` to the echo or the reply.
  pub fn last_io_stage(&self) -> IoStage {
    self.last_io_stage
  }

  /// Drain any bytes left in the RX FIFO AND clear latched RX-error flags, before a fresh exchange and after each
  /// transmit. Unlike the old `drain_rx` (whose `while let Ok(..)` no-oped the instant a latched glitch/framing
  /// flag made `read_buffered` return `Err`, leaving the flag set to poison the next read), this treats an `Err`
  /// as the flag-clearing side effect it is — `read_buffered`'s `check_for_errors` is the only public clear seam
  /// in esp-hal 1.1.1 — and RETRIES until the FIFO reads clean. So the open-drain self-echo's glitch/framing flag
  /// is cleared here rather than surfacing as a fatal error on the reply read. Best-effort housekeeping: swallowed
  /// errors do NOT touch `last_error_kind`/`last_rx_error`. Bounded to avoid a pathological spin if the FIFO never
  /// settles (32 iterations is far more than an 8-byte echo plus a couple of flag clears needs).
  fn drain_and_clear(&mut self) {
    let mut scratch = [0u8; 16];
    for _ in 0..32 {
      match self.uart.read_buffered(&mut scratch) {
        // FIFO empty and no latched error remaining: the line is clean, so stop.
        Ok(0) => break,
        // Drained stale/echo bytes; keep going until the FIFO is empty.
        Ok(_) => continue,
        // `read_buffered` reported (and, as a side effect, cleared) a latched RX-error flag; retry to confirm the
        // line is now clean. This is exactly the case the old `drain_rx` mishandled.
        Err(_) => continue,
      }
    }
  }

  /// After a reply timeout, robustly drain whatever is in the RX FIFO into a scratch buffer — clearing any
  /// latched glitch/framing flag along the way (the same `Err => continue` pattern as [`drain_and_clear`], since
  /// `read_buffered`'s `check_for_errors` is the only clear seam) — and report what was there. Returns the total
  /// byte count (saturating `u8`) and the first up to [`READ_REPLY_LEN`] bytes. This is the discriminator for the
  /// `$I+` `no-reply(fifo:N)` diagnostic: `fifo:0` ⇒ a genuinely silent driver, `fifo:N>0` with decodable bytes
  /// ⇒ the driver DID reply and the read strategy gated it. Bounded to 32 iterations. Best-effort: it does not
  /// touch the error-kind captures.
  fn drain_snapshot(&mut self) -> (u8, [u8; READ_REPLY_LEN]) {
    let mut captured = [0u8; READ_REPLY_LEN];
    let mut filled = 0usize;
    let mut total: u8 = 0;
    let mut scratch = [0u8; 16];
    for _ in 0..32 {
      match self.uart.read_buffered(&mut scratch) {
        // FIFO empty and clean: nothing (more) to snapshot.
        Ok(0) => break,
        Ok(read) => {
          for &byte in &scratch[..read] {
            // Keep the first `READ_REPLY_LEN` bytes for the eyeball dump; count all of them (saturating).
            if filled < captured.len() {
              captured[filled] = byte;
              filled += 1;
            }
            total = total.saturating_add(1);
          }
        }
        // A latched RX-error flag was reported (and cleared) instead of bytes; retry — the reply bytes, if any,
        // become readable once the flag is cleared.
        Err(_) => continue,
      }
    }
    (total, captured)
  }

  /// Boot loopback self-test: transmit [`TMC_LOOPBACK_PATTERN`] and read back its half-duplex self-echo on the
  /// shared GPIO9, reporting how many bytes returned, how many matched position-by-position, and the first RX
  /// error variant seen. Proves the MCU TX+RX path and line levels WITHOUT a scope — a full match means TX, RX,
  /// and edges are healthy and any remaining fault is driver-side; `got:0` means the MCU half is broken; a
  /// partial/mismatch means TX transmits but the levels/edges are marginal. Safe with drivers attached: the
  /// pattern is not a valid addressed request, so no driver replies. Tolerant of RX glitches (records the first
  /// variant, keeps polling) so a marginal edge still reports how much survived. Gated to the normal build (the
  /// `tmc-tx-diag` / `tmc-scope-diag` builds replace `tmc_manager` before this is ever reached).
  #[cfg(not(any(feature = "tmc-tx-diag", feature = "tmc-scope-diag")))]
  async fn loopback(&mut self) -> LoopbackReport {
    use firmware_core::protocol::TMC_LOOPBACK_PATTERN;
    self.drain_and_clear();
    if self.write_all(&TMC_LOOPBACK_PATTERN).is_err() {
      // The transmit failed outright, so nothing can echo back.
      return LoopbackReport { ran: true, got: 0, matched: 0, err: None };
    }
    let mut buf = [0u8; TMC_LOOPBACK_PATTERN.len()];
    let mut filled = 0usize;
    let mut waited_us = 0u32;
    let mut err: Option<RxErrorKind> = None;
    while filled < buf.len() {
      let progressed = match self.uart.read_buffered(&mut buf[filled..]) {
        Ok(0) => false,
        Ok(read) => {
          filled += read;
          true
        }
        Err(rx) => {
          if err.is_none() {
            err = Some(map_rx_error(rx));
          }
          false
        }
      };
      if !progressed {
        if waited_us >= BUS_TIMEOUT_US {
          break;
        }
        Timer::after(Duration::from_micros(POLL_STEP_US as u64)).await;
        waited_us += POLL_STEP_US;
      }
    }
    let matched = (0..filled).filter(|&i| buf[i] == TMC_LOOPBACK_PATTERN[i]).count() as u8;
    LoopbackReport { ran: true, got: filled as u8, matched, err }
  }

  /// Continuous-TX DMM diagnostic (feature `tmc-tx-diag`): transmit `0x00` bytes back-to-back forever, keeping
  /// the TX FIFO saturated, so a multimeter on GPIO9 reads the drooped average (each `0x00` frame is a start bit
  /// plus eight zero data bits — 9 of every 10 bit-times low). Isolates the TX half; no RX. A brief timer yield
  /// between refills keeps the core-0 executor (and any watchdog feed) alive without letting the deep FIFO drain
  /// (a 64-byte fill takes several ms to clock out at 115200, far longer than the yield), so GPIO9 hammers low
  /// continuously. The low-duty average is baud-independent, so this holds at whatever `init` baud is in effect.
  /// Never returns.
  #[cfg(feature = "tmc-tx-diag")]
  async fn tx_diag_forever(&mut self) -> ! {
    // Drive push-pull for the diag so the DMM sees the same TX drive the real transmit path now uses.
    self.pad_push_pull();
    let blast = [0u8; 64];
    loop {
      let _ = self.uart.write(&blast);
      Timer::after(Duration::from_micros(100)).await;
    }
  }

  /// Repeating-request instrument diagnostic (feature `tmc-scope-diag`): loop *send `IOIN` read to node 0 →
  /// wait ~50 ms → repeat, forever* — a stable ~20 Hz burst a scope / logic analyzer can trigger on and watch
  /// as a repeating trace (`docs/tmc-uart-scope-bringup.md` §2). Uses the full production
  /// [`read_reg`](TmcBus::read_reg) path — the half-duplex push-pull transmit, the echo drain, and the reply
  /// wait — so the captured wire behavior is exactly what the real init/poll exchanges do. Replies (or their
  /// absence) are deliberately ignored; the instrument is the observer. Never returns.
  #[cfg(feature = "tmc-scope-diag")]
  async fn scope_diag_forever(&mut self) -> ! {
    use firmware_core::drivers::tmc2209::registers::IOIN;
    loop {
      let _ = self.read_reg(0, IOIN).await;
      Timer::after(Duration::from_millis(50)).await;
    }
  }

  /// Drive the shared GPIO9 pad push-pull for a transmit: sharp edges in BOTH directions so the driver's
  /// per-request auto-baud can measure the sync byte's bit period. MUST be paired with [`pad_release`]
  /// (Self::pad_release) before the reply window opens, or the pad blocks the driver's reply (module docs).
  fn pad_push_pull(&mut self) {
    self.pad.apply_output_config(&pad_config(DriveMode::PushPull));
  }

  /// Release the shared GPIO9 pad to open-drain + pull-up (the idle/reply state): the pad now drives only the
  /// low level and floats high, so a driver can pull the line low for its reply.
  fn pad_release(&mut self) {
    self.pad.apply_output_config(&pad_config(DriveMode::OpenDrain));
  }

  /// Write every byte of `bytes` with the pad driven push-pull for the exact duration of the transmit (the
  /// half-duplex push-pull scheme, module docs), then release the line for the reply window. The release runs
  /// on the error paths too — a pad left push-pull would silently block every subsequent reply, a far worse
  /// failure than the transmit error being reported.
  fn write_all(&mut self, bytes: &[u8]) -> Result<(), TmcError> {
    self.pad_push_pull();
    let result = self.transmit_flushed(bytes);
    self.pad_release();
    result
  }

  /// The transmit body of [`write_all`](Self::write_all): push every byte (looping because the FIFO may accept
  /// only part of the slice per call), then flush. `flush` returns only after the TX FSM goes idle — the last
  /// stop bit is fully on the wire — so the caller's pad release cannot truncate the datagram, and the driver's
  /// `SENDDELAY` (≥ 8 bit-times) means the reply cannot have started yet.
  fn transmit_flushed(&mut self, mut bytes: &[u8]) -> Result<(), TmcError> {
    while !bytes.is_empty() {
      match self.uart.write(bytes) {
        // A non-empty slice should always make progress; a zero-write would otherwise spin forever, so treat
        // it as a transport fault rather than looping.
        Ok(0) => return Err(TmcError::Io),
        Ok(written) => bytes = &bytes[written..],
        Err(_) => return Err(TmcError::Io),
      }
    }
    self.uart.flush().map_err(|_| TmcError::Io)
  }

  /// Fill `buf` completely from the RX FIFO (the driver's reply — the self-echo is no longer read back), polling
  /// with [`POLL_STEP_US`] granularity up to [`BUS_TIMEOUT_US`]. Uses the non-blocking `read_buffered` so an
  /// absent node times out cleanly instead of hanging; between non-productive polls it AWAITS a [`Timer`] rather
  /// than busy-spinning, yielding the core-0 executor during the turn-around.
  ///
  /// TOLERANT of RX glitch/framing errors: an open-drain reply may trip the glitch detector yet still carry
  /// intact bytes, so instead of aborting we record the first variant in [`reply_glitched`](Self::reply_glitched)
  /// and treat the errored poll like an empty one — `read_buffered`'s `check_for_errors` has already cleared the
  /// latched flag as a side effect, and it returned zero bytes. The integrity verdict is deferred to
  /// [`decode_read_reply`]'s CRC. The SAME [`BUS_TIMEOUT_US`] budget governs the errored path, so a pathological
  /// continuous glitch degrades to a clean [`TmcError::Timeout`] (→ `ReplyTimeout`) rather than a hang.
  async fn read_filling(&mut self, buf: &mut [u8]) -> Result<(), TmcError> {
    let mut filled = 0;
    let mut waited_us = 0;
    while filled < buf.len() {
      // Whether this poll advanced the buffer. A non-productive poll (empty FIFO or a tolerated glitch, both of
      // which yield zero bytes) shares one timeout-gated wait so the budget bounds the total wait either way.
      let progressed = match self.uart.read_buffered(&mut buf[filled..]) {
        Ok(0) => false,
        Ok(read) => {
          filled += read;
          true
        }
        // A tolerated RX glitch/framing error: record the first variant for the outcome token, do NOT abort. The
        // latched flag is already cleared by `check_for_errors`, and no bytes were read this poll.
        Err(error) => {
          if self.reply_glitched.is_none() {
            self.reply_glitched = Some(map_rx_error(error));
          }
          false
        }
      };
      if !progressed {
        if waited_us >= BUS_TIMEOUT_US {
          return Err(TmcError::Timeout);
        }
        Timer::after(Duration::from_micros(POLL_STEP_US as u64)).await;
        waited_us += POLL_STEP_US;
      }
    }
    Ok(())
  }
}

impl Uart1TmcBus {
  /// Record a bus error kind for the `$I+` init-failure diagnostic and pass the error straight back. Every
  /// read/write failure path routes through here so the captured kind (`to`/`io`/`dec`) is always in step with
  /// the returned `Result`. `TmcError` is `Copy`, so this neither clones nor swallows.
  fn note_error(&mut self, error: TmcError) -> TmcError {
    self.last_error_kind = Some(BusErrorKind::from_error(error));
    error
  }
}

/// The shared GPIO9 pad config for the given drive mode. The pull-up rides along in BOTH modes so the swap
/// between push-pull (transmit) and open-drain (release) never passes through a floating/pull-less state.
fn pad_config(mode: DriveMode) -> OutputConfig {
  OutputConfig::default().with_drive_mode(mode).with_pull(Pull::Up)
}

/// Map an esp-hal [`RxError`] to the pure firmware-core [`RxErrorKind`] for the `$I+` diagnostic. This is the one
/// place the specific RX-error variant is in scope — [`read_filling`](Uart1TmcBus::read_filling) flattens it to
/// [`TmcError::Io`] immediately after — so the variant is captured here before it is lost. `RxError` is
/// `#[non_exhaustive]`, hence the catch-all: an unknown future variant maps to the nearest signal-quality bucket
/// (`Glitch`) rather than failing to build.
fn map_rx_error(error: RxError) -> RxErrorKind {
  match error {
    RxError::FifoOverflowed => RxErrorKind::Overflow,
    RxError::GlitchOccurred => RxErrorKind::Glitch,
    RxError::FrameFormatViolated => RxErrorKind::Framing,
    RxError::ParityMismatch => RxErrorKind::Parity,
    _ => RxErrorKind::Glitch,
  }
}

impl TmcBus for Uart1TmcBus {
  /// Frame and send a write-access datagram. The TMC2209 sends no reply to a write, so success here means "on
  /// the wire"; acceptance is confirmed separately by the manager via `IFCNT`. The self-echo is not read back as
  /// a datagram (its slow open-drain rise glitches/frames at speed) — instead [`write_all`] flushes to TX-idle,
  /// then [`drain_and_clear`](Self::drain_and_clear) discards the looped-back bytes AND clears any latched
  /// self-echo glitch/framing flag so it cannot poison the next exchange.
  async fn write_reg(&mut self, node: u8, reg: u8, val: u32) -> Result<(), TmcError> {
    // Record the operation for the init-failure diagnostic: a write to `reg`, no error yet.
    self.last_op_reg = reg;
    self.last_op_was_write = true;
    self.last_error_kind = None;
    self.last_rx_error = None;
    self.drain_and_clear();
    let frame = encode_write(node, reg, val);
    if let Err(error) = self.write_all(&frame) {
      return Err(self.note_error(error));
    }
    // `write_all` flushed to TX-idle, so the whole echo is already latched; discard it and clear any self-echo
    // glitch/framing flag. Best-effort — a write has no reply to verify here (the manager does that via `IFCNT`).
    self.drain_and_clear();
    Ok(())
  }

  /// Frame and send a read-request datagram, discard its self-echo (draining + clearing any latched glitch/frame
  /// flag rather than reading it back as a datagram), then collect and decode the 8-byte reply. A node that never
  /// answers surfaces as [`TmcError::Timeout`] from [`read_filling`]; a malformed reply as the matching decode
  /// error from [`decode_read_reply`]. The reply read is TOLERANT of an RX glitch/framing error — the datagram
  /// may still be intact — so the CRC is the arbiter: a glitch that still decodes surfaces as
  /// `RespondedDespiteGlitch` (`ok(<variant>)`, proven alive), a glitch that corrupts surfaces as
  /// `DecodeErrorGlitched` (`crc(<variant>)`), distinct from a clean-line decode failure (`crc`).
  async fn read_reg(&mut self, node: u8, reg: u8) -> Result<u32, TmcError> {
    // Reset the per-exchange captures: a clean read leaves the stage `Responded` with no error; the failure
    // paths below set both the probe stage and the error kind so the init task can classify and name the fault.
    self.last_probe_stage = TmcProbeStage::Responded;
    self.last_op_reg = reg;
    self.last_op_was_write = false;
    self.last_error_kind = None;
    self.last_rx_error = None;
    self.reply_glitched = None;
    self.drain_and_clear();
    let request = encode_read_request(node, reg);
    // The read-request transmit. An `Io` here leaves the probe stage `Responded` (no read stage reached), so the
    // diagnostic falls back to `err:r<REG>:io` rather than a misleading timeout token.
    if let Err(error) = self.write_all(&request) {
      return Err(self.note_error(error));
    }
    // `write_all` flushed to TX-idle, so the self-echo is fully latched and the driver's `SENDDELAY` means the
    // reply has not begun — no trailing-byte race. Discard the echo AND clear any self-echo glitch/framing flag
    // here (the old fatal echo-datagram read is gone); the reply read below is where line quality is judged.
    self.drain_and_clear();
    // The driver's reply. A timeout means the driver never answered; an RX glitch/framing error is TOLERATED
    // (recorded in `reply_glitched`, not fatal) so the CRC below arbitrates whether the bytes survived.
    self.last_io_stage = IoStage::Reply;
    let mut reply = [0u8; READ_REPLY_LEN];
    if let Err(error) = self.read_filling(&mut reply).await {
      if error == TmcError::Timeout {
        // Post-timeout RX-FIFO snapshot: sample raw occupancy FIRST (`read_ready` does NOT clear the latched
        // glitch flag), then robustly drain whatever is there. This tells a genuinely silent driver (`fifo:0`)
        // from a reply that arrived but got gated by the glitch detector (`fifo:N` + the drained bytes).
        let ready = self.uart.read_ready();
        let (fifo, drained) = self.drain_snapshot();
        // Reuse the raw-reply slot for the drained bytes so the existing `[MSG:TMC-IOIN …]` dump surfaces them.
        self.last_reply = drained;
        self.last_probe_stage = TmcProbeStage::ReplyTimeout { fifo, ready };
      }
      return Err(self.note_error(error));
    }
    // A full reply arrived: capture it before decode so a decode-error node's raw bytes are available for the
    // `$I+` framing dump. Decode is the integrity arbiter, and the outcome is tagged with whether the reply had
    // to tolerate a glitch — so a driver proven alive via tolerance (`ok(<v>)`) and a glitch-corrupted decode
    // (`crc(<v>)`) are BOTH visible, never silently swallowed nor conflated with the clean cases.
    self.last_reply = reply;
    match decode_read_reply(&reply, reg) {
      Ok(value) => {
        if let Some(variant) = self.reply_glitched {
          self.last_probe_stage = TmcProbeStage::RespondedDespiteGlitch(variant);
        }
        Ok(value)
      }
      Err(error) => {
        self.last_probe_stage = match self.reply_glitched {
          Some(variant) => TmcProbeStage::DecodeErrorGlitched(variant),
          None => TmcProbeStage::DecodeError,
        };
        Err(self.note_error(error))
      }
    }
  }
}

/// Configure UART1 as the single-wire TMC2209 bus on GPIO9 and wrap it as a [`Uart1TmcBus`].
///
/// The same GPIO9 line carries both TX and RX (single-wire half-duplex): the pin is degraded to an `AnyPin`
/// and cloned twice — one handle routes the RX signal, one sets the pad drive mode. The clones are the
/// documented esp-hal boundary use of `unsafe` — there is exactly one physical pin, and the handles merely
/// route that one line to UART1's TX/RX through the GPIO matrix and configure the shared pad; nothing else
/// touches GPIO9.
///
/// ## The pad handle (the reason for the third handle)
/// `Uart::with_tx` unconditionally applies `OutputConfig::default()` (push-pull) to the pad, so the drive mode
/// MUST be (re-)applied AFTER the TX/RX routing is established, not before (a pre-config would be overwritten).
/// `apply_output_config` on an [`OutputSignal`] writes only the pad's drive-mode / pull registers (keyed by
/// GPIO number) — it does not disturb the signal routing or the output-enable `with_tx` set up — so driving the
/// pad through a spare handle after building the UART is the minimal correct seam. This fn applies the RELEASED
/// state (open-drain + pull-up, the idle/reply mode) and hands the handle to the bus, which then swaps the pad
/// push-pull around each transmit (the half-duplex push-pull scheme — see the module docs for why both static
/// drive modes failed on the bench). Applied LAST so the pull-up is not clobbered by `with_rx`'s input-config
/// pass (pull is shared between the input and output config).
///
/// ## Pull-up strength (bench note — flagged, not decided here)
/// The released state enables the ESP32-S3 INTERNAL pull-up (~45 kΩ). The transmit no longer depends on it
/// (push-pull drives both edges), but the DRIVER's open-drain reply still rises through the bus pull-up:
/// against breadboard/wiring capacitance ~45 kΩ is marginal at 115200 (bit ≈ 8.7 µs; τ ≈ 45 kΩ · C_bus), so
/// keep the EXTERNAL ~4.7 kΩ pull-up to 3.3 V fitted on the shared line for the reply window.
///
/// # Panics
/// `expect` is used here because this runs once in `main`'s init path, where a failure to bring up UART1 is
/// an unrecoverable wiring/config fault, not a runtime condition (CLAUDE.md permits `expect` in init).
pub fn init(uart1: esp_hal::peripherals::UART1<'static>, tmc_uart_pin: esp_hal::peripherals::GPIO9<'static>) -> Uart1TmcBus {
  // 115200 baud, 8N1 (the Config defaults), matching the TMC2209 UART and the user's proven-working RAMPS
  // config (DOC-03). The TMC2209 auto-bauds off each datagram's sync nibble, so the driver needs no baud config.
  let config = Config::default().with_baudrate(115_200);
  // One physical line, three signal handles: degrade to AnyPin, clone for the RX direction and once more for
  // the post-routing pad drive-mode control (see fn docs). Only GPIO9 is ever referenced.
  let tx_line = tmc_uart_pin.degrade();
  let rx_line = unsafe { tx_line.clone_unchecked() };
  let pad_line = unsafe { tx_line.clone_unchecked() };
  let uart = Uart::new(uart1, config)
    .expect("UART1 TMC bus init")
    .with_tx(tx_line)
    .with_rx(rx_line);
  // `with_tx` forced the pad push-pull; start in the RELEASED state (open-drain + pull-up) so the shared line
  // idles high and un-driven from the first instant, then hand the pad handle to the bus for the per-transmit
  // push-pull swaps.
  let pad: OutputSignal<'static> = pad_line.into();
  pad.apply_output_config(&pad_config(DriveMode::OpenDrain));
  Uart1TmcBus::new(uart, pad)
}

/// The `tmc_manager` task (DOC-03 / DOC-01): runs once on the core-0 thread executor to configure every
/// driver, then polls `DRV_STATUS` for faults on the present drivers forever.
///
/// On startup it runs the full per-axis init sequence (presence check via `IOIN.VERSION`, register
/// programming, `IFCNT` write verification). A driver that does not answer is flagged absent and skipped —
/// motion can still proceed on a VREF-configured driver (DOC-03) — so one missing driver never aborts the
/// others. After init it sleeps [`STATUS_POLL_INTERVAL`] between rounds; each round re-reads `DRV_STATUS` on
/// every present driver to detect over-temperature / short / open-load conditions, and RE-PROBES every absent
/// node (a full `init_axis`) so a driver that appears mid-session — power applied, wiring fixed — is
/// configured and joins the `[DRIVER:]`/`[MSG:TMC-PROBE]` reports live, without a reboot.
///
/// ## Still stubbed (out of scope for this phase, deliberate TODOs)
/// - Diagnostics are surfaced via `defmt` only (no-op in the default build) so they never corrupt the grbl
///   USB stream; a present-but-misconfigured or faulting driver does not yet raise a machine ALARM. The
///   alarm-state machine (shared with the motion/limit fault path, DOC-06) is a later phase — when it lands,
///   an init/verify failure and a `DRV_STATUS` hard fault should force the spindle off and halt motion.
/// - The `TmcConfig` is now sourced from the persisted settings loaded at boot (DOC-04), but a runtime
///   `$x=val` / `$PBX` change to a current/microstep setting does not yet re-program the live drivers — it
///   takes effect on the next boot. Live re-application (a `Watch<TmcConfig>` this task selects on) is a
///   later refinement.
#[embassy_executor::task]
pub async fn tmc_manager(bus: Uart1TmcBus, config: TmcConfig) -> ! {
  // The `tmc-tx-diag` / `tmc-scope-diag` builds replace normal operation with their bench diagnostic loop
  // (never returns), so the whole normal path — boot loopback, init, and polling — is factored into
  // `run_tmc_manager` and compiled out under those features. Exactly one cfg branch survives as this task's
  // tail, so the `-> !` is satisfied; if BOTH diag features are enabled, the continuous-TX blast wins (a
  // scope-diag capture of a TX blast would be meaningless, but the additive-features build must still compile).
  #[cfg(feature = "tmc-tx-diag")]
  {
    let _ = config;
    let mut bus = bus;
    bus.tx_diag_forever().await
  }
  #[cfg(all(feature = "tmc-scope-diag", not(feature = "tmc-tx-diag")))]
  {
    let _ = config;
    let mut bus = bus;
    bus.scope_diag_forever().await
  }
  #[cfg(not(any(feature = "tmc-tx-diag", feature = "tmc-scope-diag")))]
  run_tmc_manager(bus, config).await
}

/// The normal `tmc_manager` operation: boot loopback self-test, the per-axis init/presence pass, then the
/// `DRV_STATUS` poll loop. Factored out so the `tmc-tx-diag` / `tmc-scope-diag` builds can replace it wholesale.
#[cfg(not(any(feature = "tmc-tx-diag", feature = "tmc-scope-diag")))]
async fn run_tmc_manager(mut bus: Uart1TmcBus, config: TmcConfig) -> ! {
  let manager = TmcManager::new(config);

  // Boot loopback self-test, BEFORE any driver access: read back the MCU's own half-duplex echo on GPIO9 to
  // prove the TX+RX path and line levels without a scope, published for the `$I+` `[MSG:TMC-LOOPBACK …]` line.
  // Safe with drivers attached — the pattern is not a valid addressed request, so nothing replies to it.
  TMC_LOOPBACK.store(bus.loopback().await.to_bits(), Ordering::Release);

  // Configure every driver and record which ones actually answered, so the poll loop only queries present
  // drivers (querying an absent one would just time out every interval). The per-axis loop (rather than
  // `init_all`) lets us sample the transport's outcome right after each node's presence read and fold it with
  // the manager's report via `classify_ioin`, capturing the full `IOIN` outcome (echo/reply timeout, decode
  // error, version mismatch, or ok) for the `$I+` diagnostic. For an absent/failing node the presence read is
  // `init_axis`'s only read, so the sampled outcome reflects it; for a present node it is `Responded`.
  let mut present = [false; AXIS_COUNT];
  let mut online_mask: u8 = 0;
  let mut stages_packed: u64 = 0;
  let mut raw_axis: u8 = NO_RAW_CAPTURE;
  let mut raw_bytes: u64 = 0;
  let mut init_fail_bits: u32 = 0;
  for axis in 0..AXIS_COUNT {
    let report = manager.init_axis(&mut bus, axis).await;
    // Sample the transport outcome before any further bus use so it reflects this axis's presence read, then
    // classify it against the report (which alone carries the wrong-version byte).
    let stage = classify_ioin(&report, bus.last_probe_stage());
    stages_packed |= u64::from(stage.to_bits()) << (axis * 16);
    // Capture the raw bytes of the FIRST node that has bytes worth eyeballing on `$I+`: a decode failure (the
    // reply that would not parse — clean or glitched) OR a reply timeout that nonetheless drained bytes from the
    // FIFO (`fifo:N>0`, the "driver replied but was gated" case). `bus.last_reply_bytes()` holds the reply for a
    // decode error and the drained bytes for a timeout.
    let wants_dump = matches!(stage, TmcProbeStage::DecodeError | TmcProbeStage::DecodeErrorGlitched(_))
      || matches!(stage, TmcProbeStage::ReplyTimeout { fifo, .. } if fifo > 0);
    if wants_dump && raw_axis == NO_RAW_CAPTURE {
      raw_axis = axis as u8;
      raw_bytes = u64::from_le_bytes(bus.last_reply_bytes());
    }
    // Capture the failing bus op of the FIRST bare-`InitError` node so `$I+` can name the register + op + kind.
    // The transport fields still reflect the last (failing) op of this axis's `init_axis`. `init_fail_bits == 0`
    // is the "not yet captured" sentinel — `InitFailure::to_bits` always sets the validity bit, so it is never 0.
    if matches!(stage, TmcProbeStage::InitError) && init_fail_bits == 0 {
      let kind = bus.last_error_kind();
      // Attach the RX-error detail only for an `Io` that carried an actual `RxError` (a read failure); a
      // transmit-side `Io` has no captured variant, so it stays a bare `err:...:io` (its absence is a signal).
      let io_detail = if kind == Some(BusErrorKind::Io) {
        bus.last_rx_error().map(|variant| (bus.last_io_stage(), variant))
      } else {
        None
      };
      let failure = InitFailure { axis, reg: bus.last_op_reg(), was_write: bus.last_op_was_write(), kind, io_detail };
      init_fail_bits = failure.to_bits();
    }
    let is_present = report.as_ref().map(|report| report.present).unwrap_or(false);
    present[axis] = is_present;
    if is_present {
      online_mask |= 1 << axis;
    }
    log_init_result(axis, &report);
  }
  // Publish the diagnostic snapshots for the `$I+` report. `Release` the raw capture, init-failure detail,
  // packed outcomes, and online mask before flagging init done (also `Release`) so a comms-task reader that
  // observes `TMC_INIT_DONE == true` via an `Acquire` load always sees every populated snapshot.
  TMC_IOIN_RAW.store(raw_bytes, Ordering::Release);
  TMC_IOIN_RAW_AXIS.store(raw_axis, Ordering::Release);
  TMC_INIT_FAIL.store(init_fail_bits, Ordering::Release);
  TMC_IOIN_STAGES.store(stages_packed, Ordering::Release);
  TMC_ONLINE_MASK.store(online_mask, Ordering::Release);
  TMC_INIT_DONE.store(true, Ordering::Release);

  // The live `[MSG:TMC-BUS …]` margin-meter counters (see `BusStats`): per-axis attempted/failed exchanges,
  // accumulated locally and republished each round. Saturating, so a long bench session pins at `u16::MAX`
  // instead of wrapping around to a misleadingly small count.
  let mut bus_fail = [0u16; AXIS_COUNT];
  let mut bus_total = [0u16; AXIS_COUNT];

  loop {
    // Sleep first so the bus is quiet immediately after the init burst, then poll. Awaiting here yields the
    // executor to the other core-0 tasks between rounds.
    Timer::after(STATUS_POLL_INTERVAL).await;
    for axis in 0..AXIS_COUNT {
      let bit = 1u8 << axis;
      bus_total[axis] = bus_total[axis].saturating_add(1);
      if !present[axis] {
        // Live re-probe of an absent node: re-run the full init sequence each round so a driver that appears
        // mid-session (12 V switched on, a breadboard wire reseated, a module swapped) is configured and joins
        // the report WITHOUT a reboot — the boot pass alone made `[DRIVER:]`/`[MSG:TMC-PROBE]` a frozen
        // snapshot, which turned every bench wiring/power tweak into a reflash-and-reboot cycle. `init_axis` is
        // the right call (not a bare presence read): a driver that just gained power has default registers and
        // needs the whole programming + `IFCNT` verification anyway. Cost when absent: one 5 ms reply timeout
        // per axis per [`STATUS_POLL_INTERVAL`] round — negligible bus load, entirely off the real-time path.
        let report = manager.init_axis(&mut bus, axis).await;
        // Refresh this axis's packed probe outcome so `$I+`'s `[MSG:TMC-PROBE]` tracks the live re-probe (the
        // raw-dump / init-failure captures stay boot-only; the token is the live signal that matters).
        let stage = classify_ioin(&report, bus.last_probe_stage());
        stages_packed = (stages_packed & !(0xFFFFu64 << (axis * 16))) | u64::from(stage.to_bits()) << (axis * 16);
        TMC_IOIN_STAGES.store(stages_packed, Ordering::Release);
        if report.as_ref().map(|report| report.present).unwrap_or(false) {
          present[axis] = true;
          online_mask |= bit;
          log_init_result(axis, &report);
        } else {
          // The re-probe went unanswered (or failed init): a failed exchange on the margin meter.
          bus_fail[axis] = bus_fail[axis].saturating_add(1);
        }
        continue;
      }
      match manager.read_status(&mut bus, axis).await {
        // A hard `DRV_STATUS` fault (over-temp / short / open-load): the driver still answers but is unhealthy,
        // so drop it from the live health mask and log it. It re-joins the mask on the next clean read.
        Ok(status) if status.has_fault() => {
          online_mask &= !bit;
          log_fault(axis, status);
        }
        // A clean read: the driver is present and healthy, so (re-)mark it online.
        Ok(_) => online_mask |= bit,
        // No reply this round (the driver dropped off the half-duplex bus): drop it from the live mask so the
        // report stops claiming a silent driver is `ok`, and count the failed exchange on the margin meter. A
        // transient error self-heals on the next clean read. (A `DRV_STATUS` hard fault above is NOT a bus
        // failure — the driver replied fine — so only this arm feeds `bus_fail` on the present path.)
        Err(_) => {
          online_mask &= !bit;
          bus_fail[axis] = bus_fail[axis].saturating_add(1);
        }
      }
    }
    // Republish the refreshed health so `[DRIVER:]` reflects mid-job faults, not just the boot snapshot, and
    // the margin-meter counters so `$I+` grades the round that just ran.
    TMC_ONLINE_MASK.store(online_mask, Ordering::Release);
    TMC_BUS_FAILS.store(pack_axis_counts(bus_fail), Ordering::Release);
    TMC_BUS_TOTALS.store(pack_axis_counts(bus_total), Ordering::Release);
  }
}

/// Pack the per-axis `u16` margin-meter counters into the 16-bits-per-axis `u64` layout shared with
/// [`TMC_IOIN_STAGES`] (and covered by the same `AXIS_COUNT <= 4` guard).
#[cfg(not(any(feature = "tmc-tx-diag", feature = "tmc-scope-diag")))]
fn pack_axis_counts(counts: [u16; AXIS_COUNT]) -> u64 {
  let mut packed = 0u64;
  for (axis, &count) in counts.iter().enumerate() {
    packed |= u64::from(count) << (axis * 16);
  }
  packed
}

/// Surface one axis's init outcome over `defmt` (a no-op in the default, defmt-free build). Present drivers
/// log their version and resolved current scale; absent ones log that they were skipped; errors log loudly.
#[allow(unused_variables)]
fn log_init_result(axis: usize, report: &Result<AxisReport, firmware_core::drivers::tmc2209::manager::TmcManagerError>) {
  match report {
    #[cfg(feature = "defmt")]
    Ok(report) if report.present => defmt::info!(
      "TMC axis {} node {}: configured, version {=u8:#x}, CS {=u8} vsense {}",
      axis,
      report.node,
      report.version,
      report.current.cs,
      report.current.vsense,
    ),
    #[cfg(feature = "defmt")]
    Ok(report) => defmt::warn!("TMC axis {} node {}: absent (no reply / version {=u8:#x}), skipped", axis, report.node, report.version),
    #[cfg(feature = "defmt")]
    Err(error) => defmt::error!("TMC axis {} init failed: {:?}", axis, error),
    // Without defmt there is nowhere safe to log (the USB endpoint carries the grbl protocol), so the result
    // is computed for its configuration side effects and otherwise dropped until the alarm path exists.
    #[cfg(not(feature = "defmt"))]
    _ => {}
  }
}

/// Surface a `DRV_STATUS` hard fault over `defmt` (a no-op in the default build).
#[allow(unused_variables)]
fn log_fault(axis: usize, status: firmware_core::drivers::tmc2209::registers::DrvStatus) {
  #[cfg(feature = "defmt")]
  defmt::warn!(
    "TMC axis {} fault: overtemp {} short_gnd {} short_vs {} open_load {}",
    axis,
    status.overtemp_shutdown(),
    status.short_to_ground(),
    status.short_to_supply(),
    status.open_load(),
  );
}
