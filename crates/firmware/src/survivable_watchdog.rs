//! Survivable watchdog feed + capture-at-withhold (DIAGNOSTIC-only, `capture-reset`-gated). The layer-2 capture for
//! the Signature-B hard-silent-wedge investigation (`docs/streaming-lockup-investigation.md` §17.10/§17.11).
//!
//! ## Why a TIMG1 ISR instead of the async feeder
//! The production RWDT feeder is the core-0 thread-mode async task [`crate::comms::watchdog_feed`]. In Signature B a
//! comms task wedges in a non-yielding `await` and STARVES that cooperative executor — so `watchdog_feed` stops
//! running, the RWDT feed stops, and the heartbeat freezes, yet on the bench the hardware RWDT mysteriously did NOT
//! fire (the board stayed silent for >60 s). We need a watchdog feed that SURVIVES a core-0 executor stall and, on a
//! stall, WITHHOLDS the feed so a reset (RWDT or SuperWDT) fires and leaves a breadcrumb. A HARDWARE-timer ISR fires
//! even when the core-0 thread-mode executor is stalled (its idle hook is `waiti 0`, the thread run-level masks
//! nothing, and there is no `interrupt_free` around the executor loop). TIMG1 is independent of the esp-rtos /
//! embassy time driver (esp-rtos "now" = SystemTimer; TIMG0 is the esp-rtos time source — we do NOT touch TIMG0). A
//! TIMG1 interrupt configured in `main` (which runs on ProCpu) is core-0-fielded — which is what we want, since the
//! atomics it reads are written from core 0.
//!
//! ## Step timing is sacred
//! The ISR touches ONLY the `LP_WDT` (RTC_CNTL) watchdog registers and reads a few `AtomicU32`s — NEVER the `Rtc`
//! struct (no `&mut Rtc`), NEVER a mutex, NEVER any RMT / core-1 state. It adds ZERO lock contention or latency to
//! the core-1 motion path.
//!
//! ## The GPIO18 scope heartbeat
//! [`start`] also configures GPIO18 as a push-pull output and the ISR toggles it every fire — a FREE-RUNNING ~2 Hz
//! square wave (250 ms half-period), driven BEFORE the feed/withhold branch so it advances unconditionally. A scope
//! (Rigol DHO804) watches GPIO18 with a ~600 ms Timeout trigger: the pin only flatlines when the chip is so hard-locked
//! that even this survivable ISR stops running (or at the eventual WDT reset), so a flatline is the Signature-B "is the
//! chip fundamentally alive" signal — independent of any toolpath, unlike watching a STEP line. GPIO18 is the DOC-00
//! spare RMT ch3 (4th axis); in THIS `capture-reset` build `main` binds that never-driven channel to `NoPin` and hands
//! GPIO18 here instead, so there is no contention. The pin toggle touches ONLY the GPIO output set/clear registers —
//! no RMT / core-1 state, zero step-timing impact.
//!
//! ## The `unsafe` (raw PAC boundary)
//! The dual-dog feed pokes the RWDT / SuperWDT registers via the raw `esp32s3` PAC (the same `LP_WDT` peripheral
//! `esp_hal::rtc_cntl::Rwdt`/`Swd` use), and the heartbeat toggle pokes the GPIO output set/clear registers (the same
//! registers `esp_hal::gpio::Output::set_high`/`set_low` use). This is the esp-hal/PAC boundary the crate-level
//! `#![deny(unsafe_code)]` exception in `firmware` exists for; it is confined to [`feed_rwdt_raw`] / [`feed_swd_raw`] /
//! [`toggle_heartbeat`] and commented inline. The ISR takes no `&mut` to any esp-hal driver — it only writes registers.

use core::sync::atomic::Ordering;

// On the ESP32-S3 the RWDT / SuperWDT registers live in the `RTC_CNTL` block, exposed through the `LPWR` peripheral
// (esp-hal aliases `LPWR as LP_WDT` on this chip — there is NO separate `LP_WDT` singleton on the S3). `LPWR::regs()`
// returns that register block; esp-hal's own `Rwdt::feed`/`Swd` poke the SAME registers through it. `regs()` is an
// associated fn on the peripheral TYPE, so it works even though the `LPWR` instance was consumed by `Rtc::new`.
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::peripherals::LPWR as LP_WDT;
use esp_hal::time::Duration;
use esp_hal::timer::PeriodicTimer;
use esp_hal::timer::timg::TimerGroup;
use portable_atomic::AtomicU32;
use static_cell::StaticCell;

use crate::comms::{
  COMMS_PROGRESS, EXECUTOR_ALIVE, EXECUTOR_RUNNING, MOTION_LIVENESS, RESPONSE, RX_ACTIVITY, USB_TX_COMPLETED,
  USB_TX_STALL_FINGERPRINT, USB_TX_STALL_FINGERPRINT_LEN, USB_TX_STALL_WINDOW_COUNT,
};

/// The TIMG1 feed cadence: every 250 ms the ISR decides feed-vs-withhold and (when feeding) pets BOTH dogs. Short
/// relative to the 8 s RWDT window (32 feeds per window) and to the S3 SuperWDT's fixed seconds-scale silicon period,
/// so a single late fire never trips a dog on a healthy board; short enough that a withhold lets the dog run out
/// within a few seconds of a genuine wedge.
const FEED_INTERVAL: Duration = Duration::from_millis(250);

/// The core-1 motion-stall threshold, in [`FEED_INTERVAL`] ticks: the beat may stay frozen WHILE a block is in flight
/// for this many ticks before the ISR declares a core-1 wedge. `16 * 250 ms = 4 s`, matching the async feeder's
/// `8 * 500 ms`. Passed to the pure [`firmware_core::diag::watchdog_decision`].
const CORE1_STALL_TICKS: u32 = 16;

/// The core-0 comms-stall threshold, in [`FEED_INTERVAL`] ticks: [`COMMS_PROGRESS`] may stay frozen WHILE the host is
/// active for this many ticks before the ISR declares a core-0 comms wedge. `12 * 250 ms = 3 s`, matching the async
/// feeder's `6 * 500 ms`.
const COMMS_STALL_TICKS: u32 = 12;

/// The host-active sticky window, in [`FEED_INTERVAL`] ticks since [`RX_ACTIVITY`] last advanced. The host counts as
/// active for `24 * 250 ms = 6 s` after its last received byte — matching the async feeder's `12 * 500 ms` reset-loop
/// guard / flow-control quiet-gap bridge: a board with NO host (RX never advances) goes inactive and is fed forever.
const RX_ACTIVE_TICKS: u32 = 24;

/// The core-0 executor-stall threshold, in [`FEED_INTERVAL`] ticks: the UNGATED [`EXECUTOR_ALIVE`] beat may stay frozen
/// for this many ticks before the ISR declares a full core-0 async-executor stall (the §17.15 root-cause fix). `16 *
/// 250 ms = 4 s` — above any legitimate core-0 quiesce during streaming, well below the point of no return. Mirrors
/// [`firmware_core::diag::EXECUTOR_STALL_TICKS`]; passed to the pure decision as `executor_stall_ticks`.
const EXECUTOR_STALL_TICKS: u32 = 16;

/// The TIMG1 `PeriodicTimer` instance, parked `'static` so it (and its bound ISR) outlive `main`. The ISR clears the
/// timer's interrupt flag through this handle each fire — the only mutable access, and the ISR is the sole accessor
/// after [`start`] returns, so a `'static` `&mut` is sound (see [`fire`]).
static TIMER: StaticCell<PeriodicTimer<'static, esp_hal::Blocking>> = StaticCell::new();

/// A raw pointer to the parked [`PeriodicTimer`], set once by [`start`] so the bare `extern "C"` ISR (which takes no
/// arguments) can reach the timer to clear its interrupt flag. Written exactly once before the interrupt is enabled;
/// read only inside the ISR.
static TIMER_PTR: AtomicU32 = AtomicU32::new(0);

/// The GPIO number of the scope heartbeat pin (GPIO18 — the DOC-00 spare RMT ch3, freed from the 4th axis in this
/// build). Fixed because [`start`] takes the concrete `GPIO18` singleton; kept as a named const so the ISR's raw
/// register mask reads clearly. `< 32`, so it lives in the primary GPIO output bank (`out_w1ts` / `out_w1tc`).
const HEARTBEAT_PIN: u32 = 18;

/// The GPIO number of the WITHHOLD-decision probe pin (GPIO17 — a free control-input header pin, `< 32`, OUT bank).
/// Driven HIGH on any ISR fire whose [`firmware_core::diag::watchdog_decision`] returns a `withhold_reason`, LOW
/// otherwise. This is the decisive (a)-vs-(b) instrument for docs §17.13: on a Signature-B wedge that does NOT
/// self-reset, if this line stays LOW the withhold decision NEVER fired (hypothesis a — a gated-out detector); if it
/// goes HIGH yet the board still does not reset, the withhold fired but the RWDT/SuperWDT did not (hypothesis b). A
/// LIVE mirror of the decision (not a latch) so the scope shows every assert/de-assert edge. `< 32` ⇒ primary
/// output bank (`out_w1ts` / `out_w1tc`), same raw-register discipline as [`HEARTBEAT_PIN`].
const WITHHOLD_PROBE_PIN: u32 = 17;

/// The heartbeat push-pull `Output`, parked `'static` so GPIO18 stays configured as an output for the program's life
/// (dropping it would release the pin). The ISR NEVER touches this handle — it only pokes the raw set/clear registers
/// (see [`toggle_heartbeat`]); the parked handle exists solely to hold the pin configuration, mirroring how [`TIMER`]
/// parks the timer. Written once by [`start`] before the alarm interrupt is enabled.
static HEARTBEAT_OUTPUT: StaticCell<Output<'static>> = StaticCell::new();

/// The withhold-probe push-pull `Output`, parked `'static` so GPIO17 stays configured as an output for the program's
/// life. As with [`HEARTBEAT_OUTPUT`], the ISR NEVER touches this handle — it only pokes the raw set/clear registers
/// (see [`drive_withhold_probe`]); the parked handle exists solely to hold the pin configuration. Written once by
/// [`start`] before the alarm interrupt is enabled.
static WITHHOLD_OUTPUT: StaticCell<Output<'static>> = StaticCell::new();

/// The software-tracked heartbeat level (`0` = low, `1` = high), flipped by the ISR each fire. The ISR is the SOLE
/// writer, so a plain `Relaxed` read-modify-write is race-free; it is the source of truth for which set/clear register
/// the next toggle writes (the GPIO output register is not read back). Starts `0` to match the `Level::Low` init.
static HEARTBEAT_LEVEL: AtomicU32 = AtomicU32::new(0);

// Per-interval bookkeeping statics. The ISR is the SOLE writer of each, so a plain `Relaxed` read-modify-write is
// correct (no other context races them). They mirror the local state the async `watchdog_feed` keeps, moved into
// `static`s because the ISR has no persistent stack frame.

/// Last-sampled core-1 [`MOTION_LIVENESS`] beat (for the frozen-delta test). Sentinel `u32::MAX` = "unseeded": the
/// first fire seeds it so the first delta is meaningful rather than a spurious "moved from 0".
static LAST_CORE1: AtomicU32 = AtomicU32::new(u32::MAX);
/// Last-sampled [`COMMS_PROGRESS`] beat.
static LAST_COMMS: AtomicU32 = AtomicU32::new(u32::MAX);
/// Last-sampled [`RX_ACTIVITY`] count (for host-active ageing).
static LAST_RX: AtomicU32 = AtomicU32::new(u32::MAX);
/// Last-sampled [`USB_TX_COMPLETED`] count (for the dead-zone freeze).
static LAST_TX_COMPLETED: AtomicU32 = AtomicU32::new(u32::MAX);
/// Last-sampled [`EXECUTOR_ALIVE`] beat (for the ungated core-0 executor-stall freeze).
static LAST_EXECUTOR_ALIVE: AtomicU32 = AtomicU32::new(u32::MAX);

/// Consecutive ticks the core-1 beat has been frozen while a block is in flight.
static CORE1_FROZEN: AtomicU32 = AtomicU32::new(0);
/// Consecutive ticks [`COMMS_PROGRESS`] has been frozen while the host is active.
static COMMS_FROZEN: AtomicU32 = AtomicU32::new(0);
/// Consecutive ticks `usb_tx` has completed no write (the dead-zone input).
static TX_COMPLETE_FROZEN: AtomicU32 = AtomicU32::new(0);
/// Consecutive ticks the ungated [`EXECUTOR_ALIVE`] beat has stayed frozen (the core-0 executor-stall input). Only
/// accrues once the beat has advanced past its initial `0` — the boot guard against the pre-first-bump zero.
static EXECUTOR_ALIVE_FROZEN: AtomicU32 = AtomicU32::new(0);
/// Consecutive ticks since RX last advanced (host-active ageing). Seeded at [`RX_ACTIVE_TICKS`] so a board that boots
/// with no host present starts INACTIVE and is fed forever — the reset-loop guard.
static RX_IDLE: AtomicU32 = AtomicU32::new(RX_ACTIVE_TICKS);

/// Whether a withhold was already captured this wedge, so the capture-at-withhold breadcrumb is written exactly ONCE
/// per withhold transition (not every 250 ms while the dog runs out). Reset to `0` whenever the ISR feeds (the wedge
/// cleared), so a fresh wedge captures again. `1` = a withhold is in progress and the breadcrumb is already written.
static WITHHOLD_LATCHED: AtomicU32 = AtomicU32::new(0);

/// The [`crate::crash::WithholdReason`] value [`capture_withhold`] last recorded (as its `u8`, `0` = none captured
/// yet), so the healthy `None` branch can erase exactly that breadcrumb on recovery (Fix #3/H2). This ISR is the SOLE
/// withhold writer in the capture-reset build, so there is no cross-writer contention on the recovery CAS.
static LAST_CAPTURED_WITHHOLD: AtomicU32 = AtomicU32::new(0);

/// Free-running count of ISR fires since boot, used ONLY by the `force-withhold` positive-control build to trigger a
/// deterministic withhold after a fixed delay. Plain `Relaxed` RMW — the ISR is the sole writer.
#[cfg(feature = "force-withhold")]
static FIRE_COUNT: AtomicU32 = AtomicU32::new(0);

/// The `force-withhold` build forces an UNCONDITIONAL withhold once [`FIRE_COUNT`] reaches this many fires. `20 *
/// 250 ms = 5 s` after boot — long enough to confirm a steady heartbeat on the scope first, then the RWDT (8 s)
/// should reset the board ~13 s after boot. See docs §17.16 and the `force-withhold` feature comment.
#[cfg(feature = "force-withhold")]
const FORCE_WITHHOLD_AT_FIRES: u32 = 20;

/// Build the TIMG1 periodic alarm, bind the [`fire`] ISR, and start it — and configure `heartbeat_pin` (GPIO18) as the
/// scope heartbeat output the ISR toggles each fire. Called once from `main` on ProCpu (so the interrupt is
/// core-0-fielded). The `Rtc`'s RWDT + SuperWDT must already be enabled by the caller; from here the ISR owns FEEDING
/// them (the async [`crate::comms::watchdog_feed`] is replaced by a heartbeat-only task in this build).
pub fn start(
  timg1: esp_hal::peripherals::TIMG1<'static>,
  heartbeat_pin: esp_hal::peripherals::GPIO18<'static>,
  withhold_probe_pin: esp_hal::peripherals::GPIO17<'static>,
) {
  // Configure GPIO18 as a push-pull output (starting LOW, matching `HEARTBEAT_LEVEL`'s `0`) and park it `'static` so
  // the pin stays an output for the program's life. The ISR toggles it via the raw set/clear registers, never through
  // this handle — parking it just holds the pin configuration (the same reason `TIMER` is parked). Done BEFORE the
  // alarm interrupt is enabled below, so the pin is ready when the first `fire()` toggles it.
  let heartbeat = Output::new(heartbeat_pin, Level::Low, OutputConfig::default());
  let _: &'static mut Output<'static> = HEARTBEAT_OUTPUT.init(heartbeat);

  // Configure GPIO17 as a push-pull output (starting LOW = "no withhold") and park it `'static`, mirroring the
  // heartbeat pin. The ISR drives it from the live `watchdog_decision` result via the raw set/clear registers — the
  // §17.13 (a)-vs-(b) probe. Done before the alarm interrupt is enabled so the pin is ready on the first fire.
  let withhold_probe = Output::new(withhold_probe_pin, Level::Low, OutputConfig::default());
  let _: &'static mut Output<'static> = WITHHOLD_OUTPUT.init(withhold_probe);

  let timg1 = TimerGroup::new(timg1);
  let mut timer = PeriodicTimer::new(timg1.timer0);
  // `set_interrupt_handler` binds the handler AND enables the TG1_T0 CPU interrupt on the current core (ProCpu). The
  // timer is not running and not listening yet, so no fire can occur until `listen()` below — which we call only
  // AFTER `TIMER_PTR` is published, so the FIRST fire can always reach the timer to clear its flag.
  timer.set_interrupt_handler(fire);
  // `start` returns `Err` only on a bad period (zero / out of range); 250 ms is valid. `expect` is permitted in this
  // init path (CLAUDE.md): a failure here is an unrecoverable static wiring bug, not a runtime condition. This loads
  // the period + starts the counter but does NOT yet enable the alarm interrupt (that is `listen`).
  timer.start(FEED_INTERVAL).expect("start TIMG1 survivable-watchdog timer");
  let timer: &'static mut PeriodicTimer<'static, esp_hal::Blocking> = TIMER.init(timer);
  // Publish the parked-timer pointer BEFORE enabling the alarm interrupt, so the ISR can always clear the flag on its
  // very first fire (no tight re-entry window). `Release` pairs with the ISR's `Acquire` load.
  TIMER_PTR.store(timer as *mut _ as u32, Ordering::Release);
  // Now enable the timer's alarm interrupt — the dog starts being fed from the next 250 ms boundary.
  timer.listen();
}

/// The TIMG1 ISR (core-0-fielded). A thin shell over the pure [`firmware_core::diag::watchdog_decision`]: it samples
/// the core-0/core-1 progress atomics, maintains the frozen-tick counters, asks the pure decision whether to feed or
/// withhold both dogs, and either raw-PAC feeds both or (on a withhold) captures the breadcrumb once + lets the dogs
/// run out. It MUST clear the TIMG1 interrupt flag each fire or it re-enters immediately.
#[esp_hal::handler]
fn fire() {
  // Clear the timer interrupt FIRST so we never re-enter. Reach the parked timer through the raw pointer set by
  // `start` (the ISR has no captured state). If `start` has not run yet (pointer still 0) there is nothing to clear.
  let timer_ptr = TIMER_PTR.load(Ordering::Acquire) as *mut PeriodicTimer<'static, esp_hal::Blocking>;
  if !timer_ptr.is_null() {
    // SAFETY: `timer_ptr` came from a `&'static mut PeriodicTimer` parked in a `StaticCell` (lives for the program),
    // written once with `Release` before the interrupt was enabled. The ISR is the SOLE accessor of the timer after
    // `start` returns (the timer is not touched anywhere else), so no other `&mut` aliases it — exclusive access is
    // upheld. `clear_interrupt` only writes the TIMG1 int-clear register.
    unsafe {
      (*timer_ptr).clear_interrupt();
    }
  }

  // Free-running heartbeat: the boot dump's `wdog=` value reveals whether the ISR (the live feeder in this build) ran
  // through a wedge. The ISR is the sole heartbeat bumper in the capture build (the async `watchdog_feed` is absent).
  crate::crash::bump_watchdog_heartbeat();

  // Free-running SCOPE heartbeat: toggle GPIO18 UNCONDITIONALLY (before the feed/withhold branch below), so the ~2 Hz
  // square wave stops only when even this survivable ISR cannot run — the Signature-B hard-wedge signal on the scope.
  toggle_heartbeat();

  // Sample the progress beats. Seed the last-sample sentinels on the first fire so the first delta is "no change"
  // (frozen), not a spurious move — the frozen counters need a real baseline before they can accrue.
  let core1 = MOTION_LIVENESS.load(Ordering::Relaxed);
  let comms = COMMS_PROGRESS.load(Ordering::Relaxed);
  let rx = RX_ACTIVITY.load(Ordering::Relaxed);
  let tx_completed = USB_TX_COMPLETED.load(Ordering::Relaxed);
  let executor_alive = EXECUTOR_ALIVE.load(Ordering::Relaxed);
  let last_core1 = seed_swap(&LAST_CORE1, core1);
  let last_comms = seed_swap(&LAST_COMMS, comms);
  let last_rx = seed_swap(&LAST_RX, rx);
  let last_tx_completed = seed_swap(&LAST_TX_COMPLETED, tx_completed);
  let last_executor_alive = seed_swap(&LAST_EXECUTOR_ALIVE, executor_alive);

  // Core-1 motion freeze: beat unchanged WHILE a block is in flight. An idle/parked executor (`EXECUTOR_RUNNING`
  // false) resets the count — a legitimately still beat is never a wedge.
  let block_in_flight = EXECUTOR_RUNNING.load(Ordering::Acquire);
  let core1_frozen = bump_or_reset(&CORE1_FROZEN, block_in_flight && core1 == last_core1);

  // Host-active ageing: ticks since RX last advanced, reset on any advance.
  let rx_idle = bump_or_reset(&RX_IDLE, rx == last_rx);
  let host_active = rx_idle < RX_ACTIVE_TICKS;

  // Core-0 comms freeze: `COMMS_PROGRESS` unchanged WHILE the host is active. With no host (RX aged out) the count is
  // held at 0 — a still counter with no host to serve is correct, not a wedge (the reset-loop guard).
  let comms_frozen = bump_or_reset(&COMMS_FROZEN, host_active && comms == last_comms);

  // Dead-zone freeze: `usb_tx` completed no write this tick. Ungated by host/executor — exactly the dead zone the
  // other two detectors are blind to.
  let tx_complete_frozen = bump_or_reset(&TX_COMPLETE_FROZEN, tx_completed == last_tx_completed);

  // Core-0 executor-stall freeze (§17.15 fix): the UNGATED `EXECUTOR_ALIVE` beat unchanged. Gated ONLY by the boot
  // guard `executor_alive != 0` — once the core-0 heartbeat task has run at least once, ANY subsequent freeze is a
  // stall, regardless of host / motion / response state (the blind spot the other three detectors share). The `!= 0`
  // guard stops the pre-first-bump zero (before the task is scheduled) from accruing a false stall at boot.
  let executor_frozen = bump_or_reset(&EXECUTOR_ALIVE_FROZEN, executor_alive != 0 && executor_alive == last_executor_alive);

  let decision = firmware_core::diag::watchdog_decision(firmware_core::diag::WatchdogInputs {
    core1_frozen_ticks: core1_frozen,
    comms_frozen_ticks: comms_frozen,
    tx_complete_frozen_ticks: tx_complete_frozen,
    executor_alive_frozen_ticks: executor_frozen,
    executor_stall_ticks: EXECUTOR_STALL_TICKS,
    response_depth: RESPONSE.len(),
    block_in_flight,
    host_active,
    core1_stall_ticks: CORE1_STALL_TICKS,
    comms_stall_ticks: COMMS_STALL_TICKS,
  });

  // The EFFECTIVE withhold reason: normally the pure decision, but the `force-withhold` positive-control build (§17.16)
  // OVERRIDES it to an unconditional `DeadZone` withhold after a fixed boot delay, so a healthy idle board is forced
  // down the withhold path — the retention-safe test of hypothesis (b) that needs no reproduced wedge and no probe.
  let withhold_reason = effective_withhold_reason(decision.withhold_reason);

  // §17.13 (a)-vs-(b) probe: mirror the EFFECTIVE withhold decision onto GPIO17 (scope CH4) BEFORE acting on it, so the
  // scope shows exactly when the ISR decides to withhold. HIGH ⇒ a wedge this fire; LOW ⇒ it chose to feed. On a
  // non-self-resetting wedge: line stays LOW ⇒ no withhold ever fired (hypothesis a); line goes HIGH yet no reset ⇒
  // withhold fired but the dogs did not reset (hypothesis b).
  drive_withhold_probe(withhold_reason.is_some());

  match withhold_reason {
    None => {
      // Healthy / idle / host-absent: feed BOTH dogs and clear the withhold latch so a future wedge captures afresh.
      if decision.feed_rwdt {
        feed_rwdt_raw();
      }
      if decision.feed_swd {
        feed_swd_raw();
      }
      WITHHOLD_LATCHED.store(0, Ordering::Relaxed);
      // Fix #3/H2 recovery-clear: after the raw feeds cancel any pending reset, erase a breadcrumb captured on a PRIOR
      // fire whose wedge has since cleared, so a later unrelated reset does not spuriously boot LOCKED. Ordered AFTER
      // the feeds (a knife-edge dog expiry still boots locked); this ISR is the sole withhold writer, so the CAS never
      // contends. LOAD first and only `swap` when a reason was actually captured — the common never-wedged path is then
      // a plain read, not an atomic RMW, on every healthy fire (this ISR runs for the life of the board). The `swap(0)`
      // reads-and-resets so the clear still runs at most once per recovery.
      if LAST_CAPTURED_WITHHOLD.load(Ordering::Relaxed) != 0 {
        if let Some(reason) = crate::crash::WithholdReason::from_u8(LAST_CAPTURED_WITHHOLD.swap(0, Ordering::Relaxed) as u8) {
          crate::crash::clear_withhold_if(reason);
        }
      }
    }
    Some(reason) => {
      // A wedge: do NOT feed (both dogs run out → reset). Capture the breadcrumb EXACTLY ONCE per withhold
      // transition so the boot dump names the wedge class + the last-known usb_tx fingerprint, then keep withholding.
      if WITHHOLD_LATCHED.swap(1, Ordering::Relaxed) == 0 {
        capture_withhold(reason);
      }
    }
  }
}

/// The effective withhold reason acted on this fire. In every build EXCEPT `force-withhold` this is exactly the pure
/// `watchdog_decision` result. In the `force-withhold` positive-control build (§17.16) it OVERRIDES a `None` (feeding)
/// decision with `Some(DeadZone)` once [`FIRE_COUNT`] reaches [`FORCE_WITHHOLD_AT_FIRES`], forcing an unconditional
/// withhold so the withhold→dog→reset→breadcrumb chain is exercised on demand.
#[cfg(not(feature = "force-withhold"))]
fn effective_withhold_reason(decision: Option<firmware_core::diag::WithholdKind>) -> Option<firmware_core::diag::WithholdKind> {
  decision
}

/// `force-withhold` variant: after [`FORCE_WITHHOLD_AT_FIRES`] fires, force a `DeadZone` withhold regardless of the
/// pure decision (a real wedge reason still takes precedence if one somehow fires first).
#[cfg(feature = "force-withhold")]
fn effective_withhold_reason(decision: Option<firmware_core::diag::WithholdKind>) -> Option<firmware_core::diag::WithholdKind> {
  let fires = FIRE_COUNT.load(Ordering::Relaxed).saturating_add(1);
  FIRE_COUNT.store(fires, Ordering::Relaxed);
  decision.or(if fires >= FORCE_WITHHOLD_AT_FIRES {
    Some(firmware_core::diag::WithholdKind::DeadZone)
  } else {
    None
  })
}

/// Read a last-sample sentinel: if it is still the unseeded `u32::MAX`, store `current` and return it (so the first
/// delta is "no change"); otherwise store `current` and return the PRIOR value (the real last sample).
fn seed_swap(slot: &AtomicU32, current: u32) -> u32 {
  let prev = slot.swap(current, Ordering::Relaxed);
  if prev == u32::MAX { current } else { prev }
}

/// Bump a frozen-tick counter when `frozen`, else reset it to 0. Returns the updated value. Saturating so a long
/// genuine freeze can never wrap below the threshold.
fn bump_or_reset(slot: &AtomicU32, frozen: bool) -> u32 {
  let next = if frozen { slot.load(Ordering::Relaxed).saturating_add(1) } else { 0 };
  slot.store(next, Ordering::Relaxed);
  next
}

/// Write the capture-at-withhold breadcrumb once: the wedge class, plus the LATEST usb_tx fingerprint `usb_tx`
/// published (so even a hard Signature-B lock that never reached the K-escape carries its last-known state) and the
/// windowed stall count. All three usb_tx words are republished by `usb_tx` on every write timeout, so they hold the
/// freshest stall snapshot available — the ISR just copies them into the breadcrumb words.
fn capture_withhold(reason: firmware_core::diag::WithholdKind) {
  use firmware_core::diag::WithholdKind;
  let withhold = match reason {
    WithholdKind::Core1Motion => crate::crash::WithholdReason::Core1Motion,
    WithholdKind::Core0ExecutorStall => crate::crash::WithholdReason::Core0ExecutorStall,
    WithholdKind::Core0Comms => crate::crash::WithholdReason::Core0Comms,
    WithholdKind::DeadZone => crate::crash::WithholdReason::DeadZone,
  };
  crate::crash::record_withhold(withhold);
  // Track the reason we just recorded so the healthy `None` branch can erase exactly this breadcrumb if the wedge
  // later clears before the dogs reset (Fix #3/H2).
  LAST_CAPTURED_WITHHOLD.store(withhold as u8 as u32, Ordering::Relaxed);
  // The packed `UsbTxStall` word + response length usb_tx last published on a write timeout. `0`/untagged decodes as
  // "no usb_tx stall this run" (the pure decoder rejects it) — correct for a pure core-1/comms wedge with no usb_tx
  // stall. `record_usb_tx_stall` stores the packed word verbatim, so the boot dump's verdict still classifies it.
  let fingerprint = USB_TX_STALL_FINGERPRINT.load(Ordering::Relaxed);
  let fingerprint_len = USB_TX_STALL_FINGERPRINT_LEN.load(Ordering::Relaxed).min(u16::MAX as u32) as u16;
  crate::crash::record_usb_tx_stall(fingerprint, fingerprint_len);
  crate::crash::record_usb_tx_stall_window(USB_TX_STALL_WINDOW_COUNT.load(Ordering::Relaxed).min(u16::MAX as u32) as u16);
}

/// Toggle the GPIO18 scope heartbeat (called once per ISR fire, free-running). Flips [`HEARTBEAT_LEVEL`] and drives
/// GPIO18 to the new level via the raw GPIO set/clear registers — a SINGLE write, no `&mut Output` (which the ISR
/// cannot take). `out_w1ts` (write-1-to-set) and `out_w1tc` (write-1-to-clear) touch ONLY the masked bit, leaving
/// every other output pin untouched, so this is inherently atomic against any other GPIO writer even mid-toggle.
fn toggle_heartbeat() {
  const MASK: u32 = 1 << HEARTBEAT_PIN;
  let regs = esp_hal::peripherals::GPIO::regs();
  // `fetch_xor` flips the tracked level and returns the PRIOR value: `0` (was low) ⇒ drive high; `1` ⇒ drive low. The
  // ISR is the sole writer, so `Relaxed` is race-free.
  if HEARTBEAT_LEVEL.fetch_xor(1, Ordering::Relaxed) == 0 {
    // SAFETY: a single write to the GPIO output-SET register (`out_w1ts`, bits 0..31 for GPIO0-31 — GPIO18 is `< 32`).
    // `bits(MASK)` sets only pin 18; write-1-to-set leaves other outputs unchanged. The parked `Output<GPIO18>` holds
    // the pin as a push-pull output and nothing else drives it, so there is no register race (same PAC-boundary
    // exception as `feed_rwdt_raw`).
    regs.out_w1ts().write(|w| unsafe { w.out_w1ts().bits(MASK) });
  } else {
    // SAFETY: as above, but the output-CLEAR register (`out_w1tc`, write-1-to-clear) drives pin 18 low.
    regs.out_w1tc().write(|w| unsafe { w.out_w1tc().bits(MASK) });
  }
}

/// Drive the GPIO17 withhold-decision probe to `active` via the raw GPIO set/clear registers — a SINGLE write, no
/// `&mut Output` (which the ISR cannot take). `out_w1ts` / `out_w1tc` touch ONLY the masked bit, so this is atomic
/// against any other GPIO writer. Called every fire with the live `watchdog_decision` verdict (§17.13 probe).
fn drive_withhold_probe(active: bool) {
  const MASK: u32 = 1 << WITHHOLD_PROBE_PIN;
  let regs = esp_hal::peripherals::GPIO::regs();
  if active {
    // SAFETY: a single write to the GPIO output-SET register (`out_w1ts`) for pin 17 (`< 32`); write-1-to-set leaves
    // other outputs unchanged. The parked `Output<GPIO17>` holds the pin as push-pull and nothing else drives it, so
    // there is no register race (same PAC-boundary exception as `toggle_heartbeat`).
    regs.out_w1ts().write(|w| unsafe { w.out_w1ts().bits(MASK) });
  } else {
    // SAFETY: as above, but the output-CLEAR register (`out_w1tc`) drives pin 17 low.
    regs.out_w1tc().write(|w| unsafe { w.out_w1tc().bits(MASK) });
  }
}

/// The RWDT write-protect unlock key (matches `esp_hal::rtc_cntl::Rwdt::set_write_protection`).
const RWDT_WKEY: u32 = 0x50D8_3AA1;
/// The SuperWDT write-protect unlock key (matches `esp_hal::rtc_cntl::Swd::set_write_protection`, S3 value).
const SWD_WKEY: u32 = 0x8F1D_312A;

/// Raw-PAC feed of the RTC watchdog: unlock write-protect, pulse the feed bit, re-lock. Mirrors
/// `esp_hal::rtc_cntl::Rwdt::feed` exactly (verified against esp-hal 1.1.1 source) but WITHOUT borrowing the `Rtc`
/// struct — the ISR cannot take `&mut Rtc`. This is the esp-hal/PAC-boundary `unsafe` exception the `firmware` crate
/// permits; confined here and to [`feed_swd_raw`].
fn feed_rwdt_raw() {
  let regs = LP_WDT::regs();
  // SAFETY: a side-effect-free sequence of writes to the RWDT registers of the `LP_WDT` (RTC_CNTL) block — the exact
  // sequence `Rwdt::feed` performs. `bits()` writes the raw write-protect key (esp-hal writes the key the same way);
  // `wdt_feed().set_bit()` pulses the feed. No other context writes these registers in this build (the async
  // `watchdog_feed` does not feed the RWDT here), so there is no register race.
  regs.wdtwprotect().write(|w| unsafe { w.bits(RWDT_WKEY) });
  regs.wdtfeed().write(|w| w.wdt_feed().set_bit());
  regs.wdtwprotect().write(|w| unsafe { w.bits(0) });
}

/// Raw-PAC feed of the SuperWDT: unlock its write-protect, pulse its feed bit, re-lock. The S3 SuperWDT has a SHORT
/// fixed silicon period and no `set_timeout`, so it MUST be software-fed at this cadence (esp-hal's `Swd::enable`
/// only clears auto-feed). Mirrors the `Swd` write-protect key sequence; the `swd_feed` field is the manual feed.
fn feed_swd_raw() {
  let regs = LP_WDT::regs();
  // SAFETY: writes only to the SuperWDT write-protect + config registers of `LP_WDT` (RTC_CNTL). `swd_wkey().bits()`
  // is the documented unlock key (same as `Swd::set_write_protection`); `swd_feed().set_bit()` pulses the manual
  // feed via `modify` so the other `swd_conf` fields (auto-feed-disable set by `Swd::enable`) are preserved. The ISR
  // is the sole writer of these registers in this build, so no race.
  regs.swd_wprotect().write(|w| unsafe { w.swd_wkey().bits(SWD_WKEY) });
  regs.swd_conf().modify(|_, w| w.swd_feed().set_bit());
  regs.swd_wprotect().write(|w| unsafe { w.swd_wkey().bits(0) });
}
