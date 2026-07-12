//! Production core-0 EXECUTOR-STALL DETECTOR (Design A, `docs/streaming-lockup-investigation.md` §20). A TIMG1
//! hardware-timer ISR that watches the ungated [`EXECUTOR_ALIVE`] beat and, when it FREEZES (a full core-0 async
//! executor stall — the one wedge class §18/§19 Option B did NOT eliminate), records the `core0-executor-stall`
//! breadcrumb so the NEXT boot comes up LOCKED in the fail-safe alarm instead of silently resuming.
//!
//! ## Detector-ONLY — it resets nothing and feeds nothing
//! Unlike the capture-reset [`crate::survivable_watchdog`], this ISR does NOT feed or withhold any dog, does NOT arm
//! the SuperWDT, and does NOT drive any GPIO. In production the RWDT is fed by the async [`crate::comms::watchdog_feed`]
//! (UNCHANGED, sacred path), and a full executor stall kills that feeder → the RWDT goes UNFED and resets the board on
//! its own at ~8 s (verified `sys-rtc-WDT`, §20.5). This ISR only needs to leave a breadcrumb in the ~4 s→~8 s window
//! between detecting the freeze and that reset. So it CANNOT false-reset a healthy board (it resets nothing) and — like
//! the capture-reset ISR — it touches ONLY a few `AtomicU32`s + one RTC_FAST breadcrumb word, NEVER a mutex, `&mut Rtc`,
//! RMT, or core-1 state, so it adds ZERO lock contention or latency to the core-1 step-timing path.
//!
//! ## Why a TIMG1 ISR (survives the stall)
//! A hardware-timer ISR fires even when the core-0 thread-mode executor is fully stalled (its idle hook is `waiti 0`,
//! the thread run-level masks nothing, no `interrupt_free` wraps the executor loop). TIMG1 is independent of the
//! esp-rtos / embassy time driver (TIMG0). Configured in `main` on ProCpu, so it is core-0-fielded — which is what we
//! want, since [`EXECUTOR_ALIVE`] is written from core 0. Production uses TIMG1 for this detector; the capture-reset
//! build uses TIMG1 for the survivable watchdog instead — the two never coexist (`capture-reset` vs default).

use core::sync::atomic::Ordering;

use esp_hal::time::Duration;
use esp_hal::timer::PeriodicTimer;
use esp_hal::timer::timg::TimerGroup;
use portable_atomic::AtomicU32;
use static_cell::StaticCell;

use crate::comms::EXECUTOR_ALIVE;

/// The sample cadence: every 250 ms the ISR checks whether [`EXECUTOR_ALIVE`] advanced. Matches the capture-reset
/// survivable ISR so the stall threshold means the same wall-clock time (`16 * 250 ms = 4 s`).
const SAMPLE_INTERVAL: Duration = Duration::from_millis(250);

/// Consecutive frozen samples before declaring a full core-0 executor stall. `16 * 250 ms = 4 s` — comfortably above
/// any legitimate core-0 quiesce, well below the ~8 s the unfed RWDT takes to reset (so the breadcrumb lands first).
/// Mirrors [`firmware_core::diag::EXECUTOR_STALL_TICKS`].
const EXECUTOR_STALL_TICKS: u32 = firmware_core::diag::EXECUTOR_STALL_TICKS;

/// The TIMG1 `PeriodicTimer`, parked `'static` so it (and its bound ISR) outlive `main`. The ISR clears the timer's
/// interrupt flag through this handle each fire — the only mutable access, and the ISR is the sole accessor after
/// [`start`] returns, so a `'static` `&mut` is sound.
static TIMER: StaticCell<PeriodicTimer<'static, esp_hal::Blocking>> = StaticCell::new();

/// A raw pointer to the parked [`TIMER`], published once by [`start`] so the bare `extern "C"` ISR can reach the timer
/// to clear its interrupt flag. Written once before the interrupt is enabled; read only inside the ISR.
static TIMER_PTR: AtomicU32 = AtomicU32::new(0);

/// Last-sampled [`EXECUTOR_ALIVE`] beat. Sentinel `u32::MAX` = "unseeded": the first fire seeds it so the first delta is
/// "no change" rather than a spurious move.
static LAST_EXECUTOR_ALIVE: AtomicU32 = AtomicU32::new(u32::MAX);

/// Consecutive samples the [`EXECUTOR_ALIVE`] beat has stayed frozen. Only accrues once the beat has advanced past its
/// initial `0` (the boot guard — the feeder must have run at least once before a freeze counts as a stall).
static EXECUTOR_ALIVE_FROZEN: AtomicU32 = AtomicU32::new(0);

/// Whether the `core0-executor-stall` breadcrumb was already recorded this stall, so it is written exactly ONCE per
/// stall (not every 250 ms until the RWDT resets). Reset to `0` whenever the beat advances (the executor recovered).
static STALL_LATCHED: AtomicU32 = AtomicU32::new(0);

/// Build the TIMG1 periodic timer, bind the [`fire`] ISR, and start it. Called once from `main` on ProCpu (so the
/// interrupt is core-0-fielded), in the PRODUCTION (non-`capture-reset`) build only. Detector-only: it never touches
/// the RWDT the async [`crate::comms::watchdog_feed`] owns.
pub fn start(timg1: esp_hal::peripherals::TIMG1<'static>) {
  let timg1 = TimerGroup::new(timg1);
  let mut timer = PeriodicTimer::new(timg1.timer0);
  // Binds the handler AND enables the TG1_T0 CPU interrupt on the current core (ProCpu). Not listening yet, so no fire
  // can occur until `listen()` below — which we call only AFTER `TIMER_PTR` is published, so the first fire can always
  // reach the timer to clear its flag.
  timer.set_interrupt_handler(fire);
  // `start` returns `Err` only on a bad period (zero / out of range); 250 ms is valid. `expect` is permitted in this
  // init path (CLAUDE.md): a failure here is an unrecoverable static wiring bug, not a runtime condition.
  timer.start(SAMPLE_INTERVAL).expect("start TIMG1 stall-detector timer");
  let timer: &'static mut PeriodicTimer<'static, esp_hal::Blocking> = TIMER.init(timer);
  // Publish the parked-timer pointer BEFORE enabling the alarm interrupt. `Release` pairs with the ISR's `Acquire`.
  TIMER_PTR.store(timer as *mut _ as u32, Ordering::Release);
  timer.listen();
}

/// The TIMG1 ISR (core-0-fielded): sample [`EXECUTOR_ALIVE`], track the frozen-sample count (boot-guarded), and on
/// crossing [`EXECUTOR_STALL_TICKS`] record the `core0-executor-stall` breadcrumb ONCE. Feeds/withholds NOTHING — the
/// unfed RWDT does the resetting. MUST clear the TIMG1 interrupt flag each fire or it re-enters immediately.
#[esp_hal::handler]
fn fire() {
  // Clear the timer interrupt FIRST so we never re-enter. Reach the parked timer through the raw pointer set by
  // `start`. If `start` has not run yet (pointer still 0) there is nothing to clear.
  let timer_ptr = TIMER_PTR.load(Ordering::Acquire) as *mut PeriodicTimer<'static, esp_hal::Blocking>;
  if !timer_ptr.is_null() {
    // SAFETY: `timer_ptr` came from a `&'static mut PeriodicTimer` parked in a `StaticCell` (lives for the program),
    // written once with `Release` before the interrupt was enabled. The ISR is the SOLE accessor of the timer after
    // `start` returns, so no other `&mut` aliases it. `clear_interrupt` only writes the TIMG1 int-clear register.
    unsafe {
      (*timer_ptr).clear_interrupt();
    }
  }

  // Sample the executor-liveness beat. Seed the sentinel on the first fire so the first delta is "no change".
  let alive = EXECUTOR_ALIVE.load(Ordering::Relaxed);
  let last = {
    let prev = LAST_EXECUTOR_ALIVE.swap(alive, Ordering::Relaxed);
    if prev == u32::MAX { alive } else { prev }
  };

  // Frozen iff the beat did not advance — gated ONLY by the boot guard `alive != 0` (once the production feeder has run
  // at least once, ANY subsequent freeze is a full-executor stall, regardless of host / motion / response state). The
  // `!= 0` guard stops the pre-first-bump zero (before the feeder is scheduled) from accruing a false stall at boot.
  let frozen = alive != 0 && alive == last;
  let frozen_ticks = if frozen {
    EXECUTOR_ALIVE_FROZEN.load(Ordering::Relaxed).saturating_add(1)
  } else {
    0
  };
  EXECUTOR_ALIVE_FROZEN.store(frozen_ticks, Ordering::Relaxed);

  if frozen_ticks >= EXECUTOR_STALL_TICKS {
    // A full core-0 executor stall: record the breadcrumb ONCE (the RWDT — unfed since the async feeder died — resets
    // the board within a few seconds; the breadcrumb survives that WDT reset and the next boot comes up locked).
    if STALL_LATCHED.swap(1, Ordering::Relaxed) == 0 {
      crate::crash::record_withhold(crate::crash::WithholdReason::Core0ExecutorStall);
    }
  } else if !frozen {
    // The executor advanced — clear the latch so a future stall records afresh.
    STALL_LATCHED.store(0, Ordering::Relaxed);
  }
}
