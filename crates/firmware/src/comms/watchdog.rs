//! Watchdog + executor-liveness (§17): the core-0 RTC-watchdog feeder and the diagnostic liveness beat. In the
//! PRODUCTION build [`watchdog_feed`] (a plain thread-mode task) pets the RWDT only while the firmware makes real
//! forward progress and WITHHOLDS the feed — letting the armed 8 s RWDT auto-reset — in three wedge classes
//! (core-0 executor death, a core-1 motion wedge via [`MOTION_LIVENESS`], a core-0 comms stall via `COMMS_PROGRESS`),
//! keyed by [`CORE1_STALL_TICKS`]/[`COMMS_STALL_TICKS`]/[`RX_ACTIVE_TICKS`] and the always-disarmed
//! [`DEAD_ZONE_BACKSTOP_ARMED`]. In the DIAGNOSTIC `capture-reset` build the TIMG1 ISR owns feed/withhold, so this
//! file instead carries [`watchdog_heartbeat`], the ungated core-0 executor-liveness beat. [`provoke_executor_stall`]
//! is the feature-gated stall-provocation diag task that exercises the whole path. Extracted verbatim from `comms.rs`
//! (architecture-refactor A1, step 13). The three tasks stay `pub` (spawned from `main`) and `WATCHDOG_FEED_INTERVAL`
//! stays `pub(crate)` (documented so a stall detector can derive its debounce from the same interval); the tick
//! consts stay private. `comms.rs` re-exports this module (`pub(crate) use watchdog::*;`); `use super::*` supplies the
//! whole parent surface (the liveness statics/atomics, the `crate::crash`/`crate::survivable_watchdog`/`diag`
//! helpers, and the firmware_core types) — no explicit imports needed.

use super::*;

/// How often the [`watchdog_feed`] task pets the RTC watchdog, in milliseconds. Must be COMFORTABLY shorter than the
/// RWDT stage-0 timeout (`WATCHDOG_TIMEOUT` in `main`, 8 s) so several feeds fall inside one timeout window and a
/// single late wake (e.g. a brief flash-write quiesce that parks core 0 for tens of ms) cannot trip the dog. 500 ms
/// gives a 16x margin: the timeout only expires after ~8 s of the core-0 thread-mode executor never running this
/// task at all — i.e. a genuine core-0 wedge, exactly the condition we want a reset for, never a normal stall.
// `pub(crate)` so the production stall detector can derive its recovery-clear debounce (Fix #3) from the SAME feed
// interval — the debounce must span at least one feed so the RWDT is provably re-fed before a breadcrumb is erased.
pub(crate) const WATCHDOG_FEED_INTERVAL: Duration = Duration::from_millis(500);

/// Number of consecutive [`WATCHDOG_FEED_INTERVAL`] ticks over which the core-1 liveness beat may stay frozen WHILE
/// a block is actively executing before [`watchdog_feed`] declares a core-1-only wedge and withholds the feed (Goal
/// B). At 500 ms/tick this is ~4 s — chosen CONSERVATIVELY above the worst-case legitimate gap between core-1 beats:
/// the executor bumps [`MOTION_LIVENESS`] per BURST (see `motion::RmtStepSink::emit_burst`), and the slowest possible
/// single burst is `MAX_SYMBOLS_PER_BURST` events × the max RMT period (`RMT_MAX_FIELD_LEN + $0` ≈ 0x7FFF ticks ≈
/// 33 ms at 1 MHz) ≈ 1.5 s, so 4 s leaves >2.5x margin and CANNOT false-trip on a real slow move. Only a genuine
/// "a block is in flight but core 1 has emitted no burst for 4 s" — i.e. core 1 wedged mid-motion (the suspected
/// RMT `wait()` spin) — withholds the feed; the already-armed 8 s RWDT then resets the board, converting an
/// otherwise-silent core-1-only stall into a recoverable reset + a captured breadcrumb.
///
/// PRODUCTION-only: the DIAGNOSTIC `capture-reset` build feeds via the TIMG1 ISR ([`crate::survivable_watchdog`]),
/// which keeps its OWN stall thresholds, so the async [`watchdog_feed`] and these constants are not built there.
#[cfg(not(feature = "capture-reset"))]
const CORE1_STALL_TICKS: u32 = 8;

/// Number of consecutive [`WATCHDOG_FEED_INTERVAL`] ticks the [`COMMS_PROGRESS`] counter may stay frozen WHILE the
/// host is active before [`watchdog_feed`] declares a CORE-0 COMMS wedge and withholds the feed. At 500 ms/tick this
/// is ~3 s. skirnir polls `?` every ~200 ms whenever connected and the firmware answers in EVERY state, so a
/// connected-and-active board advances `COMMS_PROGRESS` ~5 Hz (every tick) via three independent tasks
/// (`status_responder`, `usb_tx`, `comms_consumer`); if ALL THREE freeze for 3 s while the host is still present, the
/// comms path is genuinely wedged — the real-board failure (writes succeed, no responses, DRO frozen) — with no
/// legitimate counterexample (back-pressure still leaves `?` answered). ~3 s is short enough that the trip fires
/// WHILE the host is still flowing or recently-flowing RX (see [`RX_ACTIVE_TICKS`]); three independent bumpers + the
/// host-active gate keep it from false-tripping.
#[cfg(not(feature = "capture-reset"))]
const COMMS_STALL_TICKS: u32 = 6;

/// The "host is present" sticky window, in [`WATCHDOG_FEED_INTERVAL`] ticks since [`RX_ACTIVITY`] last advanced. The
/// host counts as active for ~6 s after its last received byte. This is BOTH the reset-loop guard AND the bridge
/// across the host's flow-control quiet gap: when the comms path wedges, the host keeps streaming only until its
/// character-counting window fills (it got no `ok`s) — on the real board skirnir sent ~30 more lines (~1-2 s) then
/// went quiet. A 6 s sticky window keeps the host "active" across that quiet gap so the ~3 s [`COMMS_STALL_TICKS`]
/// trip still fires, while a board with NO host (RX never advances) goes inactive after 6 s and FEEDS NORMALLY
/// forever — never a reset-loop. The counter is SEEDED idle (host inactive) at task start, so a board that boots
/// with no host present never spuriously counts as active before the first real RX byte.
#[cfg(not(feature = "capture-reset"))]
const RX_ACTIVE_TICKS: u32 = 12;

/// Whether the §13.4 dead-zone backstop WITHHOLD is armed. The backstop forces a `software_reset()` (via an RWDT
/// withhold) on an absolute "responses queued + usb_tx idle ~8 s" deadline — it converts a Signature-B SILENT lock
/// into a breadcrumb-bearing reset, which is the ONLY way that wedge leaves a trace. Per the TIER 2/3 split (§17) a
/// silent reset the host streams through corrupts a real cut (it resumes cutting in the wrong place, §14.3), so the
/// backstop is armed ONLY in the DIAGNOSTIC `capture-reset` build (operator-gated, scrap expected — the open
/// Signature-B capture channel, #21). In the PRODUCTION default it is DISARMED: a genuinely dead task cannot raise an
/// `ALARM` (the comms path is wedged), so the only recovery would be exactly the silent reset the redesign forbids —
/// production accepts a fail-safe HALT over a part-corrupting auto-reset (the user's option-A decision). TIER 1 + the
/// K-escape→`ALARM:17` conversion remove the COMMON wedges, so reaching a true dead zone in production is rare; if it
/// happens the board halts (no reset) until the operator power-cycles, which is strictly safer than a wrong cut. The
/// `dead_zone` condition is still COMPUTED in both builds so the tracking + the host-tested decision stay exercised
/// and warning-clean; only this arming flag differs.
///
/// PRODUCTION-only: the dead-zone backstop is always DISARMED in the production [`watchdog_feed`] (a silent reset
/// would corrupt a real cut; production fails safe instead). In the DIAGNOSTIC `capture-reset` build the dead-zone is
/// implemented ENTIRELY by the TIMG1 ISR ([`crate::survivable_watchdog`]) — which feeds/withholds both dogs — so the
/// async feeder and this flag are not built there (hence the single `not(capture-reset)` definition).
#[cfg(not(feature = "capture-reset"))]
const DEAD_ZONE_BACKSTOP_ARMED: bool = false;

/// The RTC watchdog feed task (core 0 / PRO_CPU, a plain thread-mode task) — a proper TASK-watchdog (revised after a
/// real-board wedge where the Embassy executor stayed alive but the comms path was stuck on a never-resolving
/// `.await`, so the original unconditional feed kept the dog quiet and a physical EN-reset was needed). It feeds the
/// RWDT only while the firmware is making real forward progress, and WITHHOLDS the feed — letting the already-armed
/// 8 s RWDT auto-reset the board — in three wedge classes:
/// 1. **Core-0 executor death** (a hang/deadlock/fault that wedged the whole thread-mode executor): this task simply
///    never runs, so the dog is never fed. Caught implicitly, no logic needed.
/// 2. **Core-1 motion wedge** (the suspected RMT `wait()` spin): [`MOTION_LIVENESS`] frozen for [`CORE1_STALL_TICKS`]
///    WHILE `EXECUTOR_RUNNING` (a block in flight). Idle/parked/dwell clear `EXECUTOR_RUNNING`, so they never trip.
/// 3. **Core-0 comms stall** (the NEW case — executor alive but the comms pipeline stuck): [`COMMS_PROGRESS`] frozen
///    for [`COMMS_STALL_TICKS`] WHILE the host is active ([`RX_ACTIVITY`] advanced within [`RX_ACTIVE_TICKS`]).
/// Either withhold records its reason in the [`crate::crash`] breadcrumb, so the boot `[MSG:CRASH ...]` names the
/// wedge class (`core1-motion-wedge` vs `core0-comms-wedge`).
///
/// ## Reset-loop / false-trip safety (the load-bearing guards)
/// - The comms-stall withhold is gated on RX activity, so a QUIESCENT or DISCONNECTED board (no host polling, so
///   `COMMS_PROGRESS` naturally sits still) is NEVER reset — there is no host to serve, so a still counter is
///   correct, not a wedge. This is what prevents a boot→reset→boot loop on a board left sitting at a prompt.
/// - `COMMS_PROGRESS` is bumped by THREE independent tasks; legitimate back-pressure (the consumer blocked in a
///   `QueueFull` retry) still leaves `status_responder`/`usb_tx` answering `?`, so the counter keeps advancing — a
///   stall requires ALL host-facing work to stop, which is the genuine wedge.
/// - The core-1 check is unchanged (block-in-flight gated), so it cannot false-trip on idle/hold/dwell.
///
/// ## Breadcrumb snapshots
/// Each tick pushes a `(seq, comms_progress, motion_liveness)` snapshot into the RTC_FAST crash ring
/// ([`crate::crash::push_snapshot`]) — OFF the real-time path — so after a reset the boot dump can show which side
/// stopped advancing FIRST. The feed task NO LONGER self-bumps `COMMS_PROGRESS` (that polluted the verdict and
/// masked the comms wedge); the snapshot reads the genuine, work-driven counters.
///
/// `Rtc::rwdt::feed` takes `&mut self`, so the task owns the `Rtc` by `&'static mut` (parked in a `StaticCell` in
/// `main`); it is the SOLE feeder, so no lock is needed.
///
/// PRODUCTION-only. The DIAGNOSTIC `capture-reset` build feeds via the TIMG1 hardware-timer ISR
/// ([`crate::survivable_watchdog`]) — which survives a core-0 executor stall this cooperative task would not — and
/// runs a thin [`watchdog_heartbeat`] for the snapshot ring instead. This task is built UNCHANGED in the default
/// build.
#[cfg(not(feature = "capture-reset"))]
#[embassy_executor::task]
pub async fn watchdog_feed(rtc: &'static mut esp_hal::rtc_cntl::Rtc<'static>) -> ! {
  // Previous samples + frozen-tick counts for the two conditional withholds. Seeded from the first read so the first
  // delta is meaningful rather than a spurious "moved from 0".
  let mut last_core1 = MOTION_LIVENESS.load(Ordering::Relaxed);
  let mut last_comms = COMMS_PROGRESS.load(Ordering::Relaxed);
  let mut last_rx = RX_ACTIVITY.load(Ordering::Relaxed);
  let mut last_tx_completed = USB_TX_COMPLETED.load(Ordering::Relaxed);
  let mut core1_frozen_ticks: u32 = 0;
  let mut comms_frozen_ticks: u32 = 0;
  // How many consecutive intervals `usb_tx` has completed NO write — the dead-zone backstop input (Signature B).
  let mut tx_complete_frozen_ticks: u32 = 0;
  // Seed the RX-idle counter at the threshold so the host starts INACTIVE: a board that boots with no host present
  // must not count as "host active" before the first real RX byte arrives (else the comms-stall check could trip on
  // a host-less board in the first few seconds — a reset loop). The first RX advance resets this to 0.
  let mut rx_idle_ticks: u32 = RX_ACTIVE_TICKS;
  // Fix #3/H2 recovery-clear: the wedge class this task last RECORDED, held across iterations so a detected-then-
  // recovered wedge (frozen counters reset → we fall through to the feed path) can erase its now-stale breadcrumb.
  // DECLARED OUTSIDE THE LOOP IS LOAD-BEARING: inside the loop it would reset to `None` on every feed-path entry and
  // the clear would never fire (a silent no-op). Set in the wedge block; consumed AFTER the feed on the healthy path.
  let mut withheld: Option<crate::crash::WithholdReason> = None;
  loop {
    // Free-running heartbeat (Signature-B instrumentation): bumped EVERY iteration, unconditionally, so the boot
    // dump's `wdog=` value reveals whether THIS task ran through a wedge (climbed → B-1 fed-but-fooled) or died
    // (froze → B-2). Off the gated logic below, so it is a pure "did the feed loop execute" beat.
    crate::crash::bump_watchdog_heartbeat();
    // The UNGATED core-0 executor-liveness beat (Design A, §20): this production feeder is the liveness producer, so
    // `EXECUTOR_ALIVE` advances iff the core-0 executor is scheduling tasks. A full executor stall stops this loop →
    // the beat FREEZES → the production `crate::stall_detector` TIMG1 ISR (which survives the stall) records the
    // `core0-executor-stall` breadcrumb before the (also unfed) RWDT resets the board. UNCONDITIONAL, like the
    // heartbeat above.
    EXECUTOR_ALIVE.fetch_add(1, Ordering::Relaxed);

    let core1 = MOTION_LIVENESS.load(Ordering::Relaxed);
    let comms = COMMS_PROGRESS.load(Ordering::Relaxed);
    let rx = RX_ACTIVITY.load(Ordering::Relaxed);
    let tx_completed = USB_TX_COMPLETED.load(Ordering::Relaxed);

    // Core-1 motion-stall detection: beat frozen WHILE a block is in flight. `EXECUTOR_RUNNING` false (idle / parked
    // / dwell) resets the count, so a legitimately non-advancing beat is never a stall.
    let block_in_flight = EXECUTOR_RUNNING.load(Ordering::Acquire);
    if block_in_flight && core1 == last_core1 {
      core1_frozen_ticks = core1_frozen_ticks.saturating_add(1);
    } else {
      core1_frozen_ticks = 0;
    }

    // Host-activity ageing: how many consecutive ticks since RX last advanced. Resets to 0 on any RX advance.
    if rx == last_rx {
      rx_idle_ticks = rx_idle_ticks.saturating_add(1);
    } else {
      rx_idle_ticks = 0;
    }
    let host_active = rx_idle_ticks < RX_ACTIVE_TICKS;

    // Core-0 comms-stall detection: `COMMS_PROGRESS` frozen WHILE the host is active. When the host is NOT active
    // (no recent RX → idle/disconnected) the count is held at 0 — a still counter with no host is correct, never a
    // wedge — which is the reset-loop guard. Only "host driving + no comms forward progress" accrues toward a reset.
    if host_active && comms == last_comms {
      comms_frozen_ticks = comms_frozen_ticks.saturating_add(1);
    } else {
      comms_frozen_ticks = 0;
    }

    // Dead-zone tracking (Signature B): how many consecutive intervals `usb_tx` has completed NO write. Resets on any
    // completed/recovered write. UNGATED by host_active / executor state — that is the whole point: the dead zone is
    // exactly "host quiet + executor idle", where the other two detectors are blind.
    if tx_completed == last_tx_completed {
      tx_complete_frozen_ticks = tx_complete_frozen_ticks.saturating_add(1);
    } else {
      tx_complete_frozen_ticks = 0;
    }

    last_core1 = core1;
    last_comms = comms;
    last_rx = rx;
    last_tx_completed = tx_completed;

    // Push a liveness snapshot (genuine work-driven counters) into the RTC_FAST crash ring so a reset's boot dump
    // can determine which side stopped advancing first.
    crate::crash::push_snapshot(comms, core1);

    let core1_wedged = core1_frozen_ticks >= CORE1_STALL_TICKS;
    let comms_wedged = comms_frozen_ticks >= COMMS_STALL_TICKS;
    // Dead-zone backstop (Signature B): responses queued yet usb_tx completed nothing for ~8 s, INDEPENDENT of
    // host_active / executor state. Pure decision in host-tested `dead_zone_withhold`; the depth read is a cheap
    // channel `len()`. ARMED only when [`DEAD_ZONE_BACKSTOP_ARMED`] — held OUT of the Signature-A `wstg` re-capture
    // so the backstop (the one behavior change) cannot perturb the wedge dynamics; the condition is still computed so
    // the tracking + host-tested decision stay exercised.
    let dead_zone = DEAD_ZONE_BACKSTOP_ARMED && firmware_core::diag::dead_zone_withhold(RESPONSE.len(), tx_complete_frozen_ticks);

    if core1_wedged || comms_wedged || dead_zone {
      // A genuine wedge: record the class in the breadcrumb, then WITHHOLD the feed and let the 8 s RWDT reset the
      // board. Precedence: core-1 (most specific stage marker), then core-0 comms, then the dead-zone backstop. We
      // still await so we never busy-spin core 0 while the dog runs out.
      let reason = if core1_wedged {
        crate::crash::WithholdReason::Core1Motion
      } else if comms_wedged {
        crate::crash::WithholdReason::Core0Comms
      } else {
        crate::crash::WithholdReason::DeadZone
      };
      crate::crash::record_withhold(reason);
      // Remember the class we just recorded so a LATER healthy tick (frozen counters reset → fed) can erase the stale
      // breadcrumb after the feed (Fix #3/H2). We overwrite any prior pending reason — `record_withhold` already
      // overwrote the breadcrumb word, so tracking the latest keeps the two consistent.
      withheld = Some(reason);
      #[cfg(feature = "defmt")]
      if core1_wedged {
        defmt::error!("watchdog: core-1 wedged mid-motion ({=u32} ticks) — withholding feed to force reset", core1_frozen_ticks);
      } else if comms_wedged {
        defmt::error!("watchdog: core-0 comms stalled ({=u32} ticks, host active) — withholding feed to force reset", comms_frozen_ticks);
      } else {
        defmt::error!("watchdog: dead-zone silent lock (usb_tx idle {=u32} ticks, responses queued) — withholding feed", tx_complete_frozen_ticks);
      }
      Timer::after(WATCHDOG_FEED_INTERVAL).await;
      continue;
    }

    // Healthy, idle, or host-absent: pet the dog. The whole loop body is a handful of atomic ops + one await, so it
    // can never delay the feed past the 8 s timeout.
    rtc.rwdt.feed();
    // Fix #3/H2: a wedge recorded on a PRIOR iteration whose condition has since cleared (frozen counters reset → we
    // reached this feed path) left a stale breadcrumb; erase it AFTER the feed — the feed cancels the pending reset, so
    // clearing after it is safe even on a knife-edge RWDT expiry (clearing BEFORE could boot UNLOCKED). `clear_withhold_if`
    // is a CAS that no-ops unless the word still holds exactly that reason, so it never clobbers another writer.
    if let Some(reason) = withheld.take() {
      crate::crash::clear_withhold_if(reason);
    }
    #[cfg(feature = "defmt")]
    {
      if core1_frozen_ticks > 0 {
        defmt::warn!("watchdog: core-1 beat frozen ({=u32}/{=u32} ticks) while executing", core1_frozen_ticks, CORE1_STALL_TICKS);
      }
      if comms_frozen_ticks > 0 {
        defmt::warn!("watchdog: comms frozen ({=u32}/{=u32} ticks) while host active", comms_frozen_ticks, COMMS_STALL_TICKS);
      }
    }
    Timer::after(WATCHDOG_FEED_INTERVAL).await;
  }
}

/// The DIAGNOSTIC capture build's thin watchdog-companion task (`capture-reset`, §17.10/§17.11). In this build the
/// TIMG1 hardware-timer ISR ([`crate::survivable_watchdog`]) OWNS feeding both dogs — it survives a core-0 executor
/// stall that would starve a cooperative async feeder. This task therefore does NOT feed the RWDT; it only runs the
/// off-real-time diagnostic SNAPSHOT RING ([`crate::crash::push_snapshot`]) every [`WATCHDOG_FEED_INTERVAL`] so the
/// boot dump can still show which side's beat (`COMMS_PROGRESS` vs `MOTION_LIVENESS`) stopped advancing first. The
/// free-running heartbeat is bumped by the ISR (the live feeder), so this task does not bump it. If THIS task is
/// starved by the same wedge, the snapshots simply stop — the ISR still feeds/withholds independently, so the capture
/// is unaffected. It takes no `Rtc` (the ISR feeds register-side), so there is no `&mut Rtc` borrow here.
#[cfg(feature = "capture-reset")]
#[embassy_executor::task]
pub async fn watchdog_heartbeat() -> ! {
  loop {
    // The UNGATED core-0 executor-liveness beat (§17.15 fix): bumped every iteration, unconditionally. This runs on
    // the core-0 thread-mode executor, so it advances iff that executor is scheduling tasks — a healthy board keeps it
    // climbing whether idle or busy, and a full executor stall FREEZES it. The survivable TIMG1 ISR watches this beat
    // and withholds both dogs on a freeze — the one detector not blind to the host-quiet + motion-idle + RESPONSE-empty
    // wedge. Bumped FIRST so even if `push_snapshot` were to stall, the liveness proof still advances.
    EXECUTOR_ALIVE.fetch_add(1, Ordering::Relaxed);
    // Push a liveness snapshot (genuine work-driven counters) into the RTC_FAST crash ring so a reset's boot dump can
    // determine which side stopped advancing first. Off the real-time path; no feeding here (the ISR owns the dogs).
    let core1 = MOTION_LIVENESS.load(Ordering::Relaxed);
    let comms = COMMS_PROGRESS.load(Ordering::Relaxed);
    crate::crash::push_snapshot(comms, core1);
    Timer::after(WATCHDOG_FEED_INTERVAL).await;
  }
}

/// DIAGNOSTIC (`provoke-executor-stall`, §17.18): deterministically induce a full core-0 executor stall to validate the
/// §17.17 executor-liveness FIX end-to-end. Waits ~10 s so the board boots and `EXECUTOR_ALIVE` establishes a healthy
/// climb (and any stream reaches steady state), then enters a NON-YIELDING busy loop that monopolizes the cooperative
/// core-0 thread-mode executor — starving every other core-0 task including [`watchdog_heartbeat`], so `EXECUTOR_ALIVE`
/// FREEZES. This is the exact stall CLASS the fix targets (a task wedged in a non-yielding section), induced on demand.
/// Two consumers select this via a feature:
/// - `provoke-executor-stall` (pulls `capture-reset`, §17.18): the survivable TIMG1 ISR detects the freeze after
///   [`EXECUTOR_STALL_TICKS`], withholds both dogs, and the SuperWDT resets ~6 s later with a `MSG:RESET super-WDT` +
///   `MSG:CRASH core0-executor-stall …` breadcrumb — validating the FIX's detector→withhold→dog chain.
/// - `provoke-stall-bare` (NO `capture-reset`, §20): the PRODUCTION recovery path — the async `watchdog_feed` (sole
///   RWDT feeder) dies in the stall so the unfed RWDT resets at ~8 s (`MSG:RESET …rtc-WDT`), AND the production
///   detector-only `crate::stall_detector` ISR records the `core0-executor-stall` breadcrumb → the next boot comes up
///   in the fail-safe alarm (`ALARM:11` / `ALARM:3`). Exercises Design A end-to-end.
///
/// Never returns (the busy loop runs until the reset).
#[cfg(any(feature = "provoke-executor-stall", feature = "provoke-stall-bare"))]
#[embassy_executor::task]
pub async fn provoke_executor_stall() -> ! {
  Timer::after(Duration::from_secs(10)).await;
  // A never-yielding loop: the cooperative executor can now schedule nothing else on core 0 → `EXECUTOR_ALIVE` freezes.
  // `spin_loop` is a hint only; the point is that this future never `.await`s again, so the executor never regains
  // control. The hardware ISR still preempts and runs the detector.
  loop {
    core::hint::spin_loop();
  }
}
