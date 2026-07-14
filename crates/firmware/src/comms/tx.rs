//! The USB transmit path (DOC-08 writer half): the single `usb_tx` writer task and its poll-based write core
//! (`write_response_polled` + the K-consecutive-stall wedge handlers), the guaranteed-delivery `enqueue`
//! primitive and the response emitters built on it (`send_banner`, `send_reset_reason`, `ack`, `error`,
//! `error_bare`), and the boot crash-report emit/stash/replay (`maybe_emit_crash_report`, the
//! `CRASH_REPORT`/`RESET_REPORT` stashes, and the `take_pending_*` replay drains) — extracted verbatim from
//! `comms.rs` (architecture-refactor A1, step 4). Everything here is the one place that turns a `Response` into
//! bytes on the wire or queues one for that writer; centralizing the writes keeps `ok`s, errors, status, and
//! `$`-reports from interleaving on the USB endpoint. The crash-emit helpers stash+emit through `enqueue`, so
//! they live with the tx path. The pure crash-type → `Response` formatters stay in `crash_report.rs` and are
//! reached via `super::` (the `comms.rs` re-export). `comms.rs` re-exports this module
//! (`pub(crate) use tx::*;`) so the `main.rs` spawns/boot calls and the many `enqueue`/`ack`/`error` call sites
//! keep resolving unqualified.

use core::cell::Cell;
use core::sync::atomic::Ordering;

use embassy_futures::yield_now;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_time::{Duration, Instant};
use esp_hal::usb_serial_jtag::UsbSerialJtagTx;
use esp_hal::Async;

use firmware_core::protocol::ResponseWriter;

// The moved code references a wide surface of parent state statics + the `crash_report::format_*` fns (several
// `capture-reset`-gated), all re-exported by `comms.rs`; a glob keeps the relocated bodies verbatim without a
// pile of individually cfg-attributed `use` lines. Parent-owned external types are imported explicitly above.
use super::*;

/// Enqueue a response for the USB writer, blocking until it is accepted so a `ok`/`error:N`/report is
/// NEVER dropped — the one-`ok`-per-line contract that drives host flow control depends on guaranteed
/// delivery (a lost `ok` permanently stalls a character-counting host). Blocking here is safe because every
/// caller of this function runs OFF the real-time path: real-time command dispatch lives entirely in the
/// reader half ([`usb_rx`]), which never enqueues responses, so a momentarily full [`RESPONSE`] channel can
/// only back-pressure the response producers (consumer / status reporter), never delay a real-time byte.
pub(crate) async fn enqueue(resp: Response) {
  RESPONSE.send(resp).await;
}

/// Render a banner into a fresh [`Response`] and queue it, blocking until accepted. Emitted on boot so a
/// host detects controller readiness (DOC-08 / the native-USB no-hard-reset rule). The soft-reset banner is
/// emitted by the consumer's pipeline reset, not here, so this never runs on the reader half.
pub async fn send_banner() {
  let mut s = Response::new();
  if ResponseWriter::banner(&mut s).is_ok() {
    enqueue(s).await;
  }
}

/// Emit a `[MSG:RESET <reason>]` line over the grbl TX at boot, UNCONDITIONALLY (Signature-B instrumentation). The
/// `reason` is the PRO_CPU reset-reason label from `main`'s `log_reset_reason` (e.g. `power-on`, `brown-out (power)`,
/// `core-rtc-WDT (auto-recovered from a wedge)`, `core-sw-reset`). Unlike the `[MSG:CRASH ...]` dump — which is gated
/// on a valid RTC_FAST breadcrumb AND a watchdog/fault reset — this ALWAYS fires, so a no-breadcrumb boot (clean
/// power-on, a brown-out that wiped RTC_FAST, or the silent-lock case that wrote nothing) still tells the host WHY it
/// reset. That single datum settles "a reset DID fire" vs "no software reset (dead-zone hang / brown-out)" on the
/// next boot. Pure formatting + one enqueue; a capacity failure simply skips the line.
pub async fn send_reset_reason(reason: &str) {
  let mut inner: heapless::String<64> = heapless::String::new();
  use core::fmt::Write as _;
  if write!(inner, "RESET {reason}").is_ok() {
    let mut s = Response::new();
    if ResponseWriter::message(&mut s, inner.as_str()).is_ok() {
      // Stash a copy for the first-`$I`/`?` replay BEFORE the live emit (capture build only), so a host that
      // connects and streams from byte 0 — missing the live boot-time emit — still learns WHICH dog fired on its
      // first request. Mirrors the `CRASH_REPORT` stash; kept in its own buffer (see `RESET_REPORT`).
      #[cfg(feature = "capture-reset")]
      RESET_REPORT.lock(|c| c.set(Some(s.clone())));
      enqueue(s).await;
    }
  }
}

/// The formatted post-mortem crash report lines: the `[MSG:CRASH ...]` summary, plus (when present) a `[MSG:CRASH
/// rmt0: ...]` RMT register-detail line and a `[MSG:CRASH comms: ...]` per-task comms-stage line. Held after boot
/// so they can be RE-EMITTED on the first `$I`/status request after a host connects. The native-USB link
/// re-enumerates on the watchdog reset, so a host that reconnects a beat late would miss the boot-time emission;
/// stashing the lines here and replaying them on the first `$I`/`?` closes that race. Empty once there is nothing
/// left to replay. A `heapless::Vec<Response, 6>` behind the cross-core blocking mutex keeps it `Send` and
/// allocation-free; each `Response` is far under [`RESPONSE_CAPACITY`], so splitting into separate lines (rather
/// than one over-long line) keeps every line in budget. Capacity 6 = panic + summary + rmt + usbtx + comms + rec lines.
static CRASH_REPORT: BlockingMutex<CriticalSectionRawMutex, Cell<heapless::Vec<Response, 6>>> =
  BlockingMutex::new(Cell::new(heapless::Vec::new()));

/// The stashed `[MSG:RESET <reason>]` boot line, held for a one-shot replay on the first `$I`/status after a host
/// connects (DIAGNOSTIC `capture-reset` build only). It is SEPARATE from [`CRASH_REPORT`] on purpose: the reset-reason
/// line is emitted on EVERY boot (it is the only field naming WHICH watchdog fired — `*-rtc-WDT` = RWDT vs `super-WDT`
/// = SuperWDT vs `*-sw-reset` — the §17.8 "was the RWDT suppressed?" answer), whereas the crash report is gated on a
/// valid breadcrumb. Folding it into `CRASH_REPORT` would be clobbered by `maybe_emit_crash_report`'s `set()` (called
/// AFTER `send_reset_reason` in `main`) and would not replay on a no-breadcrumb reset. Without this replay the live
/// boot-time emit is missed whenever the host streams from byte 0 (the §17.13 capture gap). A single `Response` behind
/// the cross-core blocking mutex, drained at the same `$I`/`?` sites as `CRASH_REPORT`. `None` once replayed/empty.
#[cfg(feature = "capture-reset")]
static RESET_REPORT: BlockingMutex<CriticalSectionRawMutex, Cell<Option<Response>>> =
  BlockingMutex::new(Cell::new(None));

/// Format the previous run's crash breadcrumb into grbl `[MSG:CRASH ...]` line(s), emit them ONCE over the normal
/// TX path right after the boot banner, AND stash them for one replay on the first `$I`/status after connect.
/// Called from `main` after [`send_banner`], with the breadcrumb read from RTC_FAST and whether the reset was a
/// watchdog/fault reset (a clean power-on / brown-out clears RTC_FAST anyway, so a valid breadcrumb after one of
/// those would be impossible — but we still gate on the reset reason for clarity and defence in depth). When an RMT
/// hang was captured, a SECOND line carries the channel-0 register detail (kept separate so neither line exceeds
/// [`RESPONSE_CAPACITY`]).
///
/// The breadcrumb survives a WATCHDOG reset, NOT a power-cycle (see [`crate::crash`]): the operator must let the
/// dog bite (~8 s) and must not yank power, or the breadcrumb is lost. A no-op when the breadcrumb is invalid
/// (clean boot, or the crumb was already consumed) — nothing is emitted or stashed.
pub async fn maybe_emit_crash_report(breadcrumb: &crate::crash::Breadcrumb, reset_was_watchdog: bool) {
  if !breadcrumb.is_valid() || !reset_was_watchdog {
    return;
  }
  let Some(summary) = format_crash_report(breadcrumb) else {
    return;
  };
  // Build the line set. The PANIC line (an independent class — a panic / CPU fault / core-1 stack overflow captured
  // by the custom `#[panic_handler]`) goes FIRST when present, as it is the most decisive datum. Then the summary,
  // and (when present) the RMT register-detail and the per-task comms-stage breakdown — each its own line so none
  // exceeds RESPONSE_CAPACITY.
  let mut lines: heapless::Vec<Response, 6> = heapless::Vec::new();
  if let Some(panic) = breadcrumb.panic.as_ref()
    && let Some(panic_line) = format_panic_report(panic)
  {
    let _ = lines.push(panic_line);
  }
  let _ = lines.push(summary);
  if let Some(hang) = breadcrumb.rmt_hang.as_ref()
    && let Some(rmt_line) = format_rmt_hang_report(hang)
  {
    let _ = lines.push(rmt_line);
  }
  // The USB-TX-stall discriminator line (the §11 drumbeat capture): present only when `usb_tx`'s bounded escape
  // fired this run. Its verdict + the rmt-wait-count POSITIVELY classify the drumbeat (USB-TX vs RMT vs host-side).
  if let Some(stall) = breadcrumb.usb_tx_stall.as_ref()
    && let Some(usbtx_line) =
      format_usb_tx_stall_report(stall, breadcrumb.rmt_wait_count, breadcrumb.usb_tx_stall_len, breadcrumb.usb_tx_stall_window)
  {
    let _ = lines.push(usbtx_line);
  }
  if let Some(comms_line) = format_comms_stage_report(&breadcrumb.comms_stages) {
    let _ = lines.push(comms_line);
  }
  // Stash a copy for the first-`$I`/`?` replay (reconnect race), then emit now over the guaranteed-delivery path.
  CRASH_REPORT.lock(|c| c.set(lines.clone()));
  for line in lines {
    enqueue(line).await;
  }
}

/// Take the stashed crash report lines for a one-shot replay (consumes them so they are sent at most once more after
/// the boot emission). Returns an empty `Vec` once nothing is pending. Called from the `$I` build-info handler and
/// the status responder so a host that reconnected late after the watchdog reset still receives the `[MSG:CRASH ...]`
/// line(s).
pub(crate) fn take_pending_crash_report() -> heapless::Vec<Response, 6> {
  CRASH_REPORT.lock(|c| c.take())
}

/// Take the stashed `[MSG:RESET <reason>]` boot line for a one-shot replay (consumes it). Returns `None` once it has
/// been replayed or there was nothing to replay. Drained at the same `$I`/`?` sites as [`take_pending_crash_report`],
/// so a host that streamed from byte 0 (missing the live boot emit) still learns the reset reason / which dog fired on
/// its first request — the §17.13 capture gap fix. Capture build only.
#[cfg(feature = "capture-reset")]
pub(crate) fn take_pending_reset_report() -> Option<Response> {
  RESET_REPORT.lock(|c| c.take())
}

/// The single USB writer: drain the [`RESPONSE`] channel and write each formatted response to the USB
/// endpoint. Centralizing writes here means status reports, `ok`s, errors, and `$`-report lines never
/// interleave on the wire (DOC-08).
/// How long a single USB-Serial-JTAG write/flush may await the host draining the TX FIFO before the response is
/// abandoned. A reading host completes a write in well under a millisecond, so this only trips when the host has
/// stopped draining (disconnect / stalled read). Kept SHORTER than the watchdog's comms-stall window (~3 s) so a
/// non-draining host degrades usb_tx gracefully (a dropped response per timeout, `COMMS_PROGRESS` still advancing)
/// rather than wedging the comms path or tripping a watchdog reset; kept comfortably ABOVE normal write latency so
/// a healthy stream never drops a response (which would desync the host's character-count flow control).
const USB_TX_TIMEOUT: Duration = Duration::from_secs(2);

/// The USB-Serial-JTAG IN-endpoint FIFO / packet size (bytes). Responses are committed one `wr_done` packet per chunk
/// of this size, matching esp-hal's own `write`/`write_async` chunking and the EP1 FIFO depth.
const USB_TX_PACKET_BYTES: usize = 64;

/// Write one response over the POLL-based USB-TX path — the §18/§19 ROOT prevention of the esp-hal lost-TX-wake. Each
/// byte is pushed with [`UsbSerialJtagTx::write_byte_nb`] (which writes IFF the IN FIFO has room), and every ≤64 B
/// packet is committed with [`UsbSerialJtagTx::flush_tx_nb`] (`wr_done`). On a full FIFO (`WouldBlock` — the host has
/// not drained the previous packet yet) we [`yield_now`] and re-poll. This NEVER awaits esp-hal's
/// `UsbSerialJtagWriteFuture`, so there is NO completion waker to lose — the lost-wake CLASS is eliminated at the
/// source (see `docs/streaming-lockup-investigation.md` §18). The per-poll decision is the host-tested
/// [`firmware_core::diag::usb_tx_poll_action`].
///
/// The stall deadline is PROGRESS-RESET (Fix #1, medium review — supersedes the per-chunk budget of the original
/// Fix #2): ONE deadline spans the whole response and is re-armed from a fresh `now` on every observed progress event
/// (a byte pushed OR a commit registered), so it measures only CONTINUOUS idle. It trips after [`USB_TX_TIMEOUT`] of
/// UNBROKEN no-progress, which bounds the total stall-detection latency at [`USB_TX_TIMEOUT`] regardless of how many
/// ≤64 B chunks the response spans (the per-chunk budget let a degraded-but-still-draining host that dripped one packet
/// per timeout drag detection out to nchunks × [`USB_TX_TIMEOUT`]). The §13 lazy clock is preserved intact: progress
/// CLEARS the deadline without reading the clock (the happy path still pays for no timestamp), and `Instant::now()` is
/// read only on a no-progress poll — the exact same clock-read cost as the per-chunk version, not the extra per-progress
/// read the deferral note anticipated. A `WouldBlock` that persists past that deadline is a genuine host-not-reading
/// stall → [`WriteOutcome::Stalled`]
/// (drop-and-continue + the K-escape). Flow-control integrity: the flow-control-critical responses (`ok` / `error:N`)
/// are ≤64 B = a SINGLE packet, so a stall either sends the whole line or none of it — they are never truncated (the
/// first `write_byte_nb` blocks on a full FIFO before any byte is committed). A mid-response stall on a MULTI-chunk
/// response (a long `<...>` status or a `$`-report) can leave the already-committed packets on the wire (a truncated
/// tail) — but those are not part of the character-count flow control (status is re-requested on the next `?`), and
/// this matches the prior await path's behavior exactly (a `write_all` timeout also stranded mid-response), so it is
/// no regression. esp-hal's write/flush error type is `Infallible`, so an `Err` is always `WouldBlock`. usb_tx runs on
/// the core-0 thread-mode executor (it yields), so `Instant` advances here.
async fn write_response_polled(
  tx: &mut UsbSerialJtagTx<'static, Async>,
  bytes: &[u8],
) -> firmware_core::diag::WriteOutcome {
  use firmware_core::diag::{PollAction, WriteOutcome};
  // Fix #5/#1: the per-poll decision shared by the byte-push and commit loops below. On progress CLEAR the deadline and
  // take `Advance` DIRECTLY (no clock read — the §13 lazy-clock: the happy path never pays for a timestamp); otherwise
  // arm-or-check the shared `deadline` via the host-tested `usb_tx_poll_action`. `deadline` is `None` until the first
  // no-progress poll arms it from a fresh `now`, and every progress event resets it to `None` — so it measures only
  // CONTINUOUS idle and re-arms lazily. A nested fn so the two loops are byte-for-byte identical and the stall-vs-yield
  // policy has one source. Self-contained `use` so it does not depend on the enclosing imports.
  fn step(progressed: bool, deadline: &mut Option<Instant>) -> PollAction {
    use firmware_core::diag::{PollAction, usb_tx_poll_action};
    if progressed {
      *deadline = None;
      PollAction::Advance
    } else {
      // ONE clock read per no-progress poll (same cost as the per-chunk version): reused to both arm the deadline on
      // its first firing and to test it thereafter, so `now >= d` cannot drift from the value that was inserted.
      let now = Instant::now();
      let d = *deadline.get_or_insert(now + USB_TX_TIMEOUT);
      usb_tx_poll_action(false, now >= d)
    }
  }
  // Fix #1: ONE progress-reset deadline for the WHOLE response (spanning every chunk), armed lazily on the first
  // stalled poll and cleared by `step` on each observed progress event. This bounds total stall-detection latency at
  // `USB_TX_TIMEOUT` of unbroken no-progress instead of the per-chunk budget's nchunks × `USB_TX_TIMEOUT` worst case;
  // single-chunk flow-control responses (`ok`/`error:N`, ≤64 B) still trip at exactly `USB_TX_TIMEOUT` of no drain.
  let mut deadline: Option<Instant> = None;
  for chunk in bytes.chunks(USB_TX_PACKET_BYTES) {
    // Fill the FIFO with this chunk's bytes. `write_byte_nb` writes IFF `serial_in_ep_data_free` (room); a `WouldBlock`
    // means the FIFO is full because the host has not drained the previous packet yet — yield and re-poll.
    for &byte in chunk {
      loop {
        match step(tx.write_byte_nb(byte).is_ok(), &mut deadline) {
          PollAction::Advance => break,
          PollAction::Stall => return WriteOutcome::Stalled,
          PollAction::Yield => yield_now().await,
        }
      }
    }
    // Commit this ≤64 B packet, then WAIT for the commit to register before touching the FIFO again — mirroring
    // esp-hal's OWN blocking `flush_tx`/`write` exactly (set `wr_done` ONCE, then poll `ep1_conf`'s low 2 bits until
    // `!= 0`), but yielding instead of busy-waiting. `flush_tx_nb` sets `wr_done` and returns `Ok` when the commit is
    // already registered (`ep1_conf & 0b011 != 0`); on `WouldBlock` we poll the SAME register condition directly —
    // NOT re-calling `flush_tx_nb`, so `wr_done` is never re-triggered (no spurious zero-length packet). Waiting here
    // (rather than relying on the next `write_byte_nb`) avoids any commit-in-progress race on a partial final chunk and
    // keeps the byte stream faithful to esp-hal's tested sequencing.
    if tx.flush_tx_nb().is_err() {
      loop {
        // The commit poll shares the response-wide progress-reset deadline (Fix #1): a commit that registers clears it
        // and advances; `step` reads the clock only while still uncommitted, and any earlier progress already reset it.
        let committed = (esp_hal::peripherals::USB_DEVICE::regs().ep1_conf().read().bits() & 0b011) != 0;
        match step(committed, &mut deadline) {
          PollAction::Advance => break,
          PollAction::Stall => return WriteOutcome::Stalled,
          PollAction::Yield => yield_now().await,
        }
      }
    }
  }
  WriteOutcome::Completed
}

#[embassy_executor::task]
pub async fn usb_tx(mut tx: UsbSerialJtagTx<'static, Async>) -> ! {
  // The bounded consecutive-stall escape. A `Completed` write resets it; a `Stalled` increments it; at
  // `USB_TX_STALL_ESCAPE_K` (= 3) it drives the wedge handler (production `ALARM:17`, diagnostic capture+reset). Since
  // the §18/§19 poll-based write eliminated the lost-TX-wake CLASS, a `Stalled` now means only ONE thing — the host
  // did not drain the FIFO within the 2 s deadline (a genuine host-not-reading / disconnected-link stall), so K
  // consecutive of them is a real non-draining host, never a phantom lost wake. The counter lives across loop turns.
  let mut stall = firmware_core::diag::UsbTxStallCounter::new();
  // The WINDOWED stall counter (§13.8) for the survivable-watchdog capture: distinguishes a PURE consecutive stall run
  // (which the K-escape catches) from an ALTERNATING recovered/stall pattern that resets the consecutive counter on
  // every recovery yet still represents a degraded link. Published every loop turn to `USB_TX_STALL_WINDOW_COUNT` so
  // the TIMG1 ISR can copy it into the breadcrumb on a withhold. `capture-reset`-gated (the only consumer is the ISR).
  #[cfg(feature = "capture-reset")]
  let mut stall_window = firmware_core::diag::WindowedStallCounter::new();
  loop {
    crate::crash::record_comms_stage(crate::crash::CommsTask::UsbTx, crate::crash::CommsStage::TxWaitResponse);
    let resp = RESPONSE.receive().await;
    // Sample the core-1 motion beat before the write so the capture-reset fingerprint can report whether core 1
    // ADVANCED across a stall window (still scheduling). A single relaxed load; compared after the write resolves.
    let motion_before = MOTION_LIVENESS.load(Ordering::Relaxed);
    crate::crash::record_comms_stage(crate::crash::CommsTask::UsbTx, crate::crash::CommsStage::TxWrite);
    // POLL-BASED write (§18/§19 ROOT prevention): `write_response_polled` pushes the response via `write_byte_nb` /
    // `flush_tx_nb`, yielding on a full FIFO — it NEVER awaits esp-hal's `UsbSerialJtagWriteFuture`, so there is NO
    // completion waker to lose. That eliminates the lost-TX-wake CLASS at the source, so the entire TIER-1
    // classify/recover apparatus (and its `CompletedLostWakeRecovered` outcome) is gone. The result is now binary:
    // `Completed` (all bytes handed to the FIFO) or `Stalled` (the host did not drain within the 2 s deadline — a
    // genuine host-not-reading stall that drops-and-continues and feeds the K-escape below).
    let outcome = write_response_polled(&mut tx, resp.as_bytes()).await;
    // Publish the survivable-watchdog fingerprint (capture-at-withhold, §17.10/§17.11). The TIMG1 ISR feeds the dogs
    // and, on a stall-driven withhold, captures the breadcrumb — but it cannot cheaply read the USB_DEVICE registers
    // from interrupt context. So here, where the discriminator is already in hand, publish (1) the WINDOWED stall
    // count (every loop turn, so it ages correctly) and (2) on a genuine stall, the packed `UsbTxStall` snapshot +
    // response length. The ISR copies these straight into the breadcrumb, so even a hard Signature-B lock that never
    // reaches the K-escape leaves the LAST-KNOWN usb_tx fingerprint. Gated off the diagnostic capture build.
    #[cfg(feature = "capture-reset")]
    {
      let window = stall_window.record(outcome.is_stall());
      USB_TX_STALL_WINDOW_COUNT.store(window as u32, Ordering::Relaxed);
      if outcome.is_stall() {
        // A genuine stall this turn: snapshot the firmware-only USB-TX discriminator (same signals
        // `capture_usb_tx_stall_and_reset` reads) and publish the packed word + the stalled response's length.
        let usb = esp_hal::peripherals::USB_DEVICE::regs();
        let int_raw = usb.int_raw().read();
        let int_ena = usb.int_ena().read();
        let snap = firmware_core::diag::UsbTxStall {
          data_free: usb.ep1_conf().read().serial_in_ep_data_free().bit_is_set(),
          serial_in_empty: int_raw.serial_in_empty().bit_is_set(),
          int_ena_armed: int_ena.serial_in_empty().bit_is_set(),
          motion_advancing: MOTION_LIVENESS.load(Ordering::Relaxed) != motion_before,
          executor_running: EXECUTOR_RUNNING.load(Ordering::Acquire),
          response_depth: RESPONSE.len().min(u8::MAX as usize) as u8,
          timeout_count: stall.count().saturating_add(1),
        };
        USB_TX_STALL_FINGERPRINT.store(firmware_core::diag::pack_usb_tx_stall(&snap), Ordering::Relaxed);
        USB_TX_STALL_FINGERPRINT_LEN.store(resp.len().min(u16::MAX as usize) as u32, Ordering::Relaxed);
      }
    }
    // Comms-progress heartbeat (the §11.6 watchdog-mask fix): bump ONLY on a non-stall outcome (a completed write
    // delivered bytes to the host). A GENUINE stall no longer advances the counter, so
    // a stalled writer stops masking the 3 s comms-stall watchdog detector (the prior per-loop bump at the 2 s
    // cadence kept the dog fed and let the wedge limp ~16 s). The other two bumpers (status_responder, comms_consumer)
    // are unchanged, so a back-pressured-but-answering board still advances the counter.
    if !outcome.is_stall() {
      COMMS_PROGRESS.fetch_add(1, Ordering::Relaxed);
      // usb_tx-SPECIFIC completed-write beat for the dead-zone watchdog backstop (Signature B). A completed/recovered
      // write delivered bytes; the backstop watches THIS counter (not COMMS_PROGRESS, which the status reporter +
      // consumer also bump and which recovered-lost-wakes can keep alive) so it sees the WRITER specifically stop.
      USB_TX_COMPLETED.fetch_add(1, Ordering::Relaxed);
    }
    if stall.record(outcome.is_stall()) {
      // K consecutive GENUINE stalls (timeout + FIFO still full = host truly not reading / peripheral stuck): the
      // lost-wake path now recovers above (TIER 1), so this K-escape is the residual backstop for a REAL
      // host-not-draining wedge (or a PARTIAL fix where a wake still slips through). The TIER 2/3 split
      // (`capture-reset`, §17) decides what happens here:
      // - DIAGNOSTIC build (`--features capture-reset`): capture the discriminator + `software_reset()` so the next
      //   boot emits `[MSG:CRASH usbtx: ...]` (RTC_FAST survives the CoreSw reset); `resp.len()` is the
      //   single-chunk-widening discriminator. This is the open-investigation capture channel (#20/#21).
      // - PRODUCTION build (default): raise the LOCKED `ALARM:17` (MotorFault) — feed-hold + require re-home — and
      //   reset the local stall run so usb_tx keeps serving the alarm/banner traffic. NEVER a silent reset the host
      //   streams through (which would resume cutting in the wrong place, §14.3).
      // DOG-ISOLATION (`force-withhold`, §17.16): SKIP the usb_tx K-escape `software_reset()` so the ONLY thing that
      // can reset the board is the survivable-watchdog ISR's WITHHELD dog. In the first force-withhold run the usb_tx
      // K-escape (`CoreSw`, `n=3 host-not-reading`) software-reset the board and MASKED whether a dog bites at all —
      // this build removes that confound, so a self-reset can ONLY be a dog (read `[MSG:RESET super-WDT|...rtc-WDT]`).
      #[cfg(not(feature = "force-withhold"))]
      handle_usb_tx_wedge(motion_before, stall.count(), resp.len());
      // Production (non-capture) AND the force-withhold dog-isolation build both fall through here (in force-withhold
      // `handle_usb_tx_wedge` is not called, so usb_tx must keep serving); clear the stall run so a single residual
      // wedge does not immediately re-trip. Only the plain `capture-reset` build diverges inside the helper (reset).
      #[cfg(any(not(feature = "capture-reset"), feature = "force-withhold"))]
      {
        stall = firmware_core::diag::UsbTxStallCounter::new();
      }
    }
  }
}

/// Handle a K-consecutive-stall `usb_tx` wedge per the TIER 2/3 split (docs/streaming-lockup-investigation.md §17).
///
/// In the DIAGNOSTIC `capture-reset` build this captures the firmware-only discriminator into the RTC_FAST
/// breadcrumb and forces a `software_reset()` (never returns — the open Signature-A/B capture channel). In the
/// PRODUCTION default build it raises the LOCKED [`AlarmCode::MotorFault`] (`ALARM:17`) via [`MOTION_FAULT`] — the
/// grbl lost-step-sync contract: feed-hold + require re-home, NEVER a silent reset the host streams through — and
/// RETURNS so usb_tx keeps serving the alarm + banner traffic to the host.
// Not compiled under `force-withhold` (§17.16 dog-isolation): there the K-escape call site is cfg'd out so usb_tx
// never software-resets, leaving the ISR's withheld dog as the sole resetter.
#[cfg(all(feature = "capture-reset", not(feature = "force-withhold")))]
fn handle_usb_tx_wedge(motion_before: u32, timeout_count: u16, response_len: usize) -> ! {
  capture_usb_tx_stall_and_reset(motion_before, timeout_count, response_len);
}

/// Production variant: raise the motion-fault alarm instead of resetting. See the `capture-reset` variant above.
#[cfg(not(feature = "capture-reset"))]
fn handle_usb_tx_wedge(_motion_before: u32, _timeout_count: u16, _response_len: usize) {
  // A residual unrecoverable USB-TX wedge: the host is not draining (or a lost wake slipped past TIER 1). Halt the
  // program into the LOCKED `ALARM:17` and force a re-home — the executor's motion-fault path and this share the one
  // alarm. The consumer (the planner owner) services `MOTION_FAULT`, flushes the queue, and emits the alarm; usb_tx
  // returns and keeps writing so the alarm line + any subsequent banner reach the host.
  MOTION_FAULT.signal(());
}

/// Capture the USB-TX-stall discriminator at the K-th consecutive `usb_tx` timeout, then force a software reset so
/// the next boot emits the `[MSG:CRASH usbtx: ...]` line over CDC. DIAGNOSTIC-only (`capture-reset` build, §17): in
/// the production default the wedge raises `ALARM:17` instead (see [`handle_usb_tx_wedge`]). Reads the firmware-only
/// signals that split the §11.4 hypotheses without any RTT (which is transport-blocked on this board — the defmt sink
/// shares the one USB-Serial-JTAG with the grbl CDC):
/// - `ep1_conf.serial_in_ep_data_free` — `false` ⇒ the EP1 IN FIFO is full because the HOST is not draining.
/// - `int_raw.serial_in_empty` — the raw TX-empty event the async write future waits on (asserted ⇒ the event
///   fired); paired with `int_ena.serial_in_empty` (still ARMED?) to tell a lost-waker from a never-serviced ISR.
/// - the core-1 [`MOTION_LIVENESS`] delta vs `motion_before` (did core 1 advance during the stall window?) plus
///   [`EXECUTOR_RUNNING`] (was a block in flight?) — the H-A-vs-H-B core-1-health test.
/// - the `RESPONSE` channel depth — confirms the writer is the head-of-line bottleneck.
///
/// The register reads are plain, side-effect-free volatile loads of the USB_DEVICE block (no lock, safe from this
/// task), mirroring `motion.rs`'s `capture_rmt_hang`. `software_reset()` is `CoreSw`, which preserves RTC_FAST.
/// `-> !`: this never returns (it resets the chip). Not compiled under `force-withhold` (dog-isolation, §17.16).
#[cfg(all(feature = "capture-reset", not(feature = "force-withhold")))]
fn capture_usb_tx_stall_and_reset(motion_before: u32, timeout_count: u16, response_len: usize) -> ! {
  let usb = esp_hal::peripherals::USB_DEVICE::regs();
  let ep1 = usb.ep1_conf().read();
  let int_raw = usb.int_raw().read();
  let int_ena = usb.int_ena().read();
  let stall = firmware_core::diag::UsbTxStall {
    data_free: ep1.serial_in_ep_data_free().bit_is_set(),
    serial_in_empty: int_raw.serial_in_empty().bit_is_set(),
    int_ena_armed: int_ena.serial_in_empty().bit_is_set(),
    // Core 1 advanced across the stall window ⇒ still scheduling (favors a lost USB wake, not a core-1 wedge).
    motion_advancing: MOTION_LIVENESS.load(Ordering::Relaxed) != motion_before,
    executor_running: EXECUTOR_RUNNING.load(Ordering::Acquire),
    // The depth-8 RESPONSE channel's current occupancy; clamps into the nibble in the packer.
    response_depth: RESPONSE.len().min(u8::MAX as usize) as u8,
    timeout_count,
  };
  crate::crash::record_usb_tx_stall(
    firmware_core::diag::pack_usb_tx_stall(&stall),
    response_len.min(u16::MAX as usize) as u16,
  );
  esp_hal::system::software_reset();
}

/// Queue a single `ok`.
pub(crate) async fn ack() {
  let mut s = Response::new();
  if ResponseWriter::ok(&mut s).is_ok() {
    // OBSERVE-ONLY probe (task #22): a terminal per-line `ok` — the "ACK" leg of the lines/acks/execs cross-check.
    // Counted on FORMAT (not on the USB write) so it tracks the consumer's one-response-per-line decision, the same
    // event the host's flow control counts. `error_bare` counts the `error:N` terminal; together they are every ack.
    ACKS_EMITTED.fetch_add(1, Ordering::Relaxed);
    // Mark the consumer's response enqueue: `enqueue` is `RESPONSE.send().await`, which BLOCKS when the channel is
    // full — i.e. `usb_tx` is behind/stuck. A wedge here (with `usb_tx`'s slot at `tx-write`) is the classic
    // "output path stalled, consumer can't ack" chain.
    crate::crash::record_comms_stage(crate::crash::CommsTask::Consumer, crate::crash::CommsStage::ConsumerEnqueue);
    enqueue(s).await;
  }
}

/// Queue a single `error:N` for a rejected line. Mirrors [`ack`]; the consumer emits exactly one of the
/// two per consumed GCode line, preserving the one-response-per-line contract. A `[MSG:error:N <name>]`
/// context push is emitted FIRST so a plain terminal sees what the rejection means; it is a push message
/// (a sender that decodes the code itself ignores it) and the byte-exact `error:N` remains the sole
/// flow-control response. The context is skipped for a code with no enumerated name (empty buffer).
///
/// Use this only when the code's NAME genuinely describes the failure (parser/planner/validation errors,
/// and the `ERROR_LOCKED` state lockout). For a code reused purely to halt the sender — where its name does
/// NOT describe the cause — use [`error_bare`] so the annotation does not mislead.
pub(crate) async fn error(code: u8) {
  let mut ctx = Response::new();
  if ResponseWriter::error_context(&mut ctx, code).is_ok() && !ctx.is_empty() {
    enqueue(ctx).await;
  }
  error_bare(code).await;
}

/// Queue a single `error:N` WITHOUT the `[MSG:error:N <name>]` context push. Used for the post-error hold
/// rejection, whose code (`ERROR_HOLD_CODE` = 1) is reused purely to halt the sender: its name ("Expected
/// command letter") does NOT describe why the held line was rejected, so annotating it would mislead a plain
/// terminal. The byte-exact `error:N` is still emitted as the sole flow-control response.
pub(crate) async fn error_bare(code: u8) {
  let mut s = Response::new();
  if ResponseWriter::error(&mut s, code).is_ok() {
    // OBSERVE-ONLY probe (task #22): a terminal per-line `error:N` — the other half of the "ACK" leg (with `ack`).
    // `error()` delegates to `error_bare`, so counting here covers BOTH the annotated and bare error paths without
    // double-counting. The `[MSG:error:N ..]` context push is NOT counted — it is not a flow-control response.
    ACKS_EMITTED.fetch_add(1, Ordering::Relaxed);
    // Same `RESPONSE.send().await` enqueue chokepoint as `ack` — marks the consumer blocked emitting an error.
    crate::crash::record_comms_stage(crate::crash::CommsTask::Consumer, crate::crash::CommsStage::ConsumerEnqueue);
    enqueue(s).await;
  }
}
