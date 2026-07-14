//! Real-time command dispatch (DOC-08 reader half): `dispatch_realtime` maps a classified single-byte real-time
//! command (`?`/`!`/`~`/`0x18`/override/…) onto its Signal or immediate synchronous action, plus the
//! `try_send_banner` best-effort reset banner it uses — extracted verbatim from `comms.rs` (architecture-refactor
//! A1, step 5). Everything here is NON-BLOCKING so the `usb_rx` reader half (in `rx.rs`, which calls
//! `dispatch_realtime`) is never delayed behind line flow control. It reads/writes the shared control/override
//! state and fires the cross-task Signals, all re-exported from `comms.rs`; `comms.rs` re-exports this module
//! (`pub(crate) use realtime::*;`) so the `rx.rs` call keeps resolving unqualified.

use core::sync::atomic::Ordering;

use firmware_core::protocol::{ControlState, RealtimeCommand, ResponseWriter};

// `dispatch_realtime` touches a wide surface of parent real-time state statics + the control/override accessors,
// all re-exported by `comms.rs`; a glob keeps the relocated body verbatim. Parent-owned external types explicit above.
use super::*;

/// Emit the banner WITHOUT blocking, for the reader half's `0x18` handler: the reader must never block (it
/// has to stay free to dispatch the next real-time byte), so a momentarily full [`RESPONSE`] drops this
/// banner. A dropped reset-banner is harmless — the host re-probes readiness with `$I`/`?`, and the
/// consumer's pipeline reset also emits a banner through the guaranteed-delivery path — so the reset is
/// still observable. This is the one response emission that is allowed to drop, precisely because it is on
/// the real-time path.
fn try_send_banner() {
  let mut s = Response::new();
  if ResponseWriter::banner(&mut s).is_ok() {
    let _ = RESPONSE.try_send(s);
  }
}

/// Map a classified real-time command onto its Signal / immediate action — all NON-BLOCKING so the reader
/// half is never delayed. Status requests fire the reporter Signal; feed-hold / cycle-start fire theirs; a
/// soft reset / stop clears the byte buffer, flushes any framed-but-unconsumed lines, and signals both the
/// line assembler ([`LINE_RESET`], drop the partial line) and the consumer ([`SOFT_RESET`], reset the
/// parser/planner pipeline and re-emit the banner). The reset-banner is emitted here only on a best-effort
/// basis (`try_send`); the consumer's guaranteed banner is the authoritative one. Override and the
/// Stage-2/3 commands are accepted and currently ignored (documented stubs).
pub(crate) fn dispatch_realtime(cmd: RealtimeCommand) {
  match cmd {
    RealtimeCommand::StatusReport | RealtimeCommand::FullStatusReport => STATUS_REQUEST.signal(()),
    RealtimeCommand::FeedHold => {
      // Latch the feed-hold into the shared control state so the very next `?` reports `Hold:0`, then RAISE the
      // hold LEVEL (Finding #11) and wake the executor so it parks at the next block boundary. The level is the
      // authoritative source of truth — the executor re-reads it at every boundary AND in its empty-queue wait
      // — so a hold can never be missed or stranded. Both updates are synchronous (a `Cell` swap and an atomic
      // store) so the non-blocking reader half is not stalled. `feed_hold` is a no-op on the state outside a
      // running mode (alarm/check/sleep), so only raise the level when the state actually became a hold.
      let next = control_state().feed_hold();
      set_control_state(next);
      if matches!(next, ControlState::Hold(_)) {
        HOLD_REQUESTED.store(true, Ordering::Release);
        HOLD_WAKE.signal(());
      }
    }
    RealtimeCommand::CycleStart => {
      // Cycle-start (`~`/`0x81`): resume a feed-hold ONLY. Gate the executor-hold release on the host-tested
      // predicate [`ControlState::resumes_on_cycle_start`] so a `~` is INERT in Idle/Run/Jog/Alarm/Check AND in
      // Sleep — only a soft reset wakes a sleeping machine (Finding #1). Clearing the hold LEVEL (not signalling
      // an edge that could be drained as stale) is what makes a legitimate resume impossible to lose (Finding
      // #11). When the state is not a hold, leave the level untouched so a stray `~` never releases a `$SLP`
      // park or a not-yet-arrived hold.
      let current = control_state();
      if current.resumes_on_cycle_start() {
        set_control_state(current.cycle_start());
        HOLD_REQUESTED.store(false, Ordering::Release);
        HOLD_WAKE.signal(());
        // If an M0/M1/M6 program-flow pause is the thing holding, nudge it to leave its hold-await and resume the
        // stream (it owns clearing PAUSE_ACTIVE + acking the line). The executor release above is shared with a
        // plain feed-hold; this dedicated nudge wakes the consumer's pause wait without racing the executor wake.
        if PAUSE_ACTIVE.load(Ordering::Acquire) {
          PAUSE_RESUME.signal(());
        }
      }
    }
    RealtimeCommand::SoftReset | RealtimeCommand::Stop => handle_soft_reset(),
    RealtimeCommand::JogCancel => {
      // Jog cancel (`0x85`, Phase D): only meaningful while a jog is in flight — grbl ignores it otherwise. When
      // jogging, wake the consumer to run the block-boundary stop + jog-block flush + position-sync (it owns the
      // planner; this reader half stays non-blocking). The control-state check is a synchronous `Cell` load.
      if matches!(control_state(), ControlState::Jog) {
        JOG_CANCEL.signal(());
      }
    }
    RealtimeCommand::ProgramStop => {
      // Graceful program stop (`0x86`, Galdr extension): a controlled decelerate-to-Idle that flushes the program
      // and RETAINS position, distinct from the `0x18` abort. Only meaningful while a program is running or held —
      // the host-tested `program_stop_quiesces` gate is true exactly for `Normal`/`Hold`, so a `0x86` from Idle is
      // a (cheap) no-op there and from Alarm/Check/Sleep/Jog is ignored entirely (a jog has its own `0x85` cancel).
      // When the gate holds, wake the consumer (the planner owner) to run the boundary stop + full flush + sync;
      // this reader half stays non-blocking (a synchronous `Cell` load + a coalesced `Signal`).
      if control_state().program_stop_quiesces() {
        PROGRAM_STOP.signal(());
      }
    }
    RealtimeCommand::Override(byte) => handle_override(byte),
    RealtimeCommand::ToggleAutoReport => {
      // `0x8C` (Phase F): toggle the auto real-time report mode at runtime. A `Relaxed` flip of the suspend flag,
      // non-blocking on the real-time path; the auto-report task observes it on its next tick (or immediately
      // wakes from a disabled idle via AUTO_REPORT_WAKE). An override is never acked. With the `$481` interval at
      // 0 this still toggles the suspend flag, but the task stays silent until an interval is configured.
      let suspended = AUTO_REPORT_SUSPENDED.fetch_xor(true, Ordering::Relaxed);
      // `fetch_xor` returns the PRIOR value; if we just RESUMED (prior == true) wake the task so it re-arms the
      // interval timer promptly instead of waiting out a stale long sleep.
      if suspended {
        AUTO_REPORT_WAKE.signal(());
      }
    }
    RealtimeCommand::ToggleOptionalStop => {
      // `0x88`: flip the optional-stop switch that gates `M1`. A `Relaxed` flip, non-blocking on the real-time
      // path; the next `M1` pause consults it. Never acked (it is a real-time toggle, not a line). grblHAL leaves
      // this OFF by default, so until a host sends `0x88` an `M1` is a no-op `ok` (an `M0`-equivalent only when on).
      OPTIONAL_STOP_ENABLED.fetch_xor(true, Ordering::Relaxed);
    }
    // Parser-state-on-demand and safety door are accepted but not yet acted on (Stage 2/3). They correctly
    // produce no `ok`.
    RealtimeCommand::ParserStateReport | RealtimeCommand::SafetyDoor => {}
  }
}

/// Handle a soft reset / stop (`0x18` / `0x19`) real-time command — the SoftReset arm extracted verbatim from
/// [`dispatch_realtime`] (B4 extract-method): the SAME latch, RX/line flush, jog-cancel + probe drain, hold-level
/// clear, and Signal choreography in the SAME order. NON-BLOCKING (synchronous stores / `Signal`s / best-effort
/// banner), so the reader half is never delayed.
fn handle_soft_reset() {
  // Latch whether the executor was mid-cycle so the consumer's pipeline reset can apply grbl's rule (a
  // reset aborting an in-progress cycle -> ALARM:3). Read EXECUTOR_RUNNING here, before the executor can
  // clear it on its own MOTION_RESET wake, so the abort decision is race-free. The CONTROL state itself is
  // updated by the consumer's reset_pipeline (which can await), not here.
  RESET_WAS_RUNNING.store(EXECUTOR_RUNNING.load(Ordering::Acquire), Ordering::Relaxed);
  // Drop every buffered RX byte and every framed-but-unconsumed line so post-reset modal state is not
  // contaminated by anything that arrived before the reset. Dedicated Signals are set for each waiter so
  // the assembler drops its partial line, the consumer rebuilds the parser/planner and re-emits the
  // banner, and the core-1 motion executor aborts its block + zeroes the live position — one Signal per
  // waiter because an embassy `Signal` wakes only ONE task (Finding #3).
  RX_PIPE.clear();
  while LINE_QUEUE.try_receive().is_ok() {}
  // Drop any pending jog-cancel so a `0x85` that arrived just before the reset cannot fire AFTER it (the
  // reset already flushes the planner and zeroes the position, superseding any jog-cancel — Phase D).
  JOG_CANCEL.try_take();
  // Clear the hold LEVEL so NO stale hold survives the reset (Finding #2): a feed-hold latched while the
  // executor was idle, or a `$SLP` park, must not leave the executor parked after the warm reset (which
  // returns to Idle / boot-lock, never Hold). Drain any pending `G38.x` PROBE_REQUEST too (Finding #4) so
  // the executor cannot run an UNREQUESTED probe move from the freshly-zeroed origin after the reset — a
  // probe queued just before `0x18` is superseded by the reset, exactly like the flushed planner blocks.
  HOLD_REQUESTED.store(false, Ordering::Release);
  PROBE_REQUEST.try_take();
  // NOTE: no `HARD_LIMIT_TRIPPED` drain here anymore. The hard-limit alarm is now EDGE-armed in the executor
  // (`check_hard_limits` -> `hard_limit_alarm_armed`), and the executor re-seeds its per-axis arming to the
  // settled levels at every reset / post-homing boundary. A switch left ENGAGED after an aborted `$H` seek is
  // therefore a HELD level, not a fresh edge, so it never signals `HARD_LIMIT_TRIPPED` in the first place —
  // there is no stale latch to drain (this was the root of the `error:9` re-lock). A genuinely new over-travel
  // during later motion still alarms on its own fresh edge, and the consumer's `hard_limit_alarm_applies`
  // guard remains as the single defensive layer so a stray trip can never downgrade a more-specific alarm.
  LINE_RESET.signal(());
  SOFT_RESET.signal(());
  // Wake the executor (idle case) and raise the poll-able mid-block abort flag (running case). Set the
  // flag with `Release` before the `Signal` so the executor, on its wake, observes the flag set. The
  // `HOLD_WAKE` nudge releases the executor if it happened to be parked on the (now-cleared) hold level so
  // it proceeds to service the reset rather than waiting on a hold that no longer exists.
  MOTION_RESET_PENDING.store(true, core::sync::atomic::Ordering::Release);
  HOLD_WAKE.signal(());
  MOTION_RESET.signal(());
  // Best-effort immediate readiness banner; the consumer's reset emits the guaranteed one. Dropping
  // this (full RESPONSE) is harmless and keeps the reader non-blocking on the real-time path.
  try_send_banner();
}

/// Handle a feed / rapid / spindle / coolant override byte (Phase E, DOC-08 §3) — the Override arm extracted
/// verbatim from [`dispatch_realtime`] (B4 extract-method): the SAME no-op-skip, `OVERRIDES` swap, and conditional
/// spindle re-drive in the SAME order. NON-BLOCKING (a `Cell` swap + an optional coalesced `Signal`).
fn handle_override(byte: u8) {
  // A feed / rapid / spindle / coolant override byte (Phase E, DOC-08 §3). Mutate the shared OVERRIDES in the
  // non-blocking reader half — a `Cell` swap, no await — so the change shows in the very next `?` (`Ov:` and
  // the realized `FS:`). An override is a real-time command and is NEVER acked. grbl ignores a no-op override
  // (one that does not change the value), so we only act when something actually changed. The core-1 executor
  // re-reads OVERRIDES once per block, so a new scale takes effect on the NEXT block boundary — a block
  // already in flight finishes at its current scale (Stage-1 per-block granularity; mid-block re-scaling is
  // the smooth-ramp Stage-2 refinement). No executor wake is needed: the level is re-read at the boundary.
  let mut ov = overrides();
  let prev_spindle = (ov.spindle, ov.spindle_stop);
  if ov.apply(byte) {
    set_overrides(ov);
    // DOC-07: ONLY a spindle-override / spindle-stop change re-drives the LEDC duty. Wake the spindle task to
    // re-apply from the SAME commanded direction at the new override-scaled RPM (its `commanded_spindle`
    // reads the live `Ov:`), so a `0x9E` spindle-stop zeroes the duty and a `0x9A`/`0x9B`/`0x99` adjusts it.
    // A feed / rapid / coolant byte never affects the spindle, so it must NOT wake the task (that would
    // needlessly re-snapshot settings and re-drive the LEDC/GPIO for an unchanged duty). Coolant (`0xA0`/
    // `0xA1`, DOC-06) is still tracked + reportable, but the flood/mist GPIO are not wired.
    if (ov.spindle, ov.spindle_stop) != prev_spindle {
      SPINDLE_UPDATE.signal(());
    }
  }
}
