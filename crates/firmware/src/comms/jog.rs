//! `$J=` jogging + the shared executor-park primitive (DOC-08 Phase D): the core-0 jog gating that parses a jog in
//! the parser's throwaway modal context, plans it into a cancelable jog block, latches [`ControlState::Jog`], and
//! wakes the executor — plus [`quiesce_executor`]/[`release_hold`] (the "park the executor at a block boundary with a
//! real acknowledgment, then release" pair) and the `0x85` jog-cancel that composes them. Extracted verbatim from
//! `comms.rs` (architecture-refactor A1, step 8). `handle_jog`/`refresh_jog_state`/`cancel_jog_cycle` are called from
//! the consumer, and `quiesce_executor`/`release_hold`/[`QuiesceOutcome`] from `program_stop_cycle` (Step 10), so all
//! are `pub(crate)`; `comms.rs` re-exports this module (`pub(crate) use jog::*;`) so those calls keep resolving.
//! `use super::*` supplies the parent surface (the state statics/signals, `control_state`/`set_control_state`,
//! `program_running`/`current_soft_limits`, `read_live_position`, `ack`/`error`/`error_bare`, and the firmware_core
//! types); the child glob needs no explicit imports.

use super::*;

/// Handle one `$J=` jog line (DOC-08 Phase D), emitting exactly one `ok`/`error:N`.
///
/// Gating (grbl): a jog is honored only when (a) no GCode error-hold is active, and (b) the machine is Idle or
/// already jogging — it is rejected while running a PROGRAM, in a feed-hold, alarm, check, or sleep state. The
/// "running a program" case is `Normal` control state with a non-jog block in flight; we reject it so a jog never
/// blends into program motion. The jog is parsed in the parser's throwaway modal context (it never mutates
/// `gc_state`), planned into a cancelable jog block (rejected up front if `$20` soft limits would be exceeded),
/// the machine is latched into `Jog`, and the executor is woken — exactly one `ok` on success.
pub(crate) async fn handle_jog(line: &[u8], parser: &mut Parser, state: &mut ConsumerState) {
  // Drop a stale jog latch first if the previous jog has fully drained, so the gate below sees the true state.
  refresh_jog_state().await;
  if state.error_hold {
    // Held by a prior GCode error: reject until a recovery trigger, like any motion line. Bare (no `[MSG:..]`
    // annotation): the hold code's name does not describe why this held line was rejected (see `error_bare`).
    error_bare(ERROR_HOLD_CODE).await;
    return;
  }
  let control = control_state();
  // Reject a jog from a non-Idle/Jog mode (hold/alarm/check/sleep), or from Normal while a PROGRAM block is in
  // flight (Run). grbl returns the generic "command requires the machine to be idle" rejection; reuse the
  // unsupported-command code so a sender halts, matching the alarm/hold rejections elsewhere.
  if !control.jog_allowed() || (control == ControlState::Normal && program_running().await) {
    error(ERROR_UNSUPPORTED_COMMAND).await;
    return;
  }
  let jog = match parser.parse_jog(line) {
    Ok(jog) => jog,
    Err(e) => {
      // A jog parse error (missing F → 22, missing axis → 23, unsupported word → 20, lexer 1/2 for a malformed
      // word/number) is reported, but a jog does NOT arm the gcode error-hold — it is independent of the program
      // stream (grbl keeps streaming after a rejected jog).
      error(e.code()).await;
      return;
    }
  };
  let limits = current_soft_limits().await;
  let outcome = {
    let mut guard = PLANNER.lock().await;
    match guard.as_mut() {
      Some(planner) => planner.plan_jog(&jog, limits),
      None => Err(PlannerError::JogExceedsTravel), // Unreachable in a wired build; fail loudly rather than fake-ack.
    }
  };
  match outcome {
    Ok(PlannerOutcome::Queued { blocks }) => {
      // Latch Jog and wake the executor so the jog block runs. A zero-block jog (target == current position) is
      // a valid no-op: it still `ok`s, but needs no executor wake and leaves the state Idle (no block in flight).
      if blocks > 0 {
        set_control_state(control.begin_jog());
        BLOCK_AVAILABLE.signal(());
      }
      ack().await;
    }
    // A soft-limit rejection (`$20` on): the jog is ignored and the host sees `error:15` (travel exceeded). No
    // block was enqueued and the control state is untouched.
    Err(e) => error(e.code()).await,
    // `plan_jog` only ever returns `Queued`/`JogExceedsTravel`; any other outcome is an internal invariant break.
    Ok(_) => ack().await,
  }
}

/// Drop a stale `Jog` latch back to `Normal` once the jog has fully drained (no jog block queued or in flight),
/// so a subsequent program GCode line is not blocked and `?` reports `Idle`. Called at the top of the jog and
/// GCode plan paths. The reported state already reads `Idle` while latched-`Jog`-but-quiescent (machine_state
/// derives Idle from `running == false`), so this only keeps the LATCH honest for the gcode/jog gates.
pub(crate) async fn refresh_jog_state() {
  if control_state() != ControlState::Jog {
    return;
  }
  if program_running().await {
    return; // Jog blocks still in flight — stay in Jog.
  }
  set_control_state(ControlState::Jog.cancel_jog());
}

/// The outcome of [`quiesce_executor`]: whether the executor reached a parked rest, or a soft reset preempted
/// the quiesce (in which case the caller must abandon its boundary-stop sequence and honor the reset).
pub(crate) enum QuiesceOutcome {
  /// The executor has come to rest at a block boundary on the hold level (a real acknowledgment, not a poll):
  /// no block is in flight and the live position is stable, so the caller may now sync the planner to it.
  Parked,
  /// A `0x18` soft reset landed during the quiesce; the consumed reset signal must be honored by the caller.
  ResetPreempted,
}

/// Stop the core-1 executor at the next block boundary and WAIT until it has actually quiesced, returning a real
/// acknowledgment (Finding #11 / #3). This is the ONE shared "park the executor and confirm it parked" primitive
/// reused by jog-cancel and probe-abort, closing the race the old code had between "executor cleared
/// `EXECUTOR_RUNNING`" and "executor has actually parked" (the old jog-cancel polled `EXECUTOR_RUNNING` then
/// blind-signalled a `CYCLE_START` that could be drained as stale).
///
/// It RAISES the hold LEVEL ([`HOLD_REQUESTED`]) — the authoritative source of truth the executor honors at
/// every boundary and in its empty-queue wait — wakes the executor, and then awaits the executor's
/// [`MOTION_PARKED`] acknowledgment, which the executor pulses exactly when it parks on the level. If the
/// executor was ALREADY idle/parked (no block in flight) it still re-evaluates the level on the `HOLD_WAKE`
/// nudge and pulses `MOTION_PARKED`, so this resolves promptly in the common already-stopped case too. The wait
/// is raced against a soft reset so a `0x18` mid-quiesce is honored rather than hanging. The caller is
/// responsible for RELEASING the hold (clearing [`HOLD_REQUESTED`] + waking) once it has finished its
/// position-sync — see [`release_hold`].
pub(crate) async fn quiesce_executor() -> QuiesceOutcome {
  // Drain any stale park acknowledgment so we wait on THIS quiesce's park, not a previous one's.
  MOTION_PARKED.try_take();
  HOLD_REQUESTED.store(true, Ordering::Release);
  HOLD_WAKE.signal(());
  crate::crash::record_comms_stage(crate::crash::CommsTask::Consumer, crate::crash::CommsStage::ConsumerSyncWait);
  match select(MOTION_PARKED.wait(), SOFT_RESET.wait()).await {
    Either::First(()) => QuiesceOutcome::Parked,
    // A soft reset preempted the quiesce: re-signal it so `comms_consumer` runs its reset, and report the
    // preemption so the caller abandons its boundary-stop (the reset clears the hold level, flushes the planner,
    // and zeroes the position, superseding whatever the caller was about to sync). The reset's own
    // `HOLD_REQUESTED` clear releases the executor, so the caller must NOT also release the hold.
    Either::Second(()) => {
      SOFT_RESET.signal(());
      QuiesceOutcome::ResetPreempted
    }
  }
}

/// Release a hold raised by [`quiesce_executor`]: clear the hold LEVEL and wake the executor so it leaves its
/// parked branch (returning to await the next block — its queue is empty after the caller's flush). Clearing
/// the LEVEL (not signalling an edge) is what makes the release impossible to lose (Finding #11).
pub(crate) fn release_hold() {
  HOLD_REQUESTED.store(false, Ordering::Release);
  HOLD_WAKE.signal(());
}

/// Run a jog-cancel (`0x85`, DOC-08 Phase D) end to end, REUSING the shared [`quiesce_executor`] boundary stop. It
/// flushes the trailing queued jog blocks under the planner lock, parks the executor at the active jog block's
/// boundary WITH A REAL ACKNOWLEDGMENT (no `EXECUTOR_RUNNING` poll + blind cycle-start — Finding #3), syncs the
/// planner's commanded position to the actual live stop point (the Phase-C [`Planner::sync_position`] mechanism),
/// releases the hold, and returns the machine to `Normal`/Idle. A jog never changed modal/coordinate state, so no
/// reset side-effects are needed and no alarm is raised.
///
// TODO(DOC-02 Stage-2): mid-block jog-cancel ramp-down. We stop at the current block boundary (reusing the
// feed-hold path) rather than ramping the velocity down mid-block; a smooth mid-block deceleration is the Stage-2
// refinement, identical to the feed-hold smooth-ramp follow-up. The block-boundary approximation lives here.
pub(crate) async fn cancel_jog_cycle() {
  if control_state() != ControlState::Jog {
    return; // Lost the race to a soft reset / drain; nothing to cancel.
  }
  // 1. Flush the trailing queued jog blocks FIRST, under the planner lock, so the executor has nothing more to
  //    pop after it finishes the active block — the in-flight block stops at its boundary and no flushed-away
  //    block follows it.
  {
    let mut guard = PLANNER.lock().await;
    if let Some(planner) = guard.as_mut() {
      planner.flush_jog_blocks();
    }
  }
  // 2. Park the executor at the active jog block's boundary via the shared quiesce primitive, which raises the
  //    hold level and AWAITS a real parked acknowledgment — closing the race the old `EXECUTOR_RUNNING` poll had
  //    with the executor's boundary clear. A soft reset mid-quiesce is honored (the reset supersedes the cancel).
  match quiesce_executor().await {
    QuiesceOutcome::Parked => {}
    // The reset already cleared the hold level, flushed the planner, and zeroed the position; abandon the cancel
    // and let `comms_consumer` run the re-signalled reset. Do NOT release the hold — the reset already did.
    QuiesceOutcome::ResetPreempted => return,
  }
  // 3. Sync the planner's commanded position to the ACTUAL live stop point (grbl sets `gc_state.position` to the
  //    jog stop), so a subsequent move resolves from where the machine really stopped, not the jog target. The
  //    executor is genuinely parked now (the quiesce acknowledged it), so the live position is stable to read.
  let stop_steps = read_live_position();
  {
    let mut guard = PLANNER.lock().await;
    if let Some(planner) = guard.as_mut() {
      planner.sync_position(stop_steps);
    }
  }
  // 4. Release the hold so the executor leaves its parked branch — the queue is now empty, so it returns to
  //    awaiting the next block — and return to Normal/Idle with no alarm, no modal change.
  release_hold();
  set_control_state(ControlState::Jog.cancel_jog());
}
