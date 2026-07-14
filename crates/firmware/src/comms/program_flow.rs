//! Program-flow control (DOC): the consumer-side machinery that pauses, ends, dwells, and gracefully stops a running
//! program — the `M0`/`M1`/`M6` pause ([`run_program_pause`]/[`hold_until_resume`], which still services read-only `$`
//! queries in place), the `M30` program end ([`program_end`]), the synchronized `G4`/spin-up dwell ([`run_dwell`] +
//! the [`wait_for_motion_idle`] buffer-synchronize), and the `0x86` graceful program stop ([`program_stop_cycle`],
//! which generalizes jog-cancel to a whole program). Extracted verbatim from `comms.rs` (architecture-refactor A1,
//! step 10). The entry points called from the consumer (`run_dwell`/`program_end`/`run_program_pause`/[`PauseOutcome`]
//! and `program_stop_cycle`) are `pub(crate)`; the pause/hold internals (`hold_until_resume`/`resolve_pause_signal`/
//! `service_held_query`/`peeked_line_is_holdable_query`/`wait_for_motion_idle`) stay private. `run_dwell` is also
//! reached from `inject_spin_up_dwell` (Step 11) via the re-export. The shared `dwell_duration`/`MAX_DWELL_S` (also
//! used by the spindle reverse-dwell) STAY in `comms.rs` and are reached here via the `use super::*` glob, which also
//! supplies the state statics/signals, `program_running`, `quiesce_executor`/`release_hold`, the spindle/coolant
//! force-off + WCS/override reset helpers, and the firmware_core types; the child glob needs no explicit imports.

use super::*;

/// The poll interval while waiting for the core-1 executor to drain to a synchronized boundary ([`run_dwell`]).
/// Short relative to a block's execution time so the dwell starts promptly after motion stops, but long enough
/// that the brief `PLANNER`-lock checks add negligible load while the machine winds down.
const MOTION_IDLE_POLL: Duration = Duration::from_millis(4);

/// Wait until the core-1 executor has fully drained the planner queue and stopped — grbl's buffer-synchronize, the
/// rest point a `G4` dwell (and the spin-up dwell) needs before timing begins. The consumer is the sole enqueuer
/// and is blocked here, so once [`program_running`] reads false no new motion can appear; the wait converges. Polls
/// on a short [`MOTION_IDLE_POLL`] ticker, racing [`SOFT_RESET`] so a `0x18` abandons the wait. Returns `true` once
/// idle, or `false` if a soft reset preempted it (the signal is consumed; the caller runs the reset, matching
/// [`inject_spin_up_dwell`]'s abort contract).
async fn wait_for_motion_idle() -> bool {
  while program_running().await {
    match select(Timer::after(MOTION_IDLE_POLL), SOFT_RESET.wait()).await {
      Either::First(()) => {}
      Either::Second(()) => return false,
    }
  }
  true
}

/// Run a synchronized dwell (DOC, grbl `G4`): wait for all prior motion to drain to a stop, then hold for `seconds`.
/// This is the ONE timed-dwell mechanism — a real `G4` and the `$392` spin-up both route through it, so the
/// "dwell = synchronized motion boundary" guarantee is enforced in one place rather than approximated by an
/// ad-hoc timer. Returns `true` when the dwell completed, or `false` if a soft reset preempted either phase (the
/// signal is consumed; the caller runs the reset).
pub(crate) async fn run_dwell(seconds: f32) -> bool {
  if !wait_for_motion_idle().await {
    return false;
  }
  match select(Timer::after(dwell_duration(seconds)), SOFT_RESET.wait()).await {
    Either::First(()) => true,
    Either::Second(()) => false,
  }
}

/// Run an `M30` program end (grbl): drain pending motion to a stop, stop the spindle, and reset the parser's modal
/// state to power-on defaults (G0/G90/G21/G54, F0/S0) so the next program starts clean — WITHOUT flushing the
/// planner or zeroing machine position (M30 is a program rewind, not a soft reset). Returns `true` on completion,
/// or `false` if a soft reset preempted the motion drain (the consumed signal is honored by the caller).
pub(crate) async fn program_end(parser: &mut Parser, state: &mut ConsumerState) -> bool {
  if !wait_for_motion_idle().await {
    return false;
  }
  // M30 turns the spindle off: park it (SPIN_EN off + duty 0) and clear the commanded direction + programmed RPM.
  // grbl's M30 also turns COOLANT off (group 8 → M9); force both off and clear the last-dispatched mask.
  force_spindle_off();
  force_coolant_off();
  state.last_coolant = 0;
  PROGRAMMED_SPINDLE_RPM.store(0, Ordering::Release);
  // Reset modal state to defaults and the spindle tracking to match, so the next line's `sync_spindle_from_modal`
  // sees a fresh `Stop`/`S0` baseline rather than the just-ended program's direction. `Parser::new()` resets the
  // modal WCS to G54 (index 0). The CURRENT tool is RETAINED across M30 (grbl: program end is a rewind, not a tool
  // change — the selected tool survives), so snapshot it and restore it after the rebuild.
  let retained_tool = parser.state().current_tool;
  *parser = Parser::new();
  parser.set_current_tool(retained_tool);
  state.spin_up = SpinUpGate::new();
  state.last_spindle_dir = SpindleState::Stop;
  state.last_spindle_rpm = 0;
  // grbl M30 also: selects G54, turns coolant OFF, and resets feed/rapid/spindle overrides to 100%. The parser
  // reset above only restored the parser-MODAL WCS — push that G54 selection into the coordinate model + planner
  // too (otherwise the planner keeps the ended program's G55-G59 offset and the next move cuts at the wrong WPos),
  // and reset the live overrides + coolant toggles (which live outside the parser, in `OVERRIDES`), matching the
  // soft-reset reset of the same cross-task state.
  sync_active_wcs(0).await;
  set_overrides(Overrides::new());
  reset_ov_reporter();
  true
}

/// The outcome of an M0/M1/M6 program-flow pause ([`run_program_pause`]).
pub(crate) enum PauseOutcome {
  /// The pause completed: motion drained, the machine held, and a cycle-start (`~`) resumed it. The caller `ok`s.
  Resumed,
  /// The pause did NOT halt: an `M1` whose optional-stop gate is off. No hold occurred; the caller still `ok`s the
  /// line so the stream continues (an `M1` is always a valid line, it simply does nothing when the switch is off).
  Skipped,
  /// A soft reset (`0x18`) preempted the drain or the hold-await; the caller runs the warm reset and drops the `ok`.
  Aborted,
  /// A graceful program stop (`0x86`) preempted the drain or the hold-await; the caller runs the clean stop and
  /// drops the `ok`. Distinct from [`Aborted`](PauseOutcome::Aborted) — a stop returns to Idle with position
  /// retained and no alarm.
  Stopped,
}

/// Run an M0/M1/M6 program-flow pause (grbl program-flow). The line's `ok` is the CALLER's job and is emitted only
/// when this returns [`Resumed`](PauseOutcome::Resumed) / [`Skipped`](PauseOutcome::Skipped) — i.e. once the pause
/// has run to completion — so the host's character-counting naturally stalls while paused (this is correct flow
/// control, exactly like the `G4` dwell, NOT a deferred ack).
///
/// Sequencing, reusing the existing graceful-hold machinery rather than a parallel hold path:
/// 1. **M1 gate**: an optional stop whose [`OPTIONAL_STOP_ENABLED`] switch is OFF returns [`Skipped`] immediately
///    (no hold) — grbl's default M1 behavior. M0 and M6 always pause.
/// 2. **Drain**: wait for the core-1 executor to drain to a stop ([`wait_for_motion_idle`] — the planner already
///    flushed look-ahead so the preceding block decelerates to rest). A soft reset here returns [`Aborted`].
/// 3. **M6 prompt**: a tool-change pause emits a `[MSG:..]` NAMING `current_tool` so the operator knows which tool
///    to swap in, then resume. (The tool number is human-readable only; `skirnir` sources it from the program.)
/// 4. **Hold**: latch `Hold:0` (M0/M1) or the `Tool` state (M6) and mark [`PAUSE_ACTIVE`], then [`hold_until_resume`]
///    awaits a cycle-start [`PAUSE_RESUME`] nudge (raced against a soft reset and a graceful stop) WHILE STILL
///    SERVICING read-only `$`-queries — so the hold returns to `Normal` on `~` and a host is never stalled.
///
/// ## `$`-QUERIES ARE SERVICED DURING THE HOLD (grbl-faithful)
/// Real grbl/grblHAL answers read-only queries (`$G`/`$#`/`$$`/`$I`…) DURING a hold — the motion is held, the
/// protocol loop is not — and so does this firmware: [`hold_until_resume`] PEEKS the line queue and answers any
/// read-only `$`-query in place (its report + `ok`) without releasing the hold, while LEAVING every motion / write /
/// action line queued to run after resume. This is the fix for the earlier divergence where a `$G` on entering the
/// `Tool` state stalled a character-counting host until `~`. (`?` was always answered — it is served by the separate
/// [`status_responder`] task — so live state/DRO is available throughout regardless.) See [`hold_until_resume`] for
/// the exact serviced-vs-deferred routing.
pub(crate) async fn run_program_pause(
  optional: bool,
  tool_change: bool,
  current_tool: u16,
  parser: &mut Parser,
  state: &mut ConsumerState,
  flash: &'static SharedFlash,
) -> PauseOutcome {
  // 1. An M1 with the optional-stop switch off does not halt (grbl default). It is still a valid, acknowledged line.
  if optional && !OPTIONAL_STOP_ENABLED.load(Ordering::Relaxed) {
    return PauseOutcome::Skipped;
  }
  // 2. Drain pending motion to a stop so the pause holds at rest (the planner flushed look-ahead already). A soft
  //    reset abandons the drain; the consumed signal is honored by the caller.
  if !wait_for_motion_idle().await {
    return PauseOutcome::Aborted;
  }
  // 3. A manual tool change: prompt the operator to swap the tool before holding. This machine has no ATC, so the
  //    swap is manual and the operator resumes with `~` when done (grblHAL reports the `Tool` state during a manual
  //    change; the state below reports the dedicated grblHAL `Tool` state for an M6 so a sender shows a tool-change
  //    prompt, while M0/M1 report `Hold:0`).
  if tool_change {
    // Build the tool-named prompt with the host-tested formatter, then push it. A formatter failure (buffer too
    // small — unreachable for this fixed-length text) simply skips the prompt rather than panicking.
    let mut text = Response::new();
    if ResponseWriter::tool_change_message(&mut text, current_tool).is_ok() {
      send_message(text.as_str()).await;
    }
  }
  // 4. Latch the hold and mark the pause active so the `~` handler nudges THIS wait, then await the resume. An M6
  //    enters the dedicated `Tool` state (grblHAL `STATE_TOOLCHANGE`, reported as `<Tool|...>`); M0/M1 enter the
  //    feed-hold `Hold:0`. BOTH resume on cycle-start (`resumes_on_cycle_start` covers Tool too) back to `Normal`.
  let paused_state = if tool_change { ControlState::tool_change() } else { control_state().feed_hold() };
  set_control_state(paused_state);
  PAUSE_ACTIVE.store(true, Ordering::Release);
  let outcome = hold_until_resume(parser, state, flash).await;
  PAUSE_ACTIVE.store(false, Ordering::Release);
  outcome
}

/// Hold (the M0/M1/M6 pause body) until a cycle-start resume, a soft reset, or a graceful stop — while STILL
/// SERVICING read-only `$`-queries (grbl answers `$G`/`$#`/`$$`/`$I` etc. during a hold). This is the fix for the
/// "held machine appears to HANG a character-counting host" bug: a query that arrives during the hold is answered
/// (report + `ok`) IN PLACE so the host's send-ahead window frees, and the hold is NOT released.
///
/// ## Exactly what is serviced vs deferred
/// Each iteration PEEKS the head of [`LINE_QUEUE`] WITHOUT consuming it ([`Channel::try_peek`]) and routes by
/// [`peeked_line_is_holdable_query`]:
/// - A pure read-only `$`-query (`$`, `$$`, `$I`/`$I+`, `$G`, `$#`, `$N`, `$ES`/`$EG`/`$EE`/`$EA`, `$SED=`, `$PBX`):
///   CONSUME it and answer via [`handle_system_command`] (its report + the one `ok`), then loop back into the hold.
///   The `$`-query also clears the gcode error-hold, exactly as it would outside a pause (a `$` command is a grbl
///   recovery trigger).
/// - ANYTHING ELSE — a gcode/jog motion line, a blank line, a setting WRITE (`$n=val`), `$X`/`$C`/`$SLP`/`$H`, a
///   `$N0=`/`$PBX=` write, or an `Unknown` `$` command — is LEFT ON THE QUEUE (peeked, not received). It is NOT
///   executed during the hold and runs IN ORDER once the pause resumes, so a held machine never mutates state or
///   moves behind the operator's back, and stream order is preserved (no reordering — the deferred line stays at
///   the head). The `select` then drops the line arm so a deferred head does not busy-spin: it waits only on the
///   resume / reset / stop signals until one fires (or `~` resumes and the consumer's normal loop reads the line).
///
/// `?` is unaffected throughout — it is served by the separate [`status_responder`] task, so a host polling `?`
/// sees the live `<Tool|...>` / `<Hold:0|...>` state and DRO for the whole hold.
async fn hold_until_resume(
  parser: &mut Parser,
  state: &mut ConsumerState,
  flash: &'static SharedFlash,
) -> PauseOutcome {
  loop {
    // An M0/M1/M6 pause holds here until cycle-start/reset/stop — a legitimately LONG park (operator-gated). Mark
    // it so a wedge here reads as the pause-wait, not a stuck await (it is RELEASED by `~`, which is expected).
    crate::crash::record_comms_stage(crate::crash::CommsTask::Consumer, crate::crash::CommsStage::ConsumerSyncWait);
    // Whether the head line (if any) is a serviceable read-only `$`-query. A peek that finds the queue empty, or a
    // head that is a deferred (non-query) line, both yield `false` — in which case we wait only on the signals so a
    // deferred head cannot busy-spin the `ready_to_receive` arm.
    let head_is_query = matches!(LINE_QUEUE.try_peek(), Ok(line) if peeked_line_is_holdable_query(line.as_slice()));
    // The three terminating signals, plus — ONLY when a serviceable query is at the head — a line-ready arm. With no
    // query at the head the line arm is omitted, so the select parks on the signals until a resume/reset/stop.
    let signals = select(PAUSE_RESUME.wait(), select(SOFT_RESET.wait(), PROGRAM_STOP.wait()));
    if head_is_query {
      match select(signals, LINE_QUEUE.ready_to_receive()).await {
        Either::First(resolved) => return resolve_pause_signal(resolved),
        // A serviceable query is at the head and ready: consume it and answer it in place, then loop back into the
        // hold. `try_receive` cannot fail here — we just peeked it ready, and this task is the sole receiver.
        Either::Second(()) => {
          if let Ok(line) = LINE_QUEUE.try_receive() {
            service_held_query(line.as_slice(), parser, state, flash).await;
          }
        }
      }
    } else {
      // No serviceable query at the head (empty queue, or a deferred non-query line that must wait for resume): park
      // on the signals only. A deferred line stays at the head and is read by the consumer's normal loop after `~`.
      return resolve_pause_signal(signals.await);
    }
  }
}

/// Map the resolved pause-await signal `select` to its [`PauseOutcome`]. The soft-reset / graceful-stop signals are
/// CONSUMED here; the caller runs `apply_soft_reset` / `program_stop_cycle` directly (matching `run_dwell`'s abort
/// contract — do NOT re-signal, or the consumer would act twice). A cycle-start resume has already cleared the hold
/// level + set the control state back to `Normal` via the `~` handler's `cycle_start()`.
fn resolve_pause_signal(resolved: Either<(), Either<(), ()>>) -> PauseOutcome {
  match resolved {
    Either::First(()) => PauseOutcome::Resumed,
    Either::Second(Either::First(())) => PauseOutcome::Aborted,
    Either::Second(Either::Second(())) => PauseOutcome::Stopped,
  }
}

/// Service a read-only `$`-query line that arrived DURING a pause hold, mirroring `handle_line`'s `$`-command path:
/// trim, strip the leading `$`, clear the gcode error-hold (a `$` command is a grbl recovery trigger), and dispatch
/// to [`handle_system_command`] (which emits the report + the single `ok`). Only ever called on a line
/// [`peeked_line_is_holdable_query`] has already confirmed is a read-only query, so the strip/classify is provably a
/// read-only command; the defensive re-check keeps it correct if the head changed between peek and receive.
async fn service_held_query(line: &[u8], parser: &mut Parser, state: &mut ConsumerState, flash: &'static SharedFlash) {
  let trimmed = trim_ascii(line);
  if let Some(rest) = trimmed.strip_prefix(b"$") {
    if SystemCommand::classify(rest).is_readonly_query() {
      state.error_hold = false;
      handle_system_command(rest, parser, state, flash).await;
    }
  }
}

/// Whether a peeked line is a read-only `$`-query that may be SERVICED while an M0/M1/M6 pause hold is active
/// (without releasing the hold). True only for a `$`-prefixed line (after trimming) whose [`SystemCommand`] is a
/// [`is_readonly_query`](SystemCommand::is_readonly_query). A `$J=` jog is deliberately excluded (it is a MOTION
/// command, not a `$`-query — `strip_jog_prefix` routes it elsewhere), as is any blank/gcode line and any `$`-write.
fn peeked_line_is_holdable_query(line: &[u8]) -> bool {
  let trimmed = trim_ascii(line);
  if strip_jog_prefix(trimmed).is_some() {
    return false; // `$J=` is a jog (motion), never a held-state query.
  }
  match trimmed.strip_prefix(b"$") {
    Some(rest) => SystemCommand::classify(rest).is_readonly_query(),
    None => false,
  }
}

/// Run a graceful program stop (`0x86`, Galdr extension) end to end, GENERALIZING [`cancel_jog_cycle`] from a jog
/// to a running program. It decelerates the running/held program to a controlled stop at the active block's
/// boundary (no step loss), flushes the WHOLE planner queue + any in-progress arc, syncs the planner's commanded
/// position to the actual live stop point, clears the program/modal-run state (mirroring `M30`), DROPS the aborted
/// program's buffered inbound stream (so no already-streamed line is re-parsed/executed/`ok`'d after the stop), and
/// returns the machine to Idle (NOT alarm) with position RETAINED. It raises no alarm and re-emits no banner — the
/// operator's clean "stop the job", distinct from the `0x18` abort ([`apply_soft_reset`] → `ALARM:3` + banner + warm
/// reset).
///
/// ## How it composes the existing machinery
/// The boundary-stop + sync is the SAME [`quiesce_executor`] / [`release_hold`] primitive jog-cancel uses (so the
/// stop is a real parked acknowledgment, not a poll); the difference is it flushes EVERY queued block via
/// [`Planner::flush_queue`] (not just trailing jogs) and [`Planner::abort_arc`] (so a partially-streamed arc is
/// discarded), then clears the modal/spindle/override state exactly as [`program_end`] (`M30`) does — WITHOUT a
/// warm reset (no parser rebuild that loses coordinates, no `force_spindle_off`-driven banner, no position zero).
///
/// ## Safe from Run or Hold, benign otherwise
/// The reader half only signals this when [`ControlState::program_stop_quiesces`] holds, but the state can change
/// between the signal and here (a soft reset winning a tie), so this RE-CHECKS the live state and is a benign no-op
/// if a program is no longer running/held. A soft reset landing mid-quiesce is honored (the quiesce reports
/// `ResetPreempted` and the re-signalled `0x18` runs its own reset), so the abort always wins a race with the stop.
///
// TODO(DOC-02 Stage-2): mid-block ramp-down. Like jog-cancel and feed-hold, we stop at the current block boundary
// rather than ramping velocity down mid-block; a smooth mid-block deceleration is the shared Stage-2 refinement.
pub(crate) async fn program_stop_cycle(parser: &mut Parser, state: &mut ConsumerState) {
  // Re-check the live state: the reader gated on `program_stop_quiesces`, but a soft reset could have won a tie and
  // moved us out of Run/Hold. A stop is only meaningful from a running/held program; anything else is a no-op.
  if !control_state().program_stop_quiesces() {
    return;
  }
  // 1. Flush the WHOLE queue (program + any trailing jog blocks) AND any in-progress arc FIRST, under the planner
  //    lock, so the executor has nothing more to pop after it finishes the active block — the in-flight block stops
  //    at its boundary and no flushed-away block follows it. `abort_arc` drops a partially-streamed over-subdivided
  //    arc so its remaining segments are never fed after the stop.
  {
    let mut guard = PLANNER.lock().await;
    if let Some(planner) = guard.as_mut() {
      planner.flush_queue();
      planner.abort_arc();
    }
  }
  // 2. Park the executor at the active block's boundary via the shared quiesce primitive (raises the hold level and
  //    AWAITS a real parked acknowledgment). A soft reset mid-quiesce is honored — the reset supersedes the stop.
  match quiesce_executor().await {
    QuiesceOutcome::Parked => {}
    // The reset already cleared the hold level, flushed the planner, and (now) RETAINED the position; abandon the
    // stop and let `comms_consumer` run the re-signalled reset. Do NOT release the hold — the reset already did.
    QuiesceOutcome::ResetPreempted => return,
  }
  // 3. Sync the planner's commanded position to the ACTUAL live stop point so a subsequent move resolves from where
  //    the machine really stopped (the executor is genuinely parked now, so the live position is stable to read).
  //    This is also what keeps the planner's commanded position consistent with the RETAINED live MPos (Change A).
  let stop_steps = read_live_position();
  {
    let mut guard = PLANNER.lock().await;
    if let Some(planner) = guard.as_mut() {
      planner.sync_position(stop_steps);
    }
  }
  // 4. Release the hold so the executor leaves its parked branch (its queue is empty, so it returns to awaiting the
  //    next block) and clear the program/modal-run state, mirroring `M30`: stop the spindle, reset the parser modal
  //    state + spindle tracking to power-on defaults, select G54, and reset the live overrides — so the next stream
  //    starts clean. Coordinates/offsets and the live MACHINE POSITION are deliberately RETAINED (this is a clean
  //    stop, not a warm reset): no banner, no parser-rebuild that drops the WCS, no position zero.
  release_hold();
  force_spindle_off();
  // A graceful program stop clears coolant too (it mirrors M30's reset-to-defaults); force both off.
  force_coolant_off();
  state.last_coolant = 0;
  PROGRAMMED_SPINDLE_RPM.store(0, Ordering::Release);
  // Like M30 / soft reset, the CURRENT tool is RETAINED across a graceful stop (the spindle still holds it); carry
  // it across the parser rebuild.
  let retained_tool = parser.state().current_tool;
  *parser = Parser::new();
  parser.set_current_tool(retained_tool);
  state.spin_up = SpinUpGate::new();
  state.last_spindle_dir = SpindleState::Stop;
  state.last_spindle_rpm = 0;
  sync_active_wcs(0).await;
  set_overrides(Overrides::new());
  reset_ov_reporter();
  // 5. Discard the aborted program's BUFFERED INBOUND stream, mirroring the `0x18` soft-reset flush in
  //    `dispatch_realtime` (drop every buffered RX byte, every framed-but-unconsumed line, and the assembler's
  //    partial line). Without this, the ~50 lines the host already streamed before the `0x86` survive the stop in
  //    `RX_PIPE`/`LINE_QUEUE`; `line_assembler` would keep feeding them to this consumer, which would re-plan and
  //    execute them (the DRO keeps running) and `ok` each — spurious acks to a host that reset its window on the
  //    host-side stop, plus an `error:1` from a leaked partial line. This is done AFTER `quiesce_executor` returns:
  //    bytes already in flight on USB keep landing in `RX_PIPE` for the whole quiesce-await window (the reader half
  //    stays non-blocking), so flushing earlier would leave those late arrivals buffered. The host stops sending the
  //    moment it issues `0x86`, so the tail is finite and fully arrived by the parked acknowledgment — one flush here
  //    drops it cleanly. The `ResetPreempted` early-return above skips this deliberately: a soft reset winning the
  //    tie already ran the identical flush in the reader half, so there is nothing left to clear.
  RX_PIPE.clear();
  while LINE_QUEUE.try_receive().is_ok() {}
  LINE_RESET.signal(());
  // Finally, latch the control state back to Idle-capable `Normal` (NO alarm). `program_stop` maps Run/Hold → Normal
  // and is a no-op elsewhere; the reported `?` state re-derives Idle from the now-empty queue.
  set_control_state(control_state().program_stop());
}
