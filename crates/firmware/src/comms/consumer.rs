//! The parser → planner consumer pipeline (DOC-08 / DOC-05): the single in-order [`comms_consumer`] task and
//! its per-line handling — [`handle_line`], [`plan_gcode_line`], [`plan_command`] (+ the `QueueFull`
//! back-pressure retry and the resumable-arc drive), the G28/G30 predefined recall, the coordinate-op apply, the
//! `0x18` soft-reset warm reset + pipeline rebuild, and the coalesced settings/coordinate flash flushes. Split out
//! of `comms.rs` (architecture-refactor A1, Step 14); the pure extract-method decomposition of `plan_gcode_line`
//! (B1) lands on top of this move.
//!
//! The shared helpers this pipeline leans on — `emit_alarm` / `send_message` / `dwell_duration` / `units_scale` /
//! `current_soft_limits` / `program_running` / `read_live_position` / `motion_idle` / `strip_jog_prefix` and the
//! [`ConsumerState`] type — stay in the `comms.rs` shared-core parent and are reached here via the `use super::*`
//! glob (which also surfaces the parent's own `firmware_core` imports), so this module needs no explicit imports.

use super::*;

/// The real parser → planner consumer (replaces the Stage-1 stub). It is the SINGLE, in-order consumer of
/// [`LINE_QUEUE`], so it is the natural owner of the grblHAL gcode error-hold (see [`ConsumerState`]). Per
/// line it: routes `$` system commands to their handlers; parses GCode through a persistent [`Parser`];
/// and feeds the resulting [`PlannerCommand`](firmware_core::gcode::PlannerCommand) to a persistent
/// [`Planner`]. It emits exactly one `ok`/`error:N` per consumed line, preserving the one-response-per-line
/// contract end to end (`line_assembler` only ever responds for the protocol-level overflow reject it
/// handles itself, so there is no double-response or gap at the boundary).
///
/// ## Error-hold ownership (race-free by construction)
/// The grblHAL contract holds all subsequent lines in an error state after a GCode line errors, until a
/// reset / empty line / `$` command. That hold lives HERE, in [`ConsumerState::error_hold`], not in the
/// `StreamEngine`: the framer runs in `line_assembler` and forwards lines asynchronously, so it cannot know
/// a line errored downstream, and any back-channel to it would race the lines already in flight in
/// `LINE_QUEUE`. Because this task is the only reader of `LINE_QUEUE` and sees parse/plan results strictly
/// in queue order, owning the hold here is inherently in-order and race-free. The `StreamEngine` was
/// deliberately reduced to pure line framing (no error-hold state); the framer still owns the independent
/// protocol-level overflow reject, which is correct.
///
/// ## Back-pressure (gated on the core-1 motion executor draining blocks)
/// `ok` for a move is emitted only once the block is ACCEPTED into the planner buffer. When the planner is
/// full ([`PlannerError::QueueFull`]) the consumer neither acks nor drops the line: it waits for the core-1
/// `motion_executor` to execute a block and free a slot, then retries the SAME command (the arc planner is
/// all-or-nothing on `QueueFull`, so re-issuing is safe). While waiting it stops reading `LINE_QUEUE`, which backs up, blocks
/// `line_assembler`'s `send().await`, stops draining `RX_PIPE`, fills the pipe, makes the reader's
/// `try_write` refuse bytes, and lets the host's character-counting throttle — exactly the correct grbl flow
/// control, now with real-time dispatch still live throughout because it sits in the separate reader half.
#[embassy_executor::task]
pub async fn comms_consumer(flash: &'static SharedFlash) -> ! {
  let mut parser = Parser::new();
  let mut state = ConsumerState::default();
  loop {
    // Coalesced settings persist (Finding #14b): if a `$n=val`/`$PBX` change is pending and the input has
    // drained (no more lines queued), this is a burst boundary — flush the live settings to flash ONCE for
    // the whole burst before blocking for the next event, instead of writing per line. The flush yields the
    // executor while the flash op runs; `is_empty` is the cheap "host paused" signal the brief specifies.
    // `motion_idle()` additionally defers the persist while a cycle is in flight: the esp-storage flash write
    // parks the real-time motion core (`multicore_auto_park`), so flushing mid-cycle would briefly stall step
    // generation — a still-pending change is caught by the safety tick once motion drains to idle.
    if SETTINGS_DIRTY.load(Ordering::Acquire) && LINE_QUEUE.is_empty() && motion_idle().await {
      // `false`: this path fires every loop iteration while dirty, so it must NOT re-mark on failure or it would
      // busy-retry a persistently-failing write each loop. A failed write here is retried by the safety timer.
      flush_settings(flash, false).await;
    }
    // Coalesced coordinate persist (Phase B): same burst-boundary rule for the persistent G54-G59 / G28 / G30
    // record (also deferred while moving, for the same auto-park reason), so a program that re-zeroes several
    // axes appends the coordinate blob once, not per line.
    if COORDINATES_DIRTY.load(Ordering::Acquire) && LINE_QUEUE.is_empty() && motion_idle().await {
      flush_coordinates(flash, false).await;
    }
    // Race the next line against a soft reset AND a periodic safety flush. A `0x18` resets the parser modal
    // state, clears the error-hold, and flushes the planner queue (and persists any pending settings) BEFORE
    // the next line is parsed; `usb_rx` already cleared `RX_PIPE` + `LINE_QUEUE` and signalled the line
    // assembler. The safety-flush timer guarantees a dirty change is persisted within [`SETTINGS_FLUSH_SAFETY`]
    // even if the queue never observably empties (e.g. a slow trickle that always has one line in flight). A
    // line that lost the reset race is dropped (it predates the reset), matching grbl's warm-reset semantics.
    //
    // Known window (Finding #6, accepted): the select only observes the reset at the loop boundary. If a
    // `0x18` lands while `handle_line` is already mid-flight for a non-back-pressured line, that line can
    // still emit its `ok`/`error` after the reset signal — a single stray response. Back-pressured lines do
    // observe the reset (they race `SOFT_RESET` inside `plan_command` and return `Aborted`). Closing the
    // remaining window for an in-flight non-back-pressured line would require cancelling `handle_line`
    // mid-await; that is deferred to the alarm-state machine (Stage 2), which is where grbl gates response
    // emission during an abort. The stray response is benign: the host discards pending acks on `0x18`.
    // About to park on the main consumer wait (next line / reset / jog-cancel / safety tick / hard-limit / stop).
    // This is the idle-class park when the queue is drained; a wedge HERE with RX live means the next line never
    // arrives through `LINE_QUEUE` (suspect `line_assembler`/`usb_rx` upstream, or `LINE_QUEUE` never delivering).
    crate::crash::record_comms_stage(crate::crash::CommsTask::Consumer, crate::crash::CommsStage::ConsumerWaitLine);
    let events = select4(LINE_QUEUE.receive(), SOFT_RESET.wait(), JOG_CANCEL.wait(), Timer::after(SETTINGS_FLUSH_SAFETY));
    // Race the four primary events against a hard-limit trip from the core-1 executor (DOC-06): a limit pressed
    // during normal motion must halt the program and enter `ALARM:1` regardless of what the consumer is waiting on.
    // Also race a graceful program stop (`0x86`): a controlled decelerate-to-Idle + full flush that, unlike the
    // hard-limit trip and the `0x18` reset, raises no alarm and retains position; AND a motion fault (`MOTION_FAULT`,
    // §15.6 / task #22): an unrecoverable mid-block step-output Transport error from the executor must halt the
    // program and enter the LOCKED `ALARM:17` regardless of what the consumer is waiting on. Nested `select`s keep
    // each arm typed.
    match select(select(events, HARD_LIMIT_TRIPPED.wait()), select(PROGRAM_STOP.wait(), MOTION_FAULT.wait())).await {
      Either::First(Either::First(Either4::First(line))) => {
        // OBSERVE-ONLY probe (task #22 §15): a line is being CONSUMED. Counted BEFORE `handle_line` so it pairs with
        // the exactly-one terminal response that line will emit — `ACKS_EMITTED > LINES_CONSUMED` would then be a
        // firmware over-ack. (A line that emits NO terminal response — e.g. an aborted/stopped back-pressured line —
        // makes `acks < cons`, which is fine; the over-ack direction is the load-bearing one.)
        LINES_CONSUMED.fetch_add(1, Ordering::Relaxed);
        // Comms-progress heartbeat: a line was fully processed (parsed, planned/queued, acked). This is the
        // consumer-side companion to the `usb_tx`/`status_responder` bumps — together they prove the WHOLE
        // host-facing pipeline (parse -> plan -> respond) is advancing. A consumer stuck on a never-resolving
        // back-pressure / probe / home await stops bumping this, which (with RX live) trips the comms-stall reset.
        handle_line(line.as_slice(), &mut parser, &mut state, flash).await;
        COMMS_PROGRESS.fetch_add(1, Ordering::Relaxed);
      }
      // A soft reset must not lose a pending settings change: persist before rebuilding the pipeline (grbl
      // applies most settings on the next reset, so they MUST be on flash by the time the reset takes them).
      Either::First(Either::First(Either4::Second(()))) => {
        // `true`: a failed persist here must be retried (by the safety timer or the next reset), not dropped —
        // grbl applies settings on the next reset, so a lost write would mean the reset takes stale flash values.
        flush_settings(flash, true).await;
        flush_coordinates(flash, true).await;
        apply_soft_reset(&mut parser, &mut state).await;
      }
      // Jog cancel (`0x85`, Phase D): reuse the feed-hold block-boundary stop, flush the jog blocks, sync the
      // planner to the live stop point, and return to Idle. Owned here (the planner owner) so it is race-free
      // with line handling — a cancel and a line never run concurrently in this single in-order consumer.
      Either::First(Either::First(Either4::Third(()))) => cancel_jog_cycle().await,
      // Safety-interval tick: persist any pending change even if the queue never observably drained — but only
      // while the machine is idle, since the flash write parks the real-time motion core (see `motion_idle`). A
      // change made mid-cycle therefore persists on the first safety tick AFTER motion drains (<=1s later); the
      // `||`-guarded `motion_idle()` is skipped entirely when nothing is dirty so an idle board never locks the
      // planner here. When nothing is dirty these are cheap no-ops and the loop simply re-arms the timer.
      Either::First(Either::First(Either4::Fourth(()))) => {
        let pending = SETTINGS_DIRTY.load(Ordering::Acquire) || COORDINATES_DIRTY.load(Ordering::Acquire);
        if pending && motion_idle().await {
          // `true`: the safety interval is the bounded-cadence retry for a failed write — re-marking dirty here lets
          // the next tick re-attempt, which is exactly the guarantee Bug A defeated (a single failure dropped it).
          flush_settings(flash, true).await;
          flush_coordinates(flash, true).await;
        }
      }
      // Hard-limit trip (`$21`, DOC-06): the executor detected a switch trip during normal motion. Enter the
      // LOCKED `ALARM:1` (position is likely lost from the abrupt stop — re-homing recommended) and reset the
      // pipeline so the queue is flushed and the machine sits in a clean, clearly-halted alarm. Only a soft
      // reset clears a locked alarm.
      Either::First(Either::Second(())) => {
        // Guard against a STALE trip clobbering an already-halted machine (Finding #5b): raise `ALARM:1` only
        // from a state where the machine could actually be MOVING (`Normal`/`Hold`/`Jog`/`Check`). The
        // host-tested `hard_limit_alarm_applies` predicate decides. If we are already in an alarm (or asleep), a
        // trip here is a stale read of a parked switch — re-raising would only downgrade a more-specific lock,
        // most damagingly turning the `ALARM:11` boot-lock into the locked `ALARM:1`, which `$X` cannot clear
        // (the `error:9` wedge). A legitimately NEW over-travel always arrives from a moving state, so this never
        // suppresses a real trip; the soft-reset drains above are the primary fix and this is the last guard.
        if control_state().hard_limit_alarm_applies() {
          set_control_state(ControlState::Alarm(AlarmCode::HardLimit));
          emit_alarm(AlarmCode::HardLimit).await;
          reset_pipeline(&mut parser, &mut state).await;
        }
      }
      // Graceful program stop (`0x86`, Galdr extension): a controlled decelerate-to-Idle that flushes the program
      // and RETAINS position — the operator's "stop the job cleanly", distinct from the `0x18` abort. Owned here
      // (the planner owner) so it is race-free with line handling; the reader half only signalled it after the
      // `program_stop_quiesces` gate held. Re-checks the live state inside the cycle so a state change between the
      // signal and here (e.g. a soft reset winning a tie) makes it a benign no-op.
      Either::Second(Either::First(())) => program_stop_cycle(&mut parser, &mut state).await,
      // Motion fault (`MOTION_FAULT`, §15.6 / task #22): the core-1 executor hit an unrecoverable mid-block
      // step-output Transport error (a swallowed RMT `wait()`/`transmit()` fault) that truncated a cutting move. The
      // step sync is broken, so on open-loop steppers position certainty is LOST — enter the LOCKED `ALARM:17`
      // (MotorFault) and reset the pipeline so the queue is flushed and the machine sits in a clean, clearly-halted
      // alarm requiring a re-home. This is the grbl lost-step-sync contract (§14.3): NEVER silent abandonment or a
      // silent reset that would cut the rest of the part in the wrong place. The same stale-trip guard as the
      // hard-limit path applies — only raise from a state where the machine could actually be MOVING, so a duplicate
      // fault signalled after the machine already halted into the alarm cannot downgrade a more-specific lock.
      Either::Second(Either::Second(())) => {
        if control_state().hard_limit_alarm_applies() {
          set_control_state(ControlState::Alarm(AlarmCode::MotorFault));
          emit_alarm(AlarmCode::MotorFault).await;
          reset_pipeline(&mut parser, &mut state).await;
        }
      }
    }
  }
}

/// Persist the live [`SETTINGS`] to flash IF a change is pending, clearing [`SETTINGS_DIRTY`]. This is the
/// single coalesced write path (Finding #14b): callers mark settings dirty per `$n=val`/`$PBX` line, and this
/// performs the actual flash append once per burst (queue-empty), on the safety interval, and on soft reset.
///
/// The dirty flag is cleared BEFORE the write so a change landing during the (awaited) flash op re-marks dirty
/// and is caught by the next flush — never silently coalesced away.
///
/// On write FAILURE we re-mark the dirty flag when `retry_on_failure` is set, so a transient flash error does not
/// permanently drop the change (Bug A): the soft-reset and safety-interval guarantees only hold if a failed write
/// is re-attempted. The burst-boundary caller at the top of the consumer loop passes `false` — it fires every
/// iteration while `dirty && queue empty`, so re-marking there would busy-retry a persistently-failing write each
/// loop. The safety-timer and soft-reset arms pass `true`: they re-attempt on a BOUNDED cadence (the next
/// [`SETTINGS_FLUSH_SAFETY`] tick, or the next reset) rather than spinning. The in-RAM value already applied and
/// the line was already `ok`'d either way — a stalled persist must never wedge a character-counting sender.
async fn flush_settings(flash: &'static SharedFlash, retry_on_failure: bool) {
  // Clear first so a concurrent `$n=val` applied during the await re-sets the flag and is not lost.
  if !SETTINGS_DIRTY.swap(false, Ordering::AcqRel) {
    return;
  }
  let snapshot = settings_snapshot().await;
  let mut store = FlashRecordStore::settings(flash);
  // Mark the flash write: `store_settings` does the NVS erase/append which `multicore_auto_park` runs with the
  // other core stalled and the cache disabled (Defect #2). A wedge here is the smoking gun for the flash hazard.
  crate::crash::record_comms_stage(crate::crash::CommsTask::Consumer, crate::crash::CommsStage::ConsumerFlashSettings);
  if settings::store_settings(&mut store, &snapshot).await.is_err() {
    #[cfg(feature = "defmt")]
    defmt::warn!("settings: failed to flush settings to flash");
    // Re-mark so the next safety-timer tick or soft reset retries; the burst-boundary path passes `false` to
    // avoid spinning the failing write every loop. The re-mark cannot clobber a concurrent newer `$n=val`: that
    // write also sets the flag, so the worst case is the same flag already being set.
    if retry_on_failure {
      SETTINGS_DIRTY.store(true, Ordering::Release);
    }
  }
}

/// Apply a `0x18` soft reset: compute and publish the post-reset [`ControlState`] from grbl's rules (a reset
/// that ABORTED an in-progress cycle → `ALARM:3`; otherwise the boot state — homing-lock when `$22` is set,
/// else Normal), emit `ALARM:N` when the reset lands in an alarm, then rebuild the parser/planner pipeline.
/// `was_in_cycle` is the [`RESET_WAS_RUNNING`] latch the reader half captured at dispatch time, so the abort
/// decision is race-free with the executor's own reset. The boot-lock branch also re-prompts `[MSG:..unlock]`.
pub(crate) async fn apply_soft_reset(parser: &mut Parser, state: &mut ConsumerState) {
  let was_in_cycle = RESET_WAS_RUNNING.swap(false, Ordering::Relaxed);
  let homing_enabled = HOMING_ENABLED.load(Ordering::Relaxed);
  let next = control_state().soft_reset(was_in_cycle, homing_enabled);
  set_control_state(next);
  // A reset returns to the boot baseline (finding #8): with homing enabled the machine is unhomed again (back in
  // the `ALARM:11` lock until `$H`), so position certainty — and thus `$20` soft-limit enforcement — is lost.
  // With homing disabled the machine stays "homed" (face-value position), mirroring `init_control_state`.
  store_homed_baseline(homing_enabled);
  // Rebuild the pipeline FIRST so the banner (the "reset and ready" signal) is emitted, then push any
  // resulting alarm so a host sees `ALARM:N` and the `[MSG:..]` prompt right after the banner — matching grbl's
  // connect/reset ordering.
  reset_pipeline(parser, state).await;
  // No `HARD_LIMIT_TRIPPED` drain here anymore (Finding #5b is now fixed at the source). The hard-limit alarm is
  // EDGE-armed in the executor, which re-seeds its per-axis arming to the settled levels at the reset / post-homing
  // boundaries — so a switch still parked engaged after an aborted `$H` seek is a HELD level, not a fresh edge, and
  // the executor's concurrent block-boundary / idle re-samples during this `reset_pipeline` await window can no
  // longer RE-latch a stale trip. The `error:9` re-lock is prevented by the arming, not by draining a latch after
  // the fact. The `hard_limit_alarm_applies` guard on the consumer's hard-limit arm stays as the one defensive
  // layer: a genuinely NEW over-travel re-signals from a moving state and still alarms, while a stray trip arriving
  // while already alarmed/asleep cannot downgrade a more-specific lock (e.g. `ALARM:11`) into the locked `ALARM:1`.
  if let ControlState::Alarm(code) = next {
    emit_alarm(code).await;
  }
}

/// Reset the parser/planner pipeline state this task owns on a soft reset (`0x18`): restore the parser to
/// default modal state, clear the gcode error-hold, flush the planner queue, reset the non-position snapshot
/// fields to idle, and emit the guaranteed readiness banner. The `RX_PIPE`, the `line_assembler`'s partial
/// line, and `LINE_QUEUE` were already cleared by `usb_rx`; this completes the warm reset for the downstream
/// half so a fresh stream starts from the documented modal defaults.
///
/// ## Machine position is RETAINED (Change A)
/// The LIVE MACHINE POSITION is deliberately NOT zeroed: matching grbl, a `0x18` abort RETAINS MPos so `$X`
/// unlocks at the same coordinates. The core-1 motion executor is the single owner of the live [`LIVE_POSITION`]
/// atomics and now RETAINS them across [`MOTION_RESET`] (it re-publishes the last step position, never zeroes it),
/// so the consumer must NOT write them — that would be a cross-core stale-overwrite race (Finding #3). But the
/// rebuilt [`Planner`] starts at the step origin, so this SYNCS its commanded position to the retained live step
/// position via [`Planner::sync_position`], keeping the planner's notion of position consistent with the retained
/// MPos: a subsequent ABSOLUTE move then resolves relative to the retained position, not the origin. Resetting
/// `MACHINE` to idle here only restores the fields the executor does not own (run-state / feed / spindle /
/// RX-free); `status_responder` reads MPos live. (Small accepted race: on an abort DURING motion the executor may
/// still be finishing its mid-block abort when this reads `LIVE_POSITION`; the step counter only advances
/// monotonically within a block, so the read is a valid recent step position — "suspect" exactly as grbl
/// documents an aborted-mid-move position, recovered by `$H`.)
pub(crate) async fn reset_pipeline(parser: &mut Parser, state: &mut ConsumerState) {
  // `Parser` exposes no in-place reset; reconstructing it restores the documented power-on modal defaults
  // (G0, G90, G21, F0, S0) — exactly grbl's warm-reset modal state. The CURRENT tool is RETAINED across the reset
  // (grbl keeps the physically-loaded tool through a soft reset), so snapshot it and restore it after the rebuild.
  let retained_tool = parser.state().current_tool;
  *parser = Parser::new();
  parser.set_current_tool(retained_tool);
  state.error_hold = false;
  // Drop any partially accumulated `$PBX=` import so a frame begun before the reset cannot bleed into one after.
  state.pb.reset();
  // DOC-07: clear the spin-up gate so a `M3` armed before the reset cannot inject a spurious `$392` dwell ahead of
  // the first post-reset move (the spindle is off after the reset's `force_spindle_off`). Reset the last-dispatched
  // spindle tracking to match the reconstructed parser's modal `Stop`/`S0`, so the first post-reset M3/M4/`S` is
  // seen as a fresh change by `sync_spindle_from_modal`.
  state.spin_up = SpinUpGate::new();
  state.last_spindle_dir = SpindleState::Stop;
  state.last_spindle_rpm = 0;
  // Reconstruct the planner to clear the block queue, work offset, and junction state in one step (it has no
  // public flush), rebuilding it from the LIVE settings so any `$x=val` changes made before the reset take effect
  // now (grbl applies most settings on the next reset). Snapshot the settings first so the `SETTINGS` lock is
  // released before the `PLANNER` lock is taken. The rebuilt planner starts at the step origin, but the live MPos
  // is RETAINED (Change A) by the executor, so SYNC the planner's commanded position to the retained live step
  // position — keeping the planner consistent with the retained MPos so a subsequent absolute move resolves from it
  // rather than the origin. `sync_position` only sets the position + clears junction state, so it is safe before the
  // WCO push below (which sets the work offset, untouched here). Reset the published snapshot's non-position fields
  // to idle; the live MPos atomics are retained by the executor on `MOTION_RESET`, not written here (no race).
  let planner_config = settings_snapshot().await.planner_config();
  let retained_steps = read_live_position();
  {
    let mut guard = PLANNER.lock().await;
    let mut planner = Planner::new(planner_config);
    planner.sync_position(retained_steps);
    *guard = Some(planner);
  }
  // Coordinate model on soft reset (grbl): the SESSION-only offsets (G92, dynamic TLO) clear to identity while
  // the persistent G54-G59 / G28 / G30 survive. Clear the volatile offsets, then push the recomputed WCO into
  // the freshly-rebuilt planner so absolute moves resolve against the surviving WCS offset (the new planner
  // starts with a zero offset). Reset the WCO refresh cadence so the next `?` re-emits `WCO:` (grbl's rule).
  {
    let mut coords = coordinates();
    coords.clear_volatile();
    set_coordinates(coords);
  }
  push_wco_to_planner().await;
  reset_wco_reporter();
  // Overrides reset to their defaults (100/100/100, no spindle-stop, coolant off) on a soft reset (grbl resets
  // the run-time overrides on `0x18`); clear the published live feed/spindle so `FS:` reads 0 until motion
  // resumes, and reset the `Ov:` cadence so the next `?` re-emits `Ov:` (grbl's first-report-after-reset rule).
  set_overrides(Overrides::new());
  LIVE_PROGRAMMED_FEED_MM_MIN.store(0, Ordering::Release);
  LIVE_BLOCK_IS_RAPID.store(false, Ordering::Relaxed);
  PROGRAMMED_SPINDLE_RPM.store(0, Ordering::Release);
  // DOC-07: a soft reset (`0x18`) forces the spindle off (grbl resets the spindle on reset), independent of
  // whether the reset lands in an alarm — a reset from Idle goes to Normal and never calls `emit_alarm`, so the
  // e-stop is issued here unconditionally. The commanded direction is cleared so the spindle stays off until a
  // fresh M3/M4 after the reset.
  force_spindle_off();
  // DOC-07: a soft reset also forces coolant off (grbl resets coolant on `0x18`). Clear the commanded state and
  // the consumer's last-dispatched mask so the first post-reset M7/M8 is seen as a fresh change.
  force_coolant_off();
  state.last_coolant = 0;
  reset_ov_reporter();
  // Phase F: a soft reset re-seeds the auto-report cadence from the (post-reset) live `$481` and clears the
  // `0x8C` runtime suspend, so auto-reporting returns to its configured state. Wake the task so it re-arms (or
  // parks) against the refreshed interval immediately. The setter applied any pre-reset `$481=` to the live
  // settings already, so this picks up the latest value (grbl applies most settings on the next reset).
  let auto_report_interval = settings_snapshot().await.auto_report_interval_ms();
  init_auto_report(auto_report_interval);
  AUTO_REPORT_WAKE.signal(());
  // grbl drops the non-persistent `G38.2` probe data on a soft reset, like G92/TLO; clear the last-probe slot so
  // `$#` reports `[PRB:0,0,0:0]` after a reset until the next probe.
  set_last_probe(LastProbe::none());
  {
    let mut snap = MACHINE.lock().await;
    *snap = MachineSnapshot::idle();
  }
  // The authoritative reset banner, delivered guaranteed (the reader half's best-effort `try_send` may have
  // dropped its copy under a momentarily full RESPONSE channel). A host treats this as "reset and ready".
  send_banner().await;
}

/// Route one accepted line. `$` system commands are dispatched to their report handlers and clear the
/// error-hold (a recovery trigger). Otherwise the line is parsed and planned. Exactly one `ok`/`error:N`
/// is emitted per call, preserving the one-response-per-line contract.
async fn handle_line(line: &[u8], parser: &mut Parser, state: &mut ConsumerState, flash: &'static SharedFlash) {
  let trimmed = trim_ascii(line);
  if trimmed.is_empty() {
    // A blank line (forwarded by `usb_rx`) is a grblHAL error-hold recovery trigger: clear the hold and
    // acknowledge with a bare `ok`. Handling it here, in queue order, keeps recovery race-free with the
    // surrounding lines, since the hold lives in this task, the sole in-order reader of `LINE_QUEUE`.
    state.error_hold = false;
    ack().await;
  } else if let Some(jog_line) = strip_jog_prefix(trimmed) {
    // `$J=` is a MOTION command, not a `$` setting — route it BEFORE the `$`-system dispatch (Phase D). Like a
    // GCode motion line it does NOT clear the error-hold by itself (only a blank line, a real `$` command, or a
    // soft reset do); `handle_jog` honors the hold and the control-state gating, and emits exactly one ok/error.
    handle_jog(jog_line, parser, state).await;
  } else if let Some(rest) = trimmed.strip_prefix(b"$") {
    // A `$` system command is the other grblHAL recovery trigger and is answered by its handler. Both
    // recovery triggers (an empty line and a `$` command) and a soft reset clear the hold; nothing else.
    // The parser is passed mutably so `$G` can report live modal state AND so `$C`-off / `$RST=$` can rebuild
    // it as part of their soft reset. `flash` lets `$RST=$` persist the restored defaults.
    state.error_hold = false;
    handle_system_command(rest, parser, state, flash).await;
  } else {
    plan_gcode_line(trimmed, parser, state, flash).await;
  }
}

/// Parse and plan one GCode line, emitting exactly one `ok`/`error:N`. Honors the error-hold: while held,
/// a GCode line is rejected without parsing. On a parse or planner error the hold is armed; on acceptance
/// (including modal-only `Ok(None)` lines) a single `ok` is emitted.
async fn plan_gcode_line(
  line: &[u8],
  parser: &mut Parser,
  state: &mut ConsumerState,
  flash: &'static SharedFlash,
) {
  // Pre-parse gating (error-hold reject / stale-`Jog` refresh / alarm-sleep-jog lockout): a gated line already
  // emitted its single reject response, so stop; otherwise `control` is the once-sampled state reused below for
  // the check-mode guard and the modal-drive gate.
  let Some(control) = gate_line(state).await else {
    return;
  };
  let parsed = parser.parse_line(line);
  // Modal side-effects on a clean parse (sync active WCS, then — outside check mode — publish programmed RPM and
  // drive the spindle/coolant tasks from modal state). Inspects `parsed` by reference so the dispatch match
  // below still owns it; runs before the match exactly as the inline block did.
  drive_modal_outputs(parser, state, &parsed, control).await;
  match parsed {
    // A blank/comment-only/modal-only line carries no action; acknowledge with a single `ok`.
    Ok(None) => ack().await,
    // `$C` check mode: the line parsed and validated cleanly, but check mode must NOT plan or execute it —
    // grbl `ok`s it so a host can verify a whole file without moving. The modal state still advanced in the
    // parser (correct: check mode tracks modal state), but no block is enqueued.
    Ok(Some(_)) if control == ControlState::Check => ack().await,
    Ok(Some(command)) => {
      // DOC-07 spin-up dwell: before the FIRST cutting move after an M3/M4, insert a synchronized `$392` dwell so
      // the spindle reaches speed before it cuts. The host-tested `SpinUpGate` decides WHEN; this injects the
      // dwell ahead of the move. `inject_spin_up_dwell` is a no-op (and returns Continue) for a rapid / non-move
      // command or when no spin-up is owed; it returns Aborted only if a soft reset preempted the awaited dwell.
      match inject_spin_up_dwell(&command, state).await {
        SpinUpInjection::Continue => {}
        // A soft reset preempted the awaited spin-up dwell: run the warm reset and drop the move.
        SpinUpInjection::Aborted => {
          apply_soft_reset(parser, state).await;
          return;
        }
        // A graceful program stop preempted the spin-up dwell's back-pressured enqueue: run the clean stop (Idle,
        // position retained, no alarm) and drop the move.
        SpinUpInjection::Stopped => {
          program_stop_cycle(parser, state).await;
          return;
        }
      }
      // G28/G30 (DOC-05 group-0 motion) is intercepted here, ahead of the generic `plan_command`: it must read the
      // stored predefined position from the consumer-owned coordinate model, so it cannot be planned by the planner
      // alone. `handle_go_to_predefined` runs the same lock + back-pressure + soft-limit flow and returns a
      // `PlanResult` the shared match below acts on (a single `ok`, or the soft-reset/soft-limit paths).
      let result = match &command {
        firmware_core::gcode::PlannerCommand::GoToPredefined { is_g28, intermediate, units, distance } => {
          handle_go_to_predefined(*is_g28, intermediate, *units, *distance).await
        }
        _ => plan_command(&command).await,
      };
      dispatch_plan_result(result, parser, state, flash).await;
    }
    Err(e) => {
      // A parse error: emit `error:N` and arm the gcode error-hold so subsequent GCode lines are held.
      error(e.code()).await;
      state.error_hold = true;
    }
  }
}

/// Pre-parse gating for one GCode line (extracted from [`plan_gcode_line`], B1). Applies, in order, the gcode
/// error-hold reject, the stale-`Jog` latch refresh, and the alarm/sleep/jog state lockout — each emitting the
/// single reject response and stopping. Returns `None` when the line was gated (the caller returns without
/// parsing) or `Some(control)` — the once-sampled [`ControlState`] the caller reuses for the check-mode guard
/// and the modal-drive gate.
async fn gate_line(state: &ConsumerState) -> Option<ControlState> {
  if state.error_hold {
    // Held by a prior error: reject without parsing until a recovery trigger. Reuse the generic
    // "expected command letter" code, matching how a sender already in error-recovery treats any further
    // rejection — it halts the stream regardless of the specific code (mirrors the engine's hold code).
    // Emit it bare: the held line may be perfectly valid GCode, so the code's "Expected command letter"
    // name does not describe the rejection and a `[MSG:..]` annotation would mislead a plain terminal.
    error_bare(ERROR_HOLD_CODE).await;
    return None;
  }
  // Drop a stale `Jog` latch back to Normal if the jog has fully drained, so a program line after a jog finishes
  // is accepted (and `?` reads Idle). If a jog is still ACTIVE this leaves the state `Jog` and the line is
  // rejected below — a program move never blends into an in-flight jog.
  refresh_jog_state().await;
  // In an alarm or sleep, GCode is blocked entirely (motion not allowed and not even parsed for `ok`): reject
  // with the unsupported-command code so a sender halts. Boot-lock (`$22` homing required) lands here too,
  // forcing the host to `$H`/`$X` before streaming. An active jog (`Jog`) likewise blocks program GCode (grbl:
  // the machine is busy jogging). Check mode is handled below (parse + `ok`, no plan).
  let control = control_state();
  if matches!(control, ControlState::Alarm(_) | ControlState::Sleep | ControlState::Jog) {
    // grbl's "G-code locked out during alarm or jog state" is `error:9`, NOT the generic post-error hold code
    // (Finding #5). Use the dedicated `ERROR_LOCKED` so a sender's display matches the `$EE` table (which
    // already defines code 9) and distinguishes a state lockout from an in-stream parse-error hold.
    error(ERROR_LOCKED).await;
    return None;
  }
  Some(control)
}

/// Drive the modal side-effects of a cleanly-parsed line (extracted from [`plan_gcode_line`], B1): keep the
/// coordinate model's active WCS in step with the parser's modal `wcs`, then — on any clean parse and only
/// outside `$C` check mode — publish the programmed spindle RPM and drive the spindle/coolant tasks from modal
/// state. Pure side-effects (no response), keyed off modal state so an M3/M4/M5 or M7/M8/M9 sharing a line with
/// a move still takes effect. `control` is the caller's once-sampled state; `parsed` is inspected by reference
/// so the caller still owns it for the dispatch match.
async fn drive_modal_outputs(
  parser: &Parser,
  state: &mut ConsumerState,
  parsed: &Result<Option<firmware_core::gcode::PlannerCommand>, firmware_core::gcode::GcodeError>,
  control: ControlState,
) {
  // Keep the active WCS in the coordinate model in sync with the parser's modal `wcs` BEFORE planning, so a
  // move sharing a line with a G54-G59 select (e.g. `G55 G0 X1`) uses the right offset — the parser emits the
  // move (not a SelectWcs op) in that case, so the WCS change would otherwise be missed until a bare select.
  if matches!(parsed, Ok(Some(_))) {
    sync_active_wcs(parser.state().wcs).await;
  }
  // Phase E / DOC-07: on any clean parse, publish the parser's modal `S` as the PROGRAMMED spindle RPM (so
  // `status_responder` renders the override-scaled `FS:` and the spindle task reads the new speed), THEN drive the
  // spindle task from the modal spindle direction. Doing it here — keyed off MODAL state, not the per-line emit —
  // is what makes an M3/M4/M5 take effect on a line that ALSO carries a move (the emit is the move, not a `Spindle`
  // command) and lets a bare `S` re-drive a running spindle. The modal `S`/direction persist across lines.
  if parsed.is_ok() {
    let modal = parser.state();
    let rpm = modal.spindle_speed.max(0.0) as u32;
    PROGRAMMED_SPINDLE_RPM.store(rpm.min(u16::MAX as u32), Ordering::Release);
    // Drive the spindle from modal state — but NOT in `$C` check mode: a dry run must validate a program without
    // actuating any output (grbl check mode moves nothing and does not energize the spindle). Publishing the
    // PROGRAMMED RPM above is report-only and safe; `sync_spindle_from_modal` actuates the LEDC/SPIN_EN hardware,
    // so it is gated here. The check guard in the `match` below only suppresses planning, which is too late.
    if control != ControlState::Check {
      sync_spindle_from_modal(modal, state);
      // Drive coolant from modal state too (M7/M8/M9), gated off check mode for the same reason: a dry run must
      // validate the program without actuating any output. Keying off modal state makes an M7/M8 sharing a line
      // with a move actuate coolant (the per-line emit is the move), mirroring the spindle.
      sync_coolant_from_modal(modal, state);
    }
  }
}

/// Dispatch a planned command's [`PlanResult`] to its response and side-effects (extracted from
/// [`plan_gcode_line`], B1): the outcome match — a single `ok` for an accepted / pass-through / already-driven
/// (spindle/coolant) outcome, the coordinate-op apply, the geometry error-hold, the soft-limit / motion alarms,
/// the synchronized dwell / program-end / M0-M1-M6 pause holds, and the `G38.x` probe cycle — plus the
/// soft-reset (`0x18`) and graceful-stop (`0x86`) preemption paths. `parser`/`state`/`flash` thread through for
/// the arms that rebuild the pipeline, read the modal tool, or persist.
async fn dispatch_plan_result(
  result: PlanResult,
  parser: &mut Parser,
  state: &mut ConsumerState,
  flash: &'static SharedFlash,
) {
  match result {
    // The command was accepted into the planner (a move enqueued, or a non-motion outcome passed
    // through); emit the single `ok`.
    PlanResult::Accepted => ack().await,
    // A coordinate-system / offset op: apply it to the shared coordinate model (against the live machine
    // position), push the new WCO into the planner, and persist the persistent subset, then `ok`.
    PlanResult::Coordinate(op) => {
      apply_coordinate_op(op).await;
      ack().await;
    }
    // The planner reported a non-back-pressure error (bad arc geometry); reject and arm the hold.
    PlanResult::Error(code) => {
      error(code).await;
      state.error_hold = true;
    }
    // A program move left the `$20` soft-limit envelope: enter the soft-limit alarm and emit `ALARM:2` (no
    // `ok`). The block was rejected before any motion, so position is intact, but grbl halts the program; a
    // soft reset / `$X` clears the alarm. The alarm latches so subsequent GCode is gated until cleared.
    PlanResult::SoftLimitAlarm => {
      set_control_state(ControlState::Alarm(AlarmCode::SoftLimit));
      emit_alarm(AlarmCode::SoftLimit).await;
    }
    // A soft reset arrived while this command was back-pressured: abort it (the host discards pending
    // acks on `0x18`), emit no response, and run the soft-reset transition whose signal was consumed here.
    PlanResult::Aborted => apply_soft_reset(parser, state).await,
    // A graceful program stop (`0x86`) arrived while this command was back-pressured: abandon the line (no
    // `ok` — the host discards pending acks the moment it sends the stop) and run the clean stop whose
    // [`PROGRAM_STOP`] signal was consumed in the back-pressure wait. Returns to Idle with position retained,
    // no alarm — unlike `Aborted`'s warm reset. The `program_stop_cycle` re-checks the live state, so a stop
    // that raced a soft reset (which already moved us out of Run/Hold) is a benign no-op.
    PlanResult::Stopped => program_stop_cycle(parser, state).await,
    // An M3/M4/M5 (DOC-07): the spindle outputs were ALREADY driven from the modal spindle state by
    // `sync_spindle_from_modal` (above, on the clean parse), so a spindle-only line just `ok`s here. Driving
    // off modal state — not this per-line outcome — is what makes an M3/M4/M5 sharing a line with a move work.
    PlanResult::Spindle(_spindle_state, _rpm) => ack().await,
    // A `G4` dwell: run the synchronized dwell (drain motion, then hold), then `ok`. A soft reset mid-dwell
    // abandons it and runs the reset (the consumed signal must be honored), emitting no `ok`.
    PlanResult::Dwell(seconds) => {
      if run_dwell(seconds).await {
        ack().await;
      } else {
        apply_soft_reset(parser, state).await;
      }
    }
    // An `M30` program end: drain motion, stop the spindle, reset modal state, then `ok`. A soft reset while
    // draining abandons the end and runs the reset instead.
    PlanResult::ProgramEnd => {
      if program_end(parser, state).await {
        ack().await;
      } else {
        apply_soft_reset(parser, state).await;
      }
    }
    // An M0/M1/M6 program-flow pause: drain motion and hold until cycle-start (`~`), then `ok`. The `ok`
    // follows normal char-counting — it is emitted when the pause COMPLETES (after the resume), exactly like
    // a dwell, so the host's send-ahead window naturally stalls while paused. `run_program_pause` returns
    // `Resumed` on cycle-start, or a preemption (soft reset / graceful stop) the caller honors with no `ok`.
    PlanResult::ProgramPause { optional, tool_change } => {
      // The committed CURRENT tool (M6 commits pending->current before this point) names the tool-change
      // prompt so a bare-terminal operator knows which tool to insert.
      let current_tool = parser.state().current_tool;
      match run_program_pause(optional, tool_change, current_tool, parser, state, flash).await {
        PauseOutcome::Resumed => ack().await,
        PauseOutcome::Skipped => ack().await,
        PauseOutcome::Aborted => apply_soft_reset(parser, state).await,
        PauseOutcome::Stopped => program_stop_cycle(parser, state).await,
      }
    }
    // An M7/M8/M9 coolant command: the coolant outputs were ALREADY driven from the modal coolant state by
    // `sync_coolant_from_modal` (on the clean parse), so a coolant-only line just `ok`s here — mirroring how
    // a spindle-only line is handled. Driving off modal state makes an M7/M8 sharing a line with a move work.
    PlanResult::Coolant(_state) => ack().await,
    // A `G38.x` probe: run the probe-watching cycle on the core-1 executor and decide the response from the
    // outcome and the mode's alarm-on-fail flag.
    PlanResult::Probe { request, alarm_on_fail } => {
      handle_probe(request, alarm_on_fail, parser, state).await;
    }
  }
}

/// The result of attempting to plan one command (collapsing the planner's back-pressure retry loop and the
/// soft-reset abort into one outcome the line handler acts on).
pub(crate) enum PlanResult {
  /// The command was accepted into the planner buffer (move enqueued or non-motion outcome passed through).
  Accepted,
  /// A coordinate-system / offset op passed through the planner; the consumer applies it to the shared
  /// [`COORDINATES`] model (resolving any "set to current position" op against the live machine position),
  /// pushes the recomputed WCO into the planner, and persists the persistent subset. Carried out of
  /// `plan_command` so the apply happens with the live machine position in hand.
  Coordinate(CoordinateOp),
  /// A non-back-pressure planner error; carries the grblHAL `error:N` code (bad arc geometry).
  Error(u8),
  /// A program move/arc exceeded the `$20` soft-limit envelope (DOC-06). grbl halts and raises `ALARM:2`
  /// (position is NOT lost — the move was rejected before any motion — but the program cannot continue). The
  /// consumer enters the soft-limit alarm and emits `ALARM:2`, no `ok`.
  SoftLimitAlarm,
  /// A soft reset preempted the command while it was back-pressured; the consumed signal must be honored.
  Aborted,
  /// A graceful program stop (`0x86`) preempted the command while it was back-pressured (or driving a pending
  /// arc): the consumed [`PROGRAM_STOP`] signal must be honored by running [`program_stop_cycle`]. Distinct from
  /// [`Aborted`](PlanResult::Aborted) — a stop returns to Idle with position retained and raises no alarm, where
  /// the `0x18` abort runs the warm reset. The in-flight line is abandoned with no `ok` (the host discards pending
  /// acks the moment it sends the stop).
  Stopped,
  /// An M3/M4/M5 spindle command (DOC-07): the planner passed it through with no motion. The consumer publishes
  /// the commanded direction, wakes the [`spindle`] task to drive the outputs, and notes the spin-up gate so the
  /// next cutting move gets a `$392` dwell. Carried out of `plan_command` so the side effects run in the
  /// consumer task (which owns the spin-up gate state and the direction/wake signals).
  Spindle(SpindleState, f32),
  /// A `G4` dwell (DOC). The planner has flushed look-ahead (the preceding block stops); the consumer runs the
  /// synchronized dwell ([`run_dwell`]) — wait for motion to drain, then hold for the dwell seconds — so the dwell
  /// blocks the stream like grbl's buffer-synchronize. Carried out of `plan_command` so the timed wait runs in the
  /// consumer task (which owns the stream).
  Dwell(f32),
  /// An `M30` program end. The planner has flushed look-ahead; the consumer drains motion, stops the spindle, and
  /// resets the parser's modal state to power-on defaults (grbl's M30 reset). Carried out of `plan_command` so the
  /// drain wait + spindle/modal reset run in the consumer task.
  ProgramEnd,
  /// An M0/M1/M6 program-flow pause. The planner flushed look-ahead; the consumer drains motion and holds until
  /// cycle-start (`~`), reusing the graceful-hold machinery. `optional` flags M1 (the optional-stop gate decides
  /// whether it actually halts); `tool_change` flags M6 (the consumer prompts the operator to swap the tool).
  /// Carried out of `plan_command` so the drain + hold-await run in the consumer task that owns the stream.
  ProgramPause {
    /// True for M1 (optional stop): only halts when the optional-stop toggle ([`OPTIONAL_STOP_ENABLED`]) is on.
    optional: bool,
    /// True for M6 (manual tool change): the consumer emits a `[MSG:..]` swap prompt before holding.
    tool_change: bool,
  },
  /// An M7/M8/M9 coolant command. The planner passed it through with no motion; the consumer drives the
  /// (hardware-gated) coolant outputs from the carried [`CoolantState`]. Mirrors [`Spindle`](PlanResult::Spindle).
  Coolant(firmware_core::gcode::CoolantState),
  /// A `G38.x` probe (Phase C): the planner resolved the machine target and flushed look-ahead. The consumer
  /// runs the probe-watching cycle on the core-1 executor (carrying the resolved request + the alarming sense),
  /// syncs the planner position to the stop point, emits `[PRB:]`, and decides ALARM/ok.
  Probe {
    /// The request handed to the core-1 executor (target / period / toward sense).
    request: ProbeRequest,
    /// `true` for the alarming modes (G38.2/.4): a no-trigger outcome raises ALARM:4/5. `false` for G38.3/.5.
    alarm_on_fail: bool,
  },
}

/// The `error:N` code surfaced when the shared planner was never installed (an init wiring bug). grblHAL's
/// "setting disabled" code 3 is reused as a distinct, loud failure so a misconfigured build fails the line
/// instead of fabricating an `ok` for motion that will never run. Unreachable in a correctly wired build.
const ERROR_PLANNER_UNINITIALIZED: u8 = 3;

/// Feed one [`PlannerCommand`](firmware_core::gcode::PlannerCommand) to the shared planner, applying
/// back-pressure: on [`PlannerError::QueueFull`] wait for the core-1 motion executor to free a block and
/// retry the SAME command (the arc planner is all-or-nothing on `QueueFull`, so re-issue is safe). On an
/// accepted motion outcome it raises [`BLOCK_AVAILABLE`] to wake the executor. The back-pressure wait is
/// raced against [`SOFT_RESET`] so a `0x18` aborts a stuck line promptly rather than after the executor frees
/// a slot. Non-motion outcomes are surfaced as distinct [`PlanResult`]s for the consumer to act on: `Spindle`
/// (DOC-07), `Dwell` (the synchronized `G4`), `ProgramEnd` (`M30`), and `Coordinate` (G10/G54-G59/G92). Still
/// passing through as `Accepted` (no side effect): the G28/G30 predefined move (real system motion, a DOC-06
/// follow-up) and a zero-block no-op `Queued`.
pub(crate) async fn plan_command(command: &firmware_core::gcode::PlannerCommand) -> PlanResult {
  loop {
    // Scope the lock so it is released before any await: hold the planner mutex only for the plan call. A
    // missing planner (an init wiring bug, unreachable in a correctly wired build — see `init_planner`) is
    // surfaced as a distinct internal `error:N` rather than a fabricated `ok`: a silent accepted-but-un-run
    // move would hide the bug, so we fail the line loudly instead.
    // `$20` soft limits are checked at PLAN time, but only when enabled AND the machine is homed (a known
    // machine zero is what makes the envelope meaningful — research finding #16). `current_soft_limits` returns
    // the envelope only when both hold; otherwise `None` skips the check (identical to the pre-DOC-06 behavior).
    let limits = current_soft_limits().await;
    let result = {
      let mut guard = PLANNER.lock().await;
      match guard.as_mut() {
        Some(planner) => planner.plan_command_with_limits(command, limits),
        None => return PlanResult::Error(ERROR_PLANNER_UNINITIALIZED),
      }
    };
    match result {
      // A motion command enqueued at least one block: wake the core-1 `motion_executor` so it can drain the
      // freshly queued block(s) instead of polling. A coalesced signal is fine — the executor re-checks the
      // queue under the lock and loops until empty, so multiple blocks behind one signal are all consumed.
      Ok(PlannerOutcome::Queued { blocks }) if blocks > 0 => {
        BLOCK_AVAILABLE.signal(());
        return PlanResult::Accepted;
      }
      // An over-subdivided arc fed only PART of its segments — the rest are saved as an in-progress arc (DOC-05
      // resumable arc). The line must NOT be acked yet: wake the executor to drain the chunk we just queued, then
      // drive `resume_arc` until the whole arc is enqueued. This is what lets an arc with more than the queue's
      // worth of segments stream without ever dead-locking on a permanent `QueueFull`. A soft reset mid-arc aborts
      // it (the executor's reset clears the queue and `abort_arc` drops the in-progress arc).
      Ok(PlannerOutcome::ArcPending { enqueued }) => {
        if enqueued > 0 {
          BLOCK_AVAILABLE.signal(());
        }
        return drive_pending_arc().await;
      }
      // A coordinate-system / offset op: surface it so the consumer applies it to the shared coordinate model
      // with the live machine position in hand, then pushes the recomputed WCO back into the planner.
      Ok(PlannerOutcome::Coordinate(op)) => return PlanResult::Coordinate(op),
      // A `G38.x` probe (Phase C): the planner resolved the machine target and flushed look-ahead. Derive the
      // fixed probe-feed step period from the seek feed + steps/mm, and hand the request back to the consumer to
      // run on the core-1 executor (which watches the probe and stops on the edge).
      Ok(PlannerOutcome::Probe { kind, target, feed, units }) => {
        let step_period_ticks = probe_step_period_ticks(&target, feed, units).await;
        let invert = settings_snapshot().await.probe_config().invert;
        return PlanResult::Probe {
          request: ProbeRequest { target, step_period_ticks, toward: kind.toward, invert },
          alarm_on_fail: kind.alarm_on_fail,
        };
      }
      // An M3/M4/M5 spindle command (DOC-07): the planner passes it through with no motion. Surface it so the
      // consumer drives the spindle task and notes the spin-up gate (it owns that state); doing the side effects
      // here in `plan_command` would scatter them away from the gate/error-hold owner.
      Ok(PlannerOutcome::Spindle(state, rpm)) => return PlanResult::Spindle(state, rpm),
      // A `G4` dwell: the planner flushed look-ahead (pinning the preceding block to a stop — the synchronized
      // boundary); the consumer runs the timed [`run_dwell`] wait. Surfaced so the wait runs in the consumer task.
      Ok(PlannerOutcome::Dwell { seconds }) => return PlanResult::Dwell(seconds),
      // `M30` program end: the planner flushed look-ahead; the consumer drains motion, stops the spindle, and
      // resets modal state. Surfaced so those side effects run in the consumer task.
      Ok(PlannerOutcome::ProgramEnd) => return PlanResult::ProgramEnd,
      // M0/M1/M6 program-flow pause: the planner flushed look-ahead (the pause is a synchronized boundary, like a
      // dwell). Surface it so the consumer drains motion and runs the hold-until-cycle-start cycle in its own task.
      Ok(PlannerOutcome::ProgramPause { optional, tool_change }) => {
        return PlanResult::ProgramPause { optional, tool_change };
      }
      // An M7/M8/M9 coolant command: the planner passes it through with no motion. Surface it so the consumer drives
      // the coolant task off the modal state, mirroring how the spindle outcome is handled.
      Ok(PlannerOutcome::Coolant(state)) => return PlanResult::Coolant(state),
      // G28/G30 is NOT planned through this generic path — the consumer intercepts it before `plan_command`
      // (see `handle_go_to_predefined`) because it must read the stored predefined position from the coordinate
      // model, which lives in the consumer, not the planner. The pass-through `GoToPredefined` outcome is therefore
      // unreachable here; a zero-block `Queued { blocks: 0 }` no-op move still falls here and needs no executor wake.
      Ok(_outcome) => return PlanResult::Accepted,
      // Back-pressure: the planner buffer is full. Do NOT ack and do NOT drop — yield to the motion
      // executor, then retry the same command. Blocking here backs `LINE_QUEUE` up and throttles the host (correct
      // grbl flow control). Race the retry delay against a soft reset so `0x18` aborts a stuck line at once;
      // the delay is short relative to a block's execution time, so a normal retry wins the freed slot
      // promptly without busy-spinning the CPU.
      Err(PlannerError::QueueFull) => {
        // Back-pressure: the consumer yields to the executor and retries. This park is TIMER-BACKED (it self-wakes
        // every `QUEUE_FULL_RETRY`), so it cannot wedge here — the marker is for completeness so a wedge that LOOKS
        // like back-pressure is distinguishable. (Motion idle ⇒ this is NOT the current wedge's site.)
        crate::crash::record_comms_stage(crate::crash::CommsTask::Consumer, crate::crash::CommsStage::ConsumerPlanBackpressure);
        // Race the retry delay against a soft reset (`0x18` → abort) AND a graceful program stop (`0x86` → clean
        // stop): both abandon a stuck back-pressured line at once rather than after the executor frees a slot.
        match select(Timer::after(QUEUE_FULL_RETRY), select(SOFT_RESET.wait(), PROGRAM_STOP.wait())).await {
          Either::First(()) => {}
          Either::Second(Either::First(())) => return PlanResult::Aborted,
          Either::Second(Either::Second(())) => return PlanResult::Stopped,
        }
      }
      // A program move/arc that left the `$20` soft-limit envelope: this is a SYSTEM ALARM in grbl (`ALARM:2`),
      // not an `error:N` line — the planner rejected the block before any enqueue, so no motion started. Surface
      // it as a distinct result the caller routes to the alarm path.
      Err(PlannerError::MoveExceedsTravel) => return PlanResult::SoftLimitAlarm,
      // A genuine geometry error (bad arc): surface the grblHAL code to the caller.
      Err(other) => return PlanResult::Error(other.code()),
    }
  }
}

/// Drive an in-progress (over-subdivided) arc to completion, feeding its remaining segments into the planner as
/// the core-1 motion executor frees queue slots (DOC-05 resumable arc). The first chunk has ALREADY been enqueued
/// by [`plan_command`]; this loops [`Planner::resume_arc`](firmware_core::planner::Planner::resume_arc) — waking
/// the executor after each chunk and yielding for it to drain — until the whole arc is enqueued, at which point the
/// line is acked exactly ONCE ([`PlanResult::Accepted`]). It NEVER acks while the arc is still pending, so the
/// host's character-counting flow control throttles correctly and the stream can never dead-lock on an arc larger
/// than the queue. The resume wait is raced against [`SOFT_RESET`] so a `0x18` aborts a stuck arc at once — the
/// executor's reset clears the queue and the planner rebuild drops the in-progress arc, so the abort is clean.
///
/// ## Proactive (event-driven) refill — Bug 4
/// The refill is woken by [`SLOT_FREED`] (raised the instant the executor pops a block) raced against a short
/// [`QUEUE_FULL_RETRY`] timer backstop and [`SOFT_RESET`]. Refilling the moment a slot opens — rather than only
/// after the full poll interval — keeps the planner buffer topped up while the arc is pending, so the executor
/// never drains down to the look-ahead's forced-stop chunk tail before the next chunk lands. That is what makes a
/// large arc execute as CONTINUOUS motion across chunk boundaries (proven host-side in
/// `over_subdivided_arc_carries_velocity_across_chunk_boundaries`) instead of stamping a decelerate-to-stop dwell
/// mark at each ~`BLOCK_QUEUE_LEN` boundary. RESIDUAL: the genuine last available block always decelerates to a
/// stop (the executor must be able to halt there — a hard safety invariant); at an extreme feed where the
/// executor could empty the queue between a pop and the refill completing, motion would still momentarily stop —
/// safe, never a step loss — but at realistic PCB-milling feeds/segment timing the producer stays ahead and the
/// curve is smooth.
///
/// ## Termination — Bug 9
/// The loop exits ONLY on `Queued` (the arc completed), `SOFT_RESET`, or an `Err` from `resume_arc`. A genuine
/// (non-`QueueFull`) per-segment error now PROPAGATES out of `resume_arc` with the in-progress arc cleared, so the
/// `Err(other)` arm returns `error:N` and the loop ends — a deterministic segment error can no longer spin here
/// forever. A `resume_arc` that enqueues zero (the queue is still full) is benign: it simply waits for the next
/// `SLOT_FREED`/timer wake, and the loop PROGRESSES because each executor pop frees a slot the next resume claims.
async fn drive_pending_arc() -> PlanResult {
  loop {
    // Refill on the executor's "slot freed" wake the instant it pops a block (proactive refill, Bug 4), with a
    // short timer backstop so a foregone signal (e.g. the executor parked on a hold) cannot wedge the loop, and a
    // soft reset so `0x18` aborts at once. The timer is short relative to a block's execution time, so even on the
    // backstop path a freed slot is claimed promptly.
    // Also race a graceful program stop (`0x86`): a stop arriving mid-arc-drive abandons the remaining segments at
    // once (the stop's `abort_arc` + `flush_queue` drops the in-progress arc and the queued chunks) and runs the
    // clean stop, exactly as `0x18` runs the abort. Reported as `Stopped` so the caller runs `program_stop_cycle`.
    match select(select(SLOT_FREED.wait(), Timer::after(QUEUE_FULL_RETRY)), select(SOFT_RESET.wait(), PROGRAM_STOP.wait())).await {
      Either::First(_) => {}
      Either::Second(Either::First(())) => return PlanResult::Aborted,
      Either::Second(Either::Second(())) => return PlanResult::Stopped,
    }
    // Feed the next chunk under the planner lock (scoped so it is dropped before any await). A missing planner is
    // a wiring bug surfaced loudly rather than fabricating an `ok`, exactly as `plan_command` does.
    let outcome = {
      let mut guard = PLANNER.lock().await;
      match guard.as_mut() {
        Some(planner) => planner.resume_arc(),
        None => return PlanResult::Error(ERROR_PLANNER_UNINITIALIZED),
      }
    };
    match outcome {
      // The final chunk is in: every segment is enqueued, so wake the executor for the last blocks and ack once.
      Ok(PlannerOutcome::Queued { blocks }) => {
        if blocks > 0 {
          BLOCK_AVAILABLE.signal(());
        }
        return PlanResult::Accepted;
      }
      // More segments fed (or none yet, if no slot freed): wake the executor for whatever we just queued and loop.
      Ok(PlannerOutcome::ArcPending { enqueued }) => {
        if enqueued > 0 {
          BLOCK_AVAILABLE.signal(());
        }
      }
      // `resume_arc` only ever returns an arc outcome on success; any other Ok variant is an invariant break,
      // surfaced loudly rather than silently acking a half-fed arc.
      Ok(_) => return PlanResult::Error(ERROR_PLANNER_UNINITIALIZED),
      // A genuine per-segment planner error (Bug 9): `resume_arc` has already cleared the in-progress arc, so we
      // surface `error:N` and STOP driving — the deterministic error can never spin this loop forever.
      Err(other) => return PlanResult::Error(other.code()),
    }
  }
}

/// Plan a `G28`/`G30` predefined-position recall (DOC-05 group-0 motion) into the planner. Unlike the generic
/// [`plan_command`] path this reads the stored predefined position from the consumer-owned coordinate model (index
/// 0 = G28, 1 = G30; a never-stored slot defaults to the machine origin, grbl's default) and hands it to the
/// host-tested [`Planner::plan_go_to_predefined`], which sequences the optional work-coordinate intermediate rapid
/// and the absolute machine-coordinate recall rapid. It mirrors `plan_command`'s flow exactly: the planner mutex is
/// scoped so it is dropped before any await, the `$20` soft-limit envelope is supplied (so an out-of-envelope
/// intermediate alarms like any rapid), `QueueFull` back-pressure yields to the executor and retries the WHOLE
/// call, and a `0x18` soft reset mid-retry aborts the line. Retrying the whole call is safe because
/// `plan_go_to_predefined` is ATOMIC: it enqueues NEITHER sub-move unless BOTH fit, so a retry always re-resolves
/// from the original, un-advanced position — critical in INCREMENTAL (G91) mode, where a partial enqueue would
/// otherwise re-apply the intermediate's increment a second time (double motion).
async fn handle_go_to_predefined(is_g28: bool, intermediate: &firmware_core::gcode::AxisWords, units: GcodeUnits,
  distance: GcodeDistance) -> PlanResult {
  // Index 0 is the G28 home, 1 is the G30 secondary; a never-stored slot reads as the machine origin (grbl default).
  let predefined = coordinates().predefined(if is_g28 { 0 } else { 1 }).unwrap_or([0.0; AXES]);
  loop {
    let limits = current_soft_limits().await;
    let result = {
      let mut guard = PLANNER.lock().await;
      match guard.as_mut() {
        Some(planner) => planner.plan_go_to_predefined(intermediate, units, distance, predefined, limits),
        None => return PlanResult::Error(ERROR_PLANNER_UNINITIALIZED),
      }
    };
    match result {
      // At least one rapid enqueued: wake the core-1 executor to drain it. A zero-block no-op (already at the
      // stored position with no intermediate words) needs no wake — fall through to `Accepted` either way.
      Ok(blocks) => {
        if blocks > 0 {
          BLOCK_AVAILABLE.signal(());
        }
        return PlanResult::Accepted;
      }
      // Back-pressure: yield to the executor and retry the whole call. The call is atomic (enqueues nothing unless
      // both sub-moves fit), so the retry re-resolves from the un-advanced position — see the doc comment. Race the
      // retry against a soft reset (`0x18` → abort) AND a graceful program stop (`0x86` → clean stop) so either
      // abandons a stuck recall at once.
      Err(PlannerError::QueueFull) => {
        match select(Timer::after(QUEUE_FULL_RETRY), select(SOFT_RESET.wait(), PROGRAM_STOP.wait())).await {
          Either::First(()) => {}
          Either::Second(Either::First(())) => return PlanResult::Aborted,
          Either::Second(Either::Second(())) => return PlanResult::Stopped,
        }
      }
      // The intermediate left the `$20` envelope: a SYSTEM ALARM (`ALARM:2`), not an `error:N` line — no motion
      // started (the planner rejected before any enqueue). The recall point is within-envelope by construction.
      Err(PlannerError::MoveExceedsTravel) => return PlanResult::SoftLimitAlarm,
      // No other error is reachable from `plan_go_to_predefined` (it plans only rapids, no arc geometry); surface
      // any future one as its grblHAL code rather than fabricating an `ok`.
      Err(other) => return PlanResult::Error(other.code()),
    }
  }
}

/// Apply a coordinate-system / offset op to the shared [`COORDINATES`] model, then push the recomputed WCO into
/// the planner and (for the PERSISTENT ops) mark the coordinate record dirty. The "set to current position" ops
/// (G10 L20, G92, G28.1/G30.1) resolve against the planner's COMMANDED machine position (`position_mm`), which
/// is grbl's `gc_state.position` — the right reference for setting offsets, and race-free with the cross-core
/// live-position atomics. Inch words are scaled to mm here (the parser passes raw values in the active units).
async fn apply_coordinate_op(op: CoordinateOp) {
  // Snapshot the commanded machine position once (used by the set-to-position ops). A missing planner is an
  // init bug, unreachable in a wired build; treat it as the origin so the op still applies coherently.
  let machine = {
    let guard = PLANNER.lock().await;
    guard.as_ref().map(Planner::position_mm).unwrap_or([0.0; AXES])
  };
  // The live `$376` rotary mask, so a rotary-A coordinate word is stored as degrees (never inch-scaled) — matching
  // the planner's per-axis scale fork. Read once here for every `axis_values_mm` call in the match below.
  let rotary_mask = settings_snapshot().await.rotary_mask;
  let mut coords = coordinates();
  // Whether this op changes the PERSISTENT subset (G54-G59 / G28 / G30 / active WCS) and so must be flushed.
  // G92 and the dynamic TLO are session-only and never marked dirty.
  let mut persistent = false;
  match op {
    CoordinateOp::SelectWcs { index } => {
      coords.select_wcs(index);
      persistent = true;
    }
    CoordinateOp::SetWcsOffset { index, axes, units } => {
      let (values, present) = axis_values_mm(&axes, units, rotary_mask);
      coords.set_wcs_offset(index, values, present);
      persistent = true;
    }
    CoordinateOp::SetWcsOffsetToPosition { index, axes, units } => {
      let (values, present) = axis_values_mm(&axes, units, rotary_mask);
      coords.set_wcs_offset_to_position(index, machine, values, present);
      persistent = true;
    }
    CoordinateOp::SetG92ToPosition { axes, units } => {
      let (values, present) = axis_values_mm(&axes, units, rotary_mask);
      coords.set_g92_to_position(machine, values, present);
    }
    CoordinateOp::ClearG92 => coords.clear_g92(),
    CoordinateOp::StorePredefined { index } => {
      coords.store_predefined(index, machine);
      persistent = true;
    }
    CoordinateOp::ApplyTlo { z, units } => coords.apply_tlo(z * units_scale(units)),
    CoordinateOp::CancelTlo => coords.cancel_tlo(),
  }
  set_coordinates(coords);
  // The WCO may have changed (every op except a no-op select can shift it); push it into the planner so the
  // next absolute work move resolves correctly.
  push_wco_to_planner().await;
  if persistent {
    mark_coordinates_dirty();
  }
}

/// Keep the coordinate model's active WCS in sync with the parser's modal `wcs` for a move that shares a line
/// with a G54-G59 select (the parser emits the move, not a SelectWcs op, in that case). When they already match
/// this is a cheap no-op; on a change it re-selects, pushes the new WCO into the planner, and marks the
/// coordinate record dirty (the active WCS is part of the persistent subset).
pub(crate) async fn sync_active_wcs(parser_wcs: usize) {
  let coords = coordinates();
  if coords.active_wcs() == parser_wcs {
    return;
  }
  let mut updated = coords;
  updated.select_wcs(parser_wcs);
  set_coordinates(updated);
  push_wco_to_planner().await;
  mark_coordinates_dirty();
}

/// Resolve a line's [`AxisWords`] into an mm value array plus a per-axis "present" mask, scaling inch words to mm.
/// Absent axes carry `0.0` with `present = false` so a mutator writes only the mentioned axes. The full [`AXES`]
/// word set is read (X/Y/Z AND the rotary A) — omitting A previously indexed a 3-element array at axis 3 and
/// PANICKED on a G92/G10 L2/L20 line carrying an A word (`AXES == 4`). Per the DOC-10.1 rotary convention a word on
/// a ROTARY axis (per the live `$376` `rotary_mask`) is in DEGREES and is NEVER inch-scaled — a `G20 ... A90` is 90
/// degrees, not 90 × 25.4 — matching [`Planner::resolve_target`]'s per-axis scale fork, so a WCS/G92 offset on a
/// rotary A stores degrees. `rotary_mask` is the live `$376` value; bit N set marks axis N angular.
fn axis_values_mm(axes: &firmware_core::gcode::AxisWords, units: GcodeUnits, rotary_mask: u8) -> ([f32; AXES], [bool; AXES]) {
  let linear_scale = units_scale(units);
  let words = [axes.x, axes.y, axes.z, axes.a];
  let mut values = [0.0f32; AXES];
  let mut present = [false; AXES];
  for axis in 0..AXES {
    if let Some(value) = words[axis] {
      // A rotary axis word is degrees — never inch-scaled (its scale is 1.0); a linear word scales mm/inch.
      let scale = if rotary_mask & (1 << axis) != 0 { 1.0 } else { linear_scale };
      values[axis] = value * scale;
      present[axis] = true;
    }
  }
  (values, present)
}

/// Persist the live PERSISTENT coordinate subset (G54-G59 / G28 / G30 / active WCS) to flash IF a change is
/// pending, clearing [`COORDINATES_DIRTY`]. The coordinate analogue of [`flush_settings`]: callers mark dirty
/// per persistent op, and this performs the actual flash append once per burst (queue-empty), on the safety
/// interval, and on soft reset. The dirty flag is cleared BEFORE the write so a change landing during the await
/// re-marks dirty and is caught by the next flush. On write FAILURE we re-mark dirty when `retry_on_failure` is
/// set (Bug A) so the change is retried, mirroring [`flush_settings`]: the burst-boundary caller passes `false`
/// to avoid spinning, while the safety-timer and soft-reset arms pass `true` for a bounded retry.
async fn flush_coordinates(flash: &'static SharedFlash, retry_on_failure: bool) {
  if !COORDINATES_DIRTY.swap(false, Ordering::AcqRel) {
    return;
  }
  let persistent = coordinates().persistent();
  let mut store = FlashRecordStore::coordinates(flash);
  // Mark the flash write (same `multicore_auto_park`/cache-disable hazard as the settings flush, Defect #2).
  crate::crash::record_comms_stage(crate::crash::CommsTask::Consumer, crate::crash::CommsStage::ConsumerFlashCoords);
  if coords::store_coordinates(&mut store, &persistent).await.is_err() {
    #[cfg(feature = "defmt")]
    defmt::warn!("coordinates: failed to flush coordinate record to flash");
    if retry_on_failure {
      COORDINATES_DIRTY.store(true, Ordering::Release);
    }
  }
}

/// How long the consumer waits before retrying a [`PlannerError::QueueFull`] command. Short relative to a
/// block's execution time (tens of ms) so the retry claims a freed slot promptly, but long enough that the
/// retry loop is not a busy-spin — it yields to the core-1 motion executor each iteration.
const QUEUE_FULL_RETRY: Duration = Duration::from_millis(2);

/// Safety interval for the coalesced settings flush (Finding #14b): an upper bound on how long a pending
/// `$n=val`/`$PBX` change can sit un-persisted when the line queue never observably empties (a slow trickle
/// that always keeps one line in flight). The primary trigger is the burst boundary (queue empty); this is the
/// backstop so a dirty change is never indefinitely deferred. One second is far longer than a normal burst yet
/// short enough that a power loss after a paused write loses at most a second of un-flushed edits.
const SETTINGS_FLUSH_SAFETY: Duration = Duration::from_secs(1);
