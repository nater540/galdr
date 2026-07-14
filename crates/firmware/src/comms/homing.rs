//! `$H` homing cycle (DOC-06): the core-0 gating that resolves the [`HomingConfig`] from live settings, publishes
//! the pre-cycle `<Home>` state, dispatches the cycle to the core-1 executor (which owns the RMT channels + limit
//! inputs) via `HOME_REQUEST`, awaits `HOME_RESULT` as a synchronized boundary (racing a soft reset), and decides
//! success (sync planner to machine zero, clear `ALARM:11`, `ok`) vs failure (`ALARM:8` + pipeline reset) — extracted
//! verbatim from `comms.rs` (architecture-refactor A1, step 9). `handle_home` is called from the `$H` handler in
//! `syscmd.rs`, so it is `pub(crate)`; `comms.rs` re-exports this module (`pub(crate) use homing::*;`) so that call
//! keeps resolving. `run_homing_cycle` is homing-internal (only `handle_home` calls it) and stays private. `use
//! super::*` supplies the parent surface (the homing signals/statics, state accessors, `enqueue`/`ack`/`error`, the
//! staying `reset_pipeline`/`apply_soft_reset`/`emit_alarm` helpers, and the firmware_core types); the child glob
//! needs no explicit imports.

use super::*;

/// Handle `$H` (run the homing cycle, DOC-06). Sequence:
/// 1. If `$22` homing is disabled → `error:5` ("Homing cycle is not enabled"), no motion.
/// 2. If the control state does not allow homing (a locked hard/soft-limit/e-stop alarm, a hold, check, or
///    sleep) → `error:9` (G-code lock); `$H` runs from Idle/Normal and from the homing-required boot lock only.
/// 3. Publish the pre-cycle `<Home>` State (set [`HOMING_ACTIVE`] so `?` reports `Home` and the hard-limit
///    monitor is suppressed) and emit one `<Home|...>` report, mirroring grbl's pre-cycle push.
/// 4. Dispatch the resolved [`HomingConfig`] to the core-1 executor (which owns the RMT channels + limit
///    inputs) via [`HOME_REQUEST`] and AWAIT [`HOME_RESULT`], racing a soft reset so a `0x18` mid-cycle aborts.
/// 5. On SUCCESS: sync the planner's commanded position to the established machine zero, clear `ALARM:11` to
///    Normal via [`ControlState::home_complete`], and `ok`. On a homing FAIL (no contact) or sink abort: raise
///    `ALARM:8` (homing fail) + its prompt and force a pipeline reset — never fabricate a homed state.
pub(crate) async fn handle_home(parser: &mut Parser, state: &mut ConsumerState) {
  if !HOMING_ENABLED.load(Ordering::Relaxed) {
    error(ERROR_HOMING_DISABLED).await;
    return;
  }
  // grbl gates `$H`: it runs from Idle/Normal and from the homing-required boot lock, but never from a locked
  // critical alarm, a feed-hold, check, or sleep. The host-tested predicate decides; reject with the lock code.
  if !control_state().homing_allowed() {
    error(ERROR_LOCKED).await;
    return;
  }

  // Build the resolved homing config from the live settings on core 0 (only core 0 reads SETTINGS), so the
  // core-1 executor runs the cycle without touching the async settings mutex — mirroring the probe dispatch.
  let config = settings_snapshot().await.homing_config(crate::MOTION_TICK_HZ);

  // Pre-cycle `<Home>` push (research finding #1): mark homing active so `?` reports `Home` and the hard-limit
  // alarm path is suppressed for the cycle's duration, then ask the status responder to emit one report before
  // the motion begins (it composes the `Home` State from `HOMING_ACTIVE`, set just above).
  HOMING_ACTIVE.store(true, Ordering::Release);
  STATUS_REQUEST.signal(());

  let result = run_homing_cycle(config).await;
  HOMING_ACTIVE.store(false, Ordering::Release);

  match result {
    Some(Ok(zero_steps)) => {
      // The executor reported a clean cycle, but the control state can have been CLOBBERED to a locked alarm in
      // the post-cycle window — `HOMING_ACTIVE` was cleared above, so a limit still engaged when the idle
      // limit-monitor (finding #1) re-samples, or any other `0x18`/alarm path, can flip `CONTROL` to
      // `Alarm(..)` across the `PLANNER.lock().await` below. `home_complete()` is a no-op from a non-`Normal`able
      // state, so it would leave the alarm in place — but unconditionally acking + marking homed would emit a
      // spurious `ok` and a FALSE `HOMED` over a real alarm (finding #2). So we ACT only when the transition
      // genuinely reached `Normal`: sync the planner, mark homed, and `ok`. Otherwise we leave the alarm
      // untouched and emit no `ok` — the alarm's own path already surfaced `ALARM:N` to the host.
      let before = control_state();
      let after = before.home_complete();
      if after == ControlState::Normal {
        // Position is established: sync the planner's commanded position to the machine zero (grbl's
        // `plan_sync_position` + `gc_sync_position`), clear the homing-required alarm to Normal, and `ok`.
        {
          let mut guard = PLANNER.lock().await;
          if let Some(planner) = guard.as_mut() {
            planner.sync_position(zero_steps);
          }
        }
        set_control_state(after);
        // Mark the machine homed so `$20` soft limits become active (research finding #16). A subsequent soft
        // reset / `$X` that loses certainty clears this in `apply_soft_reset`.
        HOMED.store(true, Ordering::Relaxed);
        ack().await;
      }
      // else: the state was clobbered to an alarm during the success window — leave it locked, emit no `ok`, and
      // do NOT mark homed. The clobbering path (e.g. the hard-limit monitor) owns reporting + the pipeline reset.
    }
    Some(Err(_)) => {
      // Homing failed (no switch contact within 1.5× travel, or a sink abort): position is unknown. Raise
      // `ALARM:8` (homing fail) and force a pipeline reset so the machine is in a clean, clearly-unhomed alarm
      // state — never an `ok`. grbl emits no `ok` for a failed homing cycle.
      set_control_state(ControlState::Alarm(AlarmCode::HomingFail));
      emit_alarm(AlarmCode::HomingFail).await;
      reset_pipeline(parser, state).await;
    }
    // A soft reset preempted the cycle: honor the consumed reset (rebuild the pipeline, emit the banner). The
    // executor's own reset path zeroes the live position; `apply_soft_reset` publishes the post-reset state.
    None => apply_soft_reset(parser, state).await,
  }
}

/// Dispatch a homing cycle to the core-1 executor and AWAIT its result, racing a soft reset (DOC-06). Mirrors
/// [`run_probe_cycle`]: drains any stale result, signals [`HOME_REQUEST`] with the resolved config, then waits
/// on [`HOME_RESULT`] vs [`SOFT_RESET`]. Returns `Some(result)` on completion, or `None` if a `0x18` landed
/// mid-cycle (the caller honors the consumed reset signal). Draining a still-latched request on the reset path
/// prevents an unrequested homing move from running after the pipeline rebuilds.
async fn run_homing_cycle(config: HomingConfig) -> Option<Result<[i32; AXES], HomingError>> {
  HOME_RESULT.try_take();
  HOME_REQUEST.signal(config);
  crate::crash::record_comms_stage(crate::crash::CommsTask::Consumer, crate::crash::CommsStage::ConsumerHomeResult);
  match select(HOME_RESULT.wait(), SOFT_RESET.wait()).await {
    Either::First(result) => Some(result),
    Either::Second(()) => {
      // Drain the request we just signalled in case the executor had not yet consumed it, so a still-latched
      // request cannot run an unrequested homing move from the freshly-zeroed origin after the reset.
      HOME_REQUEST.try_take();
      None
    }
  }
}
