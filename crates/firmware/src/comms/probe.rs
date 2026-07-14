//! `G38.x` probe cycle (DOC-09 Phase C): the core-0 probe gating that dispatches a [`ProbeRequest`] to the core-1
//! executor (which owns the RMT channels + probe input) via `PROBE_REQUEST`, awaits `PROBE_RESULT` as a synchronized
//! boundary, syncs the planner to the latched stop point, stores the `[PRB:]` result, and decides the per-line
//! `ok`/`ALARM` — extracted verbatim from `comms.rs` (architecture-refactor A1, step 7). `handle_probe` is called
//! from the consumer's plan path; `comms.rs` re-exports this module (`pub(crate) use probe::*;`) so that call keeps
//! resolving. `use super::*` supplies the parent surface (the probe signals/types, state accessors, `enqueue`/`ack`,
//! and the staying `units_scale`/`apply_soft_reset`/`emit_alarm` helpers); only names the glob leaves ambiguous are
//! imported explicitly.

use super::*;

/// Derive the fixed per-tick step period (in motion timer ticks) for a probe seeking `target` (machine steps) at
/// `feed` in `units`/min. grbl probes at a CONSTANT feed (no trapezoid), so a single period is used for every
/// step. The period is the tick rate divided by the dominant-axis step rate: `feed` → mm/s, scaled by the
/// dominant axis's steps/mm and divided into the move's mm length so the dominant axis (the one that steps every
/// tick) runs at the commanded surface speed. A degenerate/zero feed falls back to the slowest representable
/// period; [`ProbeStepper::run_probe`] clamps the final value into the encodable interval regardless.
pub(crate) async fn probe_step_period_ticks(target: &[i32; AXES], feed: f32, units: GcodeUnits) -> u32 {
  let settings = settings_snapshot().await;
  let steps_per_mm = settings.steps_per_mm();
  // The commanded machine position the probe starts from (grbl's `gc_state.position`), in steps.
  let start = {
    let guard = PLANNER.lock().await;
    guard.as_ref().map(Planner::position_steps).unwrap_or([0; AXES])
  };
  // The dominant axis is the one with the most steps over the probe travel — it steps every tick, so its step
  // rate sets the period. Compute the mm travel and the dominant step count to convert the surface feed (mm/min)
  // into a dominant-axis step period.
  let mut dom = 0usize;
  let mut dom_steps = 0u32;
  let mut sumsq_mm = 0.0f32;
  for axis in 0..AXES {
    let delta_steps = (target[axis] - start[axis]).unsigned_abs();
    if delta_steps > dom_steps {
      dom_steps = delta_steps;
      dom = axis;
    }
    if steps_per_mm[axis] > 0.0 {
      let mm = (target[axis] - start[axis]) as f32 / steps_per_mm[axis];
      sumsq_mm += mm * mm;
    }
  }
  let length_mm = libm::sqrtf(sumsq_mm);
  let feed_mm_s = (feed * units_scale(units) / 60.0).max(0.0);
  // mm of travel per dominant-axis step, then the dominant step rate, then the period in ticks.
  if dom_steps == 0 || length_mm <= 0.0 || feed_mm_s <= 0.0 || steps_per_mm[dom] <= 0.0 {
    // No motion or no feed: fall back to the slowest representable period (the stepper clamps it anyway). The
    // motion tick rate is the fixed firmware constant the executor uses.
    return u32::MAX;
  }
  let mm_per_dom_step = length_mm / dom_steps as f32;
  let dom_step_rate = feed_mm_s / mm_per_dom_step;
  let tick_hz = settings.motion_config(crate::MOTION_TICK_HZ).tick_hz;
  let period = tick_hz / dom_step_rate.max(f32::MIN_POSITIVE);
  if period.is_finite() && period > 0.0 {
    libm::roundf(period) as u32
  } else {
    u32::MAX
  }
}

/// Run a `G38.x` probe cycle end-to-end (Phase C). Dispatches the [`ProbeRequest`] to the core-1 executor and
/// AWAITS the [`ProbeResult`] (a probe is a synchronized boundary, so the consumer blocks until it completes),
/// racing a soft reset so a `0x18` mid-probe aborts cleanly. On a result it: syncs the planner's commanded
/// position to the latched stop point (grbl sets `gc_state.position` to the probe stop), stores the last-probe
/// result and pushes the immediate `[PRB:x,y,z:flag]` line, then returns the [`ProbeResult`] so the caller
/// decides ALARM (for an alarming mode that did not trigger) vs `ok`. Returns `None` if a soft reset preempted
/// the probe (the caller honors the consumed reset signal).
async fn run_probe_cycle(request: ProbeRequest) -> Option<ProbeResult> {
  // Drain any stale result so a result from a previous (aborted) probe cannot be mistaken for this one's.
  PROBE_RESULT.try_take();
  PROBE_REQUEST.signal(request);
  crate::crash::record_comms_stage(crate::crash::CommsTask::Consumer, crate::crash::CommsStage::ConsumerProbeResult);
  let result = match select(PROBE_RESULT.wait(), SOFT_RESET.wait()).await {
    Either::First(result) => result,
    // A soft reset landed mid-probe: abort. Drain the PROBE_REQUEST we just signalled in case the executor had
    // not yet consumed it (Finding #4) — otherwise a still-latched request would run an UNREQUESTED probe move
    // from the freshly-zeroed origin after the reset rebuilds the pipeline. (The `0x18` dispatch also drains
    // PROBE_REQUEST, but this is the in-order owner draining the request IT raised, so the abort is race-free
    // even if the executor consumed-then-was-reset between the two.) The executor's own MOTION_RESET path zeroes
    // the live position and the consumer's reset rebuilds the pipeline; honor the reset by returning None.
    Either::Second(()) => {
      PROBE_REQUEST.try_take();
      return None;
    }
  };
  // Sync the planner's commanded position to the actual stop point so subsequent moves resolve from there.
  {
    let mut guard = PLANNER.lock().await;
    if let Some(planner) = guard.as_mut() {
      planner.sync_position(result.stop_steps);
    }
  }
  // The latched MACHINE position in mm (the trigger point on success, end-of-travel on no-contact). Store it as
  // the last-probe result (for `$#`) and emit the immediate `[PRB:]` push line.
  let steps_per_mm = settings_snapshot().await.steps_per_mm();
  let position_mm = steps_to_mm(&result.stop_steps, &steps_per_mm);
  set_last_probe(LastProbe { position_mm, success: result.triggered });
  send_probe_report(&position_mm, result.triggered).await;
  Some(result)
}

/// Emit the immediate `[PRB:x,y,z:success]` probe-result push line (DOC-09, the auto-message a height-mapping
/// sender reads). Queued through the single USB writer like every other response; the per-line `ok`/`ALARM`
/// follows it, preserving the one-response-per-line contract (the `[PRB:]` is an extra report line, not the ack).
async fn send_probe_report(position_mm: &[f32; AXES], success: bool) {
  let mut s = Response::new();
  if ResponseWriter::probe_report(&mut s, position_mm, success).is_ok() {
    enqueue(s).await;
  }
}

/// Decide the response to a completed `G38.x` probe (Phase C). The `[PRB:]` push and the position sync already
/// happened in [`run_probe_cycle`]; this only chooses the per-line ack/alarm:
/// - **Triggered:** the probe saw its expected edge → emit a single `ok`, staying Idle (Normal). The
///   `[PRB:..:1]` line already preceded it.
/// - **Not triggered, alarming mode (G38.2/.4):** enter the alarm and emit `ALARM:N` — `ALARM:4`
///   ([`AlarmCode::ProbeFailInitial`]) when the probe was ALREADY at its expected edge before any motion (grbl's
///   "probe not in the expected initial state"), else `ALARM:5` ([`AlarmCode::ProbeFailContact`], "did not
///   contact within travel"). grbl emits no `ok` for a probe that alarms.
/// - **Not triggered, silent mode (G38.3/.5):** no alarm — emit a single `ok` and let the sender check the
///   `[PRB:..:0]` flag itself.
///
/// A soft reset preempting the probe (`run_probe_cycle` returned `None`) runs the soft-reset transition whose
/// signal was consumed, exactly like the back-pressure abort path.
pub(crate) async fn handle_probe(
  request: ProbeRequest,
  alarm_on_fail: bool,
  parser: &mut Parser,
  state: &mut ConsumerState,
) {
  let Some(result) = run_probe_cycle(request).await else {
    // A `0x18` landed mid-probe: honor the consumed reset signal (rebuild the pipeline, emit the banner).
    apply_soft_reset(parser, state).await;
    return;
  };
  // The `[PRB:..:flag]` push already went out in `run_probe_cycle`; decide the per-line ack/alarm from the
  // host-tested truth table (success → ok; alarming-mode failure → ALARM:4 already-at-edge / ALARM:5 no contact;
  // silent-mode failure → ok).
  match probe_response(result.triggered, result.already_at_edge, alarm_on_fail) {
    ProbeResponse::Ok => ack().await,
    ProbeResponse::Alarm(code) => {
      // A probe-fail alarm latches (grbl locks it until reset/`$X`), gating subsequent GCode. Emit `ALARM:N` and
      // its `[MSG:..]` continue prompt; grbl emits no `ok` for a probe that alarms.
      set_control_state(ControlState::Alarm(code));
      emit_alarm(code).await;
    }
  }
}
