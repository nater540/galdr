//! Spindle + coolant drive (DOC-07): the two single-driver core-0 tasks ([`spindle`]/[`coolant`], each the sole
//! writer of its outputs, awaiting an update vs an e-stop signal), the modal drivers the consumer calls after every
//! clean parse ([`sync_spindle_from_modal`]/`sync_coolant_from_modal` — keying off modal state so an M3/M8 sharing a
//! line with a move still actuates), the `$392` spin-up dwell injection ([`inject_spin_up_dwell`]), and the universal
//! safety chokepoints [`force_spindle_off`]/[`force_coolant_off`]. Extracted verbatim from `comms.rs`
//! (architecture-refactor A1, step 11). The consumer-facing entry points are `pub(crate)`
//! (`force_spindle_off`/`force_coolant_off` — also called from `syscmd`/`program_flow` — `sync_spindle_from_modal`/
//! `sync_coolant_from_modal`/`inject_spin_up_dwell`/[`SpinUpInjection`]); the `spindle`/`coolant` tasks and
//! `commanded_coolant` stay `pub` (spawned from `main`); the apply/dispatch internals
//! (`apply_spindle`/`complete_spindle_reverse`/`spindle_emergency_stop`/`commanded_spindle`/`dispatch_spindle`/
//! `coolant_mask`) stay private. The shared `dwell_duration`/`MAX_DWELL_S` (also a `G4`/reverse-dwell bound) and
//! `axis_values_mm` (used by the gcode dispatch) STAY in `comms.rs` and are reached here via the `use super::*` glob,
//! which also supplies the spindle/coolant statics/signals, the override/RPM accessors, `enqueue`, and the
//! firmware_core types; the child glob needs no explicit imports.

use super::*;

/// Force the spindle to a hard stop (DOC-07): clear the commanded direction to Stop and signal the spindle task's
/// emergency stop. Used by every spindle-killing event — ALARM ([`emit_alarm`]), soft reset ([`reset_pipeline`]),
/// and sleep ([`handle_sleep`]). Synchronous (an atomic store + a coalesced `Signal`), callable from any context.
pub(crate) fn force_spindle_off() {
  SPINDLE_DIRECTION.store(SPINDLE_DIR_STOP, Ordering::Release);
  SPINDLE_ESTOP.signal(());
}

/// Force coolant OFF (DOC-07 safety): clear the commanded modal coolant state and signal the [`coolant`] task's
/// emergency stop. Used by every coolant-killing event — ALARM ([`emit_alarm`]), soft reset ([`reset_pipeline`]),
/// program end ([`program_end`]), graceful stop ([`program_stop_cycle`]), and sleep ([`handle_sleep`]) — mirroring
/// [`force_spindle_off`]. Synchronous (an atomic store + a coalesced `Signal`), callable from any context.
pub(crate) fn force_coolant_off() {
  COOLANT_STATE.store(0, Ordering::Release);
  COOLANT_ESTOP.signal(());
}

/// Drive the [`coolant`] task from the parser's MODAL coolant state (M7/M8/M9), called after every clean parse.
/// Like [`sync_spindle_from_modal`], keying off modal state (not the per-line emit) is what makes an M7/M8 that
/// SHARES a line with a move still actuate coolant (the emit is the move). Acts only on a real change vs the last
/// dispatched mask. Synchronous (an atomic store + a coalesced `Signal`).
pub(crate) fn sync_coolant_from_modal(modal: &ModalState, state: &mut ConsumerState) {
  let mask = coolant_mask(modal.coolant);
  if mask != state.last_coolant {
    COOLANT_STATE.store(mask, Ordering::Release);
    COOLANT_UPDATE.signal(());
    state.last_coolant = mask;
  }
}

/// Pack a [`CoolantState`](firmware_core::gcode::CoolantState) into the [`COOLANT_STATE`] bitmask.
fn coolant_mask(coolant: firmware_core::gcode::CoolantState) -> u8 {
  (if coolant.mist { COOLANT_BIT_MIST } else { 0 }) | (if coolant.flood { COOLANT_BIT_FLOOD } else { 0 })
}

/// Unpack the live [`COOLANT_STATE`] bitmask into a [`CoolantState`](firmware_core::gcode::CoolantState), for the
/// [`coolant`] task. An `Acquire` load pairs with the consumer's `Release` store.
pub fn commanded_coolant() -> firmware_core::gcode::CoolantState {
  let mask = COOLANT_STATE.load(Ordering::Acquire);
  firmware_core::gcode::CoolantState {
    mist: mask & COOLANT_BIT_MIST != 0,
    flood: mask & COOLANT_BIT_FLOOD != 0,
  }
}

/// Drive the [`spindle`] task from the parser's MODAL spindle state (DOC-07), called after every clean parse. This
/// is the single point the firmware acts on M3/M4/M5 + `S`: keying off modal state (not the per-line emit) is what
/// makes a spindle word that SHARES a line with a move still start/change the spindle (the emit is the move), and
/// lets a bare `S` re-drive a running spindle. Acts only on a real change vs the last dispatched `(dir, rpm)`:
/// - a DIRECTION change publishes the new direction, (dis)arms the spin-up gate, and wakes the task;
/// - a pure RPM change while RUNNING wakes the task to re-drive the duty WITHOUT re-arming the spin-up (a speed
///   change is not a fresh spindle start, so it owes no spin-up dwell).
/// Synchronous (atomics + a coalesced `Signal`); `PROGRAMMED_SPINDLE_RPM` must already be stored for this `S`.
pub(crate) fn sync_spindle_from_modal(modal: &ModalState, state: &mut ConsumerState) {
  let dir = modal.spindle;
  let rpm = modal.spindle_speed.max(0.0).min(u16::MAX as f32) as u16;
  if dir != state.last_spindle_dir {
    // A direction change (start, reversal, or stop): dispatch arms/disarms the spin-up gate and wakes the task.
    dispatch_spindle(dir, state);
  } else if !matches!(dir, SpindleState::Stop) && rpm != state.last_spindle_rpm {
    // Same direction, new speed on a RUNNING spindle: wake the task to re-drive the duty from the new programmed
    // RPM (already stored). No gate re-arm — this is a speed change, not a spindle start.
    SPINDLE_UPDATE.signal(());
  }
  state.last_spindle_dir = dir;
  state.last_spindle_rpm = rpm;
}

/// Publish an M3/M4/M5 direction to the [`spindle`] task (DOC-07): record it in [`SPINDLE_DIRECTION`], note the
/// spin-up gate (so the next cutting move gets a `$392` dwell), and wake the task. Synchronous — atomics + a
/// coalesced `Signal` — so it adds no await to the line handler. The task reads the override-scaled RPM, so a
/// spindle-override / spindle-stop change later re-drives the duty without re-issuing the M-word. Called by
/// [`sync_spindle_from_modal`] on a modal direction change (the single dispatch path).
fn dispatch_spindle(spindle_state: SpindleState, state: &mut ConsumerState) {
  let dir = match spindle_state {
    SpindleState::Clockwise => SPINDLE_DIR_CW,
    SpindleState::CounterClockwise => SPINDLE_DIR_CCW,
    SpindleState::Stop => SPINDLE_DIR_STOP,
  };
  SPINDLE_DIRECTION.store(dir, Ordering::Release);
  // Arm / disarm the spin-up gate: an M3/M4 owes a dwell to the next cutting move; an M5 clears it.
  state.spin_up.note_spindle(spindle_state);
  // Wake the spindle task to drive the outputs from the new direction + the current scaled RPM.
  SPINDLE_UPDATE.signal(());
}

/// The outcome of [`inject_spin_up_dwell`]: whether the line handler should keep planning the move or abort it
/// because a soft reset preempted the awaited spin-up dwell.
pub(crate) enum SpinUpInjection {
  /// No dwell was owed (or one was inserted and completed); continue planning the move.
  Continue,
  /// A soft reset arrived while awaiting the spin-up dwell; the caller must run the soft-reset transition and
  /// drop the move (the host discards pending acks on `0x18`).
  Aborted,
  /// A graceful program stop (`0x86`) arrived while the spin-up dwell's enqueue was back-pressured; the caller must
  /// run [`program_stop_cycle`] and drop the move. Distinct from [`Aborted`](SpinUpInjection::Aborted) — a stop
  /// returns to Idle with position retained and no alarm, where the `0x18` abort runs the warm reset.
  Stopped,
}

/// DOC-07 spin-up dwell injection. Before the FIRST cutting move (a G1 feed `Move` or any `Arc`) after an M3/M4,
/// insert a synchronized `$392` dwell so the spindle reaches speed before it cuts. The host-tested [`SpinUpGate`]
/// (on [`ConsumerState`]) owns the WHEN decision; this supplies the dwell seconds from the live `$392` and runs
/// the timed wait. It is a no-op (returns [`SpinUpInjection::Continue`]) for a rapid (G0), a non-move command, or
/// when no spin-up is owed.
///
/// The dwell is realized two ways, matching grbl's "a dwell is a synchronized motion boundary": a
/// [`PlannerCommand::Dwell`] is planned first (flushing look-ahead so any preceding block stops at the boundary),
/// then the real timed wait is awaited here — raced against [`SOFT_RESET`] so a `0x18` mid-dwell aborts promptly.
pub(crate) async fn inject_spin_up_dwell(
  command: &firmware_core::gcode::PlannerCommand,
  state: &mut ConsumerState,
) -> SpinUpInjection {
  use firmware_core::gcode::PlannerCommand;
  // Only a CUTTING move consumes the spin-up: a G1 feed move or any arc. A G0 rapid is a positioning move, not a
  // cut, so it does not consume the spin-up (the dwell waits for the first real cut). Any non-move command (dwell,
  // coordinate op, spindle, probe, …) is not a cut either.
  let is_cutting_move = matches!(command, PlannerCommand::Move { rapid: false, .. } | PlannerCommand::Arc { .. });
  if !is_cutting_move {
    return SpinUpInjection::Continue;
  }
  // Cheap guard BEFORE the settings snapshot: only the first cutting move after a spindle start owes a dwell, so a
  // dense toolpath's every-G1 common case skips the full-`Settings` snapshot entirely (the gate is a single bool).
  if !state.spin_up.is_pending() {
    return SpinUpInjection::Continue;
  }
  // A spin-up IS owed: read the live `$392` and consume the gate. `take_dwell_before_move` returns the seconds to
  // dwell, or `None` only when `$392 == 0` (the gate is consumed either way, so the next move gets no dwell).
  let spin_up_s = settings_snapshot().await.spindle_on_delay_s;
  let Some(dwell_s) = state.spin_up.take_dwell_before_move(spin_up_s) else {
    return SpinUpInjection::Continue;
  };
  // Flush the planner's look-ahead at the boundary so the cut starts from rest: plan a synchronized G4 dwell (the
  // planner pins the preceding block to a stop). `plan_command` returns `PlanResult::Dwell` for it, or `Aborted`
  // if a soft reset preempted a back-pressured enqueue.
  match plan_command(&PlannerCommand::Dwell { seconds: dwell_s }).await {
    PlanResult::Aborted => return SpinUpInjection::Aborted,
    PlanResult::Stopped => return SpinUpInjection::Stopped,
    _ => {}
  }
  // Run the SAME synchronized dwell a real `G4` uses (wait for prior motion to drain, then hold `$392` so the
  // spindle reaches speed), raced against a soft reset — one dwell mechanism, no ad-hoc timer.
  if run_dwell(dwell_s).await {
    SpinUpInjection::Continue
  } else {
    SpinUpInjection::Aborted
  }
}

/// The spindle task (DOC-07, core 0 / PRO_CPU). The SINGLE driver of the spindle outputs: it owns the
/// [`SpindleController`](firmware_core::spindle::SpindleController) and awaits two signals —
/// - [`SPINDLE_UPDATE`]: re-apply from the commanded [`SPINDLE_DIRECTION`] + the override-scaled RPM (an M3/M4/M5
///   or a spindle-override / spindle-stop change). A running-spindle direction REVERSAL returns
///   [`SpindleAction::SpinDownThenReverse`]: the controller has already stopped the spindle; this task awaits the
///   `$393` reverse dwell (raced against an e-stop) then completes the reversal.
/// - [`SPINDLE_ESTOP`]: an immediate emergency stop (ALARM / soft reset / hard limit / sleep), independent of the
///   commanded state. A feed hold (`!`) deliberately does NOT signal this — the spindle keeps running (grblHAL).
///
/// The task NEVER blocks the consumer: the consumer only stores atomics + signals; all timing lives here.
#[embassy_executor::task]
pub async fn spindle(controller: &'static mut spindle::Spindle) {
  loop {
    // E-stop is polled FIRST so that when BOTH an emergency stop and an update are pending at a wake (e.g. an
    // ALARM raised on core 1 while a spindle command's update is still queued), `select`'s first-future bias
    // services the stop — never the update that would briefly re-energize the spindle before the next iteration.
    match select(SPINDLE_ESTOP.wait(), SPINDLE_UPDATE.wait()).await {
      // Emergency stop wins unconditionally: de-assert enable + zero duty regardless of the commanded state.
      Either::First(()) => spindle_emergency_stop(controller),
      Either::Second(()) => {
        // Re-read the commanded direction and the realized (override-scaled) RPM, plus the live `$30`/`$31`/`$393`.
        let action = apply_spindle(controller).await;
        if let Some(dwell_s) = action {
          // A reversal of a running spindle: the controller already stopped it. Await the `$393` reverse dwell,
          // raced against an e-stop so a reset mid-spin-down still parks the spindle, then bring up the new
          // direction (unless a newer command/e-stop changed the picture, which the re-read in `complete` honors).
          match select(Timer::after(dwell_duration(dwell_s)), SPINDLE_ESTOP.wait()).await {
            Either::First(()) => complete_spindle_reverse(controller).await,
            Either::Second(()) => spindle_emergency_stop(controller),
          }
        }
      }
    }
  }
}

/// The coolant task (DOC-07 follow-up, core 0 / PRO_CPU). The SINGLE driver of the (hardware-gated) coolant
/// outputs: it owns the [`CoolantController`](firmware_core::coolant::CoolantController) and awaits two signals,
/// exactly mirroring the [`spindle`] task —
/// - [`COOLANT_UPDATE`]: re-apply from the commanded [`COOLANT_STATE`] (an M7/M8/M9).
/// - [`COOLANT_ESTOP`]: force BOTH circuits off immediately (ALARM / soft reset / hard limit / sleep / M2/M30).
///
/// The task NEVER blocks the consumer (the consumer only stores an atomic + signals). Because the coolant GPIO is
/// stubbed today (no driver stage budgeted), the `apply`/`emergency_stop` calls drive the stub outputs — the logic,
/// the safety chokepoints, and the task topology are all real now; only the pin write is a no-op until a stage exists.
#[embassy_executor::task]
pub async fn coolant(controller: &'static mut crate::coolant::Coolant) {
  loop {
    // E-stop is polled FIRST (the `select` first-future bias) so that when BOTH an e-stop and an update are pending
    // — e.g. an ALARM raised while an M8 update is still queued — the stop wins and coolant never briefly re-asserts.
    match select(COOLANT_ESTOP.wait(), COOLANT_UPDATE.wait()).await {
      // Emergency stop wins unconditionally: both circuits off regardless of the commanded state. A driver error is
      // logged (defmt) and swallowed so the task keeps running for the next command / e-stop (matching the spindle).
      Either::First(()) => {
        if let Err(_e) = controller.emergency_stop() {
          #[cfg(feature = "defmt")]
          defmt::error!("coolant emergency-stop failed: {:?}", _e);
        }
      }
      // Re-read the commanded modal coolant state and drive both circuits to it.
      Either::Second(()) => {
        if let Err(_e) = controller.apply(commanded_coolant()) {
          #[cfg(feature = "defmt")]
          defmt::error!("coolant apply failed: {:?}", _e);
        }
      }
    }
  }
}

/// Read the commanded spindle direction + override-scaled RPM + the live `$30`/`$31`/`$393`, apply them to the
/// controller, and return `Some(dwell_s)` when the apply scheduled a direction reversal (the caller must await the
/// reverse dwell), or `None` when the command was fully applied. A driver error is logged (defmt) and swallowed —
/// the spindle task must keep running so a later command / e-stop can still reach the hardware.
async fn apply_spindle(controller: &mut spindle::Spindle) -> Option<f32> {
  let settings = settings_snapshot().await;
  let (state, rpm) = commanded_spindle();
  match controller.apply(state, rpm, settings.spindle_rpm_min, settings.spindle_rpm_max, settings.spindle_reverse_dwell_s) {
    Ok(SpindleAction::Applied) => None,
    Ok(SpindleAction::SpinDownThenReverse { dwell_s }) => Some(dwell_s),
    Err(_e) => {
      #[cfg(feature = "defmt")]
      defmt::error!("spindle apply failed: {:?}", _e);
      None
    }
  }
}

/// Complete a deferred M3↔M4 reversal after the `$393` dwell: bring up the (re-read) commanded direction at the
/// current scaled RPM. Re-reading honors a command that changed during the dwell (e.g. an M5 mid-spin-down parks
/// it rather than energizing the stale direction). A driver error is logged and swallowed.
async fn complete_spindle_reverse(controller: &mut spindle::Spindle) {
  let settings = settings_snapshot().await;
  let (state, rpm) = commanded_spindle();
  if let Err(_e) = controller.complete_reverse(state, rpm, settings.spindle_rpm_min, settings.spindle_rpm_max) {
    #[cfg(feature = "defmt")]
    defmt::error!("spindle reverse-complete failed: {:?}", _e);
  }
}

/// Emergency-stop the spindle (de-assert enable + zero duty), logging and swallowing any driver error so the task
/// keeps running. Idempotent at the controller level.
fn spindle_emergency_stop(controller: &mut spindle::Spindle) {
  if let Err(_e) = controller.emergency_stop() {
    #[cfg(feature = "defmt")]
    defmt::error!("spindle emergency-stop failed: {:?}", _e);
  }
}

/// The currently commanded spindle `(state, rpm)`: the modal direction from [`SPINDLE_DIRECTION`] and the
/// realized RPM = the programmed `S` scaled by the live spindle override + spindle-stop toggle. So an
/// override/stop change re-drives the controller at the new speed (or to a stop) on the next [`SPINDLE_UPDATE`].
fn commanded_spindle() -> (SpindleState, f32) {
  let state = match SPINDLE_DIRECTION.load(Ordering::Acquire) {
    SPINDLE_DIR_CW => SpindleState::Clockwise,
    SPINDLE_DIR_CCW => SpindleState::CounterClockwise,
    _ => SpindleState::Stop,
  };
  let programmed = PROGRAMMED_SPINDLE_RPM.load(Ordering::Acquire).min(u16::MAX as u32) as u16;
  let scaled = overrides().scaled_rpm(programmed);
  (state, scaled as f32)
}
