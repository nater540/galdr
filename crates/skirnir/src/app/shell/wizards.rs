//! Probing + rotary/datum/mesh/flip/sweep wizards: the probe-result pumps and the multi-touch
//! measurement sequences. Split out of `shell.rs` (A4); pure relocation (NOT the C5 trait unification).

use super::*;

impl SkirnirApp {
  /// Drain a completed identify probe's verdict into the console, if one finished. Non-blocking: `try_recv`
  /// never waits, so the UI thread is never parked on the probe. Returns whether a verdict was surfaced (so the
  /// caller can request a prompt repaint). Clears the slot once the probe's sender has dropped.
  #[cfg(feature = "serial")]
  pub(crate) fn pump_probe(&mut self) -> bool {
    let Some(rx) = self.pending_probe.as_ref() else {
      return false;
    };
    match rx.try_recv() {
      Ok(message) => {
        self.notice(message);
        self.pending_probe = None;
        true
      }
      // Sender dropped without a message (should not happen, but clears the slot if it does).
      Err(std::sync::mpsc::TryRecvError::Disconnected) => {
        self.pending_probe = None;
        false
      }
      Err(std::sync::mpsc::TryRecvError::Empty) => false,
    }
  }

  /// Begin a hardened Z touch-off: send a RELATIVE `G38.2` probe and arm the probe latch, but DEFER the zeroing
  /// until the probe resolves successfully. [`Self::pump_probe_z`] builds and sends a position-independent
  /// `G10 L2` zero from the contact machine-Z only on a `success:1` result, and surfaces a notice (zeroing
  /// nothing) on any failure. This replaces the old fire-and-forget sequence that zeroed unconditionally and
  /// relied on alarm-ordering — a race — to protect a failed probe.
  ///
  /// The probe is wrapped `G91` … `G90` (per `docs/tlo-offsets.md`): under the power-on `G90`, `G38.2 Z-<depth>`
  /// would resolve as an ABSOLUTE target and travel to the wrong place — the probe must advance `<depth>` mm FROM
  /// the current position. The zero is computed on resolution as `G10 L2 P0 Z<contact_Z − plate_thickness>` (see
  /// [`crate::app::probe_flow::zero_z_line`]) so it is independent of where the tool sits when it lands — a jog during
  /// the lost-push window cannot corrupt it. A probe issued while one is pending replaces it (latest wins); a
  /// wizard run in progress is cancelled so the two cannot share the latch.
  pub(crate) fn probe_z(&mut self, depth: f64, feed: f64, plate_thickness: f64) {
    // Starting a ZeroZ probe cancels every other probe flow so the shared latch cannot be claimed by two at once.
    self.cancel_probe_ops_except(ProbeOpSlot::ZeroZ);
    // Arm the latch BEFORE the probe is sent so the result (which can arrive within a frame) always finds an op
    // awaiting it. `begin_probe` supersedes any prior op, matching the "latest request wins" rule.
    self.view.begin_probe(crate::app::view_state::ProbeKind::ZeroZ);
    let lines = [
      // Incremental probe wrapper: probe relative, then restore absolute mode.
      "G91".to_string(),
      format!("G38.2 Z-{depth:.3} F{feed:.0}"),
      "G90".to_string(),
    ];
    let mut all_sent = true;
    for line in lines {
      self.view.note_sent(line.clone());
      if !self.send_command(Command::SendLine(line)) {
        all_sent = false;
        break;
      }
    }
    if all_sent {
      self.pending_zero_z = Some(PendingZeroZProbe {
        inner: crate::app::probe_flow::PendingZeroZ::new(plate_thickness),
        issued_at: Instant::now(),
        polled_at: None,
      });
    } else {
      // The send failed (no engine): there is nothing to await, so drop the latch we just armed rather than
      // leaving it awaiting a result that can never come.
      self.view.fail_probe("probe not sent (not connected)");
      self.pending_zero_z = None;
    }
  }

  /// Drive the hardened Z touch-off one frame: gate the deferred zeroing on the probe latch and run the
  /// push-or-poll fallback. Pure [`crate::app::probe_flow::decide`] chooses the action from the latch outcome and the
  /// elapsed wall-clock; the shell only performs the I/O it names (send the zero line, query `$#`, surface a
  /// notice). Returns whether anything happened, so the caller can request a prompt repaint. No-op when no
  /// touch-off is pending. A disconnect clears the latch (the reducer) AND the pending here, so this abandons
  /// cleanly.
  pub(crate) fn pump_probe_z(&mut self) -> bool {
    use crate::app::probe_flow::{ZeroZAction, decide};
    // Copy out the wall-clock stamps up front so the immutable borrow is released before the `observe_busy`
    // mutation below. No pending touch-off ⇒ nothing to do.
    let (issued_at, polled_at) = match self.pending_zero_z.as_ref() {
      Some(p) => (p.issued_at, p.polled_at),
      None => return false,
    };
    // The latch must belong to THIS flow. If it is gone (a disconnect cleared it) or it belongs to another probe
    // kind (a rotary touch armed it), abandon our follow-up rather than acting on someone else's `[PRB:]` — the
    // `ProbeKind` field exists precisely to route the shared latch.
    match self.view.probe_op.as_ref() {
      Some(op) if op.kind == crate::app::view_state::ProbeKind::ZeroZ => {}
      _ => {
        self.pending_zero_z = None;
        return false;
      }
    }
    let now = Instant::now();
    let since_issue = now.duration_since(issued_at);
    let since_poll = polled_at.map(|at| now.duration_since(at)).unwrap_or(Duration::ZERO);
    // Whether the machine is in a probe CYCLE this frame, from the live status: Run/Hold/Jog/Home are in-cycle;
    // Idle (and anything else) is not. A no-status frame counts as not-busy, but the `seen_cycle` latch below
    // means a not-yet-started probe still cannot be mistaken for "finished".
    let busy_now = self.view.status.as_ref().is_some_and(|s| is_probe_cycle_state(s.machine_state.state));
    // Latch that we have seen the machine in a cycle, so a later return to Idle is trusted as completion (the
    // startup-race guard: the pre-`Run` Idle window must not pass the `$#` fallback gate).
    if busy_now && let Some(p) = self.pending_zero_z.as_mut() {
      p.inner.observe_busy();
    }
    // Snapshot the refreshed pending + latch outcome for the pure decision. Both borrows are read-only and
    // released before any action below mutates `self`.
    let (Some(pending), Some(op)) = (self.pending_zero_z.as_ref(), self.view.probe_op.as_ref()) else {
      return false;
    };
    let probe_finished = pending.inner.probe_finished(busy_now);
    let action = decide(op.last.as_ref(), &pending.inner, probe_finished, since_issue, since_poll);
    match action {
      ZeroZAction::Wait => false,
      ZeroZAction::Zero(zero_line) => {
        self.pending_zero_z = None;
        self.notice("probe contacted — setting work-Z".to_string());
        self.send_line(zero_line);
        true
      }
      ZeroZAction::Fail(reason) => {
        self.pending_zero_z = None;
        self.notice(format!("probe failed: {reason} — work-Z NOT changed"));
        true
      }
      ZeroZAction::Poll => {
        // The immediate `[PRB:]` push did not arrive: retrieve the last probe result via `$#` (its `[PRB:]` line
        // parses through the same path and resolves the latch). Send it once and start the give-up clock.
        if let Some(p) = self.pending_zero_z.as_mut() {
          p.inner.polled = true;
          p.polled_at = Some(now);
        }
        self.send_line("$#".to_string());
        true
      }
      ZeroZAction::GiveUp(reason) => {
        self.pending_zero_z = None;
        self.view.fail_probe(reason.clone());
        self.notice(format!("probe failed: {reason} — work-Z NOT changed"));
        true
      }
    }
  }

  /// Start a fresh rotary center-finder run (DOC-11 §1.2): build the pure wizard state for the given dowel
  /// diameter / index angle with the conservative bench defaults, replacing any run in progress. The operator
  /// then jogs to each approach and triggers the touches.
  pub(crate) fn rotary_center_start(
    &mut self,
    dowel_diameter: f64,
    index_angle_deg: f64,
    params: crate::app::rotary_probe::RotaryProbeParams,
  ) {
    self.wizard = Some(RotaryCenterRun {
      state: crate::app::rotary_center::WizardState::new(dowel_diameter, index_angle_deg),
      params,
      touch_fallback: None,
    });
    self.notice(format!("rotary center-finder: dowel {dowel_diameter:.3} mm @ A{index_angle_deg:.1}°"));
  }

  /// Trigger the wizard's next touch: ask the state machine which touch is due (left Y, right Y, or Z-top), emit
  /// its rotary-safe probe lines, and arm the Phase 0 latch so [`Self::pump_wizard`] can fold the result back in.
  /// Inert if no wizard is running, one is already probing, or the due touch is off-step.
  pub(crate) fn rotary_center_probe(&mut self) {
    use crate::app::rotary_center::WizardStep;
    let Some(run) = self.wizard.as_mut() else {
      self.notice("no rotary center-finder running".to_string());
      return;
    };
    if run.state.is_probing() {
      self.notice("rotary probe already in progress".to_string());
      return;
    }
    // Advance the state machine to the next probing step, getting the touch to issue. The step the wizard is in
    // selects which touch: EnterDowel→left Y, ProbeYLeft(resolved)→right Y, MoveToYc→Z-top.
    let touch = match run.state.step {
      WizardStep::EnterDowel => run.state.begin_y_left(),
      WizardStep::ReadyYRight => run.state.begin_y_right(),
      // The top probe is allowed only AFTER the move to Y_c has been sent (MovedToYc), never from MoveToYc.
      WizardStep::MovedToYc => run.state.begin_z_top(),
      _ => None,
    };
    let Some(touch) = touch else {
      self.notice("no rotary touch is due in this step".to_string());
      return;
    };
    let params = run.params;
    let lines = crate::app::rotary_probe::rotary_safe_probe_lines(touch, params);
    // Starting a rotary touch cancels every other probe flow so the shared latch cannot be claimed by two at once
    // (the pumps also kind-gate, but clearing here is the belt to that suspenders).
    self.cancel_probe_ops_except(ProbeOpSlot::Wizard);
    // Arm the latch BEFORE sending so the result always finds an op awaiting it; the wizard owns the follow-up.
    self.view.begin_probe(crate::app::view_state::ProbeKind::RotaryCenter);
    // Stamp the touch's lost-push fallback so a dropped `[PRB:]` push does not leave the wizard awaiting forever.
    if let Some(run) = self.wizard.as_mut() {
      run.touch_fallback =
        Some(TouchFallback { issued_at: Instant::now(), polled: false, polled_at: None, seen_cycle: false });
    }
    let mut all_sent = true;
    for line in lines {
      self.view.note_sent(line.clone());
      if !self.send_command(Command::SendLine(line)) {
        all_sent = false;
        break;
      }
    }
    if !all_sent {
      // The send failed mid-sequence (no engine): fail the latch and the wizard rather than awaiting forever.
      self.view.fail_probe("rotary probe not sent (not connected)");
      if let Some(run) = self.wizard.as_mut() {
        run.state.abort("probe not sent (not connected)");
      }
    }
  }

  /// Send the wizard's move-to-Y-center positioning move (the mandatory step before the top probe). Emits the
  /// retract + absolute Y move to the computed `Y_c`, then ADVANCES the wizard to `MovedToYc` so the top probe is
  /// unlocked only after the move was actually sent. Inert unless the wizard is at the `MoveToYc` step with a
  /// known center.
  pub(crate) fn rotary_center_move_to_yc(&mut self) {
    use crate::app::rotary_center::WizardStep;
    let Some(run) = self.wizard.as_ref() else {
      return;
    };
    if run.state.step != WizardStep::MoveToYc {
      self.notice("move-to-Yc is not due in this step".to_string());
      return;
    }
    let Some(lines) = run.state.move_to_yc_lines(run.params) else {
      self.notice("Y center not yet known".to_string());
      return;
    };
    let mut all_sent = true;
    for line in lines {
      if !self.send_line(line) {
        all_sent = false;
        break;
      }
    }
    if all_sent && let Some(run) = self.wizard.as_mut() {
      // The move was actually sent: advance so `begin_z_top` (gated on `MovedToYc`) becomes reachable.
      run.state.mark_moved_to_yc();
      self.notice("moved to Y center — probe the dowel top next".to_string());
    }
  }

  /// Write the found center to the active WCS via the wizard's offered `G10 L2` line (Y/Z only, never A). Inert
  /// until the wizard has a computed center (the `Review` step).
  pub(crate) fn rotary_center_write_wcs(&mut self) {
    use crate::app::rotary_center::Wcs;
    let Some(run) = self.wizard.as_ref() else {
      return;
    };
    let Some(line) = run.state.offer_g10(Wcs::Active) else {
      self.notice("no rotary center to write yet".to_string());
      return;
    };
    // Snapshot the found center for persistence BEFORE the borrow of `run` is dropped — `(Y_c, Z_c)` plus the
    // dowel/datum that produced them (DOC-11 §1.3). `y_center`/`z_center` are `Some` here because `offer_g10`
    // returned a line, but fall through cleanly if not rather than unwrapping.
    let setup = match (run.state.y_center(), run.state.z_center()) {
      (Some(y_center), Some(z_center)) => Some(crate::profile::RotarySetup {
        y_center,
        z_center,
        dowel_diameter: run.state.dowel_diameter,
        a_datum_deg: run.state.index_angle_deg,
        z_datum: run.state.z_datum,
      }),
      _ => None,
    };
    // Only claim the WCS write — and persist the center for re-apply — if the `G10` actually went out. A dropped
    // or absent engine makes `send_line` false (and already notices why); claiming success, or saving a center we
    // could not apply, would mislead the operator. Mirror the guarded `move_to_yc` path.
    if !self.send_line(line) {
      return;
    }
    self.notice("wrote rotary center to the active WCS (Y/Z only)".to_string());
    // Persist the center so a later session can re-apply it without re-running the whole center-finder.
    if let Some(setup) = setup {
      self.save_rotary_center(setup);
    }
  }

  /// Re-apply the rotary center saved in the profile (DOC-11 §1.3): re-emit the persisted `G10 L2` line (Y/Z
  /// only, never A) so a restart restores the found center without re-probing. Inert with a notice if nothing
  /// has been saved yet.
  pub(crate) fn apply_saved_rotary_center(&mut self) {
    let Some(setup) = self.profile.rotary else {
      self.notice("no saved rotary center to apply — run the center-finder first".to_string());
      return;
    };
    let line = setup.offer_g10();
    // Don't announce success if the line never left: a dropped/absent engine makes `send_line` false (and already
    // notices why), so a "re-applied" notice would contradict it.
    if !self.send_line(line) {
      return;
    }
    self.notice("re-applied the saved rotary center to the active WCS (Y/Z only)".to_string());
  }

  /// Drive a running rotary touch one frame: fold a resolved latch result into the wizard, or run the SHARED
  /// completion-gated lost-push fallback (`$#` poll, then give up) so a dropped/suppressed `[PRB:]` never leaves
  /// the wizard awaiting forever. Mirrors [`Self::pump_probe_z`] but folds the result into the state machine
  /// instead of zeroing. Returns whether anything changed (for a prompt repaint). No-op when no wizard is running
  /// or it is not awaiting a touch.
  pub(crate) fn pump_wizard(&mut self) -> bool {
    use crate::app::probe_flow::{AwaitAction, await_action};
    // Only act while a touch is in flight (a probing step). Copy the fallback stamps up front so the immutable
    // borrow is released before the `seen_cycle` mutation below.
    let (issued_at, polled_at) = match self.wizard.as_ref() {
      Some(run) if run.state.is_probing() => match &run.touch_fallback {
        Some(f) => (f.issued_at, f.polled_at),
        // Probing but no fallback stamp (e.g. a run restored mid-touch): nothing to pace; treat as just-issued.
        None => (Instant::now(), None),
      },
      _ => return false,
    };
    // The latch must belong to THIS flow. Gone (disconnect) or another kind (a ZeroZ armed it) ⇒ abort the
    // wizard rather than wait forever or act on someone else's `[PRB:]`.
    match self.view.probe_op.as_ref() {
      Some(op) if op.kind == crate::app::view_state::ProbeKind::RotaryCenter => {}
      _ => {
        if let Some(run) = self.wizard.as_mut() {
          run.state.abort("probe latch lost");
        }
        return true;
      }
    }
    // If the latch has resolved, fold the outcome into the wizard and finish the touch.
    let resolved = self.view.probe_op.as_ref().filter(|op| !op.awaiting).and_then(|op| op.last.clone());
    if let Some(outcome) = resolved {
      if let Some(run) = self.wizard.as_mut() {
        run.state.on_probe_result(&outcome);
        run.touch_fallback = None;
      }
      // Consume the latch so the result is fed exactly once (the next touch's `begin_probe` re-arms it).
      self.view.clear_probe_op();
      return true;
    }
    // Still awaiting: run the shared lost-push fallback, gated on the touch having demonstrably finished.
    let now = Instant::now();
    let busy_now = self.view.status.as_ref().is_some_and(|s| is_probe_cycle_state(s.machine_state.state));
    if busy_now && let Some(run) = self.wizard.as_mut() && let Some(f) = run.touch_fallback.as_mut() {
      f.seen_cycle = true;
    }
    let (polled, seen_cycle) = match self.wizard.as_ref().and_then(|r| r.touch_fallback.as_ref()) {
      Some(f) => (f.polled, f.seen_cycle),
      None => return false,
    };
    let probe_finished = seen_cycle && !busy_now;
    let since_issue = now.duration_since(issued_at);
    let since_poll = polled_at.map(|at| now.duration_since(at)).unwrap_or(Duration::ZERO);
    match await_action(polled, probe_finished, since_issue, since_poll) {
      AwaitAction::Wait => false,
      AwaitAction::Poll => {
        if let Some(run) = self.wizard.as_mut() && let Some(f) = run.touch_fallback.as_mut() {
          f.polled = true;
          f.polled_at = Some(now);
        }
        self.send_line("$#".to_string());
        true
      }
      AwaitAction::GiveUp(reason) => {
        if let Some(run) = self.wizard.as_mut() {
          run.state.abort(reason.clone());
          run.touch_fallback = None;
        }
        self.view.fail_probe(reason);
        true
      }
    }
  }

  /// Start a datum-finder single-edge run: touch off `axis` in `dir` with `params`, replacing any datum run in
  /// progress. Guarded by [`Self::verify_probe_clear`] — a probe already asserted (`Pn:P`) means a short / wrong
  /// polarity, so starting would probe against a triggered input and read garbage; refuse with a notice instead.
  pub(crate) fn datum_edge_start(&mut self, axis: Axis, dir: Dir, params: crate::app::datum::ProbeParams) {
    if !self.verify_probe_clear() {
      return;
    }
    self.cancel_probe_ops_except(ProbeOpSlot::Datum);
    self.datum = Some(DatumRun {
      state: crate::app::datum::DatumState::new_edge(axis, dir, params.probe_diameter),
      params,
      touch_fallback: None,
    });
    self.notice(format!("datum: single {} edge, approach {}", axis.letter(), dir_word(dir)));
  }

  /// Start a datum-finder corner run for `corner` with `params`, replacing any datum run in progress. Guarded by
  /// [`Self::verify_probe_clear`] like the edge start.
  pub(crate) fn datum_corner_start(&mut self, corner: crate::app::datum::Corner, params: crate::app::datum::ProbeParams) {
    if !self.verify_probe_clear() {
      return;
    }
    self.cancel_probe_ops_except(ProbeOpSlot::Datum);
    self.datum = Some(DatumRun {
      state: crate::app::datum::DatumState::new_corner(corner, params.probe_diameter),
      params,
      touch_fallback: None,
    });
    let side = if corner.inside { "inside" } else { "outside" };
    self.notice(format!("datum: {side} corner (approach X{:+.0} Y{:+.0})", corner.approach_x(), corner.approach_y()));
  }

  /// The VerifyProbe guard (mirroring ioSender's `VerifyProbe`): refuse to start a probe op when the probe input
  /// is already asserted (`Pn:P`), which means it is shorted or wired the wrong polarity. Returns whether it is
  /// safe to proceed; surfaces a notice and returns `false` when the probe is already triggered.
  fn verify_probe_clear(&mut self) -> bool {
    if self.view.pins.probe {
      self.notice("probe is already asserted (Pn:P) — check wiring/polarity before probing".to_string());
      return false;
    }
    true
  }

  /// Trigger the datum wizard's next touch: ask the state machine which touch is due, emit its two-stage `G38.3`
  /// latch lines, and arm the Phase 0 latch (as a `Datum` op) before the SLOW pass so [`Self::pump_datum`] folds
  /// the kept reading back in. Inert if no datum run is active, one is already probing, or no touch is due.
  ///
  /// The fast pass's `ok` is consumed by ordinary flow control; the latch is armed for the whole emitted sequence
  /// (issued before the first line), so the slow pass's `[PRB:]` — the only probe result the sequence produces —
  /// resolves it. That is exactly the two-stage contract: `G38.3` reports `[PRB:]` on each probe, but the fast
  /// pass's push is a stale intermediate the wizard folds as the reading; to keep the KEPT reading the slow one we
  /// rely on send-order and the wizard reading the LAST resolved result (each touch clears the latch before the
  /// next). Because a single touch issues exactly one wizard step, only one `[PRB:]` is awaited per touch here.
  pub(crate) fn datum_probe_next(&mut self) {
    let Some(run) = self.datum.as_mut() else {
      self.notice("no datum run active".to_string());
      return;
    };
    if run.state.is_probing() {
      self.notice("datum probe already in progress".to_string());
      return;
    }
    let Some(touch) = run.state.begin_probe() else {
      self.notice("no datum touch is due in this step".to_string());
      return;
    };
    let params = run.params;
    let lines = crate::app::datum::touch_lines(touch, &params);
    // Only this datum run may own the latch now (cross-contamination guard), and arm it BEFORE sending so the
    // result always finds an op awaiting it; the wizard owns the follow-up.
    self.cancel_probe_ops_except(ProbeOpSlot::Datum);
    self.view.begin_probe(crate::app::view_state::ProbeKind::Datum);
    if let Some(run) = self.datum.as_mut() {
      run.touch_fallback =
        Some(TouchFallback { issued_at: Instant::now(), polled: false, polled_at: None, seen_cycle: false });
    }
    let mut all_sent = true;
    for line in lines {
      self.view.note_sent(line.clone());
      if !self.send_command(Command::SendLine(line)) {
        all_sent = false;
        break;
      }
    }
    if !all_sent {
      // The send failed mid-sequence (no engine): fail the latch and the wizard rather than awaiting forever.
      self.view.fail_probe("datum probe not sent (not connected)");
      if let Some(run) = self.datum.as_mut() {
        run.state.abort("probe not sent (not connected)");
      }
    }
  }

  /// Write the found datum to the active WCS via the wizard's offered `G10 L2` line (one axis for an edge, X&Y for
  /// a corner). Inert until the wizard has a computed datum (the `Review` step). A failed send (no engine) already
  /// notices why, so only a real write claims success.
  pub(crate) fn datum_write_wcs(&mut self) {
    let Some(run) = self.datum.as_ref() else {
      return;
    };
    let Some(line) = run.state.offer_g10() else {
      self.notice("no datum to write yet".to_string());
      return;
    };
    if !self.send_line(line) {
      return;
    }
    self.notice("wrote datum to the active WCS".to_string());
  }

  /// Drive a running datum touch one frame: fold a resolved latch result into the wizard, or run the SHARED
  /// completion-gated lost-push fallback (`$#` poll, then give up) so a dropped/suppressed `[PRB:]` never leaves
  /// the wizard awaiting forever. Mirrors [`Self::pump_wizard`] but folds into the datum state machine. Returns
  /// whether anything changed (for a prompt repaint). No-op when no datum run is active or it is not awaiting.
  pub(crate) fn pump_datum(&mut self) -> bool {
    use crate::app::probe_flow::{AwaitAction, await_action};
    let (issued_at, polled_at) = match self.datum.as_ref() {
      Some(run) if run.state.is_probing() => match &run.touch_fallback {
        Some(f) => (f.issued_at, f.polled_at),
        None => (Instant::now(), None),
      },
      _ => return false,
    };
    // The latch must belong to THIS flow. Gone (disconnect) or another kind ⇒ abort the wizard rather than wait
    // forever or act on someone else's `[PRB:]`.
    match self.view.probe_op.as_ref() {
      Some(op) if op.kind == crate::app::view_state::ProbeKind::Datum => {}
      _ => {
        if let Some(run) = self.datum.as_mut() {
          run.state.abort("probe latch lost");
        }
        return true;
      }
    }
    // If the latch has resolved, fold the outcome into the wizard and finish the touch.
    let resolved = self.view.probe_op.as_ref().filter(|op| !op.awaiting).and_then(|op| op.last.clone());
    if let Some(outcome) = resolved {
      if let Some(run) = self.datum.as_mut() {
        run.state.on_probe_result(&outcome);
        run.touch_fallback = None;
      }
      // Consume the latch so the result is fed exactly once (the next touch's `begin_probe` re-arms it).
      self.view.clear_probe_op();
      return true;
    }
    // Still awaiting: run the shared lost-push fallback, gated on the touch having demonstrably finished.
    let now = Instant::now();
    let busy_now = self.view.status.as_ref().is_some_and(|s| is_probe_cycle_state(s.machine_state.state));
    if busy_now && let Some(run) = self.datum.as_mut() && let Some(f) = run.touch_fallback.as_mut() {
      f.seen_cycle = true;
    }
    let (polled, seen_cycle) = match self.datum.as_ref().and_then(|r| r.touch_fallback.as_ref()) {
      Some(f) => (f.polled, f.seen_cycle),
      None => return false,
    };
    let probe_finished = seen_cycle && !busy_now;
    let since_issue = now.duration_since(issued_at);
    let since_poll = polled_at.map(|at| now.duration_since(at)).unwrap_or(Duration::ZERO);
    match await_action(polled, probe_finished, since_issue, since_poll) {
      AwaitAction::Wait => false,
      AwaitAction::Poll => {
        if let Some(run) = self.datum.as_mut() && let Some(f) = run.touch_fallback.as_mut() {
          f.polled = true;
          f.polled_at = Some(now);
        }
        self.send_line("$#".to_string());
        true
      }
      AwaitAction::GiveUp(reason) => {
        if let Some(run) = self.datum.as_mut() {
          run.state.abort(reason.clone());
          run.touch_fallback = None;
        }
        self.view.fail_probe(reason);
        true
      }
    }
  }

  /// Start a height-map acquisition run over the grid `[min, max]` at `spacing` (work-mm) with `params`. Guarded by
  /// [`Self::verify_probe_clear`] (a probe already asserted means bad wiring), then builds the serpentine
  /// [`crate::app::autolevel::MeshProbeState`] over a fresh [`crate::app::autolevel::Mesh`]. Replaces any run in progress.
  pub(crate) fn mesh_probe_start(
    &mut self, params: crate::app::autolevel::GridProbeParams, min: (f64, f64), max: (f64, f64), spacing: (f64, f64),
  ) {
    if !self.verify_probe_clear() {
      return;
    }
    let mut mesh = crate::app::autolevel::Mesh::from_spacing(min, max, spacing);
    // Stamp the WCS the mesh is being probed under (the active `G54`…`G59`, default G54) so a later stream can
    // warn if the job runs under a different WCS than the surface was measured in.
    mesh.wcs_index = self.view.active_wcs.unwrap_or(0);
    let (nx, ny) = (mesh.nx, mesh.ny);
    self.cancel_probe_ops_except(ProbeOpSlot::Mesh);
    self.mesh_probe =
      Some(MeshProbeRun { state: crate::app::autolevel::MeshProbeState::new(mesh, params), touch_fallback: None });
    self.notice(format!("height-map acquisition: {nx}×{ny} grid ({} points)", nx * ny));
  }

  /// Trigger the acquisition's next point: ask the state machine for its two-stage `G38.3` Z-touch lines (the
  /// clearance retract, the work-XY rapid, the probe, the retract), emit them, and arm the Phase 0 latch as a
  /// `Mesh` op so [`Self::pump_mesh`] folds the result back. Inert if no run is active, one is probing, or done.
  pub(crate) fn mesh_probe_next(&mut self) {
    let lines = {
      let Some(run) = self.mesh_probe.as_mut() else {
        self.notice("no height-map acquisition active".to_string());
        return;
      };
      if run.state.is_probing() {
        self.notice("mesh probe already in progress".to_string());
        return;
      }
      match run.state.begin_next_point() {
        Some(lines) => lines,
        None => {
          self.notice("no mesh point is due".to_string());
          return;
        }
      }
    };
    // Only this run may own the latch now; arm it BEFORE sending so the result always finds an op awaiting it.
    self.cancel_probe_ops_except(ProbeOpSlot::Mesh);
    self.view.begin_probe(crate::app::view_state::ProbeKind::Mesh);
    if let Some(run) = self.mesh_probe.as_mut() {
      run.touch_fallback =
        Some(TouchFallback { issued_at: Instant::now(), polled: false, polled_at: None, seen_cycle: false });
    }
    let mut all_sent = true;
    for line in lines {
      self.view.note_sent(line.clone());
      if !self.send_command(Command::SendLine(line)) {
        all_sent = false;
        break;
      }
    }
    if !all_sent {
      self.view.fail_probe("mesh probe not sent (not connected)");
      if let Some(run) = self.mesh_probe.as_mut() {
        run.state.abort("probe not sent (not connected)");
      }
    }
  }

  /// Drive a running mesh acquisition one frame: fold a resolved latch result into the state machine (saving the
  /// finished mesh on completion), or run the shared lost-push fallback. Mirrors [`Self::pump_datum`]. Returns
  /// whether anything changed. No-op when no run is active or it is not awaiting a point.
  pub(crate) fn pump_mesh(&mut self) -> bool {
    use crate::app::probe_flow::{AwaitAction, await_action};
    let (issued_at, polled_at) = match self.mesh_probe.as_ref() {
      Some(run) if run.state.is_probing() => match &run.touch_fallback {
        Some(f) => (f.issued_at, f.polled_at),
        None => (Instant::now(), None),
      },
      _ => return false,
    };
    // The latch must belong to THIS flow. Gone or another kind ⇒ abort rather than act on someone else's `[PRB:]`.
    match self.view.probe_op.as_ref() {
      Some(op) if op.kind == crate::app::view_state::ProbeKind::Mesh => {}
      _ => {
        if let Some(run) = self.mesh_probe.as_mut() {
          run.state.abort("probe latch lost");
        }
        return true;
      }
    }
    // Resolved ⇒ fold the Z into the mesh; if that completed the grid, persist it.
    let resolved = self.view.probe_op.as_ref().filter(|op| !op.awaiting).and_then(|op| op.last.clone());
    if let Some(outcome) = resolved {
      let mut just_done = false;
      if let Some(run) = self.mesh_probe.as_mut() {
        run.state.on_probe_result(&outcome);
        run.touch_fallback = None;
        just_done = run.state.is_done();
      }
      self.view.clear_probe_op();
      if just_done {
        self.finish_mesh_probe();
      }
      return true;
    }
    // Still awaiting ⇒ shared lost-push fallback, gated on the point having demonstrably finished.
    let now = Instant::now();
    let busy_now = self.view.status.as_ref().is_some_and(|s| is_probe_cycle_state(s.machine_state.state));
    if busy_now
      && let Some(run) = self.mesh_probe.as_mut()
      && let Some(f) = run.touch_fallback.as_mut()
    {
      f.seen_cycle = true;
    }
    let (polled, seen_cycle) = match self.mesh_probe.as_ref().and_then(|r| r.touch_fallback.as_ref()) {
      Some(f) => (f.polled, f.seen_cycle),
      None => return false,
    };
    let probe_finished = seen_cycle && !busy_now;
    let since_issue = now.duration_since(issued_at);
    let since_poll = polled_at.map(|at| now.duration_since(at)).unwrap_or(Duration::ZERO);
    match await_action(polled, probe_finished, since_issue, since_poll) {
      AwaitAction::Wait => false,
      AwaitAction::Poll => {
        if let Some(run) = self.mesh_probe.as_mut()
          && let Some(f) = run.touch_fallback.as_mut()
        {
          f.polled = true;
          f.polled_at = Some(now);
        }
        self.send_line("$#".to_string());
        true
      }
      AwaitAction::GiveUp(reason) => {
        if let Some(run) = self.mesh_probe.as_mut() {
          run.state.abort(reason.clone());
          run.touch_fallback = None;
        }
        self.view.fail_probe(reason);
        true
      }
    }
  }

  /// Persist a completed height-map: snapshot the filled mesh into [`crate::profile::Profile::mesh`], invalidate
  /// the corrected-program cache (it may have been built on an older/absent mesh), and save the profile. Called by
  /// [`Self::pump_mesh`] the moment the last point resolves. A no-op unless the run is actually done.
  fn finish_mesh_probe(&mut self) {
    let mut mesh = match self.mesh_probe.as_ref() {
      Some(run) if run.state.is_done() => run.state.mesh().clone(),
      _ => return,
    };
    // Pin `max_height` to the exact grid maximum on completion: `set_delta`'s O(1) incremental update only rises,
    // so a re-probe that lowered a node could leave it stale-high. One authoritative O(N) pass here fixes that.
    mesh.recompute_max_height();
    self.profile.mesh = Some(mesh);
    // A freshly probed mesh changes the correction inputs — drop any cached corrected program AND any stored
    // simulation (both were built against the old/absent mesh) so the next stream/simulate recomputes against the
    // new surface. The simulation's per-line timeline indexes the corrected program, so a stale one would misdrive
    // the live ETA once the new mesh changes the corrected line count.
    self.invalidate_autolevel();
    self.clear_simulation();
    self.save_profile();
    self.notice("height-map complete — saved to the profile".to_string());
  }

  /// Clear the persisted height-map from the profile and disarm autolevel's use of it (invalidate the cache).
  pub(crate) fn mesh_clear(&mut self) {
    if self.profile.mesh.is_none() {
      self.notice("no saved height-map to clear".to_string());
      return;
    }
    self.profile.mesh = None;
    self.invalidate_autolevel();
    self.clear_simulation();
    self.save_profile();
    self.notice("cleared the saved height-map".to_string());
  }

  /// Apply the saved height-map to the next stream: arm autolevel and invalidate the corrected cache so the next
  /// stream/simulate re-corrects against `profile.mesh`. A no-op (with a notice) when no mesh has been saved.
  pub(crate) fn apply_saved_mesh(&mut self) {
    if self.profile.mesh.is_none() {
      self.notice("no saved height-map to apply — probe one first".to_string());
      return;
    }
    self.ui.autolevel_enabled = true;
    self.invalidate_autolevel();
    self.clear_simulation();
    self.notice("armed autolevel with the saved height-map".to_string());
  }

  /// Cancel every OTHER in-flight probe op so only one is ever armed at a time (the cross-contamination guard).
  /// Called when any probe flow starts. The shared latch is kind-routed, but clearing the others' pending state
  /// here means a stale follow-up can never act on a new flow's `[PRB:]`.
  fn cancel_probe_ops_except(&mut self, keep: ProbeOpSlot) {
    if keep != ProbeOpSlot::ZeroZ {
      self.pending_zero_z = None;
    }
    if keep != ProbeOpSlot::Wizard {
      self.wizard = None;
    }
    if keep != ProbeOpSlot::Datum {
      self.datum = None;
    }
    if keep != ProbeOpSlot::Mesh {
      self.mesh_probe = None;
    }
    if keep != ProbeOpSlot::Sweep {
      self.sweep = None;
    }
  }

  /// Start a Phase 2 180°-flip center-verify (DOC-11 §2.1): a two-angle sweep at θ and θ+180 along `axis`/`dir`.
  /// The operator jogs the approach and triggers each touch; on completion [`crate::app::flip_verify`] computes the
  /// residual and offers a position-independent `G10 L2` correction. Cancels any other in-flight probe op.
  pub(crate) fn flip_verify_start(&mut self, angle_deg: f64, axis: crate::app::intent::Axis, dir: crate::app::intent::Dir) {
    self.cancel_probe_ops_except(ProbeOpSlot::Sweep);
    let angles = vec![angle_deg, angle_deg + 180.0];
    self.sweep = Some(SweepRun {
      kind: crate::app::view_state::ProbeKind::FlipVerify,
      sweep: crate::app::angle_sweep::AngleSweep::new(angles, axis, dir),
      // Use the operator-tuned bench params (clearance / side-probe Z / settle / feed / depth), not the placeholder
      // defaults — a flip-verify run with the default side-probe Z would touch at the wrong height and produce a
      // garbage residual that could be applied as a bogus `G10 L2` WCS correction.
      params: self.ui.rotary_bench,
      touch_fallback: None,
    });
    self.notice(format!("180°-flip verify: probe {} at A{angle_deg:.1}° then A{:.1}°", axis.letter(),
      angle_deg + 180.0));
  }

  /// Start a Phase 2 runout report (DOC-11 §2.2): an N-angle sweep (evenly spaced from `start_deg`) along
  /// `axis`/`dir`. READ-ONLY — on completion [`crate::app::runout`] reports TIR / eccentricity; nothing is written.
  /// Cancels any other in-flight probe op. `n < 2` is rejected (TIR needs at least two readings).
  pub(crate) fn runout_start(&mut self, n: usize, start_deg: f64, axis: crate::app::intent::Axis, dir: crate::app::intent::Dir) {
    if n < 2 {
      self.notice("runout needs at least 2 angles".to_string());
      return;
    }
    self.cancel_probe_ops_except(ProbeOpSlot::Sweep);
    let angles = crate::app::angle_sweep::evenly_spaced_angles(n, start_deg);
    self.sweep = Some(SweepRun {
      kind: crate::app::view_state::ProbeKind::Runout,
      sweep: crate::app::angle_sweep::AngleSweep::new(angles, axis, dir),
      // The runout sweep must probe at the operator's dialed-in side-probe height too; the placeholder default
      // would touch off the flank and report meaningless TIR / eccentricity.
      params: self.ui.rotary_bench,
      touch_fallback: None,
    });
    self.notice(format!("runout report: {n} angles along {}", axis.letter()));
  }

  /// Trigger the sweep's next touch: ask the engine for the touch due at the current angle, emit its rotary-safe
  /// probe lines, and arm the latch (with the run's [`ProbeKind`]) so [`Self::pump_sweep`] folds the result back.
  /// Inert if no sweep is running, one is already probing, or no angle remains.
  pub(crate) fn sweep_probe(&mut self) {
    let Some(run) = self.sweep.as_mut() else {
      self.notice("no verify/runout sweep running".to_string());
      return;
    };
    if run.sweep.is_probing() {
      self.notice("sweep probe already in progress".to_string());
      return;
    }
    let Some(touch) = run.sweep.begin_next_touch() else {
      self.notice("no sweep touch is due".to_string());
      return;
    };
    let (kind, params) = (run.kind, run.params);
    let lines = crate::app::rotary_probe::rotary_safe_probe_lines(touch, params);
    // Only this sweep may own the latch now (cross-contamination guard), and arm it BEFORE sending so the result
    // always finds an op awaiting it.
    self.cancel_probe_ops_except(ProbeOpSlot::Sweep);
    self.view.begin_probe(kind);
    if let Some(run) = self.sweep.as_mut() {
      run.touch_fallback =
        Some(TouchFallback { issued_at: Instant::now(), polled: false, polled_at: None, seen_cycle: false });
    }
    let mut all_sent = true;
    for line in lines {
      self.view.note_sent(line.clone());
      if !self.send_command(Command::SendLine(line)) {
        all_sent = false;
        break;
      }
    }
    if !all_sent {
      self.view.fail_probe("sweep probe not sent (not connected)");
      if let Some(run) = self.sweep.as_mut() {
        run.sweep.abort("probe not sent (not connected)");
      }
    }
  }

  /// Write the flip-verify's offered `G10 L2` correction (the verified axis only, never A). Inert unless a
  /// completed flip-verify sweep is present with a computable two-reading result.
  pub(crate) fn flip_verify_write_correction(&mut self) {
    use crate::app::flip_verify::FlipResult;
    use crate::app::rotary_center::Wcs;
    let Some(run) = self.sweep.as_ref() else {
      return;
    };
    if run.kind != crate::app::view_state::ProbeKind::FlipVerify || !run.sweep.is_done() {
      self.notice("no flip-verify correction to write yet".to_string());
      return;
    }
    // The probed axis is the sweep's axis; the readings are complete (is_done). Recover the axis from the first
    // touch description is unnecessary — the run carries it via the sweep's touches, so probe along the same axis.
    let axis = run.sweep.probe_axis();
    let Some(result) = FlipResult::from_readings(axis, run.sweep.readings()) else {
      self.notice("flip-verify needs exactly two readings".to_string());
      return;
    };
    // The correction is a RELATIVE shift of the current work origin by the measured residual, so it needs the
    // current WCO on the verified axis (the machine coordinate of work-0). Without a status report carrying `WCO:`
    // we cannot compute it safely — surface that rather than guess.
    let Some(&current_origin) = self.view.last_wco.get(axis.index()) else {
      self.notice("no WCO yet — request a status report before applying the correction".to_string());
      return;
    };
    let line = result.offer_g10(Wcs::Active, current_origin);
    self.send_line(line);
    self.notice("shifted the active WCS origin by the flip-verify residual".to_string());
  }

  /// Cancel any running Phase 2 sweep, discarding its state.
  pub(crate) fn sweep_cancel(&mut self) {
    self.sweep = None;
  }

  /// Drive a running Phase 2 sweep one frame: fold a resolved latch result into the shared engine, or run the
  /// SAME completion-gated lost-push fallback the other flows use. Kind-routed: acts only when the latch belongs
  /// to THIS sweep's kind (FlipVerify/Runout). Returns whether anything changed. The single pump for both Phase 2
  /// wizards — they differ only in the post-completion compute, which the view/`flip_verify_write_correction` do.
  pub(crate) fn pump_sweep(&mut self) -> bool {
    use crate::app::probe_flow::{AwaitAction, await_action};
    // Only act while a touch is in flight; copy the fallback stamps up front to release the borrow before mutating.
    let (kind, issued_at, polled_at) = match self.sweep.as_ref() {
      Some(run) if run.sweep.is_probing() => {
        let (issued, polled) = match &run.touch_fallback {
          Some(f) => (f.issued_at, f.polled_at),
          None => (Instant::now(), None),
        };
        (run.kind, issued, polled)
      }
      _ => return false,
    };
    // The latch must belong to THIS sweep's kind. Gone or another kind ⇒ abort the sweep rather than act on
    // someone else's `[PRB:]`.
    match self.view.probe_op.as_ref() {
      Some(op) if op.kind == kind => {}
      _ => {
        if let Some(run) = self.sweep.as_mut() {
          run.sweep.abort("probe latch lost");
        }
        return true;
      }
    }
    // Resolved ⇒ fold the outcome into the engine and finish the touch.
    let resolved = self.view.probe_op.as_ref().filter(|op| !op.awaiting).and_then(|op| op.last.clone());
    if let Some(outcome) = resolved {
      if let Some(run) = self.sweep.as_mut() {
        run.sweep.on_probe_result(&outcome);
        run.touch_fallback = None;
      }
      self.view.clear_probe_op();
      return true;
    }
    // Still awaiting ⇒ shared lost-push fallback, gated on the touch having demonstrably finished.
    let now = Instant::now();
    let busy_now = self.view.status.as_ref().is_some_and(|s| is_probe_cycle_state(s.machine_state.state));
    if busy_now && let Some(run) = self.sweep.as_mut() && let Some(f) = run.touch_fallback.as_mut() {
      f.seen_cycle = true;
    }
    let (polled, seen_cycle) = match self.sweep.as_ref().and_then(|r| r.touch_fallback.as_ref()) {
      Some(f) => (f.polled, f.seen_cycle),
      None => return false,
    };
    let probe_finished = seen_cycle && !busy_now;
    let since_issue = now.duration_since(issued_at);
    let since_poll = polled_at.map(|at| now.duration_since(at)).unwrap_or(Duration::ZERO);
    match await_action(polled, probe_finished, since_issue, since_poll) {
      AwaitAction::Wait => false,
      AwaitAction::Poll => {
        if let Some(run) = self.sweep.as_mut() && let Some(f) = run.touch_fallback.as_mut() {
          f.polled = true;
          f.polled_at = Some(now);
        }
        self.send_line("$#".to_string());
        true
      }
      AwaitAction::GiveUp(reason) => {
        if let Some(run) = self.sweep.as_mut() {
          run.sweep.abort(reason.clone());
          run.touch_fallback = None;
        }
        self.view.fail_probe(reason);
        true
      }
    }
  }
}
