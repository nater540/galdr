//! The hardened Z touch-off flow: probe → await result → validate success → only THEN zero.
//!
//! The old `probe_z()` fired `G38.2 Z-…` and *immediately* fired the `G10 L20` zeroing line with no success
//! check — saved today only by alarm-ordering, which is a race, not validation. This module replaces that with a
//! result-gated sequence. Because the egui shell cannot block, the decision is a small per-frame state machine:
//! a [`PendingZeroZ`] records the issued probe and the zero line it *would* send, and each frame the pure
//! [`decide`] function inspects the probe latch ([`super::view_state::ProbeOp`]) and the elapsed wall-clock to
//! choose the next [`ZeroZAction`]. Keeping the decision pure (no egui, no `Instant::now()` inside) makes the
//! whole gate — success-zeroing, the push-or-poll fallback, and the give-up failure — unit-testable without a
//! window or real hardware. The shell does only the I/O the action names.
//!
//! Push-or-poll fallback: Galdr's firmware pushes `[PRB:]` immediately, but grblHAL can be configured to suppress
//! that push (the last result still retrievable via `$#`). So if no result arrives within [`PUSH_TIMEOUT`] of
//! issuing the probe, the flow queries `$#` once; its `[PRB:]` answer parses through the same path and resolves
//! the latch. If even the poll yields nothing within a further [`POLL_TIMEOUT`], the flow gives up and fails the
//! op so the operator is told rather than left waiting forever.

use std::time::Duration;

use super::view_state::ProbeOutcome;

/// How long to wait for the immediate `[PRB:]` push after issuing the probe before falling back to a `$#` poll.
/// Generous relative to a probe's own motion + the firmware's report latency: a slow deep probe at a low feed
/// can take seconds, and we must not poll while the probe is still legitimately travelling. This bounds only the
/// *post-result* push latency in practice, since a real probe resolves (push or alarm) the moment it triggers.
pub const PUSH_TIMEOUT: Duration = Duration::from_secs(15);

/// How long to wait for the `$#` poll's `[PRB:]` answer after sending it before giving up entirely. A `$#` query
/// is answered within a status-poll interval, so this is short; exceeding it means the result is genuinely lost.
pub const POLL_TIMEOUT: Duration = Duration::from_secs(2);

/// An ABSOLUTE backstop, measured from probe issue, after which the op gives up no matter what — even if a cycle
/// was never observed (a probe that completed between status samples) and the push was lost. Without this a fast
/// probe whose `seen_cycle` never latched AND whose push dropped would hang the op forever. Generous: well past
/// `PUSH_TIMEOUT` so a legitimately slow/deep probe still completing is never cut off — it only catches the
/// genuinely-stuck case.
pub const ABSOLUTE_GIVE_UP: Duration = Duration::from_secs(30);

/// The follow-up the shell tracks after issuing a hardened Z probe: the plate thickness work-Z should read at
/// the contact, and whether the `$#` push-or-poll fallback has already been triggered (sent at most once).
///
/// The zero is computed from the probe's CONTACT machine-Z on resolution, NOT pre-built: a `G10 L20` (set work-Z
/// from the *current* position) deferred across the lost-push window would zero off wherever the tool happens to
/// sit if the operator jogged in the meantime. Instead we emit `G10 L2 P0 Z<contact_Z − plate_thickness>` — a
/// position-INDEPENDENT WCS write derived from the `[PRB:]` contact point, so the zero is correct regardless of
/// where the tool now is. The contact-Z arrives in the resolved [`ProbeOutcome::Success`] position (axis 2).
#[derive(Debug, Clone, PartialEq)]
pub struct PendingZeroZ {
  /// The thickness (mm) work-Z should read AT the contact point — i.e. the touch-plate thickness, so removing the
  /// plate leaves Z0 at the copper surface. The `G10 L2` Z origin is `contact_machine_Z − plate_thickness`.
  pub plate_thickness: f64,
  /// Whether the `$#` fallback poll has been sent. Starts `false`; set once the push timeout elapses and the
  /// poll is issued, so the give-up deadline ([`POLL_TIMEOUT`]) is then measured from that point.
  pub polled: bool,
  /// Whether the machine has been observed in a probe CYCLE (`Run`/`Hold`/`Jog`/`Home`) since the probe was
  /// issued. This is the startup-race guard: right after issuing the probe the last status may still read `Idle`
  /// for a frame or two before the machine reports `Run`, so a bare "Idle" must NOT be trusted as "probe
  /// finished" until a cycle has actually been seen. Set by [`Self::observe_busy`]; consumed by
  /// [`Self::probe_finished`].
  pub seen_cycle: bool,
}

impl PendingZeroZ {
  /// Begin tracking a freshly-issued Z probe whose successful contact should zero work-Z to `plate_thickness`.
  /// The cycle has not yet been observed, so the `$#` fallback is gated until the machine demonstrably enters
  /// (then leaves) a cycle.
  pub fn new(plate_thickness: f64) -> Self {
    PendingZeroZ { plate_thickness, polled: false, seen_cycle: false }
  }

  /// Record that the machine is currently in a probe cycle (not idle/disconnected). Called by the shell each
  /// frame from the live status; latches `seen_cycle` so a later return to `Idle` can be trusted as completion.
  pub fn observe_busy(&mut self) {
    self.seen_cycle = true;
  }

  /// Whether the probe has demonstrably FINISHED — the machine is back at `Idle` (`!busy_now`) AND a cycle was
  /// observed first (so the pre-`Run` startup window cannot be mistaken for completion). This is the gate the
  /// `$#` lost-push fallback runs behind. `busy_now` is the machine's current in-cycle state this frame.
  pub fn probe_finished(&self, busy_now: bool) -> bool {
    self.seen_cycle && !busy_now
  }
}

/// The action the shell should take this frame for a pending Z touch-off, decided purely from the latch state
/// and elapsed time.
#[derive(Debug, Clone, PartialEq)]
pub enum ZeroZAction {
  /// Still awaiting a result and within timeout: do nothing this frame.
  Wait,
  /// The probe succeeded: send this `G10 L2` line (built from the contact machine-Z, so it is
  /// position-independent) to zero work-Z, then surface success. Clears the pending.
  Zero(String),
  /// The probe failed (a `:0` flag, an alarm/error, or the poll gave up): surface this reason; send nothing
  /// destructive. Clears the pending.
  Fail(String),
  /// No push arrived within [`PUSH_TIMEOUT`]: query `$#` once to retrieve the last probe result. Marks `polled`.
  Poll,
  /// The `$#` poll also yielded nothing within [`POLL_TIMEOUT`]: give up. The shell fails the latch with this
  /// reason and surfaces it. Clears the pending.
  GiveUp(String),
}

/// Build the position-independent `G10 L2 P0 Z<...>` zeroing line from a probe's CONTACT machine-Z so work-Z
/// reads `plate_thickness` at that contact (`origin = contact_z − plate_thickness`). Returns `None` if the probe
/// position carries no Z axis (index 2) — a malformed result we must not zero off. Public so the shell can reuse
/// the exact form and tests can assert it.
pub fn zero_z_line(position: &[f64], plate_thickness: f64) -> Option<String> {
  let contact_z = position.get(2)?;
  Some(format!("G10 L2 P0 Z{:.3}", contact_z - plate_thickness))
}

/// The shared, KIND-AGNOSTIC lost-push fallback decision for a probe that is still AWAITING its result. This is
/// the common machinery the ZeroZ touch-off and the rotary wizard both run when no `[PRB:]` has resolved the
/// latch yet — factored out so the two flows cannot drift apart. The caller supplies the timing/gate state; this
/// returns only the next fallback step (it never decides what to do with a result — that is the caller's job).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AwaitAction {
  /// Keep waiting — within timeout, or the probe is still in flight.
  Wait,
  /// The push appears lost (probe finished, past [`PUSH_TIMEOUT`]): query `$#` once. The caller marks `polled`.
  Poll,
  /// Give up — the `$#` poll also timed out, or the absolute backstop fired. Carries the operator-facing reason.
  GiveUp(String),
}

/// Decide the next [`AwaitAction`] for a probe whose result has NOT yet landed. Shared by ZeroZ and the wizard.
/// - `polled`: whether `$#` has already been sent (then the give-up runs off `since_poll`).
/// - `probe_finished`: whether the machine has demonstrably finished the cycle (so a lost push is plausible).
/// - `since_issue` / `since_poll`: elapsed since the probe line / the `$#` poll.
///
/// While the probe is still in flight (`!probe_finished`, not yet polled) it WAITS regardless of elapsed — a slow
/// probe legitimately travels long, and polling `$#` mid-cycle would read the previous probe's stale result. An
/// ABSOLUTE backstop ([`ABSOLUTE_GIVE_UP`] from issue) catches the case where the completion gate never trips (a
/// fast probe that finished between status samples) so the op can never hang forever.
pub fn await_action(polled: bool, probe_finished: bool, since_issue: Duration, since_poll: Duration) -> AwaitAction {
  if since_issue >= ABSOLUTE_GIVE_UP {
    return AwaitAction::GiveUp("probe result never arrived (timed out)".to_string());
  }
  if polled {
    if since_poll >= POLL_TIMEOUT {
      AwaitAction::GiveUp("probe result never arrived (no push, $# poll timed out)".to_string())
    } else {
      AwaitAction::Wait
    }
  } else if probe_finished && since_issue >= PUSH_TIMEOUT {
    AwaitAction::Poll
  } else {
    AwaitAction::Wait
  }
}

/// Decide the next [`ZeroZAction`] for a pending Z touch-off. Inputs:
/// - `outcome`: the latch's resolved outcome, or `None` while still awaiting.
/// - `pending`: the tracked follow-up (the zero line and whether `$#` was already polled).
/// - `probe_finished`: whether the machine has demonstrably FINISHED the probe cycle (back to Idle, having been
///   seen in a cycle first — see [`PendingZeroZ::probe_finished`]). The `$#` fallback is for a LOST push, and a
///   lost push means the probe already completed; so the `Poll`/`GiveUp` timeouts may run only once this holds.
/// - `since_issue`: wall-clock elapsed since the probe line was sent (paces the push timeout, before a poll).
/// - `since_poll`: wall-clock elapsed since the `$#` poll was sent, once `pending.polled` (paces the give-up).
///
/// Pure and total: a resolved outcome always wins over the timeouts (a result that landed must be honoured even
/// if a frame is late), so success/failure are checked first; only an *unresolved* op consults the clock — and
/// only once the probe has finished. While the probe is still IN FLIGHT (`!probe_finished`) the result is `Wait`
/// no matter the elapsed time: a slow / no-contact probe legitimately travels longer than [`PUSH_TIMEOUT`] (e.g.
/// 20 mm at 50 mm/min ≈ 24 s, and a no-contact probe runs to end-of-travel), and polling `$#` mid-probe would
/// return the PREVIOUS probe's stale result — which the latch must never resolve on. The push (or the
/// `ALARM:5`/`[PRB:]` on a no-contact probe) resolves the latch the instant the probe completes, so the idle gate
/// only ever matters for a genuinely lost push.
pub fn decide(
  outcome: Option<&ProbeOutcome>,
  pending: &PendingZeroZ,
  probe_finished: bool,
  since_issue: Duration,
  since_poll: Duration,
) -> ZeroZAction {
  match outcome {
    // The result landed (push or poll): honour it regardless of the clock. Build the zero from the CONTACT
    // machine-Z so it is position-independent; a result with no Z axis is malformed and fails rather than zeroing.
    Some(ProbeOutcome::Success { position }) => match zero_z_line(position, pending.plate_thickness) {
      Some(line) => ZeroZAction::Zero(line),
      None => ZeroZAction::Fail("probe result has no Z axis".to_string()),
    },
    Some(ProbeOutcome::Failure { reason }) => ZeroZAction::Fail(reason.clone()),
    // No result yet: defer to the shared lost-push fallback (same machinery the wizard uses), mapping its
    // kind-agnostic decision onto this flow's `$#`/give-up actions.
    None => match await_action(pending.polled, probe_finished, since_issue, since_poll) {
      AwaitAction::Wait => ZeroZAction::Wait,
      AwaitAction::Poll => ZeroZAction::Poll,
      AwaitAction::GiveUp(reason) => ZeroZAction::GiveUp(reason),
    },
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  fn pending() -> PendingZeroZ {
    // A 1.0 mm plate: work-Z should read 1.0 at the contact, so the WCS Z origin is contact_z − 1.0.
    PendingZeroZ::new(1.0)
  }

  // `decide`'s third argument is the already-computed `probe_finished` gate (the shell derives it from
  // `PendingZeroZ::probe_finished(busy_now)`), so these tests pass it as an explicit boolean; the cycle-
  // observation latch that produces it is exercised by `probe_finished_requires_a_cycle_*` below.
  const FINISHED: bool = true;
  const IN_FLIGHT: bool = false;

  #[test]
  fn waits_while_awaiting_within_the_push_timeout() {
    let action = decide(None, &pending(), FINISHED, Duration::from_secs(1), Duration::ZERO);
    assert_eq!(action, ZeroZAction::Wait);
  }

  #[test]
  fn zeroes_from_the_contact_machine_z_not_a_prebuilt_line() {
    // The contact machine-Z is -2.5; with a 1.0 mm plate the WCS Z origin is -2.5 − 1.0 = -3.5, so work-Z reads
    // 1.0 AT the contact regardless of where the tool sits now. This must be a `G10 L2` (machine-coord) write.
    let outcome = ProbeOutcome::Success { position: vec![0.0, 0.0, -2.5] };
    let action = decide(Some(&outcome), &pending(), IN_FLIGHT, Duration::from_secs(1), Duration::ZERO);
    assert_eq!(action, ZeroZAction::Zero("G10 L2 P0 Z-3.500".to_string()));
  }

  #[test]
  fn the_zero_is_position_independent_built_from_contact_not_current_position() {
    // The defining property of the L2-from-contact fix: the offset depends ONLY on the contact-Z and the plate,
    // not on any "current position". Two successes with the same contact-Z yield the same line whatever else the
    // position vector carries (a later jog can change X/Y/A but never the computed Z origin).
    let a = ProbeOutcome::Success { position: vec![0.0, 0.0, -2.5, 0.0] };
    let b = ProbeOutcome::Success { position: vec![99.0, 99.0, -2.5, 45.0] };
    let la = decide(Some(&a), &pending(), IN_FLIGHT, Duration::ZERO, Duration::ZERO);
    let lb = decide(Some(&b), &pending(), IN_FLIGHT, Duration::ZERO, Duration::ZERO);
    assert_eq!(la, lb, "the zero must depend only on contact-Z + plate, not the rest of the position");
    assert_eq!(la, ZeroZAction::Zero("G10 L2 P0 Z-3.500".to_string()));
  }

  #[test]
  fn a_success_without_a_z_axis_fails_rather_than_zeroing() {
    // A malformed reading (no Z at index 2) must not produce a zero line.
    let outcome = ProbeOutcome::Success { position: vec![-1.0] };
    let action = decide(Some(&outcome), &pending(), IN_FLIGHT, Duration::ZERO, Duration::ZERO);
    assert!(matches!(action, ZeroZAction::Fail(_)));
  }

  #[test]
  fn fails_without_zeroing_on_a_failed_result() {
    let outcome = ProbeOutcome::Failure { reason: "ALARM:5 during probe".to_string() };
    let action = decide(Some(&outcome), &pending(), IN_FLIGHT, Duration::from_secs(1), Duration::ZERO);
    assert_eq!(action, ZeroZAction::Fail("ALARM:5 during probe".to_string()));
  }

  #[test]
  fn a_landed_result_wins_even_past_the_push_timeout() {
    // A success that arrived just as the push timeout elapsed must still zero — never be lost to a poll.
    let outcome = ProbeOutcome::Success { position: vec![0.0, 0.0, -1.0] };
    let action = decide(Some(&outcome), &pending(), IN_FLIGHT, PUSH_TIMEOUT + Duration::from_secs(5), Duration::ZERO);
    assert!(matches!(action, ZeroZAction::Zero(_)));
  }

  #[test]
  fn a_probe_still_in_flight_waits_past_the_push_timeout_rather_than_polling() {
    // THE REGRESSION: a slow / no-contact probe legitimately travels longer than PUSH_TIMEOUT. While the probe is
    // unfinished (`IN_FLIGHT`) the `$#` poll must NOT fire — polling mid-probe returns the PREVIOUS probe's stale
    // result, which the latch could then zero off. So an unfinished probe waits regardless of elapsed time — up to
    // the absolute backstop (kept below it here so this exercises the in-flight wait, not the backstop).
    let elapsed = PUSH_TIMEOUT + Duration::from_secs(5);
    assert!(elapsed < ABSOLUTE_GIVE_UP, "this test must stay below the absolute backstop");
    let action = decide(None, &pending(), IN_FLIGHT, elapsed, Duration::ZERO);
    assert_eq!(action, ZeroZAction::Wait, "an in-flight probe must wait, never poll $# mid-cycle");
  }

  #[test]
  fn polls_dollar_hash_once_the_probe_finished_and_the_push_timeout_elapsed() {
    // The push is lost only once the probe has FINISHED (cycle observed, now idle): then, past PUSH_TIMEOUT, poll.
    let action = decide(None, &pending(), FINISHED, PUSH_TIMEOUT, Duration::ZERO);
    assert_eq!(action, ZeroZAction::Poll);
  }

  #[test]
  fn after_polling_waits_within_the_poll_timeout() {
    let mut p = pending();
    p.polled = true;
    // Once polled, the gate no longer matters (the machine is idle anyway); the give-up runs off the poll clock.
    let action = decide(None, &p, IN_FLIGHT, PUSH_TIMEOUT + Duration::from_secs(1), Duration::from_millis(500));
    assert_eq!(action, ZeroZAction::Wait);
  }

  #[test]
  fn gives_up_when_the_poll_also_times_out() {
    let mut p = pending();
    p.polled = true;
    let action = decide(None, &p, IN_FLIGHT, PUSH_TIMEOUT + POLL_TIMEOUT, POLL_TIMEOUT);
    assert!(matches!(action, ZeroZAction::GiveUp(_)));
  }

  #[test]
  fn the_absolute_backstop_gives_up_even_when_no_cycle_was_ever_observed() {
    // The pathological hang: a fast probe finished between status samples (so `seen_cycle` never latched →
    // IN_FLIGHT forever) AND its push was lost (no result, never polled). Without the absolute backstop nothing
    // would ever fire. Past ABSOLUTE_GIVE_UP from issue, the op must give up rather than hang.
    let action = decide(None, &pending(), IN_FLIGHT, ABSOLUTE_GIVE_UP, Duration::ZERO);
    assert!(matches!(action, ZeroZAction::GiveUp(_)), "the absolute backstop must fire even without a cycle/poll");
    // Just before the backstop, an in-flight probe still waits (it might still be legitimately travelling).
    let action = decide(None, &pending(), IN_FLIGHT, ABSOLUTE_GIVE_UP - Duration::from_secs(1), Duration::ZERO);
    assert_eq!(action, ZeroZAction::Wait, "before the backstop an in-flight probe waits");
  }

  #[test]
  fn probe_finished_requires_a_cycle_to_have_been_observed_first() {
    // The startup-race guard: a bare "Idle" (no cycle seen yet) is the pre-Run window, NOT completion.
    let p = pending();
    assert!(!p.probe_finished(false), "idle before any cycle is the startup window, not a finished probe");
    // Once a cycle is observed and the machine returns to idle, the probe is finished.
    let mut p = pending();
    p.observe_busy();
    assert!(p.probe_finished(false), "idle after a cycle was observed is a finished probe");
    assert!(!p.probe_finished(true), "still in-cycle is not finished");
  }

  // The shared lost-push fallback (`await_action`) — exercised directly so the wizard, which also uses it, is
  // covered without a full GUI run.
  #[test]
  fn await_action_waits_in_flight_polls_when_finished_and_gives_up_on_timeouts() {
    // In flight (not finished, not polled): wait even past the push timeout.
    assert_eq!(await_action(false, false, PUSH_TIMEOUT + Duration::from_secs(5), Duration::ZERO), AwaitAction::Wait);
    // Finished + past push timeout, not yet polled: poll `$#`.
    assert_eq!(await_action(false, true, PUSH_TIMEOUT, Duration::ZERO), AwaitAction::Poll);
    // Polled, within the poll window: wait.
    assert_eq!(await_action(true, true, PUSH_TIMEOUT, Duration::from_millis(500)), AwaitAction::Wait);
    // Polled, past the poll window: give up.
    assert!(matches!(await_action(true, true, PUSH_TIMEOUT, POLL_TIMEOUT), AwaitAction::GiveUp(_)));
    // Absolute backstop: give up even when never finished/polled.
    assert!(matches!(await_action(false, false, ABSOLUTE_GIVE_UP, Duration::ZERO), AwaitAction::GiveUp(_)));
  }
}
