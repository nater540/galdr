//! Pure override-stepping logic: turn a desired override percentage into the ordered real-time bytes that
//! step the firmware's live override from its current value to the target.
//!
//! grbl's feed and spindle overrides are *relative*, not absolute: the host has only "+10% / −10% / +1% /
//! −1% / reset-to-100%" real-time commands (`docs/gcode-streaming.md` §override commands), never a "set to N%"
//! command. So a slider — which expresses an absolute target — must be realised as a sequence of those
//! relative steps applied against the override the firmware last reported in the `Ov:` status field. This
//! module is that translation, kept framework-agnostic and pure so the stepping is unit-tested without a GUI
//! or a live link; the view just renders the slider and the shell emits the [`RealtimeCommand`]s this returns.
//!
//! The firmware clamps feed/spindle overrides to 10%–200% and ignores a command that would not change the
//! value, so we mirror that clamp here and emit the minimal step sequence: a `reset` short-circuit when the
//! target is exactly 100%, otherwise a greedy run of ±10% decades followed by ±1% units. The result is the
//! smallest set of bytes that moves the live override onto the requested percentage.

use crate::protocol::RealtimeCommand;

/// The lower bound grbl clamps feed/spindle overrides to (percent). A request below this is raised to it.
pub const OVERRIDE_MIN: u32 = 10;
/// The upper bound grbl clamps feed/spindle overrides to (percent). A request above this is lowered to it.
pub const OVERRIDE_MAX: u32 = 200;
/// The neutral 100% override the `reset` command snaps to, and the value a slider centres on.
pub const OVERRIDE_NEUTRAL: u32 = 100;

/// The maximum number of status observations a post-release [`OverrideFeedback::Holding`] hold tolerates without
/// the firmware reaching the committed target before it releases anyway (Bug 7). The firmware crawls one ±10/±1
/// step per `?`/`Ov:` round-trip, so the worst-case legal ramp across the full 10–200 span is ~19 coarse steps;
/// this bound generously exceeds that, so the converging case always releases on arrival first, while a firmware
/// that will NEVER land on the target (adjusted in ALARM, an external change, a short settle) cannot pin a stale
/// value on the handle indefinitely — once the bound is spent the hold falls back to mirroring live.
pub const OVERRIDE_HOLD_MAX_OBSERVATIONS: u32 = 40;

/// Which relative-override channel a stepping request targets. Rapid is deliberately absent: grbl exposes only
/// the 100/50/25 rapid presets (no ±), so the rapid control stays a preset picker, not a slider, and never
/// routes through this stepping. Both variants here carry the full ±10 / ±1 / reset command set.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OverrideAxis {
  /// The feed-rate override (`0x90` reset, `0x91`/`0x92` ±10%, `0x93`/`0x94` ±1%).
  Feed,
  /// The spindle-speed override (`0x99` reset, `0x9A`/`0x9B` ±10%, `0x9C`/`0x9D` ±1%).
  Spindle,
}

impl OverrideAxis {
  /// The reset-to-100% command for this axis.
  fn reset(self) -> RealtimeCommand {
    match self {
      OverrideAxis::Feed => RealtimeCommand::FeedOverrideReset,
      OverrideAxis::Spindle => RealtimeCommand::SpindleOverrideReset,
    }
  }

  /// The coarse ±10% command for this axis in the given direction (`up` = +10%).
  fn coarse(self, up: bool) -> RealtimeCommand {
    match (self, up) {
      (OverrideAxis::Feed, true) => RealtimeCommand::FeedOverridePlus10,
      (OverrideAxis::Feed, false) => RealtimeCommand::FeedOverrideMinus10,
      (OverrideAxis::Spindle, true) => RealtimeCommand::SpindleOverridePlus10,
      (OverrideAxis::Spindle, false) => RealtimeCommand::SpindleOverrideMinus10,
    }
  }

  /// The fine ±1% command for this axis in the given direction (`up` = +1%).
  fn fine(self, up: bool) -> RealtimeCommand {
    match (self, up) {
      (OverrideAxis::Feed, true) => RealtimeCommand::FeedOverridePlus1,
      (OverrideAxis::Feed, false) => RealtimeCommand::FeedOverrideMinus1,
      (OverrideAxis::Spindle, true) => RealtimeCommand::SpindleOverridePlus1,
      (OverrideAxis::Spindle, false) => RealtimeCommand::SpindleOverrideMinus1,
    }
  }
}

/// Clamp a requested override percentage into grbl's accepted 10%–200% range, matching the firmware's own
/// clamp so the host never asks for a value the firmware would silently refuse and then mis-track.
pub fn clamp_override(target: u32) -> u32 {
  target.clamp(OVERRIDE_MIN, OVERRIDE_MAX)
}

/// Build the minimal ordered sequence of real-time override commands that moves `axis`'s live override from
/// `current` percent to `target` percent. The target is first clamped to grbl's 10%–200% range.
///
/// The sequence is minimal and deterministic:
/// - If the clamped target is exactly 100%, a single `reset` is emitted — the firmware snaps to 100% in one
///   byte regardless of where it was, which is both fewer bytes and more robust than stepping (a `current`
///   that drifted from the firmware's true value still lands exactly on 100%).
/// - Otherwise the difference is covered greedily: one ±10% command per whole decade of difference, then one
///   ±1% command per leftover unit. E.g. 100→137 emits +10,+10,+10,+1,+1,+1,+1 (3 decades, 7 units).
/// - If `current` already equals the clamped target, the sequence is empty — nothing to send. (The firmware
///   would ignore a no-op step anyway, but emitting nothing keeps the link quiet.)
///
/// `current` is the override the firmware last reported in `Ov:`; passing a stale value at worst sends a few
/// extra ±1 steps, never an unbounded run, because the magnitude is bounded by the clamped 10–200 span.
pub fn override_commands(current: u32, target: u32, axis: OverrideAxis) -> Vec<RealtimeCommand> {
  let target = clamp_override(target);
  if target == OVERRIDE_NEUTRAL {
    return vec![axis.reset()];
  }
  // `current` is also clamped so a wild reported value cannot inflate the step count beyond the legal span.
  let current = clamp_override(current);
  if current == target {
    return Vec::new();
  }

  let up = target > current;
  // Work in absolute difference; direction is carried by `up`. Decades first, then units.
  let diff = target.abs_diff(current);
  let decades = diff / 10;
  let units = diff % 10;

  let mut commands = Vec::with_capacity((decades + units) as usize);
  for _ in 0..decades {
    commands.push(axis.coarse(up));
  }
  for _ in 0..units {
    commands.push(axis.fine(up));
  }
  commands
}

/// One axis's slot in the [`OverrideTracker`]: the host's best estimate of the firmware's live override and
/// whether a commanded change is still in flight (told to the firmware but not yet confirmed by a report).
#[derive(Debug, Default, Clone, PartialEq, Eq)]
struct AxisEstimate {
  /// The host's current estimate of the firmware's override (percent), or `None` until a report or a command
  /// seeds it. Stepping bases on this rather than the lagging `Ov:` field of the last status report.
  estimate: Option<u32>,
  /// Whether a commanded move is still in flight — the firmware has been told to step but no report has yet
  /// shown it reaching the commanded value. While awaiting, a report still showing the pre-command value is the
  /// lagging poll and is ignored as a stepping base; once a report confirms the value, this clears.
  awaiting: bool,
}

/// Tracks the host's estimate of each override axis so a slider commit steps from the value the firmware will
/// actually be at, not the lagging `Ov:` field of the last status report.
///
/// grbl's overrides are *relative*, so two slider commits inside one status-poll interval would otherwise both
/// step from the same stale `current` and overshoot: 100→150 then 150→120, each computed from a `current` still
/// reporting 100, lands the firmware at 170 instead of 120. The tracker remembers the value it last commanded
/// and steps from that until a report confirms the move landed, while still adopting an externally-driven
/// override (e.g. a firmware reset to 100% on alarm) once no command is in flight.
#[derive(Debug, Default, Clone)]
pub struct OverrideTracker {
  feed: AxisEstimate,
  spindle: AxisEstimate,
}

impl OverrideTracker {
  /// The mutable per-axis slot.
  fn slot(&mut self, axis: OverrideAxis) -> &mut AxisEstimate {
    match axis {
      OverrideAxis::Feed => &mut self.feed,
      OverrideAxis::Spindle => &mut self.spindle,
    }
  }

  /// Build the step sequence to move `axis` to `target`, basing it on the host's estimate when one exists (a
  /// prior command or a confirmed report) and otherwise on `reported` (the firmware's last `Ov:` value). The
  /// commanded target becomes the new estimate immediately, so a follow-up commit before the next poll steps
  /// from where this command leaves the firmware, not the value the lagging report still shows.
  pub fn command(&mut self, axis: OverrideAxis, reported: u32, target: u32) -> Vec<RealtimeCommand> {
    let base = self.slot(axis).estimate.unwrap_or_else(|| clamp_override(reported));
    let commands = override_commands(base, target, axis);
    let slot = self.slot(axis);
    slot.estimate = Some(clamp_override(target));
    // A move is in flight only if we actually emitted steps; a no-op commit leaves nothing to confirm.
    slot.awaiting = !commands.is_empty();
    commands
  }

  /// Fold a freshly-reported `Ov:` value for `axis` into the estimate. Seeds an empty estimate; clears the
  /// in-flight flag once the report confirms the commanded value; and, only when nothing is in flight, adopts a
  /// differing report as firmware truth (an override changed outside the slider). A differing report while a
  /// command is awaiting is the lagging poll and is ignored, so the next commit cannot step from a stale base.
  pub fn observe(&mut self, axis: OverrideAxis, reported: u32) {
    let slot = self.slot(axis);
    match slot.estimate {
      None => slot.estimate = Some(reported),
      Some(est) if reported == est => slot.awaiting = false,
      Some(_) if !slot.awaiting => slot.estimate = Some(reported),
      Some(_) => {}
    }
  }
}

/// The per-axis feedback state of an override slider: what the handle should show across the immediate-mode
/// frames of a drag and, crucially, what it should show *after release* while the firmware ramps to the
/// commanded value.
///
/// grbl overrides are relative and ramp over several `?`/`Ov:` round-trips (the firmware crawls toward the
/// target one ±10/±1 step per poll), so on release the live `Ov:` value still lags far behind the commanded
/// target. The previous design dropped the transient value to `None` on release and reverted to live, which
/// snapped the handle back to centre and then let it visibly creep as `live` caught up. This state machine
/// instead **holds the committed target** on the handle until reality converges, so the handle stays where the
/// operator put it and the fill tracks `live` underneath it without a snap or a self-propelled crawl past it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OverrideFeedback {
  /// No interaction: the handle mirrors the live `Ov:` value the firmware reports. The resting state.
  #[default]
  Idle,
  /// The operator is dragging the handle, which sits at this value (percent). The live status poll is ignored
  /// while held so a poll cannot yank the handle out from under the pointer mid-drag.
  Dragging(u32),
  /// The drag was released with a committed target (percent); the handle holds this value while the firmware
  /// ramps toward it. Carries the `live` value observed at commit time so convergence can tell whether the
  /// firmware has begun moving, distinguishing "still lagging at the old value" from "has reached the target".
  Holding {
    /// The committed target percent the handle holds and the firmware is ramping toward (already clamped).
    target: u32,
    /// The live `Ov:` value at the moment of release, so a later report can be classified as movement toward
    /// the target rather than the pre-command lag.
    committed_from: u32,
    /// How many non-converging status observations the hold has seen so far. Bounded by
    /// [`OVERRIDE_HOLD_MAX_OBSERVATIONS`]: once it is reached the hold releases even if the target was never
    /// reached, so a firmware that will never land on the target cannot pin a stale handle value forever (Bug 7).
    observations: u32,
  },
}

impl OverrideFeedback {
  /// The value the handle should display this frame, given the live `Ov:` value. `Idle` follows `live`; a drag
  /// or a post-release hold shows its own pinned value so neither a status poll nor the firmware's ramp moves
  /// the handle away from where the operator left it.
  pub fn display(self, live: u32) -> u32 {
    match self {
      OverrideFeedback::Idle => live,
      OverrideFeedback::Dragging(value) => value,
      OverrideFeedback::Holding { target, .. } => target,
    }
  }

  /// Enter (or continue) a drag with the handle at `value`. Called every frame the slider is being dragged.
  pub fn drag_to(&mut self, value: u32) {
    *self = OverrideFeedback::Dragging(value);
  }

  /// Commit the drag on release: hold the (clamped) `target` and remember the `live` value seen at release so
  /// convergence can later distinguish the lagging poll from a real arrival. The handle keeps showing `target`
  /// until [`Self::observe`] decides the firmware has converged onto it and returns to [`Self::Idle`].
  pub fn commit(&mut self, target: u32, live: u32) {
    *self = OverrideFeedback::Holding {
      target: clamp_override(target),
      committed_from: clamp_override(live),
      observations: 0,
    };
  }

  /// Fold a freshly-reported live `Ov:` value into the feedback state, releasing the post-release hold once the
  /// firmware has converged. Only `Holding` reacts; `Idle`/`Dragging` ignore reports (idle simply renders the
  /// live value, and a drag must never be perturbed by a poll).
  ///
  /// The hold releases — returning to [`Self::Idle`] so the handle resumes tracking `live` — once `live` has
  /// **reached or passed the target** in the direction of travel: at or beyond `target` when ramping up, at or
  /// below it when ramping down. For the normal converging ramp this fires within a few polls: the committed
  /// `target` is clamped to grbl's 10–200 span and the [`OverrideTracker`] emits the exact ±10/±1 steps onto it,
  /// so the firmware lands on (or, on a unit overshoot, just past) the target quickly; "passed" catches the
  /// clamp-at-bound and overshoot cases. Lagging reports still on the near side of the target keep the hold —
  /// exactly the snap-back the old design suffered.
  ///
  /// A second, bounded escape (Bug 7) guarantees the hold cannot stick forever when the firmware never lands on
  /// the target — e.g. an override adjusted while in ALARM (ignored by the firmware), an external/competing
  /// change, or a settle short of the target. Each non-converging observation increments a counter; once it
  /// reaches [`OVERRIDE_HOLD_MAX_OBSERVATIONS`] the hold releases anyway, falling back to mirroring live, so a
  /// stuck override can never pin a wrong DRO value on the handle indefinitely.
  pub fn observe(&mut self, live: u32) {
    if let OverrideFeedback::Holding { target, committed_from, observations } = *self {
      let reached = if target >= committed_from { live >= target } else { live <= target };
      if reached || observations + 1 >= OVERRIDE_HOLD_MAX_OBSERVATIONS {
        *self = OverrideFeedback::Idle;
      } else {
        *self = OverrideFeedback::Holding { target, committed_from, observations: observations + 1 };
      }
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn a_target_of_one_hundred_resets_in_a_single_byte() {
    // 100% is the neutral point: a single reset snaps to it no matter the current value, which is fewer bytes
    // and self-correcting against a drifted `current`.
    assert_eq!(override_commands(60, 100, OverrideAxis::Feed), vec![RealtimeCommand::FeedOverrideReset]);
    assert_eq!(override_commands(175, 100, OverrideAxis::Spindle), vec![RealtimeCommand::SpindleOverrideReset]);
  }

  #[test]
  fn an_increase_uses_decades_then_units() {
    // 100 -> 137 is 3 decades (+10 ×3) then 7 units (+1 ×7), in that order.
    let cmds = override_commands(100, 137, OverrideAxis::Feed);
    let expected: Vec<RealtimeCommand> = std::iter::repeat_n(RealtimeCommand::FeedOverridePlus10, 3)
      .chain(std::iter::repeat_n(RealtimeCommand::FeedOverridePlus1, 7))
      .collect();
    assert_eq!(cmds, expected);
  }

  #[test]
  fn a_decrease_uses_the_minus_commands() {
    // 130 -> 100 would reset, so test a non-neutral decrease: 150 -> 123 is 2 decades down, 7 units down.
    let cmds = override_commands(150, 123, OverrideAxis::Spindle);
    let expected: Vec<RealtimeCommand> = std::iter::repeat_n(RealtimeCommand::SpindleOverrideMinus10, 2)
      .chain(std::iter::repeat_n(RealtimeCommand::SpindleOverrideMinus1, 7))
      .collect();
    assert_eq!(cmds, expected);
  }

  #[test]
  fn a_pure_decade_move_emits_only_coarse_steps() {
    // 100 -> 120 is exactly two decades up: no fine steps at all.
    assert_eq!(override_commands(100, 120, OverrideAxis::Feed), vec![
      RealtimeCommand::FeedOverridePlus10,
      RealtimeCommand::FeedOverridePlus10,
    ]);
  }

  #[test]
  fn a_single_percent_move_emits_one_fine_step() {
    // The fine ±1% control: 110 -> 111 is one +1, 110 -> 109 is one −1.
    assert_eq!(override_commands(110, 111, OverrideAxis::Feed), vec![RealtimeCommand::FeedOverridePlus1]);
    assert_eq!(override_commands(110, 109, OverrideAxis::Feed), vec![RealtimeCommand::FeedOverrideMinus1]);
  }

  #[test]
  fn no_movement_emits_nothing() {
    // Asking for the value the firmware already reports sends nothing — the link stays quiet.
    assert!(override_commands(115, 115, OverrideAxis::Feed).is_empty());
  }

  #[test]
  fn the_target_is_clamped_to_the_grbl_range() {
    // Below 10% clamps up to 10; the move from 100 is then 9 decades down (90 -> 9 decades, 0 units).
    let low = override_commands(100, 0, OverrideAxis::Feed);
    assert_eq!(low.len(), 9);
    assert!(low.iter().all(|c| *c == RealtimeCommand::FeedOverrideMinus10));
    // Above 200% clamps down to 200; the move from 100 is 10 decades up.
    let high = override_commands(100, 1000, OverrideAxis::Spindle);
    assert_eq!(high.len(), 10);
    assert!(high.iter().all(|c| *c == RealtimeCommand::SpindleOverridePlus10));
  }

  #[test]
  fn a_stale_current_cannot_produce_an_unbounded_run() {
    // Even a wildly out-of-range reported `current` is clamped first, so the step count stays within the
    // legal 10–200 span (here at most 19 decades-worth, never thousands).
    let cmds = override_commands(99_999, 110, OverrideAxis::Feed);
    // current clamps to 200, target 110: 9 decades down — bounded.
    assert_eq!(cmds.len(), 9);
    assert!(cmds.iter().all(|c| *c == RealtimeCommand::FeedOverrideMinus10));
  }

  #[test]
  fn clamp_override_matches_the_grbl_bounds() {
    assert_eq!(clamp_override(5), OVERRIDE_MIN);
    assert_eq!(clamp_override(250), OVERRIDE_MAX);
    assert_eq!(clamp_override(137), 137);
  }

  #[test]
  fn tracker_steps_a_second_commit_from_the_commanded_value_not_the_stale_report() {
    // The core race: two commits inside one poll interval. The firmware still reports 100 after the first
    // commit (the poll lags), but the second commit must step from 150 (where the first left it), not 100.
    let mut tracker = OverrideTracker::default();
    let first = tracker.command(OverrideAxis::Feed, 100, 150);
    assert_eq!(first.len(), 5); // 100 -> 150: five +10 decades.
    // Second commit while the report is still the lagging 100: step 150 -> 120 (three −10), not 100 -> 120.
    let second = tracker.command(OverrideAxis::Feed, 100, 120);
    let expected: Vec<RealtimeCommand> = std::iter::repeat_n(RealtimeCommand::FeedOverrideMinus10, 3).collect();
    assert_eq!(second, expected);
  }

  #[test]
  fn tracker_ignores_a_lagging_report_then_reconciles_on_confirmation() {
    let mut tracker = OverrideTracker::default();
    tracker.command(OverrideAxis::Feed, 100, 150);
    // The lagging poll still shows 100 while the move is in flight: it must not become the next stepping base.
    tracker.observe(OverrideAxis::Feed, 100);
    let after_lag = tracker.command(OverrideAxis::Feed, 100, 140);
    assert_eq!(after_lag, vec![RealtimeCommand::FeedOverrideMinus10]); // 150 -> 140, not 100 -> 140.
    // Once a report confirms the commanded value, the in-flight flag clears and later reports are trusted.
    tracker.command(OverrideAxis::Feed, 140, 140); // re-arm awaiting at 140.
    tracker.observe(OverrideAxis::Feed, 140);
    // An external change (e.g. a pendant) is now adopted because nothing is in flight: step from 130.
    tracker.observe(OverrideAxis::Feed, 130);
    let after_external = tracker.command(OverrideAxis::Feed, 130, 120);
    assert_eq!(after_external, vec![RealtimeCommand::FeedOverrideMinus10]); // 130 -> 120.
  }

  #[test]
  fn tracker_seeds_from_the_report_before_any_command() {
    // With no command yet issued, the first commit bases on the reported value.
    let mut tracker = OverrideTracker::default();
    tracker.observe(OverrideAxis::Spindle, 120);
    let cmds = tracker.command(OverrideAxis::Spindle, 120, 140);
    assert_eq!(cmds, vec![RealtimeCommand::SpindleOverridePlus10, RealtimeCommand::SpindleOverridePlus10]);
  }

  #[test]
  fn tracker_axes_are_independent() {
    // A feed commit must not disturb the spindle estimate and vice versa.
    let mut tracker = OverrideTracker::default();
    tracker.command(OverrideAxis::Feed, 100, 150);
    let spindle = tracker.command(OverrideAxis::Spindle, 100, 120);
    assert_eq!(spindle.len(), 2); // 100 -> 120 on spindle, unaffected by the feed estimate.
  }

  #[test]
  fn feedback_idle_mirrors_the_live_value() {
    // At rest the handle simply shows whatever the firmware last reported.
    let fb = OverrideFeedback::Idle;
    assert_eq!(fb.display(137), 137);
    assert_eq!(fb.display(100), 100);
  }

  #[test]
  fn feedback_drag_shows_its_own_value_and_ignores_the_live_poll() {
    // While dragging, the handle holds the operator's position — a status poll at a different value must not
    // move it (the mid-drag "yank" bug).
    let mut fb = OverrideFeedback::Idle;
    fb.drag_to(160);
    assert_eq!(fb.display(100), 160, "the live poll must not pull the handle off the drag position");
    // A status report mid-drag is ignored entirely.
    fb.observe(100);
    assert_eq!(fb.display(100), 160);
  }

  #[test]
  fn feedback_holds_the_committed_target_against_a_stale_live() {
    // The core snap-back fix: on release the handle holds the commanded target even though the firmware's `Ov:`
    // still lags at the pre-command value for several polls.
    let mut fb = OverrideFeedback::Idle;
    fb.drag_to(150);
    fb.commit(150, 100); // released at 150, firmware still reports 100.
    assert_eq!(fb.display(100), 150, "the handle holds the target, it does NOT snap to the stale live value");
    // A lagging report still showing the pre-command value leaves the hold in place.
    fb.observe(100);
    assert_eq!(fb.display(100), 150);
    // The firmware crawls partway — still short of the target — so the handle keeps holding (no creep).
    fb.observe(130);
    assert_eq!(fb.display(130), 150, "a partial ramp does not release the hold; the handle stays put");
  }

  #[test]
  fn feedback_releases_the_hold_once_live_reaches_the_target() {
    // Once reality converges onto the commanded value, the hold clears and the handle resumes tracking `live`.
    let mut fb = OverrideFeedback::Idle;
    fb.commit(150, 100);
    fb.observe(150); // the firmware has arrived.
    assert_eq!(fb, OverrideFeedback::Idle, "reaching the target releases the post-release hold");
    // Now idle, the handle follows live again — including an external change after convergence.
    assert_eq!(fb.display(140), 140);
  }

  #[test]
  fn feedback_releases_when_live_passes_the_target_on_a_unit_overshoot() {
    // Ramping down, a firmware that lands one unit past the target (or clamps) must still release the hold, so
    // it can never stick. 120 -> overshoot to 119 still counts as reached when ramping down.
    let mut fb = OverrideFeedback::Idle;
    fb.commit(120, 150); // committed down from 150 to 120.
    fb.observe(119); // overshot by a unit.
    assert_eq!(fb, OverrideFeedback::Idle, "passing the target on the travel side releases the hold");
  }

  #[test]
  fn feedback_hold_releases_after_a_bounded_number_of_non_converging_observations() {
    // Bug 7: a Holding hold must not pin the handle forever if the firmware never lands on the target — e.g. the
    // override was adjusted while in ALARM (the firmware ignores it), an external/competing change moved it, or it
    // settled short. After a bounded number of status observations that never reach the target, the hold releases
    // and the handle falls back to mirroring live, so it can never show a percentage the firmware is not at.
    let mut fb = OverrideFeedback::Idle;
    fb.commit(150, 100); // ramping up to 150; the firmware will never get there (stuck at 100).
    // Feed many lagging reports that never reach the target; the hold must NOT stick indefinitely.
    for _ in 0..OVERRIDE_HOLD_MAX_OBSERVATIONS {
      assert_eq!(fb.display(100), 150, "the hold keeps the handle on the target while within the escape bound");
      fb.observe(100);
    }
    // The escape bound is now spent: the hold has released and the handle mirrors live again.
    assert_eq!(fb, OverrideFeedback::Idle, "a stuck override must not pin the handle past the escape bound");
    assert_eq!(fb.display(100), 100, "after release the handle falls back to the live value");
  }

  #[test]
  fn feedback_converging_hold_still_releases_on_arrival_within_the_bound() {
    // The normal converging case is unchanged: a hold that reaches its target releases immediately on arrival,
    // well within the escape bound, and never snaps back in the meantime.
    let mut fb = OverrideFeedback::Idle;
    fb.commit(150, 100);
    fb.observe(120); // partial ramp, still short — stays held.
    assert_eq!(fb.display(120), 150);
    fb.observe(150); // arrived.
    assert_eq!(fb, OverrideFeedback::Idle, "reaching the target still releases the hold within the bound");
  }

  #[test]
  fn feedback_clamps_a_committed_target_to_the_grbl_range() {
    // A target beyond grbl's bounds is held at the clamped value, matching what the firmware will actually
    // reach, so convergence lands exactly and the hold releases.
    let mut fb = OverrideFeedback::Idle;
    fb.commit(1000, 100); // clamps to 200.
    assert_eq!(fb.display(100), 200);
    fb.observe(200);
    assert_eq!(fb, OverrideFeedback::Idle);
  }
}
