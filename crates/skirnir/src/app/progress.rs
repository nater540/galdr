//! Pure streaming-progress time estimation: elapsed and a remaining-time estimate from the acked rate.
//!
//! The design's dock progress shows an `m:ss / m:ss` pair (elapsed / total estimate) alongside the line
//! count and bar. The engine has no notion of wall-clock time, so the host derives it: the shell records when
//! a stream became active and, each frame, hands this module the elapsed [`Duration`] plus the live line
//! counts. Everything here is a pure function of those inputs — no `Instant::now()`, no I/O — so the estimate
//! grammar is unit-tested deterministically without a clock or a window. Keeping the math out of the view
//! layer means the dock just formats the [`TimeEstimate`] this returns.

use std::time::Duration;

/// An elapsed/remaining/total time estimate for the active stream, derived from how long the stream has run
/// and how many of its lines have been acknowledged. `remaining` and `total` are best-effort projections of
/// the acked rate; they are `None` until there is enough signal to project (at least one acked line and a
/// non-zero elapsed), so the UI shows a dash rather than a wild early guess.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TimeEstimate {
  /// How long the stream has been running.
  pub elapsed: Duration,
  /// The projected time still remaining, or `None` while it cannot yet be projected (no acks / no elapsed /
  /// already complete contributes a zero remaining, not `None`).
  pub remaining: Option<Duration>,
  /// The projected total run time (`elapsed + remaining`), or `None` when `remaining` is.
  pub total: Option<Duration>,
}

/// Project the time estimate for a stream that has run `elapsed` and acknowledged `acked` of `total` lines.
///
/// The projection assumes lines complete at a roughly constant rate, so remaining time scales with the
/// unacknowledged fraction: `remaining = elapsed * (total - acked) / acked`. It deliberately returns no
/// projection (only the raw `elapsed`) until the rate is observable — `acked == 0` or `elapsed == 0` — so a
/// fresh stream does not flash a nonsensical ETA. A finished stream (`acked >= total`) projects a zero
/// remaining and a `total` equal to the elapsed, which reads as "done".
pub fn estimate(elapsed: Duration, acked: usize, total: usize) -> TimeEstimate {
  // No program, or no progress to project from yet: surface only the raw elapsed.
  if total == 0 || acked == 0 || elapsed.is_zero() {
    return TimeEstimate { elapsed, remaining: None, total: None };
  }
  // Clamp acked to total so a late ack (or a manual line miscount) can never project a negative remaining.
  let acked = acked.min(total);
  let outstanding = total - acked;
  // Scale elapsed by the outstanding/acked ratio using the seconds-as-f64 form: line counts are small and the
  // durations are seconds-to-minutes, so f64 has ample precision and avoids `Duration` overflow on the multiply.
  let per_line = elapsed.as_secs_f64() / acked as f64;
  let remaining = Duration::from_secs_f64(per_line * outstanding as f64);
  TimeEstimate {
    elapsed,
    remaining: Some(remaining),
    total: Some(elapsed + remaining),
  }
}

/// Whether a streaming job has GENUINELY completed, given the program line counts and whether the firmware's
/// reported machine state has settled to Idle. Completion requires all three: a real program (`total > 0`), every
/// program line acknowledged (`acked >= total`), and the machine back at Idle (`run_idle`). This is the signal the
/// dock clock latches on to FREEZE the elapsed time and stop the ETA once the run is done.
///
/// The `run_idle` term is what distinguishes genuine completion from a TRANSIENT idle: a `?` poll mid-stream often
/// reads `<Idle>` for an instant when the planner momentarily drains between blocks, but at that point `acked` is
/// still short of `total`, so the `acked >= total` term holds the latch off. Equally, a pre-start idle (no lines
/// acked yet) fails `acked >= total` (with a non-zero total) or `total > 0` (with none loaded). Only the true end —
/// every line acked AND the machine parked — satisfies all three. Pure so the latch decision is unit-tested without
/// a wall clock or a live status feed.
pub fn stream_is_complete(total: usize, acked: usize, run_idle: bool) -> bool {
  total > 0 && acked >= total && run_idle
}

/// Build a [`TimeEstimate`] from a pre-computed physics-based remaining time (see [`crate::eta::EtaTimeline`])
/// rather than the acked-rate projection of [`estimate`]. The caller supplies the wall-clock `elapsed`, the
/// `total_seconds` the timeline modeled at 100 % overrides, and the live `remaining_seconds` the timeline drains
/// to for the current completed-line count and override fractions. Unlike [`estimate`], the projection exists
/// from the very first frame — even with zero elapsed and no acks — because the timeline is computed from the
/// machine's motion model, not learned from the run. So the upfront ETA (before streaming) and the live remaining
/// (during streaming) both surface immediately. Kept pure (no `Instant::now()`) so the elapsed→clock grammar is
/// unit-tested deterministically; the shell owns the wall clock and passes `elapsed` in.
pub fn physics_estimate(elapsed: Duration, total_seconds: f64, remaining_seconds: f64) -> TimeEstimate {
  // Guard against a NaN/negative remaining from a degenerate timeline; clamp to a non-negative finite value so
  // `Duration::from_secs_f64` cannot panic on a bad input.
  let remaining = Duration::from_secs_f64(remaining_seconds.max(0.0));
  // The displayed total is the modeled job time, floored at `elapsed + remaining` so a run that overshoots the
  // estimate (slower than modeled, or a paused operator wait) still shows a total that is at least what is left.
  let modeled = Duration::from_secs_f64(total_seconds.max(0.0));
  let total = modeled.max(elapsed + remaining);
  TimeEstimate { elapsed, remaining: Some(remaining), total: Some(total) }
}

/// Format a [`Duration`] as the design's compact `m:ss` clock (minutes:seconds, zero-padded seconds). Hours
/// roll into the minutes field (`75:09` for 1h15m9s) since a PCB job rarely runs that long and the design
/// reserves no hours slot. A `None` renders as the dim placeholder `--:--`.
pub fn format_mmss(duration: Option<Duration>) -> String {
  let Some(duration) = duration else {
    return "--:--".to_string();
  };
  let total_secs = duration.as_secs();
  let minutes = total_secs / 60;
  let seconds = total_secs % 60;
  format!("{minutes}:{seconds:02}")
}

/// Format the dock's elapsed/total run clock as the design's `m:ss / m:ss` pair — elapsed on the left, the
/// projected total on the right. `total` is `None` until the ETA is projectable, so the right half shows the
/// `--:--` placeholder rather than a wild early guess. Keeping this here gives the view one tested source for
/// the clock string instead of an inline `format!`.
pub fn format_progress_clock(elapsed: Duration, total: Option<Duration>) -> String {
  format!("{} / {}", format_mmss(Some(elapsed)), format_mmss(total))
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn no_estimate_before_the_first_ack() {
    // A stream that has run for a while but acked nothing yet cannot project a rate; only elapsed is known.
    let est = estimate(Duration::from_secs(5), 0, 100);
    assert_eq!(est.elapsed, Duration::from_secs(5));
    assert_eq!(est.remaining, None);
    assert_eq!(est.total, None);
  }

  #[test]
  fn no_estimate_with_zero_elapsed() {
    // The very first frame (elapsed 0) has no observable rate even if an ack already landed.
    let est = estimate(Duration::ZERO, 1, 10);
    assert_eq!(est.remaining, None);
    assert_eq!(est.total, None);
  }

  #[test]
  fn no_program_yields_no_estimate() {
    let est = estimate(Duration::from_secs(3), 0, 0);
    assert_eq!(est, TimeEstimate { elapsed: Duration::from_secs(3), remaining: None, total: None });
  }

  #[test]
  fn projects_remaining_from_the_acked_rate() {
    // 10 of 40 lines acked in 10s ⇒ 1s/line ⇒ 30 lines × 1s = 30s remaining, 40s total.
    let est = estimate(Duration::from_secs(10), 10, 40);
    assert_eq!(est.elapsed, Duration::from_secs(10));
    assert_eq!(est.remaining, Some(Duration::from_secs(30)));
    assert_eq!(est.total, Some(Duration::from_secs(40)));
  }

  #[test]
  fn a_completed_stream_projects_zero_remaining() {
    // All lines acked: remaining collapses to zero and total equals elapsed.
    let est = estimate(Duration::from_secs(20), 50, 50);
    assert_eq!(est.remaining, Some(Duration::ZERO));
    assert_eq!(est.total, Some(Duration::from_secs(20)));
  }

  #[test]
  fn a_late_overshooting_ack_never_projects_negative_remaining() {
    // A miscount where acked > total must clamp, not underflow into a negative/huge remaining.
    let est = estimate(Duration::from_secs(10), 60, 50);
    assert_eq!(est.remaining, Some(Duration::ZERO));
  }

  #[test]
  fn physics_estimate_surfaces_a_total_and_remaining_from_the_first_frame() {
    // Unlike the acked-rate `estimate`, the physics estimate projects immediately — zero elapsed, no acks — since
    // the timeline is modeled, not learned. Upfront (before streaming) elapsed is 0 and remaining is the whole job.
    let est = physics_estimate(Duration::ZERO, 600.0, 600.0);
    assert_eq!(est.elapsed, Duration::ZERO);
    assert_eq!(est.remaining, Some(Duration::from_secs(600)));
    assert_eq!(est.total, Some(Duration::from_secs(600)), "the upfront total is the modeled job time");
  }

  #[test]
  fn physics_estimate_keeps_the_modeled_total_while_remaining_drains() {
    // Mid-stream: 120s elapsed, the timeline says 480s remain. The total stays the modeled 600s (not elapsed+remaining,
    // which would also be 600 here) and the remaining is the physical figure, not an acked-rate guess.
    let est = physics_estimate(Duration::from_secs(120), 600.0, 480.0);
    assert_eq!(est.remaining, Some(Duration::from_secs(480)));
    assert_eq!(est.total, Some(Duration::from_secs(600)));
  }

  #[test]
  fn physics_estimate_floors_the_total_at_elapsed_plus_remaining_on_an_overrun() {
    // A run slower than modeled (or an operator pause) can push elapsed+remaining past the modeled total; the
    // displayed total must then grow to at least what is actually left so the clock never shows a total below now.
    let est = physics_estimate(Duration::from_secs(700), 600.0, 50.0);
    assert_eq!(est.total, Some(Duration::from_secs(750)), "the total floors at elapsed + remaining on an overrun");
  }

  #[test]
  fn physics_estimate_clamps_a_degenerate_remaining_without_panicking() {
    // A NaN/negative remaining from a degenerate timeline must clamp to zero rather than panic in `from_secs_f64`.
    let est = physics_estimate(Duration::from_secs(10), 0.0, f64::NAN);
    assert_eq!(est.remaining, Some(Duration::ZERO));
    let est = physics_estimate(Duration::from_secs(10), 0.0, -5.0);
    assert_eq!(est.remaining, Some(Duration::ZERO));
  }

  #[test]
  fn format_mmss_pads_seconds_and_handles_none() {
    assert_eq!(format_mmss(Some(Duration::from_secs(0))), "0:00");
    assert_eq!(format_mmss(Some(Duration::from_secs(9))), "0:09");
    assert_eq!(format_mmss(Some(Duration::from_secs(75))), "1:15");
    // Hours roll into minutes (no hours slot in the design's compact clock).
    assert_eq!(format_mmss(Some(Duration::from_secs(3600 + 9))), "60:09");
    assert_eq!(format_mmss(None), "--:--");
  }

  #[test]
  fn format_progress_clock_pairs_elapsed_with_total_and_dashes_an_absent_total() {
    // The projectable case: both halves render as `m:ss`, separated by ` / `.
    assert_eq!(format_progress_clock(Duration::from_secs(51), Some(Duration::from_secs(576))), "0:51 / 9:36");
    // Before the ETA is projectable the total is `None`, so the right half is the dim placeholder, not a guess.
    assert_eq!(format_progress_clock(Duration::from_secs(5), None), "0:05 / --:--");
  }

  #[test]
  fn stream_is_complete_only_when_all_lines_acked_and_the_machine_is_idle() {
    // The genuine end: every program line acked AND the machine settled to Idle.
    assert!(stream_is_complete(10, 10, true), "all 10 lines acked and Idle is complete");
    assert!(stream_is_complete(10, 11, true), "a late/over-ack past total still counts complete (clamped)");
  }

  #[test]
  fn stream_is_complete_holds_off_on_a_transient_mid_stream_idle() {
    // A `?` poll mid-stream can read Idle for an instant as the planner drains between blocks, but not all lines are
    // acked yet — the `acked >= total` term must hold the latch off so the clock does not freeze prematurely.
    assert!(!stream_is_complete(10, 4, true), "a transient Idle with lines still outstanding is NOT complete");
    // Running (not Idle) with everything acked but the machine still moving is also not yet complete.
    assert!(!stream_is_complete(10, 10, false), "all acked but still moving (not Idle) is not yet complete");
  }

  #[test]
  fn stream_is_complete_is_false_before_a_program_or_progress() {
    // No program loaded, or a pre-start idle with nothing acked, must never read as complete (no premature latch).
    assert!(!stream_is_complete(0, 0, true), "no program is never complete");
    assert!(!stream_is_complete(10, 0, true), "a pre-start idle with nothing acked is not complete");
  }
}
