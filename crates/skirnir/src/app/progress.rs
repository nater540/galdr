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
  fn format_mmss_pads_seconds_and_handles_none() {
    assert_eq!(format_mmss(Some(Duration::from_secs(0))), "0:00");
    assert_eq!(format_mmss(Some(Duration::from_secs(9))), "0:09");
    assert_eq!(format_mmss(Some(Duration::from_secs(75))), "1:15");
    // Hours roll into minutes (no hours slot in the design's compact clock).
    assert_eq!(format_mmss(Some(Duration::from_secs(3600 + 9))), "60:09");
    assert_eq!(format_mmss(None), "--:--");
  }
}
