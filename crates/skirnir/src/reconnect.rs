//! The host-side reconnect policy: a pure, framework-agnostic backoff schedule for re-opening a dropped link.
//!
//! The ESP32-S3's native USB port cannot be hard-reset by the host, and a soft reset (`0x18`) makes the device
//! re-enumerate — the serial endpoint briefly vanishes and reappears (often at the same `/dev` path, sometimes
//! after a beat). A sender that simply ends on EOF (as the engine's driver does) leaves the operator manually
//! reconnecting after every soft reset. This module is the decision half of an auto-reconnect: given a chain of
//! failed attempts it produces the delay before the next one, backing off exponentially up to a cap and giving
//! up after a bounded number of tries, then resets the moment a connection succeeds. It owns no timer, no
//! transport, and no UI — the async shell consults it and schedules the actual retry — so the schedule is
//! unit-tested deterministically without sleeping or opening a port.
//!
//! Why a *bounded* backoff rather than retry-forever: a board that was unplugged (not soft-reset) should stop
//! being chased after a reasonable window so the UI settles into a clean disconnected state the operator can
//! act on, rather than spinning forever and masking a real "the cable is out" condition. A soft-reset re-
//! enumeration completes well within the first few attempts, so the common case reconnects fast.

use std::time::Duration;

/// The tuning of a [`ReconnectPolicy`]: the first delay, the growth factor, the ceiling, and how many attempts
/// to make before giving up. Defaults target the ESP32-S3 soft-reset case: a short first delay so a quick re-
/// enumeration reconnects almost immediately, doubling up to a 5 s ceiling, over a handful of attempts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReconnectConfig {
  /// The delay before the first reconnect attempt.
  pub base: Duration,
  /// The multiplier applied per failed attempt (2 = double each time).
  pub factor: u32,
  /// The maximum delay between attempts; the exponential growth is clamped to this.
  pub max_delay: Duration,
  /// How many attempts to make before giving up (and surfacing a settled disconnect). `0` disables auto-
  /// reconnect entirely.
  pub max_attempts: u32,
}

impl Default for ReconnectConfig {
  fn default() -> Self {
    ReconnectConfig {
      base: Duration::from_millis(300),
      factor: 2,
      max_delay: Duration::from_secs(5),
      max_attempts: 6,
    }
  }
}

/// A stateful reconnect schedule built from a [`ReconnectConfig`]. Call [`ReconnectPolicy::next_delay`] after a
/// disconnect to get the delay before the next attempt (or `None` once attempts are exhausted), and
/// [`ReconnectPolicy::on_connected`] the moment a connection succeeds to reset the schedule so the *next* drop
/// starts its backoff fresh.
#[derive(Debug, Clone)]
pub struct ReconnectPolicy {
  config: ReconnectConfig,
  /// How many reconnect attempts have been scheduled since the last successful connect.
  attempts: u32,
}

impl ReconnectPolicy {
  /// Build a policy from a config.
  pub fn new(config: ReconnectConfig) -> Self {
    ReconnectPolicy { config, attempts: 0 }
  }

  /// Whether auto-reconnect is enabled at all (a `max_attempts` of zero disables it).
  pub fn is_enabled(&self) -> bool {
    self.config.max_attempts > 0
  }

  /// The number of attempts made since the last successful connect (diagnostics / UI "retrying 3/6").
  pub fn attempts(&self) -> u32 {
    self.attempts
  }

  /// Reset the schedule after a successful connection, so a future disconnect begins backoff from the first
  /// delay again rather than continuing a stale exponential.
  pub fn on_connected(&mut self) {
    self.attempts = 0;
  }

  /// Compute the delay before the next reconnect attempt and advance the schedule, or `None` when the attempt
  /// budget is exhausted (the caller then settles into a clean disconnected state). The delay is
  /// `base * factor^attempt`, clamped to `max_delay`; `attempt` is the count of attempts already made, so the
  /// first call returns `base` and each subsequent call grows geometrically.
  pub fn next_delay(&mut self) -> Option<Duration> {
    if self.attempts >= self.config.max_attempts {
      return None;
    }
    let delay = backoff_delay(self.config.base, self.config.factor, self.attempts, self.config.max_delay);
    self.attempts += 1;
    Some(delay)
  }
}

/// Compute the backoff delay for a given attempt index: `base * factor^attempt`, saturating at `max_delay`.
/// Pure and overflow-safe — the multiply is done in `u128` nanoseconds and clamps to `max_delay` the instant
/// it would exceed it, so a large attempt index or factor can never panic or wrap.
fn backoff_delay(base: Duration, factor: u32, attempt: u32, max_delay: Duration) -> Duration {
  let max_nanos = max_delay.as_nanos();
  let mut nanos = base.as_nanos();
  for _ in 0..attempt {
    nanos = nanos.saturating_mul(factor as u128);
    // Stop multiplying the moment we reach the ceiling: further growth is irrelevant and bounds the loop work.
    if nanos >= max_nanos {
      return max_delay;
    }
  }
  if nanos >= max_nanos {
    max_delay
  } else {
    // `nanos` is below `max_delay`'s nanos here, which fit in u64, so the cast is lossless.
    Duration::from_nanos(nanos as u64)
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  fn policy(base_ms: u64, factor: u32, max_ms: u64, attempts: u32) -> ReconnectPolicy {
    ReconnectPolicy::new(ReconnectConfig {
      base: Duration::from_millis(base_ms),
      factor,
      max_delay: Duration::from_millis(max_ms),
      max_attempts: attempts,
    })
  }

  #[test]
  fn the_first_delay_is_the_base_and_growth_is_geometric() {
    let mut p = policy(100, 2, 10_000, 6);
    assert_eq!(p.next_delay(), Some(Duration::from_millis(100)));
    assert_eq!(p.next_delay(), Some(Duration::from_millis(200)));
    assert_eq!(p.next_delay(), Some(Duration::from_millis(400)));
    assert_eq!(p.next_delay(), Some(Duration::from_millis(800)));
  }

  #[test]
  fn the_delay_is_clamped_to_the_ceiling() {
    let mut p = policy(100, 10, 500, 10);
    assert_eq!(p.next_delay(), Some(Duration::from_millis(100)));
    // 100 * 10 = 1000 would exceed the 500 ceiling, so it clamps.
    assert_eq!(p.next_delay(), Some(Duration::from_millis(500)));
    assert_eq!(p.next_delay(), Some(Duration::from_millis(500)), "it stays at the ceiling thereafter");
  }

  #[test]
  fn the_schedule_gives_up_after_max_attempts() {
    let mut p = policy(100, 2, 10_000, 3);
    assert!(p.next_delay().is_some());
    assert!(p.next_delay().is_some());
    assert!(p.next_delay().is_some());
    assert_eq!(p.next_delay(), None, "the fourth attempt is past the budget");
    assert_eq!(p.attempts(), 3);
  }

  #[test]
  fn a_successful_connect_resets_the_backoff() {
    let mut p = policy(100, 2, 10_000, 6);
    p.next_delay();
    p.next_delay();
    assert_eq!(p.attempts(), 2);
    // A reconnect succeeded: the next drop must start its backoff from the base again.
    p.on_connected();
    assert_eq!(p.attempts(), 0);
    assert_eq!(p.next_delay(), Some(Duration::from_millis(100)));
  }

  #[test]
  fn zero_max_attempts_disables_auto_reconnect() {
    let mut p = policy(100, 2, 10_000, 0);
    assert!(!p.is_enabled());
    assert_eq!(p.next_delay(), None, "a disabled policy never schedules a retry");
  }

  #[test]
  fn the_backoff_never_overflows_on_a_large_attempt_index() {
    // A pathological factor/index must clamp to the ceiling, never panic or wrap.
    let mut p = policy(1_000, 1_000_000, 5_000, 50);
    // First is the base; every subsequent is the ceiling.
    assert_eq!(p.next_delay(), Some(Duration::from_millis(1_000)));
    for _ in 0..40 {
      assert_eq!(p.next_delay(), Some(Duration::from_millis(5_000)));
    }
  }

  #[test]
  fn the_default_config_targets_the_soft_reset_case() {
    // A sanity check on the shipped defaults: a short first delay, doubling, a 5 s cap, a handful of attempts.
    let cfg = ReconnectConfig::default();
    assert_eq!(cfg.base, Duration::from_millis(300));
    assert_eq!(cfg.factor, 2);
    assert_eq!(cfg.max_delay, Duration::from_secs(5));
    assert!(cfg.max_attempts >= 4, "enough attempts to ride out a USB re-enumeration");
    assert!(ReconnectPolicy::new(cfg).is_enabled());
  }
}
