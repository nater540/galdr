//! Host-side character-counting flow control (the send-ahead window).
//!
//! Per `docs/gcode-streaming.md`, the host keeps a running sum of the byte-lengths (terminator included) of
//! every line it has sent but not yet seen acknowledged, and only sends another line while that sum stays
//! within the firmware's advertised RX buffer. Each `ok`/`error:N` frees the *oldest* unacknowledged line.
//! Real-time single bytes are NOT counted here — they bypass the line buffer entirely.
//!
//! This window is a pure, synchronous unit: feed it line lengths and acknowledgements, ask it whether the
//! next line of a given length fits. It carries no time, no I/O, and no transport, so it is exhaustively
//! unit-testable on its own.

use std::collections::VecDeque;

use crate::error::EngineError;

/// The grblHAL default RX buffer size in bytes. Real 32-bit drivers report 1024 in `[OPT:...]`; we default
/// to it and refine from the advertised value at connect (see [`crate::protocol::response::rx_buffer_from_opt`]).
pub const DEFAULT_RX_BUFFER: usize = 1024;

/// The character-count send-ahead window. Tracks the byte-lengths of in-flight (sent, unacknowledged) lines
/// against `rx_buffer`, releasing the next line only when it fits the remaining space.
#[derive(Debug)]
pub struct FlowWindow {
  /// The firmware's advertised RX buffer size — the budget the in-flight sum must not exceed.
  rx_buffer: usize,
  /// Byte-lengths (terminator included) of lines sent but not yet acknowledged, oldest at the front.
  inflight: VecDeque<usize>,
  /// Cached sum of `inflight`, kept incrementally so `remaining`/`fits` stay O(1) on the hot path.
  inflight_bytes: usize,
}

impl FlowWindow {
  /// Create a window sized to `rx_buffer` bytes. A zero budget is clamped to 1 so a degenerate config can
  /// never silently wedge the stream (a real line still won't fit, surfacing as `LineTooLong`).
  pub fn new(rx_buffer: usize) -> Self {
    Self { rx_buffer: rx_buffer.max(1), inflight: VecDeque::new(), inflight_bytes: 0 }
  }

  /// Update the budget once the firmware advertises its real RX buffer size (`[OPT:...]`). Safe to call mid
  /// life; it only changes future `fits` decisions, never retroactively un-sends in-flight lines.
  pub fn set_rx_buffer(&mut self, rx_buffer: usize) {
    self.rx_buffer = rx_buffer.max(1);
  }

  /// The configured RX buffer budget.
  pub fn rx_buffer(&self) -> usize {
    self.rx_buffer
  }

  /// Bytes currently in flight (sent but unacknowledged).
  pub fn inflight_bytes(&self) -> usize {
    self.inflight_bytes
  }

  /// Number of lines currently in flight.
  pub fn inflight_lines(&self) -> usize {
    self.inflight.len()
  }

  /// Remaining free space in the firmware's RX buffer, by the host's count.
  pub fn remaining(&self) -> usize {
    self.rx_buffer - self.inflight_bytes
  }

  /// Whether a line of `line_len` bytes (terminator included) fits the remaining window right now. A line
  /// larger than the whole buffer can never fit and is reported as too long rather than as "doesn't fit
  /// yet", so the caller can reject it instead of waiting forever.
  pub fn fits(&self, line_len: usize) -> Result<bool, EngineError> {
    if line_len > self.rx_buffer {
      return Err(EngineError::LineTooLong { len: line_len, rx_buffer: self.rx_buffer });
    }
    Ok(self.inflight_bytes + line_len <= self.rx_buffer)
  }

  /// Record that a line of `line_len` bytes was just sent — it joins the in-flight set. The caller must
  /// have confirmed [`Self::fits`] first; this method trusts that and only tracks accounting.
  pub fn on_line_sent(&mut self, line_len: usize) {
    self.inflight.push_back(line_len);
    self.inflight_bytes += line_len;
  }

  /// Record an `ok`/`error:N` acknowledgement, freeing the oldest in-flight line. Returns its byte-length.
  /// An acknowledgement with nothing in flight is a hard protocol-counting violation ([`EngineError::UnexpectedAck`]).
  pub fn on_ack(&mut self) -> Result<usize, EngineError> {
    let len = self.inflight.pop_front().ok_or(EngineError::UnexpectedAck)?;
    self.inflight_bytes -= len;
    Ok(len)
  }

  /// Whether any lines are still awaiting acknowledgement.
  pub fn has_inflight(&self) -> bool {
    !self.inflight.is_empty()
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn fresh_window_has_full_budget_free() {
    let w = FlowWindow::new(128);
    assert_eq!(w.remaining(), 128);
    assert_eq!(w.inflight_bytes(), 0);
    assert!(!w.has_inflight());
  }

  #[test]
  fn a_line_fits_only_while_the_running_sum_stays_within_budget() {
    let mut w = FlowWindow::new(10);
    assert_eq!(w.fits(6), Ok(true));
    w.on_line_sent(6);
    // 6 in flight, only 4 free: a 5-byte line does not fit, a 4-byte line does.
    assert_eq!(w.fits(5), Ok(false));
    assert_eq!(w.fits(4), Ok(true));
  }

  #[test]
  fn an_ack_frees_exactly_the_oldest_line() {
    let mut w = FlowWindow::new(20);
    w.on_line_sent(8); // line A
    w.on_line_sent(5); // line B
    assert_eq!(w.inflight_bytes(), 13);
    assert_eq!(w.on_ack(), Ok(8)); // frees A (the oldest), not B
    assert_eq!(w.inflight_bytes(), 5);
    assert_eq!(w.remaining(), 15);
  }

  #[test]
  fn ack_with_nothing_in_flight_is_a_counting_violation() {
    let mut w = FlowWindow::new(20);
    assert_eq!(w.on_ack(), Err(EngineError::UnexpectedAck));
  }

  #[test]
  fn a_line_larger_than_the_whole_buffer_can_never_fit() {
    let w = FlowWindow::new(16);
    assert_eq!(w.fits(17), Err(EngineError::LineTooLong { len: 17, rx_buffer: 16 }));
  }

  #[test]
  fn a_line_exactly_the_buffer_size_fits_when_empty() {
    let w = FlowWindow::new(16);
    assert_eq!(w.fits(16), Ok(true));
  }

  #[test]
  fn refining_the_rx_buffer_changes_future_decisions_only() {
    let mut w = FlowWindow::new(8);
    w.on_line_sent(8);
    assert_eq!(w.remaining(), 0);
    w.set_rx_buffer(1024);
    // The already-sent line is untouched; the budget simply grew.
    assert_eq!(w.inflight_bytes(), 8);
    assert_eq!(w.remaining(), 1016);
  }
}
