//! The pure streaming state machine.
//!
//! [`ProtocolCore`] is the load-bearing, framework-agnostic heart of the engine. It owns the
//! character-count [`FlowWindow`], the program queue, the [`ConnectionState`] lifecycle, and the
//! error-hold reaction — but it does NO async, NO time, and NO transport I/O. The async engine drives it
//! by handing it parsed firmware [`Response`]s and host intents, then draining the [`Effect`]s it produces
//! (bytes to write, events to surface). Being pure makes every grbl-contract rule directly testable:
//! script the inputs, assert the emitted bytes and state transitions.
//!
//! ## Contract rules enforced here
//! - **Character counting:** program lines are released only while the next one fits the RX window; each
//!   `ok`/`error:N` frees the oldest in-flight line and may unblock more releases.
//! - **One `ok`/`error` per line:** every acknowledgement frees exactly one in-flight line; an
//!   acknowledgement with nothing in flight is surfaced as an error rather than silently miscounted.
//! - **Error-hold:** an `error:N` mid-program stops releasing further program lines and moves to
//!   [`ConnectionState::Error`] (grblHAL holds subsequent lines until reset / empty line / `$`).
//! - **Banner / alarm reactions:** a banner mid-stream (controller reset) or an `ALARM:N` aborts the
//!   program and transitions the lifecycle accordingly.
//! - **Real-time out-of-band:** real-time bytes are emitted ahead of any queued line bytes and are never
//!   counted against the window.

use std::collections::VecDeque;

use crate::error::EngineError;
use crate::protocol::flow::{DEFAULT_RX_BUFFER, FlowWindow};
use crate::protocol::lifecycle::ConnectionState;
use crate::protocol::realtime::RealtimeCommand;
use crate::protocol::response::{Response, rx_buffer_from_opt};

/// One side effect the core wants the driver to perform or surface. The driver writes [`Effect::Write`]
/// bytes to the transport and forwards every other variant to the UI event channel.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Effect {
  /// Raw bytes to write to the transport, in order. Used for both program/manual lines (which were already
  /// counted against the window) and real-time bytes (which were not).
  Write(Vec<u8>),
  /// The lifecycle moved to a new state; the driver should surface this to the UI.
  StateChanged(ConnectionState),
  /// A parsed firmware response worth surfacing (status, message, banner, error, alarm, ...).
  Response(Response),
  /// Streaming progress: `sent` lines released, `acked` lines acknowledged, `total` lines in the program.
  Progress { sent: usize, acked: usize, total: usize },
  /// A recoverable engine-level fault (e.g. a counting violation). Surfaced, never panicked on.
  Fault(EngineError),
}

/// The pure streaming state machine. Hand it inputs via the `on_*` methods; each returns the ordered
/// [`Effect`]s the driver must carry out.
#[derive(Debug)]
pub struct ProtocolCore {
  state: ConnectionState,
  flow: FlowWindow,
  /// Remaining program lines awaiting release, each already encoded with its trailing terminator.
  program: VecDeque<Vec<u8>>,
  /// Total lines in the currently-loaded program (for progress reporting).
  program_total: usize,
  /// Program lines released to the transport so far.
  program_sent: usize,
  /// Program lines acknowledged so far.
  program_acked: usize,
}

impl Default for ProtocolCore {
  fn default() -> Self {
    Self::new()
  }
}

impl ProtocolCore {
  /// Create a core in [`ConnectionState::Disconnected`] with the default RX-buffer budget.
  pub fn new() -> Self {
    Self {
      state: ConnectionState::Disconnected,
      flow: FlowWindow::new(DEFAULT_RX_BUFFER),
      program: VecDeque::new(),
      program_total: 0,
      program_sent: 0,
      program_acked: 0,
    }
  }

  /// The current lifecycle state.
  pub fn state(&self) -> ConnectionState {
    self.state
  }

  /// Read-only access to the flow window (mainly for tests and diagnostics).
  pub fn flow(&self) -> &FlowWindow {
    &self.flow
  }

  /// Transition to `next`, emitting a [`Effect::StateChanged`] into `out` only if it actually changed.
  fn transition(&mut self, next: ConnectionState, out: &mut Vec<Effect>) {
    if self.state != next {
      self.state = next;
      out.push(Effect::StateChanged(next));
    }
  }

  /// The driver reports the transport is now attached; move Disconnected -> Connecting.
  pub fn on_connected(&mut self) -> Vec<Effect> {
    let mut out = Vec::new();
    self.transition(ConnectionState::Connecting, &mut out);
    out
  }

  /// The driver reports the transport dropped; reset streaming state and move to Disconnected.
  pub fn on_disconnected(&mut self) -> Vec<Effect> {
    let mut out = Vec::new();
    self.clear_program();
    self.flow = FlowWindow::new(self.flow.rx_buffer());
    self.transition(ConnectionState::Disconnected, &mut out);
    out
  }

  /// Load a program and begin streaming. Each line is encoded with a single `\n` terminator (the byte the
  /// firmware counts). A line that can never fit the RX buffer is rejected up front as a [`Effect::Fault`]
  /// and the program is not loaded. Returns the effects of loading plus the first release wave.
  pub fn on_stream_program<I, S>(&mut self, lines: I) -> Vec<Effect>
  where
    I: IntoIterator<Item = S>,
    S: AsRef<str>,
  {
    let mut out = Vec::new();
    self.clear_program();

    for line in lines {
      let encoded = encode_line(line.as_ref());
      // Reject an un-sendable line before we start, so the stream can never deadlock mid-flight.
      match self.flow.fits(encoded.len()) {
        Err(err) => {
          out.push(Effect::Fault(err));
          self.clear_program();
          return out;
        }
        Ok(_) => self.program.push_back(encoded),
      }
    }

    self.program_total = self.program.len();
    if self.program_total == 0 {
      // An empty program is immediately complete; stay/return Idle if connected.
      if self.state.is_connected() {
        self.transition(ConnectionState::Idle, &mut out);
      }
      return out;
    }

    self.transition(ConnectionState::Streaming, &mut out);
    self.release_ready_lines(&mut out);
    self.emit_progress(&mut out);
    out
  }

  /// Send one manual line immediately if it fits the window. Manual lines are counted against the window
  /// just like program lines (the firmware buffers them identically) but are not tracked as program
  /// progress. If it does not fit right now it is dropped with a fault rather than silently queued — manual
  /// sends are meant to be immediate; the UI can retry. Returns the effects.
  pub fn on_send_line(&mut self, line: &str) -> Vec<Effect> {
    let mut out = Vec::new();
    let encoded = encode_line(line);
    match self.flow.fits(encoded.len()) {
      Err(err) => out.push(Effect::Fault(err)),
      Ok(true) => {
        self.flow.on_line_sent(encoded.len());
        out.push(Effect::Write(encoded));
      }
      Ok(false) => out.push(Effect::Fault(EngineError::LineTooLong {
        len: encoded.len(),
        rx_buffer: self.flow.rx_buffer(),
      })),
    }
    out
  }

  /// Inject a real-time command: emit its single byte immediately, out-of-band, never counted against the
  /// window. Also reflects the obvious lifecycle effects of feed-hold / resume / soft-reset locally so the
  /// UI sees them without waiting for a status poll.
  pub fn on_realtime(&mut self, cmd: RealtimeCommand) -> Vec<Effect> {
    let mut out = vec![Effect::Write(vec![cmd.byte()])];
    match cmd {
      RealtimeCommand::FeedHold if self.state == ConnectionState::Streaming => {
        self.transition(ConnectionState::Hold, &mut out);
      }
      RealtimeCommand::CycleStart if self.state == ConnectionState::Hold => {
        self.transition(ConnectionState::Streaming, &mut out);
      }
      RealtimeCommand::SoftReset => {
        // A soft reset aborts everything; the firmware will re-emit the banner, which we also react to.
        self.clear_program();
        self.flow = FlowWindow::new(self.flow.rx_buffer());
        if self.state.is_connected() {
          self.transition(ConnectionState::Idle, &mut out);
        }
      }
      _ => {}
    }
    out
  }

  /// Feed one parsed firmware response into the core. This is where flow control advances (`ok`/`error`),
  /// the RX buffer is learned (`[OPT:...]`), and the banner/alarm/error reactions fire.
  pub fn on_response(&mut self, response: Response) -> Vec<Effect> {
    let mut out = vec![Effect::Response(response.clone())];
    match response {
      Response::Ok => self.on_ack(false, &mut out),
      Response::Error(_) => self.on_ack(true, &mut out),
      Response::Alarm(_) => {
        self.clear_program();
        self.transition(ConnectionState::Alarm, &mut out);
      }
      Response::Banner(_) => {
        // A banner means the controller reset: abort any stream, clear the window, return to Idle.
        self.clear_program();
        self.flow = FlowWindow::new(self.flow.rx_buffer());
        self.transition(ConnectionState::Idle, &mut out);
      }
      Response::Message(body) => {
        if let Some(rx) = rx_buffer_from_opt(&body) {
          self.flow.set_rx_buffer(rx);
        }
      }
      Response::Status(_) | Response::StartupEcho(_) | Response::Unknown(_) => {}
    }
    out
  }

  /// Apply one acknowledgement (`ok` when `is_error` is false, `error:N` when true). Frees the oldest
  /// in-flight line, advances program-acked accounting, applies the error-hold reaction, and — if still
  /// streaming — releases any newly-fitting lines.
  fn on_ack(&mut self, is_error: bool, out: &mut Vec<Effect>) {
    match self.flow.on_ack() {
      Err(err) => {
        // More acks than lines sent: a hard counting violation. Surface it; do not corrupt the window.
        out.push(Effect::Fault(err));
        return;
      }
      Ok(_) => {
        // Account the ack against the program only while a program is actually in flight.
        if self.program_acked < self.program_sent {
          self.program_acked += 1;
        }
      }
    }

    if is_error {
      // Error-hold: stop releasing further program lines and surface the halt. grblHAL holds subsequent
      // lines until reset / empty line / `$`, so blindly streaming on would be unsafe.
      self.clear_program();
      self.transition(ConnectionState::Error, out);
      return;
    }

    if self.state == ConnectionState::Streaming {
      self.release_ready_lines(out);
      self.emit_progress(out);
      // The whole program is done once everything released has also been acknowledged.
      if self.program.is_empty() && !self.flow.has_inflight() {
        self.transition(ConnectionState::Idle, out);
      }
    }
  }

  /// Release as many queued program lines as currently fit the window, in order. Each released line is
  /// counted against the window and emitted as a single [`Effect::Write`].
  fn release_ready_lines(&mut self, out: &mut Vec<Effect>) {
    while let Some(next) = self.program.front() {
      // `fits` only errors on an impossibly-long line, which load-time validation already excluded; treat
      // any error defensively as "does not fit now" and stop, surfacing nothing spurious.
      let fits = self.flow.fits(next.len()).unwrap_or(false);
      if !fits {
        break;
      }
      // The `front()` above guarantees a value; `pop_front` mirrors it. We avoid `expect` in library code by
      // matching, so a logic slip degrades to a no-op break rather than a panic on the hot path.
      let Some(line) = self.program.pop_front() else { break };
      self.flow.on_line_sent(line.len());
      self.program_sent += 1;
      out.push(Effect::Write(line));
    }
  }

  /// Emit a progress snapshot reflecting the current program counters.
  fn emit_progress(&self, out: &mut Vec<Effect>) {
    out.push(Effect::Progress {
      sent: self.program_sent,
      acked: self.program_acked,
      total: self.program_total,
    });
  }

  /// Drop any pending program and reset its counters. Does not touch the flow window's in-flight set —
  /// lines already in the firmware buffer will still be acknowledged and counted down normally.
  fn clear_program(&mut self) {
    self.program.clear();
    self.program_total = 0;
    self.program_sent = 0;
    self.program_acked = 0;
  }
}

/// Encode one G-code line for the wire: trim any terminator the caller included, then append exactly one
/// `\n`. This guarantees the byte-length the host counts matches what the firmware buffers, and that an
/// already-`\n`-terminated input is never double-counted.
fn encode_line(line: &str) -> Vec<u8> {
  let trimmed = line.trim_end_matches(['\r', '\n']);
  let mut bytes = Vec::with_capacity(trimmed.len() + 1);
  bytes.extend_from_slice(trimmed.as_bytes());
  bytes.push(b'\n');
  bytes
}

#[cfg(test)]
mod tests {
  use super::*;

  /// Collect just the `Write` payloads from a slice of effects, in order.
  fn writes(effects: &[Effect]) -> Vec<Vec<u8>> {
    effects.iter().filter_map(|e| match e {
      Effect::Write(bytes) => Some(bytes.clone()),
      _ => None,
    }).collect()
  }

  /// Whether the effects contain a transition to `state`.
  fn transitioned_to(effects: &[Effect], state: ConnectionState) -> bool {
    effects.iter().any(|e| matches!(e, Effect::StateChanged(s) if *s == state))
  }

  fn connected_core() -> ProtocolCore {
    let mut core = ProtocolCore::new();
    core.on_connected();
    core
  }

  #[test]
  fn streaming_releases_lines_that_fit_and_holds_the_rest() {
    let mut core = connected_core();
    // Tiny RX buffer so only a couple of short lines fit at once.
    core.flow.set_rx_buffer(8);
    // Three 4-byte lines ("G0\n" is 3 bytes; "G01\n" is 4). Use lines that encode to 4 bytes each.
    let effects = core.on_stream_program(["G00", "G01", "G02"]);
    // "G00\n" == 4 bytes; budget 8 => exactly two lines released, the third held.
    assert_eq!(writes(&effects), vec![b"G00\n".to_vec(), b"G01\n".to_vec()]);
    assert!(transitioned_to(&effects, ConnectionState::Streaming));
    assert_eq!(core.flow().inflight_bytes(), 8);
  }

  #[test]
  fn each_ok_frees_one_slot_and_releases_the_next_line() {
    let mut core = connected_core();
    core.flow.set_rx_buffer(8);
    core.on_stream_program(["G00", "G01", "G02"]);
    // Window full with two lines; the third is queued. One `ok` should release exactly the third.
    let effects = core.on_response(Response::Ok);
    assert_eq!(writes(&effects), vec![b"G02\n".to_vec()]);
  }

  #[test]
  fn one_ok_per_line_drives_completion_to_idle() {
    let mut core = connected_core();
    core.on_stream_program(["G0 X1", "G0 Y1"]);
    assert_eq!(core.state(), ConnectionState::Streaming);
    // Two lines released into a large default window; two acks complete the program.
    core.on_response(Response::Ok);
    assert_eq!(core.state(), ConnectionState::Streaming);
    let last = core.on_response(Response::Ok);
    assert!(transitioned_to(&last, ConnectionState::Idle));
    assert_eq!(core.state(), ConnectionState::Idle);
  }

  #[test]
  fn an_error_mid_program_halts_the_stream_and_holds_remaining_lines() {
    let mut core = connected_core();
    core.flow.set_rx_buffer(8);
    core.on_stream_program(["G00", "G01", "G02"]); // two released, one queued
    let effects = core.on_response(Response::Error(9));
    // After the error we must NOT release the held line, and must move to the Error state.
    assert!(writes(&effects).is_empty());
    assert!(transitioned_to(&effects, ConnectionState::Error));
    assert_eq!(core.state(), ConnectionState::Error);
    // A subsequent ok (for the second in-flight line) must not resurrect the stream.
    let after = core.on_response(Response::Ok);
    assert!(writes(&after).is_empty());
  }

  #[test]
  fn a_spurious_ack_is_surfaced_as_a_fault_not_a_panic() {
    let mut core = connected_core();
    let effects = core.on_response(Response::Ok); // nothing in flight
    assert!(effects.iter().any(|e| matches!(e, Effect::Fault(EngineError::UnexpectedAck))));
  }

  #[test]
  fn realtime_bytes_are_emitted_out_of_band_and_uncounted() {
    let mut core = connected_core();
    core.on_stream_program(["G0 X10"]);
    let before = core.flow().inflight_bytes();
    let effects = core.on_realtime(RealtimeCommand::StatusReport);
    assert_eq!(writes(&effects), vec![vec![b'?']]);
    // The `?` must not change the character-count window.
    assert_eq!(core.flow().inflight_bytes(), before);
  }

  #[test]
  fn feed_hold_then_resume_toggles_lifecycle_without_releasing_lines() {
    let mut core = connected_core();
    core.flow.set_rx_buffer(8);
    core.on_stream_program(["G00", "G01", "G02"]); // streaming, one line queued
    let hold = core.on_realtime(RealtimeCommand::FeedHold);
    assert_eq!(writes(&hold), vec![vec![b'!']]);
    assert!(transitioned_to(&hold, ConnectionState::Hold));
    // While held, an `ok` frees a slot but must NOT release the queued line.
    let acked = core.on_response(Response::Ok);
    assert!(writes(&acked).is_empty());
    let resume = core.on_realtime(RealtimeCommand::CycleStart);
    assert!(transitioned_to(&resume, ConnectionState::Streaming));
  }

  #[test]
  fn alarm_aborts_the_program_and_enters_alarm_state() {
    let mut core = connected_core();
    core.on_stream_program(["G0 X1", "G0 Y1"]);
    let effects = core.on_response(Response::Alarm(1));
    assert!(transitioned_to(&effects, ConnectionState::Alarm));
    let after = core.on_response(Response::Ok); // ack the in-flight line; must not resume streaming
    assert!(writes(&after).is_empty());
  }

  #[test]
  fn a_banner_mid_stream_aborts_and_returns_to_idle() {
    let mut core = connected_core();
    core.on_stream_program(["G0 X1", "G0 Y1"]);
    let effects = core.on_response(Response::Banner("Grbl 1.1f".to_string()));
    assert!(transitioned_to(&effects, ConnectionState::Idle));
    assert_eq!(core.flow().inflight_bytes(), 0);
  }

  #[test]
  fn opt_message_refines_the_rx_buffer_budget() {
    let mut core = connected_core();
    core.flow.set_rx_buffer(128);
    core.on_response(Response::Message("OPT:VNMSL,100,1024,3,0".to_string()));
    assert_eq!(core.flow().rx_buffer(), 1024);
  }

  #[test]
  fn an_unsendable_program_line_is_rejected_up_front() {
    let mut core = connected_core();
    core.flow.set_rx_buffer(4);
    let effects = core.on_stream_program(["this line is far too long to ever fit"]);
    assert!(effects.iter().any(|e| matches!(e, Effect::Fault(EngineError::LineTooLong { .. }))));
    // Nothing was released and we never entered Streaming.
    assert!(writes(&effects).is_empty());
    assert_ne!(core.state(), ConnectionState::Streaming);
  }

  #[test]
  fn manual_send_counts_against_the_window_but_not_program_progress() {
    let mut core = connected_core();
    let effects = core.on_send_line("$$");
    assert_eq!(writes(&effects), vec![b"$$\n".to_vec()]);
    assert_eq!(core.flow().inflight_bytes(), 3);
  }

  #[test]
  fn encode_line_appends_exactly_one_newline_even_if_already_terminated() {
    assert_eq!(encode_line("G0 X1"), b"G0 X1\n");
    assert_eq!(encode_line("G0 X1\r\n"), b"G0 X1\n");
    assert_eq!(encode_line("G0 X1\n"), b"G0 X1\n");
  }
}
