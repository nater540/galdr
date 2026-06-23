//! The async driver that turns the pure [`ProtocolCore`] into a live, channel-driven streaming engine.
//!
//! This module is deliberately thin. All flow-control intelligence — character counting, error-hold, the
//! lifecycle — lives in [`crate::protocol::ProtocolCore`]; the engine only does the things a state machine
//! cannot: own a [`Transport`], pump inbound bytes through the deframer and parser, carry out the
//! [`Effect`]s the core emits (write granted/real-time bytes, surface events), and bridge the UI over
//! channels. Keeping the hot logic pure means the contract is tested directly against the core, and the
//! engine's own tests only need to prove the plumbing: bytes in the right order, events forwarded, a clean
//! disconnect on EOF.
//!
//! ## Concurrency model
//! The engine runs as one background task (spawned by [`Engine::connect`]). It `select!`s over two sources:
//! inbound transport reads and UI [`Command`]s. Neither blocks the other, and crucially neither blocks a UI
//! thread — the UI only ever touches the [`EngineHandle`]'s non-blocking channels. Transport writes are
//! awaited inline (outside the `select!`) so a real-time byte or granted line is never torn by cancellation.
//!
//! ## What is deliberately deferred
//! - **Reconnection policy.** On EOF / I/O error the task ends after emitting [`Event::Disconnected`]; the
//!   UI decides whether to call [`Engine::connect`] again. Auto-reconnect/backoff is a follow-up.
//! - **`$PBX` settings sync.** Settings exchange via `galdr-proto` is not wired here yet; the engine streams
//!   and parses, but does not yet model the settings channel.
//!
//! `<...>` status reports are surfaced verbatim via [`Event::Response`]; the reducer decodes their fields
//! (state, position, `Pn:` pins, overrides, ...) with [`crate::protocol::parse_status`].

use std::collections::VecDeque;
use std::time::Duration;

use tokio::sync::mpsc::{self, UnboundedReceiver, UnboundedSender};
use tokio::task::JoinHandle;

use crate::error::{EngineError, TransportError};
use crate::protocol::core::Effect;
use crate::protocol::{ConnectionState, LineReassembler, ProtocolCore, RealtimeCommand, Response, parse_line};
use crate::transport::Transport;

/// Size of the inbound read buffer. One USB-CDC packet is at most 64 bytes; a slightly larger buffer lets a
/// burst of status/`ok` lines be drained in a single read without over-allocating.
const READ_CHUNK: usize = 256;

/// How often the engine polls the firmware with a `?` real-time status request while connected. grbl's docs
/// recommend senders poll at no more than 5–10 Hz to avoid overwhelming the controller; 5 Hz (200 ms) is the
/// common sender norm and keeps the DRO / state badge / feed-speed / overrides live without flooding the link.
/// The `?` rides the out-of-band real-time path (uncounted), so polling never disturbs the send-ahead window.
const STATUS_POLL_INTERVAL: Duration = Duration::from_millis(200);

/// A host intent sent from the UI to the engine. Each is fed straight into the [`ProtocolCore`]; the engine
/// adds no policy of its own beyond carrying out the resulting effects.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Command {
  /// Load a program and begin streaming it line-by-line under character-count flow control. Replaces any
  /// program already loaded. Shared as an `Arc<[String]>` so the UI hands off the whole file by pointer rather
  /// than cloning it on every stream start.
  StreamProgram(std::sync::Arc<[String]>),
  /// Send a single manual line immediately if it fits the window (e.g. `$$`, a jog, a one-off move).
  SendLine(String),
  /// Inject a real-time single-byte command out-of-band (`?`/`!`/`~`/soft-reset/overrides/jog-cancel).
  Realtime(RealtimeCommand),
  /// Tear down the connection: stop the engine task and drop the transport.
  Disconnect,
}

/// An event surfaced from the engine to the UI. The UI renders these into its state and requests a repaint;
/// it never blocks on them.
///
/// `Eq` is not derived: [`Event::Response`] can carry a [`Response::ProbeResult`] whose `Vec<f64>` is not `Eq`.
/// `PartialEq` is retained for tests and comparisons; nothing keys an `Event` in a hash/tree set.
#[derive(Debug, Clone, PartialEq)]
pub enum Event {
  /// The lifecycle moved to a new state.
  StateChanged(ConnectionState),
  /// A parsed firmware response worth displaying (status report, message, banner, error, alarm, ...).
  Response(Response),
  /// Streaming progress: `sent` released, `acked` acknowledged, `total` lines in the loaded program.
  Progress { sent: usize, acked: usize, total: usize },
  /// A recoverable engine fault (a counting violation, an un-sendable line). Surfaced, never fatal.
  Fault(EngineError),
  /// The transport ended. `None` is a clean EOF / requested disconnect; `Some` carries the I/O failure. The
  /// engine task has finished by the time this is observed.
  Disconnected(Option<TransportError>),
}

/// The UI-facing handle to a running engine. Holds the command sender, the event receiver, and the task's
/// join handle. Dropping it (or all clones of the command sender) lets the engine task wind down.
#[derive(Debug)]
pub struct EngineHandle {
  command_tx: UnboundedSender<Command>,
  event_rx: UnboundedReceiver<Event>,
  task: JoinHandle<()>,
}

impl EngineHandle {
  /// Send a command to the engine. Returns `false` if the engine task has already ended (its receiver is
  /// gone), so the UI can reflect a dead connection rather than panicking.
  pub fn send(&self, command: Command) -> bool {
    self.command_tx.send(command).is_ok()
  }

  /// Await the next event from the engine, or `None` once the engine task has ended and all events are
  /// drained. UIs that cannot await should use [`Self::try_recv`] from their repaint loop instead.
  pub async fn recv(&mut self) -> Option<Event> {
    self.event_rx.recv().await
  }

  /// Take the next pending event without awaiting, for polling from a UI frame. Returns `None` when no event
  /// is queued right now.
  pub fn try_recv(&mut self) -> Option<Event> {
    self.event_rx.try_recv().ok()
  }

  /// The background task handle, for callers that want to await a clean shutdown or abort the engine.
  pub fn task(&mut self) -> &mut JoinHandle<()> {
    &mut self.task
  }
}

/// The streaming engine. [`Engine::connect`] is the only constructor: it takes an already-open transport and
/// spawns the driver task, handing back an [`EngineHandle`].
pub struct Engine;

impl Engine {
  /// Spawn the engine task over `transport` and return the UI handle. Must be called from within a Tokio
  /// runtime (the binary's `#[tokio::main]`, or a `#[tokio::test]`); it uses `tokio::spawn` internally.
  pub fn connect<T>(transport: T) -> EngineHandle
  where
    T: Transport + 'static,
  {
    let (command_tx, command_rx) = mpsc::unbounded_channel();
    let (event_tx, event_rx) = mpsc::unbounded_channel();
    let driver = Driver {
      transport,
      core: ProtocolCore::new(),
      reassembler: LineReassembler::new(),
      event_tx,
      realtime_out: VecDeque::new(),
      line_out: VecDeque::new(),
    };
    let task = tokio::spawn(driver.run(command_rx));
    EngineHandle { command_tx, event_rx, task }
  }
}

/// The owned state of the running engine task. Not public — the UI only ever sees [`EngineHandle`].
struct Driver<T: Transport> {
  transport: T,
  core: ProtocolCore,
  reassembler: LineReassembler,
  event_tx: UnboundedSender<Event>,
  /// Pending out-of-band real-time bytes, flattened in emit order. Always flushed to the wire *before* any
  /// queued program/manual line so a `?`/`!`/`0x18` never sits behind a line under serial backpressure. Tiny
  /// (one or two bytes per command), so flushing the whole buffer at once is cheap.
  realtime_out: VecDeque<u8>,
  /// Pending counted lines awaiting the wire, oldest at the front. The core only enqueues a line here once it
  /// fits the character-count window, so these are already cleared to send and simply need transport bandwidth;
  /// writing one is the *lowest* write priority, racing against commands/poll/realtime so it can never wedge the
  /// loop while it parks under backpressure.
  line_out: VecDeque<Vec<u8>>,
}

impl<T: Transport> Driver<T> {
  /// The driver loop. Announces the connection, then interleaves inbound reads with UI commands until the
  /// transport ends, a write fails, or the UI hangs up. Always emits exactly one [`Event::Disconnected`] on
  /// the way out so the UI sees a definitive end.
  async fn run(mut self, mut command_rx: UnboundedReceiver<Command>) {
    // The transport is attached: drive Disconnected -> Connecting and surface it before any I/O.
    let connect_effects = self.core.on_connected();
    self.apply_effects(connect_effects);

    // Actively elicit readiness instead of waiting passively for a banner the board may never send: probe with
    // `?`/`0x87` and a counted `$I`. Any readiness evidence (status / banner / `ok` / `[VER:]`/`[OPT:]`) leaves
    // Connecting via the core. These bytes are enqueued now and flushed by the loop's prioritised write arms; a
    // dead link surfaces as a write failure on the first flush.
    let handshake_effects = self.core.begin_handshake();
    self.apply_effects(handshake_effects);

    // The live status poller: a periodic `?` keeps the DRO / state badge / feed-speed / overrides fresh. The
    // first tick fires one interval from now (the handshake already sent an initial `?`), and ticks ride the
    // out-of-band real-time path so polling never touches the character-count window. The interval is owned
    // here (the only timing the otherwise-pure core cannot do); it stops the instant this loop ends on
    // disconnect. `MissedTickBehavior::Delay` (the default) avoids a tick storm if the loop was briefly busy.
    let mut status_poll = tokio::time::interval_at(
      tokio::time::Instant::now() + STATUS_POLL_INTERVAL,
      STATUS_POLL_INTERVAL,
    );

    let mut read_buf = [0u8; READ_CHUNK];
    let disconnect_reason = loop {
      // Priority 1: flush every pending real-time byte first, out-of-band, before any line write or read. These
      // are tiny; we write the whole buffer at once. The flush itself races a Disconnect/Stop so it can never
      // wedge the loop under backpressure. A write failure ends the loop.
      match self.flush_realtime(&mut command_rx).await {
        FlushOutcome::Continue => {}
        FlushOutcome::WriteFailed(err) => break Some(err),
        FlushOutcome::Disconnect => break None,
      }

      // Priority 2: if a counted line is queued, write it — but raced against commands and the poll (NOT reads),
      // so a Stop/Disconnect/poll/realtime cancels a parked line write under backpressure instead of wedging the
      // loop. Reads are deliberately excluded from this race: only ONE arm may borrow the transport per
      // `select!`, and buffered `ok`s lost no ground while we write (we drain them the moment the write returns).
      if !self.line_out.is_empty() {
        match self.write_pending_line(&mut command_rx, &mut status_poll).await {
          FlushOutcome::Continue => continue,
          FlushOutcome::WriteFailed(err) => break Some(err),
          FlushOutcome::Disconnect => break None,
        }
      }

      // Priority 3: nothing to write — wait on commands, the poll, or inbound reads. Commands and the poll are
      // biased ahead of reads so a hot inbound stream can never starve the `?` poll (the DRO-freeze fix).
      tokio::select! {
        biased;

        command = command_rx.recv() => {
          match command {
            // All command senders dropped: the UI is gone. Wind down cleanly.
            None => break None,
            Some(Command::Disconnect) => break None,
            Some(command) => self.handle_command(command),
          }
        }

        _ = status_poll.tick() => {
          // Inject a `?` out-of-band through the core (uncounted); the next loop iteration's realtime flush emits
          // it. Enqueuing is infallible, so the poll never parks the loop.
          let poll_effects = self.core.on_realtime(RealtimeCommand::StatusReport);
          self.apply_effects(poll_effects);
        }

        read = self.transport.read(&mut read_buf) => {
          match read {
            // Ok(0) is end-of-stream — the device disappeared. A clean disconnect, not an error.
            Ok(0) => break None,
            Ok(n) => self.handle_inbound(&read_buf[..n]),
            Err(err) => break Some(err),
          }
        }
      }
    };

    // Best-effort teardown flush of any pending real-time bytes (most importantly a soft-reset/feed-hold the
    // operator issued just before disconnecting): leaving the machine in a safe stopped state matters more than a
    // microsecond of teardown latency. The write is awaited so a Stop that was waiting on serial room still
    // reaches the wire once room frees; a genuinely dead port returns an error promptly rather than hanging, so
    // this cannot wedge teardown. We do NOT flush queued program lines here — a teardown must not push more of an
    // aborted job at the controller.
    if !self.realtime_out.is_empty() {
      let bytes: Vec<u8> = self.realtime_out.drain(..).collect();
      let _ = self.transport.write_all(&bytes).await;
    }

    // Reset the core's view to Disconnected (best-effort: surface any resulting state change) and emit the
    // single terminal event. Errors sending the event mean the UI is already gone; nothing more to do.
    let down = self.core.on_disconnected();
    self.apply_effects(down);
    let _ = self.event_tx.send(Event::Disconnected(disconnect_reason));
  }

  /// Flush every pending real-time byte to the wire, out-of-band and ahead of any queued line. The write is
  /// raced against `command_rx` so a Disconnect arriving while the flush parks under backpressure still ends the
  /// loop promptly; any other command queued during the race is fed into the core (it may add more realtime
  /// bytes, which this same flush then drains before returning). Returns the outcome for the loop to act on.
  async fn flush_realtime(&mut self, command_rx: &mut UnboundedReceiver<Command>) -> FlushOutcome {
    while !self.realtime_out.is_empty() {
      let bytes: Vec<u8> = self.realtime_out.drain(..).collect();
      tokio::select! {
        biased;

        // A command racing the realtime write: a Disconnect ends the loop even mid-flush; any other command is
        // handled (it may enqueue more realtime bytes). The bytes we drained are re-queued at the FRONT so they
        // are not lost when we loop to retry the write — order is preserved.
        command = command_rx.recv() => {
          self.requeue_realtime_front(bytes);
          match command {
            None | Some(Command::Disconnect) => return FlushOutcome::Disconnect,
            Some(command) => self.handle_command(command),
          }
        }

        result = self.transport.write_all(&bytes) => {
          if let Err(err) = result {
            return FlushOutcome::WriteFailed(err);
          }
        }
      }
    }
    FlushOutcome::Continue
  }

  /// Put already-drained real-time bytes back at the front of the queue, preserving their order, so a write
  /// cancelled by a racing command is retried on the next flush rather than dropped.
  fn requeue_realtime_front(&mut self, bytes: Vec<u8>) {
    for byte in bytes.into_iter().rev() {
      self.realtime_out.push_front(byte);
    }
  }

  /// Feed one UI command into the core and enqueue the resulting effects. `Disconnect` is handled by the caller
  /// (it breaks the loop) and never reaches here.
  fn handle_command(&mut self, command: Command) {
    let effects = match command {
      Command::StreamProgram(lines) => self.core.on_stream_program(lines.iter()),
      Command::SendLine(line) => self.core.on_send_line(&line),
      Command::Realtime(cmd) => self.core.on_realtime(cmd),
      Command::Disconnect => Vec::new(),
    };
    self.apply_effects(effects);
  }

  /// Deframe a chunk of inbound bytes into lines, parse each, and feed every recognised response into the
  /// core, enqueuing the resulting effects. Empty lines and unparseable noise are dropped here (an empty line is
  /// not an `ok`). Enqueuing is infallible, so processing inbound bytes never parks the loop — newly-released
  /// lines wait in `line_out` and are written by the loop's prioritised line-write arm.
  fn handle_inbound(&mut self, bytes: &[u8]) {
    for line in self.reassembler.push(bytes) {
      if let Some(response) = parse_line(&line) {
        let effects = self.core.on_response(response);
        self.apply_effects(effects);
      }
    }
  }

  /// Sort an ordered batch of core effects into the outbound queues and forward every informational effect to
  /// the UI. This does NO I/O — real-time bytes go to `realtime_out` (drained first, out-of-band) and counted
  /// lines to `line_out` (drained by the low-priority line-write arm). Keeping this synchronous and infallible
  /// is the heart of the fix: feeding the core can never park the loop or block a queued Stop/Disconnect/poll.
  fn apply_effects(&mut self, effects: Vec<Effect>) {
    for effect in effects {
      match effect {
        Effect::WriteRealtime(bytes) => self.realtime_out.extend(bytes),
        Effect::WriteLine(bytes) => self.line_out.push_back(bytes),
        Effect::StateChanged(state) => self.emit(Event::StateChanged(state)),
        Effect::Response(response) => self.emit(Event::Response(response)),
        Effect::Progress { sent, acked, total } => self.emit(Event::Progress { sent, acked, total }),
        Effect::Fault(err) => self.emit(Event::Fault(err)),
      }
    }
  }

  /// Write the front queued line to the transport, raced against commands and the status poll so a parked line
  /// write (serial backpressure) never wedges the loop: a Disconnect ends it, any other command (including a
  /// real-time Stop) is fed to the core and cancels the write so the loop re-runs the realtime flush first, and a
  /// poll tick is honoured. On a successful write the line is popped. Returns [`FlushOutcome::Continue`] both on
  /// a completed write and on a command/poll that pre-empted it (the caller `continue`s either way).
  async fn write_pending_line(
    &mut self,
    command_rx: &mut UnboundedReceiver<Command>,
    status_poll: &mut tokio::time::Interval,
  ) -> FlushOutcome {
    // The caller checked `!line_out.is_empty()`, so a front line is present; treat its absence defensively as a
    // no-op rather than panicking. Clone the front line to write so the transport borrow does not collide with
    // the `&mut self` the command/poll arms need; the line is small (one G-code line) so the copy is cheap.
    let Some(line) = self.line_out.front().cloned() else {
      return FlushOutcome::Continue;
    };
    tokio::select! {
      biased;

      command = command_rx.recv() => {
        match command {
          None | Some(Command::Disconnect) => FlushOutcome::Disconnect,
          // A command pre-empts the (still-unsent) line: feed it to the core — a real-time Stop now sits in
          // `realtime_out` and the next loop iteration flushes it ahead of this line, which stays queued.
          Some(command) => {
            self.handle_command(command);
            FlushOutcome::Continue
          }
        }
      }

      _ = status_poll.tick() => {
        let poll_effects = self.core.on_realtime(RealtimeCommand::StatusReport);
        self.apply_effects(poll_effects);
        FlushOutcome::Continue
      }

      result = self.transport.write_all(&line) => {
        match result {
          Ok(()) => {
            self.line_out.pop_front();
            FlushOutcome::Continue
          }
          Err(err) => FlushOutcome::WriteFailed(err),
        }
      }
    }
  }

  /// Forward one event to the UI. A failure means the UI dropped its receiver; we simply stop emitting (the
  /// loop will end on its own once commands dry up).
  fn emit(&self, event: Event) {
    let _ = self.event_tx.send(event);
  }
}

/// The result of a real-time flush attempt, telling the loop whether to carry on, end on a write failure, or
/// end on a Disconnect that raced the flush.
enum FlushOutcome {
  Continue,
  WriteFailed(TransportError),
  Disconnect,
}

#[cfg(test)]
mod tests {
  use std::sync::Arc;
  use std::sync::atomic::{AtomicBool, Ordering};

  use tokio::sync::Notify;

  use super::*;
  use crate::transport::loopback::{LoopbackController, LoopbackTransport};

  /// Spin up an engine over a fresh loopback and return its handle plus the controller.
  fn connect() -> (EngineHandle, LoopbackController) {
    let (transport, controller) = LoopbackTransport::new();
    (Engine::connect(transport), controller)
  }

  /// Drain events until one satisfying `pred` arrives, or fail after a bounded number of awaits. Keeps tests
  /// from hanging if the engine never produces the expected event.
  async fn wait_for<F>(handle: &mut EngineHandle, mut pred: F) -> Event
  where
    F: FnMut(&Event) -> bool,
  {
    for _ in 0..256 {
      match handle.recv().await {
        Some(event) if pred(&event) => return event,
        Some(_) => continue,
        None => panic!("engine ended before the expected event"),
      }
    }
    panic!("expected event never arrived within the event budget");
  }

  #[tokio::test]
  async fn connecting_emits_a_connecting_state_change() {
    let (mut handle, _controller) = connect();
    let event = wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Connecting))).await;
    assert_eq!(event, Event::StateChanged(ConnectionState::Connecting));
  }

  #[tokio::test]
  async fn a_streamed_program_writes_its_lines_and_progresses_to_idle() {
    let (mut handle, mut controller) = connect();
    // Settle the connect handshake first: a banner means the board reset, which leaves Connecting for Idle AND
    // resets the flow window (discarding the in-flight `$I`), so the window is empty before we stream. Drain the
    // handshake's own writes so we assert only the program bytes below.
    assert!(controller.inject_line("GrblHAL 1.1f ['$' or '$HELP' for help]"));
    wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Idle))).await;
    let _ = controller.drain_written();

    // Stream two short lines. With the default 1024-byte window both release immediately.
    assert!(handle.send(Command::StreamProgram(vec!["G0 X1".to_string(), "G0 Y1".to_string()].into())));
    wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Streaming))).await;

    // Acknowledge both lines; the second `ok` should complete the program back to Idle.
    assert!(controller.inject_line("ok"));
    assert!(controller.inject_line("ok"));
    wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Idle))).await;

    // Exactly the two encoded lines were streamed, in order, each with a single `\n`.
    let written = controller.drain_written();
    assert_eq!(written, b"G0 X1\nG0 Y1\n");
  }

  #[tokio::test]
  async fn flow_control_holds_a_line_until_an_ok_frees_room() {
    let (mut handle, mut controller) = connect();
    // Shrink the window via an OPT message so only two 4-byte lines fit at once. (`G00\n` == 4 bytes.) The OPT
    // line leaves Connecting; the trailing `ok` is the `$I` reply terminator, which frees the in-flight `$I` so
    // the 8-byte window is empty before we stream. Drain the handshake's own writes after.
    assert!(controller.inject_line("[OPT:VNMSL,100,8,3,0]"));
    wait_for(&mut handle, |e| matches!(e, Event::Response(Response::Message(_)))).await;
    assert!(controller.inject_line("ok")); // the `$I` reply's terminating `ok`
    wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Idle))).await;
    let _ = controller.drain_written();

    assert!(handle.send(Command::StreamProgram(vec!["G00".to_string(), "G01".to_string(), "G02".to_string()].into())));
    wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Streaming))).await;
    // Only the first two lines fit the 8-byte window; the third is held.
    wait_for(&mut handle, |e| matches!(e, Event::Progress { sent: 2, .. })).await;
    assert_eq!(controller.drain_written(), b"G00\nG01\n");

    // One `ok` frees a slot and releases exactly the third line.
    assert!(controller.inject_line("ok"));
    wait_for(&mut handle, |e| matches!(e, Event::Progress { sent: 3, .. })).await;
    assert_eq!(controller.drain_written(), b"G02\n");
  }

  #[tokio::test]
  async fn a_realtime_byte_is_written_out_of_band_and_uncounted() {
    let (handle, mut controller) = connect();
    // Drain the connect handshake writes (`?0x87` then `$I\n`) so the next write is the realtime byte under test.
    assert_eq!(wait_for_written(&mut controller).await, vec![b'?', 0x87]);
    assert_eq!(wait_for_written(&mut controller).await, b"$I\n");
    assert!(handle.send(Command::Realtime(RealtimeCommand::StatusReport)));
    let written = wait_for_written(&mut controller).await;
    assert_eq!(written, vec![b'?']);
  }

  #[tokio::test]
  async fn an_error_mid_stream_halts_and_holds_remaining_lines() {
    let (mut handle, mut controller) = connect();
    assert!(controller.inject_line("[OPT:VNMSL,100,8,3,0]"));
    wait_for(&mut handle, |e| matches!(e, Event::Response(Response::Message(_)))).await;
    assert!(controller.inject_line("ok")); // the `$I` reply's terminating `ok`, freeing the in-flight `$I`
    wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Idle))).await;
    let _ = controller.drain_written();

    assert!(handle.send(Command::StreamProgram(vec!["G00".to_string(), "G01".to_string(), "G02".to_string()].into())));
    wait_for(&mut handle, |e| matches!(e, Event::Progress { sent: 2, .. })).await;
    assert_eq!(controller.drain_written(), b"G00\nG01\n");

    // The firmware rejects the first line. The stream must halt and never release the held third line.
    assert!(controller.inject_line("error:9"));
    wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Error))).await;
    // A following `ok` (for the second in-flight line) must not resurrect the stream.
    assert!(controller.inject_line("ok"));
    // No further line bytes are ever written.
    assert!(controller.drain_written().is_empty());
  }

  #[tokio::test]
  async fn eof_emits_a_clean_disconnect() {
    let (mut handle, mut controller) = connect();
    controller.close();
    let event = wait_for(&mut handle, |e| matches!(e, Event::Disconnected(_))).await;
    assert_eq!(event, Event::Disconnected(None));
  }

  #[tokio::test]
  async fn a_disconnect_command_ends_the_engine() {
    let (mut handle, _controller) = connect();
    assert!(handle.send(Command::Disconnect));
    let event = wait_for(&mut handle, |e| matches!(e, Event::Disconnected(_))).await;
    assert_eq!(event, Event::Disconnected(None));
  }

  /// Await the next chunk the engine writes, failing rather than hanging if it never writes.
  async fn wait_for_written(controller: &mut LoopbackController) -> Vec<u8> {
    controller.next_written().await.expect("engine wrote no bytes before ending")
  }

  #[tokio::test]
  async fn the_connect_handshake_actively_probes_the_board() {
    let (_handle, mut controller) = connect();
    // On connect the engine must elicit readiness: the uncounted real-time probes (`?`, `0x87`) then a counted
    // `$I` build-info query. It must NOT send a soft reset (`0x18`) — that would clobber any in-progress job.
    let probes = wait_for_written(&mut controller).await;
    assert_eq!(probes, vec![b'?', 0x87]);
    let build_info = wait_for_written(&mut controller).await;
    assert_eq!(build_info, b"$I\n");
    assert!(!probes.contains(&0x18) && !build_info.contains(&0x18), "connect must never auto soft-reset");
  }

  #[tokio::test]
  async fn a_status_report_leaves_connecting_without_a_banner() {
    let (mut handle, controller) = connect();
    wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Connecting))).await;
    // An already-booted Idle board answers the handshake `?` with a status and never sends a banner. The engine
    // must still go live — this is the exact bug: previously it hung in Connecting forever.
    assert!(controller.inject_line("<Idle|MPos:0.000,0.000,0.000|FS:0,0|Bf:32,1024>"));
    wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Idle))).await;
  }

  #[tokio::test]
  async fn a_status_report_surfaces_its_decoded_endstop_pins_end_to_end() {
    use crate::protocol::parse_status;
    let (mut handle, controller) = connect();
    wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Connecting))).await;

    // An Alarm report with all three limits tripped must reach the UI verbatim so the reducer/parser can decode
    // X/Y/Z. We assert on the engine boundary (the `Event::Response` it forwards), then decode it as the reducer
    // would, proving the structured pin state is recoverable end to end without a real board.
    assert!(controller.inject_line("<Alarm|MPos:0.000,0.000,0.000|Pn:XYZ>"));
    let event = wait_for(&mut handle, |e| matches!(e, Event::Response(Response::Status(_)))).await;
    let Event::Response(Response::Status(body)) = event else {
      unreachable!("matched a Status response above");
    };
    let pins = parse_status(&body).pin_state();
    assert!(pins.limit_x && pins.limit_y && pins.limit_z, "all three endstops decode as asserted");
    assert!(pins.any_xyz_limit());

    // A later running report carries no `Pn:` field at all (grblHAL omits it when nothing is asserted); the
    // decoded set must read all-clear, so the endstop chips drop back to their inactive look.
    assert!(controller.inject_line("<Run|MPos:1.000,2.000,3.000|FS:500,0>"));
    let event = wait_for(&mut handle, |e| {
      matches!(e, Event::Response(Response::Status(b)) if b.starts_with("Run"))
    }).await;
    let Event::Response(Response::Status(body)) = event else {
      unreachable!("matched the Run Status response above");
    };
    let pins = parse_status(&body).pin_state();
    assert!(!pins.any_xyz_limit(), "an absent Pn: field clears every endstop");
  }

  #[tokio::test]
  async fn an_already_held_board_is_adopted_as_hold_on_connect() {
    let (mut handle, controller) = connect();
    wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Connecting))).await;
    // The real board was observed booting into Hold; the engine must reflect Hold, not fake Idle.
    assert!(controller.inject_line("<Hold:0|WPos:5.000,0.000,0.000|FS:0,0|Bf:32,1024>"));
    wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Hold))).await;
  }

  #[tokio::test(start_paused = true)]
  async fn the_status_poller_emits_a_question_mark_every_interval() {
    let (mut handle, mut controller) = connect();
    // Drain the connect handshake writes (`?0x87` then `$I\n`) so the next writes are poller traffic.
    assert_eq!(wait_for_written(&mut controller).await, vec![b'?', 0x87]);
    assert_eq!(wait_for_written(&mut controller).await, b"$I\n");
    // Become live so the loop is steady-state; the poller runs regardless, but this mirrors a real session.
    assert!(controller.inject_line("<Idle|MPos:0,0,0|FS:0,0>"));
    wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Idle))).await;

    // With time paused, no poll has fired yet. Advancing one interval must produce exactly one `?` poll.
    tokio::time::advance(STATUS_POLL_INTERVAL).await;
    assert_eq!(wait_for_written(&mut controller).await, vec![b'?']);
    // A second interval produces a second poll, proving the poller is periodic, not one-shot.
    tokio::time::advance(STATUS_POLL_INTERVAL).await;
    assert_eq!(wait_for_written(&mut controller).await, vec![b'?']);
  }

  #[tokio::test]
  async fn the_real_opt_line_sizes_the_flow_window_to_the_advertised_buffer() {
    let (mut handle, mut controller) = connect();
    wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Connecting))).await;
    // The board's `$I` reply advertises a 1024-byte RX buffer; the engine adopts it and goes live. We prove the
    // sizing took effect by streaming a program whose lines all fit the 1024 window and complete to Idle.
    assert!(controller.inject_line("[OPT:VNMSL,32,1024,3,0]"));
    wait_for(&mut handle, |e| matches!(e, Event::Response(Response::Message(_)))).await;
    wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Idle))).await;
    let _ = controller.drain_written();

    assert!(handle.send(Command::StreamProgram(vec!["G0 X1".to_string(), "G0 Y1".to_string()].into())));
    wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Streaming))).await;
    // Both lines release immediately into the 1024-byte window (they would not under a tiny default).
    wait_for(&mut handle, |e| matches!(e, Event::Progress { sent: 2, .. })).await;
  }

  /// Drive a fresh loopback to Idle via the welcome banner (resets the window, discarding the in-flight `$I`),
  /// then drain the handshake's own writes so a test asserts only on what it streams next.
  async fn connect_idle() -> (EngineHandle, LoopbackController) {
    let (mut handle, mut controller) = connect();
    assert!(controller.inject_line("GrblHAL 1.1f ['$' or '$HELP' for help]"));
    wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Idle))).await;
    let _ = controller.drain_written();
    (handle, controller)
  }

  #[tokio::test]
  async fn a_realtime_byte_preempts_a_pending_line_write_on_the_wire() {
    // Half the streaming-deadlock bug: under serial backpressure a program-line write parks the driver and the
    // real-time byte is queued BEHIND the held line instead of jumping ahead of it. We gate the transport so a
    // line write parks, queue a Stop (soft reset) while it parks, then release the gate and prove the `0x18`
    // reaches the wire BEFORE the held line — the grbl out-of-band contract.
    let (mut handle, mut controller) = connect_idle().await;

    // Gate the transport so the program-line write parks (firmware RX full mid-cut), then stream a line. Use a
    // large window so the single line releases immediately and its write is what parks on the gate.
    controller.gate_writes();
    assert!(handle.send(Command::StreamProgram(vec!["G0 X100".to_string()].into())));
    wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Streaming))).await;
    wait_for(&mut handle, |e| matches!(e, Event::Progress { sent: 1, .. })).await;

    // Keep the read arm hot with a burst of `ok`s so the driver must not starve on reads either. (These ack the
    // in-flight line and any leftover handshake budget; extras are tolerated by the core's trailing-ack budget.)
    for _ in 0..4 {
      assert!(controller.inject_line("ok"));
    }

    // Operator hits Stop while the line write parks: a soft reset must be emitted out-of-band, ahead of the line.
    assert!(handle.send(Command::Realtime(RealtimeCommand::SoftReset)));

    // Release the backpressure so pending writes flush. The `0x18` must precede the program line on the wire.
    controller.release_writes();
    // The first chunk on the wire after release must be the soft-reset byte, not the held line.
    let first = wait_for_written(&mut controller).await;
    assert_eq!(first, vec![0x18], "the soft-reset byte must reach the wire before the held program line");
  }

  #[tokio::test]
  async fn disconnect_ends_the_loop_while_a_line_write_is_pending() {
    // The other half of the deadlock: under serial backpressure the driver parks inside a line write, so a queued
    // Disconnect is never serviced and Stop/Disconnect appear dead. Gate the transport so a line write parks,
    // keep the read arm hot, then send Disconnect and assert the loop ends PROMPTLY despite the parked write —
    // without ever releasing the gate, proving the line write does not have to complete first.
    let (mut handle, controller) = connect_idle().await;

    controller.gate_writes();
    assert!(handle.send(Command::StreamProgram(vec!["G0 X100".to_string()].into())));
    wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Streaming))).await;
    wait_for(&mut handle, |e| matches!(e, Event::Progress { sent: 1, .. })).await;
    for _ in 0..4 {
      assert!(controller.inject_line("ok"));
    }

    // Disconnect while the line write is still parked on the closed gate. The loop must end without us ever
    // releasing the gate — a parked line write must not hold teardown hostage. (No pending realtime byte means
    // the teardown flush is a no-op, so it cannot block on the still-closed gate.)
    assert!(handle.send(Command::Disconnect));
    let event = wait_for(&mut handle, |e| matches!(e, Event::Disconnected(_))).await;
    assert_eq!(event, Event::Disconnected(None), "Disconnect ends the loop even while a line write pended");
  }

  #[tokio::test(start_paused = true)]
  async fn the_status_poll_is_not_starved_by_a_saturated_read_stream() {
    // The DRO-freeze bug, reproduced deterministically. A transport whose `read` is immediately ready (no yield)
    // for a bounded burst models a firmware streaming `ok`s continuously during a hot cut. Awaiting an
    // already-ready future does NOT yield to the scheduler, so under a read-before-poll `biased` ordering the
    // engine drains the WHOLE burst synchronously before the (already-armed) poll timer ever wins — the `?`
    // status poll (the ONLY source of `<...>`, since the firmware ships `$481=0`) is starved until the burst
    // ends, and the DRO freezes for the duration of the cut. The fix orders the poll AHEAD of the read so the
    // `?` escapes on the first iteration after the timer arms, before the burst is drained.
    //
    // We make each `ok` release a fresh program line, so a read-driven iteration produces a line WRITE. We then
    // assert the `?` poll appears on the wire BEFORE those line writes — true only when the poll is not starved.
    let burst = 32usize;
    let (transport, controls, mut writes) = BurstReadyTransport::new(burst);
    let mut handle = Engine::connect(transport);
    assert_eq!(writes.recv().await, Some(vec![b'?', 0x87]));
    assert_eq!(writes.recv().await, Some(b"$I\n".to_vec()));
    // The transport's first chunk is the OPT line, sizing the window and leaving Connecting for Idle.
    wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Idle))).await;
    let _ = drain_chan(&mut writes);

    // Load a long program of fixed 6-byte lines. With the advertised 6-byte window only ONE is in flight at a
    // time, so each burst `ok` releases exactly one held line — every read-driven iteration is one line write.
    let lines: Vec<String> = (0..burst + 4).map(|_| "G0 X1".to_string()).collect();
    assert!(handle.send(Command::StreamProgram(lines.into())));
    wait_for(&mut handle, |e| matches!(e, Event::StateChanged(ConnectionState::Streaming))).await;
    let _ = drain_chan(&mut writes);

    // Arm the poll timer, release the held `ok` burst, and let the loop run to a quiescent point. Then assert the
    // `?` poll reached the wire BEFORE the burst of line writes it would otherwise be starved behind.
    tokio::time::advance(STATUS_POLL_INTERVAL).await;
    controls.release();
    let wire = settle_chan(&mut writes).await;
    let poll_at = wire.iter().position(|c| c.as_slice() == b"?").expect("a `?` poll reached the wire");
    let first_line_at = wire.iter().position(|c| c.ends_with(b"\n") && c.len() > 1);
    if let Some(line_at) = first_line_at {
      assert!(poll_at < line_at, "the `?` poll must not be starved behind the burst of program-line writes");
    }
  }

  /// Drain every chunk currently queued on a write channel, non-blocking.
  fn drain_chan(writes: &mut UnboundedReceiver<Vec<u8>>) -> Vec<Vec<u8>> {
    let mut all = Vec::new();
    while let Ok(chunk) = writes.try_recv() {
      all.push(chunk);
    }
    all
  }

  /// Let the engine run to a quiescent point, then return every chunk it wrote, in order. Yields repeatedly so
  /// the engine task makes progress; stops once no new write has appeared for a few consecutive yields.
  async fn settle_chan(writes: &mut UnboundedReceiver<Vec<u8>>) -> Vec<Vec<u8>> {
    let mut all = Vec::new();
    let mut idle = 0;
    while idle < 8 {
      match writes.try_recv() {
        Ok(chunk) => {
          all.push(chunk);
          idle = 0;
        }
        Err(_) => {
          tokio::task::yield_now().await;
          idle += 1;
        }
      }
    }
    all
  }

  /// A [`Transport`] that first delivers the `[OPT:...]` build-info line (leaving Connecting), then pends until
  /// the test releases a bounded burst of immediately-ready `ok` reads. Within the burst each `read` returns
  /// synchronously (no `.await` that pends), so awaiting them never returns to the scheduler — the engine drains
  /// the whole burst in one synchronous run under a read-first loop. Once the burst is spent the read pends
  /// forever and the loop settles. Used to prove the `?` poll is not starved behind a synchronous read burst.
  struct BurstReadyTransport {
    /// Chunks to deliver before the burst is released (just the OPT line). Drained immediately.
    prelude: std::collections::VecDeque<Vec<u8>>,
    /// The `ok` burst, made available only after [`release`] is notified, then drained synchronously.
    burst: std::collections::VecDeque<Vec<u8>>,
    /// Whether the burst has been released by the test.
    released: Arc<AtomicBool>,
    /// Notified when the test releases the burst, waking the pending read.
    release: Arc<Notify>,
    writes: UnboundedSender<Vec<u8>>,
  }

  impl BurstReadyTransport {
    /// Build the transport and the receiver for everything it writes. The burst stays held until [`release`].
    fn new(burst_len: usize) -> (Self, BurstControls, UnboundedReceiver<Vec<u8>>) {
      let (writes, rx) = mpsc::unbounded_channel();
      let mut prelude = std::collections::VecDeque::new();
      // Advertise a tiny 6-byte RX buffer so only one `G0 Xn\n` (6 bytes) is in flight at a time; each burst `ok`
      // then releases exactly one held line, turning every read-driven iteration into a single line write.
      prelude.push_back(b"[OPT:VNMSL,32,6,3,0]\n".to_vec());
      let mut burst = std::collections::VecDeque::new();
      for _ in 0..burst_len {
        burst.push_back(b"ok\n".to_vec());
      }
      let released = Arc::new(AtomicBool::new(false));
      let release = Arc::new(Notify::new());
      let controls = BurstControls { released: released.clone(), release: release.clone() };
      let transport = Self { prelude, burst, released, release, writes };
      (transport, controls, rx)
    }
  }

  /// Test-side handle to release a [`BurstReadyTransport`]'s held `ok` burst.
  struct BurstControls {
    released: Arc<AtomicBool>,
    release: Arc<Notify>,
  }

  impl BurstControls {
    /// Release the held burst so the transport's pending read wakes and delivers it.
    fn release(&self) {
      self.released.store(true, Ordering::SeqCst);
      self.release.notify_waiters();
    }
  }

  impl Transport for BurstReadyTransport {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
      if let Some(chunk) = self.prelude.pop_front() {
        let n = chunk.len().min(buf.len());
        buf[..n].copy_from_slice(&chunk[..n]);
        return Ok(n);
      }
      // Wait for the test to release the burst (only the FIRST post-prelude read parks here; once released the
      // remaining burst chunks return synchronously).
      while !self.released.load(Ordering::SeqCst) {
        let notified = self.release.notified();
        if self.released.load(Ordering::SeqCst) {
          break;
        }
        notified.await;
      }
      match self.burst.pop_front() {
        Some(chunk) => {
          let n = chunk.len().min(buf.len());
          buf[..n].copy_from_slice(&chunk[..n]);
          Ok(n)
        }
        // The burst is spent: pend forever so the read arm is no longer ready and the loop settles.
        None => std::future::pending().await,
      }
    }

    async fn write_all(&mut self, data: &[u8]) -> Result<(), TransportError> {
      self.writes.send(data.to_vec()).map_err(|_| TransportError::Closed)
    }
  }
}
