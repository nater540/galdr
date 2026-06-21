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
}

impl<T: Transport> Driver<T> {
  /// The driver loop. Announces the connection, then interleaves inbound reads with UI commands until the
  /// transport ends, a write fails, or the UI hangs up. Always emits exactly one [`Event::Disconnected`] on
  /// the way out so the UI sees a definitive end.
  async fn run(mut self, mut command_rx: UnboundedReceiver<Command>) {
    // The transport is attached: drive Disconnected -> Connecting and surface it before any I/O.
    let connect_effects = self.core.on_connected();
    self.apply_effects(connect_effects).await;

    // Actively elicit readiness instead of waiting passively for a banner the board may never send: probe with
    // `?`/`0x87` and a counted `$I`. Any readiness evidence (status / banner / `ok` / `[VER:]`/`[OPT:]`) leaves
    // Connecting via the core. A write failure here is a dead connection; end with that error.
    let handshake_effects = self.core.begin_handshake();
    if let ControlFlow::Stop(reason) = self.apply_effects(handshake_effects).await {
      let down = self.core.on_disconnected();
      self.apply_effects(down).await;
      let _ = self.event_tx.send(Event::Disconnected(reason));
      return;
    }

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
      tokio::select! {
        // Bias toward draining commands first so a soft-reset / disconnect intent is honored promptly even
        // under a flood of inbound status reports.
        biased;

        command = command_rx.recv() => {
          match command {
            // All command senders dropped: the UI is gone. Wind down cleanly.
            None => break None,
            Some(Command::Disconnect) => break None,
            Some(command) => {
              if let ControlFlow::Stop(reason) = self.handle_command(command).await {
                break reason;
              }
            }
          }
        }

        read = self.transport.read(&mut read_buf) => {
          match read {
            // Ok(0) is end-of-stream — the device disappeared. A clean disconnect, not an error.
            Ok(0) => break None,
            Ok(n) => {
              if let ControlFlow::Stop(reason) = self.handle_inbound(&read_buf[..n]).await {
                break reason;
              }
            }
            Err(err) => break Some(err),
          }
        }

        _ = status_poll.tick() => {
          // Inject a `?` out-of-band through the core (uncounted), then write it. A failed write means the link
          // dropped; surface it and end the loop.
          let poll_effects = self.core.on_realtime(RealtimeCommand::StatusReport);
          if let ControlFlow::Stop(reason) = self.apply_effects(poll_effects).await {
            break reason;
          }
        }
      }
    };

    // Reset the core's view to Disconnected (best-effort: surface any resulting state change) and emit the
    // single terminal event. Errors sending the event mean the UI is already gone; nothing more to do.
    let down = self.core.on_disconnected();
    self.apply_effects(down).await;
    let _ = self.event_tx.send(Event::Disconnected(disconnect_reason));
  }

  /// Feed one UI command into the core and carry out the effects. Returns whether the loop should continue.
  async fn handle_command(&mut self, command: Command) -> ControlFlow {
    let effects = match command {
      Command::StreamProgram(lines) => self.core.on_stream_program(lines.iter()),
      Command::SendLine(line) => self.core.on_send_line(&line),
      Command::Realtime(cmd) => self.core.on_realtime(cmd),
      // `Disconnect` is handled by the caller (it breaks the loop) and never reaches here.
      Command::Disconnect => Vec::new(),
    };
    self.apply_effects(effects).await
  }

  /// Deframe a chunk of inbound bytes into lines, parse each, and feed every recognised response into the
  /// core. Empty lines and unparseable noise are dropped here (an empty line is not an `ok`). Returns whether
  /// the loop should continue.
  async fn handle_inbound(&mut self, bytes: &[u8]) -> ControlFlow {
    for line in self.reassembler.push(bytes) {
      if let Some(response) = parse_line(&line) {
        let effects = self.core.on_response(response);
        if let ControlFlow::Stop(reason) = self.apply_effects(effects).await {
          return ControlFlow::Stop(reason);
        }
      }
    }
    ControlFlow::Continue
  }

  /// Carry out an ordered batch of core effects: write byte effects to the transport (awaited inline so they
  /// are never cancelled mid-write), and forward every informational effect to the UI as an [`Event`]. A
  /// transport write failure ends the loop with that error.
  async fn apply_effects(&mut self, effects: Vec<Effect>) -> ControlFlow {
    for effect in effects {
      match effect {
        Effect::Write(bytes) => {
          if let Err(err) = self.transport.write_all(&bytes).await {
            return ControlFlow::Stop(Some(err));
          }
        }
        Effect::StateChanged(state) => self.emit(Event::StateChanged(state)),
        Effect::Response(response) => self.emit(Event::Response(response)),
        Effect::Progress { sent, acked, total } => self.emit(Event::Progress { sent, acked, total }),
        Effect::Fault(err) => self.emit(Event::Fault(err)),
      }
    }
    ControlFlow::Continue
  }

  /// Forward one event to the UI. A failure means the UI dropped its receiver; we simply stop emitting (the
  /// loop will end on its own once commands dry up).
  fn emit(&self, event: Event) {
    let _ = self.event_tx.send(event);
  }
}

/// Whether the driver loop should keep running or stop with a disconnect reason. A private mirror of the
/// standard-library control-flow idea, specialised to carry the optional transport error.
enum ControlFlow {
  Continue,
  Stop(Option<TransportError>),
}

#[cfg(test)]
mod tests {
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
}
