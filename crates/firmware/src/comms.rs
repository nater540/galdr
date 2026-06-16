//! USB CDC streaming bring-up (DOC-08 / DOC-01): the esp-hal wiring that drives the sans-io
//! [`firmware_core::protocol`] state machine over the ESP32-S3 USB Serial/JTAG controller.
//!
//! This module is the thin, non-host-testable adapter the task brief calls for: ALL streaming logic
//! (line framing, real-time classification, error-hold, response formatting) lives in firmware-core
//! and is exercised by host tests; here we only move bytes between the USB peripheral and that state
//! machine, dispatch real-time commands through `embassy-sync` Signals, and emit responses.
//!
//! ## Task topology (core 0, thread-mode executor — DOC-01)
//! - [`usb_rx`] reads USB bytes, feeds each into a [`StreamEngine`], dispatches real-time commands via
//!   Signals, forwards accepted lines to the parser [`Channel`], and emits `error:N` for protocol-level
//!   rejections.
//! - [`usb_tx`] is the single writer to the USB peripheral: it drains the [`RESPONSE`] channel so no two
//!   tasks ever write the USB endpoint concurrently (DOC-08).
//! - [`comms_consumer`] is the real gcode parser → planner pipeline. It parses each accepted line through a
//!   persistent [`Parser`], feeds the resulting command to a persistent [`Planner`], answers the `$` system
//!   queries (`$$`/`$I`/`$I+`/`$G`/`$#`), and emits exactly one `ok`/`error:N` per line. It owns the
//!   grblHAL gcode error-hold and back-pressures the host when the planner buffer is full.
//! - [`block_drain_stub`] is a placeholder for the DOC-02 `motion_executor`: it pops planner blocks paced
//!   by an estimated execution time and publishes the drained position into [`MACHINE`], so a streamed file
//!   makes progress instead of deadlocking once the planner buffer fills. It generates no step pulses.
//! - [`status_responder`] formats a `<...>` report from the shared [`MachineSnapshot`] when the
//!   [`STATUS_REQUEST`] Signal fires.
//!
//! Real-time bytes are intercepted in [`usb_rx`] before line assembly and never receive an `ok`, exactly
//! matching the firmware-core contract.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embassy_futures::select::{select, Either};
use embedded_io_async::{Read, Write};
use esp_hal::usb_serial_jtag::{UsbSerialJtagRx, UsbSerialJtagTx};
use esp_hal::Async;

use firmware_core::gcode::Parser;
use firmware_core::planner::{Block, Planner, PlannerConfig, PlannerError, PlannerOutcome, BLOCK_QUEUE_LEN};
use firmware_core::protocol::{
  EngineEvent, MachineSnapshot, RealtimeCommand, ResponseWriter, StreamEngine, MAX_LINE_LEN,
  RESPONSE_CAPACITY,
};

/// A single assembled input line handed from `usb_rx` to the parser stub, capped to the protocol line
/// length. Owned (not borrowed) so it can cross the channel without referencing the RX task's buffer.
pub type Line = heapless::Vec<u8, MAX_LINE_LEN>;

/// A fully-formatted outgoing response (banner / `ok` / `error:N` / status / `$`-report line), rendered
/// by a firmware-core formatter and queued for the single USB writer. Sized to hold any one Stage-1
/// response line; multi-line `$` responses are queued as several `Response`s in order.
pub type Response = heapless::String<RESPONSE_CAPACITY>;

/// Depth of the accepted-line channel from `usb_rx` to the parser stub. DOC-01 specifies 4; with simple
/// send-response streaming one in-flight line is the norm, so 4 is comfortable headroom.
pub const LINE_QUEUE_DEPTH: usize = 4;

/// Depth of the outgoing response channel to `usb_tx`. DOC-01 specifies 8; a `$I+`/`$$` burst queues
/// several lines at once, so 8 keeps multi-line replies from blocking the producer.
pub const RESPONSE_QUEUE_DEPTH: usize = 8;

/// Accepted GCode/`$` lines awaiting the parser stub.
pub static LINE_QUEUE: Channel<CriticalSectionRawMutex, Line, LINE_QUEUE_DEPTH> = Channel::new();

/// Outgoing responses awaiting the single USB writer. Every task that needs to emit bytes enqueues here
/// so the USB endpoint has exactly one writer (DOC-08).
pub static RESPONSE: Channel<CriticalSectionRawMutex, Response, RESPONSE_QUEUE_DEPTH> = Channel::new();

/// `?` (status report request) — set by `usb_rx`, consumed by `status_responder`.
pub static STATUS_REQUEST: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// `!` (feed hold) — set by `usb_rx`. Stage-1 consumer is a documented stub (no motion yet).
pub static FEED_HOLD: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// `~` (cycle start / resume) — set by `usb_rx`. Stage-1 consumer is a documented stub.
pub static CYCLE_START: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// `0x18` (soft reset) — set by `usb_rx`. The reset handler re-emits the banner and (later) flushes the
/// parser/planner/motion queues; Stage 1 re-emits the banner and clears the line queue.
pub static SOFT_RESET: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// The live machine state the status formatter reads. The motion executor will publish position/state
/// here once wired; Stage 1 holds the idle default so `?` returns a well-formed report immediately.
pub static MACHINE: Mutex<CriticalSectionRawMutex, MachineSnapshot> = Mutex::new(MachineSnapshot::idle());

/// The shared motion planner: the `comms_consumer` task enqueues blocks into it, and the stub
/// block-drain task (standing in for the real DOC-02 `motion_executor`) pops them. It lives behind a
/// `Mutex` because two tasks touch it; both critical sections are short (enqueue one command / pop one
/// block + read position), so contention is negligible. The real motion executor will replace the drain
/// task and keep this same handoff shape (planner is the producer, executor the single consumer).
///
/// Initialized lazily to `None` because [`Planner::new`] is not `const`; [`init_planner`] installs the
/// constructed planner once at boot before either task runs. After init the `Option` is always `Some`.
pub static PLANNER: Mutex<CriticalSectionRawMutex, Option<Planner>> = Mutex::new(None);

/// Build the placeholder [`PlannerConfig`]. These are grbl-like defaults standing in for the real
/// `$`-settings the firmware will load from esp-storage (DOC-00) once persistence is wired: 250 steps/mm,
/// 500 mm/min max rate, 10 mm/s² accel, `$11`=0.01 mm, `$12`=0.002 mm per axis. TODO(DOC-00): replace
/// with the persisted settings load (CRC-checked, falling back to these compiled defaults).
fn placeholder_planner_config() -> PlannerConfig {
  PlannerConfig::default()
}

/// Install the planner into [`PLANNER`] at boot, before the consumer/drain tasks are spawned. Called
/// once from `main`; using `try_lock` avoids an await in init and cannot contend (no task runs yet).
pub fn init_planner() {
  // `try_lock` succeeds because this runs before any task is spawned, so nothing else holds the lock.
  // The `Ok` arm is the only reachable path at init; a failure would be a wiring bug, handled by leaving
  // the planner uninitialized (the consumer then treats every line as an internal error rather than
  // panicking), but in practice this never fails.
  if let Ok(mut guard) = PLANNER.try_lock() {
    *guard = Some(Planner::new(placeholder_planner_config()));
  }
}

/// Enqueue a response for the USB writer, dropping it if the queue is momentarily full. Dropping a
/// response rather than blocking the RX path keeps real-time latency bounded; under simple send-response
/// streaming the queue is effectively never full. A dropped `ok` would stall a host, so the queue depth
/// is sized so this does not happen in normal operation — this is a safety valve, not a routine path.
async fn enqueue(resp: Response) {
  // `try_send` never blocks; on a full queue we drop. `send().await` is avoided so a stuck writer can
  // never back-pressure the byte scanner and delay real-time command dispatch.
  let _ = RESPONSE.try_send(resp);
}

/// Render a banner into a fresh [`Response`] and queue it. Emitted on boot and on every soft reset so a
/// host detects controller readiness (DOC-08 / the native-USB no-hard-reset rule).
pub async fn send_banner() {
  let mut s = Response::new();
  if ResponseWriter::banner(&mut s).is_ok() {
    enqueue(s).await;
  }
}

/// The USB receive task: scan every incoming byte through the [`StreamEngine`], dispatch real-time
/// commands via Signals, forward accepted lines, and emit `error:N` for protocol-level rejections. This
/// is the only place real-time bytes are intercepted; they never enter a line and never get an `ok`.
#[embassy_executor::task]
pub async fn usb_rx(mut rx: UsbSerialJtagRx<'static, Async>) -> ! {
  let mut engine = StreamEngine::new();
  // A modest read chunk: the USB Serial/JTAG FIFO is 64 bytes, so reads return promptly and we scan the
  // returned bytes one at a time through the state machine.
  let mut buf = [0u8; 64];
  loop {
    let n = match rx.read(&mut buf).await {
      Ok(n) => n,
      // A transient USB read error: yield and retry. The connection persists across host reopen on
      // native USB, so we do not tear down state here.
      Err(_) => continue,
    };
    for &byte in &buf[..n] {
      match engine.ingest(byte) {
        EngineEvent::None => {}
        EngineEvent::Realtime(cmd) => dispatch_realtime(cmd, &mut engine).await,
        EngineEvent::AcceptLine(line) => {
          // Copy the borrowed line into an owned buffer for the channel. A line longer than the buffer
          // cannot occur: the engine already enforced MAX_LINE_LEN, so this never truncates.
          let mut owned = Line::new();
          let _ = owned.extend_from_slice(line);
          // Block here if the parser stub is briefly behind: this back-pressures the host stream (which
          // is correct flow control) without dropping a line. Real-time bytes already bypassed this.
          LINE_QUEUE.send(owned).await;
        }
        EngineEvent::Acknowledge => {
          // A blank line: the grblHAL recovery trigger that clears a gcode error-hold. Forward it through
          // LINE_QUEUE (empty) rather than acking here, so the single in-order consumer owns both the bare
          // `ok` and the hold-clear, keeping recovery race-free with the lines queued around it. This also
          // back-pressures identically to a real line if the consumer is briefly behind.
          LINE_QUEUE.send(Line::new()).await;
        }
        EngineEvent::Reject(code) => {
          let mut s = Response::new();
          if ResponseWriter::error(&mut s, code).is_ok() {
            enqueue(s).await;
          }
        }
      }
    }
  }
}

/// Map a classified real-time command onto its Signal / immediate action. Status requests fire the
/// reporter Signal; feed-hold / cycle-start / reset fire their Signals; the reset path also re-emits the
/// banner and clears the line queue so a host sees readiness and no stale line survives. Override and
/// the Stage-2/3 commands are accepted and currently ignored (documented stubs).
async fn dispatch_realtime(cmd: RealtimeCommand, engine: &mut StreamEngine) {
  match cmd {
    RealtimeCommand::StatusReport | RealtimeCommand::FullStatusReport => STATUS_REQUEST.signal(()),
    RealtimeCommand::FeedHold => FEED_HOLD.signal(()),
    RealtimeCommand::CycleStart => CYCLE_START.signal(()),
    RealtimeCommand::SoftReset | RealtimeCommand::Stop => {
      // The engine already dropped any partial line and lifted the error-hold on `0x18`; for `0x19`
      // (Stop) we mirror the line-buffer reset so a fresh stream starts clean.
      engine.soft_reset();
      SOFT_RESET.signal(());
      // Flush any not-yet-consumed accepted lines so post-reset modal state is not contaminated.
      while LINE_QUEUE.try_receive().is_ok() {}
      send_banner().await;
    }
    // Parser-state-on-demand, safety door, jog cancel, auto-report toggle, and overrides are accepted but
    // not yet acted on (Stage 2/3). They correctly produce no `ok`.
    RealtimeCommand::ParserStateReport
    | RealtimeCommand::SafetyDoor
    | RealtimeCommand::JogCancel
    | RealtimeCommand::ToggleAutoReport
    | RealtimeCommand::Override(_) => {}
  }
}

/// The single USB writer: drain the [`RESPONSE`] channel and write each formatted response to the USB
/// endpoint. Centralizing writes here means status reports, `ok`s, errors, and `$`-report lines never
/// interleave on the wire (DOC-08).
#[embassy_executor::task]
pub async fn usb_tx(mut tx: UsbSerialJtagTx<'static, Async>) -> ! {
  loop {
    let resp = RESPONSE.receive().await;
    // A write error on native USB means the host closed the port; drop the byte and continue, since the
    // connection re-establishes on host reopen without a controller reset.
    let _ = tx.write_all(resp.as_bytes()).await;
    let _ = tx.flush().await;
  }
}

/// The real parser → planner consumer (replaces the Stage-1 stub). It is the SINGLE, in-order consumer of
/// [`LINE_QUEUE`], so it is the natural owner of the grblHAL gcode error-hold (see [`ConsumerState`]). Per
/// line it: routes `$` system commands to their handlers; parses GCode through a persistent [`Parser`];
/// and feeds the resulting [`PlannerCommand`](firmware_core::gcode::PlannerCommand) to a persistent
/// [`Planner`]. It emits exactly one `ok`/`error:N` per consumed line, preserving the one-response-per-line
/// contract end to end (`usb_rx` only ever responds for protocol-level rejects it handles itself, so there
/// is no double-response or gap at the boundary).
///
/// ## Error-hold ownership (race-free by construction)
/// The grblHAL contract holds all subsequent lines in an error state after a GCode line errors, until a
/// reset / empty line / `$` command. That hold lives HERE, in [`ConsumerState::error_hold`], not in the
/// `StreamEngine`: the engine runs in `usb_rx` and forwards lines asynchronously, so it cannot know a line
/// errored downstream, and any back-channel from here to `usb_rx` would race the lines already in flight in
/// `LINE_QUEUE`. Because this task is the only reader of `LINE_QUEUE` and sees parse/plan results strictly
/// in queue order, owning the hold here is inherently in-order and race-free. The engine's
/// `note_line_error()` mechanism is therefore deliberately NOT driven on this path (it is reserved/unused);
/// the engine still owns the independent protocol-level overflow rejects, which is correct.
///
/// ## Back-pressure (no motion executor yet)
/// `ok` for a move is emitted only once the block is ACCEPTED into the planner buffer. When the planner is
/// full ([`PlannerError::QueueFull`]) the consumer neither acks nor drops the line: it waits for the stub
/// drain task to free a block and retries the SAME command (the arc planner is all-or-nothing on
/// `QueueFull`, so re-issuing is safe). While waiting it stops reading `LINE_QUEUE`, which backs up,
/// blocks `usb_rx`'s `send().await`, stops the byte scanner, and lets the host's character-counting throttle
/// — exactly the correct grbl flow control.
#[embassy_executor::task]
pub async fn comms_consumer() -> ! {
  let mut parser = Parser::new();
  let mut state = ConsumerState::default();
  loop {
    // Race a soft reset against the next line so a `0x18` arriving mid-stream resets the parser modal
    // state, clears the error-hold, and flushes the planner queue BEFORE the next line is parsed. `usb_rx`
    // already flushed `LINE_QUEUE` and re-emitted the banner when it fired the signal; here we reset the
    // pipeline state this task owns. A line that lost the race is dropped (it predates the reset), matching
    // grbl's warm-reset semantics.
    match select(LINE_QUEUE.receive(), SOFT_RESET.wait()).await {
      Either::First(line) => handle_line(line.as_slice(), &mut parser, &mut state).await,
      Either::Second(()) => reset_pipeline(&mut parser, &mut state).await,
    }
  }
}

/// Reset the parser/planner pipeline state this task owns on a soft reset (`0x18`): restore the parser to
/// default modal state, clear the gcode error-hold, and flush the planner queue and machine position. The
/// `StreamEngine` line buffer and `LINE_QUEUE` were already cleared by `usb_rx`; this completes the warm
/// reset for the downstream half so a fresh stream starts from defaults at the origin.
async fn reset_pipeline(parser: &mut Parser, state: &mut ConsumerState) {
  // `Parser` exposes no in-place reset; reconstructing it restores the documented power-on modal defaults
  // (G0, G90, G21, F0, S0) — exactly grbl's warm-reset modal state.
  *parser = Parser::new();
  state.error_hold = false;
  // Reconstruct the planner to clear the block queue, machine position, work offset, and junction state in
  // one step (it has no public flush). The placeholder config is re-applied; once esp-storage settings are
  // wired the reset will re-load them. Reset the published snapshot to idle so `?` reports the origin.
  {
    let mut guard = PLANNER.lock().await;
    *guard = Some(Planner::new(placeholder_planner_config()));
  }
  let mut snap = MACHINE.lock().await;
  *snap = MachineSnapshot::idle();
}

/// The consumer's persistent control state across lines: the gcode error-hold flag. Held locally in the
/// single consumer task so it is mutated only in line order.
#[derive(Default)]
struct ConsumerState {
  /// True once a GCode line errored and no recovery trigger (blank line / `$` command / soft reset) has
  /// cleared it yet. While set, GCode lines are rejected without parsing, per grblHAL safety behavior.
  error_hold: bool,
}

/// Route one accepted line. `$` system commands are dispatched to their report handlers and clear the
/// error-hold (a recovery trigger). Otherwise the line is parsed and planned. Exactly one `ok`/`error:N`
/// is emitted per call, preserving the one-response-per-line contract.
async fn handle_line(line: &[u8], parser: &mut Parser, state: &mut ConsumerState) {
  let trimmed = trim_ascii(line);
  if trimmed.is_empty() {
    // A blank line (forwarded by `usb_rx`) is a grblHAL error-hold recovery trigger: clear the hold and
    // acknowledge with a bare `ok`. Handling it here, in queue order, keeps recovery race-free with the
    // surrounding lines, since the hold lives in this task, the sole in-order reader of `LINE_QUEUE`.
    state.error_hold = false;
    ack().await;
  } else if let Some(rest) = trimmed.strip_prefix(b"$") {
    // A `$` system command is the other grblHAL recovery trigger and is answered by its handler. Both
    // recovery triggers (an empty line and a `$` command) and a soft reset clear the hold; nothing else.
    state.error_hold = false;
    handle_system_command(rest).await;
  } else {
    plan_gcode_line(trimmed, parser, state).await;
  }
}

/// Parse and plan one GCode line, emitting exactly one `ok`/`error:N`. Honors the error-hold: while held,
/// a GCode line is rejected without parsing. On a parse or planner error the hold is armed; on acceptance
/// (including modal-only `Ok(None)` lines) a single `ok` is emitted.
async fn plan_gcode_line(line: &[u8], parser: &mut Parser, state: &mut ConsumerState) {
  if state.error_hold {
    // Held by a prior error: reject without parsing until a recovery trigger. Reuse the generic
    // "expected command letter" code, matching how a sender already in error-recovery treats any further
    // rejection — it halts the stream regardless of the specific code (mirrors the engine's hold code).
    error(ERROR_HOLD_CODE).await;
    return;
  }
  match parser.parse_line(line) {
    // A blank/comment-only/modal-only line carries no action; acknowledge with a single `ok`.
    Ok(None) => ack().await,
    Ok(Some(command)) => match plan_command(&command).await {
      // The command was accepted into the planner (a move enqueued, or a non-motion outcome passed
      // through); emit the single `ok`.
      PlanResult::Accepted => ack().await,
      // The planner reported a non-back-pressure error (bad arc geometry); reject and arm the hold.
      PlanResult::Error(code) => {
        error(code).await;
        state.error_hold = true;
      }
      // A soft reset arrived while this command was back-pressured: abort it (the host discards pending
      // acks on `0x18`), emit no response, and run the pipeline reset whose signal was consumed here.
      PlanResult::Aborted => reset_pipeline(parser, state).await,
    },
    Err(e) => {
      // A parse error: emit `error:N` and arm the gcode error-hold so subsequent GCode lines are held.
      error(e.code()).await;
      state.error_hold = true;
    }
  }
}

/// The result of attempting to plan one command (collapsing the planner's back-pressure retry loop and the
/// soft-reset abort into one outcome the line handler acts on).
enum PlanResult {
  /// The command was accepted into the planner buffer (move enqueued or non-motion outcome passed through).
  Accepted,
  /// A non-back-pressure planner error; carries the grblHAL `error:N` code (bad arc geometry).
  Error(u8),
  /// A soft reset preempted the command while it was back-pressured; the consumed signal must be honored.
  Aborted,
}

/// The `error:N` code used to reject a GCode line that is held in the post-error state. Matches the
/// engine's `ERROR_HOLD_CODE` (grbl's generic "expected command letter" code 1): a sender already in
/// error-recovery halts the stream regardless of the specific code.
const ERROR_HOLD_CODE: u8 = 1;

/// Feed one [`PlannerCommand`](firmware_core::gcode::PlannerCommand) to the shared planner, applying
/// back-pressure: on [`PlannerError::QueueFull`] wait for the stub drain to free a block and retry the
/// SAME command (the arc planner is all-or-nothing on `QueueFull`, so re-issue is safe). The back-pressure
/// wait is raced against [`SOFT_RESET`] so a `0x18` aborts a stuck line promptly rather than after the
/// drain frees a slot. Non-motion outcomes (Dwell/Spindle/G28/G92/M30) currently pass through with no side
/// effect; the real dwell timer, spindle driver (DOC-07), and homing (DOC-06) consume these in later phases.
async fn plan_command(command: &firmware_core::gcode::PlannerCommand) -> PlanResult {
  loop {
    // Scope the lock so it is released before any await: hold the planner mutex only for the plan call.
    let result = {
      let mut guard = PLANNER.lock().await;
      match guard.as_mut() {
        Some(planner) => planner.plan_command(command),
        // The planner was never installed (an init wiring bug). Surface as a generic error rather than
        // panicking; this is unreachable in a correctly wired build (see `init_planner`).
        None => Ok(PlannerOutcome::Queued { blocks: 0 }),
      }
    };
    match result {
      // TODO(DOC-02/DOC-07/DOC-06): act on non-motion outcomes — start the dwell timer, drive the spindle,
      // run the predefined/homing move, reset program state on M30. Stage 1 accepts and passes through.
      Ok(_outcome) => return PlanResult::Accepted,
      // Back-pressure: the planner buffer is full. Do NOT ack and do NOT drop — yield to the drain task,
      // then retry the same command. Blocking here backs `LINE_QUEUE` up and throttles the host (correct
      // grbl flow control). Race the retry delay against a soft reset so `0x18` aborts a stuck line at once;
      // the delay is short relative to a block's execution time, so a normal retry wins the freed slot
      // promptly without busy-spinning the CPU.
      Err(PlannerError::QueueFull) => {
        match select(Timer::after(QUEUE_FULL_RETRY), SOFT_RESET.wait()).await {
          Either::First(()) => {}
          Either::Second(()) => return PlanResult::Aborted,
        }
      }
      // A genuine geometry error (bad arc): surface the grblHAL code to the caller.
      Err(other) => return PlanResult::Error(other.code()),
    }
  }
}

/// Answer a `$<rest>` system command. Stage 1 implements the connect-critical subset: `$$` settings dump
/// (stub: header + `ok`), `$I`/`$I+` build info, `$G` parser state, `$#` NGC parameters (stub). Each
/// multi-line response ends with `ok`. An unrecognized `$` command is acknowledged so the stream advances.
async fn handle_system_command(rest: &[u8]) {
  match rest {
    b"" | b"$" => {
      // `$` (help) / `$$` dump. Stage 1 returns an empty dump plus `ok`; real settings land with
      // esp-storage persistence (TODO). This keeps `$$` from stalling a sender that probes settings.
      ack().await;
    }
    b"I" => {
      send_build_info(false).await;
      ack().await;
    }
    b"I+" => {
      send_build_info(true).await;
      ack().await;
    }
    b"G" => {
      send_parser_state().await;
      ack().await;
    }
    b"#" => {
      // NGC parameters dump. Stage 1 has no stored offsets; acknowledge so `$#` does not stall.
      ack().await;
    }
    _ => ack().await,
  }
}

/// Queue the `$I`/`$I+` build-info lines.
async fn send_build_info(extended: bool) {
  let mut s = Response::new();
  if ResponseWriter::build_info(&mut s, extended).is_ok() {
    enqueue(s).await;
  }
}

/// Queue the `$G` parser-state line.
async fn send_parser_state() {
  let mut s = Response::new();
  if ResponseWriter::parser_state(&mut s).is_ok() {
    enqueue(s).await;
  }
}

/// Queue a single `ok`.
async fn ack() {
  let mut s = Response::new();
  if ResponseWriter::ok(&mut s).is_ok() {
    enqueue(s).await;
  }
}

/// Queue a single `error:N` for a rejected line. Mirrors [`ack`]; the consumer emits exactly one of the
/// two per consumed GCode line, preserving the one-response-per-line contract.
async fn error(code: u8) {
  let mut s = Response::new();
  if ResponseWriter::error(&mut s, code).is_ok() {
    enqueue(s).await;
  }
}

/// The status reporter: format a `<...>` report from the shared [`MachineSnapshot`] whenever the
/// [`STATUS_REQUEST`] Signal fires (set by `usb_rx` on `?`/`0x80`/`0x87`). Stage 1 reports the shared
/// snapshot (idle/origin until the motion executor publishes live data).
#[embassy_executor::task]
pub async fn status_responder() -> ! {
  loop {
    STATUS_REQUEST.wait().await;
    let snap = *MACHINE.lock().await;
    let mut s = Response::new();
    if ResponseWriter::status_report(&mut s, &snap).is_ok() {
      enqueue(s).await;
    }
  }
}

/// How long the consumer waits before retrying a [`PlannerError::QueueFull`] command. Short relative to a
/// block's execution time (tens of ms) so the retry claims a freed slot promptly, but long enough that the
/// retry loop is not a busy-spin — it yields the executor to the drain task each iteration.
const QUEUE_FULL_RETRY: Duration = Duration::from_millis(2);

/// Lower bound on the simulated execution time of one drained block, so a tiny/zero-length block does not
/// let the drain busy-loop and instantly empty the queue (which would mask back-pressure).
const MIN_BLOCK_DRAIN: Duration = Duration::from_millis(5);

/// Upper bound on the simulated execution time of one drained block, so a very long/slow move does not
/// stall the pipeline for an implausibly long time during bring-up testing.
const MAX_BLOCK_DRAIN: Duration = Duration::from_millis(500);

/// How long the drain task idles before re-checking an empty planner queue. Keeps the task off the CPU
/// while no motion is queued; a real motion executor would instead await the planner's `BLOCK_AVAILABLE`
/// signal (DOC-01) rather than poll.
const DRAIN_IDLE_POLL: Duration = Duration::from_millis(20);

/// STUB block-drain task: a placeholder for the real DOC-02 `motion_executor` + RMT step generation. It
/// pops one planner block at a time, paces itself by that block's approximate execution time, and publishes
/// the drained position into [`MACHINE`] so `?` reports a plausible MPos. It exists solely so an end-to-end
/// stream makes progress instead of deadlocking once the 32-block planner buffer fills; it generates NO
/// step pulses and models NO trapezoidal profile.
///
/// TODO(DOC-02): replace this task with the core-1 `InterruptExecutor` running the real `motion_executor`,
/// which runs the [`SegmentGenerator`](firmware_core::motion) over each popped block, emits RMT PulseCodes
/// on ch0/1/2, and publishes live interpolated MPos. This task and its pacing heuristics are then deleted.
#[embassy_executor::task]
pub async fn block_drain_stub() -> ! {
  loop {
    // Pop the next block under the planner lock, releasing it before the pacing delay so the consumer can
    // enqueue while this block "executes". `pop_block` is FIFO, matching the real executor's consumption.
    let popped = {
      let mut guard = PLANNER.lock().await;
      guard.as_mut().and_then(Planner::pop_block)
    };
    match popped {
      Some(block) => {
        publish_drained_position().await;
        // Pace by the block's approximate execution time (cruise-only: travel / nominal speed), clamped to
        // a sane window. This keeps the queue partly full under sustained streaming so back-pressure is
        // actually exercised, rather than draining instantly. The real executor's timing comes from the
        // segment generator's tick periods, not this estimate.
        Timer::after(simulated_block_duration(&block)).await;
      }
      // Queue empty: idle briefly, then re-check. The real executor awaits BLOCK_AVAILABLE instead.
      None => Timer::after(DRAIN_IDLE_POLL).await,
    }
  }
}

/// Estimate one block's execution time for the stub drain pacing: straight-line travel divided by the
/// block's nominal (cruise) speed, clamped to `[MIN_BLOCK_DRAIN, MAX_BLOCK_DRAIN]`. This is a deliberately
/// crude placeholder (it ignores accel/decel ramps and rest-to-rest boundaries) — only the real segment
/// generator computes true timing.
fn simulated_block_duration(block: &Block) -> Duration {
  let speed = block.nominal_speed();
  // Require strictly positive, finite speed AND travel; the `> 0.0` tests are false for NaN, zero, and
  // negatives, so a degenerate block falls through to the floor below rather than dividing badly.
  if speed > 0.0 && block.millimeters > 0.0 {
    let seconds = block.millimeters / speed;
    let millis = (seconds * 1000.0) as u64;
    return Duration::from_millis(millis).max(MIN_BLOCK_DRAIN).min(MAX_BLOCK_DRAIN);
  }
  // A degenerate block (zero speed/length): use the floor so the drain neither stalls nor busy-loops.
  MIN_BLOCK_DRAIN
}

/// Publish the planner's current position and free-block count into the shared [`MachineSnapshot`] so `?`
/// reports a plausible MPos and `Bf:` block-free count. The planner position is the end-of-look-ahead
/// (planned) position, not the live interpolated one; that is acceptable for the stub, and the real motion
/// executor will publish true live position instead.
async fn publish_drained_position() {
  let (pos, free) = {
    let guard = PLANNER.lock().await;
    match guard.as_ref() {
      Some(planner) => {
        let queued = planner.queued_len();
        let free = BLOCK_QUEUE_LEN.saturating_sub(queued) as u8;
        (planner.position_mm(), free)
      }
      None => return,
    }
  };
  let mut snap = MACHINE.lock().await;
  snap.mpos_mm = pos;
  snap.planner_blocks_free = free;
}

/// Trim leading/trailing ASCII whitespace from a line, since a sender may pad `$` commands with spaces.
/// `core` lacks a stable slice trim for `&[u8]`, so this is a small explicit helper.
fn trim_ascii(line: &[u8]) -> &[u8] {
  let mut start = 0;
  let mut end = line.len();
  while start < end && (line[start] == b' ' || line[start] == b'\t') {
    start += 1;
  }
  while end > start && (line[end - 1] == b' ' || line[end - 1] == b'\t') {
    end -= 1;
  }
  &line[start..end]
}
