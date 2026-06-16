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
//! - [`comms_consumer`] is a Stage-1 STUB standing in for the gcode_parser → planner pipeline. It drains
//!   accepted lines, answers the `$` system queries (`$$`/`$I`/`$I+`/`$G`/`$#`) via the firmware-core
//!   formatters, and acknowledges every other line with a single `ok`. It performs no motion. Wiring the
//!   real parser/planner/motion pipeline replaces this task (see the TODOs in `main.rs`).
//! - [`status_responder`] formats a `<...>` report from the shared [`MachineSnapshot`] when the
//!   [`STATUS_REQUEST`] Signal fires.
//!
//! Real-time bytes are intercepted in [`usb_rx`] before line assembly and never receive an `ok`, exactly
//! matching the firmware-core contract.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embedded_io_async::{Read, Write};
use esp_hal::usb_serial_jtag::{UsbSerialJtagRx, UsbSerialJtagTx};
use esp_hal::Async;

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
          // A blank line: acknowledge directly with a bare `ok`, no parser round-trip.
          let mut s = Response::new();
          if ResponseWriter::ok(&mut s).is_ok() {
            enqueue(s).await;
          }
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

/// Stage-1 STUB consumer standing in for the gcode_parser → planner → motion pipeline. It drains accepted
/// lines, answers the `$` system queries with firmware-core formatters, and acknowledges every other line
/// with a single `ok` so the streaming contract (one `ok` per line) is exercisable end-to-end without the
/// full machine. Replacing this task with the real parser/planner is the next subsystem.
#[embassy_executor::task]
pub async fn comms_consumer() -> ! {
  loop {
    let line = LINE_QUEUE.receive().await;
    handle_line(line.as_slice()).await;
  }
}

/// Route one accepted line: `$` system commands get their report responses; everything else is a GCode
/// line the stub simply acknowledges. The single `ok` for a line is emitted exactly here, preserving the
/// one-ok-per-line contract end to end.
async fn handle_line(line: &[u8]) {
  let trimmed = trim_ascii(line);
  if let Some(rest) = trimmed.strip_prefix(b"$") {
    handle_system_command(rest).await;
  } else {
    // A GCode line. The stub does no parsing/motion; it acknowledges so a host stream advances. The real
    // parser will instead emit `ok` on success or `error:N` (and call `engine.note_line_error()` on the
    // RX side via a feedback path) on failure.
    ack().await;
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
