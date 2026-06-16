//! grblHAL streaming protocol (DOC-08, `docs/gcode-streaming.md`).
//!
//! This module is the firmware-side, sans-io implementation of the grblHAL 1.1 streaming contract
//! shared with `skirnir`. It is a pure state machine: bytes go in, classified events come out, and a
//! set of formatters render protocol responses into caller-provided `heapless::String` buffers. It
//! performs no I/O, holds no async, and depends on no esp-hal — the `firmware` binary's `usb_rx` /
//! `usb_tx` tasks drive it, and the same logic is therefore host-testable with byte-level vectors.
//!
//! ## Responsibilities (Stage 1, "minimum viable grbl 1.1 streaming")
//! - **Line framing.** Accumulate printable bytes into a fixed line buffer; treat `CR`, `LF`, `CRLF`,
//!   and `LFCR` as a *single* terminator (no legacy double-`ok`); surface an over-length line as
//!   `error:15` rather than truncating silently.
//! - **Real-time byte interception.** Classify single-byte real-time commands (`?`/`!`/`~`/`0x18`, the
//!   grblHAL `0x80`–`0x8C` top-bit forms, `0x19` stop, override bytes) *before* the line buffer; they
//!   never enter a line and never receive an `ok`.
//! - **Flow-control contract.** The orchestrator yields exactly one accept/reject decision per consumed
//!   line so the driver can emit exactly one `ok`/`error:N` — the only signal driving host flow control.
//! - **Persistent error state.** After a GCode line errors, hold subsequent GCode lines in an error
//!   state until a soft reset, an empty line, or a `$` system command (grblHAL safety behavior).
//! - **Response formatting.** Banner, `ok`/`error:N`, `<...>` status report, `$I`/`$I+` build info,
//!   `$G` parser state, `$$` settings dump — all rendered into caller buffers.
//!
//! ## Out of scope here (Stage 2/3, left as clean extension points)
//! Alarm state machine, full status element set (`Pn:`/`Ov:`/`WCO:` refresh rules), runtime
//! enumerations (`$ES`/`$EE`/`$EA`), probing (`G38.x`/`[PRB:]`), and `$481` auto-report. The types
//! below reserve room for these (e.g. [`MachineState`] carries the states Stage 2 needs) without
//! implementing them yet.
//!
//! ## Driving contract (how the `firmware` bin uses this)
//! ```ignore
//! let mut engine = StreamEngine::new();
//! // For each received byte:
//! match engine.ingest(byte) {
//!   EngineEvent::None              => {}                       // mid-line, nothing to do yet.
//!   EngineEvent::Realtime(cmd)     => dispatch_signal(cmd),    // set the matching embassy Signal.
//!   EngineEvent::AcceptLine(line)  => forward_to_parser(line), // `ok` is emitted once consumed.
//!   EngineEvent::Reject(code)      => respond_error(code),     // emit `error:N` immediately.
//! }
//! ```

#![allow(clippy::result_unit_err)]

use core::fmt::Write as _;

use heapless::{String, Vec};

/// The advertised serial RX buffer size in bytes, reported in the `[OPT:...]` build-info line and as
/// the second field of the `Bf:` status element. 1024 is the grblHAL norm on 32-bit drivers and a good
/// fit on the ESP32-S3's ample RAM; a host sizes its character-counting send-ahead window to this value,
/// so it MUST match the real receive capacity the `firmware` bin provisions for the USB RX path.
pub const RX_BUFFER_SIZE: usize = 1024;

/// The advertised planner block-buffer depth, reported as the second field of `[OPT:...]` and the first
/// field of `Bf:`. Mirrors the planner's `BLOCK_QUEUE_LEN`; a host reads it to size look-ahead.
pub const BLOCK_BUFFER_SIZE: usize = 16;

/// The maximum length of a single assembled GCode line, in bytes, excluding the terminator. A line that
/// would exceed this is rejected with `error:15` (line length exceeded) rather than silently truncated,
/// matching the grbl-family overflow contract. 256 comfortably covers grblHAL line lengths while keeping
/// the per-connection buffer small and allocation-free.
pub const MAX_LINE_LEN: usize = 256;

/// The grblHAL `error:N` code for an over-length input line ("line length exceeded").
pub const ERROR_LINE_OVERFLOW: u8 = 15;

/// The number of motion axes reported in build info and status (`[AXS:3:XYZ]`, three `MPos` fields).
pub const AXIS_COUNT: usize = 3;

/// The firmware version string reported in the banner and the `[VER:]` build-info line. grblHAL reports
/// a grbl-1.1f-compatible version so senders compliant with grbl 1.1f recognize the controller.
pub const VERSION: &str = "1.1f";

/// A real-time command: a single byte intercepted out of the RX stream the instant it arrives, ahead of
/// the line buffer. It never enters a line and never receives an `ok`. Both the printable grbl-1.1 forms
/// and the grblHAL top-bit-set forms (advertised by `RT+` in `NEWOPT`) classify to the same variant so a
/// sender may use either. Stage-1 acts on the first four; the rest are carried so the driver can dispatch
/// or ignore them without re-scanning, and so Stage 2 can light them up without changing this surface.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RealtimeCommand {
  /// `?` / `0x80` — status report request. The driver answers with a `<...>` report immediately.
  StatusReport,
  /// `~` / `0x81` — cycle start / resume.
  CycleStart,
  /// `!` / `0x82` — feed hold.
  FeedHold,
  /// `0x18` (Ctrl-X) — soft reset / abort: halt motion, reset parser/planner, re-emit the banner.
  SoftReset,
  /// `0x19` (Ctrl-Y) — stop: like soft reset but leaves more internal state intact (no full warm reset).
  Stop,
  /// `0x83` — request the parser-state (`$G`) report on demand.
  ParserStateReport,
  /// `0x84` — safety door: suspend into the DOOR state, kill spindle/coolant.
  SafetyDoor,
  /// `0x85` — jog cancel: feed-hold plus a planner flush; ignored if not jogging.
  JogCancel,
  /// `0x87` — full real-time report (all change-only elements plus the alarm substate). Answered even in
  /// otherwise-locked states so a sender can detect an extended (grblHAL) controller on connect.
  FullStatusReport,
  /// `0x8C` — toggle the auto real-time report mode (`$481`). Carried for Stage 3; no Stage-1 action.
  ToggleAutoReport,
  /// A feed / rapid / spindle / coolant override byte (`0x90`–`0x9E`, `0xA0`–`0xA4`). The raw byte is
  /// preserved so the override handler can decode the specific adjustment without a second classify pass.
  Override(u8),
}

/// The machine run-state reported as the first field of a `<...>` status report. Stage 1 only ever
/// reports [`Idle`](MachineState::Idle), but the full grblHAL set is enumerated here so the formatter and
/// the shared [`MachineSnapshot`] are Stage-2-ready (alarm/hold/homing) without a breaking change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MachineState {
  /// No motion queued or executing.
  Idle,
  /// Executing a queued motion block.
  Run,
  /// Feed hold: `Hold:0` complete/ready-to-resume, `Hold:1` in progress. The `bool` is the substate.
  Hold(bool),
  /// Executing a jog.
  Jog,
  /// Halted in an alarm; the `u8` is the grblHAL alarm code (added to the full `0x87` report).
  Alarm(u8),
  /// Safety door open.
  Door,
  /// `$C` check mode: parse/validate without moving.
  Check,
  /// Running a homing cycle.
  Home,
}

impl MachineState {
  /// The grblHAL status-report state token, written as the first field of a `<...>` report. Substates
  /// (`Hold:0`/`Hold:1`, `Alarm:<code>`) are appended by the status formatter, not encoded here.
  fn token(self) -> &'static str {
    match self {
      MachineState::Idle => "Idle",
      MachineState::Run => "Run",
      MachineState::Hold(_) => "Hold",
      MachineState::Jog => "Jog",
      MachineState::Alarm(_) => "Alarm",
      MachineState::Door => "Door",
      MachineState::Check => "Check",
      MachineState::Home => "Home",
    }
  }
}

/// An immutable, `Copy` snapshot of the live machine state the status formatter renders. The `firmware`
/// bin fills this from shared atomics/cells (live MPos from the motion executor, buffer free-counts from
/// the planner queue and RX path) and hands it to [`ResponseWriter::status_report`]; keeping the
/// formatter pure over a snapshot is what makes status reporting host-testable.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MachineSnapshot {
  /// Current run-state (report's first field).
  pub state: MachineState,
  /// Machine position in mm, `[X, Y, Z]` (report's `MPos:` field).
  pub mpos_mm: [f32; AXIS_COUNT],
  /// Programmed feed rate in mm/min (`FS:` first field).
  pub feed_mm_min: f32,
  /// Programmed spindle speed in RPM (`FS:` second field).
  pub spindle_rpm: u16,
  /// Planner blocks free (`Bf:` first field).
  pub planner_blocks_free: u8,
  /// RX buffer bytes free (`Bf:` second field).
  pub rx_bytes_free: u16,
}

impl MachineSnapshot {
  /// A power-on default: idle at the origin with empty buffers fully free. Used as the initial shared
  /// state before the motion executor publishes a live position.
  pub const fn idle() -> Self {
    Self {
      state: MachineState::Idle,
      mpos_mm: [0.0; AXIS_COUNT],
      feed_mm_min: 0.0,
      spindle_rpm: 0,
      planner_blocks_free: BLOCK_BUFFER_SIZE as u8,
      rx_bytes_free: RX_BUFFER_SIZE as u16,
    }
  }
}

/// Classify a single byte as a real-time command, or `None` if it is ordinary line content. Both the
/// printable grbl forms and the grblHAL top-bit-set forms map to the same [`RealtimeCommand`]. The RX
/// scanner calls this on every byte and diverts any `Some` result before line assembly.
///
/// Note on `?`/`~`/`!`: grblHAL ignores the *printable* forms while reading `$`-command or message input
/// so those characters can appear in passwords/strings. Stage 1 does not implement that input-mode
/// suppression — these are always intercepted — which is the more conservative, sender-compatible
/// behavior; Stage 2 can gate the printable forms on an input-mode flag held by the orchestrator.
pub fn classify_realtime(byte: u8) -> Option<RealtimeCommand> {
  match byte {
    b'?' | 0x80 => Some(RealtimeCommand::StatusReport),
    b'~' | 0x81 => Some(RealtimeCommand::CycleStart),
    b'!' | 0x82 => Some(RealtimeCommand::FeedHold),
    0x18 => Some(RealtimeCommand::SoftReset),
    0x19 => Some(RealtimeCommand::Stop),
    0x83 => Some(RealtimeCommand::ParserStateReport),
    0x84 => Some(RealtimeCommand::SafetyDoor),
    0x85 => Some(RealtimeCommand::JogCancel),
    0x87 => Some(RealtimeCommand::FullStatusReport),
    0x8C => Some(RealtimeCommand::ToggleAutoReport),
    // Feed (0x90-0x94), rapid (0x95-0x97), spindle (0x99-0x9E), coolant (0xA0-0xA1), tool/probe
    // (0xA3-0xA4) overrides. Preserve the raw byte for the override decoder.
    0x90..=0x9E | 0xA0..=0xA4 => Some(RealtimeCommand::Override(byte)),
    _ => None,
  }
}

/// The outcome of feeding one byte to a [`LineReader`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LineEvent<'a> {
  /// The byte was buffered (or was a swallowed second half of a `CRLF`/`LFCR`); nothing to emit yet.
  Pending,
  /// A complete line is ready. The slice borrows the reader's buffer and is valid until the next
  /// [`LineReader::feed`]. An empty slice is a bare/empty line (a terminator with no content), which the
  /// orchestrator treats as the grblHAL error-state recovery trigger.
  Line(&'a [u8]),
  /// The current line exceeded [`MAX_LINE_LEN`]; the reader has entered an overflow state and will
  /// discard bytes until the next terminator. The orchestrator must respond `error:15`.
  Overflow,
}

/// The pending terminator-collapse state, so a `CRLF`/`LFCR` split across two `feed` calls still counts
/// as one terminator. After a terminator byte we record which one it was; if the very next byte is the
/// complementary terminator (`\n` after `\r`, or `\r` after `\n`), it is swallowed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PendingTerminator {
  /// No terminator was just seen.
  None,
  /// The previous byte was `\r`; a following `\n` is the second half of a `CRLF` and is swallowed.
  Cr,
  /// The previous byte was `\n`; a following `\r` is the second half of an `LFCR` and is swallowed.
  Lf,
}

/// Frames an incoming byte stream into lines. Accumulates printable bytes into a fixed buffer and emits a
/// [`LineEvent::Line`] on each terminator, collapsing `CRLF`/`LFCR` (including across `feed` calls) into a
/// single terminator so a host never sees a double-`ok`. Over-length lines yield [`LineEvent::Overflow`].
/// Real-time bytes are NOT handled here — they are diverted by the orchestrator before the reader sees a
/// byte — so the line buffer is never disturbed by an interleaved `?`/`!`.
#[derive(Debug)]
pub struct LineReader {
  buf: Vec<u8, MAX_LINE_LEN>,
  pending: PendingTerminator,
  overflowed: bool,
  /// Set when the previous byte completed a line. The completed line stays in `buf` so the caller can
  /// borrow it; the buffer is cleared lazily on the next content/terminator byte rather than eagerly, so
  /// the returned [`LineEvent::Line`] slice stays valid until the caller's next [`feed`](LineReader::feed).
  line_done: bool,
}

impl Default for LineReader {
  fn default() -> Self {
    Self::new()
  }
}

impl LineReader {
  /// Construct an empty line reader.
  pub const fn new() -> Self {
    Self {
      buf: Vec::new(),
      pending: PendingTerminator::None,
      overflowed: false,
      line_done: false,
    }
  }

  /// Discard any partially accumulated line and clear overflow/terminator state. Called on soft reset so
  /// a fresh stream is not contaminated by a half-buffered line from before the reset.
  pub fn reset(&mut self) {
    self.buf.clear();
    self.pending = PendingTerminator::None;
    self.overflowed = false;
    self.line_done = false;
  }

  /// Clear the previously completed line, if any, before assembling the next one. Called at the top of
  /// each `feed` that is not the swallowed half of a split terminator, so a completed line lives exactly
  /// from the `feed` that completed it until the next meaningful `feed`.
  fn rotate(&mut self) {
    if self.line_done {
      self.buf.clear();
      self.line_done = false;
    }
  }

  /// Feed one byte. Returns [`LineEvent::Line`] (borrowing the internal buffer) when a terminator
  /// completes a line, [`LineEvent::Overflow`] when the line is too long, or [`LineEvent::Pending`]
  /// otherwise. The returned `Line` slice is valid until the next call to `feed`.
  pub fn feed(&mut self, byte: u8) -> LineEvent<'_> {
    // Collapse the second half of a split CRLF/LFCR: a \n right after \r (or \r right after \n) is the
    // same single terminator and must be swallowed without emitting a second (empty) line. This case
    // does NOT rotate, so a line completed by the first half survives the swallowed second half.
    match (self.pending, byte) {
      (PendingTerminator::Cr, b'\n') | (PendingTerminator::Lf, b'\r') => {
        self.pending = PendingTerminator::None;
        return LineEvent::Pending;
      }
      _ => {}
    }

    // A new meaningful byte: drop any line we completed on a prior feed before touching the buffer.
    self.rotate();

    match byte {
      b'\r' | b'\n' => {
        self.pending = if byte == b'\r' { PendingTerminator::Cr } else { PendingTerminator::Lf };
        if self.overflowed {
          // The line was too long; we already reported overflow. Reset for the next line and emit
          // nothing here so the host sees exactly one error for the over-length line.
          self.buf.clear();
          self.overflowed = false;
          return LineEvent::Pending;
        }
        self.line_done = true;
        LineEvent::Line(self.buf.as_slice())
      }
      _ => {
        self.pending = PendingTerminator::None;
        if self.overflowed {
          // Still discarding the rest of an over-length line until its terminator arrives.
          return LineEvent::Pending;
        }
        if self.buf.push(byte).is_err() {
          // Buffer is full: enter overflow, drop the buffered partial, and report once. Subsequent bytes
          // of this line are discarded until the terminator.
          self.overflowed = true;
          self.buf.clear();
          return LineEvent::Overflow;
        }
        LineEvent::Pending
      }
    }
  }

  /// The most recently completed line, borrowing the internal buffer. Valid only immediately after a
  /// `feed` returned [`LineEvent::Line`] and before the next `feed`.
  fn line(&self) -> &[u8] {
    self.buf.as_slice()
  }
}

/// The persistent stream-level state of the connection: whether a prior GCode line has put the stream
/// into the grblHAL error-hold state. While held, subsequent GCode lines are rejected without parsing
/// until a recovery trigger (soft reset, empty line, or a `$` system command) clears the hold.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StreamState {
  /// Normal streaming; lines are forwarded to the parser.
  #[default]
  Ready,
  /// A GCode line errored; hold further GCode lines until a recovery trigger.
  ErrorHold,
}

/// The decision the orchestrator surfaces to the driver for one consumed byte. Exactly one of
/// [`AcceptLine`](EngineEvent::AcceptLine) or [`Reject`](EngineEvent::Reject) is produced per completed
/// GCode line so the driver emits exactly one `ok`/`error:N` — the sole host flow-control signal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EngineEvent<'a> {
  /// Mid-line, or a swallowed terminator half: nothing for the driver to do.
  None,
  /// A real-time command was intercepted; the driver dispatches the matching action/Signal and emits no
  /// `ok`.
  Realtime(RealtimeCommand),
  /// A complete GCode line is accepted for execution. The driver forwards it to the parser/planner; the
  /// single `ok` (or a deferred `error:N`) is emitted once the line is consumed downstream. The slice
  /// borrows the engine's line buffer and is valid until the next [`StreamEngine::ingest`].
  AcceptLine(&'a [u8]),
  /// A blank line (empty or whitespace-only). A blank line is a grblHAL error-hold recovery trigger, which
  /// the engine has already applied to its own hold. The driver must emit exactly one bare `ok` for it,
  /// either directly or by routing the blank line through its own line consumer when the driver owns a
  /// separate downstream error-hold that the blank must also clear.
  Acknowledge,
  /// A complete line is rejected at the protocol layer (over-length, or held by the error state). The
  /// driver emits `error:N` immediately and does not forward the line.
  Reject(u8),
}

/// The streaming orchestrator: composes a [`LineReader`] with the [`StreamState`] error-hold logic and
/// the real-time classifier into the single entry point the `usb_rx` task drives. It owns no I/O; the
/// driver feeds it bytes and acts on each [`EngineEvent`].
///
/// The error-hold transition is *not* decided here on accept — the protocol layer does not parse GCode,
/// so it cannot know a line will error. The driver calls [`note_line_error`](StreamEngine::note_line_error)
/// when the downstream parser/planner reports an error for a forwarded line, which arms the hold; the next
/// recovery trigger (empty line / `$` command / [`soft_reset`](StreamEngine::soft_reset)) clears it.
#[derive(Debug, Default)]
pub struct StreamEngine {
  reader: LineReader,
  state: StreamState,
}

impl StreamEngine {
  /// Construct a fresh engine in the ready state with an empty line buffer.
  pub const fn new() -> Self {
    Self {
      reader: LineReader::new(),
      state: StreamState::Ready,
    }
  }

  /// The current persistent stream state (ready vs. error-hold), for diagnostics/tests.
  pub fn state(&self) -> StreamState {
    self.state
  }

  /// Feed one received byte and get the driver's action. Real-time bytes are intercepted first and never
  /// enter the line buffer; otherwise the byte is framed and, on a completed line, accepted or rejected.
  pub fn ingest(&mut self, byte: u8) -> EngineEvent<'_> {
    if let Some(cmd) = classify_realtime(byte) {
      // A soft reset clears the partial line and lifts any error-hold; the driver still receives the
      // command so it can flush downstream queues and re-emit the banner.
      if cmd == RealtimeCommand::SoftReset {
        self.reader.reset();
        self.state = StreamState::Ready;
      }
      return EngineEvent::Realtime(cmd);
    }

    match self.reader.feed(byte) {
      LineEvent::Pending => EngineEvent::None,
      LineEvent::Overflow => EngineEvent::Reject(ERROR_LINE_OVERFLOW),
      LineEvent::Line(_) => self.decide_line(),
    }
  }

  /// Decide accept/reject for a freshly completed line, applying the error-hold rules. Splitting this out
  /// keeps the borrow of the reader's buffer scoped correctly for the returned slice.
  fn decide_line(&mut self) -> EngineEvent<'_> {
    let line = self.reader.line();
    let is_blank = line_is_blank(line);
    let is_system = line_is_system_command(line);

    // Recovery triggers always clear an error-hold: an empty/blank line or a `$` system command. Apply
    // this before the dispatch below so a recovery line is itself acted on, not rejected.
    if is_blank || is_system {
      self.state = StreamState::Ready;
    }

    if self.state == StreamState::ErrorHold {
      // Held by a prior error and not a recovery line: reject without forwarding. grblHAL holds the
      // stream in error after a bad line until a recovery trigger.
      return EngineEvent::Reject(ERROR_HOLD_CODE);
    }

    if is_blank {
      // A blank line is acknowledged directly (bare `ok`) and never forwarded to the parser.
      return EngineEvent::Acknowledge;
    }

    // Forward the line for execution (`$` system commands included; the driver routes those to the
    // settings/report handlers). The single `ok`/`error:N` is emitted downstream when consumed.
    EngineEvent::AcceptLine(self.reader.line())
  }

  /// Arm the error-hold after the downstream parser/planner reported `error:N` for a forwarded line. The
  /// driver calls this so subsequent GCode lines are held until a recovery trigger.
  pub fn note_line_error(&mut self) {
    self.state = StreamState::ErrorHold;
  }

  /// Clear the line buffer and lift any error-hold in response to a soft reset. Equivalent to the
  /// reset path taken when [`ingest`](StreamEngine::ingest) sees `0x18`, exposed for the driver to call
  /// when a reset originates elsewhere.
  pub fn soft_reset(&mut self) {
    self.reader.reset();
    self.state = StreamState::Ready;
  }
}

/// The `error:N` code used when the stream rejects a line because it is held in the post-error state.
/// grbl reports `error:1` family codes for parse failures; for the held-line rejection we reuse the
/// generic "expected command letter" code, matching how a sender already in an error-recovery posture
/// treats any further rejection — it is halting the stream regardless of the specific code.
const ERROR_HOLD_CODE: u8 = 1;

/// True if a line has no executable content (empty, or only whitespace). Such a line is the grblHAL
/// error-state recovery trigger and otherwise produces a bare `ok`.
fn line_is_blank(line: &[u8]) -> bool {
  line.iter().all(|&b| b == b' ' || b == b'\t')
}

/// True if a line is a `$` system command (after leading whitespace). `$` commands clear the error-hold
/// and are dispatched to the settings/report handlers rather than the GCode parser.
fn line_is_system_command(line: &[u8]) -> bool {
  for &b in line {
    match b {
      b' ' | b'\t' => continue,
      b'$' => return true,
      _ => return false,
    }
  }
  false
}

/// Formatting failure: the destination buffer was too small to hold the rendered response. Callers size
/// their buffers from the constants below, so this is a programming error rather than a runtime
/// condition, but it is surfaced as a `Result` to honor the firmware-core no-`unwrap` rule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FmtError;

/// A capacity, in bytes, that comfortably holds any single Stage-1 response line this module renders
/// (the longest is a status report or a build-info line). The `usb_tx` task allocates buffers of this
/// size on the stack / in a static pool.
pub const RESPONSE_CAPACITY: usize = 128;

/// Stateless formatters for every protocol response. Each writes a fully-formed response (including its
/// trailing line terminator where one is implied) into a caller-provided [`String`], returning
/// [`FmtError`] only if the buffer is too small. Keeping these pure and buffer-borrowing makes the wire
/// format host-testable byte-for-byte against the same vectors `skirnir` parses.
pub struct ResponseWriter;

impl ResponseWriter {
  /// The grbl welcome banner, emitted on boot and on every soft reset. A host treats receipt of this as
  /// "controller reset and ready". The grbl-compatible form (`Grbl 1.1f [...]`) is used so legacy and
  /// grblHAL senders alike recognize readiness; the extended grblHAL identity is exposed via `$I+`.
  pub fn banner<const N: usize>(out: &mut String<N>) -> Result<(), FmtError> {
    write!(out, "Grbl {VERSION} ['$' for help]\r\n").map_err(|_| FmtError)
  }

  /// The `ok` response — emitted exactly once per accepted line consumed downstream.
  pub fn ok<const N: usize>(out: &mut String<N>) -> Result<(), FmtError> {
    out.push_str("ok\r\n").map_err(|_| FmtError)
  }

  /// The `error:N` response — emitted exactly once for a rejected line.
  pub fn error<const N: usize>(out: &mut String<N>, code: u8) -> Result<(), FmtError> {
    write!(out, "error:{code}\r\n").map_err(|_| FmtError)
  }

  /// A `<...>` status report rendered from a [`MachineSnapshot`]. Stage-1 form:
  /// `<State|MPos:x,y,z|FS:feed,rpm|Bf:blocks,bytes>`. Substates are appended for `Hold`/`Alarm`. The
  /// element order follows the documented grblHAL order (State first, position second) so senders that
  /// position-parse do not break; the set is intentionally minimal, leaving `Pn:`/`Ov:`/`WCO:` for
  /// Stage 2.
  pub fn status_report<const N: usize>(out: &mut String<N>, snap: &MachineSnapshot) -> Result<(), FmtError> {
    out.push('<').map_err(|_| FmtError)?;
    out.push_str(snap.state.token()).map_err(|_| FmtError)?;
    match snap.state {
      MachineState::Hold(in_progress) => {
        write!(out, ":{}", in_progress as u8).map_err(|_| FmtError)?;
      }
      MachineState::Alarm(code) => {
        write!(out, ":{code}").map_err(|_| FmtError)?;
      }
      _ => {}
    }
    write!(
      out,
      "|MPos:{:.3},{:.3},{:.3}|FS:{:.0},{}|Bf:{},{}>\r\n",
      snap.mpos_mm[0],
      snap.mpos_mm[1],
      snap.mpos_mm[2],
      snap.feed_mm_min,
      snap.spindle_rpm,
      snap.planner_blocks_free,
      snap.rx_bytes_free,
    )
    .map_err(|_| FmtError)
  }

  /// The `$I` (or `$I+` when `extended`) build-info response. The base report emits `[VER:]` and
  /// `[OPT:]`; the extended report adds the grblHAL `[AXS:]`, `[NEWOPT:]`, and `[FIRMWARE:]` lines so a
  /// sender can detect an extended controller. The `[OPT:]` fields are, in order: options string, block
  /// buffer size, RX buffer size, axis count, tool-table entries — emitted in exactly the documented
  /// order so senders that position-parse OPT do not mis-read the buffer sizes. The caller appends `ok`.
  pub fn build_info<const N: usize>(out: &mut String<N>, extended: bool) -> Result<(), FmtError> {
    write!(out, "[VER:{VERSION}.20260616:]\r\n").map_err(|_| FmtError)?;
    write!(
      out,
      "[OPT:VNMSL,{},{},{},0]\r\n",
      BLOCK_BUFFER_SIZE, RX_BUFFER_SIZE, AXIS_COUNT,
    )
    .map_err(|_| FmtError)?;
    if extended {
      write!(out, "[AXS:{AXIS_COUNT}:XYZ]\r\n").map_err(|_| FmtError)?;
      // RT+ advertises the top-bit-set real-time command forms this module classifies.
      out.push_str("[NEWOPT:RT+]\r\n").map_err(|_| FmtError)?;
      out.push_str("[FIRMWARE:grblHAL]\r\n").map_err(|_| FmtError)?;
    }
    Ok(())
  }

  /// The `$G` parser-state report: `[GC:<modal words>]`. Stage 1 emits the power-on modal defaults
  /// (rapid, G54, XY plane, mm, absolute, units/min, no tool-length offset, spindle/coolant off). The
  /// `firmware` bin will render the live modal state from the parser once it is wired; the default form
  /// is sufficient for connect-time handshakes and is what a host expects immediately after reset.
  pub fn parser_state<const N: usize>(out: &mut String<N>) -> Result<(), FmtError> {
    out
      .push_str("[GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 F0 S0]\r\n")
      .map_err(|_| FmtError)
  }
}

#[cfg(test)]
mod tests {
  // firmware-core is `#![no_std]`; `std` only links under `#[cfg(test)]`. The recording helpers below
  // collect events into a `std::vec::Vec` to keep the byte-level assertions readable.
  extern crate std;
  use std::vec::Vec as StdVec;

  use super::*;

  // --- Real-time classification ---------------------------------------------------------------------

  #[test]
  fn classify_printable_realtime_forms() {
    assert_eq!(classify_realtime(b'?'), Some(RealtimeCommand::StatusReport));
    assert_eq!(classify_realtime(b'~'), Some(RealtimeCommand::CycleStart));
    assert_eq!(classify_realtime(b'!'), Some(RealtimeCommand::FeedHold));
    assert_eq!(classify_realtime(0x18), Some(RealtimeCommand::SoftReset));
  }

  #[test]
  fn classify_grblhal_top_bit_forms_alias_printable() {
    assert_eq!(classify_realtime(0x80), Some(RealtimeCommand::StatusReport));
    assert_eq!(classify_realtime(0x81), Some(RealtimeCommand::CycleStart));
    assert_eq!(classify_realtime(0x82), Some(RealtimeCommand::FeedHold));
    assert_eq!(classify_realtime(0x83), Some(RealtimeCommand::ParserStateReport));
    assert_eq!(classify_realtime(0x87), Some(RealtimeCommand::FullStatusReport));
    assert_eq!(classify_realtime(0x19), Some(RealtimeCommand::Stop));
    assert_eq!(classify_realtime(0x8C), Some(RealtimeCommand::ToggleAutoReport));
  }

  #[test]
  fn classify_override_bytes_preserve_raw() {
    assert_eq!(classify_realtime(0x90), Some(RealtimeCommand::Override(0x90)));
    assert_eq!(classify_realtime(0x9E), Some(RealtimeCommand::Override(0x9E)));
    assert_eq!(classify_realtime(0xA1), Some(RealtimeCommand::Override(0xA1)));
  }

  #[test]
  fn classify_ordinary_bytes_are_not_realtime() {
    for b in [b'G', b'0', b'X', b'5', b' ', b'$', b'\r', b'\n', b'='] {
      assert_eq!(classify_realtime(b), None, "byte {b:#x} must not classify as real-time");
    }
  }

  // --- Line framing & terminator collapse ------------------------------------------------------------

  /// Drive the reader with a byte string and collect each completed line as an owned `Vec` (so the
  /// transient borrow does not outlive the loop iteration). Overflow events are recorded as a sentinel.
  fn frame_lines(input: &[u8]) -> StdVec<StdVec<u8>> {
    let mut reader = LineReader::new();
    let mut out = StdVec::new();
    for &b in input {
      match reader.feed(b) {
        LineEvent::Line(line) => out.push(line.to_vec()),
        LineEvent::Overflow => out.push(b"<OVERFLOW>".to_vec()),
        LineEvent::Pending => {}
      }
    }
    out
  }

  #[test]
  fn frames_lf_terminated_line() {
    assert_eq!(frame_lines(b"G0 X1\n"), std::vec![b"G0 X1".to_vec()]);
  }

  #[test]
  fn frames_cr_terminated_line() {
    assert_eq!(frame_lines(b"G0 X1\r"), std::vec![b"G0 X1".to_vec()]);
  }

  #[test]
  fn crlf_is_a_single_terminator_no_double_ok() {
    // A CRLF must yield exactly one line, not a line plus a spurious empty line (the legacy double-ok).
    assert_eq!(frame_lines(b"G0 X1\r\n"), std::vec![b"G0 X1".to_vec()]);
  }

  #[test]
  fn lfcr_is_a_single_terminator() {
    assert_eq!(frame_lines(b"G0 X1\n\r"), std::vec![b"G0 X1".to_vec()]);
  }

  #[test]
  fn split_crlf_across_feeds_collapses_to_one_terminator() {
    // The \r and \n arrive as if in separate USB reads; the reader must still collapse them.
    let mut reader = LineReader::new();
    let mut lines = StdVec::new();
    for &b in b"M3" {
      assert_eq!(reader.feed(b), LineEvent::Pending);
    }
    match reader.feed(b'\r') {
      LineEvent::Line(l) => lines.push(l.to_vec()),
      other => panic!("expected line on CR, got {other:?}"),
    }
    // The following \n is the second half of the CRLF and must be swallowed, not start a new line.
    assert_eq!(reader.feed(b'\n'), LineEvent::Pending);
    assert_eq!(lines, std::vec![b"M3".to_vec()]);
  }

  #[test]
  fn two_consecutive_lines_each_terminate_once() {
    assert_eq!(
      frame_lines(b"G0 X1\r\nG1 Y2\r\n"),
      std::vec![b"G0 X1".to_vec(), b"G1 Y2".to_vec()],
    );
  }

  #[test]
  fn empty_line_frames_as_empty_slice() {
    // A bare terminator yields an empty line (the error-recovery trigger), not nothing.
    assert_eq!(frame_lines(b"\n"), std::vec![StdVec::<u8>::new()]);
  }

  #[test]
  fn double_lf_is_two_terminators_not_collapsed() {
    // Two LFs are two terminators (collapse only applies to the COMPLEMENTARY pair), so an empty line
    // appears between them.
    assert_eq!(frame_lines(b"X1\n\n"), std::vec![b"X1".to_vec(), StdVec::<u8>::new()]);
  }

  #[test]
  fn overlong_line_reports_overflow_once_then_recovers() {
    let mut input = std::vec![b'G'; MAX_LINE_LEN + 50];
    input.push(b'\n');
    input.extend_from_slice(b"G0\n");
    let framed = frame_lines(&input);
    // Exactly one overflow sentinel for the long line, then the following short line frames normally.
    assert_eq!(framed, std::vec![b"<OVERFLOW>".to_vec(), b"G0".to_vec()]);
  }

  // --- StreamEngine: real-time interception mid-line -------------------------------------------------

  /// Drive the engine and collect a compact, owned transcript of each ingest decision.
  #[derive(Debug, PartialEq, Eq)]
  enum Ev {
    None,
    Rt(RealtimeCommand),
    Accept(StdVec<u8>),
    Ack,
    Reject(u8),
  }

  fn run_engine(engine: &mut StreamEngine, input: &[u8]) -> StdVec<Ev> {
    let mut out = StdVec::new();
    for &b in input {
      let ev = match engine.ingest(b) {
        EngineEvent::None => Ev::None,
        EngineEvent::Realtime(c) => Ev::Rt(c),
        EngineEvent::AcceptLine(l) => Ev::Accept(l.to_vec()),
        EngineEvent::Acknowledge => Ev::Ack,
        EngineEvent::Reject(c) => Ev::Reject(c),
      };
      out.push(ev);
    }
    out
  }

  fn accepts(transcript: &[Ev]) -> StdVec<StdVec<u8>> {
    transcript
      .iter()
      .filter_map(|e| match e {
        Ev::Accept(l) => Some(l.clone()),
        _ => None,
      })
      .collect()
  }

  #[test]
  fn realtime_byte_mid_line_is_intercepted_without_disturbing_line() {
    // `?` arrives between `G1` and `X5`; it must surface as a Realtime event and NOT corrupt the line.
    let mut engine = StreamEngine::new();
    let transcript = run_engine(&mut engine, b"G1 X5\nG1 ?Y3\n");
    // Both lines frame correctly; the `?` did not enter either line.
    assert_eq!(accepts(&transcript), std::vec![b"G1 X5".to_vec(), b"G1 Y3".to_vec()]);
    // Exactly one StatusReport real-time event was emitted.
    let rt_count = transcript.iter().filter(|e| matches!(e, Ev::Rt(RealtimeCommand::StatusReport))).count();
    assert_eq!(rt_count, 1);
  }

  #[test]
  fn realtime_byte_never_produces_an_accept_or_reject() {
    let mut engine = StreamEngine::new();
    // A lone `?` with no surrounding line must produce only a Realtime event.
    let transcript = run_engine(&mut engine, b"?");
    assert_eq!(transcript.len(), 1);
    assert!(matches!(transcript[0], Ev::Rt(RealtimeCommand::StatusReport)));
  }

  #[test]
  fn one_accept_per_line_exactly() {
    let mut engine = StreamEngine::new();
    let transcript = run_engine(&mut engine, b"G0 X1\nG0 X2\nG0 X3\n");
    assert_eq!(accepts(&transcript).len(), 3, "exactly one accept per consumed line");
  }

  #[test]
  fn blank_line_acknowledges_without_forwarding() {
    let mut engine = StreamEngine::new();
    // A whitespace-only line is acknowledged directly and never forwarded as GCode.
    let transcript = run_engine(&mut engine, b"  \t \n");
    assert!(transcript.iter().any(|e| matches!(e, Ev::Ack)));
    assert!(accepts(&transcript).is_empty());
  }

  #[test]
  fn exactly_one_response_signal_per_line_across_mixed_input() {
    // Every consumed line must yield exactly one of Accept/Ack/Reject — the one-ok-per-line invariant.
    // Mix a move, a blank line, a `$` command, and (after a held error) a rejected line.
    let mut engine = StreamEngine::new();
    let mut signals = 0usize;
    for &b in b"G0 X1\n\n$$\n" {
      match engine.ingest(b) {
        EngineEvent::AcceptLine(_) | EngineEvent::Acknowledge | EngineEvent::Reject(_) => signals += 1,
        EngineEvent::None | EngineEvent::Realtime(_) => {}
      }
    }
    assert_eq!(signals, 3, "three consumed lines -> three response signals");
  }

  #[test]
  fn connect_handshake_transcript_drives_one_response_per_request() {
    // A representative connect sequence: status poll (real-time), a build-info query, then a move. The
    // status `?` produces only a Realtime event; the `$I` line and the move each produce one Accept.
    let mut engine = StreamEngine::new();
    let transcript = run_engine(&mut engine, b"?$I\nG0 X0\n");
    let rt = transcript.iter().filter(|e| matches!(e, Ev::Rt(RealtimeCommand::StatusReport))).count();
    assert_eq!(rt, 1);
    assert_eq!(accepts(&transcript), std::vec![b"$I".to_vec(), b"G0 X0".to_vec()]);
  }

  // --- Persistent error state & recovery -------------------------------------------------------------

  #[test]
  fn error_hold_rejects_subsequent_gcode_until_recovery() {
    let mut engine = StreamEngine::new();
    // First line accepted, then the driver learns it errored downstream and arms the hold.
    let t1 = run_engine(&mut engine, b"G0 X1\n");
    assert_eq!(accepts(&t1), std::vec![b"G0 X1".to_vec()]);
    engine.note_line_error();
    assert_eq!(engine.state(), StreamState::ErrorHold);

    // The next GCode line is rejected, not forwarded.
    let t2 = run_engine(&mut engine, b"G0 X2\n");
    assert!(matches!(t2.last(), Some(Ev::Reject(_))));
    assert!(accepts(&t2).is_empty());
    assert_eq!(engine.state(), StreamState::ErrorHold);
  }

  #[test]
  fn empty_line_clears_error_hold() {
    let mut engine = StreamEngine::new();
    engine.note_line_error();
    let t = run_engine(&mut engine, b"\nG0 X9\n");
    // The empty line clears the hold (and is itself not rejected); the following line is accepted.
    assert_eq!(accepts(&t), std::vec![b"G0 X9".to_vec()]);
    assert_eq!(engine.state(), StreamState::Ready);
  }

  #[test]
  fn dollar_command_clears_error_hold_and_is_accepted() {
    let mut engine = StreamEngine::new();
    engine.note_line_error();
    let t = run_engine(&mut engine, b"$$\n");
    // The `$` system command clears the hold and is itself forwarded (to the settings handler).
    assert_eq!(accepts(&t), std::vec![b"$$".to_vec()]);
    assert_eq!(engine.state(), StreamState::Ready);
  }

  #[test]
  fn soft_reset_clears_error_hold_and_partial_line() {
    let mut engine = StreamEngine::new();
    engine.note_line_error();
    // A partial line plus a soft reset: the reset must lift the hold and drop the partial.
    let t = run_engine(&mut engine, b"G0 X1\x18");
    assert!(matches!(t.last(), Some(Ev::Rt(RealtimeCommand::SoftReset))));
    assert_eq!(engine.state(), StreamState::Ready);
    // After reset the dropped partial does not resurface; a fresh line frames cleanly.
    let t2 = run_engine(&mut engine, b"G0 X2\n");
    assert_eq!(accepts(&t2), std::vec![b"G0 X2".to_vec()]);
  }

  // --- Response formatting (byte-level wire format) --------------------------------------------------

  #[test]
  fn banner_wire_format() {
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::banner(&mut s).unwrap();
    assert_eq!(s.as_str(), "Grbl 1.1f ['$' for help]\r\n");
  }

  #[test]
  fn ok_and_error_wire_format() {
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::ok(&mut s).unwrap();
    assert_eq!(s.as_str(), "ok\r\n");
    let mut e = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::error(&mut e, 15).unwrap();
    assert_eq!(e.as_str(), "error:15\r\n");
  }

  #[test]
  fn status_report_idle_wire_format() {
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::status_report(&mut s, &MachineSnapshot::idle()).unwrap();
    // Idle at origin, empty buffers fully free.
    assert_eq!(
      s.as_str(),
      "<Idle|MPos:0.000,0.000,0.000|FS:0,0|Bf:16,1024>\r\n",
    );
  }

  #[test]
  fn status_report_renders_position_and_substate() {
    let snap = MachineSnapshot {
      state: MachineState::Hold(false),
      mpos_mm: [1.5, -2.25, 0.125],
      feed_mm_min: 250.0,
      spindle_rpm: 1000,
      planner_blocks_free: 12,
      rx_bytes_free: 1000,
    };
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::status_report(&mut s, &snap).unwrap();
    assert_eq!(
      s.as_str(),
      "<Hold:0|MPos:1.500,-2.250,0.125|FS:250,1000|Bf:12,1000>\r\n",
    );
  }

  #[test]
  fn build_info_base_reports_buffer_sizes_in_documented_order() {
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::build_info(&mut s, false).unwrap();
    // OPT order: options, block buffer (16), RX buffer (1024), axes (3), tool entries (0).
    assert!(s.as_str().contains("[OPT:VNMSL,16,1024,3,0]"));
    assert!(s.as_str().contains("[VER:1.1f."));
    // Base report does not include the extended grblHAL lines.
    assert!(!s.as_str().contains("[NEWOPT:"));
    assert!(!s.as_str().contains("[FIRMWARE:"));
  }

  #[test]
  fn build_info_extended_adds_grblhal_lines() {
    let mut s = String::<256>::new();
    ResponseWriter::build_info(&mut s, true).unwrap();
    assert!(s.as_str().contains("[AXS:3:XYZ]"));
    assert!(s.as_str().contains("[NEWOPT:RT+]"));
    assert!(s.as_str().contains("[FIRMWARE:grblHAL]"));
  }

  #[test]
  fn parser_state_wire_format() {
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::parser_state(&mut s).unwrap();
    assert_eq!(s.as_str(), "[GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 F0 S0]\r\n");
  }

  #[test]
  fn fmt_error_on_undersized_buffer() {
    // A 4-byte buffer cannot hold the banner; the formatter reports FmtError rather than panicking.
    let mut s = String::<4>::new();
    assert_eq!(ResponseWriter::banner(&mut s), Err(FmtError));
  }
}
