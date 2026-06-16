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
//! - **Real-time classification.** Provide [`classify_realtime`] so the driver can intercept single-byte
//!   real-time commands (`?`/`!`/`~`/`0x18`, the grblHAL `0x80`–`0x8C` top-bit forms, `0x19` stop, override
//!   bytes) ahead of line assembly; the driver dispatches them and they never enter a line nor receive an
//!   `ok`. The interception itself lives in the bin's reader half, not in the framer, so a real-time byte
//!   is never delayed behind line back-pressure.
//! - **Flow-control contract.** The framer yields exactly one accept/reject decision per consumed line so
//!   the driver can emit exactly one `ok`/`error:N` — the only signal driving host flow control.
//! - **Response formatting.** Banner, `ok`/`error:N`, `<...>` status report, `$I`/`$I+` build info,
//!   `$G` parser state, `$$` settings dump — all rendered into caller buffers.
//!
//! The grblHAL gcode error-hold is intentionally *not* here: the framer cannot know a forwarded line will
//! error downstream, so the hold is owned by the bin's single in-order consumer (which sees parse/plan
//! results in line order). This module frames lines and formats responses; it holds no error state.
//!
//! ## Out of scope here (Stage 2/3, left as clean extension points)
//! Alarm state machine, full status element set (`Pn:`/`Ov:`/`WCO:` refresh rules), runtime
//! enumerations (`$ES`/`$EE`/`$EA`), probing (`G38.x`/`[PRB:]`), and `$481` auto-report. The types
//! below reserve room for these (e.g. [`MachineState`] carries the states Stage 2 needs) without
//! implementing them yet.
//!
//! ## Driving contract (how the `firmware` bin uses this)
//! The bin's USB reader half extracts real-time bytes with [`classify_realtime`] *before* line assembly,
//! dispatching them through Signals so they never block behind line back-pressure. Non-real-time bytes are
//! buffered and drained through the line framer by a separate task:
//! ```ignore
//! // Reader half, per received byte:
//! if let Some(cmd) = classify_realtime(byte) { dispatch_signal(cmd); } else { rx_pipe.write(byte); }
//!
//! // Line-assembly half, draining the pipe one byte at a time:
//! let mut engine = StreamEngine::new();
//! match engine.ingest(byte) {
//!   EngineEvent::None             => {}                         // mid-line, nothing to do yet.
//!   EngineEvent::AcceptLine(line) => forward_to_consumer(line), // `ok` is emitted once consumed.
//!   EngineEvent::Reject(code)     => respond_error(code),       // emit `error:15` immediately.
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
/// field of `Bf:`. This is the single source of truth — the planner's [`BLOCK_QUEUE_LEN`](crate::planner::
/// BLOCK_QUEUE_LEN) — so the advertised depth, the idle snapshot's free count, and the live `Bf:` value the
/// `firmware` bin computes from the real queue can never drift apart. A host reads it to size look-ahead.
pub const BLOCK_BUFFER_SIZE: usize = crate::planner::BLOCK_QUEUE_LEN;

/// The maximum length of a single assembled GCode line, in bytes, excluding the terminator. A line that
/// would exceed this is rejected with `error:15` (line length exceeded) rather than silently truncated,
/// matching the grbl-family overflow contract. 256 comfortably covers grblHAL line lengths while keeping
/// the per-connection buffer small and allocation-free.
pub const MAX_LINE_LEN: usize = 256;

/// The grblHAL `error:N` code for an over-length input line ("line length exceeded").
pub const ERROR_LINE_OVERFLOW: u8 = 15;

/// The number of motion axes reported in build info and status (`[AXS:3:XYZ]`, three `MPos` fields).
/// References the planner's [`AXES`](crate::planner::AXES) so the protocol layer cannot disagree with the
/// kinematics about how many axes exist.
pub const AXIS_COUNT: usize = crate::planner::AXES;

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

}

/// The decision the line framer surfaces to the driver for one consumed byte. Exactly one of
/// [`AcceptLine`](EngineEvent::AcceptLine) or [`Reject`](EngineEvent::Reject) is produced per completed
/// line so the driver emits exactly one `ok`/`error:N` — the sole host flow-control signal.
///
/// Note: this engine performs *only* line framing. Real-time byte interception is no longer done here —
/// the `firmware` bin's USB reader half extracts real-time bytes with [`classify_realtime`] before any
/// byte reaches this framer (the realtime path must never block behind line back-pressure). Likewise the
/// grblHAL gcode error-hold lives downstream in the single in-order consumer, not here, because the framer
/// cannot know a forwarded line will error. Blank lines are therefore forwarded as an empty
/// [`AcceptLine`], not acknowledged here: the consumer owns both the bare `ok` and the blank-line
/// hold-recovery, keeping recovery in strict line order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EngineEvent<'a> {
  /// Mid-line, or a swallowed terminator half: nothing for the driver to do.
  None,
  /// A complete line is accepted for framing. The driver forwards it (including an empty line) to the
  /// single in-order consumer; the one `ok`/`error:N` is emitted once the line is consumed downstream. The
  /// slice borrows the engine's line buffer and is valid until the next [`StreamEngine::ingest`].
  AcceptLine(&'a [u8]),
  /// A complete line is rejected at the protocol layer (over-length). The driver emits `error:15`
  /// immediately and does not forward the line.
  Reject(u8),
}

/// The line framer the `firmware` bin's line-assembly half drives. It wraps a [`LineReader`] and surfaces
/// one [`EngineEvent`] per byte: nothing mid-line, an [`AcceptLine`](EngineEvent::AcceptLine) on each
/// completed line (blank lines included, as an empty slice), or a [`Reject`](EngineEvent::Reject) on an
/// over-length line. It owns no I/O, no real-time classification, and no error-hold — those concerns moved
/// to the reader half and the downstream consumer respectively (see [`EngineEvent`]).
#[derive(Debug, Default)]
pub struct StreamEngine {
  reader: LineReader,
}

impl StreamEngine {
  /// Construct a fresh engine with an empty line buffer.
  pub const fn new() -> Self {
    Self { reader: LineReader::new() }
  }

  /// Frame one received byte. Returns [`AcceptLine`](EngineEvent::AcceptLine) on a completed line (an empty
  /// slice for a bare/blank line), [`Reject`](EngineEvent::Reject) with `error:15` on overflow, or
  /// [`None`](EngineEvent::None) mid-line. Real-time bytes never reach here — the reader half diverts them.
  pub fn ingest(&mut self, byte: u8) -> EngineEvent<'_> {
    match self.reader.feed(byte) {
      LineEvent::Pending => EngineEvent::None,
      LineEvent::Overflow => EngineEvent::Reject(ERROR_LINE_OVERFLOW),
      LineEvent::Line(line) => EngineEvent::AcceptLine(line),
    }
  }

  /// Discard any partially accumulated line in response to a soft reset, so a fresh stream is not
  /// contaminated by a half-buffered line from before the reset. The driver calls this on `0x18` after
  /// flushing its downstream queues.
  pub fn soft_reset(&mut self) {
    self.reader.reset();
  }
}

/// The active motion mode reported as the first word of a `$G` (`[GC:...]`) parser-state line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParserMotion {
  /// G0 rapid positioning.
  Rapid,
  /// G1 linear feed move.
  Linear,
  /// G2 clockwise arc.
  ArcCw,
  /// G3 counter-clockwise arc.
  ArcCcw,
}

impl ParserMotion {
  /// The `G<n>` word for this motion mode.
  fn word(self) -> &'static str {
    match self {
      ParserMotion::Rapid => "G0",
      ParserMotion::Linear => "G1",
      ParserMotion::ArcCw => "G2",
      ParserMotion::ArcCcw => "G3",
    }
  }
}

/// The active units mode reported in a `$G` line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParserUnits {
  /// G20 inch units.
  Inch,
  /// G21 millimeter units.
  Millimeter,
}

impl ParserUnits {
  /// The `G<n>` word for this units mode.
  fn word(self) -> &'static str {
    match self {
      ParserUnits::Inch => "G20",
      ParserUnits::Millimeter => "G21",
    }
  }
}

/// The active distance mode reported in a `$G` line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParserDistance {
  /// G90 absolute distance.
  Absolute,
  /// G91 incremental distance.
  Incremental,
}

impl ParserDistance {
  /// The `G<n>` word for this distance mode.
  fn word(self) -> &'static str {
    match self {
      ParserDistance::Absolute => "G90",
      ParserDistance::Incremental => "G91",
    }
  }
}

/// A `Copy` snapshot of the live parser modal state the `$G` formatter renders. The `firmware` bin builds
/// this from the consumer's persistent `gcode::Parser` (via its `state()`) so the `[GC:...]` line reports
/// the real motion/units/distance/feed/spindle words rather than a hardcoded constant. Keeping the
/// formatter pure over a small snapshot — rather than importing the parser's modal type here — keeps
/// `protocol` free of any GCode-parsing coupling and the `$G` rendering host-testable in isolation.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ParserSnapshot {
  /// Active motion mode (modal group 1) — the first `$G` word.
  pub motion: ParserMotion,
  /// Active units mode (modal group 6).
  pub units: ParserUnits,
  /// Active distance mode (modal group 3).
  pub distance: ParserDistance,
  /// Programmed feed rate (modal F), in the active units per minute.
  pub feed: f32,
  /// Programmed spindle speed (modal S), in RPM.
  pub spindle_rpm: u16,
}

impl ParserSnapshot {
  /// The grbl power-on modal defaults (G0 rapid, mm, absolute, no feed, spindle off). Used before any
  /// motion word has been parsed, and as the basis for partial snapshots in tests.
  pub const fn power_on() -> Self {
    Self {
      motion: ParserMotion::Rapid,
      units: ParserUnits::Millimeter,
      distance: ParserDistance::Absolute,
      feed: 0.0,
      spindle_rpm: 0,
    }
  }
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

  /// The `$G` parser-state report: `[GC:<modal words>]`, rendered from a live [`ParserSnapshot`]. The
  /// motion (`G0`–`G3`), units (`G20`/`G21`), distance (`G90`/`G91`), feed (`F`), and spindle (`S`) words
  /// reflect the snapshot; the remaining modal groups (`G54` work coordinate, `G17` plane, `G94` feed
  /// mode, `M5` spindle stop, `M9` coolant off, `T0` tool) are fixed in Stage 1, where they are not yet
  /// commandable, but are emitted so the line is a complete, grbl-faithful modal report. Feed is written
  /// with a minimal decimal (no trailing `.0` for whole values) to match grbl's compact `$G` form.
  pub fn parser_state<const N: usize>(out: &mut String<N>, snap: &ParserSnapshot) -> Result<(), FmtError> {
    write!(
      out,
      "[GC:{} G54 G17 {} {} G94 M5 M9 T0 F",
      snap.motion.word(),
      snap.units.word(),
      snap.distance.word(),
    )
    .map_err(|_| FmtError)?;
    write_minimal_f32(out, snap.feed)?;
    write!(out, " S{}]\r\n", snap.spindle_rpm).map_err(|_| FmtError)
  }
}

/// Write an `f32` with the minimal decimal representation grbl uses for `$G` feed words: an integral value
/// renders with no fractional part (`1500` not `1500.0`), while a value with a fraction keeps only its
/// significant fractional digits (`250.25`, not `250.2500`). Rendering through a fixed-precision buffer and
/// trimming trailing zeros keeps this allocation-free and avoids pulling in float-to-shortest formatting.
fn write_minimal_f32<const N: usize>(out: &mut String<N>, value: f32) -> Result<(), FmtError> {
  // Three decimals covers feed resolution finer than any real machine; the trim below removes the padding
  // so a whole or one-/two-place value renders compactly.
  let mut scratch: String<24> = String::new();
  write!(scratch, "{value:.3}").map_err(|_| FmtError)?;
  let trimmed = if scratch.contains('.') {
    scratch.trim_end_matches('0').trim_end_matches('.')
  } else {
    scratch.as_str()
  };
  out.push_str(trimmed).map_err(|_| FmtError)
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

  // --- StreamEngine framer + reader-half real-time split --------------------------------------------

  /// One outcome of driving the framer with a byte (real-time bytes never reach the framer, so there is
  /// no `Rt` variant here — see [`split_input`] for the reader-half model).
  #[derive(Debug, PartialEq, Eq)]
  enum Ev {
    None,
    Accept(StdVec<u8>),
    Reject(u8),
  }

  fn run_engine(engine: &mut StreamEngine, input: &[u8]) -> StdVec<Ev> {
    let mut out = StdVec::new();
    for &b in input {
      let ev = match engine.ingest(b) {
        EngineEvent::None => Ev::None,
        EngineEvent::AcceptLine(l) => Ev::Accept(l.to_vec()),
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

  /// Model the `firmware` bin's USB reader half: classify each byte and partition the stream into the
  /// real-time commands the reader dispatches and the line bytes it forwards to the framer. This is the
  /// invariant that keeps real-time dispatch off the line back-pressure path — real-time bytes are removed
  /// before any framing happens.
  fn split_input(input: &[u8]) -> (StdVec<RealtimeCommand>, StdVec<u8>) {
    let mut realtime = StdVec::new();
    let mut line_bytes = StdVec::new();
    for &b in input {
      match classify_realtime(b) {
        Some(cmd) => realtime.push(cmd),
        None => line_bytes.push(b),
      }
    }
    (realtime, line_bytes)
  }

  #[test]
  fn reader_half_extracts_realtime_before_framing_without_disturbing_lines() {
    // `?` arrives between `G1` and `X5` in the wire stream; the reader half removes it before the framer
    // sees the line, so neither line is corrupted and exactly one StatusReport is dispatched.
    let (realtime, line_bytes) = split_input(b"G1 X5\nG1 ?Y3\n");
    assert_eq!(realtime, std::vec![RealtimeCommand::StatusReport]);
    let mut engine = StreamEngine::new();
    let transcript = run_engine(&mut engine, &line_bytes);
    assert_eq!(accepts(&transcript), std::vec![b"G1 X5".to_vec(), b"G1 Y3".to_vec()]);
  }

  #[test]
  fn framer_never_emits_realtime_and_lone_realtime_yields_no_line() {
    // The framer surface has no real-time variant. A lone `?` is fully consumed by the reader half and
    // never reaches the framer, so no line is produced.
    let (realtime, line_bytes) = split_input(b"?");
    assert_eq!(realtime, std::vec![RealtimeCommand::StatusReport]);
    assert!(line_bytes.is_empty());
    let mut engine = StreamEngine::new();
    assert!(run_engine(&mut engine, &line_bytes).is_empty());
  }

  #[test]
  fn one_accept_per_line_exactly() {
    let mut engine = StreamEngine::new();
    let transcript = run_engine(&mut engine, b"G0 X1\nG0 X2\nG0 X3\n");
    assert_eq!(accepts(&transcript).len(), 3, "exactly one accept per consumed line");
  }

  #[test]
  fn blank_line_is_forwarded_as_empty_accept_not_acknowledged() {
    // A whitespace-only line is forwarded to the single consumer as an AcceptLine over an empty slice (the
    // consumer owns the bare `ok` and the hold-recovery), never swallowed inside the framer.
    let mut engine = StreamEngine::new();
    let transcript = run_engine(&mut engine, b"  \t \n");
    let lines = accepts(&transcript);
    assert_eq!(lines.len(), 1, "the blank line is forwarded exactly once");
    // The forwarded line is whitespace-only; the consumer's trim makes it blank. (Leading/trailing space
    // is preserved here because the framer does no trimming — trimming is the consumer's job.)
    assert!(lines[0].iter().all(|&b| b == b' ' || b == b'\t'));
  }

  #[test]
  fn exactly_one_response_signal_per_line_across_mixed_input() {
    // Every consumed line must yield exactly one of Accept/Reject — the one-ok-per-line invariant. Mix a
    // move, a blank line, and a `$` command; the framer forwards all three (including the blank) and the
    // single downstream consumer answers each with one response.
    let mut engine = StreamEngine::new();
    let mut signals = 0usize;
    for &b in b"G0 X1\n\n$$\n" {
      match engine.ingest(b) {
        EngineEvent::AcceptLine(_) | EngineEvent::Reject(_) => signals += 1,
        EngineEvent::None => {}
      }
    }
    assert_eq!(signals, 3, "three consumed lines -> three response signals");
  }

  #[test]
  fn connect_handshake_splits_realtime_from_forwarded_lines() {
    // A representative connect sequence: a status poll (real-time), a build-info query, then a move. The
    // reader half dispatches the `?`; the framer forwards `$I` and the move as two accepted lines.
    let (realtime, line_bytes) = split_input(b"?$I\nG0 X0\n");
    assert_eq!(realtime, std::vec![RealtimeCommand::StatusReport]);
    let mut engine = StreamEngine::new();
    let transcript = run_engine(&mut engine, &line_bytes);
    assert_eq!(accepts(&transcript), std::vec![b"$I".to_vec(), b"G0 X0".to_vec()]);
  }

  #[test]
  fn soft_reset_drops_partial_line() {
    // The framer drops a half-buffered line on soft reset so a fresh stream is clean. (The reader half
    // would have already classified and dispatched the `0x18`; here we exercise the framer's reset hook.)
    let mut engine = StreamEngine::new();
    let _ = run_engine(&mut engine, b"G0 X1");
    engine.soft_reset();
    // After the reset the dropped partial does not resurface; a fresh line frames cleanly.
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
      "<Idle|MPos:0.000,0.000,0.000|FS:0,0|Bf:32,1024>\r\n",
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
    // OPT order: options, block buffer (32), RX buffer (1024), axes (3), tool entries (0).
    assert!(s.as_str().contains("[OPT:VNMSL,32,1024,3,0]"));
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
  fn parser_state_power_on_defaults_wire_format() {
    // The power-on modal defaults (G0 rapid, mm, absolute, F0/S0) render the canonical `$G` line a host
    // expects immediately after a reset.
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::parser_state(&mut s, &ParserSnapshot::power_on()).unwrap();
    assert_eq!(s.as_str(), "[GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 F0 S0]\r\n");
  }

  #[test]
  fn parser_state_renders_live_modal_words() {
    // A live state mid-program: G1 linear, inch units, incremental distance, feed 12.5, spindle 8000.
    let snap = ParserSnapshot {
      motion: ParserMotion::Linear,
      units: ParserUnits::Inch,
      distance: ParserDistance::Incremental,
      feed: 12.5,
      spindle_rpm: 8000,
    };
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::parser_state(&mut s, &snap).unwrap();
    assert_eq!(s.as_str(), "[GC:G1 G54 G17 G20 G91 G94 M5 M9 T0 F12.5 S8000]\r\n");
  }

  #[test]
  fn parser_state_maps_each_motion_mode_to_its_word() {
    let cases = [
      (ParserMotion::Rapid, "G0"),
      (ParserMotion::Linear, "G1"),
      (ParserMotion::ArcCw, "G2"),
      (ParserMotion::ArcCcw, "G3"),
    ];
    for (motion, word) in cases {
      let snap = ParserSnapshot { motion, ..ParserSnapshot::power_on() };
      let mut s = String::<RESPONSE_CAPACITY>::new();
      ResponseWriter::parser_state(&mut s, &snap).unwrap();
      // The first modal word in the `[GC:...]` body is the motion mode.
      assert!(s.as_str().starts_with(&std::format!("[GC:{word} ")), "motion {motion:?} -> {word}");
    }
  }

  #[test]
  fn parser_state_feed_renders_minimal_decimal() {
    // Feed renders without a trailing `.0` for whole values, but keeps a fractional part when present, so
    // the line stays compact and grbl-faithful.
    let whole = ParserSnapshot { feed: 1500.0, ..ParserSnapshot::power_on() };
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::parser_state(&mut s, &whole).unwrap();
    assert!(s.as_str().contains(" F1500 "), "whole feed: {}", s.as_str());

    let frac = ParserSnapshot { feed: 250.25, ..ParserSnapshot::power_on() };
    let mut s2 = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::parser_state(&mut s2, &frac).unwrap();
    assert!(s2.as_str().contains(" F250.25 "), "fractional feed: {}", s2.as_str());
  }

  #[test]
  fn fmt_error_on_undersized_buffer() {
    // A 4-byte buffer cannot hold the banner; the formatter reports FmtError rather than panicking.
    let mut s = String::<4>::new();
    assert_eq!(ResponseWriter::banner(&mut s), Err(FmtError));
  }
}
