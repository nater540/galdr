//! Line framing + the streaming engine (`LineReader`/`StreamEngine`).

use super::*;
use heapless::Vec;

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
