//! Typed error surface for the streaming engine and its transports.
//!
//! These are `thiserror`-derived so callers (and the future UI) can match on variants and recover. The
//! engine itself never panics on a runtime condition: a bad port, a disconnect, or malformed firmware
//! output all flow through `EngineError` / `TransportError` rather than aborting the process.

use thiserror::Error;

/// A failure at the byte-transport boundary. Implementations of [`crate::transport::Transport`] surface
/// connection drops, I/O errors, and EOF through this type so the engine can react uniformly.
#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum TransportError {
  /// The underlying transport reported end-of-stream — the peer closed or the device disappeared. The
  /// engine treats this as a disconnect rather than an error to recover from in place.
  #[error("transport closed (end of stream)")]
  Closed,

  /// The controller stopped responding: either a write made no progress within the stall timeout (it stopped
  /// draining its RX), or it produced no inbound bytes at all within the response timeout (it still accepts writes
  /// but its processing/response path died). Both are firmware wedges; the engine surfaces them as a disconnect so
  /// the UI reflects the wedge and the serial FD is released, instead of spinning or hanging in Connecting.
  #[error("controller not responding (firmware may be wedged)")]
  Unresponsive,

  /// An I/O error occurred while reading or writing bytes. The message carries the OS-level detail.
  #[error("transport I/O error: {0}")]
  Io(String),

  /// Opening or configuring the device failed (bad path, permissions, busy port, unsupported settings).
  #[error("failed to open transport: {0}")]
  Open(String),
}

/// A failure inside the streaming engine proper — almost always a contract violation by the firmware or
/// a logic error a host must surface rather than stream through.
#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum EngineError {
  /// The firmware acknowledged a line we never sent (more `ok`/`error:N` than in-flight lines). This is a
  /// hard protocol-counting violation; continuing would desynchronise the character-count window.
  #[error("received an acknowledgement with no line in flight (spurious ok/error)")]
  UnexpectedAck,

  /// A single line, on its own, exceeds the advertised RX buffer and can never fit. Streaming it would
  /// deadlock the send-ahead window, so we reject it up front.
  #[error("line of {len} bytes exceeds the {rx_buffer}-byte RX buffer and can never be sent")]
  LineTooLong { len: usize, rx_buffer: usize },
}
