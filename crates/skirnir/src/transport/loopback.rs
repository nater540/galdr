//! An in-memory, scriptable [`Transport`] for testing the engine without a real serial port.
//!
//! A [`LoopbackTransport`] is the engine's view of a fake firmware: bytes the test *injects* are delivered
//! to the engine's `read`, and bytes the engine *writes* are captured for the test to assert on. The two
//! directions are independent `tokio::sync::mpsc` channels, so a test can drive a full streaming exchange
//! deterministically: inject `ok` bytes, observe the next line the engine streams, repeat.
//!
//! Cancel-safety: `read` awaits a single channel `recv`, which is cancel-safe — if the engine's `select!`
//! drops the read future before a chunk arrives, no buffered bytes are lost (none had been handed over).
//! When all inbound senders are dropped, `read` returns `Ok(0)` (end-of-stream), which the engine treats as
//! a disconnect — exactly how a real device disappearing behaves.

use tokio::sync::mpsc::{self, UnboundedReceiver, UnboundedSender};

use crate::error::TransportError;
use crate::transport::Transport;

/// The test-side controller paired with a [`LoopbackTransport`]. It injects bytes the fake firmware would
/// send and captures bytes the engine wrote. Dropping the controller closes the inbound channel, which the
/// engine sees as end-of-stream.
#[derive(Debug)]
pub struct LoopbackController {
  /// Bytes the test sends *to* the engine (the fake firmware's output). Closing this signals EOF.
  inbound_tx: UnboundedSender<Vec<u8>>,
  /// Bytes the engine wrote *out*; the test drains these to assert on what was streamed.
  outbound_rx: UnboundedReceiver<Vec<u8>>,
}

impl LoopbackController {
  /// Inject a chunk of "firmware" bytes for the engine to read. The engine's deframer will split these into
  /// lines exactly as it would real serial input, so callers can pass partial lines or split terminators to
  /// exercise reassembly. Returns whether the transport is still attached (the engine has not dropped it).
  pub fn inject(&self, bytes: impl Into<Vec<u8>>) -> bool {
    self.inbound_tx.send(bytes.into()).is_ok()
  }

  /// Inject a line of firmware text with a single `\n` terminator appended (the common case).
  pub fn inject_line(&self, line: &str) -> bool {
    let mut bytes = line.as_bytes().to_vec();
    bytes.push(b'\n');
    self.inject(bytes)
  }

  /// Signal end-of-stream by closing the inbound channel, so the engine's next `read` returns `Ok(0)` and it
  /// treats the device as disconnected.
  pub fn close(&mut self) {
    // Replacing the sender with a fresh, immediately-dropped one closes the original; the receiver side then
    // observes the channel as closed once any already-queued chunks drain.
    let (closed_tx, _closed_rx) = mpsc::unbounded_channel();
    self.inbound_tx = closed_tx;
  }

  /// Take the next chunk the engine wrote, or `None` if nothing is queued right now. Non-blocking.
  pub fn try_take_written(&mut self) -> Option<Vec<u8>> {
    self.outbound_rx.try_recv().ok()
  }

  /// Await the next chunk the engine writes. Returns `None` if the engine dropped its transport without
  /// writing anything more (the outbound channel closed).
  pub async fn next_written(&mut self) -> Option<Vec<u8>> {
    self.outbound_rx.recv().await
  }

  /// Drain every chunk the engine has written so far into a single flat byte vector. Useful for asserting on
  /// the exact wire stream after driving an exchange to a known quiescent point.
  pub fn drain_written(&mut self) -> Vec<u8> {
    let mut all = Vec::new();
    while let Ok(chunk) = self.outbound_rx.try_recv() {
      all.extend_from_slice(&chunk);
    }
    all
  }
}

/// The engine's side of the loopback: a [`Transport`] whose `read` yields test-injected bytes and whose
/// `write_all` captures bytes for the test. Construct it with [`LoopbackTransport::new`], which also returns
/// the paired [`LoopbackController`].
#[derive(Debug)]
pub struct LoopbackTransport {
  /// Inbound firmware bytes, delivered chunk-at-a-time from the controller.
  inbound_rx: UnboundedReceiver<Vec<u8>>,
  /// Bytes written by the engine, forwarded to the controller for assertions.
  outbound_tx: UnboundedSender<Vec<u8>>,
  /// Leftover inbound bytes from a chunk larger than the caller's `read` buffer, drained first next time.
  pending: Vec<u8>,
  /// Read cursor into `pending`.
  pending_at: usize,
}

impl LoopbackTransport {
  /// Create a loopback transport and its paired controller. Hand the transport to the engine; keep the
  /// controller in the test to inject firmware bytes and observe what the engine streams.
  pub fn new() -> (Self, LoopbackController) {
    let (inbound_tx, inbound_rx) = mpsc::unbounded_channel();
    let (outbound_tx, outbound_rx) = mpsc::unbounded_channel();
    let transport = Self { inbound_rx, outbound_tx, pending: Vec::new(), pending_at: 0 };
    let controller = LoopbackController { inbound_tx, outbound_rx };
    (transport, controller)
  }

  /// Copy as much of the current `pending` chunk as fits into `buf`, advancing the cursor. Returns the count
  /// copied; a return of 0 means `pending` was exhausted and a fresh chunk must be awaited.
  fn drain_pending_into(&mut self, buf: &mut [u8]) -> usize {
    let available = &self.pending[self.pending_at..];
    let n = available.len().min(buf.len());
    buf[..n].copy_from_slice(&available[..n]);
    self.pending_at += n;
    n
  }
}

impl Transport for LoopbackTransport {
  async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
    if buf.is_empty() {
      return Ok(0);
    }
    // Serve any leftover from a previously oversized chunk before awaiting a new one.
    if self.pending_at < self.pending.len() {
      return Ok(self.drain_pending_into(buf));
    }
    match self.inbound_rx.recv().await {
      // The controller closed (all senders dropped): end-of-stream, like a device disappearing.
      None => Ok(0),
      Some(chunk) => {
        self.pending = chunk;
        self.pending_at = 0;
        Ok(self.drain_pending_into(buf))
      }
    }
  }

  async fn write_all(&mut self, data: &[u8]) -> Result<(), TransportError> {
    // Forwarding to the controller cannot short-write; the whole slice is captured atomically. A send error
    // means the controller was dropped, which we report as a closed transport.
    self.outbound_tx.send(data.to_vec()).map_err(|_| TransportError::Closed)
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[tokio::test]
  async fn injected_bytes_are_delivered_to_read() {
    let (mut transport, controller) = LoopbackTransport::new();
    assert!(controller.inject(b"ok\n".to_vec()));
    let mut buf = [0u8; 16];
    let n = transport.read(&mut buf).await.expect("read");
    assert_eq!(&buf[..n], b"ok\n");
  }

  #[tokio::test]
  async fn a_chunk_larger_than_the_buffer_is_drained_across_reads() {
    let (mut transport, controller) = LoopbackTransport::new();
    assert!(controller.inject(b"abcdef".to_vec()));
    let mut buf = [0u8; 4];
    let n1 = transport.read(&mut buf).await.expect("read");
    assert_eq!(&buf[..n1], b"abcd");
    let n2 = transport.read(&mut buf).await.expect("read");
    assert_eq!(&buf[..n2], b"ef");
  }

  #[tokio::test]
  async fn written_bytes_are_captured_by_the_controller() {
    let (mut transport, mut controller) = LoopbackTransport::new();
    transport.write_all(b"G0 X1\n").await.expect("write");
    assert_eq!(controller.try_take_written().as_deref(), Some(b"G0 X1\n".as_slice()));
  }

  #[tokio::test]
  async fn closing_the_controller_yields_end_of_stream() {
    let (mut transport, mut controller) = LoopbackTransport::new();
    controller.close();
    let mut buf = [0u8; 8];
    assert_eq!(transport.read(&mut buf).await.expect("read"), 0);
  }
}
