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

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use tokio::sync::Notify;
use tokio::sync::mpsc::{self, UnboundedReceiver, UnboundedSender};

use crate::error::TransportError;
use crate::transport::Transport;

/// A shared write gate that lets a test simulate serial backpressure: while the gate is *closed* the
/// transport's `write_all` parks until the gate is reopened, exactly as a real port pends when the firmware
/// stops draining its RX buffer mid-cut. Default state is open, so an ungated loopback never pends — every
/// existing test keeps its instantaneous-write behaviour. Cloneable (an `Arc` inside) so the controller and
/// the transport share one gate.
#[derive(Clone, Debug)]
struct WriteGate {
  /// `true` while writes may proceed; `false` parks them. Atomic so the (single-threaded test) reader and the
  /// controller never need a lock on the fast path.
  open: Arc<AtomicBool>,
  /// Notified whenever the gate is reopened, waking a parked `write_all`.
  reopened: Arc<Notify>,
}

impl WriteGate {
  /// A fresh, open gate (writes pass straight through).
  fn new() -> Self {
    Self { open: Arc::new(AtomicBool::new(true)), reopened: Arc::new(Notify::new()) }
  }

  /// Close the gate so subsequent `write_all`s park until [`Self::open_gate`] is called.
  fn close(&self) {
    self.open.store(false, Ordering::SeqCst);
  }

  /// Reopen the gate and wake any parked writer.
  fn open_gate(&self) {
    self.open.store(true, Ordering::SeqCst);
    self.reopened.notify_waiters();
  }

  /// Await until the gate is open. Returns immediately when already open; otherwise parks on the reopen
  /// notification. Cancel-safe: if the future is dropped before the gate reopens, no state is lost.
  async fn wait_open(&self) {
    while !self.open.load(Ordering::SeqCst) {
      // Register for the next reopen *before* re-checking would race; `Notify::notified()` is edge-triggered,
      // so we arm the wait then re-test under the loop to avoid missing a reopen between the load and the await.
      let notified = self.reopened.notified();
      if self.open.load(Ordering::SeqCst) {
        break;
      }
      notified.await;
    }
  }
}

/// The test-side controller paired with a [`LoopbackTransport`]. It injects bytes the fake firmware would
/// send and captures bytes the engine wrote. Dropping the controller closes the inbound channel, which the
/// engine sees as end-of-stream.
#[derive(Debug)]
pub struct LoopbackController {
  /// Bytes the test sends *to* the engine (the fake firmware's output). Closing this signals EOF.
  inbound_tx: UnboundedSender<Vec<u8>>,
  /// Bytes the engine wrote *out*; the test drains these to assert on what was streamed.
  outbound_rx: UnboundedReceiver<Vec<u8>>,
  /// The shared write gate. Closing it makes the transport's `write_all` park, simulating serial backpressure.
  gate: WriteGate,
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

  /// Close the write gate so the engine's next `write_all` parks instead of completing, simulating the firmware
  /// no longer draining its RX buffer mid-cut. Already-captured writes are unaffected; only the *next* (and
  /// subsequent) writes pend until [`Self::release_writes`]. Default state is open.
  pub fn gate_writes(&self) {
    self.gate.close();
  }

  /// Reopen the write gate, letting a parked `write_all` complete and subsequent writes pass through again.
  pub fn release_writes(&self) {
    self.gate.open_gate();
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
  /// The shared write gate. While closed, `write_all` parks here, simulating serial backpressure.
  gate: WriteGate,
}

impl LoopbackTransport {
  /// Create a loopback transport and its paired controller. Hand the transport to the engine; keep the
  /// controller in the test to inject firmware bytes and observe what the engine streams.
  pub fn new() -> (Self, LoopbackController) {
    let (inbound_tx, inbound_rx) = mpsc::unbounded_channel();
    let (outbound_tx, outbound_rx) = mpsc::unbounded_channel();
    let gate = WriteGate::new();
    let transport = Self { inbound_rx, outbound_tx, pending: Vec::new(), pending_at: 0, gate: gate.clone() };
    let controller = LoopbackController { inbound_tx, outbound_rx, gate };
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
    // Honour the write gate first: while a test holds it closed we park here, exactly as a real port pends when
    // the firmware stops draining RX. The await is cancel-safe, so a `select!` that drops this future loses no
    // captured bytes (none were forwarded yet). With the gate open (the default) this returns immediately.
    self.gate.wait_open().await;
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
  async fn a_closed_write_gate_parks_write_all_until_reopened() {
    let (mut transport, mut controller) = LoopbackTransport::new();
    controller.gate_writes();
    // With the gate closed the write must not complete; race it against a yield to prove it is parked.
    let write = transport.write_all(b"G0 X1\n");
    tokio::pin!(write);
    tokio::select! {
      biased;
      _ = &mut write => panic!("write_all completed while the gate was closed"),
      _ = tokio::task::yield_now() => {}
    }
    // Nothing was captured while parked.
    assert_eq!(controller.try_take_written(), None);
    // Reopen the gate; the parked write now completes and the bytes are captured.
    controller.release_writes();
    write.await.expect("write completes once the gate reopens");
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
