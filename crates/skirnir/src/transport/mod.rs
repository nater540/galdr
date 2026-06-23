//! The byte-transport abstraction the engine talks to the world through.
//!
//! The streaming engine never references `tokio-serial` (or any concrete device) directly — it moves bytes
//! only through the [`Transport`] trait. That keeps the engine fully host-testable against the in-memory
//! [`loopback::LoopbackTransport`] fake, and makes the production serial path a thin, swappable adapter.
//!
//! The trait is deliberately minimal: live byte movement only. Opening/configuring a device is a
//! constructor concern of each implementation, not part of the trait, so the trait surface stays about
//! reading and writing an already-open connection.

use crate::error::TransportError;

pub mod loopback;
pub mod ports;
pub mod probe;

#[cfg(feature = "serial")]
pub mod serial;

/// An async, bidirectional byte transport. The engine is generic over this trait (no dynamic dispatch on
/// the hot path), so both the loopback fake and the real serial port satisfy it with zero boxing.
///
/// Implementations must be cancel-safe enough to live inside a `tokio::select!`: a `read` whose future is
/// dropped before completing must not lose already-delivered bytes (the standard `AsyncReadExt::read`
/// contract). `Send` is required so the engine task can move across threads on the multi-threaded runtime.
pub trait Transport: Send {
  /// Read up to `buf.len()` bytes, returning how many were read. A return of `Ok(0)` signals end-of-stream
  /// (the peer closed / the device disappeared); the engine treats that as a disconnect.
  fn read(&mut self, buf: &mut [u8]) -> impl std::future::Future<Output = Result<usize, TransportError>> + Send;

  /// Write some prefix of `data`, returning how many bytes were accepted (`1..=data.len()` on success). This is
  /// the cancel-safety primitive: a single `write` resolves after at most one syscall, so a future dropped
  /// *before* it resolves has written nothing, and one dropped *after* has written exactly the returned count —
  /// never anything in between. The driver advances a cursor by the returned count and only retires a line once
  /// the cursor reaches its end, so a write pre-empted under serial backpressure resumes from the cursor and
  /// never re-sends already-sent bytes. (`AsyncWriteExt::write` over a serial port has exactly this contract.)
  fn write(&mut self, data: &[u8]) -> impl std::future::Future<Output = Result<usize, TransportError>> + Send;

  /// Write the entire `data` slice, retrying short writes internally. Returns only once every byte has been
  /// handed to the transport, so the caller's character-count accounting stays accurate.
  ///
  /// This is a provided convenience built on the cancel-safe [`Self::write`] primitive; it is NOT cancel-safe
  /// (a partial write completed before cancellation is lost to the caller), so the engine's hot write arms use
  /// cursor-tracked [`Self::write`] calls directly. It remains for teardown/best-effort flushes where a single
  /// abandonment is acceptable.
  fn write_all(&mut self, data: &[u8]) -> impl std::future::Future<Output = Result<(), TransportError>> + Send {
    async move {
      let mut offset = 0;
      while offset < data.len() {
        match self.write(&data[offset..]).await? {
          0 => return Err(TransportError::Closed),
          n => offset += n,
        }
      }
      Ok(())
    }
  }
}
