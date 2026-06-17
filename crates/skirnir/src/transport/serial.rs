//! The production [`Transport`] over USB-CDC serial, a thin adapter over `tokio-serial`.
//!
//! This is the *only* place in the crate that names `tokio-serial`. It opens and configures a
//! [`tokio_serial::SerialStream`] and forwards the engine's `read` / `write_all` to that stream's
//! `AsyncReadExt` / `AsyncWriteExt` methods, mapping I/O failures into the crate's [`TransportError`].
//! Everything above it works against the [`Transport`] trait and is oblivious to the serial crate, which is
//! exactly what keeps the engine host-testable against the loopback fake.
//!
//! Gated behind the `serial` feature so a loopback-only build need not pull the native serialport stack.

use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio_serial::{SerialPortBuilderExt, SerialStream};

use crate::error::TransportError;
use crate::transport::Transport;

/// A live serial connection to the firmware. Construct it with [`SerialTransport::open`]; from then on the
/// engine moves bytes through it via the [`Transport`] trait.
pub struct SerialTransport {
  stream: SerialStream,
}

impl SerialTransport {
  /// Open `path` (e.g. `/dev/ttyACM0`) at `baud` and return a ready transport. The ESP32-S3 native USB CDC
  /// ignores the line rate, but a sane value keeps the host driver happy. Opening or configuring failures
  /// are mapped to [`TransportError::Open`] so the UI can surface a recoverable error rather than crashing.
  pub fn open(path: &str, baud: u32) -> Result<Self, TransportError> {
    let stream = tokio_serial::new(path, baud)
      .open_native_async()
      .map_err(|err| TransportError::Open(err.to_string()))?;
    Ok(Self { stream })
  }
}

impl Transport for SerialTransport {
  async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
    // `AsyncReadExt::read` is cancel-safe and returns `Ok(0)` only at end-of-stream, matching the trait
    // contract the engine relies on for disconnect detection.
    self.stream.read(buf).await.map_err(map_io)
  }

  async fn write_all(&mut self, data: &[u8]) -> Result<(), TransportError> {
    self.stream.write_all(data).await.map_err(map_io)
  }
}

/// Map a std I/O error to the crate's transport error, distinguishing a closed pipe from a generic failure.
fn map_io(err: std::io::Error) -> TransportError {
  match err.kind() {
    std::io::ErrorKind::UnexpectedEof | std::io::ErrorKind::BrokenPipe => TransportError::Closed,
    _ => TransportError::Io(err.to_string()),
  }
}
