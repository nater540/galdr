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
use tokio_serial::{SerialPortBuilderExt, SerialPortType, SerialStream};

use crate::error::TransportError;
use crate::transport::Transport;
use crate::transport::ports::{PortInfo, normalize_ports};

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

/// Enumerate the serial ports currently available on the host as structured [`PortInfo`] for the UI's port
/// dropdown, carrying whatever USB metadata the OS exposed (VID/PID/product/manufacturer/serial). The raw
/// listing is passed through [`normalize_ports`], which prefers the macOS `cu.*` callout over the `tty.*`
/// dialin (dedup the pair, translate a lone `tty.*`), leaves Linux nodes untouched, and floats Espressif-VID
/// (likely-Galdr) ports to the front. Enumeration failures yield an empty list rather than an error — a
/// missing port list is a recoverable, retryable UI condition, not something to crash on. `tokio-serial`
/// re-exports the underlying `serialport` enumeration.
pub fn available_ports() -> Vec<PortInfo> {
  let raw = tokio_serial::available_ports().unwrap_or_default();
  let infos = raw
    .into_iter()
    .map(|port| {
      // Lift the USB descriptor fields when this is a USB port; otherwise only the path is known. `serialport`
      // already exposes vid/pid as `u16`, so no parsing is needed — the metadata may simply be absent.
      match port.port_type {
        SerialPortType::UsbPort(usb) => PortInfo {
          path: port.port_name,
          vid: Some(usb.vid),
          pid: Some(usb.pid),
          product: usb.product,
          manufacturer: usb.manufacturer,
          serial: usb.serial_number,
        },
        _ => PortInfo::bare(port.port_name),
      }
    })
    .collect();
  normalize_ports(infos)
}

/// Map a std I/O error to the crate's transport error, distinguishing a closed pipe from a generic failure.
fn map_io(err: std::io::Error) -> TransportError {
  match err.kind() {
    std::io::ErrorKind::UnexpectedEof | std::io::ErrorKind::BrokenPipe => TransportError::Closed,
    _ => TransportError::Io(err.to_string()),
  }
}
