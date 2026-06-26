//! The production [`Transport`] over USB-CDC serial, a thin adapter over `tokio-serial`.
//!
//! This is the *only* place in the crate that names `tokio-serial`. It opens and configures a
//! [`tokio_serial::SerialStream`] and forwards the engine's `read` / `write_all` to that stream's
//! `AsyncReadExt` / `AsyncWriteExt` methods, mapping I/O failures into the crate's [`TransportError`].
//! Everything above it works against the [`Transport`] trait and is oblivious to the serial crate, which is
//! exactly what keeps the engine host-testable against the loopback fake.
//!
//! Gated behind the `serial` feature so a loopback-only build need not pull the native serialport stack.

use std::sync::OnceLock;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Instant;

use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio_serial::{SerialPortBuilderExt, SerialPortType, SerialStream};

use crate::error::TransportError;
use crate::transport::Transport;
use crate::transport::ports::{PortInfo, normalize_ports};

/// A live serial connection to the firmware. Construct it with [`SerialTransport::open`]; from then on the
/// engine moves bytes through it via the [`Transport`] trait.
pub struct SerialTransport {
  stream: SerialStream,
  /// Connection id, stamped onto this transport's raw-byte trace so each open's bytes are attributable to the
  /// attempt that produced them — the field that distinguishes a buffered ack arriving on a *new* connection
  /// from a within-connection double-ack.
  conn_id: u64,
}

/// Monotonically increasing connection id, so every [`SerialTransport::open`] tags its bytes with the attempt
/// that produced them. A buffered ack flushed across a USB re-enumeration shows up under a *later* id than the
/// line that earned it — the discriminator the streaming-lockup investigation needs.
static CONNECTION_SEQ: AtomicU64 = AtomicU64::new(0);

/// Whether the raw-byte trace is enabled; read once from the `SKIRNIR_RAW_LOG` environment variable. Off unless
/// the variable is present and non-empty, so a normal run pays nothing and emits nothing — this is a debugging
/// aid for the streaming-lockup investigation, not a production feature.
fn raw_log_enabled() -> bool {
  static ENABLED: OnceLock<bool> = OnceLock::new();
  *ENABLED.get_or_init(|| std::env::var_os("SKIRNIR_RAW_LOG").map(|v| !v.is_empty()).unwrap_or(false))
}

/// Process-start instant for the trace's relative millisecond timestamps; initialised on first use so all records
/// share one monotonic epoch and can be ordered across reconnects.
fn trace_epoch() -> Instant {
  static EPOCH: OnceLock<Instant> = OnceLock::new();
  *EPOCH.get_or_init(Instant::now)
}

/// Direction of a raw-byte trace record: bytes written to the port versus bytes read from it.
#[derive(Clone, Copy)]
enum RawDir {
  Tx,
  Rx,
}

/// Render one raw-byte trace line: the connection id, a millisecond timestamp, the direction, the byte count, and
/// the bytes themselves with control characters escaped so the record stays single-line and grep-able. Pure, so
/// the escaping is unit-tested without a port.
fn format_raw(conn_id: u64, elapsed_ms: u128, dir: RawDir, bytes: &[u8]) -> String {
  let tag = match dir {
    RawDir::Tx => "TX",
    RawDir::Rx => "RX",
  };
  let mut rendered = String::with_capacity(bytes.len());
  for &b in bytes {
    match b {
      b'\n' => rendered.push_str("\\n"),
      b'\r' => rendered.push_str("\\r"),
      b'\\' => rendered.push_str("\\\\"),
      0x20..=0x7e => rendered.push(b as char),
      other => rendered.push_str(&format!("\\x{other:02x}")),
    }
  }
  format!("[raw] c{conn_id} +{elapsed_ms}ms {tag} {}B \"{rendered}\"", bytes.len())
}

/// Emit one raw-byte trace record to stderr when the trace is enabled and there are bytes to show. Cheap and inert
/// when disabled (a single relaxed atomic load), so it can sit on the hot read/write path.
fn trace_bytes(conn_id: u64, dir: RawDir, bytes: &[u8]) {
  if raw_log_enabled() && !bytes.is_empty() {
    eprintln!("{}", format_raw(conn_id, trace_epoch().elapsed().as_millis(), dir, bytes));
  }
}

impl SerialTransport {
  /// Open `path` (e.g. `/dev/ttyACM0`) at `baud` and return a ready transport. The ESP32-S3 native USB CDC
  /// ignores the line rate, but a sane value keeps the host driver happy. Opening or configuring failures
  /// are mapped to [`TransportError::Open`] so the UI can surface a recoverable error rather than crashing.
  pub fn open(path: &str, baud: u32) -> Result<Self, TransportError> {
    let stream = tokio_serial::new(path, baud)
      .open_native_async()
      .map_err(|err| TransportError::Open(err.to_string()))?;
    let conn_id = CONNECTION_SEQ.fetch_add(1, Ordering::Relaxed);
    Ok(Self { stream, conn_id })
  }
}

impl Transport for SerialTransport {
  async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
    // `AsyncReadExt::read` is cancel-safe and returns `Ok(0)` only at end-of-stream, matching the trait
    // contract the engine relies on for disconnect detection. The trace runs only after the await resolves, so
    // a cancelled read logs nothing — consistent with no bytes having moved.
    let n = self.stream.read(buf).await.map_err(map_io)?;
    trace_bytes(self.conn_id, RawDir::Rx, &buf[..n]);
    Ok(n)
  }

  async fn write(&mut self, data: &[u8]) -> Result<usize, TransportError> {
    // The cancel-safe primitive: `AsyncWriteExt::write` resolves after a single underlying write, so a future
    // dropped before it resolves has written nothing. The engine advances a cursor by the returned count and
    // resumes from it on the next call, so a write pre-empted under backpressure never re-sends a byte. The
    // trace logs the bytes actually accepted (`data[..n]`), after the await, for the same cancel-safety reason.
    let n = self.stream.write(data).await.map_err(map_io)?;
    trace_bytes(self.conn_id, RawDir::Tx, &data[..n]);
    Ok(n)
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

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn format_raw_tags_connection_timestamp_count_and_escapes_crlf() {
    // The common case: a host-sent `$G` line. The connection id and timestamp are carried verbatim, the byte
    // count is the slice length, and CR/LF are escaped so the record stays on one grep-able line.
    let line = format_raw(3, 1234, RawDir::Tx, b"$G\r\n");
    assert_eq!(line, "[raw] c3 +1234ms TX 4B \"$G\\r\\n\"");
  }

  #[test]
  fn format_raw_renders_non_printable_bytes_as_hex() {
    // A soft-reset byte (0x18) preceding an ack must show as `\x18`, not a raw control character that would
    // corrupt the log line — this is exactly the kind of byte the investigation watches for at a reconnect.
    let line = format_raw(0, 5, RawDir::Rx, &[0x18, b'o', b'k']);
    assert_eq!(line, "[raw] c0 +5ms RX 3B \"\\x18ok\"");
  }

  #[test]
  fn format_raw_escapes_a_literal_backslash() {
    // A backslash in the wire bytes must be doubled so the escaped rendering is unambiguous.
    let line = format_raw(1, 0, RawDir::Rx, b"a\\b");
    assert_eq!(line, "[raw] c1 +0ms RX 3B \"a\\\\b\"");
  }
}
