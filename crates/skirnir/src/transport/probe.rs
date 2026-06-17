//! On-demand active probe: confirm that an open [`Transport`] speaks grblHAL.
//!
//! This is **opt-in and user-triggered** — it is deliberately NOT part of port enumeration. Opening the
//! ESP32-S3's USB-Serial-JTAG toggles DTR/RTS (the auto-reset line), so silently sweeping every port on a
//! refresh could reset a board the user is actively running. The UI invokes this only when the user asks
//! ("Identify"), or folds it into Connect as a pre-flight against the single chosen port.
//!
//! It is built on the [`Transport`] trait (not the concrete serial port), so it runs against the in-memory
//! loopback fake in tests — Confirmed (inject a `<...>`/banner/`[VER:]`/`[OPT:]` line), NoResponse (inject
//! nothing → timeout), and a non-grbl device (inject garbage → timeout/NoResponse) are all testable without
//! hardware. Recognition reuses the production [`LineReassembler`] + [`parse_line`] rather than re-scanning
//! bytes, so the probe agrees with the engine on what counts as grbl evidence.
//!
//! Time-boundedness and cancel-safety: the whole probe is wrapped in a single [`tokio::time::timeout`], and
//! each read is itself a cancel-safe `Transport::read`. A dead or silent port can never hang the caller — the
//! probe returns [`ProbeVerdict::NoResponse`] when the deadline elapses with no grbl evidence seen.

use std::time::Duration;

use crate::error::TransportError;
use crate::protocol::{LineReassembler, is_grbl_evidence, parse_line};
use crate::transport::Transport;

/// The default probe window. Long enough to catch the ESP32-S3's status reply / banner after the DTR-toggle
/// reset settles, short enough that identifying a dead port does not stall the UI. The board here answers a
/// `?` essentially immediately, and re-emits its banner on the soft reset that opening induces.
pub const DEFAULT_PROBE_TIMEOUT: Duration = Duration::from_millis(500);

/// The probe's verdict on whether the port speaks grblHAL.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProbeVerdict {
  /// The port produced grbl evidence within the window: a `<...>` status report, the welcome banner, or a
  /// `[VER:...]` / `[OPT:...]` push message. This is the device we want.
  Confirmed,
  /// The port was reachable but produced no grbl evidence before the deadline — either silent, or a device
  /// that does not speak the protocol. We do not open it as a board.
  NoResponse,
  /// A transport error occurred while probing (the port closed, an I/O failure). The message carries detail.
  Error(String),
}

/// Bytes the probe sends to elicit grbl evidence: a real-time `?` (immediate `<...>` status — uncounted, safe
/// on any state) followed by `$I\n` (build-info, replies `[VER:]`/`[OPT:]`). The `?` alone usually suffices;
/// `$I` is a cheap belt-and-braces for a controller that happens to be momentarily quiet.
const PROBE_QUERY: &[u8] = b"?$I\n";


/// Probe `transport` for grblHAL, sending the query and reading until grbl evidence appears or `timeout`
/// elapses. Pure of any concrete device — it runs against the loopback fake in tests and the real serial port
/// in production. Returns [`ProbeVerdict::Confirmed`] on the first evidence line, [`ProbeVerdict::NoResponse`]
/// on a clean timeout or end-of-stream without evidence, and [`ProbeVerdict::Error`] on a transport failure.
pub async fn probe_grbl<T: Transport>(transport: &mut T, timeout: Duration) -> ProbeVerdict {
  // Wrap the entire probe in one deadline so a silent port can never hang the caller. The inner future only
  // ever returns early on evidence, EOF, or error — otherwise it loops reading, and the timeout cuts it off.
  match tokio::time::timeout(timeout, probe_inner(transport)).await {
    Ok(verdict) => verdict,
    // The deadline elapsed before any grbl evidence: the port did not identify as a board.
    Err(_elapsed) => ProbeVerdict::NoResponse,
  }
}

/// The unbounded inner probe: send the query, then read+deframe+parse until grbl evidence or the stream ends.
/// The caller wraps this in a timeout, so this returns only on a definitive outcome (evidence / EOF / error).
async fn probe_inner<T: Transport>(transport: &mut T) -> ProbeVerdict {
  if let Err(err) = transport.write_all(PROBE_QUERY).await {
    return ProbeVerdict::Error(err.to_string());
  }

  let mut reassembler = LineReassembler::new();
  let mut buf = [0u8; 256];
  loop {
    match transport.read(&mut buf).await {
      // End-of-stream before any evidence: reachable but not a grbl device (or it vanished). Not an error.
      Ok(0) => return ProbeVerdict::NoResponse,
      Ok(n) => {
        for line in reassembler.push(&buf[..n]) {
          // `accept_acks = false`: an unsolicited `ok`/`error`/`ALARM` from some unrelated device is not, on its
          // own, proof of grblHAL — only a status, banner, or `[VER:]`/`[OPT:]` build-info confirms the probe.
          if let Some(response) = parse_line(&line)
            && is_grbl_evidence(&response, false)
          {
            return ProbeVerdict::Confirmed;
          }
        }
      }
      Err(TransportError::Closed) => return ProbeVerdict::NoResponse,
      Err(err) => return ProbeVerdict::Error(err.to_string()),
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::transport::loopback::LoopbackTransport;

  #[tokio::test]
  async fn confirms_on_a_status_report() {
    let (mut transport, controller) = LoopbackTransport::new();
    controller.inject_line("<Idle|MPos:0.000,0.000,0.000|FS:0,0>");
    let verdict = probe_grbl(&mut transport, DEFAULT_PROBE_TIMEOUT).await;
    assert_eq!(verdict, ProbeVerdict::Confirmed);
  }

  #[tokio::test]
  async fn confirms_on_the_welcome_banner() {
    let (mut transport, controller) = LoopbackTransport::new();
    controller.inject_line("Grbl 1.1f ['$' for help]");
    assert_eq!(probe_grbl(&mut transport, DEFAULT_PROBE_TIMEOUT).await, ProbeVerdict::Confirmed);
  }

  #[tokio::test]
  async fn confirms_on_build_info_ver_line() {
    let (mut transport, controller) = LoopbackTransport::new();
    controller.inject_line("[VER:1.1f.20260616:]");
    assert_eq!(probe_grbl(&mut transport, DEFAULT_PROBE_TIMEOUT).await, ProbeVerdict::Confirmed);
  }

  #[tokio::test(start_paused = true)]
  async fn no_response_when_silent_until_timeout() {
    // Nothing injected and the controller is held open, so reads pend forever — the deadline must fire. Paused
    // time advances instantly to the deadline, so the test does not actually sleep 500ms.
    let (mut transport, _controller) = LoopbackTransport::new();
    assert_eq!(probe_grbl(&mut transport, DEFAULT_PROBE_TIMEOUT).await, ProbeVerdict::NoResponse);
  }

  #[tokio::test(start_paused = true)]
  async fn non_grbl_chatter_is_not_evidence_and_times_out() {
    // A device that talks but never speaks grbl: an unsolicited `ok` and random bytes are not evidence, so the
    // probe keeps reading until the deadline and reports NoResponse rather than a false Confirmed.
    let (mut transport, controller) = LoopbackTransport::new();
    controller.inject_line("ok");
    controller.inject_line("hello from some other gadget");
    assert_eq!(probe_grbl(&mut transport, DEFAULT_PROBE_TIMEOUT).await, ProbeVerdict::NoResponse);
  }

  #[tokio::test]
  async fn end_of_stream_without_evidence_is_no_response() {
    let (mut transport, mut controller) = LoopbackTransport::new();
    controller.close();
    assert_eq!(probe_grbl(&mut transport, DEFAULT_PROBE_TIMEOUT).await, ProbeVerdict::NoResponse);
  }

  #[tokio::test]
  async fn the_query_bytes_are_written_to_the_transport() {
    // Confirm the probe actually sends `?` + `$I` so a real controller has something to answer.
    let (mut transport, mut controller) = LoopbackTransport::new();
    controller.inject_line("<Idle|MPos:0.000,0.000,0.000>");
    let _ = probe_grbl(&mut transport, DEFAULT_PROBE_TIMEOUT).await;
    assert_eq!(controller.drain_written(), b"?$I\n".to_vec());
  }
}
