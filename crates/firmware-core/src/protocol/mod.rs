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

// --- Submodule wiring ------------------------------------------------------------------------------
// Each cluster lives in its own file; all public paths stay stable via the glob re-exports below, so
// `firmware_core::protocol::X` (and in-crate `crate::protocol::X`) resolve exactly as before the split.
mod realtime;
mod codes;
mod state;
mod parser_state;
mod pins;
mod overrides;
mod diag_types;
mod report;
mod stream;
mod system_command;
mod response;

pub use realtime::*;
pub use codes::*;
pub use state::*;
pub use parser_state::*;
pub use pins::*;
pub use overrides::*;
pub use diag_types::*;
pub use report::*;
pub use stream::*;
pub use system_command::*;
pub use response::*;

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

/// The number of motion axes reported in build info and status (`[AXS:4:XYZA]`, four `MPos` fields).
/// References the planner's [`AXES`](crate::planner::AXES) so the protocol layer cannot disagree with the
/// kinematics about how many axes exist.
pub const AXIS_COUNT: usize = crate::planner::AXES;

/// The single-letter designation of each motion axis, in `MPos`/`[AXS:]` order. The `[AXS:<n>:<letters>]`
/// build-info line and the `[DRIVER:]` per-axis breakdown both derive their letters from this array, so the
/// axis-count constant and the letter set can never drift apart (e.g. a 3-axis build cannot advertise a stale
/// `XYZA`). Sized to [`AXIS_COUNT`] so adding/removing an axis is a one-line change here.
pub const AXIS_LETTERS: [char; AXIS_COUNT] = ['X', 'Y', 'Z', 'A'];

/// The firmware version string reported in the banner and the `[VER:]` build-info line. grblHAL reports
/// a grbl-1.1f-compatible version so senders compliant with grbl 1.1f recognize the controller.
pub const VERSION: &str = "1.1f";

/// `$10` status-report mask bit 0: when set, the report carries `MPos:` (machine position); when clear it
/// carries `WPos:` (work position). grbl 1.1+ always reports exactly one of the two.
pub const STATUS_MASK_MACHINE_POSITION: u8 = 0x01;

/// How many status reports may pass without a change-only element (`WCO:`/`Ov:`) before a periodic refresh
/// forces one (grbl's "every 10 or 30 reports"). 10 is grbl's motion-state cadence — a safe, frequent default
/// that keeps a host's derived state fresh without bloating every report. Shared by every [`RefreshReporter`].
pub const REFRESH_PERIOD: u16 = 10;

/// How many status reports may pass without a `WCO:` element before a periodic refresh forces one. An alias for
/// the shared [`REFRESH_PERIOD`], kept so callers and tests can name the WCO cadence specifically.
pub const WCO_REFRESH_PERIOD: u16 = REFRESH_PERIOD;

/// A capacity, in bytes, that comfortably holds any single response line this module renders (the longest is
/// a full status report or a build-info line). The `usb_tx` task allocates buffers of this size on the stack
/// / in a static pool. Sized for the Phase-E maximal report — `State` + `MPos`/`WPos` + `FS` + `Bf` + a full
/// `Pn:PXYZDHRS` + `WCO` + `Ov:200,100,200` with wide negative-millimeter positions — which approaches but
/// stays well under 160 bytes, so the formatter never returns [`FmtError`] for a real report.
pub const RESPONSE_CAPACITY: usize = 160;

/// The number of bracket lines the `$#` NGC-parameters block emits: G54-G59 (6) + G28 + G30 + G92 + TLO + PRB
/// = 11. The firmware bin loops `0..NGC_PARAMETER_LINES`, rendering one bracket per [`Response`], then `ok`.
pub const NGC_PARAMETER_LINES: usize = 11;

/// The `$#` bracket tags for the six work coordinate systems, indexed 0 = G54 … 5 = G59.
const WCS_TAGS: [&str; 6] = ["G54", "G55", "G56", "G57", "G58", "G59"];

/// How many status reports may pass without an `Ov:` element before a periodic refresh forces one. An alias for
/// the shared [`REFRESH_PERIOD`] (the `Ov:` cadence matches the `WCO:` one), kept so callers and tests can name
/// the override cadence specifically. The `Ov:` change/periodic logic itself lives in [`RefreshReporter`].
pub const OV_REFRESH_PERIOD: u16 = REFRESH_PERIOD;

#[cfg(test)]
mod tests {
  // firmware-core is `#![no_std]`; `std` only links under `#[cfg(test)]`. The recording helpers below
  // collect events into a `std::vec::Vec` to keep the byte-level assertions readable.
  extern crate std;
  use std::string::ToString;
  use std::vec::Vec as StdVec;

  use super::*;
  use core::fmt::Write as _;
  use heapless::String;

  // --- Wedge-reset fail-safe alarm (Design A, §20) --------------------------------------------------

  #[test]
  fn wedge_reset_alarm_is_homing_required_when_homing_enabled() {
    // Homing on → re-home to re-establish machine zero (ALARM:11, the user's chosen policy).
    assert_eq!(wedge_reset_alarm(true), AlarmCode::HomingRequired);
    assert_eq!(wedge_reset_alarm(true).code(), 11);
  }

  #[test]
  fn wedge_reset_alarm_is_abort_during_cycle_when_homing_disabled() {
    // Homing off → no machine reference to re-home to, so the coherent fail-safe is the "reset while in motion,
    // position lost" alarm (ALARM:3): it still rejects streaming until acknowledged, never a silent Idle resume.
    assert_eq!(wedge_reset_alarm(false), AlarmCode::AbortDuringCycle);
    assert_eq!(wedge_reset_alarm(false).code(), 3);
  }

  #[test]
  fn wedge_reset_alarm_never_returns_a_homing_requiring_alarm_when_homing_is_off() {
    // Guard the coherence property: the homing-off alarm must NOT be one whose unlock path needs `$H` (which errors
    // when homing is disabled). ALARM:3 unlocks via reset/`$X`, so it is safe.
    let off = wedge_reset_alarm(false);
    assert_ne!(off, AlarmCode::HomingRequired, "homing-off must not demand $H");
    assert_ne!(off.code(), 11);
  }

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
    // `0x88` toggles the optional-stop switch (gates M1). It must classify so it is diverted from the line buffer
    // rather than corrupting a line, even though grblHAL leaves it inert by default.
    assert_eq!(classify_realtime(0x88), Some(RealtimeCommand::ToggleOptionalStop));
    // `0x86` is the Galdr graceful program-stop real-time byte (a controlled decelerate-and-flush to Idle, no
    // alarm), distinct from the `0x18` abort. It must classify even though it carries the top bit, and must NOT
    // collide with jog-cancel (`0x85`) or the FullStatusReport (`0x87`) on either side of it.
    assert_eq!(classify_realtime(0x86), Some(RealtimeCommand::ProgramStop));
    assert_eq!(classify_realtime(0x85), Some(RealtimeCommand::JogCancel));
    assert_eq!(classify_realtime(0x87), Some(RealtimeCommand::FullStatusReport));
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
    // Idle at origin, empty buffers fully free. The idle default includes `WCO:` AND `Ov:` (grbl's first-report
    // rule re-emits both change-only elements), here zeros / 100% since nothing is set.
    assert_eq!(
      s.as_str(),
      "<Idle|MPos:0.000,0.000,0.000,0.000|FS:0,0|Bf:32,1024|WCO:0.000,0.000,0.000,0.000|Ov:100,100,100>\r\n",
    );
  }

  #[test]
  fn status_report_renders_position_and_substate() {
    // Machine-position report with the WCO element suppressed this cycle (the steady-state cadence).
    let snap = MachineSnapshot {
      state: MachineState::Hold(false),
      mpos_mm: [1.5, -2.25, 0.125, 0.0],
      wco_mm: [0.0, 0.0, 0.0, 0.0],
      position_report: PositionReport::Machine,
      include_wco: false,
      feed_mm_min: 250.0,
      spindle_rpm: 1000,
      planner_blocks_free: 12,
      rx_bytes_free: 1000,
      // Phase E elements suppressed this cycle so the assertion focuses on the position/substate fields.
      include_ov: false,
      ..MachineSnapshot::idle()
    };
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::status_report(&mut s, &snap).unwrap();
    assert_eq!(
      s.as_str(),
      "<Hold:0|MPos:1.500,-2.250,0.125,0.000|FS:250,1000|Bf:12,1000>\r\n",
    );
  }

  #[test]
  fn status_report_work_position_subtracts_wco() {
    // `$10` work-position mode: the report carries `WPos: = MPos − WCO`, plus the WCO element this cycle.
    let snap = MachineSnapshot {
      state: MachineState::Idle,
      mpos_mm: [10.0, 20.0, 5.0, 0.0],
      wco_mm: [10.0, 20.0, 5.0, 0.0],
      position_report: PositionReport::Work,
      include_wco: true,
      feed_mm_min: 0.0,
      spindle_rpm: 0,
      planner_blocks_free: 32,
      rx_bytes_free: 1024,
      include_ov: false,
      ..MachineSnapshot::idle()
    };
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::status_report(&mut s, &snap).unwrap();
    // WPos = (10,20,5) − (10,20,5) = (0,0,0); WCO element shows the offset.
    assert_eq!(
      s.as_str(),
      "<Idle|WPos:0.000,0.000,0.000,0.000|FS:0,0|Bf:32,1024|WCO:10.000,20.000,5.000,0.000>\r\n",
    );
  }

  #[test]
  fn status_report_machine_position_with_wco_element() {
    // Machine-position mode WITH the WCO element: a host can reconstruct WPos from MPos and WCO.
    let snap = MachineSnapshot {
      state: MachineState::Run,
      mpos_mm: [10.0, 20.0, 5.0, 0.0],
      wco_mm: [1.0, 2.0, 3.0, 0.0],
      position_report: PositionReport::Machine,
      include_wco: true,
      feed_mm_min: 100.0,
      spindle_rpm: 0,
      planner_blocks_free: 30,
      rx_bytes_free: 1020,
      include_ov: false,
      ..MachineSnapshot::idle()
    };
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::status_report(&mut s, &snap).unwrap();
    assert_eq!(
      s.as_str(),
      "<Run|MPos:10.000,20.000,5.000,0.000|FS:100,0|Bf:30,1020|WCO:1.000,2.000,3.000,0.000>\r\n",
    );
  }

  // --- Phase B: position-report mode, WCO cadence, and the `$#` parameters block --------------------

  #[test]
  fn position_report_from_status_mask() {
    // $10 bit 0 set ⇒ machine position; clear ⇒ work position.
    assert_eq!(PositionReport::from_status_mask(0x01), PositionReport::Machine);
    assert_eq!(PositionReport::from_status_mask(0xFF), PositionReport::Machine);
    assert_eq!(PositionReport::from_status_mask(0x00), PositionReport::Work);
    assert_eq!(PositionReport::from_status_mask(0x02), PositionReport::Work);
  }

  /// A WCO refresh reporter at its firmware-seeded baseline (origin WCO), exercising the same cadence the bin's
  /// `WCO_REPORTER` cell drives.
  fn wco_reporter() -> RefreshReporter<[f32; AXIS_COUNT]> {
    RefreshReporter::new([0.0; AXIS_COUNT])
  }

  #[test]
  fn wco_reporter_first_report_always_includes_wco() {
    let mut reporter = wco_reporter();
    assert!(reporter.should_include([0.0, 0.0, 0.0, 0.0]), "the first report after construction includes WCO");
  }

  #[test]
  fn wco_reporter_includes_on_change_then_suppresses() {
    let mut reporter = wco_reporter();
    reporter.should_include([0.0, 0.0, 0.0, 0.0]); // consume the forced first report.
    // No change → suppressed.
    assert!(!reporter.should_include([0.0, 0.0, 0.0, 0.0]));
    // A change → included immediately.
    assert!(reporter.should_include([10.0, 0.0, 0.0, 0.0]));
    // Same value again → suppressed.
    assert!(!reporter.should_include([10.0, 0.0, 0.0, 0.0]));
  }

  #[test]
  fn wco_reporter_periodic_refresh_every_period() {
    let mut reporter = wco_reporter();
    reporter.should_include([5.0, 0.0, 0.0, 0.0]); // forced first (records 5,0,0).
    let mut included = 0;
    // Run many steady (unchanged) reports; a refresh must fire on the periodic cadence.
    for _ in 0..(WCO_REFRESH_PERIOD * 3) {
      if reporter.should_include([5.0, 0.0, 0.0, 0.0]) {
        included += 1;
      }
    }
    // Over 3 periods of steady reports, exactly 3 periodic refreshes occur.
    assert_eq!(included, 3, "a periodic WCO refresh fires every {WCO_REFRESH_PERIOD} reports");
  }

  #[test]
  fn wco_reporter_reset_forces_next_include() {
    let mut reporter = wco_reporter();
    reporter.should_include([0.0, 0.0, 0.0, 0.0]);
    assert!(!reporter.should_include([0.0, 0.0, 0.0, 0.0]));
    reporter.reset([0.0; AXIS_COUNT]);
    assert!(reporter.should_include([0.0, 0.0, 0.0, 0.0]), "the first report after a reset includes WCO");
  }

  #[test]
  fn refresh_reporter_array_change_detect_is_elementwise() {
    // The `[f32; AXIS_COUNT]` change detection is the array's own element-wise `PartialEq`: a change on ANY axis
    // (here only Z) reads as changed, exactly as the prior hand-rolled per-axis loop did.
    let mut reporter = wco_reporter();
    reporter.should_include([1.0, 2.0, 3.0, 0.0]); // forced-first, records [1, 2, 3, 0.0].
    assert!(!reporter.should_include([1.0, 2.0, 3.0, 0.0]), "identical array suppresses");
    assert!(reporter.should_include([1.0, 2.0, 3.5, 0.0]), "a single-axis change is detected element-wise");
  }

  #[test]
  fn ngc_parameter_block_wire_format() {
    // A representative coordinate state: G54 offset, G28 stored, a G92, a Z TLO, and a successful probe.
    let report = CoordinateReport {
      wcs: [
        // G54 carries a rotary A offset (45°) so the test proves the fourth (A) field is actually rendered.
        [10.0, 20.0, 5.0, 45.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [-1.5, 2.5, 0.0, 0.0],
      ],
      predefined: [[100.0, 0.0, 50.0, 0.0], [0.0, 100.0, 50.0, 0.0]],
      g92: [1.0, 2.0, 3.0, 0.0],
      tlo: -14.442,
      // The probe carries an A value-at-trigger (90°) — the angle the touch happened at.
      probe: [-293.004, -16.995, -78.005, 90.0],
      probe_success: true,
    };
    let mut lines = StdVec::new();
    for index in 0..NGC_PARAMETER_LINES {
      let mut line = String::<RESPONSE_CAPACITY>::new();
      assert!(ResponseWriter::ngc_parameter_line(&mut line, &report, index));
      lines.push(line.as_str().to_string());
    }
    assert_eq!(
      lines,
      std::vec![
        "[G54:10.000,20.000,5.000,45.000]\r\n".to_string(),
        "[G55:0.000,0.000,0.000,0.000]\r\n".to_string(),
        "[G56:0.000,0.000,0.000,0.000]\r\n".to_string(),
        "[G57:0.000,0.000,0.000,0.000]\r\n".to_string(),
        "[G58:0.000,0.000,0.000,0.000]\r\n".to_string(),
        "[G59:-1.500,2.500,0.000,0.000]\r\n".to_string(),
        "[G28:100.000,0.000,50.000,0.000]\r\n".to_string(),
        "[G30:0.000,100.000,50.000,0.000]\r\n".to_string(),
        "[G92:1.000,2.000,3.000,0.000]\r\n".to_string(),
        "[TLO:-14.442]\r\n".to_string(),
        "[PRB:-293.004,-16.995,-78.005,90.000:1]\r\n".to_string(),
      ],
    );
  }

  #[test]
  fn ngc_parameter_line_out_of_range_is_false() {
    let report = CoordinateReport::default();
    let mut line = String::<RESPONSE_CAPACITY>::new();
    assert!(!ResponseWriter::ngc_parameter_line(&mut line, &report, NGC_PARAMETER_LINES));
  }

  #[test]
  fn ngc_default_report_is_all_zeros_with_failed_probe() {
    let report = CoordinateReport::default();
    let mut line = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::ngc_parameter_line(&mut line, &report, 10);
    assert_eq!(line.as_str(), "[PRB:0.000,0.000,0.000,0.000:0]\r\n");
  }

  #[test]
  fn build_info_base_reports_buffer_sizes_in_documented_order() {
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::build_info(&mut s, false).unwrap();
    // OPT order: options, block buffer (32), RX buffer (1024), axes (4), tool entries (0).
    assert!(s.as_str().contains("[OPT:VNMSL,32,1024,4,0]"));
    assert!(s.as_str().contains("[VER:1.1f."));
    // Base report does not include the extended grblHAL lines.
    assert!(!s.as_str().contains("[NEWOPT:"));
    assert!(!s.as_str().contains("[FIRMWARE:"));
  }

  #[test]
  fn build_info_extended_adds_grblhal_lines() {
    let mut s = String::<256>::new();
    ResponseWriter::build_info(&mut s, true).unwrap();
    assert!(s.as_str().contains("[AXS:4:XYZA]"));
    // Phase F: NEWOPT now advertises the enumeration + per-setting-description capabilities alongside RT+.
    assert!(s.as_str().contains("[NEWOPT:ENUMS,RT+,SED]"));
    assert!(s.as_str().contains("[FIRMWARE:grblHAL]"));
  }

  #[test]
  fn build_info_axs_letters_derive_from_axis_constants() {
    // The `[AXS:]` letters must come from AXIS_LETTERS, not a hardcoded literal — assemble the expected line
    // from the constants so a future axis-count change is caught here instead of shipping a stale `XYZA`.
    let mut expected = String::<32>::new();
    write!(expected, "[AXS:{AXIS_COUNT}:").unwrap();
    for letter in AXIS_LETTERS {
      expected.push(letter).unwrap();
    }
    expected.push(']').unwrap();
    let mut s = String::<256>::new();
    ResponseWriter::build_info(&mut s, true).unwrap();
    assert!(s.as_str().contains(expected.as_str()), "AXS must derive from AXIS_LETTERS, got {:?}", s.as_str());
  }

  #[test]
  fn build_info_extended_advertises_supported_signals() {
    // `[SIGNALS:]` lists the build's capable inputs in `Pn:` letter codes: probe + X/Y/Z limits → `PXYZ`. The
    // unbudgeted door/hold/reset/cycle-start inputs are omitted. The line is extended-only.
    let mut s = String::<256>::new();
    ResponseWriter::build_info(&mut s, true).unwrap();
    assert!(s.as_str().contains("[SIGNALS:PXYZ]\r\n"), "expected probe+XYZ-limit signals, got {:?}", s.as_str());
    let mut base = String::<256>::new();
    ResponseWriter::build_info(&mut base, false).unwrap();
    assert!(!base.as_str().contains("[SIGNALS:"), "SIGNALS must be extended-only");
  }

  #[test]
  fn driver_info_all_online_reports_each_axis_ok() {
    // Every TMC2209 answered on the UART bus: identity line plus a per-axis breakdown, all `ok`.
    let mut s = String::<160>::new();
    let status = DriverStatus { online: [true; AXIS_COUNT], initialized: true };
    ResponseWriter::driver_info(&mut s, &status).unwrap();
    assert!(s.as_str().contains("[DRIVER:TMC2209]\r\n"), "missing identity line, got {:?}", s.as_str());
    assert!(s.as_str().contains("[DRIVER:TMC2209 X:ok Y:ok Z:ok A:ok]\r\n"), "got {:?}", s.as_str());
  }

  #[test]
  fn driver_info_missing_driver_renders_dashes() {
    // The Y driver did not answer (no UART reply at init): its slot renders `Y:--`, the others stay `ok`.
    let mut s = String::<160>::new();
    let status = DriverStatus { online: [true, false, true, true], initialized: true };
    ResponseWriter::driver_info(&mut s, &status).unwrap();
    assert!(s.as_str().contains("[DRIVER:TMC2209 X:ok Y:-- Z:ok A:ok]\r\n"), "got {:?}", s.as_str());
  }

  #[test]
  fn driver_info_before_init_reports_pending_not_all_absent() {
    // A `$I+` during the boot window (init pass not yet run) must not claim every driver is absent.
    let mut s = String::<160>::new();
    let status = DriverStatus { online: [false; AXIS_COUNT], initialized: false };
    ResponseWriter::driver_info(&mut s, &status).unwrap();
    assert!(s.as_str().contains("[DRIVER:TMC2209 init pending]\r\n"), "got {:?}", s.as_str());
    assert!(!s.as_str().contains(":--"), "pending must not render per-axis dashes, got {:?}", s.as_str());
  }

  #[test]
  fn driver_probe_echo_timeout_renders_no_echo() {
    // A whole-bus echo timeout (the MCU's own bytes never looped back): the diagnostic marks every node
    // `no-echo`, pointing at the firmware/pin-matrix RX path rather than the drivers.
    let mut s = String::<160>::new();
    let probe = DriverProbe { stages: [TmcProbeStage::EchoTimeout; AXIS_COUNT], initialized: true, raw_ioin: None, init_failure: None };
    ResponseWriter::driver_probe(&mut s, &probe).unwrap();
    assert_eq!(s.as_str(), "[MSG:TMC-PROBE X:no-echo Y:no-echo Z:no-echo A:no-echo]\r\n", "got {:?}", s.as_str());
  }

  #[test]
  fn driver_probe_reply_timeout_renders_no_reply_with_fifo() {
    // Flash #4: the reply-timeout token carries the post-timeout FIFO snapshot. An empty FIFO (`fifo:0`, not
    // ready) means the driver was silent; a non-empty FIFO (`fifo:N`) means the reply arrived but was gated;
    // occupancy-without-drainable-bytes renders `fifo:0,rdy`.
    let mut s = String::<160>::new();
    let stages = [
      TmcProbeStage::ReplyTimeout { fifo: 0, ready: false },
      TmcProbeStage::ReplyTimeout { fifo: 8, ready: true },
      TmcProbeStage::ReplyTimeout { fifo: 0, ready: true },
      TmcProbeStage::ReplyTimeout { fifo: 20, ready: true },
    ];
    let probe = DriverProbe { stages, initialized: true, raw_ioin: None, init_failure: None };
    ResponseWriter::driver_probe(&mut s, &probe).unwrap();
    assert_eq!(
      s.as_str(),
      "[MSG:TMC-PROBE X:no-reply(fifo:0) Y:no-reply(fifo:8) Z:no-reply(fifo:0,rdy) A:no-reply(fifo:20)]\r\n",
      "got {:?}",
      s.as_str(),
    );
  }

  #[test]
  fn driver_probe_mixed_stages_render_per_axis() {
    // Per-axis tokens across all five outcomes: responding `ok`, echo `no-echo`, decode `crc`, wrong version
    // `ver:0xNN` (actual byte shown). This is the case-A vs case-B disambiguation the diagnostic exists for.
    let mut s = String::<160>::new();
    let stages = [
      TmcProbeStage::Responded,
      TmcProbeStage::EchoTimeout,
      TmcProbeStage::DecodeError,
      TmcProbeStage::VersionMismatch(0x10),
    ];
    let probe = DriverProbe { stages, initialized: true, raw_ioin: None, init_failure: None };
    ResponseWriter::driver_probe(&mut s, &probe).unwrap();
    assert_eq!(s.as_str(), "[MSG:TMC-PROBE X:ok Y:no-echo Z:crc A:ver:0x10]\r\n", "got {:?}", s.as_str());
  }

  #[test]
  fn driver_ioin_raw_dumps_reply_bytes_as_hex() {
    // The framing dump for a decode-error node: the raw 8-byte IOIN reply as space-separated lowercase hex, so
    // echo-reply misalignment / bit errors are eyeballable over serial without a scope.
    let mut s = String::<160>::new();
    let capture = IoinRawCapture { axis: 1, bytes: [0x05, 0xff, 0x6c, 0x00, 0x21, 0x1a, 0x00, 0x3c] };
    ResponseWriter::driver_ioin_raw(&mut s, &capture).unwrap();
    assert_eq!(s.as_str(), "[MSG:TMC-IOIN Y:0x05 0xff 0x6c 0x00 0x21 0x1a 0x00 0x3c]\r\n", "got {:?}", s.as_str());
  }

  #[test]
  fn loopback_line_renders_counts_and_error() {
    // A healthy MCU: every byte of the 8-byte pattern echoed back and matched, no RX error.
    let mut healthy = String::<80>::new();
    ResponseWriter::loopback(&mut healthy, &LoopbackReport { ran: true, got: 8, matched: 8, err: None }).unwrap();
    assert_eq!(healthy.as_str(), "[MSG:TMC-LOOPBACK sent:8 got:8 match:8/8 err:none]\r\n", "got {:?}", healthy.as_str());
    // A marginal MCU: only some bytes returned and fewer matched, with a framing error captured.
    let mut marginal = String::<80>::new();
    let report = LoopbackReport { ran: true, got: 5, matched: 3, err: Some(RxErrorKind::Framing) };
    ResponseWriter::loopback(&mut marginal, &report).unwrap();
    assert_eq!(marginal.as_str(), "[MSG:TMC-LOOPBACK sent:8 got:5 match:3/8 err:frm]\r\n", "got {:?}", marginal.as_str());
    // A dead RX half: nothing came back.
    let mut dead = String::<80>::new();
    ResponseWriter::loopback(&mut dead, &LoopbackReport { ran: true, got: 0, matched: 0, err: None }).unwrap();
    assert_eq!(dead.as_str(), "[MSG:TMC-LOOPBACK sent:8 got:0 match:0/8 err:none]\r\n", "got {:?}", dead.as_str());
  }

  #[test]
  fn tmc_bus_stats_render_per_axis() {
    // The margin meter renders every axis as fail/total in AXIS_LETTERS order, including a solid node (0/N), a
    // marginal one (3/120), a saturated long-bench counter, and a genuinely absent node (N/N).
    let stats = BusStats { fail: [0, 3, 65535, 60], total: [120, 120, 65535, 60] };
    let mut s = String::<96>::new();
    ResponseWriter::tmc_bus_stats(&mut s, &stats).unwrap();
    assert_eq!(s.as_str(), "[MSG:TMC-BUS X:0/120 Y:3/120 Z:65535/65535 A:60/60]\r\n", "got {:?}", s.as_str());
  }

  #[test]
  fn tmc_bus_stats_any_attempted_gates_on_totals() {
    // A fresh boot (no exchanges yet) must not emit the line; a single attempt on any axis unlocks it.
    assert!(!BusStats::default().any_attempted());
    let mut one = BusStats::default();
    one.total[2] = 1;
    assert!(one.any_attempted());
  }

  #[test]
  fn loopback_report_bits_roundtrip() {
    // The packed `u32` snapshot must round-trip `ran`, the counts, and every RX-error variant (incl. `None`).
    let cases = [
      LoopbackReport { ran: false, got: 0, matched: 0, err: None },
      LoopbackReport { ran: true, got: 8, matched: 8, err: None },
      LoopbackReport { ran: true, got: 5, matched: 3, err: Some(RxErrorKind::Framing) },
      LoopbackReport { ran: true, got: 8, matched: 7, err: Some(RxErrorKind::Overflow) },
      LoopbackReport { ran: true, got: 8, matched: 7, err: Some(RxErrorKind::Glitch) },
      LoopbackReport { ran: true, got: 8, matched: 7, err: Some(RxErrorKind::Parity) },
    ];
    for report in cases {
      assert_eq!(LoopbackReport::from_bits(report.to_bits()), report, "roundtrip failed for {report:?}");
    }
  }

  #[test]
  fn driver_probe_should_report_gates_on_init_and_fault() {
    // A healthy, fully-responding bus reports nothing; any non-responding outcome after init does; an
    // uninitialized probe stays quiet even if a stage looks bad (the snapshot is not yet valid).
    let healthy = DriverProbe { stages: [TmcProbeStage::Responded; AXIS_COUNT], initialized: true, raw_ioin: None, init_failure: None };
    assert!(!healthy.should_report(), "a fully-responding bus must add no diagnostic line");
    let faulted = DriverProbe { stages: [TmcProbeStage::DecodeError; AXIS_COUNT], initialized: true, raw_ioin: None, init_failure: None };
    assert!(faulted.should_report(), "a decode error after init must be reported");
    let pending = DriverProbe { stages: [TmcProbeStage::EchoTimeout; AXIS_COUNT], initialized: false, raw_ioin: None, init_failure: None };
    assert!(!pending.should_report(), "an uninitialized probe snapshot must not report");
  }

  #[test]
  fn driver_probe_write_verify_and_init_error_render() {
    // The write-verification blind-spot fix: a driver that reads fine but whose config writes did not land shows
    // `wrver:<actual>/<expected>` (0/8 = none verified, 3/8 = partial), and any other init error shows `err` —
    // so a node that errored during init is never silently `ok` again.
    let mut s = String::<160>::new();
    let stages = [
      TmcProbeStage::WriteVerify { expected: 8, actual: 0 },
      TmcProbeStage::WriteVerify { expected: 8, actual: 3 },
      TmcProbeStage::InitError,
      TmcProbeStage::Responded,
    ];
    let probe = DriverProbe { stages, initialized: true, raw_ioin: None, init_failure: None };
    ResponseWriter::driver_probe(&mut s, &probe).unwrap();
    assert_eq!(s.as_str(), "[MSG:TMC-PROBE X:wrver:0/8 Y:wrver:3/8 Z:err A:ok]\r\n", "got {:?}", s.as_str());
  }

  #[test]
  fn driver_probe_enriches_captured_init_error_node() {
    // The bare-`err` blind-spot fix: the captured InitError node names the failing register + op + kind
    // (`err:wGSTAT:to` = write to GSTAT timed out — a systemic write-path fault), while other `err` nodes stay
    // bare. This is exactly the on-board case (all `err`) resolved to a specific cause.
    let mut s = String::<160>::new();
    let stages = [TmcProbeStage::InitError; AXIS_COUNT];
    let failure = InitFailure {
      axis: 0,
      reg: crate::drivers::tmc2209::registers::GSTAT,
      was_write: true,
      kind: Some(BusErrorKind::Timeout),
      io_detail: None,
    };
    let probe = DriverProbe { stages, initialized: true, raw_ioin: None, init_failure: Some(failure) };
    ResponseWriter::driver_probe(&mut s, &probe).unwrap();
    assert_eq!(s.as_str(), "[MSG:TMC-PROBE X:err:wGSTAT:to Y:err Z:err A:err]\r\n", "got {:?}", s.as_str());
  }

  #[test]
  fn init_failure_token_renders_op_reg_and_kind() {
    // Each field is surfaced: read vs write prefix, register mnemonic (or hex fallback), and error kind — plus
    // the `cfg` fallback for an init error with no failed bus op (so a mis-eliminated cause is not hidden).
    let render = |failure: InitFailure| {
      let mut s = String::<64>::new();
      failure.write_token(&mut s).unwrap();
      s
    };
    let gconf = crate::drivers::tmc2209::registers::GCONF;
    assert_eq!(
      render(InitFailure { axis: 1, reg: gconf, was_write: true, kind: Some(BusErrorKind::Io), io_detail: None }).as_str(),
      "err:wGCONF:io",
    );
    let ifcnt = crate::drivers::tmc2209::registers::IFCNT;
    assert_eq!(
      render(InitFailure { axis: 0, reg: ifcnt, was_write: false, kind: Some(BusErrorKind::Timeout), io_detail: None }).as_str(),
      "err:rIFCNT:to",
    );
    // An unknown register falls back to hex; a `None` kind renders `cfg`.
    assert_eq!(
      render(InitFailure { axis: 2, reg: 0x42, was_write: true, kind: None, io_detail: None }).as_str(),
      "err:w0x42:cfg",
    );
  }

  #[test]
  fn init_failure_io_detail_appends_stage_and_variant_suffix() {
    // The RxError-detail enrichment: an `Io` on a read carries `@<stage><variant>` — the two discriminators the
    // bare `:io` lost. Echo-vs-reply changes the fix (a skip-echo workaround helps only the echo case), and the
    // variant separates edge quality (glt/frm) from cadence (ovf) from config drift (par).
    let render = |io_detail| {
      let mut s = String::<64>::new();
      let failure =
        InitFailure { axis: 0, reg: crate::drivers::tmc2209::registers::IOIN, was_write: false, kind: Some(BusErrorKind::Io), io_detail };
      failure.write_token(&mut s).unwrap();
      s
    };
    assert_eq!(render(Some((IoStage::Echo, RxErrorKind::Framing))).as_str(), "err:rIOIN:io@efrm");
    assert_eq!(render(Some((IoStage::Reply, RxErrorKind::Glitch))).as_str(), "err:rIOIN:io@rglt");
    assert_eq!(render(Some((IoStage::Echo, RxErrorKind::Overflow))).as_str(), "err:rIOIN:io@eovf");
    assert_eq!(render(Some((IoStage::Reply, RxErrorKind::Parity))).as_str(), "err:rIOIN:io@rpar");
    // A TX-side `io` (no RxError in scope) carries NO suffix — the absence itself distinguishes it from an RX error.
    assert_eq!(render(None).as_str(), "err:rIOIN:io");
  }

  #[test]
  fn init_failure_bits_roundtrip_including_none_and_validity() {
    // A cleared validity bit (stored 0) means "no failure captured".
    assert_eq!(InitFailure::from_bits(0), None);
    // Every field round-trips through the packed u32, including the `None`-kind (cfg) case, each axis, and the
    // optional io_detail across all stage × variant combinations.
    let base = [
      InitFailure { axis: 0, reg: crate::drivers::tmc2209::registers::GSTAT, was_write: true, kind: Some(BusErrorKind::Timeout), io_detail: None },
      InitFailure { axis: 3, reg: crate::drivers::tmc2209::registers::PWMCONF, was_write: true, kind: Some(BusErrorKind::Io), io_detail: None },
      InitFailure { axis: 2, reg: crate::drivers::tmc2209::registers::IFCNT, was_write: false, kind: Some(BusErrorKind::Decode), io_detail: None },
      InitFailure { axis: 1, reg: 0xFF, was_write: false, kind: None, io_detail: None },
    ];
    for failure in base {
      assert_eq!(InitFailure::from_bits(failure.to_bits()), Some(failure), "roundtrip failed for {failure:?}");
    }
    let ioin = crate::drivers::tmc2209::registers::IOIN;
    for stage in [IoStage::Echo, IoStage::Reply] {
      for variant in [RxErrorKind::Overflow, RxErrorKind::Glitch, RxErrorKind::Framing, RxErrorKind::Parity] {
        let failure =
          InitFailure { axis: 2, reg: ioin, was_write: false, kind: Some(BusErrorKind::Io), io_detail: Some((stage, variant)) };
        assert_eq!(InitFailure::from_bits(failure.to_bits()), Some(failure), "io_detail roundtrip failed for {failure:?}");
      }
    }
  }

  #[test]
  fn driver_probe_stage_bits_roundtrip() {
    // The 16-bit packing the firmware atomic uses must round-trip every outcome — including the version byte in
    // `VersionMismatch` and the nibble-packed IFCNT deltas in `WriteVerify` — and an unknown tag must degrade to
    // `Responded` (a corrupt snapshot reads as healthy, not a false fault).
    let cases = [
      TmcProbeStage::Responded,
      TmcProbeStage::EchoTimeout,
      TmcProbeStage::ReplyTimeout { fifo: 0, ready: false },
      TmcProbeStage::ReplyTimeout { fifo: 8, ready: true },
      TmcProbeStage::ReplyTimeout { fifo: 127, ready: false },
      TmcProbeStage::ReplyTimeout { fifo: 0, ready: true },
      TmcProbeStage::DecodeError,
      TmcProbeStage::VersionMismatch(0x00),
      TmcProbeStage::VersionMismatch(0x21),
      TmcProbeStage::VersionMismatch(0xFF),
      TmcProbeStage::WriteVerify { expected: 8, actual: 0 },
      TmcProbeStage::WriteVerify { expected: 8, actual: 8 },
      TmcProbeStage::WriteVerify { expected: 15, actual: 15 },
      TmcProbeStage::InitError,
      TmcProbeStage::RespondedDespiteGlitch(RxErrorKind::Overflow),
      TmcProbeStage::RespondedDespiteGlitch(RxErrorKind::Glitch),
      TmcProbeStage::RespondedDespiteGlitch(RxErrorKind::Framing),
      TmcProbeStage::RespondedDespiteGlitch(RxErrorKind::Parity),
      TmcProbeStage::DecodeErrorGlitched(RxErrorKind::Overflow),
      TmcProbeStage::DecodeErrorGlitched(RxErrorKind::Glitch),
      TmcProbeStage::DecodeErrorGlitched(RxErrorKind::Framing),
      TmcProbeStage::DecodeErrorGlitched(RxErrorKind::Parity),
    ];
    for stage in cases {
      assert_eq!(TmcProbeStage::from_bits(stage.to_bits()), stage, "roundtrip failed for {stage:?}");
    }
    // Counts above a nibble saturate at 15 rather than corrupting the tag (the 8-write set never hits this).
    assert_eq!(
      TmcProbeStage::from_bits(TmcProbeStage::WriteVerify { expected: 200, actual: 99 }.to_bits()),
      TmcProbeStage::WriteVerify { expected: 15, actual: 15 },
    );
    assert_eq!(TmcProbeStage::from_bits(0x0900), TmcProbeStage::Responded, "unknown tag must decode healthy");
  }

  #[test]
  fn driver_probe_glitch_tolerance_tokens_render() {
    // Flash #3: a driver that decoded only via RX tolerance shows `ok(<variant>)` (proven alive, reliance
    // visible), and a glitch that corrupted the decode shows `crc(<variant>)` — distinct from a clean-line
    // decode failure (`crc`), so a real SI problem is not mistaken for a framing coincidence.
    let mut s = String::<160>::new();
    let stages = [
      TmcProbeStage::RespondedDespiteGlitch(RxErrorKind::Glitch),
      TmcProbeStage::RespondedDespiteGlitch(RxErrorKind::Framing),
      TmcProbeStage::DecodeErrorGlitched(RxErrorKind::Glitch),
      TmcProbeStage::DecodeError,
    ];
    let probe = DriverProbe { stages, initialized: true, raw_ioin: None, init_failure: None };
    ResponseWriter::driver_probe(&mut s, &probe).unwrap();
    assert_eq!(s.as_str(), "[MSG:TMC-PROBE X:ok(glt) Y:ok(frm) Z:crc(glt) A:crc]\r\n", "got {:?}", s.as_str());
  }

  #[test]
  fn parser_state_power_on_defaults_wire_format() {
    // The power-on modal defaults (G0 rapid, mm, absolute, F0/S0) render the canonical `$G` line a host
    // expects immediately after a reset.
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::parser_state(&mut s, &ParserSnapshot::power_on()).unwrap();
    assert_eq!(s.as_str(), "[GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 G49 F0 S0]\r\n");
  }

  #[test]
  fn parser_state_renders_live_modal_words() {
    // A live state mid-program: G1 linear, inch units, incremental distance, G55 active, dynamic TLO on, feed
    // 12.5, spindle 8000.
    let snap = ParserSnapshot {
      motion: ParserMotion::Linear,
      units: ParserUnits::Inch,
      distance: ParserDistance::Incremental,
      feed_mode: ParserFeedMode::UnitsPerMin,
      wcs: 1,
      tlo_active: true,
      feed: 12.5,
      // S8000 alone (no M3) sets the speed but leaves the spindle stopped — grbl reports M5 with the S word.
      spindle: ParserSpindle::Stop,
      spindle_rpm: 8000,
      plane: ParserPlane::XY,
      coolant: ParserCoolant { mist: false, flood: false },
      tool: 0,
    };
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::parser_state(&mut s, &snap).unwrap();
    assert_eq!(s.as_str(), "[GC:G1 G55 G17 G20 G91 G94 M5 M9 T0 G43.1 F12.5 S8000]\r\n");
  }

  #[test]
  fn parser_state_renders_live_plane_and_coolant() {
    // G18/G19 and M7/M8 are now live modal state: the `$G` line must reflect them, not the old hardcoded G17/M9,
    // so a host re-establishing modal state after a reset reads the real plane and coolant. G18 ZX + flood (M8).
    let snap = ParserSnapshot {
      plane: ParserPlane::ZX,
      coolant: ParserCoolant { mist: false, flood: true },
      ..ParserSnapshot::power_on()
    };
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::parser_state(&mut s, &snap).unwrap();
    assert_eq!(s.as_str(), "[GC:G0 G54 G18 G21 G90 G94 M5 M8 T0 G49 F0 S0]\r\n");
    // Mist alone renders M7; both mist and flood render `M7 M8` (grbl reports both active group-8 words).
    let mist = ParserSnapshot { coolant: ParserCoolant { mist: true, flood: false }, ..ParserSnapshot::power_on() };
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::parser_state(&mut s, &mist).unwrap();
    assert_eq!(s.as_str(), "[GC:G0 G54 G17 G21 G90 G94 M5 M7 T0 G49 F0 S0]\r\n");
    let both = ParserSnapshot { coolant: ParserCoolant { mist: true, flood: true }, ..ParserSnapshot::power_on() };
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::parser_state(&mut s, &both).unwrap();
    assert_eq!(s.as_str(), "[GC:G0 G54 G17 G21 G90 G94 M5 M7 M8 T0 G49 F0 S0]\r\n");
    // G19 renders YZ.
    let yz = ParserSnapshot { plane: ParserPlane::YZ, ..ParserSnapshot::power_on() };
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::parser_state(&mut s, &yz).unwrap();
    assert_eq!(s.as_str(), "[GC:G0 G54 G19 G21 G90 G94 M5 M9 T0 G49 F0 S0]\r\n");
  }

  #[test]
  fn parser_state_renders_g93_inverse_time_feed_mode() {
    // G93 inverse-time mode must render as `G93` in the modal-group-5 slot (DOC-10.2), replacing the previously
    // hardcoded `G94` token. The default (UnitsPerMin) still renders `G94`, covered by the power-on test above.
    let snap = ParserSnapshot { feed_mode: ParserFeedMode::InverseTime, ..ParserSnapshot::power_on() };
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::parser_state(&mut s, &snap).unwrap();
    assert_eq!(s.as_str(), "[GC:G0 G54 G17 G21 G90 G93 M5 M9 T0 G49 F0 S0]\r\n");
  }

  #[test]
  fn parser_state_renders_spindle_direction_word() {
    // M3/M4 must report as the modal spindle word (group 7), not the hardcoded M5 — regression guard for the
    // now-commandable spindle direction.
    let cw = ParserSnapshot { spindle: ParserSpindle::Clockwise, spindle_rpm: 1000, ..ParserSnapshot::power_on() };
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::parser_state(&mut s, &cw).unwrap();
    assert_eq!(s.as_str(), "[GC:G0 G54 G17 G21 G90 G94 M3 M9 T0 G49 F0 S1000]\r\n");
    let ccw = ParserSnapshot { spindle: ParserSpindle::CounterClockwise, ..ParserSnapshot::power_on() };
    let mut s2 = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::parser_state(&mut s2, &ccw).unwrap();
    assert!(s2.as_str().contains(" M4 M9 "), "M4 reported: {}", s2.as_str());
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

  // --- Alarm code model -----------------------------------------------------------------------------

  #[test]
  fn alarm_codes_match_grbl_numbers() {
    assert_eq!(AlarmCode::HardLimit.code(), 1);
    assert_eq!(AlarmCode::SoftLimit.code(), 2);
    assert_eq!(AlarmCode::AbortDuringCycle.code(), 3);
    assert_eq!(AlarmCode::ProbeFailInitial.code(), 4);
    assert_eq!(AlarmCode::ProbeFailContact.code(), 5);
    assert_eq!(AlarmCode::EStop.code(), 10);
    assert_eq!(AlarmCode::HomingRequired.code(), 11);
    // ALARM:17 = grblHAL `Alarm_MotorFault` — a Galdr motion/step-sync transport fault (a swallowed mid-block RMT
    // error, §15.6/task #22). Mapped onto grblHAL's canonical motor-fault number so a sender labels it correctly.
    assert_eq!(AlarmCode::MotorFault.code(), 17);
  }

  #[test]
  fn motor_fault_alarm_is_locked_and_requires_rehome() {
    // ALARM:17 (motor/motion fault) is a LOCKED critical alarm: a mid-block step-sync break loses position
    // certainty on open-loop steppers, so per the grbl lost-step-sync contract (§14.3) the operator MUST re-home —
    // it is cleared only by a soft reset / `$H`, never silently resumed. The `'$H'|'$X' to unlock` prompt fits.
    assert_eq!(AlarmCode::MotorFault.code(), 17);
    assert!(AlarmCode::MotorFault.is_locked(), "a motion fault loses position — must be a locked alarm");
    assert_eq!(AlarmCode::MotorFault.unlock_hint(), "'$H'|'$X' to unlock");
    assert!(!AlarmCode::MotorFault.name().is_empty());
    assert!(!AlarmCode::MotorFault.description().is_empty());
  }

  #[test]
  fn locked_alarms_are_the_critical_subset() {
    // Codes 1, 2, 10, 17 are locked (cleared only by a soft reset / re-home); the rest accept `$X`.
    for code in [AlarmCode::HardLimit, AlarmCode::SoftLimit, AlarmCode::EStop, AlarmCode::MotorFault] {
      assert!(code.is_locked(), "{code:?} must be a locked critical alarm");
    }
    for code in [
      AlarmCode::AbortDuringCycle,
      AlarmCode::ProbeFailInitial,
      AlarmCode::ProbeFailContact,
      AlarmCode::HomingRequired,
    ] {
      assert!(!code.is_locked(), "{code:?} must be `$X`-clearable");
    }
  }

  #[test]
  fn alarm_unlock_hint_matches_grbl_prompts() {
    assert_eq!(AlarmCode::HomingRequired.unlock_hint(), "'$H'|'$X' to unlock");
    assert_eq!(AlarmCode::HardLimit.unlock_hint(), "'$H'|'$X' to unlock");
    assert_eq!(AlarmCode::AbortDuringCycle.unlock_hint(), "Reset to continue");
    assert_eq!(AlarmCode::ProbeFailContact.unlock_hint(), "Reset to continue");
  }

  // --- Control-state machine ------------------------------------------------------------------------

  #[test]
  fn boot_locks_when_homing_enabled_else_idle() {
    assert_eq!(ControlState::boot(true), ControlState::Alarm(AlarmCode::HomingRequired));
    assert_eq!(ControlState::boot(false), ControlState::Normal);
  }

  #[test]
  fn homing_allowed_from_normal_and_homing_required_only() {
    // `$H` runs from Idle/Normal and from the homing-required boot lock (the state `$H` exists to clear), but
    // never from a locked critical alarm, a feed-hold, check, or sleep (DOC-06).
    assert!(ControlState::Normal.homing_allowed());
    assert!(ControlState::Alarm(AlarmCode::HomingRequired).homing_allowed());
    assert!(!ControlState::Alarm(AlarmCode::HardLimit).homing_allowed());
    assert!(!ControlState::Alarm(AlarmCode::SoftLimit).homing_allowed());
    assert!(!ControlState::Alarm(AlarmCode::AbortDuringCycle).homing_allowed());
    assert!(!ControlState::Hold(false).homing_allowed());
    assert!(!ControlState::Check.homing_allowed());
    assert!(!ControlState::Sleep.homing_allowed());
  }

  #[test]
  fn hard_limit_alarm_applies_only_from_unlocked_states() {
    // A hard-limit trip raises `ALARM:1` only from a state where a genuine over-travel is meaningful — i.e. the
    // machine could actually be moving. From Normal, Hold, Jog, and Check the trip applies; a real over-travel
    // happens while a program/jog runs or while a held program could resume into a switch.
    assert!(ControlState::Normal.hard_limit_alarm_applies());
    assert!(ControlState::Hold(false).hard_limit_alarm_applies());
    assert!(ControlState::Hold(true).hard_limit_alarm_applies());
    assert!(ControlState::Jog.hard_limit_alarm_applies());
    assert!(ControlState::Check.hard_limit_alarm_applies());
    // From ANY alarm a trip must NOT apply: the machine is already halted, so the trip is a STALE read of a
    // parked switch, not a live over-travel. Re-raising `ALARM:1` can only clobber a more-specific state — most
    // damagingly downgrading the boot-lock `ALARM:11` (HomingRequired) into the locked `ALARM:1`, losing the
    // "homing required" semantic. This is exactly the post-aborted-homing race the soft-reset drain targets; the
    // guard is the belt-and-suspenders. Every alarm code — locked and `$X`-clearable alike — must be excluded.
    for code in [
      AlarmCode::HomingRequired,
      AlarmCode::HardLimit,
      AlarmCode::SoftLimit,
      AlarmCode::EStop,
      AlarmCode::HomingFail,
      AlarmCode::AbortDuringCycle,
    ] {
      assert!(
        !ControlState::Alarm(code).hard_limit_alarm_applies(),
        "{code:?}: a hard-limit trip must not re-fire from an already-alarmed state",
      );
    }
    // Sleep parks the drivers; a switch reading while asleep is not an over-travel — only a soft reset wakes it.
    assert!(!ControlState::Sleep.hard_limit_alarm_applies());
  }

  #[test]
  fn home_complete_clears_homing_required_to_normal() {
    // A successful `$H` establishes position and returns to Normal, clearing `ALARM:11`.
    assert_eq!(ControlState::Alarm(AlarmCode::HomingRequired).home_complete(), ControlState::Normal);
    // Re-homing from an already-unlocked Normal also lands in Normal.
    assert_eq!(ControlState::Normal.home_complete(), ControlState::Normal);
    // From a state where homing is not allowed, `home_complete` is a defensive no-op (the consumer gates first).
    // This is the invariant the `$H` success arm relies on (finding #2): if the control state was CLOBBERED to a
    // locked alarm in the post-cycle window, `home_complete` returns that SAME alarm — never `Normal` — so the
    // consumer can compare against `Normal` and refuse to emit a spurious `ok` / mark a false `HOMED`. Every
    // locked alarm must round-trip unchanged here.
    for code in [AlarmCode::HardLimit, AlarmCode::SoftLimit, AlarmCode::HomingFail, AlarmCode::AbortDuringCycle] {
      assert_eq!(
        ControlState::Alarm(code).home_complete(),
        ControlState::Alarm(code),
        "home_complete from a locked alarm stays locked (no false success window transition)",
      );
    }
  }

  #[test]
  fn normal_derives_run_vs_idle_from_in_flight_blocks() {
    // The Run/Idle distinction is derived, not latched: Normal + running -> Run, Normal + not -> Idle.
    assert_eq!(ControlState::Normal.machine_state(false), MachineState::Idle);
    assert_eq!(ControlState::Normal.machine_state(true), MachineState::Run);
  }

  #[test]
  fn latched_modes_ignore_the_running_flag() {
    // Every non-Normal mode reports its fixed state regardless of live execution, so a stray `running` can
    // never mask Hold/Alarm/Check/Sleep.
    assert_eq!(ControlState::Hold(false).machine_state(true), MachineState::Hold(false));
    assert_eq!(ControlState::Hold(true).machine_state(false), MachineState::Hold(true));
    assert_eq!(
      ControlState::Alarm(AlarmCode::HardLimit).machine_state(true),
      MachineState::Alarm(1),
    );
    assert_eq!(ControlState::Check.machine_state(true), MachineState::Check);
    assert_eq!(ControlState::Sleep.machine_state(true), MachineState::Sleep);
  }

  #[test]
  fn idle_to_hold_to_idle_round_trips() {
    // Idle -> Hold:0 on feed-hold, back to Idle (Normal, not running) on cycle-start.
    let held = ControlState::Normal.feed_hold();
    assert_eq!(held, ControlState::Hold(false));
    assert_eq!(held.machine_state(false), MachineState::Hold(false));
    let resumed = held.cycle_start();
    assert_eq!(resumed, ControlState::Normal);
    assert_eq!(resumed.machine_state(false), MachineState::Idle);
    assert_eq!(resumed.machine_state(true), MachineState::Run);
  }

  #[test]
  fn feed_hold_and_cycle_start_are_noops_outside_their_modes() {
    // `!` does nothing in an alarm; `~` does not clear an alarm or wake from sleep.
    let alarmed = ControlState::Alarm(AlarmCode::HomingRequired);
    assert_eq!(alarmed.feed_hold(), alarmed);
    assert_eq!(alarmed.cycle_start(), alarmed);
    assert_eq!(ControlState::Sleep.cycle_start(), ControlState::Sleep);
    // `~` from Normal (no hold pending) is a no-op, not an error.
    assert_eq!(ControlState::Normal.cycle_start(), ControlState::Normal);
  }

  #[test]
  fn cycle_start_resumes_only_a_feed_hold() {
    // `~` resumes ONLY a hold: it is inert everywhere else, matching grbl. This is the predicate the bin's
    // real-time `~` dispatch gates the executor-hold release on, so a `~` cannot strand a fresh hold, wake a
    // sleeping machine, or release a non-existent hold from Idle/Run/Jog/Alarm/Check.
    assert!(ControlState::Hold(false).resumes_on_cycle_start());
    assert!(ControlState::Hold(true).resumes_on_cycle_start());
    assert!(!ControlState::Normal.resumes_on_cycle_start());
    assert!(!ControlState::Jog.resumes_on_cycle_start());
    assert!(!ControlState::Check.resumes_on_cycle_start());
    // The Sleep case is the load-bearing one (Finding #1): `~` must NOT wake a sleeping machine — only a soft
    // reset does — even though `$SLP` parks the executor via the same hold latch a feed-hold uses.
    assert!(!ControlState::Sleep.resumes_on_cycle_start());
    assert!(!ControlState::Alarm(AlarmCode::HomingRequired).resumes_on_cycle_start());
    assert!(!ControlState::Alarm(AlarmCode::AbortDuringCycle).resumes_on_cycle_start());
  }

  // ---- M6 manual tool-change state (grblHAL `Tool`) ---------------------------------------------

  #[test]
  fn tool_change_state_reports_the_tool_token() {
    // An M6 manual tool change reports the dedicated grblHAL `Tool` state (no substate), distinct from a feed-hold's
    // `Hold:0`. The token has no colon-substate, so the formatter renders a bare `<Tool|...>`.
    assert_eq!(ControlState::Tool.machine_state(true), MachineState::Tool);
    assert_eq!(ControlState::Tool.machine_state(false), MachineState::Tool, "Tool is latched, ignores running");
    assert_eq!(MachineState::Tool.token(), "Tool");
  }

  #[test]
  fn tool_change_resumes_on_cycle_start_back_to_normal() {
    // Like a hold, the M6 tool-change state resumes on cycle-start (`~`) — but it returns to `Normal` (Run/Idle
    // re-derived), NOT to another hold. `resumes_on_cycle_start` includes it so the bin's `~` dispatch releases it.
    let tool = ControlState::tool_change();
    assert_eq!(tool, ControlState::Tool);
    assert!(tool.resumes_on_cycle_start(), "`~` resumes a tool-change hold");
    assert_eq!(tool.cycle_start(), ControlState::Normal, "resume returns to Normal (prior run state re-derived)");
    // Motion is allowed in the tool-change state so the resumed program can continue cutting (like Hold).
    assert!(tool.motion_allowed());
  }

  #[test]
  fn tool_change_is_distinct_from_feed_hold() {
    // The load-bearing distinction the wire contract requires: M0/M1 use `Hold(false)` → `Hold:0`; ONLY M6 enters
    // `Tool`. `feed_hold` never produces `Tool`, and `tool_change` never produces `Hold` — the two are separate.
    assert_eq!(ControlState::Normal.feed_hold(), ControlState::Hold(false));
    assert_eq!(ControlState::tool_change(), ControlState::Tool);
    assert_ne!(ControlState::tool_change(), ControlState::Hold(false));
    // A `~` from the tool state resumes (returns Normal); from a hold it also resumes — both are cycle-start-able,
    // but they are DIFFERENT wire states while held.
    assert_eq!(ControlState::Tool.machine_state(false), MachineState::Tool);
    assert_eq!(ControlState::Hold(false).machine_state(false), MachineState::Hold(false));
  }

  #[test]
  fn status_report_renders_bare_tool_state() {
    // The `?` wire string for an active M6 hold leads with a BARE `Tool` state token — no `:substate` (unlike
    // `Hold:0`). The remaining elements follow the idle defaults; the load-bearing assertion is the leading token.
    let snap = MachineSnapshot { state: MachineState::Tool, ..MachineSnapshot::idle() };
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::status_report(&mut s, &snap).unwrap();
    assert!(s.as_str().starts_with("<Tool|MPos:0.000,0.000,0.000,0.000"), "leads with a bare Tool state: {}", s);
    assert!(!s.as_str().contains("Tool:"), "the Tool state carries no `:substate`");
  }

  #[test]
  fn parser_state_reports_the_active_tool_number() {
    // `$G` must carry the current/active tool as `T<n>` (T0 = none). The power-on default is T0; a committed tool
    // (after M6) shows its number. This replaces the previously hardcoded `T0`.
    let none = ParserSnapshot::power_on();
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::parser_state(&mut s, &none).unwrap();
    assert_eq!(s.as_str(), "[GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 G49 F0 S0]\r\n");
    let with_tool = ParserSnapshot { tool: 5, ..ParserSnapshot::power_on() };
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::parser_state(&mut s, &with_tool).unwrap();
    assert_eq!(s.as_str(), "[GC:G0 G54 G17 G21 G90 G94 M5 M9 T5 G49 F0 S0]\r\n");
  }

  #[test]
  fn tool_change_message_names_the_committed_tool() {
    // The M6 hold prompt NAMES the committed tool so a bare-terminal operator knows which tool to insert. A
    // selected tool reads `... to T<n> ...`; T0 (a bare M6) reads `(no tool selected)`. Not a wire contract —
    // skirnir does not parse it — but the tool number must be present and the phrasing legible.
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::tool_change_message(&mut s, 5).unwrap();
    assert_eq!(s.as_str(), "Manual tool change to T5 \u{2014} swap tool, then cycle-start (~) to resume");
    assert!(s.as_str().contains("T5"), "the prompt names the committed tool");
    // T0 (no tool selected) phrases sensibly rather than printing a bare `T0`.
    let mut z = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::tool_change_message(&mut z, 0).unwrap();
    assert_eq!(z.as_str(), "Manual tool change (no tool selected) \u{2014} swap tool, then cycle-start (~) to resume");
    assert!(!z.as_str().contains("T0"), "T0 reads as `(no tool selected)`, not a bare `T0`");
    // Wrapped by `message`, it is a well-formed `[MSG:..]` push (the form `send_message` emits on the wire).
    let mut m = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::message(&mut m, s.as_str()).unwrap();
    assert_eq!(
      m.as_str(),
      "[MSG:Manual tool change to T5 \u{2014} swap tool, then cycle-start (~) to resume]\r\n",
    );
  }

  // ---- Phase D: jog control-state transitions ----------------------------------------------------

  #[test]
  fn jog_is_allowed_only_from_idle_or_jog() {
    // grbl accepts a jog from Idle/Run (Normal) or while already jogging; it is rejected from hold, alarm,
    // check, and sleep.
    assert!(ControlState::Normal.jog_allowed());
    assert!(ControlState::Jog.jog_allowed());
    assert!(!ControlState::Hold(false).jog_allowed());
    assert!(!ControlState::Alarm(AlarmCode::HomingRequired).jog_allowed());
    assert!(!ControlState::Check.jog_allowed());
    assert!(!ControlState::Sleep.jog_allowed());
  }

  #[test]
  fn begin_jog_latches_jog_and_reports_jog_or_idle_from_live_execution() {
    let jogging = ControlState::Normal.begin_jog();
    assert_eq!(jogging, ControlState::Jog);
    // Like Normal, the reported state derives from live execution: Jog while blocks run, Idle once drained.
    assert_eq!(jogging.machine_state(true), MachineState::Jog);
    assert_eq!(jogging.machine_state(false), MachineState::Idle);
    // Chaining a second jog while already jogging stays in Jog.
    assert_eq!(ControlState::Jog.begin_jog(), ControlState::Jog);
    // begin_jog from a non-jog-allowed mode is a defensive no-op.
    assert_eq!(ControlState::Hold(false).begin_jog(), ControlState::Hold(false));
  }

  #[test]
  fn cancel_jog_returns_to_normal_without_side_effects() {
    // A jog-cancel (`0x85`) drops Jog back to Normal; from any non-jog mode it is a no-op (ignored when not
    // jogging), since a jog never changed modal/coordinate state there is nothing to restore.
    assert_eq!(ControlState::Jog.cancel_jog(), ControlState::Normal);
    assert_eq!(ControlState::Normal.cancel_jog(), ControlState::Normal);
    assert_eq!(
      ControlState::Alarm(AlarmCode::HomingRequired).cancel_jog(),
      ControlState::Alarm(AlarmCode::HomingRequired),
    );
  }

  // ---- Graceful program stop (Galdr `0x86`) ------------------------------------------------------

  #[test]
  fn program_stop_returns_run_or_hold_to_idle_never_alarm() {
    // A graceful program stop (`0x86`) decelerates the running/held program to a controlled stop and returns the
    // machine to a motion-capable Idle (`Normal`) — NEVER an alarm, unlike the `0x18` abort which raises ALARM:3
    // mid-cycle. From `Normal` (Run or Idle) and from either `Hold` substate it lands in `Normal`.
    assert_eq!(ControlState::Normal.program_stop(), ControlState::Normal);
    assert_eq!(ControlState::Hold(false).program_stop(), ControlState::Normal);
    assert_eq!(ControlState::Hold(true).program_stop(), ControlState::Normal);
    // The resulting `Normal` reports `Idle` once motion drains (Run/Idle is derived from live execution), so a
    // host sees the machine come to rest at Idle, not in any alarm or hold.
    assert_eq!(ControlState::Hold(false).program_stop().machine_state(false), MachineState::Idle);
  }

  #[test]
  fn program_stop_is_a_benign_noop_outside_run_or_hold() {
    // From Idle the machine is already at rest in `Normal`, so a stop is a benign no-op (stays `Normal`). It must
    // never disturb an alarm, check mode, sleep, or an in-flight jog (jog has its own `0x85` cancel) — a program
    // stop is for a running PROGRAM, so every non-program state round-trips unchanged and no alarm is raised.
    for state in [
      ControlState::Alarm(AlarmCode::HomingRequired),
      ControlState::Alarm(AlarmCode::HardLimit),
      ControlState::Alarm(AlarmCode::AbortDuringCycle),
      ControlState::Check,
      ControlState::Sleep,
      ControlState::Jog,
    ] {
      assert_eq!(state.program_stop(), state, "{state:?}: program stop must be a no-op outside Run/Hold");
    }
  }

  #[test]
  fn program_stop_quiesces_only_from_run_or_hold() {
    // The bin gates the heavy boundary-quiesce + planner-flush work on this predicate: only `Normal` (which may be
    // running a program) and `Hold` need the executor parked and the queue flushed. Idle-`Normal` still answers
    // true (it is cheaply a no-op there — nothing queued), but the genuinely inert states answer false so a stray
    // `0x86` while alarmed/checking/sleeping/jogging does nothing.
    assert!(ControlState::Normal.program_stop_quiesces());
    assert!(ControlState::Hold(false).program_stop_quiesces());
    assert!(ControlState::Hold(true).program_stop_quiesces());
    assert!(!ControlState::Jog.program_stop_quiesces());
    assert!(!ControlState::Check.program_stop_quiesces());
    assert!(!ControlState::Sleep.program_stop_quiesces());
    assert!(!ControlState::Alarm(AlarmCode::HomingRequired).program_stop_quiesces());
    assert!(!ControlState::Alarm(AlarmCode::AbortDuringCycle).program_stop_quiesces());
  }

  #[test]
  fn soft_reset_clears_jog_state() {
    // `0x18` from a jog returns to boot (Normal when `$22` clear), clearing the jog latch — unless it aborted an
    // in-flight cycle, which raises the abort alarm like any other mid-motion reset.
    assert_eq!(ControlState::Jog.soft_reset(false, false), ControlState::Normal);
    assert_eq!(
      ControlState::Jog.soft_reset(true, false),
      ControlState::Alarm(AlarmCode::AbortDuringCycle),
    );
  }

  #[test]
  fn soft_reset_during_cycle_raises_abort_alarm() {
    // A reset that aborts an in-progress cycle -> ALARM:3 regardless of `$22`.
    assert_eq!(
      ControlState::Normal.soft_reset(true, false),
      ControlState::Alarm(AlarmCode::AbortDuringCycle),
    );
    assert_eq!(
      ControlState::Normal.soft_reset(true, true),
      ControlState::Alarm(AlarmCode::AbortDuringCycle),
    );
  }

  #[test]
  fn soft_reset_when_idle_returns_to_boot_state() {
    // A reset NOT aborting a cycle returns to boot: Idle when `$22` clear, homing-lock when set. A reset also
    // clears a non-homing alarm / check / sleep back to that baseline.
    assert_eq!(ControlState::Normal.soft_reset(false, false), ControlState::Normal);
    assert_eq!(
      ControlState::Normal.soft_reset(false, true),
      ControlState::Alarm(AlarmCode::HomingRequired),
    );
    assert_eq!(ControlState::Check.soft_reset(false, false), ControlState::Normal);
    assert_eq!(ControlState::Sleep.soft_reset(false, false), ControlState::Normal);
    assert_eq!(
      ControlState::Alarm(AlarmCode::AbortDuringCycle).soft_reset(false, false),
      ControlState::Normal,
    );
  }

  #[test]
  fn unlock_clears_non_locked_alarm_only() {
    // `$X` from a non-locked alarm -> Normal + Unlocked.
    let (state, outcome) = ControlState::Alarm(AlarmCode::HomingRequired).unlock();
    assert_eq!(state, ControlState::Normal);
    assert_eq!(outcome, UnlockOutcome::Unlocked);
    // `$X` from a locked critical alarm -> unchanged + Locked (reject).
    let (state, outcome) = ControlState::Alarm(AlarmCode::HardLimit).unlock();
    assert_eq!(state, ControlState::Alarm(AlarmCode::HardLimit));
    assert_eq!(outcome, UnlockOutcome::Locked);
    // `$X` from a non-alarm state -> no-op ok.
    let (state, outcome) = ControlState::Normal.unlock();
    assert_eq!(state, ControlState::Normal);
    assert_eq!(outcome, UnlockOutcome::NotAlarmed);
  }

  #[test]
  fn toggle_check_enters_and_leaves_with_reset() {
    // Normal -> Check (Enabled); Check -> boot state (Disabled, caller runs the reset).
    let (state, toggle) = ControlState::Normal.toggle_check(false);
    assert_eq!(state, ControlState::Check);
    assert_eq!(toggle, CheckToggle::Enabled);
    let (state, toggle) = ControlState::Check.toggle_check(false);
    assert_eq!(state, ControlState::Normal);
    assert_eq!(toggle, CheckToggle::Disabled);
    // Leaving check returns to the boot lock when `$22` is set.
    let (state, _) = ControlState::Check.toggle_check(true);
    assert_eq!(state, ControlState::Alarm(AlarmCode::HomingRequired));
    // `$C` is rejected from an alarm.
    let (state, toggle) = ControlState::Alarm(AlarmCode::HomingRequired).toggle_check(false);
    assert_eq!(state, ControlState::Alarm(AlarmCode::HomingRequired));
    assert_eq!(toggle, CheckToggle::Rejected);
  }

  #[test]
  fn check_mode_blocks_motion_but_allows_parsing() {
    // Check mode must not plan/execute (motion not allowed), but the consumer still parses + `ok`s the line.
    assert!(!ControlState::Check.motion_allowed());
    assert!(!ControlState::Alarm(AlarmCode::HomingRequired).motion_allowed());
    assert!(!ControlState::Sleep.motion_allowed());
    // Normal and Hold allow planning (a hold pauses at the executor, blocks may still queue).
    assert!(ControlState::Normal.motion_allowed());
    assert!(ControlState::Hold(false).motion_allowed());
  }

  #[test]
  fn enter_sleep_only_from_normal() {
    let (state, entered) = ControlState::Normal.enter_sleep();
    assert_eq!(state, ControlState::Sleep);
    assert!(entered);
    let (state, entered) = ControlState::Alarm(AlarmCode::HomingRequired).enter_sleep();
    assert_eq!(state, ControlState::Alarm(AlarmCode::HomingRequired));
    assert!(!entered);
  }

  // --- Status-report State rendering for each state -------------------------------------------------

  fn render_state(state: MachineState) -> std::string::String {
    let snap = MachineSnapshot { state, ..MachineSnapshot::idle() };
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::status_report(&mut s, &snap).unwrap();
    std::string::String::from(s.as_str())
  }

  #[test]
  fn status_report_renders_every_state_token() {
    assert!(render_state(MachineState::Idle).starts_with("<Idle|"));
    assert!(render_state(MachineState::Run).starts_with("<Run|"));
    assert!(render_state(MachineState::Hold(false)).starts_with("<Hold:0|"));
    assert!(render_state(MachineState::Hold(true)).starts_with("<Hold:1|"));
    assert!(render_state(MachineState::Alarm(11)).starts_with("<Alarm:11|"));
    assert!(render_state(MachineState::Alarm(1)).starts_with("<Alarm:1|"));
    assert!(render_state(MachineState::Check).starts_with("<Check|"));
    assert!(render_state(MachineState::Sleep).starts_with("<Sleep|"));
    assert!(render_state(MachineState::Door).starts_with("<Door|"));
    assert!(render_state(MachineState::Home).starts_with("<Home|"));
    assert!(render_state(MachineState::Jog).starts_with("<Jog|"));
  }

  // --- Alarm / message / help formatting ------------------------------------------------------------

  #[test]
  fn alarm_and_message_wire_format() {
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::alarm(&mut s, AlarmCode::HomingRequired).unwrap();
    assert_eq!(s.as_str(), "ALARM:11\r\n");
    let mut m = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::message(&mut m, "Caution: Unlocked").unwrap();
    assert_eq!(m.as_str(), "[MSG:Caution: Unlocked]\r\n");
    let mut h = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::message(&mut h, "'$H'|'$X' to unlock").unwrap();
    assert_eq!(h.as_str(), "[MSG:'$H'|'$X' to unlock]\r\n");
  }

  #[test]
  fn probe_report_push_line_wire_format() {
    // The immediate `[PRB:...]` push after a successful probe: machine position at the trigger instant, flag 1.
    // A non-zero A (90°) proves the rotary value-at-trigger field is rendered, not dropped.
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::probe_report(&mut s, &[-1.015, 0.0, -2.5, 90.0], true).unwrap();
    assert_eq!(s.as_str(), "[PRB:-1.015,0.000,-2.500,90.000:1]\r\n");
    // A failed probe (no contact) reports the end-of-travel position with flag 0.
    let mut f = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::probe_report(&mut f, &[0.0, 0.0, -5.0, 0.0], false).unwrap();
    assert_eq!(f.as_str(), "[PRB:0.000,0.000,-5.000,0.000:0]\r\n");
  }

  #[test]
  fn probe_response_truth_table() {
    // A triggered probe always succeeds, regardless of mode.
    assert_eq!(probe_response(true, false, true), ProbeResponse::Ok);
    assert_eq!(probe_response(true, false, false), ProbeResponse::Ok);
    // A non-triggered probe in a SILENT mode (G38.3/.5) still `ok`s — the sender checks the `[PRB:..:0]` flag.
    assert_eq!(probe_response(false, false, false), ProbeResponse::Ok);
    // A non-triggered probe in an ALARMING mode (G38.2/.4): ALARM:5 for no-contact, ALARM:4 for already-at-edge.
    assert_eq!(probe_response(false, false, true), ProbeResponse::Alarm(AlarmCode::ProbeFailContact));
    assert_eq!(probe_response(false, true, true), ProbeResponse::Alarm(AlarmCode::ProbeFailInitial));
    // The alarm codes are grbl's 4 (initial state) and 5 (no contact).
    assert_eq!(AlarmCode::ProbeFailInitial.code(), 4);
    assert_eq!(AlarmCode::ProbeFailContact.code(), 5);
  }

  #[test]
  fn last_probe_none_is_origin_failed() {
    let none = LastProbe::none();
    assert_eq!(none, LastProbe::default());
    assert_eq!(none.position_mm, [0.0, 0.0, 0.0, 0.0]);
    assert!(!none.success);
  }

  #[test]
  fn ngc_prb_line_reflects_last_probe_result() {
    // The `$#` `[PRB:]` line (line index 10) is driven by the stored last-probe result: feeding a real probe
    // position + success flag into the coordinate report makes `$#` show the triggered point and flag 1 (this
    // replaces the Phase-B zeros/flag-0 stub once the firmware bin fills `probe`/`probe_success` from LastProbe).
    let last = LastProbe { position_mm: [-1.015, 0.0, -2.5, 90.0], success: true };
    let report = CoordinateReport { probe: last.position_mm, probe_success: last.success, ..CoordinateReport::default() };
    let mut line = String::<RESPONSE_CAPACITY>::new();
    assert!(ResponseWriter::ngc_parameter_line(&mut line, &report, 10));
    assert_eq!(line.as_str(), "[PRB:-1.015,0.000,-2.500,90.000:1]\r\n");
  }

  #[test]
  fn help_line_is_emitted_for_bare_dollar() {
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::help(&mut s).unwrap();
    assert!(s.as_str().starts_with("[HLP:"));
    assert!(s.as_str().ends_with("]\r\n"));
  }

  // --- `$`-command classifier (the fake-ack hole, hardened) -----------------------------------------

  #[test]
  fn classifies_the_known_commands() {
    assert_eq!(SystemCommand::classify(b""), SystemCommand::Help);
    assert_eq!(SystemCommand::classify(b"$"), SystemCommand::SettingsDump);
    assert_eq!(SystemCommand::classify(b"I"), SystemCommand::BuildInfo { extended: false });
    assert_eq!(SystemCommand::classify(b"I+"), SystemCommand::BuildInfo { extended: true });
    assert_eq!(SystemCommand::classify(b"G"), SystemCommand::ParserState);
    assert_eq!(SystemCommand::classify(b"#"), SystemCommand::NgcParams);
    assert_eq!(SystemCommand::classify(b"X"), SystemCommand::Unlock);
    assert_eq!(SystemCommand::classify(b"C"), SystemCommand::ToggleCheck);
    assert_eq!(SystemCommand::classify(b"SLP"), SystemCommand::Sleep);
    assert_eq!(SystemCommand::classify(b"H"), SystemCommand::Home);
    assert_eq!(SystemCommand::classify(b"N"), SystemCommand::StartupQuery);
    assert_eq!(SystemCommand::classify(b"RST=$"), SystemCommand::RestoreSettings);
    assert_eq!(SystemCommand::classify(b"RST=#"), SystemCommand::RestoreParams);
    assert_eq!(SystemCommand::classify(b"RST=*"), SystemCommand::RestoreAll);
    assert_eq!(SystemCommand::classify(b"PBX"), SystemCommand::PbExport);
  }

  #[test]
  fn readonly_queries_are_classified_so_they_can_be_serviced_during_a_hold() {
    // A pause (M0/M1/M6) holds the consumer, but read-only `$`-QUERIES must still be answered during the hold
    // (grbl answers them while held). `is_readonly_query` is the gate: it is TRUE for the pure reporting commands
    // (no modal/planner/settings/coordinate side effect) and FALSE for every write / state-change / action, so the
    // hold loop services only the safe ones and leaves the rest queued for after resume.
    let readonly = [
      SystemCommand::Help,
      SystemCommand::SettingsDump,
      SystemCommand::BuildInfo { extended: false },
      SystemCommand::BuildInfo { extended: true },
      SystemCommand::ParserState,
      SystemCommand::NgcParams,
      SystemCommand::StartupQuery,
      SystemCommand::EnumSettings,
      SystemCommand::EnumSettingGroups,
      SystemCommand::EnumErrorCodes,
      SystemCommand::EnumAlarmCodes,
      SystemCommand::SettingDescription { id: 0 },
      SystemCommand::PbExport,
    ];
    for cmd in readonly {
      assert!(cmd.is_readonly_query(), "{cmd:?} must be a read-only query serviceable during a hold");
    }
    // Every write / state-change / action is NOT a read-only query: it must be held (deferred) during a pause.
    let not_readonly = [
      SystemCommand::Unlock,
      SystemCommand::ToggleCheck,
      SystemCommand::Sleep,
      SystemCommand::Home,
      SystemCommand::StartupSet { index: 0, gcode: b"G54" },
      SystemCommand::RestoreSettings,
      SystemCommand::RestoreParams,
      SystemCommand::RestoreAll,
      SystemCommand::SetSetting { body: b"100=250" },
      SystemCommand::PbImport { hex: b"DEAD" },
      SystemCommand::Unknown,
    ];
    for cmd in not_readonly {
      assert!(!cmd.is_readonly_query(), "{cmd:?} must NOT be serviced during a hold (it writes / changes state)");
    }
  }

  #[test]
  fn classifies_payload_commands_borrowing_their_text() {
    assert_eq!(
      SystemCommand::classify(b"N0=G54G20"),
      SystemCommand::StartupSet { index: 0, gcode: b"G54G20" },
    );
    assert_eq!(
      SystemCommand::classify(b"N1=G17"),
      SystemCommand::StartupSet { index: 1, gcode: b"G17" },
    );
    assert_eq!(SystemCommand::classify(b"PBX=DEADBEEF"), SystemCommand::PbImport { hex: b"DEADBEEF" });
    assert_eq!(SystemCommand::classify(b"100=250.000"), SystemCommand::SetSetting { body: b"100=250.000" });
    assert_eq!(SystemCommand::classify(b"0=10"), SystemCommand::SetSetting { body: b"0=10" });
  }

  #[test]
  fn unknown_commands_classify_as_unknown_not_fake_ack() {
    // The whole point of the hardening: an unmatched `$` command is Unknown (-> error:3), never a silent ok.
    for cmd in [
      &b"FOO"[..],
      &b"J=X10"[..],
      &b"abc=1"[..],
      &b"N2=G0"[..],
      &b"=5"[..],
      &b"RST=&"[..],
      &b"100x=5"[..],
    ] {
      assert_eq!(SystemCommand::classify(cmd), SystemCommand::Unknown, "{:?} must be Unknown", core::str::from_utf8(cmd));
    }
  }

  #[test]
  fn error_code_constants_match_grbl() {
    assert_eq!(ERROR_UNSUPPORTED_COMMAND, 3);
    assert_eq!(ERROR_HOMING_DISABLED, 5);
    assert_eq!(ERROR_NOT_IDLE, 8);
  }

  #[test]
  fn settings_write_allowed_only_idle_or_alarm() {
    // grbl's STATUS_IDLE_ERROR rule: settings writes pass in Normal (Idle-or-Run — the consumer gates live
    // motion) and Alarm (so a bad value can be fixed without `$X` first); every other latched mode rejects.
    assert!(ControlState::Normal.settings_write_allowed());
    assert!(ControlState::Alarm(AlarmCode::HomingRequired).settings_write_allowed());
    assert!(!ControlState::Hold(false).settings_write_allowed());
    assert!(!ControlState::Hold(true).settings_write_allowed());
    assert!(!ControlState::Jog.settings_write_allowed());
    assert!(!ControlState::Check.settings_write_allowed());
    assert!(!ControlState::Sleep.settings_write_allowed());
    assert!(!ControlState::Tool.settings_write_allowed());
  }

  // --- Phase E: feed/rapid/spindle overrides --------------------------------------------------------

  #[test]
  fn overrides_default_is_all_100_no_toggles() {
    let ov = Overrides::new();
    assert_eq!((ov.feed, ov.rapid, ov.spindle), (100, 100, 100));
    assert!(!ov.spindle_stop && !ov.flood && !ov.mist);
    assert_eq!(ov, Overrides::default());
  }

  #[test]
  fn feed_override_increment_decrement_reset_matrix() {
    let mut ov = Overrides::new();
    // +10 / -10 / +1 / -1 around the default.
    assert!(ov.apply(0x91)); // +10 -> 110
    assert_eq!(ov.feed, 110);
    assert!(ov.apply(0x92)); // -10 -> 100
    assert_eq!(ov.feed, 100);
    assert!(ov.apply(0x93)); // +1 -> 101
    assert_eq!(ov.feed, 101);
    assert!(ov.apply(0x94)); // -1 -> 100
    assert_eq!(ov.feed, 100);
    // Reset-100 from a non-default value.
    let _ = ov.apply(0x91);
    assert!(ov.apply(0x90)); // reset -> 100 (was 110, so it changed)
    assert_eq!(ov.feed, 100);
    // A reset that does not change (already 100) is reported as a no-op.
    assert!(!ov.apply(0x90));
  }

  #[test]
  fn feed_override_clamps_to_grbl_band() {
    let mut ov = Overrides::new();
    // Drive far below the floor: a run of -10s saturates at 10, not below.
    for _ in 0..50 {
      ov.apply(0x92);
    }
    assert_eq!(ov.feed, OVERRIDE_MIN_PCT);
    assert_eq!(ov.feed, 10);
    // Once at the floor, another -1 / -10 is a no-op (no change).
    assert!(!ov.apply(0x94));
    assert!(!ov.apply(0x92));
    // Drive far above the ceiling: a run of +10s saturates at 200.
    for _ in 0..50 {
      ov.apply(0x91);
    }
    assert_eq!(ov.feed, OVERRIDE_MAX_PCT);
    assert_eq!(ov.feed, 200);
    assert!(!ov.apply(0x91));
  }

  #[test]
  fn rapid_override_sets_discrete_values() {
    let mut ov = Overrides::new();
    assert!(ov.apply(0x96)); // 50%
    assert_eq!(ov.rapid, 50);
    assert!(ov.apply(0x97)); // 25%
    assert_eq!(ov.rapid, 25);
    assert!(ov.apply(0x95)); // 100%
    assert_eq!(ov.rapid, 100);
    // Re-setting the same value is a no-op.
    assert!(!ov.apply(0x95));
  }

  #[test]
  fn spindle_override_increment_decrement_reset_clamp() {
    let mut ov = Overrides::new();
    assert!(ov.apply(0x9A)); // +10 -> 110
    assert_eq!(ov.spindle, 110);
    assert!(ov.apply(0x9B)); // -10 -> 100
    assert_eq!(ov.spindle, 100);
    assert!(ov.apply(0x9C)); // +1 -> 101
    assert_eq!(ov.spindle, 101);
    assert!(ov.apply(0x9D)); // -1 -> 100
    assert_eq!(ov.spindle, 100);
    for _ in 0..50 {
      ov.apply(0x9B);
    }
    assert_eq!(ov.spindle, OVERRIDE_MIN_PCT);
    for _ in 0..50 {
      ov.apply(0x9A);
    }
    assert_eq!(ov.spindle, OVERRIDE_MAX_PCT);
    assert!(ov.apply(0x99)); // reset -> 100 (changed)
    assert_eq!(ov.spindle, 100);
  }

  #[test]
  fn spindle_stop_and_coolant_toggles_flip() {
    let mut ov = Overrides::new();
    assert!(ov.apply(0x9E)); // spindle-stop on
    assert!(ov.spindle_stop);
    assert!(ov.apply(0x9E)); // spindle-stop off
    assert!(!ov.spindle_stop);
    assert!(ov.apply(0xA0)); // flood on
    assert!(ov.flood);
    assert!(ov.apply(0xA1)); // mist on
    assert!(ov.mist);
    assert!(ov.apply(0xA0)); // flood off
    assert!(!ov.flood);
  }

  #[test]
  fn unmodeled_override_bytes_are_noops() {
    let mut ov = Overrides::new();
    // 0x98 (unused), 0xA2/0xA3/0xA4 (tool-change ack / probe-connected toggle) are not modeled by Phase E and
    // must leave the override state untouched, never panicking.
    for b in [0x98u8, 0xA2, 0xA3, 0xA4] {
      assert!(!ov.apply(b), "byte {b:#x} must be a no-op on the override model");
    }
    assert_eq!(ov, Overrides::new());
  }

  #[test]
  fn scaled_feed_applies_override_then_clamps_to_max_rate() {
    let mut ov = Overrides::new();
    // 100% leaves the programmed feed unchanged (well under the max-rate ceiling).
    assert!((ov.scaled_feed(1000.0, 6000.0) - 1000.0).abs() < 1e-3);
    // 50% halves it.
    for _ in 0..5 {
      ov.apply(0x92); // -10 five times -> 50%
    }
    assert_eq!(ov.feed, 50);
    assert!((ov.scaled_feed(1000.0, 6000.0) - 500.0).abs() < 1e-3);
    // 200% would double 4000 -> 8000, but the axis max-rate caps it at 6000 (scaling up never exceeds $110-112).
    let mut up = Overrides::new();
    for _ in 0..10 {
      up.apply(0x91); // +10 ten times -> 200%
    }
    assert_eq!(up.feed, 200);
    assert!((up.scaled_feed(4000.0, 6000.0) - 6000.0).abs() < 1e-3, "feed clamps to the axis max-rate");
    // With no ceiling (INFINITY) the 200% scale is unclamped.
    assert!((up.scaled_feed(4000.0, f32::INFINITY) - 8000.0).abs() < 1e-3);
  }

  #[test]
  fn scaled_rapid_applies_rapid_override() {
    let mut ov = Overrides::new();
    ov.apply(0x96); // 50%
    assert!((ov.scaled_rapid(6000.0) - 3000.0).abs() < 1e-3);
    ov.apply(0x97); // 25%
    assert!((ov.scaled_rapid(6000.0) - 1500.0).abs() < 1e-3);
    ov.apply(0x95); // 100%
    assert!((ov.scaled_rapid(6000.0) - 6000.0).abs() < 1e-3);
  }

  #[test]
  fn scaled_rpm_applies_spindle_override_and_stop() {
    let mut ov = Overrides::new();
    assert_eq!(ov.scaled_rpm(10000), 10000); // 100%
    ov.apply(0x9B); // -10 -> 90%
    assert_eq!(ov.scaled_rpm(10000), 9000);
    // Spindle-stop forces 0 regardless of the override percentage.
    ov.apply(0x9E);
    assert_eq!(ov.scaled_rpm(10000), 0);
    // Toggling stop back off restores the scaled value.
    ov.apply(0x9E);
    assert_eq!(ov.scaled_rpm(10000), 9000);
  }

  // --- Phase E: `Ov:` refresh cadence ---------------------------------------------------------------

  /// An `Ov:` refresh reporter at its firmware-seeded baseline (default overrides), exercising the same cadence
  /// the bin's `OV_REPORTER` cell drives.
  fn ov_reporter() -> RefreshReporter<Overrides> {
    RefreshReporter::new(Overrides::new())
  }

  #[test]
  fn ov_reporter_first_report_includes_then_suppresses() {
    let mut rep = ov_reporter();
    let ov = Overrides::new();
    assert!(rep.should_include(ov), "first report always includes Ov:");
    // The next `OV_REFRESH_PERIOD - 1` reports suppress it (no change).
    for _ in 0..(OV_REFRESH_PERIOD - 1) {
      assert!(!rep.should_include(ov));
    }
    // The periodic refresh re-includes it.
    assert!(rep.should_include(ov), "periodic refresh re-includes Ov:");
  }

  #[test]
  fn ov_reporter_includes_immediately_on_change() {
    let mut rep = ov_reporter();
    let mut ov = Overrides::new();
    assert!(rep.should_include(ov)); // first
    assert!(!rep.should_include(ov)); // suppressed
    ov.apply(0x91); // feed -> 110, a change
    assert!(rep.should_include(ov), "a changed override is reported in the very next report");
    assert!(!rep.should_include(ov), "and suppressed again afterward");
  }

  #[test]
  fn ov_reporter_reset_forces_next_include() {
    let mut rep = ov_reporter();
    let ov = Overrides::new();
    assert!(rep.should_include(ov));
    assert!(!rep.should_include(ov));
    rep.reset(Overrides::new());
    assert!(rep.should_include(ov), "after reset the next report re-includes Ov: (grbl's rule)");
  }

  // --- Phase E: `Pn:` letter assembly ---------------------------------------------------------------

  #[test]
  fn pin_report_omits_element_when_nothing_asserted() {
    let pins = PinReport::new_idle();
    assert!(!pins.any());
    let mut s = String::<32>::new();
    pins.write_letters(&mut s).unwrap();
    assert_eq!(s.as_str(), "", "no letters for a quiescent input set");
  }

  #[test]
  fn pin_report_assembles_letters_in_grbl_order() {
    // Probe + all three limits + door + hold + reset + cycle-start in the documented order: P X Y Z D H R S.
    let pins = PinReport {
      probe: true,
      limits: [true, true, true, false],
      door: true,
      hold: true,
      reset: true,
      cycle_start: true,
    };
    assert!(pins.any());
    let mut s = String::<32>::new();
    pins.write_letters(&mut s).unwrap();
    assert_eq!(s.as_str(), "PXYZDHRS");
  }

  #[test]
  fn pin_report_probe_only() {
    // The probe is the one pin wired today (Phase C); a Z-limit + probe example from the docs is `Pn:ZP`-ish,
    // but in grbl letter order probe precedes the limits, so a probe-only report is just `P`.
    let pins = PinReport { probe: true, ..PinReport::new_idle() };
    let mut s = String::<32>::new();
    pins.write_letters(&mut s).unwrap();
    assert_eq!(s.as_str(), "P");
  }

  #[test]
  fn pin_report_partial_limits_only() {
    let pins = PinReport { limits: [false, true, false, false], ..PinReport::new_idle() };
    let mut s = String::<32>::new();
    pins.write_letters(&mut s).unwrap();
    assert_eq!(s.as_str(), "Y");
  }

  #[test]
  fn pin_report_x_limit_only() {
    // The lowest limit bit on its own: the bin's `LIMIT_LEVELS` bit0 (X) asserted maps to a bare `X`.
    let pins = PinReport { limits: [true, false, false, false], ..PinReport::new_idle() };
    let mut s = String::<32>::new();
    pins.write_letters(&mut s).unwrap();
    assert_eq!(s.as_str(), "X");
  }

  #[test]
  fn pin_report_z_limit_only() {
    // The highest limit bit on its own (the common Z-probe / Z-min over-travel case): a bare `Z`.
    let pins = PinReport { limits: [false, false, true, false], ..PinReport::new_idle() };
    let mut s = String::<32>::new();
    pins.write_letters(&mut s).unwrap();
    assert_eq!(s.as_str(), "Z");
  }

  #[test]
  fn pin_report_x_and_z_limits_skip_y() {
    // A non-contiguous limit mask (X+Z, Y released) must emit the letters in axis order with no `Y` between them.
    let pins = PinReport { limits: [true, false, true, false], ..PinReport::new_idle() };
    let mut s = String::<32>::new();
    pins.write_letters(&mut s).unwrap();
    assert_eq!(s.as_str(), "XZ");
  }

  #[test]
  fn pin_report_probe_and_limits_combine_in_order() {
    // The probe plus two limits: the probe `P` precedes the limit letters in grbl's documented order.
    let pins = PinReport { probe: true, limits: [true, true, false, false], ..PinReport::new_idle() };
    let mut s = String::<32>::new();
    pins.write_letters(&mut s).unwrap();
    assert_eq!(s.as_str(), "PXY");
  }

  #[test]
  fn status_report_renders_multiple_limit_letters() {
    // Wire-level: a `Run` report with X+Z limits asserted (no probe) carries `Pn:XZ` in the documented slot,
    // after `Bf:` and before `Ov:`. This is the path a host (skirnir) parses to light its endstop indicators.
    let snap = MachineSnapshot {
      state: MachineState::Run,
      feed_mm_min: 500.0,
      spindle_rpm: 0,
      pins: PinReport { limits: [true, false, true, false], ..PinReport::new_idle() },
      include_ov: false,
      ..MachineSnapshot::idle()
    };
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::status_report(&mut s, &snap).unwrap();
    assert_eq!(
      s.as_str(),
      "<Run|MPos:0.000,0.000,0.000,0.000|FS:500,0|Bf:32,1024|Pn:XZ|WCO:0.000,0.000,0.000,0.000>\r\n",
    );
  }

  // --- Phase E: status report carries override-scaled FS:, Pn:, and Ov: -----------------------------

  #[test]
  fn status_report_includes_pn_and_ov_with_scaled_fs() {
    // A running report with the probe asserted, override-scaled FS:, and the Ov: element included this cycle.
    let mut ov = Overrides::new();
    for _ in 0..5 {
      ov.apply(0x91); // feed -> 150
    }
    assert_eq!(ov.feed, 150);
    let snap = MachineSnapshot {
      state: MachineState::Run,
      mpos_mm: [1.0, 2.0, 3.0, 0.0],
      wco_mm: [0.0, 0.0, 0.0, 0.0],
      position_report: PositionReport::Machine,
      include_wco: false,
      // The bin computes these as the REALIZED feed/spindle (programmed × override); here 1000 mm/min at 150%
      // realized to 1500, and 12000 RPM at the default 100% spindle override.
      feed_mm_min: 1500.0,
      spindle_rpm: 12000,
      planner_blocks_free: 30,
      rx_bytes_free: 1020,
      pins: PinReport { probe: true, ..PinReport::new_idle() },
      overrides: ov,
      include_ov: true,
    };
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::status_report(&mut s, &snap).unwrap();
    assert_eq!(
      s.as_str(),
      "<Run|MPos:1.000,2.000,3.000,0.000|FS:1500,12000|Bf:30,1020|Pn:P|Ov:150,100,100>\r\n",
    );
  }

  #[test]
  fn status_report_omits_pn_and_ov_when_suppressed() {
    // Idle, nothing asserted, Ov: on the suppressed cadence: neither Pn: nor Ov: appears.
    let snap = MachineSnapshot { include_ov: false, ..MachineSnapshot::idle() };
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::status_report(&mut s, &snap).unwrap();
    assert_eq!(
      s.as_str(),
      "<Idle|MPos:0.000,0.000,0.000,0.000|FS:0,0|Bf:32,1024|WCO:0.000,0.000,0.000,0.000>\r\n",
    );
  }

  // --- Phase F: runtime enumeration ($ES/$EG/$EE/$EA/$SED) + NEWOPT + auto-report toggle ------------

  #[test]
  fn build_info_newopt_advertises_enums_and_sed() {
    // The extended `$I+` NEWOPT line must advertise ENUMS (so senders query `$ES`/`$EG`/`$EE`/`$EA`) and SED
    // (so they query `$SED`), keeping the existing RT+ flag for the top-bit real-time forms.
    let mut s = String::<256>::new();
    ResponseWriter::build_info(&mut s, true).unwrap();
    assert!(s.contains("[NEWOPT:ENUMS,RT+,SED]\r\n"), "NEWOPT must advertise ENUMS and SED, got {:?}", s.as_str());
  }

  #[test]
  fn error_code_line_wire_format() {
    // Byte-exact `[ERRORCODE:id|name|description]` for a sampled code (1 — the grbl example).
    let code = ERROR_CODES.iter().find(|c| c.id == 1).expect("error 1 defined");
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::error_code_line(&mut s, code).unwrap();
    assert_eq!(
      s.as_str(),
      "[ERRORCODE:1|Expected command letter|G-code words consist of a letter and a value. Letter was not found.]\r\n",
    );
  }

  #[test]
  fn error_context_line_wire_format() {
    // The `[MSG:error:N <name>]` context push a plain terminal sees just before the bare `error:N`.
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::error_context(&mut s, 21).unwrap();
    assert_eq!(s.as_str(), "[MSG:error:21 Modal group violation]\r\n");
  }

  #[test]
  fn error_context_writes_nothing_for_an_unknown_code() {
    // A code with no `ERROR_CODES` row writes nothing, so the caller emits no stray blank line before `error:N`.
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::error_context(&mut s, 250).unwrap();
    assert!(s.is_empty(), "an unknown code yields an empty context buffer, got {:?}", s.as_str());
  }

  #[test]
  fn alarm_context_line_wire_format() {
    // The `[MSG:ALARM:N <name>]` context push emitted alongside an `ALARM:N`.
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::alarm_context(&mut s, AlarmCode::HardLimit).unwrap();
    assert_eq!(s.as_str(), "[MSG:ALARM:1 Hard limit]\r\n");
  }

  #[test]
  fn error_codes_are_ascending_unique_and_cover_emitted_codes() {
    // The `$EE` table is the single error authority: ascending, no duplicate ids, and it must include every
    // code the firmware actually emits (parser, settings, protocol/$-dispatch).
    let mut prev: Option<u8> = None;
    for code in ERROR_CODES {
      if let Some(p) = prev {
        assert!(code.id > p, "ERROR_CODES must be strictly ascending: {p} then {}", code.id);
      }
      prev = Some(code.id);
    }
    let has = |id: u8| ERROR_CODES.iter().any(|c| c.id == id);
    // Protocol/$-dispatch codes.
    assert!(has(ERROR_LINE_OVERFLOW), "error:15 (line overflow) must be enumerated");
    assert!(has(ERROR_UNSUPPORTED_COMMAND), "error:3 (unsupported command) must be enumerated");
    assert!(has(ERROR_HOMING_DISABLED), "error:5 (homing disabled) must be enumerated");
    // The `firmware` comms layer rejects GCode while in an alarm/jog state with `error:9` (its `ERROR_LOCKED`).
    assert!(has(9), "error:9 (G-code state lock) must be enumerated");
    // GCode parser codes — every code GcodeError::code() can return (1, 2, 20, 21, 22, 26).
    for &code in &[1u8, 2, 20, 21, 22, 26] {
      assert!(has(code), "GCode error:{code} must be enumerated");
    }
    // Planner codes — every code PlannerError::code() can surface to the host (33 invalid arc, 15 jog travel).
    for &code in &[33u8, 15] {
      assert!(has(code), "planner error:{code} must be enumerated");
    }
  }

  #[test]
  fn alarm_code_line_wire_format() {
    // Byte-exact `[ALARMCODE:id|name|description]` for ALARM:1 (the grbl example).
    let mut s = String::<RESPONSE_CAPACITY>::new();
    ResponseWriter::alarm_code_line(&mut s, AlarmCode::HardLimit).unwrap();
    assert_eq!(
      s.as_str(),
      "[ALARMCODE:1|Hard limit|Hard limit has been triggered. Machine position is likely lost due to sudden halt. Re-homing is highly recommended.]\r\n",
    );
  }

  #[test]
  fn alarm_all_covers_every_code_with_name_and_description() {
    // `$EA` enumerates `AlarmCode::ALL`; every defined alarm code must appear exactly once with non-empty
    // name/description, and the codes must be the canonical grbl numbers (1,2,3,4,5,8,10,11,17).
    let codes: StdVec<u8> = AlarmCode::ALL.iter().map(|a| a.code()).collect();
    assert_eq!(codes, std::vec![1, 2, 3, 4, 5, 8, 10, 11, 17]);
    for alarm in AlarmCode::ALL {
      assert!(!alarm.name().is_empty(), "alarm {} has a name", alarm.code());
      assert!(!alarm.description().is_empty(), "alarm {} has a description", alarm.code());
    }
  }

  #[test]
  fn homing_fail_alarm_is_recoverable_code_8() {
    // ALARM:8 (homing fail) is recoverable, not a locked critical alarm: `$X` clears it and the prompt is
    // "Reset to continue" (the operator checks the switch/wiring and retries `$H`), distinct from the locked
    // hard/soft-limit/e-stop codes.
    assert_eq!(AlarmCode::HomingFail.code(), 8);
    assert!(!AlarmCode::HomingFail.is_locked());
    assert_eq!(AlarmCode::HomingFail.unlock_hint(), "Reset to continue");
    // `$X` clears a non-locked alarm to Normal.
    assert_eq!(ControlState::Alarm(AlarmCode::HomingFail).unlock(), (ControlState::Normal, UnlockOutcome::Unlocked));
  }

  #[test]
  fn enum_system_commands_classify_and_unknown_variants_error() {
    // The `$E*` enumeration commands classify to their variants; an unknown `$E*` (e.g. `$EX`) is NOT faked to
    // an ack — it classifies to `Unknown`, which the bin answers with `error:3`.
    assert_eq!(SystemCommand::classify(b"ES"), SystemCommand::EnumSettings);
    assert_eq!(SystemCommand::classify(b"EG"), SystemCommand::EnumSettingGroups);
    assert_eq!(SystemCommand::classify(b"EE"), SystemCommand::EnumErrorCodes);
    assert_eq!(SystemCommand::classify(b"EA"), SystemCommand::EnumAlarmCodes);
    assert_eq!(SystemCommand::classify(b"SED=0"), SystemCommand::SettingDescription { id: 0 });
    assert_eq!(SystemCommand::classify(b"SED=481"), SystemCommand::SettingDescription { id: 481 });
    // Unknown / malformed variants.
    assert_eq!(SystemCommand::classify(b"EX"), SystemCommand::Unknown);
    assert_eq!(SystemCommand::classify(b"ESX"), SystemCommand::Unknown);
    assert_eq!(SystemCommand::classify(b"SED="), SystemCommand::Unknown);
    assert_eq!(SystemCommand::classify(b"SED=abc"), SystemCommand::Unknown);
  }
}
