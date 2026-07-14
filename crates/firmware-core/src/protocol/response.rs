//! The `ResponseWriter` formatting engine (the host<->firmware wire contract).

use super::*;
use core::fmt::Write as _;
use heapless::String;

/// Formatting failure: the destination buffer was too small to hold the rendered response. Callers size
/// their buffers from the constants below, so this is a programming error rather than a runtime
/// condition, but it is surfaced as a `Result` to honor the firmware-core no-`unwrap` rule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FmtError;

/// Stateless formatters for every protocol response. Each writes a fully-formed response (including its
/// trailing line terminator where one is implied) into a caller-provided [`String`], returning
/// [`FmtError`] only if the buffer is too small. Keeping these pure and buffer-borrowing makes the wire
/// format host-testable byte-for-byte against the same vectors `skirnir` parses.
pub struct ResponseWriter;

impl ResponseWriter {
  /// The grbl welcome banner, emitted on boot and on every soft reset. A host treats receipt of this as
  /// "controller reset and ready". The grbl-compatible form (`Grbl 1.1f [...]`) is used so legacy and
  /// grblHAL senders alike recognize readiness; the extended grblHAL identity is exposed via `$I+`.
  pub fn banner<const N: usize>(out: &mut String<N>) -> Result<(), FmtError> {
    write!(out, "Grbl {VERSION} ['$' for help]\r\n").map_err(|_| FmtError)
  }

  /// The `ok` response — emitted exactly once per accepted line consumed downstream.
  pub fn ok<const N: usize>(out: &mut String<N>) -> Result<(), FmtError> {
    out.push_str("ok\r\n").map_err(|_| FmtError)
  }

  /// The `error:N` response — emitted exactly once for a rejected line.
  pub fn error<const N: usize>(out: &mut String<N>, code: u8) -> Result<(), FmtError> {
    write!(out, "error:{code}\r\n").map_err(|_| FmtError)
  }

  /// An `ALARM:N` push line, emitted on entering an alarm so a host detects the halt and stops streaming.
  pub fn alarm<const N: usize>(out: &mut String<N>, code: AlarmCode) -> Result<(), FmtError> {
    write!(out, "ALARM:{}\r\n", code.code()).map_err(|_| FmtError)
  }

  /// The `[PRB:x,y,z,a:success]` probe-result push line (DOC-09, `docs/gcode-streaming.md` §9). Emitted
  /// immediately after a `G38.x` cycle completes so a height-mapping / touch-off sender reads the probed point
  /// without polling `$#`. `position` is the MACHINE position at the trigger instant in mm; `success` is the
  /// contact flag (`1` = the expected edge was seen, `0` = it was not). The same value is retained for `$#`'s
  /// `[PRB:]` line. All [`AXIS_COUNT`] axes are reported (the rotary A value-at-trigger included), matching the
  /// four-field live `MPos:`/`WPos:` status — grbl's `[PRB:]` is N_AXIS-wide (`[PRB:-1.015,0.000,0.000,0.000:1]`).
  pub fn probe_report<const N: usize>(
    out: &mut String<N>,
    position: &[f32; AXIS_COUNT],
    success: bool,
  ) -> Result<(), FmtError> {
    write!(out, "[PRB:").map_err(|_| FmtError)?;
    write_axes_csv(out, position).map_err(|_| FmtError)?;
    write!(out, ":{}]\r\n", success as u8).map_err(|_| FmtError)
  }

  /// One `[ERRORCODE:<id>|<name>|<description>]` line for the `$EE` enumeration, rendered from an
  /// [`ErrorCode`] row. The firmware loops [`ERROR_CODES`] and emits one line per row through the single USB
  /// writer (line-by-line, never one giant buffer), then a terminating `ok`. The form matches grblHAL's
  /// published example (`[ERRORCODE:1|Expected command letter|G-code words consist of ...]`).
  pub fn error_code_line<const N: usize>(out: &mut String<N>, code: &ErrorCode) -> Result<(), FmtError> {
    write!(out, "[ERRORCODE:{}|{}|{}]\r\n", code.id, code.name, code.description).map_err(|_| FmtError)
  }

  /// One `[ALARMCODE:<id>|<name>|<description>]` line for the `$EA` enumeration, rendered from an [`AlarmCode`].
  /// The firmware loops [`AlarmCode::ALL`] and emits one line per code through the single USB writer, then a
  /// terminating `ok`. The form matches grblHAL's published example (`[ALARMCODE:1|Hard limit|Hard limit has
  /// been triggered. ...]`).
  pub fn alarm_code_line<const N: usize>(out: &mut String<N>, code: AlarmCode) -> Result<(), FmtError> {
    write!(out, "[ALARMCODE:{}|{}|{}]\r\n", code.code(), code.name(), code.description()).map_err(|_| FmtError)
  }

  /// A `[MSG:<text>]` bracketed push message (e.g. `[MSG:Caution: Unlocked]`, `[MSG:'$H'|'$X' to unlock]`,
  /// `[MSG:Enabled]`). The caller supplies the inner text; this wraps it in the grbl `[MSG:...]` envelope.
  pub fn message<const N: usize>(out: &mut String<N>, text: &str) -> Result<(), FmtError> {
    write!(out, "[MSG:{text}]\r\n").map_err(|_| FmtError)
  }

  /// The human-readable INNER text of the `[MSG:..]` an `M6` manual tool change pushes when it holds (the firmware
  /// wraps this with [`message`](ResponseWriter::message)). It NAMES the committed tool so a bare-terminal operator
  /// knows which tool to insert: `Manual tool change to T<n> — swap tool, then cycle-start (~) to resume` for a
  /// selected tool, or `Manual tool change (no tool selected) — …` for `T0` (a bare `M6` with no pending `T`).
  ///
  /// This is a human-readable PUSH only — NOT a wire-format contract. `skirnir` sources the tool number from the
  /// streamed program independently and does NOT parse this text, so the exact wording is free to change; the test
  /// pins it only so the tool number is provably present. Kept pure (a `no_std` buffer write) so it is host-tested
  /// byte-for-byte rather than living in the untestable async `send_message` wiring.
  pub fn tool_change_message<const N: usize>(out: &mut String<N>, tool: u16) -> Result<(), FmtError> {
    if tool == 0 {
      out
        .push_str("Manual tool change (no tool selected) \u{2014} swap tool, then cycle-start (~) to resume")
        .map_err(|_| FmtError)
    } else {
      write!(out, "Manual tool change to T{tool} \u{2014} swap tool, then cycle-start (~) to resume")
        .map_err(|_| FmtError)
    }
  }

  /// A `[MSG:error:N <name>]` context push line, emitted just BEFORE an `error:N` response so a plain terminal
  /// (one that does not fetch the `$EE` table) still sees what the rejection means. This writes nothing for a
  /// code with no enumerated name — the caller checks for an empty buffer and skips the enqueue, so a stray
  /// blank line is never emitted. It is a push message: a sender that decodes the code itself ignores it, and
  /// the byte-exact `error:N` that follows remains the sole flow-control response.
  pub fn error_context<const N: usize>(out: &mut String<N>, code: u8) -> Result<(), FmtError> {
    match error_name(code) {
      Some(name) => write!(out, "[MSG:error:{code} {name}]\r\n").map_err(|_| FmtError),
      None => Ok(()),
    }
  }

  /// A `[MSG:ALARM:N <name>]` context push line, emitted alongside an `ALARM:N` so a plain terminal sees what
  /// halted the machine. grbl's unlock/continue prompt (`[MSG:'$H'|'$X' to unlock]` / `[MSG:Reset to
  /// continue]`) still follows as its own separate `[MSG:]`.
  pub fn alarm_context<const N: usize>(out: &mut String<N>, code: AlarmCode) -> Result<(), FmtError> {
    write!(out, "[MSG:ALARM:{} {}]\r\n", code.code(), code.name()).map_err(|_| FmtError)
  }

  /// The bare-`$` grbl help line, listing the system commands a sender may probe. grbl answers `$` with this
  /// one-liner rather than a bare `ok`, so a sender's `$`-help probe gets the documented response.
  pub fn help<const N: usize>(out: &mut String<N>) -> Result<(), FmtError> {
    out
      .push_str("[HLP:$$ $# $G $I $N $X $H $C $SLP $RST=$ ~ ! ?]\r\n")
      .map_err(|_| FmtError)
  }

  /// A `<...>` status report rendered from a [`MachineSnapshot`]. Form:
  /// `<State|{MPos|WPos}:x,y,z|FS:feed,rpm|Bf:blocks,bytes{|WCO:x,y,z}>`. Substates are appended for
  /// `Hold`/`Alarm`. The position element is `MPos:` or `WPos:` (= `MPos − WCO`) per
  /// [`position_report`](MachineSnapshot::position_report) — never both, matching grbl 1.1+. The `WCO:`
  /// element is appended only when [`include_wco`](MachineSnapshot::include_wco) is set (the change/periodic
  /// cadence — see [`RefreshReporter`]). The element order follows the documented grblHAL order (State first,
  /// position second) so senders that position-parse do not break.
  pub fn status_report<const N: usize>(out: &mut String<N>, snap: &MachineSnapshot) -> Result<(), FmtError> {
    out.push('<').map_err(|_| FmtError)?;
    out.push_str(snap.state.token()).map_err(|_| FmtError)?;
    match snap.state {
      MachineState::Hold(in_progress) => {
        write!(out, ":{}", in_progress as u8).map_err(|_| FmtError)?;
      }
      MachineState::Alarm(code) => {
        write!(out, ":{code}").map_err(|_| FmtError)?;
      }
      _ => {}
    }
    // The position element: `MPos:` is the machine position verbatim; `WPos:` is `MPos − WCO` per axis. grbl
    // reports exactly one of the two; the host derives the other from the `WCO:` element.
    let (label, position) = match snap.position_report {
      PositionReport::Machine => ("MPos", snap.mpos_mm),
      PositionReport::Work => {
        let mut work = [0.0f32; AXIS_COUNT];
        for ((slot, &mpos), &wco) in work.iter_mut().zip(snap.mpos_mm.iter()).zip(snap.wco_mm.iter()) {
          *slot = mpos - wco;
        }
        ("WPos", work)
      }
    };
    // Emit one comma-separated field per axis (grblHAL reports N axes: `MPos:x,y,z,a`). The A field is the
    // rotary position in degrees (DOC-10); the loop widens with `AXIS_COUNT`.
    write!(out, "|{label}:").map_err(|_| FmtError)?;
    write_axes_csv(out, &position).map_err(|_| FmtError)?;
    write!(
      out,
      "|FS:{:.0},{}|Bf:{},{}",
      snap.feed_mm_min, snap.spindle_rpm, snap.planner_blocks_free, snap.rx_bytes_free,
    )
    .map_err(|_| FmtError)?;
    // `Pn:` — asserted input pins, in grbl's signal-letter order. Omitted entirely when nothing is asserted
    // (grbl's rule), so a quiescent report carries no empty `Pn:` element.
    if snap.pins.any() {
      out.push_str("|Pn:").map_err(|_| FmtError)?;
      snap.pins.write_letters(out)?;
    }
    if snap.include_wco {
      out.push_str("|WCO:").map_err(|_| FmtError)?;
      write_axes_csv(out, &snap.wco_mm).map_err(|_| FmtError)?;
    }
    // `Ov:` — feed,rapid,spindle override percentages, on the change/periodic cadence (mirroring `WCO:`), so it
    // is not emitted in every report. Placed after `WCO:` per the documented grblHAL element order.
    if snap.include_ov {
      write!(out, "|Ov:{},{},{}", snap.overrides.feed, snap.overrides.rapid, snap.overrides.spindle)
        .map_err(|_| FmtError)?;
    }
    out.push_str(">\r\n").map_err(|_| FmtError)
  }

  /// The `$I` (or `$I+` when `extended`) build-info response. The base report emits `[VER:]` and
  /// `[OPT:]`; the extended report adds the grblHAL `[AXS:]`, `[NEWOPT:]`, `[FIRMWARE:]`, and `[SIGNALS:]`
  /// lines so a sender can detect an extended controller and its capabilities. The `[OPT:]` fields are, in
  /// order: options string, block buffer size, RX buffer size, axis count, tool-table entries — emitted in
  /// exactly the documented order so senders that position-parse OPT do not mis-read the buffer sizes. The
  /// live `[DRIVER:]` line is emitted separately by [`ResponseWriter::driver_info`] (it needs hardware
  /// state). The caller appends `ok`.
  pub fn build_info<const N: usize>(out: &mut String<N>, extended: bool) -> Result<(), FmtError> {
    write!(out, "[VER:{VERSION}.20260616:]\r\n").map_err(|_| FmtError)?;
    write!(
      out,
      "[OPT:VNMSL,{},{},{},0]\r\n",
      BLOCK_BUFFER_SIZE, RX_BUFFER_SIZE, AXIS_COUNT,
    )
    .map_err(|_| FmtError)?;
    if extended {
      // `[AXS:<n>:<letters>]` — derive both the count and the letters from the kinematics constants so the
      // line can never advertise a letter set that disagrees with [`AXIS_COUNT`] (no hardcoded `XYZA`).
      write!(out, "[AXS:{AXIS_COUNT}:").map_err(|_| FmtError)?;
      for letter in AXIS_LETTERS {
        out.push(letter).map_err(|_| FmtError)?;
      }
      out.push_str("]\r\n").map_err(|_| FmtError)?;
      // `ENUMS` advertises the runtime enumeration commands (`$ES`/`$EG`/`$EE`/`$EA`) so a sender builds its
      // settings/error/alarm UI from the controller instead of hardcoding; `RT+` advertises the top-bit-set
      // real-time command forms this module classifies; `SED` advertises the `$SED=<n>` per-setting description
      // command (Phase F). A sender reads these to know it may query the enumerations on connect.
      out.push_str("[NEWOPT:ENUMS,RT+,SED]\r\n").map_err(|_| FmtError)?;
      out.push_str("[FIRMWARE:grblHAL]\r\n").map_err(|_| FmtError)?;
      // `[SIGNALS:<letters>]` — the input signals this build supports, in the same letter codes as the
      // realtime `Pn:` status field. Rendered from [`SIGNAL_CAPABILITIES`] via the SAME letter assembly the
      // `Pn:` element uses, so the two cannot drift. This is a compile-time capability set (which inputs the
      // firmware can read), independent of whether any is currently asserted.
      out.push_str("[SIGNALS:").map_err(|_| FmtError)?;
      SIGNAL_CAPABILITIES.write_letters(out)?;
      out.push_str("]\r\n").map_err(|_| FmtError)?;
    }
    Ok(())
  }

  /// The `$I+` `[DRIVER:]` report: the stepper-driver identity plus the live per-axis TMC2209 bus health.
  /// Emitted only by the extended `$I+` response (after [`build_info`]) because it carries runtime hardware
  /// state — the `firmware` bin samples the TMC manager's per-axis online flags and hands them in via
  /// [`DriverStatus`]; this formatter stays pure and host-testable. First line is the fixed `[DRIVER:TMC2209]`
  /// identity so a sender that only string-matches the driver name still finds it. The second line is the
  /// per-axis breakdown using the [`AXIS_LETTERS`] designations and an `ok`/`--` state (UART responding vs.
  /// not detected). Before the init pass has run, `initialized` is `false` and the breakdown is replaced with
  /// `init pending` so a query during the brief boot window does not falsely report every driver absent. The
  /// caller appends `ok` for the consumed `$I+` line (this writes no terminator of its own).
  pub fn driver_info<const N: usize>(out: &mut String<N>, status: &DriverStatus) -> Result<(), FmtError> {
    out.push_str("[DRIVER:TMC2209]\r\n").map_err(|_| FmtError)?;
    if !status.initialized {
      return out.push_str("[DRIVER:TMC2209 init pending]\r\n").map_err(|_| FmtError);
    }
    out.push_str("[DRIVER:TMC2209").map_err(|_| FmtError)?;
    for (axis, &letter) in AXIS_LETTERS.iter().enumerate() {
      // A present/communicating driver renders `<letter>:ok`; an absent one (no UART reply at init) `<letter>:--`.
      let state = if status.online.get(axis).copied().unwrap_or(false) { "ok" } else { "--" };
      write!(out, " {letter}:{state}").map_err(|_| FmtError)?;
    }
    out.push_str("]\r\n").map_err(|_| FmtError)
  }

  /// The `$I+` echo-vs-reply presence-probe diagnostic: `[MSG:TMC-PROBE X:<tok> Y:<tok> Z:<tok> A:<tok>]`,
  /// rendered from a [`DriverProbe`]. The single-wire TMC2209 bus reads back the MCU's OWN transmitted bytes
  /// (the echo) before the driver's reply, so a per-node `no-echo`/`no-reply`/`ok` token localizes a whole-bus
  /// no-reply WITHOUT a scope: `no-echo` means the RX path never saw the shared pin (firmware/pin-matrix
  /// fault), `no-reply` means routing is fine and the driver simply never answered (hardware/line fault), and
  /// `ok` means the node responded. Emitted alongside the `[DRIVER:]` lines only when the caller decides it is
  /// worth reporting (see [`DriverProbe::should_report`]); this formatter stays pure and always renders when
  /// called. The letters derive from [`AXIS_LETTERS`] so the probe report and the `[DRIVER:]`/`[AXS:]` reports
  /// can never drift apart. No terminator beyond the line itself is written.
  pub fn driver_probe<const N: usize>(out: &mut String<N>, probe: &DriverProbe) -> Result<(), FmtError> {
    out.push_str("[MSG:TMC-PROBE").map_err(|_| FmtError)?;
    for (axis, &letter) in AXIS_LETTERS.iter().enumerate() {
      let stage = probe.stages.get(axis).copied().unwrap_or_default();
      write!(out, " {letter}:").map_err(|_| FmtError)?;
      // Enrich the captured `InitError` node's token with the failing register/op/kind; other nodes (and every
      // categorized outcome) render their self-describing token.
      match (stage, probe.init_failure) {
        (TmcProbeStage::InitError, Some(failure)) if failure.axis == axis => failure.write_token(out)?,
        _ => stage.write_token(out)?,
      }
    }
    out.push_str("]\r\n").map_err(|_| FmtError)
  }

  /// The `$I+` `[MSG:TMC-IOIN <letter>:0xNN 0xNN …]` framing dump: the raw 8-byte `IOIN` reply of a node whose
  /// presence read hit a [`TmcProbeStage::DecodeError`], rendered as space-separated lowercase hex bytes so the
  /// framing / echo-reply alignment can be eyeballed over serial. Emitted alongside the `[MSG:TMC-PROBE …]` line
  /// only when [`DriverProbe::raw_ioin`] is populated. The letter derives from [`AXIS_LETTERS`]; an
  /// out-of-range axis renders `?`. No terminator beyond the line itself is written.
  pub fn driver_ioin_raw<const N: usize>(out: &mut String<N>, capture: &IoinRawCapture) -> Result<(), FmtError> {
    let letter = AXIS_LETTERS.get(capture.axis).copied().unwrap_or('?');
    write!(out, "[MSG:TMC-IOIN {letter}:").map_err(|_| FmtError)?;
    for (index, byte) in capture.bytes.iter().enumerate() {
      if index > 0 {
        out.push_str(" ").map_err(|_| FmtError)?;
      }
      write!(out, "{byte:#04x}").map_err(|_| FmtError)?;
    }
    out.push_str("]\r\n").map_err(|_| FmtError)
  }

  /// The `$I+` boot loopback self-test line: `[MSG:TMC-LOOPBACK sent:8 got:N match:M/8 err:<none|ovf|glt|frm|par>]`,
  /// rendered from a [`LoopbackReport`]. `sent` is the fixed [`TMC_LOOPBACK_PATTERN`] length; `got` the bytes
  /// echoed back; `match` the positions that matched; `err` the first RX-error variant (or `none`). Proves the
  /// MCU TX+RX path and line levels without a scope (see [`LoopbackReport`]). Pure and host-testable; the caller
  /// emits it only after the test has run.
  pub fn loopback<const N: usize>(out: &mut String<N>, report: &LoopbackReport) -> Result<(), FmtError> {
    let sent = TMC_LOOPBACK_PATTERN.len();
    let err = report.err.map(RxErrorKind::token).unwrap_or("none");
    write!(out, "[MSG:TMC-LOOPBACK sent:{sent} got:{} match:{}/{sent} err:{err}]\r\n", report.got, report.matched)
      .map_err(|_| FmtError)
  }

  /// The `$I+` bus-margin meter: `[MSG:TMC-BUS X:<fail>/<total> Y:… Z:… A:…]`, rendered from a [`BusStats`]
  /// snapshot — each axis's failed vs attempted datagram exchanges since boot (see [`BusStats`] for what counts
  /// and why this grades a marginal bus). The letters derive from [`AXIS_LETTERS`] so this report can never
  /// drift from the `[DRIVER:]`/`[MSG:TMC-PROBE]` lines. Pure and always renders when called; the caller gates
  /// emission on [`BusStats::any_attempted`]. No terminator beyond the line itself is written.
  pub fn tmc_bus_stats<const N: usize>(out: &mut String<N>, stats: &BusStats) -> Result<(), FmtError> {
    out.push_str("[MSG:TMC-BUS").map_err(|_| FmtError)?;
    for (axis, &letter) in AXIS_LETTERS.iter().enumerate() {
      write!(out, " {letter}:{}/{}", stats.fail[axis], stats.total[axis]).map_err(|_| FmtError)?;
    }
    out.push_str("]\r\n").map_err(|_| FmtError)
  }

  /// The `$G` parser-state report: `[GC:<modal words>]`, rendered from a live [`ParserSnapshot`]. The
  /// motion (`G0`–`G3`), units (`G20`/`G21`), distance (`G90`/`G91`), feed mode (`G93`/`G94`), work coordinate
  /// (`G54`–`G59`), tool-offset mode (`G43.1`/`G49`), spindle state (`M3`/`M4`/`M5`), feed (`F`), and spindle speed
  /// (`S`), plane (`G17`/`G18`/`G19`), and coolant (`M7`/`M8`/`M9`) words reflect the snapshot; only `T0` (tool) is
  /// fixed where it is not yet commandable, but it is emitted so the line is a complete grbl-faithful report. Feed
  /// is written with a minimal decimal (no trailing `.0` for whole values) to match grbl's compact form.
  pub fn parser_state<const N: usize>(out: &mut String<N>, snap: &ParserSnapshot) -> Result<(), FmtError> {
    // The active work-coordinate word: G54..G59 from the modal WCS index (clamped defensively to G54 for an
    // out-of-range index, which the parser never produces).
    let wcs_word = WCS_TAGS.get(snap.wcs).copied().unwrap_or("G54");
    // The tool-offset-mode word: G43.1 when a dynamic TLO is active, else G49.
    let tlo_word = if snap.tlo_active { "G43.1" } else { "G49" };
    // The coolant word(s) (modal group 8): M9 when both off, else the active circuit word(s) — `M7`, `M8`, or
    // `M7 M8` (both active at once, grbl's group-8 output). A small fixed buffer holds the longest form (`M7 M8`).
    let mut coolant_word = String::<8>::new();
    match (snap.coolant.mist, snap.coolant.flood) {
      (false, false) => coolant_word.push_str("M9").map_err(|_| FmtError)?,
      (true, false) => coolant_word.push_str("M7").map_err(|_| FmtError)?,
      (false, true) => coolant_word.push_str("M8").map_err(|_| FmtError)?,
      (true, true) => coolant_word.push_str("M7 M8").map_err(|_| FmtError)?,
    }
    write!(
      out,
      "[GC:{} {} {} {} {} {} {} {} T{} {} F",
      snap.motion.word(),
      wcs_word,
      snap.plane.word(),
      snap.units.word(),
      snap.distance.word(),
      snap.feed_mode.word(),
      snap.spindle.word(),
      coolant_word.as_str(),
      snap.tool,
      tlo_word,
    )
    .map_err(|_| FmtError)?;
    write_minimal_f32(out, snap.feed)?;
    write!(out, " S{}]\r\n", snap.spindle_rpm).map_err(|_| FmtError)
  }
}

impl ResponseWriter {
  /// Render ONE line of the `$#` NGC-parameters block (`index` in `0..`[`NGC_PARAMETER_LINES`]) into `out`,
  /// CRLF-terminated, from a [`CoordinateReport`]. Lines 0-5 are `[G54:..]`..`[G59:..]`, 6 is `[G28:..]`, 7 is
  /// `[G30:..]`, 8 is `[G92:..]`, 9 is `[TLO:z]` (a single Z scalar, grbl's legacy form), 10 is
  /// `[PRB:x,y,z,a:flag]`. Every coordinate line reports all [`AXIS_COUNT`] axes (the rotary A included), matching
  /// the four-field live status. Returns `true` on success, `false` for an out-of-range `index` or (never, with a
  /// correctly sized buffer) a capacity failure. Emitting per-line lets the bin reuse one small [`Response`]
  /// buffer and stream the block through the single USB writer, exactly as `$$` does.
  pub fn ngc_parameter_line<const N: usize>(out: &mut String<N>, report: &CoordinateReport, index: usize) -> bool {
    let coord_line = |out: &mut String<N>, tag: &str, v: &[f32; AXIS_COUNT]| -> Result<(), core::fmt::Error> {
      write!(out, "[{tag}:")?;
      write_axes_csv(out, v)?;
      write!(out, "]\r\n")
    };
    let result = match index {
      0..=5 => {
        // The G54-G59 tag for the offset index: 0 → "G54" … 5 → "G59".
        let tag = WCS_TAGS[index];
        coord_line(out, tag, &report.wcs[index])
      }
      6 => coord_line(out, "G28", &report.predefined[0]),
      7 => coord_line(out, "G30", &report.predefined[1]),
      8 => coord_line(out, "G92", &report.g92),
      // grbl's legacy single-axis TLO form `[TLO:z]`; senders parse 1..N values, so one Z value is compatible.
      9 => write!(out, "[TLO:{:.3}]\r\n", report.tlo),
      // The probe result: machine position (all axes) at the trigger instant with a trailing `:1`/`:0` flag.
      10 => write!(out, "[PRB:")
        .and_then(|()| write_axes_csv(out, &report.probe))
        .and_then(|()| write!(out, ":{}]\r\n", report.probe_success as u8)),
      _ => return false,
    };
    result.is_ok()
  }
}

/// Write a coordinate tuple as grbl's comma-separated `{:.3}` axis list (`x,y,z,a`), one field per motion axis
/// ([`AXIS_COUNT`]). Shared by the `$#` coordinate/PRB lines and the `[PRB:]` push so every report carries all
/// four axes — the rotary A included — matching the four-field live `MPos:`/`WPos:` status (grbl reports N_AXIS
/// values). Centralizing the loop keeps a fourth axis from being silently dropped, as the hardcoded three-field
/// forms did before DOC-10.
fn write_axes_csv<const N: usize>(out: &mut String<N>, values: &[f32; AXIS_COUNT]) -> core::fmt::Result {
  for (axis, value) in values.iter().enumerate() {
    if axis == 0 {
      write!(out, "{value:.3}")?;
    } else {
      write!(out, ",{value:.3}")?;
    }
  }
  Ok(())
}

/// Write an `f32` with the minimal decimal representation grbl uses for `$G` feed words: an integral value
/// renders with no fractional part (`1500` not `1500.0`), while a value with a fraction keeps only its
/// significant fractional digits (`250.25`, not `250.2500`). Rendering through a fixed-precision buffer and
/// trimming trailing zeros keeps this allocation-free and avoids pulling in float-to-shortest formatting.
fn write_minimal_f32<const N: usize>(out: &mut String<N>, value: f32) -> Result<(), FmtError> {
  // Three decimals covers feed resolution finer than any real machine; the trim below removes the padding
  // so a whole or one-/two-place value renders compactly.
  let mut scratch: String<24> = String::new();
  write!(scratch, "{value:.3}").map_err(|_| FmtError)?;
  let trimmed = if scratch.contains('.') {
    scratch.trim_end_matches('0').trim_end_matches('.')
  } else {
    scratch.as_str()
  };
  out.push_str(trimmed).map_err(|_| FmtError)
}
