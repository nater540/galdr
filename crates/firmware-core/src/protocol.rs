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

use core::fmt::Write as _;

use heapless::{String, Vec};

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

/// The grblHAL `error:N` code for an over-length input line ("line length exceeded").
pub const ERROR_LINE_OVERFLOW: u8 = 15;

/// The number of motion axes reported in build info and status (`[AXS:4:XYZA]`, four `MPos` fields).
/// References the planner's [`AXES`](crate::planner::AXES) so the protocol layer cannot disagree with the
/// kinematics about how many axes exist.
pub const AXIS_COUNT: usize = crate::planner::AXES;

/// The firmware version string reported in the banner and the `[VER:]` build-info line. grblHAL reports
/// a grbl-1.1f-compatible version so senders compliant with grbl 1.1f recognize the controller.
pub const VERSION: &str = "1.1f";

/// A real-time command: a single byte intercepted out of the RX stream the instant it arrives, ahead of
/// the line buffer. It never enters a line and never receives an `ok`. Both the printable grbl-1.1 forms
/// and the grblHAL top-bit-set forms (advertised by `RT+` in `NEWOPT`) classify to the same variant so a
/// sender may use either. Stage-1 acts on the first four; the rest are carried so the driver can dispatch
/// or ignore them without re-scanning, and so Stage 2 can light them up without changing this surface.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RealtimeCommand {
  /// `?` / `0x80` — status report request. The driver answers with a `<...>` report immediately.
  StatusReport,
  /// `~` / `0x81` — cycle start / resume.
  CycleStart,
  /// `!` / `0x82` — feed hold.
  FeedHold,
  /// `0x18` (Ctrl-X) — soft reset / abort: halt motion, reset parser/planner, re-emit the banner.
  SoftReset,
  /// `0x19` (Ctrl-Y) — stop: like soft reset but leaves more internal state intact (no full warm reset).
  Stop,
  /// `0x83` — request the parser-state (`$G`) report on demand.
  ParserStateReport,
  /// `0x84` — safety door: suspend into the DOOR state, kill spindle/coolant.
  SafetyDoor,
  /// `0x85` — jog cancel: feed-hold plus a planner flush; ignored if not jogging.
  JogCancel,
  /// `0x86` — Galdr graceful program stop: decelerate the running/held program to a controlled stop at a block
  /// boundary, flush the planner queue + any in-progress arc, clear the program/modal-run state, and return to
  /// Idle (NOT alarm) with machine position RETAINED. Distinct from the `0x18` abort (which raises ALARM:3 on a
  /// mid-cycle reset and loses positional certainty); ignored (benign) when not running or held. This is a Galdr
  /// extension — grbl has no dedicated graceful-stop real-time byte, modelling it as feed-hold then reset.
  ProgramStop,
  /// `0x87` — full real-time report (all change-only elements plus the alarm substate). Answered even in
  /// otherwise-locked states so a sender can detect an extended (grblHAL) controller on connect.
  FullStatusReport,
  /// `0x8C` — toggle the auto real-time report mode (`$481`). Carried for Stage 3; no Stage-1 action.
  ToggleAutoReport,
  /// `0x88` — toggle the optional-stop switch that gates `M1`. grblHAL leaves this OFF by default, so an `M1` is a
  /// no-op until the operator enables it; the firmware holds the toggle and consults it when an `M1` pause runs.
  ToggleOptionalStop,
  /// A feed / rapid / spindle / coolant override byte (`0x90`–`0x9E`, `0xA0`–`0xA4`). The raw byte is
  /// preserved so the override handler can decode the specific adjustment without a second classify pass.
  Override(u8),
}

/// One grblHAL `error:N` code with its name and description, for the `$EE` enumeration
/// (`[ERRORCODE:<id>|<name>|<description>]`). The [`ERROR_CODES`] table below is the single authority: every
/// `error:N` this firmware can return over the wire (from the GCode parser, the settings setter, and the
/// protocol/`$`-dispatch layer) has exactly one row here, so a sender that builds its error display from `$EE`
/// matches the codes it actually receives. The numeric values match grblHAL's published error list.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ErrorCode {
  /// The grblHAL `error:N` numeric code.
  pub id: u8,
  /// The short error name (2nd field of an `[ERRORCODE:]` line).
  pub name: &'static str,
  /// The longer error description (3rd field of an `[ERRORCODE:]` line).
  pub description: &'static str,
}

/// Every `error:N` code this firmware can emit, in ascending numeric order, for the `$EE` enumeration. This is
/// the single authority: the GCode parser's [`GcodeError`](crate::gcode::GcodeError), the settings
/// [`SettingError`](crate::settings::SettingError), and the protocol/`$`-dispatch codes
/// ([`ERROR_LINE_OVERFLOW`], [`ERROR_UNSUPPORTED_COMMAND`], [`ERROR_HOMING_DISABLED`]) all surface codes drawn
/// from this set. The names/descriptions mirror grblHAL's published `error_codes.csv` so a sender's display
/// matches the reference controller. Codes are listed once even when several internal variants share them (e.g.
/// the parser's probe/jog "requires axis words" both map to `error:26`).
pub const ERROR_CODES: &[ErrorCode] = &[
  ErrorCode {
    id: 1,
    name: "Expected command letter",
    description: "G-code words consist of a letter and a value. Letter was not found.",
  },
  ErrorCode {
    id: 2,
    name: "Bad number format",
    description: "Numeric value format is not valid or missing an expected value.",
  },
  ErrorCode {
    id: 3,
    name: "Invalid statement",
    description: "Grbl '$' system command was not recognized or supported.",
  },
  ErrorCode {
    id: 5,
    name: "Setting disabled",
    description: "Homing cycle failure. Homing is not enabled via settings.",
  },
  ErrorCode {
    id: 9,
    name: "G-code lock",
    description: "G-code locked out during alarm or jog state.",
  },
  ErrorCode {
    id: 15,
    name: "Travel exceeded",
    description: "Jog target exceeds machine travel, or the line length was exceeded. Command ignored.",
  },
  ErrorCode {
    id: 20,
    name: "Unsupported command",
    description: "Unsupported or invalid g-code command found in block.",
  },
  ErrorCode {
    id: 21,
    name: "Modal group violation",
    description: "More than one G-code command from the same modal group was found in the block.",
  },
  ErrorCode {
    id: 22,
    name: "Undefined feed rate",
    description: "Feed rate has not yet been set or is undefined.",
  },
  ErrorCode {
    id: 23,
    name: "Invalid g-code ID:23",
    description: "A G-code command value, such as a tool number (T), must be a non-negative integer within range.",
  },
  ErrorCode {
    id: 26,
    name: "No axis words in block",
    description: "A G-code command (or the current modal state) requires axis words, but none were found in the block.",
  },
  ErrorCode {
    id: 33,
    name: "Invalid target",
    description: "A G-code motion command has an invalid target (for example, arc geometry that cannot be reconciled, or a rotary axis word in a G38.x probe, which is linear-only).",
  },
];

/// The short name of an `error:N` code from [`ERROR_CODES`], or `None` if the code is not enumerated. Used to
/// build the `[MSG:error:N <name>]` context line a plain terminal sees alongside the bare `error:N`.
pub fn error_name(code: u8) -> Option<&'static str> {
  ERROR_CODES.iter().find(|c| c.id == code).map(|c| c.name)
}

/// The machine run-state reported as the first field of a `<...>` status report. Stage 1 only ever
/// reports [`Idle`](MachineState::Idle), but the full grblHAL set is enumerated here so the formatter and
/// the shared [`MachineSnapshot`] are Stage-2-ready (alarm/hold/homing) without a breaking change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MachineState {
  /// No motion queued or executing.
  Idle,
  /// Executing a queued motion block.
  Run,
  /// Feed hold: `Hold:0` complete/ready-to-resume, `Hold:1` in progress. The `bool` is the substate.
  Hold(bool),
  /// Executing a jog.
  Jog,
  /// Halted in an alarm; the `u8` is the grblHAL alarm code (added to the full `0x87` report).
  Alarm(u8),
  /// Safety door open.
  Door,
  /// `$C` check mode: parse/validate without moving.
  Check,
  /// Running a homing cycle.
  Home,
  /// `$SLP` sleep: spindle/coolant off, drivers parked, held until a soft reset wakes the machine.
  Sleep,
  /// grblHAL `STATE_TOOLCHANGE`: an `M6` manual tool change is held, awaiting a cycle-start (`~`) resume. A bare
  /// state token with no substate — distinct from a feed-hold's `Hold:0` (M0/M1 still report `Hold:0`).
  Tool,
}

impl MachineState {
  /// The grblHAL status-report state token, written as the first field of a `<...>` report. Substates
  /// (`Hold:0`/`Hold:1`, `Alarm:<code>`) are appended by the status formatter, not encoded here.
  fn token(self) -> &'static str {
    match self {
      MachineState::Idle => "Idle",
      MachineState::Run => "Run",
      MachineState::Hold(_) => "Hold",
      MachineState::Jog => "Jog",
      MachineState::Alarm(_) => "Alarm",
      MachineState::Door => "Door",
      MachineState::Check => "Check",
      MachineState::Home => "Home",
      MachineState::Sleep => "Sleep",
      MachineState::Tool => "Tool",
    }
  }
}

/// A grblHAL alarm code (DOC-08 §7). An alarm halts motion and blocks GCode until cleared by `$X` (for the
/// non-critical-locked codes) or a soft reset. Only the subset Phase A can actually enter is defined with
/// behavior; the remaining variants are the entry points later phases (DOC-06 homing, DOC-09 probing,
/// hard/soft-limit GPIO) will raise. The numeric values are grbl's canonical alarm numbers, emitted as
/// `ALARM:N` and as the `Alarm:<code>` status substate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AlarmCode {
  /// `ALARM:1` hard limit triggered — machine position is likely lost; re-homing is recommended (DOC-06).
  HardLimit,
  /// `ALARM:2` soft limit — a commanded move exceeded the machine envelope (DOC-06 soft-limit checking).
  SoftLimit,
  /// `ALARM:3` reset/abort while a cycle was in progress — motion was halted mid-move, position is suspect.
  AbortDuringCycle,
  /// `ALARM:4` probe fail — the probe was already triggered when a `G38.2`/`G38.4` cycle began (DOC-09).
  ProbeFailInitial,
  /// `ALARM:5` probe fail — the probe did not trip within the programmed travel of a `G38.2`/`G38.4` (DOC-09).
  ProbeFailContact,
  /// `ALARM:8` homing fail — a `$H` seek/locate did not find its limit switch within 1.5× the axis max travel
  /// (`EXEC_ALARM_HOMING_FAIL_APPROACH`). Recoverable: reset, check the switch/wiring, retry `$H` (DOC-06).
  HomingFail,
  /// `ALARM:10` e-stop asserted (a locked critical alarm; cleared only by a soft reset once de-asserted).
  EStop,
  /// `ALARM:11` homing required — set on boot when `$22` is enabled; cleared by `$H` (home) or `$X` (unlock).
  HomingRequired,
}

impl AlarmCode {
  /// The grbl numeric alarm code, emitted as `ALARM:N` and as the `Alarm:<code>` status substate.
  pub fn code(self) -> u8 {
    match self {
      AlarmCode::HardLimit => 1,
      AlarmCode::SoftLimit => 2,
      AlarmCode::AbortDuringCycle => 3,
      AlarmCode::ProbeFailInitial => 4,
      AlarmCode::ProbeFailContact => 5,
      AlarmCode::HomingFail => 8,
      AlarmCode::EStop => 10,
      AlarmCode::HomingRequired => 11,
    }
  }

  /// Every alarm code this firmware defines, in ascending numeric order, for the `$EA` enumeration. Driven off
  /// this one list so the enumeration can never describe an alarm the firmware cannot actually raise (and vice
  /// versa) — [`crate::protocol::tests`] asserts the list matches the `code()`/`name()`/`description()` set.
  pub const ALL: &'static [AlarmCode] = &[
    AlarmCode::HardLimit,
    AlarmCode::SoftLimit,
    AlarmCode::AbortDuringCycle,
    AlarmCode::ProbeFailInitial,
    AlarmCode::ProbeFailContact,
    AlarmCode::HomingFail,
    AlarmCode::EStop,
    AlarmCode::HomingRequired,
  ];

  /// The short grblHAL alarm NAME, emitted as the 2nd field of an `[ALARMCODE:<id>|<name>|<description>]` line.
  pub fn name(self) -> &'static str {
    match self {
      AlarmCode::HardLimit => "Hard limit",
      AlarmCode::SoftLimit => "Soft limit",
      AlarmCode::AbortDuringCycle => "Abort during cycle",
      AlarmCode::ProbeFailInitial => "Probe fail",
      AlarmCode::ProbeFailContact => "Probe fail",
      AlarmCode::HomingFail => "Homing fail",
      AlarmCode::EStop => "EStop asserted",
      AlarmCode::HomingRequired => "Homing required",
    }
  }

  /// The longer grblHAL alarm DESCRIPTION, emitted as the 3rd field of an `[ALARMCODE:]` line. The text mirrors
  /// grblHAL's published alarm descriptions so a sender's tooltip matches the reference controller.
  pub fn description(self) -> &'static str {
    match self {
      AlarmCode::HardLimit => {
        "Hard limit has been triggered. Machine position is likely lost due to sudden halt. Re-homing is highly recommended."
      }
      AlarmCode::SoftLimit => "Soft limit alarm. G-code motion target exceeds machine travel.",
      AlarmCode::AbortDuringCycle => "Reset while in motion. Machine position is likely lost. Re-homing is highly recommended.",
      AlarmCode::ProbeFailInitial => "Probe fail. Probe is not in the expected initial state before starting probe cycle.",
      AlarmCode::ProbeFailContact => "Probe fail. Probe did not contact the workpiece within the programmed travel.",
      AlarmCode::HomingFail => "Homing fail. Could not find limit switch within search distance.",
      AlarmCode::EStop => "Emergency stop active.",
      AlarmCode::HomingRequired => "Homing is required. Execute homing cycle ($H) to continue.",
    }
  }

  /// Whether this is a *locked* critical alarm. In a locked alarm grblHAL answers only real-time report
  /// requests until a soft reset clears the physical cause (codes 1, 2, 10 per DOC-08 §5); `$X` cannot
  /// unlock these. The non-locked alarms (abort, probe-fail, homing-required) still accept `$` commands,
  /// so `$X` unlocks them. Drives the consumer's "accept `$X`?" decision, kept here so it is host-tested.
  pub fn is_locked(self) -> bool {
    matches!(self, AlarmCode::HardLimit | AlarmCode::SoftLimit | AlarmCode::EStop)
  }

  /// The `[MSG:...]` push text grbl prints on entering this alarm: the homing-required / locked-critical
  /// alarms prompt `'$H'|'$X' to unlock`, while the recoverable ones prompt `Reset to continue`. The
  /// caller wraps this in the `[MSG:...]` envelope via [`ResponseWriter::message`].
  pub fn unlock_hint(self) -> &'static str {
    match self {
      // Homing-required and the locked-critical alarms are cleared by homing or unlocking (or, for the
      // locked ones, a soft reset after the cause clears). The same prompt grbl uses fits all of them.
      AlarmCode::HomingRequired | AlarmCode::HardLimit | AlarmCode::SoftLimit | AlarmCode::EStop => {
        "'$H'|'$X' to unlock"
      }
      // The recoverable alarms (abort-during-cycle, probe-fail, homing-fail) tell the operator to reset and retry.
      AlarmCode::AbortDuringCycle | AlarmCode::ProbeFailInitial | AlarmCode::ProbeFailContact
      | AlarmCode::HomingFail => "Reset to continue",
    }
  }
}

/// The authoritative, latched control mode of the machine — the single source of truth the `firmware` bin
/// shares across its comms tasks and the status reporter (DOC-08 Stage 2). It is deliberately SMALLER than
/// [`MachineState`]: the Run-vs-Idle distinction in [`MachineState`] is *derived* from live execution facts
/// (in-flight block count) at report time via [`ControlState::machine_state`], not latched here, so two
/// tasks never race to write "Run". The `$`/real-time handlers mutate THIS; the status formatter renders the
/// composed [`MachineState`]. Kept a pure, `Copy` state machine so every transition is host-tested.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ControlState {
  /// Normal operation: the machine reports `Idle` when no block is in flight and `Run` while executing. This
  /// is the only state whose reported [`MachineState`] depends on live execution.
  Normal,
  /// Feed-hold active (`!`/`0x82`). The `bool` is the grbl substate: `false` = `Hold:0` (stopped, ready to
  /// resume), `true` = `Hold:1` (still decelerating). Phase A pauses at the block boundary, so it latches
  /// `Hold:0` directly; the `Hold:1` substate is reserved for the smooth ramp-down refinement.
  Hold(bool),
  /// A `$J=` jog is in flight (DOC-08 Phase D). Latched when a jog is accepted and held until the jog blocks
  /// drain (or a jog-cancel `0x85` flushes them), at which point the machine returns to `Normal`. Like
  /// `Normal`, the reported [`MachineState`] depends on live execution: `Jog` while jog blocks run, `Idle` once
  /// they drain. A jog NEVER changes modal/coordinate state, so leaving `Jog` needs no reset side-effects.
  Jog,
  /// Halted in an alarm; carries the [`AlarmCode`]. Entered on boot-lock (`$22`), soft-reset-during-cycle,
  /// and later phases' limit/probe/e-stop events. Cleared by `$X` (non-locked codes) or a soft reset.
  Alarm(AlarmCode),
  /// `$C` check mode: GCode is parsed and validated (and `ok`'d) but NOT planned or executed. Toggled off by
  /// a second `$C`, which grbl follows with a soft reset.
  Check,
  /// `$SLP` sleep: spindle/coolant off, drivers parked; held until a soft reset wakes the machine (DOC-07/
  /// DOC-03 own the actual peripheral shutdown — the state machine only latches the mode).
  Sleep,
  /// An `M6` manual tool change is held, awaiting a cycle-start (`~`) resume (grblHAL `STATE_TOOLCHANGE`). Reports
  /// the dedicated `Tool` wire state rather than `Hold:0` (M0/M1 still latch `Hold(false)`). Like a hold it is
  /// cycle-start-resumable (back to `Normal`) and motion-allowed (so the resumed program continues), but it is a
  /// DISTINCT wire state so a sender shows a tool-change prompt rather than a generic pause.
  Tool,
}

impl ControlState {
  /// The boot state: locked in [`AlarmCode::HomingRequired`] when `$22` homing is enabled (a host must `$H`
  /// or `$X` before streaming), otherwise [`Normal`](ControlState::Normal). Mirrors grbl's power-on behavior.
  pub fn boot(homing_enabled: bool) -> Self {
    if homing_enabled {
      ControlState::Alarm(AlarmCode::HomingRequired)
    } else {
      ControlState::Normal
    }
  }

  /// Compose the reported [`MachineState`] from this latched mode plus whether a block is currently in flight
  /// (queued or mid-burst on the executor). Only [`Normal`](ControlState::Normal) consults `running`: it
  /// reports `Run` while a block executes and `Idle` otherwise. Every other mode maps to its fixed state, so
  /// the Run/Idle derivation never has to race the latched modes. This is the one place `?` turns control
  /// state into a wire state.
  pub fn machine_state(self, running: bool) -> MachineState {
    match self {
      ControlState::Normal => {
        if running {
          MachineState::Run
        } else {
          MachineState::Idle
        }
      }
      // A jog reports `Jog` while its blocks run and `Idle` once they drain — like `Normal`, the Run/Idle
      // (here Jog/Idle) split is derived from live execution, never latched, so the reporter and the executor
      // never race to write it. The consumer drops `Jog` back to `Normal` once the queue empties.
      ControlState::Jog => {
        if running {
          MachineState::Jog
        } else {
          MachineState::Idle
        }
      }
      ControlState::Hold(in_progress) => MachineState::Hold(in_progress),
      ControlState::Alarm(code) => MachineState::Alarm(code.code()),
      ControlState::Check => MachineState::Check,
      ControlState::Sleep => MachineState::Sleep,
      // An M6 manual tool change reports the dedicated `Tool` state (latched, independent of live execution).
      ControlState::Tool => MachineState::Tool,
    }
  }

  /// Enter the `M6` manual-tool-change hold (`Tool` state). A constructor (not a transition from another mode) so
  /// the consumer's M6 pause sets it explicitly; `~` (cycle-start) resumes it back to `Normal` via
  /// [`cycle_start`](ControlState::cycle_start), and motion stays allowed so the resumed program continues.
  pub fn tool_change() -> Self {
    ControlState::Tool
  }

  /// Whether GCode motion is currently allowed. False in any alarm, in sleep, and in check mode (check parses
  /// and `ok`s but never plans). The consumer gates planning on this so a held/alarmed/asleep machine never
  /// enqueues a move. (Hold does not block *planning* — blocks may queue while held; the executor pauses at
  /// the boundary — so `Hold` is motion-allowed here.)
  pub fn motion_allowed(self) -> bool {
    matches!(self, ControlState::Normal | ControlState::Hold(_) | ControlState::Tool)
  }

  /// Apply a feed-hold (`!`/`0x82`): from `Normal` (or an existing hold) latch `Hold:0`. A hold requested in
  /// any other mode (alarm/check/sleep) is a no-op — grbl ignores `!` when not running a program. Returns the
  /// resulting state so the caller can publish it.
  pub fn feed_hold(self) -> Self {
    match self {
      ControlState::Normal | ControlState::Hold(_) => ControlState::Hold(false),
      other => other,
    }
  }

  /// Apply a cycle-start (`~`/`0x81`): release a feed-hold back to `Normal` (the Run/Idle distinction is then
  /// re-derived from live execution). In any non-hold mode it is a no-op, matching grbl (`~` does not clear an
  /// alarm or wake from sleep — only a soft reset / `$X` does).
  pub fn cycle_start(self) -> Self {
    match self {
      // Both a feed-hold and an M6 tool-change hold resume to `Normal` on `~` (Run/Idle re-derived from execution).
      ControlState::Hold(_) | ControlState::Tool => ControlState::Normal,
      other => other,
    }
  }

  /// Whether a cycle-start (`~`/`0x81`) should resume motion from THIS control state — i.e. whether the machine
  /// is in a feed-hold that `~` releases. `~` resumes ONLY a hold: it is inert in Idle/Run (`Normal`), Jog,
  /// Alarm, Check, and Sleep. Sleep in particular must NOT wake on `~` — only a soft reset wakes a sleeping
  /// machine (a `$SLP` parks the executor via the same hold latch, but a `~` must leave it parked). Pulled out
  /// as a pure predicate so the bin's real-time `~` dispatch can gate the executor-hold release on it without
  /// duplicating the state logic, and so the gate is host-tested here rather than in the untestable wiring.
  pub fn resumes_on_cycle_start(self) -> bool {
    // A feed-hold (M0/M1) AND an M6 tool-change hold both resume on `~`; every other mode is inert (an alarm/sleep
    // only clears via `$X`/soft reset).
    matches!(self, ControlState::Hold(_) | ControlState::Tool)
  }

  /// Whether a hard-limit trip from the core-1 executor should raise a fresh `ALARM:1` from THIS control state.
  /// True from the states where the executor's EDGE-armed detector can produce a genuine NEW assertion worth
  /// surfacing: `Normal` (running a program), `Hold` (a held program could resume into a switch), `Jog`, and
  /// `Check`. `Check` never enqueues motion, but the trip is edge-armed (`hard_limit_alarm_armed`), so it fires
  /// only on a switch NEWLY pressed DURING the dry-run — a real safety event grbl surfaces regardless of mode, not
  /// a stale parked-switch read. FALSE from any `Alarm(_)` and from `Sleep`: in those states the machine is
  /// already halted/parked, so a trip is a STALE read of a parked switch. Re-raising `ALARM:1` over an existing alarm changes
  /// nothing useful and can only CLOBBER a more-specific state — most damagingly downgrading the boot-lock
  /// `ALARM:11` (homing required) into the locked `ALARM:1`, losing the "homing required" semantic the host must
  /// satisfy. Pulled out as a pure predicate so the bin's hard-limit consumer arm can gate the `ALARM:1` raise
  /// without duplicating the alarm/sleep logic, and so the gate is host-tested here rather than in the untestable
  /// cross-core wiring. The PRIMARY fix for the post-aborted-homing `error:9` re-lock is the executor's EDGE-armed
  /// hard-limit alarm (`hard_limit_alarm_armed`): a switch left engaged after a `$H` abort is a HELD level, not a
  /// fresh edge, so it never signals a stale trip in the first place. This predicate is the single remaining
  /// DEFENSIVE layer (the soft-reset signal drains were removed once the arming subsumed them): it ensures any
  /// stray `HARD_LIMIT_TRIPPED` arriving while ALREADY alarmed/asleep cannot downgrade a more-specific lock (most
  /// damagingly `ALARM:11`) into the locked `ALARM:1`. A legitimately NEW over-travel still alarms because the
  /// machine is in `Normal`/`Hold`/`Jog`/`Check` while moving, and the executor re-signals on its fresh edge.
  pub fn hard_limit_alarm_applies(self) -> bool {
    matches!(
      self,
      ControlState::Normal | ControlState::Hold(_) | ControlState::Jog | ControlState::Check
    )
  }

  /// Apply a soft reset (`0x18`). Per grbl: a reset that aborts an IN-PROGRESS cycle raises
  /// [`AlarmCode::AbortDuringCycle`] (position is suspect after a mid-move halt); a reset from any other
  /// state returns to the boot state — locked in homing-required when `$22` is set, else `Normal`. A reset
  /// also clears an existing (non-homing) alarm, check, or sleep back to the boot baseline. `was_in_cycle`
  /// is the executor's "a block was mid-flight when the reset landed" fact.
  pub fn soft_reset(self, was_in_cycle: bool, homing_enabled: bool) -> Self {
    if was_in_cycle {
      // Aborting a move loses positional certainty: grbl forces an abort alarm regardless of `$22` so the
      // host must re-establish state (re-home or `$X`) before the next move.
      ControlState::Alarm(AlarmCode::AbortDuringCycle)
    } else {
      ControlState::boot(homing_enabled)
    }
  }

  /// Apply `$X` (kill alarm lock). Valid only from a NON-locked alarm (the locked critical codes 1/2/10 are
  /// cleared by a soft reset, not `$X`): it returns `Normal` and the caller emits `[MSG:Caution: Unlocked]`.
  /// From a locked alarm it stays put and the caller rejects the command. From a non-alarm state `$X` is a
  /// no-op `ok` (returns `self` unchanged); the [`Unlock`](UnlockOutcome) result distinguishes the cases.
  pub fn unlock(self) -> (Self, UnlockOutcome) {
    match self {
      ControlState::Alarm(code) if code.is_locked() => (self, UnlockOutcome::Locked),
      ControlState::Alarm(_) => (ControlState::Normal, UnlockOutcome::Unlocked),
      other => (other, UnlockOutcome::NotAlarmed),
    }
  }

  /// Toggle `$C` check mode. From `Normal` it enters [`Check`](ControlState::Check) (the caller emits
  /// `[MSG:Enabled]`); from `Check` it leaves check mode, which grbl realizes as a soft reset — so this
  /// returns the post-reset boot state and signals the caller (via [`CheckToggle::Disabled`]) to run the
  /// reset side-effects (banner, parser/planner rebuild). From any other mode it is rejected (returns `self`
  /// with [`CheckToggle::Rejected`]) — grbl only enters check mode from Idle/Normal.
  pub fn toggle_check(self, homing_enabled: bool) -> (Self, CheckToggle) {
    match self {
      ControlState::Normal => (ControlState::Check, CheckToggle::Enabled),
      ControlState::Check => (ControlState::boot(homing_enabled), CheckToggle::Disabled),
      other => (other, CheckToggle::Rejected),
    }
  }

  /// Apply `$SLP` (sleep). Allowed from `Normal` only (grbl rejects sleep while alarmed or running a hold):
  /// latches [`Sleep`](ControlState::Sleep), and the caller parks the spindle/drivers (DOC-07/DOC-03) and
  /// holds the pipeline until a soft reset wakes the machine. From any other mode it is rejected.
  pub fn enter_sleep(self) -> (Self, bool) {
    match self {
      ControlState::Normal => (ControlState::Sleep, true),
      other => (other, false),
    }
  }

  /// Whether a `$J=` jog may be accepted now (DOC-08 Phase D). grbl accepts a jog ONLY from Idle or an existing
  /// jog (so a stream of `$J=` lines chains smoothly), and rejects it while running a program, in a feed-hold,
  /// or in any alarm/check/sleep state. `Normal` here means Idle-or-Run; the consumer additionally requires the
  /// program queue to be empty (no program block in flight) before accepting, so this gates the LATCHED mode and
  /// the consumer gates live execution — together they realize grbl's "jog only from Idle/Jog".
  pub fn jog_allowed(self) -> bool {
    matches!(self, ControlState::Normal | ControlState::Jog)
  }

  /// Latch [`Jog`](ControlState::Jog) on accepting a `$J=` jog. From `Normal` or an existing `Jog` it enters
  /// (or stays in) `Jog`; from any other mode it is a no-op (returns `self`) — the consumer only calls this
  /// after [`jog_allowed`](ControlState::jog_allowed) clears, so the no-op arm is purely defensive.
  pub fn begin_jog(self) -> Self {
    match self {
      ControlState::Normal | ControlState::Jog => ControlState::Jog,
      other => other,
    }
  }

  /// Return to `Normal` on a jog-cancel (`0x85`) or once the jog blocks drain (DOC-08 Phase D). A jog NEVER
  /// changed modal/coordinate state, so leaving `Jog` needs no reset side-effects — it simply drops the latch
  /// back to `Normal` (whose reported state re-derives Idle/Run from live execution). From any non-jog mode it
  /// is a no-op (`0x85` is ignored when not jogging), returning `self`.
  pub fn cancel_jog(self) -> Self {
    match self {
      ControlState::Jog => ControlState::Normal,
      other => other,
    }
  }

  /// Apply a graceful program stop (`0x86`, Galdr extension): from `Normal` (running or idle) or either `Hold`
  /// substate, return to [`Normal`](ControlState::Normal) — a motion-capable Idle once motion drains. NEVER an
  /// alarm: this is the controlled "stop the job" the operator wants, distinct from the `0x18` abort that raises
  /// `ALARM:3` on a mid-cycle reset and loses positional certainty. From any other mode (alarm, check, sleep, or
  /// an in-flight jog — which has its own `0x85` cancel) it is a benign no-op (returns `self`). A program stop
  /// never changes the coordinate model or loses position, so leaving the running/held state needs no alarm and
  /// the position is retained; the bin clears the modal/program-run state separately (mirroring `M30`).
  pub fn program_stop(self) -> Self {
    match self {
      ControlState::Normal | ControlState::Hold(_) => ControlState::Normal,
      other => other,
    }
  }

  /// Whether a graceful program stop (`0x86`) needs the executor parked at a block boundary and the planner queue
  /// flushed — i.e. whether the machine is in a state that could have a program running or held. True for `Normal`
  /// (which may be executing a program) and `Hold`; false for the inert states (alarm, check, sleep) and for `Jog`
  /// (a jog is not a program — it has its own `0x85` cancel). Pulled out as a pure predicate so the bin gates its
  /// boundary-quiesce + flush work on it without duplicating the state logic, and so the gate is host-tested here.
  pub fn program_stop_quiesces(self) -> bool {
    matches!(self, ControlState::Normal | ControlState::Hold(_))
  }

  /// Whether a `$H` homing cycle may START from this state (DOC-06). grbl runs `$H` from Idle/Normal AND from
  /// the homing-required boot alarm (`$H` is THE way to clear `ALARM:11`), but refuses it from the locked
  /// critical alarms (hard/soft limit, e-stop — those need a soft reset first), from a feed-hold, and from
  /// check/sleep. Pulled out as a predicate so the bin gates `$H` on it without duplicating the state logic.
  pub fn homing_allowed(self) -> bool {
    match self {
      ControlState::Normal => true,
      // The boot lock (homing-required) is exactly the state `$H` exists to clear; the other (locked) alarms are
      // not — they demand a reset before any cycle.
      ControlState::Alarm(code) => matches!(code, AlarmCode::HomingRequired),
      _ => false,
    }
  }

  /// Apply a successful `$H` homing cycle (DOC-06): machine position is now established, so the machine returns
  /// to [`Normal`](ControlState::Normal) — clearing the homing-required boot alarm (`ALARM:11`). Only valid from
  /// a state [`homing_allowed`](ControlState::homing_allowed) cleared; from any other state it is a no-op
  /// (returns `self`), which is purely defensive since the consumer gates `$H` on `homing_allowed` first.
  pub fn home_complete(self) -> Self {
    if self.homing_allowed() {
      ControlState::Normal
    } else {
      self
    }
  }
}

/// The outcome of a `$X` unlock attempt, so the caller can choose the right response without re-inspecting
/// the state: emit `[MSG:Caution: Unlocked]` + `ok` on [`Unlocked`](UnlockOutcome::Unlocked), a bare `ok` on
/// [`NotAlarmed`](UnlockOutcome::NotAlarmed) (a no-op from a non-alarm state), or `error:N` on
/// [`Locked`](UnlockOutcome::Locked) (a locked critical alarm that only a soft reset can clear).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UnlockOutcome {
  /// A non-locked alarm was cleared; emit `[MSG:Caution: Unlocked]` then `ok`.
  Unlocked,
  /// The machine was not in an alarm; `$X` is a no-op `ok`.
  NotAlarmed,
  /// A locked critical alarm (hard/soft limit, e-stop) — `$X` cannot clear it; reject with `error:N`.
  Locked,
}

/// The outcome of a `$C` check-mode toggle, so the caller emits the right `[MSG:...]` and runs the
/// soft-reset side-effects only when check mode is being disabled.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CheckToggle {
  /// Check mode was entered; emit `[MSG:Enabled]` then `ok`.
  Enabled,
  /// Check mode was left; grbl follows this with a soft reset — emit `[MSG:Disabled]`, then run the reset
  /// side-effects (rebuild parser/planner, emit the banner) and `ok`.
  Disabled,
  /// The toggle was rejected (not in Normal/Check); the caller responds `error:N`.
  Rejected,
}

/// Which position element a `<...>` status report carries, selected by the `$10` mask bit 0 (DOC-08 §4):
/// grbl/grblHAL report EITHER machine position (`MPos:`) OR work position (`WPos:`), never both, and a host
/// reconstructs the other from the `WCO:` element. Held in the [`MachineSnapshot`] so the formatter is a pure
/// function of the snapshot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PositionReport {
  /// Report `MPos:` (machine position). The `$10` default — bit 0 set.
  Machine,
  /// Report `WPos:` (work position = `MPos − WCO`). Selected when `$10` bit 0 is clear.
  Work,
}

impl PositionReport {
  /// Derive the position-report mode from the `$10` status-report mask: bit 0 set ⇒ machine position,
  /// clear ⇒ work position. This is the one place the mask bit becomes a reporting choice, so the firmware
  /// bin and the formatter agree on the `$10` semantics.
  pub fn from_status_mask(mask: u8) -> Self {
    if mask & STATUS_MASK_MACHINE_POSITION != 0 {
      PositionReport::Machine
    } else {
      PositionReport::Work
    }
  }
}

/// `$10` status-report mask bit 0: when set, the report carries `MPos:` (machine position); when clear it
/// carries `WPos:` (work position). grbl 1.1+ always reports exactly one of the two.
pub const STATUS_MASK_MACHINE_POSITION: u8 = 0x01;

/// An immutable, `Copy` snapshot of the live machine state the status formatter renders. The `firmware`
/// bin fills this from shared atomics/cells (live MPos from the motion executor, the active WCO from the
/// coordinate model, buffer free-counts from the planner queue and RX path) and hands it to
/// [`ResponseWriter::status_report`]; keeping the formatter pure over a snapshot is what makes status
/// reporting host-testable.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MachineSnapshot {
  /// Current run-state (report's first field).
  pub state: MachineState,
  /// Machine position in mm, `[X, Y, Z]`. Rendered as `MPos:` directly, or used (with `wco_mm`) to derive
  /// `WPos:` when [`position_report`](MachineSnapshot::position_report) is [`PositionReport::Work`].
  pub mpos_mm: [f32; AXIS_COUNT],
  /// The active Work Coordinate Offset in mm, `[X, Y, Z]` (= `G54..59[active] + G92 + TLO`). Used to derive
  /// `WPos = MPos − WCO` and rendered as the `WCO:` element on the refresh cadence.
  pub wco_mm: [f32; AXIS_COUNT],
  /// Which position element to report (`MPos:` vs `WPos:`), from the `$10` mask.
  pub position_report: PositionReport,
  /// Whether to include the `WCO:` element in THIS report (the change/periodic-refresh cadence — see
  /// [`RefreshReporter`]). grbl emits `WCO:` on change and at least every ~10-30 reports, not every report.
  pub include_wco: bool,
  /// The REALIZED feed rate in mm/min (`FS:` first field): the programmed feed scaled by the active feed
  /// override and clamped to the axis max-rate (or the rapid rate scaled by the rapid override for a G0). The
  /// `firmware` bin computes this from the executing block and the live [`Overrides`] (Phase E) so `FS:`
  /// reflects what the machine is actually doing, not the raw programmed word.
  pub feed_mm_min: f32,
  /// The REALIZED spindle speed in RPM (`FS:` second field): the commanded RPM scaled by the active spindle
  /// override (zero when the spindle-stop toggle is set). Computed by the bin from the live [`Overrides`].
  pub spindle_rpm: u16,
  /// Planner blocks free (`Bf:` first field).
  pub planner_blocks_free: u8,
  /// RX buffer bytes free (`Bf:` second field).
  pub rx_bytes_free: u16,
  /// The asserted input pins for the `Pn:` element (Phase E). Omitted from the report when none is asserted.
  pub pins: PinReport,
  /// The live feed / rapid / spindle override percentages for the `Ov:` element (Phase E).
  pub overrides: Overrides,
  /// Whether to include the `Ov:` element in THIS report (the change/periodic-refresh cadence — see
  /// [`RefreshReporter`]). grbl emits `Ov:` on change and at least every ~10-30 reports, not every report.
  pub include_ov: bool,
}

impl MachineSnapshot {
  /// A power-on default: idle at the origin with empty buffers fully free, machine-position reporting, and the
  /// WCO included (grbl emits `WCO:` in the first report after reset). Used as the initial shared state before
  /// the motion executor publishes a live position.
  pub const fn idle() -> Self {
    Self {
      state: MachineState::Idle,
      mpos_mm: [0.0; AXIS_COUNT],
      wco_mm: [0.0; AXIS_COUNT],
      position_report: PositionReport::Machine,
      include_wco: true,
      feed_mm_min: 0.0,
      spindle_rpm: 0,
      planner_blocks_free: BLOCK_BUFFER_SIZE as u8,
      rx_bytes_free: RX_BUFFER_SIZE as u16,
      pins: PinReport::new_idle(),
      overrides: Overrides::new(),
      // The first report after construction/reset includes `Ov:` (grbl's "first report" rule); the bin's
      // `Ov:` `RefreshReporter` re-affirms this, but seeding `true` keeps a hand-built idle snapshot consistent.
      include_ov: true,
    }
  }
}

/// How many status reports may pass without a change-only element (`WCO:`/`Ov:`) before a periodic refresh
/// forces one (grbl's "every 10 or 30 reports"). 10 is grbl's motion-state cadence — a safe, frequent default
/// that keeps a host's derived state fresh without bloating every report. Shared by every [`RefreshReporter`].
pub const REFRESH_PERIOD: u16 = 10;

/// How many status reports may pass without a `WCO:` element before a periodic refresh forces one. An alias for
/// the shared [`REFRESH_PERIOD`], kept so callers and tests can name the WCO cadence specifically.
pub const WCO_REFRESH_PERIOD: u16 = REFRESH_PERIOD;

/// A generic grbl change-only-element refresh cadence (DOC-08 §4): grbl reports a change-only element (`WCO:`,
/// `Ov:`) "in every 10 or 30 status reports (configurable), immediately in the next report after the value
/// changes, and in the first report after a reset". This small state machine decides, per report, whether the
/// tracked element should be included, keeping the change-detection + periodic-refresh rule pure and host-tested
/// rather than scattered through the firmware bin's status task. One generic type serves every such element: `T`
/// is the tracked value (`[f32; AXIS_COUNT]` for `WCO:`, [`Overrides`] for `Ov:`), and change detection is the
/// type's own `PartialEq` — element-wise for the `[f32; AXIS_COUNT]` array, so a non-finite component still reads
/// as "changed" against the baseline. It is `Copy` (when `T: Copy`) so the bin can hold it in a `Cell` beside the
/// other shared state.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RefreshReporter<T: Copy + PartialEq> {
  /// The value reported in the last report that included the element (so a change is detected against it).
  last_reported: T,
  /// Reports emitted since the element was last included; forces a periodic refresh at [`REFRESH_PERIOD`].
  since_refresh: u16,
  /// Set until the first report is emitted, so the FIRST report after construction/reset always includes it.
  force_first: bool,
}

impl<T: Copy + PartialEq> RefreshReporter<T> {
  /// A fresh reporter: the first report always includes the element (grbl's "first report after reset" rule),
  /// with `baseline` as the recorded value so any initial value DIFFERENT from it also counts as a change. The
  /// firmware seeds the baseline with the element's identity (origin WCO / default overrides). `const` so it can
  /// seed a `static` cell without a lazy `Option`.
  pub const fn new(baseline: T) -> Self {
    Self { last_reported: baseline, since_refresh: 0, force_first: true }
  }

  /// Reset the cadence so the next report includes the element (used on a soft reset, matching grbl's "first
  /// report after a reset includes the change-only elements"). Re-seeds the baseline with `baseline`.
  pub fn reset(&mut self, baseline: T) {
    *self = Self::new(baseline);
  }

  /// Decide whether the report being formatted NOW should include the tracked element, given its `current`
  /// value. Returns `true` when the value changed since it was last reported, when the periodic-refresh period
  /// has elapsed, or on the first report after construction/reset. Call exactly once per emitted report: it
  /// advances the internal counters and records the value when it answers `true`, so the change baseline and the
  /// period stay correct.
  pub fn should_include(&mut self, current: T) -> bool {
    let changed = current != self.last_reported;
    // Include on the Nth report of each window of `REFRESH_PERIOD`: with `since_refresh` counting the suppressed
    // reports since the last include, the (period − 1)th suppression makes the next report the periodic one, so
    // exactly one report in every `REFRESH_PERIOD` carries the element.
    let periodic = self.since_refresh >= REFRESH_PERIOD - 1;
    if self.force_first || changed || periodic {
      self.last_reported = current;
      self.since_refresh = 0;
      self.force_first = false;
      true
    } else {
      self.since_refresh = self.since_refresh.saturating_add(1);
      false
    }
  }
}

/// Classify a single byte as a real-time command, or `None` if it is ordinary line content. Both the
/// printable grbl forms and the grblHAL top-bit-set forms map to the same [`RealtimeCommand`]. The RX
/// scanner calls this on every byte and diverts any `Some` result before line assembly.
///
/// Note on `?`/`~`/`!`: grblHAL ignores the *printable* forms while reading `$`-command or message input
/// so those characters can appear in passwords/strings. Stage 1 does not implement that input-mode
/// suppression — these are always intercepted — which is the more conservative, sender-compatible
/// behavior; Stage 2 can gate the printable forms on an input-mode flag held by the orchestrator.
pub fn classify_realtime(byte: u8) -> Option<RealtimeCommand> {
  match byte {
    b'?' | 0x80 => Some(RealtimeCommand::StatusReport),
    b'~' | 0x81 => Some(RealtimeCommand::CycleStart),
    b'!' | 0x82 => Some(RealtimeCommand::FeedHold),
    0x18 => Some(RealtimeCommand::SoftReset),
    0x19 => Some(RealtimeCommand::Stop),
    0x83 => Some(RealtimeCommand::ParserStateReport),
    0x84 => Some(RealtimeCommand::SafetyDoor),
    0x85 => Some(RealtimeCommand::JogCancel),
    0x86 => Some(RealtimeCommand::ProgramStop),
    0x87 => Some(RealtimeCommand::FullStatusReport),
    0x8C => Some(RealtimeCommand::ToggleAutoReport),
    0x88 => Some(RealtimeCommand::ToggleOptionalStop),
    // Feed (0x90-0x94), rapid (0x95-0x97), spindle (0x99-0x9E), coolant (0xA0-0xA1), tool/probe
    // (0xA3-0xA4) overrides. Preserve the raw byte for the override decoder.
    0x90..=0x9E | 0xA0..=0xA4 => Some(RealtimeCommand::Override(byte)),
    _ => None,
  }
}

/// The outcome of feeding one byte to a [`LineReader`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LineEvent<'a> {
  /// The byte was buffered (or was a swallowed second half of a `CRLF`/`LFCR`); nothing to emit yet.
  Pending,
  /// A complete line is ready. The slice borrows the reader's buffer and is valid until the next
  /// [`LineReader::feed`]. An empty slice is a bare/empty line (a terminator with no content), which the
  /// orchestrator treats as the grblHAL error-state recovery trigger.
  Line(&'a [u8]),
  /// The current line exceeded [`MAX_LINE_LEN`]; the reader has entered an overflow state and will
  /// discard bytes until the next terminator. The orchestrator must respond `error:15`.
  Overflow,
}

/// The pending terminator-collapse state, so a `CRLF`/`LFCR` split across two `feed` calls still counts
/// as one terminator. After a terminator byte we record which one it was; if the very next byte is the
/// complementary terminator (`\n` after `\r`, or `\r` after `\n`), it is swallowed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PendingTerminator {
  /// No terminator was just seen.
  None,
  /// The previous byte was `\r`; a following `\n` is the second half of a `CRLF` and is swallowed.
  Cr,
  /// The previous byte was `\n`; a following `\r` is the second half of an `LFCR` and is swallowed.
  Lf,
}

/// Frames an incoming byte stream into lines. Accumulates printable bytes into a fixed buffer and emits a
/// [`LineEvent::Line`] on each terminator, collapsing `CRLF`/`LFCR` (including across `feed` calls) into a
/// single terminator so a host never sees a double-`ok`. Over-length lines yield [`LineEvent::Overflow`].
/// Real-time bytes are NOT handled here — they are diverted by the orchestrator before the reader sees a
/// byte — so the line buffer is never disturbed by an interleaved `?`/`!`.
#[derive(Debug)]
pub struct LineReader {
  buf: Vec<u8, MAX_LINE_LEN>,
  pending: PendingTerminator,
  overflowed: bool,
  /// Set when the previous byte completed a line. The completed line stays in `buf` so the caller can
  /// borrow it; the buffer is cleared lazily on the next content/terminator byte rather than eagerly, so
  /// the returned [`LineEvent::Line`] slice stays valid until the caller's next [`feed`](LineReader::feed).
  line_done: bool,
}

impl Default for LineReader {
  fn default() -> Self {
    Self::new()
  }
}

impl LineReader {
  /// Construct an empty line reader.
  pub const fn new() -> Self {
    Self {
      buf: Vec::new(),
      pending: PendingTerminator::None,
      overflowed: false,
      line_done: false,
    }
  }

  /// Discard any partially accumulated line and clear overflow/terminator state. Called on soft reset so
  /// a fresh stream is not contaminated by a half-buffered line from before the reset.
  pub fn reset(&mut self) {
    self.buf.clear();
    self.pending = PendingTerminator::None;
    self.overflowed = false;
    self.line_done = false;
  }

  /// Clear the previously completed line, if any, before assembling the next one. Called at the top of
  /// each `feed` that is not the swallowed half of a split terminator, so a completed line lives exactly
  /// from the `feed` that completed it until the next meaningful `feed`.
  fn rotate(&mut self) {
    if self.line_done {
      self.buf.clear();
      self.line_done = false;
    }
  }

  /// Feed one byte. Returns [`LineEvent::Line`] (borrowing the internal buffer) when a terminator
  /// completes a line, [`LineEvent::Overflow`] when the line is too long, or [`LineEvent::Pending`]
  /// otherwise. The returned `Line` slice is valid until the next call to `feed`.
  pub fn feed(&mut self, byte: u8) -> LineEvent<'_> {
    // Collapse the second half of a split CRLF/LFCR: a \n right after \r (or \r right after \n) is the
    // same single terminator and must be swallowed without emitting a second (empty) line. This case
    // does NOT rotate, so a line completed by the first half survives the swallowed second half.
    match (self.pending, byte) {
      (PendingTerminator::Cr, b'\n') | (PendingTerminator::Lf, b'\r') => {
        self.pending = PendingTerminator::None;
        return LineEvent::Pending;
      }
      _ => {}
    }

    // A new meaningful byte: drop any line we completed on a prior feed before touching the buffer.
    self.rotate();

    match byte {
      b'\r' | b'\n' => {
        self.pending = if byte == b'\r' { PendingTerminator::Cr } else { PendingTerminator::Lf };
        if self.overflowed {
          // The line was too long; we already reported overflow. Reset for the next line and emit
          // nothing here so the host sees exactly one error for the over-length line.
          self.buf.clear();
          self.overflowed = false;
          return LineEvent::Pending;
        }
        self.line_done = true;
        LineEvent::Line(self.buf.as_slice())
      }
      _ => {
        self.pending = PendingTerminator::None;
        if self.overflowed {
          // Still discarding the rest of an over-length line until its terminator arrives.
          return LineEvent::Pending;
        }
        if self.buf.push(byte).is_err() {
          // Buffer is full: enter overflow, drop the buffered partial, and report once. Subsequent bytes
          // of this line are discarded until the terminator.
          self.overflowed = true;
          self.buf.clear();
          return LineEvent::Overflow;
        }
        LineEvent::Pending
      }
    }
  }

}

/// The decision the line framer surfaces to the driver for one consumed byte. Exactly one of
/// [`AcceptLine`](EngineEvent::AcceptLine) or [`Reject`](EngineEvent::Reject) is produced per completed
/// line so the driver emits exactly one `ok`/`error:N` — the sole host flow-control signal.
///
/// Note: this engine performs *only* line framing. Real-time byte interception is no longer done here —
/// the `firmware` bin's USB reader half extracts real-time bytes with [`classify_realtime`] before any
/// byte reaches this framer (the realtime path must never block behind line back-pressure). Likewise the
/// grblHAL gcode error-hold lives downstream in the single in-order consumer, not here, because the framer
/// cannot know a forwarded line will error. Blank lines are therefore forwarded as an empty
/// [`AcceptLine`], not acknowledged here: the consumer owns both the bare `ok` and the blank-line
/// hold-recovery, keeping recovery in strict line order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EngineEvent<'a> {
  /// Mid-line, or a swallowed terminator half: nothing for the driver to do.
  None,
  /// A complete line is accepted for framing. The driver forwards it (including an empty line) to the
  /// single in-order consumer; the one `ok`/`error:N` is emitted once the line is consumed downstream. The
  /// slice borrows the engine's line buffer and is valid until the next [`StreamEngine::ingest`].
  AcceptLine(&'a [u8]),
  /// A complete line is rejected at the protocol layer (over-length). The driver emits `error:15`
  /// immediately and does not forward the line.
  Reject(u8),
}

/// The line framer the `firmware` bin's line-assembly half drives. It wraps a [`LineReader`] and surfaces
/// one [`EngineEvent`] per byte: nothing mid-line, an [`AcceptLine`](EngineEvent::AcceptLine) on each
/// completed line (blank lines included, as an empty slice), or a [`Reject`](EngineEvent::Reject) on an
/// over-length line. It owns no I/O, no real-time classification, and no error-hold — those concerns moved
/// to the reader half and the downstream consumer respectively (see [`EngineEvent`]).
#[derive(Debug, Default)]
pub struct StreamEngine {
  reader: LineReader,
}

impl StreamEngine {
  /// Construct a fresh engine with an empty line buffer.
  pub const fn new() -> Self {
    Self { reader: LineReader::new() }
  }

  /// Frame one received byte. Returns [`AcceptLine`](EngineEvent::AcceptLine) on a completed line (an empty
  /// slice for a bare/blank line), [`Reject`](EngineEvent::Reject) with `error:15` on overflow, or
  /// [`None`](EngineEvent::None) mid-line. Real-time bytes never reach here — the reader half diverts them.
  pub fn ingest(&mut self, byte: u8) -> EngineEvent<'_> {
    match self.reader.feed(byte) {
      LineEvent::Pending => EngineEvent::None,
      LineEvent::Overflow => EngineEvent::Reject(ERROR_LINE_OVERFLOW),
      LineEvent::Line(line) => EngineEvent::AcceptLine(line),
    }
  }

  /// Discard any partially accumulated line in response to a soft reset, so a fresh stream is not
  /// contaminated by a half-buffered line from before the reset. The driver calls this on `0x18` after
  /// flushing its downstream queues.
  pub fn soft_reset(&mut self) {
    self.reader.reset();
  }
}

/// The active motion mode reported as the first word of a `$G` (`[GC:...]`) parser-state line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParserMotion {
  /// G0 rapid positioning.
  Rapid,
  /// G1 linear feed move.
  Linear,
  /// G2 clockwise arc.
  ArcCw,
  /// G3 counter-clockwise arc.
  ArcCcw,
}

impl ParserMotion {
  /// The `G<n>` word for this motion mode.
  fn word(self) -> &'static str {
    match self {
      ParserMotion::Rapid => "G0",
      ParserMotion::Linear => "G1",
      ParserMotion::ArcCw => "G2",
      ParserMotion::ArcCcw => "G3",
    }
  }
}

/// The active spindle state (modal group 7) reported in a `$G` line: M3/M4 when running, M5 when stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParserSpindle {
  /// M3 — spindle on, clockwise.
  Clockwise,
  /// M4 — spindle on, counter-clockwise.
  CounterClockwise,
  /// M5 — spindle stop (the power-on default).
  Stop,
}

impl ParserSpindle {
  /// The `M<n>` word for this spindle state.
  fn word(self) -> &'static str {
    match self {
      ParserSpindle::Clockwise => "M3",
      ParserSpindle::CounterClockwise => "M4",
      ParserSpindle::Stop => "M5",
    }
  }
}

/// The active plane (modal group 2) reported in a `$G` line: G17 XY / G18 ZX / G19 YZ.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParserPlane {
  /// G17 — the XY plane (the power-on default).
  #[default]
  XY,
  /// G18 — the ZX plane.
  ZX,
  /// G19 — the YZ plane.
  YZ,
}

impl ParserPlane {
  /// The `G<n>` word for this plane.
  fn word(self) -> &'static str {
    match self {
      ParserPlane::XY => "G17",
      ParserPlane::ZX => "G18",
      ParserPlane::YZ => "G19",
    }
  }
}

/// The active coolant state (modal group 8) reported in a `$G` line. Mist (M7) and flood (M8) are independent and
/// can both be active; M9 (all off) is the default. The formatter renders the active word(s): `M9` when both are
/// off, `M7` / `M8` for a single circuit, or `M7 M8` for both — matching grbl's `$G` group-8 output.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ParserCoolant {
  /// Mist coolant (M7) active.
  pub mist: bool,
  /// Flood coolant (M8) active.
  pub flood: bool,
}

/// The active units mode reported in a `$G` line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParserUnits {
  /// G20 inch units.
  Inch,
  /// G21 millimeter units.
  Millimeter,
}

impl ParserUnits {
  /// The `G<n>` word for this units mode.
  fn word(self) -> &'static str {
    match self {
      ParserUnits::Inch => "G20",
      ParserUnits::Millimeter => "G21",
    }
  }
}

/// The active distance mode reported in a `$G` line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParserDistance {
  /// G90 absolute distance.
  Absolute,
  /// G91 incremental distance.
  Incremental,
}

impl ParserDistance {
  /// The `G<n>` word for this distance mode.
  fn word(self) -> &'static str {
    match self {
      ParserDistance::Absolute => "G90",
      ParserDistance::Incremental => "G91",
    }
  }
}

/// The active feed-rate mode reported in a `$G` line (modal group 5, DOC-10.2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParserFeedMode {
  /// G93 inverse-time feed.
  InverseTime,
  /// G94 units-per-minute feed (the grbl power-on default).
  UnitsPerMin,
}

impl ParserFeedMode {
  /// The `G<n>` word for this feed mode.
  fn word(self) -> &'static str {
    match self {
      ParserFeedMode::InverseTime => "G93",
      ParserFeedMode::UnitsPerMin => "G94",
    }
  }
}

/// A `Copy` snapshot of the live parser modal state the `$G` formatter renders. The `firmware` bin builds
/// this from the consumer's persistent `gcode::Parser` (via its `state()`) so the `[GC:...]` line reports
/// the real motion/units/distance/feed/spindle words rather than a hardcoded constant. Keeping the
/// formatter pure over a small snapshot — rather than importing the parser's modal type here — keeps
/// `protocol` free of any GCode-parsing coupling and the `$G` rendering host-testable in isolation.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ParserSnapshot {
  /// Active motion mode (modal group 1) — the first `$G` word.
  pub motion: ParserMotion,
  /// Active units mode (modal group 6).
  pub units: ParserUnits,
  /// Active distance mode (modal group 3).
  pub distance: ParserDistance,
  /// Active feed-rate mode (modal group 5): reported as `G93`/`G94`.
  pub feed_mode: ParserFeedMode,
  /// Active work coordinate system (modal group 12): 0 = G54 … 5 = G59. Reported in `$G` as `G54`…`G59`.
  pub wcs: usize,
  /// Whether a dynamic tool-length offset (`G43.1`) is active (modal group 8): reported as `G43.1` when set,
  /// `G49` when clear.
  pub tlo_active: bool,
  /// Programmed feed rate (modal F), in the active units per minute.
  pub feed: f32,
  /// Active spindle state (modal group 7): reported as `M3`/`M4`/`M5`.
  pub spindle: ParserSpindle,
  /// Programmed spindle speed (modal S), in RPM.
  pub spindle_rpm: u16,
  /// Active plane (modal group 2): reported as `G17`/`G18`/`G19`.
  pub plane: ParserPlane,
  /// Active coolant state (modal group 8): reported as `M9` / `M7` / `M8` / `M7 M8`.
  pub coolant: ParserCoolant,
  /// The CURRENT (active) tool number, reported as `T<n>` (`T0` = no tool selected). Committed by `M6` from the
  /// pending `T` word; persists across program end / soft reset (grbl keeps the physically-loaded tool).
  pub tool: u16,
}

impl ParserSnapshot {
  /// The grbl power-on modal defaults (G0 rapid, mm, absolute, G54, G49, no feed, spindle off). Used before
  /// any motion word has been parsed, and as the basis for partial snapshots in tests.
  pub const fn power_on() -> Self {
    Self {
      motion: ParserMotion::Rapid,
      units: ParserUnits::Millimeter,
      distance: ParserDistance::Absolute,
      feed_mode: ParserFeedMode::UnitsPerMin,
      wcs: 0,
      tlo_active: false,
      feed: 0.0,
      spindle: ParserSpindle::Stop,
      spindle_rpm: 0,
      plane: ParserPlane::XY,
      coolant: ParserCoolant { mist: false, flood: false },
      tool: 0,
    }
  }
}

/// Formatting failure: the destination buffer was too small to hold the rendered response. Callers size
/// their buffers from the constants below, so this is a programming error rather than a runtime
/// condition, but it is surfaced as a `Result` to honor the firmware-core no-`unwrap` rule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FmtError;

/// A capacity, in bytes, that comfortably holds any single response line this module renders (the longest is
/// a full status report or a build-info line). The `usb_tx` task allocates buffers of this size on the stack
/// / in a static pool. Sized for the Phase-E maximal report — `State` + `MPos`/`WPos` + `FS` + `Bf` + a full
/// `Pn:PXYZDHRS` + `WCO` + `Ov:200,100,200` with wide negative-millimeter positions — which approaches but
/// stays well under 160 bytes, so the formatter never returns [`FmtError`] for a real report.
pub const RESPONSE_CAPACITY: usize = 160;

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
    for (axis, value) in position.iter().enumerate() {
      if axis == 0 {
        write!(out, "{value:.3}").map_err(|_| FmtError)?;
      } else {
        write!(out, ",{value:.3}").map_err(|_| FmtError)?;
      }
    }
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
      for (axis, value) in snap.wco_mm.iter().enumerate() {
        if axis == 0 {
          write!(out, "{value:.3}").map_err(|_| FmtError)?;
        } else {
          write!(out, ",{value:.3}").map_err(|_| FmtError)?;
        }
      }
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
  /// `[OPT:]`; the extended report adds the grblHAL `[AXS:]`, `[NEWOPT:]`, and `[FIRMWARE:]` lines so a
  /// sender can detect an extended controller. The `[OPT:]` fields are, in order: options string, block
  /// buffer size, RX buffer size, axis count, tool-table entries — emitted in exactly the documented
  /// order so senders that position-parse OPT do not mis-read the buffer sizes. The caller appends `ok`.
  pub fn build_info<const N: usize>(out: &mut String<N>, extended: bool) -> Result<(), FmtError> {
    write!(out, "[VER:{VERSION}.20260616:]\r\n").map_err(|_| FmtError)?;
    write!(
      out,
      "[OPT:VNMSL,{},{},{},0]\r\n",
      BLOCK_BUFFER_SIZE, RX_BUFFER_SIZE, AXIS_COUNT,
    )
    .map_err(|_| FmtError)?;
    if extended {
      write!(out, "[AXS:{AXIS_COUNT}:XYZA]\r\n").map_err(|_| FmtError)?;
      // `ENUMS` advertises the runtime enumeration commands (`$ES`/`$EG`/`$EE`/`$EA`) so a sender builds its
      // settings/error/alarm UI from the controller instead of hardcoding; `RT+` advertises the top-bit-set
      // real-time command forms this module classifies; `SED` advertises the `$SED=<n>` per-setting description
      // command (Phase F). A sender reads these to know it may query the enumerations on connect.
      out.push_str("[NEWOPT:ENUMS,RT+,SED]\r\n").map_err(|_| FmtError)?;
      out.push_str("[FIRMWARE:grblHAL]\r\n").map_err(|_| FmtError)?;
    }
    Ok(())
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

/// A `Copy` snapshot of the coordinate model the `$#` NGC-parameters report renders: the six G54-G59 work
/// offsets, the G28/G30 predefined positions, the G92 offset, the TLO scalar, and the last-probe result. The
/// firmware bin fills this from the shared [`crate::coords::CoordinateSystems`] (plus the last probe, Phase C)
/// and hands it to [`ResponseWriter::ngc_parameter_line`]; keeping the formatter pure over a snapshot makes the
/// `$#` wire output host-testable byte-for-byte.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CoordinateReport {
  /// G54-G59 work-coordinate-system offsets in machine mm (index 0 = G54 … 5 = G59).
  pub wcs: [[f32; AXIS_COUNT]; 6],
  /// G28/G30 predefined positions in machine mm (index 0 = G28, 1 = G30).
  pub predefined: [[f32; AXIS_COUNT]; 2],
  /// The G92 offset in mm.
  pub g92: [f32; AXIS_COUNT],
  /// The dynamic tool-length offset scalar in mm (the Z axis), reported as `[TLO:z]`.
  pub tlo: f32,
  /// The last probe result in machine mm (`[PRB:x,y,z,a:flag]`). Filled by Phase C probing; zeros until then.
  pub probe: [f32; AXIS_COUNT],
  /// The last probe success flag (`1` = contact made, `0` = no contact / never probed).
  pub probe_success: bool,
}

impl Default for CoordinateReport {
  /// All offsets/positions zero, no probe — the power-on / first-boot `$#` view.
  fn default() -> Self {
    Self {
      wcs: [[0.0; AXIS_COUNT]; 6],
      predefined: [[0.0; AXIS_COUNT]; 2],
      g92: [0.0; AXIS_COUNT],
      tlo: 0.0,
      probe: [0.0; AXIS_COUNT],
      probe_success: false,
    }
  }
}

/// The last `G38.x` probe result (DOC-09, Phase C): the MACHINE position at the trigger instant in mm and the
/// contact flag. `Copy` so the firmware bin holds it behind a synchronous `Cell` (exactly the [`ControlState`] /
/// coordinate-model pattern) — the probe-cycle executor publishes the stop point and the `$#` / `[PRB:]` paths
/// read it. The power-on value is all-zero with `success = false`, matching grbl's "never probed" `[PRB:..:0]`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LastProbe {
  /// The MACHINE position at the probe trigger instant, in mm, `[X, Y, Z]`.
  pub position_mm: [f32; AXIS_COUNT],
  /// The contact flag: `true` if the expected probe edge was seen within travel, `false` otherwise (or never
  /// probed). Rendered as the trailing `:1`/`:0` of the `[PRB:]` line.
  pub success: bool,
}

impl LastProbe {
  /// The power-on / never-probed value: the origin with `success = false`, so `$#` shows `[PRB:0,0,0:0]` until a
  /// probe runs. `const` so it can seed a `static` cell without a lazy `Option`.
  pub const fn none() -> Self {
    LastProbe { position_mm: [0.0; AXIS_COUNT], success: false }
  }
}

impl Default for LastProbe {
  fn default() -> Self {
    Self::none()
  }
}

/// The per-line response a completed `G38.x` probe should produce, decided from the probe outcome and the mode's
/// alarm-on-fail flag (DOC-09, Phase C). Keeping this a pure host-tested decision means the firmware bin's probe
/// handler is a thin dispatch over a unit-tested truth table rather than ad-hoc branching.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ProbeResponse {
  /// The probe succeeded (saw its expected edge): emit a single `ok`, staying Idle. The `[PRB:..:1]` push already
  /// preceded it.
  Ok,
  /// The probe failed in an ALARMING mode (G38.2/.4): enter this alarm and emit `ALARM:N` (no `ok`). `ProbeFail-
  /// Initial` (4) when the probe was already at its expected edge before motion, else `ProbeFailContact` (5).
  Alarm(AlarmCode),
}

/// Decide a probe's per-line response (DOC-09): `triggered` is whether the expected edge was seen within travel;
/// `already_at_edge` is whether the probe was already at its expected stop edge before any motion (grbl's wrong
/// initial state); `alarm_on_fail` distinguishes the alarming modes (G38.2/.4) from the silent ones (G38.3/.5).
///
/// - A triggered probe → [`ProbeResponse::Ok`] (success, one `ok`).
/// - A non-triggered probe in a SILENT mode (G38.3/.5) → [`ProbeResponse::Ok`] (the sender checks `[PRB:..:0]`).
/// - A non-triggered probe in an ALARMING mode (G38.2/.4) → [`ProbeResponse::Alarm`]: `ALARM:4`
///   ([`AlarmCode::ProbeFailInitial`]) when `already_at_edge`, else `ALARM:5` ([`AlarmCode::ProbeFailContact`]).
pub fn probe_response(triggered: bool, already_at_edge: bool, alarm_on_fail: bool) -> ProbeResponse {
  if triggered || !alarm_on_fail {
    ProbeResponse::Ok
  } else if already_at_edge {
    ProbeResponse::Alarm(AlarmCode::ProbeFailInitial)
  } else {
    ProbeResponse::Alarm(AlarmCode::ProbeFailContact)
  }
}

/// The number of bracket lines the `$#` NGC-parameters block emits: G54-G59 (6) + G28 + G30 + G92 + TLO + PRB
/// = 11. The firmware bin loops `0..NGC_PARAMETER_LINES`, rendering one bracket per [`Response`], then `ok`.
pub const NGC_PARAMETER_LINES: usize = 11;

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

/// The `$#` bracket tags for the six work coordinate systems, indexed 0 = G54 … 5 = G59.
const WCS_TAGS: [&str; 6] = ["G54", "G55", "G56", "G57", "G58", "G59"];

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

/// A classified `$` system command. [`SystemCommand::classify`] maps the bytes AFTER the leading `$` to one
/// of these so the `firmware` bin dispatches a known set and rejects everything else with `error:3` — closing
/// the lenient fake-ack hole where any unmatched `$` was silently `ok`'d. Variants that carry a payload borrow
/// the input slice (no allocation); the classifier does no I/O and is fully host-tested. The recognized set is
/// the grblHAL Stage-1/Stage-2 surface: settings, build info, parser state, NGC params, the alarm/check/sleep
/// controls, startup lines, homing, restores, and the Galdr `$PBX` host-sync extension.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SystemCommand<'a> {
  /// Bare `$` — emit the help line.
  Help,
  /// `$$` — dump all settings.
  SettingsDump,
  /// `$I` (`extended = false`) / `$I+` (`extended = true`) — build info.
  BuildInfo { extended: bool },
  /// `$G` — parser-state (`[GC:...]`) report.
  ParserState,
  /// `$#` — NGC parameters dump.
  NgcParams,
  /// `$X` — kill alarm lock / unlock.
  Unlock,
  /// `$C` — toggle check mode.
  ToggleCheck,
  /// `$SLP` — enter sleep.
  Sleep,
  /// `$H` — run the homing cycle.
  Home,
  /// `$N` — query the stored startup lines.
  StartupQuery,
  /// `$N0=<gcode>` / `$N1=<gcode>` — store startup line `index` (0 or 1). The payload borrows the GCode text.
  StartupSet { index: u8, gcode: &'a [u8] },
  /// `$RST=$` — restore `$$` settings to defaults.
  RestoreSettings,
  /// `$RST=#` — zero G54–G59 work offsets / G28/G30 positions (DOC: coordinate data; Phase B).
  RestoreParams,
  /// `$RST=*` — restore ALL persisted data (settings, parameters, startup lines, build info).
  RestoreAll,
  /// `$<n>=<value>` — write setting `n`. The payload borrows the raw `<n>=<value>` text for the setter.
  SetSetting { body: &'a [u8] },
  /// `$ES` — enumerate every setting (`[SETTING:...]` lines), then `ok` (Phase F).
  EnumSettings,
  /// `$EG` — enumerate the setting groups (`[SETTINGGROUP:...]` lines), then `ok` (Phase F).
  EnumSettingGroups,
  /// `$EE` — enumerate the error codes (`[ERRORCODE:...]` lines), then `ok` (Phase F).
  EnumErrorCodes,
  /// `$EA` — enumerate the alarm codes (`[ALARMCODE:...]` lines), then `ok` (Phase F).
  EnumAlarmCodes,
  /// `$SED=<n>` — emit the `[SETTINGDESCR:<n>|...]` description for one setting, then `ok` (Phase F). The `id`
  /// is the parsed setting number; an unparseable id classifies to [`Unknown`](SystemCommand::Unknown).
  SettingDescription { id: u16 },
  /// `$PBX` — Galdr bulk settings export (host-sync).
  PbExport,
  /// `$PBX=<hex>` — Galdr bulk settings import chunk. The payload borrows the hex text.
  PbImport { hex: &'a [u8] },
  /// An unrecognized `$` command — the caller responds `error:3` ("'$' command not recognized"), NEVER a
  /// fabricated `ok`.
  Unknown,
}

/// The grbl `error:N` code for an unrecognized `$` system command ("'$' system command was not recognized").
/// Returned for any `$` input [`SystemCommand::classify`] maps to [`SystemCommand::Unknown`].
pub const ERROR_UNSUPPORTED_COMMAND: u8 = 3;

/// The grbl `error:N` code for `$H` when `$22` homing is not enabled ("Homing cycle is not enabled").
pub const ERROR_HOMING_DISABLED: u8 = 5;

impl<'a> SystemCommand<'a> {
  /// Classify the bytes AFTER the leading `$` into a [`SystemCommand`]. Leading/trailing ASCII whitespace in
  /// the caller's line is assumed already trimmed (the consumer trims before splitting on `$`), but the
  /// payload of a setting/startup/PB write is returned verbatim for its own parser. Anything not matched is
  /// [`Unknown`](SystemCommand::Unknown) so the caller rejects it rather than fake-acking.
  pub fn classify(rest: &'a [u8]) -> Self {
    match rest {
      b"" => SystemCommand::Help,
      b"$" => SystemCommand::SettingsDump,
      b"I" => SystemCommand::BuildInfo { extended: false },
      b"I+" => SystemCommand::BuildInfo { extended: true },
      b"G" => SystemCommand::ParserState,
      b"#" => SystemCommand::NgcParams,
      b"X" => SystemCommand::Unlock,
      b"C" => SystemCommand::ToggleCheck,
      b"SLP" => SystemCommand::Sleep,
      b"H" => SystemCommand::Home,
      b"N" => SystemCommand::StartupQuery,
      b"RST=$" => SystemCommand::RestoreSettings,
      b"RST=#" => SystemCommand::RestoreParams,
      b"RST=*" => SystemCommand::RestoreAll,
      b"ES" => SystemCommand::EnumSettings,
      b"EG" => SystemCommand::EnumSettingGroups,
      b"EE" => SystemCommand::EnumErrorCodes,
      b"EA" => SystemCommand::EnumAlarmCodes,
      b"PBX" => SystemCommand::PbExport,
      _ => Self::classify_with_payload(rest),
    }
  }

  /// The payload-carrying tail of [`classify`](SystemCommand::classify): the `$N0=`/`$N1=` startup writes, the
  /// `$PBX=` import, and the `$<n>=<value>` setting write, each of which borrows part of `rest`. Split out so
  /// the exact-match arms above stay a clean lookup table.
  fn classify_with_payload(rest: &'a [u8]) -> Self {
    if let Some(gcode) = rest.strip_prefix(b"N0=") {
      return SystemCommand::StartupSet { index: 0, gcode };
    }
    if let Some(gcode) = rest.strip_prefix(b"N1=") {
      return SystemCommand::StartupSet { index: 1, gcode };
    }
    if let Some(hex) = rest.strip_prefix(b"PBX=") {
      return SystemCommand::PbImport { hex };
    }
    // `$SED=<n>` (Phase F): a per-setting description query. The id must parse as a decimal `u16`; a
    // non-numeric or out-of-range id is genuinely unknown and errors rather than fake-acking.
    if let Some(id) = rest.strip_prefix(b"SED=") {
      return match core::str::from_utf8(id).ok().and_then(|s| s.trim().parse::<u16>().ok()) {
        Some(id) => SystemCommand::SettingDescription { id },
        None => SystemCommand::Unknown,
      };
    }
    // A `$<number>=<value>` setting write is the last recognized form: the head before `=` must be all ASCII
    // digits. Anything else (a `$`-prefixed token we do not know, a `$Nx=` with x>1, a non-numeric `$abc=`)
    // is genuinely unknown and must error rather than fake-ack.
    if let Some(eq) = rest.iter().position(|&b| b == b'=') {
      let (head, _) = rest.split_at(eq);
      if !head.is_empty() && head.iter().all(|b| b.is_ascii_digit()) {
        return SystemCommand::SetSetting { body: rest };
      }
    }
    SystemCommand::Unknown
  }

  /// Whether this is a pure READ-ONLY query — a reporting command with NO side effect on modal, planner, settings,
  /// or coordinate state — and so can be safely serviced WHILE A PAUSE HOLD IS ACTIVE (M0/M1/M6) without releasing
  /// the hold. grbl answers `$G`/`$#`/`$$`/`$I` etc. during a hold (motion is held, the protocol loop is not), so
  /// the firmware's pause loop services these in-place and emits their report + `ok`. Everything that WRITES or
  /// changes state — `$<n>=val`, `$RST=*`, `$N0=`, `$PBX=`, `$X`, `$C`, `$SLP`, `$H`, and an `Unknown` rejection —
  /// is NOT a read-only query: it must be deferred (left queued) until the hold resumes, so a held machine never
  /// mutates state or runs an action behind the operator's back. Used by [`run_program_pause`](crate) in the bin.
  pub fn is_readonly_query(&self) -> bool {
    matches!(
      self,
      SystemCommand::Help
        | SystemCommand::SettingsDump
        | SystemCommand::BuildInfo { .. }
        | SystemCommand::ParserState
        | SystemCommand::NgcParams
        | SystemCommand::StartupQuery
        | SystemCommand::EnumSettings
        | SystemCommand::EnumSettingGroups
        | SystemCommand::EnumErrorCodes
        | SystemCommand::EnumAlarmCodes
        | SystemCommand::SettingDescription { .. }
        | SystemCommand::PbExport
    )
  }
}

/// The grbl override bounds (DOC-08 §3, `docs/gcode-streaming.md`). Feed and spindle overrides clamp to
/// `[10, 200]` percent; the rapid override is one of three discrete values `{25, 50, 100}`. The default for
/// all three is 100 % (no scaling). These are single-sourced here so the real-time decoder, the executor's
/// scaling, and the host tests agree on one set of limits.
pub const OVERRIDE_MIN_PCT: u8 = 10;
/// The maximum feed / spindle override percentage (grbl's `MAX_FEED_RATE_OVERRIDE` / spindle equivalent).
pub const OVERRIDE_MAX_PCT: u8 = 200;
/// The default (no-scaling) override percentage for all three overrides, latched on boot and on a `100 %`
/// reset byte (`0x90`/`0x95`/`0x99`).
pub const OVERRIDE_DEFAULT_PCT: u8 = 100;

/// The live feed / rapid / spindle override percentages plus the spindle-stop and coolant toggle state,
/// mutated by the real-time override bytes (`0x90`–`0xA4`) and rendered as the `Ov:` status element. It is a
/// pure, `Copy` state machine so the increment / clamp / reset matrix for every byte is host-tested, and so
/// the `firmware` bin can hold it behind a synchronous `Cell` (the [`ControlState`] pattern) and mutate it
/// from the non-blocking real-time reader half without awaiting.
///
/// grbl applies an override the instant the byte arrives, without re-planning the queued blocks: the executor
/// reads the live value per segment and scales the step timing. A change is never acked (override bytes are
/// real-time commands) and must be visible in the next `?` (`Ov:` and the realized `FS:`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Overrides {
  /// Feed override percentage, clamped to `[OVERRIDE_MIN_PCT, OVERRIDE_MAX_PCT]`. Scales the programmed feed
  /// of G1/G2/G3 (and jog) moves.
  pub feed: u8,
  /// Rapid override percentage, one of `{25, 50, 100}`. Scales the rapid (G0) max-rate cruise.
  pub rapid: u8,
  /// Spindle override percentage, clamped to `[OVERRIDE_MIN_PCT, OVERRIDE_MAX_PCT]`. Scales the commanded RPM.
  pub spindle: u8,
  /// Spindle-stop toggle (`0x9E`): when `true` the spindle is force-stopped (overriding the commanded RPM)
  /// until toggled off. Reported via the accessory state; the actual spindle output is a DOC-05 stub.
  pub spindle_stop: bool,
  /// Flood-coolant toggle (`0xA0`): the latched flood on/off state. The actual coolant output is a DOC-06 stub.
  pub flood: bool,
  /// Mist-coolant toggle (`0xA1`): the latched mist on/off state. The actual coolant output is a DOC-06 stub.
  pub mist: bool,
}

impl Default for Overrides {
  fn default() -> Self {
    Self::new()
  }
}

impl Overrides {
  /// The power-on / post-reset overrides: all three at 100 %, no spindle-stop, coolant off. grbl resets the
  /// overrides to their defaults on a soft reset, so the `firmware` bin re-seeds this on `0x18`.
  pub const fn new() -> Self {
    Overrides {
      feed: OVERRIDE_DEFAULT_PCT,
      rapid: OVERRIDE_DEFAULT_PCT,
      spindle: OVERRIDE_DEFAULT_PCT,
      spindle_stop: false,
      flood: false,
      mist: false,
    }
  }

  /// Apply one real-time override byte (`0x90`–`0xA4`), mutating the relevant override in place with grbl's
  /// clamping. Returns `true` if any field changed (so the caller can signal the executor / spindle and the
  /// `Ov:` cadence), `false` if the byte left the state unchanged (grbl ignores a no-op override). Unknown
  /// bytes in the range (e.g. the unused `0x98`, the `0xA2`/`0xA3`/`0xA4` tool/probe bytes Phase E does not
  /// model) are accepted as no-ops, never panicking.
  pub fn apply(&mut self, byte: u8) -> bool {
    let before = *self;
    match byte {
      // Feed override: reset-100, ±10, ±1, clamped to [MIN, MAX].
      0x90 => self.feed = OVERRIDE_DEFAULT_PCT,
      0x91 => self.feed = clamp_override(self.feed as i16 + 10),
      0x92 => self.feed = clamp_override(self.feed as i16 - 10),
      0x93 => self.feed = clamp_override(self.feed as i16 + 1),
      0x94 => self.feed = clamp_override(self.feed as i16 - 1),
      // Rapid override: one of the three discrete grbl values.
      0x95 => self.rapid = 100,
      0x96 => self.rapid = 50,
      0x97 => self.rapid = 25,
      // Spindle override: reset-100, ±10, ±1, clamped to [MIN, MAX].
      0x99 => self.spindle = OVERRIDE_DEFAULT_PCT,
      0x9A => self.spindle = clamp_override(self.spindle as i16 + 10),
      0x9B => self.spindle = clamp_override(self.spindle as i16 - 10),
      0x9C => self.spindle = clamp_override(self.spindle as i16 + 1),
      0x9D => self.spindle = clamp_override(self.spindle as i16 - 1),
      // Spindle-stop toggle and the coolant toggles are boolean flips.
      0x9E => self.spindle_stop = !self.spindle_stop,
      0xA0 => self.flood = !self.flood,
      0xA1 => self.mist = !self.mist,
      // Any other byte in the classified override range (0x98 unused, 0xA2/0xA3/0xA4 tool/probe) is a no-op.
      _ => {}
    }
    *self != before
  }

  /// The realized feed rate in mm/min for a programmed feed, after the feed override and a clamp to the most
  /// restrictive axis max-rate. Scaling UP must never exceed the configured `$110-112` rate (grbl clamps the
  /// override-scaled feed to the machine's rate limits), so `max_rate_mm_min` is the per-block dominant rate
  /// ceiling the caller supplies (`f32::INFINITY` for "no clamp", e.g. when the ceiling is unknown).
  pub fn scaled_feed(&self, programmed_mm_min: f32, max_rate_mm_min: f32) -> f32 {
    let scaled = programmed_mm_min * (self.feed as f32 / 100.0);
    scaled.clamp(0.0, max_rate_mm_min.max(0.0))
  }

  /// The realized rapid rate in mm/min for a rapid (G0) cruise rate, after the rapid override. A rapid is
  /// already governed by the axis max-rate, so the override only ever scales it DOWN (25/50/100 %); no extra
  /// max-rate clamp is needed beyond the planner's existing rate limiting, but the value is floored at 0.
  pub fn scaled_rapid(&self, rapid_mm_min: f32) -> f32 {
    (rapid_mm_min * (self.rapid as f32 / 100.0)).max(0.0)
  }

  /// The realized spindle speed in RPM after the spindle override and the spindle-stop toggle. A `spindle_stop`
  /// forces 0 regardless of the commanded RPM (grbl's `0x9E` halts the spindle); otherwise the commanded RPM is
  /// scaled by the override percentage and rounded to whole RPM (the `FS:` field is integer RPM).
  pub fn scaled_rpm(&self, programmed_rpm: u16) -> u16 {
    if self.spindle_stop {
      return 0;
    }
    let scaled = programmed_rpm as f32 * (self.spindle as f32 / 100.0);
    libm::roundf(scaled.max(0.0)) as u16
  }
}

/// Clamp an override candidate (after an increment that may under/overflow the byte range) into the grbl
/// `[OVERRIDE_MIN_PCT, OVERRIDE_MAX_PCT]` band. The arithmetic is done in `i16` so a `-1`/`-10` from the
/// minimum cannot wrap a `u8`.
fn clamp_override(candidate: i16) -> u8 {
  candidate.clamp(OVERRIDE_MIN_PCT as i16, OVERRIDE_MAX_PCT as i16) as u8
}

/// How many status reports may pass without an `Ov:` element before a periodic refresh forces one. An alias for
/// the shared [`REFRESH_PERIOD`] (the `Ov:` cadence matches the `WCO:` one), kept so callers and tests can name
/// the override cadence specifically. The `Ov:` change/periodic logic itself lives in [`RefreshReporter`].
pub const OV_REFRESH_PERIOD: u16 = REFRESH_PERIOD;

/// The asserted machine input pins, rendered as the `Pn:` status element (DOC-08 §4). Each `bool` is the
/// LOGICAL asserted state (after any invert handling, which the `firmware` bin applies when it samples the
/// raw GPIO — e.g. the probe via [`crate::hal_traits::probe_triggered`]), so this type carries no electrical
/// or settings knowledge and the `Pn:` letter assembly stays a pure, host-tested function. grbl OMITS the
/// `Pn:` element entirely when no pin is asserted, which [`ResponseWriter::status_report`] honors.
///
/// The fields cover the grbl signal letters the Galdr hardware can source: `P` probe (wired today, Phase C),
/// `X`/`Y`/`Z` limits, `D` door, `H` feed-hold input, `R` reset/e-stop, `S` cycle-start input. The limit /
/// door / control inputs are DOC-06 hardware that is not wired yet, so the bin reports them `false` (a clean
/// stub) until the GPIO backend lands — but the assembly logic is complete and tested so wiring a pin is a
/// one-line change in the bin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PinReport {
  /// `P` — probe asserted (touch made). Wired today via the Phase-C probe input.
  pub probe: bool,
  /// `X`/`Y`/`Z` — limit switches asserted, `[X, Y, Z]` (DOC-06, not wired yet).
  pub limits: [bool; AXIS_COUNT],
  /// `D` — safety-door input asserted (DOC-06, not wired yet).
  pub door: bool,
  /// `H` — feed-hold input asserted (the optional FHOLD GPIO, DOC-06, not wired yet).
  pub hold: bool,
  /// `R` — reset / e-stop input asserted (DOC-06, not wired yet).
  pub reset: bool,
  /// `S` — cycle-start input asserted (the optional CYCSTART GPIO, DOC-06, not wired yet).
  pub cycle_start: bool,
}

impl PinReport {
  /// All pins unasserted — the power-on / quiescent view. `const` so it can seed the `const`
  /// [`MachineSnapshot::idle`] without a lazy default.
  pub const fn new_idle() -> Self {
    PinReport {
      probe: false,
      limits: [false; AXIS_COUNT],
      door: false,
      hold: false,
      reset: false,
      cycle_start: false,
    }
  }

  /// Whether any input pin is asserted. When `false`, [`ResponseWriter::status_report`] omits the `Pn:`
  /// element entirely (grbl's rule), so a quiescent machine's report never carries an empty `Pn:`.
  pub fn any(&self) -> bool {
    self.probe || self.limits.iter().any(|&l| l) || self.door || self.hold || self.reset || self.cycle_start
  }

  /// Append the asserted-pin letters to `out` in grbl's documented order (`P` probe, `X`/`Y`/`Z` limits, `D`
  /// door, `H` hold, `R` reset, `S` cycle-start), writing nothing for an unasserted pin. The caller wraps this
  /// with the `Pn:` tag only when [`any`](PinReport::any) is set. Returns [`FmtError`] only on a (never, with a
  /// correctly sized buffer) capacity failure.
  fn write_letters<const N: usize>(&self, out: &mut String<N>) -> Result<(), FmtError> {
    if self.probe {
      out.push('P').map_err(|_| FmtError)?;
    }
    // Limit letters in axis order, matching grbl's `X`/`Y`/`Z` signal letters.
    for (axis, letter) in ['X', 'Y', 'Z'].into_iter().enumerate() {
      if self.limits.get(axis).copied().unwrap_or(false) {
        out.push(letter).map_err(|_| FmtError)?;
      }
    }
    if self.door {
      out.push('D').map_err(|_| FmtError)?;
    }
    if self.hold {
      out.push('H').map_err(|_| FmtError)?;
    }
    if self.reset {
      out.push('R').map_err(|_| FmtError)?;
    }
    if self.cycle_start {
      out.push('S').map_err(|_| FmtError)?;
    }
    Ok(())
  }
}

#[cfg(test)]
mod tests {
  // firmware-core is `#![no_std]`; `std` only links under `#[cfg(test)]`. The recording helpers below
  // collect events into a `std::vec::Vec` to keep the byte-level assertions readable.
  extern crate std;
  use std::string::ToString;
  use std::vec::Vec as StdVec;

  use super::*;

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
  }

  #[test]
  fn locked_alarms_are_the_critical_subset() {
    // Codes 1, 2, 10 are locked (cleared only by a soft reset); the rest accept `$X`.
    for code in [AlarmCode::HardLimit, AlarmCode::SoftLimit, AlarmCode::EStop] {
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
    // name/description, and the codes must be the canonical grbl numbers (1,2,3,4,5,8,10,11).
    let codes: StdVec<u8> = AlarmCode::ALL.iter().map(|a| a.code()).collect();
    assert_eq!(codes, std::vec![1, 2, 3, 4, 5, 8, 10, 11]);
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
