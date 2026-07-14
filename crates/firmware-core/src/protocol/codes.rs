//! grblHAL `error:N` / `ALARM:N` code tables and the wedge-reset fail-safe mapping.

/// The grblHAL `error:N` code for an over-length input line ("line length exceeded").
pub const ERROR_LINE_OVERFLOW: u8 = 15;

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
    id: 8,
    name: "Not idle",
    description: "Grbl '$' command cannot be used unless Grbl is IDLE. Ensures smooth operation during a job.",
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
  /// `ALARM:17` motor / motion fault (grblHAL `Alarm_MotorFault`). Galdr raises this when the core-1 motion
  /// executor hits an unrecoverable mid-block step-output transport fault — a swallowed RMT `wait()`/`transmit()`
  /// error that abandons the tail of a cutting move (§15.6 / task #22). The step sync is broken, so on open-loop
  /// steppers position certainty is LOST; per the grbl lost-step-sync contract this is a LOCKED alarm requiring a
  /// soft reset + re-home, NEVER a silent resume. Chosen as grblHAL's canonical motor-fault code so senders label
  /// it correctly; it does not collide with any other code this firmware emits (1,2,3,4,5,8,10,11).
  MotorFault,
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
      AlarmCode::MotorFault => 17,
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
    AlarmCode::MotorFault,
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
      AlarmCode::MotorFault => "Motor fault",
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
      AlarmCode::MotorFault => {
        "Motor fault. The step output to a motor failed mid-move. Machine position is likely lost. Re-homing is required."
      }
    }
  }

  /// Whether this is a *locked* critical alarm. In a locked alarm grblHAL answers only real-time report
  /// requests until a soft reset clears the physical cause (codes 1, 2, 10 per DOC-08 §5); `$X` cannot
  /// unlock these. The non-locked alarms (abort, probe-fail, homing-required) still accept `$` commands,
  /// so `$X` unlocks them. Drives the consumer's "accept `$X`?" decision, kept here so it is host-tested.
  pub fn is_locked(self) -> bool {
    matches!(self, AlarmCode::HardLimit | AlarmCode::SoftLimit | AlarmCode::EStop | AlarmCode::MotorFault)
  }

  /// The `[MSG:...]` push text grbl prints on entering this alarm: the homing-required / locked-critical
  /// alarms prompt `'$H'|'$X' to unlock`, while the recoverable ones prompt `Reset to continue`. The
  /// caller wraps this in the `[MSG:...]` envelope via [`ResponseWriter::message`].
  pub fn unlock_hint(self) -> &'static str {
    match self {
      // Homing-required and the locked-critical alarms are cleared by homing or unlocking (or, for the
      // locked ones, a soft reset after the cause clears). The same prompt grbl uses fits all of them.
      AlarmCode::HomingRequired | AlarmCode::HardLimit | AlarmCode::SoftLimit | AlarmCode::EStop
      | AlarmCode::MotorFault => "'$H'|'$X' to unlock",
      // The recoverable alarms (abort-during-cycle, probe-fail, homing-fail) tell the operator to reset and retry.
      AlarmCode::AbortDuringCycle | AlarmCode::ProbeFailInitial | AlarmCode::ProbeFailContact
      | AlarmCode::HomingFail => "Reset to continue",
    }
  }
}

/// The fail-safe alarm the board comes up in after a core-0 EXECUTOR-STALL watchdog reset (Design A, §20): the machine
/// must come up LOCKED so it can NEVER silently resume in a now-suspect position. With `$22` homing ENABLED the natural
/// lock is [`AlarmCode::HomingRequired`] (`ALARM:11`) — clear it by re-homing (`$H`), which re-establishes machine zero
/// (the user's chosen policy). With homing DISABLED there is no machine reference to re-home to, so
/// [`AlarmCode::AbortDuringCycle`] (`ALARM:3`, grbl's "reset while in motion — position lost") is the coherent fail-safe:
/// it rejects streaming until the operator acknowledges (reset / `$X`) and manually re-zeros — never a silent `Idle`
/// resume (the homing-disabled gap this closes). Pure so the boot-gating decision is host-tested.
pub fn wedge_reset_alarm(homing_enabled: bool) -> AlarmCode {
  if homing_enabled {
    AlarmCode::HomingRequired
  } else {
    AlarmCode::AbortDuringCycle
  }
}

/// The grbl `error:N` code for an unrecognized `$` system command ("'$' system command was not recognized").
/// Returned for any `$` input [`SystemCommand::classify`] maps to [`SystemCommand::Unknown`].
pub const ERROR_UNSUPPORTED_COMMAND: u8 = 3;

/// The grbl `error:N` code for `$H` when `$22` homing is not enabled ("Homing cycle is not enabled").
pub const ERROR_HOMING_DISABLED: u8 = 5;

/// The grbl `error:N` code for a settings-mutating `$` command issued while the machine is not Idle (or
/// Alarmed): grbl's "'$' command cannot be used unless Grbl is IDLE" (`STATUS_IDLE_ERROR`). Gates `$n=val`,
/// `$Nx=`, `$RST=`, and the `$PBX` import — see [`ControlState::settings_write_allowed`].
pub const ERROR_NOT_IDLE: u8 = 8;
