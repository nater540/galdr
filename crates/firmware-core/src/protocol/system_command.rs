//! `$` system-command classification (`SystemCommand`).

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
