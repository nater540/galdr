//! Pure parsing of the firmware's `$`-settings traffic: the `$<n>=<value>` value lines a `$$` dump emits, and
//! the `[SETTING:...]` enumeration rows a `$ES` query emits.
//!
//! Per `docs/gcode-streaming.md`, settings travel as text: `$$` dumps every setting as one `$<n>=<value>` line
//! (terminated by a final `ok`), a single `$<n>=<value>` write is acknowledged with `ok`/`error:N`, and the
//! grblHAL `$ES` query enumerates each setting's static metadata as `[SETTING:<id>|<group>|<name>|<unit>|
//! <datatype>|<format>|<min>|<max>]`. The doc is emphatic that a sender must *not* hardcode a settings UI —
//! it should learn the setting list and labels from the controller — so we parse both the live values and the
//! enumeration metadata here, keeping the grammar pure and unit-tested. The interactive per-field edit path is
//! these text lines; the binary `$PBX`/galdr-proto bulk channel is a separate whole-record optimisation, not
//! modelled here (see the settings model's module note).
//!
//! Parsing is forgiving in the grblHAL spirit: a value is kept verbatim as a string (settings are integers,
//! floats, bitmasks, and booleans with per-setting formatting, so the model stores the text and the firmware
//! validates a write), and an unparseable or non-setting line yields `None` rather than failing.

/// One live setting value parsed from a `$<n>=<value>` line: the setting number and its value text exactly as
/// the firmware rendered it (e.g. `"10"`, `"0.010"`, `"1"`). The value is kept as a string because settings
/// span integers, floats with fixed precision, bitmasks, and 0/1 booleans; the firmware is the authority on
/// formatting and range, so the host preserves the text and echoes edits back through the same `$<n>=<value>`
/// path for the firmware to validate.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SettingValue {
  /// The grbl setting number (`$0`, `$100`, ...).
  pub number: u32,
  /// The value text, trimmed, exactly as the firmware rendered it.
  pub value: String,
}

/// One enumeration row parsed from a `[SETTING:<id>|<group>|<name>|<unit>|<datatype>|<format>|<min>|<max>]`
/// line (grblHAL `$ES`). Carries the static metadata the UI labels a setting with; the live value comes from
/// the separate `$$` dump. Only the fields the panel renders are modelled; the rest are kept verbatim.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SettingMeta {
  /// The grbl setting number this row describes.
  pub number: u32,
  /// The setting group id (an opaque integer the firmware assigns; used only for ordering/section hints).
  pub group: u32,
  /// The human-readable setting name (e.g. `"Step pulse time"`).
  pub name: String,
  /// The unit string (e.g. `"microseconds"`, `"mm/min"`), empty when the setting is unitless.
  pub unit: String,
  /// The minimum value text, if the firmware advertised one (empty field → `None`).
  pub min: Option<String>,
  /// The maximum value text, if the firmware advertised one (empty field → `None`).
  pub max: Option<String>,
}

/// Parse a `$<n>=<value>` settings line into a [`SettingValue`], or `None` if `line` is not one. The line must
/// start with `$`, the part before `=` must parse as a setting number, and there must be a `=`. Startup-block
/// writes (`$N0=...`, `$N1=...`) and other `$`-prefixed commands (`$J=`, `$H`, ...) are deliberately rejected:
/// their key is not a bare number, so they never masquerade as a setting. Any trailing parenthetical
/// description (`$0=10 (Step pulse time)`) is stripped so the stored value is just the number text.
pub fn parse_setting_value(line: &str) -> Option<SettingValue> {
  let body = line.trim().strip_prefix('$')?;
  let (key, value) = body.split_once('=')?;
  let number = key.trim().parse::<u32>().ok()?;
  // grblHAL may append a `(description)` after the value when asked; keep only the value text before it.
  let value = value.split('(').next().unwrap_or(value).trim().to_string();
  Some(SettingValue { number, value })
}

/// Parse a `[SETTING:...]` enumeration row body (the text already stripped of its square brackets, as
/// [`crate::protocol::response::parse_line`] delivers it via [`crate::protocol::Response::Message`]) into a
/// [`SettingMeta`], or `None` if it is not a `SETTING:` row or lacks the id/name. The body is
/// `SETTING:<id>|<group>|<name>|<unit>|<datatype>|<format>|<min>|<max>`; missing trailing fields are tolerated
/// (an older or terser firmware may emit fewer), and empty unit/min/max collapse to empty/`None`.
pub fn parse_setting_meta(message_body: &str) -> Option<SettingMeta> {
  let rest = message_body.strip_prefix("SETTING:")?;
  let mut fields = rest.split('|');
  let number = fields.next()?.trim().parse::<u32>().ok()?;
  // Group is opaque; default to 0 if absent or unparseable rather than rejecting the whole row.
  let group = fields.next().and_then(|g| g.trim().parse::<u32>().ok()).unwrap_or(0);
  let name = fields.next()?.trim().to_string();
  let unit = fields.next().unwrap_or("").trim().to_string();
  // Skip datatype and format (positions 5 and 6) — the panel does not render them yet.
  let _datatype = fields.next();
  let _format = fields.next();
  let min = fields.next().map(str::trim).filter(|s| !s.is_empty()).map(str::to_string);
  let max = fields.next().map(str::trim).filter(|s| !s.is_empty()).map(str::to_string);
  Some(SettingMeta { number, group, name, unit, min, max })
}

/// Build the `$<n>=<value>` write line (no terminator — the engine appends `\n` when it counts the line) that
/// sets setting `number` to `value`. The single place the host forms a settings write, so the wire syntax is
/// in one tested spot. The value is trimmed; the caller is responsible for it being well-formed for the
/// setting (the firmware validates and answers `error:N` if not).
pub fn setting_write_line(number: u32, value: &str) -> String {
  format!("${}={}", number, value.trim())
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn parses_an_integer_setting_line() {
    assert_eq!(parse_setting_value("$0=10"), Some(SettingValue { number: 0, value: "10".to_string() }));
  }

  #[test]
  fn parses_a_float_and_a_high_numbered_setting() {
    assert_eq!(parse_setting_value("$11=0.010"), Some(SettingValue { number: 11, value: "0.010".to_string() }));
    assert_eq!(parse_setting_value("$132=80.000"), Some(SettingValue { number: 132, value: "80.000".to_string() }));
  }

  #[test]
  fn strips_a_trailing_parenthetical_description() {
    // grblHAL can append a human description in parentheses; the stored value is just the number text.
    assert_eq!(
      parse_setting_value("$0=10.0 (Step pulse time, microseconds)"),
      Some(SettingValue { number: 0, value: "10.0".to_string() })
    );
  }

  #[test]
  fn rejects_non_setting_dollar_commands() {
    // Startup-block writes and other `$` commands have a non-numeric key and must not be read as settings.
    assert_eq!(parse_setting_value("$N0=G54 G21"), None);
    assert_eq!(parse_setting_value("$J=G91 X1 F100"), None);
    assert_eq!(parse_setting_value("$H"), None, "a `$` command with no `=` is not a setting");
    assert_eq!(parse_setting_value("ok"), None, "a plain response is not a setting");
  }

  #[test]
  fn parses_a_full_enumeration_row() {
    let meta = parse_setting_meta("SETTING:0|1|Step pulse time|microseconds|2||1|1000").expect("a SETTING row");
    assert_eq!(meta, SettingMeta {
      number: 0,
      group: 1,
      name: "Step pulse time".to_string(),
      unit: "microseconds".to_string(),
      min: Some("1".to_string()),
      max: Some("1000".to_string()),
    });
  }

  #[test]
  fn enumeration_tolerates_empty_unit_and_missing_bounds() {
    // A unitless setting with no advertised bounds: unit empty, min/max None — still a valid row.
    let meta = parse_setting_meta("SETTING:2|1|Step pulse invert||3|").expect("a SETTING row");
    assert_eq!(meta.unit, "");
    assert_eq!(meta.min, None);
    assert_eq!(meta.max, None);
    assert_eq!(meta.name, "Step pulse invert");
  }

  #[test]
  fn enumeration_rejects_non_setting_messages() {
    assert_eq!(parse_setting_meta("OPT:VNMSL,100,1024,3,0"), None);
    assert_eq!(parse_setting_meta("MSG:hello"), None);
    assert_eq!(parse_setting_meta("SETTING:"), None, "a SETTING row with no id is rejected");
  }

  #[test]
  fn builds_a_write_line() {
    assert_eq!(setting_write_line(0, "10"), "$0=10");
    assert_eq!(setting_write_line(110, " 800.000 "), "$110=800.000", "the value is trimmed");
  }
}
