//! Human-readable decoding of grbl / grblHAL `error:N` and `ALARM:N` codes.
//!
//! A bare `error:21` or `ALARM:1` tells the operator nothing; every place skirnir surfaces a code — the
//! console line, the alarm/error banner — should read `error:21 — Modal group violation` and, on hover or in
//! the banner detail, the full sentence explaining it. This module is the single source of that decoding.
//!
//! There are two layers, and a lookup always returns something:
//!   1. A built-in **canonical fallback** table ([`error_text`]/[`alarm_text`]) covering the common grbl 1.1 /
//!      grblHAL set, with a generic gloss for anything unknown so a number is never shown alone. The
//!      firmware-specific codes match the firmware's own `$EE`/`$EA` text verbatim, so the static fallback and
//!      the runtime-enriched text read identically (a `$EA` Refresh must never visibly change the wording).
//!   2. A runtime **[`CodeBook`]** of overrides, populated from the firmware's `[ERRORCODE:...]`/`[ALARMCODE:...]`
//!      enumeration (answered to `$EE`/`$EA`). An override beats the static table; absent one, the static table
//!      answers. The codebook is per-session and cleared on disconnect, like the settings model.
//!
//! ## Allocation discipline
//! [`CodeText`]'s fields are [`Cow<'static, str>`]: the static tables return [`Cow::Borrowed`] (no heap), and
//! only the enumeration parser produces [`Cow::Owned`]. The banner runs every repaint while a fault is shown,
//! so it reads through the field-specific borrowing accessors ([`CodeBook::alarm_description`] etc.) that hand
//! back a borrow on the static path and clone only on a runtime override — neither the banner nor the console
//! allocates the field it does not use.
//!
//! Parsing mirrors [`crate::protocol::settings::parse_setting_meta`]: the row body arrives already stripped of
//! its square brackets (as [`crate::protocol::Response::Message`] delivers it), and a non-row or an id-less row
//! yields `None` rather than failing. The wire form is `ERRORCODE:<id>|<name>|<description>` (and the alarm
//! analogue) — keep this in lock-step with the firmware's enumeration emitter.

use std::borrow::Cow;
use std::collections::BTreeMap;

/// One decoded code entry: a short `name` (for the one-line console/badge gloss) and a longer `description`
/// (for the banner detail / hover). Both are [`Cow<'static, str>`] so the static fallback can borrow its
/// constants with no allocation while a runtime-enumerated entry owns its parsed strings; lookups that need
/// just one field use the borrowing accessors on [`CodeBook`] rather than materialising a whole `CodeText`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CodeText {
  /// The short name, e.g. `"Modal group violation"`. The console renders `error:N — {name}`.
  pub name: Cow<'static, str>,
  /// The full explanation, e.g. `"More than one G-code command from the same modal group ..."`.
  pub description: Cow<'static, str>,
}

impl CodeText {
  /// Build a [`CodeText`] from any two `Cow`-convertible parts. A `&'static str` borrows; a `String` is owned.
  pub fn new(name: impl Into<Cow<'static, str>>, description: impl Into<Cow<'static, str>>) -> Self {
    CodeText { name: name.into(), description: description.into() }
  }
}

/// The canonical static name+description for a grbl / grblHAL `error:N` code, borrowed from `'static`
/// constants (no allocation). Covers the codes a sender hits most; the firmware-specific entries match the
/// firmware's `$EE` text verbatim so fallback and enrichment read identically. An unknown code returns a
/// generic gloss so a bare number is never shown alone.
pub fn error_text(code: u32) -> CodeText {
  let (name, description) = error_static(code);
  CodeText { name: Cow::Borrowed(name), description: Cow::Borrowed(description) }
}

/// The static `(name, description)` pair for an `error:N` code. Kept separate from [`error_text`] so the
/// borrowing accessors can return one field without building a [`CodeText`].
fn error_static(code: u32) -> (&'static str, &'static str) {
  match code {
    1 => ("Expected command letter", "G-code words consist of a letter and a value. Letter was not found."),
    2 => ("Bad number format", "Numeric value format is not valid or missing an expected value."),
    3 => ("Invalid statement", "Grbl '$' system command was not recognized or supported."),
    5 => ("Setting disabled", "Homing cycle failure. Homing is not enabled via settings."),
    9 => ("G-code lock", "G-code locked out during alarm or jog state."),
    15 => (
      "Travel exceeded",
      "Jog target exceeds machine travel, or the line length was exceeded. Command ignored.",
    ),
    20 => ("Unsupported command", "Unsupported or invalid g-code command found in block."),
    21 => (
      "Modal group violation",
      "More than one G-code command from the same modal group was found in the block.",
    ),
    22 => ("Undefined feed rate", "Feed rate has not yet been set or is undefined."),
    // Grbl text for 23 is kept as a reasonable gloss; the firmware emits 21/26 instead, so it is never enriched.
    23 => ("Invalid g-code ID:23", "A G-code command requires an integer value but a fractional one was found."),
    26 => (
      "No axis words in block",
      "A G-code command (or the current modal state) requires axis words, but none were found in the block.",
    ),
    33 => (
      "Invalid target",
      "A G-code motion command has an invalid target (for example, arc geometry that cannot be reconciled).",
    ),
    _ => ("G-code error", "The stream is halted until reset or a '$' command clears it."),
  }
}

/// The canonical static name+description for a grbl / grblHAL `ALARM:N` code, borrowed from `'static`
/// constants. The firmware-specific entries match the firmware's `$EA` text verbatim. NOTE codes 4 and 5 were
/// historically inverted in the old banner table; the correct mapping (matching the firmware) is 4 = probe not
/// in expected initial state (already triggered), 5 = probe did not contact within travel — and both carry the
/// firmware's leading `"Probe fail. "` so a `$EA` Refresh does not change the wording. Unknown → generic lock.
pub fn alarm_text(code: u32) -> CodeText {
  let (name, description) = alarm_static(code);
  CodeText { name: Cow::Borrowed(name), description: Cow::Borrowed(description) }
}

/// The static `(name, description)` pair for an `ALARM:N` code. Separate from [`alarm_text`] so the borrowing
/// accessors can return one field without building a [`CodeText`].
fn alarm_static(code: u32) -> (&'static str, &'static str) {
  match code {
    1 => (
      "Hard limit",
      "Hard limit has been triggered. Machine position is likely lost due to sudden halt. \
Re-homing is highly recommended.",
    ),
    2 => ("Soft limit", "Soft limit alarm. G-code motion target exceeds machine travel."),
    3 => (
      "Abort during cycle",
      "Reset while in motion. Machine position is likely lost. Re-homing is highly recommended.",
    ),
    // The firmware's `$EA` text leads with "Probe fail. " for both probe alarms; match it verbatim so the
    // static fallback and the enriched description are byte-identical (no visible change after a Refresh).
    4 => ("Probe fail", "Probe fail. Probe is not in the expected initial state before starting probe cycle."),
    5 => ("Probe fail", "Probe fail. Probe did not contact the workpiece within the programmed travel."),
    // Grbl homing-fail variants the firmware does not emit, kept as reasonable grbl text.
    6 => ("Homing fail", "Homing fail. Reset during active homing cycle."),
    7 => ("Homing fail", "Homing fail. Safety door was opened during the homing cycle."),
    8 => ("Homing fail", "Homing fail. Could not find limit switch within search distance."),
    9 => ("Homing fail", "Homing fail. Second limit switch not found during pull-off."),
    10 => ("EStop asserted", "Emergency stop active."),
    11 => ("Homing required", "Homing is required. Execute homing cycle ($H) to continue."),
    _ => ("Controller locked", "Controller is locked. $X to unlock or $H to home before continuing."),
  }
}

/// Parse an `ERRORCODE:<id>|<name>|<description>` enumeration row body (already stripped of its square
/// brackets, as [`crate::protocol::Response::Message`] delivers it) into `(id, [`CodeText`])`, or `None` if the
/// body is not an `ERRORCODE:` row or lacks the id. A missing name/description collapses to empty rather than
/// rejecting the row, so a terse firmware still enriches the id.
pub fn parse_error_code_meta(message_body: &str) -> Option<(u32, CodeText)> {
  parse_code_meta(message_body, "ERRORCODE:")
}

/// Parse an `ALARMCODE:<id>|<name>|<description>` enumeration row body into `(id, [`CodeText`])`, or `None` if
/// it is not an `ALARMCODE:` row or lacks the id. Mirrors [`parse_error_code_meta`].
pub fn parse_alarm_code_meta(message_body: &str) -> Option<(u32, CodeText)> {
  parse_code_meta(message_body, "ALARMCODE:")
}

/// Shared parser for the `<TAG>:<id>|<name>|<description>` enumeration rows. Strips `tag`, takes the id (the
/// only required field), then the name and description (each defaulting to empty if absent). The description may
/// itself contain `|`, so everything after the name's separator is taken verbatim as the description. The parsed
/// strings are owned ([`Cow::Owned`]) since they outlive the borrowed input line.
fn parse_code_meta(message_body: &str, tag: &str) -> Option<(u32, CodeText)> {
  let rest = message_body.strip_prefix(tag)?;
  let mut fields = rest.splitn(3, '|');
  let id = fields.next()?.trim().parse::<u32>().ok()?;
  let name = fields.next().unwrap_or("").trim().to_string();
  // The description is the remainder verbatim (it may contain `|`), only trimmed of surrounding whitespace.
  let description = fields.next().unwrap_or("").trim().to_string();
  Some((id, CodeText { name: Cow::Owned(name), description: Cow::Owned(description) }))
}

/// A runtime codebook of error/alarm decodings enumerated from the firmware (`$EE`/`$EA`), overlaying the
/// static fallback tables. Lookups consult the override map first, then fall back to the canonical table, so
/// the accessors always return something. Per-session: [`Self::clear`] drops every override on disconnect so a
/// reconnect re-learns the (possibly different) board's set.
///
/// Two accessor shapes: [`Self::error`]/[`Self::alarm`] return a whole [`CodeText`] (used where both fields are
/// wanted), while [`Self::error_name`]/[`Self::error_description`] (and the alarm pair) return a single
/// [`Cow<'static, str>`] field — a borrow on the static path, an owned clone only on a runtime override — so a
/// per-frame caller never allocates the field it discards.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct CodeBook {
  errors: BTreeMap<u32, CodeText>,
  alarms: BTreeMap<u32, CodeText>,
}

impl CodeBook {
  /// An empty codebook: every lookup falls through to the static fallback tables until enrichment arrives.
  pub fn new() -> Self {
    Self::default()
  }

  /// Insert (or replace) the runtime override for an error code, from a parsed `[ERRORCODE:...]` row.
  pub fn apply_error(&mut self, code: u32, text: CodeText) {
    self.errors.insert(code, text);
  }

  /// Insert (or replace) the runtime override for an alarm code, from a parsed `[ALARMCODE:...]` row.
  pub fn apply_alarm(&mut self, code: u32, text: CodeText) {
    self.alarms.insert(code, text);
  }

  /// Drop every enumerated override so a fresh session re-learns from the new board's `$EE`/`$EA`. The static
  /// fallback tables are untouched — they are the floor a lookup never drops below.
  pub fn clear(&mut self) {
    self.errors.clear();
    self.alarms.clear();
  }

  /// Decode an `error:N` code into a whole [`CodeText`]: the enumerated override if present, else the canonical
  /// static text. Never `None`. Prefer [`Self::error_name`]/[`Self::error_description`] when only one field is
  /// needed, to avoid materialising the other.
  pub fn error(&self, code: u32) -> CodeText {
    self.errors.get(&code).cloned().unwrap_or_else(|| error_text(code))
  }

  /// Decode an `ALARM:N` code into a whole [`CodeText`]: the enumerated override if present, else the canonical
  /// static text. Never `None`.
  pub fn alarm(&self, code: u32) -> CodeText {
    self.alarms.get(&code).cloned().unwrap_or_else(|| alarm_text(code))
  }

  /// The `error:N` name: a borrow of the override's owned string if enumerated, else a `'static` borrow of the
  /// canonical name (no allocation). For the one-line console gloss.
  pub fn error_name(&self, code: u32) -> Cow<'static, str> {
    match self.errors.get(&code) {
      Some(text) => Cow::Owned(text.name.clone().into_owned()),
      None => Cow::Borrowed(error_static(code).0),
    }
  }

  /// The `error:N` description: the override's text if enumerated, else a `'static` borrow of the canonical
  /// description (no allocation). For the banner detail / hover.
  pub fn error_description(&self, code: u32) -> Cow<'static, str> {
    match self.errors.get(&code) {
      Some(text) => Cow::Owned(text.description.clone().into_owned()),
      None => Cow::Borrowed(error_static(code).1),
    }
  }

  /// The `ALARM:N` name: an override borrow if enumerated, else a `'static` borrow of the canonical name.
  pub fn alarm_name(&self, code: u32) -> Cow<'static, str> {
    match self.alarms.get(&code) {
      Some(text) => Cow::Owned(text.name.clone().into_owned()),
      None => Cow::Borrowed(alarm_static(code).0),
    }
  }

  /// The `ALARM:N` description: an override's text if enumerated, else a `'static` borrow of the canonical
  /// description (no allocation). For the banner detail / hover.
  pub fn alarm_description(&self, code: u32) -> Cow<'static, str> {
    match self.alarms.get(&code) {
      Some(text) => Cow::Owned(text.description.clone().into_owned()),
      None => Cow::Borrowed(alarm_static(code).1),
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn error_table_covers_the_firmware_specific_codes_with_matching_text() {
    assert_eq!(error_text(21).name, "Modal group violation");
    assert!(error_text(21).description.starts_with("More than one G-code command"));
    assert_eq!(error_text(26).name, "No axis words in block");
    assert_eq!(error_text(5).name, "Setting disabled");
    assert!(error_text(5).description.contains("Homing is not enabled"));
  }

  #[test]
  fn the_firmware_emitted_error_descriptions_match_verbatim() {
    // These eleven codes are emitted by the firmware's `$EE`; the static text MUST equal the enrichment text or
    // the banner/console wording shifts after a Refresh. Full-string equality so any drift fails the build.
    assert_eq!(error_text(1).description, "G-code words consist of a letter and a value. Letter was not found.");
    assert_eq!(error_text(2).description, "Numeric value format is not valid or missing an expected value.");
    assert_eq!(error_text(3).description, "Grbl '$' system command was not recognized or supported.");
    assert_eq!(error_text(5).description, "Homing cycle failure. Homing is not enabled via settings.");
    assert_eq!(error_text(9).description, "G-code locked out during alarm or jog state.");
    assert_eq!(
      error_text(15).description,
      "Jog target exceeds machine travel, or the line length was exceeded. Command ignored."
    );
    assert_eq!(error_text(20).description, "Unsupported or invalid g-code command found in block.");
    assert_eq!(
      error_text(21).description,
      "More than one G-code command from the same modal group was found in the block."
    );
    assert_eq!(error_text(22).description, "Feed rate has not yet been set or is undefined.");
    assert_eq!(
      error_text(26).description,
      "A G-code command (or the current modal state) requires axis words, but none were found in the block."
    );
    assert_eq!(
      error_text(33).description,
      "A G-code motion command has an invalid target (for example, arc geometry that cannot be reconciled)."
    );
  }

  #[test]
  fn unknown_error_code_still_yields_a_generic_gloss() {
    let text = error_text(9999);
    assert_eq!(text.name, "G-code error");
    assert!(text.description.contains("halted"), "an unknown error code gets a generic gloss, never a bare number");
  }

  #[test]
  fn alarm_table_covers_the_firmware_codes() {
    assert_eq!(alarm_text(1).name, "Hard limit");
    assert!(alarm_text(1).description.contains("Re-homing"));
    assert_eq!(alarm_text(11).name, "Homing required");
  }

  #[test]
  fn the_firmware_emitted_alarm_descriptions_match_verbatim() {
    // Full-string equality against the firmware's `$EA` text so static == enrichment. Codes 4/5 carry the
    // leading "Probe fail. " prefix the firmware emits, and are NOT inverted (4 = initial state, 5 = no contact).
    assert_eq!(
      alarm_text(1).description,
      "Hard limit has been triggered. Machine position is likely lost due to sudden halt. \
Re-homing is highly recommended."
    );
    assert_eq!(alarm_text(2).description, "Soft limit alarm. G-code motion target exceeds machine travel.");
    assert_eq!(
      alarm_text(3).description,
      "Reset while in motion. Machine position is likely lost. Re-homing is highly recommended."
    );
    assert_eq!(
      alarm_text(4).description,
      "Probe fail. Probe is not in the expected initial state before starting probe cycle."
    );
    assert_eq!(
      alarm_text(5).description,
      "Probe fail. Probe did not contact the workpiece within the programmed travel."
    );
    assert_eq!(alarm_text(8).description, "Homing fail. Could not find limit switch within search distance.");
    assert_eq!(alarm_text(10).description, "Emergency stop active.");
    assert_eq!(alarm_text(11).description, "Homing is required. Execute homing cycle ($H) to continue.");
  }

  #[test]
  fn alarm_four_and_five_are_not_inverted_and_match_the_firmware_verbatim() {
    // The historical banner table had these swapped AND dropped the "Probe fail. " prefix. Assert the FULL
    // firmware string for each so neither the inversion nor the prefix drift can ever return.
    assert_eq!(
      alarm_text(4).description,
      "Probe fail. Probe is not in the expected initial state before starting probe cycle.",
      "alarm 4 is the 'already triggered' initial-state failure, with the firmware's prefix"
    );
    assert_eq!(
      alarm_text(5).description,
      "Probe fail. Probe did not contact the workpiece within the programmed travel.",
      "alarm 5 is the 'no contact within travel' failure, with the firmware's prefix"
    );
  }

  #[test]
  fn unknown_alarm_code_yields_a_generic_lock_gloss() {
    assert!(alarm_text(9999).description.contains("locked"), "an unknown alarm code gets a generic lock gloss");
  }

  #[test]
  fn parses_a_real_error_code_row() {
    let (id, text) =
      parse_error_code_meta("ERRORCODE:21|Modal group violation|More than one G-code command").expect("a row");
    assert_eq!(id, 21);
    assert_eq!(text.name, "Modal group violation");
    assert_eq!(text.description, "More than one G-code command");
  }

  #[test]
  fn parses_a_real_alarm_code_row() {
    let (id, text) = parse_alarm_code_meta("ALARMCODE:1|Hard limit|Hard limit has been triggered.").expect("a row");
    assert_eq!(id, 1);
    assert_eq!(text.name, "Hard limit");
    assert_eq!(text.description, "Hard limit has been triggered.");
  }

  #[test]
  fn a_description_may_contain_pipe_characters() {
    // The description is taken verbatim after the name separator, so an embedded `|` survives.
    let (_, text) = parse_error_code_meta("ERRORCODE:15|Travel exceeded|Jog target exceeds travel | ignored")
      .expect("a row");
    assert_eq!(text.description, "Jog target exceeds travel | ignored");
  }

  #[test]
  fn parse_rejects_non_code_rows_and_idless_rows() {
    assert_eq!(parse_error_code_meta("SETTING:0|1|Step pulse time"), None, "a SETTING row is not an ERRORCODE row");
    assert_eq!(parse_alarm_code_meta("MSG:hello"), None);
    assert_eq!(parse_error_code_meta("ERRORCODE:"), None, "an ERRORCODE row with no id is rejected");
    assert_eq!(parse_error_code_meta("ERRORCODE:abc|x|y"), None, "a non-numeric id is rejected");
    // An ALARMCODE body is not an ERRORCODE row and vice versa.
    assert_eq!(parse_error_code_meta("ALARMCODE:1|Hard limit|..."), None);
    assert_eq!(parse_alarm_code_meta("ERRORCODE:21|Modal|..."), None);
  }

  #[test]
  fn an_idless_but_terse_row_still_enriches_with_empty_text() {
    // Only the id is required; a firmware that omits the description still enriches the id with an empty desc.
    let (id, text) = parse_error_code_meta("ERRORCODE:99|Mystery").expect("a row with id + name only");
    assert_eq!(id, 99);
    assert_eq!(text.name, "Mystery");
    assert_eq!(text.description, "", "a missing description collapses to empty, not a rejection");
  }

  #[test]
  fn codebook_override_beats_the_static_fallback() {
    let mut book = CodeBook::new();
    // Before enrichment, code 21 reads the static text.
    assert_eq!(book.error(21).name, "Modal group violation");
    // A firmware override for 21 with different text wins.
    book.apply_error(21, CodeText::new("Firmware-renamed".to_string(), "Custom firmware description".to_string()));
    assert_eq!(book.error(21).name, "Firmware-renamed");
    assert_eq!(book.error(21).description, "Custom firmware description");
  }

  #[test]
  fn codebook_falls_back_for_an_un_enumerated_code() {
    let mut book = CodeBook::new();
    book.apply_alarm(1, CodeText::new("Custom hard limit".to_string(), "...".to_string()));
    // Code 1 is overridden; code 2 still comes from the static table.
    assert_eq!(book.alarm(1).name, "Custom hard limit");
    assert_eq!(book.alarm(2).name, "Soft limit");
    // An entirely unknown code still yields the generic gloss, never None.
    assert!(book.error(4242).description.contains("halted"));
  }

  #[test]
  fn clear_drops_every_override_back_to_the_static_floor() {
    let mut book = CodeBook::new();
    book.apply_error(21, CodeText::new("Firmware-renamed".to_string(), "...".to_string()));
    book.apply_alarm(1, CodeText::new("Custom hard limit".to_string(), "...".to_string()));
    book.clear();
    assert_eq!(book.error(21).name, "Modal group violation", "clearing restores the static error text");
    assert_eq!(book.alarm(1).name, "Hard limit", "clearing restores the static alarm text");
  }

  #[test]
  fn borrowing_accessors_borrow_on_the_static_path_and_own_on_an_override() {
    let mut book = CodeBook::new();
    // Static path: a `'static` borrow, no allocation.
    assert!(matches!(book.error_name(21), Cow::Borrowed(_)), "the static name is borrowed, not cloned");
    assert!(matches!(book.alarm_description(1), Cow::Borrowed(_)), "the static description is borrowed");
    assert_eq!(book.error_name(21), "Modal group violation");
    assert_eq!(book.alarm_description(4), "Probe fail. Probe is not in the expected initial state before starting probe cycle.");
    // Override path: the enumerated text wins and is owned.
    book.apply_error(21, CodeText::new("Renamed".to_string(), "Long override description".to_string()));
    assert!(matches!(book.error_description(21), Cow::Owned(_)), "an override description is owned");
    assert_eq!(book.error_name(21), "Renamed");
    assert_eq!(book.error_description(21), "Long override description");
  }
}
