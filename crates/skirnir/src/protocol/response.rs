//! Parsing of a single firmware output line into a typed [`Response`].
//!
//! The firmware emits exactly three classes of output (per `docs/gcode-streaming.md`): response messages
//! (`ok` / `error:N`, the only flow-control drivers), push messages (`<...>` status, `[...]` bracketed,
//! the welcome banner), and real-time responses. This module turns one already-de-terminated line of text
//! into the typed value the engine reacts to. Per grblHAL's sender guidance we ignore unknown tags rather
//! than failing, so forward-compatible firmware extensions never break the parser.

/// A single parsed line of firmware output. Only [`Response::Ok`] and [`Response::Error`] move the
/// character-count window; everything else is informational push output the UI may display.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Response {
  /// `ok` — a line was accepted. Frees the oldest in-flight line from the send-ahead window.
  Ok,

  /// `error:N` — a line was rejected. Frees the window slot but also trips the error-hold reaction.
  Error(u32),

  /// `ALARM:N` — the controller entered an alarm state and will refuse G-code until cleared.
  Alarm(u32),

  /// `<...>` real-time status report. Carried verbatim (sans angle brackets) for now; field parsing is a
  /// follow-up. The engine uses its arrival to keep the DRO/lifecycle fresh.
  Status(String),

  /// `[...]` bracketed push message (banner info, `[MSG:]`, `[PRB:]`, `[OPT:]`, `[G54:]`, ...). Carried
  /// verbatim (sans square brackets). Recognised sub-kinds (e.g. `[OPT:]`) are interpreted by the engine.
  Message(String),

  /// The welcome banner (`Grbl 1.1f ...` / `GrblHAL 1.1f ...`), emitted on boot and after a soft reset.
  /// Seeing it mid-stream means the controller reset and the stream must stop.
  Banner(String),

  /// A startup-line execution echo, e.g. `>G54G20:ok`. Informational.
  StartupEcho(String),

  /// A non-empty line we did not recognise. Surfaced verbatim rather than discarded, so nothing is lost.
  Unknown(String),
}

/// Parse one already-trimmed firmware line (no trailing terminator) into a [`Response`]. An empty line
/// yields `None` — empty lines carry no response and must not be mistaken for an `ok`.
pub fn parse_line(line: &str) -> Option<Response> {
  let line = line.trim_end_matches([' ', '\t']);
  if line.is_empty() {
    return None;
  }

  if line == "ok" {
    return Some(Response::Ok);
  }

  if let Some(rest) = line.strip_prefix("error:") {
    // A malformed numeric tail is still an error class; fall back to code 0 rather than dropping it.
    let code = rest.trim().parse::<u32>().unwrap_or(0);
    return Some(Response::Error(code));
  }

  if let Some(rest) = line.strip_prefix("ALARM:") {
    let code = rest.trim().parse::<u32>().unwrap_or(0);
    return Some(Response::Alarm(code));
  }

  if let Some(inner) = line.strip_prefix('<').and_then(|s| s.strip_suffix('>')) {
    return Some(Response::Status(inner.to_string()));
  }

  if let Some(inner) = line.strip_prefix('[').and_then(|s| s.strip_suffix(']')) {
    return Some(Response::Message(inner.to_string()));
  }

  if let Some(rest) = line.strip_prefix('>') {
    return Some(Response::StartupEcho(rest.to_string()));
  }

  // The banner is the only bare-text line we special-case; both legacy `Grbl` and `GrblHAL` forms start
  // with `Grbl` (grblHAL drops to `Grbl` at compatibility level >= 1), so a case-insensitive prefix match
  // on `grbl` followed by a version-ish token is a safe, specific signal.
  let lower = line.to_ascii_lowercase();
  if lower.starts_with("grbl ") || lower.starts_with("grblhal ") {
    return Some(Response::Banner(line.to_string()));
  }

  Some(Response::Unknown(line.to_string()))
}

/// Extract the RX-buffer size advertised in an `[OPT:...]` message body. The OPT field order is
/// `options,block_buffer,rx_buffer{,axes{,tools}}`, so the RX buffer is the third comma-separated field.
/// Returns `None` if the body is not an OPT line or the field is missing / unparseable, leaving the engine
/// on its sensible default.
pub fn rx_buffer_from_opt(message_body: &str) -> Option<usize> {
  let rest = message_body.strip_prefix("OPT:")?;
  rest.split(',').nth(2)?.trim().parse::<usize>().ok()
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn parses_ok() {
    assert_eq!(parse_line("ok"), Some(Response::Ok));
  }

  #[test]
  fn ok_with_trailing_whitespace_is_still_ok() {
    assert_eq!(parse_line("ok  "), Some(Response::Ok));
  }

  #[test]
  fn parses_error_code() {
    assert_eq!(parse_line("error:9"), Some(Response::Error(9)));
  }

  #[test]
  fn parses_alarm_code() {
    assert_eq!(parse_line("ALARM:1"), Some(Response::Alarm(1)));
  }

  #[test]
  fn empty_line_is_not_a_response() {
    assert_eq!(parse_line(""), None);
    assert_eq!(parse_line("   "), None);
  }

  #[test]
  fn parses_status_report_without_brackets() {
    assert_eq!(
      parse_line("<Idle|MPos:0.000,0.000,0.000|FS:0,0>"),
      Some(Response::Status("Idle|MPos:0.000,0.000,0.000|FS:0,0".to_string()))
    );
  }

  #[test]
  fn parses_bracketed_message() {
    assert_eq!(
      parse_line("[MSG:'$H'|'$X' to unlock]"),
      Some(Response::Message("MSG:'$H'|'$X' to unlock".to_string()))
    );
  }

  #[test]
  fn recognises_both_banner_spellings() {
    assert!(matches!(parse_line("Grbl 1.1f ['$' for help]"), Some(Response::Banner(_))));
    assert!(matches!(parse_line("GrblHAL 1.1f ['$' or '$HELP' for help]"), Some(Response::Banner(_))));
  }

  #[test]
  fn parses_startup_echo() {
    assert_eq!(parse_line(">G54G20:ok"), Some(Response::StartupEcho("G54G20:ok".to_string())));
  }

  #[test]
  fn unknown_line_is_preserved_verbatim() {
    assert_eq!(parse_line("totally novel"), Some(Response::Unknown("totally novel".to_string())));
  }

  #[test]
  fn reads_rx_buffer_from_real_opt_line() {
    assert_eq!(rx_buffer_from_opt("OPT:VNMSL,100,1024,3,0"), Some(1024));
    assert_eq!(rx_buffer_from_opt("OPT:VNMSL,35,1024,5,0"), Some(1024));
  }

  #[test]
  fn rx_buffer_ignores_non_opt_messages() {
    assert_eq!(rx_buffer_from_opt("VER:1.1f.20250225:"), None);
    assert_eq!(rx_buffer_from_opt("OPT:VNMSL"), None);
  }
}
