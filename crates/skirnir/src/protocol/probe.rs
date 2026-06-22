//! Parsing of a `[PRB:...]` probe-result body into typed values.
//!
//! grblHAL reports the outcome of a `G38.x` probe as a bracketed push message `[PRB:<x>,<y>,<z>{,<a>…}:<flag>]`,
//! where the comma-separated axis values are the MACHINE-coordinate position at the trigger instant and the
//! trailing `:0`/`:1` flag is the success bit (`1` = the probe contacted within travel, `0` = no contact). The
//! same `[PRB:…]` line is also emitted in answer to a `$#` parameter query, carrying the LAST probe's result, so
//! one body grammar serves both the asynchronous push and the on-demand poll.
//!
//! The response layer ([`crate::protocol::response::parse_line`]) strips the square brackets AND the `PRB:` tag,
//! handing us just the value body (e.g. `-1.015,0.000,-2.500,90.000:1` for `[PRB:-1.015,0.000,-2.500,90.000:1]`).
//! This module is pure and synchronous — no async, no UI — so the grammar is unit-tested in isolation. Per
//! grblHAL's parsing rules we accept **1..N** axis values (a 3-field legacy controller and Galdr's 4-field build
//! must both parse), never a hardcoded axis count.

/// Parse a `[PRB:…]` body (already stripped of the leading `[` / trailing `]` and the `PRB:` tag) into its
/// `(position, success)` pair. The body is `<v0>,<v1>{,…}:<flag>`: one-or-more comma-separated f64 axis values
/// in report order (X, Y, Z, then any rotary axes), then a single `:0`/`:1` success flag.
///
/// Returns `None` on any malformation — a missing `:flag`, a non-numeric axis value, an empty value list, or a
/// flag that is neither `0` nor `1` — so the caller can fall through to the generic bracketed-message path
/// rather than fabricating a probe result. Never panics.
pub fn parse_prb_body(body: &str) -> Option<(Vec<f64>, bool)> {
  // The flag is the tail after the LAST colon; `rsplit_once` so a value can never contain a colon and confuse
  // the split. Everything before it is the comma-separated axis list.
  let (values_part, flag_part) = body.rsplit_once(':')?;
  let success = match flag_part.trim() {
    "1" => true,
    "0" => false,
    // Any other flag (empty, `2`, garbage) is not a well-formed PRB result; fall through rather than guess.
    _ => return None,
  };
  // Parse 1..N axis values. A single unparseable element invalidates the whole body — unlike the status DRO
  // (which drops a bad axis to keep the rest), a probe result is load-bearing: a missing/garbled coordinate must
  // not be silently treated as a complete reading the wizard would then act on. We additionally REJECT
  // non-finite values: Rust's `f64::parse` accepts `"nan"`/`"inf"`, and a non-finite coordinate would poison the
  // wizard math (a NaN `Y_c`/`Z_c` → NaN `G10`), so a non-finite axis is treated exactly like a non-numeric one.
  let mut position = Vec::new();
  for raw in values_part.split(',') {
    let value = raw.trim().parse::<f64>().ok()?;
    if !value.is_finite() {
      return None;
    }
    position.push(value);
  }
  if position.is_empty() {
    return None;
  }
  Some((position, success))
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn parses_a_four_field_successful_probe() {
    // Galdr's 4-field build: X, Y, Z, A, then the success flag.
    let (position, success) = parse_prb_body("PRB:-1.015,0.000,-2.500,90.000:1".strip_prefix("PRB:").unwrap_or(""))
      .expect("a well-formed 4-field PRB body parses");
    assert_eq!(position, vec![-1.015, 0.0, -2.5, 90.0]);
    assert!(success);
  }

  #[test]
  fn parses_a_three_field_failed_probe() {
    // A 3-field legacy controller (no rotary) reporting a no-contact probe must still parse, with success false.
    let (position, success) = parse_prb_body("0.000,0.000,0.000:0").expect("a 3-field PRB body parses");
    assert_eq!(position, vec![0.0, 0.0, 0.0]);
    assert!(!success);
  }

  #[test]
  fn parses_a_single_axis_value() {
    // The grammar accepts 1..N values; a lone axis must not be rejected for being short.
    let (position, success) = parse_prb_body("5.250:1").expect("a single-value PRB body parses");
    assert_eq!(position, vec![5.25]);
    assert!(success);
  }

  #[test]
  fn rejects_a_body_with_no_flag() {
    // No trailing `:flag` — not a probe result; the caller falls through to the generic message path.
    assert_eq!(parse_prb_body("1.0,2.0,3.0"), None);
  }

  #[test]
  fn rejects_a_non_binary_flag() {
    // A flag that is neither 0 nor 1 is malformed; we do not guess truthiness.
    assert_eq!(parse_prb_body("1.0,2.0,3.0:2"), None);
    assert_eq!(parse_prb_body("1.0,2.0,3.0:"), None);
  }

  #[test]
  fn rejects_a_non_numeric_axis_value() {
    // A garbled coordinate invalidates the whole reading — a probe result must not be acted on half-parsed.
    assert_eq!(parse_prb_body("1.0,bad,3.0:1"), None);
  }

  #[test]
  fn rejects_an_empty_value_list() {
    // `:1` alone has no axis values; there is nothing to report.
    assert_eq!(parse_prb_body(":1"), None);
  }

  #[test]
  fn rejects_non_finite_axis_values() {
    // Rust's f64::parse accepts "nan"/"inf"/"-inf"; a non-finite coordinate would poison the wizard math (NaN
    // Y_c/Z_c → NaN G10), so it must be rejected like a non-numeric value and fall through to a Message.
    assert_eq!(parse_prb_body("nan,0.0,0.0:1"), None);
    assert_eq!(parse_prb_body("0.0,inf,0.0:1"), None);
    assert_eq!(parse_prb_body("0.0,0.0,-inf:1"), None);
    assert_eq!(parse_prb_body("NaN,0,0:0"), None);
  }
}
