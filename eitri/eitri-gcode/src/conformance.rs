//! A validator that checks rendered G-code against the grblHAL / Skirnir contract
//! (`docs/eitri-gcode-skirnir-contract.md`).
//!
//! This is the load-bearing guarantee of Phase 4: if a program passes [`check_grbl_conformance`], the firmware will
//! not halt on it with an `error:N`. Every rule in the contract that can be checked from the text alone is enforced
//! here — the supported G/M allowlist, the 256-byte line limit, LF-only endings, `F` present on `G1`/`G2`/`G3` and
//! absent on `G0`, arcs as IJK (never `R`, never missing an offset), no `N` line numbers, at-least-three-decimal
//! coordinates, and a program that ends with `M2`/`M30`. It is public so tests — and, later, tooling — can assert an
//! emitter's output is safe to stream.

use std::collections::HashSet;

/// A single contract violation found in a program.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Violation {
  /// 1-based line number, or 0 for a whole-program rule (line ending, missing program end).
  pub line: usize,
  /// What was wrong.
  pub reason: Reason,
}

/// The specific contract rule a [`Violation`] broke.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Reason {
  /// A carriage return appeared; the contract mandates LF-only output.
  CarriageReturn,
  /// A line exceeded the 256-byte limit (grbl `error:15`).
  LineTooLong(usize),
  /// A G/M code outside the supported allowlist (grbl `error:20`). Carries the offending word.
  UnsupportedCode(String),
  /// An `N` line-number word, which the contract forbids on our output.
  LineNumber,
  /// A `G1`/`G2`/`G3` cut move with no `F` feed word.
  MissingFeed,
  /// A `G0` rapid carrying an `F` word.
  FeedOnRapid,
  /// A `G2`/`G3` arc with no I/J/K centre offset (grbl `error:33`).
  ArcMissingOffset,
  /// A `G2`/`G3` arc using the unsupported `R` radius form.
  ArcRadiusForm,
  /// A coordinate word (X/Y/Z/I/J/K) with fewer than three decimal places. Carries the offending word.
  CoordPrecision(String),
  /// The program did not end with `M2` or `M30`.
  NoProgramEnd,
}

/// Check `text` against the grblHAL/Skirnir contract, returning every violation found (empty = conformant).
pub fn check_grbl_conformance(text: &str) -> Vec<Violation> {
  let mut violations = Vec::new();
  let allow = SupportedCodes::new();

  if text.contains('\r') {
    violations.push(Violation { line: 0, reason: Reason::CarriageReturn });
  }

  let mut last_content: Option<Vec<Word>> = None;
  for (idx, raw) in text.lines().enumerate() {
    let line_no = idx + 1;
    if raw.len() > 256 {
      violations.push(Violation { line: line_no, reason: Reason::LineTooLong(raw.len()) });
    }
    let code = strip_comment(raw);
    let words = tokenize(&code);
    if words.is_empty() {
      continue;
    }
    check_line(line_no, &words, &allow, &mut violations);
    last_content = Some(words);
  }

  // The program must end with M2 or M30 on its final content line.
  let ends_ok = last_content
    .as_ref()
    .map(|words| words.iter().any(|w| w.letter == 'M' && (w.value == "2" || w.value == "30")))
    .unwrap_or(false);
  if !ends_ok {
    violations.push(Violation { line: 0, reason: Reason::NoProgramEnd });
  }

  violations
}

/// Whether `text` fully conforms to the grblHAL/Skirnir contract.
pub fn is_grbl_conformant(text: &str) -> bool {
  check_grbl_conformance(text).is_empty()
}

/// One tokenized G-code word: an uppercase address letter plus its value string (which may be empty).
#[derive(Debug, Clone, PartialEq, Eq)]
struct Word {
  letter: char,
  value: String,
}

/// Apply every per-line rule to one already-tokenized, comment-stripped line.
fn check_line(line_no: usize, words: &[Word], allow: &SupportedCodes, out: &mut Vec<Violation>) {
  let has = |letter: char| words.iter().any(|w| w.letter == letter);
  let g_codes: Vec<&str> = words.iter().filter(|w| w.letter == 'G').map(|w| w.value.as_str()).collect();
  let is_motion = |code: &str| g_codes.iter().any(|g| canonical_code(g) == canonical_code(code));

  for w in words {
    match w.letter {
      'N' => out.push(Violation { line: line_no, reason: Reason::LineNumber }),
      'G' => {
        if !allow.g.contains(&canonical_code(&w.value)) {
          out.push(Violation { line: line_no, reason: Reason::UnsupportedCode(format!("G{}", w.value)) });
        }
      }
      'M' => {
        if !allow.m.contains(&canonical_code(&w.value)) {
          out.push(Violation { line: line_no, reason: Reason::UnsupportedCode(format!("M{}", w.value)) });
        }
      }
      'X' | 'Y' | 'Z' | 'I' | 'J' | 'K' if !has_min_decimals(&w.value, 3) => {
        out.push(Violation { line: line_no, reason: Reason::CoordPrecision(format!("{}{}", w.letter, w.value)) });
      }
      _ => {}
    }
  }

  let is_rapid = is_motion("0");
  let is_linear = is_motion("1");
  let is_arc = is_motion("2") || is_motion("3");

  if (is_linear || is_arc) && !has('F') {
    out.push(Violation { line: line_no, reason: Reason::MissingFeed });
  }
  if is_rapid && has('F') {
    out.push(Violation { line: line_no, reason: Reason::FeedOnRapid });
  }
  if is_arc {
    if !(has('I') || has('J') || has('K')) {
      out.push(Violation { line: line_no, reason: Reason::ArcMissingOffset });
    }
    if has('R') {
      out.push(Violation { line: line_no, reason: Reason::ArcRadiusForm });
    }
  }
}

/// The supported G/M code sets from the contract, stored as canonical numeric strings.
struct SupportedCodes {
  g: HashSet<String>,
  m: HashSet<String>,
}

impl SupportedCodes {
  fn new() -> SupportedCodes {
    let g = [
      "0", "1", "2", "3", "4", "28", "28.1", "30", "30.1", "38.2", "38.3", "38.4", "38.5", "10", "43.1", "49", "53",
      "54", "55", "56", "57", "58", "59", "92", "92.1", "17", "18", "19", "20", "21", "90", "91", "93", "94",
    ];
    let m = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "30"];
    SupportedCodes {
      g: g.iter().map(|c| canonical_code(c)).collect(),
      m: m.iter().map(|c| canonical_code(c)).collect(),
    }
  }
}

/// Canonicalize a numeric code string so `G01`/`G1` and `38.20`/`38.2` compare equal: parse and reformat, dropping
/// leading/trailing zero padding. An unparseable code returns its trimmed self so it simply fails to match.
fn canonical_code(code: &str) -> String {
  let trimmed = code.trim();
  match trimmed.parse::<f64>() {
    Ok(v) if v.fract() == 0.0 => format!("{}", v as i64),
    Ok(v) => format!("{v}"),
    Err(_) => trimmed.to_string(),
  }
}

/// Whether a coordinate value string has at least `min` decimal places (digits after the decimal point).
fn has_min_decimals(value: &str, min: usize) -> bool {
  match value.split_once('.') {
    Some((_, frac)) => frac.chars().filter(|c| c.is_ascii_digit()).count() >= min,
    None => false,
  }
}

/// Remove grbl comments from a line: everything inside `(...)` and everything from a `;` to end of line.
fn strip_comment(line: &str) -> String {
  let mut out = String::with_capacity(line.len());
  let mut depth = 0u32;
  for c in line.chars() {
    match c {
      ';' if depth == 0 => break,
      '(' => depth += 1,
      ')' if depth > 0 => depth -= 1,
      _ if depth == 0 => out.push(c),
      _ => {}
    }
  }
  out
}

/// Split a comment-stripped line into words: each is an alphabetic address letter (uppercased) followed by its value
/// up to the next letter or whitespace.
fn tokenize(code: &str) -> Vec<Word> {
  let mut words = Vec::new();
  let chars: Vec<char> = code.chars().collect();
  let mut i = 0;
  while i < chars.len() {
    let c = chars[i];
    if c.is_ascii_alphabetic() {
      let letter = c.to_ascii_uppercase();
      let mut value = String::new();
      i += 1;
      while i < chars.len() && !chars[i].is_ascii_alphabetic() && !chars[i].is_whitespace() {
        value.push(chars[i]);
        i += 1;
      }
      words.push(Word { letter, value });
    } else {
      i += 1;
    }
  }
  words
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn a_minimal_conformant_program_passes() {
    let text = "(header)\nG90 G21 G54 G17 G94\nM3 S10000\nG0 X1.0000 Y2.0000\nG1 Z-0.1000 F60.0\nM5\nM2\n";
    assert_eq!(check_grbl_conformance(text), Vec::new());
  }

  #[test]
  fn unsupported_code_is_flagged() {
    let text = "G81 X1.0000 Y1.0000 Z-1.0000 R2.0000 F100.0\nM2\n";
    let v = check_grbl_conformance(text);
    assert!(v.iter().any(|x| x.reason == Reason::UnsupportedCode("G81".to_string())), "{v:?}");
  }

  #[test]
  fn missing_feed_on_cut_move_is_flagged() {
    let text = "G1 X1.0000 Y1.0000\nM2\n";
    let v = check_grbl_conformance(text);
    assert!(v.iter().any(|x| x.reason == Reason::MissingFeed), "{v:?}");
  }

  #[test]
  fn feed_on_rapid_is_flagged() {
    let text = "G0 X1.0000 Y1.0000 F100.0\nM2\n";
    let v = check_grbl_conformance(text);
    assert!(v.iter().any(|x| x.reason == Reason::FeedOnRapid), "{v:?}");
  }

  #[test]
  fn arc_without_offset_and_radius_form_are_both_flagged() {
    let no_off = check_grbl_conformance("G2 X1.0000 Y1.0000 F100.0\nM2\n");
    assert!(no_off.iter().any(|x| x.reason == Reason::ArcMissingOffset), "{no_off:?}");
    let r_form = check_grbl_conformance("G2 X1.0000 Y1.0000 R5.0000 F100.0\nM2\n");
    assert!(r_form.iter().any(|x| x.reason == Reason::ArcRadiusForm), "{r_form:?}");
  }

  #[test]
  fn arc_with_ij_offset_passes() {
    let text = "G3 X0.0000 Y1.0000 I-1.0000 J0.0000 F120.0\nM2\n";
    let v = check_grbl_conformance(text);
    assert!(!v.iter().any(|x| matches!(x.reason, Reason::ArcMissingOffset | Reason::ArcRadiusForm)), "{v:?}");
  }

  #[test]
  fn line_numbers_are_flagged() {
    let text = "N10 G0 X1.0000\nM2\n";
    let v = check_grbl_conformance(text);
    assert!(v.iter().any(|x| x.reason == Reason::LineNumber), "{v:?}");
  }

  #[test]
  fn low_precision_coordinate_is_flagged() {
    let text = "G0 X1.5 Y2.00\nM2\n";
    let v = check_grbl_conformance(text);
    assert!(v.iter().any(|x| matches!(&x.reason, Reason::CoordPrecision(_))), "{v:?}");
  }

  #[test]
  fn carriage_return_and_missing_end_are_flagged() {
    let v = check_grbl_conformance("G90 G21\r\nM3 S1000\n");
    assert!(v.iter().any(|x| x.reason == Reason::CarriageReturn), "{v:?}");
    assert!(v.iter().any(|x| x.reason == Reason::NoProgramEnd), "{v:?}");
  }

  #[test]
  fn overlong_line_is_flagged() {
    let long = format!("(<{}>)\nM2\n", "x".repeat(260));
    let v = check_grbl_conformance(&long);
    assert!(v.iter().any(|x| matches!(x.reason, Reason::LineTooLong(_))), "{v:?}");
  }

  #[test]
  fn program_ending_in_m30_passes_the_end_rule() {
    let v = check_grbl_conformance("G90 G21\nM30\n");
    assert!(!v.iter().any(|x| x.reason == Reason::NoProgramEnd), "{v:?}");
  }
}
