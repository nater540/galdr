//! Output formatting knobs shared by every postprocessor: coordinate/feed precision, unit mode, comment style, and
//! line ending. Centralizing the number rendering here (rather than scattered `format!` calls) is the §12
//! "numerical precision & robustness" requirement — one place decides how a coordinate becomes text.

use eitri_core::{GCODE_DECIMALS, Unit};

/// How comments are rendered, so a dialect that dislikes one style can pick the other (or drop comments entirely).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommentStyle {
  /// Parenthesised inline comments: `(text)`. grbl strips these.
  Paren,
  /// Semicolon-to-end-of-line comments: `; text`. grbl strips these too.
  Semicolon,
  /// Emit no comments at all (a bare, minimal program).
  None,
}

/// The line terminator. The grbl/Skirnir contract mandates LF-only output; other controllers accept CRLF.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LineEnding {
  /// `\n` — required by the Skirnir/firmware contract.
  Lf,
  /// `\r\n` — accepted by many desktop controllers.
  Crlf,
}

impl LineEnding {
  /// The terminator bytes for this line ending.
  pub fn as_str(self) -> &'static str {
    match self {
      LineEnding::Lf => "\n",
      LineEnding::Crlf => "\r\n",
    }
  }
}

/// Formatting configuration owned by a postprocessor. Defaults target the grbl/Skirnir contract: four-decimal
/// coordinates (>= the contract's three-place minimum, and firmware normalizes the rest), millimetres, parenthesised
/// comments, LF terminator.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OutputFormat {
  /// Decimal places on X/Y/Z/I/J/K coordinates. Must be >= 3 to satisfy the contract.
  pub coord_decimals: usize,
  /// Decimal places on feed rates.
  pub feed_decimals: usize,
  /// Unit mode, which selects the `G20`/`G21` header word.
  pub units: Unit,
  /// Comment rendering style.
  pub comment_style: CommentStyle,
  /// Line terminator.
  pub line_ending: LineEnding,
}

impl Default for OutputFormat {
  fn default() -> OutputFormat {
    OutputFormat {
      coord_decimals: GCODE_DECIMALS,
      feed_decimals: 1,
      units: Unit::Millimeters,
      comment_style: CommentStyle::Paren,
      line_ending: LineEnding::Lf,
    }
  }
}

impl OutputFormat {
  /// The `G20`/`G21` word for the configured units.
  pub fn units_word(&self) -> &'static str {
    match self.units {
      Unit::Millimeters => "G21",
      Unit::Inches => "G20",
    }
  }

  /// Render a coordinate to the configured precision, collapsing a rounded `-0.0` to `0` so output never carries a
  /// meaningless minus sign.
  pub fn coord(&self, value: f64) -> String {
    fmt_fixed(value, self.coord_decimals)
  }

  /// Render a feed rate to the configured precision.
  pub fn feed(&self, value: f64) -> String {
    fmt_fixed(value, self.feed_decimals)
  }

  /// Wrap `text` in the configured comment style, or return `None` when comments are disabled. The text is sanitized
  /// so it can never terminate a parenthesised comment early or inject a line break.
  pub fn comment(&self, text: &str) -> Option<String> {
    match self.comment_style {
      CommentStyle::None => None,
      CommentStyle::Paren => Some(format!("({})", sanitize_comment(text, true))),
      CommentStyle::Semicolon => Some(format!("; {}", sanitize_comment(text, false))),
    }
  }
}

/// Format `value` with `decimals` fixed places, mapping a value that rounds to zero onto a clean `0.000…` (never
/// `-0.000…`).
fn fmt_fixed(value: f64, decimals: usize) -> String {
  let factor = 10f64.powi(decimals as i32);
  let rounded = (value * factor).round() / factor;
  let cleaned = if rounded == 0.0 { 0.0 } else { rounded };
  format!("{cleaned:.decimals$}")
}

/// Remove characters that would break out of a comment: parentheses (which close a grbl comment) and any newline or
/// carriage return (which would split the line). In paren style parentheses are stripped; both styles strip newlines.
fn sanitize_comment(text: &str, strip_parens: bool) -> String {
  text
    .chars()
    .filter(|&c| c != '\n' && c != '\r' && !(strip_parens && (c == '(' || c == ')')))
    .collect()
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn coord_uses_four_places_by_default_and_meets_the_minimum() {
    let f = OutputFormat::default();
    assert!(f.coord_decimals >= 3, "contract requires >= 3 decimal places");
    assert_eq!(f.coord(1.23456), "1.2346");
    assert_eq!(f.coord(10.0), "10.0000");
  }

  #[test]
  fn negative_zero_is_cleaned() {
    let f = OutputFormat::default();
    // A tiny negative that rounds to zero must not render as "-0.0000".
    assert_eq!(f.coord(-0.00001), "0.0000");
    assert_eq!(f.coord(-0.0), "0.0000");
  }

  #[test]
  fn units_word_tracks_the_unit_mode() {
    assert_eq!(OutputFormat { units: Unit::Millimeters, ..Default::default() }.units_word(), "G21");
    assert_eq!(OutputFormat { units: Unit::Inches, ..Default::default() }.units_word(), "G20");
  }

  #[test]
  fn paren_comment_strips_nested_parens_and_newlines() {
    let f = OutputFormat { comment_style: CommentStyle::Paren, ..Default::default() };
    assert_eq!(f.comment("tool (1) change\nnext").as_deref(), Some("(tool 1 changenext)"));
  }

  #[test]
  fn semicolon_comment_keeps_parens_but_drops_newlines() {
    let f = OutputFormat { comment_style: CommentStyle::Semicolon, ..Default::default() };
    assert_eq!(f.comment("depth (mm)\n").as_deref(), Some("; depth (mm)"));
  }

  #[test]
  fn disabled_comments_return_none() {
    let f = OutputFormat { comment_style: CommentStyle::None, ..Default::default() };
    assert_eq!(f.comment("anything"), None);
  }
}
