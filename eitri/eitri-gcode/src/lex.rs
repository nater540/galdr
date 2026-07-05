//! The low-level G-code read-back lexer.
//!
//! Per `docs/eitri-porting-plan.md` §8 the G-code tokenizer lives *here* — alongside the emitter that produces
//! G-code — so the read and write sides share one word model, and `eitri-import` (§6) consumes it rather than
//! carrying a second parser. The emitter renders motions to text; this is the inverse, turning text back into the
//! address-letter/number **words** a controller consumes. It is deliberately dialect-agnostic and pure: it knows
//! nothing about which `G`/`M` codes mean what — that modal interpretation is the caller's job (see
//! `eitri-import`'s G-code preview walk).
//!
//! What it handles: whitespace-insensitivity (`G1X10Y20` lexes identically to `G1 X10 Y20`), case folding on the
//! address letter, parenthesised `(...)` and semicolon `;` comments, signed/decimal numbers (`-1.1798`, `.5`,
//! `5.`), line numbers (`N` is just another word), the leading block-delete `/`, and the `%` program-boundary
//! marker. A malformed number or an out-of-grammar character is a typed [`Error::Parse`], never a panic.

use eitri_core::{Error, Result};

/// A single G-code word: an address letter (folded to uppercase) and its numeric value. `G1` lexes to
/// `{ letter: 'G', value: 1.0 }`; `X-1.1798` to `{ letter: 'X', value: -1.1798 }`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Word {
  /// The address letter, uppercased (`G`, `X`, `Y`, `Z`, `I`, `J`, `F`, `N`, `M`, `T`, ...).
  pub letter: char,
  /// The numeric value following the letter.
  pub value: f64,
}

/// One lexed G-code block (a physical line): its words in order, any comments found on the line, and whether the
/// line carried the block-delete `/` prefix. A comment-only or block-delete-only line still lexes to a `Line`;
/// wholly blank lines are dropped by [`lex`].
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Line {
  /// The address words in source order.
  pub words: Vec<Word>,
  /// Comment text (parenthesised or semicolon), trimmed, in source order.
  pub comments: Vec<String>,
  /// Whether the block-delete `/` prefix was present (the controller may skip this block).
  pub block_delete: bool,
}

impl Line {
  /// The value of the first word addressed by `letter` (case-insensitive), or `None` if the line carries no such
  /// word. Convenience for modal walkers that ask "does this block set X?".
  pub fn value(&self, letter: char) -> Option<f64> {
    let letter = letter.to_ascii_uppercase();
    self.words.iter().find(|w| w.letter == letter).map(|w| w.value)
  }

  /// Whether the line carries any word addressed by `letter` (case-insensitive).
  pub fn has(&self, letter: char) -> bool {
    let letter = letter.to_ascii_uppercase();
    self.words.iter().any(|w| w.letter == letter)
  }
}

/// Lex a whole G-code program into one [`Line`] per non-blank physical line. Blank lines (no words, no comments,
/// no block-delete) are dropped so downstream walkers see only meaningful blocks. Both `\n` and `\r\n` separate
/// lines. A lex failure on any line short-circuits with its [`Error::Parse`].
pub fn lex(src: &str) -> Result<Vec<Line>> {
  let mut out = Vec::new();
  for raw in src.split('\n') {
    let line = lex_line(raw)?;
    if !line.words.is_empty() || !line.comments.is_empty() || line.block_delete {
      out.push(line);
    }
  }
  Ok(out)
}

/// Lex a single physical line (no embedded newline) into a [`Line`]. Exposed for callers that already own line
/// splitting; [`lex`] is the usual entry point.
pub fn lex_line(src: &str) -> Result<Line> {
  let chars: Vec<char> = src.chars().collect();
  let mut line = Line::default();
  let mut i = 0;

  // A leading block-delete `/` (after optional whitespace) flags the whole block. `/` is not a valid address, so
  // it is only accepted here at the block head; anywhere else it is an out-of-grammar character.
  while i < chars.len() && chars[i].is_whitespace() {
    i += 1;
  }
  if i < chars.len() && chars[i] == '/' {
    line.block_delete = true;
    i += 1;
  }

  while i < chars.len() {
    let c = chars[i];
    if c.is_whitespace() {
      i += 1;
    } else if c == '%' {
      // Program-start/end marker: carries no geometry, skip it.
      i += 1;
    } else if c == '(' {
      i = lex_paren_comment(&chars, i, &mut line);
    } else if c == ';' {
      // Semicolon runs to end of line.
      let text: String = chars[i + 1..].iter().collect();
      line.comments.push(text.trim().to_string());
      break;
    } else if c.is_ascii_alphabetic() {
      i = lex_word(&chars, i, &mut line)?;
    } else {
      return Err(Error::Parse(format!("unexpected character '{c}' in G-code line: {src:?}")));
    }
  }
  Ok(line)
}

/// Consume a parenthesised comment starting at `open` (`chars[open] == '('`), pushing its trimmed body onto
/// `line`. Parenthesised comments do not nest; an unterminated `(` leniently takes the rest of the line, matching
/// how forgiving controllers treat it. Returns the index just past the comment.
fn lex_paren_comment(chars: &[char], open: usize, line: &mut Line) -> usize {
  let mut j = open + 1;
  let mut body = String::new();
  while j < chars.len() && chars[j] != ')' {
    body.push(chars[j]);
    j += 1;
  }
  line.comments.push(body.trim().to_string());
  // Skip the closing ')' if present; if the comment was unterminated `j` already sits at end-of-line.
  if j < chars.len() {
    j += 1;
  }
  j
}

/// Consume one address word starting at `start` (`chars[start]` is the letter), parse its number, and push the
/// [`Word`]. Returns the index just past the number, or an [`Error::Parse`] on a missing/malformed value.
fn lex_word(chars: &[char], start: usize, line: &mut Line) -> Result<usize> {
  let letter = chars[start].to_ascii_uppercase();
  let mut j = start + 1;
  // Tolerate whitespace between the address letter and its value (`X 10`), which some emitters produce.
  while j < chars.len() && chars[j].is_whitespace() {
    j += 1;
  }
  let num_start = j;
  if j < chars.len() && (chars[j] == '+' || chars[j] == '-') {
    j += 1;
  }
  while j < chars.len() && (chars[j].is_ascii_digit() || chars[j] == '.') {
    j += 1;
  }
  let num: String = chars[num_start..j].iter().collect();
  let value = num
    .parse::<f64>()
    .map_err(|_| Error::Parse(format!("malformed number after '{letter}': {num:?}")))?;
  line.words.push(Word { letter, value });
  Ok(j)
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn lexes_a_motion_line_into_ordered_words() {
    let line = lex_line("G1 X10.2147 Y-1.1798 F120.0").expect("lex");
    assert_eq!(
      line.words,
      vec![
        Word { letter: 'G', value: 1.0 },
        Word { letter: 'X', value: 10.2147 },
        Word { letter: 'Y', value: -1.1798 },
        Word { letter: 'F', value: 120.0 },
      ]
    );
    assert!(line.comments.is_empty());
    assert!(!line.block_delete);
  }

  #[test]
  fn whitespace_between_words_is_optional() {
    let packed = lex_line("G1X10Y20").expect("lex");
    let spaced = lex_line("G1 X10 Y20").expect("lex");
    assert_eq!(packed.words, spaced.words);
  }

  #[test]
  fn address_letters_fold_to_uppercase() {
    let line = lex_line("g0 z2.5").expect("lex");
    assert_eq!(line.words, vec![Word { letter: 'G', value: 0.0 }, Word { letter: 'Z', value: 2.5 }]);
  }

  #[test]
  fn parses_leading_and_trailing_dot_numbers() {
    let line = lex_line("X.5 Y5.").expect("lex");
    assert_eq!(line.value('X'), Some(0.5));
    assert_eq!(line.value('Y'), Some(5.0));
  }

  #[test]
  fn paren_comment_is_captured_and_words_continue_after_it() {
    let line = lex_line("G1 (mid-line note) X5").expect("lex");
    assert_eq!(line.comments, vec!["mid-line note".to_string()]);
    assert_eq!(line.value('G'), Some(1.0));
    assert_eq!(line.value('X'), Some(5.0));
  }

  #[test]
  fn semicolon_comment_runs_to_end_of_line() {
    let line = lex_line("G0 X1 ; rapid home").expect("lex");
    assert_eq!(line.value('X'), Some(1.0));
    assert_eq!(line.comments, vec!["rapid home".to_string()]);
  }

  #[test]
  fn unterminated_paren_comment_takes_rest_of_line() {
    let line = lex_line("G1 (oops no close").expect("lex");
    assert_eq!(line.comments, vec!["oops no close".to_string()]);
    assert_eq!(line.value('G'), Some(1.0));
  }

  #[test]
  fn block_delete_prefix_is_flagged() {
    let line = lex_line("/G1 X5").expect("lex");
    assert!(line.block_delete);
    assert_eq!(line.value('X'), Some(5.0));
  }

  #[test]
  fn program_marker_and_line_number_are_handled() {
    let prog = lex("%\nN10 G1 X5\n%\n").expect("lex");
    // The `%` lines carry nothing and drop out; only the real block survives.
    assert_eq!(prog.len(), 1);
    assert_eq!(prog[0].value('N'), Some(10.0));
    assert_eq!(prog[0].value('G'), Some(1.0));
  }

  #[test]
  fn blank_lines_are_dropped_but_comment_lines_survive() {
    let prog = lex("\n   \n(header)\n\nG0 Z1\n").expect("lex");
    assert_eq!(prog.len(), 2);
    assert_eq!(prog[0].comments, vec!["header".to_string()]);
    assert!(prog[0].words.is_empty());
    assert_eq!(prog[1].value('Z'), Some(1.0));
  }

  #[test]
  fn crlf_and_lf_both_separate_lines() {
    let prog = lex("G0 X1\r\nG0 X2\n").expect("lex");
    assert_eq!(prog.len(), 2);
    assert_eq!(prog[0].value('X'), Some(1.0));
    assert_eq!(prog[1].value('X'), Some(2.0));
  }

  #[test]
  fn malformed_number_is_a_parse_error_not_a_panic() {
    let err = lex_line("X1.2.3").unwrap_err();
    assert!(matches!(err, Error::Parse(_)), "got {err:?}");
  }

  #[test]
  fn out_of_grammar_character_is_a_parse_error() {
    assert!(matches!(lex_line("G1 X5 @home").unwrap_err(), Error::Parse(_)));
  }

  #[test]
  fn multiple_words_with_the_same_letter_keep_the_first_via_value() {
    // `value` returns the first occurrence; both are retained in `words` for callers that need every one.
    let line = lex_line("X1 X2").expect("lex");
    assert_eq!(line.value('X'), Some(1.0));
    assert_eq!(line.words.len(), 2);
  }
}
