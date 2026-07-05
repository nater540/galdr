//! The Gerber block lexer: split the raw stream into statements.
//!
//! Gerber interleaves two statement kinds — extended commands delimited by `%...%` (which may contain several
//! `*`-terminated sub-blocks, as an aperture macro does) and ordinary `*`-terminated function-code words. Rather
//! than a regex per command (FlatCAM's approach), we tokenize once into [`Statement`]s and let the interpreter
//! drive off that. Source line numbers are tracked for diagnostics.

use crate::error::{GerberError, Result};

/// One lexed Gerber statement.
#[derive(Debug, Clone, PartialEq)]
pub enum Statement {
  /// A function-code word block (without the trailing `*`), e.g. `G01X100Y100D01`.
  Word(String),
  /// An extended command's `*`-terminated sub-blocks (without `%` or `*`), e.g. `["FSLAX36Y36"]` or an `AM` body.
  Extended(Vec<String>),
}

/// A statement paired with the 1-based source line on which it began.
#[derive(Debug, Clone, PartialEq)]
pub struct Located {
  /// 1-based source line.
  pub line: usize,
  /// The statement.
  pub statement: Statement,
}

/// Tokenize a Gerber source string into located statements. Whitespace and newlines between blocks are ignored;
/// line numbers are tracked so errors can point at the source.
pub fn tokenize(source: &str) -> Result<Vec<Located>> {
  let mut out = Vec::new();
  let mut line = 1usize;
  let mut chars = source.char_indices().peekable();

  while let Some(&(_, c)) = chars.peek() {
    match c {
      '\n' => {
        line += 1;
        chars.next();
      }
      c if c.is_whitespace() => {
        chars.next();
      }
      '%' => {
        let start_line = line;
        chars.next(); // consume opening '%'
        let mut blocks: Vec<String> = Vec::new();
        let mut current = String::new();
        let mut closed = false;
        for (_, ch) in chars.by_ref() {
          match ch {
            '\n' => line += 1,
            '%' => {
              closed = true;
              break;
            }
            '*' => {
              blocks.push(std::mem::take(&mut current));
            }
            other if other.is_whitespace() => {}
            other => current.push(other),
          }
        }
        if !current.trim().is_empty() {
          blocks.push(std::mem::take(&mut current));
        }
        if !closed {
          return Err(GerberError::Syntax { line: start_line, message: "unterminated %...% command".to_string() });
        }
        out.push(Located { line: start_line, statement: Statement::Extended(blocks) });
      }
      '*' => {
        // A stray terminator with no word before it — skip.
        chars.next();
      }
      _ => {
        let start_line = line;
        let mut word = String::new();
        let mut terminated = false;
        for (_, ch) in chars.by_ref() {
          match ch {
            '\n' => line += 1,
            '*' => {
              terminated = true;
              break;
            }
            other if other.is_whitespace() => {}
            other => word.push(other),
          }
        }
        if !terminated {
          return Err(GerberError::Syntax { line: start_line, message: "unterminated word block (missing *)".to_string() });
        }
        if !word.is_empty() {
          out.push(Located { line: start_line, statement: Statement::Word(word) });
        }
      }
    }
  }

  Ok(out)
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn splits_extended_and_word_statements() {
    let src = "%FSLAX36Y36*%\n%MOMM*%\nG04 a comment*\nX100Y100D02*\nD03*\nM02*\n";
    let toks = tokenize(src).expect("tokenize");
    assert_eq!(toks[0].statement, Statement::Extended(vec!["FSLAX36Y36".to_string()]));
    assert_eq!(toks[1].statement, Statement::Extended(vec!["MOMM".to_string()]));
    assert_eq!(toks[2].statement, Statement::Word("G04acomment".to_string()));
    assert_eq!(toks[3].statement, Statement::Word("X100Y100D02".to_string()));
    assert_eq!(toks[4].statement, Statement::Word("D03".to_string()));
    assert_eq!(toks[5].statement, Statement::Word("M02".to_string()));
  }

  #[test]
  fn groups_multi_block_aperture_macro() {
    let src = "%AMDONUT*\n1,1,$1,0,0*\n1,0,$2,0,0*%\n";
    let toks = tokenize(src).expect("tokenize");
    assert_eq!(
      toks[0].statement,
      Statement::Extended(vec!["AMDONUT".to_string(), "1,1,$1,0,0".to_string(), "1,0,$2,0,0".to_string()])
    );
  }

  #[test]
  fn tracks_line_numbers() {
    let toks = tokenize("G04 x*\n\n\nD10*\n").expect("tokenize");
    assert_eq!(toks[0].line, 1);
    assert_eq!(toks[1].line, 4);
  }

  #[test]
  fn unterminated_percent_is_an_error() {
    assert!(tokenize("%FSLAX36Y36*").is_err());
  }
}
