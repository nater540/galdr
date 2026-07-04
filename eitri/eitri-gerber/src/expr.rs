//! A tiny arithmetic evaluator for aperture-macro (`AM`) parametric expressions.
//!
//! Macro parameters are expressions over the macro's arguments (`$1`, `$2`, …) with `+ - x * /` and parentheses,
//! e.g. `$1x0.75` or `($2-$3)/2`. This is a straight recursive-descent evaluator — the macro interpreter calls it
//! for every primitive parameter. Provenance: reproduces FlatCAM's `ApertureMacro` expression handling.
//!
//! Grammar (whitespace insignificant):
//! ```text
//! expr   := term  (('+' | '-') term)*
//! term   := factor (('x' | '*' | '/') factor)*
//! factor := ('+' | '-') factor | '(' expr ')' | number | '$' digits
//! ```

use crate::error::{GerberError, Result};

/// Evaluate a macro expression, resolving `$n` against `vars` (1-based: `$1` is `vars[0]`).
pub fn eval(input: &str, vars: &[f64], line: usize) -> Result<f64> {
  let mut parser = Parser { chars: input.as_bytes(), pos: 0, vars, line };
  let value = parser.expr()?;
  parser.skip_ws();
  if parser.pos != parser.chars.len() {
    return Err(GerberError::Syntax {
      line,
      message: format!("trailing characters in macro expression '{input}'"),
    });
  }
  Ok(value)
}

struct Parser<'a> {
  chars: &'a [u8],
  pos: usize,
  vars: &'a [f64],
  line: usize,
}

impl Parser<'_> {
  fn skip_ws(&mut self) {
    while self.pos < self.chars.len() && (self.chars[self.pos] as char).is_whitespace() {
      self.pos += 1;
    }
  }

  fn peek(&mut self) -> Option<char> {
    self.skip_ws();
    self.chars.get(self.pos).map(|&b| b as char)
  }

  fn expr(&mut self) -> Result<f64> {
    let mut value = self.term()?;
    while let Some(op) = self.peek() {
      match op {
        '+' => { self.pos += 1; value += self.term()?; }
        '-' => { self.pos += 1; value -= self.term()?; }
        _ => break,
      }
    }
    Ok(value)
  }

  fn term(&mut self) -> Result<f64> {
    let mut value = self.factor()?;
    while let Some(op) = self.peek() {
      match op {
        // Gerber uses lowercase 'x' for multiply historically; '*' is also accepted.
        'x' | 'X' | '*' => { self.pos += 1; value *= self.factor()?; }
        '/' => {
          self.pos += 1;
          let divisor = self.factor()?;
          if divisor == 0.0 {
            return Err(GerberError::Syntax { line: self.line, message: "division by zero in macro".to_string() });
          }
          value /= divisor;
        }
        _ => break,
      }
    }
    Ok(value)
  }

  fn factor(&mut self) -> Result<f64> {
    match self.peek() {
      Some('+') => { self.pos += 1; self.factor() }
      Some('-') => { self.pos += 1; Ok(-self.factor()?) }
      Some('(') => {
        self.pos += 1;
        let value = self.expr()?;
        if self.peek() == Some(')') {
          self.pos += 1;
          Ok(value)
        } else {
          Err(GerberError::Syntax { line: self.line, message: "unbalanced parentheses in macro".to_string() })
        }
      }
      Some('$') => { self.pos += 1; self.variable() }
      Some(c) if c.is_ascii_digit() || c == '.' => self.number(),
      other => Err(GerberError::Syntax {
        line: self.line,
        message: format!("unexpected token {other:?} in macro expression"),
      }),
    }
  }

  fn variable(&mut self) -> Result<f64> {
    let start = self.pos;
    while self.pos < self.chars.len() && (self.chars[self.pos] as char).is_ascii_digit() {
      self.pos += 1;
    }
    let index: usize = std::str::from_utf8(&self.chars[start..self.pos])
      .ok()
      .and_then(|s| s.parse().ok())
      .ok_or_else(|| GerberError::Syntax { line: self.line, message: "malformed $ variable in macro".to_string() })?;
    if index == 0 || index > self.vars.len() {
      // An unset variable evaluates to zero (per the Gerber spec's default for undefined macro variables).
      return Ok(0.0);
    }
    Ok(self.vars[index - 1])
  }

  fn number(&mut self) -> Result<f64> {
    let start = self.pos;
    while self.pos < self.chars.len() {
      let c = self.chars[self.pos] as char;
      if c.is_ascii_digit() || c == '.' {
        self.pos += 1;
      } else {
        break;
      }
    }
    std::str::from_utf8(&self.chars[start..self.pos])
      .ok()
      .and_then(|s| s.parse().ok())
      .ok_or_else(|| GerberError::Syntax { line: self.line, message: "malformed number in macro".to_string() })
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  const L: usize = 1;

  #[test]
  fn constants_and_precedence() {
    assert_eq!(eval("2+3x4", &[], L).unwrap(), 14.0); // x binds tighter than +
    assert_eq!(eval("(2+3)x4", &[], L).unwrap(), 20.0);
    assert_eq!(eval("10/2-1", &[], L).unwrap(), 4.0);
  }

  #[test]
  fn variables_are_one_based() {
    let vars = [1.5, 4.0, 2.0];
    assert_eq!(eval("$1", &vars, L).unwrap(), 1.5);
    assert_eq!(eval("$2x0.5", &vars, L).unwrap(), 2.0);
    assert_eq!(eval("($2-$3)/2", &vars, L).unwrap(), 1.0);
  }

  #[test]
  fn unary_minus_and_star_multiply() {
    assert_eq!(eval("-3", &[], L).unwrap(), -3.0);
    assert_eq!(eval("2*-3", &[], L).unwrap(), -6.0);
  }

  #[test]
  fn undefined_variable_is_zero() {
    assert_eq!(eval("$9", &[1.0], L).unwrap(), 0.0);
  }

  #[test]
  fn errors_on_garbage_and_division_by_zero() {
    assert!(eval("2+", &[], L).is_err());
    assert!(eval("2%3", &[], L).is_err());
    assert!(eval("1/0", &[], L).is_err());
    assert!(eval("(1+2", &[], L).is_err());
  }
}
