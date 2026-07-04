//! Coordinate format decoding — the RS-274X `FS` block and the zero-omission arithmetic.
//!
//! This is the single most error-prone corner of Gerber import (porting plan §4): a coordinate word carries no
//! decimal point, so reconstructing its value depends entirely on the `FS` declaration — integer/decimal digit
//! counts and whether *leading* or *trailing* zeros were omitted. Getting the padding side wrong shifts every
//! coordinate by powers of ten. The logic is pure and exhaustively unit-tested against every variant.
//!
//! Provenance: reproduces the coordinate-reconstruction behaviour of FlatCAM's `camlib` Gerber `FS` handling.

use crate::error::{GerberError, Result};

/// Which zeros the file omits from coordinate words.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZeroOmission {
  /// Leading zeros omitted — the remaining digits are aligned to the least-significant end (the modern default).
  Leading,
  /// Trailing zeros omitted — the remaining digits are aligned to the most-significant end.
  Trailing,
}

/// Absolute vs incremental coordinate notation. Incremental is deprecated; we record it but only support absolute.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Notation {
  /// Coordinates are absolute positions.
  Absolute,
  /// Coordinates are deltas from the previous point (deprecated).
  Incremental,
}

/// A parsed `FS` coordinate format: digit counts plus omission and notation modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CoordinateFormat {
  /// Number of integer digits in a coordinate field (before the implied decimal point).
  pub integer_digits: u8,
  /// Number of decimal digits (after the implied decimal point).
  pub decimal_digits: u8,
  /// Which zeros are omitted.
  pub zero_omission: ZeroOmission,
  /// Absolute or incremental notation.
  pub notation: Notation,
}

impl CoordinateFormat {
  /// Parse the body of an `FS` block, e.g. `LAX36Y36` from `%FSLAX36Y36*%`. Expects the zero-omission letter
  /// (`L`/`T`), the notation letter (`A`/`I`), then `X<int><dec>` and `Y<int><dec>` with matching digit counts.
  pub fn parse_fs(body: &str, line: usize) -> Result<CoordinateFormat> {
    let mut chars = body.chars().peekable();

    let zero_omission = match chars.next() {
      Some('L') => ZeroOmission::Leading,
      Some('T') => ZeroOmission::Trailing,
      // A leading 'D' (deprecated) or absent letter defaults to leading-zero omission per common practice.
      other => {
        return Err(GerberError::Syntax {
          line,
          message: format!("FS zero-omission letter must be L or T, found {other:?}"),
        });
      }
    };

    let notation = match chars.next() {
      Some('A') => Notation::Absolute,
      Some('I') => Notation::Incremental,
      other => {
        return Err(GerberError::Syntax {
          line,
          message: format!("FS notation letter must be A or I, found {other:?}"),
        });
      }
    };

    let (x_int, x_dec) = Self::parse_axis(&mut chars, 'X', line)?;
    let (y_int, y_dec) = Self::parse_axis(&mut chars, 'Y', line)?;
    if (x_int, x_dec) != (y_int, y_dec) {
      return Err(GerberError::Syntax {
        line,
        message: format!("FS X and Y digit counts differ ({x_int}{x_dec} vs {y_int}{y_dec})"),
      });
    }

    Ok(CoordinateFormat { integer_digits: x_int, decimal_digits: x_dec, zero_omission, notation })
  }

  /// Read a `X`/`Y` axis specifier (`X36` → 3 integer, 6 decimal digits) from the char stream.
  fn parse_axis(
    chars: &mut std::iter::Peekable<std::str::Chars<'_>>,
    axis: char,
    line: usize,
  ) -> Result<(u8, u8)> {
    match chars.next() {
      Some(c) if c == axis => {}
      other => {
        return Err(GerberError::Syntax {
          line,
          message: format!("expected FS axis '{axis}', found {other:?}"),
        });
      }
    }
    let int_digit = chars.next().and_then(|c| c.to_digit(10));
    let dec_digit = chars.next().and_then(|c| c.to_digit(10));
    match (int_digit, dec_digit) {
      (Some(i), Some(d)) => Ok((i as u8, d as u8)),
      _ => Err(GerberError::Syntax {
        line,
        message: format!("FS axis '{axis}' needs two digits (integer then decimal count)"),
      }),
    }
  }

  /// Decode one coordinate word into its numeric value **in file units** (the `MO` unit; the caller scales to mm).
  /// Delegates the sign / explicit-decimal / zero-omission arithmetic to the shared `eitri_core` decoder so the
  /// Gerber and Excellon parsers cannot drift apart, attaching this parser's line-aware error on failure.
  pub fn decode(&self, word: &str, line: usize) -> Result<f64> {
    let omission = match self.zero_omission {
      ZeroOmission::Leading => eitri_core::ZeroOmission::Leading,
      ZeroOmission::Trailing => eitri_core::ZeroOmission::Trailing,
    };
    eitri_core::decode_zero_omitted(word, self.integer_digits, self.decimal_digits, omission)
      .map_err(|err| decode_error(word, line, err))
  }
}

/// Map a shared decode failure onto this parser's line-aware syntax error, preserving the original messages.
fn decode_error(word: &str, line: usize, err: eitri_core::CoordDecodeError) -> GerberError {
  use eitri_core::CoordDecodeError as E;
  let message = match err {
    E::Empty => "empty coordinate word".to_string(),
    E::InvalidDecimal => format!("invalid decimal coordinate '{word}'"),
    E::NonNumeric => format!("non-numeric coordinate '{word}'"),
    E::OutOfRange => format!("coordinate '{word}' out of range"),
  };
  GerberError::Syntax { line, message }
}

#[cfg(test)]
mod tests {
  use super::*;

  const L: usize = 1;

  fn fmt(int: u8, dec: u8, zo: ZeroOmission) -> CoordinateFormat {
    CoordinateFormat { integer_digits: int, decimal_digits: dec, zero_omission: zo, notation: Notation::Absolute }
  }

  #[test]
  fn parse_fs_leading_absolute() {
    let f = CoordinateFormat::parse_fs("LAX36Y36", L).expect("fs");
    assert_eq!(f.integer_digits, 3);
    assert_eq!(f.decimal_digits, 6);
    assert_eq!(f.zero_omission, ZeroOmission::Leading);
    assert_eq!(f.notation, Notation::Absolute);
  }

  #[test]
  fn parse_fs_trailing_incremental() {
    let f = CoordinateFormat::parse_fs("TIX24Y24", L).expect("fs");
    assert_eq!((f.integer_digits, f.decimal_digits), (2, 4));
    assert_eq!(f.zero_omission, ZeroOmission::Trailing);
    assert_eq!(f.notation, Notation::Incremental);
  }

  #[test]
  fn parse_fs_rejects_mismatched_axes() {
    assert!(CoordinateFormat::parse_fs("LAX36Y46", L).is_err());
  }

  #[test]
  fn parse_fs_rejects_bad_letters() {
    assert!(CoordinateFormat::parse_fs("QAX36Y36", L).is_err());
    assert!(CoordinateFormat::parse_fs("LQX36Y36", L).is_err());
  }

  // The critical matrix: the same physical values under both omission modes must decode identically.

  #[test]
  fn leading_omission_2_4() {
    let f = fmt(2, 4, ZeroOmission::Leading);
    assert_eq!(f.decode("250000", L).unwrap(), 25.0);
    assert_eq!(f.decode("50000", L).unwrap(), 5.0); // leading zero of "050000" dropped
    assert_eq!(f.decode("5", L).unwrap(), 0.0005); // right-aligned to the LSB
    assert_eq!(f.decode("0", L).unwrap(), 0.0);
    assert_eq!(f.decode("-125000", L).unwrap(), -12.5);
  }

  #[test]
  fn trailing_omission_2_4() {
    let f = fmt(2, 4, ZeroOmission::Trailing);
    // 50.0 -> "500000" -> trailing-drop -> "5"; 5.0 -> "050000" -> "05"; 25.0 -> "250000" -> "25".
    assert_eq!(f.decode("5", L).unwrap(), 50.0);
    assert_eq!(f.decode("05", L).unwrap(), 5.0);
    assert_eq!(f.decode("25", L).unwrap(), 25.0);
    assert_eq!(f.decode("-5", L).unwrap(), -50.0);
  }

  #[test]
  fn leading_and_trailing_agree_on_full_width_fields() {
    // A full-width field (no omission) decodes the same regardless of mode.
    let lead = fmt(3, 6, ZeroOmission::Leading);
    let trail = fmt(3, 6, ZeroOmission::Trailing);
    assert_eq!(lead.decode("001500000", L).unwrap(), 1.5);
    assert_eq!(trail.decode("001500000", L).unwrap(), 1.5);
  }

  #[test]
  fn kicad_default_3_6_leading() {
    // KiCad emits FSLAX36Y36: 1.0mm -> 1000000, 25.4 -> 25400000.
    let f = fmt(3, 6, ZeroOmission::Leading);
    assert_eq!(f.decode("1000000", L).unwrap(), 1.0);
    assert_eq!(f.decode("25400000", L).unwrap(), 25.4);
    assert_eq!(f.decode("-500000", L).unwrap(), -0.5);
  }

  #[test]
  fn explicit_decimal_point_overrides_format() {
    let f = fmt(2, 4, ZeroOmission::Leading);
    assert_eq!(f.decode("1.5", L).unwrap(), 1.5);
    assert_eq!(f.decode("-0.25", L).unwrap(), -0.25);
  }

  #[test]
  fn decode_rejects_garbage() {
    let f = fmt(2, 4, ZeroOmission::Leading);
    assert!(f.decode("12a4", L).is_err());
    assert!(f.decode("", L).is_err());
  }
}
