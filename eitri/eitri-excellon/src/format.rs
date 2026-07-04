//! Excellon number-format handling: the notoriously under-specified units / zero-suppression / decimal-placement,
//! plus inference when the header omits it and an explicit override path.
//!
//! Excellon files frequently fail to state their format, and guessing wrong shifts every hole by a factor of ten
//! (porting plan §5, the top risk). We decode declared formats, infer a sensible default when the header is silent,
//! and let the caller force a format. Provenance: FlatCAM's Excellon format heuristics.
//!
//! The classic trap is the `LZ`/`TZ` header keywords, which name the zeros that are *kept*, not suppressed:
//! - `LZ` ("leading zeros [kept]") ⇒ trailing zeros are omitted ⇒ [`ZeroSuppression::Trailing`] (right-pad).
//! - `TZ` ("trailing zeros [kept]") ⇒ leading zeros are omitted ⇒ [`ZeroSuppression::Leading`] (left-align).

use eitri_core::Unit;

use crate::error::{ExcellonError, Result};

/// Which zeros the file omits from coordinate words (named for what is *suppressed*, unlike the `LZ`/`TZ` header).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZeroSuppression {
  /// Leading zeros omitted; digits are right-aligned to the least-significant end (Excellon header `TZ`).
  Leading,
  /// Trailing zeros omitted; digits are left-aligned to the most-significant end (Excellon header `LZ`).
  Trailing,
}

impl ZeroSuppression {
  /// Map an Excellon header keyword (`LZ`/`TZ`) to the suppression it implies. Returns `None` for anything else.
  pub fn from_keyword(keyword: &str) -> Option<ZeroSuppression> {
    match keyword.trim() {
      "LZ" => Some(ZeroSuppression::Trailing),
      "TZ" => Some(ZeroSuppression::Leading),
      _ => None,
    }
  }
}

/// A resolved Excellon coordinate format.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NumberFormat {
  /// Coordinate unit.
  pub unit: Unit,
  /// Integer digit count.
  pub integer_digits: u8,
  /// Decimal digit count.
  pub decimal_digits: u8,
  /// Which zeros are omitted from coordinate words.
  pub zero_suppression: ZeroSuppression,
}

impl NumberFormat {
  /// The conventional default for a unit when the header does not state digit counts: metric `3.3`, inch `2.4`,
  /// with leading-zero suppression (the most common in the wild).
  pub fn default_for(unit: Unit) -> NumberFormat {
    let (integer_digits, decimal_digits) = match unit {
      Unit::Millimeters => (3, 3),
      Unit::Inches => (2, 4),
    };
    NumberFormat { unit, integer_digits, decimal_digits, zero_suppression: ZeroSuppression::Leading }
  }

  fn total_digits(&self) -> usize {
    self.integer_digits as usize + self.decimal_digits as usize
  }

  /// Decode one coordinate word into a value **in file units**. Handles a sign, an explicit decimal point, and the
  /// zero-suppression padding.
  pub fn decode(&self, word: &str, line: usize) -> Result<f64> {
    let word = word.trim();
    if word.is_empty() {
      return Err(ExcellonError::Syntax { line, message: "empty coordinate word".to_string() });
    }
    let (sign, digits) = match word.strip_prefix('-') {
      Some(rest) => (-1.0, rest),
      None => (1.0, word.strip_prefix('+').unwrap_or(word)),
    };

    if digits.contains('.') {
      let value: f64 = digits.parse().map_err(|_| ExcellonError::Syntax {
        line,
        message: format!("invalid decimal coordinate '{word}'"),
      })?;
      return Ok(sign * value);
    }

    if digits.is_empty() || !digits.chars().all(|c| c.is_ascii_digit()) {
      return Err(ExcellonError::Syntax { line, message: format!("non-numeric coordinate '{word}'") });
    }

    let scale = 10f64.powi(self.decimal_digits as i32);
    let magnitude = match self.zero_suppression {
      ZeroSuppression::Leading => digits.parse::<u64>().map(|n| n as f64 / scale),
      ZeroSuppression::Trailing => {
        let total = self.total_digits();
        let padded = if digits.len() < total {
          format!("{:0<width$}", digits, width = total)
        } else {
          digits.to_string()
        };
        padded.parse::<u64>().map(|n| n as f64 / scale)
      }
    };
    magnitude
      .map(|m| sign * m)
      .map_err(|_| ExcellonError::Syntax { line, message: format!("coordinate '{word}' out of range") })
  }

  /// Convert a decoded file-unit value to millimetres.
  pub fn to_mm(&self, value: f64) -> f64 {
    value * self.unit.mm_per_unit()
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  const L: usize = 1;

  fn fmt(unit: Unit, int: u8, dec: u8, zs: ZeroSuppression) -> NumberFormat {
    NumberFormat { unit, integer_digits: int, decimal_digits: dec, zero_suppression: zs }
  }

  #[test]
  fn header_keyword_mapping_is_inverted() {
    // The whole point: LZ keeps leading zeros (suppresses trailing), TZ keeps trailing (suppresses leading).
    assert_eq!(ZeroSuppression::from_keyword("LZ"), Some(ZeroSuppression::Trailing));
    assert_eq!(ZeroSuppression::from_keyword("TZ"), Some(ZeroSuppression::Leading));
    assert_eq!(ZeroSuppression::from_keyword("XX"), None);
  }

  #[test]
  fn metric_leading_suppression_3_3() {
    let f = fmt(Unit::Millimeters, 3, 3, ZeroSuppression::Leading);
    assert_eq!(f.decode("1500", L).unwrap(), 1.5); // 001500 leading-dropped
    assert_eq!(f.decode("125000", L).unwrap(), 125.0);
    assert_eq!(f.decode("-500", L).unwrap(), -0.5);
  }

  #[test]
  fn inch_trailing_suppression_2_4() {
    let f = fmt(Unit::Inches, 2, 4, ZeroSuppression::Trailing);
    // 1.0 inch -> 010000 -> trailing-drop -> "01"; 0.5 -> 005000 -> "005".
    assert_eq!(f.decode("01", L).unwrap(), 1.0);
    assert_eq!(f.decode("005", L).unwrap(), 0.5);
    assert!((f.to_mm(f.decode("01", L).unwrap()) - 25.4).abs() < 1e-9);
  }

  #[test]
  fn explicit_decimal_ignores_suppression() {
    let f = fmt(Unit::Millimeters, 3, 3, ZeroSuppression::Leading);
    assert_eq!(f.decode("2.5", L).unwrap(), 2.5);
    assert_eq!(f.decode("-0.8", L).unwrap(), -0.8);
  }

  #[test]
  fn defaults_differ_by_unit() {
    assert_eq!(NumberFormat::default_for(Unit::Millimeters).decimal_digits, 3);
    assert_eq!(NumberFormat::default_for(Unit::Inches).decimal_digits, 4);
  }
}
