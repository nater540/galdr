//! Shared zero-omission coordinate decoding for the Gerber and Excellon parsers.
//!
//! Both formats carry coordinates as bare integer strings with an *implied* decimal point, and both omit either the
//! leading or the trailing zeros — getting the padding side wrong shifts every coordinate by a power of ten (the top
//! risk in both porting-plan §4 and §5). The two parsers had near-identical copies of this arithmetic; folding it
//! here means one audited implementation with one set of edge cases (sign, explicit decimal point, padding).

/// Which zeros the source omits from a coordinate word (named for what is *suppressed*).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZeroOmission {
  /// Leading zeros omitted — the remaining digits are right-aligned to the least-significant end.
  Leading,
  /// Trailing zeros omitted — the remaining digits are left-aligned to the most-significant end.
  Trailing,
}

/// Why a coordinate word could not be decoded. The caller attaches file/line context and its own error type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoordDecodeError {
  /// The word was empty (after trimming whitespace and any sign).
  Empty,
  /// The word carried an explicit decimal point but did not parse as a decimal number.
  InvalidDecimal,
  /// The word had no explicit decimal point and contained a non-digit character.
  NonNumeric,
  /// The integer digits overflowed the accumulator.
  OutOfRange,
}

/// Decode one zero-omitted coordinate word into its numeric value **in file units** (the caller scales to mm).
///
/// Handles a leading sign, an explicit decimal point (trusted directly, ignoring the implied format), and the
/// leading/trailing zero-omission padding driven by `integer_digits`/`decimal_digits`.
pub fn decode_zero_omitted(
  word: &str,
  integer_digits: u8,
  decimal_digits: u8,
  omission: ZeroOmission,
) -> Result<f64, CoordDecodeError> {
  let word = word.trim();
  if word.is_empty() {
    return Err(CoordDecodeError::Empty);
  }

  let (sign, digits) = match word.strip_prefix('-') {
    Some(rest) => (-1.0, rest),
    None => (1.0, word.strip_prefix('+').unwrap_or(word)),
  };

  // Explicit decimal point: trust it directly and ignore the implied format.
  if digits.contains('.') {
    let value: f64 = digits.parse().map_err(|_| CoordDecodeError::InvalidDecimal)?;
    return Ok(sign * value);
  }

  if digits.is_empty() || !digits.chars().all(|c| c.is_ascii_digit()) {
    return Err(CoordDecodeError::NonNumeric);
  }

  let scale = 10f64.powi(decimal_digits as i32);
  let magnitude = match omission {
    // Leading omission: digits are already right-aligned to the LSB, so the integer value over 10^decimals is the
    // coordinate — no padding needed.
    ZeroOmission::Leading => digits.parse::<u64>().map(|n| n as f64 / scale),
    // Trailing omission: digits are left-aligned to the MSB, so right-pad to the full field width first.
    ZeroOmission::Trailing => {
      let total = integer_digits as usize + decimal_digits as usize;
      let padded = if digits.len() < total {
        format!("{:0<width$}", digits, width = total)
      } else {
        digits.to_string()
      };
      padded.parse::<u64>().map(|n| n as f64 / scale)
    }
  };

  magnitude.map(|m| sign * m).map_err(|_| CoordDecodeError::OutOfRange)
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn leading_omission_right_aligns_to_the_lsb() {
    // 2 integer + 4 decimal digits, leading zeros omitted.
    assert_eq!(decode_zero_omitted("250000", 2, 4, ZeroOmission::Leading).unwrap(), 25.0);
    assert_eq!(decode_zero_omitted("5", 2, 4, ZeroOmission::Leading).unwrap(), 0.0005);
    assert_eq!(decode_zero_omitted("-125000", 2, 4, ZeroOmission::Leading).unwrap(), -12.5);
  }

  #[test]
  fn trailing_omission_left_pads_to_full_width() {
    // The same physical values under trailing omission: 50.0 -> "5", 5.0 -> "05", 25.0 -> "25".
    assert_eq!(decode_zero_omitted("5", 2, 4, ZeroOmission::Trailing).unwrap(), 50.0);
    assert_eq!(decode_zero_omitted("05", 2, 4, ZeroOmission::Trailing).unwrap(), 5.0);
    assert_eq!(decode_zero_omitted("-5", 2, 4, ZeroOmission::Trailing).unwrap(), -50.0);
  }

  #[test]
  fn explicit_decimal_point_overrides_the_implied_format() {
    assert_eq!(decode_zero_omitted("1.5", 2, 4, ZeroOmission::Leading).unwrap(), 1.5);
    assert_eq!(decode_zero_omitted("-0.25", 3, 6, ZeroOmission::Trailing).unwrap(), -0.25);
  }

  #[test]
  fn full_width_fields_agree_across_omission_modes() {
    assert_eq!(decode_zero_omitted("001500000", 3, 6, ZeroOmission::Leading).unwrap(), 1.5);
    assert_eq!(decode_zero_omitted("001500000", 3, 6, ZeroOmission::Trailing).unwrap(), 1.5);
  }

  #[test]
  fn rejects_empty_and_garbage() {
    assert_eq!(decode_zero_omitted("", 2, 4, ZeroOmission::Leading), Err(CoordDecodeError::Empty));
    assert_eq!(decode_zero_omitted("12a4", 2, 4, ZeroOmission::Leading), Err(CoordDecodeError::NonNumeric));
    assert_eq!(decode_zero_omitted("1.2.3", 2, 4, ZeroOmission::Leading), Err(CoordDecodeError::InvalidDecimal));
  }
}
