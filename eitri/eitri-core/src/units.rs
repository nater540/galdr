//! Length as a checked type.
//!
//! FlatCAM tracked units globally and converted on the fly, which produced a recurring class of double-conversion
//! bugs (a tool diameter converted to mm twice, a coordinate offset applied in the wrong unit). Eitri removes that
//! class entirely: `Length` stores one canonical representation (millimetres) and the only way to get a bare `f64`
//! out is to name the unit you want it in — `as_mm()` / `as_inch()`. There is no implicit or global unit state.

/// A physical length. Constructed from a named unit, read back in a named unit; the internal representation
/// (millimetres) is private so no caller can accidentally mix units.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
pub struct Length {
  /// Canonical value in millimetres. Private on purpose — see the module docs.
  mm: f64,
}

/// Millimetres in one inch — the single conversion constant.
pub const MM_PER_INCH: f64 = 25.4;

impl Length {
  /// A zero length.
  pub const ZERO: Length = Length { mm: 0.0 };

  /// Construct from a value in millimetres.
  pub const fn from_mm(mm: f64) -> Length {
    Length { mm }
  }

  /// Construct from a value in inches.
  pub const fn from_inch(inch: f64) -> Length {
    Length { mm: inch * MM_PER_INCH }
  }

  /// Construct from a value expressed in the given unit.
  pub const fn new(value: f64, unit: Unit) -> Length {
    match unit {
      Unit::Millimeters => Length::from_mm(value),
      Unit::Inches => Length::from_inch(value),
    }
  }

  /// Read the length in millimetres.
  pub const fn as_mm(self) -> f64 {
    self.mm
  }

  /// Read the length in inches.
  pub const fn as_inch(self) -> f64 {
    self.mm / MM_PER_INCH
  }

  /// Read the length in the given unit.
  pub const fn as_unit(self, unit: Unit) -> f64 {
    match unit {
      Unit::Millimeters => self.as_mm(),
      Unit::Inches => self.as_inch(),
    }
  }

  /// Absolute value of the length.
  pub fn abs(self) -> Length {
    Length { mm: self.mm.abs() }
  }
}

impl std::ops::Add for Length {
  type Output = Length;
  fn add(self, rhs: Length) -> Length {
    Length { mm: self.mm + rhs.mm }
  }
}

impl std::ops::Sub for Length {
  type Output = Length;
  fn sub(self, rhs: Length) -> Length {
    Length { mm: self.mm - rhs.mm }
  }
}

impl std::ops::Neg for Length {
  type Output = Length;
  fn neg(self) -> Length {
    Length { mm: -self.mm }
  }
}

impl std::ops::Mul<f64> for Length {
  type Output = Length;
  fn mul(self, rhs: f64) -> Length {
    Length { mm: self.mm * rhs }
  }
}

impl std::ops::Div<f64> for Length {
  type Output = Length;
  fn div(self, rhs: f64) -> Length {
    Length { mm: self.mm / rhs }
  }
}

/// A named length unit. Used at document boundaries (a Gerber `MO` block, a project's display unit) to say what a
/// bare number means; internal geometry always works in [`Length`] / millimetres.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Unit {
  /// Millimetres.
  Millimeters,
  /// Inches.
  Inches,
}

impl Unit {
  /// The multiplier that converts a value in this unit into millimetres.
  pub const fn mm_per_unit(self) -> f64 {
    match self {
      Unit::Millimeters => 1.0,
      Unit::Inches => MM_PER_INCH,
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  const EPS: f64 = 1e-9;

  #[test]
  fn inch_to_mm_round_trips() {
    let one_inch = Length::from_inch(1.0);
    assert!((one_inch.as_mm() - 25.4).abs() < EPS);
    assert!((one_inch.as_inch() - 1.0).abs() < EPS);
  }

  #[test]
  fn mm_and_inch_agree_through_new_and_as_unit() {
    let quarter = Length::new(0.25, Unit::Inches);
    assert!((quarter.as_mm() - 6.35).abs() < EPS);
    assert!((quarter.as_unit(Unit::Inches) - 0.25).abs() < EPS);
    assert!((quarter.as_unit(Unit::Millimeters) - 6.35).abs() < EPS);
  }

  #[test]
  fn arithmetic_stays_in_canonical_mm() {
    let a = Length::from_mm(10.0);
    let b = Length::from_inch(1.0);
    assert!(((a + b).as_mm() - 35.4).abs() < EPS);
    assert!(((a - b).as_mm() - (-15.4)).abs() < EPS);
    assert!(((a * 2.0).as_mm() - 20.0).abs() < EPS);
    assert!(((a / 4.0).as_mm() - 2.5).abs() < EPS);
    assert!(((-b).as_mm() - (-25.4)).abs() < EPS);
    assert!((b.abs().as_mm() - 25.4).abs() < EPS);
  }

  #[test]
  fn ordering_compares_true_lengths_not_raw_numbers() {
    // 1 inch (25.4 mm) is greater than 20 mm even though 1 < 20 numerically — the type compares real lengths.
    assert!(Length::from_inch(1.0) > Length::from_mm(20.0));
    assert_eq!(Length::ZERO, Length::from_mm(0.0));
  }

  #[test]
  fn no_double_conversion() {
    // The classic FlatCAM bug: converting an already-converted value again. Here it is structurally impossible —
    // a Length is always canonical, so re-reading in any unit is idempotent.
    let d = Length::new(3.0, Unit::Millimeters);
    let again = Length::from_mm(d.as_mm());
    assert!((again.as_inch() - d.as_inch()).abs() < EPS);
  }
}
