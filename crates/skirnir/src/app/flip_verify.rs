//! The 180°-flip center-verify wizard (DOC-11 §2.1): validate / refine a center by cancelling eccentricity.
//!
//! Probe a feature at angle θ (reading `r1`), rotate the part 180° and probe the SAME side again (reading `r2`).
//! Both touches are `G38.x` SURFACE contacts on the same (e.g. +Y) face, so each reading carries the probe
//! contact radius `R`: with the rotation axis at machine `C` and the feature centre offset by eccentricity `e`,
//! `r1 = C + e + R` and `r2 = C − e + R`. Two consequences drive the math:
//! - `error = (r2 − r1) / 2 = −e` — the residual eccentricity. The radius `R` CANCELS in the difference, so this
//!   is radius-free and position-free. It is what we SHOW, and it is zero exactly when the feature is centred.
//! - The absolute axis `C` canNOT be recovered from two same-side touches (it would need `R = D/2`). What we CAN
//!   do is REFINE the existing centre: Phase 1's two-sided probe already placed the work origin `O` at the dowel
//!   centre, and `error = C − O` (derivation below), so the correction is a RELATIVE shift of the current work
//!   origin by `error`: `new_origin = current_origin + error`. This is radius-free and a true NO-OP when the
//!   feature is already centred (`error = 0`).
//!
//! Derivation of `error = C − O` (so `C = O + error`): with the flip taken at Phase 1's index angle, the dowel
//! centre at θ equals the current origin (`O = C + e`), so `r1 = O + R` and `r2 = (2C − O) + R`; then
//! `(r2 − r1)/2 = C − O`. Moving work-0 to `C` therefore means writing `O + error`.
//!
//! This module is pure math + the offered `G10` line over a completed two-reading [`super::angle_sweep`]; the
//! sweep does the probing/sequencing and the shell supplies the current origin (the tracked `WCO`). The
//! correction carries ONLY the verified axis word — never an `A` word — so the rotary datum is untouched. Note the
//! refine assumes no `G92`/TLO is active on the verified axis (the common probing case): it adjusts the work
//! origin via the tracked total offset; a residual far from zero usually means the workholding is eccentric and
//! should be physically dialled in rather than papered over with an offset.

use super::intent::Axis;
use super::rotary_center::Wcs;

/// The result of a 180°-flip verify, computed from the two readings `(r1, r2)` along the verified `axis`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlipResult {
  /// The axis the feature was probed along (the axis the correction would adjust).
  pub axis: Axis,
  /// The reading at the first angle θ (machine coordinate on `axis`).
  pub r1: f64,
  /// The reading at θ + 180° (machine coordinate on `axis`).
  pub r2: f64,
}

impl FlipResult {
  /// Build a result from the sweep's two readings. Returns `None` unless exactly two readings are present (the
  /// flip-verify is always a two-touch sweep; anything else is a programming error, not a value to compute on).
  pub fn from_readings(axis: Axis, readings: &[f64]) -> Option<Self> {
    match readings {
      [r1, r2] => Some(FlipResult { axis, r1: *r1, r2: *r2 }),
      _ => None,
    }
  }

  /// The residual eccentricity of the feature centre from the rotation axis: `error = (r2 − r1) / 2`. Radius-free
  /// (the probe contact radius cancels in the difference) and zero when the feature is perfectly centred. This is
  /// the value SHOWN to the operator.
  pub fn error(&self) -> f64 {
    (self.r2 - self.r1) / 2.0
  }

  /// The corrected work-origin MACHINE coordinate on the verified axis: `current_origin + error`. A RELATIVE
  /// refinement of the existing centre (the absolute axis cannot be recovered from two same-side touches), so a
  /// centred feature (`error == 0`) leaves the origin unchanged. `current_origin` is the current work offset on
  /// this axis (the tracked `WCO`, i.e. the machine coordinate of work-0).
  pub fn corrected_origin(&self, current_origin: f64) -> f64 {
    current_origin + self.error()
  }

  /// The offered `G10 L2` correction: shift the work origin on the verified axis by the measured residual, so the
  /// rotation centre is refined onto the feature. `current_origin` is the current `WCO` on this axis. Carries ONLY
  /// the verified axis word — never an `A` word — leaving the rotary datum untouched. Position-independent: the
  /// result depends only on the two readings and the current offset, not on where the tool now sits.
  pub fn offer_g10(&self, wcs: Wcs, current_origin: f64) -> String {
    format!(
      "G10 L2 P{} {}{:.3}",
      wcs.selector_digit(),
      self.axis.letter(),
      self.corrected_origin(current_origin),
    )
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn error_is_half_the_difference_of_the_two_readings() {
    // Realistic surface readings carry the probe radius R; the difference cancels it. With R=5, e=3, c=1:
    // r1 = c+e+R = 9, r2 = c−e+R = 3 → error = (3 − 9)/2 = −3 (magnitude e = 3 mm off the axis).
    let f = FlipResult::from_readings(Axis::Y, &[9.0, 3.0]).expect("two readings");
    assert_eq!(f.error(), -3.0);
  }

  #[test]
  fn a_centered_feature_correction_is_a_no_op() {
    // A feature ON the rotation axis reads the SAME value before/after the flip (r1 = r2 = c + R), so error = 0
    // and the correction must leave the current origin UNCHANGED — never shift it by the radius.
    let f = FlipResult::from_readings(Axis::Y, &[5.0, 5.0]).expect("two readings");
    assert_eq!(f.error(), 0.0);
    assert_eq!(f.corrected_origin(2.0), 2.0, "a centred verify must not move the origin");
    assert_eq!(f.offer_g10(Wcs::Active, 2.0), "G10 L2 P0 Y2.000");
  }

  #[test]
  fn an_eccentric_feature_shifts_the_origin_by_the_residual() {
    // R=5, e=3, c=1 → r1=9, r2=3, error=−3. With the work origin currently at machine-Y = 10, the corrected
    // origin is 10 + (−3) = 7 — a RELATIVE shift by the residual, NOT the surface midpoint (which would be 6).
    let f = FlipResult::from_readings(Axis::Y, &[9.0, 3.0]).expect("two readings");
    assert_eq!(f.corrected_origin(10.0), 7.0);
    assert_eq!(f.offer_g10(Wcs::Active, 10.0), "G10 L2 P0 Y7.000");
  }

  #[test]
  fn the_offered_g10_is_axis_scoped_and_l2() {
    let f = FlipResult::from_readings(Axis::Y, &[9.0, 3.0]).expect("two readings");
    let line = f.offer_g10(Wcs::Active, 10.0);
    assert!(!line.contains('A'), "the correction must never carry an A word; got {line:?}");
    assert!(line.contains("L2 ") && !line.contains("L20"), "the correction must be G10 L2; got {line:?}");
  }

  #[test]
  fn the_correction_is_on_the_probed_axis() {
    // Verifying along X writes an X correction, not Y.
    let f = FlipResult::from_readings(Axis::X, &[20.0, 10.0]).expect("two readings");
    // error = (10 − 20)/2 = −5; current origin 30 → 25.
    assert_eq!(f.offer_g10(Wcs::Active, 30.0), "G10 L2 P0 X25.000");
  }

  #[test]
  fn fewer_or_more_than_two_readings_yields_no_result() {
    assert_eq!(FlipResult::from_readings(Axis::Y, &[1.0]), None);
    assert_eq!(FlipResult::from_readings(Axis::Y, &[1.0, 2.0, 3.0]), None);
    assert_eq!(FlipResult::from_readings(Axis::Y, &[]), None);
  }
}
