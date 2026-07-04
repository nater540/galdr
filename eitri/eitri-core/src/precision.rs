//! Shared tolerance, precision, and integer-snapping constants.
//!
//! There is exactly one home for these numbers so they never drift apart across crates. In particular
//! [`INTEGER_SCALE`] is the single factor every integer-coordinate backend (Clipper2 offsetting, i_overlay
//! booleans) must snap with — CAM code passes millimetres, `eitri-geo` multiplies by this one constant.

/// Default Douglas–Peucker simplification tolerance, in millimetres. Small enough to be visually lossless at
/// CAM scales, large enough to drop the redundant vertices that boolean/offset backends emit.
pub const SIMPLIFY_TOLERANCE_MM: f64 = 0.005;

/// Number of decimal places used when formatting coordinates into G-code (millimetres). One place per micron.
pub const GCODE_DECIMALS: usize = 4;

/// The single scale factor for integer-coordinate geometry backends: millimetres are multiplied by this and
/// rounded to snap onto the integer grid the backend operates on. 1e6 gives nanometre resolution, which keeps
/// coordinates well inside `i64` for any realistic board size (a 1 m span is 1e9, far below `i64::MAX`).
pub const INTEGER_SCALE: f64 = 1.0e6;

/// A general geometric epsilon (millimetres) for dedup and "are these points coincident" checks.
pub const GEOM_EPSILON_MM: f64 = 1.0e-6;

/// Round a coordinate to the G-code output precision ([`GCODE_DECIMALS`] places). Centralized so no crate reaches
/// for an ad-hoc `format!("{:.4}")` that could disagree with the configured precision.
pub fn round_gcode(value: f64) -> f64 {
  let factor = 10f64.powi(GCODE_DECIMALS as i32);
  (value * factor).round() / factor
}

/// Snap a millimetre value onto the shared integer grid used by integer backends.
pub fn snap_to_int(value_mm: f64) -> i64 {
  (value_mm * INTEGER_SCALE).round() as i64
}

/// Reverse [`snap_to_int`], mapping an integer-grid value back to millimetres.
pub fn unsnap_from_int(value: i64) -> f64 {
  value as f64 / INTEGER_SCALE
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn gcode_rounding_is_four_places() {
    assert_eq!(round_gcode(1.234_56), 1.2346);
    assert_eq!(round_gcode(-0.000_04), 0.0);
    assert_eq!(round_gcode(10.0), 10.0);
  }

  #[test]
  fn integer_snap_round_trips_within_grid_resolution() {
    let mm = 123.456_789;
    let restored = unsnap_from_int(snap_to_int(mm));
    // Round-trip error is bounded by the grid resolution (1 / INTEGER_SCALE).
    assert!((restored - mm).abs() <= 1.0 / INTEGER_SCALE);
  }

  #[test]
  fn large_board_coordinate_stays_in_i64() {
    // A 1 metre coordinate must not overflow the integer grid.
    let snapped = snap_to_int(1000.0);
    assert_eq!(snapped, 1_000_000_000);
    assert!(snapped < i64::MAX);
  }
}
