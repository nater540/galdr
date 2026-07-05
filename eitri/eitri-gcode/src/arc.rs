//! Bulge → arc-centre-offset conversion for `G2`/`G3` emission.
//!
//! Eitri keeps arcs in the CAD **bulge** encoding (`ArcPolyline`): a segment from vertex `P0` to `P1` carries a
//! bulge `b = tan(theta/4)`, where `theta` is the signed included angle (positive = counter-clockwise). grblHAL — and
//! the Skirnir/firmware contract — wants arcs as `G2`/`G3` with **I/J centre offsets relative to the arc start**
//! (`docs/eitri-gcode-skirnir-contract.md`, "Arcs"). This module is the pure, dependency-free conversion between the
//! two, so it is unit-tested against known circles without any G-code machinery.
//!
//! Derivation (no square-root sign ambiguity). With chord `d = P1 - P0`, chord length `c = |d|`, and midpoint `M`:
//! the arc sagitta is `s = b * c / 2` and the centre sits on the chord's left normal `n = (-dy, dx)/c` at the
//! apothem, giving `C = M + ((1 - b^2)/(4b)) * (-dy, dx)`. The `1/c` in `n` cancels the `c` in the apothem, so the
//! offset is exact and stable for minor, major, and reversed arcs alike. I/J are then `C - P0`.

/// The turning sense of an arc, which selects the G-code word: counter-clockwise is `G3`, clockwise is `G2`
/// (in the default G17 XY plane).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArcDir {
  /// Counter-clockwise — emitted as `G3`. Produced by a positive bulge.
  Ccw,
  /// Clockwise — emitted as `G2`. Produced by a negative bulge.
  Cw,
}

impl ArcDir {
  /// The grbl motion word for this direction (`"G2"` clockwise, `"G3"` counter-clockwise).
  pub fn word(self) -> &'static str {
    match self {
      ArcDir::Ccw => "G3",
      ArcDir::Cw => "G2",
    }
  }
}

/// The I/J centre offset of an arc, relative to its start point (grbl IJK form).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArcOffset {
  /// Centre X minus start X (millimetres).
  pub i: f64,
  /// Centre Y minus start Y (millimetres).
  pub j: f64,
}

/// Smallest bulge treated as a real arc; below this the segment is a straight line and no `G2`/`G3` is emitted.
const BULGE_EPSILON: f64 = 1.0e-12;

/// Convert a bulge segment from `(x0, y0)` to `(x1, y1)` into an arc centre offset and turning direction, or
/// `None` when the segment is effectively straight (`|bulge|` below [`BULGE_EPSILON`]) or degenerate (zero-length
/// chord). The returned I/J are relative to the start point `(x0, y0)`, which is exactly grbl's arc-centre convention.
pub fn bulge_to_arc(x0: f64, y0: f64, x1: f64, y1: f64, bulge: f64) -> Option<(ArcOffset, ArcDir)> {
  if bulge.abs() < BULGE_EPSILON {
    return None;
  }
  let dx = x1 - x0;
  let dy = y1 - y0;
  if dx.hypot(dy) < BULGE_EPSILON {
    return None;
  }
  // Centre = midpoint + ((1 - b^2)/(4b)) * left-normal(dx, dy). The left normal is (-dy, dx); its unnormalized form
  // is used because the apothem carries the reciprocal chord length that would otherwise normalize it.
  let k = (1.0 - bulge * bulge) / (4.0 * bulge);
  let cx = (x0 + x1) / 2.0 + k * (-dy);
  let cy = (y0 + y1) / 2.0 + k * dx;
  let dir = if bulge > 0.0 { ArcDir::Ccw } else { ArcDir::Cw };
  Some((ArcOffset { i: cx - x0, j: cy - y0 }, dir))
}

#[cfg(test)]
mod tests {
  use super::*;

  /// Bulge of a quarter circle: `tan(90deg / 4) = tan(22.5deg)`.
  fn quarter_bulge() -> f64 {
    (std::f64::consts::FRAC_PI_8).tan()
  }

  #[test]
  fn straight_segment_is_not_an_arc() {
    assert!(bulge_to_arc(0.0, 0.0, 10.0, 0.0, 0.0).is_none());
    assert!(bulge_to_arc(0.0, 0.0, 10.0, 0.0, 1e-15).is_none());
  }

  #[test]
  fn degenerate_zero_length_chord_is_rejected() {
    assert!(bulge_to_arc(3.0, 4.0, 3.0, 4.0, 0.5).is_none());
  }

  #[test]
  fn ccw_quarter_circle_centres_correctly() {
    // A +90deg CCW quarter arc from (1,0) to (0,1) is centred on the origin: I/J = (0,0) - (1,0) = (-1, 0).
    let (off, dir) = bulge_to_arc(1.0, 0.0, 0.0, 1.0, quarter_bulge()).expect("arc");
    assert_eq!(dir, ArcDir::Ccw);
    assert!((off.i - (-1.0)).abs() < 1e-9, "i = {}", off.i);
    assert!((off.j - 0.0).abs() < 1e-9, "j = {}", off.j);
  }

  #[test]
  fn cw_quarter_circle_flips_direction_and_centre() {
    // Reverse the traversal: (0,1) -> (1,0) is a -90deg CW quarter arc, still centred on the origin. From the new
    // start (0,1) the centre offset is (0,0) - (0,1) = (0, -1).
    let (off, dir) = bulge_to_arc(0.0, 1.0, 1.0, 0.0, -quarter_bulge()).expect("arc");
    assert_eq!(dir, ArcDir::Cw);
    assert!((off.i - 0.0).abs() < 1e-9, "i = {}", off.i);
    assert!((off.j - (-1.0)).abs() < 1e-9, "j = {}", off.j);
  }

  #[test]
  fn semicircle_centre_is_the_chord_midpoint() {
    // A half circle (theta = 180deg, b = tan(45deg) = 1) from (2,0) to (-2,0) is centred at the chord midpoint
    // (0,0), radius 2. I/J relative to the start (2,0) is (-2, 0).
    let (off, dir) = bulge_to_arc(2.0, 0.0, -2.0, 0.0, 1.0).expect("arc");
    assert_eq!(dir, ArcDir::Ccw);
    assert!((off.i - (-2.0)).abs() < 1e-9, "i = {}", off.i);
    assert!((off.j - 0.0).abs() < 1e-9, "j = {}", off.j);
  }

  #[test]
  fn radius_is_consistent_from_start_and_end() {
    // For any arc the centre must be equidistant from both endpoints (both lie on the circle).
    let (x0, y0, x1, y1, b) = (3.0, 1.0, 7.0, 4.0, 0.35);
    let (off, _) = bulge_to_arc(x0, y0, x1, y1, b).expect("arc");
    let cx = x0 + off.i;
    let cy = y0 + off.j;
    let r_start = (cx - x0).hypot(cy - y0);
    let r_end = (cx - x1).hypot(cy - y1);
    assert!((r_start - r_end).abs() < 1e-9, "radii differ: {r_start} vs {r_end}");
  }
}
