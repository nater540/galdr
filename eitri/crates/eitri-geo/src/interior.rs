//! A robust, guaranteed-interior representative point for a ring.
//!
//! Even-odd containment (region assembly in the Gerber parser, hole-nesting in offset output) needs a point that is
//! *definitely inside* a ring, not merely near it. The arithmetic mean of a ring's vertices fails this for any
//! concave ring — for a square-with-a-hole frame the mean lands in the hole, for an L or U it lands in the notch —
//! and a misplaced probe silently corrupts the nesting. This routine defers to `geo`'s scanline interior point,
//! which is guaranteed to fall within a positive-area ring, so both call sites share one correct implementation.

use geo::algorithm::interior_point::InteriorPoint;
use geo_types::{Coord, LineString, Polygon};

/// A point guaranteed to lie strictly inside `ring`, treating it as a simple hole-free ring. Returns `None` for a
/// degenerate ring (empty or zero-area), which has no interior to probe.
pub fn ring_interior_point(ring: &LineString<f64>) -> Option<Coord<f64>> {
  // Wrap the ring as a hole-free polygon (`Polygon::new` closes it); `geo` scans a horizontal line across the
  // bounds and returns the midpoint of an interior span, which lies within the polygon by construction.
  let polygon = Polygon::new(ring.clone(), Vec::new());
  polygon.interior_point().map(|p| Coord { x: p.x(), y: p.y() })
}

#[cfg(test)]
mod tests {
  use super::*;
  use geo::algorithm::contains::Contains;

  /// The mean-of-vertices routine the fix replaces, kept here only to prove the concave cases where it fails.
  fn vertex_mean(ring: &LineString<f64>) -> Coord<f64> {
    let n = ring.0.len().max(1) as f64;
    let (sx, sy) = ring.0.iter().fold((0.0, 0.0), |(sx, sy), c| (sx + c.x, sy + c.y));
    Coord { x: sx / n, y: sy / n }
  }

  fn ring(points: &[(f64, f64)]) -> LineString<f64> {
    LineString(points.iter().map(|&(x, y)| Coord { x, y }).collect())
  }

  #[test]
  fn interior_point_is_inside_a_convex_ring() {
    let square = ring(&[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]);
    let p = ring_interior_point(&square).expect("interior point");
    assert!(Polygon::new(square, Vec::new()).contains(&p), "point {p:?} should be inside the square");
  }

  #[test]
  fn interior_point_is_inside_an_l_shape_where_the_mean_is_outside() {
    // An L (a square with the top-right quadrant removed). Its vertex mean lands in the removed notch — outside the
    // ring — which is exactly the failure that corrupts containment. The robust point must stay inside.
    let l = ring(&[(0.0, 0.0), (6.0, 0.0), (6.0, 3.0), (3.0, 3.0), (3.0, 6.0), (0.0, 6.0)]);
    let poly = Polygon::new(l.clone(), Vec::new());

    let mean = vertex_mean(&l);
    assert!(!poly.contains(&mean), "precondition: vertex mean {mean:?} must fall outside the L");

    let robust = ring_interior_point(&l).expect("interior point");
    assert!(poly.contains(&robust), "robust point {robust:?} must fall inside the L");
  }

  #[test]
  fn interior_point_is_inside_a_u_shape_where_the_mean_is_outside() {
    // A U (a 10x10 square with a top-middle notch removed). Its vertex mean lands in the notch, outside the ring;
    // the robust point must land in the U's body so even-odd depth counting sees it enclosed by the right rings.
    let u = ring(&[
      (0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (7.0, 10.0), (7.0, 5.0), (3.0, 5.0), (3.0, 10.0), (0.0, 10.0),
    ]);
    let poly = Polygon::new(u.clone(), Vec::new());

    let mean = vertex_mean(&u);
    assert!(!poly.contains(&mean), "precondition: vertex mean {mean:?} must fall in the notch, outside the U");

    let robust = ring_interior_point(&u).expect("interior point");
    assert!(poly.contains(&robust), "robust point {robust:?} must fall inside the U");
  }

  #[test]
  fn degenerate_ring_has_no_interior_point() {
    let empty = LineString::<f64>(Vec::new());
    assert!(ring_interior_point(&empty).is_none());
  }
}
