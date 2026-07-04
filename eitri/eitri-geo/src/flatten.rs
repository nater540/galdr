//! Flattening circular curves into straight-segment polygons at a shared chord tolerance.
//!
//! Curved apertures and drills are approximated by inscribed polygons. The facet count is chosen adaptively from
//! the radius and the sweep so the chord error never exceeds `eitri_core::CHORD_TOLERANCE_MM` — a tiny drill needs
//! only a few facets, a large pad needs many. Both the Gerber and Excellon front ends build circles through this
//! one routine so a 6 mm drill and a 6 mm flashed pad are faceted identically.

use std::f64::consts::TAU;

use geo_types::{Coord, LineString, Polygon};

/// Number of segments needed to approximate an arc of the given `radius` and absolute `sweep` (radians) within
/// [`eitri_core::CHORD_TOLERANCE_MM`]. Clamped to a sane range so tiny or huge radii stay well-behaved.
pub fn arc_segment_count(radius: f64, sweep_abs: f64) -> usize {
  if radius <= 0.0 || sweep_abs <= 0.0 {
    return 8;
  }
  let tol = eitri_core::CHORD_TOLERANCE_MM.min(radius * 0.5);
  let max_step = 2.0 * (1.0 - tol / radius).clamp(-1.0, 1.0).acos();
  if max_step <= f64::EPSILON {
    return 512;
  }
  ((sweep_abs / max_step).ceil() as usize).clamp(8, 512)
}

/// A filled circle of radius `r` centred at `(cx, cy)`, flattened to a polygon with an adaptive facet count.
pub fn circle_polygon(cx: f64, cy: f64, r: f64) -> Polygon<f64> {
  let n = arc_segment_count(r, TAU);
  let ring: Vec<Coord<f64>> = (0..n)
    .map(|i| {
      let a = TAU * (i as f64) / (n as f64);
      Coord { x: cx + r * a.cos(), y: cy + r * a.sin() }
    })
    .collect();
  Polygon::new(LineString(ring), Vec::new())
}

#[cfg(test)]
mod tests {
  use super::*;
  use geo::algorithm::area::Area;

  #[test]
  fn facet_count_scales_with_radius() {
    // A larger circle needs more facets to hold the same chord tolerance; a 3 mm radius clears the old fixed 48.
    let small = arc_segment_count(0.4, TAU);
    let large = arc_segment_count(3.0, TAU);
    assert!(large > small, "larger radius should need more facets ({large} vs {small})");
    assert!(large > 48, "a 3 mm radius should exceed the old hard-coded 48 facets, got {large}");
  }

  #[test]
  fn circle_area_approaches_pi_r_squared() {
    let c = circle_polygon(0.0, 0.0, 2.0);
    assert!((c.unsigned_area() - std::f64::consts::PI * 4.0).abs() < 0.05, "area {}", c.unsigned_area());
  }

  #[test]
  fn chord_error_within_tolerance() {
    // Verify the adaptive count actually holds the chord tolerance: sagitta r(1 - cos(pi/n)) <= tolerance.
    let r = 3.0;
    let n = arc_segment_count(r, TAU) as f64;
    let sagitta = r * (1.0 - (std::f64::consts::PI / n).cos());
    assert!(sagitta <= eitri_core::CHORD_TOLERANCE_MM + 1e-9, "sagitta {sagitta} exceeds tolerance");
  }
}
