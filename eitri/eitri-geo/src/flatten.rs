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

/// Maximum recursion depth for adaptive Bézier subdivision. A cap guards against a pathological curve subdividing
/// forever; at CAM scales the chord tolerance is reached long before this, so it is a safety net, not a limit.
const BEZIER_MAX_DEPTH: u32 = 24;

/// Flatten a circular arc of radius `r` centred at `(cx, cy)`, sweeping `sweep` radians (signed: counter-clockwise
/// positive) from `start_angle`. Returns the points *after* the start up to and including the end, so appending them
/// to a path already sitting at the arc start continues it without duplicating a vertex. Facet count holds the chord
/// error within [`eitri_core::CHORD_TOLERANCE_MM`] via [`arc_segment_count`].
pub fn flatten_arc(cx: f64, cy: f64, r: f64, start_angle: f64, sweep: f64) -> Vec<Coord<f64>> {
  if r <= 0.0 || sweep.abs() <= f64::EPSILON {
    // A degenerate radius or zero sweep collapses to the single endpoint the caller expects to arrive at.
    let a = start_angle + sweep;
    return vec![Coord { x: cx + r * a.cos(), y: cy + r * a.sin() }];
  }
  let n = arc_segment_count(r, sweep.abs());
  let mut pts = Vec::with_capacity(n);
  for i in 1..=n {
    let a = start_angle + sweep * (i as f64) / (n as f64);
    pts.push(Coord { x: cx + r * a.cos(), y: cy + r * a.sin() });
  }
  pts
}

/// Flatten a CAD **bulge** segment from `(x0, y0)` to `(x1, y1)` — where `bulge = tan(theta/4)` and `theta` is the
/// signed included angle (positive = counter-clockwise) — into arc chord points *after* the start up to and including
/// the end. A near-zero bulge is a straight segment and yields just the endpoint. Used by DXF `LWPOLYLINE`/`POLYLINE`
/// import, which carries arcs in this per-vertex bulge form.
pub fn flatten_bulge(x0: f64, y0: f64, x1: f64, y1: f64, bulge: f64) -> Vec<Coord<f64>> {
  let dx = x1 - x0;
  let dy = y1 - y0;
  if bulge.abs() < 1.0e-12 || dx.hypot(dy) < 1.0e-12 {
    return vec![Coord { x: x1, y: y1 }];
  }
  // Centre = midpoint + ((1 - b^2)/(4b)) * left-normal(dx, dy); the left normal is (-dy, dx). This is the same
  // stable, sqrt-free construction the G-code arc emitter uses, so bulge → arc is consistent in both directions.
  let k = (1.0 - bulge * bulge) / (4.0 * bulge);
  let cx = (x0 + x1) / 2.0 + k * (-dy);
  let cy = (y0 + y1) / 2.0 + k * dx;
  let r = (cx - x0).hypot(cy - y0);
  let start_angle = (y0 - cy).atan2(x0 - cx);
  let sweep = 4.0 * bulge.atan();
  flatten_arc(cx, cy, r, start_angle, sweep)
}

/// Flatten a cubic Bézier from `p0` through control points `c1`, `c2` to `p1` into line segments whose chord
/// deviation stays within [`eitri_core::CHORD_TOLERANCE_MM`]. Returns the points *after* `p0` up to and including
/// `p1`, so appending them to a path already ending at `p0` continues it without duplicating the start. Flatten in
/// the final (millimetre) coordinate space — since affine maps commute with subdivision, transform control points
/// first, then flatten here, so the tolerance is measured where it matters.
pub fn flatten_cubic(p0: Coord<f64>, c1: Coord<f64>, c2: Coord<f64>, p1: Coord<f64>) -> Vec<Coord<f64>> {
  let mut out = Vec::new();
  subdivide_cubic(p0, c1, c2, p1, 0, &mut out);
  out.push(p1);
  out
}

/// Flatten a quadratic Bézier from `p0` through control point `c` to `p1`. Returns the points *after* `p0` up to and
/// including `p1`, matching [`flatten_cubic`]'s contract.
pub fn flatten_quad(p0: Coord<f64>, c: Coord<f64>, p1: Coord<f64>) -> Vec<Coord<f64>> {
  // Elevate the quadratic to an equivalent cubic (c1 = p0 + 2/3 (c - p0), c2 = p1 + 2/3 (c - p1)) and reuse the one
  // subdivision routine, so both Bézier orders share exactly one flatness test.
  let c1 = Coord { x: p0.x + 2.0 / 3.0 * (c.x - p0.x), y: p0.y + 2.0 / 3.0 * (c.y - p0.y) };
  let c2 = Coord { x: p1.x + 2.0 / 3.0 * (c.x - p1.x), y: p1.y + 2.0 / 3.0 * (c.y - p1.y) };
  flatten_cubic(p0, c1, c2, p1)
}

/// Recursively subdivide a cubic Bézier, pushing interior points (never the endpoints) once the segment is flat
/// enough. The caller appends the final endpoint, so this pushes only the points strictly between `p0` and `p1`.
fn subdivide_cubic(p0: Coord<f64>, c1: Coord<f64>, c2: Coord<f64>, p1: Coord<f64>, depth: u32, out: &mut Vec<Coord<f64>>) {
  if depth >= BEZIER_MAX_DEPTH || cubic_is_flat(p0, c1, c2, p1) {
    return;
  }
  // de Casteljau split at t = 0.5 into two sub-curves sharing the midpoint `m`.
  let mid = |a: Coord<f64>, b: Coord<f64>| Coord { x: (a.x + b.x) / 2.0, y: (a.y + b.y) / 2.0 };
  let p01 = mid(p0, c1);
  let p12 = mid(c1, c2);
  let p23 = mid(c2, p1);
  let p012 = mid(p01, p12);
  let p123 = mid(p12, p23);
  let m = mid(p012, p123);
  subdivide_cubic(p0, p01, p012, m, depth + 1, out);
  out.push(m);
  subdivide_cubic(m, p123, p23, p1, depth + 1, out);
}

/// A cubic is flat when both control points lie within the chord tolerance of the chord `p0`→`p1`. Uses the squared
/// perpendicular distance so no square root is taken per test.
fn cubic_is_flat(p0: Coord<f64>, c1: Coord<f64>, c2: Coord<f64>, p1: Coord<f64>) -> bool {
  let tol = eitri_core::CHORD_TOLERANCE_MM;
  let dx = p1.x - p0.x;
  let dy = p1.y - p0.y;
  let chord_sq = dx * dx + dy * dy;
  if chord_sq <= f64::EPSILON {
    // Degenerate chord (p0 == p1): fall back to absolute control-point spread against the tolerance.
    let d1 = (c1.x - p0.x).hypot(c1.y - p0.y);
    let d2 = (c2.x - p0.x).hypot(c2.y - p0.y);
    return d1 <= tol && d2 <= tol;
  }
  // Perpendicular distance of a control point c from the chord: |(c - p0) x (p1 - p0)| / |p1 - p0|. Compare squared.
  let cross1 = (c1.x - p0.x) * dy - (c1.y - p0.y) * dx;
  let cross2 = (c2.x - p0.x) * dy - (c2.y - p0.y) * dx;
  let tol_sq = tol * tol * chord_sq;
  cross1 * cross1 <= tol_sq && cross2 * cross2 <= tol_sq
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

  /// Every returned point of an arc/bulge flatten must sit on the circle of radius `r` centred at `(cx, cy)`.
  fn assert_on_circle(pts: &[Coord<f64>], cx: f64, cy: f64, r: f64) {
    for p in pts {
      let rr = (p.x - cx).hypot(p.y - cy);
      assert!((rr - r).abs() < 1e-9, "point {p:?} off circle: radius {rr} vs {r}");
    }
  }

  #[test]
  fn flatten_arc_returns_end_and_holds_the_circle() {
    // A quarter arc (CCW) on the unit circle from angle 0 to pi/2 ends exactly at (0, 1).
    let pts = flatten_arc(0.0, 0.0, 1.0, 0.0, std::f64::consts::FRAC_PI_2);
    assert!(pts.len() >= 8, "should facet a quarter arc into several chords, got {}", pts.len());
    let end = pts.last().copied().expect("non-empty");
    assert!((end.x - 0.0).abs() < 1e-9 && (end.y - 1.0).abs() < 1e-9, "arc end {end:?}");
    assert_on_circle(&pts, 0.0, 0.0, 1.0);
  }

  #[test]
  fn flatten_arc_chord_sagitta_within_tolerance() {
    // The sagitta between successive chord points must never exceed the shared chord tolerance.
    let (cx, cy, r) = (2.0, -1.0, 5.0);
    let pts = flatten_arc(cx, cy, r, 0.3, std::f64::consts::PI);
    let mut prev = Coord { x: cx + r * 0.3_f64.cos(), y: cy + r * 0.3_f64.sin() };
    for p in &pts {
      let chord = (p.x - prev.x).hypot(p.y - prev.y);
      let sagitta = r - (r * r - (chord / 2.0).powi(2)).max(0.0).sqrt();
      assert!(sagitta <= eitri_core::CHORD_TOLERANCE_MM + 1e-9, "sagitta {sagitta} exceeds tolerance");
      prev = *p;
    }
  }

  #[test]
  fn flatten_bulge_quarter_circle_matches_arc() {
    // A +90deg CCW quarter bulge from (1,0) to (0,1) is centred on the origin, radius 1; every point on the circle.
    let b = (std::f64::consts::FRAC_PI_8).tan();
    let pts = flatten_bulge(1.0, 0.0, 0.0, 1.0, b);
    let end = pts.last().copied().expect("non-empty");
    assert!((end.x - 0.0).abs() < 1e-9 && (end.y - 1.0).abs() < 1e-9, "bulge end {end:?}");
    assert_on_circle(&pts, 0.0, 0.0, 1.0);
  }

  #[test]
  fn flatten_bulge_zero_is_a_straight_segment() {
    let pts = flatten_bulge(0.0, 0.0, 10.0, 3.0, 0.0);
    assert_eq!(pts, vec![Coord { x: 10.0, y: 3.0 }]);
  }

  #[test]
  fn flatten_cubic_straight_line_is_just_the_endpoint() {
    // Control points on the chord => already flat => only the endpoint is emitted.
    let pts = flatten_cubic(
      Coord { x: 0.0, y: 0.0 },
      Coord { x: 3.0, y: 0.0 },
      Coord { x: 6.0, y: 0.0 },
      Coord { x: 9.0, y: 0.0 },
    );
    assert_eq!(pts, vec![Coord { x: 9.0, y: 0.0 }]);
  }

  /// Perpendicular distance from point `p` to the segment `a`→`b` (clamped to the segment, not the infinite line).
  fn dist_to_segment(p: Coord<f64>, a: Coord<f64>, b: Coord<f64>) -> f64 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let len_sq = dx * dx + dy * dy;
    if len_sq <= f64::EPSILON {
      return (p.x - a.x).hypot(p.y - a.y);
    }
    let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / len_sq).clamp(0.0, 1.0);
    (p.x - (a.x + t * dx)).hypot(p.y - (a.y + t * dy))
  }

  #[test]
  fn flatten_cubic_curve_subdivides_and_stays_within_tolerance() {
    // A symmetric bump cubic. The proper chord-error metric is how far the *true curve* strays from the flattened
    // polyline: sample the analytic Bézier densely and take each sample's nearest distance to the polyline chords.
    let p0 = Coord { x: 0.0, y: 0.0 };
    let c1 = Coord { x: 0.0, y: 4.0 };
    let c2 = Coord { x: 10.0, y: 4.0 };
    let p1 = Coord { x: 10.0, y: 0.0 };
    let pts = flatten_cubic(p0, c1, c2, p1);
    assert!(pts.len() > 4, "a curved cubic should subdivide, got {}", pts.len());
    assert_eq!(pts.last().copied(), Some(p1), "must end at p1");
    let poly: Vec<Coord<f64>> = std::iter::once(p0).chain(pts.iter().copied()).collect();
    let bez = |t: f64| {
      let u = 1.0 - t;
      Coord {
        x: u * u * u * p0.x + 3.0 * u * u * t * c1.x + 3.0 * u * t * t * c2.x + t * t * t * p1.x,
        y: u * u * u * p0.y + 3.0 * u * u * t * c1.y + 3.0 * u * t * t * c2.y + t * t * t * p1.y,
      }
    };
    for k in 0..=2000 {
      let s = bez(k as f64 / 2000.0);
      let mut best = f64::MAX;
      for w in poly.windows(2) {
        best = best.min(dist_to_segment(s, w[0], w[1]));
      }
      assert!(best <= eitri_core::CHORD_TOLERANCE_MM + 1e-6, "curve sample {s:?} is {best} off the polyline");
    }
  }

  #[test]
  fn flatten_quad_matches_its_cubic_elevation_endpoint() {
    let p0 = Coord { x: 0.0, y: 0.0 };
    let c = Coord { x: 5.0, y: 6.0 };
    let p1 = Coord { x: 10.0, y: 0.0 };
    let pts = flatten_quad(p0, c, p1);
    assert!(pts.len() > 2, "a curved quad should subdivide");
    assert_eq!(pts.last().copied(), Some(p1));
  }
}
