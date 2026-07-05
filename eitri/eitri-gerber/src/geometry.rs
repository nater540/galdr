//! Pure polygon builders for aperture shapes and the affine helpers macro primitives lean on.
//!
//! Curves are flattened to line segments at a fixed chord tolerance (curved apertures are approximated by
//! polygons, matching how Shapely/FlatCAM buffer output is consumed downstream). The circle builder and its
//! adaptive facet count live in `eitri-geo` and are re-exported here so the Gerber and Excellon front ends facet
//! circles identically. Nothing here touches a geometry backend — these are direct constructions; boolean
//! composition happens one layer up.

use std::f64::consts::TAU;

use eitri_core::Affine;
use geo_types::{Coord, LineString, MultiPolygon, Polygon};

// The circle builder and its adaptive facet count are shared across the parsers — re-exported from `eitri-geo` so
// existing `crate::geometry::{circle_polygon, arc_segment_count}` call sites keep resolving.
pub use eitri_geo::{arc_segment_count, circle_polygon};

/// An axis-aligned rectangle `width × height` centred at `(cx, cy)`.
pub fn rectangle_polygon(cx: f64, cy: f64, width: f64, height: f64) -> Polygon<f64> {
  let (hw, hh) = (width / 2.0, height / 2.0);
  let ring = vec![
    Coord { x: cx - hw, y: cy - hh },
    Coord { x: cx + hw, y: cy - hh },
    Coord { x: cx + hw, y: cy + hh },
    Coord { x: cx - hw, y: cy + hh },
  ];
  Polygon::new(LineString(ring), Vec::new())
}

/// A regular `sides`-gon of the given circumscribed `diameter`, centred at `(cx, cy)`, rotated `rotation_deg`
/// degrees counter-clockwise from the +x axis.
pub fn regular_polygon(cx: f64, cy: f64, diameter: f64, sides: u32, rotation_deg: f64) -> Polygon<f64> {
  let r = diameter / 2.0;
  let sides = sides.max(3);
  let base = rotation_deg.to_radians();
  let ring: Vec<Coord<f64>> = (0..sides)
    .map(|i| {
      let a = base + TAU * (i as f64) / (sides as f64);
      Coord { x: cx + r * a.cos(), y: cy + r * a.sin() }
    })
    .collect();
  Polygon::new(LineString(ring), Vec::new())
}

/// An obround (stadium): a rectangle capped by semicircles on its shorter dimension, centred at `(cx, cy)`.
pub fn obround_polygon(cx: f64, cy: f64, width: f64, height: f64) -> Polygon<f64> {
  // The caps sit on the ends of the longer axis; their radius is half the shorter dimension.
  if (width - height).abs() < f64::EPSILON {
    return circle_polygon(cx, cy, width / 2.0);
  }
  let horizontal = width >= height;
  let radius = if horizontal { height / 2.0 } else { width / 2.0 };
  let straight = if horizontal { width - height } else { height - width } / 2.0;
  let caps = arc_segment_count(radius, std::f64::consts::PI).max(4);

  let mut ring: Vec<Coord<f64>> = Vec::with_capacity(2 * caps + 2);
  if horizontal {
    // Right cap sweeps -90°..+90° about (cx+straight, cy); left cap +90°..+270° about (cx-straight, cy).
    push_arc(&mut ring, cx + straight, cy, radius, -std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2, caps);
    push_arc(&mut ring, cx - straight, cy, radius, std::f64::consts::FRAC_PI_2, 3.0 * std::f64::consts::FRAC_PI_2, caps);
  } else {
    // Top cap sweeps 0°..180° about (cx, cy+straight); bottom cap 180°..360° about (cx, cy-straight).
    push_arc(&mut ring, cx, cy + straight, radius, 0.0, std::f64::consts::PI, caps);
    push_arc(&mut ring, cx, cy - straight, radius, std::f64::consts::PI, TAU, caps);
  }
  Polygon::new(LineString(ring), Vec::new())
}

/// Append `count`+1 points sampling the arc from `start` to `end` radians on a circle of `radius` about `(cx, cy)`.
fn push_arc(ring: &mut Vec<Coord<f64>>, cx: f64, cy: f64, radius: f64, start: f64, end: f64, count: usize) {
  for i in 0..=count {
    let a = start + (end - start) * (i as f64) / (count as f64);
    ring.push(Coord { x: cx + radius * a.cos(), y: cy + radius * a.sin() });
  }
}

/// Apply an affine transform to every vertex of a polygon (exterior and holes).
pub fn transform_polygon(poly: &Polygon<f64>, t: &Affine) -> Polygon<f64> {
  let map_ring = |ls: &LineString<f64>| -> LineString<f64> {
    LineString(ls.0.iter().map(|c| { let (x, y) = t.apply(c.x, c.y); Coord { x, y } }).collect())
  };
  Polygon::new(map_ring(poly.exterior()), poly.interiors().iter().map(map_ring).collect())
}

/// Apply an affine transform to every polygon of a multipolygon.
pub fn transform_multipolygon(mp: &MultiPolygon<f64>, t: &Affine) -> MultiPolygon<f64> {
  MultiPolygon(mp.0.iter().map(|p| transform_polygon(p, t)).collect())
}

#[cfg(test)]
mod tests {
  use super::*;
  use geo::algorithm::area::Area;

  #[test]
  fn circle_area_approaches_pi_r_squared() {
    let c = circle_polygon(0.0, 0.0, 2.0);
    // An inscribed-polygon approximation sits slightly under the true area (~0.3% for this radius/tolerance).
    assert!((c.unsigned_area() - std::f64::consts::PI * 4.0).abs() < 0.05, "area {}", c.unsigned_area());
  }

  #[test]
  fn rectangle_area_is_exact() {
    assert!((rectangle_polygon(1.0, 1.0, 4.0, 2.0).unsigned_area() - 8.0).abs() < 1e-9);
  }

  #[test]
  fn obround_area_between_rect_and_bounding_box() {
    // A 4x2 obround = 2x2 rect + a circle of r=1: 4 + pi ≈ 7.14; bounded above by the 4x2 box (8).
    let ob = obround_polygon(0.0, 0.0, 4.0, 2.0);
    let a = ob.unsigned_area();
    assert!(a > 7.0 && a < 8.0, "obround area {a}");
  }

  #[test]
  fn regular_hexagon_has_expected_area() {
    // Hexagon with circumradius 1: area = 3*sqrt(3)/2 ≈ 2.598.
    let hex = regular_polygon(0.0, 0.0, 2.0, 6, 0.0);
    assert!((hex.unsigned_area() - 2.598).abs() < 1e-2, "hex area {}", hex.unsigned_area());
  }

  #[test]
  fn transform_translates_all_vertices() {
    let sq = rectangle_polygon(0.0, 0.0, 2.0, 2.0);
    let moved = transform_polygon(&sq, &Affine::translate(5.0, 3.0));
    let c0 = moved.exterior().0[0];
    assert!((c0.x - 4.0).abs() < 1e-9 && (c0.y - 2.0).abs() < 1e-9);
  }
}
