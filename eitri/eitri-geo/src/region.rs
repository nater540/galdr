//! Region utilities: axis-aligned bounds and clipping open lines against a filled region.
//!
//! These back the CAM area operations (paint raster fill, non-copper framing, cutout, panelize) with two small,
//! backend-agnostic geometry services that do not belong on the offsetting/boolean [`crate::GeoBackend`] trait:
//! the bounding box of a region, and the portion of a set of scan lines that falls *inside* a region. Line
//! clipping routes through `geo`'s `BooleanOps::clip` (the same `i_overlay` engine the booleans use), so holes are
//! honoured — a scan line crossing a hole is cut into the spans on either side.

use geo::algorithm::bool_ops::BooleanOps;
use geo::algorithm::bounding_rect::BoundingRect;
use geo::algorithm::contains::Contains;
use geo_types::{Coord, MultiLineString, MultiPolygon};

/// The axis-aligned bounds `(min_x, min_y, max_x, max_y)` of `mp`, or `None` if it is empty.
pub fn bounds(mp: &MultiPolygon<f64>) -> Option<(f64, f64, f64, f64)> {
  mp.bounding_rect().map(|r| (r.min().x, r.min().y, r.max().x, r.max().y))
}

/// Whether the point `(x, y)` lies strictly inside `region` (holes excluded, boundary not counted). Backs the
/// raster-fill connector test: a link between two fill spans is kept only when it stays inside the region.
pub fn contains_point(region: &MultiPolygon<f64>, x: f64, y: f64) -> bool {
  region.contains(&Coord { x, y })
}

/// The portions of `lines` that lie inside `region`, with holes excluded. A line crossing a hole (or leaving the
/// region) is split into the inside spans on either side. This is the raster-fill primitive: intersect parallel
/// scan lines with the region to get the cut spans.
pub fn clip_lines(region: &MultiPolygon<f64>, lines: &MultiLineString<f64>) -> MultiLineString<f64> {
  region.clip(lines, false)
}

#[cfg(test)]
mod tests {
  use super::*;
  use geo_types::{Coord, LineString, Polygon};

  fn square(cx: f64, cy: f64, half: f64) -> Polygon<f64> {
    Polygon::new(
      LineString(vec![
        Coord { x: cx - half, y: cy - half },
        Coord { x: cx + half, y: cy - half },
        Coord { x: cx + half, y: cy + half },
        Coord { x: cx - half, y: cy + half },
        Coord { x: cx - half, y: cy - half },
      ]),
      vec![],
    )
  }

  #[test]
  fn bounds_span_the_extent() {
    let mp = MultiPolygon::new(vec![square(0.0, 0.0, 2.0), square(10.0, 5.0, 1.0)]);
    let (x0, y0, x1, y1) = bounds(&mp).expect("bounds");
    assert!((x0 + 2.0).abs() < 1e-9 && (y0 + 2.0).abs() < 1e-9);
    assert!((x1 - 11.0).abs() < 1e-9 && (y1 - 6.0).abs() < 1e-9);
  }

  #[test]
  fn empty_region_has_no_bounds() {
    assert!(bounds(&MultiPolygon::new(Vec::new())).is_none());
  }

  #[test]
  fn contains_point_respects_the_boundary_and_holes() {
    let outer = LineString(vec![
      Coord { x: -5.0, y: -5.0 },
      Coord { x: 5.0, y: -5.0 },
      Coord { x: 5.0, y: 5.0 },
      Coord { x: -5.0, y: 5.0 },
      Coord { x: -5.0, y: -5.0 },
    ]);
    let hole = LineString(vec![
      Coord { x: -2.0, y: -2.0 },
      Coord { x: 2.0, y: -2.0 },
      Coord { x: 2.0, y: 2.0 },
      Coord { x: -2.0, y: 2.0 },
      Coord { x: -2.0, y: -2.0 },
    ]);
    let region = MultiPolygon::new(vec![Polygon::new(outer, vec![hole])]);
    assert!(contains_point(&region, 3.5, 0.0), "a point in the ring body is inside");
    assert!(!contains_point(&region, 0.0, 0.0), "a point in the hole is outside");
    assert!(!contains_point(&region, 20.0, 0.0), "a point beyond the ring is outside");
  }

  #[test]
  fn a_scan_line_is_clipped_to_the_inside_span() {
    // A 10x10 square centred at origin; a horizontal line from x=-20..20 at y=0 clips to the [-5, 5] span.
    let region = MultiPolygon::new(vec![square(0.0, 0.0, 5.0)]);
    let line = MultiLineString::new(vec![LineString(vec![Coord { x: -20.0, y: 0.0 }, Coord { x: 20.0, y: 0.0 }])]);
    let clipped = clip_lines(&region, &line);
    assert_eq!(clipped.0.len(), 1, "one inside span");
    let xs: Vec<f64> = clipped.0[0].coords().map(|c| c.x).collect();
    let (min, max) = xs.iter().fold((f64::MAX, f64::MIN), |(a, b), &x| (a.min(x), b.max(x)));
    assert!((min + 5.0).abs() < 1e-6 && (max - 5.0).abs() < 1e-6, "span [{min}, {max}]");
  }

  #[test]
  fn a_scan_line_across_a_hole_is_split_into_two_spans() {
    // A 10x10 square with a 4x4 hole; a horizontal line through the middle clips to two spans, one per side.
    let outer = LineString(vec![
      Coord { x: -5.0, y: -5.0 },
      Coord { x: 5.0, y: -5.0 },
      Coord { x: 5.0, y: 5.0 },
      Coord { x: -5.0, y: 5.0 },
      Coord { x: -5.0, y: -5.0 },
    ]);
    let hole = LineString(vec![
      Coord { x: -2.0, y: -2.0 },
      Coord { x: 2.0, y: -2.0 },
      Coord { x: 2.0, y: 2.0 },
      Coord { x: -2.0, y: 2.0 },
      Coord { x: -2.0, y: -2.0 },
    ]);
    let region = MultiPolygon::new(vec![Polygon::new(outer, vec![hole])]);
    let line = MultiLineString::new(vec![LineString(vec![Coord { x: -20.0, y: 0.0 }, Coord { x: 20.0, y: 0.0 }])]);
    let clipped = clip_lines(&region, &line);
    assert_eq!(clipped.0.len(), 2, "the hole splits the scan line into two spans");
  }
}
