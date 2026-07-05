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
use geo::algorithm::line_intersection::line_intersection;
use geo_types::{Coord, Line, MultiLineString, MultiPolygon};

/// The axis-aligned bounds `(min_x, min_y, max_x, max_y)` of `mp`, or `None` if it is empty.
pub fn bounds(mp: &MultiPolygon<f64>) -> Option<(f64, f64, f64, f64)> {
  mp.bounding_rect().map(|r| (r.min().x, r.min().y, r.max().x, r.max().y))
}

/// Whether the point `(x, y)` lies strictly inside `region` (holes excluded, boundary not counted). Backs the
/// raster-fill connector test: a link between two fill spans is kept only when it stays inside the region.
pub fn contains_point(region: &MultiPolygon<f64>, x: f64, y: f64) -> bool {
  region.contains(&Coord { x, y })
}

/// Whether the straight connector from `a` to `b` stays within `region` and is safe to cut. Robust to holes and
/// concavities of *any* size: the segment is rejected if it *properly* crosses any ring edge — exterior or hole — of
/// the region, so even a hole narrower than the segment (which a fixed-sample point test would step over) is caught,
/// and additionally its midpoint must lie inside the region so a link skimming the mouth of a concavity without a
/// proper crossing is still rejected. Endpoints touching a ring (the span ends of a raster fill sit on the boundary)
/// are not proper crossings, so a legitimate in-material link is kept. Backs the raster boustrophedon linker: a link
/// that is not provably inside must lift and rapid rather than cut through empty space.
pub fn segment_within(region: &MultiPolygon<f64>, a: (f64, f64), b: (f64, f64)) -> bool {
  let connector = Line::new(Coord { x: a.0, y: a.1 }, Coord { x: b.0, y: b.1 });
  for poly in &region.0 {
    for ring in std::iter::once(poly.exterior()).chain(poly.interiors()) {
      for edge in ring.lines() {
        if let Some(hit) = line_intersection(connector, edge) {
          if hit.is_proper() {
            return false;
          }
        }
      }
    }
  }
  contains_point(region, (a.0 + b.0) * 0.5, (a.1 + b.1) * 0.5)
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
  fn segment_within_rejects_a_link_crossing_a_small_hole() {
    // A 10x10 square with a tiny hole; a connector passing through the hole must be rejected even though its length
    // dwarfs the hole (the failure mode a fixed-sample point test steps over), while a clear connector is kept.
    let outer = LineString(vec![
      Coord { x: 0.0, y: 0.0 }, Coord { x: 10.0, y: 0.0 },
      Coord { x: 10.0, y: 10.0 }, Coord { x: 0.0, y: 10.0 }, Coord { x: 0.0, y: 0.0 },
    ]);
    let hole = LineString(vec![
      Coord { x: 4.7, y: 1.6 }, Coord { x: 5.3, y: 1.6 },
      Coord { x: 5.3, y: 1.9 }, Coord { x: 4.7, y: 1.9 }, Coord { x: 4.7, y: 1.6 },
    ]);
    let region = MultiPolygon::new(vec![Polygon::new(outer, vec![hole])]);
    assert!(!segment_within(&region, (5.0, 1.0), (5.0, 3.0)), "a link through the hole must be rejected");
    assert!(segment_within(&region, (5.0, 5.0), (5.0, 8.0)), "a clear in-material link is kept");
    assert!(!segment_within(&region, (-1.0, 5.0), (11.0, 5.0)), "a link leaving the region is rejected");
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
