//! Arc-preserving polyline offsetting via `cavalier_contours`.
//!
//! Clipper2 offsets polygons but linearizes every curved join; `cavalier_contours` keeps true arc segments, which
//! means smoother toolpaths and smaller G-code once CAM output lands. Its model is open/closed polylines with a
//! per-vertex bulge (not multipolygons), so Eitri keeps the arc result in that native form — [`ArcPolyline`] — and
//! deliberately does *not* flatten it here. Flattening or arc-aware G-code emission is a later-phase concern.
//!
//! Sign convention is normalized to match the Clipper offset path: a **positive** distance grows the shape
//! outward, a negative distance shrinks it inward. Offsetting inward past a feature's size prunes cleanly to an
//! empty result (`cavalier_contours` handles the self-intersection collapse), so this never panics.

use cavalier_contours::polyline::{PlineSource, PlineSourceMut, Polyline};
use geo::algorithm::winding_order::Winding;
use geo_types::Polygon;

use eitri_core::Result;

/// One vertex of an arc-preserving polyline. `bulge` is `tan(theta/4)` of the arc swept from this vertex to the
/// next (zero for a straight segment) — the standard CAD bulge encoding `cavalier_contours` uses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArcVertex {
  /// X coordinate (millimetres).
  pub x: f64,
  /// Y coordinate (millimetres).
  pub y: f64,
  /// Bulge of the segment leaving this vertex; zero means a straight segment.
  pub bulge: f64,
}

/// A closed arc-preserving polyline produced by [`offset_arc`]. Retains bulge so downstream toolpath code can emit
/// `G02`/`G03` rather than a chain of tiny linear moves.
#[derive(Debug, Clone, PartialEq)]
pub struct ArcPolyline {
  /// The polyline vertices in order.
  pub vertices: Vec<ArcVertex>,
}

/// Offset a polygon's exterior boundary, preserving arcs, by `distance` millimetres (positive = outward). Returns
/// zero or more resulting polylines; an inward offset that collapses the shape returns an empty vector.
pub fn offset_arc(poly: &Polygon<f64>, distance: f64) -> Result<Vec<ArcPolyline>> {
  // Normalize the input ring to CCW so the offset sign is deterministic regardless of the caller's winding.
  let mut exterior = poly.exterior().clone();
  exterior.make_ccw_winding();

  let mut pline: Polyline<f64> = Polyline::new_closed();
  let coords = &exterior.0;
  // Skip the ring's redundant closing vertex; the polyline is already flagged closed.
  let count = if coords.len() > 1 && coords.first() == coords.last() { coords.len() - 1 } else { coords.len() };
  for coord in &coords[..count] {
    pline.add(coord.x, coord.y, 0.0);
  }

  // cavalier offsets a CCW polyline inward for a positive argument, so negate to make positive == outward here.
  let results = pline.parallel_offset(-distance);

  Ok(
    results
      .into_iter()
      .map(|pl| ArcPolyline {
        vertices: pl.iter_vertexes().map(|v| ArcVertex { x: v.x, y: v.y, bulge: v.bulge }).collect(),
      })
      .collect(),
  )
}

#[cfg(test)]
mod tests {
  use super::*;
  use geo_types::{Coord, LineString};

  fn square(side: f64) -> Polygon<f64> {
    let ring = LineString(vec![
      Coord { x: 0.0, y: 0.0 },
      Coord { x: side, y: 0.0 },
      Coord { x: side, y: side },
      Coord { x: 0.0, y: side },
      Coord { x: 0.0, y: 0.0 },
    ]);
    Polygon::new(ring, vec![])
  }

  fn bbox(pl: &ArcPolyline) -> (f64, f64, f64, f64) {
    pl.vertices.iter().fold((f64::MAX, f64::MAX, f64::MIN, f64::MIN), |(x0, y0, x1, y1), v| {
      (x0.min(v.x), y0.min(v.y), x1.max(v.x), y1.max(v.y))
    })
  }

  #[test]
  fn positive_distance_grows_outward() {
    let out = offset_arc(&square(10.0), 2.0).expect("offset");
    assert_eq!(out.len(), 1);
    let (x0, y0, x1, y1) = bbox(&out[0]);
    // Outward by 2 on a 10mm square: bounds expand to roughly [-2, 12] on each axis.
    assert!(x0 < -1.5 && y0 < -1.5 && x1 > 11.5 && y1 > 11.5, "expected outward growth, got {:?}", (x0, y0, x1, y1));
  }

  #[test]
  fn negative_distance_shrinks_inward() {
    let out = offset_arc(&square(10.0), -2.0).expect("offset");
    assert_eq!(out.len(), 1);
    let (x0, y0, x1, y1) = bbox(&out[0]);
    assert!(x0 > 1.5 && y0 > 1.5 && x1 < 8.5 && y1 < 8.5, "expected inward shrink, got {:?}", (x0, y0, x1, y1));
  }

  #[test]
  fn inward_past_feature_size_collapses_without_panicking() {
    // Shrinking a 10mm square inward by 6 removes it entirely; the arc backend prunes to nothing, no crash.
    let out = offset_arc(&square(10.0), -6.0).expect("offset");
    assert!(out.is_empty(), "expected empty collapse, got {} polylines", out.len());
  }
}
