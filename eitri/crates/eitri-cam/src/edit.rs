//! Shared geometry editing ops — scale/offset/rotate/mirror/skew/buffer/simplify/join applied uniformly to any
//! object.
//!
//! Provenance: FlatCAM's `Geometry` base-class transform methods and geometry editor (see
//! `docs/eitri-porting-plan.md` §7.10). These are deliberately thin: the affine *math* is composed on
//! [`eitri_core::Affine`] (translate/scale_about/rotate_about/mirror_about_line/skew_about, chained with `then`) and
//! the boolean/offset *work* is the [`eitri_geo::GeoBackend`]. This module only exposes that surface uniformly so the
//! UI and scripting layers can transform a copper `MultiPolygon` (or a point set) generically, without each caller
//! re-deriving matrices or re-implementing the walk. Consolidation, not new capability.

use geo_types::MultiPolygon;

use eitri_core::{Affine, Result};
use eitri_geo::{GeoBackend, JoinType, apply_affine};

use crate::optimize::Point;

/// Apply an arbitrary affine `transform` to a copper/geometry object. Compose the transform on
/// [`Affine`] — e.g. `Affine::rotate_about(a, ox, oy).then(Affine::scale_about(s, s, ox, oy))`.
pub fn transform(source: &MultiPolygon<f64>, transform: Affine) -> MultiPolygon<f64> {
  apply_affine(source, transform)
}

/// Apply an affine `transform` to a set of points (drill hits, features).
pub fn transform_points(points: &[Point], transform: Affine) -> Vec<Point> {
  points
    .iter()
    .map(|&p| {
      let (x, y) = transform.apply(p.x, p.y);
      Point::new(x, y)
    })
    .collect()
}

/// Buffer (grow or shrink) every polygon of `source` by `distance` millimetres and union the results, so overlapping
/// buffers merge. Positive grows, negative shrinks — the collapse-safe offset handles insets that pinch off.
pub fn buffer<B>(
  source: &MultiPolygon<f64>,
  distance: f64,
  join: JoinType,
  miter_limit: f64,
  backend: &B,
) -> Result<MultiPolygon<f64>>
where
  B: GeoBackend,
{
  let mut buffered = Vec::new();
  for poly in &source.0 {
    buffered.extend(backend.offset(poly, distance, join, miter_limit)?.0);
  }
  backend.union_all(&buffered)
}

/// Simplify every polygon of `source`, dropping vertices within `tolerance` millimetres of the retained outline.
pub fn simplify<B>(source: &MultiPolygon<f64>, tolerance: f64, backend: &B) -> Result<MultiPolygon<f64>>
where
  B: GeoBackend,
{
  let mut out = Vec::with_capacity(source.0.len());
  for poly in &source.0 {
    out.push(backend.simplify(poly, tolerance)?);
  }
  Ok(MultiPolygon::new(out))
}

/// Join (union) every polygon of `source` into its merged outline.
pub fn join<B>(source: &MultiPolygon<f64>, backend: &B) -> Result<MultiPolygon<f64>>
where
  B: GeoBackend,
{
  backend.union_all(&source.0)
}

#[cfg(test)]
mod tests {
  use super::*;
  use eitri_geo::DefaultBackend;
  use geo::algorithm::area::Area;
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
  fn transform_composes_affines_uniformly() {
    // Rotate 90 CCW about the origin then translate by (5, 0): a point at (1, 0) lands at (5, 1).
    let source = MultiPolygon::new(vec![square(1.0, 0.0, 0.0001)]);
    let t = Affine::rotate(std::f64::consts::FRAC_PI_2).then(Affine::translate(5.0, 0.0));
    let moved = transform(&source, t);
    let c = moved.0[0].exterior().0[0];
    assert!((c.x - 5.0).abs() < 1e-3 && (c.y - 1.0).abs() < 1e-3, "transformed corner {c:?}");
  }

  #[test]
  fn transform_points_maps_every_point() {
    let pts = vec![Point::new(0.0, 0.0), Point::new(2.0, 0.0)];
    let moved = transform_points(&pts, Affine::translate(1.0, -1.0));
    assert_eq!(moved, vec![Point::new(1.0, -1.0), Point::new(3.0, -1.0)]);
  }

  #[test]
  fn buffer_grows_area_and_merges_overlaps() {
    // Two 2x2 squares 0.5mm apart, each grown by 1mm, merge into a single connected polygon.
    let source = MultiPolygon::new(vec![square(-1.5, 0.0, 1.0), square(1.5, 0.0, 1.0)]);
    let grown = buffer(&source, 1.0, JoinType::Round, 2.0, &DefaultBackend::new()).expect("buffer");
    assert_eq!(grown.0.len(), 1, "grown buffers overlap into one polygon");
    assert!(grown.unsigned_area() > source.unsigned_area(), "buffering outward grows area");
  }

  #[test]
  fn simplify_drops_a_collinear_vertex() {
    // A square with a redundant midpoint on the bottom edge; simplify removes it.
    let ring = LineString(vec![
      Coord { x: 0.0, y: 0.0 },
      Coord { x: 1.0, y: 0.0 },
      Coord { x: 2.0, y: 0.0 },
      Coord { x: 2.0, y: 2.0 },
      Coord { x: 0.0, y: 2.0 },
      Coord { x: 0.0, y: 0.0 },
    ]);
    let source = MultiPolygon::new(vec![Polygon::new(ring, vec![])]);
    let out = simplify(&source, 0.01, &DefaultBackend::new()).expect("simplify");
    assert!(out.0[0].exterior().0.len() < 6, "the collinear midpoint is dropped");
  }

  #[test]
  fn join_unions_overlapping_polygons() {
    let source = MultiPolygon::new(vec![square(0.0, 0.0, 2.0), square(1.0, 0.0, 2.0)]);
    let merged = join(&source, &DefaultBackend::new()).expect("join");
    assert_eq!(merged.0.len(), 1, "overlapping squares union into one polygon");
  }
}
