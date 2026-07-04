//! Acceptance tests for the default geometry backend: offsetting at known distances, boolean union/difference/
//! intersection with asserted areas and ring structure, winding normalization, simplification, and graceful
//! collapse. These exercise the [`GeoBackend`] trait surface exactly as CAM code will.

use eitri_geo::{DefaultBackend, GeoBackend, JoinType, WindingDirection};

use geo::algorithm::area::Area;
use geo::algorithm::winding_order::{Winding, WindingOrder};
use geo_types::{Coord, LineString, MultiPolygon, Polygon};

/// Millimetre area tolerance; Clipper2 integer snapping at 1e6 introduces only sub-micron error.
const AREA_TOL: f64 = 1.0e-3;

/// Axis-aligned rectangle `[x0, x1] × [y0, y1]` as a CCW polygon.
fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon<f64> {
  Polygon::new(
    LineString(vec![
      Coord { x: x0, y: y0 },
      Coord { x: x1, y: y0 },
      Coord { x: x1, y: y1 },
      Coord { x: x0, y: y1 },
      Coord { x: x0, y: y0 },
    ]),
    vec![],
  )
}

fn square(side: f64) -> Polygon<f64> {
  rect(0.0, 0.0, side, side)
}

#[test]
fn offset_outward_grows_by_known_area() {
  let backend = DefaultBackend::new();
  // A 10×10 square offset outward by 2 with mitred corners is a 14×14 square: area 196.
  let out = backend.offset(&square(10.0), 2.0, JoinType::Miter, 2.0).expect("offset");
  assert_eq!(out.0.len(), 1);
  assert!((out.unsigned_area() - 196.0).abs() < AREA_TOL, "area was {}", out.unsigned_area());
}

#[test]
fn offset_inward_shrinks_by_known_area() {
  let backend = DefaultBackend::new();
  // Inward by 2 gives a 6×6 square: area 36.
  let out = backend.offset(&square(10.0), -2.0, JoinType::Miter, 2.0).expect("offset");
  assert_eq!(out.0.len(), 1);
  assert!((out.unsigned_area() - 36.0).abs() < AREA_TOL, "area was {}", out.unsigned_area());
}

#[test]
fn offset_inward_past_feature_size_collapses_cleanly() {
  let backend = DefaultBackend::new();
  // Shrinking a 10×10 square inward by 6 removes it entirely: an empty MultiPolygon, no panic, no degenerate ring.
  let out = backend.offset(&square(10.0), -6.0, JoinType::Miter, 2.0).expect("offset must not panic on collapse");
  assert!(out.0.is_empty(), "expected empty collapse, got {} polygons", out.0.len());
}

#[test]
fn offset_preserves_and_shrinks_a_hole() {
  let backend = DefaultBackend::new();
  // A 10×10 square with a 4×4 hole, offset outward by 1: exterior grows to 12×12 (144), the hole shrinks to
  // 2×2 (4). Net area 140, still one polygon carrying exactly one interior ring — verifies hole reconstruction.
  let with_hole = Polygon::new(
    square(10.0).exterior().clone(),
    vec![LineString(vec![
      Coord { x: 3.0, y: 3.0 },
      Coord { x: 7.0, y: 3.0 },
      Coord { x: 7.0, y: 7.0 },
      Coord { x: 3.0, y: 7.0 },
      Coord { x: 3.0, y: 3.0 },
    ])],
  );
  let out = backend.offset(&with_hole, 1.0, JoinType::Miter, 2.0).expect("offset");
  assert_eq!(out.0.len(), 1, "expected a single polygon");
  assert_eq!(out.0[0].interiors().len(), 1, "expected the hole to survive as one interior ring");
  assert!((out.unsigned_area() - 140.0).abs() < AREA_TOL, "area was {}", out.unsigned_area());
}

#[test]
fn union_all_merges_overlapping_squares() {
  let backend = DefaultBackend::new();
  // Two 10×10 squares overlapping in a 5×5 corner: 100 + 100 - 25 = 175.
  let a = rect(0.0, 0.0, 10.0, 10.0);
  let b = rect(5.0, 5.0, 15.0, 15.0);
  let merged = backend.union_all(&[a, b]).expect("union");
  assert_eq!(merged.0.len(), 1, "the two squares overlap, so they merge into one polygon");
  assert!((merged.unsigned_area() - 175.0).abs() < AREA_TOL, "area was {}", merged.unsigned_area());
}

#[test]
fn union_all_of_nothing_is_empty() {
  let backend = DefaultBackend::new();
  let merged = backend.union_all(&[]).expect("empty union");
  assert!(merged.0.is_empty());
}

#[test]
fn difference_cuts_a_hole() {
  let backend = DefaultBackend::new();
  // Big square minus a fully-interior small square leaves a square with a hole: area 100 - 16 = 84, one interior.
  let big = MultiPolygon::new(vec![rect(0.0, 0.0, 10.0, 10.0)]);
  let small = MultiPolygon::new(vec![rect(3.0, 3.0, 7.0, 7.0)]);
  let diff = backend.difference(&big, &small).expect("difference");
  assert_eq!(diff.0.len(), 1);
  assert_eq!(diff.0[0].interiors().len(), 1, "the removed interior square becomes a hole");
  assert!((diff.unsigned_area() - 84.0).abs() < AREA_TOL, "area was {}", diff.unsigned_area());
}

#[test]
fn intersection_keeps_only_the_overlap() {
  let backend = DefaultBackend::new();
  // [0,10]² ∩ [5,15]² = [5,10]²: area 25.
  let a = MultiPolygon::new(vec![rect(0.0, 0.0, 10.0, 10.0)]);
  let b = MultiPolygon::new(vec![rect(5.0, 5.0, 15.0, 15.0)]);
  let overlap = backend.intersection(&a, &b).expect("intersection");
  assert!((overlap.unsigned_area() - 25.0).abs() < AREA_TOL, "area was {}", overlap.unsigned_area());
}

#[test]
fn winding_normalization_is_deterministic() {
  let backend = DefaultBackend::new();
  // A clockwise exterior ring.
  let cw = Polygon::new(
    LineString(vec![
      Coord { x: 0.0, y: 0.0 },
      Coord { x: 0.0, y: 10.0 },
      Coord { x: 10.0, y: 10.0 },
      Coord { x: 10.0, y: 0.0 },
      Coord { x: 0.0, y: 0.0 },
    ]),
    vec![],
  );
  assert_eq!(cw.exterior().winding_order(), Some(WindingOrder::Clockwise));

  let to_ccw = backend.normalize_winding(&cw, WindingDirection::Ccw);
  assert_eq!(to_ccw.exterior().winding_order(), Some(WindingOrder::CounterClockwise));

  // Normalizing the now-CCW polygon back to CW flips it deterministically.
  let to_cw = backend.normalize_winding(&to_ccw, WindingDirection::Cw);
  assert_eq!(to_cw.exterior().winding_order(), Some(WindingOrder::Clockwise));
}

#[test]
fn winding_normalization_winds_holes_opposite_to_exterior() {
  let backend = DefaultBackend::new();
  let poly = Polygon::new(
    square(10.0).exterior().clone(),
    vec![LineString(vec![
      Coord { x: 3.0, y: 3.0 },
      Coord { x: 5.0, y: 3.0 },
      Coord { x: 5.0, y: 5.0 },
      Coord { x: 3.0, y: 5.0 },
      Coord { x: 3.0, y: 3.0 },
    ])],
  );
  let normalized = backend.normalize_winding(&poly, WindingDirection::Ccw);
  assert_eq!(normalized.exterior().winding_order(), Some(WindingOrder::CounterClockwise));
  assert_eq!(normalized.interiors()[0].winding_order(), Some(WindingOrder::Clockwise));
}

#[test]
fn simplify_drops_a_redundant_collinear_vertex() {
  let backend = DefaultBackend::new();
  // The midpoint (5,0) lies exactly on the bottom edge and should be removed by Douglas–Peucker.
  let redundant = Polygon::new(
    LineString(vec![
      Coord { x: 0.0, y: 0.0 },
      Coord { x: 5.0, y: 0.0 },
      Coord { x: 10.0, y: 0.0 },
      Coord { x: 10.0, y: 10.0 },
      Coord { x: 0.0, y: 10.0 },
      Coord { x: 0.0, y: 0.0 },
    ]),
    vec![],
  );
  let simplified = backend.simplify(&redundant, 0.01).expect("simplify");
  assert!(
    simplified.exterior().0.len() < redundant.exterior().0.len(),
    "expected fewer vertices, got {}",
    simplified.exterior().0.len()
  );
  // Area is unchanged by removing a collinear point.
  assert!((simplified.unsigned_area() - 100.0).abs() < AREA_TOL);
}
