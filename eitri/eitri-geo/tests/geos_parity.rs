//! Parity cross-check: the feature-gated GEOS (Shapely-oracle) backend vs the shipping `DefaultBackend`.
//!
//! This is the whole reason the GEOS backend exists (`docs/eitri-porting-plan.md` §2, §12): because FlatCAM leaned
//! on Shapely's `buffer`/boolean semantics, GEOS reproduces them at maximum fidelity, so measuring GEOS-vs-Default
//! here *documents how close* the shipping Clipper2/geo path lands to Shapely. The two are not bit-identical — the
//! offset engines differ (Clipper2 integer space + its own round-join tessellation vs GEOS's 8-quadrant curve) — so
//! these assert an area/shape tolerance, and the measured divergence is the deliverable of this file.
//!
//! Entirely compiled out unless `--features geos` is enabled (and a system GEOS is present).
#![cfg(feature = "geos")]

use geo::algorithm::area::Area;
use geo_types::{Coord, LineString, MultiPolygon, Polygon};

use eitri_geo::{DefaultBackend, GeoBackend, GeosBackend, JoinType};

/// Relative-area agreement tolerance between the two offset backends. Round-join offsetting tessellates corner arcs
/// differently (Clipper2 vs GEOS 8-quadrant), so a fraction of a percent of divergence on rounded corners is
/// expected and acceptable; the shapes agree far tighter than this on their straight portions.
const OFFSET_REL_TOL: f64 = 0.01;

/// Boolean ops (union/difference/intersection) go through fundamentally different engines — GEOS overlay vs geo's
/// `i_overlay` — yet on non-degenerate polygonal input they should agree to near floating precision, because the
/// result is an exact polygon set, not a tessellated curve. A tight bound documents that they truly match.
const BOOLEAN_REL_TOL: f64 = 1e-6;

/// A closed counter-clockwise square `[origin, origin+side]²`.
fn square(origin: f64, side: f64) -> Polygon<f64> {
  let (a, b) = (origin, origin + side);
  Polygon::new(
    LineString(vec![
      Coord { x: a, y: a },
      Coord { x: b, y: a },
      Coord { x: b, y: b },
      Coord { x: a, y: b },
      Coord { x: a, y: a },
    ]),
    Vec::new(),
  )
}

/// Assert two areas agree within a relative tolerance, reporting the measured relative divergence on failure.
fn assert_rel_area(label: &str, geos_area: f64, default_area: f64, tol: f64) {
  let denom = default_area.abs().max(1e-9);
  let rel = (geos_area - default_area).abs() / denom;
  assert!(
    rel <= tol,
    "{label}: geos={geos_area:.9} default={default_area:.9} rel_divergence={rel:.3e} exceeds tol {tol:.3e}"
  );
}

#[test]
fn offset_outward_agrees_within_tolerance() {
  let geos = GeosBackend::new();
  let def = DefaultBackend::new();
  let poly = square(0.0, 10.0);

  let g = geos.offset(&poly, 1.5, JoinType::Round, 5.0).expect("geos offset");
  let d = def.offset(&poly, 1.5, JoinType::Round, 5.0).expect("default offset");
  assert_rel_area("outward round offset", g.unsigned_area(), d.unsigned_area(), OFFSET_REL_TOL);
}

#[test]
fn offset_inward_agrees_within_tolerance() {
  let geos = GeosBackend::new();
  let def = DefaultBackend::new();
  let poly = square(0.0, 10.0);

  // Inward offset of a convex square has straight edges (no corner rounding), so the two backends should agree to
  // near floating precision here — a much tighter bound than the outward, rounded case.
  let g = geos.offset(&poly, -1.5, JoinType::Round, 5.0).expect("geos offset");
  let d = def.offset(&poly, -1.5, JoinType::Round, 5.0).expect("default offset");
  assert_rel_area("inward offset", g.unsigned_area(), d.unsigned_area(), BOOLEAN_REL_TOL);
}

#[test]
fn union_agrees_within_tolerance() {
  let geos = GeosBackend::new();
  let def = DefaultBackend::new();
  // Two squares whose corners overlap on a 4x4 patch: 100 + 100 - 16 = 184.
  let polys = [square(0.0, 10.0), square(6.0, 10.0)];

  let g = geos.union_all(&polys).expect("geos union");
  let d = def.union_all(&polys).expect("default union");
  assert!((g.unsigned_area() - 184.0).abs() < 1e-6, "union area sanity: {}", g.unsigned_area());
  assert_rel_area("union", g.unsigned_area(), d.unsigned_area(), BOOLEAN_REL_TOL);
}

#[test]
fn difference_agrees_within_tolerance() {
  let geos = GeosBackend::new();
  let def = DefaultBackend::new();
  let a = MultiPolygon(vec![square(0.0, 10.0)]);
  let b = MultiPolygon(vec![square(3.0, 4.0)]);

  let g = geos.difference(&a, &b).expect("geos difference");
  let d = def.difference(&a, &b).expect("default difference");
  assert!((g.unsigned_area() - 84.0).abs() < 1e-6, "difference area sanity: {}", g.unsigned_area());
  assert_rel_area("difference", g.unsigned_area(), d.unsigned_area(), BOOLEAN_REL_TOL);
}

#[test]
fn intersection_agrees_within_tolerance() {
  let geos = GeosBackend::new();
  let def = DefaultBackend::new();
  let a = MultiPolygon(vec![square(0.0, 10.0)]);
  let b = MultiPolygon(vec![square(6.0, 10.0)]);

  let g = geos.intersection(&a, &b).expect("geos intersection");
  let d = def.intersection(&a, &b).expect("default intersection");
  assert!((g.unsigned_area() - 16.0).abs() < 1e-6, "intersection area sanity: {}", g.unsigned_area());
  assert_rel_area("intersection", g.unsigned_area(), d.unsigned_area(), BOOLEAN_REL_TOL);
}
