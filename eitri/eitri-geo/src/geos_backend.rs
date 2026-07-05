//! Feature-gated GEOS parity backend (`--features geos`).
//!
//! GEOS is the exact C++ library Shapely wraps, so a GEOS-backed implementation is Eitri's path to maximum-fidelity
//! parity with FlatCAM's offset/boolean output (Shapely's round-join / 8-quadrant / 5.0-mitre `buffer` defaults).
//! This is the validation oracle described in `docs/eitri-porting-plan.md` §2 (the "porting risk") and §12: it lets
//! us measure how close the shipping Clipper2/geo [`DefaultBackend`](crate::DefaultBackend) path lands to Shapely.
//!
//! It stays feature-gated: enabling `geos` links the `geos` crate, which needs a system GEOS (`libgeos_c`) at build
//! time via `geos-config`. The default build needs no C/C++ GEOS and does not compile this module.
//!
//! Geometry crosses the FFI boundary through the geos crate's `geo-types` interop: our `geo_types` polygons convert
//! into GEOS geometries (direct `CoordSeq` construction) and back (WKT pivot at full double precision). All of that
//! conversion is centralized in [`convert`] so the operation methods read as a straight line of intent.

use geo_types::{MultiPolygon, Polygon};

use eitri_core::Result;

use crate::{GeoBackend, JoinType, WindingDirection};

/// Shapely's `buffer` uses 8 segments per quarter-circle to approximate round joins (< 2% max error in the buffer
/// distance). Pinning the same value here is what makes round-join offsets match Shapely rather than merely "round".
const SHAPELY_QUADRANT_SEGMENTS: i32 = 8;

/// The GEOS-backed geometry backend. Present only under `--features geos`. Zero-sized; construct with
/// [`GeosBackend::new`]. Each call builds its own GEOS geometries, so the backend holds no shared state and is
/// cheap to clone and pass across threads.
#[derive(Debug, Default, Clone, Copy)]
pub struct GeosBackend;

impl GeosBackend {
  /// Construct the GEOS backend.
  pub fn new() -> GeosBackend {
    GeosBackend
  }
}

impl GeoBackend for GeosBackend {
  fn offset(&self, poly: &Polygon<f64>, distance: f64, join: JoinType, miter_limit: f64) -> Result<MultiPolygon<f64>> {
    // Normalize winding (exterior CCW, holes CW) before buffering, mirroring `DefaultBackend`'s precondition so the
    // two backends receive identical input. GEOS identifies holes structurally (shell ring vs interior rings), so a
    // valid polygon's buffer is orientation-independent — this only guards a caller that hands us a mis-wound hole.
    let poly = crate::winding::normalize(poly, WindingDirection::Ccw);
    let geom = convert::polygon_to_geos(&poly)?;

    // Round join / round cap / 8 quadrant segments are Shapely's `buffer` defaults; the mitre limit flows from the
    // caller so this backend faithfully reproduces whatever offset the CAM layer requests. Square maps to GEOS bevel
    // (GEOS has no squared-off join — "Square" is only a cap style there); the mismatch is documented on the mapping.
    let params = geos::BufferParams::builder()
      .end_cap_style(geos::CapStyle::Round)
      .join_style(join_to_geos(join))
      .mitre_limit(miter_limit)
      .quadrant_segments(SHAPELY_QUADRANT_SEGMENTS)
      .build()
      .map_err(convert::geos_err)?;

    // A negative `distance` is an inward buffer — GEOS handles the sign natively, including collapse to empty.
    let buffered = geos::Geom::buffer_with_params(&geom, distance, &params).map_err(convert::geos_err)?;
    convert::geos_to_multipolygon(&buffered)
  }

  fn union_all(&self, polys: &[Polygon<f64>]) -> Result<MultiPolygon<f64>> {
    // Short-circuit the empty case so an empty input never touches GEOS, matching `DefaultBackend`.
    if polys.is_empty() {
      return Ok(MultiPolygon::new(Vec::new()));
    }
    let collection = MultiPolygon(polys.to_vec());
    let geom = convert::multipolygon_to_geos(&collection)?;
    // `unary_union` dissolves all shared boundaries across the collection — the exact `unary_union` Gerber import
    // leans on, and GEOS's robust implementation over the large ring counts Gerber assembly produces.
    let unioned = geos::Geom::unary_union(&geom).map_err(convert::geos_err)?;
    convert::geos_to_multipolygon(&unioned)
  }

  fn difference(&self, a: &MultiPolygon<f64>, b: &MultiPolygon<f64>) -> Result<MultiPolygon<f64>> {
    let ga = convert::multipolygon_to_geos(a)?;
    let gb = convert::multipolygon_to_geos(b)?;
    let out = geos::Geom::difference(&ga, &gb).map_err(convert::geos_err)?;
    convert::geos_to_multipolygon(&out)
  }

  fn intersection(&self, a: &MultiPolygon<f64>, b: &MultiPolygon<f64>) -> Result<MultiPolygon<f64>> {
    let ga = convert::multipolygon_to_geos(a)?;
    let gb = convert::multipolygon_to_geos(b)?;
    let out = geos::Geom::intersection(&ga, &gb).map_err(convert::geos_err)?;
    convert::geos_to_multipolygon(&out)
  }

  fn simplify(&self, poly: &Polygon<f64>, tolerance: f64) -> Result<Polygon<f64>> {
    // Topology-preserving simplification, matching Shapely's `simplify(tolerance, preserve_topology=True)` default —
    // it drops vertices within `tolerance` of the retained outline without ever collapsing a ring into an invalid
    // geometry. (Shapely's non-preserving path is the raw Douglas–Peucker `simplify`, which we deliberately avoid.)
    let geom = convert::polygon_to_geos(poly)?;
    let simplified = geom.topology_preserve_simplify(tolerance).map_err(convert::geos_err)?;
    convert::geos_to_polygon(&simplified)
  }

  fn normalize_winding(&self, poly: &Polygon<f64>, direction: WindingDirection) -> Polygon<f64> {
    // Winding normalization is pure geometry with no GEOS dependency; reuse the shared implementation so the two
    // backends can never disagree on the climb/conventional orientation convention.
    crate::winding::normalize(poly, direction)
  }
}

/// Map our CAM join type onto GEOS's join style. `Square` has no GEOS equivalent (GEOS "Square" is a *cap* style),
/// so it maps to `Bevel` — the closest flattened-corner join. `Round` and `Mitre` map exactly, and `Round` is the
/// join that reproduces Shapely's default.
fn join_to_geos(join: JoinType) -> geos::JoinStyle {
  match join {
    JoinType::Round => geos::JoinStyle::Round,
    JoinType::Miter => geos::JoinStyle::Mitre,
    JoinType::Square => geos::JoinStyle::Bevel,
  }
}

/// Conversions between `geo_types` geometry and GEOS geometry, plus the geos-error bridge. Centralized here so the
/// operation methods above never touch the FFI shape directly and every op coerces results the same way.
mod convert {
  use geo_types::{Geometry, MultiPolygon, Polygon};

  use eitri_core::{Error, Result};

  /// Bridge a `geos::Error` into our error type. GEOS operation failures are backend failures, not invalid input,
  /// so they land on [`Error::Geometry`].
  pub(super) fn geos_err(err: geos::Error) -> Error {
    Error::Geometry(format!("geos: {err}"))
  }

  /// Convert a `geo_types` polygon into a GEOS geometry.
  pub(super) fn polygon_to_geos(poly: &Polygon<f64>) -> Result<geos::Geometry> {
    geos::Geometry::try_from(poly).map_err(geos_err)
  }

  /// Convert a `geo_types` multipolygon into a GEOS geometry (a GEOS `MULTIPOLYGON`).
  pub(super) fn multipolygon_to_geos(mp: &MultiPolygon<f64>) -> Result<geos::Geometry> {
    geos::Geometry::try_from(mp).map_err(geos_err)
  }

  /// Convert a GEOS geometry back into a `geo_types::Geometry` (the geos crate pivots through WKT at full double
  /// precision — its default writer uses `roundingPrecision = -1`, so the round-trip is loss-free to ~16 digits).
  fn geos_to_geometry(geom: &geos::Geometry) -> Result<Geometry<f64>> {
    Geometry::try_from(geom).map_err(geos_err)
  }

  /// Coerce a GEOS result to a `MultiPolygon`. Areal results pass through; a `GeometryCollection` is flattened to its
  /// polygonal members; lower-dimensional artifacts (touching edges/points that boolean ops can emit) carry no area
  /// and become an empty result. That areal-only contract matches the trait's `MultiPolygon` return type — and the
  /// `DefaultBackend` (geo `BooleanOps`) drops the same lower-dimensional pieces by construction.
  pub(super) fn geos_to_multipolygon(geom: &geos::Geometry) -> Result<MultiPolygon<f64>> {
    let mut mp = collect_polygons(geos_to_geometry(geom)?);
    // Drop empty rings: GEOS reports a collapsed buffer (e.g. an over-inset) as `POLYGON EMPTY`, which the WKT pivot
    // turns into a polygon with an empty exterior. An empty areal result is an empty `MultiPolygon` — matching
    // `DefaultBackend`, which simply produces no rings — not a multipolygon holding a degenerate empty member.
    mp.0.retain(|poly| !poly.exterior().0.is_empty());
    Ok(mp)
  }

  /// Flatten any `geo_types::Geometry` into the multipolygon of its areal members, dropping non-areal pieces.
  fn collect_polygons(geometry: Geometry<f64>) -> MultiPolygon<f64> {
    match geometry {
      Geometry::Polygon(poly) => MultiPolygon(vec![poly]),
      Geometry::MultiPolygon(mp) => mp,
      Geometry::GeometryCollection(gc) => {
        let mut polys: Vec<Polygon<f64>> = Vec::new();
        for member in gc {
          match member {
            Geometry::Polygon(poly) => polys.push(poly),
            Geometry::MultiPolygon(mp) => polys.extend(mp.0),
            // Nested collections are not something GEOS overlay/buffer produce, but recurse defensively rather than
            // silently swallow an areal member hidden one level down.
            other => polys.extend(collect_polygons(other).0),
          }
        }
        MultiPolygon(polys)
      }
      // Points/lines and the empty geometries carry no area — an empty areal result.
      _ => MultiPolygon(Vec::new()),
    }
  }

  /// Coerce a GEOS result that is expected to be a single polygon (the shape `simplify` returns for a polygon input).
  /// A single-member multipolygon is accepted; anything else is reported loudly rather than silently reshaped.
  pub(super) fn geos_to_polygon(geom: &geos::Geometry) -> Result<Polygon<f64>> {
    match geos_to_geometry(geom)? {
      Geometry::Polygon(poly) => Ok(poly),
      Geometry::MultiPolygon(mp) if mp.0.len() == 1 => mp
        .0
        .into_iter()
        .next()
        .ok_or_else(|| Error::InvalidGeometry("geos returned an empty single-member multipolygon".to_string())),
      other => Err(Error::InvalidGeometry(format!(
        "geos simplify expected a single polygon, got {other:?}"
      ))),
    }
  }
}

#[cfg(test)]
mod tests {
  use geo::algorithm::area::Area;
  use geo_types::{Coord, LineString, MultiPolygon, Polygon};

  use crate::{GeoBackend, JoinType, WindingDirection};

  use super::GeosBackend;

  /// Build a closed square polygon `[origin, origin+side]²` (counter-clockwise).
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

  #[test]
  fn round_trips_a_polygon_with_a_hole_through_geos() {
    // A 10x10 square with a centered 2x2 hole. Converting to GEOS and back must preserve both rings and their area.
    let outer = square(0.0, 10.0);
    let hole = LineString(vec![
      Coord { x: 4.0, y: 4.0 },
      Coord { x: 4.0, y: 6.0 },
      Coord { x: 6.0, y: 6.0 },
      Coord { x: 6.0, y: 4.0 },
      Coord { x: 4.0, y: 4.0 },
    ]);
    let poly = Polygon::new(outer.exterior().clone(), vec![hole]);

    let geom = super::convert::polygon_to_geos(&poly).expect("to geos");
    let back = super::convert::geos_to_multipolygon(&geom).expect("from geos");

    assert_eq!(back.0.len(), 1, "one polygon survives the round trip");
    assert_eq!(back.0[0].interiors().len(), 1, "the hole survives the round trip");
    // 100 minus the 2x2 hole = 96; the WKT pivot is full-precision so this is exact to floating tolerance.
    assert!((back.unsigned_area() - 96.0).abs() < 1e-6, "area preserved: {}", back.unsigned_area());
  }

  #[test]
  fn outward_offset_grows_a_square_by_a_ring() {
    // A 10x10 square offset outward by 1 mm becomes a 12x12 rounded square: area ~= 144 + rounded-corner area.
    // Between the inscribed 12x12 (144) and the circumscribed 12x12-plus-full-circle (144 + pi) the round corners
    // give 143 + 4 + pi = 147.14 nominally; assert a bracket rather than an exact value.
    let backend = GeosBackend::new();
    let out = backend.offset(&square(0.0, 10.0), 1.0, JoinType::Round, 5.0).expect("offset");
    let area = out.unsigned_area();
    assert!((143.0..=148.0).contains(&area), "offset area within the rounded-corner bracket: {area}");
  }

  #[test]
  fn inward_offset_shrinks_a_square() {
    // Negative distance is an inward buffer: a 10x10 square inset by 1 mm is exactly an 8x8 square (straight edges,
    // no corner rounding on a convex inward offset) = 64.
    let backend = GeosBackend::new();
    let out = backend.offset(&square(0.0, 10.0), -1.0, JoinType::Round, 5.0).expect("offset");
    assert!((out.unsigned_area() - 64.0).abs() < 1e-6, "inward offset area: {}", out.unsigned_area());
  }

  #[test]
  fn inward_offset_past_feature_size_collapses_to_empty() {
    // Insetting a 10x10 square by 6 mm (past its half-width) collapses it — GEOS returns an empty polygon, which
    // must coerce to an empty multipolygon rather than an error.
    let backend = GeosBackend::new();
    let out = backend.offset(&square(0.0, 10.0), -6.0, JoinType::Round, 5.0).expect("offset");
    assert!(out.0.is_empty(), "over-inset collapses to empty: {} polygons", out.0.len());
  }

  #[test]
  fn union_all_merges_two_overlapping_squares() {
    // Two 10x10 squares overlapping in a 10x5 band: union area = 100 + 100 - 50 = 150, one merged polygon.
    let backend = GeosBackend::new();
    let out = backend.union_all(&[square(0.0, 10.0), square(0.0, 5.0)]).expect("union");
    // The second square (5..15 in x? no — square(0,5) is 0..5) is fully inside the first, so union is just the first.
    assert_eq!(out.0.len(), 1, "one merged polygon");
    assert!((out.unsigned_area() - 100.0).abs() < 1e-6, "union area: {}", out.unsigned_area());
  }

  #[test]
  fn union_all_of_empty_is_empty() {
    let backend = GeosBackend::new();
    let out = backend.union_all(&[]).expect("union");
    assert!(out.0.is_empty());
  }

  #[test]
  fn difference_cuts_a_hole() {
    // A 10x10 square minus a centered 4x4 square = 100 - 16 = 84, and the result has one hole.
    let backend = GeosBackend::new();
    let a = MultiPolygon(vec![square(0.0, 10.0)]);
    let b = MultiPolygon(vec![square(3.0, 4.0)]);
    let out = backend.difference(&a, &b).expect("difference");
    assert!((out.unsigned_area() - 84.0).abs() < 1e-6, "difference area: {}", out.unsigned_area());
    assert_eq!(out.0.len(), 1);
    assert_eq!(out.0[0].interiors().len(), 1, "the removed square is a hole");
  }

  #[test]
  fn intersection_keeps_only_the_overlap() {
    // square(0,10) spans x,y in [0,10]; square(6,10) spans [6,16]. They overlap on x in [6,10] AND y in [6,10] —
    // a 4x4 corner = 16.
    let backend = GeosBackend::new();
    let a = MultiPolygon(vec![square(0.0, 10.0)]);
    let b = MultiPolygon(vec![square(6.0, 10.0)]);
    let out = backend.intersection(&a, &b).expect("intersection");
    assert!((out.unsigned_area() - 16.0).abs() < 1e-6, "intersection area: {}", out.unsigned_area());
  }

  #[test]
  fn simplify_drops_a_collinear_vertex() {
    // A square with an extra midpoint vertex on one edge. Topology-preserving simplify removes the redundant
    // collinear point without changing the shape (area stays 100).
    let poly = Polygon::new(
      LineString(vec![
        Coord { x: 0.0, y: 0.0 },
        Coord { x: 5.0, y: 0.0 }, // redundant collinear midpoint on the bottom edge
        Coord { x: 10.0, y: 0.0 },
        Coord { x: 10.0, y: 10.0 },
        Coord { x: 0.0, y: 10.0 },
        Coord { x: 0.0, y: 0.0 },
      ]),
      Vec::new(),
    );
    let backend = GeosBackend::new();
    let out = backend.simplify(&poly, 0.5).expect("simplify");
    assert!((out.unsigned_area() - 100.0).abs() < 1e-6, "shape preserved: {}", out.unsigned_area());
    assert!(out.exterior().0.len() <= 5, "collinear vertex dropped: {} vertices", out.exterior().0.len());
  }

  #[test]
  fn normalize_winding_matches_the_shared_implementation() {
    // The GEOS backend's winding normalization must be byte-identical to the shared pure implementation, so the two
    // backends can never disagree on orientation.
    let poly = square(0.0, 10.0);
    let backend = GeosBackend::new();
    let via_backend = backend.normalize_winding(&poly, WindingDirection::Cw);
    let via_shared = crate::winding::normalize(&poly, WindingDirection::Cw);
    assert_eq!(via_backend, via_shared);
  }
}
