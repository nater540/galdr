//! The default geometry backend: Clipper2 offsetting, `geo`/`i_overlay` booleans, Douglas–Peucker simplification.
//!
//! This is the shipping backend. Offsetting goes through Clipper2 in the shared integer space; boolean ops go
//! through `geo`'s `BooleanOps` (backed by `i_overlay`), which is robust over the large ring counts Gerber import
//! produces. Nothing here is CAM-specific — it only fulfils the [`GeoBackend`] contract.

use clipper2::{EndType, JoinType as ClipperJoin, PointScaler, inflate};
use geo::algorithm::bool_ops::{BooleanOps, unary_union};
use geo::algorithm::simplify::Simplify;
use geo_types::{MultiPolygon, Polygon};

use eitri_core::Result;

use crate::convert::{EitriScale, paths_to_multipolygon, polygon_to_paths};
use crate::{GeoBackend, JoinType, WindingDirection, winding};

/// The default, all-Rust-plus-Clipper2 geometry backend. Zero-sized; construct with [`DefaultBackend::new`].
#[derive(Debug, Default, Clone, Copy)]
pub struct DefaultBackend;

impl DefaultBackend {
  /// Construct the default backend.
  pub fn new() -> DefaultBackend {
    DefaultBackend
  }
}

impl From<JoinType> for ClipperJoin {
  fn from(join: JoinType) -> ClipperJoin {
    match join {
      JoinType::Round => ClipperJoin::Round,
      JoinType::Miter => ClipperJoin::Miter,
      JoinType::Square => ClipperJoin::Square,
    }
  }
}

impl GeoBackend for DefaultBackend {
  fn offset(&self, poly: &Polygon<f64>, distance: f64, join: JoinType, miter_limit: f64) -> Result<MultiPolygon<f64>> {
    // Normalize winding first (exterior CCW, holes CW) so Clipper reads holes as holes rather than as a second
    // solid contour — a caller may hand us a polygon whose hole is wound the same way as its exterior.
    let poly = winding::normalize(poly, WindingDirection::Ccw);
    let paths = polygon_to_paths(&poly);
    // Clipper2 0.6 scales `miter_limit` by the point multiplier, but the limit is a dimensionless ratio — so
    // pre-divide by the multiplier here to make the public API value a true ratio again.
    let effective_miter = miter_limit / EitriScale::MULTIPLIER;
    let inflated = inflate(paths, distance, join.into(), EndType::Polygon, effective_miter);
    Ok(paths_to_multipolygon(inflated))
  }

  fn union_all(&self, polys: &[Polygon<f64>]) -> Result<MultiPolygon<f64>> {
    if polys.is_empty() {
      return Ok(MultiPolygon::new(Vec::new()));
    }
    Ok(unary_union(polys.iter()))
  }

  fn difference(&self, a: &MultiPolygon<f64>, b: &MultiPolygon<f64>) -> Result<MultiPolygon<f64>> {
    Ok(a.difference(b))
  }

  fn intersection(&self, a: &MultiPolygon<f64>, b: &MultiPolygon<f64>) -> Result<MultiPolygon<f64>> {
    Ok(a.intersection(b))
  }

  fn simplify(&self, poly: &Polygon<f64>, tolerance: f64) -> Result<Polygon<f64>> {
    Ok(poly.simplify(tolerance))
  }

  fn normalize_winding(&self, poly: &Polygon<f64>, direction: WindingDirection) -> Polygon<f64> {
    winding::normalize(poly, direction)
  }
}
