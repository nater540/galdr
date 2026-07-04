//! Feature-gated GEOS parity backend (`--features geos`).
//!
//! GEOS is the exact C++ library Shapely wraps, so a GEOS-backed implementation is Eitri's path to bit-for-bit
//! parity with FlatCAM's offset/boolean output (Shapely's round-join / 8-quadrant / 5.0-mitre defaults). This is
//! the validation oracle described in `docs/eitri-porting-plan.md` §2 and §12.
//!
//! Phase 1 ships only a compiling stub: the feature flag and the [`GeoBackend`] shape are wired now so switching
//! backends later is mechanical, but every operation returns `Error::Unsupported`. The real GEOS calls land in the
//! dedicated parity phase. Enabling this feature links the `geos` crate, which requires a system GEOS at build
//! time — the default build needs no C/C++ GEOS.

use geo_types::{MultiPolygon, Polygon};

use eitri_core::{Error, Result};

use crate::{GeoBackend, JoinType, WindingDirection};

/// The GEOS-backed geometry backend (stub). Present only under `--features geos`.
#[derive(Debug, Default, Clone, Copy)]
pub struct GeosBackend;

impl GeosBackend {
  /// Construct the GEOS backend.
  pub fn new() -> GeosBackend {
    GeosBackend
  }

  /// The single place the not-yet-implemented state is spelled out, so every method reads the same.
  fn pending<T>() -> Result<T> {
    Err(Error::Unsupported("geos parity backend is not implemented yet (Phase 1 stub)".to_string()))
  }
}

impl GeoBackend for GeosBackend {
  fn offset(&self, _poly: &Polygon<f64>, _distance: f64, _join: JoinType, _miter_limit: f64) -> Result<MultiPolygon<f64>> {
    GeosBackend::pending()
  }

  fn union_all(&self, _polys: &[Polygon<f64>]) -> Result<MultiPolygon<f64>> {
    GeosBackend::pending()
  }

  fn difference(&self, _a: &MultiPolygon<f64>, _b: &MultiPolygon<f64>) -> Result<MultiPolygon<f64>> {
    GeosBackend::pending()
  }

  fn intersection(&self, _a: &MultiPolygon<f64>, _b: &MultiPolygon<f64>) -> Result<MultiPolygon<f64>> {
    GeosBackend::pending()
  }

  fn simplify(&self, _poly: &Polygon<f64>, _tolerance: f64) -> Result<Polygon<f64>> {
    GeosBackend::pending()
  }

  fn normalize_winding(&self, poly: &Polygon<f64>, direction: WindingDirection) -> Polygon<f64> {
    // Winding normalization is pure geometry with no GEOS dependency; reuse the shared implementation even in the
    // stub so the two backends can never disagree on orientation convention.
    crate::winding::normalize(poly, direction)
  }
}
