//! `eitri-geo` — the swappable geometry backend for the Eitri CAM engine.
//!
//! Everything geometric FlatCAM did through Shapely (offsetting/buffering, boolean ops, unary unions,
//! simplification) is routed through one CAM-shaped trait here — [`GeoBackend`]. CAM code speaks distances,
//! joins, and winding direction; it never sees which backend answered, so the backend can be swapped without
//! touching a line of CAM logic. See `docs/eitri-porting-plan.md` §2.
//!
//! - [`DefaultBackend`] — the shipping backend: Clipper2 offsetting, `geo`/`i_overlay` booleans, DP simplify.
//! - [`GeosBackend`] (feature `geos`) — a compiling stub of the Shapely-parity backend; full impl is a later phase.
//! - [`offset_arc`] — arc-preserving polyline offsetting via `cavalier_contours`, for smoother toolpaths.
//!
//! Winding normalization ([`GeoBackend::normalize_winding`]) is load-bearing for milling direction (climb vs
//! conventional), which is set by ring orientation — see [`winding`].

#![forbid(unsafe_code)]

mod convert;
mod default_backend;

pub mod arc;
pub mod buffer;
pub mod flatten;
pub mod interior;
pub mod mesh;
pub mod region;
pub mod transform;
pub mod winding;

#[cfg(feature = "geos")]
pub mod geos_backend;

use geo_types::{MultiPolygon, Polygon};

use eitri_core::Result;

pub use arc::{ArcPolyline, ArcVertex, offset_arc};
pub use buffer::{CapStyle, buffer_path};
pub use default_backend::DefaultBackend;
pub use flatten::{arc_segment_count, circle_polygon, flatten_arc, flatten_bulge, flatten_cubic, flatten_quad};
pub use interior::ring_interior_point;
pub use mesh::{TriangleMesh, triangulate};
pub use region::{bounds, clip_lines, contains_point, segment_within};
pub use transform::{apply_affine, apply_affine_polygon};

#[cfg(feature = "geos")]
pub use geos_backend::GeosBackend;

// Re-export the geometry primitive types so downstream crates get one consistent `geo_types` surface.
pub use geo_types;

/// How an offset corner is joined when the outward-offset boundary turns — mirrors the Clipper2 / Shapely joins.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JoinType {
  /// Rounded corners (Shapely's default; smoothest, most vertices).
  Round,
  /// Sharp mitred corners, clipped by the miter limit ratio.
  Miter,
  /// Squared-off corners.
  Square,
}

/// A ring winding direction. Milling direction (climb vs conventional) is chosen by normalizing a toolpath ring to
/// one of these before G-code generation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WindingDirection {
  /// Counter-clockwise (positive orientation).
  Ccw,
  /// Clockwise (negative orientation).
  Cw,
}

impl WindingDirection {
  /// The opposite direction — used to wind holes opposite to their enclosing exterior.
  pub fn reversed(self) -> WindingDirection {
    match self {
      WindingDirection::Ccw => WindingDirection::Cw,
      WindingDirection::Cw => WindingDirection::Ccw,
    }
  }
}

/// The CAM-shaped geometry operations Eitri needs. Implemented by each backend; CAM code depends only on this
/// trait so the backend is swappable. Distances are in millimetres.
pub trait GeoBackend {
  /// Offset `poly` by `distance` millimetres (positive = outward, negative = inward), joining corners per `join`.
  /// Returns a `MultiPolygon` because an offset can split one polygon into several or merge holes.
  fn offset(&self, poly: &Polygon<f64>, distance: f64, join: JoinType, miter_limit: f64) -> Result<MultiPolygon<f64>>;

  /// Union many polygons into their merged outline — the heavy `unary_union` step Gerber import leans on.
  fn union_all(&self, polys: &[Polygon<f64>]) -> Result<MultiPolygon<f64>>;

  /// The regions of `a` not covered by `b`.
  fn difference(&self, a: &MultiPolygon<f64>, b: &MultiPolygon<f64>) -> Result<MultiPolygon<f64>>;

  /// The regions covered by both `a` and `b`.
  fn intersection(&self, a: &MultiPolygon<f64>, b: &MultiPolygon<f64>) -> Result<MultiPolygon<f64>>;

  /// Simplify `poly`, dropping vertices that lie within `tolerance` millimetres of the retained outline.
  fn simplify(&self, poly: &Polygon<f64>, tolerance: f64) -> Result<Polygon<f64>>;

  /// Return `poly` with its exterior wound in `direction` and its holes wound the opposite way. Never fails.
  fn normalize_winding(&self, poly: &Polygon<f64>, direction: WindingDirection) -> Polygon<f64>;
}
