//! Non-copper clearing — remove *all* copper-free area rather than a thin isolation ring.
//!
//! Provenance: FlatCAM's "non-copper regions" / "copper clear" tooling (see `docs/eitri-porting-plan.md` §7.3). This
//! is isolation taken to its extreme: compute the region to clear as `frame − copper` (optionally with a margin),
//! then [`paint`](crate::paint) that region with any [`PaintStrategy`]. It is almost pure composition over the
//! Phase-4/§7.2 primitives — the difference gives the clear region and paint fills it, so the fill paths reach
//! G-code through the same shared emitter as everything else.

use geo_types::{Coord, LineString, MultiPolygon, Polygon};

use eitri_core::{CancelToken, Error, ProgressReporter, Result};
use eitri_geo::{GeoBackend, bounds};

use crate::paint::{PaintParams, PaintResult, PaintStrategy, paint};

/// How the outer frame that bounds the clearing is defined.
#[derive(Debug, Clone, PartialEq)]
pub enum Boundary {
  /// The axis-aligned bounding box of the copper, expanded outward by `margin` millimetres on every side.
  BoundingBox {
    /// Outward expansion of the copper bounding box (millimetres, `>= 0`).
    margin: f64,
  },
  /// An explicit boundary region (a traced board outline, a convex hull, etc.) to clear within.
  Region(MultiPolygon<f64>),
}

/// The clear region `frame − copper` for the given `boundary`. Exposed so callers can inspect or further process the
/// region before painting it.
pub fn clear_region<B>(copper: &MultiPolygon<f64>, boundary: &Boundary, backend: &B) -> Result<MultiPolygon<f64>>
where
  B: GeoBackend,
{
  let frame = match boundary {
    Boundary::BoundingBox { margin } => {
      if margin.is_nan() || *margin < 0.0 {
        return Err(Error::InvalidGeometry("non-copper margin must be non-negative".to_string()));
      }
      let (x0, y0, x1, y1) = bounds(copper)
        .ok_or_else(|| Error::InvalidGeometry("non-copper clearing needs non-empty copper for a bbox frame".to_string()))?;
      MultiPolygon::new(vec![rectangle(x0 - margin, y0 - margin, x1 + margin, y1 + margin)])
    }
    Boundary::Region(region) => region.clone(),
  };
  backend.difference(&frame, copper)
}

/// Clear all non-copper area within `boundary`: compute `frame − copper`, then paint that region with `strategy`.
/// The returned [`PaintResult`] carries the fill paths, ready for the shared emitter via
/// [`PaintResult::toolpaths`](crate::PaintResult::toolpaths).
pub fn clear_noncopper<S, B>(
  copper: &MultiPolygon<f64>,
  boundary: &Boundary,
  params: &PaintParams,
  strategy: &S,
  backend: &B,
  progress: &ProgressReporter,
  cancel: &CancelToken,
) -> Result<PaintResult>
where
  S: PaintStrategy + ?Sized,
  B: GeoBackend + Sync,
{
  cancel.check()?;
  let region = clear_region(copper, boundary, backend)?;
  paint(&region, params, strategy, backend, progress, cancel)
}

/// An axis-aligned rectangle polygon with a CCW exterior ring.
fn rectangle(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon<f64> {
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

#[cfg(test)]
mod tests {
  use super::*;
  use crate::paint::Concentric;
  use eitri_geo::{DefaultBackend, contains_point};

  fn square(cx: f64, cy: f64, half: f64) -> Polygon<f64> {
    rectangle(cx - half, cy - half, cx + half, cy + half)
  }

  fn silent() -> (ProgressReporter, CancelToken) {
    (ProgressReporter::silent(), CancelToken::new())
  }

  fn backend() -> DefaultBackend {
    DefaultBackend::new()
  }

  #[test]
  fn bbox_frame_clear_region_surrounds_but_excludes_the_copper() {
    let copper = MultiPolygon::new(vec![square(0.0, 0.0, 5.0)]);
    let region = clear_region(&copper, &Boundary::BoundingBox { margin: 4.0 }, &backend()).expect("clear region");
    // The clear region reaches out to the expanded frame edge but never overlaps the copper interior.
    let (x0, y0, x1, y1) = bounds(&region).expect("bounds");
    assert!(x0 <= -9.0 + 1e-6 && y0 <= -9.0 + 1e-6 && x1 >= 9.0 - 1e-6 && y1 >= 9.0 - 1e-6, "frame reaches the margin");
    assert!(!contains_point(&region, 0.0, 0.0), "the copper centre is not part of the clear region");
    assert!(contains_point(&region, 7.0, 0.0), "the space between copper and frame is clear region");
  }

  #[test]
  fn clearing_paints_the_frame_without_touching_copper() {
    let copper = MultiPolygon::new(vec![square(0.0, 0.0, 5.0)]);
    let copper_only = copper.clone();
    let params = PaintParams { tool_diameter: 1.0, overlap: 0.3, ..Default::default() };
    let (p, c) = silent();
    let result = clear_noncopper(&copper, &Boundary::BoundingBox { margin: 4.0 }, &params, &Concentric, &backend(), &p, &c)
      .expect("clear");
    assert!(!result.is_empty(), "the non-copper ring around a pad must produce fill paths");
    let pts: Vec<_> = result.paths.iter().flat_map(|r| r.points.iter().copied()).collect();
    assert!(pts.iter().all(|pt| !contains_point(&copper_only, pt.x, pt.y)), "no fill path enters the copper");
  }

  #[test]
  fn explicit_boundary_region_is_honoured() {
    // Clear within an explicit 30x30 frame around a small central pad.
    let copper = MultiPolygon::new(vec![square(0.0, 0.0, 2.0)]);
    let frame = MultiPolygon::new(vec![square(0.0, 0.0, 15.0)]);
    let region = clear_region(&copper, &Boundary::Region(frame), &backend()).expect("clear region");
    assert!(contains_point(&region, 10.0, 10.0), "a corner of the explicit frame is clear region");
    assert!(!contains_point(&region, 0.0, 0.0), "the central pad is excluded");
  }

  #[test]
  fn empty_copper_bbox_frame_is_rejected() {
    let empty = MultiPolygon::new(Vec::new());
    assert!(clear_region(&empty, &Boundary::BoundingBox { margin: 1.0 }, &backend()).is_err());
  }

  #[test]
  fn negative_margin_is_rejected() {
    let copper = MultiPolygon::new(vec![square(0.0, 0.0, 5.0)]);
    assert!(clear_region(&copper, &Boundary::BoundingBox { margin: -1.0 }, &backend()).is_err());
  }

  #[test]
  fn cancelled_clearing_returns_error() {
    let copper = MultiPolygon::new(vec![square(0.0, 0.0, 5.0)]);
    let cancel = CancelToken::new();
    cancel.cancel();
    assert!(clear_noncopper(
      &copper,
      &Boundary::BoundingBox { margin: 2.0 },
      &PaintParams::default(),
      &Concentric,
      &backend(),
      &ProgressReporter::silent(),
      &cancel,
    )
    .is_err());
  }
}
