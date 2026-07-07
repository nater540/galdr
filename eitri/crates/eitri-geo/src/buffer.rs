//! Open-path buffering (stroking) — turn a polyline into a filled band of a given radius.
//!
//! Gerber draws (`D01`) with a round aperture are a segment stroked by the aperture radius, and Excellon routed
//! slots are the same operation; both parsers need it, so it lives in the geometry backend rather than being
//! re-derived per crate. Backed by Clipper2's open-path inflate in the shared integer space.

use clipper2::{EndType, JoinType, inflate};
use geo_types::{LineString, MultiPolygon};

use eitri_core::Result;

use crate::convert::{EitriScale, paths_to_multipolygon};

/// How the ends of a stroked open path are capped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CapStyle {
  /// Rounded ends (a round aperture / round-nose slot).
  Round,
  /// Squared ends, extended by the radius.
  Square,
  /// Flat ends, flush with the path endpoints (no extension).
  Butt,
}

impl From<CapStyle> for EndType {
  fn from(cap: CapStyle) -> EndType {
    match cap {
      CapStyle::Round => EndType::Round,
      CapStyle::Square => EndType::Square,
      CapStyle::Butt => EndType::Butt,
    }
  }
}

/// Stroke an open polyline by `radius` millimetres, producing the filled band. Bends are rounded (matching a round
/// aperture); `cap` controls the end treatment. A degenerate path (fewer than two distinct points) yields an empty
/// result rather than an error.
pub fn buffer_path(path: &LineString<f64>, radius: f64, cap: CapStyle) -> Result<MultiPolygon<f64>> {
  if path.0.len() < 2 || radius <= 0.0 {
    return Ok(MultiPolygon::new(Vec::new()));
  }
  let points: Vec<(f64, f64)> = path.0.iter().map(|c| (c.x, c.y)).collect();
  let paths: clipper2::Paths<EitriScale> = vec![points].into();
  let stroked = inflate(paths, radius, JoinType::Round, cap.into(), 2.0);
  Ok(paths_to_multipolygon(stroked))
}

#[cfg(test)]
mod tests {
  use super::*;
  use geo::algorithm::area::Area;
  use geo_types::Coord;

  #[test]
  fn round_cap_stroke_area_matches_capsule() {
    // A 10mm segment stroked by radius 1 with round caps is a capsule: 10*2 + pi*1^2 ≈ 23.14.
    let path = LineString(vec![Coord { x: 0.0, y: 0.0 }, Coord { x: 10.0, y: 0.0 }]);
    let band = buffer_path(&path, 1.0, CapStyle::Round).expect("stroke");
    assert!((band.unsigned_area() - 23.1416).abs() < 0.05, "area was {}", band.unsigned_area());
  }

  #[test]
  fn square_cap_stroke_area_matches_rectangle() {
    // Square caps extend by the radius, giving a 12x2 rectangle: area 24.
    let path = LineString(vec![Coord { x: 0.0, y: 0.0 }, Coord { x: 10.0, y: 0.0 }]);
    let band = buffer_path(&path, 1.0, CapStyle::Square).expect("stroke");
    assert!((band.unsigned_area() - 24.0).abs() < 1e-2, "area was {}", band.unsigned_area());
  }

  #[test]
  fn degenerate_path_is_empty_not_an_error() {
    let single = LineString(vec![Coord { x: 0.0, y: 0.0 }]);
    assert!(buffer_path(&single, 1.0, CapStyle::Round).expect("stroke").0.is_empty());
  }
}
