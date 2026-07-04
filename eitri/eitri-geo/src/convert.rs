//! Conversions between `geo_types` polygons and the Clipper2 integer-path model, plus the shared integer scaler.
//!
//! Clipper2 works in scaled-integer space; [`EitriScale`] pins its multiplier to `eitri_core::INTEGER_SCALE` so
//! there is one snapping factor across the whole engine. Offset results come back as a flat list of rings whose
//! winding distinguishes outer boundaries from holes — [`paths_to_multipolygon`] reassembles proper polygon
//! nesting from that (a containment test), so CAM code always receives well-formed `MultiPolygon`s.

use clipper2::{Paths, PointScaler};
use geo::algorithm::contains::Contains;
use geo_types::{Coord, LineString, MultiPolygon, Polygon};

/// The one integer scaler for every Clipper2 operation: millimetres are multiplied by `INTEGER_SCALE` and rounded
/// onto the integer grid. Keeping this equal to the core constant is what makes "one shared scale factor" true.
#[derive(Debug, Default, Clone, Copy, PartialEq, Hash)]
pub struct EitriScale;

impl PointScaler for EitriScale {
  const MULTIPLIER: f64 = eitri_core::INTEGER_SCALE;
}

/// Convert a polygon (exterior + holes) into Clipper2 paths, dropping each ring's redundant closing vertex.
pub(crate) fn polygon_to_paths(poly: &Polygon<f64>) -> Paths<EitriScale> {
  let mut rings: Vec<Vec<(f64, f64)>> = Vec::with_capacity(1 + poly.interiors().len());
  rings.push(ring_to_tuples(poly.exterior()));
  for hole in poly.interiors() {
    rings.push(ring_to_tuples(hole));
  }
  rings.into()
}

/// Convert one ring to `(x, y)` tuples with the closing duplicate vertex removed (Clipper closes implicitly).
fn ring_to_tuples(ring: &LineString<f64>) -> Vec<(f64, f64)> {
  let coords = &ring.0;
  let mut out: Vec<(f64, f64)> = coords.iter().map(|c| (c.x, c.y)).collect();
  if out.len() > 1 && out.first() == out.last() {
    out.pop();
  }
  out
}

/// Reassemble Clipper2 offset output (a flat ring list) into a `MultiPolygon`, nesting holes inside the outer
/// ring that contains them. Outer rings have positive signed area, holes negative — a standard Clipper convention.
pub(crate) fn paths_to_multipolygon(paths: Paths<EitriScale>) -> MultiPolygon<f64> {
  let rings: Vec<Vec<(f64, f64)>> = paths.into();

  // Split rings into outers and holes by signed-area sign, discarding degenerate (near-zero-area) rings.
  let mut outers: Vec<(f64, LineString<f64>)> = Vec::new();
  let mut holes: Vec<LineString<f64>> = Vec::new();
  for ring in rings {
    let area = signed_area(&ring);
    if area.abs() < f64::EPSILON {
      continue;
    }
    let line = tuples_to_linestring(&ring);
    if area > 0.0 {
      outers.push((area, line));
    } else {
      holes.push(line);
    }
  }

  // Assign each hole to the smallest-area outer that contains it, so a hole nested in an inner island lands there
  // rather than in an enclosing outer. Sorting outers by ascending area makes "smallest containing" a first match.
  outers.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
  let mut assembled: Vec<(LineString<f64>, Vec<LineString<f64>>)> =
    outers.into_iter().map(|(_, ext)| (ext, Vec::new())).collect();

  for hole in holes {
    let probe = representative_point(&hole);
    if let Some((ext, hole_bucket)) = assembled.iter_mut().find(|(ext, _)| {
      Polygon::new(ext.clone(), Vec::new()).contains(&probe)
    }) {
      let _ = ext; // The matched outer's holes bucket is what we extend.
      hole_bucket.push(hole);
    }
    // A hole with no containing outer is dropped: it cannot form a valid polygon on its own.
  }

  MultiPolygon(assembled.into_iter().map(|(ext, holes)| Polygon::new(ext, holes)).collect())
}

/// Shoelace signed area; positive for counter-clockwise rings.
fn signed_area(ring: &[(f64, f64)]) -> f64 {
  let n = ring.len();
  if n < 3 {
    return 0.0;
  }
  let mut acc = 0.0;
  for i in 0..n {
    let (x1, y1) = ring[i];
    let (x2, y2) = ring[(i + 1) % n];
    acc += x1 * y2 - x2 * y1;
  }
  acc / 2.0
}

/// A representative interior point of a ring: the mean of its vertices. Adequate for the convex-ish holes offset
/// output produces; a full point-in-polygon-safe representative point is a later-phase refinement.
fn representative_point(ring: &LineString<f64>) -> Coord<f64> {
  let coords = &ring.0;
  let n = coords.len().max(1) as f64;
  let (sx, sy) = coords.iter().fold((0.0, 0.0), |(sx, sy), c| (sx + c.x, sy + c.y));
  Coord { x: sx / n, y: sy / n }
}

/// Build a closed `LineString` from `(x, y)` tuples (`geo_types` closes it on `Polygon::new`).
fn tuples_to_linestring(ring: &[(f64, f64)]) -> LineString<f64> {
  LineString(ring.iter().map(|&(x, y)| Coord { x, y }).collect())
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn scaler_matches_the_shared_core_constant() {
    assert_eq!(EitriScale::MULTIPLIER, eitri_core::INTEGER_SCALE);
  }

  #[test]
  fn signed_area_sign_tracks_winding() {
    let ccw = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)];
    let cw = [(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0)];
    assert!(signed_area(&ccw) > 0.0);
    assert!(signed_area(&cw) < 0.0);
    assert!((signed_area(&ccw).abs() - 4.0).abs() < 1e-9);
  }
}
