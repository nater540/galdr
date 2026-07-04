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
    // A robust interior point is essential here: a concave hole's vertex mean can fall outside a concave outer,
    // which would drop a real hole. See [`crate::interior::ring_interior_point`].
    let Some(probe) = crate::interior::ring_interior_point(&hole) else { continue };
    if let Some((ext, hole_bucket)) = assembled.iter_mut().find(|(ext, _)| {
      Polygon::new(ext.clone(), Vec::new()).contains(&probe)
    }) {
      let _ = ext; // The matched outer's holes bucket is what we extend.
      hole_bucket.push(hole);
    }
    // A hole with no valid interior point or no containing outer is dropped: it cannot form a valid polygon alone.
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

  #[test]
  fn concave_outer_keeps_a_hole_whose_vertex_mean_escapes_it() {
    // Finding #3: a hole nested in a concave outer must survive. Here the outer is a U (a 10x10 square with a
    // top-middle notch removed); the hole is a U-shaped band following the outer's arms. The band's vertex mean
    // lands in the notch — outside the outer — so the old mean-of-vertices probe dropped the hole entirely.
    let outer: Vec<(f64, f64)> =
      vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (7.0, 10.0), (7.0, 5.0), (3.0, 5.0), (3.0, 10.0), (0.0, 10.0)];
    // Clockwise (negative area) so `paths_to_multipolygon` classifies it as a hole, not an outer.
    let hole: Vec<(f64, f64)> =
      vec![(2.0, 9.0), (2.0, 2.0), (8.0, 2.0), (8.0, 9.0), (9.0, 9.0), (9.0, 1.0), (1.0, 1.0), (1.0, 9.0)];

    // The vertex mean of the hole band lands at (5, 5.25) — inside the notch, outside the outer U.
    let mean_x = hole.iter().map(|p| p.0).sum::<f64>() / hole.len() as f64;
    let mean_y = hole.iter().map(|p| p.1).sum::<f64>() / hole.len() as f64;
    let outer_poly = Polygon::new(tuples_to_linestring(&outer), Vec::new());
    assert!(
      !outer_poly.contains(&Coord { x: mean_x, y: mean_y }),
      "precondition: hole vertex mean ({mean_x}, {mean_y}) must fall outside the concave outer"
    );

    let paths: Paths<EitriScale> = vec![outer, hole].into();
    let mp = paths_to_multipolygon(paths);

    assert_eq!(mp.0.len(), 1, "one outer polygon expected");
    assert_eq!(mp.0[0].interiors().len(), 1, "the nested hole must be preserved, not dropped");
  }
}
