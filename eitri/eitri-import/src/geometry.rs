//! The vector-geometry output type shared by the SVG and DXF importers.
//!
//! Both formats reduce to the same thing: a bag of **closed** shapes (filled/closed subpaths, `CIRCLE`s, closed
//! polylines) and **open** paths (open subpaths, `LINE`s, open polylines), all already flattened to millimetre
//! coordinates at the shared chord tolerance. This is a parser-local preview type on purpose — the Phase-7
//! `eitri-project` object/serde schema is deliberately *not* built here (plan §13 step 7), so importers produce
//! plain `geo_types` geometry and nothing pretends to be the project model yet.

use geo_types::{Coord, LineString, Polygon, Rect};

use eitri_core::GEOM_EPSILON_MM;

/// Flattened vector geometry recovered from an import: closed polygons and open polylines, in millimetres.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ImportedGeometry {
  /// Closed shapes (closed subpaths, circles, closed polylines) as polygons with no holes.
  pub polygons: Vec<Polygon<f64>>,
  /// Open paths (open subpaths, line segments, open polylines).
  pub polylines: Vec<LineString<f64>>,
}

impl ImportedGeometry {
  /// An empty geometry set.
  pub fn new() -> ImportedGeometry {
    ImportedGeometry::default()
  }

  /// Whether nothing was recovered.
  pub fn is_empty(&self) -> bool {
    self.polygons.is_empty() && self.polylines.is_empty()
  }

  /// Total number of recovered shapes (polygons + polylines).
  pub fn len(&self) -> usize {
    self.polygons.len() + self.polylines.len()
  }

  /// Add one flattened subpath. A `closed` subpath becomes a hole-free [`Polygon`] (its ring is closed if the
  /// endpoints do not already coincide); an open one becomes a [`LineString`]. Degenerate subpaths (fewer than two
  /// distinct points) are dropped — there is no geometry to draw.
  pub fn push_subpath(&mut self, mut points: Vec<Coord<f64>>, closed: bool) {
    dedup_coincident(&mut points);
    if points.len() < 2 {
      return;
    }
    if closed {
      // A polygon ring must be explicitly closed (first == last); add the closing vertex if the source omitted it.
      if points.first() != points.last() {
        points.push(points[0]);
      }
      // Three distinct points minimum for an area (ring len >= 4 with the closing repeat).
      if points.len() < 4 {
        return;
      }
      self.polygons.push(Polygon::new(LineString(points), Vec::new()));
    } else {
      self.polylines.push(LineString(points));
    }
  }

  /// The axis-aligned bounding box over every recovered shape, or `None` when empty. Uses `eitri_geo::bounds` so
  /// the box definition matches the rest of the engine.
  pub fn bounds(&self) -> Option<Rect<f64>> {
    let mut it = self
      .polygons
      .iter()
      .flat_map(|p| p.exterior().0.iter().copied())
      .chain(self.polylines.iter().flat_map(|l| l.0.iter().copied()));
    let first = it.next()?;
    let (mut min_x, mut min_y, mut max_x, mut max_y) = (first.x, first.y, first.x, first.y);
    for c in it {
      min_x = min_x.min(c.x);
      min_y = min_y.min(c.y);
      max_x = max_x.max(c.x);
      max_y = max_y.max(c.y);
    }
    Some(Rect::new(Coord { x: min_x, y: min_y }, Coord { x: max_x, y: max_y }))
  }
}

/// Drop consecutive coincident points (within [`GEOM_EPSILON_MM`]) so a flattened path never carries zero-length
/// segments, which downstream offset/boolean backends dislike.
fn dedup_coincident(points: &mut Vec<Coord<f64>>) {
  points.dedup_by(|a, b| (a.x - b.x).abs() < GEOM_EPSILON_MM && (a.y - b.y).abs() < GEOM_EPSILON_MM);
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn open_subpath_becomes_a_polyline() {
    let mut g = ImportedGeometry::new();
    g.push_subpath(vec![Coord { x: 0.0, y: 0.0 }, Coord { x: 10.0, y: 0.0 }], false);
    assert_eq!(g.polylines.len(), 1);
    assert!(g.polygons.is_empty());
  }

  #[test]
  fn closed_subpath_becomes_a_polygon_and_is_ring_closed() {
    let mut g = ImportedGeometry::new();
    g.push_subpath(
      vec![Coord { x: 0.0, y: 0.0 }, Coord { x: 10.0, y: 0.0 }, Coord { x: 10.0, y: 10.0 }],
      true,
    );
    assert_eq!(g.polygons.len(), 1);
    let ext = g.polygons[0].exterior();
    assert_eq!(ext.0.first(), ext.0.last(), "ring must be explicitly closed");
  }

  #[test]
  fn degenerate_subpaths_are_dropped() {
    let mut g = ImportedGeometry::new();
    g.push_subpath(vec![Coord { x: 1.0, y: 1.0 }], false);
    g.push_subpath(vec![Coord { x: 1.0, y: 1.0 }, Coord { x: 1.0, y: 1.0 }], true);
    assert!(g.is_empty(), "single/coincident points carry no geometry");
  }

  #[test]
  fn bounds_span_every_shape() {
    let mut g = ImportedGeometry::new();
    g.push_subpath(vec![Coord { x: -1.0, y: 2.0 }, Coord { x: 3.0, y: 2.0 }], false);
    g.push_subpath(vec![Coord { x: 0.0, y: -5.0 }, Coord { x: 0.0, y: 7.0 }], false);
    let b = g.bounds().expect("non-empty");
    assert_eq!(b.min(), Coord { x: -1.0, y: -5.0 });
    assert_eq!(b.max(), Coord { x: 3.0, y: 7.0 });
  }
}
