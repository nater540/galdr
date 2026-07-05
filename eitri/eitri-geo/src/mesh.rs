//! Filled-region triangulation for renderers.
//!
//! A GUI cannot hand an arbitrary polygon-with-holes to an immediate-mode painter (egui and friends only fill
//! convex paths correctly), so the engine exposes the tessellation itself: [`triangulate`] flattens a
//! `MultiPolygon` into one indexed triangle soup via `geo`'s ear-cut triangulation, which handles concavity and
//! interior rings (holes). Keeping this in `eitri-geo` keeps the front-end free of geometry math — the app maps
//! vertices to screen space and uploads the mesh, nothing more.

use geo::TriangulateEarcut;
use geo_types::MultiPolygon;

/// An indexed 2-D triangle mesh in the region's own coordinate space (millimetres for engine geometry). Every
/// consecutive index triple in `indices` names one triangle over `vertices`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TriangleMesh {
  /// The triangle-corner positions, as `[x, y]` pairs.
  pub vertices: Vec<[f64; 2]>,
  /// Triangle corner indices into `vertices`, three per triangle.
  pub indices: Vec<u32>,
}

impl TriangleMesh {
  /// The number of triangles in the mesh.
  pub fn triangle_count(&self) -> usize {
    self.indices.len() / 3
  }

  /// The total signed-area-free (absolute) area covered by the triangles, in the mesh's own units squared.
  pub fn area(&self) -> f64 {
    let mut area = 0.0;
    for tri in self.indices.chunks_exact(3) {
      let [a, b, c] = [
        self.vertices[tri[0] as usize],
        self.vertices[tri[1] as usize],
        self.vertices[tri[2] as usize],
      ];
      area += ((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])).abs() / 2.0;
    }
    area
  }
}

/// Ear-cut triangulate every polygon of `region` (holes honoured) into one merged, indexed mesh. An empty or
/// degenerate region yields an empty mesh. Vertex positions are the region's own coordinates, untransformed.
pub fn triangulate(region: &MultiPolygon<f64>) -> TriangleMesh {
  let mut mesh = TriangleMesh::default();
  for polygon in &region.0 {
    // A ring needs at least three distinct corners to enclose area; earcut on less yields nothing anyway, but
    // skipping early avoids paying for the raw conversion of obviously degenerate inputs.
    if polygon.exterior().0.len() < 4 {
      continue;
    }
    let raw = polygon.earcut_triangles_raw();
    let base = mesh.vertices.len() as u32;
    mesh.vertices.extend(raw.vertices);
    for index in raw.triangle_indices {
      mesh.indices.push(base + index as u32);
    }
  }
  mesh
}

#[cfg(test)]
mod tests {
  use super::*;
  use geo_types::{LineString, Polygon};

  /// A closed axis-aligned rectangle ring from `(x0, y0)` to `(x1, y1)`.
  fn rect_ring(x0: f64, y0: f64, x1: f64, y1: f64) -> LineString<f64> {
    LineString::from(vec![(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)])
  }

  #[test]
  fn a_unit_square_triangulates_to_two_triangles_of_area_one() {
    let square = MultiPolygon::new(vec![Polygon::new(rect_ring(0.0, 0.0, 1.0, 1.0), vec![])]);
    let mesh = triangulate(&square);
    assert_eq!(mesh.triangle_count(), 2, "a quad ear-cuts into exactly two triangles");
    assert!((mesh.area() - 1.0).abs() < 1e-9, "the triangles cover the square exactly: {}", mesh.area());
    // Every index must be in bounds — the app feeds these straight into a GPU mesh.
    assert!(mesh.indices.iter().all(|&i| (i as usize) < mesh.vertices.len()), "indices stay in bounds");
  }

  #[test]
  fn a_square_with_a_hole_covers_only_the_annular_area() {
    // 10×10 outer, 4×4 hole → area 100 − 16 = 84. Ear-cut must honour the interior ring, or a filled render
    // would paint over the hole (exactly the bug this API exists to prevent in the GUI).
    let with_hole = MultiPolygon::new(vec![Polygon::new(
      rect_ring(0.0, 0.0, 10.0, 10.0),
      vec![rect_ring(3.0, 3.0, 7.0, 7.0)],
    )]);
    let mesh = triangulate(&with_hole);
    assert!((mesh.area() - 84.0).abs() < 1e-6, "hole area must be excluded: {}", mesh.area());
    // The hole's interior must not be covered: its centre lies inside no triangle.
    let (cx, cy) = (5.0, 5.0);
    for tri in mesh.indices.chunks_exact(3) {
      let [a, b, c] =
        [mesh.vertices[tri[0] as usize], mesh.vertices[tri[1] as usize], mesh.vertices[tri[2] as usize]];
      let sign = |p: [f64; 2], q: [f64; 2]| (q[0] - p[0]) * (cy - p[1]) - (q[1] - p[1]) * (cx - p[0]);
      let (d1, d2, d3) = (sign(a, b), sign(b, c), sign(c, a));
      let strictly_inside = (d1 > 1e-12 && d2 > 1e-12 && d3 > 1e-12) || (d1 < -1e-12 && d2 < -1e-12 && d3 < -1e-12);
      assert!(!strictly_inside, "the hole centre must not be inside any triangle");
    }
  }

  #[test]
  fn two_disjoint_polygons_merge_into_one_mesh_with_offset_indices() {
    let two = MultiPolygon::new(vec![
      Polygon::new(rect_ring(0.0, 0.0, 1.0, 1.0), vec![]),
      Polygon::new(rect_ring(5.0, 5.0, 7.0, 7.0), vec![]),
    ]);
    let mesh = triangulate(&two);
    assert_eq!(mesh.triangle_count(), 4, "two quads → four triangles in one soup");
    assert!((mesh.area() - 5.0).abs() < 1e-9, "1 + 4 mm² of combined area");
    assert!(mesh.indices.iter().all(|&i| (i as usize) < mesh.vertices.len()), "offset indices stay in bounds");
  }

  #[test]
  fn an_empty_region_yields_an_empty_mesh() {
    let mesh = triangulate(&MultiPolygon::new(vec![]));
    assert_eq!(mesh, TriangleMesh::default());
    assert_eq!(mesh.triangle_count(), 0);
  }
}
