//! DXF import via the pure-Rust `dxf` crate.
//!
//! Reads the vector entities FlatCAM's DXF import cares about — `LINE`, `LWPOLYLINE`, `POLYLINE`, `ARC`, `CIRCLE`
//! — and converts them to CAD millimetre geometry (plan §6). Polyline **bulges** and `ARC`/`CIRCLE` curvature are
//! flattened with the shared [`eitri_geo`] flatteners so DXF arcs match native precision. Anything not supported
//! (`SPLINE`, polygon/polyface meshes, ellipses, text, inserts, ...) is **skipped loudly** — recorded in
//! [`DxfImport::skipped`], never silently dropped.
//!
//! DXF has no unit-normalization step, so coordinates are taken as-is; a caller that knows the drawing's units can
//! rescale the recovered geometry through `eitri-geo` afterward.

use std::f64::consts::TAU;
use std::io::Read;

use dxf::Drawing;
use dxf::entities::EntityType;
use geo_types::Coord;

use eitri_core::{Error, Result};
use eitri_geo::{flatten_arc, flatten_bulge};

use crate::diagnostic::Skipped;
use crate::geometry::ImportedGeometry;

/// The result of importing a DXF drawing: recovered geometry plus loud diagnostics for skipped entities.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct DxfImport {
  /// The recovered vector geometry.
  pub geometry: ImportedGeometry,
  /// Entities that were not converted, with the reason.
  pub skipped: Vec<Skipped>,
}

/// Import a DXF drawing from an in-memory string. Parse failures surface as [`Error::Parse`].
pub fn import_dxf(src: &str) -> Result<DxfImport> {
  let drawing = Drawing::load(&mut src.as_bytes()).map_err(|e| Error::Parse(format!("DXF parse: {e}")))?;
  Ok(convert(&drawing))
}

/// Import a DXF drawing from any reader (a file, a buffer). Parse failures surface as [`Error::Parse`].
pub fn import_dxf_reader<R: Read>(mut reader: R) -> Result<DxfImport> {
  let drawing = Drawing::load(&mut reader).map_err(|e| Error::Parse(format!("DXF parse: {e}")))?;
  Ok(convert(&drawing))
}

/// Walk every entity in a loaded drawing, converting the supported ones and recording the rest.
fn convert(drawing: &Drawing) -> DxfImport {
  let mut import = DxfImport::default();
  for entity in drawing.entities() {
    match &entity.specific {
      EntityType::Line(line) => {
        let a = Coord { x: line.p1.x, y: line.p1.y };
        let b = Coord { x: line.p2.x, y: line.p2.y };
        import.geometry.push_subpath(vec![a, b], false);
      }
      EntityType::Circle(circle) => {
        let pts = circle_points(circle.center.x, circle.center.y, circle.radius);
        import.geometry.push_subpath(pts, true);
      }
      EntityType::Arc(arc) => {
        let pts = arc_points(arc.center.x, arc.center.y, arc.radius, arc.start_angle, arc.end_angle);
        import.geometry.push_subpath(pts, false);
      }
      EntityType::LwPolyline(lw) => {
        let verts: Vec<(f64, f64, f64)> = lw.vertices.iter().map(|v| (v.x, v.y, v.bulge)).collect();
        push_bulge_polyline(&verts, lw.is_closed(), &mut import.geometry);
      }
      EntityType::Polyline(pl) => {
        if pl.is_3d_polygon_mesh() || pl.is_polyface_mesh() {
          import.skipped.push(Skipped::new("DXF POLYLINE mesh", "polygon/polyface meshes are not a 2D contour"));
          continue;
        }
        let verts: Vec<(f64, f64, f64)> =
          pl.vertices().map(|v| (v.location.x, v.location.y, v.bulge)).collect();
        push_bulge_polyline(&verts, pl.is_closed(), &mut import.geometry);
      }
      EntityType::Spline(_) => {
        import.skipped.push(Skipped::new("DXF SPLINE entity", "spline import is not yet supported"));
      }
      other => {
        import.skipped.push(Skipped::new(format!("DXF {} entity", entity_kind(other)), "unsupported entity type"));
      }
    }
  }
  import
}

/// Flatten a closed circle into a ring of chord points at the shared tolerance.
fn circle_points(cx: f64, cy: f64, r: f64) -> Vec<Coord<f64>> {
  let mut pts = vec![Coord { x: cx + r, y: cy }];
  pts.extend(flatten_arc(cx, cy, r, 0.0, TAU));
  pts
}

/// Flatten a DXF `ARC` (angles in **degrees**, always swept counter-clockwise from start to end) into chord points.
fn arc_points(cx: f64, cy: f64, r: f64, start_deg: f64, end_deg: f64) -> Vec<Coord<f64>> {
  let start = start_deg.to_radians();
  // DXF arcs sweep CCW from start to end; normalize the positive sweep, treating a coincident end as a full turn.
  let mut sweep = (end_deg - start_deg).to_radians().rem_euclid(TAU);
  if sweep <= f64::EPSILON {
    sweep = TAU;
  }
  let mut pts = vec![Coord { x: cx + r * start.cos(), y: cy + r * start.sin() }];
  pts.extend(flatten_arc(cx, cy, r, start, sweep));
  pts
}

/// Build a (possibly closed) polyline from `(x, y, bulge)` vertices, flattening each bulged segment. The bulge on a
/// vertex describes the arc leaving it toward the next; a closed polyline also arcs from the last vertex back to
/// the first using the last vertex's bulge.
fn push_bulge_polyline(verts: &[(f64, f64, f64)], closed: bool, geo: &mut ImportedGeometry) {
  if verts.is_empty() {
    return;
  }
  let mut pts = vec![Coord { x: verts[0].0, y: verts[0].1 }];
  for pair in verts.windows(2) {
    let (x0, y0, bulge) = pair[0];
    let (x1, y1, _) = pair[1];
    pts.extend(flatten_bulge(x0, y0, x1, y1, bulge));
  }
  if closed && verts.len() >= 2 {
    let (x0, y0, bulge) = verts[verts.len() - 1];
    let (x1, y1, _) = verts[0];
    pts.extend(flatten_bulge(x0, y0, x1, y1, bulge));
  }
  geo.push_subpath(pts, closed);
}

/// A short human name for an unsupported entity, for the skip diagnostic.
fn entity_kind(kind: &EntityType) -> &'static str {
  match kind {
    EntityType::Ellipse(_) => "ELLIPSE",
    EntityType::Text(_) => "TEXT",
    EntityType::MText(_) => "MTEXT",
    EntityType::Insert(_) => "INSERT",
    EntityType::Solid(_) => "SOLID",
    EntityType::Face3D(_) => "3DFACE",
    EntityType::ModelPoint(_) => "POINT",
    _ => "unsupported",
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  /// A minimal ASCII DXF wrapping the given ENTITIES body in the required section framing.
  fn dxf_with_entities(body: &str) -> String {
    format!("0\nSECTION\n2\nENTITIES\n{body}0\nENDSEC\n0\nEOF\n")
  }

  #[test]
  fn imports_a_line_as_a_polyline() {
    let body = "0\nLINE\n8\n0\n10\n0.0\n20\n0.0\n11\n10.0\n21\n5.0\n";
    let import = import_dxf(&dxf_with_entities(body)).expect("import");
    assert_eq!(import.geometry.polylines.len(), 1);
    let pts: Vec<(f64, f64)> = import.geometry.polylines[0].0.iter().map(|c| (c.x, c.y)).collect();
    assert_eq!(pts, vec![(0.0, 0.0), (10.0, 5.0)]);
  }

  #[test]
  fn imports_a_circle_as_a_closed_polygon_on_the_circle() {
    let body = "0\nCIRCLE\n8\n0\n10\n2.0\n20\n3.0\n40\n5.0\n";
    let import = import_dxf(&dxf_with_entities(body)).expect("import");
    assert_eq!(import.geometry.polygons.len(), 1);
    for c in &import.geometry.polygons[0].exterior().0 {
      let r = (c.x - 2.0).hypot(c.y - 3.0);
      assert!((r - 5.0).abs() < 1e-6, "vertex off circle: radius {r}");
    }
  }

  #[test]
  fn imports_an_arc_hugging_the_circle_and_open() {
    // A 90-degree arc, radius 4, centred at origin, from 0deg to 90deg.
    let body = "0\nARC\n8\n0\n10\n0.0\n20\n0.0\n40\n4.0\n50\n0.0\n51\n90.0\n";
    let import = import_dxf(&dxf_with_entities(body)).expect("import");
    assert_eq!(import.geometry.polylines.len(), 1, "an arc is an open path");
    let line = &import.geometry.polylines[0];
    for c in &line.0 {
      assert!((c.x.hypot(c.y) - 4.0).abs() < 1e-6, "arc point off circle");
    }
    let end = line.0.last().copied().expect("point");
    assert!((end.x - 0.0).abs() < 1e-6 && (end.y - 4.0).abs() < 1e-6, "arc end {end:?}");
  }

  #[test]
  fn imports_an_lwpolyline_with_a_bulge() {
    // Three vertices; the first segment carries a bulge (quarter arc), the rest straight; closed.
    let b = (std::f64::consts::FRAC_PI_8).tan();
    let body = format!(
      "0\nLWPOLYLINE\n8\n0\n90\n3\n70\n1\n10\n1.0\n20\n0.0\n42\n{b}\n10\n0.0\n20\n1.0\n10\n0.0\n20\n0.0\n"
    );
    let import = import_dxf(&dxf_with_entities(&body)).expect("import");
    assert_eq!(import.geometry.polygons.len(), 1, "flag 70=1 => closed => polygon");
    // The bulged first segment should have produced several intermediate points, not a single chord.
    assert!(import.geometry.polygons[0].exterior().0.len() > 6, "bulge should flatten into an arc");
  }

  #[test]
  fn spline_is_skipped_loudly() {
    let body = "0\nSPLINE\n8\n0\n10\n0.0\n20\n0.0\n10\n1.0\n20\n1.0\n";
    let import = import_dxf(&dxf_with_entities(body)).expect("import");
    assert!(import.geometry.is_empty());
    assert_eq!(import.skipped.len(), 1);
    assert!(import.skipped[0].what.contains("SPLINE"), "got {:?}", import.skipped);
  }

  #[test]
  fn reader_entry_point_matches_the_string_one() {
    let body = "0\nLINE\n8\n0\n10\n0.0\n20\n0.0\n11\n2.0\n21\n0.0\n";
    let text = dxf_with_entities(body);
    let a = import_dxf(&text).expect("str");
    let b = import_dxf_reader(text.as_bytes()).expect("reader");
    assert_eq!(a.geometry, b.geometry);
  }
}
