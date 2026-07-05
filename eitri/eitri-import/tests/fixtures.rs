//! Fixture-backed import tests. The synthetic SVG/DXF files live under `eitri/fixtures/synthetic/{svg,dxf}` and are
//! loaded from disk (like the gerber/excellon fixtures) so a hand-authored spec corner is exercised end to end.

use std::path::PathBuf;

use eitri_import::{import_dxf, import_svg, SvgOptions};

/// Resolve a path under the shared `eitri/fixtures/` tree, which sits one level above this crate.
fn fixture(rel: &str) -> PathBuf {
  PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..").join("fixtures").join(rel)
}

#[test]
fn svg_shapes_fixture_imports_all_four_shapes() {
  let src = std::fs::read_to_string(fixture("synthetic/svg/shapes.svg")).expect("read svg fixture");
  let import = import_svg(&src, &SvgOptions::default()).expect("import svg");
  // A line + a cubic + an arc (open polylines) and a rect (closed polygon) — four shapes, nothing skipped.
  assert_eq!(import.geometry.polygons.len(), 1, "the rect is the one closed shape");
  assert_eq!(import.geometry.polylines.len(), 3, "line + cubic + arc are open paths");
  assert!(import.skipped.is_empty(), "no node should be skipped: {:?}", import.skipped);
}

#[test]
fn svg_fixture_resolves_the_group_transform_and_viewbox_scale() {
  // The line runs x=0..10 under translate(5,5) then a 2x viewBox scale => absolute x spans 10..30. flip_y off to
  // keep the assertion on the raw resolved coordinates.
  let src = std::fs::read_to_string(fixture("synthetic/svg/shapes.svg")).expect("read svg fixture");
  let import = import_svg(&src, &SvgOptions { flip_y: false, ..Default::default() }).expect("import svg");
  let line = import
    .geometry
    .polylines
    .iter()
    .find(|l| l.0.len() == 2)
    .expect("the straight line survives as a two-point polyline");
  let xs: Vec<f64> = line.0.iter().map(|c| c.x).collect();
  assert!((xs[0] - 10.0).abs() < 1e-4 && (xs[1] - 30.0).abs() < 1e-4, "line x endpoints {xs:?}");
}

#[test]
fn dxf_entities_fixture_imports_line_circle_arc_and_closed_lwpolyline() {
  let src = std::fs::read_to_string(fixture("synthetic/dxf/entities.dxf")).expect("read dxf fixture");
  let import = import_dxf(&src).expect("import dxf");
  // CIRCLE + the closed LWPOLYLINE are polygons; LINE + ARC are open polylines. Nothing unsupported here.
  assert_eq!(import.geometry.polygons.len(), 2, "circle + closed lwpolyline");
  assert_eq!(import.geometry.polylines.len(), 2, "line + arc");
  assert!(import.skipped.is_empty(), "no entity should be skipped: {:?}", import.skipped);
}
