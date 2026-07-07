//! SVG import via `usvg`.
//!
//! `usvg` does the heavy lifting the plan (§6) calls for: it normalizes the SVG tree, resolves every ancestor
//! transform, applies the viewBox, and flattens styling — leaving each path as local geometry plus a resolved
//! absolute transform. This module walks that tree, maps each path point into absolute space via the path's
//! `abs_transform`, then into CAD millimetre space (an optional Y-flip and a user-units→mm scale), flattening
//! quadratic/cubic Béziers with the shared [`eitri_geo`] flatteners so imported precision matches native geometry.
//!
//! **Coordinate conventions** (the §6 porting risk). usvg emits Y-down canvas coordinates in the SVG's resolved
//! user-unit space; CAD wants Y-up millimetres. [`SvgOptions::flip_y`] reflects about the canvas height (the
//! default, matching FlatCAM's SVG import) and [`SvgOptions::scale`] treats one user unit as `scale` millimetres
//! (default 1.0). Closed subpaths become polygons, open subpaths polylines. Raster `<image>` and `<text>` nodes
//! carry no vector geometry and are skipped loudly.

use usvg::tiny_skia_path::PathSegment;
use usvg::{Node, Options, Transform, Tree};

use eitri_core::{Error, Result};
use eitri_geo::{flatten_cubic, flatten_quad};
use geo_types::Coord;

use crate::diagnostic::Skipped;
use crate::geometry::ImportedGeometry;

/// How SVG coordinates are mapped into CAD millimetre space.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SvgOptions {
  /// Millimetres per SVG user unit. usvg has already resolved the viewBox, so this is a final uniform scale; the
  /// default of `1.0` treats one resolved user unit as one millimetre (FlatCAM's convention).
  pub scale: f64,
  /// Reflect geometry about the canvas height so SVG's Y-down origin becomes CAD's Y-up. Default `true`.
  pub flip_y: bool,
  /// Rendering DPI handed to usvg; only affects physical-unit (`mm`/`in`) resolution in the source. Default `96.0`.
  pub dpi: f32,
}

impl Default for SvgOptions {
  fn default() -> SvgOptions {
    SvgOptions { scale: 1.0, flip_y: true, dpi: 96.0 }
  }
}

/// The result of importing an SVG: recovered geometry plus loud diagnostics for skipped nodes.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SvgImport {
  /// The recovered vector geometry, in millimetres.
  pub geometry: ImportedGeometry,
  /// Nodes that were not converted (raster images, text), with the reason.
  pub skipped: Vec<Skipped>,
}

/// Import an SVG document into flattened CAD geometry. Parse failures surface as [`Error::Parse`].
pub fn import_svg(src: &str, opts: &SvgOptions) -> Result<SvgImport> {
  let options = Options { dpi: opts.dpi, ..Default::default() };
  let tree = Tree::from_str(src, &options).map_err(|e| Error::Parse(format!("SVG parse: {e}")))?;
  let height = tree.size().height() as f64;

  let mut import = SvgImport::default();
  walk(tree.root().children(), opts, height, &mut import);
  Ok(import)
}

/// Recurse the resolved node tree. Group transforms are already folded into each path's `abs_transform`, so the
/// walk only needs to descend groups and convert paths; images and text are reported and skipped.
fn walk(nodes: &[Node], opts: &SvgOptions, height: f64, import: &mut SvgImport) {
  for node in nodes {
    match node {
      Node::Group(g) => walk(g.children(), opts, height, import),
      Node::Path(p) => convert_path(p.data(), &p.abs_transform(), opts, height, &mut import.geometry),
      Node::Image(_) => import.skipped.push(Skipped::new("SVG <image> node", "raster image, not vector geometry")),
      Node::Text(_) => import.skipped.push(Skipped::new("SVG <text> node", "text is not imported as geometry")),
    }
  }
}

/// Convert one path's segments into subpaths, mapping every point into CAD millimetre space and flattening curves.
fn convert_path(path: &usvg::tiny_skia_path::Path, ts: &Transform, opts: &SvgOptions, height: f64, geo: &mut ImportedGeometry) {
  // Map an SVG local point through the resolved absolute transform, then into CAD millimetre space.
  let map = |x: f32, y: f32| -> Coord<f64> {
    let ax = (ts.sx * x + ts.kx * y + ts.tx) as f64;
    let ay = (ts.ky * x + ts.sy * y + ts.ty) as f64;
    let cy = if opts.flip_y { height - ay } else { ay };
    Coord { x: ax * opts.scale, y: cy * opts.scale }
  };

  let mut current: Vec<Coord<f64>> = Vec::new();
  let mut cursor = Coord { x: 0.0, y: 0.0 };
  for seg in path.segments() {
    match seg {
      PathSegment::MoveTo(p) => {
        // A new subpath: flush the previous one as open (a Close would have already flushed it as closed).
        flush(&mut current, false, geo);
        cursor = map(p.x, p.y);
        current.push(cursor);
      }
      PathSegment::LineTo(p) => {
        if current.is_empty() {
          current.push(cursor);
        }
        cursor = map(p.x, p.y);
        current.push(cursor);
      }
      PathSegment::QuadTo(c, p) => {
        if current.is_empty() {
          current.push(cursor);
        }
        let cm = map(c.x, c.y);
        let pm = map(p.x, p.y);
        current.extend(flatten_quad(cursor, cm, pm));
        cursor = pm;
      }
      PathSegment::CubicTo(c1, c2, p) => {
        if current.is_empty() {
          current.push(cursor);
        }
        let c1m = map(c1.x, c1.y);
        let c2m = map(c2.x, c2.y);
        let pm = map(p.x, p.y);
        current.extend(flatten_cubic(cursor, c1m, c2m, pm));
        cursor = pm;
      }
      PathSegment::Close => flush(&mut current, true, geo),
    }
  }
  // A trailing open subpath with no closing Close.
  flush(&mut current, false, geo);
}

/// Push the accumulated subpath (if any) into `geo` and clear it. `closed` selects polygon vs polyline.
fn flush(current: &mut Vec<Coord<f64>>, closed: bool, geo: &mut ImportedGeometry) {
  if current.is_empty() {
    return;
  }
  geo.push_subpath(std::mem::take(current), closed);
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn imports_a_line_as_a_polyline() {
    let svg = r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100" width="100" height="100">
      <path d="M 10 20 L 30 20" fill="none" stroke="black"/></svg>"#;
    // Disable the flip to isolate the pure coordinate math for this assertion.
    let import = import_svg(svg, &SvgOptions { flip_y: false, ..Default::default() }).expect("import");
    assert_eq!(import.geometry.polylines.len(), 1);
    let pts: Vec<(f64, f64)> = import.geometry.polylines[0].0.iter().map(|c| (c.x, c.y)).collect();
    assert_eq!(pts, vec![(10.0, 20.0), (30.0, 20.0)]);
  }

  #[test]
  fn closed_rectangle_becomes_a_polygon() {
    let svg = r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100" width="100" height="100">
      <rect x="10" y="10" width="20" height="30" fill="black"/></svg>"#;
    let import = import_svg(svg, &SvgOptions::default()).expect("import");
    assert_eq!(import.geometry.polygons.len(), 1, "a rect is a closed shape");
    assert!(import.geometry.polylines.is_empty());
  }

  #[test]
  fn a_bezier_is_flattened_to_the_shared_tolerance() {
    let svg = r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100" width="100" height="100">
      <path d="M 0 50 C 0 0 100 0 100 50" fill="none" stroke="black"/></svg>"#;
    let import = import_svg(svg, &SvgOptions { flip_y: false, ..Default::default() }).expect("import");
    let poly = &import.geometry.polylines[0];
    assert!(poly.0.len() > 4, "a cubic must subdivide into several points, got {}", poly.0.len());
    assert_eq!(poly.0.first().map(|c| (c.x, c.y)), Some((0.0, 50.0)));
    assert_eq!(poly.0.last().map(|c| (c.x, c.y)), Some((100.0, 50.0)));
  }

  #[test]
  fn viewbox_scaling_and_transform_land_the_element_correctly() {
    // The viewBox is 0..50 but the viewport is 100x100, so usvg resolves a 2x scale. A <g translate(5,5)> shifts
    // the rect before that scale. A 10x10 rect at (0,0) under translate(5,5) then 2x becomes a 20x20 box at
    // (10,10). flip_y off so the assertion is pure.
    let svg = r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 50 50" width="100" height="100">
      <g transform="translate(5,5)"><rect x="0" y="0" width="10" height="10" fill="black"/></g></svg>"#;
    let import = import_svg(svg, &SvgOptions { flip_y: false, ..Default::default() }).expect("import");
    let b = import.geometry.bounds().expect("non-empty");
    assert!((b.min().x - 10.0).abs() < 1e-4 && (b.min().y - 10.0).abs() < 1e-4, "min {:?}", b.min());
    assert!((b.max().x - 30.0).abs() < 1e-4 && (b.max().y - 30.0).abs() < 1e-4, "max {:?}", b.max());
  }

  #[test]
  fn flip_y_reflects_about_the_canvas_height() {
    let svg = r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100" width="100" height="100">
      <path d="M 0 10 L 0 10" fill="none" stroke="black"/><rect x="0" y="10" width="5" height="5" fill="black"/></svg>"#;
    // With flip, a point at y=10 on a 100-tall canvas maps to y=90.
    let import = import_svg(svg, &SvgOptions::default()).expect("import");
    let b = import.geometry.bounds().expect("non-empty");
    assert!((b.max().y - 90.0).abs() < 1e-4, "flipped top edge should be at y=90, got {:?}", b.max());
  }

  #[test]
  fn scale_multiplies_final_coordinates() {
    let svg = r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100" width="100" height="100">
      <path d="M 0 0 L 10 0" fill="none" stroke="black"/></svg>"#;
    let import = import_svg(svg, &SvgOptions { flip_y: false, scale: 25.4, ..Default::default() }).expect("import");
    let end = import.geometry.polylines[0].0.last().copied().expect("point");
    assert!((end.x - 254.0).abs() < 1e-3, "10 units * 25.4 = 254 mm, got {}", end.x);
  }

  #[test]
  fn image_node_is_skipped_loudly() {
    // A valid 1x1 PNG data URI so usvg keeps the <image> node (an undecodable one is dropped before we see it).
    let png = "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNkYPhfDwAChwGA60e6kgAAAABJRU5ErkJggg==";
    let svg = format!(
      r#"<svg xmlns="http://www.w3.org/2000/svg" width="10" height="10">
      <image x="0" y="0" width="10" height="10" href="data:image/png;base64,{png}"/></svg>"#
    );
    let import = import_svg(&svg, &SvgOptions::default()).expect("import");
    assert!(import.geometry.is_empty());
    assert_eq!(import.skipped.len(), 1, "the image is reported, not silently dropped");
    assert!(import.skipped[0].what.contains("image"));
  }
}
