//! The aperture-macro (`AM`) mini-interpreter.
//!
//! An `AM` block defines a parametric shape as a sequence of primitives (circle, vector/center line, outline,
//! regular polygon, thermal) whose parameters are [`expr`](crate::expr) expressions over the macro's arguments,
//! composed by exposure: exposure-on adds (union), exposure-off subtracts (difference). This is effectively a tiny
//! interpreter, ported as one — provenance: FlatCAM's `ApertureMacro`. Resolving a macro yields a `MultiPolygon`
//! at the macro origin; the caller translates it to the flash point.
//!
//! Deferred primitive: moiré (code 6) — rare (fiducial targets); it returns [`GerberError::Unsupported`] so a file
//! using it fails loudly rather than silently dropping geometry. All other standard primitives are implemented.

use eitri_geo::{CapStyle, GeoBackend, buffer_path};
use geo_types::{Coord, LineString, MultiPolygon, Polygon};

use eitri_core::Affine;

use crate::error::{GerberError, Result};
use crate::expr::eval;
use crate::geometry::{circle_polygon, rectangle_polygon, regular_polygon, transform_multipolygon};

/// One line of a macro body: either a variable assignment (`$4=$1x0.5`) or a primitive invocation.
#[derive(Debug, Clone)]
enum MacroLine {
  Assignment { index: usize, expr: String },
  Primitive { code: u32, params: Vec<String>, line: usize },
}

/// A parsed aperture macro definition, ready to be resolved with concrete arguments.
#[derive(Debug, Clone)]
pub struct MacroDef {
  /// The macro name as declared in `%AM<name>*`.
  pub name: String,
  body: Vec<MacroLine>,
}

impl MacroDef {
  /// Parse an `AM` block body (everything between `%AM` and the closing `%`, with the name already split off).
  /// `blocks` are the `*`-terminated primitive/assignment strings following the name.
  pub fn parse(name: &str, blocks: &[&str], line: usize) -> Result<MacroDef> {
    let mut body = Vec::new();
    for raw in blocks {
      let block = raw.trim();
      if block.is_empty() {
        continue;
      }
      if let Some((lhs, rhs)) = block.split_once('=') {
        // Variable assignment: $<n>=<expr>.
        let index: usize = lhs.trim().strip_prefix('$').and_then(|s| s.parse().ok()).ok_or_else(|| {
          GerberError::Syntax { line, message: format!("malformed macro assignment '{block}'") }
        })?;
        body.push(MacroLine::Assignment { index, expr: rhs.trim().to_string() });
        continue;
      }
      let mut parts = block.split(',');
      let code: u32 = parts.next().and_then(|c| c.trim().parse().ok()).ok_or_else(|| {
        GerberError::Syntax { line, message: format!("malformed macro primitive '{block}'") }
      })?;
      if code == 0 {
        // Comment primitive — discard.
        continue;
      }
      let params = parts.map(|p| p.trim().to_string()).collect();
      body.push(MacroLine::Primitive { code, params, line });
    }
    Ok(MacroDef { name: name.to_string(), body })
  }

  /// Resolve the macro to geometry (at the macro origin) given its arguments, composing primitives by exposure.
  pub fn resolve(&self, args: &[f64], backend: &dyn GeoBackend) -> Result<MultiPolygon<f64>> {
    let mut vars: Vec<f64> = args.to_vec();
    let mut acc = MultiPolygon::new(Vec::new());

    for entry in &self.body {
      match entry {
        MacroLine::Assignment { index, expr } => {
          let value = eval(expr, &vars, 0)?;
          if *index > vars.len() {
            vars.resize(*index, 0.0);
          }
          if *index >= 1 {
            vars[*index - 1] = value;
          }
        }
        MacroLine::Primitive { code, params, line } => {
          let values = params.iter().map(|p| eval(p, &vars, *line)).collect::<Result<Vec<f64>>>()?;
          let (exposure_on, geometry) = build_primitive(*code, &values, *line, backend)?;
          if exposure_on {
            acc = union(backend, &acc, &geometry)?;
          } else {
            acc = backend.difference(&acc, &geometry)?;
          }
        }
      }
    }
    Ok(acc)
  }
}

/// Union `add` into `acc` by collecting every polygon of both and running one `union_all`.
fn union(backend: &dyn GeoBackend, acc: &MultiPolygon<f64>, add: &MultiPolygon<f64>) -> Result<MultiPolygon<f64>> {
  let polys: Vec<Polygon<f64>> = acc.0.iter().chain(add.0.iter()).cloned().collect();
  Ok(backend.union_all(&polys)?)
}

/// Build one macro primitive's geometry (before exposure composition), returning whether it is additive.
fn build_primitive(code: u32, p: &[f64], line: usize, backend: &dyn GeoBackend) -> Result<(bool, MultiPolygon<f64>)> {
  // A small helper to read a parameter with a default for the optional trailing rotation.
  let get = |i: usize, default: f64| p.get(i).copied().unwrap_or(default);
  let exposure = |v: f64| v >= 0.5;

  let (exposure_on, geometry) = match code {
    // Circle: exposure, diameter, cx, cy[, rotation].
    1 => {
      let poly = circle_polygon(get(2, 0.0), get(3, 0.0), get(1, 0.0) / 2.0);
      let rotated = rotate(&mp(poly), get(4, 0.0));
      (exposure(get(0, 1.0)), rotated)
    }
    // Vector line: exposure, width, x1, y1, x2, y2[, rotation]. A rectangle spanning the two points, butt ends.
    20 => {
      let path = LineString(vec![Coord { x: get(2, 0.0), y: get(3, 0.0) }, Coord { x: get(4, 0.0), y: get(5, 0.0) }]);
      let band = buffer_path(&path, get(1, 0.0) / 2.0, CapStyle::Butt)?;
      (exposure(get(0, 1.0)), rotate(&band, get(6, 0.0)))
    }
    // Center line: exposure, width, height, cx, cy[, rotation].
    21 => {
      let poly = rectangle_polygon(get(3, 0.0), get(4, 0.0), get(1, 0.0), get(2, 0.0));
      (exposure(get(0, 1.0)), rotate(&mp(poly), get(5, 0.0)))
    }
    // Outline: exposure, n_vertices, x0, y0, ... xn, yn[, rotation]. n+1 coordinate pairs, first repeated at end.
    4 => {
      let n = get(1, 0.0) as usize;
      let mut ring = Vec::with_capacity(n + 1);
      for v in 0..=n {
        let base = 2 + v * 2;
        ring.push(Coord { x: get(base, 0.0), y: get(base + 1, 0.0) });
      }
      let rotation = get(2 + (n + 1) * 2, 0.0);
      (exposure(get(0, 1.0)), rotate(&mp(Polygon::new(LineString(ring), Vec::new())), rotation))
    }
    // Polygon: exposure, n_vertices, cx, cy, diameter[, rotation].
    5 => {
      let poly = regular_polygon(get(2, 0.0), get(3, 0.0), get(4, 0.0), get(1, 0.0) as u32, get(5, 0.0));
      (exposure(get(0, 1.0)), mp(poly))
    }
    // Thermal: cx, cy, outer_dia, inner_dia, gap[, rotation]. Always additive; a ring cut by a cross-shaped gap.
    7 => (true, thermal(get(0, 0.0), get(1, 0.0), get(2, 0.0), get(3, 0.0), get(4, 0.0), get(5, 0.0), backend)?),
    6 => {
      return Err(GerberError::Unsupported { line, message: "moiré macro primitive (code 6) is not supported".to_string() });
    }
    other => {
      return Err(GerberError::Unsupported { line, message: format!("unknown macro primitive code {other}") });
    }
  };
  Ok((exposure_on, geometry))
}

/// Wrap a single polygon as a multipolygon.
fn mp(poly: Polygon<f64>) -> MultiPolygon<f64> {
  MultiPolygon::new(vec![poly])
}

/// Rotate geometry `degrees` counter-clockwise about the macro origin.
fn rotate(geom: &MultiPolygon<f64>, degrees: f64) -> MultiPolygon<f64> {
  if degrees.abs() < f64::EPSILON {
    return geom.clone();
  }
  transform_multipolygon(geom, &Affine::rotate(degrees.to_radians()))
}

/// A thermal-relief shape: an annulus (outer minus inner disk) with a cross-shaped gap removed, leaving four arcs.
/// Built via backend booleans so all hole windings come out correct for later unions.
fn thermal(
  cx: f64,
  cy: f64,
  outer_dia: f64,
  inner_dia: f64,
  gap: f64,
  rotation: f64,
  backend: &dyn GeoBackend,
) -> Result<MultiPolygon<f64>> {
  let outer = mp(circle_polygon(cx, cy, outer_dia / 2.0));
  let inner = mp(circle_polygon(cx, cy, inner_dia / 2.0));
  let annulus = backend.difference(&outer, &inner)?;
  // The cross gap: a horizontal and a vertical bar of width `gap` spanning the outer diameter.
  let bar_h = mp(rectangle_polygon(cx, cy, outer_dia, gap));
  let bar_v = mp(rectangle_polygon(cx, cy, gap, outer_dia));
  let cross = union(backend, &bar_h, &bar_v)?;
  let relief = backend.difference(&annulus, &cross)?;
  if rotation.abs() < f64::EPSILON {
    Ok(relief)
  } else {
    Ok(transform_multipolygon(&relief, &Affine::rotate(rotation.to_radians())))
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use eitri_geo::DefaultBackend;
  use geo::algorithm::area::Area;

  fn backend() -> DefaultBackend {
    DefaultBackend::new()
  }

  #[test]
  fn parses_and_resolves_a_simple_circle_macro() {
    // A macro that draws a circle of diameter = $1 at the origin.
    let def = MacroDef::parse("DONUT", &["1,1,$1,0,0"], 1).expect("parse");
    let geom = def.resolve(&[4.0], &backend()).expect("resolve");
    // Circle of diameter 4 -> radius 2 -> area ~= pi*4.
    assert!((geom.unsigned_area() - std::f64::consts::PI * 4.0).abs() < 0.05, "area {}", geom.unsigned_area());
  }

  #[test]
  fn exposure_off_subtracts() {
    // Outer circle dia 4 (on) then inner circle dia 2 (off) => annulus area = pi*4 - pi*1 = 3*pi.
    let def = MacroDef::parse("RING", &["1,1,4,0,0", "1,0,2,0,0"], 1).expect("parse");
    let geom = def.resolve(&[], &backend()).expect("resolve");
    assert!((geom.unsigned_area() - 3.0 * std::f64::consts::PI).abs() < 0.05, "area {}", geom.unsigned_area());
  }

  #[test]
  fn assignment_line_feeds_a_later_primitive() {
    // $2 = $1 x 2; circle diameter $2. With $1=1.5 -> diameter 3 -> radius 1.5 -> area ~ pi*2.25.
    let def = MacroDef::parse("SCALED", &["$2=$1x2", "1,1,$2,0,0"], 1).expect("parse");
    let geom = def.resolve(&[1.5], &backend()).expect("resolve");
    assert!((geom.unsigned_area() - std::f64::consts::PI * 2.25).abs() < 0.05, "area {}", geom.unsigned_area());
  }

  #[test]
  fn thermal_area_is_annulus_minus_cross() {
    // outer dia 10 (r5, disk 78.54), inner dia 6 (r3, disk 28.27) => annulus 50.27; minus the cross gap (>0).
    let def = MacroDef::parse("TH", &["7,0,0,10,6,1,0"], 1).expect("parse");
    let geom = def.resolve(&[], &backend()).expect("resolve");
    let annulus = std::f64::consts::PI * (25.0 - 9.0);
    assert!(geom.unsigned_area() < annulus && geom.unsigned_area() > annulus - 12.0, "area {}", geom.unsigned_area());
  }

  #[test]
  fn moire_is_reported_unsupported() {
    let def = MacroDef::parse("M", &["6,0,0,10,1,1,3,0.5,6,0"], 1).expect("parse");
    assert!(matches!(def.resolve(&[], &backend()), Err(GerberError::Unsupported { .. })));
  }
}
