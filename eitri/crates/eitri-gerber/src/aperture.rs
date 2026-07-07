//! The aperture model: standard apertures (circle/rectangle/obround/polygon), macro apertures, and turning an
//! aperture into geometry — a flash (place the shape at a point) or the radius used to stroke a draw.
//!
//! Provenance: FlatCAM's aperture handling in `camlib`. Standard apertures may carry an optional round hole, which
//! is subtracted from the flashed shape.

use std::collections::BTreeMap;

use eitri_geo::GeoBackend;
use geo_types::{MultiPolygon, Polygon};

use crate::error::{GerberError, Result};
use crate::geometry::{circle_polygon, obround_polygon, rectangle_polygon, regular_polygon};
use crate::macros::MacroDef;

/// A defined aperture. Dimensions are in the file's unit at definition time; the parser converts to millimetres
/// before storing, so all values here are already millimetres.
#[derive(Debug, Clone)]
pub enum Aperture {
  /// A circle of the given diameter, with an optional round hole.
  Circle { diameter: f64, hole: Option<f64> },
  /// An axis-aligned rectangle, with an optional round hole.
  Rectangle { width: f64, height: f64, hole: Option<f64> },
  /// An obround (stadium), with an optional round hole.
  Obround { width: f64, height: f64, hole: Option<f64> },
  /// A regular polygon of `vertices` sides and circumscribed `diameter`, rotated `rotation` degrees.
  Polygon { diameter: f64, vertices: u32, rotation: f64, hole: Option<f64> },
  /// A macro aperture: the macro name plus its concrete arguments, resolved to geometry at flash time.
  Macro { name: String, args: Vec<f64> },
}

impl Aperture {
  /// The radius to stroke a draw (`D01`) with, if this aperture can meaningfully stroke one. Only a circular
  /// aperture yields a well-defined stroke width; rectangle/obround/polygon/macro strokes are a deprecated, ill-
  /// defined corner of RS-274X, so they return `None` and the caller refuses loudly rather than emitting a
  /// wrong-width or empty band. (The prior code returned `0.0` for a macro, silently dropping the copper.)
  pub fn stroke_radius(&self) -> Option<f64> {
    match self {
      Aperture::Circle { diameter, .. } => Some(diameter / 2.0),
      _ => None,
    }
  }

  /// Build the geometry produced by flashing this aperture at `(cx, cy)`, subtracting any hole and resolving macro
  /// apertures against `macros`. Standard-aperture dimensions are already in millimetres; macro apertures resolve
  /// in the file unit and are scaled by `unit_scale` (mm-per-file-unit) here.
  pub fn flash(
    &self,
    cx: f64,
    cy: f64,
    macros: &BTreeMap<String, MacroDef>,
    backend: &dyn GeoBackend,
    unit_scale: f64,
    line: usize,
  ) -> Result<MultiPolygon<f64>> {
    match self {
      Aperture::Circle { diameter, hole } => {
        with_hole(circle_polygon(cx, cy, diameter / 2.0), *hole, cx, cy, backend)
      }
      Aperture::Rectangle { width, height, hole } => {
        with_hole(rectangle_polygon(cx, cy, *width, *height), *hole, cx, cy, backend)
      }
      Aperture::Obround { width, height, hole } => {
        with_hole(obround_polygon(cx, cy, *width, *height), *hole, cx, cy, backend)
      }
      Aperture::Polygon { diameter, vertices, rotation, hole } => {
        with_hole(regular_polygon(cx, cy, *diameter, *vertices, *rotation), *hole, cx, cy, backend)
      }
      Aperture::Macro { name, args } => {
        let def = macros.get(name).ok_or_else(|| GerberError::Syntax {
          line,
          message: format!("flash of undefined aperture macro '{name}'"),
        })?;
        // Macros resolve at the origin in the file unit; scale to mm, then translate to the flash point.
        let at_origin = def.resolve(args, backend)?;
        let place = eitri_core::Affine::scale(unit_scale, unit_scale).then(eitri_core::Affine::translate(cx, cy));
        Ok(crate::geometry::transform_multipolygon(&at_origin, &place))
      }
    }
  }
}

/// Subtract an optional round hole (centred at the flash point) from a flashed shape.
fn with_hole(
  shape: Polygon<f64>,
  hole: Option<f64>,
  cx: f64,
  cy: f64,
  backend: &dyn GeoBackend,
) -> Result<MultiPolygon<f64>> {
  let solid = MultiPolygon::new(vec![shape]);
  match hole {
    Some(d) if d > 0.0 => {
      let bore = MultiPolygon::new(vec![circle_polygon(cx, cy, d / 2.0)]);
      Ok(backend.difference(&solid, &bore)?)
    }
    _ => Ok(solid),
  }
}

/// Parse the body of an `AD` (aperture definition) command, e.g. `D10C,0.5` or `D12R,1X0.5X0.2`. `to_mm` converts a
/// raw dimension in the file unit into millimetres. Returns the aperture code and the parsed aperture.
pub fn parse_ad(body: &str, to_mm: impl Fn(f64) -> f64, line: usize) -> Result<(u32, Aperture)> {
  // Body looks like `D<code><Template>,<p1>X<p2>...` — split the code+template from the parameter list.
  let body = body.trim();
  let rest = body.strip_prefix('D').ok_or_else(|| GerberError::Syntax {
    line,
    message: format!("aperture definition must start with D: '{body}'"),
  })?;
  let (head, params_str) = match rest.split_once(',') {
    Some((h, p)) => (h, p),
    None => (rest, ""),
  };
  // The head is <code><template-letter-or-macro-name>; the code is the leading digits.
  let split = head.find(|c: char| !c.is_ascii_digit()).unwrap_or(head.len());
  let code: u32 = head[..split].parse().map_err(|_| GerberError::Syntax {
    line,
    message: format!("invalid aperture code in '{body}'"),
  })?;
  let template = &head[split..];
  let params: Vec<f64> = if params_str.is_empty() {
    Vec::new()
  } else {
    params_str
      .split('X')
      .map(|p| p.trim().parse::<f64>().map_err(|_| GerberError::Syntax {
        line,
        message: format!("invalid aperture parameter '{p}' in '{body}'"),
      }))
      .collect::<Result<Vec<f64>>>()?
  };

  let param = |i: usize| params.get(i).copied();
  let aperture = match template {
    "C" => Aperture::Circle {
      diameter: to_mm(param(0).ok_or_else(|| missing(line, "circle diameter"))?),
      hole: param(1).map(&to_mm),
    },
    "R" => Aperture::Rectangle {
      width: to_mm(param(0).ok_or_else(|| missing(line, "rectangle width"))?),
      height: to_mm(param(1).ok_or_else(|| missing(line, "rectangle height"))?),
      hole: param(2).map(&to_mm),
    },
    "O" => Aperture::Obround {
      width: to_mm(param(0).ok_or_else(|| missing(line, "obround width"))?),
      height: to_mm(param(1).ok_or_else(|| missing(line, "obround height"))?),
      hole: param(2).map(&to_mm),
    },
    "P" => Aperture::Polygon {
      diameter: to_mm(param(0).ok_or_else(|| missing(line, "polygon diameter"))?),
      vertices: param(1).ok_or_else(|| missing(line, "polygon vertex count"))? as u32,
      rotation: param(2).unwrap_or(0.0),
      hole: param(3).map(&to_mm),
    },
    // Anything else is a macro-aperture reference: the template is the macro name, params are its arguments. These
    // stay in the file unit — the macro resolves in file units and its geometry is scaled to mm at flash time.
    name => Aperture::Macro { name: name.to_string(), args: params.clone() },
  };
  Ok((code, aperture))
}

fn missing(line: usize, what: &str) -> GerberError {
  GerberError::Syntax { line, message: format!("aperture definition missing {what}") }
}

#[cfg(test)]
mod tests {
  use super::*;
  use eitri_geo::DefaultBackend;
  use geo::algorithm::area::Area;

  fn mm(v: f64) -> f64 {
    v
  }

  #[test]
  fn parse_circle_with_hole() {
    let (code, ap) = parse_ad("D10C,0.5X0.2", mm, 1).expect("ad");
    assert_eq!(code, 10);
    match ap {
      Aperture::Circle { diameter, hole } => {
        assert!((diameter - 0.5).abs() < 1e-9);
        assert_eq!(hole, Some(0.2));
      }
      other => panic!("expected circle, got {other:?}"),
    }
  }

  #[test]
  fn parse_rectangle_and_polygon() {
    let (_, r) = parse_ad("D11R,1X0.5", mm, 1).expect("ad");
    assert!(matches!(r, Aperture::Rectangle { width, height, .. } if (width - 1.0).abs() < 1e-9 && (height - 0.5).abs() < 1e-9));
    let (_, p) = parse_ad("D12P,2X6X0", mm, 1).expect("ad");
    assert!(matches!(p, Aperture::Polygon { vertices: 6, .. }));
  }

  #[test]
  fn parse_macro_reference() {
    let (code, ap) = parse_ad("D20THERMAL,1.5X0.8", mm, 1).expect("ad");
    assert_eq!(code, 20);
    assert!(matches!(ap, Aperture::Macro { ref name, .. } if name == "THERMAL"));
  }

  #[test]
  fn flash_circle_area() {
    let macros = BTreeMap::new();
    let backend = DefaultBackend::new();
    let ap = Aperture::Circle { diameter: 4.0, hole: None };
    let g = ap.flash(1.0, 1.0, &macros, &backend, 1.0, 1).expect("flash");
    assert!((g.unsigned_area() - std::f64::consts::PI * 4.0).abs() < 0.05);
  }

  #[test]
  fn flash_circle_with_hole_area() {
    let macros = BTreeMap::new();
    let backend = DefaultBackend::new();
    let ap = Aperture::Circle { diameter: 4.0, hole: Some(2.0) };
    let g = ap.flash(0.0, 0.0, &macros, &backend, 1.0, 1).expect("flash");
    // Disk r2 minus hole r1 = pi*(4-1) = 3pi.
    assert!((g.unsigned_area() - 3.0 * std::f64::consts::PI).abs() < 0.05, "area {}", g.unsigned_area());
  }
}
