//! Film generation — a positive or negative film of copper as a self-contained SVG.
//!
//! Provenance: FlatCAM's film tool (see `docs/eitri-porting-plan.md` §7.8). Film is **vector output, not a
//! toolpath**, and off the spindle-milling critical path, so this is a deliberately minimal writer: a positive film
//! renders the copper filled; a negative renders its inverse within a bordered frame. Scale and a horizontal mirror
//! (for emulsion-side-down exposure) and a border are supported. The full SVG *import* side (reading SVG into
//! geometry) is a separate, later concern — Phase 6's `eitri-import`; this only *writes*.
//!
//! Holes are drawn with the SVG even-odd fill rule, and the Y axis is flipped on output so the film reads
//! right-way-up (SVG's Y grows downward, geometry's upward).

use std::fmt::Write as _;

use geo_types::{Coord, LineString, MultiPolygon, Polygon};

use eitri_core::{Affine, Error, Result};
use eitri_geo::{GeoBackend, bounds};

/// Which film to render.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilmKind {
  /// Copper rendered filled (a positive).
  Positive,
  /// The inverse of the copper rendered filled within the border (a negative).
  Negative,
}

/// Parameters for a film export.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FilmParams {
  /// Positive or negative.
  pub kind: FilmKind,
  /// Uniform scale factor applied to the geometry (must be positive).
  pub scale: f64,
  /// Mirror horizontally (reflect about the vertical centre) — the emulsion-side-down flip.
  pub mirror: bool,
  /// Border around the artwork (millimetres, `>= 0`); also the frame the negative is cut from.
  pub border: f64,
}

impl Default for FilmParams {
  fn default() -> FilmParams {
    FilmParams { kind: FilmKind::Positive, scale: 1.0, mirror: false, border: 5.0 }
  }
}

impl FilmParams {
  fn validate(&self) -> Result<()> {
    if self.scale.is_nan() || self.scale <= 0.0 {
      return Err(Error::InvalidGeometry("film scale must be positive".to_string()));
    }
    if self.border.is_nan() || self.border < 0.0 {
      return Err(Error::InvalidGeometry("film border must be non-negative".to_string()));
    }
    Ok(())
  }
}

/// Render `copper` to a self-contained SVG film per `params`. A negative uses `backend` to subtract the copper from
/// its bordered frame. Empty copper yields a minimal empty SVG.
pub fn film_svg<B>(copper: &MultiPolygon<f64>, params: &FilmParams, backend: &B) -> Result<String>
where
  B: GeoBackend,
{
  params.validate()?;

  // Apply scale, then the optional horizontal mirror about the scaled artwork's centre.
  let scaled = apply(copper, Affine::scale(params.scale, params.scale));
  let Some((mut x0, mut y0, mut x1, mut y1)) = bounds(&scaled) else {
    return Ok(empty_svg());
  };
  let transformed = if params.mirror {
    let cx = (x0 + x1) / 2.0;
    apply(&scaled, Affine::mirror_about_line(cx, 0.0, std::f64::consts::FRAC_PI_2))
  } else {
    scaled
  };
  // Re-derive bounds after the mirror (it preserves them, but keep this robust to future transforms).
  if let Some(b) = bounds(&transformed) {
    (x0, y0, x1, y1) = b;
  }

  // Expand by the border to get the frame / viewBox extent.
  let (bx0, by0, bx1, by1) = (x0 - params.border, y0 - params.border, x1 + params.border, y1 + params.border);
  let (width, height) = (bx1 - bx0, by1 - by0);

  // The geometry actually drawn: copper for a positive, frame - copper for a negative.
  let drawn = match params.kind {
    FilmKind::Positive => transformed,
    FilmKind::Negative => {
      let frame = MultiPolygon::new(vec![rectangle(bx0, by0, bx1, by1)]);
      backend.difference(&frame, &transformed)?
    }
  };

  Ok(render_svg(&drawn, bx0, by1, width, height))
}

/// Assemble the SVG document from the drawn geometry. Coordinates are mapped into the viewBox with the Y axis
/// flipped (`Y = by1 - y`) so the film is right-way-up.
fn render_svg(drawn: &MultiPolygon<f64>, bx0: f64, by1: f64, width: f64, height: f64) -> String {
  let mut svg = String::new();
  let _ = writeln!(
    svg,
    "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{width:.4}mm\" height=\"{height:.4}mm\" \
viewBox=\"0 0 {width:.4} {height:.4}\">"
  );
  let mut d = String::new();
  for poly in &drawn.0 {
    append_ring(&mut d, poly.exterior(), bx0, by1);
    for hole in poly.interiors() {
      append_ring(&mut d, hole, bx0, by1);
    }
  }
  if !d.is_empty() {
    let _ = writeln!(svg, "  <path d=\"{}\" fill=\"black\" fill-rule=\"evenodd\"/>", d.trim_end());
  }
  svg.push_str("</svg>\n");
  svg
}

/// Append one ring to an SVG path data string: `M` to the first vertex, `L` to the rest, `Z` to close.
fn append_ring(d: &mut String, ring: &LineString<f64>, bx0: f64, by1: f64) {
  let mut coords = ring.0.iter();
  let Some(first) = coords.next() else {
    return;
  };
  let _ = write!(d, "M {:.4} {:.4} ", first.x - bx0, by1 - first.y);
  for c in coords {
    let _ = write!(d, "L {:.4} {:.4} ", c.x - bx0, by1 - c.y);
  }
  d.push_str("Z ");
}

/// A minimal empty SVG for empty input.
fn empty_svg() -> String {
  "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"0mm\" height=\"0mm\" viewBox=\"0 0 0 0\"></svg>\n".to_string()
}

/// Apply an affine transform to every vertex of a multipolygon.
fn apply(mp: &MultiPolygon<f64>, transform: Affine) -> MultiPolygon<f64> {
  eitri_geo::apply_affine(mp, transform)
}

/// An axis-aligned rectangle polygon with a CCW exterior ring.
fn rectangle(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon<f64> {
  Polygon::new(
    LineString(vec![
      Coord { x: x0, y: y0 },
      Coord { x: x1, y: y0 },
      Coord { x: x1, y: y1 },
      Coord { x: x0, y: y1 },
      Coord { x: x0, y: y0 },
    ]),
    vec![],
  )
}

#[cfg(test)]
mod tests {
  use super::*;
  use eitri_geo::DefaultBackend;

  fn square(cx: f64, cy: f64, half: f64) -> Polygon<f64> {
    rectangle(cx - half, cy - half, cx + half, cy + half)
  }

  /// Extract the viewBox width from an SVG string produced by [`render_svg`].
  fn viewbox_width(svg: &str) -> f64 {
    let tag = svg.split("viewBox=\"").nth(1).expect("viewBox present");
    let inner = tag.split('"').next().expect("viewBox value");
    inner.split_whitespace().nth(2).expect("width field").parse().expect("width parses")
  }

  fn backend() -> DefaultBackend {
    DefaultBackend::new()
  }

  #[test]
  fn positive_film_is_a_well_formed_svg_with_a_filled_path() {
    let copper = MultiPolygon::new(vec![square(0.0, 0.0, 5.0)]);
    let svg = film_svg(&copper, &FilmParams::default(), &backend()).expect("film");
    assert!(svg.starts_with("<svg"), "svg header");
    assert!(svg.trim_end().ends_with("</svg>"), "svg closes");
    assert!(svg.contains("<path"), "the copper is drawn as a path");
    assert!(svg.contains("fill-rule=\"evenodd\""), "even-odd fill for holes");
    // A 10mm square plus a 5mm border on each side => 20mm viewBox width.
    assert!((viewbox_width(&svg) - 20.0).abs() < 1e-3, "width {}", viewbox_width(&svg));
  }

  #[test]
  fn scale_enlarges_the_viewbox_artwork() {
    let copper = MultiPolygon::new(vec![square(0.0, 0.0, 5.0)]);
    let params1 = FilmParams { scale: 1.0, border: 2.0, ..Default::default() };
    let params2 = FilmParams { scale: 2.0, border: 2.0, ..Default::default() };
    let w1 = viewbox_width(&film_svg(&copper, &params1, &backend()).expect("f1"));
    let w2 = viewbox_width(&film_svg(&copper, &params2, &backend()).expect("f2"));
    // Border is constant, so the artwork span (width minus 2*border) must double.
    assert!(((w2 - 4.0) - 2.0 * (w1 - 4.0)).abs() < 1e-3, "scaled artwork should double: w1={w1}, w2={w2}");
  }

  #[test]
  fn negative_film_draws_the_inverse_within_the_frame() {
    let copper = MultiPolygon::new(vec![square(0.0, 0.0, 5.0)]);
    let params = FilmParams { kind: FilmKind::Negative, ..Default::default() };
    let svg = film_svg(&copper, &params, &backend()).expect("negative");
    assert!(svg.contains("<path"), "the inverse is drawn");
    // The negative's path includes the outer frame, so it reaches the viewBox corner (0,0) — the copper positive
    // (inset by the border) never does.
    assert!(svg.contains("M 0.0000 0.0000"), "the frame reaches the film corner");
  }

  #[test]
  fn mirror_changes_the_output_but_not_the_extent() {
    // An asymmetric L so the mirror is observable, not a no-op.
    let l = Polygon::new(
      LineString(vec![
        Coord { x: 0.0, y: 0.0 },
        Coord { x: 6.0, y: 0.0 },
        Coord { x: 6.0, y: 2.0 },
        Coord { x: 2.0, y: 2.0 },
        Coord { x: 2.0, y: 6.0 },
        Coord { x: 0.0, y: 6.0 },
        Coord { x: 0.0, y: 0.0 },
      ]),
      vec![],
    );
    let copper = MultiPolygon::new(vec![l]);
    let plain = film_svg(&copper, &FilmParams::default(), &backend()).expect("plain");
    let mirrored = film_svg(&copper, &FilmParams { mirror: true, ..Default::default() }, &backend()).expect("mirror");
    assert_ne!(plain, mirrored, "mirroring must change the artwork");
    assert!((viewbox_width(&plain) - viewbox_width(&mirrored)).abs() < 1e-6, "mirror preserves the extent");
  }

  #[test]
  fn empty_copper_yields_a_minimal_svg() {
    let empty = MultiPolygon::new(Vec::new());
    let svg = film_svg(&empty, &FilmParams::default(), &backend()).expect("empty");
    assert!(svg.contains("<svg") && svg.contains("</svg>"), "still a valid, empty svg");
    assert!(!svg.contains("<path"), "nothing to draw");
  }

  #[test]
  fn invalid_params_are_rejected() {
    let copper = MultiPolygon::new(vec![square(0.0, 0.0, 5.0)]);
    let bad_scale = FilmParams { scale: 0.0, ..Default::default() };
    let bad_border = FilmParams { border: -1.0, ..Default::default() };
    assert!(film_svg(&copper, &bad_scale, &backend()).is_err());
    assert!(film_svg(&copper, &bad_border, &backend()).is_err());
  }
}
