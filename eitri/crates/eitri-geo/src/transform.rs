//! Applying an [`eitri_core::Affine`] to `geo_types` geometry.
//!
//! `eitri-core` owns the transform *math* (translate/scale/rotate/mirror/skew and their composition); this module
//! is the thin coordinate walker that carries an [`Affine`] over a `Polygon`/`MultiPolygon` by mapping every
//! coordinate through [`Affine::apply`]. Keeping the walk here — rather than re-deriving matrices with `geo`'s own
//! `AffineTransform` — means there is exactly one transform source of truth for the whole engine. See
//! `docs/eitri-porting-plan.md` §3, §7.10.
//!
//! Note on winding: a reflecting or negatively-scaled transform flips ring orientation, so callers that depend on a
//! specific winding (milling direction, hole/exterior convention) must re-run [`crate::winding::normalize`]
//! afterwards — the walk itself is purely coordinate-wise and does not touch winding.

use geo::algorithm::map_coords::MapCoords;
use geo_types::{Coord, MultiPolygon, Polygon};

use eitri_core::Affine;

/// Return `poly` with every vertex mapped through `transform`.
pub fn apply_affine_polygon(poly: &Polygon<f64>, transform: Affine) -> Polygon<f64> {
  poly.map_coords(|c| {
    let (x, y) = transform.apply(c.x, c.y);
    Coord { x, y }
  })
}

/// Return `mp` with every vertex of every polygon mapped through `transform`.
pub fn apply_affine(mp: &MultiPolygon<f64>, transform: Affine) -> MultiPolygon<f64> {
  mp.map_coords(|c| {
    let (x, y) = transform.apply(c.x, c.y);
    Coord { x, y }
  })
}

#[cfg(test)]
mod tests {
  use super::*;
  use geo::algorithm::area::Area;
  use geo_types::LineString;

  fn square(cx: f64, cy: f64, half: f64) -> Polygon<f64> {
    Polygon::new(
      LineString(vec![
        Coord { x: cx - half, y: cy - half },
        Coord { x: cx + half, y: cy - half },
        Coord { x: cx + half, y: cy + half },
        Coord { x: cx - half, y: cy + half },
        Coord { x: cx - half, y: cy - half },
      ]),
      vec![],
    )
  }

  #[test]
  fn translate_moves_every_vertex() {
    let moved = apply_affine_polygon(&square(0.0, 0.0, 1.0), Affine::translate(3.0, -2.0));
    for c in moved.exterior().coords() {
      assert!(c.x >= 1.999 && c.x <= 4.001, "x {} out of translated range", c.x);
      assert!(c.y >= -3.001 && c.y <= -0.999, "y {} out of translated range", c.y);
    }
  }

  #[test]
  fn scale_about_center_grows_area_by_the_scale_factors() {
    let mp = MultiPolygon::new(vec![square(0.0, 0.0, 1.0)]);
    let scaled = apply_affine(&mp, Affine::scale_about(2.0, 3.0, 0.0, 0.0));
    // A 2x2 square (area 4) scaled by (2, 3) has area 4 * 2 * 3 = 24.
    assert!((scaled.unsigned_area() - 24.0).abs() < 1e-9, "area {}", scaled.unsigned_area());
  }

  #[test]
  fn mirror_reflects_coordinates_across_the_line() {
    // Mirror about the vertical line x = 5: a square at x in [9, 11] lands at x in [-1, 1].
    let mp = MultiPolygon::new(vec![square(10.0, 0.0, 1.0)]);
    let mirrored = apply_affine(&mp, Affine::mirror_about_line(5.0, 0.0, std::f64::consts::FRAC_PI_2));
    let xs: Vec<f64> = mirrored.0[0].exterior().coords().map(|c| c.x).collect();
    let (min, max) = xs.iter().fold((f64::MAX, f64::MIN), |(a, b), &x| (a.min(x), b.max(x)));
    assert!((min + 1.0).abs() < 1e-9 && (max - 1.0).abs() < 1e-9, "mirrored x-range [{min}, {max}]");
  }
}
