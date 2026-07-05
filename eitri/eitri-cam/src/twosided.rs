//! Two-sided PCB alignment — mirror a bottom-layer object and generate registration holes.
//!
//! Provenance: FlatCAM's 2-sided tool (see `docs/eitri-porting-plan.md` §7.9). Two related jobs, both thin over the
//! [`eitri_core::Affine`] mirror: (1) [`mirror_multipolygon`] / [`mirror_points`] reflect a bottom-layer object
//! about a chosen axis so it aligns with the top when the board is flipped; (2) [`alignment_holes`] produces
//! registration holes that are symmetric about that axis, so the same drill coordinates hit the same physical spots
//! on both the top and the flipped bottom.
//!
//! A mirror reverses ring orientation, so a mirrored copper object should be re-normalized
//! ([`eitri_geo::GeoBackend::normalize_winding`]) before a milling-direction-sensitive op consumes it — the reflect
//! here is purely coordinate-wise.

use std::f64::consts::FRAC_PI_2;

use geo_types::MultiPolygon;

use eitri_core::Affine;
use eitri_geo::apply_affine;

use crate::optimize::Point;

/// The axis a two-sided flip mirrors about.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MirrorLine {
  /// Reflect about the vertical line `x = value` (a left-right flip, the usual bottom-layer case).
  Vertical(f64),
  /// Reflect about the horizontal line `y = value` (a top-bottom flip).
  Horizontal(f64),
}

impl MirrorLine {
  /// The vertical mirror line through the centre of the X extent `[x0, x1]`.
  pub fn vertical_center(x0: f64, x1: f64) -> MirrorLine {
    MirrorLine::Vertical((x0 + x1) / 2.0)
  }

  /// The horizontal mirror line through the centre of the Y extent `[y0, y1]`.
  pub fn horizontal_center(y0: f64, y1: f64) -> MirrorLine {
    MirrorLine::Horizontal((y0 + y1) / 2.0)
  }

  /// The affine reflection this mirror line performs.
  pub fn affine(self) -> Affine {
    match self {
      MirrorLine::Vertical(x) => Affine::mirror_about_line(x, 0.0, FRAC_PI_2),
      MirrorLine::Horizontal(y) => Affine::mirror_about_line(0.0, y, 0.0),
    }
  }

  /// Reflect a single point about the mirror line.
  pub fn reflect(self, p: Point) -> Point {
    let (x, y) = self.affine().apply(p.x, p.y);
    Point::new(x, y)
  }
}

/// Reflect a copper (or geometry) object about `line`, so a bottom layer aligns with the top after the board flips.
pub fn mirror_multipolygon(source: &MultiPolygon<f64>, line: MirrorLine) -> MultiPolygon<f64> {
  apply_affine(source, line.affine())
}

/// Reflect a set of points (drill hits, features) about `line`.
pub fn mirror_points(points: &[Point], line: MirrorLine) -> Vec<Point> {
  points.iter().map(|&p| line.reflect(p)).collect()
}

/// Registration-hole centres for a two-sided job: each requested hole plus its reflection about `line`, deduped so a
/// hole already on the axis is not doubled. Drilling this symmetric set on both faces keeps the two sides aligned
/// through the flip. `diameter` is carried through for the caller's drilling parameters.
pub fn alignment_holes(base: &[Point], line: MirrorLine) -> Vec<Point> {
  let mut holes: Vec<Point> = Vec::with_capacity(base.len() * 2);
  for &p in base {
    push_unique(&mut holes, p);
    push_unique(&mut holes, line.reflect(p));
  }
  holes
}

/// Push `p` onto `holes` unless a coincident point is already present (within a tight tolerance).
fn push_unique(holes: &mut Vec<Point>, p: Point) {
  if !holes.iter().any(|q| q.distance_to(p) <= 1.0e-9) {
    holes.push(p);
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use geo::algorithm::area::Area;
  use geo_types::{Coord, LineString, Polygon};

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
  fn vertical_mirror_reflects_across_the_axis() {
    // A square centred at x = 10 mirrored about x = 5 lands centred at x = 0, area preserved.
    let source = MultiPolygon::new(vec![square(10.0, 0.0, 2.0)]);
    let mirrored = mirror_multipolygon(&source, MirrorLine::Vertical(5.0));
    let (min, max) = mirrored.0[0]
      .exterior()
      .coords()
      .fold((f64::MAX, f64::MIN), |(a, b), c| (a.min(c.x), b.max(c.x)));
    assert!((min + 2.0).abs() < 1e-9 && (max - 2.0).abs() < 1e-9, "mirrored x-range [{min}, {max}]");
    assert!((mirrored.unsigned_area() - source.unsigned_area()).abs() < 1e-9, "mirror preserves area");
  }

  #[test]
  fn mirror_is_its_own_inverse() {
    let source = MultiPolygon::new(vec![square(3.0, 4.0, 1.5)]);
    let once = mirror_multipolygon(&source, MirrorLine::Horizontal(2.0));
    let twice = mirror_multipolygon(&once, MirrorLine::Horizontal(2.0));
    // Reflecting twice about the same line restores the original coordinates.
    for (a, b) in source.0[0].exterior().coords().zip(twice.0[0].exterior().coords()) {
      assert!((a.x - b.x).abs() < 1e-9 && (a.y - b.y).abs() < 1e-9, "double mirror should restore the object");
    }
  }

  #[test]
  fn center_helpers_pick_the_extent_midpoint() {
    assert_eq!(MirrorLine::vertical_center(0.0, 20.0), MirrorLine::Vertical(10.0));
    assert_eq!(MirrorLine::horizontal_center(-4.0, 4.0), MirrorLine::Horizontal(0.0));
  }

  #[test]
  fn alignment_holes_are_symmetric_about_the_axis() {
    // An off-axis hole at (2, 3) mirrored about x = 5 gives its partner at (8, 3).
    let holes = alignment_holes(&[Point::new(2.0, 3.0)], MirrorLine::Vertical(5.0));
    assert_eq!(holes.len(), 2, "an off-axis hole yields itself plus its mirror");
    assert!(holes.contains(&Point::new(2.0, 3.0)) && holes.contains(&Point::new(8.0, 3.0)), "symmetric pair");
  }

  #[test]
  fn a_hole_on_the_axis_is_not_doubled() {
    // A hole already on the mirror line reflects onto itself, so it appears once.
    let holes = alignment_holes(&[Point::new(5.0, 1.0)], MirrorLine::Vertical(5.0));
    assert_eq!(holes.len(), 1, "an on-axis hole is not duplicated");
  }

  #[test]
  fn mirror_points_reflects_every_hit() {
    let pts = vec![Point::new(0.0, 0.0), Point::new(4.0, 1.0)];
    let mirrored = mirror_points(&pts, MirrorLine::Vertical(2.0));
    // A vertical mirror about x = 2 swaps x -> 4 - x and leaves y unchanged (within float tolerance).
    let expected = [Point::new(4.0, 0.0), Point::new(0.0, 1.0)];
    for (got, want) in mirrored.iter().zip(expected.iter()) {
      assert!(got.distance_to(*want) < 1e-9, "reflected {got:?} should equal {want:?}");
    }
  }
}
