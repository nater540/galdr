//! Pure geometry for the correction pass: linear subdivision counts and arc sub-division.
//!
//! The pre-stream correction (Part C, [`super::correct`]) subdivides each cutting move so it can resample the
//! height-map along the path and shift Z to follow the surface. This module holds the egui-free, `f64` geometry
//! that decides HOW FINELY to subdivide and WHERE the arc sub-points fall — generalising the f32, G17-only
//! [`crate::app::preview::flatten_arc`] used for the on-screen preview. Unlike the preview (which flattens an arc
//! to straight chords for drawing), the correction keeps arcs as real G2/G3 sub-arcs, so here we expose the arc's
//! centre/radius/sweep and a point-at-fraction sampler; the caller recomputes each sub-arc's I/J from the true
//! centre. Pure and unit-tested without a window.

use std::f64::consts::TAU;

/// The planar (XY) distance between two points. The correction subdivides on the PLANAR distance, not the 3D
/// length, because the height-map varies only in XY — a steep pure-Z plunge needs no XY subdivision.
pub fn planar_distance(a: (f64, f64), b: (f64, f64)) -> f64 {
  let dx = b.0 - a.0;
  let dy = b.1 - a.1;
  (dx * dx + dy * dy).sqrt()
}

/// The number of even sub-segments a path of length `distance` splits into for a target segment length `seg`:
/// `ceil(distance/seg)`, floored at 1. A non-positive `seg` (a degenerate mesh spacing) or a zero-length path
/// yields a single segment rather than dividing by zero or producing an unbounded count.
pub fn subdivision_count(distance: f64, seg: f64) -> usize {
  if seg <= 0.0 || distance <= 0.0 {
    return 1;
  }
  (distance / seg).ceil().max(1.0) as usize
}

/// An arc's geometry in its plane: the centre, radius, start angle, and SIGNED sweep (positive for CCW/G3,
/// negative for CW/G2), normalised so the magnitude is in `(0, 2π]` — a coincident start/end is a full circle,
/// exactly as [`crate::app::preview::flatten_arc`] treats it. Built from the endpoints + centre; the caller
/// samples it with [`Self::point_at`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArcSpan {
  /// The arc centre (the I/J offset applied to the start point), in work-mm.
  pub center: (f64, f64),
  /// The radius (mm), taken from the start point (the end point lies on the same circle by construction).
  pub radius: f64,
  /// The start angle (radians) of the start point about the centre.
  pub start_angle: f64,
  /// The signed sweep (radians): `+` CCW (G3), `−` CW (G2); magnitude in `(0, 2π]`.
  pub signed_sweep: f64,
}

impl ArcSpan {
  /// Build the span for an arc from `start` to `end` about `center`, clockwise (`cw`) for G2. The sweep magnitude
  /// is taken the direction-consistent way and normalised into `(0, 2π]` so a start == end is a full revolution.
  pub fn from_endpoints(start: (f64, f64), end: (f64, f64), center: (f64, f64), cw: bool) -> Self {
    let radius = planar_distance(center, start);
    let start_angle = (start.1 - center.1).atan2(start.0 - center.0);
    let end_angle = (end.1 - center.1).atan2(end.0 - center.0);
    // Magnitude of the sweep in the direction of travel: CCW increases the angle, CW decreases it. Normalise to
    // a positive magnitude in (0, 2π]; a zero/negative raw magnitude means a full circle.
    let mut magnitude = if cw { start_angle - end_angle } else { end_angle - start_angle };
    while magnitude <= 0.0 {
      magnitude += TAU;
    }
    let signed_sweep = if cw { -magnitude } else { magnitude };
    ArcSpan { center, radius, start_angle, signed_sweep }
  }

  /// The arc length (mm): `radius · |sweep|`.
  pub fn length(&self) -> f64 {
    self.radius * self.signed_sweep.abs()
  }

  /// The point at fraction `frac ∈ [0, 1]` along the sweep (0 = start, 1 = end). `angle = start_angle +
  /// signed_sweep · frac`, so the direction sign is already baked into the sweep.
  pub fn point_at(&self, frac: f64) -> (f64, f64) {
    let theta = self.start_angle + self.signed_sweep * frac;
    (self.center.0 + self.radius * theta.cos(), self.center.1 + self.radius * theta.sin())
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-9
  }

  fn close_pt(a: (f64, f64), b: (f64, f64)) -> bool {
    close(a.0, b.0) && close(a.1, b.1)
  }

  #[test]
  fn subdivision_count_is_ceil_distance_over_seg_floored_at_one() {
    assert_eq!(subdivision_count(10.0, 3.0), 4); // 10/3 = 3.33 → 4.
    assert_eq!(subdivision_count(10.0, 2.0), 5); // exact.
    assert_eq!(subdivision_count(1.0, 3.0), 1); // less than one seg → 1.
    assert_eq!(subdivision_count(0.0, 3.0), 1); // a zero-length (pure-Z) path → 1.
    assert_eq!(subdivision_count(10.0, 0.0), 1); // a degenerate seg → 1, never a divide-by-zero.
  }

  #[test]
  fn arc_span_measures_a_ccw_quarter_circle() {
    let span = ArcSpan::from_endpoints((1.0, 0.0), (0.0, 1.0), (0.0, 0.0), false);
    assert!(close(span.radius, 1.0));
    assert!(close(span.signed_sweep, std::f64::consts::FRAC_PI_2), "a CCW quarter turn sweeps +π/2");
    assert!(close(span.length(), std::f64::consts::FRAC_PI_2), "length = radius·|sweep| = π/2");
    // Midpoint of the quarter turn is at 45°.
    let mid = span.point_at(0.5);
    assert!(close_pt(mid, (std::f64::consts::FRAC_1_SQRT_2, std::f64::consts::FRAC_1_SQRT_2)));
    // Endpoints land exactly.
    assert!(close_pt(span.point_at(0.0), (1.0, 0.0)));
    assert!(close_pt(span.point_at(1.0), (0.0, 1.0)));
  }

  #[test]
  fn cw_and_ccw_sweep_opposite_ways() {
    // From (1,0) to (-1,0) about the origin: CCW goes over the top (+π), CW under the bottom (−π).
    let ccw = ArcSpan::from_endpoints((1.0, 0.0), (-1.0, 0.0), (0.0, 0.0), false);
    let cw = ArcSpan::from_endpoints((1.0, 0.0), (-1.0, 0.0), (0.0, 0.0), true);
    assert!(close(ccw.signed_sweep, std::f64::consts::PI));
    assert!(close(cw.signed_sweep, -std::f64::consts::PI));
    // CCW half-turn midpoint is the top (0,1); CW is the bottom (0,-1).
    assert!(close_pt(ccw.point_at(0.5), (0.0, 1.0)));
    assert!(close_pt(cw.point_at(0.5), (0.0, -1.0)));
  }

  #[test]
  fn coincident_endpoints_are_a_full_circle() {
    let span = ArcSpan::from_endpoints((1.0, 0.0), (1.0, 0.0), (0.0, 0.0), false);
    assert!(close(span.signed_sweep.abs(), TAU), "a start == end arc is a full revolution");
    assert!(close(span.length(), TAU)); // radius 1.
  }
}
