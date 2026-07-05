//! 2D affine transforms — the centralized transform math for the whole engine.
//!
//! FlatCAM scattered `shapely.affinity` calls (translate/scale/rotate/skew/mirror) across every geometry method.
//! Eitri owns the matrix math here in `eitri-core`; `eitri-geo` merely walks a geometry's coordinates and calls
//! [`Affine::apply`]. Transforms compose with [`Affine::then`] and invert with [`Affine::inverse`], which is what
//! makes the round-trip and undo behaviour testable without any geometry backend.
//!
//! Representation: a 2x3 matrix mapping a point `(x, y)` to
//! `(a*x + b*y + c,  d*x + e*y + f)` — i.e. the top two rows of a 3x3 homogeneous matrix.

/// A 2D affine transform stored as its six independent coefficients.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Affine {
  /// Row 0: x-from-x.
  pub a: f64,
  /// Row 0: x-from-y.
  pub b: f64,
  /// Row 0: x translation.
  pub c: f64,
  /// Row 1: y-from-x.
  pub d: f64,
  /// Row 1: y-from-y.
  pub e: f64,
  /// Row 1: y translation.
  pub f: f64,
}

impl Default for Affine {
  fn default() -> Affine {
    Affine::IDENTITY
  }
}

impl Affine {
  /// The identity transform.
  pub const IDENTITY: Affine = Affine { a: 1.0, b: 0.0, c: 0.0, d: 0.0, e: 1.0, f: 0.0 };

  /// A pure translation by `(dx, dy)`.
  pub const fn translate(dx: f64, dy: f64) -> Affine {
    Affine { a: 1.0, b: 0.0, c: dx, d: 0.0, e: 1.0, f: dy }
  }

  /// A scaling by `(sx, sy)` about the origin. Use [`Affine::scale_about`] to scale about an arbitrary point.
  pub const fn scale(sx: f64, sy: f64) -> Affine {
    Affine { a: sx, b: 0.0, c: 0.0, d: 0.0, e: sy, f: 0.0 }
  }

  /// A scaling by `(sx, sy)` about the point `(ox, oy)`.
  pub fn scale_about(sx: f64, sy: f64, ox: f64, oy: f64) -> Affine {
    Affine::about(ox, oy, Affine::scale(sx, sy))
  }

  /// A counter-clockwise rotation by `theta` radians about the origin.
  pub fn rotate(theta: f64) -> Affine {
    let (s, cs) = theta.sin_cos();
    Affine { a: cs, b: -s, c: 0.0, d: s, e: cs, f: 0.0 }
  }

  /// A counter-clockwise rotation by `theta` radians about the point `(ox, oy)`.
  pub fn rotate_about(theta: f64, ox: f64, oy: f64) -> Affine {
    Affine::about(ox, oy, Affine::rotate(theta))
  }

  /// Reflection about the x-axis (negates y).
  pub const fn mirror_x_axis() -> Affine {
    Affine { a: 1.0, b: 0.0, c: 0.0, d: 0.0, e: -1.0, f: 0.0 }
  }

  /// Reflection about the y-axis (negates x).
  pub const fn mirror_y_axis() -> Affine {
    Affine { a: -1.0, b: 0.0, c: 0.0, d: 0.0, e: 1.0, f: 0.0 }
  }

  /// Reflection about an arbitrary line through `(px, py)` at `angle` radians from the x-axis.
  pub fn mirror_about_line(px: f64, py: f64, angle: f64) -> Affine {
    // Reflection about a line through the origin at angle φ is [[cos2φ, sin2φ], [sin2φ, -cos2φ]].
    let (s2, c2) = (2.0 * angle).sin_cos();
    let reflect = Affine { a: c2, b: s2, c: 0.0, d: s2, e: -c2, f: 0.0 };
    Affine::about(px, py, reflect)
  }

  /// A skew (shear) by the given x- and y-angles (radians) about the point `(ox, oy)`.
  /// `x_angle` shears along x proportional to y; `y_angle` shears along y proportional to x — matching Shapely.
  pub fn skew_about(x_angle: f64, y_angle: f64, ox: f64, oy: f64) -> Affine {
    let shear = Affine { a: 1.0, b: x_angle.tan(), c: 0.0, d: y_angle.tan(), e: 1.0, f: 0.0 };
    Affine::about(ox, oy, shear)
  }

  /// Wrap a linear transform `inner` so it acts about the point `(ox, oy)` instead of the origin:
  /// translate the point to the origin, apply `inner`, translate back.
  fn about(ox: f64, oy: f64, inner: Affine) -> Affine {
    Affine::translate(-ox, -oy).then(inner).then(Affine::translate(ox, oy))
  }

  /// Compose two transforms: the result applies `self` first, then `next` (i.e. the matrix product `next * self`).
  pub fn then(self, next: Affine) -> Affine {
    let (m1, m2) = (self, next);
    Affine {
      a: m2.a * m1.a + m2.b * m1.d,
      b: m2.a * m1.b + m2.b * m1.e,
      c: m2.a * m1.c + m2.b * m1.f + m2.c,
      d: m2.d * m1.a + m2.e * m1.d,
      e: m2.d * m1.b + m2.e * m1.e,
      f: m2.d * m1.c + m2.e * m1.f + m2.f,
    }
  }

  /// Apply the transform to a point, returning the mapped `(x, y)`.
  pub fn apply(self, x: f64, y: f64) -> (f64, f64) {
    (self.a * x + self.b * y + self.c, self.d * x + self.e * y + self.f)
  }

  /// The determinant of the linear part. Zero (within tolerance) means the transform is singular / non-invertible.
  pub fn determinant(self) -> f64 {
    self.a * self.e - self.b * self.d
  }

  /// The inverse transform, or `None` if the transform is singular (a degenerate scale/projection).
  pub fn inverse(self) -> Option<Affine> {
    let det = self.determinant();
    if det.abs() < 1e-12 {
      return None;
    }
    let inv_det = 1.0 / det;
    let a = self.e * inv_det;
    let b = -self.b * inv_det;
    let d = -self.d * inv_det;
    let e = self.a * inv_det;
    // Inverse translation is -(linear_inverse * original_translation).
    let c = -(a * self.c + b * self.f);
    let f = -(d * self.c + e * self.f);
    Some(Affine { a, b, c, d, e, f })
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  const TOL: f64 = 1e-9;
  const PI: f64 = std::f64::consts::PI;

  fn close(p: (f64, f64), q: (f64, f64)) -> bool {
    (p.0 - q.0).abs() < TOL && (p.1 - q.1).abs() < TOL
  }

  /// Applying a transform and then its inverse returns the original point — for every primitive.
  fn assert_round_trips(t: Affine, pts: &[(f64, f64)]) {
    let inv = t.inverse().expect("primitive transforms are invertible");
    for &(x, y) in pts {
      let (mx, my) = t.apply(x, y);
      assert!(close(inv.apply(mx, my), (x, y)), "inverse of {t:?} did not restore ({x}, {y})");
    }
  }

  const SAMPLES: [(f64, f64); 4] = [(0.0, 0.0), (3.0, -2.0), (-5.5, 4.25), (10.0, 10.0)];

  #[test]
  fn identity_is_a_no_op() {
    for &(x, y) in &SAMPLES {
      assert!(close(Affine::IDENTITY.apply(x, y), (x, y)));
    }
    assert_eq!(Affine::default(), Affine::IDENTITY);
  }

  #[test]
  fn translate_round_trips() {
    assert_round_trips(Affine::translate(7.0, -3.0), &SAMPLES);
  }

  #[test]
  fn scale_about_round_trips_and_fixes_center() {
    let t = Affine::scale_about(2.0, 0.5, 4.0, -1.0);
    assert_round_trips(t, &SAMPLES);
    // The center of scaling is a fixed point.
    assert!(close(t.apply(4.0, -1.0), (4.0, -1.0)));
  }

  #[test]
  fn rotate_about_round_trips_and_fixes_center() {
    let t = Affine::rotate_about(PI / 3.0, 2.0, 2.0);
    assert_round_trips(t, &SAMPLES);
    assert!(close(t.apply(2.0, 2.0), (2.0, 2.0)));
  }

  #[test]
  fn quarter_turn_maps_axes_as_expected() {
    let t = Affine::rotate(PI / 2.0);
    // CCW 90°: (1,0) -> (0,1); (0,1) -> (-1,0).
    assert!(close(t.apply(1.0, 0.0), (0.0, 1.0)));
    assert!(close(t.apply(0.0, 1.0), (-1.0, 0.0)));
  }

  #[test]
  fn mirror_is_its_own_inverse() {
    for t in [Affine::mirror_x_axis(), Affine::mirror_y_axis(), Affine::mirror_about_line(1.0, 1.0, PI / 4.0)] {
      for &(x, y) in &SAMPLES {
        let (mx, my) = t.apply(x, y);
        assert!(close(t.apply(mx, my), (x, y)), "mirror {t:?} applied twice is not identity");
      }
    }
  }

  #[test]
  fn mirror_about_45_degree_line_swaps_coordinates() {
    // Reflecting about y = x (line through origin at 45°) swaps x and y.
    let t = Affine::mirror_about_line(0.0, 0.0, PI / 4.0);
    assert!(close(t.apply(3.0, 7.0), (7.0, 3.0)));
  }

  #[test]
  fn skew_round_trips() {
    assert_round_trips(Affine::skew_about(0.3, -0.2, 1.0, 1.0), &SAMPLES);
  }

  #[test]
  fn composition_applies_self_then_next() {
    // Translate by (1,0), then rotate 90° CCW about the origin. Point (0,0) -> (1,0) -> (0,1).
    let t = Affine::translate(1.0, 0.0).then(Affine::rotate(PI / 2.0));
    assert!(close(t.apply(0.0, 0.0), (0.0, 1.0)));
  }

  #[test]
  fn full_composition_round_trips_to_origin() {
    // Compose all five primitives, then apply the composed inverse — every sample point returns to itself.
    let composed = Affine::translate(5.0, -2.0)
      .then(Affine::scale_about(1.5, 2.0, 1.0, 1.0))
      .then(Affine::rotate_about(PI / 5.0, -3.0, 4.0))
      .then(Affine::mirror_about_line(2.0, 0.0, PI / 6.0))
      .then(Affine::skew_about(0.15, 0.1, 0.0, 0.0));
    assert_round_trips(composed, &SAMPLES);
  }

  #[test]
  fn singular_transform_has_no_inverse() {
    // Scaling y to zero collapses the plane onto a line — not invertible.
    assert!(Affine::scale(2.0, 0.0).inverse().is_none());
  }
}
