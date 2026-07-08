//! [`JobOrigin`] — the operator-facing work-zero (datum) selector, the CAM analogue of Vectric's "XY datum
//! position".
//!
//! Eitri passes a board's source (EDA plot) coordinates straight through to G-code, so without a datum the exported
//! program is referenced to wherever the EDA tool plotted from — usually far off the board. A [`JobOrigin`] names a
//! point in that native frame that should become work `(0, 0)`; [`JobOrigin::resolve`] turns it into the concrete
//! offset the emitter subtracts (`eitri_gcode::Origin`). Choosing a board corner or centre makes the exported
//! coordinates land on the board, so the operator simply zeroes the machine at that same physical point.
//!
//! The datum is a **posting** concern, not a geometry edit: it never moves the objects, only the frame the G-code is
//! emitted in — so every layer and drill file resolved against the same reference registers on top of each other.

use serde::{Deserialize, Serialize};

/// A named point of a rectangular reference boundary (the board outline), used by [`JobOrigin::Bounds`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DatumCorner {
  /// The minimum-X, minimum-Y corner.
  BottomLeft,
  /// The maximum-X, minimum-Y corner.
  BottomRight,
  /// The minimum-X, maximum-Y corner.
  TopLeft,
  /// The maximum-X, maximum-Y corner.
  TopRight,
  /// The centre of the boundary.
  Center,
}

/// Where a job's work-zero sits, in the board's native coordinate frame. Resolved against a reference boundary (the
/// board outline's bounds) into the concrete offset the emitter posts relative to.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub enum JobOrigin {
  /// Keep the source (EDA plot) coordinates unchanged — the historical behaviour and the default.
  #[default]
  Native,
  /// A corner or the centre of the reference boundary (the board outline).
  Bounds(DatumCorner),
  /// An explicit point in native coordinates.
  Absolute {
    /// Native-frame X that maps to work X0.
    x: f64,
    /// Native-frame Y that maps to work Y0.
    y: f64,
  },
}

impl JobOrigin {
  /// The native-frame point that maps to work `(0, 0)`, given reference `bounds` as `(min_x, min_y, max_x, max_y)`
  /// (the shape [`eitri_geo::bounds`] returns). [`JobOrigin::Native`] ignores the bounds and resolves to the origin,
  /// leaving coordinates unchanged.
  pub fn resolve(&self, bounds: (f64, f64, f64, f64)) -> (f64, f64) {
    match self {
      JobOrigin::Native => (0.0, 0.0),
      JobOrigin::Absolute { x, y } => (*x, *y),
      JobOrigin::Bounds(corner) => corner.resolve(bounds),
    }
  }
}

impl DatumCorner {
  /// The `(x, y)` of this corner/centre of `bounds` as `(min_x, min_y, max_x, max_y)`.
  pub fn resolve(&self, bounds: (f64, f64, f64, f64)) -> (f64, f64) {
    let (min_x, min_y, max_x, max_y) = bounds;
    match self {
      DatumCorner::BottomLeft => (min_x, min_y),
      DatumCorner::BottomRight => (max_x, min_y),
      DatumCorner::TopLeft => (min_x, max_y),
      DatumCorner::TopRight => (max_x, max_y),
      DatumCorner::Center => ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0),
    }
  }
}

/// Which face of the stock the operator zeroes Z on — the material top (the usual: touch off on the copper) or the
/// bottom (touch off on the bed/spoilboard, so work-Z0 is one stock-thickness below the surface).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum ZReference {
  /// Work Z0 is the stock's top surface.
  #[default]
  Top,
  /// Work Z0 is the stock's bottom face.
  Bottom,
}

/// The material block being machined plus the work-zero it defines — the CAM-side "Job Setup". The footprint is an
/// absolute native-frame rectangle (`min` corner + `size`); `thickness` is the Z extent; `z_ref` picks which face is
/// work-Z0; and `datum` picks which footprint corner (or centre) is work `(X0, Y0)`. [`Stock::origin`] resolves all
/// of that into the `(x, y, z)` the emitter posts relative to.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Stock {
  /// Footprint minimum X (native frame).
  pub min_x: f64,
  /// Footprint minimum Y (native frame).
  pub min_y: f64,
  /// Footprint width (X, millimetres).
  pub size_x: f64,
  /// Footprint height (Y, millimetres).
  pub size_y: f64,
  /// Material thickness (Z, millimetres).
  pub thickness: f64,
  /// Which face is work-Z0.
  pub z_ref: ZReference,
  /// Which footprint corner (or centre) is work `(X0, Y0)`.
  pub datum: DatumCorner,
}

impl Stock {
  /// The footprint as `(min_x, min_y, max_x, max_y)`.
  pub fn footprint(&self) -> (f64, f64, f64, f64) {
    (self.min_x, self.min_y, self.min_x + self.size_x, self.min_y + self.size_y)
  }

  /// The native-frame point `(x, y, z)` that maps to work `(0, 0, 0)`: the datum corner of the footprint, and the
  /// Z of the chosen reference face (`0` for a top datum, `-thickness` for a bottom datum, so the emitter lifts every
  /// Z by the thickness).
  pub fn origin(&self) -> (f64, f64, f64) {
    let (x, y) = self.datum.resolve(self.footprint());
    let z = match self.z_ref {
      ZReference::Top => 0.0,
      ZReference::Bottom => -self.thickness,
    };
    (x, y, z)
  }

  /// A stock auto-fitted to a board's `bounds` (footprint = bounds) with the given `thickness`, a bottom-left datum,
  /// and a top Z reference — the sensible default the UI seeds when a board is loaded.
  pub fn fit(bounds: (f64, f64, f64, f64), thickness: f64) -> Stock {
    let (min_x, min_y, max_x, max_y) = bounds;
    Stock {
      min_x,
      min_y,
      size_x: max_x - min_x,
      size_y: max_y - min_y,
      thickness,
      z_ref: ZReference::Top,
      datum: DatumCorner::BottomLeft,
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  // A board spanning (120, -110) .. (137, -86) — the KiCad-frame shape the starter fixtures use.
  const BOUNDS: (f64, f64, f64, f64) = (120.0, -110.0, 137.0, -86.0);

  #[test]
  fn native_leaves_coordinates_unchanged() {
    assert_eq!(JobOrigin::Native.resolve(BOUNDS), (0.0, 0.0));
    assert_eq!(JobOrigin::default(), JobOrigin::Native);
  }

  #[test]
  fn each_corner_resolves_to_its_bound() {
    assert_eq!(JobOrigin::Bounds(DatumCorner::BottomLeft).resolve(BOUNDS), (120.0, -110.0));
    assert_eq!(JobOrigin::Bounds(DatumCorner::BottomRight).resolve(BOUNDS), (137.0, -110.0));
    assert_eq!(JobOrigin::Bounds(DatumCorner::TopLeft).resolve(BOUNDS), (120.0, -86.0));
    assert_eq!(JobOrigin::Bounds(DatumCorner::TopRight).resolve(BOUNDS), (137.0, -86.0));
    assert_eq!(JobOrigin::Bounds(DatumCorner::Center).resolve(BOUNDS), (128.5, -98.0));
  }

  #[test]
  fn absolute_is_the_point_itself() {
    assert_eq!(JobOrigin::Absolute { x: 5.0, y: -3.0 }.resolve(BOUNDS), (5.0, -3.0));
  }

  #[test]
  fn bottom_left_datum_makes_the_board_start_at_the_origin() {
    // Subtracting the bottom-left datum from the board's own corner lands it at (0, 0) — the point the operator
    // zeroes the machine on.
    let (dx, dy) = JobOrigin::Bounds(DatumCorner::BottomLeft).resolve(BOUNDS);
    let (corner_x, corner_y) = (BOUNDS.0, BOUNDS.1);
    assert_eq!((corner_x - dx, corner_y - dy), (0.0, 0.0));
  }

  #[test]
  fn stock_fits_a_board_and_resolves_a_top_bottom_datum() {
    let stock = Stock::fit(BOUNDS, 1.6);
    // The footprint is the board bounds; a fresh fit is bottom-left / top.
    assert_eq!(stock.footprint(), BOUNDS);
    assert_eq!(stock.datum, DatumCorner::BottomLeft);
    // Top datum: origin is the bottom-left corner at Z0 = surface.
    assert_eq!(stock.origin(), (BOUNDS.0, BOUNDS.1, 0.0));
    // Bottom datum: Z0 drops one thickness below the surface, so the emitter lifts every Z by 1.6.
    let bottom = Stock { z_ref: ZReference::Bottom, ..stock };
    assert_eq!(bottom.origin(), (BOUNDS.0, BOUNDS.1, -1.6));
    // A centre datum on an oversized stock resolves to the footprint centre, not the board's.
    let big = Stock { size_x: 40.0, size_y: 40.0, datum: DatumCorner::Center, ..stock };
    assert_eq!(big.origin(), (BOUNDS.0 + 20.0, BOUNDS.1 + 20.0, 0.0));
  }
}
