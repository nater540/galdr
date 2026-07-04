//! Winding-order normalization.
//!
//! Milling direction (climb vs conventional) is set by the orientation of the offset ring the cutter follows, so
//! CAM code must be able to force a deterministic winding before generating a toolpath. This is the load-bearing
//! convention flagged in `docs/eitri-porting-plan.md` §7.1: different offset backends report orientation
//! differently, so every ring is normalized here. The convention: the exterior ring takes the requested
//! direction and holes take the opposite, which is the standard positively-oriented-polygon rule.

use geo::algorithm::winding_order::Winding;
use geo_types::Polygon;

use crate::WindingDirection;

/// Return a copy of `poly` with its exterior wound in `direction` and its holes wound the opposite way.
pub fn normalize(poly: &Polygon<f64>, direction: WindingDirection) -> Polygon<f64> {
  let mut exterior = poly.exterior().clone();
  set_winding(&mut exterior, direction);

  let interiors = poly
    .interiors()
    .iter()
    .map(|hole| {
      let mut ring = hole.clone();
      set_winding(&mut ring, direction.reversed());
      ring
    })
    .collect::<Vec<_>>();

  Polygon::new(exterior, interiors)
}

/// Force a single ring to the given winding direction, in place.
fn set_winding(ring: &mut geo_types::LineString<f64>, direction: WindingDirection) {
  match direction {
    WindingDirection::Ccw => ring.make_ccw_winding(),
    WindingDirection::Cw => ring.make_cw_winding(),
  }
}
