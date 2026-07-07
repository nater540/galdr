//! Probe-tip radius compensation — the one rule every datum op shares.
//!
//! A touch probe reports the machine coordinate of the tip CENTRE at the moment of contact, but the operator
//! wants the coordinate of the EDGE the tip touched. The tip centre sits short of the edge by the tip radius, in
//! the direction the probe was travelling — so the compensated edge is `contact + (Ø/2)·af`, where `af` is the
//! approach-direction sign (`+1` toward increasing coordinate, `−1` toward decreasing). This mirrors ioSender's
//! `probed + (ProbeDiameter/2)·af` (credit: ioSender / OpenCNCPilot, MIT). It is applied only to the LATERAL
//! axes (X/Y): a Z surface touch reads the surface directly under the tip, so Z is never compensated.
//!
//! Inside (pocket) features flip `af` relative to an outside feature at the same location, because the probe
//! approaches from the opposite side — but that flip is entirely captured in the caller's chosen `af`, so this
//! function stays a single pure rule. Kept egui-free and allocation-free so it unit-tests without a window.

/// The compensated edge coordinate for a lateral (X/Y) probe: `contact + (probe_diameter/2)·af`.
///
/// `contact` is the machine-coordinate tip-centre reading from `[PRB:]`; `af` is the approach-direction sign
/// (`+1.0`/`-1.0`) along that axis; `probe_diameter` is the tip/ball diameter (mm). A zero diameter is the
/// identity (an infinitely sharp tip needs no compensation). Never applied to Z.
pub fn edge_coord(contact: f64, af: f64, probe_diameter: f64) -> f64 {
  contact + probe_diameter / 2.0 * af
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn a_positive_approach_pushes_the_edge_further_along_the_axis() {
    // Approaching +X with a 4 mm tip: the tip centre stops 2 mm short of the edge, so the edge is 2 mm beyond it.
    assert_eq!(edge_coord(10.0, 1.0, 4.0), 12.0);
  }

  #[test]
  fn a_negative_approach_pushes_the_edge_the_other_way() {
    // Approaching −X, the edge is 2 mm below the tip-centre reading.
    assert_eq!(edge_coord(10.0, -1.0, 4.0), 8.0);
  }

  #[test]
  fn a_zero_diameter_tip_needs_no_compensation() {
    // A theoretically sharp tip reads the edge directly; the compensation is the identity.
    assert_eq!(edge_coord(-3.25, 1.0, 0.0), -3.25);
    assert_eq!(edge_coord(-3.25, -1.0, 0.0), -3.25);
  }

  #[test]
  fn inside_versus_outside_is_purely_the_sign_of_af() {
    // The same contact reading, with opposite approach signs (an outside vs inside feature at that location),
    // yields edges symmetric about the contact by exactly the tip radius. This is the whole inside/outside story.
    let contact = 5.0;
    let outside = edge_coord(contact, 1.0, 6.0);
    let inside = edge_coord(contact, -1.0, 6.0);
    assert_eq!(outside, 8.0);
    assert_eq!(inside, 2.0);
    assert_eq!((outside + inside) / 2.0, contact, "the two comped edges straddle the contact by the tip radius");
  }
}
