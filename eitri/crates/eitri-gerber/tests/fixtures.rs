//! Integration tests over the hand-authored Gerber fixtures in `eitri/fixtures/gerber/`.
//!
//! These drive the whole pipeline (lex → interpret → assemble) on real files, complementing the unit tests: a
//! realistic copper layer, both zero-suppression modes, a macro-aperture flash, and an LP clear cutting a region.

use std::path::PathBuf;

use eitri_core::{CancelToken, ProgressReporter};
use eitri_gerber::{GerberError, GerberImage, parse_gerber};

use geo::algorithm::area::Area;

fn fixture(name: &str) -> String {
  let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../fixtures/synthetic/gerber").join(name);
  std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

fn parse(name: &str) -> GerberImage {
  parse_gerber(&fixture(name), &ProgressReporter::silent(), &CancelToken::new()).expect("parse")
}

/// Read a real (non-synthetic) fixture from `fixtures/gerber/` — a full board plotted by KiCad.
fn real_fixture(name: &str) -> String {
  let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../fixtures/gerber").join(name);
  std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

#[test]
fn kicad_ground_pour_survives_the_shared_union() {
  // Regression: a KiCad copper pour is one self-touching G36 region — the boundary carves each isolated-pad
  // clearance with a zero-width bridge (out to the clearance and back along the same line). Pushed raw into the
  // same-polarity `union_all` alongside the pads, that non-simple boundary was mis-resolved and the whole pour
  // dropped, collapsing the parsed copper to just the pads/trace (~18 mm²). It must retain the full pour.
  let img = parse_gerber(&real_fixture("starter-F_Cu.gbr"), &ProgressReporter::silent(), &CancelToken::new())
    .expect("parse starter-F_Cu");
  assert!(img.copper.unsigned_area() > 150.0, "ground pour missing — copper area only {}", img.copper.unsigned_area());
  // The discriminating signal that the region (not just the pad flashes) assembled: the pour is a filled area with
  // clearance holes around the isolated features.
  let holes: usize = img.copper.0.iter().map(|p| p.interiors().len()).sum();
  assert!(holes > 0, "expected clearance holes in the pour, found none");
}

fn try_parse(name: &str) -> Result<GerberImage, GerberError> {
  parse_gerber(&fixture(name), &ProgressReporter::silent(), &CancelToken::new())
}

#[test]
fn kicad_two_pads_assembles_connected_copper() {
  let img = parse("kicad_two_pads.gbr");
  // Two apertures defined (2mm pad, 0.25mm trace).
  assert_eq!(img.apertures.len(), 2);
  // The trace joins both pads, so the copper is a single connected polygon.
  assert_eq!(img.copper.0.len(), 1, "expected one connected copper polygon");
  // Bounds: pads at x∈[-1,1] and [9,11], y∈[-1,1]; trace along y=0. Overall [-1,-1]..[11,1].
  let (minx, miny, maxx, maxy) = img.bounds().expect("bounds");
  assert!((minx - -1.0).abs() < 0.05 && (maxx - 11.0).abs() < 0.05, "x bounds {minx}..{maxx}");
  assert!((miny - -1.0).abs() < 0.05 && (maxy - 1.0).abs() < 0.05, "y bounds {miny}..{maxy}");
  // At least the two 2mm disks (2*pi ≈ 6.28) of copper.
  assert!(img.copper.unsigned_area() > 6.0, "area {}", img.copper.unsigned_area());
}

#[test]
fn leading_and_trailing_omission_decode_identically() {
  let lead = parse("coords_leading.gbr");
  let trail = parse("coords_trailing.gbr");
  // Both place a 1mm pad centred at (5, 5): bounds [4.5,4.5]..[5.5,5.5].
  for img in [&lead, &trail] {
    let (minx, miny, maxx, maxy) = img.bounds().expect("bounds");
    assert!((minx - 4.5).abs() < 0.05 && (maxx - 5.5).abs() < 0.05, "x bounds {minx}..{maxx}");
    assert!((miny - 4.5).abs() < 0.05 && (maxy - 5.5).abs() < 0.05, "y bounds {miny}..{maxy}");
  }
  // The decoded geometry is the same to within the polygon-approximation tolerance.
  assert!((lead.copper.unsigned_area() - trail.copper.unsigned_area()).abs() < 1e-6);
}

#[test]
fn macro_thermal_flash_has_a_hole() {
  let img = parse("macro_thermal.gbr");
  // A thermal relief: outer disk (r4, area ~50.3) minus inner disk and cross gap, so well under the solid disk.
  let area = img.copper.unsigned_area();
  let outer_disk = std::f64::consts::PI * 16.0;
  assert!(area > 0.0 && area < outer_disk, "thermal area {area} should be below the solid outer disk {outer_disk}");
}

#[test]
fn region_with_clear_area_is_dark_minus_clear() {
  let img = parse("region_with_clear.gbr");
  assert!((img.copper.unsigned_area() - 84.0).abs() < 1e-3, "area {}", img.copper.unsigned_area());
}

#[test]
fn modal_draw_repeats_the_operation_code() {
  // Finding #6: the bare `X2000000Y0*` line repeats the preceding D01, so the stroke runs the full 0..2, reaching
  // x=2.5 with the 1mm round aperture rather than stopping at x=1.5.
  let img = parse("modal_draw.gbr");
  assert!((img.copper.unsigned_area() - 2.785).abs() < 0.05, "area {}", img.copper.unsigned_area());
  let (_, _, maxx, _) = img.bounds().expect("bounds");
  assert!((maxx - 2.5).abs() < 0.05, "the modal stroke must reach x=2 (maxx {maxx})");
}

#[test]
fn macro_stroke_is_refused_loudly() {
  // Finding #4: a D01 draw with a macro aperture selected must error, not emit empty copper.
  assert!(matches!(try_parse("macro_stroke_refused.gbr"), Err(GerberError::Unsupported { .. })));
}

#[test]
fn arc_without_offset_is_invalid_geometry() {
  // Finding #5: a G02 arc with neither I nor J must error, not degenerate into a straight chord.
  assert!(matches!(try_parse("arc_no_offset.gbr"), Err(GerberError::InvalidGeometry { .. })));
}

#[test]
fn negative_image_polarity_is_refused_loudly() {
  // Finding #1: %IPNEG*% inverts the whole image; we refuse it rather than pass inverted-meaning geometry through.
  assert!(matches!(try_parse("image_neg_refused.gbr"), Err(GerberError::Unsupported { .. })));
}
