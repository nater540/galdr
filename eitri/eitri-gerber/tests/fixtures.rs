//! Integration tests over the hand-authored Gerber fixtures in `eitri/fixtures/gerber/`.
//!
//! These drive the whole pipeline (lex → interpret → assemble) on real files, complementing the unit tests: a
//! realistic copper layer, both zero-suppression modes, a macro-aperture flash, and an LP clear cutting a region.

use std::path::PathBuf;

use eitri_core::{CancelToken, ProgressReporter};
use eitri_gerber::{GerberImage, parse_gerber};

use geo::algorithm::area::Area;

fn fixture(name: &str) -> String {
  let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../fixtures/synthetic/gerber").join(name);
  std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

fn parse(name: &str) -> GerberImage {
  parse_gerber(&fixture(name), &ProgressReporter::silent(), &CancelToken::new()).expect("parse")
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
