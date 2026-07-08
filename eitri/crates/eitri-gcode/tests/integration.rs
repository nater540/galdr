//! End-to-end tests over real fixtures: parse a Gerber / Excellon, run the Phase-3 CAM operation, emit G-code, and
//! (1) assert it conforms to the grblHAL/Skirnir contract and (2) byte-compare it against a committed golden file.
//!
//! The golden files live under `eitri/fixtures/golden/`. Set `EITRI_REGEN_GOLDEN=1` to (re)write them deliberately
//! after an intended output change; the default run asserts byte-equality so an accidental drift fails loudly.

use std::path::PathBuf;

use eitri_cam::{DrillConfig, IsolationParams, MillingDirection, NearestNeighbor, isolate, plan_drilling};
use eitri_core::{CancelToken, ProgressReporter};
use eitri_gcode::{DrillJob, GrblHal, IsolationJob, Origin, check_grbl_conformance, emit_drilling, emit_isolation};
use eitri_geo::{DefaultBackend, JoinType};

/// Absolute path to a fixture under `eitri/fixtures/`.
fn fixture(rel: &str) -> PathBuf {
  PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../fixtures").join(rel)
}

/// Read a fixture file to a string.
fn read_fixture(rel: &str) -> String {
  std::fs::read_to_string(fixture(rel)).unwrap_or_else(|e| panic!("read fixture {rel}: {e}"))
}

/// Compare `actual` to the committed golden at `golden/<name>`, or rewrite it when `EITRI_REGEN_GOLDEN` is set.
fn assert_golden(name: &str, actual: &str) {
  let path = fixture(&format!("golden/{name}"));
  if std::env::var("EITRI_REGEN_GOLDEN").is_ok() {
    if let Some(parent) = path.parent() {
      std::fs::create_dir_all(parent).expect("create golden dir");
    }
    std::fs::write(&path, actual).expect("write golden");
    return;
  }
  let expected = std::fs::read_to_string(&path)
    .unwrap_or_else(|e| panic!("read golden {name} ({e}); regenerate with EITRI_REGEN_GOLDEN=1"));
  assert_eq!(actual, expected, "golden mismatch for {name}; regenerate with EITRI_REGEN_GOLDEN=1 if intended");
}

/// Build the isolation program for the two-pad KiCad fixture.
fn isolation_program() -> String {
  let src = read_fixture("synthetic/gerber/kicad_two_pads.gbr");
  let img = eitri_gerber::parse_gerber(&src, &ProgressReporter::silent(), &CancelToken::new()).expect("parse gerber");
  let params = IsolationParams {
    tool_diameter: 0.4,
    passes: 1,
    overlap: 0.0,
    combine: false,
    direction: MillingDirection::Climb,
    join: JoinType::Round,
    miter_limit: 2.0,
  };
  let paths = isolate(&DefaultBackend::new(), &img.copper, &params, &ProgressReporter::silent(), &CancelToken::new())
    .expect("isolate");
  let job = IsolationJob {
    cut_depth: 0.15,
    pass_depth: 0.15,
    cut_feed: 120.0,
    plunge_feed: 40.0,
    travel_z: 2.0,
    spindle_rpm: 10000.0,
    name: Some("isolation kicad_two_pads".to_string()),
  };
  emit_isolation(&paths, &job, Origin::NATIVE, &GrblHal::new()).render()
}

/// Build the drilling program for the metric Excellon fixture.
fn drilling_program() -> String {
  let src = read_fixture("synthetic/excellon/metric_leading.drl");
  let img = eitri_excellon::parse_excellon(&src, None, &ProgressReporter::silent(), &CancelToken::new())
    .expect("parse excellon");
  let config = DrillConfig::default();
  let plan = plan_drilling(&img, &config, &NearestNeighbor, &ProgressReporter::silent(), &CancelToken::new())
    .expect("plan drilling");
  let job = DrillJob { travel_z: 3.0, spindle_rpm: 10000.0, name: Some("drill metric_leading".to_string()) };
  emit_drilling(&plan, &job, Origin::NATIVE, &GrblHal::new()).render()
}

#[test]
fn isolation_output_is_contract_conformant() {
  let gcode = isolation_program();
  let violations = check_grbl_conformance(&gcode);
  assert!(violations.is_empty(), "isolation output violates the contract: {violations:#?}");
}

#[test]
fn drilling_output_is_contract_conformant() {
  let gcode = drilling_program();
  let violations = check_grbl_conformance(&gcode);
  assert!(violations.is_empty(), "drilling output violates the contract: {violations:#?}");
}

#[test]
fn isolation_matches_golden() {
  assert_golden("isolation_two_pads.nc", &isolation_program());
}

#[test]
fn drilling_matches_golden() {
  assert_golden("drill_metric_leading.nc", &drilling_program());
}

#[test]
fn arc_ring_emits_correct_sweep_direction() {
  // Round-trip sanity: a CCW quarter-circle arc must emit G3 with the centre offset pointing at the origin, and
  // its clockwise reverse must emit G2. This guards the bulge→I/J sweep-direction mapping independent of any golden.
  use eitri_gcode::{ArcDir, bulge_to_arc};
  let b = (std::f64::consts::FRAC_PI_8).tan();
  let (ccw_off, ccw_dir) = bulge_to_arc(1.0, 0.0, 0.0, 1.0, b).expect("ccw arc");
  assert_eq!(ccw_dir, ArcDir::Ccw);
  assert_eq!(ccw_dir.word(), "G3");
  assert!((ccw_off.i + 1.0).abs() < 1e-9 && ccw_off.j.abs() < 1e-9);
  let (_cw_off, cw_dir) = bulge_to_arc(0.0, 1.0, 1.0, 0.0, -b).expect("cw arc");
  assert_eq!(cw_dir, ArcDir::Cw);
  assert_eq!(cw_dir.word(), "G2");
}
