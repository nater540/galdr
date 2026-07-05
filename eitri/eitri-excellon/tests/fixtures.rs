//! Integration tests over the hand-authored Excellon fixtures in `eitri/fixtures/excellon/`.
//!
//! Covers the format-inference matrix (metric/inch × leading/trailing), an undeclared-format fallback, and a G85
//! slot becoming a segment with buffered geometry.

use std::path::PathBuf;

use eitri_core::{CancelToken, ProgressReporter, Unit};
use eitri_excellon::{DrillHit, ExcellonImage, ZeroSuppression, parse_excellon};

use geo::algorithm::area::Area;

fn fixture(name: &str) -> String {
  let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../fixtures/synthetic/excellon").join(name);
  std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

fn parse(name: &str) -> ExcellonImage {
  parse_excellon(&fixture(name), None, &ProgressReporter::silent(), &CancelToken::new()).expect("parse")
}

#[test]
fn metric_leading_suppression_infers_and_decodes() {
  let img = parse("metric_leading.drl");
  assert_eq!(img.format.unit, Unit::Millimeters);
  assert_eq!(img.format.zero_suppression, ZeroSuppression::Leading); // TZ keyword
  assert_eq!(img.tools.len(), 2);
  assert!((img.tools[&1].diameter - 0.8).abs() < 1e-9);
  assert_eq!(img.hits.len(), 3);
  match img.hits[0] {
    DrillHit::Drill { x, y, tool } => {
      assert_eq!(tool, 1);
      assert!((x - 10.0).abs() < 1e-6 && (y - 5.0).abs() < 1e-6, "hit at ({x}, {y})");
    }
    other => panic!("expected drill, got {other:?}"),
  }
}

#[test]
fn inch_trailing_suppression_infers_and_converts_to_mm() {
  let img = parse("inch_trailing.drl");
  assert_eq!(img.format.unit, Unit::Inches);
  assert_eq!(img.format.zero_suppression, ZeroSuppression::Trailing); // LZ keyword
  // 0.04 inch tool -> ~1.016 mm.
  assert!((img.tools[&1].diameter - 1.016).abs() < 1e-3, "dia {}", img.tools[&1].diameter);
  match img.hits[0] {
    DrillHit::Drill { x, y, .. } => assert!((x - 25.4).abs() < 1e-3 && (y - 12.7).abs() < 1e-3, "hit ({x}, {y})"),
    other => panic!("expected drill, got {other:?}"),
  }
}

#[test]
fn undeclared_format_falls_back_and_parses_decimals() {
  let img = parse("undeclared_decimal.drl");
  assert_eq!(img.format.unit, Unit::Millimeters);
  assert_eq!(img.hits.len(), 2);
  match img.hits[0] {
    DrillHit::Drill { x, y, .. } => assert!((x - 3.0).abs() < 1e-9 && (y - 4.0).abs() < 1e-9),
    other => panic!("expected drill, got {other:?}"),
  }
}

#[test]
fn g85_slot_is_a_segment_with_buffered_geometry() {
  let img = parse("slot_g85.drl");
  assert_eq!(img.hits.len(), 1);
  match img.hits[0] {
    DrillHit::Slot { start, end, tool } => {
      assert_eq!(tool, 1);
      assert!((start.0 - 2.0).abs() < 1e-6 && (start.1 - 2.0).abs() < 1e-6);
      assert!((end.0 - 8.0).abs() < 1e-6 && (end.1 - 2.0).abs() < 1e-6);
    }
    other => panic!("expected slot, got {other:?}"),
  }
  // A 6mm-long slot with a 1mm tool (r=0.5): capsule area 6*1 + pi*0.25 ≈ 6.785.
  let geom = img.hit_geometry(&img.hits[0]).unwrap().unwrap();
  assert!((geom.unsigned_area() - (6.0 + std::f64::consts::PI * 0.25)).abs() < 0.05, "area {}", geom.unsigned_area());
}

#[test]
fn combined_tool_select_and_coordinate_line_drills() {
  // Finding #7: a `T1X..Y..` line must select the tool AND drill; the old code dropped the hit.
  let img = parse("combined_tool_hit.drl");
  assert_eq!(img.hits.len(), 1, "the combined select+coordinate line must emit a hit");
  match img.hits[0] {
    DrillHit::Drill { tool, x, y } => {
      assert_eq!(tool, 1);
      assert!((x - 10.0).abs() < 1e-6 && (y - 5.0).abs() < 1e-6, "hit at ({x}, {y})");
    }
    other => panic!("expected drill, got {other:?}"),
  }
}

#[test]
fn large_drill_facets_adaptively() {
  // Finding #10: a 6mm drill uses the shared adaptive circle builder, so it exceeds the old fixed 48 facets.
  let img = parse("large_drill.drl");
  let geom = img.hit_geometry(&img.hits[0]).unwrap().unwrap();
  let facets = geom.0[0].exterior().0.len();
  assert!(facets > 48, "a 6mm drill should exceed the old fixed 48 facets, got {facets}");
}
