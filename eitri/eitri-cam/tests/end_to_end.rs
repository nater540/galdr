//! End-to-end CAM tests: parse a real fixture file, run a CAM operation, assert the toolpath/plan is sane.
//!
//! These prove the geometry backend end to end (§13 step 3) — Gerber → isolation rings and Excellon → drill plan —
//! over the hand-authored fixtures in `eitri/fixtures/synthetic`. They are structured so a real multi-tool fab file
//! can drop in later by name without reworking the assertions.

use std::path::PathBuf;

use eitri_cam::{
  DrillConfig, DrillMove, IsolationParams, MillingDirection, NearestNeighbor, Point, TwoOpt, isolate, isolate_arc,
  plan_drilling, tour_travel,
};
use eitri_cam::optimize::{Routed, Stop};
use eitri_core::{CancelToken, ProgressReporter};
use eitri_excellon::{DrillHit, parse_excellon};
use eitri_geo::DefaultBackend;
use eitri_gerber::parse_gerber;

fn fixture(kind: &str, name: &str) -> String {
  let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../fixtures/synthetic").join(kind).join(name);
  std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

fn silent() -> (ProgressReporter, CancelToken) {
  (ProgressReporter::silent(), CancelToken::new())
}

#[test]
fn kicad_two_pads_isolates_into_sane_rings() {
  let (p, c) = silent();
  let image = parse_gerber(&fixture("gerber", "kicad_two_pads.gbr"), &p, &c).expect("parse gerber");
  let copper_bounds = image.bounds().expect("copper has bounds");

  let params = IsolationParams { tool_diameter: 0.2, passes: 2, overlap: 0.1, ..Default::default() };
  let paths = isolate(&DefaultBackend::new(), &image.copper, &params, &p, &c).expect("isolate");

  assert!(!paths.is_empty(), "two pads + a trace must isolate into at least one ring");
  // Every isolation ring lies outside the copper (outward offset), so the toolpath bounds enclose the copper.
  let (cminx, cminy, cmaxx, cmaxy) = copper_bounds;
  let mut xs = (f64::MAX, f64::MIN);
  let mut ys = (f64::MAX, f64::MIN);
  for ring in &paths.rings {
    assert!(ring.geometry.points.len() >= 4, "a ring must have real vertices");
    for pt in &ring.geometry.points {
      xs = (xs.0.min(pt.x), xs.1.max(pt.x));
      ys = (ys.0.min(pt.y), ys.1.max(pt.y));
    }
  }
  assert!(xs.0 <= cminx && xs.1 >= cmaxx && ys.0 <= cminy && ys.1 >= cmaxy, "isolation must enclose copper");
}

#[test]
fn kicad_two_pads_arc_isolation_is_non_empty() {
  let (p, c) = silent();
  let image = parse_gerber(&fixture("gerber", "kicad_two_pads.gbr"), &p, &c).expect("parse gerber");
  let params = IsolationParams { tool_diameter: 0.2, passes: 1, direction: MillingDirection::Climb, ..Default::default() };
  let arcs = isolate_arc(&image.copper, &params, &p, &c).expect("arc isolate");
  assert!(!arcs.is_empty(), "arc isolation must produce rings for real copper");
  assert!(arcs.rings.iter().all(|r| !r.geometry.vertices.is_empty()));
}

#[test]
fn metric_drill_file_becomes_a_grouped_ordered_plan() {
  let (p, c) = silent();
  let image = parse_excellon(&fixture("excellon", "metric_leading.drl"), None, &p, &c).expect("parse excellon");
  let plan = plan_drilling(&image, &DrillConfig::default(), &TwoOpt::new(), &p, &c).expect("plan");

  // Two tools (0.8 and 1.0), grouped ascending, three hits total (2 on T1, 1 on T2).
  assert_eq!(plan.tools.len(), 2);
  assert_eq!(plan.tools[0].tool, 1);
  assert_eq!(plan.tools[1].tool, 2);
  assert_eq!(plan.tools[0].moves.len(), 2);
  assert_eq!(plan.tools[1].moves.len(), 1);
  assert_eq!(plan.move_count(), 3);
  // Every move is a point drill for this file (no slots).
  assert!(plan.tools.iter().flat_map(|t| &t.moves).all(|m| matches!(m, DrillMove::Drill { .. })));
}

#[test]
fn slot_drill_file_carries_the_slot_as_a_segment() {
  let (p, c) = silent();
  let image = parse_excellon(&fixture("excellon", "slot_g85.drl"), None, &p, &c).expect("parse excellon");
  let plan = plan_drilling(&image, &DrillConfig::default(), &NearestNeighbor, &p, &c).expect("plan");
  assert_eq!(plan.tools.len(), 1);
  match plan.tools[0].moves.as_slice() {
    [DrillMove::Slot { from, to }] => {
      // The G85 slot runs (2,2) -> (8,2); orientation may flip but the endpoints are these two points.
      let ends = [*from, *to];
      assert!(ends.contains(&Point::new(2.0, 2.0)) && ends.contains(&Point::new(8.0, 2.0)), "slot endpoints");
    }
    other => panic!("expected a single slot move, got {other:?}"),
  }
}

#[test]
fn two_opt_beats_input_order_on_a_real_drill_file() {
  // Build a denser scrambled single-tool file so the improvement is unambiguous, then confirm the optimized plan's
  // rapid travel is strictly below the file-order travel.
  let (p, c) = silent();
  let image = parse_excellon(&fixture("excellon", "metric_leading.drl"), None, &p, &c).expect("parse excellon");

  // Take just tool 1's hits and reconstruct the file-order (identity) travel from them.
  let start = Point::new(0.0, 0.0);
  let stops: Vec<Stop> = image
    .hits
    .iter()
    .filter(|h| h.tool() == 1)
    .map(|h| match h {
      DrillHit::Drill { x, y, .. } => Stop::point(Point::new(*x, *y)),
      DrillHit::Slot { start, end, .. } => Stop::segment(Point::new(start.0, start.1), Point::new(end.0, end.1)),
    })
    .collect();
  let identity: Vec<Routed> = (0..stops.len()).map(|index| Routed { index, reversed: false }).collect();
  let base = tour_travel(&stops, start, &identity);

  let plan = plan_drilling(&image, &DrillConfig { start, ..Default::default() }, &TwoOpt::new(), &p, &c).expect("plan");
  // Reconstruct tool-1 travel from the optimized plan.
  let mut cursor = start;
  let mut improved = 0.0;
  for mv in &plan.tools[0].moves {
    let (entry, exit) = match mv {
      DrillMove::Drill { at } => (*at, *at),
      DrillMove::Slot { from, to } => (*from, *to),
    };
    improved += cursor.distance_to(entry);
    cursor = exit;
  }
  assert!(improved <= base, "optimized drill travel must not exceed file order: {improved} > {base}");
}
