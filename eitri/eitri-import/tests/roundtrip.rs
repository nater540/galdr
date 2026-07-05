//! Emit ↔ import round-trip tests.
//!
//! The strongest check on the G-code importer is that it recovers what the emitter produced. Two flavours:
//! a fully deterministic square (emit via the grblHAL postprocessor, import, assert the recovered contour is the
//! square), and a sanity pass over a real Phase-4 golden `.nc` (import, assert the recovered cut geometry is
//! non-empty and lands in the expected extents).

use eitri_cam::{IsolationRing, IsolationToolpaths, Point, RingPath};
use eitri_gcode::{GrblHal, IsolationJob, emit_isolation};
use eitri_geo::WindingDirection;
use eitri_import::import_gcode;

/// A closed 10 mm square ring, first == last so the emitter treats it as closed.
fn square_ring() -> RingPath {
  RingPath {
    points: vec![
      Point::new(0.0, 0.0),
      Point::new(10.0, 0.0),
      Point::new(10.0, 10.0),
      Point::new(0.0, 10.0),
      Point::new(0.0, 0.0),
    ],
  }
}

#[test]
fn emit_then_import_recovers_the_square_contour() {
  let toolpaths = IsolationToolpaths {
    rings: vec![IsolationRing { pass: 0, offset: 0.5, winding: WindingDirection::Ccw, geometry: square_ring() }],
  };
  let job = IsolationJob { cut_depth: 0.1, pass_depth: 0.1, ..Default::default() };
  let text = emit_isolation(&toolpaths, &job, &GrblHal::new()).render();

  let preview = import_gcode(&text).expect("import emitted g-code");
  assert!(preview.skipped.is_empty(), "clean linear g-code has nothing to skip: {:?}", preview.skipped);
  let polys = preview.cut_polylines();
  assert_eq!(polys.len(), 1, "one closed cut contour");
  let pts: Vec<(f64, f64)> = polys[0].0.iter().map(|c| (c.x, c.y)).collect();
  // The plunge leaves no XY trace, so the recovered contour is exactly the square's four edges back to the start.
  assert_eq!(pts, vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 0.0)]);
}

#[test]
fn golden_isolation_nc_imports_to_sane_geometry() {
  let nc = include_str!("../../fixtures/golden/isolation_two_pads.nc");
  let preview = import_gcode(nc).expect("import golden isolation nc");
  let polys = preview.cut_polylines();
  assert!(!polys.is_empty(), "the golden isolation nc must recover cut geometry");

  // The two-pad isolation lives roughly within x in [-1.3, 11.3], y in [-1.3, 1.3] (pads at x~0 and x~10, radius
  // ~1.2). Assert every recovered point sits inside a generous bounding box, i.e. the coordinates decoded sanely.
  for line in &polys {
    for c in &line.0 {
      assert!(c.x > -2.0 && c.x < 12.0, "x out of expected range: {}", c.x);
      assert!(c.y > -2.0 && c.y < 2.0, "y out of expected range: {}", c.y);
    }
  }
  // The isolation is a closed contour, so the recovered polyline returns to its start.
  let first = polys[0].0.first().copied().expect("point");
  let last = polys[0].0.last().copied().expect("point");
  assert!((first.x - last.x).abs() < 1e-6 && (first.y - last.y).abs() < 1e-6, "isolation contour should close");
}

#[test]
fn golden_drill_nc_imports_and_visits_every_hole() {
  let nc = include_str!("../../fixtures/golden/drill_metric_leading.nc");
  let preview = import_gcode(nc).expect("import golden drill nc");
  // Drilling is plunges at rapided-to positions; the geometry is the hole XY, which appear as rapid-move targets.
  let targets: Vec<(f64, f64)> = preview
    .moves
    .iter()
    .filter(|m| m.kind == eitri_import::MotionKind::Rapid)
    .map(|m| (m.to.x, m.to.y))
    .collect();
  for hole in [(10.0, 5.0), (20.0, 5.0), (15.0, 10.0)] {
    assert!(targets.iter().any(|t| (t.0 - hole.0).abs() < 1e-6 && (t.1 - hole.1).abs() < 1e-6), "missing hole {hole:?}");
  }
}
