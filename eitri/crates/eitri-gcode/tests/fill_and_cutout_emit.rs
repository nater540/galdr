//! Phase-5 reuse proof: paint (fill) and cutout (profile) toolpaths reach G-code through the **existing** Phase-4
//! [`emit_isolation`] emitter — there is no parallel G-code path. Each operation's `.toolpaths()` yields the same
//! `IsolationToolpaths<RingPath>` the isolation router produces, so the one emitter serves them all. The tests
//! assert the emitted programs are grblHAL/Skirnir-contract conformant and exercise the open-path multi-depth
//! generalization (a cutout arc, being open, lifts and returns to its start between depth passes).

use eitri_cam::{
  Concentric, CutoutOutline, CutoutParams, PaintParams, Point, TabPlacement, cutout, paint,
};
use eitri_core::{CancelToken, ProgressReporter};
use eitri_gcode::{GrblHal, IsolationJob, Origin, check_grbl_conformance, emit_isolation};
use eitri_geo::DefaultBackend;
use eitri_geo::geo_types::{Coord, LineString, MultiPolygon, Polygon};

fn silent() -> (ProgressReporter, CancelToken) {
  (ProgressReporter::silent(), CancelToken::new())
}

fn square(cx: f64, cy: f64, half: f64) -> Polygon<f64> {
  Polygon::new(
    LineString(vec![
      Coord { x: cx - half, y: cy - half },
      Coord { x: cx + half, y: cy - half },
      Coord { x: cx + half, y: cy + half },
      Coord { x: cx - half, y: cy + half },
      Coord { x: cx - half, y: cy - half },
    ]),
    vec![],
  )
}

#[test]
fn paint_fill_emits_conformant_gcode_through_the_isolation_emitter() {
  let region = MultiPolygon::new(vec![square(0.0, 0.0, 10.0)]);
  let params = PaintParams { tool_diameter: 2.0, overlap: 0.3, ..Default::default() };
  let (p, c) = silent();
  let result = paint(&region, &params, &Concentric, &DefaultBackend::new(), &p, &c).expect("paint");
  assert!(!result.is_empty(), "the fill must produce paths to emit");

  // The reuse point: paint output becomes IsolationToolpaths and flows through the SAME emitter as isolation.
  let toolpaths = result.toolpaths();
  let job = IsolationJob { cut_depth: 0.1, pass_depth: 0.1, ..Default::default() };
  let prog = emit_isolation(&toolpaths, &job, Origin::NATIVE, &GrblHal::new(), None);
  let text = prog.render();

  let violations = check_grbl_conformance(&text);
  assert!(violations.is_empty(), "paint G-code must be grbl-conformant, got: {violations:?}");
  assert!(text.contains("G1 Z-0.1000"), "the fill plunges to depth:\n{text}");
}

#[test]
fn cutout_profile_emits_conformant_gcode_with_open_path_multi_depth() {
  let outline = CutoutOutline::Rectangle { min: Point::new(0.0, 0.0), max: Point::new(20.0, 20.0) };
  let params = CutoutParams { tool_diameter: 1.6, tab_width: 3.0, tabs: TabPlacement::Count(4), ..Default::default() };
  let (p, c) = silent();
  let result = cutout(&outline, &params, &DefaultBackend::new(), &p, &c).expect("cutout");
  assert_eq!(result.len(), 4, "four tabs => four open cut arcs");
  assert!(result.paths.iter().all(|a| !a.is_closed()), "cut arcs are open paths");

  // Multi-depth cut of the open arcs through the shared emitter (0.4mm total at 0.1mm/pass => 4 passes).
  let toolpaths = result.toolpaths();
  let job = IsolationJob { cut_depth: 0.4, pass_depth: 0.1, cut_feed: 200.0, plunge_feed: 60.0, ..Default::default() };
  let prog = emit_isolation(&toolpaths, &job, Origin::NATIVE, &GrblHal::new(), None);
  let text = prog.render();

  let violations = check_grbl_conformance(&text);
  assert!(violations.is_empty(), "cutout G-code must be grbl-conformant, got: {violations:?}");

  // Four arcs each cut in four depth passes => sixteen plunges.
  let plunges = prog.lines().iter().filter(|l| l.starts_with("G1 Z-")).count();
  assert_eq!(plunges, 16, "four open arcs x four depth passes => sixteen plunges");

  // The open-path generalization: between depth passes the tool lifts to the safe height and rapids back — so there
  // are interior safe-Z lifts beyond the one-per-arc final lift (4 arcs => >4 safe-Z rapids).
  let lifts = prog.lines().iter().filter(|l| l.as_str() == "G0 Z2.0000").count();
  assert!(lifts > result.len(), "open arcs lift between depth passes, got {lifts} safe-Z rapids for {} arcs", result.len());
}
