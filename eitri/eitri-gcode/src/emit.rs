//! Toolpath / operation → motion emitter.
//!
//! This is the half of Phase 4 that decides *motions*; the [`Postprocessor`] renders each as text. The emitter walks
//! an isolation toolpath or a drill plan and drives the postprocessor hooks: preamble, spindle on, rapid to a start,
//! plunge, cut (linear or arc), lift, peck, dwell, tool change, end. Keeping this dialect-agnostic means one motion
//! model feeds every controller.
//!
//! Two seams keep it testable and reusable:
//! - [`CutRing`] abstracts a closed cut ring into an ordered list of [`Segment`]s, so the linear [`RingPath`] and the
//!   arc-preserving [`ArcPolyline`] share one emission path. The arc impl is *why* Phase 3 kept native bulges: each
//!   arc segment becomes a `G2`/`G3` with I/J, never a linearized substitute.
//! - Multi-depth passes are one shared mechanism ([`depth_steps`]) so isolation, and later paint/cutout, repeat a
//!   path at increasing depths until the final depth is reached.

use eitri_cam::{DrillMove, DrillParams, DrillPlan, IsolationToolpaths, RingPath};
use eitri_geo::ArcPolyline;

use crate::arc::{ArcDir, ArcOffset, bulge_to_arc};
use crate::post::{ArcMove, Axes, JobContext, Postprocessor, Spindle, ToolChange};
use crate::program::Program;

/// Cutting parameters for an isolation (or, later, any 2.5D contour) job. Depths are positive magnitudes below the
/// work surface; `travel_z` is a positive clearance height above it. Feeds are millimetres per minute.
#[derive(Debug, Clone, PartialEq)]
pub struct IsolationJob {
  /// Total cut depth below the surface (positive); the ring is cut down to `z = -cut_depth`.
  pub cut_depth: f64,
  /// Maximum depth removed per pass (positive). If it is zero or `>= cut_depth`, the ring is cut in a single pass;
  /// otherwise the path repeats at increasing depths until `cut_depth` is reached.
  pub pass_depth: f64,
  /// Feed rate for cutting moves (millimetres per minute).
  pub cut_feed: f64,
  /// Feed rate for plunge (Z-down) moves (millimetres per minute).
  pub plunge_feed: f64,
  /// Safe rapid height above the work surface (positive).
  pub travel_z: f64,
  /// Spindle speed (RPM).
  pub spindle_rpm: f64,
  /// Optional job name for the header comment.
  pub name: Option<String>,
}

impl Default for IsolationJob {
  fn default() -> IsolationJob {
    IsolationJob {
      cut_depth: 0.1,
      pass_depth: 0.1,
      cut_feed: 120.0,
      plunge_feed: 60.0,
      travel_z: 2.0,
      spindle_rpm: 10000.0,
      name: None,
    }
  }
}

/// Job-level settings for a drilling program. Per-tool depth/feed/retract/peck/dwell ride on each tool's
/// [`DrillParams`]; this carries only the cross-tool rapid height and spindle speed.
#[derive(Debug, Clone, PartialEq)]
pub struct DrillJob {
  /// Safe rapid height for repositioning and tool changes (positive, above the surface).
  pub travel_z: f64,
  /// Spindle speed (RPM).
  pub spindle_rpm: f64,
  /// Optional job name for the header comment.
  pub name: Option<String>,
}

impl Default for DrillJob {
  fn default() -> DrillJob {
    DrillJob { travel_z: 3.0, spindle_rpm: 10000.0, name: None }
  }
}

/// One segment of a cut ring: a straight line to a point, or an arc to a point with an I/J centre offset.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Segment {
  /// A straight cut to `(x, y)`.
  Line {
    /// Destination X (millimetres).
    x: f64,
    /// Destination Y (millimetres).
    y: f64,
  },
  /// An arc cut to `(x, y)` about a centre given by `offset`, turning `dir`.
  Arc {
    /// Destination X (millimetres).
    x: f64,
    /// Destination Y (millimetres).
    y: f64,
    /// Centre offset relative to the segment start.
    offset: ArcOffset,
    /// Turning direction (`G2`/`G3`).
    dir: ArcDir,
  },
}

/// A cut path reduced to a start point and an ordered list of segments. For a **closed** ring the segments return
/// to the start (isolation rings, concentric/seed paint rings); for an **open** path they end elsewhere (raster
/// paint passes, cutout arcs between tabs). [`is_closed`](CutRing::is_closed) tells the emitter which, so multi-depth
/// passes plunge in place on a closed ring but lift and return to the start on an open one. Implemented for both
/// ring flavours so [`emit_isolation`] is generic over linear and arc-preserving toolpaths.
pub trait CutRing {
  /// The path's start point `(x, y)` — where the tool plunges before cutting.
  fn start(&self) -> (f64, f64);

  /// The ordered segments cutting along the path; for a closed ring they return to the start.
  fn segments(&self) -> Vec<Segment>;

  /// Whether the path is closed (its last cut point coincides with [`start`](CutRing::start)). Closed paths let a
  /// multi-depth cut plunge straight down between passes; open paths must lift and rapid back to the start first.
  fn is_closed(&self) -> bool;
}

impl CutRing for RingPath {
  fn start(&self) -> (f64, f64) {
    let p = RingPath::start(self);
    (p.x, p.y)
  }

  fn segments(&self) -> Vec<Segment> {
    // Cut to each subsequent point in turn; on a closed ring (first == last) this returns to the start, on an open
    // path it ends at the final vertex.
    self.points.iter().skip(1).map(|p| Segment::Line { x: p.x, y: p.y }).collect()
  }

  fn is_closed(&self) -> bool {
    // Delegate to the inherent closed-ness test the CAM crate owns, so the convention has one definition.
    RingPath::is_closed(self)
  }
}

impl CutRing for ArcPolyline {
  fn start(&self) -> (f64, f64) {
    self.vertices.first().map(|v| (v.x, v.y)).unwrap_or((0.0, 0.0))
  }

  fn segments(&self) -> Vec<Segment> {
    let n = self.vertices.len();
    let mut segs = Vec::with_capacity(n);
    for i in 0..n {
      let v0 = self.vertices[i];
      let v1 = self.vertices[(i + 1) % n];
      match bulge_to_arc(v0.x, v0.y, v1.x, v1.y, v0.bulge) {
        Some((offset, dir)) => segs.push(Segment::Arc { x: v1.x, y: v1.y, offset, dir }),
        None => segs.push(Segment::Line { x: v1.x, y: v1.y }),
      }
    }
    segs
  }

  fn is_closed(&self) -> bool {
    // Arc polylines model closed offset rings — `segments` wraps the last vertex back to the first.
    true
  }
}

/// The descending Z depth levels (all negative) for cutting `total` depth in steps no deeper than `step`, always
/// ending exactly at `-total`. A non-positive or `>= total` step yields a single `[-total]` pass.
pub fn depth_steps(total: f64, step: f64) -> Vec<f64> {
  if step <= 0.0 || step >= total {
    return vec![-total];
  }
  let mut levels = Vec::new();
  let mut cut = step;
  while cut < total - 1.0e-9 {
    levels.push(-cut);
    cut += step;
  }
  levels.push(-total);
  levels
}

/// Emit a complete isolation program from `toolpaths` using `post`. Rings are cut in the order Phase 3 produced
/// them; each ring is rapided to at safe height, plunged (multi-depth if configured), cut, and lifted.
pub fn emit_isolation<G: CutRing>(
  toolpaths: &IsolationToolpaths<G>,
  job: &IsolationJob,
  post: &dyn Postprocessor,
) -> Program {
  let mut prog = Program::new(post.format());
  let ctx = JobContext { name: job.name.clone() };
  post.start_code(&mut prog, &ctx);

  if !toolpaths.rings.is_empty() {
    post.spindle_on(&mut prog, Spindle { rpm: job.spindle_rpm });
    // Establish the safe-height invariant once; every ring restores it on exit.
    post.rapid(&mut prog, Axes::z(job.travel_z));

    let depths = depth_steps(job.cut_depth, job.pass_depth);
    for ring in &toolpaths.rings {
      let (sx, sy) = ring.geometry.start();
      let segments = ring.geometry.segments();
      let closed = ring.geometry.is_closed();
      post.rapid(&mut prog, Axes::xy(sx, sy));
      for (pass, &z) in depths.iter().enumerate() {
        // A closed ring ends each pass back at its start, so the next deeper pass just plunges in place. An open
        // path (a raster fill row, a cutout arc between tabs) ends elsewhere, so lift and rapid back to the start
        // before plunging again — never drag the tool through material back to the start. The first pass always
        // plunges in place: the rapid to the start already put the tool there.
        if pass > 0 && !closed {
          post.rapid(&mut prog, Axes::z(job.travel_z));
          post.rapid(&mut prog, Axes::xy(sx, sy));
        }
        post.linear(&mut prog, Axes::z(z), job.plunge_feed);
        emit_ring_segments(&mut prog, post, &segments, job.cut_feed);
      }
      post.rapid(&mut prog, Axes::z(job.travel_z));
    }
  }

  // The spindle is stopped by `end_code`, keeping the postamble (spindle off + program end) in one place.
  post.end_code(&mut prog, &ctx);
  prog
}

/// Cut each segment of a ring at `feed`, dispatching linear vs arc to the postprocessor.
fn emit_ring_segments(prog: &mut Program, post: &dyn Postprocessor, segments: &[Segment], feed: f64) {
  for seg in segments {
    match *seg {
      Segment::Line { x, y } => post.linear(prog, Axes::xy(x, y), feed),
      Segment::Arc { x, y, offset, dir } => post.arc(prog, ArcMove { x, y, offset, dir }, feed),
    }
  }
}

/// Emit a complete drilling program from `plan` using `post`. Each tool group gets a tool change and spindle start,
/// then every hit is drilled (with a manual peck cycle when configured — grbl has no canned `G81`/`G83`) or every
/// slot is plunged and routed.
pub fn emit_drilling(plan: &DrillPlan, job: &DrillJob, post: &dyn Postprocessor) -> Program {
  let mut prog = Program::new(post.format());
  let ctx = JobContext { name: job.name.clone() };
  post.start_code(&mut prog, &ctx);

  if !plan.tools.is_empty() {
    post.rapid(&mut prog, Axes::z(job.travel_z));
    for tool in &plan.tools {
      post.tool_change(&mut prog, ToolChange { number: tool.tool, diameter: tool.diameter });
      post.spindle_on(&mut prog, Spindle { rpm: job.spindle_rpm });
      // Lift to the cross-tool safe height after the change before repositioning.
      post.rapid(&mut prog, Axes::z(job.travel_z));

      for mv in &tool.moves {
        match *mv {
          DrillMove::Drill { at } => {
            post.rapid(&mut prog, Axes::xy(at.x, at.y));
            drill_hole(&mut prog, post, &tool.params);
          }
          DrillMove::Slot { from, to } => {
            post.rapid(&mut prog, Axes::xy(from.x, from.y));
            route_slot(&mut prog, post, to.x, to.y, &tool.params);
          }
        }
      }
      // Restore the safe height for the next tool change / program end.
      post.rapid(&mut prog, Axes::z(job.travel_z));
    }
  }

  post.end_code(&mut prog, &ctx);
  prog
}

/// Clearance above the previous peck depth that the rapid-down returns to before feeding the next increment, so a
/// peck cycle does not slowly feed-cut back through already-cleared material.
const PECK_RAPID_CLEARANCE: f64 = 0.1;

/// Drill one hole at the current XY: a single plunge, or a manual peck cycle expanded into explicit plunge/retract
/// moves because grbl supports no `G83`. A final dwell (if configured) happens at the bottom before the last retract.
fn drill_hole(prog: &mut Program, post: &dyn Postprocessor, params: &DrillParams) {
  match params.peck {
    Some(peck) if peck > 0.0 && peck < params.depth => {
      let mut prev = 0.0;
      loop {
        let target = (prev + peck).min(params.depth);
        if prev > 0.0 {
          // Rapid back down to just above the last cut depth, then feed the fresh increment.
          post.rapid(prog, Axes::z(-(prev - PECK_RAPID_CLEARANCE)));
        }
        post.linear(prog, Axes::z(-target), params.feed);
        let last = target >= params.depth - 1.0e-9;
        if last {
          if let Some(dwell) = params.dwell {
            post.dwell(prog, dwell);
          }
        }
        post.rapid(prog, Axes::z(params.retract));
        prev = target;
        if last {
          break;
        }
      }
    }
    _ => {
      post.linear(prog, Axes::z(-params.depth), params.feed);
      if let Some(dwell) = params.dwell {
        post.dwell(prog, dwell);
      }
      post.rapid(prog, Axes::z(params.retract));
    }
  }
}

/// Route a slot: plunge to full depth at the current start, feed across to `(x, y)`, optionally dwell, then retract.
/// Slots plunge in a single pass (peck is a point-drill concern).
fn route_slot(prog: &mut Program, post: &dyn Postprocessor, x: f64, y: f64, params: &DrillParams) {
  post.linear(prog, Axes::z(-params.depth), params.feed);
  post.linear(prog, Axes::xy(x, y), params.feed);
  if let Some(dwell) = params.dwell {
    post.dwell(prog, dwell);
  }
  post.rapid(prog, Axes::z(params.retract));
}

#[cfg(test)]
mod tests {
  use super::*;
  use eitri_cam::{IsolationRing, Point, ToolDrillPlan};
  use eitri_geo::{ArcVertex, WindingDirection};

  use crate::GrblHal;

  fn grbl() -> GrblHal {
    GrblHal::new()
  }

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

  fn linear_toolpaths(ring: RingPath) -> IsolationToolpaths<RingPath> {
    IsolationToolpaths {
      rings: vec![IsolationRing { pass: 0, offset: 0.5, winding: WindingDirection::Ccw, geometry: ring }],
    }
  }

  #[test]
  fn depth_steps_single_pass_when_step_covers_depth() {
    assert_eq!(depth_steps(0.1, 0.1), vec![-0.1]);
    assert_eq!(depth_steps(0.1, 0.5), vec![-0.1]);
    assert_eq!(depth_steps(0.1, 0.0), vec![-0.1]);
  }

  #[test]
  fn depth_steps_divides_into_increasing_passes_ending_at_total() {
    assert_eq!(depth_steps(0.6, 0.25), vec![-0.25, -0.5, -0.6]);
    let steps = depth_steps(1.0, 0.5);
    assert_eq!(steps, vec![-0.5, -1.0]);
  }

  #[test]
  fn isolation_emits_preamble_spindle_plunge_cut_and_end() {
    let paths = linear_toolpaths(square_ring());
    let job = IsolationJob { cut_depth: 0.1, pass_depth: 0.1, ..Default::default() };
    let prog = emit_isolation(&paths, &job, &grbl());
    let text = prog.render();
    assert!(text.contains("G90 G21 G54 G17 G94"), "preamble missing:\n{text}");
    assert!(text.contains("M3 S10000"), "spindle on missing:\n{text}");
    assert!(text.contains("G1 Z-0.1000 F60.0"), "plunge missing:\n{text}");
    // A cut move across the top edge, with a feed word.
    assert!(text.contains("G1 X10.0000 Y0.0000 F120.0"), "cut move missing:\n{text}");
    assert!(text.trim_end().ends_with("M2"), "program should end with M2:\n{text}");
  }

  #[test]
  fn isolation_rapid_moves_never_carry_a_feed_word() {
    let paths = linear_toolpaths(square_ring());
    let prog = emit_isolation(&paths, &IsolationJob::default(), &grbl());
    for line in prog.lines() {
      if line.starts_with("G0") {
        assert!(!line.contains('F'), "rapid must not carry F: {line}");
      }
    }
  }

  #[test]
  fn closed_ring_is_detected_open_path_is_not() {
    assert!(square_ring().is_closed(), "a ring whose first == last is closed");
    let open = RingPath { points: vec![Point::new(0.0, 0.0), Point::new(10.0, 0.0), Point::new(10.0, 10.0)] };
    assert!(!open.is_closed(), "a path whose endpoints differ is open");
  }

  #[test]
  fn open_path_multi_depth_lifts_and_returns_to_start_between_passes() {
    // An open three-point path cut in three depth passes. Each pass must plunge at the same start, so there is one
    // rapid to the start XY per pass (initial + a return before passes 2 and 3), and a plunge per pass.
    let open = RingPath { points: vec![Point::new(0.0, 0.0), Point::new(10.0, 0.0), Point::new(10.0, 10.0)] };
    let paths = IsolationToolpaths {
      rings: vec![IsolationRing { pass: 0, offset: 0.0, winding: WindingDirection::Ccw, geometry: open }],
    };
    let job = IsolationJob { cut_depth: 0.3, pass_depth: 0.1, ..Default::default() };
    let prog = emit_isolation(&paths, &job, &grbl());
    let lines = prog.lines();
    let starts = lines.iter().filter(|l| l.as_str() == "G0 X0.0000 Y0.0000").count();
    assert_eq!(starts, 3, "open path returns to start once per depth pass:\n{lines:?}");
    let plunges = lines.iter().filter(|l| l.starts_with("G1 Z-")).count();
    assert_eq!(plunges, 3, "three depth passes => three plunges");
  }

  #[test]
  fn closed_ring_multi_depth_plunges_in_place_without_extra_returns() {
    // The regression guard for the open-path generalization: a closed ring must still rapid to its start exactly
    // once (no per-pass return), so existing isolation output is byte-for-byte unchanged.
    let paths = linear_toolpaths(square_ring());
    let job = IsolationJob { cut_depth: 0.3, pass_depth: 0.1, ..Default::default() };
    let prog = emit_isolation(&paths, &job, &grbl());
    let starts = prog.lines().iter().filter(|l| l.as_str() == "G0 X0.0000 Y0.0000").count();
    assert_eq!(starts, 1, "a closed ring rapids to its start only once");
  }

  #[test]
  fn multi_depth_produces_a_plunge_per_level() {
    let paths = linear_toolpaths(square_ring());
    let job = IsolationJob { cut_depth: 0.3, pass_depth: 0.1, ..Default::default() };
    let prog = emit_isolation(&paths, &job, &grbl());
    let plunges = prog.lines().iter().filter(|l| l.starts_with("G1 Z-")).count();
    assert_eq!(plunges, 3, "0.3mm at 0.1mm/pass => three plunges");
  }

  #[test]
  fn arc_ring_emits_g2_g3_with_ij() {
    // A CCW quarter-circle arc segment then a straight closing segment. The first vertex bulges CCW to the second.
    let ring = ArcPolyline {
      vertices: vec![
        ArcVertex { x: 1.0, y: 0.0, bulge: (std::f64::consts::FRAC_PI_8).tan() },
        ArcVertex { x: 0.0, y: 1.0, bulge: 0.0 },
      ],
    };
    let paths = IsolationToolpaths {
      rings: vec![IsolationRing { pass: 0, offset: 0.5, winding: WindingDirection::Ccw, geometry: ring }],
    };
    let prog = emit_isolation(&paths, &IsolationJob::default(), &grbl());
    let text = prog.render();
    assert!(text.contains("G3 X0.0000 Y1.0000 I-1.0000 J0.0000 F"), "arc move missing:\n{text}");
  }

  #[test]
  fn drilling_emits_tool_change_and_a_plunge_per_hole() {
    let plan = DrillPlan {
      tools: vec![ToolDrillPlan {
        tool: 1,
        diameter: 0.8,
        params: DrillParams { depth: 1.6, feed: 100.0, retract: 2.0, peck: None, dwell: None },
        moves: vec![
          DrillMove::Drill { at: Point::new(1.0, 2.0) },
          DrillMove::Drill { at: Point::new(3.0, 4.0) },
        ],
      }],
    };
    let prog = emit_drilling(&plan, &DrillJob::default(), &grbl());
    let text = prog.render();
    assert!(text.contains("M6 T1"), "tool change missing:\n{text}");
    assert!(text.contains("G0 X1.0000 Y2.0000"), "rapid to first hole missing:\n{text}");
    let plunges = prog.lines().iter().filter(|l| l.contains("G1 Z-1.6000")).count();
    assert_eq!(plunges, 2, "two holes => two plunges");
  }

  #[test]
  fn peck_cycle_expands_into_multiple_plunges_and_retracts() {
    let plan = DrillPlan {
      tools: vec![ToolDrillPlan {
        tool: 1,
        diameter: 0.8,
        params: DrillParams { depth: 1.0, feed: 80.0, retract: 1.5, peck: Some(0.4), dwell: Some(0.2) },
        moves: vec![DrillMove::Drill { at: Point::new(0.0, 0.0) }],
      }],
    };
    let prog = emit_drilling(&plan, &DrillJob::default(), &grbl());
    // 1.0mm at 0.4mm pecks => plunges to -0.4, -0.8, -1.0 (three feed-downs) with a dwell before the final retract.
    let plunges = prog.lines().iter().filter(|l| l.starts_with("G1 Z-")).count();
    assert_eq!(plunges, 3, "peck should produce three plunge increments:\n{:?}", prog.lines());
    assert!(prog.lines().iter().any(|l| l == "G4 P0.200"), "final dwell missing:\n{:?}", prog.lines());
  }
}
