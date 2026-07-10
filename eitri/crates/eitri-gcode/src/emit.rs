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
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
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
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
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

/// The work-zero (datum) the emitter posts relative to: it subtracts this point — in the toolpaths' own coordinate
/// frame — from every emitted coordinate, so the exported program is referenced to the operator's chosen origin (the
/// Vectric-style datum + Z-zero). Absolute XY shifts by `(x, y)`; every Z shifts by `z` (which is `0` when work-Z0 is
/// the stock top, or `-thickness` when it is the stock bottom). Arc I/J offsets are relative, so they are never
/// shifted. [`Origin::NATIVE`] is the default: no shift, i.e. keep the source (EDA plot) frame with Z0 at the surface.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Origin {
  /// The X coordinate, in the native frame, that maps to work X0.
  pub x: f64,
  /// The Y coordinate, in the native frame, that maps to work Y0.
  pub y: f64,
  /// The Z coordinate, in the native (surface = 0) frame, that maps to work Z0. `0.0` keeps Z0 at the surface;
  /// `-thickness` moves it to the stock bottom.
  pub z: f64,
}

impl Origin {
  /// The identity datum: coordinates are emitted unchanged, in their native (source) frame.
  pub const NATIVE: Origin = Origin { x: 0.0, y: 0.0, z: 0.0 };

  /// A datum at native-frame point `(x, y)` with Z0 at the surface.
  pub fn new(x: f64, y: f64) -> Origin {
    Origin { x, y, z: 0.0 }
  }

  /// A datum at native-frame point `(x, y, z)`.
  pub fn with_z(x: f64, y: f64, z: f64) -> Origin {
    Origin { x, y, z }
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
  origin: Origin,
  post: &dyn Postprocessor,
  tool: Option<String>,
) -> Program {
  let mut prog = Program::new(post.format());
  let ctx = JobContext { name: job.name.clone(), tool };
  post.start_code(&mut prog, &ctx);

  if !toolpaths.rings.is_empty() {
    post.spindle_on(&mut prog, Spindle { rpm: job.spindle_rpm });
    // Establish the safe-height invariant once; every ring restores it on exit.
    post.rapid(&mut prog, Axes::z(job.travel_z - origin.z));

    let depths = depth_steps(job.cut_depth, job.pass_depth);
    for ring in &toolpaths.rings {
      let (sx, sy) = ring.geometry.start();
      let (sx, sy) = (sx - origin.x, sy - origin.y);
      let segments = ring.geometry.segments();
      let closed = ring.geometry.is_closed();
      post.rapid(&mut prog, Axes::xy(sx, sy));
      for (pass, &z) in depths.iter().enumerate() {
        // A closed ring ends each pass back at its start, so the next deeper pass just plunges in place. An open
        // path (a raster fill row, a cutout arc between tabs) ends elsewhere, so lift and rapid back to the start
        // before plunging again — never drag the tool through material back to the start. The first pass always
        // plunges in place: the rapid to the start already put the tool there.
        if pass > 0 && !closed {
          post.rapid(&mut prog, Axes::z(job.travel_z - origin.z));
          post.rapid(&mut prog, Axes::xy(sx, sy));
        }
        post.linear(&mut prog, Axes::z(z - origin.z), job.plunge_feed);
        emit_ring_segments(&mut prog, post, &segments, job.cut_feed, origin);
      }
      post.rapid(&mut prog, Axes::z(job.travel_z - origin.z));
    }
  }

  // The spindle is stopped by `end_code`, keeping the postamble (spindle off + program end) in one place.
  post.end_code(&mut prog, &ctx);
  prog
}

/// Cut each segment of a ring at `feed`, dispatching linear vs arc to the postprocessor. Absolute XY is shifted to the
/// datum; the arc I/J `offset` is relative, so it is emitted unchanged.
fn emit_ring_segments(prog: &mut Program, post: &dyn Postprocessor, segments: &[Segment], feed: f64, origin: Origin) {
  for seg in segments {
    match *seg {
      Segment::Line { x, y } => post.linear(prog, Axes::xy(x - origin.x, y - origin.y), feed),
      Segment::Arc { x, y, offset, dir } => {
        post.arc(prog, ArcMove { x: x - origin.x, y: y - origin.y, offset, dir }, feed)
      }
    }
  }
}

/// Emit a complete drilling program from `plan` using `post`. Each tool group gets a tool change and spindle start,
/// then every hit is drilled (with a manual peck cycle when configured — grbl has no canned `G81`/`G83`) or every
/// slot is plunged and routed.
pub fn emit_drilling(
  plan: &DrillPlan,
  job: &DrillJob,
  origin: Origin,
  post: &dyn Postprocessor,
  tool: Option<String>,
) -> Program {
  let mut prog = Program::new(post.format());
  let ctx = JobContext { name: job.name.clone(), tool };
  post.start_code(&mut prog, &ctx);

  if !plan.tools.is_empty() {
    post.rapid(&mut prog, Axes::z(job.travel_z - origin.z));
    for tool in &plan.tools {
      post.tool_change(&mut prog, ToolChange { number: tool.tool, diameter: tool.diameter });
      post.spindle_on(&mut prog, Spindle { rpm: job.spindle_rpm });
      // Lift to the cross-tool safe height after the change before repositioning.
      post.rapid(&mut prog, Axes::z(job.travel_z - origin.z));

      for mv in &tool.moves {
        match *mv {
          DrillMove::Drill { at } => {
            post.rapid(&mut prog, Axes::xy(at.x - origin.x, at.y - origin.y));
            drill_hole(&mut prog, post, &tool.params, origin.z);
          }
          DrillMove::Slot { from, to } => {
            post.rapid(&mut prog, Axes::xy(from.x - origin.x, from.y - origin.y));
            route_slot(&mut prog, post, to.x - origin.x, to.y - origin.y, &tool.params, origin.z);
          }
        }
      }
      // Restore the safe height for the next tool change / program end.
      post.rapid(&mut prog, Axes::z(job.travel_z - origin.z));
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
fn drill_hole(prog: &mut Program, post: &dyn Postprocessor, params: &DrillParams, z_off: f64) {
  match params.peck {
    Some(peck) if peck > 0.0 && peck < params.depth => {
      let mut prev = 0.0;
      loop {
        let target = (prev + peck).min(params.depth);
        if prev > 0.0 {
          // Rapid back down to just above the last cut depth, then feed the fresh increment. For a peck smaller than
          // the clearance, `-(prev - clearance)` would rise ABOVE the work surface (Z0), sending the tool up into the
          // air and then plunging back through material — clamp it to the surface so it never rises above Z0.
          let resume_z = (-(prev - PECK_RAPID_CLEARANCE)).min(0.0);
          post.rapid(prog, Axes::z(resume_z - z_off));
        }
        post.linear(prog, Axes::z(-target - z_off), params.feed);
        let last = target >= params.depth - 1.0e-9;
        if last {
          if let Some(dwell) = params.dwell {
            post.dwell(prog, dwell);
          }
        }
        post.rapid(prog, Axes::z(params.retract - z_off));
        prev = target;
        if last {
          break;
        }
      }
    }
    _ => {
      post.linear(prog, Axes::z(-params.depth - z_off), params.feed);
      if let Some(dwell) = params.dwell {
        post.dwell(prog, dwell);
      }
      post.rapid(prog, Axes::z(params.retract - z_off));
    }
  }
}

/// Route a slot: plunge to full depth at the current start, feed across to `(x, y)`, optionally dwell, then retract.
/// Slots plunge in a single pass (peck is a point-drill concern). `z_off` shifts every Z to the work-Z0 datum.
fn route_slot(prog: &mut Program, post: &dyn Postprocessor, x: f64, y: f64, params: &DrillParams, z_off: f64) {
  post.linear(prog, Axes::z(-params.depth - z_off), params.feed);
  post.linear(prog, Axes::xy(x, y), params.feed);
  if let Some(dwell) = params.dwell {
    post.dwell(prog, dwell);
  }
  post.rapid(prog, Axes::z(params.retract - z_off));
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
    let prog = emit_isolation(&paths, &job, Origin::NATIVE, &grbl(), None);
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
    let prog = emit_isolation(&paths, &IsolationJob::default(), Origin::NATIVE, &grbl(), None);
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
    let prog = emit_isolation(&paths, &job, Origin::NATIVE, &grbl(), None);
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
    let prog = emit_isolation(&paths, &job, Origin::NATIVE, &grbl(), None);
    let starts = prog.lines().iter().filter(|l| l.as_str() == "G0 X0.0000 Y0.0000").count();
    assert_eq!(starts, 1, "a closed ring rapids to its start only once");
  }

  #[test]
  fn multi_depth_produces_a_plunge_per_level() {
    let paths = linear_toolpaths(square_ring());
    let job = IsolationJob { cut_depth: 0.3, pass_depth: 0.1, ..Default::default() };
    let prog = emit_isolation(&paths, &job, Origin::NATIVE, &grbl(), None);
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
    let prog = emit_isolation(&paths, &IsolationJob::default(), Origin::NATIVE, &grbl(), None);
    let text = prog.render();
    assert!(text.contains("G3 X0.0000 Y1.0000 I-1.0000 J0.0000 F"), "arc move missing:\n{text}");
  }

  #[test]
  fn datum_shifts_absolute_xy_but_not_arc_ij_or_z() {
    // The same CCW quarter-circle arc, posted against a datum at (1, 0): every absolute XY drops by the datum, while
    // the relative I/J arc offset and the Z depth are untouched.
    let ring = ArcPolyline {
      vertices: vec![
        ArcVertex { x: 1.0, y: 0.0, bulge: (std::f64::consts::FRAC_PI_8).tan() },
        ArcVertex { x: 0.0, y: 1.0, bulge: 0.0 },
      ],
    };
    let paths = IsolationToolpaths {
      rings: vec![IsolationRing { pass: 0, offset: 0.5, winding: WindingDirection::Ccw, geometry: ring }],
    };
    let job = IsolationJob { cut_depth: 0.1, pass_depth: 0.1, ..Default::default() };
    let native = emit_isolation(&paths, &job, Origin::NATIVE, &grbl(), None).render();
    let shifted = emit_isolation(&paths, &job, Origin::new(1.0, 0.0), &grbl(), None).render();
    assert!(native.contains("G0 X1.0000 Y0.0000"), "native start:\n{native}");
    assert!(shifted.contains("G0 X0.0000 Y0.0000"), "datum drops the start to the origin:\n{shifted}");
    // The arc endpoint shifts in X, but the relative I/J offset is byte-for-byte identical, as is the Z plunge.
    assert!(native.contains("G3 X0.0000 Y1.0000 I-1.0000 J0.0000"), "native arc:\n{native}");
    assert!(shifted.contains("G3 X-1.0000 Y1.0000 I-1.0000 J0.0000"), "arc XY shifts, I/J unchanged:\n{shifted}");
    assert!(shifted.contains("G1 Z-0.1000"), "the Z depth is unaffected by an XY datum:\n{shifted}");
  }

  #[test]
  fn z_datum_shifts_every_z_by_the_offset() {
    // Work-Z0 at the stock bottom of a 1.6 mm board: origin.z = -1.6, so every emitted Z rises by 1.6 — the plunge
    // to −0.1 becomes +1.5, the 2 mm travel becomes 3.6. XY is unaffected (origin x/y = 0 here).
    let paths = linear_toolpaths(square_ring());
    let job = IsolationJob { cut_depth: 0.1, pass_depth: 0.1, travel_z: 2.0, ..Default::default() };
    let text = emit_isolation(&paths, &job, Origin::with_z(0.0, 0.0, -1.6), &grbl(), None).render();
    assert!(text.contains("G1 Z1.5000"), "plunge to −0.1 with Z0 at the bottom is +1.5:\n{text}");
    assert!(text.contains("G0 Z3.6000"), "the 2 mm travel height rises by the 1.6 mm thickness:\n{text}");
    // A top-zero (origin.z = 0) job is byte-for-byte the old output.
    let top = emit_isolation(&paths, &job, Origin::new(0.0, 0.0), &grbl(), None).render();
    assert!(top.contains("G1 Z-0.1000") && top.contains("G0 Z2.0000"), "top-zero is unchanged:\n{top}");
  }

  #[test]
  fn datum_shifts_drill_hits_but_not_depth() {
    let plan = DrillPlan {
      tools: vec![ToolDrillPlan {
        tool: 1,
        diameter: 0.8,
        params: DrillParams { depth: 1.6, feed: 100.0, retract: 2.0, peck: None, dwell: None },
        moves: vec![DrillMove::Drill { at: Point::new(1.0, 2.0) }],
      }],
    };
    let text = emit_drilling(&plan, &DrillJob::default(), Origin::new(1.0, 2.0), &grbl(), None).render();
    assert!(text.contains("G0 X0.0000 Y0.0000"), "the hole at (1,2) posts at the datum origin:\n{text}");
    assert!(text.contains("G1 Z-1.6000"), "drill depth is unaffected by the XY datum:\n{text}");
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
    let prog = emit_drilling(&plan, &DrillJob::default(), Origin::NATIVE, &grbl(), None);
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
    let prog = emit_drilling(&plan, &DrillJob::default(), Origin::NATIVE, &grbl(), None);
    // 1.0mm at 0.4mm pecks => plunges to -0.4, -0.8, -1.0 (three feed-downs) with a dwell before the final retract.
    let plunges = prog.lines().iter().filter(|l| l.starts_with("G1 Z-")).count();
    assert_eq!(plunges, 3, "peck should produce three plunge increments:\n{:?}", prog.lines());
    assert!(prog.lines().iter().any(|l| l == "G4 P0.200"), "final dwell missing:\n{:?}", prog.lines());
  }

  #[test]
  fn small_peck_never_rapids_above_the_work_surface() {
    // Finding #5: for a peck increment below PECK_RAPID_CLEARANCE, `-(prev - clearance)` becomes positive, so the
    // rapid-return rose above the stock (into the air) before plunging back through material. The resume rapid must
    // never carry a positive Z; the only legitimately-positive Z rapids are the full retract and the travel height.
    let retract = 1.5;
    let plan = DrillPlan {
      tools: vec![ToolDrillPlan {
        tool: 1,
        diameter: 0.5,
        params: DrillParams { depth: 0.2, feed: 60.0, retract, peck: Some(0.05), dwell: None },
        moves: vec![DrillMove::Drill { at: Point::new(0.0, 0.0) }],
      }],
    };
    let prog = emit_drilling(&plan, &DrillJob::default(), Origin::NATIVE, &grbl(), None);
    for line in prog.lines() {
      if let Some(rest) = line.strip_prefix("G0 Z") {
        let z: f64 = rest.trim().parse().expect("parse rapid Z");
        // Resume rapids sit at or below the surface (<= 0); the only positive rapids are >= the retract height.
        assert!(z <= 1e-9 || z >= retract - 1e-6, "peck rapid rose above the work surface: `{line}`");
      }
    }
  }
}
