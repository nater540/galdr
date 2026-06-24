//! Pure geometry helpers backing the toolpath preview's live-motion overlay.
//!
//! The preview's position marker and "cut so far" colouring used to be driven by GCode line acknowledgements
//! (`Progress::acked`). That makes the marker jump line-by-line and actually *lead* the real cutter, because
//! grbl emits `ok` when a line enters the planner buffer, not when the move finishes. This module derives the
//! same overlay from the live machine position carried in the `<...>` status report instead, so the marker
//! tracks where the tool actually is.
//!
//! Everything here is pure and framework-agnostic — plain `(f32, f32)` model-space points, no egui and no
//! `ViewState`/UI types — so the marker derivation, the per-frame smoothing step, and the monotonic
//! cut-progress computation are unit-tested without a window or a real status feed. The view layer adapts
//! these tuples to `egui::Vec2`/`Pos2` and projects them through its fit transform.

/// A point in toolpath model space (work-coordinate XY, millimetres). The toolpath segments live in this same
/// space, so the live marker and the segment geometry share one coordinate frame.
pub type ModelPoint = (f32, f32);

/// Squared Euclidean distance between two model points. Squared to avoid the `sqrt` in the hot per-frame
/// smoothing and progress paths; callers compare against squared thresholds.
fn dist_sq(a: ModelPoint, b: ModelPoint) -> f32 {
  let dx = a.0 - b.0;
  let dy = a.1 - b.1;
  dx * dx + dy * dy
}

/// Advance a smoothed marker one frame toward the latest live sample. The status feed arrives at 5–10 Hz while
/// the UI repaints up to 20 Hz, so lerping the held marker toward each fresh sample hides the sample-rate step
/// without extrapolating (we never dead-reckon past the latest sample, so the marker cannot overshoot the real
/// tool). `t` is the per-frame lerp fraction in `0..=1`. On first acquisition (`current` is `None`) or a jump
/// larger than `snap_dist` — a new program, a `$X`/teleport, a coordinate-system change — we snap to the target
/// instead of crawling across the canvas. Returns the new marker position.
pub fn smooth_marker(current: Option<ModelPoint>, target: ModelPoint, snap_dist: f32, t: f32) -> ModelPoint {
  let Some(current) = current else {
    return target; // First sample: nothing to lerp from, adopt the live position immediately.
  };
  if dist_sq(current, target) >= snap_dist * snap_dist {
    return target; // A large jump is a teleport, not motion to animate — snap so we never crawl across the bed.
  }
  let t = t.clamp(0.0, 1.0);
  (current.0 + (target.0 - current.0) * t, current.1 + (target.1 - current.1) * t)
}

/// A toolpath segment in model space, as its `(start, end)` end points. The progress projection needs both ends
/// (it projects the live point onto the segment's *line*, not just its endpoints), so the cached list is segment
/// pairs rather than the bare endpoints the old endpoint-proximity scan used.
pub type Segment = (ModelPoint, ModelPoint);

/// How far along a segment the projected live point must reach before that segment is counted "cut". At `0.5`
/// the segment flips to cut once the tool passes its midpoint, which keeps the coloured boundary tracking the
/// real cutter without claiming a segment the tool has barely entered or lagging a whole segment behind.
const COMPLETE_FRACTION: f32 = 0.5;

/// Project a point onto a segment and return `(fraction, distance_squared)`: `fraction` is the clamped position
/// of the foot of the perpendicular along `start→end` in `0..=1` (0 at `start`, 1 at `end`), and `distance_sq`
/// is the squared distance from `p` to that foot. A degenerate zero-length segment projects to its start.
fn project_onto(start: ModelPoint, end: ModelPoint, p: ModelPoint) -> (f32, f32) {
  let dx = end.0 - start.0;
  let dy = end.1 - start.1;
  let len_sq = dx * dx + dy * dy;
  if len_sq <= f32::EPSILON {
    return (0.0, dist_sq(start, p));
  }
  let t = (((p.0 - start.0) * dx + (p.1 - start.1) * dy) / len_sq).clamp(0.0, 1.0);
  let foot = (start.0 + dx * t, start.1 + dy * t);
  (t, dist_sq(foot, p))
}

/// Compute the cut-progress boundary by combining the two signals the sender actually has: the line counter (what
/// has been STREAMED to the controller) and the live tool position (where the cutter actually is). Returns the
/// count of leading segments to colour "cut".
///
/// Geometry alone cannot localise a sampled point on a self-intersecting path — a spiral pocket's parallel rings
/// and a star tip's out/back edges sit right on top of each other, so "where on the path is the tool?" is
/// ambiguous, and a pure-geometry rule either leaps onto a spatially-near future ring or stalls on a reversal.
/// The line counter removes that ambiguity. `acked` lines have been accepted by the controller, so nothing past
/// segment frontier `hi = {segments with line_index < acked}` can be cut. The controller runs BEHIND `acked` by
/// its buffer depth (grbl acks a line when it enters the planner buffer, not when the move finishes), so the
/// executing line is roughly `acked - in_flight_lines`; the caller derives `in_flight_lines` from the live `Bf:`
/// buffer report. That bounds the cutter to a small, CONTIGUOUS line-window `[lo, hi)` which does not fold back on
/// itself, so within it the nearest segment to the live position is the move executing now — unambiguously.
///
/// The search starts at `previous.clamp(lo, hi)`: capped at the frontier so progress can never run ahead of what
/// is streamed (no over-advance), and floored at `lo` so that as `acked` advances `lo` drags the boundary forward
/// even if the position match stumbles (no stall). The matched segment is counted cut once its foot passes
/// [`COMPLETE_FRACTION`]. The boundary never retreats below `previous` (monotonic within a run; reset `previous`
/// to 0 for a fresh run). `line_index` is the per-segment program line, parallel to `segments` and monotonic
/// non-decreasing, so the frontier lookups are binary partitions. Pure, so the whole rule is unit-tested.
pub fn live_cut_boundary(
  segments: &[Segment],
  line_index: &[u32],
  work: ModelPoint,
  acked: u32,
  in_flight_lines: u32,
  previous: usize,
) -> usize {
  debug_assert_eq!(segments.len(), line_index.len(), "segments and line_index must be parallel");
  // Frontier: segments whose program line has been acknowledged. Nothing beyond this is even in the controller
  // yet, so it is a hard upper bound on what can be cut. line_index is monotonic, so this is a binary partition.
  let hi = line_index.partition_point(|&l| l < acked);
  if hi == 0 {
    return 0; // Nothing acknowledged yet — nothing cut.
  }
  // Clamp a stale `previous` (e.g. the program shrank) to the frontier so the monotonic floor below cannot exceed
  // the path length and the `clamp` calls stay well-ordered.
  let previous = previous.min(hi);
  // The executing line trails `acked` by the live buffer depth; map that line back to the first in-flight segment.
  let lo_line = acked.saturating_sub(in_flight_lines.max(1));
  let lo = line_index.partition_point(|&l| l < lo_line);
  // Search the in-flight window: floored at `lo` (advances with `acked`, so it cannot stall) and capped at `hi`
  // (cannot run past what is streamed). Monotonic: never below `previous`.
  let start = previous.clamp(lo, hi);
  if start >= hi {
    return start;
  }
  // Within this small contiguous window the path does not self-intersect, so the nearest segment to the live
  // position is the move executing now. Ties resolve to the earliest index so the boundary advances steadily.
  let mut best = start;
  let mut best_fraction = 0.0_f32;
  let mut best_dist = f32::INFINITY;
  for (offset, &(seg_start, seg_end)) in segments[start..hi].iter().enumerate() {
    let (fraction, dist) = project_onto(seg_start, seg_end, work);
    if dist < best_dist {
      best_dist = dist;
      best_fraction = fraction;
      best = start + offset;
    }
  }
  // The executing segment counts as cut once the foot passes its midpoint, else it is the move in progress.
  let reached = if best_fraction >= COMPLETE_FRACTION { best + 1 } else { best };
  reached.clamp(previous, hi)
}

/// Decide whether the live tool marker should be drawn this frame, given the live work point, the toolpath's
/// model-space `(min, max)` bounds, the margin to allow outside them, and whether the machine is in an active
/// motion state (Run/Jog/Hold). The marker is shown when the point lies within the bounds expanded by `margin`,
/// OR unconditionally while the machine is actively moving (so a cut that legitimately runs just outside the
/// drawn extents still shows the tool). It is SUPPRESSED for an idle/parked machine whose reported point is far
/// off the path — the after-homing-at-machine-origin case (finding #4), a units mismatch that displaces the
/// point 25.4× (finding #3), or an active-WCS mismatch that floats it off the authored origin (finding #5). In
/// all three the safe behaviour is to draw nothing rather than a confident-but-wrong dot clipped to the rect
/// edge. Pure so the gate is unit-tested without a window.
pub fn marker_is_on_path(point: ModelPoint, min: ModelPoint, max: ModelPoint, margin: f32, moving: bool) -> bool {
  if moving {
    return true;
  }
  point.0 >= min.0 - margin
    && point.0 <= max.0 + margin
    && point.1 >= min.1 - margin
    && point.1 <= max.1 + margin
}

/// The maximum angular step (radians) of a flattened arc chord. An arc swept by more than this per chord is
/// subdivided further, so even a large-radius arc renders as a smooth polyline and the live-progress projection
/// (above) walks it chord-by-chord. ~9° (20 chords for a full circle) is visually smooth at preview scale while
/// keeping the segment count modest.
const MAX_ARC_STEP_RAD: f32 = std::f32::consts::PI / 20.0;

/// Flatten one G2/G3 arc (XY plane, G17) into a list of straight chord END points, in path order, EXCLUDING the
/// start point and INCLUDING the exact `end`. `start`/`end` are the arc's endpoints and `center` its centre (the
/// I/J offset applied to the start); `clockwise` is true for G2, false for G3. The caller appends one segment per
/// returned point (`from` = the previous point), so an arc becomes many short chords — which is what makes the
/// preview draw the real curve and the live-progress projection colour it smoothly as the tool sweeps it, rather
/// than a single start→end chord the swept point never lies on (finding #2).
///
/// The sweep angle is taken the short way consistent with the direction: we walk from the start angle toward the
/// end angle in the sense `clockwise` dictates, normalising to a positive sweep in `0..=2π` (a start == end is a
/// full revolution). The chord count is chosen so no chord subtends more than [`MAX_ARC_STEP_RAD`]. Pure geometry,
/// unit-tested without a window. A degenerate (near-zero-radius) arc yields just the end point.
pub fn flatten_arc(start: ModelPoint, end: ModelPoint, center: ModelPoint, clockwise: bool) -> Vec<ModelPoint> {
  let radius = (dist_sq(center, start)).sqrt();
  if radius <= f32::EPSILON {
    return vec![end];
  }
  let start_angle = (start.1 - center.1).atan2(start.0 - center.0);
  let end_angle = (end.1 - center.1).atan2(end.0 - center.0);
  let two_pi = std::f32::consts::TAU;
  // Signed sweep, positive in the direction of travel. CCW (G3) increases the angle; CW (G2) decreases it. We
  // normalise the magnitude into (0, 2π]: a start == end angle is a full circle, not a zero-length arc.
  let mut sweep = if clockwise { start_angle - end_angle } else { end_angle - start_angle };
  while sweep <= 0.0 {
    sweep += two_pi;
  }
  let steps = (sweep / MAX_ARC_STEP_RAD).ceil().max(1.0) as usize;
  let dir = if clockwise { -1.0 } else { 1.0 };
  let mut points = Vec::with_capacity(steps);
  for i in 1..steps {
    let theta = start_angle + dir * sweep * (i as f32) / (steps as f32);
    points.push((center.0 + radius * theta.cos(), center.1 + radius * theta.sin()));
  }
  // End exactly on the commanded endpoint rather than a recomputed point, so floating-point drift never leaves a
  // tiny gap between the arc and the next move.
  points.push(end);
  points
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn smooth_marker_snaps_on_first_acquisition() {
    // The very first sample has nothing to lerp from, so the marker adopts it exactly rather than easing in
    // from an arbitrary origin (which would draw a phantom sweep across the bed on connect).
    assert_eq!(smooth_marker(None, (10.0, 20.0), 50.0, 0.3), (10.0, 20.0));
  }

  #[test]
  fn smooth_marker_lerps_toward_a_nearby_sample() {
    // A small move eases part-way toward the target (here 50% of the way), smoothing the sample-rate step.
    let next = smooth_marker(Some((0.0, 0.0)), (10.0, 0.0), 50.0, 0.5);
    assert!((next.0 - 5.0).abs() < 1e-4, "x should ease half-way: {next:?}");
    assert!((next.1 - 0.0).abs() < 1e-4, "y unchanged: {next:?}");
  }

  #[test]
  fn smooth_marker_snaps_on_a_large_jump() {
    // A jump beyond the snap threshold (a new program / teleport) snaps rather than crawling across the canvas.
    let next = smooth_marker(Some((0.0, 0.0)), (100.0, 100.0), 50.0, 0.3);
    assert_eq!(next, (100.0, 100.0), "a jump past snap_dist must snap, not lerp");
  }

  #[test]
  fn smooth_marker_clamps_t_outside_unit_range() {
    // A degenerate `t` outside `0..=1` is clamped so it can neither overshoot nor reverse.
    assert_eq!(smooth_marker(Some((0.0, 0.0)), (10.0, 0.0), 50.0, 5.0), (10.0, 0.0));
    assert_eq!(smooth_marker(Some((4.0, 0.0)), (10.0, 0.0), 50.0, -5.0), (4.0, 0.0));
  }

  /// A straight path of `n` unit `(start, end)` segments along X — segment `i` spans `(i,0)→(i+1,0)` and comes
  /// from program line `i`, so the line index is parallel and monotonic (one segment per line).
  fn line_path(n: u32) -> (Vec<Segment>, Vec<u32>) {
    let segs = (0..n).map(|i| ((i as f32, 0.0), ((i + 1) as f32, 0.0))).collect();
    let lines = (0..n).collect();
    (segs, lines)
  }

  #[test]
  fn live_cut_boundary_is_zero_before_anything_is_acknowledged() {
    let (s, l) = line_path(5);
    // Nothing streamed yet (`acked == 0`): the frontier is empty, so nothing is cut regardless of position.
    assert_eq!(live_cut_boundary(&s, &l, (3.0, 0.0), 0, 8, 0), 0);
  }

  #[test]
  fn live_cut_boundary_never_colours_past_the_streamed_frontier() {
    let (s, l) = line_path(10);
    // Only lines 0..3 are acknowledged. Even though the live position sits way down at segment 8, the boundary
    // cannot exceed the streamed frontier (3) — the controller has not even received the later moves. This is the
    // over-advance cap: a spatially-near future segment that has not been streamed can never be coloured.
    let b = live_cut_boundary(&s, &l, (8.5, 0.0), 3, 8, 0);
    assert!(b <= 3, "must not colour past the acked frontier, got {b}");
  }

  #[test]
  fn live_cut_boundary_localises_the_cutter_within_the_in_flight_window() {
    let (s, l) = line_path(10);
    // Six lines acked, the controller running two lines behind (executing ~line 4). The live tool is on segment 4,
    // so the boundary lands just past it (5) — the position refines within the line-bounded in-flight window.
    assert_eq!(live_cut_boundary(&s, &l, (4.5, 0.0), 6, 2, 0), 5);
  }

  #[test]
  fn live_cut_boundary_floor_advances_with_acked_so_it_cannot_stall() {
    let (s, l) = line_path(10);
    // The live position is useless here (far off the path), so geometry alone could never localise — the failure
    // mode that left the preview all white. The streamed-line floor (`acked - in_flight_lines`) still drags the
    // boundary forward, so progress tracks the stream instead of freezing at zero.
    let b = live_cut_boundary(&s, &l, (1000.0, 1000.0), 8, 2, 0);
    assert!(b >= 6, "the streamed-line floor must advance progress even with a useless position match, got {b}");
  }

  #[test]
  fn live_cut_boundary_is_monotonic_and_never_backtracks() {
    let (s, l) = line_path(6);
    // Having progressed to 4, a jittery sample that projects back near segment 1 must not un-cut later segments.
    assert_eq!(live_cut_boundary(&s, &l, (1.0, 0.0), 6, 3, 4), 4);
  }

  #[test]
  fn live_cut_boundary_tracks_a_reversing_path_marched_frame_by_frame() {
    // THE all-white regression, now via the line window. A direction-reversing path (spiral/raster) defeated every
    // pure-geometry rule (leap, then stall). Here the line counter anchors which stretch is in flight and the
    // position localises within it. March the cutter along the whole path, advancing `acked` each frame, and
    // assert the boundary follows to the end. One program line per segment, buffer one line deep.
    let path = vec![
      ((0.0, 0.0), (10.0, 0.0)),  // line 0 → right
      ((10.0, 0.0), (10.0, 1.0)), // line 1 ↑
      ((10.0, 1.0), (0.0, 1.0)),  // line 2 ← left (a reversal)
      ((0.0, 1.0), (0.0, 2.0)),   // line 3 ↑
      ((0.0, 2.0), (10.0, 2.0)),  // line 4 → right
    ];
    let lines = vec![0, 1, 2, 3, 4];
    let frames = [((5.0, 0.0), 1u32), ((10.0, 0.5), 2), ((5.0, 1.0), 3), ((0.0, 1.5), 4), ((5.0, 2.0), 5)];
    let mut prev = 0;
    for (pt, acked) in frames {
      prev = live_cut_boundary(&path, &lines, pt, acked, 1, prev);
    }
    assert_eq!(prev, 5, "the boundary must track across reversals to the path end, not stall near 0");
  }

  #[test]
  fn live_cut_boundary_excludes_an_unstreamed_overlapping_ring() {
    // The self-approach ambiguity, resolved by the line counter. Segment 4 is a "next ring" running right on top of
    // segment 0, but it comes from a later, NOT-yet-streamed line. The cutter on segment 0 must colour segment 0,
    // never the spatially-coincident-but-unstreamed segment 4.
    let segs = vec![
      ((0.0, 0.0), (10.0, 0.0)),    // line 0: the move the cutter is on
      ((10.0, 0.0), (10.0, 1.0)),   // line 1
      ((10.0, 1.0), (0.0, 1.0)),    // line 2
      ((0.0, 1.0), (0.0, 2.0)),     // line 3
      ((0.0, 0.05), (10.0, 0.05)),  // line 4: a parallel ring ~on top of line 0, not yet streamed
    ];
    let lines = vec![0, 1, 2, 3, 4];
    // Cutter at the midpoint of segment 0; only lines 0..2 acked, so segment 4 is beyond the frontier and excluded.
    let b = live_cut_boundary(&segs, &lines, (5.0, 0.0), 2, 4, 0);
    assert!((1..=2).contains(&b), "the unstreamed overlapping ring must not be coloured, got {b}");
  }

  #[test]
  fn marker_is_on_path_suppresses_a_parked_point_far_off_the_path() {
    let (min, max) = ((0.0, 0.0), (10.0, 10.0));
    // An idle machine sitting on the path (or just inside the margin) shows the marker.
    assert!(marker_is_on_path((5.0, 5.0), min, max, 2.0, false));
    assert!(marker_is_on_path((-1.0, 11.0), min, max, 2.0, false), "within the margin still shows");
    // An idle machine parked far off the path — homed to machine origin (#4), a 25.4× units displacement (#3), or
    // a WCS-mismatch float (#5) — suppresses the marker rather than drawing a misleading dot.
    assert!(!marker_is_on_path((-50.0, -50.0), min, max, 2.0, false), "far off + idle must suppress");
    assert!(!marker_is_on_path((254.0, 254.0), min, max, 2.0, false), "a 25.4x displacement must suppress");
  }

  #[test]
  fn marker_is_on_path_always_shows_while_moving() {
    let (min, max) = ((0.0, 0.0), (10.0, 10.0));
    // While the machine is actively moving (Run/Jog/Hold) the marker shows even outside the bounds — a cut that
    // legitimately runs just past the drawn extents must still track the tool.
    assert!(marker_is_on_path((-100.0, -100.0), min, max, 2.0, true), "a moving machine always shows the marker");
  }

  #[test]
  fn flatten_arc_subdivides_a_quarter_circle_into_many_chords_on_the_radius() {
    // A G3 (CCW) quarter circle from (1,0) to (0,1) about the origin. The result must be many short chords (not a
    // single start→end chord), every intermediate point must lie on the unit radius, and it must end exactly at
    // the commanded endpoint.
    let pts = flatten_arc((1.0, 0.0), (0.0, 1.0), (0.0, 0.0), false);
    assert!(pts.len() >= 3, "a quarter circle must flatten into several chords, got {}", pts.len());
    for p in &pts {
      let r = (p.0 * p.0 + p.1 * p.1).sqrt();
      assert!((r - 1.0).abs() < 1e-3, "every chord point must sit on the radius: {p:?} r={r}");
    }
    assert_eq!(*pts.last().unwrap(), (0.0, 1.0), "the arc must end on the commanded endpoint");
    // The chords must sweep CCW: the first intermediate point is above-and-left of the start (y increases).
    assert!(pts[0].1 > 0.0, "a CCW sweep raises Y first: {:?}", pts[0]);
  }

  #[test]
  fn flatten_arc_directions_sweep_opposite_ways() {
    // The SAME endpoints with opposite directions must sweep opposite ways. From (1,0) to (-1,0) about the origin:
    // CCW (G3) goes over the top (+Y), CW (G2) goes under the bottom (−Y).
    let ccw = flatten_arc((1.0, 0.0), (-1.0, 0.0), (0.0, 0.0), false);
    let cw = flatten_arc((1.0, 0.0), (-1.0, 0.0), (0.0, 0.0), true);
    assert!(ccw[0].1 > 0.0, "G3 sweeps over the top: {:?}", ccw[0]);
    assert!(cw[0].1 < 0.0, "G2 sweeps under the bottom: {:?}", cw[0]);
  }

  #[test]
  fn flatten_arc_treats_coincident_endpoints_as_a_full_circle() {
    // A G2 arc whose start == end is a full revolution (a common bore/contour pattern), not a zero-length move; it
    // must produce a closed loop of chords, not collapse to a single point.
    let pts = flatten_arc((1.0, 0.0), (1.0, 0.0), (0.0, 0.0), true);
    assert!(pts.len() > 8, "a full circle must flatten into many chords, got {}", pts.len());
    assert_eq!(*pts.last().unwrap(), (1.0, 0.0), "a full circle returns to its start");
  }

  #[test]
  fn flatten_arc_degenerate_radius_yields_just_the_endpoint() {
    // A near-zero-radius arc (start == center) cannot define a sweep; it degrades to a single chord to the end.
    assert_eq!(flatten_arc((0.0, 0.0), (2.0, 3.0), (0.0, 0.0), false), vec![(2.0, 3.0)]);
  }

  #[test]
  fn project_onto_clamps_and_measures_perpendicular_distance() {
    // The foot of the perpendicular from a point above the middle of a unit X segment lands at the midpoint
    // (fraction 0.5) and its squared distance is the perpendicular height squared.
    let (f, d) = project_onto((0.0, 0.0), (1.0, 0.0), (0.5, 0.25));
    assert!((f - 0.5).abs() < 1e-5, "foot at the midpoint: {f}");
    assert!((d - 0.0625).abs() < 1e-5, "perpendicular distance squared: {d}");
    // A point beyond the far end clamps the fraction to 1.0 (the projection cannot run past the segment).
    let (f, _) = project_onto((0.0, 0.0), (1.0, 0.0), (5.0, 0.0));
    assert_eq!(f, 1.0);
    // A degenerate zero-length segment projects to its start.
    let (f, d) = project_onto((2.0, 2.0), (2.0, 2.0), (2.0, 5.0));
    assert_eq!(f, 0.0);
    assert!((d - 9.0).abs() < 1e-5, "distance to the degenerate point: {d}");
  }
}
