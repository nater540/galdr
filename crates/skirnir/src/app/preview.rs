//! Pure geometry helpers backing the toolpath preview's live-motion marker and depth shading.
//!
//! The live marker dot is driven by the machine position carried in the `<...>` status report — a smoothed dot
//! showing where the tool is right now. The PROGRESS BACKPLOT (the bright "what has been cut" path) is NOT drawn
//! from status here: it is a deterministic executed-prefix of the parsed toolpath, gated by the streaming engine's
//! acked-line count in the view layer. An earlier design accumulated a status-SAMPLED trail polyline instead, but
//! at the ~10 Hz poll rate a move finishing between two samples left no segment — phantom gaps that differed run-to-
//! run (the §16 render bug). Splitting the already-complete parsed geometry by acked-line index removes that under-
//! sampling entirely; the marker still shows true position, so the two together give a gap-free path plus a live
//! cursor. (`ok` leads the real tool by the planner-buffer depth, which is fine for a progress backplot.)
//!
//! Everything here is pure and framework-agnostic — plain `(f32, f32)` model-space points, no egui and no
//! `ViewState`/UI types — so the marker derivation, the per-frame smoothing step, and the depth-shading mapping are
//! unit-tested without a window or a real status feed. The view layer adapts these tuples to `egui::Vec2`/`Pos2`
//! and projects them through its fit transform.

/// A point in toolpath model space (work-coordinate XY, millimetres). The toolpath segments live in this same
/// space, so the live marker and the segment geometry share one coordinate frame.
pub type ModelPoint = (f32, f32);

/// Squared Euclidean distance between two model points. Squared to avoid the `sqrt` in the hot per-frame marker
/// smoothing path; callers compare against squared thresholds.
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

/// The dimmest a depth-shaded cut line is drawn, as a fraction of the base cut colour's brightness — the value the
/// deepest pass (the program's most negative Z) is darkened to. The shallowest cut (Z just below zero) draws at the
/// full base colour (`1.0`); everything between interpolates linearly toward this floor. Kept here as one tunable so
/// the depth-cue contrast is adjusted in a single place; ~0.35 keeps the deepest pass clearly visible (not black)
/// while still reading as "deeper" against the bright shallow passes.
pub const DEPTH_BRIGHTNESS_FLOOR: f32 = 0.35;

/// Map a cut DEPTH (a negative work-Z) to a brightness fraction in `[DEPTH_BRIGHTNESS_FLOOR, 1.0]`, job-relative:
/// the shallowest cut (`z` just below 0) is brightest (`1.0`, the full base colour) and the program's DEEPEST pass
/// (`z == job_min_z`, the most negative programmed Z) is darkest (the floor). The caller multiplies the base cut
/// colour by this fraction, so a single base colour shades by depth without a second theme token.
///
/// `job_min_z` is the program's minimum (most negative) Z, scanned once at load. The normalised depth is
/// `t = z / job_min_z` clamped to `[0, 1]` (both `z` and `job_min_z` are negative, so the ratio is positive), and
/// brightness eases linearly from `1.0` at `t == 0` to the floor at `t == 1`. When `job_min_z` is unknown or
/// non-negative (`>= 0`: no negative Z scanned, a flat or XY-only program) there is no depth scale to normalise
/// against, so every cut takes the full base colour (`1.0`) rather than dividing by zero. Pure so the mapping is
/// unit-tested.
pub fn depth_brightness(z: f32, job_min_z: f32) -> f32 {
  if job_min_z >= 0.0 {
    return 1.0; // No usable depth range (flat/positive-only program): no darkening, full base colour.
  }
  let t = (z / job_min_z).clamp(0.0, 1.0);
  1.0 - t * (1.0 - DEPTH_BRIGHTNESS_FLOOR)
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

/// The default maximum angular step (radians) of a flattened arc chord, used by [`flatten_arc`] when the caller
/// passes a non-positive step. ~9° (20 chords for a full circle) is visually smooth at preview scale while keeping
/// the segment count modest; this is the value the config's `toolpath.arc_step_deg` mirrors as its default.
pub const DEFAULT_ARC_STEP_RAD: f32 = std::f32::consts::PI / 20.0;

/// Flatten one G2/G3 arc (XY plane, G17) into a list of straight chord END points, in path order, EXCLUDING the
/// start point and INCLUDING the exact `end`. `start`/`end` are the arc's endpoints and `center` its centre (the
/// I/J offset applied to the start); `clockwise` is true for G2, false for G3. The caller appends one segment per
/// returned point (`from` = the previous point), so an arc becomes many short chords — which is what makes the
/// preview draw the real curve and the live-progress projection colour it smoothly as the tool sweeps it, rather
/// than a single start→end chord the swept point never lies on (finding #2).
///
/// `max_step_rad` is the maximum angle any one chord may subtend — the config-resolved
/// [`crate::config::ToolpathStyle::arc_step_rad`]; a non-positive value falls back to [`DEFAULT_ARC_STEP_RAD`] so a
/// degenerate config cannot divide by zero or produce an infinite chord count. The sweep angle is taken the short
/// way consistent with the direction: we walk from the start angle toward the end angle in the sense `clockwise`
/// dictates, normalising to a positive sweep in `0..=2π` (a start == end is a full revolution). Pure geometry,
/// unit-tested without a window. A degenerate (near-zero-radius) arc yields just the end point.
pub fn flatten_arc(
  start: ModelPoint, end: ModelPoint, center: ModelPoint, clockwise: bool, max_step_rad: f32,
) -> Vec<ModelPoint> {
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
  // A non-positive config step is degenerate; fall back to the default so the chord count stays finite.
  let step = if max_step_rad > 0.0 { max_step_rad } else { DEFAULT_ARC_STEP_RAD };
  let steps = (sweep / step).ceil().max(1.0) as usize;
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


  #[test]
  fn depth_brightness_is_full_at_the_surface_and_floors_at_the_deepest_pass() {
    // A job whose deepest pass is −2 mm. The shallowest cut (z → 0) is full brightness; the deepest (z == job_min_z)
    // floors; the midpoint sits exactly halfway between the floor and full.
    let job = -2.0;
    assert!((depth_brightness(0.0, job) - 1.0).abs() < 1e-6, "the surface is full brightness");
    assert!((depth_brightness(job, job) - DEPTH_BRIGHTNESS_FLOOR).abs() < 1e-6, "the deepest pass floors");
    let mid = depth_brightness(-1.0, job);
    let expected_mid = 1.0 - 0.5 * (1.0 - DEPTH_BRIGHTNESS_FLOOR);
    assert!((mid - expected_mid).abs() < 1e-6, "the midpoint is halfway to the floor: {mid}");
  }

  #[test]
  fn depth_brightness_is_monotonic_and_clamps_beyond_the_deepest_pass() {
    // Brightness must fall monotonically as the cut deepens, so a deeper line is never brighter than a shallower one.
    let job = -4.0;
    let mut last = depth_brightness(0.0, job);
    for step in 1..=8 {
      let z = -(step as f32) * 0.5; // 0 → −4 in 0.5 mm steps.
      let b = depth_brightness(z, job);
      assert!(b <= last + 1e-6, "brightness must not rise as depth increases: z={z} b={b} last={last}");
      last = b;
    }
    // A Z beyond the scanned deepest pass (e.g. an override-driven overshoot) clamps at the floor, not below it.
    assert!((depth_brightness(-10.0, job) - DEPTH_BRIGHTNESS_FLOOR).abs() < 1e-6, "past the deepest pass clamps");
  }

  #[test]
  fn depth_brightness_falls_back_to_full_when_no_depth_range_is_known() {
    // No usable job depth (a flat program, an XY-only program, or none scanned yet): every cut takes the full base
    // colour rather than dividing by a zero/non-negative range.
    assert_eq!(depth_brightness(-1.0, 0.0), 1.0, "a zero job_min_z has no range to normalise against");
    assert_eq!(depth_brightness(-1.0, 5.0), 1.0, "a non-negative job_min_z is treated as no depth range");
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
    let pts = flatten_arc((1.0, 0.0), (0.0, 1.0), (0.0, 0.0), false, DEFAULT_ARC_STEP_RAD);
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
    let ccw = flatten_arc((1.0, 0.0), (-1.0, 0.0), (0.0, 0.0), false, DEFAULT_ARC_STEP_RAD);
    let cw = flatten_arc((1.0, 0.0), (-1.0, 0.0), (0.0, 0.0), true, DEFAULT_ARC_STEP_RAD);
    assert!(ccw[0].1 > 0.0, "G3 sweeps over the top: {:?}", ccw[0]);
    assert!(cw[0].1 < 0.0, "G2 sweeps under the bottom: {:?}", cw[0]);
  }

  #[test]
  fn flatten_arc_treats_coincident_endpoints_as_a_full_circle() {
    // A G2 arc whose start == end is a full revolution (a common bore/contour pattern), not a zero-length move; it
    // must produce a closed loop of chords, not collapse to a single point.
    let pts = flatten_arc((1.0, 0.0), (1.0, 0.0), (0.0, 0.0), true, DEFAULT_ARC_STEP_RAD);
    assert!(pts.len() > 8, "a full circle must flatten into many chords, got {}", pts.len());
    assert_eq!(*pts.last().unwrap(), (1.0, 0.0), "a full circle returns to its start");
  }

  #[test]
  fn flatten_arc_degenerate_radius_yields_just_the_endpoint() {
    // A near-zero-radius arc (start == center) cannot define a sweep; it degrades to a single chord to the end.
    assert_eq!(flatten_arc((0.0, 0.0), (2.0, 3.0), (0.0, 0.0), false, DEFAULT_ARC_STEP_RAD), vec![(2.0, 3.0)]);
  }
}
