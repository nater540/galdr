//! The toolpath viewport: program parsing, bounds, live overlay/trail, and grid drawing.

use super::*;

/// The per-frame lerp fraction the live marker eases toward each fresh status sample. At up to 20 Hz repaint
/// against a 5–10 Hz status feed this hides the sample-rate step within a couple of frames without lagging
/// perceptibly. Lerp-only (no extrapolation), so the marker can never overshoot the real tool.
const MARKER_LERP: f32 = 0.35;

/// The marker snaps (rather than eases) to a new sample once the jump exceeds this fraction of the toolpath's
/// model-space span diagonal — a new program, a `$X`/teleport, or a coordinate-system change is not motion to
/// animate. Below it, normal cutting moves between samples are smoothed.
const MARKER_SNAP_SPAN_FRACTION: f32 = 0.25;

/// How far outside the toolpath's model-space bounds the live marker may sit and still be drawn, as a fraction of
/// the span diagonal. Generous enough that a tool a little outside the drawn extents (lead-in, clearance move)
/// still shows while idle, tight enough that a grossly displaced point — a 25.4× units mismatch (#3), a homed-off
/// machine-origin park (#4), or a WCS-mismatch float (#5) — falls outside and is suppressed.
const MARKER_ON_PATH_MARGIN_FRACTION: f32 = 0.15;

/// The minimum tool travel (as a fraction of the toolpath's model-space span diagonal) before a fresh live position
/// is appended to the cut trail. It decimates the 5–10 Hz status feed and rejects sub-step jitter so a stationary
/// tool does not pile up coincident points; small enough that the trail still tracks the real cut path finely.
pub(crate) const TRAIL_MIN_STEP_FRACTION: f32 = 0.002;

/// The rolling cap on cut-trail points. Once exceeded the oldest are dropped (the trail is a `VecDeque`), bounding
/// memory and per-frame draw cost on a long job; the decimating step gate keeps a typical job well under.
pub(crate) const MAX_TRAIL_POINTS: usize = 30_000;

/// The model-space span diagonal used to scale the resolution-independent marker/trail tolerances. Clamped to a
/// small floor so a degenerate (single-point) program cannot collapse the tolerances to zero.
pub(crate) fn span_diagonal(span: egui::Vec2) -> f32 {
  (span.x * span.x + span.y * span.y).sqrt().max(1.0)
}

/// Scan a loaded program for its minimum (most negative) work-Z — the deepest programmed cut depth — so the executed
/// cut render can shade each segment by depth job-relative (brightest at the surface, darkest at this minimum). Tracks the
/// modal G90/G91 distance mode so relative Z words accumulate correctly, reusing the same lexer and modal handling
/// as [`parse_xy_path`]. Comments are stripped. A program that never goes below the work zero (XY-only, or only
/// positive Z) yields `0.0` — no depth range, so every cut later takes the full base colour.
pub(crate) fn program_min_z(lines: &[String]) -> f32 {
  let mut min = 0.0_f32;
  let mut z = 0.0_f32;
  let mut absolute = true; // modal distance mode: true == G90 (absolute), false == G91 (relative).
  for line in lines {
    let code = line.split(';').next().unwrap_or("").to_ascii_uppercase();
    for (letter, number) in gcode_words(&code) {
      match letter {
        'G' => match number.trim() {
          "90" => absolute = true,
          "91" => absolute = false,
          _ => {}
        },
        'Z' => {
          if let Ok(v) = number.parse::<f32>() {
            z = if absolute { v } else { z + v };
            min = min.min(z);
          }
        }
        _ => {}
      }
    }
  }
  min
}

/// Render the 2D toolpath viewport: a top-down XY preview of the loaded program drawn with [`egui::Painter`].
/// The path is fit to the available rect. When a live status report carries a work position the marker and the
/// "cut so far" colouring track the real machine position (smoothed against the status sample rate); with no
/// live status they fall back to the acknowledged-line position. Parsing is a cheap single pass done at load.
pub fn toolpath(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState) {
  let palette = state.style.palette;
  // Snapshot the resolved toolpath render style (stroke widths, grid, marker radius) before the mutable
  // `update_live_overlay` borrow below. `Copy`, so this is free and side-steps the borrow conflict.
  let tp = state.style.toolpath;
  // The viewport header carries the filename/line-count on the right, inside the 30px strip (design §03).
  let progress = view.progress;
  let name = state.program_path.as_deref().map(|p| p.rsplit(['/', '\\']).next().unwrap_or(p).to_string());
  header_bar(
    ui, palette,
    |ui| {
      header_title(ui, palette, "Toolpath");
      if let Some(name) = &name {
        ui.label(RichText::new(name).monospace().size(10.5).color(palette.text_disabled));
      }
    },
    |ui| {
      if progress.total > 0 {
        ui.label(RichText::new(format!("{} / {}", progress.acked, progress.total))
          .monospace().size(10.5).color(palette.text_disabled));
      }
    },
  );
  ui.add_space(2.0);
  let available = ui.available_size();
  let (response, painter) = ui.allocate_painter(available, egui::Sense::hover());
  let rect = response.rect;
  painter.rect_filled(rect, 0.0, palette.inset);
  draw_grid(&painter, palette, tp, rect);

  let Some((min, max)) = state.toolpath_bounds else {
    painter.text(rect.center(), egui::Align2::CENTER_CENTER, "no program loaded",
      egui::FontId::proportional(14.0), palette.text_dim);
    return;
  };

  // Fit the cached model-space bounds into the viewport with a uniform scale (the only per-frame geometry).
  let span = (max - min).max(egui::vec2(1.0, 1.0));
  let margin = 16.0;
  let scale = ((rect.width() - 2.0 * margin) / span.x).min((rect.height() - 2.0 * margin) / span.y).max(0.0001);

  // Map a model point into screen space, flipping Y so +Y is up as on a machine bed, and centring the fit.
  let used = span * scale;
  let offset = egui::vec2(rect.left() + (rect.width() - used.x) * 0.5, rect.top() + (rect.height() - used.y) * 0.5);
  let to_screen = |p: egui::Vec2| egui::pos2(offset.x + (p.x - min.x) * scale, offset.y + (max.y - p.y) * scale);

  // Fold this frame's live status into the overlay: extend the cut trail from the live work position (only while
  // cutting below the surface) and smooth the marker. Returns the model-space marker to draw, or `None` to draw no
  // dot. This mutates the cached state (trail/marker) before the immutable draw borrows of `state.toolpath`/
  // `state.trail` below.
  let live_marker = update_live_overlay(state, view, (min, max), span);

  // The program preview underneath is drawn uniformly DIM (cuts neutral, rapids dimmer) — it is the reference
  // geometry, not the progress. Progress is shown by the cut trail drawn over it, so there is no per-segment cut
  // colouring of the planned path (every host-side attempt to colour the planned geometry by progress — acked-line
  // prefix, WPos-onto-path projection — led the cutter; see the preview-module doc).
  for seg in &state.toolpath {
    let color = if seg.rapid { palette.border_raised } else { palette.text_dim };
    painter.line_segment([to_screen(seg.from), to_screen(seg.to)], egui::Stroke::new(1.0, color));
  }

  // The cut trail: the AXIS-style breadcrumb of where the tool has actually CUT, drawn over the dim preview as a set
  // of separate strokes split at pen-up boundaries. Only below-surface cutting moves were recorded (no rapids), so
  // every drawn segment is a cut: it takes the cut base colour SHADED by depth — brightest at the surface, darkest at
  // the program's deepest pass (see [`preview::depth_brightness`]) — at the cut stroke width. A segment is drawn into
  // a point only when it does NOT begin a fresh stroke AND it connects by distance (see
  // [`preview::connect_trail_segment`]): a `stroke_start` point is the first cut after a lift, so no line is ever
  // streaked from the prior cut straight across the travel (the cross-gap artefact); a large XY gap mid-stroke (a
  // reconnect/teleport) still breaks too. The move INTO a point carries that point's depth, so the segment is shaded
  // by `cur`'s depth.
  let break_gap = span_diagonal(span) * tp.trail_break_fraction;
  let mut prev: Option<&TrailPoint> = None;
  for cur in &state.trail {
    if let Some(a) = prev {
      let within_gap = preview::trail_connects((a.pos.x, a.pos.y), (cur.pos.x, cur.pos.y), break_gap);
      if preview::connect_trail_segment(cur.stroke_start, within_gap) {
        // Shade the one base cut colour by depth: multiply its brightness by the job-relative depth fraction so a
        // deeper pass reads darker. `gamma_multiply` scales perceptually, matching the marker ring's idiom below.
        let color = palette.toolpath_cut.gamma_multiply(preview::depth_brightness(cur.z, state.job_min_z));
        painter.line_segment([to_screen(a.pos), to_screen(cur.pos)], egui::Stroke::new(tp.cut_stroke_px, color));
      }
    }
    prev = Some(cur);
  }

  // The tool dot (warm motion accent) marks where the machine is now: the smoothed live position, projected
  // through the same fit transform so it sits on the geometry. Suppressed (no dot) when there is no live position.
  // The dot radius comes from the config style; the outer ring scales with it so the two stay proportional.
  if let Some(model) = live_marker {
    let pos = to_screen(model);
    painter.circle_filled(pos, tp.marker_radius_px, palette.toolpath_cut);
    painter.circle_stroke(pos, tp.marker_radius_px * 2.0, egui::Stroke::new(1.0, palette.toolpath_cut.gamma_multiply(0.5)));
  }
}

/// Whether a run state is one of active motion (Run/Jog/Hold) — the states in which the live tool marker is shown
/// unconditionally (a legitimate cut may run just outside the drawn extents). Every other state (Idle/Alarm/…) is
/// gated on the marker actually lying near the path, so a parked-off-path point is suppressed (finding #4).
pub(crate) fn is_moving_state(run_state: Option<crate::protocol::RunState>) -> bool {
  use crate::protocol::RunState::{Hold, Jog, Run};
  matches!(run_state, Some(Run | Jog | Hold))
}

/// Whether this frame is the start of a fresh run: a rising edge into `Run` from any non-`Run` state (Idle/Hold/…)
/// or from no status at all. This is the job-start point at which the previous run's accumulated cut trail is wiped
/// — edge-detected so it fires once per run, not on every mid-run `Run` status frame (which would erase the live
/// trail as fast as it draws). A resume out of a feed-hold (`Hold → Run`) counts as an entry too; the trail begins
/// fresh from the resume rather than carrying the pre-hold path, which is the desired "this run" framing. Pure so
/// the edge rule is unit-tested.
pub(crate) fn entered_run(prev: Option<crate::protocol::RunState>, current: Option<crate::protocol::RunState>) -> bool {
  use crate::protocol::RunState::Run;
  current == Some(Run) && prev != Some(Run)
}

/// Fold one frame of live status into the cached overlay: extend the position trail from the raw work position and
/// smooth the model-space marker toward it. Returns the smoothed marker in model space when a marker dot should be
/// drawn, or `None` for no dot. Kept as a small helper so the per-frame state mutation is isolated from rendering;
/// the smoothing, trail, and gating decisions themselves live in pure, unit-tested [`crate::app::preview`] functions.
///
/// Three live-status cases are distinguished (findings #3/#4/#5/#6):
/// - **No status at all** (disconnected / pre-connect): drop the held marker so a later reconnect snaps fresh.
/// - **A status with no derivable work position** (grbl pushes `WCO` only intermittently, so a mid-run report can
///   lack one): HOLD the last marker through the gap rather than nulling it — nulling would re-snap on the next
///   valid frame, a visible blink/teleport. The trail is simply not extended this frame.
/// - **A status with a work position**: extend the trail and smooth toward it, but draw the marker dot only when
///   [`preview::marker_is_on_path`] passes (on/near the path, or while actively moving) — a parked machine far off
///   the path (homed to machine origin, a units mismatch, a WCS mismatch) draws no dot rather than a
///   confident-but-wrong one.
pub(crate) fn update_live_overlay(state: &mut UiState, view: &ViewState, bounds: (Vec2, Vec2), span: egui::Vec2) -> Option<Vec2> {
  // A new run starts here: on the rising edge into Run (Idle/Hold → Run, or the first Run after connect/load) wipe
  // the prior run's accumulated cut trail and drop the stale marker, so the fresh job draws over the dim planned
  // preview alone. Edge-detected against the previous frame's state so it fires once per run, never mid-run. The
  // previous state is updated every frame (including the no-work-position frames below), so the edge stays accurate.
  let run_state = view.status.as_ref().map(|s| s.machine_state.state);
  if entered_run(state.prev_run_state, run_state) {
    state.trail.clear();
    state.marker_pos = None;
    // A fresh run lifts the pen: the first cut of the new run must begin its own stroke, not continue the prior run.
    state.prev_frame_was_cut = false;
  }
  state.prev_run_state = run_state;
  // Capture whether the PREVIOUS frame was a cut (for the stroke-start decision), then default this frame to "not a
  // cut". Only a successful cut-point append below flips it back to `true`, so every other path — a lift, a non-Run
  // frame, a decimated frame, or a frame with no derivable position — correctly registers as a pen-up.
  let prev_was_cut = state.prev_frame_was_cut;
  state.prev_frame_was_cut = false;

  let Some(target) = view.work_xy().map(|(x, y)| (x as f32, y as f32)) else {
    // No derivable work position this frame. Two cases (finding #6):
    // - No status at all (disconnected / pre-connect): clear the marker so a later reconnect snaps fresh.
    // - A status with no derivable work position (grbl pushes WCO only intermittently, so a mid-run report can
    //   lack one): HOLD the last marker AND keep returning it, so the dot stays put rather than blinking out and
    //   re-snapping next frame.
    if view.status.is_none() {
      state.marker_pos = None;
      return None;
    }
    return state.marker_pos.map(|p| egui::vec2(p.x, p.y));
  };
  let diagonal = span_diagonal(span);
  let snap_dist = diagonal * MARKER_SNAP_SPAN_FRACTION;
  let current = state.marker_pos.map(|p| (p.x, p.y));
  let smoothed = preview::smooth_marker(current, target, snap_dist, MARKER_LERP);
  state.marker_pos = Some(egui::vec2(smoothed.0, smoothed.1));

  // Extend the CUT trail from the RAW live work position (not the smoothed marker, which lags) — but ONLY while the
  // machine is in Run AND the tool is engaged below the work surface (live work-Z < 0). A jog, a feed-hold, or an
  // above-surface travel/clearance move is not program cutting, so it records nothing — this Z < 0 gate is exactly
  // why a lead-in/rapid sample (which runs above the surface) can never contaminate the trail and make it lead the
  // cutter. The point carries its cut depth so the draw step can shade it. The step gate decimates the status feed;
  // the rolling cap bounds it. A breadcrumb of where the tool has ACTUALLY been cannot, by construction, lead it.
  let running = matches!(view.status.as_ref().map(|s| s.machine_state.state), Some(crate::protocol::RunState::Run));
  let live_z = view.work_z().map(|z| z as f32);
  if running
    && let Some(depth) = preview::cut_segment_depth(live_z)
  {
    // This cut point begins a fresh stroke when the previous frame was NOT a cut — a pen-down after a lift (a rapid
    // or retract to Z >= 0) or the first cut of the run — so the draw loop breaks the polyline before it rather than
    // streaking a line across the travel. The flag rides on the point only when it actually appends.
    let stroke_start = !prev_was_cut;
    let min_step = diagonal * TRAIL_MIN_STEP_FRACTION;
    let appended = push_trail_point(&mut state.trail, egui::vec2(target.0, target.1), depth, stroke_start, min_step);
    // The pen is down for this stroke once a cut frame is seen: on a real append, or if it was already down and this
    // sub-step (decimated) cut frame did not break it. Without the `prev_was_cut` carry, a decimated cut mid-stroke
    // would reset the tracker and spuriously flag the NEXT appended cut as a new stroke (a false break, not a join).
    state.prev_frame_was_cut = appended || prev_was_cut;
  }

  // Gate the marker: drawn on/near the path or while actively moving, suppressed for a parked-off-path point
  // (#3/#4/#5). Use the SMOOTHED position so the gate matches what is drawn.
  let (min, max) = bounds;
  let margin = diagonal * MARKER_ON_PATH_MARGIN_FRACTION;
  let moving = is_moving_state(view.status.as_ref().map(|s| s.machine_state.state));
  if preview::marker_is_on_path(smoothed, (min.x, min.y), (max.x, max.y), margin, moving) {
    Some(egui::vec2(smoothed.0, smoothed.1))
  } else {
    None
  }
}

/// Paint the viewport's major/minor reference grid, matching the design's two-tone grid over the inset canvas. The
/// minor-line spacing and how many minor cells make a major line come from the config's [`ToolpathStyle`]; the two
/// line colours come from the active palette's `grid_major`/`grid_minor` tokens.
fn draw_grid(painter: &egui::Painter, palette: Palette, tp: ToolpathStyle, rect: egui::Rect) {
  let minor = palette.grid_minor;
  let major = palette.grid_major;
  let step = tp.grid_minor_px;
  let every = tp.grid_major_every as usize;
  let mut x = rect.left();
  let mut i = 0;
  while x <= rect.right() {
    let color = if i % every == 0 { major } else { minor };
    painter.line_segment([egui::pos2(x, rect.top()), egui::pos2(x, rect.bottom())], egui::Stroke::new(1.0, color));
    x += step;
    i += 1;
  }
  let mut y = rect.top();
  let mut j = 0;
  while y <= rect.bottom() {
    let color = if j % every == 0 { major } else { minor };
    painter.line_segment([egui::pos2(rect.left(), y), egui::pos2(rect.right(), y)], egui::Stroke::new(1.0, color));
    y += step;
    j += 1;
  }
}

/// One straight XY segment of the parsed toolpath, drawn as the dim planned-geometry reference. Depth shading is a
/// property of the live CUT TRAIL (see [`TrailPoint`]), not of the planned preview, so a segment carries no Z.
#[derive(Debug, Clone)]
pub(crate) struct Segment {
  pub(crate) from: egui::Vec2,
  pub(crate) to: egui::Vec2,
  /// Whether this is a rapid (G0) travel move rather than a cut.
  pub(crate) rapid: bool,
}

/// Parse the loaded program into a flat list of XY segments for the preview. A pragmatic linear interpreter:
/// it tracks the modal motion mode (G0 travel vs G1 cut vs G2/G3 arc) and the modal distance mode (G90 absolute /
/// G91 relative), then emits one or more segments per line that actually executes a motion. A line emits geometry
/// only when the effective motion mode is G0/1/2/3 AND it carries an X/Y word AND it is not a non-motion command:
/// G10/G28/G30/G92/G53/G4 take X/Y as parameters (or modify a single line) rather than as a normal modal move, so
/// they update no position and draw nothing. Tokens may be spaced (`G1 X10 Y5`) or compact (`G1X10.0Y5.0`).
///
/// **Arcs (G2/G3) are FLATTENED into many short chord segments** via [`crate::app::preview::flatten_arc`], using the
/// I/J centre offset (XY plane / G17 assumed — the firmware's only arc plane). This makes the preview draw the
/// real curve rather than a single start→end chord. An arc with neither I/J nor a usable centre degrades to a
/// single straight chord, so a malformed or R-form arc never breaks the parse.
pub(crate) fn parse_xy_path(lines: &[String], arc_step_rad: f32) -> Vec<Segment> {
  let mut segments = Vec::new();
  let mut pos = egui::vec2(0.0, 0.0);
  // Modal motion mode: 0 == G0 rapid, 1 == G1 cut, 2 == G2 CW arc, 3 == G3 CCW arc.
  let mut motion = 0u8;
  let mut absolute = true; // modal distance mode: true == G90 (absolute), false == G91 (relative).
  for line in lines {
    let code = line.split(';').next().unwrap_or("").to_ascii_uppercase();
    if code.trim().is_empty() {
      continue;
    }
    let mut next = pos;
    let mut has_xy = false;
    let mut suppress = false; // a non-motion G-word on this line suppresses any segment for it.
    // Arc centre offsets (I/J) relative to the current position; collected only for an arc line. Both default to
    // zero (grbl treats an absent offset as zero), so an arc giving only one of I/J still resolves a centre.
    let (mut arc_i, mut arc_j) = (0.0_f32, 0.0_f32);
    let mut has_ij = false;
    for (letter, number) in gcode_words(&code) {
      match letter {
        'G' => match number.trim() {
          "0" | "00" => motion = 0,
          "1" | "01" => motion = 1,
          "2" | "02" => motion = 2,
          "3" | "03" => motion = 3,
          "90" => absolute = true,
          "91" => absolute = false,
          // Non-modal commands whose X/Y are parameters, not a move; they must not draw a segment.
          "10" | "28" | "30" | "92" | "53" | "4" | "04" => suppress = true,
          _ => {}
        },
        'X' => {
          if let Ok(v) = number.parse::<f32>() {
            next.x = if absolute { v } else { pos.x + v };
            has_xy = true;
          }
        }
        'Y' => {
          if let Ok(v) = number.parse::<f32>() {
            next.y = if absolute { v } else { pos.y + v };
            has_xy = true;
          }
        }
        'I' => {
          if let Ok(v) = number.parse::<f32>() {
            arc_i = v;
            has_ij = true;
          }
        }
        'J' => {
          if let Ok(v) = number.parse::<f32>() {
            arc_j = v;
            has_ij = true;
          }
        }
        _ => {}
      }
    }
    if has_xy && !suppress {
      let rapid = motion == 0;
      if (motion == 2 || motion == 3) && has_ij {
        // An arc with a usable I/J centre: flatten it into chords. I/J are offsets from the START position.
        let center = (pos.x + arc_i, pos.y + arc_j);
        let mut from = (pos.x, pos.y);
        for point in crate::app::preview::flatten_arc(from, (next.x, next.y), center, motion == 2, arc_step_rad) {
          segments.push(Segment { from: egui::vec2(from.0, from.1), to: egui::vec2(point.0, point.1), rapid });
          from = point;
        }
      } else {
        // A linear move, or an arc with no centre offset (degrade to its chord rather than guessing a centre).
        segments.push(Segment { from: pos, to: next, rapid });
      }
      pos = next;
    }
  }
  segments
}

/// The model-space `(min, max)` bounds of a parsed toolpath, or `None` when it is empty. Computed once at
/// program-load time (see [`UiState::set_program`]) so the viewport never re-scans the segments per frame.
pub(crate) fn toolpath_bounds(segments: &[Segment]) -> Option<(Vec2, Vec2)> {
  if segments.is_empty() {
    return None;
  }
  let mut min = egui::vec2(f32::INFINITY, f32::INFINITY);
  let mut max = egui::vec2(f32::NEG_INFINITY, f32::NEG_INFINITY);
  for seg in segments {
    for p in [seg.from, seg.to] {
      min.x = min.x.min(p.x);
      min.y = min.y.min(p.y);
      max.x = max.x.max(p.x);
      max.y = max.y.max(p.y);
    }
  }
  Some((min, max))
}

/// One point of the cut-progress trail: a model-space (work XY) tool position, the live work-Z (the cut DEPTH) at
/// that point so the trail can shade each segment by how deep the cut was, and a `stroke_start` pen-up/pen-down flag.
/// Only points cut below the work surface (`z < 0`) are ever pushed, so the stored `z` is always a negative depth.
/// `stroke_start` is true when this point begins a FRESH stroke — the first cut after a lift (the tool retracted to
/// Z >= 0 between cuts) or the first point of the trail — so the draw loop never joins a line back across the travel.
#[derive(Debug, Clone, Copy)]
pub(crate) struct TrailPoint {
  pub(crate) pos: Vec2,
  z: f32,
  pub(crate) stroke_start: bool,
}

/// Append the live work position to the trail if the tool has moved at least `min_step` from the last point, and
/// enforce the rolling [`MAX_TRAIL_POINTS`] cap (oldest dropped). `z` is the live work-Z (the cut depth) carried into
/// the point so the draw step can shade it; `stroke_start` flags it as the start of a fresh stroke (a pen-down after
/// a lift) so the draw loop breaks the polyline before it. Returns whether the point was actually appended (false
/// when decimated by the step gate), so the caller can track pen-down only on a real append. The append decision
/// itself is the pure [`crate::app::preview::trail_should_append`]; this just owns the `VecDeque` mutation.
fn push_trail_point(
  trail: &mut std::collections::VecDeque<TrailPoint>, point: Vec2, z: f32, stroke_start: bool, min_step: f32,
) -> bool {
  let last = trail.back().map(|p| (p.pos.x, p.pos.y));
  if !preview::trail_should_append(last, (point.x, point.y), min_step) {
    return false;
  }
  trail.push_back(TrailPoint { pos: point, z, stroke_start });
  while trail.len() > MAX_TRAIL_POINTS {
    trail.pop_front();
  }
  true
}

/// Split one (comment-stripped, upper-cased) G-code line into `(letter, number)` words, handling both spaced
/// (`G1 X10 Y5`) and compact (`G1X10.0Y5.0`) layouts. Each word is a letter `A..=Z` followed by its numeric
/// run (digits, sign, decimal point); whitespace and stray characters between words are skipped. Yields the
/// number as a `&str` slice so the caller parses only the words it cares about — no per-word allocation.
fn gcode_words(code: &str) -> impl Iterator<Item = (char, &str)> {
  let bytes = code.as_bytes();
  let mut i = 0;
  std::iter::from_fn(move || {
    while i < bytes.len() {
      let c = bytes[i] as char;
      if c.is_ascii_alphabetic() {
        let letter = c;
        let start = i + 1;
        let mut end = start;
        while end < bytes.len() {
          let n = bytes[end] as char;
          if n.is_ascii_digit() || n == '.' || n == '-' || n == '+' {
            end += 1;
          } else {
            break;
          }
        }
        i = end;
        return Some((letter, &code[start..end]));
      }
      i += 1;
    }
    None
  })
}
