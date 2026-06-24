---
name: toolpath-live-motion
description: Toolpath preview live-motion overlay — pure preview.rs helpers, arc flattening, monotonic projection progress, marker gating, allocation-free work_xy, adaptive poll
metadata:
  type: project
---

The TOOLPATH preview's position dot + "cut so far" colouring are driven by the live machine WPos (the `<...>`
status report), not by `Progress::acked` (which leads the real cutter, since grbl `ok`s a line on planner-buffer
entry, not move completion). See [[gui-architecture]] and [[engine-architecture]]. A 14-finding review (2026-06-24)
redesigned the Phase-3 progress logic — the notes below reflect the post-redesign code.

**Pure helpers in `app/preview.rs` (NOT gui-gated — plain `(f32,f32)` model-space tuples, egui-free, unit-tested):**
- `smooth_marker(current, target, snap_dist, t) -> ModelPoint` — lerp-ONLY (no extrapolation → no overshoot)
  toward the latest sample; snap on `None` (first sample) or a jump ≥ snap_dist. `t` clamped.
- `flatten_arc(start, end, center, clockwise) -> Vec<ModelPoint>` — flattens a G2/G3 arc (XY/G17) into chord END
  points (excl start, incl exact end) at ≤ `MAX_ARC_STEP_RAD` (~9°). Sweep normalised positive in travel dir;
  start==end ⇒ full circle; degenerate radius ⇒ just the end. `parse_xy_path` calls it using the I/J centre offset.
- `progressed_segment(segments: &[Segment], work, previous, window) -> usize` — REPLACED the old
  furthest-forward-endpoint scan. Now projects the live point onto each `(start,end)` pair in a BOUNDED forward
  window `[previous, previous+window]`, picks the nearest by perpendicular distance, counts segment k complete
  (k+1) once the projection passes `COMPLETE_FRACTION=0.5` of it, else holds at k. Monotonic (`.max(previous)`),
  O(window)/frame. `Segment = ((f32,f32),(f32,f32))`. Helper `project_onto` clamps fraction to 0..=1.
  WHY the redesign: the old endpoint scan leapt the boundary to job-end the instant the cutter neared ANY revisited
  coord (closed contours, peck drilling, return to X0Y0 — `T1_Test.tap`), stalled across a single arc chord, and
  lagged on long moves vs the 2%-reach tol. Projection-from-previous-forward kills all three.
- `marker_is_on_path(point, min, max, margin, moving) -> bool` — gate: always true while `moving` (Run/Jog/Hold),
  else require the point within bounds+margin. Suppresses a parked-off-path dot (homed to machine origin, a 25.4×
  units mismatch, a WCS-mismatch float) rather than drawing it clipped to the rect edge.

**`ViewState::work_xy() -> Option<(f64,f64)>` (view_state.rs):** allocation-free work-XY accessor for the hot path
(overlay runs ≤20 Hz). `dro()` clones `position` + builds a derived Vec; `work_xy()` reads X/Y direct for a WPos
report, or derives only the two components from cached WCO for an MPos report. Use this in the overlay, NOT `dro()`.

**View wiring (`app/views.rs`):** UiState fields reset in `set_program`: `marker_pos`, `progress_segment`,
`progress_segments: Vec<preview::Segment>` (cached (start,end) pairs, zero per-frame alloc). `update_live_overlay(
state, view: &ViewState, bounds, span)` does per-frame mutation + returns the drawn marker. THREE work-pos cases
(finding #6): no status at all ⇒ null marker + fall back to acked dot; status with no derivable work (transient WCO
gap) ⇒ HOLD the marker AND keep returning it (no blink/re-snap, colouring stays live); status with work ⇒ smooth +
project + gate via `marker_is_on_path`. Per-segment draw loop hoists the live/acked choice (`let live =
live_marker.is_some()`) so the acked endpoint isn't computed when live wins (finding #11). Constants:
`PROGRESS_FORWARD_WINDOW=24`, `MARKER_ON_PATH_MARGIN_FRACTION=0.15`, `MARKER_LERP=0.35`,
`MARKER_SNAP_SPAN_FRACTION=0.25`, `span_diagonal()` floored at 1.0. `is_moving_state(Option<RunState>)` = Run|Jog|Hold.
NOTE: units (G20/G21) are NOT tracked program- or firmware-side; the bounds gate is the deliberate fail-safe that
covers the units (#3) and WCS (#5) displacement symptoms — do not add full WCS tracking unless a real need appears.

**Adaptive poll (`engine.rs`):** `poll_interval_for(Option<RunState>)` → `STATUS_POLL_RUN`(100ms/10Hz) only for Run,
else `STATUS_POLL_IDLE`(200ms/5Hz). `handle_inbound` uses the cheap `protocol::peek_run_state(body)` (leading-token
split, NOT a full `parse_status` — finding #9; reducer still owns the full decode). `refresh_poll_interval` rebuilds
the Interval only on a rate change, first tick `next_tick_delay(old,new)=old.min(new)` out — the SHORTER period — so
rapid Run↔Idle flapping can't keep postponing the next poll past that bound (finding #8). Tested under `start_paused`.
