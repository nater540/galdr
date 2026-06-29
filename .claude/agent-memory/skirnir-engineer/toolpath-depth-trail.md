---
name: toolpath-depth-trail
description: Cut-progress trail (views.rs) is Run-only, Z-sign gated, and shaded by job-relative cut depth from one yellow base colour
metadata:
  type: project
---

The live toolpath trail draws the completed CUT path during a run, coloured by depth — it replaced the old
feed-rate rapid/cut classification entirely.

**Why:** Realized-feed rapid/cut classification was ambiguous; the live work-Z sign is unambiguous (z<0 = engaged
cut, z>=0 = travel/retract). Depth gives an at-a-glance multi-pass cue.

**How to apply (where things live, all under crates/skirnir/src):**
- Pure helpers in `app/preview.rs`: `cut_segment_depth(Option<f32>) -> Option<f32>` (Z-sign gate), and
  `depth_brightness(z, job_min_z) -> f32` (job-relative t = z/job_min_z clamped, lerp 1.0→floor). The tunable
  floor is `DEPTH_BRIGHTNESS_FLOOR: f32 = 0.35` in preview.rs. `is_rapid_feed` was REMOVED (was only used here).
- **Pen-up/pen-down stroke breaks (hardware-found bug fix):** the trail is a SET of strokes, not one polyline. Each
  `TrailPoint` has `stroke_start: bool` (true = first cut after a lift); `UiState.prev_frame_was_cut` tracks pen
  state across frames (cut frame = Run && z<0). The draw loop joins a segment only via
  `preview::connect_trail_segment(stroke_start, within_gap) = !stroke_start && within_gap` — so a lift (Z>=0 frame)
  ends the stroke and the next plunge starts a fresh one with NO bridging line, regardless of XY distance. The old
  `trail_connects` distance gate alone was insufficient (a lift+plunge near the last cut bridged the gap).
  `push_trail_point` now takes `stroke_start` and RETURNS bool (appended). Pen-down carry rule in update_live_overlay:
  `prev_frame_was_cut = appended || prev_was_cut` (so a decimated mid-stroke cut frame doesn't falsely break next).
  Reset prev_frame_was_cut=false on run-entry clear AND in set_program.
- `app/views.rs`: `TrailPoint` is now `{ pos: Vec2, z: f32 }` (was `{ pos, rapid }`); only below-surface cut
  points are pushed, so `z` is always negative. `UiState.job_min_z` replaced `max_feed`; scanned at load by
  `program_min_z(lines)` (replaced `max_programmed_feed`) — it tracks modal G90/G91 like parse_xy_path.
- Trail extends in `update_live_overlay` ONLY when `machine_state == Run` AND `cut_segment_depth(work_z)` is Some.
  Jog/Hold/Idle record nothing. The marker dot still uses the broader `is_moving_state` (Run/Jog/Hold) — that gate
  was deliberately NOT changed.
- Draw shades the one base colour: `palette.toolpath_cut.gamma_multiply(depth_brightness(cur.z, job_min_z))`.
  `toolpath_rapid` is retained as a config field but no longer drawn.
- Live Z derivation: `ViewState::work_z()` in `view_state.rs` mirrors `work_xy()` (index 2, WCO-derived, no alloc).

**Theme:** `toolpath_cut` default changed `#FF7A1A` (orange) → `#FFE000` (yellow) in BOTH `app/theme.rs`
`default_dark()` (light_slate/midnight inherit via `..default_dark()`) AND the bundled
`assets/config.default.json`'s fully-keyed `default` theme — the JSON override wins at runtime, so both must move
together or the runtime palette stays orange. The separate `accent_motion` token is also `#FF7A1A`; leave it.

**Deviation from spec:** the min-Z scan lives in views.rs (next to `gcode_words`/`parse_xy_path`), NOT preview.rs,
because it must lex G-code + track modal distance mode — preview.rs is deliberately lexer-free pure geometry.

**Run-entry trail clear (follow-up):** the prior run's trail is wiped on the rising edge into Run so a new job
draws over the dim preview alone. Edge-detected (NOT per-frame) via `entered_run(prev, current)` in views.rs (next
to `is_moving_state`; `current==Some(Run) && prev!=Some(Run)`) against a new `UiState.prev_run_state` field. The
check sits at the TOP of `update_live_overlay` (clears `trail` + `marker_pos`, then records `prev_run_state` every
frame incl. the no-work-position early-returns). Hold→Run (resume) counts as an entry — trail restarts from resume.
`set_program` already cleared trail/marker for new-file loads; it now also resets `prev_run_state` to None. There is
no separate "job start" hook to clamp to — the status run-state edge IS the job-start signal the overlay sees.
