---
name: toolpath-progress-boundary
description: The toolpath "cut so far" path is a Z<0 WPos breadcrumb polyline (trail) of actual tool positions — NOT any coloring of the planned path (acked-prefix and WPos-projection both LED the cutter and are removed)
metadata:
  type: project
---

**THE CORRECT MODEL (restored 2026-06-29 from commit `8b4e08c`):** the toolpath viewport's bright "what has been
cut" path is a LinuxCNC-AXIS-style BREADCRUMB polyline of the tool's ACTUAL reported WPos positions, accumulated ONLY
while cutting below the work surface (live work-Z < 0). The live WPos is the marker dot. A breadcrumb of where the
tool has actually, observably been CANNOT lead the cutter — and the **Z < 0 gate** is why lead-ins/rapids (which run
above the surface, Z ≥ 0) never contaminate it. The gray planned geometry (`parse_xy_path` → `Segment{from,to,rapid}`)
is drawn as a uniform DIM reference underneath; the bright depth-shaded trail rides over it. DO NOT reintroduce any
"colour the planned-path geometry by a host-side progress signal" approach — every such attempt LED the cutter.

**Mechanics (all in views.rs + preview.rs):** `UiState.trail: VecDeque<TrailPoint{pos,z,stroke_start}>` (cap
`MAX_TRAIL_POINTS=30_000`, rolling). `update_live_overlay` appends via `push_trail_point` ONLY when machine is `Run`
AND `preview::cut_segment_depth(work_z) == Some(z<0)`, decimated by `trail_should_append` (`TRAIL_MIN_STEP_FRACTION=
0.002 * span_diagonal`). `prev_frame_was_cut` tracks pen-up/down: the first cut after a non-cut frame is
`stroke_start=true`. Draw loop joins consecutive points only when `preview::connect_trail_segment(stroke_start,
within_gap)` (NOT a stroke start AND within `trail_break_fraction * span_diagonal` — breaks across lifts and
teleports, no cross-gap streak). Depth shading: `preview::depth_brightness(point.z, job_min_z)`; `job_min_z` from
`program_min_z` (scans lines, not Segments). `entered_run` (Idle/Hold→Run rising edge) WIPES the trail + marker (incl
a Hold→Run resume — deliberate "this run" framing). §16's undersampling concern is handled by drawing LINE SEGMENTS
between consecutive samples (a straight cut between two sparse samples renders straight), plus a stroke_start vertex on
each pen-down — residual gap risk is only slight coarseness, never leading.

**TWO FAILED APPROACHES (both removed — do not retry):**
1. **Acked-line prefix** (`6682e4a` regressed `8b4e08c` into this): colour planned segment k done when `line_idx <
   progress.acked`. grbl `ok`s a line on planner ACCEPT, so acked leads true motion by the planner-buffer depth →
   bright swath races far ahead of the dot. (`Ln:` executing-line is unusable: Galdr firmware emits no line field and
   `.tap` files carry no `N` words, so it's always absent.)
2. **Single-point WPos→planned-path projection** (`progressed_segment`, bounded window or contiguous near-walk): on a
   self-crossing / near-parallel-offset-contour part a lead-in passing NEAR a far segment projects onto it and a
   monotonic clamp locks the boundary ahead. CONFIRMED by host repro: a sample `(0.5, 9.0)` ratcheted the boundary 9
   segments ahead before any cutting. Single-point projection onto an ambiguous path fundamentally cannot avoid leading.

**Tests:** `preview.rs` (GUI-free) — `trail_should_append`/`trail_connects`/`connect_trail_segment`/`cut_segment_depth`
(the Z gate). `views.rs` integration over `update_live_overlay` + a square fixture (`feed_cut_sample` helper):
Z≥0 samples add NO vertices; trail vertices are exactly the Z<0 samples in order and a subset of reported positions;
the `(0.5,9.0)` lead-in at Z≥0 contributes nothing; a pen-up lift starts a fresh stroke; a fresh run-entry wipes.
600 lib + 3 bin green, clippy clean, GUI builds.

**LESSON:** read the git history of a regression FIRST. The working model lived at `8b4e08c`; `6682e4a` ("Trying to
get firmware to work over longer periods of time") replaced it with the acked-prefix. Two reasoning-based redesigns
(acked-cap, projection) both failed before restoring the proven breadcrumb. Related: [[toolpath-live-motion]] and
[[toolpath-render-two-layers]] and [[toolpath-depth-trail]] (their projection/acked-prefix notes are STALE — model is
the Z<0 breadcrumb again), [[engine-architecture]], [[gui-architecture]].
