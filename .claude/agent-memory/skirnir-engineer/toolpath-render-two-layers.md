---
name: toolpath-render-two-layers
description: Toolpath viewport = dim planned preview + a DETERMINISTIC executed-prefix progress backplot (acked-line index, not status samples) + live marker; §16 render-gap bug fixed
metadata:
  type: project
---

The toolpath viewport (`crates/skirnir/src/app/views.rs` `toolpath()`) renders a dim PLANNED preview plus a bright
EXECUTED progress backplot, with a live marker dot. A reported "missing chunks of the path" means different things
depending on which layer gaps.

**Layer 1 — planned preview (dim):** `parse_xy_path` (views.rs) emits a `Segment` for EVERY XY move (G0/G1/G2/G3;
`rapid` is only a colour flag, never a cull). Provably complete + deterministic — it CANNOT drop a cut. Gaps here ⇒
suspect the parser (near-impossible by construction).

**Layer 2 — progress backplot (bright): DETERMINISTIC since the §16 fix (2026-06-26).** Each `Segment` carries
`line_idx` (source program line, counted via `enumerate` so it stays aligned with the engine's acked-line count past
blanks/comments) + modal `z`. The single draw loop in `toolpath()` draws a cut segment bright + depth-shaded when
`segment_executed(seg.line_idx, view.progress.acked)` (i.e. `line_idx < acked`); planned cuts and ALL rapids stay dim.
Gap-free + reproducible by construction. The live WPos marker (update_live_overlay, now a pure marker-updater) is the
real-time cursor. `acked` leads true motion by the planner-buffer depth — fine for a progress backplot.

**History (the bug this fixed):** commit 8b4e08c had built a status-SAMPLED trail (recorded only when Run AND
work-Z<0, sampled at ~10 Hz, decimated). That under-sampled — a move finishing between two polls left no segment →
NON-deterministic gaps, different each run, identical-looking to a firmware motion-BLOCK drop but render-only (the §16
bug; steppers were never connected, so the user only ever saw the screen). The deterministic-prefix rewrite removed
BOTH the under-sampling gaps AND the deterministic Z>=0-cut-misclassified-as-non-cut gap (surface/engrave jobs) — it
splits by line index now, not Z sign. Removed with it: `TrailPoint`, `push_trail_point`, the stroke/pen-up-down
machinery, `TRAIL_MIN_STEP_FRACTION`/`MAX_TRAIL_POINTS`, and preview.rs helpers `trail_should_append` /
`trail_connects` / `connect_trail_segment` / `cut_segment_depth`. Kept: `depth_brightness`/`DEPTH_BRIGHTNESS_FLOOR`,
`smooth_marker`, `marker_is_on_path`.

**Discriminator for any future "missing chunks" report:** firmware air-run `exec` vs `acks` — `exec == acks` + a
complete planned preview ⇒ a render concern, firmware innocent. First question: gaps in the DIM planned geometry
(layer 1 — parser) or the BRIGHT executed path (layer 2)?

Related: [[gui-architecture]], [[toolpath-live-motion]], [[toolpath-depth-trail]], [[engine-architecture]].
