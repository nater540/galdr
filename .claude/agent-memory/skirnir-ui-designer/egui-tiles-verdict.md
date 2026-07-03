---
name: egui-tiles-verdict
description: DECISION REVERSED (2026-07-02) — egui_tiles 0.15 now hosts the viewport/console split; why the pass verdict was overridden
metadata:
  type: project
---

ORIGINAL verdict (2026-07-02 morning): pass on egui_tiles — fixed machine-control chrome vs its rearrangeable-
workspace purpose, and the resize bug seemed fixable in-place.

**REVERSED by user decision the same day**: THREE successive fixes to the hand-rolled resizable dock (exact-fill
content, content-fill windows, a persisted-size write-back guard) each went green in kittest and kept
self-resizing on the user's desktop. The theoretical assessment lost to repeated real-world failure — the panel
content-rect persistence model was the problem, and tiles' top-down share sizing removes the feedback path
entirely rather than guarding it.

**Shape shipped** (`crates/skirnir/src/app/dock_tiles.rs`): egui_tiles 0.15 (egui-0.34 match) manages ONLY the
central [toolpath viewport | console dock] vertical split; all fixed chrome stays hand-rolled. Dragging disabled
(`Behavior::is_tile_draggable → false`), no Tabs containers (panes render bare; the dock keeps its own strip),
`min_size = DOCK_MIN_H`, divider = 1px gap. Split fraction persists via `profile.ron` (`Prefs::dock_fraction`,
default `profile::DEFAULT_DOCK_FRACTION = 0.3` — lives in the ungated profile module) — NOT config.json
(operator-owned, explicit-save-only) and NOT egui memory (eframe persistence off). The collapsed dock is still a
hand-rolled full-width exact strip. Consequence accepted per scope: the expanded dock spans the viewport width,
not the window width, and the side columns run full height (which un-clipped the jog pad + rotary panels).

**Verification lesson that drove it**: `SKIRNIR_SIZE_TRACE=1` (env-gated, left in the shell) logs the dock rect
per change + forces repaints — desktop-truth evidence the harness can't fake. Real-binary runs: the pre-tiles
guard was stable under 15s of forced repaints (the remaining desktop gap needs real pointer input or a state
transition — never identified); tiles: one initial line, zero changes over 25s. Also: egui ScrollArea's default
`min_scrolled_height=64` overflows tight panes — the console log sets it to 0.

Related: [[egui-034-layout-gotchas]], [[egui-elegance-verdict]].
