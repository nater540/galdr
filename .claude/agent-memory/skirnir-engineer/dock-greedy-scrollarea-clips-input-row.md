---
name: dock-greedy-scrollarea-clips-input-row
description: Console MDI vanished TWO ways — (1) greedy auto_shrink ScrollArea pushes the input row below the exact_size dock floor (cap max_height); (2) Send button in a nested right_to_left child eats the row width, collapsing the TextEdit to ~0px (size the field first). Test BOTH height AND width via the real Panel::bottom harness.
metadata:
  type: project
---

In an `exact_size` bottom panel (the 200px dock, `Metrics::DOCK_H`), a `ScrollArea::vertical().auto_shrink([false,
false])` with NO `max_height` expands to consume the ENTIRE remaining height of the parent `Ui`. Any sibling widget
laid out AFTER it starts at the panel floor and extends BELOW it, where it is clipped away and invisible — even
though it renders (it has a rect, accesskit finds it, it's just off-panel).

This was the "no MDI / text-input on the Console tab" bug: `console_body` (views.rs) draws the log scroll area then
the manual-command (MDI) input row beneath it. The greedy scroll area ate all 200px, so the input row landed below
the dock floor. **Fix:** cap the scroll area's `max_height` to `ui.available_height() - (PANEL_CONTROL_H +
item_spacing.y)`, `.max(0.0)`, reserving the input row's height so it stays on-screen. (Caught: field bottom 589.5
vs dock floor 576 BEFORE; 564.5 AFTER.)

**SECOND half — the TextEdit had ~0 WIDTH (only Send showed).** The MDI row drew the Send button FIRST inside a nested
`ui.with_layout(Layout::right_to_left(..))` child. That child claims the WHOLE remaining row width, so the `TextEdit`
added afterward with `desired_width(f32::INFINITY)` had zero width left and collapsed to an invisible ~21px sliver
beside Send. **Fix:** size the FIELD first, left-to-right, with an EXPLICIT `desired_width(available − send_w − gap)`
floored at `MDI_FIELD_MIN_W=120`, then add the fixed-width Send button after it (`ui.available_width()` is read once
up front). General rule for "field fills row, fixed control on the right": do NOT wrap the right control in a
right_to_left child before the field — compute the field width explicitly and lay it first. (Codebase pattern: narrow
egui cells need explicit width; see the egui-aligned-rows lessons.)

**TESTING LESSON (load-bearing — BOTH halves survived passing MDI tests, twice).** `build_dock_harness` renders
`views::dock` into an UNCONSTRAINED root `Ui` (a tall, wide window), so neither the height clip nor the width collapse
reproduced — the widget-level MDI tests (Send/Enter/disabled) all passed while the field was invisible. To catch
layout bugs you MUST render through the REAL host path: `build_docked_panel_harness` (ui_test.rs) wraps `dock()` in
`egui::Panel::bottom("dock").exact_size(dock_height)` exactly like `SkirnirApp::update`, then asserts the `TextInput`
node's `rect()` for BOTH `bottom() <= dock_floor` (height/clip) AND `width() >= 100` (the field is a real box, not a
sliver). Assert every dimension that "visible and usable" depends on — a height-only assertion passed on a zero-width
field. Same family as the user's black-bar `exact_size` panel-overflow lesson.

API notes: `egui::Panel::bottom(..).exact_size(..)` (NOT the deprecated `TopBottomPanel` / `.exact_height` — both
fail `-D warnings`). kittest: `harness.get_by_role(egui::accesskit::Role::TextInput)`, `node.rect()`,
`node.click_accesskit()` (reliable for a button in a nested right_to_left layout where a raw pointer `click()`
misses). Related: [[program-panel-autoscroll]], [[gui-architecture]], [[design-spec]].
