---
name: header-bar-itemspacing-leak
description: header_bar/tab_strip reuse one child ui, so the left closure's item_spacing.x=0 leaks into the right closure (dock progress, header controls)
metadata:
  type: feedback
---

`header_bar` (views.rs) builds ONE child `content_ui`, runs `left(&mut content_ui)` then
`content_ui.with_layout(.., right)` on the SAME ui. So any `ui.spacing_mut()` change a left closure makes leaks
into the right closure. `tab_strip`'s left closure sets `item_spacing.x = 0.0` for the tab labels, which leaked
into the dock's `right` content (`dock_progress`) and crammed every field together (`9%0:51` with no separators).

**Why:** this is the same `item_spacing=0` leak class already seen in the transport group ([[skirnir-egui-aligned-rows]]).
egui spacing is per-ui mutable state, and egui's right_to_left right-side content shares the bar's child ui here.

**How to apply:** any closure handed to `header_bar`'s `right` (dock progress, header-strip controls) must set its
OWN `item_spacing.x` explicitly — never assume it inherits a sensible default. `dock_progress` now sets
`Metrics::DOCK_PROGRESS_GAP` at the top for exactly this reason. When adding a new header-strip control, do the same.
Also: kittest `query_by_label` panics when a label appears more than once (e.g. multiple `·` separators) — use
`query_all_by_label(..).count()` for repeated nodes.
