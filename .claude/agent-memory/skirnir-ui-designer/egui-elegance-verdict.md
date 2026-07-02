---
name: egui-elegance-verdict
description: Decision (2026-07-01) to NOT adopt the egui-elegance widget crate — rationale, so it isn't re-litigated
metadata:
  type: project
---

Evaluated `egui-elegance` (crates.io, v0.13.0 June 2026, egui 0.34, MIT/Apache-2.0, ~60 widgets + 4 paired themes,
~700 downloads) for skirnir and PASSED on adopting it.

**Why:** it is an *opinionated* design system with its own theme/typography/glyph-font; skirnir already has a
complete bespoke token system (`app/theme.rs` Palette + `metrics.rs`, mapped 1:1 to Skirnir.dc.html) with runtime
user themes in `config/theme.rs`. Mixing would create two visual languages, and mapping skirnir's ~36 tokens onto
elegance's theme would cost more than the widgets save. Its overlapping widgets (segmented control, badges, sliders,
tabs) are already implemented in-house and design-locked; the color picker need is covered by egui's built-in
`color_edit_button_srgba`. Young/low-adoption crate for a safety-critical control UI was a secondary concern.

**How to apply:** if a future need matches an elegance widget skirnir lacks entirely (e.g. Toast notifications,
SortableList), re-evaluate importing just that piece rather than hand-rolling — the pass was about theming coherence,
not quality.
