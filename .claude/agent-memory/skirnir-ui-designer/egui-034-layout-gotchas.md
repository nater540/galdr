---
name: egui-034-layout-gotchas
description: egui 0.34 behaviors that bit during the 2026-07-01 skirnir UI overhaul (RTL, TextEdit sizing, panel resize)
metadata:
  type: reference
---

egui 0.34 behaviors verified the hard way in skirnir (each caused a visible bug first):

- `ui.horizontal()` **preserves a right-to-left parent direction** — content added inside a `with_layout(right_to_left)`
  closure lays out reversed (the badge rendered `LABEL ●`). Fixes that DON'T work: `with_layout(left_to_right)` claims
  ALL remaining width (chip ballooned to a full-width bar); zero-sized `allocate_ui_with_layout` grows PAST the window
  edge in an RTL parent. What works: keep `horizontal` and feed items in direction-matched order (see
  `views::state_badge`).
- `TextEdit`: `.frame(...)` takes a `Frame` (not bool) in 0.34; widget height = `row_height + frame.total_margin()` —
  **`min_size.y` is ignored**. To get a 22px field, use a transparent frame with vertical `inner_margin`.
- Bottom-panel drag-resize: the grab band is only ~±5px around the panel's TOP EDGE; a kittest drag aimed at the tab
  label (~10px inside) grabs nothing. Compute the edge y from layout constants.
- Collapse-vs-resize memory: egui remembers a panel's size per id, so a collapsed `exact_size(30)` panel would clamp a
  later re-expand to the minimum. `views::dock_panel` uses **different panel ids** ("dock" vs "dock-collapsed") so the
  operator's dragged height survives a collapse cycle.
- Glyph coverage: fonts are Roboto + JetBrains Mono + egui's emoji fallback only. `∿` (U+223F) is in none of them →
  tofu box (Simulate now uses `≈`). `⚙`/`⌂`/`⏸` resolve via the emoji fallback.
- A fill-remaining `ScrollArea` (`auto_shrink(false)`) eats ALL height and clips whatever is laid out after it inside
  a pinned panel/window — reserve the trailing row's height via `max_height` first (console MDI, app-settings save row).
- `clippy::assertions_on_constants` fires on const-only `assert!` design-token pin tests; allow it locally with a
  rationale. Also: `cargo fmt --check` FAILS repo-wide pre-existing — the repo is .editorconfig 2-space, NOT
  rustfmt-formatted; do not run rustfmt here.
- kittest pointer clicks land at DOUBLED coordinates under `with_pixels_per_point(2.0)` (node rects are points,
  the synthesized click lands in pixels) — a swatch click misses and popups never open. Interaction-driven
  snapshots must render at 1×; 2× is fine for static shots.
- egui's colour-picker popup IS drivable headlessly: swatches are `Role::ColorWell`, the popup's R/G/B channels
  are `Role::SpinButton`s appended after existing ones; `focus()` + `type_text("255")` + Enter commits a channel.
- `ComboBox::width` is only a MINIMUM, and `.truncate()` bounds to `ui.available_width()` — in an open row the
  button still grows to the full selected text. To actually cap a combo, wrap it in a fixed-size
  `allocate_ui_with_layout` child + `set_max_width`; then `.width` + `.truncate()` behave.
- The toolbar is locale-responsive via the immediate-mode self-measure pattern (`views::ToolbarFit` in temp
  memory): render full, measure LTR end + RTL cluster extent, flip to icon-form next frame when the full labels
  can't fit. Swedish overflows the 800px minimum (and disconnected-sv even 1280) where English fits. The compact
  fit invariant is asserted in `snapshot_toolbar` — a longer future translation fails there, not silently.
- Glyph notes: `🗁` (open folder) renders well in egui's emoji font; `📂` renders as an odd angled shape.
