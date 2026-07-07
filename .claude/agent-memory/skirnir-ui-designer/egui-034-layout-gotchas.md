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
- Two resize-fight mechanisms (2026-07-02, both user-reported): (1) a RESIZABLE panel stores its CONTENT's
  measured rect as next frame's size, so fractional content (mono row 15.125px) that spills past the panel makes
  it creep 0.125px/frame; fix = anchor trailing rows in an inner `exact_size` bottom panel with ≥1px headroom —
  the panel machinery's `set_min_height(exact − margins)` then pins the measured rect to the panel edges exactly.
  (2) egui snaps a resizable WINDOW's height back to content height, so short non-filling dialog content makes
  vertical drags revert instantly; fix = content that always fills (bottom-anchored save row + fill scroll).
  Both pinned by `*_holds_*`/`*_accepts_a_vertical_resize*` stability tests (drift asserts over 20 frames).
- The dock self-resize bug had a SECOND act (2026-07-02): the exact-fill fix held under kittest's default fonts
  but broke under the app's real fonts/theme (row heights differ; content spilled ~2px/frame; in the shipped app
  repaints are pointer-driven → "grows only while moving the mouse"). Two durable lessons: (1) any test about
  measured sizes MUST run under `fonts::install` + `apply_theme` — default-font harnesses wave these bugs
  through; (2) the CLASS fix is the persisted-size guard in `views::dock_panel`: after `show_inside`, write the
  pre-frame height back into `PanelState` (via `insert_persisted`) unless `read_response(id.with("__resize"))`
  reports a genuine drag — content measurement can then never change a resizable panel's size.
- **egui 0.35 notes (upgrade done 2026-07-02, single API break: `Panel::show_inside` → `show`).** Re-verified:
  TextEdit height math, ComboBox width-as-minimum, SliderClamping::Always default, Window title-derived ids (our
  explicit `.id()`s still needed) — all unchanged. **0.35 FIXED the panel-persistence bug upstream**: PanelState
  now stores the chosen `outer_rect` (not the content rect) and skips storing mid-drag — the pre-tiles dock bug
  class no longer exists in 0.35 panels (calculus changes if panels are ever wanted back; tiles kept per user
  decision). NEW 0.35 hazard: a side panel whose CONTENT measures wider than the panel re-anchors its measured
  rect on the overflowed edge and shifts the central-region cursor over the column (0.34 clipped silently).
  Found: the override stepper row (5 free-flow buttons ≈283px) and sv "Snabbmatning 100%" row overflowed the
  286px column; plus sub-2px fractional font overflows. Fix = `views::contained()` (detached child ui pinned to
  the panel rect — child overflow can never expand a parent) around both column bodies, width-splitting
  `button_row` for the steppers, truncating right-aligned rapid row, and the
  `the_side_columns_never_overflow_their_fixed_widths_in_any_locale` guard test (en+sv, PanelState-asserted).
  Rename memo: this file's title says 0.34; entries above predate the 0.35 bump but were all re-verified on it.
- **egui 0.35 floating-window API facts** (verified 2026-07-07 building the setup dialogs): the viewport rect for
  centring a `Window` default_pos is `ctx.content_rect()` — NOT `ctx.screen_rect()` (removed) nor `InputState`'s
  `screen_rect` (private in 0.35). `Window::vscroll(true)` + `.max_width(W)` + `.default_width(W)` gives a
  fixed-width, vertically-scrolling dialog that can't overflow the viewport height. Centre-on-open without pinning:
  `.pivot(Align2::CENTER_CENTER).default_pos(ctx.content_rect().center())` (still draggable; egui remembers against
  the explicit `.id()`). `egui::Button::new(..).selected(bool)` exists and gives the accent "active" fill — used
  for the setup-menu launcher that's currently open.
