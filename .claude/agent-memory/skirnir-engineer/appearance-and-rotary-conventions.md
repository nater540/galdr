---
name: appearance-and-rotary-conventions
description: skirnir conventions from the 2026-07-02 code review — color-picker alpha, font-scale range, rotary A gating, appearance-no-reflow
metadata:
  type: project
---

Four conventions established fixing a code review on `feat/skirnir-ui-polish` (2026-07-02). Honor these to avoid
reintroducing the exact defects.

**Why:** an adversarial review confirmed each as a real bug; these are the load-bearing invariants of the fixes.

**How to apply:**
- **Color pickers must edit UNMULTIPLIED sRGBA.** Use `ui.color_edit_button_srgba_unmultiplied(&mut [u8;4])` with
  `ColorSpec::to_srgba_unmultiplied` / `from_srgba_unmultiplied` (config/color.rs). NEVER the premultiplied
  `color_edit_button_srgba` + `ColorSpec::from_color32` — `Color32` stores premultiplied channels, so capturing its
  raw channels and re-expanding via `from_rgba_unmultiplied` double-applies alpha and decays any alpha<255 colour
  toward black on every frame/reload. `from_color32` is fine only for OPAQUE colours (e.g. `from_palette`).
- **Font-scale bounds live in ONE place:** `config::theme::FONT_SCALE_RANGE` (0.5..=2.5) + `clamp_font_scale`,
  re-exported from `crate::config`. The settings slider AND every clamp (SetFontScale handler, `apply_theme` zoom)
  derive from it. A narrower slider than the accepted range let egui's always-clamp silently rewrite a valid
  hand-edited scale (e.g. 2.4→2.0) the moment the dialog opened, marking the config unsaved.
- **Rotary A controls gate on `ViewState::reported_axis_count()` / `has_rotary_axis()`** (the report's position-vec
  width; 4 = rotary). BOTH the DRO A row (views.rs `dro`) and the jog pad A column (`jog`) use it — a 3-axis board
  must never render an A jog, else a click sends `$J=...A...`, the firmware answers `error:N`, and the stream wedges
  in the error state. Keyboard jog only covers X/Y/Z, so no gate needed there. When A is hidden the Z jog column
  takes the full fill width (no empty gap).
- **`SkirnirApp::mark_appearance_changed` must NOT call `reflow_toolpath`.** Appearance edits (theme colours, active
  theme, font scale) never change geometry; arc density (`toolpath.arc_step_deg`) is not an appearance field and
  changes only via config reload (F5 → `reload_config`, which reflows there). Reflowing on the appearance path
  re-parsed the whole program (`parse_xy_path` over every line) on every colour-picker drag frame.

See [[i18n-fluent]] for the related i18n test-race guard from the same review.
