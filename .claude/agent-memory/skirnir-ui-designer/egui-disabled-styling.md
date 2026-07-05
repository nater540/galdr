---
name: egui-disabled-styling
description: Explicit RichText colors defeat egui's automatic disabled fade — hand-style disabled states (found via macros-tab snapshot)
metadata:
  type: project
---

In skirnir's egui 0.35 views, a `Button::new(RichText::new(..).color(..))` passed through `ui.add_enabled(false, ..)`
renders IDENTICALLY to its enabled form — the explicit text color overrides the style-driven disabled dimming, and a
custom `.fill()` overrides the disabled fill. The macros-tab streaming snapshot caught buttons that were gated but
looked live (a safety-legibility failure for machine controls).

**Why:** egui derives disabled visuals from the style's widget visuals; explicit per-widget colors bypass that path.

**How to apply:** any widget with hand-picked text/fill colors must branch on its own enabled flag — e.g.
`let (text, fill) = if enabled { (palette.text, palette.widget) } else { (palette.text_disabled, palette.inset) };`
(see `app/macro_editor.rs::macros_body`). Always snapshot the disabled state, not just the enabled one.
