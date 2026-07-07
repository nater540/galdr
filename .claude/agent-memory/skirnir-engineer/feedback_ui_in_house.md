---
name: feedback-ui-in-house
description: Do all skirnir UI/panel work yourself on Opus — do NOT delegate to the skirnir-ui-designer sub-agent
metadata:
  type: feedback
---

Do ALL skirnir UI/panel work yourself; do NOT spawn the `skirnir-ui-designer` sub-agent.

**Why:** `skirnir-ui-designer` is pinned to `model: fable`, and the team is out of Fable tokens. The datum_finder
panel in Phase 1 was (correctly) built in-house in `views.rs` — no delegation. Team lead confirmed this is the
standing rule going forward (2026-07-05).

**How to apply:** For any egui/eframe panel, widget, layout, theming, or view work in `crates/skirnir` — including
the Phase 5 `mesh_probe()` panel and any panel polish — write it directly (you're on Opus). Mirror the existing
`views.rs` idioms (section_header, full-width buttons, Grids, `crate::tr!` i18n keys added to BOTH en-US.ftl and
sv-SE.ftl per the key-parity test). Only reconsider if the team lead explicitly says Fable tokens are available again.
