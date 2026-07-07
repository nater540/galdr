---
name: setup-dialogs
description: Skirnir right column declutter — overrides stay inline, probing wizards moved to launched dialog windows
metadata:
  type: project
---

The right column (`Panel::right("rightcol")` in `views::shell_panels`) was a six-section vertical scroll (overrides +
probe + rotary-center + datum + mesh + verify). Decluttered 2026-07-07: only **overrides** stay inline (the thing
watched/adjusted during a running job); the five setup/probing panels moved into **floating dialog windows** launched
from a compact `setup_menu` (header `hdr-setup` + five full-width buttons).

**Shape (all in `views.rs`):**
- `enum SetupDialog { Probe, RotaryCenter, Datum, MeshProbe, VerifyMeasure }` + `UiState::setup_dialog: Option<_>`
  (only one open at a time).
- `setup_menu` renders the launcher buttons; a running flow's button shows an accent-green `· running` text tag
  (state never colour-alone) via `SetupRunning { rotary, datum, mesh, sweep }` booleans threaded from the shell's
  `Option` borrows. The open dialog's button uses `Button::selected(true)`.
- `setup_dialog_windows(ctx, view, state, &data, sink)` draws whichever dialog is open, hosting the existing
  `probe`/`rotary_center`/`datum_finder`/`mesh_probe`/`verify_measure` fns as the window BODY. Those fns had their
  internal `section_header(..)` REMOVED — the window title (an `hdr-*` key) is the heading now.
- **Header chrome (fixed 2026-07-07):** the window uses `.title_bar(false)` + a custom `Frame` (fill `panel`,
  1px `divider` stroke, `window_shadow`, `inner_margin(0)`, squared) so egui's default mixed-case/bright title
  bar is gone. The heading is `setup_dialog_header(ui, palette, &title, closable) -> bool` — a `header_bar` in the
  app's uppercase/tracked/dim section-header style (via `header_title`), with a ghost `×` (`Button` + transparent
  fill, `tip-close-dialog`) at the right ONLY when `closable`. Returns true when × clicked → caller clears
  `setup_dialog`. Dropping the title bar makes egui 0.35 fall back to `WindowDrag::Anywhere`, so the dialog stays
  draggable without egui chrome. `inner_margin(0)` lets the header strip + each body's own `RIGHT_PAD` reach the
  edges exactly like the inline panels did. `ctx.global_style()` (NOT `ctx.style()` — removed in 0.35) for the
  shadow.
- Pure helpers (unit-tested in views.rs `mod tests`): `forced_setup_dialog(rotary,datum,mesh,sweep)` and
  `toggle_setup_dialog(current, clicked)`.

**Safety rule:** a running probing wizard FORCES its dialog open (`forced_setup_dialog` sets `state.setup_dialog`
each frame in `shell_panels`) and its window is drawn WITHOUT `.open()` — no `X`, can't be dismissed mid-run; only
the in-body Cancel ends it. Idle dialogs get a normal close `X` that clears `setup_dialog`.

**Testability:** windows are drawn inside `shell_panels` (via `ui.ctx()`), so the whole-window kittest harness
drives them too. `ui_test::HarnessState` gained a `wizard: Option<WizardState>` fixture field; `shell_layout` builds
`ShellPanelsData` borrowing it (disjoint field borrow — `&mut state.ui` + `&state.wizard` coexist). Snapshots:
`shell_probe_dialog` (opened, centred, styled header + ghost ×), `shell_rotary_running` (forced-open, styled
header, NO ×), `shell_mesh_dialog` (Heightmap, the user's screenshotted case), `shell_probe_dialog_light_slate`
(header contrast in the lightest palette). Interaction tests in `ui_test.rs`:
`the_setup_menu_launches_a_probing_dialog_instead_of_inlining_it`, `a_running_probing_wizard_forces_its_dialog_open`,
`an_idle_setup_dialogs_title_bar_matches_the_app_section_header_style` (uppercase title present + × closes),
`a_running_setup_dialogs_title_bar_is_styled_but_has_no_close_button` (uppercase title present + no ×). Note: the
3 shipped palettes (`default_dark`/`light_slate`/`midnight`) are all DARK-family — Skirnir has no white/light theme.

Related: [[skirnir-view-data-threading]], [[egui-034-layout-gotchas]], [[snapshot-harness]]
