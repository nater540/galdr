---
name: skirnir-view-data-threading
description: How shell-owned data (profile lists, wizard state) reaches skirnir views — ShellPanelsData borrows, never UiState mirrors
metadata:
  type: project
---

When a skirnir view needs data the shell owns (profile lists, wizard/sweep state), thread it as a borrow through
`views::ShellPanelsData<'a>` (and onward through `dock_tiles::CentralSplit::ui` → `CentralBehavior` for dock tabs) —
do NOT mirror it into `UiState`.

**Why:** `shell::snapshot_prefs` rebuilds `Prefs` from `UiState` on every save; anything mirrored risks two sources
of truth (macros are explicitly `mem::take`-preserved there for this reason). `ShellPanelsData::bare()` gives the
test harness the empty/fixture form so app and harness render the SAME `shell_panels` and cannot drift.

**How to apply:** add the field to `ShellPanelsData` + `bare()`, pass it down the dock/pane chain, and update the
two dock harness builders (`ui_test.rs`) and `snapshot_dock` (`snapshot_test.rs`). Floating windows (editors,
confirm modals) render at ctx level in `shell::ui` where `self.profile`/`self.ui` are directly borrowable
(disjoint-field borrows are fine). Modal pattern: staged state in `UiState` (e.g. `pending_macro_run`), a
`*_window(ctx, ..) -> Option<decision>` view fn, shell acts on the return — and `UiState::on_disconnected` must
clear any staged state whose payload was derived from the dead session.
