---
name: settings-save-model
description: Explicit Save model for the firmware settings dialog — local staging store, Save/Refresh/Close confirm, fixes the old Enter-only silent-drop
metadata:
  type: project
---

The settings dialog uses an explicit-Save model, not per-field immediate writes.

**Why:** the old `commit_setting_edit` only wrote on Enter (`enter && !abandoned`); a value typed then
focus-lost (clicking Refresh, etc.) was silently dropped before reaching the firmware. The Save model stages on
ANY edit (Enter or focus-loss), so a typed value is never lost by construction.

**How to apply:**
- Staging store: `crates/skirnir/src/app/settings_staging.rs` — `SettingsStaging` (pure, no egui, headless-tested,
  re-exported from `app/mod.rs`). `BTreeMap<u32,String>` so flushes are ascending `$<n>`. `stage()` drops a value
  equal to live (no phantom dirty / no-op write). `write_lines()` builds via `protocol::setting_write_line`.
- `UiState` (in `app/views.rs`) gained `settings_staging: SettingsStaging` and
  `pending_settings_action: Option<PendingSettingsAction>` (Refresh/Close); `on_disconnected` clears both.
  `editing_setting` is still the transient text buffer; leaving the field stages instead of writing.
- Save handler: `SkirnirApp::save_settings` (`app/shell.rs`) — flushes each `$N=V` via `send_line` (respects flow
  control), then `$$`, then clears staging. Wired through `Intent::SaveSettings` (in `app/intent.rs`).
- Refresh with staged edits parks `PendingSettingsAction::Refresh`; close-with-dirty (window X) parks `Close`.
  The `views::settings_discard_confirm` modal resolves them; shell carries out the action on confirmed Discard.
- Dirty rows show `Theme::ACCENT_MOTION` (orange) key/value + a `•` label prefix, and display the staged value.
- Pure helpers (tested): `setting_edit_should_stage(abandoned)`, `settings_action_needs_confirm(staging)`.
- This SUPERSEDES the removed `commit_setting_edit` + `WriteSetting`-on-Enter path. `Intent::WriteSetting` still
  exists for single-setting writes but the dialog no longer emits it directly.

See [[gui-architecture]] for the reducer/views/intents split this builds on.
