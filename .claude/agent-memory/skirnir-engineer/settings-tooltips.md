---
name: settings-tooltips
description: Per-setting hover tooltips — runtime-loaded JSON descriptions + dynamic $ES meta lines; egui 0.34 ui.close_menu deprecation
metadata:
  type: project
---

The Settings panel rows have rich hover tooltips combining live `$ES` metadata with curated prose.

**Curated descriptions are runtime-loaded JSON, not a Rust table** (explicit user choice).
- Asset: `crates/skirnir/assets/setting_descriptions.json`, format `{ "$<n>": "text" }` (keys WITH leading `$`).
  Axis decades are EXPANDED to explicit per-number entries; $103/$113/$123/$133/$143/$153 say "deg" (rotary A),
  $100-102 stay "mm". Galdr-specific: $19, $376, $392, $393, $481.
- Loader: `crates/skirnir/src/app/setting_help.rs` — `SettingDescriptions` struct wrapping `HashMap<u32,String>`.
  Mirrors `profile.rs`'s pattern exactly: path-taking `load_from(path)` core, `load()` resolves via the SAME
  `directories::ProjectDirs::from("","","skirnir")` config dir (sits beside `profile.ron`). First run SEEDS the
  on-disk file from `include_str!("../../assets/...")`; on-disk file is then AUTHORITATIVE (user edits + restart).
  Any failure → bundled fallback in memory + a single console notice. Never unwrap/panic. `serde_json = "1"` added.
- Loaded once in `SkirnirApp::new` (shell.rs), stored in `UiState::setting_descriptions`; `UiState::default()` uses
  `SettingDescriptions::bundled()` so tests/default UI have working tooltips without disk.

**Two view helpers in views.rs** (both pure-ish, unit-tested):
- `setting_tooltip_meta(row: &SettingRow) -> Vec<String>` — PRIMARY dynamic lines from `$ES`: name, `Unit:`,
  `Range: min..max` (one-sided `≥`/`≤` when only one bound). Empty vec when no meta.
- `settings_tooltip_ui(ui, number, heading_name, meta_lines, descriptions)` — heading `$<n> · name`, meta lines,
  then a separator + curated prose when present. Degrades to bare `$<n>` heading, never an empty box.
- Call site threads `&state.setting_descriptions` + the disambiguated display label into `.on_hover_ui`.
- Panel-level note (views.rs ~line 1986) already flags "$22 homing applies on next reset".

**egui 0.34.3 gotcha:** `ui.close_menu()` is DEPRECATED (fails under `-D warnings`); use `ui.close()` (or
`ui.close_kind(UiKind::Menu)`). The console-Clear `context_menu` hit this. See [[gui-architecture]].
