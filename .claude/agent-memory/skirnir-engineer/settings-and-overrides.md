---
name: settings-and-overrides
description: skirnir override sliders + fine ±1%, live $-settings sync model, and the auto-reconnect/backoff wiring
metadata:
  type: project
---

Three subsystems layered on the mature engine (pure logic + unit-tested, views thin). See [[gui-architecture]]
for the layer split, [[engine-architecture]] for the reconnect/settings driver notes.

**Override sliders + fine ±1% (`app/overrides.rs`, NOT gui-gated).** `RealtimeCommand` already had the full
override byte set (0x90–0x9E). grbl overrides are RELATIVE (only ±10/±1/reset, no "set to N%"), so a slider's
absolute target becomes a step sequence: `override_commands(current, target, OverrideAxis{Feed,Spindle}) ->
Vec<RealtimeCommand>` — clamps to 10..=200, `reset` short-circuit when target==100 (one byte, self-correcting
vs a drifted `current`), else greedy ±10 decades then ±1 units; empty when current==target. `clamp_override`
guards a stale reported `current` from an unbounded run. NEW `Intent::SetOverride{axis,target}` (carries only
the intent); the SHELL reads live `Ov:` (`view.status.overrides`, default 100) and emits the steps via
`set_override`. Rapid stays preset-only (100/50/25), never routes through stepping. View: `override_axis(...)`
renders a `Slider` (10–200%, commits `SetOverride` on `drag_stopped` only if moved) + a `−10 −1 100 +1 +10`
stepper row. Slider transient state = `UiState.feed_override_drag`/`spindle_override_drag: Option<u32>`
(tracks live while idle, pins during drag, so the status poll can't yank the handle mid-drag).

**Live `$`-settings sync (TEXT path, see [[engine-architecture]] for why not binary `$PBX`).**
`protocol/settings.rs` (pure): `parse_setting_value("$0=10")` (rejects `$N0=`/`$J=`/`$H` — non-numeric key;
strips trailing `(desc)`), `parse_setting_meta("SETTING:id|group|name|unit|datatype|format|min|max")`,
`setting_write_line(n,val)`. NEW `Response::Setting{number,value}` added to `parse_line` (BEFORE the banner
fallback) + the `is_grbl_evidence` arm (accept_acks-gated). `app/settings_model.rs` (pure): `SettingsModel` =
`BTreeMap<u32,SettingRow>` (ascending-`$n` order regardless of arrival), `apply_value`/`apply_meta` merge onto
one row in either order, `SettingRow::label()` falls back to `$<n>` without meta. Reducer (`view_state.rs`):
folds `Response::Setting` → `apply_value` and `[SETTING:...]` messages → `apply_meta`, BOTH skip the console
(a `$$` dump would flood it, like status telemetry); cleared on disconnect. NEW intents `RequestSettings`
(shell sends `$ES` then `$$`) + `WriteSetting{number,value}` (shell writes then re-dumps `$$` — no
single-setting read exists). View `settings_panel`/`settings` now take `&ViewState`; render the live list
(violet `$NNN` keys = `Theme::LOG_STATUS` 0x9B7FE0, enumerated label+unit, editable value) + a `Refresh ($$)`
button (enabled only when connected). Edit transient state = `UiState.editing_setting: Option<(u32,String)>`;
`commit_setting_edit(committed, n, buf, live) -> Option<Intent>` (pure, tested) writes only a real,
non-empty, changed value (Escape/focus-loss-without-Enter = abandon).

Counts after this work: 209 tests pass (`-p skirnir`), 184 headless (`--no-default-features`), clippy clean on
default / gui-only / headless. Style note unchanged: 2-space indent, do NOT run `cargo fmt` (4-space). The
overrides/settings_model/reconnect modules are NOT gui-gated so the headless build still exercises them.
