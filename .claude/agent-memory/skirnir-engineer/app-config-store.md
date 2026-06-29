---
name: app-config-store
description: Startup-loaded JSON app config (appearance/themes + UI/connection/toolpath tuning); Palette instance refactor; store.rs; F5 reload
metadata:
  type: project
---

The JSON app config at `~/.config/skirnir/config.json` — the appearance/preferences sibling of `profile.ron`
(machine/rotary state, untouched) and `setting_descriptions.json` (tooltips). Mirrors their never-panic,
versioned, atomic-write contract. See `docs/native-app.md` for skirnir intent; the code is source of truth.

**Modules (all gui-gated except store):**
- `src/store.rs` (NOT gui-gated, no egui) — shared `config_dir() -> Result<PathBuf, StoreError>`
  (`ProjectDirs::from("", "", "skirnir")`) + `atomic_write(path, &[u8])` (temp `.tmp.<pid>` sibling + rename).
  `profile.rs`/`setting_help.rs` still keep their own copies; new stores adopt these.
- `src/config/mod.rs` — `Config` (`#[serde(default)]` on the struct + every section), `CONFIG_VERSION=1`,
  `from_json -> Result<Config,String>` (refuses version > CONFIG_VERSION), `load()/load_from -> (Config, Vec<String>)`
  (NEVER panics; corrupt/bad-hex/too-new → defaults + notice; first-run seeds from bundled asset), `save`/`save_to`,
  `palette()`/`toolpath_style()`. `finish()` folds the soft unknown-theme notice into the load notices.
- `src/config/color.rs` — `ColorSpec` newtype, serde from/to `"#RRGGBB"`/`"#RRGGBBAA"` → `Color32`. GOTCHA:
  `Color32::from_rgba_unmultiplied` stores PREmultiplied alpha, so the `from_color32(to_color32())` round-trip is
  lossy for translucent colours — only opaque round-trips exactly (every palette colour is opaque). `to_color32`
  is NOT const (egui's ctor isn't const). Hex (string) round-trip IS a true fixed point.
- `src/config/theme.rs` — `AppearanceConfig {active_theme, font_scale, themes: BTreeMap<String,ThemeOverride>}`.
  **PRECEDENCE (was a bug, now fixed): `resolve_named` checks `self.themes.get(name)` BEFORE `builtin_palette` — a
  user `themes` entry WINS over a same-named built-in**, so the intuitive `themes.default {accent:"#X"}` recolours
  (the old order short-circuited the built-in and silently ignored it). Base rules for a user entry: implicit base
  (no `base`) → same-name built-in DIRECTLY (else default built-in), NOT back through `self.themes` (no self-recurse);
  explicit `base==name` (self-referential `base:"default"`) → also short-circuited to the same-name built-in; any
  other explicit base → recurse (depth-bounded `MAX_BASE_DEPTH=8`). Unknown name/base → default + notice.
  `builtin_palette` maps "default"/"light-slate"/"midnight". `ThemeOverride` = `Option<ColorSpec>` per field + `base`.
- `src/config/sections.rs` — `UiConfig`/`ConnectionConfig`/`ToolpathConfig` (+ `ReconnectSection` mirroring
  `reconnect::ReconnectConfig`'s real fields: `base/factor:u32/max_delay/max_attempts`). `ToolpathConfig.resolve()`
  → runtime `ToolpathStyle` (arc_step_deg→rad, all knobs floored). Defaults are the EXACT legacy hard-coded values.
- `assets/config.default.json` — bundled via `include_str!`, fully-keyed (every ui/connection/toolpath knob spelled
  out). `appearance.themes` carries a `"default"` template theme (NO `base` key — implicit same-name shadow; an
  explicit `base:"default"` would also work via the self-ref short-circuit but is omitted) with EVERY `ThemeOverride`
  colour field == its `default_dark()` hex. **It is the ACTIVE theme** (`active_theme:"default"`), so editing any
  `themes.default.<color>` in the seeded file recolours IMMEDIATELY with no indirection — yet unedited it resolves to
  exactly `Palette::default_dark()` (a true no-op, no visual change). The asset is NOT `== Config::default()`. Seed
  invariant: seeded-then-read parses with NO notices, round-trips `from_json(BUNDLED_DEFAULT)`, ui/connection/toolpath
  sections == built-in defaults, and the RESOLVED palette == `default_dark()`. `ThemeOverride::all_color_fields_set()`
  + an exhaustive in-code `full_override_from` literal (no `..Default`) are the stale-guards: add a palette field →
  both fail until the template is extended. `shell.rs::new` pushes a `config: <path>` console notice (via
  `config::config_path()`) so the operator sees which file to edit.

**`theme.rs` refactor (the shared type):** `Theme` unit-struct + associated `const`s → instance `Palette` struct
(one field per token, `#[derive(Copy)]`). `Palette::default_dark()` carries today's exact hexes VERBATIM;
`light_slate()`/`midnight()` are alternate built-ins that change ONLY chrome (accents/state/alarm/console inherit via
`..default_dark()` so meaning never shifts). `badge_color`/`axis_color` are now `&self` methods; `pride_at` stays a
`&self`-free assoc fn (pride palette is fixed `PRIDE` const, NOT themeable). `app::Theme` re-export → `app::Palette`.

**View threading (the bulk churn):** `UiState.style: RuntimeStyle { palette: Palette, toolpath: ToolpathStyle }`
(in `views.rs`, `Default` = design dark + legacy constants so tests render identically). ~155 `Theme::FOO` →
`palette.foo` across `views.rs`. Top-level view fns bind `let palette = state.style.palette;` (Copy, avoids `&mut
state` borrow conflicts). Free helpers (`header_bar`/`tab_strip`/`section_header`/`endstop_chip`/`big_axis_value`/
`override_slider`/`override_axis`/`dock_progress`/`console_line_style`/`draw_grid`/`alarm_banner`/
`tool_change_banner`/`settings_tooltip_ui`/etc.) take a `palette: Palette` param (inserted after `ui`/`painter`).
Toolpath render constants → `state.style.toolpath`: `draw_grid` takes `ToolpathStyle`; trail strokes use
`palette.toolpath_cut/rapid` + `tp.{cut,rapid}_stroke_px`; marker uses `tp.marker_radius_px`. `preview::flatten_arc`
gained a `max_step_rad` param (`DEFAULT_ARC_STEP_RAD` = PI/20 fallback); `parse_xy_path` threads it; `set_program`
reads `self.style.toolpath.arc_step_rad`.

**Shell wiring (`shell.rs`):** `SkirnirApp` gained `config: Config` + `config_path_override` (test seam). Config is
loaded ONCE in `run()` and PASSED into `new(runtime, config, notices)` (no double-load — the `config_path_override`
seam lets tests `load_from` a temp file then pass the Config). **F5 reload bug fix:** the old `apply_config_to_ui`
(which reload also called) wiped operator session knobs. Split into `apply_appearance` (palette+toolpath style — both
startup AND reload) and `apply_ui_defaults` (jog/DRO/console knobs — startup ONLY). `reload_config(&ctx)` calls
`apply_appearance` ONLY + `ui.reflow_toolpath()` (re-flattens the loaded program's arcs at the new
`arc_step_deg` — strokes/grid update on paint but cached chord geometry would otherwise stay stale) + re-skins via
`apply_theme`. `new()` seeds baud via `views::sanitize_baud(default_baud)` (pub(crate) — was bypassing the clamp).
`run()` clamps window via `config.ui.window_size()` (per-axis floors: 800w/500h matching `with_min_inner_size`, NOT a
single floor — 720 default height must not clamp up; non-finite→floor, no `f32::clamp` NaN panic). `apply_theme` wires
`accent_hover`→`widgets.hovered.bg_stroke`, `accent_active`→`widgets.active.bg_stroke` (were inert).

**Engine connection wiring (`engine.rs`/`protocol/core.rs`):** `Engine::connect_with(transport, EngineConfig)` (connect
= connect_with(default)). `EngineConfig { idle_poll, rx_window: Option<usize> }`. `idle_poll` clamped 20..200ms drives
the status-poll cadence: `poll_interval_for(run_state, idle)` (run rate = `STATUS_POLL_RUN.min(idle)` so a fast idle
config doesn't slow for a run). `rx_window: Some` → `ProtocolCore::pin_rx_window(bytes)`: pins the FlowWindow, makes
`on_response` IGNORE the advertised `[OPT:...]` buffer, and re-applies across `reset_window` (survives soft-reset/
re-enum). Shell builds EngineConfig from `config.connection.{status_poll_ms, rx_window}` in `connect_inner`.

**Misc cleanups:** `profile.rs::save_to`/`profile_path` now delegate to `store::atomic_write`/`store::config_dir`
(hand-rolled dir+`.ron.tmp` rename deleted; `APP_NAME` is now `#[cfg(test)]`-only). The ~36-field colour list is a
single `palette_color_fields!($callback)` macro in `config/theme.rs` driving `apply_over`, `all_color_fields_set`, and
the test `full_override_from` — add a token in ONE place. `config::save`/`save_to` doc no longer overpromises (no
auto-persist; public store infra only — exit-time whole-file rewrite deliberately NOT added, would clobber operator
formatting).

**Tests:** the egui_kittest proof `the_toolpath_view_paints_the_configured_palette_not_the_baked_in_default` renders
`toolpath` with a sentinel `inset` colour and scans `painted_vertex_colors(harness)` (tessellates
`harness.output().shapes`) — proves views read config not consts. 579 lib tests pass under `cargo test -p skirnir` +
`cargo build --features gui`, both with `RUSTFLAGS="-D warnings"`. Clippy errors seen are pre-existing in
`cnc-kinematics`, not skirnir.
