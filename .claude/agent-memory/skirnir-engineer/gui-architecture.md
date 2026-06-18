---
name: gui-architecture
description: skirnir egui/eframe GUI layer — pure reducer vs thin views, intent flow, eframe 0.34 API quirks, dep pins
metadata:
  type: project
---

The GUI lives in `src/app/` and drives the streaming engine (see [[engine-architecture]]) over channels. Built
on **eframe/egui 0.34.3** (glow backend, no wgpu/accesskit) + **rfd 0.17.2**. egui is consumed via the
`eframe::egui` re-export — there is NO direct `egui` dep. The `gui` feature is default-on (`default=["gui",
"serial"]`); `gui=["dep:eframe","dep:rfd"]`.

**Layer split (thin views, testable logic):**
- `app/view_state.rs` — pure, egui-free reducer. `ViewState::apply(Event)` folds engine events into render
  state: lifecycle, latest `StatusReport`, **WCO cached across reports** to derive the missing MPos/WPos via
  `WPos=MPos-WCO`, capped console (`VecDeque`, `CONSOLE_CAPACITY=2000`), progress, latched alarm/error
  `Banner`. Status reports do NOT hit the console (telemetry flood). Unit-tested without a window.
- `app/intent.rs` — egui-free `Intent` enum (Connect/Disconnect/Jog/ProbeZ/SendLine/Realtime/...) + `IntentSink`
  (per-frame `Vec`, drained by the shell). Views speak intents; the shell owns the policy (open port, read
  file, form `$J=`/probe lines).
- `app/badge.rs` (NOT gui-gated — egui-free, headless-testable) — pure presentation logic: `BadgeState`
  (richer than `ConnectionState`: Jog/Home/Door/Check/Sleep), `BadgeState::derive(connection, run_state)`
  (host-latched Alarm/Error win; else firmware `RunState` wins; else lifecycle fallback), `TransportGroup`
  (Run/Hold/Stop enable+emphasis matrix), and `alarm_detail`/`error_detail` code→gloss tables. `ViewState`
  exposes `badge_state()`. The theme turns a `BadgeState` into a colour; this decides *which* state it is.
- `app/theme.rs` (gui-gated) — `Theme` exact design tokens (see [[design-spec]]): chrome/accent/text/state-dot/
  alarm-trio/console-type colours via a `const fn rgb(0xRRGGBB)`. `badge_color(BadgeState)` + `axis_color(Axis)`
  (X green / Y blue / Z amber). `state_color`/`state_label` were REPLACED by `badge_color` + `BadgeState::label`.
  `apply_theme` (in shell) maps tokens onto `Visuals.widgets.{noninteractive,inactive,hovered,active,open}` +
  panel/window/extreme/faint fills + 2px `CornerRadius` + 1px divider strokes.
- `app/views.rs` (gui-gated) — one render fn per panel (toolbar/dro/jog/overrides/probe/program_dock/console/
  status_bar/alarm_banner/toolpath/settings). Each takes `&ViewState` + `&mut UiState` + `&mut IntentSink`,
  pure render, no engine/IO. `UiState` = transient widget state (port sel, jog step, console input, probe
  params). `toolpath` = `egui::Painter` 2D XY preview; `parse_xy_path` is a cheap linear G0/G1/G2/G3 parser
  (arcs drawn as chords) that tracks the modal motion AND distance mode (G90/G91), tokenizes via `gcode_words`
  (handles compact spaceless `G1X10Y5`), and SUPPRESSES segments for non-motion G-codes (G10/G28/G30/G92/G53/
  G4). The parse is cached at load: `UiState::set_program(lines, path)` stores `program: Arc<[String]>` and
  rebuilds `toolpath: Vec<Segment>` + `toolpath_bounds` ONCE; the `toolpath` view only computes per-frame
  fit/scale. Jog enable = `jog_enabled(BadgeState)` = Idle|Jog only (grblHAL rejects `$J=` elsewhere). All
  unit-tested.
- `app/shell.rs` (gui-gated) — `SkirnirApp` impls `eframe::App`. Owns the `tokio::runtime::Runtime` (engine
  task lives there; `Engine::connect` called via `runtime.block_on`), the `EngineHandle`, `ViewState`,
  `UiState`. Each frame: `pump_events()` drains `handle.try_recv()` (NEVER `.await` on UI thread), lay out
  panels, act on drained intents. **Repaint is GATED:** `request_repaint_after(50ms)` only when `pump_events()`
  saw an event OR an engine is attached (`engine.is_some()`); when disconnected the UI sleeps (no 20Hz spin).
  Engine events are polled from this loop, so the steady timer while connected is what drains them. `runtime`
  field + the `Engine` import are `#[cfg(feature="serial")]`-gated (gui-only build never opens a port).

**Panel frame margins (bit me — black gutter + clipped left column):** egui panels carry DEFAULT inner
margins that double-pad against the views' own padding. `CentralPanel::default()` insets content 8px on all
sides (→ a black gutter between the left column and the toolpath viewport); `Panel::left/right`
(`side_top_panel`) inset `Margin::symmetric(8, 2)` (→ usable column shrinks 268→252px, so the DRO "Zero XYZ"
button + the Z± jog column overran the clip and were cut off). FIX in shell.rs: pass an explicit zero-margin
frame — side cols get `Panel::*().frame(egui::Frame::NONE.fill(Theme::PANEL))`, central gets
`CentralPanel::default().frame(egui::Frame::NONE.fill(Theme::INSET))`. The design grid `268|1fr|286` has NO
gutter; the section headers + DRO_PAD/JOG_PAD/RIGHT_PAD own all the internal padding, so panel frames must be
ZERO. `Frame::none()` is deprecated → use `Frame::NONE`. The `ui` method imports `super::theme::Theme` for these.

**Small icon buttons in the 30px strip (bit me — oversized off-centre collapse button):** `add_sized(square)`
does NOT cap a button — the global `BUTTON_PAD` (6px vertical, set in apply_theme) plus the glyph inflates the
box past the strip's control height, so it overflows the 30px bar and reads off-centre. FIX: zero the region's
`button_padding` (`ui.spacing_mut().button_padding = Vec2::ZERO`) before `add_sized(Vec2::splat(PANEL_CONTROL_H))`
and hold the glyph at `HEADER_TEXT` (not +2) so it can't grow the box; the strip's `Align::Center` then centres
the 22px square in the 30px bar. Applies to `dock_collapse_toggle` and any future strip icon button.

**eframe/egui 0.34 API quirks (bit me, will bite again):**
- `eframe::App` requires `fn ui(&mut self, ui: &mut egui::Ui, frame)` — `update(ctx,...)` is DEPRECATED.
  Panels go into the root `Ui` via `show_inside(ui, ...)`, not `show(ctx, ...)`. Reach the ctx via
  `ui.ctx().clone()` (needed for `request_repaint_after` and `Window::show`).
- Panels unified: `egui::Panel::top/bottom/left/right(id)` — `TopBottomPanel`/`SidePanel` are deprecated
  aliases. `default_width`/`default_height` → `default_size(f32)`.
- `serialport`/`tokio-serial` port enumeration: added `transport::serial::available_ports() -> Vec<String>`.

**rfd file dialog** is called synchronously in the toolbar view (`FileDialog::new().pick_file()`); fine for a
desktop tool. Needs an XDG portal daemon at runtime on Linux (handle the None return — it's already optional).

**Layout metrics: DONE (gui-gated `app/metrics.rs`).** All spec'd px dims live as named `Metrics::*` consts
(lifted verbatim from `Skirnir.dc.html` §02 component sheet + §03 full-window mock) so views size against the
design, not egui defaults — toolbar 40 / header strip 30 / status 24, control heights (toolbar 26, panel 22),
`BUTTON_PAD` 14×6 (egui x,y order), jog cell 32 / gap 4, badge pad 10×5 / dot 8, progress 260×6, body cols
268/286, dock 200. 5 unit tests assert the load-bearing values. `apply_theme` now also sets global `Spacing`
(button_padding, item_spacing 6, interact_size.y=22) via `ctx.global_style()`/`set_global_style` (NOT the
deprecated `style`/`set_style`). Panel bars are pinned with `.exact_size(..)` (`exact_height` is deprecated →
`exact_size` for top/bottom too). The toolbar view sets its own region spacing (control h 26, pad 10×0, gap 6).

**Header strip is a real 30px bar (the user-flagged fix).** `views::header_bar(ui, left, right)` allocates a
fixed `HEADER_H` `panelAlt` rect, paints a 1px bottom divider, then lays content in a `new_child` over the
horizontally-shrunk (14px pad) rect: `left` left-to-right, `right` pulled right via `with_layout`. Don't go
back to the old margin-based `Frame` header (it didn't pin height or draw the divider). `section_header` wraps
it with the uppercase tracked title (`extra_letter_spacing(11*0.1)`). `tab_strip(ui, &[(label,active)], right)`
is the dock-tab variant: 11.5px mixed-case tabs, 2px ACCENT underline pinned to the strip floor under the
active tab. Console/Program docks use `tab_strip`; the Console strip carries the §03 progress bar (260×6) +
auto-scroll on the right. Transport group is a joined segment trio (per-corner `corner_radius`, item_spacing 0).

**Body layout** is a fixed grid `268px | 1fr | 286px` (left/right panels `.resizable(false).exact_size(..)`,
NOTE: `exact_width`→`exact_size` in 0.34). Per §03 the column contents are: LEFT (268) = DRO + Jog only; CENTER
(1fr) = toolpath viewport; RIGHT (286) = Overrides + Probe + Settings (inline `views::settings_panel`, a
deferred `$NNN` placeholder — the live `$PBX` list is not wired; its ⚙ button opens the existing baud popup
`Window`). The **bottom dock (200px) spans the FULL window width under all 3 columns** and hosts Console +
Program as TWO TABS in one surface (`views::dock`): it is declared in shell.rs `Panel::bottom("dock")` BEFORE
the left/right side panels so it claims full width (columns rise only above it); the 24px status bar is declared
even earlier so it stays below the dock. `UiState.active_tab: DockTab{Console(default),Program}` drives which
body renders; the shared `tab_strip` now RETURNS `Option<usize>` (clicked tab idx — labels are `interact(click)`
sensed) and `dock_tab_for_click(current, clicked)` (pure, tested) maps it to a `DockTab`. The §03 progress
(acked/total · 260px green bar · pct) rides the strip's right edge for BOTH tabs (dock-level, `dock_progress`);
auto-scroll checkbox moved INTO the console body. Program is NO LONGER a right panel. **Dock is COLLAPSIBLE:**
`UiState.dock_collapsed` (default false) + a ghost `−`/`+` icon button at the strip's outermost-right (progress
to its left, since `tab_strip`'s `right` closure lays out right-to-left → draw toggle first); collapsed → `dock`
view early-returns after the strip (body hidden). The shell pins the panel with
`.resizable(false).exact_size(Metrics::dock_height(collapsed))` (200px expanded / `HEADER_H`=30 collapsed). **GOTCHA
that caused "dock fills the whole window":** a `resizable(true).default_size(DOCK_H)` bottom panel whose body uses a
fill-remaining `ScrollArea` (`auto_shrink([false,false])`) hits a height-feedback loop and resolves to most of the
window on first layout — pin with `exact_size`, never `resizable+default_size`, for content that fills. Pure helpers
`Metrics::dock_height(bool)` + `views::dock_toggle_label(bool)` are unit-tested. Toolbar has a segmented
Run/Hold/Stop group (Run=`RunOrResume`
intent → cycle-start if held else stream; Hold=`!`; Stop=`0x18`) + Home (`$H`) + the spec badge. New intents:
`Home`, `RunOrResume`, `SetWorkZero { axes }` (Vec empty = all XYZ). `work_offset_line(&[(Axis,f64)])` is the
single `G10 L20 P0 <axis><value>` builder (L20 = set-relative-to-current); `work_zero_line(&[Axis])` delegates
to it with 0.0 per axis (emits `X0.000` etc.) and `probe_z` uses it for the Z-zeroing line. Both pure+tested.
Alarm banner is an inline strip (not modal) with Unlock-$X/Soft-reset/Dismiss.

**Fonts: DONE.** Vendored static-weight TTFs under `crates/skirnir/assets/fonts/{roboto,jetbrains-mono}/`
(Regular/Medium/Bold each — design uses Roboto 400/500/700, JBM 400/500). Roboto from googlefonts/roboto-2
v2.136 hinted release; JetBrains Mono from JetBrains/JetBrainsMono v2.304. **Both are OFL-1.1** — Roboto was
relicensed from Apache-2.0 to OFL-1.1 in 2024, so the old "Apache for Roboto" note was STALE; each dir has
`LICENSE.txt` (OFL) + a root `NOTICE.md`. Wiring lives in gui-gated `app/fonts.rs`: `definitions()` builds a
`FontDefinitions` (pure, egui-only, no I/O) starting from `::default()` (keeps egui emoji/fallback), inserts
all six faces via `FontData::from_static(include_bytes!(..))`, and prepends Roboto Regular → `Proportional`
head + JBM Regular → `Monospace` head (opposite face appended as glyph fallback). `install(ctx)` =
`ctx.set_fonts(definitions())`, called in the eframe creation closure in shell.rs BEFORE `apply_theme`. egui
does NOT pick weight within a family (renders the first font); Medium/Bold registered under named keys
(`roboto-medium`, etc.) for a deliberate `FontFamily::Name` pick. **No view changes needed**: DRO digits +
console + status strip already used `.monospace()` (→ Monospace → JBM tabular); UI text uses Proportional →
Roboto. 3 unit tests in fonts.rs (sfnt magic guard, key↔data wiring, family-head mapping). Zero Cargo.toml
change. Boots clean (set_fonts parses the bytes on first frame; no panic). `--no-default-features` headless
build still compiles (font module is gui-gated, no bytes pulled).
