//! GPU image-snapshot tests: offscreen wgpu renders of the FULL application shell, diffed against committed PNG
//! baselines under `crates/skirnir/tests/snapshots/`. These are the visual regression net the interaction tests
//! cannot provide — a spacing tweak that silently breaks a layout shows up here as a pixel diff.
//!
//! Every test is `#[ignore]`d so a plain `cargo test -p skirnir` (and GPU-less CI) never needs a GPU. Run them
//! explicitly on a machine with one:
//!
//! ```sh
//! cargo test -p skirnir -- --ignored snapshot          # diff against the committed baselines
//! UPDATE_SNAPSHOTS=1 cargo test -p skirnir -- --ignored snapshot   # regenerate the baselines
//! ```
//!
//! The rendered states use OBVIOUSLY FAKE fixture data (fixture ports, a tiny fixture G-code square) — never a
//! real job file. Each state is rendered through [`super::ui_test::shell_layout`], the shared mirror of the real
//! shell's panel arrangement, with the app's fonts and theme applied so the pixels match the live window.

use eframe::egui;

use super::progress::TimeEstimate;
use super::ui_test::{HarnessState, build_shell_harness};
use super::view_state::ViewState;
use super::views::UiState;
use crate::engine::Event;
use crate::protocol::{ConnectionState, Response};
use crate::transport::ports::PortInfo;

/// The default window size the snapshots render at — the config's default window, a typical laptop fit.
const DEFAULT_SIZE: egui::Vec2 = egui::vec2(1280.0, 800.0);

/// A tiny, obviously-fake fixture program: a 20 mm square at Z−0.5 with a lead-in, small enough to eyeball in
/// the toolpath viewport and the Program tab. Never a real job file.
fn fixture_program() -> Vec<String> {
  vec![
    "; FIXTURE square 20x20 (snapshot test data)".to_string(),
    "G21 G90".to_string(),
    "G0 X0 Y0 Z5".to_string(),
    "G1 Z-0.5 F120".to_string(),
    "G1 X20 F300".to_string(),
    "G1 Y20".to_string(),
    "G1 X0".to_string(),
    "G1 Y0".to_string(),
    "G0 Z5".to_string(),
  ]
}

/// A [`UiState`] pre-filled with fixture ports and the fixture program, so the toolbar/dock/viewport all have
/// something real to draw.
fn fixture_ui() -> UiState {
  let mut ui = UiState::default();
  ui.ports = vec![PortInfo::bare("/dev/cu.usbmodemFAKE1"), PortInfo::bare("/dev/cu.usbserial-TEST2")];
  ui.selected_port = "/dev/cu.usbmodemFAKE1".to_string();
  ui.set_program(fixture_program(), Some("/tmp/fixture-square.gcode".to_string()));
  ui
}

/// Feed a raw status-report body (sans `<...>`) through the reducer, exactly as the engine would.
fn apply_status(view: &mut ViewState, body: &str) {
  view.apply(Event::Response(Response::Status(body.to_string())));
}

/// A connected, idle view with a fresh status report and a few console lines, as right after a connect + `$$`.
fn idle_view() -> ViewState {
  let mut view = ViewState::default();
  view.connection = ConnectionState::Idle;
  view.note_sent("connect /dev/cu.usbmodemFAKE1 @ 115200".to_string());
  view.apply(Event::Response(Response::Banner("GrblHAL 1.1f ['$' for help]".to_string())));
  view.apply(Event::Response(Response::Ok));
  // Carry a `WCO:` so the DRO can derive the (default-shown) work position — without one it renders `—` dashes.
  apply_status(&mut view, "Idle|MPos:12.500,20.000,-1.200|WCO:2.000,3.000,1.000|FS:0,0|Ov:100,100,100");
  view
}

/// A mid-stream view: machine in Run, live overrides, progress over the fixture program.
fn streaming_view(total: usize) -> ViewState {
  let mut view = ViewState::default();
  view.connection = ConnectionState::Streaming;
  view.progress = super::view_state::Progress { sent: 6, acked: 5, total };
  apply_status(&mut view, "Run|WPos:14.250,20.000,-0.500|WCO:2.000,3.000,1.000|FS:300,10000|Ov:100,100,100|Bf:15,128|Ln:6");
  view
}

/// An alarmed view: a latched `ALARM:1` banner over an Alarm status with asserted X/Z limit pins.
fn alarm_view() -> ViewState {
  let mut view = ViewState::default();
  view.connection = ConnectionState::Alarm;
  view.apply(Event::Response(Response::Alarm(1)));
  apply_status(&mut view, "Alarm|MPos:0.000,0.000,0.000|Pn:XZ");
  view
}

/// Render one shell state offscreen and snapshot it. `name` becomes `tests/snapshots/<name>.png`.
fn snapshot_shell(name: &str, size: egui::Vec2, view: ViewState, ui: UiState, time: TimeEstimate) {
  let state = HarnessState::new(view, ui);
  let mut harness = build_shell_harness(state, size, time);
  // Two settle frames: the first lays out with the freshly-installed fonts/theme, the second is steady-state.
  harness.run_steps(2);
  harness.snapshot(name);
}

/// No timing fixture: the zero estimate (nothing streamed, nothing projected).
fn no_time() -> TimeEstimate {
  TimeEstimate::default()
}

/// Render JUST the 40px toolbar at 2× pixel density and snapshot it — the close-up for glyph/spacing review,
/// where the full-window shots are too coarse to judge padding and icon rendering.
fn snapshot_toolbar(name: &str, width: f32, view: ViewState, ui: UiState) {
  use super::intent::IntentSink;
  use super::metrics::Metrics;
  let _ = crate::i18n::init();
  let palette = ui.style.palette;
  let state = HarnessState::new(view, ui);
  // kittest hosts the closure inside a default `CentralPanel` whose frame insets content by 8px on every side,
  // so the window must be 16px taller than the bar or the bottom of the buttons is clipped out of the shot.
  let mut harness = egui_kittest::Harness::builder()
    .with_size(egui::vec2(width, Metrics::TOOLBAR_H + 16.0))
    .with_pixels_per_point(2.0)
    .build_ui_state(
      |ui, state: &mut HarnessState| {
        let palette = state.ui.style.palette;
        let mut sink = IntentSink::new();
        egui::Panel::top("toolbar")
          .exact_size(Metrics::TOOLBAR_H)
          .frame(egui::Frame::NONE.fill(palette.panel_alt))
          .show_inside(ui, |ui| super::views::toolbar(ui, &state.view, &mut state.ui, &mut sink));
        state.intents.extend(sink.drain());
      },
      state,
    );
  super::fonts::install(&harness.ctx);
  super::shell::apply_theme(&harness.ctx, &palette, 1.0);
  harness.run_steps(2);
  harness.snapshot(name);
}

/// Render the bottom dock (via the real `views::dock_panel`) at 2× density — the close-up for console/MDI review.
fn snapshot_dock(name: &str, view: ViewState, ui: UiState) {
  use super::intent::IntentSink;
  use super::metrics::Metrics;
  let _ = crate::i18n::init();
  let palette = ui.style.palette;
  let state = HarnessState::new(view, ui);
  let time = no_time();
  let mut harness = egui_kittest::Harness::builder()
    .with_size(egui::vec2(900.0, Metrics::DOCK_H + Metrics::STATUS_BAR_H + 16.0))
    .with_pixels_per_point(2.0)
    .build_ui_state(
      move |ui, state: &mut HarnessState| {
        let mut sink = IntentSink::new();
        egui::Panel::bottom("status").exact_size(Metrics::STATUS_BAR_H).show_inside(ui, |_ui| {});
        super::views::dock_panel(ui, &state.view, &mut state.ui, time, None, &mut sink);
        state.intents.extend(sink.drain());
      },
      state,
    );
  super::fonts::install(&harness.ctx);
  super::shell::apply_theme(&harness.ctx, &palette, 1.0);
  harness.run_steps(2);
  harness.snapshot(name);
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p skirnir -- --ignored snapshot`"]
fn snapshot_dock_console_2x() {
  // The console tab close-up: fixture traffic in the log, the recessed MDI strip with its `›` prompt and the
  // accent Send button. Connected, so the field is enabled with the command hint.
  let mut view = idle_view();
  view.note_sent("$$".to_string());
  view.apply(Event::Response(Response::Ok));
  view.apply(Event::Response(Response::Message("MSG:Fixture message".to_string())));
  view.apply(Event::Response(Response::Error(20)));
  snapshot_dock("dock_console_2x", view, fixture_ui());
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p skirnir -- --ignored snapshot`"]
fn snapshot_dock_console_disconnected_2x() {
  // Disconnected: the MDI field and Send are disabled and the hint tells the operator why.
  snapshot_dock("dock_console_disconnected_2x", ViewState::default(), UiState::default());
}

/// Render the app settings dialog BODY at 2× density against a given config. The floating window chrome is
/// egui-standard; the body is what carries our layout, so it is snapshotted directly for determinism.
fn snapshot_app_settings(name: &str, config: crate::config::Config, ui: UiState) {
  use super::intent::IntentSink;
  let _ = crate::i18n::init();
  let palette = ui.style.palette;
  let state = HarnessState::new(ViewState::default(), ui);
  let mut harness = egui_kittest::Harness::builder()
    .with_size(egui::vec2(440.0, 640.0))
    .with_pixels_per_point(2.0)
    .build_ui_state(
      move |ui, state: &mut HarnessState| {
        let mut sink = IntentSink::new();
        super::app_settings::body(ui, &mut state.ui, &config, true, &mut sink);
        state.intents.extend(sink.drain());
      },
      state,
    );
  super::fonts::install(&harness.ctx);
  super::shell::apply_theme(&harness.ctx, &palette, 1.0);
  harness.run_steps(2);
  harness.snapshot(name);
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p skirnir -- --ignored snapshot`"]
fn snapshot_app_settings_builtin_2x() {
  // A built-in theme is active: the general rows, the create-theme row, and the read-only hint (no pickers).
  snapshot_app_settings("app_settings_builtin_2x", crate::config::Config::default(), UiState::default());
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p skirnir -- --ignored snapshot`"]
fn snapshot_app_settings_user_theme_2x() {
  // A user theme is active: the grouped colour pickers render, seeded from a fixture capture of the default.
  let mut config = crate::config::Config::default();
  let theme = crate::config::ThemeOverride::from_palette(&super::theme::Palette::default_dark());
  config.appearance.themes.insert("fixture-theme".to_string(), theme);
  config.appearance.active_theme = "fixture-theme".to_string();
  snapshot_app_settings("app_settings_user_theme_2x", config, UiState::default());
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p skirnir -- --ignored snapshot`"]
fn snapshot_app_settings_picker_open() {
  // The colour-picker POPUP, open over the dialog: click the first swatch (`bg`, near the top so the popup has
  // room to render fully) and capture the frame with the picker's colour square, sliders, and channel fields.
  use super::intent::IntentSink;
  use egui::accesskit::Role;
  use egui_kittest::kittest::Queryable;
  let _ = crate::i18n::init();
  let mut config = crate::config::Config::default();
  let theme = crate::config::ThemeOverride::from_palette(&super::theme::Palette::default_dark());
  config.appearance.themes.insert("fixture-theme".to_string(), theme);
  config.appearance.active_theme = "fixture-theme".to_string();
  let ui_state = UiState::default();
  let palette = ui_state.style.palette;
  let state = HarnessState::new(ViewState::default(), ui_state);
  // 1× density: kittest's pointer clicks land at doubled coordinates under `with_pixels_per_point(2.0)` (rects
  // are points, the click lands in pixels), so the swatch click misses and the popup never opens at 2×.
  let mut harness = egui_kittest::Harness::builder()
    .with_size(egui::vec2(440.0, 640.0))
    .build_ui_state(
      move |ui, state: &mut HarnessState| {
        let mut sink = IntentSink::new();
        super::app_settings::body(ui, &mut state.ui, &config, true, &mut sink);
        state.intents.extend(sink.drain());
      },
      state,
    );
  super::fonts::install(&harness.ctx);
  super::shell::apply_theme(&harness.ctx, &palette, 1.0);
  harness.run_steps(2);
  {
    let swatches: Vec<_> = harness.query_all_by_role(Role::ColorWell).collect();
    assert!(!swatches.is_empty(), "the user-theme editor must render colour swatches");
    swatches[0].click();
  }
  harness.run();
  // Guard the capture: the popup must actually be open (its R/G/B channel fields exist) or the shot is a lie.
  assert!(
    harness.query_all_by_role(Role::SpinButton).count() > 1,
    "the colour-picker popup must be open before the snapshot (channel fields present)"
  );
  harness.snapshot("app_settings_picker_open");
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p skirnir -- --ignored snapshot`"]
fn snapshot_shell_idle_custom_theme() {
  // The whole window rendered through a USER theme with loudly modified tokens (magenta control accent, teal
  // toolpath cut) — the visible end of the picker pipeline: config theme → resolved palette → live shell.
  let mut config = crate::config::Config::default();
  let mut theme = crate::config::ThemeOverride::from_palette(&super::theme::Palette::default_dark());
  theme.accent = crate::config::ColorSpec::parse("#FF00FF");
  theme.accent_hover = crate::config::ColorSpec::parse("#FF66FF");
  theme.accent_active = crate::config::ColorSpec::parse("#CC00CC");
  theme.toolpath_cut = crate::config::ColorSpec::parse("#00FFC8");
  config.appearance.themes.insert("fixture-magenta".to_string(), theme);
  config.appearance.active_theme = "fixture-magenta".to_string();
  let (palette, notice) = config.palette();
  assert!(notice.is_none(), "the fixture theme must resolve cleanly: {notice:?}");
  let mut ui = fixture_ui();
  ui.style.palette = palette;
  snapshot_shell("shell_idle_custom_theme", DEFAULT_SIZE, idle_view(), ui, no_time());
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p skirnir -- --ignored snapshot`"]
fn snapshot_toolbar_idle_2x() {
  snapshot_toolbar("toolbar_idle_2x", 1280.0, idle_view(), fixture_ui());
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p skirnir -- --ignored snapshot`"]
fn snapshot_toolbar_disconnected_2x() {
  let mut ui = UiState::default();
  ui.ports = vec![PortInfo::bare("/dev/cu.usbmodemFAKE1")];
  ui.selected_port = "/dev/cu.usbmodemFAKE1".to_string();
  snapshot_toolbar("toolbar_disconnected_2x", 1280.0, ViewState::default(), ui);
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p skirnir -- --ignored snapshot`"]
fn snapshot_shell_disconnected() {
  // The fresh-launch state: no link, empty console, no program. Every machine control must read disabled.
  let mut ui = UiState::default();
  ui.ports = vec![PortInfo::bare("/dev/cu.usbmodemFAKE1")];
  ui.selected_port = "/dev/cu.usbmodemFAKE1".to_string();
  snapshot_shell("shell_disconnected", DEFAULT_SIZE, ViewState::default(), ui, no_time());
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p skirnir -- --ignored snapshot`"]
fn snapshot_shell_idle() {
  snapshot_shell("shell_idle", DEFAULT_SIZE, idle_view(), fixture_ui(), no_time());
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p skirnir -- --ignored snapshot`"]
fn snapshot_shell_idle_narrow() {
  // The minimum supported window (the app's min inner size is 800×500): everything must still fit or degrade
  // gracefully — no clipped controls, no unpainted strips.
  snapshot_shell("shell_idle_narrow", egui::vec2(800.0, 500.0), idle_view(), fixture_ui(), no_time());
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p skirnir -- --ignored snapshot`"]
fn snapshot_shell_idle_wide() {
  snapshot_shell("shell_idle_wide", egui::vec2(1680.0, 950.0), idle_view(), fixture_ui(), no_time());
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p skirnir -- --ignored snapshot`"]
fn snapshot_shell_idle_rotary() {
  // A rotary-enabled (DOC-10) board: the 4-field status must grow the DRO an A row (degrees, violet) and the jog
  // pad offers the A± column beside Z±.
  let mut view = ViewState::default();
  view.connection = ConnectionState::Idle;
  apply_status(&mut view, "Idle|MPos:12.500,20.000,-1.200,45.000|WCO:2.000,3.000,1.000,0.000|FS:0,0|Ov:100,100,100");
  snapshot_shell("shell_idle_rotary", DEFAULT_SIZE, view, fixture_ui(), no_time());
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p skirnir -- --ignored snapshot`"]
fn snapshot_shell_streaming() {
  let ui = fixture_ui();
  let total = ui.program.len();
  let time = TimeEstimate {
    elapsed: std::time::Duration::from_secs(51),
    remaining: Some(std::time::Duration::from_secs(75)),
    total: Some(std::time::Duration::from_secs(126)),
  };
  snapshot_shell("shell_streaming", DEFAULT_SIZE, streaming_view(total), ui, time);
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p skirnir -- --ignored snapshot`"]
fn snapshot_shell_alarm() {
  // The safety-critical state: the alarm banner and badge must be unmissable, and jog/run controls disabled.
  snapshot_shell("shell_alarm", DEFAULT_SIZE, alarm_view(), fixture_ui(), no_time());
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p skirnir -- --ignored snapshot`"]
fn snapshot_shell_idle_light_slate() {
  // The alternate built-in chromes must hold up too: same layout, lighter surfaces, same semantic accents.
  let mut ui = fixture_ui();
  ui.style.palette = super::theme::Palette::light_slate();
  snapshot_shell("shell_idle_light_slate", DEFAULT_SIZE, idle_view(), ui, no_time());
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p skirnir -- --ignored snapshot`"]
fn snapshot_shell_idle_midnight() {
  let mut ui = fixture_ui();
  ui.style.palette = super::theme::Palette::midnight();
  snapshot_shell("shell_idle_midnight", DEFAULT_SIZE, idle_view(), ui, no_time());
}
