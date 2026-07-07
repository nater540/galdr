//! GPU image-snapshot tests: offscreen wgpu renders of the FULL application shell, diffed against committed
//! PNG baselines under `eitri-app/tests/snapshots/`. The visual regression net the interaction tests cannot
//! provide — a spacing tweak that silently breaks a layout shows up here as a pixel diff.
//!
//! Every test is `#[ignore]`d so a plain `cargo test` (and GPU-less CI) never needs a GPU. Run them explicitly:
//!
//! ```sh
//! cargo test -p eitri-app -- --ignored snapshot            # diff against the committed baselines
//! UPDATE_SNAPSHOTS=1 cargo test -p eitri-app -- --ignored snapshot   # regenerate the baselines
//! ```
//!
//! Rendered states use OBVIOUSLY FAKE fixture data (the synthetic two-pad Gerber, `fixture-*` names) — never a
//! real board. Each state renders through `shell::shell_panels`, the same function the live window draws with,
//! with the app's fonts and theme applied so the pixels match reality.

use eframe::egui;

use super::scene;
use super::theme::Palette;
use super::ui_test::{DEFAULT_SIZE, HarnessState, build_shell_harness, render_in};
use super::view_state::{LogKind, OpView, TreeRow, ViewState};
use super::views::{DockTab, SelectedInfo, UiState};
use eitri_gcode::IsolationJob;
use eitri_project::{DirectionSpec, IsolationSpec, ObjectKind};
use eitri_script::Session;

/// The application's minimum window size (`with_min_inner_size` in `shell::run`) — the tightest layout every
/// locale must survive.
const MIN_SIZE: egui::Vec2 = egui::vec2(900.0, 560.0);

const GERBER: &str = include_str!("../../../../fixtures/synthetic/gerber/kicad_two_pads.gbr");
const EXCELLON: &str = include_str!("../../../../fixtures/synthetic/excellon/metric_leading.drl");

/// Build the loaded-board fixture: the two-pad Gerber, its drills, and an isolation job, with the scene and
/// tree snapshotted exactly as the shell would after those ops.
fn loaded_board() -> (HarnessState, eitri_project::ObjectId) {
  let mut session = Session::new("fixture-board");
  let gerber = session.open_gerber_str("fixture-top", GERBER).expect("fixture gerber opens");
  session.open_excellon_str("fixture-drills", EXCELLON).expect("fixture drills open");
  let spec =
    IsolationSpec { tool_diameter: 0.2, passes: 1, overlap: 0.0, combine: false, direction: DirectionSpec::Climb };
  let job = session.isolate(gerber, spec, IsolationJob::default()).expect("fixture isolation succeeds");

  let mut view = ViewState::default();
  let rows: Vec<TreeRow> = session
    .object_ids()
    .into_iter()
    .map(|id| {
      let object = session.object(id).expect("listed ids resolve");
      TreeRow { id, name: object.meta.name.clone(), kind: object.kind(), visible: object.meta.visible }
    })
    .collect();
  view.set_tree(rows, session.can_undo(), session.can_redo());
  view.log_line(LogKind::Info, "opened fixture-top (FIXTURE data)");
  view.log_line(LogKind::Ok, "Isolation routing finished");

  let mut ui = UiState::default();
  let scene = scene::build_scene(&session);
  ui.pending_fit = true; // the first paint frames the board, as the live shell does after a first open.
  let state = HarnessState { view, ui, scene, ..HarnessState::new(ViewState::default(), UiState::default()) };
  (state, job)
}

/// Render one shell state offscreen and snapshot it. `name` becomes `tests/snapshots/<name>.png`.
fn snapshot_shell(name: &str, size: egui::Vec2, state: HarnessState, locale: &str) {
  let _locale = render_in(locale);
  let mut harness = build_shell_harness(state, size);
  // Three settle frames: fonts/theme land on the first, the pending fit consumes the real canvas rect on the
  // second, and the third is steady-state.
  harness.run_steps(3);
  harness.snapshot(name);
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p eitri-app -- --ignored snapshot`"]
fn snapshot_shell_empty() {
  // The fresh-launch state: empty tree with its invitation, empty parameter panel, the canvas empty state.
  let state = HarnessState::new(ViewState::default(), UiState::default());
  snapshot_shell("shell_empty", DEFAULT_SIZE, state, crate::i18n::EN_US);
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p eitri-app -- --ignored snapshot`"]
fn snapshot_shell_board_loaded() {
  // The vertical-slice money shot: copper + drills + isolation trails on the canvas, three tree rows, the log.
  let (state, _) = loaded_board();
  snapshot_shell("shell_board_loaded", DEFAULT_SIZE, state, crate::i18n::EN_US);
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p eitri-app -- --ignored snapshot`"]
fn snapshot_shell_gerber_selected_params() {
  // The Gerber selected: the isolation parameter grid + accent Run button in the right column, the selection
  // rim on the canvas.
  let (mut state, _) = loaded_board();
  state.view.selected = state.view.tree.iter().find(|r| r.kind == ObjectKind::Gerber).map(|r| r.id);
  state.ui.selected_info = Some(SelectedInfo::default());
  snapshot_shell("shell_gerber_selected", DEFAULT_SIZE, state, crate::i18n::EN_US);
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p eitri-app -- --ignored snapshot`"]
fn snapshot_shell_job_selected_gcode_tab() {
  // The job selected with the G-code dock tab active: the export panel and the numbered G-code preview.
  let (mut state, job) = loaded_board();
  state.view.selected = Some(job);
  state.ui.dock_tab = DockTab::Gcode;
  state.ui.selected_info = Some(SelectedInfo {
    gcode_lines: 42,
    dialect: "grbl".to_string(),
    ..SelectedInfo::default()
  });
  state.ui.gcode_preview = vec![
    "; FIXTURE isolation job (snapshot test data)".to_string(),
    "G21".to_string(),
    "G90".to_string(),
    "G17".to_string(),
    "M3 S10000".to_string(),
    "G0 Z2.000".to_string(),
    "G0 X1.100 Y-1.100".to_string(),
    "G1 Z-0.100 F60".to_string(),
    "G1 X8.900 F120".to_string(),
    "G2 X9.900 Y-0.100 I0.000 J1.000".to_string(),
  ];
  snapshot_shell("shell_job_selected_gcode", DEFAULT_SIZE, state, crate::i18n::EN_US);
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p eitri-app -- --ignored snapshot`"]
fn snapshot_shell_op_running() {
  // Mid-operation: the dock's progress cluster (label + determinate bar + Cancel), the busy status dot, the
  // locked parameter panel with its explanation, and the disabled toolbar/run controls.
  let (mut state, _) = loaded_board();
  state.view.selected = state.view.tree.iter().find(|r| r.kind == ObjectKind::Gerber).map(|r| r.id);
  state.view.op = OpView::Running { label: "Isolation routing".to_string(), done: 3, total: 8 };
  snapshot_shell("shell_op_running", DEFAULT_SIZE, state, crate::i18n::EN_US);
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p eitri-app -- --ignored snapshot`"]
fn snapshot_shell_op_failed_log() {
  // The failure state: an error line in the log (marker + text, not colour alone).
  let (mut state, _) = loaded_board();
  state.view.log_line(LogKind::Error, "Isolation routing failed: fixture reason (snapshot data)");
  state.view.log_line(LogKind::Warn, "Drill planning was cancelled");
  snapshot_shell("shell_op_failed_log", DEFAULT_SIZE, state, crate::i18n::EN_US);
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p eitri-app -- --ignored snapshot`"]
fn snapshot_shell_narrow() {
  // The minimum supported window: everything must still fit or degrade gracefully.
  let (mut state, _) = loaded_board();
  state.view.selected = state.view.tree.iter().find(|r| r.kind == ObjectKind::Gerber).map(|r| r.id);
  snapshot_shell("shell_narrow", MIN_SIZE, state, crate::i18n::EN_US);
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p eitri-app -- --ignored snapshot`"]
fn snapshot_shell_wide() {
  let (state, _) = loaded_board();
  snapshot_shell("shell_wide", egui::vec2(1680.0, 950.0), state, crate::i18n::EN_US);
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p eitri-app -- --ignored snapshot`"]
fn snapshot_shell_light_slate() {
  // The alternate chromes must hold up: same layout, lighter surfaces, same canvas semantics.
  let (mut state, _) = loaded_board();
  state.ui.style.palette = Palette::light_slate();
  snapshot_shell("shell_light_slate", DEFAULT_SIZE, state, crate::i18n::EN_US);
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p eitri-app -- --ignored snapshot`"]
fn snapshot_shell_midnight() {
  let (mut state, _) = loaded_board();
  state.ui.style.palette = Palette::midnight();
  snapshot_shell("shell_midnight", DEFAULT_SIZE, state, crate::i18n::EN_US);
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p eitri-app -- --ignored snapshot`"]
fn snapshot_shell_sv_min() {
  // Swedish at the minimum window: the longer labels ("Isolationsfräsning", "Öppna Excellon…") stress the
  // fixed columns and the toolbar, so a translation that overflows shows up as a pixel diff.
  let (mut state, _) = loaded_board();
  state.view.selected = state.view.tree.iter().find(|r| r.kind == ObjectKind::Gerber).map(|r| r.id);
  snapshot_shell("shell_sv_min", MIN_SIZE, state, crate::i18n::SV_SE);
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p eitri-app -- --ignored snapshot`"]
fn snapshot_app_settings_builtin_2x() {
  // The settings dialog body with a built-in theme active: general rows + the read-only hint. 2× density for
  // glyph/spacing review.
  snapshot_settings("app_settings_builtin_2x", crate::config::Config::default(), crate::i18n::EN_US);
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p eitri-app -- --ignored snapshot`"]
fn snapshot_app_settings_user_theme_2x() {
  // A user theme active: the grouped colour pickers render, seeded from a fixture capture of the default.
  let mut config = crate::config::Config::default();
  let theme = crate::config::ThemeOverride::from_palette(&Palette::default_dark());
  config.appearance.themes.insert("fixture-theme".to_string(), theme);
  config.appearance.active_theme = "fixture-theme".to_string();
  snapshot_settings("app_settings_user_theme_2x", config, crate::i18n::EN_US);
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p eitri-app -- --ignored snapshot`"]
fn snapshot_tool_db_2x() {
  // The tool-database dialog with a small library and one tool selected: the action row, the selectable list,
  // and the identity + isolation + drill editor grids. 2× density for glyph/spacing review.
  snapshot_tool_db("tool_db_2x", crate::i18n::EN_US);
}

#[test]
#[ignore = "needs a GPU (wgpu offscreen render) — run with `cargo test -p eitri-app -- --ignored snapshot`"]
fn snapshot_tool_db_sv_2x() {
  // The same dialog in Swedish: the longer sv-SE labels ("Isolationsstandard", "Lägg till verktyg", "osparade
  // ändringar") are where a fixed-width grid clips — this is the locale-clipping guard for the tool DB.
  snapshot_tool_db("tool_db_sv_2x", crate::i18n::SV_SE);
}

/// Render the tool-database dialog BODY at 2× density in a given locale, with a two-tool fixture library and
/// the second tool selected for editing (so every editor grid is exercised).
fn snapshot_tool_db(name: &str, locale: &str) {
  use eitri_core::Length;
  use eitri_project::{DrillDefaults, IsolationDefaults, ToolDatabase, ToolEntry, ToolId};

  let mut db = ToolDatabase::new();
  db.add(ToolEntry {
    id: ToolId(0),
    name: "0.2mm V-bit".to_string(),
    diameter: Length::from_mm(0.2),
    isolation: IsolationDefaults::default(),
    drilling: DrillDefaults::default(),
  });
  let edit_id = db.add(ToolEntry {
    id: ToolId(0),
    name: "0.8mm drill".to_string(),
    diameter: Length::from_mm(0.8),
    isolation: IsolationDefaults::default(),
    drilling: DrillDefaults::default(),
  });

  let _locale = render_in(locale);
  let ui_state = UiState { tool_db_selected: Some(edit_id), ..UiState::default() };
  let palette = ui_state.style.palette;
  let state = HarnessState::new(ViewState::default(), ui_state);
  let mut harness = egui_kittest::Harness::builder()
    .with_size(egui::vec2(440.0, 640.0))
    .with_pixels_per_point(2.0)
    .build_ui_state(
      move |ui, state: &mut HarnessState| {
        let mut sink = super::intent::IntentSink::new();
        super::tool_db::body(ui, &mut state.ui, &db, true, &mut sink);
        state.intents.extend(sink.drain());
      },
      state,
    );
  super::fonts::install(&harness.ctx);
  super::shell::apply_theme(&harness.ctx, &palette, 1.0);
  harness.run_steps(2);
  harness.snapshot(name);
}

/// Render the settings dialog BODY at 2× density against a given config (the floating window chrome is
/// egui-standard; the body carries our layout).
fn snapshot_settings(name: &str, config: crate::config::Config, locale: &str) {
  let _locale = render_in(locale);
  let ui_state = UiState::default();
  let palette = ui_state.style.palette;
  let state = HarnessState::new(ViewState::default(), ui_state);
  let mut harness = egui_kittest::Harness::builder()
    .with_size(egui::vec2(440.0, 640.0))
    .with_pixels_per_point(2.0)
    .build_ui_state(
      move |ui, state: &mut HarnessState| {
        let mut sink = super::intent::IntentSink::new();
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
