//! Tutorial-screenshot generator: offscreen wgpu renders of the FULL shell walking the REAL `starter-*` PCB
//! fixtures through the end-to-end CAM workflow (open → isolate → drill → cutout), written as polished product
//! screenshots for the "How to mill a PCB" tutorial.
//!
//! Unlike [`super::snapshot_test`], nothing here is a regression baseline: the PNGs go to the PARENT repo's
//! `docs/images/mill-a-pcb/` and are simply overwritten on each run. Run explicitly (needs a GPU):
//!
//! ```sh
//! cargo test -p eitri-app -- --ignored render_tutorial_screenshots
//! ```
//!
//! Every state is built the way the live shell builds it — `Session` calls for the real CAM outputs,
//! `scene::build_scene` for the canvas, and mirrors of the shell's `refresh_from_session` /
//! `refresh_selection_extras` for the tree and the parameter-panel facts — so the pixels match what an
//! operator actually sees, including the real emitted starter G-code in the dock preview.

use std::path::PathBuf;

use eframe::egui;

use super::scene;
use super::ui_test::{DEFAULT_SIZE, HarnessState, build_shell_harness, render_in};
use super::view_state::{LogKind, OpView, Selection, TreeRow, ViewState};
use super::views::{DockTab, SelectedInfo, StockDraft, UiState};
use eitri_project::{ObjectId, ObjectPayload};
use eitri_script::Session;

/// The repo's `eitri/fixtures/` directory, resolved from this crate so the test runs from any cwd.
fn fixtures_dir() -> PathBuf {
  PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../fixtures")
}

/// The tutorial image directory in the PARENT (Galdr) repo: `docs/images/mill-a-pcb/`, created on demand.
fn shots_dir() -> PathBuf {
  let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../docs/images/mill-a-pcb");
  std::fs::create_dir_all(&dir).expect("the tutorial image directory is creatable");
  dir
}

/// Mirror of the shell's `refresh_from_session`: the tree rows and history flags a live window would show.
fn synced_view(session: &Session) -> ViewState {
  let mut view = ViewState::default();
  let rows: Vec<TreeRow> = session
    .object_ids()
    .into_iter()
    .filter_map(|id| {
      let object = session.object(id).ok()?;
      Some(TreeRow { id, name: object.meta.name.clone(), kind: object.kind(), visible: object.meta.visible })
    })
    .collect();
  view.set_tree(rows, session.can_undo(), session.can_redo());
  view
}

/// A fresh harness state synced from the session, with the first-paint fit queued exactly as the live shell
/// queues it after an open. (The scene builder itself strips the importer's synthetic start-at-origin rapid,
/// so job previews and the fit hug the board here just as they do in the live app.)
fn shot_state(session: &Session) -> HarnessState {
  let mut state = HarnessState::new(synced_view(session), UiState::default());
  state.scene = scene::build_scene(session);
  let (x, y, z) = session.work_origin();
  state.ui.work_origin = [x, y, z];
  state.ui.stock = session.stock();
  if let Some(stock) = state.ui.stock {
    state.ui.stock_draft = StockDraft::from_stock(stock);
  }
  state.ui.pending_fit = true;
  state
}

/// Mirror of the shell's `refresh_selection_extras`: select `id` and snapshot the parameter-panel facts (and,
/// for a CNC job, the dock's G-code preview) from the session.
fn select(state: &mut HarnessState, session: &Session, id: ObjectId) {
  state.view.selected = Some(Selection::Object(id));
  state.ui.selected_info = None;
  state.ui.gcode_preview.clear();
  let object = session.object(id).expect("the selected object resolves");
  let mut info =
    SelectedInfo { bounds: state.scene.object(id).and_then(|entry| entry.bounds), ..SelectedInfo::default() };
  match &object.payload {
    ObjectPayload::Gerber(_) => {}
    ObjectPayload::Excellon(excellon) => {
      if let Some(image) = &excellon.image {
        info.hits = image.hits.len();
        info.tools = image.tools.len();
      }
    }
    ObjectPayload::Geometry(geometry) => {
      info.polygons = geometry.polygons.len();
      info.polylines = geometry.polylines.len();
    }
    ObjectPayload::CncJob(job) => {
      info.gcode_lines = job.gcode.len();
      info.dialect = job.dialect.clone();
      state.ui.gcode_preview = job.gcode.iter().take(2000).cloned().collect();
    }
  }
  state.ui.selected_info = Some(info);
}

/// Render one shell state offscreen and write it as `docs/images/mill-a-pcb/<filename>`.
fn save_shot(filename: &str, size: egui::Vec2, state: HarnessState) {
  let mut harness = build_shell_harness(state, size);
  // Three settle frames, as the snapshot suite uses: fonts/theme, then the fit against the real canvas rect,
  // then steady state.
  harness.run_steps(3);
  let image = harness.render().expect("the offscreen wgpu render succeeds");
  let path = shots_dir().join(filename);
  image.save(&path).expect("the tutorial PNG writes");
  println!("wrote {}", path.display());
}

/// Render the tool-database dialog body at 2× density (matching the committed `tool_db_2x` framing) with a
/// small realistic PCB tool library, and write it to the tutorial directory.
fn save_tool_db_shot(filename: &str) {
  use eitri_core::Length;
  use eitri_project::{DrillDefaults, IsolationDefaults, ToolDatabase, ToolEntry, ToolId};

  let mut db = ToolDatabase::new();
  let vbit = db.add(ToolEntry {
    id: ToolId(0),
    name: "0.2 mm V-bit".to_string(),
    diameter: Length::from_mm(0.2),
    isolation: IsolationDefaults::default(),
    drilling: DrillDefaults::default(),
  });
  db.add(ToolEntry {
    id: ToolId(0),
    name: "0.8 mm drill".to_string(),
    diameter: Length::from_mm(0.8),
    isolation: IsolationDefaults::default(),
    drilling: DrillDefaults::default(),
  });
  db.add(ToolEntry {
    id: ToolId(0),
    name: "1.0 mm end mill".to_string(),
    diameter: Length::from_mm(1.0),
    isolation: IsolationDefaults::default(),
    drilling: DrillDefaults::default(),
  });

  let ui_state = UiState { tool_db_selected: Some(vbit), ..UiState::default() };
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
  let image = harness.render().expect("the offscreen wgpu render succeeds");
  // The dialog body ends above the harness floor; trim the unpainted (fully transparent) band below it so the
  // tutorial PNG is a tight card, not a screenshot with a dead margin.
  let mut last = 0;
  for (y, row) in image.rows().enumerate() {
    if row.into_iter().any(|px| px.0[3] != 0) {
      last = y as u32;
    }
  }
  let height = (last + 9).min(image.height());
  let image = image::imageops::crop_imm(&image, 0, 0, image.width(), height).to_image();
  let path = shots_dir().join(filename);
  image.save(&path).expect("the tutorial PNG writes");
  println!("wrote {}", path.display());
}

/// Read one starter fixture file to a string.
fn fixture(rel: &str) -> String {
  let path = fixtures_dir().join(rel);
  std::fs::read_to_string(&path).unwrap_or_else(|err| panic!("fixture {} reads: {err}", path.display()))
}

#[test]
#[ignore = "writes tutorial PNGs via a GPU render — run with `cargo test -p eitri-app -- --ignored render_tutorial_screenshots`"]
fn render_tutorial_screenshots() {
  let _locale = render_in(crate::i18n::EN_US);

  // ── 01: the fresh launch — empty tree invitation, empty parameter panel, empty canvas state. ──────────────
  save_shot("01-fresh-launch.png", DEFAULT_SIZE, HarnessState::new(ViewState::default(), UiState::default()));

  // ── The workflow session: the starter board's front copper and plated drills, exactly as an operator would
  //    open them. Names match what the file-open path produces (the file stem).
  let mut session = Session::new("starter");
  let f_cu = session
    .open_gerber_str("starter-F_Cu", fixture("gerber/starter-F_Cu.gbr"))
    .expect("the starter front copper opens");
  let pth = session
    .open_excellon_str("starter-PTH", fixture("excellon/starter-PTH.drl"))
    .expect("the starter plated drills open");

  // ── 02: board loaded — copper filled, drill marks, two tree rows, the fit framing the board. ─────────────
  let mut state = shot_state(&session);
  state.view.log_line(LogKind::Ok, "Open Gerber finished");
  state.view.log_line(LogKind::Ok, "Open Excellon finished");
  save_shot("02-board-loaded.png", DEFAULT_SIZE, state);

  // ── 03: the front copper selected — the isolation parameter panel + Run, the selection rim. ──────────────
  let mut state = shot_state(&session);
  state.view.log_line(LogKind::Ok, "Open Gerber finished");
  state.view.log_line(LogKind::Ok, "Open Excellon finished");
  select(&mut state, &session, f_cu);
  save_shot("03-isolation-params.png", DEFAULT_SIZE, state);

  // ── 04: isolation running — the dock progress cluster, busy status, locked params. ───────────────────────
  let mut state = shot_state(&session);
  state.view.log_line(LogKind::Ok, "Open Gerber finished");
  state.view.log_line(LogKind::Ok, "Open Excellon finished");
  select(&mut state, &session, f_cu);
  state.view.op = OpView::Running { label: "Isolation routing".to_string(), done: 3, total: 8 };
  save_shot("04-isolating.png", DEFAULT_SIZE, state);

  // ── The real isolation job, run with the panel's own defaults — the same spec/job the Run button submits.
  let iso_draft = super::views::IsolationDraft::default();
  let iso_job = session
    .isolate(f_cu, iso_draft.to_spec(), iso_draft.to_job(Some("starter-F_Cu".to_string())))
    .expect("the starter isolation succeeds");

  // ── 05: the isolation job selected, G-code dock tab active — export panel + the REAL emitted program. ────
  let mut state = shot_state(&session);
  state.view.log_line(LogKind::Ok, "Open Gerber finished");
  state.view.log_line(LogKind::Ok, "Open Excellon finished");
  state.view.log_line(LogKind::Ok, "Isolation routing finished");
  select(&mut state, &session, iso_job);
  state.ui.dock_tab = DockTab::Gcode;
  // A taller dock (as if the operator dragged the divider up) so the preview shows a useful stretch of the
  // program — a dozen lines instead of the default five.
  state.split = super::dock_tiles::CentralSplit::new(0.42);
  save_shot("05-isolation-gcode.png", DEFAULT_SIZE, state);

  // ── 06: the plated drills selected — the drill parameter panel (positive depth magnitude) + Run. ─────────
  let mut state = shot_state(&session);
  state.view.log_line(LogKind::Ok, "Open Gerber finished");
  state.view.log_line(LogKind::Ok, "Open Excellon finished");
  state.view.log_line(LogKind::Ok, "Isolation routing finished");
  select(&mut state, &session, pth);
  // Through a 1.6 mm board with 0.2 mm of breakthrough — the tutorial's worked drilling depth.
  state.ui.drill.depth = 1.8;
  save_shot("06-drilling-params.png", DEFAULT_SIZE, state);

  // ── The remaining jobs: the real drilling plan and the tabbed board cutout around the real outline. ──────
  let drill_draft = super::views::DrillDraft { depth: 1.8, ..super::views::DrillDraft::default() };
  session
    .drill(pth, drill_draft.to_spec(), drill_draft.to_job(Some("starter-PTH".to_string())))
    .expect("the starter drilling succeeds");
  let edge = session
    .open_gerber_str("starter-Edge_Cuts", fixture("gerber/starter-Edge_Cuts.gbr"))
    .expect("the starter outline opens");
  let board = scene::build_scene(&session).object(edge).and_then(|entry| entry.bounds).expect("the outline has bounds");
  let cutout_draft = super::op_drafts::CutoutDraft {
    tool_diameter: 1.0,
    rect_min: [board.0, board.1],
    rect_max: [board.2, board.3],
    ..super::op_drafts::CutoutDraft::default()
  };
  session
    .cutout(cutout_draft.to_spec(cutout_draft.rectangle_outline()), cutout_draft.job.to_job(Some("starter".to_string())))
    .expect("the starter cutout succeeds");

  // ── 07: everything generated — copper, drills, outline, and all three toolpath jobs on the canvas. ───────
  let mut state = shot_state(&session);
  state.view.log_line(LogKind::Ok, "Isolation routing finished");
  state.view.log_line(LogKind::Ok, "Drill planning finished");
  state.view.log_line(LogKind::Ok, "Board cutout finished");
  save_shot("07-toolpaths.png", DEFAULT_SIZE, state);

  // ── 08: the tool database dialog — the library list + the selected tool's editor grids. ──────────────────
  save_tool_db_shot("08-tool-database.png");

  // ── 09: the Setup node — the project-wide stock/work-zero setup: stock fitted to the board outline at a
  //    1.6 mm thickness, bottom-left datum, Z0 on the material top; the dashed stock block and the placed
  //    crosshair on the canvas. (Set up BEFORE running the ops in a real workflow so every job posts near
  //    X0 Y0; here it lands after so the earlier shots stay in the native frame.) ──────────────────────────
  session.fit_stock_to(edge, 1.6).expect("the outline bounds the stock");
  let (zero_x, zero_y, zero_z) = session.work_origin();
  let mut state = shot_state(&session);
  state.view.log_line(LogKind::Ok, "Isolation routing finished");
  state.view.log_line(LogKind::Ok, "Drill planning finished");
  state.view.log_line(LogKind::Ok, "Board cutout finished");
  state.view.log_line(LogKind::Ok, format!("Stock set — work zero at X {zero_x:.3} Y {zero_y:.3} Z {zero_z:.3}"));
  state.view.selected = Some(Selection::Setup);
  state.ui.fit_reference = Some(edge); // the combo shows the outline the stock was actually fitted to.
  save_shot("09-set-datum.png", DEFAULT_SIZE, state);
}
