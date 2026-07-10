//! The eframe application shell: owns the [`SessionSlot`] (the engine session or the in-flight op), the pure
//! [`ViewState`], the transient [`UiState`], the cached [`RenderScene`], and the loaded [`Config`]; lays out
//! the window; and executes the [`Intent`]s the views emit.
//!
//! Frame order mirrors skirnir's shell: (1) pump the worker (progress + outcome) into the view state, (2) build
//! the frame — views only read state and push intents, (3) drain the sink and perform the side effects, (4)
//! schedule a repaint while an op is in flight so progress animates without input events.

use std::path::Path;

use eframe::egui;

use super::dock_tiles::CentralSplit;
use super::intent::{Intent, IntentSink};
use super::metrics::Metrics;
use super::ops::{OpOutput, OpRequest, SessionSlot};
use super::scene::{self, RenderScene};
use super::theme::Palette;
use super::view_state::{GroupView, LogKind, Selection, TreeRow, ViewState};
use super::views::{self, RuntimeStyle, SelectedInfo, StockDraft, UiState};
use crate::config::Config;
use crate::tr;
use eitri_core::ProgressEvent;
use eitri_project::{DirectionSpec, ObjectId, ObjectKind, ObjectPayload, ToolDatabase};
use eitri_script::Session;

/// The most G-code lines the dock preview keeps (a full job can run to hundreds of thousands; the preview is
/// for eyeballing the header and the shape of the program, and the export writes the real thing).
const GCODE_PREVIEW_CAP: usize = 2000;

/// The session name a fresh (unsaved) project carries.
const UNTITLED: &str = "untitled";

/// The application state.
pub struct EitriApp {
  /// The engine session, home or away on a worker.
  slot: SessionSlot,
  /// The pure view state (tree snapshot, log, op lifecycle).
  view: ViewState,
  /// The transient widget state (drafts, canvas transform, dialogs).
  ui: UiState,
  /// The cached paint-ready scene, rebuilt on collection changes.
  scene: RenderScene,
  /// The central canvas/dock split.
  split: CentralSplit,
  /// The loaded app config (appearance, language, canvas knobs).
  config: Config,
  /// Whether the in-memory config differs from disk (drives the settings dialog's Save row).
  config_dirty: bool,
  /// The user-global tool library (persisted separately from any project).
  tool_db: ToolDatabase,
  /// Whether the in-memory tool library differs from disk (drives the tool-DB dialog's Save row).
  tool_db_dirty: bool,
}

impl EitriApp {
  /// Build the app around a fresh session, applying the config's appearance and logging any load notices.
  pub fn new(config: Config, notices: Vec<String>) -> Self {
    let (tool_db, tool_notice) = crate::tool_store::load();
    let mut app = EitriApp {
      slot: SessionSlot::Home(Session::new(UNTITLED)),
      view: ViewState::default(),
      ui: UiState::default(),
      scene: RenderScene::default(),
      split: CentralSplit::new(config.ui.dock_fraction),
      config,
      config_dirty: false,
      tool_db,
      tool_db_dirty: false,
    };
    apply_appearance(&mut app.ui, &app.config);
    for notice in notices {
      app.view.log_line(LogKind::Warn, notice);
    }
    if let Some(notice) = tool_notice {
      app.view.log_line(LogKind::Warn, notice);
    }
    app.refresh_tool_list();
    app.refresh_from_session();
    app
  }

  /// Rebuild the seed-from-tool snapshot the parameter panels read, after any change to the library.
  fn refresh_tool_list(&mut self) {
    self.ui.tool_list = self.tool_db.iter().map(|tool| (tool.id, tool.name.clone())).collect();
  }

  /// Rebuild everything derived from the session — the tree snapshot, the scene, the selection extras. Called
  /// after every collection change; a no-op while the session is away (the caller refreshes when it returns).
  fn refresh_from_session(&mut self) {
    let Some(session) = self.slot.session() else { return };
    let (tree, toolpaths, groups) = partition_from_session(session);
    self.view.set_tree(tree, toolpaths, session.can_undo(), session.can_redo());
    self.view.groups = groups;
    self.scene = scene::build_scene(session);
    self.sync_setup_from_session();
    self.refresh_selection_extras();
  }

  /// Refresh everything a datum/stock change affects: the Setup snapshot (the canvas stock block + crosshair), the
  /// tree's ⟳ stale badges (a datum change stales every posted job), and the undo/redo button state (the change is
  /// now undoable). Deliberately does NOT rebuild the scene — a datum change moves no geometry (jobs redraw from
  /// their baked G-code, sources are datum-independent), and this fires on every frame of a stock-spinner drag, so
  /// it must stay cheap (docs follow-up #3).
  fn refresh_after_setup_change(&mut self) {
    self.sync_setup_from_session();
    let Some(session) = self.slot.session() else { return };
    let (tree, toolpaths, groups) = partition_from_session(session);
    self.view.set_tree(tree, toolpaths, session.can_undo(), session.can_redo());
    self.view.groups = groups;
  }

  /// Snapshot the session's stock + resolved work-zero for the Setup panel, the canvas block, and the
  /// crosshair (a project load restores them; the setup intents keep them fresh in between). A committed stock
  /// also refreshes the panel's drafts, so the numbers on screen are the numbers in force.
  fn sync_setup_from_session(&mut self) {
    let Some(session) = self.slot.session() else { return };
    let (x, y, z) = session.work_origin();
    self.ui.work_origin = [x, y, z];
    self.ui.stock = session.stock();
    if let Some(stock) = self.ui.stock {
      self.ui.stock_draft = StockDraft::from_stock(stock);
    }
  }

  /// Rebuild the parameter panel's facts and the G-code preview for the current selection.
  fn refresh_selection_extras(&mut self) {
    self.ui.selected_info = None;
    self.ui.gcode_preview.clear();
    let (Some(session), Some(id)) = (self.slot.session(), self.view.selected_object()) else { return };
    let Ok(object) = session.object(id) else { return };
    let mut info = SelectedInfo { bounds: self.scene.object(id).and_then(|entry| entry.bounds), ..SelectedInfo::default() };
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
        info.tool_diameter = job.operation.tool_diameter();
        self.ui.gcode_preview = job.gcode.iter().take(GCODE_PREVIEW_CAP).cloned().collect();
      }
    }
    self.ui.selected_info = Some(info);
  }

  /// Start a worker op: flip the view into its running state and move the session out. The caller guarantees
  /// the session is home (buttons are disabled while busy); a refused launch is logged rather than ignored.
  fn launch(&mut self, request: OpRequest) {
    let label = tr!(request.label_key());
    if self.slot.launch(request) {
      self.view.op_started(label);
    } else {
      self.view.log_line(LogKind::Error, tr!("op-failed", { label: label, reason: "busy" }));
    }
  }

  /// Pump the worker: fold buffered progress into the view, and on an outcome bring the session home, log the
  /// ending, refresh the derived state, and select a newly created object.
  fn pump(&mut self) {
    if let SessionSlot::Away(op) = &self.slot {
      for event in op.drain_progress() {
        apply_progress(&mut self.view, event);
      }
    }
    let Some((trailing, result)) = self.slot.poll() else { return };
    for event in trailing {
      apply_progress(&mut self.view, event);
    }
    let label = match &self.view.op {
      super::view_state::OpView::Running { label, .. } => label.clone(),
      super::view_state::OpView::Idle => String::new(),
    };
    self.view.op_finished();
    let was_empty = self.scene.objects.is_empty();
    match result {
      Ok(OpOutput::Object(id)) => {
        self.view.log_line(LogKind::Ok, tr!("op-done", { label: label }));
        self.view.selected = Some(Selection::Object(id));
        // A freshly imported layer auto-joins the managed import group, so a board's layers move together as one
        // locked set on the stock. Folded into the import's own undo entry (see `Session::add_to_import_group`).
        // Done BEFORE the auto-fit: the auto-fit is now its own undoable setup edit (docs #3), so the group `amend`
        // must land on the import entry, not on the stock entry the auto-fit would otherwise push first.
        if self.slot.session().is_some_and(|s| s.is_imported_source(id))
          && let Some(session) = self.slot.session_mut()
        {
          let _ = session.add_to_import_group(id);
        }
        self.auto_fit_stock(id);
        self.refresh_from_session();
        // The first thing loaded into an empty canvas gets framed automatically — the operator should see
        // their board, not a distant speck at the default zoom.
        if was_empty && !self.scene.objects.is_empty() {
          self.ui.pending_fit = true;
        }
      }
      Ok(OpOutput::ProjectLoaded) => {
        self.view.log_line(LogKind::Ok, tr!("op-done", { label: label }));
        self.view.selected = None;
        self.refresh_from_session();
        self.ui.pending_fit = true;
      }
      Err(err) => {
        if matches!(err, eitri_script::ScriptError::Engine(eitri_core::Error::Cancelled)) {
          self.view.log_line(LogKind::Warn, tr!("op-cancelled", { label: label }));
        } else {
          self.view.log_line(LogKind::Error, tr!("op-failed", { label: label, reason: err.to_string() }));
        }
        // A failed/cancelled op still returned the session; the collection may carry partial edits (it does
        // not — commands commit atomically — but the refresh is cheap and keeps the invariant simple).
        self.refresh_from_session();
      }
    }
  }

  /// Auto-fill the Setup node from the FIRST geometry-bearing object that lands in the project: when no stock
  /// is set and nothing else on the tree could have bounded one, fit the stock to the new object at the
  /// drafted thickness (the Vectric-style "the job setup knows your material" first-load seed). Quietly a
  /// no-op for boundless kinds (an Excellon opened first, a CNC job) and once the operator owns the setup —
  /// an explicit stock, or a board already present when this one arrived, is never overridden. The fit folds
  /// into the import's undo entry (coalesce), so opening a board is a single Ctrl+Z (docs review #8).
  fn auto_fit_stock(&mut self, id: ObjectId) {
    let thickness = self.ui.stock_draft.thickness.max(0.01);
    let Some(session) = self.slot.session_mut() else { return };
    if session.stock().is_some() {
      return;
    }
    // Presence, not extent: whether another object COULD have bounded a stock is a kind question, so match the
    // payload cheaply instead of materializing (and cloning) every layer's geometry just to bound-and-discard it.
    let others_have_geometry = session.object_ids().into_iter().any(|other| {
      other != id
        && session
          .object(other)
          .map(|o| matches!(o.payload, ObjectPayload::Gerber(_) | ObjectPayload::Geometry(_)))
          .unwrap_or(false)
    });
    // coalesce = true folds the fit into the import's undo entry (it runs right after the import, before any other
    // edit), so opening a board undoes in one step rather than leaving the auto-fitted stock behind.
    if others_have_geometry || session.fit_stock_to(id, thickness, true).is_err() {
      return;
    }
    let (x, y, z) = session.work_origin();
    self.view.log_line(
      LogKind::Info,
      tr!("stock-set", { x: format!("{x:.3}"), y: format!("{y:.3}"), z: format!("{z:.3}") }),
    );
  }

  /// Perform one intent. Session-touching intents check the slot (their buttons are disabled while busy, so a
  /// miss here is defensive, not a UX path).
  fn handle_intent(&mut self, ctx: &egui::Context, intent: Intent) {
    match intent {
      Intent::OpenGerber => self.open_via_dialog(FileKind::Gerber),
      Intent::OpenExcellon => self.open_via_dialog(FileKind::Excellon),
      Intent::ImportSvg => self.open_via_dialog(FileKind::Svg),
      Intent::ImportDxf => self.open_via_dialog(FileKind::Dxf),
      Intent::ImportGcode => self.open_via_dialog(FileKind::Gcode),
      Intent::OpenProject => self.open_project_via_dialog(),
      Intent::SaveProject => self.save_project_via_dialog(),
      Intent::ExportGcode(id) => self.export_gcode_via_dialog(id),

      Intent::Select(id) => {
        self.view.selected = id;
        self.refresh_selection_extras();
      }
      Intent::SetVisible(id, visible) => {
        if let Some(session) = self.slot.session_mut() {
          if let Err(err) = session.set_visible(id, visible) {
            self.view.log_line(LogKind::Error, err.to_string());
          }
          self.refresh_from_session();
        }
      }
      Intent::Rename(id, name) => {
        if let Some(session) = self.slot.session_mut() {
          if let Err(err) = session.rename(id, name) {
            self.view.log_line(LogKind::Error, err.to_string());
          }
          self.refresh_from_session();
        }
      }
      Intent::DeleteObject(id) => {
        if let Some(session) = self.slot.session_mut() {
          if let Err(err) = session.delete(id) {
            self.view.log_line(LogKind::Error, err.to_string());
          }
          self.refresh_from_session();
        }
      }
      Intent::Undo => {
        if let Some(session) = self.slot.session_mut() {
          session.undo();
          self.refresh_from_session();
        }
      }
      Intent::Redo => {
        if let Some(session) = self.slot.session_mut() {
          session.redo();
          self.refresh_from_session();
        }
      }

      Intent::RunIsolate(id) => {
        let name = self.view.selected_row().map(|row| row.name.clone());
        let request =
          OpRequest::Isolate { source: id, spec: self.ui.iso.to_spec(), job: self.ui.iso.to_job(name) };
        self.launch(request);
      }
      Intent::RunDrill(id) => {
        let name = self.view.selected_row().map(|row| row.name.clone());
        let request =
          OpRequest::Drill { source: id, spec: self.ui.drill.to_spec(), job: self.ui.drill.to_job(name) };
        self.launch(request);
      }
      Intent::RunPaint(id) => {
        let name = self.view.selected_row().map(|row| row.name.clone());
        let request =
          OpRequest::Paint { source: id, spec: self.ui.paint.to_spec(), job: self.ui.paint.job.to_job(name) };
        self.launch(request);
      }
      Intent::RunNonCopper(id) => {
        // Resolve the drafted boundary HERE, while the session is home: the bbox case is pure numbers; the
        // object case clones the picked object's region (its geometry, not a computation).
        let boundary = match self.ui.noncopper.boundary {
          super::op_drafts::BoundaryChoice::BoundingBox => Some(self.ui.noncopper.bbox_boundary()),
          super::op_drafts::BoundaryChoice::Object => self.boundary_region_of(self.ui.noncopper.boundary_object),
        };
        let Some(boundary) = boundary else {
          self.view.log_line(LogKind::Error, tr!("error-boundary-object"));
          return;
        };
        let name = self.view.selected_row().map(|row| row.name.clone());
        let request = OpRequest::NonCopper {
          source: id,
          spec: self.ui.noncopper.to_spec(boundary),
          job: self.ui.noncopper.paint.job.to_job(name),
        };
        self.launch(request);
      }
      Intent::RunCutout(id) => {
        let outline = match self.ui.cutout.outline {
          super::op_drafts::OutlineChoice::Rectangle => Some(self.ui.cutout.rectangle_outline()),
          super::op_drafts::OutlineChoice::Silhouette => {
            self.payload_region(id).map(eitri_project::CutoutOutlineSpec::Geometry)
          }
        };
        let Some(outline) = outline else {
          self.view.log_line(LogKind::Error, tr!("error-outline-object"));
          return;
        };
        let name = self.view.selected_row().map(|row| row.name.clone());
        // Associate the cutout with the board it profiles (the selected object), so dragging that board carries the
        // cutout's outline along and flags it for rebuild (docs review #5).
        let request = OpRequest::Cutout {
          spec: self.ui.cutout.to_spec(outline),
          job: self.ui.cutout.job.to_job(name),
          board: Some(id),
        };
        self.launch(request);
      }
      Intent::RunPanelize(id) => {
        self.launch(OpRequest::Panelize { source: id, spec: self.ui.panelize.to_spec() });
      }
      Intent::RunMirror(id) => {
        self.launch(OpRequest::Mirror { source: id, line: self.ui.mirror.to_line() });
      }
      Intent::ExportFilm(id) => self.export_film_via_dialog(id),
      Intent::RebuildJob(job) => self.launch(OpRequest::Rebuild { job }),
      Intent::SetStock { stock, coalesce } => {
        if let Some(session) = self.slot.session_mut() {
          session.set_stock(Some(stock), coalesce);
          self.refresh_after_setup_change();
          // Deliberately NOT logged: a DragValue re-commits on every frame of a drag, and a log line per frame
          // would flood the dock. The readout, the canvas block, and the crosshair are the live confirmation;
          // the discrete setup events (fit, clear, auto-fit) do log.
        }
      }
      Intent::ClearStock => {
        if let Some(session) = self.slot.session_mut() {
          session.set_stock(None, false);
          self.refresh_after_setup_change();
          self.view.log_line(LogKind::Ok, tr!("stock-cleared"));
        }
      }
      Intent::FitStock { reference, thickness } => {
        if let Some(session) = self.slot.session_mut() {
          // A manual Fit press is its own undo entry (coalesce = false).
          match session.fit_stock_to(reference, thickness, false) {
            Ok(()) => {
              self.refresh_after_setup_change();
              let [x, y, z] = self.ui.work_origin;
              self.view.log_line(
                LogKind::Ok,
                tr!("stock-set", { x: format!("{x:.3}"), y: format!("{y:.3}"), z: format!("{z:.3}") }),
              );
            }
            Err(err) => self.view.log_line(LogKind::Error, err.to_string()),
          }
        }
      }
      Intent::CancelOp => {
        if let Some(op) = self.slot.running() {
          op.cancel();
        }
      }

      Intent::ZoomFit => self.ui.pending_fit = true,
      Intent::TranslateGroup { anchor, dx, dy, new_edit } => {
        if let Some(session) = self.slot.session_mut() {
          match session.move_group(anchor, dx, dy, new_edit) {
            // Fast-path: shift only the moved objects' cached scene geometry so the board follows the cursor,
            // WITHOUT re-triangulating every polygon or re-importing every job's G-code each frame (docs #1). The
            // full `refresh_from_session` runs once on `Intent::EndTranslate` when the drag stops.
            Ok(()) => {
              let members = session.move_set(anchor);
              self.scene.translate_objects(&members, dx, dy);
            }
            Err(err) => self.view.log_line(LogKind::Error, err.to_string()),
          }
        }
      }
      Intent::EndTranslate => self.refresh_from_session(),

      Intent::OpenAppSettings => self.ui.app_settings_open = true,
      Intent::SetLanguage(locale) => {
        crate::i18n::set_language(&locale);
        self.config.ui.language = locale;
        self.config_dirty = true;
      }
      Intent::SetActiveTheme(name) => {
        self.config.appearance.active_theme = name;
        self.config_dirty = true;
        self.reskin(ctx);
      }
      Intent::SetFontScale(scale) => {
        self.config.appearance.font_scale = crate::config::clamp_font_scale(scale);
        self.config_dirty = true;
        self.reskin(ctx);
      }
      Intent::UpsertTheme { name, theme } => {
        self.config.appearance.themes.insert(name, theme);
        self.config_dirty = true;
        self.reskin(ctx);
      }
      Intent::SaveConfig => match crate::config::save(&self.config) {
        Ok(()) => {
          self.config_dirty = false;
          if let Some(path) = crate::config::config_path() {
            self.view.log_line(LogKind::Ok, tr!("project-saved", { path: path.display().to_string() }));
          }
        }
        Err(reason) => self.view.log_line(LogKind::Error, reason),
      },

      Intent::OpenToolDb => self.ui.tool_db_open = true,
      Intent::AddTool => {
        let id = self.tool_db.add(super::tool_db::default_entry(tr!("tool-db-new-name")));
        self.ui.tool_db_selected = Some(id);
        self.tool_db_dirty = true;
        self.refresh_tool_list();
      }
      Intent::UpdateTool(id, entry) => {
        self.tool_db.update(id, entry);
        self.tool_db_dirty = true;
        self.refresh_tool_list();
      }
      Intent::RemoveTool(id) => {
        self.tool_db.remove(id);
        if self.ui.tool_db_selected == Some(id) {
          self.ui.tool_db_selected = None;
        }
        self.tool_db_dirty = true;
        self.refresh_tool_list();
      }
      Intent::SaveToolDb => match crate::tool_store::save(&self.tool_db) {
        Ok(()) => {
          self.tool_db_dirty = false;
          if let Some(path) = crate::tool_store::path() {
            self.view.log_line(LogKind::Ok, tr!("export-done", { path: path.display().to_string() }));
          }
        }
        Err(reason) => self.view.log_line(LogKind::Error, reason),
      },
      Intent::SeedIsolationFromTool(id) => {
        if let Some(spec) = self.tool_db.get(id).map(|tool| tool.isolation_spec()) {
          self.ui.iso.tool_diameter = spec.tool_diameter;
          self.ui.iso.passes = spec.passes;
          self.ui.iso.overlap = spec.overlap;
          self.ui.iso.combine = spec.combine;
          self.ui.iso.climb = spec.direction == DirectionSpec::Climb;
        }
      }
      Intent::SeedDrillFromTool(id) => {
        if let Some(spec) = self.tool_db.get(id).map(|tool| tool.drill_spec()) {
          self.ui.drill.depth = spec.depth;
          self.ui.drill.feed = spec.feed;
          self.ui.drill.retract = spec.retract;
        }
      }
    }
  }

  /// Re-resolve the appearance from the (edited) config onto the live context and the view style.
  fn reskin(&mut self, ctx: &egui::Context) {
    apply_appearance(&mut self.ui, &self.config);
    apply_theme(ctx, &self.ui.style.palette, self.config.appearance.font_scale);
  }

  /// Pick a file of `kind` and spawn its open/import op. The file is read HERE so a read error is reported
  /// immediately with its path; the worker gets the contents.
  fn open_via_dialog(&mut self, kind: FileKind) {
    let Some(path) = kind.dialog().pick_file() else { return };
    match std::fs::read_to_string(&path) {
      Ok(source) => {
        let name = file_stem(&path);
        self.launch(kind.request(name, source));
      }
      Err(err) => self.view.log_line(
        LogKind::Error,
        tr!("error-open-file", { path: path.display().to_string(), reason: err.to_string() }),
      ),
    }
  }

  /// Pick a project file and spawn the load op (the parse + geometry hydration runs off-thread).
  fn open_project_via_dialog(&mut self) {
    let dialog = rfd::FileDialog::new().add_filter(tr!("file-filter-project"), &["json"]);
    let Some(path) = dialog.pick_file() else { return };
    match std::fs::read_to_string(&path) {
      Ok(json) => {
        self.view.log_line(LogKind::Info, tr!("project-loaded", { path: path.display().to_string() }));
        self.launch(OpRequest::LoadProject { json });
      }
      Err(err) => self.view.log_line(
        LogKind::Error,
        tr!("error-open-file", { path: path.display().to_string(), reason: err.to_string() }),
      ),
    }
  }

  /// Pick a destination and write the project JSON (serialisation is quick; no worker needed).
  fn save_project_via_dialog(&mut self) {
    let Some(session) = self.slot.session() else { return };
    let json = match session.save_project() {
      Ok(json) => json,
      Err(err) => {
        self.view.log_line(LogKind::Error, err.to_string());
        return;
      }
    };
    let dialog = rfd::FileDialog::new()
      .add_filter(tr!("file-filter-project"), &["json"])
      .set_file_name(format!("{}.eitri.json", session.name()));
    let Some(path) = dialog.save_file() else { return };
    match std::fs::write(&path, json) {
      Ok(()) => self.view.log_line(LogKind::Ok, tr!("project-saved", { path: path.display().to_string() })),
      Err(err) => self.view.log_line(
        LogKind::Error,
        tr!("error-save-file", { path: path.display().to_string(), reason: err.to_string() }),
      ),
    }
  }

  /// The copper/geometry region an object's payload already carries, cloned for a spec that owns its outline.
  /// Data access only — Gerber copper and geometry polygons are engine outputs; nothing is computed here.
  fn payload_region(&self, id: ObjectId) -> Option<geo_types::MultiPolygon<f64>> {
    let session = self.slot.session()?;
    match &session.object(id).ok()?.payload {
      ObjectPayload::Gerber(gerber) => gerber.image.as_ref().map(|image| image.copper.clone()),
      ObjectPayload::Geometry(geometry) => Some(geo_types::MultiPolygon::new(geometry.polygons.clone())),
      _ => None,
    }
  }

  /// The region of the picked non-copper boundary object, as an owned [`eitri_project::BoundarySpec`].
  fn boundary_region_of(&self, picked: Option<ObjectId>) -> Option<eitri_project::BoundarySpec> {
    let region = self.payload_region(picked?)?;
    if region.0.is_empty() {
      return None;
    }
    Some(eitri_project::BoundarySpec::Region(region))
  }

  /// Render the selected copper/geometry as a film SVG (quick, on the UI thread like the other exports) and
  /// pick a destination for it.
  fn export_film_via_dialog(&mut self, id: ObjectId) {
    let Some(session) = self.slot.session() else { return };
    let svg = match session.film_svg(id, &self.ui.film.to_params()) {
      Ok(svg) => svg,
      Err(err) => {
        self.view.log_line(LogKind::Error, err.to_string());
        return;
      }
    };
    let name = session.object_name(id).unwrap_or_else(|_| "film".to_string());
    let dialog = rfd::FileDialog::new()
      .add_filter(tr!("file-filter-svg"), &["svg"])
      .set_file_name(format!("{name}-film.svg"));
    let Some(path) = dialog.save_file() else { return };
    match std::fs::write(&path, svg) {
      Ok(()) => self.view.log_line(LogKind::Ok, tr!("export-done", { path: path.display().to_string() })),
      Err(err) => self.view.log_line(
        LogKind::Error,
        tr!("error-save-file", { path: path.display().to_string(), reason: err.to_string() }),
      ),
    }
  }

  /// Pick a destination and write a CNC job's G-code.
  fn export_gcode_via_dialog(&mut self, id: ObjectId) {
    let Some(session) = self.slot.session() else { return };
    let text = match session.write_gcode(id) {
      Ok(text) => text,
      Err(err) => {
        self.view.log_line(LogKind::Error, err.to_string());
        return;
      }
    };
    let name = session.object_name(id).unwrap_or_else(|_| "job".to_string());
    let dialog = rfd::FileDialog::new()
      .add_filter(tr!("file-filter-gcode"), &["nc", "ngc", "gcode", "tap"])
      .set_file_name(format!("{name}.nc"));
    let Some(path) = dialog.save_file() else { return };
    match std::fs::write(&path, text) {
      Ok(()) => self.view.log_line(LogKind::Ok, tr!("export-done", { path: path.display().to_string() })),
      Err(err) => self.view.log_line(
        LogKind::Error,
        tr!("error-save-file", { path: path.display().to_string(), reason: err.to_string() }),
      ),
    }
  }
}

/// Partition a session's collection into the PROJECT tree rows, the TOOLPATHS rows, and the group folders. CNC jobs
/// go to TOOLPATHS, every source/geometry object to PROJECT; both keep the collection's display order. This is the
/// single source of truth for the split so the snapshot/tutorial harnesses cannot drift from the live shell (docs
/// cleanup: the loop previously lived in three places).
pub(crate) fn partition_from_session(session: &Session) -> (Vec<TreeRow>, Vec<TreeRow>, Vec<GroupView>) {
  let (mut tree, mut toolpaths): (Vec<TreeRow>, Vec<TreeRow>) = (Vec::new(), Vec::new());
  for id in session.object_ids() {
    let Ok(object) = session.object(id) else { continue };
    let kind = object.kind();
    let stale = matches!(&object.payload, ObjectPayload::CncJob(job) if job.stale);
    let row = TreeRow { id, name: object.meta.name.clone(), kind, visible: object.meta.visible, stale };
    if kind == ObjectKind::CncJob {
      toolpaths.push(row);
    } else {
      tree.push(row);
    }
  }
  let groups = session.groups().into_iter().map(|(name, members)| GroupView { name, members }).collect();
  (tree, toolpaths, groups)
}

/// Fold one engine progress event into the view state.
fn apply_progress(view: &mut ViewState, event: ProgressEvent) {
  match event {
    ProgressEvent::Advanced { done, total } => view.op_advanced(done, total),
    ProgressEvent::Message(text) => view.log_line(LogKind::Info, text),
    // Start/finish are owned by the shell's own lifecycle (the label is already set, and the outcome — not the
    // event stream — ends the op), so the engine's markers add nothing here.
    ProgressEvent::Started { .. } | ProgressEvent::Finished => {}
  }
}

/// The pickable input-file kinds and their dialog filters/requests.
#[derive(Clone, Copy)]
enum FileKind {
  Gerber,
  Excellon,
  Svg,
  Dxf,
  Gcode,
}

impl FileKind {
  /// The rfd dialog for this kind, filtered to its conventional extensions (plus the inevitable `.txt`
  /// Excellon exports some EDA tools produce).
  fn dialog(self) -> rfd::FileDialog {
    match self {
      FileKind::Gerber => rfd::FileDialog::new()
        .add_filter(tr!("file-filter-gerber"), &["gbr", "ger", "gtl", "gbl", "gts", "gbs", "gto", "gbo", "gko", "gm1", "pho"]),
      FileKind::Excellon => rfd::FileDialog::new().add_filter(tr!("file-filter-excellon"), &["drl", "xln", "exc", "txt"]),
      FileKind::Svg => rfd::FileDialog::new().add_filter(tr!("file-filter-svg"), &["svg"]),
      FileKind::Dxf => rfd::FileDialog::new().add_filter(tr!("file-filter-dxf"), &["dxf"]),
      FileKind::Gcode => rfd::FileDialog::new().add_filter(tr!("file-filter-gcode"), &["nc", "ngc", "gcode", "tap"]),
    }
  }

  /// The worker request for this kind.
  fn request(self, name: String, source: String) -> OpRequest {
    match self {
      FileKind::Gerber => OpRequest::OpenGerber { name, source },
      FileKind::Excellon => OpRequest::OpenExcellon { name, source },
      FileKind::Svg => OpRequest::ImportSvg { name, source },
      FileKind::Dxf => OpRequest::ImportDxf { name, source },
      FileKind::Gcode => OpRequest::ImportGcode { name, source },
    }
  }
}

/// A display name from a picked path's stem.
fn file_stem(path: &Path) -> String {
  path.file_stem().and_then(|s| s.to_str()).map(str::to_string).unwrap_or_else(|| path.display().to_string())
}

impl eframe::App for EitriApp {
  fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
    // 1. Pump the worker into the view state before drawing, so the frame reflects the latest progress.
    self.pump();

    // 2. Build the frame; views push intents into the per-frame sink.
    let mut sink = IntentSink::new();
    shell_panels(ui, &self.scene, &self.view, &mut self.ui, &mut self.split, &mut sink);
    if self.ui.app_settings_open {
      super::app_settings::window(ui.ctx(), &mut self.ui, &self.config, self.config_dirty, &mut sink);
    }
    if self.ui.tool_db_open {
      super::tool_db::window(ui.ctx(), &mut self.ui, &self.tool_db, self.tool_db_dirty, &mut sink);
    }

    // 3. Act on the intents after layout, so a view never mutates app state mid-render.
    let ctx = ui.ctx().clone();
    for intent in sink.drain() {
      self.handle_intent(&ctx, intent);
    }

    // 4. While an op runs, wake regularly so progress animates without input events.
    if self.view.busy() {
      ctx.request_repaint_after(std::time::Duration::from_millis(100));
    }
  }
}

/// Lay out the ENTIRE window-panel arrangement — toolbar, status bar, the fixed left/right columns, and the
/// central canvas/dock split — into the window's root `Ui`. This is THE single description of the app's frame:
/// `EitriApp::ui` calls it with live state and the test harness calls it with fixtures, so the two can never
/// drift.
pub(crate) fn shell_panels(ui: &mut egui::Ui, scene: &RenderScene, view: &ViewState, state: &mut UiState,
  split: &mut CentralSplit, sink: &mut IntentSink) {
  let palette = state.style.palette;

  // The toolbar is a fixed 40px bar on the `panelAlt` surface, distinct chrome over the body.
  egui::Panel::top("toolbar").exact_size(Metrics::TOOLBAR_H).frame(egui::Frame::NONE.fill(palette.panel_alt)).show(
    ui,
    |ui| {
      views::toolbar(ui, view, state, sink);
    },
  );

  // The status bar is a fixed 24px strip.
  egui::Panel::bottom("status")
    .exact_size(Metrics::STATUS_BAR_H)
    .frame(egui::Frame::NONE.fill(palette.panel_alt))
    .show(ui, |ui| {
      views::status_bar(ui, view, state);
    });

  // The fixed body grid: project tree left, parameters right, canvas + dock centre. Zero-inner-margin frames —
  // the views own their padding (the skirnir lesson: the default side-panel margin shrinks the usable column
  // and clips the content's right edge).
  let column_frame = egui::Frame::NONE.fill(palette.panel);
  egui::Panel::left("tree").resizable(false).exact_size(Metrics::LEFT_COL_W).frame(column_frame).show(ui, |ui| {
    // The left column stacks PROJECT over TOOLPATHS: the CNC jobs get their own resizable section anchored to the
    // bottom, and the project tree fills the remainder above it.
    egui::Panel::bottom("toolpaths")
      .resizable(true)
      .default_size(Metrics::TOOLPATHS_PANEL_H)
      .min_size(Metrics::PANEL_CONTROL_H + 24.0)
      .frame(egui::Frame::NONE)
      .show(ui, |ui| {
        views::contained(ui, |ui| views::toolpaths_panel(ui, view, state, sink));
      });
    egui::CentralPanel::default().frame(egui::Frame::NONE).show(ui, |ui| {
      views::contained(ui, |ui| views::tree_panel(ui, view, state, sink));
    });
  });
  egui::Panel::right("params").resizable(false).exact_size(Metrics::RIGHT_COL_W).frame(column_frame).show(
    ui,
    |ui| {
      views::contained(ui, |ui| views::params_panel(ui, view, state, sink));
    },
  );
  egui::CentralPanel::default().frame(egui::Frame::NONE.fill(palette.bg)).show(ui, |ui| {
    split.ui(ui, scene, view, state, sink);
  });
}

/// Launch the window: load the config, bring up i18n, apply fonts + theme, and run the app.
pub fn run() -> eframe::Result<()> {
  let (config, mut notices) = crate::config::load();

  // Bring the i18n registry up before the first frame: seed the bundled locales, then select the configured
  // one. An init failure means a BUNDLED resource is invalid (a build problem) — surfaced as a log notice,
  // never a panic; `tr!` then renders keys verbatim rather than bringing the UI down.
  if let Err(err) = crate::i18n::init() {
    notices.push(format!("i18n initialisation failed (UI strings will show as keys): {err}"));
  }
  crate::i18n::set_language(&config.ui.language);

  let (palette, _palette_notice) = config.palette();
  let (window_w, window_h) = config.ui.window_size();
  let options = eframe::NativeOptions {
    viewport: egui::ViewportBuilder::default()
      .with_inner_size([window_w, window_h])
      .with_min_inner_size([900.0, 560.0])
      .with_title(crate::tr!("app-title")),
    ..Default::default()
  };
  let font_scale = config.appearance.font_scale;
  eframe::run_native(
    "eitri",
    options,
    Box::new(move |cc| {
      // Fonts before theme, so the first frame already renders in the design's typefaces.
      super::fonts::install(&cc.egui_ctx);
      apply_theme(&cc.egui_ctx, &palette, font_scale);
      Ok(Box::new(EitriApp::new(config, notices)))
    }),
  )
}

/// Apply the config's resolved APPEARANCE onto a [`UiState`]: the palette (active theme) + the canvas render
/// style. Pure (no egui context) so the config→style mapping is unit-tested without a window.
fn apply_appearance(ui: &mut UiState, config: &Config) {
  let (palette, _notice) = config.palette();
  ui.style = RuntimeStyle { palette, canvas: config.canvas_style() };
}

/// Skin the egui context from a [`Palette`]: spacing to the design's component sheet, then the full widget
/// visual set. Mirrored from skirnir's `apply_theme`, so the two apps' controls read identically.
pub(crate) fn apply_theme(ctx: &egui::Context, palette: &Palette, font_scale: f32) {
  use eframe::egui::{CornerRadius, Stroke};

  let mut style = (*ctx.global_style()).clone();
  let spacing = &mut style.spacing;
  spacing.button_padding = Metrics::BUTTON_PAD;
  spacing.item_spacing = egui::vec2(6.0, 6.0);
  spacing.interact_size.y = Metrics::PANEL_CONTROL_H;
  ctx.set_global_style(style);
  ctx.set_zoom_factor(crate::config::clamp_font_scale(font_scale));

  let mut visuals = egui::Visuals::dark();
  visuals.panel_fill = palette.panel;
  visuals.window_fill = palette.bg;
  visuals.extreme_bg_color = palette.inset;
  visuals.faint_bg_color = palette.panel_alt;
  visuals.override_text_color = Some(palette.text);
  visuals.hyperlink_color = palette.accent;
  visuals.selection.bg_fill = palette.accent.gamma_multiply(0.4);
  visuals.selection.stroke = Stroke::new(1.0, palette.accent);
  visuals.window_stroke = Stroke::new(1.0, palette.divider);

  let radius = CornerRadius::same(Metrics::CONTROL_RADIUS);
  let widgets = &mut visuals.widgets;
  widgets.noninteractive.bg_fill = palette.panel;
  widgets.noninteractive.weak_bg_fill = palette.panel;
  widgets.noninteractive.bg_stroke = Stroke::new(1.0, palette.divider);
  widgets.noninteractive.fg_stroke = Stroke::new(1.0, palette.text_dim);
  widgets.noninteractive.corner_radius = radius;
  widgets.inactive.bg_fill = palette.widget;
  widgets.inactive.weak_bg_fill = palette.widget;
  widgets.inactive.bg_stroke = Stroke::new(1.0, palette.border_raised);
  widgets.inactive.fg_stroke = Stroke::new(1.0, palette.text);
  widgets.inactive.corner_radius = radius;
  widgets.hovered.bg_fill = palette.widget_hover;
  widgets.hovered.weak_bg_fill = palette.widget_hover;
  widgets.hovered.bg_stroke = Stroke::new(1.0, palette.accent_hover);
  widgets.hovered.fg_stroke = Stroke::new(1.0, palette.text);
  widgets.hovered.corner_radius = radius;
  widgets.active.bg_fill = palette.widget_active;
  widgets.active.weak_bg_fill = palette.widget_active;
  widgets.active.bg_stroke = Stroke::new(1.0, palette.accent_active);
  widgets.active.fg_stroke = Stroke::new(1.0, palette.text);
  widgets.active.corner_radius = radius;
  widgets.open.bg_fill = palette.widget_active;
  widgets.open.weak_bg_fill = palette.widget_active;
  widgets.open.bg_stroke = Stroke::new(1.0, palette.border_raised);
  widgets.open.fg_stroke = Stroke::new(1.0, palette.text);
  widgets.open.corner_radius = radius;

  ctx.set_visuals(visuals);
}

#[cfg(test)]
mod tests {
  use super::*;
  use super::super::view_state::OpView;

  const GERBER: &str = include_str!("../../../../fixtures/synthetic/gerber/kicad_two_pads.gbr");

  /// An app with a session already carrying the fixture Gerber — no window, no dialogs.
  fn app_with_gerber() -> (EitriApp, ObjectId) {
    let mut app = EitriApp::new(Config::default(), Vec::new());
    let id = {
      let session = app.slot.session_mut().expect("home at start");
      session.open_gerber_str("fixture-top", GERBER).expect("fixture opens")
    };
    app.refresh_from_session();
    (app, id)
  }

  /// Pump until the worker returns, bounded.
  fn pump_until_idle(app: &mut EitriApp) {
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);
    while app.view.busy() {
      app.pump();
      assert!(std::time::Instant::now() < deadline, "the op never finished");
      std::thread::sleep(std::time::Duration::from_millis(2));
    }
  }

  #[test]
  fn a_fresh_app_snapshots_an_empty_tree_and_logs_config_notices() {
    let app = EitriApp::new(Config::default(), vec!["fixture notice".to_string()]);
    assert!(app.view.tree.is_empty());
    assert!(!app.view.busy());
    assert!(app.view.log.iter().any(|l| l.text == "fixture notice"), "load notices must reach the log");
  }

  #[test]
  fn imported_layers_auto_join_one_import_group() {
    // Imports arrive one file at a time; each auto-joins the managed import group so the board's layers stay
    // registered and move together. Two opens must land in ONE group, not two.
    let mut app = EitriApp::new(Config::default(), Vec::new());
    app.launch(OpRequest::OpenGerber { name: "top".to_string(), source: GERBER.to_string() });
    pump_until_idle(&mut app);
    app.launch(OpRequest::OpenGerber { name: "bottom".to_string(), source: GERBER.to_string() });
    pump_until_idle(&mut app);
    assert_eq!(app.view.groups.len(), 1, "exactly one managed import group: {:?}", app.view.groups);
    assert_eq!(app.view.groups[0].members.len(), 2, "both imported layers joined it");
    assert_eq!(app.view.tree.len(), 2, "both sources are in PROJECT");
  }

  #[test]
  fn opening_a_board_is_a_single_undo_step_even_with_auto_fit() {
    // Opening a board auto-fits the stock; that fit folds into the import's undo entry, so one Ctrl+Z removes the
    // whole board rather than first reverting only the auto-fitted stock (docs review #8).
    let mut app = EitriApp::new(Config::default(), Vec::new());
    app.launch(OpRequest::OpenGerber { name: "top".to_string(), source: GERBER.to_string() });
    pump_until_idle(&mut app);
    assert_eq!(app.view.tree.len(), 1, "the board loaded");
    assert!(app.slot.session().unwrap().stock().is_some(), "and its stock was auto-fitted");

    let undone = app.slot.session_mut().unwrap().undo();
    assert!(undone, "there is one undoable step");
    let session = app.slot.session().unwrap();
    assert_eq!(session.len(), 0, "one undo removes the whole board, not just the auto-fit stock");
    assert!(session.stock().is_none(), "and the stock is gone with it");
  }

  #[test]
  fn moving_a_source_flags_its_toolpath_stale() {
    let (mut app, id) = app_with_gerber();
    let ctx = egui::Context::default();
    app.handle_intent(&ctx, Intent::RunIsolate(id));
    pump_until_idle(&mut app);
    assert_eq!(app.view.toolpaths.len(), 1, "the isolate produced a toolpath");
    assert!(!app.view.toolpaths[0].stale, "a freshly posted toolpath is up to date");

    app.handle_intent(&ctx, Intent::TranslateGroup { anchor: id, dx: 10.0, dy: 0.0, new_edit: true });
    // The source moves on the stock immediately (the session is staled per frame); the VIEW's stale badge, however,
    // refreshes once at drag end — the per-frame path only shifts the cached scene (docs #1).
    assert!(
      app.slot.session().unwrap().object(id).unwrap().meta.placement != eitri_core::Affine::IDENTITY,
      "the dragged source's placement changed",
    );
    app.handle_intent(&ctx, Intent::EndTranslate);
    assert!(app.view.toolpaths[0].stale, "at drag end the toolpath row shows stale for rebuild");
  }

  #[test]
  fn changing_the_datum_flags_the_toolpath_row_stale_live() {
    // #3: a Setup datum/stock change stales every posted job, and the setup handler refreshes the tree badge live
    // (without a scene rebuild). Fitting the stock from the native frame is a real datum change.
    let (mut app, id) = app_with_gerber();
    let ctx = egui::Context::default();
    app.handle_intent(&ctx, Intent::RunIsolate(id));
    pump_until_idle(&mut app);
    assert!(!app.view.toolpaths[0].stale, "a freshly posted toolpath is up to date");

    app.handle_intent(&ctx, Intent::FitStock { reference: id, thickness: 1.6 });
    assert!(app.view.toolpaths[0].stale, "changing the work-zero flags the toolpath stale, shown live");
    assert!(app.view.can_undo, "the datum change is undoable");
  }

  #[test]
  fn rebuilding_a_stale_toolpath_refreshes_it_in_place_and_clears_stale() {
    let (mut app, id) = app_with_gerber();
    let ctx = egui::Context::default();
    app.handle_intent(&ctx, Intent::RunIsolate(id));
    pump_until_idle(&mut app);
    let job = app.view.toolpaths[0].id;
    app.handle_intent(&ctx, Intent::TranslateGroup { anchor: id, dx: 10.0, dy: 0.0, new_edit: true });
    app.handle_intent(&ctx, Intent::EndTranslate);
    assert!(app.view.toolpaths[0].stale, "the moved source staled the job");

    app.handle_intent(&ctx, Intent::RebuildJob(job));
    pump_until_idle(&mut app);
    assert_eq!(app.view.toolpaths.len(), 1, "rebuild is in place — no second job appears");
    assert_eq!(app.view.toolpaths[0].id, job, "the rebuilt job keeps its id");
    assert!(!app.view.toolpaths[0].stale, "rebuild clears the stale flag");
  }

  #[test]
  fn selecting_a_toolpath_exposes_its_tool_diameter() {
    let (mut app, id) = app_with_gerber();
    let ctx = egui::Context::default();
    app.handle_intent(&ctx, Intent::RunIsolate(id));
    pump_until_idle(&mut app);
    let job = app.view.toolpaths[0].id;
    app.handle_intent(&ctx, Intent::Select(Some(Selection::Object(job))));
    let info = app.ui.selected_info.as_ref().expect("the job's facts are snapshotted on selection");
    assert!(info.tool_diameter.is_some_and(|d| d > 0.0), "the panel exposes the isolation tool diameter");
  }

  #[test]
  fn refresh_snapshots_tree_rows_scene_and_history_flags() {
    let (app, id) = app_with_gerber();
    assert_eq!(app.view.tree.len(), 1);
    assert_eq!(app.view.tree[0].id, id);
    assert_eq!(app.view.tree[0].name, "fixture-top");
    assert!(app.view.can_undo, "the open is an undoable edit");
    assert!(!app.view.can_redo);
    assert_eq!(app.scene.objects.len(), 1, "the scene rebuilds with the tree");
  }

  #[test]
  fn run_isolate_goes_busy_finishes_selects_the_job_and_requests_a_fit() {
    let (mut app, id) = app_with_gerber();
    // Simulate the empty→loaded auto-fit already consumed; the op path sets it again only from empty.
    app.ui.pending_fit = false;
    app.view.selected = Some(Selection::Object(id));
    let ctx = egui::Context::default();
    app.handle_intent(&ctx, Intent::RunIsolate(id));
    assert!(app.view.busy(), "the isolate runs off-thread");
    assert!(matches!(app.view.op, OpView::Running { .. }));

    pump_until_idle(&mut app);
    assert_eq!(app.view.tree.len(), 1, "the source stays in PROJECT");
    assert_eq!(app.view.toolpaths.len(), 1, "the job landed in the TOOLPATHS panel, not PROJECT");
    let job = app.view.selected_object().expect("the new job is selected");
    assert_eq!(
      app.view.toolpaths.iter().find(|r| r.id == job).map(|r| r.kind),
      Some(eitri_project::ObjectKind::CncJob),
    );
    assert!(app.ui.selected_info.as_ref().is_some_and(|i| i.gcode_lines > 0), "the job facts are snapshotted");
    assert!(!app.ui.gcode_preview.is_empty(), "the dock preview is populated");
    assert!(
      app.view.log.iter().any(|l| l.kind == LogKind::Ok),
      "a finished op logs its completion: {:?}",
      app.view.log
    );
  }

  #[test]
  fn set_visible_and_rename_intents_round_trip_through_the_session() {
    let (mut app, id) = app_with_gerber();
    let ctx = egui::Context::default();

    app.handle_intent(&ctx, Intent::SetVisible(id, false));
    assert!(!app.view.tree[0].visible, "the tree snapshot reflects the hide");
    assert_eq!(app.scene.object(id).map(|o| o.visible), Some(false), "the scene entry reflects it too");
    assert_eq!(app.scene.bounds, None, "the only object is hidden, so there is nothing to fit");

    app.handle_intent(&ctx, Intent::Rename(id, "fixture-renamed".to_string()));
    assert_eq!(app.view.tree[0].name, "fixture-renamed", "the rename lands in the snapshot");

    // A rename that collides with an existing name is refused by the engine and LOGGED, never silent.
    let second = {
      let session = app.slot.session_mut().expect("home");
      session.open_gerber_str("fixture-other", GERBER).expect("second fixture opens")
    };
    app.refresh_from_session();
    app.handle_intent(&ctx, Intent::Rename(second, "fixture-renamed".to_string()));
    assert!(
      app.view.log.iter().any(|l| l.kind == LogKind::Error),
      "a name collision must surface in the log: {:?}",
      app.view.log
    );
  }

  #[test]
  fn delete_and_undo_round_trip_through_intents() {
    let (mut app, id) = app_with_gerber();
    let ctx = egui::Context::default();
    app.handle_intent(&ctx, Intent::Select(Some(Selection::Object(id))));
    app.handle_intent(&ctx, Intent::DeleteObject(id));
    assert!(app.view.tree.is_empty(), "the delete refreshes the tree");
    assert_eq!(app.view.selected, None, "the vanished selection clears");
    app.handle_intent(&ctx, Intent::Undo);
    assert_eq!(app.view.tree.len(), 1, "undo restores the object");
    assert!(app.view.can_redo);
  }

  #[test]
  fn selecting_a_job_populates_the_gcode_preview_and_deselecting_clears_it() {
    let (mut app, id) = app_with_gerber();
    let job = {
      let session = app.slot.session_mut().expect("home");
      let spec = eitri_project::IsolationSpec {
        tool_diameter: 0.2,
        passes: 1,
        overlap: 0.0,
        combine: false,
        direction: eitri_project::DirectionSpec::Climb,
      };
      session.isolate(id, spec, eitri_gcode::IsolationJob::default()).expect("isolates")
    };
    app.refresh_from_session();
    let ctx = egui::Context::default();
    app.handle_intent(&ctx, Intent::Select(Some(Selection::Object(job))));
    assert!(!app.ui.gcode_preview.is_empty(), "a selected job previews its G-code");
    app.handle_intent(&ctx, Intent::Select(None));
    assert!(app.ui.gcode_preview.is_empty(), "deselecting clears the preview");
  }

  #[test]
  fn appearance_intents_mutate_the_config_mark_it_dirty_and_reskin_the_style() {
    let (mut app, _) = app_with_gerber();
    let ctx = egui::Context::default();
    assert!(!app.config_dirty);
    app.handle_intent(&ctx, Intent::SetActiveTheme("midnight".to_string()));
    assert!(app.config_dirty, "a theme change is an unsaved edit");
    assert_eq!(app.ui.style.palette, Palette::midnight(), "the view style re-resolves immediately");
    app.handle_intent(&ctx, Intent::SetFontScale(9.0));
    assert!(
      (app.config.appearance.font_scale - *crate::config::FONT_SCALE_RANGE.end()).abs() < 1e-6,
      "an out-of-range scale clamps"
    );
  }

  #[test]
  fn set_stock_intents_commit_sync_the_snapshots_and_clear_through_the_session() {
    let (mut app, _) = app_with_gerber();
    let ctx = egui::Context::default();
    assert_eq!(app.ui.work_origin, [0.0, 0.0, 0.0], "a fresh session is in the native frame");
    assert_eq!(app.ui.stock, None);

    // Committing a stock derives the work zero from its datum corner and Z face, and mirrors it into the
    // draft so the panel shows the numbers in force.
    let stock = eitri_project::Stock {
      min_x: 2.0,
      min_y: 3.0,
      size_x: 20.0,
      size_y: 10.0,
      thickness: 1.6,
      z_ref: eitri_project::ZReference::Bottom,
      datum: eitri_project::DatumCorner::BottomRight,
    };
    app.handle_intent(&ctx, Intent::SetStock { stock, coalesce: false });
    assert_eq!(app.ui.stock, Some(stock), "the committed stock is snapshotted for the views");
    assert_eq!(app.ui.work_origin, [22.0, 3.0, -1.6], "bottom-right/bottom resolves the 3-D work zero");
    assert_eq!(app.ui.stock_draft.datum, eitri_project::DatumCorner::BottomRight, "the draft mirrors it");
    // A routine commit is deliberately SILENT (a DragValue re-commits every frame of a drag — logging each
    // would flood the dock); only errors and the discrete fit/clear events write log lines.
    assert!(!app.view.log.iter().any(|l| l.kind == LogKind::Error), "a valid commit must not error");

    // Clearing reverts to the native frame but keeps the draft numbers for a cheap re-commit.
    app.handle_intent(&ctx, Intent::ClearStock);
    assert_eq!(app.ui.stock, None);
    assert_eq!(app.ui.work_origin, [0.0, 0.0, 0.0]);
    assert!((app.ui.stock_draft.size_x - 20.0).abs() < 1e-9, "a clear keeps what the operator typed");
  }

  #[test]
  fn fit_stock_sizes_the_footprint_to_the_reference_and_an_invalid_reference_logs() {
    let (mut app, id) = app_with_gerber();
    let ctx = egui::Context::default();
    let bounds = app.scene.object(id).and_then(|o| o.bounds).expect("the fixture copper has bounds");

    app.handle_intent(&ctx, Intent::FitStock { reference: id, thickness: 1.6 });
    let stock = app.ui.stock.expect("the fit committed a stock");
    assert_eq!(stock.footprint(), bounds, "the footprint is the copper's bounding box");
    assert_eq!(app.ui.work_origin[0], bounds.0, "a fresh fit is bottom-left, so work X0 is the min corner");
    assert!((app.ui.stock_draft.size_x - stock.size_x).abs() < 1e-9, "the draft mirrors the fit");

    // A CNC job has no geometry to bound — the engine refuses and the shell surfaces that in the log rather
    // than silently keeping a stale success state.
    let job = {
      let session = app.slot.session_mut().expect("home");
      let spec = eitri_project::IsolationSpec {
        tool_diameter: 0.2,
        passes: 1,
        overlap: 0.0,
        combine: false,
        direction: eitri_project::DirectionSpec::Climb,
      };
      session.isolate(id, spec, eitri_gcode::IsolationJob::default()).expect("isolates")
    };
    app.refresh_from_session();
    app.handle_intent(&ctx, Intent::FitStock { reference: job, thickness: 1.6 });
    assert!(
      app.view.log.iter().any(|l| l.kind == LogKind::Error),
      "an invalid reference must be reported: {:?}",
      app.view.log
    );
    assert_eq!(app.ui.stock, Some(stock), "a refused fit leaves the committed stock unchanged");
  }

  #[test]
  fn the_first_loaded_board_auto_fits_the_stock_and_later_boards_do_not() {
    // The Vectric-style seed: opening the FIRST board into an empty project fills the Setup node (footprint =
    // board bbox, drafted thickness, bottom-left/top) without the operator visiting it.
    let mut app = EitriApp::new(Config::default(), Vec::new());
    let ctx = egui::Context::default();
    let _ = ctx; // the open goes through the worker, not an intent.
    app.launch(OpRequest::OpenGerber { name: "fixture-top".to_string(), source: GERBER.to_string() });
    pump_until_idle(&mut app);
    let first = app.view.selected_object().expect("the opened board is selected");
    let bounds = app.scene.object(first).and_then(|o| o.bounds).expect("the copper has bounds");
    let stock = app.ui.stock.expect("the first board auto-fits the stock");
    assert_eq!(stock.footprint(), bounds, "the auto-fit footprint is the board's bounding box");
    assert!((stock.thickness - 1.6).abs() < 1e-9, "the drafted default thickness is kept");

    // The operator now owns the setup: clearing it and opening ANOTHER board must not re-seed a stock over an
    // explicit native choice while geometry is already present.
    app.handle_intent(&egui::Context::default(), Intent::ClearStock);
    app.launch(OpRequest::OpenGerber { name: "fixture-other".to_string(), source: GERBER.to_string() });
    pump_until_idle(&mut app);
    assert_eq!(app.ui.stock, None, "a later board never overrides the operator's setup");
  }

  #[test]
  fn zoom_fit_defers_to_the_canvas_via_pending_fit() {
    let (mut app, _) = app_with_gerber();
    app.ui.pending_fit = false;
    let ctx = egui::Context::default();
    app.handle_intent(&ctx, Intent::ZoomFit);
    assert!(app.ui.pending_fit, "fit is consumed by the canvas when the real rect is known");
  }

  #[test]
  fn tool_db_add_edit_and_seed_round_trip_through_intents() {
    let (mut app, _) = app_with_gerber();
    let ctx = egui::Context::default();
    // Start from a known-empty library so the test is deterministic regardless of the seeded starter set or a
    // `tools.json` already present on this host (`EitriApp::new` loads/seeds one).
    app.tool_db = eitri_project::ToolDatabase::new();
    app.ui.tool_db_selected = None;
    app.refresh_tool_list();
    assert!(app.tool_db.is_empty() && app.ui.tool_list.is_empty());

    // Add: a new tool lands in the library, is selected for editing, marks the library dirty, and refreshes the
    // seed snapshot the panels read.
    app.handle_intent(&ctx, Intent::AddTool);
    assert_eq!(app.tool_db.len(), 1, "the tool is added");
    assert!(app.tool_db_dirty, "the library is now unsaved");
    assert_eq!(app.ui.tool_list.len(), 1, "the seed snapshot tracks the library");
    let tool_id = app.ui.tool_db_selected.expect("the new tool is selected for editing");

    // Edit: a whole-entry update replaces the tool in place.
    let mut entry = app.tool_db.get(tool_id).cloned().expect("the tool exists");
    entry.diameter = eitri_core::Length::from_mm(0.5);
    entry.isolation.passes = 3;
    entry.drilling.depth = 2.4;
    app.handle_intent(&ctx, Intent::UpdateTool(tool_id, entry));
    assert!((app.tool_db.get(tool_id).unwrap().diameter.as_mm() - 0.5).abs() < 1e-9, "the edit lands");

    // Seed: the isolation and drill drafts pick up the tool's defaults.
    app.handle_intent(&ctx, Intent::SeedIsolationFromTool(tool_id));
    assert!((app.ui.iso.tool_diameter - 0.5).abs() < 1e-9, "the diameter seeds the isolation draft");
    assert_eq!(app.ui.iso.passes, 3, "the passes seed too");
    app.handle_intent(&ctx, Intent::SeedDrillFromTool(tool_id));
    assert!((app.ui.drill.depth - 2.4).abs() < 1e-9, "the drill depth seeds from the tool");

    // Remove: the tool leaves the library and its selection clears.
    app.handle_intent(&ctx, Intent::RemoveTool(tool_id));
    assert!(app.tool_db.is_empty(), "the tool is removed");
    assert_eq!(app.ui.tool_db_selected, None, "the vanished selection clears");
    assert!(app.ui.tool_list.is_empty(), "the seed snapshot empties with it");
  }
}
