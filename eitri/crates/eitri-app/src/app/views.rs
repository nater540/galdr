//! The thin per-panel views: the toolbar, the project tree, the parameter panels, the bottom dock, and the
//! status bar. Every view reads [`super::view_state::ViewState`] + [`UiState`] and pushes
//! [`super::intent::Intent`]s — no view touches the session, the config, or the filesystem.
//!
//! Styling follows the shared design tokens: colours from [`super::theme::Palette`] (threaded through
//! [`RuntimeStyle`]), sizes from [`super::metrics::Metrics`]. Section headers are the recurring 30px strip with
//! tracked small-caps titles, exactly as in skirnir.

use eframe::egui::{self, Align, Color32, Layout, RichText, Stroke};

use super::canvas::CanvasView;
use super::intent::{Intent, IntentSink};
use super::metrics::Metrics;
use super::theme::Palette;
use super::view_state::{LogKind, OpView, Selection, TreeRow, ViewState};
use crate::config::CanvasStyle;
use crate::tr;
use eitri_gcode::{DrillJob, IsolationJob};
use eitri_project::{DatumCorner, DirectionSpec, DrillSpec, IsolationSpec, ObjectId, ObjectKind, Stock, ToolId, ZReference};

/// The resolved presentation style threaded into every view: the active palette plus the canvas render knobs.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct RuntimeStyle {
  /// The active colour palette.
  pub palette: Palette,
  /// The resolved canvas-render style.
  pub canvas: CanvasStyle,
}

/// The isolation parameter drafts the panel edits — plain numbers the operator owns until Run converts them
/// into the engine's spec/job pair. Defaults mirror the engine's own defaults.
#[derive(Debug, Clone, PartialEq)]
pub struct IsolationDraft {
  /// Tool diameter (mm).
  pub tool_diameter: f64,
  /// Number of concentric passes.
  pub passes: usize,
  /// Pass overlap as a `0..=0.9` fraction of the tool diameter.
  pub overlap: f64,
  /// Whether to combine all passes into one job.
  pub combine: bool,
  /// Climb (true) vs conventional milling.
  pub climb: bool,
  /// Total cut depth below the surface (positive mm).
  pub cut_depth: f64,
  /// Depth per pass (positive mm; 0 = single pass).
  pub pass_depth: f64,
  /// Cutting feed (mm/min).
  pub cut_feed: f64,
  /// Plunge feed (mm/min).
  pub plunge_feed: f64,
  /// Safe travel height (positive mm).
  pub travel_z: f64,
  /// Spindle speed (RPM).
  pub spindle_rpm: f64,
}

impl Default for IsolationDraft {
  fn default() -> Self {
    let job = IsolationJob::default();
    IsolationDraft {
      tool_diameter: 0.2,
      passes: 1,
      overlap: 0.15,
      combine: false,
      climb: true,
      cut_depth: job.cut_depth,
      pass_depth: job.pass_depth,
      cut_feed: job.cut_feed,
      plunge_feed: job.plunge_feed,
      travel_z: job.travel_z,
      spindle_rpm: job.spindle_rpm,
    }
  }
}

impl IsolationDraft {
  /// The engine spec, with degenerate hand-typed values held to sane floors (the engine also validates; this
  /// keeps the obvious cases from ever leaving the panel).
  pub fn to_spec(&self) -> IsolationSpec {
    IsolationSpec {
      tool_diameter: self.tool_diameter.max(0.01),
      passes: self.passes.max(1),
      overlap: self.overlap.clamp(0.0, 0.9),
      combine: self.combine,
      direction: if self.climb { DirectionSpec::Climb } else { DirectionSpec::Conventional },
    }
  }

  /// The emission job (depths/feeds/spindle), floored the same way.
  pub fn to_job(&self, name: Option<String>) -> IsolationJob {
    IsolationJob {
      cut_depth: self.cut_depth.max(0.001),
      pass_depth: self.pass_depth.max(0.0),
      cut_feed: self.cut_feed.max(1.0),
      plunge_feed: self.plunge_feed.max(1.0),
      travel_z: self.travel_z.max(0.1),
      spindle_rpm: self.spindle_rpm.max(0.0),
      name,
    }
  }
}

/// The drilling parameter drafts.
#[derive(Debug, Clone, PartialEq)]
pub struct DrillDraft {
  /// Drill depth as a positive magnitude below the surface (mm), matching the engine's convention.
  pub depth: f64,
  /// Plunge feed (mm/min).
  pub feed: f64,
  /// Retract height between holes (positive mm).
  pub retract: f64,
  /// Safe travel height (positive mm).
  pub travel_z: f64,
  /// Spindle speed (RPM).
  pub spindle_rpm: f64,
}

impl Default for DrillDraft {
  fn default() -> Self {
    let job = DrillJob::default();
    DrillDraft { depth: 1.7, feed: 120.0, retract: 2.0, travel_z: job.travel_z, spindle_rpm: job.spindle_rpm }
  }
}

impl DrillDraft {
  /// The engine spec: the depth is a positive magnitude (the emitter negates it to a negative Z), so a mistakenly
  /// negative entry is folded positive rather than air-drilling upward.
  pub fn to_spec(&self) -> DrillSpec {
    DrillSpec {
      depth: self.depth.abs().max(0.01),
      feed: self.feed.max(1.0),
      retract: self.retract.max(0.1),
      peck: None,
      dwell: None,
    }
  }

  /// The emission job.
  pub fn to_job(&self, name: Option<String>) -> DrillJob {
    DrillJob { travel_z: self.travel_z.max(0.1), spindle_rpm: self.spindle_rpm.max(0.0), name }
  }
}

/// Which CAM operation's parameters the panel shows for a copper/geometry selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OpChoice {
  /// Isolation routing.
  #[default]
  Isolate,
  /// Area clearing (paint).
  Paint,
  /// Non-copper clearing.
  NonCopper,
  /// Board cutout.
  Cutout,
  /// Panelization (produces geometry).
  Panelize,
  /// Two-sided mirror (produces geometry).
  Mirror,
  /// Photo-film SVG export.
  Film,
}

impl OpChoice {
  /// Every choice, in menu order.
  pub const ALL: [OpChoice; 7] = [
    OpChoice::Isolate,
    OpChoice::Paint,
    OpChoice::NonCopper,
    OpChoice::Cutout,
    OpChoice::Panelize,
    OpChoice::Mirror,
    OpChoice::Film,
  ];

  /// The i18n key of this choice's picker label.
  pub fn label_key(self) -> &'static str {
    match self {
      OpChoice::Isolate => "op-choice-isolate",
      OpChoice::Paint => "op-choice-paint",
      OpChoice::NonCopper => "op-choice-noncopper",
      OpChoice::Cutout => "op-choice-cutout",
      OpChoice::Panelize => "op-choice-panelize",
      OpChoice::Mirror => "op-choice-mirror",
      OpChoice::Film => "op-choice-film",
    }
  }
}

/// The Setup panel's editable stock drafts — the operator's numbers, committed to the session as a
/// [`Stock`] on every change (a cheap inline setter, like the rest of the setup controls). Kept separate from
/// the committed snapshot so a cleared (native) setup still remembers what the operator last typed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StockDraft {
  /// Footprint minimum X (native frame, mm) — set by Fit, kept through size edits.
  pub min_x: f64,
  /// Footprint minimum Y (native frame, mm).
  pub min_y: f64,
  /// Stock size along X (mm).
  pub size_x: f64,
  /// Stock size along Y (mm).
  pub size_y: f64,
  /// Material thickness (mm).
  pub thickness: f64,
  /// Which stock face is work-Z0.
  pub z_ref: ZReference,
  /// Which footprint corner (or the centre) is work X0 Y0.
  pub datum: DatumCorner,
}

impl Default for StockDraft {
  fn default() -> Self {
    // A blank project's placeholder block: a 100×100×1.6 mm sheet at the native origin, bottom-left/top —
    // the PCB-shaped defaults; the first loaded board auto-fits over them.
    StockDraft {
      min_x: 0.0,
      min_y: 0.0,
      size_x: 100.0,
      size_y: 100.0,
      thickness: 1.6,
      z_ref: ZReference::Top,
      datum: DatumCorner::BottomLeft,
    }
  }
}

impl StockDraft {
  /// The draft as the engine's [`Stock`], with degenerate hand-typed sizes floored to keep a zero-extent block
  /// from ever reaching the session.
  pub fn to_stock(&self) -> Stock {
    Stock {
      min_x: self.min_x,
      min_y: self.min_y,
      size_x: self.size_x.max(0.1),
      size_y: self.size_y.max(0.1),
      thickness: self.thickness.max(0.01),
      z_ref: self.z_ref,
      datum: self.datum,
    }
  }

  /// A draft mirroring a committed [`Stock`] (after a fit, a project load, or an intent round trip).
  pub fn from_stock(stock: Stock) -> Self {
    StockDraft {
      min_x: stock.min_x,
      min_y: stock.min_y,
      size_x: stock.size_x,
      size_y: stock.size_y,
      thickness: stock.thickness,
      z_ref: stock.z_ref,
      datum: stock.datum,
    }
  }
}

/// Facts about the selected object the parameter panel shows, snapshotted by the shell while the session is
/// home (the panel renders from this even while the session is away).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SelectedInfo {
  /// Polygon count (geometry objects).
  pub polygons: usize,
  /// Polyline count (geometry objects).
  pub polylines: usize,
  /// Drill-hit count (Excellon).
  pub hits: usize,
  /// Tool count (Excellon).
  pub tools: usize,
  /// G-code line count (CNC jobs).
  pub gcode_lines: usize,
  /// The job's dialect (CNC jobs).
  pub dialect: String,
  /// The job's cutting-tool diameter in millimetres (CNC jobs with a single tool: isolation/paint/non-copper/
  /// cutout). `None` for drilling (multi-tool) or non-job selections.
  pub tool_diameter: Option<f64>,
  /// The object's world bounds `(min_x, min_y, max_x, max_y)` from the render scene, for the panel's
  /// seed-from-object buttons (cutout rectangle, mirror centre).
  pub bounds: Option<(f64, f64, f64, f64)>,
}

/// Which dock tab is active.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DockTab {
  /// The log tail.
  #[default]
  Log,
  /// The selected job's G-code preview.
  Gcode,
}

/// The transient, egui-side widget state: drafts, the canvas transform, open dialogs, cached previews. Owned
/// by the shell, mutated by the views.
#[derive(Debug, Clone, Default)]
pub struct UiState {
  /// The resolved presentation style.
  pub style: RuntimeStyle,
  /// The canvas pan/zoom transform.
  pub canvas: CanvasView,
  /// A fit-to-bounds request consumed by the canvas at its next paint (when the real rect is known).
  pub pending_fit: bool,
  /// The isolation drafts.
  pub iso: IsolationDraft,
  /// The drilling drafts.
  pub drill: DrillDraft,
  /// Which op's parameters show for a copper/geometry selection.
  pub op_choice: OpChoice,
  /// The area-clearing drafts.
  pub paint: super::op_drafts::PaintDraft,
  /// The non-copper-clearing drafts.
  pub noncopper: super::op_drafts::NonCopperDraft,
  /// The board-cutout drafts.
  pub cutout: super::op_drafts::CutoutDraft,
  /// The panelization drafts.
  pub panelize: super::op_drafts::PanelizeDraft,
  /// The mirror drafts.
  pub mirror: super::op_drafts::MirrorDraft,
  /// The film-export drafts.
  pub film: super::op_drafts::FilmDraft,
  /// The active dock tab.
  pub dock_tab: DockTab,
  /// Facts about the selected object (rebuilt by the shell on selection/collection changes).
  pub selected_info: Option<SelectedInfo>,
  /// The selected CNC job's G-code lines, capped for display.
  pub gcode_preview: Vec<String>,
  /// Whether the application-settings dialog is open.
  pub app_settings_open: bool,
  /// The new-theme name field in the settings dialog.
  pub theme_name_draft: String,
  /// Whether the tool-database dialog is open.
  pub tool_db_open: bool,
  /// The tool selected for editing in the tool-database dialog, if any.
  pub tool_db_selected: Option<ToolId>,
  /// A snapshot of the tool library as `(id, name)` pairs, rebuilt by the shell whenever the library changes —
  /// what the parameter panels' seed-from-tool combos read (the dialog itself reads the live database).
  pub tool_list: Vec<(ToolId, String)>,
  /// The resolved work-zero `[x, y, z]` in native millimetres, snapshotted from the session by the shell —
  /// all zeros is the native frame. The canvas draws the origin crosshair at its XY and the Setup panel shows
  /// the full readout.
  pub work_origin: [f64; 3],
  /// The committed stock, snapshotted from the session by the shell — `None` is the native frame. The canvas
  /// draws the material block from its footprint; the Setup panel's grid highlight and readout key off it.
  pub stock: Option<Stock>,
  /// The Setup panel's editable stock numbers (see [`StockDraft`]); every edit re-commits via
  /// [`Intent::SetStock`].
  pub stock_draft: StockDraft,
  /// The Setup panel's fit-to-board reference pick, when the operator chose one (`None` = the first
  /// geometry-bearing row).
  pub fit_reference: Option<ObjectId>,
  /// The world position under the canvas pointer, for the status-bar readout.
  pub cursor_world: Option<[f64; 2]>,
  /// The in-progress inline tree rename: the object being renamed and the name draft.
  pub renaming: Option<(ObjectId, String)>,
  /// The in-progress canvas drag-to-move (`None` while panning or idle). Set on the frame a drag starts over an
  /// object and cleared when the drag ends, so every frame of one gesture moves the same group into one undo entry.
  pub canvas_drag: Option<super::canvas::CanvasDrag>,
}

// ── Shared building blocks ─────────────────────────────────────────────────────────────────────────────────

/// Render `content` inside a child `Ui` clipped and capped to the CURRENT available rect, then allocate exactly
/// that rect. This severs the content→panel size feedback (the skirnir lesson): under egui 0.35 a side panel
/// whose content measures wider than the column lets that content bleed over the neighbouring central region;
/// clipping to the fixed column rect keeps a too-wide row a local, visible truncation instead of a layout leak.
pub fn contained(ui: &mut egui::Ui, content: impl FnOnce(&mut egui::Ui)) {
  let rect = ui.available_rect_before_wrap();
  let mut child = ui.new_child(egui::UiBuilder::new().max_rect(rect).layout(egui::Layout::top_down(Align::Min)));
  child.set_clip_rect(rect.intersect(child.clip_rect()));
  child.set_max_width(rect.width());
  content(&mut child);
  ui.allocate_rect(rect, egui::Sense::hover());
}

/// The recurring 30px tracked-caps section header strip.
pub fn section_header(ui: &mut egui::Ui, palette: Palette, title: &str) {
  let (rect, _) = ui.allocate_exact_size(egui::vec2(ui.available_width(), Metrics::HEADER_H), egui::Sense::hover());
  ui.painter().rect_filled(rect, 0.0, palette.panel_alt);
  ui.painter().text(
    egui::pos2(rect.left() + Metrics::HEADER_PAD_X, rect.center().y),
    egui::Align2::LEFT_CENTER,
    title.to_uppercase(),
    egui::FontId::proportional(Metrics::HEADER_TEXT),
    palette.text_dim,
  );
  ui.painter().hline(rect.x_range(), rect.bottom() - 0.5, Stroke::new(1.0, palette.divider));
}

/// A thin vertical hairline separating toolbar groups, centred in its strip.
fn toolbar_divider(ui: &mut egui::Ui, palette: Palette) {
  let (rect, _) =
    ui.allocate_exact_size(egui::vec2(Metrics::TOOLBAR_DIVIDER_W, Metrics::TOOLBAR_CONTROL_H), egui::Sense::hover());
  ui.painter().vline(rect.center().x, rect.y_range(), Stroke::new(1.0, palette.divider));
}

// ── Toolbar ────────────────────────────────────────────────────────────────────────────────────────────────

/// The main toolbar: file actions, project persistence, undo/redo, fit, and the settings gear.
pub fn toolbar(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  let palette = state.style.palette;
  let busy = view.busy();
  ui.spacing_mut().item_spacing.x = Metrics::TOOLBAR_GAP;
  ui.spacing_mut().button_padding = egui::vec2(10.0, 4.0);
  ui.horizontal_centered(|ui| {
    ui.add_space(Metrics::TOOLBAR_PAD_X - Metrics::TOOLBAR_GAP);

    // Open/import: spawn worker ops, so they are gated while one runs.
    if ui.add_enabled(!busy, egui::Button::new(tr!("btn-open-gerber"))).clicked() {
      sink.push(Intent::OpenGerber);
    }
    if ui.add_enabled(!busy, egui::Button::new(tr!("btn-open-excellon"))).clicked() {
      sink.push(Intent::OpenExcellon);
    }
    ui.add_enabled_ui(!busy, |ui| {
      ui.menu_button(tr!("btn-import"), |ui| {
        if ui.button(tr!("btn-import-svg")).clicked() {
          sink.push(Intent::ImportSvg);
          ui.close();
        }
        if ui.button(tr!("btn-import-dxf")).clicked() {
          sink.push(Intent::ImportDxf);
          ui.close();
        }
        if ui.button(tr!("btn-import-gcode")).clicked() {
          sink.push(Intent::ImportGcode);
          ui.close();
        }
      });
    });

    toolbar_divider(ui, palette);

    if ui.add_enabled(!busy, egui::Button::new(tr!("btn-project-open"))).clicked() {
      sink.push(Intent::OpenProject);
    }
    if ui.add_enabled(!busy, egui::Button::new(tr!("btn-project-save"))).clicked() {
      sink.push(Intent::SaveProject);
    }

    toolbar_divider(ui, palette);

    // Undo/redo read the snapshotted history flags: disabled while busy (the session is away) and when the
    // history has nothing in that direction.
    if ui
      .add_enabled(!busy && view.can_undo, egui::Button::new(tr!("btn-undo")))
      .on_hover_text(tr!("tip-undo"))
      .clicked()
    {
      sink.push(Intent::Undo);
    }
    if ui
      .add_enabled(!busy && view.can_redo, egui::Button::new(tr!("btn-redo")))
      .on_hover_text(tr!("tip-redo"))
      .clicked()
    {
      sink.push(Intent::Redo);
    }

    toolbar_divider(ui, palette);

    if ui.button(tr!("btn-zoom-fit")).on_hover_text(tr!("tip-zoom-fit")).clicked() {
      sink.push(Intent::ZoomFit);
    }

    // The right cluster: the tool library and the settings gear.
    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
      ui.add_space(Metrics::TOOLBAR_PAD_X - Metrics::TOOLBAR_GAP);
      if ui.button(RichText::new("⚙").size(15.0)).on_hover_text(tr!("tip-settings")).clicked() {
        sink.push(Intent::OpenAppSettings);
      }
      if ui.button(tr!("btn-tools")).on_hover_text(tr!("tip-tools")).clicked() {
        sink.push(Intent::OpenToolDb);
      }
    });
  });
}

// ── Project tree ───────────────────────────────────────────────────────────────────────────────────────────

/// The kind glyph shown before a tree row's name — a shape cue alongside the kind colour, never colour alone.
fn kind_glyph(kind: ObjectKind) -> &'static str {
  match kind {
    ObjectKind::Gerber => "▰",
    ObjectKind::Excellon => "◉",
    ObjectKind::Geometry => "◇",
    ObjectKind::CncJob => "⚒",
  }
}

/// The kind's accent colour in the tree and the parameter header.
fn kind_color(palette: Palette, kind: ObjectKind) -> Color32 {
  match kind {
    ObjectKind::Gerber => palette.copper_edge,
    ObjectKind::Excellon => palette.drill,
    ObjectKind::Geometry => palette.geometry,
    ObjectKind::CncJob => palette.toolpath_cut,
  }
}

/// The localized kind label.
fn kind_label(kind: ObjectKind) -> String {
  match kind {
    ObjectKind::Gerber => tr!("kind-gerber"),
    ObjectKind::Excellon => tr!("kind-excellon"),
    ObjectKind::Geometry => tr!("kind-geometry"),
    ObjectKind::CncJob => tr!("kind-cncjob"),
  }
}

/// What a frame's interaction with one tree row asked for.
enum RowAction {
  /// Nothing this frame.
  None,
  /// Toggle the row's selection.
  Select,
  /// Flip the row's canvas visibility (the eye zone).
  ToggleVisibility,
  /// Start an inline rename (double-click on the name).
  BeginRename,
  /// Recalculate a toolpath (the ⟳ rebuild zone on a CNC-job row).
  Rebuild,
}

/// The width of the ⟳ rebuild-button zone on a CNC-job row's right edge (just left of the eye).
const TREE_REBUILD_W: f32 = 22.0;

/// The width of the eye (visibility-toggle) zone on a tree row's right edge.
const TREE_EYE_W: f32 = 24.0;

/// The left project tree: header + one selectable row per object (selection is a read-only view concern, so
/// it stays live even while an operation runs; the eye toggle and rename EDIT the session, so they are inert
/// while it is away on a worker).
pub fn tree_panel(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  let palette = state.style.palette;
  section_header(ui, palette, &tr!("tree-title"));
  egui::Frame::new().inner_margin(egui::Margin { left: 8, right: 8, top: 6, bottom: 6 }).show(ui, |ui| {
    ui.spacing_mut().item_spacing.y = 2.0;
    // The pinned Setup node: always first, always present — the project-wide stock/work-zero setup. Selection
    // is a read-only view concern, so it stays clickable even while an op runs (the panel gates its edits).
    let setup_selected = view.selected == Some(Selection::Setup);
    if setup_row(ui, palette, setup_selected) {
      sink.push(Intent::Select(if setup_selected { None } else { Some(Selection::Setup) }));
    }
    ui.add_space(2.0);
    ui.painter().hline(
      ui.available_rect_before_wrap().x_range(),
      ui.cursor().top(),
      Stroke::new(1.0, palette.divider),
    );
    ui.add_space(3.0);
    if view.tree.is_empty() {
      ui.add_space(4.0);
      ui.label(RichText::new(tr!("tree-empty")).size(11.5).color(palette.text_dim));
      ui.label(RichText::new(tr!("tree-empty-hint")).size(10.5).color(palette.text_disabled));
      return;
    }
    let busy = view.busy();
    egui::ScrollArea::vertical().id_salt("project-rows").auto_shrink([false, false]).show(ui, |ui| {
      ui.spacing_mut().item_spacing.y = 2.0;
      // Grouped objects render first — each under a folder header, members indented; then any ungrouped rows. A
      // group is a locked board (its layers move together), so the folder makes that registration visible.
      let mut grouped: std::collections::HashSet<ObjectId> = std::collections::HashSet::new();
      for group in &view.groups {
        group_header(ui, palette, &group.name);
        ui.indent(("tree-group", group.name.as_str()), |ui| {
          ui.spacing_mut().item_spacing.y = 2.0;
          for member in &group.members {
            grouped.insert(*member);
            if let Some(row) = view.tree.iter().find(|r| r.id == *member) {
              let selected = view.selected == Some(Selection::Object(row.id));
              object_row(ui, palette, row, selected, busy, state, sink);
            }
          }
        });
      }
      for row in &view.tree {
        if grouped.contains(&row.id) {
          continue;
        }
        let selected = view.selected == Some(Selection::Object(row.id));
        object_row(ui, palette, row, selected, busy, state, sink);
      }
    });
  });
}

/// A project-tree group (folder) header: a folder glyph and the group name in the dim tone. Non-interactive — the
/// folder is a visual grouping of its member rows, not a selectable node.
fn group_header(ui: &mut egui::Ui, palette: Palette, name: &str) {
  let (rect, _response) = ui.allocate_exact_size(egui::vec2(ui.available_width(), Metrics::PANEL_CONTROL_H),
    egui::Sense::hover());
  if ui.is_rect_visible(rect) {
    ui.painter().text(
      egui::pos2(rect.left() + 8.0, rect.center().y),
      egui::Align2::LEFT_CENTER,
      "▤",
      egui::FontId::proportional(11.0),
      palette.text_dim,
    );
    ui.painter().text(
      egui::pos2(rect.left() + 26.0, rect.center().y),
      egui::Align2::LEFT_CENTER,
      name,
      egui::FontId::proportional(11.5),
      palette.text_dim,
    );
  }
}

/// The TOOLPATHS panel: the collection's CNC jobs, listed below PROJECT in their own section. Reuses [`tree_row`]
/// (and [`object_row`]) so selection, the visibility eye, and inline rename behave exactly as in the project tree.
pub fn toolpaths_panel(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  let palette = state.style.palette;
  section_header(ui, palette, &tr!("toolpaths-title"));
  egui::Frame::new().inner_margin(egui::Margin { left: 8, right: 8, top: 6, bottom: 6 }).show(ui, |ui| {
    ui.spacing_mut().item_spacing.y = 2.0;
    if view.toolpaths.is_empty() {
      ui.add_space(4.0);
      ui.label(RichText::new(tr!("toolpaths-empty")).size(11.5).color(palette.text_dim));
      ui.label(RichText::new(tr!("toolpaths-empty-hint")).size(10.5).color(palette.text_disabled));
      return;
    }
    let busy = view.busy();
    egui::ScrollArea::vertical().id_salt("toolpath-rows").auto_shrink([false, false]).show(ui, |ui| {
      ui.spacing_mut().item_spacing.y = 2.0;
      for row in &view.toolpaths {
        let selected = view.selected == Some(Selection::Object(row.id));
        object_row(ui, palette, row, selected, busy, state, sink);
      }
    });
  });
}

/// Render one object row (in either panel) and fold its interaction into selection/visibility/rename — the shared
/// body behind the PROJECT tree and the TOOLPATHS list, so a job row behaves exactly like a source row.
fn object_row(ui: &mut egui::Ui, palette: Palette, row: &TreeRow, selected: bool, busy: bool, state: &mut UiState,
  sink: &mut IntentSink) {
  if state.renaming.as_ref().is_some_and(|(id, _)| *id == row.id) {
    rename_row(ui, state, row.id, sink);
    return;
  }
  match tree_row(ui, palette, row, selected, busy) {
    RowAction::Select => sink.push(Intent::Select(if selected { None } else { Some(Selection::Object(row.id)) })),
    RowAction::ToggleVisibility => sink.push(Intent::SetVisible(row.id, !row.visible)),
    RowAction::BeginRename => state.renaming = Some((row.id, row.name.clone())),
    RowAction::Rebuild => sink.push(Intent::RebuildJob(row.id)),
    RowAction::None => {}
  }
}

/// The pinned Setup row: hand-painted like [`tree_row`] (same fills, accent bar, glyph + name) but with no eye
/// zone — the node is synthetic and cannot be hidden or renamed. Returns whether it was clicked this frame.
fn setup_row(ui: &mut egui::Ui, palette: Palette, selected: bool) -> bool {
  let (rect, response) = ui.allocate_exact_size(
    egui::vec2(ui.available_width(), Metrics::PANEL_CONTROL_H),
    egui::Sense::click(),
  );
  let label = tr!("tree-setup");
  response.widget_info(|| {
    egui::WidgetInfo::selected(egui::WidgetType::Button, ui.is_enabled(), selected, label.clone())
  });
  if ui.is_rect_visible(rect) {
    let fill = if selected {
      palette.widget_active
    } else if response.hovered() {
      palette.widget_hover
    } else {
      Color32::TRANSPARENT
    };
    ui.painter().rect_filled(rect, Metrics::CONTROL_RADIUS as f32, fill);
    if selected {
      let bar = egui::Rect::from_min_max(rect.min, egui::pos2(rect.min.x + 2.0, rect.max.y));
      ui.painter().rect_filled(bar, 0.0, palette.accent);
    }
    // The glyph shares the work-zero violet: the Setup node, the canvas crosshair, and the stock block are one
    // semantic family.
    ui.painter().text(
      egui::pos2(rect.left() + 10.0, rect.center().y),
      egui::Align2::LEFT_CENTER,
      "▦",
      egui::FontId::proportional(11.0),
      palette.origin,
    );
    ui.painter().text(
      egui::pos2(rect.left() + 28.0, rect.center().y),
      egui::Align2::LEFT_CENTER,
      label,
      egui::FontId::proportional(12.5),
      if selected { palette.text } else { palette.text_dim },
    );
  }
  response.clicked()
}

/// The inline-rename editor rendered in place of a tree row: Enter commits (the shell validates against the
/// engine's name rules), Escape or clicking away cancels.
fn rename_row(ui: &mut egui::Ui, state: &mut UiState, id: ObjectId, sink: &mut IntentSink) {
  let Some((_, draft)) = state.renaming.as_mut() else { return };
  let body_row = ui.text_style_height(&egui::TextStyle::Body);
  let field = egui::TextEdit::singleline(draft)
    .margin(Metrics::text_field_margin(body_row, 6))
    .desired_width(ui.available_width() - 4.0);
  let response = ui.add(field);
  // Focus lands once, when the editor appears; egui keeps it afterwards.
  if !response.has_focus() && !response.lost_focus() {
    response.request_focus();
  }
  let commit = response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
  let cancel = ui.input(|i| i.key_pressed(egui::Key::Escape)) || (response.lost_focus() && !commit);
  if commit {
    let name = draft.trim().to_string();
    if !name.is_empty() {
      sink.push(Intent::Rename(id, name));
    }
    state.renaming = None;
  } else if cancel {
    state.renaming = None;
  }
}

/// One LEFT-ALIGNED, full-width selectable tree row: hover/selected fills, the kind glyph in its accent, the
/// name in primary/dim text, and the eye (visibility) zone on the right edge. Hand-painted (an `add_sized`
/// button centres its content — the wrong read for a tree), with real AccessKit info on both the row and the
/// eye so interaction tests and screen readers see two named controls.
fn tree_row(ui: &mut egui::Ui, palette: Palette, row: &super::view_state::TreeRow, selected: bool, busy: bool)
-> RowAction {
  let (rect, response) = ui.allocate_exact_size(
    egui::vec2(ui.available_width(), Metrics::PANEL_CONTROL_H),
    egui::Sense::click(),
  );
  response.widget_info(|| {
    egui::WidgetInfo::selected(egui::WidgetType::Button, ui.is_enabled(), selected, row.name.clone())
  });

  // The eye zone: a second, separately-labelled interaction region on the row's right edge, registered after
  // the row so it sits on top and owns its clicks. Session-editing, so inert while an op runs.
  let eye_rect = egui::Rect::from_min_max(egui::pos2(rect.right() - TREE_EYE_W, rect.top()), rect.max);
  let eye_response = ui.interact(eye_rect, ui.id().with(("tree-eye", row.id.0)), egui::Sense::click());
  let eye_label =
    if row.visible { tr!("vis-hide", { name: row.name.clone() }) } else { tr!("vis-show", { name: row.name.clone() }) };
  eye_response.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Button, !busy, eye_label.clone()));

  // The ⟳ rebuild zone: only CNC-job rows carry it, just left of the eye. Emphasized (warning tone) when the job
  // is stale, dim otherwise (a rebuild is always allowed). Session-editing, so inert while an op runs.
  let is_job = row.kind == ObjectKind::CncJob;
  let rebuild_rect = egui::Rect::from_min_max(
    egui::pos2(eye_rect.left() - TREE_REBUILD_W, rect.top()),
    egui::pos2(eye_rect.left(), rect.bottom()),
  );
  let rebuild_response = is_job.then(|| {
    let response = ui.interact(rebuild_rect, ui.id().with(("tree-rebuild", row.id.0)), egui::Sense::click());
    response.widget_info(|| {
      egui::WidgetInfo::labeled(egui::WidgetType::Button, !busy, tr!("job-rebuild", { name: row.name.clone() }))
    });
    response
  });

  if ui.is_rect_visible(rect) {
    let fill = if selected {
      palette.widget_active
    } else if response.hovered() {
      palette.widget_hover
    } else {
      Color32::TRANSPARENT
    };
    ui.painter().rect_filled(rect, Metrics::CONTROL_RADIUS as f32, fill);
    if selected {
      // A 2px accent bar on the row's left edge pairs the selection with a shape cue, not fill alone.
      let bar = egui::Rect::from_min_max(rect.min, egui::pos2(rect.min.x + 2.0, rect.max.y));
      ui.painter().rect_filled(bar, 0.0, palette.accent);
    }
    // A hidden object's whole row drops to the disabled tone — the state reads from the list itself, and the
    // struck-through eye pairs it with a shape cue.
    let name_color = if !row.visible {
      palette.text_disabled
    } else if selected {
      palette.text
    } else {
      palette.text_dim
    };
    ui.painter().text(
      egui::pos2(rect.left() + 10.0, rect.center().y),
      egui::Align2::LEFT_CENTER,
      kind_glyph(row.kind),
      egui::FontId::proportional(11.0),
      if row.visible { kind_color(palette, row.kind) } else { palette.text_disabled },
    );
    // The ⟳ rebuild button on a CNC-job row: warning-toned when stale (nudging a rebuild), dim otherwise, brighter
    // on hover. The name clips short of it so a long name never collides. Non-job rows have no button.
    let name_clip_right = if is_job { rebuild_rect.left() - 2.0 } else { eye_rect.left() - 2.0 };
    if let Some(rebuild_response) = &rebuild_response {
      let rebuild_color = if busy {
        palette.text_disabled
      } else if rebuild_response.hovered() {
        palette.text
      } else if row.stale {
        palette.state_warn
      } else {
        palette.text_dim
      };
      ui.painter().text(
        rebuild_rect.center(),
        egui::Align2::CENTER_CENTER,
        "⟳",
        egui::FontId::proportional(13.0),
        rebuild_color,
      );
    }
    // The name is clipped short of the button/eye zones so a long name never collides with them.
    let name_painter = ui
      .painter()
      .with_clip_rect(egui::Rect::from_min_max(rect.min, egui::pos2(name_clip_right, rect.bottom())));
    name_painter.text(
      egui::pos2(rect.left() + 28.0, rect.center().y),
      egui::Align2::LEFT_CENTER,
      &row.name,
      egui::FontId::proportional(12.5),
      name_color,
    );
    // The eye itself: dim when visible, disabled-toned with a diagonal strike when hidden (never colour alone).
    let eye_color = if busy {
      palette.text_disabled
    } else if eye_response.hovered() {
      palette.text
    } else if row.visible {
      palette.text_dim
    } else {
      palette.text_disabled
    };
    ui.painter().text(
      eye_rect.center(),
      egui::Align2::CENTER_CENTER,
      "👁",
      egui::FontId::proportional(11.0),
      eye_color,
    );
    if !row.visible {
      let (a, b) = (eye_rect.center() - egui::vec2(5.0, -5.0), eye_rect.center() + egui::vec2(5.0, -5.0));
      ui.painter().line_segment([a, b], Stroke::new(1.2, eye_color));
    }
  }

  if rebuild_response.as_ref().is_some_and(|r| r.clicked()) {
    return if busy { RowAction::None } else { RowAction::Rebuild };
  }
  if eye_response.clicked() {
    return if busy { RowAction::None } else { RowAction::ToggleVisibility };
  }
  if response.double_clicked() {
    return if busy { RowAction::None } else { RowAction::BeginRename };
  }
  if response.clicked() && !eye_response.clicked() {
    return RowAction::Select;
  }
  RowAction::None
}

// ── Parameter panels ───────────────────────────────────────────────────────────────────────────────────────

/// One labelled drag-value row in a parameter grid.
fn param_row(ui: &mut egui::Ui, palette: Palette, label: &str, value: &mut f64, speed: f64, range: std::ops::RangeInclusive<f64>) {
  ui.label(RichText::new(label).size(11.5).color(palette.text_dim));
  ui.add(egui::DragValue::new(value).speed(speed).range(range).max_decimals(3));
  ui.end_row();
}

/// The right parameters column: dispatches on the selected object's kind.
pub fn params_panel(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  let palette = state.style.palette;
  section_header(ui, palette, &tr!("params-title"));
  egui::Frame::new().inner_margin(Metrics::PANEL_PAD).show(ui, |ui| {
    // Dense panel rows: trim egui's default 14×6 button padding or the direction toggle and the accent action
    // buttons overflow the fixed 286px column (the recorded skirnir lesson about dense rows in exact panels).
    ui.spacing_mut().button_padding = egui::vec2(8.0, 3.0);
    if view.selected == Some(Selection::Setup) {
      setup_section(ui, palette, view, state, sink);
      return;
    }
    let Some(row) = view.selected_row().cloned() else {
      ui.add_space(4.0);
      ui.label(RichText::new(tr!("params-empty")).size(11.5).color(palette.text_dim));
      return;
    };

    // The selected object's identity line: kind glyph + name, in the kind's accent.
    ui.horizontal(|ui| {
      ui.label(RichText::new(kind_glyph(row.kind)).size(12.0).color(kind_color(palette, row.kind)));
      ui.label(RichText::new(&row.name).size(13.0).strong().color(palette.text));
    });
    ui.label(RichText::new(kind_label(row.kind)).size(10.5).color(palette.text_disabled));
    ui.add_space(6.0);

    let busy = view.busy();
    if busy {
      ui.label(RichText::new(tr!("params-busy")).size(11.0).color(palette.state_warn));
      ui.add_space(4.0);
    }

    egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
      match row.kind {
        ObjectKind::Gerber | ObjectKind::Geometry => {
          if row.kind == ObjectKind::Gerber {
            ui.label(RichText::new(tr!("gerber-info")).size(10.5).color(palette.text_dim));
          } else if let Some(info) = &state.selected_info {
            ui.label(
              RichText::new(tr!("geometry-shapes", { polygons: info.polygons, polylines: info.polylines }))
                .size(10.5)
                .color(palette.text_dim),
            );
          }
          ui.add_space(6.0);
          op_selector(ui, palette, state);
          ui.add_space(8.0);
          match state.op_choice {
            OpChoice::Isolate => isolation_section(ui, palette, state, busy, row.id, sink),
            OpChoice::Paint => paint_section(ui, palette, state, busy, row.id, sink),
            OpChoice::NonCopper => noncopper_section(ui, palette, view, state, busy, row.id, sink),
            OpChoice::Cutout => cutout_section(ui, palette, state, busy, row.id, sink),
            OpChoice::Panelize => panelize_section(ui, palette, state, busy, row.id, sink),
            OpChoice::Mirror => mirror_section(ui, palette, state, busy, row.id, sink),
            OpChoice::Film => film_section(ui, palette, state, busy, row.id, sink),
          }
        }
        ObjectKind::Excellon => {
          if let Some(info) = &state.selected_info {
            ui.label(
              RichText::new(tr!("excellon-hits", { hits: info.hits, tools: info.tools }))
                .size(10.5)
                .color(palette.text_dim),
            );
          }
          ui.add_space(6.0);
          drill_section(ui, palette, state, busy, row.id, sink);
        }
        ObjectKind::CncJob => {
          job_section(ui, palette, state, busy, row.id, sink);
        }
      }

      ui.add_space(12.0);
      ui.separator();
      ui.add_space(4.0);
      let delete = egui::Button::new(RichText::new(tr!("btn-delete-object")).size(11.5).color(palette.state_error));
      if ui.add_enabled(!busy, delete).clicked() {
        sink.push(Intent::DeleteObject(row.id));
      }
    });
  });
}

/// The isolation-routing parameter block + Run.
fn isolation_section(ui: &mut egui::Ui, palette: Palette, state: &mut UiState, busy: bool, id: ObjectId, sink: &mut IntentSink) {
  ui.label(section_title(palette, &tr!("params-isolation")));
  ui.add_space(4.0);
  egui::Grid::new("iso-params").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
    param_row(ui, palette, &tr!("params-tool-diameter"), &mut state.iso.tool_diameter, 0.01, 0.01..=10.0);
    ui.label(RichText::new(tr!("params-passes")).size(11.5).color(palette.text_dim));
    let mut passes = state.iso.passes as u32;
    if ui.add(egui::DragValue::new(&mut passes).range(1..=16)).changed() {
      state.iso.passes = passes as usize;
    }
    ui.end_row();
    ui.label(RichText::new(tr!("params-overlap")).size(11.5).color(palette.text_dim));
    let mut overlap_pct = state.iso.overlap * 100.0;
    if ui.add(egui::DragValue::new(&mut overlap_pct).speed(1).range(0.0..=90.0).suffix(" %")).changed() {
      state.iso.overlap = overlap_pct / 100.0;
    }
    ui.end_row();
    ui.label(RichText::new(tr!("params-direction")).size(11.5).color(palette.text_dim));
    // A combo, not a two-button toggle: the Swedish labels ("Medfräsning"/"Motfräsning") overflow the fixed
    // column side by side, and a combo stays compact in every locale.
    let current = if state.iso.climb { tr!("direction-climb") } else { tr!("direction-conventional") };
    egui::ComboBox::from_id_salt("iso-direction").width(130.0).selected_text(current).show_ui(ui, |ui| {
      ui.selectable_value(&mut state.iso.climb, true, tr!("direction-climb"));
      ui.selectable_value(&mut state.iso.climb, false, tr!("direction-conventional"));
    });
    ui.end_row();
    ui.label(RichText::new(tr!("params-combine")).size(11.5).color(palette.text_dim));
    ui.checkbox(&mut state.iso.combine, "");
    ui.end_row();
    param_row(ui, palette, &tr!("params-cut-depth"), &mut state.iso.cut_depth, 0.01, 0.001..=10.0);
    param_row(ui, palette, &tr!("params-pass-depth"), &mut state.iso.pass_depth, 0.01, 0.0..=10.0);
    param_row(ui, palette, &tr!("params-cut-feed"), &mut state.iso.cut_feed, 1.0, 1.0..=5000.0);
    param_row(ui, palette, &tr!("params-plunge-feed"), &mut state.iso.plunge_feed, 1.0, 1.0..=2000.0);
    param_row(ui, palette, &tr!("params-travel-z"), &mut state.iso.travel_z, 0.1, 0.1..=50.0);
    param_row(ui, palette, &tr!("params-spindle-rpm"), &mut state.iso.spindle_rpm, 100.0, 0.0..=60000.0);
  });
  seed_from_tool(ui, palette, state, "iso-seed", Intent::SeedIsolationFromTool, sink);
  ui.add_space(8.0);
  run_button(ui, palette, busy, &tr!("btn-run-isolation"), &tr!("tip-run-isolation"), || Intent::RunIsolate(id), sink);
}

/// The operation picker for a copper/geometry selection: one compact combo, so every op fits the fixed column.
fn op_selector(ui: &mut egui::Ui, palette: Palette, state: &mut UiState) {
  ui.horizontal(|ui| {
    ui.label(RichText::new(tr!("params-operation")).size(11.5).color(palette.text_dim));
    egui::ComboBox::from_id_salt("op-choice")
      .width(150.0)
      .selected_text(tr!(state.op_choice.label_key()))
      .show_ui(ui, |ui| {
        for choice in OpChoice::ALL {
          ui.selectable_value(&mut state.op_choice, choice, tr!(choice.label_key()));
        }
      });
  });
}

/// The shared emission rows (depths/feeds/spindle) the paint/non-copper/cutout grids append.
fn job_rows(ui: &mut egui::Ui, palette: Palette, job: &mut super::op_drafts::JobDraft) {
  param_row(ui, palette, &tr!("params-cut-depth"), &mut job.cut_depth, 0.01, 0.001..=10.0);
  param_row(ui, palette, &tr!("params-pass-depth"), &mut job.pass_depth, 0.01, 0.0..=10.0);
  param_row(ui, palette, &tr!("params-cut-feed"), &mut job.cut_feed, 1.0, 1.0..=5000.0);
  param_row(ui, palette, &tr!("params-plunge-feed"), &mut job.plunge_feed, 1.0, 1.0..=2000.0);
  param_row(ui, palette, &tr!("params-travel-z"), &mut job.travel_z, 0.1, 0.1..=50.0);
  param_row(ui, palette, &tr!("params-spindle-rpm"), &mut job.spindle_rpm, 100.0, 0.0..=60000.0);
}

/// One labelled combo row in a parameter grid; returns the inner response for change detection.
fn combo_row(ui: &mut egui::Ui, palette: Palette, label: &str, salt: &str, selected: String,
  content: impl FnOnce(&mut egui::Ui)) {
  ui.label(RichText::new(label).size(11.5).color(palette.text_dim));
  egui::ComboBox::from_id_salt(salt).width(130.0).selected_text(selected).show_ui(ui, content);
  ui.end_row();
}

/// The direction (climb/conventional) combo row shared by the milling ops.
fn direction_row(ui: &mut egui::Ui, palette: Palette, salt: &str, climb: &mut bool) {
  let current = if *climb { tr!("direction-climb") } else { tr!("direction-conventional") };
  combo_row(ui, palette, &tr!("params-direction"), salt, current, |ui| {
    ui.selectable_value(climb, true, tr!("direction-climb"));
    ui.selectable_value(climb, false, tr!("direction-conventional"));
  });
}

/// The area-clearing (paint) parameter block + Run.
fn paint_section(ui: &mut egui::Ui, palette: Palette, state: &mut UiState, busy: bool, id: ObjectId, sink: &mut IntentSink) {
  use super::op_drafts::StrategyChoice;
  ui.label(section_title(palette, &tr!("params-paint")));
  ui.add_space(4.0);
  egui::Grid::new("paint-params").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
    param_row(ui, palette, &tr!("params-tool-diameter"), &mut state.paint.tool_diameter, 0.01, 0.01..=10.0);
    ui.label(RichText::new(tr!("params-overlap")).size(11.5).color(palette.text_dim));
    let mut overlap_pct = state.paint.overlap * 100.0;
    if ui.add(egui::DragValue::new(&mut overlap_pct).speed(1).range(0.0..=90.0).suffix(" %")).changed() {
      state.paint.overlap = overlap_pct / 100.0;
    }
    ui.end_row();
    param_row(ui, palette, &tr!("params-margin"), &mut state.paint.margin, 0.05, 0.0..=20.0);
    let strategy_label = match state.paint.strategy {
      StrategyChoice::Concentric => tr!("strategy-concentric"),
      StrategyChoice::Seed => tr!("strategy-seed"),
      StrategyChoice::Raster => tr!("strategy-raster"),
    };
    combo_row(ui, palette, &tr!("params-strategy"), "paint-strategy", strategy_label, |ui| {
      ui.selectable_value(&mut state.paint.strategy, StrategyChoice::Concentric, tr!("strategy-concentric"));
      ui.selectable_value(&mut state.paint.strategy, StrategyChoice::Seed, tr!("strategy-seed"));
      ui.selectable_value(&mut state.paint.strategy, StrategyChoice::Raster, tr!("strategy-raster"));
    });
    if state.paint.strategy == StrategyChoice::Raster {
      param_row(ui, palette, &tr!("params-raster-angle"), &mut state.paint.raster_angle, 1.0, -90.0..=90.0);
    }
    direction_row(ui, palette, "paint-direction", &mut state.paint.climb);
    ui.label(RichText::new(tr!("params-finish-pass")).size(11.5).color(palette.text_dim));
    ui.checkbox(&mut state.paint.finish_pass, "");
    ui.end_row();
    job_rows(ui, palette, &mut state.paint.job);
  });
  ui.add_space(8.0);
  run_button(ui, palette, busy, &tr!("btn-run-paint"), &tr!("tip-run-paint"), || Intent::RunPaint(id), sink);
}

/// The non-copper-clearing parameter block + Run: the boundary pick over the paint pass that clears it.
fn noncopper_section(ui: &mut egui::Ui, palette: Palette, view: &ViewState, state: &mut UiState, busy: bool,
  id: ObjectId, sink: &mut IntentSink) {
  use super::op_drafts::BoundaryChoice;
  ui.label(section_title(palette, &tr!("params-noncopper")));
  ui.add_space(4.0);
  egui::Grid::new("noncopper-params").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
    let boundary_label = match state.noncopper.boundary {
      BoundaryChoice::BoundingBox => tr!("boundary-bbox"),
      BoundaryChoice::Object => tr!("boundary-object"),
    };
    combo_row(ui, palette, &tr!("params-boundary"), "noncopper-boundary", boundary_label, |ui| {
      ui.selectable_value(&mut state.noncopper.boundary, BoundaryChoice::BoundingBox, tr!("boundary-bbox"));
      ui.selectable_value(&mut state.noncopper.boundary, BoundaryChoice::Object, tr!("boundary-object"));
    });
    match state.noncopper.boundary {
      BoundaryChoice::BoundingBox => {
        param_row(ui, palette, &tr!("params-boundary-margin"), &mut state.noncopper.boundary_margin, 0.1, 0.0..=50.0);
      }
      BoundaryChoice::Object => {
        // The picker offers every OTHER copper/geometry object as the frame (typically a traced board outline).
        let current = state
          .noncopper
          .boundary_object
          .and_then(|picked| view.tree.iter().find(|r| r.id == picked))
          .map(|r| r.name.clone())
          .unwrap_or_else(|| tr!("boundary-object-none"));
        combo_row(ui, palette, &tr!("params-boundary-source"), "noncopper-boundary-object", current, |ui| {
          for row in &view.tree {
            let pickable = row.id != id
              && matches!(row.kind, ObjectKind::Gerber | ObjectKind::Geometry);
            if pickable {
              ui.selectable_value(&mut state.noncopper.boundary_object, Some(row.id), row.name.clone());
            }
          }
        });
      }
    }
    param_row(ui, palette, &tr!("params-tool-diameter"), &mut state.noncopper.paint.tool_diameter, 0.01, 0.01..=10.0);
    ui.label(RichText::new(tr!("params-overlap")).size(11.5).color(palette.text_dim));
    let mut overlap_pct = state.noncopper.paint.overlap * 100.0;
    if ui.add(egui::DragValue::new(&mut overlap_pct).speed(1).range(0.0..=90.0).suffix(" %")).changed() {
      state.noncopper.paint.overlap = overlap_pct / 100.0;
    }
    ui.end_row();
    direction_row(ui, palette, "noncopper-direction", &mut state.noncopper.paint.climb);
    job_rows(ui, palette, &mut state.noncopper.paint.job);
  });
  ui.add_space(8.0);
  run_button(ui, palette, busy, &tr!("btn-run-noncopper"), &tr!("tip-run-noncopper"), || Intent::RunNonCopper(id), sink);
}

/// The board-cutout parameter block + Run: outline (rectangle vs the object's silhouette), tabs, tool.
fn cutout_section(ui: &mut egui::Ui, palette: Palette, state: &mut UiState, busy: bool, id: ObjectId, sink: &mut IntentSink) {
  use super::op_drafts::OutlineChoice;
  ui.label(section_title(palette, &tr!("params-cutout")));
  ui.add_space(4.0);
  egui::Grid::new("cutout-params").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
    let outline_label = match state.cutout.outline {
      OutlineChoice::Rectangle => tr!("outline-rectangle"),
      OutlineChoice::Silhouette => tr!("outline-silhouette"),
    };
    combo_row(ui, palette, &tr!("params-outline"), "cutout-outline", outline_label, |ui| {
      ui.selectable_value(&mut state.cutout.outline, OutlineChoice::Rectangle, tr!("outline-rectangle"));
      ui.selectable_value(&mut state.cutout.outline, OutlineChoice::Silhouette, tr!("outline-silhouette"));
    });
    if state.cutout.outline == OutlineChoice::Rectangle {
      param_row(ui, palette, &tr!("params-rect-min-x"), &mut state.cutout.rect_min[0], 0.5, -1000.0..=1000.0);
      param_row(ui, palette, &tr!("params-rect-min-y"), &mut state.cutout.rect_min[1], 0.5, -1000.0..=1000.0);
      param_row(ui, palette, &tr!("params-rect-max-x"), &mut state.cutout.rect_max[0], 0.5, -1000.0..=1000.0);
      param_row(ui, palette, &tr!("params-rect-max-y"), &mut state.cutout.rect_max[1], 0.5, -1000.0..=1000.0);
    }
    param_row(ui, palette, &tr!("params-tool-diameter"), &mut state.cutout.tool_diameter, 0.05, 0.01..=10.0);
    param_row(ui, palette, &tr!("params-tab-width"), &mut state.cutout.tab_width, 0.1, 0.0..=20.0);
    ui.label(RichText::new(tr!("params-tab-count")).size(11.5).color(palette.text_dim));
    let mut tabs = state.cutout.tab_count as u32;
    if ui.add(egui::DragValue::new(&mut tabs).range(0..=16)).changed() {
      state.cutout.tab_count = tabs as usize;
    }
    ui.end_row();
    param_row(ui, palette, &tr!("params-margin"), &mut state.cutout.margin, 0.05, 0.0..=20.0);
    direction_row(ui, palette, "cutout-direction", &mut state.cutout.climb);
    job_rows(ui, palette, &mut state.cutout.job);
  });
  // Seed the rectangle from the selected object's real extent — the common "cut just outside the board" case.
  if state.cutout.outline == OutlineChoice::Rectangle
    && let Some((x0, y0, x1, y1)) = state.selected_info.as_ref().and_then(|info| info.bounds)
  {
    ui.add_space(4.0);
    if ui.button(tr!("btn-seed-bounds")).on_hover_text(tr!("tip-seed-bounds")).clicked() {
      state.cutout.rect_min = [x0, y0];
      state.cutout.rect_max = [x1, y1];
    }
  }
  ui.add_space(8.0);
  run_button(ui, palette, busy, &tr!("btn-run-cutout"), &tr!("tip-run-cutout"), || Intent::RunCutout(id), sink);
}

/// The panelization parameter block + Run (produces a geometry object, rendered like any other geometry).
fn panelize_section(ui: &mut egui::Ui, palette: Palette, state: &mut UiState, busy: bool, id: ObjectId, sink: &mut IntentSink) {
  use super::op_drafts::SpacingChoice;
  ui.label(section_title(palette, &tr!("params-panelize")));
  ui.add_space(4.0);
  egui::Grid::new("panelize-params").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
    ui.label(RichText::new(tr!("params-rows")).size(11.5).color(palette.text_dim));
    let mut rows = state.panelize.rows as u32;
    if ui.add(egui::DragValue::new(&mut rows).range(1..=50)).changed() {
      state.panelize.rows = rows as usize;
    }
    ui.end_row();
    ui.label(RichText::new(tr!("params-cols")).size(11.5).color(palette.text_dim));
    let mut cols = state.panelize.cols as u32;
    if ui.add(egui::DragValue::new(&mut cols).range(1..=50)).changed() {
      state.panelize.cols = cols as usize;
    }
    ui.end_row();
    let mode_label = match state.panelize.mode {
      SpacingChoice::Gap => tr!("spacing-gap"),
      SpacingChoice::Pitch => tr!("spacing-pitch"),
    };
    combo_row(ui, palette, &tr!("params-spacing-mode"), "panelize-mode", mode_label, |ui| {
      ui.selectable_value(&mut state.panelize.mode, SpacingChoice::Gap, tr!("spacing-gap"));
      ui.selectable_value(&mut state.panelize.mode, SpacingChoice::Pitch, tr!("spacing-pitch"));
    });
    param_row(ui, palette, &tr!("params-spacing-x"), &mut state.panelize.x, 0.5, 0.0..=500.0);
    param_row(ui, palette, &tr!("params-spacing-y"), &mut state.panelize.y, 0.5, 0.0..=500.0);
  });
  ui.add_space(8.0);
  run_button(ui, palette, busy, &tr!("btn-run-panelize"), &tr!("tip-run-panelize"), || Intent::RunPanelize(id), sink);
}

/// The two-sided mirror parameter block + Run (produces a geometry object).
fn mirror_section(ui: &mut egui::Ui, palette: Palette, state: &mut UiState, busy: bool, id: ObjectId, sink: &mut IntentSink) {
  use super::op_drafts::MirrorAxisChoice;
  ui.label(section_title(palette, &tr!("params-mirror")));
  ui.add_space(4.0);
  egui::Grid::new("mirror-params").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
    let axis_label = match state.mirror.axis {
      MirrorAxisChoice::Vertical => tr!("axis-vertical"),
      MirrorAxisChoice::Horizontal => tr!("axis-horizontal"),
    };
    combo_row(ui, palette, &tr!("params-mirror-axis"), "mirror-axis", axis_label, |ui| {
      ui.selectable_value(&mut state.mirror.axis, MirrorAxisChoice::Vertical, tr!("axis-vertical"));
      ui.selectable_value(&mut state.mirror.axis, MirrorAxisChoice::Horizontal, tr!("axis-horizontal"));
    });
    param_row(ui, palette, &tr!("params-mirror-value"), &mut state.mirror.value, 0.5, -1000.0..=1000.0);
  });
  // Seed the line from the object's centre — the flip-in-place everyone actually wants.
  if let Some((x0, y0, x1, y1)) = state.selected_info.as_ref().and_then(|info| info.bounds) {
    ui.add_space(4.0);
    if ui.button(tr!("btn-seed-center")).on_hover_text(tr!("tip-seed-center")).clicked() {
      state.mirror.value = match state.mirror.axis {
        super::op_drafts::MirrorAxisChoice::Vertical => (x0 + x1) / 2.0,
        super::op_drafts::MirrorAxisChoice::Horizontal => (y0 + y1) / 2.0,
      };
    }
  }
  ui.add_space(8.0);
  run_button(ui, palette, busy, &tr!("btn-run-mirror"), &tr!("tip-run-mirror"), || Intent::RunMirror(id), sink);
}

/// The film-export parameter block + Export. Film is a vector SVG saved via a dialog — an export action, not
/// a canvas op run — so its button exports rather than commits anything.
fn film_section(ui: &mut egui::Ui, palette: Palette, state: &mut UiState, busy: bool, id: ObjectId, sink: &mut IntentSink) {
  ui.label(section_title(palette, &tr!("params-film")));
  ui.add_space(4.0);
  egui::Grid::new("film-params").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
    let kind_label = if state.film.negative { tr!("film-negative") } else { tr!("film-positive") };
    combo_row(ui, palette, &tr!("params-film-kind"), "film-kind", kind_label, |ui| {
      ui.selectable_value(&mut state.film.negative, false, tr!("film-positive"));
      ui.selectable_value(&mut state.film.negative, true, tr!("film-negative"));
    });
    param_row(ui, palette, &tr!("params-film-scale"), &mut state.film.scale, 0.01, 0.01..=10.0);
    ui.label(RichText::new(tr!("params-film-mirror")).size(11.5).color(palette.text_dim));
    ui.checkbox(&mut state.film.mirror, "");
    ui.end_row();
    param_row(ui, palette, &tr!("params-film-border"), &mut state.film.border, 0.5, 0.0..=50.0);
  });
  ui.add_space(8.0);
  let export = egui::Button::new(RichText::new(tr!("btn-export-film")).color(palette.text)).fill(palette.accent);
  if ui.add_enabled(!busy, export).on_hover_text(tr!("tip-export-film")).clicked() {
    sink.push(Intent::ExportFilm(id));
  }
}

/// The height of the datum corner-grid glyph strip.
const DATUM_GRID_H: f32 = 104.0;
/// The board-glyph width inside the strip (the height follows from the strip minus the dot headroom).
const DATUM_BOARD_W: f32 = 150.0;
/// Radius of a corner dot's painted ring.
const DATUM_DOT_R: f32 = 5.5;
/// Side of a corner dot's square hit zone (comfortably larger than the ring).
const DATUM_HIT: f32 = 22.0;

/// Every pickable datum position with its i18n label key, in reading order.
const DATUM_POSITIONS: [(DatumCorner, &str); 5] = [
  (DatumCorner::TopLeft, "datum-corner-tl"),
  (DatumCorner::TopRight, "datum-corner-tr"),
  (DatumCorner::Center, "datum-corner-center"),
  (DatumCorner::BottomLeft, "datum-corner-bl"),
  (DatumCorner::BottomRight, "datum-corner-br"),
];

/// Where a corner's dot sits on the board glyph, as `(x, y)` fractions of the glyph rect in SCREEN space
/// (Y down). Corners are named in WORLD space (Y up), so "bottom" anchors at fraction 1 — the glyph's lower
/// edge — matching the canvas orientation the operator sees.
fn datum_anchor_fraction(corner: DatumCorner) -> (f32, f32) {
  match corner {
    DatumCorner::BottomLeft => (0.0, 1.0),
    DatumCorner::BottomRight => (1.0, 1.0),
    DatumCorner::TopLeft => (0.0, 0.0),
    DatumCorner::TopRight => (1.0, 0.0),
    DatumCorner::Center => (0.5, 0.5),
  }
}

/// The Vectric-style datum picker: a board glyph with five clickable dots (four corners + centre). Returns the
/// corner clicked this frame, if any. Each dot is a separately-labelled AccessKit control; while `busy` the
/// dots render disabled and clicks are swallowed (the session is away on a worker).
fn datum_grid(ui: &mut egui::Ui, palette: Palette, active: Option<DatumCorner>, busy: bool) -> Option<DatumCorner> {
  let (strip, _) =
    ui.allocate_exact_size(egui::vec2(ui.available_width(), DATUM_GRID_H), egui::Sense::hover());
  let board = egui::Rect::from_center_size(
    strip.center(),
    egui::vec2(DATUM_BOARD_W.min(strip.width() - DATUM_HIT), DATUM_GRID_H - DATUM_HIT),
  );
  // The board glyph: a quiet inset slab standing in for the reference object's bounds.
  ui.painter().rect_filled(board, 2.0, palette.inset);
  ui.painter().rect_stroke(board, 2.0, Stroke::new(1.0, palette.border_raised), egui::StrokeKind::Inside);

  let mut picked = None;
  for (corner, key) in DATUM_POSITIONS {
    let (fx, fy) = datum_anchor_fraction(corner);
    let center = egui::pos2(board.left() + fx * board.width(), board.top() + fy * board.height());
    let hit = egui::Rect::from_center_size(center, egui::Vec2::splat(DATUM_HIT));
    let response = ui.interact(hit, ui.id().with(("datum-corner", key)), egui::Sense::click());
    let label = tr!(key);
    let selected = active == Some(corner);
    response.widget_info(|| {
      egui::WidgetInfo::selected(egui::WidgetType::Button, !busy, selected, label.clone())
    });
    let ring = if busy {
      palette.text_disabled
    } else if selected {
      palette.accent
    } else if response.hovered() {
      palette.text
    } else {
      palette.text_dim
    };
    ui.painter().circle_stroke(center, DATUM_DOT_R, Stroke::new(1.4, ring));
    if selected {
      // The applied pick is a filled accent dot — shape + fill, so the state is not colour-alone on hover.
      ui.painter().circle_filled(center, DATUM_DOT_R - 2.0, palette.accent);
    }
    if response.clicked() && !busy {
      picked = Some(corner);
    }
    response.on_hover_text(label);
  }
  picked
}

/// The Setup panel — the Vectric-style "Job Setup" for the whole project: the stock (material block) size and
/// thickness, the fit-to-board affordance, the datum corner grid, the Z-zero face, the resolved work-zero
/// readout, and the native reset. Every edit re-commits the drafted [`Stock`] immediately (cheap session
/// setters — no Run button); the readout, the canvas block, and the crosshair confirm the result.
fn setup_section(ui: &mut egui::Ui, palette: Palette, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  // The identity block, mirroring an object selection's: glyph + name in the setup family's violet.
  ui.horizontal(|ui| {
    ui.label(RichText::new("▦").size(12.0).color(palette.origin));
    ui.label(RichText::new(tr!("tree-setup")).size(13.0).strong().color(palette.text));
  });
  ui.label(RichText::new(tr!("setup-kind")).size(10.5).color(palette.text_disabled));
  ui.add_space(6.0);

  let busy = view.busy();
  if busy {
    ui.label(RichText::new(tr!("params-busy")).size(11.0).color(palette.state_warn));
    ui.add_space(4.0);
  }

  egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
    ui.label(RichText::new(tr!("setup-hint")).size(10.5).color(palette.text_dim));
    ui.add_space(8.0);

    // ── Stock size ─────────────────────────────────────────────────────────────────────────────────────
    ui.label(section_title(palette, &tr!("params-stock")));
    ui.add_space(4.0);
    let mut edited = false;
    // Whether a size spinner is mid-drag this frame (a continuation, not the first frame): those per-frame commits
    // coalesce into one undo entry, while a typed edit or the first drag frame starts a fresh one (docs review #1).
    let mut dragging = false;
    ui.add_enabled_ui(!busy, |ui| {
      egui::Grid::new("stock-params").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
        for (label, value) in [
          (tr!("params-stock-x"), &mut state.stock_draft.size_x),
          (tr!("params-stock-y"), &mut state.stock_draft.size_y),
          (tr!("params-stock-thickness"), &mut state.stock_draft.thickness),
        ] {
          ui.label(RichText::new(label).size(11.5).color(palette.text_dim));
          let resp = ui.add(egui::DragValue::new(value).speed(0.1).range(0.01..=10000.0).max_decimals(3));
          edited |= resp.changed();
          dragging |= resp.dragged() && !resp.drag_started();
          ui.end_row();
        }
      });
    });

    // Fit-to-board: size the footprint to a geometry-bearing object's bounds (Gerber copper, geometry). Hidden
    // when nothing on the tree could bound a stock.
    let candidates: Vec<&super::view_state::TreeRow> = view
      .tree
      .iter()
      .filter(|row| matches!(row.kind, ObjectKind::Gerber | ObjectKind::Geometry))
      .collect();
    if !candidates.is_empty() {
      let reference = state
        .fit_reference
        .filter(|picked| candidates.iter().any(|row| row.id == *picked))
        .unwrap_or(candidates[0].id);
      ui.add_space(4.0);
      ui.horizontal(|ui| {
        if ui.add_enabled(!busy, egui::Button::new(tr!("btn-fit-stock"))).on_hover_text(tr!("tip-fit-stock")).clicked()
        {
          sink.push(Intent::FitStock { reference, thickness: state.stock_draft.thickness.max(0.01) });
        }
        let current = candidates.iter().find(|row| row.id == reference).map(|row| row.name.clone());
        egui::ComboBox::from_id_salt("fit-reference")
          .width(130.0)
          .selected_text(current.unwrap_or_default())
          .show_ui(ui, |ui| {
            for row in &candidates {
              ui.selectable_value(&mut state.fit_reference, Some(row.id), row.name.clone());
            }
          });
      });
    }

    // ── Datum (which stock corner is work X0 Y0) ───────────────────────────────────────────────────────
    ui.add_space(10.0);
    ui.label(section_title(palette, &tr!("params-datum")));
    ui.add_space(2.0);
    // The grid highlight shows the COMMITTED stock's corner; in the native frame nothing is lit, and the first
    // pick both sets the corner and commits the drafted stock.
    if let Some(corner) = datum_grid(ui, palette, state.stock.map(|stock| stock.datum), busy) {
      state.stock_draft.datum = corner;
      edited = true;
    }

    // ── Z zero (which stock face is work Z0) ───────────────────────────────────────────────────────────
    ui.add_enabled_ui(!busy, |ui| {
      egui::Grid::new("z-zero").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
        let z_label = match state.stock_draft.z_ref {
          ZReference::Top => tr!("z-zero-top"),
          ZReference::Bottom => tr!("z-zero-bottom"),
        };
        ui.label(RichText::new(tr!("params-z-zero")).size(11.5).color(palette.text_dim));
        egui::ComboBox::from_id_salt("z-zero-face").width(130.0).selected_text(z_label).show_ui(ui, |ui| {
          edited |= ui.selectable_value(&mut state.stock_draft.z_ref, ZReference::Top, tr!("z-zero-top")).changed();
          edited |=
            ui.selectable_value(&mut state.stock_draft.z_ref, ZReference::Bottom, tr!("z-zero-bottom")).changed();
        });
        ui.end_row();
      });
    });
    if edited && !busy {
      // Coalesce only the continuation frames of a spinner drag; the datum grid, the Z-ref combo, a typed size, and
      // the first drag frame are all discrete (`dragging` is false) and get their own undo entry.
      sink.push(Intent::SetStock { stock: state.stock_draft.to_stock(), coalesce: dragging });
    }

    // ── The resolved work zero: the truth about the frame every job posts in. Caption + value on separate
    // lines — a KiCad-frame board's coordinates run long, and one combined line would clip in the column. ─
    ui.add_space(8.0);
    // "Native" is decided by the resolved work origin, not by stock presence: a bare point datum (set via the
    // scripting API, or restored from such a project) has no stock yet a non-zero origin, and the panel must report
    // that real offset and let the operator clear it — not claim the frame is unshifted.
    let native = state.work_origin == [0.0, 0.0, 0.0];
    if native {
      ui.label(RichText::new(tr!("work-zero-native")).monospace().size(10.5).color(palette.text_disabled));
    } else {
      ui.label(RichText::new(tr!("work-zero-label")).size(10.5).color(palette.text_dim));
      let [x, y, z] = state.work_origin;
      ui.label(RichText::new(format!("X {x:.3}  Y {y:.3}  Z {z:.3}")).monospace().size(10.5).color(palette.text));
    }
    ui.add_space(6.0);
    // Enabled whenever there is anything to revert — a committed stock OR a bare non-zero datum — so a stock whose
    // datum happens to land on the origin is still clearable, and a point datum can be dropped.
    let has_setup = state.stock.is_some() || !native;
    if ui
      .add_enabled(!busy && has_setup, egui::Button::new(tr!("btn-stock-native")))
      .on_hover_text(tr!("tip-stock-native"))
      .clicked()
    {
      sink.push(Intent::ClearStock);
    }
  });
}

/// The drilling parameter block + Run.
fn drill_section(ui: &mut egui::Ui, palette: Palette, state: &mut UiState, busy: bool, id: ObjectId, sink: &mut IntentSink) {
  ui.label(section_title(palette, &tr!("params-drill")));
  ui.add_space(4.0);
  egui::Grid::new("drill-params").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
    param_row(ui, palette, &tr!("params-drill-depth"), &mut state.drill.depth, 0.05, 0.01..=20.0);
    param_row(ui, palette, &tr!("params-drill-feed"), &mut state.drill.feed, 1.0, 1.0..=2000.0);
    param_row(ui, palette, &tr!("params-drill-retract"), &mut state.drill.retract, 0.1, 0.1..=20.0);
    param_row(ui, palette, &tr!("params-travel-z"), &mut state.drill.travel_z, 0.1, 0.1..=50.0);
    param_row(ui, palette, &tr!("params-spindle-rpm"), &mut state.drill.spindle_rpm, 100.0, 0.0..=60000.0);
  });
  seed_from_tool(ui, palette, state, "drill-seed", Intent::SeedDrillFromTool, sink);
  ui.add_space(8.0);
  run_button(ui, palette, busy, &tr!("btn-run-drill"), &tr!("tip-run-drill"), || Intent::RunDrill(id), sink);
}

/// The CNC-job block: facts + export.
fn job_section(ui: &mut egui::Ui, palette: Palette, state: &mut UiState, busy: bool, id: ObjectId, sink: &mut IntentSink) {
  ui.label(section_title(palette, &tr!("params-job")));
  ui.add_space(4.0);
  if let Some(info) = &state.selected_info {
    ui.label(RichText::new(tr!("job-lines", { count: info.gcode_lines as u64 })).size(11.5).color(palette.text));
    ui.label(RichText::new(tr!("job-dialect", { dialect: info.dialect.clone() })).size(10.5).color(palette.text_dim));
    if let Some(dia) = info.tool_diameter {
      ui.label(RichText::new(tr!("job-tool", { dia: format!("{dia:.3}") })).size(10.5).color(palette.text_dim));
    }
  }
  ui.add_space(8.0);
  let export = egui::Button::new(RichText::new(tr!("btn-export-gcode")).color(palette.text)).fill(palette.accent);
  if ui.add_enabled(!busy, export).on_hover_text(tr!("tip-export-gcode")).clicked() {
    sink.push(Intent::ExportGcode(id));
  }
}

/// A tracked-caps sub-section title inside the parameter panel.
fn section_title(palette: Palette, title: &str) -> RichText {
  RichText::new(title.to_uppercase())
    .size(Metrics::HEADER_TEXT)
    .strong()
    .color(palette.text_dim)
    .extra_letter_spacing(Metrics::HEADER_TEXT * Metrics::HEADER_TRACKING_EM)
}

/// The seed-from-tool combo appended to the isolation/drill blocks: picking a tool pushes a seed intent the
/// shell applies to the drafts. Hidden when the library is empty (nothing to seed from).
fn seed_from_tool(ui: &mut egui::Ui, palette: Palette, state: &UiState, salt: &str, make: fn(ToolId) -> Intent,
  sink: &mut IntentSink) {
  if state.tool_list.is_empty() {
    return;
  }
  ui.add_space(6.0);
  ui.horizontal(|ui| {
    ui.label(RichText::new(tr!("btn-seed-tool")).size(11.5).color(palette.text_dim));
    egui::ComboBox::from_id_salt(salt).width(130.0).selected_text(tr!("seed-tool-hint")).show_ui(ui, |ui| {
      for (id, name) in &state.tool_list {
        if ui.selectable_label(false, name).clicked() {
          sink.push(make(*id));
        }
      }
    });
  });
}

/// The accent-filled Run button, disabled while an op is in flight.
fn run_button(ui: &mut egui::Ui, palette: Palette, busy: bool, label: &str, tip: &str, intent: impl FnOnce() -> Intent, sink: &mut IntentSink) {
  let button = egui::Button::new(RichText::new(label).color(palette.text)).fill(palette.accent);
  if ui.add_enabled(!busy, button).on_hover_text(tip).clicked() {
    sink.push(intent());
  }
}

// ── Bottom dock ────────────────────────────────────────────────────────────────────────────────────────────

/// The dock: a 30px tab strip (Log | G-code) with the op progress cluster on the right, over the active body.
pub fn dock(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  let palette = state.style.palette;
  // Tab strip.
  let strip_height = Metrics::HEADER_H;
  let (strip_rect, _) = ui.allocate_exact_size(egui::vec2(ui.available_width(), strip_height), egui::Sense::hover());
  ui.painter().rect_filled(strip_rect, 0.0, palette.panel_alt);
  let mut strip = ui.new_child(egui::UiBuilder::new().max_rect(strip_rect).layout(Layout::left_to_right(Align::Center)));
  strip.add_space(6.0);
  for (tab, key) in [(DockTab::Log, "dock-log"), (DockTab::Gcode, "dock-gcode")] {
    let active = state.dock_tab == tab;
    let label = RichText::new(tr!(key))
      .size(Metrics::TAB_TEXT)
      .color(if active { palette.text } else { palette.text_dim });
    let response = strip.add(egui::Button::new(label).frame(false));
    if response.clicked() {
      state.dock_tab = tab;
    }
    if active {
      let r = response.rect;
      strip.painter().hline(
        r.left()..=r.right(),
        strip_rect.bottom() - Metrics::TAB_UNDERLINE / 2.0,
        Stroke::new(Metrics::TAB_UNDERLINE, palette.accent),
      );
    }
  }
  // The progress cluster rides the right end of the strip while an op runs: label, bar, cancel.
  if let OpView::Running { label, done, total } = &view.op {
    let mut right = ui.new_child(egui::UiBuilder::new().max_rect(strip_rect).layout(Layout::right_to_left(Align::Center)));
    right.add_space(8.0);
    if right.button(tr!("btn-cancel")).on_hover_text(tr!("tip-cancel")).clicked() {
      sink.push(Intent::CancelOp);
    }
    let bar = match view.op_fraction() {
      Some(fraction) => egui::ProgressBar::new(fraction)
        .desired_width(Metrics::PROGRESS_W)
        .desired_height(Metrics::PROGRESS_H)
        .fill(palette.state_busy),
      None => egui::ProgressBar::new(0.0)
        .desired_width(Metrics::PROGRESS_W)
        .desired_height(Metrics::PROGRESS_H)
        .fill(palette.state_busy)
        .animate(true),
    };
    right.add(bar);
    let text = if *total > 0 {
      format!("{} · {done}/{total}", tr!("op-running", { label: label.clone() }))
    } else {
      tr!("op-running", { label: label.clone() })
    };
    right.label(RichText::new(text).size(10.5).color(palette.state_busy));
  }
  ui.painter().hline(strip_rect.x_range(), strip_rect.bottom() - 0.5, Stroke::new(1.0, palette.divider));

  // Body.
  match state.dock_tab {
    DockTab::Log => log_body(ui, view, palette),
    DockTab::Gcode => gcode_body(ui, state, palette),
  }
}

/// The log tail on the recessed surface, one coloured marker + text per line.
fn log_body(ui: &mut egui::Ui, view: &ViewState, palette: Palette) {
  egui::Frame::new()
    .fill(palette.inset)
    .inner_margin(egui::Margin { left: 10, right: 10, top: 6, bottom: 6 })
    .show(ui, |ui| {
      ui.set_min_size(ui.available_size());
      egui::ScrollArea::vertical().auto_shrink([false, false]).min_scrolled_height(0.0).stick_to_bottom(true).show(
        ui,
        |ui| {
          ui.spacing_mut().item_spacing.y = 2.0;
          if view.log.is_empty() {
            ui.label(RichText::new(tr!("log-ready")).monospace().size(11.0).color(palette.text_disabled));
          }
          for line in &view.log {
            let (marker, color) = match line.kind {
              LogKind::Info => ("·", palette.text_dim),
              LogKind::Ok => ("✔", palette.state_ok),
              LogKind::Warn => ("!", palette.state_warn),
              LogKind::Error => ("✖", palette.state_error),
            };
            ui.horizontal(|ui| {
              ui.label(RichText::new(marker).monospace().size(11.0).color(color));
              ui.label(RichText::new(&line.text).monospace().size(11.0).color(
                if line.kind == LogKind::Info { palette.text_dim } else { palette.text },
              ));
            });
          }
        },
      );
    });
}

/// The selected job's G-code preview (capped by the shell), line-numbered, monospace, on the recessed surface.
fn gcode_body(ui: &mut egui::Ui, state: &UiState, palette: Palette) {
  egui::Frame::new()
    .fill(palette.inset)
    .inner_margin(egui::Margin { left: 10, right: 10, top: 6, bottom: 6 })
    .show(ui, |ui| {
      ui.set_min_size(ui.available_size());
      if state.gcode_preview.is_empty() {
        ui.label(RichText::new(tr!("gcode-empty")).size(11.0).color(palette.text_disabled));
        return;
      }
      egui::ScrollArea::vertical().auto_shrink([false, false]).min_scrolled_height(0.0).show_rows(
        ui,
        14.0,
        state.gcode_preview.len(),
        |ui, range| {
          for index in range {
            ui.horizontal(|ui| {
              ui.label(
                RichText::new(format!("{:>5}", index + 1)).monospace().size(11.0).color(palette.text_disabled),
              );
              ui.label(RichText::new(&state.gcode_preview[index]).monospace().size(11.0).color(palette.text_dim));
            });
          }
        },
      );
    });
}

// ── Status bar ─────────────────────────────────────────────────────────────────────────────────────────────

/// The bottom status bar: object count + op state on the left, the cursor's world position on the right.
pub fn status_bar(ui: &mut egui::Ui, view: &ViewState, state: &UiState) {
  let palette = state.style.palette;
  ui.horizontal_centered(|ui| {
    ui.add_space(10.0);
    // Count both panels: the split moved CNC jobs out of `tree` into `toolpaths`, but they are still objects.
    let object_count = (view.tree.len() + view.toolpaths.len()) as u64;
    ui.label(
      RichText::new(tr!("status-objects", { count: object_count })).size(10.5).color(palette.text_dim),
    );
    ui.add_space(10.0);
    match &view.op {
      OpView::Running { label, .. } => {
        // The busy dot + label: state is text + colour, never colour alone.
        let (dot, _) = ui.allocate_exact_size(egui::vec2(8.0, 8.0), egui::Sense::hover());
        ui.painter().circle_filled(dot.center(), 3.0, palette.state_busy);
        ui.label(RichText::new(tr!("op-running", { label: label.clone() })).size(10.5).color(palette.state_busy));
      }
      OpView::Idle => {
        ui.label(RichText::new(tr!("status-idle")).size(10.5).color(palette.text_disabled));
      }
    }
    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
      ui.add_space(10.0);
      if let Some([x, y]) = state.cursor_world {
        ui.label(
          RichText::new(format!("X {x:>9.3}  Y {y:>9.3}  {}", tr!("status-units")))
            .monospace()
            .size(10.5)
            .color(palette.text_dim),
        );
      }
    });
  });
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn isolation_drafts_convert_to_engine_spec_and_job() {
    let draft = IsolationDraft {
      tool_diameter: 0.3,
      passes: 3,
      overlap: 0.25,
      combine: true,
      climb: false,
      ..IsolationDraft::default()
    };
    let spec = draft.to_spec();
    assert!((spec.tool_diameter - 0.3).abs() < 1e-9);
    assert_eq!(spec.passes, 3);
    assert!((spec.overlap - 0.25).abs() < 1e-9);
    assert!(spec.combine);
    assert_eq!(spec.direction, DirectionSpec::Conventional);
    let job = draft.to_job(Some("fixture".into()));
    assert_eq!(job.name.as_deref(), Some("fixture"));
    assert!((job.cut_depth - draft.cut_depth).abs() < 1e-9);
  }

  #[test]
  fn degenerate_hand_typed_drafts_are_floored_before_reaching_the_engine() {
    let draft = IsolationDraft {
      tool_diameter: 0.0,
      passes: 0,
      overlap: 7.0,
      cut_depth: -3.0,
      cut_feed: 0.0,
      plunge_feed: -5.0,
      travel_z: 0.0,
      ..IsolationDraft::default()
    };
    let spec = draft.to_spec();
    assert!(spec.tool_diameter > 0.0, "a zero tool diameter must be floored");
    assert!(spec.passes >= 1);
    assert!(spec.overlap <= 0.9, "overlap clamps to a sane fraction");
    let job = draft.to_job(None);
    assert!(job.cut_depth > 0.0, "the engine wants a positive cut depth");
    assert!(job.cut_feed >= 1.0 && job.plunge_feed >= 1.0 && job.travel_z >= 0.1);
  }

  #[test]
  fn drill_drafts_keep_the_depth_a_positive_magnitude() {
    // A positive magnitude passes straight through; the emitter negates it to a negative Z to drill down.
    let draft = DrillDraft { depth: 1.6, ..DrillDraft::default() };
    assert!((draft.to_spec().depth - 1.6).abs() < 1e-9, "a positive depth is preserved");
    // A mistakenly negative entry is folded positive so it never air-drills upward.
    let folded = DrillDraft { depth: -2.0, ..DrillDraft::default() }.to_spec().depth;
    assert!((folded - 2.0).abs() < 1e-9, "a negative entry is folded positive: {folded}");
  }

  #[test]
  fn ui_state_defaults_are_ready_to_render() {
    let state = UiState::default();
    assert_eq!(state.dock_tab, DockTab::Log);
    assert!(!state.app_settings_open && !state.pending_fit);
    assert!(state.gcode_preview.is_empty() && state.selected_info.is_none());
    assert_eq!(state.work_origin, [0.0, 0.0, 0.0], "a fresh app is in the native frame");
    assert_eq!(state.stock, None, "no stock is committed until the operator (or the auto-fit) sets one");
    assert_eq!(state.fit_reference, None);
  }

  #[test]
  fn stock_drafts_default_to_a_pcb_shaped_block_and_round_trip_through_stock() {
    let draft = StockDraft::default();
    assert_eq!((draft.size_x, draft.size_y), (100.0, 100.0));
    assert!((draft.thickness - 1.6).abs() < 1e-9, "the default thickness is the standard 1.6 mm PCB");
    assert_eq!(draft.z_ref, ZReference::Top);
    assert_eq!(draft.datum, DatumCorner::BottomLeft);
    // A committed stock mirrors back into an identical draft (the shell syncs after fits and project loads).
    let stock = Stock {
      min_x: 120.0,
      min_y: -110.0,
      size_x: 17.0,
      size_y: 24.0,
      thickness: 1.6,
      z_ref: ZReference::Bottom,
      datum: DatumCorner::Center,
    };
    let round = StockDraft::from_stock(stock).to_stock();
    assert_eq!(round, stock, "a committed stock round-trips through the draft unchanged");
  }

  #[test]
  fn degenerate_stock_drafts_are_floored_before_reaching_the_session() {
    // A hand-typed zero (or a mid-edit negative) must never commit a zero-extent block or a paper-thin stock.
    let draft = StockDraft { size_x: 0.0, size_y: -5.0, thickness: 0.0, ..StockDraft::default() };
    let stock = draft.to_stock();
    assert!(stock.size_x > 0.0 && stock.size_y > 0.0, "sizes are floored positive");
    assert!(stock.thickness > 0.0, "thickness is floored positive");
  }

  #[test]
  fn datum_grid_anchors_map_corners_to_the_right_screen_fractions() {
    use eitri_project::DatumCorner;
    // The glyph is painted in SCREEN space (Y down), but the corners are named in WORLD space (Y up): the
    // world bottom-left must anchor at the glyph's left-BOTTOM, i.e. fraction (0, 1).
    assert_eq!(datum_anchor_fraction(DatumCorner::BottomLeft), (0.0, 1.0));
    assert_eq!(datum_anchor_fraction(DatumCorner::BottomRight), (1.0, 1.0));
    assert_eq!(datum_anchor_fraction(DatumCorner::TopLeft), (0.0, 0.0));
    assert_eq!(datum_anchor_fraction(DatumCorner::TopRight), (1.0, 0.0));
    assert_eq!(datum_anchor_fraction(DatumCorner::Center), (0.5, 0.5));
  }
}
