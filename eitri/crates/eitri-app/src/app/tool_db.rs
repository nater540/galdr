//! The tool-database dialog: manage the user-global tool library (add / select / edit / remove) and persist it
//! on the explicit Save. Same thin-render shape as the app-settings dialog — it reads [`UiState`] plus the live
//! [`ToolDatabase`] the shell owns and pushes [`Intent`]s; the shell applies them (mutating the library, writing
//! the file). No view touches the library directly.
//!
//! A tool carries a diameter plus per-operation default bundles; editing pushes a whole-entry
//! [`Intent::UpdateTool`] (like the settings dialog's whole-theme upsert), so the shell re-snapshots the seed
//! list and marks the library dirty.

use eframe::egui::{self, Align, Layout, RichText};

use super::intent::{Intent, IntentSink};
use super::metrics::Metrics;
use super::views::UiState;
use crate::tr;
use eitri_core::Length;
use eitri_project::{DirectionSpec, ToolDatabase, ToolId};

/// Show the dialog as a closable, resizable window. `dirty` is whether the in-memory library has unsaved edits.
pub fn window(ctx: &egui::Context, state: &mut UiState, database: &ToolDatabase, dirty: bool, sink: &mut IntentSink) {
  let mut open = state.tool_db_open;
  egui::Window::new(tr!("tool-db-title"))
    .id(egui::Id::new("tool-db-window"))
    .open(&mut open)
    .resizable(true)
    .default_size([420.0, 500.0])
    .show(ctx, |ui| body(ui, state, database, dirty, sink));
  state.tool_db_open = open;
}

/// The dialog body: the add/save action row, the selectable tool list, and the editor for the selected tool.
/// Split from [`window`] so tests can mount it directly in a harness `Ui`.
pub fn body(ui: &mut egui::Ui, state: &mut UiState, database: &ToolDatabase, dirty: bool, sink: &mut IntentSink) {
  let palette = state.style.palette;
  ui.spacing_mut().button_padding.y = 3.0;

  // Action row: add a tool (left) and the explicit Save boundary (right).
  ui.horizontal(|ui| {
    if ui.button(tr!("tool-db-add")).clicked() {
      sink.push(Intent::AddTool);
    }
    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
      let save = egui::Button::new(RichText::new(tr!("app-settings-save")).color(palette.text)).fill(palette.accent);
      if ui.add_enabled(dirty, save).on_hover_text(tr!("tool-db-save-hint")).clicked() {
        sink.push(Intent::SaveToolDb);
      }
      if dirty {
        ui.add_space(4.0);
        ui.label(RichText::new(tr!("app-settings-unsaved")).size(10.5).color(palette.state_warn));
      }
    });
  });
  ui.add_space(6.0);
  ui.separator();

  if database.is_empty() {
    ui.add_space(6.0);
    ui.label(RichText::new(tr!("tool-db-empty")).size(11.5).color(palette.text_dim));
    return;
  }

  // The tool list: one selectable row per tool, name + diameter. Selecting stores the id in UI-local state.
  egui::ScrollArea::vertical().auto_shrink([false, true]).max_height(150.0).show(ui, |ui| {
    for tool in database.iter() {
      let selected = state.tool_db_selected == Some(tool.id);
      let label = format!("{}  ·  ⌀{:.3} mm", tool.name, tool.diameter.as_mm());
      if ui.selectable_label(selected, label).clicked() {
        state.tool_db_selected = if selected { None } else { Some(tool.id) };
      }
    }
  });
  ui.add_space(6.0);
  ui.separator();

  // The editor for the selected tool: edit a clone, push a whole-entry update on any change.
  let Some(id) = state.tool_db_selected else {
    ui.add_space(6.0);
    ui.label(RichText::new(tr!("tool-db-select")).size(11.5).color(palette.text_dim));
    return;
  };
  let Some(tool) = database.get(id) else {
    // The selection points at a removed tool (a stale frame); clear it silently.
    state.tool_db_selected = None;
    return;
  };

  let mut edited = tool.clone();
  let mut changed = false;

  egui::Grid::new("tool-db-identity").num_columns(2).spacing([12.0, 8.0]).show(ui, |ui| {
    ui.label(RichText::new(tr!("tool-db-name")).color(palette.text_dim));
    let body_row = ui.text_style_height(&egui::TextStyle::Body);
    let field = egui::TextEdit::singleline(&mut edited.name)
      .margin(Metrics::text_field_margin(body_row, 6))
      .desired_width(200.0);
    if ui.add(field).changed() {
      changed = true;
    }
    ui.end_row();

    ui.label(RichText::new(tr!("tool-db-diameter")).color(palette.text_dim));
    let mut diameter = edited.diameter.as_mm();
    if ui.add(egui::DragValue::new(&mut diameter).speed(0.01).range(0.01..=20.0).max_decimals(3)).changed() {
      edited.diameter = Length::from_mm(diameter);
      changed = true;
    }
    ui.end_row();
  });

  ui.add_space(6.0);
  ui.label(sub_header(palette, &tr!("tool-db-iso-defaults")));
  ui.add_space(2.0);
  egui::Grid::new("tool-db-iso").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
    ui.label(RichText::new(tr!("params-passes")).size(11.5).color(palette.text_dim));
    let mut passes = edited.isolation.passes as u32;
    if ui.add(egui::DragValue::new(&mut passes).range(1..=16)).changed() {
      edited.isolation.passes = passes as usize;
      changed = true;
    }
    ui.end_row();

    ui.label(RichText::new(tr!("params-overlap")).size(11.5).color(palette.text_dim));
    let mut overlap_pct = edited.isolation.overlap * 100.0;
    if ui.add(egui::DragValue::new(&mut overlap_pct).speed(1.0).range(0.0..=90.0).suffix(" %")).changed() {
      edited.isolation.overlap = overlap_pct / 100.0;
      changed = true;
    }
    ui.end_row();

    ui.label(RichText::new(tr!("params-combine")).size(11.5).color(palette.text_dim));
    if ui.checkbox(&mut edited.isolation.combine, "").changed() {
      changed = true;
    }
    ui.end_row();

    ui.label(RichText::new(tr!("params-direction")).size(11.5).color(palette.text_dim));
    let mut climb = edited.isolation.direction == DirectionSpec::Climb;
    let current = if climb { tr!("direction-climb") } else { tr!("direction-conventional") };
    egui::ComboBox::from_id_salt("tool-db-direction").width(130.0).selected_text(current).show_ui(ui, |ui| {
      changed |= ui.selectable_value(&mut climb, true, tr!("direction-climb")).changed();
      changed |= ui.selectable_value(&mut climb, false, tr!("direction-conventional")).changed();
    });
    edited.isolation.direction = if climb { DirectionSpec::Climb } else { DirectionSpec::Conventional };
    ui.end_row();
  });

  ui.add_space(6.0);
  ui.label(sub_header(palette, &tr!("tool-db-drill-defaults")));
  ui.add_space(2.0);
  egui::Grid::new("tool-db-drill").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
    ui.label(RichText::new(tr!("params-drill-depth")).size(11.5).color(palette.text_dim));
    if ui.add(egui::DragValue::new(&mut edited.drilling.depth).speed(0.05).range(0.01..=20.0).max_decimals(3)).changed()
    {
      changed = true;
    }
    ui.end_row();

    ui.label(RichText::new(tr!("params-drill-feed")).size(11.5).color(palette.text_dim));
    if ui.add(egui::DragValue::new(&mut edited.drilling.feed).speed(1.0).range(1.0..=2000.0).max_decimals(1)).changed()
    {
      changed = true;
    }
    ui.end_row();

    ui.label(RichText::new(tr!("params-drill-retract")).size(11.5).color(palette.text_dim));
    if ui.add(egui::DragValue::new(&mut edited.drilling.retract).speed(0.1).range(0.1..=20.0).max_decimals(2)).changed()
    {
      changed = true;
    }
    ui.end_row();
  });

  if changed {
    sink.push(Intent::UpdateTool(id, edited));
  }

  ui.add_space(10.0);
  let remove = egui::Button::new(RichText::new(tr!("tool-db-remove")).size(11.5).color(palette.state_error));
  if ui.add(remove).clicked() {
    sink.push(Intent::RemoveTool(id));
  }
}

/// The default entry a freshly-added tool starts from: a common 0.2 mm cutter with the engine's default
/// isolation/drill bundles. Its placeholder id is overwritten by the database on insert.
pub fn default_entry(name: String) -> eitri_project::ToolEntry {
  eitri_project::ToolEntry {
    id: ToolId(0),
    name,
    diameter: Length::from_mm(0.2),
    isolation: eitri_project::IsolationDefaults::default(),
    drilling: eitri_project::DrillDefaults::default(),
  }
}

/// A tracked-caps sub-header inside the dialog, matching the parameter panel's section titles.
fn sub_header(palette: super::theme::Palette, title: &str) -> RichText {
  RichText::new(title.to_uppercase())
    .size(Metrics::HEADER_TEXT)
    .strong()
    .color(palette.text_dim)
    .extra_letter_spacing(Metrics::HEADER_TEXT * Metrics::HEADER_TRACKING_EM)
}
