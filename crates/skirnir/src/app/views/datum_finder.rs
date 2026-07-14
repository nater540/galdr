//! The datum finder panel: edge / corner touch-off and its readings + bench params.

use super::*;

/// Render the datum finder (edge / corner / Z surface). When no run is active it offers the two shapes — a
/// four-corner picker with an inside/outside toggle, and a single-edge axis/direction picker — plus the bench
/// params and a Start button each; when a run is active it guides the operator touch by touch and shows the
/// captured readings + computed datum. The wizard state is owned by the shell and passed in as a borrow, so this
/// stays a pure render that only emits intents.
pub fn datum_finder(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState,
  datum: Option<&crate::app::datum::DatumState>, sink: &mut IntentSink) {
  use crate::app::datum::{Corner, DatumStep, DatumTarget};
  let palette = state.style.palette;
  egui::Frame::new().inner_margin(Metrics::RIGHT_PAD).show(ui, |ui| {
    let full = Vec2::new(ui.available_width(), Metrics::PANEL_CONTROL_H + 6.0);
    let idle = view.connection == ConnectionState::Idle;
    let Some(w) = datum else {
      // No run: offer the corner picker and the single-edge picker, plus the bench params, and a Start for each.
      ui.label(RichText::new(crate::tr!("datum-intro")).size(11.0).color(palette.text_dim));
      ui.add_space(4.0);

      // ── Corner ──
      ui.label(RichText::new(crate::tr!("datum-corner-label")).size(11.0).color(palette.text));
      let inside = state.datum_corner.inside;
      egui::Grid::new("datum_corner_pick").num_columns(2).show(ui, |ui| {
        // The 2×2 grid mirrors the physical corners; each button is labelled by its OUTSIDE approach signs. The
        // inside toggle (below) flips the actual approach without changing the picked location.
        corner_button(ui, state, Corner::B, "−X +Y");
        corner_button(ui, state, Corner::A, "+X +Y");
        ui.end_row();
        corner_button(ui, state, Corner::C, "−X −Y");
        corner_button(ui, state, Corner::D, "+X −Y");
        ui.end_row();
      });
      let mut inside_toggle = inside;
      if ui.checkbox(&mut inside_toggle, RichText::new(crate::tr!("datum-inside")).size(11.0)).changed() {
        state.datum_corner.inside = inside_toggle;
      }
      ui.add_enabled_ui(idle, |ui| {
        if ui.add_sized(full, egui::Button::new(crate::tr!("btn-find-corner"))).clicked() {
          sink.push(Intent::DatumCornerStart { corner: state.datum_corner, params: state.datum_bench });
        }
      });

      ui.separator();

      // ── Single edge ──
      ui.label(RichText::new(crate::tr!("datum-edge-label")).size(11.0).color(palette.text));
      egui::Grid::new("datum_edge_pick").num_columns(2).show(ui, |ui| {
        ui.label(crate::tr!("lbl-datum-axis"));
        ui.horizontal(|ui| {
          for axis in [Axis::X, Axis::Y, Axis::Z] {
            let selected = state.datum_edge_axis == axis;
            if ui.selectable_label(selected, axis.letter().to_string()).clicked() {
              state.datum_edge_axis = axis;
            }
          }
        });
        ui.end_row();
        ui.label(crate::tr!("lbl-datum-dir"));
        ui.horizontal(|ui| {
          for (dir, glyph) in [(Dir::Pos, "+"), (Dir::Neg, "−")] {
            let selected = state.datum_edge_dir == dir;
            if ui.selectable_label(selected, glyph).clicked() {
              state.datum_edge_dir = dir;
            }
          }
        });
        ui.end_row();
      });
      ui.add_enabled_ui(idle, |ui| {
        if ui.add_sized(full, egui::Button::new(crate::tr!("btn-find-edge"))).clicked() {
          sink.push(Intent::DatumEdgeStart {
            axis: state.datum_edge_axis,
            dir: state.datum_edge_dir,
            params: state.datum_bench,
          });
        }
      });

      datum_bench_params(ui, state);
      return;
    };

    // A run is active: describe the target, show the readings, and offer the step's action + Cancel.
    let target = match w.target {
      DatumTarget::Corner(c) if c.inside => crate::tr!("datum-target-corner-in"),
      DatumTarget::Corner(_) => crate::tr!("datum-target-corner-out"),
      DatumTarget::Edge { axis, .. } => crate::tr!("datum-target-edge", { axis: axis.letter().to_string() }),
    };
    ui.label(RichText::new(target).size(11.0).color(palette.text_dim));
    datum_run_readings(ui, palette, w);
    ui.add_space(6.0);
    let probing = w.is_probing();
    match w.step {
      DatumStep::EnterParams => {
        ui.label(RichText::new(crate::tr!("datum-step-enter")).size(11.0).color(palette.text_dim));
        ui.add_enabled_ui(idle && !probing, |ui| {
          if ui.add_sized(full, egui::Button::new(crate::tr!("btn-datum-probe"))).clicked() {
            sink.push(Intent::DatumProbeNext);
          }
        });
      }
      DatumStep::ReadyFaceY => {
        ui.label(RichText::new(crate::tr!("datum-step-ready-y")).size(11.0).color(palette.text_dim));
        ui.add_enabled_ui(idle && !probing, |ui| {
          if ui.add_sized(full, egui::Button::new(crate::tr!("btn-datum-probe-y"))).clicked() {
            sink.push(Intent::DatumProbeNext);
          }
        });
      }
      DatumStep::Review => {
        ui.label(RichText::new(crate::tr!("datum-step-review")).size(11.0).color(palette.state_run));
        ui.add_enabled_ui(idle, |ui| {
          if ui.add_sized(full, egui::Button::new(crate::tr!("btn-datum-write"))).clicked() {
            sink.push(Intent::DatumWriteWcs);
          }
        });
      }
      DatumStep::Aborted => {
        let reason = w.abort_reason.clone().unwrap_or_else(|| crate::tr!("reason-cancelled"));
        ui.label(RichText::new(crate::tr!("msg-aborted", { reason: reason })).size(11.0).color(palette.state_alarm));
      }
      // The probing steps await a result; only Cancel is offered.
      DatumStep::ProbeEdge | DatumStep::ProbeFaceX | DatumStep::ProbeFaceY => {
        ui.label(RichText::new(crate::tr!("msg-probing-awaiting-dot")).size(11.0).color(palette.text_dim));
      }
    }
    ui.add_space(4.0);
    if ui.add_sized(full, egui::Button::new(crate::tr!("btn-cancel-wizard"))).clicked() {
      sink.push(Intent::DatumCancel);
    }
  });
}

/// Render one corner-picker button labelled by its outside approach signs, selecting `corner` (preserving the
/// current inside/outside toggle) when clicked and highlighting the currently-picked location.
fn corner_button(ui: &mut egui::Ui, state: &mut UiState, corner: crate::app::datum::Corner, label: &str) {
  // The picked location is compared on the OUTSIDE signs alone — the inside toggle is orthogonal, so it does not
  // change which of the four buttons reads as selected.
  let selected = state.datum_corner.af_x == corner.af_x && state.datum_corner.af_y == corner.af_y;
  if ui.selectable_label(selected, label).clicked() {
    state.datum_corner = crate::app::datum::Corner { inside: state.datum_corner.inside, ..corner };
  }
}

/// Render the datum run's captured readings + computed datum: the comped edge for a single edge, or the comped
/// `(X, Y)` for a corner, once available. A thin render of the pure [`crate::app::datum::DatumState`].
fn datum_run_readings(ui: &mut egui::Ui, palette: Palette, w: &crate::app::datum::DatumState) {
  use crate::app::datum::DatumTarget;
  ui.add_space(4.0);
  match w.target {
    DatumTarget::Edge { .. } => {
      if let Some(v) = w.edge_value() {
        ui.label(RichText::new(crate::tr!("datum-reading-edge", { v: format!("{v:.3}") })).size(11.0)
          .color(palette.text));
      }
    }
    DatumTarget::Corner(_) => {
      if let Some((x, y)) = w.corner_xy() {
        ui.label(RichText::new(crate::tr!("datum-reading-x", { v: format!("{x:.3}") })).size(11.0).color(palette.text));
        ui.label(RichText::new(crate::tr!("datum-reading-y", { v: format!("{y:.3}") })).size(11.0).color(palette.text));
      }
    }
  }
}

/// Render the editable datum bench-tuned parameters in a collapsing section, mutating `state.datum_bench` in
/// place. Collapsed by default so the common path is just the corner/edge picker; opened to dial in a bench. The
/// tip diameter is the load-bearing one for accuracy (it sets the tip-radius compensation).
fn datum_bench_params(ui: &mut egui::Ui, state: &mut UiState) {
  let palette = state.style.palette;
  let p = &mut state.datum_bench;
  egui::CollapsingHeader::new(RichText::new(crate::tr!("hdr-datum-bench")).size(11.0).color(palette.text_dim))
    .id_salt("datum_bench_params")
    .show(ui, |ui| {
      egui::Grid::new("datum_bench").num_columns(2).show(ui, |ui| {
        ui.label(crate::tr!("lbl-tip-dia"));
        ui.add(egui::DragValue::new(&mut p.probe_diameter).speed(0.01).range(0.0..=50.0).suffix(" mm"));
        ui.end_row();
        ui.label(crate::tr!("lbl-xy-clearance"));
        ui.add(egui::DragValue::new(&mut p.xy_clearance).speed(0.1).range(0.0..=100.0).suffix(" mm"));
        ui.end_row();
        ui.label(crate::tr!("lbl-probe-distance"));
        ui.add(egui::DragValue::new(&mut p.probe_distance).speed(0.5).range(0.1..=200.0).suffix(" mm"));
        ui.end_row();
        ui.label(crate::tr!("lbl-latch-distance"));
        ui.add(egui::DragValue::new(&mut p.latch_distance).speed(0.1).range(0.1..=50.0).suffix(" mm"));
        ui.end_row();
        ui.label(crate::tr!("lbl-datum-probe-feed"));
        ui.add(egui::DragValue::new(&mut p.probe_feed).speed(5.0).range(1.0..=5000.0).suffix(" mm/min"));
        ui.end_row();
        ui.label(crate::tr!("lbl-latch-feed"));
        ui.add(egui::DragValue::new(&mut p.latch_feed).speed(1.0).range(1.0..=2000.0).suffix(" mm/min"));
        ui.end_row();
        ui.label(crate::tr!("lbl-corner-offset"));
        ui.add(egui::DragValue::new(&mut p.offset).speed(0.1).range(0.0..=100.0).suffix(" mm"));
        ui.end_row();
      });
    });
}
