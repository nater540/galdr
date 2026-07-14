//! The Z touch-off / probe panel and its result read-out.

use super::*;

/// Render the probe panel: depth/feed/plate inputs and a "Probe Z" action that the shell sequences.
pub fn probe(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  let palette = state.style.palette;
  egui::Frame::new().inner_margin(Metrics::RIGHT_PAD).show(ui, |ui| {
    ui.label(RichText::new(crate::tr!("probe-intro")).size(11.0).color(palette.text_dim));
    ui.add_space(4.0);
    let enabled = view.connection == ConnectionState::Idle;
    ui.add_enabled_ui(enabled, |ui| {
      egui::Grid::new("probe").num_columns(2).show(ui, |ui| {
        ui.label(crate::tr!("lbl-depth"));
        ui.add(egui::DragValue::new(&mut state.probe_depth).speed(0.5).range(0.1..=200.0).suffix(" mm"));
        ui.end_row();
        ui.label(crate::tr!("lbl-feed"));
        ui.add(egui::DragValue::new(&mut state.probe_feed).speed(5.0).range(1.0..=500.0).suffix(" mm/min"));
        ui.end_row();
        ui.label(crate::tr!("lbl-plate"));
        ui.add(egui::DragValue::new(&mut state.plate_thickness).speed(0.05).range(0.0..=20.0).suffix(" mm"));
        ui.end_row();
      });
      // The primary probe action is a full-width button, per the design's right-column treatment.
      let probe_button = egui::Button::new(crate::tr!("btn-probe-z"));
      if ui.add_sized(Vec2::new(ui.available_width(), Metrics::PANEL_CONTROL_H + 6.0), probe_button).clicked() {
        sink.push(Intent::ProbeZ {
          depth: state.probe_depth,
          feed: state.probe_feed,
          plate_thickness: state.plate_thickness,
        });
      }
    });
    // Render the latched probe result below the action so the operator sees the probed point + success here
    // rather than hunting for it in the console. Shows a live "Probing…" while awaiting, the contact point on
    // success, or the failure reason — driven entirely by the reducer's `probe_op` latch.
    probe_result(ui, palette, view);
  });
}

/// Render the probe-operation latch outcome inside the Z touch-off panel: a "Probing…" spinner-line while
/// awaiting, the contact point on success, or the failure reason. Reads only [`ViewState::probe_op`] — a thin
/// render of the pure latch. Draws nothing when no probe has been issued, OR when the latched op belongs to a
/// DIFFERENT flow (a rotary wizard touch): this panel speaks only for the ZeroZ touch-off, so it must not claim
/// "Work-Z set." for a rotary probe (the wizard has its own panel). Routing on `op.kind` is what keeps the two
/// panels from narrating each other's probes.
fn probe_result(ui: &mut egui::Ui, palette: Palette, view: &ViewState) {
  use crate::app::view_state::{ProbeKind, ProbeOutcome};
  let Some(op) = view.probe_op.as_ref().filter(|op| op.kind == ProbeKind::ZeroZ) else {
    return;
  };
  ui.add_space(6.0);
  if op.awaiting {
    ui.label(RichText::new(crate::tr!("msg-probing-awaiting")).size(11.0).color(palette.text_dim));
    return;
  }
  match op.last.as_ref() {
    Some(ProbeOutcome::Success { position }) => {
      // Show the machine-coordinate contact point (X, Y, Z, then any rotary axis) at 3 decimals, the PRB report
      // precision. A short green confirmation reads as "done" without re-reading the console.
      let coords = position.iter().map(|v| format!("{v:.3}")).collect::<Vec<_>>().join(", ");
      ui.label(RichText::new(crate::tr!("probe-contact", { coords: coords })).size(11.0).color(palette.state_run));
      ui.label(RichText::new(crate::tr!("probe-work-z-set")).size(11.0).color(palette.text_dim));
    }
    Some(ProbeOutcome::Failure { reason }) => {
      ui.label(RichText::new(crate::tr!("probe-failed", { reason: reason.clone() })).size(11.0)
        .color(palette.state_alarm));
      ui.label(RichText::new(crate::tr!("probe-work-z-unchanged")).size(11.0).color(palette.text_dim));
    }
    // Resolved but no outcome recorded — unreachable in practice (resolving always sets `last`), but render
    // nothing rather than assume.
    None => {}
  }
}
