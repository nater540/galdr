//! The Z touch-off / probe panel and its result read-out.

use super::*;

/// Render the probe panel: depth/feed/plate inputs and a "Probe Z" action that the shell sequences.
pub fn probe(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  let palette = state.style.palette;
  right_panel(ui, |ui| {
    dim_label(ui, palette, crate::tr!("probe-intro"));
    ui.add_space(4.0);
    let enabled = view.connection == ConnectionState::Idle;
    ui.add_enabled_ui(enabled, |ui| {
      egui::Grid::new("probe").num_columns(2).show(ui, |ui| {
        param_row(ui, crate::tr!("lbl-depth"), &mut state.probe_depth, 0.5, 0.1..=200.0, " mm");
        param_row(ui, crate::tr!("lbl-feed"), &mut state.probe_feed, 5.0, 1.0..=500.0, " mm/min");
        param_row(ui, crate::tr!("lbl-plate"), &mut state.plate_thickness, 0.05, 0.0..=20.0, " mm");
      });
      // The primary probe action is a full-width button, per the design's right-column treatment.
      if full_width_button(ui, crate::tr!("btn-probe-z")).clicked() {
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
    dim_label(ui, palette, crate::tr!("msg-probing-awaiting"));
    return;
  }
  match op.last.as_ref() {
    Some(ProbeOutcome::Success { position }) => {
      // Show the machine-coordinate contact point (X, Y, Z, then any rotary axis) at 3 decimals, the PRB report
      // precision. A short green confirmation reads as "done" without re-reading the console.
      let coords = position.iter().map(|v| format!("{v:.3}")).collect::<Vec<_>>().join(", ");
      ui.label(RichText::new(crate::tr!("probe-contact", { coords: coords })).size(11.0).color(palette.state_run));
      dim_label(ui, palette, crate::tr!("probe-work-z-set"));
    }
    Some(ProbeOutcome::Failure { reason }) => {
      ui.label(RichText::new(crate::tr!("probe-failed", { reason: reason.clone() })).size(11.0)
        .color(palette.state_alarm));
      dim_label(ui, palette, crate::tr!("probe-work-z-unchanged"));
    }
    // Resolved but no outcome recorded — unreachable in practice (resolving always sets `last`), but render
    // nothing rather than assume.
    None => {}
  }
}
