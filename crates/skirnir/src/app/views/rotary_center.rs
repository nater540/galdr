//! The rotary center-finder wizard panel and its bench-param / readings sub-views.

use super::*;

/// Render the rotary center-finder wizard (DOC-11 §1.2). When no run is active it shows the dowel-diameter /
/// index-angle inputs and a Start button; when a run is active it guides the operator step by step — issuing each
/// rotary-safe touch, the move-to-Y-center, and the WCS write — and shows the captured readings + computed
/// `(Y_c, Z_c)`. The wizard state is owned by the shell (the firmware has no pivot concept) and passed in as a
/// borrow, so this stays a pure render that only emits intents.
pub fn rotary_center(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState,
  wizard: Option<&crate::app::rotary_center::WizardState>, has_saved_center: bool, sink: &mut IntentSink) {
  let palette = state.style.palette;
  use crate::app::rotary_center::WizardStep;
  egui::Frame::new().inner_margin(Metrics::RIGHT_PAD).show(ui, |ui| {
    let Some(w) = wizard else {
      // No run: collect the dowel diameter + index angle and offer Start. Only meaningful while idle/connected,
      // but the inputs stay editable so the operator can set up before connecting.
      ui.label(RichText::new(crate::tr!("rotary-intro")).size(11.0).color(palette.text_dim));
      ui.add_space(4.0);
      egui::Grid::new("rotary_setup").num_columns(2).show(ui, |ui| {
        ui.label(crate::tr!("lbl-dowel-dia"));
        ui.add(egui::DragValue::new(&mut state.rotary_dowel_diameter).speed(0.1).range(0.1..=100.0).suffix(" mm"));
        ui.end_row();
        ui.label(crate::tr!("lbl-a-angle"));
        ui.add(egui::DragValue::new(&mut state.rotary_index_angle).speed(1.0).range(-360.0..=360.0).suffix(" °"));
        ui.end_row();
        // The side-probe Z is SAFETY-CRITICAL and setup-specific, so it lives in the always-visible setup rather
        // than the collapsed bench section: every center-finder run descends a side touch to it, and a wrong value
        // crashes into the part or misses the flank (finding #13). Editing it re-arms the confirmation below.
        ui.label(crate::tr!("lbl-side-probe-z"));
        if ui.add(egui::DragValue::new(&mut state.rotary_bench.side_probe_z).speed(0.1).range(-300.0..=0.0)
          .suffix(" mm")).on_hover_text(crate::tr!("tip-side-probe-z")).changed()
        {
          state.rotary_side_probe_confirmed = false;
        }
        ui.end_row();
      });
      // The remaining bench-tuned parameters stay in a collapsing section so the common path is just the inputs
      // above. The side-probe Z is hoisted out of it (above) because it is the crash-risk parameter.
      rotary_bench_params(ui, state);
      // Gate Start on an explicit acknowledgement that the side-probe Z is tuned for THIS bench. The default is a
      // conservative placeholder; an untuned descent is a crash risk, so the operator must confirm before a run.
      ui.add_space(4.0);
      ui.checkbox(&mut state.rotary_side_probe_confirmed,
        RichText::new(crate::tr!("rotary-side-confirm", { z: format!("{:.3}", state.rotary_bench.side_probe_z) }))
          .size(11.0).color(palette.text_dim));
      let enabled = view.connection == ConnectionState::Idle && state.rotary_side_probe_confirmed;
      ui.add_enabled_ui(enabled, |ui| {
        if ui.add_sized(Vec2::new(ui.available_width(), Metrics::PANEL_CONTROL_H + 6.0),
          egui::Button::new(crate::tr!("btn-start-center"))).clicked()
        {
          sink.push(Intent::RotaryCenterStart {
            dowel_diameter: state.rotary_dowel_diameter,
            index_angle_deg: state.rotary_index_angle,
            params: state.rotary_bench,
          });
        }
      });
      // If a center was saved last session (DOC-11 §1.3), offer to re-apply it to the active WCS without
      // re-running the center-finder. Enabled only when Idle (the `G10` needs an accepting machine).
      if has_saved_center {
        ui.add_space(4.0);
        ui.add_enabled_ui(enabled, |ui| {
          if ui.add_sized(Vec2::new(ui.available_width(), Metrics::PANEL_CONTROL_H + 6.0),
            egui::Button::new(crate::tr!("btn-apply-saved-center"))).clicked()
          {
            sink.push(Intent::ApplySavedRotaryCenter);
          }
        });
      }
      return;
    };

    // A run is active: render the step guidance, the readings so far, and the step's action button. Probing
    // disables the action (one touch at a time); the latch's awaiting/result is shown by the probe panel above.
    rotary_run_readings(ui, palette, w);
    ui.add_space(6.0);
    let probing = w.is_probing();
    let idle = view.connection == ConnectionState::Idle;
    // The action available depends on the step; each is gated on Idle (a probe/move needs an accepting machine)
    // and disabled while a touch is in flight.
    let full = Vec2::new(ui.available_width(), Metrics::PANEL_CONTROL_H + 6.0);
    match w.step {
      WizardStep::EnterDowel => {
        ui.label(RichText::new(crate::tr!("rotary-step-enter-dowel")).size(11.0).color(palette.text_dim));
        ui.add_enabled_ui(idle && !probing, |ui| {
          if ui.add_sized(full, egui::Button::new(crate::tr!("btn-probe-y-left"))).clicked() {
            sink.push(Intent::RotaryCenterProbe);
          }
        });
      }
      WizardStep::ReadyYRight => {
        ui.label(RichText::new(crate::tr!("rotary-step-ready-yright")).size(11.0).color(palette.text_dim));
        ui.add_enabled_ui(idle && !probing, |ui| {
          if ui.add_sized(full, egui::Button::new(crate::tr!("btn-probe-y-right"))).clicked() {
            sink.push(Intent::RotaryCenterProbe);
          }
        });
      }
      WizardStep::MoveToYc => {
        // ONLY the move is offered here — the top probe is locked until the move has actually been sent (the
        // wizard then advances to MovedToYc). This is the UI half of the type-enforced "move before top" order.
        ui.label(RichText::new(crate::tr!("rotary-step-move-yc")).size(11.0).color(palette.text_dim));
        ui.add_enabled_ui(idle, |ui| {
          if ui.add_sized(full, egui::Button::new(crate::tr!("btn-move-yc"))).clicked() {
            sink.push(Intent::RotaryCenterMoveToYc);
          }
        });
      }
      WizardStep::MovedToYc => {
        // The move was sent; now (and only now) the top probe is offered.
        ui.label(RichText::new(crate::tr!("rotary-step-moved-yc")).size(11.0).color(palette.text_dim));
        ui.add_enabled_ui(idle && !probing, |ui| {
          if ui.add_sized(full, egui::Button::new(crate::tr!("btn-probe-z-top"))).clicked() {
            sink.push(Intent::RotaryCenterProbe);
          }
        });
      }
      WizardStep::Review => {
        ui.label(RichText::new(crate::tr!("rotary-step-review")).size(11.0).color(palette.state_run));
        // The operator picks which feature work-Z0 lands on. Y0 is always the axis centerline; only Z is
        // selectable. Defaults to the axis centerline (wrap-machining convention).
        rotary_z_datum_picker(ui, palette, w, sink);
        ui.add_enabled_ui(idle, |ui| {
          if ui.add_sized(full, egui::Button::new(crate::tr!("btn-write-center"))).clicked() {
            sink.push(Intent::RotaryCenterWriteWcs);
          }
        });
      }
      WizardStep::Aborted => {
        let reason = w.abort_reason.clone().unwrap_or_else(|| crate::tr!("reason-cancelled"));
        ui.label(RichText::new(crate::tr!("msg-aborted", { reason: reason })).size(11.0).color(palette.state_alarm));
      }
      // The probing steps await a result (the probe panel shows it); only Cancel is offered here.
      WizardStep::ProbeYLeft | WizardStep::ProbeYRight | WizardStep::ProbeZTop => {
        ui.label(RichText::new(crate::tr!("msg-probing-awaiting-dot")).size(11.0).color(palette.text_dim));
      }
    }
    ui.add_space(4.0);
    if ui.add_sized(full, egui::Button::new(crate::tr!("btn-cancel-wizard"))).clicked() {
      sink.push(Intent::RotaryCenterCancel);
    }
  });
}

/// Render the editable bench-tuned rotary-touch parameters in a collapsing "Bench params" section, mutating
/// `state.rotary_bench` in place. Collapsed by default so the common setup is just dowel ⌀ + A angle; opened when
/// the operator needs to dial in a bench. The crash-risk side-probe Z is NOT here — it is hoisted into the
/// always-visible setup grid and gated behind a confirmation (finding #13); this section holds the rest.
fn rotary_bench_params(ui: &mut egui::Ui, state: &mut UiState) {
  let palette = state.style.palette;
  let p = &mut state.rotary_bench;
  egui::CollapsingHeader::new(RichText::new(crate::tr!("hdr-bench-params")).size(11.0).color(palette.text_dim))
    .id_salt("rotary_bench_params")
    .show(ui, |ui| {
      egui::Grid::new("rotary_bench").num_columns(2).show(ui, |ui| {
        ui.label(crate::tr!("lbl-clearance-z"));
        ui.add(egui::DragValue::new(&mut p.clearance_mm).speed(0.1).range(-300.0..=0.0).suffix(" mm"))
          .on_hover_text(crate::tr!("tip-clearance-z"));
        ui.end_row();
        // Side-probe Z is intentionally NOT here — it is hoisted into the always-visible setup grid above (finding
        // #13) because it is the crash-risk parameter and must be confirmed before a run, not buried in a collapse.
        ui.label(crate::tr!("lbl-settle"));
        ui.add(egui::DragValue::new(&mut p.settle_secs).speed(0.05).range(0.0..=10.0).suffix(" s"))
          .on_hover_text(crate::tr!("tip-settle"));
        ui.end_row();
        ui.label(crate::tr!("lbl-feed"));
        ui.add(egui::DragValue::new(&mut p.feed).speed(1.0).range(1.0..=2000.0).suffix(" mm/min"))
          .on_hover_text(crate::tr!("tip-bench-feed"));
        ui.end_row();
        ui.label(crate::tr!("lbl-depth"));
        ui.add(egui::DragValue::new(&mut p.depth_mm).speed(0.1).range(0.1..=200.0).suffix(" mm"))
          .on_hover_text(crate::tr!("tip-bench-depth"));
        ui.end_row();
      });
    });
}

/// Render the rotary wizard's captured readings + computed center as a compact dim list. Each is shown once
/// available; `Y_c`/`Z_c` appear as the math resolves them. Pure render of [`crate::app::rotary_center::WizardState`].
fn rotary_run_readings(ui: &mut egui::Ui, palette: Palette, w: &crate::app::rotary_center::WizardState) {
  let dim = |ui: &mut egui::Ui, text: String| {
    ui.label(RichText::new(text).size(11.0).color(palette.text_dim));
  };
  dim(ui, crate::tr!("rotary-reading-dowel",
    { dia: format!("{:.3}", w.dowel_diameter), angle: format!("{:.1}", w.index_angle_deg) }));
  if let Some(y) = w.y_left {
    dim(ui, crate::tr!("rotary-reading-y-left", { v: format!("{y:.3}") }));
  }
  if let Some(y) = w.y_right {
    dim(ui, crate::tr!("rotary-reading-y-right", { v: format!("{y:.3}") }));
  }
  if let Some(yc) = w.y_center() {
    ui.label(RichText::new(crate::tr!("rotary-reading-y-center", { v: format!("{yc:.3}") })).size(11.0)
      .color(palette.text));
  }
  if let Some(z) = w.z_top {
    dim(ui, crate::tr!("rotary-reading-z-top", { v: format!("{z:.3}") }));
  }
  if let Some(zc) = w.z_center() {
    ui.label(RichText::new(crate::tr!("rotary-reading-z-center", { v: format!("{zc:.3}") })).size(11.0)
      .color(palette.text));
  }
}

/// Render the Z-datum picker for the WCS write: two selectable labels — the rotary axis centerline (default) or
/// the probed top surface — plus a one-line clarification and a preview of which Z the offered `G10` will use.
/// Emits [`Intent::RotaryCenterSetZDatum`] on a change; pure render of the wizard's current selection.
fn rotary_z_datum_picker(ui: &mut egui::Ui, palette: Palette, w: &crate::app::rotary_center::WizardState, sink: &mut IntentSink) {
  use crate::app::rotary_center::ZDatum;
  ui.add_space(4.0);
  ui.label(RichText::new(crate::tr!("lbl-z0-datum")).size(11.0).color(palette.text_dim));
  ui.horizontal(|ui| {
    let axis = w.z_datum == ZDatum::AxisCenterline;
    let top = w.z_datum == ZDatum::TopSurface;
    if ui.selectable_label(axis, crate::tr!("z-datum-axis")).clicked() && !axis {
      sink.push(Intent::RotaryCenterSetZDatum(ZDatum::AxisCenterline));
    }
    if ui.selectable_label(top, crate::tr!("z-datum-top")).clicked() && !top {
      sink.push(Intent::RotaryCenterSetZDatum(ZDatum::TopSurface));
    }
  });
  // One-line clarification of the selected datum, plus the Z value the G10 will carry.
  let (desc, z) = match w.z_datum {
    ZDatum::AxisCenterline => (crate::tr!("z-datum-axis-desc"), w.z_datum_value()),
    ZDatum::TopSurface => (crate::tr!("z-datum-top-desc"), w.z_datum_value()),
  };
  ui.label(RichText::new(desc).size(11.0).color(palette.text_dim));
  if let Some(z) = z {
    ui.label(RichText::new(crate::tr!("z-datum-g10", { z: format!("{z:.3}") })).size(11.0).color(palette.text_dim));
  }
}
