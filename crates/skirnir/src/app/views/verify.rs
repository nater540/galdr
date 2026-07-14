//! The Phase 2 verify / measure panel (flip-verify + runout) and its readings table.

use super::*;

/// Render the Phase 2 verify/measure panel (DOC-11 §2): the 180°-flip center-verify and the runout report, both
/// driven by the shared [`crate::app::angle_sweep::AngleSweep`] engine (passed as `(sweep, kind)` when one is running).
/// When idle it offers both Start actions; while running it guides the per-angle touches and shows the readings;
/// on completion it computes the flip residual (with a `G10` correction offer) or the runout TIR/eccentricity
/// (read-only). The probes use the conventional Y radial axis (matching the center-finder), probing toward −Y.
pub fn verify_measure(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState,
  sweep: Option<(&crate::app::angle_sweep::AngleSweep, crate::app::view_state::ProbeKind)>, sink: &mut IntentSink) {
  let palette = state.style.palette;
  use crate::app::angle_sweep::SweepStep;
  use crate::app::intent::{Axis, Dir};
  use crate::app::view_state::ProbeKind;
  egui::Frame::new().inner_margin(Metrics::RIGHT_PAD).show(ui, |ui| {
    let full = Vec2::new(ui.available_width(), Metrics::PANEL_CONTROL_H + 6.0);
    let idle = view.connection == ConnectionState::Idle;
    let Some((s, kind)) = sweep else {
      // No run: collect the shared start angle + (for runout) N, and offer both Start actions.
      ui.label(RichText::new(crate::tr!("verify-intro")).size(11.0).color(palette.text_dim));
      ui.add_space(4.0);
      egui::Grid::new("verify_setup").num_columns(2).show(ui, |ui| {
        ui.label(crate::tr!("lbl-start-a"));
        ui.add(egui::DragValue::new(&mut state.verify_start_angle).speed(1.0).range(-360.0..=360.0).suffix(" °"));
        ui.end_row();
        ui.label(crate::tr!("lbl-runout-n"));
        ui.add(egui::DragValue::new(&mut state.verify_runout_n).range(2..=36));
        ui.end_row();
      });
      ui.add_enabled_ui(idle, |ui| {
        if ui.add_sized(full, egui::Button::new(crate::tr!("btn-start-flip"))).clicked() {
          sink.push(Intent::FlipVerifyStart { angle_deg: state.verify_start_angle, axis: Axis::Y, dir: Dir::Neg });
        }
        if ui.add_sized(full, egui::Button::new(crate::tr!("btn-start-runout"))).clicked() {
          sink.push(Intent::RunoutStart {
            n: state.verify_runout_n,
            start_deg: state.verify_start_angle,
            axis: Axis::Y,
            dir: Dir::Neg,
          });
        }
      });
      return;
    };

    let title = match kind {
      ProbeKind::FlipVerify => crate::tr!("verify-title-flip"),
      ProbeKind::Runout => crate::tr!("verify-title-runout"),
      _ => crate::tr!("verify-title-generic"),
    };
    ui.label(RichText::new(title).size(11.0).color(palette.text));
    verify_readings_table(ui, palette, s);
    ui.add_space(6.0);
    match s.step() {
      SweepStep::Ready => {
        ui.label(RichText::new(crate::tr!("verify-ready",
          { current: s.current_touch_number() as i64, total: s.total_touches() as i64 }))
          .size(11.0).color(palette.text_dim));
        ui.add_enabled_ui(idle, |ui| {
          if ui.add_sized(full, egui::Button::new(crate::tr!("btn-probe-angle"))).clicked() {
            sink.push(Intent::SweepProbe);
          }
        });
      }
      SweepStep::Probing => {
        ui.label(RichText::new(crate::tr!("msg-probing-awaiting-dot")).size(11.0).color(palette.text_dim));
      }
      SweepStep::Done => verify_done(ui, state, s, kind, idle, full, sink),
      SweepStep::Aborted => {
        let reason = s.abort_reason().map(String::from).unwrap_or_else(|| crate::tr!("reason-cancelled"));
        ui.label(RichText::new(crate::tr!("msg-aborted", { reason: reason })).size(11.0).color(palette.state_alarm));
      }
    }
    ui.add_space(4.0);
    if ui.add_sized(full, egui::Button::new(crate::tr!("btn-cancel-wizard"))).clicked() {
      sink.push(Intent::SweepCancel);
    }
  });
}

/// Render the completed-sweep result: the flip-verify residual + `G10` correction offer, or the read-only runout
/// TIR / eccentricity. Pure render of the computed values over the sweep's readings.
fn verify_done(ui: &mut egui::Ui, _state: &mut UiState, s: &crate::app::angle_sweep::AngleSweep,
  kind: crate::app::view_state::ProbeKind, idle: bool, full: Vec2, sink: &mut IntentSink) {
  let palette = _state.style.palette;
  use crate::app::flip_verify::FlipResult;
  use crate::app::runout::RunoutReport;
  use crate::app::view_state::ProbeKind;
  match kind {
    ProbeKind::FlipVerify => {
      let Some(result) = FlipResult::from_readings(s.probe_axis(), s.readings()) else {
        ui.label(RichText::new(crate::tr!("verify-flip-need-two")).size(11.0).color(palette.state_alarm));
        return;
      };
      ui.label(RichText::new(crate::tr!("verify-residual", { mm: format!("{:.3}", result.error()) }))
        .size(11.0).color(palette.state_run));
      ui.label(RichText::new(crate::tr!("verify-apply-desc")).size(11.0).color(palette.text_dim));
      ui.add_enabled_ui(idle, |ui| {
        if ui.add_sized(full, egui::Button::new(crate::tr!("btn-apply-correction"))).clicked() {
          sink.push(Intent::FlipVerifyWriteCorrection);
        }
      });
    }
    ProbeKind::Runout => match RunoutReport::from_readings(s.readings()) {
      Some(r) => {
        ui.label(RichText::new(crate::tr!("verify-runout-result",
          { tir: format!("{:.3}", r.tir), ecc: format!("{:.3}", r.eccentricity), count: r.count as i64 }))
          .size(11.0).color(palette.state_run));
        ui.label(RichText::new(crate::tr!("verify-runout-readonly")).size(11.0).color(palette.text_dim));
      }
      None => {
        ui.label(RichText::new(crate::tr!("verify-runout-need-two")).size(11.0).color(palette.state_alarm));
      }
    },
    _ => {}
  }
}

/// Render the sweep's per-angle readings as a compact dim list (angle → reading once captured). Pure render of
/// the shared [`crate::app::angle_sweep::AngleSweep`].
fn verify_readings_table(ui: &mut egui::Ui, palette: Palette, s: &crate::app::angle_sweep::AngleSweep) {
  let readings = s.readings();
  for (i, &angle) in s.angles().iter().enumerate() {
    let text = match readings.get(i) {
      Some(r) => format!("A{angle:.1}°  →  {r:.3}"),
      None => format!("A{angle:.1}°  →  —"),
    };
    ui.label(RichText::new(text).size(11.0).color(palette.text_dim));
  }
}
