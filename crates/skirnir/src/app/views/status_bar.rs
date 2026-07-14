//! The bottom status bar plus the alarm and tool-change banners.

use super::*;

/// Render the bottom status bar: the design's mono info strip (port dot · state · WCO · `Ln a/b · NN%` · F·S).
pub fn status_bar(ui: &mut egui::Ui, view: &ViewState, state: &UiState) {
  let palette = state.style.palette;
  // Fabulous mode: a matching Pride band along the strip's top edge, bookending the toolbar's. Painted before
  // the content so the mono text sits above it; the band is thin enough not to crowd the row.
  if state.fabulous {
    let bar = ui.max_rect();
    let stripe = egui::Rect::from_min_max(
      egui::pos2(bar.left(), bar.top()),
      egui::pos2(bar.right(), bar.top() + Metrics::PRIDE_STRIPE_H),
    );
    paint_pride(ui.painter(), stripe);
  }
  ui.horizontal(|ui| {
    let badge = view.badge_state();
    dot(ui, palette.badge_color(badge), Metrics::STATUS_DOT);
    let port = if state.selected_port.is_empty() { "—" } else { &state.selected_port };
    ui.label(RichText::new(port).monospace().size(10.5).color(palette.text_dim));
    ui.label(RichText::new("·").color(palette.text_disabled));
    ui.label(RichText::new(crate::tr!(badge.label_key())).monospace().size(10.5).color(palette.text));

    if !view.last_wco.is_empty() {
      ui.label(RichText::new("·").color(palette.text_disabled));
      ui.label(RichText::new(crate::tr!("status-wco-set")).monospace().size(10.5).color(palette.text_dim));
    }
    if view.progress.total > 0 {
      ui.label(RichText::new("·").color(palette.text_disabled));
      let pct = (view.progress.fraction() * 100.0).round() as u32;
      ui.label(RichText::new(crate::tr!("status-line",
        { acked: view.progress.acked as i64, total: view.progress.total as i64, pct: pct as i64 }))
        .monospace().size(10.5).color(palette.text_dim));
    }
    if let Some((feed, rpm, _)) = view.status.as_ref().and_then(|s| s.feed_speed) {
      ui.label(RichText::new("·").color(palette.text_disabled));
      ui.label(RichText::new(format!("F {feed:.0} · S {rpm:.0}")).monospace().size(10.5).color(palette.text_dim));
    }
  });
}

/// Render the alarm/error banner: a full-width strip under the toolbar (design §04). An alarm shows the code, a
/// human gloss, and the Unlock-$X / Soft-reset recovery actions; a stream error shows the code, gloss, and a
/// reset/dismiss. The copy comes from the pure [`crate::app::badge`] detail tables.
pub fn alarm_banner(ui: &mut egui::Ui, palette: Palette, view: &ViewState, sink: &mut IntentSink) {
  let Some(banner) = &view.banner else {
    return;
  };
  // Headline + secondary detail per banner kind; both share the alarm surface so the strip reads as a fault. The
  // detail resolves through the live codebook so an enumerated (`$EA`/`$EE`) description beats the static text;
  // absent enrichment, the codebook's static fallback still yields a full sentence rather than a bare number. We
  // run every repaint while the banner is shown, so we take ONLY the description via the borrowing accessor — it
  // hands back a `'static` borrow on the static path (no per-frame allocation) and clones only on an override.
  let (headline, detail, is_alarm) = match banner {
    Banner::Alarm(code) => {
      (crate::tr!("banner-alarm", { code: *code as i64 }), view.codes.alarm_description(*code), true)
    }
    Banner::StreamError(code) => {
      (crate::tr!("banner-error", { code: *code as i64 }), view.codes.error_description(*code), false)
    }
  };
  egui::Frame::new()
    .fill(palette.alarm_bg)
    .stroke(egui::Stroke::new(1.0, palette.alarm_border))
    .inner_margin(egui::Margin::symmetric(14, 10))
    .show(ui, |ui| {
      ui.horizontal(|ui| {
        ui.vertical(|ui| {
          ui.label(RichText::new(&headline).color(palette.alarm_text).strong());
          ui.label(RichText::new(detail).size(11.5).color(palette.text_dim));
        });
        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
          if ui.button(crate::tr!("btn-dismiss")).clicked() {
            sink.push(Intent::DismissBanner);
          }
          let reset = egui::Button::new(RichText::new(crate::tr!("btn-soft-reset")).color(Color32::WHITE))
            .fill(palette.state_alarm);
          if ui.add(reset).on_hover_text(crate::tr!("tip-soft-reset")).clicked() {
            sink.push(Intent::Realtime(RealtimeCommand::SoftReset));
          }
          // Only an alarm offers the $X unlock; a stream error clears on reset / `$` / an empty line.
          if is_alarm && ui.button(crate::tr!("btn-unlock")).on_hover_text(crate::tr!("tip-unlock")).clicked() {
            sink.push(Intent::SendLine("$X".to_string()));
          }
        });
      });
    });
}

/// The tool-change banner headline for a given active-tool value. `T0` ("none") and an as-yet-unreported tool
/// both fall back to a generic prompt rather than naming a misleading number, so the copy is always truthful.
/// Pulled out of [`tool_change_banner`] so the wording decision is a pure, testable function. `pub(crate)` so the
/// shell's integration tests can assert the exact banner copy for a resolved tool.
pub(crate) fn tool_change_headline(current_tool: Option<u32>) -> String {
  match current_tool {
    Some(tool) if tool != 0 => crate::tr!("banner-tool-with", { tool: tool as i64 }),
    _ => crate::tr!("banner-tool-generic"),
  }
}

/// Render the tool-change affordance: a full-width attention strip shown while the firmware is held for an M6
/// manual tool change (`<Tool|...>`). It names the tool to insert from the firmware's `$G`/`[GC:]`-reported tool
/// ([`ViewState::current_tool`]) — answered during the hold per the firmware's M0/M1/M6 `$G`-in-hold support — and
/// offers a Resume that issues the cycle-start (`~`) through the existing run/resume path, never a second pathway.
/// Drawn in the banner slot, distinct from the alarm surface: violet (attention), not red, so it never reads as a
/// fault. A fault banner, if latched, takes precedence (the shell shows this only when none is).
pub fn tool_change_banner(ui: &mut egui::Ui, palette: Palette, view: &ViewState, sink: &mut IntentSink) {
  let headline = tool_change_headline(view.current_tool);
  egui::Frame::new()
    .fill(palette.inset)
    .stroke(egui::Stroke::new(1.0, palette.state_check))
    .inner_margin(egui::Margin::symmetric(14, 10))
    .show(ui, |ui| {
      ui.horizontal(|ui| {
        ui.vertical(|ui| {
          ui.label(RichText::new(&headline).color(palette.state_check).strong());
          ui.label(RichText::new(crate::tr!("banner-tool-detail")).size(11.5).color(palette.text_dim));
        });
        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
          // Resume reuses the single run/resume intent — the same `~` cycle-start the toolbar's Resume segment and
          // the keyboard hotkey issue — so there is exactly one resume pathway.
          let resume = egui::Button::new(RichText::new(crate::tr!("transport-resume")).color(Color32::WHITE))
            .fill(palette.state_run);
          if ui.add(resume).on_hover_text(crate::tr!("tip-resume-tool")).clicked() {
            sink.push(Intent::RunOrResume);
          }
        });
      });
    });
}
