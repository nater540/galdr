//! The digital read-out: work/machine position, endstop chips, and the position-mode toggle.

use super::*;

/// Render the digital readout: a WPos/MPos toggle, large per-axis rows (coloured letter + big tabular value +
/// unit), the Zero X/Y/Z/XYZ button row, and the WCO strip (design §03).
pub fn dro(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  let palette = state.style.palette;
  header_bar(
    ui, palette,
    |ui| header_title(ui, palette, &crate::tr!("hdr-dro")),
    |ui| {
      // The WPos/MPos toggle sits inside the header bar (design §03); the small joined buttons pick which
      // coordinate the big readout shows. The design defaults to WPos. `right_to_left` lays MPos then WPos so
      // they read left→right as WPos · MPos.
      ui.spacing_mut().item_spacing.x = 1.0;
      ui.spacing_mut().button_padding = Metrics::DRO_TOGGLE_PAD;
      pos_toggle(ui, state, true, "MPos");
      pos_toggle(ui, state, false, "WPos");
    },
  );

  egui::Frame::new().inner_margin(Metrics::DRO_PAD).show(ui, |ui| {
  let (machine, work) = view.dro();
  // A rotary-enabled firmware reports 4-field positions (DOC-10); grow the readout an A row (in degrees) exactly
  // when the report carries one, so a plain 3-axis board never shows a phantom rotary. The shared
  // `reported_axis_count` is the same gate the jog pad's A column uses, so the two never disagree about the rotary.
  let axis_count = view.reported_axis_count();
  let shown = if state.show_machine_pos { machine.as_ref() } else { work.as_ref() };
  let axes: &[Axis] =
    if axis_count >= 4 { &[Axis::X, Axis::Y, Axis::Z, Axis::A] } else { &[Axis::X, Axis::Y, Axis::Z] };
  for axis in axes {
    ui.horizontal(|ui| {
      ui.label(RichText::new(axis.letter().to_string()).size(Metrics::DRO_LETTER).strong()
        .color(palette.axis_color(*axis)));
      ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
        let unit = if *axis == Axis::A { "°" } else { "mm" };
        ui.label(RichText::new(unit).monospace().size(Metrics::DRO_UNIT).color(palette.text_disabled));
        ui.label(big_axis_value(palette, shown, axis.index()));
      });
    });
    ui.add_space(Metrics::DRO_ROW_GAP - ui.spacing().item_spacing.y);
  }

  ui.add_space(4.0);
  // Zero buttons set the active WCS origin to the current position on the chosen axes. The first three are
  // equal-width; the design gives Zero XYZ a wider (flex 1.4), primary-blue slot, so it draws the eye. They are
  // placed at exact, shared-edge rects via [`button_row`] so the four sit on one horizontal line — the previous
  // `add_sized`-in-`horizontal` layout crept each successive button a couple of pixels lower (the staggered row
  // the user flagged).
  let enabled = view.connection == ConnectionState::Idle;
  ui.add_enabled_ui(enabled, |ui| {
    // Trim the horizontal button padding from egui's default 14px so the labels fit their narrow placed cells
    // (at the default, "Zero XYZ" had no room and wrapped onto two lines inside its slot).
    ui.spacing_mut().button_padding = Vec2::new(6.0, 0.0);
    button_row(ui, 6.0, &[1.0, 1.0, 1.0, 1.4], |ui, index, rect| {
      let (label, axes, primary): (String, Vec<Axis>, bool) = match index {
        0 => ("X".to_string(), vec![Axis::X], false),
        1 => ("Y".to_string(), vec![Axis::Y], false),
        2 => ("Z".to_string(), vec![Axis::Z], false),
        _ => (crate::tr!("btn-zero-xyz"), Vec::new(), true),
      };
      // Size the label to the design's ~11.5px and never wrap: the placed cell is narrow, and at egui's larger
      // default button font "Zero XYZ" wrapped onto two lines inside its slot. `Extend` keeps it one line.
      let text = RichText::new(label).size(11.5).color(palette.text);
      let mut button = egui::Button::new(text).wrap_mode(egui::TextWrapMode::Extend);
      if primary {
        button = button.fill(palette.accent);
      }
      if ui.put(rect, button).clicked() {
        sink.push(Intent::SetWorkZero { axes });
      }
    });
  });

  // WCO strip: the work-coordinate offset, cached across reports.
  if !view.last_wco.is_empty() {
    ui.add_space(6.0);
    let wco: Vec<String> = view.last_wco.iter().map(|v| format!("{v:.3}")).collect();
    egui::Frame::new().fill(palette.inset).inner_margin(egui::Margin::symmetric(10, 6)).corner_radius(2.0)
      .show(ui, |ui| {
        ui.horizontal(|ui| {
          ui.label(RichText::new("WCO").monospace().size(10.5).color(palette.text_dim));
          ui.label(RichText::new(wco.join(", ")).monospace().size(10.5).color(palette.text));
        });
      });
  }

  // Tool strip: the active tool reported by the firmware's `$G`/`[GC:]` parser state ([`ViewState::current_tool`]),
  // the single authoritative source. `T0` reads as "none" so the operator can tell an explicitly-empty spindle from
  // one that simply has a tool. Never sourced from the `<...>` status report (it carries no tool number).
  if let Some(tool) = view.current_tool {
    ui.add_space(6.0);
    egui::Frame::new().fill(palette.inset).inner_margin(egui::Margin::symmetric(10, 6)).corner_radius(2.0)
      .show(ui, |ui| {
        ui.horizontal(|ui| {
          ui.label(RichText::new(crate::tr!("lbl-tool")).monospace().size(10.5).color(palette.text_dim));
          let label = if tool == 0 { crate::tr!("dro-tool-none") } else { format!("T{tool}") };
          ui.label(RichText::new(label).monospace().size(10.5).color(palette.text));
        });
      });
  }

  // Endstop indicator row: three tight X/Y/Z chips that light red when the corresponding limit switch is
  // asserted in the latest `Pn:` field, and sit dim/inset when clear. Only the X/Y/Z limits are surfaced for
  // now, but the parsed `PinState` carries the full grblHAL signal set so probe/door/etc. can join this row
  // later without reopening the parser. Drawn unconditionally so the operator always has an endstop reference;
  // with no report (or an absent `Pn:`) all three read clear.
  ui.add_space(6.0);
  endstop_chips(ui, palette, view);
  });
}

/// Render the X/Y/Z endstop indicator chips. Each chip lights red while its limit switch is asserted in the
/// latest status report and sits dim/inset when clear. Kept deliberately tight — a small per-row item spacing
/// and chip inset so the three chips plus the `LIMITS` label never overflow the fixed 268px left column (the
/// panel-overflow lesson). The lit-vs-clear decision comes from the typed [`ViewState::pins`], decoded once when
/// each status report is ingested rather than re-parsed per frame, so this stays a dumb renderer.
fn endstop_chips(ui: &mut egui::Ui, palette: Palette, view: &ViewState) {
  let pins = view.pins;
  ui.horizontal(|ui| {
    ui.spacing_mut().item_spacing.x = 4.0;
    ui.label(RichText::new(crate::tr!("lbl-limits")).size(Metrics::HEADER_TEXT).color(palette.text_dim)
      .extra_letter_spacing(Metrics::HEADER_TEXT * Metrics::HEADER_TRACKING_EM));
    for (letter, asserted) in [("X", pins.limit_x), ("Y", pins.limit_y), ("Z", pins.limit_z)] {
      endstop_chip(ui, palette, letter, asserted);
    }
  });
}

/// Draw one endstop chip: a small rounded inset with the axis letter, filled with the alarm surface and
/// labelled in alarm text while asserted, dim/inset while clear. Sized tight (a 6/2 inner margin, no button
/// chrome) so three fit the left column alongside the `LIMITS` label.
fn endstop_chip(ui: &mut egui::Ui, palette: Palette, letter: &str, asserted: bool) {
  let (fill, text) = if asserted {
    (palette.alarm_bg, palette.alarm_text)
  } else {
    (palette.inset, palette.text_disabled)
  };
  let border = if asserted { palette.alarm_border } else { palette.divider };
  // Tight 6/2 inner margin so three chips plus the `LIMITS` label fit the fixed 268px column (overflow lesson).
  chip_frame(ui, fill, border, egui::Margin { left: 6, right: 6, top: 2, bottom: 2 }, |ui| {
    let mut label = RichText::new(letter).monospace().size(11.0).color(text);
    if asserted {
      label = label.strong();
    }
    ui.label(label)
      .on_hover_text(if asserted { crate::tr!("tip-limit-asserted") } else { crate::tr!("tip-limit-clear") });
  });
}

/// One DRO coordinate-toggle button (WPos/MPos), styled as a small joined segment: the active side carries the
/// `widget.active` fill and the accent text, the inactive side the widget rest fill and dim text (design §03).
fn pos_toggle(ui: &mut egui::Ui, state: &mut UiState, machine: bool, label: &str) {
  let palette = state.style.palette;
  let active = state.show_machine_pos == machine;
  let (fill, text) = if active { (palette.widget_active, palette.accent) } else { (palette.widget, palette.text_dim) };
  let button = egui::Button::new(RichText::new(label).size(10.0).color(text)).fill(fill).corner_radius(0.0);
  if ui.add(button).clicked() {
    state.show_machine_pos = machine;
  }
}

/// Format one axis value as a large monospace tabular fixed-point string, or a dash when not derivable. Padded
/// to 8 columns (`-999.999` through `9999.999`), which spans a PCB-mill envelope while keeping the row inside the
/// fixed 268px column — a wider field overflowed the column and pushed the panel edge out (an unpainted gap).
fn big_axis_value(palette: Palette, positions: Option<&Vec<f64>>, axis: usize) -> RichText {
  match positions.and_then(|p| p.get(axis)) {
    Some(value) => RichText::new(format!("{value:>8.3}")).monospace().size(Metrics::DRO_VALUE).color(palette.text),
    None => RichText::new(format!("{:>8}", "—")).monospace().size(Metrics::DRO_VALUE).color(palette.text_disabled),
  }
}
