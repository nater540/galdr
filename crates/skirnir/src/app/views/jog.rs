//! The jog pad: step selector, direction buttons, and jog intent emission.

use super::*;

/// The candidate jog step sizes (mm) offered as quick buttons.
const JOG_STEPS: [f64; 5] = [0.01, 0.1, 1.0, 5.0, 10.0];

/// Render the jog pad: a step selector, feed field, and directional buttons for X/Y/Z plus jog-cancel.
pub fn jog(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  let palette = state.style.palette;
  // The Jog header carries the cancel affordance on the right, matching the mock's "esc · cancel" hint.
  header_bar(
    ui, palette,
    |ui| header_title(ui, palette, &crate::tr!("hdr-jog")),
    |ui| {
      if ui.add(egui::Button::new(RichText::new(crate::tr!("jog-esc-cancel")).size(10.0).color(palette.text_disabled))
        .fill(Color32::TRANSPARENT)).on_hover_text(crate::tr!("tip-jog-cancel")).clicked()
      {
        sink.push(Intent::Realtime(RealtimeCommand::JogCancel));
      }
    },
  );
  // grblHAL only honours `$J=` in Idle and Jog; it rejects it in Alarm/Hold/Door/Run/Check/Sleep. Gate the pad
  // to exactly those states so we never offer a control that will always be rejected (see [`jog_enabled`]).
  let enabled = jog_enabled(view.badge_state());
  egui::Frame::new().inner_margin(Metrics::JOG_PAD).show(ui, |ui| {
    ui.add_enabled_ui(enabled, |ui| {
      // These are dense icon/chip controls, so the default 14px button padding (28px/button) blows the cells past
      // their 32px squares and the segmented step chips past their slots, overflowing the 232px jog body. A tight
      // 6px gutter keeps every button's content inside its allocation (within the design's 4–8px dense range).
      ui.spacing_mut().button_padding.x = 6.0;
      // The pad: a 3×3 XY arrow grid of 32px cells (design §03) with Z± and rotary A± columns alongside,
      // mirroring the physical axes — Y+ top, X∓ flanking the centre, Y− bottom; Z+/Z/Z− then A+/A/A− stacked to
      // the right (the A column is the DOC-10 rotary, jogging in degrees under the degrees-as-mm convention).
      // Derive the columns' width from the row's own width here, not `available_width()` mid-row: after the
      // grid, the running layout reports a stale (too-large) remaining width, which sized the fill column wide
      // enough to overflow the 268px column and leave an unpainted gap beside the panel. The grid spans three
      // 32px cells with two inter-cell gaps; the Z and A columns then split what remains after the grid, the
      // 16px inter-column gap (a `JOG_GAP` item space, the `JOG_GAP*2` separator, and a second `JOG_GAP` item
      // space), and the `JOG_GAP` between the two columns.
      // The rotary A column is shown ONLY when the firmware reports a 4-field position (the same gate the DRO's A
      // row uses). On a 3-axis board an A jog would emit `$J=...A...`, which the firmware rejects with `error:N` and
      // then holds the stream in the error state — so we must not even offer it. With no A column the Z column takes
      // the whole fill so no empty gap is left where A would sit.
      let has_rotary = view.has_rotary_axis();
      let xy_width = 3.0 * Metrics::JOG_CELL + 2.0 * Metrics::JOG_GAP;
      let fill = (ui.available_width() - xy_width - Metrics::JOG_GAP * 5.0).max(2.0 * Metrics::JOG_CELL);
      let zw = if has_rotary { (fill / 2.0).floor() } else { fill };
      ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing = Vec2::splat(Metrics::JOG_GAP);
        let gap = Vec2::splat(Metrics::JOG_GAP);
        egui::Grid::new("jog-xy").spacing(gap).min_col_width(Metrics::JOG_CELL).show(ui, |ui| {
          jog_blank(ui);
          jog_button(ui, "↑", state, sink, Some((Axis::Y, Dir::Pos)));
          jog_blank(ui);
          ui.end_row();
          jog_button(ui, "←", state, sink, Some((Axis::X, Dir::Neg)));
          jog_button(ui, "XY", state, sink, None);
          jog_button(ui, "→", state, sink, Some((Axis::X, Dir::Pos)));
          ui.end_row();
          jog_blank(ui);
          jog_button(ui, "↓", state, sink, Some((Axis::Y, Dir::Neg)));
          jog_blank(ui);
          ui.end_row();
        });

        ui.add_space(Metrics::JOG_GAP * 2.0);
        ui.vertical(|ui| {
          ui.spacing_mut().item_spacing.y = Metrics::JOG_GAP;
          jog_z(ui, "Z+", zw, state, sink, Some((Axis::Z, Dir::Pos)));
          jog_z(ui, "Z", zw, state, sink, None);
          jog_z(ui, "Z−", zw, state, sink, Some((Axis::Z, Dir::Neg)));
        });
        if has_rotary {
          ui.vertical(|ui| {
            ui.spacing_mut().item_spacing.y = Metrics::JOG_GAP;
            jog_z(ui, "A+", zw, state, sink, Some((Axis::A, Dir::Pos)));
            jog_z(ui, "A", zw, state, sink, None);
            jog_z(ui, "A−", zw, state, sink, Some((Axis::A, Dir::Neg)));
          });
        }
      });

      // Step selector (design §03: segmented quick steps) and the jog feed rate. The step drives X/Y/Z in mm and
      // the rotary A in degrees (DOC-10's degrees-as-mm convention), so the label names both units.
      ui.add_space(8.0);
      ui.label(RichText::new(crate::tr!("jog-step-label")).size(10.5).color(palette.text_dim));
      ui.add_space(2.0);
      step_selector(ui, state);
      ui.add_space(6.0);
      ui.horizontal(|ui| {
        dim_label(ui, palette, crate::tr!("lbl-feed"));
        ui.add(egui::DragValue::new(&mut state.jog_feed).speed(10.0).range(1.0..=10_000.0).suffix(" mm/min"));
      });
    });
  });
}

/// Render the jog step selector as a single joined segmented control (design §03), not a row of separate
/// bordered buttons (the "identical buttons" look the user flagged). The chips sit flush inside a recessed
/// inset track — the track shows through the 1px gaps as hairline separators — with the per-chip button border
/// and rounding stripped so the row reads as one control. One chip per numeric [`JOG_STEPS`] value plus a
/// trailing wider `cont` chip; exactly one is active (accent fill + accent text), the rest dim, so the selected
/// step is unmistakable. Picking a number turns continuous mode off; picking `cont` flips into press-and-hold.
fn step_selector(ui: &mut egui::Ui, state: &mut UiState) {
  let palette = state.style.palette;
  egui::Frame::new()
    .fill(palette.inset)
    .stroke(egui::Stroke::new(1.0, palette.border_recess))
    .corner_radius(Metrics::CONTROL_RADIUS)
    .inner_margin(1)
    .show(ui, |ui| {
      // Strip the per-button border and rounding inside the track so the chips abut as one segmented control
      // rather than reading as individual buttons; the 1px inter-chip gap then shows the inset as a separator.
      {
        let widgets = &mut ui.visuals_mut().widgets;
        for visual in [&mut widgets.inactive, &mut widgets.hovered, &mut widgets.active, &mut widgets.noninteractive]
        {
          visual.bg_stroke = egui::Stroke::NONE;
          visual.corner_radius = egui::CornerRadius::ZERO;
        }
      }
      // The cells are narrow; egui's default 14px horizontal button padding would leave no room for the label and
      // wrap "0.01" onto two lines. Trim to a hair of padding so each chip's text fits its slot on one line.
      ui.spacing_mut().button_padding = Vec2::new(2.0, 0.0);
      // One slot per numeric step plus a slightly wider (1.2×) `cont` slot, matching the design's flex ratios.
      let cont_index = JOG_STEPS.len();
      let weights: Vec<f32> = (0..=cont_index).map(|i| if i == cont_index { 1.2 } else { 1.0 }).collect();
      button_row(ui, 1.0, &weights, |ui, index, rect| {
        let is_cont = index == cont_index;
        // A numeric chip is active only when it is the chosen step AND continuous mode is off; the `cont` chip is
        // active only in continuous mode — so the selector always shows exactly one active mode.
        let active = if is_cont {
          state.jog_continuous
        } else {
          !state.jog_continuous && (state.jog_step - JOG_STEPS[index]).abs() < f64::EPSILON
        };
        let (fill, text) = if active { (palette.widget_active, palette.accent) } else { (palette.panel, palette.text_dim) };
        let label = if is_cont { crate::tr!("jog-cont") } else { format!("{}", JOG_STEPS[index]) };
        let button = egui::Button::new(RichText::new(label).monospace().size(11.0).color(text))
          .fill(fill)
          .corner_radius(0.0)
          .wrap_mode(egui::TextWrapMode::Extend);
        let response = ui.put(rect, button);
        let response = if is_cont {
          response.on_hover_text(crate::tr!("tip-jog-cont"))
        } else {
          response
        };
        if response.clicked() {
          if is_cont {
            state.jog_continuous = true;
          } else {
            state.jog_step = JOG_STEPS[index];
            state.jog_continuous = false;
          }
        }
      });
    });
}

/// Whether the jog pad should be enabled for the given machine state. grblHAL only accepts a `$J=` jog while
/// it is in `Idle` or `Jog` (a running jog can be re-targeted); every other state — `Run`, `Hold`, `Door`,
/// `Alarm`, `Check`, `Sleep`, and the disconnected/connecting host phases — rejects it, so we disable rather
/// than offer a control that is guaranteed to bounce. Pure so it is unit-tested without a window.
pub(crate) fn jog_enabled(state: BadgeState) -> bool {
  matches!(state, BadgeState::Idle | BadgeState::Jog)
}

/// An empty cell in the jog grid, sized to a jog cell so the arrows align on a true 3×3.
fn jog_blank(ui: &mut egui::Ui) {
  ui.allocate_exact_size(Vec2::splat(Metrics::JOG_CELL), egui::Sense::hover());
}

/// One XY jog cell: a fixed 32px square. With an axis/dir it issues a jog; the centre `XY` cell is an inert
/// label (design §03). In fixed-step mode a click issues a single [`Intent::Jog`]; in continuous mode holding
/// the cell issues [`Intent::JogStart`] on press and [`Intent::JogStop`] on release (see [`emit_jog`]).
fn jog_button(ui: &mut egui::Ui, label: &str, state: &UiState, sink: &mut IntentSink, motion: Option<(Axis, Dir)>) {
  let palette = state.style.palette;
  let Some((axis, dir)) = motion else {
    // The centre cell is a non-interactive label marking the pad's purpose.
    let button = egui::Button::new(RichText::new(label).monospace().size(9.5).color(palette.text_disabled));
    ui.add_sized(Vec2::splat(Metrics::JOG_CELL), button);
    return;
  };
  let response = ui.add_sized(Vec2::splat(Metrics::JOG_CELL), egui::Button::new(label).sense(jog_sense(state)));
  emit_jog(&response, state, sink, axis, dir);
}

/// One Z-column jog button: a full-width cell, 32px tall, matching the XY pad's height. The middle `Z` cell is
/// an inert label between Z+ and Z−. Step vs continuous behaviour matches [`jog_button`].
fn jog_z(ui: &mut egui::Ui, label: &str, width: f32, state: &UiState, sink: &mut IntentSink,
  motion: Option<(Axis, Dir)>) {
  let palette = state.style.palette;
  let size = Vec2::new(width, Metrics::JOG_CELL);
  let Some((axis, dir)) = motion else {
    ui.add_sized(size, egui::Button::new(RichText::new(label).color(palette.text_disabled)));
    return;
  };
  let response = ui.add_sized(size, egui::Button::new(label).sense(jog_sense(state)));
  emit_jog(&response, state, sink, axis, dir);
}

/// The egui sense a jog cell needs for the current mode: in continuous mode it must sense drag (press-and-hold)
/// so the press and release edges are observable, plus click so a brief tap still issues a bounded step jog; in
/// fixed-step mode a plain click suffices.
pub(crate) fn jog_sense(state: &UiState) -> egui::Sense {
  if state.jog_continuous {
    egui::Sense::click_and_drag()
  } else {
    egui::Sense::click()
  }
}

/// Translate a jog cell's [`egui::Response`] into the right jog intent(s) for the current mode. Fixed-step:
/// a click is one bounded [`Intent::Jog`]. Continuous: the press edge (`drag_started`) starts the long move and
/// the release edge (`drag_stopped`) cancels it, so the axis moves exactly while the control is held.
fn emit_jog(response: &egui::Response, state: &UiState, sink: &mut IntentSink, axis: Axis, dir: Dir) {
  if !state.jog_continuous {
    if response.clicked() {
      sink.push(Intent::Jog { axis, dir, distance: state.jog_step, feed: state.jog_feed });
    }
    return;
  }
  if response.drag_started() {
    sink.push(Intent::JogStart { axis, dir, feed: state.jog_feed });
  }
  if response.drag_stopped() {
    sink.push(Intent::JogStop);
  }
  // A tap too brief to register as a drag is reported as a click with no drag edges. It must NOT be bracketed as
  // a continuous start+stop: the queued `$J=` move and the out-of-band jog-cancel (`0x85`) race at the firmware,
  // and grblHAL drops a cancel that lands before the jog has actually begun — leaving the long move running away
  // (then a panic Stop soft-resets mid-motion into ALARM:3). Issue one bounded step jog instead, which completes
  // on its own and needs no cancel, so a quick nudge still moves by the last selected step.
  if response.clicked() && !response.drag_started() {
    sink.push(Intent::Jog { axis, dir, distance: state.jog_step, feed: state.jog_feed });
  }
}
