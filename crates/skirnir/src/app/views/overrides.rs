//! The feed/rapid/spindle override sliders and their axis rows.

use super::*;

/// Render the override controls: feed and spindle get a slider plus a fine/coarse/reset stepper row (design
/// §03's override sliders); rapid stays a 100/50/25 preset picker (grbl exposes no rapid ±). The sliders
/// express an absolute target; the shell turns that into the minimal relative ±10/±1/reset byte sequence
/// against the live `Ov:` value, so the view never does the override byte arithmetic.
pub fn overrides(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  let palette = state.style.palette;
  use crate::app::overrides::OverrideAxis;
  section_header(ui, palette, &crate::tr!("hdr-overrides"));
  // The cached override survives reports that omit the intermittent `Ov:` field, so the handle/steppers do not
  // snap to 100% on every Ov-less poll. See [`ViewState::overrides`].
  let (feed, rapid, spindle) = view.overrides();
  // Overrides only do anything on a ready link — a relative ±10/±1/reset byte is a no-op with nothing connected
  // to act on it — so the whole panel is disabled until the board is connected and ready (Idle/Run/Hold/…).
  let enabled = view.connection.is_connected();

  egui::Frame::new().inner_margin(Metrics::RIGHT_PAD).show(ui, |ui| {
    override_axis(ui, palette, &crate::tr!("lbl-feed"), OverrideAxis::Feed, feed, enabled,
      &mut state.feed_override_drag, sink,
      RealtimeCommand::FeedOverrideMinus1, RealtimeCommand::FeedOverrideMinus10, RealtimeCommand::FeedOverrideReset,
      RealtimeCommand::FeedOverridePlus10, RealtimeCommand::FeedOverridePlus1);
    ui.add_space(4.0);
    override_axis(ui, palette, &crate::tr!("lbl-spindle"), OverrideAxis::Spindle, spindle, enabled,
      &mut state.spindle_override_drag, sink,
      RealtimeCommand::SpindleOverrideMinus1, RealtimeCommand::SpindleOverrideMinus10,
      RealtimeCommand::SpindleOverrideReset, RealtimeCommand::SpindleOverridePlus10,
      RealtimeCommand::SpindleOverridePlus1);
    ui.add_space(4.0);

    // Rapid override is preset-only in grbl (100/50/25), so it gets buttons rather than a slider. The three
    // buttons are RIGHT-ALIGNED and the label TRUNCATES into whatever remains: laid left-to-right, the Swedish
    // "Snabbmatning 100%" pushed the row ~18px past the 286px column — and under egui 0.35 a side panel whose
    // content overflows its width shifts the whole central-region cursor and paints the viewport OVER the
    // column's edge (the measured-rect clamp re-anchors on the overflowed edge). A width-proof row cannot
    // regress that way in any locale; the column-overflow regression test guards the class.
    ui.horizontal(|ui| {
      ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
        if ui.button("25").clicked() {
          sink.push(Intent::Realtime(RealtimeCommand::RapidOverride25));
        }
        if ui.button("50").clicked() {
          sink.push(Intent::Realtime(RealtimeCommand::RapidOverride50));
        }
        if ui.button("100").clicked() {
          sink.push(Intent::Realtime(RealtimeCommand::RapidOverrideReset));
        }
        ui.add(egui::Label::new(crate::tr!("ov-rapid", { pct: format!("{rapid:>3}") })).truncate());
      });
    });

    // Realized feed/speed from the latest report: what the machine is actually doing after overrides.
    if let Some((feed, rpm, actual)) = view.status.as_ref().and_then(|s| s.feed_speed) {
      ui.add_space(6.0);
      egui::Frame::new().fill(palette.inset).inner_margin(egui::Margin::symmetric(12, 8)).corner_radius(2.0)
        .show(ui, |ui| {
          ui.horizontal(|ui| {
            ui.label(RichText::new(crate::tr!("ov-realized-feed")).size(10.5).color(palette.text_dim));
            ui.label(RichText::new(format!("{feed:.0} mm/min")).monospace().color(palette.text));
          });
          ui.horizontal(|ui| {
            ui.label(RichText::new(crate::tr!("ov-realized-speed")).size(10.5).color(palette.text_dim));
            let shown = actual.unwrap_or(rpm);
            ui.label(RichText::new(format!("{shown:.0} RPM")).monospace().color(palette.text));
          });
        });
    }
  });
}

/// A custom override slider matching the design's filled-bar look (design §03/§04): a 6px recessed inset track
/// with an accent-coloured fill that grows from the left in proportion to the value across the 10–200% span,
/// plus a thin handle at the fill edge for a grab affordance. egui's stock `Slider` rendered only a grey rail
/// and a square knob — no colour at all (the user-flagged "missing colors entirely"). The strip is taller than
/// the 6px track so it is easy to grab; dragging or clicking maps the pointer x onto the value and reports the
/// change through the returned [`egui::Response`] so the caller's commit/mirror logic is unchanged.
fn override_slider(ui: &mut egui::Ui, palette: Palette, value: &mut u32, fill_color: Color32) -> egui::Response {
  use crate::app::overrides::{OVERRIDE_MAX, OVERRIDE_MIN, OVERRIDE_NEUTRAL};
  let width = ui.available_width().max(48.0);
  let (rect, mut response) = ui.allocate_exact_size(Vec2::new(width, 18.0), egui::Sense::click_and_drag());
  let track = egui::Rect::from_center_size(rect.center(), Vec2::new(width, Metrics::SLIDER_H));
  let span = (OVERRIDE_MAX - OVERRIDE_MIN) as f32;

  // Pointer drives the value: map its x across the track onto the 10–200% span while pressed/dragged.
  if (response.dragged() || response.clicked())
    && let Some(pos) = response.interact_pointer_pos()
  {
    let frac = ((pos.x - track.left()) / track.width()).clamp(0.0, 1.0);
    let next = OVERRIDE_MIN + (frac * span).round() as u32;
    if next != *value {
      *value = next;
      response.mark_changed();
    }
  }

  // The coloured fill reads against the *nominal* 100%, so a neutral 100% override shows a full bar (design
  // §03) and reducing the override shrinks it; at or above 100% the bar saturates full. The handle, by
  // contrast, sits at the override's true position across the full 10–200% drag span, so it still tracks the
  // pointer all the way to 200% and the 100–200% range stays adjustable — the fill is the at-a-glance gauge,
  // the handle is the precise position.
  let fill_frac = (*value as f32 / OVERRIDE_NEUTRAL as f32).clamp(0.0, 1.0);
  let pos_frac = (value.saturating_sub(OVERRIDE_MIN)) as f32 / span;
  let radius = egui::CornerRadius::same(Metrics::CONTROL_RADIUS);
  let painter = ui.painter();
  painter.rect_filled(track, radius, palette.inset);
  let mut fill = track;
  fill.set_width(track.width() * fill_frac);
  painter.rect_filled(fill, radius, fill_color);
  painter.rect_stroke(track, radius, egui::Stroke::new(1.0, palette.border_recess), egui::StrokeKind::Inside);
  // A 2px handle at the override's true position, brightened to the text colour, so the operator sees the grab
  // point and can read where in the 10–200% span the value sits even while the fill is saturated full.
  let handle_x = (track.left() + track.width() * pos_frac.clamp(0.0, 1.0)).clamp(track.left(), track.right());
  let handle = egui::Rect::from_center_size(egui::pos2(handle_x, track.center().y), Vec2::new(2.0, 14.0));
  painter.rect_filled(handle, egui::CornerRadius::ZERO, palette.text);
  response
}

/// Render one override axis (feed or spindle): a label with the live percentage, a 10–200% slider that emits
/// an absolute [`Intent::SetOverride`] on release, and a fine/coarse/reset stepper row (`−10 −1 100 +1 +10`)
/// that emits single relative real-time bytes. The slider and the steppers are two equivalent ways to reach
/// the same override; the slider is coarse-grained reach, the steppers are precise nudges including the new
/// fine ±1%.
///
/// `live` is the override the firmware last reported. `feedback` is the slider's transient feedback state: it
/// mirrors `live` while idle (so the firmware's truth re-centers the handle), holds the operator's position
/// during a drag (so a status poll cannot yank it), and — the snap-back fix — holds the committed target after
/// release until the firmware's relative ramp converges onto it (see [`crate::app::overrides::OverrideFeedback`]). On
/// release we emit the target only if it moved off `live`, so merely touching the slider sends nothing. The
/// whole row is disabled unless a live link can act on the override (`enabled`), since overrides are no-ops
/// otherwise.
#[allow(clippy::too_many_arguments)]
fn override_axis(ui: &mut egui::Ui, palette: Palette, label: &str, axis: crate::app::overrides::OverrideAxis, live: u32,
  enabled: bool, feedback: &mut crate::app::overrides::OverrideFeedback, sink: &mut IntentSink, minus1: RealtimeCommand,
  minus10: RealtimeCommand, reset: RealtimeCommand, plus10: RealtimeCommand, plus1: RealtimeCommand) {
  use crate::app::overrides::OverrideAxis;

  // Fold the latest live report into the feedback state first: while idle the handle follows `live`; a
  // post-release hold releases once `live` converges onto the committed target. A drag ignores reports entirely.
  feedback.observe(live);
  // The value the handle shows this frame: `live` when idle, the pinned drag/hold value otherwise.
  let mut value = feedback.display(live);
  // Feed (and rapid) carry the cool control-blue fill; spindle carries the warm motion-orange, matching the
  // design's `#0E86D4` feed bar and `#FF7A1A` spindle bar (the colour that was missing entirely before).
  let fill_color = match axis {
    OverrideAxis::Feed => palette.accent,
    OverrideAxis::Spindle => palette.accent_motion,
  };
  ui.add_enabled_ui(enabled, |ui| {
    // Row: dim label on the left, the filled track stretching across the middle, the live percent on the right.
    ui.horizontal(|ui| {
      ui.label(RichText::new(label).size(11.0).color(palette.text_dim));
      ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
        ui.label(RichText::new(format!("{value:>3}%")).monospace().size(11.5).color(palette.text));
        let slider = override_slider(ui, palette, &mut value, fill_color);
        // A UI test (egui_kittest) needs the slider's on-screen rect to drive a pointer drag at known
        // coordinates, since the widget is hand-painted and label-less so AccessKit cannot locate it by text.
        #[cfg(all(test, feature = "gui"))]
        slider_rect_probe::record(axis, slider.rect);
        if slider.dragged() {
          // While dragging, hold the operator's position so the live status poll cannot yank the handle back.
          feedback.drag_to(value);
        }
        if slider.drag_stopped() || slider.clicked() {
          // On release/click, commit the target if it moved off the live value and HOLD it; the hold survives the
          // firmware's relative ramp so the handle stays put instead of snapping to the stale `live` and crawling.
          if value != live {
            sink.push(Intent::SetOverride { axis, target: value });
            feedback.commit(value, live);
          } else {
            // An empty touch (no movement): nothing committed, so just return to mirroring live.
            *feedback = crate::app::overrides::OverrideFeedback::Idle;
          }
        }
      });
    });
  });

  // The stepper row: fine ±1% (the new control) flanks coarse ±10% around a reset-to-100%. Gated on the same
  // live link as the slider — a relative override byte is a no-op with nothing connected to act on it. Placed
  // via [`button_row`] at exact, width-splitting rects: five free-flowing buttons at the default padding are
  // intrinsically ~283px wide — wider than the column's inner width — which was invisible under egui 0.34
  // (silently clipped) but shifts the whole central region under 0.35 (see the rapid-row comment above). The
  // split row always fits by construction.
  ui.add_enabled_ui(enabled, |ui| {
    let commands = [minus10, minus1, reset, plus1, plus10];
    let labels = ["−10", "−1", "100", "+1", "+10"];
    button_row(ui, 6.0, &[1.0; 5], |ui, index, rect| {
      let text = RichText::new(labels[index]).size(11.5);
      let button = egui::Button::new(text).wrap_mode(egui::TextWrapMode::Extend);
      if ui.put(rect, button).clicked() {
        sink.push(Intent::Realtime(commands[index]));
      }
    });
  });
}

/// A test-only side channel that records each override slider's on-screen [`egui::Rect`] as it is rendered, so
/// the egui_kittest harness can compute pointer coordinates inside a slider it cannot otherwise locate (the
/// widget is hand-painted with no label, so AccessKit exposes no findable node). Compiled only for the
/// gui-featured test build; the production render path is untouched apart from the cheap `record` call.
#[cfg(all(test, feature = "gui"))]
pub(crate) mod slider_rect_probe {
  use crate::app::overrides::OverrideAxis;
  use eframe::egui::Rect;
  use std::cell::Cell;

  thread_local! {
    /// The last-rendered rect of the feed and spindle sliders, in screen coordinates. `None` until first drawn.
    static FEED: Cell<Option<Rect>> = const { Cell::new(None) };
    static SPINDLE: Cell<Option<Rect>> = const { Cell::new(None) };
  }

  /// Record the slider rect for `axis` from the just-completed render of that axis's row.
  pub(crate) fn record(axis: OverrideAxis, rect: Rect) {
    match axis {
      OverrideAxis::Feed => FEED.with(|c| c.set(Some(rect))),
      OverrideAxis::Spindle => SPINDLE.with(|c| c.set(Some(rect))),
    }
  }

  /// The last-recorded rect for `axis`, or `None` if that axis has not been rendered yet this thread.
  pub(crate) fn last(axis: OverrideAxis) -> Option<Rect> {
    match axis {
      OverrideAxis::Feed => FEED.with(Cell::get),
      OverrideAxis::Spindle => SPINDLE.with(Cell::get),
    }
  }
}
