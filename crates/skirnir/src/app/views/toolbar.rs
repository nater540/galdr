//! The top toolbar: state badge, transport group, dock toggles, and its width-fit bookkeeping.

use super::*;

/// Map a clicked tab index from the dock's `[Console, Program]` strip onto a [`DockTab`], leaving the selection
/// unchanged when nothing was clicked. Pure so the tab-switch decision is unit-tested without a window.
pub(crate) fn dock_tab_for_click(current: DockTab, clicked: Option<usize>) -> DockTab {
  match clicked {
    Some(0) => DockTab::Console,
    Some(1) => DockTab::Program,
    _ => current,
  }
}

/// The glyph for the dock's collapse/expand toggle: a minus when expanded (click to minimise) and a plus when
/// collapsed (click to restore). Pure so the icon choice is unit-tested without a window. Uses the typographic
/// minus (U+2212) so it reads as a control glyph at the toggle's small size rather than a hyphen.
pub(crate) fn dock_toggle_label(collapsed: bool) -> &'static str {
  if collapsed { "+" } else { "−" }
}

/// Draw the machine-state badge: a coloured dot, the uppercase label, and (when streaming) the feed/speed
/// suffix. The dot colour is the *second* signal; the label is primary, per the design.
fn state_badge(ui: &mut egui::Ui, view: &ViewState, state_ui: &mut UiState) {
  let palette = state_ui.style.palette;
  let state = view.badge_state();
  let color = palette.badge_color(state);
  let (fill, border, text_color) = match state {
    BadgeState::Alarm | BadgeState::Error => (palette.alarm_bg, palette.alarm_border, palette.alarm_text),
    _ => (palette.inset, color.gamma_multiply(0.5), palette.text),
  };
  let margin = egui::Margin { left: Metrics::BADGE_PAD.x as i8, right: Metrics::BADGE_PAD.x as i8,
    top: Metrics::BADGE_PAD.y as i8, bottom: Metrics::BADGE_PAD.y as i8 };
  // The chip's parts in VISUAL left-to-right order: the dot leads, the label carries, and (while moving) the
  // realized feed/speed trails. egui's `horizontal` PRESERVES the embedding direction — the toolbar right-aligns
  // the badge through a right-to-left closure, where the first-added item lands rightmost — so the parts are fed
  // in direction-matched order below. (Forcing a left-to-right child instead either claims the whole remaining
  // bar width (`with_layout`) or grows past the window edge (a zero-sized `allocate_ui_with_layout`), so
  // direction-aware ordering inside the inherited layout is the shape that stays content-sized.)
  let feed_speed = matches!(state, BadgeState::Run | BadgeState::Jog)
    .then(|| view.status.as_ref().and_then(|s| s.feed_speed))
    .flatten();
  let resp = chip_frame(ui, fill, border, margin, |ui| {
    ui.horizontal(|ui| {
      let rtl = ui.layout().main_dir() == egui::Direction::RightToLeft;
      let draw_dot = |ui: &mut egui::Ui| {
        // The 2px spacer must land BETWEEN the dot and the label. The cursor advances leftward under RTL and
        // the dot is drawn AFTER the label there, so the spacer goes before the dot (i.e. on its label side);
        // in LTR the dot leads and the spacer follows it. A trailing spacer under RTL ended up on the chip's
        // far LEFT edge — outside the dot — instead of separating it from the label.
        if rtl {
          ui.add_space(2.0);
          dot(ui, color, Metrics::BADGE_DOT);
        } else {
          dot(ui, color, Metrics::BADGE_DOT);
          ui.add_space(2.0);
        }
      };
      let draw_label = |ui: &mut egui::Ui| {
        ui.label(RichText::new(crate::tr!(state.label_key())).color(text_color).strong());
      };
      let draw_suffix = |ui: &mut egui::Ui| {
        if let Some((feed, rpm, _)) = feed_speed {
          ui.label(RichText::new(format!("F {feed:.0} · S {rpm:.0}")).monospace().size(10.5)
            .color(palette.text_dim));
        }
      };
      if rtl {
        draw_suffix(ui);
        draw_label(ui);
        draw_dot(ui);
      } else {
        draw_dot(ui);
        draw_label(ui);
        draw_suffix(ui);
      }
    });
  });
  // The badge doubles as the hidden Pride "fabulous mode" toggle: interact over the chip's rect (the frame
  // itself only hovers) so six clicks flip the rainbow accent. Show the click cursor so the spot is at least
  // feelable, but leave it unlabelled — discovering it is the point.
  let egg = ui.interact(resp.rect, resp.id.with("fabulous_egg"), egui::Sense::click());
  if egg.clicked() {
    state_ui.register_fabulous_click();
  }
}

/// The toolbar's self-measured fit, persisted in egui temp memory across frames: whether the bar renders in
/// COMPACT form (secondary controls collapse to icon glyphs) and the width the FULL form was last measured to
/// need. Immediate mode cannot know before layout whether the full labels fit — label widths depend on the
/// locale (Swedish "Inställningar"/"Nödstopp" overflow widths English clears) — so the bar renders, measures its
/// real extent, and stores the verdict for the NEXT frame: the standard immediate-mode responsive pattern.
///
/// The full requirement can only be re-MEASURED while rendering full, so the stored value carries a
/// [`Self::fingerprint`] of the content that produced it (locale, transport attachment, badge label — the inputs
/// that change the labels' widths). When the fingerprint no longer matches, the measurement is stale and is
/// DISCARDED, forcing one full-form measuring frame. Without this, a requirement that SHRANK while compact —
/// connecting (the wide port group becomes one Disconnect button), switching to a shorter locale — was never
/// re-measured and the bar stayed icon-only forever at that width (the stuck-compact bug). Either direction now
/// costs at most one corrective flip after a content change.
#[derive(Clone, Copy, Default)]
struct ToolbarFit {
  /// Whether the bar currently renders icon-form secondary controls.
  compact: bool,
  /// The total width (px) the FULL-labelled form last measured itself to need, including both edge paddings.
  full_needs: f32,
  /// Hash of the label-width-driving content [`Self::full_needs`] was measured under; a mismatch invalidates it.
  fingerprint: u64,
}

/// Hash the inputs that determine the toolbar's full-form label widths: the active locale (every label), whether
/// a transport is attached (the port combo + refresh/identify/connect group swaps for one Disconnect button),
/// and the badge label (INAKTIV vs VERKTYGSBYTE differ by ~80px in Swedish). The port PATH is excluded — the
/// combo is width-capped and truncating, so a different path never changes the bar's requirement.
fn toolbar_content_fingerprint(view: &ViewState) -> u64 {
  use std::hash::{Hash, Hasher};
  let mut hasher = std::hash::DefaultHasher::new();
  crate::i18n::get_language().hash(&mut hasher);
  view.connection.has_transport().hash(&mut hasher);
  view.badge_state().label_key().hash(&mut hasher);
  hasher.finish()
}

/// Render the 40px main toolbar: the connect group, Open, the Run/Hold/Stop segmented transport group, Home,
/// Settings, and the right-aligned machine-state badge (design §03). Self-measuring: when the full labels
/// cannot fit the bar's width, the SECONDARY controls (identify, open, transport, simulate, home, settings)
/// collapse to icon glyphs with their full labels on hover — the primary actions (connect/disconnect) and the
/// safety-critical state badge always keep their full form. See [`ToolbarFit`].
pub fn toolbar(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  let palette = state.style.palette;
  let fit_id = egui::Id::new("toolbar-fit");
  let mut fit: ToolbarFit = ui.ctx().data(|d| d.get_temp(fit_id)).unwrap_or_default();
  let fingerprint = toolbar_content_fingerprint(view);
  if fit.fingerprint != fingerprint {
    // The labels that size the bar changed (locale, connect state, badge): the stored full-form measurement no
    // longer describes them. Discard it and render FULL this frame to re-measure, so a shrunk requirement can
    // recover to full labels instead of sticking icon-only (and a grown one flips compact next frame).
    fit = ToolbarFit { compact: false, full_needs: 0.0, fingerprint };
  }
  let compact = fit.compact;
  // 1px bottom divider under the bar (design §03's `border-bottom:1px #2E2E2E`), painted along the panel edge.
  let bar = ui.max_rect();
  ui.painter().hline(bar.x_range(), bar.bottom() - 0.5, egui::Stroke::new(1.0, palette.divider));
  // `horizontal_centered` vertically centres the 26px controls in the 40px bar, giving the design's even
  // breathing room above and below rather than the top-aligned look the plain `horizontal` produced. The closure
  // reports the LTR content's right edge and the right-aligned cluster's left edge, so the bar can measure
  // whether its content actually fit this frame.
  let (ltr_end, rtl_left) = ui.horizontal_centered(|ui| {
    // Toolbar controls are 26px tall with 10px side-padding and a 6px gap (design §03); set the region spacing
    // up front so every button/combo in the strip inherits the bar's sizing rather than the panel default.
    ui.spacing_mut().item_spacing.x = Metrics::TOOLBAR_GAP;
    ui.spacing_mut().button_padding = Metrics::TOOLBAR_BUTTON_PAD;
    ui.spacing_mut().interact_size.y = Metrics::TOOLBAR_CONTROL_H;
    // The bar's own horizontal padding (`padding:0 10px`, design §03): the panel frame is margin-free (the
    // divider must span edge to edge), so the strip insets its first item here and its last in the right-to-left
    // closure below. `add_space` moves the cursor directly — no item gap rides along with it.
    ui.add_space(Metrics::TOOLBAR_PAD_X);
    // Gate the teardown affordance on whether a transport is ATTACHED, not on whether the board is fully ready.
    // A connect that stalls in `Connecting` (the ESP32-S3 can fail to volunteer readiness) still holds the OS port
    // open; without a Disconnect here the operator could not release the FD short of killing the process, blocking
    // espflash / another sender. While still `Connecting` the button reads "Cancel" (cancelling the connect); once
    // ready it reads "Disconnect". Both push the same intent — the shell sends `Command::Disconnect`, which ends the
    // engine task and drops the serial stream regardless of which lifecycle state it was in.
    let attached = view.connection.has_transport();
    let connected = view.connection.is_connected();

    if attached {
      let label = if connected { crate::tr!("btn-disconnect") } else { crate::tr!("btn-cancel") };
      if ui.button(label).clicked() {
        sink.push(Intent::Disconnect);
      }
    } else {
      // Port dropdown + refresh + identify + connect, only meaningful while disconnected. Each row shows the
      // (cu-preferred) path and, when known, a short USB product / «likely Galdr» hint so the board stands out.
      // The combo lives in a FIXED-WIDTH child ui: `ComboBox::width` is only a MINIMUM and `truncate` bounds to
      // `ui.available_width()` — in an open toolbar row that is the whole bar, so a long path would grow the
      // button past any requested width. Capping the child's width makes the truncation real, which is what
      // keeps the connect group stable as ports come and go AND lets compact actually shrink it (the dropdown
      // rows still show every path whole).
      let combo_w = if compact { 120.0 } else { 200.0 };
      ui.allocate_ui_with_layout(
        Vec2::new(combo_w, Metrics::TOOLBAR_CONTROL_H),
        Layout::left_to_right(Align::Center),
        |ui| {
          ui.set_max_width(combo_w);
          egui::ComboBox::from_id_salt("port")
            .width(combo_w)
            .truncate()
            .selected_text(if state.selected_port.is_empty() {
              crate::tr!("port-choose")
            } else {
              state.selected_port.clone()
            })
            .show_ui(ui, |ui| {
              for port in &state.ports {
                // The selectable label carries the path; the hint (if any) trails it dimmed so the row stays
                // scannable while flagging the likely board.
                let label = match port.hint() {
                  Some(hint) => format!("{}  ·  {hint}", port.path),
                  None => port.path.clone(),
                };
                ui.selectable_value(&mut state.selected_port, port.path.clone(), label);
              }
            });
        },
      );
      // Compact thins the two icon buttons' side padding too — every pixel of the connect group counts at the
      // minimum width. Scoped: the padding is restored before the (text) Connect button below.
      let full_pad = ui.spacing().button_padding;
      if compact {
        ui.spacing_mut().button_padding.x = 5.0;
      }
      if ui.button("⟳").on_hover_text(crate::tr!("tip-refresh-ports")).clicked() {
        sink.push(Intent::RefreshPorts);
      }
      let has_port = !state.selected_port.is_empty();
      // Identify actively probes the selected port for grblHAL. It is opt-in (opening toggles the board's
      // auto-reset line) and never part of a refresh, so it sits behind its own button. Compact: a magnifier
      // glyph (probe/inspect), with the full label folded into the hover.
      let identify_label = if compact { "🔍".to_string() } else { crate::tr!("btn-identify") };
      if ui
        .add_enabled(has_port, egui::Button::new(identify_label))
        .on_hover_text(compact_tip(compact, crate::tr!("btn-identify"), crate::tr!("tip-identify")))
        .clicked()
      {
        sink.push(Intent::IdentifyPort { path: state.selected_port.clone() });
      }
      ui.spacing_mut().button_padding = full_pad;
      if ui.add_enabled(has_port, egui::Button::new(crate::tr!("btn-connect"))).clicked() {
        sink.push(Intent::Connect { path: state.selected_port.clone(), baud: state.baud });
      }
    }

    toolbar_divider(ui, palette, compact);

    let open_label = if compact { "🗁".to_string() } else { crate::tr!("btn-open") };
    if ui
      .button(open_label)
      .on_hover_text(compact_tip(compact, crate::tr!("btn-open"), crate::tr!("tip-open")))
      .clicked()
      && let Some(path) = rfd::FileDialog::new().add_filter("G-code", &["gcode", "nc", "ngc", "tap"]).pick_file()
    {
      sink.push(Intent::OpenProgram(path));
    }

    toolbar_divider(ui, palette, compact);
    transport_group(ui, view, state, compact, sink);
    toolbar_divider(ui, palette, compact);

    // Home runs the firmware homing cycle; safe to offer whenever connected and not already moving. Drawn as a
    // ghost button (transparent rest, design §03) so it reads as a secondary action beside the framed groups.
    // Compact keeps just the ⌂ glyph the full label already leads with.
    let badge = view.badge_state();
    let can_home = connected && !matches!(badge, BadgeState::Run | BadgeState::Jog | BadgeState::Home);
    let home_color = if can_home { palette.text_dim } else { palette.text_disabled };
    let home_label = if compact { "⌂".to_string() } else { crate::tr!("btn-home") };
    let home = egui::Button::new(RichText::new(home_label).color(home_color)).fill(Color32::TRANSPARENT);
    if ui
      .add_enabled(can_home, home)
      .on_hover_text(compact_tip(compact, crate::tr!("btn-home"), crate::tr!("tip-home")))
      .clicked()
    {
      sink.push(Intent::Home);
    }

    // Settings takes the same ghost treatment as Home: the two sit together as secondary chrome actions beside
    // the framed connect/transport groups, and a filled Settings next to a ghost Home read as two accidental
    // styles rather than one deliberate pair. Compact: the 🛠 wrench (firmware/machine settings — deliberately
    // distinct from the ⚙ app-settings gear beside the badge).
    let settings_label = if compact { "🛠".to_string() } else { crate::tr!("btn-settings") };
    let settings =
      egui::Button::new(RichText::new(settings_label).color(palette.text_dim)).fill(Color32::TRANSPARENT);
    if ui
      .add(settings)
      .on_hover_text(compact_tip(compact, crate::tr!("btn-settings"), crate::tr!("tip-settings")))
      .clicked()
    {
      state.settings_open = !state.settings_open;
    }
    // The LTR content ends here; its right edge is one half of the fit measurement.
    let ltr_end = ui.min_rect().right();

    // Right-aligned state badge so it is always visible regardless of toolbar width. The leading space is the
    // bar's right-edge padding (`padding:0 10px`, design §03), mirroring the inset at the left edge. The ⚙ gear
    // (the APP settings dialog — language/theme, distinct from the firmware Settings) sits just left of the badge
    // as ghost chrome. The closure reports its content's LEFT edge — the other half of the fit measurement.
    let rtl_left = ui
      .with_layout(Layout::right_to_left(Align::Center), |ui| {
        ui.add_space(Metrics::TOOLBAR_PAD_X);
        state_badge(ui, view, state);
        // Icon-tight padding: the bar is genuinely full at the 800px minimum window, and the gear at the standard
        // 10px button padding collided with the Settings button there. 4px keeps it a comfortable ~22px target.
        ui.spacing_mut().button_padding = Vec2::new(4.0, 0.0);
        let gear =
          egui::Button::new(RichText::new("⚙").size(14.0).color(palette.text_dim)).fill(Color32::TRANSPARENT);
        if ui.add(gear).on_hover_text(crate::tr!("tip-app-settings")).clicked() {
          state.app_settings_open = !state.app_settings_open;
        }
        ui.min_rect().left()
      })
      .inner;
    (ltr_end, rtl_left)
  })
  .inner;

  // The fit verdict for the NEXT frame: while rendering FULL, record what the full form actually needs (LTR
  // width + one gap + the right cluster's width); in either form, compact exactly when that requirement exceeds
  // the bar. Stored in temp memory so the flip lands next frame without a relayout mid-pass.
  let full_span = (ltr_end - bar.left()) + Metrics::TOOLBAR_GAP + (bar.right() - rtl_left);
  if !compact {
    fit.full_needs = full_span;
  }
  fit.compact = fit.full_needs > bar.width();
  ui.ctx().data_mut(|d| d.insert_temp(fit_id, fit));

  // Fabulous mode: lay a thin Pride rainbow along the toolbar's bottom edge, overpainting the divider. Drawn
  // after the bar's content so it sits on top, and bookended by the matching band on the status bar.
  if state.fabulous {
    let stripe = egui::Rect::from_min_max(
      egui::pos2(bar.left(), bar.bottom() - Metrics::PRIDE_STRIPE_H),
      egui::pos2(bar.right(), bar.bottom()),
    );
    paint_pride(ui.painter(), stripe);
  }
}

/// The hover text for a toolbar control: in compact (icon-only) form the full label leads the explanation so
/// the glyph is never the only name the operator gets; in full form the explanation stands alone (the label is
/// already on the button).
fn compact_tip(compact: bool, label: String, tip: String) -> String {
  if compact { format!("{label} — {tip}") } else { tip }
}

/// The Run/Hold/Stop segmented group plus a separate Abort control. The leading segment starts a stream (Idle) or
/// resumes (Hold); Hold issues a feed-hold; Stop issues the GRACEFUL program stop (`0x86` — decelerate, flush,
/// return to Idle, no alarm); the standalone Abort issues the HARD soft-reset (`0x18` → `ALARM:3`). Enable/emphasis
/// come from the pure [`TransportGroup`] matrix so the view stays a renderer. In `compact` form (the toolbar
/// could not fit its full labels — see [`ToolbarFit`]) the segments render their transport glyphs alone, with the
/// full translated labels on hover; the colour coding (green run / amber stop / red abort) is unchanged, so the
/// safety semantics never depend on the text fitting.
pub(crate) fn transport_group(ui: &mut egui::Ui, view: &ViewState, state: &UiState, compact: bool,
  sink: &mut IntentSink) {
  let palette = state.style.palette;
  use egui::CornerRadius;
  let group = TransportGroup::for_state(view.badge_state(), !state.program.is_empty());

  let run_full = if group.run_is_resume {
    crate::tr!("transport-resume")
  } else if group.run_active {
    crate::tr!("transport-running")
  } else {
    crate::tr!("transport-run")
  };
  let run_label = if compact { "▶".to_string() } else { run_full.clone() };
  // Joined segmented group (design §03): the three buttons abut with no gap and only the outer corners are
  // rounded — Run rounds its left, Stop its right, Hold stays square.
  let r = Metrics::CONTROL_RADIUS;
  let left = CornerRadius { nw: r, sw: r, ne: 0, se: 0 };
  let mid = CornerRadius::ZERO;
  let right = CornerRadius { nw: 0, sw: 0, ne: r, se: r };
  // Zero the gap only between the three joined segments, then restore the toolbar gap on the way out — otherwise
  // the `item_spacing.x = 0` leaked onto the shared toolbar `ui`, cramming Home/Settings hard against the group.
  let prev_gap = ui.spacing().item_spacing.x;
  ui.spacing_mut().item_spacing.x = 0.0;

  let mut run_button = egui::Button::new(RichText::new(run_label).color(palette.state_run)).corner_radius(left);
  if group.run_active {
    run_button = run_button.fill(palette.inset);
  }
  let mut run_resp = ui.add_enabled(group.run_enabled, run_button);
  if compact {
    // Icon-only: the full (state-dependent) label rides on hover so ▶ is never the only name the control has.
    run_resp = run_resp.on_hover_text(run_full);
  }
  if run_resp.clicked() {
    sink.push(Intent::RunOrResume);
  }
  let hold_label = if compact { "⏸".to_string() } else { crate::tr!("transport-hold") };
  let hold = egui::Button::new(hold_label).corner_radius(mid);
  if ui
    .add_enabled(group.hold_enabled, hold)
    .on_hover_text(compact_tip(compact, crate::tr!("transport-hold"), crate::tr!("tip-hold")))
    .clicked()
  {
    sink.push(Intent::Realtime(RealtimeCommand::FeedHold));
  }
  // Stop is now the GRACEFUL program stop (`0x86`): the everyday "stop the job cleanly" button. It decelerates to a
  // block boundary, flushes the queue and returns to Idle with no alarm, so it reads as a normal-weight control
  // (amber, not danger-red) — the hard reset lives in the separate Abort button beside the group.
  let stop_label = if compact { "■".to_string() } else { crate::tr!("transport-stop") };
  let stop = egui::Button::new(RichText::new(stop_label).color(palette.state_hold)).corner_radius(right);
  if ui
    .add_enabled(group.stop_enabled, stop)
    .on_hover_text(compact_tip(compact, crate::tr!("transport-stop"), crate::tr!("tip-stop")))
    .clicked()
  {
    sink.push(Intent::Realtime(RealtimeCommand::ProgramStop));
  }
  // Restore the toolbar gap before the standalone Abort so it sits apart from the joined segments, signalling it is
  // a separate, weightier action rather than a fourth segment of the group.
  ui.spacing_mut().item_spacing.x = prev_gap;

  // Abort / E-stop: the HARD soft-reset (`0x18` → `ALARM:3`). Visually distinct — danger-red, fully rounded, set
  // apart from the segmented group — and available the instant a transport is attached (even mid-handshake), so the
  // operator always has an emergency reset. The graceful Stop above is the routine control; this is the panic stop.
  let abort_label = if compact { "⏹".to_string() } else { crate::tr!("transport-abort") };
  let abort =
    egui::Button::new(RichText::new(abort_label).color(palette.state_alarm)).corner_radius(Metrics::CONTROL_RADIUS);
  if ui
    .add_enabled(group.abort_enabled, abort)
    .on_hover_text(compact_tip(compact, crate::tr!("transport-abort"), crate::tr!("tip-abort")))
    .clicked()
  {
    sink.push(Intent::Realtime(RealtimeCommand::SoftReset));
  }

  // Simulate: a host-only physics ETA over the loaded program, available whenever a program is loaded — even
  // disconnected, since it sends nothing to the board. Drawn as a ghost button (transparent rest, like Home) so it
  // reads as a secondary, non-machine action set apart from the run controls. Disabled (greyed) with no program.
  let has_program = !state.program.is_empty();
  let sim_color = if has_program { palette.text_dim } else { palette.text_disabled };
  // `≈` (approximately equal) — an "estimate" glyph Roboto actually covers. The earlier `∿` (sine wave) is in
  // neither vendored face nor egui's fallback fonts, so it rendered as a tofu box on every platform.
  let sim_label = if compact { "≈".to_string() } else { crate::tr!("transport-simulate") };
  let simulate = egui::Button::new(RichText::new(sim_label).color(sim_color)).fill(Color32::TRANSPARENT);
  if ui
    .add_enabled(has_program, simulate)
    .on_hover_text(compact_tip(compact, crate::tr!("transport-simulate"), crate::tr!("tip-simulate")))
    .clicked()
  {
    sink.push(Intent::Simulate);
  }

  // Autolevel: arm height-map correction for the next stream/simulate. A selectable (toggle) label so its armed
  // state reads at a glance; the mesh check + the actual `correct_program` rewrite happen at stream time (with a
  // notice if no mesh is probed). Available whenever a program is loaded, like Simulate — it is a host-only pre-pass.
  let level_label = if compact { "⌗".to_string() } else { crate::tr!("transport-autolevel") };
  // A ghost button like Simulate, but drawn `.selected()` when armed so its on/off state reads at a glance. (egui
  // has no free-standing `SelectableLabel` widget type in this version — a selected Button is the idiom here.)
  let level_color = if has_program { palette.text_dim } else { palette.text_disabled };
  let level = egui::Button::new(RichText::new(level_label).color(level_color))
    .fill(Color32::TRANSPARENT)
    .selected(state.autolevel_enabled);
  if ui
    .add_enabled(has_program, level)
    .on_hover_text(compact_tip(compact, crate::tr!("transport-autolevel"), crate::tr!("tip-autolevel")))
    .clicked()
  {
    sink.push(Intent::AutolevelToggle(!state.autolevel_enabled));
  }
}
