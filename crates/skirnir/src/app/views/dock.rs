//! The bottom dock frame: collapse toggle, progress bar, ETA qualifier, and follow-target logic.

use super::*;

/// Debug probe: the rect the dock CONTENT last rendered into, recorded by [`dock`] each frame and read by the
/// shell's `SKIRNIR_SIZE_TRACE` instrument and the stability tests. Stored in egui TEMP MEMORY (per-context, so
/// parallel test harnesses cannot race each other's readings), same side-channel idea as `slider_rect_probe` —
/// the dock body is not an AccessKit-labelled widget.
pub(crate) mod dock_rect_probe {
  use eframe::egui;

  fn key() -> egui::Id {
    egui::Id::new("skirnir-dock-rect-probe")
  }

  /// Record this frame's dock region rect on its context.
  pub fn record(ctx: &egui::Context, rect: egui::Rect) {
    ctx.data_mut(|d| d.insert_temp(key(), rect));
  }

  /// The most recently recorded dock region rect on this context, if the dock has rendered.
  pub fn last(ctx: &egui::Context) -> Option<egui::Rect> {
    ctx.data(|d| d.get_temp(key()))
  }
}

/// Render the bottom dock's CONTENT (design §03): one surface hosting the Console and Program tabs. The shared
/// tab strip switches `state.active_tab`, the strip's right edge carries the §03 progress readout (acked/total ·
/// 260px bar · percent) plus the collapse toggle for whichever tab is active, and the body below renders the
/// selected tab. Keeping both tabs in one dock matches the mock, where Console and Program share a single dock
/// rather than sitting in separate panels. The hosting geometry — the egui_tiles split when expanded, the fixed
/// collapsed strip otherwise — lives in [`shell_panels`]/[`crate::app::dock_tiles`]; this draws only the content.
pub fn dock(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, time: crate::app::progress::TimeEstimate,
  eta_qualifier: Option<EtaQualifier>, sink: &mut IntentSink) {
  // Record the dock region's rect for the SKIRNIR_SIZE_TRACE debug instrument and the stability tests
  // (implementation-agnostic: this is the region the dock actually rendered into, whatever container hosts it).
  dock_rect_probe::record(ui.ctx(), ui.max_rect());
  let palette = state.style.palette;
  let active = state.active_tab;
  let console_label = crate::tr!("tab-console");
  let program_label = crate::tr!("tab-program");
  let tabs = [
    (console_label.as_str(), active == DockTab::Console),
    (program_label.as_str(), active == DockTab::Program),
  ];
  let progress = view.progress;
  let collapsed = state.dock_collapsed;
  // The strip's right closure lays out right-to-left, so widgets are drawn outermost-right first. The collapse
  // toggle is the outermost-right control (the conventional minimise corner); the §03 progress block sits to its
  // left. The progress rides on the strip regardless of the active tab — it is a dock-level affordance — so the
  // operator always sees streaming progress while reading either tab, even when the body is collapsed.
  let mut toggle_clicked = false;
  let clicked = tab_strip(ui, palette, &tabs, |ui| {
    toggle_clicked = dock_collapse_toggle(ui, palette, collapsed);
    dock_progress(ui, palette, progress, time, eta_qualifier);
  });
  state.active_tab = dock_tab_for_click(active, clicked);
  if toggle_clicked {
    state.dock_collapsed = !state.dock_collapsed;
  }

  // When collapsed only the tab strip remains; the body is hidden and the shell pins the panel to the strip's
  // height so the central viewport reclaims the freed space. The tab strip stays interactive so the operator can
  // still switch tabs and re-expand.
  if state.dock_collapsed {
    return;
  }
  ui.add_space(2.0);
  match state.active_tab {
    DockTab::Console => console_body(ui, view, state, sink),
    DockTab::Program => program_body(ui, view, state),
  }
}

/// Draw the dock's collapse/expand toggle as a small ghost icon button at the right of the tab strip: a `−`
/// minimises the dock to its strip, a `+` restores it (see [`dock_toggle_label`]). Sized to the strip's control
/// height so it sits centred in the 30px bar. Returns whether it was clicked this frame.
fn dock_collapse_toggle(ui: &mut egui::Ui, palette: Palette, collapsed: bool) -> bool {
  let label = dock_toggle_label(collapsed);
  let hint = if collapsed { crate::tr!("tip-expand-dock") } else { crate::tr!("tip-collapse-dock") };
  // Square icon button matching the strip's control height, transparent at rest like the §02 icon-button state
  // (the same ghost treatment as the ⚙ settings and jog-cancel buttons), so it reads as chrome, not a tab. Zero
  // the button padding for this region: the global `BUTTON_PAD` (6px vertical) plus the glyph would inflate the
  // button past the strip's control height, making it overflow the 30px bar and sit off-centre (the user-flagged
  // bug). With no padding the button is pinned to the `PANEL_CONTROL_H` square, which the strip's `Align::Center`
  // layout then centres within the 30px bar. The glyph is held at the header text size so it can't grow the box.
  ui.spacing_mut().button_padding = Vec2::ZERO;
  let size = Vec2::splat(Metrics::PANEL_CONTROL_H);
  let button = egui::Button::new(RichText::new(label).size(Metrics::HEADER_TEXT).color(palette.text_dim))
    .fill(Color32::TRANSPARENT);
  ui.add_sized(size, button).on_hover_text(hint).clicked()
}

/// Draw the §03 dock progress readout: `acked / total`, the green bar, the percent, and the elapsed /
/// estimated-total `m:ss / m:ss` clock, shown only while a program is loaded/streaming (`total > 0`). The
/// strip's right closure lays out right-to-left, so the widgets are drawn rightmost-first; that puts the
/// clock at the left edge of the block and the count nearest the percent, reading left→right as the design's
/// `acked/total · bar · NN% · m:ss / m:ss` with dim `·` separators between the textual fields.
///
/// The strip's `right` closure inherits the tab row's `item_spacing.x = 0` (it is the same child `ui`), which is
/// why the percent and clock previously ran together with no gap. This sets its own roomy row spacing so the
/// fields breathe, and degrades on a narrow strip by dropping the bar first (the least-important field — the
/// percent and count carry the same information) so the block never overflows into an unpainted gap.
fn dock_progress(ui: &mut egui::Ui, palette: Palette, progress: crate::app::view_state::Progress,
  time: crate::app::progress::TimeEstimate, eta_qualifier: Option<EtaQualifier>) {
  use crate::app::progress::format_progress_clock;
  // Show the readout while a program is streaming (`total > 0`) OR a simulation is stored — the latter surfaces the
  // upfront ETA before any stream begins, when the progress total is still zero. With neither, there is nothing to
  // show; bail so the strip stays bare.
  let has_simulation = eta_qualifier.is_some();
  if progress.total == 0 && !has_simulation {
    return;
  }
  // Own the row spacing rather than inheriting the tab strip's zeroed `item_spacing.x`. A roomy 8px gap gives
  // every field air and sits between the adjacent fields and the `·` separators so nothing abuts its neighbour.
  ui.spacing_mut().item_spacing.x = Metrics::DOCK_PROGRESS_GAP;
  // Decide up front whether the bar fits. The toggle was already drawn (this `ui` excludes it), so the remaining
  // width must hold the bar plus the textual fields; when it can't, drop the bar rather than overflow the strip.
  // The bar is also meaningless before a stream (no acked fraction), so it is gated on a real program total too.
  let draw_bar = progress.total > 0
    && ui.available_width() >= Metrics::PROGRESS_W + Metrics::DOCK_PROGRESS_TEXT_RESERVE;

  // The simulation's caveats trail the clock at the far right (drawn first in this right-to-left strip): a
  // "(default settings)" flag when the estimate used the default machine model, and a pause count when the
  // timeline modeled unbounded operator waits. Rendered only when a simulation drives the ETA.
  if let Some(qualifier) = eta_qualifier {
    dock_eta_qualifier(ui, palette, qualifier);
  }
  // Rightmost (after the qualifier): the elapsed / estimated-total clock. `total` is `None` until the ETA is
  // projectable, so its right half shows the dim `--:--` placeholder rather than a wild early guess (a simulation
  // populates `total` immediately, so the upfront figure shows at once). See `format_progress_clock`.
  let clock = format_progress_clock(time.elapsed, time.total);
  ui.label(RichText::new(clock).monospace().size(10.5).color(palette.text_dim));
  // The percent/count/bar fields only mean something once a stream is timing; skip them (and their separator)
  // before streaming so the upfront ETA shows the clock + qualifier alone, not a `0%`/`0 / 0` placeholder row.
  if progress.total == 0 {
    return;
  }
  dock_progress_separator(ui, palette);
  let pct = (progress.fraction() * 100.0).round() as u32;
  ui.label(RichText::new(format!("{pct}%")).monospace().size(11.0).color(palette.text));
  if draw_bar {
    dock_progress_separator(ui, palette);
    let bar = Vec2::new(Metrics::PROGRESS_W, Metrics::PROGRESS_H);
    let (rect, _) = ui.allocate_exact_size(bar, egui::Sense::hover());
    let painter = ui.painter();
    painter.rect_filled(rect, 2.0, palette.inset);
    let mut fill = rect;
    fill.set_width(rect.width() * progress.fraction());
    painter.rect_filled(fill, 2.0, palette.state_run);
  }
  dock_progress_separator(ui, palette);
  ui.label(RichText::new(format!("{} / {}", progress.acked, progress.total)).monospace().size(10.5)
    .color(palette.text_dim));
}

/// Draw the dim middot that separates the dock progress fields (the design's `·`). Pulled out so every gap in
/// [`dock_progress`] uses one consistent glyph and colour instead of repeating the `RichText` at each call site.
fn dock_progress_separator(ui: &mut egui::Ui, palette: Palette) {
  ui.label(RichText::new("·").size(11.0).color(palette.text_disabled));
}

/// Render the simulation's caveats beside the dock clock: a dim "(default settings)" flag when the estimate used
/// the firmware's default machine model rather than the board's real `$$` config, and a "pause(s) at N line(s)"
/// note when the timeline modeled unbounded operator waits (`M0`/`M1`/`M6`) that are excluded from the timed
/// total. Each part is its own label so the strip degrades gracefully; both are omitted when neither applies. The
/// qualifier text is built by the pure [`eta_qualifier_text`] so the copy is unit-tested without a window.
fn dock_eta_qualifier(ui: &mut egui::Ui, palette: Palette, qualifier: EtaQualifier) {
  if let Some(text) = eta_qualifier_text(qualifier) {
    ui.label(RichText::new(text).size(10.0).color(palette.text_disabled));
  }
}

/// Build the dock ETA qualifier string for a simulation, or `None` when there is nothing to qualify (real
/// settings and no modeled pauses). Kept pure (no egui) so the copy — "(default settings)", the pause count, and
/// their joining — is unit-tested without a window. The pause note uses "line"/"lines" so a single pause reads
/// naturally.
pub(crate) fn eta_qualifier_text(qualifier: EtaQualifier) -> Option<String> {
  let mut parts: Vec<String> = Vec::new();
  if qualifier.default_settings {
    parts.push(crate::tr!("eta-default-settings"));
  }
  if qualifier.pauses > 0 {
    parts.push(crate::tr!("eta-pauses", { count: qualifier.pauses as i64 }));
  }
  if parts.is_empty() { None } else { Some(parts.join(" · ")) }
}

/// Decide whether the Program listing should auto-scroll to follow the executing line this frame, and which row
/// to centre on. Returns `Some(line)` only when auto-scroll is on, the stream is live, and the executing line has
/// *moved* since we last followed it — so the view nudges once per advance and otherwise leaves the operator's
/// manual scroll-back alone. `followed` is the last row we scrolled to (`UiState::program_followed_line`), updated
/// by the caller to the returned value. Kept pure (no egui) so the follow logic is unit-testable without a GUI.
pub(crate) fn program_follow_target(
  auto_scroll: bool,
  connection: ConnectionState,
  current: usize,
  program_len: usize,
  followed: Option<usize>,
) -> Option<usize> {
  if !auto_scroll || connection != ConnectionState::Streaming || current >= program_len {
    return None;
  }
  if followed == Some(current) {
    return None;
  }
  Some(current)
}
