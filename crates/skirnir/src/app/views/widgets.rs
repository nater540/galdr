//! Shared render helpers used across several views: header/tab strips, chips, dividers, and layout primitives.

use super::*;

/// Draw the recurring 30px header strip (design §03): a fixed-height `panelAlt` bar with `0 14px` padding, the
/// title at the left, the `right` closure's controls pulled to the right edge, and a 1px divider along the
/// bottom. The bar height is pinned (not content-driven) so every section header and the dock tab strip line
/// up. `title` is rendered already-styled by the caller's pick; pass it as the design's uppercase tracked
/// section title via [`section_header`], or hand-style it (e.g. dock tabs) and call this directly.
pub(crate) fn header_bar(ui: &mut egui::Ui, palette: Palette, left: impl FnOnce(&mut egui::Ui), right: impl FnOnce(&mut egui::Ui)) {
  // Claim the full strip up front so the fill and divider span the panel's width regardless of content.
  let width = ui.available_width();
  let (rect, _) = ui.allocate_exact_size(Vec2::new(width, Metrics::HEADER_H), egui::Sense::hover());
  let painter = ui.painter();
  painter.rect_filled(rect, 0.0, palette.panel_alt);
  // 1px bottom divider, matching the `border-bottom:1px solid #2E2E2E` under every header in the mock.
  let y = rect.bottom() - 0.5;
  painter.hline(rect.x_range(), y, egui::Stroke::new(Metrics::DIVIDER, palette.divider));

  // Lay the header content inside the strip, vertically centred, with the design's 14px horizontal padding.
  let content = rect.shrink2(Vec2::new(Metrics::HEADER_PAD_X, 0.0));
  let builder = egui::UiBuilder::new().max_rect(content).layout(Layout::left_to_right(Align::Center));
  let mut content_ui = ui.new_child(builder);
  left(&mut content_ui);
  content_ui.with_layout(Layout::right_to_left(Align::Center), right);
}

/// Draw a section header with the design's uppercase, letter-tracked, dim title and no right-side controls —
/// the common case above the override/probe/toolpath/jog sections.
pub fn section_header(ui: &mut egui::Ui, palette: Palette, title: &str) {
  header_bar(ui, palette, |ui| header_title(ui, palette, title), |_ui| {});
}

/// Draw a floating setup dialog's title bar in the SAME [`section_header`] treatment the inline panels used —
/// the uppercase, letter-tracked, dim title over the `panelAlt` strip with its bottom divider — so the launched
/// wizards read as the app's own panels rather than egui's default (mixed-case, bright) window chrome. When
/// `closable`, a ghost `×` sits at the strip's right edge; clicking it returns `true` so the caller clears the
/// open dialog. A running (forced-open) wizard passes `closable = false`: its bar carries no `×` and the flow
/// can only be ended from the in-body Cancel — the same "can't hide a moving machine" safety rule the window's
/// missing close button enforced before.
pub(crate) fn setup_dialog_header(ui: &mut egui::Ui, palette: Palette, title: &str, closable: bool) -> bool {
  let mut close = false;
  header_bar(
    ui, palette,
    |ui| header_title(ui, palette, title),
    |ui| {
      if closable {
        // Icon-tight ghost button (no fill, no border) so the glyph is a compact ~22px target flush to the
        // strip's 14px right pad — the same ghost-chrome recipe the toolbar gear uses.
        ui.spacing_mut().button_padding = Vec2::new(4.0, 0.0);
        let x = egui::Button::new(RichText::new("×").size(16.0).color(palette.text_dim)).fill(Color32::TRANSPARENT);
        if ui.add(x).on_hover_text(crate::tr!("tip-close-dialog")).clicked() {
          close = true;
        }
      }
    },
  );
  close
}

/// Draw a dock tab strip in the design's §03 style: the same 30px `panelAlt` bar, but with mixed-case tab
/// labels at 11.5px (Medium when active / Regular when dim) and a 2px accent underline drawn under the active
/// tab. `tabs` are `(label, active)` pairs laid left→right; `right` fills the strip's right edge (e.g. the
/// progress readout). The tabs are clickable: the index of a clicked tab is returned so the caller can flip the
/// dock's active tab, while the underline marks the current selection. Returns `None` when no tab was clicked
/// this frame.
pub(crate) fn tab_strip(ui: &mut egui::Ui, palette: Palette, tabs: &[(&str, bool)], right: impl FnOnce(&mut egui::Ui)) -> Option<usize> {
  let labels: Vec<(String, bool)> = tabs.iter().map(|(t, a)| (t.to_string(), *a)).collect();
  let mut clicked = None;
  header_bar(
    ui, palette,
    |ui| {
      ui.spacing_mut().item_spacing.x = 0.0;
      for (index, (label, active)) in labels.iter().enumerate() {
        let (weight, color) = if *active { (true, palette.text) } else { (false, palette.text_dim) };
        let mut text = RichText::new(label).size(Metrics::TAB_TEXT).color(color);
        if weight {
          text = text.strong();
        }
        // Each tab claims `0 14px` of horizontal room within the (already vertically centred) strip so the
        // underline spans its full width. The header bar lays this row out at `Align::Center`, so the label
        // sits on the strip's vertical midline without extra padding. The label is sensed for clicks so the
        // dock can switch tabs without a separate button chrome (the design draws the tabs as bare labels).
        let frame = egui::Frame::new().inner_margin(egui::Margin { left: 14, right: 14, top: 0, bottom: 0 });
        let response = frame.show(ui, |ui| ui.label(text)).response.interact(egui::Sense::click());
        if response.clicked() {
          clicked = Some(index);
        }
        if *active {
          // 2px accent underline pinned to the bottom of the 30px strip (not the label), the design's
          // active-tab signature. Snap to the strip floor so all tabs share one underline baseline.
          let r = response.rect;
          let floor = ui.max_rect().bottom();
          let y = floor - Metrics::TAB_UNDERLINE * 0.5;
          ui.painter().hline(r.x_range(), y, egui::Stroke::new(Metrics::TAB_UNDERLINE, palette.accent));
        }
      }
    },
    right,
  );
  clicked
}

/// Render a header title in the design's section-header type: 11px Medium, uppercase, dim, with the 0.1em
/// tracking approximated by `extra_letter_spacing` so the headers read as small-caps labels, not body text.
pub(crate) fn header_title(ui: &mut egui::Ui, palette: Palette, title: &str) {
  let text = RichText::new(title.to_ascii_uppercase())
    .size(Metrics::HEADER_TEXT)
    .color(palette.text_dim)
    .strong()
    .extra_letter_spacing(Metrics::HEADER_TEXT * Metrics::HEADER_TRACKING_EM);
  ui.label(text);
}

/// The shared "state-toggled chip" frame: a filled, single-pixel-stroked, control-radius inset that both the
/// machine-state badge and the endstop chips draw. Centralising it keeps the chip theme contract (1px stroke,
/// [`Metrics::CONTROL_RADIUS`] corners) in one place; callers pass the asserted/clear `fill`/`border`, the inner
/// margin (badges and endstop chips pad differently), and the chip's content. Visual output is identical to the
/// per-site frames it replaces.
pub(crate) fn chip_frame(
  ui: &mut egui::Ui, fill: Color32, border: Color32, margin: egui::Margin,
  content: impl FnOnce(&mut egui::Ui),
) -> egui::Response {
  egui::Frame::new()
    .fill(fill)
    .stroke(egui::Stroke::new(1.0, border))
    .inner_margin(margin)
    .corner_radius(Metrics::CONTROL_RADIUS)
    .show(ui, content)
    .response
}

/// Paint a smooth left-to-right six-stripe Pride rainbow filling `rect` — the "fabulous" easter-egg accent. The
/// band is sampled from [`Palette::pride_at`] as a run of thin vertical slices so it blends rather than showing six
/// hard bands, with each slice overdrawn by a pixel to hide the seams. A no-op for a non-positive-width rect.
pub(crate) fn paint_pride(painter: &egui::Painter, rect: egui::Rect) {
  const SLICES: usize = 64;
  let slice_w = rect.width() / SLICES as f32;
  if slice_w <= 0.0 {
    return;
  }
  for i in 0..SLICES {
    let t = (i as f32 + 0.5) / SLICES as f32; // sample each slice at its centre.
    let x = rect.left() + i as f32 * slice_w;
    let slice = egui::Rect::from_min_size(egui::pos2(x, rect.top()), Vec2::new(slice_w + 1.0, rect.height()));
    painter.rect_filled(slice, 0.0, Palette::pride_at(t));
  }
}

/// Lay out a horizontal row of buttons at exact, vertically-aligned rects, so none of them drift the way
/// `add_sized` inside a `horizontal` layout does — there each successive button crept a few pixels lower
/// (the staggered DRO "zero" row the user flagged: same height, but each ~2px below the last). The row claims
/// the full available width at [`Metrics::PANEL_CONTROL_H`] height, splits it by `weights` with `gap` between
/// cells, and calls `cell` once per index with the placed rect so the caller paints its own button there (via
/// [`egui::Ui::put`], which fills the rect exactly) and reads its response.
pub(crate) fn button_row(ui: &mut egui::Ui, gap: f32, weights: &[f32], mut cell: impl FnMut(&mut egui::Ui, usize, egui::Rect)) {
  let height = Metrics::PANEL_CONTROL_H;
  let full = ui.available_width();
  let sum: f32 = weights.iter().sum::<f32>().max(f32::EPSILON);
  let avail = (full - gap * (weights.len() as f32 - 1.0)).max(0.0);
  // Reserve the whole row up front (advancing the cursor below it); the per-cell `put` calls then place buttons
  // inside this reserved band without moving the cursor, so every cell shares one top and one bottom edge.
  let (rect, _) = ui.allocate_exact_size(Vec2::new(full, height), egui::Sense::hover());
  let mut x = rect.left();
  for (index, weight) in weights.iter().enumerate() {
    let w = avail * weight / sum;
    let cell_rect = egui::Rect::from_min_size(egui::pos2(x, rect.top()), Vec2::new(w, height));
    cell(ui, index, cell_rect);
    x += w + gap;
  }
}

/// A thin vertical divider for the toolbar: a 1px line at the design's ~22px height, centred in a wider strip
/// of breathing room, replacing egui's full-height `separator()` (the user-flagged toolbar styling, design §03's
/// `1px #2E2E2E` group separators). The divider colour is deliberately subtle against the `panelAlt` bar, so the
/// GROUPING is carried by the extra air around it: the strip plus the toolbar gap on either side gives ~21px
/// between groups versus the 6px within one, which reads as separation even where the hairline itself is faint.
pub(crate) fn toolbar_divider(ui: &mut egui::Ui, palette: Palette, compact: bool) {
  // Compact halves the strip's air: the icon-form bar exists precisely because width ran out, and the icons'
  // own gaps already separate the groups legibly at that density.
  let strip_w = if compact { 3.0 } else { Metrics::TOOLBAR_DIVIDER_W };
  let (rect, _) = ui.allocate_exact_size(Vec2::new(strip_w, Metrics::TOOLBAR_CONTROL_H), egui::Sense::hover());
  let center = rect.center();
  let half = 22.0 * 0.5;
  ui.painter().vline(center.x, (center.y - half)..=(center.y + half), egui::Stroke::new(1.0, palette.divider));
}

/// Draw a dim secondary label in the recurring 11px `text_dim` treatment — the one spelling for the wizard
/// intros, step hints, and readings that repeat across the right-column panels. Returns the label's
/// [`egui::Response`] so callers can still hover/inspect it. Sites that use a DIFFERENT size or colour
/// (`state_run`/`state_alarm`/`text`) stay inline — this helper is only the dominant `text_dim` form.
pub(crate) fn dim_label(ui: &mut egui::Ui, palette: Palette, text: impl Into<String>) -> egui::Response {
  ui.label(RichText::new(text).size(11.0).color(palette.text_dim))
}

/// Add a full-width panel action button at the design's right-column size — the panel's available width by
/// [`Metrics::PANEL_CONTROL_H`]` + 6.0` — returning its [`egui::Response`]. The width is measured at call time,
/// which equals the value the per-panel `let full = …` captured (these panels lay out top-down, where adding a
/// row advances the cursor down without changing the column width), so the rendered button is identical.
pub(crate) fn full_width_button(ui: &mut egui::Ui, label: impl Into<egui::WidgetText>) -> egui::Response {
  let size = Vec2::new(ui.available_width(), Metrics::PANEL_CONTROL_H + 6.0);
  ui.add_sized(size, egui::Button::new(label))
}

/// Add an enabled-gated full-width action button: a [`full_width_button`] wrapped in an `add_enabled_ui(gate)`
/// scope so it greys out and stops responding when `gate` is false, returning the button's [`egui::Response`].
/// This is the exact `add_enabled_ui(gate, |ui| if button.clicked() … )` idiom the wizard steps repeat, and the
/// enclosing scope (hence the widget id) is preserved. Sites that gate MORE than one button in a single scope
/// keep their own `add_enabled_ui` and call [`full_width_button`] inside it, so the shared scope is not split.
pub(crate) fn gated_action_button(ui: &mut egui::Ui, gate: bool, label: impl Into<egui::WidgetText>) -> egui::Response {
  ui.add_enabled_ui(gate, |ui| full_width_button(ui, label)).inner
}

/// Add a labelled `DragValue` row inside a two-column bench-param [`egui::Grid`]: the left label, a `speed`/
/// `range`/`suffix` drag field bound to `value`, then `end_row`. This is the dominant bench-param row; rows that
/// chain extra behaviour (an `.on_hover_text`, a `.changed()` side effect, or a second value in the row) stay
/// inline so their extra wiring is not hidden behind the helper.
pub(crate) fn param_row(ui: &mut egui::Ui, label: impl Into<egui::WidgetText>, value: &mut f64, speed: f64,
  range: std::ops::RangeInclusive<f64>, suffix: &str) {
  ui.label(label);
  ui.add(egui::DragValue::new(value).speed(speed).range(range).suffix(suffix));
  ui.end_row();
}

/// Wrap `content` in the right column's panel [`egui::Frame`] — the design's `RIGHT_PAD` inner margin — matching
/// the frame every right-side view opens with. Behaviourally identical to the inline
/// `Frame::new().inner_margin(Metrics::RIGHT_PAD).show(ui, …)` it replaces.
pub(crate) fn right_panel<R>(ui: &mut egui::Ui, content: impl FnOnce(&mut egui::Ui) -> R) -> egui::InnerResponse<R> {
  egui::Frame::new().inner_margin(Metrics::RIGHT_PAD).show(ui, content)
}

/// Paint a small filled circle inline (a state dot), advancing the cursor by its diameter.
pub(crate) fn dot(ui: &mut egui::Ui, color: Color32, diameter: f32) {
  let (rect, _) = ui.allocate_exact_size(Vec2::splat(diameter), egui::Sense::hover());
  ui.painter().circle_filled(rect.center(), diameter * 0.5, color);
}

/// Render `content` into a DETACHED child ui pinned to exactly the caller's remaining rect, then claim that
/// rect. A child ui's overflow never expands its parent, so whatever the content measures — fractional font
/// rows, a hand-laid row a hair too wide, a future widget — the caller's own measured size stays exactly its
/// allocated rect. This is the side columns' containment: under egui 0.35 a panel whose content measures wider
/// than the panel re-anchors its stored rect on the overflowed edge and shifts the central-region cursor over
/// the column (the clipped-labels bug); with containment that class of overflow is clipped where it happens and
/// can never move layout. (The same principle the panel machinery itself applies via clip — but the measurement
/// escape is only sealed by the detached child.)
pub(crate) fn contained(ui: &mut egui::Ui, content: impl FnOnce(&mut egui::Ui)) {
  let rect = ui.available_rect_before_wrap();
  let mut child = ui.new_child(egui::UiBuilder::new().max_rect(rect).layout(egui::Layout::top_down(Align::Min)));
  child.set_clip_rect(rect.intersect(child.clip_rect()));
  content(&mut child);
  ui.allocate_rect(rect, egui::Sense::hover());
}
