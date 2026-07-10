//! Layout metrics for the eitri-app UI, mirroring skirnir's design tokens (the shared `Skirnir.dc.html` §01–§03
//! values) so the CAM app and the sender share one spacing language. Every view sizes widgets against these
//! constants rather than egui's defaults; no ad hoc magic numbers in widget code.
//!
//! Pairs with [`super::theme`] (colour) the way the design's tokens table pairs spacing with palette: the theme
//! answers "what colour", this module answers "how big".

use eframe::egui::{Margin, Vec2};

/// Layout dimensions, in device-independent points (CSS-pixel-sized at the default scale).
pub struct Metrics;

impl Metrics {
  // ── Chrome bars ───────────────────────────────────────────────────────────────────────────────────────
  /// Main toolbar height.
  pub const TOOLBAR_H: f32 = 40.0;
  /// Status bar height.
  pub const STATUS_BAR_H: f32 = 24.0;
  /// Toolbar horizontal inner padding.
  pub const TOOLBAR_PAD_X: f32 = 10.0;
  /// Gap between toolbar items.
  pub const TOOLBAR_GAP: f32 = 6.0;
  /// Width of the strip a toolbar group divider occupies (a 1px hairline centred in it).
  pub const TOOLBAR_DIVIDER_W: f32 = 7.0;
  /// Toolbar control height.
  pub const TOOLBAR_CONTROL_H: f32 = 26.0;

  // ── Section / tab headers (the recurring 30px strip) ─────────────────────────────────────────────────
  /// Section-header and tab-strip bar height (the Project/Parameters headers and the dock tabs share this).
  pub const HEADER_H: f32 = 30.0;
  /// Section-header / tab horizontal padding.
  pub const HEADER_PAD_X: f32 = 14.0;
  /// Section-header title size.
  pub const HEADER_TEXT: f32 = 11.0;
  /// Section-header letter-spacing as a fraction of the title size (applied via `extra_letter_spacing`).
  pub const HEADER_TRACKING_EM: f32 = 0.1;
  /// Dock-tab title size (dock tabs run slightly larger than panel headers).
  pub const TAB_TEXT: f32 = 11.5;
  /// Active-tab accent underline thickness.
  pub const TAB_UNDERLINE: f32 = 2.0;

  // ── Controls ──────────────────────────────────────────────────────────────────────────────────────────
  /// Default button inner padding (x, y — egui order).
  pub const BUTTON_PAD: Vec2 = Vec2::new(14.0, 6.0);
  /// Panel control row height.
  pub const PANEL_CONTROL_H: f32 = 22.0;
  /// Control corner radius.
  pub const CONTROL_RADIUS: u8 = 2;

  // ── Body grid ─────────────────────────────────────────────────────────────────────────────────────────
  /// Left project-tree column width.
  pub const LEFT_COL_W: f32 = 232.0;
  /// Default height of the TOOLPATHS section anchored at the bottom of the left column (resizable by the operator).
  pub const TOOLPATHS_PANEL_H: f32 = 200.0;
  /// Right parameters column width.
  pub const RIGHT_COL_W: f32 = 286.0;
  /// 1px panel divider thickness.
  pub const DIVIDER: f32 = 1.0;
  /// Panel body padding (tree rows, parameter grids).
  pub const PANEL_PAD: Margin = Margin { left: 14, right: 14, top: 10, bottom: 12 };

  // ── Bottom dock ───────────────────────────────────────────────────────────────────────────────────────
  /// The smallest height either side of the central canvas/dock split may be dragged to (egui_tiles
  /// `Behavior::min_size`): the 30px tab strip plus enough body for a few log rows.
  pub const DOCK_MIN_H: f32 = 96.0;
  /// Dock progress-bar width.
  pub const PROGRESS_W: f32 = 220.0;
  /// Dock progress-bar height.
  pub const PROGRESS_H: f32 = 6.0;

  /// The `TextEdit` margin that sizes a single-line field to exactly [`Self::PANEL_CONTROL_H`], so text inputs
  /// sit at the same height as the buttons beside them. egui sizes a text edit as `row_height + vertical
  /// margins` (its `min_size.y` is ignored), so the padding is computed from the actual font row height; the
  /// odd pixel goes to the bottom, and a font taller than the control clamps to zero rather than negative.
  pub fn text_field_margin(row_height: f32, pad_x: i8) -> Margin {
    let total = (Self::PANEL_CONTROL_H - row_height).max(0.0).round() as i8;
    let top = total / 2;
    Margin { left: pad_x, right: pad_x, top, bottom: total - top }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn chrome_bar_heights_match_the_shared_design() {
    assert_eq!(Metrics::TOOLBAR_H, 40.0);
    assert_eq!(Metrics::HEADER_H, 30.0);
    assert_eq!(Metrics::STATUS_BAR_H, 24.0);
  }

  #[test]
  fn control_sizing_matches_the_component_sheet() {
    assert_eq!(Metrics::BUTTON_PAD, Vec2::new(14.0, 6.0));
    assert_eq!(Metrics::TOOLBAR_CONTROL_H, 26.0);
    assert_eq!(Metrics::PANEL_CONTROL_H, 22.0);
  }

  #[test]
  fn text_field_margin_sizes_a_field_to_the_control_height() {
    let m = Metrics::text_field_margin(15.0, 6);
    assert_eq!((m.left, m.right), (6, 6));
    assert_eq!(m.top + m.bottom, 7, "15px text + 7px pad = the 22px control height");
    assert!(m.bottom - m.top <= 1, "the split is as even as integers allow");
    let tall = Metrics::text_field_margin(30.0, 0);
    assert_eq!((tall.top, tall.bottom), (0, 0), "a tall font clamps to zero padding, never negative");
  }
}
