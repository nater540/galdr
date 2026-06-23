//! Layout metrics for the skirnir UI: the exact pixel dimensions from `Skirnir.dc.html` (§02 component sheet
//! and §03 full-window mockup), kept central so the views size widgets against the design rather than against
//! egui's defaults. Every value here is lifted verbatim from a concrete `style`/`height`/`padding` in the
//! design doc and named for its role, so a reviewer can diff the constant against the mock.
//!
//! Pairs with [`super::theme`] (colour) the way the design's tokens table pairs spacing with palette: the
//! theme answers "what colour", this module answers "how big". Both are thin token layers the thin views read.

use eframe::egui::{Margin, Vec2};

/// Layout dimensions, grouped to mirror the design doc's structure. All values are device-independent points,
/// matching the design's CSS pixels 1:1 (egui points are CSS-pixel-sized at the default scale).
pub struct Metrics;

impl Metrics {
  // ── Chrome bars (design §03) ──────────────────────────────────────────────────────────────────────────
  /// Title bar height (`height:28px`, design §03). Not drawn by us under a native title bar, but kept for the
  /// in-app chrome math and reference.
  pub const TITLE_BAR_H: f32 = 28.0;
  /// Menu bar height (`height:24px`, design §03).
  pub const MENU_BAR_H: f32 = 24.0;
  /// Main toolbar height (`height:40px`, design §03).
  pub const TOOLBAR_H: f32 = 40.0;
  /// Status bar height (`height:24px`, design §03).
  pub const STATUS_BAR_H: f32 = 24.0;
  /// Toolbar horizontal inner padding (`padding:0 10px`, design §03).
  pub const TOOLBAR_PAD_X: f32 = 10.0;
  /// Gap between toolbar items (`gap:6px`, design §03).
  pub const TOOLBAR_GAP: f32 = 6.0;

  // ── Section / tab headers (design §03 — the recurring 30px strip) ──────────────────────────────────────
  /// Section-header and tab-strip bar height (`height:30px`, design §03 — DRO/Jog/Overrides/Probe/Settings/
  /// Toolpath headers and the Console/Program dock tabs all share this).
  pub const HEADER_H: f32 = 30.0;
  /// Section-header / tab horizontal padding (`padding:0 14px`, design §03).
  pub const HEADER_PAD_X: f32 = 14.0;
  /// Section-header title size (`font:500 11px`, design §03).
  pub const HEADER_TEXT: f32 = 11.0;
  /// Section-header letter-spacing (`letter-spacing:0.1em` on an 11px title ≈ 1.1px extra per glyph, design
  /// §03). egui has no per-letter tracking on a `RichText`, so this is applied as inter-glyph spacing where we
  /// hand-letter a header; documented here so the intent is recorded.
  pub const HEADER_TRACKING_EM: f32 = 0.1;
  /// Dock-tab title size (`font:…11.5px`, design §03 — the dock tabs run slightly larger than panel headers).
  pub const TAB_TEXT: f32 = 11.5;
  /// Active-tab accent underline thickness (`border-bottom:2px`, design §03).
  pub const TAB_UNDERLINE: f32 = 2.0;

  // ── Controls (design §02 buttons + §03 toolbar) ───────────────────────────────────────────────────────
  /// Toolbar control height (`height:26px` on the connect group, Open, transport segments, Home, design §03).
  pub const TOOLBAR_CONTROL_H: f32 = 26.0;
  /// Toolbar control horizontal padding (`padding:0 10px`, design §03).
  pub const TOOLBAR_CONTROL_PAD_X: f32 = 10.0;
  /// Default button inner padding (`padding:6px 14px`, design §02 buttons).
  pub const BUTTON_PAD: Vec2 = Vec2::new(14.0, 6.0);
  /// Toolbar button inner padding (`padding:0 10px` with the height fixed by the bar — design §03).
  pub const TOOLBAR_BUTTON_PAD: Vec2 = Vec2::new(10.0, 0.0);
  /// Panel control row height (`row 22`, design §01 spacing legend — "Control height").
  pub const PANEL_CONTROL_H: f32 = 22.0;
  /// Control corner radius (`2 · control`, design §01 radii).
  pub const CONTROL_RADIUS: u8 = 2;
  /// Panel corner radius (`3 · panel`, design §01 radii).
  pub const PANEL_RADIUS: u8 = 3;

  // ── State badge (design §03) ───────────────────────────────────────────────────────────────────────────
  /// State-badge inner padding (`padding:5px 10px`, design §03).
  pub const BADGE_PAD: Vec2 = Vec2::new(10.0, 5.0);
  /// State-badge dot diameter (`width:8px;height:8px`, design §03).
  pub const BADGE_DOT: f32 = 8.0;
  /// Status-bar dot diameter (`width:6px;height:6px`, design §03).
  pub const STATUS_DOT: f32 = 6.0;

  // ── DRO (design §03 left column) ───────────────────────────────────────────────────────────────────────
  /// DRO body padding (`padding:14px 18px 16px`, design §03).
  pub const DRO_PAD: Margin = Margin { left: 18, right: 18, top: 14, bottom: 16 };
  /// DRO axis-value size (`font:500 34px`, design §03 — the body window uses 34; the close-up runs larger).
  pub const DRO_VALUE: f32 = 34.0;
  /// DRO axis-letter size (`font:700 13px`, design §03).
  pub const DRO_LETTER: f32 = 13.0;
  /// DRO unit-suffix size (`font:400 10px`, design §03).
  pub const DRO_UNIT: f32 = 10.0;
  /// Gap between DRO rows (`gap:10px`, design §03).
  pub const DRO_ROW_GAP: f32 = 10.0;
  /// WPos/MPos toggle inner padding (`padding:2px 7px`, design §03).
  pub const DRO_TOGGLE_PAD: Vec2 = Vec2::new(7.0, 2.0);

  // ── Jog pad (design §03) ───────────────────────────────────────────────────────────────────────────────
  /// Jog pad body padding (`padding:14px 18px 18px`, design §03).
  pub const JOG_PAD: Margin = Margin { left: 18, right: 18, top: 14, bottom: 18 };
  /// Jog XY arrow-cell size (`repeat(3, 32px)`, design §03).
  pub const JOG_CELL: f32 = 32.0;
  /// Gap between jog cells (`gap:4px`, design §03).
  pub const JOG_GAP: f32 = 4.0;
  /// Jog icon glyph size (`width:14`, design §03).
  pub const JOG_ICON: f32 = 14.0;

  // ── Right column (design §03) ──────────────────────────────────────────────────────────────────────────
  /// Override / probe body padding (`padding:14px 16px 16px`, design §03).
  pub const RIGHT_PAD: Margin = Margin { left: 16, right: 16, top: 14, bottom: 16 };
  /// Override slider track height (`height:6px`, design §03).
  pub const SLIDER_H: f32 = 6.0;

  // ── Bottom dock (design §03) ───────────────────────────────────────────────────────────────────────────
  /// Bottom dock height (`height:200px`, design §03).
  pub const DOCK_H: f32 = 200.0;
  /// Bottom dock height when collapsed: just the 30px tab strip stays visible so the operator can still read
  /// the tabs and re-expand, while the body (console/program) is hidden and the viewport reclaims the space.
  pub const DOCK_COLLAPSED_H: f32 = Self::HEADER_H;
  /// Dock progress-bar width (`width:260px`, design §03).
  pub const PROGRESS_W: f32 = 260.0;
  /// Dock / override progress-bar height (`height:6px`, design §03).
  pub const PROGRESS_H: f32 = 6.0;
  /// Horizontal gap between the dock progress readout's fields and their `·` separators. The strip's right
  /// closure inherits the tab row's zeroed `item_spacing`, so the readout sets this explicitly to keep its
  /// fields from running together (the crammed `9%0:51` the user flagged).
  pub const DOCK_PROGRESS_GAP: f32 = 8.0;
  /// Width the dock progress readout reserves for its text fields (count, two `·` separators, percent, clock)
  /// beside the bar. When the strip cannot hold this plus [`Self::PROGRESS_W`], the bar is dropped first so the
  /// block degrades gracefully on a narrow window instead of overflowing into an unpainted gap.
  pub const DOCK_PROGRESS_TEXT_RESERVE: f32 = 190.0;
  /// Console / command-line body padding x (`padding:…14px`/`…12px`, design §03 — use 14 to match the log).
  pub const CONSOLE_PAD_X: f32 = 14.0;
  /// Send-button horizontal padding (`padding:0 16px`, design §03).
  pub const SEND_PAD_X: f32 = 16.0;

  // ── Body grid (design §03) ─────────────────────────────────────────────────────────────────────────────
  /// Left controls column width (`268px`, design §03).
  pub const LEFT_COL_W: f32 = 268.0;
  /// Right program column width (`286px`, design §03).
  pub const RIGHT_COL_W: f32 = 286.0;
  /// 1px panel divider thickness (`1px dividers`, design §01 stroke notes).
  pub const DIVIDER: f32 = 1.0;

  /// Resolve the bottom dock's pinned height from its collapsed state: the full [`DOCK_H`](Self::DOCK_H) body
  /// when expanded, or just the [`DOCK_COLLAPSED_H`](Self::DOCK_COLLAPSED_H) tab strip when collapsed. Pure so
  /// the shell can size the panel deterministically and the choice is unit-tested without a window.
  pub fn dock_height(collapsed: bool) -> f32 {
    if collapsed { Self::DOCK_COLLAPSED_H } else { Self::DOCK_H }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn chrome_bar_heights_match_the_design() {
    // The bars the user reads first: a 40px toolbar over a 30px header strip over a 24px status bar.
    assert_eq!(Metrics::TOOLBAR_H, 40.0);
    assert_eq!(Metrics::HEADER_H, 30.0);
    assert_eq!(Metrics::STATUS_BAR_H, 24.0);
    assert_eq!(Metrics::MENU_BAR_H, 24.0);
    assert_eq!(Metrics::TITLE_BAR_H, 28.0);
  }

  #[test]
  fn control_sizing_matches_the_component_sheet() {
    // §02 buttons: padding 6px 14px (x,y order for egui's button_padding).
    assert_eq!(Metrics::BUTTON_PAD, Vec2::new(14.0, 6.0));
    // §03 toolbar controls: 26px tall.
    assert_eq!(Metrics::TOOLBAR_CONTROL_H, 26.0);
    // §01 spacing legend: panel control row 22px.
    assert_eq!(Metrics::PANEL_CONTROL_H, 22.0);
  }

  #[test]
  fn dock_and_tab_metrics_match_the_design() {
    // The dock tab strip is the same 30px bar as a section header, with a 2px active underline and a 260px
    // progress bar — the dock-header geometry the user flagged.
    assert_eq!(Metrics::HEADER_H, 30.0);
    assert_eq!(Metrics::TAB_UNDERLINE, 2.0);
    assert_eq!(Metrics::PROGRESS_W, 260.0);
    assert_eq!(Metrics::PROGRESS_H, 6.0);
    assert_eq!(Metrics::DOCK_H, 200.0);
  }

  #[test]
  fn dock_height_resolves_from_collapsed_state() {
    // Expanded opens at the spec'd 200px dock; collapsed shrinks to just the 30px tab strip so the body hides
    // and the viewport reclaims the freed 170px.
    assert_eq!(Metrics::dock_height(false), Metrics::DOCK_H);
    assert_eq!(Metrics::dock_height(false), 200.0);
    assert_eq!(Metrics::dock_height(true), Metrics::DOCK_COLLAPSED_H);
    assert_eq!(Metrics::dock_height(true), Metrics::HEADER_H);
    assert!(Metrics::dock_height(true) < Metrics::dock_height(false), "collapsing must free vertical space");
  }

  #[test]
  fn jog_pad_is_a_three_by_three_grid_of_32px_cells() {
    assert_eq!(Metrics::JOG_CELL, 32.0);
    assert_eq!(Metrics::JOG_GAP, 4.0);
  }

  #[test]
  fn body_columns_match_the_fixed_grid() {
    assert_eq!(Metrics::LEFT_COL_W, 268.0);
    assert_eq!(Metrics::RIGHT_COL_W, 286.0);
  }
}
