//! The central viewport/console split as an [`egui_tiles`] tree — the dock's host after three hand-rolled
//! resizable-panel fixes went green in the harness but kept self-resizing on the desktop (user decision:
//! cut losses and adopt the share-based engine).
//!
//! Why tiles kills the bug class: egui's resizable panels persist the measured CONTENT rect as next frame's
//! panel size, so anything the content measures — fractional font rows, theme-dependent heights — can feed back
//! into the panel's size. egui_tiles sizes panes TOP-DOWN from [`egui_tiles::Shares`]: the split is a ratio the
//! operator owns, pane content is laid into whatever rect that ratio yields, and nothing the content does can
//! alter the ratio. There is no feedback path to guard.
//!
//! Deliberately NOT adopted: tiles' headline drag-and-drop rearranging. Skirnir's machine-control chrome is
//! design-fixed (operators build muscle memory against stable control placement), so [`CentralBehavior`]
//! disables tile dragging and the tree is exactly one vertical split — toolpath viewport above, console/program
//! dock below. The fixed chrome (toolbar, banners, status bar, side columns) stays hand-rolled outside tiles.
//!
//! The split fraction persists via `profile.ron` (the app-owned, auto-saved store) — NOT `config.json`, which is
//! operator-owned and only written on an explicit Save, and NOT egui memory, which this build does not persist
//! across runs (eframe's `persistence` feature is off).

use eframe::egui;
use egui_tiles::{Container, SimplificationOptions, Tile, TileId, Tiles, Tree, UiResponse};

use super::intent::IntentSink;
use super::metrics::Metrics;
use super::view_state::ViewState;
use super::views::{self, EtaQualifier, UiState};

/// The fraction of the central region the dock opens with on a fresh profile — defined in [`crate::profile`]
/// (which also compiles without the `gui` feature) and re-exported here as the split's natural home.
pub const DEFAULT_DOCK_FRACTION: f32 = crate::profile::DEFAULT_DOCK_FRACTION;

/// The share-fraction bounds a persisted/loaded value is held to, so a hand-edited profile cannot collapse the
/// dock to nothing or swallow the viewport (the tree's own `min_size` guards live drags the same way).
const DOCK_FRACTION_RANGE: std::ops::RangeInclusive<f32> = 0.05..=0.9;

/// Which pane of the central split is being rendered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CentralPane {
  /// The toolpath viewport (top).
  Viewport,
  /// The console/program dock (bottom).
  Dock,
}

/// The central region's tiles tree plus the ids of its fixed members. Held in [`UiState`] across frames (the
/// tree carries the operator's split ratio); the shell takes it out for the render so the [`CentralBehavior`]
/// can borrow the rest of the state, then puts it back.
#[derive(Clone)]
pub struct CentralSplit {
  /// The one-vertical-split tree: `[Viewport, Dock]`.
  tree: Tree<CentralPane>,
  /// The root Linear container's id, for share access.
  root: TileId,
  /// The viewport pane's id.
  viewport: TileId,
  /// The dock pane's id.
  dock: TileId,
}

// `egui_tiles::Tree` does not implement `Debug`, and `UiState` derives it; the split's meaningful state is the
// ratio, so print that.
impl std::fmt::Debug for CentralSplit {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    f.debug_struct("CentralSplit").field("dock_fraction", &self.dock_fraction()).finish()
  }
}

impl CentralSplit {
  /// Build the split with the dock at `dock_fraction` of the central height (clamped to sane bounds).
  pub fn new(dock_fraction: f32) -> Self {
    let fraction = dock_fraction.clamp(*DOCK_FRACTION_RANGE.start(), *DOCK_FRACTION_RANGE.end());
    let mut tiles = Tiles::default();
    let viewport = tiles.insert_pane(CentralPane::Viewport);
    let dock = tiles.insert_pane(CentralPane::Dock);
    let root = tiles.insert_vertical_tile(vec![viewport, dock]);
    if let Some(Tile::Container(Container::Linear(linear))) = tiles.get_mut(root) {
      linear.shares.set_share(viewport, 1.0 - fraction);
      linear.shares.set_share(dock, fraction);
    }
    CentralSplit { tree: Tree::new("central-split", root, tiles), root, viewport, dock }
  }

  /// The dock's current share of the central region as a `0..1` fraction — what a divider drag changes and what
  /// the profile persists. Falls back to the default if the tree shape were ever not the expected split.
  pub fn dock_fraction(&self) -> f32 {
    if let Some(Tile::Container(Container::Linear(linear))) = self.tree.tiles.get(self.root) {
      let viewport = linear.shares[self.viewport];
      let dock = linear.shares[self.dock];
      let sum = viewport + dock;
      if sum > f32::EPSILON {
        return dock / sum;
      }
    }
    DEFAULT_DOCK_FRACTION
  }

  /// Render the split into the central region. `state` is the rest of the transient UI state (this split has
  /// been taken OUT of it by the caller, so both can be borrowed); intents land in `sink` as everywhere else.
  #[allow(clippy::too_many_arguments)]
  pub fn ui(&mut self, ui: &mut egui::Ui, view: &ViewState, state: &mut UiState,
    time: super::progress::TimeEstimate, eta_qualifier: Option<EtaQualifier>, sink: &mut IntentSink) {
    let mut behavior = CentralBehavior { view, state, time, eta_qualifier, sink };
    self.tree.ui(&mut behavior, ui);
  }
}

/// The per-frame [`egui_tiles::Behavior`]: renders the two panes with the same view functions as before and
/// locks down everything rearrangeable — this is a fixed split with a draggable divider, not a dockable
/// workspace.
struct CentralBehavior<'a> {
  view: &'a ViewState,
  state: &'a mut UiState,
  time: super::progress::TimeEstimate,
  eta_qualifier: Option<EtaQualifier>,
  sink: &'a mut IntentSink,
}

impl egui_tiles::Behavior<CentralPane> for CentralBehavior<'_> {
  fn pane_ui(&mut self, ui: &mut egui::Ui, _tile_id: TileId, pane: &mut CentralPane) -> UiResponse {
    match pane {
      CentralPane::Viewport => {
        // The toolpath paints its own inset canvas over the whole pane, exactly as it did in the central panel.
        views::toolpath(ui, self.view, self.state);
      }
      CentralPane::Dock => {
        // The dock content (tab strip + console/program body) on its panel surface. The frame reproduces the
        // old bottom panel's fill and margins; inside a share-sized pane its measured size feeds back into
        // nothing, so the fill-height scroll areas are safe by construction.
        egui::Frame::new()
          .fill(self.state.style.palette.panel)
          .inner_margin(egui::Margin::symmetric(8, 2))
          .show(ui, |ui| {
            ui.set_min_size(ui.available_size());
            views::dock(ui, self.view, self.state, self.time, self.eta_qualifier, self.sink);
          });
      }
    }
    UiResponse::None
  }

  /// Required by the trait; never shown — the tree has no `Tabs` container and panes render bare.
  fn tab_title_for_pane(&mut self, pane: &CentralPane) -> egui::WidgetText {
    match pane {
      CentralPane::Viewport => "Viewport".into(),
      CentralPane::Dock => "Console".into(),
    }
  }

  /// The fixed-chrome rule: nothing in this split may ever be picked up and rearranged.
  fn is_tile_draggable(&self, _tiles: &Tiles<CentralPane>, _tile_id: TileId) -> bool {
    false
  }

  /// Neither pane may be dragged below this height — the dock keeps room for its strip + a few log rows + the
  /// MDI line, and the viewport never vanishes.
  fn min_size(&self) -> f32 {
    Metrics::DOCK_MIN_H
  }

  /// The divider between viewport and dock: a hairline gap; the resize grab band around it is what drags.
  fn gap_width(&self, _style: &egui::Style) -> f32 {
    Metrics::DIVIDER
  }

  fn simplification_options(&self) -> SimplificationOptions {
    // The defaults are already right for a bare fixed split (no tab wrapping); pinned explicitly so an upstream
    // default change cannot silently start wrapping panes in tab bars.
    SimplificationOptions { all_panes_must_have_tabs: false, ..Default::default() }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn a_fresh_split_carries_the_requested_dock_fraction() {
    let split = CentralSplit::new(0.3);
    assert!((split.dock_fraction() - 0.3).abs() < 1e-4, "the built tree must encode the requested ratio");
  }

  #[test]
  fn the_persisted_fraction_is_clamped_to_sane_bounds() {
    // A hand-edited profile cannot collapse the dock to nothing or swallow the viewport.
    assert!(CentralSplit::new(0.0).dock_fraction() >= 0.05);
    assert!(CentralSplit::new(1.5).dock_fraction() <= 0.9);
  }

  #[test]
  fn the_split_round_trips_through_debug_without_a_tree_dump() {
    // `Tree` has no `Debug`; the manual impl must render the meaningful state (the ratio) and nothing panics.
    let split = CentralSplit::new(0.25);
    let text = format!("{split:?}");
    assert!(text.contains("dock_fraction"), "Debug should carry the ratio: {text}");
  }
}
