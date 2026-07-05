//! The central canvas/dock split as an [`egui_tiles`] tree, mirroring skirnir's dock host and the reasoning
//! behind it: egui's resizable panels persist the measured CONTENT rect as next frame's panel size, so pane
//! content can feed back into the divider; egui_tiles sizes panes TOP-DOWN from shares, so the split is a
//! ratio the operator owns and nothing the content does can alter it.
//!
//! Deliberately NOT adopted: tiles' drag-and-drop rearranging. The chrome is design-fixed, so
//! [`CentralBehavior`] disables tile dragging and the tree is exactly one vertical split — canvas above,
//! log/G-code dock below.

use eframe::egui;
use egui_tiles::{Container, SimplificationOptions, Tile, TileId, Tiles, Tree, UiResponse};

use super::intent::IntentSink;
use super::metrics::Metrics;
use super::scene::RenderScene;
use super::view_state::ViewState;
use super::views::{self, UiState};

/// The share-fraction bounds a configured value is held to, so a hand-edited config cannot collapse the dock
/// to nothing or swallow the canvas.
const DOCK_FRACTION_RANGE: std::ops::RangeInclusive<f32> = 0.05..=0.9;

/// Which pane of the central split is being rendered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CentralPane {
  /// The 2-D canvas (top).
  Canvas,
  /// The log/G-code dock (bottom).
  Dock,
}

/// The central region's tiles tree plus the ids of its fixed members. Held across frames (the tree carries the
/// operator's split ratio).
pub struct CentralSplit {
  tree: Tree<CentralPane>,
  root: TileId,
  canvas: TileId,
  dock: TileId,
}

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
    let canvas = tiles.insert_pane(CentralPane::Canvas);
    let dock = tiles.insert_pane(CentralPane::Dock);
    let root = tiles.insert_vertical_tile(vec![canvas, dock]);
    if let Some(Tile::Container(Container::Linear(linear))) = tiles.get_mut(root) {
      linear.shares.set_share(canvas, 1.0 - fraction);
      linear.shares.set_share(dock, fraction);
    }
    CentralSplit { tree: Tree::new("central-split", root, tiles), root, canvas, dock }
  }

  /// The dock's current share of the central region as a `0..1` fraction — what a divider drag changes.
  pub fn dock_fraction(&self) -> f32 {
    if let Some(Tile::Container(Container::Linear(linear))) = self.tree.tiles.get(self.root) {
      let canvas = linear.shares[self.canvas];
      let dock = linear.shares[self.dock];
      let sum = canvas + dock;
      if sum > f32::EPSILON {
        return dock / sum;
      }
    }
    0.25
  }

  /// Render the split into the central region.
  pub fn ui(&mut self, ui: &mut egui::Ui, scene: &RenderScene, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
    let mut behavior = CentralBehavior { scene, view, state, sink };
    self.tree.ui(&mut behavior, ui);
  }
}

/// The per-frame [`egui_tiles::Behavior`]: renders the two panes and locks down everything rearrangeable —
/// this is a fixed split with a draggable divider, not a dockable workspace.
struct CentralBehavior<'a> {
  scene: &'a RenderScene,
  view: &'a ViewState,
  state: &'a mut UiState,
  sink: &'a mut IntentSink,
}

impl egui_tiles::Behavior<CentralPane> for CentralBehavior<'_> {
  fn pane_ui(&mut self, ui: &mut egui::Ui, _tile_id: TileId, pane: &mut CentralPane) -> UiResponse {
    match pane {
      CentralPane::Canvas => {
        let rect = ui.available_rect_before_wrap();
        self.state.cursor_world = super::canvas::show(ui, rect, self.scene, self.state, self.view.selected);
      }
      CentralPane::Dock => {
        egui::Frame::new().fill(self.state.style.palette.panel).show(ui, |ui| {
          ui.set_min_size(ui.available_size());
          views::dock(ui, self.view, self.state, self.sink);
        });
      }
    }
    UiResponse::None
  }

  /// Required by the trait; never shown — the tree has no `Tabs` container and panes render bare.
  fn tab_title_for_pane(&mut self, pane: &CentralPane) -> egui::WidgetText {
    match pane {
      CentralPane::Canvas => "Canvas".into(),
      CentralPane::Dock => "Dock".into(),
    }
  }

  /// The fixed-chrome rule: nothing in this split may ever be picked up and rearranged.
  fn is_tile_draggable(&self, _tiles: &Tiles<CentralPane>, _tile_id: TileId) -> bool {
    false
  }

  /// Neither pane may be dragged below this height.
  fn min_size(&self) -> f32 {
    Metrics::DOCK_MIN_H
  }

  /// The divider between canvas and dock: a hairline gap; the resize grab band around it is what drags.
  fn gap_width(&self, _style: &egui::Style) -> f32 {
    Metrics::DIVIDER
  }

  fn simplification_options(&self) -> SimplificationOptions {
    // Pinned explicitly so an upstream default change cannot silently start wrapping panes in tab bars.
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
  fn the_configured_fraction_is_clamped_to_sane_bounds() {
    assert!(CentralSplit::new(0.0).dock_fraction() >= 0.05);
    assert!(CentralSplit::new(1.5).dock_fraction() <= 0.9);
  }

  #[test]
  fn the_split_debugs_without_a_tree_dump() {
    let text = format!("{:?}", CentralSplit::new(0.25));
    assert!(text.contains("dock_fraction"), "Debug should carry the ratio: {text}");
  }
}
