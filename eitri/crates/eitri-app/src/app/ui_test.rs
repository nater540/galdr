//! The egui_kittest interaction harness: drives the REAL shell layout (`shell::shell_panels`, the same function
//! the live window renders) headlessly with fixture state, simulating pointer input and asserting on the
//! intents the views emit. No GPU needed — these run in a plain `cargo test`.
//!
//! Global i18n note: every harness test holds [`crate::i18n::lock_global_for_test`] (the views `tr!` against
//! the ONE global registry), selects `en-US`, and restores it on drop, so locale-dependent label queries cannot
//! race across threads.

use std::sync::MutexGuard;

use eframe::egui;

use super::dock_tiles::CentralSplit;
use super::intent::Intent;
use super::scene::RenderScene;
use super::shell;
use super::view_state::{OpView, Selection, TreeRow, ViewState};
use super::views::UiState;
use eitri_project::{ObjectId, ObjectKind};

/// The default harness window: the config's default 1280×800.
pub(crate) const DEFAULT_SIZE: egui::Vec2 = egui::vec2(1280.0, 800.0);

/// The held global-i18n guard + the locale reset on drop.
pub(crate) struct LocaleGuard(#[allow(dead_code)] MutexGuard<'static, ()>);

impl Drop for LocaleGuard {
  fn drop(&mut self) {
    crate::i18n::set_language(crate::i18n::EN_US);
  }
}

/// Take the shared global-i18n test guard and select `locale` on the global registry.
pub(crate) fn render_in(locale: &str) -> LocaleGuard {
  let guard = crate::i18n::lock_global_for_test();
  let _ = crate::i18n::init();
  crate::i18n::set_language(locale);
  LocaleGuard(guard)
}

/// Everything the shell layout needs, owned by the harness across frames.
pub(crate) struct HarnessState {
  /// The pure view state (tree, log, op).
  pub view: ViewState,
  /// The transient widget state.
  pub ui: UiState,
  /// The paint-ready scene.
  pub scene: RenderScene,
  /// The central split.
  pub split: CentralSplit,
  /// Every intent the views emitted, across frames.
  pub intents: Vec<Intent>,
}

impl HarnessState {
  /// Fixture state around a view/ui pair, with an empty scene and the default split.
  pub fn new(view: ViewState, ui: UiState) -> Self {
    HarnessState { view, ui, scene: RenderScene::default(), split: CentralSplit::new(0.25), intents: Vec::new() }
  }
}

/// Build a harness rendering the real `shell_panels` with the app's fonts and theme applied, so layout and
/// hit-testing match the live window.
pub(crate) fn build_shell_harness(state: HarnessState, size: egui::Vec2) -> egui_kittest::Harness<'static, HarnessState> {
  let palette = state.ui.style.palette;
  let font_scale = 1.0;
  let harness = egui_kittest::Harness::builder().with_size(size).build_ui_state(
    |ui, state: &mut HarnessState| {
      let mut sink = super::intent::IntentSink::new();
      let HarnessState { view, ui: ui_state, scene, split, intents } = state;
      shell::shell_panels(ui, scene, view, ui_state, split, &mut sink);
      intents.extend(sink.drain());
    },
    state,
  );
  super::fonts::install(&harness.ctx);
  shell::apply_theme(&harness.ctx, &palette, font_scale);
  harness
}

#[cfg(test)]
mod tests {
  use super::*;
  use egui_kittest::kittest::Queryable;

  /// A three-object fixture tree: a Gerber, its drills, and a job — obviously fake names.
  fn fixture_view() -> ViewState {
    let mut view = ViewState::default();
    view.set_tree(
      vec![
        TreeRow { id: ObjectId(1), name: "fixture-top".into(), kind: ObjectKind::Gerber, visible: true, stale: false },
        TreeRow {
          id: ObjectId(2),
          name: "fixture-drills".into(),
          kind: ObjectKind::Excellon,
          visible: true,
          stale: false,
        },
      ],
      vec![TreeRow {
        id: ObjectId(3),
        name: "fixture-top-isolation".to_string(),
        kind: ObjectKind::CncJob,
        visible: true,
        stale: false,
      }],
      true,
      false,
    );
    view
  }

  #[test]
  fn clicking_a_tree_row_emits_a_select_intent() {
    let _locale = render_in(crate::i18n::EN_US);
    let state = HarnessState::new(fixture_view(), UiState::default());
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);
    // Exact label: the row is "fixture-drills"; its eye toggle is "Hide fixture-drills" (a separate control).
    harness.get_by_label("fixture-drills").click();
    harness.run();
    assert!(
      harness.state().intents.contains(&Intent::Select(Some(Selection::Object(ObjectId(2))))),
      "clicking a row must select it: {:?}",
      harness.state().intents,
    );
  }

  #[test]
  fn clicking_the_selected_row_emits_a_deselect() {
    let _locale = render_in(crate::i18n::EN_US);
    let mut view = fixture_view();
    view.selected = Some(Selection::Object(ObjectId(2)));
    let state = HarnessState::new(view, UiState::default());
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);
    // The selected object's name now ALSO heads the parameter panel, so the query must pick the tree's
    // clickable row (role Button), not the panel's static text.
    {
      use egui::accesskit::Role;
      use egui_kittest::kittest::By;
      let row = harness
        .query_all(By::new().role(Role::Button).label_contains("fixture-drills"))
        .next()
        .expect("the tree row is a clickable button");
      row.click();
    }
    harness.run();
    assert!(
      harness.state().intents.contains(&Intent::Select(None)),
      "clicking the selected row must clear the selection: {:?}",
      harness.state().intents,
    );
  }

  #[test]
  fn the_empty_tree_shows_the_get_started_hint() {
    let _locale = render_in(crate::i18n::EN_US);
    let state = HarnessState::new(ViewState::default(), UiState::default());
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);
    assert!(harness.query_by_label("No objects yet.").is_some(), "the empty tree must invite, not sit blank");
    assert!(
      harness.query_by_label("Select an object to edit its parameters.").is_some(),
      "the empty parameter panel explains itself"
    );
  }

  #[test]
  fn the_isolate_button_runs_when_idle_and_is_inert_while_busy() {
    let _locale = render_in(crate::i18n::EN_US);
    // Idle, Gerber selected: the Run button emits the isolate intent.
    let mut view = fixture_view();
    view.selected = Some(Selection::Object(ObjectId(1)));
    let state = HarnessState::new(view.clone(), UiState::default());
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);
    harness.get_by_label("Isolate").click();
    harness.run();
    assert!(
      harness.state().intents.contains(&Intent::RunIsolate(ObjectId(1))),
      "idle: Run must emit the isolate intent: {:?}",
      harness.state().intents,
    );

    // Busy: the same button must be DISABLED — a click emits nothing (the session is away on the worker).
    view.op = OpView::Running { label: "Isolation routing".to_string(), done: 1, total: 4 };
    let state = HarnessState::new(view, UiState::default());
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);
    harness.get_by_label("Isolate").click();
    harness.run();
    assert!(
      !harness.state().intents.iter().any(|i| matches!(i, Intent::RunIsolate(_))),
      "busy: the Run button must be disabled, not queue a second op: {:?}",
      harness.state().intents,
    );
    // And the busy explanation is on screen, so the disabled state is never silent.
    assert!(
      harness.query_by_label_contains("An operation is running").is_some(),
      "the parameter panel must say WHY it is locked"
    );
  }

  #[test]
  fn the_cancel_button_appears_while_running_and_emits_cancel() {
    let _locale = render_in(crate::i18n::EN_US);
    let mut view = fixture_view();
    view.op = OpView::Running { label: "Isolation routing".to_string(), done: 0, total: 0 };
    let state = HarnessState::new(view, UiState::default());
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);
    harness.get_by_label("Cancel").click();
    // `run_steps`, not `run`: the indeterminate progress bar animates (it requests repaints every frame by
    // design), so `run()`'s wait-for-quiescence would spin out its step budget.
    harness.run_steps(2);
    assert!(
      harness.state().intents.contains(&Intent::CancelOp),
      "the dock's Cancel must emit the cancel intent: {:?}",
      harness.state().intents,
    );
  }

  #[test]
  fn switching_dock_tabs_flips_the_view_state() {
    let _locale = render_in(crate::i18n::EN_US);
    let state = HarnessState::new(fixture_view(), UiState::default());
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);
    assert_eq!(harness.state().ui.dock_tab, super::super::views::DockTab::Log);
    harness.get_by_label("G-code").click();
    harness.run();
    assert_eq!(harness.state().ui.dock_tab, super::super::views::DockTab::Gcode, "clicking the tab switches it");
  }

  #[test]
  fn undo_is_disabled_when_the_history_is_empty_and_enabled_when_not() {
    let _locale = render_in(crate::i18n::EN_US);
    // Empty history: Undo must emit nothing.
    let state = HarnessState::new(ViewState::default(), UiState::default());
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);
    harness.get_by_label("Undo").click();
    harness.run();
    assert!(
      !harness.state().intents.contains(&Intent::Undo),
      "an empty history must disable Undo: {:?}",
      harness.state().intents,
    );

    // With history: it emits.
    let state = HarnessState::new(fixture_view(), UiState::default());
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);
    harness.get_by_label("Undo").click();
    harness.run();
    assert!(harness.state().intents.contains(&Intent::Undo), "an undoable history enables Undo");
  }

  #[test]
  fn clicking_a_rows_eye_toggles_visibility_without_selecting() {
    let _locale = render_in(crate::i18n::EN_US);
    let mut view = fixture_view();
    view.toolpaths[0].visible = false; // the job starts hidden, so both eye directions are on screen.
    let state = HarnessState::new(view, UiState::default());
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);

    harness.get_by_label("Hide fixture-drills").click();
    harness.run();
    assert!(
      harness.state().intents.contains(&Intent::SetVisible(ObjectId(2), false)),
      "the eye on a visible row must emit a hide: {:?}",
      harness.state().intents,
    );
    assert!(
      !harness.state().intents.iter().any(|i| matches!(i, Intent::Select(Some(Selection::Object(ObjectId(2)))))),
      "the eye click must not ALSO select the row: {:?}",
      harness.state().intents,
    );

    harness.get_by_label("Show fixture-top-isolation").click();
    harness.run();
    assert!(
      harness.state().intents.contains(&Intent::SetVisible(ObjectId(3), true)),
      "the eye on a hidden row must emit a show: {:?}",
      harness.state().intents,
    );
  }

  #[test]
  fn the_status_bar_counts_sources_and_toolpaths_together() {
    let _locale = render_in(crate::i18n::EN_US);
    let state = HarnessState::new(fixture_view(), UiState::default());
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);
    // The fixture has 2 sources (PROJECT) + 1 job (TOOLPATHS); the count must be 3, not the tree-only 2.
    harness.get_by_label("3 objects");
  }

  #[test]
  fn clicking_a_toolpaths_rebuild_button_emits_a_rebuild_intent() {
    let _locale = render_in(crate::i18n::EN_US);
    let state = HarnessState::new(fixture_view(), UiState::default());
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);
    // The job row (id 3) in the TOOLPATHS panel carries a ⟳ rebuild button; only jobs do.
    harness.get_by_label("Rebuild fixture-top-isolation").click();
    harness.run();
    assert!(
      harness.state().intents.contains(&Intent::RebuildJob(ObjectId(3))),
      "the rebuild button must emit a rebuild for its job: {:?}",
      harness.state().intents,
    );
  }

  #[test]
  fn the_eye_is_inert_while_an_operation_runs() {
    let _locale = render_in(crate::i18n::EN_US);
    let mut view = fixture_view();
    view.op = OpView::Running { label: "Isolation routing".to_string(), done: 1, total: 4 };
    let state = HarnessState::new(view, UiState::default());
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);
    harness.get_by_label("Hide fixture-drills").click();
    harness.run_steps(2);
    assert!(
      !harness.state().intents.iter().any(|i| matches!(i, Intent::SetVisible(..))),
      "the session is away — a visibility edit must not be queued: {:?}",
      harness.state().intents,
    );
  }

  #[test]
  fn clicking_the_canvas_selects_the_object_under_the_pointer() {
    use super::super::scene::{ObjectScene, RenderScene};
    let _locale = render_in(crate::i18n::EN_US);
    // A synthetic filled square centred on the default view centre (world [40, 30]), so a click in the middle
    // of the canvas node lands inside it.
    let square = ObjectScene {
      id: ObjectId(1),
      kind: ObjectKind::Gerber,
      visible: true,
      fill: eitri_geo::TriangleMesh {
        vertices: vec![[35.0, 25.0], [45.0, 25.0], [45.0, 35.0], [35.0, 35.0]],
        indices: vec![0, 1, 2, 0, 2, 3],
      },
      outlines: vec![vec![[35.0, 25.0], [45.0, 25.0], [45.0, 35.0], [35.0, 35.0], [35.0, 25.0]]],
      polylines: Vec::new(),
      cuts: Vec::new(),
      rapids: Vec::new(),
      bounds: Some((35.0, 25.0, 45.0, 35.0)),
    };
    let mut state = HarnessState::new(fixture_view(), UiState::default());
    state.scene = RenderScene { objects: vec![square], bounds: Some((35.0, 25.0, 45.0, 35.0)) };
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);
    harness.get_by_label("Canvas").click();
    harness.run();
    assert!(
      harness.state().intents.contains(&Intent::Select(Some(Selection::Object(ObjectId(1))))),
      "a canvas click inside the square must select it: {:?}",
      harness.state().intents,
    );
  }

  #[test]
  fn clicking_empty_canvas_clears_an_existing_selection() {
    let _locale = render_in(crate::i18n::EN_US);
    let mut view = fixture_view();
    view.selected = Some(Selection::Object(ObjectId(1)));
    let state = HarnessState::new(view, UiState::default()); // the scene is empty: every click misses.
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);
    harness.get_by_label("Canvas").click();
    harness.run();
    assert!(
      harness.state().intents.contains(&Intent::Select(None)),
      "an empty-canvas click must clear the selection: {:?}",
      harness.state().intents,
    );
  }

  #[test]
  fn clicking_the_pinned_setup_row_selects_the_setup_node() {
    let _locale = render_in(crate::i18n::EN_US);
    let state = HarnessState::new(fixture_view(), UiState::default());
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);
    harness.get_by_label("Setup").click();
    harness.run();
    assert!(
      harness.state().intents.contains(&Intent::Select(Some(Selection::Setup))),
      "the pinned row must select the Setup node: {:?}",
      harness.state().intents,
    );
  }

  #[test]
  fn the_setup_row_is_present_and_clickable_even_on_an_empty_project() {
    // The whole point of the node: it exists BEFORE anything is loaded, so the operator can set up stock first.
    let _locale = render_in(crate::i18n::EN_US);
    let state = HarnessState::new(ViewState::default(), UiState::default());
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);
    harness.get_by_label("Setup").click();
    harness.run();
    assert!(
      harness.state().intents.contains(&Intent::Select(Some(Selection::Setup))),
      "an empty tree still pins the Setup row: {:?}",
      harness.state().intents,
    );
  }

  #[test]
  fn the_setup_panels_datum_grid_commits_a_stock_with_the_clicked_corner() {
    let _locale = render_in(crate::i18n::EN_US);
    let mut view = fixture_view();
    view.selected = Some(Selection::Setup);
    let state = HarnessState::new(view, UiState::default());
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);
    harness.get_by_label("Bottom right").click();
    harness.run();
    let committed = harness.state().intents.iter().find_map(|i| match i {
      Intent::SetStock { stock, .. } => Some(*stock),
      _ => None,
    });
    let stock = committed.expect("clicking a corner dot must commit the drafted stock");
    assert_eq!(stock.datum, eitri_project::DatumCorner::BottomRight, "the pick lands in the committed stock");
    assert!(stock.size_x > 0.0 && stock.thickness > 0.0, "the drafted block is a real material size");
  }

  #[test]
  fn the_fit_button_emits_fit_stock_against_the_reference_object() {
    let _locale = render_in(crate::i18n::EN_US);
    let mut view = fixture_view();
    view.selected = Some(Selection::Setup);
    let state = HarnessState::new(view, UiState::default());
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);
    harness.get_by_label("Fit to board").click();
    harness.run();
    // The default reference is the first geometry-bearing row (the fixture Gerber, id 1) at the drafted 1.6 mm.
    assert!(
      harness
        .state()
        .intents
        .iter()
        .any(|i| matches!(i, Intent::FitStock { reference: ObjectId(1), thickness } if (thickness - 1.6).abs() < 1e-9)),
      "Fit must target the first board at the drafted thickness: {:?}",
      harness.state().intents,
    );
  }

  #[test]
  fn the_native_button_clears_the_stock() {
    let _locale = render_in(crate::i18n::EN_US);
    let mut view = fixture_view();
    view.selected = Some(Selection::Setup);
    // A committed stock makes the Native button meaningful (it is disabled in the native frame).
    let ui = UiState {
      stock: Some(super::super::views::StockDraft::default().to_stock()),
      ..UiState::default()
    };
    let state = HarnessState::new(view, ui);
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);
    harness.get_by_label("Native (no stock)").click();
    harness.run();
    assert!(
      harness.state().intents.contains(&Intent::ClearStock),
      "the Native button must revert to the source frame: {:?}",
      harness.state().intents,
    );
  }

  #[test]
  fn the_setup_controls_are_inert_while_an_operation_runs() {
    let _locale = render_in(crate::i18n::EN_US);
    let mut view = fixture_view();
    view.selected = Some(Selection::Setup);
    view.op = OpView::Running { label: "Isolation routing".to_string(), done: 1, total: 4 };
    let ui = UiState {
      stock: Some(super::super::views::StockDraft::default().to_stock()),
      ..UiState::default()
    };
    let state = HarnessState::new(view, ui);
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);
    harness.get_by_label("Bottom left").click();
    harness.run_steps(2);
    harness.get_by_label("Native (no stock)").click();
    harness.run_steps(2);
    harness.get_by_label("Fit to board").click();
    harness.run_steps(2);
    assert!(
      !harness
        .state()
        .intents
        .iter()
        .any(|i| matches!(i, Intent::SetStock { .. } | Intent::ClearStock | Intent::FitStock { .. })),
      "the session is away — setup edits must not be queued: {:?}",
      harness.state().intents,
    );
  }

  #[test]
  fn the_settings_gear_opens_the_dialog_intent() {
    let _locale = render_in(crate::i18n::EN_US);
    let state = HarnessState::new(ViewState::default(), UiState::default());
    let mut harness = build_shell_harness(state, DEFAULT_SIZE);
    harness.run_steps(2);
    harness.get_by_label("⚙").click();
    harness.run();
    assert!(harness.state().intents.contains(&Intent::OpenAppSettings));
  }
}
