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
use super::view_state::{OpView, TreeRow, ViewState};
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
        TreeRow { id: ObjectId(1), name: "fixture-top".to_string(), kind: ObjectKind::Gerber },
        TreeRow { id: ObjectId(2), name: "fixture-drills".to_string(), kind: ObjectKind::Excellon },
        TreeRow { id: ObjectId(3), name: "fixture-top-isolation".to_string(), kind: ObjectKind::CncJob },
      ],
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
    harness.get_by_label_contains("fixture-drills").click();
    harness.run();
    assert!(
      harness.state().intents.contains(&Intent::Select(Some(ObjectId(2)))),
      "clicking a row must select it: {:?}",
      harness.state().intents,
    );
  }

  #[test]
  fn clicking_the_selected_row_emits_a_deselect() {
    let _locale = render_in(crate::i18n::EN_US);
    let mut view = fixture_view();
    view.selected = Some(ObjectId(2));
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
    view.selected = Some(ObjectId(1));
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
