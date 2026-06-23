//! The project's egui UI test harness, built on `egui_kittest`. This is the pattern for reproducing and
//! regression-testing UI bugs that the pure reducer/intent tests cannot reach: anything that depends on real
//! pointer interaction, widget hit-testing, or multi-frame widget state (drags, focus, hover).
//!
//! # Why a per-view closure harness, not `Harness::new_eframe`
//!
//! `egui_kittest` can drive a whole `eframe::App` via `Harness::new_eframe`, but the full [`super::SkirnirApp`]
//! spins up the streaming engine, channels, and a tokio runtime in `new` — heavy and non-deterministic for a
//! widget-level test. The views are free functions that take exactly the state they render
//! (`views::overrides(ui, &view, &mut state, &mut sink)`), so we mount a single view in a closure with a
//! hand-built [`ViewState`]/[`UiState`] and read the emitted [`Intent`]s straight back out. That is lighter,
//! fully deterministic, and exercises the *real* view code path, which is what these tests are for.
//!
//! # How to write a new UI test
//!
//! 1. Build a [`HarnessState`] with the [`ViewState`]/[`UiState`] the view should render (use [`HarnessState::new`]).
//! 2. `egui_kittest::Harness::builder().with_size(..).build_ui_state(render_closure, state)` where the closure
//!    calls the view under test and folds the drained intents into the state (see [`build_overrides_harness`]).
//! 3. Drive interaction with the pointer/keyboard helpers (`drag_at`/`hover_at`/`drop_at`/`key_press`) at
//!    coordinates inside the widget. Hand-painted, label-less widgets cannot be found by AccessKit, so capture
//!    their rect with a test-only side channel (see [`super::views::slider_rect_probe`]) and compute the point.
//! 4. Call `harness.run()` (or `step()`) to apply the queued events, then assert on the recorded intents and
//!    the mutated [`UiState`].
//!
//! Snapshot (image-diff) tests would need the harness's `wgpu` feature and a GPU; these interaction tests use
//! egui's tessellation only and run headless/CI-clean. Snapshots are a later add.

use eframe::egui;
use egui_kittest::Harness;

use super::intent::{Intent, IntentSink};
use super::overrides::OverrideAxis;
use super::view_state::ViewState;
use super::views::{self, UiState, slider_rect_probe};

/// The per-frame test state the harness threads through the view: the engine-derived [`ViewState`], the
/// transient [`UiState`] the view mutates, and the [`Intent`]s the view emitted, accumulated across frames so a
/// test can assert what a drag commanded even though intents are drained each frame.
pub(crate) struct HarnessState {
  /// The engine-derived view state the view renders (connection, status/overrides, …).
  pub view: ViewState,
  /// The transient widget state the view reads and mutates (the slider feedback lives here).
  pub ui: UiState,
  /// Every intent the view has emitted since the harness was built, in order. The view drains its sink each
  /// frame, so we append the drained batch here to survive across frames.
  pub intents: Vec<Intent>,
}

impl HarnessState {
  /// A fresh harness state from a given view/ui pair and no recorded intents.
  pub fn new(view: ViewState, ui: UiState) -> Self {
    HarnessState { view, ui, intents: Vec::new() }
  }
}

/// Build a kittest harness that renders the real [`views::overrides`] panel into a fixed-width window, folding
/// each frame's emitted intents into the [`HarnessState`]. The width is fixed so the slider's allocated rect is
/// deterministic across runs; the slider maps pointer-x linearly across that rect onto the 10–200% span.
pub(crate) fn build_overrides_harness(state: HarnessState) -> Harness<'static, HarnessState> {
  Harness::builder()
    .with_size(egui::vec2(360.0, 320.0))
    .build_ui_state(
      |ui, state: &mut HarnessState| {
        let mut sink = IntentSink::new();
        views::overrides(ui, &state.view, &mut state.ui, &mut sink);
        state.intents.extend(sink.drain());
      },
      state,
    )
}

/// Build a kittest harness that renders the real [`views::dock`] (tab strip + progress readout + body) into a
/// fixed-size window, folding each frame's intents into the [`HarnessState`]. `time` is the elapsed/ETA estimate
/// the progress clock formats; it is fixed so the rendered readout is deterministic across runs. The width is
/// set wide enough that the full readout (count · bar · percent · clock) lays out without the narrow-strip
/// degradation, so a structural assertion can rely on every field being present.
pub(crate) fn build_dock_harness(
  state: HarnessState, time: super::progress::TimeEstimate,
) -> Harness<'static, HarnessState> {
  Harness::builder()
    .with_size(egui::vec2(900.0, 260.0))
    .build_ui_state(
      move |ui, state: &mut HarnessState| {
        let mut sink = IntentSink::new();
        views::dock(ui, &state.view, &mut state.ui, time, &mut sink);
        state.intents.extend(sink.drain());
      },
      state,
    )
}

/// The screen-space point a pointer must be at to drive `axis`'s slider to `target` percent, derived from the
/// slider's recorded rect (the view records it each render via [`slider_rect_probe`]). The slider maps pointer-x
/// linearly across the track onto the `OVERRIDE_MIN..=OVERRIDE_MAX` span, so we invert that mapping. Returns
/// `None` if the slider has not been rendered yet (call `harness.run()` once first).
pub(crate) fn slider_point_for(axis: OverrideAxis, target: u32) -> Option<egui::Pos2> {
  use super::overrides::{OVERRIDE_MAX, OVERRIDE_MIN};
  let rect = slider_rect_probe::last(axis)?;
  let span = (OVERRIDE_MAX - OVERRIDE_MIN) as f32;
  let frac = ((target.clamp(OVERRIDE_MIN, OVERRIDE_MAX) - OVERRIDE_MIN) as f32 / span).clamp(0.0, 1.0);
  Some(egui::pos2(rect.left() + frac * rect.width(), rect.center().y))
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::app::overrides::OverrideFeedback;
  use crate::app::progress::TimeEstimate;
  use crate::app::view_state::Progress;
  use crate::protocol::ConnectionState;
  use crate::protocol::status::{MachineState, PositionKind, RunState, StatusReport};
  use egui_kittest::kittest::Queryable;
  use std::time::Duration;

  /// A minimal connected [`ViewState`] reporting the given `(feed, rapid, spindle)` overrides, so the override
  /// panel is enabled and seeds the sliders from a live `Ov:` value.
  fn view_with_overrides(feed: u32, rapid: u32, spindle: u32) -> ViewState {
    let status = StatusReport {
      machine_state: MachineState { state: RunState::Idle, substate: None },
      position_kind: PositionKind::Machine,
      position: Vec::new(),
      wco: None,
      feed_speed: None,
      overrides: Some((feed, rapid, spindle)),
      pins: Vec::new(),
      buffer: None,
      line: None,
    };
    let mut view = ViewState::default();
    view.connection = ConnectionState::Idle;
    view.status = Some(status);
    view
  }

  /// Drag the spindle slider from centre to a target percent and release. Reproduces the bug report's gesture:
  /// the operator drags the handle and lets go. Asserts (a) a `SetOverride { Spindle, target }` is emitted with
  /// the dragged-to value, and (b) the handle then HOLDS that target while a lagging `Ov:` poll arrives — it
  /// does NOT snap back to the stale live value (the reported snap-to-centre bug).
  #[test]
  fn dragging_the_spindle_slider_commits_the_target_and_holds_it_against_a_lagging_live() {
    let state = HarnessState::new(view_with_overrides(100, 100, 100), UiState::default());
    let mut harness = build_overrides_harness(state);
    // One frame to lay out and record the slider rects before we can target them.
    harness.run();

    // Aim for 150%: compute the pointer x inside the spindle slider's recorded rect.
    let target = 150;
    let point = slider_point_for(OverrideAxis::Spindle, target).expect("the spindle slider must be rendered");

    // Press inside the slider, move to the target while held (this is what egui registers as a drag), release.
    harness.hover_at(point);
    harness.drag_at(point);
    harness.hover_at(point);
    harness.drop_at(point);
    harness.run();

    // (a) The drag committed an absolute SetOverride to the dragged-to value. egui's pointer rounding can land a
    // pixel either side, so accept a small tolerance around the aimed target rather than an exact equality.
    let committed = harness
      .state()
      .intents
      .iter()
      .find_map(|i| match i {
        Intent::SetOverride { axis: OverrideAxis::Spindle, target } => Some(*target),
        _ => None,
      })
      .expect("releasing the spindle drag must emit a SetOverride for the spindle axis");
    assert!(
      committed.abs_diff(target) <= 2,
      "the committed target {committed} should match the dragged-to {target} (±2 for pointer rounding)"
    );

    // The handle must now HOLD the committed target, not the live value.
    let held = match harness.state().ui.spindle_override_drag {
      OverrideFeedback::Holding { target, .. } => target,
      other => panic!("after release the spindle feedback must be Holding the target, was {other:?}"),
    };
    assert_eq!(held, committed, "the held value must be the committed target");

    // (b) Feed a still-lagging live poll (firmware `Ov:` has not caught up): the handle must keep holding the
    // target, not snap back to the stale live value. We re-render with the lagging report and check the state.
    harness.state_mut().view = view_with_overrides(100, 100, 100); // spindle still reports 100%.
    harness.run();
    assert!(
      matches!(harness.state().ui.spindle_override_drag, OverrideFeedback::Holding { .. }),
      "a lagging live poll must NOT release the hold — the handle stays on the commanded target"
    );
  }

  /// Once the firmware's `Ov:` converges onto the committed target, the post-release hold releases and the
  /// handle resumes tracking live. This is the back half of the convergence rule, driven through the real view.
  #[test]
  fn the_spindle_hold_releases_once_the_live_value_converges() {
    let mut ui = UiState::default();
    // Seed the feedback as if a drag to 150 had just been committed from a live of 100.
    ui.spindle_override_drag = OverrideFeedback::Holding { target: 150, committed_from: 100, observations: 0 };
    let state = HarnessState::new(view_with_overrides(100, 100, 150), ui);
    let mut harness = build_overrides_harness(state);
    // Render once with the converged live (spindle now reports 150%): the view's `observe` should clear the hold.
    harness.run();
    assert_eq!(
      harness.state().ui.spindle_override_drag,
      OverrideFeedback::Idle,
      "a converged live value releases the hold so the handle resumes tracking reality"
    );
  }

  /// A streaming [`ViewState`] with a loaded program of `total` lines, `acked` of them acknowledged, so the dock
  /// progress readout renders its count/bar/percent/clock (the readout is shown only while `total > 0`).
  fn view_streaming(acked: usize, total: usize) -> ViewState {
    let mut view = ViewState::default();
    view.connection = ConnectionState::Streaming;
    view.progress = Progress { sent: acked, acked, total };
    view
  }

  /// Regression for the crammed dock progress readout: render the real [`views::dock`] with a loaded, mid-stream
  /// program and a projectable estimate, and assert the readout's fields are present as distinct, separated labels
  /// — the percent (`9%`) and the clock (`0:51 / 9:36`) are separate nodes with a `·` separator between them, not
  /// the run-together `9%0:51` the user reported. Layout pixels are visual, but this proves the fields exist as
  /// separate widgets rather than one fused string, which is the structural half of the fix.
  #[test]
  fn the_dock_progress_readout_renders_separated_fields_not_a_crammed_run_on() {
    // 45 of 500 lines acked ⇒ 9%. Elapsed 51s with a 9:36 projected total reproduces the screenshot's clock.
    let view = view_streaming(45, 500);
    let time = TimeEstimate {
      elapsed: Duration::from_secs(51),
      remaining: Some(Duration::from_secs(525)),
      total: Some(Duration::from_secs(576)),
    };
    let state = HarnessState::new(view, UiState::default());
    let mut harness = build_dock_harness(state, time);
    harness.run();

    // Every field is its own label node: the count, the percent, the elapsed/total clock, and at least one `·`
    // separator between them. If the percent and clock had run together there would be no standalone `9%` node.
    assert!(harness.query_by_label("45 / 500").is_some(), "the acked/total count must render as its own label");
    assert!(harness.query_by_label("9%").is_some(), "the percent must render as its own label, not fused to the clock");
    assert!(harness.query_by_label("0:51 / 9:36").is_some(), "the elapsed/total clock must render as its own label");
    // Three `·` separators sit between the four fields (count · bar · percent · clock) when the bar is shown.
    assert!(
      harness.query_all_by_label("·").count() >= 2,
      "`·` separators must sit between the readout's fields so they never run together"
    );
  }

  /// The override panel is disabled while disconnected: a slider drag commands nothing, since a relative
  /// override byte is a no-op with no live link. Drives the gesture against a disconnected view and asserts no
  /// `SetOverride` escapes.
  #[test]
  fn the_override_sliders_are_inert_while_disconnected() {
    let mut view = ViewState::default();
    view.connection = ConnectionState::Disconnected;
    view.status = None;
    let state = HarnessState::new(view, UiState::default());
    let mut harness = build_overrides_harness(state);
    harness.run();

    // The disabled slider still occupies a rect; driving a pointer over it must not emit an override.
    if let Some(point) = slider_point_for(OverrideAxis::Spindle, 150) {
      harness.hover_at(point);
      harness.drag_at(point);
      harness.hover_at(point);
      harness.drop_at(point);
      harness.run();
    }
    assert!(
      !harness
        .state()
        .intents
        .iter()
        .any(|i| matches!(i, Intent::SetOverride { .. })),
      "a disabled override panel must not emit SetOverride from a pointer drag"
    );
  }
}
