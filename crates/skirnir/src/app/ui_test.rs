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
        views::dock(ui, &state.view, &mut state.ui, time, None, &mut sink);
        state.intents.extend(sink.drain());
      },
      state,
    )
}

/// Build a kittest harness that hosts [`views::dock`] EXACTLY as the real app does: inside a bottom
/// `Panel::bottom("dock").exact_size(dock_height)` of a window the size of the real layout, so the dock body is
/// height-constrained to the pinned 200px just like in `SkirnirApp::update`. This is the faithful reproduction
/// path: `build_dock_harness` renders `dock()` into an unconstrained root `Ui`, which masks any
/// fill-the-remaining-height clipping of the input row beneath the log scroll area. Use THIS harness to assert
/// what actually reaches the screen on the Console tab. `time` is fixed for a deterministic progress readout.
pub(crate) fn build_docked_panel_harness(
  state: HarnessState, time: super::progress::TimeEstimate,
) -> Harness<'static, HarnessState> {
  use crate::app::metrics::Metrics;
  Harness::builder()
    .with_size(egui::vec2(900.0, 600.0))
    .build_ui_state(
      move |ui, state: &mut HarnessState| {
        let mut sink = IntentSink::new();
        // Mirror the shell: a status bar then the REAL dock panel (`views::dock_panel`, the same code the app
        // runs), so the dock body sees exactly the height policy the real app gives it.
        egui::Panel::bottom("status").exact_size(Metrics::STATUS_BAR_H).show_inside(ui, |_ui| {});
        views::dock_panel(ui, &state.view, &mut state.ui, time, None, &mut sink);
        state.intents.extend(sink.drain());
      },
      state,
    )
}

/// Lay out the FULL application shell exactly as [`super::SkirnirApp`]'s frame does, driven by a
/// [`HarnessState`] instead of the live app (no engine, no runtime, no I/O). A THIN wrapper over
/// [`views::shell_panels`] — the very function the shell renders — with fixture panel data, so the harness IS
/// the real layout by construction and cannot drift from the app (the earlier hand-copied mirror silently lost
/// the tool-change banner branch, exactly the failure mode this closes).
pub(crate) fn shell_layout(ui: &mut egui::Ui, state: &mut HarnessState, time: super::progress::TimeEstimate) {
  let mut sink = IntentSink::new();
  views::shell_panels(ui, &state.view, &mut state.ui, views::ShellPanelsData::bare(time), &mut sink);
  state.intents.extend(sink.drain());
}

/// Seed the bundled locales into the global registry for a harness render WITHOUT flipping an already-selected
/// locale. `init` resets the language to en-US, which would both clobber a locale a sv-SE snapshot just pinned AND,
/// running unguarded on every call, race the i18n module's global-locale tests — so we init ONLY when the registry
/// is still empty (the first harness of the run). Errors are ignored — a bundled parse failure already fails the
/// i18n unit tests. Callers that must render a SPECIFIC locale set it themselves under the shared test guard.
pub(crate) fn ensure_locales_seeded() {
  if crate::i18n::languages().is_empty() || crate::i18n::get_language().is_empty() {
    let _ = crate::i18n::init();
  }
}

/// Build a kittest harness that renders the full [`shell_layout`] at the given window size, folding each frame's
/// intents into the [`HarnessState`]. The bundled locales are initialised (idempotently) so the `tr!` labels
/// render as real strings, and the app's fonts + theme are applied to the context so the harness paints exactly
/// what the window would. Used by the snapshot suite and available to whole-window interaction tests.
pub(crate) fn build_shell_harness(
  state: HarnessState, size: egui::Vec2, time: super::progress::TimeEstimate,
) -> Harness<'static, HarnessState> {
  // The toolbar labels go through `tr!`; seed the global registry so they render as words, not raw keys.
  ensure_locales_seeded();
  let palette = state.ui.style.palette;
  let harness = Harness::builder().with_size(size).build_ui_state(
    move |ui, state: &mut HarnessState| shell_layout(ui, state, time),
    state,
  );
  super::fonts::install(&harness.ctx);
  super::shell::apply_theme(&harness.ctx, &palette, 1.0);
  harness
}

/// Build a kittest harness that renders the real [`views::jog`] pad at the app's exact left-column width, so the
/// XY grid, the Z/A columns, and the step selector lay out with the geometry the real window gives them.
pub(crate) fn build_jog_harness(state: HarnessState) -> Harness<'static, HarnessState> {
  use crate::app::metrics::Metrics;
  Harness::builder()
    .with_size(egui::vec2(Metrics::LEFT_COL_W, 320.0))
    .build_ui_state(
      |ui, state: &mut HarnessState| {
        let mut sink = IntentSink::new();
        views::jog(ui, &state.view, &mut state.ui, &mut sink);
        state.intents.extend(sink.drain());
      },
      state,
    )
}

/// Build a kittest harness that renders the real [`views::dro`] at the app's exact left-column width.
pub(crate) fn build_dro_harness(state: HarnessState) -> Harness<'static, HarnessState> {
  use crate::app::metrics::Metrics;
  Harness::builder()
    .with_size(egui::vec2(Metrics::LEFT_COL_W, 420.0))
    .build_ui_state(
      |ui, state: &mut HarnessState| {
        let mut sink = IntentSink::new();
        views::dro(ui, &state.view, &mut state.ui, &mut sink);
        state.intents.extend(sink.drain());
      },
      state,
    )
}

/// Build a kittest harness that renders the app settings dialog BODY ([`crate::app::app_settings::body`]) with a
/// given loaded config (dirty, so the Save button is enabled), folding each frame's intents into the state. The
/// bundled locales are initialised so the `tr!` labels render as real strings the tests can query by.
pub(crate) fn build_app_settings_harness(
  state: HarnessState, config: crate::config::Config,
) -> Harness<'static, HarnessState> {
  ensure_locales_seeded();
  Harness::builder()
    .with_size(egui::vec2(440.0, 600.0))
    .build_ui_state(
      move |ui, state: &mut HarnessState| {
        let mut sink = IntentSink::new();
        crate::app::app_settings::body(ui, &mut state.ui, &config, true, &mut sink);
        state.intents.extend(sink.drain());
      },
      state,
    )
}

/// Build a kittest harness that renders just the [`views::transport_group`] (the Run/Hold/Stop segmented group plus
/// the standalone Abort control) into a fixed-size window, folding each frame's intents into the [`HarnessState`].
/// Isolating the group keeps the assertions about which intent each button fires independent of the rest of the
/// toolbar (the port combo, file dialog, etc.).
pub(crate) fn build_transport_group_harness(state: HarnessState) -> Harness<'static, HarnessState> {
  Harness::builder()
    .with_size(egui::vec2(420.0, 80.0))
    .build_ui_state(
      |ui, state: &mut HarnessState| {
        ui.horizontal(|ui| {
          let mut sink = IntentSink::new();
          views::transport_group(ui, &state.view, &state.ui, false, &mut sink);
          state.intents.extend(sink.drain());
        });
      },
      state,
    )
}

/// Build a kittest harness that renders just the [`views::toolpath`] viewport (with no program loaded, so it paints
/// the inset background fill + the reference grid from the active palette) into a fixed-size window. Used to prove the
/// views render the configured palette rather than the baked-in design constants: the test seeds a sentinel palette
/// and then scans the tessellated mesh for its colours.
pub(crate) fn build_toolpath_harness(state: HarnessState) -> Harness<'static, HarnessState> {
  Harness::builder()
    .with_size(egui::vec2(360.0, 240.0))
    .build_ui_state(
      |ui, state: &mut HarnessState| {
        // The toolpath view draws no intents; it only paints. We still drain to keep the closure shape uniform.
        views::toolpath(ui, &state.view, &mut state.ui);
      },
      state,
    )
}

/// Every vertex colour in the last frame's tessellated meshes, so a test can assert a palette colour was actually
/// painted. We tessellate the captured shapes (the harness runs glow-free, so there is no GPU image to read) and
/// collect the mesh vertex colours — a hand-painted `rect_filled`/`line_segment` lands here as `Color32` vertices.
pub(crate) fn painted_vertex_colors(harness: &Harness<'static, HarnessState>) -> Vec<egui::Color32> {
  let shapes = harness.output().shapes.clone();
  let pixels_per_point = harness.ctx.pixels_per_point();
  let primitives = harness.ctx.tessellate(shapes, pixels_per_point);
  let mut colors = Vec::new();
  for primitive in primitives {
    if let egui::epaint::Primitive::Mesh(mesh) = primitive.primitive {
      colors.extend(mesh.vertices.iter().map(|v| v.color));
    }
  }
  colors
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
  use crate::app::metrics::Metrics;
  use crate::app::overrides::OverrideFeedback;
  use crate::app::progress::TimeEstimate;
  use crate::app::view_state::Progress;
  use crate::engine::Event;
  use crate::protocol::status::{MachineState, PositionKind, RunState, StatusReport};
  use crate::protocol::{ConnectionState, Response};
  use egui_kittest::kittest::Queryable;
  use std::time::Duration;

  /// A minimal connected [`ViewState`] reporting the given `(feed, rapid, spindle)` overrides, so the override
  /// panel is enabled and seeds the sliders from a live `Ov:` value.
  fn view_with_overrides(feed: u32, rapid: u32, spindle: u32) -> ViewState {
    let mut view = ViewState::default();
    view.connection = ConnectionState::Idle;
    // Feed the `Ov:` through the reducer exactly as the engine does, so the intermittent-override cache
    // (`last_overrides`, read by `view.overrides()`) is seeded — the panel reads the cache, not the raw report.
    view.apply(Event::Response(Response::Status(format!("Idle|MPos:0,0,0|Ov:{feed},{rapid},{spindle}"))));
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

  /// The exact label the Program listing renders for `index` of `program` — `{:>5}  {line}` (1-based number,
  /// two spaces, the source line). Mirrors `program_body`'s row format so a test can look the row up by label.
  fn program_row_label(program: &[String], index: usize) -> String {
    format!("{:>5}  {}", index + 1, program[index])
  }

  /// The Program tab auto-scrolls to keep the executing line in view as the stream advances. This drives the real
  /// [`views::dock`] Program body — whose rows are virtualised via `show_rows`, so only on-screen rows exist as
  /// widgets — and asserts the *executing* row is laid out (hence visible) after the cursor jumps deep into a
  /// long file while already scrolled. That is exactly the path the pure `program_follow_target` test cannot
  /// reach: it exercises the `scroll_to_rect` coordinate math, which must recover row 0's origin from the visible
  /// slice (`range.start`) and step by the full row pitch (`row_height + item_spacing.y`). With the earlier
  /// row-0-assuming math the target landed hundreds of pixels off once scrolled, leaving the executing row off
  /// screen — so this regresses that bug.
  #[test]
  fn the_program_tab_keeps_the_executing_line_in_view_after_a_scrolled_advance() {
    let program: Vec<String> = (0..500).map(|n| format!("G1 X{n}")).collect();
    let total = program.len();
    let mut ui = UiState::default();
    ui.active_tab = views::DockTab::Program;
    ui.set_program(program.clone(), None);
    // The dock body virtualises (only a slice of the 500 rows is ever laid out) yet is small relative to the file,
    // so the executing line genuinely scrolls — `build_dock_harness` uses a 900×260 window.
    let time = TimeEstimate { elapsed: Duration::from_secs(0), remaining: None, total: None };

    // First settle the view on a line a few hundred rows down, so the listing is scrolled well away from the top
    // (`range.start > 0`) — the state in which the old row-0-assuming math went wrong.
    let state = HarnessState::new(view_streaming(250, total), ui);
    let mut harness = build_dock_harness(state, time);
    harness.run();
    // Sanity: it actually scrolled — row 0 is virtualised away, not laid out, so its label is absent.
    assert!(
      harness.query_by_label(&program_row_label(&program, 0)).is_none(),
      "the listing must have scrolled away from the top, virtualising row 0 out of the widget tree"
    );

    // Advance the cursor deeper into the file while already scrolled, then re-render. The follow must re-target
    // the new executing row and bring it into view.
    harness.state_mut().view = view_streaming(460, total);
    harness.run();
    assert!(
      harness.query_by_label(&program_row_label(&program, 460)).is_some(),
      "after a scrolled advance the executing row (461: `G1 X460`) must be scrolled into view, not left off screen"
    );
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

  /// A connected, actively-running [`ViewState`] (`Streaming` + `<Run>`), so the transport group's Stop and Abort
  /// controls are both enabled and clickable.
  fn view_running() -> ViewState {
    let status = StatusReport {
      machine_state: MachineState { state: RunState::Run, substate: None },
      position_kind: PositionKind::Machine,
      position: Vec::new(),
      wco: None,
      feed_speed: None,
      overrides: None,
      pins: Vec::new(),
      buffer: None,
      line: None,
    };
    let mut view = ViewState::default();
    view.connection = ConnectionState::Streaming;
    view.status = Some(status);
    view
  }

  #[test]
  fn the_jog_pad_offers_a_rotary_a_jog_that_emits_an_a_axis_step_jog() {
    // DOC-10: the rotary A axis jogs like Z — a fixed-step `$J=` move — via dedicated A+/A− controls beside the
    // Z column. Clicking each must emit a step `Intent::Jog` on `Axis::A` in the matching direction, carrying the
    // selected step (degrees, by the degrees-as-mm convention) and feed.
    use crate::app::intent::{Axis, Dir};
    let mut ui = UiState::default();
    ui.jog_step = 5.0;
    ui.jog_feed = 400.0;
    // A rotary (4-field) report is required for the A column to render at all (the 3-axis gate); an Idle rotary view
    // both shows it and leaves it enabled.
    let state = HarnessState::new(view_idle_rotary(), ui);
    let mut harness = build_jog_harness(state);
    harness.run();

    harness.get_by_label("A+").click();
    harness.run();
    harness.get_by_label("A−").click();
    harness.run();

    let jogs: Vec<(Axis, Dir)> = harness
      .state()
      .intents
      .iter()
      .filter_map(|i| match i {
        Intent::Jog { axis: Axis::A, dir, distance, feed } => {
          assert_eq!(*distance, 5.0, "the A jog carries the selected step");
          assert_eq!(*feed, 400.0, "the A jog carries the selected feed");
          Some((Axis::A, *dir))
        }
        _ => None,
      })
      .collect();
    assert_eq!(
      jogs,
      vec![(Axis::A, Dir::Pos), (Axis::A, Dir::Neg)],
      "A+ then A− must emit one A-axis step jog each, in order: {:?}",
      harness.state().intents
    );
  }

  #[test]
  fn the_rotary_a_jog_is_disabled_outside_idle_and_jog() {
    // The same `$J=` gate as every other jog control: with a rotary board in Alarm the A buttons render (a 4-field
    // report is present) but DISABLED, and a click commands nothing — never offer a control the firmware is
    // guaranteed to reject (and that would be unsafe on a fault).
    let mut view = view_idle_rotary();
    view.connection = ConnectionState::Alarm;
    let state = HarnessState::new(view, UiState::default());
    let mut harness = build_jog_harness(state);
    harness.run();
    // The A control renders (rotary board) but is inert on a fault.
    assert!(harness.query_by_label("A+").is_some(), "a rotary board still renders the A column, even in Alarm");
    if let Some(node) = harness.query_by_label("A+") {
      node.click();
      harness.run();
    }
    assert!(
      !harness.state().intents.iter().any(|i| matches!(i, Intent::Jog { .. } | Intent::JogStart { .. })),
      "a disabled A jog must not emit any jog intent: {:?}",
      harness.state().intents
    );
  }

  #[test]
  fn the_jog_pad_hides_the_a_column_on_a_three_axis_firmware() {
    // Finding: the jog A column was ungated, so on a plain 3-axis board a click sent `$J=...A...`, which the firmware
    // rejects with `error:N` and then holds the stream in the error state. The A controls must be HIDDEN whenever the
    // report is not 4-field (matching the DRO's A-row gate). `view_idle` reports no position → reads as 3-axis.
    let state = HarnessState::new(view_idle(), UiState::default());
    let mut harness = build_jog_harness(state);
    harness.run();
    assert!(harness.query_by_label("A+").is_none(), "a 3-axis board must not offer an A+ jog");
    assert!(harness.query_by_label("A−").is_none(), "a 3-axis board must not offer an A− jog");
    // The Z column is unaffected — a 3-axis board still jogs Z.
    assert!(harness.query_by_label("Z+").is_some(), "the Z column must remain on a 3-axis board");
  }

  #[test]
  fn the_dro_shows_an_a_axis_row_only_when_the_firmware_reports_four_axes() {
    // A rotary-enabled firmware reports 4-field positions (DOC-10); the DRO must then show an A row (in degrees).
    // A plain 3-axis report must NOT grow a phantom A row.
    let mut view3 = ViewState::default();
    view3.connection = ConnectionState::Idle;
    view3.apply(crate::engine::Event::Response(crate::protocol::Response::Status(
      "Idle|MPos:1.000,2.000,3.000|WCO:0.000,0.000,0.000".to_string(),
    )));
    let mut harness = build_dro_harness(HarnessState::new(view3, UiState::default()));
    harness.run();
    assert!(harness.query_by_label("A").is_none(), "a 3-axis report must not render an A row");

    let mut view4 = ViewState::default();
    view4.connection = ConnectionState::Idle;
    view4.apply(crate::engine::Event::Response(crate::protocol::Response::Status(
      "Idle|MPos:1.000,2.000,3.000,45.000|WCO:0.000,0.000,0.000,0.000".to_string(),
    )));
    let mut harness = build_dro_harness(HarnessState::new(view4, UiState::default()));
    harness.run();
    assert!(harness.query_by_label("A").is_some(), "a 4-axis report must render the A row");
    // The DRO pads values to 8 columns (`{:>8.3}`), so the rendered label carries leading spaces.
    assert!(
      harness.query_by_label(&format!("{:>8.3}", 45.0)).is_some(),
      "the A row must carry the reported angle"
    );
  }

  #[test]
  fn the_transport_group_stop_button_issues_the_graceful_program_stop_not_a_soft_reset() {
    // The everyday Stop is the GRACEFUL program stop (`0x86`): clicking it must emit ProgramStop and must NOT emit
    // the hard SoftReset — the regression this guards is Stop secretly alarming the controller.
    use crate::protocol::RealtimeCommand;
    let state = HarnessState::new(view_running(), UiState::default());
    let mut harness = build_transport_group_harness(state);
    harness.run();

    harness.get_by_label("■ Stop").click();
    harness.run();

    let intents = &harness.state().intents;
    assert!(
      intents.iter().any(|i| matches!(i, Intent::Realtime(RealtimeCommand::ProgramStop))),
      "Stop must emit the graceful ProgramStop",
    );
    assert!(
      !intents.iter().any(|i| matches!(i, Intent::Realtime(RealtimeCommand::SoftReset))),
      "Stop must NOT emit the hard SoftReset",
    );
  }

  #[test]
  fn the_transport_group_abort_button_issues_the_hard_soft_reset() {
    // The separate Abort / E-stop is the HARD soft-reset (`0x18`): clicking it must emit SoftReset (and not the
    // graceful ProgramStop). The two controls are distinct and fire distinct intents.
    use crate::protocol::RealtimeCommand;
    let state = HarnessState::new(view_running(), UiState::default());
    let mut harness = build_transport_group_harness(state);
    harness.run();

    harness.get_by_label("⏹ Abort").click();
    harness.run();

    let intents = &harness.state().intents;
    assert!(
      intents.iter().any(|i| matches!(i, Intent::Realtime(RealtimeCommand::SoftReset))),
      "Abort must emit the hard SoftReset",
    );
    assert!(
      !intents.iter().any(|i| matches!(i, Intent::Realtime(RealtimeCommand::ProgramStop))),
      "Abort must NOT emit the graceful ProgramStop",
    );
  }

  #[test]
  fn the_transport_group_simulate_button_emits_simulate_when_a_program_is_loaded() {
    // The Simulate button is a host-only action available whenever a program is loaded — even disconnected. With a
    // program in `UiState`, clicking it must emit `Intent::Simulate` (and nothing is sent to the engine — the shell
    // handles that purity; here we only prove the button wires the intent).
    let mut ui = UiState::default();
    ui.set_program(vec!["G1 X10 F500".to_string()], None);
    // A disconnected view is fine: Simulate does not need a live link.
    let state = HarnessState::new(ViewState::default(), ui);
    let mut harness = build_transport_group_harness(state);
    harness.run();

    harness.get_by_label("≈ Simulate").click();
    harness.run();

    assert!(
      harness.state().intents.iter().any(|i| matches!(i, Intent::Simulate)),
      "clicking Simulate with a program loaded must emit Intent::Simulate",
    );
  }

  #[test]
  fn the_transport_group_simulate_button_is_disabled_with_no_program() {
    // With no program loaded the Simulate button is greyed and a click commands nothing — there is nothing to
    // estimate. The button still renders (so it is discoverable), but emits no intent when clicked.
    let state = HarnessState::new(ViewState::default(), UiState::default());
    let mut harness = build_transport_group_harness(state);
    harness.run();
    assert!(harness.query_by_label("≈ Simulate").is_some(), "the Simulate control is present even with no program");
    if let Some(node) = harness.query_by_label("≈ Simulate") {
      node.click();
      harness.run();
    }
    assert!(
      !harness.state().intents.iter().any(|i| matches!(i, Intent::Simulate)),
      "a disabled Simulate button must not emit Intent::Simulate",
    );
  }

  #[test]
  fn the_transport_group_exposes_both_a_stop_and_a_separate_abort_control() {
    // Both controls are rendered side by side while running, so the operator has the clean Stop and the emergency
    // Abort available at once — the user's explicit two-control design.
    let state = HarnessState::new(view_running(), UiState::default());
    let mut harness = build_transport_group_harness(state);
    harness.run();
    assert!(harness.query_by_label("■ Stop").is_some(), "the graceful Stop control must be present");
    assert!(harness.query_by_label("⏹ Abort").is_some(), "the separate Abort control must be present");
  }

  #[test]
  fn the_toolpath_view_paints_the_configured_palette_not_the_baked_in_default() {
    // The integration proof of the whole config wiring: a view must render the palette threaded through `UiState`,
    // NOT the old baked-in `Theme::*` constants. We give the toolpath view a sentinel palette whose `inset`
    // background fill is a colour found nowhere in the default palette, render it, and assert the tessellated mesh
    // carries the sentinel — and does NOT carry the default `inset`. If the view had read a constant, the sentinel
    // would be absent and the default present, failing the test.
    use crate::app::theme::Palette;

    let sentinel = egui::Color32::from_rgb(0x7A, 0x12, 0x9C); // a vivid purple, not in the design palette.
    let default_inset = Palette::default_dark().inset;
    assert_ne!(sentinel, default_inset, "the sentinel must differ from the default so the assertion is meaningful");

    let mut ui = UiState::default();
    ui.style.palette.inset = sentinel; // recolour only the toolpath background fill.

    let state = HarnessState::new(ViewState::default(), ui);
    let mut harness = build_toolpath_harness(state);
    harness.run();

    let colors = painted_vertex_colors(&harness);
    assert!(
      colors.contains(&sentinel),
      "the toolpath view must paint the CONFIGURED inset colour (proves it reads the palette, not a constant)",
    );
    assert!(
      !colors.contains(&default_inset),
      "the default inset colour must be absent — the view must not fall back to the baked-in constant",
    );
  }

  /// A connected, idle [`ViewState`] — the link is up and no program is streaming, so the console's manual-command
  /// (MDI) field is enabled and a typed line may be submitted.
  fn view_idle() -> ViewState {
    let mut view = ViewState::default();
    view.connection = ConnectionState::Idle;
    view
  }

  /// An Idle view whose latest status report carries a 4-field (rotary) position, so the DOC-10 A controls — the
  /// DRO A row and the jog A column — render. Plain `view_idle` reports no position and so reads as a 3-axis board,
  /// where the A controls are correctly hidden (a `$J=...A...` on a 3-axis firmware errors and wedges the stream).
  fn view_idle_rotary() -> ViewState {
    let mut view = ViewState::default();
    view.connection = ConnectionState::Idle;
    view.apply(crate::engine::Event::Response(crate::protocol::Response::Status(
      "Idle|MPos:1.000,2.000,3.000,45.000|WCO:0.000,0.000,0.000,0.000".to_string(),
    )));
    view
  }

  /// A fixed elapsed/ETA estimate for the dock progress clock so the rendered readout is deterministic.
  fn zero_time() -> TimeEstimate {
    TimeEstimate { elapsed: Duration::from_secs(0), remaining: None, total: None }
  }

  #[test]
  fn creating_a_theme_snapshots_the_active_palette_and_selects_it() {
    // The app settings "Create from current" flow: with a name typed, clicking Create must emit a fully-keyed
    // UpsertTheme snapshotting the ACTIVE palette, then a SetActiveTheme for it, and clear the draft field.
    let mut ui = UiState::default();
    ui.theme_name_draft = "shop-red".to_string();
    let state = HarnessState::new(ViewState::default(), ui);
    let mut harness = build_app_settings_harness(state, crate::config::Config::default());
    harness.run();

    harness.get_by_label("Create from current").click();
    harness.run();

    let intents = &harness.state().intents;
    let upsert = intents
      .iter()
      .find_map(|i| match i {
        Intent::UpsertTheme { name, theme } => Some((name.clone(), theme.clone())),
        _ => None,
      })
      .expect("Create must emit an UpsertTheme");
    assert_eq!(upsert.0, "shop-red");
    assert!(upsert.1.all_color_fields_set(), "the created theme snapshots every token, so every picker is concrete");
    assert!(
      intents.iter().any(|i| matches!(i, Intent::SetActiveTheme(name) if name == "shop-red")),
      "the created theme becomes active: {intents:?}"
    );
    assert!(harness.state().ui.theme_name_draft.is_empty(), "the name draft clears once created");
  }

  #[test]
  fn the_toolbar_collapses_to_icon_form_when_the_full_labels_cannot_fit() {
    // The self-measuring toolbar: at a width where the full labels cannot fit (locale-dependent — Swedish
    // "Inställningar"/"Nödstopp" overflow the 800px minimum window, and even English overflows at 640), the
    // secondary controls collapse to icon glyphs instead of overlapping the state badge. The bar renders full
    // once, measures, and flips — so the verdict lands by the second frame.
    let state = HarnessState::new(view_idle(), UiState::default());
    let mut harness = build_shell_harness(state, egui::vec2(640.0, 500.0), zero_time());
    harness.run_steps(3);
    assert!(
      harness.query_by_label("🛠").is_some(),
      "at 640px the firmware-settings control must collapse to its 🛠 icon form"
    );
    assert!(
      harness.query_by_label("Settings").is_none(),
      "the full Settings label must be gone in compact form (it cannot fit)"
    );
    // The compact controls must actually clear each other and the right-aligned cluster — no overlap: the
    // firmware-settings icon ends left of the app-settings gear, which ends left of the badge, all on-screen.
    let settings = harness.get_by_label("🛠").rect();
    let gear = harness.get_by_label("⚙").rect();
    assert!(
      settings.right() <= gear.left() + 1.0,
      "compact settings ({}) must not overlap the gear ({})",
      settings.right(),
      gear.left()
    );
    assert!(gear.right() <= 640.0, "the right cluster must stay inside the window");
  }

  #[test]
  fn the_toolbar_recovers_from_compact_when_the_full_form_shrinks_to_fit() {
    // The shrink direction of the self-measuring toolbar: the DISCONNECTED bar (port combo + refresh + identify
    // + connect) is far wider than the connected one, so at 900px it must go compact — and after CONNECTING
    // (the whole group collapses to one Disconnect button) the full labels fit again and the bar must return to
    // them. The stored full-form measurement is only refreshed while rendering full, so without invalidating it
    // on a content change the bar stayed icon-only forever at this width (the stuck-compact bug).
    let mut ui = UiState::default();
    ui.ports = vec![crate::transport::ports::PortInfo::bare("/dev/cu.usbmodemFAKE1")];
    ui.selected_port = "/dev/cu.usbmodemFAKE1".to_string();
    let state = HarnessState::new(ViewState::default(), ui); // Disconnected.
    let mut harness = build_shell_harness(state, egui::vec2(900.0, 500.0), zero_time());
    harness.run_steps(3);
    assert!(
      harness.query_by_label("🛠").is_some(),
      "precondition: the disconnected bar must be compact at 900px (if this fails, retune the test width)"
    );

    // Connect: the port group collapses to a single Disconnect button — the full form now fits.
    harness.state_mut().view = view_idle();
    harness.run_steps(3);
    assert!(
      harness.query_by_label("Settings").is_some(),
      "after connecting, the full labels fit at 900px and the bar must RECOVER from compact"
    );
    assert!(harness.query_by_label("🛠").is_none(), "the icon form must be gone once the full form fits again");
  }

  #[test]
  fn the_toolbar_keeps_full_labels_when_they_fit() {
    // The flip side: at a comfortable width the bar stays in its full labelled form — no icon-only degradation.
    let state = HarnessState::new(view_idle(), UiState::default());
    let mut harness = build_shell_harness(state, egui::vec2(1280.0, 800.0), zero_time());
    harness.run_steps(3);
    assert!(harness.query_by_label("Settings").is_some(), "at 1280px the full Settings label fits and shows");
    assert!(harness.query_by_label("🛠").is_none(), "no icon-only degradation when the full labels fit");
  }

  #[test]
  fn the_toolbar_gear_toggles_the_app_settings_dialog() {
    // The ⚙ ghost button beside the state badge opens (and closes) the app settings dialog.
    let state = HarnessState::new(view_idle(), UiState::default());
    let mut harness = build_shell_harness(state, egui::vec2(1280.0, 800.0), zero_time());
    harness.run();
    assert!(!harness.state().ui.app_settings_open);
    harness.get_by_label("⚙").click();
    harness.run();
    assert!(harness.state().ui.app_settings_open, "clicking the gear must open the app settings dialog");
    harness.get_by_label("⚙").click();
    harness.run();
    assert!(!harness.state().ui.app_settings_open, "clicking it again must close the dialog");
  }

  #[test]
  fn the_console_dock_holds_its_size_across_frames_without_input() {
    // The user-reported "console keeps resizing itself": a resizable egui panel PERSISTS its CONTENT's measured
    // rect as next frame's panel size (`PanelState { rect }.store`), so any console body that does not exactly
    // fill the panel feeds its error back 1:1 and the dock creeps frame over frame with no input at all. The
    // tab strip's top edge must stay put across many idle frames.
    let mut ui = UiState::default();
    ui.active_tab = views::DockTab::Console;
    let state = HarnessState::new(view_idle(), ui);
    let mut harness = build_docked_panel_harness(state, zero_time());
    harness.run_steps(3); // settle fonts/theme and the first stored panel size.
    let top_before = harness.get_by_label("Console").rect().top();
    harness.run_steps(20);
    let top_after = harness.get_by_label("Console").rect().top();
    assert!(
      (top_after - top_before).abs() <= 0.5,
      "the dock must hold its size with no input: strip top drifted {top_before} -> {top_after} over 20 frames"
    );
  }

  #[test]
  fn the_console_dock_holds_a_manually_dragged_size() {
    // The other half of the report: a size the operator SET must stick. Drag the dock's top edge up, then run
    // many uneventful frames — the dragged size must survive them, not be fought back frame by frame.
    let mut ui = UiState::default();
    ui.active_tab = views::DockTab::Console;
    let state = HarnessState::new(view_idle(), ui);
    let mut harness = build_docked_panel_harness(state, zero_time());
    harness.run_steps(3);
    let dock_top = 600.0 - 8.0 - Metrics::STATUS_BAR_H - Metrics::DOCK_H;
    let handle = egui::pos2(450.0, dock_top);
    let target = egui::pos2(450.0, dock_top - 80.0);
    harness.hover_at(handle);
    harness.drag_at(handle);
    harness.hover_at(egui::pos2(450.0, dock_top - 40.0));
    harness.hover_at(target);
    harness.drop_at(target);
    harness.run();
    let top_after_drag = harness.get_by_label("Console").rect().top();
    harness.run_steps(20);
    let top_later = harness.get_by_label("Console").rect().top();
    assert!(
      (top_later - top_after_drag).abs() <= 0.5,
      "a dragged dock size must stick: strip top drifted {top_after_drag} -> {top_later} over 20 frames"
    );
  }

  #[test]
  fn the_app_settings_window_holds_its_size_across_frames() {
    // The dialog half of the report: the app-settings window (rendered as the REAL ctx-level `Window`, with the
    // tall user-theme picker content) must keep a stable rect across uneventful frames — not auto-grow or
    // auto-shrink in a content/size feedback loop.
    ensure_locales_seeded();
    let mut config = crate::config::Config::default();
    let theme = crate::config::ThemeOverride::from_palette(&crate::app::theme::Palette::default_dark());
    config.appearance.themes.insert("fixture-theme".to_string(), theme);
    config.appearance.active_theme = "fixture-theme".to_string();
    let mut ui = UiState::default();
    ui.app_settings_open = true;
    let state = HarnessState::new(ViewState::default(), ui);
    let mut harness = Harness::builder().with_size(egui::vec2(760.0, 820.0)).build_ui_state(
      move |ui, state: &mut HarnessState| {
        let mut sink = IntentSink::new();
        crate::app::app_settings::window(ui.ctx(), &mut state.ui, &config, true, &mut sink);
        state.intents.extend(sink.drain());
      },
      state,
    );
    harness.run_steps(5);
    let window_id = egui::Id::new("app-settings-window");
    let rect_before = harness
      .ctx
      .memory(|m| m.area_rect(window_id))
      .expect("the app settings window must have an area rect");
    harness.run_steps(20);
    let rect_after = harness
      .ctx
      .memory(|m| m.area_rect(window_id))
      .expect("the app settings window must still be open");
    assert!(
      (rect_after.height() - rect_before.height()).abs() <= 0.5
        && (rect_after.width() - rect_before.width()).abs() <= 0.5,
      "the window must hold its size across frames: {rect_before:?} -> {rect_after:?} over 20 frames"
    );
  }

  #[test]
  fn the_app_settings_window_holds_a_manually_dragged_size() {
    // The dialog half of the report, drag direction: resize the REAL window by its bottom-right corner, then run
    // many uneventful frames — the dragged size must stick, not be fought back by content-driven auto-sizing.
    ensure_locales_seeded();
    let mut config = crate::config::Config::default();
    let theme = crate::config::ThemeOverride::from_palette(&crate::app::theme::Palette::default_dark());
    config.appearance.themes.insert("fixture-theme".to_string(), theme);
    config.appearance.active_theme = "fixture-theme".to_string();
    let mut ui = UiState::default();
    ui.app_settings_open = true;
    let state = HarnessState::new(ViewState::default(), ui);
    let mut harness = Harness::builder().with_size(egui::vec2(760.0, 860.0)).build_ui_state(
      move |ui, state: &mut HarnessState| {
        let mut sink = IntentSink::new();
        crate::app::app_settings::window(ui.ctx(), &mut state.ui, &config, true, &mut sink);
        state.intents.extend(sink.drain());
      },
      state,
    );
    harness.run_steps(5);
    let window_id = egui::Id::new("app-settings-window");
    let rect = harness.ctx.memory(|m| m.area_rect(window_id)).expect("window area rect");
    // Drag the bottom-right resize corner 60px out both ways.
    let corner = rect.right_bottom() - egui::vec2(2.0, 2.0);
    let target = corner + egui::vec2(60.0, 60.0);
    harness.hover_at(corner);
    harness.drag_at(corner);
    harness.hover_at(corner + egui::vec2(30.0, 30.0));
    harness.hover_at(target);
    harness.drop_at(target);
    harness.run();
    let dragged = harness.ctx.memory(|m| m.area_rect(window_id)).expect("window area rect");
    assert!(
      dragged.height() > rect.height() + 30.0,
      "precondition: the corner drag must actually grow the window ({rect:?} -> {dragged:?})"
    );
    harness.run_steps(20);
    let later = harness.ctx.memory(|m| m.area_rect(window_id)).expect("window area rect");
    assert!(
      (later.height() - dragged.height()).abs() <= 0.5 && (later.width() - dragged.width()).abs() <= 0.5,
      "a dragged window size must stick: {dragged:?} -> {later:?} over 20 frames"
    );
  }

  #[test]
  fn the_app_settings_window_accepts_a_vertical_resize_with_a_builtin_theme() {
    // THE dialog defect the user hit: with a BUILT-IN theme active the dialog's content was short and did not
    // fill, and egui snaps a resizable window's height back to its content's natural height — so a manual
    // vertical resize was overridden the moment the mouse released ("it immediately starts resizing again").
    // The content now always fills the window (save row anchored at the bottom, editor/hint region filling), so
    // a height drag must both TAKE and STICK.
    ensure_locales_seeded();
    let config = crate::config::Config::default(); // built-in "default" theme — the short-content case.
    let mut ui = UiState::default();
    ui.app_settings_open = true;
    let state = HarnessState::new(ViewState::default(), ui);
    let mut harness = Harness::builder().with_size(egui::vec2(760.0, 860.0)).build_ui_state(
      move |ui, state: &mut HarnessState| {
        let mut sink = IntentSink::new();
        crate::app::app_settings::window(ui.ctx(), &mut state.ui, &config, true, &mut sink);
        state.intents.extend(sink.drain());
      },
      state,
    );
    harness.run_steps(5);
    let window_id = egui::Id::new("app-settings-window");
    let rect = harness.ctx.memory(|m| m.area_rect(window_id)).expect("window area rect");
    let corner = rect.right_bottom() - egui::vec2(2.0, 2.0);
    let target = corner + egui::vec2(60.0, 80.0);
    harness.hover_at(corner);
    harness.drag_at(corner);
    harness.hover_at(corner + egui::vec2(30.0, 40.0));
    harness.hover_at(target);
    harness.drop_at(target);
    harness.run();
    let dragged = harness.ctx.memory(|m| m.area_rect(window_id)).expect("window area rect");
    assert!(
      dragged.height() > rect.height() + 40.0,
      "the vertical drag must TAKE with a built-in theme ({rect:?} -> {dragged:?})"
    );
    harness.run_steps(20);
    let later = harness.ctx.memory(|m| m.area_rect(window_id)).expect("window area rect");
    assert!(
      (later.height() - dragged.height()).abs() <= 0.5,
      "the dragged height must STICK: {dragged:?} -> {later:?} over 20 frames"
    );
  }

  #[test]
  fn the_theme_name_field_matches_the_height_of_the_button_beside_it() {
    // The user-flagged inconsistency: text inputs must sit at the same control height as adjacent buttons. The
    // "new theme name…" field and its "Create from current" button share a row, so their heights must agree.
    let state = HarnessState::new(ViewState::default(), UiState::default());
    let mut harness = build_app_settings_harness(state, crate::config::Config::default());
    harness.run();
    let field = harness.get_by_role(egui::accesskit::Role::TextInput).rect();
    let button = harness.get_by_label("Create from current").rect();
    assert!(
      (field.height() - button.height()).abs() <= 1.5,
      "the theme-name field ({}px) must match the Create button height ({}px)",
      field.height(),
      button.height()
    );
  }

  #[test]
  fn editing_a_user_theme_color_through_the_picker_updates_the_config_and_palette() {
    // The end-to-end colour-picker proof: click a swatch in a USER theme → the picker popup opens → commit a new
    // channel value → the theme in the config, the RESOLVED live palette, and the unsaved marker all update. The
    // rig folds drained intents back the way `SkirnirApp::handle_intent` does (upsert + re-resolve), so the test
    // observes the live UI updating, not just an intent being emitted.
    use egui::accesskit::Role;
    struct Rig {
      ui: UiState,
      config: crate::config::Config,
      dirty: bool,
    }
    let mut config = crate::config::Config::default();
    let captured = crate::config::ThemeOverride::from_palette(&crate::app::theme::Palette::default_dark());
    config.appearance.themes.insert("fixture-theme".to_string(), captured);
    config.appearance.active_theme = "fixture-theme".to_string();
    ensure_locales_seeded();
    let rig = Rig { ui: UiState::default(), config, dirty: false };
    let mut harness = Harness::builder().with_size(egui::vec2(460.0, 640.0)).build_ui_state(
      |ui, rig: &mut Rig| {
        let mut sink = IntentSink::new();
        crate::app::app_settings::body(ui, &mut rig.ui, &rig.config, rig.dirty, &mut sink);
        for intent in sink.drain() {
          match intent {
            Intent::UpsertTheme { name, theme } => {
              rig.config.appearance.themes.insert(name, theme);
              rig.dirty = true;
              let (palette, _) = rig.config.palette();
              rig.ui.style.palette = palette;
            }
            Intent::SetActiveTheme(name) => {
              rig.config.appearance.active_theme = name;
              let (palette, _) = rig.config.palette();
              rig.ui.style.palette = palette;
            }
            _ => {}
          }
        }
      },
      rig,
    );
    harness.run();

    // The swatches appear in `color_entries_mut` declaration order; index 10 is `accent` (#0E86D4, R = 0x0E).
    let spin_count_before = harness.query_all_by_role(Role::SpinButton).count();
    {
      let swatches: Vec<_> = harness.query_all_by_role(Role::ColorWell).collect();
      assert!(swatches.len() > 10, "the user-theme editor must render a swatch per token, got {}", swatches.len());
      swatches[10].click();
    }
    harness.run();

    // The popup contributes the R/G/B channel DragValues (spin buttons) beyond those already in the dialog.
    {
      let spins: Vec<_> = harness.query_all_by_role(Role::SpinButton).collect();
      assert!(
        spins.len() > spin_count_before,
        "clicking a swatch must open the picker popup (spin buttons {} -> {})",
        spin_count_before,
        spins.len()
      );
      // Focus the popup's first channel field (R) and type a new value; Enter commits the DragValue edit.
      spins[spin_count_before].focus();
    }
    harness.run();
    {
      let spins: Vec<_> = harness.query_all_by_role(Role::SpinButton).collect();
      spins[spin_count_before].type_text("255");
    }
    harness.run();
    harness.key_press(egui::Key::Enter);
    harness.run();

    let theme = harness
      .state()
      .config
      .appearance
      .themes
      .get("fixture-theme")
      .expect("the fixture theme stays present");
    let accent = theme.accent.expect("the fixture theme is fully keyed");
    assert_eq!(accent.r, 255, "the picker edit must land in the theme's accent red channel, got {accent:?}");
    assert_eq!(
      harness.state().ui.style.palette.accent.r(),
      255,
      "the LIVE resolved palette must carry the edited colour"
    );
    assert!(harness.state().dirty, "a colour edit must raise the unsaved-changes marker");
  }

  #[test]
  fn the_app_settings_save_button_emits_save_config() {
    // The explicit save boundary: the Save button (enabled — the harness renders dirty=true) emits SaveConfig
    // and nothing writes the file implicitly before that.
    let state = HarnessState::new(ViewState::default(), UiState::default());
    let mut harness = build_app_settings_harness(state, crate::config::Config::default());
    harness.run();
    harness.get_by_label("Save to config.json").click();
    harness.run();
    assert!(
      harness.state().intents.iter().any(|i| matches!(i, Intent::SaveConfig)),
      "Save must emit Intent::SaveConfig: {:?}",
      harness.state().intents
    );
  }

  #[test]
  fn the_console_dock_resizes_vertically_by_dragging_its_top_edge() {
    // The dock is a resizable bottom panel: dragging its top edge upward must GROW the dock (the tab strip moves
    // up with it) and the MDI input must stay visible at the bottom. Uses the real `views::dock_panel` inside the
    // shell-faithful harness, so this drives egui's actual panel-resize interaction, not a synthetic height.
    let mut ui = UiState::default();
    ui.active_tab = views::DockTab::Console;
    let state = HarnessState::new(view_idle(), ui);
    let mut harness = build_docked_panel_harness(state, zero_time());
    harness.run();

    let strip_top_before = harness.get_by_label("Console").rect().top();

    // The resize handle is egui's ±5px grab band around the panel's TOP EDGE — computed from the harness layout
    // (600px window, kittest's 8px root inset, the 24px status bar, the 200px default dock), not from the tab
    // label, which sits ~10px below the edge (outside the band — aiming there grabs nothing). Drag it 80px up.
    let dock_top = 600.0 - 8.0 - Metrics::STATUS_BAR_H - Metrics::DOCK_H;
    let handle = egui::pos2(450.0, dock_top);
    let target = egui::pos2(450.0, dock_top - 80.0);
    harness.hover_at(handle);
    harness.drag_at(handle);
    harness.hover_at(egui::pos2(450.0, dock_top - 40.0));
    harness.hover_at(target);
    harness.drop_at(target);
    harness.run();

    let strip_top_after = harness.get_by_label("Console").rect().top();
    assert!(
      strip_top_before - strip_top_after > 40.0,
      "dragging the dock's top edge up must grow the dock (strip top {strip_top_before} -> {strip_top_after})"
    );
    // The MDI input must still be laid out inside the grown panel — resizing must never cost the command line.
    assert!(
      harness.query_by_role(egui::accesskit::Role::TextInput).is_some(),
      "the MDI input must survive a dock resize"
    );
  }

  #[test]
  fn the_console_mdi_field_renders_with_usable_size_through_the_real_docked_panel() {
    // REGRESSION (the field vanished from the Console tab), in TWO parts — both blind spots of a directly-invoked
    // widget harness, both caught here by rendering through the real `Panel::bottom().exact_size` dock path:
    //   (height) the log `ScrollArea` (`auto_shrink([false, false])`) filled ALL remaining height of the pinned 200px
    //     panel, pushing the input row below the panel floor where it was clipped away; and
    //   (WIDTH) the Send button was laid out first inside a nested `with_layout(right_to_left)` child, which claims
    //     the WHOLE remaining row width — so the field added afterward had zero width left and collapsed to an
    //     invisible sliver (the empty dark gap beside Send the user reported).
    // This asserts the TextInput exists AND has BOTH a usable height and a usable width, inside the visible panel.
    let mut ui = UiState::default();
    ui.active_tab = views::DockTab::Console;
    let state = HarnessState::new(view_idle(), ui);
    let mut harness = build_docked_panel_harness(state, zero_time());
    harness.run();
    let field = harness
      .query_by_role(egui::accesskit::Role::TextInput)
      .expect("the MDI text input must render on the Console tab");
    let rect = field.rect();
    // The dock panel floor: window 600px tall minus the 24px status bar below it. A field laid out BELOW this floor
    // is clipped away (the height half of the bug).
    let dock_floor = 600.0 - Metrics::STATUS_BAR_H;
    assert!(
      (rect.height() - Metrics::PANEL_CONTROL_H).abs() <= 1.5,
      "the MDI field must match the {}px control height of the Send button beside it, got {}px",
      Metrics::PANEL_CONTROL_H,
      rect.height()
    );
    assert!(
      rect.bottom() <= dock_floor + 1.0,
      "the MDI field bottom ({}) must lie within the dock panel floor ({dock_floor}) — a field pushed below it is \
       invisible (the vanished-MDI bug)", rect.bottom()
    );
    // The WIDTH half: in a 900px-wide dock the field must be a real, usable box, not a ~0px sliver beside Send. A
    // zero-width field satisfies "has height" and "inside the panel" yet is invisible and unusable — exactly the
    // blind spot that let the earlier test pass while the box was gone.
    assert!(
      rect.width() >= 100.0,
      "the MDI field must have a usable width, got {}px — the Send button consumed the row and collapsed the field",
      rect.width()
    );
  }

  #[test]
  fn the_console_mdi_send_button_submits_the_typed_line_and_clears_the_field() {
    // The MDI field routes a typed G-code/`$` line through the engine's manual-send path (`Intent::SendLine`), the
    // same path the jog/probe/override controls use. Pre-fill the field, click Send, and assert the intent carries
    // the exact line and the field is cleared for the next command.
    let mut ui = UiState::default();
    ui.console_input = "G0 X1 Y2".to_string();
    let state = HarnessState::new(view_idle(), ui);
    let mut harness = build_dock_harness(state, zero_time());
    harness.run();
    // Click via accesskit (the button sits in a nested right-to-left layout; the accesskit click action triggers it
    // reliably regardless of the pointer-rect geometry, where a raw pointer click can miss).
    harness.get_by_label("Send").click_accesskit();
    harness.run();
    assert!(
      harness.state().intents.iter().any(|i| matches!(i, Intent::SendLine(line) if line == "G0 X1 Y2")),
      "clicking Send must submit the typed line verbatim through Intent::SendLine: {:?}", harness.state().intents
    );
    assert!(harness.state().ui.console_input.is_empty(), "the field is cleared on send so the next command starts fresh");
  }

  #[test]
  fn the_console_mdi_submits_on_enter() {
    // Enter in the field submits exactly like the Send button (the design's command-line behaviour). Focus the
    // single text input, press Enter, and assert the line went out through Intent::SendLine and the field cleared.
    let mut ui = UiState::default();
    ui.console_input = "$$".to_string();
    let state = HarnessState::new(view_idle(), ui);
    let mut harness = build_dock_harness(state, zero_time());
    harness.run();
    // Focus the single text input, then press Enter at the harness level (the field node borrows the harness, so the
    // key press is issued after that borrow ends). egui's TextEdit treats Enter as a submit, losing focus — exactly
    // the `lost_focus() && Enter` the console body keys submission off.
    harness.get_by_role(egui::accesskit::Role::TextInput).focus();
    harness.run();
    harness.key_press(egui::Key::Enter);
    harness.run();
    assert!(
      harness.state().intents.iter().any(|i| matches!(i, Intent::SendLine(line) if line == "$$")),
      "Enter must submit the typed `$` command through Intent::SendLine: {:?}", harness.state().intents
    );
    assert!(harness.state().ui.console_input.is_empty(), "the field is cleared after an Enter submit");
  }

  #[test]
  fn the_console_mdi_recalls_sent_lines_with_arrow_up_and_down() {
    // Shell-style history on the command line: after sending a line, ↑ in the focused field recalls it, and ↓
    // steps back to the (empty) draft. Drives the real console body through the dock harness.
    let mut ui = UiState::default();
    ui.console_input = "G0 X1 Y2".to_string();
    let state = HarnessState::new(view_idle(), ui);
    let mut harness = build_dock_harness(state, zero_time());
    harness.run();

    // Send the line (clears the field and records it in the history).
    harness.get_by_label("Send").click_accesskit();
    harness.run();
    assert!(harness.state().ui.console_input.is_empty(), "sending clears the field");

    // Focus the field and press ↑: the sent line comes back for editing/re-sending.
    harness.get_by_role(egui::accesskit::Role::TextInput).focus();
    harness.run();
    harness.key_press(egui::Key::ArrowUp);
    harness.run();
    assert_eq!(harness.state().ui.console_input, "G0 X1 Y2", "↑ must recall the last sent line");

    // ↓ steps past the newest entry and restores the (empty) draft.
    harness.key_press(egui::Key::ArrowDown);
    harness.run();
    assert!(harness.state().ui.console_input.is_empty(), "↓ past the newest entry restores the empty draft");
  }

  #[test]
  fn the_console_mdi_is_inert_while_disconnected() {
    // While disconnected the MDI field and Send button are disabled, and even a buffered line is never submitted —
    // a manual line must reach the firmware only over a live link. The Send label is still present (disabled), so a
    // click on it must produce no Intent::SendLine and must not clear the field.
    let mut ui = UiState::default();
    ui.console_input = "G0 X1".to_string();
    let state = HarnessState::new(ViewState::default(), ui); // default connection == Disconnected.
    let mut harness = build_dock_harness(state, zero_time());
    harness.run();
    if let Some(send) = harness.query_by_label("Send") {
      send.click();
      harness.run();
    }
    assert!(
      !harness.state().intents.iter().any(|i| matches!(i, Intent::SendLine(_))),
      "no manual line may be submitted while disconnected: {:?}", harness.state().intents
    );
    assert_eq!(harness.state().ui.console_input, "G0 X1", "a disconnected field keeps its text (nothing was sent)");
  }
}
