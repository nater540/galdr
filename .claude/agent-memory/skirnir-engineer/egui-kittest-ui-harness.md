---
name: egui-kittest-ui-harness
description: The egui_kittest UI test harness pattern in skirnir — per-view closure harness, label-less widget rect probe, pointer drag simulation
metadata:
  type: project
---

The project's first egui UI test harness, the pattern for repro/regression of UI bugs the pure
reducer/intent tests can't reach (real pointer hit-testing, multi-frame widget state: drags/focus/hover). See
[[gui-architecture]] for the layer split.

**Dep.** `egui_kittest = { version = "0.34.3", default-features = false }` (dev-dep, version-matched to egui
0.34). NO `wgpu`/`snapshot` features — those are for image-diff snapshot tests (GPU); interaction tests use
egui's tessellation only and run headless/CI-clean. Snapshots are a later add (would need the `wgpu` feature).

**Pattern — per-view closure harness, NOT `Harness::new_eframe`.** `SkirnirApp::new` spins up the engine +
tokio + channels (heavy/non-deterministic), so don't mount the whole app. Views are free fns taking exactly
their state (`views::overrides(ui, &view, &mut state, &mut sink)`), so mount ONE view in a closure with a
hand-built ViewState/UiState and read emitted Intents back out. Harness module:
`crates/skirnir/src/app/ui_test.rs`, gated `#[cfg(all(test, feature = "gui"))]`, registered in `app/mod.rs`.
Has module-level docs with the 4-step "how to write a UI test" recipe. `HarnessState { view, ui, intents }`
accumulates drained intents across frames; `build_overrides_harness` uses
`Harness::builder().with_size(..).build_ui_state(closure, state)`.

**kittest API quirks (0.34.3, read from the installed source).** Construct: `Harness::new_ui` /
`new_ui_state` / `builder().build_ui_state`. Pointer helpers are `&self` and QUEUE events onto a lock; the next
`run()`/`step()` drains them ONE-event-per-frame. A realistic drag egui registers as a drag is:
`hover_at(p)` → `drag_at(p)` (press) → `hover_at(p2)` (MOVE while held — required, else no `dragged()`) →
`drop_at(p)` (release + PointerGone). Then `harness.run()`. Read state via `harness.state()`/`state_mut()`.
`with_size` must be set so widget rects are deterministic.

**Label-less hand-painted widget → rect probe.** `override_slider` is `allocate_exact_size` with no text, so
AccessKit can't find it. Seam: a test-only thread-local `views::slider_rect_probe` (mod gated
`cfg(all(test, feature="gui"))`) that `override_axis` calls with `slider.rect` each render
(`#[cfg(all(test, feature="gui"))] slider_rect_probe::record(axis, slider.rect)`). The harness reads it via
`slider_point_for(axis, target_pct)` which inverts the slider's linear pointer-x→10..=200% mapping to a `Pos2`.
This is the reusable trick for any hand-painted widget: record its rect via a cfg'd side channel, compute the
point.

**Gotcha — private fields block `..Default::default()` across modules.** `ViewState` has a private
`last_error_code`; from `app::ui_test` you CANNOT write `ViewState { ..ViewState::default() }`. Build
`ViewState::default()` then mutate the pub fields. Same for `UiState`'s private `toolpath`/`toolpath_bounds`.

**MachineState is a struct, not enum**: `MachineState { state: RunState::Idle, substate: None }`.

Verified the harness is a real guard: temporarily reverting the slider hold to the old snap-to-Idle made the
drag test FAIL ("must be Holding, was Idle"), restoring it passed. Do NOT run `cargo fmt` (4-space). Gate
counts after this work: `-p skirnir` (default gui+serial) 408 lib tests; `--no-default-features` 350.
