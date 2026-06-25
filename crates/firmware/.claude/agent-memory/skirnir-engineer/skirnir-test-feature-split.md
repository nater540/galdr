---
name: skirnir-test-feature-split
description: skirnir test commands — the gui feature gates which tests compile; default build runs ~522, kittest/views tests need --features gui
metadata:
  type: project
---

`cargo test -p skirnir` (default) runs the engine/reducer/intent/eta/shell logic tests (~522). The egui kittest UI
tests (`src/app/ui_test.rs`, in `#[cfg(all(test, feature = "gui"))] mod ui_test`) AND the `views.rs`/`shell.rs`
gui-gated code only compile under `--features gui`.

**Why:** the egui-touching layer (`theme`, `views`, `shell`, `metrics`, `fonts`) is behind the `gui` feature so a
headless build need not pull egui/eframe. Tests inside those modules (e.g. `views::tests::eta_qualifier_text…`) and
the kittest harness tests are therefore invisible to the default `cargo test`.

**How to apply:** when adding a view/shell test, ALWAYS verify with `RUSTFLAGS="-D warnings" cargo test -p skirnir
--features gui` — the default run will silently skip it and report green. The total count is identical (522) in both
runs because the gui-only test modules compile-in only under the feature; grep the output by test name to confirm a
new gui-gated test actually executed. Never use `--workspace` (it builds the Xtensa `firmware` on the host and fails).

The kittest UI harness pattern: build a single view in a closure via `Harness::builder().build_ui_state(closure,
state)`, fold drained intents into a `HarnessState`, then `harness.get_by_label("…").click()` and assert on the
recorded intents. Hand-painted label-less widgets need a rect side-channel (see `slider_rect_probe`). See
[[skirnir-stream-time-eta-seam]] for the ETA wiring this exercised.
