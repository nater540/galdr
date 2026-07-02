---
name: snapshot-harness
description: Skirnir's GPU snapshot test suite — where it lives, how to run/regenerate, and its harness quirks
metadata:
  type: project
---

Skirnir has an image-snapshot suite in `crates/skirnir/src/app/snapshot_test.rs` (added 2026-07-01), rendering the
FULL shell + close-ups offscreen via egui_kittest's `wgpu`+`snapshot` features (enabled on the dev-dependency in
`crates/skirnir/Cargo.toml`). Baselines are committed PNGs under `crates/skirnir/tests/snapshots/`.

**Why:** visual regressions (spacing, clipping, theme drift) don't show in interaction tests; the baselines are both
the regression net and the review artifact (Read the PNGs directly — they render as images).

**How to apply:**
- Run: `cargo test -p skirnir -- --ignored snapshot` (tests are `#[ignore]`d so plain `cargo test`/CI never needs a GPU).
- Regenerate: `UPDATE_SNAPSHOTS=1 cargo test -p skirnir -- --ignored snapshot`. Renders are deterministic on this
  machine (a plain re-run passes).
- The shared whole-window layout mirror is `ui_test::shell_layout` + `build_shell_harness` (fonts + `shell::apply_theme`
  applied to the ctx so pixels match the real window; `apply_theme` was made `pub(crate)` for this).
- kittest hosts the closure in a default `CentralPanel` with an **8px inner margin** — full-window shots carry that
  border, and a bar-only harness must be sized `content + 16px` or the bottom clips.
- Close-ups use `.with_pixels_per_point(2.0)` for glyph/spacing review (no ImageMagick/PIL on this machine — render
  sharp instead of upscaling).
- Fixture states use obviously-fake data (`/dev/cu.usbmodemFAKE1`, a 9-line fixture square). A DRO fixture status
  needs a `WCO:` field or the default WPos view shows `—` dashes.

Related: [[egui-034-layout-gotchas]]
