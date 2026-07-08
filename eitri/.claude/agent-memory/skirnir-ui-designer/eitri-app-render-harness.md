---
name: eitri-app-render-harness
description: eitri-app offscreen render facts — kittest render() to arbitrary PNGs, tutorial generator, fit-includes-origin-rapid wart, fmt not applicable
metadata:
  type: project
---

Facts learned building the mill-a-pcb tutorial screenshot generator (`crates/eitri-app/src/app/tutorial_shots.rs`, `#[ignore]`d, writes to the PARENT repo's `docs/images/mill-a-pcb/`):

- To render the shell to a PNG at an arbitrary path (not a `tests/snapshots` baseline), use `harness.render() -> Result<image::RgbaImage, String>` (egui_kittest 0.35 wgpu feature) + the `image` crate's `save`; `image = 0.25` is a dev-dep added for this. Bottom of a body-only render (e.g. tool-DB dialog) is transparent below the content — crop by last non-zero-alpha row.
- **Fit wart**: a CNC job's scene entry includes the G-code importer's synthetic start-at-(0,0) rapid in `bounds` (scene.rs extends bounds with rapids). For a KiCad-frame board (starter-* sits at X≈119–137, Y≈−86…−110) the fit and the selected job's rim zoom out to frame the empty origin. Live app still does this; the tutorial generator strips `from == [0,0]` rapids and retightens bounds. A real fix (bounds from cuts only) would shift every committed job-bearing snapshot baseline.
- The starter-* fixtures are a genuinely minimal 2-component board: 4 pads, 1 trace, 2 thermal-relief rosettes, 4 PTH holes, ~18×24 mm outline. Copper parses to 3 polygons.
- `cargo fmt --check` is NOT green in the eitri workspace (2-space .editorconfig vs rustfmt defaults, repo-wide) — definition of done there is `cargo test` + `clippy --workspace --all-targets` under `-D warnings` only.
- Dock height in a harness shot: `state.split = CentralSplit::new(0.42)` shows ~9–12 G-code preview lines at DEFAULT_SIZE (default 0.25 shows ~5).
- Missing en-US.ftl keys: `op-paint`/`op-noncopper`/`op-cutout`/`op-panelize`/`op-mirror` op labels (used by ops.rs `label key`) are absent from `assets/i18n/en-US.ftl` — a cutout completion log line would render a raw key. (Resolved upstream at some point — both locales now carry them; the parity test enforces key sets match.)
- **Committed GPU snapshot baselines do NOT reproduce on this Mac** (2026-07-08): all 13 shell/tool-db baselines fail with small pixel diffs (~500–1100 px) even on a stashed clean tree — environment/AA drift plus pre-existing uncommitted eitri working-tree changes (tool_db.rs, session/gcode/project edits, modified starter fixtures). Renders ARE deterministic run-to-run on one tree, so hash-compare `.new.png` across trees to prove a change is pixel-neutral instead of trusting the baseline diff.
- Datum UI (added 2026-07-08): `OpChoice::Datum` in the params-panel op picker; reference object = current Gerber/Geometry selection; grid widget + section in views.rs (`datum_grid`/`datum_section`, `datum_anchor_fraction` maps world corners to screen fractions Y-flipped); shell snapshots `session.datum()` into `UiState.datum` in `refresh_from_session`; canvas `origin_cross` draws the quiet cross at native `[0,0]` and a ringed 12px marker when a datum is placed (keeps old baselines pixel-identical). New baseline: `shell_datum_panel.png`.
- **Job-preview frame wart**: `scene::build_scene` previews CNC jobs by re-importing their emitted G-code, and the emitter subtracts the datum — so a job generated AFTER a datum is set renders near world (0,0), displaced from the native-frame copper. Needs `CncJobObject` to record its emit origin (engine change, skirnir-engineer). Tutorial shot 09 sets the datum after generating jobs to sidestep it.
