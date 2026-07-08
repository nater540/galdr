---
name: eitri-app-mirror
description: eitri-app (eitri/eitri-app) mirrors skirnir's UI systems — where things live, its snapshot suite, and the egui 0.35 side-panel bleed fix
metadata:
  type: project
---

Phase 9 (committed 2026-07-05, `239bed2` on `feat/eitri-phase2`) built `eitri/eitri-app`: the egui front-end for
the Eitri CAM engine, MIRRORING skirnir's patterns by user decision (replicate, do NOT extract a shared crate,
do NOT modify crates/skirnir). Same versions: eframe 0.35 (glow), egui_tiles 0.16, egui_kittest 0.35
(wgpu+snapshot), rfd 0.17.2, fluent 0.17.

**Why:** the two apps must read as one family; the workspaces are deliberately decoupled (eitri is a nested
workspace, built with `--manifest-path eitri/Cargo.toml`).

**How to apply:**
- Mirrored modules: `src/i18n/` (Translator + tr!, en-US + sv-SE), `src/config/` (JSON, layered themes,
  ColorSpec, palette_color_fields! macro), `src/store.rs` (config dir "eitri" + atomic_write), `app/theme.rs`
  (skirnir chrome hexes + CAM tokens: copper/drill/geometry/toolpath_cut yellow), `app/fonts.rs` (vendored
  Roboto/JBMono copied to eitri-app/assets/fonts), `app/metrics.rs`, `app/app_settings.rs`, `app/dock_tiles.rs`.
- Hard boundary honoured: NO CAM/geometry logic in the app. Engine additions made for the UI (all additive):
  `eitri_geo::triangulate`/`TriangleMesh` (ear-cut fills — egui can't fill concave/holed polygons),
  `Postprocessor: Send + Sync`, `Session::set_cancel` (a cancelled CancelToken is one-way and poisons the
  session otherwise).
- Off-thread ops: `app/ops.rs` `SessionSlot` = Home(Session) | Away(RunningOp); the WHOLE Session moves into
  a std::thread per op (its API is &mut self); `poll()` returns trailing ProgressEvents so none die with the
  receiver. Fresh CancelToken + ProgressReporter::channel per spawn.
- Snapshots: `cargo test -p eitri-app -- --ignored snapshot` (UPDATE_SNAPSHOTS=1 to regen), baselines in
  `eitri/eitri-app/tests/snapshots/`; delete `.old.png`/`.diff.png` after regen. Interaction tests share
  `ui_test::build_shell_harness` over `shell::shell_panels` (the ONE layout fn, like skirnir).
- egui 0.35 gotcha (repro'd here): a fixed `Panel::left/right` whose CONTENT measures wider lets the central
  region paint OVER the column's edge — wrap column content in the `views::contained()` clip helper (skirnir's
  fix) and keep dense rows on ~8×3 button_padding. Direction toggles as ComboBox, not side-by-side selectables
  (Swedish overflows).
- No rustfmt in the eitri workspace (editorconfig 2-space only); gates are `RUSTFLAGS="-D warnings" cargo test`
  + `cargo clippy --all-targets`.
- Setup node (2026-07-08): `Selection { Setup, Object(ObjectId) }` in view_state; `ViewState.selected:
  Option<Selection>` (Setup survives set_tree; `selected_object()` adapts old callers). Pinned synthetic tree
  row + right-panel Setup section (StockDraft in UiState → `Intent::SetStock/ClearStock/FitStock` →
  `Session::set_stock/fit_stock_to`). Routine SetStock commits are deliberately UNLOGGED — DragValue fires
  changed() per drag frame and would flood the log; only fit/clear/auto-fit log. First geometry-bearing object
  auto-fits the stock in `pump` (never overrides an existing/cleared-with-geometry setup). Stock block +
  crosshair reuse `palette.origin` (no new config palette field — ThemeOverride churn avoided). scene.rs strips
  the importer's synthetic (0,0) start rapid from job previews (checked pre-datum-offset).
- Known wart (pre-existing, 2026-07-08): at MIN_SIZE in sv-SE the toolbar's left group ("Anpassa vy") collides
  with the right cluster ("Verktyg" + gear) — arrived with the tool-DB toolbar button; visible in shell_sv_min
  and shell_setup_sv_min baselines. Needs a real toolbar overflow fix, not a label tweak.
- Remaining breadth (not built): rename-in-params, tool DB polish; canvas click-select and visibility toggles
  are DONE.

Related: [[snapshot-harness]], [[egui-034-layout-gotchas]], [[egui-disabled-styling]]
