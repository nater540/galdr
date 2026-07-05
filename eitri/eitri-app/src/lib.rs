//! `eitri-app` — the egui/eframe front-end over the Eitri CAM engine.
//!
//! The crate is a THIN consumer of the engine: operator intent flows into [`eitri_script::Session`] (the typed
//! command API), long commands run on a worker thread with progress/cancel via `eitri_core`'s seams, and the
//! canvas renders what the engine computed (tessellation from `eitri_geo::triangulate`, toolpath previews from
//! `eitri_import::import_gcode`). **No CAM or geometry logic lives here** — see `docs/eitri-porting-plan.md` §11.
//!
//! The UI mirrors the Galdr workspace's `skirnir` sender: the same dark near-black theme tokens, the Fluent
//! i18n layer with the [`tr!`] macro, the hand-editable JSON [`config`] with layered themes, the vendored
//! Roboto/JetBrains Mono font stack, and an `egui_tiles`-hosted central split.

pub mod app;
pub mod config;
pub mod i18n;
pub mod store;
pub mod tool_store;
