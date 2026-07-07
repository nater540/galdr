//! The GUI layer: the egui/eframe application over the engine's [`eitri_script::Session`].
//!
//! Split so the UI stays thin and the logic stays testable:
//!
//! - [`view_state`] — pure, egui-free state the views render: the tree snapshot, the log, the op lifecycle.
//! - [`intent`] — the one-way UI-intent vocabulary the views emit and the shell translates into engine commands
//!   and side effects. Also egui-free and host-testable.
//! - [`ops`] — the off-thread op runner: a long command moves the whole `Session` into a worker, streams
//!   progress over the engine's `ProgressReporter` channel, and hands the session back with the outcome.
//! - [`scene`] — the cached render model built FROM engine outputs (meshes, outlines, preview moves), rebuilt
//!   only when the collection changes so the per-frame paint stays cheap.
//! - [`canvas`] — the pure pan/zoom world↔screen transform and the canvas painter.
//!
//! The egui-touching pieces — [`theme`], [`fonts`], [`metrics`], the [`views`], the [`app_settings`] dialog,
//! the [`dock_tiles`] split, and the eframe [`shell`] — mirror skirnir's equivalents.

pub mod app_settings;
pub mod canvas;
pub mod dock_tiles;
pub mod fonts;
pub mod intent;
pub mod metrics;
pub mod op_drafts;
pub mod ops;
pub mod scene;
pub mod shell;
pub mod theme;
pub mod tool_db;
pub mod view_state;
pub mod views;

pub use shell::{EitriApp, run};
pub use theme::Palette;

/// The egui_kittest interaction-test harness, compiled only for the test build.
#[cfg(test)]
mod ui_test;

/// The `#[ignore]`d GPU image-snapshot suite: offscreen wgpu renders of the shell diffed against committed PNG
/// baselines. See its module docs for how to run/regenerate.
#[cfg(test)]
mod snapshot_test;
