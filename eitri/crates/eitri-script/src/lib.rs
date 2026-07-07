//! `eitri-script` — the scripting / command surface over the Eitri engine (plan §10).
//!
//! FlatCAM embedded a Tcl console whose `TclCommand`s (`open_gerber`, `isolate`, `cncjob`, `write_gcode`, …) drove
//! the engine. Eitri has no reason to carry Tcl. This crate replaces it with two layers:
//!
//! 1. A **typed Rust command API** — [`Session`] — that is the real, load-bearing interface: a plain, fully
//!    unit-testable surface over `eitri-project` + `eitri-cam` + `eitri-gcode` + `eitri-import` covering open/import,
//!    every CAM operation, G-code output, object management with undo, project persistence, and the tool database.
//!    It has **no** dependency on any scripting runtime, so all correctness-critical logic is testable in Rust.
//! 2. A thin **Rhai binding** ([`bindings`], behind the default-on `scripting` feature) that registers [`Session`]
//!    and the command surface into a Rhai engine so scripts can drive the whole pipeline, mapping every
//!    [`ScriptError`] onto a Rhai runtime error rather than a panic.
//!
//! The typed command types (the operator-facing `*Spec` parameter structs, the cut-job settings, and the object /
//! tool ids) are re-exported here so a caller constructs commands from one place regardless of which engine crate
//! actually owns each type.

#![forbid(unsafe_code)]

pub mod error;
pub mod session;

#[cfg(feature = "scripting")]
pub mod bindings;

pub use error::{Result, ScriptError};
pub use session::{DEFAULT_DIALECT, Session};

// The operator-facing command vocabulary, re-exported so scripts and callers build every command from `eitri_script`.
pub use eitri_project::{
  BoundarySpec, CamOperation, CutoutOutlineSpec, CutoutSpec, DirectionSpec, DrillSpec, IsolationSpec, MirrorLineSpec,
  NonCopperSpec, ObjectId, ObjectKind, PaintSpec, PaintStrategySpec, PanelizeSpec, SpacingSpec, TabPlacementSpec,
  ToolEntry, ToolId, TwoSidedSpec,
};
pub use eitri_project::{DrillDefaults, IsolationDefaults, ToolDatabase};
pub use eitri_gcode::{DrillJob, IsolationJob};
pub use eitri_cam::{FilmKind, FilmParams};
pub use eitri_core::{Affine, CancelToken, ProgressEvent, ProgressReporter};

#[cfg(feature = "scripting")]
pub use bindings::{ScriptSession, eval, new_engine};
