//! `eitri-gcode` — NC generation and the postprocessor trait.
//!
//! This crate turns the toolpath / operation models from `eitri-cam` into real G-code. It is built in two cleanly
//! separated halves (see `docs/eitri-porting-plan.md` §8):
//!
//! - The **emitter** ([`emit`]) walks an isolation toolpath or a drill plan and decides *motions* — rapid to a
//!   start, plunge, cut (linear or arc), lift, multi-depth passes, manual peck cycles, tool changes.
//! - The **postprocessor** ([`post::Postprocessor`]) renders each motion as text for a specific controller dialect.
//!   [`GrblHal`] is the default and is byte-for-byte conformant with the grblHAL / Skirnir contract; [`Generic`] is a
//!   second, deliberately different dialect that proves the trait seam is swappable.
//!
//! [`conformance::check_grbl_conformance`] validates rendered output against that contract, so the highest-value
//! test is a direct assertion that the emitter cannot produce something the firmware would reject.
//!
//! The provenance is FlatCAM's `CNCjob` class and its `preprocessors/` directory; no FlatCAM code is used, only its
//! architecture (per-controller hook modules → a Rust trait). A G-code read-back **lexer** is planned to live here
//! for `eitri-import` (Phase 6) to reuse; it is deferred until that phase consumes it.

#![forbid(unsafe_code)]

pub mod arc;
pub mod conformance;
pub mod emit;
pub mod format;
pub mod post;
mod post_generic;
mod post_grbl;
pub mod program;

pub use arc::{ArcDir, ArcOffset, bulge_to_arc};
pub use conformance::{Reason, Violation, check_grbl_conformance, is_grbl_conformant};
pub use emit::{CutRing, DrillJob, IsolationJob, Segment, depth_steps, emit_drilling, emit_isolation};
pub use format::{CommentStyle, LineEnding, OutputFormat};
pub use post::{ArcMove, Axes, JobContext, Postprocessor, Registry, Spindle, ToolChange};
pub use post_generic::Generic;
pub use post_grbl::GrblHal;
pub use program::Program;
