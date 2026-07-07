//! Datum finding — the everyday work-zero probing an operator expects from ioSender (edge, corner, Z surface).
//!
//! The subsystem follows the established skirnir split: a **pure logic core** (host-tested, no egui/no I/O)
//! composed of three small modules, driven by the shell's [`crate::app::shell`] orchestration and rendered by a
//! thin `views.rs` panel:
//!
//! - [`touch`] — the two-stage `G38.3` latch touch as a pure line builder, plus the persisted [`touch::ProbeParams`]
//!   bench parameters (the non-rotary sibling of [`crate::app::rotary_probe`]).
//! - [`comp`] — the single tip-radius compensation rule, `edge_coord = contact + (Ø/2)·af`, shared by every op.
//! - [`finder`] — the ordered wizard state machine for single-edge and corner (in + out, all four) datums,
//!   mirroring [`crate::app::rotary_center::WizardState`].
//!
//! Everything here reuses the existing DOC-11 probe latch ([`crate::app::view_state::ProbeOp`]) and the shared
//! push-or-poll fallback ([`crate::app::probe_flow::await_action`]); a `success:false` (`G38.3`'s software miss)
//! maps to [`crate::app::view_state::ProbeOutcome::Failure`] and aborts the run without writing an offset. Datums
//! are written position-independently with `G10 L2 P0 <axis><machine-coord>`. The algorithms mirror ioSender /
//! OpenCNCPilot (MIT), reimplemented clean-room in Rust.

pub mod comp;
pub mod finder;
pub mod touch;

pub use comp::edge_coord;
pub use finder::{Corner, DatumState, DatumStep, DatumTarget};
pub use touch::{ProbeParams, Touch, touch_lines};
