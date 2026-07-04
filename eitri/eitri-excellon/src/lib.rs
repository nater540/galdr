//! `eitri-excellon` — an Excellon drill-file parser.
//!
//! Parses Excellon into a tool table (diameters) plus ordered drill/slot hits, resolving the file's number format
//! (declared or inferred, with an explicit override) — see docs/eitri-porting-plan.md §5. Provenance: FlatCAM's
//! `Excellon` class in `camlib`, reimplemented clean-room.
//!
//! Entry point: [`parse_excellon`]. It threads an [`eitri_core::ProgressReporter`] and
//! [`eitri_core::CancelToken`] so a large drill program stays responsive and interruptible.

#![forbid(unsafe_code)]

pub mod error;
pub mod format;
pub mod parser;

pub use error::{ExcellonError, Result};
pub use format::{NumberFormat, ZeroSuppression};
pub use parser::{DrillHit, ExcellonImage, Tool, parse_excellon};
