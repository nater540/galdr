//! `eitri-gerber` — an RS-274X Gerber parser.
//!
//! Parses Gerber (RS-274X) into a resolved geometry model: an aperture table plus the copper polygons drawn with
//! them (flashes, stroked draws, and filled regions), accumulated by polarity into one copper `MultiPolygon` that
//! downstream CAM consumes. See docs/eitri-porting-plan.md §4. Provenance: FlatCAM's `camlib` Gerber handling and
//! its `ApertureMacro` interpreter, reimplemented clean-room on the `eitri-geo` backend.
//!
//! Entry point: [`parse_gerber`]. It threads an [`eitri_core::ProgressReporter`] and
//! [`eitri_core::CancelToken`] so a large import stays responsive and interruptible.

#![forbid(unsafe_code)]

pub mod aperture;
pub mod error;
pub mod expr;
pub mod format;
pub mod geometry;
pub mod lexer;
pub mod macros;
pub mod parser;

pub use aperture::Aperture;
pub use error::{GerberError, Result};
pub use format::{CoordinateFormat, Notation, ZeroOmission};
pub use parser::{GerberImage, parse_gerber};
