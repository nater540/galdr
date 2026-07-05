//! `eitri-core` — shared primitives for the Eitri CAM engine.
//!
//! Units, coordinate transforms, precision constants, the workspace error type, and the progress/cancellation
//! seams. No CAM logic and no I/O live here; every other Eitri crate depends on this one.
//!
//! Algorithms and structure derive from FlatCAM's `camlib.py` utility layer (unit handling and the
//! scale/offset/mirror/rotate/skew helpers on its `Geometry` base class), reworked into checked Rust types — see
//! `docs/eitri-porting-plan.md` §3.

#![forbid(unsafe_code)]

pub mod coord;
pub mod error;
pub mod precision;
pub mod progress;
pub mod transform;
pub mod units;

pub use coord::{CoordDecodeError, ZeroOmission, decode_zero_omitted};
pub use error::{Error, Result};
pub use precision::{CHORD_TOLERANCE_MM, GCODE_DECIMALS, GEOM_EPSILON_MM, INTEGER_SCALE, SIMPLIFY_TOLERANCE_MM};
pub use progress::{CancelToken, ProgressEvent, ProgressReporter};
pub use transform::Affine;
pub use units::{Length, MM_PER_INCH, Unit};
