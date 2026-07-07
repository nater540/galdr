//! The workspace-wide error type and `Result` alias.
//!
//! Every fallible Eitri operation returns `eitri_core::Result<T>`. Downstream crates add context by wrapping
//! their own failures into these variants (or, where they own richer detail, into `Error::Geometry` /
//! `Error::Parse` with a descriptive message) so a single error type flows through the whole engine.

use thiserror::Error;

/// The one error type shared across the Eitri engine crates.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum Error {
  /// A long-running operation observed its cancellation token and stopped early.
  #[error("operation cancelled")]
  Cancelled,

  /// A geometry backend (offsetting, boolean ops, simplification) reported a failure.
  #[error("geometry backend error: {0}")]
  Geometry(String),

  /// Geometry was structurally invalid for the requested operation (empty ring, degenerate polygon, etc.).
  #[error("invalid geometry: {0}")]
  InvalidGeometry(String),

  /// A unit conversion or an out-of-range/invalid length was requested.
  #[error("unit error: {0}")]
  Unit(String),

  /// A parser (Gerber, Excellon, import) could not make sense of its input.
  #[error("parse error: {0}")]
  Parse(String),

  /// An unsupported or not-yet-implemented backend/feature path was reached.
  #[error("unsupported: {0}")]
  Unsupported(String),

  /// An underlying I/O failure.
  #[error("i/o error: {0}")]
  Io(#[from] std::io::Error),
}

/// The workspace-wide result alias. Use this instead of hand-writing `Result<T, eitri_core::Error>`.
pub type Result<T> = std::result::Result<T, Error>;
