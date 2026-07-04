//! The Gerber parser's typed error, with a `From` bridge into the workspace-wide `eitri_core::Error`.
//!
//! Parser errors carry a source line for diagnostics; geometry failures from `eitri-geo` fold in via `#[from]`.
//! `From<GerberError> for eitri_core::Error` lets callers higher in the engine propagate with `?` against the
//! shared error type, as the porting plan asks (§12).

use thiserror::Error;

/// Everything that can go wrong parsing a Gerber file.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum GerberError {
  /// A malformed or unexpected construct at a specific source line.
  #[error("gerber syntax error on line {line}: {message}")]
  Syntax {
    /// 1-based source line number.
    line: usize,
    /// What was wrong.
    message: String,
  },

  /// A construct that is valid Gerber but not yet supported by this parser.
  #[error("unsupported gerber construct on line {line}: {message}")]
  Unsupported {
    /// 1-based source line number.
    line: usize,
    /// The construct that is not handled.
    message: String,
  },

  /// A coordinate word appeared before the `FS` format-specification block that decodes it.
  #[error("coordinate seen before the format specification (FS) was set")]
  MissingFormat,

  /// An operation referenced an aperture code that was never defined.
  #[error("aperture D{0} was used before it was defined")]
  UndefinedAperture(u32),

  /// A geometry backend operation failed while assembling copper.
  #[error("geometry error: {0}")]
  Geometry(#[from] eitri_core::Error),

  /// The parse was cancelled through its cancellation token.
  #[error("gerber parse cancelled")]
  Cancelled,
}

impl From<GerberError> for eitri_core::Error {
  fn from(err: GerberError) -> eitri_core::Error {
    match err {
      GerberError::Geometry(inner) => inner,
      GerberError::Cancelled => eitri_core::Error::Cancelled,
      other => eitri_core::Error::Parse(other.to_string()),
    }
  }
}

/// Convenience result alias for the Gerber parser.
pub type Result<T> = std::result::Result<T, GerberError>;
