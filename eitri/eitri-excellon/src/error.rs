//! The Excellon parser's typed error, bridged into `eitri_core::Error`.

use thiserror::Error;

/// Everything that can go wrong parsing an Excellon drill file.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum ExcellonError {
  /// A malformed or unexpected construct at a specific source line.
  #[error("excellon syntax error on line {line}: {message}")]
  Syntax {
    /// 1-based source line number.
    line: usize,
    /// What was wrong.
    message: String,
  },

  /// The number format could neither be declared nor inferred, so coordinates cannot be decoded.
  #[error("could not determine the Excellon number format; pass an explicit override")]
  UndeterminedFormat,

  /// A hit referenced a tool that was never defined.
  #[error("tool T{0} was used before it was defined")]
  UndefinedTool(u32),

  /// A geometry backend operation failed.
  #[error("geometry error: {0}")]
  Geometry(#[from] eitri_core::Error),

  /// The parse was cancelled through its cancellation token.
  #[error("excellon parse cancelled")]
  Cancelled,
}

impl From<ExcellonError> for eitri_core::Error {
  fn from(err: ExcellonError) -> eitri_core::Error {
    match err {
      ExcellonError::Geometry(inner) => inner,
      ExcellonError::Cancelled => eitri_core::Error::Cancelled,
      other => eitri_core::Error::Parse(other.to_string()),
    }
  }
}

/// Convenience result alias for the Excellon parser.
pub type Result<T> = std::result::Result<T, ExcellonError>;
