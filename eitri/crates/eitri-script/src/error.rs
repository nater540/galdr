//! The `eitri-script` error type and `Result` alias.
//!
//! The command layer composes every other Eitri crate, so its failures are a superset: a project-model failure
//! (unknown id, duplicate name, bad schema), an engine failure (a parser or a cancelled CAM op), a filesystem
//! failure from a path-based convenience wrapper, or a command-level misuse (asking to isolate a drill file, naming
//! a postprocessor that is not registered). One typed error funnels them all so the Rhai binding can map a single
//! `Result` onto a script error without caring which layer failed.

use eitri_project::ObjectKind;
use thiserror::Error;

/// Everything a [`crate::Session`] command can fail with.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum ScriptError {
  /// A project-model operation (collection lookup, rename, persistence) failed.
  #[error("{0}")]
  Project(#[from] eitri_project::ProjectError),

  /// An engine operation (parsing, a CAM op, geometry) failed. This carries [`eitri_core::Error::Cancelled`], so a
  /// cancelled long command surfaces here and — through the binding — aborts the running script.
  #[error("{0}")]
  Engine(#[from] eitri_core::Error),

  /// A command referenced an object id that no object in the session holds.
  #[error("no object with id {0}")]
  UnknownObject(u64),

  /// A command was applied to an object of the wrong kind — e.g. isolating an Excellon drill file, or asking for the
  /// G-code of a Gerber. The message names both the kind required and the kind found.
  #[error("object {id} is a {actual:?}, but this command needs {expected}")]
  WrongKind {
    /// The offending object's id.
    id: u64,
    /// A human phrase for the kind(s) the command accepts (e.g. `"a Gerber or Geometry object"`).
    expected: String,
    /// The kind the object actually is.
    actual: ObjectKind,
  },

  /// A command named a postprocessor dialect that is not in the session's registry.
  #[error("unknown postprocessor dialect '{0}'")]
  UnknownDialect(String),

  /// A command argument was outside its valid range or otherwise malformed (e.g. an unrecognized milling-direction
  /// name). The message describes the specific problem.
  #[error("invalid argument: {0}")]
  InvalidArgument(String),

  /// A path-based convenience wrapper hit a filesystem error; the path is kept for a legible message.
  #[error("i/o error for '{path}': {source}")]
  Io {
    /// The path that was being read or written.
    path: String,
    /// The underlying filesystem error.
    source: std::io::Error,
  },
}

/// The crate-wide result alias.
pub type Result<T> = std::result::Result<T, ScriptError>;

impl ScriptError {
  /// Build an [`ScriptError::Io`] tagged with the path that failed.
  pub(crate) fn io(path: impl Into<String>, source: std::io::Error) -> ScriptError {
    ScriptError::Io { path: path.into(), source }
  }
}
