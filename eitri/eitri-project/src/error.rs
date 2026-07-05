//! The `eitri-project` error type and `Result` alias.
//!
//! Persistence, collection lookups, and re-derivation are all fallible; they funnel through one crate error.
//! Failures from the engine parsers (re-parsing embedded Gerber/Excellon during [`crate::Project::hydrate`]) are
//! surfaced as [`ProjectError::Engine`] so a caller can still distinguish a corrupt source from a bad project file.

use crate::id::ObjectId;
use thiserror::Error;

/// Everything that can go wrong loading, mutating, or re-deriving a project or tool database.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum ProjectError {
  /// Serializing a project or tool database to JSON failed.
  #[error("failed to serialize: {0}")]
  Serialize(String),

  /// The on-disk JSON was malformed or did not match the schema for its declared version.
  #[error("failed to deserialize: {0}")]
  Deserialize(String),

  /// The file declared a schema version this build cannot read (either older-than-v1/zero or newer than we support).
  #[error("unsupported schema version {found} (this build reads versions 1..={supported})")]
  UnsupportedVersion {
    /// The `schema_version` read from the file.
    found: u32,
    /// The highest schema version this build understands.
    supported: u32,
  },

  /// An object name collided with one already in the collection (names are the unique human key).
  #[error("an object named '{0}' already exists")]
  DuplicateName(String),

  /// No object in the collection has the requested name.
  #[error("no object named '{0}'")]
  UnknownName(String),

  /// No object in the collection has the requested id.
  #[error("no object with id {0}")]
  UnknownId(ObjectId),

  /// A group name collided with one already in the collection.
  #[error("a group named '{0}' already exists")]
  DuplicateGroup(String),

  /// No group in the collection has the requested name.
  #[error("no group named '{0}'")]
  UnknownGroup(String),

  /// An engine crate (a re-parse during hydration) reported a failure.
  #[error("engine error: {0}")]
  Engine(#[from] eitri_core::Error),
}

/// The crate-wide result alias.
pub type Result<T> = std::result::Result<T, ProjectError>;
