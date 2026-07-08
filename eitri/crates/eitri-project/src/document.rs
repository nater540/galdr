//! Versioned persistence: the on-disk envelope, save/load, and the migration seam.
//!
//! FlatCAM saved projects as pickled Python — unportable and a code-execution liability. Eitri replaces that with a
//! versioned `serde` schema, JSON in v1 (§9, §14). The envelope always carries an explicit `schema_version`, and
//! loading is a three-step pipeline: parse to a generic value, **upgrade** it from its on-disk version to the current
//! one, then deserialize the typed document. The upgrade step is the migration hook — today v1 is the only version,
//! so it is the identity for v1 and a clean error for anything else, but the shape is built so a future `v1 -> v2`
//! field migration is a single added arm, never a format-breaking rewrite.
//!
//! ## JSON, not a binary format (v1)
//!
//! JSON is chosen for v1 because a CAM project file benefits enormously from being human-readable and diffable while
//! the schema is still young: bugs are eyeballed, hand-edited, and version-controlled. The envelope and the typed
//! documents are serializer-agnostic, so a compact binary format (bincode/CBOR) can be added later behind the same
//! `schema_version` gate without disturbing the object model.

use crate::collection::ObjectCollection;
use crate::error::{ProjectError, Result};
use crate::project::Project;
use serde::{Deserialize, Serialize};
use serde_json::Value;

/// The current on-disk schema version. Bump this — and add an `upgrade` arm — for any breaking schema change.
pub const SCHEMA_VERSION: u32 = 1;

/// The versioned project envelope. `schema_version` is written first and checked first on load.
#[derive(Debug, Serialize, Deserialize)]
struct ProjectFile {
  /// Schema version of this file.
  schema_version: u32,
  /// Project name.
  name: String,
  /// The object collection.
  collection: ObjectCollection,
  /// The work-zero (datum) offset `(x, y, z)` the CAM output is posted relative to. `#[serde(default)]` so a project
  /// written before the datum existed loads as the native frame without a schema bump.
  #[serde(default)]
  origin: (f64, f64, f64),
  /// The job's stock/work-zero setup, or `None`. `#[serde(default)]` for the same forward-compat reason.
  #[serde(default)]
  stock: Option<crate::Stock>,
}

/// Serialize a project to pretty-printed JSON at the current schema version. The re-derivable parse caches on
/// Gerber/Excellon objects are `#[serde(skip)]` and are dropped here, so the output holds only the authoritative
/// source, geometry, and parameters.
pub fn save_project(project: &Project) -> Result<String> {
  let file = ProjectFile {
    schema_version: SCHEMA_VERSION,
    name: project.name.clone(),
    collection: project.collection.clone(),
    origin: project.origin,
    stock: project.stock,
  };
  serde_json::to_string_pretty(&file).map_err(|e| ProjectError::Serialize(e.to_string()))
}

/// Load a project from JSON, dispatching on `schema_version` and running any migrations up to the current schema.
/// The returned project is *un-hydrated* — call [`Project::hydrate`] to re-derive Gerber/Excellon geometry.
///
/// Fails gracefully (never panics) on malformed JSON, a missing version field, or a version this build cannot read.
pub fn load_project(json: &str) -> Result<Project> {
  let value: Value = serde_json::from_str(json).map_err(|e| ProjectError::Deserialize(e.to_string()))?;
  let version = read_version(&value)?;
  let upgraded = upgrade(version, value)?;
  let file: ProjectFile =
    serde_json::from_value(upgraded).map_err(|e| ProjectError::Deserialize(e.to_string()))?;
  // The stock is the source of truth when present: re-derive the work origin from it rather than trusting the
  // separately-stored `origin`, so a hand-edited or stale JSON can never post G-code at an origin that disagrees
  // with the stock the Setup panel shows. A stock-less project keeps its stored origin (a bare point datum).
  let origin = match &file.stock {
    Some(stock) => stock.origin(),
    None => file.origin,
  };
  Ok(Project { name: file.name, collection: file.collection, origin, stock: file.stock })
}

/// Read the `schema_version` field from a raw document, erroring if it is absent or not an integer.
pub(crate) fn read_version(value: &Value) -> Result<u32> {
  value
    .get("schema_version")
    .and_then(Value::as_u64)
    .map(|v| v as u32)
    .ok_or_else(|| ProjectError::Deserialize("missing or non-integer 'schema_version' field".to_string()))
}

/// Upgrade a raw document from its on-disk `version` to [`SCHEMA_VERSION`], applying each migration step in turn.
///
/// This is the migration hook. With v1 as the only version it validates the range and returns the value unchanged.
/// A version of `0` or one greater than we support is rejected with [`ProjectError::UnsupportedVersion`]. When the
/// schema next changes, bump [`SCHEMA_VERSION`] and add an older-version arm that migrates then recurses, e.g.
/// `1 => upgrade(2, migrate_v1_to_v2(value)?)`, so each step is small and composable.
pub(crate) fn upgrade(version: u32, value: Value) -> Result<Value> {
  match version {
    SCHEMA_VERSION => Ok(value),
    // Older-but-supported versions migrate here and recurse toward SCHEMA_VERSION; none exist yet.
    _ => Err(ProjectError::UnsupportedVersion { found: version, supported: SCHEMA_VERSION }),
  }
}
