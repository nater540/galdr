//! [`ToolDatabase`] — a user-global, persistent tool library the CAM layer reads defaults from.
//!
//! FlatCAM kept a tool DB of diameters, feeds, speeds, and per-operation defaults. This is the same idea as a
//! versioned `serde` store, deliberately *separate* from any one project file: it is a user-global library that
//! seeds new operations. Each entry carries a diameter plus per-operation default bundles; the builder methods hand
//! back the very same operator-facing specs ([`IsolationSpec`]/[`DrillSpec`]) that [`crate::CamOperation`] stores,
//! so "read a default from the tool DB" and "set it on an object" speak one vocabulary.

use crate::error::{ProjectError, Result};
use crate::id::ToolId;
use crate::object::{DirectionSpec, DrillSpec, IsolationSpec};
use crate::serde_ext;
use eitri_core::Length;
use serde::{Deserialize, Serialize};

/// The current tool-database schema version. Versioned independently of the project schema.
pub const TOOL_DB_SCHEMA_VERSION: u32 = 1;

/// The persistent tool library.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolDatabase {
  tools: Vec<ToolEntry>,
  /// Monotonic id allocator; never reused within a database.
  next_id: u64,
}

impl Default for ToolDatabase {
  fn default() -> ToolDatabase {
    ToolDatabase::new()
  }
}

impl ToolDatabase {
  /// An empty tool database.
  pub fn new() -> ToolDatabase {
    ToolDatabase { tools: Vec::new(), next_id: 1 }
  }

  /// Add a tool, allocating and returning its stable id. The supplied entry's placeholder id is overwritten.
  pub fn add(&mut self, mut entry: ToolEntry) -> ToolId {
    let id = ToolId(self.next_id);
    self.next_id += 1;
    entry.id = id;
    self.tools.push(entry);
    id
  }

  /// Borrow a tool by id.
  pub fn get(&self, id: ToolId) -> Option<&ToolEntry> {
    self.tools.iter().find(|t| t.id == id)
  }

  /// Borrow a tool by name.
  pub fn by_name(&self, name: &str) -> Option<&ToolEntry> {
    self.tools.iter().find(|t| t.name == name)
  }

  /// Replace the entry with `id` in place, keeping its id and list position (the supplied entry's placeholder id is
  /// overwritten). Returns whether an entry with that id existed; an unknown id mutates nothing.
  pub fn update(&mut self, id: ToolId, mut entry: ToolEntry) -> bool {
    match self.tools.iter_mut().find(|t| t.id == id) {
      Some(slot) => {
        entry.id = id;
        *slot = entry;
        true
      }
      None => false,
    }
  }

  /// Remove a tool by id, returning it if present.
  pub fn remove(&mut self, id: ToolId) -> Option<ToolEntry> {
    let index = self.tools.iter().position(|t| t.id == id)?;
    Some(self.tools.remove(index))
  }

  /// Iterate tools in insertion order.
  pub fn iter(&self) -> impl Iterator<Item = &ToolEntry> {
    self.tools.iter()
  }

  /// The number of tools.
  pub fn len(&self) -> usize {
    self.tools.len()
  }

  /// Whether the database holds no tools.
  pub fn is_empty(&self) -> bool {
    self.tools.is_empty()
  }
}

/// A single tool: a diameter plus per-operation default bundles.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ToolEntry {
  /// Stable id assigned by the owning [`ToolDatabase`].
  pub id: ToolId,
  /// Human-facing tool name (e.g. `"0.2mm end mill"`).
  pub name: String,
  /// Tool diameter (stored as canonical millimetres).
  #[serde(with = "serde_ext::length_mm")]
  pub diameter: Length,
  /// Default isolation parameters seeded from this tool.
  pub isolation: IsolationDefaults,
  /// Default drilling parameters seeded from this tool.
  pub drilling: DrillDefaults,
}

impl ToolEntry {
  /// Build the isolation spec this tool seeds: its diameter plus the isolation defaults.
  pub fn isolation_spec(&self) -> IsolationSpec {
    IsolationSpec {
      tool_diameter: self.diameter.as_mm(),
      passes: self.isolation.passes,
      overlap: self.isolation.overlap,
      combine: self.isolation.combine,
      direction: self.isolation.direction,
    }
  }

  /// Build the drilling spec this tool seeds.
  pub fn drill_spec(&self) -> DrillSpec {
    DrillSpec {
      depth: self.drilling.depth,
      feed: self.drilling.feed,
      retract: self.drilling.retract,
      peck: self.drilling.peck,
      dwell: self.drilling.dwell,
    }
  }
}

/// Per-tool isolation defaults (everything except the diameter, which is the tool's own).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct IsolationDefaults {
  /// Default number of concentric passes.
  pub passes: usize,
  /// Default pass overlap fraction in `[0, 1)`.
  pub overlap: f64,
  /// Whether to combine overlapping same-pass rings by default.
  pub combine: bool,
  /// Default milling direction.
  pub direction: DirectionSpec,
}

impl Default for IsolationDefaults {
  fn default() -> IsolationDefaults {
    IsolationDefaults { passes: 1, overlap: 0.0, combine: false, direction: DirectionSpec::Climb }
  }
}

/// Per-tool drilling defaults.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DrillDefaults {
  /// Default total drill depth as a positive magnitude below the surface (millimetres); the emitter negates it to Z.
  pub depth: f64,
  /// Default plunge feed (mm/min).
  pub feed: f64,
  /// Default retract height (millimetres).
  pub retract: f64,
  /// Default peck increment (millimetres); `None` for a single plunge.
  pub peck: Option<f64>,
  /// Default bottom-of-hole dwell (seconds); `None` for no dwell.
  pub dwell: Option<f64>,
}

impl Default for DrillDefaults {
  fn default() -> DrillDefaults {
    DrillDefaults { depth: 1.6, feed: 100.0, retract: 2.0, peck: None, dwell: None }
  }
}

/// The versioned tool-database envelope.
#[derive(Debug, Serialize, Deserialize)]
struct ToolDbFile {
  schema_version: u32,
  database: ToolDatabase,
}

/// Serialize a tool database to pretty-printed JSON at the current tool-DB schema version.
pub fn save_tool_db(database: &ToolDatabase) -> Result<String> {
  let file = ToolDbFile { schema_version: TOOL_DB_SCHEMA_VERSION, database: database.clone() };
  serde_json::to_string_pretty(&file).map_err(|e| ProjectError::Serialize(e.to_string()))
}

/// Load a tool database from JSON, rejecting a missing or unsupported schema version.
pub fn load_tool_db(json: &str) -> Result<ToolDatabase> {
  let value: serde_json::Value =
    serde_json::from_str(json).map_err(|e| ProjectError::Deserialize(e.to_string()))?;
  let version = value
    .get("schema_version")
    .and_then(serde_json::Value::as_u64)
    .map(|v| v as u32)
    .ok_or_else(|| ProjectError::Deserialize("missing or non-integer 'schema_version' field".to_string()))?;
  if version != TOOL_DB_SCHEMA_VERSION {
    return Err(ProjectError::UnsupportedVersion { found: version, supported: TOOL_DB_SCHEMA_VERSION });
  }
  let mut file: ToolDbFile =
    serde_json::from_value(value).map_err(|e| ProjectError::Deserialize(e.to_string()))?;
  // Migrate databases saved under the old signed-Z convention: drill depth is now a positive magnitude, so fold any
  // legacy negative into its magnitude. Without this an old `-1.8` would be clamped to `0.01` by the editor's
  // positive-only range (silently destroying the depth) — and air-drill until then.
  for tool in &mut file.database.tools {
    tool.drilling.depth = tool.drilling.depth.abs();
  }
  Ok(file.database)
}
