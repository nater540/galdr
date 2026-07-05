//! Persistence for the user-global tool database — the sibling of [`crate::config`], stored as versioned JSON
//! under the same OS config directory (`tools.json`). The tool library is user-global (not per-project), so it
//! lives beside the config rather than inside a project file.
//!
//! Reads downgrade to an empty database with a surfaced notice (a missing file is normal on first run; an
//! unreadable one is reported, never a panic). Writes go through [`crate::store::atomic_write`] so an
//! interrupted save cannot corrupt the library. The serialization itself is the engine's versioned
//! [`eitri_project::save_tool_db`]/[`eitri_project::load_tool_db`] — the app owns only where the bytes land.

use std::path::PathBuf;

use eitri_project::{ToolDatabase, load_tool_db, save_tool_db};

use crate::store;

/// The tool-library file name under the shared config directory.
const TOOL_DB_FILE: &str = "tools.json";

/// The absolute path the tool database persists to, or `None` when no config directory resolves on this
/// platform (the app then runs with an in-memory library that cannot be saved).
pub fn path() -> Option<PathBuf> {
  store::config_dir().ok().map(|dir| dir.join(TOOL_DB_FILE))
}

/// Load the tool database from disk. A missing file yields an empty database with no notice (first run); an
/// unresolvable path or an unreadable/invalid file yields an empty database plus a notice the caller logs.
pub fn load() -> (ToolDatabase, Option<String>) {
  let Some(path) = path() else {
    return (ToolDatabase::new(), None);
  };
  match std::fs::read_to_string(&path) {
    Ok(json) => match load_tool_db(&json) {
      Ok(db) => (db, None),
      Err(err) => (
        ToolDatabase::new(),
        Some(format!("tool database at {} is unreadable ({err}); starting with an empty library", path.display())),
      ),
    },
    Err(err) if err.kind() == std::io::ErrorKind::NotFound => (ToolDatabase::new(), None),
    Err(err) => (
      ToolDatabase::new(),
      Some(format!("could not read the tool database at {} ({err}); starting with an empty library", path.display())),
    ),
  }
}

/// Persist the tool database to disk atomically, returning a human-readable error on failure (surfaced in the
/// log, never a panic).
pub fn save(database: &ToolDatabase) -> Result<(), String> {
  let path = path().ok_or_else(|| "could not resolve a config directory for the tool database".to_string())?;
  let json = save_tool_db(database).map_err(|err| err.to_string())?;
  store::atomic_write(&path, json.as_bytes()).map_err(|err| err.to_string())
}

#[cfg(test)]
mod tests {
  use super::*;
  use eitri_core::Length;
  use eitri_project::{DrillDefaults, IsolationDefaults, ToolEntry, ToolId};

  #[test]
  fn a_saved_library_round_trips_through_the_engine_serialization() {
    // The app owns only the bytes' location; this pins that save→load of a populated library preserves it (the
    // engine's versioned envelope is tested in eitri-project — here we guard the app-side wrapper).
    let mut db = ToolDatabase::new();
    db.add(ToolEntry {
      id: ToolId(0),
      name: "0.2mm end mill".to_string(),
      diameter: Length::from_mm(0.2),
      isolation: IsolationDefaults::default(),
      drilling: DrillDefaults::default(),
    });
    let json = save_tool_db(&db).expect("serializes");
    let back = load_tool_db(&json).expect("deserializes");
    assert_eq!(back.len(), 1);
    assert_eq!(back.iter().next().map(|t| t.name.as_str()), Some("0.2mm end mill"));
  }
}
