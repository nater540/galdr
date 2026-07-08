//! Persistence for the user-global tool database — the sibling of [`crate::config`], stored as versioned JSON
//! under the same OS config directory (`tools.json`). The tool library is user-global (not per-project), so it
//! lives beside the config rather than inside a project file.
//!
//! Reads downgrade to an empty database with a surfaced notice (a missing file is normal on first run; an
//! unreadable one is reported, never a panic). Writes go through [`crate::store::atomic_write`] so an
//! interrupted save cannot corrupt the library. The serialization itself is the engine's versioned
//! [`eitri_project::save_tool_db`]/[`eitri_project::load_tool_db`] — the app owns only where the bytes land.

use std::path::PathBuf;

use eitri_core::Length;
use eitri_project::{DrillDefaults, IsolationDefaults, ToolDatabase, ToolEntry, ToolId, load_tool_db, save_tool_db};

use crate::store;

/// The tool-library file name under the shared config directory.
const TOOL_DB_FILE: &str = "tools.json";

/// The absolute path the tool database persists to, or `None` when no config directory resolves on this
/// platform (the app then runs with an in-memory library that cannot be saved).
pub fn path() -> Option<PathBuf> {
  store::config_dir().ok().map(|dir| dir.join(TOOL_DB_FILE))
}

/// Load the tool database from disk. A missing file yields the [`seed_library`] starter set with no notice
/// (first run); an unresolvable path or an unreadable/invalid file yields an empty database plus a notice the
/// caller logs (we do NOT overwrite a present-but-corrupt file with the seed).
pub fn load() -> (ToolDatabase, Option<String>) {
  let Some(path) = path() else {
    return (seed_library(), None);
  };
  match std::fs::read_to_string(&path) {
    Ok(json) => match load_tool_db(&json) {
      Ok(db) => (db, None),
      Err(err) => (
        ToolDatabase::new(),
        Some(format!("tool database at {} is unreadable ({err}); starting with an empty library", path.display())),
      ),
    },
    Err(err) if err.kind() == std::io::ErrorKind::NotFound => (seed_library(), None),
    Err(err) => (
      ToolDatabase::new(),
      Some(format!("could not read the tool database at {} ({err}); starting with an empty library", path.display())),
    ),
  }
}

/// A single carbide spiral-flute PCB drill entry: the diameter plus a conservative plunge feed. The drill
/// defaults clear a 1.6 mm board — depth `1.8 mm` (a positive magnitude, into the spoilboard), retract `2.0 mm`, a
/// single plunge. Isolation defaults stay generic (a drill is not an isolation cutter, but the entry carries both).
fn pcb_drill(diameter_mm: f64, plunge_feed: f64) -> ToolEntry {
  ToolEntry {
    id: ToolId(0),
    name: format!("{diameter_mm:.1} mm PCB drill"),
    diameter: Length::from_mm(diameter_mm),
    isolation: IsolationDefaults::default(),
    drilling: DrillDefaults { depth: 1.8, feed: plunge_feed, retract: 2.0, peck: None, dwell: None },
  }
}

/// The starter tool library seeded on first run: a common ten-piece spiral-flute carbide PCB drill set,
/// 0.3–1.2 mm, with plunge feeds scaled to diameter (smaller bits break easily, so they plunge slower). These
/// are sensible defaults, not a saved library — the user edits and Saves to persist their own set.
pub fn seed_library() -> ToolDatabase {
  let mut database = ToolDatabase::new();
  for (diameter_mm, plunge_feed) in [
    (0.3, 50.0),
    (0.4, 60.0),
    (0.5, 70.0),
    (0.6, 80.0),
    (0.7, 90.0),
    (0.8, 100.0),
    (0.9, 110.0),
    (1.0, 120.0),
    (1.1, 130.0),
    (1.2, 140.0),
  ] {
    database.add(pcb_drill(diameter_mm, plunge_feed));
  }
  database
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

  #[test]
  fn the_seed_library_is_the_standard_ten_piece_pcb_drill_set() {
    let db = seed_library();
    assert_eq!(db.len(), 10, "a 0.3–1.2 mm ten-piece set");
    // The diameters are exactly 0.3..=1.2 mm in 0.1 mm steps (rounded to guard float formatting drift).
    let diameters: Vec<f64> = db.iter().map(|t| (t.diameter.as_mm() * 10.0).round() / 10.0).collect();
    assert_eq!(diameters, vec![0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2]);
    // Every entry carries sensible drilling defaults: through a 1.6 mm board with a positive plunge feed that
    // grows with diameter (the smallest bit plunges slowest).
    for tool in db.iter() {
      assert!(tool.drilling.depth > 1.6, "{} must clear a 1.6 mm board", tool.name);
      assert!(tool.drilling.feed > 0.0, "{} needs a plunge feed", tool.name);
    }
    let feeds: Vec<f64> = db.iter().map(|t| t.drilling.feed).collect();
    assert!(feeds.windows(2).all(|w| w[0] < w[1]), "plunge feed scales up with diameter");
  }
}
