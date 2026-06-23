//! Runtime-loaded, human-readable explanations of grbl/grblHAL settings, keyed by setting number — the supplement
//! the dynamic `$ES` enumeration cannot carry.
//!
//! The settings panel learns each setting's *name*, *unit*, and *bounds* from the controller's `$ES` reply (see
//! `docs/gcode-streaming.md`'s "do not hardcode a settings UI" directive, honoured by
//! [`crate::app::settings_model`]). `$ES` does not carry a prose description of what a setting *does*, though, so
//! the tooltip pairs that live metadata with a short curated sentence loaded from this module. The `$ES` data
//! stays PRIMARY (accurate per-firmware); these descriptions only add the explanation and must degrade gracefully
//! — an unknown number returns `None` and the row simply shows its `$ES`-derived label with no extra prose.
//!
//! **Mechanism — a runtime JSON file under the OS config dir, seeded from a bundled default.** The on-disk file is
//! authoritative so an operator can edit a description and restart to see it; the bundled
//! `assets/setting_descriptions.json` is embedded ONLY as the first-run seed (written to disk if absent) and as
//! the in-memory fallback when the on-disk file is missing, unreadable, or malformed. The format is a flat object
//! of `"$<n>": "text"` pairs (string keys WITH the leading `$`); a key that does not parse as `$<number>` is
//! ignored rather than failing the whole load. Mirrors [`crate::profile`]'s path-taking, never-panics contract.
//!
//! ## Per-axis settings
//! grblHAL groups axis settings into decades whose unit digit selects the axis (`$100/$101/$102/$103` = X/Y/Z/A
//! travel resolution, `$110…` = max rate, and so on). The JSON carries an explicit entry per real axis number, so
//! the rotary A entries ($103/$113/$123/$133/$143/$153) can say "deg" where the linear X/Y/Z stay "mm".
//!
//! ## Galdr-specific settings
//! `$19`, `$376`, `$392`, `$393`, and `$481` are not all stock grbl. Their descriptions were written from the
//! firmware's own setting table (`crates/firmware-core/src/settings.rs`) and the design docs
//! (`docs/4th-axis-rotary-design.md`, `docs/00-architecture.md`), not guessed.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// The file name written under the per-user config directory (e.g. `~/.config/skirnir/setting_descriptions.json`
/// on Linux). Sits beside [`crate::profile`]'s `profile.ron` in the same app config dir.
const DESCRIPTIONS_FILE: &str = "setting_descriptions.json";

/// The application identifier used to locate the OS config directory via [`directories::ProjectDirs`]. Matches
/// [`crate::profile`]'s `APP_NAME` so both stores resolve to the same `~/.config/skirnir` directory.
const APP_NAME: &str = "skirnir";

/// The bundled defaults, embedded at compile time. Used ONLY as the first-run seed written to disk and as the
/// in-memory fallback when the on-disk file cannot be read/parsed — never the live source once a file exists.
const BUNDLED_DEFAULT: &str = include_str!("../../assets/setting_descriptions.json");

/// The loaded curated setting descriptions, keyed by setting number with the leading `$` stripped. Parsed once at
/// startup (never per-frame) and stored in the view state; an absent number degrades to no curated prose.
#[derive(Debug, Clone, Default)]
pub struct SettingDescriptions {
  by_number: HashMap<u32, String>,
}

impl SettingDescriptions {
  /// A short human explanation of what setting `number` controls, or `None` when the number is not in the loaded
  /// set. The settings tooltip shows this beneath the `$ES`-derived name and range; when it is `None` the tooltip
  /// degrades to just the dynamic metadata, never an empty box.
  pub fn description(&self, number: u32) -> Option<&str> {
    self.by_number.get(&number).map(String::as_str)
  }

  /// How many descriptions are loaded. Used by tests; harmless elsewhere.
  pub fn len(&self) -> usize {
    self.by_number.len()
  }

  /// Whether no descriptions are loaded (e.g. an empty or fully-unparseable JSON object).
  pub fn is_empty(&self) -> bool {
    self.by_number.is_empty()
  }

  /// Build the in-memory set from the bundled default, never touching disk. The fallback when a config dir or the
  /// on-disk file is unavailable; the bundled JSON is well-formed by construction, so a parse failure here yields
  /// an empty (but valid) set rather than a panic.
  pub fn bundled() -> Self {
    SettingDescriptions { by_number: parse_descriptions(BUNDLED_DEFAULT) }
  }
}

/// Parse a `{ "$<n>": "text" }` JSON object into a number→text map. Keys that do not parse as `$<number>` (or a
/// bare `<number>`) are ignored rather than failing the whole parse, so one stray key cannot blank every tooltip;
/// a body that is not a JSON object of strings yields an empty map. Never panics.
fn parse_descriptions(body: &str) -> HashMap<u32, String> {
  let mut out = HashMap::new();
  // Parse leniently into a string→string map; a non-object or non-string value yields an empty map (the caller
  // then falls back), and we deliberately do not error on it — descriptions are advisory, not load-bearing.
  let raw: HashMap<String, String> = match serde_json::from_str(body) {
    Ok(map) => map,
    Err(_) => return out,
  };
  for (key, text) in raw {
    // Accept both the documented `$<n>` form and a bare `<n>`, so a hand-edited file is forgiving.
    let number = key.trim().strip_prefix('$').unwrap_or(key.trim());
    if let Ok(n) = number.parse::<u32>() {
      out.insert(n, text);
    }
  }
  out
}

/// Resolve the absolute path of the descriptions file under the OS config directory (e.g.
/// `~/.config/skirnir/setting_descriptions.json` on Linux), creating nothing. Returns `None` if no per-user config
/// base exists on this platform; the caller then runs on the bundled default. Public so the GUI can show where the
/// file lives for hand-editing.
pub fn descriptions_path() -> Option<PathBuf> {
  let dirs = directories::ProjectDirs::from("", "", APP_NAME)?;
  Some(dirs.config_dir().join(DESCRIPTIONS_FILE))
}

/// Load the curated descriptions from the default OS location. NEVER fails: a missing file is seeded from the
/// bundled default and then read; any read/parse/seed failure falls back to the bundled default in memory. The
/// returned tuple carries the descriptions plus an optional human-readable reason a fallback happened, for the
/// caller to surface as a single console notice (the ordinary first run reports `None`).
pub fn load() -> (SettingDescriptions, Option<String>) {
  match descriptions_path() {
    Some(path) => load_from(&path),
    // No config dir at all: run on the bundled default and tell the caller why, so it can note that edits won't
    // persist anywhere.
    None => (
      SettingDescriptions::bundled(),
      Some("no config directory for setting descriptions — using the bundled defaults".to_string()),
    ),
  }
}

/// Load the descriptions from a specific path — the testable core of [`load`], so no real config dir is touched in
/// tests. A missing file is seeded from the bundled default (first run, no notice); the on-disk file is then read
/// and parsed as authoritative. Any failure — a seed write that fails, an unreadable file, malformed JSON — falls
/// back to parsing the bundled default in memory WITH a reason. Never panics.
pub fn load_from(path: &Path) -> (SettingDescriptions, Option<String>) {
  // First run: the file is absent. Seed it from the bundled default so the operator has a file to edit, then read
  // it back as the authoritative source. A seed failure is non-fatal — we fall through to the bundled default.
  if !path.exists()
    && let Err(reason) = seed_default(path)
  {
    return (SettingDescriptions::bundled(), Some(reason));
  }
  match std::fs::read_to_string(path) {
    Ok(body) => {
      let map = parse_descriptions(&body);
      if map.is_empty() {
        // The file exists but parsed to nothing usable (malformed JSON, or not an object of strings): fall back to
        // the bundled default so tooltips still work, and surface why.
        (
          SettingDescriptions::bundled(),
          Some(format!(
            "setting descriptions at {} are unreadable or empty — using the bundled defaults",
            path.display()
          )),
        )
      } else {
        (SettingDescriptions { by_number: map }, None)
      }
    }
    // The file vanished between the seed and the read, or is unreadable (permissions, …): bundled default + notice.
    Err(err) => (
      SettingDescriptions::bundled(),
      Some(format!(
        "could not read setting descriptions at {}: {err} — using the bundled defaults",
        path.display()
      )),
    ),
  }
}

/// Write the bundled default to `path`, creating any missing parent directories. Returns a human-readable reason on
/// failure (a missing config base or a write error), never a panic. Only called on first run, when the file is
/// absent, so an existing hand-edited file is never clobbered.
fn seed_default(path: &Path) -> Result<(), String> {
  if let Some(parent) = path.parent() {
    std::fs::create_dir_all(parent)
      .map_err(|err| format!("could not create the setting-descriptions directory: {err} — using bundled defaults"))?;
  }
  std::fs::write(path, BUNDLED_DEFAULT)
    .map_err(|err| format!("could not seed setting descriptions at {}: {err} — using bundled defaults", path.display()))
}

#[cfg(test)]
mod tests {
  use super::*;

  /// A small, well-formed JSON sample exercising both key forms and a Galdr-specific number.
  const SAMPLE: &str = r#"{ "$0": "Step pulse time.", "22": "Enable homing.", "$376": "Rotary mask." }"#;

  #[test]
  fn parsing_a_sample_yields_the_expected_map() {
    let map = parse_descriptions(SAMPLE);
    assert_eq!(map.get(&0).map(String::as_str), Some("Step pulse time."));
    assert_eq!(map.get(&376).map(String::as_str), Some("Rotary mask."));
  }

  #[test]
  fn a_dollar_prefixed_and_a_bare_key_both_parse_to_the_right_number() {
    // The documented form is `$<n>`; a hand-edited bare `<n>` is also tolerated. Both land on the same number.
    let map = parse_descriptions(SAMPLE);
    assert!(map.contains_key(&0), "`$0` must parse to number 0");
    assert!(map.contains_key(&22), "a bare `22` must parse to number 22");
  }

  #[test]
  fn an_unknown_number_returns_none() {
    let desc = SettingDescriptions { by_number: parse_descriptions(SAMPLE) };
    assert_eq!(desc.description(9999), None, "a number not in the set degrades to no prose");
  }

  #[test]
  fn a_non_numeric_key_is_ignored_not_fatal() {
    // One stray key must not blank the rest of the set.
    let map = parse_descriptions(r#"{ "$hello": "nope", "$0": "kept" }"#);
    assert_eq!(map.len(), 1, "the unparseable key is dropped, the good one kept");
    assert_eq!(map.get(&0).map(String::as_str), Some("kept"));
  }

  #[test]
  fn malformed_json_parses_to_an_empty_map() {
    assert!(parse_descriptions("this is not json {{{").is_empty(), "garbage must not panic, just yield nothing");
  }

  #[test]
  fn the_bundled_default_parses_and_covers_the_curated_settings() {
    // The embedded asset must be valid and carry the curated spread: scalars, every axis number, and the Galdr
    // settings. This guards the asset itself against an editing slip.
    let desc = SettingDescriptions::bundled();
    assert!(!desc.is_empty(), "the bundled default must parse to a non-empty set");
    for n in [0u32, 1, 5, 10, 11, 20, 22, 27, 30] {
      assert!(desc.description(n).is_some_and(|d| !d.trim().is_empty()), "scalar ${n} must be described");
    }
    for n in [100u32, 101, 102, 103, 110, 113, 120, 123, 130, 133, 140, 143, 150, 153] {
      assert!(desc.description(n).is_some_and(|d| !d.trim().is_empty()), "axis ${n} must be described");
    }
    for n in [19u32, 376, 392, 393, 481] {
      assert!(desc.description(n).is_some_and(|d| !d.trim().is_empty()), "Galdr ${n} must be described");
    }
  }

  #[test]
  fn the_rotary_a_axis_entries_note_degrees_where_the_linear_axes_use_mm() {
    // $103 is steps/deg (A is rotary), $100–$102 stay steps/mm; $133 is deg of travel, $130–$132 are mm.
    let desc = SettingDescriptions::bundled();
    assert!(desc.description(103).is_some_and(|d| d.contains("deg")), "$103 must mention deg for the rotary A axis");
    assert!(desc.description(100).is_some_and(|d| d.contains("mm")), "$100 stays mm for the linear X axis");
    assert!(desc.description(133).is_some_and(|d| d.contains("deg")), "$133 (A travel) must mention deg");
  }

  #[test]
  fn loading_a_missing_file_seeds_it_and_reads_the_bundled_set_with_no_notice() {
    // First-run path: the file does not exist. `load_from` seeds it from the bundled default, then reads it back.
    let dir = std::env::temp_dir().join(format!("skirnir-desc-seed-{}", std::process::id()));
    let path = dir.join("nested").join(DESCRIPTIONS_FILE); // also exercises parent-dir creation.
    let _ = std::fs::remove_dir_all(&dir);
    let (desc, notice) = load_from(&path);
    assert_eq!(notice, None, "seeding a missing file is the ordinary first run — no notice");
    assert!(path.exists(), "the file must have been seeded to disk for the operator to edit");
    assert!(desc.description(376).is_some(), "the seeded-then-read set carries the bundled descriptions");
    let _ = std::fs::remove_dir_all(&dir);
  }

  #[test]
  fn the_on_disk_file_is_authoritative_over_the_bundled_default() {
    // A hand-edited file must win: write a one-entry file and confirm we read exactly it, not the bundled set.
    let dir = std::env::temp_dir().join(format!("skirnir-desc-edit-{}", std::process::id()));
    let path = dir.join(DESCRIPTIONS_FILE);
    std::fs::create_dir_all(&dir).expect("temp dir");
    std::fs::write(&path, r#"{ "$0": "my own words" }"#).expect("write an edited file");
    let (desc, notice) = load_from(&path);
    assert_eq!(notice, None, "a readable, parseable file surfaces no notice");
    assert_eq!(desc.description(0), Some("my own words"), "the on-disk text must override the bundled default");
    assert_eq!(desc.description(376), None, "the edited file's other entries are gone — it is authoritative");
    let _ = std::fs::remove_dir_all(&dir);
  }

  #[test]
  fn a_malformed_on_disk_file_falls_back_to_the_bundled_default_with_a_notice() {
    let dir = std::env::temp_dir().join(format!("skirnir-desc-corrupt-{}", std::process::id()));
    let path = dir.join(DESCRIPTIONS_FILE);
    std::fs::create_dir_all(&dir).expect("temp dir");
    std::fs::write(&path, "this is not valid JSON {{{").expect("write a garbage file");
    let (desc, notice) = load_from(&path);
    assert!(notice.is_some(), "a malformed file must surface a notice so the operator knows defaults are in use");
    assert!(desc.description(376).is_some(), "tooltips still work via the bundled fallback, never blank");
    let _ = std::fs::remove_dir_all(&dir);
  }
}
