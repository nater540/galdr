//! The eitri-app config: a startup-loaded, human-editable JSON file under the OS config dir (e.g.
//! `~/Library/Application Support/eitri/config.json` on macOS, `~/.config/eitri/config.json` on Linux) carrying
//! appearance/themes, UI defaults, and canvas-render tuning. Mirrored from skirnir's config store, with the same
//! never-panic contract:
//!
//! 1. **Partial overrides via `#[serde(default)]` on every field and section.** A file need only carry the keys
//!    it changes; every absent field, and every absent whole section, falls back to the baked default.
//! 2. **Layered theme resolution.** Built-in palettes live in code; the config's `active_theme` selects one (or
//!    a user theme that overrides over a `base`). See [`theme`]. An unknown name → the default palette + a notice.
//! 3. **Versioning / forward-compat.** A `version` greater than [`CONFIG_VERSION`] is refused (we cannot read a
//!    future layout) — defaults + a notice. Older/missing fields fill from default.
//! 4. **Bundled default + first-run seed.** The bundled `assets/config.default.json` is the canonical, fully-keyed
//!    documented example; a missing file is seeded from it, giving the operator a file to edit.
//! 5. **Never panic.** [`load`] returns `(Config, Vec<String>)`: notices instead of errors. A malformed file →
//!    full defaults + a notice. The notices surface in the log dock.
//! 6. **Atomic save** ([`save`]/[`save_to`], temp sibling + rename via [`crate::store::atomic_write`]) behind the
//!    settings dialog's explicit Save — nothing writes the operator-owned file implicitly.

pub mod color;
pub mod sections;
pub mod theme;

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

pub use color::ColorSpec;
pub use sections::{CanvasConfig, CanvasStyle, UiConfig};
pub use theme::{AppearanceConfig, FONT_SCALE_RANGE, ThemeOverride, builtin_palette, clamp_font_scale};

use crate::app::theme::Palette;

/// The on-disk schema version. Bump on any backwards-incompatible change to [`Config`]'s shape; a file whose
/// `version` exceeds this is refused, while an older-or-equal version is accepted and missing fields fill from
/// `#[serde(default)]`.
pub const CONFIG_VERSION: u32 = 1;

/// The file name written under the per-user config directory.
const CONFIG_FILE: &str = "config.json";

/// The bundled default config, embedded at compile time. Used ONLY as the first-run seed written to disk and as
/// the canonical fully-keyed example; once a file exists on disk it is authoritative. It must always parse to a
/// valid [`Config`] (a test guards this), so a seed is never garbage.
const BUNDLED_DEFAULT: &str = include_str!("../../assets/config.default.json");

/// The whole app config: a versioned envelope around the appearance, UI, and canvas sections. New sections can
/// grow here behind `#[serde(default)]` without breaking older files.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Config {
  /// The on-disk schema version, written as [`CONFIG_VERSION`]. Read back to refuse a future layout.
  pub version: u32,
  /// Appearance: the active theme, font scale, and user-defined theme overrides.
  pub appearance: AppearanceConfig,
  /// UI defaults applied when the transient view state is built at startup.
  pub ui: UiConfig,
  /// Canvas-render tuning resolved into the runtime [`CanvasStyle`].
  pub canvas: CanvasConfig,
}

impl Default for Config {
  fn default() -> Self {
    Config {
      version: CONFIG_VERSION,
      appearance: AppearanceConfig::default(),
      ui: UiConfig::default(),
      canvas: CanvasConfig::default(),
    }
  }
}

impl Config {
  /// Parse a config from JSON text. Returns `Ok` for a *usable* file (a version ≤ [`CONFIG_VERSION`]), or
  /// `Err(reason)` for an unusable one: malformed JSON, a bad colour, or a version newer than this build
  /// understands. The caller turns an `Err` into defaults + a notice. Never panics.
  pub fn from_json(body: &str) -> Result<Self, String> {
    let config: Config = serde_json::from_str(body).map_err(|err| format!("config is not valid JSON: {err}"))?;
    if config.version > CONFIG_VERSION {
      return Err(format!(
        "config version {} is newer than this build understands ({CONFIG_VERSION})",
        config.version
      ));
    }
    Ok(config)
  }

  /// Serialise to pretty JSON (the on-disk form) with a trailing newline. A serialise failure is surfaced as a
  /// reason string rather than unwrapped.
  pub fn to_json(&self) -> Result<String, String> {
    serde_json::to_string_pretty(self)
      .map(|mut body| {
        body.push('\n');
        body
      })
      .map_err(|err| format!("failed to serialise the config: {err}"))
  }

  /// Resolve the active theme into a runtime [`Palette`], plus an optional notice (unknown `active_theme`, …).
  /// Deferred here (rather than computed at parse time) so a config can be re-resolved after a live edit.
  pub fn palette(&self) -> (Palette, Option<String>) {
    self.appearance.resolve_palette()
  }

  /// Resolve the canvas section into the runtime [`CanvasStyle`] the canvas renders with.
  pub fn canvas_style(&self) -> CanvasStyle {
    self.canvas.resolve()
  }
}

/// Resolve the absolute path of the config file under the OS config directory, creating nothing. Returns `None`
/// if no per-user config base exists on this platform; the caller then runs on in-memory defaults. Public so the
/// GUI can show the operator where the config lives for hand-editing.
pub fn config_path() -> Option<PathBuf> {
  crate::store::config_dir().ok().map(|dir| dir.join(CONFIG_FILE))
}

/// Load the config from the default OS location. NEVER fails: a missing file is seeded from the bundled default
/// and then read; any read/parse/seed failure falls back to in-memory defaults. The returned tuple carries the
/// config plus a list of human-readable notices for the caller to surface (an ordinary first run reports none).
pub fn load() -> (Config, Vec<String>) {
  match config_path() {
    Some(path) => load_from(&path),
    None => (Config::default(), vec!["no config directory for the app config — using built-in defaults".to_string()]),
  }
}

/// Load the config from a specific path — the testable core of [`load`], so no real config dir is touched in
/// tests. A missing file is seeded from the bundled default (first run, no notice) then read; the on-disk file is
/// then parsed as authoritative. Any failure falls back to defaults WITH a notice. Never panics.
pub fn load_from(path: &Path) -> (Config, Vec<String>) {
  if !path.exists()
    && let Err(reason) = seed_default(path)
  {
    return finish(Config::default(), vec![reason]);
  }
  match std::fs::read_to_string(path) {
    Ok(body) => match Config::from_json(&body) {
      Ok(config) => finish(config, Vec::new()),
      Err(reason) => finish(
        Config::default(),
        vec![format!("config at {} is unusable ({reason}) — using built-in defaults", path.display())],
      ),
    },
    Err(err) => finish(
      Config::default(),
      vec![format!("could not read config at {}: {err} — using built-in defaults", path.display())],
    ),
  }
}

/// Fold a soft theme-resolution notice (an unknown `active_theme`, etc.) into the load notices, so the caller
/// learns the active theme was substituted even when the file itself parsed cleanly.
fn finish(config: Config, mut notices: Vec<String>) -> (Config, Vec<String>) {
  if let (_, Some(theme_notice)) = config.palette() {
    notices.push(theme_notice);
  }
  (config, notices)
}

/// Write the bundled default to `path` via the shared atomic write. Only called on first run, when the file is
/// absent, so an existing hand-edited file is never clobbered.
fn seed_default(path: &Path) -> Result<(), String> {
  crate::store::atomic_write(path, BUNDLED_DEFAULT.as_bytes())
    .map_err(|err| format!("could not seed the config at {}: {err} — using built-in defaults", path.display()))
}

/// Save the config to the default OS location. Returns a reason string on failure the caller surfaces (a failed
/// save must be reported, never panic the UI). An atomic write, behind the settings dialog's explicit Save.
pub fn save(config: &Config) -> Result<(), String> {
  let path = config_path().ok_or_else(|| "no config directory to save the config to".to_string())?;
  save_to(config, &path)
}

/// Save the config to a specific path — the testable core of [`save`].
pub fn save_to(config: &Config, path: &Path) -> Result<(), String> {
  let body = config.to_json()?;
  crate::store::atomic_write(path, body.as_bytes()).map_err(|err| err.to_string())
}

#[cfg(test)]
mod tests {
  use super::*;

  /// The bundled asset's name for its self-documenting, fully-keyed template theme. Named `"default"` so it
  /// shadows the built-in default and is the ACTIVE theme out of the box — editing any `themes.default.<color>`
  /// in the seeded file recolours immediately, with no indirection.
  const TEMPLATE_THEME: &str = "default";

  #[test]
  fn the_bundled_default_parses_to_a_usable_config() {
    let config = Config::from_json(BUNDLED_DEFAULT).expect("the bundled config.default.json must parse");
    assert_eq!(config.version, CONFIG_VERSION, "the bundled default carries the current version");
    let (_, notice) = config.palette();
    assert_eq!(notice, None, "the bundled default's active_theme must resolve cleanly");
  }

  #[test]
  fn the_bundled_template_theme_is_a_fully_keyed_no_op_and_is_the_active_theme() {
    // The asset ships a `default` theme (shadowing the built-in) that names EVERY overridable colour field, set
    // to its `default_dark` value — a first-run operator sees every knob in one place AND can edit it directly.
    // Unedited, it must resolve to exactly the built-in default (a true no-op until a value changes).
    let config = Config::from_json(BUNDLED_DEFAULT).expect("the bundled asset parses");
    assert_eq!(config.appearance.active_theme, TEMPLATE_THEME);
    let template = config
      .appearance
      .themes
      .get(TEMPLATE_THEME)
      .unwrap_or_else(|| panic!("the bundled asset must carry a `{TEMPLATE_THEME}` template theme"));
    assert!(
      template.all_color_fields_set(),
      "the template theme must name EVERY overridable colour field — add the new field to config.default.json",
    );
    let (palette, notice) = config.palette();
    assert_eq!(notice, None);
    assert_eq!(
      palette,
      Palette::default_dark(),
      "the unedited template must resolve to exactly the default palette — it documents, it does not recolour",
    );
  }

  #[test]
  fn the_configured_language_round_trips() {
    let mut config = Config::default();
    assert_eq!(config.ui.language, crate::i18n::EN_US, "a fresh config defaults to the bundled source locale");
    config.ui.language = crate::i18n::SV_SE.to_string();
    let body = config.to_json().expect("serialises");
    let parsed = Config::from_json(&body).expect("round-trips");
    assert_eq!(parsed.ui.language, "sv-SE", "the configured language survives a save/load round trip");
  }

  #[test]
  fn loading_a_missing_file_seeds_it_and_round_trips_the_bundled_asset_with_no_notice() {
    let dir = std::env::temp_dir().join(format!("eitri-config-seed-{}", std::process::id()));
    let path = dir.join("nested").join(CONFIG_FILE); // also exercises parent-dir creation.
    let _ = std::fs::remove_dir_all(&dir);
    let (config, notices) = load_from(&path);
    assert!(notices.is_empty(), "seeding a missing file is the ordinary first run — no notice: {notices:?}");
    assert!(path.exists(), "the file must have been seeded to disk for the operator to edit");
    let from_asset = Config::from_json(BUNDLED_DEFAULT).expect("the bundled asset parses");
    assert_eq!(config, from_asset, "the seeded-then-read config must round-trip the bundled asset exactly");
    // A fresh seed resolves to the default palette, so a first install looks exactly like the built-in default.
    let (palette, _) = config.palette();
    assert_eq!(palette, Palette::default_dark());
    let _ = std::fs::remove_dir_all(&dir);
  }

  #[test]
  fn loading_a_corrupt_file_falls_back_to_defaults_with_a_notice_and_never_panics() {
    let dir = std::env::temp_dir().join(format!("eitri-config-corrupt-{}", std::process::id()));
    let path = dir.join(CONFIG_FILE);
    std::fs::create_dir_all(&dir).expect("temp dir");
    std::fs::write(&path, "this is not valid JSON {{{").expect("write a garbage file");
    let (config, notices) = load_from(&path);
    assert_eq!(config, Config::default(), "a corrupt file must fall back to defaults, not crash");
    assert!(!notices.is_empty(), "a corrupt file should surface a notice");
    let _ = std::fs::remove_dir_all(&dir);
  }

  #[test]
  fn a_bad_hex_colour_falls_back_to_defaults_with_a_notice() {
    let dir = std::env::temp_dir().join(format!("eitri-config-badhex-{}", std::process::id()));
    let path = dir.join(CONFIG_FILE);
    std::fs::create_dir_all(&dir).expect("temp dir");
    let body = r##"{ "appearance": { "themes": { "x": { "copper": "#nothex" } } } }"##;
    std::fs::write(&path, body).expect("write the bad-hex file");
    let (config, notices) = load_from(&path);
    assert_eq!(config, Config::default(), "a bad hex colour must fall back to defaults");
    assert!(!notices.is_empty(), "a bad hex colour must surface a notice");
    let _ = std::fs::remove_dir_all(&dir);
  }

  #[test]
  fn a_config_from_a_newer_version_is_refused_in_favour_of_defaults() {
    let body = format!("{{ \"version\": {} }}", CONFIG_VERSION + 1);
    let err = Config::from_json(&body).expect_err("a newer-versioned config must be refused");
    assert!(err.contains("newer"), "the refusal should explain it is too new: {err}");
  }

  #[test]
  fn an_equal_or_older_version_is_accepted_and_missing_sections_default() {
    let minimal = format!("{{ \"version\": {CONFIG_VERSION} }}");
    let parsed = Config::from_json(&minimal).expect("a version-only config must parse via serde defaults");
    assert_eq!(parsed, Config::default(), "missing sections fill from Default");
  }

  #[test]
  fn a_partial_file_merges_one_key_per_section_over_defaults() {
    let body = r#"{
      "ui": { "language": "sv-SE" },
      "canvas": { "cut_stroke_px": 3.0 }
    }"#;
    let config = Config::from_json(body).expect("a partial config parses");
    assert_eq!(config.ui.language, "sv-SE", "the changed ui key is taken");
    assert_eq!(config.ui.window_w, UiConfig::default().window_w, "the unset ui key defaults");
    assert!((config.canvas.cut_stroke_px - 3.0).abs() < 1e-6, "the changed canvas key is taken");
    assert_eq!(config.appearance, AppearanceConfig::default(), "an unmentioned section defaults wholesale");
  }

  #[test]
  fn an_unknown_active_theme_surfaces_a_notice_through_load() {
    let dir = std::env::temp_dir().join(format!("eitri-config-unknown-theme-{}", std::process::id()));
    let path = dir.join(CONFIG_FILE);
    std::fs::create_dir_all(&dir).expect("temp dir");
    std::fs::write(&path, r#"{ "appearance": { "active_theme": "ghost" } }"#).expect("write");
    let (config, notices) = load_from(&path);
    assert_eq!(config.appearance.active_theme, "ghost", "the config still records what the file asked for");
    assert!(notices.iter().any(|n| n.contains("ghost")), "an unknown active_theme must surface a notice");
    let _ = std::fs::remove_dir_all(&dir);
  }

  #[test]
  fn save_then_load_through_a_real_file_round_trips() {
    let dir = std::env::temp_dir().join(format!("eitri-config-roundtrip-{}", std::process::id()));
    let path = dir.join("nested").join(CONFIG_FILE);
    let _ = std::fs::remove_dir_all(&dir);
    let mut config = Config::default();
    config.ui.language = crate::i18n::SV_SE.to_string();
    config.appearance.active_theme = "midnight".to_string();
    save_to(&config, &path).expect("saving to a writable temp path succeeds");
    let (loaded, notices) = load_from(&path);
    assert_eq!(loaded, config, "a saved config must load back identically");
    assert!(notices.is_empty(), "a clean round-trip surfaces no notice: {notices:?}");
    let _ = std::fs::remove_dir_all(&dir);
  }
}
