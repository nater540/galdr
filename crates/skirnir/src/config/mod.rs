//! The skirnir app config: a startup-loaded, human-editable JSON file under the OS config dir (e.g.
//! `~/Library/Application Support/skirnir/config.json` on macOS, `~/.config/skirnir/config.json` on Linux) carrying
//! appearance/themes, UI defaults, connection/streaming tuning, and toolpath-render tuning. The
//! appearance/preferences sibling of [`crate::profile`] (machine/rotary state) and
//! [`crate::app::setting_help`] (per-setting tooltips); it is a separate store and leaves `profile.ron` untouched.
//!
//! **Robustness — the same never-panic contract the other stores honour.**
//! 1. **Partial overrides via `#[serde(default)]` on every field and section.** A file need only carry the keys it
//!    changes; every absent field, and every absent whole section, falls back to the baked default.
//! 2. **Layered theme resolution.** Built-in palettes live in code; the config's `active_theme` selects one (or a
//!    user theme that overrides over a `base`). See [`theme`]. An unknown name → the default palette + a notice.
//! 3. **Versioning / forward-compat.** A `version` greater than [`CONFIG_VERSION`] is refused (we cannot read a
//!    future layout) — defaults + a notice — exactly like [`crate::profile`]. Older/missing fields fill from default.
//! 4. **Bundled default + first-run seed.** The bundled `assets/config.default.json` is the canonical, fully-keyed
//!    documented example; a missing file is seeded from it (like [`crate::app::setting_help`]), giving the operator a
//!    file to edit.
//! 5. **Never panic.** [`load`] returns `(Config, Vec<String>)`: notices instead of errors. A malformed file (bad
//!    JSON, a bad hex colour) → full defaults + a notice. The notices surface in the console.
//! 6. **Atomic save available** ([`save`]/[`save_to`], temp sibling + rename via [`crate::store::atomic_write`]) as
//!    public store infrastructure for a future programmatic write. NOTE: the app does NOT currently auto-persist the
//!    config — the file is operator-owned and hand-edited, and rewriting the whole file on exit would clobber the
//!    operator's formatting/comments. So there is no app-driven write today; `save` exists for callers that want one.

pub mod color;
pub mod sections;
pub mod theme;

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

pub use color::ColorSpec;
pub use sections::{ConnectionConfig, ReconnectSection, ToolpathConfig, ToolpathStyle, UiConfig};
pub use theme::{AppearanceConfig, FONT_SCALE_RANGE, ThemeOverride, builtin_palette, clamp_font_scale};

use crate::app::theme::Palette;

/// The on-disk schema version. Bump on any backwards-incompatible change to [`Config`]'s shape; a file whose
/// `version` exceeds this is refused (we cannot interpret a future layout), while an older-or-equal version is
/// accepted and missing fields fill from `#[serde(default)]`. The single knob that makes the format forward-aware.
pub const CONFIG_VERSION: u32 = 1;

/// The file name written under the per-user config directory (e.g. `~/Library/Application Support/skirnir/config.json`
/// on macOS, `~/.config/skirnir/config.json` on Linux). Sits beside `profile.ron` and `setting_descriptions.json` in
/// the same app config dir.
const CONFIG_FILE: &str = "config.json";

/// The bundled default config, embedded at compile time. Used ONLY as the first-run seed written to disk and as the
/// canonical fully-keyed example; once a file exists on disk it is authoritative. It must always parse to a valid
/// [`Config`] (a test guards this), so a seed is never garbage.
const BUNDLED_DEFAULT: &str = include_str!("../../assets/config.default.json");

/// The whole app config: a versioned envelope around the appearance, UI, connection, and toolpath sections. New
/// sections can grow here behind `#[serde(default)]` without breaking older files.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Config {
  /// The on-disk schema version, written as [`CONFIG_VERSION`]. Read back to refuse a future layout.
  pub version: u32,
  /// Appearance: the active theme, font scale, and user-defined theme overrides.
  pub appearance: AppearanceConfig,
  /// UI defaults applied when the transient widget state is built at startup.
  pub ui: UiConfig,
  /// Connection/streaming tuning read where the engine and reconnect schedule are constructed.
  pub connection: ConnectionConfig,
  /// Toolpath-render tuning resolved into the runtime [`ToolpathStyle`].
  pub toolpath: ToolpathConfig,
}

impl Default for Config {
  fn default() -> Self {
    Config {
      version: CONFIG_VERSION,
      appearance: AppearanceConfig::default(),
      ui: UiConfig::default(),
      connection: ConnectionConfig::default(),
      toolpath: ToolpathConfig::default(),
    }
  }
}

impl Config {
  /// Parse a config from JSON text. Returns `Ok` with the config plus an optional notice for a *usable* file (a
  /// version ≤ [`CONFIG_VERSION`], whatever its content gaps), or `Err(reason)` for an unusable one: malformed JSON,
  /// a bad colour, or a version newer than this build understands. The caller turns an `Err` into defaults + a
  /// notice. Never panics.
  ///
  /// The notice inside `Ok` is for a *soft* problem that still parsed (currently an unknown `active_theme`, surfaced
  /// by [`AppearanceConfig::resolve_palette`] at resolve time) — but theme resolution is deferred to [`Self::palette`]
  /// so a single config can be re-resolved on a live reload; parsing itself only flags hard failures via `Err`.
  pub fn from_json(body: &str) -> Result<Self, String> {
    let config: Config = serde_json::from_str(body).map_err(|err| format!("config is not valid JSON: {err}"))?;
    // A file from a newer skirnir may carry fields/shapes this build cannot interpret; refuse it rather than risk
    // misreading, and let the caller fall back to defaults (the existing file is left untouched on disk).
    if config.version > CONFIG_VERSION {
      return Err(format!(
        "config version {} is newer than this build understands ({CONFIG_VERSION})",
        config.version
      ));
    }
    Ok(config)
  }

  /// Serialise to pretty JSON (the on-disk form) with a trailing newline so the file is a tidy text file. A
  /// serialise failure is surfaced as a reason string rather than unwrapped (it should not happen for a valid
  /// in-memory config, but the surface stays non-panicking).
  pub fn to_json(&self) -> Result<String, String> {
    serde_json::to_string_pretty(self)
      .map(|mut body| {
        body.push('\n');
        body
      })
      .map_err(|err| format!("failed to serialise the config: {err}"))
  }

  /// Resolve the active theme into a runtime [`Palette`], plus an optional notice (unknown `active_theme`, etc.).
  /// Deferred here (rather than computed at parse time) so a config can be re-resolved after a live reload.
  pub fn palette(&self) -> (Palette, Option<String>) {
    self.appearance.resolve_palette()
  }

  /// Resolve the toolpath section into the runtime [`ToolpathStyle`] the viewport renders with.
  pub fn toolpath_style(&self) -> ToolpathStyle {
    self.toolpath.resolve()
  }
}

/// Resolve the absolute path of the config file under the OS config directory (e.g.
/// `~/Library/Application Support/skirnir/config.json` on macOS, `~/.config/skirnir/config.json` on Linux), creating
/// nothing. Returns `None` if no per-user config base exists on this platform; the caller then runs on in-memory
/// defaults. Public so the GUI can show the operator where the config lives for hand-editing.
pub fn config_path() -> Option<PathBuf> {
  crate::store::config_dir().ok().map(|dir| dir.join(CONFIG_FILE))
}

/// Load the config from the default OS location. NEVER fails: a missing file is seeded from the bundled default and
/// then read; any read/parse/seed failure falls back to in-memory defaults. The returned tuple carries the config
/// plus a list of human-readable notices for the caller to surface in the console (an ordinary first run reports
/// none). Notices accumulate (a load reason plus, e.g., an unknown-theme reason from resolution) so all are shown.
pub fn load() -> (Config, Vec<String>) {
  match config_path() {
    Some(path) => load_from(&path),
    // No config dir at all: run on in-memory defaults and tell the caller why, so it can note nothing will persist.
    None => (Config::default(), vec!["no config directory for the app config — using built-in defaults".to_string()]),
  }
}

/// Load the config from a specific path — the testable core of [`load`], so no real config dir is touched in tests.
/// A missing file is seeded from the bundled default (first run, no notice) then read; the on-disk file is then
/// parsed as authoritative. Any failure — a seed write that fails, an unreadable file, malformed JSON, a bad colour,
/// a too-new version — falls back to defaults WITH a notice. Never panics. The returned notices may include a
/// theme-resolution notice (an unknown `active_theme`) so the operator learns the active theme was substituted.
pub fn load_from(path: &Path) -> (Config, Vec<String>) {
  // First run: the file is absent. Seed it from the bundled default so the operator has a documented file to edit,
  // then read it back as the authoritative source. A seed failure is non-fatal — fall through to in-memory defaults.
  if !path.exists()
    && let Err(reason) = seed_default(path)
  {
    return finish(Config::default(), vec![reason]);
  }
  match std::fs::read_to_string(path) {
    Ok(body) => match Config::from_json(&body) {
      Ok(config) => finish(config, Vec::new()),
      // The file exists but is unusable (malformed JSON, a bad colour, or a too-new version): fall back to defaults
      // and surface why. The bad file is left in place so the operator can inspect/recover it.
      Err(reason) => finish(
        Config::default(),
        vec![format!("config at {} is unusable ({reason}) — using built-in defaults", path.display())],
      ),
    },
    // The file vanished between the seed and the read, or is unreadable (permissions, …): defaults + notice.
    Err(err) => finish(
      Config::default(),
      vec![format!("could not read config at {}: {err} — using built-in defaults", path.display())],
    ),
  }
}

/// Fold a soft theme-resolution notice (an unknown `active_theme`, etc.) into the load notices, so the caller learns
/// the active theme was substituted even when the file itself parsed cleanly. Keeps [`load_from`]'s branches tidy.
fn finish(config: Config, mut notices: Vec<String>) -> (Config, Vec<String>) {
  if let (_, Some(theme_notice)) = config.palette() {
    notices.push(theme_notice);
  }
  (config, notices)
}

/// Write the bundled default to `path`, creating any missing parent directories, via the shared atomic write. Returns
/// a human-readable reason on failure (a missing config base or a write error), never a panic. Only called on first
/// run, when the file is absent, so an existing hand-edited file is never clobbered.
fn seed_default(path: &Path) -> Result<(), String> {
  crate::store::atomic_write(path, BUNDLED_DEFAULT.as_bytes())
    .map_err(|err| format!("could not seed the config at {}: {err} — using built-in defaults", path.display()))
}

/// Save the config to the default OS location, creating the config directory if needed. Returns a reason string on
/// failure the caller surfaces (a failed save must be reported, never panic the UI). An atomic write.
///
/// NOTE: no production code calls this today — the config file is operator-owned and hand-edited, and the app does
/// not auto-persist it (a whole-file rewrite would clobber the operator's formatting/comments). This is public store
/// infrastructure kept ready for a future programmatic write (e.g. a theme-picker that writes back `active_theme`).
pub fn save(config: &Config) -> Result<(), String> {
  let path = config_path().ok_or_else(|| "no config directory to save the config to".to_string())?;
  save_to(config, &path)
}

/// Save the config to a specific path — the testable core of [`save`]. Serialises to pretty JSON and writes it
/// atomically (temp sibling + rename) so an interrupted write never leaves a truncated, unparseable config. All
/// failures surface as a reason string, never a panic.
pub fn save_to(config: &Config, path: &Path) -> Result<(), String> {
  let body = config.to_json()?;
  crate::store::atomic_write(path, body.as_bytes()).map_err(|err| err.to_string())
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn the_bundled_default_parses_to_a_usable_config() {
    // The embedded asset must always be valid JSON of the current schema — a seed is never garbage. This guards the
    // asset itself against an editing slip.
    let config = Config::from_json(BUNDLED_DEFAULT).expect("the bundled config.default.json must parse");
    assert_eq!(config.version, CONFIG_VERSION, "the bundled default carries the current version");
    let (_, notice) = config.palette();
    assert_eq!(notice, None, "the bundled default's active_theme must resolve cleanly");
  }

  #[test]
  fn the_configured_language_round_trips_and_drives_the_active_locale() {
    // The `ui.language` field must survive a save/load round trip and be the value the startup wiring selects on the
    // i18n registry. We assert the application step against an isolated `Translator` (not the process-wide registry)
    // so the test is deterministic regardless of test execution order — exactly the value `run()` passes to
    // `i18n::set_language` ends up as the active locale.
    let mut config = Config::default();
    assert_eq!(config.ui.language, crate::i18n::EN_US, "a fresh config defaults to the bundled source locale");

    config.ui.language = "fr-FR".to_string();
    let body = config.to_json().expect("serialises");
    let parsed = Config::from_json(&body).expect("round-trips");
    assert_eq!(parsed.ui.language, "fr-FR", "the configured language survives a save/load round trip");

    // Applying the configured locale (what `run()` does via `i18n::set_language(&config.ui.language)`) selects it.
    let mut translator = crate::i18n::Translator::new();
    translator.set_language(&parsed.ui.language);
    assert_eq!(translator.language(), "fr-FR", "the configured language becomes the active locale");
  }

  #[test]
  fn an_absent_language_field_fills_from_the_default() {
    // A config (e.g. one written before this field existed) that omits `ui.language` must fill it from the default
    // rather than failing the parse — the same `#[serde(default)]` contract every other field honours.
    let body = r#"{ "version": 1, "ui": { "jog_feed": 250.0 } }"#;
    let config = Config::from_json(body).expect("a partial config parses");
    assert_eq!(config.ui.language, crate::i18n::EN_US, "an absent language fills from the default (en-US)");
  }

  #[test]
  fn a_swedish_config_language_resolves_to_swedish_strings() {
    // End-to-end of the config→i18n path: `ui.language = "sv-SE"` round-trips, and selecting it (exactly what
    // `run()` does with `i18n::set_language(&config.ui.language)`) on a registry loaded with the bundled locales
    // makes a nav-button label resolve in Swedish. Tested on an isolated `Translator` for determinism.
    let mut config = Config::default();
    config.ui.language = crate::i18n::SV_SE.to_string();
    let body = config.to_json().expect("serialises");
    let parsed = Config::from_json(&body).expect("round-trips");
    assert_eq!(parsed.ui.language, "sv-SE", "the Swedish locale survives a save/load round trip");

    let mut translator = crate::i18n::Translator::new();
    for (locale, content) in crate::i18n::BUNDLED_LOCALES {
      translator.load_text(locale, content).expect("a bundled locale parses");
    }
    translator.set_fallback(crate::i18n::EN_US);
    translator.set_language(&parsed.ui.language);
    assert_eq!(
      translator.translate("btn-settings", &crate::i18n::fluent::FluentArgs::new()),
      "Inställningar",
      "a `sv-SE` config language drives the Swedish strings",
    );
  }

  /// The bundled asset's name for its self-documenting, fully-keyed template theme. Named `"default"` so it shadows
  /// the built-in default and is the ACTIVE theme out of the box — so editing any `themes.default.<color>` in the
  /// seeded file recolours immediately, with no indirection. Unedited, every value equals its `default_dark` channel,
  /// so it resolves to exactly the built-in default (a true no-op until the operator changes a value).
  const TEMPLATE_THEME: &str = "default";

  #[test]
  fn the_bundled_template_theme_is_a_fully_keyed_no_op_and_is_the_active_theme() {
    // The asset ships a `default` theme (shadowing the built-in) that names EVERY overridable colour field, set to
    // its `default_dark` value — so a first-run operator sees every knob in one place AND can edit it directly (it
    // is the active theme). Four claims: it exists, it is the active theme, it is exhaustively keyed (so the template
    // cannot silently go stale as fields are added), and unedited it resolves to exactly the default palette.
    let config = Config::from_json(BUNDLED_DEFAULT).expect("the bundled asset parses");
    assert_eq!(
      config.appearance.active_theme, TEMPLATE_THEME,
      "the template theme must be the ACTIVE theme so an edit takes effect with no indirection",
    );
    let template = config
      .appearance
      .themes
      .get(TEMPLATE_THEME)
      .unwrap_or_else(|| panic!("the bundled asset must carry a `{TEMPLATE_THEME}` template theme"));
    assert!(
      template.all_color_fields_set(),
      "the template theme must name EVERY overridable colour field — add the new field to config.default.json",
    );
    // Resolve the ACTIVE template and confirm it is identical to the default palette (a true no-op until edited).
    let (palette, notice) = config.palette();
    assert_eq!(notice, None, "the template theme resolves cleanly (it shadows the known built-in `default`)");
    assert_eq!(
      palette,
      Palette::default_dark(),
      "the unedited template must resolve to exactly the default palette — it documents, it does not recolour",
    );
  }

  #[test]
  fn loading_a_missing_file_seeds_it_and_round_trips_the_bundled_asset_with_no_notice() {
    // First-run path: the file does not exist. `load_from` seeds it from the bundled default, then reads it back.
    // The honest invariant (the asset carries a self-documenting template theme, so it is NOT `== Default`): the
    // seeded-then-read config must parse with NO notices and equal what `from_json` makes of the bundled asset — a
    // clean round-trip through disk. The template keeps the asset richer than `Config::default`, by design.
    let dir = std::env::temp_dir().join(format!("skirnir-config-seed-{}", std::process::id()));
    let path = dir.join("nested").join(CONFIG_FILE); // also exercises parent-dir creation.
    let _ = std::fs::remove_dir_all(&dir);
    let (config, notices) = load_from(&path);
    assert!(notices.is_empty(), "seeding a missing file is the ordinary first run — no notice: {notices:?}");
    assert!(path.exists(), "the file must have been seeded to disk for the operator to edit");
    let from_asset = Config::from_json(BUNDLED_DEFAULT).expect("the bundled asset parses");
    assert_eq!(config, from_asset, "the seeded-then-read config must round-trip the bundled asset exactly");
    // The non-appearance sections still match the built-in defaults, so the template theme is the ONLY difference
    // from `Config::default` — the asset documents the appearance without changing any actual default VALUE (the
    // active theme is `"default"` either way; the seed just carries the explicit, editable `themes.default` entry).
    assert_eq!(config.ui, Config::default().ui, "the seeded UI section equals the defaults");
    assert_eq!(config.connection, Config::default().connection, "the seeded connection section equals the defaults");
    assert_eq!(config.toolpath, Config::default().toolpath, "the seeded toolpath section equals the defaults");
    assert_eq!(
      config.appearance.active_theme,
      Config::default().appearance.active_theme,
      "the active theme name is still `default` — same as the built-in default; only now it is an editable entry",
    );
    // And critically: the seeded config's RESOLVED palette equals the default palette, so a fresh install looks
    // identical to before — the template is inert until edited.
    let (palette, _) = config.palette();
    assert_eq!(palette, Palette::default_dark(), "a fresh seed resolves to the default palette (no visual change)");
    let _ = std::fs::remove_dir_all(&dir);
  }

  #[test]
  fn loading_a_corrupt_file_falls_back_to_defaults_with_a_notice_and_never_panics() {
    let dir = std::env::temp_dir().join(format!("skirnir-config-corrupt-{}", std::process::id()));
    let path = dir.join(CONFIG_FILE);
    std::fs::create_dir_all(&dir).expect("temp dir");
    std::fs::write(&path, "this is not valid JSON {{{").expect("write a garbage file");
    let (config, notices) = load_from(&path);
    assert_eq!(config, Config::default(), "a corrupt file must fall back to defaults, not crash");
    assert!(!notices.is_empty(), "a corrupt file should surface a notice so the operator knows defaults are in use");
    let _ = std::fs::remove_dir_all(&dir);
  }

  #[test]
  fn a_bad_hex_colour_falls_back_to_defaults_with_a_notice() {
    // A malformed colour anywhere in a theme override is a deserialize error → defaults + notice, never a panic.
    let dir = std::env::temp_dir().join(format!("skirnir-config-badhex-{}", std::process::id()));
    let path = dir.join(CONFIG_FILE);
    std::fs::create_dir_all(&dir).expect("temp dir");
    let body = r##"{ "appearance": { "themes": { "x": { "accent": "#nothex" } } } }"##;
    std::fs::write(&path, body).expect("write the bad-hex file");
    let (config, notices) = load_from(&path);
    assert_eq!(config, Config::default(), "a bad hex colour must fall back to defaults");
    assert!(!notices.is_empty(), "a bad hex colour must surface a notice");
    let _ = std::fs::remove_dir_all(&dir);
  }

  #[test]
  fn a_config_from_a_newer_version_is_refused_in_favour_of_defaults() {
    // Forward-compat guard: a file written by a future skirnir (higher version) must not be misread.
    let body = format!("{{ \"version\": {} }}", CONFIG_VERSION + 1);
    let err = Config::from_json(&body).expect_err("a newer-versioned config must be refused");
    assert!(err.contains("newer"), "the refusal should explain it is too new: {err}");
  }

  #[test]
  fn an_equal_or_older_version_is_accepted() {
    let body = Config::default().to_json().expect("serialises");
    assert!(Config::from_json(&body).is_ok(), "the current version must be accepted");
    // A version-only minimal file (older/leaner) must parse with every section defaulting.
    let minimal = format!("{{ \"version\": {CONFIG_VERSION} }}");
    let parsed = Config::from_json(&minimal).expect("a version-only config must parse via serde defaults");
    assert_eq!(parsed, Config::default(), "missing sections fill from Default");
  }

  #[test]
  fn a_partial_file_merges_one_color_and_one_ui_key_over_defaults() {
    // The core partial-override claim end to end: a file changing one toolpath colour and one ui key must take those
    // two and leave everything else at the default.
    let body = r#"{
      "ui": { "jog_step_mm": 0.05 },
      "toolpath": { "cut_stroke_px": 3.0 }
    }"#;
    let config = Config::from_json(body).expect("a partial config parses");
    assert_eq!(config.ui.jog_step_mm, 0.05, "the changed ui key is taken");
    assert_eq!(config.ui.jog_feed, UiConfig::default().jog_feed, "the unset ui key defaults");
    assert!((config.toolpath.cut_stroke_px - 3.0).abs() < 1e-6, "the changed toolpath key is taken");
    assert_eq!(config.connection, ConnectionConfig::default(), "an unmentioned section defaults wholesale");
  }

  #[test]
  fn an_unknown_active_theme_surfaces_a_notice_through_load() {
    // A file that parses cleanly but names an unknown theme must still warn the operator (via the load notices) that
    // the active theme was substituted — the soft notice folded in by `finish`.
    let dir = std::env::temp_dir().join(format!("skirnir-config-unknown-theme-{}", std::process::id()));
    let path = dir.join(CONFIG_FILE);
    std::fs::create_dir_all(&dir).expect("temp dir");
    std::fs::write(&path, r#"{ "appearance": { "active_theme": "ghost" } }"#).expect("write");
    let (config, notices) = load_from(&path);
    assert_eq!(config.appearance.active_theme, "ghost", "the config still records what the file asked for");
    assert!(
      notices.iter().any(|n| n.contains("ghost")),
      "an unknown active_theme must surface a substitution notice: {notices:?}",
    );
    let _ = std::fs::remove_dir_all(&dir);
  }

  #[test]
  fn save_then_load_through_a_real_file_round_trips() {
    let dir = std::env::temp_dir().join(format!("skirnir-config-roundtrip-{}", std::process::id()));
    let path = dir.join("nested").join(CONFIG_FILE); // also exercises parent-dir creation.
    let _ = std::fs::remove_dir_all(&dir);
    let mut config = Config::default();
    config.ui.jog_step_mm = 0.25;
    config.appearance.active_theme = "midnight".to_string();
    save_to(&config, &path).expect("saving to a writable temp path succeeds");
    let (loaded, notices) = load_from(&path);
    assert_eq!(loaded, config, "a saved config must load back identically");
    assert!(notices.is_empty(), "a clean round-trip surfaces no notice: {notices:?}");
    let _ = std::fs::remove_dir_all(&dir);
  }

  #[test]
  fn the_serialised_form_is_pretty_json_with_a_trailing_newline() {
    let body = Config::default().to_json().expect("serialises");
    assert!(body.ends_with('\n'), "the file should end with a newline");
    assert!(body.contains("\"version\""), "the version envelope must be written by name");
    assert!(body.contains("\"active_theme\""), "the appearance section is written by name");
  }
}
