//! The Fluent-backed translation registry — a pure, instance-owned `Translator` with no global state.
//!
//! All loading and message-resolution logic lives here so it is deterministic and parallel-safe to unit-test:
//! each test owns its own [`Translator`], so there is no process-wide singleton for tests to fight over (the
//! global API in the parent module is a thin wrapper around exactly one of these). The engine philosophy of
//! the crate — framework-agnostic, host-testable, panic-free — applies equally to i18n.

use std::collections::HashMap;
use std::fs;
use std::path::Path;

use fluent::bundle::FluentBundle;
use fluent::{FluentArgs, FluentResource};
use intl_memoizer::concurrent::IntlLangMemoizer;
use unic_langid::LanguageIdentifier;

/// A concurrent Fluent bundle. `new_concurrent` selects the `Sync` memoizer so a [`Translator`] (and thus the
/// global `RwLock<Translator>`) is `Send + Sync` and can be shared across the UI and engine threads.
type Bundle = FluentBundle<FluentResource, IntlLangMemoizer>;

/// Typed errors from the i18n layer. We surface these rather than `unwrap`/`expect` so a malformed `.ftl`
/// resource, a bad language tag, or an unreadable file is a recoverable condition the caller handles.
#[derive(Debug, thiserror::Error)]
pub enum I18nError {
  /// The `.ftl` content failed to parse as a Fluent resource. `detail` carries the joined parser errors.
  #[error("failed to parse Fluent resource for locale `{language}`: {detail}")]
  Resource {
    /// The locale whose resource failed to parse.
    language: String,
    /// A human-readable join of the underlying Fluent parser errors.
    detail: String,
  },
  /// The language string was not a valid BCP-47 / Unicode language identifier (e.g. `en-US`).
  #[error("invalid language identifier `{language}`: {detail}")]
  LangId {
    /// The offending language string.
    language: String,
    /// The underlying `unic-langid` parse error, rendered.
    detail: String,
  },
  /// The parsed resource could not be added to the bundle (typically a duplicate message id).
  #[error("failed to add Fluent resource for locale `{language}`: {detail}")]
  AddResource {
    /// The locale whose resource could not be added.
    language: String,
    /// A human-readable join of the underlying Fluent errors.
    detail: String,
  },
  /// A filesystem error occurred while loading `.ftl` resources from a path.
  #[error("i/o error reading `{path}`: {source}")]
  Io {
    /// The path that could not be read.
    path: String,
    /// The underlying I/O error.
    source: std::io::Error,
  },
}

/// The translation registry: the active locale, the fallback locale, the isolating-marks toggle, and the
/// loaded Fluent bundles keyed by locale. Construct empty with [`Translator::new`], load with
/// [`Translator::load_text`] / [`Translator::load_path`], then resolve with [`Translator::translate`].
pub struct Translator {
  language: String,
  fallback: String,
  use_isolating: bool,
  bundles: HashMap<String, Bundle>,
}

impl Default for Translator {
  fn default() -> Self {
    Self::new()
  }
}

impl Translator {
  /// Create an empty registry. `use_isolating` defaults to `false`: skirnir is an LTR desktop app, so the
  /// Unicode FSI/PDI isolation marks Fluent injects around placeables would only add invisible noise to its
  /// strings (and break exact-match assertions). Bidirectional callers can opt back in via
  /// [`Translator::set_use_isolating`].
  pub fn new() -> Self {
    Self {
      language: String::new(),
      fallback: String::new(),
      use_isolating: false,
      bundles: HashMap::new(),
    }
  }

  /// The active locale (the empty string until set).
  pub fn language(&self) -> &str {
    &self.language
  }

  /// Set the active locale (e.g. `en-US`). Does not need to be loaded yet.
  pub fn set_language(&mut self, locale: &str) {
    self.language = locale.to_string();
  }

  /// The fallback locale consulted when a key is missing from the active locale.
  pub fn fallback(&self) -> &str {
    &self.fallback
  }

  /// Set the fallback locale (e.g. `en-US`).
  pub fn set_fallback(&mut self, locale: &str) {
    self.fallback = locale.to_string();
  }

  /// Whether Fluent wraps interpolated placeables in Unicode isolation marks. See [`Translator::new`].
  pub fn use_isolating(&self) -> bool {
    self.use_isolating
  }

  /// Toggle the isolating marks. Applies to bundles loaded *after* this call.
  pub fn set_use_isolating(&mut self, value: bool) {
    self.use_isolating = value;
  }

  /// The locales that currently have a loaded bundle.
  pub fn languages(&self) -> Vec<String> {
    self.bundles.keys().cloned().collect()
  }

  /// Whether a bundle is loaded for `locale`.
  pub fn has_language(&self, locale: &str) -> bool {
    self.bundles.contains_key(locale)
  }

  /// Load (or replace) the bundle for `language` from raw `.ftl` `content`. Returns a typed error if the
  /// content does not parse, the language tag is invalid, or the resource cannot be added.
  pub fn load_text(&mut self, language: &str, content: &str) -> Result<(), I18nError> {
    let resource = FluentResource::try_new(content.to_string()).map_err(|(_, errors)| I18nError::Resource {
      language: language.to_string(),
      detail: join_debug(&errors),
    })?;
    let lang_id: LanguageIdentifier = language.parse().map_err(|e| I18nError::LangId {
      language: language.to_string(),
      detail: format!("{e:?}"),
    })?;
    let mut bundle = FluentBundle::new_concurrent(vec![lang_id]);
    bundle.set_use_isolating(self.use_isolating);
    bundle.add_resource(resource).map_err(|errors| I18nError::AddResource {
      language: language.to_string(),
      detail: join_debug(&errors),
    })?;
    self.bundles.insert(language.to_string(), bundle);
    Ok(())
  }

  /// Load every `.ftl` file under `path` (or a single `.ftl` file). Each file's stem becomes its locale id,
  /// so `de-DE.ftl` loads as locale `de-DE`. Non-`.ftl` entries are ignored. The first I/O or parse failure
  /// is returned.
  pub fn load_path(&mut self, path: &Path) -> Result<(), I18nError> {
    let mut files = Vec::new();
    if path.is_file() {
      files.push(path.to_path_buf());
    } else {
      let read_dir = fs::read_dir(path).map_err(|source| I18nError::Io {
        path: path.display().to_string(),
        source,
      })?;
      for entry in read_dir {
        let entry = entry.map_err(|source| I18nError::Io {
          path: path.display().to_string(),
          source,
        })?;
        let file = entry.path();
        let is_ftl = file.extension().map(|ext| ext.eq_ignore_ascii_case("ftl")).unwrap_or(false);
        if is_ftl {
          files.push(file);
        }
      }
    }
    for file in files {
      let Some(stem) = file.file_stem().map(|s| s.to_string_lossy().into_owned()) else {
        continue;
      };
      let content = fs::read_to_string(&file).map_err(|source| I18nError::Io {
        path: file.display().to_string(),
        source,
      })?;
      self.load_text(&stem, &content)?;
    }
    Ok(())
  }

  /// Resolve `key` against the active locale, then the fallback locale. If neither resolves it, the `key`
  /// itself is returned as a visible, identifiable last resort (better for a tool UI than an empty string —
  /// a missing string shows up on screen as its key rather than vanishing). `args` carries named values for
  /// `{ $placeable }` interpolation; build it with the [`crate::tr!`] macro or `fluent::FluentArgs`.
  pub fn translate(&self, key: &str, args: &FluentArgs) -> String {
    let primary = if self.language.is_empty() { self.fallback.as_str() } else { self.language.as_str() };
    if let Some(value) = self.extract(primary, key, args) {
      return value;
    }
    if self.fallback != primary && let Some(value) = self.extract(&self.fallback, key, args) {
      return value;
    }
    key.to_string()
  }

  /// Look `key` up in a single locale's bundle. Returns `None` when the locale is unloaded or the message is
  /// absent (so the caller can fall through), and `Some(value)` when the message exists — even if it formats
  /// to an empty string, which is a legitimate, intentional value.
  fn extract(&self, language: &str, key: &str, args: &FluentArgs) -> Option<String> {
    let bundle = self.bundles.get(language)?;
    let message = bundle.get_message(key)?;
    let pattern = message.value()?;
    let mut errors = Vec::new();
    let value = bundle.format_pattern(pattern, Some(args), &mut errors);
    Some(value.into_owned())
  }
}

/// Join a slice of `Debug` errors into one comma-separated line for an [`I18nError`] detail field.
fn join_debug<E: std::fmt::Debug>(errors: &[E]) -> String {
  errors.iter().map(|e| format!("{e:?}")).collect::<Vec<_>>().join(", ")
}

#[cfg(test)]
mod tests {
  use super::*;

  // A small fixture catalog covering a plain string, an interpolated string, and a plural selector.
  const EN: &str = "\
greeting = Hello
welcome = Welcome, { $name }
items = { $count ->
    [0] no items
    [one] { $count } item
   *[other] { $count } items
  }
";

  fn loaded(language: &str) -> Translator {
    let mut t = Translator::new();
    t.load_text(language, EN).expect("fixture .ftl should load");
    t.set_language(language);
    t.set_fallback(language);
    t
  }

  #[test]
  fn translates_a_plain_key() {
    let t = loaded("en-US");
    assert_eq!(t.translate("greeting", &FluentArgs::new()), "Hello");
  }

  #[test]
  fn interpolates_named_arguments() {
    let t = loaded("en-US");
    let mut args = FluentArgs::new();
    args.set("name", "Galdr");
    // With isolating off (the default), the interpolated value is spliced in without FSI/PDI marks.
    assert_eq!(t.translate("welcome", &args), "Welcome, Galdr");
  }

  #[test]
  fn isolating_defaults_off_so_output_is_clean() {
    let t = loaded("en-US");
    let mut args = FluentArgs::new();
    args.set("name", "X");
    assert!(!t.use_isolating());
    // U+2068 / U+2069 are the FSI/PDI isolation marks; they must be absent with isolating disabled.
    let out = t.translate("welcome", &args);
    assert!(!out.contains('\u{2068}') && !out.contains('\u{2069}'), "unexpected isolation marks: {out:?}");
  }

  #[test]
  fn plural_selector_picks_the_right_variant() {
    let t = loaded("en-US");
    let case = |n: i64| {
      let mut args = FluentArgs::new();
      args.set("count", n);
      t.translate("items", &args)
    };
    assert_eq!(case(0), "no items");
    assert_eq!(case(1), "1 item");
    assert_eq!(case(5), "5 items");
  }

  #[test]
  fn falls_back_when_key_missing_in_active_locale() {
    let mut t = Translator::new();
    // The active locale lacks `greeting`; the fallback locale carries it.
    t.load_text("fr-FR", "farewell = Au revoir\n").expect("fr .ftl should load");
    t.load_text("en-US", EN).expect("en .ftl should load");
    t.set_language("fr-FR");
    t.set_fallback("en-US");
    assert_eq!(t.translate("farewell", &FluentArgs::new()), "Au revoir"); // present in active locale
    assert_eq!(t.translate("greeting", &FluentArgs::new()), "Hello"); // resolved via fallback
  }

  #[test]
  fn missing_key_returns_the_key_itself() {
    let t = loaded("en-US");
    assert_eq!(t.translate("nope-not-here", &FluentArgs::new()), "nope-not-here");
  }

  #[test]
  fn languages_lists_loaded_bundles() {
    let mut t = Translator::new();
    t.load_text("en-US", EN).expect("load en");
    t.load_text("fr-FR", "x = y\n").expect("load fr");
    let mut langs = t.languages();
    langs.sort();
    assert_eq!(langs, vec!["en-US".to_string(), "fr-FR".to_string()]);
    assert!(t.has_language("en-US") && !t.has_language("de-DE"));
  }

  #[test]
  fn malformed_resource_is_a_typed_error() {
    let mut t = Translator::new();
    // A bare placeable with no message id is a Fluent syntax error.
    let err = t.load_text("en-US", "= { $x }\n").unwrap_err();
    assert!(matches!(err, I18nError::Resource { .. }), "expected Resource error, got {err:?}");
  }

  #[test]
  fn invalid_language_tag_is_a_typed_error() {
    let mut t = Translator::new();
    let err = t.load_text("not a valid tag", "k = v\n").unwrap_err();
    assert!(matches!(err, I18nError::LangId { .. }), "expected LangId error, got {err:?}");
  }
}
