//! Internationalization for eitri-app — Fluent (Project Fluent) exclusively, mirroring skirnir's i18n layer.
//!
//! A process-wide registry behind free functions (`set_language` / `get_language` / `set_fallback` /
//! `languages` / `load_*` / `translate`) plus a [`tr!`] macro for ergonomic, named-argument interpolation at
//! call sites. All logic lives in the pure, instance-owned [`Translator`] ([`fluent`] submodule); the globals
//! here are a thin wrapper around one [`Translator`], so the heavy testing happens deterministically against
//! instances and the globals carry only smoke tests.
//!
//! # Quick start
//! ```no_run
//! use eitri_app::{i18n, tr};
//! i18n::init().expect("bundled en-US is valid");            // seed + select the embedded locale
//! let label = tr!("btn-undo");                              // -> "Undo"
//! let line = tr!("job-lines", { count: 42 });               // named interpolation
//! # let _ = (label, line);
//! ```
//!
//! # Adding a string
//! Add a `key = value` line to `assets/i18n/en-US.ftl` (and each sibling locale), then call `tr!("key")`.
//!
//! # Adding a locale
//! Drop a `<lang>.ftl` next to `en-US.ftl`, embed it with `include_str!`, and add it to [`BUNDLED_LOCALES`].
//! On-disk locale packs can instead be loaded at runtime with [`load_translations_from_path`].

use std::sync::{LazyLock, PoisonError, RwLock};

// Re-export the `fluent` crate so the `tr!` macro can name `fluent::FluentArgs` through `$crate` without the
// caller depending on `fluent` directly. Also re-export the pure registry and its error for callers that want
// an isolated, non-global instance (tests, headless tools).
pub use ::fluent;
pub use fluent_impl::{I18nError, Translator};

// The submodule is named `fluent_impl` on disk to avoid colliding with the re-exported `fluent` crate above.
#[path = "fluent.rs"]
mod fluent_impl;

/// The locale code of the bundled source/fallback locale.
pub const EN_US: &str = "en-US";

/// The embedded `en-US` Fluent resource (the source strings and the fallback). Bundled into the binary so the
/// app is translated out of the box with no external files.
pub const EN_US_FTL: &str = include_str!("../../assets/i18n/en-US.ftl");

/// The locale code of the bundled Swedish locale.
pub const SV_SE: &str = "sv-SE";

/// The embedded `sv-SE` Fluent resource — a full Swedish translation of every `en-US` key.
pub const SV_SE_FTL: &str = include_str!("../../assets/i18n/sv-SE.ftl");

/// The locales compiled into the binary, as `(locale, ftl_content)` pairs. To bundle a new locale, add its
/// `include_str!`-ed resource here; [`init`] loads every entry.
pub const BUNDLED_LOCALES: &[(&str, &str)] = &[(EN_US, EN_US_FTL), (SV_SE, SV_SE_FTL)];

/// The single global registry the free functions wrap. Tests that need isolation use a [`Translator`]
/// directly instead of touching this.
static REGISTRY: LazyLock<RwLock<Translator>> = LazyLock::new(|| RwLock::new(Translator::new()));

/// Acquire the registry for reading, recovering from a poisoned lock rather than panicking — a translation
/// lookup must never bring the UI down, so we accept the (logically intact) inner guard.
fn read() -> std::sync::RwLockReadGuard<'static, Translator> {
  REGISTRY.read().unwrap_or_else(PoisonError::into_inner)
}

/// Acquire the registry for writing, recovering from a poisoned lock rather than panicking.
fn write() -> std::sync::RwLockWriteGuard<'static, Translator> {
  REGISTRY.write().unwrap_or_else(PoisonError::into_inner)
}

/// Load every [`BUNDLED_LOCALES`] resource into the global registry and select [`EN_US`] as both the active
/// and the fallback locale. Idempotent: calling it again reloads the embedded resources. Returns the first
/// parse error (which would indicate a build-time-invalid bundled `.ftl`).
pub fn init() -> Result<(), I18nError> {
  let mut reg = write();
  for (locale, content) in BUNDLED_LOCALES {
    reg.load_text(locale, content)?;
  }
  reg.set_language(EN_US);
  reg.set_fallback(EN_US);
  Ok(())
}

/// Set the active locale (e.g. `en-US`). It need not be loaded yet; missing keys fall through to the fallback.
pub fn set_language(locale: &str) {
  write().set_language(locale);
}

/// The active locale (empty until set).
pub fn get_language() -> String {
  read().language().to_string()
}

/// Set the fallback locale consulted when a key is missing from the active locale.
pub fn set_fallback(locale: &str) {
  write().set_fallback(locale);
}

/// The fallback locale (empty until set).
pub fn get_fallback() -> String {
  read().fallback().to_string()
}

/// The locales currently loaded in the global registry.
pub fn languages() -> Vec<String> {
  read().languages()
}

/// Load (or replace) the global bundle for `language` from raw `.ftl` `content`.
pub fn load_translations_from_text(language: &str, content: &str) -> Result<(), I18nError> {
  write().load_text(language, content)
}

/// Load every `.ftl` file under `path` (or a single file) into the global registry; each file's stem becomes
/// its locale id. Lets users drop a locale pack beside the binary without a rebuild.
pub fn load_translations_from_path(path: impl AsRef<std::path::Path>) -> Result<(), I18nError> {
  write().load_path(path.as_ref())
}

/// Resolve `key` through the global registry's active-then-fallback locale, interpolating `args`. Prefer the
/// [`tr!`] macro at call sites; this is the function it expands to.
pub fn translate(key: &str, args: &fluent::FluentArgs) -> String {
  read().translate(key, args)
}

/// Translate a Fluent message key, with optional named arguments for `{ $placeable }` interpolation.
///
/// ```ignore
/// tr!("btn-undo");                       // no arguments
/// tr!("job-lines", { count: 42 });       // named arguments
/// ```
///
/// Argument values may be anything `Into<fluent::FluentValue>` (strings, integers, floats). The expansion
/// goes through [`translate`], so a missing key renders as the key itself.
#[macro_export]
macro_rules! tr {
  ($key:expr, { $($name:ident: $val:expr),* $(,)? }) => {{
    let mut args = $crate::i18n::fluent::FluentArgs::new();
    $(
      args.set(stringify!($name), $val);
    )*
    $crate::i18n::translate($key, &args)
  }};
  ($key:expr) => {{
    $crate::i18n::translate($key, &$crate::i18n::fluent::FluentArgs::new())
  }};
}

/// A process-wide serialization guard for ANY test — in this module or elsewhere in the crate — that touches
/// the GLOBAL i18n registry (via [`init`] / [`set_language`]) or asserts a locale-specific `tr!` result.
/// `tr!` resolves against ONE global registry, so a test that sets a non-default locale and a test that reads
/// the default one must not run concurrently. Every such test holds THIS single guard for its whole body.
#[cfg(test)]
pub(crate) fn lock_global_for_test() -> std::sync::MutexGuard<'static, ()> {
  static GUARD: std::sync::Mutex<()> = std::sync::Mutex::new(());
  GUARD.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn init_seeds_bundled_en_us() {
    let _g = lock_global_for_test();
    init().expect("bundled en-US must be valid");
    assert_eq!(get_language(), EN_US);
    assert_eq!(get_fallback(), EN_US);
    assert!(languages().contains(&EN_US.to_string()));
    assert_eq!(tr!("btn-undo"), "Undo");
  }

  #[test]
  fn tr_macro_interpolates_named_args_and_plurals() {
    let _g = lock_global_for_test();
    init().expect("bundled en-US must be valid");
    assert_eq!(tr!("job-lines", { count: 1 }), "1 line of G-code");
    assert_eq!(tr!("job-lines", { count: 42 }), "42 lines of G-code");
    assert_eq!(tr!("op-failed", { label: "Isolation routing", reason: "cancelled", }),
      "Isolation routing failed: cancelled");
  }

  #[test]
  fn tr_macro_handles_missing_key_via_key_itself() {
    let _g = lock_global_for_test();
    init().expect("bundled en-US must be valid");
    assert_eq!(tr!("totally-unknown-key"), "totally-unknown-key");
  }

  /// Build a [`Translator`] loaded with every [`BUNDLED_LOCALES`] entry and `en-US` as the fallback — the
  /// isolated, deterministic mirror of what [`init`] does to the global registry.
  fn bundled_translator() -> Translator {
    let mut t = Translator::new();
    for (locale, content) in BUNDLED_LOCALES {
      t.load_text(locale, content).expect("a bundled locale must parse");
    }
    t.set_fallback(EN_US);
    t
  }

  #[test]
  fn switching_to_swedish_and_back_translates_end_to_end() {
    let mut t = bundled_translator();
    let none = fluent::FluentArgs::new;

    t.set_language(EN_US);
    assert_eq!(t.translate("btn-undo", &none()), "Undo");
    assert_eq!(t.translate("app-settings-title", &none()), "Application Settings");

    t.set_language(SV_SE);
    assert_eq!(t.translate("btn-undo", &none()), "Ångra");
    assert_eq!(t.translate("btn-open-gerber", &none()), "Öppna Gerber…");
    assert_eq!(t.translate("app-settings-title", &none()), "Programinställningar");
    let counted = |n: i64| {
      let mut args = fluent::FluentArgs::new();
      args.set("count", n);
      t.translate("status-objects", &args)
    };
    assert_eq!(counted(0), "inga objekt");
    assert_eq!(counted(1), "1 objekt");
    assert_eq!(counted(3), "3 objekt");

    t.set_language(EN_US);
    assert_eq!(t.translate("btn-undo", &none()), "Undo");
  }

  /// Collect the top-level Fluent message ids declared in a `.ftl` source, so the parity test can compare key
  /// sets across locales without a Fluent runtime introspection API.
  fn top_level_message_ids(ftl: &str) -> std::collections::BTreeSet<String> {
    let mut ids = std::collections::BTreeSet::new();
    for line in ftl.lines() {
      if line.is_empty() || line.starts_with('#') || line.starts_with(char::is_whitespace) {
        continue;
      }
      let Some((key, _)) = line.split_once('=') else { continue };
      let key = key.trim();
      if !key.is_empty()
        && key.chars().next().is_some_and(|c| c.is_ascii_alphabetic())
        && key.chars().all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_')
      {
        ids.insert(key.to_string());
      }
    }
    ids
  }

  #[test]
  fn every_bundled_locale_declares_the_same_message_ids() {
    // The source locale is en-US; every sibling must translate exactly its keys — none missing (an untranslated
    // string) and none orphaned (a rename left behind).
    let en = top_level_message_ids(EN_US_FTL);
    assert!(!en.is_empty(), "en-US must declare message ids");
    for (locale, content) in BUNDLED_LOCALES {
      if *locale == EN_US {
        continue;
      }
      let other = top_level_message_ids(content);
      let missing: Vec<&String> = en.difference(&other).collect();
      let orphan: Vec<&String> = other.difference(&en).collect();
      assert!(missing.is_empty(), "{locale} is missing keys present in en-US: {missing:?}");
      assert!(orphan.is_empty(), "{locale} has orphan keys absent from en-US: {orphan:?}");
    }
  }

  #[test]
  fn every_bundled_locale_resolves_every_key_without_falling_back_to_the_id() {
    // Beyond key parity, each locale must actually FORMAT every message — a present-but-broken value (e.g. a
    // malformed selector) would surface as the key itself at runtime.
    let en = top_level_message_ids(EN_US_FTL);
    for (locale, content) in BUNDLED_LOCALES {
      let mut t = Translator::new();
      t.load_text(locale, content).expect("a bundled locale must parse");
      t.set_language(locale);
      for key in &en {
        // Supply a superset of the interpolation args any message might reference, so a formatted value never
        // renders as the key merely for want of an argument.
        let mut args = fluent::FluentArgs::new();
        for name in ["count", "label", "reason", "path", "dialect", "polygons", "polylines", "hits", "tools",
          "done", "total"] {
          args.set(name, 1);
        }
        let value = t.translate(key, &args);
        assert!(!value.is_empty(), "{locale}:{key} formatted to an empty string");
        assert_ne!(&value, key, "{locale}:{key} did not resolve (rendered as its own key)");
      }
    }
  }
}
