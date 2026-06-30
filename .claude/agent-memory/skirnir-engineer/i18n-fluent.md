---
name: i18n-fluent
description: skirnir i18n module — Project Fluent only, pure Translator + global tr! API, modeled on egui-i18n
metadata:
  type: project
---

skirnir has a Fluent-only i18n layer at `crates/skirnir/src/i18n/` (always-on, not feature-gated — pure, serves
CLI + GUI). Modeled on the `egui-i18n` crate's `i18n` module API shape but improved for skirnir's conventions.

**Why:** Wanted egui-i18n's ergonomics (global registry + `tr!` macro + `.ftl` resources) but Fluent exclusively
(no gettext/classic backend), host-testable, and panic-free.

**How to apply / structure:**
- `i18n/fluent.rs` (mounted as submodule `fluent_impl` via `#[path]`) holds the pure, instance-owned
  `Translator` (HashMap<locale, FluentBundle> + language/fallback/use_isolating). ALL logic + the exhaustive
  behavioral tests live here against isolated instances → deterministic, parallel-safe. This is the key
  divergence from egui-i18n, whose process-wide static makes tests interfere.
- `i18n/mod.rs` is the thin egui-i18n-style global wrapper: `static REGISTRY: LazyLock<RwLock<Translator>>`
  (std, no once_cell), free fns set_language/get_language/set_fallback/.../load_translations_from_text/_path,
  `init()` (seeds bundled locales + selects en-US), `translate()`, and the `#[macro_export] tr!` macro. Lock
  poison is recovered via `PoisonError::into_inner` (never unwrap). Global tests serialize on a `Mutex<()>`.
- `tr!("key")` / `tr!("key", { name: val, .. })` (trailing comma ok) → `skirnir::tr!`. Builds
  `fluent::FluentArgs`; values are anything `Into<FluentValue>` (str/int/float). Macro reaches FluentArgs via
  the `pub use ::fluent;` re-export so callers don't dep on fluent directly.
- Bundled locales: `assets/i18n/en-US.ftl` (source/fallback) + `assets/i18n/sv-SE.ftl` (full Swedish), embedded
  via `include_str!` as `i18n::EN_US_FTL`/`i18n::SV_SE_FTL`, both in `i18n::BUNDLED_LOCALES` (`init()` loads
  every entry; `languages()` reports both). Add a string = add a `key = value` line in EVERY locale; add a
  locale = drop `<lang>.ftl`, embed, append to BUNDLED_LOCALES (or load at runtime via
  load_translations_from_path). sv-SE proves the end-to-end switch (config.ui.language = "sv-SE" works).

**Two deliberate divergences from egui-i18n (documented in code):**
1. Missing key returns the KEY ITSELF (visible/identifiable in a tool UI), not "".
2. `use_isolating` defaults to FALSE (LTR app; FSI/PDI marks U+2068/U+2069 would pollute strings + break
   exact-match test assertions). Opt back in via set_use_isolating for bidi.

**Wiring (where it plugs into the app):** `config.ui.language` (UiConfig in config/sections.rs, default
`i18n::EN_US`, serde-default so old configs fill it) is the persisted locale. `app::shell::run()` calls
`i18n::init()` (failure → console notice, never panic) then `i18n::set_language(&config.ui.language)` before the
first frame. First `tr!` call site = the top toolbar buttons in `views::toolbar` (Disconnect/Cancel/Identify/
Connect/Open…/⌂ Home/Settings via `crate::tr!("btn-*")`); the Run/Hold/Stop segmented transport group is NOT
wired yet. To localize more UI, follow that toolbar pattern + add keys to en-US.ftl.

**Deps (aligned):** fluent 0.17 -> fluent-bundle 0.16 -> unic-langid 0.9 / intl-memoizer 0.5. Bundles use
`FluentBundle::new_concurrent` + `intl_memoizer::concurrent::IntlLangMemoizer` so Translator is Send+Sync.
Typed errors: `I18nError` (thiserror) variants Resource/LangId/AddResource/Io.
