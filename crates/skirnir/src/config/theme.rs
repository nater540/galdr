//! The config's appearance section: the `active_theme` selector, the font scale, and the user-defined theme
//! overrides, plus the layered resolution that turns all of that into a fully-populated runtime [`Palette`].
//!
//! **Built-ins live in code, user themes live in the file.** The built-in names (`"default"`, `"light-slate"`,
//! `"midnight"`) resolve to [`Palette`] presets in [`crate::app::theme`]. A user theme is a [`ThemeOverride`]: an
//! optional `base` (another theme name, defaulting to `"default"`) plus a `Some`-per-field set of colour overrides.
//! Resolution starts from the base palette and applies only the override's `Some(...)` fields, so a user theme that
//! recolours one token inherits the other ~30 from its base. An `active_theme` that names nothing known falls back to
//! `"default"` and surfaces a notice — never a panic, never a blank palette.
//!
//! Every field is `#[serde(default)]` (the `Option` fields default to `None` = "inherit"), so a partial theme block
//! merges over the defaults like every other section.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use super::color::ColorSpec;
use crate::app::theme::Palette;

/// The built-in theme name used as the universal fallback and the default `base` for a user override.
pub const DEFAULT_THEME: &str = "default";

/// The appearance section of the config: which theme is active, a global font scale, and the user-defined themes.
/// Built-in themes are NOT listed here (they live in code); `themes` carries only operator-authored overrides.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AppearanceConfig {
  /// The name of the active theme: a built-in (`"default"`/`"light-slate"`/`"midnight"`) or a key in [`Self::themes`].
  /// An unknown name resolves to [`DEFAULT_THEME`] with a notice.
  pub active_theme: String,
  /// A global UI font scale (1.0 = the design's sizes). Held to a sane range at apply time so a typo cannot make the
  /// UI unreadable.
  pub font_scale: f32,
  /// Operator-authored theme overrides, keyed by name. A `BTreeMap` so the on-disk order is stable (diff-friendly).
  pub themes: BTreeMap<String, ThemeOverride>,
}

impl Default for AppearanceConfig {
  fn default() -> Self {
    AppearanceConfig {
      active_theme: DEFAULT_THEME.to_string(),
      font_scale: 1.0,
      themes: BTreeMap::new(),
    }
  }
}

impl AppearanceConfig {
  /// Resolve the active theme into a fully-populated runtime [`Palette`], returning the palette plus an optional
  /// notice (e.g. an unknown `active_theme`, or a user theme whose explicit `base` is unknown). Never panics: any
  /// miss falls back to the default palette. A user `themes` entry takes precedence over a same-named built-in (see
  /// [`Self::resolve_named`]); a built-in name with no user entry resolves directly to its preset.
  ///
  /// Resolution layering: the result starts from the resolved *base* palette, then the override's `Some(...)` colour
  /// fields are applied on top — so an override need only carry the tokens it changes. A short recursion guard caps
  /// explicit `base` chains so a cycle cannot loop forever.
  pub fn resolve_palette(&self) -> (Palette, Option<String>) {
    self.resolve_named(&self.active_theme, 0)
  }

  /// Resolve a theme *by name* with a depth guard, returning the palette and an optional notice.
  ///
  /// **Precedence: a user `themes` entry WINS over a built-in of the same name.** This is what lets the intuitive
  /// `themes.default` recolour the default theme — we consult `self.themes` FIRST and only fall back to
  /// [`builtin_palette`] when no user entry exists. (The earlier order short-circuited to the built-in, so a
  /// `themes.default` was silently ignored — the reported bug.)
  ///
  /// Base resolution for a user entry:
  /// - An EXPLICIT `over.base` resolves recursively via this same function (so `base: "midnight"` layers over the
  ///   midnight preset, and a multi-level user chain still works), bounded by [`MAX_BASE_DEPTH`] — UNLESS the base
  ///   names this same entry (`themes.default` with `base: "default"`, the intuitive-but-circular form), which is
  ///   short-circuited to the same-name built-in directly so it cannot self-recurse.
  /// - An IMPLICIT base (`over.base` is `None`) resolves to the SAME-NAME built-in if one exists, else the
  ///   [`DEFAULT_THEME`] built-in — taken DIRECTLY from [`builtin_palette`], NOT back through `self.themes`. This is
  ///   what makes `themes.default` inherit the built-in default's other ~35 tokens without re-entering its own entry
  ///   (so the depth guard is not load-bearing for the common shadowing case — there is simply no self-recursion).
  ///
  /// An unknown name, or a base chain deeper than [`MAX_BASE_DEPTH`], falls back to the default palette with a notice.
  fn resolve_named(&self, name: &str, depth: usize) -> (Palette, Option<String>) {
    if depth > MAX_BASE_DEPTH {
      return (Palette::default_dark(), Some(format!("theme base chain for {name:?} is too deep — using the default")));
    }
    // User entries take precedence over a same-named built-in, so a `themes.<builtin>` override is honoured.
    if let Some(over) = self.themes.get(name) {
      let (base, base_notice) = match over.base.as_deref() {
        // A self-referential base (`themes.default` with `base: "default"`, the intuitive-but-circular form) is
        // resolved as the SAME-NAME built-in directly — exactly like the implicit case — so it does not recurse back
        // into its own entry and trip the depth guard. The shadow inherits the built-in's other tokens, as intended.
        Some(base_name) if base_name == name => {
          (builtin_palette(name).unwrap_or_else(Palette::default_dark), None)
        }
        // Any other explicit base: resolve it recursively (it may itself be a user theme or a built-in), depth-bounded.
        Some(base_name) => self.resolve_named(base_name, depth + 1),
        // Implicit base: the same-name built-in if one exists, else the default built-in — taken DIRECTLY so a
        // shadowing entry never recurses into itself. No notice: an implicit built-in base always resolves.
        None => (builtin_palette(name).unwrap_or_else(Palette::default_dark), None),
      };
      return (over.apply_over(base), base_notice);
    }
    // No user entry: a built-in name short-circuits to its preset.
    if let Some(builtin) = builtin_palette(name) {
      return (builtin, None);
    }
    // Neither a user theme nor a built-in: fall back to the default with a notice.
    (
      Palette::default_dark(),
      Some(format!("theme {name:?} is not a built-in or a defined theme — using the default")),
    )
  }
}

/// The maximum `base` chain depth resolved before bailing to the default, so a `base` cycle (a → b → a) or a silly
/// deep chain cannot loop or blow the stack. A handful of levels is far more than any real theme hierarchy needs.
const MAX_BASE_DEPTH: usize = 8;

/// Map a built-in theme name to its [`Palette`] preset, or `None` when the name is not a built-in (then it is looked
/// up among the user themes). The names are the stable on-disk tokens; keep them in sync with the presets.
pub fn builtin_palette(name: &str) -> Option<Palette> {
  match name {
    "default" => Some(Palette::default_dark()),
    "light-slate" => Some(Palette::light_slate()),
    "midnight" => Some(Palette::midnight()),
    _ => None,
  }
}

/// The single source of truth for the set of overridable palette colour fields. Invokes `$callback!` with the full
/// comma-separated list of field idents, so the three places that must stay in lockstep — [`ThemeOverride::apply_over`]
/// (layer each set field over a base), [`ThemeOverride::all_color_fields_set`] (the template stale-guard), and the
/// test's `full_override_from` (build an exhaustive override) — are all generated from THIS one list. Adding a new
/// palette token means adding it here (and to the [`ThemeOverride`] struct + [`Palette`]); the three call sites then
/// update automatically and the asset/stale-guard tests fail until the bundled template is extended. `base` is NOT in
/// this list — it is the inheritance pointer, not a colour token.
macro_rules! palette_color_fields {
  ($callback:ident) => {
    $callback! {
      bg, panel, panel_alt, inset, widget, widget_hover, widget_active, divider, border_recess, border_raised,
      accent, accent_hover, accent_active, accent_motion,
      text, text_dim, text_disabled,
      state_idle, state_run, state_hold, state_alarm, state_jog, state_check, state_neutral,
      alarm_bg, alarm_border, alarm_text,
      log_sent, log_recv, log_status, log_info, log_notice,
      toolpath_cut, toolpath_rapid, grid_major, grid_minor
    }
  };
}

/// A user-defined theme: an optional `base` to inherit from (default `"default"`) plus a per-field set of colour
/// overrides. Each field is `Option<ColorSpec>` — `None` (the serde default) inherits from the base, `Some(spec)`
/// overrides it. So a theme that recolours only the accent carries one `Some` and inherits the rest. Field names
/// match [`Palette`]'s so the JSON reads obviously (`"accent": "#FF00FF"`). The per-field list is kept in lockstep
/// with the resolution logic via [`palette_color_fields!`].
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct ThemeOverride {
  /// The theme this one inherits unset fields from. `None` (the default) means `"default"`.
  pub base: Option<String>,

  // Chrome & surface.
  /// Override for `bg.window`.
  pub bg: Option<ColorSpec>,
  /// Override for `bg.panel`.
  pub panel: Option<ColorSpec>,
  /// Override for `bg.panelAlt`.
  pub panel_alt: Option<ColorSpec>,
  /// Override for `bg.inset`.
  pub inset: Option<ColorSpec>,
  /// Override for `bg.widget`.
  pub widget: Option<ColorSpec>,
  /// Override for `bg.widget.hover`.
  pub widget_hover: Option<ColorSpec>,
  /// Override for `bg.widget.active`.
  pub widget_active: Option<ColorSpec>,
  /// Override for `divider`.
  pub divider: Option<ColorSpec>,
  /// Override for `border.recess`.
  pub border_recess: Option<ColorSpec>,
  /// Override for `border.raised`.
  pub border_raised: Option<ColorSpec>,

  // Accents.
  /// Override for `accent.primary`.
  pub accent: Option<ColorSpec>,
  /// Override for `accent.primary` hover.
  pub accent_hover: Option<ColorSpec>,
  /// Override for `accent.primary` active.
  pub accent_active: Option<ColorSpec>,
  /// Override for `accent.secondary` (motion).
  pub accent_motion: Option<ColorSpec>,

  // Text.
  /// Override for `text.primary`.
  pub text: Option<ColorSpec>,
  /// Override for `text.secondary`.
  pub text_dim: Option<ColorSpec>,
  /// Override for `text.disabled`.
  pub text_disabled: Option<ColorSpec>,

  // Machine-state dots.
  /// Override for the Idle dot.
  pub state_idle: Option<ColorSpec>,
  /// Override for the Run dot.
  pub state_run: Option<ColorSpec>,
  /// Override for the Hold dot.
  pub state_hold: Option<ColorSpec>,
  /// Override for the Alarm dot.
  pub state_alarm: Option<ColorSpec>,
  /// Override for the Jog dot.
  pub state_jog: Option<ColorSpec>,
  /// Override for the Check/Sleep accent.
  pub state_check: Option<ColorSpec>,
  /// Override for the Sleep/Disconnected neutral.
  pub state_neutral: Option<ColorSpec>,

  // Alarm surface trio.
  /// Override for the alarm surface fill.
  pub alarm_bg: Option<ColorSpec>,
  /// Override for the alarm surface border.
  pub alarm_border: Option<ColorSpec>,
  /// Override for the alarm surface text.
  pub alarm_text: Option<ColorSpec>,

  // Console line types.
  /// Override for the sent-line chevron.
  pub log_sent: Option<ColorSpec>,
  /// Override for the response chevron.
  pub log_recv: Option<ColorSpec>,
  /// Override for the status chevron.
  pub log_status: Option<ColorSpec>,
  /// Override for the info/`[MSG:]` line.
  pub log_info: Option<ColorSpec>,
  /// Override for the dim notice/timestamp.
  pub log_notice: Option<ColorSpec>,

  // Toolpath viewport.
  /// Override for the cut trail colour.
  pub toolpath_cut: Option<ColorSpec>,
  /// Override for the rapid trail colour.
  pub toolpath_rapid: Option<ColorSpec>,
  /// Override for the grid major-line colour.
  pub grid_major: Option<ColorSpec>,
  /// Override for the grid minor-line colour.
  pub grid_minor: Option<ColorSpec>,
}

impl ThemeOverride {
  /// Apply this override's set (`Some`) colour fields over a base palette, returning the layered result. Every unset
  /// (`None`) field inherits the base value, so a sparse override changes only the tokens it names. Pure — the base
  /// is consumed and returned mutated, so resolution is a simple fold. The field set is generated from the single
  /// [`palette_color_fields!`] list so it can never drift from the struct or the stale-guard.
  pub fn apply_over(&self, mut base: Palette) -> Palette {
    macro_rules! apply_each {
      ($($field:ident),+ $(,)?) => {
        $(
          if let Some(spec) = self.$field {
            base.$field = spec.to_color32();
          }
        )+
      };
    }
    palette_color_fields!(apply_each);
    base
  }

  /// Capture a resolved [`Palette`] back into a FULLY-KEYED override: every colour field `Some`, `base: None`.
  /// This is how the in-app theme editor materialises a new user theme — it snapshots whatever palette is active
  /// into concrete per-token values, so every colour picker has a real value to edit and the theme no longer
  /// depends on its base changing underneath it. Generated from the single [`palette_color_fields!`] list, so a
  /// new palette token extends this automatically.
  pub fn from_palette(palette: &Palette) -> Self {
    macro_rules! capture {
      ($($field:ident),+ $(,)?) => {
        ThemeOverride {
          base: None,
          $($field: Some(ColorSpec::from_color32(palette.$field)),)+
        }
      };
    }
    palette_color_fields!(capture)
  }

  /// Every colour slot as `(token name, mutable slot)`, in the palette's declaration order — the theme editor's
  /// iteration surface, so the picker list is generated from the same single [`palette_color_fields!`] list as the
  /// resolution logic and can never miss a token. The name is the `Palette` field ident (also the config's JSON
  /// key), so what the editor shows is exactly what the file says.
  pub fn color_entries_mut(&mut self) -> Vec<(&'static str, &mut Option<ColorSpec>)> {
    macro_rules! entries {
      ($($field:ident),+ $(,)?) => {
        vec![$((stringify!($field), &mut self.$field),)+]
      };
    }
    palette_color_fields!(entries)
  }

  /// Whether EVERY overridable colour field is `Some` — i.e. this override names every token, not just a subset.
  /// Used to guard the bundled `config.default.json`'s self-documenting template theme: if a new palette token is
  /// added but the template entry is not extended, this returns `false` and the asset test fails, so the template
  /// cannot silently go stale. Generated from the single [`palette_color_fields!`] list, so it stays exhaustive
  /// automatically. `base` is intentionally excluded — it is the inheritance pointer, not a token.
  pub fn all_color_fields_set(&self) -> bool {
    macro_rules! all_set {
      ($($field:ident),+ $(,)?) => {
        $(self.$field.is_some())&&+
      };
    }
    palette_color_fields!(all_set)
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use eframe::egui::Color32;

  #[test]
  fn the_default_appearance_resolves_to_the_default_dark_palette_with_no_notice() {
    let (palette, notice) = AppearanceConfig::default().resolve_palette();
    assert_eq!(palette, Palette::default_dark(), "the default active_theme is the design dark palette");
    assert_eq!(notice, None, "resolving a known built-in surfaces no notice");
  }

  #[test]
  fn a_built_in_name_resolves_to_its_preset() {
    let appearance = AppearanceConfig { active_theme: "midnight".to_string(), ..Default::default() };
    let (palette, notice) = appearance.resolve_palette();
    assert_eq!(palette, Palette::midnight());
    assert_eq!(notice, None);
  }

  #[test]
  fn an_unknown_active_theme_falls_back_to_default_with_a_notice() {
    let appearance = AppearanceConfig { active_theme: "neon-dreams".to_string(), ..Default::default() };
    let (palette, notice) = appearance.resolve_palette();
    assert_eq!(palette, Palette::default_dark(), "an unknown theme name must use the default palette");
    assert!(notice.is_some_and(|n| n.contains("neon-dreams")), "the notice should name the missing theme");
  }

  #[test]
  fn a_user_override_layers_only_its_set_fields_over_its_base() {
    // A user theme that recolours just the accent must inherit every other token from its base ("default" here).
    let mut themes = BTreeMap::new();
    themes.insert(
      "my-theme".to_string(),
      ThemeOverride { accent: ColorSpec::parse("#FF00FF"), ..Default::default() },
    );
    let appearance = AppearanceConfig { active_theme: "my-theme".to_string(), themes, ..Default::default() };
    let (palette, notice) = appearance.resolve_palette();
    assert_eq!(notice, None, "a well-formed user theme over a known base surfaces no notice");
    assert_eq!(palette.accent, Color32::from_rgb(0xFF, 0x00, 0xFF), "the one overridden field is applied");
    assert_eq!(palette.panel, Palette::default_dark().panel, "every unset field falls through to the base");
    assert_eq!(palette.text, Palette::default_dark().text, "text inherits from the base too");
  }

  #[test]
  fn a_user_override_can_base_on_another_built_in() {
    // `base: "midnight"` means unset fields come from the midnight preset, not the default.
    let mut themes = BTreeMap::new();
    themes.insert(
      "tweaked-midnight".to_string(),
      ThemeOverride {
        base: Some("midnight".to_string()),
        accent: ColorSpec::parse("#00FF00"),
        ..Default::default()
      },
    );
    let appearance =
      AppearanceConfig { active_theme: "tweaked-midnight".to_string(), themes, ..Default::default() };
    let (palette, _) = appearance.resolve_palette();
    assert_eq!(palette.accent, Color32::from_rgb(0x00, 0xFF, 0x00), "the override wins");
    assert_eq!(palette.panel, Palette::midnight().panel, "unset fields come from the named base, not the default");
  }

  #[test]
  fn a_base_cycle_is_bounded_and_falls_back_rather_than_looping() {
    // A pathological a→b→a cycle must terminate (depth guard) rather than recursing forever.
    let mut themes = BTreeMap::new();
    themes.insert("a".to_string(), ThemeOverride { base: Some("b".to_string()), ..Default::default() });
    themes.insert("b".to_string(), ThemeOverride { base: Some("a".to_string()), ..Default::default() });
    let appearance = AppearanceConfig { active_theme: "a".to_string(), themes, ..Default::default() };
    let (palette, notice) = appearance.resolve_palette();
    // It must return *some* palette (the default fallback) and a notice, not hang.
    assert_eq!(palette.accent, Palette::default_dark().accent);
    assert!(notice.is_some(), "a cycle must surface a notice");
  }

  #[test]
  fn an_override_with_an_unknown_base_falls_back_to_the_default_base_with_a_notice() {
    let mut themes = BTreeMap::new();
    themes.insert(
      "orphan".to_string(),
      ThemeOverride { base: Some("ghost".to_string()), accent: ColorSpec::parse("#123456"), ..Default::default() },
    );
    let appearance = AppearanceConfig { active_theme: "orphan".to_string(), themes, ..Default::default() };
    let (palette, notice) = appearance.resolve_palette();
    assert_eq!(palette.accent, Color32::from_rgb(0x12, 0x34, 0x56), "the override's own field still applies");
    assert_eq!(palette.panel, Palette::default_dark().panel, "the unknown base falls back to the default base");
    assert!(notice.is_some_and(|n| n.contains("ghost")), "the notice should name the missing base");
  }

  #[test]
  fn a_user_themes_entry_named_default_overrides_the_built_in_and_inherits_the_rest() {
    // The reported bug: the intuitive `themes.default` with `active_theme: "default"` must APPLY its override and
    // inherit every other token from the built-in default. Before the precedence fix the built-in short-circuited
    // and this was silently ignored.
    let mut themes = BTreeMap::new();
    themes.insert("default".to_string(), ThemeOverride { accent_motion: ColorSpec::parse("#FF00FF"), ..Default::default() });
    let appearance = AppearanceConfig { active_theme: "default".to_string(), themes, ..Default::default() };
    let (palette, notice) = appearance.resolve_palette();
    assert_eq!(notice, None, "shadowing a built-in with a user entry of the same name surfaces no notice");
    assert_eq!(palette.accent_motion, Color32::from_rgb(0xFF, 0x00, 0xFF), "the user override of `default` must apply");
    assert_eq!(palette.accent, Palette::default_dark().accent, "every unset field inherits the built-in default");
    assert_eq!(palette.panel, Palette::default_dark().panel, "chrome inherits the built-in default too");
  }

  #[test]
  fn a_user_themes_entry_shadowing_a_non_default_built_in_inherits_that_built_in() {
    // A `themes.midnight` override (active) must inherit MIDNIGHT's other tokens via its implicit same-name base —
    // not the default's. So the override changes one token and keeps midnight's chrome.
    let mut themes = BTreeMap::new();
    themes.insert("midnight".to_string(), ThemeOverride { accent: ColorSpec::parse("#00FF00"), ..Default::default() });
    let appearance = AppearanceConfig { active_theme: "midnight".to_string(), themes, ..Default::default() };
    let (palette, notice) = appearance.resolve_palette();
    assert_eq!(notice, None, "shadowing the midnight built-in surfaces no notice");
    assert_eq!(palette.accent, Color32::from_rgb(0x00, 0xFF, 0x00), "the user override applies");
    assert_eq!(palette.panel, Palette::midnight().panel, "unset fields inherit the SHADOWED built-in (midnight)");
    assert_ne!(palette.panel, Palette::default_dark().panel, "and NOT the default — the implicit base is same-name");
  }

  #[test]
  fn a_shadowing_entry_whose_implicit_base_is_its_own_name_does_not_recurse() {
    // The recursion-safety guarantee: a `themes.default` whose implicit base is "default" must resolve the base
    // DIRECTLY from the built-in, never re-entering its own entry. We prove it terminates and is correct even with a
    // depth budget of 0 effectively (a single shadow with no explicit base must not consume the depth guard). Build
    // the override over a name that is ALSO a built-in, with no explicit base, and confirm it resolves cleanly.
    let mut themes = BTreeMap::new();
    themes.insert(
      "default".to_string(),
      ThemeOverride { text: ColorSpec::parse("#ABCDEF"), ..Default::default() },
    );
    let appearance = AppearanceConfig { active_theme: "default".to_string(), themes, ..Default::default() };
    // If the implicit same-name base re-entered `self.themes`, this would recurse until the depth guard fired and
    // surface a "too deep" notice. It must instead resolve cleanly with the override applied.
    let (palette, notice) = appearance.resolve_palette();
    assert_eq!(notice, None, "the implicit same-name base must resolve directly to the built-in — no recursion notice");
    assert_eq!(palette.text, Color32::from_rgb(0xAB, 0xCD, 0xEF), "the override applies over the built-in base");
  }

  #[test]
  fn a_self_referential_explicit_base_is_short_circuited_to_the_built_in() {
    // The intuitive-but-circular form: `themes.default` with an EXPLICIT `base: "default"`. It must NOT recurse into
    // its own entry (which would trip the depth guard and notice "too deep"); the self-reference resolves to the
    // same-name built-in directly, so the override applies over the built-in default cleanly.
    let mut themes = BTreeMap::new();
    themes.insert(
      "default".to_string(),
      ThemeOverride {
        base: Some("default".to_string()),
        accent_motion: ColorSpec::parse("#FF00FF"),
        ..Default::default()
      },
    );
    let appearance = AppearanceConfig { active_theme: "default".to_string(), themes, ..Default::default() };
    let (palette, notice) = appearance.resolve_palette();
    assert_eq!(notice, None, "a self-referential `base: \"default\"` must resolve cleanly, not trip the depth guard");
    assert_eq!(palette.accent_motion, Color32::from_rgb(0xFF, 0x00, 0xFF), "the override applies");
    assert_eq!(palette.panel, Palette::default_dark().panel, "unset fields inherit the built-in default");
  }

  #[test]
  fn from_palette_captures_every_token_and_reproduces_the_palette_over_any_base() {
    // The editor's materialise step: capturing midnight must yield a fully-keyed override that resolves back to
    // exactly midnight even over a completely different base — no token may leak through from the base.
    let captured = ThemeOverride::from_palette(&Palette::midnight());
    assert!(captured.all_color_fields_set(), "a captured palette must name every token");
    assert_eq!(captured.base, None, "a captured theme carries no base dependency");
    assert_eq!(captured.apply_over(Palette::light_slate()), Palette::midnight(),
      "applying the capture over an unrelated base must reproduce the captured palette exactly");
  }

  #[test]
  fn color_entries_mut_iterates_every_token_with_its_config_key_name() {
    // The editor iterates `color_entries_mut`; if it missed a token the picker list would silently go stale. A
    // fully-keyed override must expose all-Some entries, unique names, and clearing one through the entry must
    // clear the real field.
    let mut theme = ThemeOverride::from_palette(&Palette::default_dark());
    let mut names: Vec<&'static str> = Vec::new();
    for (name, slot) in theme.color_entries_mut() {
      assert!(slot.is_some(), "a captured theme exposes a concrete value for {name}");
      names.push(name);
    }
    let count = names.len();
    names.sort_unstable();
    names.dedup();
    assert_eq!(names.len(), count, "token names must be unique");
    assert!(names.contains(&"accent") && names.contains(&"toolpath_cut"), "names are the Palette field idents");
    // Mutating through an entry hits the real field.
    for (name, slot) in theme.color_entries_mut() {
      if name == "accent" {
        *slot = ColorSpec::parse("#123456");
      }
    }
    assert_eq!(theme.accent, ColorSpec::parse("#123456"));
  }

  #[test]
  fn all_color_fields_set_distinguishes_a_full_override_from_a_partial_one() {
    // The template stale-guard must not be vacuously true: a sparse override (one field) is NOT fully keyed, while
    // an override built from a fully-populated palette IS. This pins the guard the bundled-asset test relies on.
    let partial = ThemeOverride { accent: ColorSpec::parse("#FF00FF"), ..Default::default() };
    assert!(!partial.all_color_fields_set(), "a one-field override is not fully keyed");

    let full = full_override_from(Palette::default_dark());
    assert!(full.all_color_fields_set(), "an override naming every field is fully keyed");
  }

  /// Build a `ThemeOverride` whose every colour field is `Some`, taken from `p`'s channels — the in-code mirror of
  /// the bundled asset's fully-keyed template theme, so the guard test does not depend on the JSON asset. Generated
  /// from the single [`palette_color_fields!`] list, so adding a palette token automatically extends this too (and a
  /// missing field would fail to compile the struct literal). `base` is set explicitly (not a colour field).
  fn full_override_from(p: Palette) -> ThemeOverride {
    let c = ColorSpec::from_color32;
    macro_rules! every_field_some {
      ($($field:ident),+ $(,)?) => {
        ThemeOverride {
          base: Some(DEFAULT_THEME.to_string()),
          $($field: Some(c(p.$field)),)+
        }
      };
    }
    palette_color_fields!(every_field_some)
  }
}
