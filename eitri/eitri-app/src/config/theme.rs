//! The config's appearance section: the `active_theme` selector, the font scale, and the user-defined theme
//! overrides, plus the layered resolution that turns all of that into a fully-populated runtime [`Palette`].
//! Mirrored from skirnir's config theme layer, with eitri-app's own token list.
//!
//! **Built-ins live in code, user themes live in the file.** The built-in names (`"default"`, `"light-slate"`,
//! `"midnight"`) resolve to [`Palette`] presets in [`crate::app::theme`]. A user theme is a [`ThemeOverride`]: an
//! optional `base` (another theme name, defaulting to `"default"`) plus a `Some`-per-field set of colour overrides.
//! Resolution starts from the base palette and applies only the override's `Some(...)` fields, so a user theme that
//! recolours one token inherits the other ~29 from its base. An `active_theme` that names nothing known falls back
//! to `"default"` and surfaces a notice — never a panic, never a blank palette.
//!
//! Every field is `#[serde(default)]` (the `Option` fields default to `None` = "inherit"), so a partial theme block
//! merges over the defaults like every other section.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use super::color::ColorSpec;
use crate::app::theme::Palette;

/// The built-in theme name used as the universal fallback and the default `base` for a user override.
pub const DEFAULT_THEME: &str = "default";

/// The accepted range for the global UI font scale ([`AppearanceConfig::font_scale`]). The SINGLE source of truth
/// for both the settings-dialog slider and the apply-time clamp, so egui's always-clamp slider can never silently
/// rewrite an in-range hand-edited value.
pub const FONT_SCALE_RANGE: std::ops::RangeInclusive<f32> = 0.5..=2.5;

/// Clamp a font scale to [`FONT_SCALE_RANGE`]. Used at every site that accepts a scale.
pub fn clamp_font_scale(scale: f32) -> f32 {
  scale.clamp(*FONT_SCALE_RANGE.start(), *FONT_SCALE_RANGE.end())
}

/// The appearance section of the config: which theme is active, a global font scale, and the user-defined themes.
/// Built-in themes are NOT listed here (they live in code); `themes` carries only operator-authored overrides.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AppearanceConfig {
  /// The name of the active theme: a built-in (`"default"`/`"light-slate"`/`"midnight"`) or a key in
  /// [`Self::themes`]. An unknown name resolves to [`DEFAULT_THEME`] with a notice.
  pub active_theme: String,
  /// A global UI font scale (1.0 = the design's sizes). Held to a sane range at apply time.
  pub font_scale: f32,
  /// Operator-authored theme overrides, keyed by name. A `BTreeMap` so the on-disk order is stable.
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
  /// miss falls back to the default palette. A user `themes` entry takes precedence over a same-named built-in.
  pub fn resolve_palette(&self) -> (Palette, Option<String>) {
    self.resolve_named(&self.active_theme, 0)
  }

  /// Resolve a theme *by name* with a depth guard, returning the palette and an optional notice.
  ///
  /// **Precedence: a user `themes` entry WINS over a built-in of the same name** — that is what lets the intuitive
  /// `themes.default` recolour the default theme. Base resolution for a user entry: an EXPLICIT `over.base`
  /// resolves recursively (bounded by [`MAX_BASE_DEPTH`]), except a self-referential base which short-circuits to
  /// the same-name built-in; an IMPLICIT base resolves to the same-name built-in if one exists, else the
  /// [`DEFAULT_THEME`] built-in — taken DIRECTLY from [`builtin_palette`], never back through `self.themes`.
  fn resolve_named(&self, name: &str, depth: usize) -> (Palette, Option<String>) {
    if depth > MAX_BASE_DEPTH {
      return (Palette::default_dark(), Some(format!("theme base chain for {name:?} is too deep — using the default")));
    }
    if let Some(over) = self.themes.get(name) {
      let (base, base_notice) = match over.base.as_deref() {
        // A self-referential base (`themes.default` with `base: "default"`) resolves as the SAME-NAME built-in
        // directly, so it does not recurse back into its own entry and trip the depth guard.
        Some(base_name) if base_name == name => {
          (builtin_palette(name).unwrap_or_else(Palette::default_dark), None)
        }
        Some(base_name) => self.resolve_named(base_name, depth + 1),
        None => (builtin_palette(name).unwrap_or_else(Palette::default_dark), None),
      };
      return (over.apply_over(base), base_notice);
    }
    if let Some(builtin) = builtin_palette(name) {
      return (builtin, None);
    }
    (
      Palette::default_dark(),
      Some(format!("theme {name:?} is not a built-in or a defined theme — using the default")),
    )
  }
}

/// The maximum `base` chain depth resolved before bailing to the default, so a `base` cycle cannot loop.
const MAX_BASE_DEPTH: usize = 8;

/// Map a built-in theme name to its [`Palette`] preset, or `None` when the name is not a built-in.
pub fn builtin_palette(name: &str) -> Option<Palette> {
  match name {
    "default" => Some(Palette::default_dark()),
    "light-slate" => Some(Palette::light_slate()),
    "midnight" => Some(Palette::midnight()),
    _ => None,
  }
}

/// The single source of truth for the set of overridable palette colour fields. Invokes `$callback!` with the full
/// comma-separated list of field idents, so every place that must stay in lockstep — [`ThemeOverride::apply_over`],
/// [`ThemeOverride::from_palette`], [`ThemeOverride::color_entries_mut`], [`ThemeOverride::all_color_fields_set`] —
/// is generated from THIS one list. Adding a new palette token means adding it here (and to the [`ThemeOverride`]
/// struct + [`Palette`]); the call sites then update automatically and the asset stale-guard test fails until the
/// bundled template is extended. `base` is NOT in this list — it is the inheritance pointer, not a colour token.
macro_rules! palette_color_fields {
  ($callback:ident) => {
    $callback! {
      bg, panel, panel_alt, inset, widget, widget_hover, widget_active, divider, border_recess, border_raised,
      accent, accent_hover, accent_active,
      text, text_dim, text_disabled,
      state_ok, state_warn, state_error, state_busy,
      copper, copper_edge, drill, geometry, toolpath_cut, toolpath_rapid, selection, origin, grid_major, grid_minor
    }
  };
}

/// A user-defined theme: an optional `base` to inherit from (default `"default"`) plus a per-field set of colour
/// overrides. Each field is `Option<ColorSpec>` — `None` (the serde default) inherits from the base, `Some(spec)`
/// overrides it. Field names match [`Palette`]'s so the JSON reads obviously (`"accent": "#FF00FF"`).
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct ThemeOverride {
  /// The theme this one inherits unset fields from. `None` (the default) means the same-name built-in when one
  /// exists, else `"default"`.
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

  // Text.
  /// Override for `text.primary`.
  pub text: Option<ColorSpec>,
  /// Override for `text.secondary`.
  pub text_dim: Option<ColorSpec>,
  /// Override for `text.disabled`.
  pub text_disabled: Option<ColorSpec>,

  // Status.
  /// Override for the success/finished green.
  pub state_ok: Option<ColorSpec>,
  /// Override for the caution/cancelled amber.
  pub state_warn: Option<ColorSpec>,
  /// Override for the failure red.
  pub state_error: Option<ColorSpec>,
  /// Override for the in-flight teal.
  pub state_busy: Option<ColorSpec>,

  // Canvas.
  /// Override for the copper region fill.
  pub copper: Option<ColorSpec>,
  /// Override for the copper edge stroke.
  pub copper_edge: Option<ColorSpec>,
  /// Override for the drill-hit colour.
  pub drill: Option<ColorSpec>,
  /// Override for imported/generated geometry outlines.
  pub geometry: Option<ColorSpec>,
  /// Override for the cut toolpath trail.
  pub toolpath_cut: Option<ColorSpec>,
  /// Override for the rapid toolpath trail.
  pub toolpath_rapid: Option<ColorSpec>,
  /// Override for the canvas selection rim.
  pub selection: Option<ColorSpec>,
  /// Override for the origin crosshair.
  pub origin: Option<ColorSpec>,
  /// Override for the grid major-line colour.
  pub grid_major: Option<ColorSpec>,
  /// Override for the grid minor-line colour.
  pub grid_minor: Option<ColorSpec>,
}

impl ThemeOverride {
  /// Apply this override's set (`Some`) colour fields over a base palette, returning the layered result. Every
  /// unset (`None`) field inherits the base value. Generated from the single [`palette_color_fields!`] list.
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
  /// This is how the in-app theme editor materialises a new user theme — it snapshots the active palette into
  /// concrete per-token values so every colour picker has a real value to edit.
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
  /// iteration surface, generated from the same single list as the resolution logic so it can never miss a token.
  pub fn color_entries_mut(&mut self) -> Vec<(&'static str, &mut Option<ColorSpec>)> {
    macro_rules! entries {
      ($($field:ident),+ $(,)?) => {
        vec![$((stringify!($field), &mut self.$field),)+]
      };
    }
    palette_color_fields!(entries)
  }

  /// Whether EVERY overridable colour field is `Some` — the stale-guard for the bundled config asset's
  /// self-documenting template theme. Generated from the single [`palette_color_fields!`] list.
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
  fn font_scale_range_and_clamp_share_one_source_and_admit_the_full_span() {
    assert!(FONT_SCALE_RANGE.contains(&2.4), "2.4 is a valid, accepted scale");
    assert_eq!(clamp_font_scale(2.4), 2.4, "an in-range scale passes through unchanged");
    assert_eq!(clamp_font_scale(9.0), *FONT_SCALE_RANGE.end(), "an above-range scale clamps to the max");
    assert_eq!(clamp_font_scale(0.1), *FONT_SCALE_RANGE.start(), "a below-range scale clamps to the min");
  }

  #[test]
  fn the_default_appearance_resolves_to_the_default_dark_palette_with_no_notice() {
    let (palette, notice) = AppearanceConfig::default().resolve_palette();
    assert_eq!(palette, Palette::default_dark());
    assert_eq!(notice, None);
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
    assert_eq!(palette, Palette::default_dark());
    assert!(notice.is_some_and(|n| n.contains("neon-dreams")), "the notice should name the missing theme");
  }

  #[test]
  fn a_user_override_layers_only_its_set_fields_over_its_base() {
    let mut themes = BTreeMap::new();
    themes.insert(
      "my-theme".to_string(),
      ThemeOverride { copper: ColorSpec::parse("#FF00FF"), ..Default::default() },
    );
    let appearance = AppearanceConfig { active_theme: "my-theme".to_string(), themes, ..Default::default() };
    let (palette, notice) = appearance.resolve_palette();
    assert_eq!(notice, None);
    assert_eq!(palette.copper, Color32::from_rgb(0xFF, 0x00, 0xFF), "the one overridden field is applied");
    assert_eq!(palette.panel, Palette::default_dark().panel, "every unset field falls through to the base");
  }

  #[test]
  fn a_user_override_can_base_on_another_built_in() {
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
    assert_eq!(palette.panel, Palette::midnight().panel, "unset fields come from the named base");
  }

  #[test]
  fn a_base_cycle_is_bounded_and_falls_back_rather_than_looping() {
    let mut themes = BTreeMap::new();
    themes.insert("a".to_string(), ThemeOverride { base: Some("b".to_string()), ..Default::default() });
    themes.insert("b".to_string(), ThemeOverride { base: Some("a".to_string()), ..Default::default() });
    let appearance = AppearanceConfig { active_theme: "a".to_string(), themes, ..Default::default() };
    let (palette, notice) = appearance.resolve_palette();
    assert_eq!(palette.accent, Palette::default_dark().accent);
    assert!(notice.is_some(), "a cycle must surface a notice");
  }

  #[test]
  fn a_user_themes_entry_named_default_overrides_the_built_in_and_inherits_the_rest() {
    let mut themes = BTreeMap::new();
    themes.insert("default".to_string(), ThemeOverride { toolpath_cut: ColorSpec::parse("#FF00FF"), ..Default::default() });
    let appearance = AppearanceConfig { active_theme: "default".to_string(), themes, ..Default::default() };
    let (palette, notice) = appearance.resolve_palette();
    assert_eq!(notice, None, "shadowing a built-in with a same-named user entry surfaces no notice");
    assert_eq!(palette.toolpath_cut, Color32::from_rgb(0xFF, 0x00, 0xFF), "the user override of `default` applies");
    assert_eq!(palette.panel, Palette::default_dark().panel, "every unset field inherits the built-in default");
  }

  #[test]
  fn a_self_referential_explicit_base_is_short_circuited_to_the_built_in() {
    let mut themes = BTreeMap::new();
    themes.insert(
      "default".to_string(),
      ThemeOverride {
        base: Some("default".to_string()),
        copper: ColorSpec::parse("#FF00FF"),
        ..Default::default()
      },
    );
    let appearance = AppearanceConfig { active_theme: "default".to_string(), themes, ..Default::default() };
    let (palette, notice) = appearance.resolve_palette();
    assert_eq!(notice, None, "a self-referential `base: \"default\"` must resolve cleanly, not trip the depth guard");
    assert_eq!(palette.copper, Color32::from_rgb(0xFF, 0x00, 0xFF));
    assert_eq!(palette.panel, Palette::default_dark().panel);
  }

  #[test]
  fn from_palette_captures_every_token_and_reproduces_the_palette_over_any_base() {
    let captured = ThemeOverride::from_palette(&Palette::midnight());
    assert!(captured.all_color_fields_set(), "a captured palette must name every token");
    assert_eq!(captured.base, None, "a captured theme carries no base dependency");
    assert_eq!(captured.apply_over(Palette::light_slate()), Palette::midnight(),
      "applying the capture over an unrelated base must reproduce the captured palette exactly");
  }

  #[test]
  fn color_entries_mut_iterates_every_token_with_unique_names() {
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
    assert!(names.contains(&"copper") && names.contains(&"toolpath_cut"), "names are the Palette field idents");
  }

  #[test]
  fn all_color_fields_set_distinguishes_a_full_override_from_a_partial_one() {
    let partial = ThemeOverride { accent: ColorSpec::parse("#FF00FF"), ..Default::default() };
    assert!(!partial.all_color_fields_set(), "a one-field override is not fully keyed");
    let full = ThemeOverride::from_palette(&Palette::default_dark());
    assert!(full.all_color_fields_set(), "an override naming every field is fully keyed");
  }
}
