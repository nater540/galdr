//! The non-colour config sections: UI defaults ([`UiConfig`]) and canvas-render tuning ([`CanvasConfig`]), plus
//! the resolved runtime [`CanvasStyle`] the canvas reads. Mirrored from skirnir's config sections.
//!
//! Every field carries `#[serde(default)]` with a matching [`Default`], so a config file need only mention the
//! keys it changes — every absent field, and every absent whole section, falls back to the baked default rather
//! than failing the parse. The serialisable `*Config` structs are the *on-disk* shape; [`CanvasStyle`] is the
//! *resolved* form the canvas renders with, mirroring how [`super::theme`] resolves colours into a `Palette`.

use serde::{Deserialize, Serialize};

/// UI defaults applied when the transient view state is built at startup: language, window size, and the dock
/// share the central split opens with.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct UiConfig {
  /// The UI language (a BCP-47 locale tag, e.g. `en-US`) selected on the global i18n registry at startup. Any
  /// key a non-default locale does not translate falls back to the bundled `en-US` resource.
  pub language: String,
  /// The initial window width (logical px) for the eframe viewport.
  pub window_w: f32,
  /// The initial window height (logical px) for the eframe viewport.
  pub window_h: f32,
  /// The fraction of the central region the bottom dock opens with (clamped at use).
  pub dock_fraction: f32,
}

impl Default for UiConfig {
  fn default() -> Self {
    UiConfig {
      language: crate::i18n::EN_US.to_string(),
      window_w: 1280.0,
      window_h: 800.0,
      dock_fraction: 0.25,
    }
  }
}

/// The minimum window WIDTH (logical px) the initial viewport is held to — matching the viewport's
/// `with_min_inner_size` width so a hand-edited `window_w` of `0` can never produce an unusable initial window.
const MIN_WINDOW_W: f32 = 900.0;
/// The minimum window HEIGHT (logical px), matching the viewport's `with_min_inner_size` height.
const MIN_WINDOW_H: f32 = 560.0;
/// The maximum window dimension (logical px) either axis is held to, so a garbage huge value cannot request an
/// absurd surface.
const MAX_WINDOW_DIM: f32 = 8000.0;

impl UiConfig {
  /// The initial window size held to sane positive dimensions, with a non-finite value (NaN/inf from a corrupt
  /// file) resolving to the floor — `run()` never hands an unusable dimension to the viewport builder.
  pub fn window_size(&self) -> (f32, f32) {
    (clamp_window_dim(self.window_w, MIN_WINDOW_W), clamp_window_dim(self.window_h, MIN_WINDOW_H))
  }
}

/// Clamp one window dimension to `[min, MAX_WINDOW_DIM]`, mapping a non-finite value to `min`. `f32::clamp`
/// would panic comparing against NaN, so non-finite inputs are guarded explicitly first.
fn clamp_window_dim(dim: f32, min: f32) -> f32 {
  if !dim.is_finite() {
    return min;
  }
  dim.clamp(min, MAX_WINDOW_DIM)
}

/// Canvas-render tuning: stroke widths, fill opacity, and grid spacing. The on-disk shape; resolved into
/// [`CanvasStyle`] for the views, with every knob held to a sane floor.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct CanvasConfig {
  /// Stroke width (px) of a cut (feed-move) toolpath segment.
  pub cut_stroke_px: f32,
  /// Stroke width (px) of a rapid (travel-move) toolpath segment.
  pub rapid_stroke_px: f32,
  /// Stroke width (px) of geometry/copper outline rings.
  pub outline_stroke_px: f32,
  /// Copper fill opacity, `0..=1` — the fill is dimmed slightly so toolpaths stay readable over it.
  pub copper_opacity: f32,
  /// The canvas grid's minor-line spacing (screen px).
  pub grid_minor_px: f32,
  /// How many minor cells make a major (every Nth) grid line.
  pub grid_major_every: u32,
}

impl Default for CanvasConfig {
  fn default() -> Self {
    CanvasConfig {
      cut_stroke_px: 1.6,
      rapid_stroke_px: 1.0,
      outline_stroke_px: 1.0,
      copper_opacity: 0.85,
      grid_minor_px: 16.0,
      grid_major_every: 5,
    }
  }
}

impl CanvasConfig {
  /// Resolve the on-disk config into the runtime [`CanvasStyle`], holding the numeric knobs to sane floors so a
  /// hand-edited zero/negative cannot vanish the geometry or divide the grid by zero.
  pub fn resolve(&self) -> CanvasStyle {
    CanvasStyle {
      cut_stroke_px: self.cut_stroke_px.max(0.1),
      rapid_stroke_px: self.rapid_stroke_px.max(0.1),
      outline_stroke_px: self.outline_stroke_px.max(0.1),
      copper_opacity: if self.copper_opacity.is_finite() { self.copper_opacity.clamp(0.05, 1.0) } else { 0.85 },
      grid_minor_px: self.grid_minor_px.max(2.0),
      grid_major_every: self.grid_major_every.max(1),
    }
  }
}

/// The resolved canvas-render style the canvas reads — the runtime counterpart of [`CanvasConfig`], with every
/// knob held to a sane floor. Copy-cheap so a config reload swaps it in without churn.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CanvasStyle {
  /// Stroke width (px) of a cut toolpath segment.
  pub cut_stroke_px: f32,
  /// Stroke width (px) of a rapid toolpath segment.
  pub rapid_stroke_px: f32,
  /// Stroke width (px) of outline rings.
  pub outline_stroke_px: f32,
  /// Copper fill opacity, clamped to `0.05..=1`.
  pub copper_opacity: f32,
  /// The canvas grid's minor-line spacing (screen px).
  pub grid_minor_px: f32,
  /// How many minor cells make a major grid line.
  pub grid_major_every: u32,
}

impl Default for CanvasStyle {
  fn default() -> Self {
    CanvasConfig::default().resolve()
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn ui_defaults_speak_english_at_a_laptop_size() {
    let ui = UiConfig::default();
    assert_eq!(ui.language, crate::i18n::EN_US);
    assert_eq!((ui.window_w, ui.window_h), (1280.0, 800.0));
    assert!(ui.dock_fraction > 0.0 && ui.dock_fraction < 1.0);
  }

  #[test]
  fn window_size_passes_a_sane_default_through_and_clamps_degenerate_values() {
    assert_eq!(UiConfig::default().window_size(), (1280.0, 800.0), "a sane default passes through unchanged");
    let tiny = UiConfig { window_w: 0.0, window_h: -10.0, ..UiConfig::default() };
    let (w, h) = tiny.window_size();
    assert!(w >= MIN_WINDOW_W && h >= MIN_WINDOW_H, "a 0/negative dimension floors to the per-axis minimum");
    let huge = UiConfig { window_w: 1.0e9, window_h: 1.0e9, ..UiConfig::default() };
    let (w, h) = huge.window_size();
    assert!(w <= MAX_WINDOW_DIM && h <= MAX_WINDOW_DIM, "a huge dimension caps at the maximum");
    let nan = UiConfig { window_w: f32::NAN, window_h: f32::INFINITY, ..UiConfig::default() };
    assert_eq!(nan.window_size(), (MIN_WINDOW_W, MIN_WINDOW_H), "non-finite dims resolve to the floor, no panic");
  }

  #[test]
  fn canvas_resolve_clamps_degenerate_knobs_to_safe_floors() {
    let cfg = CanvasConfig {
      cut_stroke_px: 0.0,
      rapid_stroke_px: -1.0,
      outline_stroke_px: 0.0,
      copper_opacity: f32::NAN,
      grid_minor_px: 0.0,
      grid_major_every: 0,
    };
    let style = cfg.resolve();
    assert!(style.cut_stroke_px >= 0.1 && style.rapid_stroke_px >= 0.1 && style.outline_stroke_px >= 0.1);
    assert!(style.copper_opacity > 0.0 && style.copper_opacity <= 1.0, "a NaN opacity resolves to the default");
    assert!(style.grid_minor_px >= 2.0 && style.grid_major_every >= 1);
  }

  #[test]
  fn a_partial_section_merges_over_defaults() {
    let ui: UiConfig = serde_json::from_str(r#"{ "language": "sv-SE" }"#).expect("a partial UiConfig parses");
    assert_eq!(ui.language, "sv-SE", "the mentioned key is taken");
    assert_eq!(ui.window_w, UiConfig::default().window_w, "the absent key defaults");
    let canvas: CanvasConfig = serde_json::from_str(r#"{ "cut_stroke_px": 3.0 }"#).expect("partial parses");
    assert!((canvas.cut_stroke_px - 3.0).abs() < 1e-6);
    assert_eq!(canvas.grid_major_every, CanvasConfig::default().grid_major_every);
  }
}
