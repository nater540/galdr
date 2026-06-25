//! The non-colour config sections: UI defaults ([`UiConfig`]), connection/streaming tuning ([`ConnectionConfig`]),
//! and toolpath-render tuning ([`ToolpathConfig`]), plus the resolved runtime [`ToolpathStyle`] the views read.
//!
//! Every field carries `#[serde(default)]` with a matching [`Default`], so a config file need only mention the keys
//! it changes — every absent field, and every absent whole section, falls back to the baked default rather than
//! failing the parse. The serialisable `*Config` structs are the *on-disk* shape; [`ToolpathStyle`] is the
//! *resolved* form threaded into `UiState` for the toolpath viewport, mirroring how [`super::Palette`] is the
//! resolved form of the colour theme. The numeric defaults here are the exact values the views/preview previously
//! hard-coded, so a fresh install renders pixel-identically to before the config existed.

use serde::{Deserialize, Serialize};

/// UI defaults applied when [`crate::app::views::UiState`] is built at startup: the jog/probe/DRO/console knobs an
/// operator would otherwise re-set each launch. The *profile* (`profile.ron`) still wins for genuinely last-used
/// values (the last port, a found rotary center); these are the from-scratch defaults the config seeds.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct UiConfig {
  /// The jog step distance (mm) the jog pad starts on.
  pub jog_step_mm: f64,
  /// The jog feed rate (mm/min) the jog pad starts on.
  pub jog_feed: f64,
  /// Whether the jog pad starts in continuous (press-and-hold) mode rather than fixed-step.
  pub jog_continuous: bool,
  /// Whether the DRO starts emphasising machine position (`true`) rather than work position (`false`, the default).
  pub dro_show_machine: bool,
  /// Whether the console starts showing every received line (`true`) rather than hiding bare `ok` acks (`false`).
  pub console_verbose: bool,
  /// Whether the console starts auto-scrolling to the newest line.
  pub console_auto_scroll: bool,
  /// The initial window width (logical px) for the eframe viewport.
  pub window_w: f32,
  /// The initial window height (logical px) for the eframe viewport.
  pub window_h: f32,
}

impl Default for UiConfig {
  fn default() -> Self {
    // These mirror `UiState::default` / the design's window size so config-absent behaviour is identical to before.
    UiConfig {
      jog_step_mm: 1.0,
      jog_feed: 500.0,
      jog_continuous: false,
      dro_show_machine: false,
      console_verbose: false,
      console_auto_scroll: true,
      window_w: 1100.0,
      window_h: 720.0,
    }
  }
}

/// The minimum window WIDTH (logical px) the initial viewport is held to — matching the viewport's
/// `with_min_inner_size` width so a hand-edited `window_w` of `0` (or a tiny/negative value) can never produce an
/// unusable initial window. The clamp mirrors how [`ToolpathConfig::resolve`] floors its numeric knobs.
const MIN_WINDOW_W: f32 = 800.0;
/// The minimum window HEIGHT (logical px), matching the viewport's `with_min_inner_size` height (500). Distinct from
/// the width floor so the design's default 720 height is NOT clamped up.
const MIN_WINDOW_H: f32 = 500.0;
/// The maximum window dimension (logical px) either axis is held to — large enough for any real monitor, tight
/// enough that a garbage huge value cannot request an absurd surface.
const MAX_WINDOW_DIM: f32 = 8000.0;

impl UiConfig {
  /// The initial window size held to sane positive dimensions: width clamped to `[MIN_WINDOW_W, MAX_WINDOW_DIM]`,
  /// height to `[MIN_WINDOW_H, MAX_WINDOW_DIM]` (the same per-axis minimums as the viewport's `with_min_inner_size`),
  /// with a non-finite value (NaN/inf from a corrupt file) resolving to the floor. Every other numeric config knob is
  /// floored in [`ToolpathConfig::resolve`]; the window size gets the same treatment here so `run()` never hands an
  /// unusable `0`/negative/NaN dimension to the viewport builder.
  pub fn window_size(&self) -> (f32, f32) {
    (clamp_window_dim(self.window_w, MIN_WINDOW_W), clamp_window_dim(self.window_h, MIN_WINDOW_H))
  }
}

/// Clamp one window dimension to `[min, MAX_WINDOW_DIM]`, mapping a non-finite value to `min`. Pure so the clamp is
/// unit-tested. `f32::clamp` would panic on a NaN input vs bound, so we guard non-finite inputs explicitly first.
fn clamp_window_dim(dim: f32, min: f32) -> f32 {
  if !dim.is_finite() {
    return min;
  }
  dim.clamp(min, MAX_WINDOW_DIM)
}

/// Connection/streaming tuning read where the engine task and the reconnect schedule are constructed. The ESP32-S3
/// native USB ignores the baud, but the host driver and the dropdown want a value; `default_baud` seeds the toolbar
/// only when the *profile* carries no last-used port (the profile's remembered value still wins when present).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ConnectionConfig {
  /// The baud to seed the connect dropdown with on a fresh profile (no remembered port).
  pub default_baud: u32,
  /// How often (ms) the host requests a `?` real-time status report while connected.
  pub status_poll_ms: u64,
  /// An override for the host-side character-counting RX window (bytes). `None` learns it from the firmware's
  /// advertised buffer (the safe default); a value pins it for a controller that does not advertise one.
  pub rx_window: Option<usize>,
  /// The auto-reconnect backoff schedule consulted after an unexpected drop.
  pub reconnect: ReconnectSection,
}

impl Default for ConnectionConfig {
  fn default() -> Self {
    ConnectionConfig {
      default_baud: 115_200,
      status_poll_ms: 200,
      rx_window: None,
      reconnect: ReconnectSection::default(),
    }
  }
}

/// The serialisable mirror of [`crate::reconnect::ReconnectConfig`]'s knobs, so the backoff schedule is config-driven
/// without leaking the engine type into the on-disk schema. [`Self::to_policy_config`] converts it back at the seam.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct ReconnectSection {
  /// The first retry delay (ms) after a drop.
  pub base_ms: u64,
  /// The integer multiplier applied to the delay after each failed attempt (2 = double each time).
  pub factor: u32,
  /// The ceiling (ms) the exponential backoff is clamped to.
  pub max_delay_ms: u64,
  /// How many attempts to make before giving up. `0` disables auto-reconnect entirely.
  pub max_attempts: u32,
}

impl Default for ReconnectSection {
  fn default() -> Self {
    // Mirror `ReconnectConfig::default` so an absent reconnect block behaves exactly as the hard-coded schedule.
    let defaults = crate::reconnect::ReconnectConfig::default();
    ReconnectSection {
      base_ms: defaults.base.as_millis() as u64,
      factor: defaults.factor,
      max_delay_ms: defaults.max_delay.as_millis() as u64,
      max_attempts: defaults.max_attempts,
    }
  }
}

impl ReconnectSection {
  /// Convert the config section back into the engine's [`crate::reconnect::ReconnectConfig`]. Durations are rebuilt
  /// from the millisecond fields; the factor/attempts pass through. The single conversion seam so the shell never
  /// re-derives it.
  pub fn to_policy_config(&self) -> crate::reconnect::ReconnectConfig {
    crate::reconnect::ReconnectConfig {
      base: std::time::Duration::from_millis(self.base_ms),
      factor: self.factor,
      max_delay: std::time::Duration::from_millis(self.max_delay_ms),
      max_attempts: self.max_attempts,
    }
  }
}

/// Toolpath-render tuning: the stroke widths, arc-flattening step, grid spacing, and marker radius the viewport
/// draws with. The on-disk shape; resolved into [`ToolpathStyle`] for the views. Defaults are the exact constants
/// `views.rs`/`preview.rs` previously hard-coded, so a config-absent render is pixel-identical.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ToolpathConfig {
  /// Stroke width (px) of a cut (programmed-feed) trail segment.
  pub cut_stroke_px: f32,
  /// Stroke width (px) of a rapid (travel) trail segment.
  pub rapid_stroke_px: f32,
  /// The maximum gap between two consecutive trail points still drawn as a connected line, as a fraction of the
  /// model-space span diagonal. A larger gap is a jump/reconnect and is left broken rather than streaked.
  pub trail_break_fraction: f32,
  /// The maximum angular step (degrees) of a flattened arc chord. A smaller value renders arcs smoother at more
  /// segments; the default (~9°, 20 chords per circle) is smooth at preview scale while keeping the count modest.
  pub arc_step_deg: f32,
  /// The viewport grid's minor-line spacing (screen px).
  pub grid_minor_px: f32,
  /// How many minor cells make a major (every Nth) grid line.
  pub grid_major_every: u32,
  /// The filled live tool-dot radius (px).
  pub marker_radius_px: f32,
}

impl Default for ToolpathConfig {
  fn default() -> Self {
    ToolpathConfig {
      cut_stroke_px: 1.6,
      rapid_stroke_px: 1.0,
      trail_break_fraction: 0.12,
      // 9° matches the old `MAX_ARC_STEP_RAD = PI/20` (PI/20 rad = 9°).
      arc_step_deg: 9.0,
      grid_minor_px: 16.0,
      grid_major_every: 5,
      marker_radius_px: 4.0,
    }
  }
}

impl ToolpathConfig {
  /// Resolve the on-disk config into the runtime [`ToolpathStyle`] the views render with, converting the arc step
  /// from degrees to the radians the flattener wants and holding the numeric knobs to sane floors so a hand-edited
  /// zero/negative cannot divide-by-zero or vanish the geometry.
  pub fn resolve(&self) -> ToolpathStyle {
    ToolpathStyle {
      cut_stroke_px: self.cut_stroke_px.max(0.1),
      rapid_stroke_px: self.rapid_stroke_px.max(0.1),
      trail_break_fraction: self.trail_break_fraction.max(0.0),
      // Clamp to a small positive angle so the chord count stays finite; convert deg→rad for the flattener.
      arc_step_rad: self.arc_step_deg.clamp(0.5, 90.0).to_radians(),
      grid_minor_px: self.grid_minor_px.max(2.0),
      grid_major_every: self.grid_major_every.max(1),
      marker_radius_px: self.marker_radius_px.max(0.5),
    }
  }
}

/// The resolved toolpath-render style threaded into `UiState` and read by the viewport — the runtime counterpart of
/// [`ToolpathConfig`], with the arc step pre-converted to radians and every knob held to a sane floor. Copy-cheap so
/// a config reload swaps it in without churn.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ToolpathStyle {
  /// Stroke width (px) of a cut trail segment.
  pub cut_stroke_px: f32,
  /// Stroke width (px) of a rapid trail segment.
  pub rapid_stroke_px: f32,
  /// Trail-break gap as a fraction of the model-space span diagonal.
  pub trail_break_fraction: f32,
  /// The maximum angular step (radians) of a flattened arc chord — the form the flattener consumes.
  pub arc_step_rad: f32,
  /// The viewport grid's minor-line spacing (screen px).
  pub grid_minor_px: f32,
  /// How many minor cells make a major grid line.
  pub grid_major_every: u32,
  /// The filled live tool-dot radius (px).
  pub marker_radius_px: f32,
}

impl Default for ToolpathStyle {
  fn default() -> Self {
    ToolpathConfig::default().resolve()
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn ui_config_defaults_match_the_legacy_ui_state_values() {
    let ui = UiConfig::default();
    assert_eq!(ui.jog_step_mm, 1.0);
    assert_eq!(ui.jog_feed, 500.0);
    assert_eq!(ui.window_w, 1100.0);
    assert_eq!(ui.window_h, 720.0);
    assert!(ui.console_auto_scroll, "auto-scroll defaults on, as the old UiState did");
  }

  #[test]
  fn window_size_passes_a_sane_default_through_and_clamps_degenerate_values() {
    // The default size is well within range and passes through unchanged (the 720 height is NOT clamped up — the
    // height floor matches the viewport's 500, distinct from the 800 width floor).
    assert_eq!(UiConfig::default().window_size(), (1100.0, 720.0), "a sane default passes through unchanged");
    // A hand-edited zero/negative is floored to the per-axis minimum so the initial window is never unusable.
    let tiny = UiConfig { window_w: 0.0, window_h: -10.0, ..UiConfig::default() };
    let (w, h) = tiny.window_size();
    assert!(w >= MIN_WINDOW_W && h >= MIN_WINDOW_H, "a 0/negative dimension floors to the per-axis minimum: ({w}, {h})");
    // An absurd huge value is capped so it cannot request an impossible surface.
    let huge = UiConfig { window_w: 1.0e9, window_h: 1.0e9, ..UiConfig::default() };
    let (w, h) = huge.window_size();
    assert!(w <= MAX_WINDOW_DIM && h <= MAX_WINDOW_DIM, "a huge dimension caps at the maximum: ({w}, {h})");
    // A non-finite value (NaN/inf from a corrupt file) must not panic and resolves to the per-axis floor.
    let nan = UiConfig { window_w: f32::NAN, window_h: f32::INFINITY, ..UiConfig::default() };
    assert_eq!(nan.window_size(), (MIN_WINDOW_W, MIN_WINDOW_H), "non-finite dims resolve to the floor, no panic");
  }

  #[test]
  fn connection_defaults_mirror_the_engine_reconnect_config() {
    let conn = ConnectionConfig::default();
    assert_eq!(conn.default_baud, 115_200);
    let policy = conn.reconnect.to_policy_config();
    assert_eq!(policy, crate::reconnect::ReconnectConfig::default(), "the section must round-trip to the engine config");
  }

  #[test]
  fn toolpath_defaults_preserve_the_legacy_render_constants() {
    let style = ToolpathConfig::default().resolve();
    // 9° must resolve to the old PI/20 rad arc step.
    assert!((style.arc_step_rad - std::f32::consts::PI / 20.0).abs() < 1e-5, "9° must equal the old PI/20 step");
    assert!((style.trail_break_fraction - 0.12).abs() < 1e-6);
    assert!((style.cut_stroke_px - 1.6).abs() < 1e-6);
    assert!((style.rapid_stroke_px - 1.0).abs() < 1e-6);
    assert_eq!(style.grid_major_every, 5);
    assert!((style.grid_minor_px - 16.0).abs() < 1e-6);
  }

  #[test]
  fn resolve_clamps_degenerate_knobs_to_safe_floors() {
    // A hand-edited zero/negative must not divide-by-zero in the arc flattener or vanish the strokes.
    let cfg = ToolpathConfig {
      cut_stroke_px: 0.0,
      rapid_stroke_px: -1.0,
      arc_step_deg: 0.0,
      grid_minor_px: 0.0,
      grid_major_every: 0,
      marker_radius_px: 0.0,
      trail_break_fraction: -5.0,
    };
    let style = cfg.resolve();
    assert!(style.cut_stroke_px >= 0.1 && style.rapid_stroke_px >= 0.1, "strokes are floored positive");
    assert!(style.arc_step_rad > 0.0, "the arc step stays positive so the chord count is finite");
    assert!(style.grid_minor_px >= 2.0 && style.grid_major_every >= 1);
    assert!(style.marker_radius_px >= 0.5);
    assert!(style.trail_break_fraction >= 0.0, "a negative break fraction floors to zero");
  }

  #[test]
  fn a_partial_section_merges_over_defaults() {
    // `#[serde(default)]` on the struct means a file mentioning only one key fills the rest from Default.
    let ui: UiConfig = serde_json::from_str(r#"{ "jog_step_mm": 0.05 }"#).expect("a partial UiConfig parses");
    assert_eq!(ui.jog_step_mm, 0.05, "the mentioned key is taken");
    assert_eq!(ui.jog_feed, UiConfig::default().jog_feed, "the absent key defaults");
    assert_eq!(ui.window_w, UiConfig::default().window_w, "the absent window size defaults");
  }
}
