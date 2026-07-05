//! The runtime [`Palette`]: eitri-app's colour scheme as an *instance* (one field per design token), so the
//! active colours come from the loaded config instead of being baked in.
//!
//! The design mirrors skirnir's: dark, near-black, flat — no shadows except popovers — so the CAM app and the
//! sender read as one family. The chrome/accent/text tokens carry skirnir's exact hex values; the CAM-specific
//! tokens (copper, drills, toolpath trails, canvas grid) replace skirnir's machine-state block. Three built-ins
//! ship (`default`, `light-slate`, `midnight`), selectable by name from the config; alternates change only the
//! chrome so the canvas semantics (copper is copper-coloured, cuts are yellow) never shift between themes.

use eframe::egui::Color32;

/// Build a `Color32` from a `0xRRGGBB` literal at compile time, so the default tokens read as hex values.
/// Opaque — the palette carries no per-token alpha; translucency is applied at paint time where needed.
const fn rgb(hex: u32) -> Color32 {
  Color32::from_rgb((hex >> 16) as u8, (hex >> 8) as u8, hex as u8)
}

/// The eitri-app colour palette as runtime state: a dark, high-contrast, flat scheme matching skirnir's chrome.
/// One field per design token. Cloned cheaply into the view style each time the config resolves a theme.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Palette {
  // ── Chrome & surface (identical to skirnir's tokens) ─────────────────────────────────────────────────
  /// `bg.window` — the app window / outermost fill.
  pub bg: Color32,
  /// `bg.panel` — panel / dock body.
  pub panel: Color32,
  /// `bg.panelAlt` — toolbar / tab strip / section header.
  pub panel_alt: Color32,
  /// `bg.inset` — the canvas, text fields, and the G-code/log views (recessed surfaces).
  pub inset: Color32,
  /// `bg.widget` — control rest.
  pub widget: Color32,
  /// `bg.widget.hover`.
  pub widget_hover: Color32,
  /// `bg.widget.active` — pressed / selected.
  pub widget_active: Color32,
  /// `divider` — 1px panel separators.
  pub divider: Color32,
  /// `border.recess` — inset top edge.
  pub border_recess: Color32,
  /// `border.raised` — raised control edge.
  pub border_raised: Color32,

  // ── Accents ──────────────────────────────────────────────────────────────────────────────────────────
  /// `accent.primary` — selection / focus / primary actions (cool blue).
  pub accent: Color32,
  /// `accent.primary` hover.
  pub accent_hover: Color32,
  /// `accent.primary` active/pressed.
  pub accent_active: Color32,

  // ── Text on dark ─────────────────────────────────────────────────────────────────────────────────────
  /// `text.primary` — primary readable text.
  pub text: Color32,
  /// `text.secondary` — labels, hints, units.
  pub text_dim: Color32,
  /// `text.disabled` — disabled control text.
  pub text_disabled: Color32,

  // ── Status (log lines, progress, op outcomes — paired with text, never colour alone) ─────────────────
  /// Success / finished (green).
  pub state_ok: Color32,
  /// Caution / cancelled (amber).
  pub state_warn: Color32,
  /// Failure (red).
  pub state_error: Color32,
  /// In-flight / busy (teal).
  pub state_busy: Color32,

  // ── Canvas ───────────────────────────────────────────────────────────────────────────────────────────
  /// Copper region fill (the Gerber layer).
  pub copper: Color32,
  /// Copper region edge stroke, a lighter rim so pads keep a crisp silhouette at any zoom.
  pub copper_edge: Color32,
  /// Drill hits (Excellon), drawn punched-out over copper.
  pub drill: Color32,
  /// Imported/generated geometry outlines (SVG, DXF, panelize, mirror).
  pub geometry: Color32,
  /// The cut (feed-move) toolpath trail — the same yellow as skirnir's live cut trail.
  pub toolpath_cut: Color32,
  /// The rapid (travel-move) toolpath trail — the cool control blue.
  pub toolpath_rapid: Color32,
  /// Selected-object highlight rim on the canvas.
  pub selection: Color32,
  /// The canvas origin crosshair.
  pub origin: Color32,
  /// The canvas grid's major (every Nth) line colour.
  pub grid_major: Color32,
  /// The canvas grid's minor line colour.
  pub grid_minor: Color32,
}

impl Palette {
  /// The built-in `"default"` theme: skirnir's exact dark chrome plus the CAM canvas tokens. This is the
  /// canonical baseline every config theme resolves over, and the fallback when an `active_theme` is unknown.
  pub const fn default_dark() -> Self {
    Palette {
      bg: rgb(0x121212),
      panel: rgb(0x1B1B1B),
      panel_alt: rgb(0x222222),
      inset: rgb(0x0E0E0E),
      widget: rgb(0x2A2A2A),
      widget_hover: rgb(0x333333),
      widget_active: rgb(0x3C3C3C),
      divider: rgb(0x2E2E2E),
      border_recess: rgb(0x080808),
      border_raised: rgb(0x3A3A3A),
      accent: rgb(0x0E86D4),
      accent_hover: rgb(0x2BA8F0),
      accent_active: rgb(0x0B6FB0),
      text: rgb(0xE4E4E4),
      text_dim: rgb(0x9A9A9A),
      text_disabled: rgb(0x5C5C5C),
      state_ok: rgb(0x3FB861),
      state_warn: rgb(0xE0A33E),
      state_error: rgb(0xE5484D),
      state_busy: rgb(0x2BB6C9),
      // The canvas: copper reads as copper (a warm oxide orange over the near-black inset), drills punch out
      // in a light steel, cuts take skirnir's trail yellow, rapids the control blue. The grid is the panel
      // fill at two intensities, exactly like skirnir's toolpath viewport.
      copper: rgb(0xB2602E),
      copper_edge: rgb(0xD97B42),
      drill: rgb(0x9BB4C8),
      geometry: rgb(0x3FB861),
      toolpath_cut: rgb(0xFFE000),
      toolpath_rapid: rgb(0x0E86D4),
      selection: rgb(0x2BA8F0),
      origin: rgb(0x9B7FE0),
      grid_major: rgb(0x1B1B1B),
      grid_minor: rgb(0x0D0D0D),
    }
  }

  /// A lighter "slate" built-in: a cooler, less-black chrome with the same semantic tokens, for a bright room.
  /// Chrome values match skirnir's `light_slate` exactly.
  pub const fn light_slate() -> Self {
    Palette {
      bg: rgb(0x1E2227),
      panel: rgb(0x262B31),
      panel_alt: rgb(0x2E343B),
      inset: rgb(0x171A1E),
      widget: rgb(0x363D45),
      widget_hover: rgb(0x40474F),
      widget_active: rgb(0x4A525B),
      divider: rgb(0x3A4149),
      border_recess: rgb(0x101317),
      border_raised: rgb(0x49515A),
      text: rgb(0xECEFF2),
      text_dim: rgb(0xA8B0B8),
      text_disabled: rgb(0x6B747D),
      grid_major: rgb(0x262B31),
      grid_minor: rgb(0x1B1F24),
      // Accents, status, and the canvas semantics are identical to the default so meaning never shifts between
      // built-ins — only the chrome lightens.
      ..Self::default_dark()
    }
  }

  /// A deeper "midnight" built-in: a cool blue-black chrome, darker still than the default. Chrome values match
  /// skirnir's `midnight` exactly; every semantic token stays the default.
  pub const fn midnight() -> Self {
    Palette {
      bg: rgb(0x0A0E16),
      panel: rgb(0x10151F),
      panel_alt: rgb(0x161C28),
      inset: rgb(0x070A11),
      widget: rgb(0x1C2434),
      widget_hover: rgb(0x232C3F),
      widget_active: rgb(0x2A3449),
      divider: rgb(0x1E2638),
      border_recess: rgb(0x04060A),
      border_raised: rgb(0x2C3850),
      grid_major: rgb(0x10151F),
      grid_minor: rgb(0x0A0E16),
      ..Self::default_dark()
    }
  }
}

impl Default for Palette {
  fn default() -> Self {
    Self::default_dark()
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn default_dark_matches_the_shared_design_hex_values() {
    // The chrome must be skirnir's exact values — that shared chrome is what makes the two apps read as one
    // family; a drift here is a design regression, not a tweak.
    let p = Palette::default_dark();
    assert_eq!(p.bg, Color32::from_rgb(0x12, 0x12, 0x12));
    assert_eq!(p.panel, Color32::from_rgb(0x1B, 0x1B, 0x1B));
    assert_eq!(p.inset, Color32::from_rgb(0x0E, 0x0E, 0x0E));
    assert_eq!(p.accent, Color32::from_rgb(0x0E, 0x86, 0xD4));
    assert_eq!(p.toolpath_cut, Color32::from_rgb(0xFF, 0xE0, 0x00), "cuts share skirnir's trail yellow");
  }

  #[test]
  fn alternate_builtins_keep_the_semantic_tokens_and_only_shift_chrome() {
    let dark = Palette::default_dark();
    for alt in [Palette::light_slate(), Palette::midnight()] {
      assert_eq!(alt.accent, dark.accent, "the control accent is shared across built-ins");
      assert_eq!(alt.copper, dark.copper, "copper stays copper in every chrome");
      assert_eq!(alt.toolpath_cut, dark.toolpath_cut, "the cut colour is shared across built-ins");
      assert_eq!(alt.state_error, dark.state_error, "the error colour is shared across built-ins");
      assert_ne!(alt.panel, dark.panel, "an alternate built-in must change the chrome");
    }
  }

  #[test]
  fn rgb_helper_unpacks_channels() {
    assert_eq!(rgb(0x102030), Color32::from_rgb(0x10, 0x20, 0x30));
  }
}
