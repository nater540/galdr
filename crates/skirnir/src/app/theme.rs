//! The runtime [`Palette`]: the skirnir UI's colour scheme as an *instance* (one field per design token) rather
//! than associated constants, so the active colours can come from the loaded config instead of being baked in.
//!
//! The design is "dark, near-black, flat — no shadows except popovers". Every field maps 1:1 to a value in
//! `Skirnir.dc.html` (§01) and is named for its semantic intent ("this is the alarm surface", "this is the motion
//! accent"). [`Palette::default_dark`] carries the design's exact values — the same hexes that used to live as
//! `Theme::*` consts — and is the built-in `"default"` theme. A couple of alternate built-ins
//! ([`Palette::light_slate`], [`Palette::midnight`]) ship alongside it, selectable by name from the config.
//!
//! Views read a `&Palette` threaded through `UiState` (see [`crate::app::views`]); the *decision* of which semantic
//! state a badge is in still lives in the egui-free [`super::badge`] module, and the palette turns that
//! [`BadgeState`] into a concrete [`egui::Color32`] via [`Palette::badge_color`]. Only `egui::Color32` is referenced
//! here, so this stays a thin token layer.

use eframe::egui::Color32;

use super::badge::BadgeState;
use super::intent::Axis;

/// Build a `Color32` from a `0xRRGGBB` literal at compile time, so the default tokens read as the design's hex
/// values. Opaque — the design palette carries no per-token alpha.
const fn rgb(hex: u32) -> Color32 {
  Color32::from_rgb((hex >> 16) as u8, (hex >> 8) as u8, hex as u8)
}

/// The skirnir colour palette as runtime state: a dark, high-contrast, flat scheme suited to a machine-control
/// dashboard read at a glance in a dim shop. One field per design token. Cloned cheaply into `UiState` each time the
/// config resolves a theme, so views render against the active colours without new parameters on every view fn.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Palette {
  // ── Chrome & surface ──────────────────────────────────────────────────────────────────────────────────
  /// `bg.window` — the app window / outermost fill.
  pub bg: Color32,
  /// `bg.panel` — panel / dock body.
  pub panel: Color32,
  /// `bg.panelAlt` — toolbar / tab strip / section header.
  pub panel_alt: Color32,
  /// `bg.inset` — console / fields / viewport (recessed surfaces).
  pub inset: Color32,
  /// `bg.widget` — control rest (`widgets.inactive`).
  pub widget: Color32,
  /// `bg.widget.hover` — `widgets.hovered`.
  pub widget_hover: Color32,
  /// `bg.widget.active` — `widgets.active` / pressed / selected segment.
  pub widget_active: Color32,
  /// `divider` — 1px panel separators.
  pub divider: Color32,
  /// `border.recess` — inset top edge (no bevel).
  pub border_recess: Color32,
  /// `border.raised` — raised control edge.
  pub border_raised: Color32,

  // ── Accents ───────────────────────────────────────────────────────────────────────────────────────────
  /// `accent.primary` — selection / focus / primary actions / control highlight (cool blue).
  pub accent: Color32,
  /// `accent.primary` hover.
  pub accent_hover: Color32,
  /// `accent.primary` active/pressed.
  pub accent_active: Color32,
  /// `accent.secondary` — "right now": the tool dot, the traversed toolpath, the realized-spindle override. Warm
  /// orange carries *motion*; cool blue carries *control*.
  pub accent_motion: Color32,

  // ── Text on dark ──────────────────────────────────────────────────────────────────────────────────────
  /// `text.primary` — primary readable text.
  pub text: Color32,
  /// `text.secondary` — labels, hints, units.
  pub text_dim: Color32,
  /// `text.disabled` — disabled control text / faint markers.
  pub text_disabled: Color32,

  // ── Machine-state dots (paired with an uppercase label, never colour alone) ─────────────────────────────
  /// Idle dot.
  pub state_idle: Color32,
  /// Run dot (also the progress-bar fill).
  pub state_run: Color32,
  /// Hold dot.
  pub state_hold: Color32,
  /// Alarm dot.
  pub state_alarm: Color32,
  /// Jog dot.
  pub state_jog: Color32,
  /// Check / Sleep accent (violet — also the settings `$NNN` key colour).
  pub state_check: Color32,
  /// Sleep / Disconnected neutral.
  pub state_neutral: Color32,

  // ── Alarm surface trio (banner / fault badge) ───────────────────────────────────────────────────────────
  /// Alarm surface fill.
  pub alarm_bg: Color32,
  /// Alarm surface border.
  pub alarm_border: Color32,
  /// Alarm surface text (a lighter red so the label is readable on the dark fill).
  pub alarm_text: Color32,

  // ── Console line types ──────────────────────────────────────────────────────────────────────────────────
  /// Sent-line chevron (`>`), blue.
  pub log_sent: Color32,
  /// Response chevron (`<` for `ok`), green.
  pub log_recv: Color32,
  /// Status chevron (`<` for `<...>`), violet.
  pub log_status: Color32,
  /// Info / `[MSG:]` line, amber.
  pub log_info: Color32,
  /// Timestamp prefix / dim notice text.
  pub log_notice: Color32,

  // ── Toolpath viewport ───────────────────────────────────────────────────────────────────────────────────
  /// The cut (programmed feed) trail colour — the warm motion accent.
  pub toolpath_cut: Color32,
  /// The rapid (travel) trail colour — the cool control accent.
  pub toolpath_rapid: Color32,
  /// The viewport grid's major (every 5th) line colour.
  pub grid_major: Color32,
  /// The viewport grid's minor line colour.
  pub grid_minor: Color32,
}

impl Palette {
  /// The built-in `"default"` theme: the design's exact dark palette, carrying the same hex values that used to
  /// live as the `Theme::*` associated constants verbatim. This is the canonical baseline every config theme
  /// resolves over, and the fallback when an `active_theme` cannot be found.
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
      accent_motion: rgb(0xFF7A1A),
      text: rgb(0xE4E4E4),
      text_dim: rgb(0x9A9A9A),
      text_disabled: rgb(0x5C5C5C),
      state_idle: rgb(0x3B82C4),
      state_run: rgb(0x3FB861),
      state_hold: rgb(0xE0A33E),
      state_alarm: rgb(0xE5484D),
      state_jog: rgb(0x2BB6C9),
      state_check: rgb(0x9B7FE0),
      state_neutral: rgb(0x6B6B6B),
      alarm_bg: rgb(0x2A0F11),
      alarm_border: rgb(0x5A2528),
      alarm_text: rgb(0xFF8488),
      log_sent: rgb(0x0E86D4),
      log_recv: rgb(0x3FB861),
      log_status: rgb(0x9B7FE0),
      log_info: rgb(0xE0A33E),
      log_notice: rgb(0x5C5C5C),
      // The cut trail/marker base colour is yellow; the live trail shades it darker by cut depth (the deeper the
      // pass, the dimmer the line). The rapid token is retained for config compatibility but no longer drawn — the
      // trail records only below-surface cuts now. The grid is the panel fill at two intensities (major = panel,
      // minor = panel dimmed).
      toolpath_cut: rgb(0xFFE000),
      toolpath_rapid: rgb(0x0E86D4),
      grid_major: rgb(0x1B1B1B),
      grid_minor: rgb(0x0D0D0D),
    }
  }

  /// A lighter "slate" built-in: a cooler, less-black chrome with the same semantic accents, for operators who find
  /// the near-black default too dark in a bright room. Still a dark theme — egui's `Visuals::dark` underlies it —
  /// just a few shades up, so contrast and the state-colour meanings are preserved.
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
      // Accents, state dots, alarm trio, console types, and the toolpath trails are identical to the default so the
      // dashboard's *meaning* never shifts between built-ins — only the chrome lightens.
      ..Self::default_dark()
    }
  }

  /// A deeper "midnight" built-in: a cool blue-black chrome, darker still than the default, for very dim shops.
  /// Like [`Self::light_slate`] it keeps every semantic accent/state colour, changing only the surface tones.
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
      // Accents, state dots, alarm trio, console types, and the toolpath trails stay the default so meaning holds;
      // only the cool blue-black chrome differs.
      ..Self::default_dark()
    }
  }

  /// The signature dot colour for a badge state, used by the status badge and the banner. The mapping is the
  /// single source of truth so the toolbar badge and the status bar never drift apart.
  pub fn badge_color(&self, state: BadgeState) -> Color32 {
    match state {
      BadgeState::Disconnected | BadgeState::Sleep => self.state_neutral,
      BadgeState::Connecting => self.accent,
      BadgeState::Idle => self.state_idle,
      BadgeState::Run | BadgeState::Home => self.state_run,
      BadgeState::Jog => self.state_jog,
      BadgeState::Hold | BadgeState::Door => self.state_hold,
      // A manual tool change is an operator-attention pause; the violet accent sets it apart from the amber hold.
      BadgeState::Check | BadgeState::Tool => self.state_check,
      BadgeState::Alarm | BadgeState::Error => self.state_alarm,
    }
  }

  /// The axis-letter colour used in the DRO and the viewport HUD: X green, Y blue, Z amber (design §03), and the
  /// rotary A violet — the check/settings accent, distinct from all three linear axes so a 4-axis readout scans
  /// at a glance.
  pub fn axis_color(&self, axis: Axis) -> Color32 {
    match axis {
      Axis::X => self.state_run,
      Axis::Y => self.state_idle,
      Axis::Z => self.state_hold,
      Axis::A => self.state_check,
    }
  }

  /// Sample the fixed six-stripe pride rainbow as a smooth left-to-right gradient at `t` in `[0, 1]`, linearly
  /// interpolating between the two nearest stripe stops. `t` is clamped, so out-of-range inputs saturate to the end
  /// colours rather than wrapping. This paints the thin "fabulous"-mode accent band so it blends rather than showing
  /// six hard bands. The pride palette is fixed (it deliberately sits apart from the themeable semantic colours, so
  /// the dashboard's meaning never depends on it), hence an associated `&self`-free constant rather than a field.
  pub fn pride_at(t: f32) -> Color32 {
    let stops = PRIDE.len();
    let t = t.clamp(0.0, 1.0);
    // Position along the (stops − 1) segments; `seg` is the lower stop, `frac` the blend into the next.
    let scaled = t * (stops - 1) as f32;
    let seg = (scaled as usize).min(stops - 2);
    let frac = scaled - seg as f32;
    let lo = PRIDE[seg];
    let hi = PRIDE[seg + 1];
    let lerp = |a: u8, b: u8| (a as f32 + (b as f32 - a as f32) * frac).round() as u8;
    Color32::from_rgb(lerp(lo.r(), hi.r()), lerp(lo.g(), hi.g()), lerp(lo.b(), hi.b()))
  }
}

impl Default for Palette {
  fn default() -> Self {
    Self::default_dark()
  }
}

/// The six-stripe rainbow pride palette, top-of-flag (red) → bottom (violet). Used *only* by the hidden "fabulous"
/// accent ([`Palette::pride_at`]); it deliberately sits apart from the themeable semantic colours so the dashboard's
/// meaning never depends on it. Values are the canonical 1979 six-stripe flag hexes.
pub const PRIDE: [Color32; 6] = [
  rgb(0xE40303), // red
  rgb(0xFF8C00), // orange
  rgb(0xFFED00), // yellow
  rgb(0x008026), // green
  rgb(0x004DFF), // blue
  rgb(0x750787), // violet
];

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn default_dark_matches_the_design_hex_values() {
    let p = Palette::default_dark();
    assert_eq!(p.bg, Color32::from_rgb(0x12, 0x12, 0x12));
    assert_eq!(p.panel, Color32::from_rgb(0x1B, 0x1B, 0x1B));
    assert_eq!(p.inset, Color32::from_rgb(0x0E, 0x0E, 0x0E));
    assert_eq!(p.accent, Color32::from_rgb(0x0E, 0x86, 0xD4));
    assert_eq!(p.accent_motion, Color32::from_rgb(0xFF, 0x7A, 0x1A));
  }

  #[test]
  fn run_uses_green_idle_blue_and_faults_red() {
    let p = Palette::default_dark();
    assert_eq!(p.badge_color(BadgeState::Run), p.state_run);
    assert_eq!(p.badge_color(BadgeState::Idle), p.state_idle);
    assert_eq!(p.badge_color(BadgeState::Alarm), p.state_alarm);
    assert_eq!(p.badge_color(BadgeState::Error), p.state_alarm);
    assert_eq!(p.badge_color(BadgeState::Jog), p.state_jog);
  }

  #[test]
  fn axis_colors_follow_xyz_green_blue_amber_and_a_violet() {
    let p = Palette::default_dark();
    assert_eq!(p.axis_color(Axis::X), p.state_run);
    assert_eq!(p.axis_color(Axis::Y), p.state_idle);
    assert_eq!(p.axis_color(Axis::Z), p.state_hold);
    // The rotary A takes the violet accent: distinct from all three linear axes so a 4-axis DRO scans at a glance.
    assert_eq!(p.axis_color(Axis::A), p.state_check);
  }

  #[test]
  fn rgb_helper_unpacks_channels() {
    assert_eq!(rgb(0x102030), Color32::from_rgb(0x10, 0x20, 0x30));
  }

  #[test]
  fn alternate_builtins_keep_the_semantic_accents_and_only_shift_chrome() {
    // The whole point of the alternates: a different chrome, the same meaning. The accent/state colours must match
    // the default so a badge reads the same colour across themes; the panel/window fills must differ.
    let dark = Palette::default_dark();
    for alt in [Palette::light_slate(), Palette::midnight()] {
      assert_eq!(alt.accent, dark.accent, "the control accent is shared across built-ins");
      assert_eq!(alt.state_run, dark.state_run, "the run colour is shared across built-ins");
      assert_eq!(alt.state_alarm, dark.state_alarm, "the alarm colour is shared across built-ins");
      assert_ne!(alt.panel, dark.panel, "an alternate built-in must change the chrome");
    }
  }

  #[test]
  fn pride_gradient_pins_its_endpoints_and_clamps() {
    assert_eq!(Palette::pride_at(0.0), PRIDE[0]);
    assert_eq!(Palette::pride_at(1.0), PRIDE[5]);
    assert_eq!(Palette::pride_at(-1.0), PRIDE[0]);
    assert_eq!(Palette::pride_at(2.0), PRIDE[5]);
  }

  #[test]
  fn pride_gradient_blends_between_stops() {
    let mid = Palette::pride_at(0.1); // 0.1 * 5 = 0.5 of the way from red into orange.
    let (red, orange) = (PRIDE[0], PRIDE[1]);
    assert_eq!(mid.r(), ((red.r() as f32 + orange.r() as f32) / 2.0).round() as u8);
    assert_ne!(mid, red);
    assert_ne!(mid, orange);
  }
}
