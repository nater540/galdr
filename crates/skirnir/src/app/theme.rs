//! Theme tokens for the skirnir UI: the exact palette from `Skirnir.dc.html` (§01), kept central so the views
//! never hard-code raw colours and the look is tuned in one place.
//!
//! The design is "dark, near-black, flat — no shadows except popovers". Every token here maps 1:1 to a value in
//! the design doc and is named for its semantic intent ("this is the alarm surface", "this is the motion
//! accent"). Only `egui::Color32` is referenced, so this stays a thin token layer; the *decision* of which
//! semantic state a badge is in lives in the egui-free [`super::badge`] module, and this module turns that
//! [`BadgeState`] into a concrete colour.

use eframe::egui::Color32;

use super::badge::BadgeState;
use super::intent::Axis;

/// Build a `Color32` from a `0xRRGGBB` literal at compile time, so the tokens read as the design's hex values.
const fn rgb(hex: u32) -> Color32 {
  Color32::from_rgb((hex >> 16) as u8, (hex >> 8) as u8, hex as u8)
}

/// The skirnir colour palette. A dark, high-contrast, flat scheme suited to a machine-control dashboard read at
/// a glance in a dim shop. All values are taken verbatim from the design tokens table.
pub struct Theme;

impl Theme {
  // ── Chrome & surface ──────────────────────────────────────────────────────────────────────────────────
  /// `bg.window` — the app window / outermost fill.
  pub const BG: Color32 = rgb(0x121212);
  /// `bg.panel` — panel / dock body.
  pub const PANEL: Color32 = rgb(0x1B1B1B);
  /// `bg.panelAlt` — toolbar / tab strip / section header.
  pub const PANEL_ALT: Color32 = rgb(0x222222);
  /// `bg.inset` — console / fields / viewport (recessed surfaces).
  pub const INSET: Color32 = rgb(0x0E0E0E);
  /// `bg.widget` — control rest (`widgets.inactive`).
  pub const WIDGET: Color32 = rgb(0x2A2A2A);
  /// `bg.widget.hover` — `widgets.hovered`.
  pub const WIDGET_HOVER: Color32 = rgb(0x333333);
  /// `bg.widget.active` — `widgets.active` / pressed / selected segment.
  pub const WIDGET_ACTIVE: Color32 = rgb(0x3C3C3C);
  /// `divider` — 1px panel separators.
  pub const DIVIDER: Color32 = rgb(0x2E2E2E);
  /// `border.recess` — inset top edge (no bevel).
  pub const BORDER_RECESS: Color32 = rgb(0x080808);
  /// `border.raised` — raised control edge.
  pub const BORDER_RAISED: Color32 = rgb(0x3A3A3A);

  // ── Accents ───────────────────────────────────────────────────────────────────────────────────────────
  /// `accent.primary` — selection / focus / primary actions / control highlight (cool blue).
  pub const ACCENT: Color32 = rgb(0x0E86D4);
  /// `accent.primary` hover.
  pub const ACCENT_HOVER: Color32 = rgb(0x2BA8F0);
  /// `accent.primary` active/pressed.
  pub const ACCENT_ACTIVE: Color32 = rgb(0x0B6FB0);
  /// `accent.secondary` — reserved for "right now": the tool dot, the traversed toolpath, the realized-spindle
  /// override. Warm orange carries *motion*; cool blue carries *control*.
  pub const ACCENT_MOTION: Color32 = rgb(0xFF7A1A);

  // ── Text on dark ──────────────────────────────────────────────────────────────────────────────────────
  /// `text.primary` — primary readable text.
  pub const TEXT: Color32 = rgb(0xE4E4E4);
  /// `text.secondary` — labels, hints, units.
  pub const TEXT_DIM: Color32 = rgb(0x9A9A9A);
  /// `text.disabled` — disabled control text / faint markers.
  pub const TEXT_DISABLED: Color32 = rgb(0x5C5C5C);

  // ── Machine-state dots (paired with an uppercase label, never colour alone) ─────────────────────────────
  /// Idle dot.
  pub const STATE_IDLE: Color32 = rgb(0x3B82C4);
  /// Run dot (also the progress-bar fill).
  pub const STATE_RUN: Color32 = rgb(0x3FB861);
  /// Hold dot.
  pub const STATE_HOLD: Color32 = rgb(0xE0A33E);
  /// Alarm dot.
  pub const STATE_ALARM: Color32 = rgb(0xE5484D);
  /// Jog dot.
  pub const STATE_JOG: Color32 = rgb(0x2BB6C9);
  /// Check / Sleep accent (violet — also the settings `$NNN` key colour).
  pub const STATE_CHECK: Color32 = rgb(0x9B7FE0);
  /// Sleep / Disconnected neutral.
  pub const STATE_NEUTRAL: Color32 = rgb(0x6B6B6B);

  // ── Alarm surface trio (banner / fault badge) ───────────────────────────────────────────────────────────
  /// Alarm surface fill.
  pub const ALARM_BG: Color32 = rgb(0x2A0F11);
  /// Alarm surface border.
  pub const ALARM_BORDER: Color32 = rgb(0x5A2528);
  /// Alarm surface text (a lighter red so the label is readable on the dark fill).
  pub const ALARM_TEXT: Color32 = rgb(0xFF8488);

  // ── Console line types ──────────────────────────────────────────────────────────────────────────────────
  /// Sent-line chevron (`>`), blue.
  pub const LOG_SENT: Color32 = rgb(0x0E86D4);
  /// Response chevron (`<` for `ok`), green.
  pub const LOG_RECV: Color32 = rgb(0x3FB861);
  /// Status chevron (`<` for `<...>`), violet.
  pub const LOG_STATUS: Color32 = rgb(0x9B7FE0);
  /// Info / `[MSG:]` line, amber.
  pub const LOG_INFO: Color32 = rgb(0xE0A33E);
  /// Timestamp prefix / dim notice text.
  pub const LOG_NOTICE: Color32 = rgb(0x5C5C5C);

  /// `accent.secondary` alias kept for the toolpath viewport's "motion" semantics.
  pub const OK: Color32 = Self::STATE_RUN;
  /// Caution alias (hold / door).
  pub const WARN: Color32 = Self::STATE_HOLD;
  /// Danger alias (alarm / error / disconnect fault).
  pub const DANGER: Color32 = Self::STATE_ALARM;

  /// The signature dot colour for a badge state, used by the status badge and the banner. The mapping is the
  /// single source of truth so the toolbar badge and the status bar never drift apart.
  pub fn badge_color(state: BadgeState) -> Color32 {
    match state {
      BadgeState::Disconnected | BadgeState::Sleep => Self::STATE_NEUTRAL,
      BadgeState::Connecting => Self::ACCENT,
      BadgeState::Idle => Self::STATE_IDLE,
      BadgeState::Run | BadgeState::Home => Self::STATE_RUN,
      BadgeState::Jog => Self::STATE_JOG,
      BadgeState::Hold | BadgeState::Door => Self::STATE_HOLD,
      BadgeState::Check => Self::STATE_CHECK,
      BadgeState::Alarm | BadgeState::Error => Self::STATE_ALARM,
    }
  }

  /// The axis-letter colour used in the DRO and the viewport HUD: X green, Y blue, Z amber (design §03).
  pub fn axis_color(axis: Axis) -> Color32 {
    match axis {
      Axis::X => Self::STATE_RUN,
      Axis::Y => Self::STATE_IDLE,
      Axis::Z => Self::STATE_HOLD,
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn chrome_tokens_match_the_design_hex_values() {
    assert_eq!(Theme::BG, Color32::from_rgb(0x12, 0x12, 0x12));
    assert_eq!(Theme::PANEL, Color32::from_rgb(0x1B, 0x1B, 0x1B));
    assert_eq!(Theme::INSET, Color32::from_rgb(0x0E, 0x0E, 0x0E));
    assert_eq!(Theme::ACCENT, Color32::from_rgb(0x0E, 0x86, 0xD4));
    assert_eq!(Theme::ACCENT_MOTION, Color32::from_rgb(0xFF, 0x7A, 0x1A));
  }

  #[test]
  fn run_uses_green_idle_blue_and_faults_red() {
    assert_eq!(Theme::badge_color(BadgeState::Run), Theme::STATE_RUN);
    assert_eq!(Theme::badge_color(BadgeState::Idle), Theme::STATE_IDLE);
    assert_eq!(Theme::badge_color(BadgeState::Alarm), Theme::STATE_ALARM);
    assert_eq!(Theme::badge_color(BadgeState::Error), Theme::STATE_ALARM);
    assert_eq!(Theme::badge_color(BadgeState::Jog), Theme::STATE_JOG);
  }

  #[test]
  fn axis_colors_follow_xyz_green_blue_amber() {
    assert_eq!(Theme::axis_color(Axis::X), Theme::STATE_RUN);
    assert_eq!(Theme::axis_color(Axis::Y), Theme::STATE_IDLE);
    assert_eq!(Theme::axis_color(Axis::Z), Theme::STATE_HOLD);
  }

  #[test]
  fn rgb_helper_unpacks_channels() {
    assert_eq!(rgb(0x102030), Color32::from_rgb(0x10, 0x20, 0x30));
  }
}
