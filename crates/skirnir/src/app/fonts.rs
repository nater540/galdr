//! Vendored typefaces and the egui font stack wiring.
//!
//! The design (`Skirnir.dc.html`) calls for **Roboto** as the proportional UI face and **JetBrains Mono** for
//! the DRO digits, the console, and the monospace status strip — the readouts that want tabular figures.
//! egui's stock build only ships its own default faces, so we vendor static-weight TTFs into the binary via
//! `include_bytes!` and register them as the heads of egui's `Proportional` and `Monospace` families.
//!
//! egui does not pick a weight *within* a family — it renders with the first font registered for that family —
//! so the family heads are the Regular faces. The Medium/Bold faces are vendored too (the design uses Roboto
//! 400/500/700 and JetBrains Mono 400/500) and registered under their own named keys so a view can reach a
//! specific weight via `FontFamily::Name(..)` when it needs to; the default render path uses Regular.
//!
//! Building the [`egui::FontDefinitions`] is pure, egui-only data assembly with no I/O and no window, so the
//! family-mapping decision is unit-tested directly. Installing them ([`install`]) runs once in the eframe
//! creation closure, alongside the theme.

use eframe::egui::{self, FontData, FontDefinitions, FontFamily};
use std::sync::Arc;

// The vendored faces, compiled into the binary. These are real static-weight TrueType files (verified at
// vendor time: sfnt magic `0x00010000`); see `assets/fonts/NOTICE.md` for sources and licenses (both OFL-1.1).
const ROBOTO_REGULAR: &[u8] = include_bytes!("../../assets/fonts/roboto/Roboto-Regular.ttf");
const ROBOTO_MEDIUM: &[u8] = include_bytes!("../../assets/fonts/roboto/Roboto-Medium.ttf");
const ROBOTO_BOLD: &[u8] = include_bytes!("../../assets/fonts/roboto/Roboto-Bold.ttf");
const JETBRAINS_MONO_REGULAR: &[u8] = include_bytes!("../../assets/fonts/jetbrains-mono/JetBrainsMono-Regular.ttf");
const JETBRAINS_MONO_MEDIUM: &[u8] = include_bytes!("../../assets/fonts/jetbrains-mono/JetBrainsMono-Medium.ttf");
const JETBRAINS_MONO_BOLD: &[u8] = include_bytes!("../../assets/fonts/jetbrains-mono/JetBrainsMono-Bold.ttf");

/// The named font keys we register, so the family heads and any explicit-weight lookups refer to one source of
/// truth rather than scattering string literals. The `*_HEAD` keys are what `Proportional`/`Monospace` resolve
/// to; the others are available for a deliberate `FontFamily::Name(..)` pick.
pub const ROBOTO_REGULAR_KEY: &str = "roboto-regular";
pub const ROBOTO_MEDIUM_KEY: &str = "roboto-medium";
pub const ROBOTO_BOLD_KEY: &str = "roboto-bold";
pub const JETBRAINS_MONO_REGULAR_KEY: &str = "jetbrains-mono-regular";
pub const JETBRAINS_MONO_MEDIUM_KEY: &str = "jetbrains-mono-medium";
pub const JETBRAINS_MONO_BOLD_KEY: &str = "jetbrains-mono-bold";

/// Build the egui font stack with the vendored faces. Roboto Regular heads `Proportional`; JetBrains Mono
/// Regular heads `Monospace`. The opposite family's Regular is appended as a fallback so a glyph missing from
/// one face can still resolve from the other, and egui's bundled emoji/fallback fonts (added by
/// `FontDefinitions::default`) remain at the tail for symbols neither vendored face covers.
pub fn definitions() -> FontDefinitions {
  // Start from egui's defaults so its emoji/fallback fonts stay registered, then prepend our faces as the
  // family heads. `from_static` borrows the `'static` `include_bytes!` data — no copy, no allocation per byte.
  let mut fonts = FontDefinitions::default();

  fonts.font_data.insert(ROBOTO_REGULAR_KEY.to_owned(), Arc::new(FontData::from_static(ROBOTO_REGULAR)));
  fonts.font_data.insert(ROBOTO_MEDIUM_KEY.to_owned(), Arc::new(FontData::from_static(ROBOTO_MEDIUM)));
  fonts.font_data.insert(ROBOTO_BOLD_KEY.to_owned(), Arc::new(FontData::from_static(ROBOTO_BOLD)));
  fonts
    .font_data
    .insert(JETBRAINS_MONO_REGULAR_KEY.to_owned(), Arc::new(FontData::from_static(JETBRAINS_MONO_REGULAR)));
  fonts
    .font_data
    .insert(JETBRAINS_MONO_MEDIUM_KEY.to_owned(), Arc::new(FontData::from_static(JETBRAINS_MONO_MEDIUM)));
  fonts.font_data.insert(JETBRAINS_MONO_BOLD_KEY.to_owned(), Arc::new(FontData::from_static(JETBRAINS_MONO_BOLD)));

  // Proportional: Roboto Regular first, JetBrains Mono Regular as a glyph fallback, then egui's defaults.
  let proportional = fonts.families.entry(FontFamily::Proportional).or_default();
  proportional.insert(0, JETBRAINS_MONO_REGULAR_KEY.to_owned());
  proportional.insert(0, ROBOTO_REGULAR_KEY.to_owned());

  // Monospace: JetBrains Mono Regular first (the DRO/console face), Roboto Regular as a fallback, then defaults.
  let monospace = fonts.families.entry(FontFamily::Monospace).or_default();
  monospace.insert(0, ROBOTO_REGULAR_KEY.to_owned());
  monospace.insert(0, JETBRAINS_MONO_REGULAR_KEY.to_owned());

  fonts
}

/// Install the vendored font stack into the egui context. Called once from the eframe creation closure
/// alongside the theme; an init path where the bytes are vendored-and-verified, so there is no recoverable
/// runtime failure to surface — `set_fonts` simply replaces egui's font set with ours.
pub fn install(ctx: &egui::Context) {
  ctx.set_fonts(definitions());
}

#[cfg(test)]
mod tests {
  use super::*;

  /// The vendored bytes must be real TrueType `sfnt` files (`0x00010000`), not HTML error pages or variable-
  /// font placeholders — a guard against a future re-vendor pulling a 404 body or the wrong asset.
  #[test]
  fn vendored_faces_are_truetype_sfnt() {
    for (name, bytes) in [
      ("Roboto-Regular", ROBOTO_REGULAR),
      ("Roboto-Medium", ROBOTO_MEDIUM),
      ("Roboto-Bold", ROBOTO_BOLD),
      ("JetBrainsMono-Regular", JETBRAINS_MONO_REGULAR),
      ("JetBrainsMono-Medium", JETBRAINS_MONO_MEDIUM),
      ("JetBrainsMono-Bold", JETBRAINS_MONO_BOLD),
    ] {
      assert!(bytes.len() > 50_000, "{name} is implausibly small ({} bytes) — likely not a real font", bytes.len());
      assert_eq!(&bytes[0..4], &[0x00, 0x01, 0x00, 0x00], "{name} is missing the TrueType sfnt magic");
    }
  }

  /// Every named key referenced for a family head must have backing font data registered, or egui would drop a
  /// dangling reference at layout time. This pins the key/data wiring together.
  #[test]
  fn every_named_key_has_font_data() {
    let fonts = definitions();
    for key in [
      ROBOTO_REGULAR_KEY,
      ROBOTO_MEDIUM_KEY,
      ROBOTO_BOLD_KEY,
      JETBRAINS_MONO_REGULAR_KEY,
      JETBRAINS_MONO_MEDIUM_KEY,
      JETBRAINS_MONO_BOLD_KEY,
    ] {
      assert!(fonts.font_data.contains_key(key), "no font data registered for `{key}`");
    }
  }

  /// The load-bearing mapping: Roboto leads the proportional family and JetBrains Mono leads the monospace
  /// family. This is what makes the DRO digits and the mono status strip render in JetBrains Mono and the rest
  /// of the UI in Roboto, so it gets an explicit head-of-list assertion.
  #[test]
  fn roboto_heads_proportional_and_jetbrains_mono_heads_monospace() {
    let fonts = definitions();
    let proportional = &fonts.families[&FontFamily::Proportional];
    let monospace = &fonts.families[&FontFamily::Monospace];

    assert_eq!(proportional.first().map(String::as_str), Some(ROBOTO_REGULAR_KEY), "Roboto must head Proportional");
    assert_eq!(
      monospace.first().map(String::as_str),
      Some(JETBRAINS_MONO_REGULAR_KEY),
      "JetBrains Mono must head Monospace (DRO/console tabular figures)"
    );

    // The opposite face stays in each list as a glyph fallback, after the head.
    assert!(proportional.iter().any(|k| k == JETBRAINS_MONO_REGULAR_KEY), "mono fallback missing from Proportional");
    assert!(monospace.iter().any(|k| k == ROBOTO_REGULAR_KEY), "Roboto fallback missing from Monospace");
  }
}
