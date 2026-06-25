//! [`ColorSpec`]: a serde-friendly hex colour the config file carries as a `"#RRGGBB"` / `"#RRGGBBAA"` string and
//! that resolves to an [`egui::Color32`] at load time.
//!
//! The config is hand-editable JSON, so colours are written the way a designer reads them — `"#0E86D4"` — rather
//! than as a `[14, 134, 212]` array or a packed integer. Parsing is permissive on input (case-insensitive, an
//! optional leading `#`, 6 or 8 hex digits) and canonical on output (always `#RRGGBB`, or `#RRGGBBAA` when the alpha
//! is not fully opaque), so a round-trip is stable and a hand-edited value still loads. A malformed string is a
//! deserialize error the *caller* downgrades to "use the default for this field" — `ColorSpec` itself never panics.

use std::fmt;

use eframe::egui::Color32;
use serde::de::{self, Visitor};
use serde::{Deserialize, Deserializer, Serialize, Serializer};

/// A colour parsed from / serialised to a hex string in the config file, resolving to an [`egui::Color32`]. Stored
/// as the four resolved channels so [`Self::to_color32`] is a trivial copy; the on-disk form is always the canonical
/// hex string (see [`Self::to_hex`]). Equality is channel-wise, so two specs that print the same hex compare equal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ColorSpec {
  /// Red channel, 0–255.
  pub r: u8,
  /// Green channel, 0–255.
  pub g: u8,
  /// Blue channel, 0–255.
  pub b: u8,
  /// Alpha channel, 0–255 (255 = fully opaque, the common case for chrome/accent colours).
  pub a: u8,
}

impl ColorSpec {
  /// A spec from explicit channels with full opacity — the ergonomic constructor for the built-in palette presets,
  /// which are authored as opaque `0xRRGGBB` values.
  pub const fn rgb(r: u8, g: u8, b: u8) -> Self {
    ColorSpec { r, g, b, a: 255 }
  }

  /// Build a spec from an [`egui::Color32`], reading its four channels. Note `Color32` stores PREmultiplied alpha,
  /// so this is the exact inverse of [`Self::to_color32`] only for opaque colours (alpha 255) — every palette colour
  /// is opaque, so the round-trip is exact in practice. Used when a built-in [`super::Palette`] is captured back into
  /// a serialisable form.
  pub const fn from_color32(color: Color32) -> Self {
    ColorSpec { r: color.r(), g: color.g(), b: color.b(), a: color.a() }
  }

  /// The resolved egui colour. A plain channel copy — the parse already happened at deserialize time, so this is
  /// cheap to call wherever a view needs the concrete colour. (Not `const`: egui's unmultiplied constructor is not a
  /// const fn.)
  pub fn to_color32(self) -> Color32 {
    Color32::from_rgba_unmultiplied(self.r, self.g, self.b, self.a)
  }

  /// Parse a `"#RRGGBB"` / `"#RRGGBBAA"` hex string (case-insensitive, leading `#` optional) into a spec. Returns
  /// `None` for any malformed input — a wrong length, a non-hex digit — so the caller can fall back to a default
  /// rather than the whole config failing. Never panics. A 6-digit value is fully opaque; an 8-digit value carries
  /// its own alpha.
  pub fn parse(text: &str) -> Option<Self> {
    let hex = text.trim().strip_prefix('#').unwrap_or(text.trim());
    if !hex.chars().all(|c| c.is_ascii_hexdigit()) {
      return None;
    }
    let byte = |start: usize| u8::from_str_radix(&hex[start..start + 2], 16).ok();
    match hex.len() {
      6 => Some(ColorSpec { r: byte(0)?, g: byte(2)?, b: byte(4)?, a: 255 }),
      8 => Some(ColorSpec { r: byte(0)?, g: byte(2)?, b: byte(4)?, a: byte(6)? }),
      _ => None,
    }
  }

  /// The canonical hex string: `#RRGGBB` when fully opaque, `#RRGGBBAA` when the alpha is not 255. Uppercase so the
  /// on-disk form matches the design tokens table. This is what serialisation writes, so a load/save round-trip is
  /// stable.
  pub fn to_hex(self) -> String {
    if self.a == 255 {
      format!("#{:02X}{:02X}{:02X}", self.r, self.g, self.b)
    } else {
      format!("#{:02X}{:02X}{:02X}{:02X}", self.r, self.g, self.b, self.a)
    }
  }
}

impl From<ColorSpec> for Color32 {
  fn from(spec: ColorSpec) -> Self {
    spec.to_color32()
  }
}

impl Serialize for ColorSpec {
  fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
    serializer.serialize_str(&self.to_hex())
  }
}

impl<'de> Deserialize<'de> for ColorSpec {
  fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
    deserializer.deserialize_str(ColorSpecVisitor)
  }
}

/// The serde visitor that turns a JSON string into a [`ColorSpec`], failing with a clear message on a malformed hex
/// value so the config loader can attribute the fallback to the offending field.
struct ColorSpecVisitor;

impl Visitor<'_> for ColorSpecVisitor {
  type Value = ColorSpec;

  fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
    formatter.write_str("a hex colour string like \"#RRGGBB\" or \"#RRGGBBAA\"")
  }

  fn visit_str<E: de::Error>(self, value: &str) -> Result<Self::Value, E> {
    ColorSpec::parse(value).ok_or_else(|| de::Error::custom(format!("invalid hex colour: {value:?}")))
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn parses_a_six_digit_hex_as_opaque() {
    let spec = ColorSpec::parse("#0E86D4").expect("a well-formed 6-digit hex parses");
    assert_eq!(spec, ColorSpec { r: 0x0E, g: 0x86, b: 0xD4, a: 255 }, "6 digits are opaque RGB");
    assert_eq!(spec.to_color32(), Color32::from_rgb(0x0E, 0x86, 0xD4));
  }

  #[test]
  fn parses_an_eight_digit_hex_with_alpha() {
    let spec = ColorSpec::parse("#0E86D480").expect("a well-formed 8-digit hex parses");
    assert_eq!(spec.a, 0x80, "the 8th/9th digits carry the alpha");
    assert_eq!(spec.to_color32(), Color32::from_rgba_unmultiplied(0x0E, 0x86, 0xD4, 0x80));
  }

  #[test]
  fn parsing_is_case_insensitive_and_the_hash_is_optional() {
    // A hand-edited file may drop the `#` or use lowercase; both must parse to the same colour.
    assert_eq!(ColorSpec::parse("#abcdef"), ColorSpec::parse("ABCDEF"));
    assert_eq!(ColorSpec::parse("0e86d4"), ColorSpec::parse("#0E86D4"));
  }

  #[test]
  fn malformed_hex_returns_none_never_panics() {
    // A wrong length, a non-hex digit, or empty input must all fail softly so the loader can default the field.
    assert_eq!(ColorSpec::parse("#12345"), None, "5 digits is not a valid colour");
    assert_eq!(ColorSpec::parse("#GGGGGG"), None, "non-hex digits fail");
    assert_eq!(ColorSpec::parse(""), None, "empty input fails");
    assert_eq!(ColorSpec::parse("#1234567"), None, "7 digits is neither RGB nor RGBA");
  }

  #[test]
  fn to_hex_is_canonical_and_drops_alpha_when_opaque() {
    assert_eq!(ColorSpec::rgb(0x0E, 0x86, 0xD4).to_hex(), "#0E86D4", "opaque prints 6 digits");
    assert_eq!(ColorSpec { r: 1, g: 2, b: 3, a: 0x80 }.to_hex(), "#01020380", "translucent prints 8 digits");
  }

  #[test]
  fn hex_round_trips_through_string() {
    // The core robustness claim: parse → to_hex → parse is a fixed point — the spec stores UNmultiplied channels
    // (exactly as parsed), so the hex form is stable even for a translucent colour.
    for raw in ["#0E86D4", "#FF7A1A", "#01020380", "#000000", "#FFFFFFFF"] {
      let spec = ColorSpec::parse(raw).expect("the sample parses");
      let reparsed = ColorSpec::parse(&spec.to_hex()).expect("its own hex parses back");
      assert_eq!(spec, reparsed, "{raw} must survive a hex round-trip");
    }
  }

  #[test]
  fn opaque_colors_round_trip_through_color32() {
    // egui's `Color32` stores PREmultiplied alpha, so a translucent spec is lossy through `Color32` (a documented
    // egui behaviour). Every palette colour is opaque, and an opaque colour survives the `Color32` round-trip
    // exactly — that is the property the views rely on.
    for raw in ["#0E86D4", "#FF7A1A", "#000000", "#FFFFFF"] {
      let spec = ColorSpec::parse(raw).expect("the sample parses");
      assert_eq!(ColorSpec::from_color32(spec.to_color32()), spec, "an opaque {raw} round-trips through Color32");
    }
  }

  #[test]
  fn serde_round_trips_through_json() {
    // The whole point of the newtype: it serialises as a bare JSON string and parses back, so a config file reads
    // `"accent": "#0E86D4"` rather than a struct of channels.
    let spec = ColorSpec::rgb(0x0E, 0x86, 0xD4);
    let json = serde_json::to_string(&spec).expect("serialises to a JSON string");
    assert_eq!(json, "\"#0E86D4\"", "the on-disk form is a bare hex string");
    let parsed: ColorSpec = serde_json::from_str(&json).expect("its own output parses back");
    assert_eq!(parsed, spec);
  }

  #[test]
  fn deserializing_a_malformed_hex_is_an_error_not_a_panic() {
    // A bad colour in the file is a deserialize error the config loader catches and downgrades to defaults; it must
    // never panic, and the error message must name the offending value.
    let err = serde_json::from_str::<ColorSpec>("\"#nothex\"").expect_err("a malformed hex must fail to deserialize");
    assert!(err.to_string().contains("#nothex"), "the error should name the bad value: {err}");
  }
}
