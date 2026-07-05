//! `serde` adapters for the foundational `eitri-core` value types.
//!
//! `eitri-core`'s `Unit`, `Affine`, and `Length` deliberately carry no `serde` derives — the on-disk schema is owned
//! entirely by this crate so a future refactor of those internal structs cannot silently change the file format.
//! These `with`-adapters pin the wire shape here instead: `Unit` as a short tag, `Affine` as its six coefficients,
//! and `Length` as a bare millimetre scalar (millimetres are `eitri-core`'s documented canonical representation).

use eitri_core::{Affine, Length, Unit};
use serde::de::{Deserializer, Error as _};
use serde::ser::Serializer;
use serde::{Deserialize, Serialize};

/// (De)serialize [`Unit`] as the tag `"mm"` or `"inch"`.
pub mod unit {
  use super::*;

  /// Serialize a unit as its short tag.
  pub fn serialize<S: Serializer>(unit: &Unit, ser: S) -> Result<S::Ok, S::Error> {
    let tag = match unit {
      Unit::Millimeters => "mm",
      Unit::Inches => "inch",
    };
    ser.serialize_str(tag)
  }

  /// Deserialize a unit from its short tag, rejecting anything else.
  pub fn deserialize<'de, D: Deserializer<'de>>(de: D) -> Result<Unit, D::Error> {
    let tag = String::deserialize(de)?;
    match tag.as_str() {
      "mm" => Ok(Unit::Millimeters),
      "inch" => Ok(Unit::Inches),
      other => Err(D::Error::custom(format!("unknown unit tag '{other}'"))),
    }
  }
}

/// (De)serialize [`Affine`] as its six coefficients `[a, b, c, d, e, f]`.
pub mod affine {
  use super::*;

  /// Serialize an affine transform as a fixed six-element array.
  pub fn serialize<S: Serializer>(affine: &Affine, ser: S) -> Result<S::Ok, S::Error> {
    [affine.a, affine.b, affine.c, affine.d, affine.e, affine.f].serialize(ser)
  }

  /// Deserialize an affine transform from a six-element array.
  pub fn deserialize<'de, D: Deserializer<'de>>(de: D) -> Result<Affine, D::Error> {
    let [a, b, c, d, e, f] = <[f64; 6]>::deserialize(de)?;
    Ok(Affine { a, b, c, d, e, f })
  }
}

/// (De)serialize [`Length`] as a bare millimetre scalar.
pub mod length_mm {
  use super::*;

  /// Serialize a length as its canonical millimetre value.
  pub fn serialize<S: Serializer>(length: &Length, ser: S) -> Result<S::Ok, S::Error> {
    ser.serialize_f64(length.as_mm())
  }

  /// Deserialize a length from a millimetre scalar.
  pub fn deserialize<'de, D: Deserializer<'de>>(de: D) -> Result<Length, D::Error> {
    Ok(Length::from_mm(f64::deserialize(de)?))
  }
}
