#![no_std]
#![deny(unsafe_code)]
//! Shared Protocol Buffers schema for galdr settings (DOC-04).
//!
//! This crate is the single source of truth for the settings wire format used in two places: the bytes
//! persisted to flash (wrapped in a versioned, CRC-checked frame by `firmware_core::settings::wire`) and
//! the `$PBX` host-sync channel skirnir uses to bulk read/write settings. It holds ONLY the generated
//! message types ([`Settings`], [`SettingsPatch`]) plus thin encode/decode helpers — no business logic,
//! no defaults, no validation. Those live in `firmware-core` so this crate stays a dependency-light leaf a
//! `no_std` firmware and a native sender can both share. See `proto/settings.proto` for the field map.

use micropb::{MessageDecode, MessageEncode, PbEncoder};

/// The generated protobuf types. micropb emits `#[derive(..)]`s and trait impls that reference `::micropb`;
/// the lints are relaxed because the code is machine-generated and not held to this crate's style rules.
mod generated {
  #![allow(clippy::all)]
  #![allow(non_snake_case, non_camel_case_types, unused, missing_docs)]
  // micropb output is machine-generated; if a future schema makes it emit `unsafe`, keep the crate-level
  // `deny(unsafe_code)` honest by scoping the exception to exactly this generated module.
  #![allow(unsafe_code)]
  include!(concat!(env!("OUT_DIR"), "/settings.rs"));
}

pub use generated::Settings;

/// A safe upper bound, in bytes, on the encoded length of a [`Settings`] message. The schema is ~40 scalar
/// fields (each a tag plus a varint or fixed32), so the real maximum is well under this; 512 leaves generous
/// headroom for any field added later. Callers size their wire/frame buffers from this. A host test asserts a
/// fully-populated message encodes within this bound.
pub const SETTINGS_MAX_LEN: usize = 512;

/// Failure encoding or decoding a settings message. Encode fails only if the destination buffer is too small;
/// decode fails on a malformed or truncated protobuf byte stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ProtoError {
  /// The destination buffer could not hold the encoded message.
  Encode,
  /// The input bytes were not a valid encoding of the expected message.
  Decode,
}

/// The number of bytes [`encode_settings_into`] will append for `msg` (its encoded protobuf size). Lets a
/// caller write a length prefix before the payload without a second encode pass.
pub fn settings_size(msg: &Settings) -> usize {
  msg.compute_size()
}

/// Encode `msg` as protobuf, appending the bytes to `out`. Returns [`ProtoError::Encode`] if `out` lacks the
/// capacity. The bytes are appended (not cleared first), so a caller can encode directly after a frame header
/// already pushed into `out`.
pub fn encode_settings_into<const N: usize>(msg: &Settings, out: &mut heapless::Vec<u8, N>) -> Result<(), ProtoError> {
  let mut encoder = PbEncoder::new(out);
  msg.encode(&mut encoder).map_err(|_| ProtoError::Encode)
}

/// Decode a full [`Settings`] message from `bytes`. Fields absent from the wire take their proto3 zero value;
/// the caller (`firmware-core`) is responsible for having applied real defaults beforehand and for validation.
pub fn decode_settings(bytes: &[u8]) -> Result<Settings, ProtoError> {
  let mut msg = Settings::default();
  msg.decode_from_bytes(bytes).map_err(|_| ProtoError::Decode)?;
  Ok(msg)
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn settings_round_trip_preserves_scalar_fields() {
    let mut msg = Settings::default();
    msg.step_pulse_us = 10;
    msg.junction_deviation_mm = 0.01;
    msg.steps_per_mm_x = 250.0;
    msg.steps_per_mm_y = 251.5;
    msg.steps_per_mm_z = 800.0;
    msg.homing_enable = true;
    msg.run_current_ma_x = 800;
    msg.microsteps_z = 16;
    msg.tmc_r_sense_ohms = 0.05;

    let mut buf: heapless::Vec<u8, SETTINGS_MAX_LEN> = heapless::Vec::new();
    encode_settings_into(&msg, &mut buf).expect("encode fits");
    assert_eq!(buf.len(), settings_size(&msg));

    let decoded = decode_settings(&buf).expect("decode succeeds");
    assert_eq!(decoded.step_pulse_us, 10);
    assert_eq!(decoded.junction_deviation_mm, 0.01);
    assert_eq!(decoded.steps_per_mm_x, 250.0);
    assert_eq!(decoded.steps_per_mm_y, 251.5);
    assert_eq!(decoded.steps_per_mm_z, 800.0);
    assert!(decoded.homing_enable);
    assert_eq!(decoded.run_current_ma_x, 800);
    assert_eq!(decoded.microsteps_z, 16);
    assert_eq!(decoded.tmc_r_sense_ohms, 0.05);
  }

  #[test]
  fn fully_populated_settings_fits_max_len() {
    // Set every field non-zero so the encoding is at its largest, and confirm it stays within SETTINGS_MAX_LEN.
    let mut msg = Settings::default();
    msg.step_pulse_us = u32::MAX;
    msg.step_idle_delay_ms = u32::MAX;
    msg.step_invert_mask = u32::MAX;
    msg.dir_invert_mask = u32::MAX;
    msg.status_report_mask = u32::MAX;
    msg.junction_deviation_mm = 1.0;
    msg.arc_tolerance_mm = 1.0;
    msg.soft_limits_enable = true;
    msg.hard_limits_enable = true;
    msg.homing_enable = true;
    msg.homing_dir_invert_mask = u32::MAX;
    msg.homing_feed_mm_min = 1.0;
    msg.homing_seek_mm_min = 1.0;
    msg.homing_debounce_ms = u32::MAX;
    msg.homing_pulloff_mm = 1.0;
    msg.spindle_rpm_max = 1.0;
    msg.spindle_rpm_min = 1.0;
    msg.steps_per_mm_x = 1.0;
    msg.steps_per_mm_y = 1.0;
    msg.steps_per_mm_z = 1.0;
    msg.max_rate_mm_min_x = 1.0;
    msg.max_rate_mm_min_y = 1.0;
    msg.max_rate_mm_min_z = 1.0;
    msg.accel_mm_s2_x = 1.0;
    msg.accel_mm_s2_y = 1.0;
    msg.accel_mm_s2_z = 1.0;
    msg.max_travel_mm_x = 1.0;
    msg.max_travel_mm_y = 1.0;
    msg.max_travel_mm_z = 1.0;
    msg.run_current_ma_x = u32::MAX;
    msg.run_current_ma_y = u32::MAX;
    msg.run_current_ma_z = u32::MAX;
    msg.hold_current_ma_x = u32::MAX;
    msg.hold_current_ma_y = u32::MAX;
    msg.hold_current_ma_z = u32::MAX;
    msg.microsteps_x = u32::MAX;
    msg.microsteps_y = u32::MAX;
    msg.microsteps_z = u32::MAX;
    msg.tmc_ihold_delay = u32::MAX;
    msg.tmc_tpowerdown = u32::MAX;
    msg.tmc_tpwmthrs = u32::MAX;
    msg.tmc_send_delay = u32::MAX;
    msg.tmc_r_sense_ohms = 1.0;
    assert!(settings_size(&msg) <= SETTINGS_MAX_LEN, "encoded size {} exceeds SETTINGS_MAX_LEN", settings_size(&msg));
  }
}
