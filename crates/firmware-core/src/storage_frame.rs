//! Shared persistence-frame codec (DOC-04 framing primitives).
//!
//! Both the settings record ([`crate::settings::wire`]) and the coordinate record ([`crate::coords::wire`])
//! wrap their protobuf payload in the SAME versioned, CRC-checked storage frame; only the magic bytes, the
//! schema-version sentinel, and the protobuf type differ. This module owns the one copy of that framing so the
//! two records cannot drift: the CRC32, the push helpers, the [`CodecError`] type, the [`FRAME_OVERHEAD`], and a
//! generic [`frame`]/[`unframe`] (plus the chunked-receive [`frame_progress`]) that wrap/validate a
//! caller-supplied payload.
//!
//! Frame layout (little-endian): `MAGIC(4) | SCHEMA_VERSION(1) | payload_len(2) | protobuf payload | CRC32(4)`.
//! The magic distinguishes a galdr record from arbitrary flash bytes (and one record kind from another), the
//! version sentinel rejects a record written by an incompatible build, and the CRC32 catches corruption / torn
//! writes — so [`unframe`] fails cleanly (and the infallible loader falls back to defaults) rather than feeding
//! garbage to the planner. The byte layout here is the EXACT layout the two records used before this module was
//! extracted, so already-persisted flash records still decode unchanged.

/// Bytes of fixed framing overhead around the protobuf payload (magic 4 + version 1 + len 2 + CRC 4). Every
/// record's `FRAME_MAX_LEN` is its largest protobuf payload plus this overhead.
pub const FRAME_OVERHEAD: usize = 11;

/// The offset at which the protobuf payload begins (after magic 4 + version 1 + len 2).
const HEADER_LEN: usize = 7;

/// Failure encoding or decoding a storage frame. Shared by both record codecs so a single error set covers the
/// settings and coordinate frames.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CodecError {
  /// The destination buffer could not hold the encoded frame.
  BufferFull,
  /// The frame was shorter than the fixed overhead, or its declared payload length did not fit.
  BadLength,
  /// The leading magic did not match the expected galdr record.
  BadMagic,
  /// The schema version did not match the record's build version.
  BadVersion,
  /// The trailing CRC32 did not match the computed checksum (corruption / torn write).
  BadCrc,
  /// The protobuf payload was malformed.
  BadPayload,
}

/// Append one byte to `out`, mapping an out-of-capacity push to [`CodecError::BufferFull`].
pub fn push_byte<const N: usize>(out: &mut heapless::Vec<u8, N>, byte: u8) -> Result<(), CodecError> {
  out.push(byte).map_err(|_| CodecError::BufferFull)
}

/// Append `bytes` to `out`, mapping an out-of-capacity extend to [`CodecError::BufferFull`].
pub fn push_slice<const N: usize>(out: &mut heapless::Vec<u8, N>, bytes: &[u8]) -> Result<(), CodecError> {
  out.extend_from_slice(bytes).map_err(|_| CodecError::BufferFull)
}

/// CRC-32 (IEEE 802.3, reflected, poly 0xEDB88320) over `data`. Hand-rolled to keep firmware-core's dependency
/// set unchanged, mirroring the hand-rolled CRC8 in the TMC2209 codec. This is the single CRC32 both records use.
pub fn crc32(data: &[u8]) -> u32 {
  let mut crc: u32 = 0xFFFF_FFFF;
  for &byte in data {
    crc ^= byte as u32;
    for _ in 0..8 {
      let mask = (crc & 1).wrapping_neg();
      crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
    }
  }
  !crc
}

/// Write a complete storage frame into `out` (cleared first): the `magic`/`version`/`payload_len` header, then
/// the caller's protobuf payload (appended by `encode_payload`), then a CRC32 over everything preceding it. The
/// caller supplies `payload_len` (the exact serialized size) up front so the length field can be written ahead of
/// the payload, exactly as the prior per-record `encode` did. `encode_payload` MUST append precisely `payload_len`
/// bytes; the CRC is taken over the header plus whatever it appends, so a mismatch would surface as a `BadCrc` on
/// decode rather than silent corruption. Returns [`CodecError::BadLength`] if `payload_len` exceeds the 16-bit
/// length field, or [`CodecError::BufferFull`] if `out` cannot hold the frame.
pub fn frame<const N: usize, F>(
  magic: u32,
  version: u8,
  payload_len: usize,
  out: &mut heapless::Vec<u8, N>,
  encode_payload: F,
) -> Result<(), CodecError>
where
  F: FnOnce(&mut heapless::Vec<u8, N>) -> Result<(), CodecError>,
{
  out.clear();
  if payload_len > u16::MAX as usize {
    return Err(CodecError::BadLength);
  }
  push_slice(out, &magic.to_le_bytes())?;
  push_byte(out, version)?;
  push_slice(out, &(payload_len as u16).to_le_bytes())?;
  encode_payload(out)?;
  let crc = crc32(out.as_slice());
  push_slice(out, &crc.to_le_bytes())?;
  Ok(())
}

/// Validate a storage frame against the expected `magic`/`version` and return the protobuf payload slice for the
/// caller to decode. Verifies the magic, schema version, declared length, and trailing CRC32 before returning the
/// payload; any mismatch is a [`CodecError`] so the infallible loader falls back to defaults rather than trusting a
/// damaged record. The returned slice borrows `frame_bytes`.
pub fn unframe(magic: u32, version: u8, frame_bytes: &[u8]) -> Result<&[u8], CodecError> {
  if frame_bytes.len() < FRAME_OVERHEAD {
    return Err(CodecError::BadLength);
  }
  let found_magic = u32::from_le_bytes([frame_bytes[0], frame_bytes[1], frame_bytes[2], frame_bytes[3]]);
  if found_magic != magic {
    return Err(CodecError::BadMagic);
  }
  if frame_bytes[4] != version {
    return Err(CodecError::BadVersion);
  }
  let payload_len = u16::from_le_bytes([frame_bytes[5], frame_bytes[6]]) as usize;
  let payload_end = HEADER_LEN + payload_len;
  if frame_bytes.len() != payload_end + 4 {
    return Err(CodecError::BadLength);
  }
  let expected_crc = u32::from_le_bytes([
    frame_bytes[payload_end],
    frame_bytes[payload_end + 1],
    frame_bytes[payload_end + 2],
    frame_bytes[payload_end + 3],
  ]);
  if crc32(&frame_bytes[..payload_end]) != expected_crc {
    return Err(CodecError::BadCrc);
  }
  Ok(&frame_bytes[HEADER_LEN..payload_end])
}

/// How much of a storage frame a partial byte buffer represents, for a chunked host-sync write path (which
/// accumulates a frame across several lines).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameProgress {
  /// Fewer than the fixed-header bytes have arrived; the total length is not yet known.
  NeedMore,
  /// The header is present but malformed (bad magic/version, or a declared length that cannot be valid).
  BadHeader,
  /// The header is valid; the complete frame is exactly this many bytes long.
  Total(usize),
}

/// Inspect the leading bytes of an accumulating frame against the expected `magic`/`version`: report the full
/// frame length once enough header is present and valid (and within `frame_max_len`), so a chunked receiver knows
/// when it has the whole record. Does NOT verify the CRC or payload — that is [`unframe`]'s job once the full frame
/// is assembled.
pub fn frame_progress(magic: u32, version: u8, frame_max_len: usize, buf: &[u8]) -> FrameProgress {
  if buf.len() < HEADER_LEN {
    return FrameProgress::NeedMore;
  }
  let found_magic = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
  if found_magic != magic || buf[4] != version {
    return FrameProgress::BadHeader;
  }
  let payload_len = u16::from_le_bytes([buf[5], buf[6]]) as usize;
  let total = HEADER_LEN + payload_len + 4;
  if total > frame_max_len {
    return FrameProgress::BadHeader;
  }
  FrameProgress::Total(total)
}

#[cfg(test)]
mod tests {
  use super::*;

  /// A round-trip through `frame`/`unframe` with a known magic/version returns the exact payload bytes, and the
  /// emitted frame is the canonical `MAGIC | VERSION | len | payload | CRC32` layout (proving the byte format is
  /// stable for both record codecs that depend on it).
  #[test]
  fn frame_unframe_round_trips_and_layout_is_canonical() {
    const MAGIC: u32 = 0x1234_5678;
    const VERSION: u8 = 1;
    let payload = [0xAAu8, 0xBB, 0xCC];
    let mut out: heapless::Vec<u8, 64> = heapless::Vec::new();
    frame(MAGIC, VERSION, payload.len(), &mut out, |buf| push_slice(buf, &payload)).expect("frame");
    // Layout: magic(LE) | version | len(LE) | payload | crc(LE).
    assert_eq!(&out[0..4], &MAGIC.to_le_bytes());
    assert_eq!(out[4], VERSION);
    assert_eq!(&out[5..7], &(payload.len() as u16).to_le_bytes());
    assert_eq!(&out[7..10], &payload);
    let expected_crc = crc32(&out[..7 + payload.len()]);
    assert_eq!(&out[7 + payload.len()..], &expected_crc.to_le_bytes());
    assert_eq!(out.len(), FRAME_OVERHEAD + payload.len());
    // And it round-trips back to the same payload.
    assert_eq!(unframe(MAGIC, VERSION, &out), Ok(&payload[..]));
  }

  #[test]
  fn unframe_rejects_magic_version_crc_and_length() {
    const MAGIC: u32 = 0x4764_5331;
    const VERSION: u8 = 1;
    let payload = [1u8, 2, 3, 4];
    let mut out: heapless::Vec<u8, 64> = heapless::Vec::new();
    frame(MAGIC, VERSION, payload.len(), &mut out, |buf| push_slice(buf, &payload)).expect("frame");

    let mut bad = out.clone();
    bad[0] ^= 0xFF;
    assert_eq!(unframe(MAGIC, VERSION, &bad), Err(CodecError::BadMagic));

    let mut bad = out.clone();
    bad[4] = bad[4].wrapping_add(1);
    assert_eq!(unframe(MAGIC, VERSION, &bad), Err(CodecError::BadVersion));

    let mut bad = out.clone();
    let last = bad.len() - 1;
    bad[last] ^= 0xFF;
    assert_eq!(unframe(MAGIC, VERSION, &bad), Err(CodecError::BadCrc));

    // Truncated below the framing overhead.
    assert_eq!(unframe(MAGIC, VERSION, &out[..4]), Err(CodecError::BadLength));
  }

  #[test]
  fn frame_progress_tracks_partial_then_complete() {
    const MAGIC: u32 = 0x4764_4331;
    const VERSION: u8 = 1;
    const MAX: usize = 64;
    let payload = [9u8; 5];
    let mut out: heapless::Vec<u8, MAX> = heapless::Vec::new();
    frame(MAGIC, VERSION, payload.len(), &mut out, |buf| push_slice(buf, &payload)).expect("frame");

    // Too few bytes for a header → NeedMore.
    assert_eq!(frame_progress(MAGIC, VERSION, MAX, &out[..3]), FrameProgress::NeedMore);
    // A full header reveals the exact total length.
    assert_eq!(frame_progress(MAGIC, VERSION, MAX, &out[..7]), FrameProgress::Total(out.len()));
    // A bad magic in a full header → BadHeader.
    let mut bad = out.clone();
    bad[0] ^= 0xFF;
    assert_eq!(frame_progress(MAGIC, VERSION, MAX, &bad), FrameProgress::BadHeader);
  }
}
