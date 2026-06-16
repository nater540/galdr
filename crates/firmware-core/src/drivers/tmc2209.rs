//! TMC2209 single-wire UART register codec (DOC-03).
//!
//! Pure, allocation-free encoding and decoding of TMC2209 UART datagrams plus the CRC8-ATM check
//! byte. This module has no I/O: it turns `(node, reg, value)` tuples into byte arrays and parses
//! read-reply byte slices back into register values, so it is fully host-testable against the
//! datasheet's reference CRC algorithm. The actual half-duplex transport is provided on target by
//! the `TmcBus` implementation in the `firmware` binary.
//!
//! Datagram formats (TMC2209 datasheet section 5, "Single Wire UART"):
//! - Write access (8 bytes):  `[0x05, NODE, REG | 0x80, D3, D2, D1, D0, CRC]`.
//! - Read request (4 bytes):  `[0x05, NODE, REG, CRC]`.
//! - Read reply (8 bytes):    `[0x05, 0xFF, REG, D3, D2, D1, D0, CRC]`.
//!
//! All multi-byte data words are big-endian (D3 is the most significant byte). The CRC8-ATM uses
//! polynomial x⁸+x²+x+1 (0x07), init 0x00, processing each byte LSB-first over every byte except
//! the trailing CRC byte itself.
//!
//! The byte-level codec here is paired with two layers built on top of it: [`registers`] holds the
//! per-register field codecs and the RMS-current→current-scale math, and [`manager`] orchestrates the
//! startup register sequence, write verification, and runtime status polling over the [`TmcBus`] trait
//! ([`crate::hal_traits`]). Both of those layers are pure and host-tested; only the firmware binary's
//! UART1 wiring implements the transport.
//!
//! [`TmcBus`]: crate::hal_traits::TmcBus

pub mod manager;
pub mod registers;

/// Datagram sync nibble/byte that opens every TMC2209 UART frame (the low nibble 0x5 is the sync;
/// the upper bits are reserved/zero in practice, so the first byte reads as 0x05).
pub const SYNC: u8 = 0x05;

/// Master address that the TMC2209 places in the NODE field of a read-reply datagram. Replies are
/// always addressed to the master (0xFF), regardless of which node was queried.
pub const MASTER_ADDR: u8 = 0xFF;

/// Write-access flag OR'd into the register address byte of a write datagram (bit 7 set => write).
pub const WRITE_FLAG: u8 = 0x80;

/// Mask selecting the 7-bit register address from a register byte (bits 0..=6); bit 7 is the R/W flag.
pub const REG_ADDR_MASK: u8 = 0x7F;

/// Highest valid TMC2209 node address. Nodes are 0..=3, set per driver via the MS1/MS2 strap pins.
pub const MAX_NODE: u8 = 3;

/// Length in bytes of a write-access datagram.
pub const WRITE_DATAGRAM_LEN: usize = 8;

/// Length in bytes of a read-request datagram.
pub const READ_REQUEST_LEN: usize = 4;

/// Length in bytes of a read-reply datagram.
pub const READ_REPLY_LEN: usize = 8;

/// Errors produced when talking to a TMC2209 over the single-wire UART.
///
/// The first group are *decode* failures raised by [`decode_read_reply`] when a reply datagram is
/// malformed; they are pure-logic and host-tested. The last two are *transport* failures the
/// [`TmcBus`](crate::hal_traits::TmcBus) implementation raises on target when the underlying UART
/// cannot complete a datagram exchange — a driver that never answers (standalone VREF mode, broken
/// bus) yields [`Timeout`](TmcError::Timeout), and a UART peripheral error yields [`Io`](TmcError::Io).
/// They live on the same enum so the codec, the bus trait, and the manager all speak one error type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TmcError {
  /// The reply slice did not have exactly [`READ_REPLY_LEN`] bytes.
  BadLength,
  /// The leading sync byte was not [`SYNC`].
  BadSync,
  /// The node/address field of the reply was not the master address [`MASTER_ADDR`].
  BadAddress,
  /// The reply's register byte did not match the register that was requested.
  RegisterMismatch,
  /// The trailing CRC byte did not match the CRC computed over the reply contents.
  BadCrc,
  /// The expected reply (or write echo) did not arrive within the bus turn-around window. Raised by
  /// the on-target [`TmcBus`](crate::hal_traits::TmcBus); the host codec never produces it. A node that
  /// is absent or wired for standalone VREF operation simply never answers, so this is the normal
  /// "driver not present" signal the manager treats as flag-and-skip during init (DOC-03).
  Timeout,
  /// The underlying UART peripheral reported an error (framing/overflow/glitch) during the exchange.
  /// Raised by the on-target [`TmcBus`](crate::hal_traits::TmcBus) only.
  Io,
}

/// Compute the TMC2209 CRC8-ATM over `data` (every datagram byte except the trailing CRC byte).
///
/// Implements the datasheet's reference algorithm directly: polynomial 0x07, initial value 0x00,
/// each input byte consumed LSB-first. This is the exact transcription of the C reference in the
/// TMC2209 datasheet, so it matches values produced by real drivers on the wire.
pub fn crc8_atm(data: &[u8]) -> u8 {
  let mut crc: u8 = 0;
  for &byte in data {
    let mut current = byte;
    // Process all eight bits of the byte, least-significant bit first, per the datasheet reference.
    for _ in 0..8 {
      if ((crc >> 7) ^ (current & 0x01)) != 0 {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc <<= 1;
      }
      current >>= 1;
    }
  }
  crc
}

/// Encode a write-access datagram for `reg = val` on `node`, returning the 8-byte frame with its
/// CRC already appended. The register byte has its write flag (bit 7) set and the 32-bit value is
/// serialized big-endian (most-significant byte first). Only the low 7 bits of `reg` form the
/// address (bit 7 is forced by the write flag); `node` must be a valid address (0..=[`MAX_NODE`]).
pub fn encode_write(node: u8, reg: u8, val: u32) -> [u8; WRITE_DATAGRAM_LEN] {
  // A node outside 0..=3 produces a well-formed but undeliverable frame; catch that programmer error
  // in debug builds while still emitting correct address bits in release.
  debug_assert!(node <= MAX_NODE, "TMC2209 node address must be 0..=3");
  let mut frame = [
    SYNC,
    node,
    (reg & REG_ADDR_MASK) | WRITE_FLAG,
    (val >> 24) as u8,
    (val >> 16) as u8,
    (val >> 8) as u8,
    val as u8,
    0,
  ];
  frame[WRITE_DATAGRAM_LEN - 1] = crc8_atm(&frame[..WRITE_DATAGRAM_LEN - 1]);
  frame
}

/// Encode a read-request datagram for `reg` on `node`, returning the 4-byte frame with its CRC
/// already appended. The register byte is sent without the write flag set (only the low 7 address
/// bits are used); `node` must be a valid address (0..=[`MAX_NODE`]).
pub fn encode_read_request(node: u8, reg: u8) -> [u8; READ_REQUEST_LEN] {
  debug_assert!(node <= MAX_NODE, "TMC2209 node address must be 0..=3");
  let mut frame = [SYNC, node, reg & REG_ADDR_MASK, 0];
  frame[READ_REQUEST_LEN - 1] = crc8_atm(&frame[..READ_REQUEST_LEN - 1]);
  frame
}

/// Decode a TMC2209 read-reply datagram, validating framing and CRC and returning the 32-bit
/// register value on success. `expected_reg` is the register that was requested; the reply echoes
/// back its 7-bit address (without the write flag), so only those bits are compared. The reply must
/// be addressed to the master ([`MASTER_ADDR`]).
pub fn decode_read_reply(reply: &[u8], expected_reg: u8) -> Result<u32, TmcError> {
  if reply.len() != READ_REPLY_LEN {
    return Err(TmcError::BadLength);
  }
  if reply[0] != SYNC {
    return Err(TmcError::BadSync);
  }
  if reply[1] != MASTER_ADDR {
    return Err(TmcError::BadAddress);
  }
  if reply[2] != (expected_reg & REG_ADDR_MASK) {
    return Err(TmcError::RegisterMismatch);
  }
  let expected_crc = crc8_atm(&reply[..READ_REPLY_LEN - 1]);
  if reply[READ_REPLY_LEN - 1] != expected_crc {
    return Err(TmcError::BadCrc);
  }
  let val = (u32::from(reply[3]) << 24)
    | (u32::from(reply[4]) << 16)
    | (u32::from(reply[5]) << 8)
    | u32::from(reply[6]);
  Ok(val)
}

#[cfg(test)]
mod tests {
  use super::*;

  // Register addresses referenced by the datasheet-derived vectors below (DOC-03 register map).
  const REG_GCONF: u8 = 0x00;
  const REG_IOIN: u8 = 0x06;
  const REG_IHOLD_IRUN: u8 = 0x10;

  // CRC8-ATM reference vectors. Values are produced by the datasheet's reference algorithm and were
  // cross-checked against an independent implementation of that same algorithm.
  #[test]
  fn crc_read_request_ioin_node0() {
    assert_eq!(crc8_atm(&[SYNC, 0x00, REG_IOIN]), 0x6F);
  }

  #[test]
  fn crc_read_request_gconf_node0() {
    assert_eq!(crc8_atm(&[SYNC, 0x00, REG_GCONF]), 0x48);
  }

  #[test]
  fn crc_write_gconf_node0() {
    // Write GCONF = 0x000001C0 on node 0: payload bytes before CRC are [05 00 80 00 00 01 C0].
    assert_eq!(crc8_atm(&[SYNC, 0x00, REG_GCONF | WRITE_FLAG, 0x00, 0x00, 0x01, 0xC0]), 0xF6);
  }

  #[test]
  fn crc_write_ihold_irun_node0() {
    // Write IHOLD_IRUN = 0x00071703 on node 0: payload [05 00 90 00 07 17 03].
    assert_eq!(crc8_atm(&[SYNC, 0x00, REG_IHOLD_IRUN | WRITE_FLAG, 0x00, 0x07, 0x17, 0x03]), 0x3B);
  }

  #[test]
  fn crc_empty_input_is_init_value() {
    // CRC over no bytes is just the initial value, 0x00.
    assert_eq!(crc8_atm(&[]), 0x00);
  }

  #[test]
  fn encode_read_request_ioin_node0() {
    assert_eq!(encode_read_request(0x00, REG_IOIN), [SYNC, 0x00, REG_IOIN, 0x6F]);
  }

  #[test]
  fn encode_read_request_sets_node_field() {
    // Node 2 (Z axis) read of GCONF; the node byte must carry the address, register byte unflagged.
    // The CRC is hardcoded (not recomputed via crc8_atm) so the assertion is independent of encode.
    let frame = encode_read_request(0x02, REG_GCONF);
    assert_eq!(frame, [SYNC, 0x02, REG_GCONF, 0x13]);
  }

  #[test]
  fn encode_write_gconf_node0_full_frame() {
    let frame = encode_write(0x00, REG_GCONF, 0x000001C0);
    assert_eq!(frame, [SYNC, 0x00, 0x80, 0x00, 0x00, 0x01, 0xC0, 0xF6]);
  }

  #[test]
  fn encode_write_sets_write_flag_and_big_endian_value() {
    // Full hardcoded frame: write IHOLD_IRUN = 0x00071703 on node 1, trailing CRC 0xD7.
    let frame = encode_write(0x01, REG_IHOLD_IRUN, 0x00071703);
    assert_eq!(frame, [SYNC, 0x01, REG_IHOLD_IRUN | WRITE_FLAG, 0x00, 0x07, 0x17, 0x03, 0xD7]);
  }

  #[test]
  fn encode_write_masks_preflagged_register() {
    // A reg that already carries bit 7 must still address only its low 7 bits, not collide with the
    // write flag: REG_IHOLD_IRUN | WRITE_FLAG (0x90) must encode the same frame as REG_IHOLD_IRUN.
    let masked = encode_write(0x01, REG_IHOLD_IRUN, 0x00071703);
    let preflagged = encode_write(0x01, REG_IHOLD_IRUN | WRITE_FLAG, 0x00071703);
    assert_eq!(preflagged, masked);
  }

  #[test]
  fn decode_read_reply_roundtrip() {
    // Build a valid IOIN reply carrying VERSION 0x21 in the top byte and decode it back.
    let value = 0x2100_0000_u32;
    let mut reply = [SYNC, MASTER_ADDR, REG_IOIN, 0x21, 0x00, 0x00, 0x00, 0x00];
    reply[7] = crc8_atm(&reply[..7]);
    assert_eq!(decode_read_reply(&reply, REG_IOIN), Ok(value));
  }

  #[test]
  fn decode_read_reply_pins_big_endian_byte_order() {
    // Four distinct data bytes pin the D3..D0 mapping: a swapped shift or index would change the
    // decoded value. 0xDEADBEEF -> D3=0xDE, D2=0xAD, D1=0xBE, D0=0xEF; trailing CRC is 0x3A.
    let reply = [SYNC, MASTER_ADDR, REG_IOIN, 0xDE, 0xAD, 0xBE, 0xEF, 0x3A];
    assert_eq!(decode_read_reply(&reply, REG_IOIN), Ok(0xDEAD_BEEF));
  }

  #[test]
  fn decode_read_reply_accepts_preflagged_expected_reg() {
    // A caller passing a write-flagged expected_reg must not cause a spurious RegisterMismatch: the
    // reply echoes the unflagged address, and decode compares only the low 7 bits.
    let mut reply = [SYNC, MASTER_ADDR, REG_IOIN, 0x21, 0x00, 0x00, 0x00, 0x00];
    reply[7] = crc8_atm(&reply[..7]);
    assert_eq!(decode_read_reply(&reply, REG_IOIN | WRITE_FLAG), Ok(0x2100_0000));
  }

  #[test]
  fn decode_read_reply_rejects_bad_length() {
    let reply = [SYNC, MASTER_ADDR, REG_IOIN, 0x21, 0x00, 0x00, 0x00];
    assert_eq!(decode_read_reply(&reply, REG_IOIN), Err(TmcError::BadLength));
  }

  #[test]
  fn decode_read_reply_rejects_bad_sync() {
    let mut reply = [0x00, MASTER_ADDR, REG_IOIN, 0x21, 0x00, 0x00, 0x00, 0x00];
    reply[7] = crc8_atm(&reply[..7]);
    assert_eq!(decode_read_reply(&reply, REG_IOIN), Err(TmcError::BadSync));
  }

  #[test]
  fn decode_read_reply_rejects_bad_address() {
    // Reply addressed to a node instead of the master (0xFF) must be rejected.
    let mut reply = [SYNC, 0x00, REG_IOIN, 0x21, 0x00, 0x00, 0x00, 0x00];
    reply[7] = crc8_atm(&reply[..7]);
    assert_eq!(decode_read_reply(&reply, REG_IOIN), Err(TmcError::BadAddress));
  }

  #[test]
  fn decode_read_reply_rejects_register_mismatch() {
    let mut reply = [SYNC, MASTER_ADDR, REG_GCONF, 0x21, 0x00, 0x00, 0x00, 0x00];
    reply[7] = crc8_atm(&reply[..7]);
    assert_eq!(decode_read_reply(&reply, REG_IOIN), Err(TmcError::RegisterMismatch));
  }

  #[test]
  fn decode_read_reply_rejects_bad_crc() {
    // Valid framing but a corrupted CRC byte must fail the integrity check.
    let mut reply = [SYNC, MASTER_ADDR, REG_IOIN, 0x21, 0x00, 0x00, 0x00, 0x00];
    reply[7] = crc8_atm(&reply[..7]).wrapping_add(1);
    assert_eq!(decode_read_reply(&reply, REG_IOIN), Err(TmcError::BadCrc));
  }
}
