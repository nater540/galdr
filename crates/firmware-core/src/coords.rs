//! Coordinate systems & work offsets (Phase B, `docs/tlo-offsets.md`).
//!
//! This is the host-tested heart of the coordinate model that turns the work coordinates a GCode
//! program speaks in into the machine coordinates the planner ([`crate::planner`]) consumes. It owns the
//! full grbl/grblHAL offset set and the single source of truth for the Work Coordinate Offset (WCO):
//!
//! - Six work-coordinate-system offsets for **G54–G59** (`offsets[0..6]`), and the currently active WCS
//!   index (`active`, 0..=5).
//! - The **G92** dynamic coordinate offset.
//! - The **tool-length offset** (TLO): grbl's dynamic TLO (`G43.1`) applies to one configured linear
//!   axis (Z) and is modelled as a Z scalar folded into the Z component of the WCO. It is reported as
//!   `[TLO:z]` in the `$#` parameter dump.
//! - The two predefined positions **G28** / **G30**, stored in MACHINE coordinates.
//!
//! ## The single relationship the whole model enforces
//! ```text
//! WCO  = G54..59[active] + G92 + TLO      (TLO contributes only to Z)
//! MPos = WPos + WCO   ⇔   WPos = MPos − WCO
//! ```
//! `docs/tlo-offsets.md` is explicit that the user's `WPos_Z = MPos_Z − WCO_Z − TLO_Z` form
//! double-counts the TLO: the TLO is *inside* the WCO, so a single subtraction is correct. [`wco`]
//! computes the one combined offset; [`work_to_machine`]/[`machine_to_work`] apply it.
//!
//! ## Volatility (grbl semantics, mirrored here)
//! G54–G59 and G28/G30 are PERSISTENT (the firmware bin writes them to a dedicated NVS record). G92 and
//! the dynamic TLO are SESSION-only: a soft reset / power cycle clears them to identity. This module does
//! not itself persist anything — [`wire`] frames only the persistent subset, and the firmware wiring
//! restores G92/TLO to identity on reset by calling [`CoordinateSystems::clear_volatile`].
//!
//! ## Allocation & purity
//! `#![no_std]`, allocation-free, `Copy` (it is just fixed `f32` arrays), so it rides the firmware bin's
//! shared `Cell` snapshot pattern exactly like [`crate::protocol::ControlState`]. Every mutator
//! finite-guards its inputs so a NaN/inf word from a malformed line can never poison the offset set.

use crate::hal_traits::{RecordStore, StoreError};
use crate::planner::AXES;

/// The number of work coordinate systems grbl exposes as G54–G59. Each is one [`AXES`]-length offset in
/// machine mm; the active one (plus G92 and the TLO) sums into the live WCO.
pub const WCS_COUNT: usize = 6;

/// The number of predefined positions (G28 and G30), stored in MACHINE coordinates.
pub const PREDEFINED_COUNT: usize = 2;

/// The configured tool-length-offset axis: Z (index 2). grbl applies the dynamic `G43.1` TLO to a single
/// linear axis — the machine default is Z — so the TLO scalar folds into the Z component of the WCO only.
pub const TLO_AXIS: usize = 2;

/// The full coordinate-system / offset state: the six G54–G59 work offsets, the active WCS index, the G92
/// offset, the dynamic tool-length offset (a Z scalar), and the G28/G30 predefined positions. `Copy` so the
/// firmware bin can hold it behind a synchronous `Cell` and snapshot it cheaply for the status reporter.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CoordinateSystems {
  /// G54–G59 work-coordinate-system offsets in machine mm, indexed by WCS (0 = G54 … 5 = G59).
  offsets: [[f32; AXES]; WCS_COUNT],
  /// The currently selected WCS index (0 = G54 … 5 = G59). G54 is the power-on default.
  active: usize,
  /// The G92 dynamic coordinate offset in mm, summed into every axis of the WCO. Session-only.
  g92: [f32; AXES],
  /// The dynamic tool-length offset (`G43.1`) on the configured axis ([`TLO_AXIS`]), in mm. Session-only.
  /// Modelled as a single scalar (grbl's single-axis dynamic TLO) folded into the Z component of the WCO.
  tlo: f32,
  /// The G28/G30 predefined positions in MACHINE mm (index 0 = G28, 1 = G30). Persistent.
  predefined: [[f32; AXES]; PREDEFINED_COUNT],
}

impl Default for CoordinateSystems {
  /// The power-on default: all offsets zero, G54 active, no G92/TLO, predefined positions at the origin.
  fn default() -> Self {
    Self::new()
  }
}

impl CoordinateSystems {
  /// A fresh coordinate model at power-on defaults (identical to [`Default`]). `const` so it can seed a
  /// `static` cell in the firmware bin's shared-state plumbing without a lazy `Option`.
  pub const fn new() -> Self {
    CoordinateSystems {
      offsets: [[0.0; AXES]; WCS_COUNT],
      active: 0,
      g92: [0.0; AXES],
      tlo: 0.0,
      predefined: [[0.0; AXES]; PREDEFINED_COUNT],
    }
  }

  /// The active WCS index (0 = G54 … 5 = G59).
  pub fn active_wcs(&self) -> usize {
    self.active
  }

  /// The stored offset of WCS `index` (0 = G54 … 5 = G59) in machine mm, or `None` for an out-of-range
  /// index. Used by the `$#` parameter dump.
  pub fn wcs_offset(&self, index: usize) -> Option<[f32; AXES]> {
    self.offsets.get(index).copied()
  }

  /// The active G92 offset in mm. Used by the `$#` dump (`[G92:...]`).
  pub fn g92_offset(&self) -> [f32; AXES] {
    self.g92
  }

  /// The dynamic tool-length offset scalar in mm (on the configured Z axis). Used by `$#` (`[TLO:z]`).
  pub fn tlo(&self) -> f32 {
    self.tlo
  }

  /// The stored predefined position (0 = G28, 1 = G30) in MACHINE mm, or `None` for an out-of-range index.
  pub fn predefined(&self, index: usize) -> Option<[f32; AXES]> {
    self.predefined.get(index).copied()
  }

  /// The live Work Coordinate Offset (WCO) in machine mm: `WCO = G54..59[active] + G92 + TLO`, with the TLO
  /// contributing ONLY to the configured Z axis ([`TLO_AXIS`]). This is the single combined offset the wire
  /// `WCO:` element reports and that [`work_to_machine`]/[`machine_to_work`] apply — the TLO is folded in
  /// here, never subtracted a second time (see the module docs / `docs/tlo-offsets.md`).
  pub fn wco(&self) -> [f32; AXES] {
    let wcs = self.offsets[self.active];
    let mut wco = [0.0f32; AXES];
    for axis in 0..AXES {
      wco[axis] = wcs[axis] + self.g92[axis];
    }
    wco[TLO_AXIS] += self.tlo;
    wco
  }

  /// Convert a work-coordinate position (mm) to machine coordinates: `MPos = WPos + WCO`.
  pub fn work_to_machine(&self, work: [f32; AXES]) -> [f32; AXES] {
    let wco = self.wco();
    let mut machine = [0.0f32; AXES];
    for axis in 0..AXES {
      machine[axis] = work[axis] + wco[axis];
    }
    machine
  }

  /// Convert a machine-coordinate position (mm) to work coordinates: `WPos = MPos − WCO`.
  pub fn machine_to_work(&self, machine: [f32; AXES]) -> [f32; AXES] {
    let wco = self.wco();
    let mut work = [0.0f32; AXES];
    for axis in 0..AXES {
      work[axis] = machine[axis] - wco[axis];
    }
    work
  }

  /// Select the active work coordinate system (G54–G59). `index` is 0 = G54 … 5 = G59; an out-of-range
  /// index is ignored (the active WCS is unchanged), so a malformed selection never indexes out of bounds.
  pub fn select_wcs(&mut self, index: usize) {
    if index < WCS_COUNT {
      self.active = index;
    }
  }

  /// Set the offset of WCS `index` directly to `offset` (mm) — the `G10 L2 P<n>` form, where each present
  /// axis word is the literal new offset value. Only the axes in `present` are written; an unmentioned axis
  /// keeps its current offset (grbl's per-axis G10 semantics). A non-finite value is rejected per axis so a
  /// bad word cannot poison the offset. An out-of-range `index` is ignored.
  pub fn set_wcs_offset(&mut self, index: usize, offset: [f32; AXES], present: [bool; AXES]) {
    let Some(slot) = self.offsets.get_mut(index) else {
      return;
    };
    for axis in 0..AXES {
      if present[axis] && offset[axis].is_finite() {
        slot[axis] = offset[axis];
      }
    }
  }

  /// Set the offset of WCS `index` so the current machine position `machine` maps to the given work
  /// position `work` — the `G10 L20 P<n>` form. For each present axis the new offset is
  /// `machine − work − g92 − tlo` (the TLO term only on [`TLO_AXIS`]). The G92 and TLO subtractions are
  /// essential: [`wco`] re-adds G92 (and the TLO on Z), and [`machine_to_work`] subtracts the FULL WCO, so a
  /// naive `machine − work` would leave the active G92/TLO double-counted and the current position would NOT
  /// read back `work`. Subtracting them here makes `machine_to_work(machine)[axis] == work[axis]` hold even
  /// with a live G92 and/or dynamic TLO. (grbl's plain `G92`-set deliberately does NOT re-subtract the TLO;
  /// see [`set_g92_to_position`] — only this WCS-set form needs the correction.) Only present, finite axes are
  /// written; an out-of-range `index` is ignored.
  pub fn set_wcs_offset_to_position(
    &mut self,
    index: usize,
    machine: [f32; AXES],
    work: [f32; AXES],
    present: [bool; AXES],
  ) {
    let g92 = self.g92;
    let tlo = self.tlo;
    let Some(slot) = self.offsets.get_mut(index) else {
      return;
    };
    for axis in 0..AXES {
      if present[axis] && machine[axis].is_finite() && work[axis].is_finite() {
        let tlo_term = if axis == TLO_AXIS { tlo } else { 0.0 };
        slot[axis] = machine[axis] - work[axis] - g92[axis] - tlo_term;
      }
    }
  }

  /// Apply a G92 offset so the current machine position `machine` reads as the work position `work` on each
  /// present axis. grbl's G92 sets, per present axis, `g92 = machine − (active_wcs + work)` — i.e. it is the
  /// EXTRA offset (on top of the active WCS) that makes the commanded value read back, so the offset is
  /// independent of which WCS is active. Unmentioned axes keep their current G92; non-finite words are
  /// rejected per axis.
  pub fn set_g92_to_position(&mut self, machine: [f32; AXES], work: [f32; AXES], present: [bool; AXES]) {
    let wcs = self.offsets[self.active];
    for axis in 0..AXES {
      if present[axis] && machine[axis].is_finite() && work[axis].is_finite() {
        self.g92[axis] = machine[axis] - wcs[axis] - work[axis];
      }
    }
  }

  /// Clear the G92 offset (`G92.1`): reset it to identity on every axis. Used by `G92.1` and on soft reset.
  pub fn clear_g92(&mut self) {
    self.g92 = [0.0; AXES];
  }

  /// Apply a dynamic tool-length offset from a Z word (`G43.1 Z<value>`): store `value` (mm) as the TLO on
  /// the configured axis. A non-finite value is ignored. grbl errors on a non-Z axis word for `G43.1`; the
  /// parser enforces that, so this takes only the resolved Z value.
  pub fn apply_tlo(&mut self, value: f32) {
    if value.is_finite() {
      self.tlo = value;
    }
  }

  /// Cancel the tool-length offset (`G49`): reset the TLO to zero. Also applied on soft reset.
  pub fn cancel_tlo(&mut self) {
    self.tlo = 0.0;
  }

  /// Store the current machine position `machine` as the predefined position `index` (0 = G28 via `G28.1`,
  /// 1 = G30 via `G30.1`). Non-finite axes are rejected; an out-of-range `index` is ignored. The stored
  /// value is always in MACHINE coordinates, as grbl requires.
  pub fn store_predefined(&mut self, index: usize, machine: [f32; AXES]) {
    let Some(slot) = self.predefined.get_mut(index) else {
      return;
    };
    for axis in 0..AXES {
      if machine[axis].is_finite() {
        slot[axis] = machine[axis];
      }
    }
  }

  /// Clear the SESSION-only offsets (G92 and the dynamic TLO) to identity, leaving the persistent G54–G59
  /// and G28/G30 untouched. Called by the firmware bin on a soft reset / power cycle so the volatile offsets
  /// match grbl (which drops G92 and `G43.1` on reset) while the persistent coordinate data survives.
  pub fn clear_volatile(&mut self) {
    self.clear_g92();
    self.cancel_tlo();
  }

  /// Replace ONLY the persistent subset (G54–G59 and G28/G30) from `persistent`, keeping the live active-WCS
  /// selection and the volatile G92/TLO. Used at boot when the persisted coordinate record is loaded into a
  /// freshly-defaulted model (the loaded record carries no live session state). The active WCS is taken from
  /// `persistent` too, since grbl persists the selected coordinate system.
  pub fn load_persistent(&mut self, persistent: &CoordinatePersistent) {
    self.offsets = persistent.offsets;
    self.predefined = persistent.predefined;
    if persistent.active < WCS_COUNT {
      self.active = persistent.active;
    }
  }

  /// Project the PERSISTENT subset (G54–G59, the active WCS, and G28/G30) for the NVS record. The volatile
  /// G92/TLO are deliberately excluded — grbl never persists them.
  pub fn persistent(&self) -> CoordinatePersistent {
    CoordinatePersistent {
      offsets: self.offsets,
      active: self.active,
      predefined: self.predefined,
    }
  }

  /// Clear ALL coordinate data to the power-on default (`$RST=#`): zero every G54–G59 offset and G28/G30
  /// position, reset G92/TLO to identity, and reselect G54. The firmware bin persists the result.
  pub fn clear_all(&mut self) {
    *self = CoordinateSystems::default();
  }
}

/// The PERSISTENT subset of the coordinate model the firmware bin writes to NVS: the six G54–G59 offsets,
/// the active WCS, and the G28/G30 predefined positions. The session-only G92/TLO are excluded (grbl never
/// persists them). This plain struct is what [`wire`] frames; the live [`CoordinateSystems`] projects it via
/// [`CoordinateSystems::persistent`] and restores it via [`CoordinateSystems::load_persistent`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CoordinatePersistent {
  /// G54–G59 offsets in machine mm.
  pub offsets: [[f32; AXES]; WCS_COUNT],
  /// The active WCS index (0 = G54 … 5 = G59).
  pub active: usize,
  /// G28/G30 predefined positions in machine mm.
  pub predefined: [[f32; AXES]; PREDEFINED_COUNT],
}

impl Default for CoordinatePersistent {
  /// All offsets/positions zero with G54 active — the first-boot default when nothing is persisted yet.
  fn default() -> Self {
    CoordinatePersistent {
      offsets: [[0.0; AXES]; WCS_COUNT],
      active: 0,
      predefined: [[0.0; AXES]; PREDEFINED_COUNT],
    }
  }
}

/// Load the persisted coordinate record, falling back to [`CoordinatePersistent::default`] (all zero, G54
/// active) on absence OR any decode failure. INFALLIBLE, like the settings loader: a corrupt, truncated,
/// version-skewed, or missing record can never wedge boot — it silently yields defaults.
pub async fn load_or_default<S: RecordStore>(store: &mut S) -> CoordinatePersistent {
  let mut buf = [0u8; wire::FRAME_MAX_LEN];
  match store.load(&mut buf).await {
    Ok(len) => wire::decode(&buf[..len]).unwrap_or_default(),
    Err(_) => CoordinatePersistent::default(),
  }
}

/// Encode `persistent` into a storage frame and persist it via `store`, replacing any prior record. Surfaces
/// encode/store failures to the caller (a failed persist still leaves the in-RAM value applied — the firmware
/// logs the failure, matching the settings persist policy).
pub async fn store_coordinates<S: RecordStore>(
  store: &mut S,
  persistent: &CoordinatePersistent,
) -> Result<(), StoreError> {
  let mut frame: heapless::Vec<u8, { wire::FRAME_MAX_LEN }> = heapless::Vec::new();
  wire::encode(persistent, &mut frame).map_err(|_| StoreError::TooLarge)?;
  store.save(&frame).await
}

/// Protobuf encode/decode of the persistent coordinate record wrapped in a versioned, CRC-checked storage
/// frame — the exact framing [`crate::settings::wire`] uses, with its own distinct magic.
///
/// Frame layout (little-endian): `MAGIC(4) | SCHEMA_VERSION(1) | payload_len(2) | protobuf payload | CRC32(4)`.
/// The magic distinguishes a galdr coordinate record from arbitrary flash bytes (and from a settings record),
/// the version sentinel rejects an incompatible build's record, and the CRC32 catches corruption / torn writes
/// — so [`decode`] fails cleanly (and the loader falls back to defaults) rather than feeding garbage offsets to
/// the planner.
pub mod wire {
  use super::{CoordinatePersistent, AXES, PREDEFINED_COUNT, WCS_COUNT};
  use crate::planner::A_AXIS;
  use crate::storage_frame::{self, FRAME_OVERHEAD};

  // Re-export the shared codec error so the public `coords::wire::CodecError` surface (named by `PbReceiver`'s
  // sibling settings path and the coordinate loader) is unchanged after the framing moved into `storage_frame`.
  pub use crate::storage_frame::CodecError;

  /// Frame magic, ASCII "GdC1" — identifies a galdr coordinate record (distinct from the "GdS1" settings one).
  const MAGIC: u32 = 0x4764_4331;
  /// On-flash schema version, held at 1 during early development (same additive-compatibility policy as the
  /// settings frame: bumped only on a genuinely incompatible layout change, not when adding proto fields).
  const SCHEMA_VERSION: u8 = 1;

  /// Maximum framed-record length, sized to the largest protobuf payload plus the shared framing overhead.
  pub const FRAME_MAX_LEN: usize = galdr_proto::COORDINATES_MAX_LEN + FRAME_OVERHEAD;

  /// Encode `persistent` into `out` as a complete storage frame (the buffer is cleared first). Builds the
  /// protobuf payload and hands it, with the coordinate magic/version, to the shared framer.
  pub fn encode<const N: usize>(
    persistent: &CoordinatePersistent,
    out: &mut heapless::Vec<u8, N>,
  ) -> Result<(), CodecError> {
    let proto = to_proto(persistent);
    let payload_len = galdr_proto::coordinates_size(&proto);
    storage_frame::frame(MAGIC, SCHEMA_VERSION, payload_len, out, |buf| {
      galdr_proto::encode_coordinates_into(&proto, buf).map_err(|_| CodecError::BufferFull)
    })
  }

  /// Decode and validate a storage frame, returning the [`CoordinatePersistent`] record. The shared [`unframe`]
  /// verifies the magic, schema version, declared length, and CRC32 and yields the protobuf payload; any
  /// mismatch is an error so the infallible loader falls back to defaults rather than trusting a damaged record.
  pub fn decode(frame: &[u8]) -> Result<CoordinatePersistent, CodecError> {
    let payload = storage_frame::unframe(MAGIC, SCHEMA_VERSION, frame)?;
    let proto = galdr_proto::decode_coordinates(payload).map_err(|_| CodecError::BadPayload)?;
    Ok(from_proto(&proto))
  }

  /// Project the persistent record onto the protobuf DTO. Built by mutating a `default()` so a field added to
  /// the generated message is zero-initialized rather than a hard compile break here.
  #[allow(clippy::field_reassign_with_default)]
  fn to_proto(persistent: &CoordinatePersistent) -> galdr_proto::Coordinates {
    let mut proto = galdr_proto::Coordinates::default();
    proto.active_wcs = persistent.active as u32;
    let o = &persistent.offsets;
    proto.g54_x = o[0][0]; proto.g54_y = o[0][1]; proto.g54_z = o[0][2]; proto.g54_a = o[0][A_AXIS];
    proto.g55_x = o[1][0]; proto.g55_y = o[1][1]; proto.g55_z = o[1][2]; proto.g55_a = o[1][A_AXIS];
    proto.g56_x = o[2][0]; proto.g56_y = o[2][1]; proto.g56_z = o[2][2]; proto.g56_a = o[2][A_AXIS];
    proto.g57_x = o[3][0]; proto.g57_y = o[3][1]; proto.g57_z = o[3][2]; proto.g57_a = o[3][A_AXIS];
    proto.g58_x = o[4][0]; proto.g58_y = o[4][1]; proto.g58_z = o[4][2]; proto.g58_a = o[4][A_AXIS];
    proto.g59_x = o[5][0]; proto.g59_y = o[5][1]; proto.g59_z = o[5][2]; proto.g59_a = o[5][A_AXIS];
    let p = &persistent.predefined;
    proto.g28_x = p[0][0]; proto.g28_y = p[0][1]; proto.g28_z = p[0][2]; proto.g28_a = p[0][A_AXIS];
    proto.g30_x = p[1][0]; proto.g30_y = p[1][1]; proto.g30_z = p[1][2]; proto.g30_a = p[1][A_AXIS];
    proto
  }

  /// Build the persistent record from the protobuf DTO. The active-WCS field is clamped into range so a wire
  /// value past G59 cannot select an out-of-bounds system; it falls back to G54 (0), the safe default.
  fn from_proto(proto: &galdr_proto::Coordinates) -> CoordinatePersistent {
    let active = if (proto.active_wcs as usize) < WCS_COUNT { proto.active_wcs as usize } else { 0 };
    CoordinatePersistent {
      offsets: [
        [proto.g54_x, proto.g54_y, proto.g54_z, proto.g54_a],
        [proto.g55_x, proto.g55_y, proto.g55_z, proto.g55_a],
        [proto.g56_x, proto.g56_y, proto.g56_z, proto.g56_a],
        [proto.g57_x, proto.g57_y, proto.g57_z, proto.g57_a],
        [proto.g58_x, proto.g58_y, proto.g58_z, proto.g58_a],
        [proto.g59_x, proto.g59_y, proto.g59_z, proto.g59_a],
      ],
      active,
      predefined: [
        [proto.g28_x, proto.g28_y, proto.g28_z, proto.g28_a],
        [proto.g30_x, proto.g30_y, proto.g30_z, proto.g30_a],
      ],
    }
  }

  // The X/Y/Z/A quad conversions assume exactly four axes; fail the build loudly if that ever changes.
  const _: () = assert!(AXES == 4, "coordinate wire conversions assume AXES == 4");
  // The G54-G59 / G28-G30 field maps above assume the canonical counts; fail loudly if they change.
  const _: () = assert!(WCS_COUNT == 6, "coordinate wire conversions assume 6 work coordinate systems");
  const _: () = assert!(PREDEFINED_COUNT == 2, "coordinate wire conversions assume 2 predefined positions");
}

#[cfg(test)]
mod tests {
  use super::*;

  fn approx(a: [f32; AXES], b: [f32; AXES]) {
    for axis in 0..AXES {
      assert!((a[axis] - b[axis]).abs() < 1e-4, "axis {axis}: {} != {}", a[axis], b[axis]);
    }
  }

  const ALL: [bool; AXES] = [true; AXES];

  // ---- WCO math: the WCO = G54 + G92 + TLO relationship -----------------------------------------

  #[test]
  fn default_wco_is_zero_and_g54_active() {
    let cs = CoordinateSystems::new();
    assert_eq!(cs.active_wcs(), 0);
    approx(cs.wco(), [0.0, 0.0, 0.0, 0.0]);
  }

  #[test]
  fn wco_sums_active_wcs_g92_and_tlo() {
    let mut cs = CoordinateSystems::new();
    // G54 = (10, 20, 5); G92 = (1, 2, 3); TLO = 0.5 (Z only).
    cs.set_wcs_offset(0, [10.0, 20.0, 5.0, 0.0], ALL);
    cs.g92 = [1.0, 2.0, 3.0, 0.0];
    cs.tlo = 0.5;
    // X = 10 + 1 = 11; Y = 20 + 2 = 22; Z = 5 + 3 + 0.5 = 8.5 (TLO folds into Z only).
    approx(cs.wco(), [11.0, 22.0, 8.5, 0.0]);
  }

  #[test]
  fn active_wcs_selects_which_offset_contributes() {
    let mut cs = CoordinateSystems::new();
    cs.set_wcs_offset(0, [1.0, 0.0, 0.0, 0.0], ALL); // G54
    cs.set_wcs_offset(2, [9.0, 0.0, 0.0, 0.0], ALL); // G56
    cs.select_wcs(2);
    assert_eq!(cs.active_wcs(), 2);
    approx(cs.wco(), [9.0, 0.0, 0.0, 0.0]);
  }

  #[test]
  fn select_wcs_out_of_range_is_ignored() {
    let mut cs = CoordinateSystems::new();
    cs.select_wcs(99);
    assert_eq!(cs.active_wcs(), 0);
  }

  // ---- work <-> machine round-trips -------------------------------------------------------------

  #[test]
  fn work_to_machine_adds_wco() {
    let mut cs = CoordinateSystems::new();
    cs.set_wcs_offset(0, [10.0, 20.0, 5.0, 0.0], ALL);
    approx(cs.work_to_machine([0.0, 0.0, 0.0, 0.0]), [10.0, 20.0, 5.0, 0.0]);
    approx(cs.work_to_machine([1.0, 1.0, 1.0, 0.0]), [11.0, 21.0, 6.0, 0.0]);
  }

  #[test]
  fn machine_to_work_subtracts_wco_and_round_trips() {
    let mut cs = CoordinateSystems::new();
    cs.set_wcs_offset(0, [10.0, 20.0, 5.0, 0.0], ALL);
    cs.g92 = [1.0, 2.0, 3.0, 0.0];
    cs.tlo = 0.25;
    let work = [3.0, -4.0, 7.0, 0.0];
    let machine = cs.work_to_machine(work);
    approx(cs.machine_to_work(machine), work);
  }

  // ---- G10 L2 vs L20 ----------------------------------------------------------------------------

  #[test]
  fn g10_l2_sets_offset_literally_per_present_axis() {
    let mut cs = CoordinateSystems::new();
    cs.set_wcs_offset(0, [5.0, 5.0, 5.0, 0.0], ALL);
    // Only X present: X overwritten to 12, Y/Z keep 5.
    cs.set_wcs_offset(0, [12.0, 0.0, 0.0, 0.0], [true, false, false, false]);
    approx(cs.wcs_offset(0).unwrap(), [12.0, 5.0, 5.0, 0.0]);
  }

  #[test]
  fn g10_l20_sets_offset_so_position_reads_work_value() {
    let mut cs = CoordinateSystems::new();
    // Machine is at (100, 50, 10); declare the work position there to be (0, 0, 0).
    cs.set_wcs_offset_to_position(0, [100.0, 50.0, 10.0, 0.0], [0.0, 0.0, 0.0, 0.0], ALL);
    approx(cs.wcs_offset(0).unwrap(), [100.0, 50.0, 10.0, 0.0]);
    // Now machine (100,50,10) must map to work (0,0,0).
    approx(cs.machine_to_work([100.0, 50.0, 10.0, 0.0]), [0.0, 0.0, 0.0, 0.0]);
  }

  #[test]
  fn g10_l20_with_nonzero_work_target() {
    let mut cs = CoordinateSystems::new();
    // At machine X=100 declare work X=10 → offset = 100 − 10 = 90.
    cs.set_wcs_offset_to_position(0, [100.0, 0.0, 0.0, 0.0], [10.0, 0.0, 0.0, 0.0], [true, false, false, false]);
    assert!((cs.wcs_offset(0).unwrap()[0] - 90.0).abs() < 1e-4);
    approx(cs.machine_to_work([100.0, 0.0, 0.0, 0.0]), [10.0, 0.0, 0.0, 0.0]);
  }

  #[test]
  fn g10_l20_accounts_for_active_g92() {
    let mut cs = CoordinateSystems::new();
    // With a live G92 (here g92_x = 15), `G10 L20` must still make the current machine position read the work
    // target: the WCS offset has to be machine − work − g92 so `machine_to_work` (which re-adds G92 via the WCO)
    // lands exactly on the target. The naive `machine − work` would yield −15 here (G92 double-counted).
    cs.g92 = [15.0, 0.0, 0.0, 0.0];
    cs.set_wcs_offset_to_position(0, [100.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [true, false, false, false]);
    approx(cs.machine_to_work([100.0, 0.0, 0.0, 0.0]), [0.0, 0.0, 0.0, 0.0]);
  }

  #[test]
  fn g10_l20_accounts_for_active_tlo_on_its_axis() {
    let mut cs = CoordinateSystems::new();
    // A live dynamic TLO folds into the Z component of the WCO, so `G10 L20 Z<target>` must subtract it too on the
    // TLO axis (Z) — otherwise `machine_to_work` double-counts the TLO and the probed point misses its target.
    cs.tlo = -3.0;
    cs.set_wcs_offset_to_position(0, [0.0, 0.0, 10.0, 0.0], [0.0, 0.0, 2.0, 0.0], [false, false, true, false]);
    approx(cs.machine_to_work([0.0, 0.0, 10.0, 0.0]), [0.0, 0.0, 2.0, 0.0]);
  }

  #[test]
  fn g10_l20_with_no_g92_or_tlo_is_plain_difference() {
    let mut cs = CoordinateSystems::new();
    // With neither G92 nor TLO active the corrected formula reduces to the original `machine − work`.
    cs.set_wcs_offset_to_position(0, [100.0, 50.0, 10.0, 0.0], [0.0, 0.0, 0.0, 0.0], ALL);
    approx(cs.wcs_offset(0).unwrap(), [100.0, 50.0, 10.0, 0.0]);
    approx(cs.machine_to_work([100.0, 50.0, 10.0, 0.0]), [0.0, 0.0, 0.0, 0.0]);
  }

  // ---- G92 set / clear --------------------------------------------------------------------------

  #[test]
  fn g92_makes_current_machine_read_commanded_work() {
    let mut cs = CoordinateSystems::new();
    cs.set_wcs_offset(0, [10.0, 0.0, 0.0, 0.0], ALL); // G54 X = 10.
    // At machine X = 25 declare work X = 0. The WCO must then map machine 25 → work 0.
    cs.set_g92_to_position([25.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [true, false, false, false]);
    approx(cs.machine_to_work([25.0, 0.0, 0.0, 0.0]), [0.0, 0.0, 0.0, 0.0]);
    // G92 is the extra offset on top of G54: g92_x = 25 − 10 − 0 = 15.
    assert!((cs.g92_offset()[0] - 15.0).abs() < 1e-4);
  }

  #[test]
  fn g92_is_independent_of_active_wcs() {
    let mut cs = CoordinateSystems::new();
    cs.set_wcs_offset(0, [10.0, 0.0, 0.0, 0.0], ALL);
    cs.set_wcs_offset(1, [40.0, 0.0, 0.0, 0.0], ALL);
    // Set G92 while G54 active so machine 25 reads work 0.
    cs.set_g92_to_position([25.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [true, false, false, false]);
    let g92 = cs.g92_offset();
    // Switching to G55 keeps the same G92 value (it is not folded into the WCS).
    cs.select_wcs(1);
    assert!((cs.g92_offset()[0] - g92[0]).abs() < 1e-6);
  }

  #[test]
  fn clear_g92_resets_to_identity() {
    let mut cs = CoordinateSystems::new();
    cs.g92 = [1.0, 2.0, 3.0, 0.0];
    cs.clear_g92();
    approx(cs.g92_offset(), [0.0, 0.0, 0.0, 0.0]);
  }

  // ---- G28.1 store + recall, G30 -----------------------------------------------------------------

  #[test]
  fn store_and_recall_g28_in_machine_coords() {
    let mut cs = CoordinateSystems::new();
    cs.store_predefined(0, [12.0, 34.0, 56.0, 0.0]);
    approx(cs.predefined(0).unwrap(), [12.0, 34.0, 56.0, 0.0]);
  }

  #[test]
  fn store_g30_independent_of_g28() {
    let mut cs = CoordinateSystems::new();
    cs.store_predefined(0, [1.0, 1.0, 1.0, 0.0]);
    cs.store_predefined(1, [9.0, 9.0, 9.0, 0.0]);
    approx(cs.predefined(0).unwrap(), [1.0, 1.0, 1.0, 0.0]);
    approx(cs.predefined(1).unwrap(), [9.0, 9.0, 9.0, 0.0]);
  }

  #[test]
  fn store_predefined_out_of_range_is_ignored() {
    let mut cs = CoordinateSystems::new();
    cs.store_predefined(9, [1.0, 2.0, 3.0, 0.0]);
    assert_eq!(cs.predefined(9), None);
  }

  // ---- G43.1 / G49 TLO --------------------------------------------------------------------------

  #[test]
  fn g43_1_applies_z_tlo_into_wco_z_only() {
    let mut cs = CoordinateSystems::new();
    cs.apply_tlo(-14.442);
    assert!((cs.tlo() + 14.442).abs() < 1e-4);
    // TLO contributes only to Z.
    approx(cs.wco(), [0.0, 0.0, -14.442, 0.0]);
  }

  #[test]
  fn g49_cancels_tlo() {
    let mut cs = CoordinateSystems::new();
    cs.apply_tlo(-3.0);
    cs.cancel_tlo();
    assert!(cs.tlo().abs() < 1e-6);
    approx(cs.wco(), [0.0, 0.0, 0.0, 0.0]);
  }

  // ---- finite-guards: a NaN/inf word never poisons the offset set -------------------------------

  #[test]
  fn non_finite_words_are_rejected_per_axis() {
    let mut cs = CoordinateSystems::new();
    cs.set_wcs_offset(0, [5.0, 5.0, 5.0, 0.0], ALL);
    cs.set_wcs_offset(0, [f32::NAN, 7.0, f32::INFINITY, 0.0], ALL);
    // X (NaN) and Z (inf) rejected, only the finite Y written.
    approx(cs.wcs_offset(0).unwrap(), [5.0, 7.0, 5.0, 0.0]);
    cs.apply_tlo(f32::NAN);
    assert!(cs.tlo().abs() < 1e-6);
  }

  // ---- volatile clear / persistent projection ---------------------------------------------------

  #[test]
  fn clear_volatile_drops_g92_and_tlo_but_keeps_persistent() {
    let mut cs = CoordinateSystems::new();
    cs.set_wcs_offset(0, [10.0, 0.0, 0.0, 0.0], ALL);
    cs.store_predefined(0, [1.0, 2.0, 3.0, 0.0]);
    cs.g92 = [7.0, 7.0, 7.0, 0.0];
    cs.tlo = 4.0;
    cs.clear_volatile();
    approx(cs.g92_offset(), [0.0, 0.0, 0.0, 0.0]);
    assert!(cs.tlo().abs() < 1e-6);
    // Persistent G54 + G28 survive.
    approx(cs.wcs_offset(0).unwrap(), [10.0, 0.0, 0.0, 0.0]);
    approx(cs.predefined(0).unwrap(), [1.0, 2.0, 3.0, 0.0]);
  }

  #[test]
  fn persistent_round_trips_through_load() {
    let mut cs = CoordinateSystems::new();
    cs.set_wcs_offset(3, [1.0, 2.0, 3.0, 0.0], ALL); // G57
    cs.store_predefined(1, [4.0, 5.0, 6.0, 0.0]); // G30
    cs.select_wcs(3);
    let persistent = cs.persistent();
    let mut restored = CoordinateSystems::new();
    restored.load_persistent(&persistent);
    assert_eq!(restored.active_wcs(), 3);
    approx(restored.wcs_offset(3).unwrap(), [1.0, 2.0, 3.0, 0.0]);
    approx(restored.predefined(1).unwrap(), [4.0, 5.0, 6.0, 0.0]);
  }

  #[test]
  fn clear_all_resets_everything_to_default() {
    let mut cs = CoordinateSystems::new();
    cs.set_wcs_offset(2, [1.0, 1.0, 1.0, 0.0], ALL);
    cs.store_predefined(0, [2.0, 2.0, 2.0, 0.0]);
    cs.g92 = [3.0, 3.0, 3.0, 0.0];
    cs.tlo = 4.0;
    cs.select_wcs(2);
    cs.clear_all();
    assert_eq!(cs, CoordinateSystems::default());
  }

  // ---- persistence round-trip through a mock store ----------------------------------------------

  // firmware-core is `#![no_std]`; `std` links only under `#[cfg(test)]` for the async mock below.
  extern crate std;

  /// An in-memory [`RecordStore`] for the persistence round-trip test: holds at most one framed record,
  /// mirroring the firmware bin's single-record NVS key. Its futures are immediately ready.
  #[derive(Default)]
  struct MemStore {
    record: Option<std::vec::Vec<u8>>,
  }

  impl RecordStore for MemStore {
    async fn load(&mut self, buf: &mut [u8]) -> Result<usize, StoreError> {
      match &self.record {
        Some(bytes) if bytes.len() <= buf.len() => {
          buf[..bytes.len()].copy_from_slice(bytes);
          Ok(bytes.len())
        }
        Some(_) => Err(StoreError::TooLarge),
        None => Err(StoreError::NotFound),
      }
    }

    async fn save(&mut self, frame: &[u8]) -> Result<(), StoreError> {
      self.record = Some(frame.to_vec());
      Ok(())
    }
  }

  /// Drive an immediately-ready future to completion with the no-op waker. The mock store futures never pend
  /// (no real I/O), so a single poll resolves them; `Waker::noop` avoids a hand-rolled `RawWaker` that
  /// `#![deny(unsafe_code)]` would forbid. Mirrors the `block_on` in the settings tests.
  fn block_on<F: core::future::Future>(future: F) -> F::Output {
    use core::task::{Context, Poll, Waker};
    let mut context = Context::from_waker(Waker::noop());
    let mut future = core::pin::pin!(future);
    match future.as_mut().poll(&mut context) {
      Poll::Ready(output) => output,
      Poll::Pending => panic!("store future pended; the host MemStore must resolve in one poll"),
    }
  }

  #[test]
  fn coordinate_data_persists_and_reloads_round_trip() {
    let mut store = MemStore::default();
    let mut cs = CoordinateSystems::new();
    cs.set_wcs_offset(0, [10.0, 20.0, 5.0, 0.0], ALL); // G54
    cs.set_wcs_offset(5, [-1.5, 2.5, 3.0, 0.0], ALL); // G59
    cs.store_predefined(0, [100.0, 0.0, 50.0, 0.0]); // G28
    cs.store_predefined(1, [0.0, 100.0, 50.0, 0.0]); // G30
    cs.select_wcs(5);
    // Set volatile state that must NOT persist.
    cs.g92 = [9.0, 9.0, 9.0, 0.0];
    cs.tlo = 4.0;

    block_on(store_coordinates(&mut store, &cs.persistent())).expect("save");

    let loaded = block_on(load_or_default(&mut store));
    let mut restored = CoordinateSystems::new();
    restored.load_persistent(&loaded);
    // Persistent data survives.
    assert_eq!(restored.active_wcs(), 5);
    approx(restored.wcs_offset(0).unwrap(), [10.0, 20.0, 5.0, 0.0]);
    approx(restored.wcs_offset(5).unwrap(), [-1.5, 2.5, 3.0, 0.0]);
    approx(restored.predefined(0).unwrap(), [100.0, 0.0, 50.0, 0.0]);
    approx(restored.predefined(1).unwrap(), [0.0, 100.0, 50.0, 0.0]);
    // Volatile data did NOT persist (a fresh model has zero G92/TLO).
    approx(restored.g92_offset(), [0.0, 0.0, 0.0, 0.0]);
    assert!(restored.tlo().abs() < 1e-6);
  }

  #[test]
  fn load_falls_back_to_default_when_nothing_stored() {
    let mut store = MemStore::default();
    let loaded = block_on(load_or_default(&mut store));
    assert_eq!(loaded, CoordinatePersistent::default());
  }

  #[test]
  fn load_falls_back_to_default_on_corrupt_record() {
    let mut store = MemStore { record: Some(std::vec![0xFF; 20]) };
    let loaded = block_on(load_or_default(&mut store));
    assert_eq!(loaded, CoordinatePersistent::default());
  }

  // ---- Phase C: the no-touch-plate Z-zero workflow end-to-end ------------------------------------

  /// The `docs/tlo-offsets.md` single-tool PCB Z-zero sequence: probe down to the copper, then set work Z so the
  /// probed point reads the plate thickness — making the copper TOP read WPos Z = 0. This exercises the same
  /// coordinate math the firmware bin applies on a `G38.2` → `G10 L20`/`G92` sequence, proving the end-to-end
  /// result is correct without any hardware. The probe stop is the machine Z the executor would latch.
  #[test]
  fn z_zero_via_g10_l20_puts_copper_top_at_wpos_zero() {
    let mut cs = CoordinateSystems::new();
    // The probe (`G38.2 Z-…`) tip touched the TOP of the touch plate, which sits ON the copper, so the trigger
    // machine Z is plate-thickness ABOVE the copper. The plate is 1.0 mm thick. Setting the probed point's work Z
    // to the plate thickness puts the copper surface (plate_thickness below) at work Z 0. This is ioSender's
    // `pos.Z = WorkpieceHeight(0) + TouchPlateHeight(1.0)` form realized via `G10 L20 P1 Z1.0`.
    let probe_machine_z = -42.0f32;
    let plate_thickness = 1.0f32;
    // `G10 L20 P1 Z<plate>` sets G54 so that the current machine position (the probe stop) reads work Z = plate.
    cs.set_wcs_offset_to_position(
      0,
      [0.0, 0.0, probe_machine_z, 0.0],
      [0.0, 0.0, plate_thickness, 0.0],
      [false, false, true, false],
    );
    // The probed point (plate top) now reads work Z = plate thickness (1.0).
    approx(cs.machine_to_work([0.0, 0.0, probe_machine_z, 0.0]), [0.0, 0.0, plate_thickness, 0.0]);
    // The copper TOP is plate_thickness BELOW the probed point in machine Z → its work Z is 0.
    let copper_top_machine_z = probe_machine_z - plate_thickness;
    approx(cs.machine_to_work([0.0, 0.0, copper_top_machine_z, 0.0]), [0.0, 0.0, 0.0, 0.0]);
  }

  /// The same Z-zero outcome via the `G92` path (ioSender's `G92` coordinate mode): `G92 Z<plate>` makes the
  /// probe-stop machine position read work Z = plate thickness, so the copper top is work Z 0. Independent of the
  /// active WCS (G92 is a session offset on top of it).
  #[test]
  fn z_zero_via_g92_puts_copper_top_at_wpos_zero() {
    let mut cs = CoordinateSystems::new();
    cs.set_wcs_offset(0, [0.0, 0.0, 5.0, 0.0], ALL); // a non-zero G54 Z, to prove G92 is independent of it.
    let probe_machine_z = -42.0f32;
    let plate_thickness = 1.0f32;
    cs.set_g92_to_position(
      [0.0, 0.0, probe_machine_z, 0.0],
      [0.0, 0.0, plate_thickness, 0.0],
      [false, false, true, false],
    );
    approx(cs.machine_to_work([0.0, 0.0, probe_machine_z, 0.0]), [0.0, 0.0, plate_thickness, 0.0]);
    approx(cs.machine_to_work([0.0, 0.0, probe_machine_z - plate_thickness, 0.0]), [0.0, 0.0, 0.0, 0.0]);
  }

  /// The single-tool PCB Z-zero path uses `G10 L2`/`G92`, NEVER `G43.1` (`docs/tlo-offsets.md` Decision #1 /
  /// Caveats): the surface is re-zeroed each session, so no dynamic TLO is involved. With NO TLO, the WCO is a
  /// single subtraction (`WPos = MPos − WCO`) and the `[TLO:]` report stays 0 — the canonical PCB-mill state this
  /// phase targets. (The `$TLR`/`$TPW` reference-tool path that combines `G43.1` with probing is explicitly out
  /// of scope here and is the only case where `G10 L20` would need to account for a live TLO.)
  #[test]
  fn pcb_z_zero_path_leaves_tlo_at_zero() {
    let mut cs = CoordinateSystems::new();
    let probe_machine_z = -42.0f32;
    let plate_thickness = 1.0f32;
    cs.set_wcs_offset_to_position(
      0,
      [0.0, 0.0, probe_machine_z, 0.0],
      [0.0, 0.0, plate_thickness, 0.0],
      [false, false, true, false],
    );
    // No TLO is set during the single-tool probe-zero, so `[TLO:]` is 0 and the WCO is just the WCS offset.
    assert_eq!(cs.tlo(), 0.0);
    approx(cs.machine_to_work([0.0, 0.0, probe_machine_z, 0.0]), [0.0, 0.0, plate_thickness, 0.0]);
    approx(cs.machine_to_work([0.0, 0.0, probe_machine_z - plate_thickness, 0.0]), [0.0, 0.0, 0.0, 0.0]);
  }
}
