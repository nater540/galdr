//! Coolant controller (DOC-07 follow-up): the host-tested M7/M8/M9 sequencing for the mist and flood coolant
//! outputs, mirroring [`crate::spindle::SpindleController`].
//!
//! Coolant is grbl modal group 8: M7 (mist) and M8 (flood) are INDEPENDENT — both can be active at once — and M9
//! clears both. The controller drives two [`DigitalOut`] lines (one per circuit) from a [`CoolantState`], so every
//! transition is unit-tested off-target with recording mocks; only the `firmware` binary maps these to real GPIO.
//! On THIS machine no coolant GPIO is budgeted yet (see CLAUDE.md), so the firmware wiring leaves the peripheral
//! binding a clearly-marked stub — but the controller, the traits, and the safety behavior are real and host-tested
//! now, so the day a driver stage is added the only new code is the GPIO `init`.
//!
//! ## Logical-level conventions (the firmware impl maps polarity, see [`DigitalOut`])
//! The controller deals only in LOGICAL levels: `true` = the circuit is ON. A relay/SSD that is active-low (or any
//! board-specific polarity) is the firmware [`DigitalOut`] impl's concern, exactly as it is for the spindle ENABLE
//! line — so the controller carries zero board knowledge.
//!
//! ## Safety
//! [`CoolantController::emergency_stop`] forces BOTH circuits off unconditionally, for ALARM / soft reset / sleep /
//! M2 / M30 — mirroring the spindle's `emergency_stop`. The firmware routes every coolant-killing event through it.

use crate::gcode::CoolantState;
use crate::hal_traits::{DigitalOut, DigitalOutError};

/// Bit for mist (M7) in the packed coolant `u8` bitmask (see [`coolant_mask`]). The firmware publishes this mask
/// through an atomic that the coolant task re-reads, so the encoding is a firmware↔task contract single-sourced here.
pub const COOLANT_BIT_MIST: u8 = 0b01;
/// Bit for flood (M8) in the packed coolant `u8` bitmask (see [`coolant_mask`]).
pub const COOLANT_BIT_FLOOD: u8 = 0b10;

/// Pack a [`CoolantState`] into the `u8` coolant bitmask (`bit0 = mist`, `bit1 = flood`). The firmware stores the
/// result in the atomic the coolant task reads; the inverse unpack lives in the firmware wiring.
pub fn coolant_mask(coolant: CoolantState) -> u8 {
  (if coolant.mist { COOLANT_BIT_MIST } else { 0 }) | (if coolant.flood { COOLANT_BIT_FLOOD } else { 0 })
}

/// An error driving the coolant outputs. Wraps the underlying [`DigitalOut`] error per circuit so the firmware
/// task can surface a hardware fault rather than panicking, per the firmware-core no-`unwrap` rule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CoolantError {
  /// The MIST (M7) output failed.
  Mist(DigitalOutError),
  /// The FLOOD (M8) output failed.
  Flood(DigitalOutError),
}

/// The host-tested coolant controller (DOC-07 follow-up). Generic over the two output sinks so it is exercised
/// with recording mocks off-target; the firmware binary instantiates it over the (hardware-gated) coolant GPIO.
///
/// Tracks the last applied [`CoolantState`] only for [`state`](CoolantController::state) reporting; every
/// [`apply`](CoolantController::apply) drives BOTH outputs from the requested state, so the controller is
/// idempotent and has no hidden ordering dependence — a re-apply of the same state simply re-drives the same levels.
pub struct CoolantController<Mist: DigitalOut, Flood: DigitalOut> {
  mist: Mist,
  flood: Flood,
  /// The last state actually driven onto the outputs. `off` after construction or an `emergency_stop`.
  last_state: CoolantState,
}

impl<Mist: DigitalOut, Flood: DigitalOut> CoolantController<Mist, Flood> {
  /// Build a controller over the two output sinks. The controller starts OFF (both circuits de-asserted) to match
  /// the firmware power-on (coolant is off until an M7/M8); the caller is expected to have left the physical
  /// outputs de-asserted, so no I/O is performed here.
  pub fn new(mist: Mist, flood: Flood) -> Self {
    CoolantController { mist, flood, last_state: CoolantState::off() }
  }

  /// Apply an M7/M8/M9 coolant command, driving each circuit to its requested level. M7 and M8 are independent, so
  /// `state` carries the FULL desired state (both flags), not a single toggle: the firmware resolves M7/M8/M9 into
  /// the modal [`CoolantState`] and hands it here. Drives FLOOD then MIST; records the applied state.
  pub fn apply(&mut self, state: CoolantState) -> Result<(), CoolantError> {
    self.flood.set(state.flood).map_err(CoolantError::Flood)?;
    self.mist.set(state.mist).map_err(CoolantError::Mist)?;
    self.last_state = state;
    Ok(())
  }

  /// Unconditionally force BOTH coolant circuits off: for ALARM / soft reset (`0x18`) / sleep / M2 / M30 —
  /// independent of the last command (DOC-07 safety). Idempotent: a second call from an already-off controller
  /// still re-drives the safe (off) outputs, which is harmless defense in depth, mirroring the spindle e-stop.
  pub fn emergency_stop(&mut self) -> Result<(), CoolantError> {
    self.apply(CoolantState::off())
  }

  /// The last [`CoolantState`] driven onto the outputs (`off` when both circuits are off / after an e-stop).
  pub fn state(&self) -> CoolantState {
    self.last_state
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::hal_traits::{DigitalOut, DigitalOutError};

  /// A recording [`DigitalOut`] capturing every logical level written, mirroring the spindle test's `RecordingOut`.
  #[derive(Default)]
  struct RecordingOut {
    levels: heapless::Vec<bool, 16>,
  }

  impl DigitalOut for RecordingOut {
    fn set(&mut self, level: bool) -> Result<(), DigitalOutError> {
      self.levels.push(level).expect("test out buffer overflow");
      Ok(())
    }
  }

  impl RecordingOut {
    fn last(&self) -> bool {
      *self.levels.last().expect("at least one level written")
    }
  }

  /// A [`DigitalOut`] that always fails, to prove the controller surfaces a transport error rather than panicking.
  struct FailingOut;

  impl DigitalOut for FailingOut {
    fn set(&mut self, _level: bool) -> Result<(), DigitalOutError> {
      Err(DigitalOutError::Transport)
    }
  }

  type Ctrl = CoolantController<RecordingOut, RecordingOut>;

  fn controller() -> Ctrl {
    CoolantController::new(RecordingOut::default(), RecordingOut::default())
  }

  #[test]
  fn flood_on_asserts_only_the_flood_output() {
    let mut c = controller();
    c.apply(CoolantState { mist: false, flood: true }).expect("apply");
    assert_eq!(c.state(), CoolantState { mist: false, flood: true });
    assert!(c.flood.last(), "flood circuit asserted (M8)");
    assert!(!c.mist.last(), "mist circuit stays off");
  }

  #[test]
  fn mist_on_asserts_only_the_mist_output() {
    let mut c = controller();
    c.apply(CoolantState { mist: true, flood: false }).expect("apply");
    assert!(c.mist.last(), "mist circuit asserted (M7)");
    assert!(!c.flood.last(), "flood circuit stays off");
  }

  #[test]
  fn mist_and_flood_are_independently_active() {
    // grbl modal group 8: M7 and M8 can BOTH be on at once. Applying the combined state asserts both circuits.
    let mut c = controller();
    c.apply(CoolantState { mist: true, flood: true }).expect("apply");
    assert!(c.mist.last());
    assert!(c.flood.last());
    assert_eq!(c.state(), CoolantState { mist: true, flood: true });
  }

  #[test]
  fn m9_clears_both_circuits() {
    let mut c = controller();
    c.apply(CoolantState { mist: true, flood: true }).expect("both on");
    c.apply(CoolantState::off()).expect("m9 all off");
    assert!(!c.mist.last(), "mist off after M9");
    assert!(!c.flood.last(), "flood off after M9");
    assert_eq!(c.state(), CoolantState::off());
  }

  #[test]
  fn emergency_stop_forces_both_off_and_is_idempotent() {
    let mut c = controller();
    c.apply(CoolantState { mist: true, flood: true }).expect("both on");
    c.emergency_stop().expect("estop");
    assert!(!c.mist.last());
    assert!(!c.flood.last());
    assert_eq!(c.state(), CoolantState::off());
    // A second e-stop from an already-off controller still re-drives the safe (off) levels (defense in depth).
    let before = c.flood.levels.len();
    c.emergency_stop().expect("estop again");
    assert!(c.flood.levels.len() > before, "idempotent e-stop still re-drives the safe level");
    assert_eq!(c.state(), CoolantState::off());
  }

  #[test]
  fn flood_transport_error_is_surfaced_not_panicked() {
    // A failing FLOOD output must propagate as a CoolantError, never an unwrap/panic in firmware-core.
    let mut c = CoolantController::new(RecordingOut::default(), FailingOut);
    let err = c.apply(CoolantState { mist: false, flood: true }).expect_err("flood fails");
    assert_eq!(err, CoolantError::Flood(DigitalOutError::Transport));
  }

  #[test]
  fn coolant_mask_packs_every_flood_mist_combination() {
    assert_eq!(coolant_mask(CoolantState::off()), 0);
    assert_eq!(coolant_mask(CoolantState { mist: true, flood: false }), COOLANT_BIT_MIST);
    assert_eq!(coolant_mask(CoolantState { mist: false, flood: true }), COOLANT_BIT_FLOOD);
    assert_eq!(
      coolant_mask(CoolantState { mist: true, flood: true }),
      COOLANT_BIT_MIST | COOLANT_BIT_FLOOD,
    );
  }
}
