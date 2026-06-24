//! Coolant hardware wiring (DOC-07 follow-up): the esp-hal-side binding for the firmware-core coolant traits.
//!
//! The host-tested M7/M8/M9 sequencing and the all-off safety behavior live in
//! [`firmware_core::coolant::CoolantController`]; this module maps that controller onto hardware — EXCEPT that no
//! coolant GPIO is budgeted on this board yet (see CLAUDE.md: "the one subsystem genuinely stubbed at the hardware
//! boundary is the coolant GPIO — no GPIO budgeted, no driver stage yet"). So the [`DigitalOut`] impl here is a
//! deliberate, clearly-marked **stub**: it accepts the logical level and (optionally) traces it, but drives no pin.
//! The controller, the modal coolant state, the consumer wiring, and the all-off-on-alarm/reset safety path are all
//! real and host-tested NOW; the day a relay/SSR + GPIO are added, the only new code is replacing [`CoolantPin`]'s
//! body with an `esp_hal::gpio::Output` exactly like the spindle's [`crate::spindle::GpioOut`] — nothing else moves.
//!
//! ## Logical-level contract
//! Like the spindle outputs, the controller deals in LOGICAL levels (`true` = the circuit is ON). When a real pin
//! is wired, the active polarity (a relay may be active-low) lives in this impl, NOT in firmware-core — mirroring
//! [`crate::spindle::GpioOut`]'s `active_high` field. The stub records the logical level verbatim.

use firmware_core::coolant::CoolantController;
use firmware_core::hal_traits::{DigitalOut, DigitalOutError};

/// The concrete coolant controller type the firmware drives: the host-tested [`CoolantController`] over two
/// (currently stubbed) [`CoolantPin`] outputs — one for mist (M7), one for flood (M8). The `coolant` task owns one
/// of these by `'static` mutable borrow, mirroring the spindle.
pub type Coolant = CoolantController<CoolantPin, CoolantPin>;

/// A [`DigitalOut`] for one coolant circuit. **STUB**: no coolant GPIO is budgeted on this board, so `set` does not
/// touch hardware — it only (optionally) traces the requested level so a bench operator can confirm the logic fires.
/// It never errors (a recording stub cannot fail), so the all-off safety path is exercised end to end without
/// hardware. Replace the body with an `esp_hal::gpio::Output` (plus an `active_high` polarity field, like
/// [`crate::spindle::GpioOut`]) when a driver stage is added; the trait surface and every caller stay unchanged.
pub struct CoolantPin {
  /// A human label for the circuit ("mist" / "flood"), used only in the optional defmt trace.
  #[cfg_attr(not(feature = "defmt"), allow(dead_code))]
  label: &'static str,
}

impl DigitalOut for CoolantPin {
  /// STUB: record the requested logical level; drive no pin (no coolant GPIO budgeted). Never errors.
  fn set(&mut self, _level: bool) -> Result<(), DigitalOutError> {
    #[cfg(feature = "defmt")]
    defmt::debug!("coolant[{}] = {} (GPIO stub — no driver stage budgeted)", self.label, _level);
    Ok(())
  }
}

/// Assemble the (stubbed) [`Coolant`] controller. No peripheral is claimed — there is no coolant GPIO budgeted — so
/// this takes no pins and cannot fail, unlike [`crate::spindle::init`]. The controller starts OFF (both circuits
/// de-asserted) to match the firmware power-on. When a driver stage is added, this gains the pin arguments and the
/// `CoolantPin`s wrap real `Output`s; the `coolant` task and the consumer chokepoints do not change.
pub fn init() -> Coolant {
  CoolantController::new(CoolantPin { label: "mist" }, CoolantPin { label: "flood" })
}
