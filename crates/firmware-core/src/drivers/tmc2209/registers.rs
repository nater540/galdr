//! TMC2209 register addresses, per-register field codecs, and current-scale math (DOC-03).
//!
//! This sits one layer above the byte-level datagram codec in [`super`]: it turns the abstract intent
//! ("configure 1/16 microstepping", "drive 800 mA RMS run current") into the exact 32-bit register
//! values that [`super::encode_write`] then frames. Everything here is pure integer/`f32` arithmetic
//! with no I/O, so it is fully host-testable. The [`super::manager`] layer composes these builders into
//! the startup sequence and runtime current scaling, talking to the wire through the
//! [`TmcBus`](crate::hal_traits::TmcBus) trait.
//!
//! Register addresses and field positions are taken from the TMC2209 datasheet (rev 1.09). Only the
//! fields this firmware actually programs are modelled; reserved bits are left zero.

use libm::roundf;

/// `GCONF` (0x00): global configuration — UART mode flags, microstep-register select, current source.
pub const GCONF: u8 = 0x00;
/// `GSTAT` (0x01): global status; bits are write-1-to-clear (reset / driver-error / undervoltage).
pub const GSTAT: u8 = 0x01;
/// `IFCNT` (0x02): UART write counter, increments once per accepted write datagram. Used to verify writes.
pub const IFCNT: u8 = 0x02;
/// `SLAVECONF` (0x03): UART send-delay for clean multi-node bus turn-around.
pub const SLAVECONF: u8 = 0x03;
/// `IOIN` (0x06): input pin states plus the chip `VERSION` field used to confirm a driver is present.
pub const IOIN: u8 = 0x06;
/// `IHOLD_IRUN` (0x10): run/hold current-scale selectors and the hold-current ramp-down delay.
pub const IHOLD_IRUN: u8 = 0x10;
/// `TPOWERDOWN` (0x11): delay before the hold current is reduced after standstill is reached.
pub const TPOWERDOWN: u8 = 0x11;
/// `TPWMTHRS` (0x13): velocity threshold for the StealthChop→SpreadCycle crossover.
pub const TPWMTHRS: u8 = 0x13;
/// `TCOOLTHRS` (0x14): lower velocity threshold for CoolStep / StallGuard (future sensorless homing).
pub const TCOOLTHRS: u8 = 0x14;
/// `SGTHRS` (0x40): StallGuard4 stall threshold (future sensorless homing).
pub const SGTHRS: u8 = 0x40;
/// `SG_RESULT` (0x41): StallGuard load measurement (read-only).
pub const SG_RESULT: u8 = 0x41;
/// `COOLCONF` (0x42): CoolStep configuration.
pub const COOLCONF: u8 = 0x42;
/// `CHOPCONF` (0x6C): chopper configuration — microstep resolution, interpolation, blank time, vsense.
pub const CHOPCONF: u8 = 0x6C;
/// `DRV_STATUS` (0x6F): live driver diagnostics — over-temperature, shorts, open load, standstill (read-only).
pub const DRV_STATUS: u8 = 0x6F;
/// `PWMCONF` (0x70): StealthChop PWM auto-scaling configuration.
pub const PWMCONF: u8 = 0x70;

// --- GCONF (0x00) field bits ---------------------------------------------------------------------

/// `I_scale_analog`: use the VREF analog input as the current reference (0 ⇒ internal reference, used here).
pub const GCONF_I_SCALE_ANALOG: u32 = 1 << 0;
/// `internal_Rsense`: select the internal sense resistor (0 ⇒ external sense resistor, used here).
pub const GCONF_INTERNAL_RSENSE: u32 = 1 << 1;
/// `en_SpreadCycle`: force SpreadCycle chopper (0 ⇒ StealthChop at standstill / low speed, used here).
pub const GCONF_EN_SPREADCYCLE: u32 = 1 << 2;
/// `shaft`: invert motor direction.
pub const GCONF_SHAFT: u32 = 1 << 3;
/// `pdn_disable`: disable the PDN_UART power-down function so the pin works as the UART line (required).
pub const GCONF_PDN_DISABLE: u32 = 1 << 6;
/// `mstep_reg_select`: take microstep resolution from `CHOPCONF.MRES` rather than the MS1/MS2 pins (required;
/// MS1/MS2 are repurposed as the UART node address on this bus).
pub const GCONF_MSTEP_REG_SELECT: u32 = 1 << 7;
/// `multistep_filt`: enable the step-pulse input filter for cleaner step timing.
pub const GCONF_MULTISTEP_FILT: u32 = 1 << 8;

/// The canonical `GCONF` value for UART control on this firmware (DOC-03 init step 2): take microstepping
/// from the register (`mstep_reg_select`), free PDN_UART for the bus (`pdn_disable`), use the internal
/// current reference (`I_scale_analog = 0`), and enable the step filter (`multistep_filt`). Equals 0x1C0.
pub fn gconf_uart_control() -> u32 {
  GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SELECT | GCONF_MULTISTEP_FILT
}

// --- GSTAT (0x01) --------------------------------------------------------------------------------

/// Writing this to `GSTAT` clears all three latched status bits (`reset`, `drv_err`, `uv_cp`); the bits
/// are write-1-to-clear, so the firmware writes it once at init to start from a known-clean state.
pub const GSTAT_CLEAR_ALL: u32 = 0b111;

// --- SLAVECONF (0x03) ----------------------------------------------------------------------------

/// Build a `SLAVECONF` value setting `SENDDELAY` (bits 11:8) to `delay` (units of 8 bit-times). DOC-03
/// recommends `SENDDELAY ≥ 2` on a multi-node bus so a replying driver releases the line before the next
/// device drives it. `delay` is masked to its 4-bit field.
pub fn slaveconf(send_delay: u8) -> u32 {
  (u32::from(send_delay) & 0x0F) << 8
}

// --- IHOLD_IRUN (0x10) ---------------------------------------------------------------------------

/// Build an `IHOLD_IRUN` value from the run/hold current-scale selectors and the hold ramp-down delay:
/// `IHOLD` (bits 4:0), `IRUN` (bits 12:8), `IHOLDDELAY` (bits 19:16). Each field is masked to its width,
/// so an out-of-range argument is truncated rather than corrupting an adjacent field.
pub fn ihold_irun(irun_cs: u8, ihold_cs: u8, ihold_delay: u8) -> u32 {
  (u32::from(ihold_cs) & 0x1F)
    | ((u32::from(irun_cs) & 0x1F) << 8)
    | ((u32::from(ihold_delay) & 0x0F) << 16)
}

// --- CHOPCONF (0x6C) -----------------------------------------------------------------------------

/// `CHOPCONF.vsense` (bit 17): selects the high-sensitivity sense voltage (`V_fs ≈ 0.18 V`) when set, or
/// the low-sensitivity reference (`V_fs ≈ 0.325 V`) when clear. The current-scale math picks this; the
/// CHOPCONF builder must reflect the same choice so the programmed `IRUN`/`IHOLD` map to the intended current.
pub const CHOPCONF_VSENSE: u32 = 1 << 17;
/// `CHOPCONF.intpol` (bit 28): enable the 256-microstep interpolator so the motor moves smoothly even at a
/// coarse `MRES`. Always enabled here (DOC-03 init step 3).
pub const CHOPCONF_INTPOL: u32 = 1 << 28;

/// Off-time (`TOFF`, bits 3:0) used in the chopper; 3 is the datasheet's general-purpose default and any
/// non-zero value enables the driver stage.
const CHOPCONF_TOFF: u32 = 3;
/// Hysteresis start (`HSTRT`, bits 6:4); 5 is the chip reset default and is kept as a sane general value.
const CHOPCONF_HSTRT: u32 = 5 << 4;
/// Hysteresis end (`HEND`, bits 10:7); 0 is the chip reset default and is kept.
const CHOPCONF_HEND: u32 = 0 << 7;
/// Blank time (`TBL`, bits 16:15) = 2 (24 clocks), the datasheet's recommended general value (DOC-03 step 3).
const CHOPCONF_TBL: u32 = 2 << 15;

/// Build a `CHOPCONF` value for the given microstep resolution and sense-voltage selection (DOC-03 step 3):
/// fixed `TOFF=3`, `TBL=2`, reset-default `HSTRT`/`HEND`, `intpol=1`, plus the `MRES` field for `microsteps`
/// and the `vsense` bit. Returns [`None`] if `microsteps` is not a power-of-two TMC2209 resolution
/// (see [`mres_code`]); the manager surfaces that as a configuration error rather than silently mis-stepping.
pub fn chopconf(microsteps: u16, vsense: bool) -> Option<u32> {
  let mres = mres_code(microsteps)?;
  let mut value = CHOPCONF_TOFF | CHOPCONF_HSTRT | CHOPCONF_HEND | CHOPCONF_TBL | CHOPCONF_INTPOL;
  value |= (u32::from(mres) & 0x0F) << 24;
  if vsense {
    value |= CHOPCONF_VSENSE;
  }
  Some(value)
}

/// Map a microstep resolution (microsteps per full step) to the 4-bit `CHOPCONF.MRES` field. The TMC2209
/// supports only the power-of-two resolutions 1..=256; any other value returns [`None`]. The mapping is
/// inverse-log: 256⇒0, 128⇒1, … 1⇒8 (DOC-03 notes `MRES = 4 ⇒ 1/16`).
pub fn mres_code(microsteps: u16) -> Option<u8> {
  Some(match microsteps {
    256 => 0,
    128 => 1,
    64 => 2,
    32 => 3,
    16 => 4,
    8 => 5,
    4 => 6,
    2 => 7,
    1 => 8,
    _ => return None,
  })
}

// --- PWMCONF (0x70) ------------------------------------------------------------------------------

/// `PWMCONF.pwm_autoscale` (bit 18): enable automatic StealthChop current regulation.
pub const PWMCONF_AUTOSCALE: u32 = 1 << 18;
/// `PWMCONF.pwm_autograd` (bit 19): enable automatic StealthChop gradient adaptation.
pub const PWMCONF_AUTOGRAD: u32 = 1 << 19;

/// The TMC2209 `PWMCONF` reset default (0xC10D_0024): a tuned StealthChop configuration the datasheet ships,
/// which already carries `pwm_autoscale` and `pwm_autograd`. [`pwmconf_stealthchop`] re-asserts those two
/// bits on top of it so the intent is explicit and independent of the chip's power-on state.
const PWMCONF_RESET_DEFAULT: u32 = 0xC10D_0024;

/// Build the `PWMCONF` value for automatic StealthChop tuning (DOC-03 init step 6): the datasheet's tuned
/// reset default with `pwm_autoscale` and `pwm_autograd` explicitly enabled.
pub fn pwmconf_stealthchop() -> u32 {
  PWMCONF_RESET_DEFAULT | PWMCONF_AUTOSCALE | PWMCONF_AUTOGRAD
}

// --- Current scaling -----------------------------------------------------------------------------

/// Sense-resistor value on the Adafruit TMC2209 breakout (product 6121): 0.05 Ω (DOC-03 — verify against the
/// board, this differs from the common 0.11 Ω clone value, and a wrong value silently mis-scales current).
pub const R_SENSE_ADAFRUIT_6121_OHMS: f32 = 0.05;

/// Effective parasitic resistance the datasheet adds to `R_sense` in the current formula (≈ 0.02 Ω of MOSFET
/// and bond-wire resistance).
pub const SENSE_PARASITIC_OHMS: f32 = 0.02;

/// Full-scale sense voltage with `CHOPCONF.vsense = 0` (low sensitivity, higher current range).
pub const VFS_VSENSE_LOW_SENSITIVITY: f32 = 0.325;
/// Full-scale sense voltage with `CHOPCONF.vsense = 1` (high sensitivity, finer resolution at low current).
pub const VFS_VSENSE_HIGH_SENSITIVITY: f32 = 0.180;

/// Maximum current-scale value (the field is 5 bits, CS = 0..=31).
pub const CS_MAX: u8 = 31;

/// Minimum current-scale the datasheet recommends for good microstep resolution (DOC-03 "aim for IRUN in
/// 16–31"). This is an ADVISORY AIM, not a guarantee: it is the threshold at which [`rms_current_to_cs`]
/// switches from low to high sensitivity to MAXIMIZE CS for the target current. On a small sense resistor
/// (the 0.05 Ω Adafruit 6121) the high-sensitivity CS for a low PCB-milling current can still land below this
/// — e.g. 800 mA ⇒ CS 13, 400 mA ⇒ CS 6 — and that is correct: the realized current stays within ~1% of
/// target, the chip simply has fewer CS codes to spend at that current. Forcing CS ≥ 16 is physically
/// unattainable without raising the current target, so this constant only steers the sensitivity choice.
pub const CS_MIN_RECOMMENDED: u8 = 16;

/// A resolved current-scale selection: the 5-bit `CS` value plus the `vsense` choice it was computed for.
/// Both `IRUN`/`IHOLD` (which carry `CS`) and `CHOPCONF` (which carries `vsense`) must be programmed
/// consistently, so the two travel together.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CurrentScaling {
  /// The 5-bit current-scale selector, 0..=[`CS_MAX`].
  pub cs: u8,
  /// The sense-voltage selection this `cs` assumes: `true` ⇒ high sensitivity (`vsense = 1`).
  pub vsense: bool,
}

/// Resolve a target RMS motor current (in milliamps) to a [`CurrentScaling`], choosing `vsense` to MAXIMIZE
/// CS resolution for that current (DOC-03). The low-sensitivity range (`vsense = 0`) is tried first; if it
/// would put `CS` below [`CS_MIN_RECOMMENDED`] the high-sensitivity range (`vsense = 1`) is used instead,
/// which yields a larger `CS` for the same current and so finer microstep resolution. Note this only chooses
/// the better of the two ranges: it does NOT guarantee `CS ≥ CS_MIN_RECOMMENDED`. For a low current on a
/// small sense resistor (e.g. the shipped 800 mA / 400 mA on 0.05 Ω) even the high-sensitivity CS lands below
/// 16 (13 and 6 respectively) — the realized current is still within ~1% of target, there are simply fewer CS
/// codes available at that current. The result is always clamped to 0..=[`CS_MAX`].
pub fn rms_current_to_cs(rms_ma: u16, r_sense_ohms: f32) -> CurrentScaling {
  let cs_low_sensitivity = cs_for_current(rms_ma, r_sense_ohms, false);
  if cs_low_sensitivity >= CS_MIN_RECOMMENDED {
    return CurrentScaling { cs: cs_low_sensitivity, vsense: false };
  }
  CurrentScaling { cs: cs_for_current(rms_ma, r_sense_ohms, true), vsense: true }
}

/// Compute the `CS` value that best approximates `rms_ma` at a fixed `vsense` selection, clamped to
/// 0..=[`CS_MAX`]. Inverts the datasheet RMS formula
/// `I_rms = (CS + 1)/32 · V_fs/(R_sense + 0.02) · 1/√2` for `CS`, then rounds to the nearest integer.
///
/// Production always passes a sanitized positive `r_sense_ohms` (see [`Settings::sanitized`]), but this is a
/// `pub` library entry point, so a non-finite intermediate is handled explicitly as defence-in-depth: if
/// `cs` is NaN or infinite — which a non-finite or non-positive-after-parasitic `r_sense_ohms` (e.g. exactly
/// `-0.02` Ω, dividing by zero) can produce — the result is the clamp-to-0 case (no current), NOT a silent
/// `roundf(NaN) as u8 == 0` that masks the bad input. A finite-but-out-of-range `cs` still saturates to
/// 0..=[`CS_MAX`] as before.
pub fn cs_for_current(rms_ma: u16, r_sense_ohms: f32, vsense: bool) -> u8 {
  let i_rms = f32::from(rms_ma) / 1000.0;
  let v_fs = vfs(vsense);
  // CS + 1 = I_rms · 32 · √2 · (R_sense + 0.02) / V_fs.
  let cs_plus_one = i_rms * 32.0 * core::f32::consts::SQRT_2 * (r_sense_ohms + SENSE_PARASITIC_OHMS) / v_fs;
  let cs = roundf(cs_plus_one) - 1.0;
  // Clamp before the cast: a non-finite `cs` (NaN/inf from a degenerate `r_sense`) or a negative one (tiny
  // current) deterministically floors at 0, and an over-range one caps at CS_MAX, so the `as u8` never wraps
  // and a NaN can never slip through `roundf(NaN) as u8` as a silent near-zero current.
  if !cs.is_finite() || cs <= 0.0 {
    0
  } else if cs >= f32::from(CS_MAX) {
    CS_MAX
  } else {
    roundf(cs) as u8
  }
}

/// Forward direction of the datasheet RMS formula: the RMS current (in milliamps) a given `cs`/`vsense`
/// selection produces for `r_sense_ohms`. Used by tests to assert round-trips and by callers that want to
/// report the actual current a selection realizes.
///
/// As with [`cs_for_current`], a non-finite intermediate (NaN/inf from a degenerate `r_sense_ohms`, e.g.
/// exactly `-0.02` Ω which divides by zero) is reported deterministically as 0 mA rather than letting
/// `roundf(NaN) as u16 == 0` silently masquerade as a real zero-current reading.
pub fn cs_to_rms_ma(cs: u8, vsense: bool, r_sense_ohms: f32) -> u16 {
  let v_fs = vfs(vsense);
  let i_rms = (f32::from(cs) + 1.0) / 32.0 * v_fs / (r_sense_ohms + SENSE_PARASITIC_OHMS) / core::f32::consts::SQRT_2;
  let ma = i_rms * 1000.0;
  if !ma.is_finite() || ma <= 0.0 {
    0
  } else if ma >= f32::from(u16::MAX) {
    u16::MAX
  } else {
    roundf(ma) as u16
  }
}

/// The full-scale sense voltage for a `vsense` selection.
fn vfs(vsense: bool) -> f32 {
  if vsense {
    VFS_VSENSE_HIGH_SENSITIVITY
  } else {
    VFS_VSENSE_LOW_SENSITIVITY
  }
}

// --- IOIN (0x06) ---------------------------------------------------------------------------------

/// The `VERSION` value a genuine TMC2209 reports in `IOIN` bits 31:24; the manager uses it to confirm a
/// driver actually answered on the bus (DOC-03 init step 1).
pub const EXPECTED_VERSION: u8 = 0x21;

/// Extract the `VERSION` field (bits 31:24) from a raw `IOIN` register value.
pub fn ioin_version(raw: u32) -> u8 {
  (raw >> 24) as u8
}

// --- IFCNT (0x02) --------------------------------------------------------------------------------

/// Extract the 8-bit UART write counter from a raw `IFCNT` register value. The counter wraps at 256, so
/// write verification compares wrapping deltas.
pub fn ifcnt(raw: u32) -> u8 {
  raw as u8
}

// --- DRV_STATUS (0x6F) ---------------------------------------------------------------------------

/// Decoded view of the `DRV_STATUS` diagnostic register. Wraps the raw value and exposes only the fields the
/// firmware acts on (faults, standstill, actual current scale); reserved bits are ignored.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DrvStatus {
  raw: u32,
}

impl DrvStatus {
  /// Wrap a raw `DRV_STATUS` register value.
  pub fn from_raw(raw: u32) -> Self {
    DrvStatus { raw }
  }

  /// The underlying register value, for logging or finer-grained inspection.
  pub fn raw(self) -> u32 {
    self.raw
  }

  /// `otpw` (bit 0): over-temperature pre-warning — the driver is hot but still operating.
  pub fn overtemp_prewarning(self) -> bool {
    self.raw & (1 << 0) != 0
  }

  /// `ot` (bit 1): over-temperature shutdown — the driver has disabled its output stage.
  pub fn overtemp_shutdown(self) -> bool {
    self.raw & (1 << 1) != 0
  }

  /// Short to ground on either phase (`s2ga` bit 2 / `s2gb` bit 3).
  pub fn short_to_ground(self) -> bool {
    self.raw & ((1 << 2) | (1 << 3)) != 0
  }

  /// Short to supply on either phase (`s2vsa` bit 4 / `s2vsb` bit 5).
  pub fn short_to_supply(self) -> bool {
    self.raw & ((1 << 4) | (1 << 5)) != 0
  }

  /// Open load on either phase (`ola` bit 6 / `olb` bit 7) — usually a disconnected motor wire.
  pub fn open_load(self) -> bool {
    self.raw & ((1 << 6) | (1 << 7)) != 0
  }

  /// `stst` (bit 31): the motor is at standstill (no steps for the `TPOWERDOWN` interval).
  pub fn standstill(self) -> bool {
    self.raw & (1 << 31) != 0
  }

  /// `CS_ACTUAL` (bits 20:16): the current-scale the driver is presently applying, reflecting CoolStep.
  pub fn cs_actual(self) -> u8 {
    ((self.raw >> 16) & 0x1F) as u8
  }

  /// Whether any condition that should disable motion is present: over-temperature shutdown or a short.
  /// Over-temperature *pre-warning* and open load are surfaced separately because they warrant a log/derate
  /// rather than an immediate stop.
  pub fn has_fault(self) -> bool {
    self.overtemp_shutdown() || self.short_to_ground() || self.short_to_supply()
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn gconf_uart_control_matches_datasheet_value() {
    // pdn_disable | mstep_reg_select | multistep_filt = 0x40 | 0x80 | 0x100 = 0x1C0. This is the same value
    // the codec's CRC vector (`crc_write_gconf_node0`) frames, so the two layers agree on the canonical GCONF.
    assert_eq!(gconf_uart_control(), 0x0000_01C0);
  }

  #[test]
  fn mres_code_covers_supported_resolutions() {
    assert_eq!(mres_code(256), Some(0));
    assert_eq!(mres_code(16), Some(4));
    assert_eq!(mres_code(1), Some(8));
    assert_eq!(mres_code(0), None);
    assert_eq!(mres_code(3), None);
    assert_eq!(mres_code(512), None);
  }

  #[test]
  fn chopconf_packs_mres_intpol_and_vsense() {
    // 1/16 microstepping (MRES=4), vsense off: TOFF=3 | HSTRT=5 | TBL=2 | intpol | MRES=4<<24.
    let expected = CHOPCONF_TOFF | CHOPCONF_HSTRT | CHOPCONF_HEND | CHOPCONF_TBL | CHOPCONF_INTPOL | (4 << 24);
    assert_eq!(chopconf(16, false), Some(expected));
    // Turning vsense on only sets bit 17.
    assert_eq!(chopconf(16, true), Some(expected | CHOPCONF_VSENSE));
    // An unsupported resolution is rejected, not silently mis-encoded.
    assert_eq!(chopconf(7, false), None);
  }

  #[test]
  fn ihold_irun_packs_each_field() {
    // IHOLD=3, IRUN=23 (0x17), IHOLDDELAY=7 ⇒ 0x00071703, the codec's CRC-vector payload.
    assert_eq!(ihold_irun(23, 3, 7), 0x0007_1703);
  }

  #[test]
  fn ihold_irun_masks_out_of_range_fields() {
    // Over-range CS values must not bleed into neighbouring fields: each is masked to its width.
    assert_eq!(ihold_irun(0xFF, 0xFF, 0xFF), 0x000F_1F1F);
  }

  #[test]
  fn slaveconf_places_send_delay_in_field() {
    assert_eq!(slaveconf(2), 0x0000_0200);
    assert_eq!(slaveconf(0xFF), 0x0000_0F00);
  }

  #[test]
  fn pwmconf_enables_autoscale_and_autograd() {
    let value = pwmconf_stealthchop();
    assert!(value & PWMCONF_AUTOSCALE != 0);
    assert!(value & PWMCONF_AUTOGRAD != 0);
  }

  #[test]
  fn current_scaling_picks_low_sensitivity_for_high_current() {
    // On the small 0.05 Ω sense resistor the low-sensitivity range only reaches CS 16 above ~1.75 A, so a
    // genuinely high 2.5 A target is what keeps vsense clear (low sensitivity) with CS in band.
    let scale = rms_current_to_cs(2500, R_SENSE_ADAFRUIT_6121_OHMS);
    assert!(!scale.vsense, "high current should stay in the low-sensitivity range");
    assert!(scale.cs >= CS_MIN_RECOMMENDED && scale.cs <= CS_MAX);
  }

  #[test]
  fn current_scaling_switches_to_high_sensitivity_for_low_current() {
    // A typical 1.0 A PCB-milling current on 0.05 Ω lands below CS 16 in the low-sensitivity range, so vsense
    // flips to the high-sensitivity range to keep CS in the recommended band.
    let scale = rms_current_to_cs(1000, R_SENSE_ADAFRUIT_6121_OHMS);
    assert!(scale.vsense, "low current should switch to the high-sensitivity range");
    assert!(scale.cs >= CS_MIN_RECOMMENDED && scale.cs <= CS_MAX);
  }

  #[test]
  fn current_scaling_round_trips_within_one_step() {
    // The realized current for the chosen CS/vsense must be within one CS step (~one part in 32) of target.
    for &target in &[600u16, 800, 1000, 1200, 1500, 1800] {
      let scale = rms_current_to_cs(target, R_SENSE_ADAFRUIT_6121_OHMS);
      let realized = cs_to_rms_ma(scale.cs, scale.vsense, R_SENSE_ADAFRUIT_6121_OHMS);
      let step_ma = cs_to_rms_ma(scale.cs, scale.vsense, R_SENSE_ADAFRUIT_6121_OHMS)
        - cs_to_rms_ma(scale.cs.saturating_sub(1), scale.vsense, R_SENSE_ADAFRUIT_6121_OHMS);
      let error = realized.abs_diff(target);
      assert!(error <= step_ma, "target {target}: realized {realized} off by {error} (step {step_ma})");
    }
  }

  #[test]
  fn current_scaling_clamps_extremes() {
    // Zero current floors CS at 0; an absurd current caps at CS_MAX without wrapping the u8.
    assert_eq!(cs_for_current(0, R_SENSE_ADAFRUIT_6121_OHMS, false), 0);
    assert_eq!(cs_for_current(u16::MAX, R_SENSE_ADAFRUIT_6121_OHMS, true), CS_MAX);
  }

  #[test]
  fn cs_for_current_non_finite_r_sense_clamps_to_zero() {
    // A non-finite `r_sense` makes the intermediate NaN; it must deterministically clamp to CS 0, not slip
    // through `roundf(NaN) as u8 == 0` as a silent near-zero current (Finding #9). Likewise an `r_sense` of
    // exactly -0.02 Ω makes `r_sense + 0.02 == 0`, dividing by zero into a non-finite intermediate.
    assert_eq!(cs_for_current(800, f32::NAN, true), 0);
    assert_eq!(cs_for_current(800, f32::INFINITY, true), 0);
    assert_eq!(cs_for_current(800, -SENSE_PARASITIC_OHMS, true), 0);
  }

  #[test]
  fn cs_to_rms_ma_non_finite_r_sense_reports_zero() {
    // The symmetric guard: a degenerate `r_sense` that divides by zero (exactly -0.02 Ω) or is non-finite
    // reports 0 mA rather than a misleading `roundf(NaN) as u16 == 0` masquerading as a real reading.
    assert_eq!(cs_to_rms_ma(13, true, f32::NAN), 0);
    assert_eq!(cs_to_rms_ma(13, true, -SENSE_PARASITIC_OHMS), 0);
  }

  #[test]
  fn current_scaling_shipped_defaults_select_high_sensitivity_below_band() {
    // The SHIPPED defaults (run 800 mA, hold 400 mA, 0.05 Ω) document the honest behavior corrected in
    // Finding #8: low sensitivity would put CS far below 16, so the resolver switches to high sensitivity,
    // but even there CS lands BELOW the advisory CS_MIN_RECOMMENDED band — 13 for run, 6 for hold. That is
    // expected on a small sense resistor and must not silently regress to a forced-but-wrong CS ≥ 16.
    let run = rms_current_to_cs(800, R_SENSE_ADAFRUIT_6121_OHMS);
    assert!(run.vsense, "800 mA on 0.05 Ω must switch to the high-sensitivity range");
    assert_eq!(run.cs, 13, "800 mA high-sensitivity CS is 13, below the recommended band");
    assert!(run.cs < CS_MIN_RECOMMENDED, "documents that CS_MIN_RECOMMENDED is an aim, not a guarantee");
    // The hold current shares the run current's vsense (one CHOPCONF per driver), so compute its CS at the
    // same high sensitivity rather than re-selecting a range — exactly as the manager does.
    let hold_cs = cs_for_current(400, R_SENSE_ADAFRUIT_6121_OHMS, run.vsense);
    assert_eq!(hold_cs, 6, "400 mA high-sensitivity CS is 6");
    // Despite the sub-band CS, the realized current stays within ~1% of target, so the motors drive correctly.
    let run_realized = cs_to_rms_ma(run.cs, run.vsense, R_SENSE_ADAFRUIT_6121_OHMS);
    let hold_realized = cs_to_rms_ma(hold_cs, run.vsense, R_SENSE_ADAFRUIT_6121_OHMS);
    assert!(run_realized.abs_diff(800) <= 8, "realized run {run_realized} mA within 1% of 800 mA");
    assert!(hold_realized.abs_diff(400) <= 4, "realized hold {hold_realized} mA within 1% of 400 mA");
  }

  #[test]
  fn ioin_version_extracts_top_byte() {
    assert_eq!(ioin_version(0x2100_0000), EXPECTED_VERSION);
    assert_eq!(ioin_version(0x21AB_CDEF), EXPECTED_VERSION);
    assert_eq!(ioin_version(0x0000_0000), 0x00);
  }

  #[test]
  fn ifcnt_takes_low_byte() {
    assert_eq!(ifcnt(0xDEAD_BE42), 0x42);
  }

  #[test]
  fn drv_status_decodes_faults_and_fields() {
    // otpw (bit0) + short-to-ground phase A (bit2) + CS_ACTUAL=0x1F (bits20:16) + standstill (bit31).
    let status = DrvStatus::from_raw((1 << 0) | (1 << 2) | (0x1F << 16) | (1 << 31));
    assert!(status.overtemp_prewarning());
    assert!(!status.overtemp_shutdown());
    assert!(status.short_to_ground());
    assert!(!status.short_to_supply());
    assert!(status.standstill());
    assert_eq!(status.cs_actual(), 0x1F);
    // A short is a hard fault; a bare over-temperature pre-warning on its own is not.
    assert!(status.has_fault());
    assert!(!DrvStatus::from_raw(1 << 0).has_fault());
  }
}
