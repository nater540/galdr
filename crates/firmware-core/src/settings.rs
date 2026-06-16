//! Machine settings model, `$`-command logic, and persistence framing (DOC-04).
//!
//! This is the host-tested heart of the settings subsystem. It owns:
//! - [`Settings`] — one aggregate of every persisted `$n` value plus the TMC2209 driver parameters, with a
//!   [`Default`] composed from the existing `*Config::default()` impls so there is exactly ONE source of
//!   defaults.
//! - The `$n`↔field mapping: [`Settings::set_command`] (apply `$n=val`) and [`Settings::write_setting_line`]
//!   (render one `$n=value` line for `$$`), both driven off the single [`SETTING_NUMBERS`] authority.
//! - [`Settings::sanitized`] — range-clamping that guarantees a usable config even from a corrupt flash
//!   record or a bad host write (no zero steps/mm, no invalid microstepping, etc.).
//! - Conversions to the live runtime configs ([`Settings::planner_config`], [`motion_config`], [`tmc_config`],
//!   [`steps_per_mm`]) that replace the firmware's placeholder accessors.
//! - [`wire`] — protobuf encode/decode wrapped in a `MAGIC | version | len | payload | CRC32` storage frame,
//!   plus [`load_or_default`]/[`store_settings`] over the host-testable [`SettingsStore`] trait.
//!
//! The protobuf message ([`galdr_proto::Settings`]) is a pure wire/flash DTO reached only at the encode/decode
//! boundary; the planner/motion/tmc consumers keep taking their plain `*Config` structs, untouched.

use crate::drivers::tmc2209::manager::{AxisConfig, TmcConfig};
use crate::hal_traits::{SettingsStore, StoreError};
use crate::motion::MotionConfig;
use crate::planner::{PlannerConfig, AXES};

/// The UART node addresses strapped on the three TMC2209 drivers (X=0, Y=1, Z=2 per DOC-03). These are a
/// hardware property, not a user setting, so the `Settings → TmcConfig` conversion supplies them directly.
const TMC_NODES: [u8; AXES] = [0, 1, 2];

/// `$1` stepper idle lock delay default, milliseconds. grbl convention; 255 means "always energized".
const DEFAULT_STEP_IDLE_DELAY_MS: u32 = 25;
/// `$130–$132` per-axis maximum travel default, millimeters. A modest PCB-milling work envelope.
const DEFAULT_MAX_TRAVEL_MM: f32 = 200.0;
/// `$24` homing feed (locate) rate default, mm/min.
const DEFAULT_HOMING_FEED_MM_MIN: f32 = 100.0;
/// `$25` homing seek rate default, mm/min.
const DEFAULT_HOMING_SEEK_MM_MIN: f32 = 500.0;
/// `$26` homing switch debounce default, milliseconds.
const DEFAULT_HOMING_DEBOUNCE_MS: u32 = 25;
/// `$27` homing pull-off default, millimeters.
const DEFAULT_HOMING_PULLOFF_MM: f32 = 1.0;
/// `$30` maximum spindle RPM default (WS55-220 nominal).
const DEFAULT_SPINDLE_RPM_MAX: f32 = 12_000.0;

/// Largest RMS motor current the sanitizer will accept, milliamps (the TMC2209/Adafruit 6121 ceiling).
const MAX_MOTOR_CURRENT_MA: u16 = 2_000;

/// The valid `$n` settings this firmware exposes over `$$`/`$x=val`, in dump order, derived from the single
/// [`SETTING_DESCRIPTORS`] authority below. Standard grbl numbers plus the grblHAL-style TMC run-current
/// (`$140–142`) and microsteps (`$150–152`). The advanced TMC parameters (hold current, IHOLDDELAY,
/// TPOWERDOWN, TPWMTHRS, SENDDELAY, R_sense) are reachable only via the `$PBX` protobuf channel and defaults.
pub const SETTING_NUMBERS: &[u16] = &setting_numbers();

/// Build [`SETTING_NUMBERS`] from the descriptor table at compile time, expanding each per-axis group into its
/// three `$n` values so the dump order matches the descriptor order. This is what makes the descriptor table
/// the single authority: the dump list can never drift from the setter/formatter.
const fn setting_numbers() -> [u16; SETTING_COUNT] {
  let mut out = [0u16; SETTING_COUNT];
  let mut i = 0;
  let mut d = 0;
  while d < SETTING_DESCRIPTORS.len() {
    let desc = &SETTING_DESCRIPTORS[d];
    let mut axis = 0;
    let axes = desc.field.axis_span();
    while axis < axes {
      out[i] = desc.number + axis as u16;
      i += 1;
      axis += 1;
    }
    d += 1;
  }
  out
}

/// The total number of `$n` numbers, counting each per-axis group as its three axes. Computed once at compile
/// time so [`SETTING_NUMBERS`] can be a fixed-size array (no heap, no `static mut`).
const SETTING_COUNT: usize = count_setting_numbers();

const fn count_setting_numbers() -> usize {
  let mut count = 0;
  let mut d = 0;
  while d < SETTING_DESCRIPTORS.len() {
    count += SETTING_DESCRIPTORS[d].field.axis_span();
    d += 1;
  }
  count
}

/// Error applying a `$n=value` write. Carries the grblHAL `error:N` status code to return to the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SettingError {
  /// The `$n` number is not a setting this firmware supports.
  UnknownSetting,
  /// The value text could not be parsed as the setting's type.
  BadValue,
  /// The value parsed but fell outside the acceptable range.
  OutOfRange,
}

impl SettingError {
  /// The grblHAL `error:N` status code that best represents this failure (3 = unsupported `$` statement,
  /// 2 = bad numeric value — reused for out-of-range so a sender halts on either).
  pub fn code(self) -> u8 {
    match self {
      SettingError::UnknownSetting => 3,
      SettingError::BadValue | SettingError::OutOfRange => 2,
    }
  }
}

/// Every persisted machine setting in one aggregate. Plain typed fields (no protobuf optionals) so the
/// planner/motion/tmc consumers convert from it cheaply. `Copy` because it is all scalars and fixed arrays.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
  /// `$0` step pulse time, microseconds.
  pub step_pulse_us: u32,
  /// `$1` stepper idle lock delay, milliseconds (255 = always energized).
  pub step_idle_delay_ms: u32,
  /// `$2` step port invert bitmask.
  pub step_invert_mask: u8,
  /// `$3` direction port invert bitmask.
  pub dir_invert_mask: u8,
  /// `$10` status report field bitmask.
  pub status_report_mask: u8,
  /// `$11` junction deviation, millimeters.
  pub junction_deviation_mm: f32,
  /// `$12` arc chord tolerance, millimeters.
  pub arc_tolerance_mm: f32,
  /// `$20` soft limits enable.
  pub soft_limits_enable: bool,
  /// `$21` hard limits enable.
  pub hard_limits_enable: bool,
  /// `$22` homing cycle enable.
  pub homing_enable: bool,
  /// `$23` homing direction invert bitmask.
  pub homing_dir_invert_mask: u8,
  /// `$24` homing feed (locate) rate, mm/min.
  pub homing_feed_mm_min: f32,
  /// `$25` homing seek rate, mm/min.
  pub homing_seek_mm_min: f32,
  /// `$26` homing switch debounce, milliseconds.
  pub homing_debounce_ms: u32,
  /// `$27` homing pull-off distance, millimeters.
  pub homing_pulloff_mm: f32,
  /// `$30` maximum spindle speed, RPM.
  pub spindle_rpm_max: f32,
  /// `$31` minimum spindle speed, RPM.
  pub spindle_rpm_min: f32,
  /// `$100–$102` steps per millimeter, `[X, Y, Z]`.
  pub steps_per_mm: [f32; AXES],
  /// `$110–$112` maximum rate, mm/min, `[X, Y, Z]`.
  pub max_rate_mm_min: [f32; AXES],
  /// `$120–$122` acceleration, mm/s², `[X, Y, Z]`.
  pub accel_mm_s2: [f32; AXES],
  /// `$130–$132` maximum travel, millimeters, `[X, Y, Z]`.
  pub max_travel_mm: [f32; AXES],
  /// `$140–$142` RMS run current, milliamps, `[X, Y, Z]`.
  pub run_current_ma: [u16; AXES],
  /// `$150–$152` microstep resolution (power of two 1..=256), `[X, Y, Z]`.
  pub microsteps: [u16; AXES],
  /// RMS hold current, milliamps, `[X, Y, Z]` (host-sync channel / default only).
  pub hold_current_ma: [u16; AXES],
  /// TMC `IHOLD_IRUN.IHOLDDELAY` (host-sync channel / default only).
  pub tmc_ihold_delay: u8,
  /// TMC `TPOWERDOWN` (host-sync channel / default only).
  pub tmc_tpowerdown: u8,
  /// TMC `TPWMTHRS` StealthChop→SpreadCycle threshold (host-sync channel / default only).
  pub tmc_tpwmthrs: u32,
  /// TMC `SLAVECONF.SENDDELAY`, units of 8 bit-times (host-sync channel / default only).
  pub tmc_send_delay: u8,
  /// TMC sense-resistor value, ohms (host-sync channel / default only).
  pub tmc_r_sense_ohms: f32,
}

impl Default for Settings {
  /// Compose the defaults from the existing `*Config::default()` impls so the planner, motion executor, TMC
  /// manager, and `$$` dump all agree on one set of first-boot values; the limits/homing/spindle fields that
  /// have no `*Config` home yet take the grbl-conventional `DEFAULT_*` constants above.
  fn default() -> Self {
    let planner = PlannerConfig::default();
    let motion = MotionConfig::default();
    let tmc = TmcConfig::default();
    Settings {
      step_pulse_us: ticks_to_us(motion.step_pulse_ticks, motion.tick_hz),
      step_idle_delay_ms: DEFAULT_STEP_IDLE_DELAY_MS,
      step_invert_mask: 0,
      dir_invert_mask: 0,
      status_report_mask: 0,
      junction_deviation_mm: planner.junction_deviation_mm,
      arc_tolerance_mm: planner.arc_tolerance_mm,
      soft_limits_enable: false,
      hard_limits_enable: false,
      homing_enable: false,
      homing_dir_invert_mask: 0,
      homing_feed_mm_min: DEFAULT_HOMING_FEED_MM_MIN,
      homing_seek_mm_min: DEFAULT_HOMING_SEEK_MM_MIN,
      homing_debounce_ms: DEFAULT_HOMING_DEBOUNCE_MS,
      homing_pulloff_mm: DEFAULT_HOMING_PULLOFF_MM,
      spindle_rpm_max: DEFAULT_SPINDLE_RPM_MAX,
      spindle_rpm_min: 0.0,
      steps_per_mm: planner.steps_per_mm,
      max_rate_mm_min: planner.max_rate_mm_min,
      accel_mm_s2: planner.accel_mm_s2,
      max_travel_mm: [DEFAULT_MAX_TRAVEL_MM; AXES],
      run_current_ma: [tmc.axes[0].run_current_ma, tmc.axes[1].run_current_ma, tmc.axes[2].run_current_ma],
      microsteps: [tmc.axes[0].microsteps, tmc.axes[1].microsteps, tmc.axes[2].microsteps],
      hold_current_ma: [tmc.axes[0].hold_current_ma, tmc.axes[1].hold_current_ma, tmc.axes[2].hold_current_ma],
      tmc_ihold_delay: tmc.ihold_delay,
      tmc_tpowerdown: tmc.tpowerdown,
      tmc_tpwmthrs: tmc.tpwmthrs,
      tmc_send_delay: tmc.send_delay,
      tmc_r_sense_ohms: tmc.r_sense_ohms,
    }
  }
}

impl Settings {
  /// Build the [`PlannerConfig`] the look-ahead planner consumes from the current settings.
  pub fn planner_config(&self) -> PlannerConfig {
    PlannerConfig {
      steps_per_mm: self.steps_per_mm,
      max_rate_mm_min: self.max_rate_mm_min,
      accel_mm_s2: self.accel_mm_s2,
      junction_deviation_mm: self.junction_deviation_mm,
      arc_tolerance_mm: self.arc_tolerance_mm,
    }
  }

  /// Build the [`MotionConfig`] the step generator consumes. `tick_hz` is the firmware's RMT tick rate (not a
  /// user setting); `$0` µs is converted to step-pulse ticks at that rate. The minimum LOW time stays at the
  /// `MotionConfig` default (a fixed timing floor, not a `$n`).
  pub fn motion_config(&self, tick_hz: f32) -> MotionConfig {
    MotionConfig {
      tick_hz,
      step_pulse_ticks: us_to_ticks(self.step_pulse_us, tick_hz),
      min_low_ticks: MotionConfig::default().min_low_ticks,
    }
  }

  /// Build the [`TmcConfig`] the driver manager consumes, supplying the fixed node addresses.
  pub fn tmc_config(&self) -> TmcConfig {
    let axis = |i: usize| AxisConfig {
      node: TMC_NODES[i],
      run_current_ma: self.run_current_ma[i],
      hold_current_ma: self.hold_current_ma[i],
      microsteps: self.microsteps[i],
    };
    TmcConfig {
      axes: [axis(0), axis(1), axis(2)],
      r_sense_ohms: self.tmc_r_sense_ohms,
      ihold_delay: self.tmc_ihold_delay,
      tpowerdown: self.tmc_tpowerdown,
      tpwmthrs: self.tmc_tpwmthrs,
      send_delay: self.tmc_send_delay,
    }
  }

  /// The per-axis steps/mm, for the live-position (steps→mm) conversion in the status reporter.
  pub fn steps_per_mm(&self) -> [f32; AXES] {
    self.steps_per_mm
  }

  /// Apply a `$n=value` write, parsing and range-checking `value` for setting `n`. On success the field is
  /// updated (already validated within the same bounds [`sanitized`](Settings::sanitized) enforces); on
  /// failure nothing changes and a [`SettingError`] with a grblHAL code is returned. Both the lookup and the
  /// validation come from the single [`SETTING_DESCRIPTORS`] authority, so a write can never accept a value
  /// the sanitizer would later reject (e.g. `$0=0`, a zero-width step pulse).
  pub fn set_command(&mut self, n: u16, value: &str) -> Result<(), SettingError> {
    let value = value.trim();
    let (desc, axis) = lookup_descriptor(n).ok_or(SettingError::UnknownSetting)?;
    desc.field.parse_and_apply(self, axis, value)
  }

  /// Render the `$n=value` line for setting `n` into `out` (no trailing newline). Returns `true` if `n` is a
  /// known setting, `false` otherwise. `$$` iterates [`SETTING_NUMBERS`] and calls this for each, so the dump
  /// is driven by the same [`SETTING_DESCRIPTORS`] table as the setter and can never disagree with it.
  pub fn write_setting_line<const N: usize>(&self, n: u16, out: &mut heapless::String<N>) -> bool {
    match lookup_descriptor(n) {
      Some((desc, axis)) => desc.field.format(self, n, axis, out),
      None => false,
    }
  }

  /// Clamp every field to a safe, usable range so neither a corrupt flash record nor a bad host write can
  /// produce a config that wedges motion (zero/negative steps/mm, invalid microstepping, over-current, etc.).
  /// Called after any decode and before applying a bulk host write.
  pub fn sanitized(mut self) -> Self {
    let defaults = Settings::default();
    self.step_pulse_us = self.step_pulse_us.clamp(1, 1_000);
    self.junction_deviation_mm = non_negative_or(self.junction_deviation_mm, defaults.junction_deviation_mm);
    self.arc_tolerance_mm = positive_or(self.arc_tolerance_mm, defaults.arc_tolerance_mm);
    // A zero homing rate would never reach the switch and a zero max spindle RPM disables the spindle scale,
    // so these are must-be-positive; restore the default rather than accepting the protobuf/host zero. This is
    // also what makes the proto3 "absent ⇒ grbl default, not zero" contract true on decode (an absent field
    // arrives as 0 here, and `positive_or` repairs it). `spindle_rpm_min` legitimately allows 0.
    self.homing_feed_mm_min = positive_or(self.homing_feed_mm_min, defaults.homing_feed_mm_min);
    self.homing_seek_mm_min = positive_or(self.homing_seek_mm_min, defaults.homing_seek_mm_min);
    self.homing_pulloff_mm = self.homing_pulloff_mm.max(0.0);
    self.spindle_rpm_max = positive_or(self.spindle_rpm_max, defaults.spindle_rpm_max);
    self.spindle_rpm_min = self.spindle_rpm_min.max(0.0);
    for axis in 0..AXES {
      self.steps_per_mm[axis] = positive_or(self.steps_per_mm[axis], defaults.steps_per_mm[axis]);
      // A zero max rate makes the planner emit a zero-speed never-completing block, and a zero max travel
      // breaks soft limits — both must be positive, so restore the default instead of accepting 0.
      self.max_rate_mm_min[axis] = positive_or(self.max_rate_mm_min[axis], defaults.max_rate_mm_min[axis]);
      self.accel_mm_s2[axis] = positive_or(self.accel_mm_s2[axis], defaults.accel_mm_s2[axis]);
      self.max_travel_mm[axis] = positive_or(self.max_travel_mm[axis], defaults.max_travel_mm[axis]);
      self.run_current_ma[axis] = self.run_current_ma[axis].min(MAX_MOTOR_CURRENT_MA);
      self.hold_current_ma[axis] = self.hold_current_ma[axis].min(MAX_MOTOR_CURRENT_MA);
      self.microsteps[axis] = clamp_microsteps(self.microsteps[axis]);
    }
    // TMC SENDDELAY and IHOLDDELAY are packed into 4-bit register fields downstream (silently masked to
    // 0..=15), so clamp them here — otherwise the persisted value would differ from what gets programmed.
    // TPOWERDOWN (full u8) and TPWMTHRS (full u32) span their whole register field, so they need no clamp.
    self.tmc_send_delay = self.tmc_send_delay.min(15);
    self.tmc_ihold_delay = self.tmc_ihold_delay.min(15);
    self.tmc_r_sense_ohms = positive_or(self.tmc_r_sense_ohms, defaults.tmc_r_sense_ohms);
    self
  }
}

/// Convert a step-pulse time in timer ticks back to microseconds (for the `$0` default and `$$` dump).
fn ticks_to_us(ticks: u32, tick_hz: f32) -> u32 {
  let us = ticks as f32 * 1_000_000.0 / tick_hz;
  libm::roundf(us) as u32
}

/// Convert a `$0` step-pulse time in microseconds to timer ticks at `tick_hz` (for [`Settings::motion_config`]).
fn us_to_ticks(us: u32, tick_hz: f32) -> u32 {
  let ticks = us as f32 * tick_hz / 1_000_000.0;
  libm::roundf(ticks) as u32
}

/// Snap a microstep value to the nearest valid TMC2209 power-of-two resolution (1..=256), defaulting to 16 if
/// it is not one of those values. Guarantees the `Settings → TmcConfig` conversion always yields a programmable
/// `CHOPCONF.MRES`.
fn clamp_microsteps(microsteps: u16) -> u16 {
  match microsteps {
    1 | 2 | 4 | 8 | 16 | 32 | 64 | 128 | 256 => microsteps,
    _ => 16,
  }
}

/// Return `value` if it is strictly positive (and finite), else `fallback`.
fn positive_or(value: f32, fallback: f32) -> f32 {
  if value.is_finite() && value > 0.0 {
    value
  } else {
    fallback
  }
}

/// Return `value` if it is non-negative (and finite), else `fallback`.
fn non_negative_or(value: f32, fallback: f32) -> f32 {
  if value.is_finite() && value >= 0.0 {
    value
  } else {
    fallback
  }
}

fn parse_u32(value: &str) -> Result<u32, SettingError> {
  value.parse::<u32>().map_err(|_| SettingError::BadValue)
}

/// Parse a `$0` step-pulse time in microseconds, rejecting values outside the same 1..=1000 range the
/// sanitizer enforces so a write can never store a zero-width step pulse (no motion) or an absurdly long one.
fn parse_step_pulse_us(value: &str) -> Result<u32, SettingError> {
  let parsed = value.parse::<u32>().map_err(|_| SettingError::BadValue)?;
  if (1..=1_000).contains(&parsed) {
    Ok(parsed)
  } else {
    Err(SettingError::OutOfRange)
  }
}

fn parse_u8(value: &str) -> Result<u8, SettingError> {
  value.parse::<u8>().map_err(|_| SettingError::BadValue)
}

/// Parse a grbl boolean setting: `0` is false, any other valid integer is true (grbl treats non-zero as set).
fn parse_bool(value: &str) -> Result<bool, SettingError> {
  Ok(value.parse::<u32>().map_err(|_| SettingError::BadValue)? != 0)
}

fn parse_f32_non_negative(value: &str) -> Result<f32, SettingError> {
  let parsed = value.parse::<f32>().map_err(|_| SettingError::BadValue)?;
  if parsed.is_finite() && parsed >= 0.0 {
    Ok(parsed)
  } else {
    Err(SettingError::OutOfRange)
  }
}

fn parse_f32_positive(value: &str) -> Result<f32, SettingError> {
  let parsed = value.parse::<f32>().map_err(|_| SettingError::BadValue)?;
  if parsed.is_finite() && parsed > 0.0 {
    Ok(parsed)
  } else {
    Err(SettingError::OutOfRange)
  }
}

/// Parse an RMS motor current in milliamps, rejecting values above the driver ceiling so a typo cannot drive
/// the motors over their rated current.
fn parse_current_ma(value: &str) -> Result<u16, SettingError> {
  let parsed = value.parse::<u16>().map_err(|_| SettingError::BadValue)?;
  if parsed <= MAX_MOTOR_CURRENT_MA {
    Ok(parsed)
  } else {
    Err(SettingError::OutOfRange)
  }
}

/// Parse a microstep resolution, accepting only valid TMC2209 power-of-two values (1..=256).
fn parse_microsteps(value: &str) -> Result<u16, SettingError> {
  let parsed = value.parse::<u16>().map_err(|_| SettingError::BadValue)?;
  match parsed {
    1 | 2 | 4 | 8 | 16 | 32 | 64 | 128 | 256 => Ok(parsed),
    _ => Err(SettingError::OutOfRange),
  }
}

/// Names the [`Settings`] field a descriptor targets. A scalar variant points at one field; a per-axis
/// variant points at the `[X, Y, Z]` triple the consecutive `$n` numbers index. The variant alone determines
/// the setter's parse/validation policy, the formatter's dump format, AND the dump-number expansion (via
/// [`axis_span`](Field::axis_span)), so the one descriptor table cannot drift across the three operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Field {
  StepPulseUs,
  StepIdleDelayMs,
  StepInvertMask,
  DirInvertMask,
  StatusReportMask,
  JunctionDeviationMm,
  ArcToleranceMm,
  SoftLimitsEnable,
  HardLimitsEnable,
  HomingEnable,
  HomingDirInvertMask,
  HomingFeedMmMin,
  HomingSeekMmMin,
  HomingDebounceMs,
  HomingPulloffMm,
  SpindleRpmMax,
  SpindleRpmMin,
  StepsPerMm,
  MaxRateMmMin,
  AccelMmS2,
  MaxTravelMm,
  RunCurrentMa,
  Microsteps,
}

impl Field {
  /// How many consecutive `$n` numbers this field owns: 3 for a per-axis triple, 1 for a scalar. Drives the
  /// [`SETTING_NUMBERS`] expansion in a `const fn`, so it must itself be `const`.
  const fn axis_span(self) -> usize {
    match self {
      Field::StepsPerMm | Field::MaxRateMmMin | Field::AccelMmS2 | Field::MaxTravelMm | Field::RunCurrentMa
      | Field::Microsteps => AXES,
      _ => 1,
    }
  }

  /// Parse `value` per this field's validation policy and, only if it validates, write it into
  /// `settings` (at `axis` for per-axis triples). On any parse/range failure `settings` is left untouched and
  /// a [`SettingError`] is returned — so a rejected `$x=val` never partially applies.
  fn parse_and_apply(self, settings: &mut Settings, axis: usize, value: &str) -> Result<(), SettingError> {
    match self {
      Field::StepPulseUs => settings.step_pulse_us = parse_step_pulse_us(value)?,
      Field::StepIdleDelayMs => settings.step_idle_delay_ms = parse_u32(value)?,
      Field::StepInvertMask => settings.step_invert_mask = parse_u8(value)?,
      Field::DirInvertMask => settings.dir_invert_mask = parse_u8(value)?,
      Field::StatusReportMask => settings.status_report_mask = parse_u8(value)?,
      Field::JunctionDeviationMm => settings.junction_deviation_mm = parse_f32_non_negative(value)?,
      Field::ArcToleranceMm => settings.arc_tolerance_mm = parse_f32_positive(value)?,
      Field::SoftLimitsEnable => settings.soft_limits_enable = parse_bool(value)?,
      Field::HardLimitsEnable => settings.hard_limits_enable = parse_bool(value)?,
      Field::HomingEnable => settings.homing_enable = parse_bool(value)?,
      Field::HomingDirInvertMask => settings.homing_dir_invert_mask = parse_u8(value)?,
      Field::HomingFeedMmMin => settings.homing_feed_mm_min = parse_f32_positive(value)?,
      Field::HomingSeekMmMin => settings.homing_seek_mm_min = parse_f32_positive(value)?,
      Field::HomingDebounceMs => settings.homing_debounce_ms = parse_u32(value)?,
      Field::HomingPulloffMm => settings.homing_pulloff_mm = parse_f32_non_negative(value)?,
      Field::SpindleRpmMax => settings.spindle_rpm_max = parse_f32_positive(value)?,
      Field::SpindleRpmMin => settings.spindle_rpm_min = parse_f32_non_negative(value)?,
      Field::StepsPerMm => settings.steps_per_mm[axis] = parse_f32_positive(value)?,
      Field::MaxRateMmMin => settings.max_rate_mm_min[axis] = parse_f32_positive(value)?,
      Field::AccelMmS2 => settings.accel_mm_s2[axis] = parse_f32_positive(value)?,
      Field::MaxTravelMm => settings.max_travel_mm[axis] = parse_f32_positive(value)?,
      Field::RunCurrentMa => settings.run_current_ma[axis] = parse_current_ma(value)?,
      Field::Microsteps => settings.microsteps[axis] = parse_microsteps(value)?,
    }
    Ok(())
  }

  /// Render the `$n=value` dump line for this field into `out` (no trailing newline), using the same `{:.3}`
  /// float / plain-integer formats grbl senders expect. Returns `true` on success, `false` only if `out` lacks
  /// capacity — which cannot happen with the caller's correctly sized buffer.
  fn format<const N: usize>(self, settings: &Settings, n: u16, axis: usize, out: &mut heapless::String<N>) -> bool {
    use core::fmt::Write;
    let result = match self {
      Field::StepPulseUs => write!(out, "${}={}", n, settings.step_pulse_us),
      Field::StepIdleDelayMs => write!(out, "${}={}", n, settings.step_idle_delay_ms),
      Field::StepInvertMask => write!(out, "${}={}", n, settings.step_invert_mask),
      Field::DirInvertMask => write!(out, "${}={}", n, settings.dir_invert_mask),
      Field::StatusReportMask => write!(out, "${}={}", n, settings.status_report_mask),
      Field::JunctionDeviationMm => write!(out, "${}={:.3}", n, settings.junction_deviation_mm),
      Field::ArcToleranceMm => write!(out, "${}={:.3}", n, settings.arc_tolerance_mm),
      Field::SoftLimitsEnable => write!(out, "${}={}", n, settings.soft_limits_enable as u8),
      Field::HardLimitsEnable => write!(out, "${}={}", n, settings.hard_limits_enable as u8),
      Field::HomingEnable => write!(out, "${}={}", n, settings.homing_enable as u8),
      Field::HomingDirInvertMask => write!(out, "${}={}", n, settings.homing_dir_invert_mask),
      Field::HomingFeedMmMin => write!(out, "${}={:.3}", n, settings.homing_feed_mm_min),
      Field::HomingSeekMmMin => write!(out, "${}={:.3}", n, settings.homing_seek_mm_min),
      Field::HomingDebounceMs => write!(out, "${}={}", n, settings.homing_debounce_ms),
      Field::HomingPulloffMm => write!(out, "${}={:.3}", n, settings.homing_pulloff_mm),
      Field::SpindleRpmMax => write!(out, "${}={:.3}", n, settings.spindle_rpm_max),
      Field::SpindleRpmMin => write!(out, "${}={:.3}", n, settings.spindle_rpm_min),
      Field::StepsPerMm => write!(out, "${}={:.3}", n, settings.steps_per_mm[axis]),
      Field::MaxRateMmMin => write!(out, "${}={:.3}", n, settings.max_rate_mm_min[axis]),
      Field::AccelMmS2 => write!(out, "${}={:.3}", n, settings.accel_mm_s2[axis]),
      Field::MaxTravelMm => write!(out, "${}={:.3}", n, settings.max_travel_mm[axis]),
      Field::RunCurrentMa => write!(out, "${}={}", n, settings.run_current_ma[axis]),
      Field::Microsteps => write!(out, "${}={}", n, settings.microsteps[axis]),
    };
    result.is_ok()
  }
}

/// One row of the single `$n`→field authority: the base `$n` number and the [`Field`] it drives. A per-axis
/// field owns the three consecutive numbers `number..number+3`; a scalar owns just `number`.
struct SettingDescriptor {
  number: u16,
  field: Field,
}

/// The single source of truth mapping every `$n` setting to its field, value policy, and dump format.
/// [`SETTING_NUMBERS`], [`Settings::set_command`], and [`Settings::write_setting_line`] are ALL derived from
/// this table, so the dump list, the setter, and the formatter can never disagree. Per-axis groups are one
/// row each (expanded to their three `$n` numbers), not six copy-pasted arms. Order here is `$$` dump order.
const SETTING_DESCRIPTORS: &[SettingDescriptor] = &[
  SettingDescriptor { number: 0, field: Field::StepPulseUs },
  SettingDescriptor { number: 1, field: Field::StepIdleDelayMs },
  SettingDescriptor { number: 2, field: Field::StepInvertMask },
  SettingDescriptor { number: 3, field: Field::DirInvertMask },
  SettingDescriptor { number: 10, field: Field::StatusReportMask },
  SettingDescriptor { number: 11, field: Field::JunctionDeviationMm },
  SettingDescriptor { number: 12, field: Field::ArcToleranceMm },
  SettingDescriptor { number: 20, field: Field::SoftLimitsEnable },
  SettingDescriptor { number: 21, field: Field::HardLimitsEnable },
  SettingDescriptor { number: 22, field: Field::HomingEnable },
  SettingDescriptor { number: 23, field: Field::HomingDirInvertMask },
  SettingDescriptor { number: 24, field: Field::HomingFeedMmMin },
  SettingDescriptor { number: 25, field: Field::HomingSeekMmMin },
  SettingDescriptor { number: 26, field: Field::HomingDebounceMs },
  SettingDescriptor { number: 27, field: Field::HomingPulloffMm },
  SettingDescriptor { number: 30, field: Field::SpindleRpmMax },
  SettingDescriptor { number: 31, field: Field::SpindleRpmMin },
  SettingDescriptor { number: 100, field: Field::StepsPerMm },
  SettingDescriptor { number: 110, field: Field::MaxRateMmMin },
  SettingDescriptor { number: 120, field: Field::AccelMmS2 },
  SettingDescriptor { number: 130, field: Field::MaxTravelMm },
  SettingDescriptor { number: 140, field: Field::RunCurrentMa },
  SettingDescriptor { number: 150, field: Field::Microsteps },
];

/// Find the descriptor owning `$n` and the axis index (0 for a scalar, 0..AXES for a per-axis triple) `n`
/// addresses within it. Returns `None` for an unknown number. This is the one lookup all three derived
/// operations share.
fn lookup_descriptor(n: u16) -> Option<(&'static SettingDescriptor, usize)> {
  for desc in SETTING_DESCRIPTORS {
    let span = desc.field.axis_span() as u16;
    if n >= desc.number && n < desc.number + span {
      return Some((desc, (n - desc.number) as usize));
    }
  }
  None
}

/// Load the persisted settings, falling back to [`Settings::default`] on absence OR any decode failure. This
/// is the INFALLIBLE loader (DOC-04): a corrupt, truncated, version-skewed, or missing record can never wedge
/// boot — it silently yields defaults, which the firmware then applies. The decoded record is sanitized.
pub async fn load_or_default<S: SettingsStore>(store: &mut S) -> Settings {
  let mut buf = [0u8; wire::FRAME_MAX_LEN];
  match store.load(&mut buf).await {
    Ok(len) => wire::decode(&buf[..len]).unwrap_or_default(),
    Err(_) => Settings::default(),
  }
}

/// Encode `settings` into a storage frame and persist it via `store`, replacing any prior record. Surfaces
/// encode/store failures to the caller (a failed `$x=val` persist should still `ok` the line — the in-RAM
/// value applied — but the firmware logs the failure).
pub async fn store_settings<S: SettingsStore>(store: &mut S, settings: &Settings) -> Result<(), StoreError> {
  let mut frame: heapless::Vec<u8, { wire::FRAME_MAX_LEN }> = heapless::Vec::new();
  // An encode failure here would only happen if the frame buffer were undersized, which the const sizing
  // rules out; map it to a storage `TooLarge` rather than introducing a separate error type at this boundary.
  wire::encode(settings, &mut frame).map_err(|_| StoreError::TooLarge)?;
  store.save(&frame).await
}

/// A hex-codec failure: odd input length, a non-hex digit, or output capacity exceeded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HexError;

/// Append the uppercase hex encoding of `bytes` to `out`. Returns `false` if `out` runs out of capacity. Used
/// to render a settings frame as 7-bit-clean text for the `$PBX` host-sync channel (so it never collides with
/// the line-based grbl protocol or the real-time byte interception).
pub fn write_hex<const N: usize>(bytes: &[u8], out: &mut heapless::String<N>) -> bool {
  const DIGITS: &[u8; 16] = b"0123456789ABCDEF";
  for &byte in bytes {
    if out.push(DIGITS[(byte >> 4) as usize] as char).is_err() {
      return false;
    }
    if out.push(DIGITS[(byte & 0x0F) as usize] as char).is_err() {
      return false;
    }
  }
  true
}

/// Decode the hex text `hex` (an even number of `[0-9a-fA-F]` digits), appending the bytes to `out`. Returns
/// [`HexError`] on an odd length, a non-hex digit, or insufficient `out` capacity.
pub fn decode_hex<const N: usize>(hex: &str, out: &mut heapless::Vec<u8, N>) -> Result<(), HexError> {
  let bytes = hex.as_bytes();
  if !bytes.len().is_multiple_of(2) {
    return Err(HexError);
  }
  let mut index = 0;
  while index < bytes.len() {
    let high = hex_nibble(bytes[index])?;
    let low = hex_nibble(bytes[index + 1])?;
    out.push((high << 4) | low).map_err(|_| HexError)?;
    index += 2;
  }
  Ok(())
}

/// Decode a single hex digit to its 0..=15 value.
fn hex_nibble(c: u8) -> Result<u8, HexError> {
  match c {
    b'0'..=b'9' => Ok(c - b'0'),
    b'a'..=b'f' => Ok(c - b'a' + 10),
    b'A'..=b'F' => Ok(c - b'A' + 10),
    _ => Err(HexError),
  }
}

/// The outcome of feeding one hex chunk to a [`PbReceiver`]. Not `Eq`/`defmt::Format` because the
/// `Complete` variant carries [`Settings`] (which holds `f32` fields).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PbChunkResult {
  /// The chunk was accepted; more chunks are needed before the frame is complete.
  NeedMore,
  /// The full frame arrived, decoded, and validated; here are the resulting (sanitized) settings.
  Complete(Settings),
  /// The accumulated data was malformed (bad hex, bad header, overrun, or a failed CRC/payload). The receiver
  /// has reset itself; the caller should report an error and the host may retry from the first chunk.
  Error,
}

/// Reassembles a settings frame from the hex chunks of a `$PBX=<hex>` host-sync write, which spans several
/// lines because a full frame exceeds the protocol line length. It accumulates decoded bytes and, using the
/// self-describing frame header (magic + version + length), recognizes completion on its own — so no chunk
/// indices or counts are needed on the wire; the frame's own CRC validates the result. Host-tested.
#[derive(Default)]
pub struct PbReceiver {
  buf: heapless::Vec<u8, { wire::FRAME_MAX_LEN }>,
}

impl PbReceiver {
  /// A fresh receiver with an empty buffer.
  pub fn new() -> Self {
    PbReceiver { buf: heapless::Vec::new() }
  }

  /// Discard any partially accumulated frame (e.g. on a soft reset).
  pub fn reset(&mut self) {
    self.buf.clear();
  }

  /// Feed one hex chunk. Appends its bytes to the accumulator, then reports whether the frame is now complete
  /// (decoding and validating it), still incomplete, or malformed. On [`PbChunkResult::Error`] or
  /// [`PbChunkResult::Complete`] the buffer is reset, ready for the next frame.
  pub fn accept_hex(&mut self, hex: &str) -> PbChunkResult {
    if decode_hex(hex, &mut self.buf).is_err() {
      self.buf.clear();
      return PbChunkResult::Error;
    }
    match wire::frame_progress(&self.buf) {
      wire::FrameProgress::NeedMore => PbChunkResult::NeedMore,
      wire::FrameProgress::BadHeader => {
        self.buf.clear();
        PbChunkResult::Error
      }
      wire::FrameProgress::Total(total) => {
        if self.buf.len() < total {
          PbChunkResult::NeedMore
        } else {
          // The buffer now holds at least a full frame; decode exactly that many bytes and reset. A length
          // overrun (more than the declared frame) or a CRC/payload failure is an error.
          let result = if self.buf.len() == total { wire::decode(&self.buf) } else { Err(wire::CodecError::BadLength) };
          self.buf.clear();
          match result {
            Ok(settings) => PbChunkResult::Complete(settings),
            Err(_) => PbChunkResult::Error,
          }
        }
      }
    }
  }
}

/// Protobuf encode/decode wrapped in a versioned, CRC-checked storage frame.
///
/// Frame layout (little-endian): `MAGIC(4) | SCHEMA_VERSION(1) | payload_len(2) | protobuf payload | CRC32(4)`.
/// The magic distinguishes a galdr record from arbitrary flash bytes, the version sentinel rejects a record
/// written by an incompatible build, and the CRC32 catches corruption / torn writes — so [`decode`] fails
/// cleanly (and the loader falls back to defaults) rather than feeding garbage into the planner.
pub mod wire {
  use super::Settings;
  use crate::planner::AXES;

  /// Frame magic, ASCII "GdS1" — identifies a galdr settings record.
  const MAGIC: u32 = 0x4764_5331;
  /// On-flash schema version, deliberately held at 1 during early development. We do NOT bump it when merely
  /// ADDING protobuf fields: proto3 is additively compatible, so an old record's absent new fields decode to
  /// their type zero and `positive_or` in [`Settings::sanitized`] restores the proper grbl default. The
  /// version is reserved for a genuinely incompatible layout change (a renumbered/retyped/removed field);
  /// bumping it then makes [`decode`] reject the stale record so the loader restores defaults.
  const SCHEMA_VERSION: u8 = 1;
  /// Bytes of fixed framing overhead around the protobuf payload (magic 4 + version 1 + len 2 + CRC 4).
  const FRAME_OVERHEAD: usize = 11;

  /// Maximum framed-record length, sized to the largest protobuf payload plus the framing overhead. Buffers
  /// for load/save and the `$PBX` channel are sized from this.
  pub const FRAME_MAX_LEN: usize = galdr_proto::SETTINGS_MAX_LEN + FRAME_OVERHEAD;

  /// Failure encoding or decoding a settings storage frame.
  #[derive(Debug, Clone, Copy, PartialEq, Eq)]
  #[cfg_attr(feature = "defmt", derive(defmt::Format))]
  pub enum CodecError {
    /// The destination buffer could not hold the encoded frame.
    BufferFull,
    /// The frame was shorter than the fixed overhead, or its declared payload length did not fit.
    BadLength,
    /// The leading magic did not match a galdr settings record.
    BadMagic,
    /// The schema version did not match this build's [`SCHEMA_VERSION`].
    BadVersion,
    /// The trailing CRC32 did not match the computed checksum (corruption / torn write).
    BadCrc,
    /// The protobuf payload was malformed.
    BadPayload,
  }

  /// Encode `settings` into `out` as a complete storage frame (the buffer is cleared first). Builds the
  /// protobuf payload from the settings, writes the magic/version/length header, appends the payload, then
  /// appends a CRC32 over everything preceding it.
  pub fn encode<const N: usize>(settings: &Settings, out: &mut heapless::Vec<u8, N>) -> Result<(), CodecError> {
    out.clear();
    let proto = settings.to_proto();
    let payload_len = galdr_proto::settings_size(&proto);
    if payload_len > u16::MAX as usize {
      return Err(CodecError::BadLength);
    }
    push_slice(out, &MAGIC.to_le_bytes())?;
    push_byte(out, SCHEMA_VERSION)?;
    push_slice(out, &(payload_len as u16).to_le_bytes())?;
    galdr_proto::encode_settings_into(&proto, out).map_err(|_| CodecError::BufferFull)?;
    let crc = crc32(out.as_slice());
    push_slice(out, &crc.to_le_bytes())?;
    Ok(())
  }

  /// Decode and validate a storage frame, returning the sanitized [`Settings`]. Verifies the magic, schema
  /// version, declared length, and CRC32 before decoding the protobuf payload; any mismatch is an error so
  /// the infallible loader falls back to defaults rather than trusting a damaged record.
  pub fn decode(frame: &[u8]) -> Result<Settings, CodecError> {
    if frame.len() < FRAME_OVERHEAD {
      return Err(CodecError::BadLength);
    }
    let magic = u32::from_le_bytes([frame[0], frame[1], frame[2], frame[3]]);
    if magic != MAGIC {
      return Err(CodecError::BadMagic);
    }
    if frame[4] != SCHEMA_VERSION {
      return Err(CodecError::BadVersion);
    }
    let payload_len = u16::from_le_bytes([frame[5], frame[6]]) as usize;
    let payload_end = 7 + payload_len;
    if frame.len() != payload_end + 4 {
      return Err(CodecError::BadLength);
    }
    let expected_crc = u32::from_le_bytes([frame[payload_end], frame[payload_end + 1], frame[payload_end + 2], frame[payload_end + 3]]);
    if crc32(&frame[..payload_end]) != expected_crc {
      return Err(CodecError::BadCrc);
    }
    let proto = galdr_proto::decode_settings(&frame[7..payload_end]).map_err(|_| CodecError::BadPayload)?;
    Ok(Settings::from_proto(&proto).sanitized())
  }

  /// How much of a settings frame a partial byte buffer represents, for the chunked `$PBX` host-sync write
  /// path (which accumulates a frame across several lines).
  #[derive(Debug, Clone, Copy, PartialEq, Eq)]
  pub enum FrameProgress {
    /// Fewer than the fixed-header bytes have arrived; the total length is not yet known.
    NeedMore,
    /// The header is present but malformed (bad magic/version, or a declared length that cannot be valid).
    BadHeader,
    /// The header is valid; the complete frame is exactly this many bytes long.
    Total(usize),
  }

  /// Inspect the leading bytes of an accumulating frame: report the full frame length once enough header is
  /// present and valid, so a chunked receiver knows when it has the whole record. Does NOT verify the CRC or
  /// payload — that is [`decode`]'s job once the full frame is assembled.
  pub fn frame_progress(buf: &[u8]) -> FrameProgress {
    if buf.len() < 7 {
      return FrameProgress::NeedMore;
    }
    let magic = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
    if magic != MAGIC || buf[4] != SCHEMA_VERSION {
      return FrameProgress::BadHeader;
    }
    let payload_len = u16::from_le_bytes([buf[5], buf[6]]) as usize;
    let total = 7 + payload_len + 4;
    if total > FRAME_MAX_LEN {
      return FrameProgress::BadHeader;
    }
    FrameProgress::Total(total)
  }

  fn push_byte<const N: usize>(out: &mut heapless::Vec<u8, N>, byte: u8) -> Result<(), CodecError> {
    out.push(byte).map_err(|_| CodecError::BufferFull)
  }

  fn push_slice<const N: usize>(out: &mut heapless::Vec<u8, N>, bytes: &[u8]) -> Result<(), CodecError> {
    out.extend_from_slice(bytes).map_err(|_| CodecError::BufferFull)
  }

  /// CRC-32 (IEEE 802.3, reflected, poly 0xEDB88320) over `data`. Hand-rolled to keep firmware-core's
  /// dependency set unchanged, mirroring the hand-rolled CRC8 in the TMC2209 codec.
  fn crc32(data: &[u8]) -> u32 {
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

  impl Settings {
    /// Project the typed settings onto the protobuf wire/flash DTO (the inverse of [`from_proto`]). Built by
    /// mutating a `default()` so a field added to the generated message is zero-initialized rather than a hard
    /// compile break here; the per-field assignments below set every field this firmware knows.
    #[allow(clippy::field_reassign_with_default)]
    pub(crate) fn to_proto(self) -> galdr_proto::Settings {
      let mut proto = galdr_proto::Settings::default();
      proto.step_pulse_us = self.step_pulse_us;
      proto.step_idle_delay_ms = self.step_idle_delay_ms;
      proto.step_invert_mask = self.step_invert_mask as u32;
      proto.dir_invert_mask = self.dir_invert_mask as u32;
      proto.status_report_mask = self.status_report_mask as u32;
      proto.junction_deviation_mm = self.junction_deviation_mm;
      proto.arc_tolerance_mm = self.arc_tolerance_mm;
      proto.soft_limits_enable = self.soft_limits_enable;
      proto.hard_limits_enable = self.hard_limits_enable;
      proto.homing_enable = self.homing_enable;
      proto.homing_dir_invert_mask = self.homing_dir_invert_mask as u32;
      proto.homing_feed_mm_min = self.homing_feed_mm_min;
      proto.homing_seek_mm_min = self.homing_seek_mm_min;
      proto.homing_debounce_ms = self.homing_debounce_ms;
      proto.homing_pulloff_mm = self.homing_pulloff_mm;
      proto.spindle_rpm_max = self.spindle_rpm_max;
      proto.spindle_rpm_min = self.spindle_rpm_min;
      proto.steps_per_mm_x = self.steps_per_mm[0];
      proto.steps_per_mm_y = self.steps_per_mm[1];
      proto.steps_per_mm_z = self.steps_per_mm[2];
      proto.max_rate_mm_min_x = self.max_rate_mm_min[0];
      proto.max_rate_mm_min_y = self.max_rate_mm_min[1];
      proto.max_rate_mm_min_z = self.max_rate_mm_min[2];
      proto.accel_mm_s2_x = self.accel_mm_s2[0];
      proto.accel_mm_s2_y = self.accel_mm_s2[1];
      proto.accel_mm_s2_z = self.accel_mm_s2[2];
      proto.max_travel_mm_x = self.max_travel_mm[0];
      proto.max_travel_mm_y = self.max_travel_mm[1];
      proto.max_travel_mm_z = self.max_travel_mm[2];
      proto.run_current_ma_x = self.run_current_ma[0] as u32;
      proto.run_current_ma_y = self.run_current_ma[1] as u32;
      proto.run_current_ma_z = self.run_current_ma[2] as u32;
      proto.hold_current_ma_x = self.hold_current_ma[0] as u32;
      proto.hold_current_ma_y = self.hold_current_ma[1] as u32;
      proto.hold_current_ma_z = self.hold_current_ma[2] as u32;
      proto.microsteps_x = self.microsteps[0] as u32;
      proto.microsteps_y = self.microsteps[1] as u32;
      proto.microsteps_z = self.microsteps[2] as u32;
      proto.tmc_ihold_delay = self.tmc_ihold_delay as u32;
      proto.tmc_tpowerdown = self.tmc_tpowerdown as u32;
      proto.tmc_tpwmthrs = self.tmc_tpwmthrs;
      proto.tmc_send_delay = self.tmc_send_delay as u32;
      proto.tmc_r_sense_ohms = self.tmc_r_sense_ohms;
      proto
    }

    /// Build the typed settings from the protobuf DTO (the inverse of [`to_proto`]). Mask/current/microstep
    /// fields narrow from the protobuf `u32` back to their typed widths using SATURATING conversions, never a
    /// truncating `as`: a wire `microsteps = 65552` must NOT wrap to `16` (a plausible-but-wrong "valid"
    /// value) — it saturates to `u16::MAX`, then the [`sanitized`](Settings::sanitized) pass maps the
    /// out-of-range microstep count to the default and clamps the over-range currents/masks uniformly.
    pub(crate) fn from_proto(proto: &galdr_proto::Settings) -> Settings {
      let _ = AXES; // The X/Y/Z triples below assume AXES == 3, asserted once at compile time below.
      Settings {
        step_pulse_us: proto.step_pulse_us,
        step_idle_delay_ms: proto.step_idle_delay_ms,
        step_invert_mask: saturating_u8(proto.step_invert_mask),
        dir_invert_mask: saturating_u8(proto.dir_invert_mask),
        status_report_mask: saturating_u8(proto.status_report_mask),
        junction_deviation_mm: proto.junction_deviation_mm,
        arc_tolerance_mm: proto.arc_tolerance_mm,
        soft_limits_enable: proto.soft_limits_enable,
        hard_limits_enable: proto.hard_limits_enable,
        homing_enable: proto.homing_enable,
        homing_dir_invert_mask: saturating_u8(proto.homing_dir_invert_mask),
        homing_feed_mm_min: proto.homing_feed_mm_min,
        homing_seek_mm_min: proto.homing_seek_mm_min,
        homing_debounce_ms: proto.homing_debounce_ms,
        homing_pulloff_mm: proto.homing_pulloff_mm,
        spindle_rpm_max: proto.spindle_rpm_max,
        spindle_rpm_min: proto.spindle_rpm_min,
        steps_per_mm: [proto.steps_per_mm_x, proto.steps_per_mm_y, proto.steps_per_mm_z],
        max_rate_mm_min: [proto.max_rate_mm_min_x, proto.max_rate_mm_min_y, proto.max_rate_mm_min_z],
        accel_mm_s2: [proto.accel_mm_s2_x, proto.accel_mm_s2_y, proto.accel_mm_s2_z],
        max_travel_mm: [proto.max_travel_mm_x, proto.max_travel_mm_y, proto.max_travel_mm_z],
        run_current_ma: [
          saturating_u16(proto.run_current_ma_x),
          saturating_u16(proto.run_current_ma_y),
          saturating_u16(proto.run_current_ma_z),
        ],
        microsteps: [
          saturating_u16(proto.microsteps_x),
          saturating_u16(proto.microsteps_y),
          saturating_u16(proto.microsteps_z),
        ],
        hold_current_ma: [
          saturating_u16(proto.hold_current_ma_x),
          saturating_u16(proto.hold_current_ma_y),
          saturating_u16(proto.hold_current_ma_z),
        ],
        tmc_ihold_delay: saturating_u8(proto.tmc_ihold_delay),
        tmc_tpowerdown: saturating_u8(proto.tmc_tpowerdown),
        tmc_tpwmthrs: proto.tmc_tpwmthrs,
        tmc_send_delay: saturating_u8(proto.tmc_send_delay),
        tmc_r_sense_ohms: proto.tmc_r_sense_ohms,
      }
    }
  }

  /// Narrow a protobuf `u32` field to `u16`, saturating at `u16::MAX` rather than truncating, so an
  /// out-of-range wire value lands above any valid range (and is then clamped/defaulted by `sanitized`)
  /// instead of wrapping to a plausible-but-wrong in-range value.
  fn saturating_u16(value: u32) -> u16 {
    value.min(u16::MAX as u32) as u16
  }

  /// Narrow a protobuf `u32` field to `u8`, saturating at `u8::MAX` rather than truncating (same rationale as
  /// [`saturating_u16`]).
  fn saturating_u8(value: u32) -> u8 {
    value.min(u8::MAX as u32) as u8
  }

  // The X/Y/Z triple conversions assume exactly three axes; fail the build loudly if that ever changes.
  const _: () = assert!(AXES == 3, "settings wire conversions assume AXES == 3");
}

#[cfg(test)]
mod tests {
  use super::*;

  /// An in-memory [`SettingsStore`] for host tests: holds the last saved frame, like the byte-buffer mocks
  /// used for `TmcBus`/`StepSink`. Empty until something is saved (so `load` reports `NotFound`).
  #[derive(Default)]
  struct MockStore {
    record: Option<heapless::Vec<u8, { wire::FRAME_MAX_LEN }>>,
  }

  impl SettingsStore for MockStore {
    async fn load(&mut self, buf: &mut [u8]) -> Result<usize, StoreError> {
      match &self.record {
        Some(record) => {
          if record.len() > buf.len() {
            return Err(StoreError::TooLarge);
          }
          buf[..record.len()].copy_from_slice(record);
          Ok(record.len())
        }
        None => Err(StoreError::NotFound),
      }
    }

    async fn save(&mut self, frame: &[u8]) -> Result<(), StoreError> {
      let mut record = heapless::Vec::new();
      record.extend_from_slice(frame).map_err(|_| StoreError::TooLarge)?;
      self.record = Some(record);
      Ok(())
    }
  }

  /// Drive an immediately-ready future to completion with the no-op waker. The store futures here never pend
  /// (no real I/O), so a single poll resolves them — this keeps the tests off any async runtime / dev-dep, and
  /// `Waker::noop` avoids hand-rolling a `RawWaker` (which `#![deny(unsafe_code)]` forbids).
  fn block_on<F: core::future::Future>(future: F) -> F::Output {
    use core::task::{Context, Poll, Waker};
    let mut context = Context::from_waker(Waker::noop());
    let mut future = core::pin::pin!(future);
    match future.as_mut().poll(&mut context) {
      Poll::Ready(output) => output,
      Poll::Pending => panic!("store future pended; the host MockStore must resolve in one poll"),
    }
  }

  #[test]
  fn defaults_match_each_config_default() {
    let settings = Settings::default();
    assert_eq!(settings.planner_config(), PlannerConfig::default());
    assert_eq!(settings.tmc_config(), TmcConfig::default());
    // $0 default round-trips through the µs↔ticks conversion at the default tick rate.
    let motion = settings.motion_config(MotionConfig::default().tick_hz);
    assert_eq!(motion, MotionConfig::default());
  }

  #[test]
  fn setting_numbers_are_all_writable_and_dumpable() {
    let mut settings = Settings::default();
    for &n in SETTING_NUMBERS {
      // Every advertised number must render a dump line and accept a write of its own rendered value.
      let mut line: heapless::String<32> = heapless::String::new();
      assert!(settings.write_setting_line(n, &mut line), "no dump line for $n={n}");
      let value = line.split('=').nth(1).expect("dump line has a value");
      settings.set_command(n, value).unwrap_or_else(|e| panic!("$n={n} rejected its own value {value:?}: {e:?}"));
    }
  }

  #[test]
  fn set_command_applies_and_round_trips() {
    let mut settings = Settings::default();
    settings.set_command(100, "320.5").expect("set $100");
    assert_eq!(settings.steps_per_mm[0], 320.5);
    settings.set_command(22, "1").expect("set $22");
    assert!(settings.homing_enable);
    settings.set_command(0, "5").expect("set $0");
    assert_eq!(settings.step_pulse_us, 5);
  }

  #[test]
  fn set_command_rejects_bad_input() {
    let mut settings = Settings::default();
    assert_eq!(settings.set_command(999, "1"), Err(SettingError::UnknownSetting));
    assert_eq!(settings.set_command(100, "abc"), Err(SettingError::BadValue));
    assert_eq!(settings.set_command(100, "-5"), Err(SettingError::OutOfRange));
    assert_eq!(settings.set_command(150, "7"), Err(SettingError::OutOfRange)); // not a power of two.
    assert_eq!(settings.set_command(140, "9000"), Err(SettingError::OutOfRange)); // over-current.
    // A rejected write leaves the field untouched.
    assert_eq!(settings.steps_per_mm[0], Settings::default().steps_per_mm[0]);
  }

  #[test]
  fn set_command_rejects_out_of_range_must_be_positive() {
    let mut settings = Settings::default();
    // `$0=0` is a zero-width step pulse (no motion) — must be rejected, and the field left at its default.
    assert_eq!(settings.set_command(0, "0"), Err(SettingError::OutOfRange));
    assert_eq!(settings.step_pulse_us, Settings::default().step_pulse_us);
    // `$0` above the 1..=1000 sanitizer range is rejected too.
    assert_eq!(settings.set_command(0, "2000"), Err(SettingError::OutOfRange));
    // A max-rate / max-travel of 0 wedges the planner / soft limits, so those reject 0 as well.
    assert_eq!(settings.set_command(110, "0"), Err(SettingError::OutOfRange));
    assert_eq!(settings.set_command(130, "0"), Err(SettingError::OutOfRange));
    assert_eq!(settings.max_rate_mm_min[0], Settings::default().max_rate_mm_min[0]);
    // A valid in-range `$0` still applies.
    settings.set_command(0, "5").expect("set $0=5");
    assert_eq!(settings.step_pulse_us, 5);
  }

  #[test]
  fn set_command_stays_within_sanitizer_bounds() {
    // Every accepted write must already satisfy `sanitized()`, i.e. sanitizing afterward changes nothing.
    let mut settings = Settings::default();
    settings.set_command(0, "1").expect("set $0=1");
    settings.set_command(110, "1.0").expect("set $110");
    settings.set_command(130, "0.5").expect("set $130");
    assert_eq!(settings, settings.sanitized());
  }

  #[test]
  fn sanitized_repairs_unusable_values() {
    let mut settings = Settings::default();
    settings.steps_per_mm[1] = 0.0;
    settings.accel_mm_s2[2] = -1.0;
    settings.microsteps[0] = 7;
    settings.run_current_ma[0] = 9_000;
    settings.arc_tolerance_mm = 0.0;
    let fixed = settings.sanitized();
    assert_eq!(fixed.steps_per_mm[1], Settings::default().steps_per_mm[1]);
    assert_eq!(fixed.accel_mm_s2[2], Settings::default().accel_mm_s2[2]);
    assert_eq!(fixed.microsteps[0], 16);
    assert_eq!(fixed.run_current_ma[0], MAX_MOTOR_CURRENT_MA);
    assert_eq!(fixed.arc_tolerance_mm, Settings::default().arc_tolerance_mm);
  }

  #[test]
  fn sanitized_restores_must_be_positive_zeros_to_default() {
    // A 0 in any must-be-positive field (e.g. an absent protobuf field that decoded to the proto3 zero) is
    // restored to the grbl default, NOT left as a wedging zero.
    let mut settings = Settings::default();
    settings.max_rate_mm_min[0] = 0.0;
    settings.max_travel_mm[1] = 0.0;
    settings.homing_feed_mm_min = 0.0;
    settings.homing_seek_mm_min = 0.0;
    settings.spindle_rpm_max = 0.0;
    let fixed = settings.sanitized();
    let defaults = Settings::default();
    assert_eq!(fixed.max_rate_mm_min[0], defaults.max_rate_mm_min[0]);
    assert_eq!(fixed.max_travel_mm[1], defaults.max_travel_mm[1]);
    assert_eq!(fixed.homing_feed_mm_min, defaults.homing_feed_mm_min);
    assert_eq!(fixed.homing_seek_mm_min, defaults.homing_seek_mm_min);
    assert_eq!(fixed.spindle_rpm_max, defaults.spindle_rpm_max);
    // `spindle_rpm_min` legitimately allows 0, so it is left alone.
    assert_eq!(fixed.spindle_rpm_min, 0.0);
  }

  #[test]
  fn sanitized_clamps_tmc_four_bit_fields() {
    // SENDDELAY/IHOLDDELAY occupy 4-bit register fields; an out-of-range value must be clamped so the
    // persisted value equals what gets programmed (31 would otherwise silently mask to 15 downstream).
    let mut settings = Settings::default();
    settings.tmc_send_delay = 31;
    settings.tmc_ihold_delay = 20;
    let fixed = settings.sanitized();
    assert_eq!(fixed.tmc_send_delay, 15);
    assert_eq!(fixed.tmc_ihold_delay, 15);
  }

  #[test]
  fn from_proto_saturates_then_sanitizes_out_of_range() {
    // An out-of-range wire microstep count must NOT truncate to a plausible value (65552 as u16 == 16); it
    // saturates to u16::MAX, then sanitizing maps the invalid count to the default (16). Likewise an over-max
    // current saturates and then clamps to the driver ceiling rather than wrapping to a small "valid" value.
    let mut proto = galdr_proto::Settings::default();
    proto.microsteps_x = 65_552; // == 16 under a truncating `as u16`.
    proto.run_current_ma_y = 70_000; // wraps to 4464 under a truncating `as u16`.
    let settings = Settings::from_proto(&proto).sanitized();
    assert_eq!(settings.microsteps[0], 16, "out-of-range microsteps must default, not truncate to 16");
    assert_eq!(settings.run_current_ma[1], MAX_MOTOR_CURRENT_MA, "over-max current must clamp to the ceiling");
  }

  #[test]
  fn absent_proto_field_decodes_to_grbl_default() {
    // The proto3 contract: an absent field arrives as 0, and the sanitize pass restores the grbl default for
    // must-be-positive fields. Build a frame whose max-rate fields are zero and confirm the decode repairs them.
    let mut settings = Settings::default();
    settings.max_rate_mm_min = [0.0; AXES];
    settings.spindle_rpm_max = 0.0;
    let mut frame: heapless::Vec<u8, { wire::FRAME_MAX_LEN }> = heapless::Vec::new();
    wire::encode(&settings, &mut frame).expect("encode");
    let decoded = wire::decode(&frame).expect("decode");
    let defaults = Settings::default();
    assert_eq!(decoded.max_rate_mm_min, defaults.max_rate_mm_min);
    assert_eq!(decoded.spindle_rpm_max, defaults.spindle_rpm_max);
  }

  #[test]
  fn descriptor_table_keeps_numbers_setter_formatter_consistent() {
    // The three derived operations must agree: every SETTING_NUMBERS entry has exactly one descriptor, dumps a
    // line, and accepts that rendered value back — proving the single descriptor authority cannot drift.
    let mut settings = Settings::default();
    settings.steps_per_mm = [320.5, 321.5, 800.0];
    settings.max_rate_mm_min = [1_000.0, 1_001.0, 500.0];
    for &n in SETTING_NUMBERS {
      assert!(lookup_descriptor(n).is_some(), "$n={n} has no descriptor");
      let mut line: heapless::String<32> = heapless::String::new();
      assert!(settings.write_setting_line(n, &mut line), "no dump line for $n={n}");
      let value = line.split('=').nth(1).expect("dump line has a value");
      settings
        .set_command(n, value)
        .unwrap_or_else(|e| panic!("$n={n} rejected its own dump value {value:?}: {e:?}"));
    }
    // SETTING_NUMBERS has no duplicates and matches the descriptor span count exactly.
    assert_eq!(SETTING_NUMBERS.len(), SETTING_COUNT);
  }

  #[test]
  fn wire_frame_round_trips() {
    let mut settings = Settings::default();
    settings.steps_per_mm = [250.0, 251.0, 800.0];
    settings.homing_enable = true;
    settings.run_current_ma = [900, 900, 1100];
    settings.microsteps = [16, 16, 32];
    let mut frame: heapless::Vec<u8, { wire::FRAME_MAX_LEN }> = heapless::Vec::new();
    wire::encode(&settings, &mut frame).expect("encode");
    let decoded = wire::decode(&frame).expect("decode");
    assert_eq!(decoded, settings);
  }

  #[test]
  fn wire_decode_rejects_corruption() {
    let settings = Settings::default();
    let mut frame: heapless::Vec<u8, { wire::FRAME_MAX_LEN }> = heapless::Vec::new();
    wire::encode(&settings, &mut frame).expect("encode");

    // Flipped CRC byte.
    let mut bad = frame.clone();
    let last = bad.len() - 1;
    bad[last] ^= 0xFF;
    assert_eq!(wire::decode(&bad), Err(wire::CodecError::BadCrc));

    // Wrong magic.
    let mut bad = frame.clone();
    bad[0] ^= 0xFF;
    assert_eq!(wire::decode(&bad), Err(wire::CodecError::BadMagic));

    // Wrong schema version.
    let mut bad = frame.clone();
    bad[4] = bad[4].wrapping_add(1);
    assert_eq!(wire::decode(&bad), Err(wire::CodecError::BadVersion));

    // Truncated below the framing overhead.
    assert_eq!(wire::decode(&frame[..4]), Err(wire::CodecError::BadLength));
  }

  #[test]
  fn load_or_default_is_infallible() {
    // Empty store -> defaults.
    let mut store = MockStore::default();
    assert_eq!(block_on(load_or_default(&mut store)), Settings::default());

    // A persisted record round-trips.
    let mut settings = Settings::default();
    settings.spindle_rpm_max = 24_000.0;
    block_on(store_settings(&mut store, &settings)).expect("save");
    assert_eq!(block_on(load_or_default(&mut store)), settings);

    // A corrupt record falls back to defaults rather than wedging.
    if let Some(record) = store.record.as_mut() {
      record[0] ^= 0xFF;
    }
    assert_eq!(block_on(load_or_default(&mut store)), Settings::default());
  }

  #[test]
  fn hex_codec_round_trips() {
    let bytes = [0x00u8, 0x01, 0x7F, 0x80, 0xAB, 0xFF];
    let mut hex: heapless::String<32> = heapless::String::new();
    assert!(write_hex(&bytes, &mut hex));
    assert_eq!(hex.as_str(), "00017F80ABFF");
    let mut decoded: heapless::Vec<u8, 16> = heapless::Vec::new();
    decode_hex(&hex, &mut decoded).expect("decode");
    assert_eq!(decoded.as_slice(), &bytes);
    // Odd length and non-hex digits are rejected.
    let mut sink: heapless::Vec<u8, 16> = heapless::Vec::new();
    assert_eq!(decode_hex("ABC", &mut sink), Err(HexError));
    assert_eq!(decode_hex("ZZ", &mut sink), Err(HexError));
  }

  #[test]
  fn pb_receiver_reassembles_chunked_frame() {
    // Encode a settings frame, hex it, then feed it to the receiver in small chunks; the last chunk completes.
    let mut settings = Settings::default();
    settings.steps_per_mm = [400.0, 400.0, 1000.0];
    settings.run_current_ma = [1100, 1100, 1300];
    let mut frame: heapless::Vec<u8, { wire::FRAME_MAX_LEN }> = heapless::Vec::new();
    wire::encode(&settings, &mut frame).expect("encode");
    let mut hex: heapless::String<{ wire::FRAME_MAX_LEN * 2 }> = heapless::String::new();
    assert!(write_hex(&frame, &mut hex));

    let mut receiver = PbReceiver::new();
    let hex_bytes = hex.as_bytes();
    // Feed in 40-hex-digit (20-byte) chunks; an even chunk length keeps each piece a whole number of bytes.
    let mut completed = None;
    let mut index = 0;
    while index < hex_bytes.len() {
      let end = (index + 40).min(hex_bytes.len());
      let chunk = core::str::from_utf8(&hex_bytes[index..end]).unwrap();
      match receiver.accept_hex(chunk) {
        PbChunkResult::NeedMore => {}
        PbChunkResult::Complete(s) => completed = Some(s),
        PbChunkResult::Error => panic!("unexpected error on chunk at {index}"),
      }
      index = end;
    }
    assert_eq!(completed, Some(settings));
  }

  #[test]
  fn pb_receiver_rejects_bad_hex_and_corruption() {
    let mut receiver = PbReceiver::new();
    assert_eq!(receiver.accept_hex("xy"), PbChunkResult::Error);

    // A complete frame with a corrupted CRC must fail when fully assembled.
    let settings = Settings::default();
    let mut frame: heapless::Vec<u8, { wire::FRAME_MAX_LEN }> = heapless::Vec::new();
    wire::encode(&settings, &mut frame).expect("encode");
    let last = frame.len() - 1;
    frame[last] ^= 0xFF;
    let mut hex: heapless::String<{ wire::FRAME_MAX_LEN * 2 }> = heapless::String::new();
    assert!(write_hex(&frame, &mut hex));
    assert_eq!(receiver.accept_hex(&hex), PbChunkResult::Error);
  }
}
