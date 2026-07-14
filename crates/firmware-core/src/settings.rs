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
//!   plus [`load_or_default`]/[`store_settings`] over the host-testable [`RecordStore`] trait.
//!
//! The protobuf message ([`galdr_proto::Settings`]) is a pure wire/flash DTO reached only at the encode/decode
//! boundary; the planner/motion/tmc consumers keep taking their plain `*Config` structs, untouched.

use crate::drivers::tmc2209::manager::{AxisConfig, TmcConfig};
use crate::hal_traits::{RecordStore, StoreError};
use crate::motion::MotionConfig;
use crate::planner::{PlannerConfig, AXES, DEFAULT_ROTARY_MASK};

/// The UART node addresses strapped on the four TMC2209 drivers (X=0, Y=1, Z=2, A=3 per DOC-03). These are a
/// hardware property, not a user setting, so the `Settings → TmcConfig` conversion supplies them directly.
const TMC_NODES: [u8; AXES] = [0, 1, 2, 3];

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

/// `$22` homing bitmask bit0: enable the homing cycle (grblHAL `Homing_OnOff`). Set => boot locks in
/// `ALARM:11` until `$H`/`$X`, `$H` runs, soft limits may be enabled.
const HOMING_FLAG_ENABLE: u8 = 1 << 0;
/// `$22` homing bitmask bit3: force machine zero to the origin after homing (grblHAL `HOMING_FORCE_SET_ORIGIN`).
/// Set => post-homing machine position is `0` on every axis; clear => per-axis from `$23` mask + `$130` + `$27`.
const HOMING_FLAG_FORCE_SET_ORIGIN: u8 = 1 << 3;
/// `$21` hard-limit bitmask bit0: enable hard limits (grblHAL `Hardlimits_OnOff`). Set => a limit trigger during
/// normal motion raises `ALARM:1`.
const HARD_LIMIT_FLAG_ENABLE: u8 = 1 << 0;
/// `$21` hard-limit bitmask bit1: strict mode (grblHAL) — limits are also checked on `$X` unlock. Reserved for
/// the wiring layer; the bit round-trips so a sender that sets it is honored once strict-mode `$X` lands.
const HARD_LIMIT_FLAG_STRICT: u8 = 1 << 1;
/// `$30` maximum spindle RPM default (WS55-220 nominal).
const DEFAULT_SPINDLE_RPM_MAX: f32 = 12_000.0;
/// `$392` spindle on (spin-up) delay default, seconds. grbl's `DEFAULT_SPINDLE_ON_DELAY` is 0 (no delay), so the
/// firmware comes up inserting no spin-up dwell until a host configures one (DOC-07).
const DEFAULT_SPINDLE_ON_DELAY_S: f32 = 0.0;
/// `$393` spindle reverse dwell default, seconds (Galdr-specific). The M3↔M4 direction-reversal spin-down dwell:
/// a running spindle is forced to a stop and parked for this long before the opposite direction is brought up,
/// so the mechanical spindle is at rest before reversing (DOC-07 safety interlock). 1.5 s is a conservative
/// default for the WS55-220; a host tunes it to the real spin-down time.
const DEFAULT_SPINDLE_REVERSE_DWELL_S: f32 = 1.5;
/// `$481` auto-report interval default, milliseconds. grblHAL ships `DEFAULT_AUTOREPORT_INTERVAL 0`
/// (disabled), so the firmware comes up with periodic auto-reporting OFF until a host enables it.
const DEFAULT_AUTO_REPORT_INTERVAL_MS: u32 = 0;

/// The grblHAL `$481` auto-report interval lower bound, milliseconds: grblHAL's documented "allowed range is
/// 100 - 1000". A non-zero interval below this is rejected (a `$x=val` write) or clamped up (a decoded record),
/// so the report task can never be asked for an unrealistically tight cadence that would starve the USB writer.
pub const AUTO_REPORT_INTERVAL_MIN_MS: u32 = 100;
/// The grblHAL `$481` auto-report interval upper bound, milliseconds ("allowed range is 100 - 1000").
pub const AUTO_REPORT_INTERVAL_MAX_MS: u32 = 1_000;

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
  /// `$20=1` (enable soft limits) was rejected because `$22` homing is not enabled. grblHAL's
  /// `Status_SoftLimitError`: soft limits are only meaningful once the machine can establish a homed zero,
  /// so enabling them without homing is refused (DOC-06). Maps to the grbl "Setting disabled" code.
  SoftLimitsNeedHoming,
}

impl SettingError {
  /// The grblHAL `error:N` status code that best represents this failure (3 = unsupported `$` statement,
  /// 2 = bad numeric value — reused for out-of-range so a sender halts on either; 5 = setting disabled, the
  /// grbl code whose description is "Homing is not enabled via settings", which is exactly why a soft-limit
  /// enable is refused).
  pub fn code(self) -> u8 {
    match self {
      SettingError::UnknownSetting => 3,
      SettingError::BadValue | SettingError::OutOfRange => 2,
      SettingError::SoftLimitsNeedHoming => 5,
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
  /// `$5` limit-pin invert (DOC-06): `0` is the Normally-Closed fail-safe sense (open switch / broken wire reads
  /// triggered), `1` inverts it for Normally-Open wiring or the bench jumper-to-GND bring-up case. Folds into the
  /// limit read via [`crate::hal_traits::LimitConfig`].
  pub limit_invert: bool,
  /// `$6` probe-pin invert (DOC-09, Phase C): set `$6=1` for a Normally-Open touch plate so an untouched plate
  /// reads "not triggered". Folds into the probe read via [`crate::hal_traits::ProbeConfig`].
  pub probe_invert: bool,
  /// `$19` probe-pin pull-up DISABLE (DOC-09, Phase C): `0` keeps the internal pull-up enabled, the default a
  /// passive touch plate needs. Applied at GPIO config; carried in [`crate::hal_traits::ProbeConfig`].
  pub probe_pullup_disable: bool,
  /// `$10` status report field bitmask.
  pub status_report_mask: u8,
  /// `$11` junction deviation, millimeters.
  pub junction_deviation_mm: f32,
  /// `$12` arc chord tolerance, millimeters.
  pub arc_tolerance_mm: f32,
  /// `$20` soft limits enable. grblHAL keeps `$20` a boolean (unlike `$21`/`$22`), but it may only be enabled
  /// when `$22` homing is enabled — [`set_command`](Settings::set_command) enforces that cross-field rule.
  pub soft_limits_enable: bool,
  /// `$21` hard limits EXCLUSIVE BITMASK (grblHAL diverges from legacy grbl's boolean): bit0 enable hard limits,
  /// bit1 strict mode. Decode it through [`hard_limits_enabled`](Settings::hard_limits_enabled) /
  /// [`hard_limits_strict`](Settings::hard_limits_strict) rather than treating the whole value as a bool.
  pub hard_limit_flags: u8,
  /// `$22` homing EXCLUSIVE BITMASK (grblHAL diverges from legacy grbl's boolean): bit0 enable the homing cycle,
  /// bit3 force machine zero to the origin after homing (HOMING_FORCE_SET_ORIGIN); the other grblHAL bits
  /// (single-axis cmds, startup-required, shared-pin, etc.) are reserved here. Decode it through
  /// [`homing_enabled`](Settings::homing_enabled) / [`homing_force_set_origin`](Settings::homing_force_set_origin).
  pub homing_flags: u8,
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
  /// `$392` spindle on (spin-up) delay, seconds. After an M3/M4 the planner inserts a synchronized dwell of this
  /// length before the first cutting move so the spindle reaches speed before it cuts (DOC-07). 0 = no delay.
  pub spindle_on_delay_s: f32,
  /// `$393` spindle reverse dwell, seconds (Galdr-specific). The M3↔M4 reversal spin-down dwell: a running
  /// spindle is stopped and parked this long before the opposite direction is energized (DOC-07 interlock).
  pub spindle_reverse_dwell_s: f32,
  /// `$481` auto-status-report interval, milliseconds (0 = disabled, else `[100, 1000]`). When non-zero the
  /// firmware pushes a `<...>` status report every interval-ms without the host polling `?` (DOC-08 §5).
  pub auto_report_interval_ms: u32,
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
  /// `$376` rotary-axes bitmask (DOC-10.7): bit N set ⇒ axis N is angular (degrees), continuous/rollover, and
  /// exempt from G20/G21 inch scaling. Plumbed into [`PlannerConfig::rotary_mask`]. Default
  /// [`DEFAULT_ROTARY_MASK`] (= 8, A rotary) on a fresh record; `sanitize` masks it to valid bits but never
  /// forces it non-zero, so a deliberate `$376=0` (A as a 4th linear axis) round-trips.
  pub rotary_mask: u8,
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
      limit_invert: false,
      probe_invert: false,
      probe_pullup_disable: false,
      status_report_mask: 0,
      junction_deviation_mm: planner.junction_deviation_mm,
      arc_tolerance_mm: planner.arc_tolerance_mm,
      soft_limits_enable: false,
      hard_limit_flags: 0,
      homing_flags: 0,
      homing_dir_invert_mask: 0,
      homing_feed_mm_min: DEFAULT_HOMING_FEED_MM_MIN,
      homing_seek_mm_min: DEFAULT_HOMING_SEEK_MM_MIN,
      homing_debounce_ms: DEFAULT_HOMING_DEBOUNCE_MS,
      homing_pulloff_mm: DEFAULT_HOMING_PULLOFF_MM,
      spindle_rpm_max: DEFAULT_SPINDLE_RPM_MAX,
      spindle_rpm_min: 0.0,
      spindle_on_delay_s: DEFAULT_SPINDLE_ON_DELAY_S,
      spindle_reverse_dwell_s: DEFAULT_SPINDLE_REVERSE_DWELL_S,
      auto_report_interval_ms: DEFAULT_AUTO_REPORT_INTERVAL_MS,
      steps_per_mm: planner.steps_per_mm,
      max_rate_mm_min: planner.max_rate_mm_min,
      accel_mm_s2: planner.accel_mm_s2,
      max_travel_mm: [DEFAULT_MAX_TRAVEL_MM; AXES],
      run_current_ma: core::array::from_fn(|axis| tmc.axes[axis].run_current_ma),
      microsteps: core::array::from_fn(|axis| tmc.axes[axis].microsteps),
      hold_current_ma: core::array::from_fn(|axis| tmc.axes[axis].hold_current_ma),
      tmc_ihold_delay: tmc.ihold_delay,
      tmc_tpowerdown: tmc.tpowerdown,
      tmc_tpwmthrs: tmc.tpwmthrs,
      tmc_send_delay: tmc.send_delay,
      tmc_r_sense_ohms: tmc.r_sense_ohms,
      rotary_mask: planner.rotary_mask,
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
      rotary_mask: self.rotary_mask,
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

  /// Build the [`HomingConfig`](crate::homing::HomingConfig) the homing cycle consumes from the homing/limit
  /// `$`-settings (DOC-06). `tick_hz` is the firmware RMT tick rate (the same value passed to
  /// [`motion_config`](Settings::motion_config)) so the seek/locate periods encode against the real timing. The
  /// `$23` direction mask is decoded per axis: a clear bit homes toward POSITIVE (grbl default), a set bit
  /// reverses that axis to home toward NEGATIVE. `$22` bit3 supplies `force_set_origin`.
  pub fn homing_config(&self, tick_hz: f32) -> crate::homing::HomingConfig {
    use crate::homing::HomeDirection;
    let direction = |axis: usize| {
      if self.homing_dir_invert_mask & (1 << axis) != 0 {
        HomeDirection::Negative
      } else {
        HomeDirection::Positive
      }
    };
    crate::homing::HomingConfig {
      steps_per_mm: self.steps_per_mm,
      max_travel_mm: self.max_travel_mm,
      seek_mm_min: self.homing_seek_mm_min,
      feed_mm_min: self.homing_feed_mm_min,
      pulloff_mm: self.homing_pulloff_mm,
      direction: core::array::from_fn(direction),
      force_set_origin: self.homing_force_set_origin(),
      motion: self.motion_config(tick_hz),
      limit: self.limit_config(),
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
      axes: core::array::from_fn(axis),
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

  /// The effective `$481` auto-report interval in milliseconds, clamped to a usable cadence: `0` stays `0`
  /// (disabled), and any non-zero value is held within `[AUTO_REPORT_INTERVAL_MIN_MS, AUTO_REPORT_INTERVAL_MAX_MS]`
  /// so the report task can never be driven faster than the floor (which would starve the single USB writer) nor
  /// slower than the documented ceiling. The setter and [`sanitized`](Settings::sanitized) already enforce this
  /// range, so for a clean record this is the identity; it exists so the firmware can ask for a guaranteed-safe
  /// interval directly without re-deriving the clamp at the wiring layer.
  pub fn auto_report_interval_ms(&self) -> u32 {
    clamp_auto_report_interval(self.auto_report_interval_ms)
  }

  /// Build the [`ProbeConfig`](crate::hal_traits::ProbeConfig) the probe-cycle reader consumes from `$6`/`$19`,
  /// so the host-tested probe-trigger logic sees the live invert/pull-up settings (DOC-09, Phase C).
  pub fn probe_config(&self) -> crate::hal_traits::ProbeConfig {
    crate::hal_traits::ProbeConfig { invert: self.probe_invert, pullup_disable: self.probe_pullup_disable }
  }

  /// Build the [`LimitConfig`](crate::hal_traits::LimitConfig) the homing/hard-limit reader consumes from `$5`,
  /// so the host-tested limit-trigger logic sees the live invert setting (DOC-06). Mirrors [`probe_config`].
  pub fn limit_config(&self) -> crate::hal_traits::LimitConfig {
    crate::hal_traits::LimitConfig { invert: self.limit_invert }
  }

  /// Whether the homing cycle is enabled (`$22` bit0). When set the machine boots locked in `ALARM:11`
  /// (homing required) and `$H` runs the cycle; this is the bit every homing-gating path consults.
  pub fn homing_enabled(&self) -> bool {
    self.homing_flags & HOMING_FLAG_ENABLE != 0
  }

  /// Whether homing forces machine zero to the origin (`$22` bit3, grblHAL `HOMING_FORCE_SET_ORIGIN`). When set
  /// the post-homing machine position is `0` on every axis; when clear it is derived per-axis from the `$23`
  /// direction mask, `$130–$132` max travel, and the `$27` pull-off (see the homing state machine).
  pub fn homing_force_set_origin(&self) -> bool {
    self.homing_flags & HOMING_FLAG_FORCE_SET_ORIGIN != 0
  }

  /// Whether hard limits are enabled (`$21` bit0). When set a limit trigger during normal motion raises
  /// `ALARM:1` (position lost); the limit ISR's alarm-raising is suppressed while a homing cycle runs (DOC-06).
  pub fn hard_limits_enabled(&self) -> bool {
    self.hard_limit_flags & HARD_LIMIT_FLAG_ENABLE != 0
  }

  /// Whether hard-limit strict mode is enabled (`$21` bit1) — limits are also checked on `$X` unlock. Reserved
  /// for the wiring layer; surfaced here so it round-trips and is ready when strict-`$X` lands.
  pub fn hard_limits_strict(&self) -> bool {
    self.hard_limit_flags & HARD_LIMIT_FLAG_STRICT != 0
  }

  /// Apply a `$n=value` write, parsing and range-checking `value` for setting `n`. On success the field is
  /// updated (already validated within the same bounds [`sanitized`](Settings::sanitized) enforces); on
  /// failure nothing changes and a [`SettingError`] with a grblHAL code is returned. Both the lookup and the
  /// validation come from the single [`SETTING_DESCRIPTORS`] authority, so a write can never accept a value
  /// the sanitizer would later reject (e.g. `$0=0`, a zero-width step pulse).
  pub fn set_command(&mut self, n: u16, value: &str) -> Result<(), SettingError> {
    let value = value.trim();
    let (desc, axis) = lookup_descriptor(n).ok_or(SettingError::UnknownSetting)?;
    // `$20` carries a cross-field rule `parse_and_apply` (which sees one field) cannot express: grblHAL refuses
    // to ENABLE soft limits unless `$22` homing is enabled (`Status_SoftLimitError`). Reject before applying so
    // the field is left untouched; disabling (`$20=0`) is always allowed.
    if desc.field == Field::SoftLimitsEnable {
      let enabling = parse_bool(value)?;
      if enabling && !self.homing_enabled() {
        return Err(SettingError::SoftLimitsNeedHoming);
      }
    }
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

  /// Render the `$ES` enumeration line for setting `n` into `out` (CRLF-terminated), in grblHAL's
  /// `[SETTING:<id>|<group>|<name>|<unit>|<datatype>|<format>|<min>|<max>]` form. Returns `true` if `n` is a
  /// known setting, `false` otherwise. The firmware loops `enumeration_setting_numbers()` and calls this per
  /// number, so the enumeration is driven by the SAME [`SETTING_DESCRIPTORS`] table as `$$`/`$x=val` — a sender
  /// that builds its UI from `$ES` can never see a setting the live `$$`/setter does not also expose. The value
  /// itself is NOT in the line (a sender reads `$$` for live values); this line is the static description. Per-
  /// axis settings enumerate each of their three `$n` numbers as a distinct `[SETTING:]` row.
  pub fn write_setting_enumeration<const N: usize>(n: u16, out: &mut heapless::String<N>) -> bool {
    use core::fmt::Write;
    let Some((desc, _axis)) = lookup_descriptor(n) else {
      return false;
    };
    let meta = &desc.meta;
    write!(
      out,
      "[SETTING:{}|{}|{}|{}|{}|{}|{}|{}]\r\n",
      n,
      meta.group,
      meta.name,
      meta.unit,
      meta.datatype.code(),
      meta.format,
      meta.min,
      meta.max,
    )
    .is_ok()
  }

  /// Render the `$SED=<n>` description line for setting `n` into `out` (CRLF-terminated), in grblHAL's
  /// `[SETTINGDESCR:<id>|<description>]` form. Returns `true` if `n` is a known setting, `false` otherwise.
  /// The description reuses the enumeration NAME plus the unit (when present) — Phase F does not carry a long
  /// prose description per setting, so the name/unit is the most useful short description a sender can show.
  pub fn write_setting_description<const N: usize>(n: u16, out: &mut heapless::String<N>) -> bool {
    use core::fmt::Write;
    let Some((desc, _axis)) = lookup_descriptor(n) else {
      return false;
    };
    let meta = &desc.meta;
    let result = if meta.unit.is_empty() {
      write!(out, "[SETTINGDESCR:{}|{}]\r\n", n, meta.name)
    } else {
      write!(out, "[SETTINGDESCR:{}|{} ({})]\r\n", n, meta.name, meta.unit)
    };
    result.is_ok()
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
    // The spindle delays ($392/$393) legitimately allow 0 (no delay); clamp only the sub-zero / NaN case so a
    // corrupt record can never park a negative dwell. `non_negative_or` repairs a NaN to the default.
    self.spindle_on_delay_s = non_negative_or(self.spindle_on_delay_s, defaults.spindle_on_delay_s);
    self.spindle_reverse_dwell_s = non_negative_or(self.spindle_reverse_dwell_s, defaults.spindle_reverse_dwell_s);
    // `$481` auto-report: keep `0` (disabled) as-is, but pull any non-zero value into the documented range so a
    // corrupt/legacy record can never ask the report task for a starving cadence or an out-of-range one.
    self.auto_report_interval_ms = clamp_auto_report_interval(self.auto_report_interval_ms);
    // `$20` soft limits are only meaningful with `$22` homing enabled (grblHAL refuses to enable them otherwise,
    // and `set_command` enforces that). A corrupt/legacy flash record could still carry soft-limits-on with
    // homing-off, which would gate moves on a never-established zero — force it off so the invariant the runtime
    // soft-limit check relies on (enabled ⇒ homed-capable) holds even after a bad decode.
    if !self.homing_enabled() {
      self.soft_limits_enable = false;
    }
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
    // `$376` rotary mask: only bit 3 (A) is meaningful on this machine — X/Y/Z are never rotary — so mask to the
    // single legal bit. A present 0 (A treated as a 4th linear axis) is HONORED; the fresh default of 8 lives in
    // `Settings::default()`, not a zero-fill here, so an old 3-axis record loads A linear until `$376=8` (DOC-10.7).
    self.rotary_mask &= DEFAULT_ROTARY_MASK;
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

/// Clamp a `$481` auto-report interval to a usable cadence: `0` (disabled) is preserved, and a non-zero value
/// is held within `[AUTO_REPORT_INTERVAL_MIN_MS, AUTO_REPORT_INTERVAL_MAX_MS]`. Shared by the setter, the
/// sanitizer, and [`Settings::auto_report_interval_ms`] so all three agree on the floor and ceiling.
fn clamp_auto_report_interval(value: u32) -> u32 {
  if value == 0 {
    0
  } else {
    value.clamp(AUTO_REPORT_INTERVAL_MIN_MS, AUTO_REPORT_INTERVAL_MAX_MS)
  }
}

/// Parse a `$481` auto-report interval in milliseconds. `0` disables auto-reporting; any other value must fall
/// within grblHAL's documented `[100, 1000]` range, so a typo cannot install a starving or absurd cadence.
fn parse_auto_report_interval(value: &str) -> Result<u32, SettingError> {
  let parsed = value.parse::<u32>().map_err(|_| SettingError::BadValue)?;
  if parsed == 0 || (AUTO_REPORT_INTERVAL_MIN_MS..=AUTO_REPORT_INTERVAL_MAX_MS).contains(&parsed) {
    Ok(parsed)
  } else {
    Err(SettingError::OutOfRange)
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
  LimitInvert,
  ProbeInvert,
  ProbePullupDisable,
  StatusReportMask,
  JunctionDeviationMm,
  ArcToleranceMm,
  SoftLimitsEnable,
  HardLimitFlags,
  HomingFlags,
  HomingDirInvertMask,
  HomingFeedMmMin,
  HomingSeekMmMin,
  HomingDebounceMs,
  HomingPulloffMm,
  SpindleRpmMax,
  SpindleRpmMin,
  SpindleOnDelayS,
  SpindleReverseDwellS,
  AutoReportIntervalMs,
  StepsPerMm,
  MaxRateMmMin,
  AccelMmS2,
  MaxTravelMm,
  RunCurrentMa,
  Microsteps,
  /// `$376` rotary-axes bitmask (DOC-10.7). A single scalar value (`axis_span == 1`), not a per-axis field.
  RotaryMask,
}

impl Field {
  /// How many consecutive `$n` numbers this field owns: `AXES` for a per-axis field, 1 for a scalar. Drives the
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
      Field::LimitInvert => settings.limit_invert = parse_bool(value)?,
      Field::ProbeInvert => settings.probe_invert = parse_bool(value)?,
      Field::ProbePullupDisable => settings.probe_pullup_disable = parse_bool(value)?,
      Field::StatusReportMask => settings.status_report_mask = parse_u8(value)?,
      Field::JunctionDeviationMm => settings.junction_deviation_mm = parse_f32_non_negative(value)?,
      Field::ArcToleranceMm => settings.arc_tolerance_mm = parse_f32_positive(value)?,
      Field::SoftLimitsEnable => settings.soft_limits_enable = parse_bool(value)?,
      Field::HardLimitFlags => settings.hard_limit_flags = parse_u8(value)?,
      Field::HomingFlags => settings.homing_flags = parse_u8(value)?,
      Field::HomingDirInvertMask => settings.homing_dir_invert_mask = parse_u8(value)?,
      Field::HomingFeedMmMin => settings.homing_feed_mm_min = parse_f32_positive(value)?,
      Field::HomingSeekMmMin => settings.homing_seek_mm_min = parse_f32_positive(value)?,
      Field::HomingDebounceMs => settings.homing_debounce_ms = parse_u32(value)?,
      Field::HomingPulloffMm => settings.homing_pulloff_mm = parse_f32_non_negative(value)?,
      Field::SpindleRpmMax => settings.spindle_rpm_max = parse_f32_positive(value)?,
      Field::SpindleRpmMin => settings.spindle_rpm_min = parse_f32_non_negative(value)?,
      Field::SpindleOnDelayS => settings.spindle_on_delay_s = parse_f32_non_negative(value)?,
      Field::SpindleReverseDwellS => settings.spindle_reverse_dwell_s = parse_f32_non_negative(value)?,
      Field::AutoReportIntervalMs => settings.auto_report_interval_ms = parse_auto_report_interval(value)?,
      Field::StepsPerMm => settings.steps_per_mm[axis] = parse_f32_positive(value)?,
      Field::MaxRateMmMin => settings.max_rate_mm_min[axis] = parse_f32_positive(value)?,
      Field::AccelMmS2 => settings.accel_mm_s2[axis] = parse_f32_positive(value)?,
      Field::MaxTravelMm => settings.max_travel_mm[axis] = parse_f32_positive(value)?,
      Field::RunCurrentMa => settings.run_current_ma[axis] = parse_current_ma(value)?,
      Field::Microsteps => settings.microsteps[axis] = parse_microsteps(value)?,
      // `$376` rotary mask: parse as a u8 bitmask; `sanitized` masks it to the single legal bit (A).
      Field::RotaryMask => settings.rotary_mask = parse_u8(value)?,
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
      Field::LimitInvert => write!(out, "${}={}", n, settings.limit_invert as u8),
      Field::ProbeInvert => write!(out, "${}={}", n, settings.probe_invert as u8),
      Field::ProbePullupDisable => write!(out, "${}={}", n, settings.probe_pullup_disable as u8),
      Field::StatusReportMask => write!(out, "${}={}", n, settings.status_report_mask),
      Field::JunctionDeviationMm => write!(out, "${}={:.3}", n, settings.junction_deviation_mm),
      Field::ArcToleranceMm => write!(out, "${}={:.3}", n, settings.arc_tolerance_mm),
      Field::SoftLimitsEnable => write!(out, "${}={}", n, settings.soft_limits_enable as u8),
      Field::HardLimitFlags => write!(out, "${}={}", n, settings.hard_limit_flags),
      Field::HomingFlags => write!(out, "${}={}", n, settings.homing_flags),
      Field::HomingDirInvertMask => write!(out, "${}={}", n, settings.homing_dir_invert_mask),
      Field::HomingFeedMmMin => write!(out, "${}={:.3}", n, settings.homing_feed_mm_min),
      Field::HomingSeekMmMin => write!(out, "${}={:.3}", n, settings.homing_seek_mm_min),
      Field::HomingDebounceMs => write!(out, "${}={}", n, settings.homing_debounce_ms),
      Field::HomingPulloffMm => write!(out, "${}={:.3}", n, settings.homing_pulloff_mm),
      Field::SpindleRpmMax => write!(out, "${}={:.3}", n, settings.spindle_rpm_max),
      Field::SpindleRpmMin => write!(out, "${}={:.3}", n, settings.spindle_rpm_min),
      Field::SpindleOnDelayS => write!(out, "${}={:.3}", n, settings.spindle_on_delay_s),
      Field::SpindleReverseDwellS => write!(out, "${}={:.3}", n, settings.spindle_reverse_dwell_s),
      Field::AutoReportIntervalMs => write!(out, "${}={}", n, settings.auto_report_interval_ms),
      Field::StepsPerMm => write!(out, "${}={:.3}", n, settings.steps_per_mm[axis]),
      Field::MaxRateMmMin => write!(out, "${}={:.3}", n, settings.max_rate_mm_min[axis]),
      Field::AccelMmS2 => write!(out, "${}={:.3}", n, settings.accel_mm_s2[axis]),
      Field::MaxTravelMm => write!(out, "${}={:.3}", n, settings.max_travel_mm[axis]),
      Field::RunCurrentMa => write!(out, "${}={}", n, settings.run_current_ma[axis]),
      Field::Microsteps => write!(out, "${}={}", n, settings.microsteps[axis]),
      Field::RotaryMask => write!(out, "${}={}", n, settings.rotary_mask),
    };
    result.is_ok()
  }
}

/// A grblHAL setting datatype code, emitted as the 5th field of a `[SETTING:...]` enumeration line so a sender
/// renders the right input control. These are grblHAL's `setting_datatype_t` numeric values: a sender that
/// builds its UI from `$ES` reads this to choose a checkbox (bool), a numeric spinner (integer/float), or a
/// bitfield/dropdown (mask). Only the codes this firmware's settings use are named; the rest are out of scope.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SettingDatatype {
  /// grblHAL datatype 0 — a bitfield of named flags (rendered as a set of checkboxes).
  Bitfield,
  /// grblHAL datatype 3 — a boolean (rendered as a single checkbox / on-off toggle).
  Bool,
  /// grblHAL datatype 5 — an integer (rendered as a whole-number spinner).
  Integer,
  /// grblHAL datatype 6 — a decimal / float (rendered as a fractional spinner).
  Float,
  /// grblHAL datatype 7 — a raw bitmask integer (rendered as a numeric mask field, e.g. `$2`/`$3`/`$23`).
  AxisMask,
}

impl SettingDatatype {
  /// The grblHAL `setting_datatype_t` numeric code emitted in the `[SETTING:]` line's datatype field.
  fn code(self) -> u8 {
    match self {
      SettingDatatype::Bitfield => 0,
      SettingDatatype::Bool => 3,
      SettingDatatype::Integer => 5,
      SettingDatatype::Float => 6,
      SettingDatatype::AxisMask => 7,
    }
  }
}

/// The metadata a `[SETTING:<id>|<group>|<name>|<unit>|<datatype>|<format>|<min>|<max>]` enumeration line needs
/// beyond the live value: the human name, unit string, datatype, an optional grblHAL `format` hint (a bit-flag
/// label list for a bitfield, or a decimal mask like `#0.000` for a float), and the numeric min/max bounds. The
/// min/max are rendered verbatim, with an empty string meaning "unbounded" (grblHAL leaves the field empty).
/// Carried inline in each [`SettingDescriptor`] so the one descriptor table drives `$$`, `$x=val`, AND `$ES`.
struct SettingMeta {
  /// The grblHAL setting GROUP id this setting belongs to (the `[SETTINGGROUP:]` `id`), the 2nd `[SETTING:]`
  /// field. A sender uses it to bucket settings into the same panels grblHAL/ioSender show.
  group: u16,
  /// The human-readable setting name (3rd field), e.g. "Step pulse time".
  name: &'static str,
  /// The unit string (4th field), e.g. "microseconds" / "mm" / "mm/min"; empty for a unit-less mask/bool.
  unit: &'static str,
  /// The grblHAL datatype (5th field) — drives the sender's input control choice.
  datatype: SettingDatatype,
  /// The grblHAL `format` hint (6th field): a comma-separated flag-label list for a bitfield, or a decimal mask
  /// (`#0.000`) for a float; empty when the datatype implies the format (plain integer/bool).
  format: &'static str,
  /// The minimum acceptable value, rendered verbatim (7th field); empty string = unbounded.
  min: &'static str,
  /// The maximum acceptable value, rendered verbatim (8th field); empty string = unbounded.
  max: &'static str,
}

/// One row of the single `$n`→field authority: the base `$n` number, the [`Field`] it drives, and the
/// enumeration [`SettingMeta`]. A per-axis field owns the three consecutive numbers `number..number+3`; a
/// scalar owns just `number`. The metadata makes this table ALSO the single source for the `$ES` enumeration,
/// so the dump (`$$`), the setter (`$x=val`), and the enumeration (`$ES`) can never describe different settings.
struct SettingDescriptor {
  number: u16,
  field: Field,
  meta: SettingMeta,
}

/// A grblHAL setting GROUP: the `[SETTINGGROUP:<id>|<parent>|<name>]` enumeration row. A sender uses these to
/// nest the settings panels (`parent` 0 is a root group). The id set here is self-consistent with the `group`
/// field of every [`SettingMeta`] below — [`setting_groups_cover_all_descriptor_groups`] proves the coverage.
struct SettingGroup {
  /// The group id (referenced by each setting's [`SettingMeta::group`]).
  id: u16,
  /// The parent group id (`0` = a root group with no parent).
  parent: u16,
  /// The human-readable group name.
  name: &'static str,
}

/// The setting GROUPS the `$EG` enumeration emits, one `[SETTINGGROUP:id|parent|name]` per row. The ids match
/// grblHAL's canonical group numbering where it overlaps (so an ioSender that recognizes the standard ids nests
/// our settings into the familiar panels), and every group referenced by a [`SettingMeta`] appears here.
const SETTING_GROUPS: &[SettingGroup] = &[
  SettingGroup { id: GROUP_GENERAL, parent: 0, name: "General" },
  SettingGroup { id: GROUP_CONTROL, parent: 0, name: "Control signals" },
  SettingGroup { id: GROUP_STEPPER, parent: 0, name: "Stepper" },
  SettingGroup { id: GROUP_HOMING, parent: 0, name: "Homing" },
  SettingGroup { id: GROUP_PROBING, parent: 0, name: "Probing" },
  SettingGroup { id: GROUP_SPINDLE, parent: 0, name: "Spindle" },
  SettingGroup { id: GROUP_LIMITS, parent: 0, name: "Limits" },
  SettingGroup { id: GROUP_AXIS, parent: GROUP_STEPPER, name: "Axis" },
];

/// grblHAL setting-group ids, matching its canonical numbering where it overlaps so a standard sender nests our
/// settings into the familiar panels. Named constants so the descriptor table and the group table reference one
/// authority (and [`setting_groups_cover_all_descriptor_groups`] can assert the descriptor groups are all defined).
const GROUP_GENERAL: u16 = 1;
const GROUP_CONTROL: u16 = 4;
const GROUP_STEPPER: u16 = 8;
const GROUP_HOMING: u16 = 6;
const GROUP_PROBING: u16 = 5;
const GROUP_SPINDLE: u16 = 9;
const GROUP_LIMITS: u16 = 3;
const GROUP_AXIS: u16 = 11;

/// The single source of truth mapping every `$n` setting to its field, value policy, dump format, AND its
/// `$ES` enumeration metadata. [`SETTING_NUMBERS`], [`Settings::set_command`], [`Settings::write_setting_line`],
/// and [`Settings::write_setting_enumeration`] are ALL derived from this table, so the dump list, the setter,
/// the formatter, and the enumeration can never disagree. Per-axis groups are one row each (expanded to their
/// three `$n` numbers), not six copy-pasted arms. Order here is `$$` dump (and `$ES` enumeration) order.
const SETTING_DESCRIPTORS: &[SettingDescriptor] = &[
  SettingDescriptor {
    number: 0,
    field: Field::StepPulseUs,
    meta: SettingMeta {
      group: GROUP_STEPPER,
      name: "Step pulse time",
      unit: "microseconds",
      datatype: SettingDatatype::Integer,
      format: "",
      min: "1",
      max: "1000",
    },
  },
  SettingDescriptor {
    number: 1,
    field: Field::StepIdleDelayMs,
    meta: SettingMeta {
      group: GROUP_STEPPER,
      name: "Step idle delay",
      unit: "milliseconds",
      datatype: SettingDatatype::Integer,
      format: "",
      min: "0",
      max: "255",
    },
  },
  SettingDescriptor {
    number: 2,
    field: Field::StepInvertMask,
    meta: SettingMeta {
      group: GROUP_STEPPER,
      name: "Step pulse invert",
      unit: "",
      datatype: SettingDatatype::AxisMask,
      format: "",
      min: "0",
      max: "7",
    },
  },
  SettingDescriptor {
    number: 3,
    field: Field::DirInvertMask,
    meta: SettingMeta {
      group: GROUP_STEPPER,
      name: "Step direction invert",
      unit: "",
      datatype: SettingDatatype::AxisMask,
      format: "",
      min: "0",
      max: "7",
    },
  },
  SettingDescriptor {
    number: 5,
    field: Field::LimitInvert,
    meta: SettingMeta {
      group: GROUP_LIMITS,
      name: "Invert limit pins",
      unit: "",
      datatype: SettingDatatype::Bool,
      format: "",
      min: "0",
      max: "1",
    },
  },
  SettingDescriptor {
    number: 6,
    field: Field::ProbeInvert,
    meta: SettingMeta {
      group: GROUP_PROBING,
      name: "Invert probe pin",
      unit: "",
      datatype: SettingDatatype::Bool,
      format: "",
      min: "0",
      max: "1",
    },
  },
  SettingDescriptor {
    number: 10,
    field: Field::StatusReportMask,
    meta: SettingMeta {
      group: GROUP_GENERAL,
      name: "Status report options",
      unit: "",
      datatype: SettingDatatype::Bitfield,
      format: "Position in machine coordinates,Buffer state",
      min: "0",
      max: "",
    },
  },
  SettingDescriptor {
    number: 11,
    field: Field::JunctionDeviationMm,
    meta: SettingMeta {
      group: GROUP_GENERAL,
      name: "Junction deviation",
      unit: "millimeters",
      datatype: SettingDatatype::Float,
      format: "#0.000",
      min: "0",
      max: "",
    },
  },
  SettingDescriptor {
    number: 12,
    field: Field::ArcToleranceMm,
    meta: SettingMeta {
      group: GROUP_GENERAL,
      name: "Arc tolerance",
      unit: "millimeters",
      datatype: SettingDatatype::Float,
      format: "#0.000",
      // Strictly positive (a zero arc tolerance is rejected): no clean exact minimum, so the bound is left
      // unbounded for display (grblHAL's convention for these positive floats) rather than the misleading "0".
      min: "",
      max: "",
    },
  },
  SettingDescriptor {
    number: 19,
    field: Field::ProbePullupDisable,
    meta: SettingMeta {
      group: GROUP_PROBING,
      name: "Disable probe pin pull-up",
      unit: "",
      datatype: SettingDatatype::Bool,
      format: "",
      min: "0",
      max: "1",
    },
  },
  SettingDescriptor {
    number: 20,
    field: Field::SoftLimitsEnable,
    meta: SettingMeta {
      group: GROUP_LIMITS,
      name: "Soft limits enable",
      unit: "",
      datatype: SettingDatatype::Bool,
      format: "",
      min: "0",
      max: "1",
    },
  },
  SettingDescriptor {
    number: 21,
    field: Field::HardLimitFlags,
    meta: SettingMeta {
      group: GROUP_LIMITS,
      // grblHAL `$21` is a bitfield, not a checkbox: bit0 enable, bit1 strict mode (limits checked on `$X`).
      name: "Hard limits enable",
      unit: "",
      datatype: SettingDatatype::Bitfield,
      format: "Enable,Strict mode",
      min: "0",
      max: "",
    },
  },
  SettingDescriptor {
    number: 22,
    field: Field::HomingFlags,
    meta: SettingMeta {
      group: GROUP_HOMING,
      // grblHAL `$22` is a bitfield: bit0 enable, bit1 single-axis cmds, bit2 startup-required, bit3 set origin
      // to 0. Only bit0/bit3 are honored by the firmware today; the labels mirror grblHAL so a sender's UI maps.
      name: "Homing cycle enable",
      unit: "",
      datatype: SettingDatatype::Bitfield,
      format: "Enable,Enable single axis commands,Homing on startup required,Set machine origin to 0",
      min: "0",
      max: "",
    },
  },
  SettingDescriptor {
    number: 23,
    field: Field::HomingDirInvertMask,
    meta: SettingMeta {
      group: GROUP_HOMING,
      name: "Homing direction invert",
      unit: "",
      datatype: SettingDatatype::AxisMask,
      format: "",
      min: "0",
      max: "7",
    },
  },
  SettingDescriptor {
    number: 24,
    field: Field::HomingFeedMmMin,
    meta: SettingMeta {
      group: GROUP_HOMING,
      name: "Homing locate feed rate",
      unit: "mm/min",
      datatype: SettingDatatype::Float,
      format: "#0.000",
      min: "", // Strictly positive; bound left unbounded for display (see Arc tolerance).
      max: "",
    },
  },
  SettingDescriptor {
    number: 25,
    field: Field::HomingSeekMmMin,
    meta: SettingMeta {
      group: GROUP_HOMING,
      name: "Homing search seek rate",
      unit: "mm/min",
      datatype: SettingDatatype::Float,
      format: "#0.000",
      min: "", // Strictly positive; bound left unbounded for display (see Arc tolerance).
      max: "",
    },
  },
  SettingDescriptor {
    number: 26,
    field: Field::HomingDebounceMs,
    meta: SettingMeta {
      group: GROUP_HOMING,
      name: "Homing switch debounce delay",
      unit: "milliseconds",
      datatype: SettingDatatype::Integer,
      format: "",
      min: "0",
      max: "",
    },
  },
  SettingDescriptor {
    number: 27,
    field: Field::HomingPulloffMm,
    meta: SettingMeta {
      group: GROUP_HOMING,
      name: "Homing switch pull-off distance",
      unit: "millimeters",
      datatype: SettingDatatype::Float,
      format: "#0.000",
      min: "0",
      max: "",
    },
  },
  SettingDescriptor {
    number: 30,
    field: Field::SpindleRpmMax,
    meta: SettingMeta {
      group: GROUP_SPINDLE,
      name: "Maximum spindle speed",
      unit: "RPM",
      datatype: SettingDatatype::Float,
      format: "#0.000",
      min: "", // Strictly positive; bound left unbounded for display (see Arc tolerance).
      max: "",
    },
  },
  SettingDescriptor {
    number: 31,
    field: Field::SpindleRpmMin,
    meta: SettingMeta {
      group: GROUP_SPINDLE,
      name: "Minimum spindle speed",
      unit: "RPM",
      datatype: SettingDatatype::Float,
      format: "#0.000",
      min: "0",
      max: "",
    },
  },
  SettingDescriptor {
    number: 392,
    field: Field::SpindleOnDelayS,
    meta: SettingMeta {
      group: GROUP_SPINDLE,
      name: "Spindle on delay",
      unit: "seconds",
      datatype: SettingDatatype::Float,
      format: "#0.000",
      min: "0",
      max: "",
    },
  },
  SettingDescriptor {
    number: 393,
    field: Field::SpindleReverseDwellS,
    meta: SettingMeta {
      group: GROUP_SPINDLE,
      name: "Spindle reverse dwell",
      unit: "seconds",
      datatype: SettingDatatype::Float,
      format: "#0.000",
      min: "0",
      max: "",
    },
  },
  SettingDescriptor {
    number: 481,
    field: Field::AutoReportIntervalMs,
    meta: SettingMeta {
      group: GROUP_GENERAL,
      name: "Autoreport interval",
      unit: "milliseconds",
      datatype: SettingDatatype::Integer,
      format: "",
      min: "0",
      max: "1000",
    },
  },
  SettingDescriptor {
    number: 100,
    field: Field::StepsPerMm,
    meta: SettingMeta {
      group: GROUP_AXIS,
      name: "Travel resolution",
      unit: "step/mm",
      datatype: SettingDatatype::Float,
      format: "#0.000",
      min: "", // Strictly positive; bound left unbounded for display (see Arc tolerance).
      max: "",
    },
  },
  SettingDescriptor {
    number: 110,
    field: Field::MaxRateMmMin,
    meta: SettingMeta {
      group: GROUP_AXIS,
      name: "Maximum rate",
      unit: "mm/min",
      datatype: SettingDatatype::Float,
      format: "#0.000",
      min: "", // Strictly positive; bound left unbounded for display (see Arc tolerance).
      max: "",
    },
  },
  SettingDescriptor {
    number: 120,
    field: Field::AccelMmS2,
    meta: SettingMeta {
      group: GROUP_AXIS,
      name: "Acceleration",
      unit: "mm/sec^2",
      datatype: SettingDatatype::Float,
      format: "#0.000",
      min: "", // Strictly positive; bound left unbounded for display (see Arc tolerance).
      max: "",
    },
  },
  SettingDescriptor {
    number: 130,
    field: Field::MaxTravelMm,
    meta: SettingMeta {
      group: GROUP_AXIS,
      name: "Maximum travel",
      unit: "millimeters",
      datatype: SettingDatatype::Float,
      format: "#0.000",
      min: "", // Strictly positive; bound left unbounded for display (see Arc tolerance).
      max: "",
    },
  },
  SettingDescriptor {
    number: 140,
    field: Field::RunCurrentMa,
    meta: SettingMeta {
      group: GROUP_AXIS,
      name: "Motor current",
      unit: "mA",
      datatype: SettingDatatype::Integer,
      format: "",
      min: "0",
      max: "2000",
    },
  },
  SettingDescriptor {
    number: 150,
    field: Field::Microsteps,
    meta: SettingMeta {
      group: GROUP_AXIS,
      name: "Microsteps",
      unit: "",
      datatype: SettingDatatype::Integer,
      format: "",
      min: "1",
      max: "256",
    },
  },
  SettingDescriptor {
    number: 376,
    field: Field::RotaryMask,
    meta: SettingMeta {
      group: GROUP_STEPPER,
      name: "Rotary axes",
      unit: "",
      datatype: SettingDatatype::AxisMask,
      // A single scalar bitmask (axis_span == 1). Only bit 3 (A) is meaningful on this machine, so max is 8;
      // `sanitized` masks any write down to that bit (DOC-10.7).
      format: "",
      min: "0",
      max: "8",
    },
  },
];

/// The number of setting GROUPS the `$EG` enumeration emits, so the firmware can loop `0..SETTING_GROUP_COUNT`
/// and render one `[SETTINGGROUP:]` line per index through its line-by-line response writer.
pub const SETTING_GROUP_COUNT: usize = SETTING_GROUPS.len();

/// Render the `$EG` enumeration line for setting-group `index` (`0..`[`SETTING_GROUP_COUNT`]) into `out`
/// (CRLF-terminated), in grblHAL's `[SETTINGGROUP:<id>|<parent>|<name>]` form. Returns `true` on success,
/// `false` for an out-of-range `index`. The firmware loops the count and emits each line through the single
/// USB writer (so a long enumeration never builds one giant buffer), then a terminating `ok`.
pub fn write_setting_group<const N: usize>(index: usize, out: &mut heapless::String<N>) -> bool {
  use core::fmt::Write;
  let Some(group) = SETTING_GROUPS.get(index) else {
    return false;
  };
  write!(out, "[SETTINGGROUP:{}|{}|{}]\r\n", group.id, group.parent, group.name).is_ok()
}

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

/// Why the loader produced the [`Settings`] it returned, so the firmware boot path can tell the legitimate
/// first-boot case apart from a silently-corrupted record. The distinction matters because both yield
/// [`Settings::default`] (homing off), but only the corrupt case warrants a diagnostic on the serial monitor —
/// a missing record is the normal pre-provisioned state, while a present-but-undecodable one means a real
/// `$`-setting change was lost (CRC mismatch, a write the reset button truncated, or a schema-version skew).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LoadOutcome {
  /// A stored record was present and decoded cleanly; the returned settings are the persisted values.
  Loaded,
  /// No record was stored (the `store.load` returned an error); defaults returned — the normal first-boot path.
  DefaultedAbsent,
  /// A record WAS present but failed to decode; defaults returned. This is the lossy case worth surfacing.
  DefaultedCorrupt,
}

/// Load the persisted settings and report WHY, so callers can distinguish a clean decode, the benign
/// "no record yet" first-boot path, and a present-but-undecodable record (a silently-lost change). This is the
/// INFALLIBLE loader core (DOC-04): a corrupt, truncated, version-skewed, or missing record can never wedge
/// boot — it always yields a usable [`Settings`] (sanitized when decoded, else compiled defaults) and never
/// panics or propagates. [`load_or_default`] is the discard-the-outcome convenience wrapper.
pub async fn load_reporting<S: RecordStore>(store: &mut S) -> (Settings, LoadOutcome) {
  let mut buf = [0u8; wire::FRAME_MAX_LEN];
  match store.load(&mut buf).await {
    // A record is present: decode it. A clean decode is `Loaded`; a decode failure is the lossy `DefaultedCorrupt`
    // case (CRC/length/magic/version mismatch) — we still return defaults so a bad region never wedges boot.
    Ok(len) => match wire::decode(&buf[..len]) {
      Ok(settings) => (settings, LoadOutcome::Loaded),
      Err(_) => (Settings::default(), LoadOutcome::DefaultedCorrupt),
    },
    // No record stored yet: the legitimate first-boot path, silently defaulted.
    Err(_) => (Settings::default(), LoadOutcome::DefaultedAbsent),
  }
}

/// Load the persisted settings, falling back to [`Settings::default`] on absence OR any decode failure. This
/// is the INFALLIBLE loader (DOC-04): a corrupt, truncated, version-skewed, or missing record can never wedge
/// boot — it silently yields defaults, which the firmware then applies. The decoded record is sanitized. Use
/// [`load_reporting`] when the caller needs to distinguish absence from corruption (the boot path, to warn).
pub async fn load_or_default<S: RecordStore>(store: &mut S) -> Settings {
  load_reporting(store).await.0
}

/// Encode `settings` into a storage frame and persist it via `store`, replacing any prior record. Surfaces
/// encode/store failures to the caller (a failed `$x=val` persist should still `ok` the line — the in-RAM
/// value applied — but the firmware logs the failure).
pub async fn store_settings<S: RecordStore>(store: &mut S, settings: &Settings) -> Result<(), StoreError> {
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
  use crate::planner::{A_AXIS, AXES};
  use crate::storage_frame::{self, FRAME_OVERHEAD};

  // The shared framing primitives are re-exported so the public `settings::wire` surface (the `CodecError` /
  // `FrameProgress` types comms.rs and `PbReceiver` name) is unchanged after the codec was lifted into
  // `storage_frame`. The encode/decode/frame_progress wrappers below stay settings-specific (they own the
  // settings magic/version and the protobuf type) and call the shared `frame`/`unframe`/`frame_progress`.
  pub use crate::storage_frame::{CodecError, FrameProgress};

  /// Frame magic, ASCII "GdS1" — identifies a galdr settings record.
  const MAGIC: u32 = 0x4764_5331;
  /// On-flash schema version, deliberately held at 1 during early development. We do NOT bump it when merely
  /// ADDING protobuf fields: proto3 is additively compatible, so an old record's absent new fields decode to
  /// their type zero and `positive_or` in [`Settings::sanitized`] restores the proper grbl default. The
  /// version is reserved for a genuinely incompatible layout change (a renumbered/retyped/removed field);
  /// bumping it then makes [`decode`] reject the stale record so the loader restores defaults.
  const SCHEMA_VERSION: u8 = 1;

  /// Maximum framed-record length, sized to the largest protobuf payload plus the shared framing overhead.
  /// Buffers for load/save and the `$PBX` channel are sized from this.
  pub const FRAME_MAX_LEN: usize = galdr_proto::SETTINGS_MAX_LEN + FRAME_OVERHEAD;

  /// Encode `settings` into `out` as a complete storage frame (the buffer is cleared first). Builds the
  /// protobuf payload from the settings and hands it, with the settings magic/version, to the shared framer,
  /// which writes the `MAGIC | version | len | payload | CRC32` layout.
  pub fn encode<const N: usize>(settings: &Settings, out: &mut heapless::Vec<u8, N>) -> Result<(), CodecError> {
    let proto = settings.to_proto();
    let payload_len = galdr_proto::settings_size(&proto);
    storage_frame::frame(MAGIC, SCHEMA_VERSION, payload_len, out, |buf| {
      galdr_proto::encode_settings_into(&proto, buf).map_err(|_| CodecError::BufferFull)
    })
  }

  /// Decode and validate a storage frame, returning the sanitized [`Settings`]. The shared [`unframe`] verifies
  /// the magic, schema version, declared length, and CRC32 and yields the protobuf payload slice; any mismatch
  /// is an error so the infallible loader falls back to defaults rather than trusting a damaged record.
  pub fn decode(frame: &[u8]) -> Result<Settings, CodecError> {
    let payload = storage_frame::unframe(MAGIC, SCHEMA_VERSION, frame)?;
    let proto = galdr_proto::decode_settings(payload).map_err(|_| CodecError::BadPayload)?;
    Ok(Settings::from_proto(&proto).sanitized())
  }

  /// How much of a settings frame a partial byte buffer represents, for the chunked `$PBX` host-sync write path
  /// (which accumulates a frame across several lines). Defers to the shared [`storage_frame::frame_progress`]
  /// with the settings magic/version and this record's [`FRAME_MAX_LEN`].
  pub fn frame_progress(buf: &[u8]) -> FrameProgress {
    storage_frame::frame_progress(MAGIC, SCHEMA_VERSION, FRAME_MAX_LEN, buf)
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
      proto.hard_limit_flags = self.hard_limit_flags as u32;
      proto.homing_flags = self.homing_flags as u32;
      proto.homing_dir_invert_mask = self.homing_dir_invert_mask as u32;
      proto.homing_feed_mm_min = self.homing_feed_mm_min;
      proto.homing_seek_mm_min = self.homing_seek_mm_min;
      proto.homing_debounce_ms = self.homing_debounce_ms;
      proto.homing_pulloff_mm = self.homing_pulloff_mm;
      proto.spindle_rpm_max = self.spindle_rpm_max;
      proto.spindle_rpm_min = self.spindle_rpm_min;
      proto.spindle_on_delay_s = self.spindle_on_delay_s;
      proto.spindle_reverse_dwell_s = self.spindle_reverse_dwell_s;
      proto.auto_report_interval_ms = self.auto_report_interval_ms;
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
      // 4th axis A (index 3) + the $376 rotary mask (DOC-10.7).
      proto.steps_per_mm_a = self.steps_per_mm[A_AXIS];
      proto.max_rate_mm_min_a = self.max_rate_mm_min[A_AXIS];
      proto.accel_mm_s2_a = self.accel_mm_s2[A_AXIS];
      proto.max_travel_mm_a = self.max_travel_mm[A_AXIS];
      proto.run_current_ma_a = self.run_current_ma[A_AXIS] as u32;
      proto.hold_current_ma_a = self.hold_current_ma[A_AXIS] as u32;
      proto.microsteps_a = self.microsteps[A_AXIS] as u32;
      proto.rotary_mask = self.rotary_mask as u32;
      proto.tmc_ihold_delay = self.tmc_ihold_delay as u32;
      proto.tmc_tpowerdown = self.tmc_tpowerdown as u32;
      proto.tmc_tpwmthrs = self.tmc_tpwmthrs;
      proto.tmc_send_delay = self.tmc_send_delay as u32;
      proto.tmc_r_sense_ohms = self.tmc_r_sense_ohms;
      proto.probe_invert = self.probe_invert;
      proto.probe_pullup_disable = self.probe_pullup_disable;
      proto.limit_invert = self.limit_invert;
      proto
    }

    /// Build the typed settings from the protobuf DTO (the inverse of [`to_proto`]). Mask/current/microstep
    /// fields narrow from the protobuf `u32` back to their typed widths using SATURATING conversions, never a
    /// truncating `as`: a wire `microsteps = 65552` must NOT wrap to `16` (a plausible-but-wrong "valid"
    /// value) — it saturates to `u16::MAX`, then the [`sanitized`](Settings::sanitized) pass maps the
    /// out-of-range microstep count to the default and clamps the over-range currents/masks uniformly.
    pub(crate) fn from_proto(proto: &galdr_proto::Settings) -> Settings {
      let _ = AXES; // The X/Y/Z/A quads below assume AXES == 4, asserted once at compile time below.
      Settings {
        step_pulse_us: proto.step_pulse_us,
        step_idle_delay_ms: proto.step_idle_delay_ms,
        step_invert_mask: saturating_u8(proto.step_invert_mask),
        dir_invert_mask: saturating_u8(proto.dir_invert_mask),
        limit_invert: proto.limit_invert,
        probe_invert: proto.probe_invert,
        probe_pullup_disable: proto.probe_pullup_disable,
        status_report_mask: saturating_u8(proto.status_report_mask),
        junction_deviation_mm: proto.junction_deviation_mm,
        arc_tolerance_mm: proto.arc_tolerance_mm,
        soft_limits_enable: proto.soft_limits_enable,
        hard_limit_flags: saturating_u8(proto.hard_limit_flags),
        homing_flags: saturating_u8(proto.homing_flags),
        homing_dir_invert_mask: saturating_u8(proto.homing_dir_invert_mask),
        homing_feed_mm_min: proto.homing_feed_mm_min,
        homing_seek_mm_min: proto.homing_seek_mm_min,
        homing_debounce_ms: proto.homing_debounce_ms,
        homing_pulloff_mm: proto.homing_pulloff_mm,
        spindle_rpm_max: proto.spindle_rpm_max,
        spindle_rpm_min: proto.spindle_rpm_min,
        spindle_on_delay_s: proto.spindle_on_delay_s,
        spindle_reverse_dwell_s: proto.spindle_reverse_dwell_s,
        auto_report_interval_ms: proto.auto_report_interval_ms,
        steps_per_mm: [proto.steps_per_mm_x, proto.steps_per_mm_y, proto.steps_per_mm_z, proto.steps_per_mm_a],
        max_rate_mm_min: [
          proto.max_rate_mm_min_x,
          proto.max_rate_mm_min_y,
          proto.max_rate_mm_min_z,
          proto.max_rate_mm_min_a,
        ],
        accel_mm_s2: [proto.accel_mm_s2_x, proto.accel_mm_s2_y, proto.accel_mm_s2_z, proto.accel_mm_s2_a],
        max_travel_mm: [proto.max_travel_mm_x, proto.max_travel_mm_y, proto.max_travel_mm_z, proto.max_travel_mm_a],
        run_current_ma: [
          saturating_u16(proto.run_current_ma_x),
          saturating_u16(proto.run_current_ma_y),
          saturating_u16(proto.run_current_ma_z),
          saturating_u16(proto.run_current_ma_a),
        ],
        microsteps: [
          saturating_u16(proto.microsteps_x),
          saturating_u16(proto.microsteps_y),
          saturating_u16(proto.microsteps_z),
          saturating_u16(proto.microsteps_a),
        ],
        hold_current_ma: [
          saturating_u16(proto.hold_current_ma_x),
          saturating_u16(proto.hold_current_ma_y),
          saturating_u16(proto.hold_current_ma_z),
          saturating_u16(proto.hold_current_ma_a),
        ],
        tmc_ihold_delay: saturating_u8(proto.tmc_ihold_delay),
        tmc_tpowerdown: saturating_u8(proto.tmc_tpowerdown),
        tmc_tpwmthrs: proto.tmc_tpwmthrs,
        tmc_send_delay: saturating_u8(proto.tmc_send_delay),
        tmc_r_sense_ohms: proto.tmc_r_sense_ohms,
        // `$376` rotary mask: read verbatim (a present 0 is honored), then `sanitize` masks to valid bits and
        // fills the fresh default only when the whole record is default — NOT a zero-fill (DOC-10.7).
        rotary_mask: saturating_u8(proto.rotary_mask),
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

  // The X/Y/Z/A quad conversions assume exactly four axes; fail the build loudly if that ever changes.
  const _: () = assert!(AXES == 4, "settings wire conversions assume AXES == 4");
}

#[cfg(test)]
mod tests {
  use super::*;

  /// An in-memory [`RecordStore`] for host tests: holds the last saved frame, like the byte-buffer mocks used
  /// for `TmcBus`/`StepSink`. Empty until something is saved (so `load` reports `NotFound`).
  #[derive(Default)]
  struct MockStore {
    record: Option<heapless::Vec<u8, { wire::FRAME_MAX_LEN }>>,
  }

  impl RecordStore for MockStore {
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
    assert!(settings.homing_enabled());
    settings.set_command(0, "5").expect("set $0");
    assert_eq!(settings.step_pulse_us, 5);
  }

  // ---- DOC-06: `$5` limit invert, `$21`/`$22` exclusive bitmasks, `$20`-needs-`$22` -----------------

  #[test]
  fn limit_invert_setting_applies_and_builds_limit_config() {
    // `$5` is a grbl boolean that folds into the host-tested `LimitConfig` the same way `$6` folds into
    // `ProbeConfig`, so the limit-trigger logic sees it (DOC-06). Default is the NC fail-safe, no invert.
    let mut settings = Settings::default();
    assert_eq!(settings.limit_config(), crate::hal_traits::LimitConfig::default());
    settings.set_command(5, "1").expect("set $5");
    assert!(settings.limit_invert);
    assert!(settings.limit_config().invert);
    settings.set_command(5, "0").expect("clear $5");
    assert!(!settings.limit_invert);
  }

  #[test]
  fn homing_flags_are_an_exclusive_bitmask_not_a_bool() {
    // grblHAL `$22` is a bitmask, NOT a boolean: bit0 enables homing, bit3 forces machine zero to the origin
    // (HOMING_FORCE_SET_ORIGIN). The named accessors must decode the bits; a raw `$22=9` (b0|b3) means homing
    // enabled AND force-set-origin, which a bool field would have collapsed to "true".
    let mut settings = Settings::default();
    assert!(!settings.homing_enabled());
    assert!(!settings.homing_force_set_origin());
    settings.set_command(22, "9").expect("set $22=9"); // bit0 | bit3
    assert_eq!(settings.homing_flags, 0b1001);
    assert!(settings.homing_enabled(), "bit0 set => homing enabled");
    assert!(settings.homing_force_set_origin(), "bit3 set => force-set-origin");
    // `$22=1` is the common "enable, don't force origin" case.
    settings.set_command(22, "1").expect("set $22=1");
    assert!(settings.homing_enabled());
    assert!(!settings.homing_force_set_origin());
  }

  #[test]
  fn hard_limit_flags_are_an_exclusive_bitmask_not_a_bool() {
    // grblHAL `$21` is a bitmask: bit0 enables hard limits, bit1 is strict mode. Accessors decode the bits.
    let mut settings = Settings::default();
    assert!(!settings.hard_limits_enabled());
    assert!(!settings.hard_limits_strict());
    settings.set_command(21, "3").expect("set $21=3"); // bit0 | bit1
    assert_eq!(settings.hard_limit_flags, 0b11);
    assert!(settings.hard_limits_enabled(), "bit0 => hard limits on");
    assert!(settings.hard_limits_strict(), "bit1 => strict mode");
    settings.set_command(21, "1").expect("set $21=1");
    assert!(settings.hard_limits_enabled());
    assert!(!settings.hard_limits_strict());
  }

  #[test]
  fn soft_limits_rejected_unless_homing_enabled() {
    // grblHAL rejects enabling `$20` (soft limits) unless `$22` homing is enabled (Status_SoftLimitError):
    // soft limits are only meaningful once the machine can establish a homed zero. The reject must leave `$20`
    // untouched. Disabling `$20` (=0) is always allowed.
    let mut settings = Settings::default();
    assert_eq!(settings.set_command(20, "1"), Err(SettingError::SoftLimitsNeedHoming));
    assert!(!settings.soft_limits_enable, "a rejected $20 must not apply");
    // Enable homing first, then `$20=1` is accepted.
    settings.set_command(22, "1").expect("set $22=1");
    settings.set_command(20, "1").expect("set $20=1 once homing on");
    assert!(settings.soft_limits_enable);
    // `$20=0` is always allowed, even with homing off again.
    settings.set_command(22, "0").expect("set $22=0");
    settings.set_command(20, "0").expect("clear $20 is always allowed");
    assert!(!settings.soft_limits_enable);
  }

  #[test]
  fn homing_config_decodes_dir_mask_and_force_origin() {
    use crate::homing::HomeDirection;
    let mut settings = Settings::default();
    // `$23=0` => every axis homes positive; `$22` bit3 force-origin off.
    let cfg = settings.homing_config(1_000_000.0);
    assert_eq!(cfg.direction, [HomeDirection::Positive; AXES]);
    assert!(!cfg.force_set_origin);
    assert_eq!(cfg.seek_mm_min, settings.homing_seek_mm_min);
    assert_eq!(cfg.feed_mm_min, settings.homing_feed_mm_min);
    assert_eq!(cfg.pulloff_mm, settings.homing_pulloff_mm);
    // `$23=2` reverses the Y axis (bit1) to home negative; `$22` bit3 set => force-origin.
    settings.homing_dir_invert_mask = 0b010;
    settings.homing_flags = HOMING_FLAG_ENABLE | HOMING_FLAG_FORCE_SET_ORIGIN;
    let cfg = settings.homing_config(1_000_000.0);
    assert_eq!(cfg.direction, [HomeDirection::Positive, HomeDirection::Negative, HomeDirection::Positive, HomeDirection::Positive]);
    assert!(cfg.force_set_origin);
  }

  #[test]
  fn sanitized_forces_soft_limits_off_when_homing_disabled() {
    // A corrupt/legacy flash record could carry soft-limits-on with homing-off (which `set_command` would have
    // rejected). `sanitized` must repair it so the runtime soft-limit check never gates on an unestablished
    // zero. With homing ON the enable is preserved.
    let mut settings = Settings::default();
    settings.soft_limits_enable = true;
    settings.homing_flags = 0; // homing disabled.
    assert!(!settings.sanitized().soft_limits_enable, "soft limits must be forced off without homing");
    settings.homing_flags = HOMING_FLAG_ENABLE;
    settings.soft_limits_enable = true;
    assert!(settings.sanitized().soft_limits_enable, "soft limits preserved when homing is enabled");
  }

  #[test]
  fn probe_settings_apply_and_build_probe_config() {
    // `$6` (probe invert) and `$19` (probe pull-up disable) are grbl booleans; they apply to the live settings
    // and surface through `probe_config()` so the host-tested probe-trigger logic sees them (DOC-09, Phase C).
    let mut settings = Settings::default();
    assert_eq!(settings.probe_config(), crate::hal_traits::ProbeConfig::default());
    settings.set_command(6, "1").expect("set $6");
    settings.set_command(19, "1").expect("set $19");
    assert!(settings.probe_invert && settings.probe_pullup_disable);
    let cfg = settings.probe_config();
    assert!(cfg.invert && cfg.pullup_disable);
    // grbl boolean semantics: `$6=0` clears it.
    settings.set_command(6, "0").expect("clear $6");
    assert!(!settings.probe_invert);
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
  fn rotary_mask_376_default_and_sanitize_to_bit3() {
    // A fresh record defaults to $376 = 8 (A rotary, DOC-10.7) — NOT a zero-fill.
    assert_eq!(Settings::default().rotary_mask, DEFAULT_ROTARY_MASK);
    // sanitize keeps only the single legal bit (A); X/Y/Z can never be rotary on this machine.
    let mut spurious = Settings::default();
    spurious.rotary_mask = 0b1011; // X + Y + A bits set
    assert_eq!(spurious.sanitized().rotary_mask, 0b1000, "only bit 3 (A) survives");
    // A deliberate $376 = 0 (A treated as a 4th linear axis) is HONORED, not forced back to 8.
    let mut linear_a = Settings::default();
    linear_a.rotary_mask = 0;
    assert_eq!(linear_a.sanitized().rotary_mask, 0, "$376=0 round-trips (not zero-filled)");
  }

  #[test]
  fn rotary_mask_376_round_trips_through_proto() {
    // The $376 mask and the A-axis per-axis quads survive a to_proto/from_proto round-trip (DOC-10.7).
    let mut settings = Settings::default();
    settings.rotary_mask = DEFAULT_ROTARY_MASK;
    settings.steps_per_mm[crate::planner::A_AXIS] = 8.889;
    let restored = Settings::from_proto(&settings.to_proto());
    assert_eq!(restored.rotary_mask, DEFAULT_ROTARY_MASK);
    assert!((restored.steps_per_mm[crate::planner::A_AXIS] - 8.889).abs() < 1e-4, "A steps/deg round-trips");
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
    settings.steps_per_mm = [320.5, 321.5, 800.0, 8.889];
    settings.max_rate_mm_min = [1_000.0, 1_001.0, 500.0, 3600.0];
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
    settings.steps_per_mm = [250.0, 251.0, 800.0, 8.889];
    settings.homing_flags = 1;
    settings.run_current_ma = [900, 900, 1100, 800];
    settings.microsteps = [16, 16, 32, 16];
    let mut frame: heapless::Vec<u8, { wire::FRAME_MAX_LEN }> = heapless::Vec::new();
    wire::encode(&settings, &mut frame).expect("encode");
    let decoded = wire::decode(&frame).expect("decode");
    assert_eq!(decoded, settings);
  }

  #[test]
  fn every_settings_field_is_wired_through_the_proto_mapping() {
    // LOCK (D2): every field of `Settings` must be carried through BOTH `to_proto` and `from_proto`, or its value
    // is silently dropped from the `$PBX` host-sync channel and the flash record while `$$`/`$x=val` still appear
    // to work. This uses a FULL struct literal (no `..Default::default()`) with a distinct, in-range, non-default
    // value in every field: adding a field to `Settings` breaks THIS test's compilation until it is given a value
    // here, at which point the round-trip assertion below catches a `to_proto`/`from_proto` pair that forgot it.
    // (`from_proto` is the raw mapping — `sanitized()` is applied separately — so in-range values survive exactly.)
    let settings = Settings {
      step_pulse_us: 7,
      step_idle_delay_ms: 42,
      step_invert_mask: 0b0000_0101,
      dir_invert_mask: 0b0000_0010,
      limit_invert: true,
      probe_invert: true,
      probe_pullup_disable: true,
      status_report_mask: 0b0000_0011,
      junction_deviation_mm: 0.0123,
      arc_tolerance_mm: 0.0021,
      soft_limits_enable: true,
      hard_limit_flags: 0b0000_0011,
      homing_flags: 0b0000_1001,
      homing_dir_invert_mask: 0b0000_0100,
      homing_feed_mm_min: 33.0,
      homing_seek_mm_min: 777.0,
      homing_debounce_ms: 55,
      homing_pulloff_mm: 1.75,
      spindle_rpm_max: 24_500.0,
      spindle_rpm_min: 1_200.0,
      spindle_on_delay_s: 1.5,
      spindle_reverse_dwell_s: 0.75,
      auto_report_interval_ms: 200,
      steps_per_mm: [250.0, 251.0, 800.0, 8.889],
      max_rate_mm_min: [5_100.0, 5_200.0, 900.0, 3_600.0],
      accel_mm_s2: [110.0, 120.0, 30.0, 720.0],
      max_travel_mm: [210.0, 220.0, 60.0, 360.0],
      run_current_ma: [900, 910, 1_100, 800],
      microsteps: [16, 16, 32, 8],
      hold_current_ma: [400, 410, 500, 300],
      tmc_ihold_delay: 6,
      tmc_tpowerdown: 20,
      tmc_tpwmthrs: 145,
      tmc_send_delay: 2,
      tmc_r_sense_ohms: 0.11,
      rotary_mask: 0b0000_1000,
    };
    let restored = Settings::from_proto(&settings.to_proto());
    assert_eq!(
      restored, settings,
      "a Settings field is not wired through to_proto/from_proto — it would silently drop from $PBX/flash",
    );
  }

  #[test]
  fn wire_frame_byte_layout_is_stable_after_codec_extraction() {
    // The on-flash byte layout MUST be byte-for-byte identical to before the shared `storage_frame` codec was
    // extracted, so records already in flash still decode. Assert the canonical
    // `GdS1(LE) | VERSION=1 | payload_len(LE) | payload | CRC32(LE)` framing the prior hand-rolled `encode`
    // produced. The magic ASCII is "GdS1" stored little-endian (0x47645331) and the version is 1.
    let settings = Settings::default();
    let mut frame: heapless::Vec<u8, { wire::FRAME_MAX_LEN }> = heapless::Vec::new();
    wire::encode(&settings, &mut frame).expect("encode");
    // Magic: 0x4764_5331 little-endian → bytes [0x31, 0x53, 0x64, 0x47] = b"1SdG" on the wire.
    assert_eq!(&frame[0..4], &0x4764_5331u32.to_le_bytes());
    assert_eq!(frame[4], 1, "schema version is 1");
    // The declared payload length (LE u16 at [5..7]) matches the protobuf size, and the frame is
    // header(7) + payload + crc(4) bytes long.
    let payload_len = u16::from_le_bytes([frame[5], frame[6]]) as usize;
    assert_eq!(frame.len(), 7 + payload_len + 4);
    // The trailing CRC32 covers everything before it (header + payload), reflected-poly 0xEDB88320.
    let payload_end = 7 + payload_len;
    let crc = u32::from_le_bytes([frame[payload_end], frame[payload_end + 1], frame[payload_end + 2], frame[payload_end + 3]]);
    assert_eq!(crc, firmware_core_crc(&frame[..payload_end]));
    // And the frame round-trips back to the same settings, confirming the extracted codec accepts what it emits.
    assert_eq!(wire::decode(&frame).expect("decode"), settings);
  }

  /// The reference CRC32 (IEEE 802.3, reflected, poly 0xEDB88320) used to pin the frame's checksum byte-for-byte
  /// against the layout the prior hand-rolled codec produced; recomputed here independently of `storage_frame`.
  fn firmware_core_crc(data: &[u8]) -> u32 {
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
  fn load_reporting_distinguishes_absent_from_corrupt() {
    // Absent record: the legitimate first-boot path — defaults, signalled as ABSENT (no warning at the callsite).
    let mut store = MockStore::default();
    assert_eq!(block_on(load_reporting(&mut store)), (Settings::default(), LoadOutcome::DefaultedAbsent));

    // A present, well-formed record decodes cleanly and is signalled as LOADED. Use a non-default homing flag so
    // a regression that silently returns defaults (Bug B) would fail both the value AND the outcome assertion.
    let mut settings = Settings::default();
    settings.homing_flags = HOMING_FLAG_ENABLE;
    block_on(store_settings(&mut store, &settings)).expect("save");
    assert_eq!(block_on(load_reporting(&mut store)), (settings, LoadOutcome::Loaded));

    // A present-but-corrupt record (a flipped CRC byte, as if a write were truncated by the reset button) returns
    // defaults — homing OFF — but is now distinguishable as CORRUPT, so the boot path can warn instead of silently
    // booting on factory defaults. This is the exact scenario Bug B masked.
    if let Some(record) = store.record.as_mut() {
      let last = record.len() - 1;
      record[last] ^= 0xFF;
    }
    assert_eq!(block_on(load_reporting(&mut store)), (Settings::default(), LoadOutcome::DefaultedCorrupt));

    // A truncated record (a write interrupted below the framing overhead) is likewise CORRUPT, not absent.
    if let Some(record) = store.record.as_mut() {
      record.truncate(3);
    }
    assert_eq!(block_on(load_reporting(&mut store)), (Settings::default(), LoadOutcome::DefaultedCorrupt));
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
    settings.steps_per_mm = [400.0, 400.0, 1000.0, 8.889];
    settings.run_current_ma = [1100, 1100, 1300, 800];
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

  // --- DOC-07: $392 spindle on delay + $393 spindle reverse dwell ----------------------------------

  #[test]
  fn spindle_delay_settings_default_apply_and_round_trip() {
    let settings = Settings::default();
    // grbl-aligned defaults: no spin-up delay, a conservative 1.5 s reverse spin-down dwell.
    assert_eq!(settings.spindle_on_delay_s, 0.0);
    assert_eq!(settings.spindle_reverse_dwell_s, 1.5);

    let mut settings = settings;
    settings.set_command(392, "0.25").expect("set $392=0.25");
    settings.set_command(393, "2.000").expect("set $393=2.0");
    assert_eq!(settings.spindle_on_delay_s, 0.25);
    assert_eq!(settings.spindle_reverse_dwell_s, 2.0);
    // Zero is a legitimate value for both (no delay), so it must be accepted, not rejected as out-of-range.
    settings.set_command(392, "0").expect("$392=0 accepted (no spin-up delay)");
    assert_eq!(settings.spindle_on_delay_s, 0.0);
    // A negative value is out of range and must be rejected, leaving the prior value untouched.
    assert_eq!(settings.set_command(393, "-1"), Err(SettingError::OutOfRange));
    assert_eq!(settings.spindle_reverse_dwell_s, 2.0);

    // The pair survives the protobuf/flash wire frame round-trip unchanged.
    let mut frame: heapless::Vec<u8, { wire::FRAME_MAX_LEN }> = heapless::Vec::new();
    wire::encode(&settings, &mut frame).expect("encode");
    let decoded = wire::decode(&frame).expect("decode");
    assert_eq!(decoded.spindle_on_delay_s, settings.spindle_on_delay_s);
    assert_eq!(decoded.spindle_reverse_dwell_s, settings.spindle_reverse_dwell_s);
  }

  #[test]
  fn spindle_delay_dump_lines_render_with_three_decimals() {
    let mut settings = Settings::default();
    settings.spindle_on_delay_s = 0.5;
    settings.spindle_reverse_dwell_s = 1.5;
    let mut on: heapless::String<32> = heapless::String::new();
    assert!(settings.write_setting_line(392, &mut on));
    assert_eq!(on.as_str(), "$392=0.500");
    let mut rev: heapless::String<32> = heapless::String::new();
    assert!(settings.write_setting_line(393, &mut rev));
    assert_eq!(rev.as_str(), "$393=1.500");
  }

  // --- Phase F: $481 auto-report setting + $ES/$EG/$SED enumeration ---------------------------------

  #[test]
  fn auto_report_interval_parses_clamps_and_round_trips() {
    let mut settings = Settings::default();
    // Default is disabled (0).
    assert_eq!(settings.auto_report_interval_ms, 0);
    assert_eq!(settings.auto_report_interval_ms(), 0);
    // A valid in-range value applies and the accessor returns it verbatim.
    settings.set_command(481, "250").expect("set $481=250");
    assert_eq!(settings.auto_report_interval_ms, 250);
    assert_eq!(settings.auto_report_interval_ms(), 250);
    // 0 disables.
    settings.set_command(481, "0").expect("set $481=0");
    assert_eq!(settings.auto_report_interval_ms, 0);
    // Out-of-range non-zero values are rejected by the setter (grblHAL's 100..=1000), leaving the value unchanged.
    assert_eq!(settings.set_command(481, "50"), Err(SettingError::OutOfRange));
    assert_eq!(settings.set_command(481, "2000"), Err(SettingError::OutOfRange));
    assert_eq!(settings.auto_report_interval_ms, 0);
    // A non-numeric value is a bad value.
    assert_eq!(settings.set_command(481, "fast"), Err(SettingError::BadValue));
  }

  #[test]
  fn auto_report_interval_sanitizer_floors_a_corrupt_record() {
    // A corrupt/legacy record carrying an out-of-range non-zero interval is pulled into range; 0 stays disabled.
    // Build each case as a fresh record so the sanitizer is exercised on the raw (un-clamped) field value.
    let with_interval = |ms: u32| Settings { auto_report_interval_ms: ms, ..Settings::default() };
    assert_eq!(with_interval(5).sanitized().auto_report_interval_ms, AUTO_REPORT_INTERVAL_MIN_MS);
    assert_eq!(with_interval(50_000).sanitized().auto_report_interval_ms, AUTO_REPORT_INTERVAL_MAX_MS);
    assert_eq!(with_interval(0).sanitized().auto_report_interval_ms, 0);
  }

  #[test]
  fn auto_report_interval_persists_through_the_wire_frame() {
    let mut settings = Settings::default();
    settings.set_command(481, "300").expect("set $481=300");
    let store = &mut MockStore::default();
    block_on(store_settings(store, &settings)).expect("store");
    let loaded = block_on(load_or_default(store));
    assert_eq!(loaded.auto_report_interval_ms, 300);
  }

  #[test]
  fn every_setting_number_enumerates_with_min_max_matching_its_range() {
    // Every `$$` number must also produce a well-formed `[SETTING:]` line whose 8 fields parse, and whose
    // min/max (when present) bracket the setting's accepted range — i.e. the descriptor metadata cannot drift
    // from the setter's validation.
    let mut settings = Settings::default();
    // `$20` (soft limits) is the one setting with a cross-field gate: `set_command` refuses to ENABLE it unless
    // `$22` homing is enabled (DOC-06). Pre-enable homing so this generic "the enumerated bound is an accepted
    // value" check exercises `$20`'s real range; the gate itself is covered by `soft_limits_rejected_unless_homing_enabled`.
    settings.homing_flags = HOMING_FLAG_ENABLE;
    for &n in SETTING_NUMBERS {
      let mut line: heapless::String<160> = heapless::String::new();
      assert!(Settings::write_setting_enumeration(n, &mut line), "no enumeration for $n={n}");
      let body = line.as_str().trim_end_matches("\r\n");
      let inner = body.strip_prefix("[SETTING:").and_then(|s| s.strip_suffix(']')).expect("bracketed [SETTING:]");
      let fields: heapless::Vec<&str, 8> = inner.split('|').collect();
      assert_eq!(fields.len(), 8, "[SETTING:] must have 8 pipe-separated fields, got {inner:?}");
      // Field 0 is the id; it must equal `n`.
      assert_eq!(fields[0].parse::<u16>().expect("numeric id"), n, "[SETTING:] id mismatch for $n={n}");
      // Fields 6/7 are min/max; when present they bracket the accepted range — a write of min and of max must be
      // accepted (or, for an empty bound, skipped). This proves the enumerated bounds are the REAL bounds.
      if !fields[6].is_empty() {
        assert!(settings.set_command(n, fields[6]).is_ok(), "$n={n} rejected its enumerated min {:?}", fields[6]);
      }
      if !fields[7].is_empty() {
        assert!(settings.set_command(n, fields[7]).is_ok(), "$n={n} rejected its enumerated max {:?}", fields[7]);
      }
    }
  }

  #[test]
  fn setting_enumeration_line_wire_format() {
    // Byte-exact `[SETTING:0|...]` for `$0` (step pulse time) — a sampled line type asserted in full.
    let mut line: heapless::String<160> = heapless::String::new();
    assert!(Settings::write_setting_enumeration(0, &mut line));
    assert_eq!(line.as_str(), "[SETTING:0|8|Step pulse time|microseconds|5||1|1000]\r\n");
    // An unknown number does not enumerate.
    let mut none: heapless::String<160> = heapless::String::new();
    assert!(!Settings::write_setting_enumeration(9999, &mut none));
    assert!(none.is_empty());
  }

  #[test]
  fn setting_groups_cover_all_descriptor_groups() {
    // Every group id referenced by a descriptor's metadata must have a `[SETTINGGROUP:]` row, so a sender that
    // buckets settings by group never references an undefined group.
    for desc in SETTING_DESCRIPTORS {
      assert!(
        SETTING_GROUPS.iter().any(|g| g.id == desc.meta.group),
        "setting $n={} references undefined group {}",
        desc.number,
        desc.meta.group,
      );
    }
    // And `$EG` renders one well-formed line per group, byte-exact for the General root group.
    let mut first: heapless::String<96> = heapless::String::new();
    assert!(write_setting_group(0, &mut first));
    assert_eq!(first.as_str(), "[SETTINGGROUP:1|0|General]\r\n");
    // Out-of-range index does not render.
    let mut none: heapless::String<96> = heapless::String::new();
    assert!(!write_setting_group(SETTING_GROUP_COUNT, &mut none));
    assert!(none.is_empty());
  }

  #[test]
  fn setting_description_line_wire_format() {
    // `$SED=0` renders `[SETTINGDESCR:0|Step pulse time (microseconds)]`; an unknown id does not render.
    let mut line: heapless::String<96> = heapless::String::new();
    assert!(Settings::write_setting_description(0, &mut line));
    assert_eq!(line.as_str(), "[SETTINGDESCR:0|Step pulse time (microseconds)]\r\n");
    let mut none: heapless::String<96> = heapless::String::new();
    assert!(!Settings::write_setting_description(9999, &mut none));
    assert!(none.is_empty());
  }
}
