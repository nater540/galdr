//! Spindle hardware wiring (DOC-07): the esp-hal implementations of the firmware-core spindle traits plus the
//! peripheral bring-up. The host-tested M3/M4/M5 sequencing, RPM→duty mapping, and the reversal interlock live in
//! [`firmware_core::spindle::SpindleController`]; this module only maps that logic onto real hardware:
//! - [`LedcPwmSink`] drives the WS55-220 speed input as a LEDC PWM on GPIO13 (the duty conditioned to 0–10 V by
//!   an external RC + op-amp stage, DOC-07). 5 kHz, 13-bit resolution.
//! - [`SpinEnableOut`] drives SPIN_EN (GPIO14), active-LOW: a logical RUN (`true`) pulls the pin LOW (the
//!   WS55-220 runs when its EN terminal is at GND). The trait's logical-level contract puts this inversion here.
//! - [`SpinDirectionOut`] drives SPIN_DIR (GPIO15), the F/R input: it passes the controller's logical CW/CCW
//!   level straight through (logical `true` = CW). If a board needs the opposite F/R sense, invert it here.
//!
//! The LEDC `Ledc`/`Timer`/`Channel` form a self-referential `'static` chain (the channel borrows the timer which
//! borrows the controller), so [`init`] parks each in a caller-provided `StaticCell` and returns sinks that hold
//! only `'static` references — mirroring how `main` parks the RMT sink. `unsafe` is confined to the esp-hal
//! boundary; this module adds none of its own.

use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::ledc::channel::{self, ChannelHW, ChannelIFace};
use esp_hal::ledc::timer::{self, TimerIFace};
use esp_hal::ledc::{LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::time::Rate;
use static_cell::StaticCell;

use firmware_core::hal_traits::{DigitalOut, DigitalOutError, PwmError, PwmSink};
use firmware_core::spindle::SpindleController;

/// LEDC PWM frequency for the spindle speed line, Hz. DOC-07 recommends 1–20 kHz with ≥10-bit resolution so the
/// external RC low-pass produces a smooth analog; 5 kHz at 13-bit is the documented comfortable point on the S3.
const SPINDLE_PWM_HZ: u32 = 5_000;
/// LEDC duty resolution in bits. 13-bit gives 8192 duty steps (max raw duty `2^13 - 1 = 8191`), well within the
/// RC filter's smoothing and the S3 LEDC's capability at 5 kHz.
const SPINDLE_PWM_DUTY_BITS: u32 = 13;
/// The maximum raw duty value at [`SPINDLE_PWM_DUTY_BITS`] (`2^bits - 1`): full-scale 10 V → max RPM.
const SPINDLE_PWM_DUTY_MAX: u32 = (1 << SPINDLE_PWM_DUTY_BITS) - 1;

/// The concrete spindle controller type the firmware drives: the host-tested [`SpindleController`] specialized
/// over the LEDC PWM sink and two polarity-parameterized [`GpioOut`] (SPIN_EN active-low, SPIN_DIR straight). The
/// `spindle` task owns one of these by `'static` mutable borrow.
pub type Spindle = SpindleController<LedcPwmSink, GpioOut, GpioOut>;

/// [`PwmSink`] over the LEDC channel driving the spindle speed line (GPIO13). Holds a `'static` reference to the
/// configured channel; `set_duty_hw` takes `&self`, so only a shared reference is needed. The normalized
/// `0.0..=1.0` `frac` from the controller is scaled to the 13-bit raw duty and written to hardware.
pub struct LedcPwmSink {
  channel: &'static channel::Channel<'static, LowSpeed>,
}

impl PwmSink for LedcPwmSink {
  /// Scale the normalized `frac` (the controller guarantees it is already clamped to `0.0..=1.0`) to the 13-bit
  /// raw duty and write it. A defensive clamp keeps a stray out-of-range `frac` from wrapping the `u32` cast.
  fn set_duty(&mut self, frac: f32) -> Result<(), PwmError> {
    let clamped = frac.clamp(0.0, 1.0);
    let raw = libm::roundf(clamped * SPINDLE_PWM_DUTY_MAX as f32) as u32;
    // `set_duty_hw` is infallible (it writes the duty register and latches it); the `PwmSink` Result exists for
    // transports that can fail, so a successful write simply returns Ok.
    self.channel.set_duty_hw(raw.min(SPINDLE_PWM_DUTY_MAX));
    Ok(())
  }
}

/// A logical-level [`DigitalOut`] over one esp-hal GPIO with a configurable active polarity (DOC-07). The
/// host-tested controller deals only in logical levels (true = asserted); the board polarity lives HERE, in the one
/// `active_high` field, so the two spindle outputs share a single impl instead of two near-identical ones:
/// - SPIN_EN (GPIO14) is ACTIVE-LOW (`active_high = false`): a logical RUN (`true`) drives the pin LOW (EN to GND =
///   run); a de-assert drives it HIGH (spindle off).
/// - SPIN_DIR (GPIO15) is straight-through (`active_high = true`): logical `true` (CW) → pin HIGH, `false` (CCW) →
///   LOW. A board whose F/R input has the opposite sense is handled by flipping `active_high`, not firmware-core.
pub struct GpioOut {
  pin: Output<'static>,
  active_high: bool,
}

impl DigitalOut for GpioOut {
  /// Drive the pin from the logical `level` and the configured polarity: `level == active_high` → HIGH, else LOW.
  fn set(&mut self, level: bool) -> Result<(), DigitalOutError> {
    self.pin.set_level(if level == self.active_high { Level::High } else { Level::Low });
    Ok(())
  }
}

/// Bring up the LEDC spindle PWM (timer0 + channel0 on GPIO13) and the SPIN_EN / SPIN_DIR GPIO, then assemble the
/// [`Spindle`] controller. The spindle starts SAFE: SPIN_EN de-asserted (HIGH, active-low → stopped), DIR idle
/// (latched to the real direction on the first start, before the spindle is energized), and the LEDC duty at 0
/// (0 V → 0 RPM), so the spindle is off until the first M3/M4 is processed.
///
/// The `Ledc`, its `Timer0`, and `Channel0` form a self-referential `'static` chain (the channel borrows the
/// timer which borrows the controller), so each is parked in a caller-provided `StaticCell`; the returned sinks
/// hold only `'static` references into them. The caller (`main`) owns those cells for the program's lifetime.
///
/// # Panics
/// `expect` is used here because this runs once in `main`'s init path, where a failure to bring up the LEDC
/// peripheral or claim a spindle pin is an unrecoverable wiring/config fault, not a runtime condition (CLAUDE.md
/// permits `expect` in init). It never executes after boot.
pub fn init(
  ledc_peripheral: esp_hal::peripherals::LEDC<'static>,
  pwm_pin: esp_hal::peripherals::GPIO13<'static>,
  enable_pin: esp_hal::peripherals::GPIO14<'static>,
  direction_pin: esp_hal::peripherals::GPIO15<'static>,
  ledc_cell: &'static StaticCell<Ledc<'static>>,
  timer_cell: &'static StaticCell<timer::Timer<'static, LowSpeed>>,
  channel_cell: &'static StaticCell<channel::Channel<'static, LowSpeed>>,
) -> Spindle {
  // The LEDC controller, parked `'static` so the timer/channel can borrow it for the program's lifetime.
  let mut ledc = Ledc::new(ledc_peripheral);
  ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
  let ledc: &'static Ledc<'static> = ledc_cell.init(ledc);

  // Low-speed timer0 at 5 kHz, 13-bit duty resolution (DOC-07). Parked `'static` so the channel can borrow it.
  let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
  lstimer0
    .configure(timer::config::Config {
      duty: timer::config::Duty::Duty13Bit,
      clock_source: timer::LSClockSource::APBClk,
      frequency: Rate::from_hz(SPINDLE_PWM_HZ),
    })
    .expect("LEDC spindle timer0 config");
  let lstimer0: &'static timer::Timer<'static, LowSpeed> = timer_cell.init(lstimer0);

  // Channel0 on GPIO13, bound to timer0, starting at 0 % duty (spindle off / 0 V) until an M3/M4 raises it.
  let mut channel0 = ledc.channel(channel::Number::Channel0, pwm_pin);
  channel0
    .configure(channel::config::Config {
      timer: lstimer0,
      duty_pct: 0,
      drive_mode: esp_hal::gpio::DriveMode::PushPull,
    })
    .expect("LEDC spindle channel0 config");
  let channel0: &'static channel::Channel<'static, LowSpeed> = channel_cell.init(channel0);

  // SPIN_EN (GPIO14, active-low): start HIGH = de-asserted = spindle OFF until the first M3/M4.
  let out_cfg = OutputConfig::default();
  let enable = Output::new(enable_pin, Level::High, out_cfg);
  // SPIN_DIR (GPIO15): start LOW; the first start latches the real direction before the spindle is energized.
  let direction = Output::new(direction_pin, Level::Low, out_cfg);

  SpindleController::new(
    LedcPwmSink { channel: channel0 },
    GpioOut { pin: enable, active_high: false },   // SPIN_EN: active-low (logical RUN → pin LOW).
    GpioOut { pin: direction, active_high: true }, // SPIN_DIR: straight-through (logical CW → pin HIGH).
  )
}
