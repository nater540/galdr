#![no_std]
#![no_main]
//! Galdr firmware binary: the esp-hal wiring layer for the ESP32-S3 (DOC-00).
//!
//! This is the only crate that touches esp-hal. It implements the hardware-abstraction traits from
//! `firmware-core::hal_traits` against real peripherals and hosts the Embassy async runtime.
//!
//! ## Runtime
//! This project is 100% Embassy. On stable esp-hal 1.0 the Embassy runtime is hosted by `esp-rtos`
//! (the replacement for the retired `esp-hal-embassy`, which is incompatible with stable esp-hal 1.0).
//! With `features = ["embassy"]`, esp-rtos provides the Embassy time-driver and the
//! `esp_rtos::embassy::Executor` / `InterruptExecutor` types; all task code remains ordinary Embassy
//! (`#[embassy_executor::task]`, embassy-time, embassy-sync). `#[esp_rtos::main]` builds the core-0
//! thread-mode executor on the main thread; we call `esp_rtos::start(timer)` first to start the
//! scheduler and time-driver before any task awaits.
//!
//! ## What this phase brings up (comms only — DOC-08 / DOC-01)
//! - Clocks at the maximum CPU frequency and the peripheral set.
//! - The Embassy time-driver / scheduler via `esp_rtos::start`.
//! - The USB Serial/JTAG CDC link (the recommended grblHAL host transport on the S3), split into async
//!   RX/TX halves.
//! - A core-0 thread-mode executor hosting the streaming tasks in [`comms`]: `usb_rx` (drives the
//!   firmware-core protocol state machine, dispatches real-time bytes via Signals), `usb_tx` (the single
//!   USB writer), `comms_consumer` (a Stage-1 stub for the parser/planner that acks lines and answers
//!   `$` queries), and `status_responder`.
//! - The welcome banner on boot and on every soft reset (`0x18`), so a host detects readiness on a
//!   native-USB link that cannot be hard-reset.
//!
//! ## What is deliberately still stubbed (out of scope for the comms phase)
//! - The core-1 `InterruptExecutor` + `motion_executor` and RMT step encoding (DOC-02): the planner →
//!   motion pipeline is not wired into the executor yet; accepted lines flow only to the stub consumer.
//! - TMC2209 manager / UART1 (DOC-03), LEDC spindle PWM (DOC-07), limit/control GPIO + homing (DOC-06),
//!   and esp-storage `$`-settings persistence (DOC-00): in-memory defaults are used; settings writes are
//!   not yet persisted. Each is a documented TODO below.

// esp-backtrace installs the panic handler and exception/backtrace reporting (panic-handler feature).
use esp_backtrace as _;
// esp-println routes `print!`/`println!` to the host; pulled in for early-boot diagnostics before the
// USB CDC link is up. Linked unconditionally so its initializer runs even when unused here.
use esp_println as _;

use embassy_executor::Spawner;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::usb_serial_jtag::UsbSerialJtag;

mod comms;

/// Async entry point. `#[esp_rtos::main]` expands to an `#[esp_hal::main]` reset handler that builds the
/// core-0 thread-mode `esp_rtos::embassy::Executor` and runs this function as its first task. We start
/// the esp-rtos scheduler/time-driver before spawning so `embassy-time` and channel awaits work.
#[esp_rtos::main]
async fn main(spawner: Spawner) {
  // 1. Clocks at max (240 MHz on the S3) + peripheral handle. `expect` is acceptable here: a failure to
  //    bring up clocks is unrecoverable at the very first init step (CLAUDE.md allows `expect` in init).
  let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
  let peripherals = esp_hal::init(config);

  // 2. Start the esp-rtos scheduler with the TIMG0 timer as the time source. This also installs the
  //    Embassy time-driver, so `embassy-time` and channel/Signal awaits operate from here on. On Xtensa
  //    `start` takes only the timer (the RISC-V `int0` argument does not apply).
  let timg0 = TimerGroup::new(peripherals.TIMG0);
  esp_rtos::start(timg0.timer0);

  // 3. Bring up the USB Serial/JTAG CDC controller (internal PHY on GPIO19/20; no descriptors, no eFuse).
  //    Convert to the async driver and split into independent RX/TX halves for the two USB tasks.
  let usb = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();
  let (usb_rx, usb_tx) = usb.split();

  // TODO(DOC-00): esp-storage init + load persisted `$`-settings (fall back to compiled defaults on CRC
  // failure); build the MotionConfig from `$0`/`$29`. Stage 1 uses in-memory protocol defaults.
  // TODO(DOC-03): configure UART1 for the TMC2209 single-wire bus and run the tmc_manager init sequence.
  // TODO(DOC-02): configure RMT TX ch0/1/2 (X/Y/Z step), start the core-1 InterruptExecutor, and spawn
  //   motion_executor; wire the planner BlockQueue + BLOCK_AVAILABLE Signal between planner and executor.
  // TODO(DOC-07): configure LEDC ch0 on GPIO13 for spindle PWM + SPIN_EN/SPIN_DIR GPIO.
  // TODO(DOC-06): configure limit/control GPIO (pull-ups, rising-edge IRQ) and spawn the homing task.
  // TODO(DOC-01): replace `comms_consumer` with the real gcode_parser -> planner -> motion pipeline; add
  //   the parser->RX error-feedback path that calls `engine.note_line_error()` on a downstream `error:N`.

  // 4. Spawn the comms tasks on the core-0 thread-mode executor. `must_spawn` is appropriate at init: a
  //    spawn failure (token already used) is a static, unrecoverable wiring bug, not a runtime condition.
  spawner.must_spawn(comms::usb_rx(usb_rx));
  spawner.must_spawn(comms::usb_tx(usb_tx));
  spawner.must_spawn(comms::comms_consumer());
  spawner.must_spawn(comms::status_responder());

  // 5. Emit the welcome banner on boot so a host detects readiness immediately (native USB cannot be
  //    hard-reset by the host). The banner is also re-emitted on every soft reset from `usb_rx`.
  comms::send_banner().await;

  // The executor keeps the spawned tasks running; this initial task has nothing left to do. Awaiting a
  // never-completing future parks it without burning the CPU in a busy loop.
  core::future::pending::<()>().await;
}
