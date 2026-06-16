#![no_std]
#![no_main]
//! Galdr firmware binary: the esp-hal wiring layer for the ESP32-S3 (DOC-00).
//!
//! This is the only crate that touches esp-hal. It implements the hardware-abstraction traits from
//! `firmware-core::hal_traits` against real peripherals (RMT step generation, TMC2209 UART, LEDC
//! spindle PWM, USB Serial/JTAG CDC) and hosts the Embassy async runtime.
//!
//! Runtime: this project is 100% Embassy. On stable esp-hal 1.0 the Embassy runtime is hosted by
//! `esp-rtos` (the official replacement for the retired `esp-hal-embassy`, which is incompatible
//! with stable esp-hal 1.0). With `features = ["embassy"]`, esp-rtos provides only the Embassy
//! time-driver and the `esp_rtos::embassy::Executor` / `esp_rtos::embassy::InterruptExecutor`
//! types; all task code remains ordinary Embassy (`#[embassy_executor::task]`, embassy-time,
//! embassy-sync). This corrects DOC-00, which still assumes esp-hal-embassy.
//!
//! Planned executor split (DOC-00 / DOC-01):
//! - Core 1 (APP_CPU): only the `motion_executor` task on a high-priority
//!   `esp_rtos::embassy::InterruptExecutor` for real-time RMT step generation. Started via
//!   `esp_rtos::start_second_core(...)`, which takes a `&'static mut Stack<N>` — hence `static_cell`.
//! - Core 0 (PRO_CPU): USB comms, GCode parser, planner, TMC manager, spindle, status reporter,
//!   homing on a thread-mode `esp_rtos::embassy::Executor`.
//!
//! Status: this file is a documented entry-point skeleton. It cannot be compiled on the host: this
//! crate targets `xtensa-esp32s3-none-elf` via its own `.cargo/config.toml` `[build] target` and is
//! kept out of the workspace `default-members`, so root `cargo build`/`cargo test` skip it (see the
//! mechanism note in this crate's `Cargo.toml`). It is left unimplemented until firmware bring-up
//! (DOC build order step 5); the pulled-in crate set is fixed and the body lands once the Espressif
//! toolchain (espup) is available on the build host.

// esp-backtrace installs the panic handler and exception/backtrace reporting (panic-handler feature).
use esp_backtrace as _;

// TODO(DOC-00): bring up clocks, esp-storage `$`-settings load, USB Serial/JTAG, UART1 (TMC bus),
// RMT TX channels (X/Y/Z), LEDC spindle PWM, then start esp_rtos with the Embassy feature and spawn
// the core-0 thread-mode executor tasks plus the core-1 InterruptExecutor `motion_executor`.
#[esp_hal::main]
fn main() -> ! {
  // TODO(DOC-00): peripheral init + esp_rtos::start(...) + executor spawn. Unimplemented until the
  // Espressif Xtensa toolchain is installed on the build host (see crate README / CLAUDE.md).
  loop {}
}
