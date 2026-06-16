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
//! ## Dual-core split (DOC-01 / DOC-02)
//! - **Core 1 (APP_CPU)** runs ONLY the `motion_executor` (in [`motion`]) on a high-priority
//!   [`InterruptExecutor`](esp_rtos::embassy::InterruptExecutor) at `Priority::Priority3`, so real-time RMT
//!   step generation preempts nothing and gets uncontested CPU. The second core is started with
//!   `esp_rtos::start_second_core`, which on Xtensa consumes software interrupts 0 and 1 for the esp-rtos
//!   SMP scheduler; the motion interrupt executor therefore uses the next free software interrupt, SWI 2.
//! - **Core 0 (PRO_CPU)** runs everything else (the USB comms / parser / planner pipeline in [`comms`]) on
//!   the `#[esp_rtos::main]` thread-mode executor.
//!
//! ## What this phase brings up (DOC-02 motion executor)
//! - The RMT TX step channels (ch0/1/2 on GPIO1/2/4), the DIR outputs (GPIO5/6/7), and STEP_EN (GPIO8,
//!   driven enabled at init), assembled into the [`motion::RmtStepSink`].
//! - The second core and its interrupt executor, running the `motion_executor` task that drains the shared
//!   planner queue, realizes each block as RMT pulses, and publishes the live MPos into `comms::MACHINE`.
//!   This REPLACES the Stage-1 `block_drain_stub`; the `PLANNER`/`MACHINE` handoff is unchanged.
//!
//! ## What is deliberately still stubbed (out of scope for this phase)
//! - TMC2209 manager / UART1 (DOC-03), LEDC spindle PWM (DOC-07), limit/control GPIO + homing (DOC-06),
//!   and esp-storage `$`-settings persistence (DOC-00): in-memory defaults are used; the `$0`/`$29`/steps-mm
//!   the motion executor needs come from the `comms` placeholder accessors. Each is a documented TODO below.
//! - Feed-hold deceleration is per-block (Stage 1): a hold pauses at the next block boundary and resumes on
//!   cycle-start; smooth ramp-down within a block is a later refinement.

// esp-backtrace installs the panic handler and exception/backtrace reporting (panic-handler feature).
use esp_backtrace as _;
// esp-println routes `print!`/`println!` to the host; pulled in for early-boot diagnostics before the
// USB CDC link is up. Linked unconditionally so its initializer runs even when unused here.
use esp_println as _;

use embassy_executor::Spawner;
use esp_hal::clock::CpuClock;
use esp_hal::interrupt::Priority;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::system::Stack;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::usb_serial_jtag::UsbSerialJtag;
use esp_rtos::embassy::InterruptExecutor;
use static_cell::StaticCell;

mod comms;
mod motion;

/// Stack arena for the core-1 main thread (the esp-rtos second-core scheduler thread). The motion executor
/// itself runs in interrupt context off this thread, so this only backs the brief second-core bring-up and
/// its idle loop; 8 KiB is the DOC-01 starting estimate. Tune with stack-painting under load.
static APP_CORE_STACK: StaticCell<Stack<8192>> = StaticCell::new();

/// The core-1 motion interrupt executor. Held in a `StaticCell` because `InterruptExecutor::start` needs
/// `&'static mut self` (the executor must outlive every task it runs). Uses software interrupt 2: SWI 0 and
/// 1 are claimed by the esp-rtos SMP scheduler (`start` + `start_second_core`), so 2 is the first free one.
static MOTION_EXECUTOR: StaticCell<InterruptExecutor<2>> = StaticCell::new();

/// The RMT step sink, owned for the program's lifetime so its RMT channels and DIR outputs are never
/// dropped. It is borrowed mutably by the long-running `motion_executor` task; a `StaticCell` gives that
/// task a `&'static mut` without a heap allocation.
static STEP_SINK: StaticCell<motion::RmtStepSink> = StaticCell::new();

/// The STEP_EN (GPIO8) output, parked in a `StaticCell` so the pin stays driven for the program's lifetime
/// (dropping the `Output` would release the pin and let the drivers float). Driven enabled (low) at init.
static STEP_ENABLE: StaticCell<esp_hal::gpio::Output<'static>> = StaticCell::new();

/// The core-1 `motion_executor` task: the single task on the high-priority interrupt executor. It borrows
/// the `'static` RMT step sink and runs the real-time step-generation loop forever (DOC-02). Defined here
/// (not in `motion`) because `#[embassy_executor::task]` must own its `'static` argument; the loop body
/// lives in [`motion::run`].
#[embassy_executor::task]
async fn motion_executor(sink: &'static mut motion::RmtStepSink) -> ! {
  motion::run(sink).await
}

/// Async entry point. `#[esp_rtos::main]` expands to an `#[esp_hal::main]` reset handler that builds the
/// core-0 thread-mode `esp_rtos::embassy::Executor` and runs this function as its first task. We start
/// the esp-rtos scheduler/time-driver before spawning so `embassy-time` and channel awaits work.
#[esp_rtos::main]
async fn main(spawner: Spawner) {
  // 1. Clocks at max (240 MHz on the S3) + peripheral handle. `expect`/`init` failures here are
  //    unrecoverable at the very first init step (CLAUDE.md allows `expect` in init).
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
  // failure); build the MotionConfig from `$0`/`$29` and the PlannerConfig from `$100..`. Stage 1 uses the
  // in-memory `placeholder_*` defaults in `comms` (planner config, motion config, steps/mm).
  // TODO(DOC-03): configure UART1 for the TMC2209 single-wire bus and run the tmc_manager init sequence.
  // TODO(DOC-07): configure LEDC ch0 on GPIO13 for spindle PWM + SPIN_EN/SPIN_DIR GPIO (act on the
  //   planner's Spindle outcome, currently passed through).
  // TODO(DOC-06): configure limit/control GPIO (pull-ups, rising-edge IRQ) and spawn the homing task (act
  //   on the planner's GoToPredefined outcome, currently passed through).

  // 4. Bring up the RMT step channels + DIR/STEP_EN GPIO and build the step sink. STEP_EN is driven enabled
  //    (active-low → low) so the steppers hold. Both the sink and STEP_EN are parked in `StaticCell`s so
  //    their RMT channels / pins live for the program's lifetime (the motion task borrows the sink `'static`).
  let (sink, step_enable) = motion::init(
    peripherals.RMT,
    (peripherals.GPIO1, peripherals.GPIO2, peripherals.GPIO4),
    (peripherals.GPIO5, peripherals.GPIO6, peripherals.GPIO7),
    peripherals.GPIO8,
  );
  let sink: &'static mut motion::RmtStepSink = STEP_SINK.init(sink);
  let _: &'static mut _ = STEP_ENABLE.init(step_enable);

  // 5. Install the motion planner before spawning the tasks that share it (the consumer enqueues, the
  //    core-1 executor pops). `Planner::new` is not `const`, so the static holds an `Option` filled here.
  comms::init_planner();

  // 6. Start core 1 and its high-priority interrupt executor, then spawn `motion_executor` on it. On Xtensa
  //    `start_second_core` consumes software interrupts 0 and 1 for the esp-rtos SMP scheduler; the motion
  //    interrupt executor therefore uses SWI 2. The `func` closure runs on core 1 (pinned): it starts the
  //    interrupt executor — registering the SWI-2 handler on core 1 so the task runs there — spawns the
  //    task, then returns (esp-rtos idles the core-1 main thread; the interrupt executor keeps running).
  let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
  let app_stack = APP_CORE_STACK.init(Stack::new());
  esp_rtos::start_second_core(
    peripherals.CPU_CTRL,
    sw_int.software_interrupt0,
    sw_int.software_interrupt1,
    app_stack,
    move || {
      let executor = MOTION_EXECUTOR.init(InterruptExecutor::new(sw_int.software_interrupt2));
      let motion_spawner = executor.start(Priority::Priority3);
      // `must_spawn` is appropriate at init: a spawn failure (token already used) is a static, unrecoverable
      // wiring bug, not a runtime condition. The task takes the `'static` sink by mutable borrow.
      motion_spawner.must_spawn(motion_executor(sink));
    },
  );

  // 7. Spawn the core-0 comms tasks on the thread-mode executor. `must_spawn` is appropriate at init (a
  //    spawn failure is a static wiring bug). The RX path is split: `usb_rx` (reader half) extracts real-time
  //    bytes and buffers the rest into `RX_PIPE`, while `line_assembler` frames lines from that buffer — so
  //    real-time commands never block behind line back-pressure (DOC-08 grbl ISR model). `comms_consumer` is
  //    the real parser → planner pipeline; it now wakes the core-1 motion executor via `BLOCK_AVAILABLE`.
  spawner.must_spawn(comms::usb_rx(usb_rx));
  spawner.must_spawn(comms::line_assembler());
  spawner.must_spawn(comms::usb_tx(usb_tx));
  spawner.must_spawn(comms::comms_consumer());
  spawner.must_spawn(comms::status_responder());

  // 8. Emit the welcome banner on boot so a host detects readiness immediately (native USB cannot be
  //    hard-reset by the host). The banner is also re-emitted on every soft reset (by the consumer's
  //    pipeline reset, with a best-effort copy from the reader half).
  comms::send_banner().await;

  // The executor keeps the spawned tasks running; this initial task has nothing left to do. Awaiting a
  // never-completing future parks it without burning the CPU in a busy loop.
  core::future::pending::<()>().await;
}
