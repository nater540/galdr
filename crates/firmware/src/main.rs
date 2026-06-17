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
//! - LEDC spindle PWM (DOC-07), limit/control GPIO + homing (DOC-06), and esp-storage `$`-settings
//!   persistence (DOC-00): in-memory defaults are used; the `$0`/`$29`/steps-mm the motion executor needs
//!   come from the `comms` placeholder accessors, and the TMC2209 currents/microstepping from `TmcConfig`
//!   defaults. Each is a documented TODO below.
//! - The TMC2209 manager (DOC-03) IS now brought up: UART1 on GPIO9 carries the single-wire bus and the
//!   `tmc_manager` task configures each driver and polls `DRV_STATUS`. A driver fault does not yet raise a
//!   machine ALARM — that waits on the shared alarm-state machine (DOC-06), a documented TODO in `tmc`.
//! - Feed-hold deceleration is per-block (Stage 1): a hold pauses at the next block boundary and resumes on
//!   cycle-start; smooth ramp-down within a block is a later refinement.

// esp-backtrace installs the panic handler and exception/backtrace reporting (panic-handler feature).
use esp_backtrace as _;
// esp-println routes `print!`/`println!` to the host; pulled in for early-boot diagnostics before the
// USB CDC link is up. Linked unconditionally so its initializer runs even when unused here.
use esp_println as _;

use embassy_executor::Spawner;
use embassy_sync::mutex::Mutex;
use esp_hal::clock::CpuClock;
use esp_hal::interrupt::Priority;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::system::Stack;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::usb_serial_jtag::UsbSerialJtag;
use esp_rtos::embassy::InterruptExecutor;
use esp_storage::FlashStorage;
use static_cell::StaticCell;

use firmware_core::motion::MotionConfig;

mod comms;
mod motion;
mod storage;
mod tmc;

esp_bootloader_esp_idf::esp_app_desc!();

/// The step-generator timer tick rate, Hz. A fixed firmware constant (the RMT channels are clocked to 1 MHz
/// = 1 tick/µs by `motion::init`'s clock divider); the persisted `$0` step-pulse time is converted to ticks
/// at this rate. Not a user setting — keep it in agreement with the divider in `motion::init`.
const MOTION_TICK_HZ: f32 = 1_000_000.0;

// Provide the defmt timestamp source required to link a defmt logging build (the `defmt` feature wires
// esp-println as the global logger; defmt still requires the application to supply a timestamp). Uses the
// embassy-time monotonic clock so log lines carry a microsecond stamp. Compiled out of the default build.
#[cfg(feature = "defmt")]
defmt::timestamp!("{=u64:us}", embassy_time::Instant::now().as_micros());

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

/// The PROBE input (GPIO21), owned for the program's lifetime so the pin stays configured (dropping the `Input`
/// would release it). Borrowed mutably by the `motion_executor` task, which samples it during a `G38.x` cycle.
static PROBE_INPUT: StaticCell<motion::RmtProbeInput> = StaticCell::new();

/// The STEP_EN (GPIO8) output, parked in a `StaticCell` so the pin stays driven for the program's lifetime
/// (dropping the `Output` would release the pin and let the drivers float). Driven enabled (low) at init.
static STEP_ENABLE: StaticCell<esp_hal::gpio::Output<'static>> = StaticCell::new();

/// The single flash instance plus its persistent pointer cache ([`storage::FlashState`]) behind its
/// cross-core mutex, parked in a `StaticCell` so it lives for the program and can be shared as `&'static` with
/// the settings store at boot and the coalesced persist path. `esp_storage::FlashStorage::new` panics if
/// constructed twice, so there is exactly one, created here.
static FLASH: StaticCell<storage::SharedFlash> = StaticCell::new();

/// The core-1 `motion_executor` task: the single task on the high-priority interrupt executor. It borrows
/// the `'static` RMT step sink and runs the real-time step-generation loop forever (DOC-02). Defined here
/// (not in `motion`) because `#[embassy_executor::task]` must own its `'static` argument; the loop body
/// lives in [`motion::run`].
#[embassy_executor::task]
async fn motion_executor(
  sink: &'static mut motion::RmtStepSink,
  probe: &'static mut motion::RmtProbeInput,
  config: MotionConfig,
  max_rate_mm_min: [f32; firmware_core::planner::AXES],
) -> ! {
  motion::run(sink, probe, config, max_rate_mm_min).await
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

  // 3b. Settings persistence (DOC-04). Construct the single flash instance, load the persisted `$`-settings
  //     record — the loader is INFALLIBLE: absence, corruption, or schema skew silently yields compiled
  //     defaults, so a bad flash region can never wedge boot — seed the live `SETTINGS`, and derive the
  //     planner / motion / TMC configs from it. The flash is shared `&'static` so the consumer can persist
  //     `$x=val` writes at runtime.
  let flash: &'static storage::SharedFlash =
    FLASH.init(Mutex::new(storage::FlashState::new(FlashStorage::new(peripherals.FLASH))));
  let settings = {
    let mut store = storage::FlashRecordStore::settings(flash);
    firmware_core::settings::load_or_default(&mut store).await
  };
  let planner_config = settings.planner_config();
  let motion_config = settings.motion_config(MOTION_TICK_HZ);
  let tmc_config = settings.tmc_config();
  let homing_enabled = settings.homing_enable;
  // Phase F: capture the persisted `$481` auto-report interval (clamped) before `settings` is moved into the
  // shared cell, so the auto-report task starts at the configured cadence.
  let auto_report_interval = settings.auto_report_interval_ms();
  comms::init_settings(settings);
  // Seed the cached status config (steps/mm, `$10` MPos/WPos mode, feed ceiling) so the first `?` report reads
  // them without locking the async `SETTINGS` mutex; refreshed at every settings-commit site thereafter.
  comms::init_status_cfg(&settings);
  // Seed the live control state from `$22` (DOC-08 Stage 2): the machine boots LOCKED in `ALARM:11` (homing
  // required) when homing is enabled — a host must `$H`/`$X` before streaming — else boots Idle. The boot
  // `ALARM:N` push is emitted after the banner below so a sender detects the locked state on connect.
  comms::init_control_state(homing_enabled);
  // Phase F: seed the live auto-report cadence mirror from the persisted `$481` so the auto-report task pushes
  // periodic status reports at the configured interval (a no-op when `$481=0`, the default).
  comms::init_auto_report(auto_report_interval);

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
    &motion_config,
  );
  let sink: &'static mut motion::RmtStepSink = STEP_SINK.init(sink);
  let _: &'static mut _ = STEP_ENABLE.init(step_enable);

  // 4c. Bring up the PROBE input on GPIO21 (DOC-09, Phase C) for the `G38.x` Z-probe / touch-off workflow. The
  //     internal pull-up follows `$19` (enabled unless `$19=1`); `$6` invert is applied per-sample during the
  //     probe cycle. Parked in a `StaticCell` so the pin stays configured for the program's lifetime and the
  //     `motion_executor` task can borrow it `'static`.
  let probe = motion::init_probe(peripherals.GPIO21, &settings.probe_config());
  let probe: &'static mut motion::RmtProbeInput = PROBE_INPUT.init(probe);

  // 4b. Bring up UART1 as the single-wire TMC2209 bus on GPIO9 (DOC-03). The bus is owned by the
  //     `tmc_manager` task (spawned below), which runs the per-driver init sequence and then polls
  //     `DRV_STATUS`. Built here so its peripherals (UART1 + GPIO9) are claimed alongside the others.
  let tmc_bus = tmc::init(peripherals.UART1, peripherals.GPIO9);

  // 5. Install the motion planner (built from the loaded settings) before spawning the tasks that share it
  //    (the consumer enqueues, the core-1 executor pops). `Planner::new` is not `const`, so the static holds
  //    an `Option` filled here.
  comms::init_planner(planner_config);

  // 5b. Coordinate data persistence (Phase B, DOC-08 `$#`). Load the persisted G54-G59 / G28 / G30 record — the
  //     loader is INFALLIBLE (absence / corruption / schema skew yields all-zero defaults) — seed the live
  //     coordinate model (session-only G92 / TLO start at identity), and push the resulting WCO into the planner
  //     so absolute work moves resolve against the stored offsets from the first line. Uses its own NVS key
  //     under the same flash as the settings record.
  let coordinates = {
    let mut store = storage::FlashRecordStore::coordinates(flash);
    firmware_core::coords::load_or_default(&mut store).await
  };
  comms::init_coordinates(coordinates);
  comms::seed_planner_work_offset().await;

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
      // The axis max-rates (`$110-112`) bound the Phase-E feed-override scale-up so a boosted feed never exceeds
      // the configured rate limit; pass them to the executor alongside the step-timing config.
      motion_spawner.must_spawn(motion_executor(sink, probe, motion_config, planner_config.max_rate_mm_min));
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
  // The consumer takes the shared flash so a `$x=val` setting write is persisted to the NVS region (DOC-04).
  spawner.must_spawn(comms::comms_consumer(flash));
  spawner.must_spawn(comms::status_responder());
  // The auto-report task (Phase F, DOC-08 §5) pushes a `<...>` status report every `$481`-ms when enabled,
  // reusing `status_responder`'s single formatter/writer so an auto-report is byte-identical to a `?` report.
  spawner.must_spawn(comms::auto_report_task());
  // The TMC2209 manager runs the driver init sequence at startup (from the loaded settings), then polls
  // DRV_STATUS for faults. It owns the UART1 bus by value (a `'static` peripheral handle), so no `StaticCell`
  // is needed (DOC-03).
  spawner.must_spawn(tmc::tmc_manager(tmc_bus, tmc_config));

  // 8. Emit the welcome banner on boot so a host detects readiness immediately (native USB cannot be
  //    hard-reset by the host). The banner is also re-emitted on every soft reset (by the consumer's
  //    pipeline reset, with a best-effort copy from the reader half).
  comms::send_banner().await;
  // If homing is enabled the machine booted locked in `ALARM:11`; push the boot alarm + `[MSG:'$H'|'$X' to
  // unlock]` right after the banner so a sender detects the locked state on connect (DOC-08 §5). When homing
  // is disabled this is a no-op and the machine comes up Idle.
  comms::send_boot_alarm().await;

  // The executor keeps the spawned tasks running; this initial task has nothing left to do. Awaiting a
  // never-completing future parks it without burning the CPU in a busy loop.
  core::future::pending::<()>().await;
}
