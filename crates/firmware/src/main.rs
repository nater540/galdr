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

// esp-backtrace is pulled in (its `panic-handler` feature is OFF — we install our own `#[panic_handler]` below
// that records a breadcrumb + `software_reset()`s; see Cargo.toml). Kept linked for its `println` integration; the
// CPU exception vector lives in esp-hal and routes faults into our handler, so faults are still captured.
use esp_backtrace as _;
// esp-println routes `print!`/`println!` to the host; pulled in for early-boot diagnostics before the
// USB CDC link is up. Linked unconditionally so its initializer runs even when unused here.
use esp_println as _;

use embassy_executor::Spawner;
use embassy_sync::mutex::Mutex;
use esp_hal::clock::CpuClock;
use esp_hal::interrupt::Priority;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::rtc_cntl::{reset_reason, Rtc, RwdtStage};
use esp_hal::system::{Cpu, Stack};
use esp_hal::time::Duration;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::usb_serial_jtag::UsbSerialJtag;
use esp_rtos::embassy::InterruptExecutor;
use esp_storage::FlashStorage;
use static_cell::StaticCell;

use firmware_core::motion::MotionConfig;

mod comms;
mod coolant;
mod crash;
mod motion;
mod spindle;
mod storage;
// The survivable-watchdog TIMG1 ISR + dual-dog feed (DIAGNOSTIC-only, `capture-reset`-gated, §17.10/§17.11). Absent
// from the production default build, which keeps the core-0 async `watchdog_feed` + the ALARM:17 fail-safe unchanged.
#[cfg(feature = "capture-reset")]
mod survivable_watchdog;
mod tmc;

esp_bootloader_esp_idf::esp_app_desc!();

/// Custom panic handler (replacing esp-backtrace's default `interrupt_free(|| loop {})`, which halts FOREVER with
/// no breadcrumb — making EVERY panic a silent, unrecoverable hard wedge). This records a PANIC breadcrumb into
/// RTC_FAST and then `software_reset()`s, so a panic — including a core-1 stack overflow, the prime suspect for the
/// hard wedge — becomes VISIBLE (the next boot emits `[MSG:CRASH panic <file>:<line> core=<N>]`) and RECOVERABLE
/// (the chip resets itself instead of needing a physical EN press).
///
/// ## MINIMAL STACK — this may run after a STACK OVERFLOW
/// A core-1 main-stack overflow panics, and this handler then runs on that SAME nearly-exhausted stack (there is no
/// separate Xtensa interrupt/panic stack). So it MUST be allocation-, lock-, and format-FREE: it does only a
/// handful of raw `Relaxed` stores (via [`crash::record_panic`], `#[inline(never)]` to keep one flat frame) and one
/// ROM `software_reset()` call. The source-file string is NOT formatted here — only its `.rodata` POINTER + length
/// are stored as raw words and re-read at BOOT (full stack). No `esp_println` here: it allocates a format buffer /
/// takes a UART lock, neither safe on a blown stack.
///
/// `software_reset()` (ROM `RTC_CNTL_SW_SYS_RST`, reset reason `CoreSw`) is a full-chip reset safe from a panic
/// context on EITHER core, and it PRESERVES the RTC_FAST breadcrumb (verified: the RTC domain is not cleared by a
/// CoreSw reset — only a power-cycle/brown-out clears it), so the breadcrumb survives to the next boot.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
  // `Cpu::current()` is a single CPU-register read (no peripheral, no lock) — safe here. 0 = ProCpu, 1 = AppCpu.
  let core = Cpu::current() as u8;
  // Pull the raw location words WITHOUT formatting. The file `&str` is a `.rodata` literal; store its pointer+len.
  if let Some(loc) = info.location() {
    let file = loc.file();
    crash::record_panic(core, file.as_ptr() as u32, file.len() as u32, loc.line(), true);
  } else {
    crash::record_panic(core, 0, 0, 0, false);
  }
  esp_hal::system::software_reset();
}

/// The step-generator timer tick rate, Hz. A fixed firmware constant (the RMT channels are clocked to 1 MHz
/// = 1 tick/µs by `motion::init`'s clock divider); the persisted `$0` step-pulse time is converted to ticks
/// at this rate. Not a user setting — keep it in agreement with the divider in `motion::init`.
const MOTION_TICK_HZ: f32 = 1_000_000.0;

/// The priority the core-1 motion [`InterruptExecutor`] runs at (DOC-02: real-time step generation preempts
/// the idle second-core thread). Passed to `InterruptExecutor::<2>::start`, which routes software interrupt 2
/// (`FROM_CPU_INTR2` — SWI 0/1 are claimed by the esp-rtos SMP scheduler) to a CPU interrupt at this level.
const MOTION_EXECUTOR_PRIORITY: Priority = Priority::Priority3;

/// RTC watchdog stage-0 timeout: how long the core-0 thread-mode executor may go WITHOUT running the
/// [`comms::watchdog_feed`] task before the RWDT resets the chip. Chosen at 8 s — comfortably above every
/// legitimate core-0 stall so it can NEVER false-trip during normal streaming, yet short enough that a genuine
/// wedge auto-recovers in seconds instead of needing a hard power-cycle:
/// - The feed task pets the dog every `WATCHDOG_FEED_INTERVAL` (500 ms), so 16 feeds fall inside one window — a
///   single late wake from a brief quiesce (a flash write parks core 0 for tens of ms via `multicore_auto_park`,
///   a `?`/banner burst, a back-pressure retry) is dwarfed by the margin.
/// - The longest legitimate core-0 stall is a flash erase/write (tens of ms) and the consumer's coalesced persist;
///   none approaches a second, let alone 8. Feed-hold / `$SLP` park the MOTION core, not the core-0 feed task.
///
/// Only a true core-0 hang/deadlock/fault — the reported lockup's core-0 symptom — keeps the task from running for
/// 8 s, which is exactly when we WANT the reset. The RWDT runs off the slow RTC clock, so multi-second timeouts are
/// well within range. Stage 0's default action on expiry is a system reset (we do not reconfigure the stage action).
const WATCHDOG_TIMEOUT: Duration = Duration::from_secs(8);

// Provide the defmt timestamp source required to link a defmt logging build (the `defmt` feature wires
// esp-println as the global logger; defmt still requires the application to supply a timestamp). Uses the
// embassy-time monotonic clock so log lines carry a microsecond stamp. Compiled out of the default build.
#[cfg(feature = "defmt")]
defmt::timestamp!("{=u64:us}", embassy_time::Instant::now().as_micros());

/// Size of the core-1 stack arena, in bytes. Must be a multiple of 16 (`Stack::new` const-asserts this).
///
/// ## This stack backs the WHOLE core-1 call chain, including the real-time motion task
/// On Xtensa there is NO separate interrupt stack: a software-interrupt handler runs on the interrupted thread's
/// stack, and on core 1 the current thread at idle is the esp-rtos second-core MAIN thread backed by THIS arena.
/// So the SWI-2 [`InterruptExecutor`] poll — and therefore the entire `motion_executor` call chain (the drain loop
/// → `run_block` → the segment generator → `RmtStepSink::emit_burst` → esp-hal RMT `transmit`/`poll`/`wait` → the
/// CCOUNT-bounded timeout + the crash-breadcrumb stores) — executes ON THIS STACK, charged ON TOP of the scheduler
/// thread's frames. esp-rtos plants a stack guard at `bottom + ESP_HAL_CONFIG_STACK_GUARD_OFFSET` and asserts
/// (panics) on overflow, and ALSO panics on core 0 if the second-core main task fails to init from a bring-up
/// overflow — so a deep-enough core-1 frame trips a panic (now visible via the custom `#[panic_handler]`, the prime
/// hard-wedge suspect).
///
/// ## Sizing: 32 KiB (was 16 KiB) — depth headroom under sustained streaming
/// Bumped 16→32 KiB as cheap insurance against a streaming-load stack overflow in the motion task. The usable depth
/// is `SIZE − guard_offset`; doubling it gives comfortable margin for the deepest core-1 frame (the RMT
/// transmit/wait path under the Xtensa windowed ABI, which is stack-hungry on window-overflow spills) plus the
/// breadcrumb/CCOUNT code added to that path, with room for any `core::fmt` the panic machinery itself pulls in.
/// 32 KiB of RTC/internal RAM is a negligible cost on the S3. Note the over-top corruption bug fixed by
/// [`AppCoreStackArena`]'s `_abi_headroom` was NOT a depth overflow (it reproduced identically at any size), so the
/// trailing-padding/canary fix remains independent and load-bearing — growing the stack does not replace it.
const APP_CORE_STACK_SIZE: usize = 32 * 1024;

/// Xtensa windowed-ABI base register save area, in bytes. Every call spills the caller's saved registers into a
/// 16-byte area at `[SP, SP+16)` (the four words holding the return address and the caller's a0..a3 window slot).
/// This is the minimum that the FIRST core-1 frame writes ABOVE the initial SP when esp-hal sets `SP = top()`.
const XTENSA_BASE_SAVE_AREA: usize = 16;

/// Xtensa windowed-ABI EXTRA register save area, in bytes: the worst-case window-overflow spill the handler emits
/// on a `CALL12`, saving the 12-register live frame a4..a15 (12 x 4 = 48 bytes) when the rotated window wraps. A
/// first-frame `CALL12` overflow is the largest store that can land ABOVE the initial SP, so the worst-case
/// over-top reach is base + extra, not just the 16-byte base save area.
const XTENSA_EXTRA_SAVE_AREA: usize = 48;

/// Worst-case number of bytes the Xtensa windowed ABI can spill ABOVE the initial SP on the first core-1 frame:
/// the 16-byte base save area plus the 48-byte `CALL12` window-overflow save area = 64 bytes. The ABI headroom
/// MUST be at least this large or a legitimate first-frame spill reaches past the padding into adjacent statics.
const XTENSA_MAX_OVERTOP_SPILL: usize = XTENSA_BASE_SAVE_AREA + XTENSA_EXTRA_SAVE_AREA;

/// ABI scratch headroom, in bytes, placed IMMEDIATELY above the core-1 stack to absorb the Xtensa windowed-ABI
/// over-top register-save spill (see [`AppCoreStackArena`]). Sized at 2x the worst-case over-top spill so the
/// LOWER half `[0, XTENSA_MAX_OVERTOP_SPILL)` absorbs any legitimate first-frame spill while the UPPER half
/// `[XTENSA_MAX_OVERTOP_SPILL, APP_CORE_ABI_HEADROOM)` stays untouched and can hold a regression canary (see
/// [`AppCoreStackArena::arm_canary`]): a disturbed canary means a spill reached past the worst case we modeled
/// and the headroom must grow. The doubling also covers any alignment slop the linker inserts before the field.
const APP_CORE_ABI_HEADROOM: usize = XTENSA_MAX_OVERTOP_SPILL * 2;

/// The headroom must never be sized below the ABI worst-case over-top spill, or a legitimate first-frame spill
/// corrupts adjacent statics again. This guards the size against silent regression if the consts above change.
const _: () = assert!(APP_CORE_ABI_HEADROOM >= XTENSA_MAX_OVERTOP_SPILL);

/// Sentinel byte written across the canary zone (the upper half of `_abi_headroom`) before core-1 bring-up. After
/// bring-up we read it back: any byte that differs means the over-top spill reached into the canary zone, i.e. it
/// exceeded [`XTENSA_MAX_OVERTOP_SPILL`] and the headroom is too small. `0xA5` is a non-zero, non-`0xFF` pattern
/// distinct from the zeroed stack/`.bss` fill so an accidental zero-store also trips the check.
const APP_CORE_CANARY_BYTE: u8 = 0xA5;

/// Stack arena for the core-1 main thread (the esp-rtos second-core scheduler thread), with a trailing
/// ABI-headroom guard. The `motion_executor` task runs in interrupt context (the SWI-2 `InterruptExecutor`), so
/// at steady state this stack only backs the idle scheduler loop; but the WHOLE second-core bring-up runs on it
/// (esp-rtos `setup_smp` + `allocate_main_task` + `yield_task`, then our `start_second_core` closure).
///
/// ## REGRESSION GUARD — do NOT remove `_abi_headroom` (root cause: Xtensa windowed-ABI over-top spill)
/// `Stack<SIZE>` is `{ mem: MaybeUninit<[u8; SIZE]> }` with no trailing field, and `Stack::top()` returns
/// `bottom + SIZE` bytes = ONE-PAST-THE-END of the array. The app-core boot vector `start_core1_init` sets
/// `SP = top()` and esp-hal reserves NO headroom above it. On Xtensa's windowed ABI the FIRST core-1 frame
/// spills the caller's 16-byte register save area into `[SP, SP+16)` — i.e. 16 bytes ABOVE the array, onto
/// whatever the linker placed next. With a bare `Stack` that was the adjacent `.bss` control statics
/// (`MOTION_EXECUTOR` et al.), silently corrupting them and breaking the entire bring-up. This is NOT a depth
/// overflow (it happens on the first frame at any size) and esp-hal's bottom stack guard cannot see it (SP grows
/// DOWN, so the guard only fires on a full-depth underflow, never on the over-top neighbors).
///
/// The fix is LAYOUT, not luck: embedding `Stack` as the first field of a `#[repr(C)]` arena whose second field
/// is [`APP_CORE_ABI_HEADROOM`] bytes of padding makes `_abi_headroom` sit IMMEDIATELY above the stack top
/// (`top()` == `&_abi_headroom`, since `Stack` is `align(16)` at offset 0). The over-top spill then lands inside
/// that sacrificial padding and can never reach an unrelated static. Enlarging the stack does NOT fix this — it
/// only slides the neighbors up in lockstep — so the headroom field is load-bearing and must not be deleted.
/// [`AppCoreStackArena::arm_canary`] / [`AppCoreStackArena::check_canary`] both reference the field (so the linker
/// cannot prove it dead) AND turn its upper half into a regression signal for an over-headroom spill.
#[repr(C)]
struct AppCoreStackArena {
  /// The actual core-1 stack. `start_second_core` takes `&'static mut Stack`, so we hand it `&mut self.stack`.
  stack: Stack<APP_CORE_STACK_SIZE>,
  /// Sacrificial padding that sits immediately above the stack top (`stack.top()` == `&_abi_headroom`) to absorb
  /// the Xtensa windowed-ABI over-top register-save spill described above. Its lower half is the spill landing
  /// zone (value never relied upon); its upper half carries the regression canary (see [`Self::arm_canary`]).
  _abi_headroom: [u8; APP_CORE_ABI_HEADROOM],
}

impl AppCoreStackArena {
  /// Construct the arena: an uninitialized stack plus zeroed headroom padding. `const` so it can initialize the
  /// `StaticCell` without a runtime initializer beyond `StaticCell::init`.
  const fn new() -> Self {
    Self { stack: Stack::new(), _abi_headroom: [0; APP_CORE_ABI_HEADROOM] }
  }

  /// Index where the canary zone begins within `_abi_headroom`. Bytes BELOW this (`[0, XTENSA_MAX_OVERTOP_SPILL)`)
  /// are the spill zone a legitimate first-frame over-top spill MAY write, so they must NOT be canaried or every
  /// boot would false-trip. Bytes from here up are the canary zone, which a correctly-sized headroom never reaches.
  const CANARY_START: usize = XTENSA_MAX_OVERTOP_SPILL;

  /// Write the sentinel pattern across the canary zone (the upper half of `_abi_headroom`, FARTHEST from the stack
  /// top) BEFORE `start_second_core`. The canary deliberately sits above the worst-case spill zone so a legitimate
  /// in-bounds spill — which lands in the LOWER half — never disturbs it; only a spill that exceeds the modeled
  /// worst case reaches up here. Volatile stores so neither the compiler nor the (cross-core) reader path can
  /// assume the bytes are dead. References the field, replacing the old cargo-cult keep-alive read.
  fn arm_canary(&mut self) {
    // SAFETY: `_abi_headroom` is a live, in-bounds field of `self`; we write only indices in the canary zone,
    // which are within bounds since `CANARY_START < APP_CORE_ABI_HEADROOM` (guaranteed by the 2x sizing).
    for byte in &mut self._abi_headroom[Self::CANARY_START..] {
      unsafe { core::ptr::write_volatile(byte, APP_CORE_CANARY_BYTE) };
    }
  }

  /// Read the canary zone back AFTER core-1 bring-up has run. By the time `start_second_core` returns on core 0,
  /// the first core-1 frame (and the whole bring-up closure) has executed, so the windowed-ABI over-top spill has
  /// already happened. Returns `true` if every canary byte is intact (no over-headroom spill); `false` means a
  /// spill reached into the canary zone — the headroom is undersized and [`APP_CORE_ABI_HEADROOM`] must grow.
  fn check_canary(&self) -> bool {
    // SAFETY: same field, same in-bounds canary-zone indices; a volatile read defeats the optimizer assuming the
    // cross-core writer left the bytes unchanged. We compare each byte against the armed sentinel.
    self._abi_headroom[Self::CANARY_START..]
      .iter()
      .all(|byte| unsafe { core::ptr::read_volatile(byte) } == APP_CORE_CANARY_BYTE)
  }
}

/// The core-1 stack arena (stack + trailing ABI headroom). See [`AppCoreStackArena`] for why the trailing
/// headroom field is load-bearing and must not be removed.
static APP_CORE_STACK: StaticCell<AppCoreStackArena> = StaticCell::new();

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

/// The X/Y/Z LIMIT inputs (GPIO10/11/12), owned for the program's lifetime so the pins stay configured with
/// their pull-ups. Borrowed mutably by the `motion_executor` task, which samples them during a `$H` homing cycle
/// (DOC-06). The hard-limit monitor reads the same pins for the runtime `$21` alarm path.
static LIMIT_INPUTS: StaticCell<[motion::RmtLimitInput; firmware_core::planner::AXES]> = StaticCell::new();

/// The STEP_EN (GPIO8) output, parked in a `StaticCell` so the pin stays driven for the program's lifetime
/// (dropping the `Output` would release the pin and let the drivers float). Driven enabled (low) at init.
static STEP_ENABLE: StaticCell<esp_hal::gpio::Output<'static>> = StaticCell::new();

/// The LEDC controller, its low-speed timer0, and channel0 backing the spindle PWM (DOC-07). They form a
/// self-referential `'static` chain (channel borrows timer borrows controller), so each is parked in its own
/// `StaticCell`; `spindle::init` fills them and hands back sinks holding only `'static` references.
static SPINDLE_LEDC: StaticCell<esp_hal::ledc::Ledc<'static>> = StaticCell::new();
static SPINDLE_TIMER: StaticCell<esp_hal::ledc::timer::Timer<'static, esp_hal::ledc::LowSpeed>> = StaticCell::new();
static SPINDLE_CHANNEL: StaticCell<esp_hal::ledc::channel::Channel<'static, esp_hal::ledc::LowSpeed>> =
  StaticCell::new();

/// The spindle controller (LEDC PWM + SPIN_EN/SPIN_DIR GPIO), parked for the program's lifetime so its
/// peripherals stay configured. Borrowed mutably by the long-running `spindle` task (DOC-07), which owns it as
/// the single driver of the spindle outputs.
static SPINDLE: StaticCell<spindle::Spindle> = StaticCell::new();

/// The coolant controller (M7/M8/M9 over two — currently stubbed — coolant outputs), parked for the program's
/// lifetime and borrowed mutably by the long-running `coolant` task, which owns it as the single driver of the
/// coolant outputs. No coolant GPIO is budgeted yet (CLAUDE.md), so the underlying pins are stubs — see
/// [`coolant`](crate::coolant); the task topology and the all-off safety path are real now.
static COOLANT: StaticCell<coolant::Coolant> = StaticCell::new();

/// The single flash instance plus its persistent pointer cache ([`storage::FlashState`]) behind its
/// cross-core mutex, parked in a `StaticCell` so it lives for the program and can be shared as `&'static` with
/// the settings store at boot and the coalesced persist path. `esp_storage::FlashStorage::new` panics if
/// constructed twice, so there is exactly one, created here.
static FLASH: StaticCell<storage::SharedFlash> = StaticCell::new();

/// The RTC controller, parked in a `StaticCell` so it lives for the program: the [`comms::watchdog_feed`] task
/// borrows it `&'static mut` to pet the RWDT (`Rwdt::feed` takes `&mut self`). It is the SOLE owner/feeder of the
/// dog, so no lock is needed. Parked here (not dropped) because dropping `Rtc` would disable the watchdog.
static RTC: StaticCell<Rtc<'static>> = StaticCell::new();

/// The core-1 `motion_executor` task: the single task on the high-priority interrupt executor. It borrows
/// the `'static` RMT step sink and runs the real-time step-generation loop forever (DOC-02). Defined here
/// (not in `motion`) because `#[embassy_executor::task]` must own its `'static` argument; the loop body
/// lives in [`motion::run`].
#[embassy_executor::task]
async fn motion_executor(
  sink: &'static mut motion::RmtStepSink,
  probe: &'static mut motion::RmtProbeInput,
  limits: &'static mut [motion::RmtLimitInput; firmware_core::planner::AXES],
  config: MotionConfig,
  max_rate_mm_min: [f32; firmware_core::planner::AXES],
) -> ! {
  motion::run(sink, probe, limits, config, max_rate_mm_min).await
}

/// Map a [`SocResetReason`](esp_hal::rtc_cntl::SocResetReason) to a short, stable label for logging. Returning a
/// `&'static str` (rather than relying on a `Debug`/`defmt::Format` impl that may differ across esp-hal patch
/// versions) keeps the log line identical on both the default (`esp-println`) and `defmt` builds and makes the
/// lockup-relevant reasons unmistakable in the monitor. The watchdog reset we are hunting for is `CpuRtcWdt` /
/// `CoreRtcWdt`; a panic-handler software reset is `CpuSw` / `CoreSw`; a clean boot is `ChipPowerOn`. Unlisted
/// variants fall through to `"other"` so a future esp-hal addition never fails to compile this map.
fn reset_reason_label(reason: esp_hal::rtc_cntl::SocResetReason) -> &'static str {
  use esp_hal::rtc_cntl::SocResetReason::*;
  match reason {
    ChipPowerOn => "power-on",
    CoreSw => "core-sw-reset",
    CpuSw => "cpu0-sw-reset (panic/soft-reset)",
    CoreRtcWdt => "core-rtc-WDT (auto-recovered from a wedge)",
    CpuRtcWdt => "cpu0-rtc-WDT (auto-recovered from a wedge)",
    SysBrownOut => "brown-out (power)",
    CoreMwdt0 | CpuMwdt0 => "MWDT0 (timer watchdog)",
    CoreMwdt1 | CpuMwdt1 => "MWDT1 (timer watchdog)",
    SysSuperWdt => "super-WDT",
    SysRtcWdt => "sys-rtc-WDT",
    CoreDeepSleep => "deep-sleep wake",
    _ => "other",
  }
}

/// Read and log WHY the chip last reset, for the lockup hunt (see step 1b in `main`). Reads the PRO_CPU (core 0)
/// reason and, on this dual-core S3, the APP_CPU (core 1) reason too — a core-1 fault can show a distinct reason,
/// helping pin which core died. `reset_reason` returns `None` when the cause is not decodable; we report that as
/// `"unknown"` rather than guessing.
///
/// The log sink is chosen by build: a `defmt` build routes through `defmt::info!` (the RTT sink esp-println's
/// `defmt-espflash` feature installs — plain `println!` is taken over by that logger under `defmt`), while the
/// default build uses esp-println's `println!` so a plain serial monitor still sees the reason on the next boot.
/// Gating the two avoids any interaction between esp-println's plain and defmt back-ends.
/// Returns `true` when this reset reason is a WATCHDOG or FAULT reset — i.e. an unexpected, possibly-wedge-driven
/// reset whose crash breadcrumb is worth dumping — as opposed to a clean power-on / brown-out / deliberate
/// soft-reset. A clean power-on or brown-out also CLEARS RTC_FAST (so a valid breadcrumb could not survive one
/// anyway), but classifying here keeps the emit decision explicit and lets a defmt build label the cause.
fn reset_was_watchdog_or_fault(reason: Option<esp_hal::rtc_cntl::SocResetReason>) -> bool {
  use esp_hal::rtc_cntl::SocResetReason::*;
  matches!(
    reason,
    // RTC / timer / super watchdogs (the auto-recover path this firmware arms), plus a software/CPU reset (an
    // esp-backtrace panic handler issues one — the fault-handler-hang hypothesis), plus efuse-CRC / clock-glitch
    // faults. Variant names are the esp-hal 1.1.1 esp32s3 set (`CpuRtcWdt`/`CpuSw`/`CpuMwdt0`, NOT the generic-doc
    // `Cpu0*` names) — verified against the installed source.
    Some(
      CoreRtcWdt | CpuRtcWdt | SysRtcWdt | SysSuperWdt | CoreMwdt0 | CpuMwdt0 | CoreMwdt1 | CpuMwdt1 | CoreSw | CpuSw
        | CoreEfuseCrc | SysClkGlitch
    )
  )
}

/// Read WHY the chip last reset and return `(pro_label, was_watchdog_or_fault)`: the PRO_CPU (core 0) reason as a
/// stable `&'static str` label, plus whether it was a watchdog/fault reset (gating the breadcrumb dump). Also logs
/// both cores' reasons on the esp-println / defmt sink as before. The label is returned so `main` can ALSO emit it
/// over the grbl TX as a `[MSG:RESET ...]` line UNCONDITIONALLY at boot — so even a NO-breadcrumb boot (a clean
/// power-on, a brown-out that wiped RTC_FAST, or the Signature-B silent-lock case where nothing wrote a breadcrumb)
/// still tells skirnir WHY it reset. That single datum discriminates "a reset DID fire" (`*-sw-reset`/`*-rtc-WDT`)
/// from "no software reset / dead-zone hang or brown-out" (`power-on`/`brown-out`) on the very next boot.
fn log_reset_reason() -> (&'static str, bool) {
  let pro_reason = reset_reason(Cpu::ProCpu);
  let pro = pro_reason.map(reset_reason_label).unwrap_or("unknown");
  let app = reset_reason(Cpu::AppCpu).map(reset_reason_label).unwrap_or("unknown");
  #[cfg(feature = "defmt")]
  defmt::info!("boot: reset reason PRO_CPU={=str}, APP_CPU={=str}", pro, app);
  #[cfg(not(feature = "defmt"))]
  esp_println::println!("[boot] reset reason: PRO_CPU={}, APP_CPU={}", pro, app);
  // The crash-report emit decision keys off the PRO_CPU (core 0) reason — the RWDT this firmware arms resets the
  // whole system and reports there. Returned (with the label) so `main` can pass it to `maybe_emit_crash_report`.
  (pro, reset_was_watchdog_or_fault(pro_reason))
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

  // 1b. Capture WHY the chip last reset, BEFORE arming the new watchdog, and log it. This is the whole point of the
  //     watchdog for the lockup hunt: after the dog auto-resets a wedged board, the next boot reports
  //     `Cpu0RtcWdt`/`CoreRtcWdt` here instead of leaving us with only "it needed a hard power-cycle". A clean
  //     power-on reads `ChipPowerOn`; a brown-out reads `SysBrownOut`; a panic-driven software reset reads
  //     `Cpu0Sw`/`CoreSw`. Logged on a default build too via esp-println. Returns whether it was a watchdog/fault
  //     reset, which (with a valid breadcrumb) gates the post-mortem crash report emitted after the banner.
  let (reset_reason_label, reset_was_watchdog) = log_reset_reason();

  // 1b-ii. Read the previous run's crash breadcrumb out of RTC_FAST and CONSUME it (clear the magic), THEN stamp
  //     the magic for THIS run. The breadcrumb survives a watchdog reset but NOT a power-cycle (see `crash`). It is
  //     read before arming/spawning anything so the read reflects the wedged run, not this one; the formatted
  //     `[MSG:CRASH ...]` line is emitted after the banner (step 8) over the normal grbl TX.
  let breadcrumb = crash::take_breadcrumb();
  crash::init_magic();
  // Give the free-running §15 truncation counters a clean per-BUILD baseline: RTC_FAST survives a software reset AND
  // a flash, so a fresh image must NOT inherit a prior image's bytes at the trunc word addresses (a bogus huge
  // `trunc`). Zeroes them only on a build-id mismatch; a same-image software reset preserves them (survive-the-reset).
  crash::reset_truncation_on_new_build();

  // 1c. Arm the RTC watchdog as early as possible (before the slower settings/coordinate flash loads below) so a
  //     hang anywhere in bring-up is also caught. Stage 0's default action is a system reset; we set only its
  //     timeout (`WATCHDOG_TIMEOUT`, 8 s — see its doc for the false-trip-margin rationale) and enable it. The
  //     `Rtc` is parked in a `StaticCell` and handed to the `watchdog_feed` task, which is the sole feeder. `Rtc`
  //     must stay alive for the dog to keep running, hence the `'static` park.
  let rtc: &'static mut Rtc<'static> = RTC.init(Rtc::new(peripherals.LPWR));
  rtc.rwdt.set_timeout(RwdtStage::Stage0, WATCHDOG_TIMEOUT);
  rtc.rwdt.enable();
  // 1c-ii. DIAGNOSTIC capture build (`capture-reset`, §17.10/§17.11): ALSO arm the SuperWDT as a second, independent
  //     dog. `Swd::enable` disables its auto-feed so it becomes a REAL watchdog — but the S3 SuperWDT has no
  //     `set_timeout` (its period is a short fixed silicon value), so the TIMG1 survivable-watchdog ISR must
  //     software-feed it at a short cadence (it feeds both dogs together). With BOTH dogs armed and fed only by the
  //     hardware-timer ISR — which survives a core-0 executor stall — a Signature-B wedge that starves core 0
  //     withholds both feeds and forces a breadcrumb-bearing reset. Production (default) keeps the RWDT-only async
  //     feeder.
  #[cfg(feature = "capture-reset")]
  rtc.swd.enable();

  // 2. Start the esp-rtos scheduler with the TIMG0 timer as the time source. This also installs the
  //    Embassy time-driver, so `embassy-time` and channel/Signal awaits operate from here on. As of
  //    esp-rtos 0.3 (#4459) `start` claims `software_interrupt0` for the scheduler on all CPUs, so the
  //    `SoftwareInterruptControl` must be created BEFORE `start`; its remaining interrupts (1 for the
  //    second-core bring-up, 2 for the motion `InterruptExecutor`) are consumed later.
  let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
  let timg0 = TimerGroup::new(peripherals.TIMG0);
  esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

  // 3. Bring up the USB Serial/JTAG CDC controller (internal PHY on GPIO19/20; no descriptors, no eFuse).
  //    Convert to the async driver and split into independent RX/TX halves for the two USB tasks.
  let usb = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();
  let (usb_rx, usb_tx) = usb.split();

  // 3b. Settings persistence (DOC-04). Construct the single flash instance, load the persisted `$`-settings
  //     record — the loader is INFALLIBLE: absence, corruption, or schema skew silently yields compiled
  //     defaults, so a bad flash region can never wedge boot — seed the live `SETTINGS`, and derive the
  //     planner / motion / TMC configs from it. The flash is shared `&'static` so the consumer can persist
  //     `$x=val` writes at runtime.
  // `multicore_auto_park` is REQUIRED on this dual-core board: esp-storage defaults to `MultiCoreStrategy::Error`,
  // which refuses every flash write while the other core is running (returns `OtherCoreRunning`). Core 1 (`APP_CPU`)
  // permanently runs the `motion_executor`, so without auto-park every settings/coordinate persist fails silently
  // and nothing is ever written to flash. Auto-park briefly parks the other core for the (infrequent, idle-time)
  // write and unparks it after, so persistence actually commits.
  let flash: &'static storage::SharedFlash = FLASH.init(Mutex::new(storage::FlashState::new(
    FlashStorage::new(peripherals.FLASH).multicore_auto_park(),
  )));
  let settings = {
    let mut store = storage::FlashRecordStore::settings(flash);
    // Use the reporting loader so a present-but-undecodable record (a CRC mismatch, a write the reset button
    // truncated, or a schema-version skew) is surfaced on the serial monitor instead of silently booting on
    // factory defaults (homing off). Absence is the legitimate first-boot path and stays quiet. The loader is
    // still infallible — it always yields a usable `Settings` and never wedges boot.
    let (settings, outcome) = firmware_core::settings::load_reporting(&mut store).await;
    #[cfg(feature = "defmt")]
    if outcome == firmware_core::settings::LoadOutcome::DefaultedCorrupt {
      defmt::warn!("settings: stored record present but undecodable; booting on factory defaults (homing off)");
    }
    // The `outcome` is only consulted under `defmt`; silence the unused binding in the default (no-defmt) build.
    let _ = outcome;
    settings
  };
  let planner_config = settings.planner_config();
  let motion_config = settings.motion_config(MOTION_TICK_HZ);
  let tmc_config = settings.tmc_config();
  let homing_enabled = settings.homing_enabled();
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
  // Seed the `$21` hard-limit-enable mirror so the core-1 executor's hard-limit check reads the persisted state
  // (DOC-06). The limit inputs are configured above; the executor samples them at block boundaries / on the ISR.
  comms::init_limit_settings(settings.hard_limits_enabled(), settings.limit_invert, settings.homing_debounce_ms);
  // Phase F: seed the live auto-report cadence mirror from the persisted `$481` so the auto-report task pushes
  // periodic status reports at the configured interval (a no-op when `$481=0`, the default).
  comms::init_auto_report(auto_report_interval);

  // TODO(DOC-06): the X/Y/Z limit inputs, `$H` homing, and hard/soft limits are wired below (step 4d); what
  //   remains is the optional feed-hold / cycle-start control-input GPIO (GPIO16/17) and acting on the planner's
  //   GoToPredefined (G28/G30) outcome, currently passed through.

  // 4. Bring up the RMT step channels + DIR/STEP_EN GPIO and build the step sink. STEP_EN is driven enabled
  //    (active-low → low) so the steppers hold. Both the sink and STEP_EN are parked in `StaticCell`s so
  //    their RMT channels / pins live for the program's lifetime (the motion task borrows the sink `'static`).
  let (sink, step_enable) = motion::init(
    peripherals.RMT,
    // A-STEP on the spare RMT ch3/GPIO18; A-DIR on GPIO38. PROVISIONAL (DOC-10 Phase 5, bench-unverified).
    (peripherals.GPIO1, peripherals.GPIO2, peripherals.GPIO4, peripherals.GPIO18),
    (peripherals.GPIO5, peripherals.GPIO6, peripherals.GPIO7, peripherals.GPIO38),
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

  // 4d. Bring up the X/Y/Z LIMIT inputs on GPIO10/11/12 (DOC-06) for the `$H` homing cycle and the runtime
  //     hard-limit (`$21`) path. Each pin gets the internal pull-up unconditionally (the NC broken-wire
  //     fail-safe needs it); the `$5` invert is applied per-sample by `limit_triggered`, not at pin config.
  //     Parked in a `StaticCell` so the pins stay configured and the `motion_executor` task borrows them
  //     `'static` (it owns the RMT channels, so the homing cycle — which emits steps — runs there, dispatched
  //     by `$H` via the `HOME_REQUEST` signal, exactly as a `G38.x` probe is dispatched).
  // A-LIMIT placeholder on GPIO39 — A has no physical switch (DOC-10.6); PROVISIONAL (DOC-10 Phase 5).
  let limits = motion::init_limits(peripherals.GPIO10, peripherals.GPIO11, peripherals.GPIO12, peripherals.GPIO39);
  let limits: &'static mut [motion::RmtLimitInput; firmware_core::planner::AXES] = LIMIT_INPUTS.init(limits);

  // 4b. Bring up UART1 as the single-wire TMC2209 bus on GPIO9 (DOC-03). The bus is owned by the
  //     `tmc_manager` task (spawned below), which runs the per-driver init sequence and then polls
  //     `DRV_STATUS`. Built here so its peripherals (UART1 + GPIO9) are claimed alongside the others.
  let tmc_bus = tmc::init(peripherals.UART1, peripherals.GPIO9);

  // 4e. Bring up the SPINDLE (DOC-07): LEDC ch0/timer0 on GPIO13 (PWM → external RC + op-amp → 0–10 V), plus
  //     SPIN_EN (GPIO14, active-low) and SPIN_DIR (GPIO15). The controller starts SAFE (EN de-asserted, duty 0).
  //     Parked in a `StaticCell` so the `spindle` task borrows it `'static` and is the sole driver of the spindle
  //     outputs — it consumes the planner's `Spindle` outcome and the override-scaled RPM, and runs `$393`
  //     reverse-dwell timing. The LEDC controller/timer/channel cells back the self-referential `'static` chain.
  let spindle = spindle::init(
    peripherals.LEDC,
    peripherals.GPIO13,
    peripherals.GPIO14,
    peripherals.GPIO15,
    &SPINDLE_LEDC,
    &SPINDLE_TIMER,
    &SPINDLE_CHANNEL,
  );
  let spindle: &'static mut spindle::Spindle = SPINDLE.init(spindle);

  // 4f. Bring up COOLANT (M7/M8/M9, DOC-07 follow-up). No coolant GPIO is budgeted on this board, so this claims no
  //     pins and cannot fail: the controller drives stubbed outputs (it traces the logical level under defmt but
  //     touches no hardware). Parked `'static` so the `coolant` task is the sole driver, exactly like the spindle.
  let coolant = coolant::init();
  let coolant: &'static mut coolant::Coolant = COOLANT.init(coolant);

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

  // 6. Start core 1 and its high-priority interrupt executor, then spawn `motion_executor` on it. As of
  //    esp-rtos 0.3 the SMP scheduler claims SWI 0 (consumed by `esp_rtos::start` above) and SWI 1 (consumed
  //    by `start_second_core` below); the motion interrupt executor therefore uses SWI 2. The `func` closure
  //    runs ONCE on core 1 (pinned, via the
  //    app-core boot vector): it starts the interrupt executor — binding the SWI-2 handler and enabling
  //    FROM_CPU_INTR2 on core 1 so the task is polled there — spawns `motion_executor`, then returns
  //    (esp-rtos idles the core-1 main thread in `waiti`; the interrupt executor keeps running).
  // Initialize the core-1 stack arena (stack + trailing ABI headroom). The `#[repr(C)]` arena guarantees
  // `_abi_headroom` sits immediately above the stack top, absorbing the Xtensa windowed-ABI over-top register
  // spill that would otherwise corrupt the adjacent control statics (see [`AppCoreStackArena`] — the headroom is
  // load-bearing; do not remove it). `arm_canary` writes a sentinel into the canary zone (the upper half, which a
  // correctly-sized spill never reaches) and keeps the field alive; we verify it after bring-up below.
  let arena = APP_CORE_STACK.init(AppCoreStackArena::new());
  arena.arm_canary();
  // `start_second_core` needs `&'static mut Stack`, which would otherwise hold the WHOLE `arena` borrowed for
  // `'static` and forbid the post-bring-up `check_canary` below. Capture a const raw pointer to the arena BEFORE
  // splitting off the stack, then verify the canary through it once the call returns. The raw pointer aliases the
  // `'static mut` stack borrow, but they touch disjoint fields (`stack` vs the canary zone of `_abi_headroom`),
  // and the check runs only AFTER `start_second_core` has returned (the stack borrow is conceptually done — core 1
  // owns its own SP now), so no live `&mut` to those bytes coexists with the read.
  let arena_ptr: *const AppCoreStackArena = arena;
  // Re-borrow `arena.stack` AFTER arming so the canary write is sequenced before the stack hand-off; the closure
  // does not need the arena, so only `&mut arena.stack` crosses into `start_second_core`.
  let app_stack = &mut arena.stack;
  // As of esp-rtos 0.3 (#4459) `start_second_core` no longer takes `software_interrupt0` (claimed by
  // `start` above); it takes only `software_interrupt1` for the second-core bring-up. SWI 2 remains free
  // for the motion `InterruptExecutor` below.
  esp_rtos::start_second_core(
    peripherals.CPU_CTRL,
    sw_int.software_interrupt1,
    app_stack,
    move || {
      // This closure runs exactly once on core 1: bring up the motion interrupt executor. `executor.start`
      // (called here, on core 1) binds the SWI-2 handler and enables FROM_CPU_INTR2 on `Cpu::current()` =
      // core 1, so the interrupt routing and INTENABLE land on the core that must poll the task. `start` takes
      // `&'static mut self`, hence the `StaticCell`. SWI 2 is the first software interrupt free of the esp-rtos
      // SMP scheduler (which claims 0 and 1).
      let executor = MOTION_EXECUTOR.init(InterruptExecutor::new(sw_int.software_interrupt2));
      let motion_spawner = executor.start(MOTION_EXECUTOR_PRIORITY);
      // As of embassy-executor 0.10 the `#[task]` macro returns the `SpawnToken` as a `Result` (the pool
      // check moved to token creation; #4459) and `spawn` itself is infallible. `expect` on the token at
      // init replicates the old `must_spawn`: a failure means the pool is exhausted / the task was already
      // spawned — a static, unrecoverable wiring bug, not a runtime condition (CLAUDE.md permits `expect` in
      // init). The task takes the `'static` sink/probe by mutable borrow. The axis max-rates (`$110-112`)
      // bound the Phase-E feed-override scale-up so a boosted feed never exceeds the configured rate limit;
      // they are passed alongside the step-timing config.
      motion_spawner.spawn(
        motion_executor(sink, probe, limits, motion_config, planner_config.max_rate_mm_min)
          .expect("spawn motion_executor"),
      );
    },
  );

  // Verify the ABI-headroom canary now that core-1 bring-up has completed (we are back on core 0 and the first
  // core-1 frame has already executed, so the windowed-ABI over-top spill has happened). The canary lives in the
  // UPPER half of the headroom, above the worst-case spill zone, so a disturbed byte means a spill reached past
  // [`XTENSA_MAX_OVERTOP_SPILL`] — the headroom is undersized and [`APP_CORE_ABI_HEADROOM`] must grow. Observable
  // only on a defmt build; on a default build the check is a cheap volatile read whose result is discarded.
  // SAFETY: `arena_ptr` came from a `&'static mut` (the `StaticCell` contents live for the program), so it is a
  // valid, aligned, non-null pointer to a live `AppCoreStackArena`. The `'static mut` stack borrow that aliased
  // it has ended (`start_second_core` has returned), so dereferencing for an immutable canary read is sound.
  if !unsafe { (*arena_ptr).check_canary() } {
    #[cfg(feature = "defmt")]
    defmt::error!("core1 abi-headroom canary disturbed: over-top register spill exceeded the modeled worst case");
  }

  // 7. Spawn the core-0 comms tasks on the thread-mode executor. As of embassy-executor 0.10 the `#[task]`
  //    macro returns the `SpawnToken` as a `Result` and `spawn` is infallible, so we `expect` each token at
  //    init (a spawn failure is a static wiring bug; CLAUDE.md permits `expect` in init). The RX path is
  //    split: `usb_rx` (reader half) extracts real-time bytes and buffers the rest into `RX_PIPE`, while
  //    `line_assembler` frames lines from that buffer — so real-time commands never block behind line
  //    back-pressure (DOC-08 grbl ISR model). `comms_consumer` is the real parser → planner pipeline; it now
  //    wakes the core-1 motion executor via `BLOCK_AVAILABLE`.
  spawner.spawn(comms::usb_rx(usb_rx).expect("spawn usb_rx"));
  spawner.spawn(comms::line_assembler().expect("spawn line_assembler"));
  spawner.spawn(comms::usb_tx(usb_tx).expect("spawn usb_tx"));
  // The consumer takes the shared flash so a `$x=val` setting write is persisted to the NVS region (DOC-04).
  spawner.spawn(comms::comms_consumer(flash).expect("spawn comms_consumer"));
  spawner.spawn(comms::status_responder().expect("spawn status_responder"));
  // The auto-report task (Phase F, DOC-08 §5) pushes a `<...>` status report every `$481`-ms when enabled,
  // reusing `status_responder`'s single formatter/writer so an auto-report is byte-identical to a `?` report.
  spawner.spawn(comms::auto_report_task().expect("spawn auto_report_task"));
  // The TMC2209 manager runs the driver init sequence at startup (from the loaded settings), then polls
  // DRV_STATUS for faults. It owns the UART1 bus by value (a `'static` peripheral handle), so no `StaticCell`
  // is needed (DOC-03).
  spawner.spawn(tmc::tmc_manager(tmc_bus, tmc_config).expect("spawn tmc_manager"));
  // The spindle task (DOC-07, core 0 / PRO_CPU, priority 0) is the sole driver of the spindle outputs. It awaits
  // the consumer's spindle-update wake (an M3/M4/M5 or a spindle override/stop change) and the emergency-stop
  // signal (ALARM / soft-reset / sleep), drives the `SpindleController`, and runs the `$393` reverse dwell.
  spawner.spawn(comms::spindle(spindle).expect("spawn spindle"));
  spawner.spawn(comms::coolant(coolant).expect("spawn coolant"));
  // The watchdog feed path. The DIAGNOSTIC capture build and the PRODUCTION default differ here:
  //
  // - PRODUCTION (default): the core-0 async `watchdog_feed` task pets the RWDT every 500 ms so a healthy board never
  //   resets, while a core-0 wedge stops the feed and lets the dog auto-reset (recorded, so the next boot logs the
  //   reason). It owns the `Rtc` `'static` (the sole feeder). UNCHANGED from before — a production board builds and
  //   behaves exactly as today.
  //
  // - DIAGNOSTIC (`capture-reset`, §17.10/§17.11): the TIMG1 hardware-timer ISR is the feeder — it SURVIVES a core-0
  //   executor stall (the async task would be starved) and feeds BOTH dogs via raw PAC, or on a stall WITHHOLDS both
  //   and captures the breadcrumb. So the async `watchdog_feed` must NOT also feed (that would mask the withhold);
  //   instead a thin `watchdog_heartbeat` task runs the diagnostic snapshot ring only. The `Rtc` stays parked in its
  //   `StaticCell` (dogs armed) but is NOT handed to any feeder — the ISR feeds register-side, no `&mut Rtc` needed.
  #[cfg(not(feature = "capture-reset"))]
  spawner.spawn(comms::watchdog_feed(rtc).expect("spawn watchdog_feed"));
  #[cfg(feature = "capture-reset")]
  {
    // The `Rtc` is owned by the `StaticCell` for `'static` (dogs stay armed); the ISR feeds via raw PAC, not through
    // this borrow. Bind `_ = rtc` so the unused `&'static mut` does not warn while keeping it conceptually alive.
    let _ = rtc;
    survivable_watchdog::start(peripherals.TIMG1);
    spawner.spawn(comms::watchdog_heartbeat().expect("spawn watchdog_heartbeat"));
  }

  // 8. Emit the welcome banner on boot so a host detects readiness immediately (native USB cannot be
  //    hard-reset by the host). The banner is also re-emitted on every soft reset (by the consumer's
  //    pipeline reset, with a best-effort copy from the reader half).
  comms::send_banner().await;
  // ALWAYS surface WHY the chip last reset over the grbl TX (Signature-B instrumentation): a `[MSG:RESET <reason>]`
  // line right after the banner, INDEPENDENT of the breadcrumb. The `[MSG:CRASH ...]` dump below is gated on a valid
  // breadcrumb + a watchdog/fault reset, so a NO-breadcrumb boot (clean power-on, brown-out that wiped RTC_FAST, or
  // a silent-lock that wrote nothing) would otherwise say nothing on the wire. This line discriminates "a reset DID
  // fire" (sw-reset / rtc-WDT) from "no software reset" (power-on / brown-out) on the very next boot — the cheapest,
  // highest-value Signature-B datum (it settles "reset-but-no-reenum" vs "dead-zone hang / brown-out").
  comms::send_reset_reason(reset_reason_label).await;
  // Post-mortem crash report: if the previous run left a valid RTC_FAST breadcrumb AND this was a watchdog/fault
  // reset, emit a `[MSG:CRASH ...]` line over the normal grbl TX right after the banner so a connected sender logs
  // where the firmware wedged (e.g. `stage=axis1:wait_begin core1-froze-first`). It is ALSO stashed for one replay
  // on the first `$I`/`?` after connect, so a host that reconnected late across the USB re-enumeration still gets
  // it. A no-op on a clean boot (no valid breadcrumb).
  comms::maybe_emit_crash_report(&breadcrumb, reset_was_watchdog).await;
  // If homing is enabled the machine booted locked in `ALARM:11`; push the boot alarm + `[MSG:'$H'|'$X' to
  // unlock]` right after the banner so a sender detects the locked state on connect (DOC-08 §5). When homing
  // is disabled this is a no-op and the machine comes up Idle.
  comms::send_boot_alarm().await;

  // The executor keeps the spawned tasks running; this initial task has nothing left to do. Awaiting a
  // never-completing future parks it without burning the CPU in a busy loop.
  core::future::pending::<()>().await;
}
