# CNC PCB Milling Machine Firmware — Architecture & Specification Document Set
## (ESP32-S3 / Rust / Embassy)

**TL;DR**
- Complete, implementation-ready specification (DOC-00 through DOC-09) for a grblHAL-compatible
  3-axis CNC PCB milling firmware on an ESP32-S3, written in `no_std` Rust on esp-hal 1.0.0 with
  the Embassy async runtime.
- The ESP32-S3 provides 4 dedicated TX-capable RMT channels — one per axis (X/Y/Z) with a spare —
  resolving the hard blocker on the ESP32-C6. It also adds a hardware single-precision FPU and a
  second 240 MHz Xtensa LX7 core, enabling a clean dual-core executor split.
- Step/dir pulses are generated on RMT TX channels 0/1/2 (one per axis), spindle speed via LEDC
  PWM, TMC2209 drivers share one half-duplex UART bus, and the host link uses the fixed-function
  USB Serial/JTAG CDC-ACM controller. The motion executor runs on core 1 at interrupt priority;
  GCode parsing and USB comms run on core 0.
- **Toolchain note:** The ESP32-S3 uses the Xtensa LX7 ISA, which requires Espressif's unofficial
  Xtensa LLVM fork (`espup` toolchain). This is a different setup from RISC-V targets like the C6
  that work with stock Rust. See DOC-09 for toolchain bootstrap.

---

## DOC-00: Project Overview & Hardware Manifest

### Purpose
A 3-axis (X/Y/Z) CNC PCB milling machine controller. Host software (e.g. ioSender, UGS, Candle)
streams grblHAL-compatible GCode over USB CDC serial. The firmware parses GCode, plans coordinated
multi-axis motion with trapezoidal acceleration and look-ahead, and emits step/direction pulses to
three TMC2209 stepper drivers, while controlling a WS55-220 brushless spindle driver.

### Hardware
- **MCU:** ESP32-S3 devkit. Dual Xtensa LX7 cores at up to 240 MHz with hardware single-precision
  FPU (FPv3). 512 KB SRAM; typical devkit module adds 4–16 MB flash and optionally 2–8 MB PSRAM
  (PSRAM not required for this firmware).
- **Steppers:** 3× NEMA 17, each on a TMC2209 (Adafruit 6121 breakout, up to 2 A, 5–29 VDC motor
  voltage, 3–5 V logic, 1/8 microstep default, configurable to 1/256 via UART).
- **Spindle:** WS55-220 brushless driver, controlled via a 0–10 VDC analog speed input (SV
  terminal), an EN start/stop input (to GND = run), and an F/R direction selection. The 0–10 V
  signal is derived from a 3.3 V LEDC PWM output through an external RC low-pass filter and
  non-inverting op-amp gain stage. This conditioning circuit is a required hardware element.
- **Homing:** Mechanical NC micro-switches on each axis. Upgrade path to NPN NC opto-isolated
  sensors is documented in DOC-06.
- **Frame:** 2020 aluminum extrusion.

### GPIO Pin Assignment Table (ESP32-S3 devkit)
> Strapping pins on the S3 devkit that must not be driven at boot: GPIO0, GPIO3, GPIO45, GPIO46.
> GPIO19 (USB D−) and GPIO20 (USB D+) are shared between the USB Serial/JTAG controller and the
> USB OTG (DWC2) controller; leave them for the USB subsystem. The assignments below are a
> recommended default; cross-check against the specific devkit silkscreen before wiring.

| Function              | Signal    | GPIO   | Peripheral       | Notes                                  |
|-----------------------|-----------|--------|------------------|----------------------------------------|
| X step                | X_STEP    | GPIO1  | RMT TX ch0       | RMT output, 48-symbol block            |
| Y step                | Y_STEP    | GPIO2  | RMT TX ch1       | RMT output                             |
| Z step                | Z_STEP    | GPIO4  | RMT TX ch2       | RMT output                             |
| X dir                 | X_DIR     | GPIO5  | GPIO out         | Set before each step burst             |
| Y dir                 | Y_DIR     | GPIO6  | GPIO out         |                                        |
| Z dir                 | Z_DIR     | GPIO7  | GPIO out         |                                        |
| Stepper enable (cmn.) | STEP_EN   | GPIO8  | GPIO out         | TMC2209 ENN, active low                |
| TMC UART (shared)     | TMC_UART  | GPIO9  | UART1 single-wire| 1 kΩ series resistor to PDN_UART bus   |
| X limit               | X_LIM     | GPIO10 | GPIO in + IRQ    | NC switch, internal pull-up            |
| Y limit               | Y_LIM     | GPIO11 | GPIO in + IRQ    | NC switch, internal pull-up            |
| Z limit               | Z_LIM     | GPIO12 | GPIO in + IRQ    | NC switch, internal pull-up            |
| Spindle PWM           | SPIN_PWM  | GPIO13 | LEDC ch0         | Feeds 0–10 V conditioning stage        |
| Spindle enable        | SPIN_EN   | GPIO14 | GPIO out         | WS55-220 EN (to GND = run)             |
| Spindle direction     | SPIN_DIR  | GPIO15 | GPIO out         | WS55-220 F/R selection                 |
| Feed hold (optional)  | FHOLD     | GPIO16 | GPIO in + IRQ    | NC recommended                         |
| Cycle start (optional)| CYCSTART  | GPIO17 | GPIO in + IRQ    |                                        |
| RMT TX ch3 (spare)    | —         | GPIO18 | RMT TX ch3       | Reserved for 4th axis or future use    |
| USB D−                | USB_DM    | GPIO19 | USB Serial/JTAG  | Reserved — do not use                  |
| USB D+                | USB_DP    | GPIO20 | USB Serial/JTAG  | Reserved — do not use                  |
| Probe (Z touch-off)   | PROBE     | GPIO21 | GPIO in          | G38.x touch-off; pull-up unless $19; $6 inverts |

### Peripheral Allocation Table

| ESP32-S3 Peripheral    | Allocated To                        | Notes                                              |
|------------------------|-------------------------------------|----------------------------------------------------|
| RMT TX channel 0       | X step pulse generation             | 48-symbol block, 12.5 ns tick @ divider 1          |
| RMT TX channel 1       | Y step pulse generation             |                                                    |
| RMT TX channel 2       | Z step pulse generation             |                                                    |
| RMT TX channel 3       | Spare (4th axis or future use)      | One dedicated TX channel always available          |
| LEDC low-speed timer0  | Spindle PWM (WS55-220 speed)        | S3 LEDC is low-speed only                         |
| UART1                  | TMC2209 single-wire UART bus        | Half-duplex, 115200 baud, all three drivers        |
| USB Serial/JTAG ctrl.  | Host CDC-ACM serial                 | Fixed-function CDC-ACM, internal PHY on GPIO19/20  |
| GPIO + GPIO IRQ        | Limit switches, probe, control, dir/enable |                                             |
| SYSTIMER               | embassy-time driver                 | Three hardware alarms                              |
| TIMG0                  | Watchdog / backup timer             |                                                    |
| Flash (esp-storage)    | $-settings persistence              | NVS-like CRC-versioned struct in dedicated region  |
| CPU1 (APP_CPU)         | Motion executor (InterruptExecutor) | Dedicated core for real-time step generation       |

### Crate Dependency Manifest (Cargo.toml)
> Versions reflect the esp-hal 1.0.x stable line (released 30 Oct 2025) targeting the ESP32-S3.
> The `unstable` feature on esp-hal is mandatory because RMT, LEDC, USB Serial/JTAG, and
> `CpuControl` (multicore) are all behind this feature gate. Note the Xtensa compiler target in
> `.cargo/config.toml` (see DOC-09).

```toml
[package]
name    = "pcb-mill-fw"
edition = "2021"

[dependencies]
esp-hal         = { version = "1.0.0", features = ["esp32s3", "unstable"] }
esp-hal-embassy = { version = "0.9",   features = ["esp32s3"] }
embassy-executor = { version = "0.9",  features = ["task-arena-size-32768"] }
embassy-time    = "0.5"
embassy-sync    = "0.7"
embassy-futures = "0.1"
esp-backtrace   = { version = "0.17",  features = [
  "esp32s3", "panic-handler", "exception-handler", "println",
] }
esp-println     = { version = "0.15",  features = ["esp32s3"] }
esp-storage     = { version = "0.7",   features = ["esp32s3"] }
embedded-storage    = "0.3"
embedded-io-async   = "0.6"
heapless            = "0.8"
libm                = "0.2"    # no_std float math (arc interpolation).
defmt               = "0.3"
static_cell         = "2"      # For static task arenas across cores.

[dev-dependencies]
embedded-hal-mock = "0.11"
```

> Pin all versions in Cargo.lock. The most common build break is embassy-executor/embassy-time
> version drift relative to what esp-hal-embassy expects; verify with `cargo tree`. The
> `static_cell` crate is used to create `'static` references to executor and stack storage needed
> for the second-core launch in `main`.

### Workspace Layout
```
pcb-mill-fw/
├── .cargo/
│   └── config.toml           # target = "xtensa-esp32s3-none-elf", runner = espflash
├── Cargo.toml                # workspace manifest
├── firmware/                 # binary crate (no_std, no_main) — tasks, HW wiring
│   ├── Cargo.toml
│   └── src/main.rs
├── gcode/                    # lib crate, no_std, host-testable — tokenizer + parser
├── planner/                  # lib crate, no_std, host-testable — block queue + profiler + arcs
├── protocol/                 # lib crate, no_std, host-testable — status/errors/settings model
├── motion/                   # lib crate, no_std, host-testable — segment generation + kinematics
├── drivers/                  # lib crate, no_std, host-testable — TMC2209 register codec
└── hal_traits/               # trait definitions for StepSink, PwmSink, DigitalIn/Out, Serial
```
Library crates (`gcode`, `planner`, `protocol`, `motion`, `drivers`, `hal_traits`) have no
esp-hal dependency and compile on `x86_64` for host-side unit testing. Only `firmware/` depends
on esp-hal.

---

## DOC-01: Embassy Task Architecture

### Execution model
The ESP32-S3 has two Xtensa LX7 cores: PRO_CPU (core 0) and APP_CPU (core 1). The architecture
uses both with a clean responsibility split:

- **Core 1 — `InterruptExecutor` (high priority):** the `motion_executor` task runs here at
  `Priority::Priority3`. Core 1 is started from `main` via `CpuControl::start_app_core` and
  immediately spins up the interrupt executor. Because this core runs no other tasks, the motion
  executor preempts nothing and gets uncontested CPU time for real-time step generation.
- **Core 0 — thread-mode `Executor` (normal priority):** all other tasks (USB comms, GCode parser,
  motion planner, TMC manager, spindle, status reporter, homing) run here.

The `InterruptExecutor` on core 1 uses `SoftwareInterrupt<1>`. The motion executor is the only
task spawned on it; a second interrupt executor on core 0 (`SoftwareInterrupt<0>`) runs at normal
priority to service Embassy's timer/waker infrastructure on that core.

### Core 1 startup (sketch)
```rust
// In main, after all peripherals are initialized.
static APP_CORE_STACK: StaticCell<Stack<8192>> = StaticCell::new();
static MOTION_EXECUTOR: StaticCell<InterruptExecutor<1>> = StaticCell::new();

let stack   = APP_CORE_STACK.init(Stack::new());
let mut cpu = CpuControl::new(peripherals.CPU_CTRL); // unstable-gated.

let _guard = cpu.start_app_core(stack, move || {
  let executor = MOTION_EXECUTOR.init(InterruptExecutor::new(
    sw_int.software_interrupt1,
  ));
  let spawner = executor.start(Priority::Priority3);
  spawner.spawn(motion_executor(/* channels */)).unwrap();
  loop { core::hint::spin_loop(); }
});
```

### Task decomposition

| Task              | Core | Executor    | Stack arena | Responsibility                                               |
|-------------------|------|-------------|-------------|--------------------------------------------------------------|
| `motion_executor` | 1    | Interrupt P3 | ~4 KB      | Pop planner blocks → generate step segments → feed RMT; honor feed-hold/stop signals |
| `usb_rx`          | 0    | Thread      | ~2 KB       | Read USB CDC bytes; detect real-time bytes; assemble lines   |
| `usb_tx`          | 0    | Thread      | ~2 KB       | Drain outgoing response/report queue to USB                  |
| `gcode_parser`    | 0    | Thread      | ~4 KB       | Tokenize + parse lines; emit parsed commands or errors       |
| `planner`         | 0    | Thread      | ~6 KB       | Accept moves; maintain look-ahead block ring buffer; recompute junction velocities |
| `tmc_manager`     | 0    | Thread      | ~3 KB       | Init TMC2209 registers; runtime current scaling; poll DRV_STATUS |
| `spindle`         | 0    | Thread      | ~2 KB       | M3/M4/M5 handling; LEDC duty; enable/dir sequencing + safety interlock |
| `status_reporter` | 0    | Thread      | ~2 KB       | On `?` or timer, build `<...>` report                        |
| `homing`          | 0    | Thread      | ~3 KB       | Execute `$H` homing state machine                            |

> Stack/arena sizes are starting estimates. Tune with `cargo size` and stack-painting under load.
> All thread-mode tasks on core 0 share the executor's task arena; the core 1 executor has its
> own `APP_CORE_STACK`.

### Inter-task communication topology
All primitives from `embassy-sync` with `CriticalSectionRawMutex` (single-core safe, ISR-safe,
and safe across cores on the S3 because `CriticalSectionRawMutex` disables interrupts on the
calling core only — acceptable here since the motion executor does not share channels with core-0
tasks directly; it communicates only through the `BlockQueue` Mutex and feed-hold Signals):

- `usb_rx` → `gcode_parser`: `Channel<CriticalSectionRawMutex, Line, 4>`
- `usb_rx` → real-time handlers: `Signal`s — `FEED_HOLD`, `CYCLE_START`, `SOFT_RESET`,
  `STATUS_REQUEST`
- `gcode_parser` → `planner`: `Channel<_, PlannerCommand, 8>`
- `planner` → `motion_executor`: `BlockQueue` ring buffer behind
  `Mutex<CriticalSectionRawMutex, _>` plus a `Signal` `BLOCK_AVAILABLE`
- `motion_executor` → `status_reporter`: shared `MachineState` in a `Mutex` (live MPos in steps)
- Any task → `usb_tx`: `Channel<_, Response, 8>`
- `gcode_parser`/`planner` → `spindle`: `Signal<SpindleCommand>`
- Limit-switch ISR → `motion_executor`: `Signal` `LIMIT_TRIGGERED`

### Inter-task message data structures (sketch)
```rust
/// A single assembled input line, capped to the grblHAL RX line length.
pub struct Line(pub heapless::String<128>);

/// A fully parsed, validated motion or modal command handed to the planner.
pub enum PlannerCommand {
  Linear   { target: [f32; 3], feed: f32, rapid: bool },
  Arc      { target: [f32; 3], offset: [f32; 2], cw: bool, feed: f32, plane: Plane },
  Dwell    { seconds: f32 },
  SetCoordOffset { axis: u8, value: f32 },
  Home,
}

/// Spindle command derived from M3/M4/M5 + S word.
pub struct SpindleCommand { pub state: SpindleState, pub rpm: u16 }
pub enum SpindleState { Off, Cw, Ccw }
```

### Startup / initialization sequence
1. `esp_hal::init` with max CPU clock (240 MHz); obtain `Peripherals`.
2. Initialize `esp-storage`; load persisted `$`-settings; fall back to compiled defaults on CRC
   failure.
3. Initialize embassy-time driver (SYSTIMER).
4. Configure GPIO (dir/enable outputs, limit inputs with pull-ups + interrupts, spindle pins).
5. Configure UART1 for the TMC bus; run `tmc_manager` init sequence (DOC-03). On failure, raise
   alarm but continue (drivers may be VREF-configured).
6. Configure RMT TX channels 0/1/2 for step generation (DOC-02) and LEDC for spindle (DOC-07).
7. Initialize USB Serial/JTAG controller.
8. Start core 1 via `CpuControl::start_app_core`; spawn `motion_executor` on its
   `InterruptExecutor`.
9. Start core 0 thread-mode executor; spawn remaining tasks.
10. Emit grblHAL welcome banner and enter ALARM state if homing is enabled and machine is unhomed
    (grblHAL boots into ALARM when homing is enabled and position is unknown).

---

## DOC-02: RMT Step Generation Engine

### RMT capabilities on ESP32-S3 (verified)
The ESP32-S3 has 8 RMT channels split into two fixed groups:
- **TX channels: ch0, ch1, ch2, ch3** (4 dedicated transmit channels). `SOC_RMT_TX_CANDIDATES_PER_GROUP = 4`.
- **RX channels: ch4, ch5, ch6, ch7** (4 dedicated receive channels).
- **Memory:** 48 PulseCode symbols per channel block (`SOC_RMT_MEM_WORDS_PER_CHANNEL = 48`).
  Requesting `mem_block_symbols > 48` causes the driver to borrow the adjacent channel's block,
  reducing how many channels can be independently allocated. To run all four TX channels
  simultaneously, keep `mem_block_symbols ≤ 48` per channel.
- **DMA:** `SOC_RMT_SUPPORT_DMA = 1`. The S3 is the only ESP32-family chip with DMA-capable RMT.
  However, the Rust esp-hal 1.0.0 RMT driver does **not** yet expose a DMA backend (no
  `with_dma` option in `TxChannelConfig`). The interrupt-driven ping-pong path is used. This is
  not a blocker for step generation, but note that a future DMA backend would eliminate all CPU
  overhead for step pulse trains. Monitor esp-hal releases.
- **Timing resolution:** RMT source clock is 80 MHz APB. With `clk_divider = 1`, one tick = 12.5 ns.
  The 15-bit duration field gives a maximum pulse half of 32767 ticks ≈ 409.6 µs (adequate for any
  stepper step pulse width). With `clk_divider = 80`, one tick = 1 µs for convenient integer
  calculations.
- Each PulseCode symbol is one 32-bit word: `[level2:1 | len2:15 | level1:1 | len1:15]`. A
  PulseCode encodes one complete HIGH+LOW step pulse period. A buffer must end with
  `PulseCode::end_marker()` (both lengths = 0) or transmission hangs.

### Axis-to-channel mapping
Each axis has its own dedicated TX channel. No multiplexing, no compromise.

| Axis | RMT TX channel | GPIO   |
|------|----------------|--------|
| X    | ch0            | GPIO1  |
| Y    | ch1            | GPIO2  |
| Z    | ch2            | GPIO4  |
| spare| ch3            | GPIO18 |

All three axes can transmit simultaneously and independently, coordinated by the DDA in the motion
executor (DOC-05). The spare ch3 is available for a future 4th axis.

### esp-hal 1.0.0 RMT API (init + TX)
```rust
let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80))?;

let mut ch0 = rmt.channel0.configure_tx(
  peripherals.GPIO1,
  TxChannelConfig::default()
    .with_clk_divider(1)
    .with_idle_output_level(Level::Low)
    .with_idle_output(true)
    .with_mem_block_symbols(48), // Stay within one block.
)?;
// Transmit a burst, then reclaim the channel for reuse.
let txn = ch0.transmit(&buf)?;  // buf: &[PulseCode], ends with end_marker().
ch0 = txn.wait().await?;
```
Channels 1 and 2 are initialized identically on their respective pins. All three channels are
driven concurrently from the motion executor task on core 1 using `embassy-futures::join::join3`
or equivalent to await all three channel completions before the next burst.

### Step pulse timing requirements (grblHAL conventions)
- `$0` Step pulse time (µs): default 10 µs; practical minimum 2 µs. The TMC2209 needs only ≥100 ns
  STEP high/low; 2–10 µs is comfortable.
- `$29` Direction setup delay (µs): the direction GPIO is set, then a configurable delay is inserted
  before the first step pulse. At minimum 2 µs; for opto-isolated drivers 5–15 µs may be required.
  Encode the setup delay as the LOW duration before the first PulseCode's rising edge, or via a
  short `embassy-time::Timer::after` await before starting transmission.
- Each PulseCode in the burst has: HIGH duration = `$0` ticks, LOW duration = step_period − `$0`
  ticks. Varying the LOW duration across symbols implements the velocity ramp
  (acceleration/deceleration).

### How the motion executor feeds the RMT engine
1. Pop the active planner block. The segment generator computes a burst: N step pulses at a
   constant-rate approximation of the current velocity profile point (trapezoidal ramp realized as
   a series of short constant-rate segments, as in grbl's stepper segment buffer).
2. Build `PulseCode` arrays for each axis. For each tick in the burst, the DDA decides whether each
   subordinate axis steps on that tick; axes that do not step get a zero-length PulseCode
   (effectively a no-op tick) or share the burst timing by omitting their PulseCode entirely
   (channel idle stays LOW). In practice, build a burst array per axis where silent ticks are
   encoded as full-period-LOW PulseCodes to keep all three channel bursts the same length and
   synchronized by their `join3` await.
3. Transmit on all three channels; await completion of all three. Chain the next burst. Motion is
   continuous as long as the planner block queue is non-empty.
4. A `FEED_HOLD` Signal from core 0 is checked at the boundary between bursts; do not split a burst
   in flight.
5. Keep bursts to ≤ 48 PulseCode symbols per channel (one memory block) for the initial
   implementation, avoiding the multi-block ping-pong refill path.

### Max achievable step rate
With a 10 µs pulse width and 2 µs minimum LOW, the maximum theoretical rate is
1 / (10 + 2) µs ≈ 83 kHz per axis. For PCB milling with typical 1/16 microstepping on 2 mm lead
screws (steps/mm ≈ 800), 83 kHz ≈ 6 250 mm/min — well above PCB milling feed rates. At 1/32 or
higher microstepping headroom remains comfortable.

---

## DOC-03: TMC2209 Driver Subsystem

### UART wiring topology (three drivers on one bus)
All three TMC2209 PDN_UART pins join one half-duplex single-wire bus driven by ESP32-S3 UART1
(GPIO9). A 1 kΩ series resistor on the MCU TX line is recommended; MCU TX and RX are tied to the
shared PDN_UART node. Every byte transmitted is echoed back on RX (half-duplex loopback) and must
be discarded by the driver.

Each driver is addressed by node address 0–3 set on its MS1/MS2 pins:

| Node addr | MS1  | MS2  | Axis |
|-----------|------|------|------|
| 0         | LOW  | LOW  | X    |
| 1         | HIGH | LOW  | Y    |
| 2         | LOW  | HIGH | Z    |
| 3         | HIGH | HIGH | spare|

MS1/MS2 have internal pull-downs (floating ⇒ address 0). In UART mode these pins set the node
address, not microstepping; microstepping is set via `CHOPCONF.MRES` with
`GCONF.mstep_reg_select = 1`.

### Datagram protocol
- **Write (8 bytes):** `[0x05, NODE, REG|0x80, D3, D2, D1, D0, CRC]`
- **Read request (4 bytes):** `[0x05, NODE, REG, CRC]`
- **Read reply (8 bytes):** `[0x05, 0xFF, REG, D3, D2, D1, D0, CRC]`
- **CRC8-ATM:** polynomial x⁸+x²+x+1 (0x07), init 0x00, applied LSB-first over all bytes except
  the CRC byte itself.
- **SLAVECONF/SENDDELAY (reg 0x03):** in multi-node buses set SENDDELAY ≥ 2 (units of 8 bit-times)
  for clean bus turn-around between devices.

### Register map (verified addresses)

| Register   | Addr | Use                                                            |
|------------|------|----------------------------------------------------------------|
| GCONF      | 0x00 | pdn_disable=1, mstep_reg_select=1, I_scale_analog=0           |
| GSTAT      | 0x01 | Clear reset/error flags                                        |
| IFCNT      | 0x02 | Verify writes (increments per successful write)                |
| SLAVECONF  | 0x03 | SENDDELAY for multi-node bus                                   |
| IOIN       | 0x06 | Read VERSION field to confirm driver presence                  |
| IHOLD_IRUN | 0x10 | Run/hold current scaling (5-bit CS values)                     |
| TPOWERDOWN | 0x11 | Standstill power-down delay                                    |
| TSTEP      | 0x12 | (read) Measured step interval                                  |
| TPWMTHRS   | 0x13 | StealthChop → SpreadCycle crossover threshold                  |
| TCOOLTHRS  | 0x14 | CoolStep / StallGuard minimum velocity threshold               |
| SGTHRS     | 0x40 | StallGuard4 stall threshold (future sensorless homing)         |
| SG_RESULT  | 0x41 | (read) StallGuard load value                                   |
| COOLCONF   | 0x42 | CoolStep configuration                                         |
| CHOPCONF   | 0x6C | Microstep resolution (MRES), interpolation, TOFF               |
| DRV_STATUS | 0x6F | (read) Overtemp, short, stall, CS_actual                       |
| PWMCONF    | 0x70 | StealthChop PWM auto-scaling configuration                     |

### Register initialization sequence at startup
For each axis node (0, 1, 2):
1. Read IOIN; confirm VERSION = 0x21 in bits 31..24. If absent, flag and skip (driver may be in
   standalone VREF mode; motion can still proceed).
2. Write `GCONF = pdn_disable(1) | mstep_reg_select(1) | I_scale_analog(0) | multistep_filt(1)`.
3. Write `CHOPCONF`: TOFF=3, TBL=2, default HSTRT/HEND, MRES = desired microstep resolution
   (e.g. 4 ⇒ 1/16), intpol=1 (256-microstep interpolation enabled).
4. Write `IHOLD_IRUN` with computed IRUN and IHOLD values (see current API below).
5. Write `TPOWERDOWN` (e.g. 20 ≈ 0.3 s standstill timeout) and `TPWMTHRS` for the
   StealthChop/SpreadCycle crossover velocity.
6. Optionally write `PWMCONF` with pwm_autoscale=1, pwm_autograd=1 for automatic StealthChop tuning.
7. Read IFCNT before and after the sequence to confirm all writes were accepted (counter increments
   per accepted datagram).

### Runtime current scaling API
IRUN and IHOLD are 5-bit current-scale values (CS = 0…31). The RMS current formula:

  `I_rms = (CS + 1) / 32 × V_fs / (R_sense + 0.02 Ω) × 1/√2`

- V_fs ≈ 0.325 V when vsense=0; ≈ 0.180 V when vsense=1.
- The Adafruit TMC2209 breakout (product 6121) uses **0.05 Ω** sense resistors — verify against
  the board schematic before computing CS. This differs from common clone values (0.11 Ω).
- Per the datasheet: aim for IRUN in the range 16–31 for best microstep performance at the chosen
  motor current.

```rust
/// Compute and write run/hold current; handles vsense selection automatically.
pub fn set_current_ma(
  &mut self,
  axis: Axis,
  run_ma: u16,
  hold_ma: u16,
) -> Result<(), TmcError>;
```

### StallGuard2/4 upgrade path (future sensorless homing)
Program `SGTHRS` (0x40) for the stall threshold, `TCOOLTHRS` (0x14) for the minimum valid
velocity, and read `SG_RESULT` (0x41) or poll the DIAG output pin during a sensorless homing
cycle. This is a documented future upgrade; the initial build uses mechanical switches (DOC-06).

---

## DOC-04: GCode Parser & Command Pipeline

### Tokenizer and parser architecture
A streaming, allocation-free parser in the `gcode` crate. Each input line (terminated by CR or LF)
is:
1. **Lexed** into `(letter, f32)` word pairs, stripping whitespace and `(...)`/`;` comments,
   case-insensitive.
2. **Validated** against the supported modal groups; unknown words produce a grblHAL error code.
3. **Applied** to a modal state (current motion mode, units G20/G21, distance G90/G91, plane G17,
   active coordinate offsets), then emitted as a `PlannerCommand`.

### Minimum viable GCode subset

| Code      | Meaning                                              |
|-----------|------------------------------------------------------|
| G0        | Rapid move                                           |
| G1        | Linear feed move                                     |
| G2 / G3   | CW / CCW arc                                         |
| G4        | Dwell (P seconds)                                    |
| G17       | XY plane select (only plane required for PCB milling)|
| G20 / G21 | Units: inch / mm                                     |
| G28 / G30 | Go to predefined position                            |
| G90 / G91 | Absolute / incremental distance mode                 |
| G92       | Set coordinate offset                                |
| M3 / M4   | Spindle CW / CCW                                     |
| M5        | Spindle stop                                         |
| M30       | Program end                                          |
| F         | Feed rate word                                       |
| S         | Spindle speed word                                   |

### Command queue between parser and planner
`Channel<CriticalSectionRawMutex, PlannerCommand, 8>`. The parser enqueues only on successful
validation; errors produce an `error:N` response and the line is discarded. Backpressure: when the
channel is full, the parser awaits, which throttles line assembly and provides natural flow control
equivalent to grbl's character-counting protocol.

### Error handling and grblHAL-compatible responses
- Success: `ok`.
- Failure: `error:N` where N is a grblHAL status code. Examples: `error:1` (expected command
  letter), `error:2` (bad number format), `error:20` (unsupported command), `error:33` (invalid
  motion target or arc geometry).
- Alarms: `ALARM:N`. Codes 0–9 match legacy grbl; grblHAL extends them in `alarms.h`.

### Settings ($-commands) storage
- `$$` dumps all settings as `$<n>=<value>` lines.
- `$x=val` sets and persists a setting.
- `$I` returns build/version info; `$G` returns modal state (`[GC:...]`).
- Persistence uses `esp-storage` writing a versioned, CRC-checked settings struct to a dedicated
  flash region. On boot, load and CRC-check; on mismatch, restore compiled defaults.
- Stored settings include: `$0` (step pulse µs), `$1` (idle delay), `$2/$3` (step/dir invert
  masks), `$10` (status report mask), `$11` (junction deviation), `$12` (arc tolerance), `$20/$21`
  (soft/hard limits), `$22` (homing enable), `$23` (homing dir invert mask), `$24/$25` (homing
  feed/seek rates), `$26` (homing debounce ms), `$27` (homing pull-off mm), `$30/$31` (max/min
  spindle RPM), `$100–$102` (steps/mm per axis), `$110–$112` (max rate mm/min), `$120–$122`
  (acceleration mm/s²), `$130–$132` (max travel mm).

---

## DOC-05: Motion Planner & Kinematics

### FPU note
The ESP32-S3's Xtensa LX7 includes a hardware single-precision FPU (IEEE 754 single, 32-bit). All
`f32` arithmetic in the planner and arc interpolation executes in hardware. Division and square root
are multi-instruction sequences on the LX7 FPU (not single-cycle), but are orders of magnitude
faster than soft-float. The `libm` crate is still used for transcendentals (`sinf`, `cosf`,
`atan2f`) because the LX7 FPU does not have hardware transcendentals; `sqrtf` may be lowered to a
hardware instruction by LLVM depending on the Xtensa target configuration — verify in the generated
assembly if performance is critical.

### Block queue data structure
A ring buffer of planner blocks (`heapless::Vec`-backed, fixed capacity). 16–32 blocks is ample
for PCB milling look-ahead (grbl uses ~16 on AVR; the S3 can comfortably hold 32). Each block
stores: target step counts per axis, total step count for the dominant axis, unit direction vector,
nominal speed, entry speed (computed), max entry speed, acceleration, and travel in mm.

### Trapezoidal velocity profiler
Mirrors grbl: the planner computes only the optimal *entry speed* per block via forward/reverse
passes; the actual per-block velocity profile (cruise-only, accel-cruise, cruise-decel,
full-trapezoid, accel-only, decel-only) is computed by the segment generator on execution. Two
passes:
- **Reverse pass:** from newest block backward, cap each junction entry speed by the maximum
  reachable from the next block's entry speed under max acceleration over the block's travel.
- **Forward pass:** from oldest block forward, cap exit speed by what's reachable from entry speed
  + acceleration over the block's travel.

### Junction (cornering) velocity — junction deviation
Use grbl's junction-deviation centripetal model: the maximum junction speed `v = sqrt(a × R)`,
where R is derived from the user `$11` junction deviation and the junction angle via the dot
product of unit vectors (half-angle identity; no `atan2` call required in the hot path). Junction
deviation default ≈ 0.01 mm.

### Arc interpolation (G2/G3) — chord tolerance
Subdivide arcs into short linear segments with chord error ≤ `$12` arc tolerance (default
0.002 mm). Segment count scales with radius: compute via `acosf(1 − tol/r)`. Each segment feeds
the planner as a small linear move. Keep `$12` near default; extremely small values produce
thousands of segments and starve the planner block queue.

### Steps-per-mm and unit conversion
`steps_per_mm[axis]` from `$100–$102`. Target position in mm (after G20/G21 and coordinate offset
application) converts to target steps as `round(pos_mm × steps_per_mm)`. The planner works in
step space; feed rates in mm/min convert to step rates using the dominant-axis steps/mm.
Microstepping (DOC-03) multiplies effective steps/mm and must match `$100–$102`.

### DDA / Bresenham multi-axis coordination
The dominant axis (largest step count in a block) advances every tick; subordinate axes use
Bresenham error accumulation to decide whether to step on each dominant-axis tick. This guarantees
straight-line coordination with zero floating-point cost in the inner loop. Because each axis has
its own RMT TX channel (DOC-02), the DDA decision for each tick is encoded as a PulseCode
(step present: HIGH for `$0` ticks, LOW for remainder) or a silent code (no step: full period LOW)
into each channel's burst array independently.

---

## DOC-06: Homing & Limit Switch Subsystem

### Homing cycle sequence (grblHAL convention)
Default order: **Z homes first** (lifts the tool clear of the workpiece), then **X and Y home
together**. Each axis phase:
1. **Seek:** move toward the switch at `$25` (homing seek rate) until the switch triggers.
2. **Pull-off:** back off by `$27` (homing pull-off, e.g. 1 mm) to release the switch.
3. **Locate (feed):** approach again at `$24` (homing feed rate) for an accurate trigger point.
4. **Final pull-off** to clear the switch.

The homing direction per axis is set by the `$23` invert mask. After homing, machine zero is
established. If no switch is found within 1.5× max travel (`$130–$132`), raise a homing alarm.

### GPIO interrupt configuration for NC switches
NC switches are wired to GND with internal pull-ups enabled: an intact switch holds the pin LOW;
when triggered (or on a broken wire), the pin reads HIGH. Each limit GPIO is configured as input
with pull-up and a rising-edge interrupt. The ISR sets the `LIMIT_TRIGGERED` Signal; the motion
executor responds (hard-limit alarm during normal run; phase advance during homing).

> Bring-up note: grblHAL defaults to NC inputs; the controller will start in ALARM mode if limit
> inputs are not wired. During initial bench testing, jumper each limit GPIO to GND to satisfy the
> NC-high expectation, or temporarily enable `$5=1` (limit pin invert) in settings.

### Debounce strategy
On limit interrupt, sample again after the `$26` debounce interval (ms, e.g. 25–250 ms) via an
`embassy-time::Timer::after` await before accepting the trigger. For hard limits during running, a
few-ms software debounce plus the NC pull-up arrangement rejects EMI glitches.

### Upgrade path: NPN NC opto-isolated sensors
NPN NC sensors sink current to GND when not triggered. The NC logic is preserved, so the existing
pull-up + interrupt configuration and grblHAL invert masks carry over with minimal change. Required
additions:
- A level-shift or opto-isolated stage between the sensor output (typically 6–36 V) and the 3.3 V
  ESP32-S3 GPIO.
- Potentially longer direction setup delay (`$29`) if opto-isolated stepper drivers are also added
  (some require ≥5–15 µs direction setup before a step pulse).
- All limit inputs are behind the `DigitalIn` trait (DOC-09); only the firmware wiring layer
  changes.

---

## DOC-07: Spindle Control (WS55-220)

### Interface (WS55-220 spec sheet)
The WS55-220 speed input (SV terminal) accepts 0–10 VDC analog. There is no documented logic-level
PWM input. The board also provides a +10 V reference (small current, for an external pot), an EN
start/stop terminal (to GND = run), and an F/R direction terminal.

The firmware generates spindle speed as a LEDC PWM on GPIO13 and converts it to a 0–10 V analog
level through an external RC low-pass filter + non-inverting op-amp gain stage (3.3 V PWM → 0–10
V). This conditioning circuit is a required hardware element and must appear in the BOM.

### LEDC configuration (ESP32-S3)
The S3 LEDC operates in low-speed mode only (same as the C6). Configure LEDC timer 0 + channel 0.
Recommended PWM frequency: 1–20 kHz with ≥10-bit resolution so the RC filter produces a smooth
analog. At 5 kHz a 13-bit resolution is comfortable on the S3 LEDC. Duty 0 % → 0 V → spindle
stopped; duty 100 % → 10 V → max RPM.

### Speed mapping
`$30` = max spindle RPM, `$31` = min RPM. Map the S word (RPM) linearly to LEDC duty:

  `duty = (S − $31) / ($30 − $31) × full_scale`

Clamp to [0, full_scale]. S = 0 implies M5 (spindle off) regardless.

### M3/M4/M5 and enable/direction sequencing
- **M3 (CW):** set SPIN_DIR for CW, set LEDC duty for the commanded S value, then assert SPIN_EN
  (pull GPIO14 low → WS55-220 EN to GND = run).
- **M4 (CCW):** set SPIN_DIR for CCW, set duty, assert SPIN_EN.
- **M5 (stop):** de-assert SPIN_EN and set duty to 0.

### Safety interlock
- Direction changes (M3 ↔ M4) force M5, a configurable spin-down dwell, then restart. Never
  reverse a running spindle.
- On any ALARM, soft reset (0x18), or hard limit trigger, the spindle task immediately de-asserts
  SPIN_EN and zeros LEDC duty, independent of motion state.
- A feed hold (`!`) halts motion but does **not** stop the spindle (matching grblHAL semantics);
  only ALARM/soft-reset forces spindle off.
- After M3/M4, the planner inserts a spin-up dwell before the first cutting move begins.

---

## DOC-08: USB CDC Serial Interface

### USB Serial/JTAG CDC-ACM on ESP32-S3
The S3 has two USB controllers sharing the GPIO19/GPIO20 internal PHY:
- **USB Serial/JTAG controller** — a fixed-function CDC-ACM peripheral identical to the one on the
  C6. Implemented in hardware, implements `embedded-io` / `embedded-io-async` traits in esp-hal.
  This is the recommended path for a grblHAL host link: no USB stack configuration required, works
  out of the box, no external components.
- **USB OTG (DWC2 full-speed)** — a full-speed (12 Mbit/s) OTG controller also on GPIO19/20.
  `esp_hal::otg_fs` + `embassy-usb` can build a fully custom CDC-ACM device stack. This path
  requires the `USB_PHY_SEL` eFuse to be burned to route the internal PHY to the OTG controller
  (the default routes it to the Serial/JTAG controller), or an external PHY. Use this path only if
  custom USB descriptors are required.

For this firmware: **use the USB Serial/JTAG controller**. It is simpler, reliable, and the
grblHAL host link does not require a custom USB descriptor.

### Receive ring buffer and line parsing
`usb_rx` awaits bytes from the USB Serial/JTAG async driver into a ring buffer. Each byte is
scanned for:
1. **Real-time single-byte commands** — processed immediately, never buffered:
   - `0x3F` (`?`): set `STATUS_REQUEST` Signal.
   - `0x21` (`!`): set `FEED_HOLD` Signal.
   - `0x7E` (`~`): set `CYCLE_START` Signal.
   - `0x18` (Ctrl-X): set `SOFT_RESET` Signal; flush parser channel, planner queue, stop motion.
2. **Line terminators** (CR / LF): push the accumulated line to the `gcode_parser` channel.
3. **Printable bytes**: append to the current line (128-char cap; overflow → `error`, discard).

Real-time bytes are intercepted before line assembly so `?` mid-line still triggers an immediate
status report without disturbing the partial line — matching grblHAL real-time command semantics.

### grblHAL status report generation
`status_reporter` builds the `<...>` report on `STATUS_REQUEST` Signal or an auto-report interval.
Format:

```
<State|MPos:x,y,z|FS:feed,spindle|WCO:x,y,z|Pn:XYZ|Bf:blocks,bytes|Ov:f,r,s>
```

- **State:** `Idle`, `Run`, `Hold:0`/`Hold:1`, `Jog`, `Alarm`, `Door`, `Check`, `Home`, `Sleep`.
- Report sends either `MPos:` or `WPos:`, never both. `WCO:` is sent every ~30 reports or on
  change (`WPos = MPos − WCO`).
- **FS:** current feed rate and spindle speed.
- **Pn:** triggered input pins, e.g. `Pn:ZP` for Z-limit + probe. Omitted if nothing triggered.
- **Bf:** available planner blocks and RX buffer bytes.
- **Ov:** feed/rapid/spindle override percentages.
- The `$10` status report mask selects which optional fields are included.

All responses (`ok`, `error:N`, `ALARM:N`, reports, `$$` output) are sent through the `usb_tx`
channel to avoid concurrent writes from multiple tasks.

---

## DOC-09: Testing & Quality Strategy

### Toolchain setup (Xtensa — required before any build)
The ESP32-S3 uses the Xtensa LX7 ISA, which is **not** supported by the upstream LLVM/Rust
toolchain. Espressif maintains an unofficial Xtensa LLVM fork. Setup:

```sh
# Install espup (Espressif's Rust toolchain manager).
cargo install espup
espup install        # Downloads and installs the Xtensa-enabled Rust toolchain.
# Follow espup's instructions to source the environment export file.
source $HOME/export-esp.sh   # or equivalent for your shell.
```

The `.cargo/config.toml` in the workspace root must set:
```toml
[build]
target = "xtensa-esp32s3-none-elf"

[target.xtensa-esp32s3-none-elf]
runner = "espflash flash --monitor"
```

Library crates (`gcode`, `planner`, etc.) have no Xtensa dependency and continue to build and test
with stock Rust on `x86_64-unknown-linux-gnu` (or macOS equivalent) via `cargo test`.

### What is host-testable vs. on-target
Library crates are `no_std` and host-compilable; test them on `x86_64` with `cargo test`:

| Crate      | Host-testable content                                                      |
|------------|----------------------------------------------------------------------------|
| `gcode`    | Tokenizer, parser, modal state, error code mapping. Table-driven tests: input line → expected `PlannerCommand` or `error:N`. |
| `planner`  | Forward/reverse pass velocity planning, junction speed, trapezoid segment classification. Property tests (exit speed never exceeds physical limits). |
| `motion`   | DDA/Bresenham step distribution (step ratios match expected), unit conversion (mm ↔ steps). |
| `drivers`  | TMC2209 datagram encode/decode, CRC8-ATM against known-good datasheet vectors, current→CS formula. |
| `protocol` | Status report formatting, `$$`/`$x=val` round-trip serialization.          |

On-target tests (require hardware or `embedded-test` + `probe-rs`):
- RMT pulse timing on a logic analyzer: verify step pulse width (`$0`), direction setup time
  (`$29`), and step rate accuracy across all three axes simultaneously.
- UART round-trip with real TMC2209s: read IOIN VERSION = 0x21, write IHOLD_IRUN, verify motor
  torque and silence.
- LEDC duty vs. conditioned analog voltage on a multimeter.
- USB CDC enumeration and `$$` round-trip with a real host.
- Note: `defmt-test` supports ARM only. For the Xtensa S3 use `embedded-test` with `probe-rs`,
  which supports Xtensa targets.

### Hardware abstraction traits
```rust
/// Emits coordinated step bursts; impl over RMT on target, recorder Vec in tests.
pub trait StepSink {
  fn emit_burst(&mut self, steps: &[StepEvent]) -> Result<(), StepError>;
}

/// Normalized 0.0..=1.0 spindle duty.
pub trait PwmSink {
  fn set_duty(&mut self, frac: f32) -> Result<(), PwmError>;
}

/// Limit / control input.
pub trait DigitalIn {
  fn is_active(&self) -> bool;
}

pub trait DigitalOut {
  fn set(&mut self, level: bool) -> Result<(), ()>;
}

/// Half-duplex TMC UART transport; impl over UART1 on target, byte buffer in tests.
pub trait TmcBus {
  fn write_reg(&mut self, node: u8, reg: u8, val: u32) -> Result<(), TmcError>;
  fn read_reg(&mut self, node: u8, reg: u8) -> Result<u32, TmcError>;
}
```

The planner and motion executor are generic over `StepSink`. Tests inject a `RecordingSink` that
captures every `StepEvent` for assertion. `embedded-hal-mock` covers GPIO and UART trait mocking.

### Integration test plan (staged)
1. **Bench bring-up:** flash, confirm USB CDC enumeration, `$$` dump, `$I` banner. Confirm both
   cores start without fault.
2. **Single-axis jog:** logic analyzer on X_STEP/X_DIR; verify step count, direction, pulse width.
   Calibrate `$100`.
3. **Three-axis simultaneous:** G1 diagonal on XYZ; verify Bresenham DDA coordination on scope.
4. **TMC UART:** confirm VERSION = 0x21 on all three nodes, set current, verify motor torque and
   silence.
5. **Homing:** dry-run with hand-triggered switches; verify Z-first then XY order, pull-off
   distance, and alarm on overrun.
6. **Spindle:** measure conditioned 0–10 V vs. S word; verify M3/M4/M5 sequencing and alarm
   fail-safe.
7. **Full job:** stream a real PCB isolation-milling GCode file from ioSender/UGS.

### Code style mandates
- **No `unwrap()` or `expect()` in library code.** Propagate via `Result` or handle explicitly.
  `expect` is permitted only in `main`/init paths where failure is genuinely unrecoverable.
- **Two-space indentation** throughout.
- **Short, focused functions.** Single responsibility; prefer many small functions.
- **Comment lines target ~120 characters and end with a period.**
- **All public APIs carry doc comments** (`///`).
- **Trait-based hardware abstraction throughout** so the planner/parser/driver logic is
  host-testable without hardware.
- Enable `#![deny(unsafe_code)]` in all library crates. The `firmware/` crate may use `unsafe`
  only at esp-hal boundaries.
- Enable `#![deny(warnings)]` in CI.

## Recommendations

**Build order:**
1. Toolchain: `espup install`, verify a blinky example flashes.
2. `hal_traits` + `gcode` + `protocol` — host tests green.
3. `planner` + `motion` — host tests green.
4. `drivers` (TMC datagram + CRC) — host tests green.
5. `firmware` bring-up: USB CDC enum, `$$`, `$I`.
6. RMT single-axis step generation on a logic analyzer.
7. All three RMT TX channels simultaneously — DDA verification.
8. TMC UART bring-up (all three nodes).
9. Dual-core executor split: confirm core 1 `motion_executor` runs without starving core 0.
10. Homing.
11. Spindle.
12. Full job.

## Caveats

- **`unstable` feature scope:** RMT, LEDC, USB Serial/JTAG, and `CpuControl` (multicore) are all
  behind the `unstable` feature in esp-hal 1.0.0. "Unstable" means API stability is not
  guaranteed across minor releases — the drivers are functional. Pin your esp-hal version and test
  before upgrading.
- **`mem_block_symbols ≤ 48`:** requesting more symbols than one memory block causes the RMT driver
  to borrow the next channel's block. With all four TX channels in use, keep each channel's burst
  to 48 symbols to avoid allocation failures and a known ESP-IDF two-block bug.
- **RMT DMA in Rust:** the S3 hardware supports RMT DMA, but esp-hal 1.0.0 does not expose it.
  The interrupt-driven path is perfectly adequate for PCB milling step rates. Revisit if maximum
  possible step rate becomes a concern.
- **USB PHY selection:** using `otg_fs` requires burning the `USB_PHY_SEL` eFuse or an external
  PHY. The `usb_serial_jtag` path has no such requirement and is strongly preferred for this
  application.
- **Xtensa toolchain is a separate install from stock Rust.** `rustup default stable` is not
  sufficient; `espup install` must be run and the export script sourced in every shell before
  building firmware. CI pipelines must also install `espup`. Library crates (`gcode`, `planner`,
  etc.) continue to use stock Rust.
- **GPIO pin assignments are a recommended default.** Verify GPIO0, GPIO3, GPIO45, GPIO46
  (strapping pins) are not held to non-default levels by external circuitry at boot, and confirm
  GPIO19/20 are left to the USB subsystem.
- **WS55-220 has no documented logic-PWM speed input.** The 0–10 V conditioning stage is
  mandatory; any direct PWM injection is an undocumented hardware modification.
- **Adafruit 6121 sense resistors are 0.05 Ω.** Verify against the schematic before computing
  IRUN/IHOLD CS values; an incorrect R_sense silently mis-scales motor current.
