# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project status

**Galdr** is a CNC PCB milling system in two halves:

- `crates/firmware` — grblHAL-compatible 3-axis CNC firmware for an **ESP32-S3**, `no_std` Rust on esp-hal 1.0 +
  the Embassy async runtime.
- `crates/skirnir` — a native **Linux GCode sender** (host app) that streams GCode to the firmware over USB CDC serial.

Both crates are currently empty scaffolds (`fn main() { println!("Hello, world!"); }`). The actual design lives in
`docs/` and has **not yet been implemented**. Treat `docs/` as the authoritative specification — read the relevant
doc before implementing a subsystem.

> Naming note: the docs use generic placeholder crate names (`pcb-mill-fw`, `cnc-core`, `gcode`, `planner`, `motion`,
> `drivers`, `protocol`, `hal_traits`). The real crates are `firmware` and `skirnir`. The doc workspace layout (many
> small lib crates) is a *plan*, not the current tree. Confirm the intended crate breakdown before scaffolding it. The
> docs also say firmware `edition = "2021"`; the actual `Cargo.toml` files use `edition = "2024"`.

## Where to read first

| Doc | Covers |
|-----|--------|
| `docs/00-architecture.md` | Full firmware spec, DOC-00–DOC-09: hardware/GPIO manifest, Embassy task split, RMT step generation, TMC2209 driver, GCode parser, motion planner, homing, spindle, USB CDC, testing. **Start here for firmware work.** |
| `docs/gcode-streaming.md` | grblHAL streaming protocol: character-counting flow control, real-time commands, status reports, handshake, `$`-settings, probing. Shared contract between firmware and `skirnir`. |
| `docs/tlo-offsets.md` | Tool-length-offset / Z-probe workflow (G38.x, WPos/MPos/WCO/TLO) for no-touch-plate PCB probing. |
| `docs/native-app.md` | `skirnir` design: GUI framework analysis. **egui (eframe) is the chosen UI**; isolate the serial/streaming engine into its own framework-agnostic module built on `tokio-serial`. |

## Build & test

Workspace root is the repo root (`Cargo.toml` declares `members = ["crates/firmware", "crates/skirnir"]`).

```sh
cargo build              # current scaffold builds with stock Rust
cargo test               # host tests
cargo test -p <crate>    # single crate
cargo test -p <crate> <test_name>   # single test
```

`firmware` is excluded from `default-members`, so the bare commands above stay on the host toolchain and skip it.
**`--workspace`/`--all` ignore `default-members`** and will try to build `firmware` on the host (which fails) — pass
`--exclude firmware` with those flags, or build `firmware` on its own from within `crates/firmware` (see below).

**Firmware (ESP32-S3 / Xtensa LX7) requires a separate toolchain.** The Xtensa ISA is not supported by upstream Rust
— install Espressif's fork before building/flashing firmware:

```sh
cargo install espup
espup install
source $HOME/export-esp.sh    # source in every shell before firmware builds (and in CI)
```

Firmware target is `xtensa-esp32s3-none-elf` with `runner = "espflash flash --monitor"` (set in a `.cargo/config.toml`
that does not yet exist — create it when scaffolding firmware). Pure-logic library code is `no_std` but has **no
esp-hal dependency**, so it compiles and unit-tests on the host with stock Rust — keep it that way (see "Hardware
abstraction" below).

## Architecture essentials (firmware)

- **Dual-core split.** Core 1 (`APP_CPU`) runs *only* the `motion_executor` task on a high-priority `InterruptExecutor`
  for real-time step generation — uncontested CPU, preempts nothing. Core 0 (`PRO_CPU`) runs everything else (USB
  comms, GCode parser, planner, TMC manager, spindle, status reporter, homing) on a thread-mode executor.
- **Step generation via RMT.** Each axis (X/Y/Z) gets its own dedicated RMT TX channel (ch0/1/2); ch3 is spare. Keep
  `mem_block_symbols ≤ 48` (one memory block) per channel or the driver borrows the adjacent channel's block. esp-hal
  1.0 exposes no RMT DMA backend yet — use the interrupt path.
- **Inter-task comms** use `embassy-sync` primitives with `CriticalSectionRawMutex`. Planner → motion executor goes
  through a `BlockQueue` ring buffer behind a `Mutex` plus a `BLOCK_AVAILABLE` signal; real-time bytes
  (`?`/`!`/`~`/`0x18`) are intercepted in `usb_rx` and dispatched via `Signal`s, never line-buffered.
- **Motion model** mirrors grbl: planner computes only optimal per-block entry speeds (forward/reverse passes +
  junction-deviation cornering); the segment generator realizes the trapezoidal profile at execution. Multi-axis
  coordination is Bresenham/DDA in step space.
- **TMC2209 drivers** share one half-duplex single-wire UART bus (UART1); node addresses 0–3 set via MS1/MS2. Datagrams
  use CRC8-ATM (poly 0x07). Adafruit 6121 breakout uses **0.05 Ω** sense resistors — verify before computing IRUN/IHOLD
  current scaling.
- **Spindle** (WS55-220) has no logic-PWM input: LEDC PWM on GPIO13 → external RC + op-amp → 0–10 V. Direction reversal
  always forces M5 + spin-down dwell first; ALARM/soft-reset force spindle off.

## grblHAL protocol contract (firmware ↔ skirnir)

- Emit exactly one `ok` / `error:N` per consumed line — this is the *only* thing driving host flow control. Never emit
  a spurious `ok`.
- Treat `CRLF`/`LFCR` as a **single** line terminator (avoid the legacy "double-ok" bug).
- ESP32-S3 native USB cannot be hard-reset by the host — emit the welcome banner on every boot and every soft-reset
  (0x18), and answer `0x87`/`$I+` so senders detect readiness.
- After a GCode error, hold subsequent lines in an error state until reset / empty line / `$` command (grblHAL safety
  behavior, differs from legacy grbl).
- `skirnir`'s streaming engine implements host-side character counting against the advertised RX buffer size, injects
  real-time single-byte commands out-of-band, and parses `ok`/`error`/`<...>`.

## Hardware abstraction & code style (from DOC-09)

- All hardware access goes behind traits (`StepSink`, `PwmSink`, `DigitalIn`/`DigitalOut`, `TmcBus`) so
  planner/parser/driver logic is host-testable with recording/mock impls. Only the firmware wiring layer touches esp-hal.
- **No `unwrap()`/`expect()` in library code** — propagate via `Result`. `expect` is allowed only in `main`/init paths
  where failure is genuinely unrecoverable.
- **Two-space indentation** (enforced by `.editorconfig`), LF line endings, final newline.
- Short single-responsibility functions; `///` doc comments on all public APIs; comment lines target ~120 chars and end
  with a period.
- `#![deny(unsafe_code)]` in library crates (firmware may use `unsafe` only at esp-hal boundaries); `#![deny(warnings)]`
  in CI.
