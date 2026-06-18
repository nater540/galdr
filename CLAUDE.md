# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project status

**Galdr** is a CNC PCB milling system. The workspace has four crates:

- `crates/firmware-core` — pure, `no_std`, **host-tested** logic: the GCode parser, motion planner, segment
  generator, grblHAL protocol/state machine, the homing state machine (DOC-06), coordinate systems, settings model,
  and the TMC2209 codec. No esp-hal dependency, so it compiles and unit-tests on the host with stock Rust.
- `crates/firmware` — the **ESP32-S3** binary wiring `firmware-core` to the hardware (esp-hal 1.0 + Embassy on the
  esp-rtos host): USB CDC comms, the dual-core task split, RMT step generation, the TMC UART bus, flash persistence,
  and the limit-switch inputs.
- `crates/galdr-proto` — the shared Protocol Buffers schema (micropb) for the settings/coordinate wire format, used
  by both the firmware flash records and the `$PBX` host-sync channel.
- `crates/skirnir` — a native **Linux GCode sender** (host app) that streams GCode over USB CDC serial.

The firmware side (`firmware-core` + `firmware` + `galdr-proto`) is substantially implemented and host-tested: the
GCode→planner→motion pipeline, grblHAL streaming, TMC2209 driver, settings + coordinate persistence, G38.x probing,
jogging, feed/rapid/spindle overrides, the **`$H` homing cycle**, **hard/soft limits**, and **limit-switch `Pn:`
status reporting** (all DOC-06) exist and are unit-tested off-target. The DOC-06 homing/limit *logic* is host-tested,
but its hardware boundary — the rising-edge limit IRQ + `$26` debounce, the NC broken-wire fail-safe, and the real
seek/locate timing — is compile-checked only and **not yet verified on the board** (see
`docs/homing-bench-checklist.md`). Genuinely stubbed at the hardware boundary (logic exists, no peripheral output):
**spindle PWM** (DOC-07 — the LEDC drive on GPIO13 is a `TODO` in `main`) and the coolant GPIO. `crates/skirnir` is
now a real app, not a scaffold: an egui/eframe GUI plus a framework-agnostic `tokio-serial` streaming engine
(character-counting flow control, `<...>`/`Pn:` status parsing, reconnection, endstop indicators), with ~229 host
tests run over a loopback transport. `docs/` remains the authoritative **design** spec — read the relevant doc
before extending a subsystem.

> Naming note: the docs use generic placeholder names (`pcb-mill-fw`, `cnc-core`, `gcode`, `planner`, `motion`,
> `drivers`, `protocol`, `hal_traits`) for what are now the `firmware-core` modules and the `firmware`/`galdr-proto`/
> `skirnir` crates. The doc's "many small lib crates" layout was a plan; the real tree folds the pure logic into the
> single `firmware-core` lib. The docs also say firmware `edition = "2021"`; the actual `Cargo.toml` files use
> `edition = "2024"`.

## Where to read first

| Doc | Covers |
|-----|--------|
| `docs/00-architecture.md` | Full firmware spec, DOC-00–DOC-09: hardware/GPIO manifest, Embassy task split, RMT step generation, TMC2209 driver, GCode parser, motion planner, homing, spindle, USB CDC, testing. **Start here for firmware work.** |
| `docs/gcode-streaming.md` | grblHAL streaming protocol: character-counting flow control, real-time commands, status reports, handshake, `$`-settings, probing. Shared contract between firmware and `skirnir`. |
| `docs/tlo-offsets.md` | Tool-length-offset / Z-probe workflow (G38.x, WPos/MPos/WCO/TLO) for no-touch-plate PCB probing. |
| `docs/native-app.md` + `docs/skirnir-design-brief.md` | `skirnir` design. egui (eframe) is the UI; the serial/streaming engine is a framework-agnostic module on `tokio-serial`. **Now built** — read these for intent, but `crates/skirnir/src/` is the source of truth. |
| `docs/homing-research-findings.md` | DOC-06 background: the verified grblHAL homing/limit behavioral contract that the implementation follows (cited research synthesis). Read before changing homing/limit semantics. |
| `docs/homing-bench-checklist.md` | Hardware-in-the-loop bring-up procedure for the homing cycle + limit switches. The DOC-06 hardware path is unverified until this is run on the board. |

## Build & test

Workspace root is the repo root (`Cargo.toml` `members = ["crates/firmware-core", "crates/firmware", "crates/skirnir",
"crates/galdr-proto"]`).

```sh
cargo build              # builds firmware-core + skirnir + galdr-proto on stock Rust (firmware is excluded)
cargo test               # host tests (run with RUSTFLAGS="-D warnings" to match CI)
cargo test -p <crate>    # single crate
cargo test -p <crate> <test_name>   # single test
```

`firmware` is excluded from `default-members`, so the bare commands above stay on the host toolchain and skip it.
`skirnir` has two default-on features — `gui` (eframe + rfd; `cargo run -p skirnir` launches the window) and `serial`
(`tokio-serial`; `cargo run -p skirnir -- --cli <port>` is the headless streaming path). Its tests run over an
in-memory loopback transport and need no hardware; build the UI explicitly with `cargo build -p skirnir --features gui`.
**`--workspace`/`--all` ignore `default-members`** and will try to build `firmware` on the host (which fails) — pass
`--exclude firmware` with those flags, or build `firmware` on its own from within `crates/firmware` (see below).

**Firmware (ESP32-S3 / Xtensa LX7) requires a separate toolchain.** The Xtensa ISA is not supported by upstream Rust
— install Espressif's fork before building/flashing firmware:

```sh
cargo install espup
espup install
source $HOME/export-esp.sh    # source in every shell before firmware builds (and in CI)
```

The firmware target (`xtensa-esp32s3-none-elf`), the `espflash flash --monitor --partition-table partitions.csv`
runner, and `build-std = ["core"]` live in `crates/firmware/.cargo/config.toml`; the `esp` toolchain channel is pinned
in `crates/firmware/rust-toolchain.toml`. Because both are crate-scoped, firmware **must be built from inside
`crates/firmware`** — a root `cargo build -p firmware` falls back to the host stock toolchain and fails. The `justfile`
wraps all of this (it sources the env and `cd`s for you):

```sh
just build        # cargo build for Xtensa; pass extra args, e.g. `just build --release` / `--features defmt`
just flash        # build + flash + serial monitor (espflash)
just monitor      # attach the serial monitor only (e.g. `just monitor --port /dev/ttyACM0`)
```

> **Pin `esp-bootloader-esp-idf = "=0.4.0"` — do NOT use 0.5.0 with esp-hal 1.0.** esp-hal 1.0.0's linker
> scripts reserve and KEEP the app descriptor at the FRONT of the DROM segment under the section name
> `.rodata_desc` (`ld/sections/rodata.x`; `ld/esp32s3/esp32s3.x`: `. = . + SIZEOF(.rodata_desc);`).
> esp-bootloader-esp-idf 0.5.0 renamed that section to `.flash.appdesc` (CHANGELOG #4745), so the descriptor
> falls to the END of `.rodata`; the bootloader reads the first 256 bytes of `.rodata` at flash `0x10020` as
> the descriptor and rejects the image with `Image requires efuse blk rev >= v116.31` → boot loop. 0.4.0 still
> emits `.rodata_desc`. (esp-hal 1.0.0 does NOT depend on esp-bootloader-esp-idf — the firmware pulls it
> directly.) Verify without hardware via `espflash save-image --merge` + checking magic `0xabcd5432` /
> `min_efuse=0` at flash `0x10020`. Revisit (allow 0.5.0+) only when esp-hal moves to a `.flash.appdesc`
> linker script (the esp-rtos 0.3 / esp-hal 1.1+ bump).
>
> **NOTE:** the espflash version was a red herring. The `v116.31` value is a *constant* across flashes and
> across espflash 4.3.0/4.4.0 — SHA-256 bleed would vary, so it was always fixed `.rodata` bytes, never
> espflash. The `_espflash-ok` guard in the justfile is now harmless but no longer the real fix.

Pure-logic library code is `no_std` but has **no esp-hal dependency**, so it compiles and unit-tests on the host with
stock Rust — keep it that way (see "Hardware abstraction" below).

## Architecture essentials (firmware)

- **Dual-core split.** Core 1 (`APP_CPU`) runs *only* the `motion_executor` task on a high-priority `InterruptExecutor`
  for real-time step generation — uncontested CPU, preempts nothing. Core 0 (`PRO_CPU`) runs everything else (USB
  comms, GCode parser, planner, TMC manager, spindle, status reporter) on a thread-mode executor. The `$H` homing
  cycle and G38 probing are *commanded/gated* on core 0 (`handle_home` in `comms`) but their seek/locate **motion runs
  on the core-1 executor** — it owns the RMT channels and limit inputs — dispatched via `HOME_REQUEST`/`PROBE_REQUEST`
  signals and answered with `HOME_RESULT`/`PROBE_RESULT`. The executor also samples the limit inputs (block
  boundaries, a 50 ms idle ticker, the debounced edge wait) and publishes their levels to the core-0 status reporter
  via a `LIMIT_LEVELS` atomic that sources the `Pn:` field.
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
- The `<...>` status report carries asserted input pins in the grblHAL `Pn:` field (`X`/`Y`/`Z` limit switches, `P`
  probe), and is omitted entirely when nothing is asserted. The firmware sources the limit letters from `LIMIT_LEVELS`
  (logical state after `$5` invert); `skirnir` decodes `Pn:` into a typed pin-state and shows the X/Y/Z endstops.

## Hardware abstraction & code style (from DOC-09)

- All hardware access goes behind traits so planner/parser/driver/homing logic is host-testable with recording/mock
  impls; only the firmware wiring layer touches esp-hal. `StepSink`, `TmcBus`, `ProbeInput`, and `DigitalIn` (limit
  inputs) are implemented; `PwmSink`/`DigitalOut` await the spindle/coolant wiring (DOC-07).
- **No `unwrap()`/`expect()` in library code** — propagate via `Result`. `expect` is allowed only in `main`/init paths
  where failure is genuinely unrecoverable.
- **Two-space indentation** (enforced by `.editorconfig`), LF line endings, final newline.
- Short single-responsibility functions; `///` doc comments on all public APIs; comment lines target ~120 chars and end
  with a period.
- `#![deny(unsafe_code)]` in library crates (firmware may use `unsafe` only at esp-hal boundaries); `#![deny(warnings)]`
  in CI.
