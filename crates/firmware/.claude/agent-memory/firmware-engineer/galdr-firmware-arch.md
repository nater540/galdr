---
name: galdr-firmware-arch
description: Galdr ESP32-S3 grblHAL firmware crate layout, runtime, and key module responsibilities
metadata:
  type: project
---

Galdr is a CNC PCB-milling system. The firmware half targets ESP32-S3 (Xtensa LX7), `no_std` Rust on
esp-hal 1.0 + Embassy (hosted by `esp-rtos`, NOT esp-hal-embassy — see [[firmware-embassy-runtime]] in the
user's auto-memory). 100% Embassy async is a HARD requirement.

**Crate split** (under `crates/`):
- `firmware-core` — pure-logic, `#![no_std]`, `#![deny(unsafe_code)]`, NO esp-hal dep, host-testable with stock
  Rust. Modules: `gcode`, `protocol` (grblHAL streaming state machine + response formatters), `planner`,
  `motion` (segment generator / step math), `settings` (`$n` model, `$$`/`$x=val`, protobuf `$PBX` persist),
  `drivers::tmc2209`, `hal_traits`.
- `firmware` — the ONLY esp-hal crate; thin wiring. `main.rs` (boot/spawn), `comms.rs` (USB CDC streaming
  tasks), `motion.rs` (core-1 RMT step sink + executor), `storage.rs` (flash), `tmc.rs` (UART1 bus).
- `galdr-proto` — protobuf wire/flash DTO for settings.

**Dual-core split**: core 1 (APP_CPU) runs ONLY `motion_executor` on a high-prio InterruptExecutor (SWI 2,
Priority3); core 0 runs all comms/parser/planner/status tasks on the thread-mode executor.

**grblHAL protocol contract** (firmware ↔ skirnir, spec `docs/gcode-streaming.md`): exactly one `ok`/`error:N`
per consumed line; CRLF/LFCR = one terminator; banner on boot AND every soft reset (native USB can't be
host-hard-reset); persistent gcode error-hold after an error until reset/blank-line/`$`-command. Error codes:
`error:3` = unrecognized `$` statement, `error:5` = homing not enabled, `error:2` = bad value, `error:15` =
line overflow. `MachineState` enum (protocol.rs) carries Idle/Run/Hold(bool)/Jog/Alarm(u8)/Door/Check/Home/Sleep.

**Style** (CLAUDE.md, strictly enforced): 2-space indent, LF, final newline, `///` on public APIs, comments
~120 chars ending in a period, NO `unwrap`/`expect` in library code (`expect` only in main/init), `#![deny(warnings)]`.
