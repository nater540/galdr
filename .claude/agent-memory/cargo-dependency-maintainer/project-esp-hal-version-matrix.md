---
name: esp-hal-version-matrix
description: Definitive compatible version pins for the Galdr firmware ESP32-S3 stack — current (esp-hal 1.0.0 + esp-rtos 0.2.0) and esp-hal 1.1.0 upgrade path
metadata:
  type: project
---

## Current locked stack (as of 2026-06-23)

| Crate | Pin | Features | Notes |
|---|---|---|---|
| esp-hal | =1.0.0 | esp32s3, unstable | Exact-pinned |
| esp-rtos | =0.2.0 | esp32s3, embassy | Exact-pinned; the Embassy runtime host (replaces retired esp-hal-embassy) |
| embassy-executor | 0.9.1 | executor-interrupt, executor-thread | |
| embassy-time | 0.5.0 | — | |
| embassy-sync | 0.7 | — | |
| embassy-futures | 0.1 | — | |
| embassy-embedded-hal | =0.5.0 | — | Exact-pinned; 0.6 pulls embassy-hal-internal (new dep) |
| esp-backtrace | 0.18.1 | esp32s3, panic-handler, println | |
| esp-println | 0.16.1 | esp32s3 | |
| esp-storage | 0.8.1 | esp32s3 | multicore_auto_park() used in main.rs |
| esp-bootloader-esp-idf | =0.4.0 | esp32s3 | Exact-pinned for linker section compatibility (see below) |
| sequential-storage | 7.2.0 | heapless-09 | |
| heapless | 0.9 | — | |

## The bootloader linker pin (CRITICAL — keep this in memory)

esp-hal 1.0.0 linker script (`ld/sections/rodata.x`) names the app descriptor section `.rodata_desc`.
esp-hal 1.1.0 linker script (at tag `esp-hal-v1.1.0`) names it `.flash.appdesc`.
esp-bootloader-esp-idf 0.4.0 emits `.rodata_desc`. 0.5.0 (released 2026-04-16) emits `.flash.appdesc` (PR #4745).

Consequence: esp-hal 1.0.0 REQUIRES bootloader =0.4.0; esp-hal 1.1.0 REQUIRES bootloader 0.5.0.
This is the linchpin: upgrading esp-hal to 1.1 unblocks the bootloader from 0.4.0 to 0.5.0, and the two MUST move together.

**How:** `esp_app_desc!()` macro in `main.rs` (from esp-bootloader-esp-idf) emits the descriptor
into the named section. If the section name mismatches the linker's KEEP(), the descriptor
doesn't land at the front of DROM and the bootloader rejects the image with "efuse blk rev v116.31".

## esp-hal 1.1.0 upgrade path (2026-04-24 release)

### crates that move to:
| Crate | Old | New | Notes |
|---|---|---|---|
| esp-hal | =1.0.0 | =1.1.0 | Must move with esp-rtos |
| esp-rtos | =0.2.0 | =0.3.0 | Released 2026-04-16; requires esp-hal ~1.1.0 |
| esp-bootloader-esp-idf | =0.4.0 | =0.5.0 | REQUIRED with esp-hal 1.1 (.flash.appdesc) |
| esp-backtrace | 0.18.1 | 0.19.0 | Released 2026-04-16; no breaking changes |
| esp-println | 0.16.1 | 0.17.0 | Released 2026-04-16; no API breaking changes |
| esp-storage | 0.8.1 | 0.9.0 | Released 2026-04-16; adds C5/C61; check API |
| embassy-executor | 0.9.1 | 0.10.0 | Required by esp-rtos 0.3.0; arch→platform feature rename |
| embassy-sync | 0.7 | 0.8 | Required by esp-rtos 0.3.0 / esp-hal 1.1 |
| embassy-embedded-hal | =0.5.0 | 0.6.0 | esp-hal 1.1 uses 0.6; no breaking changes to BlockingAsync |

### embassy-time 0.5 stays compatible with esp-rtos 0.3.0 (it uses embassy-time-driver 0.2, not embassy-time directly).

## Breaking API changes for Galdr code in esp-hal 1.1.0

### RMT (motion.rs — HIGH EFFORT)
- `SingleShotTxTransaction` renamed to `TxTransaction`
- `ChannelCreator::configure_tx` no longer takes a pin; use `Channel::with_pin` instead
- `ChannelCreator::configure_tx` / `configure_rx` now take config by reference
- CRITICAL: Some errors now returned by `TxTransaction::wait()` instead of `Channel::transmit()`
  → The channel-loss-on-transmit-error path in motion.rs (which has a special comment about the
    error not handing back the channel) needs to be revisited — 1.1 DOES hand back the channel on error.
- `Into<PulseCode>` / `From<PulseCode>` removed from Tx/Rx methods
- `TxChannelConfig` is now applied via `Channel::apply_config`

### esp-rtos 0.3.0 (main.rs — HIGH EFFORT)
- `esp_rtos::start` now takes `SoftwareInterrupt<'static, 0>` as a parameter (PR #4459)
  → main.rs: `esp_rtos::start(timg0.timer0)` must change to pass sw_int.software_interrupt0
- `esp_rtos::start_second_core` NO LONGER takes `SoftwareInterrupt<'static, 0>` (PR #4459)
  → main.rs: `esp_rtos::start_second_core(cpu_ctrl, sw_int.software_interrupt0, sw_int.software_interrupt1, ...)` must drop the SWI0 argument
- SWI 0 consumed by `start` (not `start_second_core` anymore) — SWI 1 consumed by `start_second_core`
  → The firmware currently claims SWI 2 for MOTION_EXECUTOR_PRIORITY (`InterruptExecutor::<2>`)
    and SWI 0/1 for the esp-rtos SMP scheduler. The new allocation may shift which SWI is free for the motion executor.

### embassy-executor 0.10.0 (main.rs)
- arch-* features renamed to platform-*: `executor-interrupt` / `executor-thread` — check if these rename

### LEDC (spindle.rs — LOW EFFORT)
- Output pin type parameter removed from channel config; verify `channel::config::Config` still accepts `drive_mode`

### embassy-embedded-hal 0.5→0.6 (storage.rs — MINIMAL)
- BlockingAsync still exists; no breaking changes documented to that adapter
- embassy-sync bumped to 0.8 inside it, but if using CriticalSectionRawMutex directly that's fine

## Crates that do NOT need changes
- embedded-storage 0.3 — stable API
- embedded-io-async 0.7 — no change
- heapless 0.9 — no change
- libm 0.2 — no change
- static_cell 2 — no change
- defmt 1 — no change
- sequential-storage 7.2 — check if it has embassy-embedded-hal dep

## Verification without hardware
`espflash save-image --merge` + check magic `0xabcd5432` / `min_efuse=0` at flash offset `0x10020`.
This validates the app descriptor is at the correct location (DROM front) regardless of hardware.

## esp-hal-embassy — retired, do not use
esp-hal-embassy is fully retired. The crate directory no longer exists in the esp-rs/esp-hal repo.
Galdr firmware correctly uses esp-rtos (as of its current Cargo.toml).
