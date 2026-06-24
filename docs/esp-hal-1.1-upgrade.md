# esp-hal 1.0 → 1.1 Upgrade Plan (Galdr firmware)

Status: **planned, not executed.** This is a staged upgrade plan for moving the ESP32-S3 firmware
(`crates/firmware`) from esp-hal 1.0 to 1.1. It is grounded in the actual tagged linker scripts and
CHANGELOGs (sources at the bottom). Items marked *(inferred)* were not confirmed by an actual compile and
must be nailed down when the upgrade is attempted.

> **Recommendation up front.** Worth doing, but **time it to land the same sitting the board is back**, not
> blind. The two highest-risk changes (the esp-rtos `start`/`start_second_core` software-interrupt reshuffle
> in `main.rs`, and the RMT bring-up in `motion.rs`) can only be fully validated on hardware. The offline
> app-descriptor check (below) de-risks the *bootloader* side, but RMT pulse shape and the core-1 ABI canary
> need a live board run. There is **no urgent feature forcing it** — notably, **RMT DMA is NOT in 1.1**
> (`motion.rs`'s "no RMT DMA backend yet" note stays true), so the feature that would most help multi-axis
> step jitter is not the trigger; it will likely land in 1.2/1.3, and *that* is the natural trigger.

## 1. The gate: bootloader linker section (resolved in 1.1)

The long-standing `esp-bootloader-esp-idf = "=0.4.0"` pin (the "efuse blk rev v116.31" boot-loop) is lifted
by moving to esp-hal 1.1.0, because 1.1's linker script renames the app-descriptor section to match the
bootloader's new expectation:

| esp-hal | `ld/sections/rodata.x` keeps the descriptor under | bootloader pin required |
|---------|----------------------------------------------------|-------------------------|
| =1.0.0  | `.rodata_desc`                                      | `=0.4.0`                |
| =1.1.0  | `.flash.appdesc`                                    | `=0.5.0`                |

These pairs are **mutually exclusive** — the descriptor is misplaced (and the board boot-loops) if esp-hal
and the bootloader crate are mismatched in either direction. Therefore **esp-hal 1.1.0 and
esp-bootloader-esp-idf 0.5.0 must move in a single atomic commit.** (esp-bootloader-esp-idf 0.5.0 changed the
`ESP_APP_DESC` section to `.flash.appdesc` to align with esptool, PR #4745, released 2026-04-16.)

## 2. The atomic bump set (one commit)

These are coupled and must move together — esp-rtos 0.3.0 requires esp-hal `~1.1` and simultaneously changes
the `start`/`start_second_core` signatures, and the bootloader section name changes atomically with the
esp-hal linker script:

| Crate | Old | New | Why coupled |
|-------|-----|-----|-------------|
| `esp-hal` | `=1.0.0` | `=1.1.0` | core; everything depends on it |
| `esp-rtos` | `=0.2.0` | `=0.3.0` | requires esp-hal `~1.1`; reshuffles SWI/`start` |
| `esp-bootloader-esp-idf` | `=0.4.0` | `=0.5.0` | linker section name — atomic with esp-hal |
| `embassy-executor` | `0.9.1` | `0.10.0` | esp-rtos 0.3 pins it |
| `embassy-sync` | `0.7` | `0.8` | esp-hal 1.1 CHANGELOG (embassy-sync → 0.8) |
| `embassy-embedded-hal` | `=0.5.0` | `0.6.0` | esp-hal 1.1 CHANGELOG; **drop the exact pin** |

Companion crates (non-breaking for Galdr code, can ride in the same commit):

| Crate | Old | New | Note |
|-------|-----|-----|------|
| `esp-backtrace` | `0.18.1` | `0.19.0` | new chip support only |
| `esp-println` | `0.16.1` | `0.17.0` | no API break for the features in use |
| `esp-storage` | `0.8.1` | `0.9.0` | verify `multicore_auto_park()` survives *(inferred present)* |

`embassy-time` (0.5.x) and `embassy-futures` (0.1) do not need to change; esp-rtos 0.3 uses
`embassy-time-driver` 0.2 internally, not `embassy-time` directly.

## 3. Toolchain (Stage 0)

esp-hal 1.1.0 bumped MSRV to 1.88.0 (set during the rc.0 cycle). The Espressif Xtensa fork runs ahead of
upstream MSRV, but before touching manifests: `rustup update esp` inside `crates/firmware/`, then
`espup check`. No new espup major version is believed required *(inferred — not confirmed)*.

## 4. Breaking-change checklist, mapped to Galdr source (ranked by effort/risk)

### A. `esp-rtos::start` / `start_second_core` SWI reshuffle — `main.rs` — HIGH
esp-rtos 0.3.0 (PR #4459) redistributed software-interrupt allocation:
- `start(timg0.timer0)` → `start(timg0.timer0, sw_int.software_interrupt0)` — `sw_int` must be created
  **before** `start`.
- `start_second_core(CPU_CTRL, sw_int.software_interrupt0, sw_int.software_interrupt1, stack, …)` →
  **drops** the `software_interrupt0` argument.
- **Re-verify** which SWI `start_second_core` now claims, and that the motion `InterruptExecutor::<2>` is
  still the next free one. *(inferred: still SWI 2)*
- The `AppCoreStackArena` / ABI-headroom / canary logic is **unaffected** (it's an esp-hal `Stack` type,
  unchanged). See the [Xtensa stack-top ABI headroom] note — that hazard is orthogonal to this bump.

### B. RMT transmit error semantics — `motion.rs` `emit_burst` — HIGH/MED
esp-hal 1.1.0 (PR #4617): "some errors are now returned by `TxTransaction::wait()` instead of
`Channel::transmit`." This actually **fixes** the channel-loss-on-`transmit`-error deficiency the existing
`emit_burst` comment flags as a TODO — the `Err((_, channel))` path from `wait()` already restores the
channel. Verify the `match channel.transmit(...)` arm that returns `Err(StepError::Transport)` (and leaves
the slot `None`) is now unreachable / has different semantics.

### C. RMT channel creation API — `motion.rs` `init` — MED
esp-hal 1.1.0 (PR #4302):
- `SingleShotTxTransaction` → `TxTransaction` (the type returned by `channel.transmit()` and stored in
  `txns`).
- `ChannelCreator::configure_tx`/`configure_rx` **no longer take a pin** → use `Channel::with_pin`.
- config now passed **by reference**; `Channel::apply_config` added.
- `Into/From<PulseCode>` removed from Tx/Rx methods (firmware already uses `PulseCode` directly → likely
  no-op).

### D. `embassy-executor` 0.10 feature rename — `Cargo.toml` — LOW
PR renamed `arch-*` → `platform-*`. Galdr uses `executor-interrupt` / `executor-thread` (capability
features, not `arch-*`) → likely unchanged. Verify they still exist in 0.10.0.

### E. LEDC config struct — `spindle.rs` — LOW
1.1 removed the output-pin type parameter from LEDC and consolidated config. Verify the
`channel::config::Config { timer, duty_pct, drive_mode }` fields (esp. `drive_mode: DriveMode::PushPull`) and
that `set_duty_hw` is unchanged.

### F. GPIO `clone_unchecked` / `AnyPin` — `tmc.rs` — LOW (verify)
1.1: `GpioPin` gained a lifetime parameter with `clone_unchecked`/`reborrow`; "all GPIOs are now available
without unsafe code." The `unsafe { tx_line.clone_unchecked() }` on the TMC single-wire UART line may no
longer need `unsafe` / may have a changed signature.

### G. `esp-storage` 0.9 — `storage.rs` — VERIFY ONLY
`FlashStorage::new(peripherals.FLASH)` should be unchanged. Confirm `multicore_auto_park()` still exists
(no removal noted). The [esp-storage multicore park] fix depends on it.

### H. `embassy-embedded-hal` 0.6 `BlockingAsync` — `storage.rs` — VERY LOW
0.6.0 upgraded embassy-sync to 0.8 and added `Clone` for shared I2C; no `BlockingAsync` break. Drop the
`=0.5.0` exact pin.

### I. UART API — `tmc.rs` — VERIFY ONLY
1.1 made `read_ready`/`write_ready` take `&self` (now stable). Firmware uses `write`/`flush`/`read_buffered`
— not the changed methods. Low risk; verify it still compiles.

## 5. Verification without hardware (pre-flash gate)

1. Build from `crates/firmware/`: `source $HOME/export-esp.sh && just build`.
2. App-descriptor position check (the project's existing method):
   ```
   espflash save-image --merge --chip esp32s3 \
     target/xtensa-esp32s3-none-elf/debug/firmware firmware.bin
   ```
   Inspect flash offset `0x10020` (first 256 bytes of DROM after the 32-byte flash header):
   - bytes 0–3 = magic `0xABCD5432` (little-endian on disk: `32 54 CD AB`)
   - the `min_efuse_rev` field (offset `0x10020 + 12`, 2 bytes) must be `0x00 0x00`.
   A non-zero `min_efuse_rev` (e.g. `0x74 0x1F` = 116.31) means the descriptor is misplaced → wrong
   esp-hal/bootloader pairing.

### Genuinely board-only (cannot verify offline)
RMT step pulse shape/jitter (scope), USB CDC enumeration + banner, LEDC 0–10 V ramp on GPIO13, UART1 TMC2209
exchanges, limit-switch GPIO IRQs + homing timing, flash persistence round-trip, and the core-1 ABI canary
check in `main.rs`.

## 6. What 1.1 actually buys Galdr

- **Real correctness fix on the inter-core path:** PR #4706 — `SoftwareInterrupt::raise()` now takes effect
  before returning — directly affects the motion executor's `BLOCK_AVAILABLE` / `MOTION_RESET` signalling.
- RMT error semantics fix (item B) — resolves the acknowledged channel-loss TODO.
- ESP32-S3 `irom_seg`/`drom_seg` raised 4 MB → 32 MB (PR #5121) — flash-image headroom as the firmware
  grows.
- PR #4844 — fixes an ELF/segment case that could make the bootloader refuse to boot.
- Interrupt API (`esp_hal::interrupt::Priority`, …) stabilized; `AtomicWaker::wake` placed in IRAM.
- Permanently retires the fragile `esp-bootloader-esp-idf` pin.
- **Not** RMT DMA — not in 1.1.

## Sources
- esp-hal CHANGELOG: https://raw.githubusercontent.com/esp-rs/esp-hal/main/esp-hal/CHANGELOG.md
- esp-rtos CHANGELOG: https://raw.githubusercontent.com/esp-rs/esp-hal/main/esp-rtos/CHANGELOG.md
- esp-bootloader-esp-idf CHANGELOG: https://raw.githubusercontent.com/esp-rs/esp-hal/main/esp-bootloader-esp-idf/CHANGELOG.md
- esp-hal v1.0.0 rodata.x: https://raw.githubusercontent.com/esp-rs/esp-hal/esp-hal-v1.0.0/esp-hal/ld/sections/rodata.x
- esp-hal v1.1.0 rodata.x: https://raw.githubusercontent.com/esp-rs/esp-hal/esp-hal-v1.1.0/esp-hal/ld/sections/rodata.x
- esp-rtos Cargo.toml: https://raw.githubusercontent.com/esp-rs/esp-hal/main/esp-rtos/Cargo.toml
- embassy CHANGELOGs (sync / executor / embedded-hal): https://github.com/embassy-rs/embassy
- esp-hal releases: https://github.com/esp-rs/esp-hal/releases
