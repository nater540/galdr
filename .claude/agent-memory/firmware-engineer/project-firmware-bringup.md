---
name: project-firmware-bringup
description: Galdr firmware-bin Xtensa bring-up mechanics — esp-rtos/esp-hal 1.0 entry point, USB Serial/JTAG, linker scripts, and the rust-toolchain pin that make `cargo build` work.
metadata:
  type: project
---

The firmware binary (`crates/firmware`) first compiled+linked for Xtensa on 2026-06-16 (comms phase,
DOC-08). Non-obvious bring-up facts a future author needs (the DOC sketches are WRONG here — they
assume the retired esp-hal-embassy API; the real path is esp-rtos 0.2):

**Entry point is `#[esp_rtos::main] async fn main(spawner: Spawner)`.** The macro expands to
`#[esp_hal::main] fn main() -> !` that builds a core-0 thread-mode `esp_rtos::embassy::Executor` and
runs the async body as its first task. The macro does NOT start the scheduler — YOU must call
`esp_rtos::start(timg0.timer0)` at the top of `main` before any await (on Xtensa `start` takes ONLY
the timer; the RISC-V `int0`/SoftwareInterrupt<0> arg is `#[cfg(riscv)]` and absent on S3). Order:
`esp_hal::init(Config::default().with_cpu_clock(CpuClock::max()))` → `TimerGroup::new(TIMG0)` →
`esp_rtos::start(timg0.timer0)` → peripherals → `spawner.must_spawn(...)`. This corrects the DOC-01
`CpuControl::start_app_core` + `InterruptExecutor::new(sw_int.software_interrupt1)` sketch (that is the
old esp-hal-embassy shape). Second core (motion, NOT yet wired) uses
`esp_rtos::start_second_core::<STACK>(swint1, || {...})` — different signature than DOC-01 shows.

**USB Serial/JTAG (the grbl host link):** `UsbSerialJtag::new(peripherals.USB_DEVICE).into_async()`
then `.split() -> (UsbSerialJtagRx<'static, Async>, UsbSerialJtagTx<'static, Async>)`. esp-hal 1.0
impls BOTH `embedded_io_async_06` and `_07` Read/Write; firmware pins `embedded-io-async = "0.7"`, so
`use embedded_io_async::{Read, Write}` resolves to the `_07` impls. `Write` gives default
`write_all`/`flush`. FIFO is 64 bytes; read into a `[u8;64]`.

**LINKER SCRIPTS ARE MANDATORY (the gotcha that cost the most time).** esp-hal 1.0's build.rs emits
the chip linker scripts (memory/alias/esp32s3/hal-defaults + generated `device.x` interrupt-vector
symbol table) into OUT_DIR and adds it to the link search path, but the APP must reference the master
script. Without it the link fails with `undefined reference to USB_DEVICE / FROM_CPU_INTR* / DMA_* /
RSA / AES`. Fix lives in `crates/firmware/.cargo/config.toml`:
`[target.xtensa-esp32s3-none-elf] rustflags = ["-C", "link-arg=-Tlinkall.x"]`. `linkall.x` INCLUDEs
the rest. esp-rtos 0.2 / esp-println 0.16 need no extra script beyond linkall.

**Toolchain selection:** `crates/firmware/rust-toolchain.toml` pins `channel = "esp"` so a plain
`cargo build` from within the crate picks the Espressif fork (which carries the Xtensa rust-src that
`build-std=["core"]` needs). WITHOUT this file cargo used the host stock toolchain and failed with
`can't find crate for core` + `'esp32s3' is not a recognized processor` (upstream LLVM has no Xtensa).
The file is scoped to crates/firmware (rustup walks up from cwd, nearest wins) so root `cargo build`/
`cargo test` stay on `stable-aarch64-apple-darwin` for firmware-core/skirnir — VERIFIED both ways.
Still must `source $HOME/export-esp.sh` first to put the Xtensa LLVM/gcc on PATH.

**Exact build command that succeeds (default + defmt):**
`cd crates/firmware && source $HOME/export-esp.sh && cargo build` and `... cargo build --features defmt`.
Both GREEN 2026-06-16; `cargo clippy` clean. The benign `'esp32s3' is not a recognized processor`
warning from the bundled GNU `ld`/gcc is noise — filter with `grep -v`; it does not affect the link.

See [[project-dependency-pins]], [[project-build-constraints]], [[project-protocol-contracts]].
