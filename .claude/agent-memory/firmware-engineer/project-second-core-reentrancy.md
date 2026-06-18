---
name: project-second-core-reentrancy
description: Galdr firmware second-core (APP_CPU) bring-up — the true root cause was an Xtensa over-top stack spill, fixed by the AppCoreStackArena trailing ABI headroom; the replay/guard/re-arm/deferral workarounds were all disproven and REMOVED.
metadata:
  type: project
---

**CONFIRMED ON HARDWARE + CLEANED UP (2026-06-17).** The Galdr core-1 motion bring-up bug is fixed and verified on
hardware: `G0 X5` runs end-to-end and returns `<Idle|WPos:5.000,0.000,0.000|...>` with the full healthy trace (block
popped → run_block → emit_burst/transmit/wait across all 3 axes → run_block returned → queue empty).

**TRUE ROOT CAUSE: Xtensa windowed-ABI OVER-TOP register spill (NOT a replay, NOT depth overflow).** `Stack<SIZE>` is
`{ mem: MaybeUninit<[u8;SIZE]> }` with NO trailing field; `Stack::top()` = `bottom + SIZE` bytes = one-past-the-end.
The app-core boot vector `start_core1_init` sets `SP = top()` and esp-hal reserves NO headroom above it. On Xtensa's
windowed ABI the FIRST core-1 frame spills the caller's 16-byte register save area into `[SP, SP+16)` = 16 bytes ABOVE
the array, onto whatever the linker placed next. With a bare `Stack` that was the adjacent `.bss` control statics
(`MOTION_EXECUTOR`/the old guard/progress statics), silently corrupting them and breaking bring-up. Size-independent
(8K and 32K corrupted identically) → it is NOT depth overflow. esp-hal's bottom stack guard can't see it (SP grows
DOWN; the guard only fires on a full-depth underflow, never on over-top neighbors).

**THE FIX (the ONLY thing that matters now) — in `crates/firmware/src/main.rs`:** `#[repr(C)] struct AppCoreStackArena
{ stack: Stack<16K>, _abi_headroom: [u8; 64] }`, parked in `static APP_CORE_STACK: StaticCell<AppCoreStackArena>`.
`#[repr(C)]` guarantees `_abi_headroom` is laid out IMMEDIATELY after `stack`, so `top()` == `&_abi_headroom` and the
over-top spill lands in that sacrificial padding, never an unrelated static. `AppCoreStackArena::touch_headroom()` (one
volatile read at init, `#[used]`-equivalent keep-alive) stops the linker eliminating the field. Pass `&mut arena.stack`
to `start_second_core`. The headroom is LOAD-BEARING; a tight regression comment on `AppCoreStackArena` documents the
mechanism so nobody "optimizes away" the padding. Enlarging the stack does NOT fix this (only slides neighbors up in
lockstep). Verified `nm -S -n`: `APP_CORE_STACK@0x3fc8ee70` size `0x4050` (16384 + 64 + 16 align); top = base+0x4000 =
`0x3fc92e70`; `_abi_headroom` = `[0x3fc92e70,0x3fc92eb0)` absorbs the spill `[0x3fc92e70,0x3fc92e80)`; `MOTION_EXECUTOR`
pushed to `0x3fc92ec0` (top+0x50, no longer the 16-B neighbor); `FLASH 0x3fc92ecc`/`STEP_SINK 0x3fc92f8c` clear.

**CLEANUP DONE (2026-06-17) — the entire replay/double-entry/garbage narrative was a FALSE model (the "two entries"
were one real entry reading corrupted statics) and ALL its scaffolding has been REMOVED from `main.rs`:**
  * `SECOND_CORE_STARTED: AtomicBool` one-shot guard + its `compare_exchange` branch — GONE.
  * `rearm_motion_swi()` + the per-entry SWI2 re-arm calls — GONE. `MOTION_SWI_INTERRUPT` const — GONE.
    `MOTION_SWI_PRIORITY` renamed to `MOTION_EXECUTOR_PRIORITY` (sole remaining use: `executor.start(...)`).
  * The spawn-deferral `interrupt::disable(...)`/re-enable bracket around `start`/`must_spawn` — GONE.
  * ALL diagnostics: `boottrace!` macro + every call, `MOTION_BRINGUP_PROGRESS` static + `BringupProgress` enum +
    store/load, `MOTION_CANARY` static, `log_core1_reset_reason()`, the core0-baseline / core1-raw-read logs, and the
    `#[allow(dead_code)]`s that only supported them — GONE. Unused imports (`Cpu`, `Interrupt`, `AtomicBool`,
    `AtomicU8`, `Ordering`) removed. Confirmed via `nm`: none of these symbols are in the ELF anymore.
  * The `Cpu::AppCpu` wrong-core gate — REMOVED (judgment call). With no guard to win, esp-rtos runs `func` on core 1
    exactly once; the gate guarded an impossible condition and, if it ever fired, would silently skip bring-up with no
    spawn — strictly worse than absent. Dead complexity, deleted.

**KEPT:** the `AppCoreStackArena` + `_abi_headroom` + `touch_headroom()` (the real fix, with the regression comment);
the motion-executor `mtrace!` defmt trace points in `motion.rs` INCLUDING the per-burst transmit/wait lines (kept
as-is — per-axis pairing is exactly what localizes an RMT TX-END deadlock; zero cost in the default build); the host
repro test `repro_runtime_g0_x5_rapid_low_speed_terminates` in firmware-core.

The `start_second_core` closure is now minimal: `MOTION_EXECUTOR.init(InterruptExecutor::new(sw_int.software_interrupt2))`
→ `executor.start(MOTION_EXECUTOR_PRIORITY)` → `must_spawn(motion_executor(...))`. Reads as "runs once on core 1; bring
up the motion executor." VERIFIED: `just build` + `just build --features defmt` clean under `#![deny(warnings)]`;
`cargo test -p firmware-core` 369 green incl. repro; `cargo clippy` (default + defmt, firmware + firmware-core) clean;
`nm -S -n` arena layout intact. This is the confirming-clean state — firmware boots and runs motion with no scaffolding.

Hard pins (unchanged): esp-hal `=1.0.0`, esp-rtos `0.2.0`, esp-bootloader-esp-idf `=0.4.0`; 100% Embassy async; core 1
dedicated to `motion_executor` on InterruptExecutor SWI2 @ Priority3.

See [[project-motion-executor]], [[project-firmware-bringup]], [[project-motion-executor-review-fixes]],
[[project-rmt-clock-and-tx-completion]].
