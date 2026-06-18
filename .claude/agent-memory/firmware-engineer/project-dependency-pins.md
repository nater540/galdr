---
name: project-dependency-pins
description: Galdr resolved dependency reality — esp-rtos hosts Embassy on stable esp-hal 1.0; vetted pin set; DOC-00 versions are wrong; host-build scoping mechanism.
metadata:
  type: project
---

The vetted, crates.io-resolvable dependency set for Galdr firmware as of 2026-06-16 (DOC-00's version
numbers are WRONG vs published crates — do not trust DOC-00 pins; use these).

**Runtime decision (corrects DOC-00):** The project is 100% Embassy, but on STABLE esp-hal 1.0 the
crate that hosts Embassy is `esp-rtos 0.2.0` with `features=["embassy"]` — NOT the retired
`esp-hal-embassy` (incompatible with stable esp-hal 1.0). esp-rtos is not a competing OS; with the
embassy feature it provides only the Embassy time-driver and `esp_rtos::embassy::Executor` /
`esp_rtos::embassy::InterruptExecutor<const SWI: u8>`. Second core via `esp_rtos::start_second_core()`
(takes `&'static mut Stack<N>`, hence `static_cell`). All task code stays ordinary Embassy.

**firmware bin pins (verified resolve):** esp-hal `=1.0.0` (features esp32s3, unstable), esp-rtos
`0.2.0` (esp32s3, embassy), embassy-executor `0.9.1` (executor-interrupt, executor-thread),
embassy-time `0.5`, embassy-sync `0.7`, embassy-futures `0.1`, esp-backtrace `0.18.1`
(esp32s3, panic-handler, println), esp-println `0.16.1` (esp32s3), esp-storage `0.8.1` (esp32s3),
embedded-storage `0.3`, embedded-io-async `0.7`, heapless `0.9`, libm `0.2`, defmt `1`,
static_cell `2`. dev-dep embedded-hal-mock `0.11`.

**firmware-core lib pins:** heapless `0.9`, libm `0.2`, defmt `1` (optional, behind `defmt` feature).
dev-dep embedded-hal-mock `0.11`. NO embedded-io-async (the TmcBus trait is sync; only add it if an
async trait actually lands). NO esp-hal — keep it host-buildable.

**Resolution gotchas observed in Cargo.lock:**
- Only ONE `defmt` (1.1.0) — no 0.3 conflict. Do NOT force-enable defmt on embassy-* crates or you
  risk pulling a conflicting defmt 0.3.
- THREE `embassy-sync` versions coexist (0.6.2, 0.7.2, 0.8.0): firmware's `0.7` -> 0.7.2; esp-rtos
  pulls 0.6.2 + 0.8.0 transitively. Fine because firmware-core has NO embassy-sync dep so no
  cross-crate type mismatch. WATCH: if firmware-core ever shares embassy-sync types across the trait
  boundary with firmware, the versions MUST be unified.
- embassy-time resolved to 0.5.1 (req `0.5`).

**Host-build scoping (CRITICAL, see [[project-build-constraints]]):** `package.forced-target` is the
ideal Xtensa pin but is NIGHTLY-ONLY (`per-package-target`) and its mere presence FAILS manifest
parsing on the stable host toolchain — so it cannot be used. The working mechanism on stable:
1. Root `Cargo.toml` lists firmware in `members` but EXCLUDES it from `default-members`
   (`default-members = ["crates/firmware-core", "crates/skirnir"]`). Root `cargo build`/`cargo test`
   then only compile host crates; esp-hal is never compiled for host. Verified GREEN.
2. `crates/firmware/.cargo/config.toml` sets `[build] target = "xtensa-esp32s3-none-elf"`,
   `runner = "espflash flash --monitor"`, and `[unstable] build-std = ["core","alloc"]`. This config
   only applies when cargo is invoked from WITHIN crates/firmware (cargo merges config up the cwd
   tree). So `cargo build -p firmware` from repo ROOT does NOT pick it up and tries the host —
   build firmware by `cd crates/firmware && cargo build` (after `source $HOME/export-esp.sh`).

See [[project-build-constraints]], [[project-hardware-map]], [[project-galdr-overview]].
