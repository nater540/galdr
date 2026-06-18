---
name: project-build-constraints
description: Galdr build/toolchain constraints — Xtensa via espup, workspace target-mixing pitfall, edition 2024, host-testable no_std libs.
metadata:
  type: project
---

Firmware target is `xtensa-esp32s3-none-elf`, which upstream Rust/LLVM does not support — requires Espressif's fork
via `espup install` + `source $HOME/export-esp.sh` in every shell (and CI). `runner = "espflash flash --monitor"`.

**Why:** Xtensa LX7 ISA is unsupported by stock Rust. The host stock toolchain here is rustc 1.96.0 (2026-05-25).

**How to apply:**
- Pure-logic library code is `no_std` but must have NO esp-hal dependency so it compiles + unit-tests on the host with
  stock Rust. Keep it that way — hardware goes behind traits (`StepSink`, `PwmSink`, `DigitalIn/Out`, `TmcBus`).
- Editions are **2024** in actual Cargo.toml (docs say 2021 — actual wins).
- **Workspace target-mixing pitfall (RESOLVED 2026-06-16):** a single `.cargo/config.toml` with
  `[build] target = "xtensa-..."` at repo root would force the whole workspace (incl. native `skirnir`) onto Xtensa.
  RESOLVED mechanism (see [[project-dependency-pins]]): firmware EXCLUDED from `default-members` + firmware-local
  `crates/firmware/.cargo/config.toml` setting the Xtensa target. `package.forced-target` was rejected: it is
  nightly-only (`per-package-target`) and its mere presence breaks manifest parse on the stable host toolchain.
  Build firmware via `cd crates/firmware && cargo build` (the root cwd does not pick up the firmware-local config).
- esp-hal `unstable` feature is mandatory (RMT, LEDC, USB Serial/JTAG, CpuControl all gated). Pin versions in
  Cargo.lock; embassy-executor/embassy-time drift vs esp-hal-embassy is the common break — verify with `cargo tree`.
- `#![deny(unsafe_code)]` in library crates; firmware may use `unsafe` only at esp-hal boundaries. `#![deny(warnings)]`
  in CI. No `unwrap`/`expect` in library code; `expect` allowed only in main/init unrecoverable paths.

See [[project-galdr-overview]] and [[project-hardware-map]].
