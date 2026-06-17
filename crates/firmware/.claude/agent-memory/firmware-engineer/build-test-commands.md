---
name: build-test-commands
description: Host test vs Xtensa firmware build commands for the Galdr firmware, with the must-run-from-inside gotcha
metadata:
  type: reference
---

- Host-test firmware-core (stock Rust): `cargo test -p firmware-core`. Must be warning-clean (`#![deny(warnings)]` in CI).
- Firmware Xtensa build: MUST run from INSIDE `crates/firmware` (NOT `-p firmware` from the workspace root):
  `cd crates/firmware && source $HOME/export-esp.sh && cargo build`. The Xtensa toolchain (Espressif fork) is
  required — `xtensa-esp32s3-none-elf` target, runner `espflash flash --monitor`.
- `firmware` is excluded from `default-members`, so bare `cargo build`/`cargo test` stay on the host toolchain.
  `--workspace`/`--all` ignore default-members and try to host-build firmware (fails) — pass `--exclude firmware`.
