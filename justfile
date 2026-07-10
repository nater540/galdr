# https://just.systems

default:
  @just --list

# Firmware recipes target the ESP32-S3 (Xtensa). They `cd` into crates/firmware so its
# rust-toolchain.toml / .cargo/config.toml apply (a plain `cargo build -p firmware` from the repo
# root falls back to the host toolchain and fails), and source the Espressif toolchain env first.
# Pass extra cargo args through, e.g. `just build --release` or `just flash --features defmt`.

# Build the ESP32-S3 firmware (Xtensa).
build *args: _esp-env
  #!/usr/bin/env bash
  set -eo pipefail
  source "$HOME/export-esp.sh"
  cd crates/firmware
  cargo build {{args}}

# Build, flash, and monitor the firmware over USB (espflash).
flash *args: _esp-env _espflash-ok
  #!/usr/bin/env bash
  set -eo pipefail
  source "$HOME/export-esp.sh"
  cd crates/firmware
  cargo run {{args}}

# Attach the serial monitor to a running board without rebuilding (e.g. `just monitor --port /dev/ttyACM0`).
monitor *args: _esp-env _espflash-ok
  #!/usr/bin/env bash
  set -eo pipefail
  source "$HOME/export-esp.sh"
  espflash monitor {{args}}

# Run the skirnir GCode sender (native host app). Builds on the stock host toolchain, no esp env needed.
# The `gui` feature is default-on, so a bare `just run` boots the egui window; pass extra cargo args
# through, e.g. `just run --release` or `just run -- --cli` for the headless smoke path.
run *args:
  cargo run -p skirnir {{args}}

# Oscilloscope (Rigol DHO804) bench tooling — a uv-managed Python project in tools/scope. Drives the
# scope over USB-TMC/LAN for screenshots, waveform export, and the single-shot streaming-lockup catcher.
# `just scope` alone prints *IDN? (link check); pass a subcommand + args, all forwarded through, e.g.
# `just scope screenshot -o step.png`, `just scope waveform -c 1 -o step.csv`, `just scope catch-lockup -c 1`.
scope *args:
  uv run --project tools/scope scope-capture {{args}}

# Sync the scope tool's Python environment (first-time setup, or after editing its dependencies).
# Needs libusb on the host for the USB backend (macOS: `brew install libusb`).
scope-sync:
  uv sync --project tools/scope

# Private helper: fail early with a clear message if the Espressif toolchain env is not installed.
_esp-env:
  #!/usr/bin/env bash
  if [ ! -f "$HOME/export-esp.sh" ]; then
    echo "error: $HOME/export-esp.sh not found — install the Xtensa toolchain first:" >&2
    echo "         cargo install espup && espup install   (see CLAUDE.md)" >&2
    exit 1
  fi

# Private helper: warn if espflash is a version with the known app-descriptor regression. espflash >= 4.4.0
# misaligns the ELF SHA-256 it writes into the esp-bootloader-esp-idf 0.5.0 app descriptor, corrupting the
# adjacent `min_efuse_blk_rev_full` field; the v5.5 bundled bootloader then rejects the image with
# "Image requires efuse blk rev >= v116.31" and the board boot-loops. Pin to 4.3.0 with this esp-hal 1.0 stack.
_espflash-ok:
  #!/usr/bin/env bash
  ver=$(espflash --version 2>/dev/null | awk '{print $2}')
  major=${ver%%.*}; rest=${ver#*.}; minor=${rest%%.*}
  if [ "${major:-0}" -gt 4 ] 2>/dev/null || { [ "${major:-0}" -eq 4 ] && [ "${minor:-0}" -ge 4 ]; } 2>/dev/null; then
    echo "warning: espflash ${ver} has a known app-descriptor regression for the esp-hal 1.0 stack —" >&2
    echo "         the flashed image boot-loops on 'efuse blk rev v116.31'. Pin it back with:" >&2
    echo "         cargo install espflash --version 4.3.0 --locked" >&2
  fi

