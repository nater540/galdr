# galdr-scope

A small [uv](https://docs.astral.sh/uv/)-managed Python tool for driving a **Rigol DHO804**
oscilloscope over USB-TMC / LAN (VISA) during Galdr bench work. It is the companion to the
"Drive it from your PC" section of the DHO804 stepper tutorial.

## What it does

| Subcommand | Purpose |
|------------|---------|
| *(none)* | List VISA resources and print `*IDN?` — a quick "is it talking?" link check. |
| `screenshot` | Save the live screen to a PNG (real captures of your STEP/DIR traces). |
| `waveform` | Export one channel to CSV as `(time_s, volts)`, deep-memory aware (chunked, up to 25 Mpts). |
| `catch-lockup` | Arm a single-shot **Timeout** trigger on STEP; when the pulse train flatlines (a streaming wedge), freeze the buffer and dump the screen + waveform. |

## Setup

One-time host prerequisite for the USB backend (macOS):

```sh
brew install libusb
```

Then sync the Python environment (uv creates a local `.venv` and resolves deps):

```sh
just scope-sync         # == uv sync --project tools/scope
```

## Use

Run everything through the justfile recipe (args pass straight through):

```sh
just scope                                   # *IDN? link check
just scope screenshot -o step.png            # save the screen
just scope waveform -c 1 -o step.csv         # dump CH1 full memory to CSV
just scope catch-lockup -c 1 --timeout 0.002 # arm the wedge recorder, then stream a job
```

Or invoke it directly without just:

```sh
uv run --project tools/scope scope-capture --help
```

Connect the scope's **USB-B (device)** port to this PC, or drive it over the LAN jack with
`--resource TCPIP::<ip>::INSTR`.

## Notes / bench-gated caveats

- **Verified against DHO804 firmware 00.01.05** (2026-07): the USB-TMC link, the screenshot
  (`:DISP:DATA? PNG` returns a 1024×600 PNG), the chunked RAW waveform export + byte→volts scaling
  (dt and levels check out), and Timeout-trigger arming/firing/dump were all confirmed on real
  hardware. Every SCPI command is still followed by a `:SYSTem:ERRor?` check, so any future firmware
  that rejects an argument prints loudly rather than failing silently.
- The `catch-lockup` slope defaults to **`RFAL`** (rise-or-fall): it fires when STEP shows no edge in
  *either* direction for the timeout, so it catches a wedge whether the line froze high (mid-pulse) or
  low (idle). This firmware's accepted slopes are `POS` / `NEG` / `RFAL`; `EITHer` is rejected (-222).
