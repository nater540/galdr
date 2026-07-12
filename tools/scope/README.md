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
just scope catch-lockup -c 1 --timeout 0.010 --mdepth 25M  # arm the wedge recorder, then stream a job
```

Or invoke it directly without just:

```sh
uv run --project tools/scope scope-capture --help
```

Connect the scope's **USB-B (device)** port to this PC, or drive it over the LAN jack with
`--resource TCPIP::<ip>::INSTR`.

## Testing & tuning the lockup catcher

`catch-lockup` arms a single-shot **Timeout trigger** with the `RFAL` (rise-or-fall) slope: it fires
when STEP shows **no edge in either direction** for longer than `--timeout`. A healthy stream keeps
re-arming it; a wedge (pulses stop, line frozen high *or* low) lets it elapse and fire, freezing deep
memory around the last pulse and dumping a screenshot + CSV. The tool only talks to the **scope** — it
never touches the firmware or serial, so it observes a wedge without interfering (consistent with the
no-silent-recovery decision).

**The dial that matters — `--timeout`.** Set it a little longer than the longest legitimate gap between
STEP edges at your *slowest* feed. Too short → false-fires on slow moves / `G4` dwell / block gaps; too
long → still catches a true wedge, just declares it later. The default 2 ms suits fast motion; real jobs
usually want 10–50 ms.

**Scope-prep flags (optional).** `--mdepth` (e.g. `25M`) and `--tscale` (seconds/div) set the capture
window so the last good pulses land in the pre-trigger buffer; `--level` sets the edge-detect threshold
(default 1.6 V for 3.3 V logic). Omit them to keep the scope's current setup.

### Phase 1 — prove the mechanism without the CNC (~2 min, no board)

Uses the scope's own 1 kHz probe-comp signal as a STEP stand-in:

1. Clip the probe to the comp pad (10×), press **Auto** so CH1 shows the 1 kHz square wave.
2. `just scope catch-lockup -c 1 --timeout 0.002 --wait-limit 30`
3. With the live signal (edges every 0.5 ms, well under the 2 ms timeout) it should sit in `WAIT` and
   **not** fire — proving it ignores healthy activity.
4. Lift the probe off the pad. The line goes idle → no edges > 2 ms → it **fires and dumps** the PNG + CSV.

### Phase 2 — catch a real wedge (on the board; motors not required)

The ESP32-S3's RMT drives the STEP pins with or without steppers attached, so this works on the bare board:

1. Probe **STEP** (GPIO1 = X); optionally **DIR** (GPIO5) and the **TMC UART** (GPIO9) on other channels
   for context.
2. `just scope catch-lockup -c 1 --timeout 0.010 --mdepth 25M --wait-limit 0`  (`0` = wait forever)
3. Stream a wedge-prone job. On a wedge it fires; inspect whether STEP died cleanly or mid-pulse, whether
   DIR was stable, and whether the UART was still moving at that instant.

## Notes / bench-gated caveats

- **Verified against DHO804 firmware 00.01.05** (2026-07): the USB-TMC link, the screenshot
  (`:DISP:DATA? PNG` returns a 1024×600 PNG), the chunked RAW waveform export + byte→volts scaling
  (dt and levels check out), and Timeout-trigger arming/firing/dump were all confirmed on real
  hardware. Every SCPI command is still followed by a `:SYSTem:ERRor?` check, so any future firmware
  that rejects an argument prints loudly rather than failing silently.
- The `catch-lockup` slope defaults to **`RFAL`** (rise-or-fall): it fires when STEP shows no edge in
  *either* direction for the timeout, so it catches a wedge whether the line froze high (mid-pulse) or
  low (idle). This firmware's accepted slopes are `POS` / `NEG` / `RFAL`; `EITHer` is rejected (-222).
