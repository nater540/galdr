"""Drive a Rigol DHO804 over USB-TMC / LAN (VISA) for Galdr bench work.

Three jobs, matching the tutorial's "Drive it from your PC" section:

  screenshot     Grab the live screen as a PNG — real captures of your STEP/DIR traces.
  waveform       Pull a channel's samples to CSV, scaled to seconds and volts (deep-memory aware).
  catch-lockup   Arm a single-shot Timeout trigger on STEP and wait for the pulse train to
                 flatline (the streaming-wedge recorder), then dump the frozen screen + buffer.

With no subcommand it lists VISA resources and prints *IDN? — a quick "is it talking?" check.

Run it via `just scope ...`, or directly with `uv run --project tools/scope scope-capture ...`.
Connect the scope's USB-B (device) port to this PC, or drive it over LAN with
--resource TCPIP::<ip>::INSTR. On macOS the pyusb backend needs libusb (`brew install libusb`).

SCPI command forms follow Rigol's DHO800/900 Programming Guide. A couple of forms are
model/firmware specific and are noted inline; every command is followed by an error-queue
check (:SYSTem:ERRor?) so a rejected argument is visible immediately, never silent.
"""

import argparse
import csv
import sys
import time
from pathlib import Path

# A single :WAVeform:DATA? read is capped by the instrument; 250 k samples/chunk is safely within
# the DHO800's per-query limit, so deep-memory (up to 25 Mpts) reads are looped in chunks.
CHUNK_POINTS = 250_000


def eprint(*args):
  """Print to stderr so status chatter never contaminates piped data on stdout."""
  print(*args, file=sys.stderr)


def open_scope(resource=None, timeout_ms=30_000):
  """Open a VISA session to the scope. Auto-picks the first USB-TMC resource if none is given."""
  import pyvisa

  # Force the pure-Python backend (pyvisa-py). Most bench Macs have no NI-VISA installed, and an
  # explicit '@py' stops pyvisa from selecting an absent IVI backend. Fall back to the default if needed.
  try:
    rm = pyvisa.ResourceManager("@py")
  except Exception:  # noqa: BLE001 — backend selection can fail oddly; let the default RM try instead.
    rm = pyvisa.ResourceManager()
  if resource is None:
    resources = rm.list_resources()
    usb = [r for r in resources if "USB" in r.upper()]
    if not usb:
      eprint("No USB VISA resource found. Available:", resources or "(none)")
      eprint("Plug in the scope's USB-B port, or pass --resource TCPIP::<ip>::INSTR.")
      sys.exit(2)
    resource = usb[0]
  eprint(f"Opening {resource} ...")
  scope = rm.open_resource(resource)
  scope.timeout = timeout_ms
  # Binary transfers of deep memory need a generous chunk size so pyvisa doesn't dribble reads.
  scope.chunk_size = 1 << 20
  return scope


def check_errors(scope, context=""):
  """Drain the SCPI error queue and warn on anything that isn't the '0,"No error"' sentinel."""
  seen = []
  for _ in range(20):
    raw = scope.query(":SYSTem:ERRor?").strip()
    seen.append(raw)
    if raw.startswith("0,") or raw.startswith("+0,"):
      return
  where = f" after {context}" if context else ""
  eprint(f"[scope error queue{where}] " + " | ".join(seen))


def cmd_idn(scope):
  """Confirm the link by printing the instrument identity string."""
  print(scope.query("*IDN?").strip())
  check_errors(scope, "*IDN?")


def cmd_screenshot(scope, path):
  """Save the live display to a PNG. Tries the explicit-format form first, then the bare query."""
  data = None
  last_exc = None
  # DHO800 firmware generally accepts ':DISP:DATA? PNG'; older/other builds return the default
  # bitmap for a bare ':DISP:DATA?'. Try both so this works across firmware without editing.
  for query in (":DISPlay:DATA? PNG", ":DISPlay:DATA?"):
    try:
      data = scope.query_binary_values(query, datatype="B", container=bytes)
      if data:
        break
    except Exception as exc:  # noqa: BLE001 — surface any transport/parse failure and try the fallback.
      last_exc = exc
  if not data:
    raise RuntimeError(f"screenshot query returned no data: {last_exc}")
  Path(path).write_bytes(data)
  check_errors(scope, "screenshot")
  eprint(f"Saved {len(data)} bytes -> {path}")


def read_preamble(scope):
  """Return the 10-field :WAVeform:PREamble? as a dict with the numeric fields converted."""
  fields = scope.query(":WAVeform:PREamble?").strip().split(",")
  keys = ("format", "type", "points", "count", "xincrement", "xorigin",
          "xreference", "yincrement", "yorigin", "yreference")
  pre = dict(zip(keys, fields))
  for k in ("xincrement", "xorigin", "yincrement", "yorigin", "yreference"):
    pre[k] = float(pre[k])
  for k in ("points", "count", "xreference"):
    pre[k] = int(float(pre[k]))
  return pre


def _total_points(scope, mode, preamble):
  """How many samples to read: full acquisition memory in RAW mode, else the on-screen count."""
  if mode.upper() != "RAW":
    return preamble["points"]
  depth = scope.query(":ACQuire:MDEPth?").strip()
  try:
    return int(float(depth))
  except ValueError:
    # :ACQ:MDEP? can report AUTO; fall back to what the preamble advertises.
    return preamble["points"]


def cmd_waveform(scope, channel, path, mode="RAW"):
  """Read one channel to CSV as (time_s, volts). RAW pulls full memory; NORMal pulls the screen."""
  # RAW (full-memory) reads require a stopped acquisition — otherwise the buffer moves under you.
  scope.write(":STOP")
  scope.write(f":WAVeform:SOURce CHANnel{channel}")
  scope.write(f":WAVeform:MODE {mode}")
  scope.write(":WAVeform:FORMat BYTE")
  check_errors(scope, "waveform setup")

  pre = read_preamble(scope)
  total = _total_points(scope, mode, pre)
  x_inc, x_org = pre["xincrement"], pre["xorigin"]
  y_inc, y_org, y_ref = pre["yincrement"], pre["yorigin"], pre["yreference"]
  eprint(f"Reading {total} points from CH{channel} ({mode}) ...")

  raw = bytearray()
  start = 1
  while start <= total:
    stop = min(start + CHUNK_POINTS - 1, total)
    scope.write(f":WAVeform:STARt {start}")
    scope.write(f":WAVeform:STOP {stop}")
    chunk = scope.query_binary_values(":WAVeform:DATA?", datatype="B", container=bytearray)
    if not chunk:
      eprint(f"Empty read at sample {start}; stopping early.")
      break
    raw.extend(chunk)
    eprint(f"  {min(stop, total):>10,} / {total:,}")
    start = stop + 1

  # Rigol byte->volts: volts = (raw - yorigin - yreference) * yincrement. Time is origin + i*inc.
  with open(path, "w", newline="") as fh:
    writer = csv.writer(fh)
    writer.writerow(("time_s", "volts"))
    for i, byte in enumerate(raw):
      volts = (byte - y_org - y_ref) * y_inc
      writer.writerow((f"{x_org + i * x_inc:.9e}", f"{volts:.6e}"))
  check_errors(scope, "waveform read")
  eprint(f"Wrote {len(raw):,} samples -> {path}")


def prep_acquisition(scope, mdepth=None, tscale=None):
  """Optionally set acquisition memory depth and horizontal time/div before arming.

  Deep memory widens the captured window held around the wedge; a slower time/div spans more real
  time inside it, so the last good pulses before the stall land in the pre-trigger buffer. Both
  default to the scope's current setup unless overridden here.
  """
  if mdepth is None and tscale is None:
    return
  if mdepth is not None:
    scope.write(f":ACQuire:MDEPth {mdepth}")  # e.g. 1M, 10M, 25M, AUTO (max depends on active channels)
  if tscale is not None:
    scope.write(f":TIMebase:MAIN:SCALe {tscale}")  # seconds/div
  check_errors(scope, "acquisition prep")
  eprint(f"Acquisition: mem depth {scope.query(':ACQuire:MDEPth?').strip()}, "
         f"{scope.query(':TIMebase:MAIN:SCALe?').strip()} s/div")


def _set_trigger_level(scope, level_v):
  """Set the timeout-trigger threshold (the HIGH/LOW split for a 3.3 V edge). Non-fatal: if this
  firmware lacks the command, warn and leave the level as-is rather than erroring on every run."""
  scope.write(f":TRIGger:TIMeout:LEVel {level_v}")
  err = scope.query(":SYSTem:ERRor?").strip()
  if not (err.startswith("0,") or err.startswith("+0,")):
    eprint(f"Note: ':TRIG:TIMeout:LEVel' was rejected ({err}); leaving the trigger level unchanged. "
           f"If RFAL doesn't detect edges, set the level to ~{level_v} V by hand (see the programming guide).")


def cmd_catch_lockup(scope, channel, timeout_s, png_path, csv_path, poll_s, wait_limit_s,
                     level_v=1.6, mdepth=None, tscale=None):
  """Arm a single-shot Timeout trigger on the STEP channel; on flatline, dump screen + waveform.

  The Timeout trigger fires when the source stops changing for longer than `timeout_s`. While a job
  streams, every STEP pulse re-arms the timer so it never fires; when the board wedges and STEP goes
  quiet, the timer elapses, the scope triggers, and deep memory freezes the window around the last edge.
  `mdepth`/`tscale` optionally prep the acquisition window; `level_v` sets the edge-detect threshold.
  """
  prep_acquisition(scope, mdepth, tscale)
  scope.write(":TRIGger:MODE TIMeout")
  scope.write(f":TRIGger:TIMeout:SOURce CHANnel{channel}")
  scope.write(f":TRIGger:TIMeout:TIMe {timeout_s}")
  _set_trigger_level(scope, level_v)
  # RFAL = fire when there's been no rising OR falling edge for `timeout_s` — a level-independent
  # dropout detector, so it catches a wedge whether STEP froze high (mid-pulse) or low (idle between
  # pulses). Verified on DHO804 fw 00.01.05: POS/NEG/RFAL are accepted, 'EITHer' is not (-222).
  scope.write(":TRIGger:TIMeout:SLOPe RFAL")
  scope.write(":TRIGger:SWEep SINGle")
  check_errors(scope, "timeout trigger setup")

  scope.write(":SINGle")  # arm one acquisition
  eprint(f"Armed single-shot Timeout trigger on CH{channel}, {timeout_s}s idle, level {level_v} V. "
         f"Stream your job now; waiting for STEP to flatline ...")

  waited = 0.0
  while True:
    status = scope.query(":TRIGger:STATus?").strip()
    if status == "STOP":
      eprint("Triggered — STEP went quiet. Freezing buffer.")
      break
    if wait_limit_s and waited >= wait_limit_s:
      eprint(f"Gave up after {wait_limit_s}s (status={status}). No lockup captured; "
             f"the run may have completed cleanly.")
      return
    time.sleep(poll_s)
    waited += poll_s

  cmd_screenshot(scope, png_path)
  cmd_waveform(scope, channel, csv_path)
  eprint("Lockup capture complete. Inspect the PNG for the last-pulse region; the CSV has the samples.")


def build_parser():
  p = argparse.ArgumentParser(prog="scope-capture",
                              description="Rigol DHO804 bench helper for Galdr STEP/DIR capture.")
  p.add_argument("--resource", help="VISA resource string (default: first USB device). "
                                     "e.g. TCPIP::192.168.1.50::INSTR")
  p.add_argument("--timeout-ms", type=int, default=30_000, help="VISA I/O timeout in ms (default 30000).")
  sub = p.add_subparsers(dest="cmd")

  sp = sub.add_parser("screenshot", help="Save the live screen to a PNG.")
  sp.add_argument("-o", "--out", default="scope_screen.png", help="Output PNG path.")

  wf = sub.add_parser("waveform", help="Export one channel to CSV (time_s, volts).")
  wf.add_argument("-c", "--channel", type=int, default=1, help="Channel number (default 1 = STEP).")
  wf.add_argument("-o", "--out", default="scope_waveform.csv", help="Output CSV path.")
  wf.add_argument("--mode", default="RAW", choices=("RAW", "NORMal"),
                  help="RAW = full acquisition memory; NORMal = on-screen points (default RAW).")

  cl = sub.add_parser("catch-lockup", help="Single-shot Timeout trigger; dump on STEP flatline.")
  cl.add_argument("-c", "--channel", type=int, default=1, help="STEP channel (default 1).")
  cl.add_argument("--timeout", type=float, default=0.002,
                  help="Idle time (s) with no STEP edge that counts as a wedge (default 0.002 = 2 ms).")
  cl.add_argument("--png", default="lockup_screen.png", help="Screenshot output path.")
  cl.add_argument("--csv", default="lockup_waveform.csv", help="Waveform CSV output path.")
  cl.add_argument("--poll", type=float, default=0.2, help="Trigger-status poll interval (s).")
  cl.add_argument("--wait-limit", type=float, default=0.0,
                  help="Give up after this many seconds (0 = wait forever).")
  cl.add_argument("--level", type=float, default=1.6,
                  help="Edge-detect threshold in volts (default 1.6 for 3.3 V logic).")
  cl.add_argument("--mdepth", default=None,
                  help="Set acquisition memory depth before arming, e.g. 1M, 10M, 25M, AUTO "
                       "(widens the captured window; default leaves the scope's setting).")
  cl.add_argument("--tscale", type=float, default=None,
                  help="Set horizontal time/div in seconds before arming, e.g. 0.001 "
                       "(default leaves the scope's setting).")
  return p


def main(argv=None):
  args = build_parser().parse_args(argv)
  try:
    scope = open_scope(args.resource, args.timeout_ms)
  except ImportError:
    eprint("pyvisa is not installed. Run `just scope-sync` (or `uv sync --project tools/scope`).")
    return 2

  try:
    if args.cmd == "screenshot":
      cmd_screenshot(scope, args.out)
    elif args.cmd == "waveform":
      cmd_waveform(scope, args.channel, args.out, args.mode)
    elif args.cmd == "catch-lockup":
      cmd_catch_lockup(scope, args.channel, args.timeout, args.png, args.csv,
                       args.poll, args.wait_limit or None,
                       level_v=args.level, mdepth=args.mdepth, tscale=args.tscale)
    else:
      cmd_idn(scope)  # default: prove the link works
  finally:
    scope.close()
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
