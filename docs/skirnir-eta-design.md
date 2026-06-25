# Skirnir Job-Time Estimation (ETA) — Design & Research Report

> Status: **design / research** (not yet implemented). Companion to `docs/native-app.md`,
> `docs/skirnir-design-brief.md`, and `docs/gcode-streaming.md`. The firmware motion model this
> mirrors lives in `crates/firmware-core/src/{planner,motion,settings,spindle,gcode}.rs`.

## 1. Goal & decisions

Give the operator a **realistic ETA**: an upfront total computed by a "Simulate" button (a 10–20 s
host computation is acceptable) plus a **live-updating remaining time** during the run. Target
accuracy: within a few percent of wall-clock for the *cut portion* of a job.

Decisions taken (2026-06-25):

- **Reuse the real planner via the shared `cnc-kinematics` crate** — `skirnir` drives the SAME
  `gcode`/`planner`/`motion` code the firmware runs, offline over the whole file. This was originally
  scoped as a "standalone simulator in skirnir" with a golden cross-check test to bound drift; it has
  since been **superseded** by extracting the planner into `crates/cnc-kinematics` (done — see that
  crate and the "five crates" note in `CLAUDE.md`), so drift is structurally impossible rather than
  test-bounded. §3 documents the motion model the crate already implements; the only NEW code is a
  small time-integration helper (below) that lives IN `cnc-kinematics` so it is unit-tested next to
  the planner.
- **Maximum-fidelity factor set**: full kinematics (per-axis accel/decel, junction-deviation
  cornering, max-rate clamps, feed rates, G93/G94, rotary), plus G4 dwell, spindle spin-up/reverse
  dwell, the USB/streaming throughput ceiling, the bounded look-ahead window, and runtime
  feed/rapid overrides.
- **Both upfront and live** ETA surfaces.

### Why this is hard (prior-art summary)

A naive `distance ÷ feedrate` estimate **systematically underestimates** because it assumes
instantaneous acceleration and infinite cornering speed. Measured magnitudes from the field:

| Source | Naive error |
|---|---|
| LinuxCNC native estimate (accel-blind) | "usually **half** the actual time" (~2× worst case) |
| Fusion 360 (distance/feed only) | community consensus **20–30%** |
| G-Wizard, acceleration correction alone | ~**8%** of total on one real file |
| Rapids (G0) omitted, small parts | ~**25%** of total time |

The settled fix in the literature is a **planner-accurate offline simulator** that re-runs the same
trapezoid + junction-deviation planner over the whole file ahead-of-time. The strongest reference is
`klipper_estimator` (Rust): it reimplements Klipper's kinematics + look-ahead and lands **< 1 minute
of error over a 12 h job (~0.14%)** for the motion it models — "anything worse is a bug." Among grbl
senders, only **Candle** does this properly (it reimplements grbl's planner incl. junction
deviation); **gSender** models per-move accel but **no cornering** (and therefore over-estimates,
per its own TODO); **UGS/CNCjs** are pure runtime extrapolation (`elapsed ÷ rows`, useless until the
job is underway); **bCNC** is naive distance/feed. firmware (grblHAL/FluidNC) computes **no** ETA —
estimation is always the sender's job.

Galdr's advantage: the `cnc-kinematics` crate IS the host-tested trapezoid + junction-deviation
planner (the exact code the board runs). `skirnir` depends on it directly, so it matches the firmware
bit-for-bit instead of approximating it — there is nothing to validate against because it *is* the
thing.

## 2. Architecture overview

`skirnir` depends on `cnc-kinematics` (host-only `sim` feature) and uses its `gcode`/`planner`/`motion`
modules directly. The code splits into a small time-integration core that lives IN the shared crate (so
it is unit-tested next to the planner) and thin glue in skirnir — **both implemented**:

1. **In `cnc-kinematics` (`motion` + the `sim` module)** — the only genuinely new motion code:
   `estimate_block_time(&Block, exit_speed_sq, &MotionConfig)` does the discrete per-step sum of §3.5
   (unit-tested to equal, tick-for-tick, the periods `SegmentGenerator::run_block` actually emits — the
   no-drift guarantee), and `sim::simulate(&[PlannerCommand], &PlannerConfig, &MotionConfig)` feeds a
   `Planner` over the command stream, slides the bounded look-ahead by popping blocks, and returns one
   `SimStep` per command (move time + rapid flag, dwell, spindle, pause, untimed). `sim` is gated behind
   a host-only `sim` feature (it allocates), kept off the no-alloc firmware build.
2. **`skirnir` parsing** — `skirnir/src/eta.rs` uses `cnc_kinematics::gcode::Parser` to parse program
   lines into `PlannerCommand`s (the SAME parser the firmware uses, so the estimator accepts exactly
   what the board accepts — this subsumes the `error:20` unsupported-codes concern; no second parser to
   keep in sync), and `configs_from_settings(get)` reads the `$$` motion settings into
   `cnc_kinematics::{PlannerConfig, MotionConfig}`, falling back to firmware defaults per field.
3. **`skirnir/src/eta.rs`** — a thin framework-agnostic module (sibling to `progress.rs`/`flow.rs`)
   that calls the shared `sim` to build a **per-line `EtaTimeline`** (cumulative seconds, split
   feed-time vs rapid-time, dwell as fixed time, operator pauses surfaced) and exposes
   `remaining_seconds(completed_lines, feed_override, rapid_override)` for the live projection (§6). No
   motion math here — it orchestrates the shared crate. The §4 spindle-dwell and §5 throughput factors
   are marked seams not yet filled (they need the spindle settings / bench-measured params).

Wiring into the app:

- A new `Intent::Simulate` (enum in `app/intent.rs:65`) with a button in `transport_group`
  (`app/views.rs:619`) beside Run/Stop. Simulate is a **pure host computation** — it does not send a
  `Command` to the engine; it parses `self.ui.program` and stores a `SimResult` on the shell.
- Surface the upfront total and live remaining through the **existing `TimeEstimate` seam** already
  threaded into `dock_progress` (`app/views.rs:1764`, computed at `app/shell.rs:1524`). Today `total`
  is `None` until the first ack; the simulation fills it immediately and replaces the naive live
  projection.

## 3. The motion model to replicate (from firmware-core)

This is the core of the estimator and must match `crates/firmware-core/src/{planner,motion}.rs`
**exactly**. Axis order `[X, Y, Z, A]`, `AXES = 4`. A is rotary by default (`$376` mask, default `8`).

All per-block speeds are carried as **squared mm/s** to stay sqrt-free, governed by
`v² = v₀² + 2·a·d`.

### 3.1 Per-block build (`planner.rs:988-1069`)

- **Length / unit vector**: `millimeters = sqrt(Σ delta_mm[axis]²)` over **all 4 axes**, with the A
  delta in **degrees treated as mm** (the degrees-as-mm convention). `unit_vec = delta / millimeters`.
  Drop zero-length moves (`LENGTH_EPSILON_MM = 1e-6`).
- **Block acceleration** (`limiting_acceleration`, `planner.rs:1510`):
  `accel = min over participating axes of (accel_mm_s2[axis] / |unit_vec[axis]|)`.
- **Nominal (cruise) speed** (`nominal_speed_mm_s`, `planner.rs:1050`):
  - G0 rapid: `nominal = axis_rate_limit` (ignores F entirely).
  - G94: `nominal = min(F·units_scale/60, axis_rate_limit)` mm/s (`units_scale` = 1.0 mm / 25.4 inch).
  - G93 inverse-time: `nominal = millimeters · F / 60` (F is **not** unit-scaled).
  - `axis_rate_limit = min over participating axes of ((max_rate_mm_min[axis]/60)/|unit_vec[axis]|)`.
- **Junction speed** (`junction_speed_sq`, `planner.rs:1126`) — grbl centripetal model over all 4
  axes: with `cos θ = -(prev_unit · unit)`, `sin_half = sqrt((1-cosθ)/2)`,
  `radius = junction_deviation · sin_half / (1 - sin_half)`,
  `junction_speed_sq = min(accel · radius, prev_nominal_speed_sq)`. First move starts from rest (0);
  full reversal (`cosθ ≥ 1-1e-6`) → 0; collinear (`sin_half ≤ 1e-6`) → +∞ (no corner limit).
- **Seed**: `max_entry_speed_sq = min(junction_speed_sq, nominal_speed_sq)`; `entry = max_entry`.

### 3.2 Mixed-rotary feed (ROTARY_FIX, `planner.rs:1080-1105`)

A **G94 move on both a linear and a rotary axis** is converted to inverse-time so F governs the
linear path, not the inflated 4-axis norm:
```
if linear_len > 1e-6 and rotary_len > 1e-6:
    effective_feed = F · units_scale / linear_len ; mode = InverseTime
```
This makes a combined XYZ+A line finish in `linear_len / F` minutes, exactly like a linear-only move
at F. Rapids skip it. **The estimator must replicate this** or any 4-axis line is badly mistimed.
(See `memory: project-4th-axis-grblhal-rotary-feed` for why this matches grbl's ROTARY_FIX.)

### 3.3 Bounded look-ahead (`BLOCK_QUEUE_LEN = 32`, `planner.rs:58`)

The planner ring buffer holds **at most 32 blocks**, and `recalculate` (reverse then forward pass)
only ever sees the resident window — never the whole program. An infinite-look-ahead host sim would
over-optimize cornering (plan deeper decel ramps than 32 blocks of travel allow). **Run the
look-ahead over a sliding 32-block window**, recalculating as blocks enter/leave:

- **Reverse pass** (newest→oldest, `planner.rs:1187`): seed `next_entry_sq = 0` (newest must be able
  to stop); `entry = min(max_entry, next_entry + 2·accel·mm)`; propagate.
- **Forward pass** (oldest→newest, `planner.rs:1215`): cap each `entry` by
  `prev.entry + 2·prev.accel·prev.mm`.
- Each block's **exit = next block's entry**; the final block exits at 0.

### 3.4 Trapezoid classification (`motion.rs:630-679`)

With `two_a = 2·accel`, `accel_dist = max((nominal²-entry²)/two_a, 0)`,
`decel_dist = max((nominal²-exit²)/two_a, 0)`:
- If `accel_dist + decel_dist ≤ length`: **trapezoid**, `cruise² = nominal²`,
  `accel_end = accel_dist`, `decel_start = length - decel_dist`.
- Else **triangle** (nominal never reached): `accel_end = ((exit²-entry²)/two_a + length)·0.5`
  clamped to `[0,length]`; `peak² = max(entry² + two_a·accel_end, max(entry²,exit²))`;
  `cruise² = peak²`; `decel_start = accel_end`.
- `accel ≤ 0` collapses ramps → pure cruise.

### 3.5 Time integration — the discrete per-step sum (`motion.rs:185-313`)

**The firmware does not integrate time analytically.** It emits one step *period* per dominant-axis
step and physical time is the sum of those periods. To match it:

- Dominant axis steps every tick; `total_ticks = step_event_count` (dominant-axis step count, from
  `steps_per_mm`). For step `i` of `total`: midpoint `d = (i+0.5)/total · length`.
- `v² at d`: accel region `min(entry² + two_a·d, cruise²)`; cruise region `cruise²`; decel region
  `min(exit² + two_a·(length-d), cruise²)`.
- `step_rate = sqrt(v²) / dom_mm_per_step`, clamped to `max_rate_hz`;
  `period_ticks = round(tick_hz / step_rate)` clamped to `[min_period_ticks, max_period_ticks]`.
- `tick_hz = 1_000_000` (1 tick = 1 µs, `firmware/src/main.rs:112`);
  `min_period_ticks = step_pulse_ticks($0 µs) + min_low_ticks(2)` (default 12 → ~83.3 kHz cap);
  `max_period_ticks = RMT_MAX_FIELD_LEN(32767) + step_pulse_ticks` (slow-feed floor ~one step /
  32.8 ms — matters only for very slow probing feeds).
- `T_block = Σ (period_ticks_i / tick_hz)`.

**Accuracy note:** a closed-form trapezoid integral `T = Σ 2Δd/(v₀+v₁)` per region is a good first
approximation and is *much* cheaper, but it drifts from the firmware by the per-step rounding +
clamp behavior. For the "as accurate as possible" target, the model should reproduce the **discrete
sum with the min/max period clamps**. A practical compromise: analytic integral as the default fast
path, discrete sum behind a flag, with a `cnc-kinematics` unit test asserting the analytic path stays
within an agreed epsilon of the discrete one. (A 10–20 s budget over a million-step file is ample for
the discrete sum if vectorized.)

### 3.6 Arc subdivision (G2/G3, `planner.rs:1551-1568`)

Arcs are subdivided into linear chord blocks **in the planner**, each a normal block subject to all
of §3.1–§3.5. Segment count uses `max_angle = acos(1 - arc_tolerance/radius)` and
`segments = max(ceil(|sweep| / max_angle), 1)`.

> **Firmware correctness flag (raise with the team):** the code comment at `planner.rs:1552-1553`
> says the per-segment angle is `2·acos(1 - tol/r)`, but the code computes a single `acos` — so the
> firmware emits **~2× the textbook segment count**. The estimator must match the **code**
> (single `acos`), but it's worth confirming whether the firmware *intends* the doubled
> subdivision, since it currently over-segments every arc. Tracked separately from this ETA work.

### 3.7 Settings the model needs (read from the host `SettingsModel`)

| `$` | Meaning | Default | Role |
|---|---|---|---|
| `$0` | step pulse µs | 10 | sets `min_period_ticks` (max step rate) |
| `$11` | junction deviation mm | 0.01 | cornering radius |
| `$12` | arc tolerance mm | 0.002 | arc chord count |
| `$100-103` | steps/mm (A: steps/deg) | 250 / 8.889 | step counts, `dom_mm_per_step` |
| `$110-113` | max rate mm/min (A: deg/min) | 500 / 3600 | rate clamp, rapid speed |
| `$120-123` | accel mm/s² (A: deg/s²) | 10 / 360 | block acceleration |
| `$376` | rotary axis mask | 8 (A) | which axes are degrees / ROTARY_FIX |
| `$392` | spindle spin-up delay s | 0.0 | non-motion time (§4) |
| `$393` | spindle reverse dwell s | 1.5 (floor 0.5) | non-motion time (§4) |

There is **no** min-feed setting (only `.max(0.0)` + the slow-step period floor) and **no**
spindle-RPM setting affects time. `min_low_ticks (2)` and the tick clock are firmware constants, not
`$`-settings.

## 4. Non-motion time

The firmware-core pipeline assigns **zero** time to non-motion ops; the durations are awaited by the
async firmware tasks. The estimator adds them explicitly:

- **G4 dwell** (`gcode.rs:599`): add `P` **seconds** directly (P is always seconds here). The single
  non-motion time computable exactly from the GCode.
- **Spindle spin-up `$392`**: add after each M3/M4 before the first cut (default 0 ⇒ usually nothing).
- **Spindle reverse dwell `$393`** (`spindle.rs:153`): add `max($393, 0.5)` s on each **M3↔M4
  reversal** of a running spindle; a same-direction speed change adds nothing.
- **M0/M1/M6** (`gcode.rs:726`): **unbounded operator pauses** (M6 is a pause, not a timed tool
  change, in this firmware). The ETA cannot estimate these — treat as 0 and **surface a "pauses at
  line N" annotation** so the displayed total is honestly a "minimal time."

`klipper_estimator` does exactly this — it ignores macros/heat-up/homing and labels its output a
*minimal* time. We follow the same convention and call out preludes (`$H`, G38 probing) and pauses.

## 5. Throughput & small-segment ceilings

Even a perfect kinematic estimate is a **lower bound** in the dense-small-segment regime (laser
raster, 3D relief, PCB isolation) — the controller becomes block-rate or serial-rate limited, so
actual time *exceeds* the kinematic time. Two independent ceilings:

1. **Block-processing rate.** Measured field data: 8-bit grbl ≈ 400 blocks/s; on grblHAL a default
   36-block buffer dropped a commanded 18,000 mm/min raster to **8,569 mm/min at 0.5 mm segments**,
   fixed only by enlarging the planner buffer. Galdr's window is **32 blocks** (`BLOCK_QUEUE_LEN`),
   so it is exposed to this. Model it as: `effective_feed = min(commanded_feed, blocks_per_s ·
   segment_length)`; clamp the block's nominal accordingly. `blocks_per_s` for the ESP32-S3 dual-core
   split is **unknown today and must be bench-measured** (parameter, not a hardcoded constant).
2. **Serial / USB throughput.** Host character-counting against the RX buffer (`flow.rs`,
   `DEFAULT_RX_BUFFER = 1024`, refined from `[OPT:]`). A line is released only when an `ok` frees a
   slot. For tiny-segment-dense jobs the job can be **ack-rate-limited**: floor the estimate at
   `line_count · per_line_ack_latency`. Galdr uses **native USB CDC**, not 115,200 baud, and a
   1024-byte RX buffer — so serial is far less likely to bind than on classic 8-bit grbl (where
   115,200 alone capped dense raster at ~2,500 mm/min). `per_line_ack_latency` is **unknown and must
   be bench-measured**; until then, model serial as non-binding and note the assumption.

Final per-job floor: `T = max(kinematic_time + non_motion_time, line_count · ack_latency)`, with the
block-rate clamp applied per-segment inside the kinematic pass. **Both ceilings carry
bench-measured parameters** — surface them as operator-visible constants (like the DOC-11 bench
params) rather than magic numbers, so they can be tuned against real runs.

## 6. Live ETA

The upfront sim produces a **per-line cumulative-time timeline**, split into `feed_seconds` and
`rapid_seconds` per line. During the run:

- **Progress key.** Map live progress onto the timeline by **cumulative time per line**, *not* line
  count — `Progress { sent, acked, total }` (`engine.rs:123`) counts lines, and `acked` means
  "accepted into the planner," which *leads* the real tool. Prefer the firmware's reported current
  line `Ln:` from the `<...>` status report (`status.rs:217`) when the program carries line numbers;
  fall back to `acked` otherwise. (Optionally refine within a line using `MPos` vs the line's start/
  end position for sub-line smoothness.)
- **Override rescaling.** Read the **live applied** override from `status.overrides` (`Ov:`,
  `status.rs:207`) rather than the host estimate. Rescale the *remaining* time component-wise:
  `remaining_feed / (feed_ov/100)` + `remaining_rapid / (rapid_ov/100)`. Spindle override does not
  affect time. This is why the timeline stores feed vs rapid separately.
- **Display.** Feed the precomputed remaining into the existing `TimeEstimate` → `dock_progress`
  clock (`m:ss / m:ss`). Keep the current naive `progress.rs::estimate` as a **fallback** for when
  no simulation has been run (e.g. operator hit Run without Simulate, or settings weren't loaded).

## 7. Accuracy budget & validation

The original dominant risk — the estimator **drifting from the real planner** — is now **eliminated
by construction**: skirnir runs the *same* `cnc-kinematics` planner the firmware runs, so there is no
second implementation to drift. (The earlier plan's "golden cross-check test against a standalone
reimplementation" is therefore obsolete — there is nothing separate to cross-check.) What remains:

1. **Unit tests in `cnc-kinematics`** for the new `sim`/`estimate_block_time` code, per the
   crate's existing test pattern: scripted GCode + a fixed settings snapshot → assert durations
   (rapid-only, single accel-limited move, triangle move, sharp corner, arc, mixed XYZ+A, G93, G4
   dwell, M3↔M4 reversal). Because the planner itself is already covered by the crate's 240+ tests,
   these only need to pin the *time integration* on top of it.
2. **Bench validation** against real wall-clock runs (hardware-gated, like the other DOC checklists),
   to fit the two unknown throughput parameters (`blocks_per_s`, `ack_latency`) and confirm the
   few-percent target. Add a `docs/skirnir-eta-bench-checklist.md` companion when implementing.

**Residual error sources** that remain even with a perfect kinematic sim (set operator expectations
accordingly): unbounded M0/M1/M6 pauses; `$H`/G38 preludes counted as zero; configured-vs-real accel
mismatch (the entire LightBurn/PrusaSlicer accuracy story — if `$120–$123` don't match the machine,
error follows directly, which is *another* reason to read live `$$`); block-rate/serial starvation
on dense paths; and live feed-holds. The honest framing, like klipper_estimator, is that the upfront
number is a **minimal cut time** plus annotated pauses.

### Hard runtime dependency

Accurate kinematics require the firmware's `$$`/`$ES` settings to be loaded on the host. skirnir does
**not** auto-fetch them at connect, and clears them on disconnect (`settings_model.rs:140`). The
**Simulate action must ensure settings are present** — trigger a `$$` fetch first, or fall back to
documented defaults and clearly mark the estimate as "using default machine settings." Note: `$PBX`
proto settings-sync is *not* implemented in skirnir today (`engine.rs:20-21`); the text `$$`/`$ES`
path is the only source, and values arrive as **untyped strings** (`settings_model.rs:36`) that the
estimator must parse to `f64` itself.

## 8. Implementation plan (suggested order)

Prerequisite (**done**): the planner is extracted into `crates/cnc-kinematics` and `skirnir` depends on
it — drift is eliminated by construction (§7).

1. **In `cnc-kinematics` — done.** `estimate_block_time(&Block, exit_speed_sq, &MotionConfig)` (discrete
   per-step sum, §3.5) + the `sim::simulate` driver (host-only `sim` feature). Unit-tested per §7.1,
   including the tick-for-tick equality with `run_block`.
2. **In `skirnir` — done.** `eta::EtaTimeline::build` parses via `cnc_kinematics::gcode::Parser`;
   `eta::configs_from_settings` builds `PlannerConfig`/`MotionConfig` from the `$$` getter with per-field
   default fallback. No new parser.
3. **`skirnir/src/eta.rs` — done (core).** Builds the per-line timeline from the shared `sim` and exposes
   override-aware `remaining_seconds`. Still open: add non-motion spindle time (§4) + throughput floors
   (§5) with bench params surfaced in UI (marked seams).
4. **Open:** `Intent::Simulate`, shell-owned timeline, Simulate button, settings-fetch guard.
5. **Open:** live projection (§6) wiring: timeline indexing by `Ln:`/`acked`, override rescaling fed from
   `status.overrides`, `TimeEstimate` seam into `dock_progress`.
6. **Open:** `docs/skirnir-eta-bench-checklist.md` + bench-fit the throughput params on hardware.

## 9. References

Internal: `crates/firmware-core/src/{planner,motion,settings,spindle,gcode}.rs`;
`crates/skirnir/src/{app/views.rs,protocol/{flow,settings,status}.rs,view_state.rs,app/progress.rs,
app/overrides.rs,engine.rs}`; `docs/4th-axis-rotary-design.md`; `docs/gcode-streaming.md`.

External prior art:
- klipper_estimator (Rust, the reference planner-accurate estimator) —
  https://github.com/Annex-Engineering/klipper_estimator ; test harness
  https://github.com/dalegaard/klipper_estimator_test
- Candle `TimeEstimator` (only grbl sender with a full planner sim) —
  https://github.com/Denvi/Candle (`src/candle/utils/timeestimator.cpp`)
- gSender per-move trapezoid, no cornering — https://github.com/Sienci-Labs/gsender
  (`GCodeVirtualizer.ts`)
- UGS/CNCjs empirical extrapolation — https://github.com/winder/Universal-G-Code-Sender ,
  https://github.com/cncjs/cncjs (`src/server/lib/Sender.js`)
- Klipper trapq time model — https://www.klipper3d.org/Kinematics.html ,
  https://github.com/Klipper3d/klipper/blob/master/klippy/chelper/trapq.c
- LinuxCNC accel-blind native estimate (~2× error) —
  https://forum.linuxcnc.org/38-general-linuxcnc-questions/13934-run-time-calculation-in-g-code-program-properties
- Small-segment / planner-buffer ceiling field data —
  https://github.com/grblHAL/core/discussions/189 , https://github.com/terjeio/grblHAL/issues/142
- grbl character-counting flow control / RX buffer — https://deepwiki.com/gnea/grbl/5-communication-interface
