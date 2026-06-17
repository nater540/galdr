---
name: overrides-phase-e
description: Phase E feed/rapid/spindle overrides + Ov:/Pn:/FS: status fields — Overrides model, executor live-scaling, status wiring
metadata:
  type: project
---

Phase E of the grblHAL streaming work: real-time feed/rapid/spindle overrides (`0x90`-`0xA4`) and the
`Ov:`/`Pn:`/`FS:` status fields. Builds on A (real-time dispatch table, status formatter), B (coords), C
(probe `DigitalIn`), D (jog feed). Landed on `firmware/core-pipeline-and-streaming`.

**Override model (`protocol.rs`, host-tested):** `Overrides { feed, rapid, spindle: u8, spindle_stop, flood,
mist: bool }`. `apply(byte) -> bool` (true if changed) decodes the matrix: feed `0x90` reset-100 / `0x91`+10 /
`0x92`-10 / `0x93`+1 / `0x94`-1; rapid `0x95`/`0x96`/`0x97` = 100/50/25; spindle `0x99`/`0x9A`-`0x9D` same as
feed; `0x9E` spindle-stop toggle; `0xA0`/`0xA1` flood/mist toggle. Bounds `OVERRIDE_MIN/MAX_PCT` = 10/200 (feed
+ spindle clamped via `clamp_override` in i16); rapid discrete. `0x98`/`0xA2`-`0xA4` are no-ops. Scaling helpers:
`scaled_feed(programmed, max_rate)` (clamps to max-rate so boost ≤ `$110-112`), `scaled_rapid(rate)`,
`scaled_rpm(rpm)` (0 when spindle_stop). `OvReporter` mirrors `WcoReporter` (change+cadence, `OV_REFRESH_PERIOD`
=10). `PinReport { probe, limits[3], door, hold, reset, cycle_start }` + `write_letters` in grbl order P/X/Y/Z/
D/H/R/S, `any()` gates omission. `MachineSnapshot` gained `pins`, `overrides`, `include_ov`; `feed_mm_min`/
`spindle_rpm` REDEFINED as REALIZED (programmed × override). `RESPONSE_CAPACITY` bumped 128→160 for the wider
report. Status formatter emits `...|FS|Bf|Pn:|WCO:|Ov:>` (Pn after Bf, Ov after WCO).

**Executor live-scaling (`firmware-core/motion.rs` + `firmware/motion.rs`):** added
`SegmentGenerator::run_block_scaled(block, exit_sq, override_scale, max_speed_sq, sink)`; `run_block` delegates
with scale 1.0 / ceiling ∞ (all existing call-sites/tests unchanged). Scaling multiplies entry/nominal/exit by
`scale²`, clamps each to `max_speed_sq` via `clamp_speed_sq`; accel NOT scaled (grbl keeps accel limits, ramp
just lengthens). Degenerate scale (≤0/NaN/∞) → 1.0. `TrapezoidProfile::plan_scaled(block, &ScaledBlock, exit)`
is the shared core; the old `plan` was removed (dead after delegation — `#![deny(warnings)]` catches it). The
core-1 executor reads `overrides()` PER BLOCK in `run_block`, picks rapid-vs-feed override by `block.rapid`,
publishes programmed feed (`block.nominal_speed()*60` mm/min) into `LIVE_PROGRAMMED_FEED_MM_MIN` (AtomicU32
f32-bits) + `LIVE_BLOCK_IS_RAPID`, zeroes feed when motion stops. `motion::run` + `motion_executor` task gained
a `max_rate_mm_min: [f32;AXES]` param (from `planner_config.max_rate_mm_min` in main.rs); `min_max_rate_mm_s`
gives the conservative squared ceiling.

**Wiring (`comms.rs`):** `OVERRIDES: BlockingMutex<Cell<Overrides>>` (synchronous, the `CONTROL` pattern — the
real-time reader half mutates without awaiting). `dispatch_realtime`'s `Override(byte)` arm: `ov.apply(byte)`,
on change `set_overrides` + `OVERRIDE_CHANGED.signal()` (promptness hint; executor re-reads per block anyway).
NEVER acked. `reset_pipeline` (0x18/0x19) resets overrides to default, zeroes live feed/spindle, `reset_ov_
reporter()`. `status_responder` fills `overrides`, `include_ov` (via `OV_REPORTER` cell), realized `feed_mm_min`
(`scaled_feed`/`scaled_rapid` w/ `min_axis_max_rate` ceiling), realized `spindle_rpm` (`scaled_rpm` of
`PROGRAMMED_SPINDLE_RPM`), and `pins.probe` from `PROBE_ASSERTED`. `PROGRAMMED_SPINDLE_RPM` published by consumer
in `plan_gcode_line` from `parser.state().spindle_speed` on any clean parse. `PROBE_ASSERTED: AtomicBool`
published by the probe cycle's `at_stop_edge` closure (logical, post-`$6`); holds last sample.

**Hardware-boundary stubs:** spindle LEDC PWM duty = `TODO(DOC-05)` in the Override arm (realized RPM computed +
reported, no PwmSink yet). Coolant flood/mist GPIO + limit/door/hold/reset/cycle-start `Pn:` letters = `TODO(DOC-
06)` (tracked in state, reported as false, assembly logic complete + host-tested). `Pn:P` only continuously
sampled DURING a probe cycle — idle input-poll task is a DOC-06 follow-up.

Verified: `cargo test -p firmware-core` 335 green (311 base + 19 protocol + 5 motion) + lib clippy clean; Xtensa
build clean (only pre-existing `drop(async_flash)` lints). See [[streaming-state-sharing]], [[jogging-phase-d]],
[[probe-cycle-phase-c]], [[build-test-commands]].
