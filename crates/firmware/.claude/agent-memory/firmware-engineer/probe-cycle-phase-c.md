---
name: probe-cycle-phase-c
description: Phase C G38.x probing — probe input trait, GPIO pin, executor watch + sampling limit, outcome/position-sync mechanism
metadata:
  type: project
---

Phase C adds the G38.x probe cycle + Z-zero workflow (docs/tlo-offsets.md, gcode-streaming.md §9). Builds on Phase A
(ProbeFailInitial=4 / ProbeFailContact=5 alarm codes already in protocol.rs) and Phase B (coords.rs WCO/TLO, the `$#`
`[PRB:]` line that was a zeros/flag-0 stub in `coordinate_report()` in comms.rs).

**Probe input trait:** `ProbeInput` in hal_traits.rs exposes RAW level only (`is_high(&self) -> bool`). The `$6` invert +
`$19` pullup live in firmware-core `ProbeConfig`; host-tested `probe_triggered(raw_high, &cfg)` applies the invert. The
firmware GPIO impl (`RmtProbeInput`) applies the pullup at pin config (`init_probe`), invert per-sample in the executor.

**`$6` semantics decision (non-obvious — differs from docs' NO-plate framing):** `probe_triggered` defines `$6=0` as
"pin LOW = triggered" (idle-high input, grounded on contact) and `$6=1` as a pure electrical INVERT. The docs
(tlo-offsets.md Finding #6) say NO plates "need $6=1", but under this clean electrical model an idle-high NO plate
actually works with `$6=0`. The docs themselves flag this as empirical ("verify the Pn:P flag is ABSENT untouched").
We kept `$6` a single unambiguous invert so calibration works; tests assert the invert *behavior*, not a contested
NO/NC mapping. Settings `$6`=`probe_invert`, `$19`=`probe_pullup_disable` (proto fields 54/55, no schema bump).

**GPIO pin:** GPIO21 (PROBE). DOC-00 GPIO manifest had NO probe pin assigned; GPIO16/17 are feed-hold/cycle-start, GPIO18
is the spare RMT ch3. GPIO21 is the first genuinely-free input pin → documented choice. Input + pull-up (when `$19=0`).

**Sampling-resolution limit (hardware boundary, DOC-02):** grbl samples in the step ISR. Galdr RMT emits whole bursts
(≤47 ticks) that can't be preempted. So `ProbeStepper` (firmware-core motion.rs, host-tested) walks the probe block
emitting ONE tick per `emit_burst`, sampling the probe between every step → per-STEP granularity, the finest possible.
Over-travel bound = 1 step + the in-flight single-step burst decel; keep probe feed 25-100 mm/min.

**Parser→planner→executor flow:** gcode.rs `PlannerCommand::Probe{kind,…}` (G38.2/.3/.4/.5 via fractional dispatch, claims
motion group 1, error:23 `ProbeNoAxis` if no axis word, does NOT change modal motion mode). planner.rs resolves work→machine
target, flushes look-ahead, returns `PlannerOutcome::Probe{kind,target,feed,units}` + `Planner::sync_position(steps)`.

**Outcome + position-sync + ordering:** consumer (comms.rs) `PROBE_REQUEST` Signal (target/period/toward/invert) →
core-1 executor `run_probe`. Executor services the probe ONLY in the empty-queue `select3` branch (after all queued blocks
drain — correct ordering, since the probe flushed look-ahead so nothing follows it). On the edge it latches the live
StepCounter steps, publishes `PROBE_RESULT{triggered,stop_steps,already_at_edge}`. Already-at-edge (steps_emitted==0 on a
toward probe) → reported as `triggered=false, already_at_edge=true` so it's a FAILURE (ALARM:4), not a zero-travel success.
Consumer syncs planner position to stop point, stores `LAST_PROBE` (Cell, read by `coordinate_report()` for `$#` `[PRB:]`,
cleared on soft reset), pushes immediate `[PRB:x,y,z:flag]` (ResponseWriter::probe_report). Per-line response decided by
host-tested `protocol::probe_response(triggered, already_at_edge, alarm_on_fail)` → Ok | Alarm(ProbeFailInitial=4 /
ProbeFailContact=5). NOTE: docs/spec brief said "already-triggered → ALARM:5" but that contradicts grbl + protocol.rs +
gcode-streaming.md; we followed the authoritative docs: ALARM:4=already-triggered/wrong-initial-state, ALARM:5=no-contact.
