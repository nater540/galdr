---
name: project-stop-and-position-retention
description: Change A (soft-reset RETAINS MPos, not zero) + Change B (graceful program stop 0x86) — firmware design + host contract.
metadata:
  type: project
---

Two grbl-alignment changes landed 2026-06-23 (branch fix/flash-persistence-skirnir-ui). Firmware side complete + host-tested; skirnir side is a SEPARATE agent's job (see host contract at bottom).

**Change A — soft reset RETAINS machine position (no longer zeroes MPos).** grbl keeps MPos through `0x18`; `$X` unlocks at the same coords. The executor used to zero `LIVE_POSITION` at every reset (`reset_live_position`), losing the operator's zero on a no-homing machine ($22=0). Fix:
- `crates/firmware/src/motion.rs`: `reset_live_position` (which did `counter.reset()`+publish) REPLACED by `retain_live_position(&StepCounter)` (re-publish only, NO reset). Called at the two reset-service sites: top-of-loop (~L427) and idle-branch MOTION_RESET wake (~L529). Hard-limit re-arming (`sample_limit_triggered`) unchanged.
- `crates/firmware/src/comms.rs` `reset_pipeline`: after `Planner::new(cfg)` (which starts at step origin), reads `read_live_position()` and `planner.sync_position(retained_steps)` BEFORE `*guard = Some(planner)`, so the rebuilt planner's commanded position matches the RETAINED MPos and a subsequent ABSOLUTE move resolves from it, not 0. `sync_position` only sets position + clears junction (safe before the WCO push). Executor is still the SINGLE owner of LIVE_POSITION (consumer never writes the atomics).
- Accepted race: on abort-DURING-motion the consumer's `read_live_position()` may run while the executor is still finishing its mid-block abort; counter advances monotonically within a block so it's a valid recent step position — "suspect" exactly as grbl documents, recovered by `$H`.
- Host-tested seam: planner test `rebuilt_planner_synced_to_retained_position_resolves_absolute_moves_from_it`.

**Change B — graceful program STOP (`0x86`), controlled stop, NO alarm, position retained.** A SECOND stop distinct from the `0x18` abort. Generalizes `cancel_jog_cycle` (jog) to a running program.
- firmware-core (protocol.rs): `RealtimeCommand::ProgramStop` mapped `0x86` in `classify_realtime` (between JogCancel 0x85 and FullStatusReport 0x87). Pure transitions on `ControlState`: `program_stop()` = `Normal|Hold(_) => Normal`, else self (never alarm); `program_stop_quiesces()` = true only for `Normal|Hold(_)` (the gate the bin uses). Both host-tested.
- planner.rs: `flush_queue()` (drains the WHOLE queue, program+jog, resets trailing junction — the program-stop counterpart to `flush_jog_blocks` which spares program blocks). Position NOT touched (caller syncs). Plus reuse existing `abort_arc()`.
- firmware/comms.rs: new `PROGRAM_STOP: Signal<CSRawMutex,()>`. `dispatch_realtime` signals it ONLY when `control_state().program_stop_quiesces()` (else benign). Added to consumer main `select` (now `select(select(events, HARD_LIMIT_TRIPPED), PROGRAM_STOP)` — note the Either-nesting shifted ALL prior arms one level deeper). New `program_stop_cycle(parser, state)`: re-check gate → flush_queue+abort_arc under lock → `quiesce_executor()` (reuse, real parked ack; ResetPreempted bails) → `sync_position(read_live_position())` → `release_hold()` → modal/spindle/override clear mirroring `program_end`/M30 (force_spindle_off, Parser::new, SpinUpGate::new, last_spindle_*=Stop/0, sync_active_wcs(0), Overrides::new, reset_ov_reporter) → `set_control_state(...program_stop())`. RETAINS coordinates/offsets/MPos, NO banner, NO warm reset.
- Back-pressure threading: new `PlanResult::Stopped` + `SpinUpInjection::Stopped`. `plan_command` QueueFull retry, `drive_pending_arc`, `handle_go_to_predefined` retry, and `inject_spin_up_dwell`'s dwell enqueue all now race PROGRAM_STOP alongside SOFT_RESET; on Stopped the caller (`plan_gcode_line`) runs `program_stop_cycle` (no `ok`). A stop during a `run_dwell`/`wait_for_motion_idle` (G4/M30) is deferred to the next main-loop select (bounded; grbl doesn't cut a dwell short either) — intentionally NOT threaded there.
- Single `Signal` `.wait()`'d at both the main-loop select AND the back-pressure waits is safe: consumer is only ever blocked at ONE at a time (same pattern as SOFT_RESET).

**HOST CONTRACT for skirnir (`0x86` graceful stop):**
- Host SENDS the single real-time byte `0x86` (out-of-band, like `0x18`/`0x85`, NOT line-buffered, NOT char-counted). No `$`-command form.
- Resulting transitions: Run(`Normal`+running) or Hold(`Hold:0`/`Hold:1`) → ... → `Idle`. From Idle/Alarm/Check/Sleep/Jog it is a BENIGN no-op (jog has its own `0x85`).
- `<...>` status DURING: stays `Run`/`Hold` until the active block decelerates to its boundary and the queue is flushed (sub-block latency, Stage-1 block-boundary granularity), then reports `Idle`.
- `<...>` status AFTER: `Idle`, `MPos:` RETAINED at the stop point (NOT zeroed), `FS:0`, planner buffer fully free (`Bf:32,...`). Spindle off. Overrides back to 100%. Modal state reset to power-on (G0/G90/G21/G54/F0/S0) — a `$G` after a stop shows defaults.
- Emits NOTHING that looks like an alarm: NO `ALARM:N`, NO banner. The in-flight line gets NO `ok` (host should discard pending acks the moment it sends `0x86`, same as `0x18`).
- DIFFERS from `0x18`: `0x18` aborts to `ALARM:3` (if mid-cycle) or boot-lock, re-emits the banner, runs a full warm reset (clears volatile coords G92/TLO, drops `[PRB:]`). `0x86` raises NO alarm, NO banner, KEEPS coordinates/offsets, and (post-Change-A) BOTH retain MPos.

Verified: `RUSTFLAGS="-D warnings" cargo test -p firmware-core` = 488 passing (was 481, +7). `just build` (Xtensa) + `just build --features defmt` both link clean. Pre-existing clippy lints in motion.rs/planner.rs (needless_range_loop, collapsible_if) are NOT in touched code — out of scope.

See [[project-hold-quiesce-protocol]] (quiesce_executor/release_hold reuse), [[project-consumer-pipeline]] (PlanResult/back-pressure), [[project-motion-executor]] (LIVE_POSITION ownership), [[project-protocol-contracts]] (RealtimeCommand/ControlState).
