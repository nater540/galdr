---
name: streaming-state-sharing
description: How firmware comms tasks share machine state, live position, and buffer counts across cores
metadata:
  type: project
---

In `crates/firmware/src/comms.rs`, the shared-state plumbing for grblHAL streaming:
- `MACHINE: Mutex<CriticalSectionRawMutex, MachineSnapshot>` — the run-state / feed / spindle / rx-free fields.
  `status_responder` reads it on `?` and OVERWRITES the two genuinely-live fields at report time: MPos from the
  `LIVE_POSITION` atomics and `planner_blocks_free` from the live planner queue depth.
- `LIVE_POSITION: [AtomicI32; AXES]` — whole-steps-per-axis, SINGLE-owner = core-1 executor (motion.rs). The
  executor zeroes it on `MOTION_RESET`; consumer's `reset_pipeline` deliberately does NOT touch it (cross-core
  race avoidance, Finding #3). Reader uses Acquire loads, writer Release stores.
- Signals (each wakes ONE waiter, so one dedicated Signal per waiter): `STATUS_REQUEST`, `FEED_HOLD`,
  `CYCLE_START`, `SOFT_RESET` (consumer + back-pressure retry), `MOTION_RESET` (+`MOTION_RESET_PENDING` AtomicBool
  poll flag for the core-1 executor), `BLOCK_AVAILABLE` (planner→executor wake), `LINE_RESET` (line assembler).
- `SETTINGS: Mutex<.., Option<Settings>>` live settings; `SETTINGS_DIRTY` AtomicBool drives coalesced flash flush
  in `comms_consumer` (flush once per burst at queue-empty / safety interval / soft reset).
- `PLANNER: Mutex<.., Option<Planner>>` shared planner; consumer enqueues, core-1 executor pops.

`comms_consumer` is the SINGLE in-order consumer of `LINE_QUEUE` — natural owner of the gcode error-hold and
any new check/alarm/sleep gating. `handle_system_command` routes `$`-commands; the lenient `write_setting_command`
fallthrough was the fake-ack hole that Phase A hardens.

Feed-hold/cycle-start are honored by the core-1 executor at block boundaries (per-block granularity, Stage 1).
The `MachineState` shown by `?` must be driven from the same logic that gates execution.

**Phase A (implemented):** the authoritative latched control mode is `CONTROL: BlockingMutex<CSRawMutex,
Cell<ControlState>>` (SYNCHRONOUS blocking mutex — the real-time reader half updates it on `!`/`~` without
awaiting). `status_responder` composes the wire `MachineState` via `ControlState::machine_state(running)` where
`running = EXECUTOR_RUNNING || planner-has-queued-blocks`. `EXECUTOR_RUNNING: AtomicBool` is set by the core-1
executor (motion.rs) around `run_block`. `HOMING_ENABLED: AtomicBool` mirrors `$22` (updated on `$22=`/`$PBX`
import). `RESET_WAS_RUNNING: AtomicBool` is latched at `0x18` dispatch so the consumer's `apply_soft_reset` can
apply grbl's abort-alarm rule race-free. The pure state machine (`ControlState`, `AlarmCode`, `SystemCommand`
classifier, `UnlockOutcome`, `CheckToggle`) lives in firmware-core `protocol.rs` and is host-tested; comms.rs is
thin wiring. The `$`-dispatch is now `SystemCommand::classify`-driven — unknown `$` → `error:3` (no fake-ack).
Startup lines (`$N0`/`$N1`) are stored in RAM only (NOT persisted — would break `Settings: Copy`) and NOT
executed (Phase A scope). Sleep driver-disable + `$H` homing are stubbed (DOC-07/DOC-03/DOC-06 boundaries).
