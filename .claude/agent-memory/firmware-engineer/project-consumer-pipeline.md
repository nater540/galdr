---
name: project-consumer-pipeline
description: Galdr firmware-bin parser→planner consumer wiring decisions — error-hold ownership, back-pressure/drain model, and the stub block-drain that stands in for the motion executor.
metadata:
  type: project
---

The firmware-bin `comms_consumer` was upgraded from the Stage-1 ack-stub to the REAL gcode parser → planner
pipeline on 2026-06-16 (branch `firmware/core-pipeline-and-streaming`). Lives in `crates/firmware/src/comms.rs`.
Decisions a future author (esp. the DOC-02 motion_executor phase) needs that are NOT obvious from the code:

**Single FUSED parse+plan consumer (rejected DOC-01's split gcode_parser + Channel<PlannerCommand> + planner
tasks).** One `comms_consumer` task owns a persistent `gcode::Parser` AND drives the shared `Planner`. **Why:**
the `ok`/`error:N` decision and the back-pressure decision both need the parse result and the planner-accept
result in ONE place; splitting scatters `error:N` across tasks and makes the error-hold racy. DOC-01's split
predates this realization.

**Gcode error-hold lives in the CONSUMER, not the StreamEngine (`ConsumerState::error_hold: bool`).** The
framer runs in `line_assembler` and forwards lines async, so it cannot know a line errored downstream; any
back-channel races lines already in `LINE_QUEUE`. The consumer is the SOLE in-order reader of `LINE_QUEUE`, so
owning the hold there is race-free by construction. **`StreamEngine::note_line_error()` was DELETED** (the engine
is now pure line framing — see [[project-protocol-contracts]] #9 refactor). The framer still owns the independent
protocol-level overflow reject. Recovery triggers in the consumer: blank line (now forwarded as empty AcceptLine
→ `handle_line` clears hold + bare `ok`), `$` command, and soft reset. **CHANGED: blank lines now DO reach the
consumer** (the framer no longer acks them upstream); the consumer's `handle_line` trims + handles empties.

**`$G` reports LIVE modal state (#3 fix).** `send_parser_state(parser)` builds a `ParserSnapshot` via
`parser_snapshot(parser.state())` (the bin's bridge from `gcode::ModalState`→protocol enums) and passes it to
`ResponseWriter::parser_state`. `handle_system_command(rest, parser)` threads the parser for the `$G` arm.

**PLANNER==None now returns `error:3` (ERROR_PLANNER_UNINITIALIZED), not a fake `ok` (#P2).** An init wiring bug
fails the line loudly instead of fabricating `Ok(Queued{blocks:0})`. Unreachable in a wired build.

**Back-pressure = block-and-retry-same-command (no drop, no ack on QueueFull).** `plan_command` loops: on
`PlannerError::QueueFull` it races `Timer::after(QUEUE_FULL_RETRY=2ms)` against `SOFT_RESET.wait()` and retries the
SAME command (arc planner is all-or-nothing on QueueFull per [[project-planner-contracts]], so re-issue is safe).
Blocking here backs up `LINE_QUEUE` → blocks `usb_rx`'s `send().await` → stops the byte scanner → host char-counting
throttles. Returns a `PlanResult` enum {Accepted, Error(code), Aborted}; Aborted (soft reset won the race) runs
`reset_pipeline` since the signal was consumed in `plan_command`.

**SOFT_RESET ownership moved to the consumer.** Only `comms_consumer` waits on `SOFT_RESET` now (via `select` in its
main loop AND inside the back-pressure retry). `reset_pipeline` rebuilds `Parser::new()` (no in-place reset) +
`Planner::new(cfg)` (no public flush — reconstruct clears queue/position/offset/junction) + resets `MACHINE` snapshot
to idle. `usb_rx` still flushes LINE_QUEUE + re-emits banner on 0x18.

**Shared `PLANNER: Mutex<Option<Planner>>` static** (Option because `Planner::new` isn't const). `init_planner()` is
called once in `main` BEFORE spawning tasks (uses `try_lock`, no await). Config = `placeholder_planner_config()` =
`PlannerConfig::default()` — TODO(DOC-00) replace with esp-storage `$`-settings load.

**`block_drain_stub` task is a PLACEHOLDER for the DOC-02 motion_executor.** Pops one planner block, paces by
`simulated_block_duration` = `millimeters / nominal_speed()` clamped to [5ms, 500ms] (crude cruise-only estimate,
ignores accel ramps), publishes `planner.position_mm()` + free-block count into `MACHINE` so `?` reports plausible
MPos. Generates NO step pulses. The real executor (core-1 InterruptExecutor + RMT, awaiting BLOCK_AVAILABLE not
polling) DELETES this task; the shared `PLANNER` handoff stays. No lib API additions were needed (used existing
`pop_block`/`position_mm`/`queued_len`/`nominal_speed`/`millimeters`/`BLOCK_QUEUE_LEN`).

**Build verified:** host `cargo test -p firmware-core` (138 green) + `RUSTFLAGS="-D warnings" cargo build` + clippy.
Xtensa: `cd crates/firmware && source $HOME/export-esp.sh && cargo build` AND `... --features defmt` both link clean.
Clippy gotcha hit: `!(x > 0.0)` NaN-guards trip `neg_cmp_op_on_partial_ord` — write the positive `x > 0.0` form.

See [[project-protocol-contracts]], [[project-planner-contracts]], [[project-motion-contracts]], [[project-firmware-bringup]].
