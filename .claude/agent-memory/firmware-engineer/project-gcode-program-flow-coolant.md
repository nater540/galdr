---
name: project-gcode-program-flow-coolant
description: Where M0/M1/M6 program-pause + G18/G19 plane + M7/M8/M9 coolant live and how they wire through the parser/consumer/spindle pattern
metadata:
  type: project
---

DOC-04/DOC-07 extension work (2026-06-23): M0/M1/M6 pause, G18/G19 planes, M7/M8/M9 coolant.

**Parser (`firmware-core/src/gcode.rs`):**
- M-word handler is `apply_m_word` (~line 1032). M2/M3/M4/M5/M30 supported. `emit` (~line 1086) applies modal precedence.
- `PlannerCommand` enum (~line 466). `ProgramEnd` is the M2/M30 variant. New pause variants added here.
- Plane modal: `Group::Plane` exists in `GroupGuard` but only G17 claimed it; no `Plane` enum on `ModalState` until this work. Arc planner (`planner.rs` `plan_arc` ~1153) is HARD-CODED to XY indices 0/1 + Z helix.
- `T` word (~849) was a no-op (no M6). M6 must consume a pending tool.

**Consumer (`firmware/src/comms.rs`):**
- `plan_command` (~1661) loops on QueueFull, maps `PlannerOutcome`→`PlanResult` (~1589 enum).
- Pause integrates with EXISTING graceful-stop machinery: `quiesce_executor`/`release_hold` (HOLD_REQUESTED level + MOTION_PARKED ack), cycle-start via `ControlState::resumes_on_cycle_start` / `cycle_start()`. `program_stop_cycle` (~2520) is the 0x86 flush. `program_end`/`run_dwell` are synchronized drains via `wait_for_motion_idle`.
- ok for M0/M1/M6 follows normal char-counting (emitted when the command executes from the consumer, like Dwell/ProgramEnd — NOT deferred at parse time).

**Coolant pattern = mirror SpindleController exactly:**
- `firmware-core/src/spindle.rs` is the template: pure controller over `DigitalOut` traits, recording-mock host tests.
- `Overrides` (protocol.rs ~1746) ALREADY has `flood`/`mist` bools for the 0xA0/0xA1 REALTIME toggles — separate from the M7/M8/M9 MODAL group-8 path.
- `firmware/src/spindle.rs` `GpioOut` (polarity-parameterized DigitalOut) is the wiring template. Coolant GPIO is hardware-gated (no GPIO budgeted) — leave peripheral binding a clearly-marked stub.
- Safety chokepoints: `force_spindle_off()` (~1262), `emit_alarm` (~1237), `reset_pipeline`, `program_end`, `program_stop_cycle`, `handle_sleep` — coolant-off must mirror these.

**IMPLEMENTED 2026-06-23 (host-tested + Xtensa compile-verified):**
- Parser: `Plane{XY,ZX,YZ}`, `CoolantState{mist,flood}`, `PlannerCommand::{ProgramPause{optional,tool_change},Coolant}`, Arc gained `k`+`plane`. ModalState gained plane/coolant/pending_tool. M0/M1/M6 in Group::Stop; M7/M8/M9 in new Group::Coolant; G17/18/19 set plane. T-word now records pending_tool; M6 consumes it.
- Planner: plane-aware arcs via `PlaneAxes{p0,p1,normal}` (G17=XY+Z/IJ, G18=ZX+Y/KI, G19=YZ+X/JK); `X_AXIS/Y_AXIS/Z_AXIS` consts added. ArcInProgress generalized (center in p0/p1 subspace, normal_start/delta helix).
- coolant.rs (firmware-core): `CoolantController<Mist,Flood>` mirrors SpindleController; recording-mock tests.
- protocol: `RealtimeCommand::ToggleOptionalStop` (0x88); `$G` now reports live plane (ParserPlane) + coolant (ParserCoolant), no longer hardcoded G17/M9.
- comms.rs: `run_program_pause`/PauseOutcome (drain via wait_for_motion_idle, Hold(false), await PAUSE_RESUME nudge on `~`; PAUSE_ACTIVE flag). OPTIONAL_STOP_ENABLED gates M1 (default off, 0x88 toggles). Coolant: COOLANT_STATE/COOLANT_UPDATE/COOLANT_ESTOP + `coolant` task + force_coolant_off() at all 5 chokepoints. sync_coolant_from_modal mirrors sync_spindle_from_modal.
- firmware/coolant.rs: GPIO STUB (no pin budgeted) — CoolantPin::set is a defmt-trace no-op; replace body with esp_hal Output when a driver stage exists. Wired in main.rs (COOLANT StaticCell + spawn).
- Counts: firmware-core 517 tests pass (-D warnings). Xtensa `just build` + `--features defmt` clean. NOTE: `-D warnings` env var alone breaks Xtensa link (drops -Tlinkall.x); add `-C link-arg=-Tlinkall.x` to test CI-parity.

**M6 Tool-state extension (2026-06-23, follow-up):** M6 now surfaces grblHAL `Tool` state, not Hold:0.
- protocol: `MachineState::Tool` (token "Tool", no substate → `<Tool|...>`), `ControlState::Tool` + `tool_change()` ctor. `cycle_start`/`resumes_on_cycle_start`/`motion_allowed`/`machine_state` all include Tool (resumes to Normal). M0/M1 STILL use Hold(false). `ParserSnapshot.tool: u16` → `$G` renders `T<n>` (was hardcoded T0).
- gcode: ModalState gained `current_tool`; M6 COMMITS pending_tool→current_tool (clears pending). `Parser::set_current_tool()` seam.
- RESET SEMANTICS (matches grbl): current_tool PERSISTS across M2/M30/soft-reset/0x86 — all 3 `*parser=Parser::new()` sites (reset_pipeline/program_end/program_stop_cycle) snapshot+restore via set_current_tool. Pending (uncommitted) T is dropped.
- comms: run_program_pause picks `tool_change()` for M6, `feed_hold()` for M0/M1. status_responder sources Tool via machine_state() (no change needed). `~` handler already gates on resumes_on_cycle_start (now covers Tool).
- WIRE STRINGS: `<Tool|MPos:0.000,0.000,0.000,0.000|...>` ; `$G` = `[GC:G0 G54 G17 G21 G90 G94 M5 M9 T<n> G49 F0 S0]`.
- M6 [MSG:] NAMES the tool: `ResponseWriter::tool_change_message(out, tool)` (host-tested) → "Manual tool change to T<n> — swap tool, then cycle-start (~) to resume" (T0 → "(no tool selected)"). Human-readable only; skirnir does NOT parse it. run_program_pause takes `current_tool` arg.
**Review fixes (2026-06-23, round 2):**
- #3 FIXED (was the KNOWN LIMITATION): `$`-QUERIES NOW SERVICED during M0/M1/M6 hold. `SystemCommand::is_readonly_query()` (host-tested) gates which: Help/$$/$I/$G/$#/$N/$ES/$EG/$EE/$EA/$SED/$PBX = readonly; all writes/actions ($n=val,$X,$C,$SLP,$H,$N0=,$PBX=,$RST,Unknown) = NOT. `hold_until_resume` in comms uses `LINE_QUEUE.try_peek()` (Line is Clone) + `ready_to_receive()`: peeks head, if readonly-$-query → try_receive+service_held_query (handle_system_command, report+ok, clears error_hold), loops; else LEAVES line queued (runs in order after `~`, no reorder) and parks on signals only (no busy-spin). `?` always worked (separate status_responder). run_program_pause now takes parser/state/flash; plan_gcode_line threaded flash too.
- #2 FIXED: T-word now `tool_number(value)` validates non-neg integer in u16 range; T1.5/T-1/T70000 → `GcodeError::BadToolNumber` (error:23, NEW code w/ ERROR_CODES row + $EE). Was silent coerce `word.value.max(0.0) as u16`.
- #7 FIXED: planner `offset_for_axis(axis,i,j,k)` free fn shared by PlaneAxes::offsets + has_offset (was duplicated I/J/K→axis closure).
- Counts: firmware-core 520 tests (-D warnings). Xtensa just build + -D warnings(+linkall) + --features defmt all clean. NOTE: skirnir is concurrently mid-edit by another agent (ConnectionState error in skirnir is NOT firmware work) — test `-p firmware-core -p galdr-proto` to isolate.
