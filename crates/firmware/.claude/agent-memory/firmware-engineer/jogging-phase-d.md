---
name: jogging-phase-d
description: Phase D `$J=` jogging + `0x85` jog-cancel — parse isolation, jog-block tagging/flush, cancel via feed-hold reuse, ControlState::Jog
metadata:
  type: project
---

Phase D of the grblHAL streaming work: `$J=` jogging + real-time `0x85` jog cancel. Builds on Phase A
(ControlState, feed-hold block-boundary stop), B (coords work→machine, `resolve_target`, `$20`/`$130-132`
soft-limit fields live in `Settings`, NOT PlannerConfig), C (`sync_position` position-sync mechanism). Landed on
branch `firmware/core-pipeline-and-streaming`.

**Jog parse (`gcode.rs`, host-tested):** `Parser::parse_jog(&self, line) -> Result<JogCommand, GcodeError>` uses
grbl's **seed-from-current-then-discard**: a throwaway `ModalState` is SEEDED from `self.state` (so a program in
G91/G20 makes `$J=X10` incremental/inch) but NEVER written back — `parse_jog` takes `&self`, so the persistent
`gc_state` is provably untouched (asserted in a test). Accepts only `G90/G91/G20/G21/G53` + X/Y/Z + `F`; any
other word → `UnsupportedCommand` (20). New errors: `FeedRateUndefined` (code 22, missing F),
`JogNoAxis` (code 23, no axis word — same wire code as `ProbeNoAxis`, distinct variant for call-site clarity).
`JogCommand { axes, distance_mode, units, feed, machine_coords }`.

**Jog as cancelable motion (`planner.rs`):** added `jog: bool` to `Block` (threaded through
`plan_line`/`enqueue_move`/`build_block` as a new param — every existing Block literal + call site needed
updating, incl. firmware/src/motion.rs `probe_block_to`). `Planner::plan_jog(&JogCommand, Option<SoftLimits>)`
resolves target via the SAME `resolve_target` work→machine path (honors G53), tags the block `jog=true`, runs
look-ahead. `SoftLimits { max_travel_mm: [f32;3] }`; envelope is `[-max_travel, 0]` per axis (grbl homes to
machine-zero at the positive end → machine coords ≤ 0). Free fn `soft_limit_violation(target, steps_per_mm,
max_travel_mm)` host-tested, with a 1-step rounding tolerance. Rejection → `PlannerError::JogExceedsTravel`
(code 15, grbl "travel exceeded"). `Planner::flush_jog_blocks() -> usize` drains trailing jog blocks ONLY
(pops from the BACK while `back().jog`), drops trailing junction state so a post-cancel move corners from rest.

**ControlState::Jog (`protocol.rs`):** new variant. `machine_state(Jog, running)` → Jog while running, Idle when
drained (derived from live execution like Normal, never latched — no reporter/executor race). `jog_allowed()` =
Normal|Jog. `begin_jog()` Normal/Jog→Jog. `cancel_jog()` Jog→Normal (no side-effects — a jog never changed
modal/coord state). `soft_reset` from Jog → boot/Normal (so `0x18` clears jog). `motion_allowed()` still
excludes Jog. Status formatter's `_ => {}` arm already emits no substate for Jog.

**Firmware wiring (`comms.rs`):** `$J=` routed in `handle_line` BEFORE the `$`-dispatch via `strip_jog_prefix`
(case-insensitive `J`, literal `$`/`=`) → `handle_jog`. `handle_jog` gates: error-hold, then `jog_allowed()` and
(Normal + `program_running()`) reject from a running PROGRAM with error:3; parse → plan_jog → `begin_jog` +
`BLOCK_AVAILABLE`; one `ok`. A jog parse error does NOT arm the gcode error-hold (independent of program stream).
`current_soft_limits()` builds `Option<SoftLimits>` from live `$20`/`$130-132`. `refresh_jog_state()` drops a
stale Jog latch→Normal once drained (called atop both jog + gcode plan paths); program GCode is rejected while
Jog is active. `0x85` in `dispatch_realtime`: signals dedicated `JOG_CANCEL` ONLY when `ControlState::Jog`.
Consumer races `JOG_CANCEL` in the `select4` loop → `cancel_jog_cycle`.

**Cancel = feed-hold reuse + Phase-C sync (the key design):** `cancel_jog_cycle` (1) flushes jog blocks under the
planner lock FIRST, (2) signals `FEED_HOLD` ONLY if `EXECUTOR_RUNNING` (avoids a stale FEED_HOLD spuriously
pausing the next motion) and polls until the in-flight block finishes (raced vs SOFT_RESET), (3) syncs planner to
`read_live_position()` via `Planner::sync_position` (the Phase-C mechanism), (4) signals `CYCLE_START` (only if it
held) and `cancel_jog()`→Normal. `0x18` dispatch drains a pending `JOG_CANCEL`. **APPROXIMATION:** stops at the
current block boundary (per-block granularity, reusing feed-hold) — `// TODO(DOC-02 Stage-2): mid-block
jog-cancel ramp-down` marks it in `cancel_jog_cycle`, same follow-up as the feed-hold smooth-ramp.

Verified: `cargo test -p firmware-core` 311 green + clippy-clean; Xtensa firmware build clean (only pre-existing
`drop(async_flash)` lints remain). See [[build-test-commands]], [[streaming-state-sharing]],
[[coordinate-model-phase-b]], [[probe-cycle-phase-c]].
