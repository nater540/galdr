---
name: t1-test-stall-investigation
description: T1_Test.tap line-401 on-HW motion stall is NOT in the pure parse→plan→segment pipeline; points at the core-1 executor / RMT boundary
metadata:
  type: project
---

On-hardware streaming of `T1_Test.tap` (repo root, 500-line Vectric contour) stalls at line 401 (no more `ok`, host
window fills). Investigated 2026-06-24.

**Ruled OUT — the pure firmware-core pipeline is innocent.** The real parse→plan→segment path was driven end-to-end
off-target with the DEVICE default settings (`PlannerConfig::default()` = 250 steps/mm etc., `MotionConfig::default()`
= 1 MHz tick / `$0`=10 / 2µs min-low — confirmed matching `firmware/src/main.rs` `MOTION_TICK_HZ` + `motion.rs`
`with_clk_divider(80)`). Two regression tests in `crates/firmware-core/src/planner.rs` tests mod:
`t1_test_line_401_region_streams_without_stalling` (inlined 398–402 slice, self-contained) and
`t1_test_full_file_streams_without_stalling` (`include_str!` the whole file). Both faithfully model the consumer:
plan each line, block-and-retry on QueueFull, drive resumable arcs via `resume_arc`, realize every popped block
through `SegmentGenerator::run_block`, with a finite iteration budget so any hang = test FAILURE and a sink that flags
NaN/Inf/0/oversized periods. **Both pass.** No degenerate segment, no NaN/Inf, no never-completing block, no permanent
QueueFull, no unbounded loop. Worst arc subdivides to 79 segments (line 458), worst move ~20k steps — all bounded. The
junction-deviation, arc-subdivision, and Bresenham guards are all present and correct.

**Why it's the HW boundary, not software.** The consumer's QueueFull retry (`comms.rs` `plan_command` ~L1890) and
`drive_pending_arc` (~L1934) both race their signal wait against `Timer::after(QUEUE_FULL_RETRY)`, so a missed
`SLOT_FREED`/`BLOCK_AVAILABLE` can't deadlock them — they re-poll. The only way the queue stays full forever (→ no
`ok` → host stalls) is the **core-1 motion executor ceasing to pop blocks**: RMT TX `wait()` never completing, the
`join3` across ch0/1/2 wedging, or a second-core fault. See [[project-motion-executor]],
[[project-rmt-clock-and-tx-completion]], [[project-second-core-reentrancy]] (Xtensa stack-top ABI headroom). Next step
is bench instrumentation of the executor (does `BLOCK_AVAILABLE`/`SLOT_FREED` fire after line 401's block? does RMT
`wait()` return?), not more host tests.
