---
name: project-planner-contracts
description: Galdr motion planner (DOC-05) design decisions that the downstream motion executor (DOC-02) must honor — arc back-pressure, squared speeds, entry-speed semantics.
metadata:
  type: project
---

The motion planner lives in `crates/firmware-core/src/planner.rs` (implemented 2026-06-16, pure sync,
no_std, host-tested). Decisions a future subsystem author needs that are NOT obvious from the code:

**`PlannerCommand` stayed in `gcode.rs`** (not relocated to a shared `command` module). The parser is
its sole producer/owner; the planner consumes via `crate::gcode::PlannerCommand`. Relocating would
churn 60+ passing parser tests for no structural gain. Revisit only if a third stakeholder appears.

**Arc back-pressure is all-or-nothing.** `plan_arc` pre-checks `queue.len() + segments > BLOCK_QUEUE_LEN`
and returns `PlannerError::QueueFull` WITHOUT enqueuing anything or mutating `position_steps`. A pure-sync
planner cannot yield mid-arc to let the executor drain. **Why:** DOC-05 warns an over-fine `$12` starves
the 32-block queue; this surfaces it as recoverable back-pressure, never lost geometry. **How to apply:**
the async planner task (firmware bin, not yet written) owns the drain-and-retry loop — on `QueueFull` it
must pump the motion executor to free blocks, then re-issue the SAME arc command. A 10 mm-radius quarter
arc at default `$12`=0.002 needs ~79 segments > 32, so this path is hit in normal use, not just edge cases.

**Speeds are stored squared throughout** (grbl's `entry_speed_sqr` model). `Block.entry_speed_sq`,
`nominal_speed_sq`, `max_entry_speed_sq` are all (mm/s)². The reverse/forward passes are sqrt-free
(`v² = v_next² + 2·a·d`); `Block::entry_speed()`/`nominal_speed()`/`max_entry_speed()` sqrt lazily for the
executor. **How to apply:** the DOC-02 segment generator reads these squared fields; do not re-derive.

**Entry-speed semantics (subtle, cost a test rewrite):** a LONE block has entry=0 — it starts at rest (no
previous block → junction speed 0) AND stops at rest (reverse pass forces zero exit on the newest block).
The cruise/nominal speed is reached MID-block by the segment generator, never at the boundaries. Don't
assert "lone block entry == nominal"; that is wrong physics.

**Planner owns all geometry** the parser deliberately skipped: G20/G21 unit scaling (`MM_PER_INCH=25.4`),
G90/G91 distance resolution, G92 work-offset application, steps/mm rounding (`round(mm × $100..102)`),
and arc subdivision (`acosf(1 − $12/r)` chord-tolerance). Planner works in step space; unit vec + mm
travel are derived back from the step delta for cornering/ramp math. Per-axis accel/rate limits
(`$120..122`/`$110..112`) are applied along the unit vector (grbl per-axis limiting).

**Arc planning is O(n), not O(n²) (fixed 2026-06-16 #8).** `plan_arc` now calls `enqueue_move` (build+enqueue
+advance junction/position state, NO look-ahead) per segment, then `recalculate()` EXACTLY ONCE after the loop.
`plan_line` = `enqueue_move` + `recalculate()` (single move keeps immediate look-ahead). Equivalence is exact
because reverse/forward passes always sweep the WHOLE queue — only final queue state matters (host test
`recalculate_once_equals_recalculate_per_segment` proves it). The all-or-nothing QueueFull pre-check guarantees
`enqueue_move` can't hit QueueFull mid-arc.

See [[project-galdr-overview]], [[project-build-constraints]], [[project-motion-contracts]].
