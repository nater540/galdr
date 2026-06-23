---
name: project-planner-contracts
description: Galdr motion planner (DOC-05) design decisions — RESUMABLE arc back-pressure (2026-06-23 deadlock fix), squared speeds, entry-speed semantics, planner-owns-geometry.
metadata:
  type: project
---

The motion planner lives in `crates/firmware-core/src/planner.rs` (implemented 2026-06-16, pure sync,
no_std, host-tested). Decisions a future subsystem author needs that are NOT obvious from the code:

**`PlannerCommand` stayed in `gcode.rs`** (not relocated to a shared `command` module). The parser is
its sole producer/owner; the planner consumes via `crate::gcode::PlannerCommand`. Relocating would
churn 60+ passing parser tests for no structural gain. Revisit only if a third stakeholder appears.

**Arc back-pressure is RESUMABLE / incremental (fixed 2026-06-23 — replaced the old all-or-nothing deadlock).**
The previous `plan_arc` did `if queue.len() + segments > BLOCK_QUEUE_LEN { return QueueFull }` — all-or-nothing.
That DEADLOCKED any arc that subdivides into MORE than `BLOCK_QUEUE_LEN`(=32) segments: from an empty queue
`0 + segments > 32` returns QueueFull unconditionally, forever, and the comms retry loop spins, never acks, the
host send-window fills, stream dead. Real-world trigger: `T1_Test.tap` arc `G2X39.499Y48.894I-15.898J10.840`
(radius ~19.24mm, sweep ~39.4°) = 48 segments at default `$12=0.002`. Now `plan_arc` feeds segments
INCREMENTALLY: it saves an `ArcInProgress` (center/radius/theta_start/theta_step/segments/next_seg/z+a interp/
seg_feed/units/feed_mode) and the new `enqueue_arc_chunk` enqueues as many remaining segments as fit, recalc once
per chunk, advances `next_seg`. Returns `PlannerOutcome::Queued{blocks}` when the whole arc fit (complete) or the
new `PlannerOutcome::ArcPending{enqueued}` when partial. New pub API: `resume_arc()` (idempotent no-op
`Queued{blocks:0}` when no arc), `arc_pending()`, `abort_arc()` (soft-reset drops in-progress arc — though
`Planner::new` rebuild already nulls it). **Chord accuracy preserved:** segment count is still
`arc_segment_count`, only chunked. **How to apply:** the comms consumer (`drive_pending_arc` in comms.rs) drives
it — on `ArcPending` it wakes the executor (`BLOCK_AVAILABLE`), yields `QUEUE_FULL_RETRY` (racing `SOFT_RESET`),
calls `resume_arc`, loops until `Queued`, acks ONCE. Never returns `QueueFull` for arcs anymore. `QueueFull` from
the planner now only comes from single moves / `plan_go_to_predefined` (still all-or-nothing, correctly).

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

**Arc planning is O(n) per chunk, not O(n²) (fixed 2026-06-16 #8; chunked 2026-06-23).** `enqueue_arc_chunk`
calls `enqueue_move` (build+enqueue+advance junction/position state, NO look-ahead) per segment, then
`recalculate()` EXACTLY ONCE after the chunk. `plan_line` = `enqueue_move` + `recalculate()` (single move keeps
immediate look-ahead). Equivalence is exact because reverse/forward passes always sweep the WHOLE queue — only
final queue state matters (host test `recalculate_once_equals_recalculate_per_segment` proves it). The
`queue.len() < BLOCK_QUEUE_LEN` free-slot guard in the chunk loop is the termination backstop — a full queue
enqueues nothing and returns `ArcPending{enqueued:0}` rather than ever hitting QueueFull mid-chunk.

See [[project-galdr-overview]], [[project-build-constraints]], [[project-motion-contracts]].
