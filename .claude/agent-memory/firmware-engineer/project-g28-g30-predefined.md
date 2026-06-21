---
name: project-g28-g30-predefined
description: G28/G30 predefined-position MOVE execution — host-tested planner method, consumer interception, machine-coord recall semantics
metadata:
  type: project
---

G28/G30 predefined-position recall is now IMPLEMENTED (was a no-op TODO). DOC-05 group-0 motion, not DOC-06 homing.

**Design (host-testable, the key choice):** sequencing lives in `Planner::plan_go_to_predefined(intermediate,
units, distance, predefined: [f32; AXES], limits) -> Result<usize, PlannerError>` in
crates/firmware-core/src/planner.rs (unit-tested, 9 cases incl. the atomic-back-pressure regression). The consumer
reads the stored position and passes it in — the planner does NOT own the coordinate model.

**Why intercept in the consumer instead of the generic plan_command path:** the predefined position lives in the
consumer-owned `CoordinateSystems` (`coordinates().predefined(index)`; index 0 = G28, 1 = G30), so the planner can't
plan it alone. `handle_go_to_predefined` (comms.rs) is intercepted in the line handler BEFORE `plan_command`,
mirroring its lock-scope + QueueFull-retry + soft-reset-abort + BLOCK_AVAILABLE flow. The planner's pass-through
`PlannerOutcome::GoToPredefined` arm is now unreachable from the wired path (kept for the host test).

**Semantics decided (grbl-faithful):** (1) no axis words => single RAPID to stored MACHINE position, all axes incl A;
never-stored slot defaults to origin (`predefined` returns zeros). (2) with axis words => intermediate rapid first
(work coords, honoring units/distance, unspecified axes hold), THEN machine-coord recall rapid (absolute, mm, WCO
bypassed like G53). Recall built straight to step target (no resolve_target — already absolute machine mm). (3) $20
soft-limit applies per sub-move like any rapid; recall point is within-envelope by construction, only intermediate
can ALARM:2. (4) QueueFull back-pressure is ATOMIC (all-or-nothing, like the arc path): plan_go_to_predefined
resolves both step targets up front (pure), counts needed blocks (0/1 each via target!=projected), and pre-checks
`queued_len()+needed > BLOCK_QUEUE_LEN` → returns QueueFull BEFORE enqueuing anything. NOT mere idempotency: a
partial enqueue (intermediate committed, recall hits QueueFull) would DOUBLE-apply the increment on a G91 retry
(resolve_target adds to the advanced position) — silent double motion. Regression test
`predefined_incremental_back_pressure_is_atomic_no_double_count` guards it. (My first cut claimed "idempotent" and
was WRONG for G91 — caught in review; fixed with the capacity pre-check.)

**How to apply:** when touching G28/G30, keep sequencing in the host-tested planner method; the consumer only reads
the position + drives back-pressure. G28.1/G30.1 STORE was already done (CoordinateOp::StorePredefined).

**Adjacent panic fix (same change):** `axis_values_mm` (comms.rs) used a 3-element `[x,y,z]` literal with a
`0..AXES` (=4) loop → `words[3]` OOB panic on any G92 / G10 L2 / L20 line with an A word. Fixed to include `axes.a`
and take a `rotary_mask` param so a rotary-A offset word is DEGREES (scale 1.0, never inch-scaled) per DOC-10.1 —
matching `resolve_target`'s per-axis scale fork. Mask sourced from `settings_snapshot().rotary_mask` in
`apply_coordinate_op`. This was the ONLY 3-vs-4 axis array literal in the firmware crate.

**Hardware-gated:** these rapids can't be bench-verified here (compile-only on Xtensa; `just build` passes). G28/G30
share the normal RMT step path so the same homing/spindle bench caveats apply.
