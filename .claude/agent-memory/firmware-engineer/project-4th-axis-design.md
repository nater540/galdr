---
name: project-4th-axis-design
description: DOC-10 (proposed) 4th-axis rotary A design — feed-mode gap, two-distinct-lengths subtlety, grblHAL deg-as-mm convention, $376 rotary-mask, phased plan. Design reviewed/resolved 2026-06-20, no code yet.
metadata:
  type: project
---

`docs/4th-axis-rotary-design.md` (created 2026-06-20) is a REVIEW-STAGE design doc, slated to become DOC-10.
Extends the planner from 3 linear axes to **exactly 4** (axis 3 = A, rotary about X, degrees) with TRUE
coordinated rotary (simultaneous 4-axis interpolation). Axis count hardcoded to 4, NOT generalized to N/6.

**Why:** risky-core design the user wanted reviewed before any code is written. No `src/` changes were made.

**Key facts I verified in the real code (load-bearing for the eventual implementation):**
- **No G93/G94 feed-mode modal group exists today.** Parser `apply_g_word` has no `93`/`94` case; `$G`
  hardcodes the literal `G94` token at `protocol.rs:1359`. Coordinated rotary REQUIRES G93 inverse-time to be
  designed in (G94 feed is ambiguous when a move mixes rotary+linear travel).
- **grblHAL convention adopted: rotary degrees are carried as "mm"** through all planner velocity/accel math.
  `$103`=steps/deg plays `$100`'s role, `$113`=deg/min plays `$110`, `$123`=deg/s² plays `$120`. Keeps DDA/
  RMT/look-ahead unit-agnostic. Rotary-ness is decided by the RUNTIME `$376` rotary-axes bitmask (default `8`,
  bit 3 → A), plumbed into `PlannerConfig::rotary_mask` + `is_rotary(axis)` — NOT a const. It gates FOUR
  places: path-length classification, G93/G94 feed convention, soft-limit/rollover, AND G20/G21 inch-scaling
  suppression. (`DEFAULT_ROTARY_MASK` const is only the power-on default.)
- **THE bug trap (R2): two distinct "length" quantities.** `Block::millimeters` becomes the GOVERNING PATH
  LENGTH (linear Euclidean norm for any linear move, else `|Δa|` for pure-rotary) — replaces the 3-term sqrt
  at `planner.rs:670`. But the `unit_vec` DIRECTION norm uses the ALL-AXIS norm `sqrt(Δx²+Δy²+Δz²+Δa²)` (deg
  as mm) per grbl. These are DIFFERENT numbers; conflating them is the likeliest implementation bug.
- **Most layers already axis-count-generic** and widen for free on an `AXES` 3→4 bump: `Block`, segment gen
  (`StepEvent{step:[bool;AXES]}` + Bresenham DDA), RMT sink (`firmware/src/motion.rs` channels/dir/scratch all
  `[_;AXES]`, ch3 is the documented spare), `limiting_acceleration`, `axis_rate_limit_mm_s`, `resolve_target`,
  `soft_limit_violation`, `AXIS_COUNT` (`protocol.rs:80` aliases `planner::AXES`), settings `axis_span()`
  (auto-enumerates `$103/$113/$123/$133` — NO new descriptor entries). `coords.rs:421` has `assert!(AXES==3)`.
- **Proto is flat `_x/_y/_z` (deliberately not `repeated`)** — extend by APPENDING `_a` fields with fresh tags
  (append-only = wire-compatible, `sanitize` fills A defaults, no flash version bump). `$376` `rotary_mask` is
  also an appended proto field (uint32) + a NEW scalar `SETTING_DESCRIPTOR` (`AxisMask`, `axis_span()==1`).
- **Soft-limit (RESOLVED, $376 model):** an axis marked rotary in `$376` is continuous/rollover — soft limits
  disabled, position mod 360°·$103, and its `$133` is IGNORED (`$133` keeps a single meaning: a hard travel
  bound for non-rotary axes only). Rotary axis is NOT homed ($H skips it — DOC-06 touchpoint).
- **Arcs:** A is NOT an arc plane; slaved linearly across G17 arc segments exactly like the existing Z helix
  (`planner.rs:884`). Rotary-plane arcs explicitly out of scope.

**User's review decisions (2026-06-20), all folded into the doc:** Q1 RESOLVED keep mm-named fields (index 3 =
degrees, doc-comment only). Q3 RESOLVED G93 over-speed = clamp/finish-slower + `[MSG:…]` advisory (no reject).
Q5 RESOLVED *against* my original recommendation — adopt the grblHAL `$376` rotary-axes bitmask (runtime,
default 8) instead of the `$133==0` convention; it consolidates rollover + inch-scaling-suppression into one
setting and subsumes Q4. Q2 (A-as-dominant-DDA step budget) still open, awareness-only. `$376` work lands in
Phase 2 (settings/proto) so Phase 3 kinematics read `config.is_rotary()`. Phased order: parser+feed-mode →
settings/proto(+$376) → planner kinematics (TDD) → protocol/skirnir → firmware ch3/TMC node3 compile-only.

See [[project-planner-contracts]], [[project-motion-contracts]], [[project-settings-contracts]],
[[project-hardware-map]].
