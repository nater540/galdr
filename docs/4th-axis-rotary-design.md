# 4th-Axis Rotary (A) Design — Coordinated Rotary About X

> **Doc status:** Design proposal, **revised 2026-06-20 after a grblHAL-grounded review** (branch
> `feature/4th-axis-rotary`). **Slated to become DOC-10** in the `docs/00-architecture.md` set once approved.
> This document is the TDD spec the implementation follows. **Phases 1–2 are now implemented** on
> `feature/4th-axis-rotary` (host-tested green: 456 firmware-core tests, firmware compiles for Xtensa): the
> G93/G94 parser feed-mode + native inverse-time execution (Phase 1), and the `AXES` 3→4 bump with `$376`,
> the `_a` proto/settings surface, `AxisWords.a`, and the 4-field protocol (Phase 2). Bumping `AXES` pulled the
> bulk of the Phase-3 kinematics forward as well, since they're the same `0..AXES` widening: the single all-axis
> norm + 4-term junction (DOC-10.3), the `$376`-gated G20/G21 units fork (DOC-10.1), arc A-slaving (DOC-10.5),
> and the rotary soft-limit exemption (DOC-10.6). The firmware ch3/A-DIR/A-limit wiring (Phase 5) is compile-only
> with PROVISIONAL GPIOs (18/38/39), bench-unverified. Still open: the optional modulo-360 rotary **position
> rollover** (DOC-10.6, deferred as safe-to-skip) and TMC node-3 hardware bring-up. (The mixed-G94→inverse-time
> conversion — earlier mis-noted as "needs no further work since `millimeters` is the full norm" — is **now
> implemented** in `planner.rs::resolve_feed`; keeping the full-norm `millimeters` was only *half* of grbl's
> `ROTARY_FIX`, the feed leg was the other half. See DOC-10.2.)
>
> **2026-06-20 review corrections (all folded in below):** (1) DOC-10.2 now matches grblHAL's *actual* mixed
> linear+rotary G94 handling — its `ROTARY_FIX` inverse-time conversion keeping `Block::millimeters` as a single
> all-axis norm — instead of the earlier (incorrect) "grbl computes a linear-only path length" claim; this
> **resolves R2**. (2) Rotary modulo-360 rollover (DOC-10.6) is owned as a deliberate Galdr divergence —
> grblHAL does **not** wrap rotary position. (3) `$376` default/max corrected (DOC-10.7): grbl's own default is
> 0, Galdr's fresh default is 8 via `MachineSettings::default()` (not a zero-fill, which would forbid `$376=0`);
> max is 8 (only bit 3 meaningful). (4) The G93 F-required rule exempts G0 rapids.
>
> **Scope:** extend the Galdr motion pipeline from 3 linear axes (X/Y/Z) to **exactly 4 axes** with a *true
> coordinated rotary* A axis (A rotates the work about the X axis, fully interpolated with X/Y/Z). Axis count
> is hardcoded to 4 — this is **not** a generalization to N/6 axes. Axis index 3 = A, rotary, units degrees;
> the linear axes stay millimeters.

## TL;DR

- Add a fourth axis everywhere the planner/parser/protocol already key off `planner::AXES` (currently
  `pub const AXES: usize = 3` at `crates/firmware-core/src/planner.rs:39`). Bump it to `4`. Most per-axis
  arrays (`[T; AXES]`) widen for free; the **load-bearing changes are kinematic**, not structural.
- The rotary axis is carried through the planner's velocity/acceleration math in its native unit — **degrees
  treated as "mm"** — exactly as grbl/grblHAL do. The runtime **`$376` rotary-axes bitmask** (default `8` →
  bit 3 = A) is the source of truth for rotary-ness, plumbed into `PlannerConfig`; it gates the four places
  where the distinction matters: **path-length computation**, the **G93/G94 feed convention**,
  **soft-limit/rollover**, and **G20/G21 inch-scaling suppression** — one setting drives all of them.
- The single largest new piece is **RS274/NGC feed semantics**: today there is no G93/G94 modal group at all
  (the `$G` report hardcodes `G94` at `crates/firmware-core/src/protocol.rs:1359`, and the parser has no
  `93 =>`/`94 =>` case). Coordinated rotary *requires* **G93 inverse-time** to have well-defined feed on any
  move that mixes rotary and linear travel; G94 alone is ambiguous for such moves. Both modes are specified
  here. **Mixed G94 moves are internally converted to inverse-time** (grbl's `ROTARY_FIX` model), so
  `Block::millimeters` stays a single all-axis norm and the G93 path is reused — see the DOC-10.2 review
  correction.
- The DDA/segment generator (`crates/firmware-core/src/motion.rs`) and the RMT executor
  (`crates/firmware/src/motion.rs`, RMT ch3 currently spare) are **already axis-count-generic** — they index
  `0..AXES` and size scratch buffers by `AXES`. They widen with the `AXES` bump and a 4th RMT channel + DIR
  pin; no algorithmic change.
- Protocol widens to 4-field `MPos`/`WPos` and `[AXS:4:XYZA]` — this is grblHAL-native N-axis behavior, and
  `skirnir` (our only client) updates to match. The test fallout is mechanical but broad.

---

## DOC-10.0: Why this is the risky core

The planner is the one subsystem where the 3→4 change is **not** mechanical. Three pieces of its math are
hardcoded to three linear, homogeneous (all-mm) axes:

1. **Euclidean path length** — `planner.rs:670`:
   ```text
   let millimeters = libm::sqrtf(delta_mm[0]² + delta_mm[1]² + delta_mm[2]²);
   ```
   Summing a degree component into a millimeter norm is dimensionally meaningless. This is the central design
   question (DOC-10.2).
2. **Junction-deviation cornering** — the 3-term dot product at `planner.rs:741`:
   ```text
   let dot = prev[0]*unit_vec[0] + prev[1]*unit_vec[1] + prev[2]*unit_vec[2];
   ```
   Whether the rotary component participates in the junction unit vector (DOC-10.3).
3. **Feed rate** — the planner derives `nominal_speed` from `feed / distance` (`nominal_speed_mm_s`,
   `planner.rs:703`). With a rotary axis, "distance" and "feed" are unit-ambiguous unless feed mode is
   defined (DOC-10.2).

Everything else (`steps: [i32; AXES]`, `unit_vec: [f32; AXES]`, `resolve_target`, the reverse/forward
look-ahead passes, the soft-limit envelope) already loops `0..AXES` and widens cleanly.

---

## DOC-10.1: Axis model & units convention

### The grblHAL convention: rotary degrees are "linear mm"

grbl and grblHAL do **not** carry a separate rotary kinematic model. A rotary axis is fed through the
identical planner as a linear axis, with **one degree treated as one millimeter** for all velocity and
acceleration math. `$103` (steps/deg) plays the role of `$100` (steps/mm); `$113` (deg/min) plays the role of
`$110` (mm/min); `$123` (deg/s²) plays the role of `$120` (mm/s²). The planner never needs to know the
physical radius of the rotary work — it operates purely in the abstract "distance" space the units define.

This is the convention we adopt. It keeps `Block`, the look-ahead passes, the DDA, and the RMT encoder
**unit-agnostic**: they see step counts and an abstract unit vector, never "mm" vs "deg".

### Proposed constants and types

In `crates/firmware-core/src/planner.rs`:

```rust
/// Number of coordinated axes: X, Y, Z (linear, mm) and A (rotary about X, degrees). Hardcoded at 4 —
/// the firmware is not generalized to arbitrary axis counts (DOC-10).
pub const AXES: usize = 4;

/// Axis index of the rotary A axis (rotation about machine X). Linear axes are 0..3.
pub const A_AXIS: usize = 3;

/// DEFAULT `$376` rotary-axes bitmask — the power-on value before flash loads. Bit N set ⇒ axis N is angular
/// (degrees). **This const is only the default**: the authoritative rotary-ness is the runtime `$376` setting
/// (DOC-10.7), read into [`PlannerConfig::rotary_mask`] and consulted by the three rotary-gated sites. The
/// default = `8` (bit 3 set → A rotary; X/Y/Z linear).
pub const DEFAULT_ROTARY_MASK: u8 = 0b0000_1000;
```

Rotary-ness is read **at runtime from `$376`**, not from a const — so the three rotary-gated sites
(path-length classification (DOC-10.2), the G93/G94 feed convention (DOC-10.2), and soft-limit/rollover
(DOC-10.6)) consult the live mask. `PlannerConfig` (`planner.rs:109`) carries it, plumbed in from settings:

```rust
pub struct PlannerConfig {
  // ... existing per-axis arrays ...
  /// `$376` rotary-axes bitmask: bit N set ⇒ axis N is angular (degrees). Drives BOTH the rotary kinematic
  /// gating (path-length / feed convention / soft-limit-rollover) AND the suppression of G20/G21 inch scaling
  /// on that axis's words (DOC-10.7) — one setting, both behaviors. Default [`DEFAULT_ROTARY_MASK`] (= 8).
  pub rotary_mask: u8,
}

impl PlannerConfig {
  /// Whether axis `axis` is rotary per the live `$376` mask. The planner's rotary-gated sites call this
  /// instead of indexing a const, so toggling `$376` re-classifies an axis at runtime (DOC-10.10 tests this).
  pub fn is_rotary(&self, axis: usize) -> bool {
    self.rotary_mask & (1 << axis) != 0
  }
}
```

The look-ahead/DDA/RMT math stays unit-agnostic per the grblHAL convention and never consults the mask.

`AxisWords` (`crates/firmware-core/src/gcode.rs:332`) gains a fourth field:

```rust
pub struct AxisWords {
  pub x: Option<f32>,
  pub y: Option<f32>,
  pub z: Option<f32>,
  /// A target word (rotation about X), in degrees (or the active linear units' numeric value — grbl applies
  /// G20/G21 inch scaling ONLY to linear axes; a rotary word is never inch-scaled). See DOC-10.2.
  pub a: Option<f32>,
}
```

**Units interaction (G20/G21) — driven by the SAME `$376` mask.** Inch scaling applies to **linear axes
only**. `resolve_target` (`planner.rs:604`) and `arc_endpoint_mm` (`planner.rs:903`) currently multiply every
word by `units_scale`. A rotary word must bypass that scale (a `G20`-active `A90` is still 90 degrees, never
90×25.4). `config.is_rotary(axis)` selects `1.0` instead of `units_scale(units)` per axis when resolving the
word → `value_mm` term. **This is the key consolidation `$376` buys (Q4 ↔ Q5):** in grblHAL the *same*
rotary-axes mask that marks an axis continuous/rollover ALSO suppresses its G20/G21 inch scaling — one setting
drives both behaviors, instead of an `AXIS_IS_ROTARY` const for scaling plus a separate `$133==0` convention
for rollover. This is the only place units handling forks per axis, and it now forks on the live mask.

---

## DOC-10.2: Feed-rate semantics (RS274/NGC)

This is the substantive new logic. **There is currently no feed-mode modal group at all** — confirmed: the
parser has no `93`/`94` G-code case (`gcode.rs` `apply_g_word`), and `$G` hardcodes the literal `G94` token
(`protocol.rs:1359`). Both modes are designed here.

### The G93/G94 modal group (modal group 5)

Add a parser modal group, mirroring how `DistanceMode` (G90/G91) and `Units` (G20/G21) are modeled:

```rust
/// Feed-rate mode (RS274/NGC modal group 5). G94 is the power-on/reset default.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FeedMode {
  /// G94 — feed is units per minute (mm/min for linear travel, deg/min for a pure-rotary move).
  #[default]
  UnitsPerMin,
  /// G93 — inverse time: F is 1/(move duration in minutes); the move takes 1/F minutes regardless of length.
  InverseTime,
}
```

Wiring:
- `gcode.rs` `apply_g_word`: add `93 => { guard.claim(Group::FeedMode)?; next_state.feed_mode = FeedMode::InverseTime; }`
  and `94 => ... FeedMode::UnitsPerMin`. Add a `Group::FeedMode` variant so `G93 G94` on one line is a modal
  conflict, matching the existing group-exclusivity guard.
- `ModalState` (`gcode.rs:~300`) gains `pub feed_mode: FeedMode` (defaults to `UnitsPerMin`), sticky across
  lines like `distance`/`units`.
- `PlannerCommand::Move`/`Arc`/`JogCommand` carry `feed_mode: FeedMode` alongside the existing `feed`/`units`
  fields, so the planner sees the active mode per move (the parser stays a pure function of the line).
- `$G` (`protocol.rs:1359`): replace the hardcoded `G94` token with the live mode (`G93`/`G94`) from the
  snapshot. `MachineSnapshot`/`GcodeStateSnapshot` gains a `feed_mode` field.

**G93 invariant:** every G93 **feed** motion line (G1/G2/G3) MUST carry an `F` word (grbl
error: feed-rate undefined, `Status_GcodeUndefinedFeedRate`). **G0 rapids are exempt** — grblHAL requires the
inverse-time F only for motion that consumes a feed, not for rapids, so a `G93`-active `G0` line needs no F. In
G93 the F is not modal in the usual sense — it describes *this* move's duration and does not carry forward: the
parser keeps the existing `FeedRateUndefined` path (`gcode.rs:731`, code 22) but tightens it so that under G93 a
feed move with no `F` **on its own line** is rejected even if a prior modal F exists. (G94 keeps the current
modal-F fallback.) Implementation note: this needs a per-line "saw an F word this line" flag in the line
accumulator, distinct from the modal `feed` value.

**G93 + G38.x probe — REJECTED (decision, 2026-06-20).** A `G38.x` probe issued while `G93` inverse-time is
modally active is rejected (`GcodeError::ProbeInverseTimeUnsupported`, wire code 22). A probe needs a well-defined
units/min **contact** speed, but inverse-time defines speed as distance ÷ duration, and a probe's distance is the
arbitrary no-contact overshoot — so an inverse-time probe seek speed is meaningless. (And since `G38.x` is
linear-only — the A axis is held, per the 4th-axis probing architecture — inverse-time's rotary-coordination
purpose does not apply.) The operator must switch to `G94` to probe, then restore `G93`. The check sits ahead of
the feed-undefined test, so a `G93` probe reports the probe-specific error regardless of whether an `F` is present
(the rejection is about the feed *mode*, not a missing feed). The distinct `GcodeError` variant shares wire code
22 with `FeedRateUndefined` because grblHAL has no canonical code for this case — the `$EE`/host text reads
"feed rate undefined", a known cosmetic imprecision accepted for the reuse.

**G38.x is LINEAR-ONLY — an A word in a probe is REJECTED (decision, 2026-06-21).** A `G38.x` probe carrying a
rotary `A` word is rejected (`GcodeError::ProbeRotaryAxisWord`, wire code 33 "Invalid target"). No mainstream
controller probes through a rotary axis: probing while A moves rotates the surface normal under a fixed probe
vector (cosine error, invalid tip-radius compensation). The A axis is therefore never part of a probe target —
**any** `A` word is rejected, even a redundant `A` equal to the current position, so the contract is unambiguous.
The check is first in the probe-emit path (ahead of the G93 and feed checks). Rotary center-finding etc. is the
host's job (index A to a fixed angle, hold, then a linear probe) — see the 4th-axis probing architecture.

**`[PRB:...]` reports all four axes (fix, 2026-06-21).** The probe-result push and the `$#` `[PRB:]` line now
render `[PRB:x,y,z,a:flag]` — the A value-at-trigger (the angle a touch happened at) is reported, matching the
four-field live `MPos:`/`WPos:` status. This also widened the whole `$#` block (`[G54:]`…`[G59:]`, `[G28:]`,
`[G30:]`, `[G92:]`) to four axes via a shared `write_axes_csv` helper — previously they silently dropped A. (TLO
stays grbl's legacy single-Z `[TLO:z]` form.) `skirnir` does not parse these field lists yet, so nothing downstream
broke.

### G94 units/min — grbl's inverse-time conversion for mixed moves (modifies `planner.rs:670`)

> **Correction (design review, 2026-06-20).** An earlier draft of this section claimed grbl computes a
> *linear-only* path length, stores it in `Block::millimeters`, and slaves the rotary axis. **That is not
> grblHAL's behavior.** Verified against grblHAL `planner.c`:
> - grblHAL's **default** (`#define ROTARY_FIX 0`) sums the degree delta straight into the all-axis Euclidean
>   norm — `block->millimeters = convert_delta_vector_to_unit_vector(unit_vec)` over *every* axis — i.e. exactly
>   the dimensionally-meaningless thing DOC-10.0 rejects.
> - The behavior we actually want lives in grblHAL only as the **non-default `ROTARY_FIX`** option, and it is
>   implemented by **converting a mixed G94 move to inverse-time** (`feed_rate = 1/(sqrtf(linear_magnitude)/
>   feed_rate); inverse_time = On`), while keeping `block->millimeters` as the **full all-axis norm**.
>
> We adopt grbl's `ROTARY_FIX` model. It reuses the G93 machinery (below), keeps **one** length quantity, and
> deletes the "two distinct lengths" bug class the earlier draft created (was R2).

`Block::millimeters` stays the **full all-axis Euclidean norm** `sqrt(Δx²+Δy²+Δz²+Δa²)` (degrees treated as mm),
the single length the look-ahead, junction, and segment generator already consume. The G94 feed semantics are
expressed entirely through the **nominal speed**, never a second length. Classify a move by its step delta — used
now only to pick the feed rule, never to replace `millimeters`:

- `linear_len = sqrt(Δx² + Δy² + Δz²)` (mm) — Euclidean norm over the **linear** axes only.
- `rotary_len = |Δa|` (deg) — magnitude of the rotary delta.

| Case | Condition | Feed rule | F means |
|------|-----------|-----------|---------|
| **Mixed** (linear + rotary) | `linear_len > ε` and `rotary_len > ε` | convert to inverse-time: `F' = F / linear_len`, mark the block inverse-time; nominal = `millimeters × F' / 60` (the G93 path) | F is mm/min along the **linear** path; wall-clock duration is `linear_len / F`, so A is slaved (sweeps `Δa` in that time). Identical to grbl `ROTARY_FIX`. |
| **Linear-only** | `linear_len > ε`, `rotary_len ≤ ε` | nominal = `F / 60` mm/s (unchanged from today) | F is mm/min; `millimeters == linear_len`. |
| **Rotary-only** | `linear_len ≤ ε`, `rotary_len > ε` | nominal = `F / 60` deg/s | F is deg/min about A; `millimeters == rotary_len` falls out naturally (the full norm of a pure-rotary move *is* `|Δa|`). |
| **Degenerate** | both `≤ ε` | no block (dropped, as today) | — |

The mixed case routes through the **same inverse-time nominal-speed computation** as G93 (next subsection): the
planner sets an effective inverse-time feed and realizes the block in `linear_len / F` minutes. This is why
`X10 A90 F100` and `X10 F100` take the *same time* — the rotary travel is "free" in G94, which is exactly why
synchronized rotary work needs explicit G93. **`Block::millimeters` is unchanged in meaning** (full all-axis
norm); only `build_block`'s *speed* derivation forks, and it forks into code G93 needs anyway.

**[implemented — `planner.rs`]** `build_block` keeps the full `AXES`-term norm as `millimeters` and calls a
`resolve_feed(feed, feed_mode, units, &delta_mm)` helper that returns `(effective_feed, effective_feed_mode)`:

```rust
let (effective_feed, effective_feed_mode) = if rapid {
  (feed, feed_mode)                                  // rapids ignore the feed; the axis-rate limit governs.
} else {
  self.resolve_feed(feed, feed_mode, units, &delta_mm)
};
let nominal_speed =
  self.nominal_speed_mm_s(effective_feed, units, effective_feed_mode, rapid, millimeters, &unit_vec);
```

`resolve_feed` returns the original feed for pure-linear, pure-rotary, and native-G93 moves, and the converted
`F × units_scale / linear_len` (flagged `InverseTime`) for a mixed G94 move — classifying linear vs rotary by the
live `$376` mask (`config.is_rotary`), not a hardcoded axis, and folding `units_scale` in so a G20 mixed move keeps
its inch→mm conversion (the inverse-time branch of `nominal_speed_mm_s` never inch-scales). The nominal-speed math
(next subsection) then handles inverse-time uniformly for both native-G93 and converted-mixed-G94 blocks. Tests
(`planner.rs`): a mixed `X10 A90 F600` runs the linear leg at exactly F600 (= 10 mm/s) with the full norm
unchanged; a pure-rotary `A90 F600` stays `F/60` deg/s; and the converted feed is still floored by the axis-rate
clamp.

### G93 inverse-time — deriving nominal speed from duration

Under G93, `F` directly specifies the inverse of the move duration in minutes:

```text
move_duration_min = 1.0 / F          (F > 0 required, per-line)
move_duration_s   = 60.0 / F
```

The nominal (cruise) speed for the block is then **path length over duration**, where the path length is the
single `Block::millimeters` (the full all-axis norm):

```text
nominal_speed (unit/s) = millimeters / move_duration_s = millimeters × F / 60.0
```

In native G93 the F describes the duration of the *whole* move, so the full norm is the right length; in the
converted mixed-G94 case the effective `F' = F/linear_len` makes `nominal = millimeters × F' / 60`, i.e. the move
finishes in `linear_len / F` minutes — the two paths share this one formula. The result is still a single scalar
speed the existing squared-speed machinery consumes unchanged
(`nominal_speed_sq = nominal_speed²`, capped by the per-axis rate limits exactly as today via
`axis_rate_limit_mm_s` / its rotary analogue).

**Why coordinated rotary needs G93.** Consider engraving a helix on a cylinder: `X10 A360 F___`. In G94, F
governs only the 10 mm of linear travel and A is slaved — but if a later move is *pure* rotary (`A360`, no
X), F abruptly switches meaning to deg/min, and there is no single F that makes the *surface* feed (the
combination of linear advance and rotary surface speed) continuous across the two moves. G93 sidesteps this:
the CAM post-processor computes the desired duration of each move from the true tool-vs-work surface velocity
and emits `1/duration` as F. The firmware then just realizes each move in its commanded time, and the surface
feed is correct by construction. This is why every serious 4-axis CAM post emits G93 for wrapped/rotary
toolpaths. **G94 remains fully supported** (and is the default) for indexing and linear-dominant work; G93 is
the mode that makes *true coordinated* rotary feed well-defined.

### Per-axis rate clamp with a rotary axis

`axis_rate_limit_mm_s` (`planner.rs:719`) computes the largest speed at which no axis exceeds `$11x`. It loops
`0..AXES` over `unit_vec` and `max_rate_mm_min`. With A in the arrays this **already works** — the rotary
component of the unit vector against `$113` (deg/min) yields a deg/s limit that, in the unit-agnostic
"distance" space, correctly clamps the block. No fork needed; only the array widens. (The clamp is applied to
both the G94-derived and G93-derived nominal speed, so a too-fast G93 duration is still floored to what the
hardware can do — the move then runs *slower* than commanded, with a status note, rather than losing steps.)

---

## DOC-10.3: Junction-deviation cornering (3 → 4 terms)

The junction model (`junction_speed_sq`, `planner.rs:734`) corners on the angle between consecutive blocks'
unit vectors. The dot product at `planner.rs:741` extends to four terms:

```rust
let dot = prev[0]*unit_vec[0] + prev[1]*unit_vec[1] + prev[2]*unit_vec[2] + prev[3]*unit_vec[3];
```

**Does the rotary component participate in the unit vector?** **Yes — grbl includes it.** grbl builds
`unit_vec` over *all* axes (linear and rotary alike) and corners on the full-dimensional direction change.
The rationale: a change in rotary direction is a real velocity discontinuity on the A motor and must be
limited the same way a linear corner is. So `unit_vec[3]` is the normalized rotary component, included in
both the length normalization and the junction dot product.

**Unit-vector normalization with mixed units.** The unit vector is normalized by the **total** move length in
the unit-agnostic space — i.e. `sqrt(Δx²+Δy²+Δz²+Δa²)` treating Δa (deg) as if it were mm, per the grblHAL
convention. This is the one place the "degrees == mm" abstraction is visible in the cornering math, and it is
*intentional* and matches grbl: it means a 1-degree rotary step and a 1-mm linear step contribute equally to
the direction vector. **With the corrected DOC-10.2 model, `Block::millimeters` is *also* this same all-axis
norm**, so the ramp-distance path length and the unit-vector normalization are now the **same quantity** — one
`sqrt(Δx²+Δy²+Δz²+Δa²)` computed once in `build_block`. (The earlier draft's separate linear-only path length is
gone, and with it the two-quantity bug surface that was risk R2.)

**Degenerate cases to handle (and test):**
- **Pure-rotary → pure-rotary, same direction:** dot = +1 → `cos_theta = -1`, no restriction. Correct (A
  spins continuously).
- **Pure-rotary → pure-rotary, reversal:** dot = -1 → `cos_theta = +1` → junction speed forced to 0
  (`planner.rs:744`). Correct (A must stop to reverse).
- **Pure-rotary → pure-linear junction:** the two unit vectors are orthogonal (one lives entirely in the A
  component, the other entirely in XYZ) → dot = 0 → `cos_theta = 0` → a 90°-equivalent corner, junction speed
  bounded by `$11`. This is **conservative but safe**: it forces a slowdown at a linear↔rotary transition,
  which is physically reasonable (the motion genuinely changes from translating to rotating). grbl behaves the
  same. Document it; do not special-case it.
- **Zero rotary participation:** if A never moves, `unit_vec[3] = 0` everywhere and the math reduces exactly
  to today's 3-axis behavior — a pure XYZ program is byte-for-byte unaffected. This is a required regression
  invariant (DOC-10.10).

---

## DOC-10.4: Acceleration & max-rate limiting across 4 axes

`limiting_acceleration` (`planner.rs:974`) already loops `0..AXES` over `unit_vec` and `accel_mm_s2`,
returning the most-restrictive per-axis acceleration along the direction vector. With `$123` (deg/s²) in
`accel_mm_s2[3]` and the rotary unit-vector component, it computes the rotary accel limit in the same
unit-agnostic space — **no code change beyond the array widening.** Units stay self-consistent because the
whole pipeline is in the "distance unit per second²" abstraction: deg and deg/s² are internally consistent
exactly as mm and mm/s² are.

`PlannerConfig` (`planner.rs:109`) per-axis arrays widen to `[T; 4]`:
- `steps_per_mm: [f32; 4]` — index 3 is **steps per degree** (`$103`).
- `max_rate_mm_min: [f32; 4]` — index 3 is **deg/min** (`$113`).
- `accel_mm_s2: [f32; 4]` — index 3 is **deg/s²** (`$123`).

The field *names* keep their `_mm` suffix (renaming churns the whole settings/proto/test surface for no
behavioral gain); the doc comments must state index 3 is degrees. (Alternative considered: rename to
`steps_per_unit` etc. — rejected as gratuitous churn; flagged as an open question DOC-10.11-Q1 if the
reviewer prefers the rename.)

`PlannerConfig::default()` (`planner.rs:129`) widens its arrays — propose A defaults of
`steps_per_mm[3] = 8.889` (200 steps × 16 microsteps / 360°), `max_rate_mm_min[3] = 3600` deg/min
(= 10 rev/min), `accel_mm_s2[3] = 360` deg/s². These are placeholders for first boot; real values come from
flash.

---

## DOC-10.5: Arcs (G2/G3)

**A is not an arc-interpolation plane.** The only supported arc plane is G17 (XY); see `plan_arc`
(`planner.rs:843`) which hardcodes the XY plane and helically interpolates Z. The rotary axis is treated
**exactly like helical Z**: A is *linearly slaved* across the arc's segments, advancing `Δa / segments` per
chord, in lockstep with the Z helix interpolation already at `planner.rs:884`.

```rust
let a_start = start[A_AXIS];
let a_delta = target[A_AXIS] - a_start;
// inside the segment loop, alongside the existing z interpolation:
let a = a_start + a_delta * (seg as f32 / segments as f32);
```

`arc_endpoint_mm` (`planner.rs:903`) and `mm_target_to_steps` (`planner.rs:926`) already loop `0..AXES`, so
they widen for free; only the explicit Z helix block needs an A sibling.

**Explicitly out of scope:** rotary-plane arcs (an arc interpolated *in* a plane that includes A, e.g. a G18
ZX arc combined with rotary). G18/G19 are already unsupported (`gcode.rs` claims only G17). A G2/G3 with an
`A` word produces an A-slaved helical arc; it never attempts circular interpolation of the rotary axis.
Document this as a hard boundary so a future "wrapped arc" feature is a deliberate, separate effort.

---

## DOC-10.6: Soft limits / rollover for the rotary axis

`SoftLimits.max_travel_mm: [f32; AXES]` (`planner.rs:101`) and `soft_limit_violation` (`planner.rs:953`)
enforce the grbl envelope `[-max_travel, 0]` per axis. **Rotary-ness is decided solely by the `$376` mask, not
by `$133`** (Q5, resolved): an axis marked rotary in `$376` is a continuous/rollover axis; its `$133` value is
**ignored** entirely. This is grblHAL's actual behavior and the reason `$376` is the cleaner model than the
earlier `$133 == 0` proposal — `$133` keeps a single, unambiguous meaning (a hard travel bound) and never
doubles as a mode flag.

Behavior, per the `$376` mask:

- **Linear axis (bit clear):** unchanged — the `[-max_travel, 0]` envelope from `$130–$132` applies.
- **Rotary axis (bit set, e.g. A under the default `$376 = 8`):** **continuous / rollover.** Soft limits are
  disabled for that axis and the commanded position rolls over modulo 360°. Its `$133` is ignored.

Implementation:

- `soft_limit_violation` takes the live `rotary_mask` (or the `PlannerConfig`) and **skips any axis where
  `config.is_rotary(axis)`** — that axis is never a soft-limit violation, regardless of `$133`. Linear axes
  fall through to the normal envelope check unchanged.
- **Position rollover** for a rotary axis: the planner reduces the *commanded* machine A position modulo
  `360° × $103` steps after each move so `position_steps[3]` never grows unbounded (preventing i32 overflow on
  a long-running spin and keeping `MPos:A` readable). The *step delta* of each move is taken **before** the
  modulo (so `A0 → A360` still emits a full revolution of steps), then the stored position is normalized.
  **Note (review, 2026-06-20): this modulo-360 normalization is a deliberate Galdr divergence, NOT
  grbl-canonical.** grblHAL's `gcode.c` does *not* wrap rotary position — positions accumulate unbounded
  (verified: no `fmodf`/360/rollover in the absolute-target resolution). We add it only to bound
  `position_steps[3]` against i32 overflow and keep `MPos:A` readable. It MUST be implemented so a G90
  **absolute** `A` target still resolves correctly against the wrapped stored position (an absolute `A370` after
  a wrap-to-10° still means 370° ≡ 10°), and so the WCS/`G92` A-offset interaction is unaffected — both are
  explicit test points (DOC-10.10 #19, plus a new "absolute target after wrap" test). If this proves fiddly, it
  is safe to ship Phase 3 *without* rollover first (i32 overflow is ~670k revolutions away at the default `$103`)
  and add it as a follow-up.
- **Homing/`$5`/`$23`:** a rotary axis (per `$376`) is **not homed** (no limit switch, no machine-zero seek).
  `$H` skips it; the A machine zero is wherever it powers up (or wherever a `G92`/`G10 L20` sets it). This is a
  DOC-06 touchpoint — homing must learn to skip rotary axes (a one-line guard keyed on `config.is_rotary`).

---

## DOC-10.7: Settings surface

New per-axis A entries extend the existing `$10x/$11x/$12x/$13x` triples to quads. The descriptor table
(`SETTING_DESCRIPTORS`, `settings.rs:891`) already derives the per-axis number from a base + axis index via
`axis_span()` (`settings.rs:697`), and `axis_span()` returns `AXES` for the per-axis fields. **Bumping
`AXES` to 4 automatically makes `$$`/`$x=val`/`$ES` enumerate the 4th axis** for every per-axis field — the
table is the single authority (per the settings-contracts memory). The new settings fall out as:

| Setting | Field | Meaning |
|---------|-------|---------|
| `$103` | `StepsPerMm[3]`   | steps per degree, A |
| `$113` | `MaxRateMmMin[3]` | max rate, A (deg/min) |
| `$123` | `AccelMmS2[3]`    | acceleration, A (deg/s²) |
| `$133` | `MaxTravelMm[3]`  | max travel, A (deg). **Ignored for an axis marked rotary in `$376`** (rotary axes are continuous/rollover, DOC-10.6); meaningful only for a bounded/non-rotary axis. |
| `$376` | `rotary_mask` (`RotaryAxes`) | **rotary-axes bitmask** — bit N set ⇒ axis N is angular (degrees), continuous/rollover, and exempt from G20/G21 inch scaling. Default `8` (bit 3 → A). New scalar descriptor. |

The `$103/$113/$123/$133` quad entries derive from the existing base descriptors at `settings.rs:1197`
(`$100`), `:1210` (`$110`), `:1223` (`$120`), `:1236` (`$130`) — base + axis 3 → 103/113/123/133. **No new
*per-axis* descriptor entries are needed** for these; the `AXES` bump plus widened `MachineSettings` arrays
(`settings.rs:212–222`) is enough.

**`$376` is a NEW scalar descriptor** (one row appended to `SETTING_DESCRIPTORS`, `settings.rs:891`), datatype
`AxisMask` (like `$2`/`$3` step/dir invert), `axis_span() == 1` (it is a single bitmask value, not a per-axis
triple), min `0` / **max `8`** — on this machine only bit 3 (A) is ever meaningful (X/Y/Z are never rotary), so
`sanitize` masks the stored value down to bit 3. `MachineSettings` gains a `rotary_mask: u8`, mirrored into
`PlannerConfig::rotary_mask` wherever the firmware builds the planner config from settings.

> **Correction (review, 2026-06-20) — default handling.** grblHAL's own `$376` default is **0** (no axis
> rotary); Galdr defaults to **8** because axis 3 is *physically* the rotary table. But the obvious "have
> `sanitize` fill 8 whenever the field is zero" is **wrong**: proto3 flat fields cannot distinguish *absent*
> (an old 3-axis flash record) from *present-and-zero* (a user who deliberately set `$376=0` to run A as a 4th
> linear axis — exactly DOC-10.10 test #18). Forcing 8-on-zero would make `$376=0` impossible. Instead: put the
> 8 default in `MachineSettings::default()` (the power-on value before any flash), have `from_proto` read the
> stored value **verbatim**, and have `sanitize` only **mask to valid bits** (bit 3), never force non-zero. An
> old 3-axis record therefore loads `rotary_mask = 0` (A treated as linear) until the user opts in with
> `$376=8` — acceptable, and consistent with grbl's own default-0. Test #16 asserts the *fresh* default is 8.

Plus A-axis TMC: `run_current_ma[3]`, `hold_current_ma[3]`, `microsteps[3]`, and `TMC_NODES`
(`settings.rs:26`) extends from `[0, 1, 2]` to `[0, 1, 2, 3]` (node 3 = A driver, MS1/MS2 address pins set
accordingly per DOC-03).

### Proto implications (brief)

The proto schema (`crates/galdr-proto/proto/settings.proto`) uses **flat `_x/_y/_z` triples, deliberately not
`repeated`** (documented at the top of the file: flat fields give a fixed-layout generated struct). Extending
to 4 axes therefore means **adding `_a` fields**, not changing cardinality:

- `steps_per_mm_a`, `max_rate_mm_min_a`, `accel_mm_s2_a`, `max_travel_mm_a`, `run_current_ma_a`,
  `hold_current_ma_a`, `microsteps_a` — each a **new field number** appended after the current max tag (the
  existing tags 18–48 stay put; A fields take fresh tags). Appending new field numbers is wire-compatible —
  an old flash record simply lacks them and they default (then `sanitize` fills the A defaults).
- `rotary_mask` (`uint32`, the `$376` bitmask) — also a **new appended field number**. An old record lacks it
  and decodes to 0; per the DOC-10.7 correction, `sanitize` does **not** force it to 8 (that would forbid a
  deliberate `$376=0`), so a pre-existing 3-axis record loads with A treated as **linear** until the user sets
  `$376=8`. The `DEFAULT_ROTARY_MASK` (8) is the *fresh-boot* default via `MachineSettings::default()`, not a
  zero-fill.
- `to_proto`/`from_proto` (`settings.rs:1556+`) gain the `[3]` ↔ `_a` mappings.
- `Coordinates` message gains `_a` for each WCS/predefined offset (the coords model `[[f32; AXES]; …]` widens
  for free; only the flat proto mirror needs the new fields).

**Migration note:** because proto fields are append-only and `sanitize` (`settings.rs:512`) fills any
zero/garbage A value with the default, an existing 3-axis flash record loads cleanly and gains sane A
settings — no flash format version bump required.

---

## DOC-10.8: Block / segment generator & RMT executor impact

**These layers are already axis-count-generic.** Audit result:

- **`Block`** (`planner.rs:144`): `steps: [i32; AXES]`, `unit_vec: [f32; AXES]` — widen for free. The
  dominant-axis `step_event_count` (`planner.rs:151`, computed by `max` over `0..AXES`) already considers all
  axes, so A can be the dominant axis of a pure-rotary block with no change.
- **Segment generator** (`crates/firmware-core/src/motion.rs`): `StepEvent { step: [bool; AXES], … }` is
  per-tick all-axes (per the motion-contracts memory); the Bresenham DDA loops `0..AXES` accumulating
  per-axis error. Widening `AXES` makes A a fully coordinated DDA axis automatically — **this is what
  delivers "true coordinated" 4-axis interpolation** at the step level. `set_direction(DirState{ dir: [bool;
  AXES] })` widens too.
- **RMT executor** (`crates/firmware/src/motion.rs`): `channels: [Option<AxisChannel>; AXES]`, `dir: [Output;
  AXES]`, and `scratch: [[PulseCode; …]; AXES]` (`firmware/src/motion.rs:102–125`) all size by `AXES`.
  `encode_channel`/`emit_burst` loop `0..AXES` and `join` the channel transmits.

**What actually widens (the concrete change list):**
1. `planner::AXES` `3 → 4`, add `A_AXIS`/`DEFAULT_ROTARY_MASK`, `PlannerConfig::rotary_mask` +
   `is_rotary()` (DOC-10.1). The `$376` setting plumbs the mask in (DOC-10.7).
2. `AxisWords` gains `a`; lexer adds the `b'A'` case (`gcode.rs:751`, next to `b'X'/Y/Z`).
3. `build_block` length/feed logic (DOC-10.2); units fork on `config.is_rotary(axis)` for the rotary word.
4. Firmware: a 4th RMT TX channel (**ch3, currently the documented spare** — DOC-00) + a 4th DIR `Output`
   pin. The `join3` over three channels becomes `join4` (or a join over the `[_; AXES]` array). **This is the
   only new *hardware* wiring.** GPIO assignment for A-STEP/A-DIR/A driver UART address pins is a DOC-00
   manifest addition.
5. Compile-time assertions keyed to 3 must update: `coords.rs:421` (`assert!(AXES == 3, …)`) — change to `== 4`
   and re-verify the wire-conversion test fixtures.

**Constraint to preserve:** `mem_block_symbols ≤ 48` per channel (one RMT memory block) — adding a 4th channel
does **not** change per-channel budget; each channel still encodes ≤ 48 symbols. The `MAX_SYMBOLS_PER_BURST`
cap is per-channel and unchanged.

---

## DOC-10.9: Protocol widening

- **`AXIS_COUNT`** (`protocol.rs:80`) already aliases `crate::planner::AXES`, so it becomes 4 automatically.
- **`MPos`/`WPos`**: `mpos_mm: [f32; AXIS_COUNT]`, `wco_mm: [f32; AXIS_COUNT]` (`protocol.rs:678–681`) widen
  for free; the status-report formatter emits a 4th comma-separated field. grblHAL natively reports N axes
  (`MPos:x,y,z,a`).
- **`[AXS:…]`**: the build-info line at `protocol.rs:77–78` documents `[AXS:3:XYZ]`; it becomes `[AXS:4:XYZA]`.
- **`$G`**: emits the live feed mode (`G93`/`G94`, DOC-10.2) and an A word where relevant.

**Test fallout (flagged, not enumerated exhaustively):** every `protocol.rs` test asserting a literal
3-field `MPos:`/`WPos:` string, `[AXS:3:XYZ]`, or the `[GC:… G94 …]` `$G` line (e.g. `protocol.rs:2388`,
`:2408`, `:2418`) updates to the 4-field / live-feed-mode form. `coords.rs` wire fixtures (`:421` onward)
update. This is mechanical but touches many assertions — budget for it.

**skirnir:** our only client. Its `<...>`/`Pn:` parser and position display update to decode a 4th axis
(`A`). The `Pn:` limit field is unaffected (A is typically unhomed/limit-switch-free, DOC-10.6). This is a
separate `skirnir-engineer` workstream once the firmware side lands; the wire format is the contract.

---

## DOC-10.10: Test plan (TDD spec)

Tests are **host unit tests in `firmware-core`** (pure logic, no hardware), written **first**. Grouped by the
subsystem they pin. The single most important suite is the **3-axis regression invariant**: any program with
no A word must produce byte-identical blocks/speeds to today.

### A. Feed-convention (G94) — `planner.rs` tests
1. `g94_linear_only_feed_is_mm_per_min` — `X10 F600`: `Block::millimeters == 10`, nominal == 600/60 mm/s.
2. `g94_mixed_move_feed_governs_linear_path_a_slaved` — `X10 A90 F600`: assert the **block duration** equals that
   of `X10 F600` (A is "free", the inverse-time conversion makes duration `linear_len/F`), and the A step delta
   equals 90° of steps. (Do NOT assert `millimeters == 10`: under the corrected model `millimeters` is the full
   norm `sqrt(10²+90²) ≈ 90.55`; the behavioral contract is the *timing*, not the length scalar.)
3. `g94_rotary_only_feed_is_deg_per_min` — `A90 F360`: path length == 90 (deg), nominal == 360/60 deg/s.
4. `g94_xyz_diagonal_unchanged_by_a_absence` — `X3 Y4 F600`: length == 5 (regression; A component zero).

### B. Feed-convention (G93 inverse-time)
5. `g93_move_duration_is_one_over_f` — `G93 X10 A90 F2`: derived nominal speed == path_len × 2 / 60; assert
   the implied duration is 0.5 min.
6. `g93_requires_f_per_line` — `G93 X10` with no F → `GcodeError::FeedRateUndefined` (code 22), even with a
   prior modal F.
7. `g93_pure_rotary_duration` — `G93 A360 F4` → duration 0.25 min; nominal == 360 × 4 / 60 deg/s.
8. `g93_g94_modal_conflict_on_one_line` — `G93 G94` → modal group violation.
9. `feed_mode_defaults_to_g94_and_is_sticky` — power-on default `UnitsPerMin`; survives across lines until
   `G93`.

### C. Junction velocity with a rotary component
10. `junction_pure_rotary_straight_no_restriction` — two collinear `A` moves: junction speed unbounded by
    cornering (dot = +1).
11. `junction_pure_rotary_reversal_forces_stop` — `A10` then `A-10`: junction speed 0.
12. `junction_linear_to_rotary_is_orthogonal_corner` — `X10` then `A10`: `cos_theta == 0`, junction speed ==
    the `$11`-bounded value (document the conservative slowdown).
13. `junction_unaffected_when_a_never_moves` — XYZ-only chain matches the current 3-axis junction speeds
    exactly (regression).

### D. Acceleration limiting in deg/s²
14. `accel_limit_pure_rotary_uses_a_accel` — `A90` block acceleration == `$123` (deg/s²).
15. `accel_limit_mixed_picks_most_restrictive` — a mixed move where A's accel is the binding constraint, vs.
    one where a linear axis binds; assert the correct one wins (unit-agnostic limiting).

### E. `$376` rotary detection, soft-limit / rollover
16. `default_mask_marks_a_rotary` — default `$376 == 8`: `config.is_rotary(3) == true`, `is_rotary(0..3) ==
    false`.
17. `rotary_axis_disables_soft_limit` — A in `$376`: a huge `A` target is never a `MoveExceedsTravel`, and its
    `$133` is ignored (set `$133 = 90` yet `A100` still passes because A is rotary).
18. `non_rotary_axis_still_enforces_envelope` — clear A's bit in `$376` (treat A as a bounded 4th linear axis):
    `$133 == 90` then `A100` → `MoveExceedsTravel` (proves `$133` governs only non-rotary axes).
19. `rotary_position_rolls_over_modulo_360` — after `A720` the stored `position_steps[3]` is normalized to a
    single-rev range, but the *move* emitted two revolutions of steps.
20. `rotary_rollover_does_not_overflow_i32` — many full revolutions keep `position_steps[3]` bounded.
21. `mask_drives_inch_scaling_and_feed_convention_at_runtime` — with A rotary, `G20 A90` resolves to 90° (no
    inch scale) and a `A90`-only move uses the deg/min feed convention; **clear A's `$376` bit** and the SAME
    `G20 A90` now inch-scales (90×25.4) and the move is treated as a 4th linear axis — proving `$376` re-gates
    units AND the path-length/feed convention at runtime, not at compile time.

### F. Arc with slaved A
22. `arc_slaves_a_linearly_like_z_helix` — `G2 X… Y… A90 I… J…`: each subdivided segment advances A by
    `90/segments`; total A delta == 90°; no circular interpolation of A.

### G. Units interaction
23. `g20_inch_does_not_scale_rotary_word` — with A rotary (`$376` default), `G20 A90` resolves to 90°, not
    90×25.4. (The runtime-toggle counterpart is test 21.)

### H. Regression (the load-bearing invariant)
24. `three_axis_program_byte_identical_with_four_axes` — a representative XYZ program (moves + arc + junction
    chain) produces the same `Block` fields with `AXES == 4` and A absent as the captured 3-axis golden
    values. **This is the gate that proves the widening is non-destructive.**

---

## DOC-10.11: Risks, open questions, and phased implementation order

### Risks
- **R1 — Feed semantics are subtle.** The G94 "A is free / slaved" rule and the G93 duration rule are the
  highest-bug-risk area. Mitigated by writing suite A/B first and treating them as the contract.
- **R2 — RESOLVED by the corrected feed model (review, 2026-06-20).** The earlier dual-length design (separate
  linear-only path length vs. all-axis direction norm) is gone: with grbl's `ROTARY_FIX` model, `Block::millimeters`
  and the `unit_vec` normalization are the **same** all-axis norm, and the mixed-G94 feed is handled by an
  inverse-time conversion (reusing the G93 path), not a second length. No two-quantity bug surface remains. The
  residual risk folds into R1 (the inverse-time conversion itself must be correct).
- **R3 — Protocol/skirnir test churn** is broad (DOC-10.9). Mechanical, but a large diff; isolate it in its
  own phase so a regression there can't be confused with a kinematics bug.
- **R4 — Hardware path unverified.** RMT ch3 + the A DIR pin + TMC node 3 are compile-only until bench
  (mirrors the DOC-06/DOC-07 "logic host-tested, hardware deferred" posture). No coordinated-rotary motion is
  trustworthy until run on the board.
- **R5 — Rollover ↔ homing interaction (DOC-10.6):** `$H` must skip a rotary axis (per `$376`); missing that
  guard would make homing seek a non-existent A limit switch and hang. DOC-06 touchpoint.

### Resolved decisions (reviewer's calls — folded into the design above)
- **Q1 — Field naming → RESOLVED: keep mm-named fields.** `steps_per_mm`/`max_rate_mm_min`/`accel_mm_s2`/
  `max_travel_mm` keep their names; index 3 carries degrees, documented in the doc-comment only. No
  unit-neutral rename (it would churn settings/proto/tests for zero behavioral gain). The design above is
  unchanged by this.
- **Q3 — G93 over-speed → RESOLVED: clamp and finish slower.** When a commanded G93 duration is faster than
  the per-axis rate clamp (`$11x`) allows, the planner clamps the nominal speed to the achievable rate, the
  move finishes *slower* than commanded (never losing steps), and the firmware emits a `[MSG:…]` advisory.
  Already specified in DOC-10.2 ("a too-fast G93 duration is still floored …"). No rejection path.
- **Q5 — Rotary semantics → RESOLVED *against* the original recommendation: adopt the grblHAL `$376`
  rotary-axes bitmask.** Rotary-ness is now a **runtime** bitmask setting (default `8`), not a const and not a
  `$133 == 0` convention. This is a net design improvement: one setting drives *both* continuous/rollover
  soft-limit handling *and* G20/G21 inch-scaling suppression (the Q4 fork), keeps `$133` meaning a single
  unambiguous travel bound, and re-classifies an axis at runtime. Threaded through DOC-10.1 (`rotary_mask` /
  `is_rotary`), DOC-10.2 (units + path-length fork on `config.is_rotary`), DOC-10.6 (rewrite), DOC-10.7 (new
  `$376` scalar descriptor + proto field), and DOC-10.10 (tests 16–21). **Q4 is subsumed by Q5** — the same
  `$376` mask that marks an axis rotary also exempts its words from inch scaling, so it is no longer a
  separate question.

### Open questions (still awaiting the reviewer)
- **Q2 — A as dominant DDA axis & step budget.** A pure-rotary move with very high `$103` (steps/deg) can make
  A the dominant axis with a large `step_event_count`. The existing ≤48-symbol burst batching handles this
  (it already chunks long blocks), but confirm the rotary default `$103` and `$113` keep per-block symbol
  counts sane for PCB-engraving-on-cylinder workloads. No design change expected; flagging for awareness.

### Phased implementation order
1. **Parser + feed-mode (G93/G94).** Add `FeedMode`, the `Group::FeedMode` modal group, the `93`/`94`
   `apply_g_word` arms, the per-line `saw_feed` flag and the G93-F-required tightening (G0 exempt), and the live
   feed mode in the `$G` `[GC:...]` report (new `ParserSnapshot::feed_mode`). Tests: suite B (G93/G94 parsing,
   conflict, defaulting, F-per-line) + the `$G` feed-mode round-trip. *No planner kinematics yet — this phase
   leaves `AXES == 3` for the linear axes.*
   **Implementation note (review, 2026-06-20 — partially superseded):** Phase 1 as built ALSO threads `feed_mode`
   through `PlannerCommand::{Move,Arc}` and **pulls native G93 inverse-time execution forward into the planner**
   (DOC-10.2): `nominal_speed_mm_s` forks on `feed_mode` (`millimeters × F / 60` for inverse-time, clamped to the
   axis rate per Q3), and an inverse-time arc scales its per-segment feed by the segment count so the whole arc
   still takes `1/F` min. This closes the otherwise-silent "G93 accepted but mis-executed" gap a code review
   caught (G93 would have run at ~`F` mm/min). A `G38.x` probe stays units/min — inverse-time is undefined for a
   probe (it stops at an unknown contact point), so `Probe` carries no `feed_mode`. **Still deferred to Phase 2**
   (no consumer at `AXES == 3`, and they share the `AXES`-bump churn over ~35 `AxisWords { x, y, z }` literals):
   `AxisWords.a` + the `b'A'` lexer case, and the **mixed linear+rotary G94→inverse-time conversion** (which needs
   a rotary axis to exist). `JogCommand` never needs `feed_mode` (the jog grammar accepts no G93/G94).
2. **Settings + proto (now includes `$376`).** Bump `AXES` to 4, widen `PlannerConfig`/`MachineSettings`/
   `SoftLimits` arrays, add `TMC_NODES` node 3, append the `_a` proto fields, update
   `to_proto`/`from_proto`/`sanitize` defaults. **Add the `$376` rotary-axes bitmask:** the new scalar
   `SETTING_DESCRIPTOR` (`AxisMask`, `axis_span() == 1`), `MachineSettings::rotary_mask` →
   `PlannerConfig::rotary_mask` + `is_rotary()`, the appended proto `rotary_mask` field, and the
   `sanitize`-fills-`DEFAULT_ROTARY_MASK` default (DOC-10.7). Tests: `$103/$113/$123/$133` enumeration + `$376`
   round-trip/default + proto round-trip + sanitize-fills-A and -mask, plus suite-E tests 16 & 18 (mask drives
   rotary detection). *Mechanical; the `coords.rs:421` assertion flips to `== 4` here.* **`$376` lands here so
   the planner kinematics in Phase 3 can read `config.is_rotary()` instead of a const.** **Also lands here
   (moved from Phase 1):** `AxisWords.a` + the `b'A'` lexer case and the `a`-inclusive `has_axes()` — these
   rewrite the ~35 `AxisWords { x, y, z }` literals in lockstep with the `AXES`-bump churn they already share.
   (`feed_mode` threading through `PlannerCommand::{Move,Arc}` and native G93 execution already landed in Phase 1;
   `Probe`/`JogCommand` never carry `feed_mode`. The mixed-G94→inverse-time conversion is Phase 3, since it needs
   the rotary axis.)
3. **Planner kinematics (TDD).** The risky core: `build_block` path-length/feed classification reading
   `config.is_rotary` (DOC-10.2), 4-term junction (DOC-10.3), accel widening (DOC-10.4), arc A-slaving
   (DOC-10.5), `$376`-gated rollover + the runtime units fork (DOC-10.6 / DOC-10.1). Write suites A, C, D, E
   (incl. the runtime-toggle tests 17, 19–21), F, G, and the **H regression** first; implement until green.
4. **Protocol + skirnir.** 4-field `MPos`/`WPos`, `[AXS:4:XYZA]`, live-feed-mode `$G`; update the
   `protocol.rs`/`coords.rs` assertions; hand the wire-format delta to the skirnir workstream (DOC-10.9).
5. **Firmware ch3 / TMC node 3 (compile-only until bench).** Wire RMT ch3 + A-DIR `Output`, `join4`, TMC node
   3 on the shared UART, and the DOC-00 GPIO manifest additions. Compile-verify only; bench-verify A motion
   per a new `docs/4th-axis-bench-checklist.md` (companion to the homing/spindle checklists).

---

## Appendix: cited code anchors

| Anchor | What it is |
|--------|-----------|
| `planner.rs:39` | `pub const AXES: usize = 3` — the hardcoded axis count to bump |
| `planner.rs:101` | `SoftLimits.max_travel_mm: [f32; AXES]` |
| `planner.rs:109–124` | `PlannerConfig` per-axis arrays + `$11`/`$12` |
| `planner.rs:144–173` | `Block` (`steps`, `unit_vec`, `millimeters`, speeds) |
| `planner.rs:604` | `resolve_target` — units/distance/WCO resolution to steps |
| `planner.rs:660–698` | `build_block` — length, accel, nominal, junction |
| `planner.rs:670` | the 3-term Euclidean path-length sqrt (DOC-10.2 changes this) |
| `planner.rs:703–729` | `nominal_speed_mm_s` / `axis_rate_limit_mm_s` |
| `planner.rs:734–759` | `junction_speed_sq` — cornering model |
| `planner.rs:741` | the 3-term junction dot product (DOC-10.3 → 4 terms) |
| `planner.rs:843–898` | `plan_arc` — XY arc + Z helix (A slaves here, DOC-10.5) |
| `planner.rs:953` | `soft_limit_violation` (DOC-10.6) |
| `planner.rs:974` | `limiting_acceleration` (widens for free, DOC-10.4) |
| `gcode.rs:332` | `AxisWords {x,y,z}` (gains `a`) |
| `gcode.rs:~300` | `ModalState` (gains `feed_mode`) |
| `gcode.rs:751–762` | axis-word lexer `b'X'/Y/Z` (add `b'A'`) |
| `gcode.rs:731` | `FeedRateUndefined` (G93 tightens this) |
| `settings.rs:26` | `TMC_NODES = [0, 1, 2]` (→ `[0,1,2,3]`) |
| `settings.rs:697–700` | `axis_span()` returns `AXES` — auto-enumerates A |
| `settings.rs:1197/1210/1223/1236` | `$100/$110/$120/$130` base descriptors (→ `$103/$113/$123/$133`) |
| `protocol.rs:77–80` | `AXIS_COUNT` / `[AXS:3:XYZ]` |
| `protocol.rs:678–681` | `mpos_mm`/`wco_mm: [f32; AXIS_COUNT]` |
| `protocol.rs:1359` | hardcoded `G94` in `$G` (→ live feed mode) |
| `firmware/src/motion.rs:102–125` | `RmtStepSink` `channels`/`dir`/`scratch: [_; AXES]` |
| `firmware/src/motion.rs:281` | `emit_burst` (join over channels → join4) |
| `coords.rs:421` | `assert!(AXES == 3, …)` compile-time guard (→ `== 4`) |
| `motion.rs` (firmware-core) | `StepEvent { step: [bool; AXES] }`, Bresenham DDA over `0..AXES` |
