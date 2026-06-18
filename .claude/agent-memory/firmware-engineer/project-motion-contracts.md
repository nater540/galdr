---
name: project-motion-contracts
description: Galdr motion model / segment generator (DOC-02) design decisions the firmware-bin RMT layer must honor — StepSink/StepEvent shape, tick model, MotionConfig ownership.
metadata:
  type: project
---

The segment generator lives in `crates/firmware-core/src/motion.rs` (implemented 2026-06-16, pure sync,
no_std, host-tested, 19 tests). Traits in `crates/firmware-core/src/hal_traits.rs`. Decisions a future
author (esp. the firmware-bin RMT encoder + dual-core wiring) needs that are NOT obvious from the code:

**`StepEvent` is per-TICK, all-axes, NOT per-axis** (the DOC-09 sketch `emit_burst(&[StepEvent])` with a
per-axis event was rejected). `StepEvent { step: [bool; AXES], period_ticks: u32 }` = one synchronized DDA
tick across X/Y/Z plus the full step period. **Why:** the three RMT channels must emit equal-length,
sample-aligned PulseCode arrays so the `join3` await stays coordinated (DOC-02 line ~333); a flat per-axis
slice loses that alignment. **How to apply (firmware bin):** map each `StepEvent` 1:1 to one PulseCode per
channel — stepping axis = HIGH for `$0` ticks / LOW for `period−$0`; silent axis = full-period-LOW symbol.
Same tick count on every channel.

**Direction is latched ONCE per block via `StepSink::set_direction(DirState)`, never per tick.** A
straight-line block never reverses an axis. The RMT impl drives DIR GPIOs then honors `$29` setup delay
before the burst's first pulse. `set_direction` is NOT called for a zero-length (no-op) block.

**`MotionConfig` (in motion.rs) owns `$0`/`$29`, NOT `PlannerConfig`.** Confirmed: `PlannerConfig` has no
step-pulse/dir-setup fields (those are firmware settings, DOC-00 line ~491). `MotionConfig { tick_hz,
step_pulse_ticks, min_low_ticks }`. Default = 1 MHz tick (clk_divider=80 → 1 tick=1µs), `$0`=10, min_low=2
→ min period 12µs → max ~83.3 kHz/axis (DOC-02 headroom figure). The firmware bin loads real `$0`/`$29`
from esp-storage and builds the MotionConfig; `$29` is applied by the sink, not modeled as a tick period.

**Entry point:** `SegmentGenerator::run_block(&Block, exit_speed_sq, &mut impl StepSink) -> Result<u32,
MotionError>`. Returns total ticks emitted (== `block.step_event_count` for a moving block, 0 for
zero-length). `exit_speed_sq` is the NEXT queued block's `entry_speed_sq` (squared, mm/s²), or 0 if queue
empty / final block — the async executor supplies it via `Planner::peek_block` (peek the 2nd block while
running the 1st). Speeds stay squared per [[project-planner-contracts]] — do NOT re-derive.

**DDA is exact integer Bresenham** (dominant axis steps every tick; subordinates accumulate `error +=
abs_steps; if >= dom_count { step; error -= dom_count }`). Step conservation is the load-bearing invariant:
per-axis emitted step sum == `|block.steps[axis]|` exactly. Only the velocity→period math is f32. Bursts
batch ticks to ≤ `MAX_SYMBOLS_PER_BURST` (48, one RMT block) and flush — never exceed one block.

**Trapezoid classified in `TrapezoidProfile::plan`** (accel/cruise/decel breakpoints in mm via
`d=(v²−v₀²)/2a`; triangle crossover when ramps overlap before nominal). Velocity sampled at each step's
MIDPOINT `(tick+0.5)/total` to center the discrete ramp on the continuous profile. A momentary v=0 at a
rest boundary floors to `SLOWEST_PERIOD_TICKS` (30000) to avoid div-by-0 / 15-bit-field saturation.

**Test no_std gotcha:** the recording-sink test module needs `extern crate std;` + `use std::vec::Vec;`
(firmware-core is `#![no_std]`; std only links under `#[cfg(test)]`). Also: `gen` is a RESERVED keyword in
edition 2024 — do not name a binding `gen`.

**Pipeline is now logically complete on host:** parser (DOC-04) → planner (DOC-05) → motion (DOC-02) all
host-tested. Remaining for firmware bin: RMT PulseCode encoding, `join3`, dual-core InterruptExecutor,
FEED_HOLD/stop signal checks at burst boundaries (all touch esp-hal, deliberately OUT of firmware-core).

**Hot-loop invariants are hoisted (2026-06-16 #10).** `emit_profile` builds a `StepTiming` ONCE per block
(dom_mm_per_step, tick_hz, max_rate_hz, min_period_ticks, floor_period_ticks) via `StepTiming::for_block`; the
per-tick `timing.period_ticks(v_sq)` only does sqrt+1 divide (the old `period_ticks` recomputed dom_mm_per_step +
max_step_rate_hz() every tick). `TrapezoidProfile` stores `two_a` (=2·accel) instead of `accel`, so
`velocity_sq_at` no longer recomputes `2.0*accel` per call. Behavior identical — same formula/clamps/rounding.

See [[project-planner-contracts]], [[project-build-constraints]], [[project-hardware-map]].
