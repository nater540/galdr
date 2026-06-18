---
name: project-motion-executor-review-fixes
description: Galdr DOC-02 motion-executor code-review fixes (2026-06-16) — verified RMT end-marker semantics, busy-block planner protection, dedicated MOTION_RESET, live-position atomics, 47-event burst cap.
metadata:
  type: project
---

A code review of commit ae3cc4c (branch `firmware/core-pipeline-and-streaming`) found real bugs in the DOC-02
motion executor; all fixed 2026-06-16. Non-obvious facts a future author needs:

**RMT end-marker semantics (esp-hal 1.0.0, VERIFIED in rmt.rs):** `PulseCode::is_end_marker()` is
`length1()==0 || length2()==0` (rmt.rs:498) and the hardware STOPS transmission at the first end marker. So the
old idle/silent-axis symbol `new_clamped(Low, period, Low, 0)` (length2==0) was an end marker — in any
coordinated move a subordinate axis is idle on tick 0 (Bresenham), so it dropped ALL its remaining steps (the
showstopper). FIX: encode a silent tick as TWO non-zero LOW halves via
`firmware_core::motion::silent_symbol_halves(period) -> (u32,u32)` (`a=period/2≥1, b=period-a≥1, a+b==period`),
host-tested. The stepping symbol HIGH(`$0`)/LOW(`period−$0`) was already safe (`$0≥1`, `period−$0≥min_low≥1`).
The end_marker is the ONLY symbol allowed a zero length, and must be last.

**15-bit period clamp (Finding #2):** an RMT length field is a single 15-bit field, `PulseCode::MAX_LEN=0x7FFF`.
`firmware-core` now exposes `pub const RMT_MAX_FIELD_LEN: u32 = 0x7FFF`. `StepTiming::period_ticks` clamps the
rounded period to `[min_period, RMT_MAX_FIELD_LEN + step_pulse_ticks]` (the LOW half `period−$0` is the binding
field). Below that floor rate (`tick_hz/max_period`) feeds run at the floor — TODO: true sub-floor slow stepping
(slow Z-probing) needs multi-symbol periods. Host-tested.

**Busy-block planner protection (Finding #4, grbl):** `Planner` gained a `head_busy: bool`. `pop_block()` sets
`head_busy = !queue.is_empty()` — once the executor pops N, the new front (N+1) is the committed next block whose
`entry_speed_sq` was read as N's exit. `reverse_pass()` FREEZES the front block's entry while `head_busy` (still
carries its value forward as the exit ceiling for the block behind it). `forward_pass` already left the oldest
untouched. The flag clears when the queue drains. This was a FOCUSED flag, not a refactor (no checkpoint needed).
Host-tested (a post-pop enqueue must NOT change the just-peeked head exit).

**Dedicated MOTION_RESET (Finding #3):** an embassy `Signal` wakes ONE waiter; `SOFT_RESET` was already consumed
by `comms_consumer` + `plan_command`, so the core-1 executor missed resets. Added `MOTION_RESET: Signal` (idle
wake) + `MOTION_RESET_PENDING: AtomicBool` (poll-able mid-block flag, set with Release before the Signal). The
`0x18` dispatch fires BOTH alongside `SOFT_RESET`. The `CountingSink` tests `MOTION_RESET_PENDING` BETWEEN bursts
and returns `StepError` to abort `run_block` early (≤1 burst latency inside a long block). Executor is the SINGLE
owner of the reset MPos; `reset_pipeline` no longer writes MPos (no cross-core stale-overwrite race).

**Live-position atomics (Finding #5):** `LIVE_POSITION: [AtomicI32; AXES]` in comms.rs. The `CountingSink`
publishes the running step position after EACH burst (Release), so MPos is live WITHIN a block (was frozen until
`run_block` returned). `status_responder` reads it (Acquire) + converts via the new host-tested
`firmware_core::motion::steps_to_mm(&[i32;AXES], &steps_per_mm)`, and reads `planner_blocks_free` live from the
queue depth under lock (Finding #10). The block-boundary `publish_position`→MACHINE write was REMOVED; MACHINE now
holds only fields the executor doesn't own (state/feed/spindle/rx-free), which `status_responder` overlays.

**Burst cap is 47, not 48 (Finding #7):** `MAX_SYMBOLS_PER_BURST = 47` so 47 events + 1 end_marker = exactly one
48-symbol RMT block (`memsize=1`) — no adjacent-channel borrow, no interrupt-priority streaming refill. Tests are
cap-parameterized. Host test in hal_traits pins `MAX_SYMBOLS_PER_BURST + 1 == 48`.

**RMT channel lost on transmit-start error (Finding #6):** `Channel::transmit(self,...)` takes the channel BY
VALUE and on the Err path returns only `Error` (channel dropped) — that axis is down until reboot. `wait()` DOES
return the channel in both arms (already handled). Comment corrected (the old "next reset re-inits RMT" was
false); TODO(DOC-06 alarm path): alarm + re-run `motion::init` RMT bring-up to reclaim it.

**#8 stale cycle-start:** executor drains a latched `CYCLE_START.try_take()` BEFORE awaiting a fresh one when
entering a feed-hold, so a `~` latched with no hold pending can't auto-release the next hold.
**#9 $29 only on dir change:** `RmtStepSink` tracks `last_dir: Option<DirState>`; the `$29` busy-spin runs only
when `set_direction` actually changes the latched direction.

Verified BOTH toolchains: host `cargo test -p firmware-core` (152 green) + `RUSTFLAGS=-D warnings cargo build` +
`cargo clippy --all-targets -- -D warnings`; Xtensa (from crates/firmware, after `source $HOME/export-esp.sh`)
`cargo build`, `cargo build --features defmt`, `cargo clippy` — all clean (filter benign `'esp32s3' is not a
recognized processor` ld note; `clippy --all-targets` on Xtensa fails only on the host-only embedded-hal-mock
dev-dep, so use plain `cargo clippy`).

See [[project-motion-executor]], [[project-motion-contracts]], [[project-planner-contracts]],
[[project-consumer-pipeline]], [[project-firmware-bringup]].
