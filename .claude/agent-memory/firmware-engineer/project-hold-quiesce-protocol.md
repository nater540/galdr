---
name: project-hold-quiesce-protocol
description: Level-based hold/resume + quiesce-ack protocol between core-0 comms and core-1 motion executor (DOC-02/DOC-08)
metadata:
  type: project
---

The motion-executor hold/resume protocol is LEVEL-based, not edge-based (replaced fragile FEED_HOLD/CYCLE_START Signals that caused findings #1-#4).

**Why:** edge Signals were consumed only at the top-of-loop hold check; the empty-queue branch and MOTION_RESET path never drained them, and the "drain stale CYCLE_START" logic discarded legitimate resumes. One design flaw, four findings.

**How to apply (the mechanism, all in comms.rs statics + motion.rs `run`/`park_on_hold`):**
- `HOLD_REQUESTED: AtomicBool` — the authoritative hold LEVEL. SET by `!`(feed_hold), `$SLP`, jog-cancel; CLEARED by `~` (gated on `ControlState::resumes_on_cycle_start`) and by soft reset. Executor re-reads it at EVERY block boundary AND in the empty-queue `select4`.
- `HOLD_WAKE: Signal` — promptness nudge on any level change; level is re-read after every wake, so a coalesced/missed/spurious wake loses nothing.
- `MOTION_PARKED: Signal` — the executor's quiesce ACK; `park_on_hold` pulses it when it rests. `quiesce_executor` (comms.rs) raises the level, wakes, awaits MOTION_PARKED → a REAL parked fact (no EXECUTOR_RUNNING poll + blind cycle-start race). `release_hold` clears level + wakes. jog-cancel and probe-abort both reuse `quiesce_executor`.
- `~` gating lives in pure `ControlState::resumes_on_cycle_start()` (protocol.rs, host-tested) — true ONLY for `Hold(_)`, so `~` is inert in Sleep (only soft reset wakes sleep — Finding #1).
- Soft-reset dispatch (`0x18`/`0x19`) must clear `HOLD_REQUESTED` AND drain `PROBE_REQUEST` so no stale hold/probe survives a warm reset (Findings #2, #4). `run_probe_cycle`'s reset arm also drains PROBE_REQUEST.
- `HARD_LIMIT_TRIPPED` (executor→consumer, latching `Signal`) is drained on soft reset in BOTH the reader `SoftReset|Stop` dispatch AND the END of `apply_soft_reset` (after `reset_pipeline`) — Finding #5b. `check_hard_limits` is LEVEL-based, so a switch left ENGAGED after an aborted `$H` seek keeps re-latching it; the reader drain handles the at-reset latch, the consumer drain closes the cross-core re-latch race during the long awaiting reset window, else a stale `ALARM:1` (locked) clobbers the post-`$X` state → every line `error:9` until reboot. `LIMIT_TRIGGERED` (motion.rs) is a documented seam with a PRODUCER but NO consumer (only `.signal()`'d, never `.wait()`/`.try_take()`'d) — needs no drain. Consumer's hard-limit arm also guards with `ControlState::hard_limit_alarm_applies()` (protocol.rs, host-tested): raise `ALARM:1` only from a MOVING-capable state (`Normal`/`Hold`/`Jog`/`Check`), never from `Alarm(_)`/`Sleep`, so a stale trip can't downgrade a more-specific lock (esp. `ALARM:11` boot-lock).
- `error:9` (ERROR_LOCKED), not error:1, for GCode rejected during Alarm/Jog/Sleep lockout (Finding #5). `$RST=*` clears+persists coords too via shared `clear_and_persist_coordinates` (Finding #7). `OVERRIDE_CHANGED` deleted — overrides apply at next block boundary, not mid-block (Finding #10).

Accepted approximation: empty-queue `select4` can run a PROBE_REQUEST even if HOLD is latched (pre-existing; consumer never issues a probe while held, since it blocks in-order on PROBE_RESULT).
