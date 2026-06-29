---
name: silent-gcode-skip-rmt-truncation
description: User-reported silent gcode-SKIP (multiple internal gaps per run, job continues) — LEADING ROOT CAUSE = silent mid-block RMT truncation (a multi-burst cutting block abandons its tail on a discarded emit_burst Transport error). Host + overflow + reset all ruled out.
metadata:
  type: project
---

**USER SYMPTOM (may outrank the streaming-lockup): streaming silently SKIPS chunks of the cut — MULTIPLE internal
gaps per run, job CONTINUES past each gap, DIFFERENT areas each run (non-deterministic), mostly-complete parts, NO
host/RX evidence.** Doc §14-15 of streaming-lockup-investigation.md. Task #22.

**LEADING ROOT CAUSE (firmware-engineer audit, bughunter-VERIFIED in source 2026-06-26): SILENT MID-BLOCK RMT
TRUNCATION.**
1. A cutting block > `MAX_SYMBOLS_PER_BURST` (=46) step events emits as MULTIPLE RMT bursts — `ceil(steps/46)` of
   them (cnc-kinematics/src/motion.rs:201,231-241). Essentially every real cut >46 steps is multi-burst.
2. Each burst is `sink.emit_burst(&burst)?` (cnc-kinematics/src/motion.rs:232). The `?` ABANDONS all REMAINING
   bursts of the block on ANY Err — the loop stops, the trailing burst never runs.
3. `RmtStepSink::emit_burst` returns `Err(StepError::Transport)` NON-FATALLY from the RMT wait()-completion-error
   arm (firmware/src/motion.rs:408-413: RESTORES the channel `self.channels[axis]=Some(channel)` then errors → a
   RECURRING, non-deterministic hardware event, channel survives). (Weaker fit: the transmit()-start arm :329-340,
   channel lost.)
4. That Err is DISCARDED at run_block: `let _ = generator.run_block_scaled(...)` (firmware/src/motion.rs:786). No
   counter, no breadcrumb, no ALARM.
5. Executor continues to the NEXT block → the TAIL of the cut (every burst after the failing one) is silently
   skipped, job continues.

Fits EVERY symptom (non-deterministic RMT timing, continues by design, multi-gap = every multi-burst block is
vulnerable, no host evidence = line ACKed on core 0 before the block reached core 1). **INVISIBLE to a
blocks-queued-vs-executed probe: BLOCKS_EXECUTED is bumped even on a TRUNCATED block (comms.rs:610/:439) — the skip
is INTRA-block, below block-count granularity.**

DECISIVE EXPERIMENT (air-run primary probe): a `RUN_BLOCK_TRUNCATED` counter on the Err return of run_block_scaled
(motion.rs:786, today `let _ =`), split by Transport source (wait-err :412 cleanest, transmit-start :340 weaker;
EXCLUDE the soft-reset path :1129). >0 tied to a visible gap PROVES it; ==0 across a skipping run exonerates →
runner-up. OBSERVE-ONLY (count, don't change behavior yet).

FIX DIRECTION (after air-run confirms, NOT on source-proof alone): the `let _ =` swallows a real motion fault that
breaks step-sync (lost motion tail = position certainty gone). Per the grbl contract (doc §14.3) it must become
feed-hold→ALARM:N→require-rehome, NEVER silent abandonment. Unifies with the recovery redesign.

WHAT WAS RULED OUT (proven, do not re-chase): resets (skirnir aborts/truncates on a banner = single truncation,
not multi-gap); HOST over-send (skirnireng proof: skirnir can only UNDER-send/disconnect on a dropped ok, never
over-send); RX_PIPE overflow (its precondition — host over-send — is impossible, so my earlier stall→overflow
hypothesis is REFUTED); the BLOCK_AVAILABLE/SLOT_FREED signal lost-wake (audited clean — Signal latches, executor
re-checks queue under lock every loop). Also-real LATENT (not the gap): emit_burst iterates 0..AXES=4 but encodes
only 0/1/2 → axis-3 stale-scratch transmit (benign for X/Y/Z; fix before DOC-10 A-axis). See
[[firmware-two-2s-timeouts]] for the separate lockup/lost-wake thread.
