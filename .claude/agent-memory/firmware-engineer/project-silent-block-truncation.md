---
name: silent-block-truncation
description: Audit finding — silent mid-block RMT truncation is the prime suspect for non-deterministic GCode chunk-skip gaps
metadata:
  type: project
---

DEEP-AUDIT FINDING (2026-06-26, for the streaming chunk-SKIP investigation, distinct from the wedge/lockup one):
the prime suspect for the user symptom "streaming silently SKIPS chunks of the cut, continues past each gap,
non-deterministic, multiple gaps/run, parts mostly-complete, NO host/RX evidence" is a SILENT MID-BLOCK RMT
TRUNCATION on core 1.

**Mechanism (all file:line verified against source):**
- A cutting block with step_event_count > 46 (MAX_SYMBOLS_PER_BURST, `cnc-kinematics/src/step.rs:35`) emits as
  MULTIPLE bursts; the generator flushes-then-continues with `sink.emit_burst(&burst)?` at
  `cnc-kinematics/src/motion.rs:232` & `:240` — the `?` abandons ALL remaining bursts of the block on any sink Err.
- `RmtStepSink::emit_burst` returns `Err(StepError::Transport)` from non-fatal, non-resetting RMT paths:
  the `wait()` completion-error arm `firmware/src/motion.rs:408-413` (channel SURVIVES → cleanest fit, scattered
  single-block-tail gaps), and the `transmit()` start-error arm `:329-340` (channel LOST → degrades to persistent
  axis death, fits less well). The reset-abort path `:1129-1130` is the EXCLUDED soft-reset truncation.
- The error is SILENTLY DISCARDED: `let _ = generator.run_block_scaled(...)` at `firmware/src/motion.rs:786`. No
  counter, no crash breadcrumb, no defmt in the production build. Executor falls through and pops the next block.

**Why it stayed invisible:** `BLOCKS_EXECUTED` (`comms.rs:610`/`:439`) is bumped EVEN ON a truncated block, so the
existing `acks vs exec` cross-check reads CLEAN during a skipping run.

**Decisive next experiment — BUILT + FLASHED + VERIFIED LIVE on HW 2026-06-26 (steppers disconnected = zero risk; the
block path still runs with steppers off so truncation still reproduces at the instrumentation level).** The counter is
split: `RUN_BLOCK_TRUNCATED` (total) at the former `let _ =` site (now captures the Result via a new
`RmtStepSink::take_last_error()` / `last_error: Option<TruncationSource>` field set on each emit_burst Err arm), plus
per-source `twait` (wait-err arm, prime suspect) / `ttx` (transmit-start, channel-lost) / `tlong` (BurstTooLong,
encoder bug), plus `taxis` = last truncation axis+1 (4⇒A stale-scratch). Surfaced UNCONDITIONALLY on `$I`:
`[MSG:SKIP drop= lines= cons= acks= exec= trunc= twait= ttx= tlong= taxis=]`. Added `LINES_CONSUMED` (cons) for the
over-ack null-check (acks>cons ⇒ firmware over-ack). OBSERVE-ONLY: zero behavior change (block still
abandon-and-continued; the ALARM is the FIX, gated on trunc>0 confirmed ≥2 runs tied to gaps). 309 host tests green,
both Xtensa configs clean -D warnings. Baseline `$I` confirmed emitting the line on HW. Non-zero `trunc` correlated
with a visible gap = §15 PROVEN; zero + clean = exonerates firmware → §16.

**§16 CO-LEADING ALTERNATIVE (lead elevated 2026-06-26 — firmware may be INNOCENT):** the user's STEPPERS ARE
DISCONNECTED, so they see ONLY skirnir's on-screen render. commit `8b4e08c` made the live toolpath trail cuts-only +
status-SAMPLED (~10Hz), non-deterministic by construction → could produce the exact internal-gap symptom with NO
firmware bug. So the air-run is a clean A/B decider: `trunc>0` (≥2 runs, tied to gaps) = §15 firmware (real lost
motion); `trunc==0` + all firmware counters clean + gap STILL on screen = §16 render (firmware innocent). A clean
firmware result POSITIVELY routes to render, not "ambiguous." [[project-skirnir-render-artifact]] (skirnir-engineer's
domain).

**DECODE (bughunter owns; send raw COUNTERS, do NOT self-classify):** (1) trunc>0 + lines==acks==exec + drop=0 ⇒ §15
truncation (per-source=which arm; taxis=if A); (2) drop>0 ⇒ contradicts skirnireng, re-open host/byte; (3) acks>exec
(trunc=0) ⇒ whole-block dual-core drop; (4) lines<expected ⇒ framer/RX corruption; (5) acks>cons ⇒ firmware over-ack;
(6) all clean + gap seen ⇒ §16 render.

**Ruled CLEAN by the same audit:** BLOCK_AVAILABLE/SLOT_FREED lost-wake (embassy Signal latches; executor re-checks
queue under lock every turn at `motion.rs:569`, awaits only in empty branch `:639`; all 5 enqueue sites signal). The
axis-3 stale-scratch transmit (`emit_burst` iterates 0..AXES=4 but only encode_channel(0/1/2); scratch[3] stays all
end_markers so ch3 instant-stops — benign for X/Y/Z, but a REAL future-bug once DOC-10 A-axis is driven).

Related: [[project-firmware-lockup-investigation]] (the wedge/HARD-lockup modes — a DIFFERENT failure than these
silent gaps), [[project-motion-executor]], [[project-rmt-clock-and-tx-completion]].
