---
name: firmware-two-2s-timeouts
description: Pikachu "2s drumbeat" = LOST USB TX-done waker (core-0 usb_tx 2s timeout, NOT RMT — refuted). Fix SHIPPED on main (6024126) but INCOMPLETE (Signature A = write-stage flavor it can't catch). SEPARATE user-reported silent gcode-SKIP root cause is now pinned — see [[silent-gcode-skip-rmt-truncation]] (the RX_PIPE-overflow lead here was REFUTED; it's a mid-block RMT truncation).
metadata:
  type: project
---

**NEW TOP PRIORITY (2026-06-26): SILENT GCODE SKIP corrupts CNC parts (may outrank the lockup). Doc §14, task
#22.** User: runs SKIP MULTIPLE chunks PER RUN with the job CONTINUING past each gap (different areas each run).
RESETS EXCLUDED as the cause (bughunter-verified): a mid-stream banner makes skirnir ABORT/TRUNCATE
(core.rs:390-402, AbortQueued) = a SINGLE truncation, job STOPS — CANNOT make N internal gaps + continuation. So
the K-escape/backstop reset is NOT the gap (it truncates). The real skip is a NON-RESET silent line-drop. LEADING
lead (code, to test): RX_PIPE.try_write overflow drop (comms.rs:1221) — on overflow a byte is SILENTLY dropped; a
dropped byte mid-stream MERGES lines / mangles a coord + desyncs host char-count → skip-and-CONTINUE, repeats →
multiple gaps. The pipe "never overflows for a compliant host" BUT a lost-wake stall makes the host keep sending
un-acked → overflow → drop → corrupt line. MAY BE LINKED to the lost-wake wedge (stall→overflow→corruption) =
most parsimonious. Other candidates: over-ack (lower, §9 showed 1:1 healthy); planner/motion block drop. NEXT: a
logged AIR-RUN (line-IN vs ACK vs EXEC + an RX_PIPE-OVERFLOW counter — the key probe; >0 on a skipping run
confirms it) to PIN the mechanism; NO fix on source-proof alone. Direct fix for the overflow path: make overflow
a HARD error:N/ALARM (loud), never a silent byte-drop.

**SEPARATE (still real): the mid-stream software_reset RECOVERY also drops gcode — but produces TRUNCATION, not
the user's internal gaps.** CORRECTION: bughunter's "arm the backstop = pure upside" ruling was WRONG (the timing
proof 6s K-escape<8s backstop was right, but it ignored that a mid-cut reset corrupts the part); the backstop must
NOT be armed for production. FIX DIRECTION (grbl contract): mid-cut recovery = feed-hold→ALARM:N→require-rehome,
NEVER a silent reset (after any reset open-loop steppers lose position certainty, so a silent resume cuts in the wrong
place anyway). TENSION: the §11-13 breadcrumb capture DEPENDS on software_reset (RTC_FAST survives CoreSw); ALARM
doesn't reset = no breadcrumb. PROPOSED SPLIT: diagnostic capture builds keep software_reset+breadcrumb
(operator-gated, scrap part); production ships ALARM+rehome. Team-lead owns the design call.

**FIX IS INCOMPLETE (2026-06-25 night) — the wedge STILL reproduces with the §12 fix LIVE on main (commit
6024126, PR #9).** The deployed recovery (poll-after-arm) fires ONLY at the FLUSH stage. Two new on-board
signatures appeared streaming Pikachu WITH the fix active:
- **Signature A** `[MSG:CRASH usbtx: free=1 empty=0 iena=1 mov=1 exec=0 rdepth=8 n=3 rmt_to=0]` — vs capture #1's
  `iena=0 exec=1`. `iena=1` = the arming ISR NEVER RAN (lost-INTERRUPT flavor, distinct from #1's
  lost-WAKER-after-ISR). The fix can't catch it, verified two ways: (1) `classify_write_stage(write_timed_out,_)`
  returns `Stalled` UNCONDITIONALLY (truncation guard, never consults data_free) so a write-stage stall K-escapes
  even at free=1; (2) STRUCTURAL: esp-hal `flush_tx_async` early-returns when `serial_in_ep_data_free=1`, so a
  free=1 stall CANNOT be flush-stage — it must be WRITE-stage. So Signature A is a write-stage/ISR-never-armed
  lost wake; the flush-stage-only fix structurally misses it. NEXT (task #18): add a `stg=` (write/flush/none)
  breadcrumb field to convert the iena inference into a measurement, re-capture to confirm stg=write/iena=1/exec=0
  is stable, THEN widen recovery to the write stage for the ≤64B single-chunk case ONLY (keep the >64B truncation
  guard). Do NOT drive a fix off one field-set.
- **Signature B** = hard silent lock, NO breadcrumb (Mode C, §2/§6). Custom panic_handler produced nothing.
  STRUCTURAL ROOT of the no-reset (bughunter, doc §13.4): the RWDT is **Stage0=ResetSystem 8s ONLY, fed by a
  SOFTWARE task (watchdog_feed, comms.rs:2990) on core-0's thread-mode executor — NO hw-independent 2nd stage, NO
  SuperWDT.** Both withholds are GATED: comms-withhold needs host_active (RX advanced within RX_ACTIVE_TICKS=12=6s);
  core1-withhold needs EXECUTOR_RUNNING. **DEAD ZONE = host quiet (RX aged out) + exec=0 (queue drained, = Sig A's
  exec=0) → NEITHER withhold fires → dog fed forever → permanent silent lock, no reset, no banner.** B hypotheses:
  B-1 (LEADING, watchdog-mask REDUX) recovered-lost-wakes are is_stall()==false and STILL bump COMMS_PROGRESS
  (comms.rs:1484), limping comms_frozen_ticks under 6 so the dog stays fed — the SAME recovery that breaks A's
  drumbeat masks B's watchdog; B-2 executor death (dog never fed, then §10's USB-no-reenum to explain silence);
  B-3 panic+reset (banner expected, argues against); B-4 brownout (RTC wiped). Discriminate via ONE capture build
  (doc §13.5): ALWAYS-emit reset_reason at boot (CpuSw/CpuRtcWdt ⇒ a reset DID fire = §13.3 (c); ChipPowerOn/Brownout
  ⇒ no sw reset = dead zone/B-4) + a free-running RTC_FAST watchdog_feed heartbeat (climbed-through ⇒ alive-but-fooled
  B-1 / froze ⇒ B-2) + a dead-zone backstop withhold (RESPONSE depth>0 AND no completed usb_tx write >~6s ⇒ withhold
  regardless of host_active/exec). SEPARATE problem from A — do not conflate. Tracked task #19.
See doc §13. Partnering with firmware-engineer (fwengineer) on the repro/flash for this round.

**ROOT CAUSE IDENTIFIED (high confidence, on-board capture 2026-06-25): a LOST USB TX-DONE WAKE (H-A).**
The K-escape build #1 fired (`/tmp/pika2.raw`): `[MSG:CRASH usbtx: ambiguous free=1 empty=0 mov=1 exec=1 rdepth=8 n=3
rmt_to=0]`. Decode (doc §12): `empty=` prints int_raw.serial_in_empty (comms.rs:1040, NOT int_ena); verdict=ambiguous
forces int_ena_armed=0 too. On-board state = data_free=1, int_ena=0, int_raw=0, core-1 healthy (mov=1, beats 942≫138),
RESPONSE full (rdepth=8), parked at tx-write. POST-ISR lost-waker: host drained → serial_in_empty ISR fired on CORE 0 →
ISR cleared int_ena (usb_serial_jtag.rs:941) + int_raw via int_clr (948-953) + called WAKER_TX.wake() → both bits 0.
WriteFuture::poll = Ready iff int_ena CLEAR, so with int_ena=0 the future would complete IF re-polled — but the wake was
lost between WAKER_TX.wake() and the embassy executor re-polling usb_tx. ISR ran ⇒ core 0 NOT starved (not H-B); free=1
⇒ not host-side; rmt_to=0 ⇒ RMT excluded by evidence. The lost link is the embassy/AtomicWaker re-poll in the
esp-rtos/embassy + esp-hal WAKER_TX path. NEXT: confirming re-run of build #1b (adds iena= field + closes the verdict()
gap — fully-serviced-ISR-but-parked leaves both bits 0 and falls through `int_ena_armed||serial_in_empty` → misclassified
Ambiguous; build #1b fix: data_free + rdepth>0 + core-1-healthy ⇒ LostTxWake, +iena= boot field; 16 diag tests green).

EVIDENCE BASE / RARITY (be honest): the fault is RARE/BURSTY — captures #2 AND #3 showed ZERO lost-wake events (clean
Pikachu streams). Those are non-reproductions (silent, neither confirm nor deny). So the root cause rests on a SINGLE
positive capture (#1); confidence is HIGH on the MECHANISM (one consistent explanation for the fingerprint) but the
frequency is one data point. Until a confirm run shows rec>0, describe it as "high confidence, one positive capture,"
NOT "confirmed." rec>0 does DOUBLE duty: fix-confirmation AND the 2nd positive observation of the mechanism (the fix's
recovery path fires only on a real timeout+FIFO-drained = a lost-wake event).

FIX (LANDED, combined usb_tx diff, both Xtensa configs clean): (a) poll-after-arm in usb_tx (in-our-control fix for the
lost re-poll), classified by pure WriteOutcome::classify → Completed/CompletedLostWakeRecovered/Stalled; (b) §11.6
watchdog-mask fix (bump COMMS_PROGRESS only on !is_stall so a real stall stops feeding the dog); + dual recovered-count
readout: live `$I` [MSG:USBTX rec=N] AND boot-persisted RTC_FAST RECOVERED_COUNT (the boot half earns its keep BECAUSE
of the rarity — a partial-fix burst zeroes the live count on the K-escape reset). K-escape RETAINED by construction
(fed by is_stall(); a recovered wake is is_stall()==false so no escalation) ⇒ partial fix still leaves a usbtx
breadcrumb. CONFIRM-RUN TRIAD = rec= climbs past wedge zone + Pikachu completes + NO usbtx breadcrumb; rec=0-completion
is INCONCLUSIVE (rarity), re-run. PARTIAL-FIX next lever (held): bounded CCOUNT re-poll loop vs the single recheck.
upstream suspect: esp-hal WAKER_TX↔esp-rtos multicore wake delivery; the 2s with_timeout stays as a correct backstop.

There are TWO distinct ~2-second timeouts in the firmware, and they were conflated in the streaming-lockup investigation.

**Why:** The 2026-06-25 `128-Pikachu.tap` raw capture (`/tmp/pika.raw`) showed a precise ≈2.00s "drumbeat" (one `ok`
per 2002ms for 8 cycles while the status reporter stayed dead). The team-lead's central hypothesis was that this WAS
the RMT-wait timeout firing in a recovery loop. I refuted it.

The two timeouts:
1. `RMT_WAIT_TIMEOUT_CYCLES = 480_000_000` cycles ÷ 240 MHz = exactly 2000ms — `crates/firmware/src/motion.rs:423`,
   used in `emit_burst` (motion.rs:370-394). On the FIRST timeout it `capture_rmt_hang()` + `software_reset()`s
   UNCONDITIONALLY. No retry loop. One RMT timeout = one whole-chip reset.
2. `USB_TX_TIMEOUT = Duration::from_secs(2)` — `crates/firmware/src/comms.rs:1342`, used in `usb_tx`'s
   `with_timeout(USB_TX_TIMEOUT, { tx.write_all; tx.flush })` (comms.rs:1368-1374). On timeout it drops the response
   and LOOPS (recovers one RESPONSE-channel slot per timeout). Does NOT reset.

**How to apply:** The drumbeat is the `usb_tx` one (#2), not RMT (#1). Proof it's not RMT: the capture is a single
unbroken connection `c0` with the `[VER:1.1f...]` banner appearing exactly once at +45ms and never again across all 8
cycles — a `software_reset()` would have re-bannered or dropped `c0`. So `emit_burst`'s reset never fired during the
drumbeat. The 2.00s *number* matching the RMT constant was a coincidence/trap. See [[streaming-lockup-doc]] §11.

Corollaries proven in the same pass:
- The status-vs-ack asymmetry (status `<...>` dead, `ok` drips) is single-writer/single-channel head-of-line
  starvation: status_responder and the ack path BOTH enqueue into the one depth-8 `RESPONSE` channel
  (comms.rs:4436 / `ack()`), drained by the one `usb_tx` task. NOT a shared-lock or priority issue.
- Why the watchdog didn't reset for 16s: `usb_tx` bumps `COMMS_PROGRESS` once per loop BEFORE the awaited write
  (comms.rs:1354). A 2s-cadence stall keeps the counter advancing every 2s; the comms-stall detector needs 6×500ms =
  3s frozen (`COMMS_STALL_TICKS=6`, `WATCHDOG_FEED_INTERVAL=500ms`). 2s < 3s, so it never trips. The per-loop bump
  MASKS the stall. Candidate fix: bump `COMMS_PROGRESS` only on a COMPLETED write, not on the timeout-drop path.
- The esp-hal USB-Serial-JTAG async write/flush awaits `serial_in_empty` via a single shared `WAKER_TX`, ISR mapped
  to CORE 0 ONLY (`into_async()` runs in main on PRO_CPU → `bind_peri_interrupt`→`enable(Cpu::current())`→
  `core_0_intr_map`). The WriteFuture poll returns Ready iff `int_ena.serial_in_empty` is CLEAR (ISR clears it after
  firing). This write-path logic is IDENTICAL in esp-hal 1.0.0 and 1.1.1 — so the USB stall, if it's the root, is NOT
  a 1.0→1.1 regression (consistent with the wedge existing on both versions).

OPEN (awaiting on-board capture): H-A (lost USB TX-done wake on core 0; executor stays healthy) vs H-B (core-0
starvation; usb_tx is a victim of a core-1 lock/scheduler hazard). Build #1 LANDED 2026-06-25 (tasks #10/#11/#12, both
Xtensa configs clean -D warnings via `just build`, 14 host tests in new pure `firmware-core::diag` module incl. all
verdict cases, clippy-clean). It's the usb_tx K-escape: NOT RTT (defmt shares the one USB-Serial-JTAG with the grbl CDC
— Cargo.toml:73-82 — so no out-of-band RTT on this board; the capture is the in-band RTC_FAST breadcrumb that
self-resets at K=3 ≈6s, readable on next boot via skirnir, survives the post-wedge USB silence). Boot emits
`[MSG:CRASH usbtx: <verdict> free=.. empty=.. mov=.. exec=.. rdepth=.. n=.. rmt_to=..]`.
Snapshot at the K-th timeout (`comms.rs::capture_usb_tx_stall_and_reset`): `ep1_conf.serial_in_ep_data_free`
(PAC esp32s3-**0.35.2**: "0 after WR_DONE until host reads the FIFO" → =1 host DID drain ⇒ device-side fault; =0 host
stopped reading) + `int_raw.serial_in_empty` (event) + `int_ena.serial_in_empty` (armed?) + MOTION_LIVENESS delta +
EXECUTOR_RUNNING + RESPONSE depth + `rmt_wait_timeout_count`.
`diag::verdict()` order: `!data_free→HostNotReading`; else `!motion_advancing && executor_running→Core1Wedged(H-B)`;
else `serial_in_empty→LostTxWake(H-A)`; else Ambiguous. Two H-A FLAVORS (WriteFuture::poll is Ready iff int_ena CLEAR;
ISR clears int_ena on fire): empty=1 + int_ena DISARMED = ISR ran, waker lost (embassy race); empty=1 + int_ena ARMED =
ISR never serviced on core 0 (masked/never fielded — leans core-0 scheduling). `n>=3 && rmt_to=0` = evidence-based RMT
exclusion. BUILD #2 (cross-core PLANNER/MACHINE lock breadcrumb) is GATED: build only if build #1 verdicts
Core1Wedged/Ambiguous (H-B); skip if LostTxWake.

CORRECTION (2026-06-25): the prior "ROOT CAUSE CONFIRMED — core-1 RMT ch0 wait()" (2026-06-24 `axis0:wait_begin`
breadcrumb) is DOWNGRADED to UNPROVEN. That capture was on the UNBOUNDED-wait build (the bounded-wait-that-resets
`8e70c45` landed LATER, 19:21 same day). `stage=axis0:wait_begin` is just the executor's LAST-RECORDED stage, not proof
of a hang — it reads identically whether the executor spins in an unbounded wait OR did that burst last then went idle
while core 0 wedged independently. The watchdog's own verdict was `comms-froze-first` (the prior note overrode it), and
the decisive `[MSG:CRASH rmt0:]` register dump was NEVER captured. A real Mode-A RMT hang may still exist as a separate
rarer mode, but it is NOT the 2026-06-25 drumbeat. **Lesson: a `stage=` last-marker is NOT a hang proof; demand the
register dump before calling an RMT `wait()` the root cause.** See [[streaming-lockup-doc]] §11.9.
