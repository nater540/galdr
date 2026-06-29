---
name: project-firmware-lockup-investigation
description: streaming-lockup investigation. 2026-06-24 (esp-hal 1.0): RMT ch0 wait() hang (Mode A). 2026-06-25 (esp-hal 1.1.1): REFRAMED — the 2.00s drumbeat is NOT the RMT timeout (that resets on FIRST fire); it matches USB_TX_TIMEOUT, the wedge is a USB-Serial-JTAG TX-drain stall + RESPONSE-channel head-of-line blocking. Plus the watchdog/breadcrumb/task-watchdog build and what was ruled out.
metadata:
  type: project
---

**§17.5 FLASHED + CAPTURE PASS #1 (2026-06-28).** `capture-reset` image built clean (both default+`capture-reset`
Xtensa configs, `-D warnings`) and flashed to `/dev/cu.usbmodem31101` via PLAIN `espflash flash` (NO `--monitor` — the
runner's baked-in monitor bypassed so no DTR/RTS reattach wipes RTC_FAST). Pikachu pass `--timeout 1800` ran HEALTHY
the full 30 min (acks 0/46→3243/3279, WPos→564.6, `Run`, NO mid-stream breadcrumb) then skirnir exited on the HARD
1800 s wall-clock cap (NOT the 12 s idle-timeout). The post-timeout boot dump's `[MSG:CRASH usbtx: host-not-reading
free=0 … wstg=1 rlen=134 n=3]` + `[MSG:RESET core-sw-reset]` is a HOST-ABANDONMENT ARTIFACT, NOT Signature A: when
skirnir hit `--timeout` it stopped reading the port mid-execution, `usb_tx` saw `free=0` for K=3 and the capture-build
K-escape `software_reset()`d. **`free=0` ⇒ verdict `host-not-reading`; Signature A needs `free=1` (host still reading).**
TIER 1's guards held CORRECTLY (`free=0` AND `rlen=134>64` both failed the §17.1 recovery test ⇒ `Stalled`, `rec=` never
climbed — no mis-recovery). VERDICT: INCONCLUSIVE for the #20 confirm (host left first on the wall clock, never reached
the lossless-streaming window). **Pikachu needs ~2483 s just to ACK + execution lags acks (RMT paces pulses at feed rate
even with steppers disconnected — completed≠executed), so 30 min CANNOT finish it. Re-run with `--timeout ≥3600`, keep
`--idle-timeout 12` as the real wedge detector (it did NOT trip during healthy streaming).** Raw logs:
`/tmp/cap_baseline.raw`, `/tmp/cap_capreset_p1.raw`, `/tmp/cap_p1_after.raw`.

**§17.6 CAPTURE PASS #2 (2026-06-28) — TIER 1 CONFIRMED, FULL CLEAN COMPLETION.** Re-ran the same `capture-reset` image
with `--idle-timeout 12 --timeout 3600`. The ENTIRE 4474-line Pikachu repro (the reliable Signature-A reproducer) ran
END-TO-END in ~42.5 min, `outcome=Completed`, all 4474 acked, `Idle` at origin, 0 errors/0 alarms. NO `[MSG:CRASH
usbtx:]`/`[MSG:RESET]`/`[MSG:BOOT]` in the stream OR on the post-run reconnect; board did NOT reset (K-escape never
armed) and RTC_FAST is now CLEAN (the pass-1 artifact breadcrumb cleared). On-board free-running counter post-run:
`MSG:SKIP … lines=4477 cons=4477 acks=4476 exec=4508 trunc=0 twait=0 ttx=0 taxis=0` — every line received+planned, 4508
blocks EXECUTED (arcs subdivide), `trunc=0` (motion.rs:786 landmine never fired), `twait=0` (Mode A never fired), `ttx=0`
(the Signature-A drumbeat NEVER STARTED). TRIAD: completion + no usbtx crash = the two decisive legs PASS; `rec=` did NOT
climb but ONLY because there was no wedge to recover (`ttx=0`), not a missed recovery. **VERDICT: TIER 1 confirmed on the
common path — full Signature-A reproducer no longer wedges / no part-corrupting `software_reset()`.** Caveat: did not
FORCE a single-chunk lost-wake, so `rec>0` increment itself is unobserved on HW (only reachable if a residual A-wedge
recurs); the §17.1 recovery logic stays host-tested. Raw logs: `/tmp/cap_capreset_p2.raw`, `/tmp/cap_p2_after.raw`.
NEXT: more confirming Pikachu passes to bound residual rate; restore `T1_Test.tap` (NOT on disk) for #21 Signature B;
NOT committed, NOT a production flash (diagnostic `capture-reset` image).

**§17.7 PASS #3 REPRODUCED A SIGNATURE-B HARD SILENT WEDGE (2026-06-28). #21 STILL OPEN + UNTRACED + REPRODUCES.** Same
flashed `capture-reset` image. Pass 3 WEDGED at ~26.7 min, ~line 2991/4474 (`outcome=IoDisconnect` host code 4, `[down]
controller not responding`). NOT Signature A: acks climbed SMOOTHLY to the disconnect (NO 2 s usb_tx drumbeat), and the
board went silent MID-WRITE — last bytes were a status/`$I` response TRUNCATED exactly at `[MSG:SKIP … exec=4508 trunc=`
(64 B chunk ends mid-word, nothing follows). THREE skirnir-only reconnects over ~1 min ALL = `Connecting→Disconnected`,
ZERO inbound bytes; port `31101` stayed enumerated (USB alive in silicon) but firmware emits NOTHING — no banner, no
`[MSG:CRASH]`, no `[MSG:RESET]`; **RWDT did NOT recover within ~1 min** (the §13.4 watchdog dead zone + original
"EN-button-only" report). The K-escape NEVER fired (it counts usb_tx TIMEOUTS in a loop; a mid-write deadlock/halt never
returns to that loop), and the armed dead-zone backstop produced NO trace either — so a `capture-reset` image CANNOT
capture B (nothing alive to write the breadcrumb). Possible clue (not proof): `lines=4483 cons=4479` = 4-line RX-vs-parser
gap → core-0 consumer/comms may have frozen while RX buffered (Mode-B `comms-froze-first` family) but the task-watchdog
didn't catch it; core-0-vs-core-1 origin UNDETERMINED. AGGREGATE on this build: clean full Pikachu completions = 1 (pass
2); pass 1 = host-timeout artifact; pass 3 = B wedge. Residual HARD-wedge rate ≈ ≥1 in ~3 real attempts (non-det).
Signature A did NOT recur in 2-3 (TIER 1 target clean). NEXT for #21: B needs a DIFFERENT capture channel than in-band
RTC-on-self-reset — (a) §13.7 always-on RTC boot-count/reset-reason/heartbeat read on NEXT power cycle (but B isn't
self-resetting, and a forced EN/espflash reset WIPES RTC — §10 trap), (b) HARDEN the RWDT to fire in the dead zone (then
B → reset+boot-dump), or (c) out-of-band JTAG/RTT. Board is CURRENTLY WEDGED (silent, port enumerated) — needs physical
EN/power reset, which wipes RTC, so no breadcrumb survives anyway. Raw: `/tmp/cap_p3.raw`, `/tmp/cap_p3_after.raw`,
`/tmp/cap_p3_retry1.raw`, `/tmp/cap_p3_retry2.raw`. Pass 4 NOT run (stopped early on the wedge). NOT committed/flashed.

**§17 PRODUCTION RECOVERY REDESIGN — TIER 1/2/3 LANDED (2026-06-26, firmware-engineer; uncommitted, build-verified,
NOT flashed).** Doc `docs/streaming-lockup-investigation.md` §17 is authoritative. Three changes, all pure logic
host-tested in firmware-core; 314 firmware-core host tests green; ALL FOUR Xtensa configs (default / `defmt` /
`capture-reset` / `defmt,capture-reset`) clean under `RUSTFLAGS="-C link-arg=-Tlinkall.x -D warnings"`:
- **TIER 1 (the part-corruption fix, §13.1):** `firmware_core::diag::WriteOutcome::classify_write_stage` is now
  4-arg `(write_timed_out, write_errored, resp_len, data_free)`; on a WRITE-stage timeout returns
  `CompletedLostWakeRecovered` (drop+continue, bump `rec=`, reset K-escape) IFF `resp_len<=SINGLE_CHUNK_MAX_BYTES(=64)
  && data_free`, else `Stalled`. ≤64 B = one `write_async` chunk, fully pushed before park ⇒ drained FIFO proves bytes
  out ⇒ cannot truncate. Exactly the captured Signature A (`wstg=1 rlen=4 free=1`). >64 B / fifo-not-free stay
  `Stalled` (truncation guard intact). Wired in `comms.rs::usb_tx`: re-read `serial_in_ep_data_free` on a write
  timeout BEFORE classify, pass `resp.len()`. STOPS the K-escape `software_reset()` firing on the common mid-cut wedge
  in BOTH builds.
- **§15.6/#22 ALARM:** new `AlarmCode::MotorFault` = grbl code **17** (grblHAL `Alarm_MotorFault`; non-colliding),
  `is_locked()` (require soft-reset+re-home), prompt `'$H'|'$X' to unlock`. New `MOTION_FAULT` signal raised in
  `motion.rs::run_block` on `Some` `emit_burst` source (WaitError/TransmitStart/BurstTooLong — `None`=InvalidConfig
  all-or-nothing NOT raised); consumer races it in its main `select` exactly like `HARD_LIMIT_TRIPPED` →
  `Alarm(MotorFault)`+`emit_alarm`+`reset_pipeline`, guarded by `hard_limit_alarm_applies()`. CLEAN increment on the
  hard-limit flow — no executor quiesce rewrite.
- **TIER 2/3 split = `capture-reset` Cargo feature (Option A).** ON (diagnostic): K-escape captures+`software_reset()`
  AND dead-zone backstop ARMED (`DEAD_ZONE_BACKSTOP_ARMED=true`) — the #20/#21 capture channel. OFF (production
  default): K-escape raises the SAME `ALARM:17` via `MOTION_FAULT` and RETURNS (clears stall run), dead-zone DISARMED.
  Compile-time, zero runtime branch. `handle_usb_tx_wedge` has two `#[cfg]` variants; `capture_usb_tx_stall_and_reset`
  + `crash::record_usb_tx_stall` (WRITER) gated to capture build; breadcrumb DECODE/boot-dump stays unconditional.
- **OPEN DESIGN NUANCE flagged to team-lead (NOT silently extended):** only the dead-zone backstop is feature-gated;
  the watchdog's `core1_wedged`/`comms_wedged` withholds STILL force RWDT reset in production (a genuinely-dead task
  can't ALARM; a bricked board mid-job is worse than a reset). Whether to also convert those to a fail-safe halt is a
  follow-up the lead owns.

**⚠️ TOP FINDING 2026-06-26 — THE AUTO-RESET RECOVERY SILENTLY CORRUPTS PARTS (outranks the lockup).** User reports
recent runs no longer hard-lock but SKIP ENTIRE CHUNKS of the cut. ROOT, mechanism proven in code (both legs): a
mid-stream `software_reset()` — the shipped usb_tx K-escape (commit 6024126, fires ~6 s into an A-wedge) AND the
dead-zone backstop if armed — (1) loses the host's in-flight character-count window (bytes hit the rebooting
USB-Serial-JTAG FIFO) + resets parser/modal state, AND (2) skirnir's stream engine (`crates/skirnir/src/protocol/
core.rs:17-18`, `:391` AbortQueued/DiscardProgram) treats the mid-stream boot banner as a controller reset →
ABORTS + discards in-flight lines, does NOT re-send. ⇒ every mid-cut auto-reset drops ~the in-flight chunk = a part
that LOOKS done but has missing toolpaths. STRICTLY WORSE than a visible hard-lock (which you'd scrap). So the SHIPPED
6024126 K-escape is already corrupting parts on any real-cut A-wedge. (No smoking-gun log: my /tmp raw logs show NO
mid-stream reset — but they're older fw / timeout-capped-while-healthy, not the skipping runs; the mechanism is airtight
regardless.) DECISION HELD: do NOT arm the dead-zone backstop (it's another silent-reset path); bughunter retracted the
earlier "arm=pure upside" ruling. AGREED 3-TIER REDESIGN (fwengineer-2 + bughunter; team-lead owns final call, task #22):
TIER 1 PREVENT = the §13.1 single-chunk widening recovers the lost-wake IN PLACE (drop+continue, NO reset, when
resp.len()≤64 && data_free=1 — bytes already delivered) → the primary corruption fix, stops the K-escape firing mid-cut;
TIER 2 FAIL-SAFE = a genuine unrecoverable wedge → feed-hold + ALARM:N + require-rehome (grbl's lost-step-sync contract),
NEVER a silent software_reset the host streams through; TIER 3 DIAGNOSTIC = instrumented capture builds keep
software_reset+breadcrumb (operator-gated, scrap expected) so we don't lose the A/B capture channel — production ships
tiers 1+2 only. ORDER: land the widening first (needs a wstg=1/rlen≤64 capture to green-light), then convert the residual
K-escape/backstop reset path to ALARM. The wstg/rlen + passive-B instrumentation (below) is BUILT + verified (309 host
tests, both Xtensa configs clean -D warnings, backstop gated `DEAD_ZONE_BACKSTOP_ARMED=false`) but NOT flashed for a real
capture pending the lead's design call; board held at /dev/cu.usbmodem31101 (fwengineer-2 sole owner).

**FIX MERGED + TWO RESIDUAL SIGNATURES (post-merge, bughunter-2 driving; capture-only phase, NO new fix yet).** The
lost-wake fix is COMMITTED & MERGED (commit `6024126` "Fix USB-TX lost-wake streaming lockup" / PR#9; HEAD now `cc89e47`).
But it is INCOMPLETE — two distinct signatures survive on `main`:
- **SIGNATURE A (task #18, diagnosis CONFIRMED) = a WRITE-stage lost wake the deployed flush-only recovery cannot catch.**
  Reproduced WITH the fix live: `[MSG:CRASH usbtx: lost-tx-wake free=1 empty=0 iena=1 mov=1 exec=0 rdepth=8 n=3 rmt_to=0]`
  (distinct from capture #1's iena=0 exec=1). TWO convergent proofs it's write-stage: (1) `classify_write_stage(write_timed_out
  =true,..)` returns `Some(Stalled)` UNCONDITIONALLY (never consults data_free) so flush-stage-only recovery can't fire; (2)
  my proof: `flush_tx_async` (usb_serial_jtag.rs:826-835) EARLY-RETURNS Ok when serial_in_ep_data_free is SET — it only parks
  the WriteFuture when data_free is CLEAR, so a free=1 stall is necessarily a write_all park, NOT flush. iena=1 is SECONDARY
  (ISR-never-ran flavor), NOT load-bearing — the gap is STAGE not flavor. Fix gap: recovery only fires at the flush stage,
  but Sig A stalls at the write stage. FUTURE FIX (task #18 item 3, NOT yet built): recover at the WRITE stage ONLY when the
  response is ≤64B (single chunk, no mid-write truncation possible) AND data_free=1 — a ≤64B ok/error is whole-or-nothing.
  STATUS (2026-06-25, fwengineer): the full wstg instrumentation is DONE-but-UNCOMMITTED + build-verified, awaiting flash.
  `diag.rs` write_stage_stall bit (UsbTxStall field + WRITE_STAGE_STALL=1<<5; count shrunk 7→6-bit sat 63, depth nibble
  5→6, count shift 9→10; round-trip tested, 28 diag tests / 308 firmware-core total). comms.rs wiring COMPLETE:
  `stall_at_write_stage = write_timed_out` threaded into capture_usb_tx_stall_and_reset; boot line gained `wstg=` (after
  iena). Both Xtensa configs clean under `RUSTFLAGS="-C link-arg=-Tlinkall.x -D warnings"`. Predicted re-capture: wstg=1
  (write-stage) stable; a wstg=0 with free=1 would contradict both proofs. BUILD GOTCHA reconfirmed: bare
  `RUSTFLAGS="-D warnings"` clobbers the crate config's `-Tlinkall.x` → flood of undefined-reference LINK errors; use
  `just build`, or append `-C link-arg=-Tlinkall.x` to RUSTFLAGS.
- **SIGNATURE B (task #19, leading hypo B-1) = a SILENT TOTAL LOCK — no [MSG:CRASH], no banner, no self-reset, skirnir can't
  reconnect (T1_Test.tap ~14min, after 6024126).** Currently INVISIBLE. ROOT of invisibility: RWDT is Stage0-only, fed by the
  SOFTWARE watchdog_feed task; its withholds are gated — comms-stall needs host_active (RX within 6s), core-1 needs
  EXECUTOR_RUNNING. DEAD ZONE: host quiet (RX aged out) + executor idle (exec=0) → neither withhold fires → dog fed forever →
  permanent silent lock. B-1 (leading, watchdog-mask REDUX): comms.rs:1487 `if !outcome.is_stall() { COMMS_PROGRESS+= }` — a
  RECOVERED lost-wake is !is_stall() so it STILL bumps COMMS_PROGRESS; intermittent recoveries keep comms_frozen_ticks<6 so the
  dog never withholds (the §11.3 defect re-introduced by the fix). Competing: B-2 executor-death, B-3 panic, B-4 brownout.
  B-INSTRUMENTATION (task #19, NOT yet built): (1) ALWAYS route reset_reason into the grbl [MSG:CRASH] line — note
  log_reset_reason() (main.rs:391) ALREADY reads+labels it but emits only via esp_println (raw channel), NOT the grbl TX
  skirnir parses; (2) free-running RTC_FAST heartbeat bumped by watchdog_feed (climbed-through ⇒ B-1 dog-fooled; froze ⇒ B-2);
  (3) DEAD-ZONE BACKSTOP withhold: RESPONSE depth>0 AND no COMPLETED usb_tx write for >~8s ⇒ withhold REGARDLESS of
  host_active/exec (instrumentation-grade — converts the silent lock into a breadcrumb-bearing reset). OWNER: bughunter-2
  (diagnosis/decode/flash) + fwengineer (impl). Repro: 128-Pikachu.tap → A ~30min; T1_Test.tap → B ~14min.
- CAPTURE-ONLY BUILD LANDED (uncommitted, 2026-06-25→26; both Xtensa configs clean -D warnings, 309 host tests, ELF symbols
  verified; NO fix — bughunter-2 directed combined A+B, reset_reason FIRST). Files: diag.rs, comms.rs, crash.rs, main.rs (all
  uncommitted). SIG A (was already wired in tree): `wstg=` bit threaded from the write-vs-flush stall into
  capture_usb_tx_stall_and_reset + printed on the usbtx boot line — a recorded fact, not iena inference. SIG B (3 items):
  (1) `send_reset_reason` emits `[MSG:RESET <label>]` over grbl TX UNCONDITIONALLY after the banner (main.rs log_reset_reason
  now returns (label,bool)) — settles "a reset DID fire (sw/rtc-WDT)" vs "no reset (power-on/brown-out/dead-zone)" even on a
  no-breadcrumb boot; (2) free-running RTC_FAST `WATCHDOG_HEARTBEAT` word bumped by watchdog_feed each loop, printed `wdog=N`
  on the crash summary — climbed-through ⇒ B-1 (feed alive but fooled), froze ⇒ B-2 (feed died); (3) DEAD-ZONE BACKSTOP:
  new `USB_TX_COMPLETED` counter (usb_tx-specific, bumped on !is_stall — distinct from COMMS_PROGRESS which 3 tasks bump and
  recovered-lost-wakes keep alive), tracked in watchdog_feed as tx_complete_frozen_ticks; host-tested
  `diag::dead_zone_withhold(response_depth, frozen_ticks)` = `depth>0 && frozen>=DEAD_ZONE_STALL_TICKS(16≈8s)` → new
  `WithholdReason::DeadZone` ("dead-zone-silent-lock") forces a breadcrumb-bearing reset REGARDLESS of host_active/exec —
  closing the dead zone that left B invisible. Diff handed to bughunter-2 for review before flash; bughunter-2 owns flash + decode.

**FIX LANDED 2026-06-25 (team-lead GO; both Xtensa configs build clean -D warnings, 300 firmware-core host tests green,
clippy-clean, ELF verified to contain the symbols/strings — NOT yet committed; commit decision stays with team-lead/user
after HW fix-confirmation).** The combined `usb_tx` patch is now IN THE TREE (no longer held):
- POLL-AFTER-ARM recovery (fix a): on a 2s `with_timeout` timeout, re-read `ep1_conf.serial_in_ep_data_free`; SET ⇒
  recovered lost-wake (reset stall run, bump `USB_TX_LOST_WAKE_RECOVERED` + mirror to RTC_FAST, NO escalate) — breaks the
  drumbeat without a reset; FIFO-still-full ⇒ genuine `Stalled` ⇒ K-escape (RETAINED as the host-not-reading / partial-fix
  backstop). Driven by host-tested `firmware_core::diag::WriteOutcome::classify`.
- FINAL COMMIT BUILD = truncation hardening + explicit write-error handling (LANDED 2026-06-25; both Xtensa configs clean
  -D warnings, 307 host tests, ELF has `classify_split` + `classify_write_stage`). The usb_tx write stage is now a 3-way
  match via host-tested `WriteOutcome::classify_write_stage(write_timed_out, write_errored) -> Option<WriteOutcome>`:
  write TIMEOUT ⇒ Some(Stalled) (possibly-mid-write, never recover); write ERROR (`Ok(Err)` host-closed) ⇒ Some(Completed)
  (clean drop-and-continue, NOT a stall, never recovered-counted — restores the pre-split `let _ = with_timeout` leniency
  that discarded write errors); clean ⇒ None → proceed to time+classify the FLUSH stage. (+3 host tests; team-lead asked
  for the write-error test explicitly. For USB-Serial-JTAG a host close usually surfaces as a write TIMEOUT not Ok(Err),
  so the error arm is rare, but explicit handling keeps "write-error ≠ stall" unambiguous.)
- TRUNCATION-SAFETY HARDENING — LANDED 2026-06-25 (team-lead review found the edge; team-lead GO'd option 2; both Xtensa
  configs build clean -D warnings, ELF has `classify_split`). DECODE-TIME (bughunter, for reading a confirm-run breadcrumb
  with the fix ACTIVE): post-split, `rec=` counts ONLY flush-stage recoveries (the captured case IS flush-stranded — an `ok`
  = 4B = single chunk, so write_async never parks mid-write_all for it → recovers identically, rec= semantics + triad +
  decode plan UNCHANGED). A usbtx breadcrumb with the fix active is therefore NOT automatically "fix failed": its VERDICT
  field disambiguates — `lost-tx-wake` + `free=1` on a SHORT line = recovery genuinely missed a case (→ tighten with the
  bounded re-poll loop); a mid-write_all strand on a LONG (>64B) line = the truncation-safety split working AS DESIGNED
  (Stalled→K-escape refusing to recover unwritten bytes — a different, lower-priority follow-up, NOT a recovery failure).
  The original fix treated ANY
  timeout-with-`data_free=1` as fully recovered, but `write_async` (esp-hal usb_serial_jtag.rs:811-824) parks the future
  BETWEEN 64-byte chunks. So a >64B response (full status ~90B, long `[MSG:]`) whose lost-wake strands write_all AFTER chunk
  1 has `data_free=1` (host drained chunk 1) yet UNWRITTEN remaining bytes → would classify recovered → advance → TRUNCATED
  line. Severity was LOW (ok/error/short status ≤64B = single chunk, never parks mid-way, so the flow-control-critical
  ok-path NEVER truncated; only >64B lines in the rare² window, dropped by skirnir + self-corrected) — but real, so closed
  before commit. FIX (option 2): `usb_tx` now times write and flush SEPARATELY — `with_timeout(tx.write_all(..))` then (only
  if that didn't time out) `with_timeout(tx.flush())`; recover ONLY at the FLUSH stage (bytes provably all in the FIFO — the
  captured case); a MID-write_all timeout ⇒ `Stalled` (possibly-unwritten bytes, not advanceable) ⇒ counts toward K-escape
  (a clean reset beats silent truncation). Encoded in host-tested `firmware_core::diag::WriteOutcome::classify_split(write_to,
  flush_to, data_free_after_flush)` (write_to short-circuits to Stalled, else defers to `classify`); `classify` UNCHANGED. +4
  host tests (mid-write→Stalled regardless of args; flush+data_free→recovered; flush+fifo-full→Stalled; clean→Completed).
  NOTE: a USB write ERROR (host closed, `Ok(Err)`) now flows to the flush (which times out → stall → eventual reset/
  re-banner) — same leniency as the prior `let _ = with_timeout(..)` that discarded write errors; not a regression. (Old note
  preserved below for the pre-hardening reasoning.) FIX = team-lead's option 2,
  verified correct: SPLIT the write — `with_timeout(write_all)` then `with_timeout(flush)` separately; recover ONLY at the
  FLUSH stage (bytes provably all in the FIFO — the captured case); a MID-write_all timeout ⇒ `Stalled` (unwritten bytes,
  not advanceable) ⇒ counts toward K-escape (a clean reset beats silent truncation). `WriteOutcome::classify` UNCHANGED
  (still host-tested); only the usb_tx write/flush sequencing changes + 1 new host test (write-stage vs flush-stage timeout →
  outcome). Held until team-lead pings post-flash (they said do NOT touch the tree while they build from it for the confirm).
- WATCHDOG-MASK (fix b, §11.6): `COMMS_PROGRESS` bump moved from before-write to `if !outcome.is_stall()` — a genuine
  stall no longer feeds the dog. Other two bumpers untouched.
- DUAL READOUT (team-lead asked for BOTH): live `$I` → `[MSG:USBTX rec=N]` (N>0 only, via `format_usb_tx_recovered` in
  send_build_info); boot → new RTC_FAST word `RECOVERED_COUNT` (crash.rs idx, `record_recovered_count`, decoded into
  `Breadcrumb.recovered_count`, emitted in maybe_emit_crash_report; CRASH_REPORT Vec 5→6). The boot half matters BECAUSE
  the bug is RARE/BURSTY (team-lead gap analysis: captures #2/#3 had ZERO lost-wake events, max RX gap 122/133ms) — a burst
  that recovers some wakes then still wedges (one slips through → K-escape reset) would lose the live count, so it's
  mirrored to survive the reset. (I'd earlier judged the boot line low-value; the bursty finding changed that — the
  team-lead was right to want both.)
- CONFIRM-RUN is INCONCLUSIVE on a quiet run: the bug is bursty/rare, so most runs read rec=0 (NOT a failure). Confirm via
  accumulating rec>0 over time / catching a burst. TRIAD = rec= climbs past the historical wedge zone (~line 400-800+) +
  Pikachu streams to COMPLETION + NO `[MSG:CRASH usbtx:]` breadcrumb on next boot. Partial fix = rec>0 AND a usbtx
  breadcrumb (recovery missed a case). Root cause was established on capture #1 + the airtight esp-hal-source decode, so
  the fix ships on that basis (it's a no-op when healthy); grinding for a 2nd wedge was low-yield (2 clean runs).
- HELD PARTIAL-FIX CANDIDATE (NOT built — only if the confirm run shows `lost-tx-wake` + boot `rec>0` = recovery missed a
  case): replace the SINGLE post-timeout `serial_in_ep_data_free` recheck with a BOUNDED RE-POLL LOOP (poll the bit a few
  times over a short window before declaring a stall) to catch a wake that lands microseconds after the one recheck.
  CAUTION (bughunter): keep the window SHORT and tightly bounded so a genuinely-non-draining host still escalates to the
  K-escape promptly rather than spinning. usb_tx is on the core-0 THREAD executor (it yields) so embassy `Instant` is fine
  here — no CCOUNT needed (unlike the core-1 RMT busy-spin) — but bound it tightly regardless. Single recheck FIRST; widen
  only on evidence of a slipped-through case.

**ROOT CAUSE = LOST USB TX-DONE WAKE (H-A) — HIGH CONFIDENCE, but ONE positive on-board capture (epistemic flag, bughunter
as root-cause owner, recorded in doc §12).** The diagnosis rests on a SINGLE positive capture (#1) + the airtight esp-hal-
source decode; captures #2 AND #3 were ZERO-event non-reproductions (clean streams, max RX gap 122/133ms) — they neither
contradict NOR corroborate it (silent). So describe it as "high confidence, one positive capture," NOT "confirmed N times,"
until a second positive lands. The confirm run's `rec>0` does DOUBLE DUTY: it confirms the fix AND is the SECOND independent
positive observation of the lost-wake mechanism (the recovery path triggers only on a genuine timeout+FIFO-drained = the
lost-wake event itself), raising the root cause to two data points. A confirm run that completes with rec=0 is DOUBLY
inconclusive (neither confirms the fix nor adds a data point) — re-run until rec>0 (may take several, given #2/#3 were zero-
event). The supporting capture #1 detail:

**Capture #1 (2026-06-25) = the one positive observation.** Build #1 flashed + Pikachu streamed; the
K=3 escape self-reset ~6s into the stall and the boot dump read (over skirnir-only CDC):
`[MSG:CRASH usbtx: ambiguous free=1 empty=0 mov=1 exec=1 rdepth=8 n=3 rmt_to=0]`. Decoded (bughunter + me): data_free=1
(host DID drain the FIFO), int_raw.serial_in_empty=0 AND int_ena.serial_in_empty=0 (the ISR RAN — it clears both — and
called WAKER_TX.wake()), core 1 healthy (mov=1), RESPONSE full (rdepth=8), `rmt_to=0` (the RMT path NEVER fired — RMT
theory POSITIVELY excluded by evidence). MECHANISM (verified vs esp-hal 1.1.1 `UsbSerialJtagWriteFuture::poll`,
usb_serial_jtag.rs:736-746 — returns Ready iff int_ena is CLEAR): the ISR fired, cleared int_ena, woke WAKER_TX — but the
embassy executor never re-polled usb_tx, so the future stayed Pending forever (would've completed if re-polled). It's a
CORE-0-LOCAL async waker re-poll race in the esp-rtos/embassy + esp-hal WAKER_TX path — NOT H-B (mov=1 + ISR-serviced
rules out both a core-1 wedge AND core-0 starvation), NOT RMT, NOT host-side. The 2.00s drumbeat = USB_TX_TIMEOUT firing
in a loop (each 2s rescue re-polls once, completes that one write, re-parks). BUILD #1b (landed, TDD, 16 host tests, both
Xtensa configs clean): closed the verdict() gap — `data_free && rdepth>0` now classifies as LostTxWake (the captured
both-bits-clear state was falling to Ambiguous); added raw `iena=` to the boot line so the H-A flavor reads directly.
NEXT: a confirming RE-RUN of #1b (verify free=1/empty=0/iena=0/mov=1 reproduces — one early-fire shouldn't be the sole
basis), THEN the fix. Do NOT build #2 (cross-core lock breadcrumb) — the capture says NOT H-B.

**FIX DRAFTED 2026-06-25 (TDD, NOT landed until capture #3 confirms; held as a diff, working tree kept at capture-only
build #1b so a `just flash` for capture #3 still fires the breadcrumb).** Fix (a) + (b) COMPOSE into ONE coherent
`usb_tx` patch (the WriteOutcome classification drives both): the pure host-tested logic is LANDED INERT in
`firmware-core::diag` (`WriteOutcome::{Completed, CompletedLostWakeRecovered, Stalled}` + `classify(timed_out,
data_free_after_timeout)` + `is_stall()`/`is_recovered_lost_wake()`; 4 new tests, 20 diag tests total — it's `pub` dead
code until wired, does not change the #1b binary). The WIRING (held as a diff, NOT in the tree):
- POLL-AFTER-ARM recovery (fix a): on a `with_timeout` TIMEOUT, re-read `ep1_conf.serial_in_ep_data_free`; if SET, the
  host drained the FIFO so the bytes are out and only esp-hal's wake was lost ⇒ `CompletedLostWakeRecovered` (resets the
  stall run, bumps a new `USB_TX_LOST_WAKE_RECOVERED` diagnostic AtomicU32) — breaks the drumbeat without a reset. Only a
  timeout with the FIFO STILL FULL is `Stalled` and counts toward the K-escape (now the backstop for a REAL host-not-
  reading wedge, which would read `host-not-reading` not `lost-tx-wake`). We can't fix esp-hal's internal future from the
  task; this layers a polling backstop over its event-driven wait. (Upstream esp-hal WAKER_TX↔esp-rtos wake delivery is a
  noted investigation, NOT the shipped fix.)
- WATCHDOG-MASK fix (b, §11.6, correct on its own merits): move the `usb_tx` COMMS_PROGRESS bump (comms.rs:1400) from
  BEFORE the write to AFTER a non-stall outcome (`!outcome.is_stall()`), so a genuinely-stalled writer stops advancing the
  counter and the 3s comms-stall detector can fire. Composes cleanly: a recovered lost-wake IS a completed write (bytes
  delivered) so it SHOULD bump; only a real stall withholds. Other two bumpers (status_responder:4358, comms_consumer:1548)
  untouched. Both fix configs build clean -D warnings on Xtensa (verified, then reverted the wiring out of the tree).
- READOUT (fix c, folded into the combined patch — bughunter: LOAD-BEARING for fix-confirmation, not nice-to-have): a new
  `pub static USB_TX_LOST_WAKE_RECOVERED: AtomicU32`, bumped on the recovered-lost-wake path, surfaced on `$I` build-info as
  a `[MSG:USBTX rec=N]` line (emitted only when N>0, via send_build_info). WHY: the bug is non-deterministic (one re-run
  streamed clean = non-event), so "stream ran to completion" alone does NOT prove the fix worked — a CLIMBING rec= during a
  COMPLETING stream proves lost wakes occurred AND were recovered. The full combined patch (a+b+readout) build-verified
  clean both Xtensa configs, then reverted so the tree stays at capture-only #1b for capture #3. (Boot/[MSG:] line for the
  count is LOWER value — the counter zeroes on reset and a working fix means NO reset; the live `$I` poll is the real
  signal. Offered RTC_FAST persistence of the count if a prior-run boot readout is wanted, but not built.)

**REFRAME 2026-06-25 (esp-hal 1.1.1, collaborative w/ embedded-bug-hunter — analysis only, NO flash yet).** The §10
`128-Pikachu.tap` repro on the CURRENT 1.1.1 image shows a precise 2.00 s "drumbeat": at the wedge, ONE `ok` drains per
≈2.00 s for ~8 cycles while the core-0 status reporter is fully DEAD (163 `?` → 0 replies), then a burst-drain + reset.
Mapping that to code OVERTURNS the long-standing RMT-ch0-hang root cause for the 1.1.1 wedge:
- **The 2.00 s period is NOT the bounded RMT wait().** `motion.rs` `RMT_WAIT_TIMEOUT_CYCLES = 480_000_000` IS exactly
  2.000 s at 240 MHz `CpuClock::max()` — BUT that path (`motion.rs` emit_burst timeout branch) does `drop(txn)` →
  `esp_hal::system::software_reset()` IMMEDIATELY on the FIRST timeout (no retry-N, no force-complete-one-block). Verified
  vs installed esp-hal 1.1.1 + esp-metadata-generated-0.4.0: esp32s3 `rmt.has_tx_immediate_stop = true`, so `TxGuard::drop`
  (rmt.rs:1414) takes the immediate-stop branch and the `while !done {}` spin (rmt.rs:1423) is cfg-compiled OUT → the reset
  fires essentially instantly. So ONE RMT-wait timeout = ONE reset. That CANNOT produce "limp 8× then reset". Disqualified.
- **The real 2.00 s suspect = `USB_TX_TIMEOUT = Duration::from_secs(2)` (comms.rs:1342).** `usb_tx` wraps every write
  `with_timeout(2s, write)`. esp-hal's async USB-Serial-JTAG write/flush awaits TX-FIFO-drained (`serial_in_empty`), which
  only fires when the HOST reads. If the host stops draining, each write parks exactly 2.000 s, the timeout drops that one
  response, and the loop pulls the next `RESPONSE` item — a perfect "1 item / 2.00 s, repeat" drumbeat. usb_tx is on the
  core-0 thread-mode executor (it yields), so `with_timeout` (embassy time) advances fine here — unlike the core-1 busy-spin.
- **Status-dead-while-acks-limp = RESPONSE-channel head-of-line blocking, NOT a shared lock.** Single MPSC `RESPONSE`
  (comms.rs:118, depth `RESPONSE_QUEUE_DEPTH=8`) drained by the ONE slow `usb_tx`. When it's stuck 2s/write the channel
  saturates; `status_responder` blocks on `enqueue(s).await` (comms.rs:4436) mid-iteration so it never answers the next `?`
  (163 `?` → 0). The executor RELEASES the PLANNER lock BEFORE any RMT transmit (`take_block`, motion.rs:564-572), so a
  core-1 RMT hang does NOT hold a lock that starves status — the lock theory is OUT; the slow shared writer is the mechanism.
- **Firmware-only discriminator (host-quit vs peripheral-wedge), verified in esp32s3-0.35.2 PAC (the version the firmware
  ACTUALLY resolves — Cargo.lock + `cargo tree -i esp32s3`; do NOT cite 0.34.0, which is also on disk but unused).** Read at
  the usb_tx timeout instant: `USB_DEVICE.ep1_conf().read().serial_in_ep_data_free()` (HW "host accepted the IN packet, FIFO
  has room"), `USB_DEVICE.int_raw().read().serial_in_empty()` (the TX-done event), AND `int_ena().read().serial_in_empty()`
  (still ARMED ⇒ the ISR never ran ⇒ the sharpest lost-wake fingerprint). `data_free == false` → host stopped draining
  (host-side/skirnir bug, peripheral healthy). `data_free == true` but the future never woke at 2s → peripheral/waker wedge.
  `/tmp/pika.raw` asymmetry (host→device `?` writes kept SUCCEEDING through the drumbeat while device→host stalled) already
  leans device-TX-drain side, RX path alive.
- **1.0 vs 1.1 reconciliation:** the 2026-06-24 `axis0:wait_begin` RTC breadcrumb (RMT ch0 hang, Mode A) was REAL but on
  the esp-hal **1.0** image; the RMT driver changed across 1.0→1.1. The 1.1.1 drumbeat is most likely a DIFFERENT/downstream
  mode (USB-TX-drain stall), not the same RMT hang. NOT a misattribution — a different image's wedge.
- **TRANSPORT BLOCKER (load-bearing for ANY capture experiment): there is NO usable out-of-band RTT on this board.** On
  the S3 the defmt sink is NOT a separate RTT channel — esp-println 0.17's `defmt-espflash` backend rides the SAME
  USB-Serial-JTAG peripheral (`peripherals.USB_DEVICE`) as the grbl CDC stream (documented in crates/firmware/Cargo.toml:73-81
  + main.rs:368-371). So you CANNOT stream GCode over skirnir's CDC AND watch defmt/RTT at once over the one built-in USB
  port. The only true separate-RTT path is an EXTERNAL JTAG probe on the dedicated JTAG pins (GPIO39-42), but GPIO39 is
  already allocated as the A-LIMIT placeholder (main.rs:534) and no probe is wired. ⇒ live-RTT capture plans (incl. doc
  §11.5's original sketch) are not viable as-is; the correct tool is the EXISTING RTC_FAST post-mortem breadcrumb over the
  normal grbl CDC in the DEFAULT (no-defmt) build — the same `crash.rs` infrastructure already built for this.
- **BUILD #1 IMPLEMENTED 2026-06-25 (TDD-first, both Xtensa configs build clean -D warnings, 15 new host tests green, my
  files clippy-clean — the 5 -D-warnings clippy errors are PRE-EXISTING in cnc-kinematics, untouched by me, and the project
  has NO clippy CI gate). Team-lead has the user's direct flash authorization and drives the hardware; I do NOT flash.**
  - NEW pure host-tested module `crates/firmware-core/src/diag.rs` (the only firmware logic that's `cargo test`-able —
    `firmware` is a Xtensa-only bin with no host test target): `UsbTxStallCounter` (K-counter, resets on a completed write,
    fires at `USB_TX_STALL_ESCAPE_K = 3`); `UsbTxStall` struct + `pack_usb_tx_stall`/`decode_usb_tx_stall` (one tagged
    RTC_FAST word, tag `0x5554_0000` "UT"); `UsbTxStall::verdict()` → `HostNotReading`/`LostTxWake`(H-A)/`Core1Wedged`(H-B)/
    `Ambiguous`. TDD CAUGHT A REAL BUG: a byte-wide timeout_count at shift 9 reached bit 16 and corrupted the high-half tag
    → shrank to a 7-bit count (sat 127); fields now all in low-16. Signals: data_free, serial_in_empty (int_raw),
    int_ena_armed (int_ena), motion_advancing, executor_running, response_depth. VERDICT now keys LostTxWake on
    `int_ena_armed || serial_in_empty` (bughunter's sharper H-A fingerprint: int_ena STILL ARMED ⇒ the ISR never ran =
    strongest lost-wake; empty set + int_ena cleared ⇒ ISR fired but waker lost = a different H-A flavor). All three
    USB-reg reads (ep1_conf.serial_in_ep_data_free, int_raw.serial_in_empty, int_ena.serial_in_empty) verified present in
    the ACTUALLY-RESOLVED esp32s3-0.35.2 PAC, `.bit_is_set()` accessor.
  - `crash.rs`: +2 RTC_FAST words `USB_TX_STALL` + `RMT_WAIT_COUNT` (RING_BASE/LEN shifted +2); `record_usb_tx_stall(word)`
    (stores the pre-packed word — bit layout owned by the tested module), `bump_rmt_wait_timeout()` (saturating); decoded
    into `Breadcrumb.usb_tx_stall`/`.rmt_wait_count`, cleared on consume.
  - `comms.rs` `usb_tx`: holds the counter across loop turns; samples MOTION_LIVENESS before the write; `with_timeout(...).
    is_err()` → `stall.record(timed_out)`; at K calls `capture_usb_tx_stall_and_reset()` which reads `USB_DEVICE::regs()`
    (`ep1_conf().serial_in_ep_data_free()`, `int_raw()`/`int_ena().serial_in_empty()` via `.bit_is_set()`, like
    capture_rmt_hang reads RMT::regs()), samples MOTION_LIVENESS delta + EXECUTOR_RUNNING + `RESPONSE.len()`, packs,
    records, `software_reset()`. Boot dump: new `[MSG:CRASH usbtx: <verdict> free=.. empty=.. mov=.. exec=.. rdepth=.. n=..
    rmt_to=..]` line (CRASH_REPORT Vec 4→5). `motion.rs:382`: `crate::crash::bump_rmt_wait_timeout()` before the existing
    RMT-wait `software_reset` (RMT path otherwise UNCHANGED — still resets on first timeout).
  - SCOPE NOTE: deliberately did NOT relocate the per-loop COMMS_PROGRESS bump (§11.6 watchdog-mask fix) — the K=3 escape
    pre-empts the ~16s limp so the mask is moot for this capture; relocating the bump is a separate behavior change. K=3 not
    K=4 (team-lead's call): ~6s is inside the 8s RWDT so the K-escape is GUARANTEED the resetter (carries the discriminator),
    not a bare RWDT reset. BUILD GOTCHA: `RUSTFLAGS="-D warnings"` on the CLI CLOBBERS the crate's config.toml
    `-Tlinkall.x` → flood of `undefined reference` LINK errors (not compile errors); must pass
    `RUSTFLAGS="-C link-arg=-Tlinkall.x -D warnings"` to keep both, or just `cargo build` (config rustflags apply).
- **PROPOSED experiment (breadcrumb-over-CDC, default build; converged w/ bughunter; pending USER go-ahead, no flash by me —
  team-lead has direct flash authorization and drives the hardware):** add a BOUNDED ESCAPE in `usb_tx` — after K consecutive
  `with_timeout` EXPIRIES (proposed K=4 ≈8s, inside the 8s RWDT), record a discriminator breadcrumb word + `software_reset()`
  (CoreSw, RTC-preserving). This (a) converts the 16s limp into a clean ~8s self-recover whose breadcrumb is readable over
  CDC next boot, AND (b) fixes doc §11.6's watchdog-mask defect (usb_tx's per-loop COMMS_PROGRESS bump at 2s cadence evading
  the 3s comms-stall detector — §11.3). Discriminator packed at the K-th timeout (firmware-only, no RTT): `ep1_conf.
  serial_in_ep_data_free` + `int_raw.serial_in_empty` (H-A lost-TX-done-wake test), MOTION_LIVENESS-advancing +
  EXECUTOR_RUNNING (H-A vs H-B core-1-health), RESPONSE depth. Decode next boot: serial_in_empty SET + wake-never-came +
  core-1 ALIVE → H-A (esp-hal USB-event loss); core-1 FROZEN/EXECUTOR_RUNNING → H-B (core-0 starvation / cross-core lock).
  Deliberately NOT suppressing the RMT-wait reset (team-lead risk #1: that would hard-lock the board on a real RMT hang). The
  K-counter / discriminator pack-decode / timeout-outcome classification are pure-fn host-testable (TDD-first), mirroring the
  existing `capture_rmt_hang` pattern; only the register reads + software_reset are thin wiring.

---

**MODE C (HARD WEDGE) = A PANIC — CUSTOM PANIC HANDLER + STACK BUMP ADDED 2026-06-25 (compiled both configs, -D warnings
+ clippy clean, 522 host tests green).** Deep-research (adversarially verified) on the hard, silent, EN-only wedge: (1)
esp-backtrace 0.19's DEFAULT panic handler is `arch::interrupt_free(|| loop {})` — no reset, no breadcrumb, hangs the
core forever → ANY panic = exactly Mode C. (2) esp-rtos 0.3 PANICS on a core-1 (APP_CPU) main STACK OVERFLOW (prime
suspect, ties to [[xtensa-stack-top-abi-headroom]]). (3) `#[ram(rtc_fast, persistent)]` IS reliable across watchdog/SW
resets (the "unreliable" claim refuted). NON-RECOVERY EXPLAINED: a core-1 panic into `interrupt_free(|| loop {})`
disables interrupts on CORE 1 ONLY → core 0 keeps FEEDING the RWDT → dog never fires → hard wedge.
- GOAL A (panic handler): dropped esp-backtrace's `panic-handler` feature (kept `esp32s3`+`println`; esp-backtrace 0.19
  has NO exception-handler feature — esp-hal owns the exception vector and routes faults into OUR handler, so fault
  capture not lost). Custom `#[panic_handler]` in main.rs: reads `Cpu::current() as u8` + `info.location()` (NO
  formatting), calls `crash::record_panic(core, file.as_ptr() as u32, file.len() as u32, line, has_loc)` then
  `esp_hal::system::software_reset()`. MINIMAL-STACK / overflow-safe: `record_panic` is `#[inline(never)]`, does ~6 raw
  Relaxed stores to RTC_FAST, no struct/lock/format/println. VERIFIED (installed source): PanicInfo/Location/str API
  const-stable; Cpu::current() is a register read (0=ProCpu,1=AppCpu); software_reset() is full-chip, both-core,
  panic-safe, preserves RTC_FAST (CoreSw).
- BUILD-ID GUARD: the file-string is stored as a `.rodata` POINTER+len (stable across SW reset of the SAME image) and
  re-read at BOOT (full stack). `build.rs` emits `GALDR_BUILD_ID` (wall-clock nanos); `crash::BUILD_ID` (const decimal
  parser) is stamped into the breadcrumb; boot dereferences the pointer ONLY when the stored build id matches → a stale
  pointer from a DIFFERENT flashed image yields `panic ?:line` not garbage. Decoder caps file len ≤120 (fits
  RESPONSE_CAPACITY).
- BREADCRUMB layout (crash.rs): new words PANIC_FLAGS(tag 0x5041 + core + has-location bit), PANIC_FILE_PTR,
  PANIC_FILE_LEN, PANIC_LINE, PANIC_BUILD_ID; RING_BASE→21, LEN→33 words. `record_panic` also stamps MAGIC (so a panic
  before init_magic still reports). New `PanicReport{core,line,file:Option<&'static str>}` decoded in take_breadcrumb
  (safe pointer deref guarded by build-id + null/len check), cleared on consume.
- GOAL B (boot dump): `format_panic_report` → `[MSG:CRASH panic <file>:<line> core=<N>]` (or `panic ?:<line>` if no
  location / stale image). Emitted FIRST (most decisive), independent class. CRASH_REPORT stash → Vec<Response,4>.
  Replayed on first `$I`/`?` like the others.
- GOAL C (stack bump): APP_CORE_STACK_SIZE 16→32 KiB. KEY VERIFIED FACT: on Xtensa there is NO separate interrupt
  stack — the SWI2 InterruptExecutor poll + the WHOLE `motion_executor` RMT call chain run on the `start_second_core`
  `Stack<N>` arena (charged on top of the scheduler thread). So bumping THIS arena IS the correct lever for
  streaming-load overflow headroom (usable depth = SIZE − guard_offset). esp-rtos guard-checks it + panics on overflow
  (now visible). `_abi_headroom` canary/padding kept (independent of depth — the over-top spill bug was size-independent).
- NON-RECOVERY SANITY (cheap, confirmed): RWDT IS armed (esp-hal init disables it, our explicit enable() re-arms, esp-rtos
  start never touches LPWR); the core-1-panic-leaves-core-0-feeding mechanism is THE non-recovery cause; the panic
  handler fixes it by resetting the whole chip from the panicking core regardless.
- CAPTURE: `just flash`, stream Pikachu, on a wedge WAIT ~12 s (panic handler resets near-instantly; a non-panic hang
  takes the 8 s RWDT) — do NOT EN/power-cycle. Read `[MSG:CRASH panic <file>:<line> core=<N>]`. `core=1` + a
  motion.rs/RMT file ⇒ core-1 stack overflow / motion-path panic (if so, the 32 KiB bump is the confirmed fix); `core=0`
  ⇒ comms-path panic.

**NEW SIGNATURE: CORE-0 COMMS WEDGE WITH MOTION IDLE — COMMS-STAGE BREADCRUMB ADDED 2026-06-24 (compiled both configs,
-D warnings + clippy clean, 522 host tests green).** After the CCOUNT timeout fix (commit a829c8a — the coordinator
switched my `Instant`-based RMT timeout to `esp_hal::xtensa_lx::timer::get_cycle_count` because `Instant::now()` was
FROZEN by the busy-spin; LESSON: embassy-time `Instant` may not advance inside a tight core-1 busy-loop, use the cycle
counter for in-spin deadlines), the operator re-ran and got a NEW breadcrumb: `[MSG:CRASH core0-comms-wedge
stage=idle_waiting comms-froze-first beats comms=34037 motion=50605 (RWDT-reset)]`. KEY: `stage=idle_waiting` (motion
executor IDLE/parked, queue drained) NOT `axis0:wait_begin` → NOT the RMT hang (which did not recur). No `rmt0:` line
(the RMT timeout didn't fire — consistent with motion idle). So core 0's comms path parked on some `.await` that never
returned while motion was idle; the task-watchdog caught it (comms froze + RX live → withheld feed → RWDT). Rare again
(~2 runs to repro). We had instrumented MOTION stages but not COMMS, so `idle_waiting` only told us motion is fine.
- FIX = a per-CORE-0-TASK comms-stage breadcrumb (same approach that nailed the RMT hang). `firmware/src/crash.rs`: new
  `CommsTask` (5 slots: UsbRx/LineAssembler/Consumer/UsbTx/Status) + `CommsStage` enum + `record_comms_stage(task,stage)`
  (one tagged relaxed store per slot). PER-TASK slots so concurrent core-0 tasks never clobber each other's marker — the
  stuck task is UNAMBIGUOUS. Each task writes its slot IMMEDIATELY before every `.await` it can park on. Idle-class
  stages (rx-read, line-wait-byte, consumer-wait-line, tx-wait-response, status-wait-request) = parked waiting for work;
  a NON-idle stage persisting on a wedge = the culprit. Breadcrumb LEN grew to 28 words (RING_BASE 11→16); slots cleared
  on consume.
- INSTRUMENTED await sites (`firmware/src/comms.rs`): usb_rx `rx.read` (rx-read); line_assembler `RX_PIPE.read` select
  (line-wait-byte) + `LINE_QUEUE.send` (line-send-queue); consumer main select (consumer-wait-line), `store_settings`
  flash (consumer-flash-settings — the Defect-#2 `multicore_auto_park` smoking gun), `store_coordinates`
  (consumer-flash-coords), `ack`/`error_bare` `RESPONSE.send` (consumer-enqueue), plan_command QueueFull
  (consumer-plan-backpressure, timer-backed so self-wakes), PROBE_RESULT (consumer-probe-result), HOME_RESULT
  (consumer-home-result), quiesce MOTION_PARKED + M0/M1/M6 pause (consumer-sync-wait); usb_tx `RESPONSE.receive`
  (tx-wait-response) + `write_all`/`flush` (tx-write — PRIME "output never completes" suspect: a stuck write backs up
  RESPONSE and blocks every producer); status_responder STATUS_REQUEST.wait (status-wait-request) + report build
  (status-build-report).
- BOOT DUMP: summary line gains `comms-stage=<first-non-idle-slot, else consumer slot>`; a THIRD `[MSG:CRASH comms:
  rx=.. line=.. con=.. tx=.. sta=..]` line shows EVERY task's parked await. CRASH_REPORT stash now `Vec<Response,3>`.
- HOW TO READ NEXT BREADCRUMB: the `comms:` line shows all 5 task park-points. The slot with a NON-idle stage is the
  stuck task. LEADING SUSPECT given motion idle + RX live: `tx=tx-write` (usb_tx stuck in USB write/flush → RESPONSE
  fills → all producers block → comms froze). Other smoking guns: `con=consumer-flash-settings/coords` (the Defect-#2
  flash/cache-disable hazard — though Pikachu is pure motion, no settings writes expected after the initial `T1`);
  `con=consumer-probe-result`/`-home-result` (a result signal from core 1 that never came); `con=consumer-sync-wait`
  (a quiesce/pause never released). All-idle slots ⇒ the wedge is in an await we did NOT instrument (widen next).
- HAPPY-PATH NO-REGRESSION: each marker is ONE relaxed store before an await, all on COLD paths (per-line/per-`?`/
  per-response), zero changes to motion.rs/core-1, no new awaits/locks. Core-1 step timing untouched.
- SECONDARY RESOLVED (no code change): VERIFIED `software_reset()` (CoreSw/RTC_CNTL_SW_SYS_RST) DOES preserve RTC_FAST
  `persistent` on the S3 — esp-hal `persistent` macro doc names `software_reset()` FIRST in its survivable list; TRM:
  all resets except Chip Reset preserve internal memory; esp-idf RTC_NOINIT survives `esp_restart()`. So the RMT-timeout
  `software_reset()` path reliably preserves the breadcrumb. The MAGIC validity word is the checksum the doc recommends.

**SUPERSEDED: RMT HARDWARE INSTRUMENTATION 2026-06-24 (the RMT hang stopped recurring after the CCOUNT fix).** Operator
flashed the 47→46 burst-cap (commit c676a3d): SAME breadcrumb, now near-INSTANT
(`comms=274 motion=942`, ~10 s in, was ~30 min). So the full-48-block-boundary theory was WRONG/incomplete — same core-1
RMT ch0 `wait()` hang. KEEP the burst-cap (harmless, one real hazard removed — do NOT revert). GOOD news: a FAST
(~seconds) repro now exists → instrument the HARDWARE instead of guessing from `rmt.rs`.
- The bifurcating fact: WHEN `wait()` spins on ch0, is `TX_END` actually SET in HW? SET ⇒ TX finished but our wait missed
  it (driver/usage bug). NOT set ⇒ TX genuinely never completed (memory/encoding/start).
- IMPLEMENTED a BOUNDED RMT wait poll-loop + register capture. `firmware/src/motion.rs` `RmtStepSink::emit_burst`: the
  per-axis blocking `txn.wait()` is replaced by `loop { if txn.poll() {break false} if Instant::now()>=deadline
  {break true} }` (RMT_WAIT_TIMEOUT=2 s, >> the ~1.5 s worst-case legit burst, < 8 s RWDT). Happy path UNCHANGED:
  `poll()` is the same volatile status read `wait()` spun on; on done → `wait()` returns immediately (esp-hal guarantees
  it) → recover channel. NO timing perturbation (the burst plays in HW regardless of poll rate; only an extra cheap
  `Instant::now()` per poll). On TIMEOUT (the hang): `capture_rmt_hang(axis,nsym,burst_seq)` reads ch0 registers, then
  `drop(txn)` (S3 `rmt_has_tx_immediate_stop=true` → immediate stop_tx, NO drop-hang), then `esp_hal::system::
  software_reset()` (deterministic — the post-abort state is ambiguous so the watchdog might not fire; `CoreSw` preserves
  RTC_FAST + is a fault-reset → breadcrumb is read next boot).
- REGISTERS captured (VERIFIED against installed esp-hal 1.1.1 + esp32s3-0.35.2 PAC; `esp_hal::peripherals::RMT::regs()`,
  no unsafe at call site, side-effect-free reads, safe from core-1 InterruptExecutor): `int_raw.ch_tx_end(axis as u8)`
  (TX_END bool — THE decider), `.ch_tx_thr_event` (thr), `.ch_tx_err` (err), whole `int_raw`/`int_st` words,
  `ch_tx_status(axis as usize)` (FSM `state` = bits 22:24 → transmitting-vs-idle) and `ch_tx_conf0(axis as usize)`
  words. NOTE the index-type split: int fields take `u8`, ch_tx_*(usize) take `usize` — matched exactly as esp-hal does.
- BREADCRUMB layout extended (`firmware/src/crash.rs`): new words RMT_FLAGS(5, tagged 0x524D + end/thr/err/axis/nsym),
  RMT_INT_RAW(6), RMT_INT_ST(7), RMT_TX_STATUS(8), RMT_TX_CONF0(9), RMT_BURST_SEQ(10); RING_BASE 5→11; LEN=23 words
  (fits RTC_FAST easily). New `RmtHang` struct + `record_rmt_hang`/decode in `take_breadcrumb` (clears RMT_FLAGS on
  consume). `RmtStepSink` gained a `burst_seq` counter bumped per burst.
- BOOT DUMP: `format_rmt_hang_report` emits a SECOND `[MSG:CRASH rmt<axis>: end=<0/1> thr=<0/1> err=<0/1> fsm=<n>
  nsym=<n> burst#=<n> ir=0x.. is=0x.. st=0x.. cf=0x..]` line (split from the summary so neither exceeds
  RESPONSE_CAPACITY=160). The CRASH_REPORT stash is now `heapless::Vec<Response,2>` replayed on first `$I`/`?`.
- HOW TO READ THE NEXT BREADCRUMB: `rmt0: end=1` ⇒ TX DID finish, our wait/poll missed completion → fix HOW we wait
  (driver/usage; e.g. a poll/clear race, or `poll()`/`wait()` not seeing the latched bit). `end=0` + `fsm`≠0 ⇒ channel
  STILL transmitting (never completed) → memory/encoding/start (e.g. a symbol the HW never terminates on, a clock/start
  glitch, a mem-owner issue). `end=0` + `fsm`=0 (idle) but no End ⇒ HW went idle without raising End (a missed-event /
  int-status anomaly). `thr=1` = a half-block refill was pending (shouldn't matter for ≤47-sym). `cf`/`st` raw words let
  us re-derive wrap_en/mem_size/etc off-board. `burst#`/`nsym` characterize the hung transmission.
- CAVEAT to bench-verify: `software_reset()` (`CoreSw`/`RTC_CNTL_SW_SYS_RST`) is documented to preserve the RTC domain
  (so RTC_FAST/breadcrumb survives) — HIGH confidence but ROM-binding, not source-readable. If the `[MSG:CRASH rmt0:]`
  line does NOT appear after a timeout-reset, the SW reset wiped RTC_FAST and we fall back to the RWDT path.
- This is the FRONT HALF of the eventual timeout-backstop; the full backstop = feed-hold + ALARM + disable steppers +
  REQUIRE RE-HOME (DOC-06, deferred). The current reset is purely diagnostic.

**SUPERSEDED ROOT-CAUSE THEORY (kept for context — the full-block fix did NOT resolve it):** The RTC crash breadcrumb
(built earlier this session) captured a Pikachu wedge: `[MSG:CRASH core0-comms-wedge stage=axis0:wait_begin comms-froze-first beats comms=41898
motion=60943 (RWDT-reset)]`. Decisive: `stage=axis0:wait_begin` with NO `wait_done` ⇒ the core-1 motion executor hung
in esp-hal's blocking RMT TX-completion `wait()` for CHANNEL 0 (X) — TX-END never fired, the busy-poll `wait()` spun
forever. (`core0-comms-wedge` is COLLATERAL: executor hung → planner queue filled → comms_consumer parked → status froze;
the 3 s comms detector tripped ~1 s before the 4 s motion detector. Trust the stuck `wait_begin`.) Non-deterministic line
(835/1387/~1500), only on the big file (4474-line Pikachu, ~all short fast G1 → vastly more X step-bursts/sec), wedged
mid-detail during cutting. T1_Test (500 lines, arc-heavy) ran clean twice.

**MECHANISM (verified against INSTALLED esp-hal 1.1.1 `rmt.rs` + `rmt/writer.rs`, S3 `channel_ram_size`=48):** the
blocking one-shot `transmit` writes the whole buffer into the 48-slot block and `start_send` sets `mem_tx_wrap_en=1` +
threshold=24. RULED OUT: the refill/threshold path (writer is `Done` for a ≤block buffer → refill is a no-op) and any
cross-channel int-clear race (`clear_tx_interrupts`/`get_tx_status` are per-channel, int_raw W1C, TX_END latches). The
REAL hazard is the COMPLETELY-FULL block: if the buffer is exactly 48 symbols and the writer's last-written code is NOT a
length-zero end marker (any off-by-one, or a 49th-marker truncated by `count=data.len().min(48)`), the writer stays
`WriterState::Active`, there's NO free slot for esp-hal to inject a terminating marker, the HW read pointer WRAPS slot-47
→ 0 and re-transmits, and `wait()` polls `Event::End` forever (writer.rs:146). Our encoder normally puts the marker in
slot 47 (→ `Done`, should be safe), so this is the full-block BOUNDARY being fragile on the busiest channel; the rare/
timing/burst-density/ch0 signature fits the full-block edge. No post-1.1.1 esp-hal fix exists for this (issues #2115/
#3477 are the missing/embedded-marker cases, already handled).

**THE FIX (landed, both configs compiled, 522 host tests green):** `firmware-core/src/hal_traits.rs` —
`MAX_SYMBOLS_PER_BURST: 47 → 46`. Now a max burst = 46 events + 1 marker = 47 symbols, ALWAYS one slot short of the
48-slot block. With `data.len() < 48` guaranteed, the `Active`/wrap/hang path (writer.rs:146) is STRUCTURALLY
UNREACHABLE: esp-hal either reaches `Done` (our marker) or injects a marker into the free slot and returns a clean
`Error` from `transmit` BEFORE TX starts — never a silent hang. Cost: a 47-event move now spans 2 bursts (one extra
sub-µs transmit on the dedicated core). Updated the `full_burst_plus_end_marker_*` test to assert `+1 < 48` (free slot),
and the two burst-sizing tests (symbolic, auto-adapt). Regression comment on the const documents "do NOT raise to 47".
DO-NOT-RAISE is load-bearing.

**CONFIDENCE: high that this is the right fix, honest caveat:** the writer-state analysis says our NORMAL 48-symbol
encoding (marker in slot 47) should reach `Done` and be safe — so I could NOT prove our encoder hits the exact `Active`
trap. But the empirical breadcrumb (ch0, rare, burst-density-correlated) points squarely at the full-block boundary, and
the fix eliminates the ENTIRE full-block hazard class (writer-state AND any HW wrap-at-full-block quirk) regardless of
the exact sub-mechanism. Low-risk, provably removes the boundary. CONFIRM on the bench: re-flash, stream Pikachu to
completion (it wedged ~1-in-a-few before); if it ever recurs, the breadcrumb still captures it and the timeout backstop
(below) becomes the next step.

**PROPOSED (NOT yet landed) defense-in-depth — timeout-bounded RMT wait → safe ALARM:** esp-hal exposes
`TxTransaction::poll(&mut self)->bool` (non-blocking; true=done). Feasible backstop: loop `poll()` against an
`embassy_time::Instant` deadline (~3 s, >> the ~1.5 s worst-case legit burst); on done → `wait()` (returns at once); on
TIMEOUT → DROP the txn (S3 has `rmt_has_tx_immediate_stop=true`, so `TxGuard::drop` does `stop_tx`+`update` and SKIPS the
`#[cfg(not(immediate_stop))]` busy-wait → clean immediate stop, no drop-hang) then raise a SAFE ALARM. CNC-SAFETY NUANCE:
aborting a burst mid-cut LOSES step sync → must feed-hold + ALARM + disable steppers + REQUIRE re-home, NEVER silently
retry. Needs the alarm state machine wired (DOC-06). Defense-in-depth regardless of root cause; left for a decision.

**RULED OUT as the ch0 cause:** the known axis-3 (A) encode gap (`emit_burst` encodes axes 0,1,2 but transmits `0..AXES`
=4, so ch3 transmits stale `scratch[3]`=all-end-markers → instant TX_END, harmless; A never steps — a separate DOC-10
latent bug, NOT corrupting ch0). We're hung on ch0 which IS encoded.

---

**Investigation 2026-06-24.** Streaming `T1_Test.tap` (repo root, 500 lines: 252 G1, 123 G2, 21 G3 arcs, 95 G0, M3/M5;
PURE MOTION — no `$n=val`, no G54-G59 select, no G10/G92 → triggers NO flash writes) the firmware locks up at a
DIFFERENT line each run, DRO/status stop, host sends ~1 KB more then stalls, hard reboot required (no WDT auto-reset).
Distinct from the now-exonerated planner-geometry theory.

**RULED OUT by source review (`crates/firmware/src/`, all read this session):**
- Core-1 stack / ABI headroom: the `AppCoreStackArena` fix is present AND hardened further than memory recorded —
  there is now a sized headroom (`APP_CORE_ABI_HEADROOM = 2× the modeled CALL12 worst-case spill = 128 B`) PLUS an
  `arm_canary`/`check_canary` regression canary checked after `start_second_core` (main.rs ~95-197, 426-483). Not the bug.
- BlockQueue ring buffer: `heapless::Deque<Block, BLOCK_QUEUE_LEN=32>` (planner.rs:505/58) — push_back/pop_front, no
  manual index math, no off-by-one. Sound.
- Arc state machine: `enqueue_arc_chunk` (planner.rs:1363) terminates — `next_seg` monotonic, full-queue guard returns
  `ArcPending{enqueued:0}`, `drive_pending_arc` (comms.rs:1934) waits on SLOT_FREED/timer and PROGRESSES per executor
  pop. The "arc_in_progress overwrite" / "stale position" theories are NOT reachable: the consumer is STRICTLY SERIAL —
  `plan_command` calls `drive_pending_arc().await` INLINE (comms.rs:1843) and does not return to handle the next line
  until the arc fully drains, so no second command runs while `arc_in_progress` is Some. (An Explore agent flagged these
  as bugs; they are guarded by the serial consumer + host pipeline test passing.)
- AXES==4 consistency: `[None,None,None,None]` in emit_burst, all `[_; AXES]` arrays, `0..AXES`/`from_fn` everywhere —
  no lingering hardcoded-3. RMT sink configures 4 real channels (ch0-3 = X/Y/Z/A). Internally consistent.
- RMT silent-symbol encoding: `silent_symbol_halves` (motion.rs:552) is provably non-zero (a,b≥1) → no accidental
  end-marker; `encode_channel` appends the real `end_marker()` and transmits `..len` (includes it). Sound.
- Back-pressure loops (`plan_command`/`drive_pending_arc`/`handle_go_to_predefined`): all `.await` with timer
  backstops, no busy-spin, no interrupt-disabled wedge.

**REAL DEFECT #1 (certain): NO WATCHDOG configured anywhere.** `grep -rni wdt|watchdog crates/firmware/src` → nothing;
main.rs init never sets up RWDT or TIMG WDT. THIS is why every wedge needs a hard power-cycle (no auto-reset) and why no
reset-reason is captured. Fix: enable RWDT in main init + feed it from a low-priority core-0 task (and ideally a core-1
liveness beat). Turns a hard-reboot into an auto-recover AND lets the bench read the reset reason. esp-hal 1.1.1 API:
`esp_hal::rtc_cntl::Rtc` + `rtc.rwdt` (`set_timeout`/`enable`/`feed`), or TIMG `Wdt`.

**REAL DEFECT #2 (latent, NOT the T1_Test trigger but a genuine landmine): `multicore_auto_park` flash-write
cache-disabled window vs. cached-flash motion/ISR code.** esp-storage 0.9.0 `multicore_auto_park()` (REQUIRED for writes
to land at all — see [[esp-storage-multicore-park]]) RUNSTALL-freezes core 1 for the flash erase/write, during which the
instruction/data CACHE IS DISABLED on both cores. NOTHING in this firmware is `#[ram]`/IRAM-resident (grep: zero `#[ram]`
sites) — the entire core-1 motion executor + all ISRs run from CACHED FLASH. If anything must execute from flash during
the window (a mid-fetch stall is benign, but a non-IRAM ISR firing/returning during it faults → "Cache disabled but
cached memory region accessed"), both cores wedge with no clean reset. Documented S3 analog: espressif/esp-idf #12271
(same chip, RMT+flash). Mitigated TODAY by `motion_idle()` deferral (comms.rs:4057 — only persist when EXECUTOR_RUNNING
clear AND queue empty), but there is a RESIDUAL race: `motion_idle()` returns true, then `BLOCK_AVAILABLE` can fire and
core 1 start an RMT transmit just as core 0 disables cache. Real, but T1_Test never writes flash so it is not THIS
lockup. Hardening: mark the motion hot path + RMT/GPIO ISRs IRAM-resident, OR gate persistence behind a stronger
quiesce, OR only persist at true Idle with a re-check.

**RMT wait() fact (esp-hal 1.1.1, verified from rmt.rs @ tag):** blocking `SingleShotTxTransaction::wait()` is an
UNBOUNDED busy-poll on TX status (no timeout/iteration bound); spins forever if TX_END never fires. Trigger = missing
end-marker (#2115; 1.1.1 is *supposed* to reject via Error per PR #2463 — verify that error path isn't swallowed). BUT
the spin holds NO lock / NO critical_section / does NOT disable interrupts → a hung core-1 wait() does NOT by itself
freeze core 0. Core 0 only stalls on what it AWAITS from core 1 (signals, the briefly-held PLANNER mutex). So a pure
RMT spin does NOT explain "core 0 status reporter also dead" — that points to a CPU FAULT into esp-backtrace's panic
handler (which runs from cached flash and can hang), not a clean spin. GPIO18/ch3 (A-axis) is a valid non-strapping,
non-USB pin; 4×memsize=1 channels do not alias on the S3.

**IMPLEMENTED 2026-06-24 (watchdog + instrumentation build, on the board pending flash — firmware NOT host-buildable, so
compile-verified by API research only, not by cargo):**
- RWDT watchdog: `main.rs` — `use esp_hal::rtc_cntl::{reset_reason, Rtc, RwdtStage}`, `use esp_hal::system::{Cpu, Stack}`,
  `use esp_hal::time::Duration`. `WATCHDOG_TIMEOUT = Duration::from_secs(8)` (16× the 500 ms feed → cannot false-trip on a
  flash-write/`?`/back-pressure stall; only a real core-0 wedge keeps the feed task from running 8 s). `static RTC:
  StaticCell<Rtc<'static>>`. In `main` step 1c: `RTC.init(Rtc::new(peripherals.LPWR))` → `set_timeout(RwdtStage::Stage0,
  WATCHDOG_TIMEOUT)` → `enable()` (stage-0 default action = system reset; esp-hal `init` disables RWDT by default so the
  explicit enable is required). VERIFIED via API research: `peripherals.LPWR` is the right field (NOT `RTC_CNTL`); esp-rtos
  0.3 `start` never touches LPWR (no double-take); `esp_hal::time::Duration` is the right type.
- Reset-reason logging: `main.rs` `log_reset_reason()` (called step 1b, BEFORE arming) reads `reset_reason(Cpu::ProCpu)` +
  `reset_reason(Cpu::AppCpu)` (AppCpu valid on dual-core S3), maps via `reset_reason_label` (SocResetReason→&str, catch-all
  `_`). defmt build → `defmt::info!`; default build → `esp_println::println!("[boot] reset reason: ...")` (gated so the two
  esp-println back-ends never interact). After the dog auto-resets a wedge, next boot logs `cpu0-rtc-WDT`/`core-rtc-WDT`; a
  panic-reset logs `cpu0-sw-reset`.
- Watchdog feed task: `comms::watchdog_feed(rtc: &'static mut Rtc)` (spawned core-0 step 7), `WATCHDOG_FEED_INTERVAL =
  500 ms`, `rtc.rwdt.feed()` UNCONDITIONAL first each loop (never gated on liveness → can't starve the dog), then under
  defmt samples MOTION_LIVENESS and logs STALLED vs advancing. `last_liveness` is defmt-cfg'd (else dead store).
- Core-1 liveness: `pub static MOTION_LIVENESS: AtomicU32` in comms.rs; bumped `Relaxed` `wrapping_add(1)` at TWO sites in
  motion.rs — top of the `run` drain loop (block/idle cadence) AND top of `RmtStepSink::emit_burst` (per-burst, so a long
  single block still reads as advancing, no false stall). Sampler tests inequality, not magnitude.
- mtrace chain: already complete in motion.rs `emit_burst` (per-axis `transmit Ok` / `wait begin` / `wait ok`/`wait err`)
  + loop chain — NO new trace was needed; a `wait begin` with no matching `wait ok` localizes the wedge to the exact axis.

**IMPLEMENTED 2026-06-24 (RTC_FAST POST-MORTEM crash breadcrumb — supersedes live-RTT capture; ACTUALLY COMPILED on the
esp toolchain, both default + `--features defmt`, `-D warnings`-clean, clippy-clean, 522 firmware-core host tests green):**
KEY INSIGHT (coordinator): live defmt/RTT is the WRONG tool — esp-println/defmt AND grbl comms BOTH ride the ONE
USB-Serial-JTAG (`peripherals.USB_DEVICE`), so you can't stream + monitor at once, and a both-cores-dead wedge emits
nothing live anyway. The capture is a POST-MORTEM over the normal grbl channel AFTER the watchdog resets the board, and
works in the DEFAULT (no-defmt) build.
- New module `crates/firmware/src/crash.rs` (+ `mod crash;` in main.rs). `static BREADCRUMB: [portable_atomic::AtomicU32;
  LEN]` under `#[esp_hal::ram(unstable(rtc_fast, persistent))]` (VERIFIED attribute: NOT `#[ram(rtc_fast)]`; args MUST be
  inside `unstable(...)`; `persistent` skips warm-reset re-init; there is no `uninitialized` kw). Type MUST be
  `portable_atomic::AtomicU32` — esp-hal 1.1.1 impls `Persistable` for THAT, not `core::sync::atomic` (confirmed in the
  installed `esp-hal-1.1.1/src/lib.rs` impl_persistable! macro). Added `portable-atomic = "1"` direct dep.
- Layout: `[MAGIC, LAST_STAGE, SEQ, HEAD, ring...]`; ring = RING_LEN=4 snapshots × SNAP_WORDS=3 `[seq, core0_beat,
  core1_beat]`. `record_stage(Stage, axis)` = ONE relaxed store on the hot path at each motion.rs mtrace site
  (LoopEntered/LockAcquired/BlockPopped/FeedPublished/EmitBurst/AxisTransmit/AxisWaitBegin/AxisWaitDone/IdleWaiting; the
  per-axis ones carry the RMT channel index). `push_snapshot` runs OFF the real-time path in the watchdog-feed task.
  `take_breadcrumb()` (boot) reads + CONSUMES (clears magic), then `init_magic()` re-stamps for this run.
- RETENTION: survives RWDT stage-0 "reset main system" (RTC domain preserved; esp-hal `persistent` doc names "watchdog
  timeouts") — NOT a power-cycle/brownout (clears RTC_FAST). OPERATOR MUST LET THE DOG BITE, not yank power. CAVEAT: if a
  ROM/bootloader path or a "reset RTC" WDT action ever clears the RTC domain the crumb is lost — BENCH-VERIFY once (write
  crumb → force RWDT → confirm crumb readable after reset). Default RWDT stage-0 is the RTC-preserving action.
- Core-0 heartbeat: `pub static CORE0_LIVENESS: AtomicU32` (comms.rs), bumped by `watchdog_feed` each tick AND
  `status_responder` per report. `crash::froze_first(&snapshots)` compares trailing frozen-run lengths of c0 vs c1 beats →
  `core1-froze-first`/`core0-froze-first`/`both-froze`/`no-stall`/`insufficient-data`.
- Boot dump: `comms::maybe_emit_crash_report(&breadcrumb, reset_was_watchdog)` (main step 8, after banner) emits a grbl
  `[MSG:CRASH stage=axis1:wait_begin core1-froze-first beats c0=N c1=M (RWDT-reset; not power-cycle)]` via the NORMAL TX
  (`ResponseWriter::message` + `enqueue`, NOT raw esp-println), gated on valid crumb AND watchdog/fault reset
  (`reset_was_watchdog_or_fault` — uses the REAL esp32s3 variants `CpuRtcWdt`/`CpuSw`/`CpuMwdt0/1`, NOT generic-doc
  `Cpu0*`; the Cargo-comment-warned variant-name trap bit me here, caught by compiling). ALSO stashed in `CRASH_REPORT`
  and replayed ONCE on the first `$I` (send_build_info) or `?` (status_responder) after connect, to survive a skirnir
  reconnect race across the USB re-enumeration.
- GOAL B (core-1-only stall → force reset): `watchdog_feed` WITHHOLDS the feed (lets the 8 s RWDT fire) when
  `EXECUTOR_RUNNING` is true AND MOTION_LIVENESS frozen for `CORE1_STALL_TICKS=8` (~4 s, >2.5× the ~1.5 s worst-case
  single burst). CONSERVATIVE: idle/parked/dwell all clear EXECUTOR_RUNNING so they never false-trip; only "provably
  executing but no burst for 4 s" forces the reset. Not behind a feature gate (judged safe given the tight gating) — if
  it ever false-trips on the bench, raise CORE1_STALL_TICKS or gate it.
- Cargo.toml (Goal C): corrected the `defmt` feature comment — the defmt sink is NOT a separate RTT channel; it's the
  SAME USB-Serial-JTAG as grbl on the S3 (no separate RTT without an external JTAG probe).
- KNOWN LATENT BUG SPOTTED (not in scope, flagged): `RmtStepSink::emit_burst` calls `encode_channel` for axes 0,1,2 only
  but the transmit/wait loops iterate `0..AXES`=0..4, so axis 3 (A) transmits STALE `scratch[3]` (all-end-marker from
  init → completes instantly, harmless for T1_Test which has no A motion, but A never steps correctly). Fix later.

**IMPLEMENTED 2026-06-24 (TASK-WATCHDOG fix — REAL-BOARD WEDGE exposed a hole; compiled both configs, -D warnings + clippy
clean, 522 host tests green):** HARDWARE EVIDENCE: user flashed the watchdog/crash fw, streamed a large file, wedged at
line ~1387; RWDT did NOT auto-reset, needed a physical EN-reset (wiped the RTC crumb → no crumb). Host side: skirnir
WRITES kept succeeding (usb_rx alive, draining FIFO) but got NO responses (DRO frozen) → the comms PROCESSING/RESPONSE
path died while the Embassy executor stayed ALIVE. ROOT CAUSE: the old `watchdog_feed` fed UNCONDITIONALLY (except the
core-1 check) AND self-bumped CORE0_LIVENESS, so a core-0 task stuck on a never-resolving `.await` (executor still
scheduling the feed task) kept the dog fed → never fired. The executor-death case and core-1 case were covered; the
CORE-0 STUCK-AWAIT case was NOT.
FIX = proper task-watchdog (feed only on REAL forward progress, gated by host activity):
- Renamed `CORE0_LIVENESS` → `COMMS_PROGRESS` and REMOVED the feed-task self-bump. Bumped ONLY on genuine host-facing
  work: `status_responder` serving a `?` (top of loop), `usb_tx` writing a response, `comms_consumer` finishing a line.
  Three independent bumpers — legitimate back-pressure (consumer blocked in QueueFull) still leaves `?` answered, so a
  stall needs ALL THREE frozen = the genuine wedge.
- Added `pub static RX_ACTIVITY: AtomicU32` bumped per non-empty `usb_rx` read (host-present signal; usb_rx keeps
  draining even when comms is wedged, which is WHY RX is the right "host driving" gate).
- `watchdog_feed` now withholds the feed in THREE classes: (1) core-0 executor death (task never runs); (2) core-1
  motion wedge (`EXECUTOR_RUNNING && MOTION_LIVENESS frozen CORE1_STALL_TICKS=8 ≈4s`); (3) NEW core-0 comms stall
  (`COMMS_PROGRESS frozen COMMS_STALL_TICKS=6 ≈3s WHILE host_active`). Records `crash::WithholdReason` (Core1Motion /
  Core0Comms) into a new breadcrumb WITHHOLD word (idx 4, RING_BASE→5; tagged 0x5748).
- RESET-LOOP / FALSE-TRIP GUARDS (load-bearing): `host_active = rx_idle_ticks < RX_ACTIVE_TICKS=12` (~6 s sticky window
  since last RX). Seeded `rx_idle_ticks = RX_ACTIVE_TICKS` so the host starts INACTIVE → a board booting with NO host
  never counts as active before the first real RX byte (prevents a boot→reset→boot loop). The 6 s sticky window BRIDGES
  the host's flow-control quiet gap: when comms wedges, skirnir streams only until its char-count window fills (~1-2 s,
  the observed ~30 lines) then goes quiet — 6 s keeps host_active=true so the ~3 s comms trip still fires; a truly
  disconnected board (RX never advances) goes inactive after 6 s and FEEDS FOREVER. Core-1 check unchanged
  (block-in-flight gated). NOT feature-gated (judged safe given the gating); raise the *_TICKS consts if a bench
  false-trip ever appears.
- Boot dump now leads with the withhold class: `[MSG:CRASH core0-comms-wedge stage=... comms-froze-first beats
  comms=N motion=M (RWDT-reset; not power-cycle)]` (or `core1-motion-wedge` + `axisN:wait_begin`). froze_first verdict
  relabeled `comms-froze-first`/`motion-froze-first`/`both-froze`/`no-stall`. Snapshot beats now = (comms_progress,
  motion_liveness). The earlier "GOAL B note above" (core-1-only) is SUBSUMED — both conditional withholds now coexist.

CAPTURE PROCEDURE (no defmt needed): `just flash` (plain), connect skirnir, stream T1_Test NORMALLY. When it wedges, WAIT
~11-12 s — do NOT power-cycle/EN-reset — for the comms-stall (or core-1) feed-withhold (~3-4 s) + the 8 s RWDT. After
reboot, skirnir's console shows the banner then a `[MSG:CRASH <class> stage=... <side>-froze-first beats comms=..
motion=..]` line (also replayed on the first `$I`/`?`). `core0-comms-wedge` + `comms-froze-first` = the comms-pipeline
wedge (the real-board case); `core1-motion-wedge` + `stage=axisN:wait_begin` pins an RMT channel-N TX-END wedge. The
`[boot] reset reason: PRO_CPU=...` (esp-println) shows `cpu-rtc-WDT` (dog fired) vs `cpu-sw-reset` (panic into
esp-backtrace = fault-handler hang). `--features defmt` adds live `watchdog:` warn/error lines but is NOT required.
THRESHOLDS: WATCHDOG_TIMEOUT=8s, WATCHDOG_FEED_INTERVAL=500ms, CORE1_STALL_TICKS=8(~4s), COMMS_STALL_TICKS=6(~3s),
RX_ACTIVE_TICKS=12(~6s sticky).

**BEST NEXT STEP = BENCH INSTRUMENTATION (source review cannot pin it):** (1) add the watchdog (defect #1) and read the
reset reason on the next lockup — distinguishes panic/fault (backtrace handler hang) from a pure spin. (2) Build
`--features defmt`, flash, stream T1_Test, read the LAST `mtrace!` line over RTT — the motion.rs trace chain
(executor loop entered → popping/lock → block popped → feed published → emit_burst → per-axis transmit/wait begin/wait
ok) localizes a core-1 wedge to the exact axis/RMT channel or shows it's NOT core 1. (3) Add a core-1 liveness counter
(AtomicU32 bumped each loop) the core-0 reporter prints, to see which core died first. (4) Check whether esp-backtrace
panicked (it logs over the SAME esp-println/RTT sink).
