---
name: project-firmware-lockup-investigation
description: 2026-06-24 streaming-lockup investigation — ROOT CAUSE FOUND via the RTC crash breadcrumb (core-1 RMT TX wait() hung on ch0/X) + the FIX (cap bursts to 46 events so the 48-slot RMT block is never completely full). Also: the watchdog/breadcrumb/task-watchdog build, and what was ruled out.
metadata:
  type: project
---

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
