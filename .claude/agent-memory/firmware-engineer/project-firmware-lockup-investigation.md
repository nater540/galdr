---
name: project-firmware-lockup-investigation
description: 2026-06-24 investigation of the intermittent both-cores-dead T1_Test.tap firmware lockup (hard-reboot, no WDT) — what was ruled OUT, the two real latent defects found (no watchdog; esp-storage auto-park cache-window vs cached-flash motion code), and the bench-instrumentation plan since source review can't pin it.
metadata:
  type: project
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

CAPTURE PROCEDURE for next lockup: flash `just flash --features defmt`, stream T1_Test, watch RTT. (a) After it wedges,
the LAST mtrace line localizes a core-1 wedge to the axis/channel (or shows core 1 fine). (b) `watchdog: core-1 liveness
STALLED` while the feed task still logs = core 1 died FIRST; both silent = core 0 (or both) wedged. (c) ~8 s after the
wedge the RWDT should auto-reset; the reboot's `boot: reset reason ... = cpu0-rtc-WDT` confirms the dog fired (vs
`cpu0-sw-reset` = a panic into esp-backtrace = the fault-handler-hang hypothesis). Default `just flash` also works
(watchdog + reset-reason via esp-println active, no defmt traces).

**BEST NEXT STEP = BENCH INSTRUMENTATION (source review cannot pin it):** (1) add the watchdog (defect #1) and read the
reset reason on the next lockup — distinguishes panic/fault (backtrace handler hang) from a pure spin. (2) Build
`--features defmt`, flash, stream T1_Test, read the LAST `mtrace!` line over RTT — the motion.rs trace chain
(executor loop entered → popping/lock → block popped → feed published → emit_burst → per-axis transmit/wait begin/wait
ok) localizes a core-1 wedge to the exact axis/RMT channel or shows it's NOT core 1. (3) Add a core-1 liveness counter
(AtomicU32 bumped each loop) the core-0 reporter prints, to see which core died first. (4) Check whether esp-backtrace
panicked (it logs over the SAME esp-println/RTT sink).
