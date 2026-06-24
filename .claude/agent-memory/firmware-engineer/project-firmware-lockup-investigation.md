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

CAPTURE PROCEDURE (no defmt needed): `just flash` (plain), connect skirnir, stream T1_Test NORMALLY. When it wedges, WAIT
~8-12 s — do NOT power-cycle — for the RWDT auto-reset. After reboot, skirnir's console shows the banner then a
`[MSG:CRASH stage=... <which-core>-froze-first beats c0=.. c1=..]` line (also replayed on the first `$I`/`?`). A frozen
`stage=axisN:wait_begin` pins the wedge to RMT channel N's TX-END never firing — the prime suspect. `core1-froze-first`
confirms core 1 died before core 0. The `[boot] reset reason: PRO_CPU=...` (esp-println) line shows `cpu-rtc-WDT` (dog
fired) vs `cpu-sw-reset` (panic into esp-backtrace = fault-handler hang). `--features defmt` still adds the live mtrace
chain if streaming over a SECOND path is ever possible, but is no longer required.

**BEST NEXT STEP = BENCH INSTRUMENTATION (source review cannot pin it):** (1) add the watchdog (defect #1) and read the
reset reason on the next lockup — distinguishes panic/fault (backtrace handler hang) from a pure spin. (2) Build
`--features defmt`, flash, stream T1_Test, read the LAST `mtrace!` line over RTT — the motion.rs trace chain
(executor loop entered → popping/lock → block popped → feed published → emit_burst → per-axis transmit/wait begin/wait
ok) localizes a core-1 wedge to the exact axis/RMT channel or shows it's NOT core 1. (3) Add a core-1 liveness counter
(AtomicU32 bumped each loop) the core-0 reporter prints, to see which core died first. (4) Check whether esp-backtrace
panicked (it logs over the SAME esp-println/RTT sink).
