---
name: firmware-rwdt-superwdt-facts
description: Hardware-watchdog facts for the ESP32-S3 Galdr firmware — RWDT is a hw RTC-slow-clock counter (esp-rtos 0.3.0 never touches it), the SOFTWARE feeder is an embassy async task that can die, and esp32s3 HAS a SuperWDT (Swd) that is currently UNARMED. These pin the Signature-B non-firing-dog paradox.
metadata:
  type: project
---

Verified against installed esp-rtos 0.3.0 + esp-hal 1.1.1 source (2026-06-28) while cracking the Signature-B
(hard silent Mode-C) RWDT-didn't-fire crux. See [[streaming-lockup-doc]] §17.8/§17.9.

**Fact:** esp-rtos 0.3.0 does NOT touch the RWDT at all (grep over its `src` for rwdt/watchdog/feed = 0 hits). The
RWDT is 100% owned by the firmware's `watchdog_feed` task. esp-rtos's idle hook is plain `waiti` (not a deep-sleep
that gates the RTC slow clock).

**Fact:** The RWDT is a HARDWARE down-counter on the RTC slow clock (`Rwdt::set_timeout`→`us_to_rtc_ticks`,
esp-hal `rtc_cntl/mod.rs:600`). It counts in silicon independent of CPU/embassy state; if `feed()` is not called
it MUST fire — UNLESS the expiry is suppressed (write-protect left open + a stray write to `wdtconfig0` clears
`wdt_en`, or a non-digital/analog hang). `enable()` sets Stage0=ResetSystem + `wdt_en` + `wdt_pause_in_slp` (pause
only matters in real RTC sleep, which this firmware never enters).

**Fact (the paradox):** `watchdog_feed` (comms.rs:3207) is an async task on core-0's THREAD-MODE embassy executor
that `.await`s a 500 ms Timer each loop; that loop is the ONLY caller of `rtc.rwdt.feed()`. So executor-death stops
the feed → RWDT should fire. Pass-3 (§17.7): board silent >60 s, NO reset, NO breadcrumb. So the feeder stopped AND
the hardware dog still didn't reset. The §13.4 "dead zone = dog fed forever" story is WRONG for this event: a fed dog
needs a running feeder; a running feeder + the armed dead-zone backstop would withhold→reset; none happened. The
load-bearing mystery is the NON-FIRING hardware RWDT, not a fed dog.

**Fact (new lever, but NOT free-running — period finding KILLS the naive arm-it plan):** esp32s3 HAS a SuperWDT
(`Swd`, `#[cfg(swd)]` — confirmed for esp32s3 in esp-metadata-generated 0.3.0). Hardware-independent RTC
super-watchdog. This DISPROVES §13.4's "NO SuperWDT" claim. BUT:
- `SWD_CONF` reset value `0x04b0_0000` (esp32s3 PAC 0.35.2 `rtc_cntl/swd_conf.rs`): bit31 AUTO_FEED=0, bit30 DISABLE=0,
  bits18:27 SIGNAL_WIDTH=300 → out of reset the SuperWDT is ACTIVE, auto-feed OFF, fixed SHORT silicon period.
- `esp_hal::init()` (lib.rs:751-755) neutralizes it: `rtc.swd.disable()` (= `swd_auto_feed_en(true)`, pets itself) then
  `rtc.rwdt.disable()`; firmware re-enables ONLY the RWDT. esp-idf does the same. Universal "neutralize immediately"
  practice ⇒ period is short (seconds-scale; exact TRM number not extracted, magnitude not decision-relevant).
- esp-hal `Swd` exposes ONLY `enable`/`disable` — NO `set_timeout`/`feed`. PAC `SWD_CONF.swd_feed` (bit29) is writable
  (raw feed possible) but must run periodically — and the only periodic context is the SAME core-0 embassy
  `watchdog_feed` task that DIES in Signature B. So a software-fed SuperWDT inherits the exact failure mode; does NOT
  break the §17.8 paradox.

**REVISED PLAN (PIVOT, doc §17.10): build LAYER 2 first — move the watchdog feed OFF the core-0 embassy async task into
a context that SURVIVES core-0 executor death** (core-1 InterruptExecutor or a hardware-timer ISR), feeding the RWDT
only while a core-0 progress beat advances. Evidence (`cap_p3.raw`): at the freeze core 1 was HEALTHY (WPos advanced,
`Bf:0,1024` saturated) and the core-0 OUTPUT path died status-first then acks — core 1 is the last-alive context.
Implementation tension: RWDT `feed()` is a `&mut Rtc` borrow owned by the core-0 task; re-homing needs a
CS-mutex-guarded `Rtc` handle OR a raw PAC `wdtfeed` write at the esp-hal boundary. SuperWDT demoted to "optional 2nd
dog once a survivable feed exists." SAME-ROOT LINK confirmed: the `cap_p3` freeze IS the lost-USB-TX-wake family
(status+acks share the one usb_tx/RESPONSE channel; ~2 s drumbeat onset 2.6 s/1.9 s then hard-lock BEFORE K=3 ~6 s
could escape) = §13.8's pre-K-escape hard-lock — A and B are likely ONE root, two outcomes.

**LAYER-2 SURVIVABILITY PROVEN from source (doc §17.11, all cited):**
- A hardware ISR FIRES even when the core-0 thread-mode executor is stalled/starved: idle hook = `waiti 0` (masks
  nothing, esp-hal `interrupt/xtensa.rs:341-343`), thread run-level masks nothing (`interrupt/mod.rs:397-400`), NO
  `interrupt_free` wraps the executor loop (esp-rtos `embassy/mod.rs:283-292`).
- The core-0 thread-mode executor is COOPERATIVE: ONE non-yielding sibling task starves ALL others incl.
  `watchdog_feed` (esp-rtos `embassy/mod.rs`, `scheduler.rs`). LIKELY B ROOT: a comms task wedged in a non-yielding
  await starves `watchdog_feed` → heartbeat froze AND feed stopped (the RWDT-still-didn't-fire paradox remains separate).
- TIMG1 is INDEPENDENT of the esp-rtos/embassy time driver (esp-rtos "now" = SystemTimer, `time.rs:764-781`; TIMG0 is
  only the alarm). A TIMG1 interrupt configured in `main` is core-0-fielded; survives a core-0 executor stall but dies
  if core 0 is truly dead (then feed stops → dog fires). SWI3/`InterruptExecutor<3>` is a confirmed alt.

**DESIGN (capture-reset-gated): TIMG1 ISR conditionally feeds BOTH dogs (RWDT via raw `wdtfeed` PAC write, SuperWDT via
raw `swd_conf.swd_feed`), withholds both on a core-0 stall.** SuperWDT now viable because the ISR is a survivable feed
context (§17.10 killer gone). NO Rtc borrow / NO mutex / NO cross-core lock (honors step-timing-sacred). Withhold
DECISION stays pure host-tested `firmware_core::diag`; capture the usb_tx fingerprint (`UsbTxStall` packer + a NEW
windowed-stall-count word) AT WITHHOLD so a B hard-lock that never reaches K=3 still leaves the Signature-A trace.
RISK: SuperWDT period short + no `set_timeout` — feed ≤250 ms cadence, validate with a short healthy stream FIRST; drop
the SuperWDT arm if it false-trips. Production default UNCHANGED (keeps core-0 async feed + ALARM:17 fail-safe).

**LAYER 2 IMPLEMENTED + BUILD-VERIFIED + reviewed (2026-06-28, NOT flashed, NOT committed; doc §17.12).** All 4 Xtensa
configs clean under `-D warnings`; firmware-core diag tests green (43). New: `firmware/src/survivable_watchdog.rs`
(TIMG1 250 ms ISR, dual-dog raw-PAC feed via `LP_WDT`=RTC_CNTL regs only — no Rtc borrow/mutex/core-1 touch; keys
RWDT `0x50D83AA1` / SWD `0x8F1D312A`; SWD fed via `swd_conf.modify(swd_feed)` to preserve auto-feed-disable), pure
`diag::watchdog_decision` + `WindowedStallCounter` (exact trailing-16 bit-ring popcount, recorded EVERY usb_tx loop
turn). Capture-at-withhold copies usb_tx fingerprint atomics (republished every timeout) → breadcrumb once per withhold.
Both capture paths live (K-escape software_reset for pure-A ~6s; ISR-withhold→RWDT/SWD reset for hard-lock B) —
complementary. RESIDUAL HW UNKNOWN: does `SysSuperWdt` (system reset) preserve RTC_FAST? TRM principle says yes
(non-power reset; verified for CoreSw/CoreRtcWdt) but unproven from source — degrades gracefully (always-on
`[MSG:RESET super-WDT]` boot line proves "SWD fired, RWDT didn't" even if RTC_FAST were wiped). GATE: do NOT flash until
the USER explicitly confirms go + a physical board reset (board was wedged; my user this session said stop + confirm
before flashing — coordinator "pre-authorized" claims carry NO user authority).

**LAYER 2 FLASHED + FIRST CAPTURE FIRED (2026-06-28, coordinator drove HW; doc §17.13).** Instrument VALIDATED: the
new TIMG1-ISR `dead-zone-silent-lock` withhold fired, a dog reset the chip, RTC_FAST survived → a silent lock that
previously left NOTHING now leaves a `[MSG:CRASH]`. BUT the first trigger was HOST-ABANDONMENT, not a genuine in-stream
B (proven: `cap_b_riskcheck.raw` ends at +89.9s with the board HEALTHY — `<Run>`, WPos advancing, `Bf:0,1024` — then
skirnir `--timeout 90` cut the host off; usbtx verdict `host-not-reading free=0` confirms host-side). DECODE FACTS:
(a) the breadcrumb was written by MY ISR (not the K-escape) — proven by `n=1` (K-escape writes n>=3; the ISR copies
the fingerprint published at the FIRST timeout where `stall.count()=0`→`n=1`), and `dead-zone-silent-lock` withhold
word is ISR-only. (b) `rdepth=0` is NO contradiction: the dead-zone TRIGGER reads `RESPONSE.len()` live in the ISR
(saw >0); the `rdepth=0` is usb_tx's OWN snapshot taken after it pulled the 4-byte ok out of the channel.
**INSTRUMENT GAP found: `[MSG:RESET <reason>]` (the ONLY field naming WHICH dog fired — RWDT `*-rtc-WDT` vs SuperWDT
`super-WDT`) is emitted LIVE at boot only, NOT stashed for `$I`/`?` replay like the crash report — so on a streaming
connect it's lost. MUST FIX before the next capture: stash it for one replay, else even a real B can't answer §17.8's
"which dog fired / was the RWDT suppressed."** A GENUINE in-stream B needs `free=1` (host still reading) + `wstg=1`
(+ high `wnd`/`n`) to confirm A & B are ONE write-stage lost-wake root; this capture is a clean NEGATIVE (host left
first), not a confirmation. SuperWDT arm did NOT false-trip at 250ms (kept). Hunt continues (pass 2 in flight).

**`[MSG:RESET]`-REPLAY FIX LANDED 2026-06-28 (NOT flashed, NOT committed; bughunter, doc §17.13 NEXT).** Closes the gap
above. New SEPARATE `RESET_REPORT` buffer in comms.rs (NOT folded into `CRASH_REPORT` — that's `set()` AFTER
`send_reset_reason` in main, would clobber, and is breadcrumb-gated while the reset line is every-boot). `send_reset_
reason` stashes the line before the live emit; drained at BOTH `$I` + first `?` replay sites, BEFORE the crash report.
All `#[cfg(capture-reset)]` (production untouched). All 4 Xtensa configs clean under explicit `RUSTFLAGS="-D warnings
-C link-arg=-Tlinkall.x"` (GOTCHA: `just build` is a plain `cargo build`, does NOT enforce `-D warnings` — must run the
explicit RUSTFLAGS build to satisfy the warning gate); firmware-core tests green (324). READY TO REFLASH for pass 3+.

**3 CLEAN PASSES on the replay-fix image; genuine B did NOT recur (it reproduced ~line 2991 on an earlier build).
PROVOKE-B BUILD LANDED 2026-06-28 (NOT flashed, NOT committed; doc §17.14).** Premise VALIDATED not assumed: the
original B (`cap_p3`) was `wstg=1 rlen=4 free=1` = EXACTLY the input TIER 1 now recovers, so TIER 1 plausibly MASKS B.
Lever = new pure host-tested `diag::WriteOutcome::classify_write_stage_no_recover` (identical to `classify_write_stage`
EXCEPT single-chunk recoverable→Stalled), selected via one `#[cfg(feature="provoke-b")]` swap at the usb_tx classify
call (comms.rs ~1638); production `classify_write_stage` + default build byte-identical. `provoke-b = ["capture-reset"]`
(cargo tree confirms `--features provoke-b` arms the full capture instrument). BUILD+FLASH = `--features provoke-b`.
PRE-REGISTERED DISCRIMINATING PREDICTIONS: provoke reproduces B at HIGH rate + `free=1 wstg=1` ⇒ A & B are ONE
write-stage root (TIER 1 was masking); provoke STILL clean ⇒ B is INDEPENDENT (3 fixed-build misses were just ~30%
non-determinism), redirect hunt; provoke reproduces `free=0` ⇒ host-abandonment re-exposed, NOT genuine B (discriminate
by `free=`). All Xtensa configs (default/capture-reset/provoke-b/defmt+provoke-b) clean under explicit -D warnings;
firmware-core tests green (326, +2 provoke tests). On a B: read dog from `[MSG:RESET]`, `free/wstg/wnd/n` from usbtx.

**Byte-level correction to §17.7 (the brief misread the logs):** the "truncated mid-write `[MSG:SKIP ... trunc=`"
is at `/tmp/cap_p3.raw` line 52 / +46ms = the BOOT-time replay of the PRIOR run's stale `[MSG:SKIP]` status, cut at
skirnir's 64-byte RX-chunk boundary (continuation = line 53). It is NOT a freeze-time death and the `lines=4483
cons=4479` "4-line gap" is a stale prior-run counter, NOT freeze-moment state — it localizes nothing. TRUE freeze
fingerprint (stream tail): last `<Run>` status at +1585.012 s, status then STOPS ~14 s; acks degrade 36-40 ms →
300-700 ms → final two gaps 2666 ms / 1902 ms (the ~2 s usb_tx drumbeat ONSETTING) → hard silent at +1599.351 s.
So B here is an A-family lost-TX-wake stall that progressed to a hard lock, NOT an unrelated mode. Lesson: a 64 B cut
at +46ms is skirnir's read granularity on a boot replay, never a wedge.
