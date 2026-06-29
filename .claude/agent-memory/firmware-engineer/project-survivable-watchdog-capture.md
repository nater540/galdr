---
name: project-survivable-watchdog-capture
description: capture-reset-gated TIMG1-ISR survivable dual-dog watchdog feed + capture-at-withhold (§17.10/17.11 Signature-B instrument) + the verified esp-hal 1.1.1 / esp32s3 0.35.2 watchdog/timer/PAC facts.
metadata:
  type: project
---

**Survivable watchdog feed + capture-at-withhold (DIAGNOSTIC `capture-reset` build, §17.10/§17.11 layer-2).** Built
2026-06-28 (firmware-engineer; NOT flashed/committed — handed to embedded-bug-hunter who owns flash+decode). The
production default build is UNCHANGED (core-0 async `watchdog_feed` + the §17.3 ALARM:17 fail-safe). Files:
`crates/firmware-core/src/diag.rs` (pure), `crates/firmware/src/survivable_watchdog.rs` (NEW, the ISR/PAC),
`crash.rs`, `comms.rs`, `main.rs`.

**Why:** in Signature B a comms task wedges in a non-yielding await and STARVES the cooperative core-0 thread-mode
executor → the async `watchdog_feed` stops running, the RWDT feed stops, the heartbeat freezes — yet the RWDT
mysteriously did NOT fire (>60 s silent). Fix = feed the dog(s) from a context that SURVIVES a core-0 executor stall:
a TIMG1 hardware-timer ISR.

**How to apply / design (all `#[cfg(feature = "capture-reset")]`):**
- PART A pure logic in `diag.rs` (host-tested, 324 firmware-core tests green): `watchdog_decision(WatchdogInputs) ->
  WatchdogDecision{feed_rwdt,feed_swd,withhold_reason}` replicates `watchdog_feed`'s withhold logic exactly (core1 ||
  comms || dead_zone → withhold BOTH dogs; precedence Core1Motion>Core0Comms>DeadZone). `WithholdKind` is a PURE
  mirror of `crash::WithholdReason`. `WindowedStallCounter` (§13.8 alternating-vs-pure) = a true trailing-N ring of
  the last `STALL_WINDOW_LEN=16` samples packed 1-bit/interval in a u16; `record(timed_out)` shifts+masks+ORs,
  `count()` = popcount. 1:1 alternation reads N/2, pure run reads N, long quiet → 0. (TDD caught my first
  linear-age-out model collapsing a 1:1 run to 0 — the ring model is the correct one.)
- PART B firmware: TIMG1 `PeriodicTimer` @250 ms + `#[esp_hal::handler]` ISR (`survivable_watchdog::start`/`fire`).
  ISR maintains frozen-tick statics (sole writer, Relaxed), calls `watchdog_decision`, then raw-PAC feeds BOTH dogs or
  (on withhold) `capture_withhold` ONCE per transition (`WITHHOLD_LATCHED`). SuperWDT armed via `rtc.swd.enable()`
  right after `rtc.rwdt.enable()` in main.rs. In the capture build `watchdog_feed` + its 4 private consts
  (CORE1/COMMS_STALL_TICKS, RX_ACTIVE_TICKS, DEAD_ZONE_BACKSTOP_ARMED) are gated OUT (`not(capture-reset)`) and a thin
  `comms::watchdog_heartbeat()` (snapshot ring only, no Rtc, no feed) replaces it; the ISR feeds register-side so the
  `Rtc` is just parked (`let _ = rtc;`).
- usb_tx fingerprint plumbing: `usb_tx` publishes the packed `UsbTxStall` word + resp_len + `WindowedStallCounter`
  count into 3 gated atomics (`USB_TX_STALL_FINGERPRINT{,_LEN}`, `USB_TX_STALL_WINDOW_COUNT`) on each write timeout;
  the ISR copies them into the breadcrumb on withhold (so a hard-B lock that never hit K=3 still leaves the last-known
  fingerprint). New `crash::idx::USB_TX_STALL_WINDOW` (after TRUNC_BUILD_ID, RING_BASE bumped +9→+10),
  `record_usb_tx_stall_window` (gated WRITER), `Breadcrumb.usb_tx_stall_window`, `wnd=N` on the boot usbtx line
  (DECODE unconditional → production reads `wnd=0`).

**VERIFIED esp-hal 1.1.1 / esp32s3 0.35.2 PAC facts (cite these — do NOT re-derive):**
- RWDT/SuperWDT regs live under **`RTC_CNTL`**, exposed via **`esp_hal::peripherals::LPWR`** (the S3 has NO `LP_WDT`
  singleton — `soc_has_lp_wdt` is unset → esp-hal aliases `LPWR as LP_WDT`). `LPWR::regs()` is an ASSOCIATED fn on the
  TYPE, callable even after `Rtc::new(peripherals.LPWR)` consumed the instance (esp-hal's own `Rwdt::feed` does this).
- RWDT raw feed (mirrors `Rwdt::feed`): `wdtwprotect().write(|w| w.bits(0x50D8_3AA1))` → `wdtfeed().write(|w|
  w.wdt_feed().set_bit())` → `wdtwprotect().write(|w| w.bits(0))`. (esp-hal writes the wkey via raw `w.bits()`, NOT a
  `wdt_wkey()` field.)
- SuperWDT: `Swd` is `#[cfg(swd)]` (set on S3). `Swd::enable()` only does `swd_conf().write(swd_auto_feed_en(false))`
  — NO `set_timeout`, NO `feed`. Raw SWD feed: `swd_wprotect().write(|w| w.swd_wkey().bits(0x8F1D_312A))` →
  `swd_conf().modify(|_,w| w.swd_feed().set_bit())` (MODIFY preserves auto-feed-off) → re-lock with key 0.
- `SWD_CONF` reset value `0x04b0_0000`: out of reset the SuperWDT is ACTIVE, auto-feed OFF, SIGNAL_WIDTH=300; period
  is SHORT (seconds-scale, exact TRM value not extracted). `esp_hal::init()` neutralizes it (`swd.disable()`). RISK:
  if it false-trips at 250 ms feed, drop the SWD arm (RWDT-only survivable feed still improves on status quo).
- TIMG1 is FREE (esp-hal disables its WDT, timer available; TIMG0 = esp-rtos time source, do NOT touch). TIMG1 is
  independent of the esp-rtos/embassy time driver (esp-rtos "now"=SystemTimer). `PeriodicTimer::new(timg1.timer0)` +
  `set_interrupt_handler(fire)` (→ `interrupt::bind_handler` → `enable(Cpu::current())` — core-0-fielded from `main`)
  + `start(Duration)` + `listen()`. The ISR MUST `clear_interrupt()` each fire (reach the parked timer via a
  `'static` raw-ptr; publish the ptr BEFORE `listen()` so the first fire can always clear). A hardware ISR fires even
  when the core-0 thread-mode executor is starved (idle hook `waiti 0`, no `interrupt_free` around the poll loop).

**Build:** all FOUR Xtensa configs clean under `RUSTFLAGS="-C link-arg=-Tlinkall.x -D warnings"` (bare `-D warnings`
clobbers `-Tlinkall.x`). See [[project-firmware-lockup-investigation]] for the §17 split and [[espflash-version-pin]].
