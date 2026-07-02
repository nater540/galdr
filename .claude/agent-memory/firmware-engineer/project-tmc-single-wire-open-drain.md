---
name: tmc-single-wire-open-drain
description: TMC2209 single-wire UART on GPIO9 needs OPEN-DRAIN TX + EXTERNAL 4.7k pull-up (physical layer CONFIRMED fixed on-board); esp-hal with_tx gotcha
metadata:
  type: project
---

On-board `$I+` on the TMC-PROBE diagnostic build reported `[MSG:TMC-PROBE X:no-reply Y:no-reply Z:no-reply A:no-reply]`
— echo loops back on GPIO9 (RX/pin-matrix/UART fine) but NO driver replies. Root cause: the MCU TX was push-pull, so
it actively held the shared single-wire line high during the driver's reply window and the driver (sourcing through the
1 kΩ series R) couldn't pull the pad low. Fix implemented in `tmc::init`: configure GPIO9 **open-drain + pull-up** so the
MCU releases the line to idle-high.

**Why:** true single-wire half-duplex requires the master to release the bus; open-drain drives only low and floats high.
This is the standard TMC2209 PDN_UART topology.

**How to apply:**
- esp-hal 1.1 GOTCHA: `Uart::with_tx(pin)` unconditionally calls `pin.apply_output_config(&OutputConfig::default())` =
  push-pull. Pre-configuring the pad does NOT survive. You MUST re-apply open-drain AFTER `with_tx`/`with_rx`.
- `Flex::new`/`Output::new` call `init_gpio()` which RESETS signal routing — do NOT use them after `with_tx` (breaks the
  UART routing). Instead reach the public `apply_output_config` on `esp_hal::gpio::interconnect::OutputSignal` (the type
  `with_tx` uses via `AnyPin: Into<OutputSignal>`). It writes only pad drive-mode/pull registers keyed by GPIO number,
  no routing/output-enable change, no Drop. Seam = a third `unsafe clone_unchecked()` handle to GPIO9 → `.into()` →
  `apply_output_config(OpenDrain + Pull::Up)`. Apply LAST so `with_rx`'s input-config pass doesn't clobber the pull-up
  (pull is shared between input and output config).
- PULL-UP CAVEAT (bench-critical): internal pull-up is ~45 kΩ. At 115200 (bit ≈ 8.7 µs) vs breadboard capacitance the
  rise time is MARGINAL; in open-drain the rising edges govern BOTH the reply AND the MCU's own echo. So after this
  change, if TMC-PROBE stays `no-reply` OR regresses to `no-echo`, suspect pull-up strength, NOT the open-drain seam —
  fit an EXTERNAL ~4.7 kΩ pull-up to 3.3 V for a conclusive test.
- STATUS (updated): PHYSICAL LAYER CONFIRMED FIXED on-board. Open-drain TX + an EXTERNAL 4.7 kΩ pull-up (GPIO9→3.3V —
  the internal ~45 kΩ WAS too weak, as flagged) makes `$I+` show no echo/reply timeouts; bidirectional bytes flow. The
  failure is now at the DATAGRAM/CONFIG layer: on-board `$I+` shows `[MSG:TMC-PROBE X:err Y:err Z:err A:err]` — i.e.
  IOIN READS decode with correct VERSION (drivers genuinely present) but init_axis then hits a Bus error during the
  8-write config burst (GSTAT/SLAVECONF/GCONF/CHOPCONF/IHOLD_IRUN/TPOWERDOWN/TPWMTHRS/PWMCONF) → WRITES aren't landing.
  Diagnostic enriched `err`→`err:<w|r><REG>:<kind>` then further to `...:io@<stage><variant>` (RxError variant + echo/
  reply). ROOT CAUSE NOW CONFIRMED on-board: `X:err:rIOIN:io@efrm` = ECHO-stage FrameFormatViolated → the open-drain
  self-echo rises too slowly for the 8.7 µs bit @115200 (rise-to-valid ~5 µs eats the stop bit) → framing error. The
  effective pull-up behaves like the weak internal ~45k, hinting the external 4.7k may not be doing much. CURRENT BENCH
  TEST (flash #1, firmware-only, single variable): tmc.rs baud 115200→38400 (26 µs bit ≈ 3× margin; TMC2209 auto-bauds
  off the sync nibble so no driver change) + BUS_TIMEOUT_US 5000→15000 (38400 reply streams ~2.1ms; empty-poll budget).
  Both marked bench-diagnostic, restore when baud restored. PENDING follow-up (hold until flash #1 reads back): robust-
  drain + skip-echo refactor — loop check_for_errors+read_buffered into scratch until clean (clear latched flags) instead
  of drain_rx breaking on first Err, and stop treating a self-echo framing error as fatal (the correctness fix for the
  echo hole regardless of electrical outcome). FLASH #1 RESULT: `@efrm`→`@eglt` — variant morphed (framing→glitch) at
  38400 but still ECHO-stage, still all `--`; baud drop did NOT fix it → the glitch is a real electrical event (esp-hal
  has no baud-scaled glitch filter), consistent with an ineffective 4.7k pull-up. FLASH #2 (BUILT, firmware-only, baud
  KEPT at 38400 to give the reply its best decode shot): SKIP-ECHO refactor in tmc.rs — replaced fragile `drain_rx`
  (whose `while let Ok` no-oped on a latched flag) with bounded `drain_and_clear` (Err=continue: read_buffered's
  check_for_errors is the ONLY public latched-flag clear seam in esp-hal 1.1.1); read_reg/write_reg now write+flush then
  `drain_and_clear` (discard echo + clear self-echo glitch/frame flag) instead of reading the echo as a datagram; reply
  read stays FATAL so a reply-side glitch surfaces as `@r<variant>` (that's the diagnostic). Verified: flush() waits
  TX-idle (st_utx_out==0) so echo fully latched + SENDDELAY means reply not started (no trailing-byte race). NEXT
  read-back decodes: `X:ok`/version ⇒ echo self-glitch was the whole problem, DONE; `@r<glt|frm>` ⇒ reply ALSO bad;
  `@r`+`to`/no-reply ⇒ driver silent. FLASH #2 RESULT: skip-echo WORKED — now REACH the reply and it's `@rglt` (glitch on
  the REPLY stage, NOT a timeout) → driver IS putting bytes on the bus; glitch detector trips but data may be intact.
  FLASH #3 (BUILT, firmware-only, baud KEPT 38400): make the REPLY read TOLERANT of RxError and let CRC/decode arbitrate.
  read_filling (now reply-only) Err arm is non-fatal — record first variant in new `reply_glitched` field, `continue`
  (check_for_errors already cleared the latched flag; 0 bytes read; SAME BUS_TIMEOUT governs so continuous glitch →
  clean ReplyTimeout, no hang). read_reg folds reply_glitched at decode into NEW firmware-core stages:
  RespondedDespiteGlitch(RxErrorKind)→`ok(<v>)` (decoded despite glitch — driver PROVEN alive, reliance visible not
  swallowed, still counts present in [DRIVER:]) and DecodeErrorGlitched→`crc(<v>)` (glitch corrupted decode, distinct
  from clean-line `crc`). classify_ioin preserves RespondedDespiteGlitch through the present-driver arm. NOTE: reply
  glitch no longer produces a fatal @<stage><variant> io_detail — that path (last_rx_error) is now inert (only reply
  reads produced RxError, now tolerant); glitch info flows via ok()/crc() tokens instead. NEXT read-back: `X:ok`/`ok(glt)`
  +version ⇒ drivers PROVEN ALIVE, glitch cosmetic at this SI, bring-up UNBLOCKED (SI hardening = follow-up, not
  blocker); `crc(glt)` ⇒ glitch genuinely corrupts data ⇒ real SI/contention ⇒ BENCH (single-driver + scope GPIO9);
  ReplyTimeout ⇒ continuous glitch starved read ⇒ also bench. FLASH #3 RESULT: all nodes `no-reply` (tolerant loop
  degraded to clean ReplyTimeout — the bound worked). FLASH #4 (BUILT, firmware-only, baud KEPT 38400, all 4 drivers):
  post-ReplyTimeout FIFO snapshot to tell (a) driver truly silent from (b) reply IS in the FIFO but glitch gate starved
  the read. Verified seam: `Uart::read_ready()` returns rx_fifo_count()>0 WITHOUT calling check_for_errors → reports
  occupancy even while a glitch flag is latched. read_reg on ReplyTimeout: sample read_ready() first, then new
  `drain_snapshot()` (robust drain into [u8;8]+saturating count, Err=continue to clear flag, bounded 32); store drained
  bytes in last_reply. ReplyTimeout is now a STRUCT variant `{ fifo: u8, ready: bool }` packed in the existing 16-bit
  stage (fifo bits0-6 sat@127, ready bit7); token `no-reply(fifo:N)` / `no-reply(fifo:0)` / `no-reply(fifo:0,rdy)`;
  fifo>0 also dumps drained bytes via the existing TMC_IOIN_RAW `[MSG:TMC-IOIN]` machinery. NEXT read-back DECISIVE:
  `no-reply(fifo:0)` all ⇒ (a) driver genuinely silent ⇒ BENCH single-driver+scope GPIO9 (request clean on wire? any
  driver drive reply? straps distinct 0/1/2/3?); `no-reply(fifo:8 +bytes)` decoding to sync 0x05/reg/good-CRC ⇒ (b)
  driver REPLIED, gated away ⇒ firmware read-strategy fix (drain-after-clear, immediate read post-clear before re-latch);
  `fifo:>8`/garbled ⇒ multiple drivers answering (contention/duplicate straps) ⇒ bench strap check. FLASH #4 RESULT:
  `no-reply(fifo:0)` all — FIFO genuinely EMPTY at timeout; eliminated pad/divider/VIO/straps/CLK/request-level. User
  has NO scope/logic-analyzer, only a MULTIMETER. FLASH #5 (BUILT, firmware-only): scope-free MCU self-test, 2 parts.
  PART A boot LOOPBACK self-test (default build, runs once before presence loop): TX+RX share GPIO9 so MCU always
  self-echoes; transmit fixed pattern P=[00,FF,55,AA,0F,F0,33,CC] (all levels + both phases + nibble splits; NOT a valid
  addressed request so no driver replies — safe with drivers attached), read back tolerant, compare position-by-position.
  New firmware-core LoopbackReport{ran,got,matched,err:Option<RxErrorKind>} + `[MSG:TMC-LOOPBACK sent:8 got:N match:M/8
  err:<none|ovf|glt|frm|par>]` on $I+. PART B feature `tmc-tx-diag` (Cargo): replaces tmc_manager with an endless 0x00
  back-to-back TX blast (start+8 zero bits=90% low duty) so a DMM reads GPIO9's drooped average — isolates the TX half,
  no RX. Verified seam: `Uart::read_ready()`=rx_fifo_count()>0 (already used in #4). tx-diag build's normal-path dead code
  suppressed via `#![cfg_attr(feature="tmc-tx-diag", allow(dead_code,unused_imports))]` (default build keeps full
  strictness). tx_diag loop yields Timer 100µs/refill so watchdog/executor stay alive while deep FIFO stays saturated.
  READ-BACK: LOOPBACK got:8 match:8/8 err:none ⇒ MCU TX+RX+levels PROVEN healthy ⇒ fault 100% driver-side; got:0 ⇒ MCU
  broken ⇒ then flash tmc-tx-diag + DMM to split TX-vs-RX half; got:partial/match<8/err ⇒ TX ok but levels/edges marginal
  (SI). BAUD RESTORED to PRODUCTION 115200 (+ BUS_TIMEOUT_US back to 5000): the datagram/CRC audit came back NEGATIVE
  (our frame is byte-identical to tmc2209-rs/datasheet — not the bug), and the 38400 drop was only a bench-diagnostic for
  framing on the OLD (now-fixed) star wiring; 115200 matches the user's proven-working RAMPS config. Everything else
  (open-drain TX, common-node topology, skip-echo, tolerant reply read, loopback self-test, all $I+ tokens) UNCHANGED —
  loopback at 115200 on the clean bus is itself informative (does the self-echo frame cleanly at the higher rate?). NOTE:
  defmt boot-monitor CANNOT catch TMC init on this board — ESP32-S3 native USB re-enumerates on post-flash reset,
  espflash monitor dies before app prints; detail MUST come over `$I+`. All UNCOMMITTED; coordinator flashes.
  See [[project-hardware-map]].
