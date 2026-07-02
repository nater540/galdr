---
name: tmc-uart-echo-framing
description: TMC2209 single-wire UART no-driver-detect root cause — RX framing error on the open-drain half-duplex self-echo; esp-hal 1.1.1 RxError semantics
metadata:
  type: project
---

# TMC2209 single-wire UART "no drivers detected" — echo-framing investigation

Symptom: `$I+` → `[DRIVER:TMC2209 X:-- Y:-- Z:-- A:--]`. NO drivers detected on the shared
single-wire TMC2209 bus (UART1, GPIO9, 115200 8N1, half-duplex — TX+RX both on GPIO9).

**Why:** bench bring-up of the TMC bus (`crates/firmware/src/tmc.rs`); coordinator flashes on
/dev/cu.usbmodem31101, firmware-engineer (agentId a67647107b1b52eec) implements, bug-hunter drives RCA.
**How to apply:** these esp-hal facts + the fingerprint are durable; use them before re-deriving.

## CONFIRMED facts (esp-hal 1.1.1 source, host-verified)
- `esp_hal::uart::TxError` is UNINHABITED (`pub enum TxError {}`). `Uart::write`/`flush` can never Err.
  So `Uart1TmcBus::write_all` is structurally `TmcError::Io`-proof — the transmit side is exonerated.
- The ONLY `TmcError::Io` source is `read_filling`'s `Err(_) => Io` arm (fires on `RxError`).
- `check_for_errors` (uart/mod.rs:3614) returns Err on exactly {FifoOverflowed, GlitchOccurred,
  FrameFormatViolated, ParityMismatch}. `FifoTout` is an explicit non-error `continue` (mod.rs:2449).
- Error flags are LATCHED in INT_RAW. `read_buffered` runs `check_for_errors` BEFORE reading FIFO bytes,
  so a flag latched during our own TX aborts the next read before consuming a byte. `check_for_errors`
  clears flags only on the Err path and resets the RX FIFO ONLY for FifoOverflowed (not glitch/frame/parity
  → stale bytes survive). Consequence: `drain_rx`'s `while let Ok` breaks on first Err → no-ops on a latent
  flag → cross-node contamination possible (hole #2, robustness bug, not the root symptom).
- Stable esp-hal 1.1.1 exposes NO public `rxfifo_reset`/`clear_rx_events`. Only public clear seam is
  `check_for_errors()`/`read_buffered()`/`read()`. A full FIFO flush for a non-overflow error is not in the
  public API — a robust drain must loop check_for_errors + read_buffered into scratch until clean.

## Diagnostic plumbing (already on-board)
`$I+` `[MSG:TMC-PROBE ...]` per-axis token, decoded via firmware-core `protocol.rs` InitFailure/BusErrorKind.
Enriched with RxError VARIANT (ovf/glt/frm/par) + STAGE (@e echo / @r reply) — token form `err:rIOIN:io@efrm`.

## FINGERPRINT (verified on board 2026-07-01)
`[MSG:TMC-PROBE X:err:rIOIN:io@efrm ...]` = ECHO-stage FrameFormatViolated. The open-drain self-echo's
0→1 rising edge is too slow for the 8.7µs bit at 115200 → stop-bit sampled low → framing. Reply stage is
NEVER reached (echo error aborts first) so the driver reply's edge quality is still UNKNOWN. Framing with an
allegedly-fitted 4.7k external pull-up is itself evidence the effective pull-up is ~internal 45k (rise-to-
valid ~5µs eats most of the bit) — suspect the 4.7k is not actually effective/connected.

## Electrical progression (each stage tracked by the token)
push-pull TX → `no-reply` (idle-high blocks driver through 1k series R) → open-drain + internal 45k pull-up
→ `no-echo` (RC-limited rise, echo times out) → open-drain + external 4.7k → `io@efrm` (echo arrives but frames).

## Experiment log (on board, /dev/cu.usbmodem31101)
- Flash #1: baud 115200→38400 + BUS_TIMEOUT 5k→15k. Result: `@efrm`→`@eglt` (variant MORPHED, still echo,
  still `--`). Slow-rise-framing DISPROVEN as sole cause. 4.7k pull-up later CONFIRMED good (continuity), so
  the "effective pull-up ~internal 45k" line above is DISPROVEN too.
- Flash #2: skip-echo (drop the echo read entirely; robust `drain_and_clear` loops read_buffered, treating
  an Err as a latched-flag clear + retry, replacing the fragile `while let Ok` drain_rx). Result: `@rglt` —
  we now REACH the reply and it glitches (NOT a timeout ⇒ the driver IS transmitting). Skip-echo confirmed
  correct + is the hole-#2 fix.
- Flash #3 (pending): tolerant reply-read — on RxError during the reply, capture variant + continue (the
  read_buffered Err already cleared the latch), bounded by BUS_TIMEOUT so continuous-glitch degrades to a
  clean ReplyTimeout not a hang; then let `decode_read_reply` (sync 0x05 + master addr + reg + CRC8) + the
  manager VERSION==0x21 check ARBITRATE. New token distinguishes `ok(glt)` (glitch cosmetic, CRC-good ⇒
  drivers PROVEN alive) from `dec(glt)` (glitch corrupts data ⇒ real SI/contention → bench). STATUS: awaiting
  build+flash+read-back.

## LEADING ROOT CAUSE (quantified) — reply-low VOLTAGE DIVIDER from wrong bench topology
Actual bench wiring: `3.3V --4.7k--> GPIO9`; then `GPIO9 --1k--> {X,Y,Z} PDN` (a STAR: per-driver 1k, RX
taps GPIO9 = the pull-up/MCU side, NO common PDN node). Straps CORRECT (X=00/Y=01/Z=10, MS2:MS1). Symptom:
- MCU TX open-drain pulls GPIO9 DIRECTLY to 0V → full swing → drivers RECEIVE the request fine.
- A driver reply pulls its PDN low through ITS 1k, fighting the 4.7k at GPIO9 → GPIO9 only reaches the
  DIVIDED low 3.3×1k/(1k+4.7k)=0.579V. ESP32-S3 V_IL≈0.25·VDD≈0.825V → only ~0.245V margin → any ring tips
  it high. Explains BOTH `@rglt` (marginal, ring tips it) and `no-reply`+`fifo:0` (never a clean start bit).
- Flash #4 (FIFO-at-timeout probe via public `read_ready()`=rx_fifo_count>0, no error gate) → `no-reply(fifo:0)`
  on ALL nodes → RX genuinely empty → rules out glitch-gating; reply isn't arriving as a clockable signal.
FIX (canonical common-node): tie X/Y/Z PDN together = one BUS node; ONE 4.7k on BUS→3.3V (moved off GPIO9);
ONE 1k GPIO9→BUS (replaces the three per-driver 1k). Then a driver pulls the whole BUS (which GPIO9 follows
through the 1k, RX draws no current) to a full ~0.2-0.27V low (solid, ~0.55V margin); request low at drivers
≈0.58V (valid TMC low, V_IL_tmc~1.0V); single 1k caps any TX-low/driver-high contention to 3.3mA. STATUS:
rewire + scope pending; scope pre-rewire GPIO9 in the reply window to split silent-driver (flat high) vs
divided-reply (dip to ~0.58V) — the one thing fifo:0 alone can't separate.
POST-REWIRE (common-node, 2R): `no-reply(fifo:1)` X/Z/A + `fifo:0` Y; captured byte = `0xff`. Rewire WORKED
ELECTRICALLY (fifo 0→1: levels now allow framing) but 0xff = idle-high + one spurious start-bit = NOISE, not a
reply (real reply = 8 bytes starting 0x05), and SPORADIC per-node (separate exchanges) ⇒ drivers appear
GENUINELY SILENT; the rewire REVEALED it, didn't fix it. FIRMWARE now EXONERATED for the reply path (RX gets
nothing real → no more flashes until the driver demonstrably transmits). NEXT = driver-side bench, prioritized:
(2) board/pad identity — a stepstick with SPLIT RX/TX pads + onboard 1k lets the request IN but blocks the
reply OUT (must wire the single PDN pad, or bridge RX+TX); (3) VIO measured AT each driver pin + GND continuity;
(1) scope GPIO9/BUS during $I+ = definitive silent-vs-transmit (pre-rewire scope was SKIPPED, so "does the
driver ever transmit" is UNPROVEN). Rewire tradeoff: request-low at drivers is now the 0.58V divider (was 0V) —
valid TMC low but verify; if marginal drop 1k→470Ω (req-low 0.30V, contention 7mA, reply still full-swing).

## BENCH-CONFIRMED driver-side fault: WRONG PAD on BTT TMC2209 v1.2 (split RX/TX)
Board = BTT TMC2209 v1.2, SEPARATE RX/TX pads; user wired BUS to the three "RX" pads = WRONG. VIO=3.23V/driver
(reply-power ruled out). RX pad measured 1.87V (NOT ~3.3V high-Z on our 4.7k) ⇒ RX pad is LOADED, behind the
board's onboard ~1k (RX_pad—1k—chip PDN_UART); TX pad taps chip PDN directly. So the driver's reply (driven from
PDN) reaches our bus only through that onboard 1k = ANOTHER divider ⇒ no clean reply, request still gets IN.
FIX = move BUS to the CHIP-SIDE pad (the "TX"/PDN pad) and leave RX open, OR bridge RX+TX per module (shorts the
onboard 1k). ACCEPTANCE TEST (pad-agnostic, no schematic): correct pad wired to our 4.7k-pulled BUS idles ~3.3V
(high-Z PDN); wrong/behind-R pad idles ~1.87V. KEEP external 4.7k(3.3V→BUS)+1k(GPIO9→BUS); add NO series R
(chip-side pad bypasses the onboard 1k → no double-1k). TMC2209 has NO UART-enable jumper (auto-detect); EN does
not gate UART reads; MS straps already correct. STATUS: awaiting re-solder to TX/PDN pad + re-run $I+.

## VM ruled out; MCU PROVEN healthy; fault = WRONG PAD (RX/TX vs PDN_UART) on BTT TMC2209 V1.3
- VM measured 12.6V at each VS pin → VM-absent DISPROVEN. (Correction: bug-hunter earlier ASSERTED "VIO
  suffices, VM not needed for IOIN" — an unverified guess; VM turned out present anyway, but the TMC2209 digital
  core IS fed by an internal reg off VM, so VIO-only would NOT reply. Don't assert power facts from memory.)
- Flash #5 boot LOOPBACK self-test (MCU transmits fixed 8-byte pattern, reads its own half-duplex echo) =
  `[MSG:TMC-LOOPBACK sent:8 got:8 match:8/8 err:none]` → MCU TX-drives-low + RX-reads + levels PROVEN 100% healthy,
  AND the common-node wiring is electrically clean (err:none, vs the old @efrm/@eglt on the per-driver-1k star).
  Fault is 100% driver-side and SYSTEMATIC (all 3 silent on a proven-good bus).
- ROOT (web-sourced, BTT docs): on BTT TMC2209 the single-wire UART line is PDN_UART = J1 header **Pin 4** (J1 =
  EN,MS1,MS2,PDN_UART(4),PDN_UART(5-alt),CLK,STEP,DIR); factory-default UART = pin 4. The board ALSO exposes
  separate RX/TX/CLK pads that route to PDN via an onboard resistor network — NOT the raw PDN pin. User wired the
  bus to the RX/TX pads; multiple V1.3 users report UART failing on exactly those pads (matches us). Earlier 1.87V
  on "RX" = onboard load/pulldown on that network, NOT the clean high-Z PDN. FIX = move the common bus to J1 Pin 4
  (PDN_UART) on all three; meter-confirm continuity bus→J1-4 ≈0Ω + map RX/TX→PDN resistances. Keep external 4.7k +
  series R. 5VOUT often not broken out; VM=12.6V ⇒ core powered. Sources: global.bttwiki.com/TMC2209.html; github
  bigtreetech/docs TMC2209.md; issue bigtreetech/BIGTREETECH-TMC2209-V1.2#16. STATUS: awaiting re-wire to pin 4 + $I+.

## Topology fact (esp-hal seam)
RX signal taps GPIO9 at the MCU PIN (`with_rx` on a GPIO9 clone). flush() waits for TX-idle
(`st_utx_out==0` + 10µs), so after write+flush the whole echo is already latched — no trailing-byte race.
`glitch_det` is a hardware RAW flag; esp-hal sets NO baud-scaled glitch filter ⇒ `@?glt` is a real electrical
glitch (leading hypothesis: ringing/overshoot on the fast actively-driven falling edges on breadboard leads).
