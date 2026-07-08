# TMC2209 UART — Oscilloscope Bring-up Walkthrough (Rigol DHO804)

Bench procedure to root-cause why the TMC2209 drivers never answer over UART (`$I+` reports
`[DRIVER:TMC2209 X:-- Y:-- Z:-- A:--]`, `no-reply(fifo:0)` on every node). Written for the **Rigol DHO804**
(4ch, 70 MHz, 12-bit, built-in UART decode) but any 2-channel scope with serial decode works.

## 0. What we already know (so we don't re-test it)

The problem is **NOT** in these — all verified:

| Verified good                   | How                                                                                                                       |
|---------------------------------|---------------------------------------------------------------------------------------------------------------------------|
| MCU transmit + receive + levels | Firmware loopback `sent:8 got:8 match:8/8 err:none` at both 38400 and 115200                                              |
| Request datagram bytes          | `read IOIN node 0` = `05 00 06 6F`, byte-identical to `tmc2209-rs` / datasheet                                            |
| Firmware read path              | No reply-eating race; `fifo:0` genuinely means nothing arrived                                                            |
| Baud                            | 115200 (matches the proven RAMPS setup) — still silent                                                                    |
| CLK                             | Grounded on all drivers                                                                                                   |
| MS1/MS2 straps                  | X=00, Y=01, Z=10 → nodes 0/1/2 (correct)                                                                                  |
| VS (motor supply)               | 12.6 V at each driver's VS pin                                                                                            |
| VIO                             | 3.23 V                                                                                                                    |
| UART pin                        | **BOTH TX and RX tested, each with confirmed-valid logic levels → both silent.** RX now used (1 kΩ pull-up, 3.17 V idle). |

Two known-good driver families (BTT TMC2209 V1.3 **and** an Adafruit 6121) are both silent on this setup, and
the same BTT drivers previously worked over UART on a **RAMPS board — with USB power only (no 12 V), single wire
+ 1 kΩ to the RX pin, PUSH-PULL, ~115200**. So the fault is **systematic and on our bench**, not the drivers or
the datagram.

**The open questions the scope must answer (highest-value first):**
1. **⭐ Are the request's edges clean enough for the driver's auto-baud?** The TMC2209 auto-bauds off the sync
   byte (`0x05`) of *every* request. Our own receiver is fixed-rate (so the loopback frames fine), but the
   driver's auto-baud is pickier. Our single-GPIO **open-drain** TX has RC-limited (soft) rising edges; RAMPS
   used **push-pull** (sharp). If our sync-byte edges are too soft, the chip mis-measures the bit period, decodes
   garbage, and never replies. **This is the leading hypothesis** — see Test B step 4.
2. **Does the driver drive *any* reply on the wire?** The definitive question. Flat-high reply window = driver
   silent. A driven reply frame = the driver *is* answering and we're failing to capture it (back to
   firmware/levels).
3. **(Low priority) Is the core powered?** The 5VOUT/core-power theory is de-prioritized: 5VOUT isn't broken out
   (chip + caps are on the board's *bottom*, inaccessible while plugged in), and a dead regulator on an intact
   factory stepstick with VS at the pin is low-probability. See Test A only if you socket/flip a driver.

## 1. Current wiring (for reference at the bench)

Latest state — **single driver isolated on the RX pin** (both TX and RX have now been tested; this is the RX
config):

```
                3.3V
                 │
               [1kΩ]            pull-up (idle-high; strong enough to beat RX's onboard pulldown → 3.17 V idle)
                 │
 GPIO9 ───────── RX             ← moved from TX to RX; NO series resistor (open-drain direct)
   (open-drain    (X only, node 0 — Y and Z removed to isolate)
    TX + RX)
```
Plus on X: VS = 12.6 V, VIO = 3.3 V, GND common, CLK → GND, EN → GND, MS1 = MS2 = GND (node 0). One ≥100 µF bulk
cap across the shared VS↔GND.

> History of pull-up/series values tried: 4.7 kΩ pull-up + 470 Ω (or 47 Ω) series on the **TX** pad (idle 3.3 V);
> then 1 kΩ pull-up, no series, on the **RX** pad (idle 3.17 V). Both gave valid levels and both stayed silent.
> When scaling back to 3 drivers on RX, three onboard pulldowns in parallel need a stronger shared pull-up
> (~470–680 Ω) to keep idle ≥ 2.5 V — verify with the meter.

## 2. Prerequisite: flash the "scope-diag" firmware

The normal firmware sends the IOIN read **once** at boot, which is nearly impossible to trigger on. Before you
scope anything, flash the **repeating-request** diagnostic build (a `tmc-scope-diag` feature that loops:
*send IOIN read to node 0 → wait ~50 ms → repeat, forever*). That gives a stable ~20 Hz request burst the scope
can trigger on and you can watch as a repeating trace.

- This build will be prepared and staged for you (ask in the session and it'll be flashed, or the exact
  `just`/`espflash` command will be provided). It changes nothing electrically — it just repeats the exact same
  `05 00 06 6F` request the normal firmware sends once.
- Keep the normal diagnostic `$I+` tokens handy too (`TMC-PROBE` / `TMC-IOIN` / `TMC-LOOPBACK`) to cross-check
  what the scope shows against what the firmware reports.

## 3. Scope setup (DHO804)

- **Probes:** use ×10 setting on the probe *and* tell the scope (Channel menu → Probe Ratio 10×). ×10 keeps the
  probe loading light so it doesn't distort the open-drain edges.
- **Channels / probe points** (clip all grounds to the common bench GND):
  - **CH1 → GPIO9** (the MCU pin).
  - **CH2 → the driver's UART pad** (currently **RX**, where the chip sees the line). With no series resistor in
    the RX config, CH1 and CH2 are nearly the same node — probe both anyway to confirm.
  - **CH3 → 5VOUT** — skip unless you flip/socket a driver (see de-prioritized Test A).
  - **CH4 → VS** (12.6 V) or leave for a trigger marker.
- **Coupling:** DC on all channels.
- **Vertical:** CH1/CH2 ≈ **1 V/div**, offset so 0 V sits near the bottom (signals swing 0–3.3 V). CH3 ≈ 2 V/div.
- **Trigger:** Edge, **CH1**, **falling**, level ≈ **1.6 V**, mode **Normal** (or **Single** for a one-shot).
- **Timebase:** start at **~200 µs/div** to capture a whole request (~350 µs at 115200) plus the reply window
  after it; zoom to ~20 µs/div once you've found the burst.

## Test A — Core power: is 5VOUT alive? (DE-PRIORITIZED)

**Low priority — do this only if you socket or flip a driver.** The TMC2209 digital core is powered by the
internal **5VOUT** regulator (fed from VS). On the BTT V1.3 the chip and all its caps (incl. 5VOUT) are on the
**bottom** side, so 5VOUT is inaccessible while the board is plugged into the breadboard. Combined with intact
factory boards + VS confirmed at the pin, a dead regulator is low-probability. Included for completeness — the
edge/auto-baud and reply-window tests (B–D) are the real priorities.

1. Identify the 5VOUT node on the BTT V1.3. It is **not** broken out to a header pin, but the chip's 5VOUT pin
   has a decoupling ceramic cap (2.2–4.7 µF) to GND right next to the chip. Locate it by elimination:
   - First find the **large VS bulk cap** (the big electrolytic/MLCC near the VMOT/GND motor pins) — it should
     read your **12.6 V**. That is VS; ignore it for this test.
   - Then look at the **small ceramic caps clustered next to the TMC2209 chip** (distinct from that VS bulk cap
     and from the 3.3 V VIO cap).
2. Power on (VS = 12.6 V). Black meter lead on GND. **Sweep** the red lead across each small cap terminal /
   test point around the chip and watch for the one net that reads **~4.8–5.1 V** — that is 5VOUT/VCC (the core
   supply). You're looking for a rail that is *neither* 12.6 V (VS) *nor* 3.3 V (VIO).
3. **Read the voltage:**
   - **~4.8–5.1 V → core is powered.** 5VOUT is fine; power is exonerated → the fault is elsewhere (proceed to
     the scope tests B–D, which will show whether the driver replies).
   - **~0 V or well below 4.5 V → root cause found.** The regulator isn't running despite VS at the pin. Likely
     a missing/weak 5VOUT decoupling cap or VS not truly reaching the chip's VS pin under load. Fix: verify VS
     continuity right at the chip's VS pin, add a 4.7 µF cap across 5VOUT↔GND if absent, and re-run `$I+`.
4. On the scope, CH3 on 5VOUT should sit at a **flat, clean ~5 V DC**. Ripple or sag under the request bursts
   would indicate a marginal regulator / inadequate decoupling.

## Test B — Is the request reaching the driver? (CH2 on BUS)

1. Flash the scope-diag build; trigger on CH1 falling edge; find the repeating request burst.
2. Look at **CH2 (BUS)** during the request — you should see the 4-byte datagram as a burst of UART pulses.
3. **Check the levels at the driver:**
   - Idle high ≈ **3.3 V** (pull-up).
   - Request low ≈ **0.03 V** (47 Ω) or **~0.3 V** (470 Ω) — must be well below the TMC's V_IL (~1.0 V).
   - If the low doesn't reach a clean valid level, the driver never sees a valid request → note it.
4. **Check the sync byte (first byte, `0x05`) timing.** The TMC2209 auto-bauds off this byte. Zoom in
   (~5 µs/div) on the first byte's edges: they should be crisp, with rise/fall reaching full level inside a bit
   period (~8.7 µs at 115200). Slow, rounded, or ringing edges here can defeat the chip's auto-baud even though
   our own fixed-rate receiver reads them fine (which is why the loopback passes).
5. Compare **CH1 (GPIO9)** vs **CH2 (BUS)** — CH1 is what our MCU drives/reads, CH2 is what the chip sees.
   They should look nearly identical across the 470 Ω; a big difference means the bus is loaded.

**Outcome:** clean full-swing request with crisp sync-byte edges on CH2 ⇒ the driver *is* receiving a valid
request ⇒ if it still doesn't answer (Test C), the problem is core power (Test A) or chip state, not the request.

## Test C — Does the driver reply? (THE definitive test)

1. Keep CH2 on BUS, trigger on the request. Set timebase so the screen shows the request **and** the ~1 ms after
   it (the reply window). The TMC2209 waits **SENDDELAY** (≥ 8 bit times ≈ 70 µs at 115200, but can be more)
   after the request before replying, then drives an **8-byte** reply frame.
2. Watch the window **after** the request ends:
   - **Flat high, only the pull-up, no driven lows → the driver is SILENT.** Combined with a good request
     (Test B) and good 5VOUT (Test A), this means the chip receives but won't answer → chip-state/config issue
     (or, if 5VOUT was bad, that's your cause). This is the expected result if the core is underpowered.
   - **The line is driven low into a multi-byte frame → the driver IS replying.** Then our firmware is failing
     to capture a reply that's physically present → back to the firmware/level side (measure the reply's low
     level at CH1 — is it reaching a valid low at the *MCU* pin through the 470 Ω?).
3. Measure the reply's low level if present: the driver pulls BUS low; CH1 (GPIO9) should follow to a valid low.
   If CH2 dips but CH1 stays high, the 470 Ω + something is preventing the MCU from seeing it.

## Test D — UART decode (read the actual bytes)

The DHO804 decodes UART on-screen — this turns the scope into a protocol analyzer and is the clearest readout.

1. Menu → **Decode** → **UART/RS232**. Source **CH2** (BUS). Baud **115200**, **8 data bits, no parity, 1 stop**,
   idle **high**, LSB-first (standard UART). Threshold ≈ **1.6 V**.
2. Trigger on the request and read the decoded bytes:
   - **Request should decode as `05 00 06 6F`** (sync, slave 0, reg IOIN 0x06, CRC). If you see exactly these,
     our transmit is provably correct *on the wire*.
   - **A valid reply decodes as 8 bytes starting `05 FF 06 …`** (sync, master addr 0xFF, reg, 4 data bytes,
     CRC). If the decode shows the request but **no reply bytes**, the driver is silent (matches Test C).
3. If you ever see garbled/партial bytes only in the reply window, that's a level/SI problem on the reply, not a
   silent chip.

## 4. Interpretation summary

| Observation                                               | Conclusion                                    | Fix                                                                                                             |
|-----------------------------------------------------------|-----------------------------------------------|-----------------------------------------------------------------------------------------------------------------|
| Request **sync-byte edges soft/rounded** (B4)             | ⭐ Driver can't auto-baud our open-drain edges | **Half-duplex push-pull** (push-pull on TX, high-Z during reply window), like RAMPS; or a much stronger pull-up |
| Request clean edges (B), reply window **flat-high** (C/D) | Chip gets a good request but won't answer     | Try a truly fresh driver; recheck EN/CLK at the chip pins; revisit core power (Test A)                          |
| Reply frame **present** on CH2 but firmware `fifo:0`      | We're mis-capturing a real reply              | Fix RX capture / reply-low level at GPIO9                                                                       |
| Request decodes `05 00 06 6F`, no reply bytes (D)         | Confirms silent driver (not a TX problem)     | → edge quality / chip state                                                                                     |
| **5VOUT ≈ 0 V** (Test A, only if measured)                | Core unpowered                                | VS continuity to chip pin; add 5VOUT cap                                                                        |

## 5. Quick reference — expected values

- Idle bus: **3.3 V**
- Request low at driver: **~0.03 V** (47 Ω) / **~0.3 V** (470 Ω)
- Reply low (if driver answers): should reach a valid low (< ~0.8 V) at GPIO9
- 5VOUT (core): **~5 V**
- VS: **12.6 V**, VIO: **3.3 V**
- Bit period at 115200: **8.68 µs**; 4-byte request ≈ **350 µs**; 8-byte reply ≈ **700 µs**
- SENDDELAY (request→reply gap): **≥ ~70 µs**

## 6. If the scope confirms a silent driver with good power + good request

That would mean the chip receives a valid request, is powered, but won't answer — an unusual state. Next steps
then: (a) swap in a **brand-new, never-powered** TMC2209 with the same wiring; (b) probe EN and CLK **at the chip
pins** (not the header) for continuity/level; (c) reconsider whether replicating the RAMPS topology exactly
(2-pin, push-pull, 1 kΩ to the driver's RX, VIO = 5 V with a level shifter on the UART line) is worth building.

---
*Save this file; it's the bench-side companion to the firmware's `$I+` diagnostics. Bring the `$I+` output and
the scope traces together — the tokens tell you what the firmware sees, the scope tells you what's physically on
the wire.*
