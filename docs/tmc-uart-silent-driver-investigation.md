# TMC2209 UART — Silent-Driver Investigation (handoff)

**Status: ROOT CAUSE CONFIRMED ON THE BOARD (2026-07-01) — hypothesis 1 was right.** The half-duplex
push-pull transmit was implemented and flashed, and node 0 immediately came alive:

```
[DRIVER:TMC2209 X:ok Y:-- Z:-- A:--]
[MSG:TMC-PROBE X:ok Y:no-reply(fifo:0) Z:no-reply(fifo:0) A:no-reply(fifo:0)]
[MSG:TMC-LOOPBACK sent:8 got:8 match:8/8 err:none]
```

`X:ok` is the FULL init path — `IOIN.VERSION` presence read, register programming, `IFCNT` write verification —
and X stays `ok` through the 1 Hz `DRV_STATUS` poll loop (reproduced across multiple `$I+` runs), i.e. dozens of
clean datagram round-trips. **The open-drain transmit's RC-limited rising edges were defeating the TMC2209's
per-request auto-baud; push-pull edges during transmit (the RAMPS parity) fix it.** No instrument was needed.

**Later the same evening — ALL THREE drivers communicating, but the bus is MARGINAL:**

```
[DRIVER:TMC2209 X:ok Y:ok Z:ok A:--]
[MSG:TMC-PROBE X:ok Y:ok Z:ok A:no-reply(fifo:0)]
```

Timeline of the evening (identical default-build binary throughout):
1. First flash: `X:ok`, Y/Z/A silent — reproducible across `$I+` runs within that boot.
2. User reflash (`just flash`): ALL silent. Confirmed across 3 fresh `espflash reset` boots — all
   `no-reply(fifo:0)` on every node, loopback still 8/8.
3. **Live re-probe added** (see below) + reflash: `X:ok Y:ok Z:ok` — Y and Z replying for the FIRST time ever.
4. Two more fresh boots, sampled at ~3 s and ~15 s: mostly all-ok, but one early sample caught
   `TMC-PROBE Z:ok` with `DRIVER Z:--` — i.e. Z's probe had succeeded but a subsequent 1 Hz `DRV_STATUS`
   exchange failed a round and dropped it from the live health mask; it recovered by the late sample.

Reading: the push-pull fix made the bus WORK, but per-exchange reliability is not 100% and appears to DRIFT
(minutes-long all-ok stretches, then an all-silent window, then all-ok again) — classic marginal-analog
behavior on a breadboard bus (contact resistance, the reply's open-drain rise through the 4.7 k pull-up against
bus capacitance, shared-ground shifts), unless the bench state was changed between windows (12 V toggled, a
wire nudged). **The instrument is still wanted — no longer to find silence, but to grade the surviving margin:**
scope the reply edges (rise time vs the 8.7 µs bit at 115200) and the request edges at the FAR driver. Options
if margin is thin: slower baud for bring-up (38400), stronger pull-up (2.2 k), shorter/tidier bus wiring, or
moving off the breadboard. The **live re-probe** (below) meanwhile makes transient dropouts self-heal and
visible instead of frozen-at-boot.

**30-minute margin measurement (2026-07-01, `[MSG:TMC-BUS]` meter):** `X:3/1300 Y:3/1300 Z:1/1300 A:1300/1300`
— a 0.08–0.23 % per-exchange failure rate, all three drivers continuously online, every dropout self-healed by
the re-probe/poll without intervention. A captured `[MSG:TMC-IOIN Y:0x05 0xff 0x02 0x00 0x00 0x00 0x08 0xfe]`
pins the failure mode: a structurally-perfect boot-time IFCNT verification reply (register echo 0x02, payload 8
= exactly the 8 init writes acknowledged) whose CRC fails (frame 0xFE vs computed 0x3C) — i.e. genuine
single-byte line corruption on an otherwise clean exchange, not a firmware timing artifact. Verdict: the bus is
FIT FOR BRING-UP (UART carries only config + 1 Hz health polls; motion is STEP/DIR and unaffected; IFCNT
verification catches corrupted writes). The scope's remaining job is grading the reply-edge rise if the rate is
ever to reach zero; candidate levers stay: bus on the TX pads, shorter wiring, off-breadboard. ~30-minute
windows (~1300 samples/axis) resolve rate differences of ~2–3× for A/B tests.

**Firmware addition — live re-probe of absent nodes (same evening):** the `tmc_manager` poll loop now re-runs
the full `init_axis` on every ABSENT node each 1 s round (present nodes keep their `DRV_STATUS` health poll),
refreshing the packed `TMC-PROBE` outcome, so a driver that appears mid-session (power applied, wiring fixed,
module swapped) is programmed and joins `[DRIVER:]`/`[MSG:TMC-PROBE]` within ~1 s — no reboot. This turned
`$I+` into a live bench meter and is what exposed the marginality above. Cost: one 5 ms timeout per absent
axis per round, off the real-time path. Bench-note: `[DRIVER:]` and `[MSG:TMC-PROBE]` can now transiently
disagree for a node (probe outcome freezes at its last probe; the `[DRIVER:]` mask tracks live health).

**2026-07-01 (earlier) update:** both next moves from the "Leading unresolved hypotheses" section were
implemented (builds clean for Xtensa, `cargo test -p firmware-core` green at 347):
- **Half-duplex push-pull transmit** (`crates/firmware/src/tmc.rs`): the bus keeps a third GPIO9 pad handle and
  drives the pad PUSH-PULL for exactly the duration of each transmit (sharp edges both directions, replicating
  the proven RAMPS drive so the driver's per-request auto-baud can lock), then releases to open-drain + pull-up
  before the reply window opens. `flush()` waits for the TX FSM to go idle (last stop bit fully out) and the
  driver's `SENDDELAY` holds the reply off ≥ 8 bit-times, so the swap can neither truncate the request nor
  collide with the reply. The release also runs on transmit-error paths so the pad can never stay push-pull.
  **Bench test:** flash the default build, run `$I+`, read `[MSG:TMC-PROBE …]` — any token other than
  `no-reply(fifo:0)` is progress. **RESULT: `X:ok` — confirmed, see the status at the top.**
- **`tmc-scope-diag` repeating-request build** (`just build --features tmc-scope-diag` → flash): `tmc_manager`
  loops *IOIN read to node 0 → ~50 ms wait → repeat, forever* through the full production transport path — the
  stable ~20 Hz trigger burst `docs/tmc-uart-scope-bringup.md` §2 calls for.

## Symptom

On the ESP32-S3 firmware, `$I+` reports the drivers as absent and never receiving a UART reply:

```
[DRIVER:TMC2209 X:-- Y:-- Z:-- A:--]
[MSG:TMC-PROBE X:no-reply(fifo:0) Y:no-reply(fifo:0) Z:no-reply(fifo:0) A:no-reply(fifo:0)]
[MSG:TMC-LOOPBACK sent:8 got:8 match:8/8 err:none]
```

`no-reply(fifo:0)` = the request goes out, the 15 ms reply window elapses, and **zero bytes ever arrive** in
the RX FIFO. The MCU's own loopback self-test passes 8/8, so the MCU side is healthy. **The drivers simply
never answer.**

## Hardware

- **MCU:** ESP32-S3, single-wire half-duplex TMC2209 UART on **GPIO9** (UART1). Both TX and RX route to GPIO9
  (open-drain TX). `crates/firmware/src/tmc.rs`.
- **Drivers:** **BigTreeTech TMC2209 V1.3** (×3, nodes 0/1/2) — the primary target. Also tested one
  **Adafruit 6121** breakout.
- **BTT V1.3 pinout** (left col top→bottom): `EN, MS1, MS2, RX, TX, CLK, STEP, DIR`. Right col:
  `VM(VS), GND, A2, A1, B1, B2, VIO, GND`. The **TMC2209 chip and ALL decoupling caps (incl. 5VOUT) are on the
  BOTTOM side**; the top has only the VREF trimpot + copper heatsink. **5VOUT is not broken out** and is
  inaccessible while the board is on the breadboard.
- **UART pad topology:** the board exposes UART only as **RX and TX pads** (no dedicated PDN pin). They are an
  aux net routed to the chip's PDN_UART through an onboard resistor network (silkscreen refs **R8/R9/R11** near
  DIAG; exact values never obtained — BTT's V1.3 schematic PDF is not machine-readable via web tools). Measured:
  **TX idles a clean 3.3 V** (high-Z) under a pull-up; **RX idles low/loaded** (1.87 V under 4.7 k, 3.17 V under
  1 k) — RX carries an onboard pulldown.

## THE key reference fact (proven-working config)

The user previously ran **these same BTT TMC2209 V1.3 drivers** in UART mode successfully on a **RAMPS 1.4 /
ATmega** board with:
- **USB power only — NO 12 V motor supply (VS) connected.** (User confirmed this firmly, twice.)
- **A single wire + 1 kΩ resistor to the driver's `RX` pin.**
- Push-pull ATmega UART, standard baud (~115200), Marlin/TMCStepper.

Because RAMPS logic is **5 V**, the driver's **VCC_IO was 5 V** there. Our ESP32 setup runs **VIO = 3.3 V**.
This is the single most important clue and the main unexplained delta (see "Leading unresolved hypotheses").

## What has been ELIMINATED (with evidence — do NOT re-litigate)

| Ruled out | Evidence |
|---|---|
| MCU transmit / receive / levels | Firmware **loopback self-test `sent:8 got:8 match:8/8 err:none`** at both 38400 and 115200. MCU drives GPIO9 low, reads its own echo back cleanly. |
| Request datagram / CRC | `read IOIN node 0` = **`05 00 06 6F`**, byte-for-byte identical to `tmc2209-rs`, TMCStepper, and the datasheet CRC8-ATM (poly 0x07). Hand-verified CRC = 0x6F. |
| Firmware read path / reply-eating race | `drain_and_clear` finishes ~200 µs before the reply can start (SENDDELAY ≥ 8 bit-times); `read_filling` then waits 15 ms. `fifo:0` genuinely means nothing arrived. FIFO-occupancy probe (`read_ready()`, bypasses the glitch gate) confirmed the FIFO is empty at timeout. |
| Baud | Tested 38400 AND 115200 (115200 matches the proven RAMPS config). Silent at both. |
| CLK | Grounded on all drivers (TMC2209 needs CLK→GND for the internal oscillator). No change. |
| MS1/MS2 straps | X=00, Y=01, Z=10 → nodes 0/1/2. Correct per BTT doc; `encode_read_request` places node in byte 1. |
| VS (motor supply) | 12.6 V measured at each driver's VS pin. Connecting/removing VS made no difference. |
| VIO | 3.23 V measured. |
| Request levels | Valid: request-low ~0.3 V (470 Ω) / ~0.03 V (47 Ω), idle-high 3.3 V — both within TMC thresholds. |
| **TX pin** | Bus on TX (clean 3.3 V idle), valid levels — **`no-reply(fifo:0)`**. |
| **RX pin** | Bus on RX with a **1 kΩ pull-up → 3.17 V idle (valid high, acceptance-checked)** — **still `no-reply(fifo:0)`**. So BOTH pins fail with confirmed-valid logic levels. |
| Driver damage | Two known-good brands (BTT + Adafruit) both silent; the BTT drivers demonstrably worked on RAMPS. |
| VM-inrush kill | A ≥100 µF bulk cap was on the shared VS rail when 12 V was connected. |
| Bus signal integrity / wiring topology | Fixed the original per-driver-1kΩ **star** wiring (wrong) → canonical **common node**. Self-echo now frames clean (`err:none`), confirming the bus is electrically sound. |
| 5VOUT / core-power (low priority, unmeasurable) | Not broken out (bottom-side); intact factory boards + VS at the pin make a dead regulator low-probability. Not measured. |

## Diagnostic infrastructure built (in the firmware, UNCOMMITTED)

`$I+` (extended build-info) now emits rich per-node TMC diagnostics — all observe-only, host-tested in
firmware-core, added across this investigation:

- **`[MSG:TMC-PROBE X:<tok> …]`** — per-node IOIN presence-read outcome. Tokens:
  `ok` / `ok(<rx>)` (decoded despite a glitch) / `no-echo` / `no-reply` / `no-reply(fifo:N)` /
  `no-reply(fifo:N,rdy)` / `crc` / `crc(<rx>)` / `ver:0xNN` / `wrver:<a>/<e>` /
  `err:<w|r><REG>:<kind>[@<stage><variant>]`. Emitted when any node isn't `ok`.
- **`[MSG:TMC-IOIN X:0x.. …]`** — raw 8-byte reply dump of the first decode-error/fifo node (framing eyeball).
- **`[MSG:TMC-LOOPBACK sent:8 got:N match:M/8 err:<v>]`** — boot loopback self-test: MCU transmits
  `00 FF 55 AA 0F F0 33 CC` on GPIO9 and reads its own half-duplex echo. Proves MCU TX+RX+levels.
- Feature **`tmc-tx-diag`** — continuous `0x00` TX blast for a DMM to read the drooped average (TX-alive test).

`RxErrorKind` = {Overflow, Glitch, Framing, Parity}; `IoStage` = {Echo, Reply}. These tokens are how the whole
electrical story was read out without a scope.

## Chronological summary of what was tried (so it isn't repeated)

1. **`$I` vs `$I+`** — the `[DRIVER:]` line only emits on `$I+` (extended). Not a bug; user was running `$I`.
2. **Push-pull TX → `no-reply`** (under the wrong star wiring): push-pull idle-high blocked the driver pulling
   the shared line. → switched to **open-drain TX**.
3. **Open-drain + internal ~45 kΩ pull-up → `no-echo`** (echo rise too slow at 115200). → added **external
   4.7 kΩ** pull-up. Echo then framed, but with **RX framing (`@efrm`)**, then at 38400 **glitch (`@eglt`)** on
   the ECHO stage.
4. **Skip-echo refactor** — stopped reading our own glitchy TX echo; read only the reply. → advanced to a
   **reply-stage glitch (`@rglt`)**.
5. **Tolerant reply read** (glitch-tolerant, CRC as arbiter) → **`no-reply` (ReplyTimeout)** — nothing framed.
6. **FIFO-at-timeout probe** → **`fifo:0`** (truly empty; not a firmware gating problem).
7. **Wiring rework:** per-driver-1kΩ star (wrong) → **common node** (`3.3V–4.7k–BUS`, `GPIO9–470Ω–BUS`,
   `BUS→all PDN`). BUS idled 3.3 V. Still `fifo:0`; briefly saw `0xff` noise bytes (indeterminate line).
8. **BTT split RX/TX pads discovered** — RX/TX are an aux net, not raw PDN. Moved bus **TX** (clean 3.3 V idle).
   Still silent.
9. **VM (12 V) connected** (was absent) — **no change** (later shown a red herring; RAMPS worked without VS).
10. **CLK grounded** — no change.
11. **Baud restored to 115200** — no change (loopback clean 8/8).
12. **Datagram/CRC audit vs `tmc2209-rs`** — **byte-identical**, ruled out.
13. **VCC/VIO power hypothesis** (3.3 V VIO underpowers the core) — raised, then the user's "RAMPS worked
    USB-only, no VS" fact made **5VOUT-not-generating** the coherent version; but 5VOUT is unmeasurable
    (bottom-side) and low-probability on intact boards. Not resolved, de-prioritized.
14. **Moved to RX pin + 1 kΩ pull-up → 3.17 V idle (valid high) → still `no-reply(fifo:0)`.** Refuted the
    "RX-is-the-bidirectional-pin" call: **both pins fail with valid levels.**

## Leading unresolved hypotheses (for the next investigator + the scope)

The fault lives in something only visible **on the wire** between GPIO9 and the chip. Ranked:

1. **⭐ Open-drain edges defeat the driver's UART auto-baud.** The TMC2209 auto-bauds off the sync byte (`0x05`)
   of every request. Our OWN receiver is fixed-rate (so the loopback frames fine), but the driver's auto-baud is
   pickier and may mis-measure the bit period from our RC-limited open-drain rising edges → decodes garbage →
   never replies. **RAMPS used PUSH-PULL** (sharp edges both directions) — the single consistent, untested-in-
   isolation difference. **Most promising next firmware experiment:** a **half-duplex push-pull** scheme — drive
   GPIO9 push-pull *during transmit* (clean edges for auto-baud), then switch to high-Z/open-drain *during the
   reply window* to read the driver's reply. This replicates what worked on RAMPS while preserving single-wire
   reply readback. Can be tried before the instrument arrives. **IMPLEMENTED 2026-07-01 (see the status update
   at the top) — awaiting bench.**
2. **VIO = 3.3 V vs RAMPS's 5 V.** Unlikely on its own (SKR/ESP32 install base runs TMC2209 at 3.3 V VIO + VS),
   but not fully excluded. Testing 5 V VIO is unsafe without a level shifter (5 V UART reply would damage the
   3.3 V-only GPIO9).
3. **Core underpowered (5VOUT not generating).** Low probability (intact boards, VS at pin), and unmeasurable
   on this board. Only revisit if a fresh/socketed driver can be probed on its underside.
4. **TX/RX is not the chip's true bidirectional reply node**, or the onboard R8/R9/R11 network attenuates the
   reply below detectability. Needs the schematic values or a scope.

## What the instrument must answer (see `docs/tmc-uart-scope-bringup.md`)

1. Does the 4-byte request appear at the driver's pin at valid levels with clean **sync-byte edges**?
2. **Does the driver drive ANY reply edge** in the ~70 µs–15 ms window after the request? Flat-high = silent
   chip; a driven frame = we're mis-capturing a real reply.
3. UART-decode the bus: request should read `05 00 06 6F`; a reply starts `05 FF 06 …`.

**Recommendation:** a **~$12 USB logic analyzer + PulseView** (ships in days) answers this faster than the
ordered Rigol DHO804 (~2 weeks). The **repeating-request "scope-diag" firmware** (loop the IOIN read ~20 Hz
instead of once at boot) is now staged — `just build --features tmc-scope-diag` — so either instrument can
trigger on it trivially.

## Firmware state

**All of the above firmware changes are UNCOMMITTED in the working tree** (`crates/firmware/src/tmc.rs` +
`crates/firmware-core/src/{protocol.rs, drivers/tmc2209/manager.rs}`): the half-duplex push-pull transmit
(2026-07-01, replacing the always-open-drain TX), skip-echo, tolerant reply read, the loopback self-test, the
FIFO probe, all `$I+` diagnostic tokens, the `tmc-scope-diag` repeating-request feature, and baud = 115200 /
`BUS_TIMEOUT_US` = 5000. `cargo test -p firmware-core` green (347 tests); `just build` clean for Xtensa in all
feature combos (default / `tmc-scope-diag` / `tmc-tx-diag` / both). Nothing committed. These are diagnostic
scaffolding — decide what to keep vs revert once the root cause is found (the loopback + TMC-PROBE tokens are
worth keeping; the half-duplex push-pull becomes THE transport if the bench confirms it).

## Board / bench facts to carry forward

- Board serial port: `/dev/cu.usbmodem31101` (ESP32-S3 UsbSerialJtag). Flash via
  `espflash flash --partition-table partitions.csv --port /dev/cu.usbmodem31101 <elf>` from `crates/firmware`
  (source `$HOME/export-esp.sh` first). `$I+` capture via `cargo run -p skirnir -- --cli <port> <gcode-file>`
  with `$I+` in the file.
- ESP32-S3 native USB **re-enumerates on reset**, so `espflash flash --monitor` loses the reader before the app
  prints — a `--features defmt` boot-log capture of TMC init is NOT possible this way.
- The `$I+` diagnostic tokens are the primary bench readout; correlate them with instrument traces.
