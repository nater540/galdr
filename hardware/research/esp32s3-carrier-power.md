# ESP32-S3 Carrier Board — Module Pinout & Power Architecture

Research for the Galdr CNC carrier board (KiCad 10) that hosts a **Lonely Binary ESP32-S3 DevKit**
on two female pin-socket headers. Covers module identity, full header pinout mapped to the firmware
GPIO contract, header footprint, and the 12 V barrel-jack power tree.

> **Date:** 2026-06-18. Sources cited inline; see end of file. Read the confidence note in §1 before
> committing the footprint — **verify every pin against the silkscreen on the physical board.**

---

## 1. Module identity & confidence

**The Lonely Binary "ESP32-S3 DevKit / DevKitC" is an ESP32-S3-DevKitC-1 clone.** Both the
lonelybinary.com listing and the Amazon "Gold Edition N16R8" listing describe it as carrying an
**ESP32-S3-WROOM-1 (N16R8: 16 MB flash, 8 MB PSRAM)** module, dual-row 2.54 mm headers, and the same
"three mutually-exclusive power options" (USB / 5V pin / 3V3 pin) as the genuine Espressif board.
Lonely Binary explicitly references the "ESP32-S3-DevKitC-1-N16R8" variant and the WROOM datasheet on
its own product page.

- The variant most likely in hand is **N16R8** (WROOM-1, PCB or IPEX antenna). Some Lonely Binary SKUs
  are N8R2 (8 MB/2 MB) — flash/PSRAM size does **not** change the pinout, so it is irrelevant to the
  carrier. What matters is that it is a **WROOM-1-based DevKitC-1 layout**, which it is.

**Confidence on the pinout: HIGH that it is the DevKitC-1 layout; the pin table below is the official
Espressif ESP32-S3-DevKitC-1 v1.1 pinout.** I could not retrieve a Lonely-Binary-specific pin-by-pin
silkscreen table (their datasheet is gated; the Cirkit Designer third-party page is partial/garbled —
it claims "2×19", which is wrong and contradicts every first-party source). Clones occasionally rotate
the board 180° or swap which physical row is J1 vs J3, and **v1.0 vs v1.1 differ** (RGB LED on GPIO48
in v1.0, GPIO38 in v1.1 — and the J3 column order is reorganized between revisions).

> ⚠️ **ACTION REQUIRED:** Before finalizing the footprint, lay the actual board on the bench and confirm
> (a) which end is pin 1, (b) that the 3V3 pair, the 5V/G pair, and the GPIO order match the table below.
> The strapping/GPIO *net* assignments in the firmware contract are fixed; the *physical header pin* they
> land on is only as trustworthy as the silkscreen match.

---

## 2. Header layout

- **Two rows, 22 pins each (2 × 22 = 44 pins total).** Espressif labels them **J1** (one long side) and
  **J3** (the other long side).
- **Pitch: 2.54 mm.** Row-to-row spacing on a DevKitC-1 with a WROOM-1 is the standard **0.9" (22.86 mm)
  centre-to-centre** wide layout — wide enough to straddle the module; **measure the actual board** (some
  clones are slightly narrower). This is breadboard-compatible per all sources.
- Logic is **3.3 V only** — never drive any GPIO from 5 V.

---

## 3. Full dual-row pinout — physical pin № ↔ silkscreen ↔ GPIO ↔ Galdr net

Pinout is **ESP32-S3-DevKitC-1 v1.1** (Espressif official user guide). Firmware net names from
`CLAUDE.md` GPIO contract.

### J1 header (22 pins)

| J1 Pin | Silk | GPIO | Galdr net / use | Notes |
|:--:|:--:|:--:|---|---|
| 1 | 3V3 | — | **+3V3** (module reg out) | power, see §5 |
| 2 | 3V3 | — | **+3V3** | tied to pin 1 |
| 3 | RST | — | EN / reset | optional reset button |
| 4 | 4 | GPIO4 | **Z_STEP** | step |
| 5 | 5 | GPIO5 | **X_DIR** | dir |
| 6 | 6 | GPIO6 | **Y_DIR** | dir |
| 7 | 7 | GPIO7 | **Z_DIR** | dir |
| 8 | 15 | GPIO15 | **SPINDLE_DIR** | |
| 9 | 16 | GPIO16 | **FEED_HOLD** | input |
| 10 | 17 | GPIO17 | **CYCLE_START** | input |
| 11 | 18 | GPIO18 | **A_STEP** (spare 4th-axis) | |
| 12 | 8 | GPIO8 | **STEP_ENN** (common ENN) | active-low enable |
| 13 | 3 | GPIO3 | *(reserve — strapping)* | **keep undriven at boot** |
| 14 | 46 | GPIO46 | *(reserve — strapping)* | **keep undriven at boot** |
| 15 | 9 | GPIO9 | **TMC_UART** | single-wire UART1 |
| 16 | 10 | GPIO10 | **X_LIMIT** | limit input |
| 17 | 11 | GPIO11 | **Y_LIMIT** | limit input |
| 18 | 12 | GPIO12 | **Z_LIMIT** | limit input |
| 19 | 13 | GPIO13 | **SPINDLE_PWM** | LEDC → RC → op-amp → 0-10 V |
| 20 | 14 | GPIO14 | **SPINDLE_EN** | |
| 21 | 5V | — | **+5V** (board 5 V rail) | **power IN/OUT — see §5** |
| 22 | G | — | **GND** | |

### J3 header (22 pins)

| J3 Pin | Silk | GPIO | Galdr net / use | Notes |
|:--:|:--:|:--:|---|---|
| 1 | G | — | **GND** | |
| 2 | TX | GPIO43 | UART0 TX (console) | leave for debug console |
| 3 | RX | GPIO44 | UART0 RX (console) | leave for debug console |
| 4 | 1 | GPIO1 | **X_STEP** | step |
| 5 | 2 | GPIO2 | **Y_STEP** | step |
| 6 | 42 | GPIO42 | *(free)* | JTAG MTMS |
| 7 | 41 | GPIO41 | *(free)* | JTAG MTDI |
| 8 | 40 | GPIO40 | *(free)* | JTAG MTDO |
| 9 | 39 | GPIO39 | *(free)* | JTAG MTCK |
| 10 | 38 | GPIO38 | *(free; onboard RGB LED v1.1)* | avoid if LED used |
| 11 | 37 | GPIO37 | *(do NOT use)* | **WROOM-1 octal PSRAM — reserved** |
| 12 | 36 | GPIO36 | *(do NOT use)* | **WROOM-1 octal PSRAM — reserved** |
| 13 | 35 | GPIO35 | *(do NOT use)* | **WROOM-1 octal PSRAM — reserved** |
| 14 | 0 | GPIO0 | *(reserve — strapping/BOOT)* | **keep undriven at boot** |
| 15 | 45 | GPIO45 | *(reserve — strapping)* | **keep undriven at boot** |
| 16 | 48 | GPIO48 | *(free; RGB LED on v1.0)* | |
| 17 | 47 | GPIO47 | *(free)* | |
| 18 | 21 | GPIO21 | **PROBE** | G38.x probe input |
| 19 | 20 | GPIO20 | **USB_D+ (reserved)** | leave to USB — do not route |
| 20 | 19 | GPIO19 | **USB_D- (reserved)** | leave to USB — do not route |
| 21 | G | — | **GND** | |
| 22 | G | — | **GND** | |

### Firmware GPIO contract → physical pin quick-reference

| Net | GPIO | Header pin |
|---|:--:|:--:|
| X_STEP | 1 | **J3-4** |
| Y_STEP | 2 | **J3-5** |
| Z_STEP | 4 | **J1-4** |
| X_DIR | 5 | **J1-5** |
| Y_DIR | 6 | **J1-6** |
| Z_DIR | 7 | **J1-7** |
| STEP_ENN (common) | 8 | **J1-12** |
| TMC_UART | 9 | **J1-15** |
| X_LIMIT | 10 | **J1-16** |
| Y_LIMIT | 11 | **J1-17** |
| Z_LIMIT | 12 | **J1-18** |
| SPINDLE_PWM | 13 | **J1-19** |
| SPINDLE_EN | 14 | **J1-20** |
| SPINDLE_DIR | 15 | **J1-8** |
| FEED_HOLD | 16 | **J1-9** |
| CYCLE_START | 17 | **J1-10** |
| A_STEP (spare) | 18 | **J1-11** |
| PROBE | 21 | **J3-18** |
| USB_D- (reserved) | 19 | **J3-20** |
| USB_D+ (reserved) | 20 | **J3-19** |
| Strapping — keep undriven | 0 | **J3-14** |
| Strapping — keep undriven | 3 | **J1-13** |
| Strapping — keep undriven | 45 | **J3-15** |
| Strapping — keep undriven | 46 | **J1-14** |

**All 21 contract GPIOs are broken out** on the DevKitC-1 headers, and none collide with the
WROOM-1 octal-PSRAM-reserved pins (35/36/37) or the USB pins (19/20). The four strapping pins
(0/3/45/46) all land on header pins — make sure nothing on the carrier pulls them at boot (no
LEDs/pull-downs to GND on 0/45/46, no pull on 3). Leave them as floating/no-connect header pins.

**Strapping reminders for the carrier:** GPIO0 = BOOT (pulled high on module; a button to GND is OK
but no fixed pulldown). GPIO45 = VDD_SPI voltage select, GPIO46 = boot-mode/ROM-msg, GPIO3 = JTAG
source — all must read their default at reset, so present them as bare/no-connect pads.

---

## 4. Recommended KiCad footprint for the two module headers

Use **two 1×22 vertical female pin sockets**, 2.54 mm pitch:

```
J1:  Connector_PinSocket_2.54mm:PinSocket_1x22_P2.54mm_Vertical
J3:  Connector_PinSocket_2.54mm:PinSocket_1x22_P2.54mm_Vertical
```

- **22 pins per row** (confirmed: 2 × 22 = 44). Do **not** use the 1×19 footprint that the third-party
  Cirkit page implies — that contradicts Espressif and the WROOM-1 DevKitC-1 layout.
- Place the two sockets **0.9" (22.86 mm) apart, row 1 ↔ row 1**, then **adjust to the measured board
  width.** Confirm with calipers — clone row-spacing can vary by a column.
- Orient so **J1 pin 1 (3V3) and J3 pin 1 (G) are on the same end** (per the table). The DevKitC-1 has
  J1-1 (3V3) diagonally opposite the USB end; J3-1 (G) is at the same end as J1-1. **Confirm against the
  silkscreen** — if the clone is mirrored, swap the row assignments, not the net list.
- Female sockets (not pin headers) so the male pins soldered to the *module* plug into the *carrier* —
  the module stays removable. Consider machined/round sockets if you want lower insertion force, or
  add 2×right-angle if the board must lie flat.

---

## 5. Carrier power architecture

### Topology

```
                 5.5×2.5 mm barrel jack (12 V DC, centre-positive)
                          │
            ┌─────────────┴───────────────┐
            │  Reverse-polarity P-FET       │   (or Schottky; P-FET preferred, ~0 drop)
            │  + TVS (SMBJ16A/18A)          │
            │  + resettable fuse / 2-3 A    │
            └─────────────┬─────────────────┘
                          │  +12V_PROT  (VM rail)
        ┌─────────────────┼──────────────────────────────┐
        │                 │                               │
   Bulk caps        TMC2209 VM ×3-4                   12V→5V buck
  (470-1000 µF      (motor power; each driver           (MP1584 / LM2596 module
   electrolytic     bulk 100 µF + 100 nF local)          or TPS562201 IC)
   + 100 nF)                                              │  +5V  (≈1 A budget)
                                                          │
                                                   ┌──────┴───────┐
                                                   │  Module 5V pin (J1-21)
                                                   │  (DevKit onboard LDO → 3V3)
                                                   └──────┬───────┘
                                                          │  +3V3 (module reg out, J1-1/2)
                                                          │
                                            VCC_IO for TMC2209 ×3-4, opto pull-ups
                                            (see 3V3 budget below — likely OK,
                                             but add a carrier LDO as insurance)
```

### Input protection & bulk (12 V)

- **Reverse polarity:** prefer a **series P-channel MOSFET** (e.g. **DMP3017SFG**, **SQ4953**, or any
  ≥30 V, low-Rds(on) P-FET) with gate-to-source resistor + clamp — near-zero forward drop, unlike a
  Schottky which would burn ~0.4 V × motor current as heat. A Schottky (**SS54 / SMC-package, ≥3 A**)
  is the simpler fallback if board area is tight.
- **TVS:** **SMBJ16A** (16 V standoff, unidirectional) across +12V→GND after the fuse, to clamp supply
  transients from the motor rail / spindle PSU.
- **Fuse:** a **PTC resettable (2–3 A hold)** or a 3 A blade/glass fuse on the 12 V input. Size above
  peak motor draw (4 × TMC2209 at ~1–1.5 A motor each is mostly handled by per-driver bulk caps; steady
  12 V draw is well under 2 A for desktop-CNC-class NEMA17, but include margin for the spindle if it shares
  this jack — if the WS55 spindle has its own PSU, 2 A is ample).
- **Bulk capacitance:** **470–1000 µF / 25 V** electrolytic on +12V_PROT close to the TMC2209 cluster,
  plus **100 nF** ceramic per driver VM pin and a **100 nF** at the jack. TMC2209 chopping injects ripple
  — generous VM bulk is cheap insurance against driver brownout/UVLO.
- **Power LED:** green LED + ~2.2 kΩ from +12V_PROT to GND (≈4 mA). Put it *after* the reverse-protection
  so it also indicates correct polarity.

### 12 V → 5 V buck

Recommendation in priority order:

1. **MP1584EN buck module** (drop-in, ~$1, set output to 5.00 V, rated 3 A / good to ~1.5–2 A thermally).
   Cheapest path to standalone power; mount as a daughter module. Easy to source on the same Amazon/AliExpress
   channels as the devkit.
2. **LM2596 module** — even more ubiquitous, but larger and noisier (150 kHz); fine here, 5 V @ ≥2 A easily.
3. **Integrated IC if you want it on-board:** **TI TPS562201** / **TPS54331** (3 A, 17–28 V in, simple
   external L+C) or **MP2315**. Use if you don't want a piggyback module. TPS562201 is the smallest-effort
   3 A synchronous buck for this rail.

**5 V current budget:**

| Load | Typical | Notes |
|---|--:|---|
| ESP32-S3 DevKit (Wi-Fi TX peaks) | 350–500 mA | RF bursts dominate; idle ~80 mA |
| TMC2209 VCC_IO + logic (×4) | ~40 mA | drawn via 3V3, sourced from the devkit's LDO off 5 V |
| Opto pull-ups / limit/probe LEDs | 20–60 mA | depends on opto count |
| Status LEDs, misc | ~30 mA | |
| **Subtotal** | **~0.6 A** | |
| **Design the buck for ≥1 A** | | comfortable headroom, low thermal stress |

A 3 A-class buck (MP1584/LM2596) loafs at this load and runs cool.

### USB vs barrel coexistence ⚠️

The DevKitC-1 power options are **mutually exclusive** by Espressif's own warning — USB 5 V, the 5V pin,
and the 3V3 pin must not be driven simultaneously. If the carrier feeds 5 V into J1-21 **and** a USB
cable is plugged into the module, the carrier's 5 V will **backfeed the USB host's VBUS** through the
module's USB connector — bad for the laptop/hub.

Two acceptable approaches:

- **Simplest (document it):** "Power from one source at a time — either the 12 V barrel jack **or** USB,
  never both. Unplug USB before applying barrel power." A clear silkscreen note + a jumper to disconnect
  the buck's 5 V output covers this. For a bench tool wired permanently to a 12 V supply this is fine.
- **Robust (diode-OR):** put a **Schottky (SS34/SS54) in series with the buck's 5 V output** before J1-21
  so the carrier can source the module but the module's USB VBUS cannot backfeed the buck. This costs
  ~0.3 V on the 5 V rail (buck → ~4.7 V at the pin → LDO still has dropout margin to 3.3 V). Combine with
  a note that USB then powers the module while the buck idles. **Recommended** if you expect both to be
  connected (flashing over USB while the motor PSU is on is a very common workflow).

> Practical recommendation: include the **series Schottky on the buck's 5 V output**, set the buck to
> ~5.1–5.2 V to compensate for the diode drop, and still print the "one-source-preferred" note. This lets
> you flash/debug over USB with 12 V applied without backfeed risk.

### 3.3 V — rely on the module LDO, or add a carrier LDO?

The DevKitC-1 onboard regulator is a **5 V→3.3 V LDO rated 1 A** (genuine Espressif boards use the
**SGM2212-3.3** 1 A LDO; clones typically use an **AMS1117-3.3**, also 1 A nominal). The module itself
draws meaningful 3V3 current during Wi-Fi (the chip is on this rail), so the **available 3V3 headroom for
external loads is well under the 1 A rating** — realistically a few hundred mA, and the SOT-223/SOT-23
package will heat up near that.

**Our 3V3 external load** = 3–4 × TMC2209 VCC_IO (each VCC_IO draws only a few mA — it's a logic-reference/
charge-pump support rail, not the motor supply) + opto/limit pull-ups. **Total external 3V3 ≈ 30–60 mA.**
That is comfortably within the devkit LDO's spare capacity.

**Recommendation:**

- **Baseline: rely on the devkit's onboard 3V3** (J1-1/J1-2) for TMC VCC_IO and pull-ups. The ~40–60 mA
  load is small; this is the simplest BOM (no extra LDO).
- **Insurance (cheap, recommended): footprint an optional carrier 3V3 LDO** (e.g. **AMS1117-3.3** or a
  better low-noise **MCP1700/AP2112** fed from the carrier 5 V rail) with **DNP jumper** selecting source.
  Populate it only if bench measurement shows the devkit LDO sagging or running hot, or if you later add
  more 3V3 loads (display, extra optos). Keep the carrier-3V3 and module-3V3 nets **separable by a jumper**
  so you never parallel two regulators.
- **Do not** try to power the module *from* a carrier 3V3 into its 3V3 pin while also using the 5V pin —
  pick one input.

---

## 6. Risks & assumptions

1. **Pinout = Espressif ESP32-S3-DevKitC-1 v1.1.** I could not obtain a Lonely-Binary-specific silkscreen
   table. **Verify J1/J3 pin order and pin-1 orientation against the physical board before fabricating.**
   Clones are sometimes mirrored or use the v1.0 column order.
2. **v1.0 vs v1.1 difference:** RGB LED is GPIO48 (v1.0) vs GPIO38 (v1.1); J3 column ordering also shifted.
   None of these are in the firmware contract, so the impact is limited to the "free/RGB" rows — but it is
   a tell for which revision (and thus which silkscreen order) you have.
3. **Row spacing (0.9"/22.86 mm) is the standard DevKitC-1 value but unverified for this clone — measure
   with calipers.** A one-column error makes the module unseatable.
4. **GPIO35/36/37 reserved** on the WROOM-1 N16R8 (octal flash/PSRAM). The contract avoids them; keep them
   no-connect. If a future N8 (quad-SPI) variant frees them, that's a bonus, not something to rely on.
5. **Strapping pins 0/3/45/46 land on header pins** — ensure the carrier adds **no fixed pulls/LEDs** that
   would change their boot-time level. Treat them as no-connect.
6. **USB backfeed** if 5V-pin and USB are connected together — mitigated by the series Schottky on the buck
   output and/or the one-source note (§5).
7. **Devkit onboard 3V3 LDO is shared with the ESP32-S3 itself**; the 1 A rating is not all available to
   external loads. Our ~40–60 mA budget is fine, but the optional carrier LDO footprint de-risks scope creep.
8. **Regulator part on the clone is unconfirmed** (SGM2212 vs AMS1117). Both are ~1 A; the budget analysis
   holds either way. Does not affect the carrier design since the carrier feeds 5 V, not 3V3.
9. **Spindle PSU sharing:** the budget assumes the WS55-220 spindle has its own supply. If it draws from
   this 12 V jack, re-size the fuse, P-FET, and bulk caps for the added current.

---

## Sources

- ESP32-S3-DevKitC-1 v1.1 user guide (header J1/J3 pinout, 2×22, three power options, WROOM-1 modules):
  https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html
- ESP32-S3-DevKitC-1 v1.0 user guide (revision differences):
  https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.0.html
- ESP32-S3-DevKitC-1 schematic PDF (regulator/power section):
  https://dl.espressif.com/dl/schematics/SCH_ESP32-S3-DevKitC-1_V1.1_20221130.pdf
- Lonely Binary ESP32-S3 product page (module = WROOM-1 N16R8, DevKitC-1 clone):
  https://lonelybinary.com/en-us/products/s3
- Lonely Binary ESP32-S3 N16R8 Gold Edition (Amazon listing — WROOM-1, 16MB/8MB, dual USB-C):
  https://www.amazon.com/Lonely-Binary-ESP32-S3-N16R8-Development/dp/B0FNQVPBV4
- Random Nerd Tutorials — ESP32-S3 DevKitC pinout reference (cross-check):
  https://randomnerdtutorials.com/esp32-s3-devkitc-pinout-guide/
- Last Minute Engineers — ESP32-S3 DevKitC pinout (cross-check, 2×22):
  https://lastminuteengineers.com/esp32-s3-devkitc-pinout-reference/
