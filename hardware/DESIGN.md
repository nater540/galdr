# Galdr CNC Carrier Board — Schematic Design

KiCad 10 schematic-only project (`galdr-carrier.kicad_pro`). Two sheets in one hierarchy:

- **Page 1 — `galdr-carrier.kicad_sch`:** the carrier board that hosts a **Lonely Binary ESP32-S3 (Gold Edition,
  N16R8)** dev board on two female header rows and breaks its GPIOs out to **socketed BigTreeTech TMC2209 V1.3
  stepstick modules**, opto-isolated limit inputs, and a 12 V supply.
- **Page 2 — `galdr-spindle-cond.kicad_sch`:** a separate **spindle 0–10 V conditioning daughterboard** (DOC-07)
  that mates to the carrier over a 6-wire cable. See "Spindle conditioning daughterboard" below.

The two pages connect electrically through KiCad **global labels** (the project uses a flat label netlist), so the
inter-board nets `SPIN_PWM` / `SPIN_EN` / `SPIN_DIR` / `+12V` / `GND` are the same net on both sheets — they model
the physical cable between the boards.

> **Scope:** schematic + symbols + footprints only. No PCB layout. Every component has a symbol and an assigned
> footprint. **THT footprints only — no SMD** (axial/disc/radial passives, DIP/TO packages). Connectivity is
> expressed with named global labels / power symbols and is **ERC-clean in KiCad 10.0.3**
> (`kicad-cli sch erc` → 0 violations). The generated sheet uses a flat label-based netlist (labels sit on pins, no
> drawn wires) — open it in Eeschema and re-arrange/route to taste; the electrical netlist is already correct.

## What it is / isn't (correcting the first research pass)

- **TMC2209 = socketed BTT TMC2209 V1.3 modules**, *not* bare QFN chips and *not* Adafruit breakouts. All the driver
  support (sense resistors, charge pump, decoupling, the LA-package chip) lives on the BTT module. The carrier only
  provides the socket, power, signal routing, address straps, and the motor/UART nets. Driver datasheet:
  `docs/hardware-references/BTT-TMC2209-V1.3.pdf`.
- **MCU = Lonely Binary ESP32-S3 dev board** (pin-compatible with the Espressif ESP32-S3-DevKitC-1 2×22 layout,
  verified against the user's board photo `docs/hardware-references/LB-ESP32S3-N16R8-Pinout.webp`), socketed on two
  1×22 female headers — *not* soldered, *not* the bare WROOM module.

## Binding firmware contract (from `docs/00-architecture.md`, must match)

| Signal | GPIO | Net |
|---|---|---|
| X/Y/Z step | 1 / 2 / 4 | `X_STEP` / `Y_STEP` / `Z_STEP` |
| X/Y/Z dir | 5 / 6 / 7 | `X_DIR` / `Y_DIR` / `Z_DIR` |
| Common stepper enable (ENN, active-low) | 8 | `STEP_EN` |
| TMC single-wire UART | 9 | `TMC_UART` → 1 kΩ → `TMC_BUS` |
| X/Y/Z limit (NC fail-safe) | 10 / 11 / 12 | `X_LIM` / `Y_LIM` / `Z_LIM` |
| Spindle PWM / EN / DIR | 13 / 14 / 15 | `SPIN_PWM` / `SPIN_EN` / `SPIN_DIR` |
| Feed-hold / cycle-start | 16 / 17 | `FHOLD` / `CYCSTART` |
| Spare 4th-axis (A) step | 18 | `A_STEP` |
| Probe | 21 | `PROBE` |
| USB D−/D+ | 19 / 20 | reserved → no-connect |

`A_DIR` is routed to GPIO42 (free header pin) for the optional 4th axis; the current firmware only generates A *step*
on GPIO18, so A-axis DIR is a future-firmware hook. Strapping pins (GPIO0/3/45/46) and the octal-PSRAM pins
(GPIO35/36/37) are left **no-connect** with no carrier pulls.

## Lonely Binary ESP32-S3 header pinout (verified from board photo)

**LEFT row (J_DKL, pins 1→22):** 3V3, 3V3, RST, IO4, IO5, IO6, IO7, IO15, IO16, IO17, IO18, IO8, IO3, IO46, IO9,
IO10, IO11, IO12, IO13, IO14, 5V0, GND
**RIGHT row (J_DKR, pins 1→22):** GND, IO43, IO44, IO1, IO2, IO42, IO41, IO40, IO39, IO38, IO37, IO36, IO35, IO0,
IO45, IO48, IO47, IO21, IO20, IO19, GND, GND

> ⚠ Verify against the silkscreen and **measure the row-to-row spacing with calipers** before committing a PCB
> layout — the two `PinSocket_1x22_P2.54mm_Vertical` footprints assume the standard DevKitC-1 ~0.9″ row pitch.

## BTT TMC2209 V1.3 module (socket pinout, datasheet p.5)

17 pins = standard 2×8 StepStick socket + a separate DIAG pin. Left col (top→bottom): EN, MS1, MS2, RX, TX, CLK,
STEP, DIR. Right col (top→bottom): VM, GND, A2, A1, B1, B2, VIO, GND. DIAG sticks up at top-centre. Default mode is
**UART**; VM range **12–28 V** (12 V is the floor); 2 A peak; use active cooling > 1.2 A.

Per-socket carrier wiring:

- **EN** → `STEP_EN` (shared by all four). **STEP/DIR** → per-axis. **CLK** → GND (internal oscillator).
- **VM** → +12 V, **VIO** → +3V3 (logic ref from the dev board), **GND** → GND. 100 nF VM decoupling per socket.
- **RX + TX** tied together per module → `TMC_BUS`. One **1 kΩ** series resistor at the MCU (GPIO9), plus a 20 kΩ
  idle pull-up on the bus to +3V3. (BTT default routes PDN_UART to the RX/TX pads; tying them is the single-wire
  topology. The R10 solder jumper on the module is for alternate UART pin mapping — leave at factory default.)
- **MS1/MS2 = UART node-address straps** (internal pulldowns; high = +3V3):

  | Axis | Node | MS1 | MS2 |
  |---|---|---|---|
  | X | 0 | GND | GND |
  | Y | 1 | +3V3 | GND |
  | Z | 2 | GND | +3V3 |
  | A | 3 | +3V3 | +3V3 |

  Microstepping is set over UART (`CHOPCONF.MRES`), not by these pins.
- **A1/A2/B1/B2** → 4-pin 3.81 mm pluggable screw terminal per axis (coil order A1,A2,B1,B2).
- **DIAG** → broken out to a 1×4 header (`J_DIAG`) for future sensorless homing (firmware polls DIAG as a future
  option). Not wired to a GPIO by default.

Footprint: custom `Galdr:TMC2209_StepStick_Socket` (17 THT pads = the 16-pad Pololu/StepStick grid + DIAG).

## Opto-isolated limit / probe inputs (X, Y, Z, PROBE)

Functional opto isolation per channel with the **LED in series with the switch, sourced from +12 V**, so the
firmware's NC fail-safe is preserved with **no `$5` invert**:

```
+12V ──[R_LED 680Ω]──┬──(PC817 LED A→K)──┬──> SIG ── JST pin2
                     └──[D 1N4148 ⟂]─────┘
PC817 emitter → GND, collector → COL
+3V3 ──[R_PU 10k]── COL ──[R_DEB 1k]── GPIO ──[C_DEB 100nF]── GND
JST: 1 = +12V (field, for 3-wire NPN-NC sensors), 2 = SIG, 3 = GND
```

Truth table (intact NC switch / 3-wire NPN-NC not triggered → LED ON → phototransistor pulls **GPIO LOW**; triggered
or broken wire → LED OFF → pull-up → **GPIO HIGH**). A 2-wire dry switch uses JST pins 2–3 only. Connector:
**JST-XH 3-pin** (`JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical`), opto **PC817** (`Isolator:PC817`, DIP-4). See
`research/opto-limit-inputs.md` for the full derivation and the per-row truth table.

## Power tree

```
12V 5.5×2.5 jack ──[F1 3A]──[Q1 P-FET rev-prot]──┬── +12V (VM rail) ──> 4× TMC VM, opto field, motor supply
   (center +)        TVS(P6KE16A)┐  470µF ┐ 100nF ┐ PWR-LED
                                 GND       GND      GND
 +12V ──[MP1584 buck module]── +5V_BUCK ──[Schottky]── +5V ──> dev-board 5V pin (standalone power)
 dev-board onboard LDO ── +3V3 ──> TMC VIO, opto pull-ups, MS address straps
```

The ESP32-S3 talks to the host (skirnir) over **USB-C**, so the dev board is normally USB-powered; the buck +
anti-backfeed Schottky let the board also run standalone from the 12 V jack. **Do not power from USB and the jack
without the Schottky in place.** 3.3 V comes from the dev board's onboard LDO (limit load ≈ 50 mA: TMC VIO + opto
pull-ups + straps — comfortably within its ~1 A headroom). See `research/esp32s3-carrier-power.md`.

## Connectors / headers summary

| Ref | Part | Footprint | Purpose |
|---|---|---|---|
| J_PWR | 5.5×2.5 barrel jack | `Connector_BarrelJack:BarrelJack_CUI_PJ-102AH_Horizontal` | 12 V in |
| J_DKL / J_DKR | 1×22 female socket ×2 | `Connector_PinSocket_2.54mm:PinSocket_1x22_P2.54mm_Vertical` | dev-board socket |
| U_X/Y/Z/A | BTT TMC2209 V1.3 | `Galdr:TMC2209_StepStick_Socket` | driver sockets |
| J_MOT_X/Y/Z/A | 4-pin screw terminal | `TerminalBlock_Phoenix:..._MKDS-1-4-3.81_1x04_P3.81mm_Horizontal` | motor coils |
| J_LIM_X/Y/Z, J_PROBE | JST-XH 3-pin | `Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical` | opto limit/probe |
| J_SPIN | 1×6 JST-XH | `Connector_JST:JST_XH_B6B-XH-A_1x06_P2.50mm_Vertical` | carrier→conditioning link: SPIN_PWM/EN/DIR/**+12V**/GND/GND (DOC-07) |
| J_AUX | 1×4 header | `…PinHeader_1x04…` | FHOLD/CYCSTART/3V3/GND |
| J_DIAG | 1×4 header | `…PinHeader_1x04…` | X/Y/Z/A DIAG (sensorless-homing future) |
| U_BUCK | MP1584 buck module | `…PinHeader_1x04…` (placeholder) | 12 V→5 V |

## Spindle conditioning daughterboard (page 2, `galdr-spindle-cond.kicad_sch`)

The WS55-220 has no logic-PWM speed input — its **SV** terminal wants 0–10 VDC analog (DOC-07). Rather than put the
analog stage on the carrier, it lives on a small **separate daughterboard** that sits between the carrier and the
spindle driver. It is drawn as page 2 of the same KiCad project (its own sheet/file in the hierarchy) but is a
distinct physical PCB.

**Inter-board connection.** A 6-conductor cable joins the carrier's `J_SPIN` (1×6 JST-XH) to the daughterboard's
`J_CIN` (1×6 JST-XH): `SPIN_PWM`, `SPIN_EN`, `SPIN_DIR`, **`+12V`**, `GND`, `GND`. The carrier feeds its +12 V rail
across so the op-amp has a supply that can swing to 10 V; JST-XH is keyed to prevent reversed insertion.

**Signal chain.**
- **PWM → DC:** two-stage RC low-pass (`R_LP1`/`C_LP1`, `R_LP2`/`C_LP2`, 10 kΩ/1 µF, fc ≈ 16 Hz) recovers the DC
  average of the GPIO13 LEDC PWM (1–20 kHz carrier).
- **Gain:** non-inverting amp `U_OA` (**LMC6482IN**, DIP-8 dual rail-to-rail I/O, 3–15.5 V) scales 3.3 V → 0–10 V.
  Channel A is the amplifier; channel B is parked as a grounded unity follower (IN+B→GND, OUTB→IN-B). Gain =
  1 + (`R_FB` 15 kΩ + `RV_CAL` 0–10 kΩ)/(`R_G` 10 kΩ) ≈ 2.5–3.5; the `RV_CAL` trimmer calibrates to exactly 10.0 V
  at full PWM duty. Output goes through `R_OS` 100 Ω + `C_OS` 100 nF to the `J_WS` **SV** terminal.
- **EN / DIR:** relayed to the WS55-220 through open-drain N-MOSFETs (`Q_EN`/`Q_DIR`, 2N7000 TO-92, gate series +
  pulldown).
  Gate HIGH → MOSFET on → the driver terminal is pulled to GND (the WS55-220 "to-GND = active" convention). Output on
  the `J_WS` 4-pin screw terminal: **SV / EN / DIR / GND**.

The page-2 netlist is ERC-clean and the inter-board nets join across both sheets (verified with
`kicad-cli sch erc` + `sch export netlist`).

## Open items / verify before fabrication

1. **Row spacing of the dev-board sockets** — measure the real board.
2. **MOSFET pin/pad order** — `Device:Q_PMOS`/`Q_NMOS` are generic; confirm the TO-220/TO-92 pad map matches the
   chosen parts (carrier rev-prot `Q1` = FQP27P06 TO-220; daughterboard `Q_EN`/`Q_DIR` = 2N7000 TO-92) at layout time.
3. The MP1584 buck is represented as a 1×4 header placeholder — confirm its pin order (IN+/IN−/OUT+/OUT−) against the
   module you buy.
4. Opto field rail is the shared +12 V (functional, not galvanic, isolation). For true galvanic isolation feed the
   JST V+ from a separate supply.
5. **WS55-220 EN/DIR input interface** — confirm against the driver datasheet that EN/DIR are open-collector "to-GND"
   inputs (the open-drain stage assumes this). Confirm the internal pull-up voltage is within the 2N7000 V_DS rating.
6. **Firmware `SPIN_EN`/`SPIN_DIR` polarity** — the open-drain stage inverts: firmware must drive the line **HIGH to
   assert** (run / direction-B), since gate-HIGH pulls the driver terminal to GND. DOC-07's parenthetical ("pull
   GPIO14 low → EN to GND") describes a *direct* connection; reconcile the firmware polarity with this board.
7. **Op-amp supply headroom / calibration** — verify the LMC6482IN reaches 10 V on the +12 V rail under the SV load
   (RRIO, 15.5 V max — comfortable), and trim `RV_CAL` to 10.0 V at full duty during bring-up. (LMC6482IN replaced
   the MCP6H02 because the MCP6H02 PDIP-8 is not DigiKey-stocked; the two are pin-identical, so the footprint and
   netlist are unchanged.)
