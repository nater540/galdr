> ⚠️ **SUPERSEDED / WRONG SCOPE.** This was a first-pass research detour that designed a *bare TMC2209-LA QFN
> chip* stage. The actual board **sockets BigTreeTech TMC2209 V1.3 stepstick modules** — all the support circuitry
> below (sense resistors, charge pump, decoupling) lives on the BTT module, not the carrier. See `../DESIGN.md` for
> the real design. Kept only because its UART-bus, MS1/MS2 addressing, and connector notes still apply.

# Bare TMC2209 (QFN-28) Carrier-Board Reference Design — Galdr

Concrete reference design for driving 3–4 NEMA17 stepper motors with **bare TMC2209-LA chips**
(QFN-28, NOT Adafruit/BTT plug-in modules) from the Galdr ESP32-S3 firmware. All support-component
values, the per-driver netlist, UART-bus wiring, address strapping, and a full BOM are below.

Sources cross-checked:
- **BTT TMC2209 V1.3 User Manual** — `docs/hardware-references/BTT-TMC2209-V1.3.pdf` (the gold-standard
  *working module*; it is a user manual, not a netlist, but the silkscreen and photos confirm key values).
- **Adafruit TMC2209 KiCad 10 schematic** — `docs/hardware-references/Adafruit-TMC2209.kicad_sch`
  (full schematic with every passive value; cited by line number).
- **TMC2209 datasheet rev 1.09** (Analog Devices/Trinamic) — typical-application circuit + IRMS formula.

> **Important reconciliation up front:** The Adafruit *schematic file in this repo* uses **0.05 Ω** sense
> resistors (R1/R2 at lines 8990 / 10540, value `0.05Ω/0.25W`). That is the *old Adafruit breakout* value
> and we are explicitly **NOT** copying it. The **BTT V1.3 module silkscreen reads "R110" / "R110"**
> (PDF p.6/p.7 photos) = **0.110 Ω**, which is the correct value for ~1.2–1.7 A RMS NEMA17. **We adopt
> 0.110 Ω**, matching the BTT reference and the firmware's stated ~0.11 Ω target. The Adafruit file is used
> only for the QFN symbol pinout and the decoupling-cap topology, not its sense-resistor value.

---

## 0. Symbol recommendation

There are two QFN-28 symbols available:

| Symbol | Source | Pin-name style | Notes |
|---|---|---|---|
| `Driver_Motor:TMC2209-LA` | **KiCad stock** (`Driver_Motor.kicad_sym` line 38349) | `~{EN}`, `MS1/AD0`, `MS2/AD1`, `~{PD}/UART`, `VCC_IO`, `5VOUT`, `CPO/CPI/VCP`, `OA1/OA2/OB1/OB2`, `BRA/BRB`, `VS`, `VREF`, `SPREAD`, `STDBY`, `DIAG`, `INDEX`, `CLK` | Footprint `Package_DFN_QFN:VQFN-28-1EP_5x5mm_P0.5mm_EP3.7x3.7mm_ThermalVias`. Datasheet rev1.09 linked. **Has the exposed-pad EP as pin 29 (GND).** |
| `TMC2209_QFN` | Adafruit lib (`Adafruit-TMC2209.kicad_sch` line 4183, eagle-import) | `ENABLE`, `MS1`, `MS2`, `PDN/UART`, `VCCIO`, `OUTA1/OUTA2/OUTB1/OUTB2`, GND as pin "THERMAL" | Footprint `QFN28_5MM_MICROCHIP`. Eagle-imported, generic GND naming. |

**Recommendation: use the stock `Driver_Motor:TMC2209-LA`.** It is the maintained KiCad 10 part, carries
the correct ADI datasheet link, exposes the EP as an explicit GND pin (29) so the thermal pad nets cleanly,
and pairs with the standard `VQFN-28-1EP_5x5mm_P0.5mm_EP3.7x3.7mm_ThermalVias` footprint. The Adafruit
symbol is fine as a cross-reference but its eagle-imported `OUTA1`/`THERMAL` naming is non-standard.

Both symbols agree pin-for-pin on numbers/functions (verified below) — only the *names* differ.

---

## 1. QFN-28 pinout table

Pin numbers verified identical across the stock symbol, the Adafruit symbol, and datasheet rev1.09.
"Galdr connection" = what each pin ties to in this carrier design.

| # | Stock name (`TMC2209-LA`) | Adafruit name | Function | **Galdr connection** |
|---|---|---|---|---|
| 1 | OB2 | OUTB2 | Motor coil B, output 2 | Stepper connector pin B2 |
| 2 | ~{EN} (EN) | ENABLE | Enable, **active-low** | **Common ENN** net → GPIO8 (one net, all drivers) |
| 3 | GND | GND | Digital/analog ground | GND plane |
| 4 | CPO | CPO | Charge-pump output | VCP cap to CPI (C_VCP), and 100 nF to VS |
| 5 | CPI | CPI | Charge-pump input | VCP cap to CPO (C_VCP) |
| 6 | VCP | VCP | Charge-pump reservoir | 100 nF (C_CP) to VS |
| 7 | SPREAD | SPREAD | StealthChop/SpreadCycle select (static) | Tie **GND** (StealthChop default; firmware can switch chopper over UART) |
| 8 | 5VOUT | 5VOUT | Internal 5 V LDO output | 4.7 µF ceramic (C_5V) to GND. **Do not load externally.** |
| 9 | MS1/AD0 | MS1 | UART node address bit 0 (in UART mode) | **Address strap** (10 kΩ to GND or VCC_IO) — see §4 |
| 10 | MS2/AD1 | MS2 | UART node address bit 1 (in UART mode) | **Address strap** (10 kΩ to GND or VCC_IO) — see §4 |
| 11 | DIAG | DIAG | Diagnostic / StallGuard output (push-pull, active-high) | **Break out to header / MCU** — see §5 (sensorless homing) |
| 12 | INDEX | INDEX | Microstep index / configurable output | Optional header; default **NC** — see §5 |
| 13 | CLK | CLK_IN | External clock in | **Tie GND** to select the internal 12 MHz oscillator |
| 14 | ~{PD}/UART (PDN_UART) | PDN/UART | UART data (half-duplex single-wire) + powerdown | **Shared UART bus** via 1 kΩ node resistor — see §3 |
| 15 | VCC_IO | VCCIO | Logic I/O supply | **3.3 V** (from ESP32-S3 devkit) + 100 nF decoupling (C_IO) |
| 16 | STEP | STEP | Step input (rising edge) | Per-axis STEP: X=GPIO1, Y=GPIO2, Z=GPIO4, A=GPIO18 |
| 17 | VREF | VREF | Current-scale reference | **Tie to 5VOUT** (full-scale ref); current set by UART IRUN/IHOLD — see §2 |
| 18 | GND | GND | Ground | GND plane |
| 19 | DIR | DIR | Direction input | Per-axis DIR: X=GPIO5, Y=GPIO6, Z=GPIO7, A=spare |
| 20 | STDBY | STDBY | Standby input | **Tie GND** (never standby; firmware uses ENN) |
| 21 | OA2 | OUTA2 | Motor coil A, output 2 | Stepper connector pin A2 |
| 22 | VS | VS | Motor/driver supply (12 V VM) | **VM** + bulk + ceramic decoupling — see §2 |
| 23 | BRA | BRA | Sense-resistor return, coil A | Sense resistor RS_A to GND |
| 24 | OA1 | OUTA1 | Motor coil A, output 1 | Stepper connector pin A1 |
| 25 | GND (NC on some) | GND | Ground | GND plane |
| 26 | OB1 | OUTB1 | Motor coil B, output 1 | Stepper connector pin B1 |
| 27 | BRB | BRB | Sense-resistor return, coil B | Sense resistor RS_B to GND |
| 28 | VS | VS | Motor/driver supply (12 V VM) | **VM** (second VS pin — bypass both) |
| **29 (EP)** | GND (exposed pad) | THERMAL | Exposed thermal pad | **GND plane via thermal via array** — see §8 |

> Note: pin 25 — the Adafruit symbol shows it as a second GND, the stock `TMC2209-LA` shows pin 25 as GND
> too (datasheet lists it GND). Tie to ground. (The stock symbol's pin 1 is OB2 and ascends in the same
> order as the table; confirmed via the symbol dump.)

---

## 2. Per-driver support circuit (the netlist, repeated ×3 or ×4)

Reference-designator scheme: `<part><axis-index>`, e.g. driver X = `U1`, its bulk cap `C1X`, sense
`RS1X`/`RS2X`; driver Y = `U2`/`C1Y`/…; Z = `U3`; A = `U4`. Below shows **one** driver's network.

### Power / decoupling (per driver)

| Ref | Value | Voltage | Connects | Purpose | Justification |
|---|---|---|---|---|---|
| C_BULK | **22 µF** electrolytic/MLCC | **50 V** | VS (pin 22 & 28) → GND | VM bulk reservoir | Adafruit C4 = `22uF/50V` (line 6658); BTT photo shows a `50V` electrolytic (PDF p.6). 50 V part on a 12 V rail = healthy derating. |
| C_VS1 | **100 nF** (0.1 µF) MLCC X7R | 50 V | VS (pin 22) → GND | HF bypass, coil A side | Adafruit uses 0.22 µF class (C3/C7/C8, lines 10857/8752/8671); datasheet typ-app uses 100 nF. 100 nF X7R close to pin 22. |
| C_VS2 | **100 nF** (0.1 µF) MLCC X7R | 50 V | VS (pin 28) → GND | HF bypass, coil B side | One per VS pin. Datasheet typical-application recommendation. |
| C_5V | **4.7 µF** MLCC X7R | 16 V | 5VOUT (pin 8) → GND | Internal 5 V LDO bypass | Datasheet rev1.09 specifies **2.2–4.7 µF** ceramic at 5VOUT; Adafruit C2/C6 = `10uF` (lines 10701/7221) on this rail. Use 4.7 µF (10 µF acceptable). **Do not draw external load from 5VOUT.** |
| C_CP | **100 nF** (0.1 µF) MLCC | 16 V | VCP (pin 6) → VS | Charge-pump reservoir (CPO↔VS) | Datasheet: VCP→VS = 100 nF. |
| C_VCP | **22 nF** MLCC X7R | **50 V** | CPI (pin 5) ↔ CPO (pin 4) | **Charge-pump flying cap** | Datasheet: 22 nF/50 V between CPO and CPI. Adafruit C1 = `22nF/50V` (line 9390). **Must be 50 V** — sits across the doubled supply. |
| C_IO | **100 nF** (0.1 µF) MLCC | 16 V | VCC_IO (pin 15) → GND | Logic-supply decoupling | Adafruit C5 = `0.22uF/50V` (line 8244) on VCC_IO; 100 nF X7R is standard. |

### Current sense (per driver)

| Ref | Value | Rating | Connects | Purpose |
|---|---|---|---|---|
| RS_A | **0.110 Ω** | **0.5 W**, 1 % | BRA (pin 23) → GND | Coil A current sense |
| RS_B | **0.110 Ω** | **0.5 W**, 1 % | BRB (pin 27) → GND | Coil B current sense |

**RSENSE choice & IRMS formula (datasheet rev1.09, confirmed):**

```
            V_FS         1      V_VREF
I_RMS  =  ---------  ×  ----  × --------
          RS + 20mΩ      √2      2.5 V
```
where V_FS ≈ 325 mV (full scale, after the internal 20 mΩ on-chip resistance term), and we tie
**VREF → 5VOUT** so V_VREF/2.5 V = full-scale and the actual run current is set digitally by
`IRUN`/`IHOLD` over UART (firmware drives CHOPCONF/IHOLD_IRUN, not the analog ref).

With **RS = 0.110 Ω, VREF = 2.5 V (full scale)**:
```
I_RMS(max) = 0.325 / (0.110 + 0.020) × 0.707 = 2.5 × 0.707 ≈ 1.77 A RMS  (≈ 2.5 A peak)
```
So 0.110 Ω gives a **1.77 A RMS full-scale ceiling**, and our **1.2 A RMS** working point sits at
IRUN ≈ 1.2/1.77 ≈ 68 % of full scale — a comfortable mid-scale code with good resolution and headroom.
This matches the BTT "R110" silkscreen exactly. (The Adafruit-file 0.05 Ω would push full-scale to
~3.4 A RMS — too coarse and hot for a 1.2 A NEMA17; rejected as noted.)

Power in each sense resistor at 1.2 A RMS: P = I²·R = 1.2² × 0.11 ≈ 0.16 W → **use 0.5 W (1206) or
two 0.22 Ω/0.25 W in parallel.** A single 0.110 Ω 1 % 1206 0.5 W is cleanest.

### Static-strap pins (per driver)

| Pin | Tie to | Via | Reason |
|---|---|---|---|
| SPREAD (7) | GND | direct | StealthChop default; chopper mode overridden over UART. |
| CLK (13) | GND | direct | Selects internal 12 MHz oscillator. |
| STDBY (20) | GND | direct | Disable standby; enable is handled by ENN. |
| VREF (17) | 5VOUT (8) | direct (or 0 Ω) | Full-scale analog ref; digital current scaling via UART. |

---

## 3. Shared UART bus wiring

The TMC2209 PDN_UART (pin 14) is a **half-duplex single-wire** line. All 3–4 drivers share one bus and one
ESP32-S3 UART (UART1, GPIO9 doing both TX and RX):

```
  ESP32-S3 GPIO9 (UART1 TX/RX) ──[ R_MCU 1 kΩ ]──┬── PDN_UART (U1, X, addr0)
                                                  ├── PDN_UART (U2, Y, addr1)
                                                  ├── PDN_UART (U3, Z, addr2)
                                                  └── PDN_UART (U4, A, addr3)
```

- **One single 1 kΩ series resistor (`R_MCU`)** sits on the MCU side of the bus (as the firmware contract
  specifies). It limits current during the brief windows when the MCU and a driver both drive the line, and
  isolates the ESP32 pin. Place it at the MCU.
- **All four PDN_UART pins join one common node** downstream of R_MCU.
- The TMC2209 has an internal RX/TX bridge resistor inside PDN_UART, so the single external 1 kΩ on the host
  side is sufficient for a single-wire bus — **no per-driver series resistor is required** on the chip side.
  (The BTT module exposes separate RX/TX pads bridged by an on-module resistor "R10"/jumper — PDF p.6/p.7 —
  purely so the *module* can be wired to boards that route RX and TX separately. Bare chips have only PDN_UART,
  so that bridge is internal and we skip it.)
- Optional: a single **20 kΩ pull-up** from the common UART node to VCC_IO (3.3 V) keeps the idle line high
  and defined. The Adafruit "20K Pack" R5 net (lines 6403/7458/8410/9471) and R3 `20K` (line 10226) are
  this idle-bias network. One 20 kΩ to 3.3 V on the shared node is enough.

Addressing on this shared bus is done by MS1/MS2 strapping (§4); the firmware then talks to each driver by
its node address 0–3.

---

## 4. MS1/MS2 UART-address strapping (per driver)

In UART mode MS1 (pin 9) and MS2 (pin 10) latch the **node address** at power-up (the chip has weak internal
pulldowns, so a pin left floating reads 0 — but we strap explicitly per the firmware contract, since address
collisions on a shared bus are fatal). Address = (MS2 << 1) | MS1.

| Driver | Axis | Addr | MS1 | MS2 | Strap MS1 | Strap MS2 |
|---|---|---|---|---|---|---|
| U1 | X | 0 | 0 | 0 | 10 kΩ → GND | 10 kΩ → GND |
| U2 | Y | 1 | 1 | 0 | 10 kΩ → VCC_IO (3.3 V) | 10 kΩ → GND |
| U3 | Z | 2 | 0 | 1 | 10 kΩ → GND | 10 kΩ → VCC_IO (3.3 V) |
| U4 | A/spare | 3 | 1 | 1 | 10 kΩ → VCC_IO (3.3 V) | 10 kΩ → VCC_IO (3.3 V) |

- Use **10 kΩ** for every strap (`R_AD0`, `R_AD1` per driver). 10 kΩ overrides the chip's internal pulldown
  for a solid logic-1 when pulled to 3.3 V, and reinforces logic-0 when pulled to GND.
- Pull HIGH straps go to **VCC_IO (3.3 V)**, never to VM or 5VOUT — MS1/MS2 are VCC_IO-domain logic inputs.
- These are hard straps (resistor to a rail), **not** MCU-driven and **not** microstep selects. Microstepping
  is set via CHOPCONF.MRES over UART, exactly as the firmware does.

---

## 5. DIAG / INDEX pins

- **DIAG (pin 11): break out to the MCU.** DIAG is the StallGuard / diagnostic output (push-pull, active-high
  on stall or error). The firmware notes DIAG polling for **sensorless homing** as a future option, so route
  each driver's DIAG to either (a) a dedicated MCU GPIO if pins are available, or (b) at minimum a per-driver
  **2-pin header / test point**. Since there are up to 4 drivers and GPIO is tight, a clean compromise is:
  route **X/Y/Z DIAG to a 0.1" header block** (3–4 pins + GND) so sensorless homing can be wired/jumped to
  spare GPIOs later without a board respin. DIAG is 3.3 V-domain (VCC_IO referenced) — safe to the ESP32.
  - If you DO wire DIAG straight to a GPIO now, add a **1 kΩ series** resistor and rely on the ESP32 internal
    pulldown; no level shifting needed (VCC_IO = 3.3 V).
- **INDEX (pin 12): leave NC, but expose a per-driver test point/pad.** INDEX (microstep position / configurable
  output) is not used by the current firmware. A small test pad per driver costs nothing and helps bring-up.

---

## 6. Stepper motor output connector

Per driver, coil order is **OA1 / OA2 / OB1 / OB2** (pins 24 / 21 / 26 / 1). Phase A = OA1/OA2, phase B =
OB1/OB2.

**Recommended connector: 4-pin 3.81 mm pluggable screw terminal** (Phoenix-style MC 1,5/4-G-3.81 or generic
"KF2EDGK 3.81mm 4P"), one per axis. Rationale: NEMA17 coils at 1.2 A RMS are well within a 3.81 mm terminal's
rating (~8 A/contact), it's field-serviceable for swapping motors, and PCB-pluggable headers let you unplug a
motor without tools.

- Pin order on the terminal (left→right): **A1, A2, B1, B2** (= OA1, OA2, OB1, OB2).
- Alternative if you want keyed locking connectors: **JST-VH (3.96 mm, 4-pin)** — rated ~10 A, polarized,
  good for 12 V/1.5 A motors. JST-XH (2.5 mm) is rated only ~3 A and is marginal at 1.2 A continuous with
  inrush — acceptable but the screw terminal / VH is the safer pick.
- Silk-label each terminal `A1 A2 B1 B2` and note the coil pairing, since NEMA17 wire colors vary by vendor.

---

## 7. Total BOM — stepper stage (per driver × 4)

| Ref (per drv) | Qty/drv | ×4 total | Value | Rating / package | Notes |
|---|---|---|---|---|---|
| U (TMC2209-LA) | 1 | 4 | TMC2209-LA | VQFN-28 5×5 mm EP | Driver IC |
| C_BULK | 1 | 4 | 22 µF | 50 V, MLCC 1210 or electrolytic | VM bulk (see §8 re: sharing) |
| C_VS1, C_VS2 | 2 | 8 | 100 nF | 50 V, X7R 0603 | VS HF bypass (one per VS pin) |
| C_5V | 1 | 4 | 4.7 µF | 16 V, X7R 0805 | 5VOUT (2.2–4.7 µF datasheet) |
| C_CP | 1 | 4 | 100 nF | 16 V, X7R 0603 | VCP→VS reservoir |
| C_VCP | 1 | 4 | 22 nF | **50 V**, X7R 0603 | CPO↔CPI flying cap |
| C_IO | 1 | 4 | 100 nF | 16 V, X7R 0603 | VCC_IO decoupling |
| RS_A, RS_B | 2 | 8 | 0.110 Ω | 1 %, 0.5 W, 1206 | Coil sense |
| R_AD0, R_AD1 | 2 | 8 | 10 kΩ | 1 %, 0603 | MS1/MS2 address straps |
| J (motor) | 1 | 4 | 4-pin terminal | 3.81 mm pluggable | OA1/OA2/OB1/OB2 |
| (DIAG series, opt.) | 1 | 4 | 1 kΩ | 0603 | only if DIAG → GPIO direct |

**Shared (whole stage, not per driver):**

| Ref | Qty | Value | Notes |
|---|---|---|---|
| R_MCU | 1 | 1 kΩ | UART series resistor at the ESP32 GPIO9 |
| R_UART_PU | 1 | 20 kΩ | optional UART-node idle pull-up to 3.3 V |
| C_VM_TANK | 1–2 | 470–1000 µF | 25–35 V electrolytic — board-level VM tank (see §8) |

Per-driver part-class count: **1 IC, ~7 caps, 4 resistors, 1 connector.** This matches the BTT/Adafruit
working modules' parts census (the only delta is our 0.110 Ω sense vs. the Adafruit *file's* 0.05 Ω).

---

## 8. Thermal & bulk-cap sizing notes

**Exposed pad:** The QFN-28 EP (pin 29) is the primary heat path AND the device ground. Connect it to the GND
plane through a **thermal via array** (the stock footprint `…_EP3.7x3.7mm_ThermalVias` already lays these out
— a 4×4 / 0.3 mm via grid into a solid bottom-side copper pour). The BTT module's gold center pad (PDF p.5
photo) is exactly this. Provide as much bottom-side copper pour as the layout allows; at 1.2 A RMS the
TMC2209 (Rds(on) ≈ 340 mΩ H+L per the BTT spec, PDF p.4) dissipates on the order of P ≈ 2·I²·Rds(on) ≈
2 × 1.2² × 0.17 ≈ 0.5 W per chip — the BTT manual itself flags **active cooling above ~1.2 A** (PDF p.8,
Safety Precautions). Plan for a small heatsink on the EP-coupled top pour or a fan if all 4 axes run hot
simultaneously.

**Bulk-cap sizing under total current:** Four drivers at ~1.2 A RMS each is **~5 A** of aggregate motor
current drawn from the 12 V VM rail. The **per-driver 22 µF/50 V** caps are local HF/ripple reservoirs and
stay as-is — they handle each chopper's switching ripple, which is per-chip and does not add across drivers.
What *does* scale with total current is the **board-level VM tank**: add a shared **470–1000 µF, 25–35 V
electrolytic** (`C_VM_TANK`) at the 12 V input where it enters the board, to absorb the combined low-frequency
current swings and supply inrush, plus a 0.1 µF + 10 µF ceramic pair at the input for HF. This keeps the VM
rail stiff when several axes accelerate together. (BTT/Adafruit modules each carry only their local 22 µF
because each module assumes the host board provides this shared tank — on our carrier we must add it.) Size
the 12 V supply and its input trace/plane for ≥6–7 A to cover the ~5 A motor current plus margin.

---

## Summary of the key numbers (quick reference)

- **Sense resistor: 0.110 Ω, 0.5 W, 1 %** (matches BTT "R110"; rejects the Adafruit-file 0.05 Ω).
- **VCP flying cap: 22 nF / 50 V** between CPO(4) and CPI(5).
- **VCP→VS reservoir: 100 nF.**
- **5VOUT cap: 4.7 µF** (datasheet 2.2–4.7 µF range).
- **VM bulk per driver: 22 µF / 50 V** + 100 nF per VS pin; **shared 470–1000 µF tank** at the 12 V input.
- **VCC_IO: 3.3 V + 100 nF.**
- **UART: single 1 kΩ at the MCU**, all PDN_UART joined, optional 20 kΩ idle pull-up to 3.3 V.
- **Address straps: 10 kΩ** per MS1/MS2 to GND or 3.3 V, giving addr 0/1/2/3 for X/Y/Z/A.
- **VREF → 5VOUT; SPREAD/CLK/STDBY → GND.**
- **Symbol: `Driver_Motor:TMC2209-LA`** with `VQFN-28-1EP_5x5mm_P0.5mm_EP3.7x3.7mm_ThermalVias`.
- **DIAG broken out to a header** (future sensorless homing); INDEX = NC + test pad.
- **Motor connector: 4-pin 3.81 mm pluggable screw terminal**, order OA1/OA2/OB1/OB2.

### Sources
- BTT TMC2209 V1.3 User Manual — `docs/hardware-references/BTT-TMC2209-V1.3.pdf` (pp. 4–8: specs, pin map, R110 silk, 50 V cap, cooling note).
- Adafruit TMC2209 KiCad schematic — `docs/hardware-references/Adafruit-TMC2209.kicad_sch` (component values cited by line).
- KiCad stock symbol — `/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols/Driver_Motor.kicad_sym` (`TMC2209-LA`, line 38349).
- [TMC2209 datasheet rev1.09 — Analog Devices](https://www.analog.com/media/en/technical-documentation/data-sheets/tmc2209_datasheet_rev1.09.pdf)
- [TMC2209 UART RMS current calculation — OpenAstroTech wiki](https://wiki.openastrotech.com/Knowledge/UART_RMS_Calculation)
- [BIGTREETECH TMC2209 wiki](https://global.bttwiki.com/TMC2209.html)
