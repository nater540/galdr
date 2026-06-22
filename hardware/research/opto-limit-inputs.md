# Opto-isolated limit-switch & probe input stage (Galdr carrier board)

ESP32-S3 + TMC2209 desktop CNC carrier. This document specifies the opto-isolated, fail-safe
limit-switch input stage and the field connectors, with exact R/C/D values, the proven
polarity truth table, KiCad symbols/footprints, a netlist for one channel, and a BOM.

## 0. Firmware contract this design MUST satisfy

The firmware (DOC-06) treats limit switches as **Normally-Closed (NC), fail-safe**:

| Field condition          | Required GPIO level | Meaning                         |
|--------------------------|---------------------|---------------------------------|
| Switch INTACT, NOT triggered | **LOW**         | normal running                  |
| Switch TRIGGERED (opens) | **HIGH**            | rising-edge IRQ = limit hit     |
| Wire BROKEN / unplugged  | **HIGH**            | fail-safe trips (treated as hit)|

Pins: `X_LIM=GPIO10`, `Y_LIM=GPIO11`, `Z_LIM=GPIO12`, `PROBE=GPIO21`, all 3.3 V.
Firmware enables the **internal pull-up** and a **rising-edge interrupt** on each pin.
A `$5` invert mask exists but we deliberately design so it can stay `0` — **intact NC = LOW
at the GPIO with no inversion**.

The single most important consequence: the **internal pull-up makes the GPIO idle HIGH**.
Therefore the opto's phototransistor must **actively pull the GPIO LOW while the switch is
intact/closed**. That means the opto **LED must be ON when the switch is INTACT/CLOSED**, and
OFF when the switch is triggered (opens) OR when the wire breaks. LED-off is the fail-safe
state, and it maps to GPIO HIGH automatically. This is the whole trick — get it backwards and
the board reports "limit hit" during normal running.

---

## 1. Chosen optocoupler

**Part: PC817 (single-channel transistor-output optocoupler), DIP-4 through-hole.**
Equivalents accepted as drop-ins on the same footprint/symbol: LTV-817, EL817, FOD817.

Why PC817 (single) over a quad TLP281-4:
- Per-channel galvanic isolation with **independent field grounds** — each axis can be on its
  own twisted pair without sharing a return, which matters for noise immunity in a mill.
- Through-hole DIP-4 is trivial to hand-place/rework and to socket; the board only needs 3–4
  of them.
- Universally stocked, ~$0.10 ea, and KiCad ships the symbol and footprint.
- A quad (TLP281-4) saves board area but ties the four LED cathodes / collector grounds into
  shared rails, partially defeating per-channel isolation and complicating the NPN-sensor case.
  Use a quad only if board area is critical; the per-channel PC817 is the recommended default.

CTR (current transfer ratio): PC817 base rank "A" is **80–160 %** at If = 5 mA; we design at
If ≈ 13–15 mA and the **worst-case CTR floor of 50 %** (aged, hot, low-rank) to guarantee the
phototransistor can sink enough current to make a valid logic LOW. See §4 for the margin check.

KiCad: symbol `Isolator:PC817`, footprint `Package_DIP:DIP-4_W7.62mm` (TH) or
`Package_SO:SOP-4_3.8x4.1mm_P2.54mm` (SMD EL817S). PC817 pinout:
**1 = LED anode, 2 = LED cathode, 3 = transistor emitter, 4 = transistor collector.**

---

## 2. Field supply rail decision

**V+ pin on every limit/probe connector = +12 V.**

Rationale: 3-wire inductive proximity sensors (the anticipated NPN-NC upgrade) are 6–36 V
devices; 12 V is squarely in range and is a rail the carrier board already has. A plain dry
mechanical NC switch does **not** need 12 V to function, but exposing 12 V on the V+ pin lets
the *same connector and the same opto stage* serve both a dry switch and an inductive sensor
with no jumpers. The opto LED resistor is sized for the 12 V loop (§3), and because the opto
LED's forward drop dominates, the *mechanical* case and the *inductive* case both land in the
same If window. The 12 V field domain is isolated from the clean 3.3 V MCU domain by the opto —
which is the entire point of opto-isolation here.

> Important wiring rule that makes 12 V work cleanly for BOTH cases: the **switch (or sensor
> output) is placed in series with the opto LED, and the LED current is sourced from +12 V**.
> The switch does NOT directly short 12 V to GND. A closed/intact NC switch *completes the LED
> loop*; an open/triggered switch *breaks the LED loop*. This keeps "intact = LED ON = GPIO LOW"
> and never dumps 12 V into a bare contact. (This resolves the "12 V makes the dry-switch case
> awkward" concern in the brief — it is not awkward as long as the LED, not the rail, is what
> the switch gates.)

---

## 3. One-axis input channel — schematic in words + ASCII netlist

Field (isolated 12 V domain) on the left, MCU (clean 3.3 V domain) on the right. The two
domains share **no copper** across the PC817.

```
        +12V_FIELD ──┬───────────────[ pin1 V+ ]  (JST-XH pin 1)
                     │
                    R1  680Ω 1/2W           <-- LED current-limit (12V loop)
                     │
            D1 (1N4148, anti-parallel ─┐    <-- reverse-protection across LED
              across LED A–K)          │
                     │                 │
   PC817 pin1 (LED A) ●───────────────┘
                     │ ▼  (opto LED)
   PC817 pin2 (LED K) ●
                     │
                     ●───────────────[ pin2 SIGNAL ]  (JST-XH pin 2)
                                          │
                                    [ NC SWITCH or NPN-NC sensor output ]
                                          │
        GND_FIELD ───────────────────[ pin3 GND ]   (JST-XH pin 3)

   --------------------------- galvanic isolation barrier (PC817) ---------------------------

        +3V3 ──── R2 10kΩ ──┬──────────────────────────●  GPIO (X/Y/Z_LIM)
                            │                          │
   PC817 pin4 (collector) ──┘                          │
                            │                       R3 1kΩ (series)
   PC817 pin3 (emitter) ─── GND_MCU                    │
                                                    C1 100nF ── GND_MCU
                                                  (RC debounce / EMI filter)
```

Equivalently, node-by-node:

```
NET +12V_FIELD : J1.1 , R1.a
NET LED_DRIVE  : R1.b , U1.1 (LED anode) , D1.cathode
NET LED_RTN    : U1.2 (LED cathode) , D1.anode , J1.2 (SIGNAL)
NET GND_FIELD  : J1.3              ; field return; switch/sensor sinks to here
NET +3V3       : R2.a
NET OPTO_COL   : U1.4 (collector) , R2.b , R3.a   ; pulled up to 3V3, opto pulls it down
NET GPIO_LIM   : R3.b , C1.a                       ; goes to ESP32-S3 GPIO10/11/12
NET GND_MCU    : U1.3 (emitter) , C1.b
```

### Wiring of the two field-device types into the 3-pin connector

- **Dry mechanical NC switch (2-wire):** wire the switch between **pin 2 (SIGNAL)** and
  **pin 3 (GND)**. Leave **pin 1 (V+) unused**. The LED loop is then
  `+12V → R1 → LED → pin2 → switch(closed) → pin3/GND_FIELD`. Closed/intact switch = LED ON.
  (The LED's own forward drop sets the current; 12 V on pin 1 is harmless because nothing
  connects pin 1 to the switch.)
- **NPN-NC inductive sensor (3-wire, sourcing 6–36 V device):** brown→**pin1 (+12 V)**,
  blue→**pin3 (GND)**, black(output)→**pin2 (SIGNAL)**. An **NPN-NC** sensor's open-collector
  output **sinks SIGNAL to GND while NOT triggered** (that is the NC behavior), completing the
  LED loop exactly like a closed dry switch. When triggered, the NPN turns off → LED loop opens
  → LED OFF → GPIO HIGH. Identical polarity to the dry switch. R3 series + the sensor's own
  open-collector tolerate the 12 V because the LED + R1 limit the current and the sensor only
  pulls SIGNAL toward GND.

> Note on the LED-loop sense for NPN: the LED current still flows **into** SIGNAL (pin 2) from
> R1 and out through the sensor's NPN to GND. The sensor must be rated to sink ≥ ~15 mA, which
> all standard M8/M12 prox sensors are (typ. 100–200 mA). Good.

---

## 4. Component values, derivations, and the logic-level margin check

### R1 — LED current-limit (12 V loop)

```
If_target ≈ 14 mA  (mid of the 10–16 mA window, comfortably above PC817's 5 mA spec point)
V_LED(PC817) ≈ 1.2 V @ ~15 mA
R1 = (12 V − 1.2 V) / 14 mA = 10.8 / 0.014 = 771 Ω  → choose standard 680 Ω
With 680 Ω:  If = (12 − 1.2) / 680 = 15.9 mA   (within the 10–16 mA target, fine)
P(R1) = If² · R1 = (0.0159)² · 680 = 0.172 W  → use a 1/4 W (derate) or 1/2 W resistor.
```

A 12 V supply with ±10 % tolerance gives If ≈ 14.4–17.5 mA — still inside PC817 absolute-max
(50 mA) with huge margin, and inside our design window. Use **R1 = 680 Ω, 1/2 W** (the 1/2 W
gives 3× headroom and runs cool).

> If you would rather run the LED softer to extend its life (CTR degrades with cumulative LED
> hours), **R1 = 820 Ω** gives If ≈ 13.2 mA. The §4 margin check below still passes at 820 Ω.
> 680 Ω is the recommended default; 820 Ω is the "long-life" option.

### R2 — collector pull-up to 3.3 V, and the LOW-level margin proof

This is the check that proves the phototransistor can pull GPIO **low enough** for a valid
logic LOW when the LED is ON.

```
Worst-case CTR floor (aged/hot/low-rank PC817): CTR_min = 50%
Phototransistor collector current available: Ic = CTR_min · If = 0.50 · 15.9 mA = 7.95 mA
Current the 10 kΩ pull-up actually pushes through the transistor at saturation:
    I_pullup = (3.3 − V_OL) / R2 ≈ 3.3 / 10k = 0.33 mA   (worst-case ~0.33 mA)
Required Ic to saturate = 0.33 mA  ;  available Ic = 7.95 mA  → ~24× headroom.
=> The transistor is driven hard into saturation. V_OL ≈ 0.1–0.2 V.
ESP32-S3 V_IL (max LOW input) ≈ 0.25·VDD = 0.825 V.  0.2 V ≪ 0.825 V  → solid valid LOW. PASS.
```

So `R2 = 10 kΩ` to +3.3 V is correct and has ~24× drive margin even at CTR=50 %. When the LED
is OFF (triggered/broken), the transistor is open, R2 pulls OPTO_COL to 3.3 V → with the firmware
internal pull-up also high, GPIO reads a clean HIGH (V_IH min ≈ 0.75·VDD = 2.48 V; we present
3.3 V). PASS.

(The firmware's internal pull-up, ~45 kΩ, is in parallel with R2 = 10 kΩ → ~8.2 kΩ effective.
The transistor must sink only 3.3 V / 8.2 kΩ ≈ 0.40 mA — still ~20× margin. We keep the
external 10 kΩ so the node is defined even before firmware enables its pull-up, and to stiffen
against leakage/EMI.)

### R3 + C1 — RC debounce / EMI filter (MCU side)

```
R3 = 1 kΩ (series, between OPTO_COL and the GPIO)
C1 = 100 nF (to GND_MCU at the GPIO)
τ = R3·C1 = 1k · 100n = 100 µs
```

100 µs filters fast EMI/ESD spikes and ringing on the opto edge, while being **far shorter than
the firmware `$26` software debounce (default tens of ms)** — so the hardware filter never
fights or masks the firmware debounce; it only cleans the edge the firmware then debounces.
The opto's own turn-off time (PC817 t_off ≈ 18 µs at If=16 mA) plus this 100 µs dominates the
edge; both are << `$26`. R3 also limits any ESD current into the GPIO. If the field wiring is
very long/noisy and 100 µs is not enough, go to **R3 = 1 kΩ, C1 = 1 µF → τ = 1 ms** (still well
under `$26`); 100 nF is the default.

> Keep R3 ≤ ~1 kΩ so the 100 µs is set by C1, and so the small input leakage of the GPIO
> (sub-µA) cannot develop a meaningful offset across R3.

### D1 — reverse-protection diode across the opto LED

`D1 = 1N4148` placed **anti-parallel across the PC817 LED (anode-to-cathode of LED reversed)**.
If the field connector is wired backwards (V+ and GND swapped, common in the field), reverse
voltage across the PC817 LED (V_R_max ≈ 6 V) would destroy it. D1 clamps reverse voltage to
~0.7 V and shunts the reverse current. In normal forward operation D1 is reverse-biased and
invisible. (A Schottky like BAT54 clamps tighter at ~0.3 V if you want extra margin; 1N4148 is
fine for a 12 V loop limited by R1.)

> The series R1 already limits reverse current to (12+0.7)/680 ≈ 18.7 mA into D1, which the
> 1N4148 (200 mA) handles easily — so even a sustained reversed connector is non-destructive.

---

## 5. The proven truth table (field-supply = 12 V, LED in series with switch)

LED ON ⇔ LED loop complete ⇔ phototransistor conducts ⇔ OPTO_COL pulled LOW ⇔ **GPIO LOW**.

| Case | Switch / sensor state | LED loop | Opto LED | Phototransistor | OPTO_COL (R2 pull-up) | **GPIO** | Firmware sees |
|------|-----------------------|----------|----------|-----------------|------------------------|----------|---------------|
| A | **Dry NC switch CLOSED (intact, not triggered)** | complete | **ON** (~16 mA) | saturated, sinks | pulled to ~0.2 V | **LOW** | normal running ✔ |
| B | **Dry NC switch OPEN (triggered)** | broken | OFF | open | pulled to 3.3 V | **HIGH** | limit hit / rising-edge IRQ ✔ |
| C | **Wire BROKEN / connector unplugged** | broken | OFF | open | pulled to 3.3 V | **HIGH** | fail-safe trip ✔ |
| D | **NPN-NC sensor, NOT triggered** (output sinks SIGNAL→GND) | complete | **ON** | saturated, sinks | ~0.2 V | **LOW** | normal running ✔ |
| E | **NPN-NC sensor, TRIGGERED** (output opens) | broken | OFF | open | 3.3 V | **HIGH** | limit hit ✔ |
| F | **NPN sensor power lost / cable cut** | broken | OFF | open | 3.3 V | **HIGH** | fail-safe trip ✔ |

Every row satisfies the contract: **intact-NC = LOW**, and **both "triggered" and "broken wire"
= HIGH** with a rising edge on the trigger. **`$5` invert can stay 0.** No polarity inversion is
needed anywhere. This is the design's central claim and it holds for both the dry-switch and the
inductive-sensor wiring on the same 12 V connector.

---

## 6. JST connector — pinout per axis

**Part: JST-XH, 3-pin, vertical through-hole.** One connector per axis (X/Y/Z) plus one for the
probe.

- KiCad footprint: **`Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical`** (verified present in
  the KiCad 10 install). Horizontal/right-angle variant if you prefer board-edge entry:
  `Connector_JST:JST_XH_S3B-XH-A_1x03_P2.50mm_Horizontal`.
- KiCad symbol: `Connector:Conn_01x03_Pin` (generic 3-pin).
- Mating: JST `XHP-3` housing + `SXH-001T-P0.6` crimps; mfr header `B3B-XH-A(LF)(SN)`.

**Pin order (define on silkscreen):**

| Pin | Net        | Dry NC switch | NPN-NC sensor      |
|-----|------------|---------------|--------------------|
| 1   | **+12 V**  | unused        | brown  (V+)        |
| 2   | **SIGNAL** | switch wire A | black  (output)    |
| 3   | **GND**    | switch wire B | blue   (0 V/GND)   |

Why 3-pin (not 2): a dry switch needs only pins 2–3, but standardizing on 3-pin lets the very
same connector accept a 3-wire inductive prox sensor with no board change — the brief's explicit
"accept BOTH" requirement. A 2-wire dry switch simply leaves pin 1 unpopulated in its crimp
housing (or uses a 2-pin XHP-2 housing that seats in positions 2–3 of the 3-pin header — XH
housings key and seat fine partially; document "insert toward the GND end").

JST-XH chosen over PH/GH because: 2.5 mm pitch is robust for field/vibration, positive lock,
locally stocked, and KiCad ships the exact footprint. (JST-PH 2.0 mm also works if space is
tight, same pin order; XH is the recommendation for a vibrating mill.)

---

## 7. Probe input (GPIO21)

Build **one more identical channel** (PC817 + R1 680 Ω + D1 1N4148 + R2 10 kΩ + R3 1 kΩ +
C1 100 nF) on its own 3-pin JST-XH, going to GPIO21. The G38.x probe is logically the same
NC-fail-safe signal, so the same polarity/truth table applies and the same `$5`-free LOW-when-
intact behavior holds. For a simple continuity "probe" (e.g. a touch plate or bit-touches-copper on
PCB work), wire the probe contacts as the dry-switch case (pins 2–3); for a real prox/touch probe, use the 3-wire
case. Treating it identically keeps one BOM line and one mental model.

---

## 8. EMI / robustness notes

- **Twisted pair / shielded cable** for every limit run: twist SIGNAL (pin 2) with GND (pin 3)
  for the dry switch; for the 3-wire sensor use shielded 3-conductor and land the shield on
  chassis/earth at the **board end only** (avoid a ground loop). The opto already breaks the
  ground loop on the logic side.
- **Small cap across the switch terminals at the connector:** 10 nF (C2, optional, one per
  channel) from SIGNAL to GND right at the JST snubs contact-bounce arcing and RF pickup on the
  field side. It sits in the field domain and is independent of C1 on the MCU side.
- **TVS clamp for long 12 V field wiring:** if limit cables exceed ~1 m or route near the
  spindle/VFD, add a bidirectional TVS (e.g. **SMAJ15CA**, 15 V standoff) from +12V_FIELD to
  GND_FIELD at the connector, and optionally a unidirectional clamp on SIGNAL→GND_FIELD. This
  protects R1/D1/the PC817 LED from inductive kicks and ESD coupled onto the field pair. The
  opto barrier protects the MCU regardless, but the TVS protects the cheaper field components.
- Place the PC817s with the **isolation barrier respected on the PCB**: keep ≥ 4 mm copper
  clearance under/around each opto between the 12 V field nets and the 3.3 V/GND_MCU nets, and
  route GND_FIELD as a separate fill from GND_MCU joined (if at all) only at a single defined
  star point — or keep them fully isolated if the 12 V supply is independent.

---

## 9. BOM (per channel × 4 channels: X, Y, Z, PROBE)

| Ref (per ch) | Qty/ch | Value / Part | KiCad symbol | KiCad footprint | Notes |
|--------------|:------:|--------------|--------------|-----------------|-------|
| U1 | 1 | PC817 (LTV-817/EL817 drop-in) | `Isolator:PC817` | `Package_DIP:DIP-4_W7.62mm` | CTR rank A (80–160 %) |
| R1 | 1 | 680 Ω, 1/2 W (820 Ω long-life opt.) | `Device:R` | `Resistor_SMD:R_0805_2012Metric` or TH `R_Axial_DIN0207` | LED limit, 12 V loop |
| R2 | 1 | 10 kΩ, 1/4 W | `Device:R` | `Resistor_SMD:R_0603_1608Metric` | collector pull-up to 3V3 |
| R3 | 1 | 1 kΩ, 1/4 W | `Device:R` | `Resistor_SMD:R_0603_1608Metric` | RC series, ESD limit |
| C1 | 1 | 100 nF X7R 50 V | `Device:C` | `Capacitor_SMD:C_0603_1608Metric` | RC debounce, MCU side |
| C2 | 1 (opt) | 10 nF X7R 50 V | `Device:C` | `Capacitor_SMD:C_0603_1608Metric` | snubber at connector, field side |
| D1 | 1 | 1N4148 (or BAT54 Schottky) | `Device:D` | `Diode_SMD:D_SOD-123` | reverse-protect across LED |
| J1 | 1 | JST-XH 3-pin vertical | `Connector:Conn_01x03_Pin` | `Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical` | field connector |
| TVS (opt) | 1 | SMAJ15CA | `Device:D_TVS` | `Diode_SMD:D_SMA` | only for long 12 V runs |

**Channel count:** 3 axes + 1 probe = **4 identical channels.**
Per-board totals (excluding optional C2/TVS): 4× PC817, 4× 680 Ω, 4× 10 kΩ, 4× 1 kΩ,
4× 100 nF, 4× 1N4148, 4× JST-XH-3. Plus 4× 10 nF and 4× TVS if the optional field protection
is populated.

---

## 10. Caveats & assumptions

1. **Polarity assumption (load-bearing):** correctness hinges on the LED being in series with
   the switch and ON when intact. If a future board revision instead wires the switch on the
   *transistor* side, or sources the LED such that "intact = LED OFF", the polarity inverts and
   you must set `$5` — avoid that; keep this topology.
2. **NPN-NC required, not NPN-NO/PNP.** For the inductive-sensor case the truth table assumes an
   **NPN, normally-closed** sensor (sinks SIGNAL→GND when NOT triggered). A PNP sensor sources
   V+ to the output and would NOT complete the LED loop to GND the same way — it needs the LED
   loop reorganized (LED from SIGNAL to GND, fed by the sensor sourcing current). If PNP support
   is desired, that is a different LED arrangement; document the connector as **NPN-NC only**.
3. **12 V rail must exist and be reasonably clean** on the carrier board. If the only field rail
   available were 5 V, R1 would drop to ~270 Ω (5−1.2)/14 mA and inductive sensors would be out
   of range — so 12 V is the deliberate choice for sensor compatibility.
4. **CTR aging:** designed at CTR floor 50 % with ~24× LOW-drive margin, so years of LED
   degradation are covered. The 820 Ω "long-life" R1 trades a little margin (still ~20×) for
   lower LED stress.
5. **Shared vs isolated field ground:** the design shows GND_FIELD distinct from GND_MCU. If the
   12 V supply is derived from the same brick as the 3.3 V/5 V logic, the isolation is "functional
   noise isolation," not safety isolation. For true galvanic isolation, power the 12 V field rail
   from a separate supply. Either way the opto correctly translates levels.
6. **`$26` debounce unchanged:** the 100 µs RC is intentionally << `$26`; do not raise C1 to where
   τ approaches the firmware debounce or you will smear the rising edge the IRQ needs.
