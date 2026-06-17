# Breadboard stepper bring-up wiring (NEMA 17 + TMC2209)

Practical wiring guide for bench-testing the Galdr motion stack on a breadboard with **3× NEMA 17**
steppers driven by **3× TMC2209 (Adafruit 6121 breakout)**. Pin assignments are the authoritative
firmware values — they match `crates/firmware/src/main.rs` (`motion::init`/`tmc::init`) and the GPIO
manifest in [`00-architecture.md`](00-architecture.md). If you change a GPIO here, change it there too.

> ⚠️ **Read "Power & breadboard safety" before applying motor power.** A NEMA 17 at the default
> 800 mA RMS pulls ~1.1 A peak per phase; that current must NOT run through breadboard power rails.

---

## 1. Bill of materials

| Qty | Item | Notes |
|-----|------|-------|
| 1 | ESP32-S3 devkit | The board the firmware runs on (native USB to host) |
| 3 | TMC2209 breakout (Adafruit 6121) | 0.05 Ω sense resistors; 5–29 V motor, 3–5 V logic |
| 3 | NEMA 17 stepper, **bipolar (4-wire)** | Confirm bipolar; 6-wire needs the right pair tapping |
| 1 | Motor PSU, **12 V (or up to 24 V)** DC | Sized for 3× motor current + margin (≥ 3 A for a start) |
| 1 | Breadboard + jumper wires | Logic/signal only — see safety note |
| 3 | 1 kΩ resistor | One per driver, in series on the shared UART line *(see §4)* |
| 3 | 100 µF electrolytic cap (≥ motor V rating) | One across **VM↔GND at each driver**, close to the chip |
| — | Heavier gauge wire (e.g. 22–20 AWG) | For VM and motor-coil runs (off the breadboard rails) |
| 1 | Multimeter | Identify motor coil pairs; verify rails before power-up |

The spindle (WS55-220), limit switches, and Z-probe are **not** needed for stepper testing and are
omitted here.

---

## 2. Pin map (ESP32-S3 → TMC2209)

These are the firmware's fixed assignments. STEP is per-axis; **DIR, EN, and the UART line follow the
same scheme**. `STEP_EN` (GPIO8) is a **single common enable** wired to *all three* drivers' EN pins.

| Firmware signal | ESP32-S3 GPIO | Goes to (each TMC2209) | Notes |
|-----------------|---------------|------------------------|-------|
| X_STEP | **GPIO1** | X driver `STEP` | RMT TX ch0 |
| Y_STEP | **GPIO2** | Y driver `STEP` | RMT TX ch1 |
| Z_STEP | **GPIO4** | Z driver `STEP` | RMT TX ch2 |
| X_DIR  | **GPIO5** | X driver `DIR` | |
| Y_DIR  | **GPIO6** | Y driver `DIR` | |
| Z_DIR  | **GPIO7** | Z driver `DIR` | |
| STEP_EN | **GPIO8** | **all** drivers `EN` (a.k.a. ENN) | Active-LOW; firmware drives it LOW (enabled) at boot |
| TMC_UART | **GPIO9** | shared `PDN_UART` bus (via 1 kΩ each) | Single-wire half-duplex, 115200 8N1 — see §4 |
| 3V3 | ESP32-S3 **3V3** | each driver `VIO` | Logic supply for the drivers |
| GND | ESP32-S3 **GND** | common ground (see §5) | **All grounds must be common** |

> Strapping pins to avoid: GPIO0/3/45/46. GPIO19/20 are the native USB lines — do not touch.
> The chosen GPIOs above are all safe, non-strapping pins.

---

## 3. NEMA 17 motor coil wiring

A bipolar NEMA 17 has two coils → four wires: **A1, A2** (coil A) and **B1, B2** (coil B). They wire to
the TMC2209 motor outputs, **directly to the driver, not through the breadboard**:

| Motor wire | TMC2209 output (silkscreen varies) |
|------------|------------------------------------|
| A1 | OA1 / A2 (one end of coil A) |
| A2 | OA2 / A1 (other end of coil A) |
| B1 | OB1 / B2 (one end of coil B) |
| B2 | OB2 / B1 (other end of coil B) |

**Find the coil pairs with a multimeter (resistance mode):** the two wires of one coil read a small
resistance (a few ohms / continuity); wires from different coils read open. Keep each coil's two wires
on the same driver output pair (OAx together, OBx together). If the motor spins the "wrong" way, swap
the two wires of **one** coil (or just flip direction in G-code / `$3` later) — never split a coil pair.

> 🔌 **Never connect or disconnect a motor while the driver is powered (VM on).** Back-EMF from the
> coils can destroy the TMC2209. Power down VM before touching motor wires.

---

## 4. Shared single-wire UART bus (the one tricky part)

All three TMC2209s talk to the MCU over **one half-duplex wire**: ESP32-S3 UART1 on **GPIO9** drives a
single shared node that every driver's `PDN_UART` pin connects to. The firmware transmits and reads its
own echo back on the same pin (DOC-03).

Wiring:
- Run **GPIO9 → a common rail node** on the breadboard.
- From that node, a **1 kΩ resistor in series to each driver's `PDN_UART`** pin (one 1 kΩ per driver).
  The series resistors limit contention current and isolate the drivers on the shared line.
- That's it — no separate RX/TX; the single line is bidirectional.

```
 ESP32-S3 GPIO9 ───────┬───[1kΩ]─── PDN_UART (X / node 0)
                       ├───[1kΩ]─── PDN_UART (Y / node 1)
                       └───[1kΩ]─── PDN_UART (Z / node 2)
```

### Node addressing — set MS1/MS2 per driver

In UART mode the **MS1/MS2 pins select the node address** (NOT microstepping — microstepping is set over
UART). Set them with jumpers to 3V3 (HIGH) or GND (LOW); they have internal pull-downs (floating = LOW).
Assign one address per axis so the firmware can reach each driver on the shared bus:

| Axis | Node | MS1 | MS2 |
|------|------|-----|-----|
| X | 0 | LOW (GND/float) | LOW (GND/float) |
| Y | 1 | **HIGH (3V3)** | LOW |
| Z | 2 | LOW | **HIGH (3V3)** |
| (spare) | 3 | HIGH | HIGH |

Getting two drivers on the same address will make them collide on the bus → the firmware reports those
nodes as `absent`. Double-check MS1/MS2 before powering up.

---

## 5. Power & breadboard safety

Two separate supplies, **one common ground**:

- **Logic:** ESP32-S3 powered over USB from the host; its **3V3** feeds every driver's **VIO**.
- **Motor:** the 12–24 V PSU feeds every driver's **VM**. Put a **100 µF cap across VM↔GND at each
  driver**, as close to the chip as possible (suppresses the voltage spikes that kill drivers).
- **Common ground is mandatory:** ESP32-S3 GND ↔ all VIO GND ↔ motor PSU GND must be tied together,
  or the UART/step signals have no reference and nothing works (or drivers get damaged).

> ⚠️ **Keep motor current off the breadboard rails.** A NEMA 17 at the default 800 mA RMS draws ~1.1 A
> peak per phase; breadboard spring contacts are only good for ~1 A and will heat/sag. For VM and the
> motor-coil runs, use heavier jumpers wired **point-to-point directly to the driver pins** (or a small
> screw-terminal strip), bypassing the breadboard power rails. The breadboard is fine for the 3V3/GND
> logic, STEP/DIR/EN, and the UART line — just not the motor power path.
>
> For the *very first* spin test you can also lower the run current to ~400 mA (`$140=400` etc., see §7)
> to keep things cool and gentle while you confirm everything's wired right, then raise it.

**Power-up order:** logic (USB) first, motors (VM) second. **Power-down:** VM off first, USB last.

---

## 6. Per-driver wiring checklist

For **each** of the three TMC2209 breakouts:

- [ ] `VIO` → ESP32-S3 3V3
- [ ] `GND` → common ground
- [ ] `VM` → motor PSU + (with 100 µF cap VM↔GND at the driver) — *heavy wire, off-rail*
- [ ] `EN` (ENN) → ESP32-S3 **GPIO8** (shared by all three)
- [ ] `STEP` → its axis STEP GPIO (X=1, Y=2, Z=4)
- [ ] `DIR` → its axis DIR GPIO (X=5, Y=6, Z=7)
- [ ] `PDN_UART` → 1 kΩ → shared GPIO9 node
- [ ] `MS1`/`MS2` → set node address per §4 table
- [ ] `OA1/OA2` → motor coil A pair; `OB1/OB2` → motor coil B pair — *direct, off-rail*

Leave `DIAG`, `INDEX`, `CLK`, `SPREAD` unconnected (CLK floating = internal oscillator).

---

## 7. Firmware defaults & relevant `$` settings

The drivers are configured over UART by the `tmc_manager` task at boot from these settings (live values
confirmed via `$$`):

| Setting | Default | Meaning |
|---------|---------|---------|
| `$100/$101/$102` | 250.000 | steps per mm (X/Y/Z) |
| `$110/$111/$112` | 500.000 | max rate mm/min (rapid speed) |
| `$120/$121/$122` | 10.000 | acceleration mm/s² |
| `$140/$141/$142` | 800 | **RMS run current, mA** (max clamp 2000) |
| `$150/$151/$152` | 16 | microstep resolution |

Adjust before/after motion with e.g. `$140=400` (gentler first test) or `$140=1000`. Set, then it
persists to flash. Hold current, R_sense (0.05 Ω), and other advanced TMC params are defaults / `$PBX`.

> **What `G0 X5` does on the bench:** with 250 steps/mm and 16 microsteps (3200 µsteps/rev), `X5` =
> 1250 µsteps ≈ **0.39 shaft revolution** (there's no leadscrew, so "mm" is just the configured step
> scale). For a more obvious spin use a bigger number, e.g. `G0 X50` ≈ 3.9 revolutions.

---

## 8. Bring-up & test procedure

1. **Wire logic only** (no VM yet): 3V3→VIO, GND common, STEP/DIR/EN, the UART bus + 1 kΩ resistors,
   MS1/MS2 addresses. Double-check addresses and that EN is on GPIO8 for all three.
2. **Power logic (USB)** and flash/connect:
   ```sh
   just flash                                  # production build
   # or, to watch driver detection over the defmt/RTT channel:
   DEFMT_LOG=trace just flash --features defmt
   ```
3. **Confirm the drivers are detected.** With the defmt build you'll see, per axis, either
   `TMC axis N node M: configured, version 0x21 ...` (good) or `... absent (no reply ...)`. The TMC2209
   answers UART on **VIO alone** (no VM needed yet), so this verifies the bus + addressing before any
   motor power. Fix any `absent` node (check MS1/MS2, the 1 kΩ run, GND continuity) before continuing.
4. **Power the motors (VM).** Caps in place, common ground confirmed, motors already connected
   (remember: never hot-plug motors). Optionally set a gentle current first: `$140=400`, `$141=400`,
   `$142=400`.
5. **Jog test** (relative move so position is obvious):
   ```
   G91          ; relative
   G0 X50       ; X should spin ~3.9 rev; repeat for Y / Z
   G0 Y50
   G0 Z50
   G90          ; back to absolute
   ?            ; expect <Idle|WPos:...> with the axis advanced
   ```
   A `?` mid-move shows `Run` with `WPos` climbing and `FS` = the rapid rate. Each axis should turn
   smoothly; reverse with `G0 X-50`.
6. **Tune.** Raise `$140` toward the motor's rating if torque is low and the driver stays cool; raise
   `$110` for faster rapids. Re-check thermals — TMC2209s get warm; add airflow/heatsinks for sustained
   current.

---

## 9. Troubleshooting

| Symptom | Likely cause |
|---------|--------------|
| Driver `absent` in defmt log | MS1/MS2 address clash or wrong; missing 1 kΩ; GPIO9 not on the bus; no common GND; VIO not 3V3 |
| All three `absent` | UART line not reaching GPIO9, or GND not common between MCU and drivers |
| Motor whines / vibrates, no rotation | One coil pair split across A/B; swap so each coil's two wires share an output pair |
| Motor spins wrong direction | Swap the two wires of **one** coil, or flip `$3` dir-invert later |
| Motor weak / skips steps | Run current too low (`$140`), accel too high (`$120`), or rapid too fast (`$110`); VM sagging |
| Driver hot / cuts out | Run current too high for breadboard wiring or no airflow; VM cap missing; lower `$140` |
| Nothing moves, `Bf` drops, returns to Idle | Steps generated but motor unpowered — VM off, or EN not LOW (check GPIO8 → all EN pins) |
| Erratic / resets under motion | Motor current through breadboard rails (voltage droop) — move VM/coils off-rail (§5) |

---

## 10. Quick reference diagram (one axis)

```
            ESP32-S3 devkit                         TMC2209 (e.g. X, node 0)
        ┌───────────────────┐                    ┌────────────────────────┐
   USB ─┤ (host)        3V3 ├────────────────────┤ VIO                    │
        │               GND ├───────┬────────────┤ GND                    │
        │             GPIO1 ├───────┼────────────┤ STEP        OA1 ├──┐    │
        │             GPIO5 ├───────┼────────────┤ DIR         OA2 ├──┤ coil A
        │             GPIO8 ├───────┼────────────┤ EN          OB1 ├──┐    │
        │             GPIO9 ├──[1kΩ]┼────────────┤ PDN_UART    OB2 ├──┤ coil B
        └───────────────────┘       │   MS1→GND  ┤ MS1                │    │
                                    │   MS2→GND  ┤ MS2          VM ├──┼──┐ │
   12–24V PSU + ──────────────────────────(heavy, off-rail)─────────┘  │ │
   12–24V PSU − ──────────────────────┴──────────────────────────(common GND)
                                                  100µF across VM↔GND ──┘
        (Y on GPIO2/6, node1 MS1=HIGH; Z on GPIO4/7, node2 MS2=HIGH; EN+UART+GND shared)
```

When all three axes jog smoothly off the breadboard, you're ready to move to a soldered harness /
proper CNC frame.
