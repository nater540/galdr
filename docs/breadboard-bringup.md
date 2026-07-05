# Breadboard Bring-Up — BTT TMC2209 stepsticks

> First-hardware bring-up on a breadboard, BEFORE milling the carrier PCB. This build uses **BTT / Watterott /
> FYSETC TMC2209 stepstick modules** (0.11 Ω sense) on a solderless breadboard — *not* bare TMC2209 silicon on
> Adafruit 6121 breakouts. Some design docs (`hardware/DESIGN.md`, DOC-03) and the firmware's compile-time
> default still carry the older Adafruit 6121 assumption; §0 below is what makes the board match the stepsticks
> you actually have. The opto-isolated limit stage is also deferred on the breadboard (see §3). Read alongside
> the bench checklists (`docs/homing-bench-checklist.md`, `docs/4th-axis-bench-checklist.md`).
>
> Board: ESP32-S3, native USB Serial/JTAG → `/dev/cu.usbmodem31101`.

## 0. The one that matters — sense resistor

The TMC current-scale math (`firmware-core/src/drivers/tmc2209/registers.rs`) is driven by the sense-resistor
value, and the BTT stepsticks use a **larger sense than the Adafruit 6121 the firmware still defaults to**:

| Driver board                                                   | R_sense    | Constant                     |
|----------------------------------------------------------------|------------|------------------------------|
| BTT / Watterott / FYSETC TMC2209 stepstick (**this build**)    | **0.11 Ω** | `R_SENSE_BTT_TMC2209_OHMS`   |
| Adafruit 6121 breakout (legacy DOC-03 default — not used here) | 0.05 Ω     | `R_SENSE_ADAFRUIT_6121_OHMS` |

**Verify the silkscreen/schematic for your board revision — clones vary.** If the firmware runs the 0.05 Ω
Adafruit default while the hardware is the 0.11 Ω stepstick, every `IRUN`/`IHOLD` resolves to ≈ 2.2× the
intended coil current (the ratio of the two senses): the motor overheats and the driver can fault. The larger
0.11 Ω sense is otherwise *advantageous* here — it gives finer CS resolution at the low currents typical of
light desktop milling.

**To select the BTT sense:** set the `BREADBOARD_STEPSTICKS` toggle in `TmcConfig::default()`
(`firmware-core/src/drivers/tmc2209/manager.rs`) to `true`. Both constants are kept present so the
swap is a single line. Note `tmc_r_sense_ohms` is a **persisted** setting — the compile-time default only seeds
a fresh/erased flash, so on a board that already has settings stored, push the value over the `$PBX` host-sync
channel (or erase flash) rather than relying on the rebuild alone.

## 1. Wiring the BTT stick for the single-wire UART

Differs from the bare-driver + breakout assumption in DOC-03:

- **VIO → 3.3 V.** The logic reference must match the ESP32-S3. Don't float it; don't tie it to 5 V.
- **PDN_UART → the shared bus (GPIO9).** The canonical multi-driver scheme is MCU-TX → **1 kΩ** → PDN_UART
  with MCU-RX tapped directly on PDN_UART. Many BTT "UART"-labeled v1.2 boards already carry that 1 kΩ on the
  UART pad — if so, do NOT add a second one. The carrier's `R_UPU` 20 kΩ idle pull-up on the bus still applies.
- **MS1/MS2 → unique node address per driver (0/1/2/3 = X/Y/Z/A).** In UART mode these pins set the bus
  *address*, not the microstepping (microstepping is programmed over UART). The firmware expects
  `TMC_NODES = [0, 1, 2, 3]`. A duplicated address garbles datagrams for *every* driver on the shared wire,
  not just the offender.
- **EN → shared STEP_EN (GPIO8), active-low.** All four drivers tie to the one enable line.
- **DIAG → leave unconnected.** That pin is for sensorless StallGuard homing; this machine homes on
  switch/opto limits, so DIAG is unused.
- **STEP / DIR** per the GPIO manifest: STEP X/Y/Z = GPIO1/2/4, A = GPIO18 (RMT ch3); DIR X/Y/Z = GPIO5/6/7,
  A = GPIO38. (A is PROVISIONAL — DOC-10 Phase 5.)

## 2. Breadboard cautions (reliability / not letting the smoke out)

- **Motor coil current must NOT pass through breadboard tie-points.** They are ~1 A-rated and intermittent;
  coil currents heat and open them. Run VM→driver and driver→motor with screw terminals or soldered leads.
  Keep only the *signal* pins (STEP/DIR/EN/UART/limits) on the breadboard.
- **NEVER plug/unplug a motor while VM is powered.** The inductive spike is the most common way these drivers
  die on the bench. Power down first, every time.
- **Bulk cap on VM:** ~100 µF electrolytic across VM↔GND right at each stick (the onboard cap is small;
  breadboard lead inductance makes the LC spikes worse).
- **Common ground** between the ESP, the drivers, and (if used) the opto/limit stage.
- Step/dir on a breadboard is fine for bring-up at modest rates. If you chase missed steps at high feed
  *later*, breadboard parasitics are a suspect — but not an initial concern.

## 3. Limit switches — skip the opto on the breadboard

You don't need to build the PC817 + 12 V field stage (`hardware/DESIGN.md`) to bring up the limit logic. Wire a
bare **2-wire NC micro switch (or just a jumper) straight from the limit GPIO to GND**. The firmware enables the
internal pull-up on every limit pin, giving the *identical* polarity as the opto design — **no `$5` invert**:

| At the pin                           | Level | Firmware reads           |
|--------------------------------------|-------|--------------------------|
| Switch closed (jumper to GND)        | LOW   | **not** triggered (idle) |
| Switch open / floating / broken wire | HIGH  | **triggered**            |

So on the bench a **trip is the *open*, not a connection to ground** — disconnecting a pin (or releasing the
jumper) raises `ALARM:1` (once `$21` is armed, §3.2); grounding it returns to idle. The GND→open transition is a
real rising edge, so this also exercises the `wait_for_rising_edge` IRQ path. PROBE (GPIO21) is the same, except
its pull-up follows `$19`. Defer the opto stage to the milled PCB; validate it there per the homing checklist
§1/§3.

### 3.1 Wiring the NC micro switches

| Axis | Limit GPIO | Note                                                                       |
|------|------------|----------------------------------------------------------------------------|
| X    | GPIO10     |                                                                            |
| Y    | GPIO11     |                                                                            |
| Z    | GPIO12     |                                                                            |
| A    | GPIO39     | Placeholder only — A has no physical switch (DOC-10.6); leave it jumpered. |

- **Use the COM + NC terminals.** Most lever micro switches carry three tabs (COM/NO/NC, usually marked on the
  body). Verify with a continuity meter before wiring: the correct pair reads **closed at rest and open when the
  lever is pressed**. On the NO tab by mistake, the polarity table above reads inverted at rest — you'd boot
  "triggered" on that axis.
- One lead to the limit GPIO, the other to GND — the switch is passive, so lead order doesn't matter. **No
  external resistor**: the firmware configures the internal pull-up on every limit pin unconditionally (the NC
  broken-wire fail-safe depends on it).
- Signal-only wiring, so breadboard tie-points are fine here (§2 applies to motor current, not limits). Keep the
  limit leads short and routed away from the motor leads — an NC input held low by the switch is noise-tolerant,
  but a long open run next to a stepper coil invites phantom trips once motion is running.
- An unwired axis MUST be jumpered to GND (that's the "switch closed" state). A floating limit pin reads
  triggered — with `$21` armed the board sits in a permanent `ALARM:1`.

### 3.2 Bench-testing the endstops (no motion, drivers unpowered)

The status pipeline samples the limit pins and publishes their logical state (after `$5`) into the `Pn:` field
of the `?` report **independently of `$21`**, so start with hard limits DISARMED and just watch pins:

1. `$21=0` (hard limits off) for the first pass — a press should *report*, not alarm.
2. Poll `?` (or connect `skirnir`, which polls for you and draws the X/Y/Z endstop indicators from `Pn:`).
   At rest — all NC switches closed — the report carries **no** `Pn:` field at all (it's omitted when nothing
   is asserted); the machine must NOT be in alarm.
3. **Press and hold each switch in turn** and confirm the matching letter appears — `<Idle|...|Pn:X>` for the
   X switch, then `Y`, then `Z`. This proves each switch lands on the right GPIO. Release and confirm the
   letter drops again. Latency note: at idle the executor samples on a 50 ms ticker, so the indicator updates
   within ~a tick, not instantaneously.
4. **Broken-wire fail-safe:** unplug one switch lead entirely — the pull-up floats the pin HIGH and the letter
   asserts, exactly like a press. This is the property that makes NC the right choice: a severed wire looks
   like a trip, never like a silently disabled limit.
5. **`$5` sense check:** if an axis reads asserted at rest and clear when pressed, you're on the NO tab
   (rewire, §3.1) or the switch is miswired — fix the wiring rather than papering over it with `$5` (the
   default `$5=0` is correct for NC-to-GND, and inverting it forfeits the broken-wire fail-safe).
6. **Arm the alarm path:** set `$21=1` and repeat one press — the board must now raise `ALARM:1` within the
   `$26` debounce window and enter alarm. Recover with `0x18`/`$X` and confirm it re-arms (a second press
   alarms again).

From here, hand over to `docs/homing-bench-checklist.md` §3–§9 for the full procedure — debounce (`$26`),
per-axis alarm mapping, `$H` homing against the real switches, hard limits during motion, and repeatability.

## 4. Pin sanity on the dev board

The assigned GPIOs avoid the ESP32-S3 strapping pins (0/3/45/46), USB (19/20), and the SPI-flash pins. Confirm
your specific module: an **N16R8 (octal PSRAM)** WROOM reserves GPIO33–37 (not used here, so you're clear), and
check no onboard peripheral squats on a limit/spindle/step pin. GPIO38/39 (the A axis) are free on most boards.

## 5. Suggested first-power sequence

1. ESP only (no VM): flash firmware, confirm the grblHAL banner, `?` status, `$$` dump. No drivers energized.
2. Set the sense-resistor value (§0) and conservative low `$103`-class currents before energizing.
3. One driver at a time: VIO + UART, confirm `tmc_manager` reports it configured (version + CS + vsense in the
   monitor) and that a node-3-style read-back proves two-way comms. Add the next address only once the prior
   one enumerates cleanly.
4. Add VM (off-breadboard wiring, bulk cap, motor connected with power OFF), then test a small jog per the
   bench checklists. Hand on the power cut.
