# DOC-10 4th-Axis Rotary (A) — Hardware Bring-Up Checklist

> Hardware-in-the-loop bench procedure for the coordinated rotary A axis. The logic is host-tested
> (456+ firmware-core tests) and Xtensa-compiled, but everything electrical/timing below is verifiable
> ONLY on the real board. Work top-to-bottom; do NOT skip the §1 wiring audit before applying power.
> Companion to `docs/homing-bench-checklist.md` (DOC-06) and the spindle bring-up — same posture: logic
> host-tested, hardware path deferred (DOC-10 R4).
>
> Board: ESP32-S3, native USB Serial/JTAG → `/dev/cu.usbmodem31101`, RX buffer 1024.
> **A-axis wiring is PROVISIONAL (DOC-10 Phase 5, `main.rs:333–356`)** — confirm the carrier routes these
> before powering: A-STEP = GPIO18 (RMT TX ch3, the documented spare), A-DIR = GPIO38, shared STEP_EN =
> GPIO8 (all axes), TMC A-driver = node 3 on the shared UART1 single-wire bus (GPIO9), MS1/MS2 strapped to
> address 3. **A has NO physical limit switch** — GPIO39 is a placeholder only (DOC-10.6); A never homes,
> never appears in `Pn:`, and never raises a hard-limit alarm.
> A is axis index 3, rotary about X, **units = degrees** carried as "mm" through the planner (grbl convention).
> Relevant settings: `$103` steps/deg, `$113` deg/min, `$123` deg/s², `$133` max-travel-deg (IGNORED for a
> rotary axis), `$376` rotary-axes bitmask (default `8` → bit 3 = A).

## 0. Safety preamble
- [ ] **E-stop / motor-power cut within reach.** A wrong `$103` or DIR polarity spins the fixture the wrong
      way or far past intent; a coordinated move can drag a linear axis with it.
- [ ] Mechanically secure the rotary fixture: chuck/collet tight, workpiece balanced, no loose cabling that
      can wrap the rotating axis. Clear pinch points around the chuck.
- [ ] Confirm cable management for the rotating fixture (slip ring / service loop) — A is **continuous**
      (rollover, no soft limit), so an unbounded program can rotate it many turns. Do not leave it unattended.
- [ ] Spindle unpowered / no tool for the entire A bring-up.
- [ ] Linear axes parked mid-travel with room to move (coordinated tests in §8 command X+A together).

## 1. Wiring audit — BEFORE power (PROVISIONAL pins)
The GPIOs in `main.rs` are provisional; a carrier mis-route here drives the wrong pin.
- [ ] **Confirm the carrier actually routes** A-STEP→GPIO18, A-DIR→GPIO38, A-driver-EN to the shared STEP_EN
      (GPIO8), and the TMC A-driver UART to the shared bus (GPIO9). If the carrier differs, update the
      `motion::init` tuples (`main.rs:334–335`) + the DOC-00 GPIO manifest FIRST — do not patch at the bench.
- [ ] **TMC node-3 address straps:** verify MS1/MS2 on the A driver are strapped for UART address **3**
      (nodes 0/1/2 = X/Y/Z). A wrong strap = address collision → garbled datagrams on the shared bus for
      *all* drivers, not just A.
- [ ] **Sense resistor:** confirm the A driver breakout uses the same **0.05 Ω** sense as X/Y/Z (Adafruit
      6121) before trusting any IRUN/IHOLD current scaling.
- [ ] **GPIO39 (A-LIMIT placeholder):** A has no switch. Confirm GPIO39 is tied to a defined level (or left
      with its internal pull-up) and is NOT cross-wired to a real limit — a stray edge here must never gate
      motion. §10 verifies the firmware ignores it regardless.
- [ ] Ch3 RMT `mem_block_symbols ≤ 48` (one memory block) — the 4-channel layout must not borrow ch0's block.
      This is a compile-time config, but confirm A-STEP pulses don't corrupt X-STEP under load (§8/§12).

## 2. Flash & connect — confirm the 4-axis protocol
- [ ] `just flash` (build + flash + monitor) — confirm the grblHAL welcome banner on boot (and on every
      `0x18` soft-reset; native USB can't be host-reset).
- [ ] `?` returns a **4-field** `<...>` status line: `MPos:0.000,0.000,0.000,0.000` (X,Y,Z,**A**). The 4th
      field is A in degrees. A 3-field report means the `AXES`/`AXIS_COUNT` bump didn't take — stop and fix.
- [ ] `$I` / `0x87` answer; the axis-count report advertises `[AXS:4:XYZA]`.
- [ ] `$G` shows the live feed mode (`G94` default, or `G93` after a G93). Pre-DOC-10 builds hardcoded `G94`;
      confirm it now reflects the modal state.
- [ ] `skirnir` connects and shows a 4th (A) DRO field — host and firmware agree on 4 axes.

## 3. Verify A settings before any motion
Dump with `$$` and confirm the A block:
- [ ] `$103` steps/degree, `$113` max rate (deg/min), `$123` accel (deg/s²) read back as intended. Start
      CONSERVATIVE (low `$113`/`$123`) for first motion.
- [ ] `$376` = `8` (default; bit 3 → A rotary). Confirm it reads back. `$376=0` runs A as a bounded linear
      4th axis (different semantics) — not what we want for the rotary fixture.
- [ ] `$133` (A max travel) — note it is **IGNORED** while A is rotary in `$376`; A is continuous. Don't rely
      on it to bound motion.
- [ ] A TMC current (`$103`-adjacent run/hold settings, node 3) set sanely for the rotary motor — verify
      against the 0.05 Ω sense before energizing.

## 4. TMC node-3 driver bring-up (shared UART)
A shares UART1 with X/Y/Z; node 3 must come up without disturbing the others.
- [ ] On boot, the `tmc_manager` init sequence runs for node 3 too. Confirm the monitor shows the A driver
      initialized (no CRC/timeout errors on node 3) alongside nodes 0–2.
- [ ] Read back a node-3 register (IFCNT increments per write; version/`IOIN`) to prove two-way comms on the
      A address — a one-way write that "works" can mask a wrong strap.
- [ ] Confirm nodes 0/1/2 still init cleanly with node 3 on the bus (no new collisions). If X/Y/Z drivers
      regress when A is connected, suspect the address strap (§1) or bus loading.
- [ ] A driver reports enabled when STEP_EN (GPIO8) is asserted; de-energizes with the others on disable.

## 5. A direction & enable (small, deliberate moves)
First motion — hand on the power cut.
- [ ] With the machine idle and unlocked (`$X` if boot-locked by `$22`), command a SMALL A move, e.g.
      `G91 G1 A10 F500` (relative, 10°). Confirm the A motor turns and STEP_EN gates it.
- [ ] **Direction sense:** confirm `+A` rotates the work in the intended direction (define +A = CCW looking
      from +X toward origin, or your shop convention). If reversed, flip the A DIR polarity in the driver
      config — record the change; don't leave it implicit.
- [ ] `G91 G1 A-10` returns it; net position back to start. `?` A field tracks the commanded degrees.
- [ ] Confirm A motion is SMOOTH at the conservative `$113`/`$123` — no missed steps / stall on accel.

## 6. Steps-per-degree calibration (`$103`)
- [ ] Mark a reference on the fixture. Command exactly one revolution: `G91 G1 A360 F1000`. The mark should
      return to start. Measure the error.
- [ ] Adjust `$103` until `A360` = one true mechanical revolution (account for gear/belt reduction in the
      rotary fixture, not just motor steps × microsteps). Re-test.
- [ ] Repeatability: command `A360` ×5; the mark spread should be within one microstep. Drift indicates lost
      steps (raise current / lower `$113`/`$123`) or a slipping coupling.

## 7. Single-axis A velocity profile
- [ ] Sweep A at increasing `$113` until you find the reliable ceiling (no stall), then back off ~20%.
      Record the safe max rate.
- [ ] Confirm accel (`$123`) gives a clean ramp — too high stalls on start, too low makes coordinated moves
      sluggish. Tune for the fixture inertia.

## 8. Coordinated 4-axis motion (the load-bearing test)
A must interpolate WITH the linear axes, not run after them.
- [ ] **Mixed linear+rotary, arrive together:** `G90 G1 X10 A90 F600`. X and A must start and FINISH
      simultaneously (DDA coordination) — neither waits for the other. Watch both; a stagger means the
      4-term DDA/`join4` widening (`motion.rs`) didn't take.
- [ ] **Long mixed move:** `G1 X50 A720 F800` — sustained co-motion, no drift between X and A over the move.
- [ ] **Arc with A slave (DOC-10.5):** an XY arc (`G2`/`G3`) with an A endpoint interpolates A **linearly**
      across the arc segments and lands on the A target at arc end. Confirm A advances steadily through the
      arc, not in a jump at the end.
- [ ] Confirm `Pn:`/status during co-motion is sane and the move reports exactly one `ok` on completion.

## 9. Feed semantics on the bench (G93 / G94)
The highest-bug-risk area (DOC-10 R1). Verify the *timing*, not just that it moves.
- [ ] **G93 inverse-time:** `G93 G1 X10 A90 F2` means "this move takes 1/2 min = 30 s". Time it — the whole
      move should take ~30 s regardless of the X/A split. `F` is per-move duration, not a rate.
- [ ] **G93 requires F per line** (except G0 rapids): a G93 `G1` with no `F` errors; a `G93 G0` rapid is
      exempt and runs. Confirm both.
- [ ] **G93 over-speed clamp (Q3):** command a G93 duration faster than `$113`/`$11x` allows. The move must
      finish *slower* than commanded (never lose steps) and emit a `[MSG:…]` advisory — NOT reject.
- [ ] **G94 mixed move → ROTARY_FIX:** under G94, `G1 X10 A90 F600` is internally converted to inverse-time
      (grbl's `ROTARY_FIX`) using the full all-axis norm. Confirm it runs at a sane coordinated speed (the
      DOC-10.2 fix) — no runaway, no crawl. Compare against a pure-linear `G1 X10 F600` for sanity.
- [ ] **G20 does not scale A:** `G20 G1 A90` rotates 90°, NOT 90×25.4. Confirm the rotary word bypasses inch
      scaling while a linear word in the same line is still scaled.
- [ ] `$G` reports the active feed mode flips between `G93`/`G94` as commanded.

## 10. Homing & limits interaction (DOC-10 R5 — the safety guard)
A is rotary with no switch; the homing/limit paths must EXCLUDE it.
- [ ] **`$H` must SKIP A:** issue `$H`. It homes Z, then X, then Y — and **never waits on GPIO39 / a
      non-existent A switch**. If `$H` hangs, the `$376` rotary skip in `run_homing` isn't gating A (R5).
- [ ] **A never in `Pn:`:** wiggle/ground GPIO39 (or its open-pull-up); the status `Pn:` field must NEVER
      show an A flag and must NEVER raise `ALARM:1`. A is excluded from limit sampling (`motion.rs:833/891`,
      Phase-5 TODO) — confirm that exclusion holds on real hardware.
- [ ] **A exempt from soft limits (`$20`):** with `$20` enabled and the machine homed, command a large A
      move (`G1 A3600`). It must NOT be rejected with `ALARM:2` — A is continuous/rollover, `$133` ignored.
      A *linear* over-travel in the same program still gets rejected.
- [ ] Confirm an X/Y/Z hard-limit trip (`$21`) during a coordinated X+A move halts BOTH axes (shared
      executor) and forces the spindle off — A doesn't keep spinning after a linear alarm.

## 11. Probing interaction (rejections — host + firmware)
Rotary probing is host-side (skirnir); the firmware stays a linear-G38 executor.
- [ ] **G93 + G38 rejected:** `G93` active then `G38.2 Z-5 F100` errors — inverse-time is undefined for a
      probe (it stops at an unknown contact). Confirm the error, not a mis-executed probe.
- [ ] **Rotary word in a probe rejected:** a `G38.x` carrying an `A` word errors (the firmware probe is
      linear-only; rotary probing is projected via G10 offsets from skirnir, not a firmware rotary probe).
- [ ] A plain linear `G38.2 Z-…` still runs at units/min (probe feed is unaffected by the feed-mode work).

## 12. Step budget / symbol batching (Q2)
- [ ] **High-`$103` pure-rotary long move:** with a realistic engraving `$103`, command a long A-only move
      (`G1 A1440`). Confirm it completes smoothly — the ≤48-symbol burst batching chunks the large
      `step_event_count` without stalling or starving ch3. No X/Y/Z step corruption during it (block-borrow
      check from §1).
- [ ] Confirm ch3 emit timing under a sustained A burst doesn't degrade X/Y/Z step timing in a 4-axis move.

## 13. Fault / abort paths
- [ ] **E-stop / ALARM / `0x18`:** during an A move, soft-reset (`0x18`). Confirm A halts immediately, the
      banner re-emits, STEP_EN disables the A driver with the others, and no stray A move runs afterward.
- [ ] **Mid-move abort under coordination:** abort an X+A move; confirm BOTH stop and position is reported
      consistently (4-field MPos) — no axis left mid-step.

## Sign-off
- [ ] 4-field protocol (`MPos`/`WPos`/`[AXS:4:XYZA]`) correct end-to-end (firmware ↔ skirnir).
- [ ] `$103` calibrated (`A360` = one true revolution), 5× repeatable; direction sense confirmed/recorded.
- [ ] Coordinated X+A and arc-A-slave moves arrive together; G93 duration and G94 ROTARY_FIX timing correct.
- [ ] `$H` skips A; A never in `Pn:`, never alarms on GPIO39, exempt from soft limits.
- [ ] TMC node 3 two-way comms verified without disturbing nodes 0–2.
- [ ] All fault paths disable A and abort cleanly.
- [ ] **Finalize the PROVISIONAL pins:** if any A-STEP/A-DIR/A-LIMIT GPIO changed at the bench, update the
      `motion::init` tuples (`main.rs:334–356`), the DOC-00 GPIO manifest, and the header of this doc, then
      drop the "PROVISIONAL" qualifier. Note any setting changes back into the persisted defaults.

> After sign-off, the remaining DOC-10 deferral is the optional modulo-360 rotary **position rollover**
> (DOC-10.6, safe-to-skip — i32 overflow is ~670k revolutions away at the default `$103`) and the Q2
> step-budget tuning for specific cylinder-engraving workloads. Neither is a correctness gate.
