# Homing & Limit-Switch Research Findings (DOC-06 background)

> Deep-research synthesis backing the DOC-06 implementation. Scoped to **mechanical normally-closed (NC)
> limit switches only**. Out of scope: StallGuard4/sensorless, NPN/opto stages, auto-squaring, spindle.
> Confidence tags are the verifier-panel result (3-0 = unanimous confirm). Where a finding is a *synthesis*
> (grbl design fact mapped onto Galdr's stated architecture) it is marked **medium / synthesis** — treat
> those as design recommendations, not documented grbl behavior.

## Primary sources
- grbl v1.1 Configuration / Interface wikis, `settings.md`, `commands.md` (gnea/grbl) — primary.
- grblHAL `core` wiki (Report-extensions, Additional/extended settings) + `errors.c` — primary.
- DeepWiki grbl 4.1 homing-cycle — secondary, every quote re-confirmed against actual `limits.c` /
  `motion_control.c` / `config.h`, so effectively primary.

---

## A. grblHAL behavioral contract (fidelity)

1. **Status during `$H` (3-0).** grblHAL pushes **one `<Home|...>` status report before the cycle starts**,
   then **queues and does not answer** further `?` requests until homing completes — the sender sees no live
   DRO motion. State field is first; `Home` is a valid state. The `$H` line terminates with a normal `ok`.
   → Galdr: emit `MachineState::Home` once at cycle entry; hold/queue `?` while homing (do not compute live
   MPos mid-seek); set MPos to the home position only at completion.

2. **Default cycle order (3-0).** `HOMING_CYCLE_0 = 1<<Z` (Z+ first to clear), then
   `HOMING_CYCLE_1 = (1<<X)|(1<<Y)` (X+Y together, positive). Matches DOC-06. `$23` dir-invert mask reverses
   per-axis approach; `$23=0` homes toward positive (top-right, spindle-up).

3. **Per-axis phases + defaults (3-0).** Fast **seek** at `$25` (default **500 mm/min**) until switch trips →
   **pull-off** reverse → slow **locate** re-approach at `$24` (grbl default **25 mm/min**; Galdr's
   `DEFAULT_HOMING_FEED_MM_MIN` is **100**) for precise zero → **final `$27` pull-off** (default **1.0 mm**) clear
   of the switch. `$26` debounce (grbl default **250 ms**; Galdr's `DEFAULT_HOMING_DEBOUNCE_MS` is **25 ms**, in the
   5–25 ms band that is typically fine). Locate repeats `N_HOMING_LOCATE_CYCLE` times; total pass count is **odd** so the
   cycle always ends on a pull-off (off the switch).

4. **`$27` pull-off vs hard-limit re-trigger (3-0).** `$27` exists specifically to move **off all limit
   switches** after the cycle so shared switches don't immediately re-raise a hard limit; also applied between
   phases (back off after each trigger before re-approach, gated by `$26`).

5. **Post-homing machine zero (3-0).** Set to **0** if `HOMING_FORCE_SET_ORIGIN` (grblHAL: `$22` **bit 3**),
   else per-axis from `$23` mask + `$130–$132` max travel + `$27` pull-off
   (`lround((max_travel+pulloff)*steps_per_mm)` homing positive, else `-pulloff*steps_per_mm`). Then
   `gc_sync_position()` + `plan_sync_position()`. **WCO unchanged** across homing (`WPos = MPos - WCO`);
   G28/G30 are stored in machine coords and only meaningful after homed zero.

6. **`$H` vs `$X` (3-0).** `$H` is the **only** command that runs the cycle and establishes position. `$X`
   kills the alarm lock to re-enable G-code **without** homing or re-establishing position (e.g. jog off a
   switch). Homing-enabled boot enters alarm because position is unknown → grblHAL `Status_HomingRequired`
   ("Home machine to continue."), cleared by `$H` or overridden by `$X`. Maps to Galdr `ALARM:11`.

7. **Homing-fail (3-0).** No switch contact within `HOMING_AXIS_SEARCH_SCALAR` (**1.5×**) the axis max travel
   (`$130–$132`) → `EXEC_ALARM_HOMING_FAIL_APPROACH` + `mc_reset()`. In legacy grbl the scalar is a
   compile-time constant; this is the seek-distance bound and the no-trigger abort path.

### grblHAL divergences from legacy grbl
- **`$22` is an exclusive bitmask** (legacy: boolean): bit0 enable homing (gating), bit1 single-axis homing
  cmds, bit2 homing-on-startup required, bit3 set origin to 0, bit4 two switches share one pin, bit5 allow
  manual homing of non-auto axes, bit6 override locks (reset clears startup alarm), bit7 keep homed on reset.
- **`$21` is also a bitmask**: bit0 enable hard limits, bit1 strict mode (limits also checked on `$X`; still
  engaged → `error 45`). `$21=1` hard limits only, `$21=3` + strict.
- `$20` soft limits **rejected unless `$22` enabled** → `Status_SoftLimitError`.
- Pre-homing `<Home>` report push is grblHAL-specific.

---

## B. Implementation on RMT bursts + Embassy (medium / synthesis)

8. **Stop-on-edge via the probe path.** Reuse the existing G38.x `run_probe` shape: walk **one tick per
   burst** at seek/locate rate, sample `DigitalIn` **between bursts**, stop on the edge, sync commanded
   position to the stop point. Worst-case overshoot per detection ≈ one burst of travel
   (steps-per-burst × mm-per-step) — bounded, which is exactly why the slow `$24` locate pass exists.

9. **No controlled decel at trigger.** grbl stops as soon as the trigger is detected (segment/step-granular
   hard stop); the fast `$25` seek is deliberately imprecise and the slow `$24` locate establishes the
   repeatable zero. → Galpr: seek tolerates overshoot; **locate pass must run slow enough that per-burst
   overshoot is below target repeatability**.

10. **Dual-axis (X+Y) coordination.** grbl masks each axis's step bits out of the shared train via
    `sys.homing_axis_lock` the instant that axis latches (faster axis holds while the slower finishes). On
    Galdr's **independent RMT channels** this is cleaner: each axis has its own channel + its own `DigitalIn`;
    stop just that channel's burst emission when its switch latches, leave the others running. **Per-axis
    independent seek, not Bresenham** — axes need only be co-active, not path-synchronized.

11. **Task placement.** `homing` lives on **Core 0** (PRO_CPU) with comms/parser/planner; Core 1 stays
    motion_executor-only. Drive seek/locate as bounded one-tick-per-burst moves with a **Signal-driven stop**
    (mirroring the probe path), not just the bulk `BlockQueue`. Homing is a dedicated cycle, distinct from
    normal block-stream motion.

12. **`DigitalIn` trait.** Mirror `ProbeInput`/`StepSink`. Expose a **level read** (`is_triggered()`/`read()`)
    for between-burst sampling and the debounce resample; the wiring layer turns the ISR edge into an
    `embassy-sync` `Signal`. A **mock `DigitalIn`** (scriptable trigger at a chosen burst/position) +
    recording `StepSink` makes the whole state machine host-testable. Untestable at the boundary: real GPIO
    edge/IRQ timing, pull-up electrical behavior, EMI rejection, RMT timing, the physical broken-wire fail-safe.

---

## C. Limit ISR, debounce, hard/soft limits

13. **Wiring / `$5` (3-0).** Default limit pins are normally-HIGH via pull-up; LOW = triggered; `$5=1`
    inverts. grbl documents NO-to-ground. **Galdr is NC**: untriggered = pin LOW (switch closed to GND);
    opening the switch **or a broken wire** lets the pull-up pull HIGH = triggered → **fail-safe**. `$5` is
    the per-pin sense control aligning firmware "triggered" polarity with NC wiring; grblHAL adds per-pin `$5`
    masking. (See open question on the exact `$5`↔rising-edge mapping.)

14. **ISR + debounce (medium / synthesis).** Rising-edge IRQ on GPIO10/11/12 with pull-ups; ISR sets a
    `LIMIT_TRIGGERED` `Signal` (no work in the ISR). Debounce = resample after `$26` (`Timer::after`),
    confirm level persists to reject EMI glitches. NC + pull-up gives broken-wire-reads-triggered.

15. **Hard limits `$21` (3-0).** Trigger during motion → immediately halt all motion, shut down
    coolant/spindle, enter ALARM. Abrupt stop ⇒ **likely lost steps** = `ALARM:1` ("Machine position is
    likely lost…"). Recovery: reset → `$X` unlock → **re-home strongly recommended**.

16. **Soft limits `$20` (2-1, both halves individually primary-sourced).** Require `$22` **and** accurate
    `$130–$132`. grblHAL **rejects enabling `$20` without `$22`** (`Status_SoftLimitError`). Runtime violation
    → immediate feed hold, spindle/coolant off, system alarm (`ALARM:2`). Envelope pre-check belongs in the
    **parser/planner pipeline before a move is queued**, and is only meaningful once homed.
    → Galdr: (a) reject `$20` enable unless `$22` set; (b) gate soft-limit checks on the homed state.

17. **Shared limit-pin during homing (3-0).** Two mechanisms: (1) **hard limits DISABLED during the cycle**,
    re-armed after (`limits_disable()` before, `limits_init()` after — clears the pin-change IRQ). (2)
    **axis-lock mask** holds each axis the moment its own switch latches while co-movers continue
    (`axislock &= ~step_pin[idx]`). → Galdr: **do not let the limit ISR raise a hard-limit alarm while
    `MachineState::Home` is active**, and stop each RMT channel independently as its axis latches.

> **Refuted (0-3) — do NOT rely on:** "Only homing allowed when a limit switch is engaged"
> (`Status_LimitsEngaged`) is **not** the shared-pin recovery mechanism. The real mechanism is
> limits-disabled-during-homing + axis-lock (finding 17).

---

## D. Best practices & validation

18. **Bring-up.** When switches aren't wired yet, grblHAL boots in ALARM expecting NC-high; satisfy it by
    jumpering each limit GPIO to GND or temporarily `$5=1`. Tune `$26` debounce (start 25 ms) and `$27`
    pull-off for repeatability; final zero comes from the slow `$24` locate, not the fast `$25` seek.

19. **Host-test strategy.** Unit-test the full homing state machine in `firmware-core` with a **recording
    `StepSink` + mock `DigitalIn`** (scriptable trigger): phase sequencing, per-axis axis-lock stop, pull-off,
    fail-on-no-trigger, position-set math. No esp-hal. Boundary stays untestable: GPIO/IRQ timing, pull-up
    electrical behavior, EMI rejection, RMT burst timing, the physical broken-wire fail-safe.

---

## Open questions (must resolve during implementation)
1. **Quantitative overshoot:** for Galdr's `steps_per_mm` and chosen burst length, what max burst size keeps
   the `$24` locate-pass per-burst overshoot below target repeatability (e.g. < 0.01 mm)? Compute from the
   mechanical config — not in grbl docs.
2. **grblHAL stop mechanics:** hard-stop vs short controlled decel at trigger, and whether it differs between
   seek and locate. Legacy grbl segment-granular stop confirmed; exact grblHAL behavior not pinned.
3. **`$26` runtime vs compile-time:** later grblHAL trended toward compile-time debounce. Galdr's settings
   model already lists `$26` — confirm runtime tuning is intended.
4. **`$5` ↔ rising-edge mapping:** verify "switch opens / wire breaks = triggered HIGH" is interpreted as a
   trigger in **both** the homing-seek sampling path and the hard-limit ISR path, against Galdr's `$5` impl.
5. **`ALARM:11` string/code:** Galdr naming mapped onto grblHAL `Alarm_HomingRequired`/`Status_HomingRequired`
   — confirm the exact code/string in Galdr's protocol module.
