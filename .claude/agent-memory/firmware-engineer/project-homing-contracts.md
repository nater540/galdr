---
name: project-homing-contracts
description: DOC-06 homing/limit subsystem design decisions and resolved open questions — DigitalIn trait, homing state machine reuse of ProbeStepper, $5/$20/$21/$22 settings semantics, ALARM mapping.
metadata:
  type: project
---

DOC-06 homing & limit subsystem (implemented from `docs/homing-research-findings.md` + DOC-06). Pure logic
lives in `firmware-core`; ISR/GPIO/RMT wiring in `crates/firmware/src/motion.rs` + `comms.rs`. Decisions and
resolved open questions a future author needs that are NOT obvious from the code:

**Reuse the probe machinery, do NOT build new step walkers.** Homing seek/locate reuse the EXACT
`ProbeStepper` pattern (`crates/firmware-core/src/motion.rs`): one tick per `emit_burst`, sample a
`FnMut()->bool` stop predicate BEFORE each tick, stop on edge, return steps_emitted. The firmware bin's
`run_probe` (motion.rs ~587) is the structural analog the homing executor mirrors. Position sync is
`Planner::sync_position(steps)` (planner.rs ~306); `StepCounter` (motion.rs ~443) tracks live MPos.

**`DigitalIn` mirrors `ProbeInput` exactly.** Trait exposes ONLY a raw level read (`is_high()`); the `$5`
invert is applied by a host-tested free fn (`limit_triggered(raw_high, &LimitConfig)`) mirroring
`probe_triggered`/`ProbeConfig`. NC fail-safe: untriggered pin LOW (switch closed to GND), triggered/broken
wire HIGH. `$5=1` inverts. `#![deny(unsafe_code)]` stays in firmware-core.

**`$5` limit-invert setting did NOT exist — added.** Settings model had no `$5` field before DOC-06.

**`$21`/`$22` are grblHAL EXCLUSIVE BITMASKS, NOT bools (research divergence section).** The settings model
typed `soft_limits_enable/hard_limits_enable/homing_enable` as `bool` — flag for fix. `$22` bits: b0 enable,
b3 force-set-origin-to-0 (HOMING_FORCE_SET_ORIGIN). `$21` bits: b0 enable hard limits, b1 strict. `$20`
(soft limits) MUST be rejected unless `$22` enabled (`Status_SoftLimitError` analog) — cross-field check in
`set_command`, not `parse_and_apply` (which sees one field).

**Resolved open questions (from homing-research-findings.md bottom):**
- Q4 ($5 ↔ rising-edge): NC wiring → triggered reads HIGH in BOTH the homing-seek sampling path AND the
  hard-limit ISR (rising-edge IRQ). `$5` inverts the LOGICAL trigger sense uniformly; the host-tested
  `limit_triggered` is the single place it is applied, identical to probe.
- Q5 (ALARM:11): confirmed `AlarmCode::HomingRequired` = code 11, name "Homing required", NOT locked (so
  `$X` clears it via `ControlState::unlock`); `ControlState::boot(homing_enabled)` already enters it.
  `$H` success needs a NEW `ControlState` transition (home_complete → Normal). Hard limit = `AlarmCode::HardLimit`
  (1, locked), soft limit = `AlarmCode::SoftLimit` (2, locked). All already in protocol.rs.

**Shared-limit-pin rule (research finding #17, refuted claim is NOT the mechanism):** during
`MachineState::Home` the limit ISR must NOT raise a hard-limit alarm; re-arm after. Per-axis independent stop
on Galdr's independent RMT channels (NOT Bresenham) — stop just the latched axis's channel, co-movers run on.

**Machine-coord reconciliation:** Galdr's soft-limit envelope is `[-max_travel, 0]` (machine coords ≤ 0,
`SoftLimits` + `soft_limit_violation` in planner.rs ~828, already wired into `plan_jog`). Homing sets machine
zero at the home-switch (positive/top-right) end; travel goes negative. `$22` bit-3 force-origin sets zero;
else per-axis from `$23` mask + pulloff.

**Cycle order:** Z first (clear tool), then X+Y together. Per-axis: seek `$25` → pull-off `$27` → locate
`$24` → final pull-off. Fail if no contact within 1.5× `$130–$132` → homing-fail alarm + reset.

**IMPLEMENTED 2026-06-17 (host-tested + Xtensa-compile-verified, NOT hardware-verified):**
- firmware-core `homing.rs`: `home_axis` (4-phase single-axis primitive reusing `ProbeStepper`), `HomingConfig`
  (built via `Settings::homing_config(tick_hz)`), `machine_zero_steps`, `HOMING_GROUPS` (`[[Z],[X,Y]]`),
  `hard_limit_alarm` (pure shared-pin-rule decision), `HomingError`. 18 homing tests.
- `StepCounter::sync_to` (set live pos to homing zero). `ControlState::homing_allowed`/`home_complete` (protocol).
  `AlarmCode::HomingFail` = grbl ALARM:8 (recoverable, not locked). `PlannerError::MoveExceedsTravel` +
  `Planner::plan_command_with_limits(cmd, Option<SoftLimits>)` (program-move soft-limit gate → ALARM:2).
- firmware bin: homing runs ON CORE 1 (owns RMT channels + limit inputs), dispatched like a probe via
  `HOME_REQUEST`/`HOME_RESULT` signals; `run_homing` in motion.rs; `RmtLimitInput`(`DigitalIn`)+`init_limits`
  (GPIO10/11/12 pull-up). `handle_home` rewritten (gate→`<Home>` push→dispatch→sync planner+clear ALARM:11+ok /
  ALARM:8+reset on fail). `HOMING_ACTIVE` atomic overrides `?`→`Home` AND gates off hard limits. Hard-limit
  check (`check_hard_limits`) at BLOCK BOUNDARY via `HARD_LIMIT_TRIPPED`→ALARM:1 (locked)+reset; mirrors
  `HARD_LIMITS_ENABLED`/`LIMIT_INVERT`. Soft limits gated on new `HOMED` atomic (set on `$H`, cleared on
  reset/boot when homing enabled).
- **X+Y home SEQUENTIALLY (not concurrent)** — simple+valid; concurrent co-motion is a speed/squaring refinement
  (explicit `// TODO(DOC-06): concurrent intra-group homing` in `run_homing`: needs a new firmware-core primitive
  interleaving axes' single-tick bursts; not contained — `home_axis` owns one sink, runs 4 phases linearly).

**REVIEW FIXES 2026-06-18 (9 findings, host-tested + Xtensa-compile-verified, NOT hardware-verified):**
- **Limit IRQ now WIRED (was the dead-signal gap).** The core-1 executor's idle `select` awaits real limit rising
  edges via `RmtLimitInput::wait_for_rising_edge` (esp-hal `Input::wait_for_rising_edge`, interrupt-driven, no
  hand ISR) in `wait_for_limit_trip` (motion.rs), runs the `$26` debounce resample (`Timer::after` + re-read +
  `limit_triggered` confirm), SIGNALS `LIMIT_TRIGGERED` on a confirmed trip, then `check_hard_limits`. Pins stay
  single-owned on core 1 (also used for homing-seek level reads), so the monitor lives in the executor itself, NOT
  a separate core-0 task. New `$26` mirror `LIMIT_DEBOUNCE_MS` (+ `limit_debounce_ms()`), seeded at the same 4
  sites as `LIMIT_INVERT`; `init_limit_settings` now takes `debounce_ms`. Remaining HW-gated: edge/pull-up/EMI
  electrical behavior only (not the wiring).
- **`$H` success arm guards on the real transition (finding #2):** only sync planner + set `HOMED=true` + `ack()`
  when `control_state().home_complete() == ControlState::Normal`; if the state was clobbered to an alarm in the
  post-cycle window, leave it locked and emit NO `ok`. `home_complete` from any locked alarm round-trips unchanged
  (protocol.rs test strengthened to cover all locked codes).
- **Homing math saturates (finding #3):** `homing.rs` `mm_to_steps_safe(mm, steps_per_mm)` single-sources the
  `roundf`+`is_finite`+positive-fallback policy AND caps at `i32::MAX as u32` so `as i32`/negation never overflow;
  `machine_zero_steps` negative-home uses `saturating_add`. (finding #7 = same helper extraction.)
- **Shared placeholder Block ctor (finding #4):** `Block::placeholder(steps)` in planner.rs (derives dominant
  count); used by both homing `single_axis_block` and probe `probe_block_to`.
- Homing publishes live MPos at PHASE boundaries not per single-step burst (finding #5): `CountingSink::live` vs
  `::quiet`; `run_homing` uses `quiet` + `publish_live_position` after each axis. `check_hard_limits` builds
  `raw_high` via `core::array::from_fn` (finding #6). `HOMED` baseline via `store_homed_baseline(homing_enabled)`
  helper at boot+reset (finding #8).
- **HARDWARE-BOUNDARY (compile-checked only, need HW):** limit edge/pull-up/EMI electrical behavior + broken-wire
  fail-safe + all RMT/GPIO/seek timing. Verify on HW: `$H` Z-then-XY order, pull-off, fail-on-no-trigger, `$5` NC
  sense + broken-wire, hard-limit halt (idle edge + block-boundary), `$26` debounce, soft-limit ALARM:2. NOTE: a
  STEADY-high limit at idle entry won't fire `wait_for_rising_edge` (edge, not level) — block-boundary sampling +
  the next edge cover it; matches finding #14's rising-edge-IRQ intent.

See [[project-motion-contracts]], [[project-planner-contracts]], [[project-protocol-contracts]],
[[project-settings-contracts]], [[project-consumer-pipeline]].
