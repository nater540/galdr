---
name: project-spindle-contracts
description: DOC-07 spindle control design decisions — controller reversal API, spin-up gate seam, $392/$393 settings, firmware task/e-stop wiring.
metadata:
  type: project
---

DOC-07 spindle control implemented 2026-06-19 (test-first). Pure logic in `firmware-core/src/spindle.rs` (host-tested);
HAL impls + LEDC bring-up in `firmware/src/spindle.rs`; the async task + signals + consumer wiring in `firmware/src/comms.rs`.

**Two new $-settings (DOC-07):** `$392` spindle_on_delay_s (grbl-aligned spin-up; default 0.0) and `$393`
spindle_reverse_dwell_s (Galdr-specific M3↔M4 spin-down dwell; default 1.5). Both f32, non-negative-OK (0 valid),
`parse_f32_non_negative` in set_command, `non_negative_or` in sanitized, `{:.3}` format, GROUP_SPINDLE. proto tags 58/59
(NOT 53/54 — the proto already had tags through limit_invert=57; the task brief's "use 53/54" was stale). Added one
SETTING_DESCRIPTORS row each + Field::SpindleOnDelayS/SpindleReverseDwellS; descriptor-consistency tests stay green.

**Controller API ([[project-planner-contracts]] sibling):** `SpindleController<P:PwmSink, En:DigitalOut, Dir:DigitalOut>`.
DIR logical `true`=CW(M3); EN logical `true`=run. `apply(state, rpm, rpm_min, rpm_max, reverse_dwell_s) ->
Result<SpindleAction, SpindleError>`. SpindleAction = `Applied` | `SpinDownThenReverse{dwell_s}`. A RUNNING-spindle
direction reversal returns SpinDownThenReverse AFTER the controller has ALREADY forced EN-off+duty-0; the async task awaits
the dwell then calls `complete_reverse(state,rpm,min,max)`. Controller is pure/sync — NEVER blocks on a timer; owns no
Settings (caller passes rpm_min/max/dwell from live Settings). `emergency_stop()` idempotent. rpm==0 OR state==Stop ⇒ stop.
rpm_to_duty: clamp((rpm-min)/(max-min),0,1), `span>0.0` positive-form guard (NaN/≤0 span ⇒ 0). Spin-up delay is NOT the
controller's job.

**Spin-up dwell seam (Stage 5):** host-tested `SpinUpGate` in planner.rs (separate from plan_command, which returns ONE
outcome and can't emit a dwell ahead of a move). `note_spindle(state)` arms on M3/M4, disarms on M5. `take_dwell_before_move
(spin_up_s) -> Option<f32>` consumes the owed spin-up (only first cutting move gets it), returns Some only if $392>0. The
FIRMWARE comms consumer drives it: `ConsumerState.spin_up`, `inject_spin_up_dwell` runs before planning a CUTTING move (G1
Move{rapid:false} or any Arc — G0 rapid is NOT a cut), plans a synthetic PlannerCommand::Dwell (flushes look-ahead) then
awaits a real Timer raced vs SOFT_RESET. NOTE: G4 dwell is otherwise a NO-OP passthrough at firmware level (no dwell-timer
task exists yet), so the spin-up's real delay comes from the consumer's own Timer await, not the executor.

**Firmware wiring (comms.rs):** new `spindle` Embassy task (core 0) is the SOLE driver of the outputs; owns the controller
by &'static mut. Signals: SPINDLE_UPDATE (re-apply from SPINDLE_DIRECTION atomic + override-scaled RPM) set by consumer on
M3/M4/M5 AND by the override handler on spindle-override/stop change; SPINDLE_ESTOP (immediate stop). `commanded_spindle()`
= (SPINDLE_DIRECTION, overrides().scaled_rpm(PROGRAMMED_SPINDLE_RPM)) so a 0x9E spindle-stop zeroes duty. `force_spindle_off()`
= clear direction + signal estop, called from `emit_alarm` (universal alarm chokepoint), `reset_pipeline` (0x18 always stops
spindle, even Idle→Normal that skips emit_alarm), and `handle_sleep` ($SLP). Feed hold `!` deliberately does NOT route
through any of these — spindle keeps running (grblHAL). LEDC: timer0/ch0 GPIO13, 5kHz/13-bit, raw duty via set_duty_hw;
SPIN_EN GPIO14 active-LOW (run=Level::Low); SPIN_DIR GPIO15 straight-through. Ledc/Timer/Channel are a self-referential
'static chain → three StaticCells in main, spindle::init returns sinks holding &'static refs.

**Verified:** host `cargo test` 698 green (firmware-core 431 / galdr-proto 2 / skirnir 265). Xtensa `just build` AND
`just build --features defmt` both link clean. See [[project-settings-contracts]], [[project-consumer-pipeline]],
[[project-protocol-contracts]], [[project-storage-codec-unification]].
