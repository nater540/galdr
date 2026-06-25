//! End-to-end probe-cycle integration test.
//!
//! This crosses the `cnc_kinematics` probe stepper + step counter with `firmware-core`'s coordinate
//! model and `[PRB:]` formatter — the exact pipeline the firmware bin wires for a G38 probe → Z-zero.
//! It lives here (rather than in `cnc_kinematics::motion`'s unit tests) because it depends on
//! `firmware_core::{coords, protocol}`, which sit a layer above the shared kinematics crate.

use firmware_core::coords::CoordinateSystems;
use firmware_core::hal_traits::{probe_triggered, DirState, ProbeConfig, StepError, StepEvent, StepSink};
use firmware_core::motion::{steps_to_mm, MotionConfig, ProbeStepper, StepCounter};
use firmware_core::planner::Block;

/// A 1 MHz / 1 µs tick config with a 10 µs pulse and 2 µs minimum LOW (matches `motion`'s unit-test config).
fn test_config() -> MotionConfig {
  MotionConfig { tick_hz: 1_000_000.0, step_pulse_ticks: 10, min_low_ticks: 2 }
}

#[test]
fn probe_cycle_latches_machine_position_and_renders_prb_and_wpos() {
  // The end-to-end simulated probe → Z-zero workflow, crossing the probe stepper, the live step counter, the
  // `[PRB:]` formatter, and the coordinate model — the exact pipeline the firmware bin wires, but host-tested.

  // A probe block straight down Z by 5 mm at 100 steps/mm, starting at machine Z = 0 (no prior moves). The
  // touch plate (with `$6=1`, NO-plate) trips after 4.2 mm of travel → 420 Z steps, i.e. machine Z = -4.2 mm.
  let cfg = test_config();
  let prober = ProbeStepper::new(cfg);
  // `Block::placeholder` is the public fixed-period constructor for exactly this case (G38 probe / homing seek):
  // the probe stepper walks at a caller-supplied period and ignores the speed/mm/unit-vec fields, so a 500-step
  // -Z block is all the geometry the cycle needs.
  let block = Block::placeholder([0, 0, -500, 0]);

  // A scripted mock probe: idles high (untouched), goes low after 420 steps of travel. Under the base sense
  // (`$6=0`) a high pin reads not-triggered and a low pin (grounded by contact) reads triggered. `steps_taken`
  // is a shared `Cell` so the counting sink and the probe predicate can both touch it without an aliasing borrow.
  let probe_cfg = ProbeConfig { invert: false, pullup_disable: false };
  let steps_taken = std::cell::Cell::new(0u32);
  let raw_high = std::cell::Cell::new(true);
  let is_at_edge = || {
    // The plate trips (goes low) once 420 steps have been emitted.
    if steps_taken.get() >= 420 {
      raw_high.set(false);
    }
    probe_triggered(raw_high.get(), &probe_cfg)
  };

  // Advance a live step counter through the same ticks the prober emits, exactly as the firmware's CountingSink
  // does, so the latched position is derived from the emitted steps.
  let mut counter = StepCounter::new();
  counter.set_direction(DirState { dir: [block.steps[0] >= 0, block.steps[1] >= 0, block.steps[2] >= 0, block.steps[3] >= 0] });

  struct CountingRecorder<'a> {
    counter: &'a mut StepCounter,
    steps_taken: &'a std::cell::Cell<u32>,
  }
  impl StepSink for CountingRecorder<'_> {
    fn set_direction(&mut self, _dir: DirState) -> Result<(), StepError> {
      Ok(())
    }
    fn emit_burst(&mut self, ticks: &[StepEvent]) -> Result<(), StepError> {
      for ev in ticks {
        self.counter.advance(ev);
        self.steps_taken.set(self.steps_taken.get() + 1);
      }
      Ok(())
    }
  }
  let mut sink = CountingRecorder { counter: &mut counter, steps_taken: &steps_taken };
  let outcome = prober.run_probe(&block, 1000, is_at_edge, &mut sink).expect("probe runs");

  assert!(outcome.triggered, "the NO plate tripped within travel");
  // The latched machine position: 420 Z steps in the negative direction → Z = -4.2 mm at 100 steps/mm.
  let stop_steps = counter.position_steps();
  assert_eq!(stop_steps, [0, 0, -420, 0], "latched at the trigger step");
  let steps_per_mm = [100.0, 100.0, 100.0, 0.0];
  let probe_mm = steps_to_mm(&stop_steps, &steps_per_mm);
  assert!((probe_mm[2] + 4.2).abs() < 1e-4, "probe machine Z is -4.2 mm, got {}", probe_mm[2]);

  // The immediate `[PRB:]` push reports the triggered machine position with flag 1.
  let mut prb = String::new();
  {
    let mut s = heapless::String::<64>::new();
    firmware_core::protocol::ResponseWriter::probe_report(&mut s, &probe_mm, outcome.triggered).expect("prb");
    prb.push_str(s.as_str());
  }
  assert_eq!(prb, "[PRB:0.000,0.000,-4.200,0.000:1]\r\n");

  // Z-zero: the plate is 1.0 mm thick, so `G10 L20 P1 Z1.0` makes the probed point read work Z = 1.0, putting
  // the copper top (1 mm below the plate top the probe touched) at WPos Z = 0.
  let mut cs = CoordinateSystems::new();
  cs.set_wcs_offset_to_position(0, probe_mm, [0.0, 0.0, 1.0, 0.0], [false, false, true, false]);
  let copper_top = [probe_mm[0], probe_mm[1], probe_mm[2] - 1.0, 0.0];
  let wpos = cs.machine_to_work(copper_top);
  assert!(wpos[2].abs() < 1e-4, "copper top reads WPos Z = 0, got {}", wpos[2]);
}
