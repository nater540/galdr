//! Hardware abstraction traits (DOC-09).
//!
//! Every hardware interaction in firmware-core is expressed through one of these traits so the
//! planner, parser, and driver logic stay host-testable with recording/mock implementations. Only
//! the `firmware` binary implements them against esp-hal peripherals.
//!
//! Status: trait surface is sketched per DOC-09. Concrete error types and method bodies land with
//! their owning subsystems. The TMC2209 register codec in [`crate::drivers::tmc2209`] does not
//! depend on a `TmcBus` trait; the transport trait is wired up when the `tmc_manager` task is built.

// StepEvent: a single step/direction event emitted to an axis output. Defined with the motion model.
// TODO(DOC-02): define StepEvent (axis, dir, timing) when the RMT step generator is implemented.

// StepSink: sink for bursts of step events; implemented over RMT TX on target, recorded in host tests.
// TODO(DOC-02): pub trait StepSink { fn emit_burst(&mut self, steps: &[StepEvent]) -> Result<(), StepError>; }

// PwmSink: normalized 0.0..=1.0 spindle duty sink (LEDC PWM on target).
// TODO(DOC-05): pub trait PwmSink { fn set_duty(&mut self, frac: f32) -> Result<(), PwmError>; }

// DigitalIn: limit / control digital input (NC limit switches, feed-hold, cycle-start).
// TODO(DOC-06): pub trait DigitalIn { fn is_active(&self) -> bool; }

// DigitalOut: digital output (stepper enable, spindle enable/direction).
// TODO(DOC-05): pub trait DigitalOut { fn set(&mut self, level: bool) -> Result<(), ()>; }

// TmcBus: half-duplex TMC2209 single-wire UART transport; implemented over UART1 on target, byte
// buffer in host tests. The byte-level datagram encode/decode it relies on lives in
// `crate::drivers::tmc2209`.
// TODO(DOC-03): pub trait TmcBus { fn write_reg(&mut self, node: u8, reg: u8, val: u32) -> Result<(), TmcError>;
//                                   fn read_reg(&mut self, node: u8, reg: u8) -> Result<u32, TmcError>; }
