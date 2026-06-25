#![no_std]
#![deny(unsafe_code)]
//! Galdr firmware-core: pure-logic, `no_std`, host-testable CNC control library.
//!
//! This crate holds all hardware-independent firmware logic — GCode parsing, the grblHAL
//! streaming protocol, the motion planner, the step-generation motion model, and the TMC2209
//! driver register codec. It has NO dependency on esp-hal, so it compiles and unit-tests on the
//! host with stock Rust. All hardware access is expressed through the traits in [`hal_traits`];
//! only the `firmware` binary wiring layer implements those traits against esp-hal peripherals.
//!
//! Module status as of this pass: [`drivers::tmc2209`] (CRC8-ATM + datagram codec) is implemented
//! and host-tested. The remaining subsystems are documented stubs awaiting their DOC-referenced
//! implementations.

// The GCode parser, motion planner, and step-generation motion model live in the shared
// `cnc-kinematics` crate (the same pure logic the `skirnir` sender is intended to drive offline for
// job-time estimation, leaving no second motion model to drift from). Re-export them flat so the rest of
// firmware-core and the firmware bin keep their `firmware_core::{gcode,planner,motion}` paths.
pub use cnc_kinematics::{gcode, motion, planner};

pub mod coolant;
pub mod coords;
pub mod drivers;
pub mod hal_traits;
pub mod homing;
pub mod protocol;
pub mod settings;
pub mod spindle;
pub mod storage_frame;
