#![no_std]
#![deny(unsafe_code)]
//! Galdr CNC kinematics core: the pure, `no_std`, host-testable motion pipeline shared by the
//! firmware and the `skirnir` sender.
//!
//! This crate holds the hardware-independent kinematics: the [`gcode`] parser, the motion
//! [`planner`] (bounded look-ahead + junction-deviation cornering), and the [`motion`] step-
//! generation model (trapezoid timing + DDA). The only hardware contract it carries is the small
//! set of step-output traits in [`step`] (`StepSink`/`StepEvent`/`DirState`), which the firmware
//! implements against the RMT peripheral and host tests implement as recorders.
//!
//! It has NO esp-hal dependency, so it compiles and unit-tests on the host with stock Rust. The
//! `firmware-core` crate depends on it and re-exports these modules (and the `step` traits via its
//! own `hal_traits`), so the firmware sees one flat `firmware_core::{gcode,planner,motion}` surface.
//! `skirnir` depends on this crate too (via the host-only `sim` feature) and drives the SAME planner
//! offline for job-time estimation — leaving no second motion model to drift from (see [`sim`] and
//! `docs/skirnir-eta-design.md`).

pub mod gcode;
pub mod motion;
pub mod planner;
pub mod step;

// Host-side offline job-time simulation. Gated behind the `sim` feature because it allocates a per-command
// timeline (`alloc`), which the no-alloc firmware build must not pull in. `skirnir` enables it.
#[cfg(feature = "sim")]
extern crate alloc;
#[cfg(feature = "sim")]
pub mod sim;
