//! Auto-levelling — the height-map mesh and (in later phases) grid acquisition + pre-stream Z correction.
//!
//! This subsystem follows the plan's Parts B and C, all egui-free and host-tested:
//! - [`mesh`] (B1) — the pure [`mesh::Mesh`] model: a grid of probed Z deltas with bilinear interpolation and a
//!   fail-safe out-of-grid lift, mirroring OpenCNCPilot / ioSender's `HeightMap` (MIT). `serde` so it persists
//!   with the profile (B4).
//! - [`segment`] + [`correct`] (C) — the pure whole-program correction pass: [`correct::correct_program`] rewrites
//!   a program's Z to follow the mesh, driven by the shared [`cnc_kinematics::gcode::Parser`] (G1 subdivided, G0
//!   corrected-not-split, arcs kept as real IJK sub-arcs, canonical absolute-mm output, fail-closed hazards).
//!
//! Grid acquisition (B2) and the shell pipeline wiring (C5) land on top of these in later phases.

pub mod acquire;
pub mod correct;
pub mod mesh;
pub mod segment;

pub use acquire::{GridProbeParams, MeshProbeState, MeshProbeStep, grid_points_serpentine, point_probe_lines};
pub use correct::{CorrectionConfig, CorrectionError, correct_program};
pub use mesh::Mesh;
