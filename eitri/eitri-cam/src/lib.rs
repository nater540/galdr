//! `eitri-cam` — CAM operations that turn parsed board geometry into toolpath / operation models.
//!
//! This crate is the heart of Eitri: it consumes the copper `MultiPolygon` from `eitri-gerber` and the drill hits
//! from `eitri-excellon` and produces **toolpath geometry** (isolation rings) and an **operation model** (an
//! ordered, travel-optimized drill plan). It deliberately stops short of G-code — NC emission (drill cycles,
//! multi-depth passes, arcs → `G02`/`G03`) is `eitri-gcode`'s job (Phase 4). See `docs/eitri-porting-plan.md` §7.
//!
//! Phase 3 ships the two highest-value operations:
//! - [`isolation`] — isolation routing: outward-offset copper into concentric cut rings, with milling-direction
//!   winding, pass combining, hole handling, and an arc-preserving output option.
//! - [`drill`] — drilling: group hits by tool and order each group to minimize rapid travel.
//!
//! Both lean on [`optimize`], a self-contained nearest-neighbour + 2-opt/Or-opt travel optimizer behind a trait so
//! a stronger solver can replace it later. Every operation threads an [`eitri_core::ProgressReporter`] and an
//! [`eitri_core::CancelToken`] so large boards stay responsive and interruptible.

#![forbid(unsafe_code)]

pub mod drill;
pub mod isolation;
pub mod optimize;

pub use drill::{DrillConfig, DrillMove, DrillParams, DrillPlan, ToolDrillPlan, plan_drilling};
pub use isolation::{
  IsolationParams, IsolationRing, IsolationToolpaths, MillingDirection, RingPath, isolate, isolate_arc,
};
pub use optimize::{NearestNeighbor, Point, Routed, Stop, TravelOptimizer, TwoOpt, order_stops, tour_travel};
