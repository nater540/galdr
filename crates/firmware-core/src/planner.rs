//! Motion planner & kinematics (DOC-05).
//!
//! Mirrors grbl's look-ahead planner. The planner consumes [`crate::gcode::PlannerCommand`]s emitted
//! by the parser (DOC-04) and turns motion commands into [`Block`]s in a fixed-capacity ring buffer.
//! For each block it computes only the optimal *entry speed* via a reverse pass (cap each junction
//! entry speed by the maximum reachable decelerating into the next block) and a forward pass (cap
//! each exit by what is reachable accelerating from the entry). The full trapezoidal velocity profile
//! (accel-only / decel-only / cruise / full-trapezoid) is realized DOWNSTREAM by the segment
//! generator in [`crate::motion`] (DOC-02); the planner never materializes it here.
//!
//! ## What the planner owns (the parser deliberately did none of this, DOC-04/DOC-05)
//! - **Unit conversion** (G20 inch / G21 mm): inch words are scaled by [`MM_PER_INCH`].
//! - **Distance mode** (G90 absolute / G91 incremental): incremental words add to the current
//!   position; absolute words replace it.
//! - **Coordinate offsets** (G92): a work offset in mm is applied so commanded positions map onto
//!   machine positions.
//! - **Steps/mm conversion** (`$100–$102`): the machine target in mm becomes a target in steps via
//!   `round(pos_mm × steps_per_mm)`. The planner works in step space; the unit vector and travel in
//!   mm are derived back from the step delta so cornering and ramp math stay in physical units.
//! - **Arc subdivision** (G2/G3): an arc is chord-tolerance subdivided (`$12`) into short linear
//!   blocks, each fed through the same linear path as a move.
//!
//! ## Squared speeds
//! Like grbl, the planner stores every speed as its square (mm/s)². The kinematic relation
//! `v_exit² = v_entry² + 2·a·d` is then evaluated with no `sqrt` in the hot reverse/forward passes;
//! `sqrt` is paid only once, lazily, when a caller asks for an actual speed via [`Block::entry_speed`]
//! or [`Block::nominal_speed`]. Junction cornering needs a single `sqrt` per junction.
//!
//! ## Allocation
//! `#![no_std]`, allocation-free. The block ring buffer is a [`heapless::Deque`] of fixed capacity
//! [`BLOCK_QUEUE_LEN`]; an over-full queue is a recoverable [`PlannerError::QueueFull`], never a
//! panic. All arithmetic is `f32`; `libm` supplies `sqrtf`, `acosf`, `sinf`, `cosf`.

use crate::gcode::{AxisWords, CoordinateOp, DistanceMode, FeedMode, JogCommand, PlannerCommand, ProbeKind, Units};
use heapless::Deque;

/// Number of axes the planner coordinates: X, Y, Z (linear, mm) and A (rotary about X, degrees) per DOC-10.
/// Hardcoded at 4 — the firmware is not generalized to arbitrary axis counts; index 3 is always the A axis.
pub const AXES: usize = 4;

/// Axis index of the rotary A axis (rotation about machine X). The linear axes are indices 0..3 (X, Y, Z).
pub const A_AXIS: usize = 3;

/// DEFAULT `$376` rotary-axes bitmask — the fresh power-on value before flash loads (DOC-10.7). Bit N set ⇒
/// axis N is angular (degrees). The authoritative value is the runtime `$376` setting in
/// [`PlannerConfig::rotary_mask`]; this const is only the default. `8` = bit 3 set → A rotary, X/Y/Z linear.
pub const DEFAULT_ROTARY_MASK: u8 = 0b0000_1000;

/// Block ring-buffer capacity. DOC-05 recommends 16–32 blocks of look-ahead for PCB milling and notes
/// the ESP32-S3 can comfortably hold 32 (grbl uses ~16 on AVR). 32 maximizes look-ahead headroom.
pub const BLOCK_QUEUE_LEN: usize = 32;

/// Millimeters per inch, the exact G20→mm conversion factor.
pub const MM_PER_INCH: f32 = 25.4;

/// A tiny epsilon used to reject zero-length moves and guard divisions. Any block whose travel is
/// below this (in mm) carries no motion and is dropped rather than enqueued.
const LENGTH_EPSILON_MM: f32 = 1.0e-6;

/// Errors the planner can return. All are recoverable: the caller (the firmware planner task) decides
/// whether to back-pressure, report an `error:N`, or alarm. The planner never panics on bad input.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PlannerError {
  /// The block ring buffer is full ([`BLOCK_QUEUE_LEN`] blocks queued). The caller should retry after
  /// the motion executor drains a block; this is the natural back-pressure point for look-ahead.
  QueueFull,
  /// An arc was malformed: neither I nor J was given, or the computed radius/sweep is degenerate so no
  /// valid circular geometry exists. Maps to grblHAL `error:33` (invalid motion/arc geometry).
  InvalidArc,
  /// A `$J=` jog target exceeded the machine travel envelope while `$20` soft limits are enabled (DOC-08 Phase
  /// D). grbl's `error:15` — "Jog target exceeds machine travel. Command ignored." The jog is rejected BEFORE
  /// any motion or block enqueue, so a soft-limited jog never moves the machine toward the limit.
  JogExceedsTravel,
  /// A PROGRAM move/arc target exceeded the machine travel envelope while `$20` soft limits are enabled and the
  /// machine is homed (DOC-06). Unlike a jog (which grbl simply rejects with `error:15`), a program soft-limit
  /// violation is a SYSTEM ALARM: grbl halts and raises `ALARM:2`. The block is rejected BEFORE any enqueue, so
  /// no motion toward the limit ever starts; the consumer maps this to `ALARM:2` ([`AlarmCode::SoftLimit`]).
  MoveExceedsTravel,
}

impl PlannerError {
  /// The grblHAL `error:N` status code that best represents this error, for protocol responses.
  pub fn code(self) -> u8 {
    match self {
      // No dedicated grblHAL code covers "look-ahead buffer momentarily full"; it is a flow-control
      // condition rather than a program error, so it borrows code 1 only if surfaced. In practice the
      // planner task back-pressures instead of emitting this to the host.
      PlannerError::QueueFull => 1,
      PlannerError::InvalidArc => 33,
      // grbl's "Travel exceeded" jog rejection code; the jog command is ignored and the host sees error:15.
      PlannerError::JogExceedsTravel => 15,
      // A program move soft-limit violation is reported as a SYSTEM ALARM (`ALARM:2`), not an `error:N` line, so
      // this code is a placeholder reused from the jog path; the consumer routes this variant to the alarm, not
      // to an `error:N` response.
      PlannerError::MoveExceedsTravel => 15,
    }
  }
}

/// The `$20`/`$130–$132` soft-limit envelope a jog target is checked against (DOC-08 Phase D). grbl homes to
/// machine zero at the positive end of each axis and treats the work volume as the closed interval
/// `[-max_travel, 0]` per axis (machine coordinates are ≤ 0). A jog whose resolved MACHINE target leaves that
/// interval on any axis is rejected with [`PlannerError::JogExceedsTravel`] before any motion. Held as a small
/// `Copy` value the consumer builds from the live `$20`/`$130–$132` settings and passes into [`Planner::plan_jog`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SoftLimits {
  /// `$130–$132` maximum travel per axis in mm, `[X, Y, Z]`. The envelope is `[-max_travel, 0]` per axis.
  pub max_travel_mm: [f32; AXES],
}

/// The machine settings the planner needs to resolve geometry and kinematics. These mirror the
/// grblHAL `$`-settings (DOC-04 lists them) but are held as a plain struct so the planner stays
/// host-constructible and flash-free; the firmware binary loads them from `esp-storage` and hands a
/// `PlannerConfig` in. All per-axis arrays are indexed `[X, Y, Z]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlannerConfig {
  /// `$100–$102` steps per millimeter for each axis (includes the microstepping multiplier).
  pub steps_per_mm: [f32; AXES],
  /// `$110–$112` maximum rate in mm/min for each axis. The block's nominal speed is clamped so no
  /// participating axis exceeds its own maximum rate.
  pub max_rate_mm_min: [f32; AXES],
  /// `$120–$122` acceleration in mm/s² for each axis. The block's acceleration along its direction of
  /// travel is the most restrictive axis acceleration (grbl's per-axis acceleration limiting).
  pub accel_mm_s2: [f32; AXES],
  /// `$11` junction deviation in mm: the allowed centripetal deviation used to derive the maximum
  /// cornering speed at each junction. grbl default ≈ 0.01 mm.
  pub junction_deviation_mm: f32,
  /// `$12` arc chord tolerance in mm: the maximum chord error when subdividing an arc into linear
  /// segments. grbl default ≈ 0.002 mm.
  pub arc_tolerance_mm: f32,
  /// `$376` rotary-axes bitmask (DOC-10.7): bit N set ⇒ axis N is angular (degrees). Drives the rotary
  /// kinematic gating (units inch-scaling suppression, continuous/rollover soft limits) at the rotary-gated
  /// sites; the unit-agnostic look-ahead/DDA never consults it. Default [`DEFAULT_ROTARY_MASK`] (= 8).
  pub rotary_mask: u8,
}

impl PlannerConfig {
  /// Whether axis `axis` is rotary per the live `$376` mask. The rotary-gated sites call this instead of
  /// indexing a const, so toggling `$376` re-classifies an axis at runtime (DOC-10.1).
  pub fn is_rotary(&self, axis: usize) -> bool {
    self.rotary_mask & (1 << axis) != 0
  }
}

impl Default for PlannerConfig {
  /// grbl-like defaults useful for tests and first boot. Linear X/Y/Z: 250 steps/mm, 500 mm/min, 10 mm/s².
  /// Rotary A (index 3, degrees): 8.889 steps/deg (200 × 16 microsteps / 360), 3600 deg/min (10 rev/min),
  /// 360 deg/s². `$11` = 0.01 mm, `$12` = 0.002 mm, `$376` = [`DEFAULT_ROTARY_MASK`] (A rotary).
  fn default() -> Self {
    PlannerConfig {
      steps_per_mm: [250.0, 250.0, 250.0, 8.889],
      max_rate_mm_min: [500.0, 500.0, 500.0, 3600.0],
      accel_mm_s2: [10.0, 10.0, 10.0, 360.0],
      junction_deviation_mm: 0.01,
      arc_tolerance_mm: 0.002,
      rotary_mask: DEFAULT_ROTARY_MASK,
    }
  }
}

/// One planned motion block: a straight-line move in step space with the kinematics the segment
/// generator needs. Speeds are stored squared (mm/s)²; use the accessors for real speeds. Fields
/// mirror DOC-05's list: target step counts per axis, the dominant-axis step count, the unit
/// direction vector, nominal/entry/max-entry speed, acceleration, and travel in mm.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Block {
  /// Signed step delta per axis for this block (`target_steps - start_steps`). The sign is the step
  /// direction; the magnitude is the per-axis step count.
  pub steps: [i32; AXES],
  /// Dominant-axis step count: `max(|steps[axis]|)`. The DDA dominant axis advances every tick and
  /// drives Bresenham coordination downstream (DOC-02). Always ≥ 1 for an enqueued block.
  pub step_event_count: u32,
  /// Unit direction vector in mm space (length 1). Used for junction cornering between blocks.
  pub unit_vec: [f32; AXES],
  /// Straight-line travel of this block in millimeters (Euclidean length of the mm delta).
  pub millimeters: f32,
  /// Acceleration along the block's direction of travel, mm/s² (the most restrictive axis accel).
  pub acceleration: f32,
  /// Nominal (cruise) speed squared, (mm/s)². The target speed when nothing limits the block: the
  /// requested feed for G1/arcs, or the rapid limit for G0, clamped to per-axis max rates.
  pub nominal_speed_sq: f32,
  /// Maximum allowable entry speed squared, (mm/s)². The lesser of the junction cornering limit with
  /// the previous block and this block's own nominal speed. The reverse pass never exceeds this.
  pub max_entry_speed_sq: f32,
  /// Planned entry speed squared, (mm/s)². Computed by the reverse/forward passes; this is the only
  /// value the planner solves for. The segment generator derives the trapezoid from it downstream.
  pub entry_speed_sq: f32,
  /// True for a G0 rapid block (speed governed by max rates, not a feed word); false for a feed move.
  pub rapid: bool,
  /// True when this block was enqueued by a `$J=` jog (DOC-08 Phase D). Tagged so a jog-cancel (`0x85`) can
  /// identify and flush ONLY the queued jog blocks via [`Planner::flush_jog_blocks`], leaving any program
  /// blocks untouched. A normal program block (move/arc/probe-derived) is always `false`.
  pub jog: bool,
}

impl Block {
  /// The planned entry speed in mm/s (lazy `sqrt` of the stored squared value).
  pub fn entry_speed(&self) -> f32 {
    libm::sqrtf(self.entry_speed_sq)
  }

  /// The nominal (cruise) speed in mm/s (lazy `sqrt` of the stored squared value).
  pub fn nominal_speed(&self) -> f32 {
    libm::sqrtf(self.nominal_speed_sq)
  }

  /// The maximum allowable entry speed in mm/s (lazy `sqrt` of the stored squared value).
  pub fn max_entry_speed(&self) -> f32 {
    libm::sqrtf(self.max_entry_speed_sq)
  }

  /// Build a fixed-period [`Block`] from explicit per-axis step deltas, for the paths that walk a move at a
  /// caller-supplied period through the [`ProbeStepper`](crate::motion::ProbeStepper) rather than realizing a
  /// planned trapezoid — the `G38.x` probe and the `$H` homing seek/locate/pull-off. Only the step deltas, the
  /// dominant `step_event_count`, and the per-axis signs matter to that stepper (it advances at a fixed period
  /// with exact-integer Bresenham and ignores the speed / mm / unit-vector fields), so the trapezoid fields are
  /// set to benign NON-degenerate placeholders. Centralizing them here keeps that "placeholder trapezoid"
  /// contract in ONE place instead of duplicated at each fixed-period call site. `step_event_count` is derived
  /// as the dominant axis magnitude, so callers cannot desync it from `steps`.
  pub fn placeholder(steps: [i32; AXES]) -> Self {
    let mut step_event_count = 0u32;
    for delta in steps {
      step_event_count = step_event_count.max(delta.unsigned_abs());
    }
    Block {
      steps,
      step_event_count,
      // The unit vector / mm length / speeds are unused by the fixed-period stepper; benign placeholders.
      unit_vec: [0.0; AXES],
      millimeters: 1.0,
      acceleration: 1.0,
      nominal_speed_sq: 1.0,
      max_entry_speed_sq: 1.0,
      entry_speed_sq: 0.0,
      rapid: false,
      jog: false,
    }
  }
}

/// The result of feeding one [`PlannerCommand`] to the planner. Motion commands enqueue blocks and
/// report [`PlannerOutcome::Queued`]; non-motion commands are passed through so the caller can act on
/// them (start a dwell timer, drive the spindle, run homing) while the planner still observes them to
/// flush look-ahead at the correct boundary.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PlannerOutcome {
  /// `n` linear blocks were enqueued (1 for a move, ≥1 for a subdivided arc). Zero means the command
  /// was a no-op move (target equals current position, no axis travelled).
  Queued { blocks: usize },
  /// A G4 dwell for `seconds`. The preceding block has been pinned to a full stop so the dwell starts
  /// from rest, as grbl requires (a dwell is a synchronized motion boundary).
  Dwell { seconds: f32 },
  /// An M3/M4/M5 spindle command. The caller drives the spindle (DOC-07); the planner does not.
  Spindle(crate::gcode::SpindleState, f32),
  /// G28/G30 go-to-predefined. Surfaced for the caller to execute as system motion; the look-ahead is
  /// flushed (the preceding block stops) because predefined moves are not blended with the program.
  GoToPredefined { is_g28: bool },
  /// M30 program end. The look-ahead is flushed to a stop; the caller resets modal/program state.
  ProgramEnd,
  /// A coordinate-system / offset op (G10, G54-G59, G92, G28.1/G30.1, G43.1/G49) passed through for the caller
  /// to apply to the shared [`crate::coords::CoordinateSystems`]. No motion is produced and look-ahead is
  /// preserved (these do not move the machine), matching grbl; the caller then pushes the recomputed WCO back
  /// into the planner via [`Planner::set_work_offset`].
  Coordinate(CoordinateOp),
  /// A `G38.x` probe move (DOC-09). The planner has resolved the work-coordinate axis words into an absolute
  /// MACHINE step `target` and flushed look-ahead (a probe is a synchronized boundary, so it starts from rest).
  /// The planner does NOT enqueue a normal block for it: the firmware bin runs a distinct, probe-watching
  /// execution path that stops on the probe edge, then syncs the planner's commanded position to the actual stop
  /// point via [`Planner::sync_position`] (grbl sets `gc_state.position` to the probe stop). `kind` carries the
  /// toward/away + alarm-on-fail semantics and `feed` the seek speed.
  Probe {
    /// The probe mode (toward/away, alarm-on-fail) from `G38.2`/`.3`/`.4`/`.5`.
    kind: ProbeKind,
    /// The absolute MACHINE step target the probe seeks toward (work words already resolved through the WCO /
    /// distance mode). The probe stops early on the expected edge; this is the no-contact end of travel.
    target: [i32; AXES],
    /// The probe seek feed in the program's active units per minute (the firmware converts to a step rate).
    feed: f32,
    /// The active units for `feed` (the firmware bin scales inch/min → mm/min before deriving the step rate).
    units: Units,
  },
}

/// The spin-up-dwell decision (DOC-07): after an M3/M4 the planner must insert a synchronized dwell of `$392`
/// (`spindle_on_delay_s`) before the FIRST cutting move that follows, so the spindle reaches speed before it
/// cuts. This is pure, host-tested decision logic kept SEPARATE from [`Planner::plan_command`] because that call
/// returns exactly ONE [`PlannerOutcome`] and cannot emit a dwell ahead of a move in a single step; instead the
/// firmware comms consumer — which already sees both the [`PlannerOutcome::Spindle`] and the live `$392` setting —
/// drives this gate: it `note_spindle`s every M3/M4/M5 it forwards, and before planning a cutting move it
/// consults [`take_dwell_before_move`](SpinUpGate::take_dwell_before_move) and, if a dwell is owed, plans a
/// synthetic [`PlannerCommand::Dwell`] first. The gate keeps the WHEN-to-insert decision unit-tested in
/// firmware-core; the consumer supplies the dwell SECONDS from settings, so the planner stays settings-free.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SpinUpGate {
  /// True when an M3/M4 has started the spindle and the spin-up dwell is still OWED to the next cutting move. A
  /// stop (M5/S0) or the consumption of the dwell clears it, so exactly one dwell is inserted per spindle start.
  pending: bool,
}

impl SpinUpGate {
  /// A fresh gate with no spin-up owed (the spindle is off at program start / after a reset).
  pub fn new() -> Self {
    SpinUpGate { pending: false }
  }

  /// Observe a spindle command. An M3/M4 (`Clockwise`/`CounterClockwise`) arms the spin-up so the next cutting
  /// move gets a dwell ahead of it; an M5 (`Stop`) disarms it (a stopped spindle owes no spin-up). Re-arming on a
  /// second M3/M4 before any move is harmless: the gate is a single boolean, so a back-to-back M3 then M4 still
  /// owes exactly one dwell.
  pub fn note_spindle(&mut self, state: crate::gcode::SpindleState) {
    self.pending = !matches!(state, crate::gcode::SpindleState::Stop);
  }

  /// Called just before a cutting move (G1/G2/G3 — a feed move, not a rapid) is planned. If a spin-up is owed it
  /// is CONSUMED (so only the first move after the spindle start gets it) and the dwell to insert is returned:
  /// `Some(spin_up_delay_s)` when `$392 > 0`, or `None` when `$392 == 0` (the spin-up is still consumed, but
  /// there is nothing to insert). When no spin-up is owed, returns `None` and leaves the gate untouched.
  ///
  /// `spin_up_delay_s` is the live `$392` value the caller reads from settings, so the planner owns no settings.
  pub fn take_dwell_before_move(&mut self, spin_up_delay_s: f32) -> Option<f32> {
    if !self.pending {
      return None;
    }
    self.pending = false;
    // A positive, finite delay is inserted as a dwell; `$392 == 0` (or a non-finite value) consumes the spin-up
    // but inserts nothing. Positive form avoids the `neg_cmp_op_on_partial_ord` lint and treats NaN as "none".
    if spin_up_delay_s > 0.0 { Some(spin_up_delay_s) } else { None }
  }

  /// True when a spin-up dwell is currently owed to the next cutting move (the spindle was started and no move has
  /// consumed the dwell yet). Exposed for the consumer's diagnostics / tests; the decision uses the `take`/`note`
  /// methods.
  pub fn is_pending(&self) -> bool {
    self.pending
  }
}

/// The geometry of one G2/G3 arc, bundled so the arc planner takes a single borrowed request rather
/// than a long positional argument list. The fields mirror [`PlannerCommand::Arc`].
struct ArcRequest<'a> {
  /// True for G2 clockwise, false for G3 counter-clockwise.
  cw: bool,
  /// Arc endpoint axis words (XY endpoint, optional Z for a helix).
  axes: &'a AxisWords,
  /// I center offset (X) relative to the start point, in the active units, if present.
  i: Option<f32>,
  /// J center offset (Y) relative to the start point, in the active units, if present.
  j: Option<f32>,
  /// Active units for the endpoint/offset/feed words.
  units: Units,
  /// Active distance mode for the endpoint words.
  distance: DistanceMode,
  /// Active feed rate (modal F). Under G94 it is `units` per minute, shared by every subdivided segment; under
  /// G93 it is the inverse-time feed for the WHOLE arc, distributed across the equal-length segments in [`plan_arc`].
  feed: f32,
  /// Active feed-rate mode (modal group 5): G94 units/min or G93 inverse-time (DOC-10.2).
  feed_mode: FeedMode,
  /// True for a `G53` machine-coordinate arc: the endpoint words are MACHINE positions (no work offset).
  machine_coords: bool,
}

/// The motion planner. Holds machine settings, the block ring buffer, the current machine position in
/// steps, the current G92 work offset in mm, and the trailing junction state (previous block's unit
/// vector and nominal speed) used to compute the next junction's cornering limit.
///
/// The planner is pure synchronous logic with no runtime dependency; the firmware binary drives it
/// from the planner task and forwards [`PlannerOutcome`]s to the motion executor and spindle/dwell
/// handlers.
pub struct Planner {
  config: PlannerConfig,
  queue: Deque<Block, BLOCK_QUEUE_LEN>,
  /// Current machine position in steps per axis. Targets are resolved relative to this and it is
  /// advanced as blocks are planned, so look-ahead chains from the program's commanded position.
  position_steps: [i32; AXES],
  /// The active Work Coordinate Offset (WCO) in mm per axis, added to commanded WORK coordinates to get
  /// MACHINE coordinates (`MPos = WPos + WCO`). The full grbl WCO — `G54..59[active] + G92 + TLO` — is computed
  /// by [`crate::coords::CoordinateSystems`] in the consumer, which pushes it here via [`set_work_offset`] on
  /// every coordinate-system change. The planner applies it only to ABSOLUTE work moves; an incremental move
  /// (offset already baked into the current machine position) and a G53 machine-coordinate move bypass it.
  work_offset_mm: [f32; AXES],
  /// Unit direction vector of the most recently planned block, for the next junction's cornering. The
  /// zero vector marks "no previous block" (start of program or after a flush): entry speed is 0.
  prev_unit_vec: [f32; AXES],
  /// Nominal speed squared of the most recently planned block, (mm/s)². Caps the junction speed so a
  /// corner never exceeds either adjoining block's cruise speed.
  prev_nominal_speed_sq: f32,
  /// True once the executor has popped a block and committed to the current queue head as the block it
  /// will enter next (grbl's "busy block"). While set, [`recalculate`](Planner::recalculate)'s reverse
  /// pass must not rewrite the head block's `entry_speed_sq`: the executor already read it as the popped
  /// block's exit speed and shaped that block's deceleration to it, so changing it now would create a
  /// velocity discontinuity / missed steps at the junction. Cleared when the queue drains empty.
  head_busy: bool,
}

impl Planner {
  /// Create a planner with the given settings, the machine homed to step position zero, no work
  /// offset, and no previous block (so the first move starts and ends at rest).
  pub fn new(config: PlannerConfig) -> Self {
    Planner {
      config,
      queue: Deque::new(),
      position_steps: [0; AXES],
      work_offset_mm: [0.0; AXES],
      prev_unit_vec: [0.0; AXES],
      prev_nominal_speed_sq: 0.0,
      head_busy: false,
    }
  }

  /// The current machine position in steps per axis. Exposed for status reporting (MPos) and tests.
  pub fn position_steps(&self) -> [i32; AXES] {
    self.position_steps
  }

  /// Force the planner's commanded machine position (in steps) to `steps`, used after a `G38.x` probe to set the
  /// commanded position to the ACTUAL stop point the executor latched (grbl sets `gc_state.position` to the probe
  /// stop). This also drops the trailing junction state so the next move corners from rest at the new position —
  /// correct because a probe is a synchronized boundary and the machine has just decelerated to a stop there. The
  /// queue is left untouched (the probe enqueued no normal block; look-ahead was already flushed when it ran).
  pub fn sync_position(&mut self, steps: [i32; AXES]) {
    self.position_steps = steps;
    self.prev_unit_vec = [0.0; AXES];
    self.prev_nominal_speed_sq = 0.0;
  }

  /// The current machine position in mm per axis, derived from the step position and `$100–$102`.
  pub fn position_mm(&self) -> [f32; AXES] {
    let mut out = [0.0; AXES];
    for (axis, slot) in out.iter_mut().enumerate() {
      *slot = self.position_steps[axis] as f32 / self.config.steps_per_mm[axis];
    }
    out
  }

  /// Set the active Work Coordinate Offset (WCO) in mm per axis. The consumer calls this after applying any
  /// coordinate-system change (G10 / G54-G59 / G92 / G43.1 / G49) to the shared
  /// [`crate::coords::CoordinateSystems`], so the planner's absolute work→machine transform always uses the
  /// live WCO. It does NOT move the machine or touch look-ahead — only the offset future absolute moves resolve
  /// against changes. Non-finite components are ignored per axis so a degenerate offset cannot poison geometry.
  pub fn set_work_offset(&mut self, wco: [f32; AXES]) {
    for (slot, &value) in self.work_offset_mm.iter_mut().zip(wco.iter()) {
      if value.is_finite() {
        *slot = value;
      }
    }
  }

  /// The active Work Coordinate Offset in mm per axis (for tests / diagnostics).
  pub fn work_offset(&self) -> [f32; AXES] {
    self.work_offset_mm
  }

  /// The number of blocks currently queued for the motion executor.
  pub fn queued_len(&self) -> usize {
    self.queue.len()
  }

  /// True when the block ring buffer holds no blocks.
  pub fn is_empty(&self) -> bool {
    self.queue.is_empty()
  }

  /// Pop the oldest planned block for the motion executor (FIFO). Popping a block hands ownership of the
  /// realized motion to the segment generator; the planner's trailing junction state is unchanged because
  /// look-ahead is computed across the still-queued blocks.
  ///
  /// Popping also arms grbl's busy-block protection: the executor reads the *new* head as the popped
  /// block's exit speed (via [`peek_block`](Planner::peek_block)) and shapes that block's deceleration to
  /// it, so the new head's planned entry is now committed. A subsequent [`recalculate`](Planner::recalculate)
  /// (triggered by enqueuing more motion behind it) must not rewrite that committed entry — see
  /// [`head_busy`](Planner::head_busy). The flag clears when the queue drains so a fresh program re-optimizes
  /// its head freely.
  pub fn pop_block(&mut self) -> Option<Block> {
    let block = self.queue.pop_front()?;
    // A block remaining after the pop is the committed next block to execute → freeze its entry. An emptied
    // queue commits nothing, so the next head (whenever it arrives) is freely optimizable again.
    self.head_busy = !self.queue.is_empty();
    Some(block)
  }

  /// Peek the oldest queued block without removing it (for the executor's planning glance).
  pub fn peek_block(&self) -> Option<&Block> {
    self.queue.front()
  }

  /// Feed one parser command to the planner. Motion commands resolve geometry, build block(s), enqueue
  /// them, and run the reverse/forward look-ahead passes; non-motion commands flush look-ahead where
  /// grbl requires and pass through as a [`PlannerOutcome`]. Returns [`PlannerError::QueueFull`] if a
  /// block cannot be enqueued (back-pressure) or [`PlannerError::InvalidArc`] for bad arc geometry.
  pub fn plan_command(&mut self, command: &PlannerCommand) -> Result<PlannerOutcome, PlannerError> {
    self.plan_command_with_limits(command, None)
  }

  /// Plan a [`PlannerCommand`] with an optional `$20` soft-limit envelope (DOC-06). When `limits` is `Some` (i.e.
  /// `$20` is enabled AND the machine is homed — the consumer supplies it only then), a `Move`/`Arc` whose
  /// resolved MACHINE endpoint leaves the `[-max_travel, 0]` envelope is rejected with
  /// [`PlannerError::MoveExceedsTravel`] BEFORE any block is enqueued, so no motion toward the limit ever starts;
  /// the consumer maps that to `ALARM:2`. With `limits = None` this is identical to [`plan_command`]. Only the
  /// motion commands (`Move`/`Arc`) are envelope-checked — a dwell/spindle/coordinate op moves nothing, and a
  /// `G38.x` probe deliberately drives toward a switch (its travel is bounded by the probe cycle, not soft
  /// limits). An arc is checked at its ENDPOINT here; full mid-arc envelope checking is a later refinement
  /// (a PCB-milling arc that starts and ends inside the envelope effectively never bulges outside it).
  pub fn plan_command_with_limits(
    &mut self,
    command: &PlannerCommand,
    limits: Option<SoftLimits>,
  ) -> Result<PlannerOutcome, PlannerError> {
    match command {
      PlannerCommand::Move { rapid, axes, units, distance, feed, feed_mode, machine_coords } => {
        let target = self.resolve_target(axes, *units, *distance, *machine_coords);
        if let Some(limits) = limits
          && soft_limit_violation(&target, &self.config.steps_per_mm, &limits.max_travel_mm, self.config.rotary_mask)
        {
          return Err(PlannerError::MoveExceedsTravel);
        }
        let queued = self.plan_line(target, *feed, *units, *feed_mode, *rapid)?;
        Ok(PlannerOutcome::Queued { blocks: queued })
      }
      PlannerCommand::Arc { cw, axes, i, j, units, distance, feed, feed_mode, machine_coords } => {
        // Check the arc ENDPOINT against the envelope (the start is wherever the machine already is, already
        // inside the envelope by induction). A degenerate arc still surfaces its `InvalidArc` below.
        if let Some(limits) = limits
          && soft_limit_violation(
            &self.resolve_target(axes, *units, *distance, *machine_coords),
            &self.config.steps_per_mm,
            &limits.max_travel_mm,
            self.config.rotary_mask,
          )
        {
          return Err(PlannerError::MoveExceedsTravel);
        }
        let request = ArcRequest {
          cw: *cw,
          axes,
          i: *i,
          j: *j,
          units: *units,
          distance: *distance,
          feed: *feed,
          feed_mode: *feed_mode,
          machine_coords: *machine_coords,
        };
        let queued = self.plan_arc(&request)?;
        Ok(PlannerOutcome::Queued { blocks: queued })
      }
      PlannerCommand::Probe { kind, axes, units, distance, feed } => {
        // Resolve the work-coordinate probe target to an absolute MACHINE step target exactly as a move does (a
        // probe is never a G53 machine-coordinate move, so `machine_coords` is false). Flush look-ahead so the
        // probe starts from rest — it is a synchronized boundary, like a dwell. The firmware bin runs the actual
        // probe-watching motion and syncs the position to the stop point via [`sync_position`].
        let target = self.resolve_target(axes, *units, *distance, false);
        self.flush_lookahead();
        Ok(PlannerOutcome::Probe { kind: *kind, target, feed: *feed, units: *units })
      }
      PlannerCommand::Dwell { seconds } => {
        self.flush_lookahead();
        Ok(PlannerOutcome::Dwell { seconds: *seconds })
      }
      PlannerCommand::Spindle { state, speed } => Ok(PlannerOutcome::Spindle(*state, *speed)),
      PlannerCommand::GoToPredefined { is_g28, .. } => {
        self.flush_lookahead();
        Ok(PlannerOutcome::GoToPredefined { is_g28: *is_g28 })
      }
      PlannerCommand::Coordinate(op) => {
        // A coordinate-system op does not move the machine; pass it through for the consumer to apply to the
        // shared coordinate model. Look-ahead is preserved (grbl does not flush on G10/G92/G54-59/G43.1).
        Ok(PlannerOutcome::Coordinate(*op))
      }
      PlannerCommand::ProgramEnd => {
        self.flush_lookahead();
        Ok(PlannerOutcome::ProgramEnd)
      }
    }
  }

  /// Plan a `G28`/`G30` predefined-position recall (DOC-05 group-0 motion) into up to two RAPID blocks, returning
  /// the number of blocks actually enqueued. The stored `predefined` position is supplied by the consumer (it owns
  /// the coordinate model — the planner does not) and is ALWAYS an absolute MACHINE position in mm, on every axis
  /// including the rotary A. The grbl-faithful sequence is:
  ///
  /// 1. If ANY `intermediate` axis word is present, FIRST a rapid to that intermediate point — the words resolved in
  ///    the ACTIVE work coordinate system honoring `units` (G20/G21) and `distance` (G90/G91), with unspecified
  ///    axes holding their current position (exactly the [`resolve_target`](Self::resolve_target) work-move path,
  ///    `machine_coords = false`). With no axis words this first move is skipped entirely (one block at most).
  /// 2. THEN a rapid to the stored predefined MACHINE position — absolute, in mm, bypassing the WCO (like a `G53`
  ///    move). All axes recall, so a rotary A returns to its stored angle too.
  ///
  /// Each sub-move's resolved MACHINE endpoint is soft-limit checked (when `limits` is `Some`) BEFORE anything is
  /// enqueued, identical to any rapid: a violation returns [`PlannerError::MoveExceedsTravel`] and enqueues
  /// nothing. The predefined position is within-envelope by construction; only the intermediate can violate. A
  /// zero-length sub-move (target equals the current position) enqueues no block — so `G28`/`G30` already at the
  /// stored position with no intermediate words returns `Ok(0)` and the consumer simply `ok`s with no motion.
  ///
  /// Back-pressure is ALL-OR-NOTHING (like the arc path): both targets are resolved up front (pure, no mutation),
  /// the exact number of blocks needed is computed (0 or 1 per move — a no-op move where target == the projected
  /// position needs none; each rapid is a single block, no subdivision), and if the free queue capacity cannot
  /// hold that count the call returns [`PlannerError::QueueFull`] BEFORE enqueuing anything. This is essential for
  /// the consumer's retry to be safe in INCREMENTAL (G91) mode: were the intermediate enqueued and only the recall
  /// to fail, the planner position would have advanced and a retry's `resolve_target` would add the G91 increment
  /// to the ALREADY-advanced position — double motion. Pre-checking capacity means a `QueueFull` retry always
  /// re-resolves from the original, un-advanced position, so the intermediate is applied exactly once.
  pub fn plan_go_to_predefined(
    &mut self,
    intermediate: &AxisWords,
    units: Units,
    distance: DistanceMode,
    predefined: [f32; AXES],
    limits: Option<SoftLimits>,
  ) -> Result<usize, PlannerError> {
    let from = self.position_steps;
    // Resolve the optional intermediate target (work coords, honoring units + distance) WITHOUT mutating. Only
    // present when at least one axis word is given — a bare `G28`/`G30` recalls directly with no intermediate.
    let has_words =
      intermediate.x.is_some() || intermediate.y.is_some() || intermediate.z.is_some() || intermediate.a.is_some();
    let inter_target = has_words.then(|| self.resolve_target(intermediate, units, distance, false));
    // Resolve the recall target (pure): the stored position is already an absolute MACHINE position in mm on every
    // axis, so it converts straight to steps — no WCO, no unit scaling, no rotary fork (degrees and mm alike are
    // stored as the model's native value). It is planned FROM the intermediate point when one exists, else `from`.
    let mut recall = [0i32; AXES];
    for axis in 0..AXES {
      recall[axis] = mm_to_steps(predefined[axis], self.config.steps_per_mm[axis]);
    }
    // Soft-limit check BEFORE any enqueue (a violation moves nothing). The intermediate is a work move that can
    // leave the envelope; the recall point is within-envelope by construction but is checked symmetrically.
    if let Some(limits) = limits {
      if let Some(target) = inter_target
        && soft_limit_violation(&target, &self.config.steps_per_mm, &limits.max_travel_mm, self.config.rotary_mask)
      {
        return Err(PlannerError::MoveExceedsTravel);
      }
      if soft_limit_violation(&recall, &self.config.steps_per_mm, &limits.max_travel_mm, self.config.rotary_mask) {
        return Err(PlannerError::MoveExceedsTravel);
      }
    }
    // Count the blocks actually needed, chaining the projected position so a no-op move (target == projected) costs
    // nothing — exactly mirroring `build_block`'s zero-length skip (`step_event_count == 0` ⇔ target == from for
    // integer step targets). All-or-nothing capacity pre-check: if both blocks won't fit, enqueue NEITHER so a
    // retry restarts from the un-advanced position (the G91 idempotency guarantee above).
    let mut projected = from;
    let mut needed = 0usize;
    if let Some(target) = inter_target {
      if target != projected {
        needed += 1;
        projected = target;
      }
    }
    if recall != projected {
      needed += 1;
    }
    if self.queued_len() + needed > BLOCK_QUEUE_LEN {
      return Err(PlannerError::QueueFull);
    }
    // Capacity is reserved; the enqueues below cannot hit `QueueFull`. Built as rapids (`feed` is ignored for G0).
    let mut blocks = 0;
    if let Some(target) = inter_target {
      blocks += self.plan_line(target, 0.0, units, FeedMode::UnitsPerMin, true)?;
    }
    blocks += self.plan_line(recall, 0.0, units, FeedMode::UnitsPerMin, true)?;
    Ok(blocks)
  }

  /// Plan a `$J=` jog (DOC-08 Phase D) into one cancelable [`jog`](Block::jog)-tagged block. The jog target is
  /// resolved through the SAME work→machine path as a move (honoring units, distance mode, and — for `G53` — the
  /// machine-coordinate bypass of the WCO), so a jog blends with the program's coordinate frame. When `limits`
  /// is `Some` (i.e. `$20` soft limits are enabled) the resolved MACHINE target is checked against the
  /// `[-max_travel, 0]` envelope FIRST and a violating jog is rejected with [`PlannerError::JogExceedsTravel`]
  /// before any block is built — so a soft-limited jog never moves the machine toward the limit. A jog runs at
  /// its own `F` feed (clamped to per-axis max rates exactly like a feed move), not the modal feed.
  ///
  /// On success the block is enqueued, tagged `jog = true` so [`flush_jog_blocks`](Planner::flush_jog_blocks) can
  /// drain it on a jog-cancel, and look-ahead is recalculated. A zero-length jog (target equals the current
  /// position) enqueues nothing and returns `Queued { blocks: 0 }`.
  pub fn plan_jog(&mut self, jog: &JogCommand, limits: Option<SoftLimits>) -> Result<PlannerOutcome, PlannerError> {
    let target = self.resolve_target(&jog.axes, jog.units, jog.distance_mode, jog.machine_coords);
    if let Some(limits) = limits
      && soft_limit_violation(&target, &self.config.steps_per_mm, &limits.max_travel_mm, self.config.rotary_mask)
    {
      return Err(PlannerError::JogExceedsTravel);
    }
    // A jog is a feed move (its `F` governs speed, not the rapid max-rate path), tagged `jog = true` so a
    // jog-cancel can flush exactly the jog blocks. A jog feed is always units/min — the jog grammar accepts no
    // G93/G94 word — so it plans as `FeedMode::UnitsPerMin`. Run look-ahead immediately like a single move.
    let enqueued = self.enqueue_move(target, jog.feed, jog.units, FeedMode::UnitsPerMin, false, true)?;
    if enqueued == 1 {
      self.recalculate();
    }
    Ok(PlannerOutcome::Queued { blocks: enqueued })
  }

  /// Drain the trailing `$J=` jog blocks from the queue on a jog-cancel (`0x85`), returning how many were
  /// removed. ONLY [`jog`](Block::jog)-tagged blocks are flushed, and only from the BACK of the queue, so a
  /// program block can never be dropped: in normal operation a jog never shares the queue with program motion
  /// (the consumer accepts a jog only from Idle/Jog), so the whole queue is jog blocks — but draining from the
  /// back and stopping at the first non-jog block is defensive against any future interleaving. The trailing
  /// junction state is reset so the next planned move starts from rest at the (about-to-be-synced) stop point,
  /// matching the executor decelerating the active jog block to a stop at its boundary.
  pub fn flush_jog_blocks(&mut self) -> usize {
    let mut flushed = 0;
    while matches!(self.queue.back(), Some(block) if block.jog) {
      self.queue.pop_back();
      flushed += 1;
    }
    // The active (front) jog block keeps executing to its boundary; dropping the trailing junction state means a
    // post-cancel move corners from rest. `head_busy` is left as-is: if the front block is still in flight the
    // executor's committed entry must not be disturbed, and an emptied queue clears it on the next pop anyway.
    self.prev_unit_vec = [0.0; AXES];
    self.prev_nominal_speed_sq = 0.0;
    flushed
  }

  /// Resolve a line's axis words into an absolute machine target in *steps*, applying units, distance mode, and
  /// — for an ABSOLUTE work move — the active Work Coordinate Offset (WCO). A `machine_coords` (G53) move treats
  /// the words as MACHINE positions and skips the offset; an incremental move adds to the current machine
  /// position (the offset is already baked in). Axes not mentioned on the line keep their current position.
  fn resolve_target(&self, axes: &AxisWords, units: Units, distance: DistanceMode, machine_coords: bool) -> [i32; AXES] {
    let words = [axes.x, axes.y, axes.z, axes.a];
    let mut target = self.position_steps;
    for axis in 0..AXES {
      if let Some(value) = words[axis] {
        // G20/G21 inch scaling applies to LINEAR axes only; a rotary axis word (per the live `$376` mask) is
        // never inch-scaled — a `G20 A90` is 90 degrees, not 90 × 25.4 (DOC-10.1). The same mask that marks an
        // axis rotary thus suppresses its unit scaling, so the rotary fork lives entirely in this `scale` choice.
        let scale = if self.config.is_rotary(axis) { 1.0 } else { units_scale(units) };
        let value_mm = value * scale;
        let machine_mm = match distance {
          // Absolute words are MACHINE coordinates under G53 (no offset), else WORK coordinates (add the WCO).
          DistanceMode::Absolute if machine_coords => value_mm,
          DistanceMode::Absolute => value_mm + self.work_offset_mm[axis],
          // Incremental words add to the current machine position; the offset is already baked in (G53 has no
          // effect on an incremental move, matching grbl — the delta is identical either way).
          DistanceMode::Incremental => {
            self.position_steps[axis] as f32 / self.config.steps_per_mm[axis] + value_mm
          }
        };
        target[axis] = mm_to_steps(machine_mm, self.config.steps_per_mm[axis]);
      }
    }
    target
  }

  /// Plan a single straight-line move to an absolute step target. Builds the block, enqueues it, runs
  /// look-ahead, and advances the planner position. Returns 1 if a block was enqueued, 0 for a no-op
  /// move (target equals current position).
  fn plan_line(
    &mut self,
    target: [i32; AXES],
    feed: f32,
    units: Units,
    feed_mode: FeedMode,
    rapid: bool,
  ) -> Result<usize, PlannerError> {
    let enqueued = self.enqueue_move(target, feed, units, feed_mode, rapid, false)?;
    if enqueued == 1 {
      // A single move runs the full look-ahead immediately, so its planned entry speeds are final the
      // moment the command returns (the arc path defers this to one recalculate after the whole sweep).
      self.recalculate();
    }
    Ok(enqueued)
  }

  /// Build and enqueue one straight-line move WITHOUT running look-ahead, advancing the per-segment
  /// position and junction state (`prev_unit_vec`/`prev_nominal_speed_sq`) so a following segment corners
  /// against this one correctly. Returns 1 if a block was enqueued, 0 for a no-op move. Separated from the
  /// `recalculate()` pass so an arc can enqueue all its segments first and recalculate exactly once — the
  /// per-block reverse+forward passes are O(n), so calling them per segment makes arc planning O(n²).
  fn enqueue_move(
    &mut self,
    target: [i32; AXES],
    feed: f32,
    units: Units,
    feed_mode: FeedMode,
    rapid: bool,
    jog: bool,
  ) -> Result<usize, PlannerError> {
    let block = match self.build_block(target, feed, units, feed_mode, rapid, jog) {
      Some(block) => block,
      None => return Ok(0),
    };
    self.enqueue(block)?;
    self.position_steps = target;
    self.prev_unit_vec = block.unit_vec;
    self.prev_nominal_speed_sq = block.nominal_speed_sq;
    Ok(1)
  }

  /// Build a block from the current position to an absolute step `target`. Returns `None` for a
  /// zero-length move. Computes the step delta, dominant-axis count, unit vector and mm travel, the
  /// limiting acceleration and nominal speed, and the junction-deviation entry-speed cap.
  fn build_block(
    &self,
    target: [i32; AXES],
    feed: f32,
    units: Units,
    feed_mode: FeedMode,
    rapid: bool,
    jog: bool,
  ) -> Option<Block> {
    let mut steps = [0i32; AXES];
    let mut delta_mm = [0.0f32; AXES];
    let mut step_event_count = 0u32;
    for axis in 0..AXES {
      let d = target[axis] - self.position_steps[axis];
      steps[axis] = d;
      step_event_count = step_event_count.max(d.unsigned_abs());
      delta_mm[axis] = d as f32 / self.config.steps_per_mm[axis];
    }
    // The single block length is the FULL all-axis Euclidean norm (degrees treated as mm per the grblHAL
    // convention), used identically for ramp distance, the direction unit vector, and junction cornering — there
    // is one length quantity, never a separate linear-only path length (DOC-10.2 review correction / grbl
    // `ROTARY_FIX`). A pure-rotary move's norm is just |Δa|, which is exactly what the G93 rotary-only feed wants.
    let mut len_sq = 0.0f32;
    for axis in 0..AXES {
      len_sq += delta_mm[axis] * delta_mm[axis];
    }
    let millimeters = libm::sqrtf(len_sq);
    if millimeters < LENGTH_EPSILON_MM || step_event_count == 0 {
      return None;
    }
    let inv_mm = 1.0 / millimeters;
    let mut unit_vec = [0.0f32; AXES];
    for axis in 0..AXES {
      unit_vec[axis] = delta_mm[axis] * inv_mm;
    }

    let acceleration = limiting_acceleration(&unit_vec, &self.config.accel_mm_s2);
    let nominal_speed = self.nominal_speed_mm_s(feed, units, feed_mode, rapid, millimeters, &unit_vec);
    let nominal_speed_sq = nominal_speed * nominal_speed;

    // The junction cornering limit caps the entry speed; it never exceeds this block's own nominal.
    let junction_speed_sq = self.junction_speed_sq(&unit_vec, acceleration);
    let max_entry_speed_sq = junction_speed_sq.min(nominal_speed_sq);

    Some(Block {
      steps,
      step_event_count,
      unit_vec,
      millimeters,
      acceleration,
      nominal_speed_sq,
      max_entry_speed_sq,
      // Seeded to the cap; the reverse/forward passes lower it as the chain requires.
      entry_speed_sq: max_entry_speed_sq,
      rapid,
      jog,
    })
  }

  /// The block's nominal (cruise) speed in mm/s. For a rapid (G0) the speed is governed by the per-axis
  /// maximum rates; for a feed move it is derived from the requested feed, clamped so no participating axis
  /// exceeds its own maximum rate along the unit vector.
  ///
  /// The feed interpretation forks on `feed_mode` (DOC-10.2):
  /// - **G94 units/min:** the feed is `units`-per-minute; convert inch/min → mm/min → mm/s.
  /// - **G93 inverse-time:** the feed is `1/(duration in minutes)`, so the block (path length `millimeters`)
  ///   runs in `1/feed` minutes → speed `= millimeters × feed / 60`. The inverse-time feed is NOT unit-scaled
  ///   (grbl never inch-scales an inverse-time `F`), and `millimeters` is already true mm, so the result is
  ///   correct under both G20 and G21. An over-fast G93 duration is floored by the axis-rate clamp below, so
  ///   the move finishes slower than commanded rather than losing steps (DOC-10.2 Q3).
  fn nominal_speed_mm_s(
    &self,
    feed: f32,
    units: Units,
    feed_mode: FeedMode,
    rapid: bool,
    millimeters: f32,
    unit_vec: &[f32; AXES],
  ) -> f32 {
    let axis_rate_limit = self.axis_rate_limit_mm_s(unit_vec);
    if rapid {
      // A rapid has no feed word; it cruises at the most restrictive axis rate limit.
      return axis_rate_limit;
    }
    let requested_mm_s = match feed_mode {
      FeedMode::UnitsPerMin => feed * units_scale(units) / 60.0,
      FeedMode::InverseTime => millimeters * feed / 60.0,
    };
    requested_mm_s.min(axis_rate_limit).max(0.0)
  }

  /// The speed limit in mm/s imposed by the per-axis maximum rates (`$110–$112`) for a move along
  /// `unit_vec`: the largest speed at which no axis exceeds its own max rate. An axis with a near-zero
  /// component imposes no limit. Returns `f32::INFINITY` only for the degenerate zero vector, which
  /// never reaches a real block.
  fn axis_rate_limit_mm_s(&self, unit_vec: &[f32; AXES]) -> f32 {
    let mut limit = f32::INFINITY;
    for (axis, &unit) in unit_vec.iter().enumerate() {
      let component = libm::fabsf(unit);
      if component > LENGTH_EPSILON_MM {
        let axis_limit = (self.config.max_rate_mm_min[axis] / 60.0) / component;
        limit = limit.min(axis_limit);
      }
    }
    limit
  }

  /// The maximum junction (cornering) entry speed squared at the boundary between the previous block
  /// and a new block with direction `unit_vec`, using grbl's junction-deviation centripetal model.
  /// With no previous block (zero vector) the junction speed is 0: the first move starts from rest.
  fn junction_speed_sq(&self, unit_vec: &[f32; AXES], acceleration: f32) -> f32 {
    if is_zero_vec(&self.prev_unit_vec) {
      return 0.0;
    }
    // grbl: junction_cos_theta = -dot(prev, curr) over ALL axes (linear and rotary alike, DOC-10.3) — a change
    // in rotary direction is a real velocity discontinuity on the A motor and corners the same as a linear one.
    // For a straight continuation prev==curr so the dot is +1 and cos_theta = -1 (no restriction); for a full
    // reversal the dot is -1 and cos_theta = +1. The all-axis unit vectors are normalized by the full norm, so
    // a 1° rotary step and a 1 mm linear step contribute equally — the grblHAL "degrees == mm" convention.
    let prev = &self.prev_unit_vec;
    let mut dot = 0.0f32;
    for axis in 0..AXES {
      dot += prev[axis] * unit_vec[axis];
    }
    let cos_theta = -dot;
    // A reversal (cos_theta → +1) forces the junction speed to zero: the machine must stop and back up.
    if cos_theta >= 1.0 - JUNCTION_REVERSAL_EPSILON {
      return 0.0;
    }
    // Half-angle identity: sin(theta/2) = sqrt((1 - cos_theta) / 2). Clamp the radicand to ≥ 0 so f32
    // round-off near a straight line cannot produce a NaN.
    let sin_half = libm::sqrtf(((1.0 - cos_theta) * 0.5).max(0.0));
    // A collinear continuation (sin_half → 0) yields an unbounded R; the per-block nominal cap that the
    // caller applies bounds it, so we return +inf here to mean "no cornering restriction".
    if sin_half <= JUNCTION_REVERSAL_EPSILON {
      return f32::INFINITY;
    }
    // grbl's cornering radius from the junction deviation, then v² = a · R, capped by the previous
    // block's nominal speed so a corner never exceeds either adjoining block's cruise speed.
    let radius = self.config.junction_deviation_mm * sin_half / (1.0 - sin_half);
    (acceleration * radius).min(self.prev_nominal_speed_sq)
  }

  /// Enqueue a block, returning [`PlannerError::QueueFull`] when the ring buffer is full.
  fn enqueue(&mut self, block: Block) -> Result<(), PlannerError> {
    self.queue.push_back(block).map_err(|_| PlannerError::QueueFull)
  }

  /// Recompute planned entry speeds across the queued blocks (grbl's planner recalculate): a reverse
  /// pass caps each block's entry by what is reachable decelerating into the next block (the newest
  /// block must be able to stop at its end), then a forward pass caps each entry by what is reachable
  /// accelerating out of the previous block.
  fn recalculate(&mut self) {
    self.reverse_pass();
    self.forward_pass();
  }

  /// Reverse pass (newest → oldest). The newest block must decelerate to a full stop at its end, so its
  /// entry is bounded by `2·a·d`. Each earlier block's entry is bounded by the next block's entry plus
  /// what one block of travel can add under deceleration, never exceeding its own `max_entry_speed_sq`.
  ///
  /// Walks the ring buffer back-to-front via the `Deque`'s double-ended iterator (O(n), no indexing or
  /// fallible access), carrying the next (downstream) block's entry speed as the exit ceiling. The exit
  /// of the newest block is zero: the program may end at any block, so it must be able to stop.
  ///
  /// ## Busy-block protection (Finding #4)
  /// When [`head_busy`](Planner::head_busy) is set, the executor has already committed to the front (oldest)
  /// block's entry speed as the previously-popped block's exit. The reverse pass therefore FREEZES the front
  /// block: it leaves that block's `entry_speed_sq` exactly as the executor read it, but still carries that
  /// committed value forward as the deceleration ceiling for the block behind it — so the rest of the queue
  /// stays consistent with the committed junction without the executor ever seeing its head exit change.
  fn reverse_pass(&mut self) {
    // The front block is the last one the reverse iterator visits; when the head is busy it must not be
    // rewritten. `remaining` counts down so the final (front) block is recognized without indexing.
    let mut remaining = self.queue.len();
    let mut next_entry_sq = 0.0f32;
    for block in self.queue.iter_mut().rev() {
      remaining -= 1;
      let is_busy_head = self.head_busy && remaining == 0;
      if is_busy_head {
        // Frozen: keep the committed entry the executor already read, and carry it as the exit ceiling for
        // the block behind it (already handled by `next_entry_sq` below using this unchanged value).
        next_entry_sq = block.entry_speed_sq;
        continue;
      }
      // Maximum entry that still allows decelerating to `next_entry_sq` over this block's travel, capped
      // by the block's own cornering/nominal ceiling.
      let reachable = next_entry_sq + 2.0 * block.acceleration * block.millimeters;
      block.entry_speed_sq = block.max_entry_speed_sq.min(reachable);
      next_entry_sq = block.entry_speed_sq;
    }
  }

  /// Forward pass (oldest → newest). Each block's entry cannot exceed what is reachable accelerating
  /// from the previous block's entry over the previous block's travel. The oldest block keeps the entry
  /// speed the reverse pass left it (it is the active/continuing motion).
  ///
  /// Walks the ring buffer front-to-back via the iterator (O(n)), carrying the previous block's entry
  /// speed, acceleration, and travel forward to bound each successor's entry.
  fn forward_pass(&mut self) {
    let mut prev: Option<(f32, f32, f32)> = None;
    for block in self.queue.iter_mut() {
      if let Some((prev_entry_sq, prev_accel, prev_mm)) = prev {
        let reachable = prev_entry_sq + 2.0 * prev_accel * prev_mm;
        if block.entry_speed_sq > reachable {
          block.entry_speed_sq = reachable;
        }
      }
      prev = Some((block.entry_speed_sq, block.acceleration, block.millimeters));
    }
  }

  /// Flush look-ahead at a synchronized motion boundary (dwell, predefined move, program end): force
  /// the newest queued block to decelerate to a stop and clear the trailing junction state so the next
  /// move starts from rest. The reverse pass already targets a zero exit on the newest block, so this
  /// only needs to drop the trailing junction context.
  fn flush_lookahead(&mut self) {
    self.prev_unit_vec = [0.0; AXES];
    self.prev_nominal_speed_sq = 0.0;
    // The newest block already decelerates to zero (reverse pass invariant); nothing else to pin.
  }

  /// Plan a G2/G3 arc by chord-tolerance subdivision into short linear blocks. The arc is in the G17
  /// (XY) plane; Z is linearly interpolated across the segments (helical support falls out for free).
  /// Returns the number of segments enqueued, or [`PlannerError::InvalidArc`] for bad geometry.
  fn plan_arc(&mut self, request: &ArcRequest) -> Result<usize, PlannerError> {
    let scale = units_scale(request.units);
    let start = self.position_mm();
    let target = self.arc_endpoint_mm(request.axes, scale, request.distance, request.machine_coords, &start);

    // Center offsets I/J are relative to the start point. At least one must be present (grbl IJ form).
    if request.i.is_none() && request.j.is_none() {
      return Err(PlannerError::InvalidArc);
    }
    let center = [start[0] + request.i.unwrap_or(0.0) * scale, start[1] + request.j.unwrap_or(0.0) * scale];

    let r0 = [start[0] - center[0], start[1] - center[1]];
    let r1 = [target[0] - center[0], target[1] - center[1]];
    let radius = libm::sqrtf(r0[0] * r0[0] + r0[1] * r0[1]);
    if radius < LENGTH_EPSILON_MM {
      return Err(PlannerError::InvalidArc);
    }

    let sweep = arc_sweep_angle(r0, r1, request.cw);
    let segments = arc_segment_count(radius, sweep, self.config.arc_tolerance_mm);

    // All-or-nothing back-pressure: an arc subdivides into `segments` linear blocks, but a pure-sync
    // planner cannot yield mid-arc to let the executor drain. If the whole arc would not fit in the
    // remaining queue space, enqueue nothing and leave the planner position untouched so the caller can
    // drain the motion executor and re-issue the same arc cleanly. This realizes the DOC-05 warning that
    // an over-fine `$12` starves the queue — it surfaces as recoverable back-pressure, never lost
    // geometry. (A zero-length / single-point arc still produces ≥ 1 segment by `arc_segment_count`.)
    if self.queue.len() + segments as usize > BLOCK_QUEUE_LEN {
      return Err(PlannerError::QueueFull);
    }

    let z_start = start[2];
    let z_delta = target[2] - z_start;
    // The rotary A axis is slaved linearly across the arc exactly like the Z helix (DOC-10.5): it advances
    // `a_delta / segments` per chord, in lockstep with the XY interpolation. A is never circularly interpolated;
    // a G2/G3 with an `A` word produces an A-slaved helical arc, not a rotary-plane arc (that is out of scope).
    let a_start = start[A_AXIS];
    let a_delta = target[A_AXIS] - a_start;
    let theta_start = libm::atan2f(r0[1], r0[0]);
    let theta_step = sweep / segments as f32;

    // The per-segment feed. Under G94 every segment carries the same units/min feed. Under G93 the inverse-time
    // F describes the WHOLE arc's duration; since a circular arc with a constant angular step subdivides into
    // EQUAL-length segments (identical chord, identical Z/A step), each segment must take `1/(feed × segments)`
    // minutes, i.e. its inverse-time feed is `feed × segments`. This keeps the arc's total duration at `1/feed`
    // minutes while reusing the same per-block inverse-time math (DOC-10.2).
    let seg_feed = match request.feed_mode {
      FeedMode::UnitsPerMin => request.feed,
      FeedMode::InverseTime => request.feed * segments as f32,
    };
    let mut enqueued = 0usize;
    for seg in 1..=segments {
      let theta = theta_start + theta_step * seg as f32;
      let x = center[0] + radius * libm::cosf(theta);
      let y = center[1] + radius * libm::sinf(theta);
      let frac = seg as f32 / segments as f32;
      let z = z_start + z_delta * frac;
      let a = a_start + a_delta * frac;
      let seg_target = self.mm_target_to_steps(&[x, y, z, a]);
      // Each segment is a linear feed move; the arc feed applies (already in active units). Enqueue without
      // look-ahead — the all-or-nothing pre-check above guaranteed the queue has room for every segment, so
      // this cannot hit `QueueFull` mid-arc, and the per-segment junction state still advances so adjacent
      // segments corner against each other. The single `recalculate()` below then resolves all entry speeds
      // in one O(n) reverse+forward pass instead of one pass per segment (which would be O(n²)).
      enqueued += self.enqueue_move(seg_target, seg_feed, request.units, request.feed_mode, false, false)?;
    }
    // Resolve look-ahead once across the whole arc. This is exactly equivalent to recalculating after each
    // segment, because the reverse/forward passes always sweep the entire queue — only the final state of
    // the queue matters, and it is identical either way.
    self.recalculate();
    Ok(enqueued)
  }

  /// Resolve an arc endpoint to absolute machine mm, honouring units, distance mode, and the G53
  /// machine-coordinate flag. An absolute work endpoint adds the WCO; an absolute G53 endpoint is already in
  /// machine coordinates; an incremental endpoint adds to the start. Unmentioned axes keep the start position.
  fn arc_endpoint_mm(
    &self,
    axes: &AxisWords,
    scale: f32,
    distance: DistanceMode,
    machine_coords: bool,
    start: &[f32; AXES],
  ) -> [f32; AXES] {
    let words = [axes.x, axes.y, axes.z, axes.a];
    let mut endpoint = *start;
    for axis in 0..AXES {
      if let Some(value) = words[axis] {
        // A rotary axis word bypasses G20/G21 inch scaling (DOC-10.1), matching `resolve_target`; the I/J center
        // offsets handled by the caller are always linear (X/Y), so they keep the units scale.
        let axis_scale = if self.config.is_rotary(axis) { 1.0 } else { scale };
        endpoint[axis] = match distance {
          DistanceMode::Absolute if machine_coords => value * axis_scale,
          DistanceMode::Absolute => value * axis_scale + self.work_offset_mm[axis],
          DistanceMode::Incremental => start[axis] + value * axis_scale,
        };
      }
    }
    endpoint
  }

  /// Convert a machine-mm target into absolute step counts per axis (`round(mm × steps_per_mm)`).
  fn mm_target_to_steps(&self, target_mm: &[f32; AXES]) -> [i32; AXES] {
    let mut steps = [0i32; AXES];
    for (axis, slot) in steps.iter_mut().enumerate() {
      *slot = mm_to_steps(target_mm[axis], self.config.steps_per_mm[axis]);
    }
    steps
  }
}

/// The multiplicative factor converting a value in `units` to millimeters (1 for mm, 25.4 for inch).
fn units_scale(units: Units) -> f32 {
  match units {
    Units::Millimeter => 1.0,
    Units::Inch => MM_PER_INCH,
  }
}

/// Convert a position in mm to the nearest whole step count for an axis with `steps_per_mm` resolution.
fn mm_to_steps(pos_mm: f32, steps_per_mm: f32) -> i32 {
  libm::roundf(pos_mm * steps_per_mm) as i32
}

/// Whether a resolved MACHINE step `target` violates the `$20`/`$130–$132` soft-limit envelope on any axis
/// (DOC-08 Phase D). grbl homes each axis to machine zero at its positive end, so the work volume is the closed
/// interval `[-max_travel, 0]` per axis (machine coordinates are non-positive). A small one-step tolerance
/// absorbs the `round()` at the mm→step boundary so a jog exactly to `-max_travel` is accepted, not spuriously
/// rejected by sub-step rounding. Host-tested in isolation so the envelope rule cannot drift from the jog path.
pub fn soft_limit_violation(
  target: &[i32; AXES],
  steps_per_mm: &[f32; AXES],
  max_travel_mm: &[f32; AXES],
  rotary_mask: u8,
) -> bool {
  for axis in 0..AXES {
    // A rotary axis (per the `$376` mask) is continuous / rollover: soft limits never apply to it, and its
    // `$133` max-travel is ignored entirely — `$133` keeps a single unambiguous meaning for bounded axes only
    // (DOC-10.6). Skip it before the envelope check so a huge A target is never a violation.
    if rotary_mask & (1 << axis) != 0 {
      continue;
    }
    // The lower bound in steps, with a one-step rounding tolerance: a target at exactly `-max_travel` (which
    // `round()` may place one step beyond) is still inside the envelope.
    let lower = mm_to_steps(-max_travel_mm[axis], steps_per_mm[axis]) - 1;
    // The upper bound is machine zero, with the same one-step tolerance for a jog to the home end.
    if target[axis] < lower || target[axis] > 1 {
      return true;
    }
  }
  false
}

/// True when every component of `vec` is effectively zero (used to detect "no previous block").
fn is_zero_vec(vec: &[f32; AXES]) -> bool {
  vec.iter().all(|&c| libm::fabsf(c) < LENGTH_EPSILON_MM)
}

/// The acceleration limit in mm/s² along `unit_vec` given per-axis acceleration limits: the largest
/// acceleration at which no axis exceeds its own limit. An axis with a near-zero component imposes no
/// limit. grbl's per-axis acceleration limiting model.
fn limiting_acceleration(unit_vec: &[f32; AXES], accel_mm_s2: &[f32; AXES]) -> f32 {
  let mut limit = f32::INFINITY;
  for axis in 0..AXES {
    let component = libm::fabsf(unit_vec[axis]);
    if component > LENGTH_EPSILON_MM {
      limit = limit.min(accel_mm_s2[axis] / component);
    }
  }
  limit
}

/// Epsilon for the junction cosine near the straight-line and full-reversal limits. Round-off in the
/// dot product can push a collinear case slightly past ±1; clamping here keeps the half-angle stable.
const JUNCTION_REVERSAL_EPSILON: f32 = 1.0e-6;

/// The signed sweep angle (radians) of an arc from start radius vector `r0` to end radius vector `r1`
/// in the chosen direction. grbl/GCode convention: positive angles are counter-clockwise. For a CW arc
/// the sweep is taken negative; for CCW positive. A full circle (start == end) sweeps a full turn.
fn arc_sweep_angle(r0: [f32; 2], r1: [f32; 2], cw: bool) -> f32 {
  use core::f32::consts::PI;
  // The signed angle between r0 and r1 via atan2 of the 2D cross and dot products; range (-PI, PI].
  let cross = r0[0] * r1[1] - r0[1] * r1[0];
  let dot = r0[0] * r1[0] + r0[1] * r1[1];
  let mut angle = libm::atan2f(cross, dot);
  if cw {
    // Clockwise sweeps in the negative direction; map a non-negative atan2 result into (−2PI, 0).
    if angle >= 0.0 {
      angle -= 2.0 * PI;
    }
  } else {
    // Counter-clockwise sweeps positive; map a non-positive result into (0, 2PI].
    if angle <= 0.0 {
      angle += 2.0 * PI;
    }
  }
  angle
}

/// The number of linear segments to subdivide an arc into so the chord error stays within `tolerance`.
/// grbl derives the per-segment angle from `acos(1 − tol/r)`; the segment count is the total sweep
/// divided by that angle, rounded up, and always at least 1.
fn arc_segment_count(radius: f32, sweep: f32, tolerance: f32) -> u32 {
  // Chord error of a segment subtending angle dθ on radius r is r·(1 − cos(dθ/2)). Solving for the
  // largest dθ with error ≤ tol gives dθ = 2·acos(1 − tol/r). grbl uses the equivalent single-acos
  // form below; guard the radicand so a tolerance larger than the radius cannot produce a NaN.
  let ratio = 1.0 - (tolerance / radius);
  let max_angle = if ratio <= -1.0 {
    core::f32::consts::PI
  } else if ratio >= 1.0 {
    // Tolerance is effectively zero relative to the radius; fall back to a fine angle to avoid div-by-0.
    f32::EPSILON
  } else {
    libm::acosf(ratio)
  };
  let sweep_abs = libm::fabsf(sweep);
  let count = libm::ceilf(sweep_abs / max_angle.max(f32::EPSILON));
  // At least one segment, even for a tiny sweep; cast is safe because count ≥ 1 and finite here.
  (count as u32).max(1)
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::gcode::SpindleState;

  // A planner config with whole-number step resolution and generous limits so hand-computed vectors
  // are exact: 100 steps/mm, 6000 mm/min (= 100 mm/s) max rate, 1000 mm/s² accel on every axis.
  fn test_config() -> PlannerConfig {
    PlannerConfig {
      steps_per_mm: [100.0; AXES],
      max_rate_mm_min: [6000.0; AXES],
      accel_mm_s2: [1000.0; AXES],
      junction_deviation_mm: 0.01,
      arc_tolerance_mm: 0.002,
      rotary_mask: DEFAULT_ROTARY_MASK,
    }
  }

  fn mm_move(x: Option<f32>, y: Option<f32>, z: Option<f32>, feed: f32, rapid: bool) -> PlannerCommand {
    PlannerCommand::Move {
      rapid,
      axes: AxisWords { x, y, z, a: None },
      units: Units::Millimeter,
      distance: DistanceMode::Absolute,
      feed,
      feed_mode: FeedMode::UnitsPerMin,
      machine_coords: false,
    }
  }

  // ---- Target resolution: units, offsets, distance mode, steps/mm -------------------------------

  #[test]
  fn resolve_absolute_mm_target_to_steps() {
    let planner = Planner::new(test_config());
    let axes = AxisWords { x: Some(10.0), y: Some(-5.0), z: None, a: None };
    let target = planner.resolve_target(&axes, Units::Millimeter, DistanceMode::Absolute, false);
    // 10 mm × 100 steps/mm = 1000; -5 mm × 100 = -500; Z unmentioned stays at 0.
    assert_eq!(target, [1000, -500, 0, 0]);
  }

  #[test]
  fn resolve_inch_target_scales_by_25_4() {
    let planner = Planner::new(test_config());
    let axes = AxisWords { x: Some(1.0), y: None, z: None, a: None };
    let target = planner.resolve_target(&axes, Units::Inch, DistanceMode::Absolute, false);
    // 1 inch = 25.4 mm × 100 steps/mm = 2540 steps.
    assert_eq!(target, [2540, 0, 0, 0]);
  }

  #[test]
  fn resolve_incremental_adds_to_current_position() {
    let mut planner = Planner::new(test_config());
    // Move to X10 absolute first so the position advances to 1000 steps.
    planner.plan_command(&mm_move(Some(10.0), None, None, 600.0, false)).expect("queued");
    let axes = AxisWords { x: Some(2.5), y: None, z: None, a: None };
    let target = planner.resolve_target(&axes, Units::Millimeter, DistanceMode::Incremental, false);
    // 1000 steps (10 mm) + 2.5 mm × 100 = 1250 steps.
    assert_eq!(target, [1250, 0, 0, 0]);
  }

  #[test]
  fn work_offset_maps_absolute_work_words_to_machine() {
    // The planner no longer owns G92 — the consumer computes the full WCO via `coords::CoordinateSystems` and
    // pushes it here. The planner's job is to ADD that WCO to absolute work words. Push a WCO of (10, 20) so an
    // absolute work (0, 0) resolves to machine (10, 20).
    let mut planner = Planner::new(test_config());
    planner.set_work_offset([10.0, 20.0, 0.0, 0.0]);
    assert_eq!(planner.work_offset(), [10.0, 20.0, 0.0, 0.0]);
    let axes = AxisWords { x: Some(0.0), y: Some(0.0), z: None, a: None };
    let target = planner.resolve_target(&axes, Units::Millimeter, DistanceMode::Absolute, false);
    assert_eq!(target, [1000, 2000, 0, 0]);
  }

  #[test]
  fn g53_machine_move_bypasses_the_work_offset() {
    // A G53 one-shot move's words are MACHINE coordinates, so the WCO must NOT be applied: an absolute G53
    // X0 Y0 resolves to machine (0, 0) even with a non-zero work offset set.
    let mut planner = Planner::new(test_config());
    planner.set_work_offset([10.0, 20.0, 0.0, 0.0]);
    let axes = AxisWords { x: Some(0.0), y: Some(0.0), z: None, a: None };
    let target = planner.resolve_target(&axes, Units::Millimeter, DistanceMode::Absolute, true);
    assert_eq!(target, [0, 0, 0, 0]);
  }

  #[test]
  fn coordinate_op_passes_through_without_motion() {
    // A coordinate op (here G92.1 clear) is passed through for the consumer to apply to the coordinate model;
    // the planner produces no block and leaves the queue untouched.
    let mut planner = Planner::new(test_config());
    let op = CoordinateOp::ClearG92;
    assert_eq!(
      planner.plan_command(&PlannerCommand::Coordinate(op)).expect("passthrough"),
      PlannerOutcome::Coordinate(op),
    );
    assert!(planner.is_empty());
  }

  // ---- Phase C: probe command resolution + position sync ----------------------------------------

  #[test]
  fn probe_resolves_work_target_to_machine_steps_and_flushes() {
    use crate::gcode::ProbeKind;
    let mut planner = Planner::new(test_config());
    // A WCO of (0, 0, 50): an absolute work probe to Z-5 resolves to machine Z = -5 + 50 = 45 mm → 4500 steps.
    planner.set_work_offset([0.0, 0.0, 50.0, 0.0]);
    // Queue a move first so the flush is observable (the probe must clear trailing junction state).
    planner.plan_command(&mm_move(Some(10.0), None, None, 600.0, false)).expect("queued");
    let cmd = PlannerCommand::Probe {
      kind: ProbeKind::G38_2,
      axes: AxisWords { x: None, y: None, z: Some(-5.0), a: None },
      units: Units::Millimeter,
      distance: DistanceMode::Absolute,
      feed: 50.0,
    };
    let outcome = planner.plan_command(&cmd).expect("probe");
    assert_eq!(
      outcome,
      PlannerOutcome::Probe {
        kind: ProbeKind::G38_2,
        target: [1000, 0, 4500, 0],
        feed: 50.0,
        units: Units::Millimeter,
      }
    );
    // A probe does NOT enqueue a block; the queue is unchanged (still holds the earlier move).
    assert_eq!(planner.queued_len(), 1);
  }

  #[test]
  fn sync_position_sets_commanded_position_to_the_probe_stop() {
    let mut planner = Planner::new(test_config());
    planner.plan_command(&mm_move(Some(10.0), None, None, 600.0, false)).expect("queued");
    // The executor latched a stop at machine Z = 4.2 mm → 420 steps; sync the commanded position to it.
    planner.sync_position([0, 0, 420, 0]);
    assert_eq!(planner.position_steps(), [0, 0, 420, 0]);
    // The next absolute move resolves relative to the synced position (Z stays, X moves to 10).
    let target = planner.resolve_target(
      &AxisWords { x: Some(10.0), y: None, z: None, a: None },
      Units::Millimeter,
      DistanceMode::Absolute,
      false,
    );
    assert_eq!(target, [1000, 0, 420, 0]);
  }

  #[test]
  fn probe_after_sync_starts_next_move_from_rest() {
    use crate::gcode::ProbeKind;
    let mut planner = Planner::new(test_config());
    // Run a probe (flushes look-ahead), sync to the stop, then a following move must start from rest (no
    // previous-block junction carried across the probe boundary).
    planner
      .plan_command(&PlannerCommand::Probe {
        kind: ProbeKind::G38_2,
        axes: AxisWords { x: None, y: None, z: Some(-5.0), a: None },
        units: Units::Millimeter,
        distance: DistanceMode::Absolute,
        feed: 50.0,
      })
      .expect("probe");
    planner.sync_position([0, 0, 420, 0]);
    planner.plan_command(&mm_move(Some(0.0), None, Some(10.0), 600.0, false)).expect("queued");
    let block = planner.peek_block().expect("a block");
    assert!(block.entry_speed_sq < 1e-3, "the post-probe move starts at rest");
  }

  // ---- Phase D: jog planning, tagging, flush, and soft-limit rejection ---------------------------

  fn jog(x: Option<f32>, y: Option<f32>, z: Option<f32>, feed: f32, machine_coords: bool) -> JogCommand {
    JogCommand {
      axes: AxisWords { x, y, z, a: None },
      distance_mode: DistanceMode::Absolute,
      units: Units::Millimeter,
      feed,
      machine_coords,
    }
  }

  #[test]
  fn plan_jog_enqueues_a_jog_tagged_block_resolved_through_the_work_offset() {
    let mut planner = Planner::new(test_config());
    // An absolute work jog with a WCO of (10, 0, 0): work X0 resolves to machine X10 → 1000 steps.
    planner.set_work_offset([10.0, 0.0, 0.0, 0.0]);
    let outcome = planner.plan_jog(&jog(Some(0.0), None, None, 600.0, false), None).expect("jog");
    assert_eq!(outcome, PlannerOutcome::Queued { blocks: 1 });
    let block = planner.peek_block().expect("a jog block");
    assert!(block.jog, "the block is tagged as a jog so a jog-cancel can flush it");
    assert_eq!(block.steps, [1000, 0, 0, 0]);
  }

  #[test]
  fn plan_jog_g53_bypasses_the_work_offset() {
    let mut planner = Planner::new(test_config());
    planner.set_work_offset([10.0, 20.0, 0.0, 0.0]);
    // A G53 jog's words are MACHINE coordinates: machine X-5 resolves to -500 steps regardless of the WCO.
    let outcome = planner.plan_jog(&jog(Some(-5.0), None, None, 600.0, true), None).expect("jog");
    assert_eq!(outcome, PlannerOutcome::Queued { blocks: 1 });
    assert_eq!(planner.peek_block().expect("block").steps, [-500, 0, 0, 0]);
  }

  #[test]
  fn flush_jog_blocks_drains_only_jog_blocks() {
    let mut planner = Planner::new(test_config());
    // Enqueue a program move first (NOT a jog), then two jogs behind it. A flush must remove ONLY the two jog
    // blocks from the back, leaving the program block intact.
    planner.plan_command(&mm_move(Some(1.0), None, None, 600.0, false)).expect("move");
    planner.plan_jog(&jog(Some(2.0), None, None, 600.0, false), None).expect("jog");
    planner.plan_jog(&jog(Some(3.0), None, None, 600.0, false), None).expect("jog");
    assert_eq!(planner.queued_len(), 3);
    let flushed = planner.flush_jog_blocks();
    assert_eq!(flushed, 2, "exactly the two jog blocks are flushed");
    assert_eq!(planner.queued_len(), 1, "the program block survives");
    assert!(!planner.peek_block().expect("block").jog, "the surviving block is the program move");
  }

  #[test]
  fn flush_jog_blocks_on_an_all_jog_queue_empties_it() {
    let mut planner = Planner::new(test_config());
    planner.plan_jog(&jog(Some(1.0), None, None, 600.0, false), None).expect("jog");
    planner.plan_jog(&jog(Some(2.0), None, None, 600.0, false), None).expect("jog");
    assert_eq!(planner.flush_jog_blocks(), 2);
    assert!(planner.is_empty());
  }

  #[test]
  fn plan_jog_rejects_a_target_outside_the_soft_limit_envelope() {
    let mut planner = Planner::new(test_config());
    // 100 steps/mm; a 50 mm travel envelope means machine [-50, 0] mm. A jog to machine X+10 (positive → past
    // the home end) violates, and a jog to X-60 (past -max_travel) violates; a jog to X-25 is accepted.
    let limits = SoftLimits { max_travel_mm: [50.0; AXES] };
    assert_eq!(
      planner.plan_jog(&jog(Some(10.0), None, None, 600.0, true), Some(limits)),
      Err(PlannerError::JogExceedsTravel),
    );
    assert!(planner.is_empty(), "a rejected jog enqueues nothing");
    assert_eq!(
      planner.plan_jog(&jog(Some(-60.0), None, None, 600.0, true), Some(limits)),
      Err(PlannerError::JogExceedsTravel),
    );
    planner.plan_jog(&jog(Some(-25.0), None, None, 600.0, true), Some(limits)).expect("in-envelope jog");
    assert_eq!(planner.queued_len(), 1);
  }

  #[test]
  fn plan_jog_ignores_soft_limits_when_disabled() {
    let mut planner = Planner::new(test_config());
    // With `$20` off (limits = None) even an out-of-envelope jog is accepted (grbl only checks when enabled).
    planner.plan_jog(&jog(Some(10.0), None, None, 600.0, true), None).expect("jog");
    assert_eq!(planner.queued_len(), 1);
  }

  /// Build a `G53` (machine-coordinate) absolute `Move` command to `x` mm — the pre-check operates on the
  /// resolved MACHINE target, and `G53` bypasses the work offset so the test target IS the machine target.
  fn move_to_machine_x(x: f32, feed: f32) -> PlannerCommand {
    PlannerCommand::Move {
      rapid: false,
      axes: AxisWords { x: Some(x), y: None, z: None, a: None },
      units: Units::Millimeter,
      distance: DistanceMode::Absolute,
      feed,
      feed_mode: FeedMode::UnitsPerMin,
      machine_coords: true,
    }
  }

  #[test]
  fn plan_command_with_limits_rejects_a_program_move_outside_the_envelope() {
    let mut planner = Planner::new(test_config());
    // 100 steps/mm, 50 mm travel => machine envelope [-50, 0] mm. A program move to machine X+10 (past the home
    // end) and to X-60 (past -max_travel) both violate → `MoveExceedsTravel` (the consumer maps it to ALARM:2),
    // and nothing is enqueued. An in-envelope move (X-25) is accepted.
    let limits = SoftLimits { max_travel_mm: [50.0; AXES] };
    assert_eq!(
      planner.plan_command_with_limits(&move_to_machine_x(10.0, 600.0), Some(limits)),
      Err(PlannerError::MoveExceedsTravel),
    );
    assert!(planner.is_empty(), "a rejected program move enqueues nothing");
    assert_eq!(
      planner.plan_command_with_limits(&move_to_machine_x(-60.0, 600.0), Some(limits)),
      Err(PlannerError::MoveExceedsTravel),
    );
    assert!(planner.is_empty());
    planner.plan_command_with_limits(&move_to_machine_x(-25.0, 600.0), Some(limits)).expect("in-envelope move");
    assert_eq!(planner.queued_len(), 1);
  }

  #[test]
  fn plan_command_with_limits_none_matches_plain_plan_command() {
    // With `limits = None` (soft limits off / unhomed) an out-of-envelope move is accepted, identical to the
    // plain `plan_command` — the soft-limit gate is purely additive and only active when the consumer supplies
    // an envelope.
    let mut planner = Planner::new(test_config());
    planner.plan_command_with_limits(&move_to_machine_x(10.0, 600.0), None).expect("no-limit move accepted");
    assert_eq!(planner.queued_len(), 1);
  }

  #[test]
  fn soft_limit_violation_envelope() {
    let steps_per_mm = [100.0; AXES];
    let max_travel = [50.0; AXES];
    // Mask 0 => every axis (including A) is treated as a bounded linear axis for this envelope test.
    let mask = 0;
    // Inside the [-50, 0] mm envelope (machine coordinates ≤ 0): accepted.
    assert!(!soft_limit_violation(&[0, -2500, -5000, 0], &steps_per_mm, &max_travel, mask));
    // A positive target (past the home end) violates.
    assert!(soft_limit_violation(&[100, 0, 0, 0], &steps_per_mm, &max_travel, mask));
    // Past -max_travel violates.
    assert!(soft_limit_violation(&[0, -5200, 0, 0], &steps_per_mm, &max_travel, mask));
    // Exactly at the -max_travel boundary is accepted (one-step rounding tolerance).
    assert!(!soft_limit_violation(&[-5000, 0, 0, 0], &steps_per_mm, &max_travel, mask));
  }

  #[test]
  fn rotary_axis_is_exempt_from_soft_limits() {
    // DOC-10.6: an axis marked rotary in `$376` is continuous/rollover — its `$133` is ignored and a huge target
    // is never a violation. With the default mask (bit 3 = A), a massive A target passes; clearing the bit (A as
    // a bounded 4th linear axis) makes the same target violate, proving `$133` governs only non-rotary axes.
    let steps_per_mm = [100.0; AXES];
    let max_travel = [50.0; AXES]; // A's $133 = 50 mm/deg-equivalent, ignored while A is rotary.
    let huge_a = [0, 0, 0, 1_000_000];
    assert!(!soft_limit_violation(&huge_a, &steps_per_mm, &max_travel, DEFAULT_ROTARY_MASK), "A rotary => exempt");
    assert!(soft_limit_violation(&huge_a, &steps_per_mm, &max_travel, 0), "A linear => $133 envelope enforced");
  }

  // ---- Block geometry: unit vector, mm travel, dominant axis ------------------------------------

  #[test]
  fn pure_x_move_has_unit_vector_and_mm() {
    let mut planner = Planner::new(test_config());
    planner.plan_command(&mm_move(Some(10.0), None, None, 600.0, false)).expect("queued");
    let block = planner.peek_block().expect("a block");
    assert_eq!(block.steps, [1000, 0, 0, 0]);
    assert_eq!(block.step_event_count, 1000);
    assert!((block.unit_vec[0] - 1.0).abs() < 1e-6);
    assert!(block.unit_vec[1].abs() < 1e-6 && block.unit_vec[2].abs() < 1e-6);
    assert!((block.millimeters - 10.0).abs() < 1e-4);
  }

  #[test]
  fn diagonal_move_unit_vector_is_normalized() {
    let mut planner = Planner::new(test_config());
    // X3 Y4 → 5 mm hypotenuse, unit vector (0.6, 0.8, 0).
    planner.plan_command(&mm_move(Some(3.0), Some(4.0), None, 600.0, false)).expect("queued");
    let block = planner.peek_block().expect("a block");
    assert!((block.millimeters - 5.0).abs() < 1e-4);
    assert!((block.unit_vec[0] - 0.6).abs() < 1e-4);
    assert!((block.unit_vec[1] - 0.8).abs() < 1e-4);
    assert_eq!(block.step_event_count, 400); // dominant axis is Y at 4 mm × 100 steps/mm.
  }

  #[test]
  fn zero_length_move_enqueues_nothing() {
    let mut planner = Planner::new(test_config());
    let outcome = planner.plan_command(&mm_move(Some(0.0), Some(0.0), Some(0.0), 600.0, false)).expect("ok");
    assert_eq!(outcome, PlannerOutcome::Queued { blocks: 0 });
    assert!(planner.is_empty());
  }

  // ---- Nominal speed: feed conversion, rapid rate, axis-rate clamp ------------------------------

  #[test]
  fn feed_move_nominal_speed_is_feed_in_mm_per_s() {
    let mut planner = Planner::new(test_config());
    // F600 mm/min = 10 mm/s along X; well under the 100 mm/s axis limit, so nominal = 10 mm/s.
    planner.plan_command(&mm_move(Some(50.0), None, None, 600.0, false)).expect("queued");
    let block = planner.peek_block().expect("a block");
    assert!((block.nominal_speed() - 10.0).abs() < 1e-3);
  }

  #[test]
  fn feed_move_is_clamped_to_axis_max_rate() {
    let mut planner = Planner::new(test_config());
    // F12000 mm/min = 200 mm/s requested but the X max rate is 6000 mm/min = 100 mm/s.
    planner.plan_command(&mm_move(Some(50.0), None, None, 12000.0, false)).expect("queued");
    let block = planner.peek_block().expect("a block");
    assert!((block.nominal_speed() - 100.0).abs() < 1e-3);
  }

  #[test]
  fn rapid_move_uses_axis_rate_limit_not_feed() {
    let mut planner = Planner::new(test_config());
    // A G0 with feed 0 should still cruise at the axis rate limit (100 mm/s on X).
    planner.plan_command(&mm_move(Some(50.0), None, None, 0.0, true)).expect("queued");
    let block = planner.peek_block().expect("a block");
    assert!(block.rapid);
    assert!((block.nominal_speed() - 100.0).abs() < 1e-3);
  }

  // ---- Block queue: fill, drain order, full-queue error -----------------------------------------

  #[test]
  fn queue_fills_to_capacity_then_errors() {
    let mut planner = Planner::new(test_config());
    // Each move advances X by 1 mm so every block is non-zero-length and distinct.
    for n in 1..=BLOCK_QUEUE_LEN {
      let outcome = planner.plan_command(&mm_move(Some(n as f32), None, None, 600.0, false)).expect("queued");
      assert_eq!(outcome, PlannerOutcome::Queued { blocks: 1 });
    }
    assert_eq!(planner.queued_len(), BLOCK_QUEUE_LEN);
    // The next non-zero move overflows the ring buffer.
    let err = planner.plan_command(&mm_move(Some((BLOCK_QUEUE_LEN + 1) as f32), None, None, 600.0, false));
    assert_eq!(err, Err(PlannerError::QueueFull));
  }

  #[test]
  fn blocks_drain_in_fifo_order() {
    let mut planner = Planner::new(test_config());
    planner.plan_command(&mm_move(Some(1.0), None, None, 600.0, false)).expect("queued");
    planner.plan_command(&mm_move(Some(3.0), None, None, 600.0, false)).expect("queued");
    let first = planner.pop_block().expect("first");
    let second = planner.pop_block().expect("second");
    assert_eq!(first.steps, [100, 0, 0, 0]); // 0 → 1 mm
    assert_eq!(second.steps, [200, 0, 0, 0]); // 1 → 3 mm
    assert!(planner.is_empty());
  }

  // ---- Junction-deviation cornering: collinear, 90°, reversal -----------------------------------

  #[test]
  fn collinear_junction_is_unrestricted_by_cornering() {
    let mut planner = Planner::new(test_config());
    // Two consecutive +X moves: the junction is a straight line, so the second block's max entry is
    // bounded only by its nominal speed, not by cornering.
    planner.plan_command(&mm_move(Some(10.0), None, None, 600.0, false)).expect("queued");
    planner.plan_command(&mm_move(Some(20.0), None, None, 600.0, false)).expect("queued");
    let second = planner.queue.iter().nth(1).expect("second block");
    // Nominal is 10 mm/s → nominal_sq = 100; max entry should equal nominal (no cornering cut).
    assert!((second.max_entry_speed_sq - 100.0).abs() < 1e-2);
  }

  #[test]
  fn full_reversal_junction_speed_is_zero() {
    let mut planner = Planner::new(test_config());
    // +X then −X: a 180° reversal. The machine must stop at the junction.
    planner.plan_command(&mm_move(Some(10.0), None, None, 600.0, false)).expect("queued");
    planner.plan_command(&mm_move(Some(0.0), None, None, 600.0, false)).expect("queued");
    let second = planner.queue.iter().nth(1).expect("second block");
    assert!(second.max_entry_speed_sq < 1e-3);
    assert!(second.entry_speed_sq < 1e-3);
  }

  #[test]
  fn right_angle_junction_matches_grbl_formula() {
    let mut planner = Planner::new(test_config());
    // +X then +Y is a 90° corner. grbl: cos_theta = -dot = 0, sin_half = sqrt(0.5), R = jd·s/(1−s),
    // v² = a·R. With jd = 0.01, a = 1000: s = 0.70710677, R = 0.01·s/(1−s) ≈ 0.024142, v² ≈ 24.142.
    planner.plan_command(&mm_move(Some(10.0), None, None, 600.0, false)).expect("queued");
    planner.plan_command(&mm_move(Some(10.0), Some(10.0), None, 600.0, false)).expect("queued");
    let second = planner.queue.iter().nth(1).expect("second block");
    let sin_half = libm::sqrtf(0.5);
    let radius = 0.01 * sin_half / (1.0 - sin_half);
    let expected_sq = 1000.0 * radius;
    // The junction limit (≈ 24.1) is well below the nominal cap (100), so it governs max entry.
    assert!((second.max_entry_speed_sq - expected_sq).abs() < 0.1);
  }

  // ---- Reverse / forward passes: hand-checked entry speeds --------------------------------------

  #[test]
  fn lone_block_starts_and_stops_at_rest() {
    let mut planner = Planner::new(test_config());
    // A single move has no previous block, so its junction (and thus max-entry) speed is 0 — it starts
    // from rest — and the reverse pass forces a zero exit so it also stops. Entry is therefore 0; the
    // 10 mm/s cruise is reached mid-block by the segment generator (DOC-02), never at the boundaries.
    planner.plan_command(&mm_move(Some(10.0), None, None, 600.0, false)).expect("queued");
    let block = planner.peek_block().expect("a block");
    assert!(block.entry_speed_sq < 1e-3);
    // The block can still cruise at its nominal speed (10 mm/s) somewhere in its interior.
    assert!((block.nominal_speed() - 10.0).abs() < 1e-3);
  }

  #[test]
  fn second_block_entry_is_capped_by_what_the_first_block_can_reach() {
    let mut planner = Planner::new(test_config());
    // First block: a short 0.05 mm move that starts at rest (no previous block). The most the machine
    // can accelerate over it is reachable² = 0 + 2·a·d = 2·1000·0.05 = 100 (mm/s)². So however fast the
    // second block wants to enter, the forward pass caps its entry at ≈ 100 (mm/s)².
    planner.plan_command(&mm_move(Some(0.05), None, None, 12000.0, false)).expect("queued");
    planner.plan_command(&mm_move(Some(20.0), None, None, 12000.0, false)).expect("queued");
    let second = planner.queue.iter().nth(1).expect("second");
    assert!((second.entry_speed_sq - 100.0).abs() < 1.0);
  }

  #[test]
  fn forward_pass_limits_entry_by_acceleration_from_rest() {
    let mut planner = Planner::new(test_config());
    // First block is short so its entry is forced to 0 by the reverse pass (must stop)? No — the first
    // block starts at rest (no previous block → junction speed 0 → entry 0). The SECOND block's entry
    // is then limited by acceleration over the first block's travel: reachable² = 0 + 2·a·d1.
    // d1 = 0.05 mm → reachable² = 100. Second block requests high feed but its entry is forward-capped.
    planner.plan_command(&mm_move(Some(0.05), None, None, 12000.0, false)).expect("queued");
    planner.plan_command(&mm_move(Some(20.0), None, None, 12000.0, false)).expect("queued");
    let first = planner.queue.iter().next().expect("first");
    let second = planner.queue.iter().nth(1).expect("second");
    // First block starts from rest (no previous block).
    assert!(first.entry_speed_sq < 1e-3);
    // Second block's entry is capped by acceleration over the first block: 0 + 2·1000·0.05 = 100.
    assert!((second.entry_speed_sq - 100.0).abs() < 1.0);
  }

  // ---- Busy-block protection: a popped head's committed exit is frozen (Finding #4) -------------

  #[test]
  fn popping_a_block_freezes_the_new_head_entry_speed() {
    // The executor pops block N and reads the new head (N+1) `entry_speed_sq` as N's *exit* speed. If a
    // later enqueue runs `recalculate()` and its reverse pass lowers that head entry, N would have
    // decelerated to the wrong exit — a velocity discontinuity / missed steps. grbl's busy-block rule
    // freezes the head once the executor has committed to it; this test pins that contract.
    //
    // To be sensitive to the bug, the chain must be SHORT collinear blocks: short enough that the stop-at-rest
    // ramp forced on the newest block spans many blocks. 0.1 mm blocks at accel 1000 mm/s² add only
    // 2·a·d = 200 (mm/s)² of reachable speed each, so the head sits partway up a multi-block deceleration
    // envelope — and EXTENDING that envelope (enqueuing more blocks) shifts where the stop is and rewrites the
    // head's entry. The freeze must pin the head once the executor has committed to it, whichever way the
    // unprotected recalculate would move it.
    let mut planner = Planner::new(test_config());
    // Build a short initial look-ahead so the head's committed entry is partway up the stop ramp (not yet at
    // nominal, not at rest): three 0.1 mm collinear moves. The reverse pass forces the newest to stop, so the
    // head enters at a modest, non-trivial speed that a longer runway would later raise.
    let mut x = 0.0;
    for _ in 0..3 {
      x += 0.1;
      planner.plan_command(&mm_move(Some(x), None, None, 12000.0, false)).expect("queued");
    }
    // The executor pops N; the new head (N+1) is the committed next block. Record its entry — the value the
    // executor has already used to shape N's deceleration to this exit.
    let _n = planner.pop_block().expect("first block");
    let committed_exit_sq = planner.peek_block().expect("head").entry_speed_sq;
    assert!(committed_exit_sq > 1.0, "the committed head must enter above rest for a meaningful test");
    // Now core 0 streams more short collinear moves. Each enqueue runs `recalculate()`; the extended runway
    // would, unprotected, RAISE the head's entry (more room to keep speed up before the new final stop) — a
    // change to a value the executor already committed to. The freeze must keep it constant.
    for _ in 0..12 {
      x += 0.1;
      planner.plan_command(&mm_move(Some(x), None, None, 12000.0, false)).expect("queued");
    }
    let after = planner.peek_block().expect("head").entry_speed_sq;
    assert!(
      (after - committed_exit_sq).abs() < 1e-3,
      "the committed head entry must stay frozen: was {committed_exit_sq}, became {after}",
    );
  }

  #[test]
  fn unpopped_queue_still_optimizes_the_front_block() {
    // Before the executor pops anything, no block is committed, so the front is freely optimizable: a fresh
    // queue must still run full look-ahead across the head (the freeze only arms after a pop).
    let mut planner = Planner::new(test_config());
    planner.plan_command(&mm_move(Some(0.05), None, None, 12000.0, false)).expect("queued");
    planner.plan_command(&mm_move(Some(20.0), None, None, 12000.0, false)).expect("queued");
    // The lone-head reverse-pass invariant: the FRONT (oldest) block still starts at rest (no previous
    // block), and the SECOND is forward-capped by accel over the first — i.e. look-ahead ran across the head.
    let first = planner.queue.iter().next().expect("first").entry_speed_sq;
    let second = planner.queue.iter().nth(1).expect("second").entry_speed_sq;
    assert!(first < 1e-3, "an un-committed front block still solves to rest at program start");
    assert!((second - 100.0).abs() < 1.0, "second is forward-capped by accel over the first (look-ahead ran)");
  }

  #[test]
  fn freeze_releases_when_the_busy_head_is_itself_popped() {
    // Freezing must track the head: when the busy head is popped, the NEXT block becomes the committed one,
    // and the previously-frozen value is gone. Popping twice must leave the new head frozen at its own
    // committed entry, never resurrect the old frozen block.
    let mut planner = Planner::new(test_config());
    for n in 1..=4 {
      planner.plan_command(&mm_move(Some(n as f32 * 10.0), None, None, 12000.0, false)).expect("queued");
    }
    planner.pop_block().expect("pop N");
    let head_after_first_pop = planner.peek_block().expect("head").entry_speed_sq;
    planner.pop_block().expect("pop N+1 (the previously frozen head)");
    let new_head = planner.peek_block().expect("new head").entry_speed_sq;
    // The new head is a different block; its committed entry is its own, independent of the old frozen one.
    assert!(new_head > 0.0, "the new committed head enters above rest in a continuous chain");
    // Enqueue more and recalculate: the new head must now be the frozen one and stay put.
    let committed = new_head;
    for n in 5..=10 {
      planner.plan_command(&mm_move(Some(n as f32 * 10.0), None, None, 12000.0, false)).expect("queued");
    }
    let after = planner.peek_block().expect("head").entry_speed_sq;
    assert!((after - committed).abs() < 1e-3, "the new head freezes at its own committed entry");
    let _ = head_after_first_pop;
  }

  // ---- Arc subdivision: segment count, chord tolerance, direction ------------------------------

  #[test]
  fn arc_segment_count_matches_chord_tolerance_formula() {
    // Quarter circle, r = 10 mm, tol = 0.002 mm. max_angle = acos(1 − 0.002/10) = acos(0.9998).
    let radius = 10.0;
    let tol = 0.002;
    let sweep = core::f32::consts::FRAC_PI_2; // 90°.
    let max_angle = libm::acosf(1.0 - tol / radius);
    let expected = libm::ceilf(sweep / max_angle) as u32;
    assert_eq!(arc_segment_count(radius, sweep, tol), expected);
    assert!(expected > 1); // a fine tolerance must produce many segments.
  }

  // A config with a coarse arc tolerance so a quarter circle subdivides into a queue-fitting handful of
  // segments (the default 0.002 mm tolerance on a 10 mm radius needs ~79 segments, exceeding the 32
  // block queue — that over-subdivision is exercised separately by `over_subdivided_arc_backpressures`).
  fn coarse_arc_config() -> PlannerConfig {
    PlannerConfig { arc_tolerance_mm: 0.05, ..test_config() }
  }

  fn ccw_quarter_arc() -> PlannerCommand {
    // Start at origin, center at (0, 10) via I0 J10, endpoint (10, 10): a true CCW quarter circle of
    // radius 10. The start is at the bottom of the circle (angle −90°); CCW sweeps +90° to the right
    // side (angle 0°), arriving at (10, 10).
    PlannerCommand::Arc {
      cw: false,
      axes: AxisWords { x: Some(10.0), y: Some(10.0), z: None, a: None },
      i: Some(0.0),
      j: Some(10.0),
      units: Units::Millimeter,
      distance: DistanceMode::Absolute,
      feed: 600.0,
      feed_mode: FeedMode::UnitsPerMin,
      machine_coords: false,
    }
  }

  #[test]
  fn quarter_arc_ccw_endpoint_is_reached() {
    let mut planner = Planner::new(coarse_arc_config());
    let outcome = planner.plan_command(&ccw_quarter_arc()).expect("queued");
    let segments = match outcome {
      PlannerOutcome::Queued { blocks } => blocks,
      other => panic!("expected queued blocks, got {other:?}"),
    };
    assert!(segments > 1);
    // After planning, the planner position is the arc endpoint: X10 Y10 → (1000, 1000) steps.
    assert_eq!(planner.position_steps(), [1000, 1000, 0, 0]);
  }

  #[test]
  fn arc_segments_stay_on_the_circle() {
    let mut planner = Planner::new(coarse_arc_config());
    planner.plan_command(&ccw_quarter_arc()).expect("queued");
    let center = [0.0_f32, 10.0_f32];
    let radius = 10.0_f32;
    // Walk the queued blocks accumulating the running position in steps; every vertex (each block's
    // endpoint) must lie on the circle within the chord tolerance plus step rounding, and each chord
    // must be short relative to the radius. This proves the subdivision tracks the true arc.
    let mut pos = [0i32; AXES];
    while let Some(block) = planner.pop_block() {
      assert!(block.millimeters < radius); // each chord is short.
      for (axis, p) in pos.iter_mut().enumerate() {
        *p += block.steps[axis];
      }
      let x = pos[0] as f32 / 100.0;
      let y = pos[1] as f32 / 100.0;
      let dist = libm::sqrtf((x - center[0]) * (x - center[0]) + (y - center[1]) * (y - center[1]));
      // Chord tolerance is 0.05 mm; allow a small extra margin for step-rounding at 100 steps/mm.
      assert!((dist - radius).abs() < 0.07, "vertex ({x}, {y}) off circle by {}", (dist - radius).abs());
    }
  }

  #[test]
  fn over_subdivided_arc_backpressures_without_mutating_state() {
    let mut planner = Planner::new(test_config()); // default 0.002 mm tolerance → ~79 segments.
    let start = planner.position_steps();
    // The arc needs more blocks than the queue holds; back-pressure must be all-or-nothing so the caller
    // can drain the executor and retry. Nothing is enqueued and the position is untouched on failure.
    assert_eq!(planner.plan_command(&ccw_quarter_arc()), Err(PlannerError::QueueFull));
    assert!(planner.is_empty());
    assert_eq!(planner.position_steps(), start);
  }

  #[test]
  fn arc_cw_and_ccw_sweep_opposite_directions() {
    // Both arcs start at the origin with center (0, 10) (start at the bottom of the circle, angle −90°).
    // CCW sweeps toward +X (right side, angle 0°) so its first vertex has X > 0; CW sweeps toward −X
    // (left side, angle −180°) so its first vertex has X < 0. The X sign of the first chord distinguishes
    // the rotation direction.
    let mut ccw = Planner::new(coarse_arc_config());
    ccw.plan_command(&ccw_quarter_arc()).expect("queued");
    let first_ccw = ccw.peek_block().expect("a block");
    assert!(first_ccw.steps[0] > 0); // CCW curves toward +X immediately.

    let mut cw = Planner::new(coarse_arc_config());
    cw.plan_command(&PlannerCommand::Arc {
      cw: true,
      axes: AxisWords { x: Some(-10.0), y: Some(10.0), z: None, a: None },
      i: Some(0.0),
      j: Some(10.0),
      units: Units::Millimeter,
      distance: DistanceMode::Absolute,
      feed: 600.0,
      feed_mode: FeedMode::UnitsPerMin,
      machine_coords: false,
    })
    .expect("queued");
    let first_cw = cw.peek_block().expect("a block");
    assert!(first_cw.steps[0] < 0); // CW curves toward −X immediately.
  }

  #[test]
  fn recalculate_once_equals_recalculate_per_segment() {
    // The arc refactor enqueues every segment first and runs look-ahead exactly once, rather than once
    // per segment. This is only valid if a single trailing `recalculate()` yields the SAME final entry
    // speeds as recalculating after each enqueue — because the reverse/forward passes always sweep the
    // whole queue, only the final queue contents matter. Prove it on a representative cornering chain.
    //
    // A chain of short collinear-ish moves with direction changes, sized to fit the 32-block queue. The
    // direction changes engage the junction limiter so the entry speeds are non-trivial (not all clamped
    // to nominal), making this a meaningful equivalence check rather than a degenerate one.
    let chain: [[i32; AXES]; 6] = [
      [200, 0, 0, 0],
      [400, 50, 0, 0],
      [600, 0, 0, 0],
      [800, 80, 0, 0],
      [1000, 0, 0, 0],
      [1200, 40, 0, 0],
    ];
    let feed = 12000.0;

    // Path A: recalculate after every segment (the pre-refactor behavior).
    let mut per_segment = Planner::new(test_config());
    for &target in &chain {
      let n =
        per_segment.enqueue_move(target, feed, Units::Millimeter, FeedMode::UnitsPerMin, false, false).expect("enqueued");
      assert_eq!(n, 1, "each chain step is a real move");
      per_segment.recalculate();
    }

    // Path B: enqueue every segment, then recalculate exactly once (the refactored arc behavior).
    let mut once = Planner::new(test_config());
    for &target in &chain {
      once.enqueue_move(target, feed, Units::Millimeter, FeedMode::UnitsPerMin, false, false).expect("enqueued");
    }
    once.recalculate();

    // The two queues must be block-for-block identical, most importantly in the planned entry speeds.
    assert_eq!(per_segment.queued_len(), once.queued_len());
    for (a, b) in per_segment.queue.iter().zip(once.queue.iter()) {
      assert!(
        (a.entry_speed_sq - b.entry_speed_sq).abs() < 1e-3,
        "entry speed mismatch: per-segment {} vs once {}",
        a.entry_speed_sq,
        b.entry_speed_sq,
      );
      assert!((a.max_entry_speed_sq - b.max_entry_speed_sq).abs() < 1e-3);
      assert_eq!(a.steps, b.steps);
    }
  }

  #[test]
  fn arc_without_offsets_is_invalid() {
    let mut planner = Planner::new(test_config());
    let arc = PlannerCommand::Arc {
      cw: true,
      axes: AxisWords { x: Some(10.0), y: Some(0.0), z: None, a: None },
      i: None,
      j: None,
      units: Units::Millimeter,
      distance: DistanceMode::Absolute,
      feed: 600.0,
      feed_mode: FeedMode::UnitsPerMin,
      machine_coords: false,
    };
    assert_eq!(planner.plan_command(&arc), Err(PlannerError::InvalidArc));
  }

  // ---- Non-motion commands: pass-through and look-ahead flush -----------------------------------

  #[test]
  fn dwell_passes_through_and_flushes_lookahead() {
    let mut planner = Planner::new(test_config());
    planner.plan_command(&mm_move(Some(10.0), None, None, 600.0, false)).expect("queued");
    let outcome = planner.plan_command(&PlannerCommand::Dwell { seconds: 2.5 }).expect("dwell");
    assert_eq!(outcome, PlannerOutcome::Dwell { seconds: 2.5 });
    // After a dwell the trailing junction state is cleared, so the next move starts from rest.
    planner.plan_command(&mm_move(Some(10.0), Some(10.0), None, 600.0, false)).expect("queued");
    // The post-dwell block is the newest; its entry must be 0 (no previous block to corner from).
    let last = planner.queue.iter().next_back().expect("a block");
    assert!(last.entry_speed_sq < 1e-3 || last.max_entry_speed_sq < 1e-3);
  }

  #[test]
  fn spindle_command_passes_through_without_motion() {
    let mut planner = Planner::new(test_config());
    let outcome = planner
      .plan_command(&PlannerCommand::Spindle { state: SpindleState::Clockwise, speed: 1000.0 })
      .expect("spindle");
    assert_eq!(outcome, PlannerOutcome::Spindle(SpindleState::Clockwise, 1000.0));
    assert!(planner.is_empty());
  }

  // --- DOC-07 spin-up gate: a dwell of `$392` is inserted before the first cutting move after M3/M4 ----------

  #[test]
  fn spin_up_gate_inserts_dwell_before_first_move_after_m3() {
    // M3 S1000 arms the spin-up; the next cutting move owes a dwell of $392 seconds ahead of it.
    let mut gate = SpinUpGate::new();
    assert!(!gate.is_pending());
    gate.note_spindle(SpindleState::Clockwise);
    assert!(gate.is_pending());
    // $392 = 0.5 s: the first move consumes the spin-up and asks for a 0.5 s dwell to be inserted ahead of it.
    assert_eq!(gate.take_dwell_before_move(0.5), Some(0.5));
    assert!(!gate.is_pending(), "the spin-up is consumed by the first move");
  }

  #[test]
  fn spin_up_gate_inserts_nothing_when_delay_is_zero() {
    // With $392 = 0 (grbl default) the spin-up is still consumed but NO dwell is inserted.
    let mut gate = SpinUpGate::new();
    gate.note_spindle(SpindleState::CounterClockwise);
    assert_eq!(gate.take_dwell_before_move(0.0), None);
    assert!(!gate.is_pending());
  }

  #[test]
  fn spin_up_gate_does_not_reinsert_on_a_second_move() {
    // The dwell is inserted before the FIRST move only; a second move after the same M3 gets nothing.
    let mut gate = SpinUpGate::new();
    gate.note_spindle(SpindleState::Clockwise);
    assert_eq!(gate.take_dwell_before_move(1.0), Some(1.0));
    assert_eq!(gate.take_dwell_before_move(1.0), None, "second move owes no spin-up");
  }

  #[test]
  fn spin_up_gate_stop_disarms_a_pending_spin_up() {
    // An M5 (or S0-as-stop) before any move clears the owed spin-up: a stopped spindle owes no spin-up dwell.
    let mut gate = SpinUpGate::new();
    gate.note_spindle(SpindleState::Clockwise);
    gate.note_spindle(SpindleState::Stop);
    assert!(!gate.is_pending());
    assert_eq!(gate.take_dwell_before_move(0.5), None);
  }

  #[test]
  fn program_end_flushes_lookahead() {
    let mut planner = Planner::new(test_config());
    planner.plan_command(&mm_move(Some(10.0), None, None, 600.0, false)).expect("queued");
    let outcome = planner.plan_command(&PlannerCommand::ProgramEnd).expect("end");
    assert_eq!(outcome, PlannerOutcome::ProgramEnd);
  }

  #[test]
  fn go_to_predefined_passes_through() {
    let mut planner = Planner::new(test_config());
    let cmd = PlannerCommand::GoToPredefined {
      is_g28: true,
      intermediate: AxisWords { x: None, y: None, z: Some(5.0), a: None },
      units: Units::Millimeter,
      distance: DistanceMode::Absolute,
    };
    let outcome = planner.plan_command(&cmd).expect("g28");
    assert_eq!(outcome, PlannerOutcome::GoToPredefined { is_g28: true });
  }

  // ---- G28/G30 predefined-position MOVE planning (DOC-05 group-0 motion) -------------------------

  // No axis words: a bare `G28`/`G30` recalls the stored MACHINE position in a single rapid, all axes.
  #[test]
  fn predefined_no_words_is_a_single_rapid_to_the_stored_machine_position() {
    let mut planner = Planner::new(test_config());
    // Move somewhere first so the recall is a real (non-zero) move from the current position.
    planner.plan_command(&mm_move(Some(10.0), Some(10.0), None, 600.0, true)).expect("queued");
    let no_words = AxisWords { x: None, y: None, z: None, a: None };
    let predefined = [5.0, 0.0, 2.0, 0.0];
    let blocks = planner
      .plan_go_to_predefined(&no_words, Units::Millimeter, DistanceMode::Absolute, predefined, None)
      .expect("recall");
    assert_eq!(blocks, 1, "no intermediate words => one recall rapid");
    let block = *planner.peek_block().expect("the recall block");
    assert!(block.rapid, "the recall is a rapid (G0)");
    assert_eq!(planner.position_steps(), [500, 0, 200, 0], "ends at the stored MACHINE position (mm * steps/mm)");
  }

  // The recall is in MACHINE coordinates: it must IGNORE the active work offset (like a G53 move).
  #[test]
  fn predefined_recall_ignores_the_work_offset() {
    let mut planner = Planner::new(test_config());
    planner.set_work_offset([10.0, 20.0, 0.0, 0.0]);
    let no_words = AxisWords { x: None, y: None, z: None, a: None };
    let predefined = [1.0, 1.0, 0.0, 0.0];
    let blocks = planner
      .plan_go_to_predefined(&no_words, Units::Millimeter, DistanceMode::Absolute, predefined, None)
      .expect("recall");
    assert_eq!(blocks, 1);
    // Machine X1 Y1 = 100,100 steps — the WCO of (10, 20) is NOT applied (recall is a G53-style machine move).
    assert_eq!(planner.position_steps(), [100, 100, 0, 0]);
  }

  // With axis words: FIRST a rapid to the work-coordinate intermediate point, THEN the recall — two blocks.
  #[test]
  fn predefined_with_words_does_intermediate_then_recall() {
    let mut planner = Planner::new(test_config());
    let inter = AxisWords { x: None, y: None, z: Some(5.0), a: None };
    let predefined = [0.0, 0.0, 0.0, 0.0];
    let blocks = planner
      .plan_go_to_predefined(&inter, Units::Millimeter, DistanceMode::Absolute, predefined, None)
      .expect("recall");
    assert_eq!(blocks, 2, "intermediate rapid + recall rapid");
    // The final committed position is the stored predefined (machine origin here).
    assert_eq!(planner.position_steps(), [0, 0, 0, 0]);
  }

  // The intermediate honors units (G20) and distance mode (G91); unspecified axes hold their current position.
  #[test]
  fn predefined_intermediate_honors_units_distance_and_holds_unspecified_axes() {
    let mut planner = Planner::new(test_config());
    // Start at machine X10 Y10 Z10 so an incremental intermediate and the "hold unspecified" rule are observable.
    planner.plan_command(&mm_move(Some(10.0), Some(10.0), Some(10.0), 600.0, true)).expect("queued");
    // The intermediate alone, checked by inspecting the planner state mid-sequence is awkward, so verify via a
    // predefined that equals the intermediate point: G91 inch Z1 => +25.4 mm on Z, X/Y hold at 10 mm.
    let inter = AxisWords { x: None, y: None, z: Some(1.0), a: None };
    // Predefined = the intermediate point in machine mm, so the recall is a zero-length no-op and only the
    // intermediate rapid enqueues — letting us assert the resolved intermediate directly via the final position.
    let predefined = [10.0, 10.0, 10.0 + 25.4, 0.0];
    let blocks = planner
      .plan_go_to_predefined(&inter, Units::Inch, DistanceMode::Incremental, predefined, None)
      .expect("recall");
    assert_eq!(blocks, 1, "intermediate moved; recall is zero-length and enqueues nothing");
    // X/Y held at 10 mm (1000 steps); Z advanced by 1 inch = 25.4 mm to 35.4 mm = 3540 steps.
    assert_eq!(planner.position_steps(), [1000, 1000, 3540, 0]);
  }

  // The rotary A axis recalls too, and a rotary A word in the intermediate is treated as degrees (never inch-scaled).
  #[test]
  fn predefined_recalls_rotary_a_and_intermediate_a_is_degrees() {
    let mut planner = Planner::new(test_config());
    // G20 inch with a rotary A word: A90 is 90 degrees, NOT 90 * 25.4 (DOC-10.1 rotary unit-scale suppression).
    let inter = AxisWords { x: None, y: None, z: None, a: Some(90.0) };
    // Predefined A = 90 deg in machine "mm" (degrees stored natively), so the recall is zero-length on A and only
    // the intermediate enqueues — isolating the intermediate's A scaling for the assertion.
    let predefined = [0.0, 0.0, 0.0, 90.0];
    let blocks = planner
      .plan_go_to_predefined(&inter, Units::Inch, DistanceMode::Absolute, predefined, None)
      .expect("recall");
    assert_eq!(blocks, 1, "A intermediate moved; recall is zero-length on A");
    // A90 degrees = 90 * 100 steps/mm = 9000 steps (NOT inch-scaled); a recall to the same value is a no-op.
    assert_eq!(planner.position_steps(), [0, 0, 0, 9000]);
  }

  // Bare recall already AT the stored position with no intermediate words is a zero-block no-op (no spurious motion).
  #[test]
  fn predefined_already_at_target_with_no_words_enqueues_nothing() {
    let mut planner = Planner::new(test_config());
    let no_words = AxisWords { x: None, y: None, z: None, a: None };
    let blocks = planner
      .plan_go_to_predefined(&no_words, Units::Millimeter, DistanceMode::Absolute, [0.0; AXES], None)
      .expect("recall");
    assert_eq!(blocks, 0, "already at machine origin (the never-stored default) => no motion");
  }

  // Regression: a G91 (incremental) recall must be ATOMIC under back-pressure. If only one queue slot is free but
  // the call needs two blocks (intermediate + recall), it must enqueue NEITHER and leave the committed position
  // un-advanced — otherwise a retry re-applies the incremental intermediate a SECOND time (double motion). This
  // guards the all-or-nothing capacity pre-check.
  #[test]
  fn predefined_incremental_back_pressure_is_atomic_no_double_count() {
    let mut planner = Planner::new(test_config());
    // Fill the queue to leave EXACTLY one free slot. Each move advances X by 1 mm so every block is distinct.
    for n in 1..BLOCK_QUEUE_LEN {
      planner.plan_command(&mm_move(Some(n as f32), None, None, 600.0, false)).expect("queued");
    }
    assert_eq!(planner.queued_len(), BLOCK_QUEUE_LEN - 1, "one free slot remains");
    let committed = planner.position_steps();
    // A G91 intermediate of +5 mm on Y, plus a recall to a DISTINCT machine point (X0 Y0 — non-zero from here),
    // so the call genuinely needs two blocks but only one slot is free.
    let inter = AxisWords { x: None, y: Some(5.0), z: None, a: None };
    let predefined = [0.0, 0.0, 0.0, 0.0];
    let result = planner.plan_go_to_predefined(&inter, Units::Millimeter, DistanceMode::Incremental, predefined, None);
    assert_eq!(result, Err(PlannerError::QueueFull), "two blocks needed, one slot free => QueueFull");
    assert_eq!(planner.queued_len(), BLOCK_QUEUE_LEN - 1, "NOTHING enqueued — queue length unchanged");
    assert_eq!(planner.position_steps(), committed, "committed position NOT advanced by the intermediate");
    // Drain one slot so two are free, then retry. The incremental intermediate must be applied EXACTLY once: Y
    // ends at the committed Y + 5 mm (500 steps), proving the retry did not double-count it.
    planner.pop_block().expect("drain one");
    let blocks = planner
      .plan_go_to_predefined(&inter, Units::Millimeter, DistanceMode::Incremental, predefined, None)
      .expect("recall fits now");
    assert_eq!(blocks, 2, "intermediate + recall both enqueue on the retry");
    // The FINAL committed position is the recall point (machine origin); the intermediate was a transient waypoint.
    // To prove the intermediate was applied exactly once we inspect the FIRST of the two new blocks: its Y delta is
    // +5 mm (500 steps) from the committed position, not +10 mm (which a double-count would have produced).
    let first_new = planner.queue.iter().nth(BLOCK_QUEUE_LEN - 2).expect("the intermediate block");
    assert_eq!(first_new.steps[1], 500, "intermediate Y delta is +5 mm exactly once, never +10");
  }

  // A soft-limited intermediate that leaves the envelope is rejected BEFORE any block is enqueued.
  #[test]
  fn predefined_intermediate_outside_envelope_is_rejected() {
    let mut planner = Planner::new(test_config());
    let inter = AxisWords { x: Some(10.0), y: None, z: None, a: None };
    let limits = SoftLimits { max_travel_mm: [5.0; AXES] };
    // Intermediate X10 is past +0 / -max_travel; the envelope is `[-5, 0]`, so X10 violates.
    let result = planner.plan_go_to_predefined(&inter, Units::Millimeter, DistanceMode::Absolute, [0.0; AXES], Some(limits));
    assert_eq!(result, Err(PlannerError::MoveExceedsTravel));
    assert_eq!(planner.position_steps(), [0, 0, 0, 0], "no block enqueued, position intact");
  }

  #[test]
  fn placeholder_block_derives_dominant_count_and_keeps_benign_trapezoid() {
    // A fixed-period block: the dominant `step_event_count` is the max axis magnitude, the signs are preserved,
    // and the trapezoid fields are non-degenerate placeholders the fixed-period stepper ignores.
    let block = Block::placeholder([-300, 50, 0, 0]);
    assert_eq!(block.steps, [-300, 50, 0, 0]);
    assert_eq!(block.step_event_count, 300, "dominant count = max axis magnitude");
    assert_eq!(block.millimeters, 1.0, "non-degenerate placeholder mm length");
    assert!(block.acceleration > 0.0 && block.nominal_speed_sq > 0.0, "non-degenerate placeholder trapezoid");
    assert!(!block.rapid && !block.jog);
  }

  // ---- G93 inverse-time feed (DOC-10.2) ---------------------------------------------------------

  /// Build a single-axis X move under a given feed mode for the inverse-time tests.
  fn x_move(x: f32, feed: f32, feed_mode: FeedMode) -> PlannerCommand {
    PlannerCommand::Move {
      rapid: false,
      axes: AxisWords { x: Some(x), y: None, z: None, a: None },
      units: Units::Millimeter,
      distance: DistanceMode::Absolute,
      feed,
      feed_mode,
      machine_coords: false,
    }
  }

  #[test]
  fn g93_inverse_time_nominal_speed_is_path_over_duration() {
    // G93 `F` is 1/(duration in minutes): a 10 mm move with F2 takes 0.5 min, so nominal = 10 mm / 0.5 min
    // = 20 mm/min = 10 * 2 / 60 mm/s. (test_config: 100 steps/mm, 6000 mm/min max rate → no clamp here.)
    let mut planner = Planner::new(test_config());
    planner.plan_command(&x_move(10.0, 2.0, FeedMode::InverseTime)).expect("queued");
    let block = *planner.peek_block().expect("a block");
    assert!((block.nominal_speed() - (10.0 * 2.0 / 60.0)).abs() < 1e-4, "nominal = mm * F / 60");
    let duration_min = block.millimeters / (block.nominal_speed() * 60.0);
    assert!((duration_min - 0.5).abs() < 1e-4, "duration must be 1/F = 0.5 min, got {duration_min}");
  }

  #[test]
  fn g93_and_g94_with_same_f_differ() {
    // The same `F2` means 2 mm/min under G94 but "0.5 min for this move" under G93 — distinct nominal speeds.
    // Regression guard against the pre-fix bug where G93 F was silently interpreted as units/min.
    let mut g94 = Planner::new(test_config());
    g94.plan_command(&x_move(10.0, 2.0, FeedMode::UnitsPerMin)).expect("queued");
    let mut g93 = Planner::new(test_config());
    g93.plan_command(&x_move(10.0, 2.0, FeedMode::InverseTime)).expect("queued");
    let n94 = g94.peek_block().expect("block").nominal_speed();
    let n93 = g93.peek_block().expect("block").nominal_speed();
    assert!((n94 - 2.0 / 60.0).abs() < 1e-4, "G94 F2 = 2 mm/min");
    assert!((n93 - 20.0 / 60.0).abs() < 1e-4, "G93 F2 over 10 mm = 20 mm/min");
    assert!(n93 > n94 * 9.0, "G93 here is ~10x faster than G94 for the same F");
  }

  #[test]
  fn g93_over_fast_duration_is_clamped_to_axis_rate() {
    // A commanded G93 duration faster than the per-axis max rate is floored to the achievable rate (DOC-10.2 Q3):
    // 10 mm with F1200 demands 200 mm/s, but the X axis caps at 6000 mm/min = 100 mm/s — the move finishes slower.
    let mut planner = Planner::new(test_config());
    planner.plan_command(&x_move(10.0, 1200.0, FeedMode::InverseTime)).expect("queued");
    let block = *planner.peek_block().expect("a block");
    assert!((block.nominal_speed() - 100.0).abs() < 1e-3, "clamped to the 100 mm/s axis rate, not 200 mm/s");
  }

  #[test]
  fn g93_arc_total_duration_is_one_over_f() {
    // A G93 arc's inverse-time F governs the WHOLE arc: a quarter arc with F1 must take exactly 1 min total
    // across every subdivided segment (the per-segment feed is scaled by the segment count, DOC-10.2). The
    // coarse arc config keeps the subdivided block count inside the queue, exactly like the other arc tests.
    let mut planner = Planner::new(coarse_arc_config());
    planner
      .plan_command(&PlannerCommand::Arc {
        cw: false,
        axes: AxisWords { x: Some(10.0), y: Some(10.0), z: None, a: None },
        i: Some(0.0),
        j: Some(10.0),
        units: Units::Millimeter,
        distance: DistanceMode::Absolute,
        feed: 1.0,
        feed_mode: FeedMode::InverseTime,
        machine_coords: false,
      })
      .expect("queued");
    // Sum the per-block cruise durations (millimeters / nominal). The chord-vs-arc-length error cancels exactly
    // because each equal-length segment is planned to take 1/(F * segments) min regardless of its chord.
    let total_s: f32 = planner.queue.iter().map(|b| b.millimeters / b.nominal_speed()).sum();
    assert!((total_s - 60.0).abs() < 0.1, "G93 arc should take 1/F = 60 s total, got {total_s}");
  }

  // ---- $376 rotary axis: detection, units fork, arc A-slaving (DOC-10.1/10.5/10.6) --------------

  /// A rapid move carrying only an A word, for the rotary-axis tests (test_config is 100 steps/unit per axis).
  fn a_move(a: f32, units: Units, rotary_mask: u8) -> (Planner, i32) {
    let mut cfg = test_config();
    cfg.rotary_mask = rotary_mask;
    let mut planner = Planner::new(cfg);
    planner
      .plan_command(&PlannerCommand::Move {
        rapid: true,
        axes: AxisWords { x: None, y: None, z: None, a: Some(a) },
        units,
        distance: DistanceMode::Absolute,
        feed: 0.0,
        feed_mode: FeedMode::UnitsPerMin,
        machine_coords: false,
      })
      .expect("queued");
    let steps = planner.peek_block().expect("a block").steps[A_AXIS];
    (planner, steps)
  }

  #[test]
  fn default_rotary_mask_marks_only_a_rotary() {
    let cfg = test_config();
    assert!(cfg.is_rotary(A_AXIS), "default $376 = 8 marks A rotary");
    assert!(!cfg.is_rotary(0) && !cfg.is_rotary(1) && !cfg.is_rotary(2), "X/Y/Z stay linear");
  }

  #[test]
  fn g20_inch_does_not_scale_a_rotary_word() {
    // A rotary A word is never inch-scaled (DOC-10.1): G20 A90 is 90 degrees → 90 * 100 steps/deg = 9000 steps,
    // NOT 90 * 25.4. test_config marks A rotary by default ($376 = 8).
    let (_, steps) = a_move(90.0, Units::Inch, DEFAULT_ROTARY_MASK);
    assert_eq!(steps, 9000, "rotary A bypasses inch scaling");
  }

  #[test]
  fn clearing_rotary_mask_makes_a_inch_scale_like_a_linear_axis() {
    // Runtime re-gating: clear A's $376 bit and the SAME G20 A90 now inch-scales (A is a 4th LINEAR axis):
    // 90 in * 25.4 mm/in * 100 steps/mm = 228600 steps. Proves $376 drives the units fork at runtime (DOC-10.1).
    let (_, steps) = a_move(90.0, Units::Inch, 0);
    assert_eq!(steps, 228_600, "non-rotary A inch-scales like X/Y/Z");
  }

  #[test]
  fn rotary_a_target_is_never_a_soft_limit_violation() {
    // End-to-end through plan_command_with_limits: a huge A target with A rotary is accepted despite a tiny $133.
    let mut planner = Planner::new(test_config());
    let limits = SoftLimits { max_travel_mm: [50.0; AXES] };
    let outcome = planner.plan_command_with_limits(
      &PlannerCommand::Move {
        rapid: true,
        axes: AxisWords { x: None, y: None, z: None, a: Some(100_000.0) },
        units: Units::Millimeter,
        distance: DistanceMode::Absolute,
        feed: 0.0,
        feed_mode: FeedMode::UnitsPerMin,
        machine_coords: false,
      },
      Some(limits),
    );
    assert!(matches!(outcome, Ok(PlannerOutcome::Queued { .. })), "rotary A is exempt from soft limits");
  }

  #[test]
  fn arc_slaves_a_linearly_like_the_z_helix() {
    // A G2/G3 with an A word produces an A-slaved helical arc (DOC-10.5): A advances linearly across the segments
    // to its endpoint, never circularly interpolated. A90 (rotary, no inch scale) = 9000 steps total.
    let mut planner = Planner::new(coarse_arc_config());
    planner
      .plan_command(&PlannerCommand::Arc {
        cw: false,
        axes: AxisWords { x: Some(10.0), y: Some(10.0), z: None, a: Some(90.0) },
        i: Some(0.0),
        j: Some(10.0),
        units: Units::Millimeter,
        distance: DistanceMode::Absolute,
        feed: 600.0,
        feed_mode: FeedMode::UnitsPerMin,
        machine_coords: false,
      })
      .expect("queued");
    let total_a: i32 = planner.queue.iter().map(|b| b.steps[A_AXIS]).sum();
    assert_eq!(total_a, 9000, "A reaches 90 deg total, slaved linearly across the arc");
  }

  #[test]
  fn pure_rotary_reversal_forces_a_junction_stop() {
    // Two opposite pure-rotary moves (A+10 then A−10) reverse the A direction: the 4-term junction dot is −1, so
    // the cornering model forces the second block's entry speed to 0 (DOC-10.3). A never moves linearly here.
    let mut planner = Planner::new(test_config());
    for target in [10.0f32, -10.0f32] {
      planner
        .plan_command(&PlannerCommand::Move {
          rapid: false,
          axes: AxisWords { x: None, y: None, z: None, a: Some(target) },
          units: Units::Millimeter,
          distance: DistanceMode::Absolute,
          feed: 600.0,
          feed_mode: FeedMode::UnitsPerMin,
          machine_coords: false,
        })
        .expect("queued");
    }
    // The trailing (reversing) block must start from rest.
    let reversing = planner.queue.back().expect("second block");
    assert_eq!(reversing.entry_speed_sq, 0.0, "a rotary reversal corners to a full stop");
  }
}
