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

use crate::gcode::{AxisWords, DistanceMode, PlannerCommand, Units};
use heapless::Deque;

/// Number of axes the planner coordinates (X, Y, Z) per DOC-02. The spare RMT channel's 4th axis is
/// out of scope until a 4th axis is wired, so the planner is fixed at three.
pub const AXES: usize = 3;

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
    }
  }
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
}

impl Default for PlannerConfig {
  /// grbl-like defaults useful for tests and first boot: 250 steps/mm, 500 mm/min max rate,
  /// 10 mm/s² acceleration on every axis, `$11` = 0.01 mm, `$12` = 0.002 mm.
  fn default() -> Self {
    PlannerConfig {
      steps_per_mm: [250.0; AXES],
      max_rate_mm_min: [500.0; AXES],
      accel_mm_s2: [10.0; AXES],
      junction_deviation_mm: 0.01,
      arc_tolerance_mm: 0.002,
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
  /// G92 set coordinate offset; the work offset was updated. No motion is produced and look-ahead is
  /// preserved (G92 does not move the machine), matching grbl.
  OffsetUpdated,
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
  /// Active feed rate (modal F) in `units` per minute, shared by every subdivided segment.
  feed: f32,
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
  /// Active G92 work offset in mm per axis, added to commanded work coordinates to get machine
  /// coordinates. Updated by [`PlannerCommand::SetCoordinateOffset`].
  work_offset_mm: [f32; AXES],
  /// Unit direction vector of the most recently planned block, for the next junction's cornering. The
  /// zero vector marks "no previous block" (start of program or after a flush): entry speed is 0.
  prev_unit_vec: [f32; AXES],
  /// Nominal speed squared of the most recently planned block, (mm/s)². Caps the junction speed so a
  /// corner never exceeds either adjoining block's cruise speed.
  prev_nominal_speed_sq: f32,
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
    }
  }

  /// The current machine position in steps per axis. Exposed for status reporting (MPos) and tests.
  pub fn position_steps(&self) -> [i32; AXES] {
    self.position_steps
  }

  /// The current machine position in mm per axis, derived from the step position and `$100–$102`.
  pub fn position_mm(&self) -> [f32; AXES] {
    let mut out = [0.0; AXES];
    for (axis, slot) in out.iter_mut().enumerate() {
      *slot = self.position_steps[axis] as f32 / self.config.steps_per_mm[axis];
    }
    out
  }

  /// The number of blocks currently queued for the motion executor.
  pub fn queued_len(&self) -> usize {
    self.queue.len()
  }

  /// True when the block ring buffer holds no blocks.
  pub fn is_empty(&self) -> bool {
    self.queue.is_empty()
  }

  /// Pop the oldest planned block for the motion executor (FIFO). Popping a block hands ownership of
  /// the realized motion to the segment generator; the planner's trailing junction state is unchanged
  /// because look-ahead is computed across the still-queued blocks.
  pub fn pop_block(&mut self) -> Option<Block> {
    self.queue.pop_front()
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
    match command {
      PlannerCommand::Move { rapid, axes, units, distance, feed } => {
        let target = self.resolve_target(axes, *units, *distance);
        let queued = self.plan_line(target, *feed, *units, *rapid)?;
        Ok(PlannerOutcome::Queued { blocks: queued })
      }
      PlannerCommand::Arc { cw, axes, i, j, units, distance, feed } => {
        let request = ArcRequest { cw: *cw, axes, i: *i, j: *j, units: *units, distance: *distance, feed: *feed };
        let queued = self.plan_arc(&request)?;
        Ok(PlannerOutcome::Queued { blocks: queued })
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
      PlannerCommand::SetCoordinateOffset { axes, units } => {
        self.apply_g92_offset(axes, *units);
        Ok(PlannerOutcome::OffsetUpdated)
      }
      PlannerCommand::ProgramEnd => {
        self.flush_lookahead();
        Ok(PlannerOutcome::ProgramEnd)
      }
    }
  }

  /// Resolve a line's axis words into an absolute machine target in *steps*, applying units, distance
  /// mode, and the active G92 work offset. Axes not mentioned on the line keep their current position.
  fn resolve_target(&self, axes: &AxisWords, units: Units, distance: DistanceMode) -> [i32; AXES] {
    let scale = units_scale(units);
    let words = [axes.x, axes.y, axes.z];
    let mut target = self.position_steps;
    for axis in 0..AXES {
      if let Some(value) = words[axis] {
        let value_mm = value * scale;
        let machine_mm = match distance {
          // Absolute work coordinates map to machine coordinates by adding the work offset.
          DistanceMode::Absolute => value_mm + self.work_offset_mm[axis],
          // Incremental words add to the current machine position; the offset is already baked in.
          DistanceMode::Incremental => {
            self.position_steps[axis] as f32 / self.config.steps_per_mm[axis] + value_mm
          }
        };
        target[axis] = mm_to_steps(machine_mm, self.config.steps_per_mm[axis]);
      }
    }
    target
  }

  /// Apply a G92 offset so the current machine position reads as the commanded work values. The offset
  /// for a mentioned axis is `machine_position − commanded_value`; unmentioned axes keep their offset.
  fn apply_g92_offset(&mut self, axes: &AxisWords, units: Units) {
    let scale = units_scale(units);
    let words = [axes.x, axes.y, axes.z];
    for (axis, word) in words.iter().enumerate() {
      if let Some(value) = word {
        let machine_mm = self.position_steps[axis] as f32 / self.config.steps_per_mm[axis];
        self.work_offset_mm[axis] = machine_mm - value * scale;
      }
    }
  }

  /// Plan a single straight-line move to an absolute step target. Builds the block, enqueues it, runs
  /// look-ahead, and advances the planner position. Returns 1 if a block was enqueued, 0 for a no-op
  /// move (target equals current position).
  fn plan_line(&mut self, target: [i32; AXES], feed: f32, units: Units, rapid: bool) -> Result<usize, PlannerError> {
    let block = match self.build_block(target, feed, units, rapid) {
      Some(block) => block,
      None => return Ok(0),
    };
    self.enqueue(block)?;
    self.position_steps = target;
    self.prev_unit_vec = block.unit_vec;
    self.prev_nominal_speed_sq = block.nominal_speed_sq;
    self.recalculate();
    Ok(1)
  }

  /// Build a block from the current position to an absolute step `target`. Returns `None` for a
  /// zero-length move. Computes the step delta, dominant-axis count, unit vector and mm travel, the
  /// limiting acceleration and nominal speed, and the junction-deviation entry-speed cap.
  fn build_block(&self, target: [i32; AXES], feed: f32, units: Units, rapid: bool) -> Option<Block> {
    let mut steps = [0i32; AXES];
    let mut delta_mm = [0.0f32; AXES];
    let mut step_event_count = 0u32;
    for axis in 0..AXES {
      let d = target[axis] - self.position_steps[axis];
      steps[axis] = d;
      step_event_count = step_event_count.max(d.unsigned_abs());
      delta_mm[axis] = d as f32 / self.config.steps_per_mm[axis];
    }
    let millimeters = libm::sqrtf(delta_mm[0] * delta_mm[0] + delta_mm[1] * delta_mm[1] + delta_mm[2] * delta_mm[2]);
    if millimeters < LENGTH_EPSILON_MM || step_event_count == 0 {
      return None;
    }
    let inv_mm = 1.0 / millimeters;
    let unit_vec = [delta_mm[0] * inv_mm, delta_mm[1] * inv_mm, delta_mm[2] * inv_mm];

    let acceleration = limiting_acceleration(&unit_vec, &self.config.accel_mm_s2);
    let nominal_speed = self.nominal_speed_mm_s(feed, units, rapid, &unit_vec);
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
    })
  }

  /// The block's nominal (cruise) speed in mm/s. For a rapid (G0) the speed is governed by the per-axis
  /// maximum rates; for a feed move it is the requested feed (mm/min → mm/s), each clamped so no
  /// participating axis exceeds its own maximum rate along the unit vector.
  fn nominal_speed_mm_s(&self, feed: f32, units: Units, rapid: bool, unit_vec: &[f32; AXES]) -> f32 {
    let axis_rate_limit = self.axis_rate_limit_mm_s(unit_vec);
    if rapid {
      // A rapid has no feed word; it cruises at the most restrictive axis rate limit.
      return axis_rate_limit;
    }
    // Feed words are in the active units per minute; convert inch/min to mm/min, then to mm/s.
    let feed_mm_min = feed * units_scale(units);
    let feed_mm_s = feed_mm_min / 60.0;
    feed_mm_s.min(axis_rate_limit).max(0.0)
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
    // grbl: junction_cos_theta = -dot(prev, curr). For a straight continuation prev==curr so the dot is
    // +1 and cos_theta = -1 (no restriction); for a full reversal the dot is -1 and cos_theta = +1.
    let prev = &self.prev_unit_vec;
    let dot = prev[0] * unit_vec[0] + prev[1] * unit_vec[1] + prev[2] * unit_vec[2];
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
  fn reverse_pass(&mut self) {
    let mut next_entry_sq = 0.0f32;
    for block in self.queue.iter_mut().rev() {
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
    let target = self.arc_endpoint_mm(request.axes, scale, request.distance, &start);

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
    let theta_start = libm::atan2f(r0[1], r0[0]);
    let theta_step = sweep / segments as f32;

    let mut enqueued = 0usize;
    for seg in 1..=segments {
      let theta = theta_start + theta_step * seg as f32;
      let x = center[0] + radius * libm::cosf(theta);
      let y = center[1] + radius * libm::sinf(theta);
      let z = z_start + z_delta * (seg as f32 / segments as f32);
      let seg_target = self.mm_target_to_steps(&[x, y, z]);
      // Each segment is a linear feed move; the arc feed applies (already in active units). Subdivided
      // segments share the requested feed; cornering between them keeps the path smooth via look-ahead.
      enqueued += self.plan_line(seg_target, request.feed, request.units, false)?;
    }
    Ok(enqueued)
  }

  /// Resolve an arc endpoint to absolute machine mm, honouring units and distance mode. Unmentioned
  /// axes keep the start position. Returns `[x, y, z]` in machine mm.
  fn arc_endpoint_mm(&self, axes: &AxisWords, scale: f32, distance: DistanceMode, start: &[f32; AXES]) -> [f32; AXES] {
    let words = [axes.x, axes.y, axes.z];
    let mut endpoint = *start;
    for axis in 0..AXES {
      if let Some(value) = words[axis] {
        endpoint[axis] = match distance {
          DistanceMode::Absolute => value * scale + self.work_offset_mm[axis],
          DistanceMode::Incremental => start[axis] + value * scale,
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
    }
  }

  fn mm_move(x: Option<f32>, y: Option<f32>, z: Option<f32>, feed: f32, rapid: bool) -> PlannerCommand {
    PlannerCommand::Move {
      rapid,
      axes: AxisWords { x, y, z },
      units: Units::Millimeter,
      distance: DistanceMode::Absolute,
      feed,
    }
  }

  // ---- Target resolution: units, offsets, distance mode, steps/mm -------------------------------

  #[test]
  fn resolve_absolute_mm_target_to_steps() {
    let planner = Planner::new(test_config());
    let axes = AxisWords { x: Some(10.0), y: Some(-5.0), z: None };
    let target = planner.resolve_target(&axes, Units::Millimeter, DistanceMode::Absolute);
    // 10 mm × 100 steps/mm = 1000; -5 mm × 100 = -500; Z unmentioned stays at 0.
    assert_eq!(target, [1000, -500, 0]);
  }

  #[test]
  fn resolve_inch_target_scales_by_25_4() {
    let planner = Planner::new(test_config());
    let axes = AxisWords { x: Some(1.0), y: None, z: None };
    let target = planner.resolve_target(&axes, Units::Inch, DistanceMode::Absolute);
    // 1 inch = 25.4 mm × 100 steps/mm = 2540 steps.
    assert_eq!(target, [2540, 0, 0]);
  }

  #[test]
  fn resolve_incremental_adds_to_current_position() {
    let mut planner = Planner::new(test_config());
    // Move to X10 absolute first so the position advances to 1000 steps.
    planner.plan_command(&mm_move(Some(10.0), None, None, 600.0, false)).expect("queued");
    let axes = AxisWords { x: Some(2.5), y: None, z: None };
    let target = planner.resolve_target(&axes, Units::Millimeter, DistanceMode::Incremental);
    // 1000 steps (10 mm) + 2.5 mm × 100 = 1250 steps.
    assert_eq!(target, [1250, 0, 0]);
  }

  #[test]
  fn g92_offset_makes_current_position_read_commanded_value() {
    let mut planner = Planner::new(test_config());
    planner.plan_command(&mm_move(Some(10.0), Some(20.0), None, 600.0, false)).expect("queued");
    // At machine (10, 20) declare the work position to be (0, 0); the offset becomes the machine pos.
    let g92 = PlannerCommand::SetCoordinateOffset {
      axes: AxisWords { x: Some(0.0), y: Some(0.0), z: None },
      units: Units::Millimeter,
    };
    assert_eq!(planner.plan_command(&g92).expect("offset"), PlannerOutcome::OffsetUpdated);
    // Now an absolute G0 X0 Y0 must map back to machine (10, 20) — i.e. produce no motion.
    let axes = AxisWords { x: Some(0.0), y: Some(0.0), z: None };
    let target = planner.resolve_target(&axes, Units::Millimeter, DistanceMode::Absolute);
    assert_eq!(target, [1000, 2000, 0]);
  }

  // ---- Block geometry: unit vector, mm travel, dominant axis ------------------------------------

  #[test]
  fn pure_x_move_has_unit_vector_and_mm() {
    let mut planner = Planner::new(test_config());
    planner.plan_command(&mm_move(Some(10.0), None, None, 600.0, false)).expect("queued");
    let block = planner.peek_block().expect("a block");
    assert_eq!(block.steps, [1000, 0, 0]);
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
    assert_eq!(first.steps, [100, 0, 0]); // 0 → 1 mm
    assert_eq!(second.steps, [200, 0, 0]); // 1 → 3 mm
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
      axes: AxisWords { x: Some(10.0), y: Some(10.0), z: None },
      i: Some(0.0),
      j: Some(10.0),
      units: Units::Millimeter,
      distance: DistanceMode::Absolute,
      feed: 600.0,
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
    assert_eq!(planner.position_steps(), [1000, 1000, 0]);
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
      axes: AxisWords { x: Some(-10.0), y: Some(10.0), z: None },
      i: Some(0.0),
      j: Some(10.0),
      units: Units::Millimeter,
      distance: DistanceMode::Absolute,
      feed: 600.0,
    })
    .expect("queued");
    let first_cw = cw.peek_block().expect("a block");
    assert!(first_cw.steps[0] < 0); // CW curves toward −X immediately.
  }

  #[test]
  fn arc_without_offsets_is_invalid() {
    let mut planner = Planner::new(test_config());
    let arc = PlannerCommand::Arc {
      cw: true,
      axes: AxisWords { x: Some(10.0), y: Some(0.0), z: None },
      i: None,
      j: None,
      units: Units::Millimeter,
      distance: DistanceMode::Absolute,
      feed: 600.0,
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
      intermediate: AxisWords { x: None, y: None, z: Some(5.0) },
      units: Units::Millimeter,
      distance: DistanceMode::Absolute,
    };
    let outcome = planner.plan_command(&cmd).expect("g28");
    assert_eq!(outcome, PlannerOutcome::GoToPredefined { is_g28: true });
  }
}
