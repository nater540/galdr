//! Shared global state for the USB CDC comms subsystem (DOC-08): the `static`/`const` declarations, the
//! cross-core atomics / signals / blocking cells, and their thin synchronous accessors plus boot `init_*`
//! seeders. This is the state layer extracted verbatim from `comms.rs` (architecture-refactor A1, step 1); it
//! holds NO task or protocol logic — only the storage every comms task shares and the cheap getters/setters
//! over it. `comms.rs` re-exports this module (`pub(crate) use state::*;`) so both external `crate::comms::X`
//! callers and the sibling comms code keep resolving these names unqualified, exactly as before the split.

use core::cell::Cell;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, AtomicU8, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::pipe::Pipe;
use embassy_sync::signal::Signal;

use firmware_core::coords::{CoordinatePersistent, CoordinateSystems};
use firmware_core::homing::{HomingConfig, HomingError};
use firmware_core::planner::{Planner, PlannerConfig, AXES};
use firmware_core::protocol::{
  ControlState, LastProbe, MachineSnapshot, Overrides, PositionReport, RefreshReporter, MAX_LINE_LEN,
  RESPONSE_CAPACITY, RX_BUFFER_SIZE,
};
use firmware_core::settings::Settings;

// The only outbound reference from the moved state block: the pure `$110-112` fold used to seed the cached
// `StatusCfg`. It still lives in `comms.rs`; a child module may name a parent's private item via `super`.
use super::min_axis_max_rate;

/// A single assembled input line handed from `usb_rx` to the parser stub, capped to the protocol line
/// length. Owned (not borrowed) so it can cross the channel without referencing the RX task's buffer.
pub type Line = heapless::Vec<u8, MAX_LINE_LEN>;

/// A fully-formatted outgoing response (banner / `ok` / `error:N` / status / `$`-report line), rendered
/// by a firmware-core formatter and queued for the single USB writer. Sized to hold any one Stage-1
/// response line; multi-line `$` responses are queued as several `Response`s in order.
pub type Response = heapless::String<RESPONSE_CAPACITY>;

/// Depth of the accepted-line channel from `usb_rx` to the parser stub. DOC-01 specifies 4; with simple
/// send-response streaming one in-flight line is the norm, so 4 is comfortable headroom.
pub const LINE_QUEUE_DEPTH: usize = 4;

/// Depth of the outgoing response channel to `usb_tx`. DOC-01 specifies 8; a `$I+`/`$$` burst queues
/// several lines at once, so 8 keeps multi-line replies from blocking the producer.
pub const RESPONSE_QUEUE_DEPTH: usize = 8;

/// Capacity of the RX byte buffer, in bytes. Sized to exactly the advertised [`RX_BUFFER_SIZE`] so the
/// number reported to the host (in `[OPT:]` and `Bf:`) is truthfully backed by real buffer space, and so a
/// compliant host's character-counting send-ahead window can never overrun it. This is the invariant that
/// lets the reader half's pipe write be non-blocking: a host is only permitted `RX_BUFFER_SIZE` outstanding
/// non-real-time bytes, and the pipe can hold exactly that many.
pub const RX_PIPE_CAPACITY: usize = RX_BUFFER_SIZE;

/// The real RX byte buffer between the reader half ([`usb_rx`]) and the line-assembly half
/// ([`line_assembler`]). The reader pushes every non-real-time byte here; the assembler drains it through
/// the line framer. Its capacity is the advertised RX buffer size, making that advertisement truthful.
pub static RX_PIPE: Pipe<CriticalSectionRawMutex, RX_PIPE_CAPACITY> = Pipe::new();

/// Accepted GCode/`$` lines awaiting the consumer.
pub static LINE_QUEUE: Channel<CriticalSectionRawMutex, Line, LINE_QUEUE_DEPTH> = Channel::new();

/// Soft-reset notification for the line-assembly half: set by [`usb_rx`] on `0x18`/`0x19` so the assembler
/// drops any partially framed line (the consumer's pipeline reset is signalled separately via
/// [`SOFT_RESET`]). A dedicated Signal per waiter avoids two tasks racing to consume one Signal.
pub static LINE_RESET: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Outgoing responses awaiting the single USB writer. Every task that needs to emit bytes enqueues here
/// so the USB endpoint has exactly one writer (DOC-08).
pub static RESPONSE: Channel<CriticalSectionRawMutex, Response, RESPONSE_QUEUE_DEPTH> = Channel::new();

/// `?` (status report request) — set by `usb_rx`, consumed by `status_responder`.
pub static STATUS_REQUEST: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// The hold LEVEL (Finding #11): `true` while the core-1 motion executor must stay parked at the next block
/// boundary, `false` while it may run. This is a LEVEL, not an edge — it is the authoritative source of truth
/// the executor reads at EVERY block boundary AND in its empty-queue wait, so a hold can never be "missed" or
/// "drained as stale" the way the old edge-`Signal` pair could (the root cause of findings #1-#4). It is SET
/// by a feed-hold (`!`), by `$SLP`, and by a jog-cancel (which reuses the boundary stop); it is CLEARED by a
/// genuine cycle-start resume (`~` from a hold — gated by [`ControlState::resumes_on_cycle_start`]) and by a
/// soft reset (which must leave NO stale hold latched — Finding #2). Paired with [`HOLD_WAKE`] for promptness
/// and [`MOTION_PARKED`] for the quiesce acknowledgment. `AcqRel`/`Acquire` publishes the level across cores.
pub static HOLD_REQUESTED: AtomicBool = AtomicBool::new(false);

/// A promptness wake for the executor whenever [`HOLD_REQUESTED`] CHANGES (set or cleared). The level itself is
/// authoritative — the executor re-reads it after every wake — so this `Signal` only needs to nudge the
/// executor out of an `.await` (the empty-queue wait, or the parked-on-hold wait) to re-evaluate the level
/// promptly; a coalesced or even missed wake loses nothing because the executor always re-reads the level. Set
/// by `!`/`~`/`$SLP`/jog-cancel after they mutate [`HOLD_REQUESTED`], and by the soft-reset clear.
pub static HOLD_WAKE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// The executor's quiesce ACKNOWLEDGMENT (Finding #11 / #3): the core-1 executor pulses this each time it
/// PARKS at a block boundary on the hold level (and after a block abort), i.e. each time it has actually come
/// to rest with no block in flight. The shared [`quiesce_executor`] primitive sets [`HOLD_REQUESTED`], wakes
/// the executor, and then AWAITS this — so jog-cancel and probe-abort observe a real "the executor has parked"
/// fact rather than racing the `EXECUTOR_RUNNING` clear with a blind cycle-start. A dedicated `Signal` per
/// waiter (only [`quiesce_executor`] awaits it) keeps it from being stolen by another task.
pub static MOTION_PARKED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// `0x85` (jog cancel, DOC-08 Phase D) — set by the non-blocking reader half when a jog is in flight, consumed
/// by `comms_consumer` (which owns the planner). The consumer reuses the feed-hold block-boundary stop to
/// decelerate the active jog block to its boundary, flushes the trailing jog blocks, syncs the planner's
/// commanded position to the actual live stop point, and returns the machine to Idle. A dedicated Signal per
/// waiter (the same rule [`SOFT_RESET`]/[`MOTION_RESET`] follow) so it is never lost to another waiter.
pub static JOG_CANCEL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// `0x86` (graceful program stop, Galdr extension) — set by the non-blocking reader half ONLY when the live
/// [`ControlState::program_stop_quiesces`] gate is true (Run/Hold), consumed by `comms_consumer` (which owns the
/// planner). The consumer runs [`program_stop_cycle`]: it flushes the WHOLE planner queue + any in-progress arc,
/// parks the executor at the active block's boundary with a real acknowledgment (reusing [`quiesce_executor`]),
/// syncs the planner's commanded position to the live stop point, clears the program/modal-run state (mirroring
/// `M30`), and returns the machine to Idle (NOT alarm) with position RETAINED. It differs from `0x18` ([`SOFT_RESET`])
/// which aborts to `ALARM:3` and re-emits the banner. Like [`SOFT_RESET`], this single `Signal` is `.wait()`'d at
/// both the consumer's main-loop select AND the back-pressure/arc-drive waits, which is safe because the consumer is
/// only ever blocked at ONE of those at a time (an embassy `Signal` wakes exactly one waiter).
pub static PROGRAM_STOP: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// `0x18` (soft reset) — set by `usb_rx`, consumed by `comms_consumer` (it rebuilds the parser/planner and
/// emits the banner) and by `plan_command`'s back-pressure retry. The CORE-1 motion executor does NOT share
/// this Signal — it has its own [`MOTION_RESET`] — because an embassy `Signal` wakes only one waiter
/// (Finding #3).
pub static SOFT_RESET: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Dedicated soft-reset notification for the CORE-1 motion executor (Finding #3). An embassy `Signal` wakes
/// exactly ONE waiter, so the single [`SOFT_RESET`] cannot be shared by the consumer, the back-pressure
/// retry, AND the cross-core executor — whichever waiter happens to win consumes it and the others miss the
/// reset. The `0x18` dispatch fires this in addition to [`SOFT_RESET`], giving the executor its own guaranteed
/// wake (the same "dedicated Signal per waiter" rule [`LINE_RESET`] follows). The executor races it between
/// blocks AND tests it between bursts (via [`MOTION_RESET_PENDING`]) so an in-flight block aborts promptly,
/// then zeroes the live step position.
pub static MOTION_RESET: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// A poll-able "a soft reset is pending for the executor" flag, set alongside [`MOTION_RESET`] by the `0x18`
/// dispatch. The motion executor's step sink tests this BETWEEN bursts (not mid-burst — a burst in flight is
/// never split) and returns a [`StepError`] to abort the current `run_block` early, so a reset during a long
/// (multi-second) block zeroes the position within one burst rather than after the whole block (Finding #3).
/// It is a separate `AtomicBool` rather than a second `Signal` read so the sink can peek it WITHOUT consuming
/// the wake that the executor's between-block `select` also needs. The executor clears it when it services
/// the reset. `AcqRel`/`Acquire` ordering publishes the flag across cores on the S3.
pub static MOTION_RESET_PENDING: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

/// The live machine position in whole steps per axis, `[X, Y, Z]`, published by the core-1 motion executor's
/// counting sink after EACH burst (decoupled from the executor's block-level `.await`) and read by
/// [`status_responder`] to render a genuinely live MPos (Finding #5). 32-bit atomics are native single-cycle
/// loads/stores on the S3, so the cross-core publish is lock-free. This is the SINGLE owner of the live
/// position: the executor zeroes it on a soft reset, and nothing else writes it — so there is no stale
/// overwrite race with the consumer's pipeline reset (which no longer touches MPos). Steps→mm conversion for
/// the report stays in the host-tested [`steps_to_mm`].
pub static LIVE_POSITION: [AtomicI32; AXES] =
  [AtomicI32::new(0), AtomicI32::new(0), AtomicI32::new(0), AtomicI32::new(0)];

/// The live LOGICAL limit-switch state as a per-axis bitmask (`bit0 = X`, `bit1 = Y`, `bit2 = Z`), published by
/// the core-1 motion executor and read by [`status_responder`] to source the `Pn:` X/Y/Z letters (DOC-06 / DOC-08).
/// Each bit is the TRIGGERED state AFTER the live `$5` invert (and the NC fail-safe) is applied at write time
/// through the host-tested [`limit_triggered`](firmware_core::hal_traits::limit_triggered) — exactly mirroring how
/// [`PROBE_ASSERTED`] stores the post-`$6`-invert logical probe state, so the reader carries no settings or
/// electrical knowledge and the `Pn:` letter assembly stays the pure, host-tested [`PinReport`] path. A single
/// `AtomicU8` is a native lock-free cross-core publish on the S3 (`Release` store / `Acquire` load).
///
/// CRITICAL — this tracks RELEASE, not just assert. The idle limit detector parks on `wait_for_rising_edge`, which
/// fires only on a press and NEVER on a release, so publishing solely on that edge would latch a stale "triggered"
/// forever after the switch opens. The executor instead republishes this mask from a level SAMPLE at every point it
/// already reads the pins — each idle `Ticker` tick (~50 ms), every block boundary, and after homing — so the
/// reported state follows both press and release within one tick at idle, with NO work added to the real-time
/// burst path (the tick arm runs only in the empty-queue idle `select`). Defaults to `0` (nothing triggered).
pub static LIMIT_LEVELS: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);

/// Decode the [`LIMIT_LEVELS`] bitmask into the per-axis `[X, Y, Z]` logical-triggered array the [`PinReport`]
/// expects. An `Acquire` load pairs with the executor's `Release` publish so the core-0 reader sees a coherent
/// mask. Kept beside the static so the bit layout (`bit0 = X`, `bit1 = Y`, `bit2 = Z`) has a single definition.
pub fn limit_levels() -> [bool; AXES] {
  let mask = LIMIT_LEVELS.load(Ordering::Acquire);
  core::array::from_fn(|axis| mask & (1 << axis) != 0)
}

/// Planner → motion-executor readiness signal (DOC-01). Set by [`plan_command`] after it enqueues a motion
/// block, so the core-1 `motion_executor` can AWAIT a fresh block when it finds the queue empty instead of
/// polling (the Stage-1 stub polled, which the review flagged). Living here in the bin keeps the pure
/// planner lib free of any async primitive: the planner reports a `Queued` outcome and the consumer raises
/// the signal. A `Signal` (not a counter) is sufficient because the executor re-checks the queue under the
/// lock after each wake and loops until it is drained, so a coalesced multi-block signal loses no block.
pub static BLOCK_AVAILABLE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Motion-executor → consumer "a queue slot just freed" wake (Bug 4: chunk-boundary decelerate-to-stop on large
/// arcs). Raised by the core-1 executor the instant it pops a block, so [`drive_pending_arc`] refills an
/// in-progress over-subdivided arc PROACTIVELY — the moment a slot opens — rather than only after the fixed
/// [`QUEUE_FULL_RETRY`] poll interval. Keeping the planner buffer topped up while an arc is pending stops the
/// executor from draining down to the look-ahead's forced-stop chunk tail before the next chunk arrives, which
/// otherwise stamps a dwell mark (a physical decelerate-to-stop and re-accelerate) at every ~`BLOCK_QUEUE_LEN`
/// segment boundary of the milled curve. A `Signal` (not a counter) suffices: `drive_pending_arc` re-checks the
/// queue under the lock after each wake and resumes until the arc completes, so a coalesced multi-pop wake is
/// fine. It is ONLY consumed by the arc-drive loop, so a slot-freed wake raised while no arc is pending is simply
/// overwritten by the next pop and costs nothing. The timer race remains as a backstop so a missed/foregone
/// signal (e.g. the executor parked on a hold) can never wedge the drive loop.
pub static SLOT_FREED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Limit-switch rising-edge trip → executor idle-trip seam (DOC-06). The core-1 executor's idle loop awaits a
/// rising edge on the X/Y/Z limit pins via the interrupt-driven `Input::wait_for_rising_edge`, runs the `$26`
/// debounce resample, and on a CONFIRMED trip signals this static (`wait_for_limit_trip` in [`crate::motion`]) —
/// then samples the switches via `check_hard_limits` and raises `ALARM:1` if `$21` is on and a switch tripped
/// WHILE IDLE (the in-motion case is already covered by the block-boundary `check_hard_limits` call). The
/// hard-limit alarm is suppressed while a homing cycle is active (`HOMING_ACTIVE`) — the switches are expected to
/// trip then (research finding #17).
///
/// HARDWARE-BOUNDARY NOTE (compile-checked only, no hardware): the rising-edge interrupt routing, the pull-up
/// electrical behavior, the broken-wire fail-safe, EMI rejection, and the exact debounce timing are the boundary
/// the research findings flag as untestable on the host — they must be verified on hardware. The producer
/// (`wait_for_limit_trip`) IS wired now: the edge wait + `$26` resample run and signal this static on a confirmed
/// trip. This static is the documented seam; today only the core-1 executor produces and observes it.
pub static LIMIT_TRIGGERED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Executor → consumer: "a hard limit FRESHLY tripped during normal motion" (DOC-06). The core-1 executor samples
/// the limit inputs at block boundaries and on the [`LIMIT_TRIGGERED`] wake; when
/// [`firmware_core::homing::hard_limit_alarm_armed`] says an alarm is due ([`HARD_LIMITS_ENABLED`] set, homing NOT
/// active, and an axis made a not-triggered -> triggered EDGE since the last sample) it halts motion and raises this
/// signal. The edge-arming means a switch the machine is merely parked on (held level — e.g. after an aborted `$H`
/// seek) never raises it, so no stale trip survives a soft reset to re-lock the machine (the `error:9` fix). The
/// consumer enters `ALARM:1` ([`AlarmCode::HardLimit`], a LOCKED alarm — position is likely lost from the abrupt
/// stop) and runs the pipeline reset. A coalesced `Signal` is sufficient: the alarm latches, so a second trip
/// before the first is serviced is harmless.
pub static HARD_LIMIT_TRIPPED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Executor → consumer: "an unrecoverable mid-block step-output Transport fault truncated a cutting move" (§15.6 /
/// task #22). The core-1 executor raises this from [`run_block`](crate::motion) when [`SegmentGenerator::run_block`]
/// returns an error sourced from an `emit_burst` arm (a bounded RMT `wait()` error, a failed `transmit()` start, or a
/// burst-too-long) — a real step-sync break that abandons the rest of the block. The consumer enters the LOCKED
/// `ALARM:17` ([`AlarmCode::MotorFault`](firmware_core::protocol::AlarmCode::MotorFault)) and runs the pipeline reset,
/// halting the program and forcing a re-home — the grbl lost-step-sync contract (§14.3), NEVER silent abandonment or
/// a silent reset that would cut a wrong part. A coalesced `Signal` suffices: the alarm latches, so a second fault
/// before the first is serviced is harmless (the machine is already halted into the locked alarm).
pub static MOTION_FAULT: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// A `G38.x` probe request handed from the core-0 consumer to the core-1 motion executor (Phase C, DOC-09). It
/// carries everything the probe-watching execution path needs: the absolute MACHINE step `target` the probe
/// seeks (the no-contact end of travel), the per-tick `step_period_ticks` derived from the probe feed, and the
/// `toward` sense (stop on TRIGGER for G38.2/.3, on RELEASE for G38.4/.5). `Copy` so it rides a `Signal`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProbeRequest {
  /// The absolute MACHINE step target the probe seeks toward (the no-contact end of travel).
  pub target: [i32; AXES],
  /// The fixed per-tick step period in motion timer ticks, derived from the probe `F` word and `$100..102`.
  pub step_period_ticks: u32,
  /// `true` to stop on the probe TRIGGER edge (toward modes G38.2/.3); `false` to stop on RELEASE (away modes
  /// G38.4/.5).
  pub toward: bool,
  /// The live `$6` probe-pin invert, snapshotted by the consumer and applied by the executor's probe read (via
  /// [`firmware_core::hal_traits::probe_triggered`]) so the sample sense matches the host-tested setting. `$19`
  /// (pull-up) is a pin-config concern applied at GPIO bring-up, not carried here.
  pub invert: bool,
}

/// The outcome the core-1 executor publishes back after running a probe cycle: whether the expected edge was
/// seen and the latched MACHINE step position at the stop point. The consumer turns this into the `[PRB:]` push,
/// the `$#` last-probe slot, the planner position sync, and (for an alarming mode that did not trigger) ALARM:4/5.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProbeResult {
  /// `true` if the expected probe edge occurred within the programmed travel, `false` otherwise.
  pub triggered: bool,
  /// The latched MACHINE position in whole steps per axis at the stop point (the trigger position on success, or
  /// the end-of-travel position on no-contact).
  pub stop_steps: [i32; AXES],
  /// `true` when the probe was ALREADY at its expected stop edge before the first step (no steps were emitted) —
  /// for a TOWARD probe this is the "probe already triggered" condition that grbl maps to ALARM:4, distinct from
  /// the "no contact within travel" ALARM:5. The consumer uses it to choose the alarm code.
  pub already_at_edge: bool,
}

/// Consumer → executor: a pending `G38.x` probe to run. Set by the consumer when the planner emits a
/// [`PlannerOutcome::Probe`](firmware_core::planner::PlannerOutcome::Probe); awaited by the core-1 executor,
/// which runs the probe-watching cycle. A `Signal` carrying the request is sufficient because a probe is a
/// synchronized boundary — only one is ever in flight (the consumer blocks on [`PROBE_RESULT`] until it finishes
/// before reading the next line).
pub static PROBE_REQUEST: Signal<CriticalSectionRawMutex, ProbeRequest> = Signal::new();

/// Executor → consumer: the result of the just-run probe cycle (the latched stop position + trigger flag). The
/// consumer awaits this after dispatching a [`PROBE_REQUEST`], so the probe is a blocking, in-order operation
/// from the consumer's view — exactly grbl's synchronized probe semantics.
pub static PROBE_RESULT: Signal<CriticalSectionRawMutex, ProbeResult> = Signal::new();

/// Consumer → executor: a request to run the `$H` homing cycle (DOC-06). Carries the resolved [`HomingConfig`]
/// (the consumer builds it from the live settings, which only core 0 reads) so the core-1 executor — which owns
/// the RMT step channels and the limit inputs — runs the whole Z-then-X+Y cycle without touching settings. Like
/// the probe request, only one homing cycle is ever in flight: the consumer blocks on [`HOME_RESULT`] until it
/// finishes before reading the next line, so a `Signal` carrying the config is sufficient.
pub static HOME_REQUEST: Signal<CriticalSectionRawMutex, HomingConfig> = Signal::new();

/// Executor → consumer: the result of the just-run homing cycle. `Ok(zero_steps)` carries the post-homing
/// MACHINE step position per axis the consumer syncs into the planner/parser; `Err` is the homing-fail (no
/// contact within 1.5× travel) or a sink/abort error, which the consumer maps to the homing-fail alarm + reset.
pub static HOME_RESULT: Signal<CriticalSectionRawMutex, Result<[i32; AXES], HomingError>> = Signal::new();

/// The last `G38.x` probe result (Phase C): the MACHINE position at the trigger instant and the contact flag,
/// read by the `$#` `[PRB:]` line (replacing the Phase-B zeros/flag-0 stub) and the immediate `[PRB:]` push. It
/// lives behind a synchronous `Cell` — the [`CONTROL`] / [`COORDINATES`] pattern — because the consumer writes it
/// on probe completion and `coordinate_report` reads it; a `Cell` swap is a few instructions and `LastProbe` is
/// `Copy`. Seeded to the never-probed value (`[PRB:0,0,0:0]`) at boot.
pub static LAST_PROBE: BlockingMutex<CriticalSectionRawMutex, Cell<LastProbe>> =
  BlockingMutex::new(Cell::new(LastProbe::none()));

/// Read the last-probe result (a synchronous `Cell` load under the blocking mutex). `Copy`, cheap.
pub(crate) fn last_probe() -> LastProbe {
  LAST_PROBE.lock(|c| c.get())
}

/// Store the last-probe result (a synchronous `Cell` store under the blocking mutex).
pub(crate) fn set_last_probe(result: LastProbe) {
  LAST_PROBE.lock(|c| c.set(result));
}

/// The live machine state the status formatter reads. The core-1 `motion_executor` publishes the live
/// (interpolated) MPos and the planner free-block count here as it executes blocks; Stage 1 holds the idle
/// default at boot so `?` returns a well-formed report immediately. Shared across cores via
/// [`CriticalSectionRawMutex`], which gates both cores on the S3.
pub static MACHINE: Mutex<CriticalSectionRawMutex, MachineSnapshot> = Mutex::new(MachineSnapshot::idle());

/// The authoritative, latched [`ControlState`] (DOC-08 Stage 2): the single source of truth for the machine's
/// control mode (Normal / Hold / Alarm / Check / Sleep), read by `status_responder` to compose the reported
/// [`MachineState`] and gated on by `comms_consumer` before planning. It lives behind a SYNCHRONOUS blocking
/// `Mutex<Cell<..>>` rather than the async `Mutex` because the real-time `dispatch_realtime` handler (in the
/// non-blocking reader half) must update it on `!`/`~`/`0x18` WITHOUT awaiting — a feed-hold has to show in the
/// very next `?`. The critical section is a few-instruction `Cell` swap, so holding it across both cores on the
/// S3 costs nothing; `ControlState` is `Copy`. Seeded at boot by [`init_control_state`] from `$22`.
pub static CONTROL: BlockingMutex<CriticalSectionRawMutex, Cell<ControlState>> =
  BlockingMutex::new(Cell::new(ControlState::Normal));

/// "A motion block is currently in flight on the core-1 executor" — set by the executor around `run_block`
/// (motion.rs) and read by `status_responder` to derive Run vs Idle while [`CONTROL`] is `Normal` (DOC-08).
/// An `AtomicBool` (not part of the planner queue depth) so the reporter sees Run even in the brief window
/// after the queue drains but a burst is still emitting. `AcqRel`/`Acquire` publishes it across cores.
pub static EXECUTOR_RUNNING: AtomicBool = AtomicBool::new(false);

/// Core-1 motion-executor LIVENESS counter (diagnostic): the executor bumps this once per drain-loop turn in
/// [`motion::run`](crate::motion::run), so a monotonically increasing value proves core 1 is still scheduling its
/// loop and a frozen value proves it has stopped advancing. The watchdog-feed task ([`watchdog_feed`]) samples it
/// each tick and, under `defmt`, logs whether it moved since the previous sample — so a lockup capture shows which
/// core died FIRST (core 1 frozen while core 0 still feeds the dog, vs. both frozen). It is a pure progress beat,
/// NOT a liveness gate on the watchdog: the dog is fed unconditionally so an instrumentation bug can never starve
/// it. `Relaxed` is correct — this is a cross-core diagnostic where exact ordering against other state does not
/// matter, only that the value advances; a single `AtomicU32` is a native lock-free store on the S3. Wraps at
/// `u32::MAX` (harmless: the sampler compares for INEQUALITY, not magnitude, so a wrap still reads as "advanced").
pub static MOTION_LIVENESS: AtomicU32 = AtomicU32::new(0);

/// Core-0 (PRO_CPU) COMMS-PROGRESS counter — the heart of the task-watchdog (revised after a real-board wedge where
/// the Embassy executor stayed alive but the comms PROCESSING/RESPONSE path was stuck on a never-resolving `.await`,
/// so the old unconditional feed kept the dog quiet). It counts GENUINE host-facing forward progress, NOT the feed
/// task running: it is bumped where the firmware actually serves the host — a `?` answered ([`status_responder`]),
/// a response/ack written out ([`usb_tx`]), and a line consumed by the parser pipeline ([`comms_consumer`]). The
/// watchdog withholds the feed (forcing a recoverable RWDT reset) when this counter STOPS advancing WHILE the host
/// is actively driving the board ([`RX_ACTIVITY`] live) — exactly the stuck-comms wedge the executor-alive feed
/// could not catch. It also feeds the breadcrumb's per-core "which froze first" compare, now untainted by a
/// feed-task self-bump (removed). `Relaxed` lock-free; only advancement (not magnitude) is read.
pub static COMMS_PROGRESS: AtomicU32 = AtomicU32::new(0);

/// Count of COMPLETED `usb_tx` writes (a clean write OR a recovered lost-wake — anything that delivered bytes to the
/// host). Distinct from [`COMMS_PROGRESS`] (which THREE tasks bump): this is `usb_tx`-SPECIFIC, so the dead-zone
/// watchdog backstop can tell whether the WRITER is making progress independent of the status reporter / consumer.
/// That distinction is load-bearing for Signature B: recovered-lost-wakes keep `COMMS_PROGRESS` advancing and fool
/// the comms-stall detector, but a board genuinely emitting nothing while responses are queued freezes THIS counter.
/// `Relaxed`: read only by the watchdog feed task; advancement, not magnitude, is what matters.
pub static USB_TX_COMPLETED: AtomicU32 = AtomicU32::new(0);

/// UNGATED core-0 executor-liveness beat (§17.15 root-cause fix). A dedicated core-0 async task
/// ([`watchdog_heartbeat`]) bumps this every interval unconditionally, so on a healthy board it advances REGARDLESS of
/// host traffic, motion, or queued responses — the one signal the three work-driven detectors lack. When the whole
/// core-0 thread-mode executor stalls (the Signature-B wedge), that task cannot run and this beat FREEZES while the
/// survivable hardware TIMG1 ISR keeps firing; the ISR watches for that freeze and withholds both dogs so a reset
/// fires. `Relaxed`: advancement, not magnitude, is what matters; the ISR is the sole reader. Starts at 0 and the ISR
/// only accrues a stall once it has advanced past 0 (the boot guard against the pre-first-bump zero). Bumped by
/// whichever core-0 feed task the build runs — [`watchdog_heartbeat`] (capture-reset) OR the production
/// [`watchdog_feed`] — so it advances iff the core-0 executor is scheduling tasks. Consumed by the capture-reset
/// survivable ISR AND (Design A, §20) the production detector-only stall ISR (`crate::stall_detector`).
pub static EXECUTOR_ALIVE: AtomicU32 = AtomicU32::new(0);

/// The latest `usb_tx` stall FINGERPRINT, published by [`usb_tx`] on each write timeout for the survivable-watchdog
/// TIMG1 ISR (the §17.10/§17.11 capture-at-withhold instrument, `capture-reset`-gated). The ISR cannot cheaply read
/// the USB_DEVICE registers from interrupt context, but `usb_tx` already computes the Signature-A discriminator at
/// each timeout — so it PUBLISHES the packed [`firmware_core::diag::UsbTxStall`] word here (and the response length +
/// windowed stall count in the siblings below). On a withhold the ISR copies these straight into the breadcrumb, so
/// even a hard Signature-B lock that never reached the K-escape still leaves the LAST-KNOWN usb_tx fingerprint. A
/// cold `0` (untagged) decodes as "no stall published this run" — the pure decoder rejects it.
#[cfg(feature = "capture-reset")]
pub static USB_TX_STALL_FINGERPRINT: AtomicU32 = AtomicU32::new(0);

/// The byte length of the response whose write last timed out, published alongside [`USB_TX_STALL_FINGERPRINT`] (the
/// §13.1 single-chunk-widening discriminator). `capture-reset`-gated; copied into the breadcrumb on an ISR withhold.
#[cfg(feature = "capture-reset")]
pub static USB_TX_STALL_FINGERPRINT_LEN: AtomicU32 = AtomicU32::new(0);

/// The latest WINDOWED `usb_tx`-stall count ([`firmware_core::diag::WindowedStallCounter::count`], §13.8), published
/// by [`usb_tx`] every write attempt. The ISR copies it into the breadcrumb on a withhold so the boot dump can tell a
/// PURE consecutive stall run from an ALTERNATING recovered/stall pattern. `capture-reset`-gated.
#[cfg(feature = "capture-reset")]
pub static USB_TX_STALL_WINDOW_COUNT: AtomicU32 = AtomicU32::new(0);

/// OBSERVE-ONLY air-run probes (the gcode-chunk-skip investigation, task #22). All four are pure diagnostic
/// counters with ZERO behavior change — surfaced on `$I` so a partial run reports them even without a wedge. The
/// cross-check is the diagnostic: on a clean run `lines == acks == execs`; a divergence localizes WHERE a line
/// vanishes. CRITICAL: the RX_PIPE overflow stays a SILENT DROP this build (just counted) — converting it to an
/// `error:N` would HOLD the stream and mask the very skip we are trying to observe; the overflow→hard-error fix is
/// a LATER build, gated on this air-run confirming + counting the drop.
///
/// `RX_PIPE_OVERFLOW`: dropped input bytes at [`usb_rx`]'s `RX_PIPE.try_write` (the prime non-reset skip suspect — a
/// dropped terminator merges two gcode lines → one silently lost). `> 0` on a skipping run = the overflow path is
/// real and firing. The single most important probe.
pub static RX_PIPE_OVERFLOW: AtomicU32 = AtomicU32::new(0);

/// OBSERVE-ONLY probe (task #22): count of input lines FRAMED by [`line_assembler`] and forwarded to [`LINE_QUEUE`]
/// (blank lines included — they are real protocol lines that earn a bare `ok`). The "line-IN" leg of the
/// lines/acks/execs cross-check. `LINES_FRAMED > ACKS_EMITTED` ⇒ a line was framed but never acked (consumed/dropped
/// before its terminal response).
pub static LINES_FRAMED: AtomicU32 = AtomicU32::new(0);

/// OBSERVE-ONLY probe (task #22): count of TERMINAL per-line responses (`ok`/`error:N`) emitted by the consumer — the
/// one-response-per-line flow-control acks, NOT status/banner/`[MSG:]` lines. The "ACK" leg of the cross-check.
/// `ACKS_EMITTED > BLOCKS_EXECUTED` ⇒ a line was acked but its motion block never ran (a motion-side drop — a skip
/// with NO host-visible gap, the hardest case).
pub static ACKS_EMITTED: AtomicU32 = AtomicU32::new(0);

/// OBSERVE-ONLY probe (task #22 §15, bughunter's over-ack null-check): count of lines actually CONSUMED by the
/// `comms_consumer` (pulled from `LINE_QUEUE` and run through `handle_line`). Compared against [`ACKS_EMITTED`]:
/// `ACKS_EMITTED > LINES_CONSUMED` ⇒ a firmware OVER-ACK (more terminal responses than lines consumed), which would
/// let a compliant host over-send by one line. Should stay equal (each consumed line emits exactly one terminal
/// response). Distinct from [`LINES_FRAMED`] (framed but maybe not yet consumed); the consume count is what the ack
/// count must match.
pub static LINES_CONSUMED: AtomicU32 = AtomicU32::new(0);

/// OBSERVE-ONLY probe (task #22): count of motion blocks actually EXECUTED (run to completion) by the core-1
/// executor — the "EXEC" leg of the cross-check. Bumped per block the executor finishes. Note this counts MOTION
/// blocks, not lines: a non-motion line (`$`-query, modal-only, M-code) acks without enqueuing a block, so on a real
/// program `BLOCKS_EXECUTED <= ACKS_EMITTED` is normal; the probe's value is in its DELTA over a run, cross-checked
/// against the per-line counts, not an exact equality.
pub static BLOCKS_EXECUTED: AtomicU32 = AtomicU32::new(0);

// NOTE (task #22 §15): the truncation counters (RUN_BLOCK_TRUNCATED total + the per-source twait/ttx/tlong split +
// the last-truncation axis) now live in FREE-RUNNING RTC_FAST (`crash.rs` `bump_run_block_truncated` /
// `read_run_block_truncated`), NOT `.bss` atomics — so they SURVIVE the K-escape `software_reset` that fires on a
// usb_tx wedge and a single end-of-run `$I` poll reads the cumulative total even through an intervening reset (the
// run-1 confound, where a plain atomic zeroed mid-run). `format_skip_probes` reads them from there.

/// Host RX-activity counter: bumped by [`usb_rx`] on every received byte, so the watchdog can tell whether a HOST is
/// actively driving the firmware (skirnir polls `?` ~5 Hz whenever connected, so bytes flow continuously while
/// connected). This is the CRITICAL guard against a reset-loop on a quiescent or disconnected board: the comms-stall
/// feed-withhold fires ONLY when RX is live (a host is present and expecting answers). With no recent RX — idle or
/// disconnected — the watchdog feeds normally and never resets, no matter how long the comms counter sits still
/// (there is simply no host to serve). `Relaxed` lock-free; the watchdog ages it across its ticks, reading only
/// whether it advanced, never the absolute value.
pub static RX_ACTIVITY: AtomicU32 = AtomicU32::new(0);

/// "A `$H` homing cycle is currently running" (DOC-06). Set by [`handle_home`] for the duration of the cycle and
/// cleared when it ends, so two things happen: the status reporter overrides the wire State to `Home` (grbl
/// reports `Home` during `$H` and queues live DRO motion — research finding #1), and the hard-limit monitor
/// SUPPRESSES the `$21` alarm path (the limit switches are EXPECTED to trip while homing — research finding #17,
/// the shared-pin rule). `AcqRel`/`Acquire` publishes it across cores. A coalesced flag is sufficient: only the
/// single in-order consumer sets/clears it around one cycle.
pub static HOMING_ACTIVE: AtomicBool = AtomicBool::new(false);

/// "Homing (`$22`) is enabled" — mirrored here from the live [`SETTINGS`] at boot (and on a `$22=` write) so
/// the control-state transitions that need it (boot-lock, soft-reset-to-boot, `$H` gating) can read it without
/// locking the async `SETTINGS` mutex from the synchronous real-time path. `Relaxed` is sufficient: it changes
/// only via `$22=` and is read for state decisions, never to guard other memory.
pub static HOMING_ENABLED: AtomicBool = AtomicBool::new(false);

/// "`$21` hard limits are enabled" — mirrored from the live [`SETTINGS`] at boot (and on a `$21=` write) so the
/// core-1 executor can read the hard-limit-enable bit without locking the async settings mutex from its
/// real-time loop. The executor samples the limit inputs at block boundaries / on the [`LIMIT_TRIGGERED`] wake
/// and raises [`HARD_LIMIT_TRIPPED`] only when this is set AND a homing cycle is not active (DOC-06). `Relaxed`
/// — it gates a real-time decision, not other memory.
pub static HARD_LIMITS_ENABLED: AtomicBool = AtomicBool::new(false);

/// "`$5` limit-pin invert" — mirrored from the live [`SETTINGS`] so the core-1 executor builds the
/// [`LimitConfig`](firmware_core::hal_traits::LimitConfig) for its homing-seek and hard-limit sampling without
/// locking the async settings mutex. Seeded at boot and on a `$5=` write / bulk import. `Relaxed` — it feeds a
/// real-time trigger-sense decision, not other memory.
pub static LIMIT_INVERT: AtomicBool = AtomicBool::new(false);

/// "`$26` limit-switch debounce, milliseconds" — mirrored from the live [`SETTINGS`] so the core-1 executor's
/// limit-edge monitor can run the debounce RESAMPLE (wait this long after a rising edge, then confirm the level
/// persists, rejecting EMI glitches / NC-pull-up settling — research finding #14) without locking the async
/// settings mutex. Seeded at boot and on a `$26=` write / bulk import. `Relaxed` — it feeds a real-time timing
/// decision, not other memory.
pub static LIMIT_DEBOUNCE_MS: AtomicU32 = AtomicU32::new(0);

/// Read the live `$26` debounce in milliseconds for the core-1 limit-edge resample. A synchronous atomic load,
/// callable from the real-time executor loop.
pub fn limit_debounce_ms() -> u32 {
  LIMIT_DEBOUNCE_MS.load(Ordering::Relaxed)
}

/// "The machine has a known (homed) machine position" (DOC-06). Set on a successful `$H`; cleared on every soft
/// reset / boot when homing is enabled (the machine returns to the unhomed boot-lock alarm). This is the gate
/// that makes `$20` soft limits MEANINGFUL: the envelope check is applied only when soft limits are enabled AND
/// the machine is homed (research finding #16), so an unhomed machine never rejects a move against an
/// unestablished zero. When homing is DISABLED entirely, the machine is treated as "homed" (position is taken at
/// face value, grbl's no-homing behavior) so soft limits — which require `$22` to even be enabled — still work
/// if a user force-enables them. `Relaxed` is sufficient: it gates a plan-time decision, not other memory.
pub static HOMED: AtomicBool = AtomicBool::new(false);

/// Latched at `0x18` dispatch time: "the executor was mid-cycle when this soft reset landed". Captured in the
/// non-blocking reader half (reading [`EXECUTOR_RUNNING`] before the executor can clear it) so the consumer's
/// `reset_pipeline` can faithfully apply grbl's rule — a reset that ABORTS an in-progress cycle raises
/// `ALARM:3`, while a reset from idle returns to the boot state. Without this latch the consumer would race the
/// executor's `EXECUTOR_RUNNING` clear and could miss the abort. `Relaxed` paired with the SOFT_RESET signal.
pub static RESET_WAS_RUNNING: AtomicBool = AtomicBool::new(false);

/// The live feed / rapid / spindle overrides (Phase E, DOC-08 §3): the single source of truth for the override
/// percentages and the spindle-stop / coolant toggles. Like [`CONTROL`] it lives behind a SYNCHRONOUS blocking
/// `Mutex<Cell<..>>` rather than an async `Mutex` because the real-time `dispatch_realtime` handler (in the
/// non-blocking reader half) mutates it on an override byte WITHOUT awaiting — a `0x91` feed-bump must show in
/// the very next `?` (`Ov:` and the realized `FS:`). The critical section is a few-instruction `Cell` swap, so
/// holding it across both cores on the S3 costs nothing; [`Overrides`] is `Copy`. The core-1 executor reads it
/// per block to scale step timing; `status_responder` reads it to render `Ov:` and the override-scaled `FS:`.
pub static OVERRIDES: BlockingMutex<CriticalSectionRawMutex, Cell<Overrides>> =
  BlockingMutex::new(Cell::new(Overrides::new()));

/// The PROGRAMMED nominal feed of the block currently executing, in mm/min, published by the core-1 executor as
/// it starts each block and zeroed when motion stops. `status_responder` reads it and applies the live feed/rapid
/// override to render the REALIZED `FS:` feed (Phase E). Held as an `AtomicU32` bit-cast of the `f32` so the
/// cross-core publish is lock-free (32-bit atomics are single-cycle on the S3). `0.0` means "no motion" → `FS:`
/// reports feed 0. The executor publishes the RAPID max-rate here for a G0 block and the feed nominal for a
/// feed/jog block, plus the [`LIVE_BLOCK_IS_RAPID`] flag so the reporter applies the right override.
pub static LIVE_PROGRAMMED_FEED_MM_MIN: AtomicU32 = AtomicU32::new(0);

/// "The block currently executing is a G0 RAPID" flag, published alongside [`LIVE_PROGRAMMED_FEED_MM_MIN`] so
/// `status_responder` applies the RAPID override (25/50/100 %) to a rapid and the FEED override to a feed/jog
/// move when rendering the realized `FS:`. `Relaxed` paired with the feed store; both are read together per report.
pub static LIVE_BLOCK_IS_RAPID: AtomicBool = AtomicBool::new(false);

/// The PROGRAMMED spindle speed in RPM, published by the consumer from the parser's modal `S` word so
/// `status_responder` can render the override-scaled realized RPM (Phase E). Updated whenever the consumer plans
/// a line (the parser's `S` is the commanded speed); `status_responder` applies the spindle override + stop toggle.
pub static PROGRAMMED_SPINDLE_RPM: AtomicU32 = AtomicU32::new(0);

/// The commanded modal spindle DIRECTION (DOC-07): `0` = M5/Stop, `1` = M3/CW, `2` = M4/CCW. Published by the
/// consumer from the planner's `Spindle` outcome and read by the [`spindle`] task, which combines it with the
/// override-scaled RPM (so a spindle-override / spindle-stop change re-drives the same direction at a new speed).
/// `Release`/`Acquire` orders it ahead of the [`SPINDLE_UPDATE`] wake that always follows a store.
pub static SPINDLE_DIRECTION: AtomicU8 = AtomicU8::new(SPINDLE_DIR_STOP);
/// [`SPINDLE_DIRECTION`] value for M5 / spindle stop.
pub const SPINDLE_DIR_STOP: u8 = 0;
/// [`SPINDLE_DIRECTION`] value for M3 / clockwise.
pub const SPINDLE_DIR_CW: u8 = 1;
/// [`SPINDLE_DIRECTION`] value for M4 / counter-clockwise.
pub const SPINDLE_DIR_CCW: u8 = 2;

/// Wakes the [`spindle`] task to RE-APPLY the spindle outputs from the current [`SPINDLE_DIRECTION`] + the
/// override-scaled RPM. Coalesced (a `Signal`): the task always re-reads the live direction/RPM after a wake, so
/// a missed-and-coalesced wake loses nothing. Set by the consumer on an M3/M4/M5 and by the real-time override
/// handler when the spindle override or spindle-stop toggle changes (so the realized RPM re-drives the duty).
pub static SPINDLE_UPDATE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Forces the [`spindle`] task to EMERGENCY-STOP (SPIN_EN off + duty 0) immediately, independent of motion state,
/// for any ALARM, soft reset (`0x18`), hard-limit trip, or `$SLP` sleep (DOC-07). A feed hold (`!`) deliberately
/// does NOT set this — a held program keeps the spindle running, matching grblHAL. The task races this against
/// the update wake AND against the reverse-dwell timer, so an e-stop during a spin-down still parks the spindle.
pub static SPINDLE_ESTOP: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// The commanded modal COOLANT state (M7/M8/M9), packed into a `u8` bitmask (`bit0 = mist`, `bit1 = flood`).
/// Published by the consumer from the parser's modal coolant state and read by the [`coolant`] task, which drives
/// the (hardware-gated) coolant outputs. `Release`/`Acquire` orders it ahead of the [`COOLANT_UPDATE`] wake that
/// always follows a store. Mirrors [`SPINDLE_DIRECTION`].
pub static COOLANT_STATE: AtomicU8 = AtomicU8::new(0);
/// [`COOLANT_STATE`] bit for mist (M7).
pub const COOLANT_BIT_MIST: u8 = 0b01;
/// [`COOLANT_STATE`] bit for flood (M8).
pub const COOLANT_BIT_FLOOD: u8 = 0b10;

/// Wakes the [`coolant`] task to RE-APPLY the coolant outputs from the current [`COOLANT_STATE`]. Coalesced (a
/// `Signal`): the task always re-reads the live state after a wake, so a missed-and-coalesced wake loses nothing.
/// Set by the consumer on an M7/M8/M9. Mirrors [`SPINDLE_UPDATE`].
pub static COOLANT_UPDATE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Forces the [`coolant`] task to turn BOTH circuits off immediately, independent of modal state, for any ALARM,
/// soft reset (`0x18`), hard-limit trip, `$SLP` sleep, or program end (M2/M30) — mirroring [`SPINDLE_ESTOP`] and
/// DOC-07's "ALARM/soft-reset force coolant off" rule. A feed hold (`!`) deliberately does NOT set this.
pub static COOLANT_ESTOP: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// The optional-stop (`M1`) switch state, toggled by the `0x88` real-time byte (grblHAL's optional-stop toggle).
/// `false` (the power-on / grblHAL default) makes an `M1` a no-op `ok`; `true` makes `M1` halt like `M0`. There is
/// no `$`-setting for this in grbl — the switch is a runtime toggle — so it is held here as a plain flag, NOT
/// persisted. A soft reset does NOT reset it (grbl keeps the operator's switch position across a reset). `Relaxed`
/// is sufficient: it gates a pause decision, not other memory.
pub static OPTIONAL_STOP_ENABLED: AtomicBool = AtomicBool::new(false);

/// Cycle-start (`~`) resume nudge for an active M0/M1/M6 program-flow pause. The `~` handler raises it whenever a
/// resume is requested AND a pause is active ([`PAUSE_ACTIVE`]); [`run_program_pause`] awaits it to leave the hold.
/// The pause's own state check ([`ControlState::resumes_on_cycle_start`]) is authoritative — this `Signal` only
/// nudges the awaiting pause out of its wait, so a coalesced wake loses nothing. A dedicated `Signal` per waiter
/// (only `run_program_pause` awaits it) keeps it from being stolen by the executor's `HOLD_WAKE`.
pub static PAUSE_RESUME: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// "An M0/M1/M6 program-flow pause is currently holding" — set by [`run_program_pause`] for the duration of the
/// hold, read by the `~` real-time handler so a cycle-start during a pause raises [`PAUSE_RESUME`] (and not just
/// the feed-hold release path). `AcqRel`/`Acquire` publishes it from the consumer to the reader half. Cleared when
/// the pause ends (resume / preemption).
pub static PAUSE_ACTIVE: AtomicBool = AtomicBool::new(false);

/// The last-sampled LOGICAL probe-asserted state (after the `$6` invert), published by the core-1 probe cycle and
/// read by `status_responder` to source the `Pn:P` letter (Phase E / Phase C). `AcqRel`/`Acquire` publishes it
/// across cores.
///
/// SCOPE NOTE (hardware boundary): the probe input ([`crate::motion::RmtProbeInput`]) is sampled CONTINUOUSLY
/// only during a `G38.x` cycle on core 1; there is no idle input-poll task yet, so `Pn:P` reflects the last
/// probe sample, not a live idle read. Continuous `Pn:` sampling of the probe / limit / control inputs is a
/// DOC-06 follow-up (an input-poll task behind the `DigitalIn` trait); until then this is the best available
/// source and is correct DURING a probe — which is when a host most wants `Pn:P`. It HOLDS the last sample after
/// a cycle ends (a successful toward probe is physically still in contact, so `Pn:P` correctly stays asserted
/// until the next probe re-samples), which is more faithful than force-zeroing it.
pub static PROBE_ASSERTED: AtomicBool = AtomicBool::new(false);

/// The live `$481` auto-report interval in milliseconds (Phase F), mirrored here from the persisted setting so
/// the auto-report task and the `0x8C` toggle read it without locking the async [`SETTINGS`] mutex. `0` means
/// auto-reporting is disabled by the setting. Seeded at boot by [`init_auto_report`] and updated on a `$481=`
/// write / `$PBX` import WITHOUT a reboot, so a host can turn periodic DRO on or off live. The value stored here
/// is always the CLAMPED interval ([`Settings::auto_report_interval_ms`]), so the task can never be asked for a
/// starving cadence. `Relaxed` is sufficient: it changes via settings writes and is read by the report timer,
/// never to guard other memory.
pub static AUTO_REPORT_INTERVAL_MS: AtomicU32 = AtomicU32::new(0);

/// "Auto-reporting is suspended by the runtime `0x8C` toggle" flag (Phase F). grblHAL's `0x8C` real-time byte
/// toggles the auto-report mode on the fly; this is the runtime override on top of the `$481` interval. The
/// auto-report task pushes a periodic `<...>` ONLY when the interval is non-zero AND this is `false`. A soft
/// reset clears it (auto-reporting resumes its `$481`-configured state) and re-seeds the interval. `Relaxed` —
/// it gates the report timer, not other memory.
pub static AUTO_REPORT_SUSPENDED: AtomicBool = AtomicBool::new(false);

/// Seed the live auto-report interval mirror at boot from the loaded `$481` (clamped to a safe cadence). Called
/// once from `main` before any task runs, so the auto-report task starts at the persisted interval.
pub fn init_auto_report(interval_ms: u32) {
  AUTO_REPORT_INTERVAL_MS.store(interval_ms, Ordering::Relaxed);
  AUTO_REPORT_SUSPENDED.store(false, Ordering::Relaxed);
}

/// Set the [`HOMED`] flag to its boot/reset baseline given whether `$22` homing is enabled (finding #8). The
/// invariant: the machine is UNHOMED unless homing is DISABLED — with homing disabled, position is taken at face
/// value (grbl's no-homing behavior) so the machine is treated as homed; with homing enabled, a fresh boot or a
/// soft reset returns to the unhomed `ALARM:11` lock until `$H`. Pulled out so the boot ([`init_control_state`])
/// and reset ([`apply_soft_reset`]) sites cannot drift apart and the bare `!homing_enabled` encoding is documented
/// in ONE place. A successful `$H` overrides this with `HOMED = true`.
pub(crate) fn store_homed_baseline(homing_enabled: bool) {
  HOMED.store(!homing_enabled, Ordering::Relaxed);
}

/// Read the live overrides (a synchronous `Cell` load under the blocking mutex). `Copy`, cheap, callable from
/// any context including the real-time reader half and the core-1 executor.
pub fn overrides() -> Overrides {
  OVERRIDES.lock(|c| c.get())
}

/// Store updated overrides (a synchronous `Cell` store under the blocking mutex).
pub(crate) fn set_overrides(ov: Overrides) {
  OVERRIDES.lock(|c| c.set(ov));
}

/// Seed the [`CONTROL`] state and the [`HOMING_ENABLED`] mirror at boot from the loaded `$22`. Called once from
/// `main` before any task runs. When homing is enabled the machine boots LOCKED in `ALARM:11` (homing
/// required) per grbl; otherwise it boots `Normal` (Idle). The boot `ALARM:N` push is emitted by `main` after
/// the banner so a host sees the locked state on connect.
pub fn init_control_state(homing_enabled: bool) {
  HOMING_ENABLED.store(homing_enabled, Ordering::Relaxed);
  CONTROL.lock(|c| c.set(ControlState::boot(homing_enabled)));
  // Seed the homed baseline (finding #8): homed at boot only when homing is DISABLED; with homing enabled it
  // boots unhomed in the `ALARM:11` lock until `$H` (DOC-06).
  store_homed_baseline(homing_enabled);
}

/// Seed the [`HARD_LIMITS_ENABLED`], [`LIMIT_INVERT`], and [`LIMIT_DEBOUNCE_MS`] mirrors at boot from the loaded
/// `$21` bit0 / `$5` / `$26` (DOC-06). Called once from `main` before any task runs, so the core-1 executor's
/// hard-limit / homing-seek sampling and the limit-edge debounce read the persisted state without an async lock.
pub fn init_limit_settings(hard_limits_enabled: bool, limit_invert: bool, debounce_ms: u32) {
  HARD_LIMITS_ENABLED.store(hard_limits_enabled, Ordering::Relaxed);
  LIMIT_INVERT.store(limit_invert, Ordering::Relaxed);
  LIMIT_DEBOUNCE_MS.store(debounce_ms, Ordering::Relaxed);
}

/// Build the live [`LimitConfig`](firmware_core::hal_traits::LimitConfig) from the [`LIMIT_INVERT`] (`$5`) mirror,
/// for the core-1 executor's homing-seek / hard-limit sampling. Reads the atomic, not the async settings mutex,
/// so it is callable from the real-time loop.
pub fn limit_config() -> firmware_core::hal_traits::LimitConfig {
  firmware_core::hal_traits::LimitConfig { invert: LIMIT_INVERT.load(Ordering::Relaxed) }
}

/// Read the current [`ControlState`] (a synchronous `Cell` load under the blocking mutex). Cheap and callable
/// from any context, including the real-time reader half.
pub(crate) fn control_state() -> ControlState {
  CONTROL.lock(|c| c.get())
}

/// Store a new [`ControlState`] (a synchronous `Cell` store under the blocking mutex).
pub(crate) fn set_control_state(state: ControlState) {
  CONTROL.lock(|c| c.set(state));
}

/// The shared motion planner: the core-0 `comms_consumer` task enqueues blocks into it, and the core-1
/// `motion_executor` ([`crate::motion`]) pops them. It lives behind a `Mutex` because two tasks on two cores
/// touch it; [`CriticalSectionRawMutex`] makes the lock cross-core-safe on the S3 (the critical section
/// gates both cores). Both critical sections are short (enqueue one command / pop one block + peek the next),
/// and the executor always RELEASES the lock before any RMT transmit, so the planner mutex is never held
/// across step emission and contention stays negligible.
///
/// Initialized lazily to `None` because [`Planner::new`] is not `const`; [`init_planner`] installs the
/// constructed planner once at boot before either task runs. After init the `Option` is always `Some`.
pub static PLANNER: Mutex<CriticalSectionRawMutex, Option<Planner>> = Mutex::new(None);

/// The live machine settings (DOC-04): the persisted `$`-settings plus TMC parameters, loaded from flash at
/// boot. Shared so the consumer mutates them on `$x=val`, the status reporter reads steps/mm for the live
/// MPos conversion, and the soft-reset path rebuilds the planner from them. Held as an `Option` because
/// [`Settings::default`] is not `const`; [`init_settings`] seeds it once at boot (after which it is always
/// `Some`). Shared across cores via [`CriticalSectionRawMutex`].
pub static SETTINGS: Mutex<CriticalSectionRawMutex, Option<Settings>> = Mutex::new(None);

/// "The live [`SETTINGS`] differ from what is persisted in flash" flag, the heart of the write-coalescing
/// (Finding #14b). Every `$n=val` / `$PBX` change applies to the in-RAM [`SETTINGS`] and sets this; the actual
/// flash write happens ONCE per burst, not per line. Without coalescing a `$$`-bulk restore (~36 `$n=val`
/// lines back to back) would append the WHOLE settings blob to the wear-leveled flash log ~36 times, thrashing
/// the NVS region. The single [`comms_consumer`] task is the sole owner of the flush: it persists the live
/// settings and clears this flag when its input has drained (the line queue is empty — a burst boundary), on a
/// safety interval, and on soft reset, so a change is coalesced with its burst yet never lost. `AcqRel`/
/// `Acquire` ordering publishes the flag against the `SETTINGS` mutex release on the same core-0 executor.
pub static SETTINGS_DIRTY: AtomicBool = AtomicBool::new(false);

/// Mark the live [`SETTINGS`] as changed-but-not-yet-persisted. Called after a `$n=val` / `$PBX` write has been
/// applied to the in-RAM [`SETTINGS`]; the consumer's coalesced flush picks it up at the next burst boundary.
pub(crate) fn mark_settings_dirty() {
  SETTINGS_DIRTY.store(true, Ordering::Release);
}

/// The authoritative coordinate model (Phase B): G54-G59 work offsets, the active WCS, G92, the dynamic TLO,
/// and the G28/G30 predefined positions. It lives behind a SYNCHRONOUS blocking `Mutex<Cell<..>>` — exactly the
/// [`CONTROL`] pattern — because `status_responder` reads it on every `?` to render `WPos:`/`WCO:` and the
/// consumer mutates it on coordinate ops; a `Cell` swap is a few instructions, so holding it across both cores
/// on the S3 costs nothing and `CoordinateSystems` is `Copy`. Seeded at boot from the persisted record by
/// [`init_coordinates`]; the persistent subset is written back to NVS by the coalesced coordinate flush.
pub static COORDINATES: BlockingMutex<CriticalSectionRawMutex, Cell<CoordinateSystems>> =
  BlockingMutex::new(Cell::new(CoordinateSystems::new()));

/// "The PERSISTENT coordinate subset (G54-G59 / G28 / G30) differs from what is persisted in flash" flag — the
/// coordinate analogue of [`SETTINGS_DIRTY`]. A G10 / G28.1 / G30.1 / WCS-select change marks this; the consumer
/// flushes the coordinate record ONCE per burst (queue-empty), on the safety interval, and on soft reset, so a
/// program that re-zeroes several axes does not append the blob per line. G92 / G43.1 (TLO) changes do NOT mark
/// it — they are session-only and never persisted. `AcqRel`/`Acquire` publishes it against the `COORDINATES`
/// cell on the same core-0 executor.
pub static COORDINATES_DIRTY: AtomicBool = AtomicBool::new(false);

/// The `WCO:` refresh-cadence state machine (DOC-08 §4), owned by `status_responder`. It lives behind a
/// synchronous `Cell` so the status task can advance it per report without an async lock; the soft-reset path
/// resets it so the first report after a reset re-emits `WCO:` (grbl's rule). Read/written only from core 0.
pub(crate) static WCO_REPORTER: BlockingMutex<CriticalSectionRawMutex, Cell<RefreshReporter<[f32; AXES]>>> =
  BlockingMutex::new(Cell::new(RefreshReporter::new([0.0; AXES])));

/// The `Ov:` refresh-cadence state machine (DOC-08 §4, Phase E), owned by `status_responder` and mirroring
/// [`WCO_REPORTER`]. It lives behind a synchronous `Cell` so the status task advances it per report without an
/// async lock; the soft-reset path resets it so the first report after a reset re-emits `Ov:` (grbl's rule).
/// Read/written only from core 0.
pub(crate) static OV_REPORTER: BlockingMutex<CriticalSectionRawMutex, Cell<RefreshReporter<Overrides>>> =
  BlockingMutex::new(Cell::new(RefreshReporter::new(Overrides::new())));

/// Exactly the derived settings values [`status_responder`] needs on the hot `?`/auto-report path, cached so the
/// steady-state report reads them from a synchronous `Cell` WITHOUT locking the cross-core async [`SETTINGS`]
/// mutex or bit-copying the whole ~30-field [`Settings`] every report (Finding #14). It is `Copy` (three small
/// scalars/arrays) so a `Cell` swap is a few instructions. Refreshed from the full [`Settings`] at EVERY settings
/// commit so a `$100=`/`$10=`/`$110=` change is reflected in the very next report — see [`refresh_status_cfg`]
/// for the exhaustive list of refresh sites.
#[derive(Debug, Clone, Copy)]
pub(crate) struct StatusCfg {
  /// `$100-102` steps/mm, for the live steps→mm MPos conversion.
  pub(crate) steps_per_mm: [f32; AXES],
  /// Whether the report carries `MPos:` or `WPos:`, derived from the `$10` status-report-mask bit 0.
  pub(crate) position_report: PositionReport,
  /// The most-restrictive per-axis max-rate (mm/min), the conservative ceiling for the override-scaled `FS:`
  /// feed — derived once from `$110-112` here so the hot path does not re-fold it per report.
  pub(crate) min_axis_max_rate: f32,
}

impl StatusCfg {
  /// Derive the cached status configuration from the full settings.
  fn from_settings(settings: &Settings) -> Self {
    StatusCfg {
      steps_per_mm: settings.steps_per_mm(),
      position_report: PositionReport::from_status_mask(settings.status_report_mask),
      min_axis_max_rate: min_axis_max_rate(&settings.max_rate_mm_min),
    }
  }
}

/// The cached [`StatusCfg`] the steady-state `?`/auto-report path reads with NO [`SETTINGS`] lock and no struct
/// copy. A synchronous `Cell` behind a blocking mutex (the [`CONTROL`]/[`COORDINATES`] pattern); seeded at boot
/// with the compiled defaults and overwritten by [`refresh_status_cfg`] at every settings-commit site, so it is
/// always at least as fresh as the live [`SETTINGS`] from the reporter's point of view.
pub(crate) static STATUS_CFG: BlockingMutex<CriticalSectionRawMutex, Cell<StatusCfg>> = BlockingMutex::new(Cell::new(
  StatusCfg { steps_per_mm: [0.0; AXES], position_report: PositionReport::Machine, min_axis_max_rate: f32::INFINITY },
));

/// Recompute the cached [`STATUS_CFG`] from the live [`SETTINGS`] so the next status report reflects a settings
/// change without locking the async mutex on the hot path. This MUST be called at EVERY place [`SETTINGS`] is
/// mutated, or a `$100`/`$10`/`$110` change would not appear in reports (a missed invalidation is a regression).
/// The refresh sites are:
/// - the boot seed ([`init_status_cfg`], called from `main` after `init_settings`),
/// - the `$x=val` write path ([`write_setting_command`]),
/// - the `$PBX` / bulk-import completion ([`handle_pb_write`]),
/// - the `$RST=$` / `$RST=*` restore-defaults path ([`handle_restore_settings`]).
///
/// There is no other writer of [`SETTINGS`]; every one of the above calls this after applying the change.
pub(crate) async fn refresh_status_cfg() {
  let settings = settings_snapshot().await;
  STATUS_CFG.lock(|c| c.set(StatusCfg::from_settings(&settings)));
}

/// Seed the cached [`STATUS_CFG`] at boot from the loaded settings. Called once from `main` after
/// [`init_settings`], before any task runs, so the first status report reads correct steps/mm and `$10` mode.
pub fn init_status_cfg(settings: &Settings) {
  STATUS_CFG.lock(|c| c.set(StatusCfg::from_settings(settings)));
}

/// Read a copy of the live coordinate model (a synchronous `Cell` load under the blocking mutex). `Copy`, cheap,
/// callable from any context.
pub(crate) fn coordinates() -> CoordinateSystems {
  COORDINATES.lock(|c| c.get())
}

/// Store an updated coordinate model (a synchronous `Cell` store under the blocking mutex).
pub(crate) fn set_coordinates(coords: CoordinateSystems) {
  COORDINATES.lock(|c| c.set(coords));
}

/// Mark the PERSISTENT coordinate subset as changed-but-not-yet-persisted, for the coalesced coordinate flush.
pub(crate) fn mark_coordinates_dirty() {
  COORDINATES_DIRTY.store(true, Ordering::Release);
}

/// Seed [`COORDINATES`] at boot from the persisted record: load the persistent G54-G59 / G28 / G30 (and active
/// WCS) into a fresh model, leaving the session-only G92 / TLO at identity. Called once from `main` before any
/// task runs. The planner's work offset is seeded separately from the resulting WCO (see [`init_planner`] +
/// [`push_wco_to_planner`]).
pub fn init_coordinates(persistent: CoordinatePersistent) {
  let mut coords = CoordinateSystems::new();
  coords.load_persistent(&persistent);
  set_coordinates(coords);
}

/// Push the active Work Coordinate Offset (WCO) from the live coordinate model into the shared planner, so the
/// planner's absolute work→machine transform uses the current offset. Called after any coordinate change and
/// once at boot. Takes the planner lock briefly; the coordinate read is a synchronous `Cell` load.
pub(crate) async fn push_wco_to_planner() {
  let wco = coordinates().wco();
  let mut guard = PLANNER.lock().await;
  if let Some(planner) = guard.as_mut() {
    planner.set_work_offset(wco);
  }
}

/// Seed the planner's work offset from the boot-loaded coordinate model. Called once from `main` after
/// [`init_planner`] and [`init_coordinates`] so the first absolute work move resolves against the persisted
/// WCS offset; a public wrapper over [`push_wco_to_planner`] for the init path.
pub async fn seed_planner_work_offset() {
  push_wco_to_planner().await;
}

/// Seed [`SETTINGS`] at boot with the settings loaded from flash (or defaults). Called once from `main`
/// before any task runs; `try_lock` cannot contend yet.
pub fn init_settings(settings: Settings) {
  if let Ok(mut guard) = SETTINGS.try_lock() {
    *guard = Some(settings);
  }
}

/// A copy of the live settings (or [`Settings::default`] if somehow unseeded). Copies out under a brief lock
/// so a caller reads fields without holding the mutex across `.await`s. `Settings` is `Copy`, so this is cheap.
pub(crate) async fn settings_snapshot() -> Settings {
  let guard = SETTINGS.lock().await;
  (*guard).unwrap_or_default()
}

/// Install the planner into [`PLANNER`] at boot from `config` (derived from the loaded settings). Called once
/// from `main`; `try_lock` avoids an await in init and cannot contend (no task runs yet). A `try_lock` failure
/// would be a wiring bug, handled by leaving the planner uninitialized (the consumer then fails lines loudly
/// rather than panicking), but in practice it never fails.
pub fn init_planner(config: PlannerConfig) {
  if let Ok(mut guard) = PLANNER.try_lock() {
    *guard = Some(Planner::new(config));
  }
}
