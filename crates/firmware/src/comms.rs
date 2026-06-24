//! USB CDC streaming bring-up (DOC-08 / DOC-01): the esp-hal wiring that drives the sans-io
//! [`firmware_core::protocol`] state machine over the ESP32-S3 USB Serial/JTAG controller.
//!
//! This module is the thin, non-host-testable adapter the task brief calls for: ALL streaming logic
//! (line framing, real-time classification, error-hold, response formatting) lives in firmware-core
//! and is exercised by host tests; here we only move bytes between the USB peripheral and that state
//! machine, dispatch real-time commands through `embassy-sync` Signals, and emit responses.
//!
//! ## RX split: real-time dispatch never blocks behind line back-pressure (DOC-08, grbl ISR model)
//! The receive path is split into two tasks around a real byte buffer ([`RX_PIPE`], sized to the advertised
//! [`RX_BUFFER_SIZE`]), mirroring grbl's ISR-ring-buffer architecture:
//! - [`usb_rx`] (the *reader half*) reads USB bytes and, per byte, runs [`classify_realtime`]: a real-time
//!   command (`?`/`!`/`~`/`0x18`/…) is dispatched through its Signal *immediately* (non-blocking); any other
//!   byte is pushed into [`RX_PIPE`]. The reader never blocks on line flow control, so a feed-hold or soft
//!   reset arriving during sustained streaming is acted on at once, never stalled behind a full line queue.
//! - [`line_assembler`] (the *line-assembly half*) drains [`RX_PIPE`] through a [`StreamEngine`] line framer
//!   and forwards completed lines (blank lines included) to [`LINE_QUEUE`]. Blocking here on planner
//!   back-pressure is correct: it stops draining the byte buffer, which fills, and the host's
//!   character-counting throttles. Because the host counts outstanding *non-real-time* bytes against
//!   [`RX_BUFFER_SIZE`] and [`RX_PIPE`] is sized to exactly that, the pipe can always absorb every byte a
//!   compliant host is permitted to have in flight — so the reader half's pipe write never blocks.
//! - [`usb_tx`] is the single writer to the USB peripheral: it drains the [`RESPONSE`] channel so no two
//!   tasks ever write the USB endpoint concurrently (DOC-08).
//! - [`comms_consumer`] is the real gcode parser → planner pipeline. It parses each accepted line through a
//!   persistent [`Parser`], feeds the resulting command to a persistent [`Planner`], answers the `$` system
//!   queries (`$$`/`$I`/`$I+`/`$G`/`$#`), and emits exactly one `ok`/`error:N` per line. It owns the
//!   grblHAL gcode error-hold and back-pressures the host when the planner buffer is full. While this single task
//!   is held in an M0/M1/M6 pause ([`run_program_pause`]), it STILL services read-only `$`-queries (`$G`/`$#`/`$$`/
//!   `$I`…) in place via [`hold_until_resume`] — answering report + `ok` without releasing the hold, so a
//!   character-counting host is never stalled — while DEFERRING every motion / write / action line until resume.
//!   `?` is served by the separate [`status_responder`] task and is answered throughout regardless.
//! - The DOC-02 `motion_executor` (in [`crate::motion`], on core 1) is the consumer end of the planner
//!   queue: it pops blocks, realizes them as RMT step pulses, and publishes the *live* position into
//!   [`MACHINE`]. The consumer raises [`BLOCK_AVAILABLE`] after enqueuing a motion block so the executor
//!   wakes without polling. This replaced the Stage-1 `block_drain_stub`, which only paced time.
//! - [`status_responder`] formats a `<...>` report from the shared [`MachineSnapshot`] when the
//!   [`STATUS_REQUEST`] Signal fires.
//!
//! Real-time bytes are intercepted in [`usb_rx`] before they ever reach the byte buffer and never receive
//! an `ok`, exactly matching the firmware-core contract.

use core::cell::Cell;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, AtomicU8, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::pipe::Pipe;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embassy_futures::select::{select, select4, Either, Either4};
use embedded_io_async::{Read, Write};
use esp_hal::usb_serial_jtag::{UsbSerialJtagRx, UsbSerialJtagTx};
use esp_hal::Async;

use firmware_core::coords::{self, CoordinatePersistent, CoordinateSystems};
use firmware_core::homing::{HomingConfig, HomingError};
use firmware_core::gcode::{
  CoordinateOp, DistanceMode as GcodeDistance, FeedMode as GcodeFeedMode, ModalState, MotionMode, Parser,
  SpindleState, Units as GcodeUnits,
};
use firmware_core::motion::steps_to_mm;
use firmware_core::planner::{Planner, PlannerConfig, PlannerError, PlannerOutcome, SoftLimits, SpinUpGate, AXES};
use firmware_core::spindle::SpindleAction;
use firmware_core::protocol::{
  classify_realtime, probe_response, AlarmCode, CheckToggle, ControlState, CoordinateReport, EngineEvent,
  LastProbe, MachineSnapshot, MachineState, Overrides, ParserCoolant, ParserDistance, ParserFeedMode, ParserMotion,
  ParserPlane, ParserSnapshot, ParserSpindle, ParserUnits,
  PinReport, PositionReport, ProbeResponse, RealtimeCommand, RefreshReporter, ResponseWriter, StreamEngine,
  SystemCommand, UnlockOutcome, ERROR_CODES, ERROR_HOMING_DISABLED, ERROR_UNSUPPORTED_COMMAND, MAX_LINE_LEN,
  NGC_PARAMETER_LINES, RX_BUFFER_SIZE, RESPONSE_CAPACITY,
};
use firmware_core::settings::{self, PbChunkResult, PbReceiver, SettingError, Settings};

use crate::spindle;
use crate::storage::{FlashRecordStore, SharedFlash};

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
fn last_probe() -> LastProbe {
  LAST_PROBE.lock(|c| c.get())
}

/// Store the last-probe result (a synchronous `Cell` store under the blocking mutex).
fn set_last_probe(result: LastProbe) {
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
fn store_homed_baseline(homing_enabled: bool) {
  HOMED.store(!homing_enabled, Ordering::Relaxed);
}

/// Read the live overrides (a synchronous `Cell` load under the blocking mutex). `Copy`, cheap, callable from
/// any context including the real-time reader half and the core-1 executor.
pub fn overrides() -> Overrides {
  OVERRIDES.lock(|c| c.get())
}

/// Store updated overrides (a synchronous `Cell` store under the blocking mutex).
fn set_overrides(ov: Overrides) {
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
fn control_state() -> ControlState {
  CONTROL.lock(|c| c.get())
}

/// Store a new [`ControlState`] (a synchronous `Cell` store under the blocking mutex).
fn set_control_state(state: ControlState) {
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
fn mark_settings_dirty() {
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
static WCO_REPORTER: BlockingMutex<CriticalSectionRawMutex, Cell<RefreshReporter<[f32; AXES]>>> =
  BlockingMutex::new(Cell::new(RefreshReporter::new([0.0; AXES])));

/// The `Ov:` refresh-cadence state machine (DOC-08 §4, Phase E), owned by `status_responder` and mirroring
/// [`WCO_REPORTER`]. It lives behind a synchronous `Cell` so the status task advances it per report without an
/// async lock; the soft-reset path resets it so the first report after a reset re-emits `Ov:` (grbl's rule).
/// Read/written only from core 0.
static OV_REPORTER: BlockingMutex<CriticalSectionRawMutex, Cell<RefreshReporter<Overrides>>> =
  BlockingMutex::new(Cell::new(RefreshReporter::new(Overrides::new())));

/// Exactly the derived settings values [`status_responder`] needs on the hot `?`/auto-report path, cached so the
/// steady-state report reads them from a synchronous `Cell` WITHOUT locking the cross-core async [`SETTINGS`]
/// mutex or bit-copying the whole ~30-field [`Settings`] every report (Finding #14). It is `Copy` (three small
/// scalars/arrays) so a `Cell` swap is a few instructions. Refreshed from the full [`Settings`] at EVERY settings
/// commit so a `$100=`/`$10=`/`$110=` change is reflected in the very next report — see [`refresh_status_cfg`]
/// for the exhaustive list of refresh sites.
#[derive(Debug, Clone, Copy)]
struct StatusCfg {
  /// `$100-102` steps/mm, for the live steps→mm MPos conversion.
  steps_per_mm: [f32; AXES],
  /// Whether the report carries `MPos:` or `WPos:`, derived from the `$10` status-report-mask bit 0.
  position_report: PositionReport,
  /// The most-restrictive per-axis max-rate (mm/min), the conservative ceiling for the override-scaled `FS:`
  /// feed — derived once from `$110-112` here so the hot path does not re-fold it per report.
  min_axis_max_rate: f32,
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
static STATUS_CFG: BlockingMutex<CriticalSectionRawMutex, Cell<StatusCfg>> = BlockingMutex::new(Cell::new(
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
async fn refresh_status_cfg() {
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
fn coordinates() -> CoordinateSystems {
  COORDINATES.lock(|c| c.get())
}

/// Store an updated coordinate model (a synchronous `Cell` store under the blocking mutex).
fn set_coordinates(coords: CoordinateSystems) {
  COORDINATES.lock(|c| c.set(coords));
}

/// Mark the PERSISTENT coordinate subset as changed-but-not-yet-persisted, for the coalesced coordinate flush.
fn mark_coordinates_dirty() {
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
async fn push_wco_to_planner() {
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
async fn settings_snapshot() -> Settings {
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

/// Enqueue a response for the USB writer, blocking until it is accepted so a `ok`/`error:N`/report is
/// NEVER dropped — the one-`ok`-per-line contract that drives host flow control depends on guaranteed
/// delivery (a lost `ok` permanently stalls a character-counting host). Blocking here is safe because every
/// caller of this function runs OFF the real-time path: real-time command dispatch lives entirely in the
/// reader half ([`usb_rx`]), which never enqueues responses, so a momentarily full [`RESPONSE`] channel can
/// only back-pressure the response producers (consumer / status reporter), never delay a real-time byte.
async fn enqueue(resp: Response) {
  RESPONSE.send(resp).await;
}

/// Render a banner into a fresh [`Response`] and queue it, blocking until accepted. Emitted on boot so a
/// host detects controller readiness (DOC-08 / the native-USB no-hard-reset rule). The soft-reset banner is
/// emitted by the consumer's pipeline reset, not here, so this never runs on the reader half.
pub async fn send_banner() {
  let mut s = Response::new();
  if ResponseWriter::banner(&mut s).is_ok() {
    enqueue(s).await;
  }
}

/// The formatted post-mortem crash report (`[MSG:CRASH ...]`), held after boot so it can be RE-EMITTED on the
/// first `$I`/status request after a host connects. The native-USB link re-enumerates on the watchdog reset, so a
/// host that reconnects a beat late would miss the boot-time emission; stashing the line here and replaying it on
/// the first `$I`/`?` closes that race. `None` once there is nothing to report (a clean boot, or after a single
/// replay — see [`take_pending_crash_report`]). A `Cell<Option<Response>>` behind the cross-core blocking mutex
/// keeps it lock-free-ish and `Send`; `Response` is `heapless::String`, no allocation.
static CRASH_REPORT: BlockingMutex<CriticalSectionRawMutex, Cell<Option<Response>>> = BlockingMutex::new(Cell::new(None));

/// Format the previous run's crash breadcrumb into a grbl `[MSG:CRASH ...]` line, emit it ONCE over the normal TX
/// path right after the boot banner, AND stash it for one replay on the first `$I`/status after connect. Called
/// from `main` after [`send_banner`], with the breadcrumb read from RTC_FAST and whether the reset was a
/// watchdog/fault reset (a clean power-on / brown-out clears RTC_FAST anyway, so a valid breadcrumb after one of
/// those would be impossible — but we still gate on the reset reason for clarity and defence in depth).
///
/// The breadcrumb survives a WATCHDOG reset, NOT a power-cycle (see [`crate::crash`]): the operator must let the
/// dog bite (~8 s) and must not yank power, or the breadcrumb is lost. A no-op when the breadcrumb is invalid
/// (clean boot, or the crumb was already consumed) — nothing is emitted or stashed.
pub async fn maybe_emit_crash_report(breadcrumb: &crate::crash::Breadcrumb, reset_was_watchdog: bool) {
  if !breadcrumb.is_valid() || !reset_was_watchdog {
    return;
  }
  let Some(report) = format_crash_report(breadcrumb) else {
    return;
  };
  // Stash a copy for the first-`$I`/`?` replay (reconnect race), then emit now over the guaranteed-delivery path.
  CRASH_REPORT.lock(|c| c.set(Some(report.clone())));
  enqueue(report).await;
}

/// Take the stashed crash report for a one-shot replay (consumes it so it is sent at most once more after the boot
/// emission). Returns `None` once nothing is pending. Called from the `$I` build-info handler and the status
/// responder so a host that reconnected late after the watchdog reset still receives the `[MSG:CRASH ...]` line.
fn take_pending_crash_report() -> Option<Response> {
  CRASH_REPORT.lock(|c| c.take())
}

/// Build the `[MSG:CRASH ...]` line from a decoded breadcrumb. Renders, in order: the WATCHDOG WITHHOLD CLASS when
/// present (`core1-motion-wedge` / `core0-comms-wedge` — the most decisive datum, naming WHY the dog was forced to
/// fire), the last executor stage (with axis for the per-axis RMT stages, e.g. `axis1:wait_begin`), the snapshot
/// "which side froze first" verdict, and the newest comms/motion beats. Returns `None` only if the text could not be
/// wrapped (never in practice — the line is far under [`RESPONSE_CAPACITY`]). Pure formatting; no I/O.
fn format_crash_report(breadcrumb: &crate::crash::Breadcrumb) -> Option<Response> {
  use core::fmt::Write as _;
  // Build the inner text (without the `[MSG:...]` envelope), then wrap it. Sized well under RESPONSE_CAPACITY.
  let mut inner: heapless::String<128> = heapless::String::new();
  let _ = write!(inner, "CRASH");
  // Withhold class first when the watchdog deliberately forced the reset — the single most useful datum (an executor
  // death that just stopped feeding leaves no withhold marker, so this is absent then).
  if let Some(reason) = crate::crash::withhold_label(breadcrumb.withhold) {
    let _ = write!(inner, " {}", reason);
  }
  // Stage: name plus axis for the per-axis RMT stages. `write!` into a fixed string cannot panic; ignore the
  // `Result` (a full buffer just truncates, which still yields a usable, if clipped, report).
  let stage = breadcrumb.last_stage;
  if crate::crash::stage_has_axis(stage) {
    let _ = write!(inner, " stage=axis{}:{}", crate::crash::stage_axis(stage), crate::crash::stage_label(stage));
  } else {
    let _ = write!(inner, " stage={}", crate::crash::stage_label(stage));
  }
  // Which side stopped advancing first, from the snapshot ring (comms = core-0 side, motion = core-1 side).
  let verdict = crate::crash::froze_first(&breadcrumb.snapshots);
  let _ = write!(inner, " {}", verdict);
  // Newest beats (index 0 of the newest-first ring): `comms` is the core-0 host-facing progress counter, `motion`
  // the core-1 executor beat — so the operator sees the absolute counters too.
  let newest = breadcrumb.snapshots[0];
  let _ = write!(inner, " beats comms={} motion={}", newest.core0_beat, newest.core1_beat);
  // Remind that the breadcrumb is watchdog-survival only (so a power-cycle would have lost it — useful context).
  let _ = write!(inner, " (RWDT-reset; not power-cycle)");
  let mut out = Response::new();
  ResponseWriter::message(&mut out, inner.as_str()).ok()?;
  Some(out)
}

/// Emit the boot `ALARM:N` (plus its `[MSG:..unlock]` prompt) IF the machine booted into an alarm — i.e.
/// `$22` homing is enabled, so [`init_control_state`] latched `ALARM:11` (homing required). Called once from
/// `main` after the banner so a sender detects the locked state on connect (DOC-08 §5). A no-op when the
/// machine boots Idle.
pub async fn send_boot_alarm() {
  if let ControlState::Alarm(code) = control_state() {
    emit_alarm(code).await;
  }
}

/// Emit the banner WITHOUT blocking, for the reader half's `0x18` handler: the reader must never block (it
/// has to stay free to dispatch the next real-time byte), so a momentarily full [`RESPONSE`] drops this
/// banner. A dropped reset-banner is harmless — the host re-probes readiness with `$I`/`?`, and the
/// consumer's pipeline reset also emits a banner through the guaranteed-delivery path — so the reset is
/// still observable. This is the one response emission that is allowed to drop, precisely because it is on
/// the real-time path.
fn try_send_banner() {
  let mut s = Response::new();
  if ResponseWriter::banner(&mut s).is_ok() {
    let _ = RESPONSE.try_send(s);
  }
}

/// The USB receive task — the *reader half*. It reads USB bytes and, per byte, intercepts real-time
/// commands ([`classify_realtime`]) and dispatches them through their Signals IMMEDIATELY; every other byte
/// is pushed into [`RX_PIPE`] for the [`line_assembler`] to frame. This task NEVER blocks on line flow
/// control: real-time dispatch is non-blocking, and the pipe write is non-blocking (and, for a compliant
/// host, never full — see [`RX_PIPE_CAPACITY`]). So a feed-hold / soft-reset arriving during sustained
/// streaming is acted on within one byte, not after a whole move's worth of back-pressure clears.
#[embassy_executor::task]
pub async fn usb_rx(mut rx: UsbSerialJtagRx<'static, Async>) -> ! {
  // A modest read chunk: the USB Serial/JTAG FIFO is 64 bytes, so reads return promptly.
  let mut buf = [0u8; 64];
  loop {
    let n = match rx.read(&mut buf).await {
      Ok(n) => n,
      // A persistent USB read error must not become a tight spin that starves the other core-0 tasks: back
      // off briefly before retrying. The connection persists across host reopen on native USB, so we do not
      // tear down state here.
      Err(_) => {
        Timer::after(USB_RX_ERROR_BACKOFF).await;
        continue;
      }
    };
    // Host-activity heartbeat: advance once per non-empty read (by the byte count, harmlessly) so the watchdog can
    // tell a host is actively driving the firmware. Bumping per read rather than per byte is enough for the "did it
    // advance since the last tick" check and stays off the per-byte path. `usb_rx` keeps draining the FIFO even when
    // the comms PROCESSING path is wedged (the real-board failure mode), which is EXACTLY why RX activity is the
    // right "host present" signal to gate the comms-stall feed-withhold on.
    if n > 0 {
      RX_ACTIVITY.fetch_add(n as u32, Ordering::Relaxed);
    }
    for &byte in &buf[..n] {
      match classify_realtime(byte) {
        // A real-time byte: dispatch its action and do NOT let it enter a line or the byte buffer.
        Some(cmd) => dispatch_realtime(cmd),
        // An ordinary line byte: hand it to the line-assembly half. `try_write` never blocks, keeping the
        // reader free for the next real-time byte; for a compliant host the pipe (sized to the advertised
        // RX buffer) is never full, so no byte is lost. If a misbehaving host overruns its character-count
        // window the overflowing byte is dropped — the resulting framed line errors, which is the correct
        // push-back for a host that ignored flow control, and real-time dispatch stays alive throughout.
        None => {
          let _ = RX_PIPE.try_write(&[byte]);
        }
      }
    }
  }
}

/// The line-assembly half: drain [`RX_PIPE`] one byte at a time through the [`StreamEngine`] line framer
/// and forward each completed line (blank lines included, as an empty [`Line`]) to [`LINE_QUEUE`]. Blocking
/// on a full `LINE_QUEUE` is CORRECT back-pressure: it stops draining the pipe, the pipe fills, the reader's
/// `try_write` starts refusing bytes, and the host's character-counting throttles. A soft reset
/// ([`LINE_RESET`]) drops any partially framed line so a fresh stream is not contaminated by a half-line
/// from before the reset. Reading a single byte per iteration keeps the reset cancellation point exact (no
/// pre-buffered chunk to discard) and is not a throughput concern — the assembler is bounded by the USB
/// byte rate, not by per-byte overhead.
#[embassy_executor::task]
pub async fn line_assembler() -> ! {
  let mut engine = StreamEngine::new();
  let mut byte = [0u8; 1];
  loop {
    // Race the next byte against a soft reset so a `0x18` mid-line drops the partial line promptly. `read`
    // is cancel-safe (it consumes from the pipe only on completion), so losing this race loses no byte.
    match select(RX_PIPE.read(&mut byte), LINE_RESET.wait()).await {
      Either::First(0) => {}
      Either::First(_) => frame_byte(byte[0], &mut engine).await,
      Either::Second(()) => engine.soft_reset(),
    }
  }
}

/// Feed one byte to the line framer and act on the resulting [`EngineEvent`]: forward a completed line
/// (blank included) to the single in-order consumer, or emit `error:15` immediately for an over-length
/// line. The framer performs no real-time classification or error-hold — those live in the reader half and
/// the consumer respectively (see [`StreamEngine`]).
async fn frame_byte(byte: u8, engine: &mut StreamEngine) {
  match engine.ingest(byte) {
    EngineEvent::None => {}
    EngineEvent::AcceptLine(line) => {
      // Copy the borrowed line into an owned buffer for the channel. A line longer than the buffer cannot
      // occur: the framer already enforced MAX_LINE_LEN, so this never truncates. A blank line is forwarded
      // as an empty `Line`, so the consumer owns both its bare `ok` and the error-hold recovery.
      let mut owned = Line::new();
      let _ = owned.extend_from_slice(line);
      // Block here if the consumer is briefly behind: this back-pressures the host stream (correct flow
      // control) without dropping a line. Real-time bytes already bypassed this path entirely.
      LINE_QUEUE.send(owned).await;
    }
    EngineEvent::Reject(code) => error(code).await,
  }
}

/// Map a classified real-time command onto its Signal / immediate action — all NON-BLOCKING so the reader
/// half is never delayed. Status requests fire the reporter Signal; feed-hold / cycle-start fire theirs; a
/// soft reset / stop clears the byte buffer, flushes any framed-but-unconsumed lines, and signals both the
/// line assembler ([`LINE_RESET`], drop the partial line) and the consumer ([`SOFT_RESET`], reset the
/// parser/planner pipeline and re-emit the banner). The reset-banner is emitted here only on a best-effort
/// basis (`try_send`); the consumer's guaranteed banner is the authoritative one. Override and the
/// Stage-2/3 commands are accepted and currently ignored (documented stubs).
fn dispatch_realtime(cmd: RealtimeCommand) {
  match cmd {
    RealtimeCommand::StatusReport | RealtimeCommand::FullStatusReport => STATUS_REQUEST.signal(()),
    RealtimeCommand::FeedHold => {
      // Latch the feed-hold into the shared control state so the very next `?` reports `Hold:0`, then RAISE the
      // hold LEVEL (Finding #11) and wake the executor so it parks at the next block boundary. The level is the
      // authoritative source of truth — the executor re-reads it at every boundary AND in its empty-queue wait
      // — so a hold can never be missed or stranded. Both updates are synchronous (a `Cell` swap and an atomic
      // store) so the non-blocking reader half is not stalled. `feed_hold` is a no-op on the state outside a
      // running mode (alarm/check/sleep), so only raise the level when the state actually became a hold.
      let next = control_state().feed_hold();
      set_control_state(next);
      if matches!(next, ControlState::Hold(_)) {
        HOLD_REQUESTED.store(true, Ordering::Release);
        HOLD_WAKE.signal(());
      }
    }
    RealtimeCommand::CycleStart => {
      // Cycle-start (`~`/`0x81`): resume a feed-hold ONLY. Gate the executor-hold release on the host-tested
      // predicate [`ControlState::resumes_on_cycle_start`] so a `~` is INERT in Idle/Run/Jog/Alarm/Check AND in
      // Sleep — only a soft reset wakes a sleeping machine (Finding #1). Clearing the hold LEVEL (not signalling
      // an edge that could be drained as stale) is what makes a legitimate resume impossible to lose (Finding
      // #11). When the state is not a hold, leave the level untouched so a stray `~` never releases a `$SLP`
      // park or a not-yet-arrived hold.
      let current = control_state();
      if current.resumes_on_cycle_start() {
        set_control_state(current.cycle_start());
        HOLD_REQUESTED.store(false, Ordering::Release);
        HOLD_WAKE.signal(());
        // If an M0/M1/M6 program-flow pause is the thing holding, nudge it to leave its hold-await and resume the
        // stream (it owns clearing PAUSE_ACTIVE + acking the line). The executor release above is shared with a
        // plain feed-hold; this dedicated nudge wakes the consumer's pause wait without racing the executor wake.
        if PAUSE_ACTIVE.load(Ordering::Acquire) {
          PAUSE_RESUME.signal(());
        }
      }
    }
    RealtimeCommand::SoftReset | RealtimeCommand::Stop => {
      // Latch whether the executor was mid-cycle so the consumer's pipeline reset can apply grbl's rule (a
      // reset aborting an in-progress cycle -> ALARM:3). Read EXECUTOR_RUNNING here, before the executor can
      // clear it on its own MOTION_RESET wake, so the abort decision is race-free. The CONTROL state itself is
      // updated by the consumer's reset_pipeline (which can await), not here.
      RESET_WAS_RUNNING.store(EXECUTOR_RUNNING.load(Ordering::Acquire), Ordering::Relaxed);
      // Drop every buffered RX byte and every framed-but-unconsumed line so post-reset modal state is not
      // contaminated by anything that arrived before the reset. Dedicated Signals are set for each waiter so
      // the assembler drops its partial line, the consumer rebuilds the parser/planner and re-emits the
      // banner, and the core-1 motion executor aborts its block + zeroes the live position — one Signal per
      // waiter because an embassy `Signal` wakes only ONE task (Finding #3).
      RX_PIPE.clear();
      while LINE_QUEUE.try_receive().is_ok() {}
      // Drop any pending jog-cancel so a `0x85` that arrived just before the reset cannot fire AFTER it (the
      // reset already flushes the planner and zeroes the position, superseding any jog-cancel — Phase D).
      JOG_CANCEL.try_take();
      // Clear the hold LEVEL so NO stale hold survives the reset (Finding #2): a feed-hold latched while the
      // executor was idle, or a `$SLP` park, must not leave the executor parked after the warm reset (which
      // returns to Idle / boot-lock, never Hold). Drain any pending `G38.x` PROBE_REQUEST too (Finding #4) so
      // the executor cannot run an UNREQUESTED probe move from the freshly-zeroed origin after the reset — a
      // probe queued just before `0x18` is superseded by the reset, exactly like the flushed planner blocks.
      HOLD_REQUESTED.store(false, Ordering::Release);
      PROBE_REQUEST.try_take();
      // NOTE: no `HARD_LIMIT_TRIPPED` drain here anymore. The hard-limit alarm is now EDGE-armed in the executor
      // (`check_hard_limits` -> `hard_limit_alarm_armed`), and the executor re-seeds its per-axis arming to the
      // settled levels at every reset / post-homing boundary. A switch left ENGAGED after an aborted `$H` seek is
      // therefore a HELD level, not a fresh edge, so it never signals `HARD_LIMIT_TRIPPED` in the first place —
      // there is no stale latch to drain (this was the root of the `error:9` re-lock). A genuinely new over-travel
      // during later motion still alarms on its own fresh edge, and the consumer's `hard_limit_alarm_applies`
      // guard remains as the single defensive layer so a stray trip can never downgrade a more-specific alarm.
      LINE_RESET.signal(());
      SOFT_RESET.signal(());
      // Wake the executor (idle case) and raise the poll-able mid-block abort flag (running case). Set the
      // flag with `Release` before the `Signal` so the executor, on its wake, observes the flag set. The
      // `HOLD_WAKE` nudge releases the executor if it happened to be parked on the (now-cleared) hold level so
      // it proceeds to service the reset rather than waiting on a hold that no longer exists.
      MOTION_RESET_PENDING.store(true, core::sync::atomic::Ordering::Release);
      HOLD_WAKE.signal(());
      MOTION_RESET.signal(());
      // Best-effort immediate readiness banner; the consumer's reset emits the guaranteed one. Dropping
      // this (full RESPONSE) is harmless and keeps the reader non-blocking on the real-time path.
      try_send_banner();
    }
    RealtimeCommand::JogCancel => {
      // Jog cancel (`0x85`, Phase D): only meaningful while a jog is in flight — grbl ignores it otherwise. When
      // jogging, wake the consumer to run the block-boundary stop + jog-block flush + position-sync (it owns the
      // planner; this reader half stays non-blocking). The control-state check is a synchronous `Cell` load.
      if matches!(control_state(), ControlState::Jog) {
        JOG_CANCEL.signal(());
      }
    }
    RealtimeCommand::ProgramStop => {
      // Graceful program stop (`0x86`, Galdr extension): a controlled decelerate-to-Idle that flushes the program
      // and RETAINS position, distinct from the `0x18` abort. Only meaningful while a program is running or held —
      // the host-tested `program_stop_quiesces` gate is true exactly for `Normal`/`Hold`, so a `0x86` from Idle is
      // a (cheap) no-op there and from Alarm/Check/Sleep/Jog is ignored entirely (a jog has its own `0x85` cancel).
      // When the gate holds, wake the consumer (the planner owner) to run the boundary stop + full flush + sync;
      // this reader half stays non-blocking (a synchronous `Cell` load + a coalesced `Signal`).
      if control_state().program_stop_quiesces() {
        PROGRAM_STOP.signal(());
      }
    }
    RealtimeCommand::Override(byte) => {
      // A feed / rapid / spindle / coolant override byte (Phase E, DOC-08 §3). Mutate the shared OVERRIDES in the
      // non-blocking reader half — a `Cell` swap, no await — so the change shows in the very next `?` (`Ov:` and
      // the realized `FS:`). An override is a real-time command and is NEVER acked. grbl ignores a no-op override
      // (one that does not change the value), so we only act when something actually changed. The core-1 executor
      // re-reads OVERRIDES once per block, so a new scale takes effect on the NEXT block boundary — a block
      // already in flight finishes at its current scale (Stage-1 per-block granularity; mid-block re-scaling is
      // the smooth-ramp Stage-2 refinement). No executor wake is needed: the level is re-read at the boundary.
      let mut ov = overrides();
      let prev_spindle = (ov.spindle, ov.spindle_stop);
      if ov.apply(byte) {
        set_overrides(ov);
        // DOC-07: ONLY a spindle-override / spindle-stop change re-drives the LEDC duty. Wake the spindle task to
        // re-apply from the SAME commanded direction at the new override-scaled RPM (its `commanded_spindle`
        // reads the live `Ov:`), so a `0x9E` spindle-stop zeroes the duty and a `0x9A`/`0x9B`/`0x99` adjusts it.
        // A feed / rapid / coolant byte never affects the spindle, so it must NOT wake the task (that would
        // needlessly re-snapshot settings and re-drive the LEDC/GPIO for an unchanged duty). Coolant (`0xA0`/
        // `0xA1`, DOC-06) is still tracked + reportable, but the flood/mist GPIO are not wired.
        if (ov.spindle, ov.spindle_stop) != prev_spindle {
          SPINDLE_UPDATE.signal(());
        }
      }
    }
    RealtimeCommand::ToggleAutoReport => {
      // `0x8C` (Phase F): toggle the auto real-time report mode at runtime. A `Relaxed` flip of the suspend flag,
      // non-blocking on the real-time path; the auto-report task observes it on its next tick (or immediately
      // wakes from a disabled idle via AUTO_REPORT_WAKE). An override is never acked. With the `$481` interval at
      // 0 this still toggles the suspend flag, but the task stays silent until an interval is configured.
      let suspended = AUTO_REPORT_SUSPENDED.fetch_xor(true, Ordering::Relaxed);
      // `fetch_xor` returns the PRIOR value; if we just RESUMED (prior == true) wake the task so it re-arms the
      // interval timer promptly instead of waiting out a stale long sleep.
      if suspended {
        AUTO_REPORT_WAKE.signal(());
      }
    }
    RealtimeCommand::ToggleOptionalStop => {
      // `0x88`: flip the optional-stop switch that gates `M1`. A `Relaxed` flip, non-blocking on the real-time
      // path; the next `M1` pause consults it. Never acked (it is a real-time toggle, not a line). grblHAL leaves
      // this OFF by default, so until a host sends `0x88` an `M1` is a no-op `ok` (an `M0`-equivalent only when on).
      OPTIONAL_STOP_ENABLED.fetch_xor(true, Ordering::Relaxed);
    }
    // Parser-state-on-demand and safety door are accepted but not yet acted on (Stage 2/3). They correctly
    // produce no `ok`.
    RealtimeCommand::ParserStateReport | RealtimeCommand::SafetyDoor => {}
  }
}

/// Wakes the auto-report task (Phase F) when its enable state changes — set by the `0x8C` resume toggle and by a
/// `$481=` write that turns auto-reporting on — so the task re-arms its interval timer immediately rather than
/// sleeping out a stale (possibly long) interval. A `Signal` (coalesced) is enough: the task re-reads the live
/// interval/suspend state after every wake, so a missed-and-coalesced wake loses nothing.
pub static AUTO_REPORT_WAKE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// The single USB writer: drain the [`RESPONSE`] channel and write each formatted response to the USB
/// endpoint. Centralizing writes here means status reports, `ok`s, errors, and `$`-report lines never
/// interleave on the wire (DOC-08).
#[embassy_executor::task]
pub async fn usb_tx(mut tx: UsbSerialJtagTx<'static, Async>) -> ! {
  loop {
    let resp = RESPONSE.receive().await;
    // Comms-progress heartbeat: a response/ack/status line is leaving the firmware, which is the most direct
    // evidence the core-0 comms path is making host-facing forward progress. Bumped here (the single writer) so the
    // task-watchdog sees real output flow; a stuck pipeline produces no responses, so this counter freezes — the
    // signal the watchdog needs. Bumped BEFORE the write so even a write that the host-closed-port path drops still
    // counts as "the firmware produced a response" (the wedge is upstream of the writer, not in the USB write).
    COMMS_PROGRESS.fetch_add(1, Ordering::Relaxed);
    // A write error on native USB means the host closed the port; drop the byte and continue, since the
    // connection re-establishes on host reopen without a controller reset.
    let _ = tx.write_all(resp.as_bytes()).await;
    let _ = tx.flush().await;
  }
}

/// The real parser → planner consumer (replaces the Stage-1 stub). It is the SINGLE, in-order consumer of
/// [`LINE_QUEUE`], so it is the natural owner of the grblHAL gcode error-hold (see [`ConsumerState`]). Per
/// line it: routes `$` system commands to their handlers; parses GCode through a persistent [`Parser`];
/// and feeds the resulting [`PlannerCommand`](firmware_core::gcode::PlannerCommand) to a persistent
/// [`Planner`]. It emits exactly one `ok`/`error:N` per consumed line, preserving the one-response-per-line
/// contract end to end (`line_assembler` only ever responds for the protocol-level overflow reject it
/// handles itself, so there is no double-response or gap at the boundary).
///
/// ## Error-hold ownership (race-free by construction)
/// The grblHAL contract holds all subsequent lines in an error state after a GCode line errors, until a
/// reset / empty line / `$` command. That hold lives HERE, in [`ConsumerState::error_hold`], not in the
/// `StreamEngine`: the framer runs in `line_assembler` and forwards lines asynchronously, so it cannot know
/// a line errored downstream, and any back-channel to it would race the lines already in flight in
/// `LINE_QUEUE`. Because this task is the only reader of `LINE_QUEUE` and sees parse/plan results strictly
/// in queue order, owning the hold here is inherently in-order and race-free. The `StreamEngine` was
/// deliberately reduced to pure line framing (no error-hold state); the framer still owns the independent
/// protocol-level overflow reject, which is correct.
///
/// ## Back-pressure (gated on the core-1 motion executor draining blocks)
/// `ok` for a move is emitted only once the block is ACCEPTED into the planner buffer. When the planner is
/// full ([`PlannerError::QueueFull`]) the consumer neither acks nor drops the line: it waits for the core-1
/// `motion_executor` to execute a block and free a slot, then retries the SAME command (the arc planner is
/// all-or-nothing on `QueueFull`, so re-issuing is safe). While waiting it stops reading `LINE_QUEUE`, which backs up, blocks
/// `line_assembler`'s `send().await`, stops draining `RX_PIPE`, fills the pipe, makes the reader's
/// `try_write` refuse bytes, and lets the host's character-counting throttle — exactly the correct grbl flow
/// control, now with real-time dispatch still live throughout because it sits in the separate reader half.
#[embassy_executor::task]
pub async fn comms_consumer(flash: &'static SharedFlash) -> ! {
  let mut parser = Parser::new();
  let mut state = ConsumerState::default();
  loop {
    // Coalesced settings persist (Finding #14b): if a `$n=val`/`$PBX` change is pending and the input has
    // drained (no more lines queued), this is a burst boundary — flush the live settings to flash ONCE for
    // the whole burst before blocking for the next event, instead of writing per line. The flush yields the
    // executor while the flash op runs; `is_empty` is the cheap "host paused" signal the brief specifies.
    // `motion_idle()` additionally defers the persist while a cycle is in flight: the esp-storage flash write
    // parks the real-time motion core (`multicore_auto_park`), so flushing mid-cycle would briefly stall step
    // generation — a still-pending change is caught by the safety tick once motion drains to idle.
    if SETTINGS_DIRTY.load(Ordering::Acquire) && LINE_QUEUE.is_empty() && motion_idle().await {
      // `false`: this path fires every loop iteration while dirty, so it must NOT re-mark on failure or it would
      // busy-retry a persistently-failing write each loop. A failed write here is retried by the safety timer.
      flush_settings(flash, false).await;
    }
    // Coalesced coordinate persist (Phase B): same burst-boundary rule for the persistent G54-G59 / G28 / G30
    // record (also deferred while moving, for the same auto-park reason), so a program that re-zeroes several
    // axes appends the coordinate blob once, not per line.
    if COORDINATES_DIRTY.load(Ordering::Acquire) && LINE_QUEUE.is_empty() && motion_idle().await {
      flush_coordinates(flash, false).await;
    }
    // Race the next line against a soft reset AND a periodic safety flush. A `0x18` resets the parser modal
    // state, clears the error-hold, and flushes the planner queue (and persists any pending settings) BEFORE
    // the next line is parsed; `usb_rx` already cleared `RX_PIPE` + `LINE_QUEUE` and signalled the line
    // assembler. The safety-flush timer guarantees a dirty change is persisted within [`SETTINGS_FLUSH_SAFETY`]
    // even if the queue never observably empties (e.g. a slow trickle that always has one line in flight). A
    // line that lost the reset race is dropped (it predates the reset), matching grbl's warm-reset semantics.
    //
    // Known window (Finding #6, accepted): the select only observes the reset at the loop boundary. If a
    // `0x18` lands while `handle_line` is already mid-flight for a non-back-pressured line, that line can
    // still emit its `ok`/`error` after the reset signal — a single stray response. Back-pressured lines do
    // observe the reset (they race `SOFT_RESET` inside `plan_command` and return `Aborted`). Closing the
    // remaining window for an in-flight non-back-pressured line would require cancelling `handle_line`
    // mid-await; that is deferred to the alarm-state machine (Stage 2), which is where grbl gates response
    // emission during an abort. The stray response is benign: the host discards pending acks on `0x18`.
    let events = select4(LINE_QUEUE.receive(), SOFT_RESET.wait(), JOG_CANCEL.wait(), Timer::after(SETTINGS_FLUSH_SAFETY));
    // Race the four primary events against a hard-limit trip from the core-1 executor (DOC-06): a limit pressed
    // during normal motion must halt the program and enter `ALARM:1` regardless of what the consumer is waiting on.
    // Also race a graceful program stop (`0x86`): a controlled decelerate-to-Idle + full flush that, unlike the
    // hard-limit trip and the `0x18` reset, raises no alarm and retains position. Nested `select`s keep each arm typed.
    match select(select(events, HARD_LIMIT_TRIPPED.wait()), PROGRAM_STOP.wait()).await {
      Either::First(Either::First(Either4::First(line))) => {
        // Comms-progress heartbeat: a line was fully processed (parsed, planned/queued, acked). This is the
        // consumer-side companion to the `usb_tx`/`status_responder` bumps — together they prove the WHOLE
        // host-facing pipeline (parse -> plan -> respond) is advancing. A consumer stuck on a never-resolving
        // back-pressure / probe / home await stops bumping this, which (with RX live) trips the comms-stall reset.
        handle_line(line.as_slice(), &mut parser, &mut state, flash).await;
        COMMS_PROGRESS.fetch_add(1, Ordering::Relaxed);
      }
      // A soft reset must not lose a pending settings change: persist before rebuilding the pipeline (grbl
      // applies most settings on the next reset, so they MUST be on flash by the time the reset takes them).
      Either::First(Either::First(Either4::Second(()))) => {
        // `true`: a failed persist here must be retried (by the safety timer or the next reset), not dropped —
        // grbl applies settings on the next reset, so a lost write would mean the reset takes stale flash values.
        flush_settings(flash, true).await;
        flush_coordinates(flash, true).await;
        apply_soft_reset(&mut parser, &mut state).await;
      }
      // Jog cancel (`0x85`, Phase D): reuse the feed-hold block-boundary stop, flush the jog blocks, sync the
      // planner to the live stop point, and return to Idle. Owned here (the planner owner) so it is race-free
      // with line handling — a cancel and a line never run concurrently in this single in-order consumer.
      Either::First(Either::First(Either4::Third(()))) => cancel_jog_cycle().await,
      // Safety-interval tick: persist any pending change even if the queue never observably drained — but only
      // while the machine is idle, since the flash write parks the real-time motion core (see `motion_idle`). A
      // change made mid-cycle therefore persists on the first safety tick AFTER motion drains (<=1s later); the
      // `||`-guarded `motion_idle()` is skipped entirely when nothing is dirty so an idle board never locks the
      // planner here. When nothing is dirty these are cheap no-ops and the loop simply re-arms the timer.
      Either::First(Either::First(Either4::Fourth(()))) => {
        let pending = SETTINGS_DIRTY.load(Ordering::Acquire) || COORDINATES_DIRTY.load(Ordering::Acquire);
        if pending && motion_idle().await {
          // `true`: the safety interval is the bounded-cadence retry for a failed write — re-marking dirty here lets
          // the next tick re-attempt, which is exactly the guarantee Bug A defeated (a single failure dropped it).
          flush_settings(flash, true).await;
          flush_coordinates(flash, true).await;
        }
      }
      // Hard-limit trip (`$21`, DOC-06): the executor detected a switch trip during normal motion. Enter the
      // LOCKED `ALARM:1` (position is likely lost from the abrupt stop — re-homing recommended) and reset the
      // pipeline so the queue is flushed and the machine sits in a clean, clearly-halted alarm. Only a soft
      // reset clears a locked alarm.
      Either::First(Either::Second(())) => {
        // Guard against a STALE trip clobbering an already-halted machine (Finding #5b): raise `ALARM:1` only
        // from a state where the machine could actually be MOVING (`Normal`/`Hold`/`Jog`/`Check`). The
        // host-tested `hard_limit_alarm_applies` predicate decides. If we are already in an alarm (or asleep), a
        // trip here is a stale read of a parked switch — re-raising would only downgrade a more-specific lock,
        // most damagingly turning the `ALARM:11` boot-lock into the locked `ALARM:1`, which `$X` cannot clear
        // (the `error:9` wedge). A legitimately NEW over-travel always arrives from a moving state, so this never
        // suppresses a real trip; the soft-reset drains above are the primary fix and this is the last guard.
        if control_state().hard_limit_alarm_applies() {
          set_control_state(ControlState::Alarm(AlarmCode::HardLimit));
          emit_alarm(AlarmCode::HardLimit).await;
          reset_pipeline(&mut parser, &mut state).await;
        }
      }
      // Graceful program stop (`0x86`, Galdr extension): a controlled decelerate-to-Idle that flushes the program
      // and RETAINS position — the operator's "stop the job cleanly", distinct from the `0x18` abort. Owned here
      // (the planner owner) so it is race-free with line handling; the reader half only signalled it after the
      // `program_stop_quiesces` gate held. Re-checks the live state inside the cycle so a state change between the
      // signal and here (e.g. a soft reset winning a tie) makes it a benign no-op.
      Either::Second(()) => program_stop_cycle(&mut parser, &mut state).await,
    }
  }
}

/// Persist the live [`SETTINGS`] to flash IF a change is pending, clearing [`SETTINGS_DIRTY`]. This is the
/// single coalesced write path (Finding #14b): callers mark settings dirty per `$n=val`/`$PBX` line, and this
/// performs the actual flash append once per burst (queue-empty), on the safety interval, and on soft reset.
///
/// The dirty flag is cleared BEFORE the write so a change landing during the (awaited) flash op re-marks dirty
/// and is caught by the next flush — never silently coalesced away.
///
/// On write FAILURE we re-mark the dirty flag when `retry_on_failure` is set, so a transient flash error does not
/// permanently drop the change (Bug A): the soft-reset and safety-interval guarantees only hold if a failed write
/// is re-attempted. The burst-boundary caller at the top of the consumer loop passes `false` — it fires every
/// iteration while `dirty && queue empty`, so re-marking there would busy-retry a persistently-failing write each
/// loop. The safety-timer and soft-reset arms pass `true`: they re-attempt on a BOUNDED cadence (the next
/// [`SETTINGS_FLUSH_SAFETY`] tick, or the next reset) rather than spinning. The in-RAM value already applied and
/// the line was already `ok`'d either way — a stalled persist must never wedge a character-counting sender.
async fn flush_settings(flash: &'static SharedFlash, retry_on_failure: bool) {
  // Clear first so a concurrent `$n=val` applied during the await re-sets the flag and is not lost.
  if !SETTINGS_DIRTY.swap(false, Ordering::AcqRel) {
    return;
  }
  let snapshot = settings_snapshot().await;
  let mut store = FlashRecordStore::settings(flash);
  if settings::store_settings(&mut store, &snapshot).await.is_err() {
    #[cfg(feature = "defmt")]
    defmt::warn!("settings: failed to flush settings to flash");
    // Re-mark so the next safety-timer tick or soft reset retries; the burst-boundary path passes `false` to
    // avoid spinning the failing write every loop. The re-mark cannot clobber a concurrent newer `$n=val`: that
    // write also sets the flag, so the worst case is the same flag already being set.
    if retry_on_failure {
      SETTINGS_DIRTY.store(true, Ordering::Release);
    }
  }
}

/// Apply a `0x18` soft reset: compute and publish the post-reset [`ControlState`] from grbl's rules (a reset
/// that ABORTED an in-progress cycle → `ALARM:3`; otherwise the boot state — homing-lock when `$22` is set,
/// else Normal), emit `ALARM:N` when the reset lands in an alarm, then rebuild the parser/planner pipeline.
/// `was_in_cycle` is the [`RESET_WAS_RUNNING`] latch the reader half captured at dispatch time, so the abort
/// decision is race-free with the executor's own reset. The boot-lock branch also re-prompts `[MSG:..unlock]`.
async fn apply_soft_reset(parser: &mut Parser, state: &mut ConsumerState) {
  let was_in_cycle = RESET_WAS_RUNNING.swap(false, Ordering::Relaxed);
  let homing_enabled = HOMING_ENABLED.load(Ordering::Relaxed);
  let next = control_state().soft_reset(was_in_cycle, homing_enabled);
  set_control_state(next);
  // A reset returns to the boot baseline (finding #8): with homing enabled the machine is unhomed again (back in
  // the `ALARM:11` lock until `$H`), so position certainty — and thus `$20` soft-limit enforcement — is lost.
  // With homing disabled the machine stays "homed" (face-value position), mirroring `init_control_state`.
  store_homed_baseline(homing_enabled);
  // Rebuild the pipeline FIRST so the banner (the "reset and ready" signal) is emitted, then push any
  // resulting alarm so a host sees `ALARM:N` and the `[MSG:..]` prompt right after the banner — matching grbl's
  // connect/reset ordering.
  reset_pipeline(parser, state).await;
  // No `HARD_LIMIT_TRIPPED` drain here anymore (Finding #5b is now fixed at the source). The hard-limit alarm is
  // EDGE-armed in the executor, which re-seeds its per-axis arming to the settled levels at the reset / post-homing
  // boundaries — so a switch still parked engaged after an aborted `$H` seek is a HELD level, not a fresh edge, and
  // the executor's concurrent block-boundary / idle re-samples during this `reset_pipeline` await window can no
  // longer RE-latch a stale trip. The `error:9` re-lock is prevented by the arming, not by draining a latch after
  // the fact. The `hard_limit_alarm_applies` guard on the consumer's hard-limit arm stays as the one defensive
  // layer: a genuinely NEW over-travel re-signals from a moving state and still alarms, while a stray trip arriving
  // while already alarmed/asleep cannot downgrade a more-specific lock (e.g. `ALARM:11`) into the locked `ALARM:1`.
  if let ControlState::Alarm(code) = next {
    emit_alarm(code).await;
  }
}

/// Emit an `ALARM:N` push line plus its `[MSG:..]` unlock/continue prompt, so a host detects the halt and
/// learns how to clear it (`$H`/`$X` for homing/locked alarms, reset for the recoverable ones).
async fn emit_alarm(code: AlarmCode) {
  // DOC-07: any ALARM forces the spindle off IMMEDIATELY, independent of motion state — a hard limit, soft
  // limit, homing failure, or any other alarm kills the spindle. Issue the e-stop FIRST, before the message
  // enqueues/awaits below: `force_spindle_off` is synchronous (an atomic store + a coalesced `Signal`), so the
  // spindle is parked at once even if the RESPONSE channel is back-pressured and the alarm-text awaits stall.
  // The commanded direction is also cleared so a later `~`/`$X` does not silently restart a spindle that the
  // program never re-commanded. (A feed hold does NOT route through here, so it leaves the spindle running,
  // matching grblHAL.) This is the universal alarm-emit chokepoint, so the e-stop lives here once. Coolant is
  // forced off alongside the spindle: DOC-07 kills both on any alarm (a feed hold, which does not route here,
  // leaves both running).
  force_spindle_off();
  force_coolant_off();
  let mut a = Response::new();
  if ResponseWriter::alarm(&mut a, code).is_ok() {
    enqueue(a).await;
  }
  // A `[MSG:ALARM:N <name>]` context push so a plain terminal sees what halted the machine, then grbl's
  // standard unlock/continue prompt as its own `[MSG:]`.
  let mut ctx = Response::new();
  if ResponseWriter::alarm_context(&mut ctx, code).is_ok() {
    enqueue(ctx).await;
  }
  send_message(code.unlock_hint()).await;
}

/// Force the spindle to a hard stop (DOC-07): clear the commanded direction to Stop and signal the spindle task's
/// emergency stop. Used by every spindle-killing event — ALARM ([`emit_alarm`]), soft reset ([`reset_pipeline`]),
/// and sleep ([`handle_sleep`]). Synchronous (an atomic store + a coalesced `Signal`), callable from any context.
fn force_spindle_off() {
  SPINDLE_DIRECTION.store(SPINDLE_DIR_STOP, Ordering::Release);
  SPINDLE_ESTOP.signal(());
}

/// Force coolant OFF (DOC-07 safety): clear the commanded modal coolant state and signal the [`coolant`] task's
/// emergency stop. Used by every coolant-killing event — ALARM ([`emit_alarm`]), soft reset ([`reset_pipeline`]),
/// program end ([`program_end`]), graceful stop ([`program_stop_cycle`]), and sleep ([`handle_sleep`]) — mirroring
/// [`force_spindle_off`]. Synchronous (an atomic store + a coalesced `Signal`), callable from any context.
fn force_coolant_off() {
  COOLANT_STATE.store(0, Ordering::Release);
  COOLANT_ESTOP.signal(());
}

/// Drive the [`coolant`] task from the parser's MODAL coolant state (M7/M8/M9), called after every clean parse.
/// Like [`sync_spindle_from_modal`], keying off modal state (not the per-line emit) is what makes an M7/M8 that
/// SHARES a line with a move still actuate coolant (the emit is the move). Acts only on a real change vs the last
/// dispatched mask. Synchronous (an atomic store + a coalesced `Signal`).
fn sync_coolant_from_modal(modal: &ModalState, state: &mut ConsumerState) {
  let mask = coolant_mask(modal.coolant);
  if mask != state.last_coolant {
    COOLANT_STATE.store(mask, Ordering::Release);
    COOLANT_UPDATE.signal(());
    state.last_coolant = mask;
  }
}

/// Pack a [`CoolantState`](firmware_core::gcode::CoolantState) into the [`COOLANT_STATE`] bitmask.
fn coolant_mask(coolant: firmware_core::gcode::CoolantState) -> u8 {
  (if coolant.mist { COOLANT_BIT_MIST } else { 0 }) | (if coolant.flood { COOLANT_BIT_FLOOD } else { 0 })
}

/// Unpack the live [`COOLANT_STATE`] bitmask into a [`CoolantState`](firmware_core::gcode::CoolantState), for the
/// [`coolant`] task. An `Acquire` load pairs with the consumer's `Release` store.
pub fn commanded_coolant() -> firmware_core::gcode::CoolantState {
  let mask = COOLANT_STATE.load(Ordering::Acquire);
  firmware_core::gcode::CoolantState {
    mist: mask & COOLANT_BIT_MIST != 0,
    flood: mask & COOLANT_BIT_FLOOD != 0,
  }
}

/// Reset the parser/planner pipeline state this task owns on a soft reset (`0x18`): restore the parser to
/// default modal state, clear the gcode error-hold, flush the planner queue, reset the non-position snapshot
/// fields to idle, and emit the guaranteed readiness banner. The `RX_PIPE`, the `line_assembler`'s partial
/// line, and `LINE_QUEUE` were already cleared by `usb_rx`; this completes the warm reset for the downstream
/// half so a fresh stream starts from the documented modal defaults.
///
/// ## Machine position is RETAINED (Change A)
/// The LIVE MACHINE POSITION is deliberately NOT zeroed: matching grbl, a `0x18` abort RETAINS MPos so `$X`
/// unlocks at the same coordinates. The core-1 motion executor is the single owner of the live [`LIVE_POSITION`]
/// atomics and now RETAINS them across [`MOTION_RESET`] (it re-publishes the last step position, never zeroes it),
/// so the consumer must NOT write them — that would be a cross-core stale-overwrite race (Finding #3). But the
/// rebuilt [`Planner`] starts at the step origin, so this SYNCS its commanded position to the retained live step
/// position via [`Planner::sync_position`], keeping the planner's notion of position consistent with the retained
/// MPos: a subsequent ABSOLUTE move then resolves relative to the retained position, not the origin. Resetting
/// `MACHINE` to idle here only restores the fields the executor does not own (run-state / feed / spindle /
/// RX-free); `status_responder` reads MPos live. (Small accepted race: on an abort DURING motion the executor may
/// still be finishing its mid-block abort when this reads `LIVE_POSITION`; the step counter only advances
/// monotonically within a block, so the read is a valid recent step position — "suspect" exactly as grbl
/// documents an aborted-mid-move position, recovered by `$H`.)
async fn reset_pipeline(parser: &mut Parser, state: &mut ConsumerState) {
  // `Parser` exposes no in-place reset; reconstructing it restores the documented power-on modal defaults
  // (G0, G90, G21, F0, S0) — exactly grbl's warm-reset modal state. The CURRENT tool is RETAINED across the reset
  // (grbl keeps the physically-loaded tool through a soft reset), so snapshot it and restore it after the rebuild.
  let retained_tool = parser.state().current_tool;
  *parser = Parser::new();
  parser.set_current_tool(retained_tool);
  state.error_hold = false;
  // Drop any partially accumulated `$PBX=` import so a frame begun before the reset cannot bleed into one after.
  state.pb.reset();
  // DOC-07: clear the spin-up gate so a `M3` armed before the reset cannot inject a spurious `$392` dwell ahead of
  // the first post-reset move (the spindle is off after the reset's `force_spindle_off`). Reset the last-dispatched
  // spindle tracking to match the reconstructed parser's modal `Stop`/`S0`, so the first post-reset M3/M4/`S` is
  // seen as a fresh change by `sync_spindle_from_modal`.
  state.spin_up = SpinUpGate::new();
  state.last_spindle_dir = SpindleState::Stop;
  state.last_spindle_rpm = 0;
  // Reconstruct the planner to clear the block queue, work offset, and junction state in one step (it has no
  // public flush), rebuilding it from the LIVE settings so any `$x=val` changes made before the reset take effect
  // now (grbl applies most settings on the next reset). Snapshot the settings first so the `SETTINGS` lock is
  // released before the `PLANNER` lock is taken. The rebuilt planner starts at the step origin, but the live MPos
  // is RETAINED (Change A) by the executor, so SYNC the planner's commanded position to the retained live step
  // position — keeping the planner consistent with the retained MPos so a subsequent absolute move resolves from it
  // rather than the origin. `sync_position` only sets the position + clears junction state, so it is safe before the
  // WCO push below (which sets the work offset, untouched here). Reset the published snapshot's non-position fields
  // to idle; the live MPos atomics are retained by the executor on `MOTION_RESET`, not written here (no race).
  let planner_config = settings_snapshot().await.planner_config();
  let retained_steps = read_live_position();
  {
    let mut guard = PLANNER.lock().await;
    let mut planner = Planner::new(planner_config);
    planner.sync_position(retained_steps);
    *guard = Some(planner);
  }
  // Coordinate model on soft reset (grbl): the SESSION-only offsets (G92, dynamic TLO) clear to identity while
  // the persistent G54-G59 / G28 / G30 survive. Clear the volatile offsets, then push the recomputed WCO into
  // the freshly-rebuilt planner so absolute moves resolve against the surviving WCS offset (the new planner
  // starts with a zero offset). Reset the WCO refresh cadence so the next `?` re-emits `WCO:` (grbl's rule).
  {
    let mut coords = coordinates();
    coords.clear_volatile();
    set_coordinates(coords);
  }
  push_wco_to_planner().await;
  reset_wco_reporter();
  // Overrides reset to their defaults (100/100/100, no spindle-stop, coolant off) on a soft reset (grbl resets
  // the run-time overrides on `0x18`); clear the published live feed/spindle so `FS:` reads 0 until motion
  // resumes, and reset the `Ov:` cadence so the next `?` re-emits `Ov:` (grbl's first-report-after-reset rule).
  set_overrides(Overrides::new());
  LIVE_PROGRAMMED_FEED_MM_MIN.store(0, Ordering::Release);
  LIVE_BLOCK_IS_RAPID.store(false, Ordering::Relaxed);
  PROGRAMMED_SPINDLE_RPM.store(0, Ordering::Release);
  // DOC-07: a soft reset (`0x18`) forces the spindle off (grbl resets the spindle on reset), independent of
  // whether the reset lands in an alarm — a reset from Idle goes to Normal and never calls `emit_alarm`, so the
  // e-stop is issued here unconditionally. The commanded direction is cleared so the spindle stays off until a
  // fresh M3/M4 after the reset.
  force_spindle_off();
  // DOC-07: a soft reset also forces coolant off (grbl resets coolant on `0x18`). Clear the commanded state and
  // the consumer's last-dispatched mask so the first post-reset M7/M8 is seen as a fresh change.
  force_coolant_off();
  state.last_coolant = 0;
  reset_ov_reporter();
  // Phase F: a soft reset re-seeds the auto-report cadence from the (post-reset) live `$481` and clears the
  // `0x8C` runtime suspend, so auto-reporting returns to its configured state. Wake the task so it re-arms (or
  // parks) against the refreshed interval immediately. The setter applied any pre-reset `$481=` to the live
  // settings already, so this picks up the latest value (grbl applies most settings on the next reset).
  let auto_report_interval = settings_snapshot().await.auto_report_interval_ms();
  init_auto_report(auto_report_interval);
  AUTO_REPORT_WAKE.signal(());
  // grbl drops the non-persistent `G38.2` probe data on a soft reset, like G92/TLO; clear the last-probe slot so
  // `$#` reports `[PRB:0,0,0:0]` after a reset until the next probe.
  set_last_probe(LastProbe::none());
  {
    let mut snap = MACHINE.lock().await;
    *snap = MachineSnapshot::idle();
  }
  // The authoritative reset banner, delivered guaranteed (the reader half's best-effort `try_send` may have
  // dropped its copy under a momentarily full RESPONSE channel). A host treats this as "reset and ready".
  send_banner().await;
}

/// The consumer's persistent control state across lines: the gcode error-hold flag. Held locally in the
/// single consumer task so it is mutated only in line order.
#[derive(Default)]
struct ConsumerState {
  /// True once a GCode line errored and no recovery trigger (blank line / `$` command / soft reset) has
  /// cleared it yet. While set, GCode lines are rejected without parsing, per grblHAL safety behavior.
  error_hold: bool,
  /// Reassembly state for a chunked `$PBX=<hex>` bulk settings import (a full frame spans several lines).
  /// Lives here so it persists across the per-line `handle_line` calls; reset on a soft reset.
  pb: PbReceiver,
  /// The two stored startup GCode lines (`$N0`/`$N1`). Phase A SCOPE: these are stored and echoed by `$N` but
  /// NOT executed on reset (running startup lines on init is a deliberate follow-up — see `handle_startup_set`).
  /// They are held in RAM only: persisting them needs either a separate NVS record or a non-scalar proto field
  /// (which would break `Settings: Copy`), both larger than Phase A — flagged as a TODO(DOC-04) in the report.
  startup_lines: [StartupLine; 2],
  /// The DOC-07 spin-up gate: tracks whether an M3/M4 owes a `$392` spin-up dwell to the next cutting move, so
  /// the consumer injects a synthetic [`PlannerCommand::Dwell`] ahead of that move (the planner cannot emit two
  /// outcomes from one command). The when-to-insert decision is host-tested in [`SpinUpGate`]; the consumer
  /// supplies the dwell seconds from the live `$392`. Reset (cleared) on a soft reset with the rest of the state.
  spin_up: SpinUpGate,
  /// The last spindle direction this consumer dispatched to the [`spindle`] task (DOC-07). Compared against the
  /// parser's modal `spindle` after each clean parse so a direction change — including an M3/M4 that SHARED a line
  /// with a move (where the per-line emit is the move, not a `Spindle` command) — re-drives the outputs exactly
  /// once. Reset to `Stop` on a soft reset alongside the parser. See [`sync_spindle_from_modal`].
  last_spindle_dir: SpindleState,
  /// The last programmed spindle RPM this consumer dispatched (whole RPM, the modal `S` clamped to `u16`). Lets a
  /// bare `S` change re-drive the duty of a RUNNING spindle (grbl updates a spinning spindle's speed on a lone
  /// `S`); compared only while the spindle is running. Reset to `0` on a soft reset.
  last_spindle_rpm: u16,
  /// The last COOLANT bitmask this consumer dispatched to the [`coolant`] task (`bit0 = mist`, `bit1 = flood`).
  /// Compared against the parser's modal coolant after each clean parse so an M7/M8/M9 change — including one that
  /// SHARED a line with a move — re-drives the outputs exactly once. Reset to `0` (off) on a soft reset / M30.
  last_coolant: u8,
}

/// The maximum stored startup-line length, in bytes. Bounded so the `$N` echo (`$Nn=<gcode>\r\n`) always fits
/// one [`Response`] buffer: the `$Nn=` prefix is 4 bytes and the CRLF 2, so the GCode body must leave room
/// inside [`RESPONSE_CAPACITY`]. grbl startup lines (e.g. `G54G20`) are short, so this is generous; a longer
/// `$N0=` is rejected rather than stored-then-silently-dropped from the echo.
const STARTUP_LINE_MAX: usize = RESPONSE_CAPACITY - 8;

/// One stored startup GCode line (`$N0`/`$N1`), capped to [`STARTUP_LINE_MAX`] so its `$N` echo always fits a
/// [`Response`]. `None` when unset, echoed as an empty `$Nn=` by `$N`. Held by [`ConsumerState`]; cleared on
/// `$RST=*`/`$RST=$` and rebuilt on reset.
type StartupLine = Option<heapless::Vec<u8, STARTUP_LINE_MAX>>;

/// Route one accepted line. `$` system commands are dispatched to their report handlers and clear the
/// error-hold (a recovery trigger). Otherwise the line is parsed and planned. Exactly one `ok`/`error:N`
/// is emitted per call, preserving the one-response-per-line contract.
async fn handle_line(line: &[u8], parser: &mut Parser, state: &mut ConsumerState, flash: &'static SharedFlash) {
  let trimmed = trim_ascii(line);
  if trimmed.is_empty() {
    // A blank line (forwarded by `usb_rx`) is a grblHAL error-hold recovery trigger: clear the hold and
    // acknowledge with a bare `ok`. Handling it here, in queue order, keeps recovery race-free with the
    // surrounding lines, since the hold lives in this task, the sole in-order reader of `LINE_QUEUE`.
    state.error_hold = false;
    ack().await;
  } else if let Some(jog_line) = strip_jog_prefix(trimmed) {
    // `$J=` is a MOTION command, not a `$` setting — route it BEFORE the `$`-system dispatch (Phase D). Like a
    // GCode motion line it does NOT clear the error-hold by itself (only a blank line, a real `$` command, or a
    // soft reset do); `handle_jog` honors the hold and the control-state gating, and emits exactly one ok/error.
    handle_jog(jog_line, parser, state).await;
  } else if let Some(rest) = trimmed.strip_prefix(b"$") {
    // A `$` system command is the other grblHAL recovery trigger and is answered by its handler. Both
    // recovery triggers (an empty line and a `$` command) and a soft reset clear the hold; nothing else.
    // The parser is passed mutably so `$G` can report live modal state AND so `$C`-off / `$RST=$` can rebuild
    // it as part of their soft reset. `flash` lets `$RST=$` persist the restored defaults.
    state.error_hold = false;
    handle_system_command(rest, parser, state, flash).await;
  } else {
    plan_gcode_line(trimmed, parser, state, flash).await;
  }
}

/// Parse and plan one GCode line, emitting exactly one `ok`/`error:N`. Honors the error-hold: while held,
/// a GCode line is rejected without parsing. On a parse or planner error the hold is armed; on acceptance
/// (including modal-only `Ok(None)` lines) a single `ok` is emitted.
async fn plan_gcode_line(
  line: &[u8],
  parser: &mut Parser,
  state: &mut ConsumerState,
  flash: &'static SharedFlash,
) {
  if state.error_hold {
    // Held by a prior error: reject without parsing until a recovery trigger. Reuse the generic
    // "expected command letter" code, matching how a sender already in error-recovery treats any further
    // rejection — it halts the stream regardless of the specific code (mirrors the engine's hold code).
    // Emit it bare: the held line may be perfectly valid GCode, so the code's "Expected command letter"
    // name does not describe the rejection and a `[MSG:..]` annotation would mislead a plain terminal.
    error_bare(ERROR_HOLD_CODE).await;
    return;
  }
  // Drop a stale `Jog` latch back to Normal if the jog has fully drained, so a program line after a jog finishes
  // is accepted (and `?` reads Idle). If a jog is still ACTIVE this leaves the state `Jog` and the line is
  // rejected below — a program move never blends into an in-flight jog.
  refresh_jog_state().await;
  // In an alarm or sleep, GCode is blocked entirely (motion not allowed and not even parsed for `ok`): reject
  // with the unsupported-command code so a sender halts. Boot-lock (`$22` homing required) lands here too,
  // forcing the host to `$H`/`$X` before streaming. An active jog (`Jog`) likewise blocks program GCode (grbl:
  // the machine is busy jogging). Check mode is handled below (parse + `ok`, no plan).
  let control = control_state();
  if matches!(control, ControlState::Alarm(_) | ControlState::Sleep | ControlState::Jog) {
    // grbl's "G-code locked out during alarm or jog state" is `error:9`, NOT the generic post-error hold code
    // (Finding #5). Use the dedicated `ERROR_LOCKED` so a sender's display matches the `$EE` table (which
    // already defines code 9) and distinguishes a state lockout from an in-stream parse-error hold.
    error(ERROR_LOCKED).await;
    return;
  }
  let parsed = parser.parse_line(line);
  // Keep the active WCS in the coordinate model in sync with the parser's modal `wcs` BEFORE planning, so a
  // move sharing a line with a G54-G59 select (e.g. `G55 G0 X1`) uses the right offset — the parser emits the
  // move (not a SelectWcs op) in that case, so the WCS change would otherwise be missed until a bare select.
  if matches!(parsed, Ok(Some(_))) {
    sync_active_wcs(parser.state().wcs).await;
  }
  // Phase E / DOC-07: on any clean parse, publish the parser's modal `S` as the PROGRAMMED spindle RPM (so
  // `status_responder` renders the override-scaled `FS:` and the spindle task reads the new speed), THEN drive the
  // spindle task from the modal spindle direction. Doing it here — keyed off MODAL state, not the per-line emit —
  // is what makes an M3/M4/M5 take effect on a line that ALSO carries a move (the emit is the move, not a `Spindle`
  // command) and lets a bare `S` re-drive a running spindle. The modal `S`/direction persist across lines.
  if parsed.is_ok() {
    let modal = parser.state();
    let rpm = modal.spindle_speed.max(0.0) as u32;
    PROGRAMMED_SPINDLE_RPM.store(rpm.min(u16::MAX as u32), Ordering::Release);
    // Drive the spindle from modal state — but NOT in `$C` check mode: a dry run must validate a program without
    // actuating any output (grbl check mode moves nothing and does not energize the spindle). Publishing the
    // PROGRAMMED RPM above is report-only and safe; `sync_spindle_from_modal` actuates the LEDC/SPIN_EN hardware,
    // so it is gated here. The check guard in the `match` below only suppresses planning, which is too late.
    if control != ControlState::Check {
      sync_spindle_from_modal(modal, state);
      // Drive coolant from modal state too (M7/M8/M9), gated off check mode for the same reason: a dry run must
      // validate the program without actuating any output. Keying off modal state makes an M7/M8 sharing a line
      // with a move actuate coolant (the per-line emit is the move), mirroring the spindle.
      sync_coolant_from_modal(modal, state);
    }
  }
  match parsed {
    // A blank/comment-only/modal-only line carries no action; acknowledge with a single `ok`.
    Ok(None) => ack().await,
    // `$C` check mode: the line parsed and validated cleanly, but check mode must NOT plan or execute it —
    // grbl `ok`s it so a host can verify a whole file without moving. The modal state still advanced in the
    // parser (correct: check mode tracks modal state), but no block is enqueued.
    Ok(Some(_)) if control == ControlState::Check => ack().await,
    Ok(Some(command)) => {
      // DOC-07 spin-up dwell: before the FIRST cutting move after an M3/M4, insert a synchronized `$392` dwell so
      // the spindle reaches speed before it cuts. The host-tested `SpinUpGate` decides WHEN; this injects the
      // dwell ahead of the move. `inject_spin_up_dwell` is a no-op (and returns Continue) for a rapid / non-move
      // command or when no spin-up is owed; it returns Aborted only if a soft reset preempted the awaited dwell.
      match inject_spin_up_dwell(&command, state).await {
        SpinUpInjection::Continue => {}
        // A soft reset preempted the awaited spin-up dwell: run the warm reset and drop the move.
        SpinUpInjection::Aborted => {
          apply_soft_reset(parser, state).await;
          return;
        }
        // A graceful program stop preempted the spin-up dwell's back-pressured enqueue: run the clean stop (Idle,
        // position retained, no alarm) and drop the move.
        SpinUpInjection::Stopped => {
          program_stop_cycle(parser, state).await;
          return;
        }
      }
      // G28/G30 (DOC-05 group-0 motion) is intercepted here, ahead of the generic `plan_command`: it must read the
      // stored predefined position from the consumer-owned coordinate model, so it cannot be planned by the planner
      // alone. `handle_go_to_predefined` runs the same lock + back-pressure + soft-limit flow and returns a
      // `PlanResult` the shared match below acts on (a single `ok`, or the soft-reset/soft-limit paths).
      let result = match &command {
        firmware_core::gcode::PlannerCommand::GoToPredefined { is_g28, intermediate, units, distance } => {
          handle_go_to_predefined(*is_g28, intermediate, *units, *distance).await
        }
        _ => plan_command(&command).await,
      };
      match result {
        // The command was accepted into the planner (a move enqueued, or a non-motion outcome passed
        // through); emit the single `ok`.
        PlanResult::Accepted => ack().await,
        // A coordinate-system / offset op: apply it to the shared coordinate model (against the live machine
        // position), push the new WCO into the planner, and persist the persistent subset, then `ok`.
        PlanResult::Coordinate(op) => {
          apply_coordinate_op(op).await;
          ack().await;
        }
        // The planner reported a non-back-pressure error (bad arc geometry); reject and arm the hold.
        PlanResult::Error(code) => {
          error(code).await;
          state.error_hold = true;
        }
        // A program move left the `$20` soft-limit envelope: enter the soft-limit alarm and emit `ALARM:2` (no
        // `ok`). The block was rejected before any motion, so position is intact, but grbl halts the program; a
        // soft reset / `$X` clears the alarm. The alarm latches so subsequent GCode is gated until cleared.
        PlanResult::SoftLimitAlarm => {
          set_control_state(ControlState::Alarm(AlarmCode::SoftLimit));
          emit_alarm(AlarmCode::SoftLimit).await;
        }
        // A soft reset arrived while this command was back-pressured: abort it (the host discards pending
        // acks on `0x18`), emit no response, and run the soft-reset transition whose signal was consumed here.
        PlanResult::Aborted => apply_soft_reset(parser, state).await,
        // A graceful program stop (`0x86`) arrived while this command was back-pressured: abandon the line (no
        // `ok` — the host discards pending acks the moment it sends the stop) and run the clean stop whose
        // [`PROGRAM_STOP`] signal was consumed in the back-pressure wait. Returns to Idle with position retained,
        // no alarm — unlike `Aborted`'s warm reset. The `program_stop_cycle` re-checks the live state, so a stop
        // that raced a soft reset (which already moved us out of Run/Hold) is a benign no-op.
        PlanResult::Stopped => program_stop_cycle(parser, state).await,
        // An M3/M4/M5 (DOC-07): the spindle outputs were ALREADY driven from the modal spindle state by
        // `sync_spindle_from_modal` (above, on the clean parse), so a spindle-only line just `ok`s here. Driving
        // off modal state — not this per-line outcome — is what makes an M3/M4/M5 sharing a line with a move work.
        PlanResult::Spindle(_spindle_state, _rpm) => ack().await,
        // A `G4` dwell: run the synchronized dwell (drain motion, then hold), then `ok`. A soft reset mid-dwell
        // abandons it and runs the reset (the consumed signal must be honored), emitting no `ok`.
        PlanResult::Dwell(seconds) => {
          if run_dwell(seconds).await {
            ack().await;
          } else {
            apply_soft_reset(parser, state).await;
          }
        }
        // An `M30` program end: drain motion, stop the spindle, reset modal state, then `ok`. A soft reset while
        // draining abandons the end and runs the reset instead.
        PlanResult::ProgramEnd => {
          if program_end(parser, state).await {
            ack().await;
          } else {
            apply_soft_reset(parser, state).await;
          }
        }
        // An M0/M1/M6 program-flow pause: drain motion and hold until cycle-start (`~`), then `ok`. The `ok`
        // follows normal char-counting — it is emitted when the pause COMPLETES (after the resume), exactly like
        // a dwell, so the host's send-ahead window naturally stalls while paused. `run_program_pause` returns
        // `Resumed` on cycle-start, or a preemption (soft reset / graceful stop) the caller honors with no `ok`.
        PlanResult::ProgramPause { optional, tool_change } => {
          // The committed CURRENT tool (M6 commits pending->current before this point) names the tool-change
          // prompt so a bare-terminal operator knows which tool to insert.
          let current_tool = parser.state().current_tool;
          match run_program_pause(optional, tool_change, current_tool, parser, state, flash).await {
            PauseOutcome::Resumed => ack().await,
            PauseOutcome::Skipped => ack().await,
            PauseOutcome::Aborted => apply_soft_reset(parser, state).await,
            PauseOutcome::Stopped => program_stop_cycle(parser, state).await,
          }
        }
        // An M7/M8/M9 coolant command: the coolant outputs were ALREADY driven from the modal coolant state by
        // `sync_coolant_from_modal` (on the clean parse), so a coolant-only line just `ok`s here — mirroring how
        // a spindle-only line is handled. Driving off modal state makes an M7/M8 sharing a line with a move work.
        PlanResult::Coolant(_state) => ack().await,
        // A `G38.x` probe: run the probe-watching cycle on the core-1 executor and decide the response from the
        // outcome and the mode's alarm-on-fail flag.
        PlanResult::Probe { request, alarm_on_fail } => {
          handle_probe(request, alarm_on_fail, parser, state).await;
        }
      }
    }
    Err(e) => {
      // A parse error: emit `error:N` and arm the gcode error-hold so subsequent GCode lines are held.
      error(e.code()).await;
      state.error_hold = true;
    }
  }
}

/// The result of attempting to plan one command (collapsing the planner's back-pressure retry loop and the
/// soft-reset abort into one outcome the line handler acts on).
enum PlanResult {
  /// The command was accepted into the planner buffer (move enqueued or non-motion outcome passed through).
  Accepted,
  /// A coordinate-system / offset op passed through the planner; the consumer applies it to the shared
  /// [`COORDINATES`] model (resolving any "set to current position" op against the live machine position),
  /// pushes the recomputed WCO into the planner, and persists the persistent subset. Carried out of
  /// `plan_command` so the apply happens with the live machine position in hand.
  Coordinate(CoordinateOp),
  /// A non-back-pressure planner error; carries the grblHAL `error:N` code (bad arc geometry).
  Error(u8),
  /// A program move/arc exceeded the `$20` soft-limit envelope (DOC-06). grbl halts and raises `ALARM:2`
  /// (position is NOT lost — the move was rejected before any motion — but the program cannot continue). The
  /// consumer enters the soft-limit alarm and emits `ALARM:2`, no `ok`.
  SoftLimitAlarm,
  /// A soft reset preempted the command while it was back-pressured; the consumed signal must be honored.
  Aborted,
  /// A graceful program stop (`0x86`) preempted the command while it was back-pressured (or driving a pending
  /// arc): the consumed [`PROGRAM_STOP`] signal must be honored by running [`program_stop_cycle`]. Distinct from
  /// [`Aborted`](PlanResult::Aborted) — a stop returns to Idle with position retained and raises no alarm, where
  /// the `0x18` abort runs the warm reset. The in-flight line is abandoned with no `ok` (the host discards pending
  /// acks the moment it sends the stop).
  Stopped,
  /// An M3/M4/M5 spindle command (DOC-07): the planner passed it through with no motion. The consumer publishes
  /// the commanded direction, wakes the [`spindle`] task to drive the outputs, and notes the spin-up gate so the
  /// next cutting move gets a `$392` dwell. Carried out of `plan_command` so the side effects run in the
  /// consumer task (which owns the spin-up gate state and the direction/wake signals).
  Spindle(SpindleState, f32),
  /// A `G4` dwell (DOC). The planner has flushed look-ahead (the preceding block stops); the consumer runs the
  /// synchronized dwell ([`run_dwell`]) — wait for motion to drain, then hold for the dwell seconds — so the dwell
  /// blocks the stream like grbl's buffer-synchronize. Carried out of `plan_command` so the timed wait runs in the
  /// consumer task (which owns the stream).
  Dwell(f32),
  /// An `M30` program end. The planner has flushed look-ahead; the consumer drains motion, stops the spindle, and
  /// resets the parser's modal state to power-on defaults (grbl's M30 reset). Carried out of `plan_command` so the
  /// drain wait + spindle/modal reset run in the consumer task.
  ProgramEnd,
  /// An M0/M1/M6 program-flow pause. The planner flushed look-ahead; the consumer drains motion and holds until
  /// cycle-start (`~`), reusing the graceful-hold machinery. `optional` flags M1 (the optional-stop gate decides
  /// whether it actually halts); `tool_change` flags M6 (the consumer prompts the operator to swap the tool).
  /// Carried out of `plan_command` so the drain + hold-await run in the consumer task that owns the stream.
  ProgramPause {
    /// True for M1 (optional stop): only halts when the optional-stop toggle ([`OPTIONAL_STOP_ENABLED`]) is on.
    optional: bool,
    /// True for M6 (manual tool change): the consumer emits a `[MSG:..]` swap prompt before holding.
    tool_change: bool,
  },
  /// An M7/M8/M9 coolant command. The planner passed it through with no motion; the consumer drives the
  /// (hardware-gated) coolant outputs from the carried [`CoolantState`]. Mirrors [`Spindle`](PlanResult::Spindle).
  Coolant(firmware_core::gcode::CoolantState),
  /// A `G38.x` probe (Phase C): the planner resolved the machine target and flushed look-ahead. The consumer
  /// runs the probe-watching cycle on the core-1 executor (carrying the resolved request + the alarming sense),
  /// syncs the planner position to the stop point, emits `[PRB:]`, and decides ALARM/ok.
  Probe {
    /// The request handed to the core-1 executor (target / period / toward sense).
    request: ProbeRequest,
    /// `true` for the alarming modes (G38.2/.4): a no-trigger outcome raises ALARM:4/5. `false` for G38.3/.5.
    alarm_on_fail: bool,
  },
}

/// The `error:N` code used to reject a GCode line that is held in the post-error state. Matches the
/// engine's `ERROR_HOLD_CODE` (grbl's generic "expected command letter" code 1): a sender already in
/// error-recovery halts the stream regardless of the specific code.
const ERROR_HOLD_CODE: u8 = 1;

/// The grblHAL `error:N` code for GCode rejected because the machine is in an alarm, jog, or sleep state —
/// grbl's "G-code locked out during alarm or jog state" (Finding #5). Distinct from the in-stream
/// post-error hold (`ERROR_HOLD_CODE` = 1): this is a STATE lockout, and code 9 already has a row in the
/// `$EE` [`ERROR_CODES`](firmware_core::protocol::ERROR_CODES) table, so a sender's display matches.
const ERROR_LOCKED: u8 = 9;

/// The `error:N` code surfaced when the shared planner was never installed (an init wiring bug). grblHAL's
/// "setting disabled" code 3 is reused as a distinct, loud failure so a misconfigured build fails the line
/// instead of fabricating an `ok` for motion that will never run. Unreachable in a correctly wired build.
const ERROR_PLANNER_UNINITIALIZED: u8 = 3;

/// Feed one [`PlannerCommand`](firmware_core::gcode::PlannerCommand) to the shared planner, applying
/// back-pressure: on [`PlannerError::QueueFull`] wait for the core-1 motion executor to free a block and
/// retry the SAME command (the arc planner is all-or-nothing on `QueueFull`, so re-issue is safe). On an
/// accepted motion outcome it raises [`BLOCK_AVAILABLE`] to wake the executor. The back-pressure wait is
/// raced against [`SOFT_RESET`] so a `0x18` aborts a stuck line promptly rather than after the executor frees
/// a slot. Non-motion outcomes are surfaced as distinct [`PlanResult`]s for the consumer to act on: `Spindle`
/// (DOC-07), `Dwell` (the synchronized `G4`), `ProgramEnd` (`M30`), and `Coordinate` (G10/G54-G59/G92). Still
/// passing through as `Accepted` (no side effect): the G28/G30 predefined move (real system motion, a DOC-06
/// follow-up) and a zero-block no-op `Queued`.
async fn plan_command(command: &firmware_core::gcode::PlannerCommand) -> PlanResult {
  loop {
    // Scope the lock so it is released before any await: hold the planner mutex only for the plan call. A
    // missing planner (an init wiring bug, unreachable in a correctly wired build — see `init_planner`) is
    // surfaced as a distinct internal `error:N` rather than a fabricated `ok`: a silent accepted-but-un-run
    // move would hide the bug, so we fail the line loudly instead.
    // `$20` soft limits are checked at PLAN time, but only when enabled AND the machine is homed (a known
    // machine zero is what makes the envelope meaningful — research finding #16). `current_soft_limits` returns
    // the envelope only when both hold; otherwise `None` skips the check (identical to the pre-DOC-06 behavior).
    let limits = current_soft_limits().await;
    let result = {
      let mut guard = PLANNER.lock().await;
      match guard.as_mut() {
        Some(planner) => planner.plan_command_with_limits(command, limits),
        None => return PlanResult::Error(ERROR_PLANNER_UNINITIALIZED),
      }
    };
    match result {
      // A motion command enqueued at least one block: wake the core-1 `motion_executor` so it can drain the
      // freshly queued block(s) instead of polling. A coalesced signal is fine — the executor re-checks the
      // queue under the lock and loops until empty, so multiple blocks behind one signal are all consumed.
      Ok(PlannerOutcome::Queued { blocks }) if blocks > 0 => {
        BLOCK_AVAILABLE.signal(());
        return PlanResult::Accepted;
      }
      // An over-subdivided arc fed only PART of its segments — the rest are saved as an in-progress arc (DOC-05
      // resumable arc). The line must NOT be acked yet: wake the executor to drain the chunk we just queued, then
      // drive `resume_arc` until the whole arc is enqueued. This is what lets an arc with more than the queue's
      // worth of segments stream without ever dead-locking on a permanent `QueueFull`. A soft reset mid-arc aborts
      // it (the executor's reset clears the queue and `abort_arc` drops the in-progress arc).
      Ok(PlannerOutcome::ArcPending { enqueued }) => {
        if enqueued > 0 {
          BLOCK_AVAILABLE.signal(());
        }
        return drive_pending_arc().await;
      }
      // A coordinate-system / offset op: surface it so the consumer applies it to the shared coordinate model
      // with the live machine position in hand, then pushes the recomputed WCO back into the planner.
      Ok(PlannerOutcome::Coordinate(op)) => return PlanResult::Coordinate(op),
      // A `G38.x` probe (Phase C): the planner resolved the machine target and flushed look-ahead. Derive the
      // fixed probe-feed step period from the seek feed + steps/mm, and hand the request back to the consumer to
      // run on the core-1 executor (which watches the probe and stops on the edge).
      Ok(PlannerOutcome::Probe { kind, target, feed, units }) => {
        let step_period_ticks = probe_step_period_ticks(&target, feed, units).await;
        let invert = settings_snapshot().await.probe_config().invert;
        return PlanResult::Probe {
          request: ProbeRequest { target, step_period_ticks, toward: kind.toward, invert },
          alarm_on_fail: kind.alarm_on_fail,
        };
      }
      // An M3/M4/M5 spindle command (DOC-07): the planner passes it through with no motion. Surface it so the
      // consumer drives the spindle task and notes the spin-up gate (it owns that state); doing the side effects
      // here in `plan_command` would scatter them away from the gate/error-hold owner.
      Ok(PlannerOutcome::Spindle(state, rpm)) => return PlanResult::Spindle(state, rpm),
      // A `G4` dwell: the planner flushed look-ahead (pinning the preceding block to a stop — the synchronized
      // boundary); the consumer runs the timed [`run_dwell`] wait. Surfaced so the wait runs in the consumer task.
      Ok(PlannerOutcome::Dwell { seconds }) => return PlanResult::Dwell(seconds),
      // `M30` program end: the planner flushed look-ahead; the consumer drains motion, stops the spindle, and
      // resets modal state. Surfaced so those side effects run in the consumer task.
      Ok(PlannerOutcome::ProgramEnd) => return PlanResult::ProgramEnd,
      // M0/M1/M6 program-flow pause: the planner flushed look-ahead (the pause is a synchronized boundary, like a
      // dwell). Surface it so the consumer drains motion and runs the hold-until-cycle-start cycle in its own task.
      Ok(PlannerOutcome::ProgramPause { optional, tool_change }) => {
        return PlanResult::ProgramPause { optional, tool_change };
      }
      // An M7/M8/M9 coolant command: the planner passes it through with no motion. Surface it so the consumer drives
      // the coolant task off the modal state, mirroring how the spindle outcome is handled.
      Ok(PlannerOutcome::Coolant(state)) => return PlanResult::Coolant(state),
      // G28/G30 is NOT planned through this generic path — the consumer intercepts it before `plan_command`
      // (see `handle_go_to_predefined`) because it must read the stored predefined position from the coordinate
      // model, which lives in the consumer, not the planner. The pass-through `GoToPredefined` outcome is therefore
      // unreachable here; a zero-block `Queued { blocks: 0 }` no-op move still falls here and needs no executor wake.
      Ok(_outcome) => return PlanResult::Accepted,
      // Back-pressure: the planner buffer is full. Do NOT ack and do NOT drop — yield to the motion
      // executor, then retry the same command. Blocking here backs `LINE_QUEUE` up and throttles the host (correct
      // grbl flow control). Race the retry delay against a soft reset so `0x18` aborts a stuck line at once;
      // the delay is short relative to a block's execution time, so a normal retry wins the freed slot
      // promptly without busy-spinning the CPU.
      Err(PlannerError::QueueFull) => {
        // Race the retry delay against a soft reset (`0x18` → abort) AND a graceful program stop (`0x86` → clean
        // stop): both abandon a stuck back-pressured line at once rather than after the executor frees a slot.
        match select(Timer::after(QUEUE_FULL_RETRY), select(SOFT_RESET.wait(), PROGRAM_STOP.wait())).await {
          Either::First(()) => {}
          Either::Second(Either::First(())) => return PlanResult::Aborted,
          Either::Second(Either::Second(())) => return PlanResult::Stopped,
        }
      }
      // A program move/arc that left the `$20` soft-limit envelope: this is a SYSTEM ALARM in grbl (`ALARM:2`),
      // not an `error:N` line — the planner rejected the block before any enqueue, so no motion started. Surface
      // it as a distinct result the caller routes to the alarm path.
      Err(PlannerError::MoveExceedsTravel) => return PlanResult::SoftLimitAlarm,
      // A genuine geometry error (bad arc): surface the grblHAL code to the caller.
      Err(other) => return PlanResult::Error(other.code()),
    }
  }
}

/// Drive an in-progress (over-subdivided) arc to completion, feeding its remaining segments into the planner as
/// the core-1 motion executor frees queue slots (DOC-05 resumable arc). The first chunk has ALREADY been enqueued
/// by [`plan_command`]; this loops [`Planner::resume_arc`](firmware_core::planner::Planner::resume_arc) — waking
/// the executor after each chunk and yielding for it to drain — until the whole arc is enqueued, at which point the
/// line is acked exactly ONCE ([`PlanResult::Accepted`]). It NEVER acks while the arc is still pending, so the
/// host's character-counting flow control throttles correctly and the stream can never dead-lock on an arc larger
/// than the queue. The resume wait is raced against [`SOFT_RESET`] so a `0x18` aborts a stuck arc at once — the
/// executor's reset clears the queue and the planner rebuild drops the in-progress arc, so the abort is clean.
///
/// ## Proactive (event-driven) refill — Bug 4
/// The refill is woken by [`SLOT_FREED`] (raised the instant the executor pops a block) raced against a short
/// [`QUEUE_FULL_RETRY`] timer backstop and [`SOFT_RESET`]. Refilling the moment a slot opens — rather than only
/// after the full poll interval — keeps the planner buffer topped up while the arc is pending, so the executor
/// never drains down to the look-ahead's forced-stop chunk tail before the next chunk lands. That is what makes a
/// large arc execute as CONTINUOUS motion across chunk boundaries (proven host-side in
/// `over_subdivided_arc_carries_velocity_across_chunk_boundaries`) instead of stamping a decelerate-to-stop dwell
/// mark at each ~`BLOCK_QUEUE_LEN` boundary. RESIDUAL: the genuine last available block always decelerates to a
/// stop (the executor must be able to halt there — a hard safety invariant); at an extreme feed where the
/// executor could empty the queue between a pop and the refill completing, motion would still momentarily stop —
/// safe, never a step loss — but at realistic PCB-milling feeds/segment timing the producer stays ahead and the
/// curve is smooth.
///
/// ## Termination — Bug 9
/// The loop exits ONLY on `Queued` (the arc completed), `SOFT_RESET`, or an `Err` from `resume_arc`. A genuine
/// (non-`QueueFull`) per-segment error now PROPAGATES out of `resume_arc` with the in-progress arc cleared, so the
/// `Err(other)` arm returns `error:N` and the loop ends — a deterministic segment error can no longer spin here
/// forever. A `resume_arc` that enqueues zero (the queue is still full) is benign: it simply waits for the next
/// `SLOT_FREED`/timer wake, and the loop PROGRESSES because each executor pop frees a slot the next resume claims.
async fn drive_pending_arc() -> PlanResult {
  loop {
    // Refill on the executor's "slot freed" wake the instant it pops a block (proactive refill, Bug 4), with a
    // short timer backstop so a foregone signal (e.g. the executor parked on a hold) cannot wedge the loop, and a
    // soft reset so `0x18` aborts at once. The timer is short relative to a block's execution time, so even on the
    // backstop path a freed slot is claimed promptly.
    // Also race a graceful program stop (`0x86`): a stop arriving mid-arc-drive abandons the remaining segments at
    // once (the stop's `abort_arc` + `flush_queue` drops the in-progress arc and the queued chunks) and runs the
    // clean stop, exactly as `0x18` runs the abort. Reported as `Stopped` so the caller runs `program_stop_cycle`.
    match select(select(SLOT_FREED.wait(), Timer::after(QUEUE_FULL_RETRY)), select(SOFT_RESET.wait(), PROGRAM_STOP.wait())).await {
      Either::First(_) => {}
      Either::Second(Either::First(())) => return PlanResult::Aborted,
      Either::Second(Either::Second(())) => return PlanResult::Stopped,
    }
    // Feed the next chunk under the planner lock (scoped so it is dropped before any await). A missing planner is
    // a wiring bug surfaced loudly rather than fabricating an `ok`, exactly as `plan_command` does.
    let outcome = {
      let mut guard = PLANNER.lock().await;
      match guard.as_mut() {
        Some(planner) => planner.resume_arc(),
        None => return PlanResult::Error(ERROR_PLANNER_UNINITIALIZED),
      }
    };
    match outcome {
      // The final chunk is in: every segment is enqueued, so wake the executor for the last blocks and ack once.
      Ok(PlannerOutcome::Queued { blocks }) => {
        if blocks > 0 {
          BLOCK_AVAILABLE.signal(());
        }
        return PlanResult::Accepted;
      }
      // More segments fed (or none yet, if no slot freed): wake the executor for whatever we just queued and loop.
      Ok(PlannerOutcome::ArcPending { enqueued }) => {
        if enqueued > 0 {
          BLOCK_AVAILABLE.signal(());
        }
      }
      // `resume_arc` only ever returns an arc outcome on success; any other Ok variant is an invariant break,
      // surfaced loudly rather than silently acking a half-fed arc.
      Ok(_) => return PlanResult::Error(ERROR_PLANNER_UNINITIALIZED),
      // A genuine per-segment planner error (Bug 9): `resume_arc` has already cleared the in-progress arc, so we
      // surface `error:N` and STOP driving — the deterministic error can never spin this loop forever.
      Err(other) => return PlanResult::Error(other.code()),
    }
  }
}

/// Plan a `G28`/`G30` predefined-position recall (DOC-05 group-0 motion) into the planner. Unlike the generic
/// [`plan_command`] path this reads the stored predefined position from the consumer-owned coordinate model (index
/// 0 = G28, 1 = G30; a never-stored slot defaults to the machine origin, grbl's default) and hands it to the
/// host-tested [`Planner::plan_go_to_predefined`], which sequences the optional work-coordinate intermediate rapid
/// and the absolute machine-coordinate recall rapid. It mirrors `plan_command`'s flow exactly: the planner mutex is
/// scoped so it is dropped before any await, the `$20` soft-limit envelope is supplied (so an out-of-envelope
/// intermediate alarms like any rapid), `QueueFull` back-pressure yields to the executor and retries the WHOLE
/// call, and a `0x18` soft reset mid-retry aborts the line. Retrying the whole call is safe because
/// `plan_go_to_predefined` is ATOMIC: it enqueues NEITHER sub-move unless BOTH fit, so a retry always re-resolves
/// from the original, un-advanced position — critical in INCREMENTAL (G91) mode, where a partial enqueue would
/// otherwise re-apply the intermediate's increment a second time (double motion).
async fn handle_go_to_predefined(is_g28: bool, intermediate: &firmware_core::gcode::AxisWords, units: GcodeUnits,
  distance: GcodeDistance) -> PlanResult {
  // Index 0 is the G28 home, 1 is the G30 secondary; a never-stored slot reads as the machine origin (grbl default).
  let predefined = coordinates().predefined(if is_g28 { 0 } else { 1 }).unwrap_or([0.0; AXES]);
  loop {
    let limits = current_soft_limits().await;
    let result = {
      let mut guard = PLANNER.lock().await;
      match guard.as_mut() {
        Some(planner) => planner.plan_go_to_predefined(intermediate, units, distance, predefined, limits),
        None => return PlanResult::Error(ERROR_PLANNER_UNINITIALIZED),
      }
    };
    match result {
      // At least one rapid enqueued: wake the core-1 executor to drain it. A zero-block no-op (already at the
      // stored position with no intermediate words) needs no wake — fall through to `Accepted` either way.
      Ok(blocks) => {
        if blocks > 0 {
          BLOCK_AVAILABLE.signal(());
        }
        return PlanResult::Accepted;
      }
      // Back-pressure: yield to the executor and retry the whole call. The call is atomic (enqueues nothing unless
      // both sub-moves fit), so the retry re-resolves from the un-advanced position — see the doc comment. Race the
      // retry against a soft reset (`0x18` → abort) AND a graceful program stop (`0x86` → clean stop) so either
      // abandons a stuck recall at once.
      Err(PlannerError::QueueFull) => {
        match select(Timer::after(QUEUE_FULL_RETRY), select(SOFT_RESET.wait(), PROGRAM_STOP.wait())).await {
          Either::First(()) => {}
          Either::Second(Either::First(())) => return PlanResult::Aborted,
          Either::Second(Either::Second(())) => return PlanResult::Stopped,
        }
      }
      // The intermediate left the `$20` envelope: a SYSTEM ALARM (`ALARM:2`), not an `error:N` line — no motion
      // started (the planner rejected before any enqueue). The recall point is within-envelope by construction.
      Err(PlannerError::MoveExceedsTravel) => return PlanResult::SoftLimitAlarm,
      // No other error is reachable from `plan_go_to_predefined` (it plans only rapids, no arc geometry); surface
      // any future one as its grblHAL code rather than fabricating an `ok`.
      Err(other) => return PlanResult::Error(other.code()),
    }
  }
}

/// Drive the [`spindle`] task from the parser's MODAL spindle state (DOC-07), called after every clean parse. This
/// is the single point the firmware acts on M3/M4/M5 + `S`: keying off modal state (not the per-line emit) is what
/// makes a spindle word that SHARES a line with a move still start/change the spindle (the emit is the move), and
/// lets a bare `S` re-drive a running spindle. Acts only on a real change vs the last dispatched `(dir, rpm)`:
/// - a DIRECTION change publishes the new direction, (dis)arms the spin-up gate, and wakes the task;
/// - a pure RPM change while RUNNING wakes the task to re-drive the duty WITHOUT re-arming the spin-up (a speed
///   change is not a fresh spindle start, so it owes no spin-up dwell).
/// Synchronous (atomics + a coalesced `Signal`); `PROGRAMMED_SPINDLE_RPM` must already be stored for this `S`.
fn sync_spindle_from_modal(modal: &ModalState, state: &mut ConsumerState) {
  let dir = modal.spindle;
  let rpm = modal.spindle_speed.max(0.0).min(u16::MAX as f32) as u16;
  if dir != state.last_spindle_dir {
    // A direction change (start, reversal, or stop): dispatch arms/disarms the spin-up gate and wakes the task.
    dispatch_spindle(dir, state);
  } else if !matches!(dir, SpindleState::Stop) && rpm != state.last_spindle_rpm {
    // Same direction, new speed on a RUNNING spindle: wake the task to re-drive the duty from the new programmed
    // RPM (already stored). No gate re-arm — this is a speed change, not a spindle start.
    SPINDLE_UPDATE.signal(());
  }
  state.last_spindle_dir = dir;
  state.last_spindle_rpm = rpm;
}

/// Publish an M3/M4/M5 direction to the [`spindle`] task (DOC-07): record it in [`SPINDLE_DIRECTION`], note the
/// spin-up gate (so the next cutting move gets a `$392` dwell), and wake the task. Synchronous — atomics + a
/// coalesced `Signal` — so it adds no await to the line handler. The task reads the override-scaled RPM, so a
/// spindle-override / spindle-stop change later re-drives the duty without re-issuing the M-word. Called by
/// [`sync_spindle_from_modal`] on a modal direction change (the single dispatch path).
fn dispatch_spindle(spindle_state: SpindleState, state: &mut ConsumerState) {
  let dir = match spindle_state {
    SpindleState::Clockwise => SPINDLE_DIR_CW,
    SpindleState::CounterClockwise => SPINDLE_DIR_CCW,
    SpindleState::Stop => SPINDLE_DIR_STOP,
  };
  SPINDLE_DIRECTION.store(dir, Ordering::Release);
  // Arm / disarm the spin-up gate: an M3/M4 owes a dwell to the next cutting move; an M5 clears it.
  state.spin_up.note_spindle(spindle_state);
  // Wake the spindle task to drive the outputs from the new direction + the current scaled RPM.
  SPINDLE_UPDATE.signal(());
}

/// The outcome of [`inject_spin_up_dwell`]: whether the line handler should keep planning the move or abort it
/// because a soft reset preempted the awaited spin-up dwell.
enum SpinUpInjection {
  /// No dwell was owed (or one was inserted and completed); continue planning the move.
  Continue,
  /// A soft reset arrived while awaiting the spin-up dwell; the caller must run the soft-reset transition and
  /// drop the move (the host discards pending acks on `0x18`).
  Aborted,
  /// A graceful program stop (`0x86`) arrived while the spin-up dwell's enqueue was back-pressured; the caller must
  /// run [`program_stop_cycle`] and drop the move. Distinct from [`Aborted`](SpinUpInjection::Aborted) — a stop
  /// returns to Idle with position retained and no alarm, where the `0x18` abort runs the warm reset.
  Stopped,
}

/// DOC-07 spin-up dwell injection. Before the FIRST cutting move (a G1 feed `Move` or any `Arc`) after an M3/M4,
/// insert a synchronized `$392` dwell so the spindle reaches speed before it cuts. The host-tested [`SpinUpGate`]
/// (on [`ConsumerState`]) owns the WHEN decision; this supplies the dwell seconds from the live `$392` and runs
/// the timed wait. It is a no-op (returns [`SpinUpInjection::Continue`]) for a rapid (G0), a non-move command, or
/// when no spin-up is owed.
///
/// The dwell is realized two ways, matching grbl's "a dwell is a synchronized motion boundary": a
/// [`PlannerCommand::Dwell`] is planned first (flushing look-ahead so any preceding block stops at the boundary),
/// then the real timed wait is awaited here — raced against [`SOFT_RESET`] so a `0x18` mid-dwell aborts promptly.
async fn inject_spin_up_dwell(
  command: &firmware_core::gcode::PlannerCommand,
  state: &mut ConsumerState,
) -> SpinUpInjection {
  use firmware_core::gcode::PlannerCommand;
  // Only a CUTTING move consumes the spin-up: a G1 feed move or any arc. A G0 rapid is a positioning move, not a
  // cut, so it does not consume the spin-up (the dwell waits for the first real cut). Any non-move command (dwell,
  // coordinate op, spindle, probe, …) is not a cut either.
  let is_cutting_move = matches!(command, PlannerCommand::Move { rapid: false, .. } | PlannerCommand::Arc { .. });
  if !is_cutting_move {
    return SpinUpInjection::Continue;
  }
  // Cheap guard BEFORE the settings snapshot: only the first cutting move after a spindle start owes a dwell, so a
  // dense toolpath's every-G1 common case skips the full-`Settings` snapshot entirely (the gate is a single bool).
  if !state.spin_up.is_pending() {
    return SpinUpInjection::Continue;
  }
  // A spin-up IS owed: read the live `$392` and consume the gate. `take_dwell_before_move` returns the seconds to
  // dwell, or `None` only when `$392 == 0` (the gate is consumed either way, so the next move gets no dwell).
  let spin_up_s = settings_snapshot().await.spindle_on_delay_s;
  let Some(dwell_s) = state.spin_up.take_dwell_before_move(spin_up_s) else {
    return SpinUpInjection::Continue;
  };
  // Flush the planner's look-ahead at the boundary so the cut starts from rest: plan a synchronized G4 dwell (the
  // planner pins the preceding block to a stop). `plan_command` returns `PlanResult::Dwell` for it, or `Aborted`
  // if a soft reset preempted a back-pressured enqueue.
  match plan_command(&PlannerCommand::Dwell { seconds: dwell_s }).await {
    PlanResult::Aborted => return SpinUpInjection::Aborted,
    PlanResult::Stopped => return SpinUpInjection::Stopped,
    _ => {}
  }
  // Run the SAME synchronized dwell a real `G4` uses (wait for prior motion to drain, then hold `$392` so the
  // spindle reaches speed), raced against a soft reset — one dwell mechanism, no ad-hoc timer.
  if run_dwell(dwell_s).await {
    SpinUpInjection::Continue
  } else {
    SpinUpInjection::Aborted
  }
}

/// The absolute ceiling on any dwell, in seconds — a sanity bound shared by a real `G4`, the `$392` spin-up, and
/// the `$393` reverse delay. A guard against a grossly mis-set / corrupt value: `(secs * 1e6) as u64` SATURATES
/// for an enormous `secs`, which would park the stream on a multi-century `Timer`. One hour is far longer than any
/// real PCB-milling dwell yet safely below the `u64`-microsecond saturation point, so a typo can never wedge the
/// machine while a legitimate seconds-to-minutes `G4` is unaffected.
const MAX_DWELL_S: f32 = 3_600.0;

/// Convert a dwell in seconds to an [`embassy_time::Duration`], clamped to `[0, MAX_DWELL_S]` so a negative / NaN /
/// absurdly large value can neither underflow nor saturate the microsecond `Timer`. Shared by the `G4` dwell
/// ([`run_dwell`]) and the reverse-dwell ([`spindle`]) waits so all dwell paths honor the same bound.
fn dwell_duration(secs: f32) -> Duration {
  let clamped = if secs.is_finite() { secs.clamp(0.0, MAX_DWELL_S) } else { 0.0 };
  Duration::from_micros((clamped * 1_000_000.0) as u64)
}

/// The poll interval while waiting for the core-1 executor to drain to a synchronized boundary ([`run_dwell`]).
/// Short relative to a block's execution time so the dwell starts promptly after motion stops, but long enough
/// that the brief `PLANNER`-lock checks add negligible load while the machine winds down.
const MOTION_IDLE_POLL: Duration = Duration::from_millis(4);

/// Wait until the core-1 executor has fully drained the planner queue and stopped — grbl's buffer-synchronize, the
/// rest point a `G4` dwell (and the spin-up dwell) needs before timing begins. The consumer is the sole enqueuer
/// and is blocked here, so once [`program_running`] reads false no new motion can appear; the wait converges. Polls
/// on a short [`MOTION_IDLE_POLL`] ticker, racing [`SOFT_RESET`] so a `0x18` abandons the wait. Returns `true` once
/// idle, or `false` if a soft reset preempted it (the signal is consumed; the caller runs the reset, matching
/// [`inject_spin_up_dwell`]'s abort contract).
async fn wait_for_motion_idle() -> bool {
  while program_running().await {
    match select(Timer::after(MOTION_IDLE_POLL), SOFT_RESET.wait()).await {
      Either::First(()) => {}
      Either::Second(()) => return false,
    }
  }
  true
}

/// Run a synchronized dwell (DOC, grbl `G4`): wait for all prior motion to drain to a stop, then hold for `seconds`.
/// This is the ONE timed-dwell mechanism — a real `G4` and the `$392` spin-up both route through it, so the
/// "dwell = synchronized motion boundary" guarantee is enforced in one place rather than approximated by an
/// ad-hoc timer. Returns `true` when the dwell completed, or `false` if a soft reset preempted either phase (the
/// signal is consumed; the caller runs the reset).
async fn run_dwell(seconds: f32) -> bool {
  if !wait_for_motion_idle().await {
    return false;
  }
  match select(Timer::after(dwell_duration(seconds)), SOFT_RESET.wait()).await {
    Either::First(()) => true,
    Either::Second(()) => false,
  }
}

/// Run an `M30` program end (grbl): drain pending motion to a stop, stop the spindle, and reset the parser's modal
/// state to power-on defaults (G0/G90/G21/G54, F0/S0) so the next program starts clean — WITHOUT flushing the
/// planner or zeroing machine position (M30 is a program rewind, not a soft reset). Returns `true` on completion,
/// or `false` if a soft reset preempted the motion drain (the consumed signal is honored by the caller).
async fn program_end(parser: &mut Parser, state: &mut ConsumerState) -> bool {
  if !wait_for_motion_idle().await {
    return false;
  }
  // M30 turns the spindle off: park it (SPIN_EN off + duty 0) and clear the commanded direction + programmed RPM.
  // grbl's M30 also turns COOLANT off (group 8 → M9); force both off and clear the last-dispatched mask.
  force_spindle_off();
  force_coolant_off();
  state.last_coolant = 0;
  PROGRAMMED_SPINDLE_RPM.store(0, Ordering::Release);
  // Reset modal state to defaults and the spindle tracking to match, so the next line's `sync_spindle_from_modal`
  // sees a fresh `Stop`/`S0` baseline rather than the just-ended program's direction. `Parser::new()` resets the
  // modal WCS to G54 (index 0). The CURRENT tool is RETAINED across M30 (grbl: program end is a rewind, not a tool
  // change — the selected tool survives), so snapshot it and restore it after the rebuild.
  let retained_tool = parser.state().current_tool;
  *parser = Parser::new();
  parser.set_current_tool(retained_tool);
  state.spin_up = SpinUpGate::new();
  state.last_spindle_dir = SpindleState::Stop;
  state.last_spindle_rpm = 0;
  // grbl M30 also: selects G54, turns coolant OFF, and resets feed/rapid/spindle overrides to 100%. The parser
  // reset above only restored the parser-MODAL WCS — push that G54 selection into the coordinate model + planner
  // too (otherwise the planner keeps the ended program's G55-G59 offset and the next move cuts at the wrong WPos),
  // and reset the live overrides + coolant toggles (which live outside the parser, in `OVERRIDES`), matching the
  // soft-reset reset of the same cross-task state.
  sync_active_wcs(0).await;
  set_overrides(Overrides::new());
  reset_ov_reporter();
  true
}

/// The outcome of an M0/M1/M6 program-flow pause ([`run_program_pause`]).
enum PauseOutcome {
  /// The pause completed: motion drained, the machine held, and a cycle-start (`~`) resumed it. The caller `ok`s.
  Resumed,
  /// The pause did NOT halt: an `M1` whose optional-stop gate is off. No hold occurred; the caller still `ok`s the
  /// line so the stream continues (an `M1` is always a valid line, it simply does nothing when the switch is off).
  Skipped,
  /// A soft reset (`0x18`) preempted the drain or the hold-await; the caller runs the warm reset and drops the `ok`.
  Aborted,
  /// A graceful program stop (`0x86`) preempted the drain or the hold-await; the caller runs the clean stop and
  /// drops the `ok`. Distinct from [`Aborted`](PauseOutcome::Aborted) — a stop returns to Idle with position
  /// retained and no alarm.
  Stopped,
}

/// Run an M0/M1/M6 program-flow pause (grbl program-flow). The line's `ok` is the CALLER's job and is emitted only
/// when this returns [`Resumed`](PauseOutcome::Resumed) / [`Skipped`](PauseOutcome::Skipped) — i.e. once the pause
/// has run to completion — so the host's character-counting naturally stalls while paused (this is correct flow
/// control, exactly like the `G4` dwell, NOT a deferred ack).
///
/// Sequencing, reusing the existing graceful-hold machinery rather than a parallel hold path:
/// 1. **M1 gate**: an optional stop whose [`OPTIONAL_STOP_ENABLED`] switch is OFF returns [`Skipped`] immediately
///    (no hold) — grbl's default M1 behavior. M0 and M6 always pause.
/// 2. **Drain**: wait for the core-1 executor to drain to a stop ([`wait_for_motion_idle`] — the planner already
///    flushed look-ahead so the preceding block decelerates to rest). A soft reset here returns [`Aborted`].
/// 3. **M6 prompt**: a tool-change pause emits a `[MSG:..]` NAMING `current_tool` so the operator knows which tool
///    to swap in, then resume. (The tool number is human-readable only; `skirnir` sources it from the program.)
/// 4. **Hold**: latch `Hold:0` (M0/M1) or the `Tool` state (M6) and mark [`PAUSE_ACTIVE`], then [`hold_until_resume`]
///    awaits a cycle-start [`PAUSE_RESUME`] nudge (raced against a soft reset and a graceful stop) WHILE STILL
///    SERVICING read-only `$`-queries — so the hold returns to `Normal` on `~` and a host is never stalled.
///
/// ## `$`-QUERIES ARE SERVICED DURING THE HOLD (grbl-faithful)
/// Real grbl/grblHAL answers read-only queries (`$G`/`$#`/`$$`/`$I`…) DURING a hold — the motion is held, the
/// protocol loop is not — and so does this firmware: [`hold_until_resume`] PEEKS the line queue and answers any
/// read-only `$`-query in place (its report + `ok`) without releasing the hold, while LEAVING every motion / write /
/// action line queued to run after resume. This is the fix for the earlier divergence where a `$G` on entering the
/// `Tool` state stalled a character-counting host until `~`. (`?` was always answered — it is served by the separate
/// [`status_responder`] task — so live state/DRO is available throughout regardless.) See [`hold_until_resume`] for
/// the exact serviced-vs-deferred routing.
async fn run_program_pause(
  optional: bool,
  tool_change: bool,
  current_tool: u16,
  parser: &mut Parser,
  state: &mut ConsumerState,
  flash: &'static SharedFlash,
) -> PauseOutcome {
  // 1. An M1 with the optional-stop switch off does not halt (grbl default). It is still a valid, acknowledged line.
  if optional && !OPTIONAL_STOP_ENABLED.load(Ordering::Relaxed) {
    return PauseOutcome::Skipped;
  }
  // 2. Drain pending motion to a stop so the pause holds at rest (the planner flushed look-ahead already). A soft
  //    reset abandons the drain; the consumed signal is honored by the caller.
  if !wait_for_motion_idle().await {
    return PauseOutcome::Aborted;
  }
  // 3. A manual tool change: prompt the operator to swap the tool before holding. This machine has no ATC, so the
  //    swap is manual and the operator resumes with `~` when done (grblHAL reports the `Tool` state during a manual
  //    change; the state below reports the dedicated grblHAL `Tool` state for an M6 so a sender shows a tool-change
  //    prompt, while M0/M1 report `Hold:0`).
  if tool_change {
    // Build the tool-named prompt with the host-tested formatter, then push it. A formatter failure (buffer too
    // small — unreachable for this fixed-length text) simply skips the prompt rather than panicking.
    let mut text = Response::new();
    if ResponseWriter::tool_change_message(&mut text, current_tool).is_ok() {
      send_message(text.as_str()).await;
    }
  }
  // 4. Latch the hold and mark the pause active so the `~` handler nudges THIS wait, then await the resume. An M6
  //    enters the dedicated `Tool` state (grblHAL `STATE_TOOLCHANGE`, reported as `<Tool|...>`); M0/M1 enter the
  //    feed-hold `Hold:0`. BOTH resume on cycle-start (`resumes_on_cycle_start` covers Tool too) back to `Normal`.
  let paused_state = if tool_change { ControlState::tool_change() } else { control_state().feed_hold() };
  set_control_state(paused_state);
  PAUSE_ACTIVE.store(true, Ordering::Release);
  let outcome = hold_until_resume(parser, state, flash).await;
  PAUSE_ACTIVE.store(false, Ordering::Release);
  outcome
}

/// Hold (the M0/M1/M6 pause body) until a cycle-start resume, a soft reset, or a graceful stop — while STILL
/// SERVICING read-only `$`-queries (grbl answers `$G`/`$#`/`$$`/`$I` etc. during a hold). This is the fix for the
/// "held machine appears to HANG a character-counting host" bug: a query that arrives during the hold is answered
/// (report + `ok`) IN PLACE so the host's send-ahead window frees, and the hold is NOT released.
///
/// ## Exactly what is serviced vs deferred
/// Each iteration PEEKS the head of [`LINE_QUEUE`] WITHOUT consuming it ([`Channel::try_peek`]) and routes by
/// [`peeked_line_is_holdable_query`]:
/// - A pure read-only `$`-query (`$`, `$$`, `$I`/`$I+`, `$G`, `$#`, `$N`, `$ES`/`$EG`/`$EE`/`$EA`, `$SED=`, `$PBX`):
///   CONSUME it and answer via [`handle_system_command`] (its report + the one `ok`), then loop back into the hold.
///   The `$`-query also clears the gcode error-hold, exactly as it would outside a pause (a `$` command is a grbl
///   recovery trigger).
/// - ANYTHING ELSE — a gcode/jog motion line, a blank line, a setting WRITE (`$n=val`), `$X`/`$C`/`$SLP`/`$H`, a
///   `$N0=`/`$PBX=` write, or an `Unknown` `$` command — is LEFT ON THE QUEUE (peeked, not received). It is NOT
///   executed during the hold and runs IN ORDER once the pause resumes, so a held machine never mutates state or
///   moves behind the operator's back, and stream order is preserved (no reordering — the deferred line stays at
///   the head). The `select` then drops the line arm so a deferred head does not busy-spin: it waits only on the
///   resume / reset / stop signals until one fires (or `~` resumes and the consumer's normal loop reads the line).
///
/// `?` is unaffected throughout — it is served by the separate [`status_responder`] task, so a host polling `?`
/// sees the live `<Tool|...>` / `<Hold:0|...>` state and DRO for the whole hold.
async fn hold_until_resume(
  parser: &mut Parser,
  state: &mut ConsumerState,
  flash: &'static SharedFlash,
) -> PauseOutcome {
  loop {
    // Whether the head line (if any) is a serviceable read-only `$`-query. A peek that finds the queue empty, or a
    // head that is a deferred (non-query) line, both yield `false` — in which case we wait only on the signals so a
    // deferred head cannot busy-spin the `ready_to_receive` arm.
    let head_is_query = matches!(LINE_QUEUE.try_peek(), Ok(line) if peeked_line_is_holdable_query(line.as_slice()));
    // The three terminating signals, plus — ONLY when a serviceable query is at the head — a line-ready arm. With no
    // query at the head the line arm is omitted, so the select parks on the signals until a resume/reset/stop.
    let signals = select(PAUSE_RESUME.wait(), select(SOFT_RESET.wait(), PROGRAM_STOP.wait()));
    if head_is_query {
      match select(signals, LINE_QUEUE.ready_to_receive()).await {
        Either::First(resolved) => return resolve_pause_signal(resolved),
        // A serviceable query is at the head and ready: consume it and answer it in place, then loop back into the
        // hold. `try_receive` cannot fail here — we just peeked it ready, and this task is the sole receiver.
        Either::Second(()) => {
          if let Ok(line) = LINE_QUEUE.try_receive() {
            service_held_query(line.as_slice(), parser, state, flash).await;
          }
        }
      }
    } else {
      // No serviceable query at the head (empty queue, or a deferred non-query line that must wait for resume): park
      // on the signals only. A deferred line stays at the head and is read by the consumer's normal loop after `~`.
      return resolve_pause_signal(signals.await);
    }
  }
}

/// Map the resolved pause-await signal `select` to its [`PauseOutcome`]. The soft-reset / graceful-stop signals are
/// CONSUMED here; the caller runs `apply_soft_reset` / `program_stop_cycle` directly (matching `run_dwell`'s abort
/// contract — do NOT re-signal, or the consumer would act twice). A cycle-start resume has already cleared the hold
/// level + set the control state back to `Normal` via the `~` handler's `cycle_start()`.
fn resolve_pause_signal(resolved: Either<(), Either<(), ()>>) -> PauseOutcome {
  match resolved {
    Either::First(()) => PauseOutcome::Resumed,
    Either::Second(Either::First(())) => PauseOutcome::Aborted,
    Either::Second(Either::Second(())) => PauseOutcome::Stopped,
  }
}

/// Service a read-only `$`-query line that arrived DURING a pause hold, mirroring `handle_line`'s `$`-command path:
/// trim, strip the leading `$`, clear the gcode error-hold (a `$` command is a grbl recovery trigger), and dispatch
/// to [`handle_system_command`] (which emits the report + the single `ok`). Only ever called on a line
/// [`peeked_line_is_holdable_query`] has already confirmed is a read-only query, so the strip/classify is provably a
/// read-only command; the defensive re-check keeps it correct if the head changed between peek and receive.
async fn service_held_query(line: &[u8], parser: &mut Parser, state: &mut ConsumerState, flash: &'static SharedFlash) {
  let trimmed = trim_ascii(line);
  if let Some(rest) = trimmed.strip_prefix(b"$") {
    if SystemCommand::classify(rest).is_readonly_query() {
      state.error_hold = false;
      handle_system_command(rest, parser, state, flash).await;
    }
  }
}

/// Whether a peeked line is a read-only `$`-query that may be SERVICED while an M0/M1/M6 pause hold is active
/// (without releasing the hold). True only for a `$`-prefixed line (after trimming) whose [`SystemCommand`] is a
/// [`is_readonly_query`](SystemCommand::is_readonly_query). A `$J=` jog is deliberately excluded (it is a MOTION
/// command, not a `$`-query — `strip_jog_prefix` routes it elsewhere), as is any blank/gcode line and any `$`-write.
fn peeked_line_is_holdable_query(line: &[u8]) -> bool {
  let trimmed = trim_ascii(line);
  if strip_jog_prefix(trimmed).is_some() {
    return false; // `$J=` is a jog (motion), never a held-state query.
  }
  match trimmed.strip_prefix(b"$") {
    Some(rest) => SystemCommand::classify(rest).is_readonly_query(),
    None => false,
  }
}

/// The spindle task (DOC-07, core 0 / PRO_CPU). The SINGLE driver of the spindle outputs: it owns the
/// [`SpindleController`](firmware_core::spindle::SpindleController) and awaits two signals —
/// - [`SPINDLE_UPDATE`]: re-apply from the commanded [`SPINDLE_DIRECTION`] + the override-scaled RPM (an M3/M4/M5
///   or a spindle-override / spindle-stop change). A running-spindle direction REVERSAL returns
///   [`SpindleAction::SpinDownThenReverse`]: the controller has already stopped the spindle; this task awaits the
///   `$393` reverse dwell (raced against an e-stop) then completes the reversal.
/// - [`SPINDLE_ESTOP`]: an immediate emergency stop (ALARM / soft reset / hard limit / sleep), independent of the
///   commanded state. A feed hold (`!`) deliberately does NOT signal this — the spindle keeps running (grblHAL).
///
/// The task NEVER blocks the consumer: the consumer only stores atomics + signals; all timing lives here.
#[embassy_executor::task]
pub async fn spindle(controller: &'static mut spindle::Spindle) {
  loop {
    // E-stop is polled FIRST so that when BOTH an emergency stop and an update are pending at a wake (e.g. an
    // ALARM raised on core 1 while a spindle command's update is still queued), `select`'s first-future bias
    // services the stop — never the update that would briefly re-energize the spindle before the next iteration.
    match select(SPINDLE_ESTOP.wait(), SPINDLE_UPDATE.wait()).await {
      // Emergency stop wins unconditionally: de-assert enable + zero duty regardless of the commanded state.
      Either::First(()) => spindle_emergency_stop(controller),
      Either::Second(()) => {
        // Re-read the commanded direction and the realized (override-scaled) RPM, plus the live `$30`/`$31`/`$393`.
        let action = apply_spindle(controller).await;
        if let Some(dwell_s) = action {
          // A reversal of a running spindle: the controller already stopped it. Await the `$393` reverse dwell,
          // raced against an e-stop so a reset mid-spin-down still parks the spindle, then bring up the new
          // direction (unless a newer command/e-stop changed the picture, which the re-read in `complete` honors).
          match select(Timer::after(dwell_duration(dwell_s)), SPINDLE_ESTOP.wait()).await {
            Either::First(()) => complete_spindle_reverse(controller).await,
            Either::Second(()) => spindle_emergency_stop(controller),
          }
        }
      }
    }
  }
}

/// The coolant task (DOC-07 follow-up, core 0 / PRO_CPU). The SINGLE driver of the (hardware-gated) coolant
/// outputs: it owns the [`CoolantController`](firmware_core::coolant::CoolantController) and awaits two signals,
/// exactly mirroring the [`spindle`] task —
/// - [`COOLANT_UPDATE`]: re-apply from the commanded [`COOLANT_STATE`] (an M7/M8/M9).
/// - [`COOLANT_ESTOP`]: force BOTH circuits off immediately (ALARM / soft reset / hard limit / sleep / M2/M30).
///
/// The task NEVER blocks the consumer (the consumer only stores an atomic + signals). Because the coolant GPIO is
/// stubbed today (no driver stage budgeted), the `apply`/`emergency_stop` calls drive the stub outputs — the logic,
/// the safety chokepoints, and the task topology are all real now; only the pin write is a no-op until a stage exists.
#[embassy_executor::task]
pub async fn coolant(controller: &'static mut crate::coolant::Coolant) {
  loop {
    // E-stop is polled FIRST (the `select` first-future bias) so that when BOTH an e-stop and an update are pending
    // — e.g. an ALARM raised while an M8 update is still queued — the stop wins and coolant never briefly re-asserts.
    match select(COOLANT_ESTOP.wait(), COOLANT_UPDATE.wait()).await {
      // Emergency stop wins unconditionally: both circuits off regardless of the commanded state. A driver error is
      // logged (defmt) and swallowed so the task keeps running for the next command / e-stop (matching the spindle).
      Either::First(()) => {
        if let Err(_e) = controller.emergency_stop() {
          #[cfg(feature = "defmt")]
          defmt::error!("coolant emergency-stop failed: {:?}", _e);
        }
      }
      // Re-read the commanded modal coolant state and drive both circuits to it.
      Either::Second(()) => {
        if let Err(_e) = controller.apply(commanded_coolant()) {
          #[cfg(feature = "defmt")]
          defmt::error!("coolant apply failed: {:?}", _e);
        }
      }
    }
  }
}

/// How often the [`watchdog_feed`] task pets the RTC watchdog, in milliseconds. Must be COMFORTABLY shorter than the
/// RWDT stage-0 timeout (`WATCHDOG_TIMEOUT` in `main`, 8 s) so several feeds fall inside one timeout window and a
/// single late wake (e.g. a brief flash-write quiesce that parks core 0 for tens of ms) cannot trip the dog. 500 ms
/// gives a 16x margin: the timeout only expires after ~8 s of the core-0 thread-mode executor never running this
/// task at all — i.e. a genuine core-0 wedge, exactly the condition we want a reset for, never a normal stall.
const WATCHDOG_FEED_INTERVAL: Duration = Duration::from_millis(500);

/// Number of consecutive [`WATCHDOG_FEED_INTERVAL`] ticks over which the core-1 liveness beat may stay frozen WHILE
/// a block is actively executing before [`watchdog_feed`] declares a core-1-only wedge and withholds the feed (Goal
/// B). At 500 ms/tick this is ~4 s — chosen CONSERVATIVELY above the worst-case legitimate gap between core-1 beats:
/// the executor bumps [`MOTION_LIVENESS`] per BURST (see `motion::RmtStepSink::emit_burst`), and the slowest possible
/// single burst is `MAX_SYMBOLS_PER_BURST` events × the max RMT period (`RMT_MAX_FIELD_LEN + $0` ≈ 0x7FFF ticks ≈
/// 33 ms at 1 MHz) ≈ 1.5 s, so 4 s leaves >2.5x margin and CANNOT false-trip on a real slow move. Only a genuine
/// "a block is in flight but core 1 has emitted no burst for 4 s" — i.e. core 1 wedged mid-motion (the suspected
/// RMT `wait()` spin) — withholds the feed; the already-armed 8 s RWDT then resets the board, converting an
/// otherwise-silent core-1-only stall into a recoverable reset + a captured breadcrumb.
const CORE1_STALL_TICKS: u32 = 8;

/// Number of consecutive [`WATCHDOG_FEED_INTERVAL`] ticks the [`COMMS_PROGRESS`] counter may stay frozen WHILE the
/// host is active before [`watchdog_feed`] declares a CORE-0 COMMS wedge and withholds the feed. At 500 ms/tick this
/// is ~3 s. skirnir polls `?` every ~200 ms whenever connected and the firmware answers in EVERY state, so a
/// connected-and-active board advances `COMMS_PROGRESS` ~5 Hz (every tick) via three independent tasks
/// (`status_responder`, `usb_tx`, `comms_consumer`); if ALL THREE freeze for 3 s while the host is still present, the
/// comms path is genuinely wedged — the real-board failure (writes succeed, no responses, DRO frozen) — with no
/// legitimate counterexample (back-pressure still leaves `?` answered). ~3 s is short enough that the trip fires
/// WHILE the host is still flowing or recently-flowing RX (see [`RX_ACTIVE_TICKS`]); three independent bumpers + the
/// host-active gate keep it from false-tripping.
const COMMS_STALL_TICKS: u32 = 6;

/// The "host is present" sticky window, in [`WATCHDOG_FEED_INTERVAL`] ticks since [`RX_ACTIVITY`] last advanced. The
/// host counts as active for ~6 s after its last received byte. This is BOTH the reset-loop guard AND the bridge
/// across the host's flow-control quiet gap: when the comms path wedges, the host keeps streaming only until its
/// character-counting window fills (it got no `ok`s) — on the real board skirnir sent ~30 more lines (~1-2 s) then
/// went quiet. A 6 s sticky window keeps the host "active" across that quiet gap so the ~3 s [`COMMS_STALL_TICKS`]
/// trip still fires, while a board with NO host (RX never advances) goes inactive after 6 s and FEEDS NORMALLY
/// forever — never a reset-loop. The counter is SEEDED idle (host inactive) at task start, so a board that boots
/// with no host present never spuriously counts as active before the first real RX byte.
const RX_ACTIVE_TICKS: u32 = 12;

/// The RTC watchdog feed task (core 0 / PRO_CPU, a plain thread-mode task) — a proper TASK-watchdog (revised after a
/// real-board wedge where the Embassy executor stayed alive but the comms path was stuck on a never-resolving
/// `.await`, so the original unconditional feed kept the dog quiet and a physical EN-reset was needed). It feeds the
/// RWDT only while the firmware is making real forward progress, and WITHHOLDS the feed — letting the already-armed
/// 8 s RWDT auto-reset the board — in three wedge classes:
/// 1. **Core-0 executor death** (a hang/deadlock/fault that wedged the whole thread-mode executor): this task simply
///    never runs, so the dog is never fed. Caught implicitly, no logic needed.
/// 2. **Core-1 motion wedge** (the suspected RMT `wait()` spin): [`MOTION_LIVENESS`] frozen for [`CORE1_STALL_TICKS`]
///    WHILE `EXECUTOR_RUNNING` (a block in flight). Idle/parked/dwell clear `EXECUTOR_RUNNING`, so they never trip.
/// 3. **Core-0 comms stall** (the NEW case — executor alive but the comms pipeline stuck): [`COMMS_PROGRESS`] frozen
///    for [`COMMS_STALL_TICKS`] WHILE the host is active ([`RX_ACTIVITY`] advanced within [`RX_ACTIVE_TICKS`]).
/// Either withhold records its reason in the [`crate::crash`] breadcrumb, so the boot `[MSG:CRASH ...]` names the
/// wedge class (`core1-motion-wedge` vs `core0-comms-wedge`).
///
/// ## Reset-loop / false-trip safety (the load-bearing guards)
/// - The comms-stall withhold is gated on RX activity, so a QUIESCENT or DISCONNECTED board (no host polling, so
///   `COMMS_PROGRESS` naturally sits still) is NEVER reset — there is no host to serve, so a still counter is
///   correct, not a wedge. This is what prevents a boot→reset→boot loop on a board left sitting at a prompt.
/// - `COMMS_PROGRESS` is bumped by THREE independent tasks; legitimate back-pressure (the consumer blocked in a
///   `QueueFull` retry) still leaves `status_responder`/`usb_tx` answering `?`, so the counter keeps advancing — a
///   stall requires ALL host-facing work to stop, which is the genuine wedge.
/// - The core-1 check is unchanged (block-in-flight gated), so it cannot false-trip on idle/hold/dwell.
///
/// ## Breadcrumb snapshots
/// Each tick pushes a `(seq, comms_progress, motion_liveness)` snapshot into the RTC_FAST crash ring
/// ([`crate::crash::push_snapshot`]) — OFF the real-time path — so after a reset the boot dump can show which side
/// stopped advancing FIRST. The feed task NO LONGER self-bumps `COMMS_PROGRESS` (that polluted the verdict and
/// masked the comms wedge); the snapshot reads the genuine, work-driven counters.
///
/// `Rtc::rwdt::feed` takes `&mut self`, so the task owns the `Rtc` by `&'static mut` (parked in a `StaticCell` in
/// `main`); it is the SOLE feeder, so no lock is needed.
#[embassy_executor::task]
pub async fn watchdog_feed(rtc: &'static mut esp_hal::rtc_cntl::Rtc<'static>) -> ! {
  // Previous samples + frozen-tick counts for the two conditional withholds. Seeded from the first read so the first
  // delta is meaningful rather than a spurious "moved from 0".
  let mut last_core1 = MOTION_LIVENESS.load(Ordering::Relaxed);
  let mut last_comms = COMMS_PROGRESS.load(Ordering::Relaxed);
  let mut last_rx = RX_ACTIVITY.load(Ordering::Relaxed);
  let mut core1_frozen_ticks: u32 = 0;
  let mut comms_frozen_ticks: u32 = 0;
  // Seed the RX-idle counter at the threshold so the host starts INACTIVE: a board that boots with no host present
  // must not count as "host active" before the first real RX byte arrives (else the comms-stall check could trip on
  // a host-less board in the first few seconds — a reset loop). The first RX advance resets this to 0.
  let mut rx_idle_ticks: u32 = RX_ACTIVE_TICKS;
  loop {
    let core1 = MOTION_LIVENESS.load(Ordering::Relaxed);
    let comms = COMMS_PROGRESS.load(Ordering::Relaxed);
    let rx = RX_ACTIVITY.load(Ordering::Relaxed);

    // Core-1 motion-stall detection: beat frozen WHILE a block is in flight. `EXECUTOR_RUNNING` false (idle / parked
    // / dwell) resets the count, so a legitimately non-advancing beat is never a stall.
    let block_in_flight = EXECUTOR_RUNNING.load(Ordering::Acquire);
    if block_in_flight && core1 == last_core1 {
      core1_frozen_ticks = core1_frozen_ticks.saturating_add(1);
    } else {
      core1_frozen_ticks = 0;
    }

    // Host-activity ageing: how many consecutive ticks since RX last advanced. Resets to 0 on any RX advance.
    if rx == last_rx {
      rx_idle_ticks = rx_idle_ticks.saturating_add(1);
    } else {
      rx_idle_ticks = 0;
    }
    let host_active = rx_idle_ticks < RX_ACTIVE_TICKS;

    // Core-0 comms-stall detection: `COMMS_PROGRESS` frozen WHILE the host is active. When the host is NOT active
    // (no recent RX → idle/disconnected) the count is held at 0 — a still counter with no host is correct, never a
    // wedge — which is the reset-loop guard. Only "host driving + no comms forward progress" accrues toward a reset.
    if host_active && comms == last_comms {
      comms_frozen_ticks = comms_frozen_ticks.saturating_add(1);
    } else {
      comms_frozen_ticks = 0;
    }

    last_core1 = core1;
    last_comms = comms;
    last_rx = rx;

    // Push a liveness snapshot (genuine work-driven counters) into the RTC_FAST crash ring so a reset's boot dump
    // can determine which side stopped advancing first.
    crate::crash::push_snapshot(comms, core1);

    let core1_wedged = core1_frozen_ticks >= CORE1_STALL_TICKS;
    let comms_wedged = comms_frozen_ticks >= COMMS_STALL_TICKS;

    if core1_wedged || comms_wedged {
      // A genuine wedge: record the class in the breadcrumb, then WITHHOLD the feed and let the 8 s RWDT reset the
      // board. The core-1 check takes precedence in the (impossible-in-practice) both-true case since its breadcrumb
      // stage marker is the more specific datum. We still await so we never busy-spin core 0 while the dog runs out.
      let reason = if core1_wedged {
        crate::crash::WithholdReason::Core1Motion
      } else {
        crate::crash::WithholdReason::Core0Comms
      };
      crate::crash::record_withhold(reason);
      #[cfg(feature = "defmt")]
      if core1_wedged {
        defmt::error!("watchdog: core-1 wedged mid-motion ({=u32} ticks) — withholding feed to force reset", core1_frozen_ticks);
      } else {
        defmt::error!("watchdog: core-0 comms stalled ({=u32} ticks, host active) — withholding feed to force reset", comms_frozen_ticks);
      }
      Timer::after(WATCHDOG_FEED_INTERVAL).await;
      continue;
    }

    // Healthy, idle, or host-absent: pet the dog. The whole loop body is a handful of atomic ops + one await, so it
    // can never delay the feed past the 8 s timeout.
    rtc.rwdt.feed();
    #[cfg(feature = "defmt")]
    {
      if core1_frozen_ticks > 0 {
        defmt::warn!("watchdog: core-1 beat frozen ({=u32}/{=u32} ticks) while executing", core1_frozen_ticks, CORE1_STALL_TICKS);
      }
      if comms_frozen_ticks > 0 {
        defmt::warn!("watchdog: comms frozen ({=u32}/{=u32} ticks) while host active", comms_frozen_ticks, COMMS_STALL_TICKS);
      }
    }
    Timer::after(WATCHDOG_FEED_INTERVAL).await;
  }
}

/// Read the commanded spindle direction + override-scaled RPM + the live `$30`/`$31`/`$393`, apply them to the
/// controller, and return `Some(dwell_s)` when the apply scheduled a direction reversal (the caller must await the
/// reverse dwell), or `None` when the command was fully applied. A driver error is logged (defmt) and swallowed —
/// the spindle task must keep running so a later command / e-stop can still reach the hardware.
async fn apply_spindle(controller: &mut spindle::Spindle) -> Option<f32> {
  let settings = settings_snapshot().await;
  let (state, rpm) = commanded_spindle();
  match controller.apply(state, rpm, settings.spindle_rpm_min, settings.spindle_rpm_max, settings.spindle_reverse_dwell_s) {
    Ok(SpindleAction::Applied) => None,
    Ok(SpindleAction::SpinDownThenReverse { dwell_s }) => Some(dwell_s),
    Err(_e) => {
      #[cfg(feature = "defmt")]
      defmt::error!("spindle apply failed: {:?}", _e);
      None
    }
  }
}

/// Complete a deferred M3↔M4 reversal after the `$393` dwell: bring up the (re-read) commanded direction at the
/// current scaled RPM. Re-reading honors a command that changed during the dwell (e.g. an M5 mid-spin-down parks
/// it rather than energizing the stale direction). A driver error is logged and swallowed.
async fn complete_spindle_reverse(controller: &mut spindle::Spindle) {
  let settings = settings_snapshot().await;
  let (state, rpm) = commanded_spindle();
  if let Err(_e) = controller.complete_reverse(state, rpm, settings.spindle_rpm_min, settings.spindle_rpm_max) {
    #[cfg(feature = "defmt")]
    defmt::error!("spindle reverse-complete failed: {:?}", _e);
  }
}

/// Emergency-stop the spindle (de-assert enable + zero duty), logging and swallowing any driver error so the task
/// keeps running. Idempotent at the controller level.
fn spindle_emergency_stop(controller: &mut spindle::Spindle) {
  if let Err(_e) = controller.emergency_stop() {
    #[cfg(feature = "defmt")]
    defmt::error!("spindle emergency-stop failed: {:?}", _e);
  }
}

/// The currently commanded spindle `(state, rpm)`: the modal direction from [`SPINDLE_DIRECTION`] and the
/// realized RPM = the programmed `S` scaled by the live spindle override + spindle-stop toggle. So an
/// override/stop change re-drives the controller at the new speed (or to a stop) on the next [`SPINDLE_UPDATE`].
fn commanded_spindle() -> (SpindleState, f32) {
  let state = match SPINDLE_DIRECTION.load(Ordering::Acquire) {
    SPINDLE_DIR_CW => SpindleState::Clockwise,
    SPINDLE_DIR_CCW => SpindleState::CounterClockwise,
    _ => SpindleState::Stop,
  };
  let programmed = PROGRAMMED_SPINDLE_RPM.load(Ordering::Acquire).min(u16::MAX as u32) as u16;
  let scaled = overrides().scaled_rpm(programmed);
  (state, scaled as f32)
}

/// Derive the fixed per-tick step period (in motion timer ticks) for a probe seeking `target` (machine steps) at
/// `feed` in `units`/min. grbl probes at a CONSTANT feed (no trapezoid), so a single period is used for every
/// step. The period is the tick rate divided by the dominant-axis step rate: `feed` → mm/s, scaled by the
/// dominant axis's steps/mm and divided into the move's mm length so the dominant axis (the one that steps every
/// tick) runs at the commanded surface speed. A degenerate/zero feed falls back to the slowest representable
/// period; [`ProbeStepper::run_probe`] clamps the final value into the encodable interval regardless.
async fn probe_step_period_ticks(target: &[i32; AXES], feed: f32, units: GcodeUnits) -> u32 {
  let settings = settings_snapshot().await;
  let steps_per_mm = settings.steps_per_mm();
  // The commanded machine position the probe starts from (grbl's `gc_state.position`), in steps.
  let start = {
    let guard = PLANNER.lock().await;
    guard.as_ref().map(Planner::position_steps).unwrap_or([0; AXES])
  };
  // The dominant axis is the one with the most steps over the probe travel — it steps every tick, so its step
  // rate sets the period. Compute the mm travel and the dominant step count to convert the surface feed (mm/min)
  // into a dominant-axis step period.
  let mut dom = 0usize;
  let mut dom_steps = 0u32;
  let mut sumsq_mm = 0.0f32;
  for axis in 0..AXES {
    let delta_steps = (target[axis] - start[axis]).unsigned_abs();
    if delta_steps > dom_steps {
      dom_steps = delta_steps;
      dom = axis;
    }
    if steps_per_mm[axis] > 0.0 {
      let mm = (target[axis] - start[axis]) as f32 / steps_per_mm[axis];
      sumsq_mm += mm * mm;
    }
  }
  let length_mm = libm::sqrtf(sumsq_mm);
  let feed_mm_s = (feed * units_scale(units) / 60.0).max(0.0);
  // mm of travel per dominant-axis step, then the dominant step rate, then the period in ticks.
  if dom_steps == 0 || length_mm <= 0.0 || feed_mm_s <= 0.0 || steps_per_mm[dom] <= 0.0 {
    // No motion or no feed: fall back to the slowest representable period (the stepper clamps it anyway). The
    // motion tick rate is the fixed firmware constant the executor uses.
    return u32::MAX;
  }
  let mm_per_dom_step = length_mm / dom_steps as f32;
  let dom_step_rate = feed_mm_s / mm_per_dom_step;
  let tick_hz = settings.motion_config(crate::MOTION_TICK_HZ).tick_hz;
  let period = tick_hz / dom_step_rate.max(f32::MIN_POSITIVE);
  if period.is_finite() && period > 0.0 {
    libm::roundf(period) as u32
  } else {
    u32::MAX
  }
}

/// Run a `G38.x` probe cycle end-to-end (Phase C). Dispatches the [`ProbeRequest`] to the core-1 executor and
/// AWAITS the [`ProbeResult`] (a probe is a synchronized boundary, so the consumer blocks until it completes),
/// racing a soft reset so a `0x18` mid-probe aborts cleanly. On a result it: syncs the planner's commanded
/// position to the latched stop point (grbl sets `gc_state.position` to the probe stop), stores the last-probe
/// result and pushes the immediate `[PRB:x,y,z:flag]` line, then returns the [`ProbeResult`] so the caller
/// decides ALARM (for an alarming mode that did not trigger) vs `ok`. Returns `None` if a soft reset preempted
/// the probe (the caller honors the consumed reset signal).
async fn run_probe_cycle(request: ProbeRequest) -> Option<ProbeResult> {
  // Drain any stale result so a result from a previous (aborted) probe cannot be mistaken for this one's.
  PROBE_RESULT.try_take();
  PROBE_REQUEST.signal(request);
  let result = match select(PROBE_RESULT.wait(), SOFT_RESET.wait()).await {
    Either::First(result) => result,
    // A soft reset landed mid-probe: abort. Drain the PROBE_REQUEST we just signalled in case the executor had
    // not yet consumed it (Finding #4) — otherwise a still-latched request would run an UNREQUESTED probe move
    // from the freshly-zeroed origin after the reset rebuilds the pipeline. (The `0x18` dispatch also drains
    // PROBE_REQUEST, but this is the in-order owner draining the request IT raised, so the abort is race-free
    // even if the executor consumed-then-was-reset between the two.) The executor's own MOTION_RESET path zeroes
    // the live position and the consumer's reset rebuilds the pipeline; honor the reset by returning None.
    Either::Second(()) => {
      PROBE_REQUEST.try_take();
      return None;
    }
  };
  // Sync the planner's commanded position to the actual stop point so subsequent moves resolve from there.
  {
    let mut guard = PLANNER.lock().await;
    if let Some(planner) = guard.as_mut() {
      planner.sync_position(result.stop_steps);
    }
  }
  // The latched MACHINE position in mm (the trigger point on success, end-of-travel on no-contact). Store it as
  // the last-probe result (for `$#`) and emit the immediate `[PRB:]` push line.
  let steps_per_mm = settings_snapshot().await.steps_per_mm();
  let position_mm = steps_to_mm(&result.stop_steps, &steps_per_mm);
  set_last_probe(LastProbe { position_mm, success: result.triggered });
  send_probe_report(&position_mm, result.triggered).await;
  Some(result)
}

/// Emit the immediate `[PRB:x,y,z:success]` probe-result push line (DOC-09, the auto-message a height-mapping
/// sender reads). Queued through the single USB writer like every other response; the per-line `ok`/`ALARM`
/// follows it, preserving the one-response-per-line contract (the `[PRB:]` is an extra report line, not the ack).
async fn send_probe_report(position_mm: &[f32; AXES], success: bool) {
  let mut s = Response::new();
  if ResponseWriter::probe_report(&mut s, position_mm, success).is_ok() {
    enqueue(s).await;
  }
}

/// Decide the response to a completed `G38.x` probe (Phase C). The `[PRB:]` push and the position sync already
/// happened in [`run_probe_cycle`]; this only chooses the per-line ack/alarm:
/// - **Triggered:** the probe saw its expected edge → emit a single `ok`, staying Idle (Normal). The
///   `[PRB:..:1]` line already preceded it.
/// - **Not triggered, alarming mode (G38.2/.4):** enter the alarm and emit `ALARM:N` — `ALARM:4`
///   ([`AlarmCode::ProbeFailInitial`]) when the probe was ALREADY at its expected edge before any motion (grbl's
///   "probe not in the expected initial state"), else `ALARM:5` ([`AlarmCode::ProbeFailContact`], "did not
///   contact within travel"). grbl emits no `ok` for a probe that alarms.
/// - **Not triggered, silent mode (G38.3/.5):** no alarm — emit a single `ok` and let the sender check the
///   `[PRB:..:0]` flag itself.
///
/// A soft reset preempting the probe (`run_probe_cycle` returned `None`) runs the soft-reset transition whose
/// signal was consumed, exactly like the back-pressure abort path.
async fn handle_probe(request: ProbeRequest, alarm_on_fail: bool, parser: &mut Parser, state: &mut ConsumerState) {
  let Some(result) = run_probe_cycle(request).await else {
    // A `0x18` landed mid-probe: honor the consumed reset signal (rebuild the pipeline, emit the banner).
    apply_soft_reset(parser, state).await;
    return;
  };
  // The `[PRB:..:flag]` push already went out in `run_probe_cycle`; decide the per-line ack/alarm from the
  // host-tested truth table (success → ok; alarming-mode failure → ALARM:4 already-at-edge / ALARM:5 no contact;
  // silent-mode failure → ok).
  match probe_response(result.triggered, result.already_at_edge, alarm_on_fail) {
    ProbeResponse::Ok => ack().await,
    ProbeResponse::Alarm(code) => {
      // A probe-fail alarm latches (grbl locks it until reset/`$X`), gating subsequent GCode. Emit `ALARM:N` and
      // its `[MSG:..]` continue prompt; grbl emits no `ok` for a probe that alarms.
      set_control_state(ControlState::Alarm(code));
      emit_alarm(code).await;
    }
  }
}

/// Strip a case-insensitive `$J=` jog prefix, returning the jog body (the bytes after `=`) when present. grbl's
/// jog command is `$J=<gcode>`; only the leading `J` is case-insensitive (the `$`/`=` are literal). Returning
/// the body lets `handle_line` route a jog as MOTION ahead of the `$`-system dispatch (a jog is not a setting).
fn strip_jog_prefix(line: &[u8]) -> Option<&[u8]> {
  let rest = line.strip_prefix(b"$")?;
  match rest.split_first() {
    Some((&c, tail)) if c.eq_ignore_ascii_case(&b'J') => tail.strip_prefix(b"="),
    _ => None,
  }
}

/// Handle one `$J=` jog line (DOC-08 Phase D), emitting exactly one `ok`/`error:N`.
///
/// Gating (grbl): a jog is honored only when (a) no GCode error-hold is active, and (b) the machine is Idle or
/// already jogging — it is rejected while running a PROGRAM, in a feed-hold, alarm, check, or sleep state. The
/// "running a program" case is `Normal` control state with a non-jog block in flight; we reject it so a jog never
/// blends into program motion. The jog is parsed in the parser's throwaway modal context (it never mutates
/// `gc_state`), planned into a cancelable jog block (rejected up front if `$20` soft limits would be exceeded),
/// the machine is latched into `Jog`, and the executor is woken — exactly one `ok` on success.
async fn handle_jog(line: &[u8], parser: &mut Parser, state: &mut ConsumerState) {
  // Drop a stale jog latch first if the previous jog has fully drained, so the gate below sees the true state.
  refresh_jog_state().await;
  if state.error_hold {
    // Held by a prior GCode error: reject until a recovery trigger, like any motion line. Bare (no `[MSG:..]`
    // annotation): the hold code's name does not describe why this held line was rejected (see `error_bare`).
    error_bare(ERROR_HOLD_CODE).await;
    return;
  }
  let control = control_state();
  // Reject a jog from a non-Idle/Jog mode (hold/alarm/check/sleep), or from Normal while a PROGRAM block is in
  // flight (Run). grbl returns the generic "command requires the machine to be idle" rejection; reuse the
  // unsupported-command code so a sender halts, matching the alarm/hold rejections elsewhere.
  if !control.jog_allowed() || (control == ControlState::Normal && program_running().await) {
    error(ERROR_UNSUPPORTED_COMMAND).await;
    return;
  }
  let jog = match parser.parse_jog(line) {
    Ok(jog) => jog,
    Err(e) => {
      // A jog parse error (missing F → 22, missing axis → 23, unsupported word → 20, lexer 1/2 for a malformed
      // word/number) is reported, but a jog does NOT arm the gcode error-hold — it is independent of the program
      // stream (grbl keeps streaming after a rejected jog).
      error(e.code()).await;
      return;
    }
  };
  let limits = current_soft_limits().await;
  let outcome = {
    let mut guard = PLANNER.lock().await;
    match guard.as_mut() {
      Some(planner) => planner.plan_jog(&jog, limits),
      None => Err(PlannerError::JogExceedsTravel), // Unreachable in a wired build; fail loudly rather than fake-ack.
    }
  };
  match outcome {
    Ok(PlannerOutcome::Queued { blocks }) => {
      // Latch Jog and wake the executor so the jog block runs. A zero-block jog (target == current position) is
      // a valid no-op: it still `ok`s, but needs no executor wake and leaves the state Idle (no block in flight).
      if blocks > 0 {
        set_control_state(control.begin_jog());
        BLOCK_AVAILABLE.signal(());
      }
      ack().await;
    }
    // A soft-limit rejection (`$20` on): the jog is ignored and the host sees `error:15` (travel exceeded). No
    // block was enqueued and the control state is untouched.
    Err(e) => error(e.code()).await,
    // `plan_jog` only ever returns `Queued`/`JogExceedsTravel`; any other outcome is an internal invariant break.
    Ok(_) => ack().await,
  }
}

/// Build the `$20`/`$130–$132` soft-limit envelope to check a move/jog against, or `None` so the planner skips
/// the check. The envelope is supplied ONLY when `$20` is enabled AND the machine is homed ([`HOMED`]) — a soft
/// limit is only meaningful once a machine zero is established (research finding #16), so an unhomed machine
/// never gates a move against an unestablished zero. (`$20` itself can only be enabled with `$22` set, and the
/// machine is homed after `$H`; when homing is disabled, [`HOMED`] is treated as true so a force-enabled `$20`
/// still works.) Shared by the program-move path ([`plan_command`]) and the jog path ([`handle_jog`]).
async fn current_soft_limits() -> Option<SoftLimits> {
  if !HOMED.load(Ordering::Relaxed) {
    return None;
  }
  let settings = settings_snapshot().await;
  settings
    .soft_limits_enable
    .then_some(SoftLimits { max_travel_mm: settings.max_travel_mm })
}

/// Whether a PROGRAM (non-jog) block is currently in flight: the executor is mid-block or the planner queue holds
/// blocks. Used to reject a jog from a running program (grbl: jog only from Idle). When the machine is already in
/// `Jog`, queued blocks are jog blocks, so the caller does not consult this — it only matters in `Normal`.
async fn program_running() -> bool {
  if EXECUTOR_RUNNING.load(Ordering::Acquire) {
    return true;
  }
  let guard = PLANNER.lock().await;
  guard.as_ref().map(|p| !p.is_empty()).unwrap_or(false)
}

/// Drop a stale `Jog` latch back to `Normal` once the jog has fully drained (no jog block queued or in flight),
/// so a subsequent program GCode line is not blocked and `?` reports `Idle`. Called at the top of the jog and
/// GCode plan paths. The reported state already reads `Idle` while latched-`Jog`-but-quiescent (machine_state
/// derives Idle from `running == false`), so this only keeps the LATCH honest for the gcode/jog gates.
async fn refresh_jog_state() {
  if control_state() != ControlState::Jog {
    return;
  }
  if program_running().await {
    return; // Jog blocks still in flight — stay in Jog.
  }
  set_control_state(ControlState::Jog.cancel_jog());
}

/// The outcome of [`quiesce_executor`]: whether the executor reached a parked rest, or a soft reset preempted
/// the quiesce (in which case the caller must abandon its boundary-stop sequence and honor the reset).
enum QuiesceOutcome {
  /// The executor has come to rest at a block boundary on the hold level (a real acknowledgment, not a poll):
  /// no block is in flight and the live position is stable, so the caller may now sync the planner to it.
  Parked,
  /// A `0x18` soft reset landed during the quiesce; the consumed reset signal must be honored by the caller.
  ResetPreempted,
}

/// Stop the core-1 executor at the next block boundary and WAIT until it has actually quiesced, returning a real
/// acknowledgment (Finding #11 / #3). This is the ONE shared "park the executor and confirm it parked" primitive
/// reused by jog-cancel and probe-abort, closing the race the old code had between "executor cleared
/// `EXECUTOR_RUNNING`" and "executor has actually parked" (the old jog-cancel polled `EXECUTOR_RUNNING` then
/// blind-signalled a `CYCLE_START` that could be drained as stale).
///
/// It RAISES the hold LEVEL ([`HOLD_REQUESTED`]) — the authoritative source of truth the executor honors at
/// every boundary and in its empty-queue wait — wakes the executor, and then awaits the executor's
/// [`MOTION_PARKED`] acknowledgment, which the executor pulses exactly when it parks on the level. If the
/// executor was ALREADY idle/parked (no block in flight) it still re-evaluates the level on the `HOLD_WAKE`
/// nudge and pulses `MOTION_PARKED`, so this resolves promptly in the common already-stopped case too. The wait
/// is raced against a soft reset so a `0x18` mid-quiesce is honored rather than hanging. The caller is
/// responsible for RELEASING the hold (clearing [`HOLD_REQUESTED`] + waking) once it has finished its
/// position-sync — see [`release_hold`].
async fn quiesce_executor() -> QuiesceOutcome {
  // Drain any stale park acknowledgment so we wait on THIS quiesce's park, not a previous one's.
  MOTION_PARKED.try_take();
  HOLD_REQUESTED.store(true, Ordering::Release);
  HOLD_WAKE.signal(());
  match select(MOTION_PARKED.wait(), SOFT_RESET.wait()).await {
    Either::First(()) => QuiesceOutcome::Parked,
    // A soft reset preempted the quiesce: re-signal it so `comms_consumer` runs its reset, and report the
    // preemption so the caller abandons its boundary-stop (the reset clears the hold level, flushes the planner,
    // and zeroes the position, superseding whatever the caller was about to sync). The reset's own
    // `HOLD_REQUESTED` clear releases the executor, so the caller must NOT also release the hold.
    Either::Second(()) => {
      SOFT_RESET.signal(());
      QuiesceOutcome::ResetPreempted
    }
  }
}

/// Release a hold raised by [`quiesce_executor`]: clear the hold LEVEL and wake the executor so it leaves its
/// parked branch (returning to await the next block — its queue is empty after the caller's flush). Clearing
/// the LEVEL (not signalling an edge) is what makes the release impossible to lose (Finding #11).
fn release_hold() {
  HOLD_REQUESTED.store(false, Ordering::Release);
  HOLD_WAKE.signal(());
}

/// Run a jog-cancel (`0x85`, DOC-08 Phase D) end to end, REUSING the shared [`quiesce_executor`] boundary stop. It
/// flushes the trailing queued jog blocks under the planner lock, parks the executor at the active jog block's
/// boundary WITH A REAL ACKNOWLEDGMENT (no `EXECUTOR_RUNNING` poll + blind cycle-start — Finding #3), syncs the
/// planner's commanded position to the actual live stop point (the Phase-C [`Planner::sync_position`] mechanism),
/// releases the hold, and returns the machine to `Normal`/Idle. A jog never changed modal/coordinate state, so no
/// reset side-effects are needed and no alarm is raised.
///
// TODO(DOC-02 Stage-2): mid-block jog-cancel ramp-down. We stop at the current block boundary (reusing the
// feed-hold path) rather than ramping the velocity down mid-block; a smooth mid-block deceleration is the Stage-2
// refinement, identical to the feed-hold smooth-ramp follow-up. The block-boundary approximation lives here.
async fn cancel_jog_cycle() {
  if control_state() != ControlState::Jog {
    return; // Lost the race to a soft reset / drain; nothing to cancel.
  }
  // 1. Flush the trailing queued jog blocks FIRST, under the planner lock, so the executor has nothing more to
  //    pop after it finishes the active block — the in-flight block stops at its boundary and no flushed-away
  //    block follows it.
  {
    let mut guard = PLANNER.lock().await;
    if let Some(planner) = guard.as_mut() {
      planner.flush_jog_blocks();
    }
  }
  // 2. Park the executor at the active jog block's boundary via the shared quiesce primitive, which raises the
  //    hold level and AWAITS a real parked acknowledgment — closing the race the old `EXECUTOR_RUNNING` poll had
  //    with the executor's boundary clear. A soft reset mid-quiesce is honored (the reset supersedes the cancel).
  match quiesce_executor().await {
    QuiesceOutcome::Parked => {}
    // The reset already cleared the hold level, flushed the planner, and zeroed the position; abandon the cancel
    // and let `comms_consumer` run the re-signalled reset. Do NOT release the hold — the reset already did.
    QuiesceOutcome::ResetPreempted => return,
  }
  // 3. Sync the planner's commanded position to the ACTUAL live stop point (grbl sets `gc_state.position` to the
  //    jog stop), so a subsequent move resolves from where the machine really stopped, not the jog target. The
  //    executor is genuinely parked now (the quiesce acknowledged it), so the live position is stable to read.
  let stop_steps = read_live_position();
  {
    let mut guard = PLANNER.lock().await;
    if let Some(planner) = guard.as_mut() {
      planner.sync_position(stop_steps);
    }
  }
  // 4. Release the hold so the executor leaves its parked branch — the queue is now empty, so it returns to
  //    awaiting the next block — and return to Normal/Idle with no alarm, no modal change.
  release_hold();
  set_control_state(ControlState::Jog.cancel_jog());
}

/// Run a graceful program stop (`0x86`, Galdr extension) end to end, GENERALIZING [`cancel_jog_cycle`] from a jog
/// to a running program. It decelerates the running/held program to a controlled stop at the active block's
/// boundary (no step loss), flushes the WHOLE planner queue + any in-progress arc, syncs the planner's commanded
/// position to the actual live stop point, clears the program/modal-run state (mirroring `M30`), DROPS the aborted
/// program's buffered inbound stream (so no already-streamed line is re-parsed/executed/`ok`'d after the stop), and
/// returns the machine to Idle (NOT alarm) with position RETAINED. It raises no alarm and re-emits no banner — the
/// operator's clean "stop the job", distinct from the `0x18` abort ([`apply_soft_reset`] → `ALARM:3` + banner + warm
/// reset).
///
/// ## How it composes the existing machinery
/// The boundary-stop + sync is the SAME [`quiesce_executor`] / [`release_hold`] primitive jog-cancel uses (so the
/// stop is a real parked acknowledgment, not a poll); the difference is it flushes EVERY queued block via
/// [`Planner::flush_queue`] (not just trailing jogs) and [`Planner::abort_arc`] (so a partially-streamed arc is
/// discarded), then clears the modal/spindle/override state exactly as [`program_end`] (`M30`) does — WITHOUT a
/// warm reset (no parser rebuild that loses coordinates, no `force_spindle_off`-driven banner, no position zero).
///
/// ## Safe from Run or Hold, benign otherwise
/// The reader half only signals this when [`ControlState::program_stop_quiesces`] holds, but the state can change
/// between the signal and here (a soft reset winning a tie), so this RE-CHECKS the live state and is a benign no-op
/// if a program is no longer running/held. A soft reset landing mid-quiesce is honored (the quiesce reports
/// `ResetPreempted` and the re-signalled `0x18` runs its own reset), so the abort always wins a race with the stop.
///
// TODO(DOC-02 Stage-2): mid-block ramp-down. Like jog-cancel and feed-hold, we stop at the current block boundary
// rather than ramping velocity down mid-block; a smooth mid-block deceleration is the shared Stage-2 refinement.
async fn program_stop_cycle(parser: &mut Parser, state: &mut ConsumerState) {
  // Re-check the live state: the reader gated on `program_stop_quiesces`, but a soft reset could have won a tie and
  // moved us out of Run/Hold. A stop is only meaningful from a running/held program; anything else is a no-op.
  if !control_state().program_stop_quiesces() {
    return;
  }
  // 1. Flush the WHOLE queue (program + any trailing jog blocks) AND any in-progress arc FIRST, under the planner
  //    lock, so the executor has nothing more to pop after it finishes the active block — the in-flight block stops
  //    at its boundary and no flushed-away block follows it. `abort_arc` drops a partially-streamed over-subdivided
  //    arc so its remaining segments are never fed after the stop.
  {
    let mut guard = PLANNER.lock().await;
    if let Some(planner) = guard.as_mut() {
      planner.flush_queue();
      planner.abort_arc();
    }
  }
  // 2. Park the executor at the active block's boundary via the shared quiesce primitive (raises the hold level and
  //    AWAITS a real parked acknowledgment). A soft reset mid-quiesce is honored — the reset supersedes the stop.
  match quiesce_executor().await {
    QuiesceOutcome::Parked => {}
    // The reset already cleared the hold level, flushed the planner, and (now) RETAINED the position; abandon the
    // stop and let `comms_consumer` run the re-signalled reset. Do NOT release the hold — the reset already did.
    QuiesceOutcome::ResetPreempted => return,
  }
  // 3. Sync the planner's commanded position to the ACTUAL live stop point so a subsequent move resolves from where
  //    the machine really stopped (the executor is genuinely parked now, so the live position is stable to read).
  //    This is also what keeps the planner's commanded position consistent with the RETAINED live MPos (Change A).
  let stop_steps = read_live_position();
  {
    let mut guard = PLANNER.lock().await;
    if let Some(planner) = guard.as_mut() {
      planner.sync_position(stop_steps);
    }
  }
  // 4. Release the hold so the executor leaves its parked branch (its queue is empty, so it returns to awaiting the
  //    next block) and clear the program/modal-run state, mirroring `M30`: stop the spindle, reset the parser modal
  //    state + spindle tracking to power-on defaults, select G54, and reset the live overrides — so the next stream
  //    starts clean. Coordinates/offsets and the live MACHINE POSITION are deliberately RETAINED (this is a clean
  //    stop, not a warm reset): no banner, no parser-rebuild that drops the WCS, no position zero.
  release_hold();
  force_spindle_off();
  // A graceful program stop clears coolant too (it mirrors M30's reset-to-defaults); force both off.
  force_coolant_off();
  state.last_coolant = 0;
  PROGRAMMED_SPINDLE_RPM.store(0, Ordering::Release);
  // Like M30 / soft reset, the CURRENT tool is RETAINED across a graceful stop (the spindle still holds it); carry
  // it across the parser rebuild.
  let retained_tool = parser.state().current_tool;
  *parser = Parser::new();
  parser.set_current_tool(retained_tool);
  state.spin_up = SpinUpGate::new();
  state.last_spindle_dir = SpindleState::Stop;
  state.last_spindle_rpm = 0;
  sync_active_wcs(0).await;
  set_overrides(Overrides::new());
  reset_ov_reporter();
  // 5. Discard the aborted program's BUFFERED INBOUND stream, mirroring the `0x18` soft-reset flush in
  //    `dispatch_realtime` (drop every buffered RX byte, every framed-but-unconsumed line, and the assembler's
  //    partial line). Without this, the ~50 lines the host already streamed before the `0x86` survive the stop in
  //    `RX_PIPE`/`LINE_QUEUE`; `line_assembler` would keep feeding them to this consumer, which would re-plan and
  //    execute them (the DRO keeps running) and `ok` each — spurious acks to a host that reset its window on the
  //    host-side stop, plus an `error:1` from a leaked partial line. This is done AFTER `quiesce_executor` returns:
  //    bytes already in flight on USB keep landing in `RX_PIPE` for the whole quiesce-await window (the reader half
  //    stays non-blocking), so flushing earlier would leave those late arrivals buffered. The host stops sending the
  //    moment it issues `0x86`, so the tail is finite and fully arrived by the parked acknowledgment — one flush here
  //    drops it cleanly. The `ResetPreempted` early-return above skips this deliberately: a soft reset winning the
  //    tie already ran the identical flush in the reader half, so there is nothing left to clear.
  RX_PIPE.clear();
  while LINE_QUEUE.try_receive().is_ok() {}
  LINE_RESET.signal(());
  // Finally, latch the control state back to Idle-capable `Normal` (NO alarm). `program_stop` maps Run/Hold → Normal
  // and is a no-op elsewhere; the reported `?` state re-derives Idle from the now-empty queue.
  set_control_state(control_state().program_stop());
}

/// Apply a coordinate-system / offset op to the shared [`COORDINATES`] model, then push the recomputed WCO into
/// the planner and (for the PERSISTENT ops) mark the coordinate record dirty. The "set to current position" ops
/// (G10 L20, G92, G28.1/G30.1) resolve against the planner's COMMANDED machine position (`position_mm`), which
/// is grbl's `gc_state.position` — the right reference for setting offsets, and race-free with the cross-core
/// live-position atomics. Inch words are scaled to mm here (the parser passes raw values in the active units).
async fn apply_coordinate_op(op: CoordinateOp) {
  // Snapshot the commanded machine position once (used by the set-to-position ops). A missing planner is an
  // init bug, unreachable in a wired build; treat it as the origin so the op still applies coherently.
  let machine = {
    let guard = PLANNER.lock().await;
    guard.as_ref().map(Planner::position_mm).unwrap_or([0.0; AXES])
  };
  // The live `$376` rotary mask, so a rotary-A coordinate word is stored as degrees (never inch-scaled) — matching
  // the planner's per-axis scale fork. Read once here for every `axis_values_mm` call in the match below.
  let rotary_mask = settings_snapshot().await.rotary_mask;
  let mut coords = coordinates();
  // Whether this op changes the PERSISTENT subset (G54-G59 / G28 / G30 / active WCS) and so must be flushed.
  // G92 and the dynamic TLO are session-only and never marked dirty.
  let mut persistent = false;
  match op {
    CoordinateOp::SelectWcs { index } => {
      coords.select_wcs(index);
      persistent = true;
    }
    CoordinateOp::SetWcsOffset { index, axes, units } => {
      let (values, present) = axis_values_mm(&axes, units, rotary_mask);
      coords.set_wcs_offset(index, values, present);
      persistent = true;
    }
    CoordinateOp::SetWcsOffsetToPosition { index, axes, units } => {
      let (values, present) = axis_values_mm(&axes, units, rotary_mask);
      coords.set_wcs_offset_to_position(index, machine, values, present);
      persistent = true;
    }
    CoordinateOp::SetG92ToPosition { axes, units } => {
      let (values, present) = axis_values_mm(&axes, units, rotary_mask);
      coords.set_g92_to_position(machine, values, present);
    }
    CoordinateOp::ClearG92 => coords.clear_g92(),
    CoordinateOp::StorePredefined { index } => {
      coords.store_predefined(index, machine);
      persistent = true;
    }
    CoordinateOp::ApplyTlo { z, units } => coords.apply_tlo(z * units_scale(units)),
    CoordinateOp::CancelTlo => coords.cancel_tlo(),
  }
  set_coordinates(coords);
  // The WCO may have changed (every op except a no-op select can shift it); push it into the planner so the
  // next absolute work move resolves correctly.
  push_wco_to_planner().await;
  if persistent {
    mark_coordinates_dirty();
  }
}

/// Keep the coordinate model's active WCS in sync with the parser's modal `wcs` for a move that shares a line
/// with a G54-G59 select (the parser emits the move, not a SelectWcs op, in that case). When they already match
/// this is a cheap no-op; on a change it re-selects, pushes the new WCO into the planner, and marks the
/// coordinate record dirty (the active WCS is part of the persistent subset).
async fn sync_active_wcs(parser_wcs: usize) {
  let coords = coordinates();
  if coords.active_wcs() == parser_wcs {
    return;
  }
  let mut updated = coords;
  updated.select_wcs(parser_wcs);
  set_coordinates(updated);
  push_wco_to_planner().await;
  mark_coordinates_dirty();
}

/// Resolve a line's [`AxisWords`] into an mm value array plus a per-axis "present" mask, scaling inch words to mm.
/// Absent axes carry `0.0` with `present = false` so a mutator writes only the mentioned axes. The full [`AXES`]
/// word set is read (X/Y/Z AND the rotary A) — omitting A previously indexed a 3-element array at axis 3 and
/// PANICKED on a G92/G10 L2/L20 line carrying an A word (`AXES == 4`). Per the DOC-10.1 rotary convention a word on
/// a ROTARY axis (per the live `$376` `rotary_mask`) is in DEGREES and is NEVER inch-scaled — a `G20 ... A90` is 90
/// degrees, not 90 × 25.4 — matching [`Planner::resolve_target`]'s per-axis scale fork, so a WCS/G92 offset on a
/// rotary A stores degrees. `rotary_mask` is the live `$376` value; bit N set marks axis N angular.
fn axis_values_mm(axes: &firmware_core::gcode::AxisWords, units: GcodeUnits, rotary_mask: u8) -> ([f32; AXES], [bool; AXES]) {
  let linear_scale = units_scale(units);
  let words = [axes.x, axes.y, axes.z, axes.a];
  let mut values = [0.0f32; AXES];
  let mut present = [false; AXES];
  for axis in 0..AXES {
    if let Some(value) = words[axis] {
      // A rotary axis word is degrees — never inch-scaled (its scale is 1.0); a linear word scales mm/inch.
      let scale = if rotary_mask & (1 << axis) != 0 { 1.0 } else { linear_scale };
      values[axis] = value * scale;
      present[axis] = true;
    }
  }
  (values, present)
}

/// The mm-per-unit scale for a [`GcodeUnits`] value (1 for mm, 25.4 for inch). Coordinate words arrive in the
/// active units; the coordinate model stores everything in mm, so the consumer scales at the boundary.
fn units_scale(units: GcodeUnits) -> f32 {
  match units {
    GcodeUnits::Millimeter => 1.0,
    GcodeUnits::Inch => firmware_core::planner::MM_PER_INCH,
  }
}

/// Answer a `$<rest>` system command, dispatched off the host-tested [`SystemCommand`] classifier so an
/// UNRECOGNIZED `$` command returns `error:3` instead of a fabricated `ok` (the Phase A hardening — the old
/// lenient fallthrough is gone). Every arm emits exactly one `ok`/`error:N` (multi-line reports end with the
/// `ok`), preserving the one-response-per-line contract. Real handlers: `$X` unlock, `$C` check toggle, `$SLP`
/// sleep, `$N`/`$N0=`/`$N1=` startup lines, `$H` homing (gated on `$22`, unimplemented per DOC-06), `$RST=$`
/// restore-defaults; the existing `$$`/`$I`/`$G`/`$#`/`$PBX`/`$x=val` paths are preserved.
async fn handle_system_command(rest: &[u8], parser: &mut Parser, state: &mut ConsumerState, flash: &'static SharedFlash) {
  match SystemCommand::classify(rest) {
    SystemCommand::Help => {
      // Bare `$` (help): emit the grbl help line, then `ok` — not a bare ack (the host's `$`-help probe wants
      // the documented one-liner).
      send_help().await;
      ack().await;
    }
    SystemCommand::SettingsDump => {
      // `$$` settings dump: one `$n=value` line per supported setting, then the terminating `ok`.
      dump_settings().await;
      ack().await;
    }
    SystemCommand::BuildInfo { extended } => {
      send_build_info(extended).await;
      ack().await;
    }
    SystemCommand::ParserState => {
      send_parser_state(parser).await;
      ack().await;
    }
    SystemCommand::NgcParams => {
      // `$#` NGC parameters dump (Phase B): emit `[G54:..]`..`[G59:..]`, `[G28:..]`, `[G30:..]`, `[G92:..]`,
      // `[TLO:z]`, and `[PRB:..]` from the live coordinate model, then the terminating `ok`.
      dump_ngc_parameters().await;
      ack().await;
    }
    SystemCommand::Unlock => handle_unlock().await,
    SystemCommand::ToggleCheck => handle_toggle_check(parser, state).await,
    SystemCommand::Sleep => handle_sleep().await,
    SystemCommand::Home => handle_home(parser, state).await,
    SystemCommand::StartupQuery => handle_startup_query(state).await,
    SystemCommand::StartupSet { index, gcode } => handle_startup_set(index, gcode, state).await,
    SystemCommand::RestoreSettings => handle_restore_settings(parser, state, flash).await,
    SystemCommand::RestoreParams => handle_restore_params(flash).await,
    SystemCommand::RestoreAll => handle_restore_all(parser, state, flash).await,
    // Phase F runtime enumerations: each streams its bracket lines through the single USB writer, then `ok`.
    SystemCommand::EnumSettings => {
      enumerate_settings().await;
      ack().await;
    }
    SystemCommand::EnumSettingGroups => {
      enumerate_setting_groups().await;
      ack().await;
    }
    SystemCommand::EnumErrorCodes => {
      enumerate_error_codes().await;
      ack().await;
    }
    SystemCommand::EnumAlarmCodes => {
      enumerate_alarm_codes().await;
      ack().await;
    }
    SystemCommand::SettingDescription { id } => {
      // `$SED=<n>`: emit the one `[SETTINGDESCR:]` line for a KNOWN setting, then `ok`. An unknown id emits no
      // bracket line but still `ok`s the (recognized) `$SED` command — grbl treats a description query for an
      // absent setting as an empty result, not an error (the command itself is valid).
      send_setting_description(id).await;
      ack().await;
    }
    SystemCommand::SetSetting { body } => write_setting_command(body).await,
    SystemCommand::PbExport => {
      // `$PBX` bulk settings export (DOC-04 host-sync): emit the live settings as hex-encoded protobuf frame
      // chunks, then the terminating `ok`. skirnir reassembles and decodes via the shared `galdr-proto` schema.
      export_pb().await;
      ack().await;
    }
    SystemCommand::PbImport { hex } => handle_pb_write(hex, state).await,
    SystemCommand::Unknown => {
      // The hardened fallthrough: an unrecognized `$` command returns `error:3` ("'$' command not
      // recognized"), NEVER a spurious `ok`. This closes the old lenient fake-ack hole.
      error(ERROR_UNSUPPORTED_COMMAND).await;
    }
  }
}

/// Handle `$X` (kill alarm lock). From a non-locked alarm: clear to Normal, emit `[MSG:Caution: Unlocked]`
/// then `ok`. From a non-alarm state: a no-op `ok`. From a LOCKED critical alarm (hard/soft limit, e-stop):
/// `error:N` — only a soft reset (after the physical cause clears) can unlock those. The control-state machine
/// (host-tested) decides which; this wiring just emits the matching response and publishes the new state.
async fn handle_unlock() {
  let (next, outcome) = control_state().unlock();
  set_control_state(next);
  match outcome {
    UnlockOutcome::Unlocked => {
      send_message("Caution: Unlocked").await;
      ack().await;
    }
    UnlockOutcome::NotAlarmed => ack().await,
    UnlockOutcome::Locked => error(ERROR_UNSUPPORTED_COMMAND).await,
  }
}

/// Handle `$C` (toggle check mode). Enter from Normal → `[MSG:Enabled]` + `ok`. Leave from Check, which grbl
/// realizes as a soft reset: emit `[MSG:Disabled]`, run the pipeline reset (rebuild parser/planner, flush the
/// planner, re-emit the banner) — which also publishes the post-reset control state and `ok`s nothing — then
/// `ok`. Rejected (not Normal/Check) → `error:N`. The new control state is published before the side-effects.
async fn handle_toggle_check(parser: &mut Parser, state: &mut ConsumerState) {
  let (next, toggle) = control_state().toggle_check(HOMING_ENABLED.load(Ordering::Relaxed));
  match toggle {
    CheckToggle::Enabled => {
      set_control_state(next);
      send_message("Enabled").await;
      ack().await;
    }
    CheckToggle::Disabled => {
      // Leaving check mode is a soft reset per grbl: emit `[MSG:Disabled]`, then rebuild the pipeline. The
      // control state is set to the post-reset boot value (computed by `toggle_check`) here so the executor
      // reset and the published state agree; `reset_pipeline` emits the banner and re-arms the parser/planner.
      send_message("Disabled").await;
      set_control_state(next);
      reset_pipeline(parser, state).await;
    }
    CheckToggle::Rejected => error(ERROR_UNSUPPORTED_COMMAND).await,
  }
}

/// Handle `$SLP` (sleep). Allowed only from Normal: publish the Sleep state, signal the core-1 executor to
/// hold (a sleeping machine must not run queued blocks), and `ok`. The actual spindle/coolant shutdown and
/// driver de-energize are a HARDWARE BOUNDARY owned by DOC-07 (spindle) / DOC-03 (TMC STEP_EN) and are
/// stubbed here — flagged in the report. Wake is by a soft reset (`0x18`), handled by the existing reset path.
async fn handle_sleep() {
  let (next, entered) = control_state().enter_sleep();
  if entered {
    set_control_state(next);
    // Hold the executor at the next block boundary so no further blocks run while asleep: RAISE the hold LEVEL
    // (Finding #11) and wake the executor. A `~` will NOT release it — `resumes_on_cycle_start` is false in
    // Sleep, so the `~` dispatch leaves the level set (Finding #1) — and only a soft reset wakes the machine
    // (which clears the level and zeroes the executor via the reset path). Reusing the same level a feed-hold
    // uses keeps one parking mechanism for both.
    HOLD_REQUESTED.store(true, Ordering::Release);
    HOLD_WAKE.signal(());
    // DOC-07: `$SLP` stops the spindle (LEDC duty → 0, SPIN_EN de-asserted) AND coolant, independent of motion
    // state. The TMC driver de-energize (STEP_EN high) remains a DOC-03 follow-up, flagged in the report.
    force_spindle_off();
    force_coolant_off();
    ack().await;
  } else {
    // Sleep is rejected from any non-Normal state (alarm/check/already asleep), matching grbl.
    error(ERROR_UNSUPPORTED_COMMAND).await;
  }
}

/// Handle `$H` (run the homing cycle, DOC-06). Sequence:
/// 1. If `$22` homing is disabled → `error:5` ("Homing cycle is not enabled"), no motion.
/// 2. If the control state does not allow homing (a locked hard/soft-limit/e-stop alarm, a hold, check, or
///    sleep) → `error:9` (G-code lock); `$H` runs from Idle/Normal and from the homing-required boot lock only.
/// 3. Publish the pre-cycle `<Home>` State (set [`HOMING_ACTIVE`] so `?` reports `Home` and the hard-limit
///    monitor is suppressed) and emit one `<Home|...>` report, mirroring grbl's pre-cycle push.
/// 4. Dispatch the resolved [`HomingConfig`] to the core-1 executor (which owns the RMT channels + limit
///    inputs) via [`HOME_REQUEST`] and AWAIT [`HOME_RESULT`], racing a soft reset so a `0x18` mid-cycle aborts.
/// 5. On SUCCESS: sync the planner's commanded position to the established machine zero, clear `ALARM:11` to
///    Normal via [`ControlState::home_complete`], and `ok`. On a homing FAIL (no contact) or sink abort: raise
///    `ALARM:8` (homing fail) + its prompt and force a pipeline reset — never fabricate a homed state.
async fn handle_home(parser: &mut Parser, state: &mut ConsumerState) {
  if !HOMING_ENABLED.load(Ordering::Relaxed) {
    error(ERROR_HOMING_DISABLED).await;
    return;
  }
  // grbl gates `$H`: it runs from Idle/Normal and from the homing-required boot lock, but never from a locked
  // critical alarm, a feed-hold, check, or sleep. The host-tested predicate decides; reject with the lock code.
  if !control_state().homing_allowed() {
    error(ERROR_LOCKED).await;
    return;
  }

  // Build the resolved homing config from the live settings on core 0 (only core 0 reads SETTINGS), so the
  // core-1 executor runs the cycle without touching the async settings mutex — mirroring the probe dispatch.
  let config = settings_snapshot().await.homing_config(crate::MOTION_TICK_HZ);

  // Pre-cycle `<Home>` push (research finding #1): mark homing active so `?` reports `Home` and the hard-limit
  // alarm path is suppressed for the cycle's duration, then ask the status responder to emit one report before
  // the motion begins (it composes the `Home` State from `HOMING_ACTIVE`, set just above).
  HOMING_ACTIVE.store(true, Ordering::Release);
  STATUS_REQUEST.signal(());

  let result = run_homing_cycle(config).await;
  HOMING_ACTIVE.store(false, Ordering::Release);

  match result {
    Some(Ok(zero_steps)) => {
      // The executor reported a clean cycle, but the control state can have been CLOBBERED to a locked alarm in
      // the post-cycle window — `HOMING_ACTIVE` was cleared above, so a limit still engaged when the idle
      // limit-monitor (finding #1) re-samples, or any other `0x18`/alarm path, can flip `CONTROL` to
      // `Alarm(..)` across the `PLANNER.lock().await` below. `home_complete()` is a no-op from a non-`Normal`able
      // state, so it would leave the alarm in place — but unconditionally acking + marking homed would emit a
      // spurious `ok` and a FALSE `HOMED` over a real alarm (finding #2). So we ACT only when the transition
      // genuinely reached `Normal`: sync the planner, mark homed, and `ok`. Otherwise we leave the alarm
      // untouched and emit no `ok` — the alarm's own path already surfaced `ALARM:N` to the host.
      let before = control_state();
      let after = before.home_complete();
      if after == ControlState::Normal {
        // Position is established: sync the planner's commanded position to the machine zero (grbl's
        // `plan_sync_position` + `gc_sync_position`), clear the homing-required alarm to Normal, and `ok`.
        {
          let mut guard = PLANNER.lock().await;
          if let Some(planner) = guard.as_mut() {
            planner.sync_position(zero_steps);
          }
        }
        set_control_state(after);
        // Mark the machine homed so `$20` soft limits become active (research finding #16). A subsequent soft
        // reset / `$X` that loses certainty clears this in `apply_soft_reset`.
        HOMED.store(true, Ordering::Relaxed);
        ack().await;
      }
      // else: the state was clobbered to an alarm during the success window — leave it locked, emit no `ok`, and
      // do NOT mark homed. The clobbering path (e.g. the hard-limit monitor) owns reporting + the pipeline reset.
    }
    Some(Err(_)) => {
      // Homing failed (no switch contact within 1.5× travel, or a sink abort): position is unknown. Raise
      // `ALARM:8` (homing fail) and force a pipeline reset so the machine is in a clean, clearly-unhomed alarm
      // state — never an `ok`. grbl emits no `ok` for a failed homing cycle.
      set_control_state(ControlState::Alarm(AlarmCode::HomingFail));
      emit_alarm(AlarmCode::HomingFail).await;
      reset_pipeline(parser, state).await;
    }
    // A soft reset preempted the cycle: honor the consumed reset (rebuild the pipeline, emit the banner). The
    // executor's own reset path zeroes the live position; `apply_soft_reset` publishes the post-reset state.
    None => apply_soft_reset(parser, state).await,
  }
}

/// Dispatch a homing cycle to the core-1 executor and AWAIT its result, racing a soft reset (DOC-06). Mirrors
/// [`run_probe_cycle`]: drains any stale result, signals [`HOME_REQUEST`] with the resolved config, then waits
/// on [`HOME_RESULT`] vs [`SOFT_RESET`]. Returns `Some(result)` on completion, or `None` if a `0x18` landed
/// mid-cycle (the caller honors the consumed reset signal). Draining a still-latched request on the reset path
/// prevents an unrequested homing move from running after the pipeline rebuilds.
async fn run_homing_cycle(config: HomingConfig) -> Option<Result<[i32; AXES], HomingError>> {
  HOME_RESULT.try_take();
  HOME_REQUEST.signal(config);
  match select(HOME_RESULT.wait(), SOFT_RESET.wait()).await {
    Either::First(result) => Some(result),
    Either::Second(()) => {
      // Drain the request we just signalled in case the executor had not yet consumed it, so a still-latched
      // request cannot run an unrequested homing move from the freshly-zeroed origin after the reset.
      HOME_REQUEST.try_take();
      None
    }
  }
}

/// Handle `$N` (query stored startup lines): echo both slots as `$N0=...`/`$N1=...` (empty when unset), then
/// `ok`. Phase A stores and echoes startup lines but does NOT execute them on reset (see [`handle_startup_set`]).
async fn handle_startup_query(state: &ConsumerState) {
  for (index, slot) in state.startup_lines.iter().enumerate() {
    let mut line = Response::new();
    let gcode: &[u8] = slot.as_deref().unwrap_or(&[]);
    // `$Nn=<gcode>` — the stored line is valid UTF-8 by construction (it was a received GCode line). A
    // formatting/capacity failure simply skips the echo line; the closing `ok` still terminates the response.
    if write_startup_echo(index as u8, gcode, &mut line) {
      enqueue(line).await;
    }
  }
  ack().await;
}

/// Render a `$Nn=<gcode>` startup-line echo (CRLF-terminated) into `out`. Returns `false` only on a capacity
/// failure (unreachable with the caller's correctly sized buffer). The GCode bytes are ASCII (a received line).
fn write_startup_echo(index: u8, gcode: &[u8], out: &mut Response) -> bool {
  use core::fmt::Write as _;
  if write!(out, "$N{index}=").is_err() {
    return false;
  }
  // Append the raw GCode bytes as chars; they are 7-bit ASCII from the line framer, so this never mis-encodes.
  for &b in gcode {
    if out.push(b as char).is_err() {
      return false;
    }
  }
  out.push_str("\r\n").is_ok()
}

/// Handle `$N0=<gcode>` / `$N1=<gcode>` (store a startup line). The line is stored in RAM and `ok`'d. Phase A
/// SCOPE: startup lines are STORED and ECHOED (`$N`) but NOT executed on reset — running them at init is a
/// deliberate follow-up (it must re-establish modal state safely and is gated on the homing/alarm-exit rules).
/// PERSISTENCE BOUNDARY (TODO DOC-04): the lines are held only in RAM. Persisting them needs a separate NVS
/// record or a non-scalar proto field (which would break `Settings: Copy`), both larger than Phase A.
async fn handle_startup_set(index: u8, gcode: &[u8], state: &mut ConsumerState) {
  let slot = index as usize;
  if slot >= state.startup_lines.len() {
    // `classify` only emits index 0/1, so this is unreachable; reject defensively rather than panic-index.
    error(ERROR_UNSUPPORTED_COMMAND).await;
    return;
  }
  if gcode.is_empty() {
    // `$Nn=` with empty body clears the slot (grbl behavior).
    state.startup_lines[slot] = None;
    ack().await;
    return;
  }
  let mut stored = heapless::Vec::new();
  if stored.extend_from_slice(gcode).is_err() {
    // The startup line exceeds the line buffer — reject rather than truncate.
    error(ERROR_UNSUPPORTED_COMMAND).await;
    return;
  }
  state.startup_lines[slot] = Some(stored);
  ack().await;
}

/// Handle `$RST=$` / `$RST=*` (restore default settings). Replace the live [`SETTINGS`] with the compiled
/// defaults, persist them to flash, clear the stored startup lines, then soft-reset the pipeline so the
/// defaults take effect (grbl auto-resets after a `$RST`). `$RST=*` additionally clears startup lines / build
/// info — Phase A clears the startup lines for both; the build-info string is compiled, not stored, so there
/// is nothing further to clear. The terminating banner comes from `reset_pipeline`.
async fn handle_restore_settings(parser: &mut Parser, state: &mut ConsumerState, flash: &'static SharedFlash) {
  let defaults = Settings::default();
  {
    let mut guard = SETTINGS.lock().await;
    *guard = Some(defaults);
  }
  // Persist immediately (a restore is an explicit, infrequent operation, so the coalesced-flush deferral is
  // not needed): write the defaults straight to flash. A failed persist is logged and swallowed — the in-RAM
  // defaults already apply — matching the existing best-effort store policy.
  {
    let mut store = FlashRecordStore::settings(flash);
    if settings::store_settings(&mut store, &defaults).await.is_err() {
      #[cfg(feature = "defmt")]
      defmt::warn!("settings: failed to persist restored defaults");
    }
  }
  SETTINGS_DIRTY.store(false, Ordering::Release);
  // Refresh the cached status config from the restored defaults so the next report reflects them (Finding #14).
  refresh_status_cfg().await;
  // Clear the stored startup lines (`$RST=$`/`$RST=*` both drop them in grbl) and update the homing mirror so
  // the post-reset boot state is computed from the restored `$22` (default: disabled → Normal).
  state.startup_lines = [None, None];
  HOMING_ENABLED.store(defaults.homing_enabled(), Ordering::Relaxed);
  HARD_LIMITS_ENABLED.store(defaults.hard_limits_enabled(), Ordering::Relaxed);
  LIMIT_INVERT.store(defaults.limit_invert, Ordering::Relaxed);
  LIMIT_DEBOUNCE_MS.store(defaults.homing_debounce_ms, Ordering::Relaxed);
  set_control_state(ControlState::boot(defaults.homing_enabled()));
  // Soft-reset the pipeline so the restored settings take effect (it rebuilds the planner from the live
  // settings and emits the banner). It does not emit `ok`; grbl's `$RST` is acknowledged by the reset itself.
  reset_pipeline(parser, state).await;
}

/// Queue the bare-`$` help line.
async fn send_help() {
  let mut s = Response::new();
  if ResponseWriter::help(&mut s).is_ok() {
    enqueue(s).await;
  }
}

/// Queue a `[MSG:<text>]` push message (e.g. `[MSG:Caution: Unlocked]`, `[MSG:Enabled]`).
async fn send_message(text: &str) {
  let mut s = Response::new();
  if ResponseWriter::message(&mut s, text).is_ok() {
    enqueue(s).await;
  }
}

/// Number of frame BYTES hex-encoded per `[PB:...]` export line. 48 bytes → 96 hex chars; with the `[PB:`/`]`
/// wrapper and CRLF that is ~103 chars, comfortably under [`RESPONSE_CAPACITY`].
const PB_CHUNK_BYTES: usize = 48;

/// Emit the `$PBX` bulk export: encode the live settings into a storage frame and stream it as hex-encoded
/// `[PB:<hex>]` lines (chunked to fit the line length). The terminating `ok` is emitted by the caller; the
/// host concatenates the chunk payloads in order, hex-decodes, and decodes the frame with the shared schema.
async fn export_pb() {
  let snapshot = settings_snapshot().await;
  let mut frame: heapless::Vec<u8, { settings::wire::FRAME_MAX_LEN }> = heapless::Vec::new();
  // Encoding the live settings cannot fail (the buffer is sized for the largest frame); on the impossible
  // error, emit nothing and let the caller's `ok` close the (empty) export so the host can retry.
  if settings::wire::encode(&snapshot, &mut frame).is_err() {
    return;
  }
  for chunk in frame.chunks(PB_CHUNK_BYTES) {
    let mut line = Response::new();
    if line.push_str("[PB:").is_ok() && settings::write_hex(chunk, &mut line) && line.push_str("]\r\n").is_ok() {
      enqueue(line).await;
    }
  }
}

/// Handle one `$PBX=<hex>` import chunk: feed it to the reassembly receiver. A non-final chunk acknowledges
/// and waits; the final chunk completes the frame, which is applied to the live [`SETTINGS`] and marked DIRTY
/// (the coalesced flush persists it once the burst drains — Finding #14b) before acknowledging; malformed
/// input resets the receiver and returns `error:N` so the host can retry from the first chunk.
async fn handle_pb_write(hex: &[u8], state: &mut ConsumerState) {
  let Ok(hex) = core::str::from_utf8(hex) else {
    state.pb.reset();
    error(SettingError::BadValue.code()).await;
    return;
  };
  match state.pb.accept_hex(hex.trim()) {
    PbChunkResult::NeedMore => ack().await,
    PbChunkResult::Complete(new_settings) => {
      {
        let mut guard = SETTINGS.lock().await;
        *guard = Some(new_settings);
      }
      // Apply to RAM and mark dirty; the consumer flushes once the burst drains (or on soft reset / safety
      // interval), so a `$PBX` import that arrives split across many lines is persisted with a SINGLE flash
      // append rather than one per chunk. Planner-affecting fields take effect on the next soft reset, as with
      // `$x=val`. Keep the `$22` homing mirror in sync so the boot-lock / `$H` / soft-reset decisions reflect a
      // bulk import that changed it. Acknowledge immediately — the in-RAM value already applied.
      HOMING_ENABLED.store(new_settings.homing_enabled(), Ordering::Relaxed);
      // Keep the `$21` hard-limit-enable mirror in sync too so the executor's hard-limit check reflects a bulk
      // import that changed it (DOC-06).
      HARD_LIMITS_ENABLED.store(new_settings.hard_limits_enabled(), Ordering::Relaxed);
      LIMIT_INVERT.store(new_settings.limit_invert, Ordering::Relaxed);
      // Keep the `$26` debounce mirror in sync so a bulk import that retunes it takes effect on the executor's
      // limit-edge resample without a reboot (DOC-06).
      LIMIT_DEBOUNCE_MS.store(new_settings.homing_debounce_ms, Ordering::Relaxed);
      // Phase F: a bulk import can change `$481`; re-seed the live auto-report cadence and wake the task so the
      // imported interval (enable/disable/retune) takes effect immediately, no reboot.
      AUTO_REPORT_INTERVAL_MS.store(new_settings.auto_report_interval_ms(), Ordering::Relaxed);
      AUTO_REPORT_WAKE.signal(());
      // A bulk import can change `$100`/`$10`/`$110`; refresh the cached status config (Finding #14).
      refresh_status_cfg().await;
      mark_settings_dirty();
      ack().await;
    }
    PbChunkResult::Error => error(SettingError::BadValue.code()).await,
  }
}

/// Emit the `$#` NGC-parameters block: one bracket line (`[G54:..]`..`[PRB:..]`) per index, rendered from a
/// [`CoordinateReport`] snapshot of the live coordinate model. The closing `ok` is emitted by the caller. Each
/// line is enqueued through the single USB writer, preserving in-order delivery (exactly the `$$` pattern).
async fn dump_ngc_parameters() {
  let report = coordinate_report();
  for index in 0..NGC_PARAMETER_LINES {
    let mut line = Response::new();
    if ResponseWriter::ngc_parameter_line(&mut line, &report, index) {
      enqueue(line).await;
    }
  }
}

/// Build the [`CoordinateReport`] the `$#` formatter renders from the live coordinate model and the last-probe
/// result (Phase C). The `[PRB:]` line now carries the real triggered machine position and contact flag from
/// [`LAST_PROBE`]; it is `[PRB:0,0,0:0]` (the never-probed default) until the first `G38.x` cycle runs.
fn coordinate_report() -> CoordinateReport {
  let coords = coordinates();
  let mut wcs = [[0.0f32; AXES]; 6];
  for (index, slot) in wcs.iter_mut().enumerate() {
    if let Some(offset) = coords.wcs_offset(index) {
      *slot = offset;
    }
  }
  let predefined = [
    coords.predefined(0).unwrap_or([0.0; AXES]),
    coords.predefined(1).unwrap_or([0.0; AXES]),
  ];
  // Phase C: the `[PRB:]` line reflects the real last-probe result (machine position + contact flag), replacing
  // the Phase-B zeros/flag-0 stub. It is `[PRB:0,0,0:0]` until the first probe runs (the never-probed default).
  let probe = last_probe();
  CoordinateReport {
    wcs,
    predefined,
    g92: coords.g92_offset(),
    tlo: coords.tlo(),
    probe: probe.position_mm,
    probe_success: probe.success,
  }
}

/// Handle `$RST=#` (zero coordinate data). Clear ALL coordinate offsets — G54-G59 → 0, G28/G30 → 0, G92/TLO →
/// identity, active WCS → G54 — persist the (now-default) persistent record immediately, push the zero WCO into
/// the planner, reset the WCO refresh cadence so the next `?` re-emits `WCO:`, and `ok`. grbl auto-resets after
/// a `$RST`; coordinate data has no planner-modal coupling beyond the WCO, so a full soft reset is not required
/// here (mirroring how a `$RST=#` only touches parameters, not `$$` settings or modal state).
async fn handle_restore_params(flash: &'static SharedFlash) {
  clear_and_persist_coordinates(flash).await;
  ack().await;
}

/// Clear ALL coordinate offsets and persist the cleared (default) record immediately, then push the zero WCO
/// into the planner and reset the `WCO:` refresh cadence. Shared by `$RST=#` ([`handle_restore_params`]) and
/// `$RST=*` ([`handle_restore_all`], Finding #7) so a "restore all" wipes G54-G59 / G28 / G30 exactly as
/// `$RST=#` does (the `$RST=*` doc says it should), rather than touching only the `$$` settings. Emits NO `ok`:
/// the caller decides the acknowledgment (`$RST=#` acks directly; `$RST=*` is acked by its soft reset).
async fn clear_and_persist_coordinates(flash: &'static SharedFlash) {
  let mut coords = coordinates();
  coords.clear_all();
  set_coordinates(coords);
  push_wco_to_planner().await;
  // Persist the cleared (default) record immediately — a restore is explicit and infrequent, so the coalesced
  // deferral is unnecessary. A failed persist is logged and swallowed (the in-RAM value already applies),
  // matching the settings restore policy.
  {
    let mut store = FlashRecordStore::coordinates(flash);
    if coords::store_coordinates(&mut store, &coords.persistent()).await.is_err() {
      #[cfg(feature = "defmt")]
      defmt::warn!("coordinates: failed to persist cleared params");
    }
  }
  COORDINATES_DIRTY.store(false, Ordering::Release);
  reset_wco_reporter();
}

/// Handle `$RST=*` (restore EVERYTHING): restore the `$$` settings to defaults AND clear+persist the coordinate
/// parameters (G54-G59 / G28 / G30), so a "restore all" wipes both — its own grblHAL doc says it should, and the
/// old routing wiped only the settings (Finding #7). The coordinate clear is reused from the `$RST=#` path. The
/// settings restore runs the soft-reset pipeline rebuild (which is the acknowledgment, as for `$RST=$`), so the
/// coordinates are cleared FIRST and the reset's `push_wco_to_planner` then resolves against the now-zero WCS.
async fn handle_restore_all(parser: &mut Parser, state: &mut ConsumerState, flash: &'static SharedFlash) {
  clear_and_persist_coordinates(flash).await;
  // Restore the `$$` settings to defaults and rebuild the pipeline (its banner is the `$RST=*` acknowledgment).
  // `handle_restore_settings` re-pushes the WCO into the freshly-rebuilt planner via `reset_pipeline`, so the
  // zeroed coordinate model is what the post-restore planner uses.
  handle_restore_settings(parser, state, flash).await;
}

/// Persist the live PERSISTENT coordinate subset (G54-G59 / G28 / G30 / active WCS) to flash IF a change is
/// pending, clearing [`COORDINATES_DIRTY`]. The coordinate analogue of [`flush_settings`]: callers mark dirty
/// per persistent op, and this performs the actual flash append once per burst (queue-empty), on the safety
/// interval, and on soft reset. The dirty flag is cleared BEFORE the write so a change landing during the await
/// re-marks dirty and is caught by the next flush. On write FAILURE we re-mark dirty when `retry_on_failure` is
/// set (Bug A) so the change is retried, mirroring [`flush_settings`]: the burst-boundary caller passes `false`
/// to avoid spinning, while the safety-timer and soft-reset arms pass `true` for a bounded retry.
async fn flush_coordinates(flash: &'static SharedFlash, retry_on_failure: bool) {
  if !COORDINATES_DIRTY.swap(false, Ordering::AcqRel) {
    return;
  }
  let persistent = coordinates().persistent();
  let mut store = FlashRecordStore::coordinates(flash);
  if coords::store_coordinates(&mut store, &persistent).await.is_err() {
    #[cfg(feature = "defmt")]
    defmt::warn!("coordinates: failed to flush coordinate record to flash");
    if retry_on_failure {
      COORDINATES_DIRTY.store(true, Ordering::Release);
    }
  }
}

/// Reset the `WCO:` refresh cadence so the next status report re-emits the `WCO:` element (grbl's "first report
/// after a reset includes WCO"). Called on a soft reset and after `$RST=#`.
fn reset_wco_reporter() {
  WCO_REPORTER.lock(|c| {
    let mut reporter = c.get();
    reporter.reset([0.0; AXES]);
    c.set(reporter);
  });
}

/// Reset the `Ov:` refresh cadence so the next status report re-emits the `Ov:` element (grbl's "first report
/// after a reset re-emits the change-only elements"). Called on a soft reset, after the overrides are reset.
fn reset_ov_reporter() {
  OV_REPORTER.lock(|c| {
    let mut reporter = c.get();
    reporter.reset(Overrides::new());
    c.set(reporter);
  });
}

/// Emit the `$$` settings dump: one `$n=value` line (CRLF-terminated) per number in
/// [`settings::SETTING_NUMBERS`], rendered from a snapshot of the live settings. The closing `ok` is emitted
/// by the caller. Each line is enqueued through the single USB writer, preserving in-order delivery.
async fn dump_settings() {
  let snapshot = settings_snapshot().await;
  for &n in settings::SETTING_NUMBERS {
    let mut line = Response::new();
    // Render `$n=value`, then append the line terminator; a formatting/capacity failure simply skips the line
    // (it never trips — `Response` is sized well above the longest setting line).
    if snapshot.write_setting_line(n, &mut line) && line.push_str("\r\n").is_ok() {
      enqueue(line).await;
    }
  }
}

/// Emit the `$ES` settings enumeration: one `[SETTING:<id>|<group>|<name>|<unit>|<datatype>|<format>|<min>|
/// <max>]` line per `$n` number (Phase F), driven off the SAME [`settings::SETTING_NUMBERS`] authority `$$`
/// uses, so the enumeration and the live dump describe the same settings. The closing `ok` is the caller's.
/// Each line is built in its own [`Response`] and enqueued through the single USB writer with blocking `send`
/// (back-pressure), never assembled into one giant buffer — the block can be long over the 1024-byte pipe.
async fn enumerate_settings() {
  for &n in settings::SETTING_NUMBERS {
    let mut line = Response::new();
    if Settings::write_setting_enumeration(n, &mut line) {
      enqueue(line).await;
    }
  }
}

/// Emit the `$EG` setting-group enumeration: one `[SETTINGGROUP:<id>|<parent>|<name>]` line per group (Phase
/// F), covering every group referenced by a setting's metadata. The closing `ok` is the caller's; each line is
/// enqueued through the single USB writer with back-pressure.
async fn enumerate_setting_groups() {
  for index in 0..settings::SETTING_GROUP_COUNT {
    let mut line = Response::new();
    if settings::write_setting_group(index, &mut line) {
      enqueue(line).await;
    }
  }
}

/// Emit the `$EE` error-code enumeration: one `[ERRORCODE:<id>|<name>|<description>]` line per code in the
/// single [`ERROR_CODES`] authority (Phase F) — every `error:N` this firmware can return. The closing `ok` is
/// the caller's; each line is enqueued through the single USB writer with back-pressure.
async fn enumerate_error_codes() {
  for code in ERROR_CODES {
    let mut line = Response::new();
    if ResponseWriter::error_code_line(&mut line, code).is_ok() {
      enqueue(line).await;
    }
  }
}

/// Emit the `$EA` alarm-code enumeration: one `[ALARMCODE:<id>|<name>|<description>]` line per code in
/// [`AlarmCode::ALL`] (Phase F) — every alarm this firmware can raise. The closing `ok` is the caller's; each
/// line is enqueued through the single USB writer with back-pressure.
async fn enumerate_alarm_codes() {
  for &code in AlarmCode::ALL {
    let mut line = Response::new();
    if ResponseWriter::alarm_code_line(&mut line, code).is_ok() {
      enqueue(line).await;
    }
  }
}

/// Emit the single `[SETTINGDESCR:<id>|<description>]` line for `$SED=<id>` (Phase F) when `id` is a known
/// setting; a no-op for an unknown id (the caller still `ok`s the recognized `$SED` command). Enqueued through
/// the single USB writer.
async fn send_setting_description(id: u16) {
  let mut line = Response::new();
  if Settings::write_setting_description(id, &mut line) {
    enqueue(line).await;
  }
}

/// Handle a `$<number>=<value>` setting write. The [`SystemCommand`] classifier already guaranteed the body
/// is `<digits>=<value>`, so the only failures here are a value out of range / wrong type (→ `error:N`) — a
/// malformed body NEVER reaches this path (it classifies to `Unknown` → `error:3`), so there is no lenient
/// fake-ack. A valid write is applied to the live [`SETTINGS`] and marked DIRTY for the coalesced flush; the
/// flash append is deferred to the consumer's burst-boundary flush, so a `$$`-bulk restore is one flash write.
async fn write_setting_command(body: &[u8]) {
  // The classifier ensured `<digits>=<value>`; re-parse to extract the number/value for the setter. A parse
  // failure here would be an internal inconsistency with the classifier, surfaced as a bad-value error rather
  // than a fabricated `ok` (it cannot occur for a body the classifier accepted).
  let parsed = core::str::from_utf8(body)
    .ok()
    .and_then(|text| text.split_once('='))
    .and_then(|(number, value)| number.trim().parse::<u16>().ok().map(|n| (n, value.trim())));
  let Some((n, value)) = parsed else {
    error(SettingError::BadValue.code()).await;
    return;
  };

  // Apply under the lock; the validated change goes to the in-RAM settings, the flush persists it later.
  let outcome = {
    let mut guard = SETTINGS.lock().await;
    match guard.as_mut() {
      Some(settings) => settings.set_command(n, value),
      // Unseeded settings is an init wiring bug (unreachable after boot); reject as an unknown setting.
      None => Err(SettingError::UnknownSetting),
    }
  };

  match outcome {
    Ok(()) => {
      // Apply-and-mark: the in-RAM value already applied, so acknowledge immediately. The coalesced flush
      // persists it once the burst drains (or on soft reset / safety interval). Deferring avoids re-appending
      // the WHOLE settings blob to flash for every `$n=val` line in a bulk restore (grbl semantics — a
      // stalled `ok` would wedge a character-counting sender; here the `ok` never waits on flash at all). Keep
      // the `$22` homing mirror in sync so a `$22=` write changes the boot-lock / `$H` / soft-reset decisions.
      if n == 22 {
        HOMING_ENABLED.store(settings_snapshot().await.homing_enabled(), Ordering::Relaxed);
      }
      // A `$21=` write changes the hard-limit enable live; mirror it so the executor's hard-limit check reflects
      // it without a reboot (DOC-06).
      if n == 21 {
        HARD_LIMITS_ENABLED.store(settings_snapshot().await.hard_limits_enabled(), Ordering::Relaxed);
      }
      // A `$5=` write changes the limit-pin invert live; mirror it for the executor's sampling (DOC-06).
      if n == 5 {
        LIMIT_INVERT.store(settings_snapshot().await.limit_invert, Ordering::Relaxed);
      }
      // A `$26=` write retunes the limit debounce live; mirror it for the executor's edge resample (DOC-06).
      if n == 26 {
        LIMIT_DEBOUNCE_MS.store(settings_snapshot().await.homing_debounce_ms, Ordering::Relaxed);
      }
      // Phase F: a `$481=` write retunes the auto-report cadence live (no reboot). Mirror the CLAMPED interval and
      // wake the auto-report task so an enable takes effect immediately and a disable/retune is picked up at once.
      if n == 481 {
        let interval = settings_snapshot().await.auto_report_interval_ms();
        AUTO_REPORT_INTERVAL_MS.store(interval, Ordering::Relaxed);
        AUTO_REPORT_WAKE.signal(());
      }
      // Refresh the cached status config so a `$100`/`$10`/`$110` change is in the next report (Finding #14).
      refresh_status_cfg().await;
      mark_settings_dirty();
      ack().await;
    }
    Err(error_code) => error(error_code.code()).await,
  }
}

/// Queue the `$I`/`$I+` build-info lines. Also REPLAYS a pending crash report (`[MSG:CRASH ...]`) here: `$I` is
/// the readiness probe a host sends right after connecting, so a sender that reconnected too late to catch the
/// boot-time emission still receives the post-mortem breadcrumb. Consumed (sent at most once more).
async fn send_build_info(extended: bool) {
  let mut s = Response::new();
  if ResponseWriter::build_info(&mut s, extended).is_ok() {
    enqueue(s).await;
  }
  if let Some(report) = take_pending_crash_report() {
    enqueue(report).await;
  }
}

/// Queue the `$G` parser-state line, rendered from the consumer's live parser modal state so the host sees
/// the real motion/units/distance/feed/spindle words rather than a constant default.
async fn send_parser_state(parser: &Parser) {
  let mut s = Response::new();
  if ResponseWriter::parser_state(&mut s, &parser_snapshot(parser.state())).is_ok() {
    enqueue(s).await;
  }
}

/// Translate the gcode parser's [`ModalState`] into the protocol layer's [`ParserSnapshot`] for `$G`
/// formatting. This is the one place the firmware bin bridges the parser's modal enums to the protocol's
/// rendering enums, keeping `firmware-core::protocol` free of any GCode-parsing coupling.
fn parser_snapshot(state: &ModalState) -> ParserSnapshot {
  ParserSnapshot {
    motion: match state.motion {
      MotionMode::Rapid => ParserMotion::Rapid,
      MotionMode::Linear => ParserMotion::Linear,
      MotionMode::ArcCw => ParserMotion::ArcCw,
      MotionMode::ArcCcw => ParserMotion::ArcCcw,
    },
    units: match state.units {
      GcodeUnits::Inch => ParserUnits::Inch,
      GcodeUnits::Millimeter => ParserUnits::Millimeter,
    },
    distance: match state.distance {
      GcodeDistance::Absolute => ParserDistance::Absolute,
      GcodeDistance::Incremental => ParserDistance::Incremental,
    },
    feed_mode: match state.feed_mode {
      GcodeFeedMode::InverseTime => ParserFeedMode::InverseTime,
      GcodeFeedMode::UnitsPerMin => ParserFeedMode::UnitsPerMin,
    },
    wcs: state.wcs,
    tlo_active: state.tlo_active,
    feed: state.feed,
    spindle: match state.spindle {
      SpindleState::Clockwise => ParserSpindle::Clockwise,
      SpindleState::CounterClockwise => ParserSpindle::CounterClockwise,
      SpindleState::Stop => ParserSpindle::Stop,
    },
    // The parser tracks spindle speed as f32 RPM; the snapshot reports whole RPM (grbl's `$G` S word).
    spindle_rpm: state.spindle_speed.max(0.0) as u16,
    plane: match state.plane {
      firmware_core::gcode::Plane::XY => ParserPlane::XY,
      firmware_core::gcode::Plane::ZX => ParserPlane::ZX,
      firmware_core::gcode::Plane::YZ => ParserPlane::YZ,
    },
    coolant: ParserCoolant { mist: state.coolant.mist, flood: state.coolant.flood },
    // The CURRENT (active) tool, committed by M6; reported as `T<n>` in `$G` (`T0` = none).
    tool: state.current_tool,
  }
}

/// Queue a single `ok`.
async fn ack() {
  let mut s = Response::new();
  if ResponseWriter::ok(&mut s).is_ok() {
    enqueue(s).await;
  }
}

/// Queue a single `error:N` for a rejected line. Mirrors [`ack`]; the consumer emits exactly one of the
/// two per consumed GCode line, preserving the one-response-per-line contract. A `[MSG:error:N <name>]`
/// context push is emitted FIRST so a plain terminal sees what the rejection means; it is a push message
/// (a sender that decodes the code itself ignores it) and the byte-exact `error:N` remains the sole
/// flow-control response. The context is skipped for a code with no enumerated name (empty buffer).
///
/// Use this only when the code's NAME genuinely describes the failure (parser/planner/validation errors,
/// and the `ERROR_LOCKED` state lockout). For a code reused purely to halt the sender — where its name does
/// NOT describe the cause — use [`error_bare`] so the annotation does not mislead.
async fn error(code: u8) {
  let mut ctx = Response::new();
  if ResponseWriter::error_context(&mut ctx, code).is_ok() && !ctx.is_empty() {
    enqueue(ctx).await;
  }
  error_bare(code).await;
}

/// Queue a single `error:N` WITHOUT the `[MSG:error:N <name>]` context push. Used for the post-error hold
/// rejection, whose code (`ERROR_HOLD_CODE` = 1) is reused purely to halt the sender: its name ("Expected
/// command letter") does NOT describe why the held line was rejected, so annotating it would mislead a plain
/// terminal. The byte-exact `error:N` is still emitted as the sole flow-control response.
async fn error_bare(code: u8) {
  let mut s = Response::new();
  if ResponseWriter::error(&mut s, code).is_ok() {
    enqueue(s).await;
  }
}

/// The status reporter: format a `<...>` report whenever the [`STATUS_REQUEST`] Signal fires (set by
/// `usb_rx` on `?`/`0x80`/`0x87`). It starts from the shared [`MachineSnapshot`] (run-state / feed / spindle
/// / RX free — the fields the executor does not own) and OVERWRITES the two genuinely-live fields at report
/// time: the MPos from the [`LIVE_POSITION`] atomics (Finding #5 — no longer frozen for a whole block), and
/// the planner free-block count from the live queue depth (Finding #10 — accurate while idle or streaming).
/// Reading these live, rather than from a periodically-published copy, keeps `?` truthful between publishes.
#[embassy_executor::task]
pub async fn status_responder() -> ! {
  loop {
    STATUS_REQUEST.wait().await;
    // Comms-progress heartbeat: serving a `?` is the most frequent host-facing work (skirnir polls ~5 Hz), so this
    // is the watchdog's primary "the comms path is alive" signal. A wedge that stops answering `?` (the real-board
    // failure: writes succeed, DRO frozen) freezes this counter, which — with the host still sending RX — is what
    // trips the comms-stall feed-withhold. NOTE: this bump is BEFORE the report is built, so it advances on the
    // INTENT to serve; the `usb_tx` bump covers actual output, so the two together bracket the response path.
    COMMS_PROGRESS.fetch_add(1, Ordering::Relaxed);
    // Read the cached, pre-derived status config (steps/mm, the `$10` MPos/WPos choice, and the feed ceiling)
    // from a synchronous `Cell` — NO `SETTINGS` lock, no ~30-field `Settings` copy on the hot path (Finding
    // #14). The cache is refreshed at every settings-commit site (see `refresh_status_cfg`), so a `$100`/`$10`/
    // `$110` change is reflected in the very next report.
    let cfg = STATUS_CFG.lock(|c| c.get());
    let steps_per_mm = cfg.steps_per_mm;
    let mut snap = *MACHINE.lock().await;
    // Live MPos: read the executor's per-burst step atomics and convert via the host-tested `steps_to_mm`.
    let position = read_live_position();
    snap.mpos_mm = steps_to_mm(&position, &steps_per_mm);
    // Phase B work-position reporting: the live WCO from the coordinate model, the MPos-vs-WPos choice from the
    // `$10` mask bit 0 (cached above), and the `WCO:` include decision from the refresh cadence. Folding these in
    // here keeps the math host-tested (protocol) and the wiring thin.
    let wco = coordinates().wco();
    snap.wco_mm = wco;
    snap.position_report = cfg.position_report;
    snap.include_wco = WCO_REPORTER.lock(|c| {
      let mut reporter = c.get();
      let include = reporter.should_include(wco);
      c.set(reporter);
      include
    });
    // Live `Bf:` free-block count, read from the planner queue under its lock so it is accurate whether the
    // machine is idle, streaming, or back-pressured — consistent with the advertised `BLOCK_QUEUE_LEN`.
    let blocks_free = planner_blocks_free().await;
    snap.planner_blocks_free = blocks_free;
    // Phase E: the live overrides drive both the `Ov:` element (on its change/periodic cadence) and the realized
    // `FS:` feed/spindle. Read the override snapshot once and apply it to the executor-published programmed feed
    // (scaled by the FEED override for a feed/jog move or the RAPID override for a G0, and CLAMPED to the most-
    // restrictive axis max-rate so a feed boost never exceeds `$110-112`) and to the programmed spindle RPM.
    let ov = overrides();
    snap.overrides = ov;
    snap.include_ov = OV_REPORTER.lock(|c| {
      let mut reporter = c.get();
      let include = reporter.should_include(ov);
      c.set(reporter);
      include
    });
    let programmed_feed = f32::from_bits(LIVE_PROGRAMMED_FEED_MM_MIN.load(Ordering::Acquire));
    let is_rapid = LIVE_BLOCK_IS_RAPID.load(Ordering::Relaxed);
    snap.feed_mm_min = if is_rapid {
      // A rapid is already governed by the axis max-rate; the rapid override only ever scales it DOWN.
      ov.scaled_rapid(programmed_feed)
    } else {
      // A feed/jog move scales by the feed override, clamped to the most-restrictive axis max-rate so scaling up
      // cannot exceed the configured rate limit (grbl's rule). The cached `min_axis_max_rate` is the ceiling.
      ov.scaled_feed(programmed_feed, cfg.min_axis_max_rate)
    };
    let programmed_rpm = PROGRAMMED_SPINDLE_RPM.load(Ordering::Acquire).min(u16::MAX as u32) as u16;
    snap.spindle_rpm = ov.scaled_rpm(programmed_rpm);
    // `Pn:` input pins. The probe sources `Pn:P` from its last-sampled logical asserted state (after `$6` invert,
    // published by the probe cycle). X/Y/Z limits source from [`LIMIT_LEVELS`] — the logical-triggered mask (after
    // `$5` invert) the core-1 executor republishes from a level sample at idle (the `Ticker`), at every block
    // boundary, and post-homing, so it tracks both press AND release. Door / feed-hold / reset / cycle-start are
    // optional DOC-06 control-input GPIO that this board does not wire, so they stay `false`; the assembly handles
    // them when a future board sources them — the path is the same host-tested [`PinReport::write_letters`].
    snap.pins = PinReport {
      probe: PROBE_ASSERTED.load(Ordering::Acquire),
      limits: limit_levels(),
      ..PinReport::new_idle()
    };
    // Compose the reported State from the authoritative latched control mode plus whether a block is in
    // flight. "Running" is true if the executor is mid-block OR the planner still holds queued blocks, so the
    // report shows `Run` from the instant a move is queued until the queue drains and the last burst finishes,
    // and `Idle` only when truly quiescent. Every non-Normal mode (Hold/Alarm/Check/Sleep) ignores `running`.
    let queued = blocks_free < firmware_core::planner::BLOCK_QUEUE_LEN as u8;
    let running = EXECUTOR_RUNNING.load(Ordering::Acquire) || queued;
    // A `$H` cycle in progress overrides the wire State to `Home` regardless of the latched control mode (which
    // sits in the boot-lock alarm or Normal while homing runs) — grbl reports `Home` for the cycle's duration
    // (research finding #1). Otherwise the State is the latched control mode + live Run/Idle derivation.
    snap.state = if HOMING_ACTIVE.load(Ordering::Acquire) {
      MachineState::Home
    } else {
      control_state().machine_state(running)
    };
    let mut s = Response::new();
    if ResponseWriter::status_report(&mut s, &snap).is_ok() {
      enqueue(s).await;
    }
    // Replay a pending crash report after the first status too (a host may poll `?` before `$I`). Consumed, so it
    // is emitted at most once more total across the `$I` and `?` paths — whichever the host reaches first.
    if let Some(report) = take_pending_crash_report() {
      enqueue(report).await;
    }
  }
}

/// The firmware-side floor for the auto-report cadence, milliseconds (Phase F). The `$481` setter and the
/// settings sanitizer already enforce grblHAL's `[100, 1000]` range, but this is a defense-in-depth clamp at the
/// task boundary: even if the live [`AUTO_REPORT_INTERVAL_MS`] mirror were somehow set to a tiny value, the
/// report task can never fire faster than this, so it can never starve the single `usb_tx` writer (a report
/// every few ms would crowd out `ok`s and break the host's character-counting flow control). 50 ms is half the
/// grblHAL floor — comfortably below any real DRO cadence yet a hard upper bound on report frequency.
const AUTO_REPORT_FLOOR_MS: u32 = 50;

/// The periodic auto-status-report task (Phase F, DOC-08 §5): when `$481` is non-zero and auto-reporting is not
/// suspended by `0x8C`, it raises [`STATUS_REQUEST`] every interval-ms so [`status_responder`] builds and emits
/// the SAME `<...>` report a `?` poll produces — there is exactly one status-report formatter and one USB
/// writer, so an auto-report and a `?`-triggered report are byte-identical and never interleave a partial line.
///
/// It signals `STATUS_REQUEST` rather than formatting the report itself, so this task adds NO new writer to the
/// USB endpoint and cannot race `status_responder` (the single owner of report formatting). When disabled (the
/// interval is 0 or auto-reporting is suspended) it parks on [`AUTO_REPORT_WAKE`] instead of spinning, so a
/// disabled auto-report costs nothing; a `$481=` enable or a `0x8C` resume wakes it at once. The interval is
/// re-read every iteration, so a `$481=` change takes effect on the next tick WITHOUT a reboot, and is floored at
/// [`AUTO_REPORT_FLOOR_MS`] so it can never starve the writer.
#[embassy_executor::task]
pub async fn auto_report_task() -> ! {
  loop {
    let interval = effective_auto_report_interval();
    match interval {
      // Disabled (interval 0 or suspended by `0x8C`): park until the enable state changes, then re-evaluate.
      None => AUTO_REPORT_WAKE.wait().await,
      // Enabled: race the interval tick against a wake (an enable-state change). On the tick, request a report;
      // on a wake, just loop to re-read the (possibly changed) interval. Reusing `STATUS_REQUEST` means the
      // report goes out through the one formatter / one writer, never a partial or interleaved line.
      Some(period) => match select(Timer::after(period), AUTO_REPORT_WAKE.wait()).await {
        Either::First(()) => STATUS_REQUEST.signal(()),
        Either::Second(()) => {}
      },
    }
  }
}

/// The effective auto-report period (Phase F): `None` when auto-reporting is disabled (a zero `$481` interval or
/// suspended by the `0x8C` toggle), else the configured interval floored at [`AUTO_REPORT_FLOOR_MS`] so the
/// report task can never be driven fast enough to starve the USB writer. Reads the live mirrors, so a `$481=`
/// change or a `0x8C` toggle is observed on the next call.
fn effective_auto_report_interval() -> Option<Duration> {
  if AUTO_REPORT_SUSPENDED.load(Ordering::Relaxed) {
    return None;
  }
  let interval = AUTO_REPORT_INTERVAL_MS.load(Ordering::Relaxed);
  if interval == 0 {
    None
  } else {
    Some(Duration::from_millis(interval.max(AUTO_REPORT_FLOOR_MS) as u64))
  }
}

/// The most-restrictive (smallest) per-axis max-rate in mm/min, used as the conservative ceiling for the
/// override-scaled `FS:` feed (Phase E): a feed boost must not exceed `$110-112`, and without per-block geometry
/// at report time the smallest axis rate is the safe upper bound on any block's realized feed. A non-positive
/// or empty rate set yields `f32::INFINITY` (no clamp) so a degenerate setting cannot pin the reported feed to 0.
fn min_axis_max_rate(max_rate_mm_min: &[f32; AXES]) -> f32 {
  max_rate_mm_min
    .iter()
    .copied()
    .filter(|r| r.is_finite() && *r > 0.0)
    .fold(f32::INFINITY, f32::min)
}

/// Read the live machine position (whole steps per axis) from the cross-core [`LIVE_POSITION`] atomics. Uses
/// `Acquire` loads so a reader on either core sees a coherent per-axis value the executor's sink published.
fn read_live_position() -> [i32; AXES] {
  let mut pos = [0i32; AXES];
  for (axis, slot) in pos.iter_mut().enumerate() {
    *slot = LIVE_POSITION[axis].load(Ordering::Acquire);
  }
  pos
}

/// The number of planner blocks currently free, read live from the shared planner queue and clamped to the
/// advertised [`BLOCK_QUEUE_LEN`](firmware_core::planner::BLOCK_QUEUE_LEN) so `Bf:` always stays consistent
/// with the buffer size reported in `[OPT:]` (Finding #10). A missing planner (init wiring bug, unreachable
/// in a wired build) reports the full queue free.
async fn planner_blocks_free() -> u8 {
  let guard = PLANNER.lock().await;
  match guard.as_ref() {
    Some(planner) => {
      let queued = planner.queued_len();
      firmware_core::planner::BLOCK_QUEUE_LEN.saturating_sub(queued) as u8
    }
    None => firmware_core::planner::BLOCK_QUEUE_LEN as u8,
  }
}

/// True when no motion is in flight: the core-1 executor is not mid-block AND the planner queue is fully drained.
/// This is the exact inverse of the `running` predicate the status reporter uses for `Run`/`Idle` (executor
/// busy OR blocks queued), so "idle" here means the same `Idle` the host sees. It also reads false during `$H`
/// homing and jogging, which both run on the executor with `EXECUTOR_RUNNING` set.
///
/// Used to defer flash persistence while the machine is moving: the esp-storage flash write parks the real-time
/// motion core (`multicore_auto_park` in `main`), so flushing mid-cycle would briefly stall step generation. The
/// settings/coordinate persists are not time-critical, so they wait for the machine to be quiescent (the next
/// burst boundary or safety tick once motion drains). The soft-reset flush deliberately does NOT consult this —
/// a reset is already aborting motion, and grbl applies settings on the next reset, so they must be on flash by
/// then. Checks the cheap [`EXECUTOR_RUNNING`] atom first and only locks the planner if it is clear.
async fn motion_idle() -> bool {
  if EXECUTOR_RUNNING.load(Ordering::Acquire) {
    return false;
  }
  planner_blocks_free().await == firmware_core::planner::BLOCK_QUEUE_LEN as u8
}

/// How long the consumer waits before retrying a [`PlannerError::QueueFull`] command. Short relative to a
/// block's execution time (tens of ms) so the retry claims a freed slot promptly, but long enough that the
/// retry loop is not a busy-spin — it yields to the core-1 motion executor each iteration.
const QUEUE_FULL_RETRY: Duration = Duration::from_millis(2);

/// Safety interval for the coalesced settings flush (Finding #14b): an upper bound on how long a pending
/// `$n=val`/`$PBX` change can sit un-persisted when the line queue never observably empties (a slow trickle
/// that always keeps one line in flight). The primary trigger is the burst boundary (queue empty); this is the
/// backstop so a dirty change is never indefinitely deferred. One second is far longer than a normal burst yet
/// short enough that a power loss after a paused write loses at most a second of un-flushed edits.
const SETTINGS_FLUSH_SAFETY: Duration = Duration::from_secs(1);

/// Backoff before retrying a USB read after a read error, so a persistent error does not become a tight
/// spin that starves the other core-0 tasks. Short enough that a transient glitch barely delays reception,
/// long enough to yield the executor on a sustained fault.
const USB_RX_ERROR_BACKOFF: Duration = Duration::from_millis(5);

/// Trim leading/trailing ASCII whitespace from a line, since a sender may pad `$` commands with spaces.
/// `core` lacks a stable slice trim for `&[u8]`, so this is a small explicit helper.
fn trim_ascii(line: &[u8]) -> &[u8] {
  let mut start = 0;
  let mut end = line.len();
  while start < end && (line[start] == b' ' || line[start] == b'\t') {
    start += 1;
  }
  while end > start && (line[end - 1] == b' ' || line[end - 1] == b'\t') {
    end -= 1;
  }
  &line[start..end]
}
