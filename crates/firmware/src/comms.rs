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
//!   grblHAL gcode error-hold and back-pressures the host when the planner buffer is full.
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
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};

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
use firmware_core::gcode::{
  CoordinateOp, DistanceMode as GcodeDistance, ModalState, MotionMode, Parser, Units as GcodeUnits,
};
use firmware_core::motion::steps_to_mm;
use firmware_core::planner::{Planner, PlannerConfig, PlannerError, PlannerOutcome, SoftLimits, AXES};
use firmware_core::protocol::{
  classify_realtime, probe_response, AlarmCode, CheckToggle, ControlState, CoordinateReport, EngineEvent,
  LastProbe, MachineSnapshot, Overrides, ParserDistance, ParserMotion, ParserSnapshot, ParserUnits,
  PinReport, PositionReport, ProbeResponse, RealtimeCommand, RefreshReporter, ResponseWriter, StreamEngine,
  SystemCommand, UnlockOutcome, ERROR_CODES, ERROR_HOMING_DISABLED, ERROR_UNSUPPORTED_COMMAND, MAX_LINE_LEN,
  NGC_PARAMETER_LINES, RX_BUFFER_SIZE, RESPONSE_CAPACITY,
};
use firmware_core::settings::{self, PbChunkResult, PbReceiver, SettingError, Settings};

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
pub static LIVE_POSITION: [AtomicI32; AXES] = [AtomicI32::new(0), AtomicI32::new(0), AtomicI32::new(0)];

/// Planner → motion-executor readiness signal (DOC-01). Set by [`plan_command`] after it enqueues a motion
/// block, so the core-1 `motion_executor` can AWAIT a fresh block when it finds the queue empty instead of
/// polling (the Stage-1 stub polled, which the review flagged). Living here in the bin keeps the pure
/// planner lib free of any async primitive: the planner reports a `Queued` outcome and the consumer raises
/// the signal. A `Signal` (not a counter) is sufficient because the executor re-checks the queue under the
/// lock after each wake and loops until it is drained, so a coalesced multi-block signal loses no block.
pub static BLOCK_AVAILABLE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

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

/// "Homing (`$22`) is enabled" — mirrored here from the live [`SETTINGS`] at boot (and on a `$22=` write) so
/// the control-state transitions that need it (boot-lock, soft-reset-to-boot, `$H` gating) can read it without
/// locking the async `SETTINGS` mutex from the synchronous real-time path. `Relaxed` is sufficient: it changes
/// only via `$22=` and is read for state decisions, never to guard other memory.
pub static HOMING_ENABLED: AtomicBool = AtomicBool::new(false);

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
    EngineEvent::Reject(code) => {
      let mut s = Response::new();
      if ResponseWriter::error(&mut s, code).is_ok() {
        enqueue(s).await;
      }
    }
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
    RealtimeCommand::Override(byte) => {
      // A feed / rapid / spindle / coolant override byte (Phase E, DOC-08 §3). Mutate the shared OVERRIDES in the
      // non-blocking reader half — a `Cell` swap, no await — so the change shows in the very next `?` (`Ov:` and
      // the realized `FS:`). An override is a real-time command and is NEVER acked. grbl ignores a no-op override
      // (one that does not change the value), so we only act when something actually changed. The core-1 executor
      // re-reads OVERRIDES once per block, so a new scale takes effect on the NEXT block boundary — a block
      // already in flight finishes at its current scale (Stage-1 per-block granularity; mid-block re-scaling is
      // the smooth-ramp Stage-2 refinement). No executor wake is needed: the level is re-read at the boundary.
      let mut ov = overrides();
      if ov.apply(byte) {
        set_overrides(ov);
        // TODO(DOC-05): a spindle-override / spindle-stop change must also re-drive the LEDC PWM duty on the
        // spindle task once the `PwmSink` backend lands; today the realized RPM is computed and reported but the
        // peripheral output is stubbed (no spindle hardware wired). Coolant (`0xA0`/`0xA1`, DOC-06) is likewise
        // tracked in the override state and reportable, but the flood/mist GPIO outputs are not wired yet.
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
    if SETTINGS_DIRTY.load(Ordering::Acquire) && LINE_QUEUE.is_empty() {
      flush_settings(flash).await;
    }
    // Coalesced coordinate persist (Phase B): same burst-boundary rule for the persistent G54-G59 / G28 / G30
    // record, so a program that re-zeroes several axes appends the coordinate blob once, not per line.
    if COORDINATES_DIRTY.load(Ordering::Acquire) && LINE_QUEUE.is_empty() {
      flush_coordinates(flash).await;
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
    match select4(LINE_QUEUE.receive(), SOFT_RESET.wait(), JOG_CANCEL.wait(), Timer::after(SETTINGS_FLUSH_SAFETY)).await {
      Either4::First(line) => handle_line(line.as_slice(), &mut parser, &mut state, flash).await,
      // A soft reset must not lose a pending settings change: persist before rebuilding the pipeline (grbl
      // applies most settings on the next reset, so they MUST be on flash by the time the reset takes them).
      Either4::Second(()) => {
        flush_settings(flash).await;
        flush_coordinates(flash).await;
        apply_soft_reset(&mut parser, &mut state).await;
      }
      // Jog cancel (`0x85`, Phase D): reuse the feed-hold block-boundary stop, flush the jog blocks, sync the
      // planner to the live stop point, and return to Idle. Owned here (the planner owner) so it is race-free
      // with line handling — a cancel and a line never run concurrently in this single in-order consumer.
      Either4::Third(()) => cancel_jog_cycle().await,
      // Safety-interval tick: persist any pending change even if the queue never observably drained. When
      // nothing is dirty these are cheap no-ops and the loop simply re-arms the timer on the next iteration.
      Either4::Fourth(()) => {
        flush_settings(flash).await;
        flush_coordinates(flash).await;
      }
    }
  }
}

/// Persist the live [`SETTINGS`] to flash IF a change is pending, clearing [`SETTINGS_DIRTY`]. This is the
/// single coalesced write path (Finding #14b): callers mark settings dirty per `$n=val`/`$PBX` line, and this
/// performs the actual flash append once per burst (queue-empty), on the safety interval, and on soft reset.
///
/// The dirty flag is cleared BEFORE the write so a change landing during the (awaited) flash op re-marks dirty
/// and is caught by the next flush — never silently coalesced away. A failed flush is logged via `defmt` (a
/// no-op in the default build) and otherwise swallowed: the in-RAM value already applied and the line was
/// already `ok`'d, matching the existing best-effort `store_settings` error handling — a stalled persist must
/// never wedge a character-counting sender. (The change is not re-marked on failure: the next dirty write or
/// the soft-reset flush will re-attempt persistence; re-marking here would spin the failing write every loop.)
async fn flush_settings(flash: &'static SharedFlash) {
  // Clear first so a concurrent `$n=val` applied during the await re-sets the flag and is not lost.
  if !SETTINGS_DIRTY.swap(false, Ordering::AcqRel) {
    return;
  }
  let snapshot = settings_snapshot().await;
  let mut store = FlashRecordStore::settings(flash);
  if settings::store_settings(&mut store, &snapshot).await.is_err() {
    #[cfg(feature = "defmt")]
    defmt::warn!("settings: failed to flush settings to flash");
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
  // Rebuild the pipeline FIRST so the banner (the "reset and ready" signal) is emitted, then push any
  // resulting alarm so a host sees `ALARM:N` and the `[MSG:..]` prompt right after the banner — matching grbl's
  // connect/reset ordering.
  reset_pipeline(parser, state).await;
  if let ControlState::Alarm(code) = next {
    emit_alarm(code).await;
  }
}

/// Emit an `ALARM:N` push line plus its `[MSG:..]` unlock/continue prompt, so a host detects the halt and
/// learns how to clear it (`$H`/`$X` for homing/locked alarms, reset for the recoverable ones).
async fn emit_alarm(code: AlarmCode) {
  let mut a = Response::new();
  if ResponseWriter::alarm(&mut a, code).is_ok() {
    enqueue(a).await;
  }
  send_message(code.unlock_hint()).await;
}

/// Reset the parser/planner pipeline state this task owns on a soft reset (`0x18`): restore the parser to
/// default modal state, clear the gcode error-hold, flush the planner queue, reset the non-position snapshot
/// fields to idle, and emit the guaranteed readiness banner. The `RX_PIPE`, the `line_assembler`'s partial
/// line, and `LINE_QUEUE` were already cleared by `usb_rx`; this completes the warm reset for the downstream
/// half so a fresh stream starts from defaults at the origin.
///
/// The LIVE MACHINE POSITION is deliberately NOT touched here: the core-1 motion executor is its single owner
/// (it zeroes the [`LIVE_POSITION`] atomics on [`MOTION_RESET`]), so the consumer writing it too would be a
/// cross-core stale-overwrite race (Finding #3). Resetting `MACHINE` to idle here only restores the fields
/// the executor does not own (run-state / feed / spindle / RX-free); `status_responder` reads MPos live.
async fn reset_pipeline(parser: &mut Parser, state: &mut ConsumerState) {
  // `Parser` exposes no in-place reset; reconstructing it restores the documented power-on modal defaults
  // (G0, G90, G21, F0, S0) — exactly grbl's warm-reset modal state.
  *parser = Parser::new();
  state.error_hold = false;
  // Drop any partially accumulated `$PBX=` import so a frame begun before the reset cannot bleed into one after.
  state.pb.reset();
  // Reconstruct the planner to clear the block queue, machine position, work offset, and junction state in
  // one step (it has no public flush), rebuilding it from the LIVE settings so any `$x=val` changes made
  // before the reset take effect now (grbl applies most settings on the next reset). Snapshot the settings
  // first so the `SETTINGS` lock is released before the `PLANNER` lock is taken. Reset the published
  // snapshot's non-position fields to idle; the live MPos atomics are zeroed by the executor on
  // `MOTION_RESET`, not here (single-owner, no race).
  let planner_config = settings_snapshot().await.planner_config();
  {
    let mut guard = PLANNER.lock().await;
    *guard = Some(Planner::new(planner_config));
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
    plan_gcode_line(trimmed, parser, state).await;
  }
}

/// Parse and plan one GCode line, emitting exactly one `ok`/`error:N`. Honors the error-hold: while held,
/// a GCode line is rejected without parsing. On a parse or planner error the hold is armed; on acceptance
/// (including modal-only `Ok(None)` lines) a single `ok` is emitted.
async fn plan_gcode_line(line: &[u8], parser: &mut Parser, state: &mut ConsumerState) {
  if state.error_hold {
    // Held by a prior error: reject without parsing until a recovery trigger. Reuse the generic
    // "expected command letter" code, matching how a sender already in error-recovery treats any further
    // rejection — it halts the stream regardless of the specific code (mirrors the engine's hold code).
    error(ERROR_HOLD_CODE).await;
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
  // Phase E: publish the parser's modal `S` word as the PROGRAMMED spindle RPM so `status_responder` can render
  // the override-scaled realized RPM in `FS:`. The modal `S` persists across lines (an `S` set once governs until
  // changed), so we publish it on any clean parse — including a modal-only `Ok(None)` line that only set `S`.
  if parsed.is_ok() {
    let rpm = parser.state().spindle_speed.max(0.0) as u32;
    PROGRAMMED_SPINDLE_RPM.store(rpm.min(u16::MAX as u32), Ordering::Release);
  }
  match parsed {
    // A blank/comment-only/modal-only line carries no action; acknowledge with a single `ok`.
    Ok(None) => ack().await,
    // `$C` check mode: the line parsed and validated cleanly, but check mode must NOT plan or execute it —
    // grbl `ok`s it so a host can verify a whole file without moving. The modal state still advanced in the
    // parser (correct: check mode tracks modal state), but no block is enqueued.
    Ok(Some(_)) if control == ControlState::Check => ack().await,
    Ok(Some(command)) => match plan_command(&command).await {
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
      // A soft reset arrived while this command was back-pressured: abort it (the host discards pending
      // acks on `0x18`), emit no response, and run the soft-reset transition whose signal was consumed here.
      PlanResult::Aborted => apply_soft_reset(parser, state).await,
      // A `G38.x` probe: run the probe-watching cycle on the core-1 executor and decide the response from the
      // outcome and the mode's alarm-on-fail flag.
      PlanResult::Probe { request, alarm_on_fail } => {
        handle_probe(request, alarm_on_fail, parser, state).await;
      }
    },
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
  /// A soft reset preempted the command while it was back-pressured; the consumed signal must be honored.
  Aborted,
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
/// a slot. Non-motion outcomes (Dwell/Spindle/G28/G92/M30) currently pass through with no side
/// effect; the real dwell timer, spindle driver (DOC-07), and homing (DOC-06) consume these in later phases.
async fn plan_command(command: &firmware_core::gcode::PlannerCommand) -> PlanResult {
  loop {
    // Scope the lock so it is released before any await: hold the planner mutex only for the plan call. A
    // missing planner (an init wiring bug, unreachable in a correctly wired build — see `init_planner`) is
    // surfaced as a distinct internal `error:N` rather than a fabricated `ok`: a silent accepted-but-un-run
    // move would hide the bug, so we fail the line loudly instead.
    let result = {
      let mut guard = PLANNER.lock().await;
      match guard.as_mut() {
        Some(planner) => planner.plan_command(command),
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
      // TODO(DOC-07/DOC-06): act on the remaining non-motion outcomes — start the dwell timer, drive the
      // spindle, run the predefined/homing move, reset program state on M30. Stage 1 accepts and passes
      // through. A zero-block `Queued` (no-op move) needs no executor wake, so it falls here too.
      Ok(_outcome) => return PlanResult::Accepted,
      // Back-pressure: the planner buffer is full. Do NOT ack and do NOT drop — yield to the motion
      // executor, then retry the same command. Blocking here backs `LINE_QUEUE` up and throttles the host (correct
      // grbl flow control). Race the retry delay against a soft reset so `0x18` aborts a stuck line at once;
      // the delay is short relative to a block's execution time, so a normal retry wins the freed slot
      // promptly without busy-spinning the CPU.
      Err(PlannerError::QueueFull) => {
        match select(Timer::after(QUEUE_FULL_RETRY), SOFT_RESET.wait()).await {
          Either::First(()) => {}
          Either::Second(()) => return PlanResult::Aborted,
        }
      }
      // A genuine geometry error (bad arc): surface the grblHAL code to the caller.
      Err(other) => return PlanResult::Error(other.code()),
    }
  }
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
    // Held by a prior GCode error: reject until a recovery trigger, like any motion line.
    error(ERROR_HOLD_CODE).await;
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

/// Build the `$20`/`$130–$132` soft-limit envelope to check a jog against, or `None` when `$20` is disabled so
/// the planner skips the check (grbl only enforces soft limits on a jog when they are enabled).
async fn current_soft_limits() -> Option<SoftLimits> {
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
      let (values, present) = axis_values_mm(&axes, units);
      coords.set_wcs_offset(index, values, present);
      persistent = true;
    }
    CoordinateOp::SetWcsOffsetToPosition { index, axes, units } => {
      let (values, present) = axis_values_mm(&axes, units);
      coords.set_wcs_offset_to_position(index, machine, values, present);
      persistent = true;
    }
    CoordinateOp::SetG92ToPosition { axes, units } => {
      let (values, present) = axis_values_mm(&axes, units);
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

/// Resolve a line's [`AxisWords`] into an mm value array plus a per-axis "present" mask, scaling inch words to
/// mm. Absent axes carry `0.0` with `present = false` so a mutator writes only the mentioned axes.
fn axis_values_mm(axes: &firmware_core::gcode::AxisWords, units: GcodeUnits) -> ([f32; AXES], [bool; AXES]) {
  let scale = units_scale(units);
  let words = [axes.x, axes.y, axes.z];
  let mut values = [0.0f32; AXES];
  let mut present = [false; AXES];
  for axis in 0..AXES {
    if let Some(value) = words[axis] {
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
    SystemCommand::Home => handle_home().await,
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
    // TODO(DOC-07/DOC-03): stop the spindle (LEDC PWM → 0) and de-energize the drivers (STEP_EN high) here.
    ack().await;
  } else {
    // Sleep is rejected from any non-Normal state (alarm/check/already asleep), matching grbl.
    error(ERROR_UNSUPPORTED_COMMAND).await;
  }
}

/// Handle `$H` (run the homing cycle). If `$22` homing is disabled, return `error:5` ("Homing cycle is not
/// enabled"). If enabled, homing itself is DOC-06 and NOT implemented yet, so return `error:N` with a clear
/// `[MSG:...]` rather than fake success — a fabricated `ok` would leave the machine UNHOMED but reported as
/// homed, the worst possible lie. Real homing is a separate later effort (DOC-06).
async fn handle_home() {
  if !HOMING_ENABLED.load(Ordering::Relaxed) {
    error(ERROR_HOMING_DISABLED).await;
    return;
  }
  // TODO(DOC-06): run the real homing cycle (seek → locate → pull-off per axis, then clear the homing-required
  // alarm). Until then, refuse loudly so no caller mistakes the machine for homed.
  send_message("Homing not implemented (DOC-06)").await;
  error(ERROR_UNSUPPORTED_COMMAND).await;
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
  HOMING_ENABLED.store(defaults.homing_enable, Ordering::Relaxed);
  set_control_state(ControlState::boot(defaults.homing_enable));
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
      HOMING_ENABLED.store(new_settings.homing_enable, Ordering::Relaxed);
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
/// re-marks dirty and is caught by the next flush; a failed flush is logged and swallowed.
async fn flush_coordinates(flash: &'static SharedFlash) {
  if !COORDINATES_DIRTY.swap(false, Ordering::AcqRel) {
    return;
  }
  let persistent = coordinates().persistent();
  let mut store = FlashRecordStore::coordinates(flash);
  if coords::store_coordinates(&mut store, &persistent).await.is_err() {
    #[cfg(feature = "defmt")]
    defmt::warn!("coordinates: failed to flush coordinate record to flash");
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
        HOMING_ENABLED.store(settings_snapshot().await.homing_enable, Ordering::Relaxed);
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

/// Queue the `$I`/`$I+` build-info lines.
async fn send_build_info(extended: bool) {
  let mut s = Response::new();
  if ResponseWriter::build_info(&mut s, extended).is_ok() {
    enqueue(s).await;
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
    wcs: state.wcs,
    tlo_active: state.tlo_active,
    feed: state.feed,
    // The parser tracks spindle speed as f32 RPM; the snapshot reports whole RPM (grbl's `$G` S word).
    spindle_rpm: state.spindle_speed.max(0.0) as u16,
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
/// two per consumed GCode line, preserving the one-response-per-line contract.
async fn error(code: u8) {
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
    // `Pn:` input pins. The probe is the one input wired today (Phase C); its last-sampled logical asserted state
    // (after `$6` invert, published by the probe cycle) sources `Pn:P`. The limit / door / control inputs are
    // DOC-06 hardware that is not wired yet, so they stay `false` — a clean stub whose assembly logic is complete
    // and host-tested, so wiring a real `DigitalIn` is a one-line change here.
    snap.pins = PinReport {
      probe: PROBE_ASSERTED.load(Ordering::Acquire),
      // TODO(DOC-06): source X/Y/Z limits, door, feed-hold, reset/e-stop, and cycle-start from the `DigitalIn`
      // GPIO inputs (with `$5`/`$14` invert handling) once the limit/control-input backend lands.
      ..PinReport::new_idle()
    };
    // Compose the reported State from the authoritative latched control mode plus whether a block is in
    // flight. "Running" is true if the executor is mid-block OR the planner still holds queued blocks, so the
    // report shows `Run` from the instant a move is queued until the queue drains and the last burst finishes,
    // and `Idle` only when truly quiescent. Every non-Normal mode (Hold/Alarm/Check/Sleep) ignores `running`.
    let queued = blocks_free < firmware_core::planner::BLOCK_QUEUE_LEN as u8;
    let running = EXECUTOR_RUNNING.load(Ordering::Acquire) || queued;
    snap.state = control_state().machine_state(running);
    let mut s = Response::new();
    if ResponseWriter::status_report(&mut s, &snap).is_ok() {
      enqueue(s).await;
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
