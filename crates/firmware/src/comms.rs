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

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::pipe::Pipe;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embassy_futures::select::{select, Either};
use embedded_io_async::{Read, Write};
use esp_hal::usb_serial_jtag::{UsbSerialJtagRx, UsbSerialJtagTx};
use esp_hal::Async;

use firmware_core::gcode::{ModalState, MotionMode, DistanceMode as GcodeDistance, Parser, Units as GcodeUnits};
use firmware_core::motion::MotionConfig;
use firmware_core::planner::{Planner, PlannerConfig, PlannerError, PlannerOutcome, AXES};
use firmware_core::protocol::{
  classify_realtime, EngineEvent, MachineSnapshot, ParserDistance, ParserMotion, ParserSnapshot,
  ParserUnits, RealtimeCommand, ResponseWriter, StreamEngine, MAX_LINE_LEN, RX_BUFFER_SIZE,
  RESPONSE_CAPACITY,
};

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

/// `!` (feed hold) — set by `usb_rx`. Stage-1 consumer is a documented stub (no motion yet).
pub static FEED_HOLD: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// `~` (cycle start / resume) — set by `usb_rx`. Stage-1 consumer is a documented stub.
pub static CYCLE_START: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// `0x18` (soft reset) — set by `usb_rx`. The reset handler re-emits the banner and (later) flushes the
/// parser/planner/motion queues; Stage 1 re-emits the banner and clears the line queue.
pub static SOFT_RESET: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Planner → motion-executor readiness signal (DOC-01). Set by [`plan_command`] after it enqueues a motion
/// block, so the core-1 `motion_executor` can AWAIT a fresh block when it finds the queue empty instead of
/// polling (the Stage-1 stub polled, which the review flagged). Living here in the bin keeps the pure
/// planner lib free of any async primitive: the planner reports a `Queued` outcome and the consumer raises
/// the signal. A `Signal` (not a counter) is sufficient because the executor re-checks the queue under the
/// lock after each wake and loops until it is drained, so a coalesced multi-block signal loses no block.
pub static BLOCK_AVAILABLE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// The live machine state the status formatter reads. The core-1 `motion_executor` publishes the live
/// (interpolated) MPos and the planner free-block count here as it executes blocks; Stage 1 holds the idle
/// default at boot so `?` returns a well-formed report immediately. Shared across cores via
/// [`CriticalSectionRawMutex`], which gates both cores on the S3.
pub static MACHINE: Mutex<CriticalSectionRawMutex, MachineSnapshot> = Mutex::new(MachineSnapshot::idle());

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

/// Build the placeholder [`PlannerConfig`]. These are grbl-like defaults standing in for the real
/// `$`-settings the firmware will load from esp-storage (DOC-00) once persistence is wired: 250 steps/mm,
/// 500 mm/min max rate, 10 mm/s² accel, `$11`=0.01 mm, `$12`=0.002 mm per axis. TODO(DOC-00): replace
/// with the persisted settings load (CRC-checked, falling back to these compiled defaults).
fn placeholder_planner_config() -> PlannerConfig {
  PlannerConfig::default()
}

/// Build the placeholder [`MotionConfig`] for the step generator: the `$0` step-pulse width and the timer
/// tick rate the RMT channels are clocked at (1 MHz → 1 tick = 1 µs). Defaults stand in for the persisted
/// `$0`/`$29` firmware settings (DOC-00). The `motion_executor` uses this to drive the generator, and the
/// RMT init derives the channel clock divider from `tick_hz` — keep them in agreement. TODO(DOC-00):
/// replace with the persisted settings load (CRC-checked, falling back to these compiled defaults).
pub fn placeholder_motion_config() -> MotionConfig {
  MotionConfig::default()
}

/// The placeholder per-axis steps/mm (`$100..102`) the motion executor uses to convert its live step
/// counter into MPos millimeters. Sourced from the SAME [`placeholder_planner_config`] the planner rounds
/// with, so the planner and the live position report agree on resolution (a single source, not a second
/// copy). TODO(DOC-00): load `$100..102` from esp-storage alongside the rest of the settings.
pub fn placeholder_steps_per_mm() -> [f32; AXES] {
  placeholder_planner_config().steps_per_mm
}

/// Install the planner into [`PLANNER`] at boot, before the consumer and the core-1 motion executor are
/// spawned. Called once from `main`; `try_lock` avoids an await in init and cannot contend (no task runs yet).
pub fn init_planner() {
  // `try_lock` succeeds because this runs before any task is spawned, so nothing else holds the lock.
  // The `Ok` arm is the only reachable path at init; a failure would be a wiring bug, handled by leaving
  // the planner uninitialized (the consumer then treats every line as an internal error rather than
  // panicking), but in practice this never fails.
  if let Ok(mut guard) = PLANNER.try_lock() {
    *guard = Some(Planner::new(placeholder_planner_config()));
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
    RealtimeCommand::FeedHold => FEED_HOLD.signal(()),
    RealtimeCommand::CycleStart => CYCLE_START.signal(()),
    RealtimeCommand::SoftReset | RealtimeCommand::Stop => {
      // Drop every buffered RX byte and every framed-but-unconsumed line so post-reset modal state is not
      // contaminated by anything that arrived before the reset. Both Signals are set so the assembler drops
      // its partial line and the consumer rebuilds the parser/planner and re-emits the banner.
      RX_PIPE.clear();
      while LINE_QUEUE.try_receive().is_ok() {}
      LINE_RESET.signal(());
      SOFT_RESET.signal(());
      // Best-effort immediate readiness banner; the consumer's reset emits the guaranteed one. Dropping
      // this (full RESPONSE) is harmless and keeps the reader non-blocking on the real-time path.
      try_send_banner();
    }
    // Parser-state-on-demand, safety door, jog cancel, auto-report toggle, and overrides are accepted but
    // not yet acted on (Stage 2/3). They correctly produce no `ok`.
    RealtimeCommand::ParserStateReport
    | RealtimeCommand::SafetyDoor
    | RealtimeCommand::JogCancel
    | RealtimeCommand::ToggleAutoReport
    | RealtimeCommand::Override(_) => {}
  }
}

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
pub async fn comms_consumer() -> ! {
  let mut parser = Parser::new();
  let mut state = ConsumerState::default();
  loop {
    // Race a soft reset against the next line so a `0x18` arriving mid-stream resets the parser modal
    // state, clears the error-hold, and flushes the planner queue BEFORE the next line is parsed. `usb_rx`
    // already cleared `RX_PIPE` + `LINE_QUEUE` and signalled the line assembler when it fired the reset;
    // here we reset the pipeline state this task owns AND emit the guaranteed readiness banner. A line that
    // lost the race is dropped (it predates the reset), matching grbl's warm-reset semantics.
    //
    // Known window (Finding #6, accepted): the select only observes the reset at the loop boundary. If a
    // `0x18` lands while `handle_line` is already mid-flight for a non-back-pressured line, that line can
    // still emit its `ok`/`error` after the reset signal — a single stray response. Back-pressured lines do
    // observe the reset (they race `SOFT_RESET` inside `plan_command` and return `Aborted`). Closing the
    // remaining window for an in-flight non-back-pressured line would require cancelling `handle_line`
    // mid-await; that is deferred to the alarm-state machine (Stage 2), which is where grbl gates response
    // emission during an abort. The stray response is benign: the host discards pending acks on `0x18`.
    match select(LINE_QUEUE.receive(), SOFT_RESET.wait()).await {
      Either::First(line) => handle_line(line.as_slice(), &mut parser, &mut state).await,
      Either::Second(()) => reset_pipeline(&mut parser, &mut state).await,
    }
  }
}

/// Reset the parser/planner pipeline state this task owns on a soft reset (`0x18`): restore the parser to
/// default modal state, clear the gcode error-hold, flush the planner queue and machine position, and emit
/// the guaranteed readiness banner. The `RX_PIPE`, the `line_assembler`'s partial line, and `LINE_QUEUE`
/// were already cleared by `usb_rx`; this completes the warm reset for the downstream half so a fresh stream
/// starts from defaults at the origin.
async fn reset_pipeline(parser: &mut Parser, state: &mut ConsumerState) {
  // `Parser` exposes no in-place reset; reconstructing it restores the documented power-on modal defaults
  // (G0, G90, G21, F0, S0) — exactly grbl's warm-reset modal state.
  *parser = Parser::new();
  state.error_hold = false;
  // Reconstruct the planner to clear the block queue, machine position, work offset, and junction state in
  // one step (it has no public flush). The placeholder config is re-applied; once esp-storage settings are
  // wired the reset will re-load them. Reset the published snapshot to idle so `?` reports the origin.
  {
    let mut guard = PLANNER.lock().await;
    *guard = Some(Planner::new(placeholder_planner_config()));
  }
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
}

/// Route one accepted line. `$` system commands are dispatched to their report handlers and clear the
/// error-hold (a recovery trigger). Otherwise the line is parsed and planned. Exactly one `ok`/`error:N`
/// is emitted per call, preserving the one-response-per-line contract.
async fn handle_line(line: &[u8], parser: &mut Parser, state: &mut ConsumerState) {
  let trimmed = trim_ascii(line);
  if trimmed.is_empty() {
    // A blank line (forwarded by `usb_rx`) is a grblHAL error-hold recovery trigger: clear the hold and
    // acknowledge with a bare `ok`. Handling it here, in queue order, keeps recovery race-free with the
    // surrounding lines, since the hold lives in this task, the sole in-order reader of `LINE_QUEUE`.
    state.error_hold = false;
    ack().await;
  } else if let Some(rest) = trimmed.strip_prefix(b"$") {
    // A `$` system command is the other grblHAL recovery trigger and is answered by its handler. Both
    // recovery triggers (an empty line and a `$` command) and a soft reset clear the hold; nothing else.
    // The parser is passed so `$G` can report the live modal state rather than a hardcoded default.
    state.error_hold = false;
    handle_system_command(rest, parser).await;
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
  match parser.parse_line(line) {
    // A blank/comment-only/modal-only line carries no action; acknowledge with a single `ok`.
    Ok(None) => ack().await,
    Ok(Some(command)) => match plan_command(&command).await {
      // The command was accepted into the planner (a move enqueued, or a non-motion outcome passed
      // through); emit the single `ok`.
      PlanResult::Accepted => ack().await,
      // The planner reported a non-back-pressure error (bad arc geometry); reject and arm the hold.
      PlanResult::Error(code) => {
        error(code).await;
        state.error_hold = true;
      }
      // A soft reset arrived while this command was back-pressured: abort it (the host discards pending
      // acks on `0x18`), emit no response, and run the pipeline reset whose signal was consumed here.
      PlanResult::Aborted => reset_pipeline(parser, state).await,
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
  /// A non-back-pressure planner error; carries the grblHAL `error:N` code (bad arc geometry).
  Error(u8),
  /// A soft reset preempted the command while it was back-pressured; the consumed signal must be honored.
  Aborted,
}

/// The `error:N` code used to reject a GCode line that is held in the post-error state. Matches the
/// engine's `ERROR_HOLD_CODE` (grbl's generic "expected command letter" code 1): a sender already in
/// error-recovery halts the stream regardless of the specific code.
const ERROR_HOLD_CODE: u8 = 1;

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
      // TODO(DOC-07/DOC-06): act on the non-motion outcomes — start the dwell timer, drive the spindle, run
      // the predefined/homing move, reset program state on M30. Stage 1 accepts and passes through. A
      // zero-block `Queued` (no-op move) needs no executor wake, so it falls here too.
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

/// Answer a `$<rest>` system command. Stage 1 implements the connect-critical subset: `$$` settings dump
/// (stub: header + `ok`), `$I`/`$I+` build info, `$G` parser state, `$#` NGC parameters (stub). Each
/// multi-line response ends with `ok`. An unrecognized `$` command is acknowledged so the stream advances.
async fn handle_system_command(rest: &[u8], parser: &Parser) {
  match rest {
    b"" | b"$" => {
      // `$` (help) / `$$` dump. Stage 1 returns an empty dump plus `ok`; real settings land with
      // esp-storage persistence (TODO). This keeps `$$` from stalling a sender that probes settings.
      ack().await;
    }
    b"I" => {
      send_build_info(false).await;
      ack().await;
    }
    b"I+" => {
      send_build_info(true).await;
      ack().await;
    }
    b"G" => {
      send_parser_state(parser).await;
      ack().await;
    }
    b"#" => {
      // NGC parameters dump. Stage 1 has no stored offsets; acknowledge so `$#` does not stall.
      ack().await;
    }
    _ => ack().await,
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

/// The status reporter: format a `<...>` report from the shared [`MachineSnapshot`] whenever the
/// [`STATUS_REQUEST`] Signal fires (set by `usb_rx` on `?`/`0x80`/`0x87`). Stage 1 reports the shared
/// snapshot (idle/origin until the motion executor publishes live data).
#[embassy_executor::task]
pub async fn status_responder() -> ! {
  loop {
    STATUS_REQUEST.wait().await;
    let snap = *MACHINE.lock().await;
    let mut s = Response::new();
    if ResponseWriter::status_report(&mut s, &snap).is_ok() {
      enqueue(s).await;
    }
  }
}

/// How long the consumer waits before retrying a [`PlannerError::QueueFull`] command. Short relative to a
/// block's execution time (tens of ms) so the retry claims a freed slot promptly, but long enough that the
/// retry loop is not a busy-spin — it yields to the core-1 motion executor each iteration.
const QUEUE_FULL_RETRY: Duration = Duration::from_millis(2);

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
