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

use core::sync::atomic::Ordering;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embassy_futures::select::{select, select4, Either, Either4};

use firmware_core::coords;
use firmware_core::homing::{HomingConfig, HomingError};
use firmware_core::gcode::{
  CoordinateOp, DistanceMode as GcodeDistance, ModalState, Parser, SpindleState, Units as GcodeUnits,
};
use firmware_core::motion::steps_to_mm;
use firmware_core::planner::{Planner, PlannerError, PlannerOutcome, SoftLimits, SpinUpGate, AXES};
use firmware_core::spindle::SpindleAction;
use firmware_core::protocol::{
  probe_response, AlarmCode, CheckToggle, ControlState, CoordinateReport, LastProbe, MachineSnapshot,
  MachineState, Overrides,
  PinReport, ProbeResponse, ResponseWriter, SystemCommand, UnlockOutcome,
  ERROR_CODES, ERROR_HOMING_DISABLED, ERROR_NOT_IDLE, ERROR_UNSUPPORTED_COMMAND, NGC_PARAMETER_LINES,
  RESPONSE_CAPACITY,
};
use firmware_core::settings::{self, PbChunkResult, PbReceiver, SettingError, Settings};

use crate::spindle;
use crate::storage::{FlashRecordStore, SharedFlash};

// E1 (architecture-refactor): these four genuinely-pure helpers moved to the host-tested `firmware-core` so they
// gain unit coverage and honor the DOC-09 layering rule. Re-exported `pub(crate)` here so the existing call sites
// across the `comms/` submodules keep resolving unchanged via their `use super::*` globs, at the new core paths.
pub(crate) use firmware_core::coolant::coolant_mask;
pub(crate) use firmware_core::coords::{axis_values_mm, units_scale};
pub(crate) use firmware_core::protocol::parser_snapshot;

mod state;
pub(crate) use state::*;

mod crash_report;
pub(crate) use crash_report::*;

mod rx;
pub(crate) use rx::*;

mod tx;
pub(crate) use tx::*;

mod realtime;
pub(crate) use realtime::*;

mod syscmd;
pub(crate) use syscmd::*;

mod probe;
pub(crate) use probe::*;

mod jog;
pub(crate) use jog::*;

mod homing;
pub(crate) use homing::*;

mod program_flow;
pub(crate) use program_flow::*;

mod spindle_coolant;
pub(crate) use spindle_coolant::*;

mod status;
pub(crate) use status::*;

mod watchdog;
pub(crate) use watchdog::*;

mod consumer;
pub(crate) use consumer::*;

/// Emit the boot `ALARM:N` (plus its `[MSG:..unlock]` prompt) IF the machine booted into an alarm — i.e.
/// `$22` homing is enabled, so [`init_control_state`] latched `ALARM:11` (homing required). Called once from
/// `main` after the banner so a sender detects the locked state on connect (DOC-08 §5). A no-op when the
/// machine boots Idle.
pub async fn send_boot_alarm() {
  if let ControlState::Alarm(code) = control_state() {
    emit_alarm(code).await;
  }
}

/// Force the machine into the fail-safe wedge-reset alarm (Design A, §20 / Fix #1), called at boot by `main` when
/// [`crate::crash::position_suspect`] is true — i.e. the prior reset was ANY watchdog-withheld wedge (core-1 motion,
/// core-0 comms, the dead-zone backstop, or the core-0 executor stall; all leave the position suspect, not just the
/// executor stall).
/// Overrides the default boot state so the board comes up LOCKED and can NEVER silently resume in a now-suspect
/// position: homing ENABLED ⇒ `ALARM:11` (re-home) — usually already the boot state, so this is idempotent; homing
/// DISABLED ⇒ `ALARM:3` (position lost, reset/`$X` + re-zero) instead of the default `Idle` — the gap this closes.
/// The alarm CODE is the host-tested [`firmware_core::protocol::wedge_reset_alarm`]. Synchronous (a single latched
/// store); the boot path emits it via the subsequent [`send_boot_alarm`]. Also un-sets `HOMED` so an operator cannot
/// jog/stream against the suspect position without re-establishing it.
pub fn force_wedge_alarm(homing_enabled: bool) {
  let code = firmware_core::protocol::wedge_reset_alarm(homing_enabled);
  set_control_state(ControlState::Alarm(code));
  // Position is suspect after the wedge reset — treat the machine as unhomed so the re-home / re-zero is required.
  HOMED.store(false, Ordering::Relaxed);
}

/// Wakes the auto-report task (Phase F) when its enable state changes — set by the `0x8C` resume toggle and by a
/// `$481=` write that turns auto-reporting on — so the task re-arms its interval timer immediately rather than
/// sleeping out a stale (possibly long) interval. A `Signal` (coalesced) is enough: the task re-reads the live
/// interval/suspend state after every wake, so a missed-and-coalesced wake loses nothing.
pub static AUTO_REPORT_WAKE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Emit an `ALARM:N` push line plus its `[MSG:..]` unlock/continue prompt, so a host detects the halt and
/// learns how to clear it (`$H`/`$X` for homing/locked alarms, reset for the recoverable ones).
pub(crate) async fn emit_alarm(code: AlarmCode) {
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

/// The consumer's persistent control state across lines: the gcode error-hold flag. Held locally in the
/// single consumer task so it is mutated only in line order.
#[derive(Default)]
pub(crate) struct ConsumerState {
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

/// The `error:N` code used to reject a GCode line that is held in the post-error state. Matches the
/// engine's `ERROR_HOLD_CODE` (grbl's generic "expected command letter" code 1): a sender already in
/// error-recovery halts the stream regardless of the specific code.
const ERROR_HOLD_CODE: u8 = 1;

/// The grblHAL `error:N` code for GCode rejected because the machine is in an alarm, jog, or sleep state —
/// grbl's "G-code locked out during alarm or jog state" (Finding #5). Distinct from the in-stream
/// post-error hold (`ERROR_HOLD_CODE` = 1): this is a STATE lockout, and code 9 already has a row in the
/// `$EE` [`ERROR_CODES`](firmware_core::protocol::ERROR_CODES) table, so a sender's display matches.
const ERROR_LOCKED: u8 = 9;

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

/// Queue a `[MSG:<text>]` push message (e.g. `[MSG:Caution: Unlocked]`, `[MSG:Enabled]`).
pub(crate) async fn send_message(text: &str) {
  let mut s = Response::new();
  if ResponseWriter::message(&mut s, text).is_ok() {
    enqueue(s).await;
  }
}

/// Reset the `WCO:` refresh cadence so the next status report re-emits the `WCO:` element (grbl's "first report
/// after a reset includes WCO"). Called on a soft reset and after `$RST=#`.
pub(crate) fn reset_wco_reporter() {
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

/// The grbl "settings only when idle" gate (`error:8`), shared by every settings-mutating `$` handler
/// (`$n=val`, `$Nx=`, `$RST=…`, the `$PBX` import). Two halves, mirroring the jog gate's split: the pure
/// [`ControlState::settings_write_allowed`] rejects every latched non-Normal/non-Alarm mode (Hold/Jog/Check/
/// Sleep/Tool), and — because `Normal` means Idle-or-Run — [`motion_idle`] additionally requires live motion to
/// be fully quiescent (executor idle AND queue drained), so a write can never land under a running job and
/// leave the planner mixing old- and new-scale kinematics. Alarm passes the gate (grbl allows it) so a bad
/// soft-limit/travel value can be corrected without `$X` first.
pub(crate) async fn settings_write_blocked() -> bool {
  let control = control_state();
  if !control.settings_write_allowed() {
    return true;
  }
  control == ControlState::Normal && !motion_idle().await
}

/// Push the LIVE settings' [`PlannerConfig`] into the planner in place — the settings-commit companion to
/// [`refresh_status_cfg`], so a `$100–$102`/`$110+`/junction/soft-limit change takes effect on the NEXT motion
/// line instead of waiting for a soft reset (grbl applies these immediately; a planner that lagged them would
/// also disagree with the status reporter's already-live steps→mm conversion). Position and work offset are
/// preserved by [`Planner::set_config`]; the settings-write gate guarantees the queue is empty when any commit
/// site runs, so no queued block mixes scales. Call at every site that mutates a planner-affecting setting:
/// the `$x=val` write path and the `$PBX` import completion (the `$RST` paths instead run the full pipeline
/// reset, which rebuilds the planner from the restored settings wholesale).
async fn refresh_planner_config() {
  let config = settings_snapshot().await.planner_config();
  let mut guard = PLANNER.lock().await;
  if let Some(planner) = guard.as_mut() {
    planner.set_config(config);
  }
}
