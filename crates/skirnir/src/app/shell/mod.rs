//! The eframe application shell: the one place that owns the engine, the tokio runtime, and the window.
//!
//! The shell is the seam between the immediate-mode UI and the async streaming engine. It holds a tokio
//! runtime (the engine's driver task lives there), an [`EngineHandle`] when connected, the engine-derived
//! [`ViewState`], and the transient [`UiState`]. Every frame it: drains pending engine events into the view
//! state (non-blocking — never `.await`s on the UI thread), lays out the panels, then acts on the [`Intent`]s
//! the views emitted. It calls `request_repaint_after` so live status telemetry refreshes promptly without a
//! busy loop.
//!
//! All serial I/O and the streaming state machine run on the engine's background task; the shell only ever
//! touches the engine through the handle's non-blocking channels, so a stalled port can never freeze the UI.

use std::time::{Duration, Instant};

use eframe::egui;

use super::intent::{Axis, Dir, Intent};
use super::theme::Palette;
use super::views::{self, UiState};
use super::view_state::ViewState;
use crate::engine::{Command, EngineHandle};
// `Engine` is only constructed on the serial connect path; gate the import so a gui-only build (no `serial`)
// does not warn on an unused import under `#![deny(warnings)]`.
#[cfg(feature = "serial")]
use crate::engine::Engine;

mod connection;
mod controls;
mod lifecycle;
mod persistence;
mod streaming;
mod wizards;

/// How often to wake the UI for fresh telemetry even when idle, so the DRO/console stay live under the
/// firmware's auto-report cadence without spinning the CPU between frames.
const REPAINT_INTERVAL: Duration = Duration::from_millis(50);

/// The motion time one streamed continuous-jog increment is sized for. A held jog is streamed as a run of short
/// `$J=` moves rather than one long move, so a jog-cancel (`0x85`) on release stops within a single short block
/// instead of running a 10 m move to its far boundary (the firmware cancels at the active block's boundary; a
/// mid-block ramp-down is a firmware Stage-2 TODO). Each increment's length is `feed * BLOCK_SECS`, so its
/// motion time — and thus the worst-case stop latency after release — is ~this regardless of feed.
const JOG_STREAM_BLOCK_SECS: f64 = 0.12;

/// How often a held continuous jog emits the next increment. Matched to [`JOG_STREAM_BLOCK_SECS`] so the host
/// produces blocks at roughly the rate the firmware executes them, keeping the planner queue shallow; the `Bf:`
/// backstop below absorbs any drift.
const JOG_STREAM_INTERVAL: Duration = Duration::from_millis(120);

/// Stop streaming new increments while the firmware reports fewer than this many planner blocks free, so a held
/// jog can never overrun the 32-block queue into a `QueueFull` rejection. `Bf:` lags (status polls at ~5 Hz), so
/// the margin is generous enough to absorb several increments' worth of staleness.
const JOG_STREAM_MIN_BLOCKS_FREE: u32 = 8;

/// How stale the last `<...>` status report may be before its `Bf:` blocks-free reading is no longer trusted to
/// gate the jog stream. Status polls at ~5 Hz (a ~200 ms interval), so this is ~3 polls' worth of slack: a single
/// dropped or delayed report still trusts the last reading, but a sustained stall (firmware busy, polling starved)
/// crosses it. Past this age the queue state is treated as UNKNOWN and the stream HOLDS rather than streaming on a
/// frozen `Bf:` — otherwise a held jog could overrun the 32-block queue into a `QueueFull` rejection.
const JOG_STREAM_STATUS_MAX_AGE: Duration = Duration::from_millis(600);

/// An in-progress continuous (press-and-hold) jog, streamed as short `$J=` increments until the operator
/// releases. `Copy` so the per-frame pump can snapshot it without holding a borrow across the send.
#[derive(Clone, Copy)]
struct JogStream {
  /// The axis being jogged.
  axis: Axis,
  /// The direction along that axis.
  dir: Dir,
  /// The feed (mm/min) the increments carry, which also sizes each increment's length.
  feed: f64,
  /// Wall-clock instant the next increment is due, so increments pace to one block's motion time.
  next_send_at: Instant,
}

/// The skirnir application: state plus the tokio runtime that hosts the engine task.
pub struct SkirnirApp {
  /// The tokio runtime the engine's driver task runs on. Held for the app's lifetime; `Engine::connect` is
  /// called inside its context so the internal `tokio::spawn` has a runtime to attach to. Only the serial
  /// connect path enters it, so a gui-only build (no `serial`) carries no runtime and never reads this field.
  #[cfg(feature = "serial")]
  runtime: tokio::runtime::Runtime,
  /// The live engine handle when connected, else `None`. Dropping it winds the engine task down.
  engine: Option<EngineHandle>,
  /// The last port/baud a user-initiated connect opened, remembered so an auto-reconnect can re-open the same
  /// endpoint after a drop. `None` until the first connect. Survives a disconnect so a soft-reset re-enumeration
  /// (which often brings the board back at the same path) can be chased.
  #[cfg(feature = "serial")]
  last_endpoint: Option<(String, u32)>,
  /// Whether the operator wants to stay connected, so a drop should auto-reconnect. Set on a user connect,
  /// cleared on a user disconnect — an explicit disconnect must never trigger a reconnect, only an unexpected
  /// drop does. Gone the moment the operator tears the link down on purpose.
  #[cfg(feature = "serial")]
  auto_reconnect: bool,
  /// The host-side backoff schedule consulted after an unexpected drop. Reset on a successful connect so each
  /// fresh drop starts its backoff from the base delay.
  #[cfg(feature = "serial")]
  reconnect: crate::reconnect::ReconnectPolicy,
  /// The wall-clock instant the next auto-reconnect attempt is due, or `None` when none is scheduled. Checked
  /// each frame (a cheap deadline compare, no thread/timer) so the retry rides the existing repaint loop and
  /// never blocks the UI; the repaint scheduler keeps the loop alive while this is `Some`.
  #[cfg(feature = "serial")]
  reconnect_at: Option<Instant>,
  /// The engine-derived render state.
  view: ViewState,
  /// The transient widget state the views read and mutate.
  ui: UiState,
  /// The host's estimate of each override axis, so a slider commit steps from where the previous commit left
  /// the firmware rather than the lagging `Ov:` field of the last status report. Reset on a disconnect — the
  /// estimate belongs to the session that just ended.
  override_tracker: super::overrides::OverrideTracker,
  /// When the current stream began, for the dock's elapsed/ETA clock. Set the first frame the lifecycle enters
  /// `Streaming` and cleared when a fresh run begins or the link drops; `None` means no stream has timed. Held in
  /// the shell (not the pure reducer) because it is wall-clock state the egui frame owns — the reducer stays free of
  /// `Instant::now()`.
  stream_started: Option<Instant>,
  /// When the current stream GENUINELY COMPLETED — every program line acked and the machine back at Idle (see
  /// [`super::progress::stream_is_complete`]). Latched ONCE at that moment so the dock's elapsed FREEZES at its final
  /// value and the ETA stops, rather than counting up forever off the live `stream_started.elapsed()` (the
  /// keeps-ticking-after-the-job-finished bug). `None` while a run is still in progress or none has run; cleared
  /// alongside [`Self::stream_started`] when a new run starts. The freeze is purely a display concern — it does not
  /// touch the streaming lifecycle, which stays host-driven in the reducer.
  stream_finished_at: Option<Instant>,
  /// The most recent physics-based job-time estimate ([`crate::eta::EtaTimeline`]) the operator computed with the
  /// Simulate button, or `None` until they run one (cleared when a new program is opened). When present it drives
  /// the dock's ETA: the upfront total before a stream starts, and the live remaining (drained by completed line
  /// and rescaled by the live overrides) during one. A pure host calc — built once on the click, not per frame —
  /// so it costs only the per-frame `remaining_seconds` sum to read. `None` falls back to the acked-rate estimate.
  simulated: Option<crate::eta::EtaTimeline>,
  /// Whether the last simulation fell back to the firmware's DEFAULT machine settings because no `$$` snapshot was
  /// loaded (the operator simulated while disconnected, or before fetching settings). Surfaced as a small "(default
  /// settings)" qualifier near the ETA so the figure is not mistaken for one grounded in the board's real config.
  simulated_default_settings: bool,
  /// The height-corrected program cached from the last [`Self::resolve_stream_program`], or `None` when cold. When
  /// autoleveling is armed, [`crate::app::autolevel::correct_program`] rewrites the loaded file once and the result
  /// is reused by both the stream and the ETA build (which must key on the SAME, longer, corrected line total).
  /// Invalidated ([`Self::invalidate_autolevel`]) on program load, a mesh change, and any autolevel toggle/config
  /// change, so a stale correction can never reach the wire.
  autolevel_cache: Option<std::sync::Arc<[String]>>,
  /// The continuous jog currently being streamed while the operator holds a jog control, or `None`. Owned by the
  /// shell (not the reducer) because pacing the increments is wall-clock work the egui frame drives.
  jog_stream: Option<JogStream>,
  /// Wall-clock instant the last `<...>` status report was observed, or `None` if none has arrived this session.
  /// The jog-stream throttle reads this to age the `Bf:` blocks-free reading: a stale report (older than
  /// [`JOG_STREAM_STATUS_MAX_AGE`]) is no longer trusted, so the stream holds rather than pacing on a frozen queue
  /// reading. Held in the shell (not the reducer) because it is wall-clock state the egui frame owns; cleared on a
  /// disconnect so a stale timestamp never carries into the next session.
  last_status_at: Option<Instant>,
  /// The badge state observed at the end of the previous event drain, so the shell can act on a *transition*
  /// rather than on the steady state. Used to request `$G` exactly once when the firmware enters the `Tool`
  /// (M6 manual tool change) state, so the tool-change banner can name the tool the firmware is awaiting; the
  /// `<...>` status report carries no tool number, so the authoritative `T<n>` must be solicited via `$G`.
  last_badge: super::badge::BadgeState,
  /// The result channel of an in-flight on-demand port identify probe, if one is running. The probe runs on
  /// the runtime (off the UI thread); the verdict arrives here and is drained into the console each frame, so a
  /// 500ms probe never blocks rendering. `None` when no probe is in flight.
  #[cfg(feature = "serial")]
  pending_probe: Option<std::sync::mpsc::Receiver<String>>,
  /// The in-flight hardened Z touch-off, if one is running, or `None`. Tracks the zero line to send *only* on a
  /// successful probe plus the wall-clock instants that pace the push-or-poll fallback ([`PendingZeroZProbe`]).
  /// Held in the shell (not the reducer) because the timeouts are `Instant` work the egui frame owns; the
  /// success/failure *decision* is the pure [`super::probe_flow::decide`]. `None` once resolved or never started.
  pending_zero_z: Option<PendingZeroZProbe>,
  /// The active rotary center-finder run, or `None`. Holds the pure [`super::rotary_center::WizardState`] plus the
  /// shared bench [`super::rotary_probe::RotaryProbeParams`] for the run. The wizard is the follow-up owner for
  /// its probes: [`Self::pump_wizard`] folds each resolved Phase 0 latch result into the state machine. Held in
  /// app state (not persisted) — DOC-11 §1.3 flags cross-session persistence as a follow-up.
  wizard: Option<RotaryCenterRun>,
  /// The active datum-finder run (single-edge or corner), or `None`. Holds the pure [`super::datum::DatumState`]
  /// plus the shared bench [`super::datum::ProbeParams`]. The wizard owns its probes' follow-up:
  /// [`Self::pump_datum`] folds each resolved Phase 0 latch result into the state machine. Not persisted — a
  /// found datum is written straight to the WCS (only the bench params persist, via the profile).
  datum: Option<DatumRun>,
  /// The active height-map acquisition run, or `None`. Holds the pure [`super::autolevel::MeshProbeState`] plus its
  /// current point's lost-push fallback. [`Self::pump_mesh`] folds each resolved Z into the mesh; on completion the
  /// filled mesh is saved to [`crate::profile::Profile::mesh`] and the corrected-program cache is invalidated.
  mesh_probe: Option<MeshProbeRun>,
  /// The active Phase 2 angle-sweep run (180°-flip verify OR runout report), or `None`. Both wizards share one
  /// [`super::angle_sweep::AngleSweep`] engine and one kind-dispatched pump ([`Self::pump_sweep`]) rather than
  /// each owning a bespoke pump — they are the same "probe at a list of A angles, collect readings" shape.
  sweep: Option<SweepRun>,
  /// The persisted cross-session profile (DOC-11 §1.3): the rotary-A center and connection/UI defaults. Loaded
  /// once at startup (seeding [`UiState`]), mutated as the operator finds/writes a center or connects, and
  /// written back through [`crate::profile::save`] on those changes (with an exit backstop in `App::save`). The
  /// in-memory copy is the source of truth for a session; disk is the durable mirror, and a failed write is a
  /// surfaced notice, never a crash.
  profile: crate::profile::Profile,
  /// An explicit path to persist the profile to, overriding the default OS config location. `None` in production
  /// (the OS path is used); set in tests so they round-trip through a temp file instead of the operator's real
  /// `~/.config/skirnir/profile.ron`. The single seam that keeps the persistence wiring hermetically testable.
  profile_path_override: Option<std::path::PathBuf>,
  /// The startup-loaded app config (appearance/themes, UI defaults, connection/streaming, toolpath tuning). Loaded
  /// once in [`Self::new`] (seeding the resolved palette/toolpath style into [`UiState`] and the UI/connection
  /// defaults), and re-loadable on demand via [`Self::reload_config`] so an operator can iterate on `config.json`
  /// without restarting. The in-memory copy is the source of truth for a session; the file is the durable mirror.
  config: crate::config::Config,
  /// An explicit path to load/persist the config from, overriding the default OS location. `None` in production;
  /// set in tests so a config reload round-trips through a temp file rather than the operator's real
  /// `~/.config/skirnir/config.json`. The seam that keeps the config wiring hermetically testable, like
  /// [`Self::profile_path_override`].
  config_path_override: Option<std::path::PathBuf>,
  /// Whether the in-memory config carries edits (language/theme/font-scale changes from the app settings dialog)
  /// not yet written to `config.json`. Drives the dialog's unsaved-changes marker and its Save button; cleared by
  /// a successful [`Intent::SaveConfig`]. The file is operator-owned, so nothing writes it implicitly.
  config_dirty: bool,
  /// Whether an appearance edit this frame still needs the LIVE egui context re-skinned (`apply_theme` needs the
  /// `Context`, which intent handlers do not carry). The pure half — re-resolving the palette into `UiState` — is
  /// applied immediately in the handler; `ui()` consumes this flag right after the intent drain.
  appearance_dirty: bool,
}

/// Which probe-flow "slot" owns the shared latch, for the mutual-cancel guard. Exactly one may be armed at a
/// time; starting any flow cancels the others' pending follow-up so a stale one cannot act on a new `[PRB:]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ProbeOpSlot {
  /// The hardened Z touch-off (`pending_zero_z`).
  ZeroZ,
  /// The rotary center-finder (`wizard`).
  Wizard,
  /// The datum finder (`datum`) — single-edge or corner.
  Datum,
  /// The height-map acquisition grid probe (`mesh_probe`).
  Mesh,
  /// The Phase 2 angle-sweep (`sweep`) — flip-verify or runout.
  Sweep,
}

/// The shell-side bookkeeping for a height-map acquisition run: the pure [`super::autolevel::MeshProbeState`] plus
/// the current point's lost-push fallback. Mirrors [`DatumRun`]; all decisions live in the pure state machine.
struct MeshProbeRun {
  /// The pure acquisition state machine (mesh being filled, serpentine order, cursor, `Z₀`, step).
  state: super::autolevel::MeshProbeState,
  /// The CURRENT point's lost-push fallback (the same completion-gated push-or-poll-`$#`-or-give-up machinery the
  /// other probe flows use, shared via [`super::probe_flow::await_action`]). `None` between points.
  touch_fallback: Option<TouchFallback>,
}

/// The shell-side bookkeeping for a Phase 2 angle-sweep run: the shared pure sweep engine, the bench probe
/// params, which probe kind owns the latch (so the pump stays kind-routed), and the current touch's lost-push
/// fallback. The verified `wcs` is carried for the flip-verify's `G10` offer; runout writes nothing.
struct SweepRun {
  /// Which Phase 2 wizard this run is — selects the latch `ProbeKind` and the completion compute (flip vs runout).
  kind: super::view_state::ProbeKind,
  /// The shared multi-touch sweep engine (angles, axis/dir, collected readings, step).
  sweep: super::angle_sweep::AngleSweep,
  /// The bench-tuned clearance/settle/feed/depth shared across the run's touches.
  params: super::rotary_probe::RotaryProbeParams,
  /// The current touch's lost-push fallback (shared with the ZeroZ / center-finder flows via `await_action`).
  touch_fallback: Option<TouchFallback>,
}

/// The shell-side bookkeeping for a rotary center-finder run: the pure wizard state machine plus the bench-tuned
/// probe parameters shared across its touches. Kept in the shell so the egui frame owns it; all decisions live in
/// the pure [`super::rotary_center::WizardState`].
struct RotaryCenterRun {
  /// The pure wizard state machine (step, readings, computed center).
  state: super::rotary_center::WizardState,
  /// The bench-tuned clearance/settle/feed/depth shared across the run's touches and the move-to-Yc.
  params: super::rotary_probe::RotaryProbeParams,
  /// The CURRENT touch's lost-push fallback bookkeeping (the same completion-gated push-or-poll-`$#`-or-give-up
  /// machinery the ZeroZ flow uses, shared via [`super::probe_flow::await_action`]). `None` between touches; set
  /// when a touch is issued and cleared when it resolves. So a dropped/suppressed `[PRB:]` push cannot leave the
  /// wizard awaiting forever — it polls `$#` once the touch has finished, then gives up cleanly.
  touch_fallback: Option<TouchFallback>,
}

/// The shell-side bookkeeping for a datum-finder run: the pure [`super::datum::DatumState`] wizard plus the
/// bench-tuned [`super::datum::ProbeParams`] shared across its touches, and the current touch's lost-push
/// fallback. Mirrors [`RotaryCenterRun`]; all decisions live in the pure state machine.
struct DatumRun {
  /// The pure datum wizard state machine (step, target, captured contacts, computed datum).
  state: super::datum::DatumState,
  /// The bench-tuned clearances/feeds/tip-diameter/offset shared across the run's touches.
  params: super::datum::ProbeParams,
  /// The CURRENT touch's lost-push fallback (the same completion-gated push-or-poll-`$#`-or-give-up machinery the
  /// ZeroZ and rotary flows use, shared via [`super::probe_flow::await_action`]). `None` between touches.
  touch_fallback: Option<TouchFallback>,
}

/// One rotary touch's lost-push fallback state: whether `$#` has been polled, whether a probe cycle was observed
/// (the completion gate, mirroring [`super::probe_flow::PendingZeroZ`]'s `seen_cycle`), and the wall-clock stamps
/// pacing the timeouts. Reset per touch.
struct TouchFallback {
  /// When the touch's probe lines were sent, pacing the push timeout before a `$#` poll.
  issued_at: Instant,
  /// Whether the `$#` fallback poll has been sent (so the give-up deadline then runs off `polled_at`).
  polled: bool,
  /// When the `$#` poll was sent, or `None` until it is, pacing the give-up deadline after a poll.
  polled_at: Option<Instant>,
  /// Whether the machine has been observed in a cycle since the touch was issued (the completion gate).
  seen_cycle: bool,
}

/// The shell-side bookkeeping for an in-flight hardened Z touch-off: the pure [`super::probe_flow::PendingZeroZ`]
/// follow-up plus the wall-clock stamps the egui frame uses to pace the push-or-poll fallback. Kept out of the
/// pure reducer because `Instant` is frame-owned state.
struct PendingZeroZProbe {
  /// The pure follow-up state: the zero line to send on success and whether `$#` was already polled.
  inner: super::probe_flow::PendingZeroZ,
  /// When the probe line was sent, pacing the push timeout before a `$#` poll.
  issued_at: Instant,
  /// When the `$#` poll was sent, or `None` until it is, pacing the give-up deadline after a poll.
  polled_at: Option<Instant>,
}

/// A short human word for a jog/approach direction, for datum-run notices (`+` vs `−`).
fn dir_word(dir: Dir) -> &'static str {
  match dir {
    Dir::Pos => "positive",
    Dir::Neg => "negative",
  }
}

/// Whether a machine [`RunState`](crate::protocol::RunState) represents an active probe CYCLE — the machine is
/// moving or paused mid-program (`Run`/`Hold`/`Jog`/`Home`), as opposed to settled (`Idle`) or faulted
/// (`Alarm`/...). The hardened touch-off uses this to gate the `$#` lost-push fallback: the fallback runs only
/// once the machine has been in a cycle and then returned to `Idle`, so a probe still travelling — which can
/// legitimately exceed the push timeout — is never polled mid-cycle on the previous probe's stale result.
fn is_probe_cycle_state(state: crate::protocol::RunState) -> bool {
  use crate::protocol::RunState;
  // A probe is "finished" ONLY on a clean return to `Idle`. Every other non-idle state is treated as in-cycle so
  // the lost-push `$#` fallback never arms while the machine is still busy or merely SUSPENDED: `Door` (safety-door
  // suspend), `Sleep` (`$SLP`), and `Tool` (tool-change wait) can interrupt a probe mid-flight, and reading them as
  // "finished" would poll `$#` for the previous probe's stale result. A genuine probe FAILURE (alarm/error) already
  // resolves the latch via the Alarm/Error path, so `probe_finished` is never consulted on that path. `Check` and
  // `Unknown` are likewise treated as not-finished (conservative). Only `Idle` is a clean completion.
  !matches!(state, RunState::Idle)
}

/// The `SKIRNIR_SIZE_TRACE=1` frame hook: print the dock region's rect (recorded by [`views::dock`] via
/// [`super::views::dock_rect_probe`]) whenever it changes between frames, with the frame counter and the live
/// `pixels_per_point`. Chatty by design — an unsolicited line here IS the self-resizing bug reproducing in the
/// real eframe loop, which the offscreen harness failed to catch three times. Debug instrumentation only.
fn size_trace(ctx: &egui::Context, ui_state: &UiState) {
  use std::sync::Mutex;
  use std::sync::atomic::{AtomicU64, Ordering};
  static FRAME: AtomicU64 = AtomicU64::new(0);
  static LAST: Mutex<Option<egui::Rect>> = Mutex::new(None);
  let frame = FRAME.fetch_add(1, Ordering::Relaxed);
  let now = super::views::dock_rect_probe::last(ctx);
  let mut last = LAST.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
  if let Some(now) = now
    && *last != Some(now)
  {
    eprintln!(
      "[size-trace] frame {frame}: dock rect {:?} -> [{:.3},{:.3}]..[{:.3},{:.3}] h={:.3} (ppp {}, collapsed {})",
      last.map(|r| r.height()),
      now.min.x, now.min.y, now.max.x, now.max.y,
      now.height(),
      ctx.pixels_per_point(),
      ui_state.dock_collapsed,
    );
    *last = Some(now);
  }
}

/// Build the tokio runtime and launch the eframe window. This is the binary's GUI entry point; `expect` is
/// acceptable here because a failure to build the runtime or the window is genuinely unrecoverable at startup.
pub fn run() -> eframe::Result<()> {
  let runtime = tokio::runtime::Builder::new_multi_thread()
    .enable_all()
    .build()
    .expect("failed to build the tokio runtime");

  // Load the app config ONCE here: it sets the initial window size and the palette the very first frame paints, so
  // it must be resolved before the viewport and the visuals are built. Load failures fall back to defaults with
  // notices. The parsed config + its notices are handed to `SkirnirApp::new`, so the normal path reads/parses/
  // resolves exactly once (no duplicate load) and the notices still reach the console.
  let (config, mut config_notices) = crate::config::load();

  // Bring the i18n registry up before the first frame: seed the bundled locales (en-US source + fallback), then
  // select the operator's configured locale so every `tr!` label resolves against it. An init failure means the
  // BUNDLED resource is invalid (a build problem) — surfaced as a console notice, never a panic, and `tr!` then
  // renders keys verbatim rather than bringing the UI down.
  if let Err(err) = crate::i18n::init() {
    config_notices.push(format!("i18n initialisation failed (UI strings will show as keys): {err}"));
  }
  crate::i18n::set_language(&config.ui.language);

  let (palette, _palette_notice) = config.palette();
  // Clamp the window size to sane positive dimensions (a hand-edited 0/negative would make the window unusable).
  let (window_w, window_h) = config.ui.window_size();

  let options = eframe::NativeOptions {
    viewport: egui::ViewportBuilder::default()
      .with_inner_size([window_w, window_h])
      .with_min_inner_size([800.0, 500.0]),
    ..Default::default()
  };

  eframe::run_native("skirnir", options, Box::new(move |cc| {
    // Install the vendored Roboto + JetBrains Mono faces before the theme so the first frame already renders in
    // the design's typefaces — Roboto for UI text, JetBrains Mono (tabular) for the DRO digits and console.
    super::fonts::install(&cc.egui_ctx);
    apply_theme(&cc.egui_ctx, &palette, config.appearance.font_scale);
    Ok(Box::new(SkirnirApp::new(runtime, config, config_notices)))
  }))
}

/// Apply the config's resolved APPEARANCE onto a [`UiState`]: the palette (active theme) + the toolpath render style,
/// threaded into the views. This is the only part safe to re-apply on a live reload — it is pure presentation and
/// carries no operator session state. Pure (no I/O, no egui context) so the config→style mapping is unit-tested
/// without a window. Re-applied on BOTH startup and `reload_config`.
fn apply_appearance(ui: &mut UiState, config: &crate::config::Config) {
  let (palette, _theme_notice) = config.palette();
  ui.style = super::views::RuntimeStyle { palette, toolpath: config.toolpath_style() };
}

/// Apply the config's from-scratch UI DEFAULTS onto a [`UiState`]: the jog/DRO/console knobs an operator would
/// otherwise re-set each launch. Applied ONLY at startup (in `SkirnirApp::new`), never on a live reload — a reload
/// must not wipe the operator's in-session changes to these (the reported F5 bug). Deliberately does NOT touch the
/// profile-seeded "remembered last entry" fields (port/baud/rotary inputs) — those are restored from `profile.ron`
/// and must win over a config default. Pure so the mapping is unit-tested without a window.
fn apply_ui_defaults(ui: &mut UiState, config: &crate::config::Config) {
  ui.jog_step = config.ui.jog_step_mm;
  ui.jog_feed = config.ui.jog_feed;
  ui.jog_continuous = config.ui.jog_continuous;
  ui.show_machine_pos = config.ui.dro_show_machine;
  ui.verbose = config.ui.console_verbose;
  ui.auto_scroll = config.ui.console_auto_scroll;
}

#[cfg(test)]
mod config_wiring_tests {
  use super::*;

  #[test]
  fn startup_apply_sets_the_ui_defaults_and_resolved_appearance() {
    // The startup config→UI mapping: a non-default config must drive the from-scratch UI knobs AND resolve the
    // active theme's palette + toolpath style into `UiState.style`. (Startup applies both halves; reload only the
    // appearance — see the reload test below.)
    let mut config = crate::config::Config::default();
    config.ui.jog_step_mm = 0.05;
    config.ui.jog_feed = 250.0;
    config.ui.jog_continuous = true;
    config.ui.dro_show_machine = true;
    config.ui.console_verbose = true;
    config.ui.console_auto_scroll = false;
    config.appearance.active_theme = "midnight".to_string();
    config.toolpath.cut_stroke_px = 4.0;

    let mut ui = UiState::default();
    apply_appearance(&mut ui, &config);
    apply_ui_defaults(&mut ui, &config);

    assert_eq!(ui.jog_step, 0.05, "the jog step comes from config");
    assert_eq!(ui.jog_feed, 250.0);
    assert!(ui.jog_continuous, "continuous jog default comes from config");
    assert!(ui.show_machine_pos, "DRO machine-pos default comes from config");
    assert!(ui.verbose, "console verbose default comes from config");
    assert!(!ui.auto_scroll, "console auto-scroll default comes from config");
    assert_eq!(ui.style.palette, Palette::midnight(), "the resolved palette is the active theme's");
    assert!((ui.style.toolpath.cut_stroke_px - 4.0).abs() < 1e-6, "the toolpath style is resolved from config");
  }

  #[test]
  fn startup_apply_leaves_the_profile_seeded_fields_untouched() {
    // The profile owns the "remembered last entry" fields (port/baud/rotary); the config mapping must NOT clobber
    // them, so a remembered port/baud survives applying the config defaults over a `from_prefs` UI.
    let prefs = crate::profile::Prefs {
      last_port: Some("/dev/ttyACM0".to_string()),
      baud: 250_000,
      ..crate::profile::Prefs::default()
    };
    let mut ui = UiState::from_prefs(&prefs);
    apply_appearance(&mut ui, &crate::config::Config::default());
    apply_ui_defaults(&mut ui, &crate::config::Config::default());
    assert_eq!(ui.selected_port, "/dev/ttyACM0", "the remembered port must survive the config apply");
    assert_eq!(ui.baud, 250_000, "the remembered baud must survive the config apply");
  }

  #[test]
  fn reload_applies_appearance_but_preserves_operator_modified_ui_state() {
    // The reported F5 bug: a reload must update the palette/toolpath style WITHOUT wiping the operator's in-session
    // jog/console changes. We simulate startup (defaults), then the operator changes jog_step + verbose, then a
    // reload with a different active theme. The split functions model the two code paths: reload calls
    // `apply_appearance` only, NOT `apply_ui_defaults`.
    let config = crate::config::Config::default();
    let mut ui = UiState::default();
    apply_appearance(&mut ui, &config);
    apply_ui_defaults(&mut ui, &config);

    // Operator changes session knobs away from the config defaults.
    ui.jog_step = 0.123;
    ui.verbose = !config.ui.console_verbose;
    let operator_jog = ui.jog_step;
    let operator_verbose = ui.verbose;

    // A reload with a new theme: appearance ONLY.
    let mut reloaded = config.clone();
    reloaded.appearance.active_theme = "midnight".to_string();
    apply_appearance(&mut ui, &reloaded);

    assert_eq!(ui.style.palette, Palette::midnight(), "the reload must update the palette");
    assert_eq!(ui.jog_step, operator_jog, "the reload must NOT reset the operator's jog step (the F5 bug)");
    assert_eq!(ui.verbose, operator_verbose, "the reload must NOT reset the operator's console verbose toggle");
  }

  #[test]
  fn reflow_toolpath_re_flattens_the_loaded_program_at_a_new_arc_density() {
    // F5 with a changed `arc_step_deg` must re-flatten the cached arcs. Load a program with a G2 arc at a coarse
    // step, capture the chord count, then tighten the step and `reflow_toolpath` — the arc must subdivide into more
    // chords, proving the cached geometry tracked the new resolution without a GCode reload.
    let program = vec!["G0 X10 Y0".to_string(), "G2 X0 Y10 I-10 J0".to_string()];

    let mut coarse = UiState::default();
    coarse.style.toolpath = crate::config::ToolpathConfig { arc_step_deg: 45.0, ..Default::default() }.resolve();
    coarse.set_program(program.clone(), None);
    let coarse_segments = coarse.toolpath_segment_count();

    let mut fine = UiState::default();
    fine.style.toolpath = crate::config::ToolpathConfig { arc_step_deg: 45.0, ..Default::default() }.resolve();
    fine.set_program(program, None);
    // Now tighten the arc step and re-flow (the reload path) — no new `set_program`.
    fine.style.toolpath = crate::config::ToolpathConfig { arc_step_deg: 3.0, ..Default::default() }.resolve();
    fine.reflow_toolpath();

    assert!(
      fine.toolpath_segment_count() > coarse_segments,
      "a finer arc step must re-flatten the arc into MORE chords ({} vs {})",
      fine.toolpath_segment_count(),
      coarse_segments,
    );
  }

  #[test]
  fn the_config_reconnect_section_round_trips_to_the_engine_policy() {
    // The connection wiring: the config's reconnect knobs must convert back to the exact engine `ReconnectConfig`
    // the shell builds its policy from, so a config-tuned backoff is honoured.
    let mut config = crate::config::Config::default();
    config.connection.reconnect.base_ms = 750;
    config.connection.reconnect.max_attempts = 9;
    let policy_config = config.connection.reconnect.to_policy_config();
    assert_eq!(policy_config.base, std::time::Duration::from_millis(750));
    assert_eq!(policy_config.max_attempts, 9);
  }

  #[test]
  fn an_out_of_range_config_default_baud_is_sanitized_before_it_can_reach_the_port_open() {
    // The startup baud-seed path runs the config's `default_baud` through `sanitize_baud` (the same clamp every
    // other baud path uses), so a hand-edited `0`/absurd value is held to the accepted range rather than reaching
    // `SerialTransport::open` and failing the port open. This pins that contract on the function the shell calls.
    assert_eq!(crate::app::views::sanitize_baud(0), 115_200, "a zero default_baud must clamp to the fallback");
    assert_eq!(
      crate::app::views::sanitize_baud(9_999_999), 115_200,
      "an above-range default_baud must clamp to the fallback",
    );
    assert_eq!(crate::app::views::sanitize_baud(250_000), 250_000, "an in-range default_baud passes through");
  }

  #[test]
  fn apply_theme_wires_the_hover_and_active_accent_edges_so_those_tokens_have_effect() {
    // `accent_hover`/`accent_active` must reach a real egui surface (the hovered/active widget edge strokes),
    // otherwise they are inert config knobs. Apply a palette with sentinel hover/active accents and read them back
    // out of the resolved visuals to prove the wiring.
    let mut palette = Palette::default_dark();
    palette.accent_hover = egui::Color32::from_rgb(0x11, 0x22, 0x33);
    palette.accent_active = egui::Color32::from_rgb(0x44, 0x55, 0x66);

    let ctx = egui::Context::default();
    apply_theme(&ctx, &palette, 1.0);

    let visuals = ctx.global_style().visuals.clone();
    assert_eq!(
      visuals.widgets.hovered.bg_stroke.color, palette.accent_hover,
      "accent_hover must drive the hovered widget edge",
    );
    assert_eq!(
      visuals.widgets.active.bg_stroke.color, palette.accent_active,
      "accent_active must drive the active/pressed widget edge",
    );
  }
}

/// Apply the visuals so the app matches the active [`Palette`], mapping the design tokens onto egui's
/// `Visuals.widgets.{noninteractive,inactive,hovered,active,open}` plus the panel/window/inset fills, 2px control
/// rounding, and the 1px divider stroke. The design maps 1:1 onto these fields. `font_scale` (1.0 = the design's
/// sizes) is applied as the global zoom, held to a sane range so a config typo cannot make the UI unreadable.
/// Driven by the config-resolved palette so a theme change re-skins the whole window from one call.
/// `pub(crate)` so the snapshot harness can skin its offscreen context identically to the real window.
pub(crate) fn apply_theme(ctx: &egui::Context, palette: &Palette, font_scale: f32) {
  use eframe::egui::{CornerRadius, Stroke};
  use super::metrics::Metrics;

  // Spacing first, so every default-sized button/field matches the design's component sheet rather than egui's
  // larger defaults. The design's controls are 6×14 padding, ~22px tall in panels, 2px corners.
  let mut style = (*ctx.global_style()).clone();
  let spacing = &mut style.spacing;
  spacing.button_padding = Metrics::BUTTON_PAD; // §02 buttons: 6px 14px (egui order is x,y).
  spacing.item_spacing = egui::vec2(6.0, 6.0); // §01 default cluster pad.
  spacing.interact_size.y = Metrics::PANEL_CONTROL_H; // §01 spacing legend: 22px control row.
  ctx.set_global_style(style);
  // Apply the UI font scale as the global zoom; clamp so a hand-edited extreme cannot shrink/blow up the UI.
  ctx.set_zoom_factor(crate::config::clamp_font_scale(font_scale));

  let mut visuals = egui::Visuals::dark();
  visuals.panel_fill = palette.panel;
  visuals.window_fill = palette.bg;
  visuals.extreme_bg_color = palette.inset; // text edits / inset fields.
  visuals.faint_bg_color = palette.panel_alt; // striped rows / faint surfaces.
  visuals.override_text_color = Some(palette.text);
  visuals.hyperlink_color = palette.accent;
  visuals.selection.bg_fill = palette.accent.gamma_multiply(0.4);
  visuals.selection.stroke = Stroke::new(1.0, palette.accent);
  visuals.window_stroke = Stroke::new(1.0, palette.divider);

  let radius = CornerRadius::same(2);
  let widgets = &mut visuals.widgets;
  // Non-interactive surfaces (labels, separators): panel fill, divider stroke.
  widgets.noninteractive.bg_fill = palette.panel;
  widgets.noninteractive.weak_bg_fill = palette.panel;
  widgets.noninteractive.bg_stroke = Stroke::new(1.0, palette.divider);
  widgets.noninteractive.fg_stroke = Stroke::new(1.0, palette.text_dim);
  widgets.noninteractive.corner_radius = radius;
  // Inactive (control at rest): widget fill, raised edge.
  widgets.inactive.bg_fill = palette.widget;
  widgets.inactive.weak_bg_fill = palette.widget;
  widgets.inactive.bg_stroke = Stroke::new(1.0, palette.border_raised);
  widgets.inactive.fg_stroke = Stroke::new(1.0, palette.text);
  widgets.inactive.corner_radius = radius;
  // Hovered: the hover-accent edge highlight (`accent_hover`) so a pointed-at control lifts toward the accent.
  widgets.hovered.bg_fill = palette.widget_hover;
  widgets.hovered.weak_bg_fill = palette.widget_hover;
  widgets.hovered.bg_stroke = Stroke::new(1.0, palette.accent_hover);
  widgets.hovered.fg_stroke = Stroke::new(1.0, palette.text);
  widgets.hovered.corner_radius = radius;
  // Active / pressed: the pressed-accent edge (`accent_active`), a notch deeper than the hover accent.
  widgets.active.bg_fill = palette.widget_active;
  widgets.active.weak_bg_fill = palette.widget_active;
  widgets.active.bg_stroke = Stroke::new(1.0, palette.accent_active);
  widgets.active.fg_stroke = Stroke::new(1.0, palette.text);
  widgets.active.corner_radius = radius;
  // Open (combo box popups): match active.
  widgets.open.bg_fill = palette.widget_active;
  widgets.open.weak_bg_fill = palette.widget_active;
  widgets.open.bg_stroke = Stroke::new(1.0, palette.border_raised);
  widgets.open.fg_stroke = Stroke::new(1.0, palette.text);
  widgets.open.corner_radius = radius;

  ctx.set_visuals(visuals);
}

#[cfg(all(test, feature = "serial"))]
mod tests {
  use super::*;
  use crate::protocol::ConnectionState;
  use crate::transport::loopback::{LoopbackController, LoopbackTransport};
  use std::time::Duration;

  /// Build an app with a live engine wired to an in-memory loopback transport, as if the operator had just
  /// connected. Returns the controller so the test can inject "firmware" bytes the engine will read.
  fn app_with_engine() -> (SkirnirApp, LoopbackController) {
    let runtime = tokio::runtime::Builder::new_multi_thread().enable_all().build().expect("runtime");
    let (transport, controller) = LoopbackTransport::new();
    // `Engine::connect` spawns the driver task, so it must run inside the runtime context.
    let handle = {
      let _guard = runtime.enter();
      Engine::connect(transport)
    };
    let mut app = SkirnirApp::new(runtime, crate::config::Config::default(), Vec::new());
    app.engine = Some(handle);
    // Keep these tests hermetic: `new` reads the real OS profile, so reset to a clean default and redirect every
    // save to a unique temp file so no test ever touches the operator's config dir. A persistence test reads this
    // path back; others simply never pollute `~/.config/skirnir`. A nanosecond-stamped name keeps runs distinct.
    app.profile = crate::profile::Profile::default();
    let stamp = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos()).unwrap_or(0);
    let temp = std::env::temp_dir().join(format!("skirnir-test-profile-{}-{stamp}", std::process::id()));
    app.profile_path_override = Some(temp.join("profile.ron"));
    // Mirror a user connect: the desire is armed and an endpoint recorded, so the test proves a deliberate
    // disconnect clears them rather than scheduling a reconnect.
    app.auto_reconnect = true;
    app.last_endpoint = Some(("loopback".to_string(), 115_200));
    (app, controller)
  }

  /// Pump engine events repeatedly — the driver task runs on background runtime threads, so events arrive
  /// asynchronously — until `predicate` holds or a short budget is exhausted. Returns whether it was met.
  fn pump_until(app: &mut SkirnirApp, predicate: impl Fn(&SkirnirApp) -> bool) -> bool {
    for _ in 0..200 {
      app.pump_events();
      if predicate(app) {
        return true;
      }
      std::thread::sleep(Duration::from_millis(5));
    }
    predicate(app)
  }

  /// A user-initiated disconnect must drive the view all the way to `Disconnected`, even when the firmware is
  /// latched in `Alarm`. Regression: nulling the engine handle inside `disconnect` dropped the event receiver
  /// before the engine's terminal `Disconnected` could be drained, so the UI stayed stuck in its last state.
  #[test]
  fn a_user_disconnect_from_alarm_drives_the_view_to_disconnected() {
    let (mut app, controller) = app_with_engine();
    // Drive the firmware into Alarm so the lifecycle latches there — the operator's "stuck in alarm" start.
    assert!(controller.inject_line("<Alarm:1|MPos:0.000,0.000,0.000>"));
    assert!(
      pump_until(&mut app, |a| a.view.connection == ConnectionState::Alarm),
      "the engine never reported Alarm; got {:?}",
      app.view.connection,
    );

    // The operator clicks Disconnect.
    app.disconnect();

    // The engine's terminal `Disconnected` event must be drained and applied: the view leaves Alarm for
    // Disconnected and the dead handle is released without arming a reconnect.
    assert!(
      pump_until(&mut app, |a| a.view.connection == ConnectionState::Disconnected),
      "the view stayed in {:?}; a user disconnect must reach Disconnected",
      app.view.connection,
    );
    assert!(app.engine.is_none(), "the engine handle must be released once the terminal event drains");
    assert!(!app.auto_reconnect, "a deliberate disconnect must not re-arm auto-reconnect");
    assert!(app.reconnect_at.is_none(), "a deliberate disconnect must not schedule a reconnect");
  }

  /// App-settings intents mutate the config, mark it dirty, and (for appearance) re-resolve the palette into the
  /// view state synchronously — the ctx-level re-skin is deferred to `ui()` via `appearance_dirty`, but the pure
  /// half must be observable immediately so the next frame's views already render the new colours.
  #[test]
  fn a_theme_upsert_does_not_reflow_the_toolpath() {
    // A colour-picker drag emits `UpsertTheme` every changed frame; that must NOT re-flatten the whole program
    // (a full `parse_xy_path` over every line) — colour edits never change geometry. We flatten an arc at a COARSE
    // density, then make the config's arc density much finer so a reflow WOULD be detectable (more chords), then
    // fire a theme upsert. The cached geometry must stay coarse, proving the appearance path no longer reflows.
    let (mut app, _controller) = app_with_engine();
    app.ui.style.toolpath = crate::config::ToolpathConfig { arc_step_deg: 45.0, ..Default::default() }.resolve();
    app.ui.set_program(vec!["G0 X10 Y0".to_string(), "G2 X0 Y10 I-10 J0".to_string()], None);
    let before = app.ui.toolpath_segment_count();

    // A finer arc density in the config: were the appearance path to reflow, it would re-flatten the arc into MANY
    // more chords at this density — so an unchanged count is proof the theme edit did not reparse the program.
    app.config.toolpath.arc_step_deg = 3.0;
    let theme = crate::config::ThemeOverride::from_palette(&crate::app::theme::Palette::midnight());
    app.handle_intent(Intent::UpsertTheme { name: "fixture".to_string(), theme });

    assert_eq!(
      app.ui.toolpath_segment_count(), before,
      "a theme upsert must not re-flatten the toolpath at the config's (now finer) arc density",
    );
  }

  #[test]
  fn appearance_intents_update_the_config_and_resolve_the_palette_immediately() {
    // This test drives `SetLanguage`, which mutates the process-global i18n registry — serialize on the shared guard
    // so it can never race the i18n module's own global-locale tests (finding: several independent locks).
    let _lang = crate::i18n::lock_global_for_test();
    let (mut app, _controller) = app_with_engine();
    assert!(!app.config_dirty, "a fresh app starts with no unsaved config edits");

    app.handle_intent(Intent::SetActiveTheme("midnight".to_string()));
    assert_eq!(app.config.appearance.active_theme, "midnight");
    assert_eq!(app.ui.style.palette, crate::app::theme::Palette::midnight(), "the palette re-resolves in-handler");
    assert!(app.config_dirty && app.appearance_dirty, "an appearance edit marks both flags");

    // The font scale is held to the range the window can actually render at (a wild value would blow up zoom).
    app.handle_intent(Intent::SetFontScale(9.0));
    assert_eq!(app.config.appearance.font_scale, 2.5, "the scale clamps to the apply_theme range");

    // A created user theme lands in the config and, once active, wins resolution over its base.
    let theme = crate::config::ThemeOverride::from_palette(&crate::app::theme::Palette::light_slate());
    app.handle_intent(Intent::UpsertTheme { name: "fixture-theme".to_string(), theme });
    app.handle_intent(Intent::SetActiveTheme("fixture-theme".to_string()));
    assert_eq!(app.ui.style.palette, crate::app::theme::Palette::light_slate(), "the user theme resolves");

    // The language choice is recorded in the config (the global i18n switch is exercised by the i18n tests; using
    // the default locale here keeps this test from mutating shared global state under a parallel runner).
    app.handle_intent(Intent::SetLanguage("en-US".to_string()));
    assert_eq!(app.config.ui.language, "en-US");
  }

  /// `SaveConfig` writes the in-memory config to the (test-overridden) path atomically, clears the dirty marker,
  /// and the file round-trips to the same config.
  #[test]
  fn save_config_persists_to_the_override_path_and_clears_dirty() {
    let (mut app, _controller) = app_with_engine();
    let stamp = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_nanos();
    let dir = std::env::temp_dir().join(format!("skirnir-test-config-{}-{stamp}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("temp dir");
    let path = dir.join("config.json");
    app.config_path_override = Some(path.clone());

    app.handle_intent(Intent::SetActiveTheme("midnight".to_string()));
    assert!(app.config_dirty);
    app.handle_intent(Intent::SaveConfig);
    assert!(!app.config_dirty, "a successful save clears the unsaved marker");

    let (loaded, notices) = crate::config::load_from(&path);
    assert!(notices.is_empty(), "the saved file must load back cleanly: {notices:?}");
    assert_eq!(loaded.appearance.active_theme, "midnight", "the saved file carries the edit");
    let _ = std::fs::remove_dir_all(&dir);
  }

  /// The tool the tool-change banner would name: the firmware-reported `current_tool`, the single authoritative
  /// source the banner reads directly. A test helper so a test can assert the banner's data source without driving
  /// a real egui frame (the banner is a pure render of this value).
  fn banner_tool(app: &SkirnirApp) -> Option<u32> {
    app.view.current_tool
  }

  /// THE STREAMING-HOLD CASE under the new contract: the firmware answers `$G` DURING the M6 hold (including a hold
  /// reached mid-stream), so the banner names the tool from that `[GC:...]` answer — no program-stream derivation.
  /// We stream a program, enter the Tool hold mid-stream, let the shell's `$G` nudge fire, and answer it during the
  /// hold; the banner must then name the reported tool.
  #[test]
  fn the_tool_change_banner_names_the_tool_from_the_g_answer_during_a_streaming_hold() {
    // The final assertion reads the banner copy through `tr!`, so pin the global locale to en-US under the shared
    // guard: this test must see the English string regardless of any concurrent locale-switching test.
    let _lang = crate::i18n::lock_global_for_test();
    let _ = crate::i18n::init();
    let (mut app, mut controller) = app_with_engine();
    // Complete the handshake to Idle (the banner is the readiness signal), then drain its writes (including the
    // connect-time `$G` seed) so we assert only on the traffic the hold provokes.
    assert!(controller.inject_line("GrblHAL 1.1f ['$' or '$HELP' for help]"));
    assert!(pump_until(&mut app, |a| a.view.connection == ConnectionState::Idle));
    let _ = pump_collect(&mut app, &mut controller);

    // Stream a program through the engine for real, so the streaming state is the engine's own.
    let program = vec![
      "G21 G90".to_string(),
      "T2 M6".to_string(),
      "G1 X10 F300".to_string(),
      "G1 X20".to_string(),
    ];
    app.ui.set_program(program.clone(), None);
    app.start_stream();
    assert!(
      pump_until(&mut app, |a| a.view.connection == ConnectionState::Streaming),
      "the engine never reached Streaming; got {:?}",
      app.view.connection,
    );
    assert!(controller.inject_line("ok")); // ack the first streamed line so the cursor advances.
    let _ = pump_collect(&mut app, &mut controller);

    // The firmware halts mid-stream on the M6 tool change.
    assert!(controller.inject_line("<Tool|MPos:0.000,0.000,0.000|FS:0,0>"));
    assert!(
      pump_until(&mut app, |a| a.view.badge_state() == super::super::badge::BadgeState::Tool),
      "the engine never reached the Tool badge state; got {:?}",
      app.view.badge_state(),
    );
    assert_eq!(app.view.connection, ConnectionState::Streaming, "the Tool hold must keep the lifecycle streaming");

    // The shell nudges `$G` on entering Tool — even mid-stream — and the firmware ANSWERS it during the hold (the
    // M0/M1/M6 `$G`-in-hold support). The answer's `T2` populates `current_tool`, which the banner names.
    let written = String::from_utf8_lossy(&pump_collect(&mut app, &mut controller)).to_string();
    assert!(written.contains("$G"), "entering Tool must nudge $G even mid-stream; wrote {written:?}");
    assert!(controller.inject_line("[GC:G0 G54 G17 G21 G90 G94 M5 M9 T2 G49 F0 S0]"));
    assert!(
      pump_until(&mut app, |a| a.view.current_tool == Some(2)),
      "the [GC:] answer's T2 must set the active tool during the hold; got {:?}",
      app.view.current_tool,
    );
    assert_eq!(banner_tool(&app), Some(2), "the banner names the firmware-reported tool from the in-hold $G answer");
    assert_eq!(
      views::tool_change_headline(banner_tool(&app)),
      "🔧 Tool change: insert T2, then Resume",
      "the banner copy names the firmware-reported tool",
    );
  }

  /// THE NON-STREAMING CASE: an M6 issued from the console / MDI while idle. Same single source — the shell nudges
  /// `$G` on entering `Tool`, the firmware answers with `[GC:...]`, and the banner names the reported tool.
  #[test]
  fn the_tool_change_banner_names_the_g_reported_tool_when_not_streaming() {
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    // No program is streaming. The firmware reports the Tool hold.
    assert!(controller.inject_line("<Tool|MPos:0.000,0.000,0.000|FS:0,0>"));
    assert!(
      pump_until(&mut app, |a| a.view.badge_state() == super::super::badge::BadgeState::Tool),
      "the engine never reached the Tool badge state; got {:?}",
      app.view.badge_state(),
    );

    // The shell nudges `$G` on entering Tool. The write reaches the loopback on the engine's background task, so
    // collect across a short window.
    let written = String::from_utf8_lossy(&pump_collect(&mut app, &mut controller)).to_string();
    assert!(written.contains("$G"), "entering Tool must nudge $G; wrote {written:?}");

    // The firmware answers; the reported tool populates and the banner names it.
    assert!(controller.inject_line("[GC:G0 G54 G17 G21 G90 G94 M5 M9 T3 G49 F0 S0]"));
    assert!(
      pump_until(&mut app, |a| a.view.current_tool == Some(3)),
      "the [GC:] answer's T3 must set the active tool; got {:?}",
      app.view.current_tool,
    );
    assert_eq!(banner_tool(&app), Some(3), "the banner names the firmware-reported tool");
  }

  /// Becoming ready (the connect handshake reaching a live state) must seed `current_tool` with a single `$G`, so
  /// the DRO tool strip is populated BEFORE any first M6 — not left blank until a tool change.
  #[test]
  fn becoming_ready_seeds_the_active_tool_with_a_single_g_request() {
    let (mut app, mut controller) = app_with_engine();

    // Drive the handshake to a live (Idle) state; the readiness edge must provoke exactly one `$G` seed.
    assert!(controller.inject_line("GrblHAL 1.1f ['$' or '$HELP' for help]"));
    assert!(pump_until(&mut app, |a| a.view.connection == ConnectionState::Idle));
    let written = String::from_utf8_lossy(&pump_collect(&mut app, &mut controller)).to_string();
    assert!(written.contains("$G"), "becoming ready must seed the tool with a $G; wrote {written:?}");

    // The firmware answers and the DRO source is populated before any tool change.
    assert!(controller.inject_line("[GC:G0 G54 G17 G21 G90 G94 M5 M9 T1 G49 F0 S0]"));
    assert!(
      pump_until(&mut app, |a| a.view.current_tool == Some(1)),
      "the seed $G answer must populate the active tool; got {:?}",
      app.view.current_tool,
    );
  }

  /// The `$G` request fires once per tool change, not on every frame the machine sits in `Tool`. Regression guard
  /// for gating on the steady state instead of the transition edge, which would spam `$G` while held.
  #[test]
  fn the_tool_state_requests_g_only_on_the_transition_not_every_frame() {
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    assert!(controller.inject_line("<Tool|MPos:0.000,0.000,0.000|FS:0,0>"));
    assert!(pump_until(&mut app, |a| a.view.badge_state() == super::super::badge::BadgeState::Tool));
    // Let the one provoked `$G` fully flush to the loopback, then discard it (and any status polls). Only after
    // the transition's request has drained do we open the assertion window, so we measure repeats, not the first.
    let _ = pump_collect(&mut app, &mut controller);
    // Keep the machine in Tool across more reports and confirm no further `$G` is issued while it stays held.
    for _ in 0..5 {
      assert!(controller.inject_line("<Tool|MPos:0.000,0.000,0.000|FS:0,0>"));
    }
    let after = String::from_utf8_lossy(&pump_collect(&mut app, &mut controller)).to_string();
    assert!(!after.contains("$G"), "$G must not repeat while the machine stays in Tool; wrote {after:?}");
  }

  /// Pump a short window, accumulating everything written, so a test can assert on the traffic a steady state did
  /// (or did not) provoke. Mirrors [`collect_written`] but drives the general event pump, not the jog-stream pump.
  fn pump_collect(app: &mut SkirnirApp, controller: &mut LoopbackController) -> Vec<u8> {
    let mut out = Vec::new();
    for _ in 0..20 {
      app.pump_events();
      out.extend(controller.drain_written());
      std::thread::sleep(Duration::from_millis(5));
    }
    out
  }

  /// Let the connect handshake's writes flush, then discard everything written so far so a test sees only the
  /// traffic it provokes afterward.
  fn flush_handshake(app: &mut SkirnirApp, controller: &mut LoopbackController) {
    for _ in 0..20 {
      app.pump_events();
      std::thread::sleep(Duration::from_millis(5));
    }
    controller.drain_written();
  }

  /// Drive the jog-stream pump across a short window, accumulating everything the engine writes to the transport.
  /// Drains the controller each step so nothing is lost; sleeps so the wall-clock cadence and async writes advance.
  fn collect_written(app: &mut SkirnirApp, controller: &mut LoopbackController, steps: usize) -> Vec<u8> {
    let mut out = Vec::new();
    for _ in 0..steps {
      app.pump_jog_stream();
      out.extend(controller.drain_written());
      std::thread::sleep(Duration::from_millis(5));
    }
    out
  }

  /// A held continuous jog must stream repeated SHORT `$J=` increments (so a jog-cancel stops within one block),
  /// and releasing must end the stream and inject jog-cancel (`0x85`). Regression: the old one-shot 10 m move ran
  /// to its far boundary because jog-cancel only stops at a block boundary, so a hold-then-release jogged endlessly.
  #[test]
  fn a_held_continuous_jog_streams_short_increments_and_cancels_on_release() {
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    // Hold a jog at 600 mm/min: each increment is 600/60 * 0.12 = 1.200 mm.
    app.jog_start(Axis::X, Dir::Pos, 600.0);
    let streamed = collect_written(&mut app, &mut controller, 80); // ~400 ms of pumping.
    let text = String::from_utf8_lossy(&streamed);
    let increments = text.matches("$J=G91 G21 X1.200 F600").count();
    assert!(increments >= 2, "a held jog must stream repeated short increments; saw {increments} in {text:?}");

    // Release: the stream clears and a jog-cancel byte is injected.
    app.jog_stop();
    assert!(app.jog_stream.is_none(), "releasing must end the stream");
    // Let the cancel (and any last in-flight increment) flush, and confirm the cancel byte was written.
    let settling = collect_written(&mut app, &mut controller, 40);
    assert!(settling.contains(&0x85), "release must inject the jog-cancel byte (0x85)");

    // After settling, nothing more should stream: drain clean, then further pumps must emit no new jog lines.
    controller.drain_written();
    let after = collect_written(&mut app, &mut controller, 60); // ~300 ms.
    let after_text = String::from_utf8_lossy(&after);
    assert!(!after_text.contains("$J="), "no jog increments may stream after release; saw {after_text:?}");
  }

  /// Drive the probe-z pump across a short window, draining engine events (so an injected `[PRB:]` reaches the
  /// latch) and the transport writes each step. Returns everything the engine wrote, so a test can assert whether
  /// the deferred `G10 L2` zeroing line was emitted.
  fn pump_probe_steps(app: &mut SkirnirApp, controller: &mut LoopbackController, steps: usize) -> Vec<u8> {
    let mut out = Vec::new();
    for _ in 0..steps {
      app.pump_events();
      app.pump_probe_z();
      out.extend(controller.drain_written());
      std::thread::sleep(Duration::from_millis(5));
    }
    out
  }

  /// A hardened Z touch-off must DEFER the `G10 L2` zeroing: it sends the relative `G38.2` probe, then emits the
  /// zeroing line only after a successful `[PRB:]` result lands. Regression: the old `probe_z` fired both lines
  /// back-to-back with no success check, relying on alarm-ordering — a race — to protect a failed probe.
  #[test]
  fn a_successful_probe_emits_the_zeroing_line_only_after_the_result_lands() {
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    // Issue the probe. The RELATIVE probe (`G91 G38.2 Z-…` then `G90`) goes out; the zeroing `G10` must NOT yet
    // (it is deferred until a successful result lands).
    app.probe_z(2.5, 50.0, 1.0);
    assert!(app.view.probe_is_awaiting(), "the latch must be awaiting the probe result");
    let issued = String::from_utf8_lossy(&pump_probe_steps(&mut app, &mut controller, 10)).into_owned();
    assert!(issued.contains("G38.2 Z-2.500 F50"), "the probe line must be sent immediately; saw {issued:?}");
    assert!(issued.contains("G91"), "the probe must be wrapped incremental (G91); saw {issued:?}");
    assert!(!issued.contains("G10"), "the zeroing line must be deferred, not sent up front; saw {issued:?}");

    // The firmware acks the probe lines, then pushes a successful `[PRB:]` with contact machine-Z = -2.500. The
    // deferred zero is computed from the CONTACT (position-independent): work-Z reads the 1.0 mm plate AT the
    // contact, so the WCS Z origin is -2.500 − 1.000 = -3.500, emitted as `G10 L2` (not the position-dependent L20).
    assert!(controller.inject_line("ok"));
    assert!(controller.inject_line("[PRB:0.000,0.000,-2.500:1]"));
    let after = String::from_utf8_lossy(&pump_probe_steps(&mut app, &mut controller, 40)).into_owned();
    assert!(after.contains("G10 L2 P0 Z-3.500"), "a successful probe must emit the contact-based zero; saw {after:?}");
    assert!(!after.contains("G10 L20"), "the zero must be position-independent L2, not L20; saw {after:?}");
    assert!(app.pending_zero_z.is_none(), "the pending touch-off resolves once zeroed");
  }

  /// The `$#` lost-push fallback must be gated on probe COMPLETION, not raw elapsed time. A slow / no-contact
  /// probe legitimately travels longer than `PUSH_TIMEOUT`; while the machine still reports a cycle (`Run`),
  /// querying `$#` would return the PREVIOUS probe's stale result, which the latch could then zero off. So even
  /// past the timeout, an in-flight probe must NOT poll — only once the machine has been in a cycle and returned
  /// to `Idle` (a genuinely lost push) does the fallback fire. We backdate `issued_at` past the timeout to drive
  /// this deterministically without a 15 s wait.
  #[test]
  fn the_dollar_hash_fallback_is_gated_on_probe_completion_not_elapsed_time() {
    use crate::app::probe_flow::PUSH_TIMEOUT;
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    app.probe_z(2.5, 50.0, 1.0);
    // Backdate the issue so the push timeout is already exceeded; no result has arrived (the probe is "slow").
    if let Some(p) = app.pending_zero_z.as_mut() {
      p.issued_at = Instant::now() - (PUSH_TIMEOUT + Duration::from_secs(5));
    }
    controller.drain_written();

    // The machine reports it is mid-probe (`Run`). Despite the elapsed timeout, NO `$#` may be sent — polling
    // mid-cycle would read the previous probe's stale result.
    assert!(controller.inject_line("<Run|MPos:0.000,0.000,-1.000>"));
    let during = String::from_utf8_lossy(&pump_probe_steps(&mut app, &mut controller, 20)).into_owned();
    assert!(!during.contains("$#"), "an in-flight (Run) probe past the timeout must NOT poll $#; saw {during:?}");
    assert!(app.pending_zero_z.is_some(), "the touch-off stays pending while the probe is still travelling");

    // The machine finishes the move and returns to Idle with NO `[PRB:]` push (a genuinely lost push). NOW the
    // fallback is allowed to fire: `$#` is queried to retrieve the last probe result.
    assert!(controller.inject_line("<Idle|MPos:0.000,0.000,-2.500>"));
    let after = String::from_utf8_lossy(&pump_probe_steps(&mut app, &mut controller, 20)).into_owned();
    assert!(after.contains("$#"), "a finished (Idle, post-cycle) probe with a lost push must poll $#; saw {after:?}");
  }

  /// A failed probe (a no-contact `:0` flag, here from the silent `G38.3` path) must NOT emit the zeroing line —
  /// the success flag is the guard, and a non-contact result must leave work-Z untouched.
  #[test]
  fn a_failed_probe_never_emits_the_zeroing_line() {
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    app.probe_z(2.5, 50.0, 1.0);
    controller.drain_written();

    // The probe acks, then reports no contact (`:0`). The zeroing line must never be sent.
    assert!(controller.inject_line("ok"));
    assert!(controller.inject_line("[PRB:0.000,0.000,0.000:0]"));
    let after = String::from_utf8_lossy(&pump_probe_steps(&mut app, &mut controller, 40)).into_owned();
    assert!(!after.contains("G10"), "a failed probe must NOT zero work-Z; saw {after:?}");
    assert!(app.pending_zero_z.is_none(), "the failed touch-off resolves without zeroing");
    // The failure outcome is latched for the UI to render.
    assert!(app.view.probe_op.as_ref().and_then(|op| op.last.as_ref()).is_some_and(|o| !o.is_success()));
  }

  /// An alarming probe failure (`ALARM:5`, no contact within travel) must fail the op off the alarm and never
  /// emit the zeroing line — even though no `[PRB:]` push arrives. This is the case the old code only "protected"
  /// by the `error:9` g-code lock racing the unconditional `G10`.
  #[test]
  fn an_alarming_probe_fails_the_op_without_zeroing() {
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    app.probe_z(2.5, 50.0, 1.0);
    controller.drain_written();

    // No `[PRB:]` push — a no-contact `G38.2` raises `ALARM:5` instead. The latch fails off the alarm.
    assert!(controller.inject_line("ALARM:5"));
    let after = String::from_utf8_lossy(&pump_probe_steps(&mut app, &mut controller, 40)).into_owned();
    assert!(!after.contains("G10"), "an alarming probe must NOT zero work-Z; saw {after:?}");
    assert!(app.pending_zero_z.is_none());
    assert_eq!(app.view.banner, Some(crate::app::view_state::Banner::Alarm(5)));
  }

  /// Drive the wizard pump across a short window, draining engine events (so an injected `[PRB:]` reaches the
  /// latch and is folded into the wizard) and the transport writes each step. Returns everything written.
  fn pump_wizard_steps(app: &mut SkirnirApp, controller: &mut LoopbackController, steps: usize) -> Vec<u8> {
    let mut out = Vec::new();
    for _ in 0..steps {
      app.pump_events();
      app.pump_wizard();
      out.extend(controller.drain_written());
      std::thread::sleep(Duration::from_millis(5));
    }
    out
  }

  /// Issue one wizard touch and feed back its `[PRB:]` result, returning the emitted probe lines. Mirrors the
  /// real flow: the operator triggers a probe (`rotary_center_probe`), the firmware acks each line and pushes a
  /// `[PRB:]`, and the pump folds the result into the wizard.
  fn wizard_touch(app: &mut SkirnirApp, controller: &mut LoopbackController, prb: &str) -> String {
    app.rotary_center_probe();
    let issued = String::from_utf8_lossy(&pump_wizard_steps(app, controller, 8)).into_owned();
    // Ack the probe line and push the result; the pump folds it into the wizard.
    assert!(controller.inject_line("ok"));
    assert!(controller.inject_line(prb));
    pump_wizard_steps(app, controller, 12);
    issued
  }

  /// The rotary center-finder must drive a full multi-step run over the loopback transport: each touch composes
  /// the Phase 0 latch, the readings compute `(Y_c, Z_c)` with the symmetric formulas, and the offered WCS write
  /// carries only Y/Z (never A) as a `G10 L2`. This is the headline Phase 1 integration.
  #[test]
  fn the_rotary_center_finder_runs_a_full_three_touch_sequence_and_writes_yz_only() {
    use crate::app::rotary_center::WizardStep;
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    // Start a run: 6 mm dowel at A0.
    app.rotary_center_start(6.0, 0.0, crate::app::rotary_probe::RotaryProbeParams::default());
    controller.drain_written();

    // Touch 1 — left Y face. The emitted sequence must be the rotary-safe primitive (retract, index, settle,
    // linear probe) and the probe line must carry NO A word.
    let left = wizard_touch(&mut app, &mut controller, "[PRB:0.000,-3.000,0.000:1]");
    assert!(left.contains("G53 G0 Z"), "the touch must retract first; saw {left:?}");
    assert!(left.contains("G0 A0.000"), "the touch must index the rotary; saw {left:?}");
    assert!(left.contains("G4 P"), "the touch must settle before probing; saw {left:?}");
    let probe_line = left.lines().find(|l| l.contains("G38.2")).expect("a probe line");
    assert!(!probe_line.contains('A'), "the probe line must never carry an A word; saw {probe_line:?}");

    // Touch 2 — right Y face. After it, the wizard is at MoveToYc with Y_c = (-3 + 5)/2 = 1.0.
    wizard_touch(&mut app, &mut controller, "[PRB:0.000,5.000,0.000:1]");
    assert_eq!(app.wizard.as_ref().unwrap().state.step, WizardStep::MoveToYc);
    assert_eq!(app.wizard.as_ref().unwrap().state.y_center(), Some(1.0));

    // The mandatory move-to-Yc before the top probe.
    app.rotary_center_move_to_yc();
    let moved = String::from_utf8_lossy(&pump_wizard_steps(&mut app, &mut controller, 8)).into_owned();
    assert!(moved.contains("G53 G0 Y1.000"), "the wizard must move to the computed Y center; saw {moved:?}");

    // Touch 3 — dowel top. Z_c = Z_top - D/2 = -10 - 3 = -13.
    wizard_touch(&mut app, &mut controller, "[PRB:0.000,0.000,-10.000:1]");
    let state = &app.wizard.as_ref().unwrap().state;
    assert_eq!(state.step, WizardStep::Review);
    assert_eq!(state.z_center(), Some(-13.0));

    // Write the center to the active WCS: the line must be G10 L2 carrying only Y and Z. The default datum is the
    // axis centerline, so Z = Z_top − D/2 = -10 - 3 = -13.
    app.rotary_center_write_wcs();
    let wrote = String::from_utf8_lossy(&pump_wizard_steps(&mut app, &mut controller, 8)).into_owned();
    assert!(wrote.contains("G10 L2 P0 Y1.000 Z-13.000"), "the WCS write must be G10 L2 Y/Z; saw {wrote:?}");
    let g10 = wrote.lines().find(|l| l.contains("G10")).expect("a G10 line");
    assert!(!g10.contains('A'), "the WCS write must never carry an A word; saw {g10:?}");
  }

  /// The operator-tuned bench params must flow all the way into the emitted g-code — not just live in the UI.
  /// Starting a run with a custom `side_probe_z`/`clearance` and then probing a SIDE (Y) touch must emit those
  /// exact machine-Z moves (the retract clearance and the side-probe descent the §1.1 fix introduced).
  #[test]
  fn the_operator_tuned_bench_params_reach_the_emitted_probe_lines() {
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    // A non-default bench setup: a deeper clearance and a side-probe height the operator dialed in for the bench.
    let params = crate::app::rotary_probe::RotaryProbeParams {
      clearance_mm: -4.0,
      settle_secs: 0.5,
      feed: 60.0,
      depth_mm: 12.0,
      side_probe_z: -7.5,
    };
    app.rotary_center_start(6.0, 0.0, params);
    controller.drain_written();

    // The first (left Y) touch is a SIDE touch: it must retract to the custom clearance, then descend to the
    // custom side-probe Z before the lateral G38.2 — proving the tuned params were not silently replaced by the
    // old hard-coded defaults.
    let left = wizard_touch(&mut app, &mut controller, "[PRB:0.000,-3.000,0.000:1]");
    assert!(left.contains("G53 G0 Z-4.000"), "the retract must use the tuned clearance; saw {left:?}");
    assert!(left.contains("G53 G0 Z-7.500"), "the side touch must descend to the tuned side-probe Z; saw {left:?}");
    assert!(left.contains("G38.2 Y") && left.contains("F60"), "the probe must use the tuned feed; saw {left:?}");
  }

  /// Writing a found rotary center must PERSIST it (DOC-11 §1.3): the wizard's `(Y_c, Z_c, D, A-datum, Z-datum)`
  /// is saved to the profile file, and a fresh app loading that file restores the center so it can be re-applied
  /// next session without re-probing. This is the end-to-end persistence wiring over a temp profile file.
  #[test]
  fn writing_the_rotary_center_persists_it_and_a_fresh_app_restores_it() {
    use crate::app::rotary_center::{WizardStep, ZDatum};
    let dir = std::env::temp_dir().join(format!("skirnir-shell-persist-{}", std::process::id()));
    let path = dir.join("profile.ron");
    let _ = std::fs::remove_dir_all(&dir);

    let (mut app, mut controller) = app_with_engine();
    // Redirect persistence at the test seam so we round-trip through a temp file, never the operator's real
    // config dir, and start from a clean default profile regardless of what is on this machine.
    app.profile = crate::profile::Profile::default();
    app.profile_path_override = Some(path.clone());
    flush_handshake(&mut app, &mut controller);

    // Run the full three-touch center-finder: Y_c = (-3+5)/2 = 1.0, Z_c = -10 - 6/2 = -13.0.
    app.rotary_center_start(6.0, 0.0, crate::app::rotary_probe::RotaryProbeParams::default());
    controller.drain_written();
    wizard_touch(&mut app, &mut controller, "[PRB:0.000,-3.000,0.000:1]");
    wizard_touch(&mut app, &mut controller, "[PRB:0.000,5.000,0.000:1]");
    app.rotary_center_move_to_yc();
    pump_wizard_steps(&mut app, &mut controller, 8);
    wizard_touch(&mut app, &mut controller, "[PRB:0.000,0.000,-10.000:1]");
    assert_eq!(app.wizard.as_ref().unwrap().state.step, WizardStep::Review);

    // Writing the WCS must both emit the G10 AND persist the center to the temp file.
    app.rotary_center_write_wcs();
    pump_wizard_steps(&mut app, &mut controller, 8);
    assert!(path.exists(), "writing the center must persist a profile file at the override path");
    assert_eq!(
      app.profile.rotary,
      Some(crate::profile::RotarySetup {
        y_center: 1.0,
        z_center: -13.0,
        dowel_diameter: 6.0,
        a_datum_deg: 0.0,
        z_datum: ZDatum::AxisCenterline,
      }),
      "the in-memory profile must record the found center",
    );

    // A fresh load of that file (as a new launch would) must restore the same center.
    let (reloaded, notice) = crate::profile::load_from(&path);
    assert_eq!(notice, None, "a clean reload surfaces no notice");
    let restored = reloaded.rotary.expect("the reloaded profile carries the saved center");
    assert_eq!(restored.y_center, 1.0);
    assert_eq!(restored.z_center, -13.0);
    // And the restored center re-applies as the same G10 L2 line the wizard wrote — no re-probing needed.
    assert_eq!(restored.offer_g10(), "G10 L2 P0 Y1.000 Z-13.000");

    let _ = std::fs::remove_dir_all(&dir);
  }

  /// Re-applying a saved center (the `ApplySavedRotaryCenter` intent) must emit the persisted `G10 L2` line
  /// (Y/Z only, never A) without running the center-finder — the DOC-11 §1.3 "restore without re-probe" payoff.
  #[test]
  fn applying_a_saved_rotary_center_emits_the_g10_without_re_probing() {
    use crate::app::rotary_center::ZDatum;
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);
    // Seed a saved center directly (as a load from a previous session would), with no wizard running.
    app.profile.rotary = Some(crate::profile::RotarySetup {
      y_center: 1.0,
      z_center: -13.0,
      dowel_diameter: 6.0,
      a_datum_deg: 0.0,
      z_datum: ZDatum::AxisCenterline,
    });
    assert!(app.wizard.is_none(), "no center-finder run is needed to re-apply a saved center");
    controller.drain_written();

    app.handle_intent(Intent::ApplySavedRotaryCenter);
    let wrote = String::from_utf8_lossy(&pump_wizard_steps(&mut app, &mut controller, 8)).into_owned();
    assert!(wrote.contains("G10 L2 P0 Y1.000 Z-13.000"), "re-apply must emit the saved G10 L2; saw {wrote:?}");
    let g10 = wrote.lines().find(|l| l.contains("G10")).expect("a G10 line");
    assert!(!g10.contains('A'), "the re-apply must never carry an A word; saw {g10:?}");
  }

  /// Re-applying a saved center with no engine must NOT announce success: `send_line` fails (and already notices
  /// why), so a "re-applied" notice would contradict the "not connected" one. Mirrors the guarded `move_to_yc`.
  #[test]
  fn applying_a_saved_rotary_center_while_disconnected_does_not_claim_success() {
    use crate::app::rotary_center::ZDatum;
    let (mut app, _controller) = app_with_engine();
    // Drop the engine to model a disconnect / mid-session engine loss.
    app.engine = None;
    app.profile.rotary = Some(crate::profile::RotarySetup {
      y_center: 1.0,
      z_center: -13.0,
      dowel_diameter: 6.0,
      a_datum_deg: 0.0,
      z_datum: ZDatum::AxisCenterline,
    });

    app.handle_intent(Intent::ApplySavedRotaryCenter);
    let said_success = app.view.console.iter().any(|l| l.text.contains("re-applied the saved rotary center"));
    assert!(!said_success, "a failed send must not claim the center was re-applied");
    let said_failure = app.view.console.iter().any(|l| l.text.contains("not connected"));
    assert!(said_failure, "the underlying send failure must still be surfaced");
  }

  /// Selecting the top-surface Z datum must change the emitted `G10` Z word end-to-end: work-Z0 lands on the
  /// raw probed top (`Z_top`) instead of the axis centerline, while Y stays the axis and no A word appears.
  #[test]
  fn selecting_the_top_surface_datum_writes_z_top_to_the_wcs() {
    use crate::app::rotary_center::{WizardStep, ZDatum};
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    app.rotary_center_start(6.0, 0.0, crate::app::rotary_probe::RotaryProbeParams::default());
    controller.drain_written();
    wizard_touch(&mut app, &mut controller, "[PRB:0.000,-3.000,0.000:1]");
    wizard_touch(&mut app, &mut controller, "[PRB:0.000,5.000,0.000:1]");
    app.rotary_center_move_to_yc();
    pump_wizard_steps(&mut app, &mut controller, 8);
    wizard_touch(&mut app, &mut controller, "[PRB:0.000,0.000,-10.000:1]");
    assert_eq!(app.wizard.as_ref().unwrap().state.step, WizardStep::Review);

    // Operator switches the Z datum to the probed top surface, then writes.
    app.handle_intent(Intent::RotaryCenterSetZDatum(ZDatum::TopSurface));
    controller.drain_written();
    app.rotary_center_write_wcs();
    let wrote = String::from_utf8_lossy(&pump_wizard_steps(&mut app, &mut controller, 8)).into_owned();
    // Y is still the axis center (1.0); Z is now the raw top (-10.0), not the axis (-13.0).
    assert!(wrote.contains("G10 L2 P0 Y1.000 Z-10.000"), "the top-surface datum must write Z_top; saw {wrote:?}");
    let g10 = wrote.lines().find(|l| l.contains("G10")).expect("a G10 line");
    assert!(!g10.contains('A'), "the WCS write must never carry an A word; saw {g10:?}");
  }

  /// A failed touch mid-run must abort the wizard with no partial compute — and crucially never reach the WCS
  /// write. Here the second Y touch reports no contact (`:0`).
  #[test]
  fn a_failed_touch_aborts_the_rotary_wizard_without_writing() {
    use crate::app::rotary_center::WizardStep;
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    app.rotary_center_start(6.0, 0.0, crate::app::rotary_probe::RotaryProbeParams::default());
    controller.drain_written();
    wizard_touch(&mut app, &mut controller, "[PRB:0.000,-3.000,0.000:1]");
    // The right touch fails (no contact, `:0`): the wizard must abort.
    wizard_touch(&mut app, &mut controller, "[PRB:0.000,0.000,0.000:0]");
    assert_eq!(app.wizard.as_ref().unwrap().state.step, WizardStep::Aborted);

    // A write attempt on an aborted wizard must emit no G10.
    controller.drain_written();
    app.rotary_center_write_wcs();
    let after = String::from_utf8_lossy(&pump_wizard_steps(&mut app, &mut controller, 8)).into_owned();
    assert!(!after.contains("G10"), "an aborted wizard must never write a WCS offset; saw {after:?}");
  }

  /// Drive the datum pump across a short window, draining engine events (so an injected `[PRB:]` reaches the latch
  /// and is folded into the wizard) and the transport writes each step. Returns everything written.
  fn pump_datum_steps(app: &mut SkirnirApp, controller: &mut LoopbackController, steps: usize) -> Vec<u8> {
    let mut out = Vec::new();
    for _ in 0..steps {
      app.pump_events();
      app.pump_datum();
      out.extend(controller.drain_written());
      std::thread::sleep(Duration::from_millis(5));
    }
    out
  }

  /// Issue one datum touch and feed back its two-stage `[PRB:]` results, returning the emitted probe lines. Mirrors
  /// the real flow: the operator triggers the touch (`datum_probe_next`), the firmware runs the FAST search then
  /// the SLOW re-probe (each pushing a `[PRB:]`), and the pump folds the KEPT (slow) reading into the wizard. The
  /// fast reading is a DISTINCT junk value so a caller's assertion would fail if the latch wrongly kept it.
  fn datum_touch(app: &mut SkirnirApp, controller: &mut LoopbackController, prb: &str) -> String {
    app.datum_probe_next();
    let issued = String::from_utf8_lossy(&pump_datum_steps(app, controller, 8)).into_owned();
    // Fast pass: a junk reading that MUST be discarded (the two-stage keeps the slow re-probe, not this).
    assert!(controller.inject_line("ok"));
    assert!(controller.inject_line("[PRB:99.000,99.000,99.000:1]"));
    pump_datum_steps(app, controller, 6);
    // Slow pass: the KEPT reading (or a `:0` miss, which aborts).
    assert!(controller.inject_line("ok"));
    assert!(controller.inject_line(prb));
    pump_datum_steps(app, controller, 12);
    issued
  }

  /// The datum finder must drive a full outside-corner run over the loopback: each face composes the Phase 0
  /// latch as a two-stage `G38.3` touch, the readings are tip-comped, and the WCS write is a `G10 L2 P0 X.. Y..`.
  /// This is the headline datum integration test (the non-rotary sibling of the rotary center-finder run).
  #[test]
  fn the_datum_finder_runs_a_corner_and_writes_comped_xy() {
    use crate::app::datum::{Corner, DatumStep, ProbeParams};
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    // Outside corner A (+X,+Y), 2 mm tip (radius 1): X contact 5 → 6, Y contact 8 → 9.
    let params = ProbeParams { probe_diameter: 2.0, ..ProbeParams::default() };
    app.datum_corner_start(Corner::A, params);
    controller.drain_written();

    // Face 1 — X. The emitted sequence must be the two-stage latch (fast G38.3, retract, slow G38.3), never carry
    // an A word, and never use the alarming G38.2.
    let face_x = datum_touch(&mut app, &mut controller, "[PRB:5.000,0.000,0.000:1]");
    assert!(face_x.contains("G91 G38.3 X"), "the X face must probe X with a no-alarm G38.3; saw {face_x:?}");
    assert!(!face_x.contains("G38.2"), "a datum touch must never use the alarming G38.2; saw {face_x:?}");
    let probe_lines: Vec<&str> = face_x.lines().filter(|l| l.contains("G38")).collect();
    assert_eq!(probe_lines.len(), 2, "a touch must emit exactly the fast + slow passes; saw {face_x:?}");
    assert!(!face_x.contains('A'), "no datum line may carry an A word; saw {face_x:?}");
    assert_eq!(app.datum.as_ref().unwrap().state.step, DatumStep::ReadyFaceY);

    // Face 2 — Y. After it the wizard reaches Review with the comped corner.
    datum_touch(&mut app, &mut controller, "[PRB:0.000,8.000,0.000:1]");
    let state = &app.datum.as_ref().unwrap().state;
    assert_eq!(state.step, DatumStep::Review);
    assert_eq!(state.corner_xy(), Some((6.0, 9.0)));

    // Write the datum: a G10 L2 carrying the comped X and Y.
    app.datum_write_wcs();
    let wrote = String::from_utf8_lossy(&pump_datum_steps(&mut app, &mut controller, 8)).into_owned();
    assert!(wrote.contains("G10 L2 P0 X6.000 Y9.000"), "the datum write must be G10 L2 X/Y; saw {wrote:?}");
    let g10 = wrote.lines().find(|l| l.contains("G10")).expect("a G10 line");
    assert!(!g10.contains('A'), "the datum write must never carry an A word; saw {g10:?}");
  }

  /// A single-edge touch-off writes exactly one tip-comped axis. Approaching +X with a 4 mm tip and a contact at
  /// machine-X 10 → edge at 12, written as a one-axis `G10 L2 P0 X12.000`.
  #[test]
  fn a_datum_single_edge_touch_off_writes_one_comped_axis() {
    use crate::app::datum::ProbeParams;
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    let params = ProbeParams { probe_diameter: 4.0, ..ProbeParams::default() };
    app.datum_edge_start(Axis::X, Dir::Pos, params);
    controller.drain_written();
    datum_touch(&mut app, &mut controller, "[PRB:10.000,0.000,0.000:1]");

    app.datum_write_wcs();
    let wrote = String::from_utf8_lossy(&pump_datum_steps(&mut app, &mut controller, 8)).into_owned();
    assert!(wrote.contains("G10 L2 P0 X12.000"), "a single edge must write one comped axis; saw {wrote:?}");
  }

  /// A datum touch that reports no contact (`G38.3`'s software miss, flag `:0`) must abort the run and never
  /// reach the WCS write — the fail-closed contract that keeps a bad reading from writing a bogus offset.
  #[test]
  fn a_missed_datum_touch_aborts_without_writing() {
    use crate::app::datum::{DatumStep, ProbeParams};
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    app.datum_edge_start(Axis::Y, Dir::Neg, ProbeParams::default());
    controller.drain_written();
    // The touch misses: `[PRB:…:0]` (no alarm, thanks to G38.3). The wizard must abort.
    datum_touch(&mut app, &mut controller, "[PRB:0.000,0.000,0.000:0]");
    assert_eq!(app.datum.as_ref().unwrap().state.step, DatumStep::Aborted);

    // A write attempt on an aborted run must emit no G10.
    controller.drain_written();
    app.datum_write_wcs();
    let after = String::from_utf8_lossy(&pump_datum_steps(&mut app, &mut controller, 8)).into_owned();
    assert!(!after.contains("G10"), "an aborted datum run must never write a WCS offset; saw {after:?}");
  }

  /// The VerifyProbe guard: starting a datum run while the probe is already asserted (`Pn:P`) must be refused, so
  /// the operator never probes against a shorted / wrong-polarity input. No run is created and a notice explains.
  #[test]
  fn a_datum_start_is_refused_when_the_probe_is_already_asserted() {
    use crate::app::datum::ProbeParams;
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    // The firmware reports the probe input asserted (`Pn:P`); pump it into the view so the guard sees it.
    assert!(controller.inject_line("<Idle|MPos:0.000,0.000,0.000|Pn:P>"));
    assert!(pump_until(&mut app, |a| a.view.pins.probe), "the Pn:P status must reach the view");

    app.datum_edge_start(Axis::X, Dir::Pos, ProbeParams::default());
    assert!(app.datum.is_none(), "a datum run must not start while the probe is asserted");
    assert!(
      app.view.console.iter().any(|l| l.text.contains("already asserted")),
      "the refusal must be explained to the operator",
    );
  }

  /// Drive the mesh pump across a short window, draining engine events + transport writes each step.
  fn pump_mesh_steps(app: &mut SkirnirApp, controller: &mut LoopbackController, steps: usize) -> Vec<u8> {
    let mut out = Vec::new();
    for _ in 0..steps {
      app.pump_events();
      app.pump_mesh();
      out.extend(controller.drain_written());
      std::thread::sleep(Duration::from_millis(5));
    }
    out
  }

  /// Probe one grid point and feed back its two-stage `[PRB:]` Z results, returning the emitted lines. Like
  /// [`datum_touch`], the grid probe is a two-stage latch: a junk FAST reading (discarded) then the SLOW re-probe's
  /// KEPT reading (`prb`, or a `:0` miss which aborts).
  fn mesh_point(app: &mut SkirnirApp, controller: &mut LoopbackController, prb: &str) -> String {
    app.mesh_probe_next();
    let issued = String::from_utf8_lossy(&pump_mesh_steps(app, controller, 8)).into_owned();
    assert!(controller.inject_line("ok"));
    assert!(controller.inject_line("[PRB:99.000,99.000,99.000:1]"));
    pump_mesh_steps(app, controller, 6);
    assert!(controller.inject_line("ok"));
    assert!(controller.inject_line(prb));
    pump_mesh_steps(app, controller, 12);
    issued
  }

  /// The headline acquisition integration: a full serpentine grid probe over the loopback stores deltas from the
  /// first point and, on the last point, persists the completed mesh to the profile + invalidates the corrected
  /// cache. A 2×2 grid → 4 points; Z readings 5.0/5.2/4.9/5.1 → deltas 0/+0.2/−0.1/+0.1 in COLUMN-major
  /// serpentine order (0,0),(0,1),(1,1),(1,0).
  #[test]
  fn a_full_mesh_acquisition_probes_the_grid_and_persists_the_mesh() {
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);
    // Prime a stale corrected cache AND a stale simulation so we can prove completion invalidates both.
    app.autolevel_cache = Some(std::sync::Arc::from(vec!["stale".to_string()]));
    app.ui.set_program(vec!["G0 X0 Y0 Z-1".to_string(), "G1 X5 Y0 F100".to_string()], None);
    app.handle_intent(Intent::Simulate);
    assert!(app.simulated.is_some(), "a simulation is primed");

    app.mesh_probe_start(crate::app::autolevel::GridProbeParams::default(), (0.0, 0.0), (10.0, 10.0), (10.0, 10.0));
    controller.drain_written();
    assert_eq!(app.mesh_probe.as_ref().unwrap().state.progress(), (0, 4), "a 2×2 grid is four points");

    // Point 1 must emit the two-stage no-alarm Z touch (never G38.2) and a work-coord XY rapid.
    let first = mesh_point(&mut app, &mut controller, "[PRB:0.000,0.000,5.000:1]");
    assert!(first.contains("G90 G0 X") && first.contains("G53 G0 Z"), "a point positions in work XY + machine Z; saw {first:?}");
    let probes: Vec<&str> = first.lines().filter(|l| l.contains("G38")).collect();
    assert_eq!(probes.len(), 2, "a grid touch is two-stage; saw {first:?}");
    assert!(probes.iter().all(|p| p.contains("G38.3")), "a grid touch must be the no-alarm G38.3; saw {first:?}");

    mesh_point(&mut app, &mut controller, "[PRB:0.000,0.000,5.200:1]");
    mesh_point(&mut app, &mut controller, "[PRB:0.000,0.000,4.900:1]");
    mesh_point(&mut app, &mut controller, "[PRB:0.000,0.000,5.100:1]");

    // On the last point the run finished and the mesh was persisted to the profile with deltas-from-first-point.
    let mesh = app.profile.mesh.as_ref().expect("the completed mesh must be saved to the profile");
    assert!((mesh.z[mesh.index(0, 0)] - 0.0).abs() < 1e-9, "the first point is the reference (delta 0)");
    assert!((mesh.z[mesh.index(0, 1)] - 0.2).abs() < 1e-9);
    assert!((mesh.z[mesh.index(1, 1)] + 0.1).abs() < 1e-9);
    assert!((mesh.z[mesh.index(1, 0)] - 0.1).abs() < 1e-9);
    // Completion invalidated the stale corrected cache AND the stale simulation so the next stream/ETA rebuilds
    // against the new surface (the simulation's per-line timeline indexes the corrected program).
    assert!(app.autolevel_cache.is_none(), "a completed mesh must invalidate the corrected-program cache");
    assert!(app.simulated.is_none(), "a completed mesh must clear a simulation built against the old surface");
  }

  /// Clearing the saved mesh (MeshClear) invalidates the corrected cache AND a stale simulation (same reasoning as
  /// a completed acquisition — the correction inputs changed).
  #[test]
  fn clearing_the_mesh_invalidates_the_cache_and_the_simulation() {
    let mut app = app_disconnected();
    app.profile.mesh = Some(test_mesh(0.3));
    app.autolevel_cache = Some(std::sync::Arc::from(vec!["stale".to_string()]));
    app.ui.set_program(vec!["G1 X10 F100".to_string()], None);
    app.handle_intent(Intent::Simulate);
    assert!(app.simulated.is_some(), "a simulation is primed");

    app.handle_intent(Intent::MeshClear);
    assert!(app.profile.mesh.is_none(), "the mesh is cleared");
    assert!(app.autolevel_cache.is_none(), "clearing the mesh invalidates the corrected cache");
    assert!(app.simulated.is_none(), "clearing the mesh clears a simulation built against it");
  }

  /// A missed grid point (`G38.3` → `:0`) aborts acquisition fail-closed: no mesh is persisted, the run is Aborted.
  #[test]
  fn a_missed_grid_point_aborts_acquisition_without_persisting() {
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);
    app.mesh_probe_start(crate::app::autolevel::GridProbeParams::default(), (0.0, 0.0), (10.0, 10.0), (10.0, 10.0));
    controller.drain_written();

    mesh_point(&mut app, &mut controller, "[PRB:0.000,0.000,5.000:1]"); // point 1 ok.
    mesh_point(&mut app, &mut controller, "[PRB:0.000,0.000,0.000:0]"); // point 2 misses.
    assert_eq!(
      app.mesh_probe.as_ref().unwrap().state.step,
      crate::app::autolevel::MeshProbeStep::Aborted,
      "a missed point must abort the run",
    );
    assert!(app.profile.mesh.is_none(), "an aborted acquisition must not persist a partial mesh");
  }

  /// Starting acquisition while the probe is already asserted (`Pn:P`) is refused by the shared VerifyProbe guard.
  #[test]
  fn a_mesh_acquisition_is_refused_when_the_probe_is_asserted() {
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);
    assert!(controller.inject_line("<Idle|MPos:0.000,0.000,0.000|Pn:P>"));
    assert!(pump_until(&mut app, |a| a.view.pins.probe), "the Pn:P status must reach the view");
    app.mesh_probe_start(crate::app::autolevel::GridProbeParams::default(), (0.0, 0.0), (10.0, 10.0), (10.0, 10.0));
    assert!(app.mesh_probe.is_none(), "acquisition must not start while the probe is asserted");
  }

  /// Pump ALL probe follow-ups (ZeroZ, center-finder, and Phase 2 sweep) each step, draining events + writes — so
  /// a test can prove no flow acts on another's `[PRB:]` through the shared kind-routed latch.
  fn pump_both_steps(app: &mut SkirnirApp, controller: &mut LoopbackController, steps: usize) -> Vec<u8> {
    let mut out = Vec::new();
    for _ in 0..steps {
      app.pump_events();
      app.pump_probe_z();
      app.pump_wizard();
      app.pump_sweep();
      out.extend(controller.drain_written());
      std::thread::sleep(Duration::from_millis(5));
    }
    out
  }

  /// Drive the sweep pump across a short window, draining events + writes each step.
  fn pump_sweep_steps(app: &mut SkirnirApp, controller: &mut LoopbackController, steps: usize) -> Vec<u8> {
    let mut out = Vec::new();
    for _ in 0..steps {
      app.pump_events();
      app.pump_sweep();
      out.extend(controller.drain_written());
      std::thread::sleep(Duration::from_millis(5));
    }
    out
  }

  /// Issue one Phase 2 sweep touch and feed back its `[PRB:]` result, returning the emitted probe lines. Mirrors
  /// the real flow: trigger `sweep_probe`, the firmware acks each line and pushes a `[PRB:]`, the pump folds it.
  fn sweep_touch(app: &mut SkirnirApp, controller: &mut LoopbackController, prb: &str) -> String {
    app.sweep_probe();
    let issued = String::from_utf8_lossy(&pump_sweep_steps(app, controller, 8)).into_owned();
    assert!(controller.inject_line("ok"));
    assert!(controller.inject_line(prb));
    pump_sweep_steps(app, controller, 12);
    issued
  }

  /// The 180°-flip verify (DOC-11 §2.1) must run a two-touch sweep (θ, θ+180), compute `error=(r2−r1)/2`, and
  /// offer a position-independent `G10 L2` correction on the verified axis only (never A).
  #[test]
  fn the_flip_verify_runs_two_touches_and_offers_a_yz_only_correction() {
    use crate::app::intent::{Axis, Dir};
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    app.flip_verify_start(0.0, Axis::Y, Dir::Neg);
    controller.drain_written();

    // Touch 1 at A0 → reading r1 = 4.0 (Y component). The probe must be rotary-safe and carry no A in the probe.
    let t1 = sweep_touch(&mut app, &mut controller, "[PRB:0.000,4.000,0.000:1]");
    assert!(t1.contains("G90 G0 A0.000"), "the first touch indexes A0 absolutely; saw {t1:?}");
    let probe = t1.lines().find(|l| l.contains("G38.2")).expect("a probe line");
    assert!(!probe.contains('A'), "the probe line must never carry an A word; saw {probe:?}");

    // Touch 2 at A180 → reading r2 = -2.0. error = (-2 - 4)/2 = -3.0 (the residual eccentricity; radius-free).
    let t2 = sweep_touch(&mut app, &mut controller, "[PRB:0.000,-2.000,0.000:1]");
    assert!(t2.contains("G90 G0 A180.000"), "the second touch indexes A180; saw {t2:?}");
    assert!(app.sweep.as_ref().unwrap().sweep.is_done(), "both touches done");

    // The correction is a RELATIVE shift of the current work origin by the residual. With the work-Y origin
    // currently at machine-Y = 10.0 (WCO), the corrected origin is 10 + (-3) = 7.0 — Y-only, never A, and position-
    // independent (it does NOT write the surface midpoint 1.0, which would be a full radius off the axis).
    app.view.last_wco = std::vec![0.0, 10.0, 0.0, 0.0];
    app.flip_verify_write_correction();
    let wrote = String::from_utf8_lossy(&pump_sweep_steps(&mut app, &mut controller, 8)).into_owned();
    assert!(wrote.contains("G10 L2 P0 Y7.000"), "the correction must shift the origin by the residual; saw {wrote:?}");
    let g10 = wrote.lines().find(|l| l.contains("G10")).expect("a G10 line");
    assert!(!g10.contains('A'), "the correction must never carry an A word; saw {g10:?}");
  }

  /// The runout report (DOC-11 §2.2) must run an N-touch sweep, compute TIR = max−min and eccentricity = TIR/2,
  /// and write NO `G10` (read-only).
  #[test]
  fn the_runout_report_runs_n_touches_and_writes_nothing() {
    use crate::app::intent::{Axis, Dir};
    use crate::app::runout::RunoutReport;
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    app.runout_start(4, 0.0, Axis::Y, Dir::Neg);
    controller.drain_written();

    // Four readings spanning [-0.5, 1.5] → TIR = 2.0, eccentricity = 1.0.
    for prb in ["[PRB:0.000,0.000,0.000:1]", "[PRB:0.000,1.500,0.000:1]", "[PRB:0.000,-0.500,0.000:1]",
      "[PRB:0.000,1.000,0.000:1]"]
    {
      sweep_touch(&mut app, &mut controller, prb);
    }
    let run = app.sweep.as_ref().expect("a sweep");
    assert!(run.sweep.is_done(), "all four touches done");
    let report = RunoutReport::from_readings(run.sweep.readings()).expect("a report");
    assert_eq!(report.tir, 2.0);
    assert_eq!(report.eccentricity, 1.0);

    // Read-only: the whole run must have emitted NO G10.
    controller.drain_written();
    let after = String::from_utf8_lossy(&pump_sweep_steps(&mut app, &mut controller, 8)).into_owned();
    assert!(!after.contains("G10"), "runout is read-only and must never write a G10; saw {after:?}");
  }

  /// A failed touch mid-sweep aborts the whole run with no partial result (no TIR, no correction).
  #[test]
  fn a_failed_sweep_touch_aborts_with_no_partial_result() {
    use crate::app::angle_sweep::SweepStep;
    use crate::app::intent::{Axis, Dir};
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    app.runout_start(3, 0.0, Axis::Y, Dir::Neg);
    controller.drain_written();
    sweep_touch(&mut app, &mut controller, "[PRB:0.000,1.000,0.000:1]");
    // The second touch reports no contact (`:0`) → the sweep aborts.
    sweep_touch(&mut app, &mut controller, "[PRB:0.000,0.000,0.000:0]");
    assert_eq!(app.sweep.as_ref().unwrap().sweep.step(), SweepStep::Aborted);
  }

  /// Finding #14: a Phase 2 sweep (flip-verify and runout) must probe with the OPERATOR-TUNED bench params, not
  /// the placeholder `RotaryProbeParams::default()`. The decisive evidence is the side-probe descent height: a Y
  /// touch emits `G53 G0 Z<side_probe_z>`, and the operator's dialed-in value (here −7.5) must appear, never the
  /// −10.0 default. A garbage descent height would touch the wrong place and apply a bogus WCS correction.
  #[test]
  fn a_sweep_uses_the_operator_tuned_bench_params_not_the_default() {
    use crate::app::intent::{Axis, Dir};
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    // Dial in a distinctive side-probe Z the default never produces.
    app.ui.rotary_bench.side_probe_z = -7.5;

    // Flip-verify: the first Y touch's lateral descent must use the tuned −7.5, not the −10.0 placeholder.
    app.flip_verify_start(0.0, Axis::Y, Dir::Neg);
    let t1 = sweep_touch(&mut app, &mut controller, "[PRB:0.000,4.000,0.000:1]");
    assert!(t1.contains("G53 G0 Z-7.500"), "flip-verify must descend to the tuned side-probe Z; saw {t1:?}");
    assert!(!t1.contains("G53 G0 Z-10.000"), "flip-verify must NOT use the default side-probe Z; saw {t1:?}");

    // Runout: same requirement on its touches.
    app.runout_start(2, 0.0, Axis::Y, Dir::Neg);
    let r1 = sweep_touch(&mut app, &mut controller, "[PRB:0.000,1.000,0.000:1]");
    assert!(r1.contains("G53 G0 Z-7.500"), "runout must descend to the tuned side-probe Z; saw {r1:?}");
  }

  /// A lost wizard-touch push in a Phase 2 sweep must fall back to `$#` (the SHARED fallback), exactly like the
  /// ZeroZ and center-finder flows — a dropped `[PRB:]` must not hang the sweep.
  #[test]
  fn a_lost_sweep_touch_push_falls_back_to_dollar_hash() {
    use crate::app::intent::{Axis, Dir};
    use crate::app::probe_flow::PUSH_TIMEOUT;
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    app.runout_start(2, 0.0, Axis::Y, Dir::Neg);
    app.sweep_probe();
    // Backdate the touch past the push timeout; no `[PRB:]` arrives (lost).
    if let Some(run) = app.sweep.as_mut() && let Some(f) = run.touch_fallback.as_mut() {
      f.issued_at = Instant::now() - (PUSH_TIMEOUT + Duration::from_secs(5));
    }
    controller.drain_written();

    // In a cycle → no poll. A clean return to Idle (push genuinely lost) → poll `$#`.
    assert!(controller.inject_line("<Run|MPos:0.000,-1.000,0.000>"));
    let during = String::from_utf8_lossy(&pump_sweep_steps(&mut app, &mut controller, 12)).into_owned();
    assert!(!during.contains("$#"), "an in-flight sweep touch must not poll $# mid-cycle; saw {during:?}");
    assert!(controller.inject_line("<Idle|MPos:0.000,-3.000,0.000>"));
    let after = String::from_utf8_lossy(&pump_sweep_steps(&mut app, &mut controller, 12)).into_owned();
    assert!(after.contains("$#"), "a finished sweep touch with a lost push must poll $#; saw {after:?}");
  }

  /// Cross-contamination guard (extended to Phase 2): starting a flip-verify cancels a pending ZeroZ, and the
  /// sweep's `[PRB:]` must be consumed ONLY by the sweep — never firing the ZeroZ `G10`.
  #[test]
  fn a_sweep_touch_does_not_fire_a_pending_zeroz_or_wizard() {
    use crate::app::intent::{Axis, Dir};
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    // Arm a ZeroZ and a center-finder, then start a flip-verify and issue its touch — both others must cancel.
    app.probe_z(2.5, 50.0, 1.0);
    app.rotary_center_start(6.0, 0.0, crate::app::rotary_probe::RotaryProbeParams::default());
    app.flip_verify_start(0.0, Axis::Y, Dir::Neg);
    app.sweep_probe();
    assert!(app.pending_zero_z.is_none(), "starting a sweep must cancel a pending ZeroZ");
    assert!(app.wizard.is_none(), "starting a sweep must cancel a center-finder run");
    controller.drain_written();

    // The sweep result lands. It must feed the sweep and NOT emit any G10 (no ZeroZ zero, no wizard write).
    assert!(controller.inject_line("ok"));
    assert!(controller.inject_line("[PRB:0.000,4.000,0.000:1]"));
    let after = String::from_utf8_lossy(&pump_both_steps(&mut app, &mut controller, 20)).into_owned();
    assert!(!after.contains("G10"), "a sweep `[PRB:]` must not fire any other flow's G10; saw {after:?}");
    assert_eq!(app.sweep.as_ref().unwrap().sweep.readings(), &[4.0], "the sweep consumes its own result");
  }

  /// THE CROSS-CONTAMINATION REGRESSION (finding #2): starting a rotary touch while a ZeroZ touch-off is pending
  /// must cancel the ZeroZ follow-up, and the rotary `[PRB:]` that lands must be consumed ONLY by the wizard — it
  /// must never fire the deferred ZeroZ `G10` off an unrelated probe.
  #[test]
  fn a_rotary_touch_does_not_fire_a_pending_zeroz_zeroing() {
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    // Arm a ZeroZ touch-off (deferred). Then, before it resolves, start a rotary run and issue its first touch —
    // which re-arms the shared latch as a RotaryCenter op and must cancel the ZeroZ pending.
    app.probe_z(2.5, 50.0, 1.0);
    app.rotary_center_start(6.0, 0.0, crate::app::rotary_probe::RotaryProbeParams::default());
    app.rotary_center_probe();
    assert!(app.pending_zero_z.is_none(), "starting a rotary touch must cancel the pending ZeroZ follow-up");
    controller.drain_written();

    // The rotary touch's result lands. It must advance the WIZARD (ReadyYRight) and NOT emit the ZeroZ `G10`.
    assert!(controller.inject_line("ok"));
    assert!(controller.inject_line("[PRB:0.000,-3.000,0.000:1]"));
    let after = String::from_utf8_lossy(&pump_both_steps(&mut app, &mut controller, 20)).into_owned();
    assert!(!after.contains("G10"), "a rotary `[PRB:]` must NOT fire the ZeroZ zeroing; saw {after:?}");
    assert_eq!(app.wizard.as_ref().unwrap().state.y_left, Some(-3.0), "the wizard must consume its own result");
  }

  /// A safety-door suspend mid-probe must NOT be treated as "finished" — `Door` (like Sleep/Tool) is in-cycle, so
  /// the lost-push `$#` fallback must not arm while suspended (finding #6). Only a clean return to Idle finishes.
  #[test]
  fn a_door_suspend_mid_probe_does_not_arm_the_dollar_hash_fallback() {
    use crate::app::probe_flow::PUSH_TIMEOUT;
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    app.probe_z(2.5, 50.0, 1.0);
    if let Some(p) = app.pending_zero_z.as_mut() {
      p.issued_at = Instant::now() - (PUSH_TIMEOUT + Duration::from_secs(5));
    }
    // The machine was in a cycle, then a safety door opened → `Door` suspend. Despite the elapsed timeout, the
    // probe is NOT finished (Door is in-cycle), so no `$#` may be sent.
    assert!(controller.inject_line("<Run|MPos:0.000,0.000,-1.000>"));
    pump_both_steps(&mut app, &mut controller, 4);
    controller.drain_written();
    assert!(controller.inject_line("<Door:0|MPos:0.000,0.000,-1.000>"));
    let during = String::from_utf8_lossy(&pump_both_steps(&mut app, &mut controller, 20)).into_owned();
    assert!(!during.contains("$#"), "a Door suspend must not arm the lost-push fallback; saw {during:?}");
    assert!(app.pending_zero_z.is_some(), "the touch-off stays pending through the suspend");
  }

  /// The rotary wizard must have the SAME lost-push fallback as ZeroZ (finding #7): a dropped/suppressed `[PRB:]`
  /// push, once the touch has finished, falls back to a `$#` poll rather than leaving the wizard awaiting forever.
  #[test]
  fn a_lost_wizard_touch_push_falls_back_to_dollar_hash() {
    use crate::app::probe_flow::PUSH_TIMEOUT;
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    app.rotary_center_start(6.0, 0.0, crate::app::rotary_probe::RotaryProbeParams::default());
    app.rotary_center_probe();
    // Backdate the touch's issue past the push timeout; no `[PRB:]` push arrives (it was lost).
    if let Some(run) = app.wizard.as_mut() && let Some(f) = run.touch_fallback.as_mut() {
      f.issued_at = Instant::now() - (PUSH_TIMEOUT + Duration::from_secs(5));
    }
    controller.drain_written();

    // While still in a cycle, no poll (would read the previous probe's stale result).
    assert!(controller.inject_line("<Run|MPos:0.000,-1.000,0.000>"));
    let during = String::from_utf8_lossy(&pump_wizard_steps(&mut app, &mut controller, 12)).into_owned();
    assert!(!during.contains("$#"), "an in-flight wizard touch must not poll $# mid-cycle; saw {during:?}");

    // A clean return to Idle (the push genuinely lost): the wizard polls `$#`, exactly like the ZeroZ flow.
    assert!(controller.inject_line("<Idle|MPos:0.000,-3.000,0.000>"));
    let after = String::from_utf8_lossy(&pump_wizard_steps(&mut app, &mut controller, 12)).into_owned();
    assert!(after.contains("$#"), "a finished wizard touch with a lost push must poll $#; saw {after:?}");

    // The `$#` answer (a `[PRB:]` line) then resolves the touch normally.
    assert!(controller.inject_line("[PRB:0.000,-3.000,0.000:1]"));
    pump_wizard_steps(&mut app, &mut controller, 12);
    assert_eq!(app.wizard.as_ref().unwrap().state.y_left, Some(-3.0), "the $# answer resolves the wizard touch");
  }

  /// While the firmware reports its planner queue nearly full (`Bf:` blocks-free below the margin), a held jog
  /// must stop streaming new increments so it can never overrun the 32-block queue into a `QueueFull` rejection.
  #[test]
  fn a_held_jog_throttles_when_the_planner_queue_is_nearly_full() {
    let (mut app, mut controller) = app_with_engine();
    // Report a nearly-full planner queue: 2 blocks free, below the margin of `JOG_STREAM_MIN_BLOCKS_FREE`.
    assert!(controller.inject_line("<Run|MPos:0.000,0.000,0.000|Bf:2,1000>"));
    assert!(
      pump_until(&mut app, |a| a.view.status.as_ref().and_then(|s| s.buffer).map(|b| b.0) == Some(2)),
      "the engine never reported the Bf buffer state",
    );
    controller.drain_written();

    // Hold a jog: `collect_written` never ingests a fresh status, so the low blocks-free report stays in force and
    // must gate off every due increment.
    app.jog_start(Axis::X, Dir::Pos, 600.0);
    let streamed = collect_written(&mut app, &mut controller, 60); // ~300 ms.
    let text = String::from_utf8_lossy(&streamed);
    assert!(!text.contains("$J="), "a nearly-full planner queue must throttle the jog stream; saw {text:?}");
  }

  /// Saving the settings dialog must flush every staged edit as an ordered `$<n>=<value>` line through the
  /// streaming engine, then `$$` to re-confirm. The explicit Save is the only thing that reaches the firmware —
  /// proves an edit is no longer dropped on focus-loss (it is staged, then this writes it). The edits stay staged
  /// until the re-dump confirms them (Bug 6), so this test only asserts the write/order/re-dump traffic.
  #[test]
  fn saving_flushes_staged_settings_in_order_then_redumps() {
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    // Stage three edits out of numeric order; Save must emit them ascending.
    app.ui.settings_staging.stage(110, "800", Some("500"));
    app.ui.settings_staging.stage(0, "12", Some("10"));
    app.ui.settings_staging.stage(22, "1", Some("0"));

    app.handle_intent(Intent::SaveSettings);
    // The writes go through the engine asynchronously; pump events and accumulate the transport bytes until the
    // last staged write and the trailing `$$` re-dump have both landed (or the budget is exhausted).
    let mut written = Vec::new();
    for _ in 0..200 {
      app.pump_events();
      written.extend(controller.drain_written());
      let text = String::from_utf8_lossy(&written);
      if text.contains("$110=800") && text.contains("$$") {
        break;
      }
      std::thread::sleep(Duration::from_millis(5));
    }
    let text = String::from_utf8_lossy(&written).into_owned();
    // Each staged setting is written, the `$$` re-confirm follows, and the order is ascending `$<n>`.
    let pos0 = text.find("$0=12").expect("the $0 write must be sent");
    let pos22 = text.find("$22=1").expect("the $22 write must be sent");
    let pos110 = text.find("$110=800").expect("the $110 write must be sent");
    assert!(pos0 < pos22 && pos22 < pos110, "staged writes must be sent in ascending order; saw {text:?}");
    let pos_dump = text.rfind("$$").expect("a $$ re-confirm must follow the writes");
    assert!(pos110 < pos_dump, "the $$ re-confirm must come after the staged writes; saw {text:?}");
    // The edits remain staged and dirty until the re-dump confirms them — Save no longer drops them up front, so
    // a firmware rejection cannot vanish silently (Bug 6). Confirmation/clearing is exercised below.
    assert!(!app.ui.settings_staging.is_empty(), "Save arms confirmation; edits persist until the re-dump lands");
    assert!(app.ui.settings_staging.is_dirty(0), "a saved-but-unconfirmed edit stays dirty");
  }

  /// Bug 6 end to end: after Save, the firmware's `$$` re-dump confirms one write and refutes another (the
  /// rejected one comes back unchanged). The accepted setting must clear from staging while the rejected one
  /// stays staged/dirty (so the operator still sees their edit) and is announced in the console — never silently
  /// dropped. Driven over the loopback so the real reduce/confirm path is exercised.
  #[test]
  fn a_rejected_setting_survives_save_while_an_accepted_one_clears() {
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    // Stage two edits: $0 will be accepted (re-dump shows the new value), $110 will be rejected (re-dump reverts).
    app.ui.settings_staging.stage(0, "12", Some("10"));
    app.ui.settings_staging.stage(110, "999999", Some("500"));
    app.handle_intent(Intent::SaveSettings);

    // Let the writes flush, then the firmware answers: an `ok` per write, then the `$$` re-dump. $0 took the new
    // value; $110 was refused and reverted to its old value. (The acks keep flow control honest; the re-dump
    // values are the authoritative verdict the confirm path reads.)
    pump_until(&mut app, |_| false); // pump a few frames so the writes reach the wire.
    controller.drain_written();
    assert!(controller.inject_line("ok")); // $0=12
    assert!(controller.inject_line("error:3")); // $110 rejected
    assert!(controller.inject_line("ok")); // the $$ line itself
    assert!(controller.inject_line("$0=12")); // re-dump: accepted
    assert!(controller.inject_line("$110=500")); // re-dump: reverted (rejected)

    assert!(
      pump_until(&mut app, |a| !a.ui.settings_staging.is_dirty(0) && a.ui.settings_staging.is_dirty(110)),
      "after the re-dump, the accepted $0 must clear and the rejected $110 must stay dirty",
    );
    assert_eq!(
      app.ui.settings_staging.staged_value(110),
      Some("999999"),
      "the rejected edit's value must remain visible to the operator, not vanish",
    );
    // The rejection is announced in the console so the operator learns the write failed.
    let noted_rejection = app.view.console.iter().any(|l| l.text.contains("$110") && l.text.contains("rejected"));
    assert!(noted_rejection, "a rejected setting must produce a console notice naming the failed $N");
  }

  /// Saving with nothing staged must be a quiet no-op: no `$<n>=` write and no `$$` re-dump reaches the firmware
  /// (the Save button is disabled then, but the handler must stay safe if invoked regardless).
  #[test]
  fn saving_with_nothing_staged_sends_nothing() {
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);
    assert!(app.ui.settings_staging.is_empty(), "the test starts with no staged edits");

    app.handle_intent(Intent::SaveSettings);
    // Pump events so any (erroneous) write would have a chance to flush to the transport, then assert silence.
    let mut written = Vec::new();
    for _ in 0..20 {
      app.pump_events();
      written.extend(controller.drain_written());
      std::thread::sleep(Duration::from_millis(5));
    }
    let text = String::from_utf8_lossy(&written);
    assert!(!text.contains('$'), "an empty Save must send no settings traffic at all; saw {text:?}");
  }

  // ---- Simulate / physics-based ETA ----------------------------------------------------------------------------

  /// Build a disconnected app (no engine) for the host-only Simulate path, with the OS profile reset to a hermetic
  /// default and its saves redirected to a unique temp file (mirroring `app_with_engine`'s isolation). Simulate is
  /// a pure host calc that touches no engine, so it needs no transport — the point is to prove it works disconnected.
  fn app_disconnected() -> SkirnirApp {
    let runtime = tokio::runtime::Builder::new_multi_thread().enable_all().build().expect("runtime");
    let mut app = SkirnirApp::new(runtime, crate::config::Config::default(), Vec::new());
    app.profile = crate::profile::Profile::default();
    let stamp = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos()).unwrap_or(0);
    let temp = std::env::temp_dir().join(format!("skirnir-test-profile-{}-{stamp}", std::process::id()));
    app.profile_path_override = Some(temp.join("profile.ron"));
    app
  }

  /// Simulate on a loaded program (while disconnected, with no settings) yields a stored timeline with a positive
  /// total, and flags that it fell back to default machine settings.
  #[test]
  fn simulate_on_a_loaded_program_builds_a_timeline_and_flags_default_settings() {
    let mut app = app_disconnected();
    app.ui.set_program(vec!["G0 X50".to_string(), "G1 X100 F500".to_string()], None);
    assert!(app.simulated.is_none(), "no simulation exists until the operator runs one");

    app.handle_intent(Intent::Simulate);

    let timeline = app.simulated.as_ref().expect("Simulate must store a timeline");
    assert!(timeline.total_seconds > 0.0, "a program with real moves has a positive modeled time");
    assert_eq!(timeline.lines.len(), 2, "one LineEta per source line");
    assert!(
      app.simulated_default_settings,
      "with no `$$` snapshot loaded, the estimate used default settings and must flag it",
    );
  }

  /// Startup must announce WHICH config file the app reads, so an operator asking "which file do I edit?" sees the
  /// path in the console on launch. The notice is prefixed `config:`; on a platform with a config dir it carries the
  /// resolved path, otherwise it explains there is none. Either way a `config:` line must be present.
  #[test]
  fn startup_announces_the_config_file_path_in_the_console() {
    let app = app_disconnected();
    let announced = app.view.console.iter().any(|l| l.text.starts_with("config:"));
    assert!(announced, "startup must push a `config:` notice naming the file the app reads");
    // When a per-user config base exists (the usual case in CI/dev), the notice must carry the resolved path so it
    // is directly actionable; the path ends at the app's `config.json`.
    if let Some(path) = crate::config::config_path() {
      let said_path = app.view.console.iter().any(|l| l.text.contains(&path.display().to_string()));
      assert!(said_path, "the config notice must carry the actual resolved path when one exists");
    }
  }

  /// Simulate with no program loaded is a quiet no-op (a notice, no stored timeline) rather than building an empty
  /// estimate or panicking.
  #[test]
  fn simulate_with_no_program_is_a_noop() {
    let mut app = app_disconnected();
    assert!(app.ui.program.is_empty(), "the test starts with no program");
    app.handle_intent(Intent::Simulate);
    assert!(app.simulated.is_none(), "Simulate with no program must store nothing");
  }

  /// When a `$$` snapshot IS present in the view settings, Simulate uses it (no default-settings flag) and the
  /// configs are read from it — a faster max-rate produces a shorter modeled total than the firmware default.
  #[test]
  fn simulate_uses_loaded_settings_and_does_not_flag_defaults() {
    use crate::protocol::SettingValue;
    let mut app = app_disconnected();
    app.ui.set_program(vec!["G0 X100".to_string()], None);

    // Baseline against defaults.
    app.handle_intent(Intent::Simulate);
    let default_total = app.simulated.as_ref().expect("a timeline").total_seconds;
    assert!(app.simulated_default_settings, "the baseline used defaults");

    // Load a generous X max-rate (`$110`) and X accel (`$120`) so the rapid completes faster than the default.
    app.view.settings.apply_value(SettingValue { number: 110, value: "100000".to_string() });
    app.view.settings.apply_value(SettingValue { number: 120, value: "100000".to_string() });
    app.handle_intent(Intent::Simulate);
    let fast_total = app.simulated.as_ref().expect("a timeline").total_seconds;
    assert!(!app.simulated_default_settings, "with a `$$` snapshot present, the default-settings flag must clear");
    assert!(fast_total < default_total, "a faster max-rate must shorten the modeled total ({fast_total} < {default_total})");
  }

  /// Opening a new program clears a stale simulation so the dock ETA never reflects the previous file's lines.
  #[test]
  fn opening_a_new_program_clears_a_stale_simulation() {
    let mut app = app_disconnected();
    app.ui.set_program(vec!["G1 X10 F500".to_string()], None);
    app.handle_intent(Intent::Simulate);
    assert!(app.simulated.is_some(), "a simulation is stored");

    // Open a fresh program from a temp file: the open path must drop the prior simulation.
    let dir = std::env::temp_dir().join(format!("skirnir-sim-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("temp dir");
    let path = dir.join("fresh.gcode");
    std::fs::write(&path, "G1 X20 F500\n").expect("write program");
    app.open_program(&path);
    assert!(app.simulated.is_none(), "opening a new program must clear the stale simulation");
    let _ = std::fs::remove_file(&path);
  }

  /// A small flat mesh covering the test programs' XY, uniformly offset so a correction is observable. A uniform
  /// +0.5 mesh shifts every corrected cut Z by exactly 0.5.
  fn test_mesh(delta: f64) -> crate::app::autolevel::Mesh {
    let mut m = crate::app::autolevel::Mesh::from_spacing((0.0, 0.0), (100.0, 100.0), (10.0, 10.0));
    for iy in 0..m.ny {
      for ix in 0..m.nx {
        m.set_delta(ix, iy, delta);
      }
    }
    m
  }

  /// With autoleveling OFF, the stream resolver returns the source lines verbatim (byte-identical) — the opt-out
  /// path must never touch the program.
  #[test]
  fn resolve_stream_program_returns_the_source_verbatim_when_autolevel_is_off() {
    let mut app = app_disconnected();
    let src = vec!["G0 X0 Y0 Z-1".to_string(), "G1 X20 Y0 F100".to_string()];
    app.ui.set_program(src.clone(), None);
    assert!(!app.ui.autolevel_enabled, "autolevel starts off");
    let lines = app.resolve_stream_program().expect("off never errors");
    assert_eq!(&*lines, src.as_slice(), "with autolevel off the source must stream verbatim");
    assert!(app.autolevel_cache.is_none(), "the off path must not populate the corrected cache");
  }

  /// Arming autoleveling with NO mesh probed must refuse (Err) rather than silently sending the uncorrected file.
  #[test]
  fn resolve_stream_program_refuses_when_armed_without_a_mesh() {
    let mut app = app_disconnected();
    app.ui.set_program(vec!["G0 X0 Y0 Z-1".to_string()], None);
    app.ui.autolevel_enabled = true;
    app.profile.mesh = None;
    let result = app.resolve_stream_program();
    assert!(result.is_err(), "armed without a mesh must refuse, not stream the uncorrected file");
    assert!(result.unwrap_err().contains("no height map"), "the refusal reason must name the missing mesh");
  }

  /// Arming autoleveling with a probed mesh returns the height-CORRECTED program (canonical header + shifted Z),
  /// and caches it so a Run and its ETA share one correction pass.
  #[test]
  fn resolve_stream_program_corrects_and_caches_when_armed_with_a_mesh() {
    let mut app = app_disconnected();
    app.ui.set_program(vec!["G0 X0 Y0 Z5".to_string(), "G1 Z-1 F100".to_string(), "G1 X20 Y0 F100".to_string()], None);
    app.ui.autolevel_enabled = true;
    app.profile.mesh = Some(test_mesh(0.5));

    let lines = app.resolve_stream_program().expect("a probed mesh corrects successfully");
    assert_eq!(lines[0], "G90 G21 G94", "the corrected output opens with the canonical header");
    // The cut Z is shifted by the uniform +0.5 mesh (-1 → -0.5), proving the correction actually ran.
    assert!(
      lines.iter().any(|l| l.contains("G1") && l.contains("Z-0.500")),
      "the corrected cut Z must carry the mesh offset; got {lines:?}",
    );
    // The cache is populated and reused (same allocation) on a second call.
    let cached = app.autolevel_cache.as_ref().expect("the corrected program is cached").clone();
    let again = app.resolve_stream_program().expect("the second resolve reuses the cache");
    assert!(std::sync::Arc::ptr_eq(&cached, &again), "a second resolve must reuse the cached Arc, not re-correct");
  }

  /// The AutolevelToggle intent flips the armed flag AND invalidates both the corrected cache and any stale
  /// simulation (a sim over one program shape must not drive the live ETA for the other).
  #[test]
  fn the_autolevel_toggle_invalidates_the_cache_and_a_stale_simulation() {
    let mut app = app_disconnected();
    app.ui.set_program(vec!["G0 X0 Y0 Z-1".to_string(), "G1 X20 Y0 F100".to_string()], None);
    app.ui.autolevel_enabled = true;
    app.profile.mesh = Some(test_mesh(0.5));
    // Prime the cache and a simulation (both now over the corrected program).
    app.resolve_stream_program().expect("corrects");
    app.handle_intent(Intent::Simulate);
    assert!(app.autolevel_cache.is_some() && app.simulated.is_some(), "cache + simulation are primed");

    // Toggling OFF must clear both so nothing stale survives the change of program shape.
    app.handle_intent(Intent::AutolevelToggle(false));
    assert!(!app.ui.autolevel_enabled, "the toggle flips the armed flag");
    assert!(app.autolevel_cache.is_none(), "the toggle must invalidate the corrected cache");
    assert!(app.simulated.is_none(), "the toggle must drop a simulation built over the other program shape");
  }

  /// Changing the `correct_rapids` correction option flips the config and invalidates the cached corrected program
  /// (and any simulation over it), so the next stream/simulate re-corrects under the new setting.
  #[test]
  fn setting_correct_rapids_invalidates_the_corrected_cache() {
    let mut app = app_disconnected();
    app.ui.set_program(vec!["G0 X0 Y0 Z-1".to_string(), "G1 X20 Y0 F100".to_string()], None);
    app.ui.autolevel_enabled = true;
    app.profile.mesh = Some(test_mesh(0.5));
    app.resolve_stream_program().expect("corrects");
    assert!(app.autolevel_cache.is_some(), "the cache is primed");
    assert!(app.ui.autolevel_cfg.correct_rapids, "correct_rapids defaults on");

    app.handle_intent(Intent::SetCorrectRapids(false));
    assert!(!app.ui.autolevel_cfg.correct_rapids, "the config flag flips");
    assert!(app.autolevel_cache.is_none(), "changing the config must invalidate the corrected cache");
  }

  /// Opening a new program invalidates the corrected cache (it belonged to the closed file), so the next stream
  /// re-corrects the freshly loaded lines rather than sending the previous file's corrected output.
  #[test]
  fn opening_a_new_program_invalidates_the_corrected_cache() {
    let mut app = app_disconnected();
    app.ui.set_program(vec!["G0 X0 Y0 Z-1".to_string()], None);
    app.ui.autolevel_enabled = true;
    app.profile.mesh = Some(test_mesh(0.5));
    app.resolve_stream_program().expect("corrects");
    assert!(app.autolevel_cache.is_some(), "the cache is primed for the first file");

    let dir = std::env::temp_dir().join(format!("skirnir-autolevel-open-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("temp dir");
    let path = dir.join("fresh.gcode");
    std::fs::write(&path, "G0 X0 Y0 Z-2\nG1 X10 F100\n").expect("write program");
    app.open_program(&path);
    assert!(app.autolevel_cache.is_none(), "opening a new program must invalidate the corrected cache");
    let _ = std::fs::remove_file(&path);
  }

  /// With autoleveling armed, Simulate builds the ETA over the CORRECTED (longer, subdivided) program, so the
  /// live per-line remaining keys on the same total the corrected stream acks against.
  #[test]
  fn simulate_estimates_over_the_corrected_program_when_armed() {
    let mut app = app_disconnected();
    // A single 20 mm cut: uncorrected it is one G1 line; corrected at grid 10 it subdivides into 2 sub-moves, so
    // the corrected timeline has MORE lines than the source.
    app.ui.set_program(vec!["G0 X0 Y0 Z-1".to_string(), "G1 X20 Y0 F100".to_string()], None);

    // Baseline: autolevel off → timeline over the 2 source lines.
    app.handle_intent(Intent::Simulate);
    let source_lines = app.simulated.as_ref().expect("a timeline").lines.len();
    assert_eq!(source_lines, 2, "the source program is two lines");

    // Arm autolevel with a mesh and re-simulate: the corrected program subdivides the cut, so the timeline grows.
    app.profile.mesh = Some(test_mesh(0.5));
    app.handle_intent(Intent::AutolevelToggle(true));
    app.handle_intent(Intent::Simulate);
    let corrected_lines = app.simulated.as_ref().expect("a timeline over the corrected program").lines.len();
    assert!(corrected_lines > source_lines, "the corrected timeline must span the subdivided program ({corrected_lines} > {source_lines})");
  }

  /// A CorrectionError (here G93 inverse-time on a corrected move) surfaces as an `Err` from the resolver so the
  /// caller can notice "autolevel refused: …" and stream nothing — never fall back to the uncorrected file.
  #[test]
  fn resolve_stream_program_surfaces_a_correction_error() {
    let mut app = app_disconnected();
    // G93 inverse-time on a corrected move is a fail-closed hazard in correct_program.
    app.ui.set_program(
      vec!["G0 X0 Y0 Z-1".to_string(), "G93".to_string(), "G1 X10 F0.5".to_string()],
      None,
    );
    app.ui.autolevel_enabled = true;
    app.profile.mesh = Some(test_mesh(0.5));
    let err = app.resolve_stream_program().expect_err("a G93 corrected move must be refused");
    assert!(err.contains("autolevel refused"), "the refusal must be surfaced as an autolevel notice; got {err:?}");
    assert!(app.autolevel_cache.is_none(), "a refused correction must not populate the cache");
  }

  /// ApplySavedMesh arms autolevel against the persisted mesh and invalidates a stale corrected cache; with no
  /// saved mesh it refuses (a notice, no arming).
  #[test]
  fn apply_saved_mesh_arms_autolevel_and_invalidates_the_cache() {
    let mut app = app_disconnected();
    // No saved mesh: apply must refuse and NOT arm autolevel.
    app.handle_intent(Intent::ApplySavedMesh);
    assert!(!app.ui.autolevel_enabled, "with no saved mesh, apply must not arm autolevel");
    assert!(
      app.view.console.iter().any(|l| l.text.contains("no saved height-map")),
      "the refusal must be explained to the operator",
    );
    // With a saved mesh: apply arms autolevel and drops the stale corrected cache so the next stream re-corrects.
    app.profile.mesh = Some(test_mesh(0.3));
    app.autolevel_cache = Some(std::sync::Arc::from(vec!["stale".to_string()]));
    app.handle_intent(Intent::ApplySavedMesh);
    assert!(app.ui.autolevel_enabled, "apply must arm autolevel when a mesh is saved");
    assert!(app.autolevel_cache.is_none(), "apply must invalidate the stale corrected cache");
  }

  /// The WCS-mismatch warning fires when the armed mesh was probed under a different WCS than the job now runs in,
  /// and stays silent when they match (or the active WCS is unknown).
  #[test]
  fn a_wcs_mismatch_between_the_mesh_and_the_active_wcs_warns() {
    let mut app = app_disconnected();
    app.ui.autolevel_enabled = true;
    let mut mesh = test_mesh(0.3);
    mesh.wcs_index = 1; // the mesh was probed under G55.
    app.profile.mesh = Some(mesh);

    // Job runs under G54 (index 0): mismatch → a warning notice.
    app.view.active_wcs = Some(0);
    app.warn_on_wcs_mismatch();
    assert!(
      app.view.console.iter().any(|l| l.text.contains("may be misaligned")),
      "a WCS mismatch must warn the operator",
    );

    // Job now runs under the SAME WCS (G55): no new warning.
    app.view.clear_console();
    app.view.active_wcs = Some(1);
    app.warn_on_wcs_mismatch();
    assert!(
      !app.view.console.iter().any(|l| l.text.contains("may be misaligned")),
      "a matching WCS must not warn",
    );

    // Active WCS unknown (no `$G` answer): no warning (we cannot compare).
    app.view.clear_console();
    app.view.active_wcs = None;
    app.warn_on_wcs_mismatch();
    assert!(app.view.console.is_empty(), "an unknown active WCS must not warn");
  }

  /// The acquisition stamps the mesh with the WCS it was probed under (from the `[GC:]` active WCS).
  #[test]
  fn acquisition_stamps_the_probed_wcs_onto_the_mesh() {
    let (mut app, _controller) = app_with_engine();
    app.view.active_wcs = Some(2); // probing under G56.
    app.mesh_probe_start(crate::app::autolevel::GridProbeParams::default(), (0.0, 0.0), (10.0, 10.0), (10.0, 10.0));
    assert_eq!(
      app.mesh_probe.as_ref().unwrap().state.mesh().wcs_index,
      2,
      "the mesh must record the WCS it was probed under",
    );
  }

  /// End-to-end over the loopback: with autolevel armed and a mesh, a Run streams the CORRECTED program and the
  /// engine's reported Progress.total is the corrected (longer) line count — so acks count against the right total.
  #[test]
  fn streaming_with_autolevel_reports_the_corrected_total() {
    let _ = crate::i18n::init();
    let (mut app, mut controller) = app_with_engine();
    assert!(controller.inject_line("GrblHAL 1.1f ['$' or '$HELP' for help]"));
    assert!(pump_until(&mut app, |a| a.view.connection == ConnectionState::Idle));
    let _ = pump_collect(&mut app, &mut controller);

    // A 30 mm cut subdivides (grid 10 → 3 sub-moves), so the corrected program is longer than the 2 source lines.
    let source = vec!["G0 X0 Y0 Z-1".to_string(), "G1 X30 Y0 F100".to_string()];
    app.ui.set_program(source.clone(), None);
    app.profile.mesh = Some(test_mesh(0.5));
    app.handle_intent(Intent::AutolevelToggle(true));
    app.start_stream();
    assert!(
      pump_until(&mut app, |a| a.view.progress.total > 0),
      "the engine never reported a stream total; got {:?}",
      app.view.progress,
    );
    let corrected_total = app.autolevel_cache.as_ref().expect("a corrected program was cached").len();
    assert!(corrected_total > source.len(), "the corrected program must be longer than the source");
    assert_eq!(
      app.view.progress.total, corrected_total,
      "Progress.total must be the CORRECTED line count, not the source ({}), so acks key on the right total",
      source.len(),
    );
  }

  /// `stream_time` returns the physics-based UPFRONT total the moment a simulation exists, before any stream — the
  /// acked-rate `estimate` could not (it needs elapsed + acks). Elapsed is zero, remaining is the whole job.
  #[test]
  fn the_upfront_eta_shows_the_simulated_total_before_streaming() {
    let mut app = app_disconnected();
    app.ui.set_program(vec!["G1 X100 F500".to_string()], None);
    app.handle_intent(Intent::Simulate);
    let modeled = app.simulated.as_ref().expect("a timeline").total_seconds;

    // No stream is timing (`stream_started` is None), yet the estimate must carry the modeled total immediately.
    assert!(app.stream_started.is_none());
    let time = app.stream_time();
    assert_eq!(time.elapsed, std::time::Duration::ZERO, "no stream means zero elapsed");
    let total = time.total.expect("the upfront total is populated from the simulation");
    assert!(
      (total.as_secs_f64() - modeled).abs() < 0.01,
      "the upfront total must equal the simulated job time ({} vs {modeled})",
      total.as_secs_f64(),
    );
    let remaining = time.remaining.expect("the upfront remaining is the whole job");
    assert!((remaining.as_secs_f64() - modeled).abs() < 0.01, "before streaming the whole job remains");
  }

  /// The live remaining drains as completed lines advance and rescales when a feed override slows the run. Drives
  /// `stream_time` directly by setting `stream_started` plus a status report carrying `Ln:` and `Ov:`.
  #[test]
  fn the_live_remaining_drains_with_progress_and_rescales_with_a_feed_override() {
    use crate::protocol::status::{MachineState, PositionKind, RunState, StatusReport};
    let mut app = app_disconnected();
    // Three equal feed moves so completing lines drains the remaining in thirds.
    app.ui.set_program(
      vec!["G1 X20 F500".to_string(), "G1 X40 F500".to_string(), "G1 X60 F500".to_string()],
      None,
    );
    app.handle_intent(Intent::Simulate);
    let total = app.simulated.as_ref().expect("a timeline").total_seconds;

    // Pretend a stream is timing so elapsed is populated; the exact elapsed does not matter to the remaining math.
    app.stream_started = Some(Instant::now());

    /// Build a status report with the given current line and feed-override percent (rapid/spindle at 100 %).
    fn status(line: u32, feed_ov: u32) -> StatusReport {
      StatusReport {
        machine_state: MachineState { state: RunState::Run, substate: None },
        position_kind: PositionKind::Machine,
        position: Vec::new(),
        wco: None,
        feed_speed: None,
        overrides: Some((feed_ov, 100, 100)),
        pins: Vec::new(),
        buffer: None,
        line: Some(line),
      }
    }

    // At line 0 (nothing completed), the full job remains.
    app.view.status = Some(status(0, 100));
    let at_start = app.stream_time().remaining.expect("remaining").as_secs_f64();
    assert!((at_start - total).abs() < 0.01, "at the start the whole job remains ({at_start} vs {total})");

    // After one of three equal lines, the remaining drops below the whole.
    app.view.status = Some(status(1, 100));
    let after_one = app.stream_time().remaining.expect("remaining").as_secs_f64();
    assert!(after_one < at_start, "completing a line drains the remaining ({after_one} < {at_start})");

    // A 50 % feed override doubles the remaining feed time relative to 100 % at the same completed line.
    app.view.status = Some(status(1, 50));
    let slowed = app.stream_time().remaining.expect("remaining").as_secs_f64();
    assert!(
      (slowed - after_one * 2.0).abs() < 0.01,
      "a 50 % feed override must double the remaining feed time ({slowed} vs {after_one})",
    );
  }

  /// With NO simulation stored, `stream_time` keeps the legacy acked-rate behaviour exactly: no projection before a
  /// stream is timing, and the acked-rate projection once one is.
  #[test]
  fn without_a_simulation_the_eta_falls_back_to_the_acked_rate_estimate() {
    let mut app = app_disconnected();
    assert!(app.simulated.is_none());
    // No stream timing and no simulation: the empty estimate, exactly as before.
    assert_eq!(app.stream_time(), super::super::progress::TimeEstimate::default());

    // A timing stream with acks projects from the acked rate (the unchanged fallback path).
    app.stream_started = Some(Instant::now() - Duration::from_secs(10));
    app.view.progress = super::super::view_state::Progress { sent: 10, acked: 10, total: 40 };
    let time = app.stream_time();
    assert!(time.remaining.is_some(), "the acked-rate fallback projects once a stream is timing with acks");
  }

  /// The live `completed_lines` prefers the firmware-reported `Ln:` over the host ack count, since `Ln:` is the
  /// line the controller is actually executing (it leads the ack cursor). When no `Ln:` is present it falls back to
  /// the acked count.
  #[test]
  fn the_live_completed_lines_prefer_the_firmware_reported_line_over_acks() {
    use crate::protocol::status::{MachineState, PositionKind, RunState, StatusReport};
    let mut app = app_disconnected();
    app.ui.set_program(
      vec!["G1 X20 F500".to_string(), "G1 X40 F500".to_string(), "G1 X60 F500".to_string()],
      None,
    );
    app.handle_intent(Intent::Simulate);
    app.stream_started = Some(Instant::now());
    // Acks say 1 line done, but the firmware reports it is on line 2 (`Ln:`): the remaining must reflect 2 done.
    app.view.progress = super::super::view_state::Progress { sent: 3, acked: 1, total: 3 };
    let report = StatusReport {
      machine_state: MachineState { state: RunState::Run, substate: None },
      position_kind: PositionKind::Machine,
      position: Vec::new(),
      wco: None,
      feed_speed: None,
      overrides: None,
      pins: Vec::new(),
      buffer: None,
      line: Some(2),
    };
    app.view.status = Some(report);
    let with_ln = app.stream_time().remaining.expect("remaining").as_secs_f64();

    // Drop the `Ln:` field: now the ack count (1) drives, leaving MORE remaining than the `Ln:`-led case (2).
    if let Some(s) = app.view.status.as_mut() {
      s.line = None;
    }
    let with_acks = app.stream_time().remaining.expect("remaining").as_secs_f64();
    assert!(
      with_acks > with_ln,
      "the `Ln:`-led case completes more lines, so it has less remaining than the ack-led fallback ({with_ln} < {with_acks})",
    );
  }

  /// Build a minimal [`StatusReport`] reporting the given run state, for driving the elapsed-clock state machine.
  fn status_in(state: crate::protocol::status::RunState) -> crate::protocol::status::StatusReport {
    use crate::protocol::status::{MachineState, PositionKind, StatusReport};
    StatusReport {
      machine_state: MachineState { state, substate: None },
      position_kind: PositionKind::Machine,
      position: Vec::new(),
      wco: None,
      feed_speed: None,
      overrides: None,
      pins: Vec::new(),
      buffer: None,
      line: None,
    }
  }

  /// THE BUG: the dock elapsed clock kept counting after the job finished and the machine returned to Idle. The fix
  /// LATCHES the finish on genuine completion (all lines acked + Idle) so the displayed elapsed FREEZES at its final
  /// value rather than ticking off the live `stream_started.elapsed()`.
  #[test]
  fn the_elapsed_clock_freezes_once_the_job_completes_and_the_machine_is_idle() {
    use crate::protocol::status::RunState;
    use crate::protocol::ConnectionState;
    let mut app = app_disconnected();
    // A run is timing: started 30s ago, mid-stream (4 of 10 acked), machine running.
    app.stream_started = Some(Instant::now() - Duration::from_secs(30));
    app.view.connection = ConnectionState::Streaming;
    app.view.progress = super::super::view_state::Progress { sent: 10, acked: 4, total: 10 };
    app.view.status = Some(status_in(RunState::Run));
    app.track_stream_clock();
    assert!(app.stream_finished_at.is_none(), "mid-stream the clock must not freeze");

    // A TRANSIENT mid-stream Idle (planner momentarily drained, lines still outstanding) must NOT freeze it.
    app.view.status = Some(status_in(RunState::Idle));
    app.track_stream_clock();
    assert!(app.stream_finished_at.is_none(), "a transient Idle with lines outstanding must not freeze the clock");

    // Genuine completion: every line acked AND the machine settled to Idle. The finish latches now.
    app.view.progress = super::super::view_state::Progress { sent: 10, acked: 10, total: 10 };
    app.view.connection = ConnectionState::Idle; // the reducer's complete_if_drained returned to Idle.
    app.view.status = Some(status_in(RunState::Idle));
    app.track_stream_clock();
    let latched = app.stream_finished_at.expect("the finish must latch on genuine completion");

    // The displayed elapsed is now FROZEN: it is `finished − started`, not `now − started`, so repeated frames
    // (and the re-latch guard) leave it fixed. Read it twice across a real gap and assert it does not advance.
    let frozen = app.stream_time().elapsed;
    std::thread::sleep(Duration::from_millis(20));
    app.track_stream_clock(); // a later frame must not re-stamp the latch...
    assert_eq!(app.stream_finished_at, Some(latched), "the finish latch is stamped once, not re-stamped each frame");
    assert_eq!(app.stream_time().elapsed, frozen, "the elapsed must be frozen at its final value, not keep counting");
  }

  /// A fresh run after a completed one must CLEAR the frozen latch and time from zero again, and a feed-`Hold`
  /// (a pause, not the end) must keep the clock running.
  #[test]
  fn a_new_run_clears_the_freeze_and_a_hold_keeps_counting() {
    use crate::protocol::status::RunState;
    use crate::protocol::ConnectionState;
    let mut app = app_disconnected();
    // A completed, frozen run.
    app.stream_started = Some(Instant::now() - Duration::from_secs(30));
    app.stream_finished_at = Some(Instant::now());
    app.view.progress = super::super::view_state::Progress { sent: 10, acked: 10, total: 10 };

    // A new stream begins (lifecycle re-enters Streaming): the freeze clears and timing restarts from now.
    app.view.connection = ConnectionState::Streaming;
    app.view.progress = super::super::view_state::Progress { sent: 0, acked: 0, total: 8 };
    app.view.status = Some(status_in(RunState::Run));
    app.track_stream_clock();
    assert!(app.stream_finished_at.is_none(), "a fresh run must clear the prior run's frozen finish");
    assert!(app.stream_started.is_some(), "a fresh run re-stamps the start");

    // A feed-hold mid-run is a pause, not the end: the clock keeps running (no freeze).
    app.view.connection = ConnectionState::Hold;
    app.view.status = Some(status_in(RunState::Hold));
    app.track_stream_clock();
    assert!(app.stream_finished_at.is_none(), "a feed-hold must not freeze the clock — it is a pause, not completion");
  }

  /// A graceful Stop / Abort mid-job (the lifecycle reaches a terminal Idle/Alarm with lines still outstanding) also
  /// ends the run, so the clock stops rather than counting on after the operator halted the job.
  #[test]
  fn a_graceful_stop_mid_job_freezes_the_clock() {
    use crate::protocol::status::RunState;
    use crate::protocol::ConnectionState;
    let mut app = app_disconnected();
    app.stream_started = Some(Instant::now() - Duration::from_secs(15));
    app.view.connection = ConnectionState::Streaming;
    app.view.progress = super::super::view_state::Progress { sent: 6, acked: 5, total: 10 }; // only half done.
    app.view.status = Some(status_in(RunState::Run));
    app.track_stream_clock();
    assert!(app.stream_finished_at.is_none(), "still streaming — not frozen");

    // Graceful Stop: the lifecycle returns to Idle with the job incomplete. The run is over, so the clock freezes.
    app.view.connection = ConnectionState::Idle;
    app.view.status = Some(status_in(RunState::Idle));
    app.track_stream_clock();
    assert!(app.stream_finished_at.is_some(), "a terminal Idle after a stop ends the run and freezes the clock");
  }
}
