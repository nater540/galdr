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
}

/// Which probe-flow "slot" owns the shared latch, for the mutual-cancel guard. Exactly one may be armed at a
/// time; starting any flow cancels the others' pending follow-up so a stale one cannot act on a new `[PRB:]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ProbeOpSlot {
  /// The hardened Z touch-off (`pending_zero_z`).
  ZeroZ,
  /// The rotary center-finder (`wizard`).
  Wizard,
  /// The Phase 2 angle-sweep (`sweep`) — flip-verify or runout.
  Sweep,
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

impl SkirnirApp {
  /// Build the app, enumerating serial ports once up front so the connect dropdown is populated immediately.
  /// The `runtime` hosts the engine task; it is only retained when the `serial` connect path can use it.
  pub fn new(runtime: tokio::runtime::Runtime, config: crate::config::Config, config_notices: Vec<String>) -> Self {
    #[cfg(not(feature = "serial"))]
    let _ = runtime; // a gui-only build never opens a port, so the runtime has nothing to host.
    // Load the persisted profile before building the UI state so the connect dropdown and the rotary inputs come
    // up pre-filled from last session. A missing/corrupt/too-new file falls back to defaults (never panics); the
    // optional reason is surfaced as a console notice once the view exists (below).
    let (profile, load_notice) = crate::profile::load();
    // Load the curated per-setting tooltip descriptions once (seeding the on-disk file on first run); the optional
    // reason is surfaced as a console notice below, like the profile load. Never fails — falls back to bundled.
    let (descriptions, descriptions_notice) = crate::app::setting_help::load();
    // The app config (appearance/themes, UI defaults, connection/streaming, toolpath tuning) is loaded ONCE in
    // `run()` and handed in, so the normal startup path does a single read/parse/resolve (no duplicate load). Tests
    // build it directly (or via `config::load_from` against a temp file) and pass it here — the test seam.
    // Build the transient UI state from the profile's last-used values, then layer the config's from-scratch
    // defaults and the resolved appearance over it. The profile still wins for genuinely remembered values (the
    // last port/baud, rotary inputs) — `apply_config_to_ui` only touches the from-scratch knobs and the style.
    let mut ui = UiState::from_prefs(&profile.prefs);
    // At startup apply BOTH the resolved appearance and the from-scratch UI defaults. (A live F5 reload re-applies
    // ONLY the appearance, so it never clobbers the operator's in-session jog/DRO/console changes.)
    apply_appearance(&mut ui, &config);
    apply_ui_defaults(&mut ui, &config);
    #[cfg(feature = "serial")]
    let reconnect = crate::reconnect::ReconnectPolicy::new(config.connection.reconnect.to_policy_config());
    let mut app = SkirnirApp {
      #[cfg(feature = "serial")]
      runtime,
      engine: None,
      #[cfg(feature = "serial")]
      last_endpoint: None,
      #[cfg(feature = "serial")]
      auto_reconnect: false,
      #[cfg(feature = "serial")]
      reconnect,
      #[cfg(feature = "serial")]
      reconnect_at: None,
      view: ViewState::default(),
      ui,
      override_tracker: super::overrides::OverrideTracker::default(),
      stream_started: None,
      stream_finished_at: None,
      simulated: None,
      simulated_default_settings: false,
      jog_stream: None,
      last_status_at: None,
      last_badge: super::badge::BadgeState::Disconnected,
      #[cfg(feature = "serial")]
      pending_probe: None,
      pending_zero_z: None,
      wizard: None,
      sweep: None,
      profile,
      profile_path_override: None,
      config,
      config_path_override: None,
    };
    // Install the loaded tooltip descriptions over the bundled default the `from_prefs` UI came up with.
    app.ui.setting_descriptions = descriptions;
    app.refresh_ports();
    // Surface why the profile fell back to defaults (corrupt / unreadable / no config dir), if it did. A missing
    // file is silent — that is the ordinary first run.
    if let Some(reason) = load_notice {
      app.notice(reason);
    }
    // Likewise surface why setting descriptions fell back to the bundled defaults (corrupt/unreadable file or no
    // config dir), if they did. A first-run seed reports nothing.
    if let Some(reason) = descriptions_notice {
      app.notice(reason);
    }
    // Surface every config load notice (corrupt file, bad colour, unknown active_theme, too-new version), if any.
    // A first-run seed reports none. These tell the operator the config fell back to defaults and why.
    for reason in config_notices {
      app.notice(reason);
    }
    // Tell the operator exactly which file the app reads its config from, so "which file do I edit?" is answered on
    // sight (the OS config path varies by platform: `~/.config/skirnir` on Linux, `~/Library/Application Support/
    // skirnir` on macOS). `None` only when no per-user config base exists — then config edits cannot persist anyway.
    match crate::config::config_path() {
      Some(path) => app.notice(format!("config: {}", path.display())),
      None => app.notice("config: no per-user config directory on this platform — using built-in defaults".to_string()),
    }
    // On a true first run (no remembered port) seed the connect baud from the config's `default_baud` so an operator
    // who pins a non-standard baud in config.json sees it pre-filled. Route it through `sanitize_baud` so a
    // hand-edited out-of-range value (e.g. `0`) is clamped to the accepted range rather than reaching
    // `SerialTransport::open` and failing the port open — the same clamp `from_prefs` applies to a remembered baud.
    // When the profile remembers a port it also remembers that session's baud, which wins (already applied), so
    // leave it untouched.
    if app.profile.prefs.last_port.is_none() {
      app.ui.baud = super::views::sanitize_baud(app.config.connection.default_baud);
    }
    // If the saved port is still present in the freshly-enumerated list, prefer it as the dropdown selection so a
    // reconnect lands on last session's board; otherwise the Galdr-ranked first port (set by `refresh_ports`) stands.
    if let Some(saved) = app.profile.prefs.last_port.clone()
      && app.ui.ports.iter().any(|port| port.path == saved)
    {
      app.ui.selected_port = saved;
    }
    app
  }

  /// Drain every pending engine event into the view state without blocking. Returns whether any event was
  /// seen, so the caller can request an immediate repaint when state changed.
  fn pump_events(&mut self) -> bool {
    let Some(engine) = self.engine.as_mut() else {
      return false;
    };
    let mut saw_any = false;
    let mut dropped = false;
    // Bounded drain: take what is queued this frame. `try_recv` never blocks, so the UI thread stays free.
    while let Some(event) = engine.try_recv() {
      // The engine's terminal event: the transport ended. Note it so we can wind the handle down and (if the
      // operator wanted to stay connected) schedule an auto-reconnect after the drained events are applied.
      if matches!(event, crate::engine::Event::Disconnected(_)) {
        dropped = true;
      }
      // Stamp the arrival of a `<...>` status report so the jog-stream throttle can age its `Bf:` reading. This is
      // the one place the shell observes incoming status, so freshness is recorded exactly when the report lands.
      if matches!(event, crate::engine::Event::Response(crate::protocol::Response::Status(_))) {
        self.last_status_at = Some(Instant::now());
      }
      // A `$<n>=<value>` re-dump line confirms (or refutes) a pending Save: feed it to the staging store so an
      // accepted edit clears and a firmware-rejected one stays dirty and visible rather than silently vanishing
      // (Bug 6). `confirm` is a no-op unless that `$<n>` is awaiting a Save's confirmation, so this is cheap and
      // safe to call on every settings line. We do it before `view.apply` consumes the event.
      if let crate::engine::Event::Response(crate::protocol::Response::Setting { number, value }) = &event {
        self.ui.settings_staging.confirm(*number, value);
      }
      self.view.apply(event);
      saw_any = true;
    }
    // Report any settings the firmware refused during a Save's re-dump, so the operator is told which `$<n>`
    // edits did not take instead of a row quietly reverting. Draining the set here means each rejection is noted
    // once; the row stays dirty so the failed value remains on screen for a retry.
    for number in self.ui.settings_staging.take_rejected() {
      self.view.note(format!("setting ${number} rejected by the firmware — value unchanged"));
    }
    // A live, healthy link clears the backoff so the *next* drop starts fresh: once the lifecycle reaches a
    // connected state, the reconnect succeeded (or never dropped), so reset the schedule.
    #[cfg(feature = "serial")]
    if self.view.connection.is_connected() {
      self.reconnect.on_connected();
    }
    // Fold any freshly-reported override into the tracker so its estimate tracks the firmware: this confirms an
    // in-flight commit landed and adopts an externally-driven change once nothing is in flight.
    if let Some((feed, _rapid, spindle)) = self.view.status.as_ref().and_then(|s| s.overrides) {
      use super::overrides::OverrideAxis;
      self.override_tracker.observe(OverrideAxis::Feed, feed);
      self.override_tracker.observe(OverrideAxis::Spindle, spindle);
    }
    if dropped {
      // The session ended: drop the transient widget state and the override estimate that belonged to it, so a
      // reconnect to a (possibly different) board never resumes a stale edit or steps from the old override. A
      // held continuous jog belongs to the dead link too — stop streaming increments into a gone engine.
      self.ui.on_disconnected();
      self.clear_jog_stream();
      // A pending Z touch-off belongs to the dead link: drop it so the follow-up never fires into a gone engine
      // (the reducer has already cleared the latch on the Disconnected event).
      self.pending_zero_z = None;
      self.last_status_at = None;
      self.override_tracker = super::overrides::OverrideTracker::default();
      self.on_engine_dropped();
    }
    // Request the parser state (`$G`) so `current_tool` reflects the firmware's active tool — the single source
    // for the DRO tool strip and the tool-change banner. We send it on two edges, each fired once (gated on the
    // badge transition, never every frame):
    //   • Becoming ready (a non-live badge → a live one): seeds `current_tool` on connect, BEFORE any first M6.
    //   • Entering `Tool` (an M6 manual tool change): refreshes the tool the operator must insert. The firmware
    //     answers `$G` DURING the M0/M1/M6 hold, so this resolves the banner even for a hold reached mid-stream.
    let badge = self.view.badge_state();
    use super::badge::BadgeState;
    let became_live = !Self::badge_is_live(self.last_badge) && Self::badge_is_live(badge);
    let entered_tool = badge == BadgeState::Tool && self.last_badge != BadgeState::Tool;
    if became_live || entered_tool {
      self.send_line("$G".to_string());
    }
    self.last_badge = badge;
    // Maintain the stream clock from the (now-current) lifecycle: start it the first frame streaming begins,
    // clear it the moment streaming ends, so the dock's elapsed/ETA times exactly one run.
    self.track_stream_clock();
    saw_any
  }

  /// Whether a badge state represents a live, ready link (the firmware can answer commands), as opposed to the
  /// pre-readiness states. Used to fire the connect-time `$G` seed exactly once, on the edge into readiness.
  fn badge_is_live(badge: super::badge::BadgeState) -> bool {
    use super::badge::BadgeState;
    !matches!(badge, BadgeState::Disconnected | BadgeState::Connecting)
  }

  /// React to the engine task ending (its terminal [`crate::engine::Event::Disconnected`]): drop the dead
  /// handle and, when the operator still wants to be connected, schedule the next auto-reconnect attempt per
  /// the backoff policy. An explicit disconnect has already cleared the auto-reconnect desire, so a deliberate
  /// teardown never re-opens the port. Exhausting the attempt budget settles into a clean disconnected state.
  fn on_engine_dropped(&mut self) {
    self.engine = None;
    // A held continuous jog belongs to the now-dead link: stop streaming increments so the pump cannot keep
    // firing `send_command` into a gone engine (which would spam "not connected" notices every frame). This is the
    // single source of truth for tearing the stream down on an engine drop — both drop sites route through here.
    self.clear_jog_stream();
    #[cfg(feature = "serial")]
    {
      if !self.auto_reconnect || self.reconnect_at.is_some() {
        return; // not wanted, or an attempt is already pending.
      }
      let Some((path, _baud)) = self.last_endpoint.clone() else {
        return; // nothing to reconnect to.
      };
      match self.reconnect.next_delay() {
        Some(delay) => {
          self.reconnect_at = Some(Instant::now() + delay);
          self.notice(format!(
            "link dropped; reconnecting to {path} in {:.1}s (attempt {})",
            delay.as_secs_f32(),
            self.reconnect.attempts(),
          ));
        }
        None => {
          // The budget is spent: stop chasing and let the UI settle into a clean disconnected state.
          self.auto_reconnect = false;
          self.notice("link dropped; auto-reconnect gave up — reconnect manually".to_string());
        }
      }
    }
  }

  /// Fire a due auto-reconnect: when a scheduled attempt's deadline has passed and no engine is attached,
  /// re-open the last endpoint. Cheap to call every frame — it is a deadline compare and only acts on the edge.
  /// Returns whether an attempt was fired (so the caller can request a prompt repaint).
  #[cfg(feature = "serial")]
  fn pump_reconnect(&mut self) -> bool {
    let Some(deadline) = self.reconnect_at else {
      return false;
    };
    if self.engine.is_some() || Instant::now() < deadline {
      return false;
    }
    self.reconnect_at = None;
    if let Some((path, baud)) = self.last_endpoint.clone() {
      self.notice(format!("reconnecting to {path}…"));
      // `connect` re-opens and re-attaches the engine; on failure it surfaces a notice and the next frame's
      // `pump_events` sees no engine, so `on_engine_dropped`'s logic re-arms via the still-running schedule.
      self.connect_inner(&path, baud);
      // If the open failed (no engine attached), schedule the next backoff attempt so we keep trying.
      if self.engine.is_none() && self.auto_reconnect {
        self.on_engine_dropped();
      }
    }
    true
  }

  /// Maintain the dock's elapsed/ETA clock across one run. Three edges, all idempotent so this is safe to call
  /// every drain:
  /// - **Stream starts** (lifecycle enters `Streaming` with no run timing): stamp `stream_started = now` and clear
  ///   any prior finish latch, so a fresh run times from zero.
  /// - **Run ends** (a run is timing and is now over): latch `stream_finished_at = now` ONCE, freezing the elapsed
  ///   the dock shows. "Over" is genuine completion — every program line acked and the machine back at Idle (the
  ///   pure [`super::progress::stream_is_complete`]) — OR the lifecycle has settled to a terminal non-streaming
  ///   state (`Idle`/`Alarm`/`Error`), which also covers a graceful Stop or an Abort mid-job. A feed-`Hold` is NOT
  ///   terminal, so the clock keeps accumulating through a pause and resumes cleanly (the bug was specifically the
  ///   counter never STOPPING after completion, not pause behaviour).
  ///
  /// The finish latch is purely a display freeze: it never touches the streaming lifecycle, which stays host-driven
  /// in the reducer. `stream_started` is no longer cleared when streaming ends (that blanked the clock instead of
  /// freezing it); it is reset only on a fresh run start (here) and on a disconnect.
  fn track_stream_clock(&mut self) {
    use crate::protocol::ConnectionState;
    let state = self.view.connection;
    if state == ConnectionState::Disconnected {
      // The link is gone: forget the run entirely so a later session times fresh (and the dock shows no stale clock).
      self.stream_started = None;
      self.stream_finished_at = None;
      return;
    }
    if state == ConnectionState::Streaming && (self.stream_started.is_none() || self.stream_finished_at.is_some()) {
      // A fresh run begins — either nothing has timed yet, or a PREVIOUS run had already frozen (its finish is
      // latched) and the operator started another. Either way, time from now and drop the prior run's frozen finish.
      self.stream_started = Some(Instant::now());
      self.stream_finished_at = None;
      return;
    }
    // While a run is timing and not yet frozen, latch the finish the moment the run is over.
    if self.stream_started.is_some() && self.stream_finished_at.is_none() {
      let progress = self.view.progress;
      let run_idle = self
        .view
        .status
        .as_ref()
        .map(|s| s.machine_state.state == crate::protocol::status::RunState::Idle)
        .unwrap_or(false);
      let complete = super::progress::stream_is_complete(progress.total, progress.acked, run_idle);
      // A terminal lifecycle state (Idle after `complete_if_drained`/graceful-Stop, or Alarm/Error on abort) also
      // ends the run. `Hold` is excluded so a pause keeps the clock running.
      let terminal = matches!(state, ConnectionState::Idle | ConnectionState::Alarm | ConnectionState::Error);
      if complete || terminal {
        self.stream_finished_at = Some(Instant::now());
      }
    }
  }

  /// Translate this frame's pressed keys into jog/transport intents via the pure [`super::intent::key_to_intent`]
  /// policy. Skipped entirely when a widget (e.g. the console command field) holds keyboard focus, so typing
  /// never drives the machine. Each recognised key is read with `key_pressed` so it fires on the press edge and
  /// then repeats at the OS key-repeat cadence — which gives held-arrow jogging without a separate timer.
  fn pump_hotkeys(&mut self, ctx: &egui::Context, sink: &mut super::intent::IntentSink) {
    use super::intent::{Hotkey, key_to_intent};
    use egui::Key;
    // A focused text edit owns the keyboard; do not steal its arrows/Escape/etc.
    if ctx.memory(|m| m.focused().is_some()) {
      return;
    }
    let badge = self.view.badge_state();
    let step = self.ui.jog_step;
    let feed = self.ui.jog_feed;
    // The egui keys we map, paired with our egui-free [`Hotkey`]; everything else is left to egui. `H`/`R` drive
    // feed-hold and cycle-resume (the toolbar's Hold/Resume) so the operator has one-handed pause/resume; the
    // focus guard above means they only fire when no text field owns the keyboard.
    let bindings = [
      (Key::ArrowLeft, Hotkey::ArrowLeft),
      (Key::ArrowRight, Hotkey::ArrowRight),
      (Key::ArrowUp, Hotkey::ArrowUp),
      (Key::ArrowDown, Hotkey::ArrowDown),
      (Key::PageUp, Hotkey::PageUp),
      (Key::PageDown, Hotkey::PageDown),
      (Key::Escape, Hotkey::Escape),
      (Key::H, Hotkey::FeedHold),
      (Key::R, Hotkey::CycleResume),
    ];
    ctx.input(|i| {
      for (key, hotkey) in bindings {
        if i.key_pressed(key)
          && let Some(intent) = key_to_intent(hotkey, badge, step, feed)
        {
          sink.push(intent);
        }
      }
    });
  }

  /// Carry out one UI intent: the policy layer that turns view intents into engine commands and side effects.
  /// Anything fallible (opening a port, reading a file) surfaces into the console as a notice rather than
  /// panicking — the UI must stay alive through a bad port or unreadable file.
  fn handle_intent(&mut self, intent: Intent) {
    match intent {
      Intent::Connect { path, baud } => self.connect(&path, baud),
      Intent::Disconnect => self.disconnect(),
      Intent::RefreshPorts => self.refresh_ports(),
      Intent::IdentifyPort { path } => self.identify_port(&path),
      Intent::OpenProgram(path) => self.open_program(&path),
      Intent::StartStream => self.start_stream(),
      Intent::Simulate => self.simulate(),
      Intent::SendLine(line) => {
        self.send_line(line);
      }
      Intent::ClearConsole => self.view.clear_console(),
      Intent::Realtime(cmd) => {
        self.send_command(Command::Realtime(cmd));
      }
      Intent::SetOverride { axis, target } => self.set_override(axis, target),
      Intent::Jog { axis, dir, distance, feed } => self.jog(axis, dir, distance, feed),
      Intent::JogStart { axis, dir, feed } => self.jog_start(axis, dir, feed),
      Intent::JogStop => self.jog_stop(),
      Intent::DismissBanner => self.view.dismiss_banner(),
      Intent::ProbeZ { depth, feed, plate_thickness } => self.probe_z(depth, feed, plate_thickness),
      Intent::RequestSettings => self.request_settings(),
      Intent::WriteSetting { number, value } => self.write_setting(number, &value),
      Intent::SaveSettings => self.save_settings(),
      Intent::Home => {
        self.send_line("$H".to_string());
      }
      Intent::RunOrResume => self.run_or_resume(),
      Intent::SetWorkZero { axes } => {
        self.send_line(super::intent::work_zero_line(&axes));
      }
      Intent::RotaryCenterStart { dowel_diameter, index_angle_deg, params } => {
        self.rotary_center_start(dowel_diameter, index_angle_deg, params)
      }
      Intent::RotaryCenterProbe => self.rotary_center_probe(),
      Intent::RotaryCenterMoveToYc => self.rotary_center_move_to_yc(),
      Intent::RotaryCenterWriteWcs => self.rotary_center_write_wcs(),
      Intent::RotaryCenterSetZDatum(datum) => {
        if let Some(run) = self.wizard.as_mut() {
          run.state.z_datum = datum;
        }
      }
      Intent::RotaryCenterCancel => self.wizard = None,
      Intent::ApplySavedRotaryCenter => self.apply_saved_rotary_center(),
      Intent::FlipVerifyStart { angle_deg, axis, dir } => self.flip_verify_start(angle_deg, axis, dir),
      Intent::RunoutStart { n, start_deg, axis, dir } => self.runout_start(n, start_deg, axis, dir),
      Intent::SweepProbe => self.sweep_probe(),
      Intent::FlipVerifyWriteCorrection => self.flip_verify_write_correction(),
      Intent::SweepCancel => self.sweep_cancel(),
    }
  }

  /// Open the serial port and attach the engine in response to a user-initiated connect. Records the endpoint
  /// and arms auto-reconnect so a later unexpected drop is chased, and resets the backoff schedule so this fresh
  /// session starts clean. The actual open is delegated to [`Self::connect_inner`], shared with the
  /// auto-reconnect path. A failure to open is surfaced as a console notice, leaving the app disconnected.
  fn connect(&mut self, path: &str, baud: u32) {
    #[cfg(feature = "serial")]
    {
      // A new user connect supersedes any pending reconnect from a previous endpoint and re-arms the desire.
      self.last_endpoint = Some((path.to_string(), baud));
      self.auto_reconnect = true;
      self.reconnect_at = None;
      self.reconnect.on_connected(); // reset the schedule for this fresh session.
      self.connect_inner(path, baud);
      // Remember this endpoint as the default selection next launch. Mirror it into `UiState` first so the
      // profile snapshot (taken from the live UI state) records exactly what we connected to, even if the connect
      // came from somewhere other than the dropdown.
      self.ui.selected_port = path.to_string();
      self.ui.baud = baud;
      self.save_profile();
    }
    #[cfg(not(feature = "serial"))]
    {
      let _ = (path, baud);
      self.notice("built without the `serial` feature; cannot open a port".to_string());
    }
  }

  /// Open the serial port and attach the engine without touching the auto-reconnect bookkeeping. Shared by the
  /// user [`Self::connect`] and the auto-reconnect [`Self::pump_reconnect`] paths. Done inside the runtime
  /// context so the port registration and the engine's internal `tokio::spawn` have a home. A failure to open is
  /// surfaced as a notice, leaving the app disconnected (the caller decides whether to retry).
  #[cfg(feature = "serial")]
  fn connect_inner(&mut self, path: &str, baud: u32) {
    use crate::transport::serial::SerialTransport;
    // Both opening the port (`open_native_async` registers the stream with tokio's I/O reactor) and the
    // engine's internal `tokio::spawn` need the runtime context. Enter it for the duration of the connect.
    let _guard = self.runtime.enter();
    match SerialTransport::open(path, baud) {
      Ok(transport) => {
        // Build the engine with the config's connection tunables: the idle status-poll cadence and any RX-window
        // override. Defaults reproduce the engine's built-in behaviour, so an unconfigured connection is unchanged.
        let engine_config = crate::engine::EngineConfig {
          idle_poll: std::time::Duration::from_millis(self.config.connection.status_poll_ms),
          rx_window: self.config.connection.rx_window,
        };
        let handle = Engine::connect_with(transport, engine_config);
        self.engine = Some(handle);
        self.view.note_sent(format!("connect {path} @ {baud}"));
      }
      Err(err) => self.notice(format!("connect failed: {err}")),
    }
  }

  /// Tear the connection down at the operator's request. Clears the auto-reconnect desire and any pending
  /// attempt so a deliberate disconnect is final — only an *unexpected* drop reconnects. Sending `Disconnect`
  /// ends the engine task, which emits a terminal [`crate::engine::Event::Disconnected`]; we keep the handle
  /// attached so the next `pump_events` drains that event and applies it to the view (flipping the lifecycle to
  /// `Disconnected` and clearing report-derived state such as an alarm). `on_engine_dropped` then releases the
  /// dead handle — and with the desire already cleared, it does not reconnect. Nulling the handle here instead
  /// would drop the receiver before that terminal event could be drained, leaving the UI stuck in its last state.
  fn disconnect(&mut self) {
    if let Some(engine) = &self.engine {
      engine.send(Command::Disconnect);
    }
    #[cfg(feature = "serial")]
    {
      self.auto_reconnect = false;
      self.reconnect_at = None;
    }
  }

  /// Re-enumerate available serial ports for the connect dropdown.
  fn refresh_ports(&mut self) {
    #[cfg(feature = "serial")]
    {
      self.ui.ports = crate::transport::serial::available_ports();
    }
    #[cfg(not(feature = "serial"))]
    {
      self.ui.ports = Vec::new();
    }
    // Keep the selection valid: drop it if the port vanished, else default to the first available. The list is
    // already Galdr-ranked, so "first" lands on the likely board when present.
    if !self.ui.ports.iter().any(|port| port.path == self.ui.selected_port) {
      self.ui.selected_port = self.ui.ports.first().map(|port| port.path.clone()).unwrap_or_default();
    }
  }

  /// Actively probe a port for grblHAL on demand and surface the verdict in the console. Opening a port toggles
  /// the ESP32-S3's DTR/RTS auto-reset line, so this is user-triggered only and refused while connected — the
  /// engine already owns the live port and a second open would disturb it. The probe is time-bounded AND runs
  /// on the runtime (off the UI thread): the verdict returns through a channel drained each frame, so a 500ms
  /// probe never freezes rendering. Only one probe runs at a time; a second request while one is in flight is
  /// ignored.
  fn identify_port(&mut self, path: &str) {
    #[cfg(feature = "serial")]
    {
      if self.engine.is_some() {
        self.notice("identify skipped: already connected (the port is in use)".to_string());
        return;
      }
      if self.pending_probe.is_some() {
        self.notice("identify already in progress".to_string());
        return;
      }
      let (tx, rx) = std::sync::mpsc::channel();
      self.notice(format!("identifying {path}…"));
      let path = path.to_string();
      let baud = self.ui.baud;
      // Run the open + probe on the runtime so the transport's async reads have a reactor and the UI thread
      // stays free. The verdict is formatted into a console line and sent back; a dropped receiver (the app
      // closing mid-probe) just discards it.
      self.runtime.spawn(async move {
        use crate::transport::probe::{DEFAULT_PROBE_TIMEOUT, ProbeVerdict, probe_grbl};
        use crate::transport::serial::SerialTransport;
        let verdict = match SerialTransport::open(&path, baud) {
          Ok(mut transport) => probe_grbl(&mut transport, DEFAULT_PROBE_TIMEOUT).await,
          Err(err) => ProbeVerdict::Error(err.to_string()),
        };
        let message = match verdict {
          ProbeVerdict::Confirmed => format!("identify {path}: grblHAL confirmed"),
          ProbeVerdict::NoResponse => format!("identify {path}: no grbl response (not the board, or busy)"),
          ProbeVerdict::Error(detail) => format!("identify {path} failed: {detail}"),
        };
        let _ = tx.send(message);
      });
      self.pending_probe = Some(rx);
    }
    #[cfg(not(feature = "serial"))]
    {
      let _ = path;
      self.notice("built without the `serial` feature; cannot identify a port".to_string());
    }
  }

  /// Drain a completed identify probe's verdict into the console, if one finished. Non-blocking: `try_recv`
  /// never waits, so the UI thread is never parked on the probe. Returns whether a verdict was surfaced (so the
  /// caller can request a prompt repaint). Clears the slot once the probe's sender has dropped.
  #[cfg(feature = "serial")]
  fn pump_probe(&mut self) -> bool {
    let Some(rx) = self.pending_probe.as_ref() else {
      return false;
    };
    match rx.try_recv() {
      Ok(message) => {
        self.notice(message);
        self.pending_probe = None;
        true
      }
      // Sender dropped without a message (should not happen, but clears the slot if it does).
      Err(std::sync::mpsc::TryRecvError::Disconnected) => {
        self.pending_probe = None;
        false
      }
      Err(std::sync::mpsc::TryRecvError::Empty) => false,
    }
  }

  /// Load a G-code file into the program dock. A read failure is surfaced, not fatal. The parse of the toolpath
  /// happens once here (in [`UiState::set_program`]), not per frame.
  fn open_program(&mut self, path: &std::path::Path) {
    match std::fs::read_to_string(path) {
      Ok(body) => {
        let lines: Vec<String> = body.lines().map(str::to_string).collect();
        let count = lines.len();
        self.ui.set_program(lines, Some(path.display().to_string()));
        // A fresh program invalidates any prior simulation: the estimate belongs to the file that just closed, so
        // drop it (and its default-settings flag) until the operator re-simulates against the newly loaded lines.
        self.clear_simulation();
        self.notice(format!("loaded {count} lines from {}", path.display()));
      }
      Err(err) => self.notice(format!("open failed: {err}")),
    }
  }

  /// Begin streaming the loaded program. Echoes the line count; the engine drives the per-line flow control.
  /// The program is shared as an `Arc<[String]>`, so streaming never re-clones the whole file.
  fn start_stream(&mut self) {
    if self.ui.program.is_empty() {
      self.notice("no program loaded".to_string());
      return;
    }
    let lines = self.ui.program.clone();
    self.notice(format!("streaming {} lines", lines.len()));
    self.send_command(Command::StreamProgram(lines));
  }

  /// Simulate the loaded program: build a physics-based job-time estimate ([`crate::eta::EtaTimeline`]) over the
  /// loaded lines and stash it on the shell so the dock surfaces an upfront ETA (and a physical live remaining
  /// once streaming). This is a PURE host computation — it parses + runs the shared motion model, sends no engine
  /// command, and needs no live link, so it works while disconnected. The motion configs come from the firmware's
  /// `$$` snapshot in the live [`ViewState::settings`] via [`crate::eta::configs_from_settings`]; every field
  /// falls back to its firmware default when absent, so an empty/partial snapshot still estimates — we flag that
  /// case so the UI can qualify the figure with "(default settings)". A no-op (with a notice) when no program is
  /// loaded, since there is nothing to estimate.
  fn simulate(&mut self) {
    if self.ui.program.is_empty() {
      self.notice("no program to simulate".to_string());
      return;
    }
    // Whether the live settings model carries any `$$` values: with none, every config field defaults, so the
    // estimate is grounded in the firmware's default machine model rather than this board's real config. We flag
    // that so the UI qualifies the ETA rather than presenting a defaulted figure as authoritative.
    let settings = &self.view.settings;
    self.simulated_default_settings = settings.is_empty();
    // Build the planner/motion configs from the snapshot, reading each `$<n>` as an `f64` and letting absent or
    // unparseable values fall back to the firmware default inside `configs_from_settings`.
    let (planner, motion) =
      crate::eta::configs_from_settings(|n| settings.value_of(n).and_then(|s| s.trim().parse::<f64>().ok()));
    let timeline = crate::eta::EtaTimeline::build(&self.ui.program, &planner, &motion);
    let total = timeline.total_seconds;
    let pauses = timeline.pauses.len();
    self.simulated = Some(timeline);
    // Echo a one-line summary so the operator has a record of the simulated total (and any unbounded pauses),
    // using the same `m:ss`/`h:mm:ss` grammar the dock clock shows.
    let clock = super::progress::format_mmss(Some(std::time::Duration::from_secs_f64(total.max(0.0))));
    let qualifier = if self.simulated_default_settings { " (default settings)" } else { "" };
    let pause_note = if pauses > 0 { format!(", {pauses} operator pause(s)") } else { String::new() };
    self.notice(format!("simulated job time ~{clock}{qualifier}{pause_note}"));
  }

  /// Drop any stored simulation and its default-settings flag. Called when a new program is opened, so a stale
  /// estimate from the previous file never drives the dock ETA against the freshly loaded lines.
  fn clear_simulation(&mut self) {
    self.simulated = None;
    self.simulated_default_settings = false;
  }

  /// The toolbar Run/Resume segment: resume from a feed hold with a cycle-start, else start streaming the
  /// loaded program. Mirrors the [`TransportGroup`](super::badge::TransportGroup) decision the view rendered.
  fn run_or_resume(&mut self) {
    use super::badge::{BadgeState, TransportGroup};
    let group = TransportGroup::for_state(self.view.badge_state(), !self.ui.program.is_empty());
    if group.run_is_resume {
      // Held/door-suspended: a cycle-start resumes motion without re-sending the program.
      let _ = self.send_command(Command::Realtime(crate::protocol::RealtimeCommand::CycleStart));
    } else if matches!(self.view.badge_state(), BadgeState::Idle | BadgeState::Check | BadgeState::Sleep) {
      self.start_stream();
    }
  }

  /// Drive a feed/spindle override slider to an absolute target percent. grbl exposes only relative override
  /// steps, so we read the override the firmware last reported in `Ov:` (defaulting to 100% before any report)
  /// and emit the minimal ±10/±1/reset sequence the pure [`super::overrides::override_commands`] computes. Each
  /// step rides the out-of-band real-time path (uncounted), so an override never disturbs the send-ahead
  /// window. The live status reporter will reflect the new value within a poll interval, re-centering the
  /// slider on the firmware's truth.
  fn set_override(&mut self, axis: super::overrides::OverrideAxis, target: u32) {
    use super::overrides::OverrideAxis;
    // The firmware's last-reported override for this axis seeds the tracker; the tracker then steps from its own
    // estimate so back-to-back commits inside one status-poll interval never both base on the same stale value.
    // Read the CACHED override (not the per-report `status.overrides`): `Ov:` is intermittent, so an Ov-less poll
    // would otherwise feed the tracker a spurious 100% and step the relative bytes from the wrong base.
    let (feed, _rapid, spindle) = self.view.overrides();
    let reported = match axis {
      OverrideAxis::Feed => feed,
      OverrideAxis::Spindle => spindle,
    };
    for cmd in self.override_tracker.command(axis, reported, target) {
      self.send_command(Command::Realtime(cmd));
    }
  }

  /// Fetch the firmware's settings into the live model: `$$` dumps every `$<n>=<value>`, and `$ES` enumerates
  /// the metadata (name/unit/bounds) that labels each row. Both are ordinary counted lines the engine streams
  /// and acks; the reducer folds the replies into [`ViewState::settings`]. Sending `$ES` first means a row's
  /// label is usually present by the time its value arrives, so the panel never flickers from `$110` to its
  /// real name. The doc directs senders to learn the UI from `$ES` rather than hardcode it, which this does.
  ///
  /// Alongside the settings enumeration we fetch the firmware's error/alarm code enumeration (`$EE` dumps every
  /// `[ERRORCODE:...]`, `$EA` every `[ALARMCODE:...]`), folded into [`ViewState::codes`] so `error:N`/`ALARM:N`
  /// render with the firmware's own names/descriptions. This is the same operator-triggered "learn the board"
  /// moment as the settings fetch; the static fallback decodes codes even before this lands, so it is pure
  /// enrichment. The firmware advertises `ENUMS` in `[NEWOPT:...]`; an older firmware simply `error`s the
  /// unknown `$EE`/`$EA`, which is surfaced in the console and otherwise harmless.
  fn request_settings(&mut self) {
    self.send_line("$ES".to_string());
    self.send_line("$$".to_string());
    self.send_line("$EE".to_string());
    self.send_line("$EA".to_string());
  }

  /// Write one setting edit as a `$<n>=<value>` line, then re-dump `$$` so the panel reflects what the firmware
  /// actually stored — it clamps/validates and may answer `error:N`, in which case the re-dump shows the value
  /// unchanged. The firmware has no single-setting read (`$<n>` alone is not a command), so a full `$$` re-read
  /// is the authoritative way to confirm the write; a dump is only ~40 short lines. Using the shared
  /// [`crate::protocol::setting_write_line`] builder keeps the wire form in one tested place.
  fn write_setting(&mut self, number: u32, value: &str) {
    let line = crate::protocol::setting_write_line(number, value);
    self.send_line(line);
    // Re-read all settings so the just-written value (or a rejected, unchanged one) is reflected in the model.
    self.send_line("$$".to_string());
  }

  /// Commit every staged settings edit (the explicit Save): flush the dirty store as ordered `$<n>=<value>`
  /// lines through the streaming engine, then `$$` to re-confirm what the firmware actually stored (it
  /// validates/clamps each write and may answer `error:N`, in which case the re-dump shows the value unchanged),
  /// then clear the staging so the rows return to showing live values with no modified markers. Each write flows
  /// through [`Self::send_line`] like any other line, so the engine's flow control is respected; grbl has no
  /// batch, so the lines are independent and only ascending-ordered for predictability. A no-op when nothing is
  /// staged (the Save button is disabled then, but this stays safe if it is ever called regardless).
  fn save_settings(&mut self) {
    // `begin_save` returns the write lines AND arms each edit for confirmation by the `$$` re-dump below — it does
    // NOT clear the staging. Clearing on Save (the old behaviour) silently dropped a firmware-rejected setting:
    // the re-dump reverted the row and the operator never learned the write failed. Now each edit stays dirty and
    // visible until `pump_events` folds the re-dump in via `SettingsStaging::confirm`, which clears an accepted
    // setting and flags a rejected one for the console notice (Bug 6).
    let lines = self.ui.settings_staging.begin_save();
    if lines.is_empty() {
      return;
    }
    for line in lines {
      self.send_line(line);
    }
    self.send_line("$$".to_string());
  }

  /// Send one manual line, echoing it to the console as sent traffic. Returns whether it was actually sent (an
  /// empty line, or a missing/dead engine, yields `false`) so sequencing callers can stop on a failed send.
  fn send_line(&mut self, line: String) -> bool {
    let trimmed = line.trim().to_string();
    if trimmed.is_empty() {
      return false;
    }
    self.view.note_sent(trimmed.clone());
    self.send_command(Command::SendLine(trimmed))
  }

  /// Form and send a step `$J=` jog line via the shared [`super::intent::jog_line`] builder, echoing it.
  fn jog(&mut self, axis: Axis, dir: Dir, distance: f64, feed: f64) {
    let line = super::intent::jog_line(axis, dir, distance, feed);
    self.view.note_sent(line.clone());
    self.send_command(Command::SendLine(line));
  }

  /// Begin a continuous (press-and-hold) jog. Rather than one long move (which a jog-cancel could only stop at
  /// its far boundary — the runaway bug), the held jog is *streamed* as short `$J=` increments by
  /// [`Self::pump_jog_stream`]; the operator stops it by releasing, which fires [`Intent::JogStop`]. The first
  /// increment is due immediately so motion starts without waiting a cadence.
  fn jog_start(&mut self, axis: Axis, dir: Dir, feed: f64) {
    self.jog_stream = Some(JogStream { axis, dir, feed, next_send_at: Instant::now() });
  }

  /// End a continuous jog: stop streaming increments and inject jog-cancel (`0x85`). The firmware flushes the
  /// queued jog blocks and decelerates the active (short) block at its boundary, so motion halts within one
  /// increment's travel. Safe to send when not jogging — the firmware ignores it.
  fn jog_stop(&mut self) {
    self.clear_jog_stream();
    self.send_command(Command::Realtime(crate::protocol::RealtimeCommand::JogCancel));
  }

  /// Tear down any in-progress continuous jog. The single point that clears the streamed-jog state, so every site
  /// that ends a jog — operator release, a disconnect, an engine drop — stops the increment pump the same way.
  fn clear_jog_stream(&mut self) {
    self.jog_stream = None;
  }

  /// Emit the next increment of a held continuous jog if one is active and due. Paced by wall clock so blocks are
  /// produced at roughly the firmware's execution rate, and gated by the firmware's reported planner-blocks-free
  /// (`Bf:`) so a long hold never overruns the queue. Each increment is `feed * BLOCK_SECS` long, so its motion
  /// time — the worst-case stop latency after release — stays ~[`JOG_STREAM_BLOCK_SECS`] regardless of feed. The
  /// increments are not echoed to the console: at several per second the echo would bury real traffic.
  fn pump_jog_stream(&mut self) {
    // No engine means nothing to stream into: clear any lingering jog so the pump cannot keep re-entering a
    // dead-engine send path frame after frame. Belt-and-suspenders with the clear at the engine-drop sites.
    if self.engine.is_none() {
      self.clear_jog_stream();
      return;
    }
    let now = Instant::now();
    // Decide whether this increment is due and what to do, holding a single `&mut` to the stream for the pacing
    // update. We copy out only the scalar fields needed to build the send line, and re-arm `next_send_at` exactly
    // once on the paths that "consume" this slot (a send or a backstop hold) so a held jog paces uniformly.
    let send_line = {
      let Some(stream) = self.jog_stream.as_mut() else {
        return;
      };
      if now < stream.next_send_at {
        return; // not yet due — no state change, retry next frame.
      }
      // This slot is due, so re-arm the pacing deadline exactly once here regardless of whether we end up sending
      // or holding — both outcomes consume the slot and should re-evaluate after one interval.
      stream.next_send_at = now + JOG_STREAM_INTERVAL;
      let (axis, dir, feed) = (stream.axis, stream.dir, stream.feed);
      // Backstop against drift: hold off when the firmware's reported planner queue is nearly full, or when that
      // reading is too stale to trust, so a held jog can never overrun the 32-block queue into a `QueueFull`
      // rejection. `view.status.buffer` carries the last `Bf:` blocks-free; `last_status_at` ages it. A missing
      // `Bf:` (no status yet) skips the gate — the queue is empty early in a jog, so the first sends are safe; a
      // present-but-stale reading instead HOLDS, since a frozen `Bf:` would let the stream run past a queue we can
      // no longer observe.
      let hold = match self.view.status.as_ref().and_then(|s| s.buffer) {
        Some((blocks_free, _)) => {
          let stale =
            self.last_status_at.map(|at| now.duration_since(at) > JOG_STREAM_STATUS_MAX_AGE).unwrap_or(true);
          stale || blocks_free < JOG_STREAM_MIN_BLOCKS_FREE
        }
        None => false, // no `Bf:` yet (fresh connection, queue known-empty): safe to stream.
      };
      if hold {
        None
      } else {
        let distance = feed / 60.0 * JOG_STREAM_BLOCK_SECS;
        Some(super::intent::jog_line(axis, dir, distance, feed))
      }
    };
    if let Some(line) = send_line {
      self.send_command(Command::SendLine(line));
    }
  }

  /// Begin a hardened Z touch-off: send a RELATIVE `G38.2` probe and arm the probe latch, but DEFER the zeroing
  /// until the probe resolves successfully. [`Self::pump_probe_z`] builds and sends a position-independent
  /// `G10 L2` zero from the contact machine-Z only on a `success:1` result, and surfaces a notice (zeroing
  /// nothing) on any failure. This replaces the old fire-and-forget sequence that zeroed unconditionally and
  /// relied on alarm-ordering — a race — to protect a failed probe.
  ///
  /// The probe is wrapped `G91` … `G90` (per `docs/tlo-offsets.md`): under the power-on `G90`, `G38.2 Z-<depth>`
  /// would resolve as an ABSOLUTE target and travel to the wrong place — the probe must advance `<depth>` mm FROM
  /// the current position. The zero is computed on resolution as `G10 L2 P0 Z<contact_Z − plate_thickness>` (see
  /// [`super::probe_flow::zero_z_line`]) so it is independent of where the tool sits when it lands — a jog during
  /// the lost-push window cannot corrupt it. A probe issued while one is pending replaces it (latest wins); a
  /// wizard run in progress is cancelled so the two cannot share the latch.
  fn probe_z(&mut self, depth: f64, feed: f64, plate_thickness: f64) {
    // Starting a ZeroZ probe cancels every other probe flow so the shared latch cannot be claimed by two at once.
    self.cancel_probe_ops_except(ProbeOpSlot::ZeroZ);
    // Arm the latch BEFORE the probe is sent so the result (which can arrive within a frame) always finds an op
    // awaiting it. `begin_probe` supersedes any prior op, matching the "latest request wins" rule.
    self.view.begin_probe(super::view_state::ProbeKind::ZeroZ);
    let lines = [
      // Incremental probe wrapper: probe relative, then restore absolute mode.
      "G91".to_string(),
      format!("G38.2 Z-{depth:.3} F{feed:.0}"),
      "G90".to_string(),
    ];
    let mut all_sent = true;
    for line in lines {
      self.view.note_sent(line.clone());
      if !self.send_command(Command::SendLine(line)) {
        all_sent = false;
        break;
      }
    }
    if all_sent {
      self.pending_zero_z = Some(PendingZeroZProbe {
        inner: super::probe_flow::PendingZeroZ::new(plate_thickness),
        issued_at: Instant::now(),
        polled_at: None,
      });
    } else {
      // The send failed (no engine): there is nothing to await, so drop the latch we just armed rather than
      // leaving it awaiting a result that can never come.
      self.view.fail_probe("probe not sent (not connected)");
      self.pending_zero_z = None;
    }
  }

  /// Drive the hardened Z touch-off one frame: gate the deferred zeroing on the probe latch and run the
  /// push-or-poll fallback. Pure [`super::probe_flow::decide`] chooses the action from the latch outcome and the
  /// elapsed wall-clock; the shell only performs the I/O it names (send the zero line, query `$#`, surface a
  /// notice). Returns whether anything happened, so the caller can request a prompt repaint. No-op when no
  /// touch-off is pending. A disconnect clears the latch (the reducer) AND the pending here, so this abandons
  /// cleanly.
  fn pump_probe_z(&mut self) -> bool {
    use super::probe_flow::{ZeroZAction, decide};
    // Copy out the wall-clock stamps up front so the immutable borrow is released before the `observe_busy`
    // mutation below. No pending touch-off ⇒ nothing to do.
    let (issued_at, polled_at) = match self.pending_zero_z.as_ref() {
      Some(p) => (p.issued_at, p.polled_at),
      None => return false,
    };
    // The latch must belong to THIS flow. If it is gone (a disconnect cleared it) or it belongs to another probe
    // kind (a rotary touch armed it), abandon our follow-up rather than acting on someone else's `[PRB:]` — the
    // `ProbeKind` field exists precisely to route the shared latch.
    match self.view.probe_op.as_ref() {
      Some(op) if op.kind == super::view_state::ProbeKind::ZeroZ => {}
      _ => {
        self.pending_zero_z = None;
        return false;
      }
    }
    let now = Instant::now();
    let since_issue = now.duration_since(issued_at);
    let since_poll = polled_at.map(|at| now.duration_since(at)).unwrap_or(Duration::ZERO);
    // Whether the machine is in a probe CYCLE this frame, from the live status: Run/Hold/Jog/Home are in-cycle;
    // Idle (and anything else) is not. A no-status frame counts as not-busy, but the `seen_cycle` latch below
    // means a not-yet-started probe still cannot be mistaken for "finished".
    let busy_now = self.view.status.as_ref().is_some_and(|s| is_probe_cycle_state(s.machine_state.state));
    // Latch that we have seen the machine in a cycle, so a later return to Idle is trusted as completion (the
    // startup-race guard: the pre-`Run` Idle window must not pass the `$#` fallback gate).
    if busy_now && let Some(p) = self.pending_zero_z.as_mut() {
      p.inner.observe_busy();
    }
    // Snapshot the refreshed pending + latch outcome for the pure decision. Both borrows are read-only and
    // released before any action below mutates `self`.
    let (Some(pending), Some(op)) = (self.pending_zero_z.as_ref(), self.view.probe_op.as_ref()) else {
      return false;
    };
    let probe_finished = pending.inner.probe_finished(busy_now);
    let action = decide(op.last.as_ref(), &pending.inner, probe_finished, since_issue, since_poll);
    match action {
      ZeroZAction::Wait => false,
      ZeroZAction::Zero(zero_line) => {
        self.pending_zero_z = None;
        self.notice("probe contacted — setting work-Z".to_string());
        self.send_line(zero_line);
        true
      }
      ZeroZAction::Fail(reason) => {
        self.pending_zero_z = None;
        self.notice(format!("probe failed: {reason} — work-Z NOT changed"));
        true
      }
      ZeroZAction::Poll => {
        // The immediate `[PRB:]` push did not arrive: retrieve the last probe result via `$#` (its `[PRB:]` line
        // parses through the same path and resolves the latch). Send it once and start the give-up clock.
        if let Some(p) = self.pending_zero_z.as_mut() {
          p.inner.polled = true;
          p.polled_at = Some(now);
        }
        self.send_line("$#".to_string());
        true
      }
      ZeroZAction::GiveUp(reason) => {
        self.pending_zero_z = None;
        self.view.fail_probe(reason.clone());
        self.notice(format!("probe failed: {reason} — work-Z NOT changed"));
        true
      }
    }
  }

  /// Start a fresh rotary center-finder run (DOC-11 §1.2): build the pure wizard state for the given dowel
  /// diameter / index angle with the conservative bench defaults, replacing any run in progress. The operator
  /// then jogs to each approach and triggers the touches.
  fn rotary_center_start(
    &mut self,
    dowel_diameter: f64,
    index_angle_deg: f64,
    params: super::rotary_probe::RotaryProbeParams,
  ) {
    self.wizard = Some(RotaryCenterRun {
      state: super::rotary_center::WizardState::new(dowel_diameter, index_angle_deg),
      params,
      touch_fallback: None,
    });
    self.notice(format!("rotary center-finder: dowel {dowel_diameter:.3} mm @ A{index_angle_deg:.1}°"));
  }

  /// Trigger the wizard's next touch: ask the state machine which touch is due (left Y, right Y, or Z-top), emit
  /// its rotary-safe probe lines, and arm the Phase 0 latch so [`Self::pump_wizard`] can fold the result back in.
  /// Inert if no wizard is running, one is already probing, or the due touch is off-step.
  fn rotary_center_probe(&mut self) {
    use super::rotary_center::WizardStep;
    let Some(run) = self.wizard.as_mut() else {
      self.notice("no rotary center-finder running".to_string());
      return;
    };
    if run.state.is_probing() {
      self.notice("rotary probe already in progress".to_string());
      return;
    }
    // Advance the state machine to the next probing step, getting the touch to issue. The step the wizard is in
    // selects which touch: EnterDowel→left Y, ProbeYLeft(resolved)→right Y, MoveToYc→Z-top.
    let touch = match run.state.step {
      WizardStep::EnterDowel => run.state.begin_y_left(),
      WizardStep::ReadyYRight => run.state.begin_y_right(),
      // The top probe is allowed only AFTER the move to Y_c has been sent (MovedToYc), never from MoveToYc.
      WizardStep::MovedToYc => run.state.begin_z_top(),
      _ => None,
    };
    let Some(touch) = touch else {
      self.notice("no rotary touch is due in this step".to_string());
      return;
    };
    let params = run.params;
    let lines = super::rotary_probe::rotary_safe_probe_lines(touch, params);
    // Starting a rotary touch cancels every other probe flow so the shared latch cannot be claimed by two at once
    // (the pumps also kind-gate, but clearing here is the belt to that suspenders).
    self.cancel_probe_ops_except(ProbeOpSlot::Wizard);
    // Arm the latch BEFORE sending so the result always finds an op awaiting it; the wizard owns the follow-up.
    self.view.begin_probe(super::view_state::ProbeKind::RotaryCenter);
    // Stamp the touch's lost-push fallback so a dropped `[PRB:]` push does not leave the wizard awaiting forever.
    if let Some(run) = self.wizard.as_mut() {
      run.touch_fallback =
        Some(TouchFallback { issued_at: Instant::now(), polled: false, polled_at: None, seen_cycle: false });
    }
    let mut all_sent = true;
    for line in lines {
      self.view.note_sent(line.clone());
      if !self.send_command(Command::SendLine(line)) {
        all_sent = false;
        break;
      }
    }
    if !all_sent {
      // The send failed mid-sequence (no engine): fail the latch and the wizard rather than awaiting forever.
      self.view.fail_probe("rotary probe not sent (not connected)");
      if let Some(run) = self.wizard.as_mut() {
        run.state.abort("probe not sent (not connected)");
      }
    }
  }

  /// Send the wizard's move-to-Y-center positioning move (the mandatory step before the top probe). Emits the
  /// retract + absolute Y move to the computed `Y_c`, then ADVANCES the wizard to `MovedToYc` so the top probe is
  /// unlocked only after the move was actually sent. Inert unless the wizard is at the `MoveToYc` step with a
  /// known center.
  fn rotary_center_move_to_yc(&mut self) {
    use super::rotary_center::WizardStep;
    let Some(run) = self.wizard.as_ref() else {
      return;
    };
    if run.state.step != WizardStep::MoveToYc {
      self.notice("move-to-Yc is not due in this step".to_string());
      return;
    }
    let Some(lines) = run.state.move_to_yc_lines(run.params) else {
      self.notice("Y center not yet known".to_string());
      return;
    };
    let mut all_sent = true;
    for line in lines {
      if !self.send_line(line) {
        all_sent = false;
        break;
      }
    }
    if all_sent && let Some(run) = self.wizard.as_mut() {
      // The move was actually sent: advance so `begin_z_top` (gated on `MovedToYc`) becomes reachable.
      run.state.mark_moved_to_yc();
      self.notice("moved to Y center — probe the dowel top next".to_string());
    }
  }

  /// Write the found center to the active WCS via the wizard's offered `G10 L2` line (Y/Z only, never A). Inert
  /// until the wizard has a computed center (the `Review` step).
  fn rotary_center_write_wcs(&mut self) {
    use super::rotary_center::Wcs;
    let Some(run) = self.wizard.as_ref() else {
      return;
    };
    let Some(line) = run.state.offer_g10(Wcs::Active) else {
      self.notice("no rotary center to write yet".to_string());
      return;
    };
    // Snapshot the found center for persistence BEFORE the borrow of `run` is dropped — `(Y_c, Z_c)` plus the
    // dowel/datum that produced them (DOC-11 §1.3). `y_center`/`z_center` are `Some` here because `offer_g10`
    // returned a line, but fall through cleanly if not rather than unwrapping.
    let setup = match (run.state.y_center(), run.state.z_center()) {
      (Some(y_center), Some(z_center)) => Some(crate::profile::RotarySetup {
        y_center,
        z_center,
        dowel_diameter: run.state.dowel_diameter,
        a_datum_deg: run.state.index_angle_deg,
        z_datum: run.state.z_datum,
      }),
      _ => None,
    };
    // Only claim the WCS write — and persist the center for re-apply — if the `G10` actually went out. A dropped
    // or absent engine makes `send_line` false (and already notices why); claiming success, or saving a center we
    // could not apply, would mislead the operator. Mirror the guarded `move_to_yc` path.
    if !self.send_line(line) {
      return;
    }
    self.notice("wrote rotary center to the active WCS (Y/Z only)".to_string());
    // Persist the center so a later session can re-apply it without re-running the whole center-finder.
    if let Some(setup) = setup {
      self.save_rotary_center(setup);
    }
  }

  /// Re-apply the rotary center saved in the profile (DOC-11 §1.3): re-emit the persisted `G10 L2` line (Y/Z
  /// only, never A) so a restart restores the found center without re-probing. Inert with a notice if nothing
  /// has been saved yet.
  fn apply_saved_rotary_center(&mut self) {
    let Some(setup) = self.profile.rotary else {
      self.notice("no saved rotary center to apply — run the center-finder first".to_string());
      return;
    };
    let line = setup.offer_g10();
    // Don't announce success if the line never left: a dropped/absent engine makes `send_line` false (and already
    // notices why), so a "re-applied" notice would contradict it.
    if !self.send_line(line) {
      return;
    }
    self.notice("re-applied the saved rotary center to the active WCS (Y/Z only)".to_string());
  }

  /// Drive a running rotary touch one frame: fold a resolved latch result into the wizard, or run the SHARED
  /// completion-gated lost-push fallback (`$#` poll, then give up) so a dropped/suppressed `[PRB:]` never leaves
  /// the wizard awaiting forever. Mirrors [`Self::pump_probe_z`] but folds the result into the state machine
  /// instead of zeroing. Returns whether anything changed (for a prompt repaint). No-op when no wizard is running
  /// or it is not awaiting a touch.
  fn pump_wizard(&mut self) -> bool {
    use super::probe_flow::{AwaitAction, await_action};
    // Only act while a touch is in flight (a probing step). Copy the fallback stamps up front so the immutable
    // borrow is released before the `seen_cycle` mutation below.
    let (issued_at, polled_at) = match self.wizard.as_ref() {
      Some(run) if run.state.is_probing() => match &run.touch_fallback {
        Some(f) => (f.issued_at, f.polled_at),
        // Probing but no fallback stamp (e.g. a run restored mid-touch): nothing to pace; treat as just-issued.
        None => (Instant::now(), None),
      },
      _ => return false,
    };
    // The latch must belong to THIS flow. Gone (disconnect) or another kind (a ZeroZ armed it) ⇒ abort the
    // wizard rather than wait forever or act on someone else's `[PRB:]`.
    match self.view.probe_op.as_ref() {
      Some(op) if op.kind == super::view_state::ProbeKind::RotaryCenter => {}
      _ => {
        if let Some(run) = self.wizard.as_mut() {
          run.state.abort("probe latch lost");
        }
        return true;
      }
    }
    // If the latch has resolved, fold the outcome into the wizard and finish the touch.
    let resolved = self.view.probe_op.as_ref().filter(|op| !op.awaiting).and_then(|op| op.last.clone());
    if let Some(outcome) = resolved {
      if let Some(run) = self.wizard.as_mut() {
        run.state.on_probe_result(&outcome);
        run.touch_fallback = None;
      }
      // Consume the latch so the result is fed exactly once (the next touch's `begin_probe` re-arms it).
      self.view.clear_probe_op();
      return true;
    }
    // Still awaiting: run the shared lost-push fallback, gated on the touch having demonstrably finished.
    let now = Instant::now();
    let busy_now = self.view.status.as_ref().is_some_and(|s| is_probe_cycle_state(s.machine_state.state));
    if busy_now && let Some(run) = self.wizard.as_mut() && let Some(f) = run.touch_fallback.as_mut() {
      f.seen_cycle = true;
    }
    let (polled, seen_cycle) = match self.wizard.as_ref().and_then(|r| r.touch_fallback.as_ref()) {
      Some(f) => (f.polled, f.seen_cycle),
      None => return false,
    };
    let probe_finished = seen_cycle && !busy_now;
    let since_issue = now.duration_since(issued_at);
    let since_poll = polled_at.map(|at| now.duration_since(at)).unwrap_or(Duration::ZERO);
    match await_action(polled, probe_finished, since_issue, since_poll) {
      AwaitAction::Wait => false,
      AwaitAction::Poll => {
        if let Some(run) = self.wizard.as_mut() && let Some(f) = run.touch_fallback.as_mut() {
          f.polled = true;
          f.polled_at = Some(now);
        }
        self.send_line("$#".to_string());
        true
      }
      AwaitAction::GiveUp(reason) => {
        if let Some(run) = self.wizard.as_mut() {
          run.state.abort(reason.clone());
          run.touch_fallback = None;
        }
        self.view.fail_probe(reason);
        true
      }
    }
  }

  /// Cancel every OTHER in-flight probe op so only one is ever armed at a time (the cross-contamination guard).
  /// Called when any probe flow starts. The shared latch is kind-routed, but clearing the others' pending state
  /// here means a stale follow-up can never act on a new flow's `[PRB:]`.
  fn cancel_probe_ops_except(&mut self, keep: ProbeOpSlot) {
    if keep != ProbeOpSlot::ZeroZ {
      self.pending_zero_z = None;
    }
    if keep != ProbeOpSlot::Wizard {
      self.wizard = None;
    }
    if keep != ProbeOpSlot::Sweep {
      self.sweep = None;
    }
  }

  /// Start a Phase 2 180°-flip center-verify (DOC-11 §2.1): a two-angle sweep at θ and θ+180 along `axis`/`dir`.
  /// The operator jogs the approach and triggers each touch; on completion [`super::flip_verify`] computes the
  /// residual and offers a position-independent `G10 L2` correction. Cancels any other in-flight probe op.
  fn flip_verify_start(&mut self, angle_deg: f64, axis: super::intent::Axis, dir: super::intent::Dir) {
    self.cancel_probe_ops_except(ProbeOpSlot::Sweep);
    let angles = vec![angle_deg, angle_deg + 180.0];
    self.sweep = Some(SweepRun {
      kind: super::view_state::ProbeKind::FlipVerify,
      sweep: super::angle_sweep::AngleSweep::new(angles, axis, dir),
      // Use the operator-tuned bench params (clearance / side-probe Z / settle / feed / depth), not the placeholder
      // defaults — a flip-verify run with the default side-probe Z would touch at the wrong height and produce a
      // garbage residual that could be applied as a bogus `G10 L2` WCS correction.
      params: self.ui.rotary_bench,
      touch_fallback: None,
    });
    self.notice(format!("180°-flip verify: probe {} at A{angle_deg:.1}° then A{:.1}°", axis.letter(),
      angle_deg + 180.0));
  }

  /// Start a Phase 2 runout report (DOC-11 §2.2): an N-angle sweep (evenly spaced from `start_deg`) along
  /// `axis`/`dir`. READ-ONLY — on completion [`super::runout`] reports TIR / eccentricity; nothing is written.
  /// Cancels any other in-flight probe op. `n < 2` is rejected (TIR needs at least two readings).
  fn runout_start(&mut self, n: usize, start_deg: f64, axis: super::intent::Axis, dir: super::intent::Dir) {
    if n < 2 {
      self.notice("runout needs at least 2 angles".to_string());
      return;
    }
    self.cancel_probe_ops_except(ProbeOpSlot::Sweep);
    let angles = super::angle_sweep::evenly_spaced_angles(n, start_deg);
    self.sweep = Some(SweepRun {
      kind: super::view_state::ProbeKind::Runout,
      sweep: super::angle_sweep::AngleSweep::new(angles, axis, dir),
      // The runout sweep must probe at the operator's dialed-in side-probe height too; the placeholder default
      // would touch off the flank and report meaningless TIR / eccentricity.
      params: self.ui.rotary_bench,
      touch_fallback: None,
    });
    self.notice(format!("runout report: {n} angles along {}", axis.letter()));
  }

  /// Trigger the sweep's next touch: ask the engine for the touch due at the current angle, emit its rotary-safe
  /// probe lines, and arm the latch (with the run's [`ProbeKind`]) so [`Self::pump_sweep`] folds the result back.
  /// Inert if no sweep is running, one is already probing, or no angle remains.
  fn sweep_probe(&mut self) {
    let Some(run) = self.sweep.as_mut() else {
      self.notice("no verify/runout sweep running".to_string());
      return;
    };
    if run.sweep.is_probing() {
      self.notice("sweep probe already in progress".to_string());
      return;
    }
    let Some(touch) = run.sweep.begin_next_touch() else {
      self.notice("no sweep touch is due".to_string());
      return;
    };
    let (kind, params) = (run.kind, run.params);
    let lines = super::rotary_probe::rotary_safe_probe_lines(touch, params);
    // Only this sweep may own the latch now (cross-contamination guard), and arm it BEFORE sending so the result
    // always finds an op awaiting it.
    self.cancel_probe_ops_except(ProbeOpSlot::Sweep);
    self.view.begin_probe(kind);
    if let Some(run) = self.sweep.as_mut() {
      run.touch_fallback =
        Some(TouchFallback { issued_at: Instant::now(), polled: false, polled_at: None, seen_cycle: false });
    }
    let mut all_sent = true;
    for line in lines {
      self.view.note_sent(line.clone());
      if !self.send_command(Command::SendLine(line)) {
        all_sent = false;
        break;
      }
    }
    if !all_sent {
      self.view.fail_probe("sweep probe not sent (not connected)");
      if let Some(run) = self.sweep.as_mut() {
        run.sweep.abort("probe not sent (not connected)");
      }
    }
  }

  /// Write the flip-verify's offered `G10 L2` correction (the verified axis only, never A). Inert unless a
  /// completed flip-verify sweep is present with a computable two-reading result.
  fn flip_verify_write_correction(&mut self) {
    use super::flip_verify::FlipResult;
    use super::rotary_center::Wcs;
    let Some(run) = self.sweep.as_ref() else {
      return;
    };
    if run.kind != super::view_state::ProbeKind::FlipVerify || !run.sweep.is_done() {
      self.notice("no flip-verify correction to write yet".to_string());
      return;
    }
    // The probed axis is the sweep's axis; the readings are complete (is_done). Recover the axis from the first
    // touch description is unnecessary — the run carries it via the sweep's touches, so probe along the same axis.
    let axis = run.sweep.probe_axis();
    let Some(result) = FlipResult::from_readings(axis, run.sweep.readings()) else {
      self.notice("flip-verify needs exactly two readings".to_string());
      return;
    };
    // The correction is a RELATIVE shift of the current work origin by the measured residual, so it needs the
    // current WCO on the verified axis (the machine coordinate of work-0). Without a status report carrying `WCO:`
    // we cannot compute it safely — surface that rather than guess.
    let Some(&current_origin) = self.view.last_wco.get(axis.index()) else {
      self.notice("no WCO yet — request a status report before applying the correction".to_string());
      return;
    };
    let line = result.offer_g10(Wcs::Active, current_origin);
    self.send_line(line);
    self.notice("shifted the active WCS origin by the flip-verify residual".to_string());
  }

  /// Cancel any running Phase 2 sweep, discarding its state.
  fn sweep_cancel(&mut self) {
    self.sweep = None;
  }

  /// Drive a running Phase 2 sweep one frame: fold a resolved latch result into the shared engine, or run the
  /// SAME completion-gated lost-push fallback the other flows use. Kind-routed: acts only when the latch belongs
  /// to THIS sweep's kind (FlipVerify/Runout). Returns whether anything changed. The single pump for both Phase 2
  /// wizards — they differ only in the post-completion compute, which the view/`flip_verify_write_correction` do.
  fn pump_sweep(&mut self) -> bool {
    use super::probe_flow::{AwaitAction, await_action};
    // Only act while a touch is in flight; copy the fallback stamps up front to release the borrow before mutating.
    let (kind, issued_at, polled_at) = match self.sweep.as_ref() {
      Some(run) if run.sweep.is_probing() => {
        let (issued, polled) = match &run.touch_fallback {
          Some(f) => (f.issued_at, f.polled_at),
          None => (Instant::now(), None),
        };
        (run.kind, issued, polled)
      }
      _ => return false,
    };
    // The latch must belong to THIS sweep's kind. Gone or another kind ⇒ abort the sweep rather than act on
    // someone else's `[PRB:]`.
    match self.view.probe_op.as_ref() {
      Some(op) if op.kind == kind => {}
      _ => {
        if let Some(run) = self.sweep.as_mut() {
          run.sweep.abort("probe latch lost");
        }
        return true;
      }
    }
    // Resolved ⇒ fold the outcome into the engine and finish the touch.
    let resolved = self.view.probe_op.as_ref().filter(|op| !op.awaiting).and_then(|op| op.last.clone());
    if let Some(outcome) = resolved {
      if let Some(run) = self.sweep.as_mut() {
        run.sweep.on_probe_result(&outcome);
        run.touch_fallback = None;
      }
      self.view.clear_probe_op();
      return true;
    }
    // Still awaiting ⇒ shared lost-push fallback, gated on the touch having demonstrably finished.
    let now = Instant::now();
    let busy_now = self.view.status.as_ref().is_some_and(|s| is_probe_cycle_state(s.machine_state.state));
    if busy_now && let Some(run) = self.sweep.as_mut() && let Some(f) = run.touch_fallback.as_mut() {
      f.seen_cycle = true;
    }
    let (polled, seen_cycle) = match self.sweep.as_ref().and_then(|r| r.touch_fallback.as_ref()) {
      Some(f) => (f.polled, f.seen_cycle),
      None => return false,
    };
    let probe_finished = seen_cycle && !busy_now;
    let since_issue = now.duration_since(issued_at);
    let since_poll = polled_at.map(|at| now.duration_since(at)).unwrap_or(Duration::ZERO);
    match await_action(polled, probe_finished, since_issue, since_poll) {
      AwaitAction::Wait => false,
      AwaitAction::Poll => {
        if let Some(run) = self.sweep.as_mut() && let Some(f) = run.touch_fallback.as_mut() {
          f.polled = true;
          f.polled_at = Some(now);
        }
        self.send_line("$#".to_string());
        true
      }
      AwaitAction::GiveUp(reason) => {
        if let Some(run) = self.sweep.as_mut() {
          run.sweep.abort(reason.clone());
          run.touch_fallback = None;
        }
        self.view.fail_probe(reason);
        true
      }
    }
  }

  /// Forward a command to the engine if connected; surface a notice if not. Returns whether it was sent.
  fn send_command(&mut self, command: Command) -> bool {
    match &self.engine {
      Some(engine) if engine.send(command) => true,
      Some(_) => {
        self.notice("engine is gone; reconnect".to_string());
        // Route the drop through `on_engine_dropped` rather than nulling `engine` inline, so the same teardown
        // (clearing a held jog stream so its pump cannot re-enter this dead-engine arm every frame) runs here too.
        self.on_engine_dropped();
        false
      }
      None => {
        self.notice("not connected".to_string());
        false
      }
    }
  }

  /// Append a local notice to the console (kept distinct from sent/received traffic).
  fn notice(&mut self, text: String) {
    self.view.note(text);
  }

  /// Re-load the app config from disk and re-apply the APPEARANCE only: re-resolve the palette/toolpath style and
  /// re-skin the live egui visuals so a theme/colour edit takes effect without a restart. The from-scratch UI
  /// defaults (jog/DRO/console knobs) are deliberately NOT re-applied — a reload must preserve the operator's
  /// in-session changes to those (the reported F5 bug where reload wiped a hand-set jog step). The
  /// connection/reconnect knobs are likewise not re-applied to a live session (they bind at connect time). If a
  /// program is loaded, its cached toolpath is re-flattened at the new arc density so an `arc_step_deg` edit takes
  /// effect immediately rather than waiting for a GCode reload. Load failures fall back to defaults with notices,
  /// like the startup load. `ctx` is the live egui context whose visuals are re-skinned; notices surface in the console.
  fn reload_config(&mut self, ctx: &egui::Context) {
    let (config, notices) = match &self.config_path_override {
      Some(path) => crate::config::load_from(path),
      None => crate::config::load(),
    };
    // Appearance ONLY — leave the operator's session-modified UI knobs untouched.
    apply_appearance(&mut self.ui, &config);
    // The toolpath render STYLE just changed (strokes/grid/marker/colours update on next paint), but the cached arc
    // geometry was flattened at the OLD chord density. Re-flatten the loaded program at the new resolution so an
    // `arc_step_deg` edit is visible without reloading the GCode. A no-op when no program is loaded.
    self.ui.reflow_toolpath();
    // Re-skin the live window from the freshly-resolved palette + font scale so the reload is visible immediately.
    let (palette, _) = config.palette();
    apply_theme(ctx, &palette, config.appearance.font_scale);
    self.config = config;
    self.notice("reloaded config.json".to_string());
    for reason in notices {
      self.notice(reason);
    }
  }

  /// Fold the current connection/UI prefs into the in-memory profile, then persist the whole thing to disk. The
  /// rotary `RotarySetup` is written separately at the moment a center is found/saved ([`Self::save_rotary_center`])
  /// — here we only refresh the prefs from the live [`UiState`] so the last port/baud and rotary input defaults
  /// survive. A write failure is surfaced as a notice (never a panic): the in-memory profile is still correct,
  /// only the durable mirror lagged.
  fn save_profile(&mut self) {
    self.snapshot_prefs();
    if let Err(err) = self.persist_profile() {
      self.notice(format!("could not save profile: {err}"));
    }
  }

  /// Refresh the profile's prefs section from the live [`UiState`] (last port/baud, rotary input defaults). The
  /// rotary `RotarySetup` is set separately; this only mirrors the "remember my last entry" widget fields.
  fn snapshot_prefs(&mut self) {
    self.profile.prefs = crate::profile::Prefs {
      last_port: if self.ui.selected_port.is_empty() { None } else { Some(self.ui.selected_port.clone()) },
      baud: self.ui.baud,
      rotary_dowel_diameter: self.ui.rotary_dowel_diameter,
      rotary_index_angle: self.ui.rotary_index_angle,
      rotary_bench: self.ui.rotary_bench,
    };
  }

  /// Write the in-memory profile to disk: the test-injected [`Self::profile_path_override`] when set, else the
  /// default OS config location. The single I/O seam so the persistence wiring is hermetically testable.
  fn persist_profile(&self) -> Result<(), crate::profile::ProfileError> {
    match &self.profile_path_override {
      Some(path) => crate::profile::save_to(&self.profile, path),
      None => crate::profile::save(&self.profile),
    }
  }

  /// Persist a found rotary center (DOC-11 §1.3) into the profile and to disk, so a restart can re-apply it
  /// without re-probing. Records `(Y_c, Z_c, D, A-datum, Z-datum)` from the live wizard state. Surfaces a write
  /// failure as a notice rather than panicking; the in-memory center remains usable this session either way.
  fn save_rotary_center(&mut self, setup: crate::profile::RotarySetup) {
    self.profile.rotary = Some(setup);
    self.save_profile();
  }

  /// The current stream's elapsed/ETA estimate. When a simulation exists it is the authoritative source — the
  /// physics-based total shows upfront (before any stream) and a physical remaining drains during one; otherwise
  /// the legacy acked-rate projection stands, exactly as before. The wall-clock elapsed comes from
  /// [`Self::stream_started`]; all the projection math lives in the pure [`super::progress`].
  ///
  /// With a simulation, `completed_lines` prefers the firmware-reported current line (`Ln:`) when present, else
  /// the host's acked-line count — both index the source-line-indexed timeline directly. The live feed/rapid
  /// override fractions come from the `Ov:` percentages (defaulting to 100 % when absent), so a slowed-down run
  /// stretches the remaining estimate the way the machine actually will.
  fn stream_time(&self) -> super::progress::TimeEstimate {
    // Elapsed is FROZEN once the run has finished: measure to the latched finish instant rather than to `now`, so a
    // completed job's clock holds its final value instead of ticking up forever (the keeps-counting-after-Idle bug).
    // While the run is live (no finish latched) it is the running `now − start` delta as before. `None` start (no
    // run has timed) yields the zero default.
    let elapsed = match (self.stream_started, self.stream_finished_at) {
      (Some(start), Some(finished)) => finished.saturating_duration_since(start),
      (Some(start), None) => start.elapsed(),
      (None, _) => Duration::default(),
    };
    if let Some(timeline) = &self.simulated {
      let progress = self.view.progress;
      // Prefer the firmware's reported current line over the host ack count: `Ln:` is the line the controller is
      // actually executing, which leads the host ack cursor during the send-ahead window. Both index the
      // source-line timeline (`lines.len() == program.len()`), so either maps directly to "lines completed".
      let completed = self
        .view
        .status
        .as_ref()
        .and_then(|s| s.line)
        .map(|line| line as usize)
        .unwrap_or(progress.acked);
      // Live override fractions from the `Ov:` percentages (e.g. 100 → 1.0); default to 100 % when no report has
      // carried overrides yet. Only feed and rapid rescale motion time; the spindle override does not change it.
      let (feed_frac, rapid_frac) = self
        .view
        .status
        .as_ref()
        .and_then(|s| s.overrides)
        .map(|(feed, rapid, _spindle)| (feed as f64 / 100.0, rapid as f64 / 100.0))
        .unwrap_or((1.0, 1.0));
      let remaining = timeline.remaining_seconds(completed, feed_frac, rapid_frac);
      return super::progress::physics_estimate(elapsed, timeline.total_seconds, remaining);
    }
    // No simulation: keep the acked-rate behaviour exactly — only project once a stream is timing.
    match self.stream_started {
      Some(_) => {
        let progress = self.view.progress;
        super::progress::estimate(elapsed, progress.acked, progress.total)
      }
      None => super::progress::TimeEstimate::default(),
    }
  }

  /// The dock-ETA qualifier the view renders beside the clock: whether the active simulation fell back to default
  /// machine settings, and how many unbounded operator pauses it modeled. `None` when no simulation is stored, so
  /// the dock shows no qualifier and the legacy acked-rate clock stands alone.
  fn eta_qualifier(&self) -> Option<super::views::EtaQualifier> {
    self.simulated.as_ref().map(|timeline| super::views::EtaQualifier {
      default_settings: self.simulated_default_settings,
      pauses: timeline.pauses.len(),
    })
  }
}

impl eframe::App for SkirnirApp {
  /// Persist the profile once on shutdown, as a backstop to the per-change saves (a center write, a connect).
  /// This captures any prefs the operator changed in the session that did not trigger a save of their own —
  /// e.g. a tweaked rotary input default — so the next launch comes up with them. A write failure here cannot
  /// be surfaced (the window is gone), so it is best-effort and silent; the per-change saves are the primary path.
  fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
    self.snapshot_prefs();
    let _ = self.persist_profile();
  }

  fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
    // 1. Drain engine events into the view state before drawing, so the frame reflects the latest telemetry.
    //    Remember whether anything arrived so we can wake promptly for follow-on telemetry (step 4).
    let saw_event = self.pump_events();
    // Also drain a finished identify probe's verdict (runs off the UI thread; this is a non-blocking poll).
    #[cfg(feature = "serial")]
    let saw_probe = self.pump_probe();
    #[cfg(not(feature = "serial"))]
    let saw_probe = false;
    // Fire a due auto-reconnect attempt (a deadline compare; only acts on the edge). On a fresh attempt this
    // re-attaches the engine, so the frame below reflects the reconnecting link.
    #[cfg(feature = "serial")]
    let fired_reconnect = self.pump_reconnect();
    #[cfg(not(feature = "serial"))]
    let fired_reconnect = false;

    // 2. Build the frame. Views push intents into a per-frame sink; we act on them after layout so a view
    //    never mutates engine state mid-render. eframe 0.34 hands us the root `Ui`; panels are laid out into
    //    it with `show_inside`, and the central panel is what remains after the docked panels claim their
    //    edges. We reach the `Context` (for repaint scheduling and the settings window) via `ui.ctx()`.
    let mut sink = super::intent::IntentSink::new();
    let ctx = ui.ctx().clone();
    use super::metrics::Metrics;
    // The active palette resolved from the config (or the default). `Palette` is `Copy`, so snapshot it once for
    // this frame's panel-frame fills and the banner views, rather than re-borrowing `self.ui` under each closure.
    let palette = self.ui.style.palette;

    // Lift global hotkeys out of egui's per-frame input and turn them into intents (jog by arrows/PageUp-Down,
    // Escape to cancel/abort, hold/resume). Only fire when no text field has keyboard focus, so typing a line
    // in the console command box never jogs the machine. The pure [`key_to_intent`] decides the effect; the
    // shell only does the thin egui→Hotkey translation and the focus guard.
    self.pump_hotkeys(&ctx, &mut sink);

    // F5 hot-reloads config.json: re-resolve the palette/toolpath style and re-skin the live visuals so an operator
    // can iterate on the file without restarting. Gated on no text field holding focus, so typing F5 into a field
    // (it has no text effect, but be consistent with the jog hotkeys) never reloads. A cheap per-frame key check.
    if ctx.input(|i| i.key_pressed(egui::Key::F5)) && !ctx.egui_wants_keyboard_input() {
      self.reload_config(&ctx);
    }

    // The toolbar is a fixed 40px bar (design §03); pin it so it neither collapses nor grows with content. It
    // carries the `panelAlt` (#222222) surface — a shade lighter than the panels below — so the toolbar reads as
    // distinct chrome rather than blending into the body (the design's toolbar fill, previously the panel grey).
    egui::Panel::top("toolbar").exact_size(Metrics::TOOLBAR_H)
      .frame(egui::Frame::NONE.fill(palette.panel_alt))
      .show_inside(ui, |ui| {
        views::toolbar(ui, &self.view, &mut self.ui, &mut sink);
      });

    if self.view.banner.is_some() {
      egui::Panel::top("banner").show_inside(ui, |ui| {
        views::alarm_banner(ui, palette, &self.view, &mut sink);
      });
    } else if self.view.badge_state() == super::badge::BadgeState::Tool {
      // No fault is latched, but the firmware is held for an M6 manual tool change: surface the prominent
      // tool-change affordance in the same top slot (a fault banner, if any, takes precedence above). The Resume
      // action routes through the existing cycle-start path, not a second control. The banner names the tool from
      // `view.current_tool` — the firmware answers `$G` during the hold (the shell nudges it on the transition).
      egui::Panel::top("tool_change").show_inside(ui, |ui| {
        views::tool_change_banner(ui, palette, &self.view, &mut sink);
      });
    }

    // The status bar is a fixed 24px mono strip (design §03).
    egui::Panel::bottom("status").exact_size(Metrics::STATUS_BAR_H).show_inside(ui, |ui| {
      views::status_bar(ui, &self.view, &self.ui);
    });

    // The bottom dock spans the full window width under the body grid (design §03: a single 200px dock hosting
    // the Console and Program tabs across all three columns). It must be laid out BEFORE the side panels so it
    // claims the full width and the columns rise only above it; the status bar, declared earlier, stays below.
    // Pin the height with `exact_size` (like the toolbar/status bars) rather than `resizable` + `default_size`:
    // the dock's body uses a fill-remaining `ScrollArea` (`auto_shrink([false, false])`), and on a resizable
    // panel that height-feedback resolves the panel to most of the window on first layout. Pinning gives a
    // deterministic 200px so the viewport reclaims the rest, and collapsing shrinks it to just the tab strip.
    let dock_h = Metrics::dock_height(self.ui.dock_collapsed);
    // Project the stream's elapsed/ETA from the start stamp and the live acked/total, so the dock can show the
    // design's `m:ss / m:ss` clock. When no stream is timing this is the zero estimate (both times absent).
    let time = self.stream_time();
    // The ETA qualifier — "(default settings)" and any modeled operator pauses — rides alongside the clock when a
    // simulation is stored, so the operator can read the figure with its caveats. `None` falls back to no qualifier.
    let eta_qualifier = self.eta_qualifier();
    egui::Panel::bottom("dock").resizable(false).exact_size(dock_h).show_inside(ui, |ui| {
      views::dock(ui, &self.view, &mut self.ui, time, eta_qualifier, &mut sink);
    });

    // The design body grid is a fixed `268px | 1fr | 286px`: the left (DRO + Jog) and right (Overrides + Probe +
    // Settings) columns are exact widths, not resizable, so the layout matches the mock regardless of window
    // size. Program no longer lives in the right column — it is a dock tab now (design §03).
    //
    // Each panel is given a zero-inner-margin `Frame` (panel-filled) rather than egui's default side-panel frame
    // (`Margin::symmetric(8, 2)`). The default 8px L/R inset would shrink the usable column to 252px while the
    // section headers and DRO/Jog bodies already own their padding (`HEADER_PAD_X`, `DRO_PAD`, `JOG_PAD`), so the
    // content overran the clipped 252px and the rightmost controls ("Zero XYZ", the Z± column) were cut off. With
    // the margin zeroed the full 268/286 is usable and the views' own padding sets the gutters the design intends.
    let column_frame = egui::Frame::NONE.fill(palette.panel);
    egui::Panel::left("controls").resizable(false).exact_size(Metrics::LEFT_COL_W).frame(column_frame)
      .show_inside(ui, |ui| {
        // `auto_shrink([false, false])` pins the content to the full 268px column instead of letting the scroll
        // area shrink to the widest child, which otherwise leaves an unfilled strip on the column's inner edge.
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
          views::dro(ui, &self.view, &mut self.ui, &mut sink);
          ui.separator();
          views::jog(ui, &self.view, &mut self.ui, &mut sink);
        });
      });

    egui::Panel::right("rightcol").resizable(false).exact_size(Metrics::RIGHT_COL_W).frame(column_frame)
      .show_inside(ui, |ui| {
        // `auto_shrink([false, false])`: fill the full fixed column width and height so the content never
        // collapses to its natural size and leaves a bare strip beside it. Settings live only in the toolbar's
        // Settings window now, not as a right-column section.
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
          views::overrides(ui, &self.view, &mut self.ui, &mut sink);
          ui.separator();
          views::probe(ui, &self.view, &mut self.ui, &mut sink);
          ui.separator();
          // The rotary center-finder reads the shell-owned wizard state (the firmware has no pivot concept, so
          // the center lives in skirnir state); pass a borrow so the view stays a pure render of it.
          let wizard = self.wizard.as_ref().map(|run| &run.state);
          // Whether a center was persisted last session (DOC-11 §1.3): the no-run panel offers a one-click
          // re-apply so a restart restores the found center without re-probing.
          let has_saved_center = self.profile.rotary.is_some();
          views::rotary_center(ui, &self.view, &mut self.ui, wizard, has_saved_center, &mut sink);
          ui.separator();
          // The Phase 2 verify/measure panel reads the shared sweep engine + which wizard owns it.
          let sweep = self.sweep.as_ref().map(|run| (&run.sweep, run.kind));
          views::verify_measure(ui, &self.view, &mut self.ui, sweep, &mut sink);
        });
      });

    // The central toolpath panel takes a zero-margin frame too. egui's default central-panel frame insets the
    // content by 8px on every side, which left a black gutter between the left column's right edge and the
    // viewport (the user-flagged band). With no margin the viewport sits flush against both columns — exactly the
    // design's `268 | 1fr | 286` grid, where the columns abut the viewport with no gap. The toolpath view paints
    // its own `INSET` canvas over the rect, so the frame fill never shows through.
    egui::CentralPanel::default().frame(egui::Frame::NONE.fill(palette.inset)).show_inside(ui, |ui| {
      views::toolpath(ui, &self.view, &mut self.ui);
    });

    if self.ui.settings_open {
      let mut open = self.ui.settings_open;
      // Give the window a real default size and let it resize in both axes; the settings list inside fills the
      // available height (see `settings`), so dragging the bottom edge actually grows the list rather than
      // snapping back to a fixed content height (the prior vertical-resize stall).
      egui::Window::new("Settings").open(&mut open).resizable(true).default_size([340.0, 460.0]).show(&ctx, |ui| {
        views::settings(ui, &self.view, &mut self.ui, &mut sink);
      });
      // The window's `X` set `open` false. With unsaved edits staged, defer the close behind the discard
      // confirmation rather than dropping them silently; otherwise close as requested. The confirm modal below
      // resolves a deferred Close by clearing the dialog once Discard is chosen.
      if !open && views::settings_action_needs_confirm(&self.ui.settings_staging) {
        self.ui.pending_settings_action = Some(views::PendingSettingsAction::Close);
      } else {
        self.ui.settings_open = open;
      }
      // Render the "Discard N unsaved change(s)?" modal when a refresh/close is parked; carry out the deferred
      // action once the operator confirms Discard (the staging is already cleared inside the helper).
      if let Some(action) = views::settings_discard_confirm(&ctx, &mut self.ui) {
        match action {
          views::PendingSettingsAction::Refresh => sink.push(Intent::RequestSettings),
          views::PendingSettingsAction::Close => self.ui.settings_open = false,
        }
      }
    }

    // 3. Act on the intents the views emitted this frame, in order.
    for intent in sink.drain() {
      self.handle_intent(intent);
    }

    // 3b. Service a held continuous jog: a JogStart this frame (or an earlier one still held) streams its next
    //     short `$J=` increment when due. Runs after the intents so a fresh JogStart emits immediately.
    self.pump_jog_stream();

    // 3c. Service a pending hardened Z touch-off: gate the deferred zeroing on the probe latch and run the
    //     push-or-poll fallback. Runs after the intents so a ProbeZ issued this frame is tracked before its
    //     follow-up is evaluated next frame (the result cannot have landed yet this frame).
    let saw_probe_z = self.pump_probe_z();

    // 3d. Service a running rotary center-finder: fold a resolved probe latch result into its state machine so the
    //     wizard advances (or aborts) the instant a touch resolves.
    let saw_wizard = self.pump_wizard();

    // 3e. Service a running Phase 2 sweep (flip-verify / runout) the same way, via the shared kind-routed pump.
    let saw_sweep = self.pump_sweep();

    // 4. Schedule the next wake. Only spin the steady timer when there is live traffic to expect: while an
    //    engine is attached (the firmware auto-reports and a stream needs prompt progress updates, and the
    //    engine's events are polled from this loop), or for one extra frame after an event arrived. When
    //    disconnected there is no engine to poll, so we request no repaint and the UI sleeps until the next
    //    user input rather than waking 20×/sec for nothing.
    // A pending probe must keep the timer alive even when disconnected, so its verdict is drained promptly.
    // A pending auto-reconnect likewise keeps the loop awake (while disconnected there is no engine to poll)
    // so the scheduled retry's deadline is actually checked and fired.
    let (probe_pending, reconnect_pending) = {
      #[cfg(feature = "serial")]
      {
        (self.pending_probe.is_some(), self.reconnect_at.is_some())
      }
      #[cfg(not(feature = "serial"))]
      {
        (false, false)
      }
    };
    if saw_event || saw_probe || saw_probe_z || saw_wizard || saw_sweep || fired_reconnect || probe_pending
      || reconnect_pending || self.engine.is_some() || self.jog_stream.is_some() || self.pending_zero_z.is_some()
    {
      ctx.request_repaint_after(REPAINT_INTERVAL);
    }
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
fn apply_theme(ctx: &egui::Context, palette: &Palette, font_scale: f32) {
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
  ctx.set_zoom_factor(font_scale.clamp(0.5, 2.5));

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
