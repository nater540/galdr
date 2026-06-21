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
  /// `Streaming` and cleared when it leaves; `None` means no stream is timing. Held in the shell (not the pure
  /// reducer) because it is wall-clock state the egui frame owns — the reducer stays free of `Instant::now()`.
  stream_started: Option<Instant>,
  /// The continuous jog currently being streamed while the operator holds a jog control, or `None`. Owned by the
  /// shell (not the reducer) because pacing the increments is wall-clock work the egui frame drives.
  jog_stream: Option<JogStream>,
  /// Wall-clock instant the last `<...>` status report was observed, or `None` if none has arrived this session.
  /// The jog-stream throttle reads this to age the `Bf:` blocks-free reading: a stale report (older than
  /// [`JOG_STREAM_STATUS_MAX_AGE`]) is no longer trusted, so the stream holds rather than pacing on a frozen queue
  /// reading. Held in the shell (not the reducer) because it is wall-clock state the egui frame owns; cleared on a
  /// disconnect so a stale timestamp never carries into the next session.
  last_status_at: Option<Instant>,
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
  pub fn new(runtime: tokio::runtime::Runtime) -> Self {
    #[cfg(not(feature = "serial"))]
    let _ = runtime; // a gui-only build never opens a port, so the runtime has nothing to host.
    let mut app = SkirnirApp {
      #[cfg(feature = "serial")]
      runtime,
      engine: None,
      #[cfg(feature = "serial")]
      last_endpoint: None,
      #[cfg(feature = "serial")]
      auto_reconnect: false,
      #[cfg(feature = "serial")]
      reconnect: crate::reconnect::ReconnectPolicy::new(crate::reconnect::ReconnectConfig::default()),
      #[cfg(feature = "serial")]
      reconnect_at: None,
      view: ViewState::default(),
      ui: UiState::default(),
      override_tracker: super::overrides::OverrideTracker::default(),
      stream_started: None,
      jog_stream: None,
      last_status_at: None,
      #[cfg(feature = "serial")]
      pending_probe: None,
      pending_zero_z: None,
      wizard: None,
    };
    app.refresh_ports();
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
      self.view.apply(event);
      saw_any = true;
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
    // Maintain the stream clock from the (now-current) lifecycle: start it the first frame streaming begins,
    // clear it the moment streaming ends, so the dock's elapsed/ETA times exactly one run.
    self.track_stream_clock();
    saw_any
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

  /// Keep [`Self::stream_started`] in step with the lifecycle: stamp `now` when streaming starts and clear it
  /// when it stops. Cheap and idempotent — only the edges mutate it — so it is safe to call every drain.
  fn track_stream_clock(&mut self) {
    use crate::protocol::ConnectionState;
    let streaming = self.view.connection == ConnectionState::Streaming;
    match (streaming, self.stream_started.is_some()) {
      (true, false) => self.stream_started = Some(Instant::now()),
      (false, true) => self.stream_started = None,
      _ => {}
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
      Intent::SendLine(line) => {
        self.send_line(line);
      }
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
      Intent::Home => {
        self.send_line("$H".to_string());
      }
      Intent::RunOrResume => self.run_or_resume(),
      Intent::SetWorkZero { axes } => {
        self.send_line(super::intent::work_zero_line(&axes));
      }
      Intent::RotaryCenterStart { dowel_diameter, index_angle_deg } => {
        self.rotary_center_start(dowel_diameter, index_angle_deg)
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
        let handle = Engine::connect(transport);
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
    use super::overrides::{OVERRIDE_NEUTRAL, OverrideAxis};
    // The firmware's last-reported override for this axis seeds the tracker; the tracker then steps from its own
    // estimate so back-to-back commits inside one status-poll interval never both base on the same stale value.
    let (feed, _rapid, spindle) = self
      .view
      .status
      .as_ref()
      .and_then(|s| s.overrides)
      .unwrap_or((OVERRIDE_NEUTRAL, OVERRIDE_NEUTRAL, OVERRIDE_NEUTRAL));
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
    // Starting a ZeroZ probe cancels any rotary wizard run so the shared latch cannot be claimed by both flows.
    self.wizard = None;
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
  fn rotary_center_start(&mut self, dowel_diameter: f64, index_angle_deg: f64) {
    self.wizard = Some(RotaryCenterRun {
      state: super::rotary_center::WizardState::new(dowel_diameter, index_angle_deg),
      params: super::rotary_probe::RotaryProbeParams::default(),
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
    // Starting a rotary touch cancels any pending ZeroZ follow-up so the shared latch cannot be claimed by both
    // flows (the ZeroZ pump also kind-gates, but clearing here is the belt to that suspenders).
    self.pending_zero_z = None;
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
    self.send_line(line);
    self.notice("wrote rotary center to the active WCS (Y/Z only)".to_string());
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

  /// The current stream's elapsed/ETA estimate, or the empty estimate when no stream is timing. The wall-clock
  /// elapsed comes from [`Self::stream_started`]; the projection math lives in the pure [`super::progress`].
  fn stream_time(&self) -> super::progress::TimeEstimate {
    match self.stream_started {
      Some(start) => {
        let progress = self.view.progress;
        super::progress::estimate(start.elapsed(), progress.acked, progress.total)
      }
      None => super::progress::TimeEstimate::default(),
    }
  }
}

impl eframe::App for SkirnirApp {
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
    use super::theme::Theme;

    // Lift global hotkeys out of egui's per-frame input and turn them into intents (jog by arrows/PageUp-Down,
    // Escape to cancel/abort, hold/resume). Only fire when no text field has keyboard focus, so typing a line
    // in the console command box never jogs the machine. The pure [`key_to_intent`] decides the effect; the
    // shell only does the thin egui→Hotkey translation and the focus guard.
    self.pump_hotkeys(&ctx, &mut sink);

    // The toolbar is a fixed 40px bar (design §03); pin it so it neither collapses nor grows with content. It
    // carries the `panelAlt` (#222222) surface — a shade lighter than the panels below — so the toolbar reads as
    // distinct chrome rather than blending into the body (the design's toolbar fill, previously the panel grey).
    egui::Panel::top("toolbar").exact_size(Metrics::TOOLBAR_H)
      .frame(egui::Frame::NONE.fill(Theme::PANEL_ALT))
      .show_inside(ui, |ui| {
        views::toolbar(ui, &self.view, &mut self.ui, &mut sink);
      });

    if self.view.banner.is_some() {
      egui::Panel::top("banner").show_inside(ui, |ui| {
        views::alarm_banner(ui, &self.view, &mut sink);
      });
    }

    // The status bar is a fixed 24px mono strip (design §03).
    egui::Panel::bottom("status").exact_size(Metrics::STATUS_BAR_H).show_inside(ui, |ui| {
      views::status_bar(ui, &self.view, &self.ui, &mut sink);
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
    egui::Panel::bottom("dock").resizable(false).exact_size(dock_h).show_inside(ui, |ui| {
      views::dock(ui, &self.view, &mut self.ui, time, &mut sink);
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
    let column_frame = egui::Frame::NONE.fill(Theme::PANEL);
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
          views::rotary_center(ui, &self.view, &mut self.ui, wizard, &mut sink);
        });
      });

    // The central toolpath panel takes a zero-margin frame too. egui's default central-panel frame insets the
    // content by 8px on every side, which left a black gutter between the left column's right edge and the
    // viewport (the user-flagged band). With no margin the viewport sits flush against both columns — exactly the
    // design's `268 | 1fr | 286` grid, where the columns abut the viewport with no gap. The toolpath view paints
    // its own `INSET` canvas over the rect, so the frame fill never shows through.
    egui::CentralPanel::default().frame(egui::Frame::NONE.fill(Theme::INSET)).show_inside(ui, |ui| {
      views::toolpath(ui, &self.view, &self.ui);
    });

    if self.ui.settings_open {
      let mut open = self.ui.settings_open;
      // Give the window a real default size and let it resize in both axes; the settings list inside fills the
      // available height (see `settings`), so dragging the bottom edge actually grows the list rather than
      // snapping back to a fixed content height (the prior vertical-resize stall).
      egui::Window::new("Settings").open(&mut open).resizable(true).default_size([340.0, 460.0]).show(&ctx, |ui| {
        views::settings(ui, &self.view, &mut self.ui, &mut sink);
      });
      self.ui.settings_open = open;
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
    if saw_event || saw_probe || saw_probe_z || saw_wizard || fired_reconnect || probe_pending || reconnect_pending
      || self.engine.is_some() || self.jog_stream.is_some() || self.pending_zero_z.is_some()
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

  let options = eframe::NativeOptions {
    viewport: egui::ViewportBuilder::default().with_inner_size([1100.0, 720.0]).with_min_inner_size([800.0, 500.0]),
    ..Default::default()
  };

  eframe::run_native("skirnir", options, Box::new(move |cc| {
    // Install the vendored Roboto + JetBrains Mono faces before the theme so the first frame already renders in
    // the design's typefaces — Roboto for UI text, JetBrains Mono (tabular) for the DRO digits and console.
    super::fonts::install(&cc.egui_ctx);
    apply_theme(&cc.egui_ctx);
    Ok(Box::new(SkirnirApp::new(runtime)))
  }))
}

/// Apply the dark visuals so the app matches the [`super::theme::Theme`] palette, mapping the design tokens
/// onto egui's `Visuals.widgets.{noninteractive,inactive,hovered,active}` plus the panel/window/inset fills,
/// 2px control rounding, and the 1px divider stroke. The design maps 1:1 onto these fields.
fn apply_theme(ctx: &egui::Context) {
  use eframe::egui::{CornerRadius, Stroke};
  use super::metrics::Metrics;
  use super::theme::Theme;

  // Spacing first, so every default-sized button/field matches the design's component sheet rather than egui's
  // larger defaults. The design's controls are 6×14 padding, ~22px tall in panels, 2px corners.
  let mut style = (*ctx.global_style()).clone();
  let spacing = &mut style.spacing;
  spacing.button_padding = Metrics::BUTTON_PAD; // §02 buttons: 6px 14px (egui order is x,y).
  spacing.item_spacing = egui::vec2(6.0, 6.0); // §01 default cluster pad.
  spacing.interact_size.y = Metrics::PANEL_CONTROL_H; // §01 spacing legend: 22px control row.
  ctx.set_global_style(style);

  let mut visuals = egui::Visuals::dark();
  visuals.panel_fill = Theme::PANEL;
  visuals.window_fill = Theme::BG;
  visuals.extreme_bg_color = Theme::INSET; // text edits / inset fields.
  visuals.faint_bg_color = Theme::PANEL_ALT; // striped rows / faint surfaces.
  visuals.override_text_color = Some(Theme::TEXT);
  visuals.hyperlink_color = Theme::ACCENT;
  visuals.selection.bg_fill = Theme::ACCENT.gamma_multiply(0.4);
  visuals.selection.stroke = Stroke::new(1.0, Theme::ACCENT);
  visuals.window_stroke = Stroke::new(1.0, Theme::DIVIDER);

  let radius = CornerRadius::same(2);
  let widgets = &mut visuals.widgets;
  // Non-interactive surfaces (labels, separators): panel fill, divider stroke.
  widgets.noninteractive.bg_fill = Theme::PANEL;
  widgets.noninteractive.weak_bg_fill = Theme::PANEL;
  widgets.noninteractive.bg_stroke = Stroke::new(1.0, Theme::DIVIDER);
  widgets.noninteractive.fg_stroke = Stroke::new(1.0, Theme::TEXT_DIM);
  widgets.noninteractive.corner_radius = radius;
  // Inactive (control at rest): widget fill, raised edge.
  widgets.inactive.bg_fill = Theme::WIDGET;
  widgets.inactive.weak_bg_fill = Theme::WIDGET;
  widgets.inactive.bg_stroke = Stroke::new(1.0, Theme::BORDER_RAISED);
  widgets.inactive.fg_stroke = Stroke::new(1.0, Theme::TEXT);
  widgets.inactive.corner_radius = radius;
  // Hovered.
  widgets.hovered.bg_fill = Theme::WIDGET_HOVER;
  widgets.hovered.weak_bg_fill = Theme::WIDGET_HOVER;
  widgets.hovered.bg_stroke = Stroke::new(1.0, Theme::BORDER_RAISED);
  widgets.hovered.fg_stroke = Stroke::new(1.0, Theme::TEXT);
  widgets.hovered.corner_radius = radius;
  // Active / pressed.
  widgets.active.bg_fill = Theme::WIDGET_ACTIVE;
  widgets.active.weak_bg_fill = Theme::WIDGET_ACTIVE;
  widgets.active.bg_stroke = Stroke::new(1.0, Theme::ACCENT);
  widgets.active.fg_stroke = Stroke::new(1.0, Theme::TEXT);
  widgets.active.corner_radius = radius;
  // Open (combo box popups): match active.
  widgets.open.bg_fill = Theme::WIDGET_ACTIVE;
  widgets.open.weak_bg_fill = Theme::WIDGET_ACTIVE;
  widgets.open.bg_stroke = Stroke::new(1.0, Theme::BORDER_RAISED);
  widgets.open.fg_stroke = Stroke::new(1.0, Theme::TEXT);
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
    let mut app = SkirnirApp::new(runtime);
    app.engine = Some(handle);
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
    app.rotary_center_start(6.0, 0.0);
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

  /// Selecting the top-surface Z datum must change the emitted `G10` Z word end-to-end: work-Z0 lands on the
  /// raw probed top (`Z_top`) instead of the axis centerline, while Y stays the axis and no A word appears.
  #[test]
  fn selecting_the_top_surface_datum_writes_z_top_to_the_wcs() {
    use crate::app::rotary_center::{WizardStep, ZDatum};
    let (mut app, mut controller) = app_with_engine();
    flush_handshake(&mut app, &mut controller);

    app.rotary_center_start(6.0, 0.0);
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

    app.rotary_center_start(6.0, 0.0);
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

  /// Pump BOTH probe follow-ups (ZeroZ and wizard) each step, draining events + writes — so a test can prove the
  /// two never act on each other's `[PRB:]` through the shared latch.
  fn pump_both_steps(app: &mut SkirnirApp, controller: &mut LoopbackController, steps: usize) -> Vec<u8> {
    let mut out = Vec::new();
    for _ in 0..steps {
      app.pump_events();
      app.pump_probe_z();
      app.pump_wizard();
      out.extend(controller.drain_written());
      std::thread::sleep(Duration::from_millis(5));
    }
    out
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
    app.rotary_center_start(6.0, 0.0);
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

    app.rotary_center_start(6.0, 0.0);
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
}
