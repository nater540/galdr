//! Lifecycle + per-frame pump: construction, engine-event draining, reconnect, hotkeys, intent
//! dispatch, and the `eframe::App` frame loop. Split out of `shell.rs` (A4); pure relocation.

use super::*;

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
      override_tracker: crate::app::overrides::OverrideTracker::default(),
      stream_started: None,
      stream_finished_at: None,
      simulated: None,
      simulated_default_settings: false,
      autolevel_cache: None,
      jog_stream: None,
      last_status_at: None,
      last_badge: crate::app::badge::BadgeState::Disconnected,
      #[cfg(feature = "serial")]
      pending_probe: None,
      pending_zero_z: None,
      wizard: None,
      datum: None,
      mesh_probe: None,
      sweep: None,
      profile,
      profile_path_override: None,
      config,
      config_path_override: None,
      config_dirty: false,
      appearance_dirty: false,
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
      app.ui.baud = crate::app::views::sanitize_baud(app.config.connection.default_baud);
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
  pub(crate) fn pump_events(&mut self) -> bool {
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
      use crate::app::overrides::OverrideAxis;
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
      self.override_tracker = crate::app::overrides::OverrideTracker::default();
      self.on_engine_dropped();
    }
    // Request the parser state (`$G`) so `current_tool` reflects the firmware's active tool — the single source
    // for the DRO tool strip and the tool-change banner. We send it on two edges, each fired once (gated on the
    // badge transition, never every frame):
    //   • Becoming ready (a non-live badge → a live one): seeds `current_tool` on connect, BEFORE any first M6.
    //   • Entering `Tool` (an M6 manual tool change): refreshes the tool the operator must insert. The firmware
    //     answers `$G` DURING the M0/M1/M6 hold, so this resolves the banner even for a hold reached mid-stream.
    let badge = self.view.badge_state();
    use crate::app::badge::BadgeState;
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
  fn badge_is_live(badge: crate::app::badge::BadgeState) -> bool {
    use crate::app::badge::BadgeState;
    !matches!(badge, BadgeState::Disconnected | BadgeState::Connecting)
  }

  /// React to the engine task ending (its terminal [`crate::engine::Event::Disconnected`]): drop the dead
  /// handle and, when the operator still wants to be connected, schedule the next auto-reconnect attempt per
  /// the backoff policy. An explicit disconnect has already cleared the auto-reconnect desire, so a deliberate
  /// teardown never re-opens the port. Exhausting the attempt budget settles into a clean disconnected state.
  pub(crate) fn on_engine_dropped(&mut self) {
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
  ///   pure [`crate::app::progress::stream_is_complete`]) — OR the lifecycle has settled to a terminal non-streaming
  ///   state (`Idle`/`Alarm`/`Error`), which also covers a graceful Stop or an Abort mid-job. A feed-`Hold` is NOT
  ///   terminal, so the clock keeps accumulating through a pause and resumes cleanly (the bug was specifically the
  ///   counter never STOPPING after completion, not pause behaviour).
  ///
  /// The finish latch is purely a display freeze: it never touches the streaming lifecycle, which stays host-driven
  /// in the reducer. `stream_started` is no longer cleared when streaming ends (that blanked the clock instead of
  /// freezing it); it is reset only on a fresh run start (here) and on a disconnect.
  pub(crate) fn track_stream_clock(&mut self) {
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
      let complete = crate::app::progress::stream_is_complete(progress.total, progress.acked, run_idle);
      // A terminal lifecycle state (Idle after `complete_if_drained`/graceful-Stop, or Alarm/Error on abort) also
      // ends the run. `Hold` is excluded so a pause keeps the clock running.
      let terminal = matches!(state, ConnectionState::Idle | ConnectionState::Alarm | ConnectionState::Error);
      if complete || terminal {
        self.stream_finished_at = Some(Instant::now());
      }
    }
  }

  /// Translate this frame's pressed keys into jog/transport intents via the pure [`crate::app::intent::key_to_intent`]
  /// policy. Skipped entirely when a widget (e.g. the console command field) holds keyboard focus, so typing
  /// never drives the machine. Each recognised key is read with `key_pressed` so it fires on the press edge and
  /// then repeats at the OS key-repeat cadence — which gives held-arrow jogging without a separate timer.
  fn pump_hotkeys(&mut self, ctx: &egui::Context, sink: &mut crate::app::intent::IntentSink) {
    use crate::app::intent::{Hotkey, key_to_intent};
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
  pub(crate) fn handle_intent(&mut self, intent: Intent) {
    match intent {
      Intent::Connect { path, baud } => self.connect(&path, baud),
      Intent::Disconnect => self.disconnect(),
      Intent::RefreshPorts => self.refresh_ports(),
      Intent::IdentifyPort { path } => self.identify_port(&path),
      Intent::OpenProgram(path) => self.open_program(&path),
      Intent::StartStream => self.start_stream(),
      Intent::Simulate => self.simulate(),
      Intent::AutolevelToggle(on) => {
        // Arm/disarm height-map correction for the next stream/simulate. The armed state changes which lines a Run
        // sends, so any cached corrected program is now stale — drop it so the next resolve recomputes (or returns
        // the source verbatim when disarmed). A stored simulation was built over the OTHER program shape (source vs
        // corrected have different line counts), so it too must be dropped or the live per-line ETA would index the
        // wrong timeline — exactly as a fresh program load clears both.
        self.ui.autolevel_enabled = on;
        self.invalidate_autolevel();
        self.clear_simulation();
      }
      Intent::SetCorrectRapids(on) => {
        // The correction config changed, so a cached corrected program (and any simulation over it) is stale.
        self.ui.autolevel_cfg.correct_rapids = on;
        self.invalidate_autolevel();
        self.clear_simulation();
      }
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
        self.send_line(crate::app::intent::work_zero_line(&axes));
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
      Intent::DatumEdgeStart { axis, dir, params } => self.datum_edge_start(axis, dir, params),
      Intent::DatumCornerStart { corner, params } => self.datum_corner_start(corner, params),
      Intent::DatumProbeNext => self.datum_probe_next(),
      Intent::DatumWriteWcs => self.datum_write_wcs(),
      Intent::DatumCancel => self.datum = None,
      Intent::MeshProbeStart { params, min, max, spacing } => self.mesh_probe_start(params, min, max, spacing),
      Intent::MeshProbeNext => self.mesh_probe_next(),
      Intent::MeshProbeCancel => self.mesh_probe = None,
      Intent::MeshClear => self.mesh_clear(),
      Intent::ApplySavedMesh => self.apply_saved_mesh(),
      Intent::FlipVerifyStart { angle_deg, axis, dir } => self.flip_verify_start(angle_deg, axis, dir),
      Intent::RunoutStart { n, start_deg, axis, dir } => self.runout_start(n, start_deg, axis, dir),
      Intent::SweepProbe => self.sweep_probe(),
      Intent::FlipVerifyWriteCorrection => self.flip_verify_write_correction(),
      Intent::SweepCancel => self.sweep_cancel(),
      Intent::SetLanguage(locale) => self.set_language(locale),
      Intent::SetActiveTheme(name) => {
        self.config.appearance.active_theme = name;
        self.mark_appearance_changed();
      }
      Intent::SetFontScale(scale) => {
        // Hold the scale to the same range `apply_theme` clamps to, so the config never records a value the
        // window will refuse to render at.
        self.config.appearance.font_scale = crate::config::clamp_font_scale(scale);
        self.mark_appearance_changed();
      }
      Intent::UpsertTheme { name, theme } => {
        self.config.appearance.themes.insert(name, theme);
        self.mark_appearance_changed();
      }
      Intent::SaveConfig => self.save_config(),
    }
  }

  /// Switch the UI language: select the locale on the global i18n registry (every `tr!` label re-resolves next
  /// frame — immediate mode needs no relayout pass) and record it in the config so the choice persists once the
  /// operator saves.
  fn set_language(&mut self, locale: String) {
    crate::i18n::set_language(&locale);
    self.config.ui.language = locale;
    self.config_dirty = true;
  }

  /// Record that an appearance edit happened: re-resolve the palette/toolpath style into the view state NOW (the
  /// pure half, so the next frame's views already render the new colours and tests observe it synchronously) and
  /// flag the context re-skin for `ui()` (the egui half, which needs the `Context`).
  ///
  /// Deliberately does NOT re-flatten the toolpath: an appearance edit (theme colours, active theme, font scale)
  /// only changes render STYLE, never geometry — arc density (`toolpath.arc_step_deg`) is not an appearance field
  /// and changes only via a config reload (F5), which does its own [`Self::reload_config`] reflow. Reflowing here
  /// re-parsed the whole program on EVERY colour-picker drag frame (a full `parse_xy_path` over every line) for no
  /// geometric change; the cached geometry is already correct under an unchanged arc density.
  fn mark_appearance_changed(&mut self) {
    self.config_dirty = true;
    self.appearance_dirty = true;
    apply_appearance(&mut self.ui, &self.config);
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
    //    it with `show`, and the central panel is what remains after the docked panels claim their
    //    edges. We reach the `Context` (for repaint scheduling and the settings window) via `ui.ctx()`.
    let mut sink = crate::app::intent::IntentSink::new();
    let ctx = ui.ctx().clone();

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

    // The entire window-panel arrangement — toolbar, banners, status bar, dock, columns, viewport — is the
    // shared [`views::shell_panels`], the SAME function the whole-window test harness renders, so the app and
    // its snapshots/interaction tests can never drift (the harness's hand-copied mirror once silently lost the
    // tool-change banner branch). Only the ctx-level floating windows below stay shell-owned.
    // The dock clock projects the stream's elapsed/ETA from the start stamp and the live acked/total; the ETA
    // qualifier — "(default settings)" and any modeled operator pauses — rides alongside it when a simulation is
    // stored, so the operator reads the figure with its caveats.
    let data = views::ShellPanelsData {
      time: self.stream_time(),
      eta_qualifier: self.eta_qualifier(),
      // The rotary center-finder reads the shell-owned wizard state (the firmware has no pivot concept, so the
      // center lives in skirnir state); a borrow keeps the view a pure render of it.
      wizard: self.wizard.as_ref().map(|run| &run.state),
      // Whether a center was persisted last session (DOC-11 §1.3): the no-run panel offers a one-click
      // re-apply so a restart restores the found center without re-probing.
      has_saved_center: self.profile.rotary.is_some(),
      // The datum finder reads the shell-owned wizard state (a found datum is host state written straight to the
      // WCS); a borrow keeps the view a pure render of it.
      datum: self.datum.as_ref().map(|run| &run.state),
      // The height-map acquisition panel reads the shell-owned acquisition state; `has_saved_mesh` offers a clear.
      mesh_probe: self.mesh_probe.as_ref().map(|run| &run.state),
      has_saved_mesh: self.profile.mesh.is_some(),
      // The Phase 2 verify/measure panel reads the shared sweep engine + which wizard owns it.
      sweep: self.sweep.as_ref().map(|run| (&run.sweep, run.kind)),
    };
    views::shell_panels(ui, &self.view, &mut self.ui, data, &mut sink);

    // SKIRNIR_SIZE_TRACE=1: log the dock region's geometry every frame it CHANGES, and force continuous
    // repaints so frame-paced effects reproduce without anyone wiggling the mouse. This is the desktop-truth
    // instrument for the self-resizing-dock investigation — the kittest harness said green three times while
    // the shipped binary disagreed, so the confirmation pass now runs in the REAL eframe loop. Env-gated and
    // near-free when off; deliberately left in place so the operator can run a verification pass themselves.
    if std::env::var_os("SKIRNIR_SIZE_TRACE").is_some() {
      size_trace(&ctx, &self.ui);
      ctx.request_repaint();
    }

    if self.ui.settings_open {
      let mut open = self.ui.settings_open;
      // Give the window a real default size and let it resize in both axes; the settings list inside fills the
      // available height (see `settings`), so dragging the bottom edge actually grows the list rather than
      // snapping back to a fixed content height (the prior vertical-resize stall).
      // The explicit `.id()` keeps egui's remembered position/size keyed on a STABLE token: without it the id
      // derives from the translated title, so switching language "forgot" where the operator had dragged the
      // window and snapped it back to the default placement.
      egui::Window::new(crate::tr!("settings-window-title")).id(egui::Id::new("firmware-settings-window"))
        .open(&mut open).resizable(true)
        .default_size([340.0, 460.0]).show(&ctx, |ui| {
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

    // The app settings dialog (language / theme / font scale / theme editor), toggled from the toolbar gear.
    // Drawn before the intent drain so an edit made this frame is acted on this frame.
    if self.ui.app_settings_open {
      crate::app::app_settings::window(&ctx, &mut self.ui, &self.config, self.config_dirty, &mut sink);
    }

    // 3. Act on the intents the views emitted this frame, in order.
    for intent in sink.drain() {
      self.handle_intent(intent);
    }

    // 3a. An appearance intent re-resolved the palette into the view state (the pure half, in the handler); the
    //     LIVE context re-skin needs the `Context`, so it happens here, once, on the flag's edge.
    if self.appearance_dirty {
      self.appearance_dirty = false;
      let (palette, _) = self.config.palette();
      apply_theme(&ctx, &palette, self.config.appearance.font_scale);
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

    // 3d′. Service a running datum finder the same way, via its kind-routed pump.
    let saw_datum = self.pump_datum();

    // 3d″. Service a running height-map acquisition the same way, via its kind-routed pump.
    let saw_mesh = self.pump_mesh();

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
    if saw_event || saw_probe || saw_probe_z || saw_wizard || saw_datum || saw_mesh || saw_sweep || fired_reconnect
      || probe_pending || reconnect_pending || self.engine.is_some() || self.jog_stream.is_some()
      || self.pending_zero_z.is_some()
    {
      ctx.request_repaint_after(REPAINT_INTERVAL);
    }
  }
}
