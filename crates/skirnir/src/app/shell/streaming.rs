//! Program streaming + estimation: open/start/resolve a program, autolevel correction, offline
//! simulation, run/resume, line/command sends, and the ETA readouts. Split out of `shell.rs` (A4).

use super::*;

impl SkirnirApp {
  /// Load a G-code file into the program dock. A read failure is surfaced, not fatal. The parse of the toolpath
  /// happens once here (in [`UiState::set_program`]), not per frame.
  pub(crate) fn open_program(&mut self, path: &std::path::Path) {
    match std::fs::read_to_string(path) {
      Ok(body) => {
        let lines: Vec<String> = body.lines().map(str::to_string).collect();
        let count = lines.len();
        self.ui.set_program(lines, Some(path.display().to_string()));
        // A fresh program invalidates any prior simulation: the estimate belongs to the file that just closed, so
        // drop it (and its default-settings flag) until the operator re-simulates against the newly loaded lines.
        self.clear_simulation();
        // The cached height-corrected program belonged to the closed file — drop it so the next stream/simulate
        // re-corrects the freshly loaded lines rather than sending the previous file's corrected output.
        self.invalidate_autolevel();
        self.notice(format!("loaded {count} lines from {}", path.display()));
      }
      Err(err) => self.notice(format!("open failed: {err}")),
    }
  }

  /// Begin streaming the loaded program. Echoes the line count; the engine drives the per-line flow control.
  /// The program is shared as an `Arc<[String]>`, so streaming never re-clones the whole file.
  pub(crate) fn start_stream(&mut self) {
    if self.ui.program.is_empty() {
      self.notice("no program loaded".to_string());
      return;
    }
    // Resolve the lines to actually send: the source file, or — when autoleveling is armed and a mesh exists — the
    // height-corrected rewrite. An `Err` (no mesh, or a program the corrector refuses) surfaces a notice and streams
    // NOTHING, so we never fall back to sending the uncorrected file behind the operator's back.
    let lines = match self.resolve_stream_program() {
      Ok(lines) => lines,
      Err(reason) => {
        self.notice(reason);
        return;
      }
    };
    if self.ui.autolevel_enabled {
      self.warn_on_wcs_mismatch();
      self.notice(format!("streaming {} lines (autolevel: {} source)", lines.len(), self.ui.program.len()));
    } else {
      self.notice(format!("streaming {} lines", lines.len()));
    }
    self.send_command(Command::StreamProgram(lines));
  }

  /// Surface a warning (not a block) if the armed height-map was probed under a DIFFERENT WCS than the job is
  /// about to run in — the mesh indexes work-XY, so a WCS shift moves the surface out from under the correction.
  /// A no-op when no mesh is armed or the active WCS is unknown (no `$G` answer yet). Reads the active WCS from
  /// the `[GC:]` parser state ([`ViewState::active_wcs`]) against the mesh's stamped `wcs_index`.
  pub(crate) fn warn_on_wcs_mismatch(&mut self) {
    let mesh_wcs = self.profile.mesh.as_ref().map(|m| m.wcs_index);
    if let (Some(mesh_wcs), Some(active)) = (mesh_wcs, self.view.active_wcs)
      && mesh_wcs != active
    {
      self.notice(format!(
        "warning: height-map was probed under G{} but the job runs under G{} — the correction may be misaligned",
        54 + mesh_wcs,
        54 + active,
      ));
    }
  }

  /// Resolve the program to stream/estimate. With autoleveling off, that is the source lines verbatim (today's
  /// behavior). With it on, the loaded program is height-corrected through [`crate::app::autolevel::correct_program`]
  /// against the probed [`crate::profile::Profile::mesh`] and the result is cached ([`Self::autolevel_cache`]) so a
  /// Run and its ETA build share ONE correction pass. Returns `Err(reason)` — surfaced as a notice by the caller —
  /// when autoleveling is armed but cannot be applied (no mesh probed, or the corrector refuses the program), so the
  /// caller aborts rather than silently sending an uncorrected file. Never sends a command itself.
  pub(crate) fn resolve_stream_program(&mut self) -> Result<std::sync::Arc<[String]>, String> {
    if !self.ui.autolevel_enabled {
      return Ok(self.ui.program.clone());
    }
    if let Some(cached) = &self.autolevel_cache {
      return Ok(cached.clone());
    }
    let Some(mesh) = self.profile.mesh.as_ref() else {
      return Err("autolevel on but no height map probed".to_string());
    };
    match crate::app::autolevel::correct_program(&self.ui.program, mesh, &self.ui.autolevel_cfg) {
      Ok(lines) => {
        let arc: std::sync::Arc<[String]> = std::sync::Arc::from(lines);
        self.autolevel_cache = Some(arc.clone());
        Ok(arc)
      }
      Err(e) => Err(format!("autolevel refused: {e}")),
    }
  }

  /// Drop the cached corrected program so the next stream/simulate recomputes it. Called whenever an input to the
  /// correction changes: a freshly loaded program, a mesh change, or an autolevel toggle/config edit. Cheap (clears
  /// one `Option`); the recompute is deferred to the next [`Self::resolve_stream_program`].
  pub(crate) fn invalidate_autolevel(&mut self) {
    self.autolevel_cache = None;
  }

  /// Simulate the loaded program: build a physics-based job-time estimate ([`crate::eta::EtaTimeline`]) over the
  /// loaded lines and stash it on the shell so the dock surfaces an upfront ETA (and a physical live remaining
  /// once streaming). This is a PURE host computation — it parses + runs the shared motion model, sends no engine
  /// command, and needs no live link, so it works while disconnected. The motion configs come from the firmware's
  /// `$$` snapshot in the live [`ViewState::settings`] via [`crate::eta::configs_from_settings`]; every field
  /// falls back to its firmware default when absent, so an empty/partial snapshot still estimates — we flag that
  /// case so the UI can qualify the figure with "(default settings)". A no-op (with a notice) when no program is
  /// loaded, since there is nothing to estimate.
  pub(crate) fn simulate(&mut self) {
    if self.ui.program.is_empty() {
      self.notice("no program to simulate".to_string());
      return;
    }
    // Estimate over the SAME lines a Run would send: with autoleveling armed, that is the height-corrected (longer)
    // program, so the ETA and the live per-line remaining key on the same total the stream acks against. A refusal
    // (no mesh / uncorrectable program) surfaces the notice and skips the estimate rather than timing the wrong file.
    let program = match self.resolve_stream_program() {
      Ok(program) => program,
      Err(reason) => {
        self.notice(reason);
        return;
      }
    };
    // Whether the live settings model carries any `$$` values: with none, every config field defaults, so the
    // estimate is grounded in the firmware's default machine model rather than this board's real config. We flag
    // that so the UI qualifies the ETA rather than presenting a defaulted figure as authoritative.
    let settings = &self.view.settings;
    self.simulated_default_settings = settings.is_empty();
    // Build the planner/motion configs from the snapshot, reading each `$<n>` as an `f64` and letting absent or
    // unparseable values fall back to the firmware default inside `configs_from_settings`.
    let (planner, motion) =
      crate::eta::configs_from_settings(|n| settings.value_of(n).and_then(|s| s.trim().parse::<f64>().ok()));
    let timeline = crate::eta::EtaTimeline::build(&program, &planner, &motion);
    let total = timeline.total_seconds;
    let pauses = timeline.pauses.len();
    self.simulated = Some(timeline);
    // Echo a one-line summary so the operator has a record of the simulated total (and any unbounded pauses),
    // using the same `m:ss`/`h:mm:ss` grammar the dock clock shows.
    let clock = crate::app::progress::format_mmss(Some(std::time::Duration::from_secs_f64(total.max(0.0))));
    let qualifier = if self.simulated_default_settings { " (default settings)" } else { "" };
    let pause_note = if pauses > 0 { format!(", {pauses} operator pause(s)") } else { String::new() };
    self.notice(format!("simulated job time ~{clock}{qualifier}{pause_note}"));
  }

  /// Drop any stored simulation and its default-settings flag. Called when a new program is opened, so a stale
  /// estimate from the previous file never drives the dock ETA against the freshly loaded lines.
  pub(crate) fn clear_simulation(&mut self) {
    self.simulated = None;
    self.simulated_default_settings = false;
  }

  /// The toolbar Run/Resume segment: resume from a feed hold with a cycle-start, else start streaming the
  /// loaded program. Mirrors the [`TransportGroup`](crate::app::badge::TransportGroup) decision the view rendered.
  pub(crate) fn run_or_resume(&mut self) {
    use crate::app::badge::{BadgeState, TransportGroup};
    let group = TransportGroup::for_state(self.view.badge_state(), !self.ui.program.is_empty());
    if group.run_is_resume {
      // Held/door-suspended: a cycle-start resumes motion without re-sending the program.
      let _ = self.send_command(Command::Realtime(crate::protocol::RealtimeCommand::CycleStart));
    } else if matches!(self.view.badge_state(), BadgeState::Idle | BadgeState::Check | BadgeState::Sleep) {
      self.start_stream();
    }
  }

  /// Send one manual line, echoing it to the console as sent traffic. Returns whether it was actually sent (an
  /// empty line, or a missing/dead engine, yields `false`) so sequencing callers can stop on a failed send.
  pub(crate) fn send_line(&mut self, line: String) -> bool {
    let trimmed = line.trim().to_string();
    if trimmed.is_empty() {
      return false;
    }
    self.view.note_sent(trimmed.clone());
    self.send_command(Command::SendLine(trimmed))
  }

  /// Forward a command to the engine if connected; surface a notice if not. Returns whether it was sent.
  pub(crate) fn send_command(&mut self, command: Command) -> bool {
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
  pub(crate) fn notice(&mut self, text: String) {
    self.view.note(text);
  }

  /// The current stream's elapsed/ETA estimate. When a simulation exists it is the authoritative source — the
  /// physics-based total shows upfront (before any stream) and a physical remaining drains during one; otherwise
  /// the legacy acked-rate projection stands, exactly as before. The wall-clock elapsed comes from
  /// [`Self::stream_started`]; all the projection math lives in the pure [`crate::app::progress`].
  ///
  /// With a simulation, `completed_lines` prefers the firmware-reported current line (`Ln:`) when present, else
  /// the host's acked-line count — both index the source-line-indexed timeline directly. The live feed/rapid
  /// override fractions come from the `Ov:` percentages (defaulting to 100 % when absent), so a slowed-down run
  /// stretches the remaining estimate the way the machine actually will.
  pub(crate) fn stream_time(&self) -> crate::app::progress::TimeEstimate {
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
      return crate::app::progress::physics_estimate(elapsed, timeline.total_seconds, remaining);
    }
    // No simulation: keep the acked-rate behaviour exactly — only project once a stream is timing.
    match self.stream_started {
      Some(_) => {
        let progress = self.view.progress;
        crate::app::progress::estimate(elapsed, progress.acked, progress.total)
      }
      None => crate::app::progress::TimeEstimate::default(),
    }
  }

  /// The dock-ETA qualifier the view renders beside the clock: whether the active simulation fell back to default
  /// machine settings, and how many unbounded operator pauses it modeled. `None` when no simulation is stored, so
  /// the dock shows no qualifier and the legacy acked-rate clock stands alone.
  pub(crate) fn eta_qualifier(&self) -> Option<crate::app::views::EtaQualifier> {
    self.simulated.as_ref().map(|timeline| crate::app::views::EtaQualifier {
      default_settings: self.simulated_default_settings,
      pauses: timeline.pauses.len(),
    })
  }
}
