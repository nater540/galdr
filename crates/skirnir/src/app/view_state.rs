//! The pure view-state reducer: the bridge between engine [`Event`]s and what the egui layer draws.
//!
//! egui is immediate-mode — every frame the UI is a pure function of state — so all the decisions live here,
//! in a plain reducer with no egui types, and the view code just renders [`ViewState`] and forwards intents.
//! That keeps the hard logic (deriving the missing position from WCO, capping the console, latching the
//! alarm/error banner, tracking progress) unit-testable without a window. The reducer never performs I/O; it
//! folds one [`Event`] into the state and is driven from the UI's per-frame event drain.

use std::collections::VecDeque;

use super::badge::BadgeState;
use super::settings_model::SettingsModel;
use crate::engine::Event;
use crate::error::TransportError;
use crate::protocol::{
  CodeBook, ConnectionState, PinState, PositionKind, Response, SettingValue, StatusReport, parse_alarm_code_meta,
  parse_error_code_meta, parse_gc_body, parse_setting_meta, parse_status,
};

/// How many console lines to retain. The console is a diagnostic tail, not a transcript — capping it bounds
/// memory under a long stream and keeps per-frame rendering cheap (egui draws only visible rows anyway).
pub const CONSOLE_CAPACITY: usize = 2000;

/// The origin of a console line, so the UI can colour/tag sent vs received vs notice traffic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogSource {
  /// A line skirnir sent to the firmware (echoed locally for the operator).
  Sent,
  /// A line received from the firmware (`ok`, `error`, status, message, banner, ...).
  Received,
  /// A skirnir-generated notice (connection lifecycle, faults, local errors).
  Notice,
}

/// One entry in the rolling console buffer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LogLine {
  /// Where the line came from.
  pub source: LogSource,
  /// The text, without a trailing newline.
  pub text: String,
}

/// A latched, dismissable banner for an alarm or a stream error. The UI shows it prominently until the
/// operator clears it (or a state change supersedes it).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Banner {
  /// `ALARM:N` — the controller is in an alarm state and refuses G-code until cleared.
  Alarm(u32),
  /// `error:N` halted the stream; grblHAL holds subsequent lines until reset / empty line / `$`.
  StreamError(u32),
}

/// What a tracked probe operation was for, so the UI can label the result and the shell can route the
/// follow-up action (e.g. zeroing after a Z touch-off). The latch mechanics are kind-agnostic — the kind only
/// tells the shell which pending follow-up owns the resolved result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProbeKind {
  /// A `G38.x` Z touch-off whose successful result drives a `G10 L20` work-Z zeroing.
  ZeroZ,
  /// One touch of the rotary center-finder wizard (DOC-11 §1.2). The wizard ([`super::rotary_center`]) owns the
  /// follow-up: it folds the resolved result into its state machine and advances or aborts.
  RotaryCenter,
  /// One touch of the 180°-flip center-verify wizard (DOC-11 §2.1). The shared angle-sweep engine
  /// ([`super::angle_sweep`]) collects the reading; the [`super::flip_verify`] computation runs on completion.
  FlipVerify,
  /// One touch of the runout report (DOC-11 §2.2). The same angle-sweep engine collects the N radial readings;
  /// the read-only [`super::runout`] computation (TIR / eccentricity) runs on completion.
  Runout,
}

/// The resolved outcome of a probe operation: either a typed `[PRB:]` reading, or a failure with a reason. A
/// failure carries no position — a non-contact `G38.2` alarms (or, for the silent `G38.3`, reports `:0`), and
/// in neither case is the reported position a trustworthy trigger point to act on.
#[derive(Debug, Clone, PartialEq)]
pub enum ProbeOutcome {
  /// The probe contacted within travel: `success:1`. `position` is the machine-coordinate trigger point.
  Success { position: Vec<f64> },
  /// The probe failed — no contact (`success:0`), or an intervening `ALARM`/`error` resolved the op. The reason
  /// is a short operator-facing string; the destructive follow-up (zeroing) is suppressed on this path.
  Failure { reason: String },
}

impl ProbeOutcome {
  /// Whether the probe contacted successfully — the guard the shell checks before any destructive follow-up.
  pub fn is_success(&self) -> bool {
    matches!(self, ProbeOutcome::Success { .. })
  }
}

/// The latch that correlates a probe the app issued with the result that comes back asynchronously off the
/// event stream. grbl streaming has no request→response correlation — `ok` only acks the probe *line*, and the
/// `[PRB:]` result arrives as a separate push — so the only sound model is to mark an op `awaiting` and capture
/// the next [`Response::ProbeResult`] (or resolve it failed on an intervening `Alarm`/`Error`). This stays in
/// the pure reducer so the whole mechanism is unit-testable without a window or real hardware.
#[derive(Debug, Clone, PartialEq)]
pub struct ProbeOp {
  /// What the operation was for, so the UI/shell can label and route it.
  pub kind: ProbeKind,
  /// Whether a result is still outstanding. Set when the probe is issued; cleared when a result lands or an
  /// `Alarm`/`Error` resolves the op as failed.
  pub awaiting: bool,
  /// The last resolved outcome, or `None` while still awaiting the first result. The UI renders this; the shell
  /// reads it to decide the follow-up (zero on success, surface a notice on failure).
  pub last: Option<ProbeOutcome>,
}

impl ProbeOp {
  /// Begin tracking a freshly-issued probe of `kind`: awaiting a result, no outcome yet.
  pub fn issued(kind: ProbeKind) -> Self {
    ProbeOp { kind, awaiting: true, last: None }
  }
}

/// Streaming progress, mirrored from [`Event::Progress`]. `total == 0` means no program is loaded.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Progress {
  /// Lines released to the firmware so far.
  pub sent: usize,
  /// Lines acknowledged (`ok`/`error`) so far.
  pub acked: usize,
  /// Total lines in the loaded program.
  pub total: usize,
}

impl Progress {
  /// The fraction acknowledged in `0.0..=1.0`, or `0.0` when no program is loaded. For a progress bar.
  pub fn fraction(self) -> f32 {
    if self.total == 0 {
      0.0
    } else {
      (self.acked as f32 / self.total as f32).clamp(0.0, 1.0)
    }
  }
}

/// Everything the egui layer needs to draw one frame. The reducer owns it; the UI reads it and emits intents.
#[derive(Debug, Clone, PartialEq)]
pub struct ViewState {
  /// The authoritative connection/streaming lifecycle from the engine.
  pub connection: ConnectionState,
  /// The most recent parsed status report, if any has arrived since connect. Drives the DRO/overrides/pins.
  pub status: Option<StatusReport>,
  /// The typed input-pin set decoded once when each status report is ingested, so the endstop/probe/door UI can
  /// read it on every immediate-mode frame without re-running [`PinState::from_letters`] over the raw letters. An
  /// absent or empty `Pn:` (the firmware omits the field when nothing is asserted) leaves this at its all-clear
  /// default, so the renderer can read it unconditionally.
  pub pins: PinState,
  /// The last-seen `WCO:` vector, cached across reports (grbl pushes it only intermittently) so we can always
  /// derive the position kind the current report did not carry.
  pub last_wco: Vec<f64>,
  /// The last-seen `Ov:` override triple `(feed, rapid, spindle)` in percent, cached across reports. Like `WCO:`,
  /// grblHAL emits `Ov:` only intermittently (on change / periodic refresh, not every report — see
  /// docs/gcode-streaming.md §status, `{|Ov:f,r,s}`), so a report that omits it carries the LAST value, not a
  /// reset to 100%. Reading the per-report `status.overrides` directly snapped the sliders/steppers back to
  /// centre on every Ov-less poll; consumers read [`Self::overrides`] instead. `None` until the first `Ov:`.
  last_overrides: Option<(u32, u32, u32)>,
  /// The active tool number, sourced from the `T<n>` word of the `$G` / `[GC:]` parser-state line — the
  /// authoritative "what tool is loaded" per the streaming contract (the `<...>` status report carries no tool
  /// number). `Some(0)` means no tool; `None` means none has been reported yet this session. Cleared on
  /// disconnect so a reconnect to a (possibly different) board never shows a stale tool.
  pub current_tool: Option<u32>,
  /// Streaming progress.
  pub progress: Progress,
  /// A latched alarm/error banner, if active.
  pub banner: Option<Banner>,
  /// The current probe operation latch, if one is in flight or its result is still being shown. `None` when no
  /// probe has been issued this session. Populated by the shell on a probe issue (via [`Self::begin_probe`]) and
  /// resolved here when a [`Response::ProbeResult`] lands or an intervening `Alarm`/`Error` fails the op.
  pub probe_op: Option<ProbeOp>,
  /// The code of the most recent `error:N`, stashed so the stream-error banner can carry it when the lifecycle
  /// transition into [`ConnectionState::Error`] arrives (the transition event itself carries only the state).
  /// Not part of the rendered view — purely a one-event bridge from the error response to the halt transition.
  last_error_code: Option<u32>,
  /// The rolling console buffer, oldest first, capped at [`CONSOLE_CAPACITY`].
  pub console: VecDeque<LogLine>,
  /// The live firmware settings, merged from `$<n>=<value>` values and `$ES` enumeration metadata. Populated
  /// when the operator requests a `$$` dump (and `$ES`); cleared on disconnect so a reconnect starts clean.
  pub settings: SettingsModel,
  /// The runtime error/alarm code decodings enumerated from the firmware (`$EE`/`$EA`), overlaying the static
  /// fallback tables. Used to render `error:N`/`ALARM:N` with a human name in the console and to enrich the
  /// banner detail. Per-session: cleared on disconnect so a reconnect re-learns the board's set. The static
  /// fallback means decoding works even before (or without) any enrichment.
  pub codes: CodeBook,
}

impl Default for ViewState {
  fn default() -> Self {
    ViewState {
      connection: ConnectionState::Disconnected,
      status: None,
      pins: PinState::default(),
      last_wco: Vec::new(),
      last_overrides: None,
      current_tool: None,
      progress: Progress::default(),
      banner: None,
      probe_op: None,
      last_error_code: None,
      console: VecDeque::new(),
      settings: SettingsModel::new(),
      codes: CodeBook::new(),
    }
  }
}

impl ViewState {
  /// Fold one engine [`Event`] into the state. Called once per drained event in the UI's frame loop; pure,
  /// allocation-light, and never blocking.
  pub fn apply(&mut self, event: Event) {
    match event {
      Event::StateChanged(state) => self.on_state_changed(state),
      Event::Response(response) => self.on_response(response),
      Event::Progress { sent, acked, total } => self.progress = Progress { sent, acked, total },
      Event::Fault(err) => self.log(LogSource::Notice, format!("fault: {err}")),
      Event::Disconnected(reason) => self.on_disconnected(reason),
    }
  }

  /// Record that the UI sent a manual line, so it appears in the console even though the engine does not echo
  /// host writes back as events. Called by the UI right after it forwards a `SendLine`/`StreamProgram` intent.
  pub fn note_sent(&mut self, line: impl Into<String>) {
    self.log(LogSource::Sent, line.into());
  }

  /// Record a skirnir-generated notice (connection lifecycle, a local error, a sequenced action). Called by
  /// the shell for side effects the engine does not surface as events.
  pub fn note(&mut self, text: impl Into<String>) {
    self.log(LogSource::Notice, text.into());
  }

  /// Clear a latched banner (operator dismissed it, or issued the clearing command).
  pub fn dismiss_banner(&mut self) {
    self.banner = None;
  }

  /// Begin tracking a freshly-issued probe of `kind`: arm the latch to await its result. Called by the shell the
  /// moment it sends the probe line, so the next [`Response::ProbeResult`] (or an intervening `Alarm`/`Error`)
  /// resolves *this* op. Replaces any prior op — a new probe supersedes a stale, already-resolved one.
  pub fn begin_probe(&mut self, kind: ProbeKind) {
    self.probe_op = Some(ProbeOp::issued(kind));
  }

  /// Whether a probe is currently awaiting its result, so the shell can run its push-or-poll timeout only while
  /// one is genuinely outstanding.
  pub fn probe_is_awaiting(&self) -> bool {
    self.probe_op.as_ref().is_some_and(|op| op.awaiting)
  }

  /// Resolve the in-flight probe op as failed for an external reason (the shell's push-or-poll timeout giving
  /// up, or a disconnect). No-op when nothing is awaiting, so a late call cannot clobber a result already
  /// captured. Kept here (not just in the shell) so the failure path is reduced uniformly with the event-driven
  /// `Alarm`/`Error` failures.
  pub fn fail_probe(&mut self, reason: impl Into<String>) {
    if let Some(op) = self.probe_op.as_mut()
      && op.awaiting
    {
      op.awaiting = false;
      op.last = Some(ProbeOutcome::Failure { reason: reason.into() });
    }
  }

  /// Clear the probe-op latch entirely, so a resolved result is consumed exactly once. The rotary wizard folds
  /// the result into its own state machine and then calls this, so the same `[PRB:]` is not re-fed on the next
  /// frame — and the next touch's [`Self::begin_probe`] re-arms a fresh op.
  pub fn clear_probe_op(&mut self) {
    self.probe_op = None;
  }

  /// The semantic badge state for the toolbar/status badge: the host lifecycle reconciled with the firmware's
  /// last-reported run state, so Jog/Home/Door/Check render distinctly while a latched alarm/error still wins.
  pub fn badge_state(&self) -> BadgeState {
    let run_state = self.status.as_ref().map(|s| s.machine_state.state);
    BadgeState::derive(self.connection, run_state)
  }

  /// The DRO positions to display as `(machine, work)`, each `Some(vec)` when derivable. A report carries only
  /// one kind; we synthesize the other from the cached WCO via `WPos = MPos − WCO`. When no WCO is known yet,
  /// only the reported kind is returned.
  pub fn dro(&self) -> (Option<Vec<f64>>, Option<Vec<f64>>) {
    let Some(status) = &self.status else {
      return (None, None);
    };
    let reported = status.position.clone();
    let other = self.derive_other_position(status);
    match status.position_kind {
      PositionKind::Machine => (Some(reported), other),
      PositionKind::Work => (other, Some(reported)),
    }
  }

  /// The number of axes the firmware's latest status report carries: 3 for a plain XYZ board, 4 when a rotary A is
  /// present (DOC-10). Read from the reported position vector; defaults to 3 before any report. The DRO's A row and
  /// the jog pad's A column BOTH gate on this so a 3-axis board never shows — nor tries to jog — a phantom rotary
  /// axis: a `$J=...A...` sent to a 3-axis firmware is rejected with `error:N`, which then holds the stream in the
  /// error state (the finding this fixes). Equivalent to the length the DRO derives from [`Self::dro`], without its
  /// allocation.
  pub fn reported_axis_count(&self) -> usize {
    self.status.as_ref().map(|s| s.position.len()).unwrap_or(3)
  }

  /// Whether the firmware reports a rotary A axis (a 4-field position report). The single gate the DRO A row and the
  /// jog A column share; see [`Self::reported_axis_count`].
  pub fn has_rotary_axis(&self) -> bool {
    self.reported_axis_count() >= 4
  }

  /// The live work-coordinate `(x, y)` for the toolpath overlay, with NO per-frame allocation. [`Self::dro`]
  /// clones `status.position` and builds a second derived `Vec` every call; the overlay runs at up to 20 Hz and
  /// reads only the work XY, so it takes this allocation-free path instead (finding #10). When the report already
  /// carries the work position we read X/Y straight from it; when it carries the machine position we derive only
  /// the two work components we need (`WPos = MPos − WCO`) rather than a whole vector. `None` until a status with
  /// at least an X and Y axis (and, for a machine report, a usable cached WCO) is available.
  pub fn work_xy(&self) -> Option<(f64, f64)> {
    let status = self.status.as_ref()?;
    if status.position.len() < 2 {
      return None;
    }
    match status.position_kind {
      // The report is already in work coordinates: read X/Y directly, no offset, no allocation.
      PositionKind::Work => Some((status.position[0], status.position[1])),
      // The report is in machine coordinates: derive only the two work components from the cached WCO. A WCO of a
      // different length than the position is a malformed mix; refuse it rather than mis-pairing axes.
      PositionKind::Machine => {
        if self.last_wco.len() != status.position.len() {
          return None;
        }
        Some((status.position[0] - self.last_wco[0], status.position[1] - self.last_wco[1]))
      }
    }
  }

  /// The live work-coordinate Z (millimetres) for the toolpath overlay's depth colouring, derived exactly as
  /// [`Self::work_xy`] derives XY but for the third axis — and with the same no-allocation discipline (the overlay
  /// runs at up to 20 Hz). A work report reads Z straight from index 2; a machine report derives the single Z
  /// component from the cached WCO (`WPos = MPos − WCO`). `None` until a status with at least three axes (and, for a
  /// machine report, a usable cached WCO) is available — so an XY-only report yields no depth and draws no cut.
  pub fn work_z(&self) -> Option<f64> {
    let status = self.status.as_ref()?;
    if status.position.len() < 3 {
      return None;
    }
    match status.position_kind {
      // The report is already in work coordinates: read Z directly, no offset, no allocation.
      PositionKind::Work => Some(status.position[2]),
      // The report is in machine coordinates: derive only the Z work component from the cached WCO. A WCO of a
      // different length than the position is a malformed mix; refuse it rather than mis-pairing axes.
      PositionKind::Machine => {
        if self.last_wco.len() != status.position.len() {
          return None;
        }
        Some(status.position[2] - self.last_wco[2])
      }
    }
  }

  /// The live override triple `(feed, rapid, spindle)` in percent for the override panel and the shell's
  /// relative-stepping base. Reads the *cached* `Ov:` value, not the per-report `status.overrides`: grblHAL
  /// reports `Ov:` only intermittently (on change / periodic refresh), so a report that omits it carries the
  /// last value, and reading the per-report field directly would snap the override to 100% on every Ov-less
  /// poll (the slider/stepper snap-back-to-centre bug). Falls back to the neutral 100% triple until the first
  /// `Ov:` arrives, so the panel always has a sensible value to render before any override is reported.
  pub fn overrides(&self) -> (u32, u32, u32) {
    let neutral = super::overrides::OVERRIDE_NEUTRAL;
    self.last_overrides.unwrap_or((neutral, neutral, neutral))
  }

  /// Derive the position kind the current report omitted, from the cached WCO. `MPos` and `WPos` relate by
  /// `WPos = MPos − WCO`. Returns `None` if no WCO is known or the lengths disagree (a malformed mix).
  fn derive_other_position(&self, status: &StatusReport) -> Option<Vec<f64>> {
    if self.last_wco.is_empty() || self.last_wco.len() != status.position.len() {
      return None;
    }
    let derived = status
      .position
      .iter()
      .zip(&self.last_wco)
      .map(|(p, wco)| match status.position_kind {
        // Reported machine → derive work by subtracting the offset.
        PositionKind::Machine => p - wco,
        // Reported work → derive machine by adding the offset back.
        PositionKind::Work => p + wco,
      })
      .collect();
    Some(derived)
  }

  /// React to a lifecycle transition. The stream-error banner is bound to the `Error` lifecycle state: it
  /// latches only when the stream actually halts (a program `error:N` drove the core into
  /// [`ConnectionState::Error`]) and clears the moment the operator recovers to a running state. A bare
  /// `error:N` from a manual command (e.g. `$H` rejected with `error:5`) never enters `Error`, so it surfaces
  /// in the console without raising a misleading "stream halted" strip — and a soft reset, which returns the
  /// halted lifecycle to `Idle`, drops the banner here rather than relying on a no-op `Idle`→`Idle` transition.
  fn on_state_changed(&mut self, state: ConnectionState) {
    self.connection = state;
    match state {
      // The stream halted: latch the banner with the code stashed from the preceding `error:N` response.
      ConnectionState::Error => self.banner = Some(Banner::StreamError(self.last_error_code.unwrap_or(0))),
      // A return to Idle/Streaming means the operator recovered; drop a stale stream-error banner. An alarm
      // banner persists until an explicit clear, since the firmware still reports Alarm until `$X`/`$H`.
      ConnectionState::Idle | ConnectionState::Streaming => {
        if matches!(self.banner, Some(Banner::StreamError(_))) {
          self.banner = None;
        }
      }
      // Disconnecting resets progress so a fresh connection does not show a stale bar.
      ConnectionState::Disconnected => self.progress = Progress::default(),
      _ => {}
    }
    self.log(LogSource::Notice, format!("state: {state:?}"));
  }

  /// React to a parsed firmware response: refresh the DRO from status reports, latch banners on alarm/error,
  /// and append the line to the console.
  fn on_response(&mut self, response: Response) {
    match &response {
      Response::Status(body) => {
        let report = parse_status(body);
        // Cache any fresh WCO so later reports of the other kind stay derivable.
        if let Some(wco) = &report.wco {
          self.last_wco = wco.clone();
        }
        // Cache any fresh `Ov:` for the same reason WCO is cached: it is intermittent, so an Ov-less report must
        // preserve the last-known override rather than letting consumers read it as a reset to 100% (the slider
        // snap-back). Only a report that actually carried `Ov:` updates the cache.
        if let Some(ov) = report.overrides {
          self.last_overrides = Some(ov);
        }
        // Decode the input pins once here rather than on every render frame: the ~21-arm letter match runs at the
        // status-report rate (a few Hz) instead of the egui repaint rate (continuous), and the endstop chips just
        // read the cached typed value.
        self.pins = report.pin_state();
        self.status = Some(report);
        // Status reports are high-frequency telemetry; echoing each to the console would drown it. Skip them.
        return;
      }
      Response::ProbeResult { position, success } => {
        // Capture the typed result into the latch: a `success:1` is the trustworthy trigger point the shell can
        // act on; a `success:0` (the silent `G38.3`/`G38.5` non-contact path) resolves the op as a failure so no
        // destructive follow-up runs. The reading still reaches the console below for the operator's record.
        self.resolve_probe(if *success {
          ProbeOutcome::Success { position: position.clone() }
        } else {
          ProbeOutcome::Failure { reason: "probe did not contact (flag :0)".to_string() }
        });
      }
      // An alarm latches the banner AND fails any awaiting probe: a no-contact `G38.2`/`G38.4` raises `ALARM:5`,
      // so a probe op outstanding when the alarm lands resolved as a non-contact failure — the shell must not
      // then run the zeroing line (which the alarm would `error:9`-lock anyway, but the latch makes it explicit
      // rather than relying on that race).
      Response::Alarm(code) => {
        self.banner = Some(Banner::Alarm(*code));
        self.fail_probe(format!("ALARM:{code} during probe"));
      }
      // Stash the code but do not latch the banner here: a `error:N` only halts the stream when it belongs to a
      // program line, which the core signals with a transition to `Error`. `on_state_changed` latches the
      // banner on that transition, so a manual-command rejection (which never enters `Error`) only logs below.
      // An `error:N` arriving while a probe is awaiting also fails the op (e.g. a rejected probe line, or the
      // `error:9` g-code lock after an alarm) so the shell never zeroes off a probe the firmware refused.
      Response::Error(code) => {
        self.last_error_code = Some(*code);
        self.fail_probe(format!("error:{code} during probe"));
      }
      Response::Setting { number, value } => {
        // A `$<n>=<value>` line merges into the live settings model. Like status telemetry, a `$$` dump is many
        // lines of structured data the settings panel renders, so it does not flood the console.
        self.settings.apply_value(SettingValue { number: *number, value: value.clone() });
        return;
      }
      Response::Message(body) => {
        // A `[GC:...]` parser-state line (the `$G` answer) carries the active tool (`T<n>`) — the single
        // authoritative tool source — among the modal G/M words. Fold the tool into the DRO when present. We
        // suppress the console echo ONLY when a tool was successfully extracted (the auto-reconcile `$G` the shell
        // fires on connect / on entering Tool would otherwise be console noise). A `[GC:]` that yields no tool —
        // a hand-typed `$G` we model no field of, or a malformed/garbled line — must NOT be swallowed: it falls
        // through to the console below so it stays visible-as-text rather than vanishing silently.
        if let Some(parser_state) = parse_gc_body(body)
          && let Some(tool) = parser_state.tool
        {
          self.current_tool = Some(tool);
          return;
        }
        // A `[SETTING:...]` enumeration row enriches the settings model with the setting's label/unit/bounds and
        // is not console noise; every other bracketed message still reaches the console below.
        if let Some(meta) = parse_setting_meta(body) {
          self.settings.apply_meta(meta);
          return;
        }
        // A `[ERRORCODE:...]`/`[ALARMCODE:...]` enumeration row (the firmware's `$EE`/`$EA` answer) enriches the
        // codebook and, like the settings dump, is structured data the UI consults — not console noise. Fold it
        // and skip the console so the enumeration does not flood it.
        if let Some((code, text)) = parse_error_code_meta(body) {
          self.codes.apply_error(code, text);
          return;
        }
        if let Some((code, text)) = parse_alarm_code_meta(body) {
          self.codes.apply_alarm(code, text);
          return;
        }
        // The firmware pushes a context line right before each error/alarm: `[MSG:error:<n> <name>]` /
        // `[MSG:ALARM:<n> <name>]`. skirnir now decodes the code itself (`error:21 — Modal group violation`), so
        // that annotation is a redundant duplicate in the console — suppress it. grbl's own `[MSG:...]` pushes
        // never start with `error:`/`ALARM:`, so the prefix-plus-numeric-code match is safe and won't swallow
        // legitimate context like `[MSG:Pgm End]`.
        if is_redundant_error_annotation(body) {
          return;
        }
      }
      _ => {}
    }
    let text = self.render_response(&response);
    self.log(LogSource::Received, text);
  }

  /// Capture a resolved outcome into the in-flight probe op, clearing `awaiting`. Only resolves an op that is
  /// still awaiting, so a stray second `[PRB:]` (e.g. a push followed by a redundant `$#` echo) cannot overwrite
  /// a result the shell may have already acted on. No-op when no probe is outstanding.
  fn resolve_probe(&mut self, outcome: ProbeOutcome) {
    if let Some(op) = self.probe_op.as_mut()
      && op.awaiting
    {
      op.awaiting = false;
      op.last = Some(outcome);
    }
  }

  /// React to the terminal disconnect event.
  fn on_disconnected(&mut self, reason: Option<TransportError>) {
    self.connection = ConnectionState::Disconnected;
    self.progress = Progress::default();
    self.status = None;
    // A probe op belongs to the session that just ended: drop it so a reconnect never resumes awaiting a result
    // from the dead link or shows the previous board's reading. If one was awaiting, the shell's per-frame poll
    // sees the cleared latch and abandons its follow-up.
    self.probe_op = None;
    // Drop the cached pin state with the report it came from, so a reconnect does not show the old board's
    // endstops asserted before its first status report arrives.
    self.pins = PinState::default();
    // Drop the cached WCO so a reconnect does not show "WCO set" or derive WPos/MPos from a stale offset before
    // the new session reports its own. Report-derived state must not survive across a disconnect.
    self.last_wco.clear();
    // Drop the cached override for the same reason: it is the previous board's, and a reconnect must start from
    // neutral rather than carrying a stale feed/rapid/spindle override into a session that has not reported one.
    self.last_overrides = None;
    // The active tool is the previous board's parser state; clear it so a reconnect shows no tool until the new
    // session's `$G` answers, rather than carrying a stale `T<n>` across the disconnect.
    self.current_tool = None;
    // The settings list is the previous board's; clear it so a reconnect re-fetches rather than showing stale.
    self.settings.clear();
    // The codebook overrides are the previous board's `$EE`/`$EA` enumeration; clear them so a reconnect
    // re-learns. Lookups still work via the static fallback in the meantime.
    self.codes.clear();
    match reason {
      Some(err) => self.log(LogSource::Notice, format!("disconnected: {err}")),
      None => self.log(LogSource::Notice, "disconnected".to_string()),
    }
  }

  /// Empty the console buffer. Drives the right-click "Clear" affordance — a pure state mutation the shell calls
  /// in response to [`crate::app::Intent::ClearConsole`], so the view stays a renderer and the wipe is testable.
  pub fn clear_console(&mut self) {
    self.console.clear();
  }

  /// Append a console line, evicting the oldest once at capacity so the buffer never grows without bound.
  fn log(&mut self, source: LogSource, text: String) {
    if self.console.len() >= CONSOLE_CAPACITY {
      self.console.pop_front();
    }
    self.console.push_back(LogLine { source, text });
  }

  /// Render a non-status response as a one-line console string. Status reports are handled separately and never
  /// reach here. An `error:N`/`ALARM:N` is decoded through the [`CodeBook`] so the operator sees a name beside
  /// the number (`error:21 — Modal group violation`) — the short name only; the full description lives in the
  /// banner detail / hover. Every other variant is rendered verbatim as before.
  fn render_response(&self, response: &Response) -> String {
    match response {
      Response::Ok => "ok".to_string(),
      Response::Error(code) => format!("error:{code} — {}", self.codes.error_name(*code)),
      Response::Alarm(code) => format!("ALARM:{code} — {}", self.codes.alarm_name(*code)),
      Response::Message(body) => format!("[{body}]"),
      // Reconstruct the `[PRB:…]` line for the console so the operator's record is unchanged from the untyped
      // days, while the latch consumes the typed value separately. Values render at 3 decimals, the firmware's
      // PRB precision.
      Response::ProbeResult { position, success } => {
        let coords = position.iter().map(|v| format!("{v:.3}")).collect::<Vec<_>>().join(",");
        format!("[PRB:{coords}:{}]", if *success { 1 } else { 0 })
      }
      Response::Banner(text) => text.clone(),
      Response::StartupEcho(text) => format!(">{text}"),
      Response::Unknown(text) => text.clone(),
      // Status is rendered by the DRO and settings by the panel, not the console; included for exhaustiveness.
      Response::Status(body) => format!("<{body}>"),
      Response::Setting { number, value } => format!("${number}={value}"),
    }
  }
}

/// The firmware's pre-error context-push prefix (after bracket-strip). These MUST track the firmware's
/// `ResponseWriter::error_context` / `alarm_context` format — `[MSG:error:<N> <name>]` and
/// `[MSG:ALARM:<N> <name>]` — emitted immediately before each `error:N` / `ALARM:N`. There is no shared type
/// binding host and firmware here, so if that emitter's surface text changes, these two consts (and the shape
/// check below) are the coupling point to update.
const FW_ERROR_ANNOTATION_PREFIX: &str = "MSG:error:";
const FW_ALARM_ANNOTATION_PREFIX: &str = "MSG:ALARM:";

/// Whether a `[MSG:...]` body (already stripped of brackets) is the firmware's redundant pre-error/alarm
/// context push — exactly `MSG:error:<digits> <name>` or `MSG:ALARM:<digits> <name>`. skirnir decodes the
/// following `error:N`/`ALARM:N` itself, so this annotation duplicates the console line and is dropped.
///
/// The shape is matched tightly to shrink the false-positive surface: the prefix, then one-or-more ASCII
/// digits, then a single space, then at least one more character (the name). That rejects near-misses like
/// `MSG:error:` (no code), `MSG:error:21` (no trailing name), and `MSG:errored sensor` (no digit after the
/// colon), and a generic `[MSG:...]` push (which never starts with `error:`/`ALARM:`) is never swallowed.
fn is_redundant_error_annotation(body: &str) -> bool {
  let Some(tail) = body
    .strip_prefix(FW_ERROR_ANNOTATION_PREFIX)
    .or_else(|| body.strip_prefix(FW_ALARM_ANNOTATION_PREFIX))
  else {
    return false;
  };
  // Consume one-or-more leading ASCII digits (the code). No digit ⇒ not the annotation (`MSG:errored ...`).
  let digits = tail.trim_start_matches(|c: char| c.is_ascii_digit());
  if digits.len() == tail.len() {
    return false;
  }
  // After the code there must be a single space, then a non-empty name. This rejects `MSG:error:21` (no name)
  // and anything where the code is not followed by a space-delimited name.
  matches!(digits.strip_prefix(' '), Some(name) if !name.is_empty())
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::protocol::RunState;

  /// Convenience: feed a status-report body through the reducer as the engine would.
  fn feed_status(view: &mut ViewState, body: &str) {
    view.apply(Event::Response(Response::Status(body.to_string())));
  }

  /// Convenience: feed a bracketed `[...]` message body through the reducer as the engine would.
  fn feed_message(view: &mut ViewState, body: &str) {
    view.apply(Event::Response(Response::Message(body.to_string())));
  }

  #[test]
  fn reported_axis_count_gates_the_rotary_controls_on_the_report_width() {
    // The DRO A row and the jog A column share this single signal. No report yet reads as a 3-axis board (the safe
    // default); a 3-field report stays 3-axis; only a 4-field (rotary) report flips `has_rotary_axis`, so a plain
    // board never offers an A jog (which the firmware would reject with `error:N` and wedge the stream).
    let mut view = ViewState::default();
    assert_eq!(view.reported_axis_count(), 3, "no report yet defaults to 3 axes");
    assert!(!view.has_rotary_axis(), "no report is not a rotary board");

    feed_status(&mut view, "Idle|MPos:1.000,2.000,3.000");
    assert_eq!(view.reported_axis_count(), 3, "a 3-field report is a 3-axis board");
    assert!(!view.has_rotary_axis(), "a 3-field report must not read as rotary");

    feed_status(&mut view, "Idle|MPos:1.000,2.000,3.000,45.000");
    assert_eq!(view.reported_axis_count(), 4, "a 4-field report is a rotary board");
    assert!(view.has_rotary_axis(), "a 4-field report reads as rotary");
  }

  #[test]
  fn a_gc_parser_state_line_sets_the_active_tool_without_console_noise() {
    let mut view = ViewState::default();
    feed_message(&mut view, "GC:G0 G54 G17 G21 G90 G94 M5 M9 T3 F0 S0");
    assert_eq!(view.current_tool, Some(3), "the `T<n>` word sources the active tool");
    assert!(view.console.is_empty(), "a `[GC:]` parser-state line is folded, not echoed to the console");
  }

  #[test]
  fn a_gc_line_with_no_tool_word_leaves_a_known_tool_untouched() {
    let mut view = ViewState::default();
    feed_message(&mut view, "GC:T2 F0 S0");
    assert_eq!(view.current_tool, Some(2));
    // A later parser-state line that happens to carry no `T` word must not clobber the known tool with `None`.
    feed_message(&mut view, "GC:G0 G54 F0 S0");
    assert_eq!(view.current_tool, Some(2), "an absent `T` word must not wipe a good tool");
  }

  #[test]
  fn a_gc_line_without_an_extractable_tool_is_echoed_not_swallowed() {
    // A `[GC:]` that yields no tool — a hand-typed `$G` we model no field of, or a malformed/garbled line — must
    // still reach the console so it is visible-as-text rather than vanishing silently. We suppress the echo ONLY
    // when a tool was successfully extracted (the auto-reconcile `$G` the shell fires would otherwise be noise).
    let mut view = ViewState::default();
    feed_message(&mut view, "GC:G0 G54 F0 S0");
    assert_eq!(view.console.len(), 1, "a `[GC:]` with no extractable tool must be echoed, not swallowed");
    assert!(view.console.back().expect("a console line").text.contains("GC:"), "the verbatim line reaches the console");
    // And a `[GC:]` that DID set the tool stays suppressed (no console noise from the auto-reconcile `$G`).
    feed_message(&mut view, "GC:G0 G54 T7 F0 S0");
    assert_eq!(view.current_tool, Some(7));
    assert_eq!(view.console.len(), 1, "a parseable `[GC:]` is folded, not echoed");
  }

  #[test]
  fn the_active_tool_is_cleared_on_disconnect() {
    let mut view = ViewState::default();
    feed_message(&mut view, "GC:T4 F0 S0");
    assert_eq!(view.current_tool, Some(4));
    view.apply(Event::Disconnected(None));
    assert_eq!(view.current_tool, None, "the previous board's tool must not survive a disconnect");
  }

  #[test]
  fn starts_disconnected_and_empty() {
    let view = ViewState::default();
    assert_eq!(view.connection, ConnectionState::Disconnected);
    assert!(view.status.is_none());
    assert!(view.console.is_empty());
    assert_eq!(view.progress, Progress::default());
  }

  #[test]
  fn state_changes_are_tracked_and_logged() {
    let mut view = ViewState::default();
    view.apply(Event::StateChanged(ConnectionState::Idle));
    assert_eq!(view.connection, ConnectionState::Idle);
    assert_eq!(view.console.len(), 1);
    assert_eq!(view.console.back().unwrap().source, LogSource::Notice);
  }

  #[test]
  fn status_reports_update_the_dro_but_not_the_console() {
    let mut view = ViewState::default();
    feed_status(&mut view, "Idle|MPos:1.000,2.000,3.000|FS:0,0");
    assert!(view.console.is_empty(), "status telemetry must not flood the console");
    let status = view.status.as_ref().expect("a status report was applied");
    assert_eq!(status.machine_state.state, RunState::Idle);
    assert_eq!(status.position, vec![1.0, 2.0, 3.0]);
  }

  #[test]
  fn caches_wco_and_derives_work_position_from_a_later_machine_report() {
    let mut view = ViewState::default();
    // First report carries the WCO alongside machine position.
    feed_status(&mut view, "Idle|MPos:10.000,20.000,5.000|WCO:1.000,2.000,3.000");
    // A later report carries only machine position; work must still be derivable from the cached WCO.
    feed_status(&mut view, "Idle|MPos:11.000,22.000,8.000");
    let (machine, work) = view.dro();
    assert_eq!(machine, Some(vec![11.0, 22.0, 8.0]));
    assert_eq!(work, Some(vec![10.0, 20.0, 5.0]));
  }

  #[test]
  fn derives_machine_position_from_a_work_report_using_cached_wco() {
    let mut view = ViewState::default();
    feed_status(&mut view, "Idle|MPos:10.000,20.000,5.000|WCO:1.000,2.000,3.000");
    feed_status(&mut view, "Idle|WPos:0.000,0.000,0.000");
    let (machine, work) = view.dro();
    assert_eq!(work, Some(vec![0.0, 0.0, 0.0]));
    // machine = work + wco
    assert_eq!(machine, Some(vec![1.0, 2.0, 3.0]));
  }

  #[test]
  fn dro_returns_only_reported_kind_when_no_wco_is_known() {
    let mut view = ViewState::default();
    feed_status(&mut view, "Idle|MPos:1.0,2.0,3.0");
    let (machine, work) = view.dro();
    assert_eq!(machine, Some(vec![1.0, 2.0, 3.0]));
    assert_eq!(work, None);
  }

  #[test]
  fn work_xy_returns_the_work_position_without_allocating_a_derived_vec() {
    let mut view = ViewState::default();
    // No status yet: no work XY.
    assert_eq!(view.work_xy(), None);
    // A work report yields its XY directly (no offset, the allocation-free fast path of finding #10).
    feed_status(&mut view, "Run|WPos:-1.500,2.250,-0.100");
    assert_eq!(view.work_xy(), Some((-1.5, 2.25)));
    // A machine report derives work XY from the cached WCO — only the two components, never a whole Vec.
    feed_status(&mut view, "Run|MPos:10.000,20.000,5.000|WCO:1.000,2.000,3.000");
    feed_status(&mut view, "Run|MPos:11.000,22.000,8.000");
    assert_eq!(view.work_xy(), Some((10.0, 20.0))); // 11−1, 22−2
  }

  #[test]
  fn work_z_returns_the_work_z_mirroring_work_xy() {
    let mut view = ViewState::default();
    // No status yet: no work Z.
    assert_eq!(view.work_z(), None);
    // A work report yields its Z directly (index 2, no offset).
    feed_status(&mut view, "Run|WPos:-1.500,2.250,-0.100");
    assert_eq!(view.work_z(), Some(-0.100));
    // A machine report derives work Z from the cached WCO (WPos = MPos − WCO), only the one component.
    feed_status(&mut view, "Run|MPos:10.000,20.000,5.000|WCO:1.000,2.000,3.000");
    feed_status(&mut view, "Run|MPos:11.000,22.000,8.000");
    assert_eq!(view.work_z(), Some(5.0)); // 8 − 3
  }

  #[test]
  fn work_z_is_none_without_a_z_axis_or_usable_wco() {
    let mut view = ViewState::default();
    // A machine report with no cached WCO cannot derive work Z.
    feed_status(&mut view, "Run|MPos:1.0,2.0,3.0");
    assert_eq!(view.work_z(), None);
    // A two-axis (XY-only) report has no Z component to read.
    feed_status(&mut view, "Run|WPos:5.0,6.0");
    assert_eq!(view.work_z(), None);
  }

  #[test]
  fn work_xy_is_none_when_a_machine_report_has_no_usable_wco() {
    let mut view = ViewState::default();
    // A machine report with no cached WCO cannot derive work coordinates: no marker (mirrors `dro`).
    feed_status(&mut view, "Run|MPos:1.0,2.0,3.0");
    assert_eq!(view.work_xy(), None);
    // A degenerate single-axis report cannot place a planar marker either.
    feed_status(&mut view, "Run|WPos:5.0");
    assert_eq!(view.work_xy(), None);
  }

  #[test]
  fn ingesting_a_status_report_caches_the_decoded_pin_state() {
    let mut view = ViewState::default();
    // No report yet: the cached pin state reads all-clear so the endstop chips can render unconditionally.
    assert_eq!(view.pins, PinState::default());
    // A `Pn:XYZ` report must leave the typed pin state cached, decoded once, ready for per-frame reads.
    feed_status(&mut view, "Alarm|MPos:0,0,0|Pn:XYZ");
    assert!(view.pins.limit_x && view.pins.limit_y && view.pins.limit_z, "X/Y/Z limits must be cached as asserted");
    // A later report without a `Pn:` field clears the cache (nothing asserted), matching the firmware's omission.
    feed_status(&mut view, "Idle|MPos:0,0,0");
    assert_eq!(view.pins, PinState::default(), "an absent Pn: field must clear the cached pin state");
  }

  #[test]
  fn disconnect_clears_the_cached_pin_state() {
    let mut view = ViewState::default();
    feed_status(&mut view, "Alarm|MPos:0,0,0|Pn:Z");
    assert!(view.pins.limit_z);
    view.apply(Event::Disconnected(None));
    assert_eq!(view.pins, PinState::default(), "cached pin state must not survive a disconnect");
  }

  #[test]
  fn badge_state_prefers_the_firmware_run_state_when_connected() {
    let mut view = ViewState::default();
    view.apply(Event::StateChanged(ConnectionState::Idle));
    // With no report yet the badge falls back to the lifecycle.
    assert_eq!(view.badge_state(), crate::app::badge::BadgeState::Idle);
    // A `Jog` report should surface on the badge even though the host lifecycle is still Idle.
    feed_status(&mut view, "Jog|MPos:1,2,3");
    assert_eq!(view.badge_state(), crate::app::badge::BadgeState::Jog);
  }

  #[test]
  fn an_alarm_response_latches_a_banner() {
    let mut view = ViewState::default();
    view.apply(Event::Response(Response::Alarm(5)));
    assert_eq!(view.banner, Some(Banner::Alarm(5)));
  }

  #[test]
  fn a_stream_error_latches_a_banner_cleared_by_recovery() {
    let mut view = ViewState::default();
    // A program error halts the stream: the core emits the error response then the `Error` transition, in that
    // order. The banner latches on the transition, carrying the code stashed from the response.
    view.apply(Event::Response(Response::Error(9)));
    view.apply(Event::StateChanged(ConnectionState::Error));
    assert_eq!(view.banner, Some(Banner::StreamError(9)));
    // Recovering to Idle (e.g. a soft reset returning the halted lifecycle to Idle) clears the banner.
    view.apply(Event::StateChanged(ConnectionState::Idle));
    assert_eq!(view.banner, None);
  }

  #[test]
  fn a_manual_command_error_does_not_raise_a_stream_halted_banner() {
    // A manual command rejected with `error:N` (e.g. `$H` → `error:5` when homing is disabled) never drives the
    // core into `Error`, so it must not raise the "stream halted" strip — only log to the console. This is the
    // regression behind the `$H` banner that a soft reset could not clear (the lifecycle never actually moved).
    let mut view = ViewState::default();
    view.apply(Event::StateChanged(ConnectionState::Idle));
    view.apply(Event::Response(Response::Error(5)));
    assert_eq!(view.banner, None, "a manual-command error must not latch the stream-halted banner");
    // The error is still surfaced to the operator in the console.
    // The error is surfaced to the operator in the console, now decoded with the code's human name.
    assert!(
      view.console.iter().any(|l| l.text == "error:5 — Setting disabled"),
      "the error code still reaches the console, decoded with its name"
    );
  }

  #[test]
  fn an_alarm_banner_survives_a_state_change_until_dismissed() {
    let mut view = ViewState::default();
    view.apply(Event::Response(Response::Alarm(1)));
    view.apply(Event::StateChanged(ConnectionState::Idle));
    assert_eq!(view.banner, Some(Banner::Alarm(1)), "alarm persists until explicitly cleared");
    view.dismiss_banner();
    assert_eq!(view.banner, None);
  }

  #[test]
  fn progress_is_mirrored_and_fraction_computed() {
    let mut view = ViewState::default();
    view.apply(Event::Progress { sent: 7, acked: 5, total: 10 });
    assert_eq!(view.progress, Progress { sent: 7, acked: 5, total: 10 });
    assert!((view.progress.fraction() - 0.5).abs() < f32::EPSILON);
  }

  #[test]
  fn empty_program_has_zero_progress_fraction() {
    assert_eq!(Progress::default().fraction(), 0.0);
  }

  #[test]
  fn console_is_capped_at_capacity() {
    let mut view = ViewState::default();
    for i in 0..(CONSOLE_CAPACITY + 50) {
      view.note_sent(format!("G0 X{i}"));
    }
    assert_eq!(view.console.len(), CONSOLE_CAPACITY);
    // The oldest 50 lines were evicted; the front is now line 50.
    assert_eq!(view.console.front().unwrap().text, "G0 X50");
  }

  #[test]
  fn clear_console_empties_a_populated_buffer() {
    let mut view = ViewState::default();
    for i in 0..5 {
      view.note_sent(format!("G0 X{i}"));
    }
    assert_eq!(view.console.len(), 5, "the console is populated before the clear");
    view.clear_console();
    assert!(view.console.is_empty(), "clear_console wipes every line");
  }

  #[test]
  fn disconnect_clears_progress_status_and_cached_wco() {
    let mut view = ViewState::default();
    view.apply(Event::Progress { sent: 3, acked: 3, total: 5 });
    // A report carrying a WCO populates the cache; the disconnect must wipe it so a reconnect starts clean.
    feed_status(&mut view, "Run|MPos:1,2,3|WCO:1.000,2.000,3.000");
    assert_eq!(view.last_wco, vec![1.0, 2.0, 3.0]);
    view.apply(Event::Disconnected(None));
    assert_eq!(view.connection, ConnectionState::Disconnected);
    assert_eq!(view.progress, Progress::default());
    assert!(view.status.is_none());
    assert!(view.last_wco.is_empty(), "cached WCO must not survive a disconnect");
  }

  #[test]
  fn the_last_seen_override_persists_across_reports_that_omit_ov() {
    // The `Ov:` field is intermittent: grblHAL emits it on change / periodic refresh, NOT on every status
    // report (docs/gcode-streaming.md §status, `{|Ov:f,r,s}`). A report that omits it must NOT be read as
    // "overrides are now 100%" — that was the snap-back-to-centre bug. The last-seen value is cached and the
    // accessor returns it across the intervening Ov-less reports.
    let mut view = ViewState::default();
    // No report yet: the accessor falls back to the neutral 100% triple so the panel has something to render.
    assert_eq!(view.overrides(), (100, 100, 100), "with no report yet the override reads as neutral 100%");
    // A report carrying `Ov:` seeds the cache.
    feed_status(&mut view, "Run|MPos:1,2,3|FS:500,0|Ov:60,100,140");
    assert_eq!(view.overrides(), (60, 100, 140), "a report with `Ov:` is reflected");
    // The very next report — exactly as grblHAL sends them between refreshes — carries NO `Ov:`. The cached
    // value must survive: this is the regression. Before the fix the accessor snapped to (100,100,100).
    feed_status(&mut view, "Run|MPos:4,5,6|FS:500,0");
    assert_eq!(view.overrides(), (60, 100, 140), "an Ov-less report must NOT reset the override to centre");
    // A later report with a fresh `Ov:` updates the cache as expected.
    feed_status(&mut view, "Run|MPos:7,8,9|FS:500,0|Ov:60,100,150");
    assert_eq!(view.overrides(), (60, 100, 150), "a fresh `Ov:` updates the cached value");
  }

  #[test]
  fn the_cached_override_is_dropped_on_disconnect() {
    // The cached override is the previous board's; like the cached WCO it must not survive a disconnect, so a
    // reconnect to a (possibly different) board starts from neutral rather than the old session's value.
    let mut view = ViewState::default();
    feed_status(&mut view, "Run|MPos:1,2,3|Ov:60,100,140");
    assert_eq!(view.overrides(), (60, 100, 140));
    view.apply(Event::Disconnected(None));
    assert_eq!(view.overrides(), (100, 100, 100), "the previous board's override must not survive a disconnect");
  }

  #[test]
  fn setting_lines_fold_into_the_model_and_skip_the_console() {
    let mut view = ViewState::default();
    // A `$$` dump of `$<n>=<value>` lines populates the settings model, not the console (it would flood it).
    view.apply(Event::Response(Response::Setting { number: 0, value: "10".to_string() }));
    view.apply(Event::Response(Response::Setting { number: 110, value: "500.000".to_string() }));
    assert!(view.console.is_empty(), "a settings dump must not flood the console");
    assert_eq!(view.settings.len(), 2);
    assert_eq!(view.settings.value_of(0), Some("10"));
    assert_eq!(view.settings.value_of(110), Some("500.000"));
  }

  #[test]
  fn setting_enumeration_messages_label_rows_and_skip_the_console() {
    let mut view = ViewState::default();
    // A `$ES` enumeration row enriches the model's label/unit and is not console noise.
    view.apply(Event::Response(Response::Message("SETTING:0|1|Step pulse time|microseconds|2||1|1000".to_string())));
    view.apply(Event::Response(Response::Setting { number: 0, value: "10".to_string() }));
    assert!(view.console.is_empty(), "settings metadata must not reach the console");
    let row = view.settings.rows().next().expect("a settings row");
    assert_eq!(row.label(), "Step pulse time");
    assert_eq!(row.unit(), "microseconds");
    assert_eq!(row.value.as_deref(), Some("10"));
  }

  #[test]
  fn a_non_setting_message_still_reaches_the_console() {
    let mut view = ViewState::default();
    // A `[MSG:...]` that is not a SETTING row is still surfaced in the console as before.
    view.apply(Event::Response(Response::Message("MSG:hello".to_string())));
    assert_eq!(view.console.len(), 1);
    assert_eq!(view.console.back().unwrap().text, "[MSG:hello]");
  }

  #[test]
  fn disconnect_clears_the_settings_model() {
    let mut view = ViewState::default();
    view.apply(Event::Response(Response::Setting { number: 0, value: "10".to_string() }));
    assert!(!view.settings.is_empty());
    view.apply(Event::Disconnected(None));
    assert!(view.settings.is_empty(), "the previous board's settings must not survive a disconnect");
  }

  #[test]
  fn an_error_response_renders_with_its_decoded_name_in_the_console() {
    let mut view = ViewState::default();
    // Even with no enumeration fetched, the static fallback decodes the code: bare numbers are never shown alone.
    view.apply(Event::Response(Response::Error(21)));
    assert_eq!(view.console.back().unwrap().text, "error:21 — Modal group violation");
    view.apply(Event::Response(Response::Alarm(1)));
    assert_eq!(view.console.back().unwrap().text, "ALARM:1 — Hard limit");
  }

  #[test]
  fn an_errorcode_enumeration_row_enriches_the_codebook_and_skips_the_console() {
    let mut view = ViewState::default();
    // The firmware's `$EE` answer arrives as `[ERRORCODE:...]` messages; they enrich the codebook silently.
    view.apply(Event::Response(Response::Message(
      "ERRORCODE:21|Modal group violation|More than one G-code command from the same modal group".to_string(),
    )));
    assert!(view.console.is_empty(), "an ERRORCODE enumeration row must not flood the console");
    // A subsequent `error:21` renders with the enumerated name (here matching the static text).
    view.apply(Event::Response(Response::Error(21)));
    assert_eq!(view.console.back().unwrap().text, "error:21 — Modal group violation");
  }

  #[test]
  fn an_alarmcode_enumeration_override_drives_the_console_render() {
    let mut view = ViewState::default();
    // A firmware override with custom text must beat the static fallback when the console renders the code.
    view.apply(Event::Response(Response::Message(
      "ALARMCODE:1|Custom hard limit|A firmware-specific hard-limit explanation".to_string(),
    )));
    assert!(view.console.is_empty(), "an ALARMCODE enumeration row must not flood the console");
    view.apply(Event::Response(Response::Alarm(1)));
    assert_eq!(view.console.back().unwrap().text, "ALARM:1 — Custom hard limit");
  }

  #[test]
  fn the_firmwares_redundant_error_annotation_is_suppressed_but_generic_msgs_still_log() {
    let mut view = ViewState::default();
    // The firmware pushes `[MSG:error:21 Modal group violation]` right before the `error:21` — skirnir already
    // decodes the code, so the annotation is a duplicate and must not reach the console.
    view.apply(Event::Response(Response::Message("MSG:error:21 Modal group violation".to_string())));
    view.apply(Event::Response(Response::Message("MSG:ALARM:1 Hard limit".to_string())));
    assert!(view.console.is_empty(), "the firmware's pre-error/alarm context push is suppressed");
    // The skirnir-decoded error line still appears, so no context is lost.
    view.apply(Event::Response(Response::Error(21)));
    assert_eq!(view.console.back().unwrap().text, "error:21 — Modal group violation");
    // A generic `[MSG:...]` push (which never starts with `error:`/`ALARM:`) must still log normally.
    view.apply(Event::Response(Response::Message("MSG:Pgm End".to_string())));
    assert_eq!(view.console.back().unwrap().text, "[MSG:Pgm End]", "a legitimate MSG is never swallowed");
  }

  #[test]
  fn redundant_annotation_predicate_matches_only_the_exact_firmware_shape() {
    // Prefix + digits + space + name: the firmware's annotation, redundant.
    assert!(is_redundant_error_annotation("MSG:error:21 Modal group violation"));
    assert!(is_redundant_error_annotation("MSG:ALARM:1 Hard limit"));
    assert!(is_redundant_error_annotation("MSG:error:9 G-code lock"), "a multi-word name after the space matches");
    // Near-misses must NOT be swallowed.
    assert!(!is_redundant_error_annotation("MSG:error: no code"), "the prefix alone, sans digit, is not redundant");
    assert!(!is_redundant_error_annotation("MSG:error:21"), "a code with no trailing name is not the annotation");
    assert!(!is_redundant_error_annotation("MSG:error:21 "), "a trailing space but an empty name is not the shape");
    assert!(!is_redundant_error_annotation("MSG:errored sensor"), "a word starting with the prefix is not a code");
    assert!(!is_redundant_error_annotation("MSG:Pgm End"));
    assert!(!is_redundant_error_annotation("MSG:'$H'|'$X' to unlock"));
  }

  #[test]
  fn disconnect_clears_the_codebook_overrides() {
    let mut view = ViewState::default();
    view.apply(Event::Response(Response::Message("ERRORCODE:21|Renamed|desc".to_string())));
    assert_eq!(view.codes.error(21).name, "Renamed");
    view.apply(Event::Disconnected(None));
    // The override is gone; the static fallback name is restored for the next session.
    assert_eq!(view.codes.error(21).name, "Modal group violation");
  }

  #[test]
  fn a_probe_op_latches_a_successful_result_and_clears_awaiting() {
    let mut view = ViewState::default();
    view.begin_probe(ProbeKind::ZeroZ);
    assert!(view.probe_is_awaiting(), "the op is awaiting once issued");
    // An intervening status report must not resolve the op — only the `[PRB:]` result does.
    feed_status(&mut view, "Run|MPos:0,0,-1.0");
    assert!(view.probe_is_awaiting(), "a status report does not resolve the probe op");
    // The result lands: the latch captures the typed position, marks success, and clears awaiting.
    view.apply(Event::Response(Response::ProbeResult { position: vec![-1.015, 0.0, -2.5, 90.0], success: true }));
    assert!(!view.probe_is_awaiting(), "a result clears awaiting");
    let op = view.probe_op.as_ref().expect("the op is still present, now resolved");
    assert_eq!(op.last, Some(ProbeOutcome::Success { position: vec![-1.015, 0.0, -2.5, 90.0] }));
    // The line still reaches the console for the operator's record.
    assert_eq!(view.console.back().unwrap().text, "[PRB:-1.015,0.000,-2.500,90.000:1]");
  }

  #[test]
  fn a_zero_flag_probe_result_resolves_the_op_as_a_failure() {
    // The silent `G38.3`/`G38.5` path: no alarm, just a `:0` flag. The flag check is the only guard, so a
    // non-contact result must resolve the op as a failure (never a Success the shell would zero off).
    let mut view = ViewState::default();
    view.begin_probe(ProbeKind::ZeroZ);
    view.apply(Event::Response(Response::ProbeResult { position: vec![0.0, 0.0, 0.0], success: false }));
    assert!(!view.probe_is_awaiting());
    let outcome = view.probe_op.as_ref().and_then(|op| op.last.as_ref()).expect("a resolved outcome");
    assert!(!outcome.is_success(), "a :0 flag is a failure, not a success");
  }

  #[test]
  fn an_alarm_before_any_probe_result_resolves_the_op_as_failed() {
    // A no-contact alarming probe (`G38.2`) raises `ALARM:5` with no `[PRB:]` push: the op must resolve failed
    // off the alarm so the shell's follow-up is suppressed.
    let mut view = ViewState::default();
    view.begin_probe(ProbeKind::ZeroZ);
    view.apply(Event::Response(Response::Alarm(5)));
    assert!(!view.probe_is_awaiting(), "an alarm resolves the awaiting op");
    let outcome = view.probe_op.as_ref().and_then(|op| op.last.as_ref()).expect("a resolved outcome");
    assert!(!outcome.is_success());
    assert!(matches!(outcome, ProbeOutcome::Failure { reason } if reason.contains("ALARM:5")));
    // The alarm banner still latches as usual.
    assert_eq!(view.banner, Some(Banner::Alarm(5)));
  }

  #[test]
  fn an_error_while_awaiting_resolves_the_probe_as_failed() {
    // A rejected probe line (or the `error:9` g-code lock after an alarm) arriving while awaiting fails the op.
    let mut view = ViewState::default();
    view.begin_probe(ProbeKind::ZeroZ);
    view.apply(Event::Response(Response::Error(9)));
    assert!(!view.probe_is_awaiting());
    assert!(view.probe_op.as_ref().and_then(|op| op.last.as_ref()).is_some_and(|o| !o.is_success()));
  }

  #[test]
  fn a_late_probe_result_does_not_clobber_an_already_resolved_op() {
    // Once an op resolves (here, failed by an alarm), a trailing `[PRB:]` push (e.g. a redundant `$#` echo) must
    // not overwrite the captured outcome — the shell may already have acted on it.
    let mut view = ViewState::default();
    view.begin_probe(ProbeKind::ZeroZ);
    view.apply(Event::Response(Response::Alarm(5)));
    view.apply(Event::Response(Response::ProbeResult { position: vec![0.0, 0.0, 0.0], success: true }));
    let outcome = view.probe_op.as_ref().and_then(|op| op.last.as_ref()).expect("the failure outcome stands");
    assert!(!outcome.is_success(), "a late result must not flip a resolved op to success");
  }

  #[test]
  fn fail_probe_is_a_noop_when_nothing_is_awaiting() {
    // The shell's timeout/disconnect failure path must not fabricate an op when none is in flight.
    let mut view = ViewState::default();
    view.fail_probe("timeout");
    assert!(view.probe_op.is_none());
    // And it must not flip an already-resolved op.
    view.begin_probe(ProbeKind::ZeroZ);
    view.apply(Event::Response(Response::ProbeResult { position: vec![1.0], success: true }));
    view.fail_probe("timeout");
    assert!(view.probe_op.as_ref().unwrap().last.as_ref().unwrap().is_success());
  }

  #[test]
  fn disconnect_clears_the_probe_op() {
    let mut view = ViewState::default();
    view.begin_probe(ProbeKind::ZeroZ);
    view.apply(Event::Disconnected(None));
    assert!(view.probe_op.is_none(), "a probe op must not survive a disconnect");
  }

  #[test]
  fn non_status_responses_reach_the_console() {
    let mut view = ViewState::default();
    view.apply(Event::Response(Response::Ok));
    view.apply(Event::Response(Response::Message("MSG:hello".to_string())));
    assert_eq!(view.console.len(), 2);
    assert_eq!(view.console.front().unwrap().text, "ok");
    assert_eq!(view.console.back().unwrap().text, "[MSG:hello]");
  }
}
