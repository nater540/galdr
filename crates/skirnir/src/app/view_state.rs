//! The pure view-state reducer: the bridge between engine [`Event`]s and what the egui layer draws.
//!
//! egui is immediate-mode — every frame the UI is a pure function of state — so all the decisions live here,
//! in a plain reducer with no egui types, and the view code just renders [`ViewState`] and forwards intents.
//! That keeps the hard logic (deriving the missing position from WCO, capping the console, latching the
//! alarm/error banner, tracking progress) unit-testable without a window. The reducer never performs I/O; it
//! folds one [`Event`] into the state and is driven from the UI's per-frame event drain.

use std::collections::VecDeque;

use super::badge::BadgeState;
use crate::engine::Event;
use crate::error::TransportError;
use crate::protocol::{ConnectionState, PositionKind, Response, StatusReport, parse_status};

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
  /// The last-seen `WCO:` vector, cached across reports (grbl pushes it only intermittently) so we can always
  /// derive the position kind the current report did not carry.
  pub last_wco: Vec<f64>,
  /// Streaming progress.
  pub progress: Progress,
  /// A latched alarm/error banner, if active.
  pub banner: Option<Banner>,
  /// The rolling console buffer, oldest first, capped at [`CONSOLE_CAPACITY`].
  pub console: VecDeque<LogLine>,
}

impl Default for ViewState {
  fn default() -> Self {
    ViewState {
      connection: ConnectionState::Disconnected,
      status: None,
      last_wco: Vec::new(),
      progress: Progress::default(),
      banner: None,
      console: VecDeque::new(),
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

  /// React to a lifecycle transition. Reaching a clean running state clears a stale error banner so the UI
  /// does not show "error" over a recovered connection.
  fn on_state_changed(&mut self, state: ConnectionState) {
    self.connection = state;
    match state {
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
        self.status = Some(report);
        // Status reports are high-frequency telemetry; echoing each to the console would drown it. Skip them.
        return;
      }
      Response::Alarm(code) => self.banner = Some(Banner::Alarm(*code)),
      Response::Error(code) => self.banner = Some(Banner::StreamError(*code)),
      _ => {}
    }
    self.log(LogSource::Received, render_response(&response));
  }

  /// React to the terminal disconnect event.
  fn on_disconnected(&mut self, reason: Option<TransportError>) {
    self.connection = ConnectionState::Disconnected;
    self.progress = Progress::default();
    self.status = None;
    // Drop the cached WCO so a reconnect does not show "WCO set" or derive WPos/MPos from a stale offset before
    // the new session reports its own. Report-derived state must not survive across a disconnect.
    self.last_wco.clear();
    match reason {
      Some(err) => self.log(LogSource::Notice, format!("disconnected: {err}")),
      None => self.log(LogSource::Notice, "disconnected".to_string()),
    }
  }

  /// Append a console line, evicting the oldest once at capacity so the buffer never grows without bound.
  fn log(&mut self, source: LogSource, text: String) {
    if self.console.len() >= CONSOLE_CAPACITY {
      self.console.pop_front();
    }
    self.console.push_back(LogLine { source, text });
  }
}

/// Render a non-status response as a one-line console string. Status reports are handled separately and never
/// reach here.
fn render_response(response: &Response) -> String {
  match response {
    Response::Ok => "ok".to_string(),
    Response::Error(code) => format!("error:{code}"),
    Response::Alarm(code) => format!("ALARM:{code}"),
    Response::Message(body) => format!("[{body}]"),
    Response::Banner(text) => text.clone(),
    Response::StartupEcho(text) => format!(">{text}"),
    Response::Unknown(text) => text.clone(),
    // Status is rendered by the DRO, not the console; included for exhaustiveness only.
    Response::Status(body) => format!("<{body}>"),
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::protocol::RunState;

  /// Convenience: feed a status-report body through the reducer as the engine would.
  fn feed_status(view: &mut ViewState, body: &str) {
    view.apply(Event::Response(Response::Status(body.to_string())));
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
    view.apply(Event::Response(Response::Error(9)));
    assert_eq!(view.banner, Some(Banner::StreamError(9)));
    // Recovering to Idle clears a stream-error banner.
    view.apply(Event::StateChanged(ConnectionState::Idle));
    assert_eq!(view.banner, None);
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
  fn non_status_responses_reach_the_console() {
    let mut view = ViewState::default();
    view.apply(Event::Response(Response::Ok));
    view.apply(Event::Response(Response::Message("MSG:hello".to_string())));
    assert_eq!(view.console.len(), 2);
    assert_eq!(view.console.front().unwrap().text, "ok");
    assert_eq!(view.console.back().unwrap().text, "[MSG:hello]");
  }
}
