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

use std::time::Duration;

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

/// The skirnir application: state plus the tokio runtime that hosts the engine task.
pub struct SkirnirApp {
  /// The tokio runtime the engine's driver task runs on. Held for the app's lifetime; `Engine::connect` is
  /// called inside its context so the internal `tokio::spawn` has a runtime to attach to. Only the serial
  /// connect path enters it, so a gui-only build (no `serial`) carries no runtime and never reads this field.
  #[cfg(feature = "serial")]
  runtime: tokio::runtime::Runtime,
  /// The live engine handle when connected, else `None`. Dropping it winds the engine task down.
  engine: Option<EngineHandle>,
  /// The engine-derived render state.
  view: ViewState,
  /// The transient widget state the views read and mutate.
  ui: UiState,
  /// The result channel of an in-flight on-demand port identify probe, if one is running. The probe runs on
  /// the runtime (off the UI thread); the verdict arrives here and is drained into the console each frame, so a
  /// 500ms probe never blocks rendering. `None` when no probe is in flight.
  #[cfg(feature = "serial")]
  pending_probe: Option<std::sync::mpsc::Receiver<String>>,
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
      view: ViewState::default(),
      ui: UiState::default(),
      #[cfg(feature = "serial")]
      pending_probe: None,
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
    // Bounded drain: take what is queued this frame. `try_recv` never blocks, so the UI thread stays free.
    while let Some(event) = engine.try_recv() {
      self.view.apply(event);
      saw_any = true;
    }
    saw_any
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
      Intent::SendLine(line) => self.send_line(line),
      Intent::Realtime(cmd) => {
        self.send_command(Command::Realtime(cmd));
      }
      Intent::Jog { axis, dir, distance, feed } => self.jog(axis, dir, distance, feed),
      Intent::DismissBanner => self.view.dismiss_banner(),
      Intent::ProbeZ { depth, feed, plate_thickness } => self.probe_z(depth, feed, plate_thickness),
      Intent::Home => self.send_line("$H".to_string()),
      Intent::RunOrResume => self.run_or_resume(),
      Intent::SetWorkZero { axes } => self.send_line(super::intent::work_zero_line(&axes)),
    }
  }

  /// Open the serial port and attach the engine. Done inside the runtime context so the engine's internal
  /// spawn has a home. A failure to open is surfaced as a console notice, leaving the app disconnected.
  fn connect(&mut self, path: &str, baud: u32) {
    #[cfg(feature = "serial")]
    {
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
    #[cfg(not(feature = "serial"))]
    {
      let _ = (path, baud);
      self.notice("built without the `serial` feature; cannot open a port".to_string());
    }
  }

  /// Tear the connection down. Sending `Disconnect` ends the engine task; dropping the handle releases it.
  fn disconnect(&mut self) {
    if let Some(engine) = &self.engine {
      engine.send(Command::Disconnect);
    }
    self.engine = None;
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

  /// Send one manual line, echoing it to the console as sent traffic.
  fn send_line(&mut self, line: String) {
    let trimmed = line.trim().to_string();
    if trimmed.is_empty() {
      return;
    }
    self.view.note_sent(trimmed.clone());
    self.send_command(Command::SendLine(trimmed));
  }

  /// Form and send a `$J=` jog line. Jog is independent of modal state, so a fixed `G91` (relative) +
  /// `G21` (mm) prefix keeps it predictable regardless of the program's modal context.
  fn jog(&mut self, axis: Axis, dir: Dir, distance: f64, feed: f64) {
    let signed = distance * dir.sign();
    let line = format!("$J=G91 G21 {}{:.3} F{:.0}", axis.letter(), signed, feed);
    self.view.note_sent(line.clone());
    self.send_command(Command::SendLine(line));
  }

  /// Sequence a Z probe: `G38.2` toward `-depth` at `feed`, then set work-Z to the plate thickness via
  /// `G10 L20 P0` (L20, the set-relative-to-current form — so the copper top becomes work-Z `plate_thickness`,
  /// i.e. Z0 at the copper surface after the plate is removed). Each line is sent and echoed; the engine acks
  /// them in order. The offset line is built by the shared [`super::intent::work_offset_line`] helper.
  fn probe_z(&mut self, depth: f64, feed: f64, plate_thickness: f64) {
    let probe = format!("G38.2 Z-{depth:.3} F{feed:.0}");
    let zero = super::intent::work_offset_line(&[(Axis::Z, plate_thickness)]);
    for line in [probe, zero] {
      self.view.note_sent(line.clone());
      self.send_command(Command::SendLine(line));
    }
  }

  /// Forward a command to the engine if connected; surface a notice if not. Returns whether it was sent.
  fn send_command(&mut self, command: Command) -> bool {
    match &self.engine {
      Some(engine) if engine.send(command) => true,
      Some(_) => {
        self.notice("engine is gone; reconnect".to_string());
        self.engine = None;
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

    // 2. Build the frame. Views push intents into a per-frame sink; we act on them after layout so a view
    //    never mutates engine state mid-render. eframe 0.34 hands us the root `Ui`; panels are laid out into
    //    it with `show_inside`, and the central panel is what remains after the docked panels claim their
    //    edges. We reach the `Context` (for repaint scheduling and the settings window) via `ui.ctx()`.
    let mut sink = super::intent::IntentSink::new();
    let ctx = ui.ctx().clone();
    use super::metrics::Metrics;
    use super::theme::Theme;

    // The toolbar is a fixed 40px bar (design §03); pin it so it neither collapses nor grows with content.
    egui::Panel::top("toolbar").exact_size(Metrics::TOOLBAR_H).show_inside(ui, |ui| {
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
    egui::Panel::bottom("dock").resizable(false).exact_size(dock_h).show_inside(ui, |ui| {
      views::dock(ui, &self.view, &mut self.ui, &mut sink);
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
        egui::ScrollArea::vertical().show(ui, |ui| {
          views::dro(ui, &self.view, &mut self.ui, &mut sink);
          ui.separator();
          views::jog(ui, &self.view, &mut self.ui, &mut sink);
        });
      });

    egui::Panel::right("rightcol").resizable(false).exact_size(Metrics::RIGHT_COL_W).frame(column_frame)
      .show_inside(ui, |ui| {
        egui::ScrollArea::vertical().show(ui, |ui| {
          views::overrides(ui, &self.view, &mut sink);
          ui.separator();
          views::probe(ui, &self.view, &mut self.ui, &mut sink);
          ui.separator();
          views::settings_panel(ui, &mut self.ui, &mut sink);
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
      egui::Window::new("Settings").open(&mut open).show(&ctx, |ui| {
        views::settings(ui, &mut self.ui, &mut sink);
      });
      self.ui.settings_open = open;
    }

    // 3. Act on the intents the views emitted this frame, in order.
    for intent in sink.drain() {
      self.handle_intent(intent);
    }

    // 4. Schedule the next wake. Only spin the steady timer when there is live traffic to expect: while an
    //    engine is attached (the firmware auto-reports and a stream needs prompt progress updates, and the
    //    engine's events are polled from this loop), or for one extra frame after an event arrived. When
    //    disconnected there is no engine to poll, so we request no repaint and the UI sleeps until the next
    //    user input rather than waking 20×/sec for nothing.
    // A pending probe must keep the timer alive even when disconnected, so its verdict is drained promptly.
    let probe_pending = {
      #[cfg(feature = "serial")]
      {
        self.pending_probe.is_some()
      }
      #[cfg(not(feature = "serial"))]
      {
        false
      }
    };
    if saw_event || saw_probe || probe_pending || self.engine.is_some() {
      ctx.request_repaint_after(REPAINT_INTERVAL);
    }
  }
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
