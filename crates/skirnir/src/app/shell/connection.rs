//! Serial connection control: connect/disconnect, port enumeration, and the identify probe.
//! Split out of `shell.rs` (A4); pure relocation.

use super::*;

impl SkirnirApp {
  /// Open the serial port and attach the engine in response to a user-initiated connect. Records the endpoint
  /// and arms auto-reconnect so a later unexpected drop is chased, and resets the backoff schedule so this fresh
  /// session starts clean. The actual open is delegated to [`Self::connect_inner`], shared with the
  /// auto-reconnect path. A failure to open is surfaced as a console notice, leaving the app disconnected.
  pub(crate) fn connect(&mut self, path: &str, baud: u32) {
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
  pub(crate) fn connect_inner(&mut self, path: &str, baud: u32) {
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
  pub(crate) fn disconnect(&mut self) {
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
  pub(crate) fn refresh_ports(&mut self) {
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
  pub(crate) fn identify_port(&mut self, path: &str) {
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
}
