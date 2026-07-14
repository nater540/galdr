//! Operator controls: feed/rapid/spindle overrides, the `$`-settings channel, and jogging (both
//! discrete moves and the streamed continuous-jog pump). Split out of `shell.rs` (A4).

use super::*;

impl SkirnirApp {
  /// Drive a feed/spindle override slider to an absolute target percent. grbl exposes only relative override
  /// steps, so we read the override the firmware last reported in `Ov:` (defaulting to 100% before any report)
  /// and emit the minimal ±10/±1/reset sequence the pure [`crate::app::overrides::override_commands`] computes. Each
  /// step rides the out-of-band real-time path (uncounted), so an override never disturbs the send-ahead
  /// window. The live status reporter will reflect the new value within a poll interval, re-centering the
  /// slider on the firmware's truth.
  pub(crate) fn set_override(&mut self, axis: crate::app::overrides::OverrideAxis, target: u32) {
    use crate::app::overrides::OverrideAxis;
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
  pub(crate) fn request_settings(&mut self) {
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
  pub(crate) fn write_setting(&mut self, number: u32, value: &str) {
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
  pub(crate) fn save_settings(&mut self) {
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

  /// Form and send a step `$J=` jog line via the shared [`crate::app::intent::jog_line`] builder, echoing it.
  pub(crate) fn jog(&mut self, axis: Axis, dir: Dir, distance: f64, feed: f64) {
    let line = crate::app::intent::jog_line(axis, dir, distance, feed);
    self.view.note_sent(line.clone());
    self.send_command(Command::SendLine(line));
  }

  /// Begin a continuous (press-and-hold) jog. Rather than one long move (which a jog-cancel could only stop at
  /// its far boundary — the runaway bug), the held jog is *streamed* as short `$J=` increments by
  /// [`Self::pump_jog_stream`]; the operator stops it by releasing, which fires [`Intent::JogStop`]. The first
  /// increment is due immediately so motion starts without waiting a cadence.
  pub(crate) fn jog_start(&mut self, axis: Axis, dir: Dir, feed: f64) {
    self.jog_stream = Some(JogStream { axis, dir, feed, next_send_at: Instant::now() });
  }

  /// End a continuous jog: stop streaming increments and inject jog-cancel (`0x85`). The firmware flushes the
  /// queued jog blocks and decelerates the active (short) block at its boundary, so motion halts within one
  /// increment's travel. Safe to send when not jogging — the firmware ignores it.
  pub(crate) fn jog_stop(&mut self) {
    self.clear_jog_stream();
    self.send_command(Command::Realtime(crate::protocol::RealtimeCommand::JogCancel));
  }

  /// Tear down any in-progress continuous jog. The single point that clears the streamed-jog state, so every site
  /// that ends a jog — operator release, a disconnect, an engine drop — stops the increment pump the same way.
  pub(crate) fn clear_jog_stream(&mut self) {
    self.jog_stream = None;
  }

  /// Emit the next increment of a held continuous jog if one is active and due. Paced by wall clock so blocks are
  /// produced at roughly the firmware's execution rate, and gated by the firmware's reported planner-blocks-free
  /// (`Bf:`) so a long hold never overruns the queue. Each increment is `feed * BLOCK_SECS` long, so its motion
  /// time — the worst-case stop latency after release — stays ~[`JOG_STREAM_BLOCK_SECS`] regardless of feed. The
  /// increments are not echoed to the console: at several per second the echo would bury real traffic.
  pub(crate) fn pump_jog_stream(&mut self) {
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
        Some(crate::app::intent::jog_line(axis, dir, distance, feed))
      }
    };
    if let Some(line) = send_line {
      self.send_command(Command::SendLine(line));
    }
  }
}
