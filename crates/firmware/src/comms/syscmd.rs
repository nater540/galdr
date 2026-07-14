//! `$`-system-command handlers (DOC-08 / DOC-04): `handle_system_command` dispatches every `$…` query/action off
//! the host-tested `SystemCommand` classifier, plus its family of dumps, enumerations, build-info, parser-state,
//! coordinate/NGC-parameter, startup-line, `$PBX` import/export, and `$RST` restore handlers — extracted verbatim
//! from `comms.rs` (architecture-refactor A1, step 6). Probe/jog/homing/spindle handlers stay in `comms.rs` for their
//! own steps; the homing pair (`handle_home`/`run_homing_cycle`), the shared `send_message`, and the coordinate /
//! reporter-cadence helpers (`flush_coordinates`/`reset_wco_reporter`/`reset_ov_reporter`) are consumed by BOTH this
//! module and staying code, so they remain in `comms.rs` and are reached via the re-export. `comms.rs` re-exports this
//! module (`pub(crate) use syscmd::*;`) so the consumer's `handle_line` call to `handle_system_command` still resolves.

use firmware_core::gcode::{DistanceMode as GcodeDistance, ModalState, Parser, Units as GcodeUnits};
use firmware_core::protocol::{AlarmCode, ControlState, ResponseWriter, SystemCommand, ERROR_UNSUPPORTED_COMMAND};

// `use super::*` supplies the wide parent surface the moved handlers touch: the state accessors + statics, the
// `enqueue`/`ack`/`error` emitters, the staying helpers (`send_message`, `reset_pipeline`, `force_spindle_off` /
// `force_coolant_off`, `reset_wco_reporter`, `settings_write_blocked`), the `settings` / `firmware_core::protocol`
// enum + error-code + descriptor types, and `NGC_PARAMETER_LINES` — all already in scope in `comms.rs` and visible to
// this child module through the glob. Only the few names the glob does not resolve unambiguously are explicit above.
use super::*;

/// Answer a `$<rest>` system command, dispatched off the host-tested [`SystemCommand`] classifier so an
/// UNRECOGNIZED `$` command returns `error:3` instead of a fabricated `ok` (the Phase A hardening — the old
/// lenient fallthrough is gone). Every arm emits exactly one `ok`/`error:N` (multi-line reports end with the
/// `ok`), preserving the one-response-per-line contract. Real handlers: `$X` unlock, `$C` check toggle, `$SLP`
/// sleep, `$N`/`$N0=`/`$N1=` startup lines, `$H` homing (gated on `$22`, unimplemented per DOC-06), `$RST=$`
/// restore-defaults; the existing `$$`/`$I`/`$G`/`$#`/`$PBX`/`$x=val` paths are preserved.
pub(crate) async fn handle_system_command(
  rest: &[u8],
  parser: &mut Parser,
  state: &mut ConsumerState,
  flash: &'static SharedFlash,
) {
  match SystemCommand::classify(rest) {
    SystemCommand::Help => {
      // Bare `$` (help): emit the grbl help line, then `ok` — not a bare ack (the host's `$`-help probe wants
      // the documented one-liner).
      send_help().await;
      ack().await;
    }
    SystemCommand::SettingsDump => {
      // `$$` settings dump: one `$n=value` line per supported setting, then the terminating `ok`.
      dump_settings().await;
      ack().await;
    }
    SystemCommand::BuildInfo { extended } => {
      send_build_info(extended).await;
      ack().await;
    }
    SystemCommand::ParserState => {
      send_parser_state(parser).await;
      ack().await;
    }
    SystemCommand::NgcParams => {
      // `$#` NGC parameters dump (Phase B): emit `[G54:..]`..`[G59:..]`, `[G28:..]`, `[G30:..]`, `[G92:..]`,
      // `[TLO:z]`, and `[PRB:..]` from the live coordinate model, then the terminating `ok`.
      dump_ngc_parameters().await;
      ack().await;
    }
    SystemCommand::Unlock => handle_unlock().await,
    SystemCommand::ToggleCheck => handle_toggle_check(parser, state).await,
    SystemCommand::Sleep => handle_sleep().await,
    SystemCommand::Home => handle_home(parser, state).await,
    SystemCommand::StartupQuery => handle_startup_query(state).await,
    SystemCommand::StartupSet { index, gcode } => handle_startup_set(index, gcode, state).await,
    SystemCommand::RestoreSettings => handle_restore_settings(parser, state, flash).await,
    SystemCommand::RestoreParams => handle_restore_params(flash).await,
    SystemCommand::RestoreAll => handle_restore_all(parser, state, flash).await,
    // Phase F runtime enumerations: each streams its bracket lines through the single USB writer, then `ok`.
    SystemCommand::EnumSettings => {
      enumerate_settings().await;
      ack().await;
    }
    SystemCommand::EnumSettingGroups => {
      enumerate_setting_groups().await;
      ack().await;
    }
    SystemCommand::EnumErrorCodes => {
      enumerate_error_codes().await;
      ack().await;
    }
    SystemCommand::EnumAlarmCodes => {
      enumerate_alarm_codes().await;
      ack().await;
    }
    SystemCommand::SettingDescription { id } => {
      // `$SED=<n>`: emit the one `[SETTINGDESCR:]` line for a KNOWN setting, then `ok`. An unknown id emits no
      // bracket line but still `ok`s the (recognized) `$SED` command — grbl treats a description query for an
      // absent setting as an empty result, not an error (the command itself is valid).
      send_setting_description(id).await;
      ack().await;
    }
    SystemCommand::SetSetting { body } => write_setting_command(body).await,
    SystemCommand::PbExport => {
      // `$PBX` bulk settings export (DOC-04 host-sync): emit the live settings as hex-encoded protobuf frame
      // chunks, then the terminating `ok`. skirnir reassembles and decodes via the shared `galdr-proto` schema.
      export_pb().await;
      ack().await;
    }
    SystemCommand::PbImport { hex } => handle_pb_write(hex, state).await,
    SystemCommand::Unknown => {
      // The hardened fallthrough: an unrecognized `$` command returns `error:3` ("'$' command not
      // recognized"), NEVER a spurious `ok`. This closes the old lenient fake-ack hole.
      error(ERROR_UNSUPPORTED_COMMAND).await;
    }
  }
}

/// Handle `$X` (kill alarm lock). From a non-locked alarm: clear to Normal, emit `[MSG:Caution: Unlocked]`
/// then `ok`. From a non-alarm state: a no-op `ok`. From a LOCKED critical alarm (hard/soft limit, e-stop):
/// `error:N` — only a soft reset (after the physical cause clears) can unlock those. The control-state machine
/// (host-tested) decides which; this wiring just emits the matching response and publishes the new state.
async fn handle_unlock() {
  let (next, outcome) = control_state().unlock();
  set_control_state(next);
  match outcome {
    UnlockOutcome::Unlocked => {
      send_message("Caution: Unlocked").await;
      ack().await;
    }
    UnlockOutcome::NotAlarmed => ack().await,
    UnlockOutcome::Locked => error(ERROR_UNSUPPORTED_COMMAND).await,
  }
}

/// Handle `$C` (toggle check mode). Enter from Normal → `[MSG:Enabled]` + `ok`. Leave from Check, which grbl
/// realizes as a soft reset: emit `[MSG:Disabled]`, run the pipeline reset (rebuild parser/planner, flush the
/// planner, re-emit the banner) — which also publishes the post-reset control state and `ok`s nothing — then
/// `ok`. Rejected (not Normal/Check) → `error:N`. The new control state is published before the side-effects.
async fn handle_toggle_check(parser: &mut Parser, state: &mut ConsumerState) {
  let (next, toggle) = control_state().toggle_check(HOMING_ENABLED.load(Ordering::Relaxed));
  match toggle {
    CheckToggle::Enabled => {
      set_control_state(next);
      send_message("Enabled").await;
      ack().await;
    }
    CheckToggle::Disabled => {
      // Leaving check mode is a soft reset per grbl: emit `[MSG:Disabled]`, then rebuild the pipeline. The
      // control state is set to the post-reset boot value (computed by `toggle_check`) here so the executor
      // reset and the published state agree; `reset_pipeline` emits the banner and re-arms the parser/planner.
      send_message("Disabled").await;
      set_control_state(next);
      reset_pipeline(parser, state).await;
    }
    CheckToggle::Rejected => error(ERROR_UNSUPPORTED_COMMAND).await,
  }
}

/// Handle `$SLP` (sleep). Allowed only from Normal: publish the Sleep state, signal the core-1 executor to
/// hold (a sleeping machine must not run queued blocks), and `ok`. The actual spindle/coolant shutdown and
/// driver de-energize are a HARDWARE BOUNDARY owned by DOC-07 (spindle) / DOC-03 (TMC STEP_EN) and are
/// stubbed here — flagged in the report. Wake is by a soft reset (`0x18`), handled by the existing reset path.
async fn handle_sleep() {
  let (next, entered) = control_state().enter_sleep();
  if entered {
    set_control_state(next);
    // Hold the executor at the next block boundary so no further blocks run while asleep: RAISE the hold LEVEL
    // (Finding #11) and wake the executor. A `~` will NOT release it — `resumes_on_cycle_start` is false in
    // Sleep, so the `~` dispatch leaves the level set (Finding #1) — and only a soft reset wakes the machine
    // (which clears the level and zeroes the executor via the reset path). Reusing the same level a feed-hold
    // uses keeps one parking mechanism for both.
    HOLD_REQUESTED.store(true, Ordering::Release);
    HOLD_WAKE.signal(());
    // DOC-07: `$SLP` stops the spindle (LEDC duty → 0, SPIN_EN de-asserted) AND coolant, independent of motion
    // state. The TMC driver de-energize (STEP_EN high) remains a DOC-03 follow-up, flagged in the report.
    force_spindle_off();
    force_coolant_off();
    ack().await;
  } else {
    // Sleep is rejected from any non-Normal state (alarm/check/already asleep), matching grbl.
    error(ERROR_UNSUPPORTED_COMMAND).await;
  }
}

/// Handle `$N` (query stored startup lines): echo both slots as `$N0=...`/`$N1=...` (empty when unset), then
/// `ok`. Phase A stores and echoes startup lines but does NOT execute them on reset (see [`handle_startup_set`]).
async fn handle_startup_query(state: &ConsumerState) {
  for (index, slot) in state.startup_lines.iter().enumerate() {
    let mut line = Response::new();
    let gcode: &[u8] = slot.as_deref().unwrap_or(&[]);
    // `$Nn=<gcode>` — the stored line is valid UTF-8 by construction (it was a received GCode line). A
    // formatting/capacity failure simply skips the echo line; the closing `ok` still terminates the response.
    if write_startup_echo(index as u8, gcode, &mut line) {
      enqueue(line).await;
    }
  }
  ack().await;
}

/// Render a `$Nn=<gcode>` startup-line echo (CRLF-terminated) into `out`. Returns `false` only on a capacity
/// failure (unreachable with the caller's correctly sized buffer). The GCode bytes are ASCII (a received line).
fn write_startup_echo(index: u8, gcode: &[u8], out: &mut Response) -> bool {
  use core::fmt::Write as _;
  if write!(out, "$N{index}=").is_err() {
    return false;
  }
  // Append the raw GCode bytes as chars; they are 7-bit ASCII from the line framer, so this never mis-encodes.
  for &b in gcode {
    if out.push(b as char).is_err() {
      return false;
    }
  }
  out.push_str("\r\n").is_ok()
}

/// Handle `$N0=<gcode>` / `$N1=<gcode>` (store a startup line). The line is stored in RAM and `ok`'d. Phase A
/// SCOPE: startup lines are STORED and ECHOED (`$N`) but NOT executed on reset — running them at init is a
/// deliberate follow-up (it must re-establish modal state safely and is gated on the homing/alarm-exit rules).
/// PERSISTENCE BOUNDARY (TODO DOC-04): the lines are held only in RAM. Persisting them needs a separate NVS
/// record or a non-scalar proto field (which would break `Settings: Copy`), both larger than Phase A.
async fn handle_startup_set(index: u8, gcode: &[u8], state: &mut ConsumerState) {
  // grbl's error:8 gate: a `$Nx=` startup-line write is a persisted-settings mutation, Idle/Alarm only.
  if settings_write_blocked().await {
    error(ERROR_NOT_IDLE).await;
    return;
  }
  let slot = index as usize;
  if slot >= state.startup_lines.len() {
    // `classify` only emits index 0/1, so this is unreachable; reject defensively rather than panic-index.
    error(ERROR_UNSUPPORTED_COMMAND).await;
    return;
  }
  if gcode.is_empty() {
    // `$Nn=` with empty body clears the slot (grbl behavior).
    state.startup_lines[slot] = None;
    ack().await;
    return;
  }
  let mut stored = heapless::Vec::new();
  if stored.extend_from_slice(gcode).is_err() {
    // The startup line exceeds the line buffer — reject rather than truncate.
    error(ERROR_UNSUPPORTED_COMMAND).await;
    return;
  }
  state.startup_lines[slot] = Some(stored);
  ack().await;
}

/// Handle `$RST=$` / `$RST=*` (restore default settings). Replace the live [`SETTINGS`] with the compiled
/// defaults, persist them to flash, clear the stored startup lines, then soft-reset the pipeline so the
/// defaults take effect (grbl auto-resets after a `$RST`). `$RST=*` additionally clears startup lines / build
/// info — Phase A clears the startup lines for both; the build-info string is compiled, not stored, so there
/// is nothing further to clear. The terminating banner comes from `reset_pipeline`.
async fn handle_restore_settings(parser: &mut Parser, state: &mut ConsumerState, flash: &'static SharedFlash) {
  // grbl's error:8 gate: `$RST=$` wipes the whole settings blob (and runs the pipeline reset), Idle/Alarm only.
  if settings_write_blocked().await {
    error(ERROR_NOT_IDLE).await;
    return;
  }
  let defaults = Settings::default();
  {
    let mut guard = SETTINGS.lock().await;
    *guard = Some(defaults);
  }
  // Persist immediately (a restore is an explicit, infrequent operation, so the coalesced-flush deferral is
  // not needed): write the defaults straight to flash. A failed persist is logged and swallowed — the in-RAM
  // defaults already apply — matching the existing best-effort store policy.
  {
    let mut store = FlashRecordStore::settings(flash);
    if settings::store_settings(&mut store, &defaults).await.is_err() {
      #[cfg(feature = "defmt")]
      defmt::warn!("settings: failed to persist restored defaults");
    }
  }
  SETTINGS_DIRTY.store(false, Ordering::Release);
  // Refresh the cached status config from the restored defaults so the next report reflects them (Finding #14).
  refresh_status_cfg().await;
  // Clear the stored startup lines (`$RST=$`/`$RST=*` both drop them in grbl) and update the homing mirror so
  // the post-reset boot state is computed from the restored `$22` (default: disabled → Normal).
  state.startup_lines = [None, None];
  HOMING_ENABLED.store(defaults.homing_enabled(), Ordering::Relaxed);
  HARD_LIMITS_ENABLED.store(defaults.hard_limits_enabled(), Ordering::Relaxed);
  LIMIT_INVERT.store(defaults.limit_invert, Ordering::Relaxed);
  LIMIT_DEBOUNCE_MS.store(defaults.homing_debounce_ms, Ordering::Relaxed);
  set_control_state(ControlState::boot(defaults.homing_enabled()));
  // Soft-reset the pipeline so the restored settings take effect (it rebuilds the planner from the live
  // settings and emits the banner). It does not emit `ok`; grbl's `$RST` is acknowledged by the reset itself.
  reset_pipeline(parser, state).await;
}

/// Queue the bare-`$` help line.
async fn send_help() {
  let mut s = Response::new();
  if ResponseWriter::help(&mut s).is_ok() {
    enqueue(s).await;
  }
}

/// Number of frame BYTES hex-encoded per `[PB:...]` export line. 48 bytes → 96 hex chars; with the `[PB:`/`]`
/// wrapper and CRLF that is ~103 chars, comfortably under [`RESPONSE_CAPACITY`].
const PB_CHUNK_BYTES: usize = 48;

/// Emit the `$PBX` bulk export: encode the live settings into a storage frame and stream it as hex-encoded
/// `[PB:<hex>]` lines (chunked to fit the line length). The terminating `ok` is emitted by the caller; the
/// host concatenates the chunk payloads in order, hex-decodes, and decodes the frame with the shared schema.
async fn export_pb() {
  let snapshot = settings_snapshot().await;
  let mut frame: heapless::Vec<u8, { settings::wire::FRAME_MAX_LEN }> = heapless::Vec::new();
  // Encoding the live settings cannot fail (the buffer is sized for the largest frame); on the impossible
  // error, emit nothing and let the caller's `ok` close the (empty) export so the host can retry.
  if settings::wire::encode(&snapshot, &mut frame).is_err() {
    return;
  }
  for chunk in frame.chunks(PB_CHUNK_BYTES) {
    let mut line = Response::new();
    if line.push_str("[PB:").is_ok() && settings::write_hex(chunk, &mut line) && line.push_str("]\r\n").is_ok() {
      enqueue(line).await;
    }
  }
}

/// Handle one `$PBX=<hex>` import chunk: feed it to the reassembly receiver. A non-final chunk acknowledges
/// and waits; the final chunk completes the frame, which is applied to the live [`SETTINGS`] and marked DIRTY
/// (the coalesced flush persists it once the burst drains — Finding #14b) before acknowledging; malformed
/// input resets the receiver and returns `error:N` so the host can retry from the first chunk.
async fn handle_pb_write(hex: &[u8], state: &mut ConsumerState) {
  // grbl's error:8 gate, applied per chunk: a `$PBX` import mutates the whole settings blob, so it is accepted
  // only while Idle (or Alarmed). A mid-import rejection also drops the partial reassembly — the import failed
  // as a unit, and keeping the prefix would splice a stale sequence onto a later retry.
  if settings_write_blocked().await {
    state.pb.reset();
    error(ERROR_NOT_IDLE).await;
    return;
  }
  let Ok(hex) = core::str::from_utf8(hex) else {
    state.pb.reset();
    error(SettingError::BadValue.code()).await;
    return;
  };
  match state.pb.accept_hex(hex.trim()) {
    PbChunkResult::NeedMore => ack().await,
    PbChunkResult::Complete(new_settings) => {
      {
        let mut guard = SETTINGS.lock().await;
        *guard = Some(new_settings);
      }
      // Apply to RAM and mark dirty; the consumer flushes once the burst drains (or on soft reset / safety
      // interval), so a `$PBX` import that arrives split across many lines is persisted with a SINGLE flash
      // append rather than one per chunk. Keep the `$22` homing mirror in sync so the boot-lock / `$H` /
      // soft-reset decisions reflect a bulk import that changed it. Acknowledge immediately — the in-RAM value
      // already applied.
      HOMING_ENABLED.store(new_settings.homing_enabled(), Ordering::Relaxed);
      // Keep the `$21` hard-limit-enable mirror in sync too so the executor's hard-limit check reflects a bulk
      // import that changed it (DOC-06).
      HARD_LIMITS_ENABLED.store(new_settings.hard_limits_enabled(), Ordering::Relaxed);
      LIMIT_INVERT.store(new_settings.limit_invert, Ordering::Relaxed);
      // Keep the `$26` debounce mirror in sync so a bulk import that retunes it takes effect on the executor's
      // limit-edge resample without a reboot (DOC-06).
      LIMIT_DEBOUNCE_MS.store(new_settings.homing_debounce_ms, Ordering::Relaxed);
      // Phase F: a bulk import can change `$481`; re-seed the live auto-report cadence and wake the task so the
      // imported interval (enable/disable/retune) takes effect immediately, no reboot.
      AUTO_REPORT_INTERVAL_MS.store(new_settings.auto_report_interval_ms(), Ordering::Relaxed);
      AUTO_REPORT_WAKE.signal(());
      // A bulk import can change `$100`/`$10`/`$110`; refresh the cached status config (Finding #14) AND push
      // the planner-affecting fields into the live planner, exactly as the `$x=val` path does — the error:8
      // gate above guarantees the machine is idle, so the swap is safe.
      refresh_status_cfg().await;
      refresh_planner_config().await;
      mark_settings_dirty();
      ack().await;
    }
    PbChunkResult::Error => error(SettingError::BadValue.code()).await,
  }
}

/// Emit the `$#` NGC-parameters block: one bracket line (`[G54:..]`..`[PRB:..]`) per index, rendered from a
/// [`CoordinateReport`] snapshot of the live coordinate model. The closing `ok` is emitted by the caller. Each
/// line is enqueued through the single USB writer, preserving in-order delivery (exactly the `$$` pattern).
async fn dump_ngc_parameters() {
  let report = coordinate_report();
  for index in 0..NGC_PARAMETER_LINES {
    let mut line = Response::new();
    if ResponseWriter::ngc_parameter_line(&mut line, &report, index) {
      enqueue(line).await;
    }
  }
}

/// Build the [`CoordinateReport`] the `$#` formatter renders from the live coordinate model and the last-probe
/// result (Phase C). The `[PRB:]` line now carries the real triggered machine position and contact flag from
/// [`LAST_PROBE`]; it is `[PRB:0,0,0:0]` (the never-probed default) until the first `G38.x` cycle runs.
fn coordinate_report() -> CoordinateReport {
  let coords = coordinates();
  let mut wcs = [[0.0f32; AXES]; 6];
  for (index, slot) in wcs.iter_mut().enumerate() {
    if let Some(offset) = coords.wcs_offset(index) {
      *slot = offset;
    }
  }
  let predefined = [
    coords.predefined(0).unwrap_or([0.0; AXES]),
    coords.predefined(1).unwrap_or([0.0; AXES]),
  ];
  // Phase C: the `[PRB:]` line reflects the real last-probe result (machine position + contact flag), replacing
  // the Phase-B zeros/flag-0 stub. It is `[PRB:0,0,0:0]` until the first probe runs (the never-probed default).
  let probe = last_probe();
  CoordinateReport {
    wcs,
    predefined,
    g92: coords.g92_offset(),
    tlo: coords.tlo(),
    probe: probe.position_mm,
    probe_success: probe.success,
  }
}

/// Handle `$RST=#` (zero coordinate data). Clear ALL coordinate offsets — G54-G59 → 0, G28/G30 → 0, G92/TLO →
/// identity, active WCS → G54 — persist the (now-default) persistent record immediately, push the zero WCO into
/// the planner, reset the WCO refresh cadence so the next `?` re-emits `WCO:`, and `ok`. grbl auto-resets after
/// a `$RST`; coordinate data has no planner-modal coupling beyond the WCO, so a full soft reset is not required
/// here (mirroring how a `$RST=#` only touches parameters, not `$$` settings or modal state).
async fn handle_restore_params(flash: &'static SharedFlash) {
  // grbl's error:8 gate: `$RST=#` wipes and persists the coordinate parameters, Idle/Alarm only.
  if settings_write_blocked().await {
    error(ERROR_NOT_IDLE).await;
    return;
  }
  clear_and_persist_coordinates(flash).await;
  ack().await;
}

/// Clear ALL coordinate offsets and persist the cleared (default) record immediately, then push the zero WCO
/// into the planner and reset the `WCO:` refresh cadence. Shared by `$RST=#` ([`handle_restore_params`]) and
/// `$RST=*` ([`handle_restore_all`], Finding #7) so a "restore all" wipes G54-G59 / G28 / G30 exactly as
/// `$RST=#` does (the `$RST=*` doc says it should), rather than touching only the `$$` settings. Emits NO `ok`:
/// the caller decides the acknowledgment (`$RST=#` acks directly; `$RST=*` is acked by its soft reset).
async fn clear_and_persist_coordinates(flash: &'static SharedFlash) {
  let mut coords = coordinates();
  coords.clear_all();
  set_coordinates(coords);
  push_wco_to_planner().await;
  // Persist the cleared (default) record immediately — a restore is explicit and infrequent, so the coalesced
  // deferral is unnecessary. A failed persist is logged and swallowed (the in-RAM value already applies),
  // matching the settings restore policy.
  {
    let mut store = FlashRecordStore::coordinates(flash);
    if coords::store_coordinates(&mut store, &coords.persistent()).await.is_err() {
      #[cfg(feature = "defmt")]
      defmt::warn!("coordinates: failed to persist cleared params");
    }
  }
  COORDINATES_DIRTY.store(false, Ordering::Release);
  reset_wco_reporter();
}

/// Handle `$RST=*` (restore EVERYTHING): restore the `$$` settings to defaults AND clear+persist the coordinate
/// parameters (G54-G59 / G28 / G30), so a "restore all" wipes both — its own grblHAL doc says it should, and the
/// old routing wiped only the settings (Finding #7). The coordinate clear is reused from the `$RST=#` path. The
/// settings restore runs the soft-reset pipeline rebuild (which is the acknowledgment, as for `$RST=$`), so the
/// coordinates are cleared FIRST and the reset's `push_wco_to_planner` then resolves against the now-zero WCS.
async fn handle_restore_all(parser: &mut Parser, state: &mut ConsumerState, flash: &'static SharedFlash) {
  // grbl's error:8 gate, checked BEFORE the coordinate wipe so a rejected `$RST=*` is all-or-nothing (the
  // inner `handle_restore_settings` re-checks, but by then the coordinates would already be gone).
  if settings_write_blocked().await {
    error(ERROR_NOT_IDLE).await;
    return;
  }
  clear_and_persist_coordinates(flash).await;
  // Restore the `$$` settings to defaults and rebuild the pipeline (its banner is the `$RST=*` acknowledgment).
  // `handle_restore_settings` re-pushes the WCO into the freshly-rebuilt planner via `reset_pipeline`, so the
  // zeroed coordinate model is what the post-restore planner uses.
  handle_restore_settings(parser, state, flash).await;
}

/// Emit the `$$` settings dump: one `$n=value` line (CRLF-terminated) per number in
/// [`settings::SETTING_NUMBERS`], rendered from a snapshot of the live settings. The closing `ok` is emitted
/// by the caller. Each line is enqueued through the single USB writer, preserving in-order delivery.
async fn dump_settings() {
  let snapshot = settings_snapshot().await;
  for &n in settings::SETTING_NUMBERS {
    let mut line = Response::new();
    // Render `$n=value`, then append the line terminator; a formatting/capacity failure simply skips the line
    // (it never trips — `Response` is sized well above the longest setting line).
    if snapshot.write_setting_line(n, &mut line) && line.push_str("\r\n").is_ok() {
      enqueue(line).await;
    }
  }
}

/// Emit the `$ES` settings enumeration: one `[SETTING:<id>|<group>|<name>|<unit>|<datatype>|<format>|<min>|
/// <max>]` line per `$n` number (Phase F), driven off the SAME [`settings::SETTING_NUMBERS`] authority `$$`
/// uses, so the enumeration and the live dump describe the same settings. The closing `ok` is the caller's.
/// Each line is built in its own [`Response`] and enqueued through the single USB writer with blocking `send`
/// (back-pressure), never assembled into one giant buffer — the block can be long over the 1024-byte pipe.
async fn enumerate_settings() {
  enqueue_lines(settings::SETTING_NUMBERS.iter().copied(), |n, line| {
    Settings::write_setting_enumeration(n, line)
  })
  .await;
}

/// Stream one enqueued [`Response`] line per item — the shared `$E*` enumeration skeleton (C2). For each item it
/// builds a fresh line, invokes `fmt` to render it, and enqueues it through the single USB writer ONLY when `fmt`
/// reports success, so a formatting/capacity failure skips that one line exactly as the hand-written loops did.
/// Iteration order and the skip-on-failure semantics are byte-for-byte those of the per-enumeration loops it
/// replaces; the closing `ok` remains the caller's. NOT used for `$$` — [`dump_settings`] snapshots the live
/// settings first and appends its own `\r\n`, a different shape the nemesis flagged, so it keeps its own loop.
async fn enqueue_lines<T>(items: impl IntoIterator<Item = T>, mut fmt: impl FnMut(T, &mut Response) -> bool) {
  for item in items {
    let mut line = Response::new();
    if fmt(item, &mut line) {
      enqueue(line).await;
    }
  }
}

/// Emit the `$EG` setting-group enumeration: one `[SETTINGGROUP:<id>|<parent>|<name>]` line per group (Phase
/// F), covering every group referenced by a setting's metadata. The closing `ok` is the caller's; each line is
/// enqueued through the single USB writer with back-pressure.
async fn enumerate_setting_groups() {
  enqueue_lines(0..settings::SETTING_GROUP_COUNT, |index, line| settings::write_setting_group(index, line)).await;
}

/// Emit the `$EE` error-code enumeration: one `[ERRORCODE:<id>|<name>|<description>]` line per code in the
/// single [`ERROR_CODES`] authority (Phase F) — every `error:N` this firmware can return. The closing `ok` is
/// the caller's; each line is enqueued through the single USB writer with back-pressure.
async fn enumerate_error_codes() {
  enqueue_lines(ERROR_CODES, |code, line| ResponseWriter::error_code_line(line, code).is_ok()).await;
}

/// Emit the `$EA` alarm-code enumeration: one `[ALARMCODE:<id>|<name>|<description>]` line per code in
/// [`AlarmCode::ALL`] (Phase F) — every alarm this firmware can raise. The closing `ok` is the caller's; each
/// line is enqueued through the single USB writer with back-pressure.
async fn enumerate_alarm_codes() {
  enqueue_lines(AlarmCode::ALL.iter().copied(), |code, line| ResponseWriter::alarm_code_line(line, code).is_ok()).await;
}

/// Emit the single `[SETTINGDESCR:<id>|<description>]` line for `$SED=<id>` (Phase F) when `id` is a known
/// setting; a no-op for an unknown id (the caller still `ok`s the recognized `$SED` command). Enqueued through
/// the single USB writer.
async fn send_setting_description(id: u16) {
  enqueue_lines(core::iter::once(id), |id, line| Settings::write_setting_description(id, line)).await;
}

/// Handle a `$<number>=<value>` setting write. The [`SystemCommand`] classifier already guaranteed the body
/// is `<digits>=<value>`, so the only failures here are a value out of range / wrong type (→ `error:N`) — a
/// malformed body NEVER reaches this path (it classifies to `Unknown` → `error:3`), so there is no lenient
/// fake-ack. A valid write is applied to the live [`SETTINGS`] and marked DIRTY for the coalesced flush; the
/// flash append is deferred to the consumer's burst-boundary flush, so a `$$`-bulk restore is one flash write.
async fn write_setting_command(body: &[u8]) {
  // grbl's error:8 gate: a `$n=val` is accepted only while Idle (or Alarmed) — see `settings_write_blocked`.
  if settings_write_blocked().await {
    error(ERROR_NOT_IDLE).await;
    return;
  }
  // The classifier ensured `<digits>=<value>`; re-parse to extract the number/value for the setter. A parse
  // failure here would be an internal inconsistency with the classifier, surfaced as a bad-value error rather
  // than a fabricated `ok` (it cannot occur for a body the classifier accepted).
  let parsed = core::str::from_utf8(body)
    .ok()
    .and_then(|text| text.split_once('='))
    .and_then(|(number, value)| number.trim().parse::<u16>().ok().map(|n| (n, value.trim())));
  let Some((n, value)) = parsed else {
    error(SettingError::BadValue.code()).await;
    return;
  };

  // Apply under the lock; the validated change goes to the in-RAM settings, the flush persists it later.
  let outcome = {
    let mut guard = SETTINGS.lock().await;
    match guard.as_mut() {
      Some(settings) => settings.set_command(n, value),
      // Unseeded settings is an init wiring bug (unreachable after boot); reject as an unknown setting.
      None => Err(SettingError::UnknownSetting),
    }
  };

  match outcome {
    Ok(()) => {
      // Apply-and-mark: the in-RAM value already applied, so acknowledge immediately. The coalesced flush
      // persists it once the burst drains (or on soft reset / safety interval). Deferring avoids re-appending
      // the WHOLE settings blob to flash for every `$n=val` line in a bulk restore (grbl semantics — a
      // stalled `ok` would wedge a character-counting sender; here the `ok` never waits on flash at all). Keep
      // the `$22` homing mirror in sync so a `$22=` write changes the boot-lock / `$H` / soft-reset decisions.
      if n == 22 {
        HOMING_ENABLED.store(settings_snapshot().await.homing_enabled(), Ordering::Relaxed);
      }
      // A `$21=` write changes the hard-limit enable live; mirror it so the executor's hard-limit check reflects
      // it without a reboot (DOC-06).
      if n == 21 {
        HARD_LIMITS_ENABLED.store(settings_snapshot().await.hard_limits_enabled(), Ordering::Relaxed);
      }
      // A `$5=` write changes the limit-pin invert live; mirror it for the executor's sampling (DOC-06).
      if n == 5 {
        LIMIT_INVERT.store(settings_snapshot().await.limit_invert, Ordering::Relaxed);
      }
      // A `$26=` write retunes the limit debounce live; mirror it for the executor's edge resample (DOC-06).
      if n == 26 {
        LIMIT_DEBOUNCE_MS.store(settings_snapshot().await.homing_debounce_ms, Ordering::Relaxed);
      }
      // Phase F: a `$481=` write retunes the auto-report cadence live (no reboot). Mirror the CLAMPED interval and
      // wake the auto-report task so an enable takes effect immediately and a disable/retune is picked up at once.
      if n == 481 {
        let interval = settings_snapshot().await.auto_report_interval_ms();
        AUTO_REPORT_INTERVAL_MS.store(interval, Ordering::Relaxed);
        AUTO_REPORT_WAKE.signal(());
      }
      // Refresh the cached status config so a `$100`/`$10`/`$110` change is in the next report (Finding #14).
      refresh_status_cfg().await;
      // Push the planner-affecting settings (`$100–$102` steps/mm, rates, accel, junction, soft limits) into
      // the live planner too, so the NEXT motion line uses them — grbl applies these immediately, and the
      // status conversion above already did. Safe: the error:8 gate above guarantees the machine is idle.
      refresh_planner_config().await;
      mark_settings_dirty();
      ack().await;
    }
    Err(error_code) => error(error_code.code()).await,
  }
}

/// Queue the `$I`/`$I+` build-info lines. Also REPLAYS a pending crash report (`[MSG:CRASH ...]`) here: `$I` is
/// the readiness probe a host sends right after connecting, so a sender that reconnected too late to catch the
/// boot-time emission still receives the post-mortem breadcrumb. Consumed (sent at most once more).
async fn send_build_info(extended: bool) {
  let mut s = Response::new();
  if ResponseWriter::build_info(&mut s, extended).is_ok() {
    enqueue(s).await;
  }
  // `$I+` only: the live `[DRIVER:]` TMC2209 bus-health line(s). Sourced from the lock-free snapshot the TMC
  // manager publishes after its init pass, so this never touches the half-duplex UART from the comms task.
  if extended {
    let mut driver = Response::new();
    if ResponseWriter::driver_info(&mut driver, &crate::tmc::driver_status()).is_ok() {
      enqueue(driver).await;
    }
    // The per-node `IOIN` presence-read diagnostic, only when the init pass saw at least one node not cleanly
    // respond (a healthy bus adds no line). Localizes a whole-bus failure (`[DRIVER:.. X:-- Y:-- Z:-- A:--]`)
    // through every layer without a scope: echo/reply timeout (pin-matrix vs silent driver), `crc` (framing),
    // or `ver:0xNN` (wrong version). A `crc` node also gets a `[MSG:TMC-IOIN …]` raw-bytes framing dump.
    let probe = crate::tmc::driver_probe();
    if probe.should_report() {
      let mut diag = Response::new();
      if ResponseWriter::driver_probe(&mut diag, &probe).is_ok() {
        enqueue(diag).await;
      }
    }
    if let Some(capture) = probe.raw_ioin {
      let mut raw = Response::new();
      if ResponseWriter::driver_ioin_raw(&mut raw, &capture).is_ok() {
        enqueue(raw).await;
      }
    }
    // The boot loopback self-test line (once it has run): proves the MCU TX+RX path + line levels via the shared
    // GPIO9 self-echo, so a scope-free bench can tell an MCU-side fault from a driver-side one.
    let loopback = crate::tmc::loopback_report();
    if loopback.ran {
      let mut lb = Response::new();
      if ResponseWriter::loopback(&mut lb, &loopback).is_ok() {
        enqueue(lb).await;
      }
    }
    // The per-axis bus margin meter (fail/total exchanges since boot): grades the marginal single-wire bus
    // numerically, so a bench A/B (pull-up, bus pad, baud, wiring) is a failure-rate comparison over a fixed
    // interval instead of an eyeballed `ok`/`--` flicker. Gated on any_attempted so a fresh boot (before the
    // first poll round) adds no all-zero line.
    let stats = crate::tmc::bus_stats();
    if stats.any_attempted() {
      let mut meter = Response::new();
      if ResponseWriter::tmc_bus_stats(&mut meter, &stats).is_ok() {
        enqueue(meter).await;
      }
    }
  }
  // OBSERVE-ONLY air-run readout (task #22, gcode-chunk-skip): the four diagnostic counters on every `$I`, so a host
  // polling `$I` periodically sees them advance and diverge live — even on a partial run with no wedge. Emitted
  // UNCONDITIONALLY (unlike `rec=`) so a value of 0 is itself informative (`drop=0` ⇒ the overflow path did NOT fire
  // this run). On a clean stream `lines == acks`; `drop>0` at the point `lines`/`acks`/`exec` diverge pins the skip.
  if let Some(line) = format_skip_probes() {
    enqueue(line).await;
  }
  // Replay the reset-reason line FIRST (it names which dog fired — the frame for any crash that follows), then the
  // crash report. Capture build only; a no-op (compiled out) in production.
  #[cfg(feature = "capture-reset")]
  if let Some(line) = take_pending_reset_report() {
    enqueue(line).await;
  }
  for line in take_pending_crash_report() {
    enqueue(line).await;
  }
}

/// Render the `[MSG:SKIP ...]` air-run probe line (task #22 §15). ALL counters are OBSERVE-ONLY (zero behavior
/// change). The §15 PRIMARY field is `trunc` (silently-swallowed mid-block RMT truncations) — it is the one that
/// catches the leading silent-skip mechanism, which `exec` CANNOT (a truncated block still bumps `exec`). Fields:
/// - `drop` = RX_PIPE-overflow dropped bytes (should be 0 — skirnireng proved the host can't over-send; >0 re-opens
///   the host/byte path).
/// - `lines`/`cons` = lines FRAMED / CONSUMED by the consumer; `acks` = terminal `ok`/`error:N` emitted; `exec` =
///   motion blocks executed. The chain cross-check: `acks > cons` ⇒ firmware OVER-ACK (the host then over-sends);
///   `acks > exec` (beyond the non-motion lines) ⇒ a whole-block drop in the dual-core path.
/// - `trunc` = total swallowed `run_block` truncations, split by source: `twait` (RMT wait-error arm — the prime
///   recurring suspect), `ttx` (transmit-start arm — channel lost), `tlong` (burst-too-long — an encoder bug); plus
///   `taxis` = the last truncation's axis+1 (4 ⇒ the axis-3 stale-scratch path). `trunc > 0` tied to a visible gap
///   confirms the §15 mid-block-RMT-truncation mechanism.
/// Pure formatting; no I/O. The line fits [`RESPONSE_CAPACITY`] (160) comfortably.
fn format_skip_probes() -> Option<Response> {
  use core::fmt::Write as _;
  let mut inner: heapless::String<144> = heapless::String::new();
  // The §15 truncation counters live in FREE-RUNNING RTC_FAST (so they SURVIVE the K-escape reset that fires on a
  // usb_tx wedge); read them back here for the `$I` line. The chain counters (drop/lines/cons/acks/exec) are plain
  // `.bss` atomics — they DO zero on a reset, but they are not the §15 measurement (they cross-check pipeline stages
  // within a single boot, which is sufficient for them).
  let (trunc, twait, ttx, tlong, taxis) = crate::crash::read_run_block_truncated();
  write!(
    inner,
    "SKIP drop={} lines={} cons={} acks={} exec={} trunc={} twait={} ttx={} tlong={} taxis={}",
    RX_PIPE_OVERFLOW.load(Ordering::Relaxed),
    LINES_FRAMED.load(Ordering::Relaxed),
    LINES_CONSUMED.load(Ordering::Relaxed),
    ACKS_EMITTED.load(Ordering::Relaxed),
    BLOCKS_EXECUTED.load(Ordering::Relaxed),
    trunc,
    twait,
    ttx,
    tlong,
    taxis,
  )
  .ok()?;
  let mut out = Response::new();
  ResponseWriter::message(&mut out, inner.as_str()).ok()?;
  Some(out)
}

/// Queue the `$G` parser-state line, rendered from the consumer's live parser modal state so the host sees
/// the real motion/units/distance/feed/spindle words rather than a constant default.
async fn send_parser_state(parser: &Parser) {
  let mut s = Response::new();
  if ResponseWriter::parser_state(&mut s, &parser_snapshot(parser.state())).is_ok() {
    enqueue(s).await;
  }
}

/// Translate the gcode parser's [`ModalState`] into the protocol layer's [`ParserSnapshot`] for `$G`
/// formatting. This is the one place the firmware bin bridges the parser's modal enums to the protocol's
/// rendering enums, keeping `firmware-core::protocol` free of any GCode-parsing coupling.
fn parser_snapshot(state: &ModalState) -> ParserSnapshot {
  ParserSnapshot {
    motion: match state.motion {
      MotionMode::Rapid => ParserMotion::Rapid,
      MotionMode::Linear => ParserMotion::Linear,
      MotionMode::ArcCw => ParserMotion::ArcCw,
      MotionMode::ArcCcw => ParserMotion::ArcCcw,
    },
    units: match state.units {
      GcodeUnits::Inch => ParserUnits::Inch,
      GcodeUnits::Millimeter => ParserUnits::Millimeter,
    },
    distance: match state.distance {
      GcodeDistance::Absolute => ParserDistance::Absolute,
      GcodeDistance::Incremental => ParserDistance::Incremental,
    },
    feed_mode: match state.feed_mode {
      GcodeFeedMode::InverseTime => ParserFeedMode::InverseTime,
      GcodeFeedMode::UnitsPerMin => ParserFeedMode::UnitsPerMin,
    },
    wcs: state.wcs,
    tlo_active: state.tlo_active,
    feed: state.feed,
    spindle: match state.spindle {
      SpindleState::Clockwise => ParserSpindle::Clockwise,
      SpindleState::CounterClockwise => ParserSpindle::CounterClockwise,
      SpindleState::Stop => ParserSpindle::Stop,
    },
    // The parser tracks spindle speed as f32 RPM; the snapshot reports whole RPM (grbl's `$G` S word).
    spindle_rpm: state.spindle_speed.max(0.0) as u16,
    plane: match state.plane {
      firmware_core::gcode::Plane::XY => ParserPlane::XY,
      firmware_core::gcode::Plane::ZX => ParserPlane::ZX,
      firmware_core::gcode::Plane::YZ => ParserPlane::YZ,
    },
    coolant: ParserCoolant { mist: state.coolant.mist, flood: state.coolant.flood },
    // The CURRENT (active) tool, committed by M6; reported as `T<n>` in `$G` (`T0` = none).
    tool: state.current_tool,
  }
}
