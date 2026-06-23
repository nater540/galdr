---
name: engine-architecture
description: skirnir streaming engine layout — pure ProtocolCore vs async Driver, channel topology, serial feature gating, deferred work
metadata:
  type: project
---

The skirnir crate is split lib (`src/lib.rs`) + bin (`src/main.rs`). The bin launches the egui GUI by default;
`skirnir --cli <port> [gcode] [baud]` selects the headless streaming smoke loop (behind the `serial` feature).
See [[gui-architecture]] for the egui/eframe layer.

**Layer split.** `src/protocol/` is pure/synchronous (no async, no I/O, no UI): `FlowWindow` (char-count
window), `LineReassembler` (CRLF/LFCR-as-one deframer), `parse_line`/`Response`, `RealtimeCommand`,
`ConnectionState`, and `ProtocolCore` (state machine emitting `Effect`s). `src/engine.rs` is the only async
piece: a `Driver` task that owns a `Transport`, `select!`s inbound reads vs UI commands, feeds parsed
responses + commands into `ProtocolCore`, and drains the returned `Effect`s. All flow-control logic stays in
the pure core; the engine is just the async pump + channel glue, so the contract is tested against the core.

**Channel topology.** `Engine::connect(transport) -> EngineHandle` spawns the driver via `tokio::spawn`.
UI -> engine: `UnboundedSender<Command>` (StreamProgram/SendLine/Realtime/Disconnect). Engine -> UI:
`UnboundedReceiver<Event>` (StateChanged/Response/Progress/Fault/Disconnected). Loop exits emit exactly one
`Event::Disconnected` (None=clean EOF/requested, Some=I/O error).

**Prioritized write loop (load-bearing — fixes the streaming-deadlock bug).** `Effect` has TWO write variants:
`WriteRealtime(Vec<u8>)` (out-of-band `?`/`!`/`~`/`0x18`/overrides — `begin_handshake` probes + `on_realtime`)
and `WriteLine(Vec<u8>)` (counted program/manual lines — `on_send_line` + `release_ready_lines`). Provenance is
in the PURE core so the priority contract is testable. The driver does NOT write inline; `apply_effects` is now
SYNCHRONOUS + INFALLIBLE — it just sorts byte-effects into `Driver.realtime_out: VecDeque<u8>` /
`Driver.line_out: VecDeque<Vec<u8>>` and forwards informational effects. So feeding the core (commands, inbound
acks, poll) can NEVER park the loop. Each iteration, in strict priority: (1) `flush_realtime` writes all queued
realtime bytes, RACED against `command_rx` so a Disconnect mid-flush ends the loop and cancelled bytes are
`requeue_realtime_front`'d (order preserved); (2) if a line is queued, `write_pending_line` writes the front
line RACED against command_rx + status_poll (NOT read — only one transport borrow per `select!`), so a parked
line write under backpressure yields to Stop/Disconnect/poll/realtime; (3) else `select!` over command / poll /
read, `biased` so command+poll sit AHEAD of read (the read arm can no longer starve the `?` poll → DRO stays
live during a hot stream). Teardown does a best-effort `realtime_out` flush (a Stop issued just before Disconnect
still reaches the wire) but NEVER flushes `line_out` (don't push an aborted job). WHY two queues + races instead
of inline await: the old single inline `write_all().await` parked the whole task under serial backpressure
(Stop/Disconnect dead) and, with biased read-first, starved the `?` poll (DRO freeze). grbl contract
(`docs/gcode-streaming.md` §3) says realtime bytes may be sent mid-line, so realtime-before-line is always safe.

**RX buffer.** `DEFAULT_RX_BUFFER = 1024`; refined at runtime from `[OPT:...]` (3rd CSV field) via
`rx_buffer_from_opt`, applied by `ProtocolCore` on a `Response::Message`.

**Stray-ack tolerance — TWO distinct mechanisms in `ProtocolCore` (both gate the `UnexpectedAck` fault in `on_ack`,
checked in this order).** (1) `trailing_acks: usize` — COUNT-bounded, BANNER-scoped only: a boot banner may re-ack
each line that was in flight when the firmware reset (connect race, the `$I`-racing-banner case); `reset_window(true)`
grants exactly `inflight_kinds.len()`. A host reset/stop grants ZERO here (would linger unspent + mask a real over-ack).
(2) `ignore_stray_acks: bool` — BASELINE-bounded latch for the post-Stop/Abort stray-ack STORM: set in the `ProgramStop`
(`0x86`) AND `SoftReset` (`0x18`) arms of `on_realtime` (both empty the window via `reset_window(false)`), it silently
tolerates ANY number of unmatched acks (in-flight count is unknown, and more arrive until the companion firmware fix
lands) while quiescent post-stop. WHY: per the grbl contract the host discards pending acks on a reset — without this,
every Stop/Abort spammed the console with `Fault(UnexpectedAck)`. It is NOT count-bounded; it CLEARS the instant a
baseline is re-established: the next counted line sent (`on_send_line`/`release_ready_lines`, at the `flow.on_line_sent`
site), a banner (the `0x18` reset's own boot banner, in `on_response`), or a session boundary (`on_connected`/
`on_disconnected`). That keeps the genuine-bug guard: a spurious double-ok during ACTIVE streaming still faults because
the latch is long cleared by then. Tests: `a_{program_stop,soft_reset}_silently_tolerates_in_flight_acks_until_a_line_
re_establishes_counting`, `the_soft_reset_banner_clears_the_post_reset_stray_ack_latch`, and the renamed
`a_{program_stop,soft_reset}_does_not_widen_the_banner_trailing_ack_budget` (prove the count-budget stays banner-only).

**In-flight line-kind FIFO (load-bearing invariant).** `ProtocolCore` keeps `inflight_kinds: VecDeque<
InflightKind{Program,Other}>` in LOCK-STEP with `FlowWindow`'s in-flight set. Push a kind on EVERY send:
`Program` in `release_ready_lines`, `Other` in `on_send_line` (manual/jog). `on_ack` pops the oldest kind and
routes: an `Other` ack updates ONLY flow control (never `program_acked`, never the error-hold), a `Program`
ack advances progress and a `Program` `error:N` triggers `clear_program()`+`Error`. This is why a manual/jog
`error:N` no longer nukes a running program. Anything that empties the firmware buffer (soft reset, banner,
disconnect) must call `reset_window()` (NOT bare `flow = FlowWindow::new(...)`) so the kinds FIFO is cleared
too. Program completion (`complete_if_drained`) is gated on `has_program_inflight()`, not `flow.has_inflight()`
— an unrelated manual line in flight must not hold the program "in progress". `StreamProgram` carries
`Arc<[String]>` (shared, not cloned) from the UI.

**Serial feature.** `tokio-serial` is `optional = true`; `[features] default=["serial"]`,
`serial=["dep:tokio-serial"]`. `transport/serial.rs` + the `main.rs` CLI are `#[cfg(feature="serial")]`.
Tests run against `transport/loopback.rs` (in-memory `LoopbackTransport` + `LoopbackController`) and need no
port — `cargo test -p skirnir --no-default-features` passes, proving the gate is clean.

**Testability.** `Transport` trait uses `impl Future` (no boxing). `LoopbackController` injects "firmware"
bytes and captures written bytes; engine integration tests drive scripted exchanges under `#[tokio::test]`.
`LoopbackController::gate_writes()`/`release_writes()` (a shared `WriteGate` Notify/AtomicBool) make
`write_all` PARK until released — the seam for simulating serial backpressure (firmware not draining RX mid-cut).
Default state open, so existing tests are unaffected. Used by the realtime-preemption + Disconnect-while-pending
deadlock tests. Poll-starvation is hard to repro under `start_paused` (an always-ready read busy-loops); the
trick that works is a transport delivering a BURST of immediately-ready reads with NO `.await` between them
(awaiting an already-ready future does not yield to the scheduler) so the engine drains the whole burst before a
read-after-poll loop would ever reach the timer — assert the `?` poll lands BEFORE the burst of line writes.

**Status field parsing.** `src/protocol/status.rs`: `parse_status(body) -> StatusReport` over the verbatim
`<...>` body (`Response::Status` still carries the raw string; the structured parse lives in the reducer/UI,
not the engine). Models MachineState+substate, MPos/WPos + kind, WCO, FS, Ov, Pn, Bf, Ln; ignores unknown
tags. Pure + unit-tested. `Pn:` is kept as the raw `pins: Vec<char>` PLUS a typed decode: `PinState`
(all-bool struct, full grblHAL letter set — X/Y/Z + A/B/C/U/V/W limits, P probe, O probe-disc, D door, R/H/S/E,
L/T/M/F/Q) via `PinState::from_letters` (order-independent, unknown letters ignored, empty ⇒ all-clear) and
`StatusReport::pin_state()` derives it from the vec (single source of truth, can't drift). `any_xyz_limit()` is
the at-a-glance endstop signal. UI: DRO panel `endstop_chips`/`endstop_chip` (views.rs) render 3 tight X/Y/Z
chips (red ALARM_BG when asserted, INSET dim when clear), drawn unconditionally. Only X/Y/Z surfaced for now;
PinState holds the rest so probe/door can join without reopening the parser.

**Reconnect (DONE, host-side):** `src/reconnect.rs` (pure, NOT feature-gated) = `ReconnectConfig` +
`ReconnectPolicy` (exponential `base*factor^attempt` clamped to `max_delay`, bounded `max_attempts`,
`on_connected()` resets, `next_delay()->Option` gives up at budget). Defaults 300ms/×2/5s/6 attempts target the
ESP32-S3 USB re-enumeration on soft reset. The ENGINE driver still just ends on EOF (unchanged) — the SHELL
drives reconnect: `last_endpoint`/`auto_reconnect`/`reconnect`/`reconnect_at` fields (serial-gated). `connect`
arms auto_reconnect + resets policy + delegates to `connect_inner` (shared open); `disconnect` clears the
desire (deliberate teardown never reconnects). `pump_events` flags a drained `Event::Disconnected` →
`on_engine_dropped` schedules `reconnect_at=now+next_delay()` (or gives up); `pump_reconnect` (per-frame
deadline compare, no thread/timer) fires the due open. Policy reset only on `is_connected()` (excludes
Connecting). Repaint scheduler keeps the loop awake while `reconnect_at.is_some()`. See [[settings-and-overrides]].

**Error/alarm code decoding (DONE):** `src/protocol/codes.rs` (pure) decodes `error:N`/`ALARM:N` into name +
description. `CodeText { name, description }` fields are `Cow<'static, str>` (static tables `Cow::Borrowed`, parser
`Cow::Owned`) so the static path is ALLOC-FREE. Two layers: static fallback (`error_text`/`alarm_text` build a
whole CodeText; `error_static`/`alarm_static` return the raw `(&'static str,&'static str)` pair — common grbl
1.1/grblHAL set, generic gloss for unknown so a bare number is NEVER shown) matching the firmware's own `$EE`/`$EA`
text VERBATIM (full-string-equality tests guard drift); plus a runtime `CodeBook` of overrides
(`apply_error`/`apply_alarm`/`clear`, override beats static). CodeBook accessors: `error()`/`alarm()` return a whole
`CodeText` (both fields); `error_name`/`error_description`/`alarm_name`/`alarm_description` return ONE `Cow<'static,
str>` field — borrow on static path, clone only on override — for per-frame callers (banner). GOTCHA fixed: alarm 4
= probe not in expected initial state (already triggered), 5 = no contact within travel — old `badge.rs` had these
INVERTED; ALSO firmware's `$EA` text for 4/5 leads with `"Probe fail. "` (static must include it or banner wording
shifts after Refresh). `badge::alarm_detail`/`error_detail` are DELETED (codes.rs supersedes). `parse_error_code_meta`/
`parse_alarm_code_meta` mirror `parse_setting_meta` over `ERRORCODE:id|name|desc`/`ALARMCODE:...` (already-debracketed
`Response::Message` body; desc may contain `|`, taken via `splitn(3,'|')`). Reducer (`view_state.rs`) holds
`codes: CodeBook`, folds enumeration rows in the `Response::Message` arm + `return`s (skips console, like SETTING),
clears on disconnect. `render_response` is a `&self` METHOD reading `self.codes` via `error_name`/`alarm_name`:
console shows `error:21 — Modal group violation` (name only). Banner (`views.rs alarm_banner`) uses
`alarm_description`/`error_description` (override beats static, borrow on static path — runs every repaint).
ENRICHMENT TRIGGER: `$EE`/`$EA` are sent in `shell.rs::request_settings` beside `$ES`/`$$` (operator "Refresh"),
NOT in the engine handshake (which only sends `$I`) — the static fallback means display works pre-fetch.
DEDUP: firmware (Track 2) pushes a context line `[MSG:error:<n> <name>]` / `[MSG:ALARM:<n> <name>]` right before
each error/alarm. Since skirnir decodes the code itself, that MSG is redundant — `is_redundant_error_annotation`
(free fn in view_state.rs) suppresses it (return early in the `Response::Message` arm). Prefixes are named consts
`FW_ERROR_ANNOTATION_PREFIX = "MSG:error:"` / `FW_ALARM_ANNOTATION_PREFIX = "MSG:ALARM:"` (MUST track firmware's
`ResponseWriter::error_context`/`alarm_context`). TIGHT shape: prefix + ≥1 digit + single space + ≥1 char (name).
Rejects `MSG:error:` (no code), `MSG:error:21` (no name), `MSG:errored sensor` (no digit), generic `[MSG:Pgm End]`.

**Settings sync (DONE, text path):** `$PBX` BINARY bulk channel is `[PB:<hex>]` chunks wrapping a
firmware-core `storage_frame` (MAGIC|ver|len|payload|CRC32) around the galdr-proto `Settings` — skirnir does
NOT depend on firmware-core/galdr-proto, so that frame codec is NOT available; binary `$PBX` decode is
deliberately DEFERRED. The shipped path is the TEXT `$<n>=<value>` channel (`$$` dump + single `$n=val` write,
firmware-validated) enriched by `$ES` `[SETTING:id|group|name|unit|datatype|format|min|max]` enumeration.
Firmware has NO single-setting read (`$<n>` alone → error), so a write re-dumps `$$`. See [[settings-and-overrides]].
