---
name: project-protocol-contracts
description: Galdr grblHAL streaming layer (DOC-08) design decisions — sans-io StreamEngine shape, one-ok-per-line model, error-hold, and the firmware-bin comms task wiring.
metadata:
  type: project
---

The grblHAL streaming protocol lives in `crates/firmware-core/src/protocol.rs` (Stage-1 complete,
implemented 2026-06-16, sans-io, no_std, host-tested 32 tests). The esp-hal USB wiring is
`crates/firmware/src/comms.rs`. Decisions a future author needs that are NOT obvious from the code:

**protocol.rs is PURE / sans-io and does NOT parse GCode.** It frames lines + classifies real-time
bytes + holds error state + formats responses. It deliberately does NOT call the gcode parser — that
is the `gcode_parser` task's job behind the `LINE_QUEUE` channel handoff (matches DOC-01). Pieces:
`classify_realtime(byte)->Option<RealtimeCommand>` (printable `?`/`!`/`~`/`0x18` AND grblHAL top-bit
`0x80..0x8C` alias to the same variants; overrides `0x90..0xA4` carry the raw byte); `LineReader`
(CRLF/LFCR collapse incl. split-across-feeds via a PendingTerminator latch + a `line_done` lazy-clear
latch so the returned `&[u8]` stays valid until next `feed`); `StreamEngine` (orchestrator);
`ResponseWriter` (stateless formatters into `heapless::String`); `MachineSnapshot` (Copy, the status
formatter is pure over it — bin fills it from a shared `Mutex<MachineSnapshot>`).

**StreamEngine is now PURE LINE FRAMING ONLY (reduced 2026-06-16, P0/#9 refactor).** It no longer
classifies real-time bytes and no longer holds error state. `EngineEvent` = {None, AcceptLine(&[u8]),
Reject(u8)} — `Realtime` and `Acknowledge` variants DELETED, `StreamState`/`note_line_error()` DELETED.
`ingest(byte)` is just `LineReader::feed` → None / AcceptLine (any completed line INCLUDING blank, as an
empty slice) / Reject(error:15 overflow). Blank lines are forwarded as empty AcceptLine — the consumer
owns the bare `ok` + hold-recovery. `engine.soft_reset()` (drops the partial line) is the only state hook.
Real-time classification is `classify_realtime()` (still a free fn), called by the bin's RX READER HALF
before the framer ever sees a byte. The gcode error-hold lives ONLY in the consumer (see
[[project-consumer-pipeline]]).

**`$G` parser-state now renders LIVE modal state (#3).** `ResponseWriter::parser_state(out, &ParserSnapshot)`
takes a `ParserSnapshot { motion: ParserMotion, units: ParserUnits, distance: ParserDistance, feed: f32,
spindle_rpm: u16 }` (Copy, with `power_on()`). New protocol enums ParserMotion/ParserUnits/ParserDistance
keep protocol decoupled from gcode enums; the bin's `parser_snapshot()` bridges `gcode::ModalState`. Feed
renders minimal-decimal via `write_minimal_f32` (`{:.3}` then trim trailing 0/`.`): F0/F1500/F12.5/F250.25.

**Wire-format constants (must stay truthful, NOW single-sourced):** RX_BUFFER_SIZE=1024,
`BLOCK_BUFFER_SIZE = planner::BLOCK_QUEUE_LEN` (=32, was wrongly 16 — #4), `AXIS_COUNT = planner::AXES` (=3),
MAX_LINE_LEN=256, VERSION="1.1f", ERROR_LINE_OVERFLOW=15. `[OPT:VNMSL,32,1024,3,0]`, idle `Bf:32,1024`.
Banner =
`Grbl 1.1f ['$' for help]\r\n` (grbl-compatible form, not "GrblHAL", for max sender compat; extended
identity via `$I+` → `[FIRMWARE:grblHAL]`). `[OPT:VNMSL,16,1024,3,0]` order = opts,block-buf,rx-buf,
axes,tool-entries. Status `<Idle|MPos:x,y,z|FS:f,s|Bf:b,r>` (Stage 1 minimal; Pn/Ov/WCO are Stage 2).

**comms.rs task topology (core-0 thread-mode) — RX PATH SPLIT 2026-06-16 (P0 #1/#2/#4):** the RX path is
now TWO tasks around a real byte buffer `RX_PIPE: Pipe<CSRawMutex, RX_PIPE_CAPACITY=RX_BUFFER_SIZE=1024>`
(grbl ISR-ring model). `usb_rx` (READER HALF) reads USB bytes, per byte `classify_realtime`→dispatch Signal
NON-BLOCKING, else `RX_PIPE.try_write(&[byte])` (never blocks — pipe sized == advertised RX so a compliant
host can't overrun; overflow byte dropped only if host ignores char-counting). `line_assembler` (LINE half)
drains RX_PIPE ONE byte at a time (cancel-safe), feeds StreamEngine framer, forwards AcceptLine (incl empty)
to LINE_QUEUE via blocking `send().await` (correct back-pressure: fills pipe→reader throttles host). Races
`LINE_RESET` signal to drop the partial line on 0x18. `usb_tx` (single writer, drains `RESPONSE` chan).
`comms_consumer` (real parser→planner, owns error-hold), `status_responder`. **enqueue() now uses BLOCKING
`RESPONSE.send().await` (#1 fix — guaranteed delivery, never drop an `ok`); safe because no response emitter
runs on the realtime path.** Reset banner: best-effort `try_send_banner()` from reader + guaranteed
`send_banner()` from consumer's `reset_pipeline`. USB read error → `Timer::after(5ms)` backoff (#5, no spin).

**Stage 2/3 extension points already reserved (do NOT rebuild the surface):** `MachineState` enum
carries Alarm/Hold/Home/Door/Check/Jog; `RealtimeCommand` carries Stop/SafetyDoor/JogCancel/
FullStatusReport/ToggleAutoReport/Override — all classified, most currently no-op. Next: alarm state
machine, full status elements + `$10` mask, runtime enumerations (`$ES`/`$EE`/`$EA`), `G38.x`/`[PRB:]`
probing, `$481` auto-report.

See [[project-firmware-bringup]], [[project-galdr-overview]], [[project-build-constraints]].
