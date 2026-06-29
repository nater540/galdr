---
name: firmware-ack-tx-path
description: Firmware emits exactly one response per line through a single RESPONSE channel→usb_tx; CRLF collapse rules out framer double-ok; $G is pure core-0
metadata:
  type: project
---

Firmware ack/TX path (`crates/firmware/src/comms.rs`):
- Single `RESPONSE: Channel<_, Response, RESPONSE_QUEUE_DEPTH=8>` drained by ONE `usb_tx`
  task. `ack()`/`error()`/`error_bare()` each enqueue exactly one terminal response.
- Line framing collapses `CR`/`LF`/`CRLF`/`LFCR` (even split across feeds) into ONE
  terminator in `crates/firmware-core/src/protocol.rs` `LineReader`/`StreamEngine`
  (host-tested). So `$G\r\n` cannot produce a double-`ok` from the framer — the legacy
  double-ok bug is structurally excluded. A spurious ack is therefore NOT a CRLF artifact.
- `$G` path: `SystemCommand::ParserState` → `send_parser_state(parser)` (enqueues `[GC:...]`)
  → `ack()`. Pure core-0 comms; NO motion, NO RMT, NO BlockQueue. `$G` cannot itself reach
  the Mode-A RMT wedge.
- Crash-report replay (`take_pending_crash_report`, fired on first `$I` and first `?`) emits
  only `[MSG:CRASH ...]` BRACKET messages, never `ok`/`error` — so the replay path is NOT a
  source of the spurious-ack fault.

Implication: a "spurious ok" during/after a `$G` is almost certainly NOT a firmware
over-emission on the `$G` itself. Either (a) a real, correctly-counted `ok` buffered across
an Unresponsive-disconnect boundary that the host mis-attributes ([[skirnir-spurious-ack-accounting]]),
or (b) the `$G` exposed a pre-existing core-1 wedge from a prior stream (the `$G`'s own ok
never came; the stray ack is leftover from before).

See [[skirnir-spurious-ack-accounting]], [[streaming-lockup-doc]].
