---
name: engine-write-cancel-safety
description: Driver write path is cursor-based & cancel-safe; soft-reset/banner emit Effect::AbortQueued; teardown flush is timeout-bounded
metadata:
  type: project
---

The skirnir streaming Driver (engine.rs) write path is cursor-based for cancel safety, and the Transport trait's
write primitive is `write(&[u8]) -> usize` (one syscall, cancel-safe); `write_all` is a provided default looping
`write` and is NOT cancel-safe (only for teardown/best-effort).

**Why:** tokio `write_all` raced inside a `select!` re-sends already-on-the-wire bytes when the future is dropped
after a partial write under serial backpressure (corrupt/duplicated line, desynced char-count window). Fixed by
tracking `Driver.line_in_flight: Option<(Vec<u8>, usize)>` and `realtime_in_flight` cursors: write the unwritten
tail with a single `transport.write`, advance the cursor by the returned count, retire only at completion. A
pre-empted write resumes from the cursor.

**Effect::AbortQueued** (protocol/core.rs) is emitted by `on_realtime(SoftReset)` AND the banner reaction in
`on_response` (controller reset). The Driver's `apply_effects` clears `line_out` + `line_in_flight` on it, so
program lines released before a Stop never reach the wire AFTER the 0x18 (safety). Feed-hold (`!`) does NOT emit
it — held lines resume on cycle-start. Core stays the source of truth.

**Teardown flush** is bounded by `TEARDOWN_FLUSH_TIMEOUT` (300ms) via `tokio::time::timeout`, so an alive-but-
wedged port can't block `Event::Disconnected` forever and leak the serial FD.

**Loopback test seam:** `LoopbackController::set_partial_write_limit(Some(n))` caps bytes accepted per `write` so
tests can prove a split write puts each byte on the wire exactly once. `gate_writes()`/`release_writes()` still
park/resume writes for backpressure tests.

How to apply: any new hot write arm must use the cursor pattern, never `write_all` inside a `select!`. Any new
"abort the job" path (e.g. a future hard-abort) should emit AbortQueued from the core, not clear queues ad hoc.
See [[engine-architecture]].
