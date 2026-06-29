---
name: skirnir-spurious-ack-accounting
description: Where skirnir's in-flight/ack-tolerance lives and the Unresponsive-disconnect zero-tolerance gap that can fault a benign post-reconnect ack
metadata:
  type: project
---

Skirnir's "received an acknowledgement with no line in flight (spurious ok/error)" fault
(`EngineError::UnexpectedAck`) is raised in `crates/skirnir/src/protocol/flow.rs`
`FlowWindow::on_ack` (pop_front on an empty deque) and gated in
`crates/skirnir/src/protocol/core.rs` `ProtocolCore::on_ack`.

Two ack-tolerance mechanisms exist for the reset/reconnect race:
- `trailing_acks` (count-bounded): granted ONLY on the banner path (`reset_window(true)`),
  sized to the in-flight line count when a boot banner clears the window.
- `ignore_stray_acks` (baseline-bounded latch): set on host-issued `0x18` SoftReset /
  `0x86` ProgramStop; cleared when counting resumes / banner / session boundary.

**The gap (HYPOTHESIS, not yet bench-proven):** an **Unresponsive** disconnect
(`RESPONSE_STALL_TIMEOUT`/`WRITE_STALL_TIMEOUT`, engine.rs) tears down via
`on_disconnected()` → `reset_window(false)` → grants NEITHER tolerance. So any `ok`/`error`
the firmware had already queued for the in-flight line (e.g. `$G`) but not yet delivered —
sitting in the firmware RESPONSE channel (depth 8) / USB-Serial-JTAG FIFO — can be delivered
after the native-USB re-enumeration on the NEXT connection and fault as spurious, because the
fresh `on_connected()` zeroed both tolerances and the new link's window holds only `$I`.

**Why:** the silence timeout is a host-side liveness guess; it does not mean the firmware
emitted nothing. A wedged-then-recovered firmware (or one that was merely slow) can leave a
real, correctly-counted `ok` buffered across the disconnect boundary, which the host then
cannot attribute.

**How to apply:** when a spurious-ack fault follows an Unresponsive disconnect + reconnect,
suspect THIS host-side accounting gap before suspecting a firmware over-ack. Discriminate
with a raw-byte TX log on both sides (does the firmware emit exactly one ok per line?). A
fix direction is to grant a bounded stray-ack tolerance across an Unresponsive teardown the
same way the banner path does — but only after proving the firmware does not double-emit.

See [[firmware-ack-tx-path]], [[skirnir-cli-harness]].
