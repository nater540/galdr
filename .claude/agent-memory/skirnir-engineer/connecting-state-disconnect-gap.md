---
name: connecting-state-disconnect-gap
description: Disconnect UI affordance must gate on has_transport (incl. Connecting), not is_connected, or a stalled connect leaks the serial FD
metadata:
  type: project
---

The toolbar's Disconnect button was gated on `ConnectionState::is_connected()`, which is FALSE for `Connecting`. A
connect that stalled in `Connecting` (common on the ESP32-S3 — it can fail to volunteer readiness; see
[[board-connection-facts]]) left the engine task alive holding an open `tokio_serial::SerialStream`, with no UI way
to send `Command::Disconnect`. `lsof` showed skirnir still holding `/dev/cu.usbmodem*`, blocking espflash.

**Why:** the engine's transport (owned solely by the `Driver`, no clones/Arcs) IS dropped correctly when the task
ends — the bug was purely that the operator had no affordance to end it while `Connecting`. The teardown gate must be
"is a port attached" not "is the board ready".

**How to apply:** UI affordances that release the link must use `ConnectionState::has_transport()` (true for everything
except `Disconnected`), not `is_connected()` (false for `Connecting`). `is_connected()` is for gating *ready-only*
controls (jog/stream/home). The fix added `has_transport()` to `protocol/lifecycle.rs` (with tests) and made the
toolbar show "Cancel" while `Connecting` / "Disconnect" once ready — both push `Intent::Disconnect`. Related:
[[engine-architecture]].
