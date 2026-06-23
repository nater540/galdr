---
name: graceful-program-stop
description: 0x86 graceful program-stop vs 0x18 hard soft-reset; the two-control Stop/Abort UI split and where each lives
metadata:
  type: project
---

Skirnir has TWO distinct stop controls in the transport group (`views.rs` `transport_group`, made `pub(crate)` for the
`ui_test.rs` harness). This is the user's explicit design choice — keep them separate.

- **Stop** (`■ Stop`, amber `Theme::STATE_HOLD`) = GRACEFUL program stop, real-time byte `0x86`
  (`RealtimeCommand::ProgramStop`). Firmware decelerates to the block boundary, flushes the queue, spindle off,
  overrides→100%, modal→power-on, State→Idle. NO alarm, NO banner, MPos retained, in-flight line gets no `ok`.
  No-op from Idle/Alarm/Check/Sleep/Jog.
- **Abort / E-stop** (`⏹ Abort`, danger-red, set apart from the joined segments) = HARD soft-reset `0x18`
  (`RealtimeCommand::SoftReset`) → `ALARM:3` + re-emitted banner.

**Why:** `0x86` is the everyday clean stop; `0x18` is the panic reset. Separating them stops the routine "stop the
job" button from alarming the controller.

**How to apply:**
- In `core.rs` `on_realtime`, `ProgramStop` and `SoftReset` are SEPARATE arms with identical host bookkeeping
  (`clear_program` + `reset_window(false)` (no trailing-ack tolerance — firmware discards in-flight without re-ack)
  + `Effect::AbortQueued` + leave Streaming/Hold→Idle if connected). The DIFFERENCE is only semantic intent — neither
  takes the banner/alarm path here; the banner/alarm transitions come from `on_response` when the firmware actually
  emits them, which `0x86` never does. Do NOT collapse the two arms; their bytes and meaning differ.
- `TransportGroup` (`badge.rs`) gained `abort_enabled`: true for every state with a transport (incl. `Connecting`),
  false only `Disconnected` — mirrors `ConnectionState::has_transport`. `stop_enabled` stays gated on a ready board
  (off while Connecting). The two diverge while Connecting on purpose.
- Re-Run after a stop needs NO shell change: `start_stream` only clones the `Arc` program; ProgramStop→Idle re-enables
  Run (`for_state(Idle, has_program)`), so Run re-streams from the start. See [[engine-write-cancel-safety]] for the
  AbortQueued line-drop guarantee the loopback test reuses.

Note: firmware's `RealtimeCommand` enum is separate from skirnir's — the `0x86` mapping is defined in skirnir's own
`protocol/realtime.rs`.
