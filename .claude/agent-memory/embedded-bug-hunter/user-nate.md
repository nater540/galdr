---
name: user-nate
description: Nate is the Galdr firmware owner; deep embedded experience; wants evidence-backed root cause, not plausible guesses
metadata:
  type: user
---

Nate (nater540@gmail.com) owns the Galdr ESP32-S3 CNC firmware and the streaming-lockup
investigation. He engages this agent as the deep-investigation specialist and the
firmware-engineer agent as the implementer.

**How to collaborate:**
- He explicitly distinguishes a known-bug match from a new failure mode and does NOT want
  the known root cause assumed. When he hands a fresh capture, he wants same-bug-or-new
  adjudicated with evidence first.
- He values: confirmed-facts / leading-hypothesis+alternatives / single-next-experiment /
  what-remains-UNKNOWN structure, and labelling HYPOTHESIS vs FINDING.
- He owns the bench hardware; repro is via `skirnir --cli <port> <gcode>` (timeout-bounded),
  never ad-hoc cat/printf to the port. Board at /dev/cu.usbmodem31101 over UsbSerialJtag,
  RX buffer 1024.

See [[streaming-lockup-doc]].
