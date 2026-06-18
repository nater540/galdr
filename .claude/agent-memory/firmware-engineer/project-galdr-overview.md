---
name: project-galdr-overview
description: What Galdr is — ESP32-S3 grblHAL CNC milling firmware plus a Linux egui GCode sender, with docs/ as authoritative spec.
metadata:
  type: project
---

Galdr is a CNC PCB milling system in two halves, both currently empty `fn main` scaffolds as of 2026-06-16.

- `crates/firmware` — grblHAL-compatible 3-axis CNC firmware for an ESP32-S3, `no_std` Rust on esp-hal 1.0 + Embassy.
- `crates/skirnir` — native Linux GCode sender (host app) streaming GCode over USB CDC serial; UI is egui (eframe),
  serial/streaming engine isolated into a framework-agnostic module on `tokio-serial`.

**Why:** PCB isolation milling needs deterministic, grblHAL-compatible motion control and a sender that reliably
injects real-time bytes (host-side character counting against the advertised RX buffer).

**How to apply:** `docs/` is the authoritative specification — read the relevant doc before implementing a subsystem.
Doc index: `00-architecture.md` (full firmware spec DOC-00..09), `gcode-streaming.md` (grblHAL protocol contract,
shared firmware<->skirnir), `tlo-offsets.md` (Z-probe / TLO workflow), `native-app.md` (skirnir design + UI choice).
Doc crate names (`pcb-mill-fw`, `cnc-core`, `gcode`, etc.) are placeholders; real crates are `firmware`/`skirnir`.
See [[project-build-constraints]] and [[project-hardware-map]].
