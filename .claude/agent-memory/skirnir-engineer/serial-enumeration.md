---
name: serial-enumeration
description: skirnir serial port enumeration — macOS cu/tty reality, ESP32-S3 vid/pid populated, ports.rs + probe.rs module split
metadata:
  type: project
---

**What `tokio_serial::available_ports()` actually returns for the Galdr board on macOS** (verified on this Mac
via a throwaway test, 2026-06-17): the ESP32-S3 USB-Serial-JTAG enumerates as BOTH `/dev/cu.usbmodem31101`
(callout) AND `/dev/tty.usbmodem31101` (dialin), with IDENTICAL USB metadata on each. Critically, **vid/pid IS
populated on macOS** for this device: `vid=0x303A` (Espressif), `pid=0x1001`, `manufacturer="Espressif"`,
`product="USB JTAG/serial debug unit"`, `serial="14:C1:9F:DB:8B:7C"`. So VID filtering is viable on macOS here,
not just Linux. Do NOT assume metadata is always present though — the classifier must degrade gracefully (None
vid → still listed, never hidden), since a non-USB port or a permission combo can hide it.

**The cu-preference / classifier / probe live in pure modules (`transport/ports.rs`, `transport/probe.rs`),
NOT feature-gated**, so they unit-test headlessly (`--no-default-features` passes). `transport/serial.rs` (the
only `serial`-gated, tokio-serial-touching file) just maps `SerialPortInfo` → `PortInfo` and calls
`normalize_ports`.

- `ports::PortInfo { path, vid, pid, product, manufacturer, serial }` + `classify(Option<u16>) -> PortClass`
  (Espressif `GALDR_VID=0x303A` → LikelyGaldr; other → Other; None → Unknown/listed). `GALDR_JTAG_PID=0x1001`.
- `prefer_cu(&str)` rewrites `/dev/tty.X`→`/dev/cu.X` (keys on the leaf `tty.` prefix so Linux `ttyACM0`/`ttyUSB0`
  — no `.` after `tty` — pass through untouched). `normalize_ports(Vec<PortInfo>)` dedups cu/tty siblings (keeps
  the metadata-bearing one, presents the cu path), then stable-sorts LikelyGaldr first. `PortInfo::hint()` →
  "likely Galdr — <product>" or the bare product. `available_ports()` now returns `Vec<PortInfo>` (was
  `Vec<String>`); `UiState.ports: Vec<PortInfo>`, `selected_port` stays the `String` path.

**On-demand probe (`probe::probe_grbl<T: Transport>(t, timeout) -> ProbeVerdict`)**: opt-in only, NOT part of
`refresh_ports` — opening the ESP32-S3 toggles DTR/RTS (auto-reset), so a silent sweep could reset a running
board. Sends `?$I\n`, reads via `LineReassembler`+`parse_line`, Confirmed on Status/Banner/`[VER:]`/`[OPT:]`,
NoResponse on `tokio::time::timeout` (`DEFAULT_PROBE_TIMEOUT=500ms`) or EOF, Error on I/O. Tested against
loopback (use `start_paused` for the timeout cases — no real sleep). Real-hardware probe of the board returns
Confirmed. UI: `Intent::IdentifyPort{path}` + an "Identify" toolbar button; the shell runs the probe via
`runtime.spawn` (NOT block_on — would freeze the UI ~500ms) and drains the verdict through a
`std::sync::mpsc::Receiver` polled each frame in `pump_probe()`, refused while connected. See
[[engine-architecture]] / [[gui-architecture]].
