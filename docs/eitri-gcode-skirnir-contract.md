# eitri-gcode ↔ Skirnir interface contract (preliminary, pin before Phase 4)

Derived from a read of `crates/skirnir/src/`, `crates/cnc-kinematics/src/gcode.rs`,
`crates/firmware-core/src/protocol.rs`, and `docs/gcode-streaming.md`.

## Dialect
- **grblHAL 1.1f**. Unsupported G/M → `error:20`.
  - `cnc-kinematics/src/gcode.rs:887–897`, `firmware-core/src/protocol.rs:195`.

## Arcs — REQUIRED, do NOT pre-linearize
- Emit `G2`/`G3` with **I/J/K center offsets** relative to arc start (grbl IJK form).
  **R-radius form is NOT supported.**
- Plane-matched: G17→I/J, G18→I/K, G19→J/K (`gcode.rs:283–301`).
- At least one offset required or `error:33` (`gcode.rs:1254`, `protocol.rs:221`).
- Firmware owns subdivision via chord tolerance `$12` (default 0.002 mm), `planner.rs:20–21`.
  Eitri emits true arcs; the firmware flattens. Pre-flattening would bake in the wrong tolerance.

## Precision / formatting
- **3 decimal places** on coords/offsets (firmware accepts more; status normalizes to .3).
  Skirnir emits `{:.3}` (`profile.rs:99`); status format `.3` (`protocol.rs:1475`).
- **LF-only** terminator on output (firmware tolerates CR/LF/CRLF/LFCR as one).
- **Max 256 bytes/line** or `error:15` (`protocol.rs:72,75`).
- **No `Nnn` line numbers.** Comments `(...)` and `;` are stripped — safe to include (`gcode.rs:131–151`).
- Whitespace flexible; case-insensitive.

## Feed contract
- `G0` rapid: no F needed (uses `$110/$111/$112`).
- `G1/G2/G3`: **F required.** G94 → units/min; G93 inverse-time → F on EVERY line (not modal) (`gcode.rs:271–272`).
- `G38.x` probe: F required; **G93 forbidden** → `error:22` (`gcode.rs:63–68`).

## Modal / preamble
- Power-on defaults: G0, G90, G21, G17, G94, M5, M9 (preamble optional if these suffice).
- Safe preamble: `G90 G21 G54 G17 G94`.
- Units: G20 (inch, ÷25.4) / G21 (mm). Distance: G90 / G91 (I/J/K always relative to arc start).
- Spindle M3/M4/M5 modal; S sets RPM; M3↔M4 reversal forces M5 + dwell `$393` (default 1.5 s).
- Coolant M7/M8 independent, M9 off.
- End with **M2 or M30** (both reset modal state).

## Supported code list (anything else → error:20)
- **G:** G0 G1 G2 G3 G4 G28 G28.1 G30 G30.1 | G38.2–G38.5 | G10 G43.1 G49 G53 G54–G59 G92 G92.1 |
  G17 G18 G19 G20 G21 G90 G91 G93 G94.  (`gcode.rs:891–1166`)
- **M:** M0 M1 M2 M3 M4 M5 M6 M7 M8 M9 M30.  (`gcode.rs:1118–1166`)

## Streaming / error behavior (Skirnir's job, but constrains the producer)
- Character-counting flow control vs 1024-byte RX buffer (`RX_BUFFER_SIZE`, `protocol.rs:60`).
- One `ok`/`error:N` per line drives flow control.
- grblHAL error-hold: after `error:N`, subsequent lines stay held until soft-reset / empty line / `$`
  (`docs/gcode-streaming.md` §7). So a single bad emitted line halts the whole job.

## Implication for the eitri-gcode postprocessor trait
The default/GRBL-grblHAL postprocessor must: emit IJK arcs (never R, never pre-flatten), F on
cut moves only, 3-decimal coords, LF, ≤256 B lines, no N-numbers, program end M2/M30, and restrict
to the supported code set. This is the concrete target to design the `Postprocessor` hook contract
against in Phase 4 — validate its output byte-for-byte against these rules.
