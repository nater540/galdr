# grblHAL & ioSender Tool Length Offsets for NO Touch-Plate Probing — Reference for an ESP32-S3 Rust/Embassy Firmware

## TL;DR
- For a simple Normally-Open (NO) touch plate on a **single-tool PCB mill, you do not need dynamic TLO at all**: the standard, ioSender-native workflow is a two-stage `G38.2`/`G38.3` probe followed by **`G92`** or **`G10 L2 P1`** to set Z work-zero at the copper surface using `probe_Z + plate_thickness`. `G43.1` is only used for multi-tool "reference tool" workflows via grblHAL's controller-side `$TLR`/`$TPW` system commands.
- grblHAL stores TLO as a **session-only (RAM, not NVS)** dynamic value applied to one configured linear axis (Z by default); it folds into the Work Coordinate Offset so that `WPos = MPos − WCO`, where `WCO = (G54..G59) + G92 + G43.1 TLO`. It is reported as `[TLO:z]` in the `$#` parameter report and reflected live in the `|WCO:|` status element.
- Your firmware must: treat the probe as a **dedicated input separate from limits**, honor `$6` invert (set **`$6=1`** for a NO plate), implement `G38.2` (alarm on no-contact), store `[PRB:x,y,z:success]`, implement `G43.1`/`G49` to set/clear the RAM TLO, recompute WCO, and emit `[TLO:]`/`[PRB:]`/`|WCO:|`/`|TLR:|` — clearing TLO on `G49`, on homing of the Z axis, on soft reset (if in motion), and on power cycle.

## Key Findings

### 1. TLO is session-only and folds into the WCO
grblHAL keeps the dynamic tool length offset in the live g-code parser state (`gc_state.tool_length_offset[]`), **not** in NVS/flash. It is explicitly non-persistent — like `G92` and the `G38.2` probe data. From the gnea/grbl v1.1 Commands documentation (which grblHAL inherits): "The non-persistent parameters, which will are not retained when reset or power-cycled, are G92, G43.1 tool length offsets, and the G38.2 probing data." By contrast the `G54..G59` work coordinate systems live in NVS and survive power cycles (and grblHAL also persists `G92` to NVS, a difference from legacy Grbl). grblHAL additionally **clears the TLO on homing** of the relevant linear axis (in lathe mode on X- or Z-homing), via the `grbl.on_homing complete` event.

The TLO is applied by **subtracting** a positive offset from the commanded position on the configured linear axis. grblHAL's config documentation: the tool-length-offset axis "Assumes the spindle is always parallel with the selected axis with the tool oriented toward the negative direction. In other words, a positive tool length offset value is subtracted from the current location." Internally the offset is rolled into the Work Coordinate Offset reported to senders.

### 2. The WPos / MPos / WCO / TLO relationship (correct the user's formula)
The canonical relationship (Grbl 1.1 interface spec, implemented by grblHAL) is:

```
WPos = MPos − WCO
WCO  = (active G54..G59 work offset) + G92 offset + G43.1 tool length offset
```

The Grbl interface doc states it explicitly: "WCO: is simply the sum of the work coordinate system, G92, and G43.1 tool length offsets." Therefore the user's proposed `WPos_Z = MPos_Z − WCO_Z − TLO_Z` **double-counts the TLO**. In grblHAL the TLO is *already part of* WCO, so the correct single-subtraction form is `WPos = MPos − WCO`, with `WCO_Z = G5x_Z + G92_Z + TLO_Z`. Your firmware should compute **one combined WCO per axis** and report it in the `|WCO:|` element; do not subtract TLO a second time.

### 3. G43 vs G43.1 vs G49 in grblHAL
From grblHAL's `gcode.c`: "G43.1 and G49 are always supported, G43 and G43.2 if `grbl.tool_table.n_tools > 0`."
- **`G43.1 Z<value>`** — *dynamic* TLO: applies the value in the block directly to the configured axis. This is the command used for touch-plate / toolsetter probing because the offset is computed at runtime from the probe result. It must be alone on its line (no other-axis motion words) or it errors ("[G43.1 Errors]: Motion command in same line").
- **`G43 H<n>`** — table TLO: looks up tool n in the tool table; only available in builds compiled with a tool table (`n_tools > 0`). Not used for touch-plate probing on a typical hobby PCB mill.
- **`G43.2`** — additive tool offset (tool table only).
- **`G49`** — cancels TLO (sets `tool_offset_mode = ToolLengthOffset_Cancel`, offset → 0).

grblHAL applies G43.1 to a **single configured linear axis** (Z by default, set by the `TOOL_LENGTH_OFFSET_AXIS` compile-time symbol). Sending G43.1 with a non-configured axis word raises **error 37**: "The G43.1 dynamic tool length offset command cannot apply an offset to an axis other than its configured axis. The Machine default axis is the Z-axis." It is **not multi-axis** in normal mill builds — though note point 4 on report formatting.

### 4. The `[TLO:]` report and `$#` parameters
`$#` (the NGC parameter report; in grblHAL the handler is `output_ngc_parameters` in `system.c`, "output offsets, tool table, probing and home position") prints the full offset set, ending with `[TLO:...]` and `[PRB:...]`. A response looks like:

```
[G54:0.000,0.000,0.000]
 … [G59:…]
[G28:…][G30:…]
[G92:0.000,0.000,0.000]
[TLO:0.000]
[PRB:0.000,0.000,0.000:0]
```

The legacy single-axis form is `[TLO:0.000]`. **In grblHAL the `[TLO:]` element can carry all axes depending on compatibility level** — a real grblHAL example (build 1.1f.20210608) shows a three-value form: `[TLO:0.000,0.000,-14.442] [PRB:-293.004,-16.995,-78.005:1]`. The grblHAL wiki notes "TLO report may include offsets for all axes, dependent on compatibility level… Senders parsing this tag should be coded to handle that." **Code your parser to accept 1..N comma-separated values.**

There are **two distinct TLO-related surfaces**:
- The **`[TLO:]` bracket message** appears only in response to `$#`.
- During live operation, a TLO change shows up in the **`|WCO:|` status element** (because TLO is summed into WCO), pushed immediately when WCO changes and intermittently as a refresh.
- grblHAL adds a real-time **`|TLR:<0|1>|`** element reporting tool-length-*reference* status (0 = not set, 1 = set), pushed on change. ioSender lights its "TLO ref'd" indicator from this.

### 5. The `$10` status mask and probe push messages
`$10` is the status-report-options mask. Relevant behaviors:
- A `$10` flag enables pushing the parser-state report on change.
- The probe-coordinates auto-message: "Upon a successful probe cycle, this option provides immediate feedback of the probe coordinates through an automatically generated message. If disabled, users can still access the last probe coordinates through grblHAL's `$#` print parameters command." That message is the `[PRB:x,y,z:success]` line.
- A `$11` flag (bit 11, "Run substates") enables `Run:2` during a probing motion; "this can be used by senders to provide a simple probe protection scheme." ioSender uses this.

### 6. NO vs NC plate and the `$6` setting
- For a **Normally-Open touch plate** (open until the tool touches metal, then closed), set **`$6=1`** (invert probe pin). ioSender's own author documents it (grbl.org, "One sender to rule them all?"): "If you have a contact probe, one that makes a connection when it touches (normally open, NO), invert your probe pin (Probing Section, `$6`). If your probe breaks connection when it touches (normally closed, NC), do not invert your probe pin."
- Mechanism: grblHAL treats the probe pin held high (via internal pull-up) as *not triggered*, and a low/grounded pin as triggered. A NO plate sits open (pulled high) until contact pulls it toward tool-ground, so most touch-plate wiring needs `$6=1`. grblHAL defaults its switch *philosophy* to NC, so **test empirically**: with the plate untouched the `Pn:P` flag must be ABSENT; if `P` shows when untouched, flip `$6`.
- In grblHAL with multiple probes enabled, `$6` becomes a **bitmask** (one invert bit per probe input).
- `$19` disables the probe-pin pull-up — leave it `$19=0` (pull-up enabled) for a passive touch plate; disabling it makes a passive plate non-functional.

### 7. The probe is a dedicated input separate from limits
In grblHAL the probe is its own input, never shared with limit switches. Recent builds "Moved probe and safety door inputs to ioPorts pin pool; if not assigned at compile time they will be free to use by M66 or plugin code." grblHAL also supports **multiple probe inputs** selectable via a `P` word on `G38.x` (`P0` primary probe, `P1` toolsetter, `P2` secondary) in recent builds. For a single NO plate you only need one probe input.

### 8. `[PRB:...:1]` vs `:0` and the G38.2 alarm
- `[PRB:x,y,z:1]` = the last probe **succeeded** (contact made); `:0` = no contact. This success flag is identical for NO and NC plates — it reflects whether contact was detected within travel, not plate polarity.
- **`G38.2`** = "probe toward, stop on contact, **error if no contact**." If the tool fails to contact within the programmed travel, grblHAL raises **`ALARM:5`**: "Probe fail. Probe did not contact the workpiece within the programmed travel for G38.2 and G38.4." This is the correct command for a touch plate where contact is required.
- `G38.3` = probe toward, **no error** if no contact (the sender handles failure). ioSender's console logs show it actually emits `G38.3` for its automated probes and does its own NaN/`ProbePosition` failure check.
- `ALARM:4` ("Probe fail. Probe is not in the expected initial state…") fires if the probe already reads triggered before the move starts — usually wrong `$6` polarity or an already-shorted plate.
- **`G38.4`/`G38.5`** = probe **away** (stop on loss of contact). Used in edge-of-contact / re-touch strategies, **not** needed for a simple NO Z-probe — use a plain `G0`/`G1` retract instead.

### 9. PCB-milling-specific workflow (single tool)
For isolation routing the simplest, most robust flow:
1. Load board, jog to XY origin, set X/Y zero (`G10 L2 P1 X0 Y0` or `G92`).
2. Place the NO touch plate on the copper; clip the croc lead to the tool/spindle.
3. Probe Z down with `G38.2`; on contact read `[PRB:…:1]`.
4. Set Z work-zero so the copper top = Z0: the work-Z value at the probed point = `plate_thickness` (ioSender computes `WorkpieceHeight + TouchPlateHeight`, where WorkpieceHeight = 0 when zeroing on the top surface).
5. Remove the plate, run the job. Because PCB copper is uneven, most users then add **height-map / autolevel** probing (ioSender's Height Map tab) using the **tool itself** as the probe against the copper — connect probe-ground to the tool and the other input to the copper surface.

This single-tool re-zero approach means **TLO/G43.1 is unnecessary** — you re-establish Z0 at the surface each time. A crocodile-clip touch plate is a thin conductive plate whose **exact thickness must be measured with calipers** and entered as the plate height (do not assume a value; commodity Z-probe pucks and fixture blocks vary widely). Because the probe can share the tool/spindle circuit (croc clip on the bit, other lead on copper), **no separate probe input is needed beyond the one probe pin** — the bit *is* the probe, the same wiring used for autoleveling.

### 10. Probe debounce / protection (critical for PCB mills)
A real hazard: grblHAL Issue #353 ("Z Stop in PROBE for PCB Milling", MKS SBASE/LPC1768) documents the tool over-travelling after contact — verbatim: "when the tool touches the PCB, it does not stop at that moment, but continues to advance for another **110 ms**, penetrating the tip of the engraving bit into the copper of the PCB, damaging it." (The user's probe logic there: "3.3V → No touch, 0V → Touch.")

grblHAL handles probe debouncing in firmware; recent builds added "a new debounce option for input pins that are interrupt capable" in the ioPorts interface, plus an optional **probe-protection plugin** (Expatria `grblhal_probe_plugin`) that halts if the probe is asserted outside a probing move and can block the spindle when a probe is connected. Your Rust/Embassy firmware should:
- Sample the probe in the **step ISR** so motion stops on the first confirmed trigger;
- Debounce a few ms (interrupt-capable pin);
- Keep the slow-pass probe feed low (25–100 mm/min) to bound over-travel from deceleration.

### 11. The multi-tool "reference tool" path (`$TLR`/`$TPW`, `$341`)
grblHAL implements semi/automatic tool-change touch-off natively:
- **`$341`** = tool change mode: `0` Normal (manual, allows jogging), `1` Manual touch off, `2` Manual touch off @ G59.3, `3` Automatic touch off @ G59.3, `4` Ignore M6. (Per the grblHAL wiki: modes 1–3 "available from build 20200805, mode 4 from build 20200929"; "$341 is used to set the mode and settings $342 - $344 for probing distance and feed rates.")
- **`$15`** = invert coolant pins (Flood/Mist) — **not** TLO-related. The user's note tying `$15` to "tool change on/off" is a misconception carried over from legacy Grbl numbering; tool-change behavior is governed by `$341` (mode) and `$342–$345` (distances/feeds).
- **`$TLR`** (Tool Length Reference): after a successful probe of the *reference* tool, stores the current linear-axis machine position as the reference and sets `TLR:1`. Per the wiki: "Automatic touch off mode and the `$TPW` command requires an initial tool length offset to be established before use. This can be done by manually issuing a tool change command for the first tool and then setting the offset by issuing a `$TLR` system command."
- **`$TPW`** (Tool Probe Workpiece, modes 1–2): probes the new tool, and the **controller** computes the dynamic offset as `probe_position − tlo_reference` and applies it as a dynamic TLO. From grblHAL `tool_change.c`: `gc_set_tool_offset(ToolLengthOffset_EnableDynamic, plane.axis_linear, sys.probe_position[plane.axis_linear] - sys.tlo_reference[plane.axis_linear]);` This is the "relative" strategy that **preserves Z work-zero across tool changes**.

### 12. ioSender's actual probing implementation
From the ioSender source (`CNC Controls Probing` folder) and maintainer statements:
- **Two-stage probe**: fast probe → rapid retract by the latch distance → slow latch probe. Verbatim from `EdgeFinderControl.xaml.cs`, the `OnCompleted()` Z-probe path:
  ```csharp
  ok = probing.WaitForResponse(probing.FastProbe + "Z-" + probing.Depth.ToInvariantString());
  ok = ok && probing.WaitForResponse(probing.RapidCommand + "Z" + probing.LatchDistance.ToInvariantString());
  ok = ok && probing.RemoveLastPosition();
  ok = ok && probing.WaitForResponse(probing.SlowProbe + "Z-" + probing.Depth.ToInvariantString());
  ```
  Console logs show the emitted commands as `G38.3F300Z-170` (fast) and `G38.3F25Z-170` (slow), preceded once by `G91F<ProbeFeedRate>`.
- **Latch / second-move distance** = `latch distance × 1.5, minimum 2 mm` (maintainer-stated, to avoid soft-limit trips on the retract).
- **Setting work zero**: ioSender's coordinate modes are `Measure`, `G92`, `G10`. In **`G10` mode it emits `G10 L2 P<n>`** (NOT `L20`); in **`G92` mode** it emits `G92<axiswords>` then `$G` to refresh parser state. For the Z/edge touch-plate the new Z value = **`WorkpieceHeight + TouchPlateHeight`** (verbatim: `pos.Z = probing.WorkpieceHeight + probing.TouchPlateHeight;`). The simple Z-probe path uses **G92 / G10 L2, never G43.1.**
- **Touch-plate thickness** is stored in the "Touch plate/fixture height" field on the Probing tab and simply added to the workpiece height.
- **Tool Length Offset tab** (`ToolLengthControl.xaml.cs`): uses the controller-side `$TLR`/`$TPW` mechanism with the "Establish reference offset" checkbox and "Probe fixture @ G59.3" option. The dynamic offset is computed by the **controller** (via `$TPW`), with ioSender orchestrating and displaying it; "TLO ref'd" lights when `TLR:1`.
- ioSender does its own probe-failure handling (checks `ProbePosition`/NaN), which is why it can safely use `G38.3`.

## Details

### grblHAL source-file map (for your Rust/Embassy port)
- **`gcode.c`** — parses G43/G43.1/G49 (modal group "tool offset"); enforces single-axis G43.1, errors on conflicts; sets `gc_block.modal.tool_offset_mode` ∈ {`ToolLengthOffset_Cancel`(G49), `ToolLengthOffset_Enable`(G43), `ToolLengthOffset_EnableDynamic`(G43.1), `ToolLengthOffset_ApplyAdditional`(G43.2)}.
- **`gcode.h`** — `tool_offset_mode_t`, `gc_state.tool_length_offset[]`, `g43_pending` (tool offset applied on M6 completion), `g92_offset` (noted persistent in grblHAL).
- **`motion_control.c`** — `mc_probe_cycle()` runs the probe, sets `sys.flags.probe_succeeded`, stores `sys.probe_position[]`, resets/syncs the planner; returns `GC_PROBE_FAIL_END` on no contact (→ alarm for G38.2/.4).
- **`tool_change.c`** — `$TLR`/`$TPW`/`$341` logic; `gc_set_tool_offset(ToolLengthOffset_EnableDynamic, plane.axis_linear, sys.probe_position − sys.tlo_reference)`; G59.3 fixture handling; restore-position-after-M6 (`no_restore_position_after_M6` flag).
- **`probe.c`/`probe.h`** — probe input read + state; honors `$6` invert and `$19` pull-up; multi-probe selection.
- **`report.c`** — formats `[TLO:]`, `[PRB:]`, `|WCO:|`, `|TLR:|`; legacy path prints `[TLO:` + `printFloat_CoordValue(gc_state.tool_length_offset)`.
- **`system.c`** — `$#` (`output_ngc_parameters`, "output offsets, tool table, probing and home position"); clears TLO on homing of the linear axis; raises `grbl.on_report_ngc_parameters`.
- **`settings.h`/`defaults.h`** — `$6` probe invert, `$19` probe pull-up disable, `$341`+ tool-change settings, `$10`/`$11` report mask, `TOOL_LENGTH_OFFSET_AXIS`.

### Decision: which strategy for a PCB mill?
1. **Absolute / re-zero (recommended for single-tool PCB work):** probe, then `G10 L2 P1 Z<plate_thickness>` (or `G92 Z<plate_thickness>`) so copper top = Z0. No `G43.1`, no persistence concerns. This is exactly what ioSender's Edge-finder Z probe does.
2. **Relative / reference-tool (only if you change tools mid-job without re-probing the surface):** establish `$TLR` on tool 1, then `$TPW` per subsequent tool so the controller applies a dynamic `G43.1`-equivalent offset and Z0 is preserved. Adds complexity (G59.3 fixture, `$341` mode) you almost never need for isolation routing + drilling on a hobby mill — you can simply re-probe the copper after a manual tool change instead.

## Recommendations

**Stage 1 — Minimum viable, single-tool PCB probing (do this first):**
1. Implement the probe as a dedicated, debounced input; expose `$6` (invert; document `$6=1` for NO plates) and `$19` (pull-up disable, default 0).
2. Implement `G38.2` (ALARM:5 on no-contact, ALARM:4 on already-triggered) and `G38.3` (no error). Store `[PRB:x,y,z:flag]` and emit the `[PRB:]` push message on success (gated by the `$10` probe flag).
3. Implement `G10 L2 P<n>` and `G92` so the sender can set Z0 = `plate_thickness` after probing. Verify `[PRB:…:1]` and the `WPos = MPos − WCO` math.
4. Sample the probe in the step ISR; cap the slow-pass feed at ≤100 mm/min to limit over-travel. This passes ioSender's "Edge finder, external → Probe Z" workflow out of the box.

**Stage 2 — Full grblHAL TLO compatibility:**
5. Implement `G43.1 Z<v>` (store dynamic TLO in RAM, fold into WCO, emit updated `|WCO:|`), `G49` (clear), and `[TLO:]` in `$#` (emit 1..N axis values per your compatibility level). Clear TLO on power-up, on Z-axis homing, and on soft reset if the machine was in motion.
6. Implement the `|TLR:<0|1>|` status element and (optionally) `$TLR`/`$TPW`/`$341` if you want ioSender's Tool-Length-Offset tab and multi-tool touch-off to work.

**Stage 3 — Robustness:**
7. Add probe-protection (halt if the probe is asserted outside a probing move) and the `Run:2` substate so ioSender shows probing state.

**Benchmarks that change the plan:** If you only ever run single-tool isolation + drill jobs, **stop at Stage 1** — you never need G43.1. If you adopt a fixed toolsetter or ATC, implement Stage 2 fully (the `$TLR`/`$TPW` reference-tool path). If over-travel into copper exceeds ~0.05 mm, lower the probe feed and tighten ISR debounce before changing anything else.

## Complete Annotated Reference Macro (NO touch plate, Z work-zero)

This is the **absolute / re-zero** macro — the one you actually want for a single-tool PCB job. It is what an ioSender-compatible firmware must support. Plate thickness is the measured value `<PLATE_T>` (e.g. `1.000`).

```gcode
; --- PRE-CONDITIONS ---
; $6=1            (NO plate: invert probe pin so untouched = not-triggered)
; $19=0           (probe pull-up enabled for passive plate)
; Croc clip on tool/bit, plate on copper surface, tool ~5-10 mm above plate.
; XY work zero already set.

G21                 ; mm
G91                 ; incremental for the probe moves
F100                ; default feed

; --- FAST SEEK (error if no contact within 15 mm) ---
G38.2 Z-15 F100     ; probe toward plate; ALARM:5 if no contact
; controller emits: [PRB:x,y,z:1] on success

; --- RETRACT then SLOW LATCH for accuracy ---
G0 Z2               ; rapid up 2 mm (>= latch*1.5, min 2 mm)
G38.2 Z-3 F25       ; slow re-probe; precise contact captured in [PRB:]

; --- SET Z WORK ZERO AT COPPER SURFACE ---
G90                 ; back to absolute
G10 L2 P1 Z[PLATE_T]   ; set G54 so the probed point = plate thickness,
                        ; i.e. copper top becomes Z0.
; (Equivalent ioSender G92 form:  G92 Z[PLATE_T] )

; --- SAFE RETRACT ---
G0 Z5               ; lift clear; remove plate before running the job
```

**Multi-tool / reference-tool variant (only if needed)** — uses dynamic TLO via `G43.1`, mirroring grblHAL's internal `$TPW` math (`offset = probe_Z − reference_probe_Z`):

```gcode
; ---- Tool 1 = reference (run once) ----
G49                 ; cancel any existing TLO
G38.2 Z-15 F100
G0 Z2
G38.2 Z-3 F25
$TLR                ; store this probe position as the tool-length reference (TLR:1)
; (then set Z work zero on the workpiece surface as above)

; ---- Each later tool, after manual change ----
; jog over the SAME probe/plate position, then:
$TPW                ; controller probes and applies dynamic offset =
                    ; (new_probe_Z - reference_Z) via G43.1-equivalent.
; OR explicit equivalent if you compute it host-side:
;   G38.2 Z-15 F100
;   G0 Z2
;   G38.2 Z-3 F25
;   G43.1 Z[ new_PRB_Z - reference_PRB_Z ]   ; preserves work Z0
```

Notes baked into the macro: `G38.2` (not `G38.3`) is used so a missed plate **alarms** rather than silently continuing; the fast/slow two-pass with a ≥2 mm retract matches ioSender's behavior; `G10 L2 P1` (or `G92`) — **not** `G43.1` — sets the surface zero for the single-tool case; `G43.1` and `$TLR`/`$TPW` appear only in the reference-tool variant.

## Caveats
- **`$15` is not "tool change on/off"** in grblHAL — it inverts coolant pins. Tool-change mode is **`$341`**. Do not wire TLO behavior to `$15`.
- The user's formula `WPos_Z = MPos_Z − WCO_Z − TLO_Z` **double-counts TLO**. In grblHAL the TLO is *inside* WCO; use `WPos = MPos − WCO`.
- ioSender uses **`G10 L2`** (not `G10 L20`) and **`G92`** for the touch-plate Z-zero path — **not** `G43.1`. `G43.1` is reserved for the `$TLR`/`$TPW` tool-length tab, where the **controller** computes the dynamic offset. Whether `ToolLengthControl.xaml.cs` ever emits a literal `G43.1` versus relying entirely on controller-side `$TPW` could not be confirmed at the source-line level; the evidence points to controller-side computation.
- `[TLO:]` axis count varies with `COMPATIBILITY_LEVEL`: single value `[TLO:0.000]` at high compat, up to all-axes (e.g. `[TLO:0.000,0.000,-14.442]`) at low compat. Parse 1..N comma-separated values.
- The exact `$6` value depends on your board's probe-input circuitry (opto-isolators / hardware inverters can flip the sense). Always verify empirically that the `Pn:P` status flag is **absent** when the plate is untouched.
- Plate thickness is hardware-specific and must be measured; never hardcode it.
- On PCB copper specifically, a single Z-probe only zeros one point; flatness errors across the board usually require **height-map autoleveling** (probe the bare tool against copper) for reliable isolation depth — plan for it even though it is beyond the TLO scope.