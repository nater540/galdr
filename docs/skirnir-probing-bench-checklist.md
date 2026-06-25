# DOC-11 skirnir Probing & Rotary Setup — Hardware Bring-Up Checklist

> Hardware-in-the-loop bench procedure for the host-side probing pipeline and the rotary-A setup/measure
> wizards. The logic is host-tested (skirnir's ~460 tests over a loopback transport) and the emitted g-code is
> asserted line-for-line, but the **physical accuracy** — does the probe actually find the center, does the
> wizard's number match a dial indicator — is verifiable ONLY on the real board with a probe and a dowel. Work
> top-to-bottom; do NOT skip the §0 safety gates or the §1 prerequisites.
>
> Companion to `docs/homing-bench-checklist.md` (DOC-06), `docs/4th-axis-bench-checklist.md` (DOC-10), and the
> spindle bring-up — same posture: logic host-tested, hardware path deferred. **Read `docs/skirnir-probing-design.md`
> (DOC-11) first** — this checklist verifies what that design specifies.
>
> Board: ESP32-S3, native USB Serial/JTAG → `/dev/cu.usbmodem31101`, RX buffer 1024. skirnir is the driver.
> Probe input: **PROBE = GPIO21** (`Pn:P` in status), internal pull-up unless `$19`, polarity invert via `$6`.
> Firmware stays a **linear-G38 executor**: it owns `G38.x` + the `[PRB:]` push; ALL rotary trig/center math is
> host-side in skirnir (`crates/skirnir/src/app/rotary_center.rs`), projected to the firmware only as `G10 L2`
> offsets and the per-touch lines from `rotary_probe.rs::rotary_safe_probe_lines`.
> Relevant alarms: `ALARM:5` no-contact (the alarming `G38.2`/`G38.4` ran out of travel), `ALARM:4` probe already
> in the wrong initial state, `error:9` g-code locked out by an active alarm.

## 0. Safety preamble
- [ ] **E-stop / motor-power cut within reach.** A probe that doesn't trigger drives the tip into the dowel/plate
      at the probe feed for the full `depth_mm` before it alarms — be ready to cut power.
- [ ] Use a real **conductive touch probe or touch-plate** and confirm continuity end-to-end (probe tip → stock →
      return lead) with a meter BEFORE any `G38`. A floating probe input reads noise and can false-trigger or
      never trigger.
- [ ] Start every touch with the tip a SHORT, known distance from the surface — keep `depth_mm` (the §5 param)
      just larger than the real gap, never a blind long search.
- [ ] Spindle unpowered / no tool — a gauge probe in the collet only, for the entire probing bring-up.
- [ ] Know the recovery before you need it: a no-contact probe raises `ALARM:5` and LOCKS subsequent g-code with
      `error:9`; clear with `$X` (or `0x18`), jog clear, retry. skirnir surfaces the alarm and fails the latch.

## 1. Prerequisites — sign these off first
- [ ] **DOC-10 (4th-axis A) is bench-signed-off** (`docs/4th-axis-bench-checklist.md`). The rotary wizards index
      `G0 A<angle>` and read the 4-field `[PRB:]`/`MPos:`; if A isn't calibrated (`$103`) and direction-confirmed,
      every center number inherits that error. The §6–§9 rotary tests are meaningless until A is trustworthy.
- [ ] **DOC-06 probe path:** the firmware answers a manual `G38.2 Z-… F…` with motion + a single `[PRB:]` line +
      one `ok`. Confirm a bare manual probe works at the console before driving it from a wizard.
- [ ] **skirnir connects** and elicits readiness (sends `?`/`$I`, polls `?` — a passive connect hangs in
      Connecting; see the board-connection notes). The 4-field DRO shows X/Y/Z/A.
- [ ] A known-diameter **gauge dowel** clamped concentric in the rotary chuck for §6+ (record its true diameter
      `D`, measured, not nominal — it feeds `Z_c = Z_top − D/2`).
- [ ] For repro outside the GUI, drive scripted lines with `skirnir --cli <port> <gcode>` (timeout-bounded, exit
      codes) — never ad-hoc `cat`/`printf` to the port (it hangs).

## 2. Probe input & typed `[PRB:]` (Phase 0.1 / 0.2)
Verify the pin sense and that skirnir parses the result into a typed latch, not console noise.
- [ ] **Idle sense:** untouched, `?` shows NO `P` in `Pn:` (the field is omitted when nothing is asserted). Short
      the probe to its return by hand → `Pn:P` appears. If inverted (idle reads `P`, touch clears it), set `$6`.
- [ ] **Successful touch:** a manual `G38.2 Z-… F…` that contacts emits `[PRB:x,y,z,a:1]`. In skirnir the probe
      panel shows the typed point + success — NOT just a raw console line (`protocol/probe.rs` parsed it ahead of
      the generic message fallback; the latch in `view_state.rs::ProbeOp` captured it).
- [ ] **No-contact:** a probe that runs the full travel raises `ALARM:5`, no `[PRB:]` push (or `:0`). skirnir's
      latch must resolve the op **failed** (an `Alarm` before any `ProbeResult` fails it) and surface the reason —
      never leave the UI awaiting forever.
- [ ] **Lost-push fallback (`$#`):** the latch is defensive — if no `[PRB:]` push arrives within the timeout after
      the probe's `ok`, skirnir queries `$#` and parses its `[PRB:]` line (`transport/probe.rs`). Hard to force on
      a pushing firmware; if you can suppress the push, confirm the `$#` fallback still latches the last result.
- [ ] **4-field tolerance:** confirm skirnir reads `[PRB:x,y,z,a:flag]` (4 values) AND a legacy 3-field
      `[PRB:x,y,z:flag]` without panicking — it parses 1..N values then the flag.

## 3. Hardened Z touch-off (Phase 0.3)
The fire-and-forget bug this phase fixed: never zero off a failed probe.
- [ ] **Success path:** probe Z onto the plate; only AFTER `[PRB:…:1]` does skirnir emit the `G10 L20` zeroing
      line. Confirm work-Z is set and the probed point is shown in the panel.
- [ ] **Failure path:** force a no-contact Z probe (start too far / lift the plate). On `ALARM:5` skirnir must do
      **nothing destructive** — NO `G10` zeroing line is sent — and surface the failure. (Pre-fix this raced on
      alarm-ordering; now it validates `success` before zeroing.)
- [ ] **Silent probe caution:** if you ever use `G38.3`/`G38.5` (non-alarming), confirm skirnir checks the `:0`
      flag and refuses to zero on a non-contact — those don't alarm, so the flag is the only signal.

## 4. Rotary-safe probe primitive (Phase 1.1) — the side/top Z fix
`rotary_safe_probe_lines` emits: retract → index → settle → (side touches only) descend → `G91 G38.2` → `G90`.
The side-vs-top Z split is the recently-fixed correctness point — verify it physically.
- [ ] **Top (Z) touch:** at the dowel, a top touch retracts to `clearance_mm`, indexes A, settles, then
      `G91 G38.2 Z-…` descends straight onto the top. Confirm it does NOT insert a side descend (it would drive
      into the top before probing). Watch the monitor: only ONE `G53 G0 Z` before the probe.
- [ ] **Side (X/Y) touch:** a side touch retracts to `clearance_mm` (clear above the dowel), indexes, settles,
      then inserts `G53 G0 Z<side_probe_z>` to drop back INTO the dowel's Z-extent at the approach Y (off to the
      side, clear of the flank) BEFORE the lateral `G38.2 Y…`. Confirm the descend appears and the tool meets the
      flank — not sails over it.
- [ ] **The defining failure mode (regression guard):** with `side_probe_z` mis-set ABOVE the dowel top, a side
      touch must MISS (run full `depth_mm`, `ALARM:5`) — it must never silently read a bad height. This is exactly
      what the single-`clearance_mm` design did before the fix; confirm a too-high `side_probe_z` fails loudly.
- [ ] **No `A` word in any probe line:** scan the emitted lines — the index carries `A`, the `G38.2` NEVER does
      (the firmware rejects a rotary word in a probe; §11 of the DOC-10 checklist).
- [ ] **Relative probe restored:** the `G38.2` is wrapped `G91…G90`; confirm A subsequent absolute move (the
      index, the move-to-Yc) is not silently incremental.

## 5. Bench-param tuning (now operator-editable + persisted)
The "Bench params" section of the wizard edits `RotaryProbeParams` (`UiState.rotary_bench`), persisted to
`Prefs.rotary_bench`. Dial these in for THIS bench before §6.
- [ ] **Clearance Z (`clearance_mm`):** set so the retract clears the TOP of the dowel + chuck with margin — the A
      index and the move-to-Yc lateral sweep both happen at this height and must not graze anything.
- [ ] **Side-probe Z (`side_probe_z`):** set WITHIN the dowel's Z-extent (between top and bottom — roughly the
      flank near mid-height). This is the one parameter whose wrong value crashes or misses (§4). It is a separate
      knob from clearance precisely because no single height serves both the index/top-probe and the side touches.
- [ ] **Settle / Feed / Depth:** start conservative (long settle, slow feed, depth just over the real gap). Raise
      the feed only once touches are reliable; keep depth bounded so a no-contact stops soon.
- [ ] **Persistence:** set non-default values, run a touch, then restart skirnir — confirm the values come back
      pre-filled (written to the profile at connect/write-WCS/exit). A pre-existing profile without the block must
      still load with the params defaulted (host-tested, but eyeball it once on the real config file).

## 6. Rotary center-finder wizard (Phase 1.2) — the headline accuracy test
Find the A centerline (its machine Y and Z) from the concentric gauge dowel. This is where the math meets metal.
- [ ] **Enter D** (the MEASURED dowel diameter) and the index angle; Start the wizard.
- [ ] **Y-left touch:** jog to the −Y face approach, probe. Captures `Y_left`. **Y-right touch:** jog to the +Y
      approach, probe. Captures `Y_right`. The wizard shows `Y_c = (Y_left + Y_right)/2`. The two touches descend
      to the SAME `side_probe_z` (§4) — that shared height is what makes the midpoint cancel the tool/probe radius.
- [ ] **Move-to-Yc gate:** the top probe is LOCKED until the wizard's "Move to Y center" actually runs (retract to
      `clearance_mm`, then `G53 G0 Y<Y_c>`). Confirm the button order is enforced — you cannot probe the top from
      off-center (which would read a chord, not the diameter).
- [ ] **Z-top touch at Y_c:** probe the top. `Z_c = Z_top − D/2`. Confirm the shown `Z_c` is sane for the dowel.
- [ ] **Independent accuracy check (the real sign-off):** sweep a dial indicator across the dowel and compare to
      the wizard's `Y_c`/`Z_c`. Target agreement within a few hundredths of a mm. A consistent Y bias hints at a
      tool-radius / side-touch-height issue; a Z bias hints at a wrong measured `D` or a chord (off-center top).
- [ ] **WCS write:** "Write center → WCS" emits `G10 L2 P0 Y<Y_c> Z<…>` — **Y/Z only, never an `A` word** (leave
      the rotary datum alone). Confirm via the monitor. After it, work-Y0 sits on the axis; spin A and watch a
      dial on the axis line — Y0/Z0 should stay put through rotation if the center is right.
- [ ] **Z-datum selector:** default `AxisCenterline` writes `Z = Z_top − D/2` (work-Z0 on the axis, wrap-machining
      convention); switch to `TopSurface` and confirm `Z = Z_top` (axis then at work-Z = −D/2). Y is unchanged.
- [ ] **Abort safety:** fail a touch mid-run (lift the probe) — the wizard must abort with a reason and expose NO
      partial center (never a `G10` off a bad reading).

## 7. Profile persistence & re-apply (Phase 1.3)
- [ ] After a successful §6 write, confirm the center is saved (`RotarySetup{y_center, z_center, dowel_diameter,
      a_datum_deg, z_datum}`) to the profile RON under the OS config dir.
- [ ] **Restart skirnir, reconnect, "Apply saved center"** — it re-emits the SAME `G10 L2` (Y/Z only) WITHOUT
      re-probing. Verify a dial on the axis confirms Y0/Z0 land where the fresh probe put them — the §1.3 payoff.
- [ ] Corrupt/edit the profile by hand → skirnir falls back to defaults WITH a notice, never a crash or a torn
      file (atomic temp+rename write).

## 8. 180°-flip center-verify (Phase 2.1)
- [ ] With a center written, run the flip-verify: it probes a face, indexes A 180°, probes again, and offers a
      Y/Z-only correction. Confirm the two touches index-then-probe (never probe-while-rotating) and that the
      offered correction is small when the center is good — a LARGE correction flags a bad center or A backlash.
- [ ] The offered correction line is `G10 L2` Y/Z only, never `A`. Confirm.

## 9. Runout report (Phase 2.2)
- [ ] Run the runout report over N angles. It probes the dowel at each indexed angle and reports the spread —
      and writes NOTHING (it's a measurement, not a datum set). Confirm no `G10` is emitted.
- [ ] A true gauge dowel should report low runout; a deliberately eccentric setup should report more. This
      sanity-checks both the A indexing repeatability and the probe consistency.

## 10. Fault / abort paths
- [ ] **Alarm mid-wizard:** trigger `ALARM:5` (no-contact) on any wizard touch — the latch fails, the wizard
      aborts with the reason, and g-code is locked (`error:9`) until `$X`/`0x18`. Confirm recovery: unlock, jog
      clear, restart the wizard cleanly.
- [ ] **Soft-reset mid-probe:** `0x18` while a touch is awaiting — skirnir drops the in-flight line, fails the
      latch, and the banner re-emits. No stray probe runs after.
- [ ] **Disconnect mid-wizard:** pull the link during a run — the wizard fails the send (not connected) and
      aborts rather than hanging awaiting a result that can't arrive.
- [ ] **Cross-flow guard:** starting a wizard touch cancels any other probe op so two flows can't claim the one
      latch — confirm a Z touch-off and a wizard touch don't interleave results.

## 11. Non-conductive stock (open question — record findings)
- [ ] A conductive touch-plate cannot center a NON-conductive blank. For PCB/non-conductive work, center on a
      conductive PROXY dowel of known diameter in the same chuck, write the WCS, then mount the real stock without
      disturbing A. Confirm this workflow and note any fixture needed. (A touch-trigger probe avoids the proxy —
      record which probe type the shop standardizes on so the wizard UX can make it explicit.)

## Sign-off
- [ ] Probe pin sense (`$6`/`$19`) correct; `[PRB:]` parses typed (4- and 3-field), latch resolves success AND
      failure, `$#` fallback covers a lost push.
- [ ] Hardened Z touch-off never zeroes on a failed probe.
- [ ] Side touches descend to `side_probe_z` and meet the flank; top touch descends straight onto the top; a
      too-high `side_probe_z` fails loudly (the §4 regression guard).
- [ ] Center-finder `Y_c`/`Z_c` match an independent dial-indicator measurement within target; `G10 L2` is Y/Z
      only; both Z-datums correct; abort exposes no partial center.
- [ ] Center persists and re-applies across a restart without re-probing.
- [ ] Flip-verify correction small on a good center; runout report writes nothing and tracks reality.
- [ ] All fault paths (alarm / soft-reset / disconnect) fail the latch and recover cleanly.
- [ ] Bench params dialed in and persisted; note the final `clearance_mm`/`side_probe_z`/feed/depth back into the
      profile (and into `RotaryProbeParams::DEFAULT_*` only if they make a better universal default).

> After sign-off, the remaining DOC-11 deferrals are UX, not correctness: fully-parameterized auto-approach
> (deferred as a crash risk until the manual flow is bench-proven — that's this checklist) and making the probe
> type (conductive plate vs touch-trigger) explicit in the wizard for the non-conductive-stock case (§11).
