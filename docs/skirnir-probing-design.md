# DOC-11 — skirnir Probing & Rotary Setup (design + TDD scope)

> Host-side probing for Galdr's skirnir sender. Covers the foundational probe-result pipeline and the three
> rotary-A setup/measure wizards. **Firmware stays a linear-G38 executor; skirnir owns the rotary center of
> rotation and all trig** (see `docs/4th-axis-rotary-design.md` DOC-10 and the probing-architecture research). Read
> this before adding probe UI or touching the `[PRB:]` path. Source of truth for behavior remains `crates/skirnir/src/`.
>
> **Validation (deep-research fact-check, 2026-06-21).** The design's protocol/math claims were independently
> verified (18/25 claims confirmed; sources: gnea/grbl interface+commands docs, grblHAL `core` alarms.c + issue
> #395, LinuxCNC/NIST RS274). Corrections from that pass are folded in below and tagged **[verified]** /
> **[corrected]**. Confidence is HIGH for the protocol items (PRB format, async model, alarm-lock, G10 L2/L20) and
> MEDIUM for the center-finding *arithmetic* (the Y-before-Z ordering is mathematically forced and high-confidence;
> the formulas are the symmetric two-sided ones — a single-value blog recipe was refuted).

## 0. Context — the architectural gap this closes

Probing wizards are **multi-step, result-dependent sequences**: probe → await `[PRB:]` → validate success →
compute → act. skirnir today has none of that machinery:

- **`[PRB:x,y,z,a:flag]` is never parsed.** It arrives as a generic `Response::Message(body)`
  (`protocol/response.rs`) and only logs to the console — no typed `{x,y,z,a,success}` exists.
- **No request→await-response mechanism.** The engine is fire-and-forget; the app polls an `Event` stream
  (`engine.rs` `Event::Response(Response)`) and latches state. There is no "send this probe and await its result".
- **The existing `ProbeZ` is fragile.** `shell.rs::probe_z()` fires `G38.2 Z-…` then **immediately** fires the
  `G10 L20` zeroing line with no success check — saved today only by alarm-ordering (a failed `G38.2` alarms and
  *usually* gets the `G10` rejected). It relies on timing, not validation. Phase 0 fixes this.

Everything below is built on the firmware contracts already in place: the **4-field `[PRB:x,y,z,a:flag]`** report,
the **4-field `MPos:`/`WPos:`** status, **G38 linear-only** (a rotary `A` word in a probe is rejected — so the
wizards index A *separately*, never inside a probe), and **`G10 L2/L20`** offset writes.

---

## Phase 0 — Foundations (prerequisite for every wizard)

### 0.1 Typed `[PRB:]` parsing
- New parser (extend `protocol/status.rs` or a new `protocol/probe.rs`) for the `[PRB:<x>,<y>,<z>{,<a>…}:<flag>]`
  body. Per grblHAL parsing rules, accept **1..N** comma-separated axis values (don't hardcode 4 — handle a
  3-field legacy controller and a future 5th axis), then the trailing `:0`/`:1` flag.
- New `Response::ProbeResult { position: Vec<f64>, success: bool }` variant (parsed in `response.rs::parse_line`
  ahead of the generic `Message` fallback). The console still shows it; the app now also gets typed access.
- TDD: parse `[PRB:-1.015,0.000,-2.500,90.000:1]` → `{position:[-1.015,0,-2.5,90], success:true}`;
  `[PRB:0,0,0:0]` (3-field) → success:false, 3 values; malformed → `Message` fallthrough (never panic).

### 0.2 Probe-operation latch (the result-await mechanism)
- The app cannot block; it must **correlate a probe it issued with the `ProbeResult` that comes back** off the
  `Event` stream. Add a small state machine in `view_state.rs`/`shell.rs` (generalize the existing port-identify
  escape-hatch): `ProbeOp { kind, awaiting: bool, last: Option<ProbeResult> }`.
- Flow: issue a probe line → set `awaiting` → on the next `Event::Response(Response::ProbeResult)` capture it,
  clear `awaiting`, expose `last` + success to the UI. An intervening `Alarm`/`Error` resolves the op as failed.
- Keep it event-driven (no new async await primitive) — consistent with the rest of the engine. **[verified]** grbl
  streaming has no request→response correlation; `ok` only acks the line, and the `[PRB:]` line arrives
  asynchronously — so latching off the event stream is the correct (and only) model. (Do NOT model this on g2core,
  which reports synchronous JSON status — a different protocol that would mislead toward a call/response design.)
- **Push-or-poll fallback:** grblHAL can be configured to suppress the immediate `[PRB:]` push, with the last
  result still retrievable via `$#`. Galdr's firmware *does* push today, but make the latch defensive: if no
  `ProbeResult` push arrives within a timeout after the probe's `ok`, query `$#` and parse its `[PRB:]` line. This
  also covers a missed/dropped push.
- TDD: feed a synthetic event sequence (status, then `ProbeResult`) through the reducer; assert the op latches the
  result and clears `awaiting`; assert an `Alarm` before any `ProbeResult` resolves the op as failed.

### 0.3 Harden the existing `ProbeZ`
- Split `probe_z()` into **probe → await result → validate success → zero**. Only emit the `G10 L20` zeroing line
  after a `ProbeResult{success:true}`; on failure surface it and do nothing destructive.
- **[corrected] Probe-failure semantics:** a failed *alarming* probe (`G38.2`/`G38.4`) raises **`ALARM:5`** (no
  contact within travel) — NOT `ALARM:4`, which is the *other* probe alarm (probe already in/not-in the expected
  initial state). An active alarm **locks subsequent g-code with `error:9`** until `$X`/reset — which is the
  alarm-ordering that *partially* protects today's unconditional `probe_z()`, but it's a race, not validation
  (a host that streams the `G10` before the alarm latches can still apply it). The *silent* probes (`G38.3`/`G38.5`)
  do **not** alarm at all — they just set the `:0` flag — so for those the host MUST check the flag or it will zero
  on a non-contact. This is exactly why the latch validates `success` before zeroing.
- Render the probed point + success in the probe panel (`views.rs`) instead of leaving it as console noise.
- This Phase-0 work is independently valuable — it fixes today's fire-and-forget Z touch-off, rotary or not.

---

## Phase 1 — Rotary setup

### 1.1 Rotary-safe probe primitive
Every rotary probe is **index-then-probe**, never probe-while-rotating (cosine error / invalid tip comp):
1. Retract Z to a safe clearance (`G53 G0 Z<safe>` or a configured clearance height).
2. `G0 A<angle>` to index, then **HOLD** — insert a short settle/dwell (`G4 P<settle>`) before the probe so any
   rotary backlash/oscillation damps out. Both the clearance height and the settle time are **bench-tuned
   parameters** (open question — start conservative). **[verified]** index-then-probe with the rotary held is the
   universal practice; probing during rotation is unsound (cosine error / invalid tip comp).
3. A single **linear** `G38.2` along the chosen axis (X/Y/Z) — never an `A` word in the probe (firmware rejects it).
4. Await the `ProbeResult` (Phase 0.2).
A reusable helper drives this; the wizards below compose it. TDD: assert the emitted line sequence for a given
angle/axis/feed (retract, `G0 A…`, `G38.2 …`), and that no probe line ever contains an `A` word.

### 1.2 Rotary center-finder wizard (the headline feature)
Finds the A centerline (its Y and Z machine coords) relative to the spindle, using a known-diameter dowel/gauge
clamped concentric. **Math (research-confirmed):**
- **Y center first**, probe both sides at center height: `Y_c = (Y_left + Y_right) / 2`. The tool/probe radius
  cancels in the midpoint (equal-and-opposite), so no tip-radius term is needed for Y.
- **Then Z center**, probe the top *at the true Y center*: `Z_c = Z_top − D/2` (D = dowel diameter, operator
  input). Order matters — a top probe off the Y center reads a chord, not the diameter.
- UX: a guided wizard — operator enters D, jogs the tool to the approximate approach for each touch; the wizard
  issues each rotary-safe probe (1.1), captures `Y_left`/`Y_right`/`Z_top`, computes `(Y_c, Z_c)`, and shows them.
  Semi-automatic (operator positions; wizard probes) for the first version — full auto-positioning is a crash risk
  deferred. Then offer to **write the WCS via `G10 L2`/`L20`** so Y0/Z0 land on the cylinder axis/top (the Fusion
  wrap convention). **[verified]** `G10 L2 Pn` sets the offset from absolute machine coords; `G10 L20 Pn` makes the
  *current* position equal the given value (`Pn` selects G54–G59; `P0` = active). The wizard emits **only the Y and
  Z words** — never an `A` word — so the rotary datum is untouched (whether an A word in L2/L20 perturbs the rotary
  on this build is an unresolved edge case best avoided entirely).
- **[implemented — datum is operator-selectable]** Y0 is **always** the axis centerline (`Y_c`); the **Z0 datum is
  a product choice** the operator picks: the **rotary axis centerline** (`Z_c = Z_top − D/2`, the DEFAULT — the
  wrap-machining convention) or the **probed top surface** (`Z_top`, with the axis then at work-Z = −D/2). The
  wizard uses **`G10 L2`** (not `L20`): `Y_c` and the chosen Z are machine coordinates computed from the
  machine-coordinate `[PRB:]` readings, so `L2` writes the WCS origin directly from absolute machine coords (the
  tool need not be sitting on the feature); `L20` ("make the *current position* read this") would require the tool
  to be physically at the centerline/top, which it is not. The emitted line is `G10 L2 P0 Y… Z…` — Y/Z only, never
  A. The selector lives in `crates/skirnir/src/app/rotary_center.rs::ZDatum` (`offer_g10` follows it).
- TDD: feed scripted `ProbeResult`s for the three touches → assert `Y_c`/`Z_c` math and the exact `G10` line
  offered (both Z datums); assert the wizard refuses to advance on a failed touch.

### 1.3 Data model — where the rotary center lives
firmware has **no pivot/kinematics concept**, so `(Y_c, Z_c)` (and the dowel D, A-datum) live as **skirnir
project/profile state**, projected into firmware only as `G10` WCS offsets and the A-relative geometry baked into
sent g-code. (See the probing-architecture memory.)

**[implemented — `crates/skirnir/src/profile.rs`]** skirnir had no persistence layer, so a found center evaporated
on exit and forced a full re-probe every launch. The store is now a **versioned RON file under the OS config dir**
(`~/.config/skirnir/profile.ron` on Linux, via `directories::ProjectDirs`) — framework-agnostic (no egui/eframe
`Storage`), so the headless `--cli` path shares the same project state as the GUI and the whole thing unit-tests as
a pure round-trip. It persists `RotarySetup { y_center, z_center, dowel_diameter, a_datum_deg, z_datum }` plus a
tight `Prefs` block (last port/baud and the rotary input defaults); transient runtime state is never written. I/O is
non-fatal by contract: a missing file is the silent first-run default, a corrupt/too-new file falls back to defaults
**with a notice** (the leading `version` field refuses a newer layout rather than misreading it), and a save returns
a typed error the shell surfaces instead of panicking. Writes go to a sibling temp + atomic rename so an interrupted
write can't leave a torn profile. The shell loads on startup (seeding the connect dropdown + rotary inputs) and
saves at the write-WCS, connect, and `on_exit` points; an `Intent::ApplySavedRotaryCenter` re-emits the persisted
`G10 L2` (Y/Z only, never A) so a restart restores the center without re-probing — the §1.3 payoff.

---

## Phase 2 — Verify / measure

> **Implementation note — probe axis.** Both Phase-2 wizards take an `axis`/`dir` in their intents
> (`FlipVerifyStart`, `RunoutStart`) and the shared `angle_sweep` engine honors it, but the UI currently hardcodes
> **−Y** (`views.rs`: `axis: Axis::Y, dir: Dir::Neg`, and the panel label reads "probing −Y"). An X/Y/Z axis picker
> that threads the selection through is a small host-testable follow-up; the math below is axis-agnostic.

### 2.1 180°-flip center-verify
Cancels eccentricity to validate/refine the center: probe a feature at θ (reading `r1`), `G0 A<θ+180>`, probe the
SAME side again (reading `r2`). Both are `G38.x` SURFACE touches, so each reading carries the probe contact radius
`R` (`r1 = C + e + R`, `r2 = C − e + R`, with rotation axis `C` and eccentricity `e`):
- `error = (r2 − r1) / 2 = −e` — the residual eccentricity. The radius **cancels in the difference**, so this is
  radius-free and is what we SHOW. Zero exactly when the feature is centered.
- **The absolute axis `C` cannot be recovered from two same-side touches** (it would need `R = D/2`). So the
  correction is a **RELATIVE shift** of the current work origin by `error`: `new_origin = current_origin + error`
  (since Phase 1 placed the origin at the dowel center, `error = C − O`). This is radius-free and a **true no-op
  when centered** — it must NEVER write the surface midpoint `(r1+r2)/2` (= axis + radius), which would move the
  origin a full radius off the axis. `current_origin` is the tracked `WCO` on the verified axis; if no `WCO` has
  arrived the correction is withheld. Carries only the verified axis word, never `A`. Composes the 1.1 primitive
  twice with an `A180` between. TDD: realistic readings WITH the `+R` term → `error = Δ/2`; centered → correction
  is a no-op; the offered `G10 L2` shifts the current origin by the residual (not the surface midpoint).

### 2.2 Runout report
Probe at N angles around the part at fixed X (rotary-safe primitive at each angle), collect the radial readings,
report **TIR = max − min** and **eccentricity = TIR/2**. Read-only (no offset written). UX: pick N, run, show a
small table + TIR/eccentricity. TDD: scripted N readings → assert TIR and eccentricity.

---

## New types / modules / files touched

| Area | Change |
|------|--------|
| `protocol/response.rs` | `Response::ProbeResult { position: Vec<f64>, success: bool }`; parse ahead of `Message`. |
| `protocol/probe.rs` (new) or `status.rs` | `[PRB:…]` body parser (1..N values + flag). |
| `app/view_state.rs` | `ProbeOp` latch state; reduce `ProbeResult`/`Alarm` into it; expose `last`/`awaiting`. |
| `app/intent.rs` | New intents: `ProbeRotaryCenter{…}`, `ProbeFlipVerify{…}`, `ProbeRunout{n,…}` (+ the rotary-safe primitive params); harden `ProbeZ`. |
| `app/shell.rs` | Wizard sequencing (compose the 1.1 primitive; latch results; compute; offer `G10`). |
| `app/views.rs` | Probe-result panel (XYZ/A + success + retry/cancel); the three wizard panels. |
| `profile.rs` (new) | Versioned RON profile store: persist `RotarySetup` `(Y_c, Z_c, D, A-datum, Z-datum)` + `Prefs` (port/baud, rotary input defaults); load on startup, save at write-WCS/connect/exit. |

## Firmware contract dependencies (all already satisfied)
- 4-field `[PRB:x,y,z,a:flag]` ✓ (DOC-11 consumes it; DOC-10 firmware side shipped).
- 4-field `MPos:`/`WPos:` for reading the live A angle ✓.
- **G38 linear-only / A-word rejected** ✓ — the wizards rely on this and must index A *outside* every probe.
- `G10 L2/L20` offset writes ✓.

## Open questions / risks
- **Probe hardware for non-conductive stock.** A conductive touch-plate can't center a non-conductive blank in the
  chuck — needs a touch-trigger probe or a conductive proxy dowel. UX should make the probe type explicit. (PCB
  use case specifically.)
- **Approach choreography.** First version is semi-automatic (operator jogs the approach). Fully-parameterized
  auto-approach is a crash risk; deferred until the manual flow is proven on the bench.
- **rotary hard-limit conflict — largely MOOT for Galdr [corrected].** In grblHAL a hard-limit switch on a rotary
  axis can re-trigger during continuous rotation; the prescribed workaround is to drop that axis from hard-limit
  checking (the maintainer cited `$21=5`, but that exact value/bit is undocumented in public labels and was NOT
  independently confirmed — verify on-target against the *pinned* grblHAL revision if it ever matters). **Galdr's
  firmware already sidesteps this:** the A axis has no limit switch and is *excluded from all limit sampling*
  (DOC-10 — A never appears in `Pn:`, never raises `ALARM:1`). So the `G0 A…` indexing in the rotary-safe primitive
  does not face this hazard on our build. Do NOT bake a specific `$21` value into the wizard.
- **Bench-gated.** None of this is verifiable without the board + a dowel; the math and emitted-g-code are
  host-testable, the physical accuracy is not.
