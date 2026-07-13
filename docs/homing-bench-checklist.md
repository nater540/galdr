# DOC-06 Homing & Limits — Hardware Bring-Up Checklist

> Hardware-in-the-loop bench procedure for the homing cycle + limit subsystem. The logic is host-tested
> (396 firmware-core tests) and Xtensa-compiled, but everything electrical/timing below is verifiable
> ONLY on the real board. Work top-to-bottom; do not skip the safety gates in §3 before §5.
>
> Board: ESP32-S3, native USB Serial/JTAG → `/dev/cu.usbmodem31101`, RX buffer 1024.
> Limit pins: X=GPIO10, Y=GPIO11, Z=GPIO12 (NC switches, internal pull-ups, rising-edge IRQ).
> Default homing order: Z first, then X+Y (sequential intra-group for now — DOC-06 TODO).
> Relevant alarms: `ALARM:1` hard limit, `ALARM:2` soft limit, `ALARM:8` homing fail, `ALARM:11` homing required.

## 0. Safety preamble
- [ ] **E-stop / power-cut within reach.** A wrong `$23` mask or a dead switch drives an axis INTO its
      hard stop at `$25` seek rate. Be ready to cut motor power.
- [ ] Start with the gantry **mid-travel** on all axes (room to move both directions).
- [ ] Spindle unpowered / no tool for the entire homing bring-up.
- [ ] Know your jog-off-limit recovery before you need it: `$X` unlock → jog away → re-home (§7).

## 1. Flash & connect
- [ ] `just flash` (build + flash + monitor) — confirm the grblHAL welcome banner on boot.
- [ ] Banner appears on every boot AND every `0x18` soft-reset (native USB can't be host-reset).
- [ ] If `$22` homing is enabled, the board should boot LOCKED in `ALARM:11` with the
      `[MSG:'$H'|'$X' to unlock]` push — confirm a sender sees the locked state on connect.
- [ ] `?` returns a `<...>` status line; `$$` dumps settings; `$I` / `0x87` answer (readiness probe).

## 2. Verify settings before any motion
Dump with `$$` and confirm the homing/limit block:
- [ ] `$5` limit-pin invert — set per NC wiring so an UNtriggered (closed) switch reads "not triggered" (§3).
- [ ] `$20` soft limits, `$21` hard limits, `$22` homing — note these are now **bitmasks** (`$21=1` hard
      only, `$21=3` +strict; `$22` bit0 enable, bit3 force-origin). Verify the value reads back as intended.
- [ ] `$23` homing dir-invert mask — `0` homes toward positive on every axis. **Set this to match where
      your switches physically are BEFORE homing** (wrong direction = drive away from the switch into the
      far stop).
- [ ] `$24` feed (slow locate) / `$25` seek (fast) rates, `$26` debounce ms, `$27` pull-off mm.
- [ ] `$130`/`$131`/`$132` max travel — used for the 1.5× no-contact search bound; a too-small value
      aborts homing early (`ALARM:8`), a too-large value lets a dead switch drive into the stop.
- [ ] Confirm `$20=1` is REJECTED unless `$22` is enabled (try it; expect an error, not silent accept).

## 3. Limit-switch sense & fail-safe (NO motion — manual switch actuation)
Do this with motors disabled or power to drivers off; you're only reading pins.
- [ ] At rest (all NC switches closed): `?` shows no limit flags; an idle machine is NOT in `ALARM:1`.
- [ ] **Manually depress X limit** → within `$26` ms the board raises `ALARM:1` (hard limit), halts,
      enters alarm. Repeat for Y, then Z. Confirms each pin maps to the right axis.
- [ ] **Broken-wire fail-safe:** with `$21` enabled, unplug one limit lead (open circuit). The pull-up
      should pull the pin HIGH = triggered → `ALARM:1`. Confirms NC + pull-up fail-safe (a severed wire
      stops the machine rather than silently disabling the limit).
- [ ] **`$5` sense check:** if the polarity is inverted (idle reads as triggered, or a press reads as
      clear), flip `$5` for that pin and re-test. Bring-up shortcut if switches aren't wired yet:
      jumper each limit GPIO to GND (satisfies NC-closed) or temporarily `$5=1`.
- [ ] **Known limitation to confirm, not fix:** a switch ALREADY held closed→open at the instant the
      machine goes idle won't generate a rising EDGE, so the idle monitor won't fire until the next edge
      or the next move's block-boundary sample. Verify the in-MOTION path catches it (§6) — that's the
      real safety path.
- [ ] **Debounce:** tap a switch rapidly / introduce contact bounce. With `$26` set (try 25 ms), confirm
      no spurious double-alarms; a glitch shorter than `$26` that doesn't persist is rejected.

## 4. Re-arm check
- [ ] After each §3 alarm: `0x18` reset (or `$X`), then jog away if needed, and confirm the machine
      RE-ARMS — i.e. a subsequent switch press alarms again (the limit path isn't dead after one trip).

## 5. Homing cycle `$H` — first run (hand on the power cut)
- [ ] With the gantry mid-travel, issue `$H`. Observe ordering: **Z seeks first**, then X, then Y.
- [ ] Each axis: fast seek (`$25`) into the switch → pull-off (`$27`) → slow locate (`$24`) re-approach →
      final pull-off so it ends **clear** of the switch. Watch the locate pass is visibly slower.
- [ ] During the cycle, `?` reports `<Home|...>` and does NOT stream live DRO (status requests are held).
- [ ] On success: exactly one `ok`, `ALARM:11` clears to Idle/Normal, and the machine is now homed.
- [ ] Confirm the axis ends **off** the switch (final pull-off worked) — a switch still depressed after
      homing means `$27` is too small or the pull-off direction is wrong.
- [ ] Check `?` MPos after homing matches expectation: `$22` bit3 (force-origin) → 0; otherwise derived
      from `$23`/`$130-132`/`$27`. WPos/WCO unchanged by homing.

## 6. Hard limits during motion (`$21`)
- [ ] With `$21` enabled and homed, jog SLOWLY toward a limit and trip it (or hand-press during a jog).
      Confirm: immediate halt, `ALARM:1` ("position likely lost"), spindle/coolant forced off.
- [ ] Confirm a hard limit does NOT fire DURING `$H` (shared-pin rule — the homing switch press is
      expected; the cycle must not self-abort with `ALARM:1`).
- [ ] Recovery: `0x18` → `$X` unlock → jog off the switch → `$H` re-home. Re-homing strongly recommended
      after any hard-limit hit (steps likely lost).

## 7. Soft limits (`$20`)
- [ ] Enable `$20` (requires `$22`). While homed, command a move whose target exceeds `$130-$132`.
      Confirm it's rejected with `ALARM:2` BEFORE motion starts (envelope pre-check, no travel toward
      the limit).
- [ ] Confirm soft limits are INACTIVE until homed (a force-enabled `$20` on an unhomed machine must not
      block moves on a bogus position).

## 8. Failure / abort paths
- [ ] **No-contact homing fail:** temporarily set a `$130` smaller than the real distance to the switch
      (or disconnect one switch) and `$H`. Expect `ALARM:8` (homing fail) + reset, NEVER a false `ok`.
      Restore `$130` afterward.
- [ ] **Mid-homing soft reset:** issue `$H`, then `0x18` mid-cycle. Confirm clean abort — banner re-emits,
      no stray homing move runs afterward (the latched HOME_REQUEST is drained), machine returns to the
      boot lock.
- [ ] **`$X` vs `$H`:** from the `ALARM:11` boot lock, `$X` should unlock to allow jogging WITHOUT
      establishing position (still unhomed); only `$H` clears the homing requirement by actually homing.

## 9. Repeatability
- [ ] Home 5× from different starting positions; record MPos each time. Spread should be within one
      `$24` locate-pass step (sub-0.01 mm target). If it drifts: lower `$24`, check `$26` debounce, and
      confirm the seek/locate two-rate behavior (accuracy comes from the slow locate, not the fast seek).
- [ ] Confirm the per-burst overshoot at the locate rate is below your repeatability target — this is the
      one quantitative open item from the research (depends on `$100-102` steps/mm and burst length).

## 10. Streaming-safety boot-lock (Fix #1, Option A fail-safe) — verification
> Compile-checked only until run here. The wedge→reset→breadcrumb→next-boot-lock chain and the `$22` alarm
> branch are exercised with the diagnostic provoke builds. Cross-ref `docs/streaming-lockup-investigation.md`
> §14.3 / §17.2 (the grbl feed-hold→ALARM→re-home contract Galdr adopted) and the four `WithholdReason`
> classes in `crash.rs` (`Core1Motion` / `Core0Comms` / `DeadZone` / `Core0ExecutorStall`).

- [ ] **Boot-lock, homing ENABLED (`$22=1`):** flash `--features provoke-stall-bare` (a production build that
      self-stalls the core-0 executor ~10 s after boot). The unfed RWDT resets at ~8 s and `stall_detector`
      records the `core0-executor-stall` breadcrumb. Confirm the NEXT boot comes up **LOCKED in `ALARM:11`**
      (not Idle), emits `[MSG:CRASH … core0-executor-stall]`, and refuses to stream until `$H`/`$X`.
- [ ] **Boot-lock, homing DISABLED (`$22=0`):** repeat with `$22=0`. Confirm the next boot comes up **LOCKED
      in `ALARM:3`** (position lost) rather than the default Idle. Restore `$22` afterward.
- [ ] **All four classes share this boot path:** `force_wedge_alarm` is gated on `withhold_was_wedge`, which is
      true for ANY tagged reason, so a locked boot for the executor-stall class proves the lock composition for
      all four — the classes differ ONLY in the decoded `[MSG:CRASH]` label. Spot-check a SECOND class with
      `--features force-withhold` (an unconditional withhold → RWDT reset → breadcrumb) and confirm the next
      boot is likewise LOCKED and carries a withhold label. (A per-class deterministic inducer does not exist
      for `Core1Motion` / `Core0Comms` / `DeadZone`; their boot-lock is the same code path, verified above.)

- [ ] **GAP-4 — no-step-loss on a RECOVERED withhold:** construct a `Core1Motion` withhold that RECOVERS before
      the ~8 s RWDT fires (the beat un-freezes in time), so `watchdog_feed` falls through to `feed()` and keeps
      cutting — grbl-consistent (§14.3/§17.2), since a genuine recovery here loses no steps. With a dial
      indicator (or a return-to-zero probe), confirm **zero position error** across the recovered wedge.
      ⚠️ If a recovery CAN follow REAL step loss, this fails — then the `watchdog_feed` recovery branch MUST be
      wired to a MOTION_FAULT (feed-hold + `ALARM:17` + require re-home) instead of falling through to `feed()`.

## Sign-off
- [ ] All §3 limit senses + fail-safe correct.
- [ ] `$H` completes, ends clear of switches, sets correct MPos, 5× repeatable.
- [ ] `ALARM:1` / `ALARM:2` / `ALARM:8` / `ALARM:11` all behave and recover.
- [ ] §10 boot-lock verified: `$22=1`→`ALARM:11`, `$22=0`→`ALARM:3` after a provoked wedge; GAP-4 no-step-loss
      confirmed on a recovered `Core1Motion` withhold (or the recovery branch re-wired to MOTION_FAULT).
- [ ] Note any setting changes made at the bench back into the persisted defaults.

> After sign-off, the remaining DOC-06 deferral is concurrent intra-group (X+Y) homing
> (`TODO(DOC-06)` in `run_homing`) — a speed/squaring refinement, not a correctness gap.
