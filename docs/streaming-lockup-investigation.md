# Streaming Lockup Investigation (ESP32-S3 firmware)

**Status: OPEN (fix shipped but INCOMPLETE — wedge still reproduces).** Root cause of the streaming drumbeat is a
LOST USB TX-DONE WAKE (§11–§12, high confidence on the mechanism, on ONE positive capture). A recovery fix
shipped to main (`6024126`) but the wedge STILL reproduces (§13): **Signature A** = a write-stage/ISR-never-armed
lost-wake sub-flavor the flush-stage-only fix structurally cannot catch (the single-chunk widening, §13.1, is the
identified minimal fix — GATED on a `wstg=`-stable confirming capture); **Signature B** = a hard silent total lock
with no trace, whose structural root is a verified WATCHDOG DEAD ZONE (§13.4). CURRENT FRONTIER: a combined
diagnostic capture build (wstg breadcrumb for A + the §13.7 always-on boot-status/boot-count/heartbeat for B) is
being flashed; no A or B fix ships until a capture confirms A's mechanism (`wstg=1` stable) and discriminates B's
(B-1 dead-zone / B-2 executor-death / B-4 brownout / re-enum-failure). This document is the running record so the
investigation can be resumed cold.

**UPDATE 2026-06-28 — Signature A (lost USB-TX wake) FIXED + hardware-confirmed (TIER 1, §17.6, commit `225d4a4`).
Signature B (the hard silent Mode-C wedge) is the OPEN FRONTIER and REPRODUCED on the capture build (§17.7, pass 3).
CRUX RESOLVED to a paradox + a plan (§17.8/§17.9): the RWDT genuinely did NOT fire over >60 s (proven by >1 min of
silent reconnects), and the §13.4 dead-zone "dog fed forever" story is WRONG for this event — the software feeder
(`watchdog_feed`, an embassy async task) itself stopped, yet the HARDWARE RWDT still didn't reset. esp-rtos 0.3.0
does NOT touch the RWDT; the RWDT is a hardware RTC-slow-clock counter that should fire if unfed. The
load-bearing mystery is the NON-FIRING hardware dog. NEW LEVER: esp32s3 HAS a SuperWDT (`Swd`, `#[cfg(swd)]`
confirmed) that §13.4 wrongly said didn't exist — it is hardware-independent and currently unarmed. PLAN (§17.9): arm
the SuperWDT as the primary B-capture instrument (fires where the RWDT didn't ⇒ B becomes readable; doesn't fire ⇒
near-proof of a non-digital hang, escalate to power/PHY). Also a §17.7 byte-level CORRECTION: the "mid-write
truncation at the freeze" was a stale boot-replay cut at skirnir's 64 B RX boundary, not a death; the true
fingerprint is "status dies at +1585 s, ack drumbeat ONSETS (last gaps 2.6 s/1.9 s), then hard silent" — an
A-family stall that progressed to a hard lock, not an unrelated mode.**

**UPDATE 2026-06-26 — the user's SILENT GCODE-SKIP (the priority that outranked the lockup, §14) is RESOLVED in
favor of §16 (RENDER ARTIFACT), firmware exonerated.** The §15 firmware mid-block RMT truncation was tested
on-board with a synthetic max-exposure stimulus (~25-40k `emit_burst` calls across 17 multi-burst blocks, clean
wedge-free run) and fired ZERO truncations (§15.10) — confirmed on a 2nd F3000 pass. So the "missing chunks" the
user sees on-screen (steppers never connected) are skirnir's status-sampled live-trail render (commit `8b4e08c`,
~10 Hz sampling, non-deterministic by construction; §16), NOT lost motion. Firmware executes every line. → host
render fix (skirnireng). **STILL TO FIX regardless (§15.6): the `let _ =` swallow at `motion.rs:786` is a confirmed
silent-failure landmine → feed-hold+ALARM before any real cut.** The LOCKUP (§11-13, Signatures A/B) remains OPEN
and separate.

Date opened: 2026-06-24. Stack: esp-hal `1.1.1`, esp-rtos `0.3.0`, embassy-executor `0.10`, embassy-sync `0.8`,
esp-bootloader-esp-idf `0.5.0`, esp-backtrace `0.19.0`, target `xtensa-esp32s3-none-elf`, `CpuClock::max()` (240 MHz).

---

## 1. Symptom

Streaming a GCode program over USB CDC (USB-Serial-JTAG, `USB_DEVICE`) to the firmware **locks the controller up at
a non-deterministic point**. The DRO/status stops updating, no further `ok`s are emitted, and (in the worst mode)
the board is dead until physically reset. Reproduced with two CAM files:

- `T1_Test.tap` (500 lines, arc-heavy) — committed at `crates/firmware-core/tests/fixtures/T1_Test.tap`.
- `128-Pikachu.tap` (4474 lines, ~4465 moves, almost all short G1, only 4 arcs) — **uncommitted, untracked, and per
  the owner's instruction NOT to be committed or used in any automated test.** It is the more reliable reproducer
  (far more RMT step bursts per second on X).

Non-deterministic line across runs (e.g. 401 / ~201 / 835 / 1387 / ~1500) ⇒ **not** a deterministic bad GCode line.
The class is memory corruption / a race / a hardware hang, not motion math (the pure planner/segment/gcode pipeline
is exonerated by host tests — see `crates/firmware-core/src/planner.rs` `t1_test_*`).

---

## 2. The three observed failure modes

The diagnostic instrumentation (below) revealed the single "lockup" is actually **three distinct failure modes**:

### Mode A — core-1 RMT channel-0 (X) `wait()` hang
- **Breadcrumb:** `stage=axis0:wait_begin` with no matching `wait_done`.
- **Mechanism:** the core-1 motion executor spins forever in esp-hal's blocking RMT TX-completion `wait()` for
  channel 0 — the RMT TX-END event never fires.
- **Characterization:** rare on the original build (~30 min); became a ~10 s repro on the burst-cap build; channel 0
  (X) specifically; mid-detail during normal cutting; correlated with burst density.
- **Status:** the RMT `wait()` is now **timeout-bounded** (commits `8e70c45` + `a829c8a`) — on a 2 s hang it captures
  the ch0 RMT registers and resets. Hypotheses (full-48-block boundary, refill/wraparound race) were investigated;
  the burst-cap fix (`c676a3d`) did NOT stop it, so the full-block theory was wrong/incomplete. **Has not recurred
  since the CCOUNT-timeout build** (inconclusive — the register-capture run never landed because the next wedge was
  Mode C). The decisive data — `[MSG:CRASH rmt0: end=…]` — has not yet been captured.

### Mode B — soft core-0 comms wedge (RECOVERS)
- **Breadcrumb:** `core0-comms-wedge stage=idle_waiting comms-froze-first` (motion executor IDLE, queue drained
  empty — so NOT parked on a full planner; core-0 comms path parked on some `.await`).
- **Mechanism:** a core-0 comms task is stuck on an await; the planner drained and the executor went idle.
- **Status:** the **task-watchdog** (commit `06977bb`) catches it (comms-progress heartbeat freezes → withhold the
  RWDT feed → reset) and the boot dumps the breadcrumb. A per-task **comms-stage breadcrumb** (commit `b642713`)
  was added to pin WHICH core-0 `.await` is stuck — **not yet captured on the board** (Mode C intervened).

### Mode C — HARD silent wedge (DOES NOT RECOVER) ← the original "EN-button-only" bug
- **Breadcrumb:** NONE. The board goes completely silent; skirnir sees total inbound silence (not a visible
  reset-loop — no boot-banner bytes). The RWDT does **not** recover it. Requires a physical EN/power reset.
- **Reproduced on the bench 2026-06-24** via CLI Pikachu streaming: wedged on the first run (~20 s) and stayed dead.
- **Matches the owner's original report:** "unplugging USB didn't help, only the reset button worked."
- **Leading hypothesis:** a panic into esp-backtrace's handler. esp-backtrace `0.19.0` default post-panic is
  `arch::interrupt_free(|| loop {})` (verified in its source) — disable interrupts and spin forever, **no reset, no
  breadcrumb written.** Open puzzle: a panic-halt *should* still let the hardware RWDT fire (~8 s, feed task dies →
  not fed → reset), but it didn't — so either the panic path masks the watchdog, it is a silent boot-loop (re-wedge
  before serial comes up), or it is a non-panic hardware lockup. Undetermined.
- **Status:** invisible today (nothing writes a breadcrumb for a panic/halt). **This is the current frontier.**

---

## 3. Instrumentation & fixes landed (commits)

Firmware (`crates/firmware`, `crates/firmware-core`):
- `ed18de7` — initial lockup watchdog + diagnostics scaffolding.
- `1dcffc1` — **RTC_FAST crash breadcrumb + boot dump.** `#[ram(unstable(rtc_fast, persistent))]`
  `portable_atomic::AtomicU32` array (the only `Persistable` atomic), survives a watchdog/CPU reset (NOT a
  power-cycle / EN reset). Holds the last motion-executor stage marker, a liveness ring, a validity magic. Boot
  emits `[MSG:CRASH …]` over the **normal grbl TX** (not raw esp-println), replayed on first `$I`/`?`.
- `06977bb` — **task-watchdog feed.** Feeds the RWDT only on real forward progress: a `COMMS_PROGRESS` heartbeat
  bumped by `status_responder`/`usb_tx`/`comms_consumer` (not self-bumped), gated by `RX_ACTIVITY` (host-active),
  plus the existing core-1 `MOTION_LIVENESS` check. Withholds the feed (→ RWDT fires) on a core-0 comms stall or a
  core-1 motion stall. Reset-loop guards: host-active seeded inactive + a 6 s sticky RX window.
- `c676a3d` — `firmware-core` `MAX_SYMBOLS_PER_BURST` 47→46 (never completely fill the 48-symbol RMT block).
  **Did NOT fix Mode A** — kept as a harmless hazard-removal.
- `8e70c45` — bound the RMT `wait()` with a 2 s poll-loop; on timeout capture ch0 RMT registers (`int_raw`
  `ch_tx_end`/`thr`/`err`, `ch_tx_status` FSM, `ch_tx_conf0`, the burst symbol count + a burst-seq) → `[MSG:CRASH
  rmt0: …]`, then `software_reset()`.
- `a829c8a` — **time the RMT wait off the Xtensa CPU cycle counter (`get_cycle_count()`), NOT `embassy_time::Instant`.**
  See §4: `Instant::now()` FREEZES inside the non-yielding core-1 InterruptExecutor busy-spin.
- `b642713` — **per-task core-0 comms-stage breadcrumb.** Each of 5 core-0 tasks records its own park-point before
  every await; boot dump gains `comms-stage=<x>` + a `[MSG:CRASH comms: rx=… line=… con=… tx=… sta=…]` line.
  Also verified `software_reset()` (CoreSw / `RTC_CNTL_SW_SYS_RST`) DOES preserve RTC_FAST `persistent` data on the
  S3 (esp-hal `persistent` doc, S3 TRM §6.1.1, esp-idf `RTC_NOINIT` analogue).

Host (`crates/skirnir`):
- `c9145ea` — **write-stall timeout** (`WRITE_STALL_TIMEOUT` 5 s): a wedged controller that stops draining its RX
  surfaces as a `TransportError::Unresponsive` disconnect instead of the engine spinning on a forever-pending write.
- `21f075d` — **response-silence timeout** (`RESPONSE_STALL_TIMEOUT` 5 s): the OTHER wedge mode — the firmware still
  accepts writes (usb_rx alive) but stops responding; total inbound silence ⇒ `Unresponsive` disconnect. Armed from
  connect, so a fresh connect to an already-wedged board no longer hangs in Connecting.

---

## 4. Key technical findings (load-bearing; do not re-derive)

1. **`embassy_time::Instant::now()` freezes in a core-1 InterruptExecutor busy-spin.** A non-yielding busy loop on
   the high-priority InterruptExecutor masks the timer interrupt that advances esp-rtos/embassy time, so `Instant`
   does not progress *during the spin* (Ticker/Timer elsewhere on core 1 work because they `.await`/yield). Any
   busy-poll timeout on core 1 MUST use `esp_hal::xtensa_lx::timer::get_cycle_count()` (per-core CCOUNT, immune to
   the spin). 480M cycles = 2 s at 240 MHz; compare with `wrapping_sub` (one u32 wrap ≈ 17.9 s).
2. **`software_reset()` (CoreSw) preserves RTC_FAST `persistent` memory on the S3; EN-button / power-cycle / brownout
   do NOT.** So the operator must let the watchdog (or software reset) recover a wedge to read the breadcrumb — never
   press EN/power, which wipes it.
3. **esp-backtrace 0.19.0 default post-panic = `interrupt_free(|| loop {})`** — disable interrupts + spin forever. A
   panic is therefore a permanent silent hang with no breadcrumb. (Fix candidate: `custom-halt` feature or a custom
   `#[panic_handler]` that records a panic breadcrumb + `software_reset()`.)
4. **esp32s3 `SocResetReason` variants are `CpuSw`/`CpuRtcWdt`/`CpuMwdt0`/`CpuMwdt1`** (NOT `Cpu0*`). This API-name
   guess bit three times — verify all esp-hal/PAC names against the *installed* source, not memory.
5. **esp-hal RMT `poll()` is genuinely non-blocking** (single `get_tx_status()` read). So a hung `wait()` is the
   hardware never asserting TX-END, not `poll()` looping.
6. **RWDT init is correct:** `enable()` sets Stage0 = `ResetSystem` + `wdt_en`; `set_timeout(Stage0, 8 s)` arms it.
   Not a stage-action bug. Yet it did not recover Mode C — see §2 open puzzle.
7. **Separately-flagged latent bug (NOT a lockup cause):** `RmtStepSink::emit_burst` encodes axes 0/1/2 but iterates
   `0..AXES`(=4), so axis 3 (A) transmits stale all-end-marker `scratch[3]` (completes instantly; harmless without A
   motion, but A would never step). Fix before any 4th-axis bring-up.
8. **Pure-logic ruled out:** the `firmware-core` parse→plan→segment pipeline streams the full real file to
   completion in host tests, no hang/NaN/zero-completion. BlockQueue (`heapless::Deque<_,32>`), arc state machine,
   the [[xtensa-stack-top-abi-headroom]] arena (present + canary), AXES==4 consistency, RMT silent-symbol — all clean.

---

## 5. Breadcrumb / capture reference

Build & flash (Xtensa toolchain is installed locally: `source $HOME/export-esp.sh`):
```
just build [--features defmt]      # both configs build clean under RUSTFLAGS="-D warnings"
just flash                          # plain (no defmt); breadcrumb path is plain [MSG:] + atomics, no defmt needed
```
Capture procedure: flash → stream the file → on a wedge, **wait ~11–12 s; do NOT press EN / power-cycle** (that
wipes the RTC breadcrumb) → the board self-resets (RWDT or the RMT-timeout software_reset) → the next boot prints
the `[MSG:CRASH …]` lines (also replayed on first `$I`/`?`). Read them via skirnir's console or
`skirnir --cli <port>` (connect, no gcode, short `--timeout`).

Boot dump format:
```
[MSG:CRASH <verdict> stage=<motion-stage> comms-stage=<core0-stage> <which>-froze-first beats comms=.. motion=.. (RWDT-reset; not power-cycle)]
[MSG:CRASH rmt0: end=<0/1> thr=.. err=.. fsm=.. nsym=.. burst#=.. ir=0x.. is=0x.. st=0x.. cf=0x..]   (only on an RMT-wait timeout)
[MSG:CRASH comms: rx=.. line=.. con=.. tx=.. sta=..]                                                   (per-task core-0 park-points)
```
Reading it: `stage=axisN:wait_begin` ⇒ Mode A (RMT ch N). `stage=idle_waiting` + a non-idle `comms:` slot ⇒ Mode B
(that slot names the stuck core-0 await). `rmt0: end=1` ⇒ TX-END fired but our wait missed it (driver bug); `end=0
& fsm≠0` ⇒ still transmitting, never completed (encoding/start). NO breadcrumb at all + silent board ⇒ Mode C.

**Bench note (2026-06-24):** automating the repro via `skirnir --cli` and reconnecting on a fixed port name is
fragile — a watchdog reset re-enumerates the USB and the `/dev/cu.usbmodem*` name can change. The board's port was
`/dev/cu.usbmodem31101`; two other `/dev/cu.usbmodem203NT*` nodes are unrelated devices.

---

## 6. Open questions / next steps

1. **Catch Mode C (the hard wedge) — LANDED 2026-06-25 (compiled both configs).** A custom `#[panic_handler]` now
   replaces esp-backtrace's silent `interrupt_free(|| loop {})` halt: it records a PANIC breadcrumb (file-string
   `.rodata` pointer+len, line, panicking core, build id) into RTC_FAST with MINIMAL stack (a handful of raw stores,
   no formatting/locks — safe even after a stack overflow), then `software_reset()`s (CoreSw, RTC-preserving). The
   next boot recovers the file string via the stored pointer (guarded by a per-build `BUILD_ID` so a stale pointer
   from a different image is not dereferenced) and emits `[MSG:CRASH panic <file>:<line> core=<N>]`. esp-backtrace's
   `panic-handler` feature was dropped (kept `println`); esp-hal owns the exception vector and routes faults into our
   handler, so hard-fault/illegal-instruction capture is not lost. **The core-1 stack arena was bumped 16→32 KiB**
   (the likely Mode C root — see §7) — on Xtensa there is NO separate interrupt stack, so the `motion_executor` RMT
   call chain runs on this arena; doubling it is cheap streaming-load-overflow insurance. The `_abi_headroom`
   canary/padding fix is kept (independent of depth).
2. **Why didn't the RWDT recover Mode C? — ANSWERED.** A core-1 panic into esp-backtrace's `interrupt_free(|| loop {})`
   disables interrupts on CORE 1 only; **core 0 keeps running and keeps FEEDING the RWDT**, so the dog never fires →
   hard wedge. (The RWDT is correctly armed and not touched by esp-rtos; it DID fire for Mode B, where core 0 itself
   wedged.) The custom panic handler resolves this directly: it `software_reset()`s the WHOLE chip from the panicking
   core regardless of which core panicked, so a core-1 panic no longer relies on core 0 stopping its feed.
3. **Land the Mode A / Mode B / Mode C captures.** The `rmt0:`, `comms:`, and now `panic` lines are instrumented but
   not all yet read on the board. A clean capture of each remains outstanding; the panic line is the new frontier.
4. **Suspect surface for the underlying corruption/hang:** the esp-hal 1.1.1 RMT driver (TX-END never firing), the
   esp-rtos dual-core / InterruptExecutor interaction, a core-1 stack overflow (now caught), and any esp-hal/esp-rtos
   multicore time/sync hazard. See the companion deep-research report.

---

## 7. Prior-art research (deep-research, 2026-06-24, adversarially verified)

Searched esp-rs/esp-hal, esp-rtos, embassy, esp-idf issues/PRs/changelogs. Key verified findings:

- **Mode C is almost certainly a PANIC.** esp-backtrace `0.19.0` default post-panic is `arch::interrupt_free(|| loop {})`
  — no reset, no watchdog feed, hangs the core until physical reset (verified 3-0, confirmed 0.19.0 + main). So any
  panic produces exactly Mode C's silent-dead-EN-only symptom.
- **PRIME SUSPECT: core-1 (APP_CPU) stack overflow.** esp-rtos `0.3.0` explicitly panics if the second core's `main`
  overflows its stack — its startup spins on the core-1 init flag then panics naming "main stack overflow" (verified
  3-0 from source). Combined with the esp-backtrace halt = board dead. Ties directly to the project's prior
  [[xtensa-stack-top-abi-headroom]] core-1 stack corruption history. **The core-1 stack arena has NOT been
  re-validated under sustained streaming load — likely too small; deeper call stacks during sustained RMT generation
  would overflow it.**
- **`#[ram(rtc_fast, persistent)]` IS reliable across resets** — the "unreliable across deep-sleep" claim was REFUTED
  (0-3). So the breadcrumb approach is sound.
- **Mode 2 mechanism confirmed plausible:** a non-yielding high-priority Embassy InterruptExecutor task starves
  Embassy and can stall `embassy_time` (driver owned by esp-rtos under the `embassy` feature). Validates the CCOUNT
  timeout fix (`a829c8a`).
- **Mode A has NO direct esp-hal Rust prior art.** Closest is ESP-IDF #10429 (S3 RMT silently stops, TX-done never
  fires) — but triggered by LARGE-buffer memory-block wrap (256 symbols fails, 64 WORKS), not many short transmits;
  our bursts are ≤48, which WEAKENS the wrap-race theory. esp-hal #2115 is a DIFFERENT bug (missing end marker, fixed
  in 0.22.0). Mode A stays unexplained by prior art.
- **RWDT should fire independent of CPU interrupt state** (verified): an interrupt-disabled panic loop does NOT mask
  the hardware RWDT reset. So a non-recovering RWDT means it "was never armed or fed" — but we DID arm it and it DID
  fire for Mode B, so the live question is whether a surviving core 0 keeps FEEDING it during a core-1-only panic.
  Re-validate that the RWDT is armed/counting under esp-rtos and not fed across a core-1 panic.
- **RTC reset-reason caveat:** a USB-Serial-JTAG / DTR-RTS-toggle reset (reason `0x15`) WIPES RTC (ESP-IDF #8889);
  watchdog and `software_reset` (CoreSw) PRESERVE it. Do NOT read the breadcrumb with a tool that toggles DTR/RTS.

Sources (primary): esp-rs/esp-hal #2115 #707 #633 #269 #2516 #10324, esp-rs/esp-wifi-sys #437, esp-rs/esp-backtrace,
esp-rtos 0.3.0 lib.rs, esp-hal-embassy 0.8.1 time_driver, embassy #3758 #2603, esp-idf #10429 #8889.

### Refined next step (highest value) — BOTH LANDED 2026-06-25
1. **Custom panic handler — DONE.** On panic, records the panic location (file ptr+len, line, core, build id) into
   RTC_FAST + `software_reset()` (CoreSw, verified RTC-preserving — esp-hal's `persistent` macro doc names
   `software_reset()` first in its survivable list; S3 TRM: all resets except Chip Reset preserve internal memory).
   Boot emits `[MSG:CRASH panic <file>:<line> core=<N>]`. `core=1` ⇒ a core-1 (APP_CPU) panic = the stack-overflow
   prime suspect.
2. **Increase the core-1 stack arena — DONE (16→32 KiB).** Verified against installed esp-rtos 0.3.0 + xtensa-lx-rt:
   the Xtensa SWI handler runs on the interrupted thread's stack (no dedicated interrupt stack), so the
   `motion_executor` deep RMT call chain is charged to the `start_second_core` `Stack<N>` arena — bumping it is the
   correct (and only) lever for streaming-load depth headroom. esp-rtos guard-checks this stack and panics on
   overflow (now visible via the handler), so if the next panic line names a core-1 frame, this is the confirmed fix.

### Capture procedure for the panic line (Mode C)
Flash (`just flash`, no defmt needed — the panic line is plain `[MSG:]` over the grbl TX), stream Pikachu. On a
wedge, **wait ~12 s** (the panic handler resets near-instantly, but if the wedge is a non-panic hang the RWDT takes
~8 s) — do NOT press EN / power-cycle (that wipes RTC_FAST). After the self-reset, the console shows the banner then
`[MSG:CRASH panic <file>:<line> core=<N>]` (also replayed on the first `$I`/`?`). `core=1` + a `motion.rs`/esp-hal RMT
file ⇒ core-1 stack overflow / a panic in the motion path; `core=0` ⇒ a panic in the comms path. The line/file pins
the exact panic site.

---

## 8. Live capture 2026-06-25 — wedge after a single `$G`, plus a spurious-ack fault

### 8.1 The capture (skirnir UI/status lines, in order)
```
state: Disconnected
disconnected: controller not responding (firmware may be wedged)
link dropped; reconnecting to /dev/cu.usbmodem31101 in 1.2s (attempt 3)
reconnecting to /dev/cu.usbmodem31101…
> connect /dev/cu.usbmodem31101 @ 115200
state: Connecting
state: Idle
> $G
state: Disconnected
disconnected: controller not responding (firmware may be wedged)
link dropped; reconnecting to /dev/cu.usbmodem31101 in 0.3s (attempt 1)
fault: received an acknowledgement with no line in flight (spurious ok/error)
```

### 8.2 Confirmed facts (with evidence)
1. **The handshake fully completed before the wedge.** `Connecting → Idle` means readiness
   evidence was adopted and the counted `$I` was acked (`core.rs::on_response` Connecting path
   + `on_ack`). So USB enumeration, RX, the line framer, the consumer, and `usb_tx` were ALL
   alive at `Idle`. (Evidence: the state sequence; `crates/skirnir/src/protocol/core.rs:365`.)
2. **`$G` is pure core-0 and never touches motion/RMT/BlockQueue.** Handler:
   `SystemCommand::ParserState → send_parser_state → enqueue([GC:..]) → ack()`
   (`comms.rs:3552`, `:4245`, `:4297`). No `HOME_REQUEST`/`PROBE_REQUEST`, no planner, no
   `RmtStepSink`. So **`$G` cannot itself reach the Mode-A RMT ch0 wedge** (§2 Mode A) — that
   path requires step bursts, which `$G` does not generate.
3. **The CRLF/LFCR double-`ok` bug is structurally excluded.** `firmware-core`
   `LineReader`/`StreamEngine` collapse `CR`/`LF`/`CRLF`/`LFCR` (even split across feeds) into a
   single terminator, host-tested (`firmware-core/src/protocol.rs:900-985`). `$G\r\n` yields one
   line → one `[GC:]` → one `ok`. The spurious ack is **not** a terminator artifact.
4. **The crash-report replay path emits no `ok`/`error`.** `take_pending_crash_report`
   (fired on first `$I` and first `?`, `comms.rs:4238`,`:4440`) enqueues only `[MSG:CRASH …]`
   BRACKET messages. So the watchdog/panic boot-dump replay is **not** a source of the spurious
   ack. (DISPROVES the "stale ok flushed by the boot dump" sub-hypothesis from the brief.)
5. **The firmware emits exactly one terminal response per consumed line** through a single
   `RESPONSE` channel (depth 8) drained by one `usb_tx` task (`comms.rs:118`,`:4297`,`:4345`).
   There is no second ack producer for `$G`.

### 8.3 Same-bug-or-new verdict
**Two separate phenomena, neither is a clean repeat of the confirmed Mode-A RMT wedge.**

- The **wedge after `$G`** is NOT reached through the `$G` line's own code path (fact 2). Two
  live HYPOTHESES, not yet discriminated:
  - **H-W1 (incidental exposure):** core-1 (or core-0) was ALREADY wedged from the *prior*
    stream — note the capture opens mid-recovery ("attempt 3", "controller not responding").
    `Idle` was reached only because `?`/`$I` are answered by the *status reporter* / core-0
    comms, which can still run while core-1 motion is dead and the BlockQueue is empty. `$G`
    then elicited a `[GC:]`+`ok` that never arrived because the wedge had progressed to also
    take down the TX path — or the board self-reset mid-reply. `$G` is the messenger, not the
    cause. This is the **leading** hypothesis (it explains the mid-recovery framing and fact 2).
  - **H-W2 (core-0 comms stall on `$G`):** a genuine core-0 `.await` stall in the `$G`
    enqueue/ack path (e.g. `RESPONSE.send().await` blocking because `usb_tx` is stuck — the
    `ConsumerEnqueue` stage, `comms.rs:4303`). This is Mode B's family, recoverable by the
    task-watchdog. Discriminated by the boot `[MSG:CRASH … comms-stage=…]` line.
  - Either way this is **Mode A-or-B territory reached without motion**, OR a *new* core-0/TX
    stall — the breadcrumb decides. It is **not** proven to be the RMT-ch0 wedge.

- The **spurious-ack fault** is a **distinct, host-side accounting issue** (see §8.4), most
  likely NOT a firmware over-emission.

### 8.4 Root cause of the spurious ack — leading hypothesis (host-side accounting gap)
**HYPOTHESIS (well-supported by code reading; bench-unproven):** the spurious-ack fault is a
**skirnir cross-disconnect mis-accounting**, not a firmware double-`ok`.

Mechanism, traced through `crates/skirnir/src/protocol/core.rs` + `flow.rs` + `engine.rs`:
1. At `Idle`, the user `$G` is sent counted: `on_send_line` → `flow.on_line_sent` (1 line in
   flight, kind `Other`), `trailing_acks=0`, `ignore_stray_acks=false` (`core.rs:266-286`).
2. The firmware wedges/slows; inbound goes silent for `RESPONSE_STALL_TIMEOUT` (5 s) →
   `engine.rs:314` `break Some(Unresponsive)`. **Crucially, the firmware may have already
   queued the real `[GC:]`+`ok` for `$G`** into its `RESPONSE` channel / USB-Serial-JTAG FIFO;
   the silence timeout is a host liveness *guess* and does not prove nothing was emitted.
3. Teardown: `on_disconnected` → `reset_window(false)` (`core.rs:213-220`). The **`false`**
   grants **neither** `trailing_acks` (banner-path only) **nor** `ignore_stray_acks` (reset-path
   only). The in-flight `$G` is dropped from accounting with **zero ack tolerance**.
4. Native-USB re-enumeration; reconnect attempt 1: `on_connected` zeroes both tolerances again
   (`core.rs:181-182`), handshake sends a fresh counted `$I`.
5. The firmware's **buffered `ok` from before the disconnect** is delivered across the
   re-enumeration. The host attributes the first ack to the new `$I`, then a SECOND leftover ack
   (the `[GC:]` line's `ok`, or a buffered straggler) hits `flow.on_ack` with an empty deque →
   `UnexpectedAck` → the fault. (`flow.rs:84-88`, `core.rs:421-436`.)

The gap is explicit in the code: **`trailing_acks` is granted only on the banner path; the
`ignore_stray_acks` latch only on host-issued `0x18`/`0x86`.** An **Unresponsive disconnect**
(the silence/write-stall timeout) takes neither, so a legitimately-buffered ack crossing that
boundary has no tolerance budget. This is the single most likely root cause and it is a
host-side bug, not a firmware contract violation.

Competing alternative (must be disproven, not assumed away): **the firmware genuinely
double-emits** an `ok` for some line (a real over-ack). Fact 3 excludes the CRLF cause and fact
5 excludes a second producer for `$G`, but a double-ack on a *different* line (e.g. an
error-then-ok, or a reset/banner interleave) is not yet ruled out by capture — only by reading.

### 8.5 The single next experiment (discriminating)
**Add a raw-byte TX/RX log on BOTH ends and re-capture the `$G` wedge + reconnect.** This
splits the host-accounting hypothesis from a firmware over-ack in one shot:

- **Host side (skirnir):** a timestamped raw-bytes-in / raw-bytes-out log spanning the
  disconnect→reconnect boundary (every byte read from and written to the port, with the
  connection-attempt id). Expected results:
  - *If host-accounting gap (§8.4):* the firmware sends exactly one `ok` per line overall, and
    the "spurious" `ok` is a real, pre-disconnect-buffered `ok` delivered just after reconnect
    while the host's window holds only the new `$I`. Byte count of `ok`s == byte count of lines
    sent across the whole session.
  - *If firmware over-ack:* two `ok`s are seen for one line *within a single connection*, before
    any disconnect — a genuine contract violation.
- **Firmware side:** let the wedge self-recover (~12 s; do NOT press EN) and read the boot
  `[MSG:CRASH …]` line on the next `$I`/`?`. The `stage=`/`comms-stage=`/`panic` fields say
  WHICH wedge `$G` exposed (Mode A `axisN:wait_begin` ⇒ a prior-stream RMT hang the `$G` merely
  surfaced; `idle_waiting` + a non-idle `comms:` slot ⇒ Mode B core-0 stall; `ConsumerEnqueue`
  in the `con=`/`tx=` slot ⇒ the `RESPONSE.send().await`/`usb_tx` chokepoint; a `panic` line ⇒
  Mode C). NO breadcrumb + silent board ⇒ Mode C (panic/halt).

Repro harness (per project rule — never ad-hoc cat/printf): a short `skirnir --cli
/dev/cu.usbmodem31101` connect (no gcode, short `--timeout`) to read the boot dump, and a
`skirnir --cli … <small-gcode-that-streams-then-idles>` to recreate the prior-stream-then-`$G`
sequence. Note the watchdog reset re-enumerates USB and the `/dev/cu.usbmodem*` name can change.

### 8.6 What remains UNKNOWN
- Whether the `$G` wedge is incidental exposure of a prior-stream wedge (H-W1, leading) or a
  genuine core-0 stall on the `$G` path (H-W2). The boot breadcrumb decides.
- Whether the firmware EVER double-emits an `ok` on any line (the §8.4 competing alternative).
  Only the dual raw-byte log can disprove this — do not assume the host-accounting story until
  the firmware is shown to emit exactly one ack per line across the boundary.
- Whether the buffered-ok-across-re-enumeration delivery (§8.4 step 5) actually occurs on this
  USB-Serial-JTAG stack, or whether the OS/driver discards the firmware FIFO on close. This is
  the load-bearing assumption of the host-accounting hypothesis and the raw log will show it
  directly.

### 8.7 Proposed fix DIRECTION (do not implement until §8.5 proves the mechanism)
- *If §8.4 confirmed:* grant a bounded stray-ack tolerance across an **Unresponsive** teardown
  the way the banner path grants `trailing_acks` — e.g. on `on_disconnected`/`on_connected`,
  seed a small count-bounded budget equal to the lines that were in flight at disconnect, so a
  legitimately-buffered ack crossing the re-enumeration is absorbed, while a true mid-stream
  double-ok (latch long cleared) still faults. Scope it tightly so it cannot mask a real
  over-ack — mirror the existing `trailing_acks` count-bound rationale in `core.rs:101-117`.
- *If firmware over-ack confirmed instead:* fix the offending emission site; the host guard is
  correct and should stay.

**Verdict for the brief:** (a) **NEW/distinct** from the confirmed Mode-A RMT wedge — `$G`
cannot reach that path; the wedge is most likely incidental exposure of a prior-stream wedge
(Mode A/B), pending the boot breadcrumb. (b) Spurious ack: **leading root cause is a host-side
Unresponsive-disconnect zero-tolerance accounting gap**, not a firmware over-ack — to be
confirmed by a dual raw-byte log. (c) Next: dual raw-byte TX/RX log + read the boot `[MSG:CRASH]`.
(d) Doc updated here (§8); no fix landed yet (mechanism unproven).

## 9. Bench capture — 2026-06-25 (dual raw-byte log on real hardware)

The §8.5 experiment was run on the board (`/dev/cu.usbmodem31101`, esp-hal-1.1 image freshly
flashed). The host-side raw-byte trace from §8.5 was implemented as a `SKIRNIR_RAW_LOG`-gated
TX/RX log in `crates/skirnir/src/transport/serial.rs` (off by default, pure formatter unit-tested;
each `SerialTransport::open` stamps a connection id `cN` so a buffered ack on a *later* connection
is distinguishable from a within-connection double-ack). What the captured bytes establish:

- **Fresh flash boots clean — NO `[MSG:CRASH …]` breadcrumb.** First connect: `<Idle …>`,
  `[VER:1.1f.20260616:]`, `[OPT:VNMSL,32,1024,4,0]`, one `ok`. No carried-over wedge, no panic
  record. So the original capture's `$G` wedge was reached from a *prior-stream* state that this
  fresh boot does not carry — consistent with H-W1 (incidental exposure), not a `$G`-path stall.
- **Firmware emits EXACTLY one `ok` per consumed line — the over-ack alternative is DISPROVEN by
  capture, not just by reading.** A clean stream-then-`$G` program (11 lines incl. `$G`) returned
  `$I`+11 lines → **12 `ok`s**, 1:1, and the `$G` line returned exactly one `[GC:…]` + one `ok`.
  This closes the §8.4/§8.6 competing alternative for the normal G-code + `$G` path: the firmware
  does **not** double-emit. The spurious ack therefore originates host-side (the §8.4
  Unresponsive-disconnect zero-tolerance accounting gap remains the leading mechanism), **but its
  load-bearing step — a buffered `ok` surviving a USB re-enumeration (§8.6 #3) — was NOT directly
  observed here, because no wedge/disconnect was reproduced** (below). That step is still UNKNOWN.
- **Settings on this image:** `$20=0`/`$21=0` (soft/hard limits OFF), so the persistent `Pn:XYZ`
  (floating NC limit inputs, no machine) does **not** gate motion — moves ack and execute. `$100=
  250` steps/mm, `$110=500` mm/min max, `$120=10` accel.
- **No wedge reproduced in sustained real-time streaming (negative result).** A synthetic X-heavy
  stress program (503 short reversing 0.5 mm moves, varied feeds —
  NOT a known-lockup file) **ran to completion: all 503 lines acked, exit 0, ≈211 s**, status held
  `Run` throughout, WPos advanced gradually, feed rates live (`FS:480/120/300…`), planner buffer
  full (`Bf:0,1024`), `ok`s ~1 per block-completion. **Ack accounting end-to-end was 504 `ok`s for
  503 lines + `$I` — perfect 1:1, zero `error`/`ALARM`/`[MSG:CRASH]`/`0x18`.** It passed line ~400
  (the historical wedge zone: T1_Test ≈401, 128-Pikachu ≈835) with no stall. An earlier 40 s run
  *looked* stalled only because the timeout cut it off at line 123/503 while healthy (full program
  ≈160–210 s). A single isolated 8 mm move reported `Idle|WPos:8.000` almost immediately (≈16 ms),
  contradicting the clearly-paced stress motion; treated as a status-report artifact (target
  reported optimistically), not a real timing bypass — flagged, not load-bearing.

- **CORRECTION (2026-06-25, supersedes an earlier "1.0 vs 1.1 confound" written here): there is NO
  esp-hal version confound — both this baseline AND the original diagnosis are on esp-hal `=1.1.1`.**
  An earlier draft of this section claimed "this image is 1.1.1, not the 1.0 the wedge was diagnosed
  on... entangled with a different RMT driver." That premise is WRONG (git, §11.9): the 1.0→1.1
  upgrade merged 2026-06-24 07:11 (`421c58c`), and EVERY breadcrumb/RMT capture — including the lone
  `axis0:wait_begin` marker (`1dcffc1`, 15:26, Cargo.toml pins `esp-hal =1.1.1`) — was on the 1.1.1
  tree. There was never a 1.0 RMT-hang capture to be "diagnosed on." So this clean 211 s run is NOT
  discounted by any 1.0/1.1 driver difference — which *strengthens* the §10 reproduction (same
  version throughout; no version confound to explain away). The negative here is purely a
  stimulus/timing miss (the synthetic stress is not the historical geometry), not a version artifact.
  The remaining caveat is only that it is a *single* run of a *non-deterministic* fault on a synthetic
  program; the real `128-Pikachu.tap` DID reproduce it on this same 1.1.1 image (§10).

**Where this leaves the two questions.** Spurious-ack: firmware over-ack **EXCLUDED by capture**
(504/503+`$I`, 1:1, three runs incl. the 211 s stress); host-side Unresponsive-disconnect
accounting gap is the surviving hypothesis, but its USB-FIFO-survives-re-enumeration premise (§8.6
#3) is **still unobserved** — no wedge/disconnect was triggered to produce a buffered ack. Wedge:
the synthetic stress did NOT reproduce it — **but the real `128-Pikachu.tap` file DID; see §10.**

## 10. WEDGE REPRODUCED on esp-hal-1.1.1 — `128-Pikachu.tap`, 2026-06-25 (raw byte capture)

The synthetic stress was the wrong stimulus. Streaming the real CAM file `128-Pikachu.tap` (4474
lines, Vectric, spindle M3 S18000) on the same fresh esp-hal-1.1.1 image **reproduced the lockup**:
the link died at **777/816 acked** (line ≈816 — squarely the historical ≈835 zone), `outcome=
IoDisconnect`. **So the 1.0→1.1 RMT change did NOT fix the bug** — the §9 negative was a
stimulus/timing miss, not a fix. The `SKIRNIR_RAW_LOG` trace (`/tmp/pika.raw`) captured the exact
death dynamics, which are sharper than the prior "core-1 hangs, core-0 stays alive" sketch:

- **Healthy right up to a sudden total freeze.** Through +495.083 s the stream is fast and normal:
  `ok`s ~80–100 ms apart, `<Run …>` status answered every ~100 ms, planner buffer full
  (`Bf:0,1024`), WPos advancing. The last status is at +495.003 s, the last normal `ok` at
  +495.083 s, on the line `G1X166.930Y124.724Z-5.000` — **an ordinary ~0.5 mm G1 segment, no
  spindle/Z/dwell/mode change at the boundary.** This re-confirms the earlier "non-deterministic
  line ⇒ not geometry" note: the trigger is not the toolpath.
- **Both response paths freeze together; only the ack path partially recovers.** At ≈+495.083 s the
  firmware stops answering *everything*. The host keeps polling `?` (163 queries over the next
  16 s) and gets **zero** status replies — the core-0 status reporter is dead and **never recovers**
  for the rest of the run. The ack/line path, by contrast, resumes at a **precise 1 `ok` per
  ≈2.00 s drumbeat**: +497.085, +499.088, +501.090, +503.092, +505.096, +507.099, +509.102 s.
- **A ~2.00 s period is the key new lead.** One motion block is released every ≈2 s while the status
  reporter stays dead — the signature of a **~2 s timeout repeatedly un-wedging exactly one block,
  then re-hanging**. This fits the prior root cause (core-1 stuck in esp-hal RMT `wait()` for ch0,
  TX-END never firing) *with its recovery dynamics now visible*: a 2 s timeout force-completes a
  block, the next block re-hangs. **Next firmware question: what in the motion/RMT/executor path has
  a ~2 s timeout, and why does the core-0 status reporter never recover (shared lock / starvation
  with the stalled executor)?**
- **Ends in a burst-drain then a reset.** At +511.103 s (~16 s / ~8 drumbeat cycles in) the whole
  remaining RX buffer suddenly acks in a ~200 ms burst (~30 `ok`s), immediately followed by a
  truncated mid-line host write (`"G1X151.882Y99"`, 13 B) and a USB drop — i.e. the board reset /
  re-enumerated. This is *not* a clean task-watchdog reset at the first stall (that would fire at
  ~2 s, not after 16 s of drumbeat).
- **Ack contract held to the very end.** Over the whole run: 813 program lines → 778 `ok`s with
  35 still in-flight at the drop, **zero `error`/`ALARM`, zero double-`ok`** — the firmware never
  over-acked even while wedging. (Consistent with §9's over-ack exclusion.)

**Recovery / breadcrumb — NOT captured cleanly (procedure note).** After the drop the board did
**not** cleanly recover over USB: two plain `skirnir` reconnects got **silence** (no banner, no
`[MSG:CRASH …]`, port re-enumerating 14:16→14:17). A clean `Grbl 1.1f` banner only returned after
`espflash monitor` forced a hard `rst:0x15 (USB_UART_CHIP_RESET)` — which **wipes RTC_FAST and
therefore destroyed any crash breadcrumb** (boot then showed `Saved PC:0x40378eb1`, `reset reason:
PRO_CPU=other, APP_CPU=other`, no `[MSG:CRASH]`). Lesson for next time: to read the breadcrumb,
reconnect with **skirnir only** and NEVER attach `espflash`/press EN — but note that here the plain
reopen was silent, so the post-wedge firmware may not be able to emit a breadcrumb over USB at all
(USB-CDC or core-0 taken down with the wedge). That the board stays silent until a hard chip reset
is itself a finding: the wedge is not a clean core-1-only hang with a healthy core-0 comms path.

**Reproduce it again:** `SKIRNIR_RAW_LOG=1 skirnir --cli <port> 128-Pikachu.tap --idle-timeout 12
--timeout 600 2>raw.log` — wedges ~8.5 min in, around line ≈816. (`128-Pikachu.tap` is the repro
vector but is gitignored/never-commit; keep it local.)

---

## 11. The 2 s drumbeat is NOT the RMT-wait timeout — central hypothesis REFUTED (2026-06-25)

The team-lead brief asked to verify the identity "2.00 s drumbeat == the bounded RMT `wait()` timeout
firing in a recovery loop." Worked it to ground against `/tmp/pika.raw` and the firmware source.
**Verdict: the period MATCHES the RMT constant numerically, but the mechanism is DISPROVEN — the
RMT-timeout path cannot produce the observed drumbeat.**

### 11.1 Confirmed facts (evidence cited)
1. **The period equals the RMT constant to the digit.** Drumbeat `ok` timestamps (s):
   495.083 → 497.085 → 499.088 → 501.090 → 503.092 → 505.096 → 507.099 → 509.102 → 511.103.
   Eight inter-`ok` intervals, each **≈2002 ms** (`/tmp/pika.raw`). The firmware's only motion-side
   cycle timeout is `RMT_WAIT_TIMEOUT_CYCLES = 480_000_000` cycles ÷ 240 MHz = **exactly 2000.0 ms**
   (`crates/firmware/src/motion.rs:423`). The 2 ms excess is host `?`/read scheduling latency. So the
   *number* matches — but matching a number is not a mechanism.
2. **`emit_burst` resets on the FIRST RMT-wait timeout — it does NOT loop.** `motion.rs:370-394`: the
   bounded poll-loop, on `timed_out`, calls `capture_rmt_hang(...)` then `esp_hal::system::software_reset()`
   **unconditionally on the first timeout**. There is no retry-N-times path. One RMT timeout = one whole-chip
   software reset. (Confirmed it is the *only* 480M-cycle timeout and the only motion-path `software_reset`
   in the firmware: `grep` over `crates/firmware/src` → just `motion.rs:394`.)
3. **NO reset occurred during the 8 drumbeat cycles — proven by the absence of a banner and an unbroken
   connection id.** The grbl/CLAUDE.md contract emits the welcome banner on every boot/soft-reset. The
   capture's `[VER:1.1f...]` banner appears **exactly once, at +45 ms** (the initial handshake) and **NEVER
   again** — not at any drumbeat tick, not at the drain, not before `[down]` (`grep -i 'grbl|VER:|MSG:'
   /tmp/pika.raw`). Moreover the entire 8.5-min session, including the whole drumbeat and the final drain,
   is on a **single unbroken connection `c0`** (16600 `c0` lines, ZERO re-opens; `SerialTransport::open`
   stamps a fresh `cN` per port open, so a `software_reset()` re-enumeration would force a new id or a
   disconnect). **Therefore `emit_burst`'s `software_reset()` did not fire during the drumbeat.**
4. **Conclusion:** the RMT-wait-timeout path is logically excluded as the drumbeat source — it would have
   reset+rebanner'd on the FIRST tick. **DISPROVEN.** (The 2.00 s numeric coincidence is real and was the
   trap; it is the *same constant reused elsewhere*, see below.)

### 11.2 The actual leading mechanism — `USB_TX_TIMEOUT` (a SECOND, core-0, 2 s timeout)
There is a second 2 s timeout in the firmware, on **core 0**, that fires repeatedly WITHOUT resetting:
`USB_TX_TIMEOUT = Duration::from_secs(2)` (`comms.rs:1342`), wrapping the USB write in `usb_tx`
(`comms.rs:1368-1374`): `with_timeout(USB_TX_TIMEOUT, { tx.write_all(resp).await?; tx.flush().await })`.

**HYPOTHESIS (leading, evidence-backed, bench-unconfirmed):** the drumbeat is `usb_tx`'s 2 s write/flush
timeout firing in a loop. Each iteration: `RESPONSE.receive()` pulls one queued `ok`, `write_all` pushes its
bytes to the USB-Serial-JTAG `ep1` FIFO (they reach the host at once — which is why the host *reads each `ok`
instantly*, +4 ms after a `?`), then the code `await`s the `serial_in_empty` (TX-done) event. That event
**never fires for ~2 s**, so `with_timeout` aborts, the response is dropped-and-continued, and `usb_tx` loops
to the next item — **releasing exactly ONE `RESPONSE` slot per 2 s**. This reproduces every observed fact:
- One `ok` per ≈2.00 s, host-readable immediately (bytes go out before the await stalls).
- Status `<...>` never returns: 162 `?` polls after the freeze, **ZERO** status replies — because
  `status_responder` enqueues its report via `enqueue() = RESPONSE.send().await` (`comms.rs:4436`) into the
  same depth-8 `RESPONSE` channel drained by the same single `usb_tx`. With `usb_tx` draining one item / 2 s
  and the channel back-pressured, the status line either never gets a slot or sits behind the ack backlog and
  is starved. **The status/ack asymmetry is explained: there is only one writer and one channel; the `ok`s
  win the slots, the reports starve.** (This REPLACES the brief's "shared lock / priority starvation" guess —
  it is simpler: single-writer single-channel head-of-line.)
- The ~30-`ok` burst-drain at +511.103 (27 `ok`s in ~200 ms): when the underlying TX-done event finally
  fires again, the whole backed-up `RESPONSE`/USB FIFO flushes at once.

### 11.3 Why the watchdog did NOT reset for ~16 s (was an open puzzle)
`usb_tx` bumps `COMMS_PROGRESS` **before** the awaited write, once per loop (`comms.rs:1354`). The drumbeat
loops `usb_tx` once / 2 s, so `COMMS_PROGRESS` advances once / 2 s. The comms-stall watchdog samples every
`WATCHDOG_FEED_INTERVAL` = 500 ms and needs `COMMS_STALL_TICKS` = 6 consecutive frozen samples = **3 s** of a
frozen counter to withhold the RWDT feed (`comms.rs:2771,2793,2868-2889`). 2 s < 3 s, so `comms_frozen_ticks`
resets every drumbeat tick and never reaches 6. **The 2 s usb_tx bump cadence sits JUST under the 3 s
comms-stall threshold and masks the stall** — the board limps at 1 block/2 s instead of resetting promptly.
The final reset at ~16 s is most likely the **core-1 motion-stall** path (`CORE1_STALL_TICKS` = 8 = 4 s of
frozen `MOTION_LIVENESS` while `EXECUTOR_RUNNING`) finally accruing once the executor froze with a block in
flight, or the bare RWDT — TBD by the boot breadcrumb.

### 11.4 What this re-frames — the root stall is the missed USB TX-DONE event, gated by core 0
The USB `serial_in_empty`/`serial_out_recv_pkt` interrupt is mapped to **core 0's** interrupt matrix:
`UsbSerialJtag::into_async()` (run in `main` on PRO_CPU before `start_second_core`) → `bind_peri_interrupt`
→ `interrupt::enable(.., Cpu::current())` → `map_raw` writes `core_0_intr_map` (verified in esp-hal-1.1.1
`peripherals/mod.rs:75-109`, `interrupt/mod.rs:336-368`; doc: "enabled on the core that calls this
function"). The single shared `WAKER_TX` is woken only by that core-0 ISR (`usb_serial_jtag.rs:932-961`).
So the drumbeat is core 0 failing to observe/service the `serial_in_empty` event for ~2 s at a time. Two
SUB-HYPOTHESES remain to be discriminated (the new frontier):
- **H-A (lost TX-done wake / event-bit race):** the esp-hal 1.1.1 `serial_in_empty` interrupt-enable /
  `int_clr` / `WAKER_TX.register` sequence drops a wake under the streaming cadence, so the write future
  parks until the 2 s host-side `with_timeout` rescues it. The host reads keep the bytes flowing; only the
  *completion event* is lost. esp-hal-side; would be ch-agnostic and NOT motion-related. (Discriminated by the
  §11.5 breadcrumb, NOT live RTT — there is no usable out-of-band RTT on this board; see the §11.8 transport
  note. The capture is the post-mortem RTC_FAST breadcrumb over the grbl CDC in the default build.)
- **H-B (core-0 starvation by a cross-core hazard):** core 0 is itself blocked from running the `usb_tx`
  poll / fielding the USB ISR for ~2 s spans — e.g. it spins on `PLANNER`/`MACHINE` `CriticalSectionRawMutex`
  contention against a core-1 executor that holds (or churns) the lock, or an esp-rtos SMP scheduler hazard.
  In this case the 2 s gate is set elsewhere and `usb_tx` is a victim. Motion-adjacent.

**Note:** H-A vs H-B is the real open question now, NOT "is it the RMT timeout." The RMT timeout is excluded
(§11.1). The original "RMT ch0 TX-END never fires" finding from 2026-06-24 (RTC breadcrumb `axis0:wait_begin`)
was a *different* capture/build; it may still be a real, separate Mode-A wedge, but it is NOT what produced
the 2026-06-25 Pikachu drumbeat.

### 11.5 The next experiment — the `usb_tx` K-escape breadcrumb (agreed with firmware-engineer, 2026-06-25)
The board is silent post-wedge over USB-CDC, so the boot `[MSG:CRASH]` can't be read over the *wedged* link,
and `espflash`/EN wipes RTC_FAST. Rather than out-of-band RTT, the chosen capture is **in-band via the existing
RTC_FAST breadcrumb + a deterministic self-reset on the K-th sustained `usb_tx` timeout** — readable on the
next boot through skirnir, no JTAG. This works because the drumbeat itself PROVES core 0 schedules `usb_tx`
≥ once / 2 s, so the K-th-timeout snapshot is guaranteed reachable on core 0. Design (firmware tasks #10-12):

1. **`usb_tx` counts CONSECUTIVE `with_timeout` expiries** (reset to 0 on any completed write). On the K-th
   (K=3 ≈ 6 s, inside the 8 s RWDT), it snapshots the decisive registers + state, records a `UsbTxStall`
   breadcrumb word, then `software_reset()` (CoreSw, RTC-preserving). The boot then emits `[MSG:CRASH usb_tx
   …]`. Confirms/kills the §11.2 identity: reaching K=3 at all == `USB_TX_TIMEOUT` is firing in a loop.
2. **The snapshot is the H-A/H-B discriminator.** Captured at the K-th timeout, all on core 0:
   - **`int_ena.serial_in_empty`** — set by `UsbSerialJtagWriteFuture::new` (esp-hal `usb_serial_jtag.rs:715-718`),
     CLEARED by the ISR on fire (`:941`). If it is **still 1** at the snapshot → the ISR NEVER ran for this
     write → **lost-wake / H-A** (future parked, interrupt still armed, host drained but no empty-transition
     edge re-fired). If **0** → the ISR fired but the future still didn't complete → a waker-registration race
     (a different H-A flavor).
   - **`ep1_conf.serial_in_ep_data_free`** — host-drained = IN FIFO free. `=1` confirms the host IS reading
     (matches the capture: drumbeat `ok`s reach the host), isolating the fault to the *completion event*, not
     the byte transfer.
   - **`int_raw.serial_in_empty`** (the raw latched TX-empty event the write future waits on) PAIRED with
     **`int_ena.serial_in_empty`** (is the interrupt still ARMED?). Together they split H-A into TWO flavors —
     because `UsbSerialJtagWriteFuture::poll` (`usb_serial_jtag.rs:736-746`) returns Ready iff `int_ena` is
     CLEAR, and the ISR (`:941`) clears `int_ena` when it fires:
     - `serial_in_empty=1 + int_ena DISARMED (clear)` ⇒ the ISR DID run (cleared the mask + called
       `WAKER_TX.wake()`) but the task was never re-polled ⇒ a **lost waker** (an embassy/esp-hal wake race).
     - `serial_in_empty=1 + int_ena ARMED (set)` ⇒ the ISR NEVER ran for this write ⇒ the `serial_in_empty`
       interrupt was **never serviced on core 0** (masked, or core 0 never fielded it — leans toward a core-0
       scheduling/SMP issue even within the "USB event lost" family). [firmware-engineer's sharpening — kept.]
   - **`MOTION_LIVENESS`-advanced-across-the-window + `EXECUTOR_RUNNING`** — core-1 health. `MOTION_LIVENESS`
     bumps per-burst AND per-loop-turn (`motion.rs:297,527`), so even a slowly-draining executor ticks it many
     times within the ~2 s window; reading "frozen" therefore means core 1 did literally nothing for 2 s =
     genuinely wedged. Advancing ⇒ executor alive ⇒ favors H-A. Frozen with `EXECUTOR_RUNNING` ⇒ core-1 wedge
     upstream ⇒ H-B. (Window note: `motion_before` is sampled at the top of EACH usb_tx loop iteration
     (`comms.rs:1401`), so the compared window is the FINAL ~2 s write, not the full ~6 s — adequate given the
     per-burst granularity, but if a future build wants the whole-stall delta, sample `motion_before` once when
     the counter first arms.)
   - **`RESPONSE` channel depth** — non-empty confirms acks are backed up behind the stalled writer.
   - **The landed `diag::verdict()` order** (`firmware-core/src/diag.rs`, host-tested): `!data_free →
     HostNotReading`; else `!motion_advancing && executor_running → Core1Wedged (H-B)`; else `serial_in_empty →
     LostTxWake (H-A)`; else `Ambiguous`. Core1Wedged is checked BEFORE LostTxWake so a genuine core-1 wedge
     wins regardless of the USB event state — correct precedence.
3. **A `rmt_wait_timeout_count` word** is bumped in the RMT-wait-timeout branch (`motion.rs:387`, kept
   reset-on-first-timeout) and dumped alongside. `usb_tx_timeouts ≥ 3 && rmt_wait_timeouts == 0` POSITIVELY
   nails the §11.1 refutation in the breadcrumb itself: the drumbeat was `usb_tx`, the RMT path never fired.
   This is the *evidence-based* RMT exclusion (vs the §11.1 inference from "no reset across 8 cycles").
4. **Run/read procedure (for the user, via main):** flash the K-escape build (DEFAULT config — `just flash`,
   NO defmt: the breadcrumb is plain `[MSG:]` over the grbl CDC, and there is no usable out-of-band RTT on this
   board, see §11.8). Stream `128-Pikachu.tap`; on the wedge the board self-resets at ≈ 6 s (NOT 16 s —
   intended; this build escapes on the FIRST sustained stall). Read the boot `[MSG:CRASH usbtx: <verdict>
   free=.. empty=.. mov=.. exec=.. rdepth=.. n=.. rmt_to=..]` line via `skirnir --cli <port>` (connect, no
   gcode, short `--timeout`) — do **NOT** attach espflash or press EN (wipes RTC_FAST). Interpret per the
   verdict above.

**STATUS: build #1 LANDED & verified (2026-06-25).** Both Xtensa configs build clean under `-D warnings`
(via `just build` / `just build --features defmt`); the new pure `firmware-core::diag` module has 14 host
tests (incl. all four verdict cases) all passing; K-escape code is clippy-clean. Wired:
`crash.rs::{record_usb_tx_stall,bump_rmt_wait_timeout}`, `comms.rs` K=3 consecutive-timeout escape via
`diag::pack_usb_tx_stall`, `motion.rs:387` `bump_rmt_wait_timeout()`. Awaiting the user's flash go-ahead.

**Build #2 gating (agreed):** the §11.5-#2 cross-core `PLANNER`/`MACHINE` lock acquire/release breadcrumb is
DEFERRED — built ONLY IF build #1's verdict comes back `Core1Wedged` or `Ambiguous` (i.e. points at H-B and we
must localize WHICH contention). If build #1 verdicts `LostTxWake` or the `int_ena`-DISARMED waker-race flavor,
build #2 is unnecessary and we go straight to the esp-hal USB async write path. Keeps build #1 minimal.

(Note: K=3 escaping the stall does NOT *fix* the §11.3 watchdog-mask defect for the production firmware — it is
a diagnostic self-reset for THIS capture build; the per-loop `COMMS_PROGRESS`-bump fix (§11.6) is the real
remediation, deferred until the root flavor is known.)

### 11.6 Proposed fix DIRECTIONS (do NOT implement until §11.5 proves the mechanism)
- The watchdog mask is a real, separate defect regardless of root cause: **`usb_tx` bumping `COMMS_PROGRESS`
  once per (timed-out) loop lets a 2 s-cadence stall evade the 3 s comms-stall detector.** Candidate: bump
  `COMMS_PROGRESS` only on a *completed* write (not on a timeout-drop), or shorten `USB_TX_TIMEOUT` /
  lengthen the detector so a stalled writer is caught. This makes the wedge self-recover in ~3 s instead of
  limping 16 s — independent of whether H-A or H-B is the root.
- If **H-A (esp-hal USB TX-done event loss):** the durable fix is in the USB-Serial-JTAG async write path
  (esp-hal upstream or a firmware-side workaround); the `with_timeout` is already a correct backstop.
- If **H-B (core-0 starvation / lock hazard):** fix the cross-core `CriticalSectionRawMutex` contention or
  scheduler hazard that blocks core 0; the usb_tx symptom then disappears.

### 11.7 Falsification work on the existing capture (narrowing H-A/H-B without hardware)
Two cheap tests run against `/tmp/pika.raw` and the source, trying to KILL the leading theory rather than
confirm it:
- **The drain burst is 100 % `ok`s, ZERO `<...>` status** (`+511.103…+511.320`, 27 `ok`s, no report), despite
  162 `?` polls during the stall. If `status_responder` had run normally and only the *write* were stalled
  (a clean H-A), built reports would have queued into `RESPONSE` and flushed in the drain — they are absent.
  This says reports were essentially **never enqueued** during the stall. (Caveat: `STATUS_REQUEST` is a
  `Signal` that COALESCES — 162 `?` collapse to one pending wake — so at most ONE report would ever be in
  flight; if it parked at `MACHINE.lock().await`/`planner_blocks_free().await`/`enqueue().await` it would
  produce exactly the observed "no reports," and a single trailing report could be lost in the truncated
  final drop. So this weakly favors a core-0-side block but does NOT cleanly split H-A/H-B.)
- **A wedged `emit_burst` does NOT hold the `PLANNER` lock** (FALSIFIES the simple "held-lock starves status"
  sub-theory). The executor scopes the `PLANNER` lock to `take_block` only (`motion.rs:564-572`, pure pop+peek,
  no await) and drops it BEFORE `run_block`/`emit_burst` (line 597). So if core 1 is hung in `emit_burst`,
  `status_responder`'s `planner_blocks_free()` would acquire the lock fine — a held PLANNER lock is NOT why
  status is dead. (A `CriticalSectionRawMutex` *contention spin* or an esp-rtos scheduler hazard is still on
  the table for H-B, but not a simple lock-hold.)

### 11.8 Verdict for the brief
(a) drumbeat == RMT-wait-timeout — **REFUTED** (the RMT path resets on the first timeout; the capture shows
NO reset/banner across all 8 cycles on an unbroken `c0`). (b) The drumbeat is the **`USB_TX_TIMEOUT` (2 s,
core-0, in `usb_tx`)** firing in a loop — a different 2 s constant that recovers one `RESPONSE` slot per
timeout without resetting (leading, to confirm via §11.5 #1). (c) status-stays-dead is **single-writer /
single `RESPONSE` channel head-of-line starvation** (status + acks share one depth-8 channel and one writer),
not a shared-lock guess; the simple held-lock variant is falsified (§11.7). (d) the 16 s-no-reset puzzle is
**`usb_tx`'s per-loop `COMMS_PROGRESS` bump masking the 3 s comms-stall watchdog**. (e) NEW frontier:
H-A (lost USB TX-done wake on core 0, executor healthy) vs H-B (core-0 starvation / SMP hazard, usb_tx a
victim) — discriminate via the §11.5 **in-band RTC_FAST breadcrumb K-escape** (NOT RTT — see the transport
note below), snapshotting `int_ena`/`int_raw.serial_in_empty` + `ep1_conf.serial_in_ep_data_free` +
`MOTION_LIVENESS`/`EXECUTOR_RUNNING` + `RESPONSE` depth at the K-th `usb_tx` timeout, read on the next boot.

**TRANSPORT REALITY (corrects an earlier §11.4/§11.5 note that proposed out-of-band RTT): there is NO usable
out-of-band RTT on this board.** On the ESP32-S3 the defmt sink is NOT a separate RTT channel — esp-println
0.17's S3 backend rides the SAME built-in USB-Serial-JTAG peripheral (`USB_DEVICE`) as the grbl CDC (documented
in `crates/firmware/Cargo.toml:73-82`). You cannot stream Pikachu over skirnir's CDC AND watch defmt over the
one USB port at once — one host owns it and the byte streams would corrupt. A genuine separate-RTT channel
needs an EXTERNAL JTAG probe on the JTAG pads, which this board does not expose (and GPIO39 is taken as the
A-LIMIT placeholder). And for a both-paths-dead wedge a LIVE defmt trace is doubly useless — the dead core-0
sink emits nothing at the moment of death. **So the capture MUST be the post-mortem RTC_FAST breadcrumb over
the grbl CDC in the DEFAULT (no-defmt) build** — exactly the K-escape in §11.5. (Credit firmware-engineer for
catching the RTT error.)

### 11.9 RECONCILING the 2026-06-24 `axis0:wait_begin` breadcrumb — the RMT attribution was OVER-CLAIMED
We cannot hold two contradictory "confirmed root causes." Worked the timeline + the breadcrumb text; the
honest finding is that **the 2026-06-24 RMT-ch0 attribution was a plausible read of an AMBIGUOUS stage marker,
never closed by the decisive evidence** — downgrade it from "CONFIRMED" to "UNPROVEN."

Timeline (git, all 2026-06-24): breadcrumb instrumentation `1dcffc1` landed 15:26; watchdog `06977bb` 16:06;
the bounded-RMT-wait-that-resets `8e70c45` only at 19:21. **The `axis0:wait_begin` capture was on the
UNBOUNDED-wait build** (the old blocking `wait()` that could spin forever) — a different binary from the
2026-06-25 bounded-wait image that produced the drumbeat.

The actual breadcrumb text:
`[MSG:CRASH core0-comms-wedge stage=axis0:wait_begin comms-froze-first beats comms=41898 motion=60943]`.
Two facts in it cut AGAINST the RMT-root reading, and the prior note explicitly *overrode* them:
- The watchdog's own verdict was **`core0-comms-wedge` / `comms-froze-first`** — it said CORE 0 froze first.
  The prior analysis dismissed this as "collateral" and told the reader to "trust the stuck `wait_begin`."
- **`stage=` is just the LAST stage the executor recorded, not proof it is HUNG there.** It reads
  `axis0:wait_begin` whether the executor is (a) genuinely spinning in an unbounded `wait()`, OR (b) merely
  recorded that as its last burst stage then went idle/parked between blocks while core 0 wedged for an
  INDEPENDENT reason. The breadcrumb cannot distinguish these. The one datum that WOULD — the `[MSG:CRASH
  rmt0: end=… fsm=…]` register dump — was **NEVER captured** (§2 Mode A admits this).

Under the USB_TX reframe the SAME breadcrumb is explained with causality INVERTED: a `usb_tx` drain stall →
`comms_consumer` parks on RESPONSE back-pressure → planner fills → the executor DRAINS the queue and goes idle,
its last recorded stage being the last burst it did (`axis0:wait_begin`) → core-0 comms froze first (matching
the `comms-froze-first` label). This fits the breadcrumb at least as well as the RMT-hang story, and better
fits its own verdict.

**Verdict on reconciliation:** NO genuine contradiction once the RMT attribution is correctly downgraded.
Different builds (timeline); and the 2026-06-24 marker is consistent with EITHER an unbounded-wait RMT hang OR
a core-0 usb_tx stall that idled the executor — it was never disambiguated. **The doc's earlier "ROOT CAUSE
CONFIRMED — core-1 RMT ch0 wait" (and the matching user-memory claim) is corrected here to UNPROVEN.** A real
Mode-A RMT hang may still exist as a SEPARATE, rarer mode (the unbounded-wait build genuinely needed a hard
reset), but it is NOT established as the cause of the 2026-06-25 drumbeat, and the drumbeat is not it (§11.1).
The K-escape capture's `rmt_wait_timeout_count` word settles whether the RMT path fires at all on this build.

Companion notes: `.claude/agent-memory/firmware-engineer/project-firmware-lockup-investigation.md` (the
firmware-engineer agent's working notes) and the user memory `project-firmware-streaming-lockup`.

---

## 12. CAPTURE LANDED 2026-06-25 — `[MSG:CRASH usbtx: ...]` decoded → LOST-WAKER (H-A), evidence-based

The K-escape build #1 fired on the FIRST run (`/tmp/pika2.raw`). Wedged EARLY (~+6.7 s, ~line 34/74 — the
non-determinism, cf. prior 401/201/835/1387). The K=3 escape self-reset (CoreSw kept USB enumerated → replayed
on the same `c0`). Full boot dump:
```
[MSG:CRASH core0-comms-wedge stage=axis0:wait_begin comms-stage=line-send-queue comms-froze-first beats comms=138 motion=942]
[MSG:CRASH usbtx: ambiguous free=1 empty=0 mov=1 exec=1 rdepth=8 n=3 rmt_to=0]
[MSG:CRASH comms: rx=rx-read line=line-send-queue con=consumer-enqueue tx=tx-write sta=status-build-report]
```

### 12.1 Settled by the capture (facts)
- **`rmt_to=0` ⇒ the RMT-wait timeout NEVER fired. RMT is POSITIVELY EXCLUDED, by evidence, on this exact
  build.** This upgrades the §11.1 inference (no reset across 8 cycles) to a direct measurement. The co-firing
  `stage=axis0:wait_begin` is the executor's last-stage marker only (§11.9) — NOT a hang; `motion=942` beats
  prove core 1 was scheduling fast.
- **`n=3` ⇒ the `usb_tx` 2 s-timeout K-escape fired ⇒ `USB_TX_TIMEOUT` is the confirmed drumbeat mechanism**
  (not RMT). `rdepth=8` ⇒ the depth-8 `RESPONSE` channel was FULL behind the stalled writer — the head-of-line
  starvation (§11.2) confirmed on-board.
- **`mov=1`, `exec=1`, `motion=942 ≫ comms=138`, `comms-froze-first` ⇒ core-1 executor ALIVE; core-0
  comms/usb_tx froze first. NOT H-B** (core 1 is not wedged).
- **`free=1` ⇒ `serial_in_ep_data_free=1`: the host HAD drained the EP1 IN FIFO (room available) ⇒ NOT
  "host stopped reading"** (not a skirnir/host bug). The byte path is healthy.

### 12.2 The `ambiguous` verdict DECODES to the strongest lost-waker (H-A) signature — a verdict-logic gap
The printed `empty=` field is **`int_raw().serial_in_empty()`** (the raw event), NOT `int_ena` (confirmed:
`comms.rs:1040` prints `stall.serial_in_empty`; the boot line does NOT surface `int_ena_armed` separately —
it is folded into the verdict only). So `empty=0` = the raw TX-empty event is NOT currently asserted.

`verdict()` (`firmware-core/src/diag.rs`, host-tested) returns `Ambiguous` only when it falls through ALL of:
`data_free` true, `motion_advancing` true, and `int_ena_armed || serial_in_empty` FALSE. Therefore the
`ambiguous` verdict + `empty=0` FORCES **`int_ena_armed == false`** as well (else the `int_ena_armed` branch
would have returned `LostTxWake`). So the on-board state at the K-th timeout was:
**`data_free=1`, `int_ena.serial_in_empty=0`, `int_raw.serial_in_empty=0`, core 1 healthy, RESPONSE full.**

This is precisely the **post-ISR lost-waker condition**, and it is the SHARPEST H-A evidence, not a
non-result. Trace (esp-hal-1.1.1 `usb_serial_jtag.rs`):
1. `usb_tx` is parked in `write_async`'s `UsbSerialJtagWriteFuture::new(..).await` (line 820) — the comms
   breadcrumb `tx=tx-write` confirms it; `flush_tx_async` early-returns here because `data_free=1` (line 827).
2. The host drained the packet → the `serial_in_empty` interrupt fired on CORE 0 → the ISR (line 932-961)
   **cleared `int_ena.serial_in_empty` (line 941) AND cleared `int_raw` via `int_clr` (line 948-953), THEN
   called `WAKER_TX.wake()`**. After the ISR runs, BOTH bits read 0 — exactly the captured state.
3. `WriteFuture::poll` (line 736-746) returns `Ready` iff `int_ena.serial_in_empty` is CLEAR. With `int_ena=0`,
   the future WOULD complete **if it were polled** — but it never was. The `WAKER_TX.wake()` did not result in
   `usb_tx` being re-polled, so the future sits parked until the 2 s `with_timeout` aborts it. **The wake was
   lost between the ISR's `WAKER_TX.wake()` and the embassy executor re-polling the task.**
4. Crucially, **the ISR DID run** (it cleared `int_ena`) — so core 0 fielded the interrupt; core 0 is NOT
   starved. This rules out the H-B "core-0 can't service the USB ISR" story and the "ISR-never-serviced"
   (`int_ena`-armed) flavor. What's lost is the embassy/`AtomicWaker` re-poll, a core-0-local async-runtime
   wake race in the `esp-rtos`/`embassy-executor` + esp-hal `WAKER_TX` interaction.

**ROOT CAUSE (high confidence, evidence-based): a LOST USB TX-DONE WAKE (H-A). The esp-hal `serial_in_empty`
ISR fires and wakes `WAKER_TX`, but `usb_tx`'s write future is not re-polled, so the write never completes and
`with_timeout` drops it every 2 s — the drumbeat.**

The `verdict()` `ambiguous` was a CLASSIFICATION GAP, not an unknown mechanism — and it is now CLOSED
(build #1b, 2026-06-25): the fully-serviced-ISR-but-waker-lost case leaves both `int_ena` and `int_raw` at 0,
so it fell through the old `int_ena_armed || serial_in_empty` test. The landed `verdict()`
(`firmware-core/src/diag.rs`, 16 host tests green) now classifies — after the `data_free`/`Core1Wedged`
checks — **`data_free && response_depth > 0` ⇒ `LostTxWake`**, covering all THREE H-A peripheral sub-states
(the raw `iena=`/`empty=` dump fields tell the flavor apart: `iena=1` ⇒ ISR never ran; `empty=1 iena=0` ⇒
waker race; `empty=0 iena=0` ⇒ the captured ISR-ran-but-re-poll-lost case). `Ambiguous` is now reserved ONLY
for `response_depth == 0` (a non-backed-up / non-reproducing snapshot — a real lost-waker stall always has
`rdepth > 0`, so it can never be silently filed as Ambiguous again). The regression test
`verdict_lost_waker_when_isr_fully_serviced_but_parked` (the exact captured state → `LostTxWake`) failed red
against the old logic and passes now (`cargo test -p firmware-core diag` = 16/16, verified). The boot line
gained `iena=`: `[MSG:CRASH usbtx: <verdict> free=.. empty=.. iena=.. mov=.. exec=.. rdepth=.. n=.. rmt_to=..]`
— so the confirming re-run reports `lost-tx-wake` DIRECTLY and surfaces the raw `int_ena` bit (no
back-inference). Predicted re-run line: `[MSG:CRASH usbtx: lost-tx-wake free=1 empty=0 iena=0 mov=1 exec=1
rdepth=8 n=3 rmt_to=0]` (the same state, now self-labeling).

### 12.3 Confidence & the one residual alternative
HIGH confidence it is a lost waker, not H-B/host. But the capture is ONE early-fire data point, and the
`AtomicWaker` re-poll loss is INFERRED from the register state, not directly observed. The single residual
alternative worth a cheap kill: that `usb_tx`'s task was somehow not the registered `WAKER_TX` waker at the
ISR instant (e.g. a reborrow/registration ordering issue in `write_async`'s per-chunk
`UsbSerialJtagWriteFuture::new`, which RE-arms `int_ena` and RE-registers the waker every 64-byte chunk). A
4-byte `ok` is a single chunk, so that's one arm/register per write — but the interaction with esp-rtos's
multi-core executor wake delivery is the unproven link.

### 12.4 Recommended next step (for main / the user)
1. **A confirming RE-RUN of build #1** (cheap, no code change) to verify `free=1 empty=0 mov=1` reproduces — one
   early-fire point shouldn't be the sole basis. If a second run shows the same fingerprint, the lost-waker
   conclusion is solid.
2. **DEFER build #2** (the cross-core `PLANNER`/`MACHINE` lock breadcrumb): its purpose was to localize H-B, and
   the capture says NOT H-B (`mov=1`, ISR serviced). Building it would test a hypothesis the evidence already
   downgraded. Skip unless the re-run flips to `core1-wedged`/a frozen `mov`.
3. **Fix direction (do not implement until the re-run confirms):** the durable fix is in the device→host USB
   write-completion path. Candidates, in order: (a) a firmware-side workaround in `usb_tx` — after arming the
   write, re-check `int_ena.serial_in_empty`/`data_free` before awaiting (a poll-after-arm closes a lost-edge
   window), or drive the write with an explicit `serial_in_ep_data_free`-poll loop rather than relying solely on
   the `WAKER_TX` wake; (b) the esp-hal `UsbSerialJtagWriteFuture` / `WAKER_TX` ↔ esp-rtos multicore wake
   delivery (upstream — the lost re-poll is the suspect). The existing 2 s `with_timeout` is already a correct
   liveness backstop; the §11.6 `COMMS_PROGRESS`-on-completed-write fix should also land so the watchdog stops
   being masked. Also: close the `verdict()` ambiguous→LostTxWake classification gap (§12.2) so a re-run reports
   the verdict directly.

### 12.5 Capture #2 (non-reproduction) + the held combined fix + the confirm-run TRIAD (2026-06-25)
- **Capture #2 did NOT reproduce the wedge** (Pikachu streamed clean). Known non-determinism (cf. the
  synthetic-stress negative, §9), NOT a build defect and NOT a refutation — a single clean run of a
  non-deterministic fault proves nothing either way. Capture #3 does double duty (reproduce + then confirm).
- **[SUPERSEDED 2026-06-25 by §13 — the fix was NOT left reverted; it SHIPPED on main (commit `6024126`, PR #9)
  and the wedge REPRODUCED with it live. See §13.]** The combined fix was first wired, build-verified on both
  Xtensa configs, then briefly reverted to hold the tree at capture-only #1b — but it was subsequently committed
  to main (`6024126`). This bullet's "stays reverted" framing is stale; the current `usb_tx` HAS the full
  recovery path. It is ONE coherent `usb_tx` diff:
  - **(a) poll-after-arm** in `usb_tx` (the in-our-control durable fix for the lost re-poll), classified by a
    pure `WriteOutcome::classify(timed_out, data_free_after)` into `Completed`/`CompletedLostWakeRecovered`/`Stalled`.
  - **(b) the §11.6 watchdog-mask fix** — bump `COMMS_PROGRESS` only on a non-stall (`if !outcome.is_stall()`),
    so a real stall stops feeding the dog instead of the per-loop bump masking it.
  - **a DUAL recovered-count readout** — `USB_TX_LOST_WAKE_RECOVERED` (`AtomicU32`), surfaced live on `$I` as
    `[MSG:USBTX rec=N]` (N>0) AND boot-persisted via an RTC_FAST `RECOVERED_COUNT` word so a partial-fix
    K-escape reset doesn't lose the recovered count (the persistence earns its keep because the fault is
    rare/bursty — §12.6; the team-lead pushed for it and was right).
  - **truncation-safety split (team-lead review, hardening — VERIFIED 2026-06-25):** esp-hal `write_async`
    parks the future BETWEEN 64-byte chunks (`usb_serial_jtag.rs:811-824`), so a `>64 B` response (a full
    status ~90 B) whose lost-wake strands `write_all` after chunk 1 has `data_free=1` yet unwritten bytes —
    recovering THAT would emit a TRUNCATED line. Fix: `usb_tx` times `write_all` and `flush` SEPARATELY and
    recovery fires ONLY at the flush stage (`WriteOutcome::classify_split`: a `write_all` timeout ⇒ `Stalled`
    ⇒ K-escape, clean reset beats truncation; only a flush-stage lost-wake with all bytes in the FIFO is
    recovered). **No impact on the decode/confirm — and I verified WHY:** the captured case (#1) was the
    drumbeat releasing `ok`s (4 B = a SINGLE chunk, ≤64 B, never parks mid-`write_all`) with `data_free=1`
    (host drained) — i.e. a FLUSH-stranded lost-wake, which recovers IDENTICALLY under the split. (`ok`/`error`
    /short ≤64 B never truncate; only a `>64 B` status in the rare² window, which skirnir drops + self-corrects.)
    24 diag host tests green (+4 `classify_split`), verified locally.
  - **write-error arm (final commit, team-lead GO — VERIFIED 2026-06-25):** the write stage is a 3-way
    `WriteOutcome::classify_write_stage(write_timed_out, write_errored)` — write TIMEOUT ⇒ `Stalled`; write
    ERROR (`Ok(Err)`, host-closed port) ⇒ `Completed` (clean drop-and-continue, restoring the pre-split
    leniency that discarded write errors); clean ⇒ `None` (defer to the flush stage). **It does NOT pollute
    `rec=`: `is_recovered_lost_wake()` matches ONLY `CompletedLostWakeRecovered`, and the write-error path
    returns plain `Completed`, so a host-closed write is NEVER recovered-counted** — verified, so the `rec=`
    accumulator stays a clean mechanism-evidence signal for the decode. (For USB-Serial-JTAG a host close
    usually surfaces as a write TIMEOUT not `Ok(Err)`, so this arm is rare in practice — it removes a wart, not
    a hot path.) Does NOT touch recovery or the decode. 27 diag host tests green (+3), verified locally.
    **FINAL build = recovery (poll-after-arm) + watchdog-mask fix + dual `rec=` readout + `classify_split` +
    `classify_write_stage`, K-escape retained.**
  - **the K=3 K-escape breadcrumb is RETAINED by construction** — fed by `outcome.is_stall()`; a recovered
    lost-wake is `is_stall()==false` (resets the escape counter, bumps `rec=`, no escalation), only a genuinely
    wedged write (timeout + FIFO still full) is `is_stall()==true` and counts toward K. So a PARTIAL fix still
    produces a `[MSG:CRASH usbtx: ...]` breadcrumb — we are not blind to it.
- **CONFIRM-RUN TRIAD (all three = fix confirmed):** (1) poll `$I`, watch `rec=` CLIMB past the historical
  wedge zone (~line 400-800+); (2) Pikachu streams to COMPLETION; (3) NO `[MSG:CRASH usbtx:]` on the next boot.
  - **`rec=0`-with-completion is INCONCLUSIVE, not success** — lost wakes may simply not have occurred this run.
    Re-run until `rec>0`. `rec>0` is load-bearing: it proves lost wakes ACTUALLY OCCURRED and were recovered
    (fix exercised + working), vs a lucky clean run.
  - Outcome map: WORKING => `rec>0` and NO usbtx breadcrumb; PARTIAL => `rec>0` AND a usbtx breadcrumb; INERT =>
    `rec=0` (inconclusive, re-run).
- **Gate to apply the fix:** the moment capture #3 reproduces any lost-wake flavor, apply the held combined
  diff, re-verify (20 diag host tests + both Xtensa builds), flash, run the triad. Until then the tree holds at
  capture-only #1b.

### 12.6 Evidence-base caveat — the lost-waker root cause rests on ONE positive capture (be honest about this)
Captures **#2 AND #3 showed ZERO lost-wake events** (clean Pikachu streams, no wedge) — the fault is
rare/bursty. Those are NON-reproductions: they neither contradict nor corroborate the lost-waker mechanism;
they are silent. So the lost-tx-wake root cause currently rests on a **SINGLE positive on-board capture (#1)**,
decoded against esp-hal source and host-tested. Confidence is HIGH on the *mechanism* (the fingerprint
`data_free=1, int_ena=0, int_raw=0, core-1 alive, RESPONSE full, parked at tx-write` has exactly one consistent
explanation — a lost re-poll after a fully-serviced ISR), but the *frequency/reproducibility* is a single
point. Consequence: **`rec>0` on the confirm run does DOUBLE duty** — the fix's recovery path fires only on a
genuine timeout+FIFO-drained, which IS a lost-wake event, so `rec>0` is simultaneously (a) fix confirmation AND
(b) the SECOND independent positive observation of the mechanism. A `rec=0` completion is doubly inconclusive
(no fix proof, no new root-cause point) — and given #2/#3 were zero-event, expect the confirm to need several
runs. Until `rec>0` lands, describe the root cause as **"high confidence, one positive capture,"** NOT
"confirmed." (Process note: the RTC_FAST `RECOVERED_COUNT` persistence — which I initially recommended
SKIPPING — is correct precisely BECAUSE of this rarity: in a partial-fix burst the live `$I` count zeroes on
the K-escape reset, so the boot-persisted mirror preserves the recovered-count evidence across that reset. The
team-lead's push for it was right; my skip recommendation assumed reliable reproduction, which #2/#3 falsified.)

---

## 13. FIX SHIPPED, WEDGE STILL REPRODUCES — Signatures A & B (2026-06-25 night)

**Headline (FACT, firmware-engineer-2 + bughunter-verified): the §12 lost-wake fix is LIVE on main (commit
`6024126`, PR #9 `fix/usb-tx-lost-wake-streaming-lockup`), and streaming `128-Pikachu.tap` STILL wedged. The
fix did not prevent the wedge — only the K-escape backstop caught it.** So §12's recovery is INCOMPLETE, not
done. Two distinct new failure signatures appeared tonight.

### 13.1 Signature A — a DIFFERENT lost-wake sub-flavor the fix structurally cannot catch
`[MSG:CRASH usbtx: <verdict> free=1 empty=0 iena=1 mov=1 exec=0 rdepth=8 n=3 rmt_to=0]` — vs the original
capture #1 `free=1 empty=0 iena=0 mov=1 exec=1 …`. Two fields flipped:
- **`iena 0→1`**: `int_ena.serial_in_empty` is STILL ARMED at the K-th timeout ⇒ the arming ISR NEVER RAN for
  this write (per the diag field doc + esp-hal ISR, which clears `int_ena` when it fires). This is the
  **lost-INTERRUPT flavor** (ISR never ran), distinct from #1's **lost-WAKER-after-ISR** flavor (`iena=0`,
  ISR ran + cleared both bits, only the re-poll lost).
- **`exec 1→0`**: core 1 is IDLE (no block in flight) here; `mov=1` still ⇒ core 1 scheduling, so NOT H-B.

**Why the deployed fix can't catch A (verified two ways):**
1. **Code:** `usb_tx` recovers ONLY at the FLUSH stage — `classify_write_stage(write_timed_out, _)` returns
   `Some(Stalled)` UNCONDITIONALLY on a write timeout (the truncation guard, never consulting `data_free`); only
   a CLEAN write (`None`) proceeds to the flush-stage `classify_split` recovery. A write-stage lost wake counts
   toward K and K-escapes **even with `free=1`**.
2. **Independent corroboration (structural):** esp-hal `flush_tx_async` (`usb_serial_jtag.rs:826-838`) only
   awaits if `serial_in_ep_data_free` is CLEAR — with `data_free=1` it EARLY-RETURNS without parking. So a stall
   captured at `free=1` CANNOT be a flush-stage park; it MUST be at the WRITE stage. This proves the
   write-stage reading independent of the `iena=1` bit.
- **Consequence:** A is a WRITE-stage / ISR-never-armed lost wake; the §12 recovery (flush-stage only) does not
  cover it. And because the *arming* ISR never fired, a flush-stage recovery wouldn't help even if relocated —
  the fix must address the write-stage arm.
- **The minimal fix is SMALLER than a write-path rewrite — it does NOT re-poll esp-hal's future (bughunter,
  verified against `comms.rs:1452-1474` + `write_async` 811-824).** The deployed "recovery" never re-polls the
  esp-hal future: `with_timeout` DROPS it, `usb_tx` loops, the current `resp` is ABANDONED, and the next
  `RESPONSE` item proceeds. "Recovery" = **drop-the-response-and-continue**, which is CORRECT iff the bytes were
  fully delivered. `write_async` pushes ALL bytes of a chunk + sets `wr_done` BEFORE it parks, so a SINGLE-CHUNK
  (≤64 B) response stranded at a write-stage timeout with `data_free=1` was FULLY DELIVERED (a 4 B `ok` =
  Signature A). The un-re-pollable future (`iena=1`) is IRRELEVANT — we're confirming the bytes left, not
  completing the future. **So the fix is: at a write-stage timeout, recover (drop+continue, count `rec=`) IFF
  `resp.len() ≤ 64` AND `data_free=1`; else `Stalled`.** ~5 lines (extend `classify_write_stage` to take
  `resp_len` + `data_free`); catches Signature A; keeps the `>64 B` truncation guard fully intact (a multi-chunk
  write-stage timeout can have unwritten later chunks → still `Stalled`). firmware-engineer-2's `data_free`-poll
  write-path REWRITE (bypass the `WriteFuture`, poll `data_free` directly — immune to the lost edge for all
  sizes/flavors) is the more robust long-term FALLBACK, but bigger; do the single-chunk widening first.

### 13.2 Next experiment (agreed) — instrument the STAGE, don't infer it
Before any recovery-widening: add a `stall_stage` field (write/flush/none, from which `with_timeout` fired) to
`UsbTxStall`, packed into the breadcrumb word, surfaced as `stg=` on the boot line. A re-capture then says
DIRECTLY `stg=write + iena=1` (confirms A's write-stage reading) vs `stg=flush` (the targeted case — and if
that didn't recover, a different bug). Converts inference→measurement. THEN re-capture to confirm
`iena=1/exec=0/stg=write` is STABLE (one field-set must not drive a fix), and watch whether #1's
`iena=0/exec=1/stg=flush` flavor ALSO reappears (⇒ the fix is partial across two flavors, not simply wrong).
Only after `stg=write` is proven: widen recovery to the write stage for the ≤64 B single-chunk case only.
Tracked as task #18.

### 13.3 Signature B — hard silent lock, NO breadcrumb (Mode C; SEPARATE problem, do not conflate with A)
No `[MSG:CRASH]` at all, no recovery, skirnir failed to reconnect 6×. This is §2/§6 Mode C. The custom
`#[panic_handler]` (§6/§7) should turn any panic into `[MSG:CRASH panic …]` + `software_reset`; B produced
NOTHING. Candidates: (a) not a panic (true HW hang / brownout / USB-stack death); (b) the panic handler itself
re-wedged before its store landed; (c) it DID reset but the post-reset USB never re-enumerated → skirnir saw
silence. **§10's bench finding (post-Pikachu-wedge skirnir reopen got SILENCE until a hard espflash chip reset)
makes (c) the LEADING suspect** — "silent to skirnir" ≠ "no reset." Proposed instrumentation (AFTER A's stage
field lands — one capture build at a time): a boot-count word in RTC_FAST + `SocResetReason` on next boot, to
detect a silent reset loop vs a true hang. NOT yet built.

### 13.4 Signature B — the WATCHDOG DEAD ZONE (bughunter, structural root of the no-reset; tracked task #19)
§13.3 leads with "(c) it reset but USB never re-enumerated." There is a STRONGER, more falsifiable candidate that
explains B getting NO reset AT ALL — a structural gap in the watchdog itself, found by reading `watchdog_feed`
(comms.rs:2990) + the RWDT setup (`main.rs:435`):

**The RWDT is Stage0=`ResetSystem`, 8 s, ONLY — fed by a SOFTWARE task on core-0's thread-mode executor. There is
NO hardware-independent second-stage reset and NO SuperWDT.** The two withholds that would force a reset are each
GATED: the core-0 comms withhold needs `host_active` (`RX_ACTIVITY` advanced within `RX_ACTIVE_TICKS`=12=6 s); the
core-1 motion withhold needs `EXECUTOR_RUNNING` (a block in flight). **DEAD ZONE: host quiet (RX aged out →
`host_active=false`) AND executor idle (`exec=0` — exactly Signature A's `exec=0`, queue drained) ⇒ NEITHER withhold
can fire ⇒ the dog is fed forever ⇒ permanent silent lock, no reset, no banner.** That reproduces B without any
"reset-then-silent-USB" step. **RE-VERIFIED 2026-06-25 line-by-line against the `watchdog_feed` body: the ONLY
non-feeding path is `if core1_wedged || comms_wedged { withhold }`; every other tick hits the unconditional
`rtc.rwdt.feed()`. So with both withhold gates false, the dog is unconditionally fed — the dead zone is real, not
hypothetical.** The structural fix (independent of which B-hypothesis wins): an UNCONDITIONAL absolute-deadline
backstop — if `usb_tx` has produced NO completed write for > ~N s while `rdepth > 0` (responses queued but nothing
leaving), withhold regardless of `host_active`/`exec`. A board that has stopped emitting ANY response while work is
queued must reset, period.

**IMPLEMENTATION (lead-approved 2026-06-25, ships in the §13.5 capture build — a real behavior change, justified:
converting a permanent silent lock into an auto-reset-that-records-`reset_reason` is strictly better than the
EN-button-only status quo, and it is the ONLY way Signature B leaves a trace):** add a THIRD, UNCONDITIONAL
withhold term — `if core1_wedged || comms_wedged || absolute_deadline_exceeded { withhold }`. `absolute_deadline_
exceeded` = ticks-since-`COMMS_PROGRESS`-last-advanced ≥ N WHILE `rdepth > 0`, reset by any completed/recovered
write (the non-stall outcome `usb_tx` already computes). N ≈ 10-15 s. **CRITICAL: it must NOT be gated on
`host_active` or `EXECUTOR_RUNNING` — gating on either reproduces the very dead zone it closes (host-quiet +
`exec=0` makes both existing gates false).** Keep the two existing gated withholds UNCHANGED (they catch their
cases faster); the new term is purely additive. Host-testable: counter advances when `COMMS_PROGRESS` frozen AND
`rdepth>0`, resets on a completed write, fires at N. NOTE: when this backstop fires it produces a `CoreRtcWdt`
reset — it does NOT blind the diagnosis: the §13.7 boot line still reports `reason=` + the heartbeat (climbed ⇒
B-1 dog-fooled / froze ⇒ B-2) + boot-count, which is exactly the B-1/B-2 discrimination.

**LANDED + ARMING DECISION (2026-06-26) — the backstop is already BUILT (gated), and PROVEN safe to arm during the
Signature-A capture.** The mechanism is implemented and host-tested, gated behind `DEAD_ZONE_BACKSTOP_ARMED: bool`
(comms.rs): a usb_tx-SPECIFIC `USB_TX_COMPLETED` beat (bumped on a completed/recovered write, comms.rs:1535 — NOT
`COMMS_PROGRESS`, which the status reporter + a recovered-lost-wake can keep alive), `tx_complete_frozen_ticks`
tracking UNGATED by host_active/exec (comms.rs:3114), and `diag::dead_zone_withhold(RESPONSE.len(),
tx_complete_frozen_ticks) = response_depth>0 && ticks >= DEAD_ZONE_STALL_TICKS(16)` = **8 s**. A `DeadZone`
withhold reason + breadcrumb are wired.
**PROOF the backstop CANNOT preempt/contaminate the Signature-A wstg capture (the team-lead's gating question,
answered from the code):** the backstop needs **8 s** of no completed usb_tx write (`DEAD_ZONE_STALL_TICKS=16 ×
500 ms`); a Signature-A wedge K-escapes at `K=3` consecutive `USB_TX_TIMEOUT=2 s` stalls = **~6 s** and
`software_reset()`s FIRST. **6 s < 8 s ⇒ the K-escape always wins the race on an A-wedge; the backstop's 8 s
deadline is never reached.** The backstop only reaches 8 s in the true DEAD ZONE (host-quiet + nothing completing,
where the K-escape's *consecutive*-stall counter is reset by intermittent recovered writes — §13.8 — and the
comms-stall detector is host-inactive-blind) = exactly Signature B / a no-reset hang. So arming is PURE UPSIDE for
the overnight loop: A still captured cleanly via the faster K-escape, AND a B event becomes a `CoreRtcWdt` reset +
`[MSG:CRASH … DeadZone]` breadcrumb instead of a hard-lock that kills the rest of the unattended night (B fired
once tonight at ~14 min on T1, so "B ends the night" was a real, not hypothetical, cost). **DECISION: arm it
(`DEAD_ZONE_BACKSTOP_ARMED = true`) — a one-line flip + reflash; sequence by the in-flight run's depth.** (Also
noted: the §13.7 `wdog=` heartbeat is ALREADY wired live — `bump_watchdog_heartbeat()` at comms.rs:3078, `wdog=`
in the boot line — NOT inert; reconciling with fwengineer-2 whether the flashed image carries it.)

**B hypothesis tree (discriminated by the §13.5 capture):**
- **B-1 (LEADING — watchdog-mask REDUX):** `COMMS_PROGRESS` is not fully frozen. A *recovered lost-wake* is
  `is_stall()==false` and STILL bumps `COMMS_PROGRESS` (comms.rs:1484); intermittent recovered-wakes interleaved
  with genuine stalls limp the counter so `comms_frozen_ticks` never reaches 6 → dog stays fed. SAME defect class as
  §11.3, RE-INTRODUCED by the `6024126` fix. (This is a genuinely NEW way to be fooled that did not exist before the
  fix — the fix that recovers A's drumbeat is the mechanism that masks B's watchdog.)
- **B-2 (executor death):** the wedge killed core-0's executor incl. `watchdog_feed` → dog never fed → a reset
  SHOULD have fired; the no-banner then needs §13.3's (c) re-enum-failure to explain the silence.
- **B-3 (panic + reset, host couldn't reconnect):** a reset emits a banner; 6 silent reconnects argue against.
- **B-4 (brownout/power glitch):** wipes RTC, no breadcrumb, possibly no clean re-enum.

### 13.5 The ONE capture build that advances BOTH signatures (bughunter direction → fwengineer)
Fold §13.2's stage field together with the B instrumentation into a SINGLE capture-only build so one overnight run
informs both:
1. **Record write-vs-flush STAGE** at the K-escape (§13.2) — `stg=write` confirms A.
2. **ALWAYS emit `reset_reason`** in the boot line (already imported `main.rs:57`) — splits B-4 (Brownout) from the
   rest on the very next wedge, even with NO breadcrumb. Directly tests §13.3's (c): a `CpuSw`/`CpuRtcWdt` reason ⇒
   a reset DID happen (favors (c)/B-2); a `ChipPowerOn`/`Brownout` ⇒ no software reset (favors the dead zone/B-4).
3. **Free-running RTC_FAST `watchdog_feed` heartbeat** — climbed THROUGH the wedge ⇒ `watchdog_feed` alive but
   FOOLED (B-1); froze ⇒ executor death (B-2). This is the B-1/B-2 discriminator.
4. **Dead-zone backstop withhold:** `RESPONSE` depth>0 AND no COMPLETED `usb_tx` write for >~6 s ⇒ withhold the feed
   REGARDLESS of `host_active`/`exec`. Converts B's silent lock into a breadcrumb-bearing reset so B becomes readable
   at all (also a partial fix; framed instrumentation-grade — a board sitting on a non-empty RESPONSE backlog
   emitting nothing for 6 s is wedged by definition).

Run `SKIRNIR_RAW_LOG=1 skirnir --cli <port> 128-Pikachu.tap …` (A ~30 min) and `… T1_Test.tap …` (B ~14 min); on a
wedge wait ≥12 s; do NOT press EN/espflash (wipes RTC_FAST).

### 13.6 What remains UNKNOWN
- B-1 vs B-2 vs B-4 — undetermined until the heartbeat + `reset_reason` lands. B-1 is leading by reasoning only.
- Whether §13.3's "(c) reset-then-silent-USB" or §13.4's "no reset at all (dead zone)" is what actually happened —
  the `reset_reason` capture settles it in one boot.
- The `error:1` after A's recovery — a stray/corrupt byte surviving the CoreSw reset (a partial line left in RX
  across the K-escape). Real, lower priority than B, not yet traced.
- Whether A and B share one root (both downstream of the lost USB TX-done event) or B is an independent hard fault.

### 13.7 Signature B — CONCRETE instrumentation spec (bughunter design → fwengineer; rides the §13.5 build)
Designed against the actual `crash.rs` breadcrumb infra + `main.rs` boot sequence (read in full). Three additive
words + an ALWAYS-ON boot line. All diagnostic, no behavior change. Drop into the §13.5 combined capture build.

**KEY GAP this closes (found by reading the emit path):** `maybe_emit_crash_report` (comms.rs:887) returns
early on `!is_valid() || !reset_was_watchdog`, so on a NO-breadcrumb boot — EXACTLY Signature B's silent-reset
case — NOTHING goes over the grbl CDC. `reset_reason` IS read at boot (`log_reset_reason`, main.rs:391) but
only to `println!`/defmt, which the host can't see during normal grbl streaming. So today a silent reset is
invisible over CDC. Fix: emit the reset reason + boot count UNCONDITIONALLY over the grbl TX, right after the
(already-unconditional) `send_banner()` (main.rs:680), BEFORE the breadcrumb gate.

**(1) Always-on boot line over the grbl CDC.** New `comms::emit_boot_status(pro_reason, boot_count, last_feed_age)`
called unconditionally after `send_banner()` (and stashed for one `$I`/`?` replay like the crash report):
`[MSG:BOOT reason=<label> n=<boot_count> feedage=<ticks>]`. `reason` reuses the existing `reset_reason_label`
map (main.rs:345) — no new decode. This line appears on EVERY boot, breadcrumb or not, so a silent reset that
re-enumerated even briefly is caught.

**(2) RTC_FAST boot-count word (the silent-reset-loop detector).** New `idx::BOOT_COUNT` slot (append after
`RECOVERED_COUNT`; `RING_BASE` auto-shifts since it's `PANIC_BUILD_ID + N`). Incremented ONCE per boot in
`take_breadcrumb` (or a dedicated `bump_boot_count()` called in `main` right after `take_breadcrumb`), saturating.
Decode: the boot line's `n=` is this value. **Reading it across the espflash-wipe trap (the §10 gotcha):**
- RTC_FAST survives CoreSw/RWDT resets but is WIPED by power-on/brownout AND by an espflash DTR/RTS reset (§10).
  So `BOOT_COUNT` counts ONLY consecutive software/watchdog reboots — which is EXACTLY a silent reset loop, and
  it is correctly ZEROED by the power-cycle/brownout that would otherwise confound it. The trap works FOR us here.
- A clean first boot (cold) reads `n=1` (the cold-boot zero-init + this boot's increment). A silent reset LOOP
  shows `n=2,3,4…` climbing on each successive `[MSG:BOOT …]` IF the board re-enumerates each loop; if it never
  re-enumerates, you see nothing live — but the moment you force a hard reset to look, RTC_FAST is wiped and
  `n` reads 1 again. THEREFORE: the boot count is only meaningful if read on a boot the board ITSELF reached over
  USB (a re-enumerating loop). For a loop that never re-enumerates, the boot count alone can't prove it — which
  is why we ALSO need (3) + the reset-reason in (1) to distinguish the two.

**(3) Free-running RTC_FAST `watchdog_feed` heartbeat (the B-1-vs-B-2 + reset-vs-hang discriminator).** New
`idx::WDT_HEARTBEAT` slot, incremented every tick by `watchdog_feed` (one relaxed store, off the hot path). It
is NOT consumed/cleared by `take_breadcrumb` — it free-runs across resets (only power-cycle/brownout zeroes it).
The boot line's `feedage=` reports `heartbeat - heartbeat_at_last_boot` (store the prior value in another word, or
just report the raw heartbeat and diff across two boot lines). Reads:
- **heartbeat CLIMBED across the wedge** (the boot after a reset shows it advanced well past the prior boot's
  value) ⇒ `watchdog_feed` was ALIVE and feeding through the wedge ⇒ the dog was FOOLED = **B-1 (dead zone /
  watchdog-mask redux)**. The recovered-lost-wake `COMMS_PROGRESS` bumps kept `comms_frozen_ticks < 6`.
- **heartbeat FROZE** (barely advanced before the reset) ⇒ `watchdog_feed` itself stopped ⇒ **B-2 (executor
  death)** — and then the reset that DID happen was the RWDT finally firing because the feed stopped.

**The decisive 2×2 (what the combined build resolves in ONE wedge):**
| `reason=` (boot line) | heartbeat | ⇒ verdict |
|---|---|---|
| `core-sw-reset`/`*-rtc-WDT` + `n` climbing | climbed through wedge | **B-1 dead zone** — dog fooled, fix = unconditional absolute-deadline withhold (§13.4) |
| `*-rtc-WDT` | froze | **B-2** — executor/`watchdog_feed` died, dog fired on the stopped feed |
| `power-on`/`brown-out` (n resets to 1) | n/a (RTC wiped) | **B-4** — power glitch / brownout; not a firmware logic wedge |
| board NEVER emits `[MSG:BOOT]` at all | unreadable | **true hang OR a reset that never re-enumerates USB** — distinguish by §13.3(c): does an EXTERNAL `espflash` see a `reset_reason` of CpuSw/RtcWdt (reset happened, USB re-enum failed) vs the board truly frozen (a JTAG halt would show the PC spinning). This is the ONE case the in-band readout cannot reach; it needs the external probe. |

**"Reset that never re-enumerated USB" vs "true total hang, no reset" — the distinction the team-lead flagged:**
- If the board emits `[MSG:BOOT reason=core-sw-reset n=…]` at all (even once, even late) → a reset DID happen and
  USB came back at least once → it is the re-enumeration-flakiness / silent-reset-loop class (fix locus: the USB
  re-enum path / a reset cause we must stop, NOT a missing watchdog).
- If the board emits NOTHING over CDC and an external espflash sees a `reset_reason` other than `power-on` →
  reset happened, USB never re-enumerated (the §13.3(c) leading suspect).
- If an external espflash/JTAG shows the board is STILL RUNNING (PC advancing) with no reset → a TRUE hang the
  watchdog failed to catch (the dead zone, B-1) — and the fix is the §13.4 absolute-deadline withhold.
- These three change the fix entirely (USB-reenum fix vs reset-cause fix vs watchdog-coverage fix), which is why
  the boot line + heartbeat are worth landing before touching any fix.

**CORRECTION (2026-06-26, verified — supersedes the optimistic "external espflash/JTAG saves the no-reset case"
above): the external readout is NOT a reliable safety net for a TRUE no-reset silent lock on this board.** Facts:
(1) the board uses the S3's BUILT-IN USB-Serial-JTAG (`USB_DEVICE`, internal PHY GPIO19/20) — same peripheral for
CDC and JTAG; NO external JTAG probe is wired and GPIO39 (the pad option) is taken by A-LIMIT, so there is no
independent debug channel. (2) `espflash`'s default attach toggles DTR/RTS → reason `0x15` → WIPES RTC_FAST
(§10/ESP-IDF #8889), destroying the very heartbeat/boot-count/reset_reason words. (3) `espflash monitor --before
no-reset` avoids the wipe BUT is a serial MONITOR, not a debugger — on a truly silent hang the firmware emits
nothing, so it reads silence; it cannot read RTC memory or the `reset_reason` register (that needs JTAG
memory-read, which needs the built-in USB-JTAG that is likely down WITH the USB-CDC on the wedge). **So the table
row "board NEVER emits `[MSG:BOOT]` → needs the external probe" is only resolvable when B RESET-but-USB-didn't-
re-enumerate (the `reset_reason` register survives until the next toggle and `--before no-reset` can read it
before re-toggling); a TRUE no-reset hang is NOT externally readable here.** CONSEQUENCE: the §13.4 dead-zone
BACKSTOP is the MUST-HAVE that makes a no-reset B leave a trace — it converts the no-reset hang into a CoreSw/RWDT
reset that PRESERVES RTC_FAST and emits `[MSG:BOOT reason=… n=… feedage=…]` in-band. Under a passive-only build,
if B fires as a true no-reset lock, that run is a wasted cycle for B (accepted ONCE for the ready wstg/A capture
since B is rare; the backstop is next-build priority to close the hole).

**Layout summary (append to `idx`, after `RECOVERED_COUNT=PANIC_BUILD_ID+3`):** `BOOT_COUNT = +4`,
`WDT_HEARTBEAT = +5`, `LAST_BOOT_HEARTBEAT = +6` (for the `feedage` diff), `RING_BASE = +7`. Add `bump_boot_count()`,
`bump_wdt_heartbeat()`, and surface `boot_count`/`heartbeat` on the `Breadcrumb` (NOT consumed by `take_breadcrumb`
— they free-run). Cost: 3 relaxed stores total per relevant site; all off the real-time path. Stays behind the
existing gated/`#[ram(rtc_fast, persistent)]` scheme.

### 13.8 PRE-REGISTERED THIRD MODE — "Mode C / ALTERNATING" (bughunter-2; mechanism VERIFIED, agreed pre-capture)
Registered BEFORE the wstg capture so an empty result is interpreted, not rationalized after. **HYPOTHESIS:** a
third failure mode beyond A (write-stage lost wake → K-escapes) and B (silent dead-zone lock) — an INTERLEAVED
pattern of flush-recovered writes and write-genuine-stalls that evades BOTH backstops, so the board limps (or
hard-locks) with NEITHER a `[MSG:CRASH usbtx:]` breadcrumb NOR a self-reset.
**Mechanism — VERIFIED against the code (bughunter):**
- A recovered lost-wake is `outcome.is_stall() == false`, and `UsbTxStallCounter::record(false)` RESETS
  `consecutive_timeouts` to 0 (diag.rs). So a recovered write between genuine stalls keeps the K-escape from ever
  reaching 3 CONSECUTIVE stalls → the usbtx K-escape never fires.
- The same recovered lost-wake (`!is_stall()`) ALSO bumps `COMMS_PROGRESS` (comms.rs), which resets the
  watchdog's `comms_frozen_ticks` → the (future §13.4) dead-zone backstop's freeze counter never reaches its
  threshold either.
- So an alternating recovered/genuine-stall sequence resets BOTH counters and evades BOTH backstops — a real,
  code-grounded escape path, not just plausible. (Both KNOWN captures #1 and Signature-A are PURE-flavor, so this
  is hypothetical until observed.)
**FALSIFIABLE TELL (pre-registered):** a wedge that produces NEITHER a usbtx breadcrumb NOR a self-reset — the
board just limps at a degraded ack cadence or hard-locks silently — is the LIVE SUSPECT for this alternating mode,
NOT "the capture failed." An EMPTY capture is itself a signal. (Distinguish from a true no-reset Signature-B by the
raw ack dynamics: B = total freeze then nothing; Mode C = an IRREGULAR limp — bursts of recovered acks interleaved
with ~2 s stall gaps, never 3-in-a-row, COMMS_PROGRESS crawling.) If observed, the fix must make the K-escape
count NON-consecutive (e.g. a leaky-bucket / rate of stalls over a window, not strictly consecutive) AND/OR the
dead-zone backstop key on an absolute "no NET forward progress past the queue" deadline rather than a frozen
counter that a single recovered write resets.

---

## 14. SILENT GCODE SKIP — user-reported part corruption (2026-06-26; task #22)

**This may outrank the lockup.** User report: recent runs no longer hard-lock but SKIP MULTIPLE chunks of gcode
PER RUN with the job CONTINUING past each gap (different areas each run, mostly-complete parts). A VISIBLE
hard-lock became a SILENT corruption — strictly WORSE for CNC (a hard-lock = obviously-incomplete part you
scrap; a silent skip = a part that LOOKS finished but is missing toolpaths).

### 14.0 RESETS ARE EXCLUDED as the cause of the user's symptom (team-lead reframe, bughunter-verified)
My first §14 draft (below, §14.1) said the auto-reset "drops the chunk" — TRUE, but it is NOT the user's
symptom, and the discriminator is decisive: **a mid-stream banner makes skirnir ABORT/TRUNCATE the program, NOT
skip-and-continue.** Verified `crates/skirnir/src/protocol/core.rs:390-402`: `Banner ⇒ clear_program() +
reset_window(true) + AbortQueued + transition(Idle)` — a SINGLE truncation; the job STOPS at the first banner.
It CANNOT produce MULTIPLE internal gaps + continuation. The user sees multiple gaps with continuation ⇒ **the
real skip is a NON-RESET silent line-drop**; the K-escape/backstop reset is NOT it (a reset truncates, ending
the job at the first occurrence). Two families remain:
- **(i) a separate NON-RESET line-drop bug** (over-ack, RX-pipe byte loss, drop-and-continue, planner/motion
  block drop) — independent of the recovery, possibly latent before it.
- **(ii) skirnir MISSES the banner in the USB-reset chaos** → does NOT cleanly abort → desyncs char-counting and
  streams on, dropping a window and continuing (could REPEAT → multiple gaps). Reset-linked, via a missed
  banner, not the clean abort.
**LEADING firmware lead (bughunter, code, to test): the `RX_PIPE.try_write` overflow drop** (`comms.rs:1221`):
on pipe overflow a byte is SILENTLY dropped. A dropped byte mid-line does NOT cleanly "error the line" — it
MERGES two lines (`G1X10\nG1Y20` → `G1X10G1Y20`) or mangles a coordinate, AND desyncs the host char count (host
counted 2 lines, firmware emits 1 response). That is a skip-and-CONTINUE that can REPEAT → multiple gaps —
fits the symptom. The comment says the pipe "never overflows for a compliant host," BUT a lost-wake stall
(§12/§13) makes the host keep sending while un-acked → overflow → dropped bytes → merged/corrupt lines → skip.
**So the lost-wake wedge and the skip may be LINKED: stall → RX_PIPE overflow → silent line corruption.**
Testable via the §14.5 logged air-run (line-IN vs ACK vs EXEC + whether skirnir aborts vs misses-banner, gaps
1 vs N).

### 14.1 The reset-drop mechanism (REAL, but a SEPARATE production concern — NOT the gap cause, see §14.0)
A mid-stream `software_reset()` recovery (the §12 K-escape; the §13.4 backstop if armed) DOES drop the in-flight
gcode and truncates via the host abort. Real defect for production (→ §14.3 fail-safe), but it produces
TRUNCATION, not the user's internal gaps. Mechanism, two legs:

### 14.1 Mechanism — two independent legs, both proven
1. **Firmware side:** a mid-stream `software_reset()` (CoreSw) re-emits the boot banner, loses the bytes the host
   streamed into the rebooting USB-Serial-JTAG FIFO during the ~6 s reboot, and resets modal state.
2. **Host side — VERIFIED in the skirnir source** (`crates/skirnir/src/protocol/core.rs`): `:17` "a banner
   mid-stream (controller reset) … ABORTS THE PROGRAM"; `:391` "A banner means the controller reset: abort any
   stream, clear the window, return to Idle"; the `AbortQueued` effect (`:54`) DISCARDS queued/in-flight lines —
   they "must NOT reach the wire after the abort." **skirnir does NOT re-send the in-flight chunk; it aborts.**
   So firmware reset → banner → host aborts the program. The dropped chunk is gone, silently.

### 14.2 What this reframes
- **The SHIPPED §12 recovery (K-escape, on main `6024126`) is already a chunk-eater** — it silently corrupts the
  part on ANY Signature-A wedge that fires during a real cut. The lost-wake "fix" cured the drumbeat but
  introduced silent part corruption.
- **The §13.4 dead-zone backstop is a chunk-eater too** — it resets mid-stream.
- **CORRECTION of my earlier ruling (§13.4 "arm the backstop = pure upside", 2026-06-26):** that ruling was
  WRONG — I weighed "the overnight loop survives a B event" but did NOT weigh "a silent reset corrupts the
  part." A mid-cut backstop reset is unacceptable. **Do NOT arm the backstop for production.** (fwengineer-2
  correctly withheld arming on this basis; the board is held with the backstop OUT.)

### 14.3 Correct recovery direction (grbl's own contract)
Any mid-cut recovery MUST be **feed-hold → ALARM → require re-home**, NEVER a silent `software_reset()` the host
streams through. After ANY reset the machine has lost position certainty (open-loop steppers), so a silent
resume would cut in the WRONG place even if the gcode weren't dropped. grbl's rule for a step-sync-breaking
fault is exactly this — raise `ALARM:N` (skirnir surfaces it and HOLDS the stream, loud + visible), force the
operator to re-home/re-zero, restore certainty. The operator KNOWS the cut is compromised instead of
unknowingly running a corrupt part.

### 14.4 The diagnostic-vs-production tension (resolve before the recovery redesign)
The breadcrumb capture (§11–§13) DEPENDS on `software_reset()` (RTC_FAST survives CoreSw; it's how `[MSG:CRASH
…]` is read on the next boot). A feed-hold+ALARM recovery does NOT reset → no breadcrumb → loses the A/B capture
channel. **Proposed split:** keep `software_reset()`+breadcrumb in the INSTRUMENTED capture builds (operator-
gated: "diagnostic run, scrap the part"); ship feed-hold+ALARM+require-rehome in the PRODUCTION recovery. So the
capture work (#20/#21) continues on the diagnostic build while the user-facing corruption is fixed by the ALARM
redesign (task #22). **Team-lead owns the design call; this §14 is the authoritative finding it rests on.**

### 14.5 HOST EXONERATED as over-send initiator (skirnireng audit) → the skip is FIRMWARE-side, two paths
skirnireng (host-accounting owner) did a read-only audit of `flow.rs`/`core.rs` and CODE-VERIFIED that **skirnir
cannot initiate an over-send**:
- **No phantom-ack / over-credit.** The window frees ONLY via `flow.on_ack()` from a real `Ok`/`Error`; an ack
  with an empty window is a HARD FAULT (`UnexpectedAck`). The §8.4 budgets only SUPPRESS that fault, never free
  send space: `trailing_acks` (banner-path, count-bounded) absorbs orphaned acks; `ignore_stray_acks` (a latch,
  cleared on the next real line) only governs whether an unmatched ack faults. Neither touches `inflight_bytes`
  or program release. Both re-fault a genuine over-ack once cleared (tested).
- **Window size matches EXACTLY:** firmware `RX_PIPE_CAPACITY = RX_BUFFER_SIZE = 1024` (`comms.rs:101`,
  `protocol.rs:60`) == host `DEFAULT_RX_BUFFER = 1024` (`flow.rs:18`) == the advertised `[OPT:]` field. No
  over-send-by-construction.
- **No early release:** the host holds a line in-flight from write until its `ok`; release gated on
  `inflight_bytes + next_len ≤ 1024`. Because the firmware acks AFTER draining the pipe byte, a
  buffered-but-not-acked line is still fully counted, so no fast-ack early release. `Bf:` is NOT wired to the
  char-count window (only the jog 32-block throttle).

**CONCLUSION: the RX_PIPE overflow (and the skip) can ONLY be FIRMWARE-side. Two paths:**
- **(a) STALL→OVERFLOW:** the firmware stops acking (the §12 lost-wake stall) while a compliant host keeps the
  1024 window full → pipe fills → `RX_PIPE.try_write` silently drops a byte (`comms.rs:1221`) → a merged/corrupt
  line → skip-and-continue, repeats → multi-gap. Fits the symptom (non-deterministic, continues). LEADING.
- **(b) FIRMWARE OVER-ACK:** the firmware emits a SPURIOUS/DUPLICATE `ok` → the host FAITHFULLY releases one line
  early → over-send by one line → desync. Firmware-sourced; the host acts correctly on a bad ack. (skirnireng's
  probe.) On the wire (a) and (b) look identical; the discriminator is on-device counters.

### 14.6 The air-run instrumentation (OBSERVE-ONLY) — the 6 probes that discriminate every candidate
fwengineer-2 builds, bughunter directs+decodes. OBSERVE-ONLY — count, do NOT convert any drop/over-ack to an
error yet (converting would HOLD the stream and MASK the skip we're trying to see — team-lead guardrail). Probes,
all `$I`-readable:
1. **`RX_PIPE_OVERFLOW`** — count the `try_write` Err (dropped byte) at `comms.rs:1221`. `>0` ⇒ path (a) firing.
2. **LINE-IN** — lines framed by `line_assembler` → `LINE_QUEUE`.
3. **ACK** — `ok`/`error` emitted by `usb_tx`.
4. **EXEC** — blocks actually executed by the motion executor.
5. The existing `wstg`/usbtx breadcrumb (orthogonal; still gives Signature A if it wedges).
6. **`oks_emitted` vs `lines_consumed`** — `oks_emitted > lines_consumed` ⇒ path (b) firmware over-ack.
**Discriminator table:** overflow>0 (+ oks==lines) ⇒ (a) stall→overflow; oks_emitted>lines_consumed ⇒ (b)
over-ack; LINE-IN==ACK but ACK>EXEC ⇒ a motion/planner BLOCK drop (acked line, block never ran); all equal +
overflow=0 ⇒ none of these, look elsewhere. Each candidate has a UNIQUE counter signature — that's the design.
**RUN gating:** operator-gated air-cut (no material — part is scrap; user confirms first). Backstop stays OUT
(a reset would mask the observation); the K-escape reset stays (gives the wstg breadcrumb if A wedges). NO fix on
source-proof alone until the air-run pins which signature fires.

### 14.7 §14 OVERFLOW HYPOTHESIS REFUTED (skirnireng proof) — the skip is NOT a flow-control/overflow bug
skirnireng proved (read-only, file:line) that **skirnir cannot over-send AT ALL**: a dropped `ok` makes the
host UNDER-send (the un-acked window stops releasing lines) and eventually DISCONNECT — never over-send. Both
host timeouts disconnect, never ack. So the RX_PIPE-overflow PRECONDITION (host over-send) is **proven
impossible**, and the §14.0/§14.5 "(a) stall→overflow→skip" path is **REFUTED at step 1**. Path "(b) firmware
over-ack" survives only as a NULL-check (the firmware emitting a spurious `ok` would be a real bug, but it's a
separate symptom, not shown). **The skip is NOT a flow-control / inbound-drop bug.** The RX_PIPE-overflow
counter stays in the air-run ONLY as a null-confirm (it should read 0). The real locus is §15.

---

## 15. LEADING ROOT CAUSE (verified): SILENT MID-BLOCK RMT TRUNCATION drops the tail of a cutting move

**Found by the firmware-engineer deep audit, bughunter-VERIFIED in source 2026-06-26.** This fits the user's
silent-skip symptom better than anything prior and is code-confirmed end to end.

### 15.1 The mechanism (file:line, verified)
1. A cutting block with **> `MAX_SYMBOLS_PER_BURST` (=46) step events** is emitted as MULTIPLE RMT bursts —
   `ceil(steps/46)` of them (`cnc-kinematics/src/motion.rs:201,231-241,930`). Essentially every real cutting
   segment longer than 46 steps is multi-burst.
2. Each burst is `sink.emit_burst(&burst)?` (`cnc-kinematics/src/motion.rs:232`). **The `?` ABANDONS all
   REMAINING bursts of the block on ANY `Err`** — the loop never continues, the trailing `:240` burst never runs.
3. `RmtStepSink::emit_burst` returns `Err(StepError::Transport)` **NON-FATALLY** from the RMT
   `wait()`-completion-error arm (`firmware/src/motion.rs:408-413`: it RESTORES the channel
   `self.channels[axis]=Some(channel)` then sets `result=Err(Transport)`). Channel survives ⇒ a RECURRING,
   non-deterministic error (depends on RMT hardware asserting an error status on a `wait()`), NOT a one-shot.
4. That error is **DISCARDED** at `run_block`: `let _ = generator.run_block_scaled(...)`
   (`firmware/src/motion.rs:786`) — no counter, no breadcrumb, no defmt, no ALARM.
5. The executor proceeds to the NEXT block. Result: **the TAIL of the cutting move (every burst after the failing
   one) is silently skipped, and the job continues.**

### 15.2 Why it fits EVERY symptom (and why it stayed invisible)
- **Non-deterministic** (RMT hardware wait-error timing → different blocks each run). ✓
- **Continues past the gap** (executor moves to the next block by design — the discard at :786 is explicit). ✓
- **Multiple gaps per run** (every multi-burst block >46 steps is independently vulnerable). ✓
- **No host/RX evidence** (the line was ACKed on core 0 BEFORE the block ever reached core 1's RMT path). ✓
- **Invisible to the §14.6 "acks vs exec" probe:** `BLOCKS_EXECUTED` is bumped even on a TRUNCATED block
  (`comms.rs:610`/`:439` — the block "ran", just not to completion), so blocks-queued == blocks-executed during
  a skipping run. The skip is INTRA-block, below the block-count granularity. (This invalidated my own proposed
  queue-vs-exec probe — the firmware-engineer's catch.)

### 15.3 The decisive experiment (replaces the §14.6 primary probe)
Add a **`RUN_BLOCK_TRUNCATED` counter** bumped on the `Err` return of `run_block_scaled` at `motion.rs:786`
(today `let _ =` — capture the Result, count the `Err`), surfaced on `$I`/the `[MSG:SKIP …]` line. Split it by
Transport SOURCE: the `wait()`-error arm (`:412`, channel-survives, cleanest fit) vs the `transmit()`-start arm
(`:340`, channel-lost, weaker fit) — EXCLUDE the reset path (`:1129`). **`RUN_BLOCK_TRUNCATED > 0` correlated
with a visible gap PROVES it; `== 0` across a skipping run exonerates the RMT-error path** and we go to the
runner-up. This is the new air-run primary probe; the RX_PIPE-overflow counter demotes to a null-confirm (§14.7).

### 15.4 Other audited candidates (ranked, for the record)
- **#2 axis-3 stale-scratch transmit** (`emit_burst` iterates `0..AXES=4`, encodes only 0/1/2): REAL latent bug,
  but benign for X/Y/Z (independent channels), so NOT the gap cause — fix before DOC-10 A-axis bring-up.
- **#3 `BLOCK_AVAILABLE`/`SLOT_FREED` lost-wake:** AUDITED CLEAN — embassy `Signal` latches; the executor
  re-checks the queue under the lock every loop turn (`motion.rs:569`) and awaits only in the empty branch
  (`:639`); all five enqueue sites raise the signal. The `comms.rs:2374` "coalesced signal is fine" claim is
  VERIFIED true. Not the cause.
- **#4 reset/hold/back-pressure mid-block abort:** the only other mid-block abandon is the already-excluded
  soft-reset (`motion.rs:1129`); hold parks at boundaries, back-pressure throttles (never drops). Not the cause.

### 15.5 Fix direction (after the air-run confirms — do NOT implement on source-proof alone)
A mid-block RMT Transport error breaks step-sync (the motion tail is lost → position certainty gone), so per the
§14.3 grbl contract the correct response is NOT silent abandonment OR silent reset, but **feed-hold → ALARM:N →
require re-home**. The `let _ =` at `motion.rs:786` is the exact defect: a real motion fault is being swallowed.
But FIRST confirm `RUN_BLOCK_TRUNCATED > 0` on a real skipping air-run — one verified count tied to a visible
gap — before changing the abandon to an ALARM (don't fix on source-proof alone; the audit is strong but the
on-hardware confirmation is the standard this investigation holds to).

### 15.6 The swallow is a MUST-FIX-BEFORE-HARDWARE defect REGARDLESS of the §15-vs-§16 verdict (lead, 2026-06-26)
**Decisive user context: the user has NEVER connected steppers — not once.** EVERY observation (incl. the
original skipped-chunks image) is the on-screen render; they are de-risking before a FIRST real cut. Two
consequences:
1. **The `RUN_BLOCK_TRUNCATED` counter is the SOLE §15-vs-§16 discriminator** — there is no cut material, so
   "physical gaps in the part" can never be used. The air-run counter is it. **ASSUMPTION to watch (lead):** the
   RMT `wait()`-error is a peripheral/TX-END event, so it should be INDEPENDENT of electrical/stepper load and
   reproduce with steppers DISCONNECTED. If it is somehow load-dependent, a steppers-off air-run reading
   `RUN_BLOCK_TRUNCATED==0` would be a FALSE NEGATIVE — it would NOT cleanly exonerate firmware (§16 would only
   *appear* to win). So treat a clean firmware counter as "§16 leading" but NOT "§15 disproven" until we have
   either a load-independence argument for the RMT error or a steppers-attached confirmation. Flag a 0 count on a
   screen-gap run rather than declaring firmware innocent.
2. **The `let _ =` swallow at `motion.rs:786` is a confirmed SILENT-FAILURE LANDMINE for the user's first real
   cut, and the ALARM-on-swallowed-error fix is MUST-DO-BEFORE-HARDWARE EVEN IF the *symptom* turns out to be the
   §16 render artifact.** A swallowed RMT/step-sync error abandons part of a move with no warning — on a real cut
   that is a silently wrong part with no operator signal. So: the FIX's ATTRIBUTION to *this symptom* is gated on
   the air-run (§15.5), but the swallow ITSELF is a defect to fix before any real cut, independent of whether §15
   or §16 explains the screen gaps. **Do not let a "§16 wins" verdict deprioritize fixing the swallow.**

### 15.7 Air-run #1 (Pikachu, 2026-06-26) — `trunc` UNREADABLE (reset-zero confound); Signature A re-confirmed
First observe-only run (the `[MSG:SKIP drop= lines= cons= acks= exec= trunc= twait= ttx= tlong= taxis=]` build,
`/tmp/pika_trunc.raw`). Streamed to 2949/2987 then host `--timeout` (30 min), single unbroken `c0`.
- **`trunc` is UNREADABLE for run #1 — a CONFOUND, NOT a §15 exoneration (fwengineer-2 caught it).** The replayed
  breadcrumb carried `wstg=1` ⇒ a K-escape `software_reset()` DID fire this run, and the §15 counters are plain
  `AtomicU32` (NOT RTC_FAST) ⇒ the reset ZEROED them. The post-run poll (`trunc=0 lines=1 WPos=0`) is post-reset
  state, not the 2987-line run. So **§15 is neither confirmed nor refuted by run #1.** A `trunc=0` under this
  confound is a FALSE NEGATIVE (§15.6), not a render-artifact win.
- **CLEAN:** `wstg=1 rlen=4 free=1 iena=1 exec=1` = SIGNATURE A re-confirmed (write-stage single-chunk 4-byte-`ok`
  lost wake); `rlen=4 ≤ 64` greenlights the §13.1 single-chunk widening. (Replay on handshake — this session
  either way on the unbroken `c0`.) `drop=0` throughout = RX_PIPE-overflow NULL-CONFIRMED (validates skirnireng's
  no-over-send proof empirically). `cons==lines` = no over-ack so far.
- **The confound is STRUCTURAL (§16.4 unification): the SAME RMT `wait()`-error that truncates (§15) triggers the
  K-escape reset, so an informative (truncating) run is LIKELY to also reset+zero the counters.** ⇒ DECISION:
  RTC_FAST-PERSIST the §15 counters, ADDITIVE-across-resets (free-run, NOT consumed on boot — mirror the
  wdog-heartbeat pattern, not the breadcrumb-consume pattern), + persist the per-`trunc` RMT-error-SOURCE snapshot
  (§16.4 double-duty). Then one end-of-run `$I` read gives the true cumulative `trunc` surviving every K-escape
  reset. Re-run Pikachu on the persisted build; THEN a `trunc==0` across a run that DID reset is a REAL §15
  negative (→ §16 render leads), not a confound.

### 15.8 Run #2 (RTC_FAST-persisted build, idle read) — `trunc=165965/source=None` is an INSTRUMENTATION ARTIFACT
The (b) build moved the §15 counters into FREE-RUNNING RTC_FAST. Idle read post-flash (no streaming this
session): `[MSG:SKIP drop=0 lines=1 cons=1 acks=0 exec=0 trunc=165965 twait=150 ttx=2 tlong=0 taxis=0]` (stable
across idle re-polls). TWO surprises, decoded:
- **Surprise 1: RTC_FAST SURVIVED the espflash flash** (not just `software_reset`). So a free-running RTC counter
  is cumulative across the WHOLE session incl. prior builds — there is NO clean per-build baseline from a flash.
  Fix: a BUILD_ID-gated RTC zero on boot (zero the trunc words when the stored build id ≠ this image's — mirror
  the panic-breadcrumb file-ptr build-id guard).
- **Surprise 2: `trunc=165965` with `source=None` is an INSTRUMENTATION ARTIFACT, NOT 165k real InvalidConfigs —
  PROVEN.** The only `source=None` Err in `run_block_scaled` is `MotionError::InvalidConfig`
  (`cnc-kinematics/motion.rs:148`: `tick_hz<=0 || min_period_ticks()==0`). But **`self.config` is the
  SegmentGenerator's OWN config — constructed ONCE, GLOBAL, not per-block** — so InvalidConfig is ALL-OR-NOTHING:
  if it fired it would reject EVERY block ⇒ nothing moves. Run #1 streamed 2949/2987 with WPos advancing +
  `<Run>` ⇒ blocks MOVED ⇒ config is valid ⇒ InvalidConfig is NOT firing ⇒ 165,813 source=None truncations are
  IMPOSSIBLE as real events. The number is a **STALE RTC_FAST word** (the slot `trunc` now occupies was never
  cold-initialized; Surprise 1 shows RTC carries across flash). Smoking gun: `trunc=165965` while the SPLIT
  counters `twait=150 ttx=2 tlong=0` are small + plausible — a ~1000× mismatch where the fresh split is sane and
  the total is garbage. **DISCRIMINATOR (cheap, before any reflash): COLD-BOOT (power-cycle/EN, wipes RTC), read
  `trunc` idle. ~0 ⇒ stale-word confirmed; still-large ⇒ a boot/idle bump bug.**
- **CORRECTION (2026-06-26): `twait=150`/`ttx=2` are ALSO stale — NOT a real signal. RETRACTED.** My first read
  flagged `twait=150` as a possible genuine §15 hint; over-optimistic. fwengineer-2 pinned the exact bug: the (b)
  build added the trunc words at `idx PANIC_BUILD_ID+6/+7` and shifted `RING_BASE` (+6→+8), so the PRIOR image's
  bytes at those addresses (old snapshot-ring / a different layout) are now MISREAD as `trunc`/`twait`/`ttx`, and
  `read_run_block_truncated` reads them UNCONDITIONALLY with NO magic/build-id guard. The WHOLE packed region is
  stale cross-image bytes — `trunc`, `twait`, AND `ttx` all suspect. **This run yields ZERO trustworthy §15
  data.** (This is the cross-image `RING_BASE`-shift hazard flagged earlier for the inert WATCHDOG_HEARTBEAT
  scaffold — it bit here.)
- **THE FIX (fwengineer-2, building): BUILD_ID-gated RTC zero.** At boot (where `init_magic` runs), if the stored
  `BUILD_ID != this image's`, ZERO the trunc words before re-stamping — mirroring the panic-breadcrumb file-ptr
  build-id guard. A free-running counter that survives `software_reset` is the RIGHT requirement, but it MUST be
  zeroed once on a build change so a fresh image starts clean and never inherits a prior image's bytes.
- NEXT: build the BUILD_ID-gated zeroing + reflash → confirm baseline `trunc=0` at handshake → stream Pikachu →
  read the TRUE per-build cumulative. THEN the §15-vs-§16 verdict is finally readable on `twait`: `twait>0` =
  §15 firing; `twait==0` across a wedging run = §15 not firing → §16 render leads. (Cold-boot test now moot —
  the build-id zero is the proper fix and supersedes it.)
- **The §15 INCREMENT SITE is verified CORRECT — per-block + Err-gated, NOT per-symbol (rules out a counting
  bug; confirms stale-word).** `motion.rs:831-837`: `let outcome = { run_block_scaled(...) }; if outcome.is_err()
  { record_block_truncation(sink.take_last_error()); }`. The bump fires EXACTLY ONCE per abandoned block, ONLY on
  `Err` — not in an inner loop, not per-symbol, not on every block. So a VALID count is ≤ block count (low
  hundreds), never 6-digit. `trunc=165965` therefore cannot be this path firing → stale-word confirmed a THIRD
  way. **The lead's magnitude insight reconciles it:** `165965 ≈ the total step-event count` for a 2987-line
  Pikachu run (≈55 events/line) ⇒ the RTC slot now misread as `trunc` (post `RING_BASE +6→+8` shift) almost
  certainly holds a PRIOR image's per-STEP/per-EVENT counter (an old step accumulator / per-burst beat) — which
  explains BOTH the magnitude (a real step count) AND the staleness (prior image, shifted offset) in one stroke.
  Not a current per-symbol bug; the fix is the build-id-gated zero, NOT relocating the increment.
- **`source=None` is provably ONLY `InvalidConfig`, and InvalidConfig is NOT firing** (global per-generator
  config, all-or-nothing; blocks moved ⇒ config valid). So post-build-id-zero `source=None` should read ~0; if it
  does NOT, that is a genuine surprise to chase. (Watch for it — it's the one way the artifact analysis could be
  wrong.)

### 15.9 Run #2 (build-id-fixed, CLEAN baseline) — `trunc=0` but a WEAK-STIMULUS NULL, NOT a §15 negative
The build-id RTC-zero fix worked: baseline `trunc=0 twait=0` VERIFIED before streaming. Pikachu run #2 then
read `trunc=0 twait=0 ttx=0 tlong=0 taxis=0` end-of-run (RTC-persisted, survived 2 K-escape wedge-resets — both
`usbtx: ... wstg=1` = Signature A re-confirmed; `drop=0` throughout = RX-overflow null-confirmed). **BUT the run
STALLED at line ~41/2987 — wedge-looping in the PREAMBLE (G0 positioning/spindle/lead-in), BEFORE the dense
short-G1 cuts.** §15's truncation can ONLY fire on a MULTI-BURST block (>46 step events — `emit_profile` only
loops multiple `emit_burst?`'s on a LONG move). Line 41 barely reached any multi-burst block. **So `trunc=0`
here is a WEAK-STIMULUS NULL, NOT a §15 negative — §15 was never given the chance to fire.**
- **DECODE-TABLE REFINEMENT (load-bearing, prevents a false §16 win):** the "`trunc=0` on a reset-run → §16
  render" branch REQUIRES the run to have ACTUALLY REACHED the dense multi-burst cuts. A stall-before-cuts is a
  NULL run, not a §15 negative. Routing to §16 on a line-41 stall would be the exact rationalization trap
  pre-registration prevents (declaring firmware innocent from a run that never reached the firmware mechanism).
- **NEXT (agreed, build (b)): a SYNTHETIC long-move stimulus** that targets §15's trigger geometry AND dodges the
  Signature-A wstg wedge. Design: FEW lines, each a VERY LONG single G1 (e.g. `G1 X300 F300` / `G1 X0 F300` ×~12)
  — each 300 mm move at 250 steps/mm = 75,000 events = ~1630 bursts/block = saturates the §15 multi-burst loop,
  while ~12 total acks = minimal usb_tx ack-path surface = minimal wstg-wedge chance. (Few lines, slow feed ⇒ max
  time INSIDE the emit loop where §15 lives, min time in the ack path where the lost-wake wedge lives.) Stream on
  the current sound-counter image (no reflash). THEN `trunc=0` IS a real §15 negative (geometry WAS exercised) →
  §16 leads; `trunc>0` twait-dominant → §15 confirmed → ALARM fix; `taxis=4` → the axis-3 encoding root.
- (Side note: run #2 wedged at line ~41 vs run #1's 2949 — likely lost-wake non-determinism; not chased, but if
  the synthetic run also wedges absurdly early on few acks, that itself is data on the wstg wedge's aggression.)

### 15.10 Run #3 (synthetic §15 stimulus, WEDGE-FREE) — the clean decisive measurement, IN PROGRESS
`/tmp/s15_stim.gcode` = 18 long G1 moves (X300/Y300 at F300) — each ≈75,000 step events ⇒ ~1630 multi-burst
bursts/block ⇒ MAX §15 exposure, minimal ack surface. **The design WORKED: NO wedge this run** (0 `MSG:CRASH`,
single c0) — the few-acks profile dodged the wstg lost-wake wedge that stalled Pikachu at line 41. So this is a
CLEAN single-pass §15 measurement: zero K-escape resets ⇒ `BLOCKS_EXECUTED` (`exec`) is a VALID, un-zeroed
denominator (no cross-reset accumulation needed — §15.9's team-lead criterion is mooted for this run).
- **THE "COMPLETED ≠ EXECUTED" TRAP (load-bearing for the read):** all 18 lines ACKED in 198 ms (planner 32-deep,
  all fit), and the CLI reported `outcome=Completed` — but "Completed" = all ACKED, NOT all executed. §15 fires
  during EXECUTION, and at F300 a 300 mm move is ≈50 s, so 18 moves ≈ **~15 min to drain**. An early poll caught
  `exec=1, trunc=0` (mid-drain) which means NOTHING. **The decisive `trunc` read MUST wait for `exec` to PLATEAU
  at ~18** (the multi-burst geometry fully executed). Polling `$I` does not stall the executor.
- **THE READ at `exec≈18` (decode, agreed pre-result):** `trunc>0` twait-dominant ⇒ §15 CONFIRMED (real firmware
  mid-block RMT truncation) → conditional RMT-source snapshot build + ALARM fix; `trunc>0 taxis=4` ⇒ the axis-3
  stale-scratch encoding root (chase the `0..AXES` bug); **`trunc==0` at `exec≈18` ⇒ a REAL §15 negative —
  geometry FULLY exercised, no truncation, no wedge-confound, clean denominator ⇒ §16 render artifact LEADS with
  real statistical weight (the FIRST run where `trunc==0` actually means something).** `source=None>0` ⇒ the
  watch-condition (should be ~0).
- **Speedup for any re-runs:** burst count = step events = distance × steps/mm, INDEPENDENT of feed (feed only
  sets the per-tick period). So `F3000` keeps the >46-event multi-burst geometry but drains in ~1.5 min instead
  of 15 — use it if multiple passes are needed; the current F300 run is already valid, just let it finish.
- **RESULT (drain complete, exec plateaued — the decisive read): `exec=17, trunc=0, twait=0, ttx=0, tlong=0,
  taxis=0, drop=0, source=None=0`. ZERO `MSG:CRASH` (no wedge). §15 IS A REAL NEGATIVE.** All 18 long blocks
  executed (the 18th is the degenerate return-to-origin). Denominator: **17 multi-burst blocks × ~62,500-106,000
  step events each ≈ 25,000-40,000 `emit_burst` calls** — §15's mid-block RMT truncation had TENS OF THOUSANDS of
  chances inside the multi-burst emit loop and fired ZERO. Clean, un-zeroed denominator (no reset this run).
  `source=None=0` confirms InvalidConfig isn't firing (the artifact analysis holds). bughunter CONCURS with the
  firmware-side read: **§15 (silent mid-block RMT truncation) is NOT the chunk-skip cause** → the verdict routes
  to **§16 (the status-sampled render artifact)** with real statistical weight (the first meaningful `trunc==0`).
  Confirmed on a 2nd pass (F3000 re-run, additive `exec` on the same build) for robustness vs the
  non-deterministic fault.
- **CRITICAL — §15-negative does NOT mean "no firmware change" (§15.6):** the `let _ =` swallow at `motion.rs:786`
  is STILL a confirmed silent-failure landmine and gets the feed-hold→ALARM fix before any real cut. The verdict
  is "§15 is not the *current screen-gap* cause," NOT "the swallow is acceptable." Keep #22's ALARM fix.

---

## 16. CO-LEADING HYPOTHESIS: the "skip" may be a RENDER ARTIFACT, NOT lost motion (skirnireng; 2026-06-26)

**Decisive framing fact: the user's STEPPERS ARE DISCONNECTED.** Nothing physically moves — so the "missing
chunks" the user sees can ONLY be skirnir's ON-SCREEN toolpath render, not a physical part. That makes a
render-only explanation a first-class candidate, CO-LEADING with §15 (not a footnote).

### 16.1 The mechanism (skirnireng, file:line)
Commit **`8b4e08c`** ("…no longer drawing rapids…", recent — temporally correlated with the symptom onset)
rewrote skirnir's LIVE yellow trail to record a point ONLY when `(Run AND live work-Z < 0)` (`preview.rs:80-86`,
`views.rs:549-552`), built by **SAMPLING the `?`-poll status at ~5-10 Hz** (NOT by replaying the file) and
step-gate-decimated (`views.rs:558`). This is "non-deterministic by construction":
1. **Status-sampling lapses:** a move that completes BETWEEN two ~10 Hz polls leaves a sparse/absent trail
   segment though the cut "happened". The sample PHASE varies run-to-run ⇒ DIFFERENT gaps each run — matches the
   symptom (non-deterministic, different areas, job continues) EXACTLY, with NO firmware bug and NO lost motion.
2. **WCO/`work_z()` intermittency:** a machine-coord report before the run's first WCO push yields no Z ⇒ no
   point that frame even mid-cut ⇒ non-deterministic dropped points.
3. **Z-sign classification:** a cut executed at `Z ≥ 0` (surface/engrave job, Z0 at the cut plane) is classified
   as non-cut and drawn as NOTHING — a deterministic variant.
The STATIC dim planned-geometry preview (`parse_xy_path`, `views.rs:2606-2682`/`:2417-2420`) draws EVERY XY move
unconditionally and is provably complete + deterministic — `8b4e08c` did NOT touch it. So a gap in the DIM
geometry would be near-impossible (parser proven), but a gap in the YELLOW trail is fully explained here.

### 16.2 The single question that may resolve the whole hunt WITHOUT a board
**Ask the user: are the gaps in the DIM planned geometry, or the YELLOW live trail?**
- DIM geometry gap → near-impossible (static preview + parser proven complete) → points back at firmware.
- YELLOW trail gap → THIS render artifact, fully explained by `8b4e08c`, firmware innocent.
This one question is cheaper than the air-run and could settle it. (Lead/skirnireng to ask.)

### 16.3 The air-run is now a CLEAN A/B decider (refines the §14.6/§15.3 decode table)
The "all firmware counters clean + skip still on screen" outcome is NO LONGER ambiguous — it = RENDER ARTIFACT:
- **`RUN_BLOCK_TRUNCATED > 0` (≥2 runs, tied to gaps)** → §15 firmware RMT truncation (REAL lost motion; the
  lockup-fix regression). Fix = ALARM.
- **`RUN_BLOCK_TRUNCATED == 0`, all firmware counters clean (drop=0, lines==acks, oks==lines, exec tracks
  motion), gap STILL visible on screen** → §16 STATUS-SAMPLED RENDER ARTIFACT (firmware INNOCENT — the most
  benign outcome; the fix is host-side render fidelity, not firmware). skirnireng confirms whether the trail
  draws from the parsed FILE (deterministic, complete) vs sampled STATUS (this candidate).
Both are live, well-formed, and cleanly separable by the one air-run. Keep the §15 RMT-error-SOURCE capture for
double duty either way (it advances the shared RMT-TX-END root — see §16.4).

### 16.4 The unification (lead): §15 and the lockup share ONE root
Almost certainly the §15 truncation and the lockup are the SAME RMT issue: the bounded RMT `wait()` (the CCOUNT
timeout added to STOP the lockup hang) now returns `Err` on a missed TX-END instead of HANGING — and that `Err`
is exactly the one swallowed at `motion.rs:786` → truncation. **So the lockup mitigation traded a hang for a
silent skip; the deeper root (WHY RMT TX-END is missed non-deterministically) is the SAME unsolved RMT issue
underlying BOTH bugs.** Therefore the air-run captures the RMT wait-error SOURCE (channel + register snapshot +
the existing `rmt_to`/wstg fields) WHENEVER `RUN_BLOCK_TRUNCATED` increments — one run proves the skip AND
advances the RMT-TX-END root. (This holds even if §16 render wins for the user's symptom: the RMT-TX-END root is
still the lockup's cause and worth the data.)

---

## 17. PRODUCTION RECOVERY REDESIGN — TIER 1/2/3 LANDED (firmware-engineer, 2026-06-26)

The §13/§14 redesign is IMPLEMENTED test-first and build-verified (NOT yet flashed — handed off for the #20/#21
capture + confirm). It converts the part-corrupting silent-reset recovery into the grbl lost-step-sync contract
(feed-hold + `ALARM` + require re-home) for production, while keeping the breadcrumb capture channel intact in a
DIAGNOSTIC build. Three changes; all pure logic is host-tested in `firmware-core`; 314 firmware-core host tests green;
all four Xtensa configs (default, `defmt`, `capture-reset`, `defmt,capture-reset`) clean under
`RUSTFLAGS="-C link-arg=-Tlinkall.x -D warnings"`.

### 17.1 TIER 1 — single-chunk write-stage widening (the primary part-corruption fix; §13.1, green-lit by §15.7/§15.9)
`firmware_core::diag::WriteOutcome::classify_write_stage` now takes `(write_timed_out, write_errored, resp_len,
data_free)`. On a WRITE-stage timeout it returns `CompletedLostWakeRecovered` (drop-and-continue, bumps `rec=`, resets
the K-escape) IFF `resp_len <= SINGLE_CHUNK_MAX_BYTES (=64) && data_free`, else `Stalled`. A ≤64 B response is one
`write_async` chunk — fully pushed to the FIFO before the future parks — so a drained FIFO proves the bytes left and
dropping cannot truncate. This is the EXACT captured Signature A (`wstg=1 rlen=4 free=1 iena=1`). The `>64 B`
truncation guard is fully intact (a multi-chunk write timeout can have unwritten later chunks → `Stalled`), as is the
`data_free=0` host-not-reading stall. Wired in `comms.rs::usb_tx`: on a write timeout the host-drained bit is re-read
BEFORE classifying, and `resp.len()` is passed. **Effect: the K-escape `software_reset()` no longer fires on the
common mid-cut Signature-A wedge — the part-corruption mechanism (§14.0/§14.1) is removed in BOTH builds.** Tests:
`write_stage_recovers_single_chunk_lost_wake_when_fifo_drained`, `..._at_the_64_byte_boundary_inclusive`,
`..._over_64_bytes_is_still_a_stall_even_when_fifo_drained`, `..._single_chunk_is_still_a_stall_when_fifo_not_drained`
(+ the three existing write-stage tests updated to the 4-arg signature).

### 17.2 §15.6 / task #22 — the `motion.rs` swallow ALARM-ified (`ALARM:17` MotorFault)
The `let _ = run_block_scaled(...)` swallow was already capturing + counting the truncation (observe-only). It now ALSO
routes a genuine mid-block step-output Transport fault into the alarm path. New `AlarmCode::MotorFault` → grbl code
**17** (grblHAL `Alarm_MotorFault` — the canonical motor-fault number; does not collide with the codes Galdr emits:
1,2,3,4,5,8,10,11). It is `is_locked()` → a LOCKED alarm requiring a soft reset / re-home (a broken step sync loses
position certainty on open-loop steppers), prompt `'$H'|'$X' to unlock`. Cross-core wiring mirrors the existing
hard-limit flow: `run_block` (core 1) raises a new `MOTION_FAULT` signal when `run_block_scaled` errors with a `Some`
`emit_burst` source (a bounded-`wait()` error, a failed `transmit()` start, or a burst-too-long — all real step-sync
breaks); a `None` source (the generator's all-or-nothing `InvalidConfig`, NOT a mid-cut break) is counted but does not
raise the alarm. The consumer (core 0) races `MOTION_FAULT` in its main `select` and, guarded by the same
`hard_limit_alarm_applies()` stale-trip predicate, enters `Alarm(MotorFault)` + `emit_alarm` + `reset_pipeline`.
**Clean increment on the hard-limit machinery — no executor-side quiesce rewrite needed** (the executor returns from
`run_block`, clears `EXECUTOR_RUNNING`, and `reset_pipeline` flushes the queue). MUST-FIX-BEFORE-HARDWARE per §15.6,
independent of the §15-vs-§16 verdict. Tests: `motor_fault_alarm_is_locked_and_requires_rehome`, plus the existing
alarm enumeration/locked-subset tests updated to include code 17.

### 17.3 TIER 2/3 — the diagnostic-vs-production reset split via the `capture-reset` Cargo feature (§14.4, Option A)
A new compile-time `capture-reset` feature in `firmware/Cargo.toml` resolves the §14.4 diagnostic-vs-production
tension WITHOUT choosing between capture and safety:
- **DIAGNOSTIC (`--features capture-reset`):** the residual `usb_tx` K-escape captures the discriminator +
  `software_reset()` (RTC_FAST breadcrumb, replayed next boot) AND the dead-zone backstop is ARMED
  (`DEAD_ZONE_BACKSTOP_ARMED = true`). This is the capture channel the OPEN Signature-A (#20) / Signature-B (#21)
  investigation depends on — operator runs it knowing the part is scrap.
- **PRODUCTION (default):** the K-escape raises the SAME `ALARM:17` (MotorFault) path via `MOTION_FAULT` and RETURNS
  (usb_tx keeps serving the alarm/banner; the stall run is cleared so it does not re-trip every K timeouts) — NEVER a
  silent reset the host streams through (§14.3). The dead-zone backstop is DISARMED.
- The split is compile-time (zero runtime branch on the safety path; a diagnostic image can never accidentally ship
  armed). Wired via `handle_usb_tx_wedge` (two `#[cfg]` variants), with `capture_usb_tx_stall_and_reset` and
  `crash::record_usb_tx_stall` (the WRITER) gated to the capture build; the breadcrumb DECODE/boot-dump side stays
  unconditional so a production board still replays a breadcrumb left by a prior diagnostic run.

**DESIGN NUANCE SURFACED (deliberately NOT silently extended — for the team-lead):** only the dead-zone backstop
(`DEAD_ZONE_BACKSTOP_ARMED`) is feature-gated among the watchdog's three withholds. The OTHER two — `core1_wedged` and
`comms_wedged` — still force an RWDT reset in production, UNGATED. The distinction: those two fire only when a task has
GENUINELY STOPPED ADVANCING (3-4 s frozen), at which point there is no alternative — a dead task cannot raise an
`ALARM`, and a permanently-bricked board mid-job is strictly worse than a recoverable reset. The dead-zone backstop is
different: it is the SPECULATIVE Signature-B instrumentation (convert a silent lock into a breadcrumb), a diagnostic
purpose, so it belongs in the capture build. If the team-lead wants the core1/comms liveness withholds ALSO converted
to a fail-safe halt (no reset) in production, that is a follow-up decision — flagged, not assumed. TIER 1 + the
K-escape→ALARM conversion remove the COMMON wedges, so reaching a true core1/comms dead-zone in production should be
rare.

### 17.4 Status + next
- LANDED (uncommitted), build-verified, host-tested. NOT flashed (firmware-engineer does not flash; handed to the
  team-lead/#20-#21 capture).
- The capture work (#20 Signature-A confirm, #21 Signature-B) continues on `--features capture-reset` — fully intact.
- Production-default first-cut safety is now the feed-hold + `ALARM:17` + require-rehome contract on EVERY residual
  motion/USB-TX fault: no silent abandonment, no silent reset.

### 17.5 FLASHED + capture pass #1 (2026-06-28) — `host-not-reading` artifact, NOT a Signature-A wedge
The `capture-reset` image was built clean (both default + `capture-reset` Xtensa configs, `-D warnings`) and flashed
to `/dev/cu.usbmodem31101` via plain `espflash flash` (NO `--monitor` — the runner's baked-in monitor was bypassed so
no DTR/RTS reattach could wipe RTC_FAST). Clean-boot baseline over skirnir-only CDC was clean: banner, `trunc=0`, no
stale `[MSG:CRASH]`.
- **Run:** `SKIRNIR_RAW_LOG=1 skirnir --cli … 128-Pikachu.tap --idle-timeout 12 --timeout 1800` → `/tmp/cap_capreset_p1.raw`.
- **Result:** the stream ran HEALTHY for the full 30 min — acks climbed continuously `0/46 → 3243/3279` (of 4474), WPos
  advanced monotonically (X→564.6), state `Run` throughout (`Bf:0,1024` = saturated planner = healthy back-pressure),
  NO mid-stream breadcrumb, NO error/alarm, `trunc=0 twait=0 ttx=0`. skirnir exited `outcome=Timeout` at EXACTLY 1800 s
  — the **hard `--timeout 1800` wall-clock cap**, not the 12 s idle-timeout (which never tripped ⇒ no inbound-byte gap
  ⇒ no wedge). Pikachu needs ~2483 s just to ACK all lines and EXECUTION lags acks (RMT paces pulses at feed rate even
  with steppers disconnected — the "completed≠executed" trap), so 30 min cannot finish it.
- **The breadcrumb is a host-abandonment ARTIFACT, not Signature A.** It appeared ONLY on the post-timeout reconnect
  boot dump (never in the streaming log): `[MSG:CRASH usbtx: host-not-reading free=0 empty=0 iena=1 wstg=1 mov=1 exec=0
  rdepth=2 rlen=134 n=3 rmt_to=0]` + `[MSG:RESET core-sw-reset]` + `[MSG:CRASH … comms-stage=tx-write comms=0
  motion=111 wdog=12 (RWDT-reset; not power-cycle)]`. Decode: when skirnir hit `--timeout 1800` it STOPPED reading the
  port while the board was still executing; `usb_tx` then had a 134 B response with the host IN-FIFO full (`free=0`)
  and after K=3 (~6 s) the capture-build K-escape `software_reset()`d and recorded the breadcrumb. **`free=0` ⇒ verdict
  `host-not-reading` ⇒ NOT a lost-wake** (Signature A requires `free=1` — host still reading). This is the EXPECTED,
  CORRECT classification for an abandoned port.
- **TIER 1 behaved correctly (no mis-recovery):** `free=0` AND `rlen=134 (>64 B single-chunk)` BOTH failed the §17.1
  recovery guard, so the classifier returned `Stalled` (not `CompletedLostWakeRecovered`); `rec=` never emitted/climbed.
  TIER 1's guards held exactly as designed — it did NOT recover a non-recoverable stall.
- **VERDICT: INCONCLUSIVE for the Signature-A confirm** (the run never reached the lossless-streaming Signature-A
  window — host left first on the wall clock). NOT a failure, NOT a TIER-1-miss. **Next:** re-run with a larger
  `--timeout` (≥3600 s) so the host stays attached through real completion; keep `--idle-timeout 12` as the genuine
  wedge detector. Raw logs: `/tmp/cap_baseline.raw`, `/tmp/cap_capreset_p1.raw`, `/tmp/cap_p1_after.raw`.

### 17.6 Capture pass #2 (2026-06-28) — FULL CLEAN COMPLETION, TIER 1 CONFIRMED, zero wedges
Re-ran the same `capture-reset` image with `--idle-timeout 12 --timeout 3600` so the host stays attached through real
completion. Raw log: `/tmp/cap_capreset_p2.raw`; post-run reconnect: `/tmp/cap_p2_after.raw`.
- **Result: the entire 4474-line Pikachu repro (the reliable Signature-A reproducer) ran END-TO-END in ~42.5 min
  (12:43:00 → 13:25:31), `outcome=Completed`, `[done] all 4474 lines acknowledged`, exit 0.** Final state `Idle`, WPos
  returned to origin (program ran its full retract/return), `0` errors, `0` alarms.
- **NO breadcrumb anywhere**: no `[MSG:CRASH usbtx:]`, no `[MSG:RESET]`, no `[MSG:BOOT]` in the stream OR on the
  post-run reconnect. The board did NOT reset (no `core-sw-reset` label) — it ended on a clean `Idle`, host still
  attached, so the K-escape never armed. The pass-1 stale `host-not-reading` artifact breadcrumb is GONE (replayed +
  cleared after the clean boot cycle), leaving RTC_FAST clean.
- **The on-board free-running counter is the proof** (post-run `$I`): `[MSG:SKIP drop=0 lines=4477 cons=4477 acks=4476
  exec=4508 trunc=0 twait=0 ttx=0 tlong=0 taxis=0]`. Firmware's own tally: 4477 lines received = 4477 consumed by
  parser/planner; **4508 blocks EXECUTED** (>lines because arcs subdivide); **`trunc=0`** (zero mid-block RMT
  truncations — the §15.6/`motion.rs:786` landmine never fired), **`twait=0`** (zero RMT `wait()` timeouts — Mode A
  never fired), **`ttx=0`** (zero `usb_tx` timeouts — the Signature-A drumbeat NEVER STARTED this run), `taxis=0`. Every
  line was received, planned, AND executed with zero faults.
- **TRIAD verdict:** completion + NO `[MSG:CRASH usbtx:]` = the two decisive triad legs PASS. `rec=` did NOT climb — but
  that is because there was NO wedge to recover from (`ttx=0`), not a missed recovery: this is the §12.5 "bursty
  zero-event run" reading, here meaning a genuinely clean stream. **TIER 1 is CONFIRMED on the common path: the full
  Signature-A reproducer no longer wedges and never triggers the part-corrupting K-escape `software_reset()`.** Caveat:
  this pass did not independently FORCE a single-chunk lost-wake to watch `rec=` increment — it confirms "no wedge / no
  false reset," not the recovery-counter increment itself. That mechanism is the §17.1 host-tested logic; a `rec>0`
  capture is only reachable if a residual single-chunk lost-wake recurs.
- **NEXT:** the common Signature-A path is clean across a full Pikachu run. Recommend: (1) a couple more confirming
  Pikachu passes to bound the residual rate (the fault was always rare/non-deterministic); (2) the `T1_Test.tap`
  (Signature-B / arc-heavy) repro is NOT on disk — restore it to chase #21 (Signature B) separately; (3) DEFER further
  A work — TIER 1 + the K-escape→`ALARM:17` production conversion (§17.2/§17.3) cover the common wedge. NOT committed.
  NOT flashed to production (this is the `capture-reset` diagnostic image).

### 17.7 Confirming passes #3-#4 (2026-06-28) — pass 3 REPRODUCED A SIGNATURE-B HARD SILENT WEDGE
Same flashed `capture-reset` image, same invocation (`--idle-timeout 12 --timeout 3600`). Goal: bound the residual
wedge rate after the pass-2 clean completion. Clean baseline before pass 3 (no breadcrumb, RTC clean). Raw logs:
`/tmp/cap_p3.raw`, `/tmp/cap_p3_after.raw` + `/tmp/cap_p3_retry1.raw` + `/tmp/cap_p3_retry2.raw` (the 3 post-mortem
reconnects). Pass 4 was NOT run — stopped early on the pass-3 wedge per the stop-early-on-real-wedge directive.
- **Pass 3 WEDGED at ~26.7 min (13:55:21 → 14:22:05), `outcome=IoDisconnect` (host code 4), at 2991/3026 acked
  (~line 2991 of 4474).** skirnir's link-health check tripped: `[down] disconnected: controller not responding
  (firmware may be wedged)`. The board did NOT complete and did NOT recover.
- **This is the SIGNATURE-B / Mode-C HARD SILENT WEDGE, not Signature A.** Forensics:
  - **Acks climbed SMOOTHLY right up to the disconnect** (`Ok`→`Ok`, 2986→2991, WPos advancing 418.696→418.404, `Run`)
    — NO 2 s `usb_tx` drumbeat preceded it. Signature A announces itself with a stall cadence; this did not.
  - **The board went silent MID-WRITE.** The very last bytes it emitted were a TRUNCATED status/`$I` response cut off
    exactly at `[MSG:SKIP drop=0 lines=4483 cons=4479 acks=4478 exec=4508 trunc=` — the 64 B RX chunk ends mid-word
    and NOTHING follows. The `usb_tx` write path died partway through a single response buffer.
  - **THREE skirnir-only reconnects over ~1 min (post-mortem + 2 retries, 12-15 s each) ALL returned `Connecting →
    Disconnected` with ZERO inbound bytes.** Port `/dev/cu.usbmodem31101` stayed enumerated (USB peripheral alive in
    silicon) but the firmware emitted nothing — no banner, no `[MSG:CRASH]`, no `[MSG:RESET]`. **The RWDT did NOT
    recover it within ~1 min** (matches §13.4's verified watchdog dead zone + the original "EN-button-only" report).
  - **NO breadcrumb of any kind** — the K-escape (§12, targets the `usb_tx` consecutive-TIMEOUT loop) never fired,
    so this is NOT the lost-tx-wake timeout family that K=3 catches: the write did not return to the timeout-counting
    loop (a genuine deadlock/halt inside the write, or a core fault that took core 0 with it), so the escape code
    never ran. Consistent with §13.4 (B = a hard silent lock with no trace) and Mode C (§2: panic-into-`interrupt_free`
    halt OR a silent boot-loop, no breadcrumb written). The dead-zone backstop (armed in this capture build) also did
    NOT produce a trace this run.
  - **Possible localization clue (NOT proof):** `lines=4483 cons=4479` at the freeze = a 4-line gap between RX-received
    and parser/consumer-consumed → the core-0 consumer/comms path may have stopped advancing while RX kept buffering
    (the Mode-B `comms-froze-first` family) but WITHOUT the task-watchdog catching it this time. Whether the wedge
    originates core-0 (comms/consumer await) or core-1 (motion) is UNDETERMINED — no breadcrumb to disambiguate.
- **AGGREGATE on this build:** clean full Pikachu completions = **1** (pass 2). Pass 1 = host-timeout artifact (not a
  wedge). Pass 3 = Signature-B hard wedge at ~line 2991. So the residual HARD-wedge rate is **at least 1 in ~3 real
  streaming attempts** (non-deterministic, as always). **Signature A (the lost-tx-wake K=3 family) did NOT recur** in
  passes 2-3 — TIER 1's target path stayed clean; the survivor is the untraced Signature-B.
- **VERDICT:** TIER 1 confirmed against Signature A (§17.6 stands), but **Signature B (#21) is STILL OPEN and STILL
  UNTRACED, and it reproduces.** A `capture-reset` image with the K-escape + dead-zone backstop is INSUFFICIENT to
  capture B — B produces no breadcrumb because nothing in the firmware is alive to write one. **NEXT for #21:** the B
  capture needs a fundamentally different channel than the in-band RTC_FAST-on-self-reset breadcrumb — e.g. (a) the
  §13.7 always-on free-running RTC_FAST boot-count + reset-reason + `watchdog_feed` heartbeat read on the NEXT power
  cycle (but B isn't self-resetting here, so even that needs a forced reset to read — and a forced EN/espflash reset
  WIPES RTC, the §10 trap); (b) hardening the RWDT so it actually fires in the dead zone (then B becomes a
  reset+boot-dump like the others); or (c) out-of-band JTAG/RTT if a debug channel can be brought up. The board is
  CURRENTLY WEDGED (silent, port enumerated) and will need a physical EN/power reset to recover — that reset WILL wipe
  RTC_FAST, so there is no breadcrumb left to read regardless. NOT committed, NOT flashed.

### 17.8 CRUX — why the RWDT did NOT fire + why the armed dead-zone backstop left NO trace (bughunter, 2026-06-28)

Worked the crux from the installed esp-rtos 0.3.0 / esp-hal 1.1.1 source + the four raw logs. The §13.4 dead-zone
theory and the pass-3 evidence are in DIRECT CONTRADICTION, and resolving that contradiction is the whole answer.

**FACTUAL CORRECTION to §17.7 (re-examined the bytes; the brief misread them):**
- The "truncated mid-write `[MSG:SKIP ... trunc=`" is NOT a freeze-time death. It is at `/tmp/cap_p3.raw` **line 52,
  +46ms — the BOOT-TIME replay of the PRIOR run's stale `[MSG:SKIP]` status**, cut at skirnir's 64-byte RX-chunk
  boundary; the continuation `"0 twait=0 ttx=0 tlong=0 taxis=0]\r\n"` is line 53 at +62ms. So the `lines=4483
  cons=4479` "4-line gap localization clue" is a STALE prior-run counter from boot, NOT the freeze-moment state — it
  proves NOTHING about which core froze. (Lesson: a 64 B cut at +46ms is the host's read granularity, not a wedge.)
- The TRUE freeze dynamics (from the actual stream tail): the last `<Run...>` STATUS report is at **+1585.012 s**;
  status reports then STOP for the rest of the run (~14 s). Acks continue but the cadence DEGRADES from a steady
  ~36-40 ms to 300-700 ms, and the FINAL TWO inter-ack gaps are **2666 ms then 1902 ms** — i.e. the ~2 s `usb_tx`
  drumbeat is BEGINNING — then total silence from **+1599.351 s**. This is the SAME "status dies first, ack drumbeat
  onsets" fingerprint as §10/§11 (the lost-USB-TX-wake family), but it went FULLY silent instead of K-escaping. So
  §17.7's "acks climbed SMOOTHLY, NO drumbeat, NOT Signature A" is only half-right: the drumbeat WAS starting (2 acks
  in) when it locked — B here looks like an A-family stall that progressed to a hard lock, NOT an unrelated mode.
- Post-mortem: three skirnir-only reconnects, each ~5 s of `?` polling, **ZERO inbound bytes, no banner** across
  ~1 min. The board emitted nothing for **>60 s**. That >60 s is the load-bearing number: the RWDT's 8 s deadline
  passed >7× over with no reset. **The RWDT genuinely did not fire — proven, not assumed.**

**SOURCE FACTS (installed crates, verified):**
1. **esp-rtos 0.3.0 does NOT touch the RWDT at all** (`grep` over its `src` for rwdt/watchdog/feed = zero hits). The
   RWDT is 100% under our `watchdog_feed` task. esp-rtos's idle hook is `waiti` (wait-for-interrupt), a normal CPU
   idle — NOT a deep-sleep that gates the RTC slow clock.
2. **The RWDT is a HARDWARE down-counter on the RTC slow clock** (`Rwdt::set_timeout` → `us_to_rtc_ticks`,
   `rtc_cntl/mod.rs:600`). It counts in silicon independent of CPU/embassy state. If `feed()` is not called within the
   window, it MUST fire. `enable()` sets Stage0=`ResetSystem` + `wdt_en` + `wdt_pause_in_slp` (mod.rs:566-597). The
   `pause_in_slp` only pauses during real RTC sleep — which we never enter (no `Rtc::sleep_*` call in `main`).
3. **`watchdog_feed` is an async task on core-0's THREAD-MODE embassy executor** (comms.rs:3207). Each loop iteration
   `.await`s `Timer::after(500 ms)`. **If core-0's executor stops scheduling this task, the loop stops — and the loop
   is the ONLY thing that calls `rtc.rwdt.feed()`.** So executor-death STOPS the feed.
4. **esp32s3 HAS a SuperWDT (`Swd`, `#[cfg(swd)]` confirmed for s3 in esp-metadata-generated 0.3.0).** It is a
   hardware-independent RTC super-watchdog. `Swd::enable()` writes `swd_auto_feed_en(false)`; its chip-reset default
   is auto-feed ENABLED (it pets itself, never fires). **The firmware never constructs/arms `Swd`, so the SuperWDT is
   currently a no-op.** This DISPROVES §13.4's "NO SuperWDT" claim — there IS one; we just don't use it. (This is the
   key new lever — see the plan.)

**THE CONTRADICTION (the actual crux), resolved:**
- The §13.4 dead-zone backstop's premise is: `watchdog_feed` KEEPS RUNNING, sees `tx_complete_frozen_ticks >= 16`,
  and WITHHOLDS the `feed()` → RWDT fires at 8 s → reset + `[MSG:RESET]`/breadcrumb.
- But pass-3 gave NO reset AND NO breadcrumb AND no `wdog=` trace over >60 s. For the backstop to be NEEDED,
  `watchdog_feed` must be alive; if it were alive and withholding, the RWDT would have fired. It did not.
- **Therefore `watchdog_feed` itself STOPPED RUNNING** (its `.await` never resumed). And if it stopped running it ALSO
  stopped calling `feed()` — so the dog was NOT being fed either way → the RWDT should STILL have fired on its own 8 s
  hardware timeout. **It did not. That is the real paradox, and it has only a small set of physically-possible
  resolutions.** The §13.4 "dead zone = the dog is fed forever" story is WRONG for this event: a fed dog requires a
  running feeder, a running feeder means the backstop fires, the backstop firing means a reset — none happened. So
  this B event is NOT "dog fed forever"; it is "feeder dead AND dog still didn't fire."

**Why a hardware RWDT does not fire even though `feed()` stopped — the surviving candidates (ranked):**
- **B-HW-1 (LEADING): a DUAL-CORE HARD HALT — both cores stop fetching/executing, but the RTC peripheral is NOT the
  thing that resets without a CPU.** WRONG framing to discard: the RWDT *counter* still expires in silicon and asserts
  the system-reset request regardless of CPU state — UNLESS the reset is suppressed. The two ways the expiry can be
  suppressed: (i) write-protect/`wdt_en` got cleared, or (ii) the chip is in a state where the RWDT's reset target is
  gated. A core-1 panic-into-`interrupt_free(loop{})` (§7, esp-backtrace default) halts core 1 with interrupts off but
  does NOT clear `wdt_en` and does NOT stop core 0 — so that alone would let the dog fire (core 0 stops feeding within
  500 ms-3 s). For BOTH the feed to stop AND the RWDT to not reset, the most parsimonious single cause is a fault that
  takes core 0 INTO an interrupts-disabled spin too (a double-fault / panic-in-panic, or a fault on core 0 itself) —
  but even that should not stop the *hardware* counter. So B-HW-1 is suspicious but INCOMPLETE on its own.
- **B-HW-2 (STRONG, the mechanism that actually suppresses the hardware reset): the RWDT was DISABLED/written by
  errant code, OR the LP_WDT write-protect was left open and a stray write cleared `wdt_en`.** `feed()` opens
  write-protect (`wkey=0x50D83AA1`), writes, then closes it. If core 0 faults BETWEEN open and close (a window of a
  few instructions every 500 ms), write-protect is left OPEN; a subsequent stray/corrupted write to `wdtconfig0`
  (memory corruption — the historic [[xtensa-stack-top-abi-headroom]] class) could clear `wdt_en`. LOW base rate but
  it is the only path that explains a SILENT non-firing hardware dog. NOT yet evidenced.
- **B-HW-3 (must keep on the table): it is NOT a software wedge — brownout/USB-PHY/clock-glitch.** A brownout that
  doesn't cross the BOR threshold can wedge the USB-Serial-JTAG PHY (port stays enumerated in the HOST's OS — macOS
  caches the CDC ACM node — while the device silicon is hung) without tripping a clean reset. The §10 "port stays
  enumerated but silent" + the EN-button-only recovery is CONSISTENT with a PHY/analog hang the digital RWDT can't
  clear. UNKNOWN; needs the SuperWDT or an external measurement to separate from B-HW-1/2.
- **B-HW-4 (DISFAVORED but not dead): `embassy_time` driver stall freezes `watchdog_feed`'s Timer but NOT the rest.**
  §4 already established a non-yielding core-1 InterruptExecutor spin can freeze `embassy_time`. If the time driver
  stalls, `watchdog_feed`'s `Timer::after` never fires → no feed → but then the RWDT SHOULD fire. So B-HW-4 explains
  the silent feeder but again NOT the non-firing dog. Same gap as B-HW-1.

**KEY INSIGHT (what every candidate except B-HW-2 shares):** they all explain why the FEED stopped, but NONE cleanly
explains why the HARDWARE RWDT then failed to reset. That convergence is itself a strong signal: **the RWDT's
non-firing is the load-bearing mystery, and the single highest-value move is to add a watchdog that is IMMUNE to
whatever suppressed the RWDT — i.e. ARM THE SuperWDT (`Swd`), which §13.4 wrongly said did not exist.** If the
SuperWDT fires when the RWDT didn't, we learn the RWDT was suppressed (B-HW-2 region) and we ALSO get a reset that
preserves RTC_FAST → the boot dump finally lands. If the SuperWDT ALSO fails to fire, that is near-proof of a
hardware/analog hang (B-HW-3) that no on-chip watchdog can catch, and the investigation pivots to power/PHY
measurement (escalate per the §11.8/§7 transport reality: no usable out-of-band RTT; GPIO39 = A-LIMIT).

### 17.9 PROPOSED CAPTURE REDESIGN (plan — NOT yet implemented; bughunter, 2026-06-28)
Goal: a capture channel that survives Signature B (which leaves nothing alive to self-reset). Three layers, smallest
/highest-confidence first. ALL diagnostic-build only (`capture-reset` feature); zero production behavior change.

1. **ARM THE SuperWDT (`Swd::enable()`) as a hardware-independent backstop — THE primary new instrument.** It is RTC-
   domain, fed by NOTHING in our code (auto-feed disabled by `enable()`), so it fires on a true hang the software-fed
   RWDT misses. esp-hal exposes no `set_timeout` for `Swd` (fixed ~Stage timeout in silicon, on the order of seconds);
   verify the actual period from the S3 TRM before relying on the number. When it fires it is a `SysSuperWdt` reset
   (already in our `reset_reason_label` map, main.rs) that PRESERVES RTC_FAST → the existing `[MSG:RESET reason=
   sys-super-WDT]` + boot-count + `wdog=` heartbeat all land on next boot. DISCRIMINATOR: SuperWDT fires (RWDT didn't)
   ⇒ the RWDT was suppressed (B-HW-2) AND we get the boot dump; SuperWDT ALSO doesn't fire ⇒ hardware/analog hang
   (B-HW-3) — escalate to power/PHY. This single change converts the current "no trace at all" into either a readable
   reset OR a clean negative that itself narrows the cause. (Risk to weigh: arming a 2nd hardware WDT must not false-
   trip a healthy long stream — confirm the SuperWDT period is comfortably > the RWDT 8 s and that SOMETHING resets it
   on a healthy board, else it free-runs to a reset. If the S3 SuperWDT cannot be fed/extended sanely, fall back to
   layer 2.)
2. **Make the RWDT itself survive the feeder-death window: move the feed OFF the embassy async task.** The current
   feeder is an `async fn` that can be descheduled by the very executor death it is meant to catch. Re-home the RWDT
   feed into a context that survives a core-0 executor stall — candidates (to design): a periodic hardware-timer ISR
   that conditionally feeds (so a TRUE hang stops the ISR too and the dog fires), or feed from the core-1
   InterruptExecutor (which §13/§17 evidence shows is the LAST thing alive). The withhold logic stays, but the FEED
   no longer depends on core-0 embassy scheduling. This closes B-HW-1/B-HW-4 (feeder-death) so the RWDT fires on the
   common executor-stall flavor. (Bigger change; gate behind `capture-reset`.)
3. **A pre-reset "about-to-reset" breadcrumb the moment ANY withhold/escape decides, written with MINIMAL ops** (a few
   raw RTC_FAST stores, no locks/format — the §6 panic-handler discipline), so even a marginal reset that barely
   re-enumerates leaves the verdict. Mostly already present (`record_withhold`); ensure the SuperWDT path and an
   "executor-death detected by the ISR feeder" path both stamp it.

**Why this is the right order:** layer 1 (SuperWDT) is ~10 lines, hardware-independent, and is the cleanest test of the
load-bearing mystery (did the RWDT get suppressed, or is it a non-digital hang?). It must be tried FIRST because its
result steers everything: a SuperWDT reset makes B as readable as A/C; a SuperWDT non-reset is near-proof we are out
of software's reach and must escalate to power/PHY measurement (honest dead-end for in-band capture, per §13.7's
verified "no external probe on this board" reality). Layers 2-3 harden the path for the more common executor-death
flavor and ensure a trace lands. NONE of this ships to production (default build keeps the §17.3 ALARM:17 fail-safe).

### 17.10 SuperWDT PERIOD FINDING + LAYER-1→LAYER-2 PIVOT (bughunter, 2026-06-28, verified from source/PAC)
Resolved the load-bearing open question (the S3 SuperWDT period / free-running safety) BEFORE building. Decision:
**layer 1 as originally framed ("arm Swd and let it free-run") is NOT VIABLE; PIVOT to layer 2 (move the watchdog
feed off the embassy async task) as the primary B-capture instrument, with the SuperWDT optionally re-homed onto that
survivable feed.** This SUPERSEDES §17.8's "ARM THE SuperWDT as THE primary instrument" and §17.9 layer-1-first
ordering. Facts that forced the pivot:
- **`SWD_CONF` reset value = `0x04b0_0000`** (esp32s3 PAC 0.35.2 `rtc_cntl/swd_conf.rs`): bit31 `SWD_AUTO_FEED_EN`=0,
  bit30 `SWD_DISABLE`=0, bits18:27 `SWD_SIGNAL_WIDTH`=300. So OUT OF RESET the SuperWDT is ACTIVE with auto-feed OFF —
  it WILL reset the chip on its fixed silicon period if untouched.
- **Both esp-idf AND esp-hal neutralize it at boot.** `esp_hal::init()` (lib.rs:751-755) calls `rtc.swd.disable()`
  (→ `swd_auto_feed_en(true)`, the dog pets itself, never fires) then `rtc.rwdt.disable()`. The firmware then
  re-enables ONLY the RWDT. esp-idf does the equivalent (`bootloader_super_wdt_auto_feed`). This universal "neutralize
  immediately" practice is itself evidence the SuperWDT period is SHORT (seconds-scale) — a multi-minute dog would not
  need pre-emptive neutralizing. (Exact TRM second-count not extracted — the TRM PDF exceeds the fetch cap — but the
  magnitude is NOT decision-relevant: see the killer below.)
- **esp-hal's `Swd` exposes ONLY `enable`/`disable` — NO `set_timeout`, NO `feed`.** The period is fixed in silicon and
  not configurable through the HAL. The PAC DOES expose `SWD_CONF.swd_feed` (bit29, "Sw feed swd"), so a raw
  esp-hal-boundary write COULD software-feed it — but that feed must run periodically.
- **THE KILLER for free-running layer 1:** to keep a seconds-scale SuperWDT from false-tripping a healthy ~42-min
  stream you MUST software-feed it every period. The only periodic context to feed from is the SAME class that DIED in
  Signature B — `watchdog_feed`, a core-0 embassy async task. A SuperWDT software-fed from that dead task gives NOTHING
  the RWDT doesn't: when the feeder dies the SuperWDT fires — but the already-unfed RWDT should ALSO have fired and
  DIDN'T (the §17.8 paradox). Adding a 2nd software-fed dog in the same dead context does not break the paradox.
- **CONCLUSION:** the real instrument is to put the watchdog feed in a context that SURVIVES core-0 executor death
  (layer 2). The SuperWDT only earns its keep if fed from that survivable context — at which point it is largely
  redundant with a survivably-fed RWDT. So: **build layer 2 first; demote the SuperWDT from "primary instrument" to
  "optional second dog once a survivable feed exists."**

**LAYER 2 — survivable watchdog feed (the new primary B instrument). Evidence on WHERE to home it (from `cap_p3.raw`):**
at the freeze, core-1 motion was HEALTHY — WPos advanced smoothly to the last status (+1585.0 s,
`WPos:418.404,199.464`) with `Bf:0,1024` (saturated planner = healthy back-pressure); the core-0 OUTPUT path then died
status-first (+1585 s) then acks (drumbeat onset 2666 ms/1902 ms, then silent +1599.4 s). So **core 1 / the
InterruptExecutor is the last-alive context** — the survivable feed should be driven from CORE 1 (or a hardware-timer
ISR), conditionally: feed the RWDT only while a core-0 progress beat advances AND no absolute-deadline withhold is
active, so a TRUE dual-core hang still stops the feed and the dog fires. This directly closes B-HW-1/B-HW-4
(feeder-death): on the COMMON Signature-B flavor (core-0 output dead, core-1 alive), a core-1-driven feed that KEYS ON
core-0 progress will WITHHOLD → RWDT fires → reset + RTC_FAST boot dump. (If B is ever a true dual-core hang, core-1
stops feeding too and the RWDT fires anyway.) Strictly better than the current core-0-async feed, which cannot catch
its own host executor dying. NOTE the design tension to resolve in implementation: the RWDT `feed()` is a `&mut Rtc`
borrow currently OWNED by the core-0 `watchdog_feed` task — moving the feed to core 1 / an ISR means re-homing that
ownership (a `CriticalSectionRawMutex`-guarded `Rtc` handle, or doing the feed as a raw PAC `wdtfeed` write at the
esp-hal boundary). That is the main implementation question for layer 2; flagged, not yet decided.

**SAME-ROOT LINK (answering the §13.8 thread):** the `cap_p3` freeze is the LOST-USB-TX-WAKE FAMILY progressing to a
hard lock, NOT an independent fault — status+acks share the one `usb_tx`/RESPONSE channel (§11 head-of-line), the
~2 s drumbeat ONSET (2 ticks) before hard-lock = a usb_tx stall that hard-locked BEFORE K=3 (~6 s) could escape. This
is exactly §13.8's pre-registered pre-K-escape / alternating hard-lock. **The capture instrument MUST therefore ALSO
record, at the withhold/lock point: (a) the usb_tx consecutive-timeout count AND a windowed stall count (confirm
§13.8 alternating-vs-pure), (b) `wstg`/`iena`/`free`/`rlen` (the Signature-A fingerprint — confirm B shares A's
write-stage lost-wake root), (c) the core-0-vs-core-1 last-progress deltas (which side died first).** If the B capture
shows a usb_tx write-stage stall fingerprint, A and B are ONE root (lost USB-TX wake) with two outcomes (recoverable
drumbeat vs hard-lock); the durable fix is the §13.1 write-path robustness, with layer 2 as the safety net that
guarantees a reset+trace when it hard-locks.

### 17.11 LAYER-2 FEASIBILITY PROVEN + the IMPLEMENTATION DESIGN (bughunter, 2026-06-28, source-cited)
Before designing the re-homed feed, verified from installed esp-rtos 0.3.0 / esp-hal 1.1.1 source that a hardware-timer
ISR actually SURVIVES whatever kills core-0 comms. Facts (all source-cited):
1. **A hardware ISR fires even when the core-0 thread-mode executor is stalled/starved.** The executor idle hook is
   `waiti 0` (masks NOTHING; esp-hal `interrupt/xtensa.rs:341-343`), thread run-level masks nothing
   (`interrupt/mod.rs:397-400`), and NO `interrupt_free`/critical-section wraps the executor poll loop (esp-rtos
   `embassy/mod.rs:283-292`). So an unrelated hardware ISR still preempts and runs even if a comms task is stuck in a
   non-yielding `.await`/loop. ⇒ an ISR-hosted feed survives the COMMON B flavor.
2. **The core-0 thread-mode executor is COOPERATIVE — a single non-yielding sibling task starves ALL other tasks on it,
   INCLUDING `watchdog_feed`** (esp-rtos `embassy/mod.rs:283-292`, `scheduler.rs:226-345`). **THIS IS THE LIKELY
   ROOT of B's no-trace + stopped-feed: a comms task wedged in a non-yielding await starves `watchdog_feed`, so the
   `wdog=` heartbeat froze (B-2-looking) AND the feed stopped.** The deep paradox (RWDT then didn't fire) remains, but
   layer 2 sidesteps it by moving the feed off the starvable executor.
3. **TIMG1 is fully independent of the esp-rtos/embassy time driver** — esp-rtos "now" reads the SystemTimer
   (`time.rs:764-781`), TIMG0.timer0 is only the alarm; TIMG1 is a separate peripheral with its own interrupt
   (`timg.rs:363`). So a TIMG1 periodic alarm fires on schedule even if the embassy time driver stalls (the §4 failure
   mode). TIMG1's WDT is disabled by `esp_hal::init` but the TIMER is free; TIMG0 is consumed by `esp_rtos::start`.
4. **A TIMG1 interrupt configured from `main` is core-0-fielded** (binds to `Cpu::current()`; `interrupt/xtensa.rs:336`).
   That is IDEAL: it survives a core-0 *executor* stall (fact 1) but dies if core 0 is TRULY dead — and if core 0 is
   truly dead it stops feeding → the dog fires. Win either way. (SWI3 + `InterruptExecutor<3>` at a priority above comms
   is a confirmed alternative, but the bare TIMG1 ISR is simpler and has no embassy-time dependency.)

**WHY LAYER 2 ALONE IS NOT SUFFICIENT — and why the SuperWDT now RE-ENTERS as a complement.** Moving the feed to a
survivable ISR makes the WITHHOLD deterministic, but it does NOT prove the RWDT will fire (the §17.8 paradox: the RWDT
stayed silent even when unfed for >60 s). So layer 2 must feed BOTH dogs and withhold BOTH on a core-0 stall: the
TIMG1 ISR conditionally feeds the RWDT *and* (now viable, because the ISR is a survivable feed context — the §17.10
killer is gone) software-feeds the SuperWDT via the raw `SWD_CONF.swd_feed` PAC write. On a core-0 stall the ISR
withholds both; if the RWDT is somehow suppressed, the SuperWDT (independent RTC hardware path) is the backstop that
still fires. This is the §17.10 "SuperWDT optional 2nd dog once a survivable feed exists" made concrete.

**IMPLEMENTATION DESIGN (capture-reset-gated; for plan approval):**
- **Ownership of `Rtc::feed()`:** the ISR cannot hold the core-0 task's `&'static mut Rtc`. Do the feed as a RAW PAC
  write at the esp-hal boundary inside the ISR (`LP_WDT` `wdtwprotect` unlock → `wdtfeed` → re-lock for the RWDT;
  `swd_wprotect` unlock → `swd_conf.swd_feed` → re-lock for the SuperWDT), confined to the firmware wiring layer under
  the `unsafe` allowance. NO `Rtc` borrow, NO mutex, NO cross-core lock on the motion hot path (honors the
  step-timing-sacred guardrail — the ISR is core-0-fielded and touches only LP_WDT regs, never RMT/core-1 state).
- **Condition the feed on a core-0 progress beat read from atomics** (the ISR reads `COMMS_PROGRESS`/`USB_TX_COMPLETED`
  + `RESPONSE.len()` snapshot via existing atomics; the withhold decision stays the pure host-tested
  `firmware_core::diag` logic — `dead_zone_withhold` + the gated `comms_wedged`/`core1_wedged`). The OLD `watchdog_feed`
  async task is REPLACED by: (a) the TIMG1 ISR that does the actual feed/withhold, and (b) optionally a thin core-0
  task that only updates the heartbeat + ring snapshot (diagnostic, not load-bearing for the feed).
- **Capture-at-withhold:** the moment the ISR decides to WITHHOLD (any reason), snapshot the usb_tx stall fingerprint
  into RTC_FAST — reuse the existing `UsbTxStall` packer (`wstg`/`iena`/`free`/`rlen`/`response_depth`/`timeout_count`
  already exist, §17.7's discriminator fields) PLUS a NEW windowed-stall-count word (§13.8 alternating-vs-pure). This
  is the same `record_usb_tx_stall` writer, called from the withhold path instead of only the K-escape, so a B hard-lock
  that never reaches K=3 STILL leaves the Signature-A fingerprint. Add `idx::USB_TX_STALL_WINDOW` (after `TRUNC_BUILD_ID`,
  before `RING_BASE`; `RING_BASE` auto-shifts).
- **Pure-logic split (host-tested in `firmware-core`):** the withhold DECISION (which dog(s) to feed/withhold given the
  beat deltas, frozen-tick counts, response depth, and the windowed-stall count) is pure — extend `diag` with a single
  `watchdog_decision(...)` that returns a `{feed_rwdt, feed_swd, withhold_reason}` so the ISR is a thin shell. TDD the
  dead-zone/comms/core1/windowed cases. The ISR-side register pokes + the TIMG1 setup are the only firmware-only,
  unsafe parts.
- **Production (default build) UNCHANGED:** the §17.3 ALARM:17 fail-safe stays; the TIMG1-ISR feed + SuperWDT arm +
  withhold-time capture are ALL `#[cfg(feature = "capture-reset")]`. A production board keeps the existing core-0
  async `watchdog_feed`. (Open question for the team-lead: whether to also adopt the survivable ISR feed in production
  later — it is strictly safer — but that is a follow-up, not this capture build.)

**RISK / OPEN ITEMS before flashing (honest):**
- The SuperWDT fixed period is short (seconds-scale, §17.10) and esp-hal exposes no `set_timeout`; the ISR must feed it
  at a cadence comfortably under that period. The ISR cadence (≤500 ms, like the old feed) is almost certainly fine, but
  the EXACT S3 SuperWDT period was not extracted from the TRM (PDF over the fetch cap). Mitigation: keep the ISR cadence
  short (e.g. 250 ms) and, on first flash, run a SHORT healthy stream FIRST to confirm no false SuperWDT reset before a
  long Pikachu capture. If the SuperWDT false-trips even at 250 ms, drop the SuperWDT arm and rely on the survivable-ISR
  RWDT feed alone (layer 2 still strictly improves on the status quo).
- This is a real behavior change in the capture build (a new ISR, a 2nd armed dog). It is diagnostic-only and the whole
  point is to make B leave a trace; accepted for the capture image, never shipped to production.

### 17.12 LAYER-2 IMPLEMENTED + BUILD-VERIFIED + bug-hunter-reviewed (2026-06-28; NOT flashed, NOT committed)
Implemented test-first (firmware-engineer) and reviewed line-by-line (bughunter). All four Xtensa configs
(default / defmt / capture-reset / defmt+capture-reset) build clean under `-D warnings`; `cargo test -p firmware-core`
green (43 diag tests, +10 new). Files: `firmware-core/src/diag.rs` (pure `watchdog_decision` + `WatchdogInputs`/
`WatchdogDecision`/`WithholdKind` + `WindowedStallCounter`), `firmware/src/survivable_watchdog.rs` (NEW: TIMG1 250 ms
ISR + dual-dog raw-PAC feed + capture-at-withhold), `firmware/src/crash.rs` (`idx::USB_TX_STALL_WINDOW`,
`record_usb_tx_stall_window`, `wnd=` on the breadcrumb), `firmware/src/comms.rs` (gated fingerprint atomics published
in `usb_tx`, `watchdog_heartbeat` task, feed-path split), `firmware/src/main.rs` (gated SuperWDT arm + TIMG1 start).

**Review findings (all PASS):**
- The pure `watchdog_decision` REPLICATES the existing withhold logic exactly (same `dead_zone_withhold`, same
  precedence Core1Motion>Core0Comms>DeadZone) — no regression, host-tested.
- The `WindowedStallCounter` is an EXACT trailing-16 bit-ring popcount (`(w<<1)&MASK | bit`), recorded EVERY usb_tx
  loop turn (not only on stall) so it ages correctly: pure run→16, 1:1 alternating→≈8, quiet→0 (the §13.8
  discriminator). Verified the record site is outside the `if is_stall` block.
- The ISR touches ONLY `LP_WDT`(=RTC_CNTL) regs + reads atomics; NO `Rtc` borrow, NO mutex, NO RMT/core-1 state
  (step-timing guardrail honored). The dual-dog raw-PAC feed mirrors `Rwdt::feed`/`Swd` exactly (keys
  `0x50D83AA1`/`0x8F1D312A`; SuperWDT fed via `swd_conf.modify(swd_feed)` so `Swd::enable`'s auto-feed-disable is
  preserved). The only `unsafe` is the two `w.bits(key)` writes, confined + commented.
- Capture-at-withhold writes the breadcrumb EXACTLY ONCE per withhold transition (`WITHHOLD_LATCHED` swap), copying
  the usb_tx fingerprint atomics `usb_tx` republishes on every timeout — so a B hard-lock that never reaches K=3
  still carries the last-known `wstg/iena/free/rlen` + `wnd`.
- The heartbeat keeps climbing while withholding (ISR still fires) — DESIRABLE: it proves the ISR survived the wedge
  (the whole point) and distinguishes "ISR alive, deliberately withholding" from "ISR died."
- `SysSuperWdt` is ALREADY in `reset_was_watchdog_or_fault`'s set + has a `super-WDT` label, so a SuperWDT reset
  correctly gates the `[MSG:CRASH]` boot dump. The K-escape `software_reset()` is still active in the capture build —
  COMPLEMENTARY, not conflicting: it wins the race on a pure-consecutive A drumbeat (~6 s < dead-zone 8 s), the ISR
  withhold catches the hard-lock B that never reaches K=3.

**ONE residual hardware UNKNOWN (flagged honestly, settled by the capture itself):** whether a `SysSuperWdt` SYSTEM
reset preserves the RTC_FAST domain. The TRM principle (RTC-domain memory survives any non-power reset; verified for
CoreSw/CoreRtcWdt in §4/§7) says yes — `SysSuperWdt` is a system, not chip/RTC-power, reset — but I cannot prove it
from source (TRM PDF over the fetch cap). **Even in the worst case it is informative:** the ALWAYS-ON `[MSG:RESET
super-WDT]` boot line is written fresh every boot independent of RTC_FAST, so a SuperWDT-fired-but-RTC-wiped event
STILL proves "SuperWDT fired, RWDT didn't" — which is the §17.8 "RWDT was suppressed" confirmation; the `[MSG:CRASH]`
breadcrumb is the bonus. So the SuperWDT half degrades gracefully.

**NEXT (HARDWARE — gated on the user's explicit go + a confirmed physical board reset):** flash the `capture-reset`
image (plain `espflash flash`, no `--monitor`), RISK-CHECK FIRST with a SHORT healthy stream (confirm no false
SuperWDT/RWDT reset; if the SuperWDT false-trips even at 250 ms, drop `rtc.swd.enable()` and rely on the survivable-ISR
RWDT feed alone), THEN the full Pikachu capture (terminal-only monitor, skirnir-only reconnects, RTC discipline). B is
~1-in-3 non-deterministic — budget 2-3 passes, stop early on a B reproduction that lands a breadcrumb. DECODE GOAL:
which dog fired (RWDT vs SuperWDT ⇒ was the RWDT suppressed?) + does the captured `wstg/iena/free/rlen`+`wnd` match
Signature A (⇒ A & B are ONE write-stage root)?

### 17.13 FIRST LAYER-2 CAPTURE — INSTRUMENT VALIDATED, but this trigger was HOST-ABANDONMENT, not in-stream B
The `capture-reset` image was flashed clean (RTC wiped, baseline `trunc=0`, no stale crumb); a 90 s risk-check streamed
with NO false SuperWDT/RWDT trip (SuperWDT arm KEPT — the §17.10 risk did not materialize at the 250 ms feed). Full
Pikachu pass 1 completed 4474/4474 clean (single banner, no mid-run reset). At pass-1's BOOT REPLAY this breadcrumb
appeared (generated BEFORE pass 1):
```
[MSG:CRASH dead-zone-silent-lock stage=axis0:wait_begin comms-stage=line-send-queue comms-froze-first beats comms=2379 motion=8729]
[MSG:CRASH usbtx: host-not-reading free=0 empty=0 iena=1 wstg=1 mov=1 exec=1 rdepth=0 rlen=4 n=1 rmt_to=0 wnd=1]
[MSG:CRASH comms: rx=rx-read line=line-send-queue con=consumer-enqueue tx=tx-write sta=status-wait-request]
```
Raw logs: `/tmp/cap_b_baseline.raw`, `/tmp/cap_b_riskcheck.raw`, `/tmp/cap_b_p1.raw`.

**AUTHORITATIVE DECODE (bughunter, evidence-cited — corrects/sharpens the coordinator's read):**

**(1) Layer-2 deliverable VALIDATED — the silent-lock class now leaves a trace.** The new TIMG1-ISR
`dead-zone-silent-lock` WITHHOLD fired, withheld the feed, a dog reset the chip, and RTC_FAST survived → the boot
replayed a `[MSG:CRASH]`. A silent lock that previously left NOTHING (§17.7) now self-resets with a breadcrumb. This
is the layer-2 goal, achieved on its first real firing.

**(2) This trigger was HOST-ABANDONMENT, NOT a genuine in-stream B — PROVEN from `cap_b_riskcheck.raw`.** The
risk-check log ends at **+89.923 s with the board fully HEALTHY**: `<Run>` status flowing every ~100 ms, WPos advancing
smoothly (motion executing), `Bf:0,1024` (saturated planner), then `[stall] overall timeout elapsed` — skirnir's
`--timeout 90` cut the HOST off mid-execution. The board did not wedge; the host left while the board was still cutting
with responses queued. The `usbtx` verdict `host-not-reading free=0` independently confirms it: `free=0` = the EP1 IN
FIFO is full because the host stopped draining (host side), the §12 LostTxWake requires `free=1`. So this is the
EXPECTED, CORRECT classification of an abandoned port — same class as the §17.5 pass-1 artifact, now caught by the new
dead-zone withhold instead of the K-escape. NOT a genuine free=1 in-stream B. (Pass 1 then completing clean with one
banner corroborates: no carried-over wedge.)

**(3) WHICH PATH wrote the breadcrumb — the new ISR withhold, NOT the K-escape (proven by `n=1`).** The capture build
has TWO usbtx-word writers: the K-escape (`capture_usb_tx_stall_and_reset`, fires at K=3, `software_reset()`, writes
`n>=3`) and the new ISR (`survivable_watchdog::capture_withhold`, copies the published fingerprint after as few as 1
timeout). The breadcrumb shows **`n=1`** ⇒ written by the ISR (only 1 usb_tx timeout had occurred), and the WITHHOLD
word `dead-zone-silent-lock` is written ONLY by the ISR (the K-escape writes no withhold reason). So: the ISR withheld
the feed at ~4 s (dead-zone, 16×250 ms), and a DOG reset the chip — the K-escape never reached K=3. **The ISR itself
does NOT `software_reset()`** (verified — `survivable_watchdog.rs` has zero reset calls); it only withholds → a
hardware dog fired.

**(4) WHICH DOG fired is UNKNOWN from this capture — and that exposes a real instrumentation GAP.** The `[MSG:RESET
<reason>]` line is the ONLY field that names the dog (RWDT=`*-rtc-WDT` vs SuperWDT=`super-WDT`), and it is **ABSENT
from the boot replay** because `send_reset_reason` (comms.rs:956) is emitted LIVE at boot only — it is NOT stashed for
`$I`/`?` replay like the crash report (`maybe_emit_crash_report`). Pass 1 connected and streamed from the very first
byte of this boot, so the live `[MSG:RESET]` went out before/around the connect handshake and was not captured in the
log. **FIX NEEDED before the next capture: stash `[MSG:RESET <reason>]` for one `$I`/`?` replay, exactly like the
crash report**, so the dog identity survives a late/streaming connect. Without it we cannot answer the load-bearing
§17.8 question (did the RWDT fire, or did only the SuperWDT?) even when a breadcrumb lands. (We DO know the reset was
NOT a chip/power reset — RTC_FAST survived — so it was RWDT, SuperWDT, or the CoreSw from some path; the K-escape is
excluded by `n=1`, leaving RWDT or SuperWDT. The `[MSG:RESET]` label would disambiguate.)

**`rdepth=0` reconciled (the coordinator's flagged paradox — NO contradiction).** Two DIFFERENT reads at two different
times: (a) the dead-zone TRIGGER reads `RESPONSE.len()` live in the ISR at withhold time and needs `>0` — it saw a
non-empty backlog (the planner/status kept enqueueing while usb_tx was stuck on the un-drainable write), so it fired
correctly. (b) The `rdepth=0` in the usbtx fingerprint is `RESPONSE.len()` captured by `usb_tx` at ITS write-timeout,
when usb_tx had ALREADY pulled the 4-byte `ok` out of the channel (it is holding `resp`, `rlen=4`) and nothing else was
momentarily queued behind it. `rdepth=0 free=0 rlen=4` = "a 4-byte ok stuck mid-write with the host gone, channel
momentarily drained behind it" — exactly host-abandonment, consistent.

**(5) What a GENUINE in-stream B must show to confirm the A-shared-root.** This capture's `wstg=1 iena=1` IS the
write-stage / ISR-never-armed Signature-A fingerprint — but `free=0` makes it host-abandonment, so it does NOT prove
the A-B link (the host left; the device-side write path was not the thing that died). A GENUINE in-stream B must show
**`free=1`** (host STILL reading — proving the wedge is device-side, not host-abandonment) together with `wstg=1` and
ideally `wnd` near `STALL_WINDOW_LEN`/2 or higher (the §13.8 alternating signature) or a high consecutive `n`. THAT
combination — `free=1 wstg=1` on a wedge where skirnir is still polling `?` and getting silence — is what proves A and
B are ONE write-stage lost-wake root with two outcomes (recoverable drumbeat vs hard-lock). This capture is a clean
NEGATIVE for that question (host left first), not a confirmation.

**NEXT (two items):**
- **INSTRUMENT FIX — LANDED 2026-06-28 (NOT flashed, NOT committed; bughunter).** `[MSG:RESET <reason>]` is now stashed
  for a one-shot `$I`/`?` replay so the dog identity survives a streaming connect. Implemented in `comms.rs` mirroring
  the `CRASH_REPORT` stash-and-replay, in a SEPARATE `RESET_REPORT` buffer (the reset line is emitted every boot, gated
  only on the capture build, NOT on a valid breadcrumb — folding it into `CRASH_REPORT`, which `maybe_emit_crash_report`
  `set()`s AFTER `send_reset_reason` in `main`, would clobber it and skip a no-breadcrumb reset). Drained at BOTH replay
  sites (`$I` build-info + first `?` status), replayed BEFORE the crash report (the reset line frames any crash). All
  `#[cfg(feature = "capture-reset")]` (production untouched). All FOUR Xtensa configs build clean under `-D warnings`
  (explicit `RUSTFLAGS="-D warnings -C link-arg=-Tlinkall.x"` — `just build` is a plain `cargo build`, does NOT enforce
  `-D warnings`); `cargo test -p firmware-core` green (324). No pure-logic to TDD (a format string + stash/replay
  identical in shape to the already-exercised crash-report path). Effect: on the NEXT capture, a B breadcrumb's boot
  replay carries `[MSG:RESET <reason>]` — `*-rtc-WDT` (RWDT) vs `super-WDT` (SuperWDT) vs `*-sw-reset` — answering
  §17.8's "was the RWDT suppressed?". READY TO REFLASH for pass 3+.
- **Keep hunting the free=1 in-stream B** (pass 2 in flight on the PRE-fix image; pass 3+ on the reflashed replay-fix
  image). On a B that lands: read the dog from the now-stashed `[MSG:RESET]`, and `free/wstg/wnd/n` from `usbtx` for the
  A-shared-root verdict.

### 17.14 PROVOKE-AND-CAPTURE BUILD (`provoke-b` feature) — re-enable B at its native rate (LANDED, not flashed)
After 3 full clean Pikachu passes on the `capture-reset` image (the `[MSG:RESET]` replay confirmed live —
baseline shows `[MSG:RESET other]`), the genuine in-stream B did NOT recur (it reproduced ~line 2991 on an EARLIER
build). Blind passes are low-yield. Built a diagnostic variant that lets B reproduce at its pre-fix rate.

**PREMISE — validated, NOT assumed (bughunter).** The hypothesis "TIER 1 is now MASKING B" is mechanistically sound:
the original B (`cap_p3`, §17.8) was a write-stage lost wake (`wstg=1`) on a 4-byte `ok` (`rlen=4`) with the host
still reading (`free=1`) — which is EXACTLY the input `classify_write_stage` now RECOVERS
(`write_timed_out && resp_len<=64 && data_free → CompletedLostWakeRecovered`). So TIER 1 rescues precisely the wedge B
was made of; on the pre-TIER-1 build that wake was NOT recovered and cascaded (§13.8 alternating recovered/genuine
stalls → neither K=3 nor the old dead-zone → hard lock). Disabling that recovery re-creates the pre-fix cascade.

**THE LEVER (minimal, production untouched).** New pure host-tested sibling `firmware_core::diag::WriteOutcome::
classify_write_stage_no_recover` — byte-identical to `classify_write_stage` EXCEPT the single-chunk recoverable branch
returns `Stalled` instead of `CompletedLostWakeRecovered`. The firmware swaps to it via a single
`#[cfg(feature = "provoke-b")]` at the `usb_tx` write-stage classify call (comms.rs ~1638); the production
`classify_write_stage` and the DEFAULT build are byte-for-byte unchanged. The K-escape + survivable watchdog +
`[MSG:RESET]` replay all stay ON, so whichever way B manifests leaves a trace. `provoke-b = ["capture-reset"]` in
`firmware/Cargo.toml` (verified via `cargo tree`: `--features provoke-b` transitively arms `capture-reset`) — provoking
B without the capture channel would be pointless + unsafe, so the feature pulls it in. **BUILD+FLASH COMBO:
`--features provoke-b`** (no need to also pass `capture-reset`; it is pulled in).

**DISCRIMINATING PREDICTIONS (pre-registered so the result is interpreted, not rationalized):**
- **If `provoke-b` reproduces B at a HIGH rate** (≫ the ~1-in-3 baseline, ideally most passes): strong evidence TIER 1
  was MASKING the common B (H1). Combined with the breadcrumb showing `free=1 wstg=1` (device-side write-stage lost
  wake, host still reading) ⇒ **A and B are ONE write-stage lost-wake root**, two outcomes (recoverable drumbeat vs
  hard-lock); TIER 1 is the durable fix, layer 2 the safety net. Read `[MSG:RESET <dog>]` for the §17.8 RWDT-suppressed
  answer.
- **If `provoke-b` STILL does not reproduce B** (clean passes at the same rate as the fixed build): H1 is WRONG —
  B is NOT the single-chunk write-stage lost wake (disabling its recovery changed nothing), so B is an INDEPENDENT
  mechanism and the 3 clean fixed-build passes were just non-determinism. That redirects the hunt entirely (B is not
  A's root). A valuable negative.
- **If `provoke-b` reproduces a wedge with `free=0`** (host-not-reading) rather than `free=1`: that is NOT the genuine
  in-stream B — it is the K-escape/host-abandonment class re-exposed by removing recovery, not the §13.8 hard lock.
  Discriminate by `free=`.

**STATUS:** LANDED, NOT flashed, NOT committed. firmware-core host tests green (326, +2 provoke tests:
`provoke_no_recover_stalls_the_single_chunk_lost_wake_the_production_path_recovers`,
`provoke_no_recover_matches_production_on_every_non_recoverable_input`). All Xtensa configs (default / capture-reset /
provoke-b / defmt+provoke-b) build clean under explicit `RUSTFLAGS="-D warnings -C link-arg=-Tlinkall.x"`. Files:
`firmware-core/src/diag.rs` (+`classify_write_stage_no_recover` + 2 tests), `firmware/src/comms.rs` (cfg swap at the
write-stage classify), `firmware/Cargo.toml` (`provoke-b = ["capture-reset"]`).

### 17.12 GPIO18 scope heartbeat — an out-of-band "is the chip alive" probe (firmware-engineer, 2026-07-11)

The breadcrumb/`[MSG:RESET]` post-mortems tell us WHY the last reset fired, but only AFTER the WDT bites and the board
reboots. For live bench triage of a Signature-B hard lock we want a real-time, toolpath-independent "is the chip still
executing anything" signal on the scope. Watching a STEP line does not give this — STEP is idle between/within moves by
design, so a flat STEP line is ambiguous (idle vs wedged).

**The signal.** The survivable-watchdog TIMG1 ISR (`firmware/src/survivable_watchdog.rs`, `capture-reset`-gated) now
ALSO toggles **GPIO18** once per fire, BEFORE its feed/withhold branch — a FREE-RUNNING ~2 Hz square wave (250 ms
half-period). Because this ISR is a hardware timer that survives a core-0 executor stall (that is its whole reason to
exist), the square wave keeps running through the comms/motion wedges the STEP line cannot disambiguate. It stops ONLY
when the chip is so hard-locked that even this ISR cannot run, or at the eventual WDT reset. A flat GPIO18 is therefore
the "chip is fundamentally dead" signal; a still-toggling GPIO18 with a dead stream says the wedge is above the ISR
(a stuck task / lost wake), not a total CPU lock. The toggle is a raw-PAC `out_w1ts`/`out_w1tc` write (GPIO18 `< 32`),
touching only the GPIO output register — zero RMT / core-1 / step-timing impact.

**Pin note.** GPIO18 is the DOC-00 spare RMT ch3 (provisional 4th axis). In the `capture-reset` build `main` binds that
never-driven channel to `NoPin` and hands GPIO18 to the heartbeat instead, so there is no contention; production builds
are unchanged (GPIO18 → ch3). Probe GPIO18 on the carrier header (pin 11).

**Scope usage.** Enable with `--features capture-reset` (or `provoke-b`, which pulls it in). On the DHO804 bench tool
(`tools/scope`), watch GPIO18 with the `catch-lockup` recipe at a ~600 ms Timeout trigger: healthy = a steady ~2 Hz
square wave; a Timeout capture (no edge for >600 ms) fires exactly at a true hard wedge or the reset, catching the lock
the moment the chip stops — independent of what the toolpath was doing.

### 17.15 FRESH REPRODUCED B (2026-07-11) — heartbeat ALIVE through the wedge, board NEVER reset (bughunter)

**Reproduced on the `capture-reset` heartbeat build:** streaming `128-Pikachu.tap` wedged at line 1583/4474 (WPos
268.4, 88.0, Z-5). Abrupt SILENT mid-stream cut (clean `ok`s right up to it, then skirnir `IoDisconnect: controller not
responding`, exit 4). NO ALARM, NO reset banner. Measured LIVE on the scope during the wedge: **CH3 GPIO18 heartbeat =
~2 Hz TOGGLING, CH1/CH2 X/Y-step = FLAT.** So: motion dead, USB comms dead, but the core-0 TIMG1 hardware ISR is STILL
being serviced (it toggles the heartbeat AND therefore evaluates `watchdog_decision` every 250 ms). The board sat wedged
for MINUTES; port stayed enumerated (no re-enum); an `espflash reset` (EN) booted it clean with NO withhold breadcrumb.

**The question:** the ISR ran the whole time, yet the dogs never bit. Two hypotheses:
- **(a) the withhold decision NEVER fired** — every 250 ms `watchdog_decision` returned `None`, so the ISR kept FEEDING
  both dogs → no reset.
- **(b) the withhold DID fire but the RWDT/SuperWDT did not reset** — the §17.8 "RWDT mysteriously did not fire" mystery.

**CODE-ANALYSIS FINDING — (a) is strongly predicted; here is the exact mechanism.** `watchdog_decision`
(`firmware-core/src/diag.rs`) has exactly THREE withhold conditions, and at this wedge ALL THREE are gated OFF:

1. **core-1 motion withhold** needs `core1_frozen_ticks >= 16` which requires `EXECUTOR_RUNNING` (a block in flight).
   Core 1 is INDEPENDENT of core 0: when core 0 stalls it can no longer refill the `BlockQueue`, so core 1 drains its
   queued blocks and PARKS (`EXECUTOR_RUNNING=false`) within a second or two — matching the FLAT X/Y step lines. A
   parked executor resets `CORE1_FROZEN` to 0, so this detector correctly does NOT fire (core 1 is idle, not wedged).
2. **core-0 comms withhold** needs `comms_frozen_ticks >= 12` which requires `host_active` (RX advanced within
   `RX_ACTIVE_TICKS=24` ⇒ 6 s). The host detects the disconnect and STOPS sending → `RX_ACTIVITY` freezes → `host_active`
   ages out ~6 s after the wedge → `COMMS_FROZEN` is reset to 0. The comms withhold is BLIND to a wedge that coincides
   with / precedes the host going quiet (the reset-loop guard, working as designed but structurally blind here).
3. **dead-zone backstop** needs `response_depth > 0 && tx_complete_frozen_ticks >= 16`. The `tx_complete_frozen` side is
   satisfied within 4 s (a full executor stall freezes `USB_TX_COMPLETED`). The BLOCKER is `response_depth =
   RESPONSE.len()`: `usb_tx` dequeues its response with `RESPONSE.receive().await` into a LOCAL frame BEFORE parking in
   the write await (`comms.rs:1579`), and with the executor stalled + the host quiet NOTHING refills the channel, so
   `RESPONSE.len()` is FROZEN at its wedge-instant depth — which for a writer that was keeping up (RESPONSE near-empty)
   is **0**. `dead_zone_withhold(0, …)` is `false` by its false-trip guard. So the one ungated backstop is structurally
   blind to exactly this wedge.

⇒ `watchdog_decision` returns `withhold_reason = None` every fire → the ISR feeds both dogs forever → no reset. This
also PREDICTS the observed non-determinism (§17.7 self-reset-with-breadcrumb vs §17.13-style hard-lock): whether the
board self-resets depends purely on whether `RESPONSE.len()` happened to be `> 0` at the freeze instant.

**Why this is a FULL core-0 executor stall, not merely `usb_tx` parked.** If the async executor were alive with only
`usb_tx` parked, `usb_tx`'s own 2 s `with_timeout` would fire → TIER 1 recovers the single-chunk lost wake in place →
`USB_TX_COMPLETED` bumps → comms flows again. The observed HARD lock (no acks for minutes, reconnect fails) means the
2 s timeout is NOT firing ⇒ embassy-time is not being serviced for that future ⇒ the whole core-0 thread-mode executor
is stalled. The hardware TIMG1 ISR (heartbeat) runs regardless — which is exactly why it is a survivable feed, and
exactly why it keeps FEEDING the dogs through the stall.

**(b) is not dismissed but has no live support.** The §17.8 ">60 s RWDT did not fire" was on the PRODUCTION async
feeder, where the dead-zone is `DEAD_ZONE_BACKSTOP_ARMED = false` (comms.rs:3278) — i.e. the feeder KEPT FEEDING for the
same gating reason (a). RWDT resets ARE observed on this board (`CpuRtcWdt` reset reason), and both dogs are armed in
this build (main.rs:449-460), so an unfed RWDT should bite in ≤8 s — the minutes-long wedge is far more consistent with
"still being fed" than "unfed but silicon won't reset."

**INSTRUMENT (built + compiles, `capture-reset`-gated) — decisive (a)-vs-(b) probe on GPIO17 / scope CH4.** The TIMG1
ISR now drives **GPIO17** as a LIVE mirror of `decision.withhold_reason.is_some()` each fire (raw `out_w1ts`/`out_w1tc`,
same discipline as the GPIO18 heartbeat; zero step-timing impact). `survivable_watchdog::start` now takes GPIO17
alongside GPIO18. Predictions on a re-reproduced wedge:
- **CH4 stays LOW the entire wedge ⇒ hypothesis (a) confirmed:** the withhold decision never fired; the fix is to add an
  ungated core-0 executor-liveness detector (an embassy task bumps an `EXECUTOR_ALIVE` beat every tick; the ISR
  withholds if that beat freezes for N ticks while the hardware ISR keeps firing) — this catches ANY full-executor
  stall regardless of host/response/motion state, which the three current detectors structurally cannot.
- **CH4 goes HIGH yet the board still does not reset ⇒ hypothesis (b):** the withhold fired but the dogs did not bite —
  pivot to the raw-PAC RWDT/Swd feed/withhold correctness (is Stage0's action actually `reset`, is the write-protect
  key right, does `Swd::enable` actually leave a biting dog).

Bench recipe: `just flash --features capture-reset`; scope CH4 → GPIO17 (carrier header), same 10x/2V setup as CH3;
`skirnir --cli /dev/cu.usbmodem31101 128-Pikachu.tap`; watch CH4 vs CH3 through the wedge.

### 17.16 RTC memory is WIPED by an EN reset — both breadcrumb-via-EN paths are UNSOUND (bughunter, 2026-07-11)

The first wedge's EN-reset boot dump (`MSG:RESET other`, no withhold line) was floated as (a) evidence. It is NOT.
**On the ESP32-S3, toggling EN/CHIP_PU is treated as a power-on reset that powers down the RTC domain — RTC_FAST
(`#[ram(rtc_fast, persistent)]`) AND any RTC-NOINIT word are both LOST** ([ESP-IDF forum](https://esp32.com/viewtopic.php?t=34596),
[memory-types](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/memory-types.html)). The
breadcrumb is gated by `crash::MAGIC`; an EN reset clears the magic → `Breadcrumb::is_valid` fails → the WHOLE
breadcrumb (withhold word included) is discarded. So "no withhold line after an EN reset" is what you see WHETHER OR NOT
a withhold latched — it cannot distinguish (a) from (b). This kills BOTH probe-free breadcrumb paths (the one-shot word
AND an RTC-NOINIT counter — same RTC power domain, same wipe). Corollary: **any RTC-memory breadcrumb is useless for a
wedge that never self-resets, because reading it requires an EN reset that erases it.** The trace MUST live on a channel
that survives / needs no reset: the GPIO17 live probe, or an actual WDT reset (RTC_FAST survives a WDT reset, just not
EN). RWDT resets ARE observed on this board (`CpuRtcWdt`) so the dog itself is real.

**Retention-safe positive control (built, `force-withhold` feature; §17.16).** To test hypothesis (b) — "a withhold
fires but the dogs don't reset" — WITHOUT the physical probe or a reproduced wedge: the `force-withhold` build makes the
TIMG1 ISR UNCONDITIONALLY withhold after `FORCE_WITHHOLD_AT_FIRES=20` fires (~5 s post-boot), overriding
`watchdog_decision`. Expected on a healthy idle board: steady ~2 Hz GPIO18 heartbeat for ~5 s, then both feeds stop, and
the RWDT resets the board ~13 s after boot. Because a WDT reset (unlike EN) RETAINS RTC_FAST, the boot dump should then
show `MSG:RESET` (rtcwdt) + a withhold breadcrumb — validating the ENTIRE withhold→dog→reset→breadcrumb chain on this
board. Outcomes: a clean self-reset ⇒ (b) REFUTED (dogs bite when withheld) ⇒ the wedge's failure to reset is purely
(a) no-withhold-fired, corroborating the minutes-no-reset inference. NO self-reset after the forced withhold ⇒ (b) is
real (raw-PAC feed/withhold or dog-arm defect). Uses only the existing GPIO18 scope probe + serial. Recipe:
`just flash --features force-withhold`; watch GPIO18 on the scope + the serial monitor for the ~13 s self-reset + boot
dump. (Also note: even absent any of this, the minutes-long non-reset with both dogs armed and RWDT known-functional is
already strong (a); the probe and this positive control just make it airtight and rule out the silicon-(b) corner.)

**FORCE-WITHHOLD RESULT #1 (2026-07-11) — the board self-resets, but via `CoreSw`, NOT a dog — the dog test was
CONFOUNDED.** Boot-loops ~every 13 s (USB re-enumerates); the breadcrumb survives + prints every loop:
`MSG:CRASH dead-zone-silent-lock stage=idle_waiting … wdog=24` / `usbtx: host-not-reading free=0 empty=0 iena=1 wstg=1
… rdepth=2 rlen=126 n=3 …` / **`MSG:RESET core-sw-reset`**. Interpretation, from code:
- `core-sw-reset` decodes to **`CoreSw`** (main.rs:353), which is a `software_reset()` — NOT a dog (a dog prints
  `super-WDT` for `SysSuperWdt` (main.rs:360) or `…rtc-WDT`). The only two `software_reset()` sites are the panic
  handler (main.rs:110 — not hit; no panic breadcrumb) and the **usb_tx K-escape** (`capture_usb_tx_stall_and_reset`,
  comms.rs:1801). The `n=3 host-not-reading free=0` in the dump is the K-escape's own fingerprint ⇒ **the usb_tx
  K-escape software-reset the board.**
- Timing corroborates: `wdog=24` vs the `FORCE_WITHHOLD_AT_FIRES=20` withhold start = reset ~4 fires (~1 s) after the
  withhold — far short of the 8 s RWDT, and the K-escape's 3×2 s fuse (started at boot as the host failed to drain the
  126-byte status report during the messy reconnect) comes due at ~6 s ISR-uptime = exactly `wdog≈24`. So the usb_tx
  K-escape won the race; my forced withhold was incidental and **a dog never got the chance to bite.**
- ⇒ This run VALIDATES the capture→retain-across-CoreSw→print machinery, but does NOT establish that a bare withhold
  makes a dog reset. The silicon-(b) corner ("dogs don't reset when unfed") is therefore NOT closed. This is
  load-bearing: in the REAL executor-stall wedge usb_tx is DEAD and cannot software-reset, so a DOG is the only
  possible resetter — we must prove a dog bites.

**DOG-ISOLATION BUILD (`force-withhold` extended, 2026-07-11; all 3 configs compile clean).** `force-withhold` now
ALSO cfg's out the usb_tx K-escape `software_reset()` (the `handle_usb_tx_wedge` call at comms.rs:1728 is
`#[cfg(not(feature="force-withhold"))]`; usb_tx just resets its stall run and keeps serving). So in this build the ONLY
thing that can reset the board is the survivable-watchdog ISR's WITHHELD dog. DECISIVE outcomes on `just flash
--features force-withhold`:
- **Board self-resets ~1–8 s after the forced withhold with `MSG:RESET super-WDT` (or `…rtc-WDT`) ⇒ a dog DOES bite on
  a withhold.** Combined with the real Pikachu wedge NEVER resetting (minutes, heartbeat alive), this proves the
  withhold decision never fired in the real wedge ⇒ **(a) CONFIRMED, and (b) refuted** — no probe / no user needed.
- **Board does NOT self-reset (heartbeat keeps toggling, no re-enum for ≥20 s) ⇒ the dogs DON'T bite when unfed ⇒ (b)
  is REAL:** the survivable-watchdog can never reset a real (executor-dead) wedge, which reframes the fix (the ISR must
  reset the chip ITSELF — e.g. call `software_reset()` directly after `capture_withhold`, which we KNOW works, the
  CoreSw path just did — rather than relying on the dog silicon).
Either way this closes (a)/(b) from the bench WITHOUT the physical GPIO17 probe. Read `MSG:RESET <reason>` + whether the
GPIO18 heartbeat gaps (a dog reset) vs keeps toggling (no reset).

**DOG-ISOLATION RESULT (2026-07-11) — `MSG:RESET super-WDT`: (b) REFUTED, (a) CONFIRMED.** The dog-isolation build
self-reset via **the SuperWDT** (`SysSuperWdt`), ~2 s after the forced withhold (`wdog=20`→`28`), `n=1` (K-escape gone —
purely the dog). So a withheld dog DOES reset the board; the dog silicon works ⇒ **(b) refuted**. Combined with the real
Pikachu wedge NEVER resetting (minutes, heartbeat alive) ⇒ the withhold decision NEVER fired there ⇒ **(a) CONFIRMED.**
Root cause, closed from the bench with no probe and no user: the three withhold detectors are all structurally gated off
in a full core-0 executor stall (host aged out, motion idle, RESPONSE drained), so the survivable ISR fed both dogs
forever. The SuperWDT (fast, ~seconds) is the effective resetter, not the 8 s RWDT.

### 17.17 ROOT-CAUSE FIX — ungated core-0 executor-liveness detector (bughunter, 2026-07-11; host-tested, builds clean)

The fix adds the ONE detector the three work-driven ones lack: an UNGATED core-0 executor-liveness beat. A dedicated
core-0 async task ([`comms::watchdog_heartbeat`], `capture-reset`) bumps `EXECUTOR_ALIVE` every interval
unconditionally, so it advances on a healthy board regardless of host/motion/response state; a full executor stall
FREEZES it while the survivable TIMG1 ISR keeps firing. The ISR tracks the freeze (ungated except a boot guard: only
accrue once the beat has advanced past its initial 0) and passes it to the pure host-tested
`firmware_core::diag::watchdog_decision`, which withholds both dogs once frozen ≥ `EXECUTOR_STALL_TICKS` (16 × 250 ms =
4 s) → the SuperWDT then resets the board ~2 s later, leaving the `core0-executor-stall` breadcrumb. Precedence:
`Core1Motion > Core0ExecutorStall > Core0Comms > DeadZone`. Threshold 4 s is well above any legitimate core-0 quiesce
during streaming and (Pikachu does no mid-stream flash writes) carries no false-trip risk. Files:
`firmware-core/src/diag.rs` (new `WithholdKind::Core0ExecutorStall`, `EXECUTOR_STALL_TICKS`, two `WatchdogInputs`
fields, folded into `watchdog_decision`; 3 new host tests, 351 firmware-core tests green),
`firmware/src/crash.rs` (`WithholdReason::Core0ExecutorStall` = `4` → label `core0-executor-stall`),
`firmware/src/comms.rs` (`EXECUTOR_ALIVE` beat + bump in `watchdog_heartbeat`),
`firmware/src/survivable_watchdog.rs` (sample + freeze-track + wire). All three Xtensa configs
(default / capture-reset / force-withhold) build clean; **production default UNCHANGED** (the detector is
`capture-reset`-gated — the production async feeder can't use it, it dies in the same stall it would detect).

**END-TO-END VALIDATION (pending on the bench):** `just flash --features capture-reset`, re-stream `128-Pikachu.tap`.
Expected: at the wedge the board now self-resets ~6 s later (4 s detect + ~2 s SuperWDT) with **`MSG:RESET super-WDT`**
and a **`MSG:CRASH core0-executor-stall …`** breadcrumb (was: sat forever, heartbeat alive, no reset). On the scope,
GPIO18 keeps toggling through the wedge + the 4 s withhold, then GAPS at the SuperWDT reset and resumes. Confirm the
reset follows a genuine stream stall (skirnir stops getting acks THEN ~6 s later the reset), not a mid-healthy-stream
false trip.

**OPEN — PRODUCTION recovery is a separate USER decision (option A tension).** This diagnostic fix RESETS on the wedge;
the user's option A forbids silent auto-recovery in production (a mid-cut reset loses gcode / resumes in the wrong
place). But a full executor stall CANNOT run a graceful ALARM+halt (that logic is on the dead executor) — only the
survivable ISR is alive, and its only lever is a reset. The likely production answer: ISR resets → reboot into
`ALARM:11` (require re-home) so the board returns to a safe known state that the host must re-zero, NOT a silent resume
— which honors option A's fail-safe intent. Needs the user's call before wiring the detector into the production feed
path (which itself requires moving production to the survivable-ISR feed, since the async feeder dies in the stall).

### 17.18 Deterministic fix validation — `provoke-executor-stall` (bughunter, 2026-07-11; builds clean)

The `capture-reset` fix build behaved perfectly on the bench — idle-stable, GPIO17 flat-low, and **zero false-trips over
a full 30-min Pikachu stream** (past the first run's wedge point, X330+) — but the non-deterministic wedge did NOT
reproduce, so the fix was not yet seen catching a REAL stall. Rather than grind 15–30-min runs, decompose the fix chain:
**L1** `EXECUTOR_ALIVE` freezes in the real stall → **L2** the ISR detects + withholds → **L3** withhold → SuperWDT
reset. L2 is host-tested; L3 is bench-proven (§17.16 dog-isolation). Only **L1** is unproven-by-observation — and it is
FORCED by the same deduction that established (a): both `usb_tx`'s 2 s `with_timeout` and `watchdog_heartbeat`'s
`Timer::after` depend on embassy-time, so the minutes-long total silence requires that mechanism dead, which freezes
`EXECUTOR_ALIVE` too (a state where `watchdog_heartbeat` keeps advancing while `usb_tx` stays permanently stuck is
self-contradictory — both need the timer).

`provoke-executor-stall` tests L1→L2→L3 as a chain DETERMINISTICALLY: a core-0 task waits ~10 s then enters a
non-yielding busy loop, starving the cooperative core-0 executor (incl. `watchdog_heartbeat`) — the exact
full-executor-stall CLASS the memory names as the likely B root — so `EXECUTOR_ALIVE` genuinely freezes and the ISR's
detector must catch it. Expected ~6 s after the stall onset: **`MSG:RESET super-WDT`** + **`MSG:CRASH
core0-executor-stall …`**, CH4/GPIO17 HIGH at the withhold, CH3 heartbeat gapping at the reset. This exercises the
REAL new code path (`EXECUTOR_ALIVE` freeze → `watchdog_decision` → withhold → dog) on a genuine stall, unlike
`force-withhold` (which forces the withhold directly, bypassing `EXECUTOR_ALIVE`). Recipe: `just flash --features
provoke-executor-stall`; the board should self-reset ~16 s after boot and boot-loop. Files: `Cargo.toml`
(`provoke-executor-stall` feature), `comms.rs` (`provoke_executor_stall` task), `main.rs` (spawn). All Xtensa configs
build clean; production default UNCHANGED.

For the NATURAL-wedge catch (belt-and-suspenders), `provoke-b` (§17.14, disables TIER-1 so B cascades at its native
rate — and it pulls in `capture-reset`, so the fix is active) reproduces B far faster than stock Pikachu. Caveat: a
`provoke-b` wedge can be EITHER the usb_tx K-escape variant (executor still alive → `core-sw-reset`, CH4 low — the fix
correctly does NOT fire) OR the full-executor-stall variant (the fix's target → `super-WDT`, CH4 high); the reset reason
+ CH4 tell them apart. Assessment: after `provoke-executor-stall` proves the chain deterministically, the fix is
validated to a high bar; a natural-wedge super-WDT catch is desirable confirmation of L1 but not blocking.

**CAPSTONE PROVEN (2026-07-11).** `provoke-executor-stall` boot dump:
`MSG:CRASH core0-executor-stall stage=idle_waiting comms-stage=consumer-wait-line comms-froze-first beats comms=0
motion=191 wdog=67` / `usbtx: host-not-reading … n=2 …` / **`MSG:RESET super-WDT`**. Scope: CH4/GPIO17 = clean square,
HIGH ~2 s (the withhold→SuperWDT window) then reset; CH3 heartbeat toggling through it; board boot-loops via super-WDT.
Every link confirmed: the `core0-executor-stall` label = the NEW detector fired (not one of the old three);
`comms=0`/`motion=191` = core-0 frozen while core 1 ran (the expected stall signature); `wdog=67` (~16.75 s uptime) =
10 s stall-delay + ~4 s detect + ~2 s SuperWDT. **L1 (EXECUTOR_ALIVE froze) → L2 (detect+withhold) → L3 (super-WDT
reset) proven on a GENUINE executor stall.**

### 17.19 VALIDATION CLOSE-OUT (bughunter, 2026-07-11)

Signature-B "survivable watchdog never reset the wedged board" — ROOT-CAUSED, FIXED, and VALIDATED:
- **Root cause (a) CONFIRMED:** in a full core-0 executor stall the three withhold detectors are all structurally gated
  off (core-1 needs `EXECUTOR_RUNNING`, comms needs `host_active`, dead-zone needs `RESPONSE.len()>0` — all quiescent),
  so the ISR fed both dogs forever. (b) REFUTED: a withheld dog DOES reset (super-WDT, §17.16 dog-isolation).
- **Fix (§17.17):** an ungated core-0 executor-liveness detector (`EXECUTOR_ALIVE` beat + `WithholdKind::
  Core0ExecutorStall`). Host-tested (351 firmware-core tests), false-trip-safe (idle + 30-min real stream, GPIO17
  flat-low), and proven to fire on a genuine stall (§17.18 capstone). Production default UNCHANGED (capture-reset-gated).
- **Instruments left in the tree (all capture-reset-gated, production-inert):** GPIO18 heartbeat, GPIO17 withhold
  mirror, `force-withhold` (dog test), `provoke-executor-stall` (fix capstone), `provoke-b` (fast natural-B repro).
- **PRODUCTION RECOVERY POLICY — DECIDED by the user (2026-07-11): reset → reboot into `ALARM:11` (require re-home).**
  On an executor-stall wedge the survivable ISR resets the chip; the board boots locked in `ALARM:11` so the operator
  must re-home/re-zero — a fail-safe known state, NOT a silent resume that would resume cutting in the wrong place
  (honors option A). Productionization is DEFERRED (safety-critical; the diagnostic tree is uncommitted — the user is
  deciding sequencing: commit the diagnostic work first vs. go straight to production wiring) and will come back to the
  bug-hunter as a PLAN-FIRST ask. Implementation notes for that work: it requires moving the PRODUCTION watchdog feed
  onto the survivable-ISR path (the async `watchdog_feed` dies in the same stall it must detect, so it cannot be the
  production feeder for this class); the `EXECUTOR_ALIVE` beat + the `Core0ExecutorStall` detector then move out of the
  `capture-reset` gate; and the boot path must force `ALARM:11` when the reset reason is the survivable-ISR dog
  (super-WDT) with a `core0-executor-stall` breadcrumb, rather than resuming.
- **Belt-and-suspenders (not blocking):** a natural-Pikachu-wedge super-WDT self-reset (did not reproduce in one 30-min
  fix-build run; `provoke-b` is the fast path if desired).
- All work UNCOMMITTED (diagnostic tree).

## 18. grblHAL / ESP-IDF USB-TX comparison — is the lost-TX-wake PREVENTABLE? (bughunter, 2026-07-11, source-cited)

Research question: does the grblHAL ESP32 / ESP-IDF USB-serial TX path structurally eliminate the lost-TX-done-wake
class we root-caused in `usb_tx` over esp-hal `UsbSerialJtag`, and can we mirror that primitive? **Answer: YES — both
reference designs make the lost-wake structurally impossible, via a RETAINED (data/count) completion signal instead of
esp-hal's single-edge AtomicWaker-on-a-mask-bit. It is preventable, and the fix belongs primarily UPSTREAM in esp-hal
(with a low-effort in-our-code mitigation available now).**

### 18.1 Our esp-hal mechanism and the exact lost-wake vectors (source: esp-hal 1.1.1 `src/usb_serial_jtag.rs`)
`UsbSerialJtagTx::write_async` (line 811) writes each ≤64 B chunk DIRECTLY to the EP1 FIFO, sets `wr_done`, then awaits
a fresh `UsbSerialJtagWriteFuture` per chunk. That future (707–747): `new()` ARMS `int_ena.serial_in_empty`; `poll()`
does `WAKER_TX.register(cx.waker())` then returns `Ready` iff `int_ena.serial_in_empty` is now CLEAR. The single ISR
`async_interrupt_handler` (932) on TX-empty CLEARS `int_ena.serial_in_empty` (the completion latch), clears the raw
flag, and calls `WAKER_TX.wake()`. `WAKER_TX` is ONE shared `AtomicWaker` (703). Three structural fragilities, all
source-confirmed:
1. **No `Drop` on `UsbSerialJtagWriteFuture`** (grep: zero `Drop` impls in the file). When our `with_timeout` fires and
   DROPS the awaited future, `int_ena.serial_in_empty` stays ARMED. The next write re-arms an already-armed bit and the
   arm/event/waker state is desynced — this is the captured Signature-A `iena=1` write-stage lost wake (the interrupt
   armed but never serviced for that write).
2. **Single, latest-only, NON-counting `AtomicWaker`.** It holds only the most recent waker and carries no count. If
   the ISR fires + wakes but the executor's re-poll is lost/raced (our classic `iena=0 empty=0 free=1`: ISR ran, host
   drained, future never completed), there is NO retained state to recover from — the signal is an EDGE, and a dropped
   edge strands the write until the next unrelated event.
3. **Completion = a mask bit, not data occupancy.** "Done" is signalled by the ISR clearing an enable bit; the payload
   was already pushed to the FIFO with nothing retained to re-drive. One missed edge = a stranded write.

### 18.2 Reference design A — ESP-IDF `usb_serial_jtag` driver (SAME peripheral as ours; source: esp-idf `components/esp_driver_usb_serial_jtag/src/usb_serial_jtag.c`)
`usb_serial_jtag_write_bytes()` does NOT touch the FIFO directly — it `xRingbufferSend()`s into a TX RING BUFFER. The
ISR `usb_serial_jtag_isr_handler_default()` on `USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY` `xRingbufferReceiveUpToFromISR()`s
from the ring, `usb_serial_jtag_ll_write_txfifo()`s to refill the FIFO, stashes any leftover in `tx_stash_buf` for the
next IRQ, and keeps SERIAL_IN_EMPTY enabled while ring data remains (disables it only when drained). Completion/backpressure
is a RETAINED binary semaphore: `xSemaphoreGiveFromISR(tx_idle_sem)` when the ring is empty, waited on by
`usb_serial_jtag_wait_tx_done()`. Why it can't lose a wake: the pending bytes live in the RING (retained state) and the
ISR re-fires on every TX-empty until the ring drains — the completion is DATA-OCCUPANCY-driven and self-healing, not a
single edge. The semaphore is a retained/counting resource, not a latest-only waker.

### 18.3 Reference design B — grblHAL ESP32 actual (source: grblHAL/ESP32 `main/usb_serial.c`)
grblHAL on ESP32-S3 uses **TinyUSB CDC** (the native USB-OTG peripheral, NOT the USB-Serial-JTAG). Its `_usb_write()`
is a POLL loop: check `tud_cdc_write_available()`, write what fits, `tinyusb_cdcacm_write_flush()` (2 ms timeout), and if
the FIFO is full call `hal.stream_blocking_callback()` to YIELD, then re-poll. There is NO TX-empty interrupt handler
for the data path (the only ISR, `hw_cdc_reset_handler`, is bus-reset-only). Why it can't lose a wake: there is NO
event-wake at all — completion is re-read from `tud_cdc_write_available()` every iteration. A poll design is immune to
the lost-wake class by construction. (RX buffer advertised 512 B; TinyUSB staging 64 B.)

### 18.4 The translatable primitive + recommendation
The common prevention principle across A and B: **the TX-complete/room signal must be RETAINED (ring occupancy /
counting semaphore) or RE-READ each poll (poll the FIFO-free bit) — never a single edge delivered to a latest-only
waker.** Three ways to bring that into our esp-hal + Embassy stack:
- **Option B (poll, mirror grblHAL) — LOW effort, in OUR code, recommended NOW.** Make `usb_tx` POLL-based: write ≤64 B
  chunks to the FIFO, then between chunks re-read `ep1_conf.serial_in_ep_data_free` with a yield
  (`embassy_futures::yield_now().await`, or a short `Timer`), instead of awaiting `UsbSerialJtagWriteFuture`. This is
  EXACTLY our validated TIER-1 poll-after-arm recovery PROMOTED from a 2 s backstop to the primary loop — no waker
  exists to lose. Cost: slightly higher poll wakeups (negligible at CNC TX rates; bounded by yielding). Stays entirely
  in `firmware`, no upstream dependency, low risk. This makes the lost-wake structurally impossible for us.
- **Option A (ring-buffer ISR, mirror IDF) — the "correct" root fix, HIGH effort, belongs UPSTREAM in esp-hal.** A
  SERIAL_IN_EMPTY-ISR-drained TX ring with the ISR re-arming while data remains and an embassy-sync retained completion
  (a `Channel`/counting signal, not a bare `AtomicWaker`). This is reimplementing the IDF driver in Rust; it is a genuine
  esp-hal `UsbSerialJtagTx` deficiency and should be filed/fixed upstream rather than forked into our tree.
- **Option C (minimal esp-hal patch) — cheapest upstream fix.** Add a `Drop` to `UsbSerialJtagWriteFuture` that disarms
  `int_ena.serial_in_empty` (closes our `iena=1` with_timeout-drop vector), and make `poll()` ALSO return `Ready` on the
  hardware `serial_in_ep_data_free` bit (data-driven, closes the latest-only-waker edge loss). Small, upstreamable, and
  it fixes the class for every esp-hal user — but until it lands, it is not ours to rely on.

**Verdict:** the lost-wake is a genuine esp-hal `UsbSerialJtagTx` design defect (edge-signalled completion + no `Drop`),
NOT inherent to Embassy or our code. It IS preventable. Recommended path: **adopt Option B now** (poll-based `usb_tx` —
makes the wedge impossible for us, low risk, reuses proven logic) and **file Option C upstream** to esp-hal as the
durable ecosystem fix; keep the §17.17 executor-liveness reset→ALARM:11 as defense-in-depth. This would make the ROOT
lost-USB-TX-wake impossible rather than merely recoverable — the prize the user asked about. PLAN-FIRST before any code.

### 18.5 Streaming-contract cross-check (light)
From `main/usb_serial.c`: grblHAL advertises a 512 B RX buffer; ours advertises 1024 B (larger — fine). Nothing in the
TX-path review contradicts our ok/error, single-CRLF, post-error-hold, or realtime-byte-interception contract (those
live in grblHAL `protocol.c`/`grbllib`, not the serial driver, and were not re-read here). A deeper protocol cross-check
against grblHAL `protocol.c` is a separate focused pass if the user wants it — flag NONE from the driver layer.

## 19. Option B — poll-based `usb_tx` (root prevention) — IMPLEMENTED, host-tested, awaiting BENCH (bughunter, 2026-07-11)

User approved Option B (§18.4): replace `usb_tx`'s waker-based write await with a hardware-poll loop, making the
lost-TX-wake structurally impossible (no waker to lose). This is a PRODUCTION comms-path change on the sacred streaming
path. **STATUS: implemented on the working tree; 342 firmware-core tests green under `-D warnings`; all four Xtensa
configs (default / capture-reset / provoke-b / provoke-executor-stall) build clean under `-D warnings`; NOT flashed
(handed to the bench). Two refinements vs the plan sketch below, both flagged to the team:**
- **esp-hal-exact commit sequencing (§19.6 sacred-path safety):** `write_response_polled` commits each ≤64 B packet
  with `flush_tx_nb` (sets `wr_done` ONCE) then POLLS esp-hal's own `ep1_conf & 0b011 != 0` "commit registered"
  condition (yielding), rather than relying on the next `write_byte_nb` to back-pressure. This mirrors esp-hal's tested
  blocking `flush_tx`/`write` exactly and removes any dependence on unverified FIFO-buffering assumptions / a
  commit-in-progress race on a partial final chunk. Cost: a per-packet commit-registration poll (~1 USB frame ≈ ms when
  the host is draining), cooperative (yields), consistent with the chosen `yield_now`.
- **`provoke-b` degenerates:** its only mechanism was the `classify_write_stage_no_recover` swap on the AWAIT path,
  which no longer exists — so `provoke-b` now compiles as `= capture-reset` (instruments armed, no unique provocation).
  It cannot "provoke" the lost wake because the lost-wake path is GONE; running it just watches the poll path with full
  capture instrumentation. (If an A/B control that still wedges is wanted, the OLD await path would have to be kept
  under a separate cfg — more retained code; recommend NOT.) The recovered-counter apparatus
  (`USB_TX_LOST_WAKE_RECOVERED`, `record_recovered_count`, the `$I rec=` line, its RTC_FAST slot) is left DORMANT
  (reads 0 — a "no lost-wakes" indicator) to avoid perturbing the breadcrumb layout on this change; removed with
  `provoke-b` in the retirement cleanup.

### 19.1 The primitive (esp-hal exposes exactly what we need on the Async TX)
`UsbSerialJtagTx<'_, Dm>` (any `Dm`, incl. `Async`) exposes non-blocking, WAKER-FREE methods that re-read the hardware
each call (esp-hal 1.1.1 `usb_serial_jtag.rs`):
- `write_byte_nb(b) -> nb::Result<(), _>` (193): writes `b` to the EP1 FIFO IFF `ep1_conf.serial_in_ep_data_free` is
  set, else `WouldBlock`. Re-reads the FIFO-room bit every call.
- `flush_tx_nb() -> nb::Result<(), _>` (223): sets `wr_done`, returns `Ok` iff the packet was accepted
  (`ep1_conf & 0b011 != 0`), else `WouldBlock`.
Neither touches `int_ena` or `WAKER_TX` / `UsbSerialJtagWriteFuture` — the entire lost-wake surface (§18.1) is bypassed.

### 19.2 Before / after control flow (exact)
BEFORE (per response, comms.rs `usb_tx`): `RESPONSE.receive().await` → `with_timeout(2s, tx.write_all(bytes))` →
on timeout re-read `serial_in_ep_data_free` → `classify_write_stage` (TIER-1 single-chunk widening) → if clean,
`with_timeout(2s, tx.flush())` → `classify_split` → `WriteOutcome` {Completed | CompletedLostWakeRecovered | Stalled}
→ K-escape counter. The `with_timeout` drop is what strands `int_ena` (§18.1 vector 1).

AFTER (per response): `RESPONSE.receive().await` → poll-write the bytes, NEVER awaiting the esp-hal write future:
```
outcome = write_response_polled(&mut tx, resp.as_bytes(), Instant::now() + POLL_STALL_TIMEOUT):
  for chunk in bytes.chunks(64):
    for &b in chunk:
      loop:
        match tx.write_byte_nb(b):
          Ok(())               => break               // byte in FIFO
          Err(WouldBlock)      =>                      // FIFO full = host not draining yet
            if Instant::now() >= deadline: return Stalled
            yield_now().await                          // re-poll next executor tick — NO waker
    loop:                                              // push packet + confirm host accepted it
      match tx.flush_tx_nb():
        Ok(())          => break
        Err(WouldBlock) =>
          if Instant::now() >= deadline: return Stalled
          yield_now().await
  return Completed
```
Completion is re-read from `serial_in_ep_data_free` each poll; `yield_now()` re-polls unconditionally next tick, so the
loop makes progress every executor cycle with ZERO dependency on the TX interrupt. A genuine host-not-reading host makes
`data_free` never set → the bounded `POLL_STALL_TIMEOUT` (keep 2 s, matching today) returns `Stalled` → drop-and-continue
+ the existing K-escape, exactly as before. Flow-control integrity (CORRECTED from the sketch's over-broad "one unit"
claim): the flow-control-critical `ok`/`error:N` are ≤64 B = a SINGLE packet, so a stall sends the whole line or none
(the first `write_byte_nb` blocks on a full FIFO before any byte commits) — they are never truncated. A mid-response
stall on a MULTI-chunk response (a long `<...>` status / `$`-report) can strand its already-committed packets on the
wire (a truncated tail), but those are NOT part of the character-count flow control (status is re-requested on the next
`?`), and this is exactly the prior await path's behavior (a `write_all` timeout also stranded mid-response) — no
regression.

### 19.3 Relationship to the existing recovery tiers — what stays / goes / simplifies
- **REMOVED (loses its purpose):** the entire lost-wake RECOVERY machinery — `WriteOutcome::CompletedLostWakeRecovered`,
  `classify_write_stage`, `classify_split`, `classify_write_stage_no_recover`, the TIER-1 single-chunk widening, the
  `USB_TX_LOST_WAKE_RECOVERED` counter, and the `SINGLE_CHUNK_MAX_BYTES` reasoning. All of it existed ONLY to
  disambiguate "the bytes went out but the wake was lost" from "a real stall" on the await path. With polling there is
  no lost wake to recover: `data_free` set ⇒ proceed, not set ⇒ genuine stall. `WriteOutcome` collapses to
  {Completed, Stalled}.
- **RETURN mostly unchanged (a REAL condition, not a lost-wake artifact):** the K-escape `UsbTxStallCounter` +
  `handle_usb_tx_wedge` — a genuine host-not-reading stall (data_free never sets for K consecutive responses) still
  drops-and-continues and, at K, raises production `ALARM:17` / captures in the diagnostic build. It now fires ONLY on
  a true non-draining host, never on a phantom lost-wake.
- **UNCHANGED, coexists (defense-in-depth):** the §17.17 executor-liveness watchdog (reset→ALARM:11). It catches a FULL
  core-0 executor stall from ANY cause, independent of the usb_tx path. Do NOT entangle the two changes — Option B ships
  as its own commit; the executor-liveness productionization is a separate follow-on.
- **RETIRE after Option B lands:** the `provoke-b` feature — its whole purpose (provoke the lost-wake B by disabling
  TIER-1 recovery) is moot once the lost-wake path is gone. Keep it only through the validation window (see §19.5),
  then remove.

### 19.4 Productionization scope + throughput/timing
- **PRODUCTION always-on, NOT capture-reset-gated.** Option B is the root prevention; it replaces the fragile await for
  every build. It is a production streaming-path change.
- **Throughput (healthy host):** unchanged. When the host reads promptly `data_free` is set almost immediately, so
  `write_byte_nb` rarely `WouldBlock`s and the poll loop barely yields — same effective latency as the await path, which
  also completed in sub-ms when healthy. No change to the ok/status/ack cadence (usb_tx is still the single writer;
  relative ordering is identical).
- **Throughput (back-pressured host):** the loop cooperatively `yield_now()`s and re-polls each executor tick until
  `data_free` — a brief cooperative spin (ms, until the host catches up), yielding to status/consumer each iteration so
  nothing is starved. Costs a few extra executor cycles during transient backpressure, NOT throughput. If profiling
  ever shows a spin spike, bound it with a `Timer::after(~200 µs)` between polls (adds ≤200 µs TX latency, negligible);
  default is `yield_now` for lowest latency.

### 19.5 Testability + bench validation
- **Pure host-testable helper:** factor the per-poll decision into `firmware_core::diag`, e.g.
  `usb_tx_poll_action(data_free: bool, deadline_exceeded: bool) -> PollAction {WriteOrFlush, Yield, Stall}`, plus a thin
  `PollWriteOutcome` reducer over a sequence of `(data_free, elapsed)` observations → {Completed, Stalled}. Host tests:
  data_free-always-true ⇒ Completed with N writes; data_free-always-false ⇒ Stalled at the deadline; intermittent ⇒
  Completed. Same pure-decision / firmware-does-I/O split as `watchdog_decision`. The byte cursor + `nb` calls stay in
  comms.rs (I/O, compile-checked).
- **Bench validation** (absence-of-wedge is structural, but we get positive signals):
  1. `provoke-b` should NO LONGER reproduce Signature B at all (there is no lost-wake path left to provoke) — a strong
     positive test: run the same stream that reproduced B and confirm it now completes clean.
  2. Full 30-min Pikachu stream completes with unchanged throughput; monitor core-0 CPU for a spin regression.
  3. Host-not-reading test (pause the reader mid-stream) still cleanly drops → recovers → resumes (the genuine-stall
     path preserved).
  4. `USB_TX_LOST_WAKE_RECOVERED` (if kept transitionally) reads 0 — the class is gone, not merely quiet.

### 19.6 Risk + rollback
- **Risk: byte drop/dup on the sacred wire** (would desync ok/error flow-control). Mitigation: the pure state machine +
  host tests; the byte cursor advances ONLY on `Ok`; `wr_done`/`data_free` semantics copied from esp-hal's own blocking
  `write`/`flush_tx`. Catch: the grblHAL char-counting host (skirnir) desyncs loudly if an ok is dropped/duplicated.
- **Risk: back-pressure spin starves core 0.** Mitigation: `yield_now` yields each iteration; optional `Timer` bound;
  validate CPU over the 30-min stream.
- **Risk: changed drop-on-stall behavior.** It is the SAME drop-and-continue the 2 s timeout does today, so host
  flow-control behavior is preserved.
- **Rollback:** self-contained to `usb_tx`'s write section + the removed classify helpers — one revertible commit. Keep
  the §17.17 executor-liveness net armed as the backstop during rollout.

### 19.7 Files touched + rough size
- `crates/firmware/src/comms.rs`: rewrite `usb_tx`'s write/flush/classify section (~80 lines changed); remove the
  TIER-1 recovery branches.
- `crates/firmware-core/src/diag.rs`: ADD `usb_tx_poll_action` + `PollWriteOutcome` + tests (~80 lines); REMOVE
  `CompletedLostWakeRecovered`, `classify_write_stage[_no_recover]`, `classify_split`, `SINGLE_CHUNK_MAX_BYTES` and their
  tests (~150 lines) — net SIMPLIFICATION.
- `crates/firmware/Cargo.toml` + call sites: retire `provoke-b` after validation.
- Net: roughly neutral-to-smaller line count, large fragility reduction. Estimated diff ~250–350 lines touched, mostly
  deletions.

**DECISIONS FOR THE USER:** (1) `yield_now` vs a `Timer`-bounded poll (recommend `yield_now`, Timer only if profiling
demands). (2) Remove the TIER-1 machinery now vs keep it dormant one release (recommend REMOVE — dead code on the sacred
path is a liability, and the pure helper's tests cover the new path). (3) Retire `provoke-b` after the validation window
(recommend yes). Nothing is written until you approve.
