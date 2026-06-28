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
