# Streaming Lockup Investigation (ESP32-S3 firmware)

**Status: OPEN.** Root cause not fully resolved. Two of three observed failure modes are recovered/diagnosable;
the third (a hard, silent, non-recovering wedge) is still being chased. This document is the running record so
the investigation can be resumed cold.

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
- **The combined fix is WIRED, build-verified on both Xtensa configs, then REVERTED OUT** so the tree stays at
  capture-only #1b for capture #3 (0 fix refs in `comms.rs`; the pure `WriteOutcome` logic stays as inert
  host-tested dead-code, 20 diag tests, not in the #1b binary). It is ONE coherent `usb_tx` diff:
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
