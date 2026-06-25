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

Companion notes: `.claude/agent-memory/firmware-engineer/project-firmware-lockup-investigation.md` (the
firmware-engineer agent's working notes) and the user memory `project-firmware-streaming-lockup`.
