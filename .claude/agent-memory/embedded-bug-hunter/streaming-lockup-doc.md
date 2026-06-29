---
name: streaming-lockup-doc
description: Pointer to the owned streaming-lockup investigation doc and its three failure modes A/B/C
metadata:
  type: reference
---

The canonical investigation record this agent owns is
`docs/streaming-lockup-investigation.md` (append-mostly log).

Three distinct failure modes behind the one "lockup":
- **Mode A** — core-1 RMT ch0 (X) `wait()` hang; breadcrumb `stage=axis0:wait_begin`, no
  wait_done; RMT TX-END never fires. Now timeout-bounded (CCOUNT, not Instant — Instant
  freezes in the non-yielding core-1 InterruptExecutor). Decisive `[MSG:CRASH rmt0:...]`
  line not yet captured on board.
- **Mode B** — soft core-0 comms wedge (RECOVERS via task-watchdog); breadcrumb
  `core0-comms-wedge stage=idle_waiting`.
- **Mode C** — HARD silent wedge, no breadcrumb, EN-only. Leading cause: a PANIC into
  esp-backtrace 0.19.0's `interrupt_free(|| loop {})`; a core-1 panic disables IRQs on
  core 1 only so core 0 keeps feeding RWDT → no recovery. Custom `#[panic_handler]` +
  core-1 stack 16→32 KiB landed 2026-06-25; panic-line capture is the frontier.

Breadcrumb channel: RTC_FAST `#[ram(rtc_fast, persistent)]` atomics, survives
software_reset()/watchdog but NOT EN/power-cycle/brownout/DTR-RTS-toggle. Boot emits
`[MSG:CRASH ...]` over grbl TX, replayed once on first `$I`/`?`. NEVER press EN before
reading the crumb.

Companion: `.claude/agent-memory/firmware-engineer/project-firmware-lockup-investigation.md`,
user memory `project-firmware-streaming-lockup`.
