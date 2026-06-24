---
name: tool-change-state
description: grblHAL Tool machine-state + active-tool ($G/[GC:] T<n>) parsing and UI surfacing in skirnir
metadata:
  type: project
---

Skirnir parses and surfaces the grblHAL M6 manual-tool-change `Tool` machine state and the active tool number,
implemented against the firmware's exact wire contract (firmware side in `firmware-core/src/protocol.rs` —
`MachineState::Tool` → `"Tool"` token, and `ResponseWriter::parser_state` emits
`[GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 G49 F0 S0]`).

**Wire contract:**
- `<...>` status: a `Tool` state token, held while an M6 manual tool change awaits cycle-start (`~`) resume.
- `$G` / `[GC:]` parser-state line carries `T<n>` (T0 = none). The `<...>` status report carries NO tool number.
- Resume is the existing cycle-start `~`; on resume the state returns to Run/Idle.

**Tool sourcing — SINGLE source `ViewState::current_tool` (the `$G`/`[GC:]`-reported tool):**
The firmware was updated (PARALLEL work) to answer `$G` (and `$#`) DURING an M0/M1/M6 hold — including a hold
reached mid-stream — emitting `[GC: ... T<n> ...]`. That made the firmware-reported tool the single authoritative
source for BOTH the DRO `TOOL` strip and the tool-change banner; both read `view.current_tool` directly. (An EARLIER
revision derived the tool from the program stream because `$G` was thought blocked during the hold — that
`program_commanded_tool` / `effective_tool` pair was DELETED once the firmware answered in-hold. Don't reintroduce it.)
- The shell nudges `$G` on TWO edges, each once (gated on a badge transition via `SkirnirApp::last_badge`, never
  every frame): (a) becoming-ready (`badge_is_live` false→true) to SEED `current_tool` on connect before any first
  M6; (b) entering `BadgeState::Tool` to refresh the tool to insert. Do NOT gate the nudge on `!streaming` — we WANT
  it during a streaming hold (the firmware answers it).
- Do NOT depend on the firmware's M6 `[MSG:]` tool-name text either.

**BOARD-GATED CAVEAT:** skirnir now depends on the firmware answering `$G` during the hold. That is
hardware-unverified (no board) — tests validate it only against the loopback mock of that behavior.

**`[GC:]` console handling (#4 fix, `view_state.rs::on_response` Message arm):** suppress the console echo ONLY when a
tool was successfully extracted (`parse_gc_body(body).tool == Some(_)`) — the auto-reconcile `$G` would be noise. A
`[GC:]` that yields NO tool (hand-typed `$G`, malformed/garbled line) must NOT be swallowed — it falls through to the
console below so it stays visible-as-text.

**Where it lives in skirnir:**
- `RunState::Tool` already existed in `protocol/status.rs` (parsed from the `<...>` token).
- NEW `protocol/parser_state.rs`: pure `parse_gc_body(body) -> Option<ParserState>` — recognises the `GC:` sub-kind,
  scans space-separated modal words, extracts `T<n>`. `[GC:]` arrives as a `Response::Message` (generic `[...]`),
  so it's folded in `view_state.rs::on_response`'s `Message` arm BEFORE console (skips console noise). An absent `T`
  word leaves the cached tool untouched (does not clobber with None).
- `ViewState::current_tool: Option<u32>` — Some(0)=none vs None=unreported; cleared on disconnect.
- `badge.rs`: NEW `BadgeState::Tool` (was previously folded into host lifecycle). `RunState::Tool → BadgeState::Tool`.
  `TransportGroup` row shares the Hold/Door resume row (`run_is_resume=true`), so `run_or_resume` sends CycleStart
  (`~`) with NO second pathway. Label "TOOL CHANGE", violet `STATE_CHECK` colour (distinct from amber Hold).
- UI: DRO shows a `TOOL` strip (next to WCO) reading `view.current_tool` directly;
  `views::tool_change_banner(ui, view: &ViewState, sink)` renders a prominent violet attention strip in the top
  banner slot (shell.rs, when badge==Tool and no fault banner), naming `view.current_tool`. Resume button →
  `Intent::RunOrResume`. Headline copy is the pure `tool_change_headline(Option<u32>)`. See [[gui-architecture]] for
  the pure-reducer + thin-view split this fits.
