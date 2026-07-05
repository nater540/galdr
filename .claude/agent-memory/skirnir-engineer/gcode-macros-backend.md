---
name: gcode-macros-backend
description: GCode macros backend — model, {var} substitution, and the queued-but-manual RunMacro execution path (why it does NOT reuse the program queue)
metadata:
  type: project
---

GCode macros (ioSender-style saved snippets, run from buttons) backend, built test-first. UI (Macros dock tab +
editor modal) is owned by the skirnir-ui-designer agent; this note is the backend only.

**Model/persistence** — `profile::GcodeMacro { name, body, #[serde(default) ]confirm }` + `Prefs.macros:
Vec<GcodeMacro>` (`#[serde(default)]`, no `PROFILE_VERSION` bump) + `profile::MACRO_LIMIT = 16`. Watch:
`shell.rs::snapshot_prefs` rebuilds the WHOLE `Prefs` from `UiState`, so it must `std::mem::take` and re-insert
`macros` or a save wipes them (macros live only on the profile, not `UiState`).

**Substitution** — pure `app/macros.rs`: `expand(body, &MacroContext) -> Result<Vec<String>, MacroError>`.
`MacroContext::from_view(&ViewState)` reads `dro()` / `overrides()` / `current_wcs`. Vars: `{wx/wy/wz}` `{mx/my/mz}`
(3-dp, matches `work_offset_line`), `{wcs}` (e.g. G54), `{fo}` `{so}` (percent). `{{`/`}}` = literal brace. Unknown
OR not-yet-available var = hard error (never send a fabricated coord / wrong WCS); blank lines skipped; error line
numbers 1-based over the original body. `{wcs}` needed new plumbing: `protocol::Wcs` enum + `ParserState.wcs` parsed
from the `$G`/`[GC:]` line, folded into `ViewState.current_wcs` (does NOT gate the tool-based console suppression;
cleared on disconnect). See [[tool-change-state]].

**Execution — the load-bearing decision:** macros must NOT go through `on_send_line` (it DROPS a manual line with a
fault if it doesn't fit the window — silently loses later lines of a multi-line macro). Instead
`ProtocolCore::on_run_macro` enqueues into a SEPARATE `macro_queue` (NOT the `program` VecDeque — reusing it would
tag lines `Program`, advancing progress + arming the error-hold) that shares the SAME `FlowWindow` and the driver's
drain / backpressure / `AbortQueued` teardown. In-flight macro lines are tagged the new `InflightKind::Macro`
(alongside Program/Other). A macro deliberately stays in whatever state it started (Idle — it does NOT enter
Streaming) and emits NO `Progress` — "a macro is not the loaded job", so the toolpath/Program tab is untouched.
Completion = `Effect::MacroFinished{ok}` → `Event::MacroFinished{ok}` once the last macro line acks and the queue
drains (the only re-enable signal, since there's no Progress). A macro line `error:N` stops the rest (grbl
error-hold would reject them) and reports `ok:false` WITHOUT entering the Error lifecycle. `abort_macro` is wired
into every abort path (SoftReset/ProgramStop/Banner/Alarm → `MacroFinished{ok:false}`; disconnect clears silently).
`Command::RunMacro(Arc<[String]>)` mirrors `StreamProgram`. Reducer tracks `ViewState.macro_running` (armed by
`note_macro_started()`, cleared by `MacroFinished`/disconnect) as the UI's gate.

**Intent scaffolding (defined, handling left to UI agent as no-op arms):** `ExecuteMacro(usize)`, `OpenMacroEditor`,
`AddMacro`, `UpdateMacro { index, updated: GcodeMacro }`, `DeleteMacro(usize)`.
