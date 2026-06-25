---
name: skirnir-stream-time-eta-seam
description: skirnir dock ETA seam — shell stream_time() chooses physics EtaTimeline vs acked-rate; reducer stays free of Instant::now()
metadata:
  type: project
---

The dock's elapsed/ETA clock flows: `SkirnirApp::stream_time()` (shell.rs) → `TimeEstimate` → `views::dock` →
`dock_progress`. The shell owns ALL wall-clock state (`stream_started: Option<Instant>`); the reducer/`progress.rs`
math stays pure (no `Instant::now()`), taking `elapsed` as a parameter. Preserve that boundary.

Two ETA sources, chosen in `stream_time()`:
- **Physics-based** when `SkirnirApp::simulated: Option<crate::eta::EtaTimeline>` is set (Simulate button →
  `Intent::Simulate` → pure host calc, no engine Command). `progress::physics_estimate(elapsed, total_secs,
  remaining_secs)` projects a total from frame 1 (no acks needed). Live `completed_lines` prefers `status.line`
  (`Ln:`) over `progress.acked`; override fractions from `status.overrides` (PERCENT → /100.0). The timeline is
  source-line-indexed (`lines.len() == program.len()`), so line/ack counts map directly.
- **Acked-rate fallback** (`progress::estimate`) when no simulation — unchanged legacy behaviour.

`configs_from_settings(|n| view.settings.value_of(n)…parse::<f64>())` builds planner/motion configs from the live
`$$` snapshot; empty snapshot → firmware defaults + `simulated_default_settings=true` flag, surfaced in the dock via
`views::EtaQualifier` ("(default settings)" + pause count). Simulate clears on `open_program`. See
[[skirnir-test-feature-split]] for how the view-side of this is tested (kittest, gui feature).
