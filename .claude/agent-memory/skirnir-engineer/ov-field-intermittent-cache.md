---
name: ov-field-intermittent-cache
description: grbl Ov: override field is intermittent like WCO:; must be cached in ViewState or sliders/steppers snap to 100%
metadata:
  type: project
---

The grblHAL `<...>` status report emits the `Ov:` (feed,rapid,spindle override %) field ONLY intermittently — on
change / periodic refresh, NOT on every report — exactly like `WCO:`. `docs/gcode-streaming.md` marks it `{|Ov:...}`
(braced = optional) and calls out "change-only/intermittent rules".

**Why this matters:** `StatusReport.overrides` is `Option<(u32,u32,u32)>` parsed per-report, so it is `Some` only on
the rare Ov-bearing report and `None` on every poll in between. Reading it directly with `.unwrap_or((100,100,100))`
makes the override snap to 100% on every Ov-less poll — the override sliders/steppers visibly snapped back to centre
and drifted under rapid input.

**How to apply:** never read the per-report `status.overrides` for UI/stepping. `ViewState` caches the last-seen `Ov:`
in `last_overrides` (mirroring `last_wco`) and exposes `ViewState::overrides() -> (u32,u32,u32)` (neutral-100 fallback
until first `Ov:`, cleared on disconnect). The override panel (`views.rs`) and the shell's `set_override` relative-step
base both read `view.overrides()`. The pure stepping module (`app/overrides.rs` — `OverrideTracker`/`OverrideFeedback`,
relative ±10/±1/reset bytes, host-side hold-against-stale-live) was already correct; the bug was purely the binding
feeding it a per-report `None→100` instead of the cached value. Related: [[settings-and-overrides]].

The shell's per-frame `override_tracker.observe(...)` block deliberately STAYS on the per-report `status.overrides` —
it wants firmware-truth confirmation EVENTS, and an Ov-less report carries no new truth (feeding the cache every frame
would re-confirm continuously). Only the display/stepping-base reads go through the cache.

Test seam: `ui_test.rs::view_with_overrides` now seeds the cache by feeding the `Ov:` body through `view.apply(...)`
(the real reducer path) rather than hand-building `view.status`, so UI tests exercise the cache end-to-end.
