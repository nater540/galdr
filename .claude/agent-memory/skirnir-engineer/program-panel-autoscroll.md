---
name: program-panel-autoscroll
description: How the Program dock auto-scrolls to follow the executing line in a virtualized show_rows list (egui 0.34)
metadata:
  type: project
---

The Program dock tab (`program_body` in `crates/skirnir/src/app/views.rs`) auto-scrolls to follow the executing
line while streaming, distinct from the Console which uses `ScrollArea::stick_to_bottom(state.auto_scroll)`.

**Why follow differs from stick-to-bottom:** the executing line (`view.progress.acked`) is a moving cursor through
the file, not the bottom, so stick-to-bottom is wrong for the program listing.

**How it's implemented (the reusable idiom for virtualized lists here):**
- The list uses `ScrollArea::show_rows`, which *virtualizes* — the active row may not be laid out this frame, so a
  row-local `scroll_to_me` won't fire for an off-screen line. Instead compute the row rect from
  `ui.min_rect().top() + index * row_height` (the content `ui`'s top is the virtual row-0 origin) and call
  `ui.scroll_to_rect(rect, Some(Align::Center))` *inside* the closure. This works regardless of virtualization.
- Pure decision lives in `program_follow_target(auto_scroll, connection, current, program_len, followed)` — no egui
  types, unit-tested. It returns `Some(line)` only when auto-scroll is on, state == Streaming, cursor in range, and
  the line *moved* since last followed.
- `UiState::program_followed_line: Option<usize>` remembers the last followed row so we only scroll on advance — NOT
  every frame — which leaves an operator who scrolled back to read an earlier line undisturbed. Cleared in
  `set_program`. Reuses the shared `state.auto_scroll` toggle (same checkbox that governs the console).

**How to apply:** when adding follow/scroll behaviour to any virtualized `show_rows` list, compute the target rect
from row index × row_height and `scroll_to_rect` inside the closure; gate re-scroll on a "last followed" field so it
doesn't fight manual scrolling. See [[gui-architecture]] for the pure-reducer + thin-view split this fits.
