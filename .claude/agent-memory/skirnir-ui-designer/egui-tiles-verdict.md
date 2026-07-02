---
name: egui-tiles-verdict
description: Decision (2026-07-02) to NOT adopt egui_tiles for skirnir's panel layout — rationale + revisit criteria
metadata:
  type: project
---

Evaluated `egui_tiles` (rerun-io, 0.16.0 June 2026, 1.7M downloads, actively maintained; 0.15.0 is the egui-0.34
match; `Behavior::is_tile_draggable`/`is_container_resizable` can lock tiles down) as a replacement for the
hand-rolled panel arrangement, prompted by the resize-fight bug. PASSED on adopting it.

**Why:** its value is USER-REARRANGEABLE tiled workspaces (drag-and-drop docking, tabs, share-based splits) — the
opposite of skirnir's deliberately FIXED machine-control chrome (268|1fr|286 grid, pinned toolbar/status,
design-locked; operators build muscle memory against fixed control placement). Adopting it means disabling its
headline feature via Behavior overrides, keeping the toolbar/status/banner layout hand-rolled anyway (tiles only
manage a central tree), churning every snapshot baseline, and tracking another dependency one release behind our
egui pin. Crucially it would NOT have prevented the reported bug class: the dialog half lived in `egui::Window`
auto-sizing (tiles don't manage windows), and the dock half (panel content-rect feedback) is fixed and pinned by
five stability tests. `views::shell_panels` is now a single small shared layout function — low maintenance.

**How to apply:** revisit if user-rearrangeable/dockable panels become a product goal (pop-out tabs, custom
workspaces) — egui_tiles is the right tool then, proven at scale in rerun; its top-down share-based sizing is
also inherently immune to the content-feedback drift. Related: [[egui-elegance-verdict]], [[egui-034-layout-gotchas]].
