---
name: skirnir-ui-stack
description: Vetted and pinned egui/eframe/rfd versions for skirnir's GUI, with rationale for feature choices and Linux gotchas
metadata:
  type: project
---

Vetted 2026-06-17. All three crates confirmed on crates.io at these versions.

## Pinned versions

- `eframe = "0.34.3"` — latest stable as of 2026-05-27
- `egui = "0.34.3"` — version-locked 1:1 with eframe; both are from the same monorepo tag
- `rfd = "0.17.2"` — latest stable; tokio feature removed in 0.17.0 (no longer needed)

## MSRV

All three require Rust 1.92. Host stable toolchain is 1.96.0 — satisfied with room to spare.

## eframe default features (as of 0.34.3)

`accesskit`, `default_fonts`, `wayland`, `web_screen_reader`, `wgpu`, `winit/default`, `x11`

**Recommended feature set for skirnir** (drop accesskit, web_screen_reader; keep glow instead of wgpu):

```toml
eframe = { version = "0.34.3", default-features = false, features = [
  "default_fonts",
  "x11",
  "wayland",
  "glow",
] }
```

Rationale:
- `default_fonts` — needed unless providing a custom font; keep it
- `x11` + `wayland` — both needed for Linux coverage; wlroots compositors and X11 both in common use
- `glow` over `wgpu` — OpenGL backend; lighter compile (no wgpu + its 20+ transitive crates); wgpu 29 is a heavy dep. For a 2D toolpath painter, glow is more than sufficient.
- Drop `accesskit` — adds non-trivial compile cost; can be re-added later if accessibility is required
- Drop `web_screen_reader` — web-sys pull-in; irrelevant for native Linux app

## rfd 0.17.2 features

Default: `xdg-portal`, `wayland`

- `tokio` feature was removed in 0.17.0 — rfd now handles async internally without requiring caller to pick a runtime
- `xdg-portal` uses pollster 0.4 (block-on executor), not ashpd anymore — no conflict with tokio 1
- `gtk3` is an alternative to xdg-portal; do NOT enable both — use xdg-portal default for modern desktops

Recommended for skirnir:
```toml
rfd = { version = "0.17.2", default-features = true }
# or equivalently:
rfd = "0.17.2"
```
Defaults (`xdg-portal` + `wayland`) are the right choice for a modern Linux desktop app.

## egui as direct dep vs re-export

`eframe` re-exports egui as `eframe::egui`. For simple usage (widgets, `Painter`, contexts) the re-export is sufficient.
Add `egui` as a direct dep ONLY if you need `egui` features not on by default, e.g. `serde` for egui type serialization.
For the 2D Painter viewport, no extra features needed — `Painter` is part of egui core, no feature gate.

## Conflict check

- Neither eframe 0.34.3 nor rfd 0.17.2 depend on tokio — zero conflict with skirnir's tokio 1 dep
- rfd 0.17.2 no longer uses ashpd (removed from Linux deps) — no async runtime conflict
- wgpu (if kept) would be 29.0.1; glow path avoids this entirely
- No version conflicts found against anyhow 1, thiserror 2, tokio-serial 5.5

**Why:** User is building skirnir GUI. egui/eframe is the chosen framework per docs/native-app.md.
**How to apply:** Use these exact versions and features when adding to crates/skirnir/Cargo.toml. Do not deviate from the glow recommendation without noting the compile-time cost of wgpu 29.
