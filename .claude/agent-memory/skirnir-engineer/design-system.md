---
name: design-system
description: skirnir egui visual design system — UE5-Starship dark theme tokens, layout geometry, components, sourced from the Claude Design handoff
metadata:
  type: project
---

The skirnir UI follows a Claude Design handoff (`Skirnir.dc.html`, exported 2026-06-17) realizing the
`docs/skirnir-design-brief.md` "Unreal Engine 5.5 Starship" aesthetic: dark, flat, dense, panel/dock-based.
Implemented as egui `Visuals`/`Style` tokens. The brief + the .dc.html are the source of truth; re-fetch the
design URL if pixel details are needed.

**Core color tokens (hex).** bg.window #121212, bg.panel #1B1B1B, bg.panelAlt(toolbar/header) #222222,
bg.inset(console/fields/viewport) #0E0E0E, bg.widget #2A2A2A, hover #333333, active #3C3C3C, divider #2E2E2E,
border.recess #080808, border.raised #3A3A3A, accent.primary "Unreal blue" #0E86D4 (hover #2BA8F0, active
#0B6FB0), accent.secondary orange #FF7A1A (active toolpath / run emphasis). Text: primary #E4E4E4, secondary
#9A9A9A, disabled #5C5C5C, onAccent #FFFFFF.

**Machine-state colors.** Idle #3B82C4, Run #3FB861, Hold #E0A33E, Alarm #E5484D, Jog #2BB6C9, Check #9B7FE0,
Sleep/Disconnected #6B6B6B. Always pair color with icon+label, never color alone. Semantic: success #3FB861,
warning #E0A33E, error #E5484D, info #0E86D4.

**Type.** UI = Roboto (400/500/700); mono = JetBrains Mono (400/500) for console + DRO, with tabular figures
(critical: DRO updates at status rate, must not jitter). Scale px: caption/units 10, body.ui 11, control.label
12, section.header 13 Medium uppercase +0.08em tracking, panel.title 15 Medium, console 11.5-12, DRO numeric
34-38 mono tabular. Radius: 2px controls, 3px panels, 0 dividers (square-ish, no pills). Row height 22-24px,
toolbar 40px, panel headers 30px, title bar 28px, menu bar 24px, status bar 24px.

**Window geometry (1600x940 default).** Top→bottom: title bar 28 (#0E0E0E) · menu bar 24 (File/Machine/View/
Help) · main toolbar 40 #222 (connect group, Open, divider, Run/Hold/Stop segmented group, Home, state badge
pushed right with F/S readout) · body grid `268px | 1fr | 286px` height ~600 · bottom dock 200 (Console/Program
tabs + progress bar far-right) · status bar 24. Left col = DRO (header w/ WPos|MPos toggle, 3 axes big mono
right-aligned, Zero X/Y/Z + Zero XYZ accent, WCO strip) then Jog (3x3 XY pad + Z+/Z- column + step-size
segmented 0.01/0.10/1.00/10.0/cont). Center = toolpath viewport #0E0E0E (grid bg, traversed=orange, pending=
faint dashed, tool dot orange, HUD coords top-left). Right col = Overrides (Feed/Rapid/Spindle bars + realized
F/S) · Probe (Probe Z→set zero, last [PRB:] result) · Settings ($) list (`$NNN` violet #9B7FE0, name, value).

**Alarm.** Full-width banner under toolbar: bg #2A0F11, border #5A2528, icon+`ALARM:N · reason` in #FF8488,
secondary reason line, buttons Unlock $X / Soft reset (red) / Docs. State badge turns red.

**Axis accent in DRO/viewport:** X #3FB861 (green), Y #3B82C4 (blue), Z #E0A33E (amber) — note these differ
from the state colors; they are per-axis labels.
</content>
</invoke>
