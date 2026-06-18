---
name: design-spec
description: skirnir visual design source-of-truth — tokens, type scale, layout dims, component states, state colors, alarm banner
metadata:
  type: project
---

The authoritative visual design is `Skirnir.dc.html` (a Claude Design doc). Working copy was extracted to
`/tmp/skirnir-design/untitled/project/Skirnir.dc.html` (1586 lines, 6 sections: tokens, components, full
window, close-ups, lifecycle, rationale). `docs/skirnir-design-brief.md` is the prose brief. The design maps
1:1 to egui `Visuals`. See [[gui-architecture]] for how it's implemented.

**Color tokens (dark, near-black, flat — NO shadows except popovers/menus):**
- Chrome: `bg.window #121212`, `bg.panel #1B1B1B`, `bg.panelAlt #222222` (toolbar/tab strip/header),
  `bg.inset #0E0E0E` (console/fields/viewport), `bg.widget #2A2A2A` (control rest), `bg.widget.hover #333333`,
  `bg.widget.active #3C3C3C`, `divider #2E2E2E` (1px separators), `border.recess #080808` (inset top edge),
  `border.raised #3A3A3A`.
- Accents: `accent.primary #0E86D4` (blue — selection/focus/primary/control), `accent.secondary #FF7A1A`
  (orange — RESERVED for "right now": tool dot, traversed toolpath, realized spindle).
- Text on dark: `primary #E4E4E4`, `secondary #9A9A9A`, `disabled #5C5C5C`, `onAccent #FFFFFF`.
- Machine state dots: Idle `#3B82C4`, Run `#3FB861`, Hold `#E0A33E`, Alarm `#E5484D` (label text `#FF8488`),
  Jog `#2BB6C9`, Check `#9B7FE0`, Sleep/Disconnected `#6B6B6B`. State badge = dot + UPPERCASE label + thin
  matching outline; color is the SECOND signal, label is primary (shop lighting / safety glasses).
- Axis letter colors in DRO/viewport: X `#3FB861` (green), Y `#3B82C4` (blue), Z `#E0A33E` (amber).
- Alarm surfaces: bg `#2A0F11`, border `#5A2528`, text `#FF8488`. Console error/alarm rows get
  `rgba(229,72,77,0.10)` row bg. Settings `$NNN` keys are `#9B7FE0` (violet).

**Type:** Roboto 400/500/700 (UI) + JetBrains Mono 400/500 (DRO/console/numbers, `tabular-nums`). Both faces
are now VENDORED + wired into the egui font stack (see [[gui-architecture]] "Fonts: DONE"; both OFL-1.1). Scale:
caption.units 10, body.ui 11, control.label 12, section.header 13/Medium UPPERCASE +0.08–0.1em letter-spacing,
panel.title 15, console 12 mono, dro.numeric 34–42 mono Medium tabular. Section headers are uppercase tracked.

**Radii:** 0 divider, 2px control, 3px panel. Square-ish, no pills, no per-corner. 1px panel dividers,
2px accent underline on active tab, 1–2px splitter grab.

**Layout (full window 1600×940):** title bar 28px (`#0E0E0E`) → menu bar 24px (`#1B1B1B`: File/Machine/View/
Help) → main toolbar **40px** (`#222222`): connect group (port dropdown w/ power icon, green when connected) ·
Open btn · divider · **Run/Hold/Stop segmented group** (joined buttons, 26px tall, radius 2 0 0 2 / 0 / 0 2 2 0;
Running=green play, Hold=pause, Stop=red square) · divider · Home btn · state badge pushed right (dot+UPPERCASE
label + `F nnn · S nnnn` mono). Body grid **`268px | 1fr | 286px`**, height 600px. Bottom dock **200px** with
Console/Program tabs + right-aligned progress (`acked/total` + 260px bar (green `#3FB861`) + `NN%` + time
`m:ss / m:ss`) + command line w/ Send button. Status bar **24px** (`#0E0E0E`, mono 10.5): port dot, grblHAL
build, state, WCO set, `Ln a/b · NN%`, `F nnn · S nnnn`.

**Body left col (DRO+Jog):** every section has a 30px header (`#222222`, uppercase mono-ish title). DRO header
has WPos/MPos toggle (active = `#3C3C3C` bg `#0E86D4` text). DRO rows: big axis letter (colored) + 34–42px mono
tabular value right-aligned + `mm` unit. Zero X / Y / Z / **Zero XYZ** (primary blue) button row. WCO readout
strip (inset). Jog header shows "Jog active" badge when jogging. Jog = 3×3 XY arrow pad (center cell `XY` label)
+ Z+/Z/Z− column + esc Cancel-jog (alarm-styled). Step segmented selector `0.01 / 0.10 / 1.00 / 10.0 / cont`
(active = `#3C3C3C`/`#0E86D4`). Feed field.

**Body center (viewport):** 30px header "Toolpath" + filename + `412/650` + zoom/fit icon buttons. Canvas
`#0E0E0E` with major(80px `#1B1B1B`)/minor(16px `#161616`) grid, X/Y axis indicator bottom-left, dashed board
outline, traversed path ORANGE `#FF7A1A`, pending path dashed `#3A3A3A`, tool dot orange w/ crosshair + rings,
scale bar "10 mm". HUD top-left: tool coords (colored axis letters). Legend bottom-left.

**Body right col:** Overrides (Feed/Rapid/Spindle: −/slider/+ /percent; spindle slider orange; Realized F/S
readout) · Probe (instructions, primary "Probe Z → set zero" btn, last-probe result panel w/ green success dot +
contact Z) · Settings ($) list of `$NNN` rows (violet key, name, value; edit row highlighted blue).

**Component states (buttons):** primary blue rest `#0E86D4` / hover `#2BA8F0` / active `#0B6FB0` / focus
2px blue glow ring / disabled bg `#1F1F1F` text `#5C5C5C`. Secondary rest `#2A2A2A`+`#3A3A3A` border / hover
`#333333` / active `#3C3C3C` / focus blue border. Destructive (Stop): text `#E5484D` rest, hover `#3A1F20`bg/
`#FF7A7E`, active solid `#E5484D`/white. Icon buttons transparent rest, `#2A2A2A` hover.

**Console line types (mono, optional timestamp prefix `#5C5C5C`):** sent `> ` (`#0E86D4` chevron, white text),
response `< ok` (green `#3FB861` chevron, `#9A9A9A` text), status `<` violet `#9B7FE0` chevron, info `[MSG:]`
amber `#E0A33E`, error/ALARM red `#E5484D` chevron + `#FF8488` + row bg `rgba(229,72,77,0.10)`. "auto-scroll"
checkbox in tab strip. Program tab: line numbers, current line highlighted bg `rgba(14,134,212,0.12)` + 2px
`#0E86D4` left border + blue line number.

**Alarm banner (full-width strip under toolbar):** bg `#2A0F11`, border `#5A2528`, alert triangle icon
`#E5484D`, two-line message (`ALARM:1 · Hard limit triggered` `#FF8488` + secondary detail `#9A9A9A`), action
buttons: **Unlock $X** (secondary), **Soft reset** (destructive solid red), **Docs ↗** (ghost). NOT a modal —
inline strip. Each lifecycle phase (Disconnected/Connecting/Idle/Run/Hold/Alarm) has a distinct toolbar +
status-bar + badge signature (section 05).

**Out of scope v1:** light theme, floating windows, 3D viewport, multi-doc tabs, asset browser.
