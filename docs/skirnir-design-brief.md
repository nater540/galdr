# Skirnir — Visual Design Brief & Handoff Prompt

> **How to use this document.** This is a self-contained design prompt. Hand it to a designer (or paste it into
> Claude) to produce a **visual design system + screen mockups** for `skirnir`. It is intentionally opinionated about
> the *aesthetic direction* (Unreal Engine 5.5 editor) and the *screens that exist*, while leaving the actual visual
> craft to the design pass. Deliverables are listed at the end. Implementation target is **egui (eframe)** in Rust, so
> keep every choice egui-implementable (see "egui constraints").

---

## 1. What skirnir is

`skirnir` is the native desktop **GCode sender** for the Galdr CNC PCB-milling system. It connects to an ESP32-S3
running grblHAL-compatible firmware over USB CDC serial, streams GCode programs, and gives the operator live control
and feedback while a job runs. Linux-first, single-window desktop app. The user is a maker at a workbench milling PCBs
— they need a dense, glanceable, *confidence-inspiring* control surface, not a consumer app.

Primary jobs the UI must support: connect to the board, jog the machine, set work zero (incl. Z-probe touch-off), load
and stream a GCode program with live progress, watch a digital readout (DRO) of position, see machine state, adjust
feed/spindle overrides live, and read/write firmware `$`-settings.

## 2. Design north star

**Unreal Engine 5.5+ editor ("Starship" UI) — but simpler.** We want skirnir to feel like a focused professional
tool from the same family as the UE5 editor: a **dark, near-black, flat, dense, panel/dock-based** workspace with a
restrained cool palette, crisp typography, thin monochrome line icons, and a central "viewport." We are NOT cloning
the engine's complexity — skirnir has a handful of panels, not hundreds. Borrow the *language*, not the scope.

Keywords: professional, dark, flat, dense, precise, calm, glanceable. Avoid: rounded "friendly" consumer styling,
drop-shadow-heavy material design, bright saturated fills, playful illustration.

## 3. Visual system

### 3.1 Color palette (UE5-Starship-inspired)

Dark, desaturated, cool. These hexes capture the target feel; **refine against the actual UE5 dark theme** (its
Slate theme JSON lives under `…/UnrealEngine/Slate/Themes/`). Deliver final values as design tokens (§6).

| Token | Hex | Use |
|-------|-----|-----|
| `bg.window` | `#121212` | App window / outermost background |
| `bg.panel` | `#1B1B1B` | Panel/dock body background |
| `bg.panelAlt` | `#222222` | Toolbars, tab strips, panel headers |
| `bg.inset` | `#0E0E0E` | Recessed wells: console, text fields, the viewport letterbox |
| `bg.widget` | `#2A2A2A` | Buttons / controls (rest) |
| `bg.widget.hover` | `#333333` | Control hover |
| `bg.widget.active` | `#3C3C3C` | Control pressed / active |
| `border.recess` | `#080808` | Dark inner-shadow line under insets |
| `border.raised` | `#3A3A3A` | Subtle light edge on raised controls |
| `divider` | `#2E2E2E` | 1px panel/section separators |
| `text.primary` | `#E4E4E4` | Default text (~89% white, never pure #FFF on dark) |
| `text.secondary` | `#9A9A9A` | Labels, units, secondary info |
| `text.disabled` | `#5C5C5C` | Disabled / placeholder |
| `text.onAccent` | `#FFFFFF` | Text on an accent fill |
| `accent.primary` | `#0E86D4` | "Unreal blue" — selection, active tab, focus ring, primary button |
| `accent.hover` | `#2BA8F0` | Accent hover / highlight |
| `accent.secondary` | `#FF7A1A` | Sparingly: active toolpath segment, "running" emphasis (UE viewport-orange) |

**Machine-state colors** (state badge, DRO accent, status bar — skirnir-specific, not from UE5):

| State | Hex | | State | Hex |
|-------|-----|-|-------|-----|
| Idle | `#3B82C4` (calm blue) | | Jog | `#2BB6C9` (cyan) |
| Run | `#3FB861` (green) | | Check | `#9B7FE0` (violet) |
| Hold | `#E0A33E` (amber) | | Sleep / Disconnected | `#6B6B6B` (gray) |
| Alarm | `#E5484D` (red) | | | |

**Semantic:** success `#3FB861` · warning `#E0A33E` · error `#E5484D` · info `#0E86D4`. Use color as a *secondary*
signal (paired with an icon/label), never the only one — operators glance under shop lighting.

### 3.2 Typography

- **UI font: Roboto** (the UE Slate default — keeps the family resemblance). Weights: Regular 400, Medium 500
  (emphasis / active tab / primary button), Bold 700 (rare, headers only).
- **Monospace: JetBrains Mono** (or Roboto Mono) for the **GCode console/terminal** and the **DRO numeric readout** —
  tabular figures matter so digits don't jitter as values change.
- Scale (px, dense): caption/units 10 · base UI 11 · control labels 12 · section headers 13–14 (Medium) ·
  panel/title 15–16. **DRO numerals: large, 30–44** (mono, tabular), per-axis label small above/beside.
- Tight but legible line height (~1.25 for UI, ~1.4 for console). Generous letter-spacing only on tiny ALL-CAPS
  section headers (UE5 uses subtle uppercase labels for panel/section titles).

### 3.3 Iconography

Thin, single-weight **line icons** (~1.5px stroke on a 16px grid) — Lucide / Feather style matches the UE5 flat-modern
look. Monochrome: `text.secondary` at rest, `text.primary` on hover, `accent.primary` when active/toggled. No filled
or skeuomorphic icons. Provide an icon for each toolbar/panel action (connect, open, play, pause, stop, home, probe,
zero, jog directions, settings, console).

### 3.4 Shape, elevation, density

- **Square-ish.** Corner radius 2–3px max (UE5 is nearly square). No pill buttons.
- **Flat.** Separate regions with **1px dividers**, not drop shadows. The only shadows are on floating menus/popovers
  (subtle). Insets read as recessed via the `border.recess` top line + darker fill, not heavy bevels.
- **Dense.** Row/control height ~22–24px; toolbar ~36–40px; padding 4–8px; splitters between panels are draggable
  1–2px grab strips that brighten on hover. Tabs are flat with a 2px `accent.primary` underline on the active tab.

## 4. Layout

Single window, **dockable panel layout** echoing the UE5 editor: title/menu bar, a main toolbar, a central viewport,
side panels, a bottom output dock, and a status bar. Target wireframe (panels are resizable; this is the default):

```
┌─ Galdr · skirnir ───────────────────────────────────────────── ─ □ × ┐
│  File   Machine   View   Help                                          │  ← menu bar (thin)
├───────────────────────────────────────────────────────────────────────┤
│ [⏻ Connect ▾] [📂 Open]   │  [▶ Run] [⏸ Hold] [■ Stop]   │  ◍ RUN      │  ← main toolbar + state badge
├──────────────┬──────────────────────────────────────┬─────────────────┤
│  D R O       │                                      │  Overrides      │
│  X  12.340   │            TOOLPATH VIEWPORT          │  Feed  ▮▮▮▯ 100%│
│  Y   8.005   │        (2D top-down; pan/zoom;        │  Rapid ▮▮▮▮ 100%│
│  Z  -1.200   │         tool dot + traversed path)    │  Spin  ▮▮▯▯  80%│
│  [Zero XYZ]  │                                      ├─────────────────┤
│  Work ▾      │                                      │  Probe          │
│ ───────────  │                                      │  [G38 Z touch]  │
│  JOG  ◄ ▲ ►  │                                      ├─────────────────┤
│      ▼  step │                                      │  Settings ($)   │
│  feed [____] │                                      │  …              │
├──────────────┴──────────────────────────────────────┴─────────────────┤
│  Console │ Program                          [█████████░░░░] 412/650     │  ← bottom dock + stream progress
│  > $H                                                                   │
│  < ok                                                                   │
│  < <Run|MPos:12.3,8.0,-1.2|FS:300,800>                                  │
│  ┌─────────────────────────────────────────────────────────┐  [Send]   │  ← command input
├───────────────────────────────────────────────────────────────────────┤
│  ● /dev/ttyACM0 · grblHAL 1.1 · Idle · WCO set      Ln 412/650 · 63%   │  ← status bar
└───────────────────────────────────────────────────────────────────────┘
```

## 5. Screens & components to design (with key states)

1. **Connection bar** — port dropdown (auto-detected), connect/disconnect, post-connect firmware banner/version,
   connecting/error states.
2. **Machine-state badge** — Idle/Run/Hold/Alarm/Jog/Check/Sleep/Disconnected, color + icon + label; the alarm state
   must be unmistakable (and pairs with an alarm banner/modal carrying the `ALARM:N` reason + unlock hint).
3. **DRO (digital readout)** — large mono X/Y/Z, Work vs Machine toggle (WPos/MPos), per-axis and all-axis zero,
   shows `WCO`/units; values update at the status-report rate without layout jitter.
4. **Jog pad** — X/Y arrows + Z up/down, step-size selector (continuous + discrete), feed field; clear "jog cancel."
5. **GCode console / output log** — sent lines vs responses (`ok`/`error:N`/`<…>`/`[…]`) visually distinguished;
   error lines highlighted; auto-scroll w/ pause-on-scroll; a command input line for manual `$`/GCode.
6. **Program / streaming view** — loaded file, line list w/ current-line highlight, **progress bar**, run/pause/stop,
   elapsed/remaining, throughput; clear "streaming" vs "idle" affordance.
7. **Toolpath viewport** — the central "viewport": 2D top-down toolpath, tool position dot, traversed vs pending path,
   pan/zoom, fit; a thin viewport toolbar. (3D is a later nice-to-have — design the 2D first.)
8. **Overrides panel** — feed / rapid / spindle as sliders or +/- steppers showing live percentages and the realized
   feed/RPM; reflects the firmware `Ov:`/`FS:` values.
9. **Probing panel** — the no-touch-plate Z-zero workflow (G38.x): a guided "probe Z → set zero" action with the
   `[PRB:]` result and a clear success/fail state.
10. **Settings ($) editor** — list/edit firmware `$`-settings (synced via the `$PBX` protobuf channel), grouped,
    with units and ranges; dirty/saved state.
11. **Status bar** — port, firmware, state, key flags, line counter, % complete.

## 6. Interaction, states & motion

- Every control: rest / hover / active-pressed / focused (accent ring) / disabled. Toggles show an accent underline
  or fill when on.
- **Motion is minimal and functional** (UE5 is restrained): ~80–120ms ease on hover/expand, a subtle progress-bar
  fill, a brief flash on a new alarm. No bouncy or decorative animation.
- Connection lifecycle (Disconnected → Connecting → Connected/Idle → Streaming → Hold → Alarm → Error) should each
  have an obvious visual signature in the toolbar + status bar + state badge.

## 7. egui (eframe) constraints — keep it implementable

The design will be built in **egui**, an immediate-mode GUI. Stay within what egui does cleanly:

- **Yes:** flat solid fills, 1px strokes/dividers, small uniform corner radius, simple hover/active fills, tab strips,
  resizable side/bottom panels and splitters, custom fonts (Roboto + a mono), monochrome icons (font-icon or small
  textures), a custom-painted 2D toolpath canvas.
- **Avoid / use sparingly:** gradients, layered drop shadows, blur, fine per-corner radii, pixel-perfect overlapping
  translucency — egui can fake some but it fights the immediate-mode model and the flat aesthetic anyway.
- Deliver the palette/typography/spacing as **design tokens** (a flat table of named colors, font sizes, spacing
  units, radii) that map directly onto egui's `Visuals`/`Style`/`TextStyle` — this is the most useful single artifact
  for implementation. Note `widgets.{noninteractive,inactive,hovered,active,open}` fills + strokes explicitly.

## 8. Out of scope / explicit simplifications vs UE5

No asset browser, no node graphs, no property "details" mega-panels, no multi-document tabs, no light theme (dark
only for v1), no floating/tear-off windows (fixed dock is fine for v1). One window, the panels in §4, that's it.

## 9. Deliverables requested from the design pass

1. **Design tokens** — final color palette (all §3.1 tokens + the egui `widgets` states), type scale, spacing, radii,
   icon set list. (Highest priority — unblocks implementation.)
2. **Component sheet** — the controls in §5 in each state (rest/hover/active/disabled/focus).
3. **Screen mockups** — the default full-window layout (§4) plus close-ups of: DRO + jog, console + program, the
   toolpath viewport, overrides, probing, and the alarm state.
4. **A short rationale** mapping each major choice back to the UE5-editor reference so we can sanity-check fidelity.

Reference for the streaming behavior the UI wraps: `docs/gcode-streaming.md`, `docs/tlo-offsets.md`, and the grblHAL
contract in `CLAUDE.md`. Reference for skirnir's architecture: `docs/native-app.md`.
