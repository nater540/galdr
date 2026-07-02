# skirnir + egui_tiles — integration evaluation

**Status:** evaluation only — no decision made, no code written. This doc captures the findings from a first look
at adopting [`egui_tiles`](https://github.com/rerun-io/egui_tiles) (rerun-io) in `skirnir`, so a later decision has
a written baseline. Read `docs/native-app.md` and `docs/skirnir-design-brief.md` for the current UI intent first;
`crates/skirnir/src/app/shell.rs` is the source of truth for the live layout.

## Why this came up

Two motivations, in the requester's words:

1. **Make the console / program panel resizable.** Today it is a fixed-height bottom dock.
2. **Prefer a well-maintained, proven dependency over bespoke (LLM-written) layout code** that may be more brittle
   and harder to maintain long-term.

Both are legitimate, but they point at different things, and only one of them actually argues for `egui_tiles`.
The findings below separate them.

## What egui_tiles is

A docking / tiling layout manager for egui: panes arranged in **tabs**, **grids**, and **linear splits**, with
drag-to-rearrange docking, resizable splitters, and a **serde-serializable `Tree`** describing the whole layout.
It is what Rerun's viewer uses for its space-view grid. You give it a `Pane` type and implement an
`egui_tiles::Behavior` (tab titles, per-pane `ui`, simplification rules); it owns the arrangement and the splitters.

It shines when there are **many interchangeable views the user wants to arrange freely** (Rerun's space views, a
CAM/IDE workstation). It is heavier than needed when the layout is a small set of **fixed structural panels**.

## Hard constraint: version pairing

`skirnir` is on `egui`/`eframe` **0.34.3**. `egui_tiles` tracks egui minors tightly, so the pairing is fixed:

| egui_tiles | egui |
|------------|------|
| **0.15.0** | **0.34** ← the one we'd use |
| 0.16.0 | 0.35 (what `cargo add egui_tiles` grabs by default — **wrong**, would force an egui bump) |
| 0.14.0 | 0.33 |
| 0.13.0 | 0.32 |

So the dependency would be pinned `egui_tiles = "=0.15"`, and **every future egui/eframe bump must move in lockstep
with a matching egui_tiles release** — the same "move atomically" discipline the firmware already carries for the
esp-hal ↔ esp-bootloader pairing. That is a real, recurring maintenance tax, not a one-time cost.

## The current skirnir layout reality

`shell.rs` (~206 KB) is a deliberately **fixed, pixel-tuned `Panel` tree**, every panel `exact_size` and
`resizable(false)`:

- `Panel::top("toolbar")` — exact height
- `Panel::bottom("status")` — exact height
- `Panel::bottom("dock")` — the console / program panel, exact height (`Metrics::dock_height`)
- `Panel::left("controls")` — fixed 268 px (DRO + Jog)
- `Panel::right("rightcol")` — fixed 286 px (Overrides + Probe + Settings)
- `CentralPanel` — the backplot / preview

This rigidity is **intentional and hard-won**. The code comments and the project's own UI-debugging history record
that dense control rows overflow fixed panels (the "black-bar" clipping episode), so the metrics were nailed down on
purpose. The panels are egui's own well-maintained `Panel` API — *not* hand-rolled docking code. The bespoke,
higher-risk code in skirnir is the **custom-painted widgets** in `views.rs` (~213 KB), which `egui_tiles` does not
touch.

There is also a `ui_test.rs` harness that snapshots layout by rendering **through the real panel paths**. A tiles
`Tree` renders from stored state instead, so those tests would need the `Tree` seeded to a known layout — some are
rewritten, not just ported.

## Motivation 1 — resizable console/program panel

**`egui_tiles` is not required for this, and the panel's current fixedness is a deliberate workaround, not an
oversight.** The dock is pinned with `exact_size` specifically because (from `shell.rs`):

> the dock's body uses a fill-remaining `ScrollArea` (`auto_shrink([false, false])`), and on a resizable panel that
> height-feedback resolves the panel to most of the window on first layout. Pinning gives a deterministic 200px so
> the viewport reclaims the rest.

So naively flipping `Panel::bottom("dock")` to `.resizable(true)` re-triggers exactly the height-feedback blow-up the
author already hit. Two honest paths to a resizable console:

- **Plain egui (smaller):** keep `Panel::bottom("dock")` but make it resizable with an explicit
  `default_height`/`min_height`/`max_height` and constrain the inner `ScrollArea` so it can't drive the height
  feedback loop (e.g. give the scroll area a bounded `max_height` tied to the panel rect rather than fill-remaining).
  This is the least-code fix and keeps the rest of the shell untouched. It needs care to avoid the documented
  feedback bug, but it is a local change.
- **egui_tiles (larger):** a tiles split uses **explicit stored fractions** for the divider position rather than
  egui's height feedback, which sidesteps the `ScrollArea` interaction by construction — and the divider position
  persists for free via the serializable `Tree`. This genuinely solves motivation 1 more cleanly *if* the console
  becomes a pane in a tile group, but it only pays off as part of the broader adoption below, not for one divider.

**Verdict:** motivation 1 alone does **not** justify egui_tiles. It's achievable with a targeted egui change.
egui_tiles is the better mechanism only if we also want the next motivation's broader restructuring.

## Motivation 2 — maintained dependency vs bespoke code

This is the stronger argument, but it needs a precise target:

- The **layout scaffolding** in skirnir is already egui's first-party `Panel`/`CentralPanel` API — a maintained
  dependency, not bespoke code. Swapping it for egui_tiles trades one maintained layout system for another; it does
  not retire risky hand-written code, because there isn't hand-written docking to retire.
- The genuinely bespoke, higher-maintenance code is the **custom widgets and the pixel-tuned metrics** (`views.rs`,
  the `Metrics` constants, the alignment fixes). **egui_tiles does not replace any of that** — panes still render
  those same widgets inside tiles frames.
- Where motivation 2 *does* apply: if skirnir were ever going to grow its **own** docking / tear-off / save-layout
  behavior, hand-writing that on top of egui panels would be exactly the brittle, LLM-authored surface the requester
  wants to avoid — and egui_tiles is the proven, maintained answer for it. So the maintainability win is **real but
  conditional**: it accrues only for docking/rearrangement features we don't have yet, not for the layout we do.

**Verdict:** egui_tiles is worth adopting for maintainability **only if** we commit to user-rearrangeable layout as a
direction. It does not reduce maintenance burden on the existing fixed shell.

## Options

- **Option 0 — targeted egui change (no new dep).** Make just the dock resizable within the existing `Panel` tree,
  respecting the height-feedback constraint. Satisfies motivation 1. Zero dependency/version cost. Does nothing for
  motivation 2.
- **Option A — scoped egui_tiles (recommended trial).** Add `egui_tiles = "=0.15"` behind the existing `gui`
  feature. Convert **only the `CentralPanel`'s inner content** — backplot / console / DRO-detail / wizard views —
  into a dockable tile group. Keep toolbar / status / left-controls / right-col as fixed chrome. Persist the `Tree`
  in the existing RON profile store (`profile.rs`) — it's serde, so this is a natural fit. Gets a resizable,
  rearrangeable console (motivation 1) and a proven layout engine where it matters (motivation 2), while preserving
  the tuned chrome and most of `ui_test.rs`.
- **Option B — full docking rework.** Replace the whole shell with a tiles `Tree`. Only justified if a
  user-customizable multi-pane workstation (CAM-/Rerun-style) is an explicit product goal. Largest migration; fights
  the existing fixed-layout investment; `ui_test.rs` largely rewritten.

## Recommendation

Lean **Option A as a spike**, with **Option 0 as the fallback** if the spike shows egui_tiles fighting the tuned
widgets. Concretely:

1. Add `egui_tiles = "=0.15"` (serde feature) behind `gui`; confirm it resolves against the locked egui 0.34.3.
2. Define a `Pane` enum for the center views and a minimal `Behavior` (tab titles + dispatch to existing view fns).
3. Move the console/program dock and the backplot into a tiles split inside the current `CentralPanel` region.
4. Persist/restore the `Tree` via `profile.rs` (versioned RON, like the existing profile store).
5. Keep chrome panels (toolbar/status/left/right) as-is; keep their `ui_test.rs` coverage; add a Tree-seeded test
   for the center layout.
6. Evaluate: does the console resize cleanly, do the custom widgets behave inside tile frames (min-size/clipping),
   is the version pin acceptable? If yes, consider widening toward B; if it's fighting the widgets, fall back to
   Option 0.

Do **not** jump straight to Option B unless customizable operator layouts become a stated product direction.

## Risks / watch-items

- **Version lock-step** — `=0.15` pins us to egui 0.34; future egui bumps wait on egui_tiles. Document the pairing
  next to the dependency (mirror the esp-hal/bootloader note in `CLAUDE.md`).
- **Min-size / clipping regressions** — resizable tiles can shrink a pane below its content and reintroduce the
  black-bar clipping the fixed metrics were tuned to prevent. Each pane needs a `min_size`.
- **Test harness** — `ui_test.rs` snapshots via real panels; tiles panes render from `Tree` state, so center-layout
  tests must seed a known `Tree`.
- **Persistence migration** — a stored `Tree` needs a schema version and a sane default/reset, or a layout from an
  older build could restore broken (same discipline as the DOC-11 profile store).
- **Scope creep** — tiling invites turning modal wizard flows (probing/rotary) into dockable tabs; tempting but a
  separate decision, out of scope for the first spike.

## Bottom line

- Motivation 1 (resizable console) is real and **does not need egui_tiles** — a targeted egui change (Option 0) does
  it, minding the documented `ScrollArea` height-feedback gotcha.
- Motivation 2 (maintained over bespoke) **only argues for egui_tiles if we want user-rearrangeable layouts**; it
  does not retire the existing fixed shell's code, which is already on egui's first-party panels.
- If we want both — a resizable console *and* a proven, extensible layout engine — **Option A** is the right-sized
  trial, at the cost of a fixed egui/egui_tiles version pairing.
