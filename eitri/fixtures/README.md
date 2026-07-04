# Eitri fixtures

Test corpora for the parsers and (later) the CAM ops. Two kinds live here, kept apart on purpose:

```
fixtures/
  synthetic/        hand-authored, license-clean spec-corner cases (committed now)
    gerber/
    excellon/
  gerber/           REAL sample sets, one subdir per producing tool: gerber/<tool>/  (dropped in as they arrive)
  excellon/         REAL sample sets, one subdir per producing tool: excellon/<tool>/
  gcode/            (later) golden NC output for the postprocessors (§8, §12)
```

- **`synthetic/`** — small files written by hand for Eitri, each targeting one spec corner so a parser regression
  points straight at the construct that broke. No copyrighted vendor sample files.
- **`gerber/<tool>/`, `excellon/<tool>/`** — real fab files from actual EDA tools, chosen to include aperture
  macros, `G36/G37` pours, and varied Excellon number formats. Note the producing tool + version per set (see the
  README in each landing directory). The "parse a real KiCad Gerber into a copper `MultiPolygon`" acceptance check
  runs over the real KiCad set once it lands — until then it is pending-real-file, not blocking.

`kicad-cli` is not available on this host and the repo's `hardware/` board has no layout to plot, so real files are
supplied externally rather than generated here.

## `synthetic/gerber/` (see `docs/eitri-porting-plan.md` §4)

| File | Exercises |
|------|-----------|
| `kicad_two_pads.gbr` | KiCad `F.Cu` structure (`%TF` attributes, `FSLAX46Y46`, `MOMM`, `LPD`): two 2 mm `C` pads joined by a 0.25 mm trace → one connected copper polygon. Bounds/area sanity. |
| `coords_leading.gbr` | Coordinate decode with **leading-zero omission** (`FSLAX24Y24`), a pad at (5.0, 5.0) via `X50000`. |
| `coords_trailing.gbr` | Same physical pad via **trailing-zero omission** (`FSTAX24Y24`, `X05`) — both omission modes decode identically. |
| `macro_thermal.gbr` | An aperture **macro** (`AM` thermal primitive) defined and flashed — the macro interpreter + a macro-aperture flash. |
| `region_with_clear.gbr` | A `G36/G37` **region** and an `LP` **clear** region cutting the dark one: copper area 100 − 16 = 84. |
| `modal_draw.gbr` | **Finding #6** — a bare coordinate line after `…D01*` repeats the modal operation (a stroke), producing one continuous 0..2 capsule rather than stopping at x=1. |
| `macro_stroke_refused.gbr` | **Finding #4** — a `D01` draw with a macro aperture selected must error (`Unsupported`), not silently emit empty copper. |
| `arc_no_offset.gbr` | **Finding #5** — a `G02` arc with neither `I` nor `J` is invalid geometry (`InvalidGeometry`), not a silent straight chord. |
| `image_neg_refused.gbr` | **Finding #1** — `%IPNEG*%` (negative image polarity) is refused loudly (`Unsupported`) rather than ignored. |

> **Finding #2** (concave-region even-odd nesting with a robust interior point) is guarded by the `assemble_region`
> unit test in `eitri-gerber/src/parser.rs`, not a fixture: the full pipeline's `union_all` re-derives fill globally
> and launders the intermediate nesting error, so only the pre-union nesting step is discriminating.

## `synthetic/excellon/` (see §5)

| File | Exercises |
|------|-----------|
| `metric_leading.drl` | Declared **metric**, `TZ` (leading-zero suppression), `3.3`; a two-tool table and point drills. |
| `inch_trailing.drl` | Declared **inch**, `LZ` (trailing-zero suppression), `2.4` — the inverted `LZ`/`TZ` keyword trap. |
| `slot_g85.drl` | A `G85` **slot** (segment, not a point) buffered by the tool radius. |
| `undeclared_decimal.drl` | **No** unit/format declaration; explicit-decimal coordinates parsed via inference fallback. |
| `combined_tool_hit.drl` | **Finding #7** — a combined `T1X..Y..` line selects the tool AND drills the hit (the old code dropped it). |
| `large_drill.drl` | **Finding #10** — a 6 mm drill facets adaptively via the shared circle builder (more than the old fixed 48 segments). |
