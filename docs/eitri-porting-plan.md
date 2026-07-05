# Eitri — CAM/Toolpath Engine Port Plan

*A Rust reimplementation of FlatCAM's CAM functionality, structured for the Galdr project and the Skirnir G-code host.*

Eitri is the forge-worker: it takes raw fabrication input (Gerber, Excellon, DXF, SVG, G-code) and produces finished toolpaths and NC output. Skirnir then streams that output to the machine. This document is a functionality-and-algorithm plan only — no implementation code — organized as a proposed Cargo workspace, module by module. Each module notes its responsibility, the FlatCAM source it derives from, the algorithms it must reproduce, the geometry crates it leans on, and the porting risks.

The source we are porting is **FlatCAM** (jpcgt master lineage, with selective reference to the FlatCAM Beta / community 8.x branches where their algorithms are cleaner). FlatCAM is referenced throughout purely as provenance.

---

## 1. Guiding architecture decisions

**Separate the engine from the UI completely.** FlatCAM fuses geometry, CAM logic, project model, and a PyQt GUI in a handful of very large files (`camlib.py`, `FlatCAMApp.py`, `FlatCAMObj.py`). The port splits the GUI-agnostic engine into focused library crates; the egui front-end (`eitri-app`) is a thin consumer that never contains CAM logic. This is the single most important structural departure from the original.

**Geometry backend is a swappable layer.** FlatCAM leans on Shapely (which wraps GEOS) for essentially all planar geometry: buffering/offsetting, boolean ops, unary unions, simplification. Rather than scatter geometry calls across the codebase, Eitri routes all of it through one crate (`eitri-geo`) that exposes CAM-oriented operations. That lets us choose the best backend per operation — and swap it — without touching CAM code.

**Idiomatic Rust seams.** Parsers implement a common trait, postprocessors are plugins behind a trait, objects are an enum rather than a class hierarchy, and errors are typed (`thiserror`) rather than Python exceptions. Long-running operations return progress via channels so the UI stays responsive.

**Workspace layout:**

```
eitri/
  eitri-geo        # geometry backend abstraction (buffering, boolean ops, offsetting)
  eitri-core       # units, coordinates, transforms, shared primitives, error types
  eitri-gerber     # RS-274X Gerber parser
  eitri-excellon   # Excellon drill parser
  eitri-import     # SVG / DXF / G-code import
  eitri-cam        # isolation, paint, non-copper clear, cutout, drilling, panelize, film, 2-sided
  eitri-gcode      # G-code generation + postprocessor trait & plugins
  eitri-project    # object/document model, serialization, undo
  eitri-script     # scripting/command surface (replaces FlatCAM's Tcl console)
  eitri-app        # egui front-end (thin; no CAM logic)
```

Build order roughly follows dependency depth: `eitri-core` → `eitri-geo` → parsers → `eitri-cam` → `eitri-gcode` → `eitri-project` → `eitri-script` → `eitri-app`.

---

## 2. The geometry backend (`eitri-geo`) — crate research

This is the crux of the port. FlatCAM's isolation routing, painting, non-copper clearing, and cutout all reduce to two families of operation: **polygon offsetting** (Shapely's `buffer`) and **boolean ops** (`union`/`intersection`/`difference`, plus `unary_union`). The quality and robustness of these directly determines whether generated toolpaths are usable. Findings on the current Rust options:

### Boolean operations

- **`geo` / `geo-types`** — the core georust ecosystem. Mature, pure Rust, excellent primitive coverage (area, centroid, simplify via Douglas–Peucker/Visvalingam, convex hull, rotate/translate/scale affine ops). Its `BooleanOps` trait (union/intersection/difference/xor) is now backed by `i_overlay`. Solid for the bulk of CAM boolean work. Native buffering remains its weak spot (see below). **Use as the primary primitive/affine layer and for boolean ops.**
- **`i_overlay`** — high-performance, robust boolean ops on integer-snapped coordinates; this is what modern `geo` delegates to. Handles self-intersections and large polygon sets well, which matters for `unary_union` over thousands of Gerber traces. Also offers outline/stroke generation. **Use (directly or via `geo`) as the boolean-ops workhorse.**

### Polygon offsetting / buffering (the hard part)

- **`clipper2`** — Rust bindings to Angus Johnson's Clipper2 C++ library. This is the pragmatic CAM-grade choice for offsetting: `InflatePaths` with mitre/round/square joins, correct handling of inward (negative) offsets, and polygon clipping. CAM tools historically use Clipper precisely because its offsetting is battle-tested for toolpaths. Downside: C++ dependency (bundled, builds via `cc`), and it works in scaled-integer space so you manage a scaling factor. **Recommended primary offsetting backend.**
- **`cavalier_contours`** — pure-Rust polyline offsetting with **native arc support** (true arc segments, not polygonal approximations), self-intersection pruning, and robust handling of the collapse cases that appear when offsetting inward past feature size. This is uniquely valuable for toolpaths: arc-preserving offsets mean smoother motion and smaller G-code. Its model is open/closed polylines with per-vertex bulge, not Shapely multipolygons, so it needs an adapter. **Recommended for isolation and paint toolpath generation where arc output is desirable.**
- **`geo-buffer`** — straight-skeleton-based buffering. Useful and pure-Rust, but join/cap control is limited and it's less proven for the negative-offset, multi-ring cases CAM hits constantly. **Fallback / cross-check only.**
- **`geos`** — direct bindings to GEOS, the exact C++ library Shapely wraps. This is the maximum-fidelity option: if a FlatCAM result must match bit-for-bit, GEOS reproduces Shapely's `buffer` semantics (join style, mitre limit, quadrant segments) precisely. Cost is a heavier C/C++ dependency (system GEOS or vendored). **Recommended as an optional, feature-gated backend for parity validation and for users who want Shapely-identical output.**

### Supporting geometry

- **`spade`** — Delaunay and constrained Delaunay triangulation. Relevant if we implement certain area-fill or medial-axis-adjacent strategies, and potentially for robust point-in-polygon acceleration. Keep available; not on the critical path.
- Triangulation via **`earcutr`** if we need polygon tessellation for any preview/geometry validation.

### Recommendation summary

Route everything through `eitri-geo`'s own trait so the backend is swappable, and ship three implementations behind features:

- **Default:** `geo` + `i_overlay` for primitives/booleans, **`clipper2`** for offsetting, **`cavalier_contours`** for arc-preserving toolpath offsets.
- **Parity:** feature-gated **`geos`** backend that mirrors Shapely exactly, for validation against FlatCAM output and for users who need identical results.
- **Pure-Rust:** `geo` + `i_overlay` + `cavalier_contours` + `geo-buffer`, no C/C++, for environments where a pure-Rust build matters (accepting some offset-quality tradeoffs).

The `eitri-geo` public API should be CAM-shaped, not GEOS-shaped — e.g. `offset(poly, distance, join, miter_limit)`, `union_all(polys)`, `difference(a, b)`, `simplify(poly, tol)` — so CAM code never sees which backend answered.

**Porting risk:** Shapely's `buffer` has specific defaults (round joins, 8 quadrant segments, 5.0 mitre limit) that FlatCAM depends on implicitly. Reproducing FlatCAM output requires matching these, which is exactly why the `geos` parity backend earns its keep during validation even if the shipping default is Clipper2.

---

## 3. `eitri-core` — shared primitives

**Responsibility:** Units, coordinate types, affine transforms, tolerances, and the shared error type. No CAM logic, no I/O.

**Derives from:** The utility scattered through `camlib.py` (unit handling, `scale`/`offset`/`mirror`/`rotate`/`skew` helpers on the `Geometry` base class) and FlatCAM's app-wide unit conversion.

**Key functionality:**

- Length units (mm/inch) as a first-class type with explicit conversion. FlatCAM tracks units globally and converts on the fly; a great deal of subtle behavior lives in `convert_units()` on each object. Model units explicitly to avoid the silent double-conversion bugs FlatCAM was prone to.
- Affine transform primitives: translate, scale (about a point), rotate (about a point), mirror (about an axis/line), skew. FlatCAM implements each as a Shapely `affinity` call applied to every geometry; centralize the transform math here and let `eitri-geo` apply it.
- Tolerance/precision constants: the global geometric tolerance FlatCAM uses for simplification and dedup, and the decimal-rounding used on G-code output.
- The workspace-wide `Error` enum (via `thiserror`) and `Result` alias.

**Porting risk:** FlatCAM's unit handling is a recurring source of bugs — conversions applied twice, or applied to already-converted tool diameters. Making units a type the compiler checks removes a whole bug class.

---

## 4. `eitri-gerber` — RS-274X parser

**Responsibility:** Parse Gerber (RS-274X) into a resolved geometry model: a set of apertures and the flashes/traces/regions drawn with them, unioned into copper polygons.

**Derives from:** The `Gerber` class in `camlib.py` (the `parse_lines`/`parse_file` machinery), and the `ApertureMacro` class.

**Key algorithms:**

- **Tokenizing the command stream.** Gerber interleaves parameter blocks (`%...%`) and coordinate/operation words. FlatCAM uses a large set of compiled regexes per command type; a Rust port is better served by a proper lexer that recognizes format specification (`FS`), unit mode (`MO`), aperture definitions (`AD`), aperture macros (`AM`), aperture selection (`Dnn`), interpolation modes (`G01/02/03`), operations (`D01/02/03`), region mode (`G36/G37`), polarity (`LP`), and deprecated-but-seen constructs.
- **Coordinate format decoding.** The `FS` block defines integer/decimal digit counts and leading/trailing zero omission. Correctly reconstructing coordinates from zero-omitted words is fiddly and a known source of import errors; implement and unit-test this thoroughly against real-world files.
- **Aperture model.** Standard apertures (circle, rectangle, obround, polygon) plus macro apertures. Each aperture becomes a geometry generator: a flash (`D03`) places the aperture shape at a point; a draw (`D01`) with a circular aperture becomes a stroked path (a line buffered by radius).
- **Aperture macros.** The `AM` mini-language has primitives (circle, vector line, center line, outline, polygon, moiré, thermal) with expression evaluation and exposure (on/off) that add or subtract. This is effectively a tiny interpreter — port it as one: parse primitives, evaluate parametric expressions, compose via union/difference. FlatCAM's `ApertureMacro` is the reference.
- **Regions (`G36/G37`).** Contours drawn in region mode become filled polygons. Track contour winding and holes.
- **Polarity (`LP`).** Dark adds copper, clear subtracts. Accumulate as boolean ops, respecting order — later clear regions cut earlier dark ones.
- **Final assembly.** Union all dark geometry, subtract clear geometry, yielding the copper polygons downstream CAM consumes. This `unary_union` over many primitives is the heaviest geometry step in import; `i_overlay` handles it well.

**Geometry crates:** `eitri-geo` for stroking (buffer), flashing (primitive shapes), and the accumulate-by-polarity boolean pipeline.

**Porting risk:** Aperture macro expression evaluation and zero-omitted coordinate decoding are the two areas where subtle bugs hide. Build a corpus of real Gerbers (from KiCad, Eagle, Altium, and older tools) as parser fixtures — output from different EDA tools stresses different corners of the spec.

---

## 5. `eitri-excellon` — drill parser

**Responsibility:** Parse Excellon drill files into a tool table (diameters) and drill/slot hits.

**Derives from:** The `Excellon` class in `camlib.py`.

**Key algorithms:**

- **Header vs body modes.** Excellon has a header (tool definitions, `M48`) and a body (tool selection and hits). Parse both phases.
- **Number format inference.** Excellon is notoriously underspecified: units, zero suppression (leading/trailing), and decimal placement are often implicit. FlatCAM has heuristics to guess format when it isn't declared. Port these heuristics and expose an override, because guessing wrong shifts every hole by a factor of ten.
- **Tool definitions.** Map tool numbers to diameters; handle files that define tools inline or reference an external table.
- **Drills and slots.** Point hits (`X.. Y..`) and routed slots (`G85` or repeat/pattern constructs). Slots become segments, not just points.
- **Coordinate repeat / patterns.** Some files use repeat commands; expand them.

**Geometry crates:** minimal — `eitri-core` coordinates plus `eitri-geo` for representing slots as buffered segments when needed.

**Porting risk:** Format inference. Assemble fixtures with declared and undeclared formats, metric and imperial, leading and trailing suppression. This is where real files break parsers.

---

## 6. `eitri-import` — SVG / DXF / G-code import

**Responsibility:** Import vector and NC formats FlatCAM accepts into Eitri geometry.

**Derives from:** FlatCAM's SVG import (`svgparse`/`ObjectCollection` import paths), DXF import, and the G-code opening path in the `CNCjob` class.

**Key functionality:**

- **SVG.** Parse with `usvg` (which normalizes the SVG tree, resolves transforms, and flattens styling) and convert path/shape geometry into polygons/polylines. Curves (cubic/quadratic Béziers, arcs) get flattened to a tolerance. FlatCAM supports importing SVG as geometry for engraving/cutting; reproduce that.
- **DXF.** Use the `dxf` crate to read entities (LINE, LWPOLYLINE, POLYLINE, ARC, CIRCLE, SPLINE where feasible) and convert to polylines/polygons with arc flattening.
- **G-code import.** Parse existing G-code back into a toolpath/geometry preview. FlatCAM can open G-code to visualize and re-post it. This shares the token model with `eitri-gcode` (below), so the low-level G-code lexer should live in `eitri-gcode` and be reused here.

**Geometry crates:** `usvg` + `svgtypes`/`roxmltree` (SVG), `dxf` (DXF), curve flattening helpers in `eitri-geo`.

**Porting risk:** SVG transform and unit handling (user units vs mm, viewBox scaling) is easy to get subtly wrong. Arc/Bézier flattening tolerance should be shared with the rest of Eitri so imported geometry matches native precision.

---

## 7. `eitri-cam` — the CAM operations

This is the heart of Eitri and the main reason the port exists. Each operation below is a module. All derive from `camlib.py`'s `Geometry`/`Gerber`/`CNCjob` methods and the corresponding tool modules in FlatCAM's `flatcamTools/`.

### 7.1 Isolation routing (`isolation`)

**Derives from:** `Gerber.isolation_geometry()` and the isolation tool.

**Algorithm:** To isolate copper, offset every copper polygon outward by (tool_radius + overlap adjustments) and take the boundary as the cut path. Multiple passes offset at increasing distances, each pass a separate ring. Combine passes and optionally union overlapping rings so the cutter doesn't retrace. Key parameters: tool diameter, pass count, pass overlap, whether to "combine" passes, and milling direction (climb/conventional) which sets ring orientation.

- Offset each copper polygon by `radius + n * (tool_dia * (1 - overlap))` for pass `n`.
- The isolation path is the resulting offset ring(s) as polylines.
- Optionally offset a final "wall" pass. Optionally clear the whole non-copper region instead (that's the non-copper module).

**Geometry crates:** offsetting via `clipper2` (default) or `cavalier_contours` when arc-preserving output is wanted; boolean cleanup via `geo`/`i_overlay`.

**Porting risk:** Matching FlatCAM's exact ring spacing and the climb/conventional orientation convention. Direction is set by ring winding order, which each offset backend reports differently — normalize winding in `eitri-geo`.

### 7.2 Paint / area clearing (`paint`)

**Derives from:** `Geometry.paint_poly()` and its variants; the paint tool.

**Algorithm:** Clear the interior of a region with a tool, using one of several fill strategies FlatCAM offers. Reproduce all three:

- **Standard (concentric):** repeatedly inward-offset the region boundary by `tool_dia * (1 - overlap)` until it collapses, emitting each ring as a path. Concentric rings hugging the shape.
- **Seed-based:** start from a seed point and spiral/step outward with concentric offsets from the centroid — a variation on concentric that begins interior.
- **Line-based (raster):** intersect the region with a set of parallel lines spaced by `tool_dia * (1 - overlap)`, optionally at an angle, producing back-and-forth raster passes; connect ends into a boustrophedon path.

All strategies must handle a margin inset from the boundary, holes in the region, and multiple disjoint regions. After filling, optionally add a boundary-following finishing pass.

**Geometry crates:** inward offsetting (`clipper2`/`cavalier_contours`) for concentric/seed; line–polygon intersection (`geo`) for raster; `i_overlay` for combining.

**Porting risk:** Robustness as inward offsets collapse — the point where a region pinches off is exactly where offset backends misbehave. `cavalier_contours`' self-intersection pruning helps. Raster fill needs careful end-connection so the tool doesn't lift unnecessarily.

### 7.3 Non-copper clearing / copper pour clear (`noncopper`)

**Derives from:** FlatCAM's "non-copper regions" and "copper clear" tooling.

**Algorithm:** Compute the region to clear as `bounding_frame - copper` (optionally with a margin), then paint that region using the paint strategies above. The frame is either the board bounding box plus margin, or a convex/traced boundary. This is isolation taken to its extreme — remove *all* non-copper rather than a thin isolation ring — and reuses paint fill.

**Geometry crates:** `difference` via `geo`/`i_overlay` to get the clear region; paint module for filling.

### 7.4 Board cutout (`cutout`)

**Derives from:** the cutout tool.

**Algorithm:** Generate a profile path around the board outline for a routing bit, with **holding tabs (bridges)** left uncut so the board stays in the panel. Support both a geometry-derived outline and a simple rectangular cutout. Tab placement: either evenly spaced by count, or at specified positions; each tab is a gap in the cut path of a given width. Offset the outline outward by tool radius (cut outside the board), then subtract tab regions from the path.

**Geometry crates:** outline offset (`clipper2`), tab subtraction as path splitting (`eitri-geo`).

**Porting risk:** Splitting a closed offset path at tab locations cleanly, preserving order and direction, is the fiddly part.

### 7.5 Drilling (`drill`)

**Derives from:** `Excellon` → `CNCjob.generate_from_excellon...()`.

**Algorithm:** Turn drill hits into a drilling program: for each tool, order the hits to minimize rapid travel (see travel optimization below), then emit peck/drill cycles at each point per the tool's parameters (depth, feed, retract, peck depth, dwell). Slots become plunge-and-route moves rather than simple drills. Group by tool so tool changes are minimized.

**Geometry crates:** minimal; ordering is a point-sequence problem.

### 7.6 Travel-path optimization (`optimize`)

**Derives from:** FlatCAM's drill path optimization, which historically used Google OR-Tools TSP plus simpler fallbacks.

**Algorithm:** Minimize non-cutting rapid moves between drill hits (and between toolpath segments). Provide a tiered approach:

- **Nearest-neighbor** greedy ordering as the fast default.
- **2-opt / Or-opt improvement** pass over the nearest-neighbor tour for better results at low cost.
- Optionally an exact/【near-exact solver for small tool groups.

FlatCAM's dependency on OR-Tools is a porting liability (heavy, non-Rust). Replace with a self-contained Rust TSP heuristic (nearest-neighbor + 2-opt) that needs no external solver; this covers the drill-count ranges that matter and keeps the build clean. Leave a trait seam so a stronger solver can drop in later.

**Porting risk:** None severe — this is a clean win over FlatCAM. Just don't reproduce the OR-Tools dependency.

### 7.7 Panelization (`panelize`)

**Derives from:** the panelize tool.

**Algorithm:** Array a source object into a grid of `rows × cols` with X/Y spacing (gap or pitch), producing one combined object. Works on Gerber, Excellon, and geometry alike — translate copies and union/merge. Optionally add panel-level features. Handle the distinction between spacing-as-gap and spacing-as-pitch (a FlatCAM gotcha).

**Geometry crates:** translate (`eitri-core`), merge/union (`i_overlay`).

### 7.8 Film generation (`film`)

**Derives from:** the film tool.

**Algorithm:** Export a positive or negative film of copper (or other layers) to SVG/PDF for photo-etching/toner-transfer. Positive renders copper filled; negative renders the inverse within a border. Support scaling and mirroring (films are often mirrored for emulsion-side-down exposure) and a border/frame. Output is vector, not toolpath.

**Geometry crates:** boolean inverse for negative (`difference`), transforms; render via SVG writer (shared with `eitri-import`'s SVG side or a small writer here).

### 7.9 Two-sided PCB alignment (`twosided`)

**Derives from:** the 2-sided tool.

**Algorithm:** Two related jobs: (1) **mirror** a bottom-layer object about a chosen axis/point so it aligns with the top when the board is flipped; (2) generate **alignment drill holes** (registration holes) at symmetric points, plus optional alignment pins geometry. Support mirror about an axis line, about a point, or about a box center.

**Geometry crates:** mirror transform (`eitri-core`/`eitri-geo`).

### 7.10 Shared geometry editing ops

**Derives from:** the `Geometry` base class transform methods and the geometry editor.

Scale, offset, rotate, mirror, skew, buffer, simplify, and "join/union" applied to any object. These are thin wrappers over `eitri-core` transforms and `eitri-geo` ops, exposed uniformly across object types so the UI and scripting layer can apply them generically.

---

## 8. `eitri-gcode` — NC generation and postprocessors

**Responsibility:** Turn toolpaths (from `eitri-cam`) into G-code, and parse G-code back (shared lexer with `eitri-import`).

**Derives from:** the `CNCjob` class in `camlib.py` and FlatCAM's `preprocessors/` (formerly "postprocessors") directory.

**Key functionality:**

- **Toolpath → motion.** Walk each path emitting rapids to the start, plunge to cut depth (with multi-depth passes if the total depth exceeds pass depth), cut moves along the path, and lift. Handle feed rates (cut vs plunge), spindle speed, dwell, and tool changes. Arc-aware output emits `G02/G03` when the toolpath carries arcs (a reason `cavalier_contours` arc output is valuable), otherwise linearized `G01`.
- **Multi-pass depth.** Repeat a path at increasing depths until final depth is reached — shared by isolation, paint, cutout.
- **Postprocessor plugin architecture.** FlatCAM's postprocessors are Python modules with hook methods (`start_code`, `pre_move`, `move`, `spindle`, `end_code`, etc.) that different controllers (GRBL, LinuxCNC, Marlin, generic) customize. Port this as a **trait** — `Postprocessor` with methods for each hook — plus a registry of built-in implementations and the ability to register more. This is a clean, idiomatic mapping of FlatCAM's most extensible subsystem.
- **G-code lexer.** A tokenizer for reading G-code (used by import and for re-posting). Lives here; `eitri-import` calls it.
- **Output formatting.** Coordinate decimal precision, unit mode, line numbering, comments — all controlled per postprocessor and matching `eitri-core` precision settings.

**Porting risk:** The postprocessor hook contract is the delicate part — the set of hook points and the data each receives must be rich enough to reproduce every built-in FlatCAM postprocessor. Define the trait against the union of what existing postprocessors touch, and validate by reproducing GRBL and LinuxCNC output. Since Skirnir consumes Eitri's G-code, coordinate the dialect/precision defaults with Skirnir directly.

---

## 9. `eitri-project` — object/document model

**Responsibility:** The in-memory project: the set of loaded/derived objects, their parameters, serialization, and undo.

**Derives from:** `FlatCAMObj.py` (the object hierarchy: `FlatCAMGerber`, `FlatCAMExcellon`, `FlatCAMGeometry`, `FlatCAMCNCjob`) and `ObjectCollection`, minus all Qt.

**Key functionality:**

- **Object model as an enum, not inheritance.** FlatCAM uses a class hierarchy with Qt mixins. Rust models this better as an `Object` enum (`Gerber`, `Excellon`, `Geometry`, `CncJob`) with shared metadata (name, units, transforms, visibility) in a common struct and variant-specific payloads. Operations dispatch by match.
- **Object collection.** Named lookup, ordering, grouping — the data behind the UI's project tree, with no UI dependency.
- **Parameters.** Each object carries the CAM parameters used to derive it (tool tables, feeds/speeds, pass counts). This is what the UI edits and what scripting sets.
- **Serialization.** FlatCAM saves projects as pickled Python — not portable and a security liability. Replace with a versioned `serde` format (JSON or a compact binary) with an explicit schema and migration path. This is a deliberate break from FlatCAM's format.
- **Undo/redo.** A command/edit history over the object model.
- **Tool database.** FlatCAM's persistent tool library (diameters, feeds, speeds, per-operation defaults) as a serde-serialized store the CAM modules read defaults from.

**Porting risk:** Deciding the serialization schema up front and versioning it from v1. Don't inherit pickle. Migration from FlatCAM projects, if wanted, is a separate import shim — not the native format.

---

## 10. `eitri-script` — scripting / command surface

**Responsibility:** A programmatic command layer over the engine, replacing FlatCAM's Tcl console.

**Derives from:** FlatCAM's `TclCommand` framework (the `tclCommands/` package), which exposes operations like `open_gerber`, `isolate`, `cncjob`, `write_gcode` as console commands and scripts.

**Key decision:** FlatCAM embeds Tcl. Eitri has no reason to carry Tcl. Options, in preference order:

- Expose a **typed Rust command API** (each command a function/struct over the engine) as the real interface, and
- optionally bind it into an **embeddable Rust scripting engine** (`rhai` or `rune`) for users who want scripts, and/or
- a simple line-oriented command parser that mirrors FlatCAM's command names for muscle-memory compatibility.

Each command is a thin, testable function over `eitri-project` + `eitri-cam` + `eitri-gcode`. Because the engine is already cleanly separated, this layer is small.

**Porting risk:** Minimal. The main choice is which scripting runtime (if any) to embed; the typed command API is the load-bearing part and should exist regardless.

---

## 11. `eitri-app` — egui front-end

**Responsibility:** The GUI. Visualization of objects/toolpaths, parameter editing, running operations, and progress display. Contains **no** CAM logic — it drives the library crates.

**Derives from:** FlatCAM's PyQt UI, but reimagined for egui and an immediate-mode model. This document intentionally does not plan the UI in depth per your scope (egui, functionality-first). Notes only:

- Rendering the 2D canvas (copper, toolpaths, drills) — egui's painter or a plotting layer, with pan/zoom.
- Parameter panels bound to `eitri-project` object parameters.
- Running long operations off the UI thread with progress via channels (the engine already exposes progress hooks).
- The project tree over `eitri-project`'s collection.

**Porting risk:** Out of scope here by design; the key discipline is that no geometry or CAM code leaks into this crate.

---

## 12. Cross-cutting concerns

**Progress & cancellation.** FlatCAM long-ops block or use ad-hoc signals. Define a progress/cancel mechanism in `eitri-core` (a channel-based reporter + a cancellation token) that all `eitri-cam` operations accept, so the UI stays responsive and operations are interruptible.

**Parallelism.** Independent per-tool and per-object operations (isolating many polygons, drilling many tool groups, painting disjoint regions) parallelize cleanly with `rayon`. FlatCAM is largely single-threaded here; this is a free win, provided the geometry backend calls are thread-safe (Clipper2/GEOS calls should be isolated per-thread).

**Numerical precision & robustness.** Two recurring hazards: (1) offset backends misbehave as insets collapse — centralize collapse handling in `eitri-geo`; (2) coordinate rounding on G-code output must be consistent and controlled by `eitri-core` precision, not scattered `format!` calls. Snapping/scaling for integer-based backends (Clipper2, i_overlay) must use one shared scale factor.

**Testing strategy.** Build fixture corpora early: real Gerbers/Excellons from multiple EDA tools (KiCad, Eagle, Altium, older tools) for the parsers, and known-good FlatCAM outputs for the CAM ops. The feature-gated `geos` parity backend is the tool for validating that Eitri's offset/boolean results match Shapely/FlatCAM before switching the default to Clipper2. Golden-file tests on generated G-code guard the postprocessors.

---

## 13. Phased build order

1. **`eitri-core` + `eitri-geo`** — get units, transforms, and the geometry trait with a first backend (`geo`+`i_overlay`+`clipper2`) working and tested. Everything depends on this.
2. **`eitri-gerber` + `eitri-excellon`** — parsers with fixture corpora. Import is where real files break things; get it solid early.
3. **`eitri-cam` isolation + drilling** — the two highest-value operations; proves the geometry backend end to end.
4. **`eitri-gcode` + postprocessor trait** — output GRBL/LinuxCNC, validate against Skirnir.
5. **`eitri-cam` paint / non-copper / cutout / panelize / film / two-sided** — the remaining operations, reusing the offset/fill primitives.
6. **`eitri-import`** (SVG/DXF/G-code) — additive input formats.
7. **`eitri-project`** — serde model, tool DB, undo.
8. **`eitri-script`** — typed command API (+ optional `rhai`/`rune`).
9. **`eitri-app`** — egui front-end last, over a proven engine.
10. **`geos` parity backend** — feature-gated, in parallel from step 1 onward for validation.

---

## 14. Summary of key departures from FlatCAM

- Engine fully decoupled from GUI; egui front-end is a thin consumer.
- All geometry behind one swappable backend; default Clipper2 + `geo`/`i_overlay` + `cavalier_contours`, with a feature-gated GEOS parity backend replacing Shapely's exact semantics.
- OR-Tools drill optimization replaced by a self-contained nearest-neighbor + 2-opt heuristic.
- Pickled project format replaced by a versioned serde schema; FlatCAM import becomes a separate shim.
- Tcl console replaced by a typed Rust command API with optional `rhai`/`rune` scripting.
- Object class hierarchy replaced by an `Object` enum with match-based dispatch.
- Parallelism via `rayon` and first-class progress/cancellation throughout.
