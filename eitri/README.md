# Eitri

**Eitri** is a Rust CAM engine for PCB fabrication — a clean-room port of the useful half of
[FlatCAM](https://bitbucket.org/jpcgt/flatcam). It turns board artwork (Gerber / Excellon, plus SVG / DXF / G-code
import) into toolpaths and controller-ready G-code, and ships an egui front-end for visualizing and driving the
pipeline. It targets the Galdr firmware's grblHAL dialect but the postprocessor layer is pluggable.

Eitri is a **self-contained Cargo workspace nested inside the [Galdr](../README.md) repo**. It is deliberately *not*
a member of the root Galdr workspace, so the firmware/skirnir build stays untouched — build it with its own manifest
(see [Build & test](#build--test)).

## Workspace layout

The engine is a pipeline of small, single-responsibility library crates, plus one binary front-end. Each stage
consumes the previous stage's typed output, so the layers stay independently testable:

| Crate | Role |
|-------|------|
| [`crates/eitri-core`](crates/eitri-core) | Shared primitives: checked length units, affine transforms, precision constants, errors, and progress/cancel plumbing. |
| [`crates/eitri-geo`](crates/eitri-geo) | The geometry backend: CAM-shaped offsetting, boolean ops, simplification, and winding normalization (over `geo`/`i_overlay`/`clipper2`/`cavalier_contours`, with an optional `geos` Shapely-parity backend). |
| [`crates/eitri-gerber`](crates/eitri-gerber) | RS-274X Gerber parser — apertures, macros, regions, and polarity assembled into a copper `MultiPolygon`. |
| [`crates/eitri-excellon`](crates/eitri-excellon) | Excellon drill parser — tool table plus drill/slot hits, with number-format inference and override. |
| [`crates/eitri-import`](crates/eitri-import) | SVG / DXF / G-code import into Eitri geometry — the additive input formats beyond Gerber/Excellon. |
| [`crates/eitri-cam`](crates/eitri-cam) | The CAM heart: turns parsed board geometry into toolpath geometry (isolation rings) and an operation model (a travel-optimized drill plan). |
| [`crates/eitri-gcode`](crates/eitri-gcode) | NC generation — an emitter that decides motions and a pluggable postprocessor trait that renders them to a controller dialect (grblHAL is the shipped target). |
| [`crates/eitri-project`](crates/eitri-project) | The object/document model, versioned `serde` persistence, undo history, and the tool database — the integration crate that holds every other crate's output as one coherent project. |
| [`crates/eitri-script`](crates/eitri-script) | The scripting / command surface: a typed `Session` API over the engine (open/import, every CAM op, G-code output, undo, persistence), plus an embedded Rhai console on top of it. |
| [`crates/eitri-app`](crates/eitri-app) | The **egui/eframe** front-end: visualization, parameter editing, and op execution over the engine crates. Ships the `eitri-app` binary. |

The `fixtures/` directory holds the hand-authored Gerber / Excellon / SVG / DXF inputs and golden NC outputs that back
the integration tests; see [`fixtures/README.md`](fixtures/README.md).

## Pipeline

```
Gerber ─┐                        ┌─ isolation toolpaths ─┐
        ├─ eitri-gerber ─────────┤                       │
Excellon┤  eitri-excellon        ├─ eitri-cam ── eitri-gcode ──▶ G-code (grblHAL)
        │  eitri-import (SVG/DXF/ │  (operations)          post
SVG/DXF ┘   G-code)               └─ drill plan ──────────┘

        eitri-core (units / transforms)   ·   eitri-geo (boolean / offset backend)
        eitri-project (objects · undo · tool DB · persistence)   ·   eitri-script (Session API + Rhai)
        eitri-app (egui front-end over all of the above)
```

## Build & test

Eitri is host-native (stock Rust, no cross toolchain). Because it is a nested workspace, run cargo against its own
manifest — either from inside `eitri/` or with `--manifest-path`:

```sh
cd eitri
cargo build --workspace            # build every crate
cargo test  --workspace            # host tests (add RUSTFLAGS="-D warnings" to match CI)
cargo run   -p eitri-app           # launch the egui front-end
```

or from the repo root:

```sh
cargo test --manifest-path eitri/Cargo.toml --workspace
```

The `geos` Shapely-parity backend in `eitri-geo` is feature-gated (it needs a system GEOS library); the default build
uses the pure-Rust backends and needs no system deps.

## Conventions

Eitri follows the parent repo's style:

- Two-space indentation (`.editorconfig`), LF endings, final newline; comment lines target ~120 chars.
- No `unwrap()`/`expect()` in library code — errors propagate via `Result` and `thiserror` types.
- Short, single-responsibility functions; `///` docs on public APIs.
- `#![deny(warnings)]` in CI.

## Design docs

The design lives in the parent repo's `docs/`:

- [`docs/eitri-porting-plan.md`](../docs/eitri-porting-plan.md) — the full FlatCAM porting plan, phase by phase; the
  section references (`§6`–`§14`) cited in crate docs point here.
- [`docs/eitri-gcode-skirnir-contract.md`](../docs/eitri-gcode-skirnir-contract.md) — the grblHAL 1.1f wire contract
  `eitri-gcode` emits against (IJK arcs, 3-decimal / LF / ≤256 B lines, the strict supported-code list).
