# gerber2gcode — Design Specification

Single-binary Rust CLI that parses Gerber RS-274X copper layer files and
emits grblHAL-compatible isolation-routing G-code for PCB milling.

---

## Pipeline

```
.gbr  →  parser  →  GerberDoc  →  renderer  →  [CopperPoly]
                                                      ↓
 .nc  ←  gcode   ←  [Toolpath] ←  toolpath  ←  [Polygon]
```

---

## Crate layout

```
src/
  main.rs          CLI (clap derive) + orchestration
  error.rs         thiserror Error enum + Result alias
  parser/
    mod.rs         parse(source: &str) -> Result<GerberDoc>
    types.rs       GerberDoc, Primitive, Aperture, CoordFormat …
    state.rs       RS-274X parser state machine
  geometry.rs      Point, Polygon, constructors, offset()
  renderer.rs      render(doc, circle_segs) -> Vec<CopperPoly>
  toolpath.rs      isolation_paths(…) -> Vec<Toolpath>
  gcode.rs         emit(paths, params) -> String
```

---

## Dependencies

```toml
clap      = { version = "4", features = ["derive"] }
thiserror = "1"
log       = "0.4"
env_logger = "0.10"
```

No C bindings in the MVP. See §Polygon union below for the one recommended
addition.

---

## parser/types.rs — key types

### CoordFormat
```
zero_suppression: Leading | Trailing   (FSL / FST)
coord_mode:       Absolute | Incremental
x_int, x_frac:   u8  (digit counts from FS command, e.g. X35 → 3 int, 5 frac)
y_int, y_frac:   u8
unit:             Mm | In
```

Coordinate decode rule (Leading suppression, the common case):
  `value = raw_integer / 10^frac_digits`
  The raw integer is just `s.parse::<i64>()` — no padding needed.
For Trailing suppression: right-pad the string to `int+frac` digits before
parsing, then divide.
Always call `Unit::to_mm()` on the result so all internal values are in mm.

### Aperture
```
code:           u32
shape:          Circle { diameter }
              | Rect   { x, y }
              | Obround { x, y }
              | Polygon { diameter, vertices: u32, rotation_deg: f64 }
hole_diameter:  Option<f64>   (inner hole, ignored for routing purposes)
```
`diameter_for_stroke()` → returns the effective line width when this aperture
is used with D01 (min of x/y for rectangles).

### Primitive (collected output of the parser)
```
Stroke    { from, to: Point, width: f64, polarity }
ArcStroke { from, to, i, j: f64, cw: bool, width, polarity }
Flash     { pos: Point, aperture: Aperture, polarity }
Region    { start: Point, segments: Vec<RegionSegment>, polarity }

RegionSegment = Line { to } | Arc { to, i, j: f64, cw: bool }
```

### Polarity / Interp / ZeroSuppression
Simple enums; defaults are Dark, Linear, Leading.

### GerberDoc
```
format:     CoordFormat
unit:       Unit
apertures:  HashMap<u32, Aperture>
primitives: Vec<Primitive>
```

---

## parser/state.rs — state machine

Track:
- `x, y: f64`       current position (mm)
- `aperture: Option<u32>`
- `interp: Interp`  (G01 / G02 / G03)
- `multi_quad: bool` (G75 = true, default; G74 = false)
- `region: bool`    (between G36 / G37)
- `polarity: Polarity`
- `apertures: HashMap<u32, Aperture>`
- `primitives: Vec<Primitive>`
- region accumulator: `start: Point`, `segments: Vec<RegionSegment>`

### Extended parameters (`%...%` blocks, split on `*`)
| Tag | Action |
|-----|--------|
| `FS` | Parse `FSLAXiiYjj` → set CoordFormat |
| `MO` | `MM` or `IN` → set unit |
| `AD` | `ADDnn<shape>,<params>` → insert aperture |
| `LP` | `LPD` / `LPC` → set polarity |
| `AM` | warn "not supported", skip |
| `SR` | warn "not supported", skip |
| `TF/TA/TD/TO` | X2 attributes — silently ignore |
| `IP/IR/OF/SF/AS/MI` | deprecated — silently ignore |

`AD` param parsing:
- `C,diameter[Xhole]`
- `R,x_sizeXy_size[Xhole]`   (y defaults to x if omitted)
- `O,x_sizeXy_size[Xhole]`
- `P,diameterXvertices[Xrotation[Xhole]]`

### Data commands (blocks ending with `*`)
A single block may contain any combination of `X…Y…I…J…` coordinates
followed by G and/or D codes, e.g. `X12500Y7500D01` or `G01X5000D01`.

Parse tokens left to right:
1. Consume `X`, `Y`, `I`, `J` prefixed numbers into local vars.
2. Consume G codes: update `interp` / `region` / `multi_quad` etc.
3. Consume D codes:
   - `Dnn` where nn ≥ 10 → select aperture
   - `D01` → do_draw (Stroke or ArcStroke or RegionSegment)
   - `D02` → do_move (update x/y)
   - `D03` → do_flash

`G36` → set `region = true`, record `region_start = (x, y)`, clear segments.
`G37` → set `region = false`, push `Primitive::Region { start, segments }`.

`M02` / `M00` → stop parsing (end of file).

---

## geometry.rs

### Point
Standard 2-D point with `x: f64, y: f64`.
Implement `Add`, `Sub`, `Mul<f64>`, `Neg`.
Methods: `dist`, `dot`, `cross`, `norm`, `normalized`, `perp_left`
(`perp_left(v)` = `(-v.y, v.x)`, i.e. 90° CCW rotation).

### Polygon
```rust
struct Polygon { verts: Vec<Point> }
```

Constructors:
- `circle(center, radius, segs)` — N-gon approximation
- `rect(center, w, h)` — axis-aligned, CCW winding
- `obround(center, w, h, cap_segs)` — rounded rectangle (semicircular caps
  on the shorter axis)
- `regular(center, diameter, n, rotation_deg)` — regular n-gon
- `stroke(from, to, width, cap_segs)` — stadium / discorectangle shape
  (rectangle + two semicircular end caps)
- `arc_stroke(from, to, i, j, cw, width, segs)` — tessellate the arc into
  a polyline then build a strip polygon with `width`

Utilities:
- `signed_area()` → shoelace formula; positive = CCW
- `ensure_ccw()` → reverse if CW
- `area()` → `signed_area().abs()`
- `bbox()` → `(Point, Point)` min/max corners
- `offset(dist: f64) -> Polygon` — see §Offset below

### Arc tessellation helper
```
tessellate_arc(from, to, i, j, cw, segs) -> Vec<Point>
```
Center = `from + (i, j)`. Compute start angle from center→from,
end angle from center→to. Adjust end angle to enforce sweep direction
(add/subtract TAU as needed). Divide sweep into `ceil(sweep/TAU * segs)`
steps, sample `center + r*(cos θ, sin θ)`.

### Polygon offset algorithm

For each vertex `curr` with neighbours `prev` and `next`:

1. Compute outward unit normals `n1` (edge prev→curr) and `n2` (edge curr→next).
   For a CCW polygon, outward normal of edge a→b is `normalize(b.y-a.y, a.x-b.x)`.

2. Compute cross product of the two edge directions:
   `cross = (curr-prev) × (next-curr)`

3. **Convex corner** (`cross ≥ 0` for CCW): insert an arc from the
   `n1`-offset direction to the `n2`-offset direction (CCW sweep).
   Number of arc steps: `ceil(angle / TAU * segs)`, minimum 1.
   This is the Minkowski-sum approximation with a disk.

4. **Concave corner** (`cross < 0`): miter join.
   `m = n1 + n2`, `mhat = m / |m|`.
   Scale = `1 / dot(n1, mhat)`, clamped to 4.0 to prevent spikes.
   Output vertex: `curr + mhat * dist * scale`.

> **Production note:** For overlapping copper features or extremely concave
> geometries, replace this with `clipper2` (see §Polygon union).

---

## renderer.rs

```rust
struct CopperPoly { poly: Polygon, polarity: Polarity }

fn render(doc: &GerberDoc, circle_segs: usize) -> Result<Vec<CopperPoly>>
```

Map each `Primitive` to a `CopperPoly`:

| Primitive | Polygon constructor |
|-----------|-------------------|
| `Stroke { from, to, width }` | `Polygon::stroke(from, to, width, cap_segs)` |
| `ArcStroke { from, to, i, j, cw, width }` | `Polygon::arc_stroke(…)` |
| `Flash { pos, aperture }` | dispatch on `aperture.shape` → circle / rect / obround / regular |
| `Region { start, segments }` | walk segments, tessellate arcs inline, build `Polygon::new(verts)` |

`cap_segs = circle_segs / 2` (half-circle end caps).

Skip primitives where `width < 1e-9` or `poly.area() < 1e-6`.

---

## toolpath.rs

```rust
struct Toolpath { points: Vec<Point> }
// points[0] == points[last] (closed loop)

fn isolation_paths(
  copper:          &[CopperPoly],
  total_offset_mm: f64,      // tool_radius + clearance
  passes:          usize,
  tool_diam_mm:    f64,      // additional offset per pass
  circle_segs:     usize,
) -> Result<Vec<Toolpath>>
```

For each Dark-polarity polygon with `area > 1e-6`:
  For pass in `0..passes`:
    `offset_dist = total_offset_mm + pass * tool_diam_mm`
    Call `poly.offset(offset_dist)`.
    Close the loop by appending `points[0]` to the end.
    Push as `Toolpath`.

Skip polygons that vanish after offsetting (fewer than 3 vertices).

---

## gcode.rs

```rust
struct GcodeParams {
  depth_mm:       f64,
  safe_z_mm:      f64,
  feed_mm_min:    f64,
  plunge_mm_min:  f64,
  spindle_rpm:    u32,
  precision:      usize,   // decimal places (default 4)
}

fn emit(paths: &[Toolpath], params: &GcodeParams) -> Result<String, fmt::Error>
```

### Preamble
```
G21     ; mm
G90     ; absolute
G94     ; feed in mm/min
G17     ; XY plane
M5      ; spindle off during setup
G0 Z{safe_z}
M3 S{rpm}
G4 P2   ; 2 s spin-up dwell
```

### Per toolpath
```
G0 X{start.x} Y{start.y}   ; rapid to start (at safe Z)
G1 Z{-depth} F{plunge}      ; plunge
F{feed}
G1 X… Y…                   ; each subsequent point
G0 Z{safe_z}                ; retract
```

### Postamble
```
M5
G0 Z{safe_z}
G0 X0 Y0
M30
```

---

## main.rs — CLI flags

| Flag | Default | Description |
|------|---------|-------------|
| `<input>` | — | Gerber file path |
| `-o / --output` | stdout | G-code output file |
| `--tool-diameter` | 0.1 mm | V-bit effective cutting diameter |
| `--clearance` | 0.0 mm | Extra margin beyond tool radius |
| `--depth` | 0.1 mm | Cut depth (negated for Z) |
| `--safe-z` | 2.0 mm | Rapid Z height |
| `--feed` | 200 mm/min | Milling feed rate |
| `--plunge` | 50 mm/min | Z plunge rate |
| `--rpm` | 10000 | Spindle speed |
| `--passes` | 1 | Concentric isolation passes |
| `--circle-segs` | 32 | Circle/arc tessellation segments |
| `-v / --verbose` | false | Print stats to stderr |

---

## Known gaps / extension points

**Polygon union** — overlapping copper features each produce independent
toolpaths. For a proper merged boundary, union all Dark polygons before
offsetting. Recommended: add `clipper2 = "0.1"` and replace the inner loop
in `isolation_paths` with a Clipper2 union + offset call. The function
signature stays the same.

**Aperture macros (AM)** — complex parameterised shapes used by some EDA
tools. Parser warns and skips. Rare in practice for simple boards.

**Step-and-repeat (SR)** — array replication of features. Parser warns and
skips.

**Excellon drill files** — through-holes and vias are in a separate
`.drl` / `.xln` file with a different format. Natural second subcommand.

**Arc G-code output** — arcs from Gerber G02/G03 are currently tessellated
to line segments. Could emit `G2`/`G3` for smoother cuts; grblHAL and the
firmware use the same I/J-from-current-position convention as Gerber, so
translation is direct.

**Clear polarity (LPC)** — copper-removal regions are rendered but not
subtracted from Dark regions before toolpath generation. Correct handling
requires polygon difference (also a Clipper2 operation).
