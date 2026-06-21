# Hand-Authoring KiCad 10 Schematic-Only Projects — Reference

Target: KiCad **10.0.3** (verified locally: `kicad-cli version` → `10.0.3`). Schematic file format
`(version 20260306)`, `(generator_version "10.0")`. Goal: hand-write a valid, ERC-clean `.kicad_sch`
for a CNC carrier board (schematic only, no PCB) using stock global symbol/footprint libraries.

All library IDs below were grepped against the local install:
- Symbols: `/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols/*.kicad_sym`
- Footprints: `/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints/*.pretty/`

> **Library-availability caveat (verified):** This install ships **223 symbol libraries** but does
> **NOT** include `Connector_JST.kicad_sym` or `Connector_Phoenix_MC.kicad_sym` as *symbol* libraries.
> The matching **footprint** libs (`Connector_JST.pretty`, `Connector_Phoenix_MC.pretty`,
> `TerminalBlock_Phoenix.pretty`) **are** present. So JST and Phoenix parts must be drawn with a
> **generic `Connector_Generic:Conn_01x0N` symbol** + the specific footprint. This matches standard
> KiCad practice anyway (JST/Phoenix are mechanical connectors with no dedicated symbols).

---

## Part 1 — Exact symbol + footprint library IDs per component

Format of each row: **Symbol lib-id** → exists? → notes; **Footprint lib-id** (recommended stock).

### 1. TMC2209 stepper driver
- **Symbol:** `Driver_Motor:TMC2209-LA` — **EXISTS** (`Driver_Motor.kicad_sym`).
- **Footprint:** `Package_DFN_QFN:VQFN-28-1EP_5x5mm_P0.5mm_EP3.25x3.25mm_ThermalVias` — **EXISTS**, VQFN-28
  1EP, 5×5 mm, 0.5 mm pitch, thermal vias. (Non-vias variant `...EP3.25x3.25mm` also present; the
  `TQFN-28-1EP_5x5mm_P0.5mm_EP3.25x3.25mm_ThermalVias` is an alias-equivalent.)
- **Pins (28 + EP, EP = pin 29 "GND"):**

  | # | name | # | name | # | name | # | name |
  |---|------|---|------|---|------|---|------|
  | 1 | OB2 | 9 | MS1/AD0 | 17 | VREF | 25 | NC |
  | 2 | ~{EN} | 10 | MS2/AD1 | 18 | GND | 26 | OB1 |
  | 3 | GND | 11 | DIAG | 19 | DIR | 27 | BRB |
  | 4 | CPO | 12 | INDEX | 20 | STDBY | 28 | VS |
  | 5 | CPI | 13 | CLK | 21 | OA2 | 29 | GND (EP) |
  | 6 | VCP | 14 | ~{PD}/UART | 22 | VS | | |
  | 7 | SPREAD | 15 | VCC_IO | 23 | BRA | | |
  | 8 | 5VOUT | 16 | STEP | 24 | OA1 | | |

  Note: `~{EN}` and `~{PD}/UART` are KiCad overbar notation. Pins 22 & 28 both `VS`; pins 3/18/29 all `GND`.

### 2. ESP32-S3 devkit mounting
- **Module symbol (reference / direct-solder):** `RF_Module:ESP32-S3-WROOM-1` — **EXISTS**
  (`RF_Module.kicad_sym`). Bare chip symbol `MCU_Espressif:ESP32-S3` also exists.
- **Devkit-as-headers (recommended for a carrier board):** use two pin sockets. The standard part:
  - **Symbol:** `Connector_Generic:Conn_01x22` — **NOTE:** `Connector_Generic.kicad_sym` only defines
    `Conn_01x01 … Conn_01x08` as discrete symbols; **`Conn_01x22` does not exist as a pre-baked symbol**.
    For >8 pins KiCad uses the *parameterized* generic family. In a hand-authored file the practical
    options are: (a) split the 22-pin header into e.g. `Conn_01x08` + `Conn_01x08` + `Conn_01x06`
    instances, or (b) hand-edit one embedded symbol def to 22 pins (tedious). Cleanest is to place the
    devkit as the **WROOM module symbol** and footprint it with the pin sockets.
  - **Footprint:** `Connector_PinSocket_2.54mm:PinSocket_1x22_P2.54mm_Vertical` — **EXISTS**
    (1x20…1x29 all present, each with `_Vertical`, `_Vertical_SMD_Pin1Left`, `_Vertical_SMD_Pin1Right`).
    A typical ESP32-S3 DevKitC is 2× 1x22 rows (44 pins).

### 3. DC barrel jack 5.5×2.5 mm
- **Symbol:** `Connector:Barrel_Jack` (2-term) or `Connector:Barrel_Jack_Switch` (3-term, internal
  switch) — **both EXIST** (`Connector.kicad_sym`). Use `Barrel_Jack_Switch` if you want the switched
  ground-detect contact.
- **Footprint:** `Connector_BarrelJack:BarrelJack_Horizontal` (generic) or
  `Connector_BarrelJack:BarrelJack_CUI_PJ-102AH_Horizontal` (specific CUI PJ-102AH, 2.5 mm pin) —
  **both EXIST**. PJ-102AH is the standard 5.5×2.5 mm panel jack.

### 4. JST limit-switch connectors
- **Symbol:** no `Connector_JST` symbol lib here → use **`Connector_Generic:Conn_01x02`** /
  **`Conn_01x03`** / **`Conn_01x04`** — **all EXIST**.
- **Footprints (`Connector_JST.pretty`, JST-XH 2.50 mm — all EXIST):**
  - 1x02 vertical: `Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical`
  - 1x03 vertical: `Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical`
  - 1x04 vertical: `Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical`
  - Right-angle variants: swap `B?B-XH-A...Vertical` → `S?B-XH-A_1x0N_P2.50mm_Horizontal`
    (e.g. `JST_XH_S3B-XH-A_1x03_P2.50mm_Horizontal`).

### 5. Optocoupler
- **Symbol:** `Isolator:PC817` — **EXISTS**. Also `Isolator:LTV-817` (+ `LTV-817M/S`) — **EXIST**.
  `EL357N` and `TLP281` were **NOT FOUND** in `Isolator.kicad_sym` in this install — use `PC817` (the
  EL357N/LTV817 are pin-compatible 4-pin phototransistor optos; PC817 is the safe generic).
- **Footprints:**
  - THT DIP-4: `Package_DIP:DIP-4_W7.62mm`
  - SMD gull-wing (SO-4 / "SMD-4"): `Package_DIP:SMDIP-4_W7.62mm`, or `Package_SO:SO-4_4.4x3.6mm_P2.54mm`.
  (PC817 SMD parts are typically the SMDIP-4 / SOP-4 with 2.54 mm pin pitch.)

### 6. Pluggable screw terminal — motor outputs (1x04)
- **Symbol:** no `Connector_Phoenix_MC` symbol lib here → use **`Connector_Generic:Conn_01x04`**.
- **Footprints (3.81 mm pitch, all EXIST in `Connector_Phoenix_MC.pretty`):**
  - Horizontal (wire-entry): `Connector_Phoenix_MC:PhoenixContact_MC_1,5_4-G-3.81_1x04_P3.81mm_Horizontal`
  - Vertical: `Connector_Phoenix_MC:PhoenixContact_MCV_1,5_4-G-3.81_1x04_P3.81mm_Vertical`
  - 5.0 mm-pitch alternatives live in `Connector_Phoenix_MSTB.pretty` / `TerminalBlock_Phoenix.pretty`.
  - Note the comma in `MC_1,5` is part of the real filename — keep it verbatim in the lib-id.

### 7. Regulators
- **Buck module (MP1584 bare module):** no MP1584 symbol exists in `Regulator_Switching.kicad_sym`
  (confirmed: only `LM2596*` and similar discrete ICs). MP1584 modules are pre-built boards →
  **represent as `Connector_Generic:Conn_01x04`** (VIN, GND, VOUT, GND / EN) with footprint
  `Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical`. If you instead want a discrete buck IC,
  `Regulator_Switching:LM2596S-5` (+ `Package_TO_SOT_SMD:TO-263-...`) is a stock option.
- **LDO 3V3 fallback:**
  - `Regulator_Linear:AMS1117-3.3` — **EXISTS**; pins GND(1)/VO(2)/VI(3); footprint
    `Package_TO_SOT_SMD:SOT-223-3_TabPin2`.
  - `Regulator_Linear:AP2112K-3.3` — **EXISTS**; SOT-23-5, pins VIN(1)/GND(2)/EN(3)/NC(4)/VOUT(5);
    footprint `Package_TO_SOT_SMD:SOT-23-5`.

### 8. Passives (all `Device:*` — all EXIST in `Device.kicad_sym`)
| Symbol lib-id | exists | recommended footprint lib-id |
|---|---|---|
| `Device:R` | yes | `Resistor_SMD:R_0805_2012Metric` |
| `Device:C` | yes | `Capacitor_SMD:C_0805_2012Metric` |
| `Device:C_Polarized` | yes | `Capacitor_SMD:CP_Elec_6.3x3.9` (or `..._6.3x4.5`) |
| `Device:LED` | yes | `LED_SMD:LED_0805_2012Metric` |
| `Device:D_Schottky` | yes | `Diode_SMD:D_SMA` |
| `Device:D_TVS` | yes | `Diode_SMD:D_SMA` |
| `Device:Q_PMOS_GSD` | **NOT FOUND** — use `Device:Q_PMOS` (generic 3-term PMOS, G/D/S) | `Package_TO_SOT_SMD:SOT-23` |
| `Device:Fuse` | yes | `Fuse:Fuse_1206_3216Metric` (or `Fuse:Fuse_0805_2012Metric`) |

> `Q_PMOS_GSD` (fixed pin-order variant) is absent; `Device:Q_PMOS` exists. If you need an explicit
> GSD ordering, the generic `Q_PMOS` symbol's pins map G/S/D — verify against your MOSFET's pinout when
> assigning the SOT-23 footprint.

### 9. Power symbols (all in `power.kicad_sym`)
| Symbol lib-id | exists |
|---|---|
| `power:GND` | yes |
| `power:+12V` | yes |
| `power:+5V` | yes |
| `power:+3V3` | yes (also `power:+3V0`) |
| `power:VBUS` | yes |
| `power:VCC` | yes |
| `power:+VM` | **NOT FOUND** |

Each power symbol has exactly **one pin**, `(pin power_in line ... (name "") (number "1"))`, and a
`(power global)` flag — its **Value** string is the global net name. To make a custom **`+VM`**
motor-rail flag, copy the `power:+12V` definition into `lib_symbols`, rename the embedded symbol id to
`+VM`, and set its `Value`/graphic-text to `+VM`. Simpler: reuse `power:+12V` (or `power:VBUS`) and
just relabel, **or** drive the motor rail with an ordinary `(global_label "+VM" ...)` plus a
`power:PWR_FLAG` to satisfy ERC. (`power:PWR_FLAG` exists in `power.kicad_sym` and is the standard way
to tell ERC "this net is externally powered.")

---

## Part 2 — KiCad 10 `.kicad_sch` authoring format

All snippets below are pulled from the local template
`docs/hardware-references/Adafruit-TMC2209.kicad_sch` (version 20260306, generator_version "10.0") and
the stock `.kicad_sym` libraries. Indentation in the real files is **TAB**-based.

### 2.1 Top-level header
KiCad 10.0.3 writes and accepts:

```
(kicad_sch
	(version 20260306)
	(generator "eeschema")
	(generator_version "10.0")
	(uuid "990f79c5-2aa4-4f04-b85e-6c36907f72a4")
	(paper "A3")
	(lib_symbols
		...
	)
	... wires / junctions / labels / symbol instances / text ...
	(sheet_instances
		(path "/" (page "1"))
	)
	(embedded_fonts no)
)
```

- **`version` integer = `20260306`** — CONFIRMED accepted by 10.0.3 (the template carries it and
  `kicad-cli sch erc/export` parse it without complaint; see §2.7).
- `(paper ...)` accepts standard names: `"A4"`, `"A3"`, `"A2"`, `"A1"`, `"A0"`, `"USLetter"`, or
  `"User" <w_mm> <h_mm>` (the template uses `"User" 425.45 298.6024`). Use `"A3"` for a carrier board.
- Generate fresh UUIDs (RFC-4122 v4) with `uuidgen` (lowercase) for the sheet and **every** element.
- `(embedded_fonts no)` is present at the end in v20260306 files — include it.

### 2.2 `(lib_symbols ...)` — embedding symbol definitions (MANDATORY)
**Yes — every symbol referenced by a placed `(symbol (lib_id "Lib:Name") ...)` instance must have its
full definition embedded in the file's `lib_symbols` block.** The `.kicad_sch` is self-contained; it
does not read the global `.kicad_sym` files at load time for geometry — it uses the embedded copy. The
`lib_id` of a placed instance must exactly match a `(symbol "Lib:Name" ...)` inside `lib_symbols`.

**Precise embed mechanism (copy-and-prefix):**
1. Open the stock library, e.g. `.../symbols/Device.kicad_sym`.
2. Copy the entire top-level `(symbol "R" ... )` block (from `(symbol "R"` through its matching close
   paren — this includes the child `(symbol "R_0_1" ...)` / `(symbol "R_1_1" ...)` graphic sub-units).
3. Paste it inside `(lib_symbols ...)` and **rename the top-level id** by prefixing the library name:
   `(symbol "R"` → `(symbol "Device:R"`. (Leave the child `R_0_1` / `R_1_1` ids unchanged — they are
   referenced internally by the `_unit_bodystyle` suffix convention, not by lib-id.)
4. Placed instances then use `(lib_id "Device:R")`.

That is exactly how the template does it (its ids are prefixed with the project-import library name,
e.g. `"Adafruit TMC2209 Stepper Motor Driver-eagle-import:+5V"`).

**Faster alternative — let KiCad build `lib_symbols` for you:** author the instances with bare
`lib_id`s, place them in the GUI once, save, and copy the auto-populated `lib_symbols` back; or use
`kicad-cli` export round-trips. But the copy-and-prefix method above needs **no GUI** and is fully
hand-authorable. (`kicad-cli sym export`/`upgrade` operate on `.kicad_sym` libraries, not on embedding
into a sheet, so they don't shortcut this step.)

A trimmed real example of an embedded def (header of `Device:R`):

```
		(symbol "Device:R"
			(pin_numbers (hide yes))
			(pin_names (offset 0))
			(exclude_from_sim no)
			(in_bom yes)
			(on_board yes)
			(duplicate_pin_numbers_are_jumpers no)
			(property "Reference" "R" (at 2.032 0 90) ... )
			(property "Value" "R" (at 0 0 90) ... )
			(property "Footprint" "" (at -1.778 0 90) (hide yes) ... )
			(property "Datasheet" "" ... )
			(property "Description" "Resistor" ... )
			(property "ki_keywords" "R res resistor" ... )
			(property "ki_fp_filters" "R_*" ... )
			(symbol "R_0_1"
				... rectangle graphic ...
			)
			(symbol "R_1_1"
				(pin passive line (at 0 3.81 270) (length 1.27) (name "~" ...) (number "1" ...))
				(pin passive line (at 0 -3.81 90) (length 1.27) (name "~" ...) (number "2" ...))
			)
		)
```

### 2.3 A placed component instance
Real instance from the template (a power symbol-style placement). For a resistor it looks like:

```
	(symbol
		(lib_id "Device:R")
		(at 100.33 76.20 0)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(uuid "0a1b2c3d-1111-4222-8333-444455556666")
		(property "Reference" "R1"
			(at 102.87 73.66 0)
			(effects (font (size 1.27 1.27)) (justify left))
		)
		(property "Value" "10k"
			(at 102.87 78.74 0)
			(effects (font (size 1.27 1.27)) (justify left))
		)
		(property "Footprint" "Resistor_SMD:R_0805_2012Metric"
			(at 98.552 76.20 90)
			(hide yes)
			(effects (font (size 1.27 1.27)))
		)
		(property "Datasheet" "~" (at 100.33 76.20 0) (hide yes) (effects (font (size 1.27 1.27))))
		(pin "1" (uuid "aaaa1111-2222-4333-8444-555566667777"))
		(pin "2" (uuid "bbbb1111-2222-4333-8444-555566667777"))
		(instances
			(project "galdr-carrier"
				(path "/<SHEET-UUID>"
					(reference "R1")
					(unit 1)
				)
			)
		)
	)
```

Key rules:
- `(at x y rot)` is mm; `rot` ∈ {0, 90, 180, 270}.
- `(unit 1)` for single-unit parts; `(body_style 1)` may appear (= De Morgan body 1; optional, defaults 1).
- One `(pin "<num>" (uuid ...))` line per symbol pin (just the UUID mapping, geometry is in lib_symbols).
- `(instances (project "<proj-name>" (path "/<root-sheet-uuid>" (reference "R1") (unit 1))))` — the
  `path` is `"/"` + the **sheet UUID** from the file header (root sheet). The `reference` here is what
  ERC/netlist use. `<proj-name>` should match the `.kicad_pro` base name.

### 2.4 Wires, junctions, labels, no-connects, power placement
**Wire** (verbatim from template):
```
	(wire
		(pts
			(xy 220.98 149.86) (xy 236.22 149.86)
		)
		(stroke (width 0.1524) (type solid))
		(uuid "056006e5-b629-4ec9-a43d-20a7a0c6ba1d")
	)
```
Wires connect only at endpoints; place a wire endpoint exactly on a pin's grid coordinate to connect.

**Junction** (needed where 3+ wires meet / a wire crosses onto another mid-span):
```
	(junction
		(at 243.84 233.68)
		(diameter 0)
		(color 0 0 0 0)
		(uuid "272e2318-3b59-4b16-8848-6e82b8edc597")
	)
```

**Local label** (net name, scoped to this sheet — verbatim from template):
```
	(label "STEP"
		(at 218.44 220.98 90)
		(effects (font (size 1.2446 1.2446)) (justify left bottom))
		(uuid "02e63a97-8a35-4318-968f-db2888b8019a")
	)
```

**Global label** (net spans sheets; also the simplest way to name a power-ish rail):
```
	(global_label "+VM"
		(shape input)
		(at 150.00 100.00 0)
		(effects (font (size 1.27 1.27)) (justify left))
		(uuid "ffff0000-1111-4222-8333-444455556666")
	)
```
`shape` ∈ `input | output | bidirectional | tri_state | passive`.

**Hierarchical label** (only inside a child sheet, matched to a parent `(sheet (pin ...))`):
```
	(hierarchical_label "BUS0"
		(shape bidirectional)
		(at 120.00 90.00 0)
		(effects (font (size 1.27 1.27)) (justify left))
		(uuid "1234abcd-1111-4222-8333-444455556666")
	)
```
For a **single-sheet** carrier board you do **not** need hierarchical labels or `(sheet ...)` blocks.

**No-connect** (suppresses ERC "unconnected pin"; place its `at` exactly on the pin coordinate):
```
	(no_connect
		(at 175.26 152.40)
		(uuid "9999aaaa-1111-4222-8333-444455556666")
	)
```
(The template has none — `no_connect` is a top-level element identical in shape to the above.)

**Power-symbol placement** is just a normal `(symbol ...)` instance whose `lib_id` is `power:GND`,
`power:+12V`, etc., with the symbol's single pin landing on the net. Example:
```
	(symbol
		(lib_id "power:GND")
		(at 100.33 90.17 0)
		(unit 1)
		(in_bom yes) (on_board yes) (dnp no)
		(uuid "7777cccc-1111-4222-8333-444455556666")
		(property "Reference" "#PWR01" (at 100.33 96.52 0) (hide yes) (effects (font (size 1.27 1.27))))
		(property "Value" "GND" (at 100.33 94.49 0) (effects (font (size 1.27 1.27))))
		(pin "1" (uuid "8888dddd-1111-4222-8333-444455556666"))
		(instances (project "galdr-carrier" (path "/<SHEET-UUID>" (reference "#PWR01") (unit 1))))
	)
```
Power-symbol references use the `#PWR0N` convention (hidden). `power:PWR_FLAG` is placed the same way
with reference `#FLG0N` to assert externally-driven nets for ERC.

### 2.5 `sheet_instances` (REQUIRED)
Every schematic needs exactly this near the end (root sheet, page 1):
```
	(sheet_instances
		(path "/" (page "1"))
	)
```
Without it KiCad treats the file as malformed / pages won't resolve.

### 2.6 Companion files

**`.kicad_pro`** (JSON) — minimal valid project. KiCad will backfill defaults, but this loads cleanly:
```json
{
  "board": { "design_settings": {}, "layer_presets": [], "viewports": [] },
  "boards": [],
  "libraries": { "pinned_footprint_libs": [], "pinned_symbol_libs": [] },
  "meta": { "filename": "galdr-carrier.kicad_pro", "version": 1 },
  "net_settings": { "classes": [] },
  "pcbnew": { "page_layout_descr_file": "" },
  "schematic": {
    "drawing": {},
    "legacy_lib_dir": "",
    "legacy_lib_list": []
  },
  "sheets": [
    [ "<ROOT-SHEET-UUID>", "Root" ]
  ],
  "text_variables": {}
}
```
The `sheets` UUID must equal the `.kicad_sch` header `(uuid ...)`. Name the file the same base as the
`.kicad_sch` (e.g. `galdr-carrier.kicad_pro` ↔ `galdr-carrier.kicad_sch`).

**`sym-lib-table` / `fp-lib-table`:**
- **Stock-only project → NO local tables needed.** Confirmed: the global tables live in the KiCad
  config dir (`~/Library/Preferences/kicad/10.0/sym-lib-table` and `.../fp-lib-table` both present),
  and lib-ids like `Device:R`, `power:GND`, `Connector_JST:...` resolve through those global tables.
  A project that references only stock `Library:Symbol` / `Library:Footprint` ids needs no
  project-local `sym-lib-table` or `fp-lib-table` at all.
- **If you add a CUSTOM library**, drop a project-local table. Minimal `sym-lib-table`:
  ```
  (sym_lib_table
    (version 7)
    (lib (name "MyParts")(type "KiCad")(uri "${KIPRJMOD}/MyParts.kicad_sym")(options "")(descr ""))
  )
  ```
  Minimal `fp-lib-table`:
  ```
  (fp_lib_table
    (version 7)
    (lib (name "MyParts")(type "KiCad")(uri "${KIPRJMOD}/MyParts.pretty")(options "")(descr ""))
  )
  ```
  `${KIPRJMOD}` resolves to the project directory. Referenced lib-ids become `MyParts:SymbolName`.
  (Note: even with a custom library, you STILL must embed its symbol def into the sheet's
  `lib_symbols` — the table only matters for re-editing in the GUI, not for the sheet being
  self-contained.)

### 2.7 Validating without the GUI — `kicad-cli` (CONFIRMED working on 10.0.3)
All three commands below were run against the template and **succeeded** (the `Fontconfig` lines on
stderr are harmless macOS noise, not errors):

```sh
KCLI=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli

# 1) ERC — exit code 0 even with violations; read the printed count + .rpt file.
$KCLI sch erc galdr-carrier.kicad_sch
#   → "Found N violations" ... "Saved ERC Report to galdr-carrier-erc.rpt"
#   A CLEAN pass prints "Found 0 violations" (or "No errors"/"No violations"); inspect the .rpt.
#   (On the template it reported "Found 121 violations" — expected, it's an Eagle import with
#    unconnected pins, which is the baseline to beat in your own file.)

# 2) SVG render — proves geometry/lib_symbols parse and pins resolve.
$KCLI sch export svg --output ./svgout galdr-carrier.kicad_sch
#   → "Plotted to './svgout/galdr-carrier.svg'." "Done."

# 3) Netlist — proves connectivity/instances/references resolve.
$KCLI sch export netlist --output galdr-carrier.net galdr-carrier.kicad_sch
#   → writes a KiCad netlist; inspect nets/components to confirm wiring.
```

**What a clean ERC pass looks like:** `Found 0 violations` (or report body listing no errors). Use
`--severity-error`/`--severity-warning`/`--exit-code-violations` flags (run `$KCLI sch erc --help`) if
you want a non-zero exit on violations for CI gating. For a hand-authored board, iterate: render SVG to
eyeball placement, run ERC, fix unconnected-pin / power-input-not-driven / missing-PWR_FLAG issues,
repeat.

---

## Authoring checklist (single-sheet carrier board)
1. Generate a root sheet UUID (`uuidgen`); put it in the `.kicad_sch` header and in `.kicad_pro`
   `"sheets"`.
2. Build `lib_symbols`: copy each needed stock symbol def, prefix its id with the library name
   (`Device:R`, `power:GND`, `Driver_Motor:TMC2209-LA`, `Connector_Generic:Conn_01x04`, …).
3. Place each `(symbol ...)` instance with a fresh UUID, a `Reference`, a `Value`, a `Footprint`
   property (the lib-ids from Part 1), and an `(instances (project ...(path "/<SHEET-UUID>")...))` block.
4. Add `power:*` symbols and at least one `power:PWR_FLAG` per externally-driven rail (+12V/+5V/+3V3/GND).
5. Draw `(wire ...)` segments endpoint-on-pin; add `(junction ...)` at 3-way meets; name nets with
   `(label ...)`; cap intentionally-floating pins with `(no_connect ...)`.
6. Add `(sheet_instances (path "/" (page "1")))` and `(embedded_fonts no)`.
7. Run `kicad-cli sch erc`, drive violations to 0; render SVG + export netlist to confirm.

No local `sym-lib-table`/`fp-lib-table` is needed as long as every lib-id is a stock global library.
