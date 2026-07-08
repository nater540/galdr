# eitri-app — English (US), the source locale and the fallback.
# One key per UI string. Sibling locales must declare exactly this key set (a test enforces parity).

app-title = Eitri

# ── Toolbar ─────────────────────────────────────────────────────────────────────────────────────────────
btn-open-gerber = Open Gerber…
btn-open-excellon = Open Excellon…
btn-import = Import
btn-import-svg = SVG…
btn-import-dxf = DXF…
btn-import-gcode = G-code…
btn-project-open = Open Project…
btn-project-save = Save Project…
btn-undo = Undo
btn-redo = Redo
btn-zoom-fit = Fit view
tip-settings = Application settings
tip-undo = Undo the last edit
tip-redo = Redo the last undone edit
tip-zoom-fit = Zoom the canvas to the loaded geometry

# ── Project tree ────────────────────────────────────────────────────────────────────────────────────────
tree-title = Project
tree-empty = No objects yet.
tree-empty-hint = Open a Gerber or Excellon file to begin.
kind-gerber = Gerber
kind-excellon = Excellon
kind-geometry = Geometry
kind-cncjob = CNC job
btn-delete-object = Delete object
vis-hide = Hide { $name }
vis-show = Show { $name }

# ── Parameter panels ────────────────────────────────────────────────────────────────────────────────────
params-title = Parameters
params-empty = Select an object to edit its parameters.
params-busy = An operation is running — parameters unlock when it finishes.
params-isolation = Isolation routing
params-tool-diameter = Tool dia. (mm)
params-passes = Passes
params-overlap = Overlap
params-combine = Combine passes
params-direction = Direction
direction-climb = Climb
direction-conventional = Conventional
params-cut-depth = Cut depth (mm)
params-pass-depth = Per pass (mm)
params-cut-feed = Cut feed (mm/min)
params-plunge-feed = Plunge (mm/min)
params-travel-z = Travel Z (mm)
params-spindle-rpm = Spindle (RPM)
btn-run-isolation = Isolate
tip-run-isolation = Compute isolation toolpaths and emit a CNC job
params-drill = Drilling
params-drill-depth = Depth (mm)
params-drill-feed = Feed (mm/min)
params-drill-retract = Retract Z (mm)
btn-run-drill = Drill
tip-run-drill = Order the drill hits and emit a drilling job
params-job = CNC job
job-lines = { $count ->
    [one] 1 line of G-code
   *[other] { $count } lines of G-code
  }
job-dialect = Dialect: { $dialect }
btn-export-gcode = Export G-code…
tip-export-gcode = Write this job's G-code to a file
params-geometry = Geometry
geometry-shapes = { $polygons } polygons · { $polylines } polylines
excellon-hits = { $hits } drill hits · { $tools } tools
gerber-info = Copper regions parsed and cached

# ── Operation picker + the breadth op panels ────────────────────────────────────────────────────────────
params-operation = Operation
op-choice-isolate = Isolate
op-choice-paint = Paint
op-choice-noncopper = Non-copper clear
op-choice-cutout = Cutout
op-choice-panelize = Panelize
op-choice-mirror = Mirror
op-choice-film = Film export
params-paint = Area clearing
params-margin = Margin (mm)
params-strategy = Strategy
strategy-concentric = Concentric
strategy-seed = Seed
strategy-raster = Raster
params-raster-angle = Raster angle (°)
params-finish-pass = Finish pass
btn-run-paint = Paint
tip-run-paint = Clear the copper area and emit a CNC job
params-noncopper = Non-copper clearing
params-boundary = Boundary
boundary-bbox = Bounding box
boundary-object = Object silhouette
boundary-object-none = pick an object…
params-boundary-margin = Bbox margin (mm)
params-boundary-source = Boundary object
btn-run-noncopper = Clear non-copper
tip-run-noncopper = Clear everything but the copper inside the boundary and emit a CNC job
error-boundary-object = Pick a boundary object with geometry before running the non-copper clear.
params-cutout = Board cutout
params-outline = Outline
outline-rectangle = Rectangle
outline-silhouette = Object silhouette
params-rect-min-x = Rect min X (mm)
params-rect-min-y = Rect min Y (mm)
params-rect-max-x = Rect max X (mm)
params-rect-max-y = Rect max Y (mm)
params-tab-width = Tab width (mm)
params-tab-count = Tabs
btn-seed-bounds = Use object bounds
tip-seed-bounds = Fill the rectangle from the selected object's extent
btn-run-cutout = Cut out
tip-run-cutout = Route the board outline with holding tabs and emit a CNC job
error-outline-object = The selected object has no silhouette geometry to cut around.
params-panelize = Panelize
params-rows = Rows
params-cols = Columns
params-spacing-mode = Spacing
spacing-gap = Gap
spacing-pitch = Pitch
params-spacing-x = Spacing X (mm)
params-spacing-y = Spacing Y (mm)
btn-run-panelize = Panelize
tip-run-panelize = Array the object into a grid as a new geometry object
params-mirror = Mirror (two-sided)
params-mirror-axis = Axis
axis-vertical = Vertical (flip X)
axis-horizontal = Horizontal (flip Y)
params-mirror-value = Line at (mm)
btn-seed-center = Object centre
tip-seed-center = Place the mirror line at the selected object's centre
btn-run-mirror = Mirror
tip-run-mirror = Reflect the object about the line as a new geometry object

# ── Setup node (stock & work zero) ──────────────────────────────────────────────────────────────────────
tree-setup = Setup
setup-kind = Job setup — stock & work zero
setup-hint = The material block being machined and where work X0 Y0 Z0 sits on it. Objects never move — only the coordinates posted into G-code shift.
params-stock = Stock
params-stock-x = Size X (mm)
params-stock-y = Size Y (mm)
params-stock-thickness = Thickness (mm)
btn-fit-stock = Fit to board
tip-fit-stock = Size the stock to the chosen object's bounding box
params-datum = Datum (X0 Y0)
datum-corner-bl = Bottom left
datum-corner-br = Bottom right
datum-corner-tl = Top left
datum-corner-tr = Top right
datum-corner-center = Center
params-z-zero = Z zero
z-zero-top = Top of stock
z-zero-bottom = Bottom of stock
work-zero-label = Work zero (mm)
work-zero-native = Native frame — no offset applied
btn-stock-native = Native (no stock)
tip-stock-native = Post G-code in the source file's own coordinate frame
stock-set = Stock set — work zero at X { $x } Y { $y } Z { $z }
stock-cleared = Stock cleared — posting in native coordinates
params-film = Photo film
params-film-kind = Kind
film-positive = Positive
film-negative = Negative
params-film-scale = Scale
params-film-mirror = Mirror
params-film-border = Border (mm)
btn-export-film = Export film SVG…
tip-export-film = Write a positive/negative film of the copper as an SVG file

# ── Operations / progress ───────────────────────────────────────────────────────────────────────────────
op-running = Running: { $label }
btn-cancel = Cancel
tip-cancel = Request cancellation of the running operation
op-cancelled = { $label } was cancelled
op-failed = { $label } failed: { $reason }
op-done = { $label } finished
op-open-gerber = Open Gerber
op-open-excellon = Open Excellon
op-import-svg = Import SVG
op-import-dxf = Import DXF
op-import-gcode = Import G-code
op-isolate = Isolation routing
op-drill = Drill planning
op-paint = Area clearing
op-noncopper = Non-copper clearing
op-cutout = Board cutout
op-panelize = Panelizing
op-mirror = Mirroring
op-load-project = Open project

# ── Dock ────────────────────────────────────────────────────────────────────────────────────────────────
dock-log = Log
dock-gcode = G-code
gcode-empty = Select a CNC job to preview its G-code.
log-ready = Ready.

# ── Canvas ──────────────────────────────────────────────────────────────────────────────────────────────
canvas-label = Canvas
canvas-empty-title = Nothing to show yet
canvas-empty-hint = Open a Gerber, Excellon, SVG, or DXF file to see it here.

# ── Status bar ──────────────────────────────────────────────────────────────────────────────────────────
status-objects = { $count ->
    [0] no objects
    [one] 1 object
   *[other] { $count } objects
  }
status-units = mm
status-idle = idle

# ── Files / errors ──────────────────────────────────────────────────────────────────────────────────────
file-filter-gerber = Gerber
file-filter-excellon = Excellon
file-filter-svg = SVG
file-filter-dxf = DXF
file-filter-gcode = G-code
file-filter-project = Eitri project
error-open-file = Could not read { $path }: { $reason }
error-save-file = Could not write { $path }: { $reason }
export-done = Wrote { $path }
project-saved = Project saved to { $path }
project-loaded = Project loaded: { $path }

# ── Application settings dialog ─────────────────────────────────────────────────────────────────────────
app-settings-title = Application Settings
app-settings-language = Language
app-settings-theme = Theme
app-settings-font-scale = Font scale
app-settings-new-theme-hint = new theme name
app-settings-create = Create from current
app-settings-create-hint = Snapshot the active colours into an editable theme
app-settings-save = Save
app-settings-save-hint = Write the config file
app-settings-unsaved = unsaved changes
app-settings-builtin-hint = Built-in themes are read-only — create a copy above to edit colours.
theme-group-chrome = Chrome & surfaces
theme-group-accents = Accents
theme-group-text = Text
theme-group-status = Status
theme-group-canvas = Canvas
theme-group-other = Other

# ── Tool database ───────────────────────────────────────────────────────────────────────────────────────
btn-tools = Tools
tip-tools = Manage the tool library
tool-db-title = Tool Database
tool-db-empty = No tools yet. Add one to build your library.
tool-db-add = Add tool
tool-db-remove = Remove tool
tool-db-save-hint = Write the tool library to disk
tool-db-select = Select a tool to edit it.
tool-db-name = Name
tool-db-diameter = Diameter (mm)
tool-db-iso-defaults = Isolation defaults
tool-db-drill-defaults = Drill defaults
tool-db-new-name = new tool
btn-seed-tool = Seed from tool
seed-tool-hint = pick a tool…
