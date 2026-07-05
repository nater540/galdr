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
op-load-project = Open project

# ── Dock ────────────────────────────────────────────────────────────────────────────────────────────────
dock-log = Log
dock-gcode = G-code
gcode-empty = Select a CNC job to preview its G-code.
log-ready = Ready.

# ── Canvas ──────────────────────────────────────────────────────────────────────────────────────────────
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
