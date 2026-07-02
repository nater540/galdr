# Skirnir UI strings — en-US (the source/reference locale and the fallback).
#
# This file is embedded into the binary via `include_str!` (see `i18n::EN_US`) and seeded by `i18n::init`.
# To add a string: add a `key = value` line here, then translate it in every sibling locale `.ftl`. To add a
# locale: drop a `<lang>.ftl` next to this file, embed it, and register it in `i18n::BUNDLED_LOCALES`.
#
# Interpolation uses Fluent placeables: `{ $name }` is replaced by the named argument passed through `tr!`.
#
# NOT translated (kept in the source strings deliberately): GCode/grbl protocol tokens (`$$`, `$X`, `G10 L2`,
# `ALARM:`, `error:`, `0x18`, `?`, `$ES`), unit strings (`mm`, `mm/min`, `RPM`, `°`, `s`), axis letters, the raw
# console echo of machine traffic, the CLI (`--cli`) diagnostic log, and the scrolling console activity-log
# notices (an operator-facing but log-shaped stream intermixed with untranslatable protocol echo).

app-title = Skirnir

# Top toolbar — the row of nav/action buttons at the very top of the window.
btn-connect = Connect
btn-disconnect = Disconnect
btn-cancel = Cancel
btn-identify = Identify
btn-open = Open…
btn-home = ⌂ Home
btn-settings = Settings
port-choose = Choose port…
tip-refresh-ports = Refresh ports
tip-identify = Probe the selected port for grblHAL (sends ?/$I)
tip-open = Load a G-code program
tip-home = Run homing cycle ($H)
tip-settings = Firmware settings ($$)
tip-app-settings = Application settings (language, theme, font scale)

# Program transport controls (segmented Run/Hold/Stop group plus the standalone Abort and Simulate).
transport-run = ▶ Run
transport-running = ▶ Running
transport-resume = ▶ Resume
transport-hold = ⏸ Hold
transport-stop = ■ Stop
transport-abort = ⏹ Abort
transport-simulate = ≈ Simulate
tip-hold = Feed hold (!)
tip-stop = Stop the job cleanly (0x86) — decelerate, flush, return to Idle
tip-abort = Emergency hard reset (0x18) — aborts to ALARM and resets the controller
tip-simulate = Estimate job time from the machine settings (no motion — host-only)

# Machine-state badge (toolbar + status bar). Uppercase per the design: the label is the primary signal.
badge-disconnected = DISCONNECTED
badge-connecting = CONNECTING
badge-idle = IDLE
badge-run = RUN
badge-jog = JOG
badge-hold = HOLD
badge-home = HOMING
badge-door = DOOR
badge-check = CHECK
badge-sleep = SLEEP
badge-tool = TOOL CHANGE
badge-alarm = ALARM
badge-error = ERROR

# Digital readout (DRO) panel.
hdr-dro = Digital Readout
lbl-limits = LIMITS
lbl-tool = TOOL
dro-tool-none = none
tip-limit-asserted = Limit switch asserted
tip-limit-clear = Limit clear
btn-zero-xyz = Zero XYZ

# Jog panel.
hdr-jog = Jog
jog-esc-cancel = esc · cancel
tip-jog-cancel = Jog cancel (0x85)
jog-step-label = Step (mm · A°)
jog-cont = cont
tip-jog-cont = Continuous jog: hold a direction to move, release to stop

# Override panel.
hdr-overrides = Overrides
lbl-feed = Feed
lbl-spindle = Spindle
ov-rapid = Rapid { $pct }%
ov-realized-feed = Realized F
ov-realized-speed = Realized S

# Probe (Z touch-off) panel.
hdr-probe = Probe Z · no plate
probe-intro = No-touch-plate Z zero. Lower until continuity, set Z = 0.
lbl-depth = Depth
lbl-plate = Plate
btn-probe-z = Probe Z → set work-zero
msg-probing-awaiting = Probing… awaiting result
msg-probing-awaiting-dot = Probing… awaiting result.
probe-contact = Contact at [{ $coords }]
probe-work-z-set = Work-Z set.
probe-work-z-unchanged = Work-Z unchanged.
probe-failed = Probe failed: { $reason }

# Rotary center-finder wizard (DOC-11 §1.2).
hdr-rotary-center = Rotary center-finder
rotary-intro = Find the A centerline from a known-diameter dowel clamped concentric.
lbl-dowel-dia = Dowel ⌀
lbl-a-angle = A angle
lbl-side-probe-z = Side-probe Z
tip-side-probe-z = Machine-Z the side (X/Y) touches descend to — must lie within the dowel's Z-extent. A wrong value will crash into the part or miss the flank.
rotary-side-confirm = Side-probe Z { $z } mm is set for this dowel
btn-start-center = Start center-finder
btn-apply-saved-center = Apply saved center
rotary-step-enter-dowel = Jog to the −Y face approach, then probe.
btn-probe-y-left = Probe Y (left side)
rotary-step-ready-yright = Jog to the +Y face approach, then probe.
btn-probe-y-right = Probe Y (right side)
rotary-step-move-yc = Move to the Y center before probing the top.
btn-move-yc = Move to Y center
rotary-step-moved-yc = At the Y center. Probe the dowel top.
btn-probe-z-top = Probe Z (dowel top)
rotary-step-review = Center found. Write it to the active WCS (Y/Z only).
btn-write-center = Write center → WCS (G10 L2)

# Rotary bench-tuned parameters (collapsing section).
hdr-bench-params = Bench params
lbl-clearance-z = Clearance Z
tip-clearance-z = Machine-Z retract above the dowel before the A index (G53).
lbl-settle = Settle
tip-settle = Dwell after the A index so backlash/oscillation damps out before the probe.
tip-bench-feed = Probe feed for the G38.2 touch.
tip-bench-depth = How far the probe advances seeking contact before it gives up (alarms).

# Rotary captured readings + computed center.
rotary-reading-dowel = Dowel ⌀ { $dia } mm · A { $angle }°
rotary-reading-y-left = Y left  { $v }
rotary-reading-y-right = Y right { $v }
rotary-reading-y-center = Y center { $v }
rotary-reading-z-top = Z top   { $v }
rotary-reading-z-center = Z center { $v }

# Rotary Z-datum picker.
lbl-z0-datum = Work-Z0 datum
z-datum-axis = Axis centerline
z-datum-top = Top surface
z-datum-axis-desc = Z0 at the rotary axis (Z_top − D/2).
z-datum-top-desc = Z0 at the probed top surface (Z_top).
z-datum-g10 = G10 will set Z { $z }

# Verify / measure panel (DOC-11 §2): 180°-flip verify and runout report.
hdr-verify = Verify · measure
verify-intro = 180°-flip verify or N-angle runout, probing −Y.
lbl-start-a = Start A
lbl-runout-n = Runout N
btn-start-flip = Start 180°-flip verify
btn-start-runout = Start runout report
verify-title-flip = 180°-flip verify
verify-title-runout = Runout report
verify-title-generic = Verify
verify-ready = Jog the approach for touch { $current } of { $total }, then probe.
btn-probe-angle = Probe this angle
verify-flip-need-two = Flip verify needs two readings.
verify-residual = Residual eccentricity { $mm } mm.
verify-apply-desc = Apply shifts the active WCS origin on this axis by the residual.
btn-apply-correction = Apply correction → WCS (G10 L2)
verify-runout-result = TIR { $tir } mm · eccentricity { $ecc } mm ({ $count } pts)
verify-runout-readonly = Read-only — no offset written.
verify-runout-need-two = Runout needs at least two readings.

# Shared wizard status lines.
msg-aborted = Aborted: { $reason }
reason-cancelled = cancelled
btn-cancel-wizard = Cancel

# Bottom dock: tabs, collapse toggle, and the streaming progress readout.
tab-console = Console
tab-program = Program
tip-expand-dock = Expand dock
tip-collapse-dock = Collapse dock
eta-default-settings = (default settings)
eta-pauses = { $count ->
    [one] pauses at { $count } line
   *[other] pauses at { $count } lines
  }

# Console tab body.
console-auto-scroll = auto-scroll
console-verbose = verbose
btn-clear = Clear
btn-send = Send
mdi-hint-disconnected = connect to send commands

# Bottom status bar.
status-wco-set = WCO set
status-line = Ln { $acked } / { $total } · { $pct }%

# Alarm / stream-error banner.
banner-alarm = ⚠ ALARM:{ $code }
banner-error = ⚠ error:{ $code } — stream halted
btn-dismiss = Dismiss
btn-soft-reset = Soft reset
tip-soft-reset = Soft reset (0x18)
btn-unlock = Unlock $X
tip-unlock = Clear the alarm lock

# Manual tool-change banner.
banner-tool-with = 🔧 Tool change: insert T{ $tool }, then Resume
banner-tool-generic = 🔧 Tool change: insert the tool, then Resume
banner-tool-detail = The machine is paused for a manual tool change (M6). Insert the tool and press Resume (cycle-start) to continue.
tip-resume-tool = Resume after the tool change (cycle-start, ~)

# Firmware settings window ($$ / $ES).
settings-window-title = Settings
lbl-connection = Connection
lbl-baud = Baud
hdr-firmware-settings = Firmware settings
settings-stage-note = Edits stage locally — Save writes them. Some settings (e.g. $22 homing) apply on the next reset.
settings-save = Save
settings-save-n = Save ({ $count })
tip-settings-save = Write every staged setting to the controller
settings-refresh = Refresh ($$)
tip-settings-refresh = Fetch $$ values and $ES labels from the controller
settings-none = No settings loaded — Refresh to fetch the controller's $$ / $ES.
tip-setting-edit = Click to edit
setting-unit = Unit: { $unit }
setting-range = Range: { $min }..{ $max }
setting-range-min = Range: ≥ { $min }
setting-range-max = Range: ≤ { $max }
settings-discard-title = Discard unsaved changes?
settings-discard-body = Discard { $count } unsaved change(s)?
btn-discard = Discard
btn-keep-editing = Cancel (Keep editing)

# Application settings dialog — language/theme/font-scale (host-side appearance, distinct from firmware Settings).
app-settings-title = Application Settings
app-settings-language = Language
app-settings-theme = Theme
app-settings-font-scale = Font scale
app-settings-new-theme-hint = new theme name…
app-settings-create = Create from current
app-settings-create-hint = Snapshot the active colors into a new editable theme
app-settings-builtin-hint = Built-in themes are read-only — create a copy to customize its colors.
app-settings-save = Save to config.json
app-settings-save-hint = Write the current settings to the config file (rewrites the whole file)
app-settings-unsaved = unsaved changes

# Theme colour-editor group headers (rendered uppercase).
theme-group-accents = accents
theme-group-text = text
theme-group-states = machine states
theme-group-alarm = alarm surface
theme-group-console = console
theme-group-toolpath = toolpath
theme-group-chrome = chrome & surfaces
theme-group-other = other

# Streaming progress — interpolates the acknowledged line and the program length. (Not currently wired to a
# view, but kept as a ready string and covered by tests.)
stream-progress = Streaming line { $current } of { $total }

# Port enumeration result — a plural selector keyed on the discovered port count. (Ready string; test-covered.)
ports-found = { $count ->
    [0] No serial ports found
    [one] { $count } serial port found
   *[other] { $count } serial ports found
  }

# Recoverable error surfaced when a port cannot be opened. (Ready string; test-covered.)
error-port-open = Could not open port { $port }: { $reason }
