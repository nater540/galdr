# Skirnir UI strings — en-US (the source/reference locale and the fallback).
#
# This file is embedded into the binary via `include_str!` (see `i18n::EN_US`) and seeded by `i18n::init`.
# To add a string: add a `key = value` line here, then translate it in every sibling locale `.ftl`. To add a
# locale: drop a `<lang>.ftl` next to this file, embed it, and register it in `i18n::BUNDLED_LOCALES`.
#
# Interpolation uses Fluent placeables: `{ $name }` is replaced by the named argument passed through `tr!`.

app-title = Skirnir

# Top toolbar — the row of nav/action buttons at the very top of the window (wired to `tr!`).
btn-connect = Connect
btn-disconnect = Disconnect
btn-cancel = Cancel
btn-identify = Identify
btn-open = Open…
btn-home = ⌂ Home
btn-settings = Settings

# Program transport controls (segmented Run/Hold/Stop group — not yet wired to `tr!`).
btn-run = Run
btn-hold = Hold
btn-stop = Stop

# Connection / streaming lifecycle states surfaced in the header.
status-disconnected = Disconnected
status-connecting = Connecting
status-idle = Idle
status-streaming = Streaming
status-hold = Hold
status-alarm = Alarm

# Streaming progress — interpolates the acknowledged line and the program length.
stream-progress = Streaming line { $current } of { $total }

# Port enumeration result — a plural selector keyed on the discovered port count.
ports-found = { $count ->
    [0] No serial ports found
    [one] { $count } serial port found
   *[other] { $count } serial ports found
  }

# Recoverable error surfaced when a port cannot be opened.
error-port-open = Could not open port { $port }: { $reason }

# App settings dialog — language/theme/font-scale (host-side appearance, distinct from the firmware Settings).
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
