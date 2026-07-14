//! The egui views: thin render functions, one per panel of the default window.
//!
//! Each function is a pure render of [`ViewState`] (engine-derived) plus [`UiState`] (transient widget state
//! the shell owns), pushing [`Intent`]s into an [`IntentSink`] for the shell to act on. No view touches the
//! engine, the transport, or performs I/O — that policy lives in the shell. This keeps the immediate-mode
//! frame cheap and the only hard logic (already in the reducer) out of the view layer.

use eframe::egui::{self, Align, Color32, Layout, RichText, ScrollArea, Vec2};

use super::badge::{BadgeState, TransportGroup};
use super::intent::{Axis, Dir, Intent, IntentSink};
use super::metrics::Metrics;
use super::preview;
use super::settings_model::SettingRow;
use super::theme::Palette;
use super::view_state::{Banner, LogLine, LogSource, ViewState};
use crate::config::ToolpathStyle;
use crate::protocol::{ConnectionState, RealtimeCommand};
use crate::transport::ports::PortInfo;

mod widgets;
mod shell_panels;
mod toolbar;
mod dro;
mod jog;
mod overrides;
mod probe;
mod rotary_center;
mod datum_finder;
mod mesh_probe;
mod verify;
mod dock;
mod console;
mod status_bar;
mod toolpath;
mod settings;

pub(crate) use widgets::*;
pub(crate) use shell_panels::*;
pub(crate) use toolbar::*;
pub(crate) use dro::*;
pub(crate) use jog::*;
pub(crate) use overrides::*;
pub(crate) use probe::*;
pub(crate) use rotary_center::*;
pub(crate) use datum_finder::*;
pub(crate) use mesh_probe::*;
pub(crate) use verify::*;
pub(crate) use dock::*;
pub(crate) use console::*;
pub(crate) use status_bar::*;
pub(crate) use toolpath::*;
pub(crate) use settings::*;

/// The baud range the connection knob accepts and that a loaded profile's baud is held to. The ESP32-S3 native
/// USB ignores the value, but the host driver and the dropdown want a sane one; both the settings widget and
/// [`UiState::from_prefs`] use this single range so they cannot drift.
const BAUD_RANGE: std::ops::RangeInclusive<u32> = 9_600..=2_000_000;

/// The fallback baud when none is remembered or a loaded one is out of [`BAUD_RANGE`].
const DEFAULT_BAUD: u32 = 115_200;

/// How many clicks on the machine-state badge flip the hidden "fabulous" pride accent (the June easter egg).
/// Six — one per rainbow stripe — so it is reachable by a curious operator yet never triggered by an idle
/// double-click. The toggle policy lives in [`UiState::register_fabulous_click`].
const FABULOUS_CLICKS: u8 = 6;

/// Hold a baud to [`BAUD_RANGE`], falling back to [`DEFAULT_BAUD`] when it is out of range (the settings knob
/// clamps live edits, but a value loaded from a hand-edited/corrupt profile OR from the config's `default_baud`
/// bypasses that — `0` would fail the port open). The widget's own range still clamps subsequent edits. `pub(crate)`
/// so the shell can hold the config-supplied `default_baud` to the same range before it reaches `SerialTransport::open`.
pub(crate) fn sanitize_baud(baud: u32) -> u32 {
  if BAUD_RANGE.contains(&baud) { baud } else { DEFAULT_BAUD }
}

/// The resolved runtime appearance the views render against: the colour [`Palette`] (resolved from the config's
/// active theme) and the [`ToolpathStyle`] (resolved toolpath-render tuning). Threaded through [`UiState`] so the
/// ~15 view fns read `state.style.*` rather than baked-in constants — a config theme change swaps this in. `Default`
/// is the design's dark palette plus the legacy render constants, so a `Default`-built UI (and every test) renders
/// exactly as before the config existed.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct RuntimeStyle {
  /// The active colour palette (one field per design token).
  pub palette: Palette,
  /// The active toolpath-render tuning (stroke widths, arc step, grid, marker radius).
  pub toolpath: ToolpathStyle,
}

/// Transient widget state the shell owns across frames: selections, text fields, and tunables that belong to
/// the UI, not to the engine-derived [`ViewState`]. Kept here so the views read and mutate it directly while
/// the shell persists it.
#[derive(Debug, Clone)]
pub struct UiState {
  /// The resolved runtime appearance (palette + toolpath style) the views render with, resolved from the loaded
  /// config in `SkirnirApp::new` and re-resolved on a config reload. `Default` is the design dark palette + legacy
  /// render constants, so a test-built `UiState` renders identically to before the config existed.
  pub style: RuntimeStyle,
  /// Serial ports discovered by the last enumeration, shown in the connect dropdown. Structured so the row can
  /// surface USB product / «likely Galdr» hints and the list arrives cu-preferred and Galdr-ranked.
  pub ports: Vec<PortInfo>,
  /// The device path currently selected in the dropdown (the `cu.*` callout path the transport opens).
  pub selected_port: String,
  /// The baud rate to open with (the ESP32-S3 native USB ignores it, but the host driver wants a value).
  pub baud: u32,
  /// The path of the loaded program, for display.
  pub program_path: Option<String>,
  /// The currently loaded program lines, for the program dock and stream intent. Shared as an `Arc<[String]>`
  /// so handing it to the streaming engine never clones the whole file (see [`Self::set_program`]).
  pub program: std::sync::Arc<[String]>,
  /// The parsed toolpath, cached at program-load time. The toolpath viewport re-derives only the per-frame fit
  /// (scale/offset) from this; it never re-parses the program each frame. Invalidated by [`Self::set_program`].
  toolpath: Vec<Segment>,
  /// The model-space `(min, max)` bounds of [`Self::toolpath`], cached alongside it. `None` when empty.
  toolpath_bounds: Option<(Vec2, Vec2)>,
  /// The smoothed live tool-position marker, in toolpath **model space** (work-coordinate XY mm) — NOT screen
  /// space, so a viewport resize cannot corrupt the lerp. Eased one frame at a time toward the latest status
  /// sample (see [`super::preview::smooth_marker`]); `None` until the first live work position is acquired or
  /// while no live status exists. Reset by [`Self::set_program`].
  marker_pos: Option<Vec2>,
  /// The position trail: a LinuxCNC-AXIS-style "backplot" of where the tool has ACTUALLY been, in toolpath model
  /// space (work-coordinate XY mm), oldest first. Live work positions are appended ONLY while running and cutting
  /// (machine in Run with the tool below the work surface, live work-Z < 0), decimated by a step gate, and the trail
  /// is drawn as a polyline over the dim preview — so it shows the real cut path rather than projecting/colouring
  /// which planned segment the tool is on (both of which led the cutter — see the module doc). Each point carries the
  /// live cut DEPTH so the draw step can shade it. Capped at [`MAX_TRAIL_POINTS`] (rolling, oldest dropped) and
  /// cleared by [`Self::set_program`] and on each fresh run-entry.
  trail: std::collections::VecDeque<TrailPoint>,
  /// The loaded program's minimum (most negative) work-Z — the deepest programmed cut — scanned once at load. The
  /// cut trail shades each segment by depth job-relative against this: brightest at the surface, darkest at this
  /// minimum (see [`super::preview::depth_brightness`]). Zero when no program is loaded or it never cuts below the
  /// work zero, in which case the cut path takes the full base colour with no depth darkening.
  job_min_z: f32,
  /// The machine run state seen on the previous overlay frame, kept only to edge-detect the start of a run. The
  /// rising edge into `Run` (see [`entered_run`]) wipes the prior run's trail and drops the stale marker once, so a
  /// fresh job draws over the dim planned preview alone rather than under the last run's path. `None` until the first
  /// frame; reset by [`Self::set_program`] so a freshly loaded file's first Run is treated as a clean entry.
  prev_run_state: Option<crate::protocol::RunState>,
  /// Whether the previous overlay frame was a CUT frame (running with the tool engaged below the work surface).
  /// Tracks the pen-up/pen-down state across frames: a cut point appended after this was `false` (a lift, a non-Run
  /// frame, or the start of the trail) begins a fresh stroke, so the draw loop does not bridge a line across the
  /// travel between cuts. Set `false` on any non-cut frame, on run-entry, and by [`Self::set_program`].
  prev_frame_was_cut: bool,
  /// Whether height-map autoleveling is armed for the next stream/simulate. When set (and [`crate::profile::Profile::mesh`]
  /// holds a probed mesh), the shell height-corrects the loaded program through [`super::autolevel::correct_program`]
  /// before streaming; when clear, the source file streams verbatim. Toggled via [`super::intent::Intent::AutolevelToggle`],
  /// which also invalidates the shell's cached corrected program. Transient — not persisted.
  pub autolevel_enabled: bool,
  /// The correction knobs applied when autoleveling (currently just whether rapids are Z-corrected). Carried into
  /// [`super::autolevel::correct_program`]; a change invalidates the shell's corrected-program cache.
  pub autolevel_cfg: super::autolevel::CorrectionConfig,
  /// The jog step distance (mm) selected in the jog pad.
  pub jog_step: f64,
  /// Whether the jog pad is in continuous (press-and-hold) mode rather than fixed-step. In continuous mode a
  /// jog button held down issues a long `$J=` move and releasing it injects jog-cancel, so the operator drives
  /// the axis smoothly to position; the `cont` selector chip toggles this (design §03's `cont` step).
  pub jog_continuous: bool,
  /// The jog feed rate (mm/min).
  pub jog_feed: f64,
  /// The feed-override slider's feedback state: idle (mirror live), dragging (hold the operator's position so a
  /// status poll cannot yank it), or holding the committed target after release until the firmware's relative
  /// ramp converges onto it. Held here so the slider survives the immediate-mode frames of a drag and the
  /// post-release ramp without snapping back to the lagging `Ov:` value. See [`super::overrides::OverrideFeedback`].
  pub feed_override_drag: super::overrides::OverrideFeedback,
  /// The spindle-override slider's feedback state; see [`Self::feed_override_drag`].
  pub spindle_override_drag: super::overrides::OverrideFeedback,
  /// The manual-command input buffer in the console.
  pub console_input: String,
  /// The MDI recall history (↑/↓ in the command field steps through previously sent lines, shell-style). The
  /// navigation/dedupe/draft policy is the pure [`super::mdi::MdiHistory`]; the console body only feeds it key
  /// presses and submitted lines. Session-scoped — deliberately not persisted.
  pub mdi_history: super::mdi::MdiHistory,
  /// Probe depth (mm, travelled downward as a positive magnitude here; the shell negates it).
  pub probe_depth: f64,
  /// Probe feed rate (mm/min).
  pub probe_feed: f64,
  /// Measured touch-plate thickness (mm) used to set work-Z after a successful probe.
  pub plate_thickness: f64,
  /// The rotary center-finder's dowel/gauge diameter input (mm) used to start a run (`Z_c = Z_top − D/2`).
  pub rotary_dowel_diameter: f64,
  /// The rotary center-finder's index-angle input (degrees) every touch holds A at during a run.
  pub rotary_index_angle: f64,
  /// The rotary-safe touch's bench-tuned parameters edited in the wizard's advanced section (retract clearance,
  /// side-probe descent height, settle, feed, depth). Seeded from [`crate::profile::Prefs`] and carried into a
  /// run via [`Intent::RotaryCenterStart`]; these used to be hard-coded `RotaryProbeParams::default()`.
  pub rotary_bench: super::rotary_probe::RotaryProbeParams,
  /// Whether the operator has explicitly confirmed the side-probe Z is tuned for the mounted dowel this session.
  /// The side-probe Z ([`super::rotary_probe::RotaryProbeParams::side_probe_z`]) defaults to a conservative
  /// PLACEHOLDER that is setup-specific: an untuned value can drive a side touch into the part or miss the flank
  /// entirely (a crash risk, finding #13). Every center-finder run uses a side (Y) touch, so Start is gated on
  /// this acknowledgement — it is transient (never persisted), so each session must re-confirm the bench is right.
  pub rotary_side_probe_confirmed: bool,
  /// The datum finder's bench-tuned probe parameters (approach clearances, latch/probe feeds, tip diameter,
  /// corner slide offset), edited in the datum panel and carried into a run via [`Intent::DatumEdgeStart`] /
  /// [`Intent::DatumCornerStart`]. Seeded from [`crate::profile::Prefs`] and persisted with the profile.
  pub datum_bench: super::datum::ProbeParams,
  /// The datum panel's single-edge selection: which axis and approach direction a single-edge touch-off probes.
  /// Transient (a live UI selection, never persisted).
  pub datum_edge_axis: super::intent::Axis,
  /// The datum panel's single-edge approach direction. Transient.
  pub datum_edge_dir: super::intent::Dir,
  /// The datum panel's corner selection (one of the four rectangular corners, inside or outside). Transient.
  pub datum_corner: super::datum::Corner,
  /// The height-map panel's grid bounds `(min_x, min_y)` in work-mm. Transient inputs (the probed mesh persists).
  pub mesh_min: (f64, f64),
  /// The height-map panel's grid bounds `(max_x, max_y)` in work-mm.
  pub mesh_max: (f64, f64),
  /// The height-map panel's target grid spacing (mm) — the point counts derive from it (`ceil(range/spacing)+1`).
  pub mesh_spacing: f64,
  /// The height-map panel's bench-tuned grid-probe params (clearance/feeds/latch/offsets). Seeded from defaults;
  /// carried into a run via [`super::intent::Intent::MeshProbeStart`].
  pub mesh_bench: super::autolevel::GridProbeParams,
  /// The Phase 2 verify/measure starting A angle (degrees): θ for the flip-verify pair, and the first runout angle.
  pub verify_start_angle: f64,
  /// The Phase 2 runout report's number of evenly-spaced angles (N ≥ 2).
  pub verify_runout_n: usize,
  /// Whether the FIRMWARE settings window (`$$`) is open.
  pub settings_open: bool,
  /// Whether the APP settings dialog (language/theme/font scale — [`super::app_settings`]) is open.
  pub app_settings_open: bool,
  /// Which setup/probing dialog is currently open (the right column's compact menu launches these as separate
  /// floating windows instead of a long inline scroll). `None` when none is open. A running probing flow forces
  /// its dialog open regardless (see [`forced_setup_dialog`]).
  pub setup_dialog: Option<SetupDialog>,
  /// The in-progress name for a new user theme in the app settings dialog, kept across frames while typing.
  pub theme_name_draft: String,
  /// The setting currently being edited in the panel, as `(number, edit_buffer)`, or `None` when no row is in
  /// edit mode. Held here so the in-progress text survives the immediate-mode frames of an edit; leaving the
  /// field (Enter or focus-loss) stages the value into [`Self::settings_staging`] and clears this, so a value is
  /// never lost the way it was when only Enter committed.
  pub editing_setting: Option<(u32, String)>,
  /// The locally-staged firmware-settings edits awaiting an explicit Save. Editing any value stages it here
  /// (on Enter or focus-loss) rather than writing immediately, so a typed value is never silently dropped; Save
  /// flushes the lot and clears it, a confirmed Discard clears it without writing. A row is shown "modified"
  /// exactly while it has an entry here.
  pub settings_staging: super::settings_staging::SettingsStaging,
  /// A settings dialog action (Refresh or Close) the operator requested while edits were still staged, parked
  /// here until they answer the "Discard N unsaved change(s)?" confirmation. `None` when no confirmation is
  /// pending; the modal is shown exactly while this is `Some`.
  pub pending_settings_action: Option<PendingSettingsAction>,
  /// DRO coordinate toggle: `true` shows machine position emphasised, `false` shows work position (the design
  /// default — WPos is the active toggle in the mock).
  pub show_machine_pos: bool,
  /// Whether the console auto-scrolls to the newest line (the design's "auto-scroll" checkbox).
  pub auto_scroll: bool,
  /// Whether the console shows every received line. When `false` (the default), bare `ok` acknowledgements are
  /// hidden so continuous jogging — which acks each `$J=` line — does not bury the log in `‹ ok` noise.
  pub verbose: bool,
  /// The curated per-setting tooltip descriptions, loaded once at startup from the runtime JSON file (seeded from
  /// the bundled default) and parsed into a number→text map. Held here so the settings tooltip is a pure render of
  /// already-parsed state — the file is never re-read or re-parsed per frame. An absent number degrades to no prose.
  pub setting_descriptions: super::setting_help::SettingDescriptions,
  /// The active tab in the bottom dock: Console or Program (design §03 — the two tabs share one dock surface).
  pub active_tab: DockTab,
  /// The program line the listing last auto-scrolled to follow, so the Program tab only nudges the view when the
  /// executing line actually moves — not on every frame, which would fight an operator who has scrolled back to
  /// read an earlier line. `None` until the first followed line; cleared when a new program is loaded.
  pub program_followed_line: Option<usize>,
  /// Whether the bottom dock is collapsed to just its tab strip, hiding the console/program body so the toolpath
  /// and panels reclaim the space. Defaults to expanded.
  pub dock_collapsed: bool,
  /// The central viewport/console split (an egui_tiles tree), built lazily on the first expanded frame from
  /// [`Self::dock_fraction`] and held here so the operator's divider drag survives frames and collapse cycles.
  pub central_split: Option<super::dock_tiles::CentralSplit>,
  /// The dock's share of the central region (`0..1`), mirrored out of the live split each frame so the profile
  /// can persist it and a fresh split (or next launch) reopens at the operator's chosen ratio.
  pub dock_fraction: f32,
  /// Whether "fabulous" mode — the hidden Pride-month accent — is on, painting a thin rainbow band along the
  /// toolbar and status bar. A pure cosmetic flourish that never touches the machine; off by default and
  /// transient (not persisted), toggled by the [`Self::register_fabulous_click`] badge easter egg.
  pub fabulous: bool,
  /// Running count of state-badge clicks toward the next fabulous-mode toggle, reset each time the threshold
  /// ([`FABULOUS_CLICKS`]) is reached. Transient bookkeeping for the easter egg; see [`Self::register_fabulous_click`].
  pub fabulous_click_streak: u8,
}

/// A settings dialog action the operator requested while edits were still staged, deferred behind the discard
/// confirmation. Both actions throw away the staged edits when confirmed; they differ only in what happens next.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PendingSettingsAction {
  /// Refresh ($$): re-fetch the controller's settings, which would overwrite the staged values with live ones.
  Refresh,
  /// Close the settings dialog, abandoning the staged edits.
  Close,
}

/// The caveats that ride beside the dock ETA when a physics-based simulation drives it. A simulation grounded in
/// the firmware's default machine model (no `$$` snapshot was loaded) is flagged so its figure is not mistaken for
/// one grounded in the real board config; modeled operator pauses (`M0`/`M1`/`M6`) are surfaced as a count since
/// their waits are unbounded and therefore excluded from the timed total. `None` (no simulation) renders nothing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct EtaQualifier {
  /// Whether the simulation fell back to default machine settings because no live `$$` snapshot was available.
  pub default_settings: bool,
  /// How many operator-pause lines (`M0`/`M1`/`M6`) the timeline modeled — unbounded waits not in the total.
  pub pauses: usize,
}

/// The two tabs hosted by the bottom dock (design §03). The dock is a single surface whose body switches
/// between the rolling console and the loaded-program listing; this selects which one is shown.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DockTab {
  /// The rolling, colour-tagged serial console plus the manual command line (the design default).
  #[default]
  Console,
  /// The loaded G-code program listing with the executing line highlighted.
  Program,
}

/// The setup/probing dialogs launched from the right column's compact "Setup &amp; probing" menu (the panel
/// declutter — these were a long inline scroll of six stacked sections). Each is a separate floating window and
/// only one is open at a time. A running probing flow (rotary center-finder, datum finder, height-map,
/// verify/measure) FORCES its dialog open (see [`forced_setup_dialog`]) so an in-progress wizard — which is
/// moving the machine — can never be hidden behind a closed window.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SetupDialog {
  /// The Z touch-off / probe panel ([`probe`]).
  Probe,
  /// The rotary center-finder wizard ([`rotary_center`]).
  RotaryCenter,
  /// The datum finder — edge / corner / Z surface ([`datum_finder`]).
  Datum,
  /// The height-map (mesh) acquisition panel ([`mesh_probe`]).
  MeshProbe,
  /// The Phase 2 verify / measure — flip-verify + runout ([`verify_measure`]).
  VerifyMeasure,
}

/// The setup dialog a running probing flow FORCES visible, so an in-progress wizard can never be hidden behind a
/// closed window (these flows move the machine — safety). At most one probing flow runs at a time in practice;
/// the priority here is fixed and deterministic only as a tie-break. Returns `None` when nothing is running,
/// leaving the operator's own open/closed choice untouched.
fn forced_setup_dialog(rotary: bool, datum: bool, mesh: bool, sweep: bool) -> Option<SetupDialog> {
  if rotary {
    Some(SetupDialog::RotaryCenter)
  } else if datum {
    Some(SetupDialog::Datum)
  } else if mesh {
    Some(SetupDialog::MeshProbe)
  } else if sweep {
    Some(SetupDialog::VerifyMeasure)
  } else {
    None
  }
}

/// Toggle logic for a setup-menu button: clicking the entry for the already-open dialog closes it, otherwise it
/// switches to that dialog (only one setup dialog is open at a time). A running flow's force
/// ([`forced_setup_dialog`]) is applied separately and overrides this.
fn toggle_setup_dialog(current: Option<SetupDialog>, clicked: SetupDialog) -> Option<SetupDialog> {
  if current == Some(clicked) { None } else { Some(clicked) }
}

impl Default for UiState {
  fn default() -> Self {
    UiState {
      style: RuntimeStyle::default(),
      ports: Vec::new(),
      selected_port: String::new(),
      baud: DEFAULT_BAUD,
      program_path: None,
      program: std::sync::Arc::from([] as [String; 0]),
      toolpath: Vec::new(),
      toolpath_bounds: None,
      marker_pos: None,
      trail: std::collections::VecDeque::new(),
      job_min_z: 0.0,
      prev_run_state: None,
      prev_frame_was_cut: false,
      autolevel_enabled: false,
      autolevel_cfg: super::autolevel::CorrectionConfig::default(),
      jog_step: 1.0,
      jog_continuous: false,
      jog_feed: 500.0,
      feed_override_drag: super::overrides::OverrideFeedback::default(),
      spindle_override_drag: super::overrides::OverrideFeedback::default(),
      console_input: String::new(),
      mdi_history: super::mdi::MdiHistory::default(),
      probe_depth: 10.0,
      probe_feed: 50.0,
      plate_thickness: 1.0,
      rotary_dowel_diameter: 6.0,
      rotary_index_angle: 0.0,
      rotary_bench: super::rotary_probe::RotaryProbeParams::default(),
      rotary_side_probe_confirmed: false,
      datum_bench: super::datum::ProbeParams::default(),
      datum_edge_axis: super::intent::Axis::X,
      datum_edge_dir: super::intent::Dir::Pos,
      datum_corner: super::datum::Corner::A,
      mesh_min: (0.0, 0.0),
      mesh_max: (100.0, 100.0),
      mesh_spacing: 10.0,
      mesh_bench: super::autolevel::GridProbeParams::default(),
      verify_start_angle: 0.0,
      verify_runout_n: 4,
      settings_open: false,
      app_settings_open: false,
      setup_dialog: None,
      theme_name_draft: String::new(),
      editing_setting: None,
      settings_staging: super::settings_staging::SettingsStaging::new(),
      pending_settings_action: None,
      show_machine_pos: false,
      auto_scroll: true,
      verbose: false,
      // Default to the bundled descriptions so a `Default`-built UI (and every test that uses one) has working
      // tooltips without touching disk; the real app replaces this with the loaded set in `SkirnirApp::new`.
      setting_descriptions: super::setting_help::SettingDescriptions::bundled(),
      active_tab: DockTab::default(),
      dock_collapsed: false,
      central_split: None,
      dock_fraction: super::dock_tiles::DEFAULT_DOCK_FRACTION,
      program_followed_line: None,
      fabulous: false,
      fabulous_click_streak: 0,
    }
  }
}

impl UiState {
  /// Build the transient widget state from the persisted [`crate::profile::Prefs`], so the connect dropdown and
  /// the rotary inputs come up pre-filled from last session. Only the genuinely "remember my last entry" fields
  /// are seeded (port, baud, rotary input defaults); everything else stays at the [`Default`] value — transient
  /// runtime state is never restored from a profile.
  pub fn from_prefs(prefs: &crate::profile::Prefs) -> Self {
    UiState {
      selected_port: prefs.last_port.clone().unwrap_or_default(),
      // Hold a hand-edited or corrupted-but-still-valid baud (e.g. `0`) to the accepted range; an out-of-range
      // value would otherwise reach `connect` unclamped and fail the port open with no recovery.
      baud: sanitize_baud(prefs.baud),
      rotary_dowel_diameter: prefs.rotary_dowel_diameter,
      rotary_index_angle: prefs.rotary_index_angle,
      rotary_bench: prefs.rotary_bench,
      datum_bench: prefs.datum_bench,
      mesh_bench: prefs.grid_bench,
      dock_fraction: prefs.dock_fraction,
      ..UiState::default()
    }
  }

  /// Register one click on the machine-state badge toward the hidden "fabulous" Pride accent. Every
  /// [`FABULOUS_CLICKS`]th click flips [`Self::fabulous`] and resets the streak; the rainbow band that appears is
  /// the only feedback. Kept egui-free so the easter-egg toggle policy is unit-tested without a window. Returns
  /// whether this click toggled the mode.
  pub fn register_fabulous_click(&mut self) -> bool {
    self.fabulous_click_streak = self.fabulous_click_streak.saturating_add(1);
    if self.fabulous_click_streak >= FABULOUS_CLICKS {
      self.fabulous = !self.fabulous;
      self.fabulous_click_streak = 0;
      return true;
    }
    false
  }

  /// Clear the connection-scoped transient widget state when the link drops. An in-progress setting edit and
  /// the override sliders' drag positions belong to the session that just ended: on a reconnect to a (possibly
  /// different) board they must not resume editing a stale `$<n>` row or pin a slider to the previous board's
  /// override. Mirrors `ViewState::on_disconnected` for the transient state the reducer cannot reach.
  pub fn on_disconnected(&mut self) {
    self.editing_setting = None;
    // Staged but unsaved settings edits belong to the board that just dropped; a reconnect (possibly to a
    // different controller) must not resume them or pop a stale discard confirmation.
    self.settings_staging.clear();
    self.pending_settings_action = None;
    self.feed_override_drag = super::overrides::OverrideFeedback::default();
    self.spindle_override_drag = super::overrides::OverrideFeedback::default();
  }

  /// Load a program: store its lines (shared, so streaming never re-clones the file) and rebuild the cached
  /// toolpath + bounds once, here, rather than on every frame. `path` is the display name, if any.
  pub fn set_program(&mut self, lines: Vec<String>, path: Option<String>) {
    self.program = std::sync::Arc::from(lines);
    self.program_path = path;
    // Flatten arcs at the config-resolved chord step so the preview density follows `toolpath.arc_step_deg`. Parsed
    // once here (cached), not per frame.
    self.toolpath = parse_xy_path(&self.program, self.style.toolpath.arc_step_rad);
    self.toolpath_bounds = toolpath_bounds(&self.toolpath);
    self.job_min_z = program_min_z(&self.program);
    // A fresh program starts unfollowed: the first streamed line of the new file must re-trigger an auto-scroll
    // even if the old program happened to leave us followed at the same row index.
    self.program_followed_line = None;
    // The live marker and the cut trail are program-scoped: a new file starts with no acquired marker (so it snaps to
    // the first sample of the new run) and an empty trail (so the new job draws over the dim planned preview alone,
    // not under the old program's path).
    self.marker_pos = None;
    self.trail.clear();
    // Forget the prior file's run state so the new program's first Run frame is treated as a clean run-entry rather
    // than a continuation (a stale `Some(Run)` here would suppress the entry-clear on the next run). Lift the
    // pen-down tracker too, so the new file's first cut begins a fresh stroke.
    self.prev_run_state = None;
    self.prev_frame_was_cut = false;
  }

  /// Re-flatten the CURRENTLY LOADED program's cached toolpath at the current `style.toolpath.arc_step_rad`, without
  /// disturbing the program lines or the live marker. Used after a config reload changes
  /// `toolpath.arc_step_deg`: the cached arc-chord density would otherwise stay stale until the GCode is reloaded
  /// (the reported F5 bug). A no-op when no program is loaded. Unlike [`Self::set_program`] this preserves the
  /// program-scoped overlay state, since the program itself is unchanged — only the render density is recomputed.
  pub fn reflow_toolpath(&mut self) {
    if self.program.is_empty() {
      return;
    }
    self.toolpath = parse_xy_path(&self.program, self.style.toolpath.arc_step_rad);
    self.toolpath_bounds = toolpath_bounds(&self.toolpath);
    // The cut trail is a polyline of real tool positions in model space — independent of the planned-segment chord
    // density — so a re-flow leaves it untouched: only the dim planned geometry is rebuilt.
  }

  /// The loaded program's XY extents `((min_x, min_y), (max_x, max_y))` in work-mm, or `None` when no program is
  /// loaded (or it has no XY geometry). The height-map panel's "auto from program" button seeds the grid bounds
  /// from this — the toolpath-bounds scan already computed at load, exposed as `f64` tuples for the mesh setup.
  pub fn program_xy_bounds(&self) -> Option<((f64, f64), (f64, f64))> {
    let (min, max) = self.toolpath_bounds?;
    Some(((min.x as f64, min.y as f64), (max.x as f64, max.y as f64)))
  }

  /// The number of cached toolpath segments (chords). Exposed so the reload/arc-density behaviour can be asserted
  /// without reaching into private fields; the count rises as arcs are flattened more finely.
  pub fn toolpath_segment_count(&self) -> usize {
    self.toolpath.len()
  }
}

/// Everything the shell threads into the window-panel grid beyond the view/ui state: the dock's stream clock and
/// ETA qualifier, plus the right column's wizard/sweep borrows. The whole-window test harness passes
/// [`ShellPanelsData::bare`] (a fixture clock, no wizard state), so the app and the harness drive the SAME
/// [`shell_panels`] layout and cannot drift.
pub struct ShellPanelsData<'a> {
  /// The stream's elapsed/ETA estimate the dock clock renders.
  pub time: super::progress::TimeEstimate,
  /// The "(default settings)" / pause-count caveats riding beside the dock ETA when a simulation drives it.
  pub eta_qualifier: Option<EtaQualifier>,
  /// The running rotary center-finder's pure state, if one is active.
  pub wizard: Option<&'a super::rotary_center::WizardState>,
  /// Whether a rotary center was persisted in the profile (the no-run panel offers a one-click re-apply).
  pub has_saved_center: bool,
  /// The running datum-finder's pure state, if one is active.
  pub datum: Option<&'a super::datum::DatumState>,
  /// The running height-map acquisition's pure state, if one is active.
  pub mesh_probe: Option<&'a super::autolevel::MeshProbeState>,
  /// Whether a height-map is persisted in the profile (the panel offers a one-click clear).
  pub has_saved_mesh: bool,
  /// The running Phase 2 sweep and which wizard owns it, if one is active.
  pub sweep: Option<(&'a super::angle_sweep::AngleSweep, super::view_state::ProbeKind)>,
}

impl ShellPanelsData<'_> {
  /// The harness/fixture form: the given clock, no ETA qualifier, and no wizard/datum/mesh/sweep state.
  pub fn bare(time: super::progress::TimeEstimate) -> Self {
    ShellPanelsData {
      time,
      eta_qualifier: None,
      wizard: None,
      has_saved_center: false,
      datum: None,
      mesh_probe: None,
      has_saved_mesh: false,
      sweep: None,
    }
  }
}

/// Which setup/probing flows are currently RUNNING, threaded into the [`setup_menu`] so a button whose wizard is
/// in progress carries a live tag. A pure boolean snapshot of the shell's `Option` borrows.
#[derive(Clone, Copy, Default)]
pub struct SetupRunning {
  /// The rotary center-finder is mid-run.
  pub rotary: bool,
  /// The datum finder is mid-run.
  pub datum: bool,
  /// A height-map acquisition is mid-run.
  pub mesh: bool,
  /// A Phase 2 verify/measure sweep is mid-run.
  pub sweep: bool,
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn a_running_probing_flow_forces_its_setup_dialog_open() {
    // Nothing running: the operator's own open/closed choice is left untouched (no force).
    assert_eq!(forced_setup_dialog(false, false, false, false), None);
    // Each running flow forces exactly its own dialog visible so an in-progress wizard can never be hidden.
    assert_eq!(forced_setup_dialog(true, false, false, false), Some(SetupDialog::RotaryCenter));
    assert_eq!(forced_setup_dialog(false, true, false, false), Some(SetupDialog::Datum));
    assert_eq!(forced_setup_dialog(false, false, true, false), Some(SetupDialog::MeshProbe));
    assert_eq!(forced_setup_dialog(false, false, false, true), Some(SetupDialog::VerifyMeasure));
    // If more than one somehow reads active, the priority is fixed and deterministic (rotary wins).
    assert_eq!(forced_setup_dialog(true, true, true, true), Some(SetupDialog::RotaryCenter));
  }

  #[test]
  fn setup_menu_button_toggles_its_dialog_open_and_closed() {
    // From closed, a click opens that dialog.
    assert_eq!(toggle_setup_dialog(None, SetupDialog::Probe), Some(SetupDialog::Probe));
    // Clicking the entry for the already-open dialog closes it (a toggle).
    assert_eq!(toggle_setup_dialog(Some(SetupDialog::Probe), SetupDialog::Probe), None);
    // Clicking a different entry switches to it (only one setup dialog open at a time).
    assert_eq!(toggle_setup_dialog(Some(SetupDialog::Probe), SetupDialog::Datum), Some(SetupDialog::Datum));
  }

  #[test]
  fn eta_qualifier_text_joins_the_default_settings_flag_and_the_pause_count() {
    // The qualifier parts resolve through `tr!`, so seed the bundled en-US registry first (idempotent).
    let _lang = crate::i18n::lock_global_for_test();
    let _ = crate::i18n::init();
    // Nothing to qualify (real settings, no pauses): no text at all, so the dock shows the clock alone.
    assert_eq!(eta_qualifier_text(EtaQualifier { default_settings: false, pauses: 0 }), None);
    // Default settings only.
    assert_eq!(
      eta_qualifier_text(EtaQualifier { default_settings: true, pauses: 0 }).as_deref(),
      Some("(default settings)")
    );
    // A single pause reads "line"; multiple read "lines".
    assert_eq!(
      eta_qualifier_text(EtaQualifier { default_settings: false, pauses: 1 }).as_deref(),
      Some("pauses at 1 line")
    );
    assert_eq!(
      eta_qualifier_text(EtaQualifier { default_settings: false, pauses: 3 }).as_deref(),
      Some("pauses at 3 lines")
    );
    // Both caveats join with the dim middot the dock uses between fields.
    assert_eq!(
      eta_qualifier_text(EtaQualifier { default_settings: true, pauses: 2 }).as_deref(),
      Some("(default settings) · pauses at 2 lines")
    );
  }

  #[test]
  fn program_min_z_takes_the_deepest_absolute_z() {
    let program = vec![
      "G90".to_string(),             // absolute distance mode
      "G0 Z5".to_string(),           // a safe-height rapid above the work (positive Z, not a depth)
      "G1 X10 Y0 Z-0.5 F300".to_string(), // a shallow cut at −0.5
      "G1 X10 Y10 Z-2.0".to_string(),     // the deepest pass at −2.0
      "G1 Z-1.0".to_string(),        // a shallower pass at −1.0
      "; Z-99 in a comment".to_string(),  // comment-only line is ignored
    ];
    assert_eq!(program_min_z(&program), -2.0);
    // A program that never goes below the surface (XY-only, or only positive Z) has no negative depth → 0.0.
    assert_eq!(program_min_z(&["G0 X1 Y1".to_string(), "G1 Z3".to_string()]), 0.0);
  }

  #[test]
  fn program_min_z_honours_relative_distance_mode() {
    // In G91 (relative) the Z words are deltas from the running Z, not absolute targets, so the deepest reached Z is
    // the accumulation: 0 → −1 → −2.5 here, with the −2.5 being the minimum even though no single word is that deep.
    let program = vec![
      "G91".to_string(),     // relative distance mode
      "G1 Z-1.0 F100".to_string(), // Z: 0 → −1.0
      "G1 Z-1.5".to_string(),      // Z: −1.0 → −2.5 (the deepest)
      "G1 Z2.0".to_string(),       // Z: −2.5 → −0.5 (retract, shallower)
    ];
    assert_eq!(program_min_z(&program), -2.5);
  }

  #[test]
  fn parse_xy_path_emits_one_segment_per_motion_line_skipping_blanks_and_comments() {
    // Blank and comment lines emit no geometry; only the two motion lines do. The progress backplot is keyed off the
    // segment INDEX swept by the live tool (not a per-line tag), so the parser's contract here is simply: one segment
    // per executing XY move, in order.
    let program = vec![
      "G90".to_string(),            // modal only, emits no segment
      "".to_string(),               // blank — emits nothing
      "; a comment".to_string(),    // comment — emits nothing
      "G1 X10 Y0 Z-1".to_string(),  // the first cut segment
      "G1 X10 Y10".to_string(),     // the second cut segment
    ];
    let segs = parse_xy_path(&program, 0.1);
    assert_eq!(segs.len(), 2, "two motion lines emit two segments, blanks/comments emit none");
  }

  #[test]
  fn parse_xy_path_flattens_an_arc_into_many_chords() {
    // An arc (G2/G3) flattens into many short chord segments rather than a single start→end chord, so the dim planned
    // preview draws the real curve.
    let program = vec![
      "G90".to_string(),
      "G0 X1 Y0".to_string(),            // position to the arc start
      "G3 X0 Y1 I-1 J0".to_string(),     // a quarter arc → many chords
    ];
    let segs = parse_xy_path(&program, 0.05);
    // One positioning move plus several arc chords; the arc alone must contribute more than two chords.
    assert!(segs.len() > 3, "the arc flattens into several chords beyond the positioning move (got {})", segs.len());
  }

  #[test]
  fn span_diagonal_has_a_floor_so_tolerances_never_collapse() {
    // A normal span yields its Euclidean diagonal.
    let d = span_diagonal(egui::vec2(3.0, 4.0));
    assert!((d - 5.0).abs() < 1e-4, "3-4-5 diagonal: {d}");
    // A degenerate (point) program clamps to the floor so reach/snap distances stay positive.
    assert_eq!(span_diagonal(egui::vec2(0.0, 0.0)), 1.0, "a zero span must clamp to the floor");
  }

  #[test]
  fn ui_state_has_sane_defaults() {
    let state = UiState::default();
    assert_eq!(state.baud, DEFAULT_BAUD);
    assert_eq!(state.jog_step, 1.0);
    assert!(!state.jog_continuous, "the jog pad defaults to fixed-step, not continuous");
    assert!(state.program.is_empty());
    assert!(!state.settings_open);
    assert!(state.auto_scroll, "console auto-scrolls by default");
    assert!(!state.show_machine_pos, "the DRO defaults to work coordinates");
    assert_eq!(state.active_tab, DockTab::Console, "the dock opens on the Console tab");
    assert!(!state.dock_collapsed, "the dock opens expanded at its full height");
  }

  /// Build a `SettingRow` with no `$ES` metadata (value only) — a row that has arrived before its enumeration.
  fn row_without_meta(number: u32) -> SettingRow {
    SettingRow { number, value: Some("10".to_string()), meta: None }
  }

  /// Build a `SettingRow` carrying `$ES` metadata, for the with-metadata tooltip path.
  fn row_with_meta(number: u32, name: &str, unit: &str, min: Option<&str>, max: Option<&str>) -> SettingRow {
    SettingRow {
      number,
      value: Some("10".to_string()),
      meta: Some(crate::protocol::SettingMeta {
        number,
        group: 0,
        name: name.to_string(),
        unit: unit.to_string(),
        min: min.map(str::to_string),
        max: max.map(str::to_string),
      }),
    }
  }

  #[test]
  fn tooltip_meta_is_empty_without_es_metadata() {
    // A row that has a value but no `$ES` row yet carries no dynamic lines; the renderer still shows the bare
    // `$<n>` heading, so the tooltip is never an empty box even here.
    assert!(setting_tooltip_meta(&row_without_meta(0)).is_empty(), "no metadata → no dynamic lines");
  }

  #[test]
  fn tooltip_meta_lists_name_unit_and_full_range() {
    // The full case: name, unit, and a two-sided range all enumerated. The unit/range lines resolve through `tr!`,
    // so seed the bundled en-US registry first (idempotent) or they would come back as raw keys.
    let _lang = crate::i18n::lock_global_for_test();
    let _ = crate::i18n::init();
    let row = row_with_meta(110, "Max rate", "mm/min", Some("0"), Some("10000"));
    let lines = setting_tooltip_meta(&row);
    assert_eq!(lines, vec!["Max rate".to_string(), "Unit: mm/min".to_string(), "Range: 0..10000".to_string()]);
  }

  #[test]
  fn tooltip_meta_handles_a_one_sided_range_and_a_unitless_setting() {
    // Only a max advertised, and no unit (a unitless bitmask like a status-report mask): the unit line is omitted
    // and the range is shown one-sided. The range line resolves through `tr!`, so seed the registry first.
    let _lang = crate::i18n::lock_global_for_test();
    let _ = crate::i18n::init();
    let row = row_with_meta(10, "Report mask", "", None, Some("255"));
    let lines = setting_tooltip_meta(&row);
    assert_eq!(lines, vec!["Report mask".to_string(), "Range: ≤ 255".to_string()]);
  }

  #[test]
  fn from_prefs_holds_an_out_of_range_baud_to_the_default() {
    use crate::profile::Prefs;
    // A valid baud passes through untouched.
    let ok = UiState::from_prefs(&Prefs { baud: 250_000, ..Prefs::default() });
    assert_eq!(ok.baud, 250_000, "an in-range baud must be kept as-is");
    // A hand-edited/corrupt-but-valid-`u32` baud out of range (0, or absurdly high) falls back to the default
    // rather than reaching `connect` and failing the port open with no recovery.
    let zero = UiState::from_prefs(&Prefs { baud: 0, ..Prefs::default() });
    assert_eq!(zero.baud, DEFAULT_BAUD, "a zero baud must fall back to the default");
    let huge = UiState::from_prefs(&Prefs { baud: 9_000_000, ..Prefs::default() });
    assert_eq!(huge.baud, DEFAULT_BAUD, "a baud above the accepted range must fall back to the default");
  }

  #[test]
  fn fabulous_toggles_every_sixth_badge_click_and_is_off_by_default() {
    let mut ui = UiState::default();
    assert!(!ui.fabulous, "fabulous mode is off until the easter egg is found");
    // The first five clicks only accumulate the streak; nothing flips yet.
    for click in 1..FABULOUS_CLICKS {
      assert!(!ui.register_fabulous_click(), "click {click} must not toggle yet");
      assert!(!ui.fabulous);
    }
    // The sixth click flips it on and resets the streak.
    assert!(ui.register_fabulous_click(), "the sixth click toggles fabulous mode");
    assert!(ui.fabulous);
    assert_eq!(ui.fabulous_click_streak, 0, "the streak resets after a toggle");
    // Another six clicks turn it back off — the toggle is symmetric.
    for _ in 1..FABULOUS_CLICKS {
      assert!(!ui.register_fabulous_click());
    }
    assert!(ui.register_fabulous_click());
    assert!(!ui.fabulous, "a second six-click run turns fabulous mode back off");
  }

  #[test]
  fn program_follow_advances_only_while_streaming_and_on_a_new_line() {
    // Off: auto-scroll disabled never follows, even mid-stream.
    assert_eq!(program_follow_target(false, ConnectionState::Streaming, 5, 100, None), None);
    // Wrong state: not streaming (e.g. Idle/Hold) never follows even with auto-scroll on.
    assert_eq!(program_follow_target(true, ConnectionState::Idle, 5, 100, None), None);
    // First streamed line follows (no prior followed row).
    assert_eq!(program_follow_target(true, ConnectionState::Streaming, 0, 100, None), Some(0));
    // The line advanced past the last followed row → follow the new one.
    assert_eq!(program_follow_target(true, ConnectionState::Streaming, 6, 100, Some(5)), Some(6));
    // The line has not moved since we last followed it → do not re-scroll (leave a manual scroll-back alone).
    assert_eq!(program_follow_target(true, ConnectionState::Streaming, 5, 100, Some(5)), None);
  }

  #[test]
  fn program_follow_ignores_an_out_of_range_cursor() {
    // At end-of-program the acked count can equal the line count (cursor past the last index); never target a row
    // that does not exist, and never follow an empty program.
    assert_eq!(program_follow_target(true, ConnectionState::Streaming, 100, 100, Some(99)), None);
    assert_eq!(program_follow_target(true, ConnectionState::Streaming, 0, 0, None), None);
  }

  #[test]
  fn jog_sense_senses_drag_only_in_continuous_mode() {
    // egui's `Sense::click()` senses clicks but not drags; `click_and_drag()` senses both. The drag bit must
    // flip with the mode so the press-and-hold edges become observable only when continuous, while a click is
    // always sensed (a tap is a bounded jog in step mode and a quick nudge in continuous mode).
    let mut state = UiState::default();
    let step_sense = jog_sense(&state);
    assert!(step_sense.senses_click(), "fixed-step jog senses a click");
    assert!(!step_sense.senses_drag(), "fixed-step jog must not sense drag");
    state.jog_continuous = true;
    let cont_sense = jog_sense(&state);
    assert!(cont_sense.senses_drag(), "continuous jog must sense drag for press-and-hold");
    assert!(cont_sense.senses_click(), "continuous jog still senses a click for a quick tap");
  }

  #[test]
  fn disconnect_clears_transient_edit_and_drag_state() {
    // An in-progress setting edit, the staged settings edits, a pending discard confirmation, and the override
    // slider drags belong to the ended session; a disconnect must wipe them so a reconnect never resumes a stale
    // `$<n>` edit, write the old board's staged values, or pop a leftover confirmation.
    let mut staging = super::super::settings_staging::SettingsStaging::new();
    staging.stage(110, "250", Some("500"));
    let mut state = UiState {
      editing_setting: Some((110, "250".to_string())),
      settings_staging: staging,
      pending_settings_action: Some(PendingSettingsAction::Refresh),
      feed_override_drag: super::super::overrides::OverrideFeedback::Dragging(140),
      spindle_override_drag: super::super::overrides::OverrideFeedback::Holding {
        target: 90,
        committed_from: 100,
        observations: 0,
      },
      ..UiState::default()
    };
    state.on_disconnected();
    assert_eq!(state.editing_setting, None, "an in-progress setting edit must not survive a disconnect");
    assert!(state.settings_staging.is_empty(), "staged settings edits must not survive a disconnect");
    assert_eq!(state.pending_settings_action, None, "a pending discard confirmation must not survive a disconnect");
    let idle = super::super::overrides::OverrideFeedback::Idle;
    assert_eq!(state.feed_override_drag, idle, "the feed-override feedback must reset to Idle on a disconnect");
    assert_eq!(state.spindle_override_drag, idle, "the spindle-override feedback must reset to Idle on a disconnect");
  }

  #[test]
  fn dock_toggle_label_reflects_collapsed_state() {
    // Expanded: the toggle is a minimise control; collapsed: a restore control. Distinct glyphs so the operator
    // can tell the dock's state at a glance from the strip's right corner.
    assert_eq!(dock_toggle_label(false), "−");
    assert_eq!(dock_toggle_label(true), "+");
    assert_ne!(dock_toggle_label(false), dock_toggle_label(true));
  }

  #[test]
  fn dock_tab_for_click_maps_strip_indices_and_holds_on_no_click() {
    // The dock strip is `[Console, Program]`: index 0 selects Console, 1 selects Program.
    assert_eq!(dock_tab_for_click(DockTab::Program, Some(0)), DockTab::Console);
    assert_eq!(dock_tab_for_click(DockTab::Console, Some(1)), DockTab::Program);
    // No click leaves the current selection untouched, so reading one tab never flips to the other.
    assert_eq!(dock_tab_for_click(DockTab::Program, None), DockTab::Program);
    assert_eq!(dock_tab_for_click(DockTab::Console, None), DockTab::Console);
    // An out-of-range index (no such tab) is inert rather than a panic — defensive against a strip change.
    assert_eq!(dock_tab_for_click(DockTab::Console, Some(9)), DockTab::Console);
  }

  #[test]
  fn console_line_style_distinguishes_status_info_and_errors() {
    let palette = Palette::default_dark();
    // Sent and notice are fixed by source.
    assert_eq!(console_line_style(palette, LogSource::Sent, "$H").1, palette.log_sent);
    assert_eq!(console_line_style(palette, LogSource::Notice, "connect").1, palette.log_notice);
    // Received lines are typed by their leading glyph: status `<…>`, info `[…]`, error/alarm, else plain ok.
    assert_eq!(console_line_style(palette, LogSource::Received, "<Idle|MPos:0,0,0>").1, palette.log_status);
    assert_eq!(console_line_style(palette, LogSource::Received, "[MSG:hi]").1, palette.log_info);
    assert_eq!(console_line_style(palette, LogSource::Received, "error:9").1, palette.state_alarm);
    assert_eq!(console_line_style(palette, LogSource::Received, "ok").1, palette.log_recv);
  }

  #[test]
  fn should_submit_mdi_gates_on_connection_gesture_and_a_non_blank_line() {
    // The common path: connected, a submit gesture (Enter or Send), and a real line → submit.
    assert!(should_submit_mdi(true, true, "G0 X0"), "a non-blank line with a gesture while connected submits");
    assert!(should_submit_mdi(true, true, "$$"), "a `$` command submits like any other line");
    // No submit gesture this frame (just typing into the field) → no send.
    assert!(!should_submit_mdi(true, false, "G0 X0"), "without Enter/Send the line is not submitted");
    // Disconnected → never send, even with a gesture and a line (the field/button are also disabled in the UI).
    assert!(!should_submit_mdi(false, true, "G0 X0"), "a disconnected link never submits");
    // A blank or whitespace-only field is a no-op, so a stray Enter does not push an empty line.
    assert!(!should_submit_mdi(true, true, ""), "an empty field never submits");
    assert!(!should_submit_mdi(true, true, "   "), "a whitespace-only field never submits");
  }

  #[test]
  fn ok_noise_filters_only_received_acks() {
    let recv = |text: &str| LogLine { source: LogSource::Received, text: text.to_string() };
    assert!(is_ok_noise(&recv("ok")), "a bare received ok is noise");
    assert!(is_ok_noise(&recv("ok\r")), "trailing whitespace still counts as a bare ok");
    assert!(!is_ok_noise(&recv("error:9")), "errors are never filtered");
    assert!(!is_ok_noise(&recv("[MSG:ok]")), "an info line that merely contains ok stays");
    // A line the operator typed and we echoed is a Sent source, so it is never treated as ack noise.
    assert!(!is_ok_noise(&LogLine { source: LogSource::Sent, text: "ok".to_string() }));
  }

  #[test]
  fn leaving_a_settings_field_stages_unless_abandoned() {
    // Enter and plain focus-loss both stage (the silent-drop fix); only Escape (abandoned) discards. The value's
    // realness is handled separately by the staging store, so this is purely the stage-vs-abandon decision.
    assert!(setting_edit_should_stage(false), "leaving on Enter / focus-loss stages the edit");
    assert!(!setting_edit_should_stage(true), "Escape abandons — nothing is staged");
  }

  #[test]
  fn a_refresh_or_close_needs_confirmation_only_with_staged_edits() {
    // Nothing staged → Refresh/Close proceed with no dialog (the pre-existing behaviour).
    let mut staging = super::super::settings_staging::SettingsStaging::new();
    assert!(!settings_action_needs_confirm(&staging), "no staged edits → no discard confirmation");
    // A real staged edit → a refresh/close must confirm before throwing it away.
    staging.stage(0, "12", Some("10"));
    assert!(settings_action_needs_confirm(&staging), "staged edits gate a refresh/close behind the confirmation");
  }

  #[test]
  fn the_tool_change_headline_names_a_real_tool_and_falls_back_otherwise() {
    // The headline resolves through `tr!`, so seed the bundled en-US registry first (idempotent).
    let _lang = crate::i18n::lock_global_for_test();
    let _ = crate::i18n::init();
    // A real tool number is named so the operator knows which tool to fit.
    assert_eq!(tool_change_headline(Some(3)), "🔧 Tool change: insert T3, then Resume");
    // `T0` (no tool) and an unreported tool both use the generic prompt rather than naming a misleading number.
    assert_eq!(tool_change_headline(Some(0)), "🔧 Tool change: insert the tool, then Resume");
    assert_eq!(tool_change_headline(None), "🔧 Tool change: insert the tool, then Resume");
  }

  #[test]
  fn jog_is_enabled_only_in_idle_and_jog() {
    // grblHAL accepts `$J=` only in Idle and Jog.
    assert!(jog_enabled(BadgeState::Idle));
    assert!(jog_enabled(BadgeState::Jog));
    // Every other state rejects a jog, so the pad must be disabled.
    for state in [
      BadgeState::Disconnected,
      BadgeState::Connecting,
      BadgeState::Run,
      BadgeState::Hold,
      BadgeState::Home,
      BadgeState::Door,
      BadgeState::Check,
      BadgeState::Sleep,
      BadgeState::Tool,
      BadgeState::Alarm,
      BadgeState::Error,
    ] {
      assert!(!jog_enabled(state), "{state:?} must not enable the jog pad");
    }
  }

  #[test]
  fn toolpath_parser_tracks_motion_mode_and_position() {
    let program = vec![
      "G0 X0 Y0".to_string(),
      "G1 X10 Y0".to_string(),
      "G1 X10 Y10".to_string(),
      "; a comment line".to_string(),
      "G0 X0 Y0".to_string(),
    ];
    let segments = parse_xy_path(&program, super::preview::DEFAULT_ARC_STEP_RAD);
    // Four segments: the initial `G0 X0 Y0` carries motion words so it is a (zero-length) rapid; the two G1
    // cuts; then the final G0 travel back to origin. The comment line contributes nothing.
    assert_eq!(segments.len(), 4);
    assert!(segments[0].rapid, "leading G0 is a rapid");
    assert!(!segments[1].rapid, "first G1 is a cut");
    assert_eq!(segments[1].from, egui::vec2(0.0, 0.0));
    assert_eq!(segments[1].to, egui::vec2(10.0, 0.0));
    assert_eq!(segments[2].to, egui::vec2(10.0, 10.0));
    assert!(segments[3].rapid, "final G0 travel is rapid");
    assert_eq!(segments[3].to, egui::vec2(0.0, 0.0));
  }

  #[test]
  fn toolpath_parser_ignores_lines_without_xy_motion() {
    let program = vec!["M3 S1000".to_string(), "F500".to_string(), "G1 X5".to_string()];
    let segments = parse_xy_path(&program, super::preview::DEFAULT_ARC_STEP_RAD);
    assert_eq!(segments.len(), 1);
    assert_eq!(segments[0].to, egui::vec2(5.0, 0.0));
  }

  #[test]
  fn toolpath_parser_does_not_draw_segments_for_non_motion_g_codes() {
    // G10/G28/G30/G92/G53/G4 carry X/Y as parameters, not as a move — none must produce a segment, even though
    // the modal motion mode (G1, set first) is a cutting mode.
    let program = vec![
      "G1 X1 Y1".to_string(),         // a real move sets the modal motion mode and position
      "G10 L20 P0 X0 Y0".to_string(), // set work zero — not a move
      "G92 X5 Y5".to_string(),        // set coordinate offset — not a move
      "G53 X9 Y9".to_string(),        // machine-coord move modifier — skipped for the preview
      "G28 X0 Y0".to_string(),        // go-home via — not a normal segment
      "G4 P0.5".to_string(),          // dwell — no XY anyway
    ];
    let segments = parse_xy_path(&program, super::preview::DEFAULT_ARC_STEP_RAD);
    // Only the leading G1 is a segment; every non-motion line is suppressed.
    assert_eq!(segments.len(), 1, "non-motion G-codes must not draw segments");
    assert_eq!(segments[0].to, egui::vec2(1.0, 1.0));
  }

  #[test]
  fn toolpath_parser_handles_compact_spaceless_gcode() {
    // Standard CAM post output packs words with no spaces: `G1X10.0Y5.0`.
    let program = vec!["G0X0Y0".to_string(), "G1X10.0Y5.0".to_string(), "X20.5".to_string()];
    let segments = parse_xy_path(&program, super::preview::DEFAULT_ARC_STEP_RAD);
    assert_eq!(segments.len(), 3);
    assert!(segments[0].rapid, "G0 is a rapid");
    assert!(!segments[1].rapid, "G1 is a cut");
    assert_eq!(segments[1].to, egui::vec2(10.0, 5.0));
    // The bare `X20.5` continues the modal G1 and updates only X.
    assert!(!segments[2].rapid);
    assert_eq!(segments[2].to, egui::vec2(20.5, 5.0));
  }

  #[test]
  fn toolpath_parser_flattens_a_g2_arc_into_many_chords() {
    // A G2 (CW) quarter arc from (1,0) to (0,1) about the origin (I-1 J0 → centre = start + (−1,0) = (0,0)) must
    // flatten into many short chord segments on the unit radius, not a single start→end chord — so the preview
    // draws the curve.
    let program = vec!["G0 X1 Y0".to_string(), "G2 X0 Y1 I-1 J0".to_string()];
    let segments = parse_xy_path(&program, super::preview::DEFAULT_ARC_STEP_RAD);
    // One rapid to the start, then several arc chords (a quarter circle subdivides into multiple segments).
    let arc: Vec<&Segment> = segments.iter().filter(|s| !s.rapid).collect();
    assert!(arc.len() >= 3, "the arc must flatten into several chords, got {}", arc.len());
    // Every arc-chord endpoint sits on the unit radius, and the chain is continuous (each from == previous to).
    for w in arc.windows(2) {
      assert_eq!(w[0].to, w[1].from, "the flattened chords must be continuous");
    }
    for s in &arc {
      let r = (s.to.x * s.to.x + s.to.y * s.to.y).sqrt();
      assert!((r - 1.0).abs() < 1e-2, "every chord endpoint sits on the radius: {:?} r={r}", s.to);
    }
    // The final chord ends exactly on the commanded endpoint.
    assert_eq!(arc.last().unwrap().to, egui::vec2(0.0, 1.0));
  }

  #[test]
  fn toolpath_parser_degrades_an_arc_without_ij_to_a_single_chord() {
    // An arc with no I/J (e.g. an R-form arc, which this preview does not resolve) must not break the parse — it
    // degrades to a single straight chord to the endpoint rather than guessing a centre or panicking.
    let program = vec!["G0 X1 Y0".to_string(), "G3 X0 Y1".to_string()];
    let segments = parse_xy_path(&program, super::preview::DEFAULT_ARC_STEP_RAD);
    let arc: Vec<&Segment> = segments.iter().filter(|s| !s.rapid).collect();
    assert_eq!(arc.len(), 1, "an arc with no centre offset degrades to one chord");
    assert_eq!(arc[0].from, egui::vec2(1.0, 0.0));
    assert_eq!(arc[0].to, egui::vec2(0.0, 1.0));
  }

  #[test]
  fn toolpath_parser_respects_relative_distance_mode() {
    // Under G91 the X/Y words are offsets from the current position, not absolute targets.
    let program = vec![
      "G90 G0 X10 Y10".to_string(), // absolute: move to (10,10)
      "G91".to_string(),            // switch to relative
      "G1 X5 Y0".to_string(),       // +5 in X -> (15,10)
      "G1 X0 Y-3".to_string(),      // -3 in Y -> (15,7)
      "G90 X0 Y0".to_string(),      // back to absolute and to the origin
    ];
    let segments = parse_xy_path(&program, super::preview::DEFAULT_ARC_STEP_RAD);
    assert_eq!(segments.len(), 4);
    assert_eq!(segments[0].to, egui::vec2(10.0, 10.0));
    assert_eq!(segments[1].to, egui::vec2(15.0, 10.0));
    assert_eq!(segments[2].to, egui::vec2(15.0, 7.0));
    assert_eq!(segments[3].to, egui::vec2(0.0, 0.0));
  }

  #[test]
  fn is_moving_state_is_run_jog_and_hold_only() {
    use crate::protocol::RunState;
    for s in [RunState::Run, RunState::Jog, RunState::Hold] {
      assert!(is_moving_state(Some(s)), "{s:?} is an active-motion state");
    }
    for s in [RunState::Idle, RunState::Alarm, RunState::Door, RunState::Check, RunState::Home, RunState::Sleep,
      RunState::Tool, RunState::Unknown]
    {
      assert!(!is_moving_state(Some(s)), "{s:?} must not count as moving");
    }
    assert!(!is_moving_state(None), "no report yet is not moving");
  }

  /// Build a `(UiState, ViewState)` pair with a small square program loaded, for the overlay-folding tests.
  fn overlay_fixture() -> (UiState, ViewState) {
    use crate::protocol::Response;
    let mut state = UiState::default();
    state.set_program(vec![
      "G0 X0 Y0".to_string(),
      "G1 X10 Y0".to_string(),
      "G1 X10 Y10".to_string(),
      "G1 X0 Y10".to_string(),
      "G1 X0 Y0".to_string(),
    ], None);
    let mut view = ViewState::default();
    // A WCO so a later machine report is derivable; the program is authored at WCO origin so work == machine here.
    view.apply(crate::Event::Response(Response::Status("Run|MPos:0.000,0.000,0.000|WCO:0.000,0.000,0.000".into())));
    (state, view)
  }

  fn feed_status_into(view: &mut ViewState, body: &str) {
    use crate::protocol::Response;
    view.apply(crate::Event::Response(Response::Status(body.to_string())));
  }

  #[test]
  fn update_live_overlay_holds_the_marker_through_a_transient_missing_work_frame() {
    // Finding #6: grbl pushes WCO only intermittently, so a mid-run machine report can momentarily lack a
    // derivable work position. That frame must HOLD the last marker, not null it (which would re-snap/blink and
    // revert colouring to the acked fallback on the next valid frame).
    let (mut state, mut view) = overlay_fixture();
    let bounds = state.toolpath_bounds.expect("bounds");
    let span = bounds.1 - bounds.0;

    // A normal running frame acquires a marker mid-path.
    feed_status_into(&mut view, "Run|MPos:5.000,0.000,0.000|WCO:0.000,0.000,0.000");
    assert!(update_live_overlay(&mut state, &view, bounds, span).is_some(), "a valid frame draws the marker");
    let held = state.marker_pos.expect("a marker was acquired");

    // A transient frame: a status arrives but with no cached-WCO-derivable work position. Simulate by clearing
    // the cached WCO so the machine report is no longer derivable to work — the marker must be HELD, not cleared.
    view.last_wco.clear();
    feed_status_into(&mut view, "Run|MPos:6.000,0.000,0.000");
    assert!(view.work_xy().is_none(), "this frame genuinely lacks a derivable work position");
    let drawn = update_live_overlay(&mut state, &view, bounds, span);
    // The held marker is both KEPT in state AND returned as the drawn marker, so the dot does not blink to the
    // acked fallback and the colouring stays on the live boundary for this frame (#6).
    assert_eq!(state.marker_pos, Some(held), "the marker is HELD through the transient gap, not nulled (#6)");
    assert_eq!(drawn, Some(held), "the held marker is still drawn this frame (no revert to the acked dot)");
  }

  #[test]
  fn update_live_overlay_clears_the_marker_only_when_no_status_at_all() {
    // The disconnected/pre-connect case (no status report): the held marker IS cleared, so a later reconnect
    // snaps fresh rather than easing in from a stale position.
    let (mut state, mut view) = overlay_fixture();
    let bounds = state.toolpath_bounds.expect("bounds");
    let span = bounds.1 - bounds.0;
    feed_status_into(&mut view, "Run|MPos:5.000,0.000,0.000|WCO:0.000,0.000,0.000");
    update_live_overlay(&mut state, &view, bounds, span);
    assert!(state.marker_pos.is_some());
    // Now drop the status entirely (a disconnect clears it).
    view.status = None;
    update_live_overlay(&mut state, &view, bounds, span);
    assert_eq!(state.marker_pos, None, "with no status at all the marker is cleared for a fresh snap on reconnect");
  }

  #[test]
  fn update_live_overlay_suppresses_a_parked_off_path_marker_but_still_colours() {
    // Finding #4/#3/#5: an IDLE machine parked far off the loaded path (homed to machine origin, a units
    // mismatch, or a WCS mismatch) must not draw a confident-but-wrong dot. The marker is suppressed (None), yet
    // the cut boundary is unaffected (it only advances from real progress, never retreats).
    let (mut state, mut view) = overlay_fixture();
    let bounds = state.toolpath_bounds.expect("bounds");
    let span = bounds.1 - bounds.0;
    // An IDLE report parked far off the path (work −500,−500 — well outside the 0..10 square plus margin).
    feed_status_into(&mut view, "Idle|MPos:-500.000,-500.000,0.000|WCO:0.000,0.000,0.000");
    let drawn = update_live_overlay(&mut state, &view, bounds, span);
    assert!(drawn.is_none(), "a parked-off-path idle marker must be suppressed, not clipped to the rect edge");

    // The SAME point while actively running IS shown (a legitimate cut can run outside the drawn extents).
    feed_status_into(&mut view, "Run|MPos:-500.000,-500.000,0.000|WCO:0.000,0.000,0.000");
    assert!(update_live_overlay(&mut state, &view, bounds, span).is_some(), "a moving machine always shows the dot");
  }

  /// Feed one Run-state status sample at work position `(x, y)` and depth `z` (work-Z) and fold it into the overlay.
  /// The fixture is authored at WCO origin, so the work XY equals the reported MPos. Used by the trail tests below to
  /// drive a sequence of (WPos, Z) samples through `update_live_overlay`.
  fn feed_cut_sample(state: &mut UiState, view: &mut ViewState, bounds: (Vec2, Vec2), span: Vec2, x: f32, y: f32, z: f32) {
    feed_status_into(view, &format!("Run|MPos:{x:.3},{y:.3},{z:.3}|WCO:0.000,0.000,0.000"));
    update_live_overlay(state, view, bounds, span);
  }

  #[test]
  fn the_trail_records_only_below_surface_cutting_samples_in_order() {
    // THE WORKING MODEL: the cut trail is a breadcrumb of the tool's ACTUAL reported positions, accumulated ONLY
    // while cutting below the work surface (Z < 0). Samples at Z >= 0 (rapids, lead-ins, clearance) add NO vertices.
    // We feed a mix and assert the trail vertices are exactly the Z<0 samples, in order, at the reported positions.
    let (mut state, mut view) = overlay_fixture();
    let bounds = state.toolpath_bounds.expect("bounds");
    let span = bounds.1 - bounds.0;
    // The step gate decimates sub-step moves; the square spans 10mm so a 2mm step clears TRAIL_MIN_STEP_FRACTION.
    let samples = [
      (0.0, 0.0, 5.0),   // above surface (rapid to start): NO vertex.
      (0.0, 0.0, -1.0),  // plunge, first cut: vertex at (0,0).
      (2.0, 0.0, -1.0),  // cutting along the bottom: vertex at (2,0).
      (4.0, 0.0, -1.0),  // cutting: vertex at (4,0).
      (4.0, 0.0, 5.0),   // retract above surface: NO vertex.
      (8.0, 0.0, 6.0),   // rapid across (above surface): NO vertex.
    ];
    for (x, y, z) in samples {
      feed_cut_sample(&mut state, &mut view, bounds, span, x, y, z);
    }
    let recorded: Vec<(f32, f32)> = state.trail.iter().map(|p| (p.pos.x, p.pos.y)).collect();
    assert_eq!(recorded, vec![(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)], "only the three Z<0 samples are recorded, in order");
    // (d) the trail never contains a point the tool was not reported at: every recorded point is one of the input
    // sample XYs (it is a subset of reported positions, never a projected/interpolated point).
    let reported: Vec<(f32, f32)> = samples.iter().map(|(x, y, _)| (*x, *y)).collect();
    assert!(recorded.iter().all(|p| reported.contains(p)), "every trail vertex is an actually-reported position");
  }

  #[test]
  fn the_trail_ignores_an_above_surface_lead_in_that_passes_near_a_far_segment() {
    // THE REGRESSION, killed by the Z gate: the lead-in sample `(0.5, 9.0)` that ratcheted every projection approach
    // ahead of the cutter runs ABOVE the surface (Z >= 0). The breadcrumb simply never records it, so it can never
    // contaminate the trail — no leading is even possible.
    let (mut state, mut view) = overlay_fixture();
    let bounds = state.toolpath_bounds.expect("bounds");
    let span = bounds.1 - bounds.0;
    feed_cut_sample(&mut state, &mut view, bounds, span, 0.5, 9.0, 1.0); // the lead-in, above the surface.
    assert!(state.trail.is_empty(), "an above-surface lead-in contributes no trail vertex");
    // It does record once the tool actually plunges to cut at the bottom-left where the cutter really is.
    feed_cut_sample(&mut state, &mut view, bounds, span, 0.0, 0.0, -1.0);
    let recorded: Vec<(f32, f32)> = state.trail.iter().map(|p| (p.pos.x, p.pos.y)).collect();
    assert_eq!(recorded, vec![(0.0, 0.0)], "only the actual below-surface cut is recorded — at the tool, never ahead");
  }

  #[test]
  fn the_trail_breaks_a_stroke_across_a_pen_up_lift() {
    // A lift between two cuts (the tool retracts to Z >= 0 and plunges again elsewhere) must begin a FRESH stroke, so
    // the draw loop never streaks a line across the travel. The first cut after a non-cut frame is flagged
    // `stroke_start`; a continuing cut is not.
    let (mut state, mut view) = overlay_fixture();
    let bounds = state.toolpath_bounds.expect("bounds");
    let span = bounds.1 - bounds.0;
    feed_cut_sample(&mut state, &mut view, bounds, span, 0.0, 0.0, -1.0); // first cut: stroke start.
    feed_cut_sample(&mut state, &mut view, bounds, span, 4.0, 0.0, -1.0); // continuing cut: joins.
    feed_cut_sample(&mut state, &mut view, bounds, span, 4.0, 0.0, 5.0);  // lift (above surface): no vertex, pen up.
    feed_cut_sample(&mut state, &mut view, bounds, span, 0.0, 8.0, -1.0); // new cut after the lift: stroke start.
    let starts: Vec<bool> = state.trail.iter().map(|p| p.stroke_start).collect();
    assert_eq!(starts, vec![true, false, true], "first cut and the post-lift cut start strokes; the middle joins");
  }

  #[test]
  fn the_trail_is_wiped_on_a_fresh_run_entry() {
    // A rising edge into Run wipes the prior run's trail so the fresh job draws over the dim planned preview alone.
    // This is "this run" framing — a resume out of a feed-hold (Hold → Run) counts as a run-entry too and begins the
    // trail fresh (matching `entered_run`), so the trail never carries a previous run's path under the new one.
    let (mut state, mut view) = overlay_fixture();
    let bounds = state.toolpath_bounds.expect("bounds");
    let span = bounds.1 - bounds.0;
    feed_cut_sample(&mut state, &mut view, bounds, span, 0.0, 0.0, -1.0);
    feed_cut_sample(&mut state, &mut view, bounds, span, 4.0, 0.0, -1.0);
    assert!(state.trail.len() >= 2, "two cuts recorded within the run");
    // Mid-run cut frames do NOT wipe (no rising edge): the trail keeps growing.
    feed_cut_sample(&mut state, &mut view, bounds, span, 8.0, 0.0, -1.0);
    assert!(state.trail.len() >= 3, "a continuing Run frame keeps accumulating, never wipes mid-run");
    // A genuine rising edge (Idle → Run) DOES wipe the prior run's trail for the fresh job.
    feed_status_into(&mut view, "Idle|MPos:8.000,0.000,0.000|WCO:0.000,0.000,0.000");
    update_live_overlay(&mut state, &view, bounds, span);
    feed_status_into(&mut view, "Run|MPos:0.000,0.000,0.000|WCO:0.000,0.000,0.000");
    update_live_overlay(&mut state, &view, bounds, span);
    assert!(state.trail.is_empty(), "a fresh run-entry wipes the prior run's trail");
  }

  #[test]
  fn update_live_overlay_tracks_the_marker_in_run_jog_and_hold() {
    // The live marker dot follows the tool whenever a work position is derivable — across Run, Jog, and Hold. The
    // cut trail is extended alongside it (only while running and cutting below the surface); the overlay's two
    // per-frame jobs are the marker dot and the cut-trail breadcrumb.
    let (mut state, mut view) = overlay_fixture();
    let bounds = state.toolpath_bounds.expect("bounds");
    let span = bounds.1 - bounds.0;

    for status in [
      "Run|MPos:4.000,0.000,-1.500|WCO:0.000,0.000,0.000",
      "Jog|MPos:2.000,0.000,-1.000|WCO:0.000,0.000,0.000",
      "Hold|MPos:3.000,0.000,-1.000|WCO:0.000,0.000,0.000",
    ] {
      feed_status_into(&mut view, status);
      assert!(update_live_overlay(&mut state, &view, bounds, span).is_some(), "{status} must show the marker");
    }
  }

  #[test]
  fn update_live_overlay_keeps_the_marker_above_the_surface() {
    // A frame with the tool AT or ABOVE the work surface (Z >= 0 — a rapid, a clearance move, a retract) still tracks
    // the marker: the marker reflects true position regardless of cut depth (the cut TRAIL is gated on Z < 0
    // separately; the marker is not).
    let (mut state, mut view) = overlay_fixture();
    let bounds = state.toolpath_bounds.expect("bounds");
    let span = bounds.1 - bounds.0;

    feed_status_into(&mut view, "Run|MPos:5.000,0.000,2.000|WCO:0.000,0.000,0.000");
    assert!(update_live_overlay(&mut state, &view, bounds, span).is_some(), "the marker tracks above the surface");
  }

  #[test]
  fn entered_run_edge_detects_the_transition_into_run_only() {
    use crate::protocol::RunState;
    // The edge fires exactly on a (not-Run) → Run transition: the first frame that is Run when the prior was not.
    assert!(entered_run(None, Some(RunState::Run)), "first-ever Run frame is an entry");
    assert!(entered_run(Some(RunState::Idle), Some(RunState::Run)), "Idle → Run is an entry");
    assert!(entered_run(Some(RunState::Hold), Some(RunState::Run)), "Hold → Run (resume) is an entry");
    // A run already in progress is NOT a fresh entry — so a mid-run Run frame must not re-snap the marker.
    assert!(!entered_run(Some(RunState::Run), Some(RunState::Run)), "Run → Run is not a fresh entry");
    // Leaving or never being Run is not an entry.
    assert!(!entered_run(Some(RunState::Run), Some(RunState::Idle)), "Run → Idle is not an entry");
    assert!(!entered_run(Some(RunState::Idle), Some(RunState::Idle)), "Idle → Idle is not an entry");
    assert!(!entered_run(None, None), "no status either side is not an entry");
  }

  #[test]
  fn update_live_overlay_snaps_the_marker_fresh_on_entering_run() {
    // Entering Run (the Idle/Hold → Run edge, or cycle-start) drops the stale marker so the fresh run's marker snaps
    // to its first sampled position rather than lerping in from the prior run's last one.
    let (mut state, mut view) = overlay_fixture();
    let bounds = state.toolpath_bounds.expect("bounds");
    let span = bounds.1 - bounds.0;

    // A first run acquires a marker far across the bed.
    feed_status_into(&mut view, "Run|MPos:8.000,0.000,-1.000|WCO:0.000,0.000,0.000");
    update_live_overlay(&mut state, &view, bounds, span);
    let first = state.marker_pos.expect("the first run acquired a marker");

    // End the run (Idle), then a NEW run starts near the origin. The entry edge drops the stale marker, so the new
    // run's marker snaps to its first sample rather than lerping from the far-away prior position.
    feed_status_into(&mut view, "Idle|MPos:8.000,0.000,5.000|WCO:0.000,0.000,0.000");
    update_live_overlay(&mut state, &view, bounds, span);
    feed_status_into(&mut view, "Run|MPos:0.000,0.000,-1.000|WCO:0.000,0.000,0.000");
    update_live_overlay(&mut state, &view, bounds, span);
    let snapped = state.marker_pos.expect("the new run acquired a marker");
    assert!(snapped.x < first.x * 0.5, "the marker snapped to the new run's origin, not lerped from the far prior");
  }

  #[test]
  fn set_program_clears_the_stale_marker_from_a_prior_file() {
    // Loading a NEW program must reset the overlay so a prior file's lagging marker never lingers over the freshly
    // loaded geometry. (The progress backplot is program-scoped via the parsed toolpath, recomputed by set_program.)
    let (mut state, mut view) = overlay_fixture();
    let bounds = state.toolpath_bounds.expect("bounds");
    let span = bounds.1 - bounds.0;
    feed_status_into(&mut view, "Run|MPos:3.000,0.000,-1.000|WCO:0.000,0.000,0.000");
    update_live_overlay(&mut state, &view, bounds, span);
    assert!(state.marker_pos.is_some(), "the prior run seeded a marker");

    // Load a different program: the marker is program-scoped and must reset so it snaps fresh on the next run.
    state.set_program(vec!["G0 X0 Y0".to_string(), "G1 X5 Y5 Z-1".to_string()], Some("new.nc".to_string()));
    assert!(state.marker_pos.is_none(), "a new program drops the stale marker so it snaps fresh");
  }

}
