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
use super::theme::Theme;
use super::view_state::{Banner, LogLine, LogSource, ViewState};
use crate::protocol::{ConnectionState, RealtimeCommand};
use crate::transport::ports::PortInfo;

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
/// clamps live edits, but a value loaded from a hand-edited/corrupt profile bypasses that — `0` would fail the
/// port open). The widget's own range still clamps subsequent edits.
fn sanitize_baud(baud: u32) -> u32 {
  if BAUD_RANGE.contains(&baud) { baud } else { DEFAULT_BAUD }
}

/// Transient widget state the shell owns across frames: selections, text fields, and tunables that belong to
/// the UI, not to the engine-derived [`ViewState`]. Kept here so the views read and mutate it directly while
/// the shell persists it.
#[derive(Debug, Clone)]
pub struct UiState {
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
  /// space (work-coordinate XY mm), oldest first. Live work positions are appended as the tool moves (decimated by
  /// a step gate) and the trail is drawn as a polyline over the dim preview, so it shows real travel rather than
  /// guessing which preview segment the tool is on. Each point carries whether it was a rapid (travel) move so the
  /// trail can colour rapids apart from cuts. Capped at [`MAX_TRAIL_POINTS`] (rolling, oldest dropped) and cleared
  /// by [`Self::set_program`].
  trail: std::collections::VecDeque<TrailPoint>,
  /// The maximum programmed feed (`F` word, mm/min) in the loaded program, scanned once at load. Rapids run faster
  /// than any programmed cut, so the live realized feed measured against this (scaled by the live feed override)
  /// classifies each trail point as rapid vs cut (see [`super::preview::is_rapid_feed`]). Zero when no program is
  /// loaded or it carries no feed.
  max_feed: f64,
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
  /// The Phase 2 verify/measure starting A angle (degrees): θ for the flip-verify pair, and the first runout angle.
  pub verify_start_angle: f64,
  /// The Phase 2 runout report's number of evenly-spaced angles (N ≥ 2).
  pub verify_runout_n: usize,
  /// Whether the settings window is open.
  pub settings_open: bool,
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
  /// and panels reclaim the space. Defaults to expanded (the design opens the dock at its full 200px height).
  pub dock_collapsed: bool,
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

impl Default for UiState {
  fn default() -> Self {
    UiState {
      ports: Vec::new(),
      selected_port: String::new(),
      baud: DEFAULT_BAUD,
      program_path: None,
      program: std::sync::Arc::from([] as [String; 0]),
      toolpath: Vec::new(),
      toolpath_bounds: None,
      marker_pos: None,
      trail: std::collections::VecDeque::new(),
      max_feed: 0.0,
      jog_step: 1.0,
      jog_continuous: false,
      jog_feed: 500.0,
      feed_override_drag: super::overrides::OverrideFeedback::default(),
      spindle_override_drag: super::overrides::OverrideFeedback::default(),
      console_input: String::new(),
      probe_depth: 10.0,
      probe_feed: 50.0,
      plate_thickness: 1.0,
      rotary_dowel_diameter: 6.0,
      rotary_index_angle: 0.0,
      rotary_bench: super::rotary_probe::RotaryProbeParams::default(),
      rotary_side_probe_confirmed: false,
      verify_start_angle: 0.0,
      verify_runout_n: 4,
      settings_open: false,
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
    self.toolpath = parse_xy_path(&self.program);
    self.toolpath_bounds = toolpath_bounds(&self.toolpath);
    self.max_feed = max_programmed_feed(&self.program);
    // A fresh program starts unfollowed: the first streamed line of the new file must re-trigger an auto-scroll
    // even if the old program happened to leave us followed at the same row index.
    self.program_followed_line = None;
    // The live overlay is program-scoped: a new file starts with no acquired marker and an empty trail, so the
    // marker snaps to the first sample of the new run and the trail does not carry over the old program's travel.
    self.marker_pos = None;
    self.trail.clear();
  }
}

/// The candidate jog step sizes (mm) offered as quick buttons.
const JOG_STEPS: [f64; 5] = [0.01, 0.1, 1.0, 5.0, 10.0];

/// Draw the recurring 30px header strip (design §03): a fixed-height `panelAlt` bar with `0 14px` padding, the
/// title at the left, the `right` closure's controls pulled to the right edge, and a 1px divider along the
/// bottom. The bar height is pinned (not content-driven) so every section header and the dock tab strip line
/// up. `title` is rendered already-styled by the caller's pick; pass it as the design's uppercase tracked
/// section title via [`section_header`], or hand-style it (e.g. dock tabs) and call this directly.
fn header_bar(ui: &mut egui::Ui, left: impl FnOnce(&mut egui::Ui), right: impl FnOnce(&mut egui::Ui)) {
  // Claim the full strip up front so the fill and divider span the panel's width regardless of content.
  let width = ui.available_width();
  let (rect, _) = ui.allocate_exact_size(Vec2::new(width, Metrics::HEADER_H), egui::Sense::hover());
  let painter = ui.painter();
  painter.rect_filled(rect, 0.0, Theme::PANEL_ALT);
  // 1px bottom divider, matching the `border-bottom:1px solid #2E2E2E` under every header in the mock.
  let y = rect.bottom() - 0.5;
  painter.hline(rect.x_range(), y, egui::Stroke::new(Metrics::DIVIDER, Theme::DIVIDER));

  // Lay the header content inside the strip, vertically centred, with the design's 14px horizontal padding.
  let content = rect.shrink2(Vec2::new(Metrics::HEADER_PAD_X, 0.0));
  let builder = egui::UiBuilder::new().max_rect(content).layout(Layout::left_to_right(Align::Center));
  let mut content_ui = ui.new_child(builder);
  left(&mut content_ui);
  content_ui.with_layout(Layout::right_to_left(Align::Center), right);
}

/// Draw a section header with the design's uppercase, letter-tracked, dim title and no right-side controls —
/// the common case above the override/probe/toolpath/jog sections.
pub fn section_header(ui: &mut egui::Ui, title: &str) {
  header_bar(ui, |ui| header_title(ui, title), |_ui| {});
}

/// Draw a dock tab strip in the design's §03 style: the same 30px `panelAlt` bar, but with mixed-case tab
/// labels at 11.5px (Medium when active / Regular when dim) and a 2px accent underline drawn under the active
/// tab. `tabs` are `(label, active)` pairs laid left→right; `right` fills the strip's right edge (e.g. the
/// progress readout). The tabs are clickable: the index of a clicked tab is returned so the caller can flip the
/// dock's active tab, while the underline marks the current selection. Returns `None` when no tab was clicked
/// this frame.
fn tab_strip(ui: &mut egui::Ui, tabs: &[(&str, bool)], right: impl FnOnce(&mut egui::Ui)) -> Option<usize> {
  let labels: Vec<(String, bool)> = tabs.iter().map(|(t, a)| (t.to_string(), *a)).collect();
  let mut clicked = None;
  header_bar(
    ui,
    |ui| {
      ui.spacing_mut().item_spacing.x = 0.0;
      for (index, (label, active)) in labels.iter().enumerate() {
        let (weight, color) = if *active { (true, Theme::TEXT) } else { (false, Theme::TEXT_DIM) };
        let mut text = RichText::new(label).size(Metrics::TAB_TEXT).color(color);
        if weight {
          text = text.strong();
        }
        // Each tab claims `0 14px` of horizontal room within the (already vertically centred) strip so the
        // underline spans its full width. The header bar lays this row out at `Align::Center`, so the label
        // sits on the strip's vertical midline without extra padding. The label is sensed for clicks so the
        // dock can switch tabs without a separate button chrome (the design draws the tabs as bare labels).
        let frame = egui::Frame::new().inner_margin(egui::Margin { left: 14, right: 14, top: 0, bottom: 0 });
        let response = frame.show(ui, |ui| ui.label(text)).response.interact(egui::Sense::click());
        if response.clicked() {
          clicked = Some(index);
        }
        if *active {
          // 2px accent underline pinned to the bottom of the 30px strip (not the label), the design's
          // active-tab signature. Snap to the strip floor so all tabs share one underline baseline.
          let r = response.rect;
          let floor = ui.max_rect().bottom();
          let y = floor - Metrics::TAB_UNDERLINE * 0.5;
          ui.painter().hline(r.x_range(), y, egui::Stroke::new(Metrics::TAB_UNDERLINE, Theme::ACCENT));
        }
      }
    },
    right,
  );
  clicked
}

/// Map a clicked tab index from the dock's `[Console, Program]` strip onto a [`DockTab`], leaving the selection
/// unchanged when nothing was clicked. Pure so the tab-switch decision is unit-tested without a window.
fn dock_tab_for_click(current: DockTab, clicked: Option<usize>) -> DockTab {
  match clicked {
    Some(0) => DockTab::Console,
    Some(1) => DockTab::Program,
    _ => current,
  }
}

/// The glyph for the dock's collapse/expand toggle: a minus when expanded (click to minimise) and a plus when
/// collapsed (click to restore). Pure so the icon choice is unit-tested without a window. Uses the typographic
/// minus (U+2212) so it reads as a control glyph at the toggle's small size rather than a hyphen.
fn dock_toggle_label(collapsed: bool) -> &'static str {
  if collapsed { "+" } else { "−" }
}

/// Render a header title in the design's section-header type: 11px Medium, uppercase, dim, with the 0.1em
/// tracking approximated by `extra_letter_spacing` so the headers read as small-caps labels, not body text.
fn header_title(ui: &mut egui::Ui, title: &str) {
  let text = RichText::new(title.to_ascii_uppercase())
    .size(Metrics::HEADER_TEXT)
    .color(Theme::TEXT_DIM)
    .strong()
    .extra_letter_spacing(Metrics::HEADER_TEXT * Metrics::HEADER_TRACKING_EM);
  ui.label(text);
}

/// The shared "state-toggled chip" frame: a filled, single-pixel-stroked, control-radius inset that both the
/// machine-state badge and the endstop chips draw. Centralising it keeps the chip theme contract (1px stroke,
/// [`Metrics::CONTROL_RADIUS`] corners) in one place; callers pass the asserted/clear `fill`/`border`, the inner
/// margin (badges and endstop chips pad differently), and the chip's content. Visual output is identical to the
/// per-site frames it replaces.
fn chip_frame(
  ui: &mut egui::Ui, fill: Color32, border: Color32, margin: egui::Margin,
  content: impl FnOnce(&mut egui::Ui),
) -> egui::Response {
  egui::Frame::new()
    .fill(fill)
    .stroke(egui::Stroke::new(1.0, border))
    .inner_margin(margin)
    .corner_radius(Metrics::CONTROL_RADIUS)
    .show(ui, content)
    .response
}

/// Paint a smooth left-to-right six-stripe Pride rainbow filling `rect` — the "fabulous" easter-egg accent. The
/// band is sampled from [`Theme::pride_at`] as a run of thin vertical slices so it blends rather than showing six
/// hard bands, with each slice overdrawn by a pixel to hide the seams. A no-op for a non-positive-width rect.
fn paint_pride(painter: &egui::Painter, rect: egui::Rect) {
  const SLICES: usize = 64;
  let slice_w = rect.width() / SLICES as f32;
  if slice_w <= 0.0 {
    return;
  }
  for i in 0..SLICES {
    let t = (i as f32 + 0.5) / SLICES as f32; // sample each slice at its centre.
    let x = rect.left() + i as f32 * slice_w;
    let slice = egui::Rect::from_min_size(egui::pos2(x, rect.top()), Vec2::new(slice_w + 1.0, rect.height()));
    painter.rect_filled(slice, 0.0, Theme::pride_at(t));
  }
}

/// Lay out a horizontal row of buttons at exact, vertically-aligned rects, so none of them drift the way
/// `add_sized` inside a `horizontal` layout does — there each successive button crept a few pixels lower
/// (the staggered DRO "zero" row the user flagged: same height, but each ~2px below the last). The row claims
/// the full available width at [`Metrics::PANEL_CONTROL_H`] height, splits it by `weights` with `gap` between
/// cells, and calls `cell` once per index with the placed rect so the caller paints its own button there (via
/// [`egui::Ui::put`], which fills the rect exactly) and reads its response.
fn button_row(ui: &mut egui::Ui, gap: f32, weights: &[f32], mut cell: impl FnMut(&mut egui::Ui, usize, egui::Rect)) {
  let height = Metrics::PANEL_CONTROL_H;
  let full = ui.available_width();
  let sum: f32 = weights.iter().sum::<f32>().max(f32::EPSILON);
  let avail = (full - gap * (weights.len() as f32 - 1.0)).max(0.0);
  // Reserve the whole row up front (advancing the cursor below it); the per-cell `put` calls then place buttons
  // inside this reserved band without moving the cursor, so every cell shares one top and one bottom edge.
  let (rect, _) = ui.allocate_exact_size(Vec2::new(full, height), egui::Sense::hover());
  let mut x = rect.left();
  for (index, weight) in weights.iter().enumerate() {
    let w = avail * weight / sum;
    let cell_rect = egui::Rect::from_min_size(egui::pos2(x, rect.top()), Vec2::new(w, height));
    cell(ui, index, cell_rect);
    x += w + gap;
  }
}

/// A thin vertical divider for the toolbar: a 1px line at the design's ~22px height with the toolbar gap of
/// breathing room either side, replacing egui's full-height `separator()` so the toolbar groups read as
/// distinct without the bar feeling crammed (the user-flagged toolbar styling, design §03's `1px #2E2E2E`
/// group separators).
fn toolbar_divider(ui: &mut egui::Ui) {
  let (rect, _) = ui.allocate_exact_size(Vec2::new(1.0, Metrics::TOOLBAR_CONTROL_H), egui::Sense::hover());
  let center = rect.center();
  let half = 22.0 * 0.5;
  ui.painter().vline(center.x, (center.y - half)..=(center.y + half), egui::Stroke::new(1.0, Theme::DIVIDER));
}

/// Draw the machine-state badge: a coloured dot, the uppercase label, and (when streaming) the feed/speed
/// suffix. The dot colour is the *second* signal; the label is primary, per the design.
fn state_badge(ui: &mut egui::Ui, view: &ViewState, state_ui: &mut UiState) {
  let state = view.badge_state();
  let color = Theme::badge_color(state);
  let (fill, border, text_color) = match state {
    BadgeState::Alarm | BadgeState::Error => (Theme::ALARM_BG, Theme::ALARM_BORDER, Theme::ALARM_TEXT),
    _ => (Theme::INSET, color.gamma_multiply(0.5), Theme::TEXT),
  };
  let margin = egui::Margin { left: Metrics::BADGE_PAD.x as i8, right: Metrics::BADGE_PAD.x as i8,
    top: Metrics::BADGE_PAD.y as i8, bottom: Metrics::BADGE_PAD.y as i8 };
  let resp = chip_frame(ui, fill, border, margin, |ui| {
    ui.horizontal(|ui| {
      dot(ui, color, Metrics::BADGE_DOT);
      ui.add_space(2.0);
      ui.label(RichText::new(state.label()).color(text_color).strong());
      // The realized feed/speed rides along on the badge while a report is in hand and the machine is moving.
      if matches!(state, BadgeState::Run | BadgeState::Jog)
        && let Some((feed, rpm, _)) = view.status.as_ref().and_then(|s| s.feed_speed)
      {
        ui.label(RichText::new(format!("F {feed:.0} · S {rpm:.0}")).monospace().size(10.5)
          .color(Theme::TEXT_DIM));
      }
    });
  });
  // The badge doubles as the hidden Pride "fabulous mode" toggle: interact over the chip's rect (the frame
  // itself only hovers) so six clicks flip the rainbow accent. Show the click cursor so the spot is at least
  // feelable, but leave it unlabelled — discovering it is the point.
  let egg = ui.interact(resp.rect, resp.id.with("fabulous_egg"), egui::Sense::click());
  if egg.clicked() {
    state_ui.register_fabulous_click();
  }
}

/// Paint a small filled circle inline (a state dot), advancing the cursor by its diameter.
fn dot(ui: &mut egui::Ui, color: Color32, diameter: f32) {
  let (rect, _) = ui.allocate_exact_size(Vec2::splat(diameter), egui::Sense::hover());
  ui.painter().circle_filled(rect.center(), diameter * 0.5, color);
}

/// Render the 40px main toolbar: the connect group, Open, the Run/Hold/Stop segmented transport group, Home,
/// Settings, and the right-aligned machine-state badge (design §03).
pub fn toolbar(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  // 1px bottom divider under the bar (design §03's `border-bottom:1px #2E2E2E`), painted along the panel edge.
  let bar = ui.max_rect();
  ui.painter().hline(bar.x_range(), bar.bottom() - 0.5, egui::Stroke::new(1.0, Theme::DIVIDER));
  // `horizontal_centered` vertically centres the 26px controls in the 40px bar, giving the design's even
  // breathing room above and below rather than the top-aligned look the plain `horizontal` produced.
  ui.horizontal_centered(|ui| {
    // Toolbar controls are 26px tall with 10px side-padding and a 6px gap (design §03); set the region spacing
    // up front so every button/combo in the strip inherits the bar's sizing rather than the panel default.
    ui.spacing_mut().item_spacing.x = Metrics::TOOLBAR_GAP;
    ui.spacing_mut().button_padding = Metrics::TOOLBAR_BUTTON_PAD;
    ui.spacing_mut().interact_size.y = Metrics::TOOLBAR_CONTROL_H;
    // Gate the teardown affordance on whether a transport is ATTACHED, not on whether the board is fully ready.
    // A connect that stalls in `Connecting` (the ESP32-S3 can fail to volunteer readiness) still holds the OS port
    // open; without a Disconnect here the operator could not release the FD short of killing the process, blocking
    // espflash / another sender. While still `Connecting` the button reads "Cancel" (cancelling the connect); once
    // ready it reads "Disconnect". Both push the same intent — the shell sends `Command::Disconnect`, which ends the
    // engine task and drops the serial stream regardless of which lifecycle state it was in.
    let attached = view.connection.has_transport();
    let connected = view.connection.is_connected();

    if attached {
      let label = if connected { "Disconnect" } else { "Cancel" };
      if ui.button(label).clicked() {
        sink.push(Intent::Disconnect);
      }
    } else {
      // Port dropdown + refresh + identify + connect, only meaningful while disconnected. Each row shows the
      // (cu-preferred) path and, when known, a short USB product / «likely Galdr» hint so the board stands out.
      egui::ComboBox::from_id_salt("port")
        .selected_text(if state.selected_port.is_empty() { "Choose port…" } else { &state.selected_port })
        .show_ui(ui, |ui| {
          for port in &state.ports {
            // The selectable label carries the path; the hint (if any) trails it dimmed so the row stays
            // scannable while flagging the likely board.
            let label = match port.hint() {
              Some(hint) => format!("{}  ·  {hint}", port.path),
              None => port.path.clone(),
            };
            ui.selectable_value(&mut state.selected_port, port.path.clone(), label);
          }
        });
      if ui.button("⟳").on_hover_text("Refresh ports").clicked() {
        sink.push(Intent::RefreshPorts);
      }
      let has_port = !state.selected_port.is_empty();
      // Identify actively probes the selected port for grblHAL. It is opt-in (opening toggles the board's
      // auto-reset line) and never part of a refresh, so it sits behind its own button.
      if ui
        .add_enabled(has_port, egui::Button::new("Identify"))
        .on_hover_text("Probe the selected port for grblHAL (sends ?/$I)")
        .clicked()
      {
        sink.push(Intent::IdentifyPort { path: state.selected_port.clone() });
      }
      if ui.add_enabled(has_port, egui::Button::new("Connect")).clicked() {
        sink.push(Intent::Connect { path: state.selected_port.clone(), baud: state.baud });
      }
    }

    toolbar_divider(ui);

    if ui.button("Open…").on_hover_text("Load a G-code program").clicked()
      && let Some(path) = rfd::FileDialog::new().add_filter("G-code", &["gcode", "nc", "ngc", "tap"]).pick_file()
    {
      sink.push(Intent::OpenProgram(path));
    }

    toolbar_divider(ui);
    transport_group(ui, view, state, sink);
    toolbar_divider(ui);

    // Home runs the firmware homing cycle; safe to offer whenever connected and not already moving. Drawn as a
    // ghost button (transparent rest, design §03) so it reads as a secondary action beside the framed groups.
    let badge = view.badge_state();
    let can_home = connected && !matches!(badge, BadgeState::Run | BadgeState::Jog | BadgeState::Home);
    let home_color = if can_home { Theme::TEXT_DIM } else { Theme::TEXT_DISABLED };
    let home = egui::Button::new(RichText::new("⌂ Home").color(home_color)).fill(Color32::TRANSPARENT);
    if ui.add_enabled(can_home, home).on_hover_text("Run homing cycle ($H)").clicked() {
      sink.push(Intent::Home);
    }

    if ui.button("Settings").clicked() {
      state.settings_open = !state.settings_open;
    }

    // Right-aligned state badge so it is always visible regardless of toolbar width.
    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
      state_badge(ui, view, state);
    });
  });

  // Fabulous mode: lay a thin Pride rainbow along the toolbar's bottom edge, overpainting the divider. Drawn
  // after the bar's content so it sits on top, and bookended by the matching band on the status bar.
  if state.fabulous {
    let stripe = egui::Rect::from_min_max(
      egui::pos2(bar.left(), bar.bottom() - Metrics::PRIDE_STRIPE_H),
      egui::pos2(bar.right(), bar.bottom()),
    );
    paint_pride(ui.painter(), stripe);
  }
}

/// The Run/Hold/Stop segmented group plus a separate Abort control. The leading segment starts a stream (Idle) or
/// resumes (Hold); Hold issues a feed-hold; Stop issues the GRACEFUL program stop (`0x86` — decelerate, flush,
/// return to Idle, no alarm); the standalone Abort issues the HARD soft-reset (`0x18` → `ALARM:3`). Enable/emphasis
/// come from the pure [`TransportGroup`] matrix so the view stays a renderer.
pub(crate) fn transport_group(ui: &mut egui::Ui, view: &ViewState, state: &UiState, sink: &mut IntentSink) {
  use egui::CornerRadius;
  let group = TransportGroup::for_state(view.badge_state(), !state.program.is_empty());

  let run_label = if group.run_is_resume {
    "▶ Resume"
  } else if group.run_active {
    "▶ Running"
  } else {
    "▶ Run"
  };
  // Joined segmented group (design §03): the three buttons abut with no gap and only the outer corners are
  // rounded — Run rounds its left, Stop its right, Hold stays square.
  let r = Metrics::CONTROL_RADIUS;
  let left = CornerRadius { nw: r, sw: r, ne: 0, se: 0 };
  let mid = CornerRadius::ZERO;
  let right = CornerRadius { nw: 0, sw: 0, ne: r, se: r };
  // Zero the gap only between the three joined segments, then restore the toolbar gap on the way out — otherwise
  // the `item_spacing.x = 0` leaked onto the shared toolbar `ui`, cramming Home/Settings hard against the group.
  let prev_gap = ui.spacing().item_spacing.x;
  ui.spacing_mut().item_spacing.x = 0.0;

  let mut run_button = egui::Button::new(RichText::new(run_label).color(Theme::STATE_RUN)).corner_radius(left);
  if group.run_active {
    run_button = run_button.fill(Theme::INSET);
  }
  if ui.add_enabled(group.run_enabled, run_button).clicked() {
    sink.push(Intent::RunOrResume);
  }
  let hold = egui::Button::new("⏸ Hold").corner_radius(mid);
  if ui.add_enabled(group.hold_enabled, hold).on_hover_text("Feed hold (!)").clicked() {
    sink.push(Intent::Realtime(RealtimeCommand::FeedHold));
  }
  // Stop is now the GRACEFUL program stop (`0x86`): the everyday "stop the job cleanly" button. It decelerates to a
  // block boundary, flushes the queue and returns to Idle with no alarm, so it reads as a normal-weight control
  // (amber, not danger-red) — the hard reset lives in the separate Abort button beside the group.
  let stop = egui::Button::new(RichText::new("■ Stop").color(Theme::STATE_HOLD)).corner_radius(right);
  if ui
    .add_enabled(group.stop_enabled, stop)
    .on_hover_text("Stop the job cleanly (0x86) — decelerate, flush, return to Idle")
    .clicked()
  {
    sink.push(Intent::Realtime(RealtimeCommand::ProgramStop));
  }
  // Restore the toolbar gap before the standalone Abort so it sits apart from the joined segments, signalling it is
  // a separate, weightier action rather than a fourth segment of the group.
  ui.spacing_mut().item_spacing.x = prev_gap;

  // Abort / E-stop: the HARD soft-reset (`0x18` → `ALARM:3`). Visually distinct — danger-red, fully rounded, set
  // apart from the segmented group — and available the instant a transport is attached (even mid-handshake), so the
  // operator always has an emergency reset. The graceful Stop above is the routine control; this is the panic stop.
  let abort = egui::Button::new(RichText::new("⏹ Abort").color(Theme::DANGER))
    .corner_radius(Metrics::CONTROL_RADIUS);
  if ui
    .add_enabled(group.abort_enabled, abort)
    .on_hover_text("Emergency hard reset (0x18) — aborts to ALARM and resets the controller")
    .clicked()
  {
    sink.push(Intent::Realtime(RealtimeCommand::SoftReset));
  }

  // Simulate: a host-only physics ETA over the loaded program, available whenever a program is loaded — even
  // disconnected, since it sends nothing to the board. Drawn as a ghost button (transparent rest, like Home) so it
  // reads as a secondary, non-machine action set apart from the run controls. Disabled (greyed) with no program.
  let has_program = !state.program.is_empty();
  let sim_color = if has_program { Theme::TEXT_DIM } else { Theme::TEXT_DISABLED };
  let simulate = egui::Button::new(RichText::new("∿ Simulate").color(sim_color)).fill(Color32::TRANSPARENT);
  if ui
    .add_enabled(has_program, simulate)
    .on_hover_text("Estimate job time from the machine settings (no motion — host-only)")
    .clicked()
  {
    sink.push(Intent::Simulate);
  }
}

/// Render the digital readout: a WPos/MPos toggle, large per-axis rows (coloured letter + big tabular value +
/// unit), the Zero X/Y/Z/XYZ button row, and the WCO strip (design §03).
pub fn dro(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  header_bar(
    ui,
    |ui| header_title(ui, "Digital Readout"),
    |ui| {
      // The WPos/MPos toggle sits inside the header bar (design §03); the small joined buttons pick which
      // coordinate the big readout shows. The design defaults to WPos. `right_to_left` lays MPos then WPos so
      // they read left→right as WPos · MPos.
      ui.spacing_mut().item_spacing.x = 1.0;
      ui.spacing_mut().button_padding = Metrics::DRO_TOGGLE_PAD;
      pos_toggle(ui, state, true, "MPos");
      pos_toggle(ui, state, false, "WPos");
    },
  );

  egui::Frame::new().inner_margin(Metrics::DRO_PAD).show(ui, |ui| {
  let (machine, work) = view.dro();
  let shown = if state.show_machine_pos { machine.as_ref() } else { work.as_ref() };
  let axes = [Axis::X, Axis::Y, Axis::Z];
  for (index, axis) in axes.iter().enumerate() {
    ui.horizontal(|ui| {
      ui.label(RichText::new(axis.letter().to_string()).size(Metrics::DRO_LETTER).strong()
        .color(Theme::axis_color(*axis)));
      ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
        ui.label(RichText::new("mm").monospace().size(Metrics::DRO_UNIT).color(Theme::TEXT_DISABLED));
        ui.label(big_axis_value(shown, index));
      });
    });
    ui.add_space(Metrics::DRO_ROW_GAP - ui.spacing().item_spacing.y);
  }

  ui.add_space(4.0);
  // Zero buttons set the active WCS origin to the current position on the chosen axes. The first three are
  // equal-width; the design gives Zero XYZ a wider (flex 1.4), primary-blue slot, so it draws the eye. They are
  // placed at exact, shared-edge rects via [`button_row`] so the four sit on one horizontal line — the previous
  // `add_sized`-in-`horizontal` layout crept each successive button a couple of pixels lower (the staggered row
  // the user flagged).
  let enabled = view.connection == ConnectionState::Idle;
  ui.add_enabled_ui(enabled, |ui| {
    // Trim the horizontal button padding from egui's default 14px so the labels fit their narrow placed cells
    // (at the default, "Zero XYZ" had no room and wrapped onto two lines inside its slot).
    ui.spacing_mut().button_padding = Vec2::new(6.0, 0.0);
    button_row(ui, 6.0, &[1.0, 1.0, 1.0, 1.4], |ui, index, rect| {
      let (label, axes, primary): (&str, Vec<Axis>, bool) = match index {
        0 => ("X", vec![Axis::X], false),
        1 => ("Y", vec![Axis::Y], false),
        2 => ("Z", vec![Axis::Z], false),
        _ => ("Zero XYZ", Vec::new(), true),
      };
      // Size the label to the design's ~11.5px and never wrap: the placed cell is narrow, and at egui's larger
      // default button font "Zero XYZ" wrapped onto two lines inside its slot. `Extend` keeps it one line.
      let text = RichText::new(label).size(11.5).color(Theme::TEXT);
      let mut button = egui::Button::new(text).wrap_mode(egui::TextWrapMode::Extend);
      if primary {
        button = button.fill(Theme::ACCENT);
      }
      if ui.put(rect, button).clicked() {
        sink.push(Intent::SetWorkZero { axes });
      }
    });
  });

  // WCO strip: the work-coordinate offset, cached across reports.
  if !view.last_wco.is_empty() {
    ui.add_space(6.0);
    let wco: Vec<String> = view.last_wco.iter().map(|v| format!("{v:.3}")).collect();
    egui::Frame::new().fill(Theme::INSET).inner_margin(egui::Margin::symmetric(10, 6)).corner_radius(2.0)
      .show(ui, |ui| {
        ui.horizontal(|ui| {
          ui.label(RichText::new("WCO").monospace().size(10.5).color(Theme::TEXT_DIM));
          ui.label(RichText::new(wco.join(", ")).monospace().size(10.5).color(Theme::TEXT));
        });
      });
  }

  // Tool strip: the active tool reported by the firmware's `$G`/`[GC:]` parser state ([`ViewState::current_tool`]),
  // the single authoritative source. `T0` reads as "none" so the operator can tell an explicitly-empty spindle from
  // one that simply has a tool. Never sourced from the `<...>` status report (it carries no tool number).
  if let Some(tool) = view.current_tool {
    ui.add_space(6.0);
    egui::Frame::new().fill(Theme::INSET).inner_margin(egui::Margin::symmetric(10, 6)).corner_radius(2.0)
      .show(ui, |ui| {
        ui.horizontal(|ui| {
          ui.label(RichText::new("TOOL").monospace().size(10.5).color(Theme::TEXT_DIM));
          let label = if tool == 0 { "none".to_string() } else { format!("T{tool}") };
          ui.label(RichText::new(label).monospace().size(10.5).color(Theme::TEXT));
        });
      });
  }

  // Endstop indicator row: three tight X/Y/Z chips that light red when the corresponding limit switch is
  // asserted in the latest `Pn:` field, and sit dim/inset when clear. Only the X/Y/Z limits are surfaced for
  // now, but the parsed `PinState` carries the full grblHAL signal set so probe/door/etc. can join this row
  // later without reopening the parser. Drawn unconditionally so the operator always has an endstop reference;
  // with no report (or an absent `Pn:`) all three read clear.
  ui.add_space(6.0);
  endstop_chips(ui, view);
  });
}

/// Render the X/Y/Z endstop indicator chips. Each chip lights red while its limit switch is asserted in the
/// latest status report and sits dim/inset when clear. Kept deliberately tight — a small per-row item spacing
/// and chip inset so the three chips plus the `LIMITS` label never overflow the fixed 268px left column (the
/// panel-overflow lesson). The lit-vs-clear decision comes from the typed [`ViewState::pins`], decoded once when
/// each status report is ingested rather than re-parsed per frame, so this stays a dumb renderer.
fn endstop_chips(ui: &mut egui::Ui, view: &ViewState) {
  let pins = view.pins;
  ui.horizontal(|ui| {
    ui.spacing_mut().item_spacing.x = 4.0;
    ui.label(RichText::new("LIMITS").size(Metrics::HEADER_TEXT).color(Theme::TEXT_DIM)
      .extra_letter_spacing(Metrics::HEADER_TEXT * Metrics::HEADER_TRACKING_EM));
    for (letter, asserted) in [("X", pins.limit_x), ("Y", pins.limit_y), ("Z", pins.limit_z)] {
      endstop_chip(ui, letter, asserted);
    }
  });
}

/// Draw one endstop chip: a small rounded inset with the axis letter, filled with the alarm surface and
/// labelled in alarm text while asserted, dim/inset while clear. Sized tight (a 6/2 inner margin, no button
/// chrome) so three fit the left column alongside the `LIMITS` label.
fn endstop_chip(ui: &mut egui::Ui, letter: &str, asserted: bool) {
  let (fill, text) = if asserted {
    (Theme::ALARM_BG, Theme::ALARM_TEXT)
  } else {
    (Theme::INSET, Theme::TEXT_DISABLED)
  };
  let border = if asserted { Theme::ALARM_BORDER } else { Theme::DIVIDER };
  // Tight 6/2 inner margin so three chips plus the `LIMITS` label fit the fixed 268px column (overflow lesson).
  chip_frame(ui, fill, border, egui::Margin { left: 6, right: 6, top: 2, bottom: 2 }, |ui| {
    let mut label = RichText::new(letter).monospace().size(11.0).color(text);
    if asserted {
      label = label.strong();
    }
    ui.label(label).on_hover_text(if asserted { "Limit switch asserted" } else { "Limit clear" });
  });
}

/// One DRO coordinate-toggle button (WPos/MPos), styled as a small joined segment: the active side carries the
/// `widget.active` fill and the accent text, the inactive side the widget rest fill and dim text (design §03).
fn pos_toggle(ui: &mut egui::Ui, state: &mut UiState, machine: bool, label: &str) {
  let active = state.show_machine_pos == machine;
  let (fill, text) = if active { (Theme::WIDGET_ACTIVE, Theme::ACCENT) } else { (Theme::WIDGET, Theme::TEXT_DIM) };
  let button = egui::Button::new(RichText::new(label).size(10.0).color(text)).fill(fill).corner_radius(0.0);
  if ui.add(button).clicked() {
    state.show_machine_pos = machine;
  }
}

/// Format one axis value as a large monospace tabular fixed-point string, or a dash when not derivable. Padded
/// to 8 columns (`-999.999` through `9999.999`), which spans a PCB-mill envelope while keeping the row inside the
/// fixed 268px column — a wider field overflowed the column and pushed the panel edge out (an unpainted gap).
fn big_axis_value(positions: Option<&Vec<f64>>, axis: usize) -> RichText {
  match positions.and_then(|p| p.get(axis)) {
    Some(value) => RichText::new(format!("{value:>8.3}")).monospace().size(Metrics::DRO_VALUE).color(Theme::TEXT),
    None => RichText::new(format!("{:>8}", "—")).monospace().size(Metrics::DRO_VALUE).color(Theme::TEXT_DISABLED),
  }
}

/// Render the jog pad: a step selector, feed field, and directional buttons for X/Y/Z plus jog-cancel.
pub fn jog(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  // The Jog header carries the cancel affordance on the right, matching the mock's "esc · cancel" hint.
  header_bar(
    ui,
    |ui| header_title(ui, "Jog"),
    |ui| {
      if ui.add(egui::Button::new(RichText::new("esc · cancel").size(10.0).color(Theme::TEXT_DISABLED))
        .fill(Color32::TRANSPARENT)).on_hover_text("Jog cancel (0x85)").clicked()
      {
        sink.push(Intent::Realtime(RealtimeCommand::JogCancel));
      }
    },
  );
  // grblHAL only honours `$J=` in Idle and Jog; it rejects it in Alarm/Hold/Door/Run/Check/Sleep. Gate the pad
  // to exactly those states so we never offer a control that will always be rejected (see [`jog_enabled`]).
  let enabled = jog_enabled(view.badge_state());
  egui::Frame::new().inner_margin(Metrics::JOG_PAD).show(ui, |ui| {
    ui.add_enabled_ui(enabled, |ui| {
      // These are dense icon/chip controls, so the default 14px button padding (28px/button) blows the cells past
      // their 32px squares and the segmented step chips past their slots, overflowing the 232px jog body. A tight
      // 6px gutter keeps every button's content inside its allocation (within the design's 4–8px dense range).
      ui.spacing_mut().button_padding.x = 6.0;
      // The pad: a 3×3 XY arrow grid of 32px cells (design §03) with a Z± column alongside, mirroring the
      // physical axes — Y+ top, X∓ flanking the centre, Y− bottom; Z+ / Z / Z− stacked to the right.
      // Derive the Z column's width from the row's own width here, not `available_width()` mid-row: after the
      // grid, the running layout reports a stale (too-large) remaining width, which sized the Z column wide
      // enough to overflow the 268px column and leave an unpainted gap beside the panel. The grid spans three
      // 32px cells with two inter-cell gaps; the Z column then fills what remains after the grid and the 16px
      // inter-column gap (a `JOG_GAP` item space, the `JOG_GAP*2` separator, and a second `JOG_GAP` item space).
      let xy_width = 3.0 * Metrics::JOG_CELL + 2.0 * Metrics::JOG_GAP;
      let zw = (ui.available_width() - xy_width - Metrics::JOG_GAP * 4.0).max(Metrics::JOG_CELL);
      ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing = Vec2::splat(Metrics::JOG_GAP);
        let gap = Vec2::splat(Metrics::JOG_GAP);
        egui::Grid::new("jog-xy").spacing(gap).min_col_width(Metrics::JOG_CELL).show(ui, |ui| {
          jog_blank(ui);
          jog_button(ui, "↑", state, sink, Some((Axis::Y, Dir::Pos)));
          jog_blank(ui);
          ui.end_row();
          jog_button(ui, "←", state, sink, Some((Axis::X, Dir::Neg)));
          jog_button(ui, "XY", state, sink, None);
          jog_button(ui, "→", state, sink, Some((Axis::X, Dir::Pos)));
          ui.end_row();
          jog_blank(ui);
          jog_button(ui, "↓", state, sink, Some((Axis::Y, Dir::Neg)));
          jog_blank(ui);
          ui.end_row();
        });

        ui.add_space(Metrics::JOG_GAP * 2.0);
        ui.vertical(|ui| {
          ui.spacing_mut().item_spacing.y = Metrics::JOG_GAP;
          jog_z(ui, "Z+", zw, state, sink, Some((Axis::Z, Dir::Pos)));
          jog_z(ui, "Z", zw, state, sink, None);
          jog_z(ui, "Z−", zw, state, sink, Some((Axis::Z, Dir::Neg)));
        });
      });

      // Step selector (design §03: segmented quick steps) and the jog feed rate.
      ui.add_space(8.0);
      ui.label(RichText::new("Step (mm)").size(10.5).color(Theme::TEXT_DIM));
      ui.add_space(2.0);
      step_selector(ui, state);
      ui.add_space(6.0);
      ui.horizontal(|ui| {
        ui.label(RichText::new("Feed").size(11.0).color(Theme::TEXT_DIM));
        ui.add(egui::DragValue::new(&mut state.jog_feed).speed(10.0).range(1.0..=10_000.0).suffix(" mm/min"));
      });
    });
  });
}

/// Render the jog step selector as a single joined segmented control (design §03), not a row of separate
/// bordered buttons (the "identical buttons" look the user flagged). The chips sit flush inside a recessed
/// inset track — the track shows through the 1px gaps as hairline separators — with the per-chip button border
/// and rounding stripped so the row reads as one control. One chip per numeric [`JOG_STEPS`] value plus a
/// trailing wider `cont` chip; exactly one is active (accent fill + accent text), the rest dim, so the selected
/// step is unmistakable. Picking a number turns continuous mode off; picking `cont` flips into press-and-hold.
fn step_selector(ui: &mut egui::Ui, state: &mut UiState) {
  egui::Frame::new()
    .fill(Theme::INSET)
    .stroke(egui::Stroke::new(1.0, Theme::BORDER_RECESS))
    .corner_radius(Metrics::CONTROL_RADIUS)
    .inner_margin(1)
    .show(ui, |ui| {
      // Strip the per-button border and rounding inside the track so the chips abut as one segmented control
      // rather than reading as individual buttons; the 1px inter-chip gap then shows the inset as a separator.
      {
        let widgets = &mut ui.visuals_mut().widgets;
        for visual in [&mut widgets.inactive, &mut widgets.hovered, &mut widgets.active, &mut widgets.noninteractive]
        {
          visual.bg_stroke = egui::Stroke::NONE;
          visual.corner_radius = egui::CornerRadius::ZERO;
        }
      }
      // The cells are narrow; egui's default 14px horizontal button padding would leave no room for the label and
      // wrap "0.01" onto two lines. Trim to a hair of padding so each chip's text fits its slot on one line.
      ui.spacing_mut().button_padding = Vec2::new(2.0, 0.0);
      // One slot per numeric step plus a slightly wider (1.2×) `cont` slot, matching the design's flex ratios.
      let cont_index = JOG_STEPS.len();
      let weights: Vec<f32> = (0..=cont_index).map(|i| if i == cont_index { 1.2 } else { 1.0 }).collect();
      button_row(ui, 1.0, &weights, |ui, index, rect| {
        let is_cont = index == cont_index;
        // A numeric chip is active only when it is the chosen step AND continuous mode is off; the `cont` chip is
        // active only in continuous mode — so the selector always shows exactly one active mode.
        let active = if is_cont {
          state.jog_continuous
        } else {
          !state.jog_continuous && (state.jog_step - JOG_STEPS[index]).abs() < f64::EPSILON
        };
        let (fill, text) = if active { (Theme::WIDGET_ACTIVE, Theme::ACCENT) } else { (Theme::PANEL, Theme::TEXT_DIM) };
        let label = if is_cont { "cont".to_string() } else { format!("{}", JOG_STEPS[index]) };
        let button = egui::Button::new(RichText::new(label).monospace().size(11.0).color(text))
          .fill(fill)
          .corner_radius(0.0)
          .wrap_mode(egui::TextWrapMode::Extend);
        let response = ui.put(rect, button);
        let response = if is_cont {
          response.on_hover_text("Continuous jog: hold a direction to move, release to stop")
        } else {
          response
        };
        if response.clicked() {
          if is_cont {
            state.jog_continuous = true;
          } else {
            state.jog_step = JOG_STEPS[index];
            state.jog_continuous = false;
          }
        }
      });
    });
}

/// Whether the jog pad should be enabled for the given machine state. grblHAL only accepts a `$J=` jog while
/// it is in `Idle` or `Jog` (a running jog can be re-targeted); every other state — `Run`, `Hold`, `Door`,
/// `Alarm`, `Check`, `Sleep`, and the disconnected/connecting host phases — rejects it, so we disable rather
/// than offer a control that is guaranteed to bounce. Pure so it is unit-tested without a window.
fn jog_enabled(state: BadgeState) -> bool {
  matches!(state, BadgeState::Idle | BadgeState::Jog)
}

/// An empty cell in the jog grid, sized to a jog cell so the arrows align on a true 3×3.
fn jog_blank(ui: &mut egui::Ui) {
  ui.allocate_exact_size(Vec2::splat(Metrics::JOG_CELL), egui::Sense::hover());
}

/// One XY jog cell: a fixed 32px square. With an axis/dir it issues a jog; the centre `XY` cell is an inert
/// label (design §03). In fixed-step mode a click issues a single [`Intent::Jog`]; in continuous mode holding
/// the cell issues [`Intent::JogStart`] on press and [`Intent::JogStop`] on release (see [`emit_jog`]).
fn jog_button(ui: &mut egui::Ui, label: &str, state: &UiState, sink: &mut IntentSink, motion: Option<(Axis, Dir)>) {
  let Some((axis, dir)) = motion else {
    // The centre cell is a non-interactive label marking the pad's purpose.
    let button = egui::Button::new(RichText::new(label).monospace().size(9.5).color(Theme::TEXT_DISABLED));
    ui.add_sized(Vec2::splat(Metrics::JOG_CELL), button);
    return;
  };
  let response = ui.add_sized(Vec2::splat(Metrics::JOG_CELL), egui::Button::new(label).sense(jog_sense(state)));
  emit_jog(&response, state, sink, axis, dir);
}

/// One Z-column jog button: a full-width cell, 32px tall, matching the XY pad's height. The middle `Z` cell is
/// an inert label between Z+ and Z−. Step vs continuous behaviour matches [`jog_button`].
fn jog_z(ui: &mut egui::Ui, label: &str, width: f32, state: &UiState, sink: &mut IntentSink,
  motion: Option<(Axis, Dir)>) {
  let size = Vec2::new(width, Metrics::JOG_CELL);
  let Some((axis, dir)) = motion else {
    ui.add_sized(size, egui::Button::new(RichText::new(label).color(Theme::TEXT_DISABLED)));
    return;
  };
  let response = ui.add_sized(size, egui::Button::new(label).sense(jog_sense(state)));
  emit_jog(&response, state, sink, axis, dir);
}

/// The egui sense a jog cell needs for the current mode: in continuous mode it must sense drag (press-and-hold)
/// so the press and release edges are observable, plus click so a brief tap still issues a bounded step jog; in
/// fixed-step mode a plain click suffices.
fn jog_sense(state: &UiState) -> egui::Sense {
  if state.jog_continuous {
    egui::Sense::click_and_drag()
  } else {
    egui::Sense::click()
  }
}

/// Translate a jog cell's [`egui::Response`] into the right jog intent(s) for the current mode. Fixed-step:
/// a click is one bounded [`Intent::Jog`]. Continuous: the press edge (`drag_started`) starts the long move and
/// the release edge (`drag_stopped`) cancels it, so the axis moves exactly while the control is held.
fn emit_jog(response: &egui::Response, state: &UiState, sink: &mut IntentSink, axis: Axis, dir: Dir) {
  if !state.jog_continuous {
    if response.clicked() {
      sink.push(Intent::Jog { axis, dir, distance: state.jog_step, feed: state.jog_feed });
    }
    return;
  }
  if response.drag_started() {
    sink.push(Intent::JogStart { axis, dir, feed: state.jog_feed });
  }
  if response.drag_stopped() {
    sink.push(Intent::JogStop);
  }
  // A tap too brief to register as a drag is reported as a click with no drag edges. It must NOT be bracketed as
  // a continuous start+stop: the queued `$J=` move and the out-of-band jog-cancel (`0x85`) race at the firmware,
  // and grblHAL drops a cancel that lands before the jog has actually begun — leaving the long move running away
  // (then a panic Stop soft-resets mid-motion into ALARM:3). Issue one bounded step jog instead, which completes
  // on its own and needs no cancel, so a quick nudge still moves by the last selected step.
  if response.clicked() && !response.drag_started() {
    sink.push(Intent::Jog { axis, dir, distance: state.jog_step, feed: state.jog_feed });
  }
}

/// Render the override controls: feed and spindle get a slider plus a fine/coarse/reset stepper row (design
/// §03's override sliders); rapid stays a 100/50/25 preset picker (grbl exposes no rapid ±). The sliders
/// express an absolute target; the shell turns that into the minimal relative ±10/±1/reset byte sequence
/// against the live `Ov:` value, so the view never does the override byte arithmetic.
pub fn overrides(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  use super::overrides::OverrideAxis;
  section_header(ui, "Overrides");
  let (feed, rapid, spindle) = view.status.as_ref().and_then(|s| s.overrides).unwrap_or((100, 100, 100));
  // Overrides only do anything on a ready link — a relative ±10/±1/reset byte is a no-op with nothing connected
  // to act on it — so the whole panel is disabled until the board is connected and ready (Idle/Run/Hold/…).
  let enabled = view.connection.is_connected();

  egui::Frame::new().inner_margin(Metrics::RIGHT_PAD).show(ui, |ui| {
    override_axis(ui, "Feed", OverrideAxis::Feed, feed, enabled, &mut state.feed_override_drag, sink,
      RealtimeCommand::FeedOverrideMinus1, RealtimeCommand::FeedOverrideMinus10, RealtimeCommand::FeedOverrideReset,
      RealtimeCommand::FeedOverridePlus10, RealtimeCommand::FeedOverridePlus1);
    ui.add_space(4.0);
    override_axis(ui, "Spindle", OverrideAxis::Spindle, spindle, enabled, &mut state.spindle_override_drag, sink,
      RealtimeCommand::SpindleOverrideMinus1, RealtimeCommand::SpindleOverrideMinus10,
      RealtimeCommand::SpindleOverrideReset, RealtimeCommand::SpindleOverridePlus10,
      RealtimeCommand::SpindleOverridePlus1);
    ui.add_space(4.0);

    // Rapid override is preset-only in grbl (100/50/25), so it gets buttons rather than a slider.
    ui.horizontal(|ui| {
      ui.label(format!("Rapid {rapid:>3}%"));
      if ui.button("100").clicked() {
        sink.push(Intent::Realtime(RealtimeCommand::RapidOverrideReset));
      }
      if ui.button("50").clicked() {
        sink.push(Intent::Realtime(RealtimeCommand::RapidOverride50));
      }
      if ui.button("25").clicked() {
        sink.push(Intent::Realtime(RealtimeCommand::RapidOverride25));
      }
    });

    // Realized feed/speed from the latest report: what the machine is actually doing after overrides.
    if let Some((feed, rpm, actual)) = view.status.as_ref().and_then(|s| s.feed_speed) {
      ui.add_space(6.0);
      egui::Frame::new().fill(Theme::INSET).inner_margin(egui::Margin::symmetric(12, 8)).corner_radius(2.0)
        .show(ui, |ui| {
          ui.horizontal(|ui| {
            ui.label(RichText::new("Realized F").size(10.5).color(Theme::TEXT_DIM));
            ui.label(RichText::new(format!("{feed:.0} mm/min")).monospace().color(Theme::TEXT));
          });
          ui.horizontal(|ui| {
            ui.label(RichText::new("Realized S").size(10.5).color(Theme::TEXT_DIM));
            let shown = actual.unwrap_or(rpm);
            ui.label(RichText::new(format!("{shown:.0} RPM")).monospace().color(Theme::TEXT));
          });
        });
    }
  });
}

/// A custom override slider matching the design's filled-bar look (design §03/§04): a 6px recessed inset track
/// with an accent-coloured fill that grows from the left in proportion to the value across the 10–200% span,
/// plus a thin handle at the fill edge for a grab affordance. egui's stock `Slider` rendered only a grey rail
/// and a square knob — no colour at all (the user-flagged "missing colors entirely"). The strip is taller than
/// the 6px track so it is easy to grab; dragging or clicking maps the pointer x onto the value and reports the
/// change through the returned [`egui::Response`] so the caller's commit/mirror logic is unchanged.
fn override_slider(ui: &mut egui::Ui, value: &mut u32, fill_color: Color32) -> egui::Response {
  use super::overrides::{OVERRIDE_MAX, OVERRIDE_MIN, OVERRIDE_NEUTRAL};
  let width = ui.available_width().max(48.0);
  let (rect, mut response) = ui.allocate_exact_size(Vec2::new(width, 18.0), egui::Sense::click_and_drag());
  let track = egui::Rect::from_center_size(rect.center(), Vec2::new(width, Metrics::SLIDER_H));
  let span = (OVERRIDE_MAX - OVERRIDE_MIN) as f32;

  // Pointer drives the value: map its x across the track onto the 10–200% span while pressed/dragged.
  if (response.dragged() || response.clicked())
    && let Some(pos) = response.interact_pointer_pos()
  {
    let frac = ((pos.x - track.left()) / track.width()).clamp(0.0, 1.0);
    let next = OVERRIDE_MIN + (frac * span).round() as u32;
    if next != *value {
      *value = next;
      response.mark_changed();
    }
  }

  // The coloured fill reads against the *nominal* 100%, so a neutral 100% override shows a full bar (design
  // §03) and reducing the override shrinks it; at or above 100% the bar saturates full. The handle, by
  // contrast, sits at the override's true position across the full 10–200% drag span, so it still tracks the
  // pointer all the way to 200% and the 100–200% range stays adjustable — the fill is the at-a-glance gauge,
  // the handle is the precise position.
  let fill_frac = (*value as f32 / OVERRIDE_NEUTRAL as f32).clamp(0.0, 1.0);
  let pos_frac = (value.saturating_sub(OVERRIDE_MIN)) as f32 / span;
  let radius = egui::CornerRadius::same(Metrics::CONTROL_RADIUS);
  let painter = ui.painter();
  painter.rect_filled(track, radius, Theme::INSET);
  let mut fill = track;
  fill.set_width(track.width() * fill_frac);
  painter.rect_filled(fill, radius, fill_color);
  painter.rect_stroke(track, radius, egui::Stroke::new(1.0, Theme::BORDER_RECESS), egui::StrokeKind::Inside);
  // A 2px handle at the override's true position, brightened to the text colour, so the operator sees the grab
  // point and can read where in the 10–200% span the value sits even while the fill is saturated full.
  let handle_x = (track.left() + track.width() * pos_frac.clamp(0.0, 1.0)).clamp(track.left(), track.right());
  let handle = egui::Rect::from_center_size(egui::pos2(handle_x, track.center().y), Vec2::new(2.0, 14.0));
  painter.rect_filled(handle, egui::CornerRadius::ZERO, Theme::TEXT);
  response
}

/// Render one override axis (feed or spindle): a label with the live percentage, a 10–200% slider that emits
/// an absolute [`Intent::SetOverride`] on release, and a fine/coarse/reset stepper row (`−10 −1 100 +1 +10`)
/// that emits single relative real-time bytes. The slider and the steppers are two equivalent ways to reach
/// the same override; the slider is coarse-grained reach, the steppers are precise nudges including the new
/// fine ±1%.
///
/// `live` is the override the firmware last reported. `feedback` is the slider's transient feedback state: it
/// mirrors `live` while idle (so the firmware's truth re-centers the handle), holds the operator's position
/// during a drag (so a status poll cannot yank it), and — the snap-back fix — holds the committed target after
/// release until the firmware's relative ramp converges onto it (see [`super::overrides::OverrideFeedback`]). On
/// release we emit the target only if it moved off `live`, so merely touching the slider sends nothing. The
/// whole row is disabled unless a live link can act on the override (`enabled`), since overrides are no-ops
/// otherwise.
#[allow(clippy::too_many_arguments)]
fn override_axis(ui: &mut egui::Ui, label: &str, axis: super::overrides::OverrideAxis, live: u32, enabled: bool,
  feedback: &mut super::overrides::OverrideFeedback, sink: &mut IntentSink, minus1: RealtimeCommand,
  minus10: RealtimeCommand, reset: RealtimeCommand, plus10: RealtimeCommand, plus1: RealtimeCommand) {
  use super::overrides::OverrideAxis;

  // Fold the latest live report into the feedback state first: while idle the handle follows `live`; a
  // post-release hold releases once `live` converges onto the committed target. A drag ignores reports entirely.
  feedback.observe(live);
  // The value the handle shows this frame: `live` when idle, the pinned drag/hold value otherwise.
  let mut value = feedback.display(live);
  // Feed (and rapid) carry the cool control-blue fill; spindle carries the warm motion-orange, matching the
  // design's `#0E86D4` feed bar and `#FF7A1A` spindle bar (the colour that was missing entirely before).
  let fill_color = match axis {
    OverrideAxis::Feed => Theme::ACCENT,
    OverrideAxis::Spindle => Theme::ACCENT_MOTION,
  };
  ui.add_enabled_ui(enabled, |ui| {
    // Row: dim label on the left, the filled track stretching across the middle, the live percent on the right.
    ui.horizontal(|ui| {
      ui.label(RichText::new(label).size(11.0).color(Theme::TEXT_DIM));
      ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
        ui.label(RichText::new(format!("{value:>3}%")).monospace().size(11.5).color(Theme::TEXT));
        let slider = override_slider(ui, &mut value, fill_color);
        // A UI test (egui_kittest) needs the slider's on-screen rect to drive a pointer drag at known
        // coordinates, since the widget is hand-painted and label-less so AccessKit cannot locate it by text.
        #[cfg(all(test, feature = "gui"))]
        slider_rect_probe::record(axis, slider.rect);
        if slider.dragged() {
          // While dragging, hold the operator's position so the live status poll cannot yank the handle back.
          feedback.drag_to(value);
        }
        if slider.drag_stopped() || slider.clicked() {
          // On release/click, commit the target if it moved off the live value and HOLD it; the hold survives the
          // firmware's relative ramp so the handle stays put instead of snapping to the stale `live` and crawling.
          if value != live {
            sink.push(Intent::SetOverride { axis, target: value });
            feedback.commit(value, live);
          } else {
            // An empty touch (no movement): nothing committed, so just return to mirroring live.
            *feedback = super::overrides::OverrideFeedback::Idle;
          }
        }
      });
    });
  });

  // The stepper row: fine ±1% (the new control) flanks coarse ±10% around a reset-to-100%. Gated on the same
  // live link as the slider — a relative override byte is a no-op with nothing connected to act on it.
  ui.add_enabled_ui(enabled, |ui| {
    ui.horizontal(|ui| {
      if ui.button("−10").clicked() {
        sink.push(Intent::Realtime(minus10));
      }
      if ui.button("−1").clicked() {
        sink.push(Intent::Realtime(minus1));
      }
      if ui.button("100").clicked() {
        sink.push(Intent::Realtime(reset));
      }
      if ui.button("+1").clicked() {
        sink.push(Intent::Realtime(plus1));
      }
      if ui.button("+10").clicked() {
        sink.push(Intent::Realtime(plus10));
      }
    });
  });
}

/// A test-only side channel that records each override slider's on-screen [`egui::Rect`] as it is rendered, so
/// the egui_kittest harness can compute pointer coordinates inside a slider it cannot otherwise locate (the
/// widget is hand-painted with no label, so AccessKit exposes no findable node). Compiled only for the
/// gui-featured test build; the production render path is untouched apart from the cheap `record` call.
#[cfg(all(test, feature = "gui"))]
pub(crate) mod slider_rect_probe {
  use super::super::overrides::OverrideAxis;
  use eframe::egui::Rect;
  use std::cell::Cell;

  thread_local! {
    /// The last-rendered rect of the feed and spindle sliders, in screen coordinates. `None` until first drawn.
    static FEED: Cell<Option<Rect>> = const { Cell::new(None) };
    static SPINDLE: Cell<Option<Rect>> = const { Cell::new(None) };
  }

  /// Record the slider rect for `axis` from the just-completed render of that axis's row.
  pub(crate) fn record(axis: OverrideAxis, rect: Rect) {
    match axis {
      OverrideAxis::Feed => FEED.with(|c| c.set(Some(rect))),
      OverrideAxis::Spindle => SPINDLE.with(|c| c.set(Some(rect))),
    }
  }

  /// The last-recorded rect for `axis`, or `None` if that axis has not been rendered yet this thread.
  pub(crate) fn last(axis: OverrideAxis) -> Option<Rect> {
    match axis {
      OverrideAxis::Feed => FEED.with(Cell::get),
      OverrideAxis::Spindle => SPINDLE.with(Cell::get),
    }
  }
}

/// Render the probe panel: depth/feed/plate inputs and a "Probe Z" action that the shell sequences.
pub fn probe(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  section_header(ui, "Probe Z · no plate");
  egui::Frame::new().inner_margin(Metrics::RIGHT_PAD).show(ui, |ui| {
    ui.label(RichText::new("No-touch-plate Z zero. Lower until continuity, set Z = 0.").size(11.0)
      .color(Theme::TEXT_DIM));
    ui.add_space(4.0);
    let enabled = view.connection == ConnectionState::Idle;
    ui.add_enabled_ui(enabled, |ui| {
      egui::Grid::new("probe").num_columns(2).show(ui, |ui| {
        ui.label("Depth");
        ui.add(egui::DragValue::new(&mut state.probe_depth).speed(0.5).range(0.1..=200.0).suffix(" mm"));
        ui.end_row();
        ui.label("Feed");
        ui.add(egui::DragValue::new(&mut state.probe_feed).speed(5.0).range(1.0..=500.0).suffix(" mm/min"));
        ui.end_row();
        ui.label("Plate");
        ui.add(egui::DragValue::new(&mut state.plate_thickness).speed(0.05).range(0.0..=20.0).suffix(" mm"));
        ui.end_row();
      });
      // The primary probe action is a full-width button, per the design's right-column treatment.
      let probe_button = egui::Button::new("Probe Z → set work-zero");
      if ui.add_sized(Vec2::new(ui.available_width(), Metrics::PANEL_CONTROL_H + 6.0), probe_button).clicked() {
        sink.push(Intent::ProbeZ {
          depth: state.probe_depth,
          feed: state.probe_feed,
          plate_thickness: state.plate_thickness,
        });
      }
    });
    // Render the latched probe result below the action so the operator sees the probed point + success here
    // rather than hunting for it in the console. Shows a live "Probing…" while awaiting, the contact point on
    // success, or the failure reason — driven entirely by the reducer's `probe_op` latch.
    probe_result(ui, view);
  });
}

/// Render the probe-operation latch outcome inside the Z touch-off panel: a "Probing…" spinner-line while
/// awaiting, the contact point on success, or the failure reason. Reads only [`ViewState::probe_op`] — a thin
/// render of the pure latch. Draws nothing when no probe has been issued, OR when the latched op belongs to a
/// DIFFERENT flow (a rotary wizard touch): this panel speaks only for the ZeroZ touch-off, so it must not claim
/// "Work-Z set." for a rotary probe (the wizard has its own panel). Routing on `op.kind` is what keeps the two
/// panels from narrating each other's probes.
fn probe_result(ui: &mut egui::Ui, view: &ViewState) {
  use super::view_state::{ProbeKind, ProbeOutcome};
  let Some(op) = view.probe_op.as_ref().filter(|op| op.kind == ProbeKind::ZeroZ) else {
    return;
  };
  ui.add_space(6.0);
  if op.awaiting {
    ui.label(RichText::new("Probing… awaiting result").size(11.0).color(Theme::TEXT_DIM));
    return;
  }
  match op.last.as_ref() {
    Some(ProbeOutcome::Success { position }) => {
      // Show the machine-coordinate contact point (X, Y, Z, then any rotary axis) at 3 decimals, the PRB report
      // precision. A short green confirmation reads as "done" without re-reading the console.
      let coords = position.iter().map(|v| format!("{v:.3}")).collect::<Vec<_>>().join(", ");
      ui.label(RichText::new(format!("Contact at [{coords}]")).size(11.0).color(Theme::OK));
      ui.label(RichText::new("Work-Z set.").size(11.0).color(Theme::TEXT_DIM));
    }
    Some(ProbeOutcome::Failure { reason }) => {
      ui.label(RichText::new(format!("Probe failed: {reason}")).size(11.0).color(Theme::DANGER));
      ui.label(RichText::new("Work-Z unchanged.").size(11.0).color(Theme::TEXT_DIM));
    }
    // Resolved but no outcome recorded — unreachable in practice (resolving always sets `last`), but render
    // nothing rather than assume.
    None => {}
  }
}

/// Render the rotary center-finder wizard (DOC-11 §1.2). When no run is active it shows the dowel-diameter /
/// index-angle inputs and a Start button; when a run is active it guides the operator step by step — issuing each
/// rotary-safe touch, the move-to-Y-center, and the WCS write — and shows the captured readings + computed
/// `(Y_c, Z_c)`. The wizard state is owned by the shell (the firmware has no pivot concept) and passed in as a
/// borrow, so this stays a pure render that only emits intents.
pub fn rotary_center(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState,
  wizard: Option<&super::rotary_center::WizardState>, has_saved_center: bool, sink: &mut IntentSink) {
  use super::rotary_center::WizardStep;
  section_header(ui, "Rotary center-finder");
  egui::Frame::new().inner_margin(Metrics::RIGHT_PAD).show(ui, |ui| {
    let Some(w) = wizard else {
      // No run: collect the dowel diameter + index angle and offer Start. Only meaningful while idle/connected,
      // but the inputs stay editable so the operator can set up before connecting.
      ui.label(RichText::new("Find the A centerline from a known-diameter dowel clamped concentric.").size(11.0)
        .color(Theme::TEXT_DIM));
      ui.add_space(4.0);
      egui::Grid::new("rotary_setup").num_columns(2).show(ui, |ui| {
        ui.label("Dowel ⌀");
        ui.add(egui::DragValue::new(&mut state.rotary_dowel_diameter).speed(0.1).range(0.1..=100.0).suffix(" mm"));
        ui.end_row();
        ui.label("A angle");
        ui.add(egui::DragValue::new(&mut state.rotary_index_angle).speed(1.0).range(-360.0..=360.0).suffix(" °"));
        ui.end_row();
        // The side-probe Z is SAFETY-CRITICAL and setup-specific, so it lives in the always-visible setup rather
        // than the collapsed bench section: every center-finder run descends a side touch to it, and a wrong value
        // crashes into the part or misses the flank (finding #13). Editing it re-arms the confirmation below.
        ui.label("Side-probe Z");
        if ui.add(egui::DragValue::new(&mut state.rotary_bench.side_probe_z).speed(0.1).range(-300.0..=0.0)
          .suffix(" mm")).on_hover_text("Machine-Z the side (X/Y) touches descend to — must lie within the dowel's \
          Z-extent. A wrong value will crash into the part or miss the flank.").changed()
        {
          state.rotary_side_probe_confirmed = false;
        }
        ui.end_row();
      });
      // The remaining bench-tuned parameters stay in a collapsing section so the common path is just the inputs
      // above. The side-probe Z is hoisted out of it (above) because it is the crash-risk parameter.
      rotary_bench_params(ui, state);
      // Gate Start on an explicit acknowledgement that the side-probe Z is tuned for THIS bench. The default is a
      // conservative placeholder; an untuned descent is a crash risk, so the operator must confirm before a run.
      ui.add_space(4.0);
      ui.checkbox(&mut state.rotary_side_probe_confirmed,
        RichText::new(format!("Side-probe Z {:.3} mm is set for this dowel", state.rotary_bench.side_probe_z))
          .size(11.0).color(Theme::TEXT_DIM));
      let enabled = view.connection == ConnectionState::Idle && state.rotary_side_probe_confirmed;
      ui.add_enabled_ui(enabled, |ui| {
        if ui.add_sized(Vec2::new(ui.available_width(), Metrics::PANEL_CONTROL_H + 6.0),
          egui::Button::new("Start center-finder")).clicked()
        {
          sink.push(Intent::RotaryCenterStart {
            dowel_diameter: state.rotary_dowel_diameter,
            index_angle_deg: state.rotary_index_angle,
            params: state.rotary_bench,
          });
        }
      });
      // If a center was saved last session (DOC-11 §1.3), offer to re-apply it to the active WCS without
      // re-running the center-finder. Enabled only when Idle (the `G10` needs an accepting machine).
      if has_saved_center {
        ui.add_space(4.0);
        ui.add_enabled_ui(enabled, |ui| {
          if ui.add_sized(Vec2::new(ui.available_width(), Metrics::PANEL_CONTROL_H + 6.0),
            egui::Button::new("Apply saved center")).clicked()
          {
            sink.push(Intent::ApplySavedRotaryCenter);
          }
        });
      }
      return;
    };

    // A run is active: render the step guidance, the readings so far, and the step's action button. Probing
    // disables the action (one touch at a time); the latch's awaiting/result is shown by the probe panel above.
    rotary_run_readings(ui, w);
    ui.add_space(6.0);
    let probing = w.is_probing();
    let idle = view.connection == ConnectionState::Idle;
    // The action available depends on the step; each is gated on Idle (a probe/move needs an accepting machine)
    // and disabled while a touch is in flight.
    let full = Vec2::new(ui.available_width(), Metrics::PANEL_CONTROL_H + 6.0);
    match w.step {
      WizardStep::EnterDowel => {
        ui.label(RichText::new("Jog to the −Y face approach, then probe.").size(11.0).color(Theme::TEXT_DIM));
        ui.add_enabled_ui(idle && !probing, |ui| {
          if ui.add_sized(full, egui::Button::new("Probe Y (left side)")).clicked() {
            sink.push(Intent::RotaryCenterProbe);
          }
        });
      }
      WizardStep::ReadyYRight => {
        ui.label(RichText::new("Jog to the +Y face approach, then probe.").size(11.0).color(Theme::TEXT_DIM));
        ui.add_enabled_ui(idle && !probing, |ui| {
          if ui.add_sized(full, egui::Button::new("Probe Y (right side)")).clicked() {
            sink.push(Intent::RotaryCenterProbe);
          }
        });
      }
      WizardStep::MoveToYc => {
        // ONLY the move is offered here — the top probe is locked until the move has actually been sent (the
        // wizard then advances to MovedToYc). This is the UI half of the type-enforced "move before top" order.
        ui.label(RichText::new("Move to the Y center before probing the top.").size(11.0).color(Theme::TEXT_DIM));
        ui.add_enabled_ui(idle, |ui| {
          if ui.add_sized(full, egui::Button::new("Move to Y center")).clicked() {
            sink.push(Intent::RotaryCenterMoveToYc);
          }
        });
      }
      WizardStep::MovedToYc => {
        // The move was sent; now (and only now) the top probe is offered.
        ui.label(RichText::new("At the Y center. Probe the dowel top.").size(11.0).color(Theme::TEXT_DIM));
        ui.add_enabled_ui(idle && !probing, |ui| {
          if ui.add_sized(full, egui::Button::new("Probe Z (dowel top)")).clicked() {
            sink.push(Intent::RotaryCenterProbe);
          }
        });
      }
      WizardStep::Review => {
        ui.label(RichText::new("Center found. Write it to the active WCS (Y/Z only).").size(11.0)
          .color(Theme::OK));
        // The operator picks which feature work-Z0 lands on. Y0 is always the axis centerline; only Z is
        // selectable. Defaults to the axis centerline (wrap-machining convention).
        rotary_z_datum_picker(ui, w, sink);
        ui.add_enabled_ui(idle, |ui| {
          if ui.add_sized(full, egui::Button::new("Write center → WCS (G10 L2)")).clicked() {
            sink.push(Intent::RotaryCenterWriteWcs);
          }
        });
      }
      WizardStep::Aborted => {
        let reason = w.abort_reason.as_deref().unwrap_or("cancelled");
        ui.label(RichText::new(format!("Aborted: {reason}")).size(11.0).color(Theme::DANGER));
      }
      // The probing steps await a result (the probe panel shows it); only Cancel is offered here.
      WizardStep::ProbeYLeft | WizardStep::ProbeYRight | WizardStep::ProbeZTop => {
        ui.label(RichText::new("Probing… awaiting result.").size(11.0).color(Theme::TEXT_DIM));
      }
    }
    ui.add_space(4.0);
    if ui.add_sized(full, egui::Button::new("Cancel")).clicked() {
      sink.push(Intent::RotaryCenterCancel);
    }
  });
}

/// Render the editable bench-tuned rotary-touch parameters in a collapsing "Bench params" section, mutating
/// `state.rotary_bench` in place. Collapsed by default so the common setup is just dowel ⌀ + A angle; opened when
/// the operator needs to dial in a bench. The crash-risk side-probe Z is NOT here — it is hoisted into the
/// always-visible setup grid and gated behind a confirmation (finding #13); this section holds the rest.
fn rotary_bench_params(ui: &mut egui::Ui, state: &mut UiState) {
  let p = &mut state.rotary_bench;
  egui::CollapsingHeader::new(RichText::new("Bench params").size(11.0).color(Theme::TEXT_DIM))
    .id_salt("rotary_bench_params")
    .show(ui, |ui| {
      egui::Grid::new("rotary_bench").num_columns(2).show(ui, |ui| {
        ui.label("Clearance Z");
        ui.add(egui::DragValue::new(&mut p.clearance_mm).speed(0.1).range(-300.0..=0.0).suffix(" mm"))
          .on_hover_text("Machine-Z retract above the dowel before the A index (G53).");
        ui.end_row();
        // Side-probe Z is intentionally NOT here — it is hoisted into the always-visible setup grid above (finding
        // #13) because it is the crash-risk parameter and must be confirmed before a run, not buried in a collapse.
        ui.label("Settle");
        ui.add(egui::DragValue::new(&mut p.settle_secs).speed(0.05).range(0.0..=10.0).suffix(" s"))
          .on_hover_text("Dwell after the A index so backlash/oscillation damps out before the probe.");
        ui.end_row();
        ui.label("Feed");
        ui.add(egui::DragValue::new(&mut p.feed).speed(1.0).range(1.0..=2000.0).suffix(" mm/min"))
          .on_hover_text("Probe feed for the G38.2 touch.");
        ui.end_row();
        ui.label("Depth");
        ui.add(egui::DragValue::new(&mut p.depth_mm).speed(0.1).range(0.1..=200.0).suffix(" mm"))
          .on_hover_text("How far the probe advances seeking contact before it gives up (alarms).");
        ui.end_row();
      });
    });
}

/// Render the rotary wizard's captured readings + computed center as a compact dim list. Each is shown once
/// available; `Y_c`/`Z_c` appear as the math resolves them. Pure render of [`super::rotary_center::WizardState`].
fn rotary_run_readings(ui: &mut egui::Ui, w: &super::rotary_center::WizardState) {
  let dim = |ui: &mut egui::Ui, text: String| {
    ui.label(RichText::new(text).size(11.0).color(Theme::TEXT_DIM));
  };
  dim(ui, format!("Dowel ⌀ {:.3} mm · A {:.1}°", w.dowel_diameter, w.index_angle_deg));
  if let Some(y) = w.y_left {
    dim(ui, format!("Y left  {y:.3}"));
  }
  if let Some(y) = w.y_right {
    dim(ui, format!("Y right {y:.3}"));
  }
  if let Some(yc) = w.y_center() {
    ui.label(RichText::new(format!("Y center {yc:.3}")).size(11.0).color(Theme::TEXT));
  }
  if let Some(z) = w.z_top {
    dim(ui, format!("Z top   {z:.3}"));
  }
  if let Some(zc) = w.z_center() {
    ui.label(RichText::new(format!("Z center {zc:.3}")).size(11.0).color(Theme::TEXT));
  }
}

/// Render the Z-datum picker for the WCS write: two selectable labels — the rotary axis centerline (default) or
/// the probed top surface — plus a one-line clarification and a preview of which Z the offered `G10` will use.
/// Emits [`Intent::RotaryCenterSetZDatum`] on a change; pure render of the wizard's current selection.
fn rotary_z_datum_picker(ui: &mut egui::Ui, w: &super::rotary_center::WizardState, sink: &mut IntentSink) {
  use super::rotary_center::ZDatum;
  ui.add_space(4.0);
  ui.label(RichText::new("Work-Z0 datum").size(11.0).color(Theme::TEXT_DIM));
  ui.horizontal(|ui| {
    let axis = w.z_datum == ZDatum::AxisCenterline;
    let top = w.z_datum == ZDatum::TopSurface;
    if ui.selectable_label(axis, "Axis centerline").clicked() && !axis {
      sink.push(Intent::RotaryCenterSetZDatum(ZDatum::AxisCenterline));
    }
    if ui.selectable_label(top, "Top surface").clicked() && !top {
      sink.push(Intent::RotaryCenterSetZDatum(ZDatum::TopSurface));
    }
  });
  // One-line clarification of the selected datum, plus the Z value the G10 will carry.
  let (desc, z) = match w.z_datum {
    ZDatum::AxisCenterline => ("Z0 at the rotary axis (Z_top − D/2).", w.z_datum_value()),
    ZDatum::TopSurface => ("Z0 at the probed top surface (Z_top).", w.z_datum_value()),
  };
  ui.label(RichText::new(desc).size(11.0).color(Theme::TEXT_DIM));
  if let Some(z) = z {
    ui.label(RichText::new(format!("G10 will set Z {z:.3}")).size(11.0).color(Theme::TEXT_DIM));
  }
}

/// Render the Phase 2 verify/measure panel (DOC-11 §2): the 180°-flip center-verify and the runout report, both
/// driven by the shared [`super::angle_sweep::AngleSweep`] engine (passed as `(sweep, kind)` when one is running).
/// When idle it offers both Start actions; while running it guides the per-angle touches and shows the readings;
/// on completion it computes the flip residual (with a `G10` correction offer) or the runout TIR/eccentricity
/// (read-only). The probes use the conventional Y radial axis (matching the center-finder), probing toward −Y.
pub fn verify_measure(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState,
  sweep: Option<(&super::angle_sweep::AngleSweep, super::view_state::ProbeKind)>, sink: &mut IntentSink) {
  use super::angle_sweep::SweepStep;
  use super::intent::{Axis, Dir};
  use super::view_state::ProbeKind;
  section_header(ui, "Verify · measure");
  egui::Frame::new().inner_margin(Metrics::RIGHT_PAD).show(ui, |ui| {
    let full = Vec2::new(ui.available_width(), Metrics::PANEL_CONTROL_H + 6.0);
    let idle = view.connection == ConnectionState::Idle;
    let Some((s, kind)) = sweep else {
      // No run: collect the shared start angle + (for runout) N, and offer both Start actions.
      ui.label(RichText::new("180°-flip verify or N-angle runout, probing −Y.").size(11.0).color(Theme::TEXT_DIM));
      ui.add_space(4.0);
      egui::Grid::new("verify_setup").num_columns(2).show(ui, |ui| {
        ui.label("Start A");
        ui.add(egui::DragValue::new(&mut state.verify_start_angle).speed(1.0).range(-360.0..=360.0).suffix(" °"));
        ui.end_row();
        ui.label("Runout N");
        ui.add(egui::DragValue::new(&mut state.verify_runout_n).range(2..=36));
        ui.end_row();
      });
      ui.add_enabled_ui(idle, |ui| {
        if ui.add_sized(full, egui::Button::new("Start 180°-flip verify")).clicked() {
          sink.push(Intent::FlipVerifyStart { angle_deg: state.verify_start_angle, axis: Axis::Y, dir: Dir::Neg });
        }
        if ui.add_sized(full, egui::Button::new("Start runout report")).clicked() {
          sink.push(Intent::RunoutStart {
            n: state.verify_runout_n,
            start_deg: state.verify_start_angle,
            axis: Axis::Y,
            dir: Dir::Neg,
          });
        }
      });
      return;
    };

    let title = match kind {
      ProbeKind::FlipVerify => "180°-flip verify",
      ProbeKind::Runout => "Runout report",
      _ => "Verify",
    };
    ui.label(RichText::new(title).size(11.0).color(Theme::TEXT));
    verify_readings_table(ui, s);
    ui.add_space(6.0);
    match s.step() {
      SweepStep::Ready => {
        ui.label(RichText::new(format!("Jog the approach for touch {} of {}, then probe.",
          s.current_touch_number(), s.total_touches())).size(11.0).color(Theme::TEXT_DIM));
        ui.add_enabled_ui(idle, |ui| {
          if ui.add_sized(full, egui::Button::new("Probe this angle")).clicked() {
            sink.push(Intent::SweepProbe);
          }
        });
      }
      SweepStep::Probing => {
        ui.label(RichText::new("Probing… awaiting result.").size(11.0).color(Theme::TEXT_DIM));
      }
      SweepStep::Done => verify_done(ui, state, s, kind, idle, full, sink),
      SweepStep::Aborted => {
        let reason = s.abort_reason().unwrap_or("cancelled");
        ui.label(RichText::new(format!("Aborted: {reason}")).size(11.0).color(Theme::DANGER));
      }
    }
    ui.add_space(4.0);
    if ui.add_sized(full, egui::Button::new("Cancel")).clicked() {
      sink.push(Intent::SweepCancel);
    }
  });
}

/// Render the completed-sweep result: the flip-verify residual + `G10` correction offer, or the read-only runout
/// TIR / eccentricity. Pure render of the computed values over the sweep's readings.
fn verify_done(ui: &mut egui::Ui, _state: &mut UiState, s: &super::angle_sweep::AngleSweep,
  kind: super::view_state::ProbeKind, idle: bool, full: Vec2, sink: &mut IntentSink) {
  use super::flip_verify::FlipResult;
  use super::runout::RunoutReport;
  use super::view_state::ProbeKind;
  match kind {
    ProbeKind::FlipVerify => {
      let Some(result) = FlipResult::from_readings(s.probe_axis(), s.readings()) else {
        ui.label(RichText::new("Flip verify needs two readings.").size(11.0).color(Theme::DANGER));
        return;
      };
      ui.label(RichText::new(format!("Residual eccentricity {:.3} mm.", result.error()))
        .size(11.0).color(Theme::OK));
      ui.label(RichText::new("Apply shifts the active WCS origin on this axis by the residual.")
        .size(11.0).color(Theme::TEXT_DIM));
      ui.add_enabled_ui(idle, |ui| {
        if ui.add_sized(full, egui::Button::new("Apply correction → WCS (G10 L2)")).clicked() {
          sink.push(Intent::FlipVerifyWriteCorrection);
        }
      });
    }
    ProbeKind::Runout => match RunoutReport::from_readings(s.readings()) {
      Some(r) => {
        ui.label(RichText::new(format!("TIR {:.3} mm · eccentricity {:.3} mm ({} pts)", r.tir, r.eccentricity,
          r.count)).size(11.0).color(Theme::OK));
        ui.label(RichText::new("Read-only — no offset written.").size(11.0).color(Theme::TEXT_DIM));
      }
      None => {
        ui.label(RichText::new("Runout needs at least two readings.").size(11.0).color(Theme::DANGER));
      }
    },
    _ => {}
  }
}

/// Render the sweep's per-angle readings as a compact dim list (angle → reading once captured). Pure render of
/// the shared [`super::angle_sweep::AngleSweep`].
fn verify_readings_table(ui: &mut egui::Ui, s: &super::angle_sweep::AngleSweep) {
  let readings = s.readings();
  for (i, &angle) in s.angles().iter().enumerate() {
    let text = match readings.get(i) {
      Some(r) => format!("A{angle:.1}°  →  {r:.3}"),
      None => format!("A{angle:.1}°  →  —"),
    };
    ui.label(RichText::new(text).size(11.0).color(Theme::TEXT_DIM));
  }
}

/// Render the bottom dock (design §03): one surface hosting the Console and Program tabs. The shared tab strip
/// switches `state.active_tab`, the strip's right edge carries the §03 progress readout (acked/total · 260px
/// bar · percent) for whichever tab is active, and the body below renders the selected tab. Keeping both tabs
/// in one dock matches the mock, where Console and Program share the 200px dock rather than sitting in
/// separate panels.
pub fn dock(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, time: super::progress::TimeEstimate,
  eta_qualifier: Option<EtaQualifier>, sink: &mut IntentSink) {
  let active = state.active_tab;
  let tabs = [("Console", active == DockTab::Console), ("Program", active == DockTab::Program)];
  let progress = view.progress;
  let collapsed = state.dock_collapsed;
  // The strip's right closure lays out right-to-left, so widgets are drawn outermost-right first. The collapse
  // toggle is the outermost-right control (the conventional minimise corner); the §03 progress block sits to its
  // left. The progress rides on the strip regardless of the active tab — it is a dock-level affordance — so the
  // operator always sees streaming progress while reading either tab, even when the body is collapsed.
  let mut toggle_clicked = false;
  let clicked = tab_strip(ui, &tabs, |ui| {
    toggle_clicked = dock_collapse_toggle(ui, collapsed);
    dock_progress(ui, progress, time, eta_qualifier);
  });
  state.active_tab = dock_tab_for_click(active, clicked);
  if toggle_clicked {
    state.dock_collapsed = !state.dock_collapsed;
  }

  // When collapsed only the tab strip remains; the body is hidden and the shell pins the panel to the strip's
  // height so the central viewport reclaims the freed space. The tab strip stays interactive so the operator can
  // still switch tabs and re-expand.
  if state.dock_collapsed {
    return;
  }
  ui.add_space(2.0);
  match state.active_tab {
    DockTab::Console => console_body(ui, view, state, sink),
    DockTab::Program => program_body(ui, view, state),
  }
}

/// Draw the dock's collapse/expand toggle as a small ghost icon button at the right of the tab strip: a `−`
/// minimises the dock to its strip, a `+` restores it (see [`dock_toggle_label`]). Sized to the strip's control
/// height so it sits centred in the 30px bar. Returns whether it was clicked this frame.
fn dock_collapse_toggle(ui: &mut egui::Ui, collapsed: bool) -> bool {
  let label = dock_toggle_label(collapsed);
  let hint = if collapsed { "Expand dock" } else { "Collapse dock" };
  // Square icon button matching the strip's control height, transparent at rest like the §02 icon-button state
  // (the same ghost treatment as the ⚙ settings and jog-cancel buttons), so it reads as chrome, not a tab. Zero
  // the button padding for this region: the global `BUTTON_PAD` (6px vertical) plus the glyph would inflate the
  // button past the strip's control height, making it overflow the 30px bar and sit off-centre (the user-flagged
  // bug). With no padding the button is pinned to the `PANEL_CONTROL_H` square, which the strip's `Align::Center`
  // layout then centres within the 30px bar. The glyph is held at the header text size so it can't grow the box.
  ui.spacing_mut().button_padding = Vec2::ZERO;
  let size = Vec2::splat(Metrics::PANEL_CONTROL_H);
  let button = egui::Button::new(RichText::new(label).size(Metrics::HEADER_TEXT).color(Theme::TEXT_DIM))
    .fill(Color32::TRANSPARENT);
  ui.add_sized(size, button).on_hover_text(hint).clicked()
}

/// Draw the §03 dock progress readout: `acked / total`, the green bar, the percent, and the elapsed /
/// estimated-total `m:ss / m:ss` clock, shown only while a program is loaded/streaming (`total > 0`). The
/// strip's right closure lays out right-to-left, so the widgets are drawn rightmost-first; that puts the
/// clock at the left edge of the block and the count nearest the percent, reading left→right as the design's
/// `acked/total · bar · NN% · m:ss / m:ss` with dim `·` separators between the textual fields.
///
/// The strip's `right` closure inherits the tab row's `item_spacing.x = 0` (it is the same child `ui`), which is
/// why the percent and clock previously ran together with no gap. This sets its own roomy row spacing so the
/// fields breathe, and degrades on a narrow strip by dropping the bar first (the least-important field — the
/// percent and count carry the same information) so the block never overflows into an unpainted gap.
fn dock_progress(ui: &mut egui::Ui, progress: super::view_state::Progress, time: super::progress::TimeEstimate,
  eta_qualifier: Option<EtaQualifier>) {
  use super::progress::format_progress_clock;
  // Show the readout while a program is streaming (`total > 0`) OR a simulation is stored — the latter surfaces the
  // upfront ETA before any stream begins, when the progress total is still zero. With neither, there is nothing to
  // show; bail so the strip stays bare.
  let has_simulation = eta_qualifier.is_some();
  if progress.total == 0 && !has_simulation {
    return;
  }
  // Own the row spacing rather than inheriting the tab strip's zeroed `item_spacing.x`. A roomy 8px gap gives
  // every field air and sits between the adjacent fields and the `·` separators so nothing abuts its neighbour.
  ui.spacing_mut().item_spacing.x = Metrics::DOCK_PROGRESS_GAP;
  // Decide up front whether the bar fits. The toggle was already drawn (this `ui` excludes it), so the remaining
  // width must hold the bar plus the textual fields; when it can't, drop the bar rather than overflow the strip.
  // The bar is also meaningless before a stream (no acked fraction), so it is gated on a real program total too.
  let draw_bar = progress.total > 0
    && ui.available_width() >= Metrics::PROGRESS_W + Metrics::DOCK_PROGRESS_TEXT_RESERVE;

  // The simulation's caveats trail the clock at the far right (drawn first in this right-to-left strip): a
  // "(default settings)" flag when the estimate used the default machine model, and a pause count when the
  // timeline modeled unbounded operator waits. Rendered only when a simulation drives the ETA.
  if let Some(qualifier) = eta_qualifier {
    dock_eta_qualifier(ui, qualifier);
  }
  // Rightmost (after the qualifier): the elapsed / estimated-total clock. `total` is `None` until the ETA is
  // projectable, so its right half shows the dim `--:--` placeholder rather than a wild early guess (a simulation
  // populates `total` immediately, so the upfront figure shows at once). See `format_progress_clock`.
  let clock = format_progress_clock(time.elapsed, time.total);
  ui.label(RichText::new(clock).monospace().size(10.5).color(Theme::TEXT_DIM));
  // The percent/count/bar fields only mean something once a stream is timing; skip them (and their separator)
  // before streaming so the upfront ETA shows the clock + qualifier alone, not a `0%`/`0 / 0` placeholder row.
  if progress.total == 0 {
    return;
  }
  dock_progress_separator(ui);
  let pct = (progress.fraction() * 100.0).round() as u32;
  ui.label(RichText::new(format!("{pct}%")).monospace().size(11.0).color(Theme::TEXT));
  if draw_bar {
    dock_progress_separator(ui);
    let bar = Vec2::new(Metrics::PROGRESS_W, Metrics::PROGRESS_H);
    let (rect, _) = ui.allocate_exact_size(bar, egui::Sense::hover());
    let painter = ui.painter();
    painter.rect_filled(rect, 2.0, Theme::INSET);
    let mut fill = rect;
    fill.set_width(rect.width() * progress.fraction());
    painter.rect_filled(fill, 2.0, Theme::STATE_RUN);
  }
  dock_progress_separator(ui);
  ui.label(RichText::new(format!("{} / {}", progress.acked, progress.total)).monospace().size(10.5)
    .color(Theme::TEXT_DIM));
}

/// Draw the dim middot that separates the dock progress fields (the design's `·`). Pulled out so every gap in
/// [`dock_progress`] uses one consistent glyph and colour instead of repeating the `RichText` at each call site.
fn dock_progress_separator(ui: &mut egui::Ui) {
  ui.label(RichText::new("·").size(11.0).color(Theme::TEXT_DISABLED));
}

/// Render the simulation's caveats beside the dock clock: a dim "(default settings)" flag when the estimate used
/// the firmware's default machine model rather than the board's real `$$` config, and a "pause(s) at N line(s)"
/// note when the timeline modeled unbounded operator waits (`M0`/`M1`/`M6`) that are excluded from the timed
/// total. Each part is its own label so the strip degrades gracefully; both are omitted when neither applies. The
/// qualifier text is built by the pure [`eta_qualifier_text`] so the copy is unit-tested without a window.
fn dock_eta_qualifier(ui: &mut egui::Ui, qualifier: EtaQualifier) {
  if let Some(text) = eta_qualifier_text(qualifier) {
    ui.label(RichText::new(text).size(10.0).color(Theme::TEXT_DISABLED));
  }
}

/// Build the dock ETA qualifier string for a simulation, or `None` when there is nothing to qualify (real
/// settings and no modeled pauses). Kept pure (no egui) so the copy — "(default settings)", the pause count, and
/// their joining — is unit-tested without a window. The pause note uses "line"/"lines" so a single pause reads
/// naturally.
fn eta_qualifier_text(qualifier: EtaQualifier) -> Option<String> {
  let mut parts: Vec<String> = Vec::new();
  if qualifier.default_settings {
    parts.push("(default settings)".to_string());
  }
  if qualifier.pauses > 0 {
    let noun = if qualifier.pauses == 1 { "line" } else { "lines" };
    parts.push(format!("pauses at {} {noun}", qualifier.pauses));
  }
  if parts.is_empty() { None } else { Some(parts.join(" · ")) }
}

/// Decide whether the Program listing should auto-scroll to follow the executing line this frame, and which row
/// to centre on. Returns `Some(line)` only when auto-scroll is on, the stream is live, and the executing line has
/// *moved* since we last followed it — so the view nudges once per advance and otherwise leaves the operator's
/// manual scroll-back alone. `followed` is the last row we scrolled to (`UiState::program_followed_line`), updated
/// by the caller to the returned value. Kept pure (no egui) so the follow logic is unit-testable without a GUI.
fn program_follow_target(
  auto_scroll: bool,
  connection: ConnectionState,
  current: usize,
  program_len: usize,
  followed: Option<usize>,
) -> Option<usize> {
  if !auto_scroll || connection != ConnectionState::Streaming || current >= program_len {
    return None;
  }
  if followed == Some(current) {
    return None;
  }
  Some(current)
}

/// Render the Program tab body: the loaded file's lines with the acked line highlighted, drawn lazily so a
/// large program stays cheap to render. While streaming, the listing auto-scrolls to keep the executing line in
/// view (gated on the shared `auto-scroll` toggle), mirroring the console's stick-to-bottom follow.
fn program_body(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState) {
  let row_height = ui.text_style_height(&egui::TextStyle::Monospace);
  let current = view.progress.acked;
  // Resolve the follow target before drawing so we can scroll the area to the executing row even when row
  // virtualisation has it off-screen (a `scroll_to_me` on the row only fires for rows actually in `range`).
  let follow = program_follow_target(
    state.auto_scroll,
    view.connection,
    current,
    state.program.len(),
    state.program_followed_line,
  );
  ScrollArea::vertical().auto_shrink([false, false]).show_rows(ui, row_height, state.program.len(), |ui, range| {
    // `show_rows` lays out only the visible slice, positioning the content `ui` so its top sits at the origin of
    // the FIRST visible row (`range.start`), not row 0 — and rows advance by `row_height + item_spacing.y`, not
    // the bare text height. Recover row 0's origin from the slice top, then any row's y is a flat multiple of the
    // full row pitch; this lets us scroll to a line virtualisation has not drawn this frame, which `scroll_to_me`
    // (row-local) cannot.
    if let Some(target) = follow {
      let row_pitch = row_height + ui.spacing().item_spacing.y;
      let row_zero_top = ui.min_rect().top() - range.start as f32 * row_pitch;
      let top = row_zero_top + target as f32 * row_pitch;
      let rect = egui::Rect::from_min_max(
        egui::pos2(ui.min_rect().left(), top),
        egui::pos2(ui.min_rect().right(), top + row_height),
      );
      ui.scroll_to_rect(rect, Some(Align::Center));
    }
    for index in range {
      let line = &state.program[index];
      let is_current = index == current && view.connection == ConnectionState::Streaming;
      // Executed lines dim out; the current line is emphasised with the accent and an inset highlight; pending
      // lines sit at the default text colour.
      let color = if is_current {
        Theme::ACCENT
      } else if index < current {
        Theme::TEXT_DISABLED
      } else {
        Theme::TEXT
      };
      let row = RichText::new(format!("{:>5}  {line}", index + 1)).monospace().color(color);
      if is_current {
        // Highlight the executing line with the accent-tinted inset the design uses (bg + left accent border).
        egui::Frame::new().fill(Theme::ACCENT.gamma_multiply(0.12)).inner_margin(egui::Margin {
          left: 4,
          right: 0,
          top: 0,
          bottom: 0,
        }).show(ui, |ui| {
          ui.label(row);
        });
      } else {
        ui.label(row);
      }
    }
  });
  // Record the row we just followed so the next frame only re-scrolls when the executing line advances again,
  // leaving an operator who has scrolled back to read an earlier line undisturbed until the cursor moves.
  if let Some(target) = follow {
    state.program_followed_line = Some(target);
  }
}

/// Render the Console tab body: a rolling, colour-tagged log (chevron coloured by line type) above an
/// auto-scroll toggle and the manual-command entry line with a Send button (design §03). The dock's shared tab
/// strip (see [`dock`]) carries the tab labels and the progress readout; this draws only the tab's content.
fn console_body(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  // The auto-scroll toggle sits above the log within the Console body (the mock's strip is now shared by both
  // tabs, so the toggle moves into the body where it only applies to the console).
  ui.horizontal(|ui| {
    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
      ui.checkbox(&mut state.auto_scroll, "auto-scroll");
      ui.checkbox(&mut state.verbose, "verbose");
    });
  });

  // Map the visible rows back onto the full buffer: in non-verbose mode bare `ok` acks are dropped so the row
  // virtualisation below counts and indexes only the lines actually drawn.
  let visible: Vec<usize> = view
    .console
    .iter()
    .enumerate()
    .filter(|(_, entry)| state.verbose || !is_ok_noise(entry))
    .map(|(index, _)| index)
    .collect();

  let row_height = ui.text_style_height(&egui::TextStyle::Monospace);
  let scroll = ScrollArea::vertical().auto_shrink([false, false]).stick_to_bottom(state.auto_scroll).show_rows(
    ui,
    row_height,
    visible.len(),
    |ui, range| {
      for row in range {
        if let Some(entry) = visible.get(row).and_then(|&index| view.console.get(index)) {
          let (chevron, color) = console_line_style(entry.source, &entry.text);
          ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 4.0;
            ui.label(RichText::new(chevron).monospace().color(color));
            ui.label(RichText::new(&entry.text).monospace().color(Theme::TEXT));
          });
        }
      }
    },
  );
  // Right-clicking anywhere over the console viewport (a row or the empty space below the last line) offers a
  // single "Clear" action. `interact` over the scroll-area rect makes the whole body — not just a painted row —
  // catch the secondary click, so the menu is reliable on a sparse or empty console.
  let console_rect = scroll.inner_rect;
  ui.interact(console_rect, ui.id().with("console_context"), egui::Sense::click()).context_menu(|ui| {
    if ui.button("Clear").clicked() {
      sink.push(Intent::ClearConsole);
      ui.close();
    }
  });

  // Manual command entry: the input fills the row and Send sits flush to its right (design §03 command line),
  // sending on Enter or the button, only while connected.
  ui.horizontal(|ui| {
    let connected = view.connection.is_connected();
    // Reserve the Send button's slot on the right first, then let the field claim the rest of the row.
    let send_w = Metrics::SEND_PAD_X * 2.0 + 32.0;
    let send_clicked = ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
      ui.add_enabled_ui(connected, |ui| {
        ui.add_sized(Vec2::new(send_w, Metrics::PANEL_CONTROL_H), egui::Button::new("Send")).clicked()
      }).inner
    }).inner;
    let response = ui.add_enabled(connected, egui::TextEdit::singleline(&mut state.console_input)
      .hint_text("$$, G0 X0, …").desired_width(f32::INFINITY));
    let submit = response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
    if connected && (submit || send_clicked) && !state.console_input.trim().is_empty() {
      let line = std::mem::take(&mut state.console_input);
      sink.push(Intent::SendLine(line));
      response.request_focus();
    }
  });
}

/// Whether a console line is a bare `ok` acknowledgement — the per-line ack the firmware emits for every consumed
/// command. These are hidden when `verbose` is off so continuous jogging does not flood the log. Pure so it is
/// unit-tested. Only `Received` lines count: a literal `ok` the operator typed and we echoed stays visible.
fn is_ok_noise(entry: &LogLine) -> bool {
  entry.source == LogSource::Received && entry.text.trim() == "ok"
}

/// Decide the chevron glyph and colour for one console line, distinguishing status (`<…>`) and info (`[…]`)
/// firmware lines from a plain response, per the design's console line types. Pure so it is unit-tested.
fn console_line_style(source: LogSource, text: &str) -> (&'static str, Color32) {
  match source {
    LogSource::Sent => ("›", Theme::LOG_SENT),
    LogSource::Notice => ("·", Theme::LOG_NOTICE),
    LogSource::Received => {
      if text.starts_with('<') {
        ("‹", Theme::LOG_STATUS)
      } else if text.starts_with('[') {
        ("‹", Theme::LOG_INFO)
      } else if text.starts_with("error") || text.starts_with("ALARM") {
        ("‹", Theme::DANGER)
      } else {
        ("‹", Theme::LOG_RECV)
      }
    }
  }
}

/// Render the bottom status bar: the design's mono info strip (port dot · state · WCO · `Ln a/b · NN%` · F·S).
pub fn status_bar(ui: &mut egui::Ui, view: &ViewState, state: &UiState) {
  // Fabulous mode: a matching Pride band along the strip's top edge, bookending the toolbar's. Painted before
  // the content so the mono text sits above it; the band is thin enough not to crowd the row.
  if state.fabulous {
    let bar = ui.max_rect();
    let stripe = egui::Rect::from_min_max(
      egui::pos2(bar.left(), bar.top()),
      egui::pos2(bar.right(), bar.top() + Metrics::PRIDE_STRIPE_H),
    );
    paint_pride(ui.painter(), stripe);
  }
  ui.horizontal(|ui| {
    let badge = view.badge_state();
    dot(ui, Theme::badge_color(badge), Metrics::STATUS_DOT);
    let port = if state.selected_port.is_empty() { "—" } else { &state.selected_port };
    ui.label(RichText::new(port).monospace().size(10.5).color(Theme::TEXT_DIM));
    ui.label(RichText::new("·").color(Theme::TEXT_DISABLED));
    ui.label(RichText::new(badge.label()).monospace().size(10.5).color(Theme::TEXT));

    if !view.last_wco.is_empty() {
      ui.label(RichText::new("·").color(Theme::TEXT_DISABLED));
      ui.label(RichText::new("WCO set").monospace().size(10.5).color(Theme::TEXT_DIM));
    }
    if view.progress.total > 0 {
      ui.label(RichText::new("·").color(Theme::TEXT_DISABLED));
      let pct = (view.progress.fraction() * 100.0).round() as u32;
      ui.label(RichText::new(format!("Ln {} / {} · {pct}%", view.progress.acked, view.progress.total))
        .monospace().size(10.5).color(Theme::TEXT_DIM));
    }
    if let Some((feed, rpm, _)) = view.status.as_ref().and_then(|s| s.feed_speed) {
      ui.label(RichText::new("·").color(Theme::TEXT_DISABLED));
      ui.label(RichText::new(format!("F {feed:.0} · S {rpm:.0}")).monospace().size(10.5).color(Theme::TEXT_DIM));
    }
  });
}

/// Render the alarm/error banner: a full-width strip under the toolbar (design §04). An alarm shows the code, a
/// human gloss, and the Unlock-$X / Soft-reset recovery actions; a stream error shows the code, gloss, and a
/// reset/dismiss. The copy comes from the pure [`super::badge`] detail tables.
pub fn alarm_banner(ui: &mut egui::Ui, view: &ViewState, sink: &mut IntentSink) {
  let Some(banner) = &view.banner else {
    return;
  };
  // Headline + secondary detail per banner kind; both share the alarm surface so the strip reads as a fault. The
  // detail resolves through the live codebook so an enumerated (`$EA`/`$EE`) description beats the static text;
  // absent enrichment, the codebook's static fallback still yields a full sentence rather than a bare number. We
  // run every repaint while the banner is shown, so we take ONLY the description via the borrowing accessor — it
  // hands back a `'static` borrow on the static path (no per-frame allocation) and clones only on an override.
  let (headline, detail, is_alarm) = match banner {
    Banner::Alarm(code) => (format!("⚠ ALARM:{code}"), view.codes.alarm_description(*code), true),
    Banner::StreamError(code) => {
      (format!("⚠ error:{code} — stream halted"), view.codes.error_description(*code), false)
    }
  };
  egui::Frame::new()
    .fill(Theme::ALARM_BG)
    .stroke(egui::Stroke::new(1.0, Theme::ALARM_BORDER))
    .inner_margin(egui::Margin::symmetric(14, 10))
    .show(ui, |ui| {
      ui.horizontal(|ui| {
        ui.vertical(|ui| {
          ui.label(RichText::new(&headline).color(Theme::ALARM_TEXT).strong());
          ui.label(RichText::new(detail).size(11.5).color(Theme::TEXT_DIM));
        });
        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
          if ui.button("Dismiss").clicked() {
            sink.push(Intent::DismissBanner);
          }
          let reset = egui::Button::new(RichText::new("Soft reset").color(Color32::WHITE)).fill(Theme::DANGER);
          if ui.add(reset).on_hover_text("Soft reset (0x18)").clicked() {
            sink.push(Intent::Realtime(RealtimeCommand::SoftReset));
          }
          // Only an alarm offers the $X unlock; a stream error clears on reset / `$` / an empty line.
          if is_alarm && ui.button("Unlock $X").on_hover_text("Clear the alarm lock").clicked() {
            sink.push(Intent::SendLine("$X".to_string()));
          }
        });
      });
    });
}

/// The tool-change banner headline for a given active-tool value. `T0` ("none") and an as-yet-unreported tool
/// both fall back to a generic prompt rather than naming a misleading number, so the copy is always truthful.
/// Pulled out of [`tool_change_banner`] so the wording decision is a pure, testable function. `pub(crate)` so the
/// shell's integration tests can assert the exact banner copy for a resolved tool.
pub(crate) fn tool_change_headline(current_tool: Option<u32>) -> String {
  match current_tool {
    Some(tool) if tool != 0 => format!("🔧 Tool change: insert T{tool}, then Resume"),
    _ => "🔧 Tool change: insert the tool, then Resume".to_string(),
  }
}

/// Render the tool-change affordance: a full-width attention strip shown while the firmware is held for an M6
/// manual tool change (`<Tool|...>`). It names the tool to insert from the firmware's `$G`/`[GC:]`-reported tool
/// ([`ViewState::current_tool`]) — answered during the hold per the firmware's M0/M1/M6 `$G`-in-hold support — and
/// offers a Resume that issues the cycle-start (`~`) through the existing run/resume path, never a second pathway.
/// Drawn in the banner slot, distinct from the alarm surface: violet (attention), not red, so it never reads as a
/// fault. A fault banner, if latched, takes precedence (the shell shows this only when none is).
pub fn tool_change_banner(ui: &mut egui::Ui, view: &ViewState, sink: &mut IntentSink) {
  let headline = tool_change_headline(view.current_tool);
  egui::Frame::new()
    .fill(Theme::INSET)
    .stroke(egui::Stroke::new(1.0, Theme::STATE_CHECK))
    .inner_margin(egui::Margin::symmetric(14, 10))
    .show(ui, |ui| {
      ui.horizontal(|ui| {
        ui.vertical(|ui| {
          ui.label(RichText::new(&headline).color(Theme::STATE_CHECK).strong());
          ui.label(RichText::new("The machine is paused for a manual tool change (M6). Insert the tool and press \
            Resume (cycle-start) to continue.").size(11.5).color(Theme::TEXT_DIM));
        });
        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
          // Resume reuses the single run/resume intent — the same `~` cycle-start the toolbar's Resume segment and
          // the keyboard hotkey issue — so there is exactly one resume pathway.
          let resume = egui::Button::new(RichText::new("▶ Resume").color(Color32::WHITE)).fill(Theme::STATE_RUN);
          if ui.add(resume).on_hover_text("Resume after the tool change (cycle-start, ~)").clicked() {
            sink.push(Intent::RunOrResume);
          }
        });
      });
    });
}

/// The per-frame lerp fraction the live marker eases toward each fresh status sample. At up to 20 Hz repaint
/// against a 5–10 Hz status feed this hides the sample-rate step within a couple of frames without lagging
/// perceptibly. Lerp-only (no extrapolation), so the marker can never overshoot the real tool.
const MARKER_LERP: f32 = 0.35;

/// The marker snaps (rather than eases) to a new sample once the jump exceeds this fraction of the toolpath's
/// model-space span diagonal — a new program, a `$X`/teleport, or a coordinate-system change is not motion to
/// animate. Below it, normal cutting moves between samples are smoothed.
const MARKER_SNAP_SPAN_FRACTION: f32 = 0.25;

/// Minimum tool travel before a new live work position is appended to the trail, as a fraction of the toolpath's
/// model-space span diagonal (so it is resolution-independent). The step gate decimates the 5–10 Hz status feed
/// and rejects sub-step jitter; small enough to keep fine detail, large enough that a stationary tool does not
/// pile up points.
const TRAIL_MIN_STEP_FRACTION: f32 = 0.002;

/// Maximum gap between two consecutive trail points that is still drawn as a connected line, as a fraction of the
/// span diagonal. A larger gap is a jump — a rapid reposition between moves, or a reconnect — and is left broken
/// rather than drawn as a spurious streak across uncut work.
const TRAIL_BREAK_FRACTION: f32 = 0.12;

/// Rolling cap on trail length. Past this the oldest points are dropped (like AXIS's limited live-plot history),
/// bounding memory and per-frame draw cost on a long job; the decimating step gate keeps a typical job well under.
const MAX_TRAIL_POINTS: usize = 30_000;

/// Headroom above the (override-scaled) max programmed feed before a realized feed counts as a rapid. Absorbs the
/// gap between cutting feeds and the machine's faster rapid-traverse rate while tolerating modest feed variation.
const RAPID_FEED_MARGIN: f64 = 1.2;

/// How far outside the toolpath's model-space bounds the live marker may sit and still be drawn, as a fraction of
/// the span diagonal. Generous enough that a tool a little outside the drawn extents (lead-in, clearance move)
/// still shows while idle, tight enough that a grossly displaced point — a 25.4× units mismatch (#3), a homed-off
/// machine-origin park (#4), or a WCS-mismatch float (#5) — falls outside and is suppressed.
const MARKER_ON_PATH_MARGIN_FRACTION: f32 = 0.15;

/// The model-space span diagonal used to scale the resolution-independent marker/progress tolerances. Clamped to
/// a small floor so a degenerate (single-point) program cannot collapse the tolerances to zero.
fn span_diagonal(span: egui::Vec2) -> f32 {
  (span.x * span.x + span.y * span.y).sqrt().max(1.0)
}

/// One point of the position trail: a model-space tool position plus whether the move into it was a rapid (travel)
/// rather than a cut, so the trail can colour the two apart.
#[derive(Debug, Clone, Copy)]
struct TrailPoint {
  pos: Vec2,
  rapid: bool,
}

/// Append the live work position to the trail if the tool has moved at least `min_step` from the last point, and
/// enforce the rolling [`MAX_TRAIL_POINTS`] cap (oldest dropped). `rapid` is whether the current move is a travel
/// move (see [`super::preview::is_rapid_feed`]). The append decision itself is the pure
/// [`super::preview::trail_should_append`]; this just owns the `VecDeque` mutation.
fn push_trail_point(trail: &mut std::collections::VecDeque<TrailPoint>, point: Vec2, rapid: bool, min_step: f32) {
  let last = trail.back().map(|p| (p.pos.x, p.pos.y));
  if preview::trail_should_append(last, (point.x, point.y), min_step) {
    trail.push_back(TrailPoint { pos: point, rapid });
    while trail.len() > MAX_TRAIL_POINTS {
      trail.pop_front();
    }
  }
}

/// Scan a loaded program for the maximum programmed feed (`F` word, mm/min). Used to classify trail points as
/// rapid vs cut: rapids run faster than any programmed cut. Comments are stripped and words read with the same
/// lexer as the toolpath parser; a program with no `F` word yields `0.0` (then nothing is classed as a rapid).
fn max_programmed_feed(lines: &[String]) -> f64 {
  let mut max = 0.0_f64;
  for line in lines {
    let code = line.split(';').next().unwrap_or("").to_ascii_uppercase();
    for (letter, number) in gcode_words(&code) {
      if letter == 'F'
        && let Ok(f) = number.parse::<f64>()
      {
        max = max.max(f);
      }
    }
  }
  max
}

/// Render the 2D toolpath viewport: a top-down XY preview of the loaded program drawn with [`egui::Painter`].
/// The path is fit to the available rect. When a live status report carries a work position the marker and the
/// "cut so far" colouring track the real machine position (smoothed against the status sample rate); with no
/// live status they fall back to the acknowledged-line position. Parsing is a cheap single pass done at load.
pub fn toolpath(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState) {
  // The viewport header carries the filename/line-count on the right, inside the 30px strip (design §03).
  let progress = view.progress;
  let name = state.program_path.as_deref().map(|p| p.rsplit(['/', '\\']).next().unwrap_or(p).to_string());
  header_bar(
    ui,
    |ui| {
      header_title(ui, "Toolpath");
      if let Some(name) = &name {
        ui.label(RichText::new(name).monospace().size(10.5).color(Theme::TEXT_DISABLED));
      }
    },
    |ui| {
      if progress.total > 0 {
        ui.label(RichText::new(format!("{} / {}", progress.acked, progress.total))
          .monospace().size(10.5).color(Theme::TEXT_DISABLED));
      }
    },
  );
  ui.add_space(2.0);
  let available = ui.available_size();
  let (response, painter) = ui.allocate_painter(available, egui::Sense::hover());
  let rect = response.rect;
  painter.rect_filled(rect, 0.0, Theme::INSET);
  draw_grid(&painter, rect);

  let Some((min, max)) = state.toolpath_bounds else {
    painter.text(rect.center(), egui::Align2::CENTER_CENTER, "no program loaded",
      egui::FontId::proportional(14.0), Theme::TEXT_DIM);
    return;
  };

  // Fit the cached model-space bounds into the viewport with a uniform scale (the only per-frame geometry).
  let span = (max - min).max(egui::vec2(1.0, 1.0));
  let margin = 16.0;
  let scale = ((rect.width() - 2.0 * margin) / span.x).min((rect.height() - 2.0 * margin) / span.y).max(0.0001);

  // Map a model point into screen space, flipping Y so +Y is up as on a machine bed, and centring the fit.
  let used = span * scale;
  let offset = egui::vec2(rect.left() + (rect.width() - used.x) * 0.5, rect.top() + (rect.height() - used.y) * 0.5);
  let to_screen = |p: egui::Vec2| egui::pos2(offset.x + (p.x - min.x) * scale, offset.y + (max.y - p.y) * scale);

  // Fold this frame's live status into the overlay: extend the position trail and smooth the marker. Returns the
  // model-space marker to draw, or `None` to draw no dot. This mutates the cached state (trail/marker) before the
  // immutable draw borrows of `state.toolpath`/`state.trail` below.
  let live_marker = update_live_overlay(state, view, (min, max), span);

  // The program preview underneath is drawn uniformly DIM (cuts neutral, rapids dimmer) — it is the reference
  // geometry, not the progress. Progress is shown by the trail drawn over it, so there is no per-segment cut state.
  for seg in &state.toolpath {
    let color = if seg.rapid { Theme::BORDER_RAISED } else { Theme::TEXT_DIM };
    painter.line_segment([to_screen(seg.from), to_screen(seg.to)], egui::Stroke::new(1.0, color));
  }

  // The position trail: the AXIS-style backplot of where the tool has actually been, drawn over the dim preview.
  // Cut moves take the warm motion accent; rapid (travel) moves take the cool accent and a thinner stroke, so the
  // two read apart. Consecutive points are joined only when close enough — a large gap is a reconnect/teleport and
  // is left broken rather than streaked across uncut work (see [`preview::trail_connects`]). The move INTO a point
  // carries that point's rapid flag, so the drawn segment is coloured by `cur`.
  let break_gap = span_diagonal(span) * TRAIL_BREAK_FRACTION;
  let mut prev: Option<&TrailPoint> = None;
  for cur in &state.trail {
    if let Some(a) = prev
      && preview::trail_connects((a.pos.x, a.pos.y), (cur.pos.x, cur.pos.y), break_gap)
    {
      let (color, width) =
        if cur.rapid { (Theme::ACCENT, 1.0) } else { (Theme::ACCENT_MOTION, 1.6) };
      painter.line_segment([to_screen(a.pos), to_screen(cur.pos)], egui::Stroke::new(width, color));
    }
    prev = Some(cur);
  }

  // The tool dot (warm motion accent) marks where the machine is now: the smoothed live position, projected
  // through the same fit transform so it sits on the geometry. Suppressed (no dot) when there is no live position.
  if let Some(model) = live_marker {
    let pos = to_screen(model);
    painter.circle_filled(pos, 4.0, Theme::ACCENT_MOTION);
    painter.circle_stroke(pos, 8.0, egui::Stroke::new(1.0, Theme::ACCENT_MOTION.gamma_multiply(0.5)));
  }
}

/// Whether a run state is one of active motion (Run/Jog/Hold) — the states in which the live tool marker is shown
/// unconditionally (a legitimate cut may run just outside the drawn extents). Every other state (Idle/Alarm/…) is
/// gated on the marker actually lying near the path, so a parked-off-path point is suppressed (finding #4).
fn is_moving_state(run_state: Option<crate::protocol::RunState>) -> bool {
  use crate::protocol::RunState::{Hold, Jog, Run};
  matches!(run_state, Some(Run | Jog | Hold))
}

/// Fold one frame of live status into the cached overlay: extend the position trail from the raw work position and
/// smooth the model-space marker toward it. Returns the smoothed marker in model space when a marker dot should be
/// drawn, or `None` for no dot. Kept as a small helper so the per-frame state mutation is isolated from rendering;
/// the smoothing, trail, and gating decisions themselves live in pure, unit-tested [`super::preview`] functions.
///
/// Three live-status cases are distinguished (findings #3/#4/#5/#6):
/// - **No status at all** (disconnected / pre-connect): drop the held marker so a later reconnect snaps fresh.
/// - **A status with no derivable work position** (grbl pushes `WCO` only intermittently, so a mid-run report can
///   lack one): HOLD the last marker through the gap rather than nulling it — nulling would re-snap on the next
///   valid frame, a visible blink/teleport. The trail is simply not extended this frame.
/// - **A status with a work position**: extend the trail and smooth toward it, but draw the marker dot only when
///   [`preview::marker_is_on_path`] passes (on/near the path, or while actively moving) — a parked machine far off
///   the path (homed to machine origin, a units mismatch, a WCS mismatch) draws no dot rather than a
///   confident-but-wrong one.
fn update_live_overlay(state: &mut UiState, view: &ViewState, bounds: (Vec2, Vec2), span: egui::Vec2) -> Option<Vec2> {
  let Some(target) = view.work_xy().map(|(x, y)| (x as f32, y as f32)) else {
    // No derivable work position this frame. Two cases (finding #6):
    // - No status at all (disconnected / pre-connect): clear the marker so a later reconnect snaps fresh.
    // - A status with no derivable work position (grbl pushes WCO only intermittently, so a mid-run report can
    //   lack one): HOLD the last marker AND keep returning it, so the dot stays put rather than blinking out and
    //   re-snapping next frame. The trail is simply not extended this frame (no live point), which is correct.
    if view.status.is_none() {
      state.marker_pos = None;
      return None;
    }
    return state.marker_pos.map(|p| egui::vec2(p.x, p.y));
  };
  let diagonal = span_diagonal(span);
  let snap_dist = diagonal * MARKER_SNAP_SPAN_FRACTION;
  let current = state.marker_pos.map(|p| (p.x, p.y));
  let smoothed = preview::smooth_marker(current, target, snap_dist, MARKER_LERP);
  state.marker_pos = Some(egui::vec2(smoothed.0, smoothed.1));

  // Extend the position trail (the backplot of where the tool has actually been) from the RAW live work position —
  // not the smoothed marker, which lags — so the trail records true travel. The step gate decimates the status
  // feed; the rolling cap bounds it. Classify the point as rapid vs cut from the live realized feed measured
  // against the program's max programmed feed, scaled by the live feed override.
  let feed = view.status.as_ref().and_then(|s| s.feed_speed).map(|(feed, _, _)| feed);
  let feed_override = view.status.as_ref().and_then(|s| s.overrides).map_or(1.0, |(f, _, _)| f as f64 / 100.0);
  let rapid = preview::is_rapid_feed(feed, state.max_feed, feed_override, RAPID_FEED_MARGIN);
  push_trail_point(&mut state.trail, egui::vec2(target.0, target.1), rapid, diagonal * TRAIL_MIN_STEP_FRACTION);

  // Gate the marker: drawn on/near the path or while actively moving, suppressed for a parked-off-path point
  // (#3/#4/#5). Use the SMOOTHED position so the gate matches what is drawn. The trail still extended above, so it
  // keeps recording even when the dot is hidden.
  let (min, max) = bounds;
  let margin = diagonal * MARKER_ON_PATH_MARGIN_FRACTION;
  let moving = is_moving_state(view.status.as_ref().map(|s| s.machine_state.state));
  if preview::marker_is_on_path(smoothed, (min.x, min.y), (max.x, max.y), margin, moving) {
    Some(egui::vec2(smoothed.0, smoothed.1))
  } else {
    None
  }
}

/// Paint the viewport's major/minor reference grid, matching the design's two-tone grid over the inset canvas.
fn draw_grid(painter: &egui::Painter, rect: egui::Rect) {
  let minor = Theme::PANEL.gamma_multiply(0.5);
  let major = Theme::PANEL;
  let mut x = rect.left();
  let mut i = 0;
  while x <= rect.right() {
    let color = if i % 5 == 0 { major } else { minor };
    painter.line_segment([egui::pos2(x, rect.top()), egui::pos2(x, rect.bottom())], egui::Stroke::new(1.0, color));
    x += 16.0;
    i += 1;
  }
  let mut y = rect.top();
  let mut j = 0;
  while y <= rect.bottom() {
    let color = if j % 5 == 0 { major } else { minor };
    painter.line_segment([egui::pos2(rect.left(), y), egui::pos2(rect.right(), y)], egui::Stroke::new(1.0, color));
    y += 16.0;
    j += 1;
  }
}

/// One straight XY segment of the parsed toolpath.
#[derive(Debug, Clone)]
struct Segment {
  from: egui::Vec2,
  to: egui::Vec2,
  /// Whether this is a rapid (G0) travel move rather than a cut.
  rapid: bool,
}

/// Parse the loaded program into a flat list of XY segments for the preview. A pragmatic linear interpreter:
/// it tracks the modal motion mode (G0 travel vs G1 cut vs G2/G3 arc) and the modal distance mode (G90 absolute /
/// G91 relative), then emits one or more segments per line that actually executes a motion. A line emits geometry
/// only when the effective motion mode is G0/1/2/3 AND it carries an X/Y word AND it is not a non-motion command:
/// G10/G28/G30/G92/G53/G4 take X/Y as parameters (or modify a single line) rather than as a normal modal move, so
/// they update no position and draw nothing. Tokens may be spaced (`G1 X10 Y5`) or compact (`G1X10.0Y5.0`).
///
/// **Arcs (G2/G3) are FLATTENED into many short chord segments** via [`super::preview::flatten_arc`], using the
/// I/J centre offset (XY plane / G17 assumed — the firmware's only arc plane). This makes the preview draw the
/// real curve rather than a single start→end chord. An arc with neither I/J nor a usable centre degrades to a
/// single straight chord, so a malformed or R-form arc never breaks the parse.
fn parse_xy_path(lines: &[String]) -> Vec<Segment> {
  let mut segments = Vec::new();
  let mut pos = egui::vec2(0.0, 0.0);
  // Modal motion mode: 0 == G0 rapid, 1 == G1 cut, 2 == G2 CW arc, 3 == G3 CCW arc.
  let mut motion = 0u8;
  let mut absolute = true; // modal distance mode: true == G90 (absolute), false == G91 (relative).
  for line in lines {
    let code = line.split(';').next().unwrap_or("").to_ascii_uppercase();
    if code.trim().is_empty() {
      continue;
    }
    let mut next = pos;
    let mut has_xy = false;
    let mut suppress = false; // a non-motion G-word on this line suppresses any segment for it.
    // Arc centre offsets (I/J) relative to the current position; collected only for an arc line. Both default to
    // zero (grbl treats an absent offset as zero), so an arc giving only one of I/J still resolves a centre.
    let (mut arc_i, mut arc_j) = (0.0_f32, 0.0_f32);
    let mut has_ij = false;
    for (letter, number) in gcode_words(&code) {
      match letter {
        'G' => match number.trim() {
          "0" | "00" => motion = 0,
          "1" | "01" => motion = 1,
          "2" | "02" => motion = 2,
          "3" | "03" => motion = 3,
          "90" => absolute = true,
          "91" => absolute = false,
          // Non-modal commands whose X/Y are parameters, not a move; they must not draw a segment.
          "10" | "28" | "30" | "92" | "53" | "4" | "04" => suppress = true,
          _ => {}
        },
        'X' => {
          if let Ok(v) = number.parse::<f32>() {
            next.x = if absolute { v } else { pos.x + v };
            has_xy = true;
          }
        }
        'Y' => {
          if let Ok(v) = number.parse::<f32>() {
            next.y = if absolute { v } else { pos.y + v };
            has_xy = true;
          }
        }
        'I' => {
          if let Ok(v) = number.parse::<f32>() {
            arc_i = v;
            has_ij = true;
          }
        }
        'J' => {
          if let Ok(v) = number.parse::<f32>() {
            arc_j = v;
            has_ij = true;
          }
        }
        _ => {}
      }
    }
    if has_xy && !suppress {
      let rapid = motion == 0;
      if (motion == 2 || motion == 3) && has_ij {
        // An arc with a usable I/J centre: flatten it into chords. I/J are offsets from the START position.
        let center = (pos.x + arc_i, pos.y + arc_j);
        let mut from = (pos.x, pos.y);
        for point in super::preview::flatten_arc(from, (next.x, next.y), center, motion == 2) {
          segments.push(Segment { from: egui::vec2(from.0, from.1), to: egui::vec2(point.0, point.1), rapid });
          from = point;
        }
      } else {
        // A linear move, or an arc with no centre offset (degrade to its chord rather than guessing a centre).
        segments.push(Segment { from: pos, to: next, rapid });
      }
      pos = next;
    }
  }
  segments
}

/// The model-space `(min, max)` bounds of a parsed toolpath, or `None` when it is empty. Computed once at
/// program-load time (see [`UiState::set_program`]) so the viewport never re-scans the segments per frame.
fn toolpath_bounds(segments: &[Segment]) -> Option<(Vec2, Vec2)> {
  if segments.is_empty() {
    return None;
  }
  let mut min = egui::vec2(f32::INFINITY, f32::INFINITY);
  let mut max = egui::vec2(f32::NEG_INFINITY, f32::NEG_INFINITY);
  for seg in segments {
    for p in [seg.from, seg.to] {
      min.x = min.x.min(p.x);
      min.y = min.y.min(p.y);
      max.x = max.x.max(p.x);
      max.y = max.y.max(p.y);
    }
  }
  Some((min, max))
}

/// Split one (comment-stripped, upper-cased) G-code line into `(letter, number)` words, handling both spaced
/// (`G1 X10 Y5`) and compact (`G1X10.0Y5.0`) layouts. Each word is a letter `A..=Z` followed by its numeric
/// run (digits, sign, decimal point); whitespace and stray characters between words are skipped. Yields the
/// number as a `&str` slice so the caller parses only the words it cares about — no per-word allocation.
fn gcode_words(code: &str) -> impl Iterator<Item = (char, &str)> {
  let bytes = code.as_bytes();
  let mut i = 0;
  std::iter::from_fn(move || {
    while i < bytes.len() {
      let c = bytes[i] as char;
      if c.is_ascii_alphabetic() {
        let letter = c;
        let start = i + 1;
        let mut end = start;
        while end < bytes.len() {
          let n = bytes[end] as char;
          if n.is_ascii_digit() || n == '.' || n == '-' || n == '+' {
            end += 1;
          } else {
            break;
          }
        }
        i = end;
        return Some((letter, &code[start..end]));
      }
      i += 1;
    }
    None
  })
}

/// Whether leaving a settings field should stage its buffer (vs. abandon it). Leaving on Enter *or* plain
/// focus-loss stages — the fix for the old silent-drop bug, where focus-loss without Enter discarded the edit —
/// while Escape abandons. Pure so the stage-vs-abandon policy is unit-tested without a window; whether the
/// staged value is a real change (and so actually marks the row dirty) is decided by
/// [`super::settings_staging::SettingsStaging::stage`], which drops a value equal to the live one.
fn setting_edit_should_stage(abandoned: bool) -> bool {
  !abandoned
}

/// Whether a settings dialog action (Refresh/Close) needs the discard confirmation: only when edits are still
/// staged. Pure so the confirm gate is unit-tested without a window — with nothing staged, Refresh/Close proceed
/// immediately (the pre-existing behaviour); with edits staged, the caller parks the action behind the modal.
pub fn settings_action_needs_confirm(staging: &super::settings_staging::SettingsStaging) -> bool {
  !staging.is_empty()
}

/// Render the settings window: the connection-level baud knob plus the same live `$NNN` settings list the
/// right-column panel shows, in a roomier form. The list is driven by [`ViewState::settings`], populated from
/// the firmware's `$$`/`$ES` replies; editing a value writes it back via [`Intent::WriteSetting`].
pub fn settings(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  ui.label("Connection");
  ui.horizontal(|ui| {
    ui.label("Baud");
    ui.add(egui::DragValue::new(&mut state.baud).speed(100.0).range(BAUD_RANGE));
  });
  ui.separator();
  ui.horizontal(|ui| {
    ui.label(RichText::new("Firmware settings").color(Theme::TEXT));
    settings_save_button(ui, view, state, sink);
    settings_refresh_button(ui, view, state, sink);
  });
  // The explicit-Save model: edits stage locally and only reach the controller on Save. The note also flags that
  // some settings (e.g. `$22` homing) take effect only after the next reset, so a saved value may look inert.
  ui.label(RichText::new("Edits stage locally — Save writes them. Some settings (e.g. $22 homing) apply on the \
    next reset.").size(10.0).color(Theme::TEXT_DIM));
  ui.add_space(4.0);
  settings_list(ui, view, state);
}

/// The "Save" button: flushes every staged edit to the controller via [`Intent::SaveSettings`]. Enabled only
/// when connected *and* something is staged (nothing to write otherwise), so it greys out until the operator
/// actually changes a value. Saving each `$<n>=<value>` flows through the streaming engine, then `$$` re-confirms.
fn settings_save_button(ui: &mut egui::Ui, view: &ViewState, state: &UiState, sink: &mut IntentSink) {
  let connected = !matches!(view.connection, ConnectionState::Disconnected | ConnectionState::Connecting);
  let dirty = !state.settings_staging.is_empty();
  let label = if dirty { format!("Save ({})", state.settings_staging.len()) } else { "Save".to_string() };
  ui.add_enabled_ui(connected && dirty, |ui| {
    if ui.button(label).on_hover_text("Write every staged setting to the controller").clicked() {
      sink.push(Intent::SaveSettings);
    }
  });
}

/// The "fetch settings from the firmware" button: enabled only when connected (a `$$`/`$ES` request would just
/// error while disconnected). With unsaved edits staged it parks behind the discard confirmation (a refresh
/// would overwrite them with live values); with nothing staged it emits [`Intent::RequestSettings`] directly,
/// which the shell turns into `$ES` + `$$`.
fn settings_refresh_button(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  let connected = !matches!(view.connection, ConnectionState::Disconnected | ConnectionState::Connecting);
  ui.add_enabled_ui(connected, |ui| {
    if ui.button("Refresh ($$)").on_hover_text("Fetch $$ values and $ES labels from the controller").clicked() {
      if settings_action_needs_confirm(&state.settings_staging) {
        // Defer the fetch behind the discard modal when edits are staged; the modal emits it on confirm so the
        // request never silently clobbers staged edits without the operator's say-so.
        state.pending_settings_action = Some(PendingSettingsAction::Refresh);
      } else {
        // Nothing staged → fetch straight away, the pre-existing behaviour (no confirmation needed).
        sink.push(Intent::RequestSettings);
      }
    }
  });
}

/// The PRIMARY, dynamic tooltip lines derived from the `$ES`/`$$` metadata on `row` — the bits that are always
/// accurate per-firmware: the enumerated name (or a `$<n>` fallback), the unit, and the advertised `min..max`
/// range when present. Pure (no egui) so it unit-tests off-screen. A row with no metadata at all returns an empty
/// vec; the renderer still shows the bare `$<n>` heading, so the tooltip is never an empty box.
pub fn setting_tooltip_meta(row: &SettingRow) -> Vec<String> {
  let mut lines = Vec::new();
  let Some(meta) = &row.meta else { return lines };
  // The enumerated name, when the firmware advertised a non-empty one (the heading already carries `$<n>`, so this
  // line is the human name only).
  if !meta.name.is_empty() {
    lines.push(meta.name.clone());
  }
  if !meta.unit.is_empty() {
    lines.push(format!("Unit: {}", meta.unit));
  }
  // The advertised bounds, shown as whichever ends the firmware gave: a full `min..max`, or a one-sided `≥ min` /
  // `≤ max` when only one end was enumerated.
  match (&meta.min, &meta.max) {
    (Some(min), Some(max)) => lines.push(format!("Range: {min}..{max}")),
    (Some(min), None) => lines.push(format!("Range: ≥ {min}")),
    (None, Some(max)) => lines.push(format!("Range: ≤ {max}")),
    (None, None) => {}
  }
  lines
}

/// Render the rich hover panel for one setting: a heading line (`$<number>` plus the disambiguated display name),
/// the PRIMARY dynamic `meta_lines` from `$ES`, and the curated prose from `descriptions` when the number is known.
/// Degrades gracefully — a setting with neither metadata nor a description still shows the bare `$<number>` heading,
/// never an empty box. Kept thin (a pure render of already-decided state); the decisions live in
/// [`setting_tooltip_meta`] and [`super::setting_help`].
pub fn settings_tooltip_ui(
  ui: &mut egui::Ui,
  number: u32,
  heading_name: &str,
  meta_lines: &[String],
  descriptions: &super::setting_help::SettingDescriptions,
) {
  // Keep the panel from stretching to the screen edge on a long sentence; a fixed cap reads as a tidy tooltip.
  ui.set_max_width(320.0);
  // Heading: always present, so an unknown, metadata-less setting still shows `$<n>` (optionally with its name).
  let heading = if heading_name.is_empty() {
    format!("${number}")
  } else {
    format!("${number} · {heading_name}")
  };
  ui.label(RichText::new(heading).size(11.5).color(Theme::TEXT).strong());
  for line in meta_lines {
    ui.label(RichText::new(line).size(11.0).color(Theme::TEXT_DIM));
  }
  // The curated explanation, when the number is in the loaded set. Separated from the metadata by a thin rule so
  // the "what it does" prose reads distinctly from the "$ES says" facts above it.
  if let Some(desc) = descriptions.description(number) {
    ui.separator();
    ui.label(RichText::new(desc).size(11.0).color(Theme::TEXT_DIM));
  }
}

/// Render the live settings rows: each is a violet `$<n>` key, the enumerated label, and an editable value
/// field. A click into a value enters edit mode (a transient buffer in [`UiState::editing_setting`] seeded from
/// the current — staged-or-live — value); Enter or focus-loss *stages* the value (never silently dropped, the
/// fix for the old commit bug), Escape abandons it. A staged row is shown with an accent marker and its staged
/// value, until Save writes it or a confirmed Discard clears it. When no settings are known the section prompts
/// a refresh.
fn settings_list(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState) {
  if view.settings.is_empty() {
    ui.label(RichText::new("No settings loaded — Refresh to fetch the controller's $$ / $ES.").size(11.0)
      .color(Theme::TEXT_DIM));
    return;
  }
  // The staged edit (if any) is applied after the row loop so we never mutate `editing_setting` mid-borrow.
  let mut stage: Option<(bool, u32, String)> = None;
  // `auto_shrink([false, false])`: fill the host's available height so the surrounding Settings window resizes
  // vertically instead of snapping back to a fixed list height.
  egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
    egui::Grid::new("settings_list").num_columns(3).spacing([8.0, 4.0]).striped(true).show(ui, |ui| {
      for row in view.settings.rows() {
        let dirty = state.settings_staging.is_dirty(row.number);
        // The `$<n>` key turns to the accent-motion colour while the row has a staged edit, so a glance down the
        // list shows exactly which settings are modified and unsaved.
        let key_color = if dirty { Theme::ACCENT_MOTION } else { Theme::LOG_STATUS };
        ui.label(RichText::new(format!("${}", row.number)).monospace().size(11.0).color(key_color));
        // The label disambiguates grblHAL's per-axis settings (e.g. `$150/$151/$152` all named "Microsteps")
        // by appending the axis letter; a setting with a unique name is shown verbatim. A modified row prefixes a
        // dot so the marker survives even without colour.
        let label = view.settings.display_label(row);
        let unit = row.unit();
        let base = if unit.is_empty() { label.clone() } else { format!("{label} ({unit})") };
        let label_text = if dirty { format!("• {base}") } else { base };
        let label_color = if dirty { Theme::TEXT } else { Theme::TEXT_DIM };
        // Hovering the label cell explains the setting: the `$ES`-learned name/unit/range (PRIMARY, always
        // accurate per-firmware) plus the curated prose from [`super::setting_help`] when the number is known. A
        // row with neither still degrades to a bare `$<n>` line — never an empty box.
        let number = row.number;
        let heading_name = label; // the disambiguated display label, for the tooltip heading.
        let meta_lines = setting_tooltip_meta(row);
        let descriptions = &state.setting_descriptions;
        ui.label(RichText::new(label_text).size(11.0).color(label_color)).on_hover_ui(|ui| {
          settings_tooltip_ui(ui, number, &heading_name, &meta_lines, descriptions);
        });

        // The value cell: an in-edit row binds the transient buffer; an idle row shows the staged value when
        // dirty, else the live value, which a click promotes into edit mode seeded from whatever is shown.
        let editing_this = matches!(&state.editing_setting, Some((n, _)) if *n == row.number);
        if editing_this {
          if let Some((_, buffer)) = state.editing_setting.as_mut() {
            let resp = ui.add(egui::TextEdit::singleline(buffer).desired_width(72.0));
            let enter = resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            let abandoned = ui.input(|i| i.key_pressed(egui::Key::Escape));
            // Leave edit mode on any of: Enter, Escape (abandon), or focus moving elsewhere. Enter and focus-loss
            // both stage (the silent-drop fix); only Escape abandons. `setting_edit_should_stage` encodes that.
            if enter || abandoned || resp.lost_focus() {
              stage = Some((abandoned, row.number, buffer.clone()));
            }
          }
        } else {
          // Show the staged value while dirty (the pending edit), else the live value, else an em-dash placeholder.
          let shown = state.settings_staging.staged_value(row.number).map(str::to_string)
            .or_else(|| row.value.clone()).unwrap_or_else(|| "—".to_string());
          let value_color = if dirty { Theme::ACCENT_MOTION } else { Theme::TEXT };
          if ui.add(egui::Button::new(RichText::new(shown).monospace().size(11.0).color(value_color))
            .fill(Theme::INSET)).on_hover_text("Click to edit").clicked()
          {
            // Seed the buffer from what the row currently shows (staged value if dirty, else live).
            let seed = state.settings_staging.staged_value(row.number).map(str::to_string)
              .or_else(|| row.value.clone()).unwrap_or_default();
            state.editing_setting = Some((row.number, seed));
          }
        }
        ui.end_row();
      }
    });
  });

  if let Some((abandoned, number, buffer)) = stage {
    if setting_edit_should_stage(abandoned) {
      // `stage` already drops a no-op (value equal to live), so an unchanged edit leaves the row unmarked.
      state.settings_staging.stage(number, &buffer, view.settings.value_of(number));
    }
    // Leave edit mode whether we staged a change or not, so the row returns to a button showing its value.
    state.editing_setting = None;
  }
}

/// Render the "Discard N unsaved change(s)?" confirmation when a refresh/close was requested with edits staged.
/// Returns the deferred action to perform once the operator confirms Discard (and the staging has been cleared),
/// or `None` if there is no pending action or the operator chose Cancel (Keep editing). Modal: it grabs focus so
/// the operator must resolve it before doing anything else. Splitting the decision out keeps the caller's branch
/// on the returned action small and explicit.
pub fn settings_discard_confirm(ctx: &egui::Context, state: &mut UiState) -> Option<PendingSettingsAction> {
  let action = state.pending_settings_action?;
  let n = state.settings_staging.len();
  let mut resolved = None;
  egui::Window::new("Discard unsaved changes?")
    .collapsible(false).resizable(false).anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0]).show(ctx, |ui| {
      ui.label(format!("Discard {n} unsaved change(s)?"));
      ui.add_space(8.0);
      ui.horizontal(|ui| {
        if ui.button("Discard").clicked() {
          // Throw the staged edits away, then let the caller carry out the deferred action against clean state.
          state.settings_staging.clear();
          state.pending_settings_action = None;
          resolved = Some(action);
        }
        if ui.button("Cancel (Keep editing)").clicked() {
          // Abort the refresh/close; the staged edits and the dialog stay exactly as they were.
          state.pending_settings_action = None;
        }
      });
    });
  resolved
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn eta_qualifier_text_joins_the_default_settings_flag_and_the_pause_count() {
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
  fn push_trail_point_appends_real_moves_decimates_jitter_and_caps_length() {
    use std::collections::VecDeque;
    let mut trail: VecDeque<TrailPoint> = VecDeque::new();
    // First point is always taken; a sub-step wiggle is dropped; a real move is appended (with its rapid flag).
    push_trail_point(&mut trail, egui::vec2(0.0, 0.0), false, 1.0);
    push_trail_point(&mut trail, egui::vec2(0.3, 0.0), false, 1.0); // jitter < min_step → ignored
    push_trail_point(&mut trail, egui::vec2(2.0, 0.0), true, 1.0); // real move → appended
    assert_eq!(trail.len(), 2);
    assert_eq!(trail.back().unwrap().pos, egui::vec2(2.0, 0.0));
    assert!(trail.back().unwrap().rapid, "the move's rapid flag is recorded");
    // The rolling cap drops the oldest once full.
    let mut full: VecDeque<TrailPoint> =
      (0..MAX_TRAIL_POINTS as i32).map(|i| TrailPoint { pos: egui::vec2(i as f32 * 10.0, 0.0), rapid: false }).collect();
    let newest = egui::vec2(MAX_TRAIL_POINTS as f32 * 10.0, 0.0);
    push_trail_point(&mut full, newest, false, 1.0);
    assert_eq!(full.len(), MAX_TRAIL_POINTS, "length is capped");
    assert_eq!(full.back().unwrap().pos, newest, "newest point retained");
    assert_eq!(full.front().unwrap().pos, egui::vec2(10.0, 0.0), "oldest point dropped");
  }

  #[test]
  fn max_programmed_feed_takes_the_largest_f_word() {
    let program = vec![
      "G0 X0 Y0".to_string(),         // rapid, no F
      "G1 X10 Y0 F300".to_string(),   // cut at 300
      "G1 X10 Y10 F1200.5".to_string(), // cut at 1200.5 (the max), decimal
      "G1 X0 Y10 F800".to_string(),   // cut at 800
      "; F9999 in a comment".to_string(), // comment-only line is ignored
    ];
    assert_eq!(max_programmed_feed(&program), 1200.5);
    // A program with no feed word at all reports zero (nothing will be classed as a rapid).
    assert_eq!(max_programmed_feed(&["G0 X1 Y1".to_string()]), 0.0);
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
    // The full case: name, unit, and a two-sided range all enumerated.
    let row = row_with_meta(110, "Max rate", "mm/min", Some("0"), Some("10000"));
    let lines = setting_tooltip_meta(&row);
    assert_eq!(lines, vec!["Max rate".to_string(), "Unit: mm/min".to_string(), "Range: 0..10000".to_string()]);
  }

  #[test]
  fn tooltip_meta_handles_a_one_sided_range_and_a_unitless_setting() {
    // Only a max advertised, and no unit (a unitless bitmask like a status-report mask): the unit line is omitted
    // and the range is shown one-sided.
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
    // Sent and notice are fixed by source.
    assert_eq!(console_line_style(LogSource::Sent, "$H").1, Theme::LOG_SENT);
    assert_eq!(console_line_style(LogSource::Notice, "connect").1, Theme::LOG_NOTICE);
    // Received lines are typed by their leading glyph: status `<…>`, info `[…]`, error/alarm, else plain ok.
    assert_eq!(console_line_style(LogSource::Received, "<Idle|MPos:0,0,0>").1, Theme::LOG_STATUS);
    assert_eq!(console_line_style(LogSource::Received, "[MSG:hi]").1, Theme::LOG_INFO);
    assert_eq!(console_line_style(LogSource::Received, "error:9").1, Theme::DANGER);
    assert_eq!(console_line_style(LogSource::Received, "ok").1, Theme::LOG_RECV);
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
    let segments = parse_xy_path(&program);
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
    let segments = parse_xy_path(&program);
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
    let segments = parse_xy_path(&program);
    // Only the leading G1 is a segment; every non-motion line is suppressed.
    assert_eq!(segments.len(), 1, "non-motion G-codes must not draw segments");
    assert_eq!(segments[0].to, egui::vec2(1.0, 1.0));
  }

  #[test]
  fn toolpath_parser_handles_compact_spaceless_gcode() {
    // Standard CAM post output packs words with no spaces: `G1X10.0Y5.0`.
    let program = vec!["G0X0Y0".to_string(), "G1X10.0Y5.0".to_string(), "X20.5".to_string()];
    let segments = parse_xy_path(&program);
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
    let segments = parse_xy_path(&program);
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
    let segments = parse_xy_path(&program);
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
    let segments = parse_xy_path(&program);
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
}
