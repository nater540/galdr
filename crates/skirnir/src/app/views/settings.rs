//! The firmware `$$` settings panel: the list, save/refresh, tooltips, and the discard confirmation.

use super::*;

/// Whether leaving a settings field should stage its buffer (vs. abandon it). Leaving on Enter *or* plain
/// focus-loss stages — the fix for the old silent-drop bug, where focus-loss without Enter discarded the edit —
/// while Escape abandons. Pure so the stage-vs-abandon policy is unit-tested without a window; whether the
/// staged value is a real change (and so actually marks the row dirty) is decided by
/// [`crate::app::settings_staging::SettingsStaging::stage`], which drops a value equal to the live one.
pub(crate) fn setting_edit_should_stage(abandoned: bool) -> bool {
  !abandoned
}

/// Whether a settings dialog action (Refresh/Close) needs the discard confirmation: only when edits are still
/// staged. Pure so the confirm gate is unit-tested without a window — with nothing staged, Refresh/Close proceed
/// immediately (the pre-existing behaviour); with edits staged, the caller parks the action behind the modal.
pub fn settings_action_needs_confirm(staging: &crate::app::settings_staging::SettingsStaging) -> bool {
  !staging.is_empty()
}

/// Render the settings window: the connection-level baud knob plus the same live `$NNN` settings list the
/// right-column panel shows, in a roomier form. The list is driven by [`ViewState::settings`], populated from
/// the firmware's `$$`/`$ES` replies; editing a value writes it back via [`Intent::WriteSetting`].
pub fn settings(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  let palette = state.style.palette;
  ui.label(crate::tr!("lbl-connection"));
  ui.horizontal(|ui| {
    ui.label(crate::tr!("lbl-baud"));
    ui.add(egui::DragValue::new(&mut state.baud).speed(100.0).range(BAUD_RANGE));
  });
  ui.separator();
  ui.horizontal(|ui| {
    ui.label(RichText::new(crate::tr!("hdr-firmware-settings")).color(palette.text));
    settings_save_button(ui, view, state, sink);
    settings_refresh_button(ui, view, state, sink);
  });
  // The explicit-Save model: edits stage locally and only reach the controller on Save. The note also flags that
  // some settings (e.g. `$22` homing) take effect only after the next reset, so a saved value may look inert.
  ui.label(RichText::new(crate::tr!("settings-stage-note")).size(10.0).color(palette.text_dim));
  ui.add_space(4.0);
  settings_list(ui, view, state);
}

/// The "Save" button: flushes every staged edit to the controller via [`Intent::SaveSettings`]. Enabled only
/// when connected *and* something is staged (nothing to write otherwise), so it greys out until the operator
/// actually changes a value. Saving each `$<n>=<value>` flows through the streaming engine, then `$$` re-confirms.
fn settings_save_button(ui: &mut egui::Ui, view: &ViewState, state: &UiState, sink: &mut IntentSink) {
  let connected = !matches!(view.connection, ConnectionState::Disconnected | ConnectionState::Connecting);
  let dirty = !state.settings_staging.is_empty();
  let label = if dirty {
    crate::tr!("settings-save-n", { count: state.settings_staging.len() as i64 })
  } else {
    crate::tr!("settings-save")
  };
  ui.add_enabled_ui(connected && dirty, |ui| {
    if ui.button(label).on_hover_text(crate::tr!("tip-settings-save")).clicked() {
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
    if ui.button(crate::tr!("settings-refresh")).on_hover_text(crate::tr!("tip-settings-refresh")).clicked() {
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
    lines.push(crate::tr!("setting-unit", { unit: meta.unit.as_str() }));
  }
  // The advertised bounds, shown as whichever ends the firmware gave: a full `min..max`, or a one-sided `≥ min` /
  // `≤ max` when only one end was enumerated.
  match (&meta.min, &meta.max) {
    (Some(min), Some(max)) => lines.push(crate::tr!("setting-range", { min: min.as_str(), max: max.as_str() })),
    (Some(min), None) => lines.push(crate::tr!("setting-range-min", { min: min.as_str() })),
    (None, Some(max)) => lines.push(crate::tr!("setting-range-max", { max: max.as_str() })),
    (None, None) => {}
  }
  lines
}

/// Render the rich hover panel for one setting: a heading line (`$<number>` plus the disambiguated display name),
/// the PRIMARY dynamic `meta_lines` from `$ES`, and the curated prose from `descriptions` when the number is known.
/// Degrades gracefully — a setting with neither metadata nor a description still shows the bare `$<number>` heading,
/// never an empty box. Kept thin (a pure render of already-decided state); the decisions live in
/// [`setting_tooltip_meta`] and [`crate::app::setting_help`].
pub fn settings_tooltip_ui(
  ui: &mut egui::Ui,
  palette: Palette,
  number: u32,
  heading_name: &str,
  meta_lines: &[String],
  descriptions: &crate::app::setting_help::SettingDescriptions,
) {
  // Keep the panel from stretching to the screen edge on a long sentence; a fixed cap reads as a tidy tooltip.
  ui.set_max_width(320.0);
  // Heading: always present, so an unknown, metadata-less setting still shows `$<n>` (optionally with its name).
  let heading = if heading_name.is_empty() {
    format!("${number}")
  } else {
    format!("${number} · {heading_name}")
  };
  ui.label(RichText::new(heading).size(11.5).color(palette.text).strong());
  for line in meta_lines {
    dim_label(ui, palette, line);
  }
  // The curated explanation, when the number is in the loaded set. Separated from the metadata by a thin rule so
  // the "what it does" prose reads distinctly from the "$ES says" facts above it.
  if let Some(desc) = descriptions.description(number) {
    ui.separator();
    dim_label(ui, palette, desc);
  }
}

/// Render the live settings rows: each is a violet `$<n>` key, the enumerated label, and an editable value
/// field. A click into a value enters edit mode (a transient buffer in [`UiState::editing_setting`] seeded from
/// the current — staged-or-live — value); Enter or focus-loss *stages* the value (never silently dropped, the
/// fix for the old commit bug), Escape abandons it. A staged row is shown with an accent marker and its staged
/// value, until Save writes it or a confirmed Discard clears it. When no settings are known the section prompts
/// a refresh.
fn settings_list(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState) {
  let palette = state.style.palette;
  if view.settings.is_empty() {
    dim_label(ui, palette, crate::tr!("settings-none"));
    return;
  }
  // The staged edit (if any) is applied after the row loop so we never mutate `editing_setting` mid-borrow.
  let mut stage: Option<(bool, u32, String)> = None;
  // `auto_shrink([false, false])`: fill the host's available height so the surrounding Settings window resizes
  // vertically instead of snapping back to a fixed list height.
  egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
    // Dense table rows: land the value buttons on the 22px control height (3px vertical padding on ~15px text)
    // so they match the computed-margin edit field — one control height per row, not two.
    ui.spacing_mut().button_padding.y = 3.0;
    egui::Grid::new("settings_list").num_columns(3).spacing([8.0, 4.0]).striped(true).show(ui, |ui| {
      for row in view.settings.rows() {
        let dirty = state.settings_staging.is_dirty(row.number);
        // The `$<n>` key turns to the accent-motion colour while the row has a staged edit, so a glance down the
        // list shows exactly which settings are modified and unsaved.
        let key_color = if dirty { palette.accent_motion } else { palette.log_status };
        ui.label(RichText::new(format!("${}", row.number)).monospace().size(11.0).color(key_color));
        // The label disambiguates grblHAL's per-axis settings (e.g. `$150/$151/$152` all named "Microsteps")
        // by appending the axis letter; a setting with a unique name is shown verbatim. A modified row prefixes a
        // dot so the marker survives even without colour.
        let label = view.settings.display_label(row);
        let unit = row.unit();
        let base = if unit.is_empty() { label.clone() } else { format!("{label} ({unit})") };
        let label_text = if dirty { format!("• {base}") } else { base };
        let label_color = if dirty { palette.text } else { palette.text_dim };
        // Hovering the label cell explains the setting: the `$ES`-learned name/unit/range (PRIMARY, always
        // accurate per-firmware) plus the curated prose from [`crate::app::setting_help`] when the number is known. A
        // row with neither still degrades to a bare `$<n>` line — never an empty box.
        let number = row.number;
        let heading_name = label; // the disambiguated display label, for the tooltip heading.
        let meta_lines = setting_tooltip_meta(row);
        let descriptions = &state.setting_descriptions;
        ui.label(RichText::new(label_text).size(11.0).color(label_color)).on_hover_ui(|ui| {
          settings_tooltip_ui(ui, palette, number, &heading_name, &meta_lines, descriptions);
        });

        // The value cell: an in-edit row binds the transient buffer; an idle row shows the staged value when
        // dirty, else the live value, which a click promotes into edit mode seeded from whatever is shown.
        let editing_this = matches!(&state.editing_setting, Some((n, _)) if *n == row.number);
        if editing_this {
          if let Some((_, buffer)) = state.editing_setting.as_mut() {
            // The computed margin lands the edit field on the same 22px control height as the value buttons in
            // this column, so entering edit mode does not jiggle the row height.
            let body_row = ui.text_style_height(&egui::TextStyle::Body);
            let resp = ui.add(egui::TextEdit::singleline(buffer)
              .margin(Metrics::text_field_margin(body_row, 4))
              .desired_width(72.0));
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
          let value_color = if dirty { palette.accent_motion } else { palette.text };
          if ui.add(egui::Button::new(RichText::new(shown).monospace().size(11.0).color(value_color))
            .fill(palette.inset)).on_hover_text(crate::tr!("tip-setting-edit")).clicked()
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
  egui::Window::new(crate::tr!("settings-discard-title"))
    .collapsible(false).resizable(false).anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0]).show(ctx, |ui| {
      ui.label(crate::tr!("settings-discard-body", { count: n as i64 }));
      ui.add_space(8.0);
      ui.horizontal(|ui| {
        if ui.button(crate::tr!("btn-discard")).clicked() {
          // Throw the staged edits away, then let the caller carry out the deferred action against clean state.
          state.settings_staging.clear();
          state.pending_settings_action = None;
          resolved = Some(action);
        }
        if ui.button(crate::tr!("btn-keep-editing")).clicked() {
          // Abort the refresh/close; the staged edits and the dialog stay exactly as they were.
          state.pending_settings_action = None;
        }
      });
    });
  resolved
}
