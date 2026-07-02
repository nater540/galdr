//! The application settings dialog: language, theme selection, font scale, and the user-theme colour editor.
//!
//! Distinct from the FIRMWARE settings window (`$$`, `views::settings`): everything here is host-side appearance
//! and localisation carried by `config.json`. The view is the same thin-render shape as every panel — it reads
//! [`UiState`] + the loaded [`Config`] and pushes [`Intent`]s; the shell owns applying them (re-skinning the live
//! context, mutating the config, writing the file on the explicit Save).
//!
//! Theme editing follows the config's own model (see [`crate::config::theme`]): built-ins live in code and are
//! read-only here; "Create from current" snapshots the ACTIVE resolved palette into a fully-keyed user
//! [`ThemeOverride`] (via [`ThemeOverride::from_palette`]) so every colour picker edits a concrete value, and each
//! picker change round-trips as an [`Intent::UpsertTheme`] so the shell re-resolves and the window re-skins live.

use eframe::egui::{self, Align, Layout, RichText};

use super::intent::{Intent, IntentSink};
use super::metrics::Metrics;
use super::views::UiState;
use crate::config::{Config, ThemeOverride};

/// The built-in theme names offered in the picker, in menu order. Kept in step with
/// [`crate::config::builtin_palette`] by a unit test below.
const BUILTIN_THEMES: [&str; 3] = ["default", "light-slate", "midnight"];

/// Show the dialog as a closable, resizable window. `dirty` is whether the in-memory config has unsaved edits,
/// driving the Save button and its marker.
pub fn window(ctx: &egui::Context, state: &mut UiState, config: &Config, dirty: bool, sink: &mut IntentSink) {
  let mut open = state.app_settings_open;
  // The explicit `.id()` keeps egui's remembered position/size keyed on a STABLE token: without it the id
  // derives from the translated title — and THIS dialog is where the operator switches language, so the window
  // would jump back to its default placement the instant a new locale was picked.
  egui::Window::new(crate::tr!("app-settings-title"))
    .id(egui::Id::new("app-settings-window"))
    .open(&mut open)
    .resizable(true)
    .default_size([400.0, 520.0])
    .show(ctx, |ui| body(ui, state, config, dirty, sink));
  state.app_settings_open = open;
}

/// The dialog body: the language/theme/scale rows, the create-theme row, the colour editor for a user theme, and
/// the explicit Save row. Split from [`window`] so tests can mount it directly in a harness `Ui`.
pub fn body(ui: &mut egui::Ui, state: &mut UiState, config: &Config, dirty: bool, sink: &mut IntentSink) {
  let palette = state.style.palette;
  // Dialog controls sit on the shared 22px control height: trim egui's default 6px vertical button padding so
  // the buttons and combo boxes land on the same row height as the computed-margin text field beside them —
  // inputs and buttons must read as one control family, not two heights.
  ui.spacing_mut().button_padding.y = 3.0;

  egui::Grid::new("app-settings-general").num_columns(2).spacing([12.0, 8.0]).show(ui, |ui| {
    // Language: the global i18n registry is the source of truth for both the current selection and the choices.
    ui.label(RichText::new(crate::tr!("app-settings-language")).color(palette.text_dim));
    let current_language = crate::i18n::get_language();
    let mut locales = crate::i18n::languages();
    locales.sort();
    egui::ComboBox::from_id_salt("app-settings-language")
      .width(200.0)
      .selected_text(language_display_name(&current_language))
      .show_ui(ui, |ui| {
        for locale in locales {
          let selected = locale == current_language;
          if ui.selectable_label(selected, language_display_name(&locale)).clicked() && !selected {
            sink.push(Intent::SetLanguage(locale));
          }
        }
      });
    ui.end_row();

    // Theme: built-ins first, then the user themes from the config (a user theme shadowing a built-in name shows
    // once, as the built-in slot — resolution already prefers the user entry).
    ui.label(RichText::new(crate::tr!("app-settings-theme")).color(palette.text_dim));
    let active = config.appearance.active_theme.as_str();
    egui::ComboBox::from_id_salt("app-settings-theme").width(200.0).selected_text(active).show_ui(ui, |ui| {
      for name in BUILTIN_THEMES {
        if ui.selectable_label(active == name, name).clicked() && active != name {
          sink.push(Intent::SetActiveTheme(name.to_string()));
        }
      }
      for name in config.appearance.themes.keys() {
        if BUILTIN_THEMES.contains(&name.as_str()) {
          continue; // shadowing entries already appear under the built-in name above.
        }
        let label = format!("{name} · custom");
        if ui.selectable_label(active == name, label).clicked() && active != name {
          sink.push(Intent::SetActiveTheme(name.clone()));
        }
      }
    });
    ui.end_row();

    // Font scale: a bounded slider over the same range the shell clamps to. The slider edits a local copy; a
    // change is pushed as an intent so the config mutation stays in the shell.
    ui.label(RichText::new(crate::tr!("app-settings-font-scale")).color(palette.text_dim));
    let mut scale = config.appearance.font_scale;
    // The slider spans the SAME range the config accepts (`FONT_SCALE_RANGE`); a narrower slider let egui's
    // always-clamp behaviour rewrite a valid hand-edited scale (e.g. 2.4) the moment the dialog opened.
    if ui.add(egui::Slider::new(&mut scale, crate::config::FONT_SCALE_RANGE).step_by(0.05).fixed_decimals(2)).changed() {
      sink.push(Intent::SetFontScale(scale));
    }
    ui.end_row();
  });

  ui.add_space(8.0);
  ui.separator();

  // Create a user theme from whatever is on screen: snapshot the ACTIVE resolved palette into a fully-keyed
  // override, so the new theme starts as an exact copy and every picker below edits a concrete value.
  ui.horizontal(|ui| {
    // The computed margin lands the field on the same control height as the Create button beside it.
    let body_row = ui.text_style_height(&egui::TextStyle::Body);
    let field = egui::TextEdit::singleline(&mut state.theme_name_draft)
      .hint_text(crate::tr!("app-settings-new-theme-hint"))
      .margin(Metrics::text_field_margin(body_row, 6))
      .desired_width(180.0);
    ui.add(field);
    let name = state.theme_name_draft.trim().to_string();
    let create = egui::Button::new(crate::tr!("app-settings-create"));
    if ui.add_enabled(!name.is_empty(), create)
      .on_hover_text(crate::tr!("app-settings-create-hint"))
      .clicked()
    {
      sink.push(Intent::UpsertTheme { name: name.clone(), theme: ThemeOverride::from_palette(&palette) });
      sink.push(Intent::SetActiveTheme(name));
      state.theme_name_draft.clear();
    }
  });

  ui.add_space(4.0);

  // The save row is ANCHORED AS AN INNER BOTTOM PANEL and the editor/hint region below FILLS the remainder, so
  // the dialog's content always expands to exactly the height the window is given. This is what makes a MANUAL
  // window resize stick: egui snaps a resizable window's height back to its content's natural height, so with a
  // built-in theme active (short, non-filling content) a vertical drag was overridden the instant the mouse
  // released (the user-reported self-resizing dialog). Content that fills makes any dragged size a fixed point —
  // the window opens at its default size and only the operator changes it. The panel also keeps the save
  // controls on-screen regardless of the picker list's length (the greedy-scroll lesson), replacing the earlier
  // reserved-height arithmetic.
  egui::Panel::bottom("app-settings-save")
    .exact_size(Metrics::PANEL_CONTROL_H + 16.0)
    .resizable(false)
    .show_separator_line(false)
    .frame(egui::Frame::NONE)
    .show_inside(ui, |ui| {
      ui.separator();
      // The explicit save boundary: nothing writes the operator-owned config.json implicitly. The unsaved marker
      // rides on the right; Save is disabled when there is nothing to write.
      ui.horizontal(|ui| {
        let save = egui::Button::new(RichText::new(crate::tr!("app-settings-save")).color(palette.text))
          .fill(palette.accent);
        if ui.add_enabled(dirty, save).on_hover_text(crate::tr!("app-settings-save-hint")).clicked() {
          sink.push(Intent::SaveConfig);
        }
        if dirty {
          ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
            // A hair of trailing air so the (longer in Swedish) marker never kisses or clips the right edge.
            ui.add_space(4.0);
            ui.label(RichText::new(crate::tr!("app-settings-unsaved")).size(10.5).color(palette.state_hold));
          });
        }
      });
    });

  // The colour editor: only a USER theme is editable (built-ins live in code). Each picker edits a clone of the
  // active entry; any change this frame is pushed once as a whole-theme upsert so the shell re-resolves live.
  // `auto_shrink([false, false])`: the list FILLS down to the anchored save row (see above — the fill is what
  // keeps the window's size stable), scrolling internally when the tokens outgrow it.
  match config.appearance.themes.get(config.appearance.active_theme.as_str()) {
    Some(theme) => {
      let mut edited = theme.clone();
      let mut changed = false;
      egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
        // Fall back to the ACTIVE resolved palette for any token the (possibly hand-edited, sparse) theme leaves
        // unset, so its picker starts from the colour actually on screen. Same order as `color_entries_mut`.
        let mut resolved = ThemeOverride::from_palette(&palette);
        let resolved_values: Vec<[u8; 4]> = resolved
          .color_entries_mut()
          .into_iter()
          .map(|(_, slot)| slot.map(crate::config::ColorSpec::to_srgba_unmultiplied).unwrap_or([0, 0, 0, 255]))
          .collect();
        let mut group: &'static str = "";
        for (index, (name, slot)) in edited.color_entries_mut().into_iter().enumerate() {
          let this_group = group_for(name);
          if this_group != group {
            group = this_group;
            ui.add_space(6.0);
            ui.label(
              RichText::new(crate::tr!(group_display_key(this_group)).to_uppercase())
                .size(Metrics::HEADER_TEXT)
                .color(palette.text_dim)
                .strong()
                .extra_letter_spacing(Metrics::HEADER_TEXT * Metrics::HEADER_TRACKING_EM),
            );
            ui.add_space(2.0);
          }
          ui.horizontal(|ui| {
            // Edit UNmultiplied sRGBA channels directly: `color_edit_button_srgba` (premultiplied) hands back a
            // Color32 whose raw channels, captured into a spec and re-expanded via `from_rgba_unmultiplied`, double-
            // apply alpha and decay a translucent colour toward black on every frame/reload. The unmultiplied picker
            // round-trips the [r,g,b,a] the operator actually chose, so config.json keeps the picked colour exactly.
            let mut rgba = slot
              .map(crate::config::ColorSpec::to_srgba_unmultiplied)
              .unwrap_or_else(|| resolved_values.get(index).copied().unwrap_or([0, 0, 0, 255]));
            if ui.color_edit_button_srgba_unmultiplied(&mut rgba).changed() {
              *slot = Some(crate::config::ColorSpec::from_srgba_unmultiplied(rgba));
              changed = true;
            }
            ui.label(RichText::new(name).monospace().size(11.0).color(palette.text));
          });
        }
      });
      if changed {
        sink.push(Intent::UpsertTheme {
          name: config.appearance.active_theme.clone(),
          theme: edited,
        });
      }
    }
    None => {
      ui.label(RichText::new(crate::tr!("app-settings-builtin-hint")).size(11.0).color(palette.text_dim));
    }
  }
}

/// A human-readable name for a locale tag in the language picker. Known bundled locales get their native names;
/// an unrecognised tag (a dropped-in locale pack) shows as its tag, which is still selectable and unambiguous.
pub(crate) fn language_display_name(tag: &str) -> String {
  match tag {
    "en-US" => "English (US)".to_string(),
    "sv-SE" => "Svenska".to_string(),
    other => other.to_string(),
  }
}

/// The i18n message key for a colour-editor group header, mapped from the stable identity string [`group_for`]
/// returns. Kept separate from `group_for` so the group's identity (used for change detection) stays a plain
/// `&str` while only the displayed header is localized. The total match mirrors `group_for`'s outputs.
fn group_display_key(group: &str) -> &'static str {
  match group {
    "accents" => "theme-group-accents",
    "text" => "theme-group-text",
    "machine states" => "theme-group-states",
    "alarm surface" => "theme-group-alarm",
    "console" => "theme-group-console",
    "toolpath" => "theme-group-toolpath",
    "chrome & surfaces" => "theme-group-chrome",
    _ => "theme-group-other",
  }
}

/// The editor section a palette token belongs to, keyed off the token (field) name. Tokens arrive from
/// `color_entries_mut` in declaration order, which is already grouped, so the editor emits a header whenever the
/// group changes. The mapping is total — an unmatched token lands in "other" (and a unit test keeps that empty).
pub(crate) fn group_for(token: &str) -> &'static str {
  if token.starts_with("accent") {
    "accents"
  } else if token.starts_with("text") {
    "text"
  } else if token.starts_with("state_") {
    "machine states"
  } else if token.starts_with("alarm_") {
    "alarm surface"
  } else if token.starts_with("log_") {
    "console"
  } else if token.starts_with("toolpath_") || token.starts_with("grid_") {
    "toolpath"
  } else if matches!(
    token,
    "bg" | "panel" | "panel_alt" | "inset" | "widget" | "widget_hover" | "widget_active" | "divider"
      | "border_recess" | "border_raised"
  ) {
    "chrome & surfaces"
  } else {
    "other"
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::app::theme::Palette;

  #[test]
  fn every_palette_token_maps_to_a_named_editor_group() {
    // The group mapping must be total over the real token list: an unmapped token would land under a stray
    // "other" header, which is the drift this guards (add a palette token → extend `group_for` or its prefixes).
    let mut theme = ThemeOverride::from_palette(&Palette::default_dark());
    for (name, _) in theme.color_entries_mut() {
      assert_ne!(group_for(name), "other", "token {name} must belong to a named editor group");
    }
  }

  #[test]
  fn builtin_theme_list_matches_the_resolver() {
    // The picker's built-in list and the config resolver must agree, or the dialog offers a theme that resolves
    // to the default with a notice (or misses one that exists).
    for name in BUILTIN_THEMES {
      assert!(crate::config::builtin_palette(name).is_some(), "{name} must be a resolvable built-in");
    }
    for known in ["default", "light-slate", "midnight"] {
      assert!(BUILTIN_THEMES.contains(&known), "the picker must offer the {known} built-in");
    }
  }

  #[test]
  fn language_display_names_cover_the_bundled_locales() {
    for (locale, _) in crate::i18n::BUNDLED_LOCALES {
      assert_ne!(
        language_display_name(locale),
        locale.to_string(),
        "bundled locale {locale} should have a human-readable picker name"
      );
    }
    // An unknown tag falls back to itself rather than being hidden or panicking.
    assert_eq!(language_display_name("de-DE"), "de-DE");
  }
}
