//! Persistence: the app config and the operator profile (rotary center, prefs, saved mesh).
//! Split out of `shell.rs` (A4); pure relocation.

use super::*;

impl SkirnirApp {
  /// Write the in-memory config to `config.json` (the test-injected override path when set). Surfaces the outcome
  /// as a console notice; a success clears the dialog's unsaved-changes marker. Note the write re-serialises the
  /// whole file, so hand-authored comments/formatting in an operator-edited config are not preserved.
  pub(crate) fn save_config(&mut self) {
    let result = match &self.config_path_override {
      Some(path) => crate::config::save_to(&self.config, path),
      None => crate::config::save(&self.config),
    };
    match result {
      Ok(()) => {
        self.config_dirty = false;
        match &self.config_path_override {
          Some(path) => self.notice(format!("saved config to {}", path.display())),
          None => match crate::config::config_path() {
            Some(path) => self.notice(format!("saved config to {}", path.display())),
            None => self.notice("saved config".to_string()),
          },
        }
      }
      Err(reason) => self.notice(format!("could not save config: {reason}")),
    }
  }

  /// Re-load the app config from disk and re-apply the APPEARANCE only: re-resolve the palette/toolpath style and
  /// re-skin the live egui visuals so a theme/colour edit takes effect without a restart. The from-scratch UI
  /// defaults (jog/DRO/console knobs) are deliberately NOT re-applied — a reload must preserve the operator's
  /// in-session changes to those (the reported F5 bug where reload wiped a hand-set jog step). The
  /// connection/reconnect knobs are likewise not re-applied to a live session (they bind at connect time). If a
  /// program is loaded, its cached toolpath is re-flattened at the new arc density so an `arc_step_deg` edit takes
  /// effect immediately rather than waiting for a GCode reload. Load failures fall back to defaults with notices,
  /// like the startup load. `ctx` is the live egui context whose visuals are re-skinned; notices surface in the console.
  pub(crate) fn reload_config(&mut self, ctx: &egui::Context) {
    let (config, notices) = match &self.config_path_override {
      Some(path) => crate::config::load_from(path),
      None => crate::config::load(),
    };
    // Appearance ONLY — leave the operator's session-modified UI knobs untouched.
    apply_appearance(&mut self.ui, &config);
    // The toolpath render STYLE just changed (strokes/grid/marker/colours update on next paint), but the cached arc
    // geometry was flattened at the OLD chord density. Re-flatten the loaded program at the new resolution so an
    // `arc_step_deg` edit is visible without reloading the GCode. A no-op when no program is loaded.
    self.ui.reflow_toolpath();
    // Re-skin the live window from the freshly-resolved palette + font scale so the reload is visible immediately.
    let (palette, _) = config.palette();
    apply_theme(ctx, &palette, config.appearance.font_scale);
    self.config = config;
    self.notice("reloaded config.json".to_string());
    for reason in notices {
      self.notice(reason);
    }
  }

  /// Fold the current connection/UI prefs into the in-memory profile, then persist the whole thing to disk. The
  /// rotary `RotarySetup` is written separately at the moment a center is found/saved ([`Self::save_rotary_center`])
  /// — here we only refresh the prefs from the live [`UiState`] so the last port/baud and rotary input defaults
  /// survive. A write failure is surfaced as a notice (never a panic): the in-memory profile is still correct,
  /// only the durable mirror lagged.
  pub(crate) fn save_profile(&mut self) {
    self.snapshot_prefs();
    if let Err(err) = self.persist_profile() {
      self.notice(format!("could not save profile: {err}"));
    }
  }

  /// Refresh the profile's prefs section from the live [`UiState`] (last port/baud, rotary input defaults). The
  /// rotary `RotarySetup` is set separately; this only mirrors the "remember my last entry" widget fields.
  pub(crate) fn snapshot_prefs(&mut self) {
    self.profile.prefs = crate::profile::Prefs {
      last_port: if self.ui.selected_port.is_empty() { None } else { Some(self.ui.selected_port.clone()) },
      baud: self.ui.baud,
      rotary_dowel_diameter: self.ui.rotary_dowel_diameter,
      rotary_index_angle: self.ui.rotary_index_angle,
      rotary_bench: self.ui.rotary_bench,
      datum_bench: self.ui.datum_bench,
      grid_bench: self.ui.mesh_bench,
      dock_fraction: self.ui.dock_fraction,
    };
  }

  /// Write the in-memory profile to disk: the test-injected [`Self::profile_path_override`] when set, else the
  /// default OS config location. The single I/O seam so the persistence wiring is hermetically testable.
  pub(crate) fn persist_profile(&self) -> Result<(), crate::profile::ProfileError> {
    match &self.profile_path_override {
      Some(path) => crate::profile::save_to(&self.profile, path),
      None => crate::profile::save(&self.profile),
    }
  }

  /// Persist a found rotary center (DOC-11 §1.3) into the profile and to disk, so a restart can re-apply it
  /// without re-probing. Records `(Y_c, Z_c, D, A-datum, Z-datum)` from the live wizard state. Surfaces a write
  /// failure as a notice rather than panicking; the in-memory center remains usable this session either way.
  pub(crate) fn save_rotary_center(&mut self, setup: crate::profile::RotarySetup) {
    self.profile.rotary = Some(setup);
    self.save_profile();
  }
}
