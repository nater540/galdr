//! The one-way UI-intent vocabulary: views *emit* [`Intent`]s into an [`IntentSink`]; the shell drains the sink
//! after the frame and performs the side effects (engine commands, file dialogs, config writes). Mirrors
//! skirnir's intent layer: egui-free, so the vocabulary and the sink are host-testable, and no view ever
//! mutates the session or the config directly.

use eitri_project::ObjectId;

use crate::config::ThemeOverride;

/// Everything a view can ask the shell to do. One frame may emit several; the shell handles them in order.
#[derive(Debug, Clone, PartialEq)]
pub enum Intent {
  // ── Files (the shell shows the dialog, then spawns the op) ─────────────────────────────────────────────
  /// Pick a Gerber file and open it as a new object.
  OpenGerber,
  /// Pick an Excellon file and open it as a new object.
  OpenExcellon,
  /// Pick an SVG file and import it as geometry.
  ImportSvg,
  /// Pick a DXF file and import it as geometry.
  ImportDxf,
  /// Pick a G-code file and import its cut moves as geometry.
  ImportGcode,
  /// Pick a project file and load it, replacing the current session.
  OpenProject,
  /// Pick a destination and save the current project as versioned JSON.
  SaveProject,
  /// Pick a destination and write a CNC job's G-code.
  ExportGcode(ObjectId),

  // ── Collection ─────────────────────────────────────────────────────────────────────────────────────────
  /// Select an object in the tree (or clear the selection with `None`).
  Select(Option<ObjectId>),
  /// Show or hide an object on the canvas (undoable in the engine's history).
  SetVisible(ObjectId, bool),
  /// Rename an object to the given (already-trimmed) name.
  Rename(ObjectId, String),
  /// Delete an object (undoable in the engine's history).
  DeleteObject(ObjectId),
  /// Undo the last collection edit.
  Undo,
  /// Redo the last undone edit.
  Redo,

  // ── CAM operations (run off-thread through the op runner) ─────────────────────────────────────────────
  /// Run isolation routing on a copper/geometry source with the drafts currently in the parameter panel.
  RunIsolate(ObjectId),
  /// Run drill planning on an Excellon source with the current drill drafts.
  RunDrill(ObjectId),
  /// Run area clearing (paint) on a copper/geometry source with the current paint drafts.
  RunPaint(ObjectId),
  /// Run non-copper clearing on a copper/geometry source (the shell resolves the drafted boundary).
  RunNonCopper(ObjectId),
  /// Run a board cutout for the selected source (the shell resolves the drafted outline).
  RunCutout(ObjectId),
  /// Panelize a copper/geometry source into a grid (commits a geometry object).
  RunPanelize(ObjectId),
  /// Mirror a copper/geometry source about the drafted line (commits a geometry object).
  RunMirror(ObjectId),
  /// Export a copper/geometry source as a photo-film SVG via a save dialog (vector output, not an op run).
  ExportFilm(ObjectId),
  /// Request cancellation of the in-flight operation.
  CancelOp,

  // ── Canvas ─────────────────────────────────────────────────────────────────────────────────────────────
  /// Zoom the canvas to fit the loaded geometry.
  ZoomFit,

  // ── Appearance / settings ─────────────────────────────────────────────────────────────────────────────
  /// Open the application-settings dialog.
  OpenAppSettings,
  /// Select a UI locale on the global i18n registry (and record it in the in-memory config).
  SetLanguage(String),
  /// Select the active theme by name and re-skin the live context.
  SetActiveTheme(String),
  /// Apply a global font scale.
  SetFontScale(f32),
  /// Create or replace a user theme (the settings dialog's colour editor emits whole-theme upserts).
  UpsertTheme {
    /// The theme's config key.
    name: String,
    /// The full override to store under that key.
    theme: ThemeOverride,
  },
  /// Write the in-memory config to disk (the explicit Save boundary).
  SaveConfig,
}

/// The per-frame collector views push into. Drained by the shell after the frame is built.
#[derive(Debug, Default)]
pub struct IntentSink {
  intents: Vec<Intent>,
}

impl IntentSink {
  /// An empty sink for this frame.
  pub fn new() -> Self {
    Self::default()
  }

  /// Queue an intent for the shell.
  pub fn push(&mut self, intent: Intent) {
    self.intents.push(intent);
  }

  /// Take every queued intent, in emit order, leaving the sink empty.
  pub fn drain(&mut self) -> Vec<Intent> {
    std::mem::take(&mut self.intents)
  }

  /// Whether nothing has been queued this frame.
  pub fn is_empty(&self) -> bool {
    self.intents.is_empty()
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn the_sink_preserves_emit_order_and_drains_clean() {
    let mut sink = IntentSink::new();
    assert!(sink.is_empty());
    sink.push(Intent::Undo);
    sink.push(Intent::Select(None));
    sink.push(Intent::ZoomFit);
    assert!(!sink.is_empty());
    assert_eq!(sink.drain(), vec![Intent::Undo, Intent::Select(None), Intent::ZoomFit]);
    assert!(sink.is_empty(), "drain must leave the sink empty for the next frame");
    assert_eq!(sink.drain(), Vec::<Intent>::new());
  }
}
