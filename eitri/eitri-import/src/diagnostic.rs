//! Import diagnostics.
//!
//! Every importer skips *something* — an unsupported SVG node, a DXF `SPLINE`, an out-of-plane arc. The plan's
//! discipline (plan §6) is to **skip loudly**: never silently drop input geometry, always record what was dropped
//! and why, and hand that list back beside the recovered geometry so a caller (or the UI) can surface it.

/// One thing an importer chose not to convert, with enough detail to tell an operator what was lost. `what` names
/// the input construct (`"DXF SPLINE entity"`, `"SVG <image> node"`); `reason` says why it was skipped.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Skipped {
  /// The input construct that was not converted.
  pub what: String,
  /// Why it was skipped (unsupported, degenerate, out-of-plane, ...).
  pub reason: String,
}

impl Skipped {
  /// Build a diagnostic from any two string-ish parts.
  pub fn new(what: impl Into<String>, reason: impl Into<String>) -> Skipped {
    Skipped { what: what.into(), reason: reason.into() }
  }
}
