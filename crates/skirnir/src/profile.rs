//! The skirnir profile/project store: the persisted, cross-session host state.
//!
//! skirnir owns state the firmware has no concept of — most importantly the rotary A centerline (its machine-Y
//! and machine-Z), found by the DOC-11 §1.2 center-finder and projected into the firmware only as a `G10` WCS
//! offset (DOC-11 §1.3). That value used to evaporate on exit, forcing a full re-probe every restart. This module
//! gives it (and a handful of sensible connection/UI defaults) a durable home.
//!
//! **Mechanism — a versioned RON file under the OS config dir, NOT eframe's `set_value`/`get_value`.** The store
//! is framework-agnostic (no egui, no eframe `Storage`) so the headless `--cli` path can read/write the same
//! project state as the GUI, and so the whole thing is a pure serialize/deserialize round-trip that unit-tests
//! with neither a window nor hardware. RON is the egui ecosystem's native, human-readable serde format; the file
//! is hand-editable and reviewable. A leading [`Profile::version`] field carries forward-compat: an unknown
//! newer version is refused (rather than silently misread), and any read failure — missing file, malformed RON,
//! a permissions error — falls back to [`Profile::default`] so a bad profile can never take the app down.
//!
//! **I/O is non-fatal by contract.** [`load`] never returns an error for "no profile yet" (that is just
//! defaults) and downgrades a corrupt/unreadable file to defaults with the reason carried back for the caller to
//! surface as a console notice. [`save`] returns a typed [`ProfileError`] the caller reports rather than
//! unwraps — a failed save must surface, never crash the UI.

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::app::rotary_center::ZDatum;

/// The on-disk schema version. Bump on any backwards-incompatible change to [`Profile`]'s shape; a file whose
/// `version` exceeds this is refused by [`Profile::from_ron`] (we cannot know how to read a future layout), while
/// an older-or-equal version is accepted and missing fields fill from `#[serde(default)]`. This is the single
/// knob that makes the format forward-aware.
pub const PROFILE_VERSION: u32 = 1;

/// The file name written under the per-user config directory (e.g. `~/.config/skirnir/profile.ron` on Linux).
const PROFILE_FILE: &str = "profile.ron";

/// The application identifier the profile path is expected to live under (e.g. `~/.config/skirnir` on Linux,
/// `~/Library/Application Support/skirnir` on macOS). The directory itself is resolved by [`crate::store::config_dir`]
/// now; this constant is retained only for the path-shape assertion in the tests, hence `#[cfg(test)]`.
#[cfg(test)]
const APP_NAME: &str = "skirnir";

/// A typed failure from the profile store. Read failures are NOT modelled here — [`load`] downgrades them to
/// defaults — so this surface is only the things a *save* (or an explicit path resolution) can fail on, which
/// the caller reports rather than panics on.
#[derive(Debug, thiserror::Error)]
pub enum ProfileError {
  /// The OS config directory could not be resolved (no `HOME`/XDG base on this platform). Persisting is then
  /// impossible; the caller surfaces this and runs with in-memory defaults.
  #[error("could not resolve a config directory for the profile")]
  NoConfigDir,

  /// Creating the config directory or writing the file failed (permissions, full disk, …). Carries the detail.
  #[error("failed to write the profile: {0}")]
  Io(String),

  /// Serialising the profile to RON failed — a logic error rather than an environment one, surfaced uniformly.
  #[error("failed to serialise the profile: {0}")]
  Serialize(String),
}

/// The persisted rotary-A setup found by the center-finder (DOC-11 §1.2/§1.3). The firmware has no pivot concept,
/// so `(Y_c, Z_c)` are skirnir-owned machine coordinates re-applied as a `G10` WCS offset; the dowel diameter and
/// A-datum/index angle that produced them are kept so a session can be resumed (or re-written) without re-probing.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RotarySetup {
  /// The rotary axis centerline's machine-Y (`Y_c = (Y_left + Y_right)/2`).
  pub y_center: f64,
  /// The rotary axis centerline's machine-Z (`Z_c = Z_top − D/2`). Always the AXIS math, independent of the
  /// chosen [`Self::z_datum`] — the datum only selects which Z a `G10` write puts at work-Z0.
  pub z_center: f64,
  /// The dowel/gauge diameter `D` (mm) used to derive `z_center`. Kept so the value is auditable and a re-write
  /// reproduces the same geometry.
  pub dowel_diameter: f64,
  /// The A-datum / index angle (degrees) every center-finder touch held A at. The dowel is round, so this is
  /// mostly the angle A was held still at; recorded as the rotary datum the center was found against.
  pub a_datum_deg: f64,
  /// Which feature work-Z0 was chosen to land on when the center is written to the WCS (axis centerline vs the
  /// probed top surface). Reused from the live wizard so there is a single source of truth for the enum.
  pub z_datum: ZDatum,
}

impl RotarySetup {
  /// The machine-Z this setup writes to work-Z0, per the saved [`Self::z_datum`]: `AxisCenterline → z_center`
  /// (the wrap-machining default) or `TopSurface → z_center + dowel_diameter/2` (the probed cylinder top, since
  /// `z_top = z_center + D/2`). Kept here so a re-apply reproduces the wizard's chosen datum exactly.
  pub fn z_datum_value(&self) -> f64 {
    match self.z_datum {
      ZDatum::AxisCenterline => self.z_center,
      // `z_center = z_top − D/2`, so the probed top is `z_center + D/2`.
      ZDatum::TopSurface => self.z_center + self.dowel_diameter / 2.0,
    }
  }

  /// The `G10 L2 P0` line that re-applies this saved center to the active WCS, IDENTICAL in form to the live
  /// wizard's [`crate::app::rotary_center::WizardState::offer_g10`]: Y is always the axis centerline (`y_center`),
  /// Z follows the saved datum, and the line carries ONLY Y and Z — never an `A` word, so the rotary datum is
  /// left untouched. This is the payoff of DOC-11 §1.3: a restart re-applies the found center without re-probing.
  pub fn offer_g10(&self) -> String {
    format!("G10 L2 P0 Y{:.3} Z{:.3}", self.y_center, self.z_datum_value())
  }
}

/// Connection + UI defaults worth remembering across sessions, kept deliberately tight: only the inputs an
/// operator re-enters identically run after run. Transient runtime state (the live status, the console, an
/// in-flight edit) is NEVER persisted — it belongs to the session, not the profile.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Prefs {
  /// The device path of the last port a connect opened, offered as the default selection next launch. `None`
  /// until the first successful connect.
  pub last_port: Option<String>,
  /// The baud rate last used (the ESP32-S3 native USB ignores it, but the host driver and the dropdown want a
  /// value, and the operator's choice is worth keeping).
  pub baud: u32,
  /// The rotary center-finder's dowel-diameter input default (mm) — remembered so the common gauge size is
  /// pre-filled. Distinct from a *found* [`RotarySetup::dowel_diameter`]: this is the unfilled-form default.
  pub rotary_dowel_diameter: f64,
  /// The rotary center-finder's index-angle input default (degrees).
  pub rotary_index_angle: f64,
  /// The rotary-safe touch's bench-tuned parameters (retract clearance, side-probe height, settle, feed, depth) —
  /// remembered so the per-bench values an operator dials in once survive a session. `#[serde(default)]` so a
  /// profile written before this block existed still loads, defaulting the whole struct rather than failing to
  /// parse on the missing field.
  #[serde(default)]
  pub rotary_bench: crate::app::rotary_probe::RotaryProbeParams,
  /// The console dock's share of the central region (`0..1`) — the operator's dragged split ratio, restored on
  /// the next launch. `#[serde(default = ...)]` so a profile written before the tiles split still loads.
  #[serde(default = "default_dock_fraction")]
  pub dock_fraction: f32,
}

/// The fraction of the central region the console dock opens with on a fresh profile (~200px of a typical
/// ~660px central region, the design's dock height as closely as a share-based split can express it). Lives
/// here (not in the gui-gated `dock_tiles`) because the profile compiles in `--no-default-features` builds too;
/// the gui side re-exports it.
pub const DEFAULT_DOCK_FRACTION: f32 = 0.3;

/// The serde default for [`Prefs::dock_fraction`] on profiles that predate the tiles split.
fn default_dock_fraction() -> f32 {
  DEFAULT_DOCK_FRACTION
}

impl Default for Prefs {
  fn default() -> Self {
    // These mirror the live `UiState` defaults so a fresh profile and a fresh UI agree out of the box.
    Prefs {
      last_port: None,
      baud: 115_200,
      rotary_dowel_diameter: 6.0,
      rotary_index_angle: 0.0,
      rotary_bench: crate::app::rotary_probe::RotaryProbeParams::default(),
      dock_fraction: DEFAULT_DOCK_FRACTION,
    }
  }
}

/// The whole persisted profile: a versioned envelope around the rotary setup and the connection/UI prefs. The
/// rest of skirnir can grow new sections here behind `#[serde(default)]` without breaking older files.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Profile {
  /// The on-disk schema version, written as [`PROFILE_VERSION`]. Read back by [`Self::from_ron`] to refuse a
  /// future layout and to drive any future migration.
  pub version: u32,
  /// The persisted rotary-A center setup, or `None` until a center-finder run has been saved.
  #[serde(default)]
  pub rotary: Option<RotarySetup>,
  /// Connection + UI defaults remembered across sessions.
  #[serde(default)]
  pub prefs: Prefs,
}

impl Default for Profile {
  fn default() -> Self {
    Profile { version: PROFILE_VERSION, rotary: None, prefs: Prefs::default() }
  }
}

impl Profile {
  /// Serialise to pretty RON (the on-disk form), with a trailing newline so the file is a tidy text file. A
  /// serialise failure is surfaced as [`ProfileError::Serialize`] rather than unwrapped.
  pub fn to_ron(&self) -> Result<String, ProfileError> {
    let pretty = ron::ser::PrettyConfig::default();
    ron::ser::to_string_pretty(self, pretty)
      .map(|mut body| {
        body.push('\n');
        body
      })
      .map_err(|err| ProfileError::Serialize(err.to_string()))
  }

  /// Parse a profile from RON text. Returns `Ok(None)` for a malformed body OR a version newer than this build
  /// understands — both are "fall back to defaults" cases the caller turns into a defaulted profile plus a
  /// notice — and never panics. A valid, understood file parses to `Ok(Some(profile))`.
  ///
  /// Why `Option` rather than a hard error on a bad parse: a corrupt or future profile must not take the app
  /// down, and the *reason* (corrupt vs too-new) is not actionable beyond "using defaults", so the caller only
  /// needs to know "no usable profile here". The distinction is logged at the call site, not encoded in a type.
  pub fn from_ron(body: &str) -> Option<Self> {
    let profile: Profile = ron::from_str(body).ok()?;
    // A file from a newer skirnir may carry fields/shapes this build cannot interpret; refuse it rather than
    // risk misreading, and let the caller fall back to defaults (the existing file is left untouched on disk).
    if profile.version > PROFILE_VERSION {
      return None;
    }
    Some(profile)
  }
}

/// Resolve the absolute path of the profile file under the OS config directory (e.g.
/// `~/Library/Application Support/skirnir/profile.ron` on macOS, `~/.config/skirnir/profile.ron` on Linux), creating
/// nothing. Returns [`ProfileError::NoConfigDir`] if no per-user config base exists on this platform. Public so the
/// GUI can show the operator where the profile lives. Shares [`crate::store::config_dir`] with the other stores so
/// all three resolve to the same directory from one place.
pub fn profile_path() -> Result<PathBuf, ProfileError> {
  let dir = crate::store::config_dir().map_err(|_| ProfileError::NoConfigDir)?;
  Ok(dir.join(PROFILE_FILE))
}

/// Load the profile from the default OS location. NEVER fails for the common cases: a missing file (first run)
/// and an unreadable/corrupt/too-new file both resolve to [`Profile::default`]. The returned tuple carries the
/// profile plus an optional human-readable reason the fallback happened, for the caller to surface as a notice
/// (a missing file reports `None` — that is normal, not worth a notice).
pub fn load() -> (Profile, Option<String>) {
  match profile_path() {
    Ok(path) => load_from(&path),
    // No config dir at all: run on defaults and tell the caller why so it can warn that nothing will persist.
    Err(err) => (Profile::default(), Some(err.to_string())),
  }
}

/// Load the profile from a specific path — the testable core of [`load`]. A missing file is the first-run case
/// (defaults, no notice); a read error or an unparseable/too-new body is downgraded to defaults WITH a reason.
pub fn load_from(path: &Path) -> (Profile, Option<String>) {
  match std::fs::read_to_string(path) {
    Ok(body) => match Profile::from_ron(&body) {
      Some(profile) => (profile, None),
      // The file exists but we cannot use it (corrupt RON, or a version newer than we understand): fall back to
      // defaults and surface why. The bad file is left in place so the operator can inspect/recover it.
      None => (
        Profile::default(),
        Some(format!("profile at {} is unreadable or too new — using defaults", path.display())),
      ),
    },
    // A missing file is the ordinary first-run case: defaults, silently. Any other I/O error (permissions, …)
    // is worth a notice but still non-fatal — the app runs on defaults.
    Err(err) if err.kind() == std::io::ErrorKind::NotFound => (Profile::default(), None),
    Err(err) => (
      Profile::default(),
      Some(format!("could not read profile at {}: {err} — using defaults", path.display())),
    ),
  }
}

/// Save the profile to the default OS location, creating the config directory if needed. Returns a typed
/// [`ProfileError`] the caller surfaces (a failed save must be reported, never panic the UI).
pub fn save(profile: &Profile) -> Result<(), ProfileError> {
  let path = profile_path()?;
  save_to(profile, &path)
}

/// Save the profile to a specific path — the testable core of [`save`]. Delegates the crash-safe write to the
/// shared [`crate::store::atomic_write`]: it creates any missing parent directories, writes to a per-process temp
/// sibling, then atomically renames it into place, so an interrupted write never leaves a truncated, unparseable
/// profile (a torn profile would silently reset the operator's rotary center on the next launch). All failures
/// surface as [`ProfileError`], never a panic.
pub fn save_to(profile: &Profile, path: &Path) -> Result<(), ProfileError> {
  let body = profile.to_ron()?;
  crate::store::atomic_write(path, body.as_bytes()).map_err(|err| ProfileError::Io(err.to_string()))
}

#[cfg(test)]
mod tests {
  use super::*;

  /// A fully-populated profile for round-trip checks — every field set to a non-default value so a dropped or
  /// mis-wired field shows up as a mismatch rather than coincidentally matching the default.
  fn sample() -> Profile {
    Profile {
      version: PROFILE_VERSION,
      rotary: Some(RotarySetup {
        y_center: -12.5,
        z_center: -30.25,
        dowel_diameter: 8.0,
        a_datum_deg: 90.0,
        z_datum: ZDatum::TopSurface,
      }),
      prefs: Prefs {
        last_port: Some("/dev/cu.usbmodem31101".to_string()),
        baud: 250_000,
        rotary_dowel_diameter: 10.0,
        rotary_index_angle: 45.0,
        dock_fraction: 0.42,
        rotary_bench: crate::app::rotary_probe::RotaryProbeParams {
          clearance_mm: -3.0,
          settle_secs: 0.75,
          feed: 40.0,
          depth_mm: 12.0,
          side_probe_z: -9.0,
        },
      },
    }
  }

  #[test]
  fn a_saved_axis_centerline_setup_re_applies_the_wizard_form_g10_with_only_y_and_z() {
    // The §1.3 payoff: a persisted center re-applies identically to the live wizard's offer — Y is the axis,
    // Z is the axis (Z_top − D/2), and the line carries only Y/Z, never A.
    let setup = RotarySetup {
      y_center: 0.0,
      z_center: -13.0, // a 6 mm dowel topping at -10 → axis at -13.
      dowel_diameter: 6.0,
      a_datum_deg: 0.0,
      z_datum: ZDatum::AxisCenterline,
    };
    let line = setup.offer_g10();
    assert_eq!(line, "G10 L2 P0 Y0.000 Z-13.000", "the saved axis-datum re-apply must match the wizard's L2 line");
    assert!(!line.contains('A'), "the re-apply line must never carry an A word; got {line:?}");
  }

  #[test]
  fn a_saved_top_surface_setup_re_applies_the_probed_top_as_z() {
    // The alternate datum: the re-apply Z is the probed top (z_center + D/2), here -13 + 3 = -10. Y unchanged.
    let setup = RotarySetup {
      y_center: 0.0,
      z_center: -13.0,
      dowel_diameter: 6.0,
      a_datum_deg: 0.0,
      z_datum: ZDatum::TopSurface,
    };
    assert_eq!(setup.z_datum_value(), -10.0, "the top-surface datum re-applies the probed top, z_center + D/2");
    assert_eq!(setup.offer_g10(), "G10 L2 P0 Y0.000 Z-10.000");
  }

  #[test]
  fn round_trips_through_ron_preserving_every_field() {
    let profile = sample();
    let body = profile.to_ron().expect("a well-formed profile serialises");
    let parsed = Profile::from_ron(&body).expect("its own output parses back");
    assert_eq!(parsed, profile, "the profile must survive a serialise/deserialise round-trip unchanged");
  }

  #[test]
  fn the_serialised_form_is_human_readable_ron_with_a_trailing_newline() {
    let body = sample().to_ron().expect("serialises");
    assert!(body.ends_with('\n'), "the file should end with a newline; got {body:?}");
    // Spot-check that named fields are present (pretty RON), so the file is hand-editable/reviewable.
    assert!(body.contains("version"), "the version envelope must be written: {body}");
    assert!(body.contains("y_center"), "rotary fields must be written by name: {body}");
    assert!(body.contains("TopSurface"), "the ZDatum variant name is the on-disk token: {body}");
  }

  #[test]
  fn the_default_profile_carries_the_current_version_and_no_rotary() {
    let profile = Profile::default();
    assert_eq!(profile.version, PROFILE_VERSION);
    assert_eq!(profile.rotary, None, "a fresh profile has no found center yet");
    assert_eq!(profile.prefs, Prefs::default());
  }

  #[test]
  fn save_then_load_through_a_real_file_round_trips() {
    let dir = std::env::temp_dir().join(format!("skirnir-profile-test-{}", std::process::id()));
    let path = dir.join("nested").join("profile.ron"); // also exercises parent-dir creation.
    let _ = std::fs::remove_dir_all(&dir);
    let profile = sample();
    save_to(&profile, &path).expect("saving to a writable temp path succeeds");
    let (loaded, notice) = load_from(&path);
    assert_eq!(loaded, profile, "a saved profile must load back identically");
    assert_eq!(notice, None, "a clean load surfaces no notice");
    let _ = std::fs::remove_dir_all(&dir);
  }

  #[test]
  fn loading_a_missing_file_yields_defaults_with_no_notice() {
    // The first-run case: no file at the path. It must default silently — a missing profile is not an error.
    let path = std::env::temp_dir().join("skirnir-profile-does-not-exist-xyzzy.ron");
    let _ = std::fs::remove_file(&path);
    let (loaded, notice) = load_from(&path);
    assert_eq!(loaded, Profile::default(), "a missing file falls back to the default profile");
    assert_eq!(notice, None, "a missing file is the ordinary first run — no notice");
  }

  #[test]
  fn loading_a_corrupt_file_falls_back_to_defaults_with_a_notice_and_never_panics() {
    let dir = std::env::temp_dir().join(format!("skirnir-profile-corrupt-{}", std::process::id()));
    let path = dir.join("profile.ron");
    std::fs::create_dir_all(&dir).expect("temp dir");
    std::fs::write(&path, "this is not valid RON {{{").expect("write a garbage file");
    let (loaded, notice) = load_from(&path);
    assert_eq!(loaded, Profile::default(), "a corrupt file must fall back to defaults, not crash");
    assert!(notice.is_some(), "a corrupt file should surface a notice so the operator knows defaults are in use");
    let _ = std::fs::remove_dir_all(&dir);
  }

  #[test]
  fn a_profile_from_a_newer_version_is_refused_in_favour_of_defaults() {
    // Forward-compat guard: a file written by a future skirnir (higher version) must not be misread. We cannot
    // know its layout, so `from_ron` refuses it and the caller defaults.
    let future = format!("(version:{},rotary:None,prefs:(last_port:None,baud:115200,rotary_dowel_diameter:6.0,rotary_index_angle:0.0))", PROFILE_VERSION + 1);
    assert_eq!(Profile::from_ron(&future), None, "a newer-versioned profile must be refused");
  }

  #[test]
  fn an_equal_or_older_version_is_accepted() {
    // The current version parses; this is the normal path and guards against an off-by-one in the version gate.
    let body = sample().to_ron().expect("serialises");
    assert!(Profile::from_ron(&body).is_some(), "the current version must be accepted");
  }

  #[test]
  fn missing_optional_sections_fill_from_defaults() {
    // Forward-tolerance the other way: a minimal file carrying only the version (an older/leaner profile) must
    // load, with the absent sections defaulting rather than failing to parse. This is what `#[serde(default)]`
    // on `rotary`/`prefs` buys us, so a future field-add stays backwards-compatible.
    let minimal = format!("(version:{})", PROFILE_VERSION);
    let parsed = Profile::from_ron(&minimal).expect("a version-only profile must parse via serde defaults");
    assert_eq!(parsed.rotary, None);
    assert_eq!(parsed.prefs, Prefs::default());
  }

  #[test]
  fn a_prefs_block_without_the_bench_params_loads_with_them_defaulted() {
    // The exact backward-compat case for adding `rotary_bench`: a profile written before that block existed
    // carries a full `prefs` minus the new field. `#[serde(default)]` on `rotary_bench` must let it parse, with
    // the bench params filled from `RotaryProbeParams::default()` rather than failing the whole load.
    let pre = format!(
      "(version:{},rotary:None,prefs:(last_port:Some(\"/dev/ttyACM0\"),baud:115200,rotary_dowel_diameter:8.0,rotary_index_angle:30.0))",
      PROFILE_VERSION,
    );
    let parsed = Profile::from_ron(&pre).expect("a pre-bench prefs block must still parse");
    assert_eq!(parsed.prefs.rotary_dowel_diameter, 8.0, "the existing prefs fields must survive");
    assert_eq!(parsed.prefs.rotary_index_angle, 30.0);
    assert_eq!(
      parsed.prefs.rotary_bench,
      crate::app::rotary_probe::RotaryProbeParams::default(),
      "the absent bench block must default, not fail the load",
    );
  }

  #[test]
  fn profile_path_lands_under_an_app_named_config_dir() {
    // We cannot assert the exact base across CI platforms, but the resolved path must end at our file under an
    // app-named directory. On a platform with no config base this errors instead — also acceptable (load() then
    // defaults). Either branch is fine; if it resolves, the shape must be right.
    if let Ok(path) = profile_path() {
      assert!(path.ends_with(PROFILE_FILE), "the path must end at the profile file: {}", path.display());
      assert!(
        path.to_string_lossy().contains(APP_NAME),
        "the profile must live under an app-named dir: {}",
        path.display(),
      );
    }
  }

  #[test]
  fn a_save_leaves_only_the_target_and_no_temp_behind() {
    // The save delegates the crash-safe write to `crate::store::atomic_write` (the sibling-temp/rename invariant is
    // pinned in `store.rs`'s own tests). Here we assert the profile-level outcome: after a successful save the
    // target exists and no leftover temp litters the directory.
    let dir = std::env::temp_dir().join(format!("skirnir-profile-atomic-{}", std::process::id()));
    let path = dir.join("profile.ron");
    let _ = std::fs::remove_dir_all(&dir);
    save_to(&sample(), &path).expect("save succeeds");
    assert!(path.exists(), "the target profile must exist after save");
    // No sibling file other than the target should remain (the temp was renamed into place).
    let leftovers: Vec<_> = std::fs::read_dir(&dir)
      .expect("the dir exists")
      .filter_map(|e| e.ok().map(|e| e.file_name()))
      .filter(|name| name != "profile.ron")
      .collect();
    assert!(leftovers.is_empty(), "no temp file must be left behind, found: {leftovers:?}");
    let _ = std::fs::remove_dir_all(&dir);
  }
}
