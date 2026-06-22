---
name: profile-persistence
description: DOC-11 §1.3 profile/project store — versioned RON under OS config dir, framework-agnostic, shell wiring + test seam
metadata:
  type: project
---

skirnir's cross-session persistence layer (DOC-11 §1.3, the last DOC-11 piece). Built as a framework-agnostic
versioned RON file, NOT eframe's `set_value`/`get_value`.

**Why RON-file over eframe persistence:** the `--cli` headless path (no `gui` feature) has no eframe `Storage`,
and a project file should be shared by both paths. The store is pure serialize/deserialize, unit-tested with no
window and no hardware. eframe's `persistence` feature is NOT enabled.

**Where it lives:**
- `crates/skirnir/src/profile.rs` — top-level lib module (always compiled). `Profile` (versioned envelope),
  `RotarySetup`, `Prefs`, `ProfileError`. `load()/save()` hit the OS path; `load_from(path)/save_to(path)` are the
  testable cores. `profile_path()` → `~/.config/skirnir/profile.ron` via `directories::ProjectDirs::from("","","skirnir")`.
- `PROFILE_VERSION = 1`; `from_ron` refuses a newer version (falls back to defaults). `#[serde(default)]` on
  `rotary`/`prefs` gives backwards-tolerance for older/leaner files.
- Save is atomic: temp sibling (`profile.ron.tmp`) + rename, so a torn write never resets the operator's center.
- I/O is non-fatal by contract: missing file → defaults silently; corrupt/too-new → defaults + a notice string;
  a failed save surfaces as a console notice, never panics.

**Persisted fields:** `RotarySetup{ y_center, z_center, dowel_diameter, a_datum_deg, z_datum }` (the found rotary
center) + `Prefs{ last_port, baud, rotary_dowel_diameter, rotary_index_angle }` (UI defaults). `ZDatum` (in
`app/rotary_center.rs`) got `serde::{Serialize,Deserialize}` derives — variant names are the on-disk tokens.

**Shell wiring (`app/shell.rs`):**
- `SkirnirApp.profile` (loaded in `new`, seeds `UiState::from_prefs`) + `profile_path_override: Option<PathBuf>`
  (the test seam; `None` in prod). `persist_profile()` routes through the override or the OS path; `snapshot_prefs()`
  mirrors live UiState into prefs. `save_profile()` = snapshot + persist + notice-on-error.
- Save points: `rotary_center_write_wcs` (snapshots the found center), `connect` (remembers port/baud), and
  `eframe::App::on_exit(&mut self, Option<&eframe::glow::Context>)` backstop (fires on shutdown WITHOUT the
  persistence feature, gated by the `glow` feature which is on).
- `Intent::ApplySavedRotaryCenter` + `apply_saved_rotary_center()` re-emit the saved `RotarySetup::offer_g10()`
  line (identical form to `WizardState::offer_g10`, Y/Z only, never A) so a restart restores the center without
  re-probing — the §1.3 payoff. The rotary_center view gained a `has_saved_center: bool` arg + "Apply saved
  center" button in the no-run branch.

**Test hermeticity gotcha:** `app_with_engine()` (shell tests) calls `SkirnirApp::new` which reads the REAL OS
profile. The helper now resets `app.profile = Profile::default()` and sets a unique temp `profile_path_override`
so NO shell test ever writes `~/.config/skirnir`. Several existing tests call `rotary_center_write_wcs` (which now
saves), so this redirect is load-bearing.

**Deps added (vetted, conservative):** `serde = { version = "1", features = ["derive"] }` (unifies on existing
1.0.228), `ron = "0.12"` (→0.12.1), `directories = "6"`. Net new lock crates: ron, directories, dirs-sys, typeid.
No major duplication.

Test delta: ~349 → 366 (13 profile-module tests + 2 shell integration tests for persist-and-restore / re-apply,
plus 2 RotarySetup::offer_g10 datum tests). All green with `RUSTFLAGS="-D warnings"`; gui + no-default-features +
serial-only all build clean; clippy clean.
