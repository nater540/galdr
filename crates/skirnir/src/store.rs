//! Shared host-side persistence primitives for the skirnir config stores: the per-user config directory and an
//! atomic file write. Both [`crate::profile`] (RON) and [`crate::config`] (JSON) persist under the same OS config
//! dir and both want the same crash-safe write, so the mechanics live here once rather than re-implemented per store.
//!
//! **[`config_dir`]** resolves the OS config dir (e.g. `~/Library/Application Support/skirnir` on macOS,
//! `~/.config/skirnir` on Linux) via [`directories::ProjectDirs`], the same `("", "", "skirnir")` triple the
//! profile/setting-help stores already use, so every store lands in one directory. **[`atomic_write`]** writes to a
//! sibling temp file and renames it into
//! place, so an interrupted write can never leave a half-written, unparseable file behind: a crash mid-write loses
//! the new content but keeps the old file intact. The temp shares the target's directory so the rename stays on one
//! filesystem (a cross-device rename fails).
//!
//! Neither helper panics; both return a typed [`StoreError`] the caller surfaces (a failed save is reported, never
//! crashes the UI). Both [`profile`](crate::profile) and [`config`](crate::config) build on these — the hand-rolled
//! directory resolution and temp-write/rename that profile once carried have been deleted in favour of this single
//! implementation. (`setting_help` still resolves its own path; it can adopt [`config_dir`] later.)

use std::path::{Path, PathBuf};

/// The application identifier used to locate the OS config directory via [`directories::ProjectDirs`]. The Linux/XDG
/// resolution ignores the qualifier/organisation, yielding `~/.config/skirnir` (on macOS it is
/// `~/Library/Application Support/skirnir`); this matches the value [`crate::profile`] and
/// [`crate::app::setting_help`] use so all three stores resolve to the same directory.
const APP_NAME: &str = "skirnir";

/// A typed failure from the shared store helpers. Read failures are not modelled — callers downgrade those to
/// defaults — so this surface is only what a path resolution or a write can fail on, which the caller reports.
#[derive(Debug, thiserror::Error)]
pub enum StoreError {
  /// The OS config directory could not be resolved (no `HOME`/XDG base on this platform). Persisting is then
  /// impossible; the caller surfaces this and runs with in-memory defaults.
  #[error("could not resolve a config directory")]
  NoConfigDir,

  /// Creating the config directory or writing/renaming the file failed (permissions, full disk, …). Carries detail.
  #[error("config I/O failed: {0}")]
  Io(String),
}

/// Resolve the absolute path of skirnir's per-user config directory (e.g. `~/Library/Application Support/skirnir` on
/// macOS, `~/.config/skirnir` on Linux), creating nothing. Returns [`StoreError::NoConfigDir`] when no per-user
/// config base exists on this platform. Public so each store can build its own file path under one shared directory.
pub fn config_dir() -> Result<PathBuf, StoreError> {
  let dirs = directories::ProjectDirs::from("", "", APP_NAME).ok_or(StoreError::NoConfigDir)?;
  Ok(dirs.config_dir().to_path_buf())
}

/// Write `bytes` to `path` atomically: create any missing parent directories, write to a sibling temp file, then
/// rename it over the target. An interrupted write leaves the old file intact rather than a truncated one; a
/// leftover temp from a crashed run is harmless (it is overwritten next save). The temp lives in the target's parent
/// so the rename stays on one filesystem — a cross-device rename fails. All failures surface as [`StoreError::Io`],
/// never a panic.
pub fn atomic_write(path: &Path, bytes: &[u8]) -> Result<(), StoreError> {
  if let Some(parent) = path.parent() {
    std::fs::create_dir_all(parent).map_err(|err| StoreError::Io(err.to_string()))?;
  }
  // A `.tmp`-suffixed sibling, distinct per process so two concurrent writers cannot clobber each other's temp.
  let tmp = temp_sibling(path);
  std::fs::write(&tmp, bytes).map_err(|err| StoreError::Io(err.to_string()))?;
  std::fs::rename(&tmp, path).map_err(|err| StoreError::Io(err.to_string()))?;
  Ok(())
}

/// The sibling temp path used by [`atomic_write`]: the target with a `.tmp.<pid>` suffix appended, so it shares the
/// target's directory (one-filesystem rename) and is unique per process. Kept separate so the invariant is testable.
fn temp_sibling(path: &Path) -> PathBuf {
  let mut name = path.file_name().map(|n| n.to_os_string()).unwrap_or_default();
  name.push(format!(".tmp.{}", std::process::id()));
  match path.parent() {
    Some(parent) => parent.join(name),
    None => PathBuf::from(name),
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn config_dir_lands_under_an_app_named_directory() {
    // We cannot assert the exact base across CI platforms, but the resolved dir must be app-named. A platform with
    // no config base errors instead — also acceptable (the caller then runs on in-memory defaults).
    if let Ok(dir) = config_dir() {
      assert!(dir.to_string_lossy().contains(APP_NAME), "config dir must be app-named: {}", dir.display());
    }
  }

  #[test]
  fn atomic_write_creates_parents_and_lands_the_bytes() {
    let dir = std::env::temp_dir().join(format!("skirnir-store-write-{}", std::process::id()));
    let path = dir.join("nested").join("config.json"); // also exercises parent-dir creation.
    let _ = std::fs::remove_dir_all(&dir);
    atomic_write(&path, b"hello").expect("writing to a writable temp path succeeds");
    assert_eq!(std::fs::read_to_string(&path).expect("the file exists"), "hello");
    let _ = std::fs::remove_dir_all(&dir);
  }

  #[test]
  fn atomic_write_leaves_no_temp_sibling_behind() {
    // After a successful write the temp must be renamed away, not left littering the config dir.
    let dir = std::env::temp_dir().join(format!("skirnir-store-temp-{}", std::process::id()));
    let path = dir.join("config.json");
    let _ = std::fs::remove_dir_all(&dir);
    atomic_write(&path, b"{}").expect("write succeeds");
    assert!(path.exists(), "the target must exist after the write");
    assert!(!temp_sibling(&path).exists(), "the temp sibling must be renamed away, not left behind");
    let _ = std::fs::remove_dir_all(&dir);
  }

  #[test]
  fn the_temp_sibling_shares_the_targets_parent_so_the_rename_stays_on_one_filesystem() {
    // The atomic-write temp must live in the target's parent directory; otherwise the rename could cross a
    // filesystem boundary and fail. This pins the sibling-temp invariant independent of any real I/O.
    let path = Path::new("/some/dir/config.json");
    let tmp = temp_sibling(path);
    assert_eq!(tmp.parent(), path.parent(), "the temp must be a sibling of the target");
    assert!(
      tmp.file_name().is_some_and(|n| n.to_string_lossy().starts_with("config.json.tmp.")),
      "the temp keeps the target name plus a per-process suffix: {}",
      tmp.display(),
    );
  }

  #[test]
  fn atomic_write_overwrites_an_existing_file() {
    // A second write must replace the prior content (the common save-again case), not append or fail.
    let dir = std::env::temp_dir().join(format!("skirnir-store-overwrite-{}", std::process::id()));
    let path = dir.join("config.json");
    let _ = std::fs::remove_dir_all(&dir);
    atomic_write(&path, b"first").expect("first write");
    atomic_write(&path, b"second").expect("second write");
    assert_eq!(std::fs::read_to_string(&path).expect("readable"), "second", "the second write must replace the first");
    let _ = std::fs::remove_dir_all(&dir);
  }
}
