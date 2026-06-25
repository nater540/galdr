//! Firmware build script.
//!
//! Its ONLY job is to pull in defmt's linker script (`defmt.x`) for a logging build. defmt stores its
//! interned log strings in a non-allocated `.defmt` section that `defmt.x` defines and KEEPs; without it the
//! ELF has no `.defmt` section and espflash's defmt decoder fails with
//! "defmt version found, but no `.defmt` section". `defmt.x` is emitted by the `defmt` crate's own build
//! script and is only on the link search path when `defmt` is a dependency, so the link arg is added ONLY for
//! the `defmt` feature — adding `-Tdefmt.x` unconditionally would break the default (defmt-free) build with a
//! "cannot find linker script" error. The base `-Tlinkall.x` stays in `.cargo/config.toml`; it applies to
//! every build, defmt or not.
fn main() {
  // `CARGO_FEATURE_<NAME>` is set by cargo when the feature is active; `DEFMT` is the upper-cased feature name.
  if std::env::var_os("CARGO_FEATURE_DEFMT").is_some() {
    println!("cargo:rustc-link-arg=-Tdefmt.x");
  }

  // Emit a per-build identity word for the crash-breadcrumb panic decoder (`crash.rs` `BUILD_ID`). The panic
  // handler stores a `.rodata` file-string POINTER into RTC_FAST; that pointer is only valid against the EXACT
  // flashed image. Stamping a build-unique id alongside it lets the boot decoder refuse to dereference a pointer
  // left by a DIFFERENT image (it reports the panic without the now-meaningless file string instead of reading
  // garbage). The low 32 bits of the build's wall-clock nanoseconds are unique-enough per build and need no git.
  // `rerun-if-env-changed` keeps incremental rebuilds stable unless cargo actually re-runs this script.
  let build_id: u32 = std::time::SystemTime::now()
    .duration_since(std::time::UNIX_EPOCH)
    .map(|d| d.as_nanos() as u32)
    .unwrap_or(0);
  println!("cargo:rustc-env=GALDR_BUILD_ID={build_id}");
}
