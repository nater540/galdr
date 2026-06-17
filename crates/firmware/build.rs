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
}
