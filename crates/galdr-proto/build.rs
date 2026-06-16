//! Build script: generate Rust types from `proto/settings.proto` via micropb-gen (which shells out to
//! `protoc`). The output is written to `$OUT_DIR/settings.rs` and `include!`d from `src/lib.rs`. The
//! schema is scalar-only, so no per-field container configuration is needed; `use_container_heapless`
//! is set regardless so any future `string`/`bytes`/`repeated` field is backed by heapless (no alloc).

use std::path::PathBuf;

fn main() {
  let out = PathBuf::from(std::env::var("OUT_DIR").expect("OUT_DIR is always set by cargo")).join("settings.rs");

  let mut generator = micropb_gen::Generator::new();
  // Back any variable-length fields with heapless containers (no alloc); harmless for the current
  // scalar-only schema, future-proof if a `string`/`bytes`/`repeated` field is added.
  generator.use_container_heapless();

  generator
    .compile_protos(&["proto/settings.proto"], out)
    .expect("micropb-gen failed to compile proto/settings.proto (is `protoc` installed and on PATH?)");

  println!("cargo:rerun-if-changed=proto/settings.proto");
}
