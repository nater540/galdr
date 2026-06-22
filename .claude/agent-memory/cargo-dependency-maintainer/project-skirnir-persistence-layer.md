---
name: skirnir-persistence-layer
description: Vetted dependency choices for skirnir's config persistence layer (ron + directories), pinned versions and transitive-dep impact
metadata:
  type: project
---

Skirnir's persistence layer uses `ron = "0.12"` and `directories = "6"` (not `dirs`).

- **ron 0.12.1** (latest stable as of 2026-06): serde 1.0.181+ compatible, MIT/Apache-2.0. Default `std` feature
  only — do NOT enable `indexmap` (adds indexmap 2.x dep) or `integer128`/`unicode-segmentation`. Requires
  `serde/std` feature (enabled via ron's `std` default). New transitive deps vs the existing lock: `typeid = "1.0.1"`
  and `serde_derive` (already in lock at 1.0.228). `bitflags 2.x`, `once_cell 1.x`, `unicode-ident 1.x`, and
  `unicode-segmentation 1.x` are already in the lock and will unify cleanly.

- **directories = "6.0.0"** chosen over `dirs` because it provides `ProjectDirs::from()` with `config_dir()`,
  `data_dir()`, and `cache_dir()` in one struct — more ergonomic for app-level code. `dirs` is the low-level
  crate (single functions); `directories` is the mid-level wrapper. Both are maintained by the same author
  (Simon Ochsenreither) and both pull only `dirs-sys = "0.5.0"` as a transitive dep. MIT/Apache-2.0.
  Neither `directories` nor `dirs-sys` appear in the lock before adding — net-new additions.

- **serde**: workspace already has `serde = "1.0.228"` in the lock (single instance, no duplication risk).
  Adding `serde = { version = "1", features = ["derive"] }` to skirnir will reuse that locked instance.

**Why:** [[skirnir-ui-stack]]

**How to apply:** When adding config persistence deps to skirnir, use exactly these lines. Never add `indexmap` feature to ron.
