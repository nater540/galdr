---
name: project-storage-codec-unification
description: Post-refactor (2026-06-16) shapes of the unified storage-frame codec, generic RefreshReporter, single RecordStore, and StatusCfg cache
metadata:
  type: project
---

Four behavior-preserving cleanup refactors landed on `firmware/core-pipeline-and-streaming` (2026-06-16). These are the
NEW canonical shapes; prefer them over the duplicated forms the docs/older code implied.

**Shared storage-frame codec** — `firmware-core/src/storage_frame.rs` (registered `pub mod` in lib.rs) owns the one copy
of: `CodecError`, `FRAME_OVERHEAD = 11`, `crc32`, `push_byte`/`push_slice`, generic `frame(magic, version, payload_len,
out, encode_payload)`, `unframe(magic, version, &[u8]) -> Result<&[u8] payload, CodecError>`, and `frame_progress(magic,
version, frame_max_len, buf) -> FrameProgress`. `settings::wire` and `coords::wire` now just hold their MAGIC
(GdS1=0x47645331 / GdC1=0x47644331) + SCHEMA_VERSION=1 + protobuf type and call the shared primitives; they `pub use
storage_frame::{CodecError, FrameProgress}` so their public surface is unchanged. Wire bytes are byte-for-byte identical
(test `wire_frame_byte_layout_is_stable_after_codec_extraction` pins it).

**Generic RefreshReporter** — `protocol.rs` replaced `WcoReporter`/`OvReporter` with one `RefreshReporter<T: Copy +
PartialEq>` (`new(baseline)`, `reset(baseline)`, `should_include(current)`). Both are now `const fn`-constructible with a
baseline arg. `REFRESH_PERIOD = 10` is the authority; `WCO_REFRESH_PERIOD`/`OV_REFRESH_PERIOD` are aliases of it. comms.rs
holds `WCO_REPORTER: Cell<RefreshReporter<[f32; AXES]>>` (baseline `[0.0; AXES]`) and `OV_REPORTER:
Cell<RefreshReporter<Overrides>>` (baseline `Overrides::new()`). Array change-detect is the array's own element-wise
`PartialEq` (== last), not a per-axis loop.

**Single RecordStore trait** — `hal_traits.rs` merged `SettingsStore`+`CoordinateStore` into one `RecordStore`
(load/save, key-less). firmware-core loaders are `<S: RecordStore>`. firmware bin: `storage.rs` collapsed
`FlashSettingsStore`+`FlashCoordinateStore` into one `FlashRecordStore { flash, key: NvsKey }` with `::settings(flash)` /
`::coordinates(flash)` constructors; `NvsKey` is now `pub` with `as_byte()`. The persistent-cache mem::replace/destroy
dance lives once. Two distinct NVS keys (Settings=0, Coordinates=1) preserved → identical flash bytes.

**StatusCfg cache** — comms.rs `status_responder` no longer locks SETTINGS / copies the whole Settings per `?`. A `Copy`
`StatusCfg { steps_per_mm, position_report (from $10 mask), min_axis_max_rate (from $110-112) }` lives in
`STATUS_CFG: BlockingMutex<Cell<StatusCfg>>`. CRITICAL invariant: `refresh_status_cfg().await` MUST be called at every
SETTINGS writer. The four sites: boot seed `init_status_cfg(&settings)` in main.rs; `write_setting_command` ($x=val);
`handle_pb_write` ($PBX/bulk import complete); `handle_restore_settings` ($RST=$/*). A missed call = stale report
regression.
