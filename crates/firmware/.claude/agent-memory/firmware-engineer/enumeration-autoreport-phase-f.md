---
name: enumeration-autoreport-phase-f
description: Phase F (final) — $481 auto-status-report task + 0x8C toggle, and $ES/$EG/$EE/$EA/$SED runtime enumeration driven off SETTING_DESCRIPTORS + code enums
metadata:
  type: project
---

Phase F (final phase) of the grblHAL streaming work: periodic auto-status-reporting (`$481`/`0x8C`) and runtime
enumeration commands (`$ES`/`$EG`/`$EE`/`$EA`/`$SED`). Builds on A-E. Landed on
`firmware/core-pipeline-and-streaming`.

**Auto-report `$481` (`firmware-core/settings.rs`):** new `Settings.auto_report_interval_ms: u32` (0=disabled,
else clamped to `[100,1000]`). `Field::AutoReportIntervalMs`, descriptor `{ number: 481 }`, `parse_auto_report_
interval` (rejects non-zero outside 100..=1000), proto field 56 (additive, no schema-version bump). `clamp_auto_
report_interval` (0 stays 0, else clamp to MIN/MAX) is shared by setter + sanitizer + `Settings::auto_report_
interval_ms()` accessor. `AUTO_REPORT_INTERVAL_MIN_MS=100`/`MAX_MS=1000` pub consts.

**Auto-report task (`firmware/comms.rs`):** `auto_report_task()` reuses `STATUS_REQUEST.signal(())` so the
EXISTING `status_responder` builds the report — exactly ONE formatter, ONE USB writer, so auto-report == `?`
report byte-for-byte, never a partial/interleaved line and never an `ok`. Live mirrors `AUTO_REPORT_INTERVAL_MS:
AtomicU32` + `AUTO_REPORT_SUSPENDED: AtomicBool` (Relaxed). Task: when disabled (interval 0 OR suspended) parks
on `AUTO_REPORT_WAKE` Signal; when enabled `select(Timer::after(period), AUTO_REPORT_WAKE.wait())`. Firmware-side
floor `AUTO_REPORT_FLOOR_MS=50` (half the grblHAL floor) in `effective_auto_report_interval()` — defense-in-depth
so the task can NEVER starve `usb_tx`. `0x8C` (`ToggleAutoReport`) in `dispatch_realtime`: `AUTO_REPORT_SUSPENDED.
fetch_xor(true)`, on resume signals `AUTO_REPORT_WAKE`. Live retune (no reboot): `$481=` write + `$PBX` import +
soft reset all re-seed `AUTO_REPORT_INTERVAL_MS` via `init_auto_report(...)` and signal `AUTO_REPORT_WAKE`. Soft
reset also clears the suspend. Seeded at boot in main.rs (`comms::init_auto_report(auto_report_interval)` captured
before `settings` is moved into the cell). Task spawned in main.rs alongside `status_responder`.

**Enumeration metadata (single source `SETTING_DESCRIPTORS`):** each `SettingDescriptor` gained `meta:
SettingMeta { group, name, unit, datatype, format, min, max }`. `SettingDatatype` enum → grblHAL codes (Bitfield
0/Bool 3/Integer 5/Float 6/AxisMask 7). `SETTING_GROUPS: &[SettingGroup{id,parent,name}]` table + named `GROUP_*`
consts (General 1/Limits 3/Control 4/Probing 5/Homing 6/Stepper 8/Spindle 9/Axis 11). GOTCHA: strictly-positive
float settings (arc tol, homing feed/seek, spindle max, steps/mm, max rate, accel, max travel — all use
`parse_f32_positive` which rejects 0) MUST have `min: ""` not `min: "0"`, else the
`every_setting_number_enumerates_with_min_max_matching_its_range` test fails (it writes the enumerated min back
and expects acceptance). Non-negative floats (junction dev, homing pulloff, spindle min) keep `min:"0"`.

**Enumeration formatters:** `Settings::write_setting_enumeration(n)` → `[SETTING:id|group|name|unit|datatype|
format|min|max]` (per-axis enumerates each of its 3 numbers). `write_setting_group(index)` (pub free fn) →
`[SETTINGGROUP:id|parent|name]`, `SETTING_GROUP_COUNT` pub. `Settings::write_setting_description(n)` →
`[SETTINGDESCR:n|name (unit)]`. In `protocol.rs`: `ErrorCode{id,name,description}` + `ERROR_CODES: &[ErrorCode]`
(single authority for every `error:N` the fw emits: 1,2,3,5,9,15,20,22,23), `ResponseWriter::error_code_line` →
`[ERRORCODE:..]`. `AlarmCode::ALL` + `name()`/`description()`, `ResponseWriter::alarm_code_line` →
`[ALARMCODE:..]`. All emitted LINE-BY-LINE through the single response writer with blocking `send` (back-pressure
over the 1024-byte pipe), never one giant buffer; each fits `RESPONSE_CAPACITY` (160).

**`$E*` dispatch:** `SystemCommand` enum gained `EnumSettings`/`EnumSettingGroups`/`EnumErrorCodes`/
`EnumAlarmCodes`/`SettingDescription{id:u16}`. `classify` exact-matches `ES`/`EG`/`EE`/`EA`; `$SED=<n>` parsed in
`classify_with_payload` (non-numeric id → Unknown). Unknown `$E*` (e.g. `$EX`) → Unknown → `error:3` (preserves
Phase-A hardening). comms.rs `handle_system_command` arms call `enumerate_settings/_setting_groups/_error_codes/
_alarm_codes` (drive off `SETTING_NUMBERS`/`SETTING_GROUP_COUNT`/`ERROR_CODES`/`AlarmCode::ALL`) + `send_setting_
description`, each then `ack()`. `$SED` for an unknown id emits no line but still `ok`s (valid command, empty
result).

**NEWOPT:** `[NEWOPT:RT+]` → `[NEWOPT:ENUMS,RT+,SED]` in `ResponseWriter::build_info`. Pre-existing test
`build_info_extended_adds_grblhal_lines` updated.

**$SED scope:** Phase F has no long prose per-setting description; `[SETTINGDESCR:]` reuses name+unit as the
short description (documented in the fn). That's the only stub — everything else is fully implemented.

Verified: `cargo test -p firmware-core` 348 green (was 335; +13 Phase F), lib clippy clean (only pre-existing
test-only `field_reassign_with_default` warnings, I removed one net), Xtensa firmware build clean (genuine
Tensilica Xtensa ELF, no new warnings). See [[overrides-phase-e]], [[streaming-state-sharing]],
[[coordinate-model-phase-b]], [[build-test-commands]].
