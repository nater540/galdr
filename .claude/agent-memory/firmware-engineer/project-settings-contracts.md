---
name: project-settings-contracts
description: DOC-04 settings subsystem — single descriptor-table authority, sanitize policy, saturating proto decode, async SettingsStore.
metadata:
  type: project
---

DOC-04 `$`-settings live in `crates/firmware-core/src/settings.rs` (host-tested) + `SettingsStore` trait in `hal_traits.rs`.

Single-authority design (post code-review fixes 2026-06-16): there is ONE `const SETTING_DESCRIPTORS: &[SettingDescriptor]`
(each row = base `$n` number + a `Field` enum variant). `SETTING_NUMBERS` (const-fn expansion over per-axis spans),
`Settings::set_command`, and `Settings::write_setting_line` are ALL derived from it via `lookup_descriptor(n)` →
`(desc, axis)`. `Field::axis_span()` is `const` (3 for X/Y/Z triples, else 1); `Field::parse_and_apply` and `Field::format`
hold the per-field parse/validate and `{:.3}`-float dump policy. Do NOT reintroduce parallel `$n` match tables — that drift
was the bug.

**Why:** three parallel tables had drifted and `set_command` skipped range checks (`$0=0` stored a zero-width step pulse).
**How to apply:** add a new `$n` setting by adding ONE descriptor row + one `Field` variant + arms in parse_and_apply/format.

Validation contract: `set_command` must reject out-of-range so the stored value already satisfies `sanitized()` (no
clamp-on-write). `$0` range is 1..=1000 (`parse_step_pulse_us`). Must-be-positive f32 fields use `parse_f32_positive` in
set_command AND `positive_or(v, default)` in `sanitized`: `max_rate_mm_min`, `max_travel_mm`, `steps_per_mm`, `accel_mm_s2`,
`arc_tolerance_mm`, `homing_feed/seek_mm_min`, `spindle_rpm_max`. Genuinely-zero-OK fields stay `.max(0.0)`:
`spindle_rpm_min`, `homing_pulloff_mm`. `sanitized` also clamps TMC 4-bit fields `tmc_send_delay`/`tmc_ihold_delay` to
0..=15 (else silently masked downstream). `from_proto` uses `saturating_u16`/`saturating_u8` (NOT `as`) so an out-of-range
wire value lands above any valid range and is then defaulted/clamped by sanitize (e.g. microsteps 65552 → default 16, not a
truncated "valid" 16). `SCHEMA_VERSION` stays 1 (proto3 additive compat; bump only for incompatible layout change).

`SettingsStore` is now async: `async fn load(&mut self, buf: &mut [u8]) -> Result<usize, StoreError>` and
`async fn save(&mut self, frame: &[u8]) -> Result<(), StoreError>` (`#[allow(async_fn_in_trait)]` on the trait — static
dispatch only, like TmcManager). `load_or_default`/`store_settings` are `async fn` that `.await` the store. Host tests drive
the immediately-ready mock futures with an inline `block_on` using `Waker::noop()` (no dev-dependency added). Gate is rustc
`#![deny(warnings)]` (clean); repo does NOT run clippy `-D warnings`, so the pre-existing `field_reassign_with_default` test
idiom is fine. See [[project-protocol-contracts]] for the streaming layer that calls set_command/write_setting_line.
