---
name: esp-hal-version-matrix
description: Definitive compatible version pins for the esp-hal firmware stack — includes esp-hal-embassy constraint analysis, esp-rtos path, and known incompatibilities
metadata:
  type: project
---

## Hard constraint from user (2026-06-16)

Firmware MUST use `esp-hal-embassy` as the Embassy runtime integration. Do NOT recommend
`esp-rtos`. See [[esp-hal-embassy-deprecation-status]] for context on why this creates
a tension with published crates.

## esp-hal-embassy status (as of 2026-06-16)

- **Officially deprecated** by the esp-rs team: functionality merged into `esp-rtos`.
- docs.rs marks 0.9.1 as failing to build (because crates.io resolves esp-hal to 1.0.0+).
- The `esp-hal-embassy/` directory no longer exists in the `esp-rs/esp-hal` main branch.
- Last published version: **0.9.1** (2025-10-14).

## Why esp-hal-embassy 0.9.1 is BROKEN with esp-hal 1.0.0 stable

esp-hal-embassy 0.9.1 `[features]` section:
```toml
executors = ["dep:embassy-executor", "esp-hal/__esp_hal_embassy"]
```

`esp-hal 1.0.0-rc.0` has `__esp_hal_embassy = []` (empty marker feature).
`esp-hal 1.0.0` stable **removed** `__esp_hal_embassy` — it was replaced by `requires-unstable = []`.

Result: `executors` feature activates `esp-hal/__esp_hal_embassy`, which does not exist in
esp-hal >=1.0.0. Cargo fails with "feature `__esp_hal_embassy` does not exist."

`InterruptExecutor` for Xtensa is ONLY provided by esp-hal-embassy's `executors` feature;
embassy-executor's own `InterruptExecutor` is Cortex-M only. So disabling `executors` is not viable.

## The only working published combination

`esp-hal = "=1.0.0-rc.0"` + `esp-hal-embassy = "0.9.1"` with `features = ["executors", "esp32s3"]`.

Must use `=1.0.0-rc.0` (exact pin, not caret) because:
- Caret `^1.0.0-rc.0` would resolve to 1.0.0 stable, which breaks the feature.
- `1.0.0-rc.0` is a pre-release; only use exact `=` pin.

## Verified Compatibility Matrix (firmware binary crate — esp-hal-embassy path)

| Crate | Pin | Features | Notes |
|---|---|---|---|
| esp-hal | =1.0.0-rc.0 | esp32s3, unstable | MUST be exact `=`; `unstable` required for interrupt module |
| esp-hal-embassy | 0.9.1 | executors, esp32s3 | Last published; requires __esp_hal_embassy which only rc.0 has |
| embassy-executor | 0.7.0 | executor-thread, noop-executor | Required by esp-hal-embassy 0.9.1 (^0.7.0) |
| embassy-time | 0.4.0 | — | Required by esp-hal-embassy 0.9.1 (^0.4.0); only 0.4.x release |
| embassy-sync | 0.6.2 | — | Required by esp-hal-embassy 0.9.1 (^0.6.2); pulls heapless ^0.8 |
| embassy-futures | 0.1.2 | — | No version coupling constraint |
| esp-backtrace | 0.17.0 | esp32s3, panic-handler, println | Pairs with esp-hal rc.0; requires heapless ^0.8, defmt ^1 |
| esp-println | 0.15.0 | esp32s3, uart | No direct esp-hal dep in 0.15.x |
| esp-storage | 0.7.0 | nor-flash | No esp-hal dep (uses esp-rom-sys); pairs with rc.0 stack |
| embedded-storage | 0.3 | — | No coupling constraint |
| embedded-io-async | 0.7.0 | — | One release; rc.0 unstable feature provides embedded-io-async-07 |
| heapless | 0.8.0 | — | Forced by embassy-sync 0.6.2 (^0.8); also the rc.0 stack uses ^0.8 |
| libm | 0.2 | — | No coupling constraint |
| defmt | 1.0.1 | — | Use ^1.0.1; embassy-sync 0.6.2 pulls defmt ^0.3 as optional; do NOT enable defmt on embassy-sync/embassy-time to avoid two-defmt conflict |
| static_cell | 2.1.1 | — | esp-hal-embassy 0.9.1 re-exports it; include for task storage |

## Defmt version warning

embassy-sync 0.6.2 and embassy-time 0.4.0 both require `defmt = "^0.3"` (optional).
esp-hal rc.0 and esp-hal-embassy 0.9.1 require `defmt = "^1.0.1"`.
**Never enable the `defmt` feature on `embassy-sync` or `embassy-time` in this stack.**
Only enable `defmt` on esp-hal, esp-hal-embassy, and esp-backtrace.

## esp-backtrace feature note

The `exception-handler` feature was REMOVED from esp-backtrace 0.17.x+ (already gone by 0.17.0).
Correct features: `["esp32s3", "panic-handler", "println"]`

## firmware-core lib pins

| Crate | Pin | Features | Notes |
|---|---|---|---|
| heapless | 0.8 | — | Match embassy-sync 0.6.2 requirement |
| libm | 0.2 | — | No constraint |
| defmt | 1.0.1 | (optional) | Feature-gate with cfg(feature="defmt") |

firmware-core dev dependencies:
| embedded-hal-mock | 0.11.1 | eh1 | Latest stable; eh1 feature for embedded-hal 1.x mocks |

## esp-rtos alternative (if user lifts esp-hal-embassy constraint)

See prior recommendation for esp-rtos 0.2.0 + esp-hal 1.0.0 stable path.
User has mandated esp-hal-embassy; do not recommend esp-rtos.

## Known duplicates in the dep graph (expected, not a bug)

With esp-hal rc.0 + esp-hal-embassy 0.9.1:
- Two defmt: 0.3.x (embassy-sync/embassy-time internal, disabled in practice) and 1.0.x (user-facing)
- These do NOT conflict as long as defmt feature is not enabled on embassy-sync/embassy-time.
