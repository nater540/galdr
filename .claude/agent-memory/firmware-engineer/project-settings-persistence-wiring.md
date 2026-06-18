---
name: project-settings-persistence-wiring
description: Firmware-side wiring for async SettingsStore/TmcBus, persistent sequential-storage cache, and coalesced settings writes
metadata:
  type: project
---

DOC-04 persistence + DOC-03 TMC firmware wiring after the async-trait migration (storage.rs, tmc.rs, comms.rs, main.rs).

**Async traits.** `SettingsStore` (load/save) and `TmcBus` (write_reg/read_reg) are `async fn` traits (both `#[allow(async_fn_in_trait)]`), and `firmware_core::settings::{load_or_default, store_settings}` plus `TmcManager::{init_all,init_axis,set_current,read_status}` are all `async` — `.await` them.

**Why:** A `block_on` settings persist and a `delay_micros` TMC reply poll both stalled the single core-0 executor (up to 5 ms per absent TMC driver). Async lets the bus turn-around / flash op yield the executor.

**How to apply:**
- `Uart1TmcBus` (tmc.rs): non-blocking `read_buffered` poll loop; between empty polls await `Timer::after(Duration::from_micros(POLL_STEP_US))`, NOT a blocking delay. `drain_rx`/`write_all` stay sync (FIFO ops return immediately); `read_filling`/`consume_echo`/`read_reg`/`write_reg` are async. No `Delay` field. 5 ms total timeout preserved.
- `FlashSettingsStore` (storage.rs): no `block_on`; `.await` the `SharedFlash` mutex lock and `sequential_storage` calls directly.

**Persistent sequential-storage cache (avoids full NVS scan per call).** `SharedFlash = Mutex<CriticalSectionRawMutex, FlashState>` where `FlashState { flash: FlashStorage, cache: KeyPointerCache<16, u8, 1> }`. 16 pages = NVS_SIZE 0x10000 / 0x1000 sector; 1 key slot (single settings record). `MapStorage::new` takes the cache BY VALUE, so each op: `core::mem::replace(cache, KeyPointerCache::new())` to move the real cache into a temporary `MapStorage`, run the op, then `store.destroy()` returns `(flash_wrapper, cache)` — `drop` the flash wrapper (it reborrows `guard.flash`) BEFORE `guard.cache = recovered`. Cache is never dropped, only loaned through the map. `KeyPointerCache::new()` is `const`. There is NO blanket `&mut C: KeyCacheImpl` impl in seq-storage 7.2 — destroy/recover is the only way to persist the cache.

**Coalesced settings writes (comms.rs).** `$n=val`/`$PBX` apply to in-RAM `SETTINGS` mutex and set `SETTINGS_DIRTY: AtomicBool` (Release); they do NOT write flash inline and `ok` immediately. The single `comms_consumer` task owns the flush: `flush_settings()` does `SETTINGS_DIRTY.swap(false, AcqRel)` (clear-before-write so a change during the awaited flash op re-marks and is caught next time), snapshots `SETTINGS`, `store_settings(...).await`, logs failure via defmt without re-marking. Flush triggers: queue-empty burst boundary (`LINE_QUEUE.is_empty()` checked at loop top), `SETTINGS_FLUSH_SAFETY` (1 s) timer, and soft-reset (flush before reset_pipeline). Consumer loop uses `select3(LINE_QUEUE.receive(), SOFT_RESET.wait(), Timer::after(safety))`. `write_setting_command`/`handle_pb_write` no longer take `flash`; `flash` only threads to `comms_consumer` for the flush.

**Why coalesce:** a `$$` bulk restore = ~36 `$n=val` lines; per-line persist appended the WHOLE blob to the wear-leveled log ~36×, thrashing NVS. Now one flash append per burst.

**Build invocation (IMPORTANT).** `cargo build -p firmware` FROM THE REPO ROOT FAILS — the Xtensa target/toolchain are pinned in crate-scoped `.cargo/config.toml` + `rust-toolchain.toml`, which cargo/rustup only pick up when cwd is inside `crates/firmware`. Build with `source $HOME/export-esp.sh && cd crates/firmware && cargo build` (optionally `--features defmt`). From root it tries the host stable toolchain and panics in esp-hal's build.rs. See [[project-build-constraints]].
