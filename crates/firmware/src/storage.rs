//! Settings persistence backend (DOC-04): the esp-hal wiring that implements `firmware_core`'s
//! [`SettingsStore`] over the ESP32-S3 internal flash.
//!
//! This is the thin, non-host-testable adapter for the settings subsystem, mirroring how [`crate::tmc`]
//! adapts the TMC bus. ALL framing, protobuf encode/decode, versioning, and CRC live in
//! [`firmware_core::settings`]; here we only move opaque framed bytes to and from a dedicated NVS flash
//! region.
//!
//! ## Storage stack
//! [`esp_storage::FlashStorage`] provides the raw, BLOCKING `embedded-storage` NorFlash access. The settings
//! record is stored as a single key/value entry in a wear-leveled [`sequential_storage`] map over the NVS
//! partition (`NVS_OFFSET..NVS_OFFSET+NVS_SIZE`, matching `partitions.csv`). `sequential-storage` is
//! async-only, so the blocking flash is wrapped in [`BlockingAsync`]; the [`SettingsStore`] methods are now
//! `async`, so they `.await` the lock and the `sequential-storage` futures directly — no `block_on`, no risk
//! of a same-executor deadlock from forcing an async transport to complete synchronously.
//!
//! ## Persistent pointer cache (avoids a full NVS scan per call)
//! `sequential-storage` can use a cache so a load/save does not re-scan the whole NVS region from scratch each
//! time. A throw-away [`NoCache`] rebuilt per call would force exactly that full-region scan on every `$x=val`
//! persist. Instead a single [`KeyPointerCache`] lives FOR THE PROGRAM beside the flash inside [`FlashState`],
//! behind the same mutex, so its page-state and key-pointer knowledge persists across operations. Because
//! [`MapStorage::new`] takes the cache by value, each call MOVES the cache into a temporary `MapStorage`, runs
//! the op, then [`MapStorage::destroy`] recovers `(flash, cache)` so both are stored back for the next call —
//! the cache is never dropped, only borrowed through the map for the duration of one operation.
//!
//! ## Flash access is serialized
//! [`esp_storage::FlashStorage::new`] panics if constructed twice, and flash erase/write must not race, so a
//! single [`FlashState`] (flash + its persistent cache) lives behind a [`SharedFlash`] `Mutex` shared as
//! `&'static`; the boot loader and the runtime `$x=val`/`$PBX` persist both go through it.

use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use esp_storage::FlashStorage;
use sequential_storage::cache::KeyPointerCache;
use sequential_storage::map::{MapConfig, MapStorage};

use firmware_core::hal_traits::{SettingsStore, StoreError};
use firmware_core::settings::wire::FRAME_MAX_LEN;

/// Start of the dedicated settings NVS region in flash. MUST match the `storage` partition offset in
/// `partitions.csv` and be flash-sector (4 KiB) aligned. Sits above the application image.
pub const NVS_OFFSET: u32 = 0x0032_0000;
/// Length of the settings NVS region. MUST match the `storage` partition size in `partitions.csv` and be a
/// multiple of the 4 KiB sector size (sequential-storage needs at least two sectors).
pub const NVS_SIZE: u32 = 0x0001_0000;

/// Flash sector (erase-unit / page) size in bytes on the ESP32-S3 internal flash, 4 KiB. Used to size the
/// pointer cache's per-page tracking to cover the whole NVS region.
const NVS_SECTOR_SIZE: u32 = 0x1000;

/// The number of `sequential-storage` pages (flash sectors) the NVS region spans: `NVS_SIZE / NVS_SECTOR_SIZE`
/// = `0x10000 / 0x1000` = 16. Sizes the [`KeyPointerCache`] page-state arrays so the cache can track the page
/// state of every sector in the region.
const NVS_PAGE_COUNT: usize = (NVS_SIZE / NVS_SECTOR_SIZE) as usize;

/// The number of distinct keys the cache tracks pointers for. Exactly one record is stored (the framed
/// settings blob under [`NvsKey::Settings`]), so one key slot caches its location with a guaranteed hit.
const NVS_KEY_SLOTS: usize = 1;

/// The persistent pointer cache type for the settings map: tracks the page states of all [`NVS_PAGE_COUNT`]
/// sectors and the location of the single [`NvsKey`], so a load/save skips the full-region scan a fresh
/// [`NoCache`](sequential_storage::cache::NoCache) would force.
type SettingsCache = KeyPointerCache<NVS_PAGE_COUNT, u8, NVS_KEY_SLOTS>;

/// The `sequential-storage` map key under which the single framed settings record is stored. Modelled as a
/// `#[repr(u8)]` namespace so future records get distinct discriminants; retired keys must never be reused.
#[repr(u8)]
enum NvsKey {
  /// The framed settings record (`firmware_core::settings::wire`).
  Settings = 0,
}

/// Scratch buffer length for `sequential-storage` operations: must hold the key plus the longest serialized
/// value (a full settings frame), with headroom for the library's word-alignment rounding.
const DATA_BUF_LEN: usize = FRAME_MAX_LEN + 64;

/// The flash instance plus its persistent pointer cache, co-located so the cache outlives every individual
/// operation and survives between calls (see module docs). Both live behind the [`SharedFlash`] mutex.
pub struct FlashState {
  /// The single blocking NorFlash instance. `esp_storage::FlashStorage::new` panics if constructed twice, so
  /// exactly one exists, created once in `main`.
  flash: FlashStorage<'static>,
  /// The persistent `sequential-storage` pointer cache. Moved through a temporary `MapStorage` for each op and
  /// recovered via `destroy`, so its accumulated page/key knowledge is reused rather than rebuilt per call.
  cache: SettingsCache,
}

impl FlashState {
  /// Build the flash state from the single flash instance, with a fresh (empty) pointer cache. An empty cache
  /// is correct for any flash contents — `sequential-storage` populates it from the medium on first use.
  pub fn new(flash: FlashStorage<'static>) -> Self {
    FlashState { flash, cache: SettingsCache::new() }
  }
}

/// The flash state behind a cross-core-safe mutex, shared as `&'static`. [`CriticalSectionRawMutex`] gates
/// both cores on the S3; flash operations are brief and infrequent (boot load, occasional coalesced persist).
pub type SharedFlash = Mutex<CriticalSectionRawMutex, FlashState>;

/// The flash range the settings map occupies, derived from the partition constants.
fn nvs_range() -> core::ops::Range<u32> {
  NVS_OFFSET..NVS_OFFSET + NVS_SIZE
}

/// The on-target [`SettingsStore`]: reads and writes the single framed settings record in the NVS map. Holds
/// only a borrow of the [`SharedFlash`]; all settings knowledge stays in firmware-core.
pub struct FlashSettingsStore {
  flash: &'static SharedFlash,
}

impl FlashSettingsStore {
  /// Build a store over the shared flash instance.
  pub fn new(flash: &'static SharedFlash) -> Self {
    FlashSettingsStore { flash }
  }
}

impl SettingsStore for FlashSettingsStore {
  /// Fetch the framed settings record into `buf`. Returns [`StoreError::NotFound`] if nothing is stored yet,
  /// [`StoreError::TooLarge`] if the record does not fit `buf`, and [`StoreError::Io`] on a flash fault.
  async fn load(&mut self, buf: &mut [u8]) -> Result<usize, StoreError> {
    let mut guard = self.flash.lock().await;
    // Split the borrow so the flash and the persistent cache can be passed to `MapStorage` independently. The
    // cache is moved in (the map owns it) and recovered via `destroy` so it survives for the next call.
    let FlashState { flash, cache } = &mut *guard;
    let async_flash = BlockingAsync::new(flash);
    let cache = core::mem::replace(cache, SettingsCache::new());
    let mut store = MapStorage::<u8, _, _>::new(async_flash, MapConfig::new(nvs_range()), cache);
    let mut scratch = [0u8; DATA_BUF_LEN];
    let result = match store.fetch_item::<&[u8]>(&mut scratch, &(NvsKey::Settings as u8)).await {
      Ok(Some(bytes)) => {
        if bytes.len() > buf.len() {
          Err(StoreError::TooLarge)
        } else {
          buf[..bytes.len()].copy_from_slice(bytes);
          Ok(bytes.len())
        }
      }
      Ok(None) => Err(StoreError::NotFound),
      Err(_) => Err(StoreError::Io),
    };
    // Recover the cache so the populated cache is stored back for the next call; drop the flash wrapper first
    // (it reborrows `guard.flash`) so the borrow ends before `guard.cache` is reassigned.
    let (async_flash, recovered) = store.destroy();
    drop(async_flash);
    guard.cache = recovered;
    result
  }

  /// Persist `frame` (a complete settings record) under the settings key, replacing any prior record.
  async fn save(&mut self, frame: &[u8]) -> Result<(), StoreError> {
    let mut guard = self.flash.lock().await;
    let FlashState { flash, cache } = &mut *guard;
    let async_flash = BlockingAsync::new(flash);
    let cache = core::mem::replace(cache, SettingsCache::new());
    let mut store = MapStorage::<u8, _, _>::new(async_flash, MapConfig::new(nvs_range()), cache);
    let mut scratch = [0u8; DATA_BUF_LEN];
    // `store_item` takes the value by reference; `&[u8]` is the `Value` impl, so the item is `&&[u8]`.
    let value: &[u8] = frame;
    let result = store
      .store_item::<&[u8]>(&mut scratch, &(NvsKey::Settings as u8), &value)
      .await
      .map_err(|_| StoreError::Io);
    // Recover the (now-updated) cache so the next call reuses it rather than re-scanning the region; drop the
    // flash wrapper first (it reborrows `guard.flash`) so the borrow ends before `guard.cache` is reassigned.
    let (async_flash, recovered) = store.destroy();
    drop(async_flash);
    guard.cache = recovered;
    result
  }
}
