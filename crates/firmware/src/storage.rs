//! Record persistence backend (DOC-04 / Phase B): the esp-hal wiring that implements `firmware_core`'s
//! [`RecordStore`] over the ESP32-S3 internal flash, for BOTH the settings record and the coordinate record.
//!
//! This is the thin, non-host-testable adapter for the persisted records, mirroring how [`crate::tmc`] adapts
//! the TMC bus. ALL framing, protobuf encode/decode, versioning, and CRC live in `firmware_core` (the records'
//! `wire` modules); here we only move opaque framed bytes to and from a dedicated NVS flash region. One
//! [`FlashRecordStore`] impl serves both records — they differ only by the [`NvsKey`] each instance targets.
//!
//! ## Storage stack
//! [`esp_storage::FlashStorage`] provides the raw, BLOCKING `embedded-storage` NorFlash access. Each record is
//! stored as a key/value entry (keyed by its [`NvsKey`]) in a wear-leveled [`sequential_storage`] map over the
//! NVS partition (`NVS_OFFSET..NVS_OFFSET+NVS_SIZE`, matching `partitions.csv`). `sequential-storage` is
//! async-only, so the blocking flash is wrapped in [`BlockingAsync`]; the [`RecordStore`] methods are now
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

use firmware_core::coords::wire::FRAME_MAX_LEN as COORD_FRAME_MAX_LEN;
use firmware_core::hal_traits::{RecordStore, StoreError};
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

/// The number of distinct keys the cache tracks pointers for. Two records are stored — the framed settings
/// blob under [`NvsKey::Settings`] and the framed coordinate blob under [`NvsKey::Coordinates`] — so two key
/// slots cache both locations with a guaranteed hit.
const NVS_KEY_SLOTS: usize = 2;

/// The persistent pointer cache type for the settings map: tracks the page states of all [`NVS_PAGE_COUNT`]
/// sectors and the location of the single [`NvsKey`], so a load/save skips the full-region scan a fresh
/// [`NoCache`](sequential_storage::cache::NoCache) would force.
type SettingsCache = KeyPointerCache<NVS_PAGE_COUNT, u8, NVS_KEY_SLOTS>;

/// The `sequential-storage` map keys under which the framed records are stored. Modelled as a `#[repr(u8)]`
/// namespace so each record gets a distinct discriminant; retired keys must never be reused. Each
/// [`FlashRecordStore`] targets exactly one of these, so one store impl serves both records.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum NvsKey {
  /// The framed settings record (`firmware_core::settings::wire`).
  Settings = 0,
  /// The framed coordinate record — G54-G59 / G28 / G30 (`firmware_core::coords::wire`). A SEPARATE key from
  /// the settings record because coordinate data has its own lifecycle (`$RST=#` clears it independently) and
  /// is written far more often (every G10/G28.1).
  Coordinates = 1,
}

impl NvsKey {
  /// The `sequential-storage` byte key under which this record is stored.
  fn as_byte(self) -> u8 {
    self as u8
  }
}

/// Scratch buffer length for `sequential-storage` operations: must hold the key plus the longest serialized
/// value (a full settings frame, which is larger than a coordinate frame), with headroom for the library's
/// word-alignment rounding.
const DATA_BUF_LEN: usize = FRAME_MAX_LEN + 64;

// The settings frame is the larger of the two records, so the shared scratch buffer sized to it also fits a
// coordinate frame; assert that invariant so a future schema change cannot silently undersize the buffer.
const _: () = assert!(COORD_FRAME_MAX_LEN <= FRAME_MAX_LEN, "coordinate frame must fit the settings scratch buffer");

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

/// The on-target [`RecordStore`]: reads and writes ONE framed record (selected by its [`NvsKey`]) in the NVS map.
/// One store impl serves both the settings record and the coordinate record — they differ only by the `key`
/// field, so the flash plumbing (the persistent-cache `mem::replace`/`destroy` recovery dance, the
/// [`BlockingAsync`] wrapping, the scratch sizing, and the error mapping) lives exactly once here. The settings
/// and coordinate data remain independent records under distinct keys, so each can be written and cleared
/// without disturbing the other. Holds only a borrow of the [`SharedFlash`]; all record knowledge stays in
/// firmware-core.
pub struct FlashRecordStore {
  flash: &'static SharedFlash,
  key: NvsKey,
}

impl FlashRecordStore {
  /// Build a store over the shared flash instance targeting the framed settings record ([`NvsKey::Settings`]).
  pub fn settings(flash: &'static SharedFlash) -> Self {
    FlashRecordStore { flash, key: NvsKey::Settings }
  }

  /// Build a store over the shared flash instance targeting the framed coordinate record
  /// ([`NvsKey::Coordinates`]).
  pub fn coordinates(flash: &'static SharedFlash) -> Self {
    FlashRecordStore { flash, key: NvsKey::Coordinates }
  }
}

impl RecordStore for FlashRecordStore {
  /// Fetch this store's framed record into `buf`. Returns [`StoreError::NotFound`] if nothing is stored yet,
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
    let result = match store.fetch_item::<&[u8]>(&mut scratch, &self.key.as_byte()).await {
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
    // Recover the cache so the populated cache is stored back for the next call; the `_` discards the flash
    // wrapper (it reborrows `guard.flash`) so the borrow ends before `guard.cache` is reassigned.
    let (_, recovered) = store.destroy();
    guard.cache = recovered;
    result
  }

  /// Persist `frame` (a complete framed record) under this store's key, replacing any prior record.
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
      .store_item::<&[u8]>(&mut scratch, &self.key.as_byte(), &value)
      .await
      .map_err(|_| StoreError::Io);
    // Recover the (now-updated) cache so the next call reuses it rather than re-scanning the region; the `_`
    // discards the flash wrapper (it reborrows `guard.flash`) so the borrow ends before `guard.cache` is set.
    let (_, recovered) = store.destroy();
    guard.cache = recovered;
    result
  }
}
