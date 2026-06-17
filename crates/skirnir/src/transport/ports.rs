//! Pure, framework-agnostic port enumeration model: structured [`PortInfo`], Galdr classification, and the
//! macOS `cu.*`-callout preference / dedup.
//!
//! None of this references `tokio-serial` (or any I/O), so it is unit-testable on every platform without a
//! real port — the `serial` adapter only maps the live `SerialPortInfo` into these types and then calls
//! [`normalize_ports`]. Keeping the policy here is what lets the macOS dual-listing, the `tty`→`cu`
//! translation, the Linux passthrough, and the Espressif-VID ranking all be tested headlessly.
//!
//! Platform note (macOS): a USB serial device enumerates as BOTH a `/dev/cu.*` callout and a `/dev/tty.*`
//! dialin node. The callout is the one to open — the dialin can block on carrier (DCD). On Linux there is no
//! such split (`/dev/ttyACM0`, `/dev/ttyUSB0`), so the normalisation is a careful no-op there.

/// Espressif's USB vendor id. The ESP32-S3's native USB-Serial-JTAG bridge — which the Galdr board uses —
/// enumerates under this VID, so it is the strongest static (no-I/O) signal that a port is the board.
pub const GALDR_VID: u16 = 0x303A;

/// The USB-Serial-JTAG product id of the ESP32-S3 bridge. Recorded for labelling; classification keys on the
/// VID alone, since a custom USB descriptor could change the PID while staying Espressif.
pub const GALDR_JTAG_PID: u16 = 0x1001;

/// How a port relates to "is this the Galdr board?", derived purely from its (possibly absent) USB metadata.
/// Crucially [`PortClass::Unknown`] still means *listed* — a port is never hidden for lacking metadata, or the
/// board could vanish from the dropdown on a platform/permission combo that does not expose the VID.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortClass {
  /// The USB VID is Espressif's ([`GALDR_VID`]) — very likely the board. Ranked first and tagged in the UI.
  LikelyGaldr,
  /// The port advertises a USB VID that is not Espressif's — a real device, just not the board.
  Other,
  /// No USB VID is available (a non-USB port, or a platform/permission combo that hides the metadata). We do
  /// not know, so we list it plainly rather than guessing — never hidden.
  Unknown,
}

/// Classify a port from its optional USB vendor id. Pure and total: Espressif VID → [`PortClass::LikelyGaldr`],
/// any other VID → [`PortClass::Other`], absent VID → [`PortClass::Unknown`] (still listed).
pub fn classify(vid: Option<u16>) -> PortClass {
  match vid {
    Some(GALDR_VID) => PortClass::LikelyGaldr,
    Some(_) => PortClass::Other,
    None => PortClass::Unknown,
  }
}

/// A structured serial port entry for the UI dropdown: the (cu-preferred) device path plus whatever USB
/// metadata the platform exposed. Everything but `path` is optional — on a bare non-USB port, or where the OS
/// does not surface descriptors, only the path is known, and the port is still listed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PortInfo {
  /// The device path the UI shows and the transport opens, e.g. `/dev/cu.usbmodem31101` or `/dev/ttyACM0`.
  pub path: String,
  /// USB vendor id, when the port is a USB device and the OS exposed it.
  pub vid: Option<u16>,
  /// USB product id, when available.
  pub pid: Option<u16>,
  /// USB product string (e.g. `"USB JTAG/serial debug unit"`), when available.
  pub product: Option<String>,
  /// USB manufacturer string (e.g. `"Espressif"`), when available.
  pub manufacturer: Option<String>,
  /// USB serial-number string, when available.
  pub serial: Option<String>,
}

impl PortInfo {
  /// A path-only port with no USB metadata. Handy for the Linux passthrough case and for tests.
  pub fn bare(path: impl Into<String>) -> Self {
    PortInfo { path: path.into(), vid: None, pid: None, product: None, manufacturer: None, serial: None }
  }

  /// This port's [`PortClass`], derived from its VID.
  pub fn class(&self) -> PortClass {
    classify(self.vid)
  }

  /// Whether this port is very likely the Galdr board (Espressif VID). The UI ranks and tags these.
  pub fn is_likely_galdr(&self) -> bool {
    self.class() == PortClass::LikelyGaldr
  }

  /// A short human hint for the dropdown row, or `None` when there is nothing useful to add beyond the path.
  /// Prefers the «likely Galdr» tag, then the USB product string, so the user lands on the board by default.
  pub fn hint(&self) -> Option<String> {
    if self.is_likely_galdr() {
      // Fold the product in when present so the tag is informative, not just a bare label.
      return Some(match &self.product {
        Some(product) => format!("likely Galdr — {product}"),
        None => "likely Galdr".to_string(),
      });
    }
    self.product.clone()
  }
}

/// On macOS, the callout sibling of a `/dev/tty.*` dialin path: `/dev/tty.X` → `/dev/cu.X`. Any other path
/// (already a `cu.*`, or a Linux `/dev/ttyACM0`/`/dev/ttyUSB0`) is returned unchanged. This is intentionally a
/// string-suffix rule on the leaf `tty.`/`cu.` prefix — Linux device nodes (`ttyACM0`) have no `.` after
/// `tty`, so they never match and are passed through verbatim.
pub fn prefer_cu(path: &str) -> String {
  match dialin_key(path) {
    Some((dir, key)) => format!("{dir}cu.{key}"),
    None => path.to_string(),
  }
}

/// If `path` is a macOS dialin node (`<dir>/tty.<key>`), return `(<dir>/, <key>)` so a callout sibling can be
/// built and dedup can group the pair. Returns `None` for callouts and for Linux nodes (no `.` after `tty`).
fn dialin_key(path: &str) -> Option<(&str, &str)> {
  // Split off the leaf so we only ever rewrite the device name, never a directory that happens to contain
  // "tty." earlier in the path.
  let slash = path.rfind('/').map(|i| i + 1).unwrap_or(0);
  let (dir, leaf) = path.split_at(slash);
  leaf.strip_prefix("tty.").map(|key| (dir, key))
}

/// The dedup/ranking key for a port: the callout-normalised path. Two listings of the same physical device
/// (the macOS `cu.*`/`tty.*` pair) collapse to one key, so dedup keeps a single entry.
fn dedup_key(path: &str) -> String {
  prefer_cu(path)
}

/// Normalise a raw enumeration into the list the UI should show. The transformation is pure:
///
/// 1. **Callout preference / dedup (macOS):** for each physical device, keep one entry under its `cu.*`
///    callout path. When both the `cu.*` and `tty.*` siblings are present, the `cu.*` listing's metadata is
///    kept and the `tty.*` one is dropped; a lone `tty.*` is rewritten to its `cu.*` path. Linux nodes carry
///    no `cu/tty` split and pass through untouched.
/// 2. **Ranking:** Espressif-VID ports ([`PortClass::LikelyGaldr`]) are floated to the front so the board is
///    the default selection, preserving the original relative order within each group (stable).
///
/// Nothing is ever hidden — a port with no USB metadata is still listed (just not ranked first).
pub fn normalize_ports(ports: Vec<PortInfo>) -> Vec<PortInfo> {
  let mut deduped: Vec<PortInfo> = Vec::with_capacity(ports.len());
  for mut port in ports {
    let key = dedup_key(&port.path);
    // Rewrite the path to the callout form up front so both a lone `tty.*` and the surviving entry of a pair
    // present the `cu.*` path the UI opens.
    port.path = key.clone();
    // Every stored entry already had its path rewritten to the canonical key on insertion, so a direct path
    // compare is equivalent to re-deriving the key and avoids the redundant per-entry recompute.
    match deduped.iter_mut().find(|existing| existing.path == key) {
      // A sibling already won the slot. Prefer to keep whichever carries USB metadata so a `cu.*`-first or
      // `tty.*`-first enumeration order both end up with the populated descriptors.
      Some(existing) => {
        if existing.vid.is_none() && port.vid.is_some() {
          *existing = port;
        }
      }
      None => deduped.push(port),
    }
  }

  // Stable partition: likely-Galdr ports first, everything else after, each preserving discovery order.
  deduped.sort_by_key(|port| !port.is_likely_galdr());
  deduped
}

#[cfg(test)]
mod tests {
  use super::*;

  fn galdr(path: &str) -> PortInfo {
    PortInfo {
      path: path.to_string(),
      vid: Some(GALDR_VID),
      pid: Some(GALDR_JTAG_PID),
      product: Some("USB JTAG/serial debug unit".to_string()),
      manufacturer: Some("Espressif".to_string()),
      serial: Some("14:C1:9F:DB:8B:7C".to_string()),
    }
  }

  fn other(path: &str, vid: u16) -> PortInfo {
    PortInfo { vid: Some(vid), pid: Some(0x9A39), ..PortInfo::bare(path) }
  }

  #[test]
  fn classify_maps_espressif_vid_to_likely_galdr() {
    assert_eq!(classify(Some(GALDR_VID)), PortClass::LikelyGaldr);
  }

  #[test]
  fn classify_maps_other_vid_to_other() {
    assert_eq!(classify(Some(0x043E)), PortClass::Other);
  }

  #[test]
  fn classify_maps_missing_vid_to_unknown_not_hidden() {
    assert_eq!(classify(None), PortClass::Unknown);
  }

  #[test]
  fn prefer_cu_translates_macos_dialin_to_callout() {
    assert_eq!(prefer_cu("/dev/tty.usbmodem31101"), "/dev/cu.usbmodem31101");
  }

  #[test]
  fn prefer_cu_leaves_callout_unchanged() {
    assert_eq!(prefer_cu("/dev/cu.usbmodem31101"), "/dev/cu.usbmodem31101");
  }

  #[test]
  fn prefer_cu_passes_linux_nodes_through_untouched() {
    // Linux device nodes have no `.` after `tty`, so they must not be mangled.
    assert_eq!(prefer_cu("/dev/ttyACM0"), "/dev/ttyACM0");
    assert_eq!(prefer_cu("/dev/ttyUSB0"), "/dev/ttyUSB0");
  }

  #[test]
  fn macos_dual_listing_keeps_cu_and_drops_tty() {
    // Both siblings present (the real enumeration on this Mac); exactly one entry survives, at the cu.* path.
    let ports = vec![galdr("/dev/cu.usbmodem31101"), galdr("/dev/tty.usbmodem31101")];
    let out = normalize_ports(ports);
    assert_eq!(out.len(), 1);
    assert_eq!(out[0].path, "/dev/cu.usbmodem31101");
  }

  #[test]
  fn macos_dual_listing_keeps_metadata_when_tty_enumerates_first() {
    // If the metadata-bearing listing arrives second, dedup must still retain its descriptors.
    let ports = vec![PortInfo::bare("/dev/tty.usbmodem31101"), galdr("/dev/cu.usbmodem31101")];
    let out = normalize_ports(ports);
    assert_eq!(out.len(), 1);
    assert_eq!(out[0].path, "/dev/cu.usbmodem31101");
    assert_eq!(out[0].vid, Some(GALDR_VID));
  }

  #[test]
  fn lone_tty_is_rewritten_to_cu() {
    let out = normalize_ports(vec![galdr("/dev/tty.usbmodem31101")]);
    assert_eq!(out.len(), 1);
    assert_eq!(out[0].path, "/dev/cu.usbmodem31101");
  }

  #[test]
  fn linux_nodes_pass_through_and_are_not_deduped_together() {
    // Two distinct Linux devices must both survive — they share no cu/tty key.
    let ports = vec![PortInfo::bare("/dev/ttyACM0"), PortInfo::bare("/dev/ttyUSB0")];
    let out = normalize_ports(ports);
    assert_eq!(out.len(), 2);
    assert_eq!(out[0].path, "/dev/ttyACM0");
    assert_eq!(out[1].path, "/dev/ttyUSB0");
  }

  #[test]
  fn espressif_ports_are_ranked_first() {
    // A non-Galdr device enumerated before the board must still land after it, and the board is not hidden.
    let ports = vec![other("/dev/cu.usbmodem-lg", 0x043E), galdr("/dev/cu.usbmodem31101")];
    let out = normalize_ports(ports);
    assert_eq!(out[0].path, "/dev/cu.usbmodem31101");
    assert!(out[0].is_likely_galdr());
    assert_eq!(out[1].path, "/dev/cu.usbmodem-lg");
    assert!(!out[1].is_likely_galdr());
  }

  #[test]
  fn ranking_is_stable_within_groups() {
    // Within the non-Galdr group the original discovery order is preserved.
    let ports = vec![PortInfo::bare("/dev/ttyUSB1"), PortInfo::bare("/dev/ttyUSB0")];
    let out = normalize_ports(ports);
    assert_eq!(out[0].path, "/dev/ttyUSB1");
    assert_eq!(out[1].path, "/dev/ttyUSB0");
  }

  #[test]
  fn unknown_metadata_port_is_listed_not_hidden() {
    let out = normalize_ports(vec![PortInfo::bare("/dev/ttyACM0")]);
    assert_eq!(out.len(), 1);
    assert_eq!(out[0].class(), PortClass::Unknown);
  }

  #[test]
  fn hint_tags_likely_galdr_with_product() {
    assert_eq!(galdr("/dev/cu.x").hint().as_deref(), Some("likely Galdr — USB JTAG/serial debug unit"));
  }

  #[test]
  fn hint_falls_back_to_product_for_non_galdr() {
    let mut p = other("/dev/cu.lg", 0x043E);
    p.product = Some("LG Monitor Controls".to_string());
    assert_eq!(p.hint().as_deref(), Some("LG Monitor Controls"));
  }
}
