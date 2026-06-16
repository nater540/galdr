//! grblHAL streaming protocol (DOC-08, `docs/gcode-streaming.md`).
//!
//! Owns the firmware-side contract shared with `skirnir`: exactly one `ok`/`error:N` per consumed
//! line, CRLF/LFCR treated as a single terminator (no double-ok), real-time single-byte command
//! interception (`?`/`!`/`~`/`0x18`), status reports, the welcome banner, and `$`-settings.
//!
//! Status: not yet implemented.
// TODO(DOC-08): implement line framing, real-time byte dispatch, status report formatting, banner.
