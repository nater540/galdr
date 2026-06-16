//! Hardware driver register codecs (DOC-03).
//!
//! Pure, allocation-free encode/decode logic for the peripheral drivers. The byte-level work lives
//! here so it is host-testable against datasheet vectors, independent of the UART transport trait
//! ([`crate::hal_traits`]) that carries the datagrams on target.

pub mod tmc2209;
