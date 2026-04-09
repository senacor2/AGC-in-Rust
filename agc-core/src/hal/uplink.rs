//! Uplink receiver sub-trait.
//!
//! AGC source: UPLINK_STORAGE.agc — ground uplink word storage.

/// Ground uplink interface.
///
/// The ground can uplink commands and data to the AGC via a 2-bit serial
/// channel. Each uplink word is 15 bits.
///
/// AGC source: ERASABLE_ASSIGNMENTS.agc — INBFR uplink buffer.
pub trait Uplink {
    /// True if an uplink word is available to be read.
    fn word_available(&self) -> bool;

    /// Read the next pending uplink word (15-bit value in u16).
    /// Returns `None` if no word is available.
    fn read_word(&mut self) -> Option<u16>;

    /// Return the number of words currently buffered.
    fn buffered_count(&self) -> u8;
}
