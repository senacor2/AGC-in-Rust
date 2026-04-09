//! Downlink telemetry sub-trait.
//!
//! AGC source: DOWNLINK_LISTS.agc — telemetry word output.

/// Downlink telemetry output interface.
///
/// The AGC sends telemetry to the ground in 14-bit word pairs over a PCM
/// downlink. The T4RUPT handler writes one word pair per 20 ms frame.
///
/// AGC source: DOWNLINK_LISTS.agc — DOWNLIST table, DNTMGOTO dispatch.
pub trait Telemetry {
    /// True if the downlink transmitter is ready for the next word pair.
    fn ready(&self) -> bool;

    /// Write a 14-bit downlink word to the transmit buffer.
    fn write_word(&mut self, word: u16);
}
