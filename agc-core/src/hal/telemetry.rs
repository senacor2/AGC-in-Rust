// SPDX-License-Identifier: GPL-3.0-or-later
/// Telemetry downlink interface.
pub trait Telemetry {
    /// Send one downlink word to the telemetry transmitter.
    fn send_word(&mut self, word: u16);
}
