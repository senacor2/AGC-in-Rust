/// Telemetry downlink interface.
pub trait Telemetry {
    /// Send one downlink word to the telemetry transmitter.
    fn send_word(&mut self, word: u16);
}
