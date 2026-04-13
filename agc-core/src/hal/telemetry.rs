//! PCM telemetry downlink interface.
//!
//! AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
//!             DNTM1 = octal 34 — downlink telemetry word 1
//!             DNTM2 = octal 35 — downlink telemetry word 2
//!             CHAN33 bit 12 — downlink too fast (read in C33TEST)

/// PCM telemetry downlink interface.
///
/// Writes downlink word pairs for transmission to the ground.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc.
/// DNTM1 = octal 34, DNTM2 = octal 35.
pub trait TelemetryIo {
    /// Write a downlink word pair (called on DOWNRUPT).
    ///
    /// `word1`: data for DNTM1 (octal 34).
    /// `word2`: data for DNTM2 (octal 35).
    ///
    /// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc DNTM1/DNTM2 registers.
    fn write_downlink_pair(&mut self, word1: u16, word2: u16);

    /// True if the downlink system has signaled "too fast" (channel 33 bit 12).
    ///
    /// AGC source: Comanche055/T4RUPT_PROGRAM.agc C33TEST routine.
    fn downlink_overrun(&self) -> bool;

    /// Clear the overrun flag.
    fn clear_overrun(&mut self);

    /// Write channel 13 (SPS/TVC discrete outputs).
    ///
    /// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc CHAN13 = octal 13.
    /// Used by fresh_start to clear TEST ALARMS and STANDBY ENABLE bits.
    fn write_chan13(&mut self, bits: u16);
}

/// Bare-metal telemetry implementation skeleton.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc DNTM1/DNTM2 (octal 34-35).
pub struct TelemetryImpl {
    last_word1: u16,
    last_word2: u16,
    overrun: bool,
    chan13: u16,
}

impl TelemetryImpl {
    /// Construct with no pending words and no overrun.
    pub const fn new() -> Self {
        Self {
            last_word1: 0,
            last_word2: 0,
            overrun: false,
            chan13: 0,
        }
    }

    /// Release the underlying peripheral handle (C-FREE).
    pub fn free(self) -> (u16, u16) {
        (self.last_word1, self.last_word2)
    }
}

impl Default for TelemetryImpl {
    fn default() -> Self {
        Self::new()
    }
}

impl TelemetryIo for TelemetryImpl {
    fn write_downlink_pair(&mut self, word1: u16, word2: u16) {
        self.last_word1 = word1;
        self.last_word2 = word2;
    }

    fn downlink_overrun(&self) -> bool {
        self.overrun
    }

    fn clear_overrun(&mut self) {
        self.overrun = false;
    }

    fn write_chan13(&mut self, bits: u16) {
        self.chan13 = bits;
    }
}
