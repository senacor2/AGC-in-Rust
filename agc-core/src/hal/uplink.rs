//! Uplink receiver I/O interface.
//!
//! AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
//!             INLINK = octal 45 — uplink word input register
//!             CHAN33 bit 11 — uplink too fast (read in C33TEST, T4RUPT_PROGRAM.agc page 146)

/// Uplink receiver I/O interface.
///
/// Provides access to uplink words received from ground via UPRUPT.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc, INLINK = octal 45.
///             Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc (uplink = keyboard equivalent).
pub trait UplinkIo {
    /// Read the next uplink word (5-bit code), or `None` if none is pending.
    ///
    /// Corresponds to reading INLINK (octal 45) on UPRUPT.
    ///
    /// AGC source: Comanche055/T4RUPT_PROGRAM.agc UPRUPT handler.
    fn read_uplink_word(&mut self) -> Option<u8>;

    /// True if the "uplink too fast" flag is set (channel 33 bit 11).
    ///
    /// AGC source: Comanche055/T4RUPT_PROGRAM.agc C33TEST routine.
    fn uplink_overrun(&self) -> bool;

    /// Clear the uplink overrun flag.
    fn clear_overrun(&mut self);
}

/// Bare-metal uplink implementation skeleton.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc INLINK register (octal 45).
pub struct UplinkImpl {
    pending: Option<u8>,
    overrun: bool,
}

impl UplinkImpl {
    /// Construct with no pending word and no overrun.
    pub const fn new() -> Self {
        Self {
            pending: None,
            overrun: false,
        }
    }

    /// Release the underlying peripheral handle (C-FREE).
    pub fn free(self) -> Option<u8> {
        self.pending
    }
}

impl Default for UplinkImpl {
    fn default() -> Self {
        Self::new()
    }
}

impl UplinkIo for UplinkImpl {
    fn read_uplink_word(&mut self) -> Option<u8> {
        self.pending.take()
    }

    fn uplink_overrun(&self) -> bool {
        self.overrun
    }

    fn clear_overrun(&mut self) {
        self.overrun = false;
    }
}
