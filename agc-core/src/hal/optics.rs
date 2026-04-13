//! CM optics shaft/trunnion CDU interface.
//!
//! AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
//!             CDUT = octal 35 (optics trunnion CDU = OPTY)
//!             CDUS = octal 36 (optics shaft CDU = OPTX)
//!             CDUTCMD = octal 53 (trunnion command = OPTYCMD)
//!             CDUSCMD = octal 54 (shaft command = OPTXCMD)

use crate::types::CduAngle;

/// Optics shaft/trunnion CDU interface.
///
/// Provides position readback and incremental drive commands for the
/// sextant/telescope optics assembly.
///
/// Note: TVC gimbal trim commands (TVCPITCH, TVCYAW) share the CDU command
/// registers but are issued through `EngineIo::trim_gimbal`, not `OpticsIo`.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
/// CDUT = octal 35 (trunnion), CDUS = octal 36 (shaft).
/// CDUTCMD = octal 53, CDUSCMD = octal 54.
pub trait OpticsIo {
    /// Read current shaft CDU angle (CDUS, octal 36).
    fn read_shaft(&self) -> CduAngle;

    /// Read current trunnion CDU angle (CDUT, octal 35).
    fn read_trunnion(&self) -> CduAngle;

    /// Command an incremental shaft drive (CDUSCMD, octal 54).
    ///
    /// Units: raw CDU pulse count; positive = one direction.
    fn drive_shaft(&mut self, delta: i16);

    /// Command an incremental trunnion drive (CDUTCMD, octal 53).
    ///
    /// Units: raw CDU pulse count.
    fn drive_trunnion(&mut self, delta: i16);

    /// Write channel 14 bits (ISS CDU pulse enables / optics channel 14).
    ///
    /// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc CHAN14 = octal 14.
    fn write_chan14(&mut self, bits: u16);
}

/// Bare-metal optics implementation skeleton.
///
/// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc optics CDU routines.
pub struct OpticsImpl {
    shaft: CduAngle,
    trunnion: CduAngle,
    chan14_shadow: u16,
}

impl OpticsImpl {
    /// Construct a new optics implementation with zeroed positions.
    pub const fn new() -> Self {
        Self {
            shaft: CduAngle(0),
            trunnion: CduAngle(0),
            chan14_shadow: 0,
        }
    }

    /// Release the underlying peripheral handle (C-FREE).
    pub fn free(self) -> u16 {
        self.chan14_shadow
    }
}

impl Default for OpticsImpl {
    fn default() -> Self {
        Self::new()
    }
}

impl OpticsIo for OpticsImpl {
    fn read_shaft(&self) -> CduAngle {
        self.shaft
    }

    fn read_trunnion(&self) -> CduAngle {
        self.trunnion
    }

    fn drive_shaft(&mut self, delta: i16) {
        self.shaft = CduAngle(self.shaft.0.wrapping_add(delta as u16));
    }

    fn drive_trunnion(&mut self, delta: i16) {
        self.trunnion = CduAngle(self.trunnion.0.wrapping_add(delta as u16));
    }

    fn write_chan14(&mut self, bits: u16) {
        self.chan14_shadow = bits;
    }
}
