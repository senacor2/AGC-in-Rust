//! Navigation and guidance program modules (Major Modes).
//!
//! Each public sub-module corresponds to one or more AGC major modes (P00–P67)
//! as defined in `Comanche055`. Programs receive `&mut AgcState` and
//! `&mut dyn AgcHardware` and must not perform heap allocation.
//!
//! AGC source: Comanche055/FRESH_START_AND_RESTART.agc (V37 dispatch table,
//!             PREMM1 allowed-mode table, pages 194-201).
//!             Comanche055/EXECUTIVE.agc (DUMMYJOB, job dispatch).

pub mod p00_idle;
pub mod p11_eoi;
pub mod p30_ext_dv;
pub mod p37_return;
pub mod p40_thrusting;
pub mod p51_imu_align;
pub mod p61_entry;

/// AGC major mode identifier.
///
/// Corresponds to the MMNUMBER / MODREG encoding in the AGC.
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc PREMM1 table (page 195).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ProgramId {
    /// P00 — CMC Idle (POOH).
    P00 = 0,
    /// P11 — Earth Orbit Insertion Monitor.
    P11 = 11,
    /// P30 — External Delta-V Targeting.
    P30 = 30,
    /// P37 — Return-to-Earth Targeting (Lambert).
    P37 = 37,
    /// P40 — SPS Burn Execution.
    P40 = 40,
    /// P41 — RCS (+X) Burn Execution.
    P41 = 41,
    /// P51 — Initial IMU Alignment.
    P51 = 51,
    /// P52 — In-Flight IMU Realignment.
    P52 = 52,
    /// P61 — Entry Guidance Pre-Entry.
    P61 = 61,
}

impl ProgramId {
    /// Convert a major-mode register value to a `ProgramId`, if valid.
    ///
    /// Returns `None` for unrecognised or out-of-scope major modes.
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc PREMM1 table.
    pub fn from_modreg(modreg: i16) -> Option<Self> {
        match modreg {
            0 => Some(Self::P00),
            11 => Some(Self::P11),
            30 => Some(Self::P30),
            37 => Some(Self::P37),
            40 => Some(Self::P40),
            41 => Some(Self::P41),
            51 => Some(Self::P51),
            52 => Some(Self::P52),
            61 => Some(Self::P61),
            _ => None,
        }
    }

    /// Return the MODREG value for this program.
    pub const fn modreg(self) -> i16 {
        self as i16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn program_id_roundtrip() {
        assert_eq!(ProgramId::from_modreg(0), Some(ProgramId::P00));
        assert_eq!(ProgramId::from_modreg(40), Some(ProgramId::P40));
        assert_eq!(ProgramId::from_modreg(99), None);
        assert_eq!(ProgramId::P37.modreg(), 37);
    }
}
