//! Mission programs (P-codes).
//!
//! Each program implements a phase of the Apollo mission. Programs are
//! dispatched by the crew via the DSKY (Verb 37) or by other programs.
//!
//! AGC source: P11.agc, P30-P37.agc, P40-P47.agc, P61-P67.agc.

pub mod p00_idle;
pub mod p11_eoi;
pub mod p30_ext_dv;
pub mod p37_return;
pub mod p40_thrusting;
pub mod p51_imu_align;
pub mod p61_entry;

/// Action returned by a program's cycle function.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProgramAction {
    /// No action needed — program is idle or waiting for crew input.
    Idle,
    /// Request DSKY display update with the given verb and noun.
    Display { verb: u8, noun: u8 },
    /// Request transition to another program.
    Transfer { prog: u8 },
}

/// Tag identifying the currently active program.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActiveProgram {
    P00,
    P11,
    P30,
    P37,
    P40,
}
