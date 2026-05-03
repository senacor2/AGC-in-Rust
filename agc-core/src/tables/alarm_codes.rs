//! Program alarm code definitions.

/// Executive overflow — no free job slots (the Apollo 11 "1202" alarm).
pub const EXEC_OVERFLOW: u16 = 1202;
/// Executive overflow — no free VAC areas. Unused: the interpretive language
/// is eliminated in this port (ADR-001), so no VAC pool exists and FINDVAC
/// has no Rust equivalent. Retained for reference only.
#[allow(dead_code)]
pub const NO_VAC: u16 = 1210;
/// Waitlist overflow — no free task slots.
pub const WAITLIST_OVERFLOW: u16 = 1211;
/// IMU not aligned (REFSMMAT invalid).
pub const IMU_NOT_ALIGNED: u16 = 0o210;
/// Celestial body too close to Sun for optical sighting.
pub const BODY_TOO_CLOSE_TO_SUN: u16 = 0o206;
/// Navigation integration failed to converge.
pub const NAV_NO_CONVERGE: u16 = 0o401;
/// Invalid orbit (sub-parabolic or degenerate conic).
pub const INVALID_ORBIT: u16 = 0o404;
