//! Earth and Moon gravity models.
//!
//! Provides gravitational acceleration as a function of position, including
//! the J2 oblateness correction for Earth and the point-mass model for the Moon.

use crate::types::Vec3;

/// Earth gravitational parameter μ = GM (m³/s²).
pub const MU_EARTH: f64 = 3.986_004_418e14;

/// Moon gravitational parameter μ = GM (m³/s²).
pub const MU_MOON: f64 = 4.902_800_118e12;

/// Earth equatorial radius (m).
pub const R_EARTH: f64 = 6_378_137.0;

/// Earth J2 oblateness coefficient.
pub const J2_EARTH: f64 = 1.082_626_68e-3;

/// Gravitational acceleration due to Earth at position `r` (m), including J2.
/// Returns acceleration in m/s².
pub fn earth_gravity(r: Vec3) -> Vec3 {
    let _ = r;
    todo!("Earth gravity with J2 oblateness")
}

/// Gravitational acceleration due to the Moon at position `r` (m).
/// Returns acceleration in m/s².
pub fn moon_gravity(r: Vec3) -> Vec3 {
    let _ = r;
    todo!("Moon point-mass gravity")
}
