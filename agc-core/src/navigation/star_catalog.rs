//! Fixed star catalog for IMU alignment (P51/P52).
//!
//! Contains the 37 navigational stars used by Comanche055, with unit vectors
//! in the Earth mean equatorial frame of epoch J2000.

use crate::types::Vec3;

/// A navigational star entry.
pub struct Star {
    /// Star number (AGC catalog number, 1-based).
    pub number: u8,
    /// Common name or designation.
    pub name: &'static str,
    /// Unit vector toward the star in J2000 Earth equatorial frame.
    pub direction: Vec3,
}

/// The 37 navigational stars. Directions are populated during implementation.
pub static STAR_CATALOG: &[Star] = &[
    // TODO: populate with actual star directions from the AGC star table
];
