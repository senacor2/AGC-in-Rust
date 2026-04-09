//! Time conversion utilities.
//!
//! Converts between Mission Elapsed Time (MET) and other time representations
//! used in the guidance equations.

use crate::types::Met;

/// Julian date of the Apollo 11 launch (21 July 1969 UTC) as a reference.
/// Real missions use the actual launch MET uploaded by the ground.
pub const REFERENCE_JD: f64 = 2440422.5;

/// Convert MET to Greenwich Mean Sidereal Time (radians).
/// Required for Earth-fixed ↔ inertial frame conversions.
pub fn met_to_gmst(t: Met, launch_jd: f64) -> f64 {
    let _ = (t, launch_jd);
    todo!("MET to GMST conversion")
}
