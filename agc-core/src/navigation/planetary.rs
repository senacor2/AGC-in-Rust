//! Planetary and lunar ephemeris.
//!
//! Provides Earth and Moon state vectors as a function of time for use in
//! gravity computations and coordinate frame transformations.

use crate::types::{Met, Vec3};

/// Moon position in Earth-centred inertial frame at time `t`.
/// Returns position in metres.
pub fn moon_position(t: Met) -> Vec3 {
    let _ = t;
    todo!("lunar ephemeris")
}
