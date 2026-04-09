//! Maneuver targeting — Time of Ignition (TIG) and burn attitude.

use crate::types::{DeltaV, Met};
use crate::navigation::state_vector::StateVector;

/// A targeted maneuver: when to ignite and how much delta-V to apply.
#[derive(Clone, Copy, Debug)]
pub struct Maneuver {
    /// Time of ignition.
    pub tig: Met,
    /// Delta-V in the reference frame (m/s).
    pub delta_v: DeltaV,
}

/// Compute the required delta-V to reach `target` state from `current`
/// at time `tig`, given the gravitational parameter `mu`.
pub fn compute_delta_v(current: StateVector, target: StateVector, tig: Met, mu: f64) -> DeltaV {
    let _ = (current, target, tig, mu);
    todo!("delta-V targeting")
}
