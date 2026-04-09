//! Kepler equation solvers and universal variable formulation.
//!
//! Implements the conic propagation routines that replace the AGC's
//! KEPSILON interpretive subroutine.

use crate::types::Vec3;

/// Propagate a state vector by time `dt` seconds under a central body
/// with gravitational parameter `mu` (m³/s²).
///
/// Uses the universal variable (Battin) method, valid for all conic sections.
/// Returns `(position_m, velocity_m_s)` at time `t0 + dt`.
pub fn kepler_step(r0: Vec3, v0: Vec3, dt: f64, mu: f64) -> (Vec3, Vec3) {
    let _ = (r0, v0, dt, mu);
    todo!("universal-variable Kepler propagator")
}
