//! Numerical integration of the equations of motion.
//!
//! Implements Cowell's method (direct numerical integration) as the primary
//! integrator used in the SERVICER average-G loop.

use crate::navigation::state_vector::StateVector;

/// Propagate `state` forward by `dt` seconds using a single Runge-Kutta step.
/// Gravity is computed from the current frame and body.
pub fn rk4_step(state: StateVector, dt: f64) -> StateVector {
    let _ = dt;
    todo!("RK4 integration step")
}
