//! Orbital integration: 4th-order Runge-Kutta propagator.
//!
//! Advances a `StateVector` by one timestep using classic RK4. The gravity
//! function is injected by the caller, enabling both production use (with
//! `total_gravity`) and unit testing (with mock gravity functions).
//!
//! AGC source: `Comanche055/ORBITAL_INTEGRATION.agc`, DIFEQ+0/+1/+2 (pages 1348-1349).
//! Substitution: RK4 replaces the AGC Nystrom predictor-corrector for Milestone 2.
//!   // TODO(M3): replace with AGC Nystrom predictor-corrector matching
//!   //           ORBITAL_INTEGRATION.agc DIFEQ+0/+1/+2.
//!
//! AGC source: `Comanche055/SERVICER207.agc`, CALCRVG (predictor-corrector, page 835-836).
//! // TODO: implement RECTIFY if Encke method is adopted
//!   (Comanche055/ORBITAL_INTEGRATION.agc, RECTIFY subroutine).

use crate::math::linalg::{add, scale};
use crate::navigation::state_vector::StateVector;
use crate::types::{Met, Vec3};

/// Advance a state vector by one timestep `dt` seconds using 4th-order Runge-Kutta.
///
/// Algorithm (Milestone 2): 4th-order Runge-Kutta applied to the
/// second-order ODE  r'' = grav(r, t).
///
/// RK4 stages:
///   k1_r = v,                k1_v = grav(r, t)
///   k2_r = v + dt/2*k1_v,   k2_v = grav(r + dt/2*k1_r, t + dt/2)
///   k3_r = v + dt/2*k2_v,   k3_v = grav(r + dt/2*k2_r, t + dt/2)
///   k4_r = v + dt*k3_v,     k4_v = grav(r + dt*k3_r,   t + dt)
///   r_new = r + dt/6*(k1_r + 2*k2_r + 2*k3_r + k4_r)
///   v_new = v + dt/6*(k1_v + 2*k2_v + 2*k3_v + k4_v)
///   t_new = t + dt (rounded to nearest centisecond)
///
/// // TODO(M3): replace with AGC Nystrom predictor-corrector (DIFEQ+0/+1/+2)
/// //           per Comanche055/ORBITAL_INTEGRATION.agc pages 1348-1349.
///
/// The gravity function `grav` receives the ECI position vector and the
/// time at that substep. Callers typically pass:
///   `|r, t| total_gravity(r, t, PrimaryBody::Earth)`
///
/// Preconditions (callers must ensure):
///   - `dt` is finite and positive (behaviour for dt <= 0.0 is undefined; returns input).
///   - `grav` never panics for any finite position.
///   - `|state.position()| > R_MIN_GUARD` (1.0 m) before calling.
///
/// The `gdt_over_2` field of the returned `StateVector` is updated to
/// `grav(r_new, t_new) * dt / 2` for use by SERVICER's predictor term.
///
/// Input: state with position in metres, velocity in m/s; dt in seconds.
/// Output: advanced `StateVector` with updated position, velocity, time, gdt_over_2.
///
/// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc`, DIFEQ+0/+1/+2 (pages 1348-1349).
/// Substitution rationale: `docs/agc-reference-constants.md` §Algorithms §RK4.
pub fn propagate(state: &StateVector, dt: f64, grav: &dyn Fn(&Vec3, Met) -> Vec3) -> StateVector {
    // Guard: if dt is not positive or not finite, return unchanged state
    if !dt.is_finite() || dt <= 0.0 {
        return *state;
    }

    let r = state.position();
    let v = state.velocity();
    let t = state.time();

    // dt in centiseconds for time advancement
    let dt_cs = (dt * 100.0 + 0.5) as u32;
    let t_half = t + (dt_cs / 2);
    let t_end = t + dt_cs;

    let half = dt * 0.5;
    let sixth = dt / 6.0;

    // Stage 1
    let k1_r = v;
    let k1_v = grav(&r, t);

    // Stage 2 (midpoint using k1)
    let r2 = add(&r, &scale(&k1_r, half));
    let v2 = add(&v, &scale(&k1_v, half));
    let k2_r = v2;
    let k2_v = grav(&r2, t_half);

    // Stage 3 (midpoint using k2)
    let r3 = add(&r, &scale(&k2_r, half));
    let v3 = add(&v, &scale(&k2_v, half));
    let k3_r = v3;
    let k3_v = grav(&r3, t_half);

    // Stage 4 (endpoint using k3)
    let r4 = add(&r, &scale(&k3_r, dt));
    let v4 = add(&v, &scale(&k3_v, dt));
    let k4_r = v4;
    let k4_v = grav(&r4, t_end);

    // Weighted combination: r_new = r + dt/6*(k1_r + 2*k2_r + 2*k3_r + k4_r)
    let r_new = add(
        &r,
        &scale(
            &add(
                &add(&k1_r, &scale(&k2_r, 2.0)),
                &add(&scale(&k3_r, 2.0), &k4_r),
            ),
            sixth,
        ),
    );
    let v_new = add(
        &v,
        &scale(
            &add(
                &add(&k1_v, &scale(&k2_v, 2.0)),
                &add(&scale(&k3_v, 2.0), &k4_v),
            ),
            sixth,
        ),
    );

    // Update gdt_over_2 = grav(r_new, t_end) * dt / 2
    let gdt_new = scale(&grav(&r_new, t_end), dt * 0.5);

    StateVector::with_gdt(r_new, v_new, t_end, gdt_new)
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::constants::{MU_EARTH, RE_EARTH, R_MIN_GUARD};
    use crate::navigation::gravity::earth_gravity;
    use crate::types::Met;

    /// Test 1 — Energy conservation over one LEO orbit.
    ///
    /// A 200 km circular LEO propagated for one full orbital period must conserve
    /// specific orbital energy to within 1e-6 relative error (RK4 with dt=10 s).
    ///
    /// Derived from `docs/testing.md` §5 (SERVICER cycle energy tolerance).
    #[test]
    fn energy_conservation_leo() {
        let r_mag = RE_EARTH + 200_000.0; // 200 km LEO
        let v_circ = libm::sqrt(MU_EARTH / r_mag);

        let r0 = [r_mag, 0.0_f64, 0.0];
        let v0 = [0.0_f64, v_circ, 0.0];
        let state0 = StateVector::new(r0, v0, Met(0));

        // Orbital period: T = 2π * r^(3/2) / sqrt(MU_EARTH)
        let t_orbit = 2.0 * core::f64::consts::PI * libm::pow(r_mag, 1.5) / libm::sqrt(MU_EARTH);
        let dt = 10.0_f64;
        let n_steps = (t_orbit / dt) as usize;

        // Use point-mass only for this test (no J2, no Moon) for clean energy check
        let grav_fn = |r: &Vec3, _t: Met| {
            let rm = crate::math::linalg::norm(r);
            if rm <= R_MIN_GUARD {
                return [0.0_f64, 0.0, 0.0];
            }
            let coeff = -MU_EARTH / (rm * rm * rm);
            [r[0] * coeff, r[1] * coeff, r[2] * coeff]
        };

        let mut state = state0;
        for _ in 0..n_steps {
            state = propagate(&state, dt, &grav_fn);
        }

        let e0 = 0.5 * v_circ * v_circ - MU_EARTH / r_mag;
        let e1 = 0.5 * state.speed() * state.speed() - MU_EARTH / state.radius();

        let rel_err = (e1 - e0).abs() / e0.abs();
        assert!(
            rel_err < 1e-6,
            "Energy conservation: rel_err = {rel_err} (must be < 1e-6)"
        );
    }

    /// Test 2 — Position continuity (dt → 0 limit).
    ///
    /// A very small step should change position by approximately v * dt.
    #[test]
    fn small_step_position_continuity() {
        let state = StateVector::new([6_578_000.0_f64, 0.0, 0.0], [0.0_f64, 7784.0, 0.0], Met(0));
        let dt_small = 1e-6_f64; // 1 microsecond
        let state2 = propagate(&state, dt_small, &|r, _t| earth_gravity(r));
        let expected_dy = 7784.0 * dt_small; // ≈ 7.784e-3 m
        assert!(
            (state2.position()[1] - expected_dy).abs() < 1e-10,
            "dy = {}, expected ~{expected_dy}",
            state2.position()[1]
        );
    }

    /// Test 3 — Gravity injection: zero gravity gives straight-line motion.
    #[test]
    fn zero_gravity_straight_line() {
        let state = StateVector::new([0.0_f64, 0.0, 0.0], [1.0_f64, 0.0, 0.0], Met(0));
        let state2 = propagate(&state, 5.0, &|_r, _t| [0.0_f64, 0.0, 0.0]);
        assert!(
            (state2.position()[0] - 5.0).abs() < 1e-12,
            "x={}",
            state2.position()[0]
        );
        assert!(
            (state2.velocity()[0] - 1.0).abs() < 1e-12,
            "vx={}",
            state2.velocity()[0]
        );
        // 5.0 s = 500 cs
        assert_eq!(state2.time(), Met(500), "time={:?}", state2.time());
    }

    /// Guard: dt <= 0 returns state unchanged.
    #[test]
    fn non_positive_dt_returns_unchanged() {
        let state = StateVector::new([6_578_000.0_f64, 0.0, 0.0], [0.0_f64, 7784.0, 0.0], Met(0));
        let result = propagate(&state, 0.0, &|r, _t| earth_gravity(r));
        assert_eq!(result.position(), state.position());
        let result_neg = propagate(&state, -1.0, &|r, _t| earth_gravity(r));
        assert_eq!(result_neg.position(), state.position());
    }
}
