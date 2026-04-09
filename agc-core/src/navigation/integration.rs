//! Orbital state vector propagation via fixed-step RK4 integrator.
//!
//! AGC source: Comanche055/ORBITAL_INTEGRATION.agc — INTGRATE Cowell integrator.
//!
//! The AGC used a second-order trapezoidal predictor-corrector. A 4th-order
//! Runge-Kutta (RK4) is substituted here: it achieves the same 4th-order
//! truncation error per step with a cleaner, branchless implementation.

use crate::math::linalg::{add, scale};
use crate::types::Vec3;

use super::state_vector::StateVector;

/// Function that computes total acceleration (gravity + perturbations) at (r, v).
pub type AccelFn = fn(r: &Vec3, v: &Vec3) -> Vec3;

/// Propagate a state vector forward by `dt` seconds using a single RK4 step.
///
/// `accel` computes total acceleration (gravity + perturbations) at (r, v).
///
/// AGC source: ORBITAL_INTEGRATION.agc — INTGRATE.
/// Note: the AGC used a second-order predictor-corrector. RK4 is substituted
/// here as it achieves the same 4th-order accuracy with a cleaner implementation.
pub fn rk4_step(sv: &StateVector, dt: f64, accel: AccelFn) -> StateVector {
    let r0 = sv.r;
    let v0 = sv.v;

    // k1
    let a1 = accel(&r0, &v0);
    let k1r = v0;
    let k1v = a1;

    // k2
    let r2 = add(&r0, &scale(&k1r, dt * 0.5));
    let v2 = add(&v0, &scale(&k1v, dt * 0.5));
    let a2 = accel(&r2, &v2);
    let k2r = v2;
    let k2v = a2;

    // k3
    let r3 = add(&r0, &scale(&k2r, dt * 0.5));
    let v3 = add(&v0, &scale(&k2v, dt * 0.5));
    let a3 = accel(&r3, &v3);
    let k3r = v3;
    let k3v = a3;

    // k4
    let r4 = add(&r0, &scale(&k3r, dt));
    let v4 = add(&v0, &scale(&k3v, dt));
    let a4 = accel(&r4, &v4);
    let k4r = v4;
    let k4v = a4;

    // Combine: y_{n+1} = y_n + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    let dr = weighted_sum(&k1r, &k2r, &k3r, &k4r, dt);
    let dv = weighted_sum(&k1v, &k2v, &k3v, &k4v, dt);

    let r_new = add(&r0, &dr);
    let v_new = add(&v0, &dv);

    let dt_cs = libm::round(dt * 100.0) as u32;
    StateVector {
        frame: sv.frame,
        r: r_new,
        v: v_new,
        t: sv.t.advance(dt_cs),
    }
}

/// RK4 weighted sum: (k1 + 2*k2 + 2*k3 + k4) * dt/6.
#[inline]
fn weighted_sum(k1: &Vec3, k2: &Vec3, k3: &Vec3, k4: &Vec3, dt: f64) -> Vec3 {
    let s = add(k1, &add(&scale(k2, 2.0), &add(&scale(k3, 2.0), k4)));
    scale(&s, dt / 6.0)
}

/// Propagate forward by `total_dt` seconds in steps of `step` seconds.
///
/// If `total_dt` is not a multiple of `step`, the last step is shortened to
/// reach exactly `total_dt`.
pub fn propagate(sv: &StateVector, total_dt: f64, step: f64, accel: AccelFn) -> StateVector {
    let mut current = *sv;
    let mut remaining = total_dt;
    while remaining > 0.0 {
        let dt = if remaining < step { remaining } else { step };
        current = rk4_step(&current, dt, accel);
        remaining -= dt;
    }
    current
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::linalg::norm;
    use crate::navigation::gravity::{j2_perturbation, point_mass, MU_EARTH};
    use crate::navigation::state_vector::Frame;
    use crate::types::Met;
    use core::f64::consts::PI;

    fn earth_accel(r: &Vec3, _v: &Vec3) -> Vec3 {
        let pm = point_mass(r, MU_EARTH);
        let j2 = j2_perturbation(r);
        [pm[0] + j2[0], pm[1] + j2[1], pm[2] + j2[2]]
    }

    fn earth_accel_no_j2(r: &Vec3, _v: &Vec3) -> Vec3 {
        point_mass(r, MU_EARTH)
    }

    #[test]
    fn circular_orbit_energy_conservation() {
        // 200 km altitude circular orbit.
        let r0 = 6_578_000.0_f64;
        let v_circ = libm::sqrt(MU_EARTH / r0);
        let t_orbit = 2.0 * PI * r0 / v_circ;

        let sv0 = StateVector {
            frame: Frame::Eci,
            r: [r0, 0.0, 0.0],
            v: [0.0, v_circ, 0.0],
            t: Met::ZERO,
        };

        // Propagate half period with 10-second steps.
        let sv_half = propagate(&sv0, t_orbit / 2.0, 10.0, earth_accel_no_j2);

        let r_initial = norm(&sv0.r);
        let r_final = norm(&sv_half.r);

        // Energy conservation: radius should remain within 100 m.
        let dr = libm::fabs(r_final - r_initial);
        assert!(
            dr < 100.0,
            "radius drifted by {dr:.2} m over half orbit (threshold 100 m)"
        );
    }

    #[test]
    fn met_advances_during_propagation() {
        let r0 = 6_578_000.0_f64;
        let v_circ = libm::sqrt(MU_EARTH / r0);
        let sv0 = StateVector {
            frame: Frame::Eci,
            r: [r0, 0.0, 0.0],
            v: [0.0, v_circ, 0.0],
            t: Met::ZERO,
        };
        let sv1 = propagate(&sv0, 10.0, 10.0, earth_accel);
        assert_eq!(sv1.t.0, 1000, "10 s = 1000 centiseconds");
    }
}
