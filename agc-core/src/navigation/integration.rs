//! Numerical integration of the equations of motion.
//!
//! Implements Cowell's method (direct numerical integration) providing two
//! propagation modes that correspond to the two integration strategies used
//! in Comanche055:
//!
//! - [`average_g_step`] — two-stage predictor-corrector for powered flight
//!   (the SERVICER loop). Called every 2 seconds by `services::average_g`.
//! - [`propagate_coast`] — RK4 Cowell fallback for coasting flight.
//!   Will be replaced by Kepler + perturbation once `math::kepler` is implemented.
//! - [`total_gravity`] — combined primary + third-body gravitational acceleration.
//! - [`soi_check`] — sphere-of-influence boundary detection and frame conversion.
//!
//! # AGC source references
//!
//! - `Comanche055/AVERAGE_G_INTEGRATOR.agc` — SERVICER entry point, Average-G loop
//! - `Comanche055/ORBITAL_INTEGRATION.agc` — Cowell/Encke integrators
//! - `Comanche055/INTEGRATION_INITIALIZATION.agc` — body selection, constant tables

use crate::math::linalg::{norm, vadd, vscale, vsub};
use crate::navigation::gravity::{
    earth_gravity, moon_gravity, third_body_perturbation, MU_EARTH, MU_MOON, R_SOI_MOON,
};
use crate::navigation::state_vector::{Frame, StateVector};
use crate::types::{Met, Vec3};

// Maximum sub-step size for the Cowell RK4 fallback in `propagate_coast`.
const COAST_SUBSTEP: f64 = 10.0; // seconds

// ── total_gravity ─────────────────────────────────────────────────────────────

/// Compute the combined gravitational acceleration at `position` in `frame`.
///
/// Dispatches to `earth_gravity` or `moon_gravity` for the primary body, then
/// adds `third_body_perturbation` for the opposing body.
///
/// The `moon_pos` vector must be expressed in the same inertial frame as `position`:
/// - `EarthInertial`: `moon_pos` is the Moon's ECI position (from `planetary::moon_position`).
/// - `MoonInertial`: `moon_pos` is the Moon's ECI position; Earth's MCI position is
///   derived internally as `vscale(moon_pos, -1.0)` (the Moon's ECI position negated
///   gives Earth's position in MCI).
///
/// Panics via `debug_assert!` if `frame == StableMember` (logic error in caller).
///
/// AGC source: `Comanche055/AVERAGE_G_INTEGRATOR.agc`,
/// `Comanche055/ORBITAL_INTEGRATION.agc`.
pub fn total_gravity(position: Vec3, frame: Frame, moon_pos: Vec3) -> Vec3 {
    match frame {
        Frame::EarthInertial => {
            let g_primary = earth_gravity(position);
            let g_third = third_body_perturbation(position, moon_pos, MU_MOON);
            vadd(g_primary, g_third)
        }
        Frame::MoonInertial => {
            let g_primary = moon_gravity(position);
            // In MCI, Earth's position is the negation of the Moon's ECI position.
            let earth_pos = vscale(moon_pos, -1.0);
            let g_third = third_body_perturbation(position, earth_pos, MU_EARTH);
            vadd(g_primary, g_third)
        }
        Frame::StableMember => {
            debug_assert!(
                false,
                "total_gravity: StableMember frame is invalid for integration"
            );
            [0.0; 3]
        }
    }
}

// ── average_g_step ────────────────────────────────────────────────────────────

/// Advance `sv` by `dt` seconds using the Average-G trapezoidal scheme.
///
/// This is a two-stage Cowell predictor-corrector. It is **not** classical RK4.
/// Gravity is evaluated at the start (`g0`) and the predicted end (`g1`) of the
/// interval and averaged (trapezoidal rule). This gives second-order accuracy in
/// `dt` for the gravity term and correctly applies the discrete thrust `delta_v`.
///
/// # Parameters
///
/// - `sv` — current state vector (position m, velocity m/s, epoch MET, frame).
/// - `delta_v` — thrust-induced velocity increment already rotated to the inertial
///   frame by REFSMMAT (m/s). Zero for free-fall.
/// - `dt` — integration interval in seconds. Normally 2.0 s (the SERVICER cycle).
/// - `moon_pos` — Moon position in the same inertial frame as `sv.position` (m).
///
/// # Algorithm (Average-G)
///
/// ```text
/// g0          = total_gravity(sv.position, sv.frame, moon_pos)
/// v_half      = sv.velocity + delta_v + g0 * (dt / 2)
/// new_position = sv.position + v_half * dt
/// g1          = total_gravity(new_position, sv.frame, moon_pos)
/// new_velocity = sv.velocity + delta_v + (g0 + g1) * (dt / 2)
/// new_epoch    = sv.epoch + Met::from_seconds(dt)
/// ```
///
/// AGC source: `Comanche055/AVERAGE_G_INTEGRATOR.agc`.
pub fn average_g_step(sv: StateVector, delta_v: Vec3, dt: f64, moon_pos: Vec3) -> StateVector {
    debug_assert!(
        dt > 0.0 && dt.is_finite(),
        "average_g_step: dt must be positive and finite"
    );
    debug_assert!(
        sv.position.iter().all(|x| x.is_finite()),
        "average_g_step: sv.position contains NaN or Inf"
    );

    // Step 1 — gravity at interval start.
    let g0 = total_gravity(sv.position, sv.frame, moon_pos);

    // Step 2 — midpoint velocity estimate (predictor):
    //   v_half = velocity + delta_v + g0 * (dt/2)
    let v_half = vadd(vadd(sv.velocity, delta_v), vscale(g0, dt * 0.5));

    // Step 3 — new position using midpoint velocity.
    let new_position = vadd(sv.position, vscale(v_half, dt));

    // Step 4 — gravity at interval end.
    let g1 = total_gravity(new_position, sv.frame, moon_pos);

    // Step 5 — new velocity (corrector, trapezoidal gravity average):
    //   new_velocity = velocity + delta_v + (g0 + g1) * (dt/2)
    let g_avg = vscale(vadd(g0, g1), dt * 0.5);
    let new_velocity = vadd(vadd(sv.velocity, delta_v), g_avg);

    // Step 6 — advance epoch.
    let new_epoch = Met(sv.epoch.0.wrapping_add(Met::from_seconds(dt).0));

    StateVector {
        position: new_position,
        velocity: new_velocity,
        epoch: new_epoch,
        frame: sv.frame,
    }
}

// ── propagate_coast ───────────────────────────────────────────────────────────

/// Propagate `sv` by `dt` seconds during coasting flight (no thrust).
///
/// Uses a Cowell RK4 fallback with automatic sub-stepping (10 s per sub-step)
/// for large `dt`. This will be replaced by `math::kepler::kepler_step` plus a
/// first-order perturbation correction once that module is implemented.
///
/// # Parameters
///
/// - `sv` — current state vector.
/// - `dt` — propagation interval in seconds (may be large; up to ~5400 s for LEO).
/// - `moon_pos` — Moon position in the same inertial frame as `sv` (m).
///
/// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc`.
pub fn propagate_coast(sv: StateVector, dt: f64, moon_pos: Vec3) -> StateVector {
    // TODO: Replace with kepler_step + perturbation correction once
    // math::kepler::kepler_step is implemented.
    cowell_rk4_substepped(sv, dt, moon_pos)
}

/// Sub-stepped Cowell RK4 integrator. Splits `dt` into steps of at most
/// `COAST_SUBSTEP` seconds and applies a standard RK4 step to the 6-element
/// state [position; velocity] at each sub-step.
fn cowell_rk4_substepped(sv: StateVector, dt: f64, moon_pos: Vec3) -> StateVector {
    debug_assert!(
        dt > 0.0 && dt.is_finite(),
        "propagate_coast: dt must be positive and finite"
    );

    let n_steps = libm::ceil(dt / COAST_SUBSTEP) as usize;
    let h = dt / n_steps as f64;
    let mut current = sv;

    for _ in 0..n_steps {
        current = cowell_rk4_single(current, h, moon_pos);
    }
    current
}

/// Single RK4 step on the 6-element ODE state [position, velocity].
/// The acceleration function is `total_gravity(pos, frame, moon_pos)`.
fn cowell_rk4_single(sv: StateVector, h: f64, moon_pos: Vec3) -> StateVector {
    let frame = sv.frame;
    let r = sv.position;
    let v = sv.velocity;

    // k1
    let k1_r = v;
    let k1_v = total_gravity(r, frame, moon_pos);

    // k2
    let r2 = vadd(r, vscale(k1_r, h / 2.0));
    let v2 = vadd(v, vscale(k1_v, h / 2.0));
    let k2_r = v2;
    let k2_v = total_gravity(r2, frame, moon_pos);

    // k3
    let r3 = vadd(r, vscale(k2_r, h / 2.0));
    let v3 = vadd(v, vscale(k2_v, h / 2.0));
    let k3_r = v3;
    let k3_v = total_gravity(r3, frame, moon_pos);

    // k4
    let r4 = vadd(r, vscale(k3_r, h));
    let v4 = vadd(v, vscale(k3_v, h));
    let k4_r = v4;
    let k4_v = total_gravity(r4, frame, moon_pos);

    // Weighted sum: (k1 + 2*k2 + 2*k3 + k4) / 6
    let dr = vscale(
        vadd(vadd(k1_r, vscale(k2_r, 2.0)), vadd(vscale(k3_r, 2.0), k4_r)),
        h / 6.0,
    );
    let dv = vscale(
        vadd(vadd(k1_v, vscale(k2_v, 2.0)), vadd(vscale(k3_v, 2.0), k4_v)),
        h / 6.0,
    );

    let new_epoch = Met(sv.epoch.0.wrapping_add(Met::from_seconds(h).0));

    StateVector {
        position: vadd(r, dr),
        velocity: vadd(v, dv),
        epoch: new_epoch,
        frame,
    }
}

// ── soi_check ─────────────────────────────────────────────────────────────────

/// Test whether `sv` has crossed the sphere-of-influence boundary and, if so,
/// convert the state vector to the new frame.
///
/// The SOI radius is [`R_SOI_MOON`] (≈ 66,183 km from the Moon's centre).
///
/// - `EarthInertial` and distance from Moon < `R_SOI_MOON` → convert to `MoonInertial`.
/// - `MoonInertial`  and distance from Moon > `R_SOI_MOON` → convert to `EarthInertial`.
/// - Otherwise: return `sv` unchanged.
///
/// # Parameters
///
/// - `sv` — state vector after a propagation step.
/// - `moon_pos_eci` — Moon's ECI position at `sv.epoch` (m).
/// - `moon_vel_eci` — Moon's ECI velocity at `sv.epoch` (m/s), used for
///   velocity frame conversion.
///
/// AGC source: `Comanche055/INTEGRATION_INITIALIZATION.agc` (body-selection logic).
pub fn soi_check(sv: StateVector, moon_pos_eci: Vec3, moon_vel_eci: Vec3) -> StateVector {
    // Compute ECI position regardless of current frame.
    let pos_eci = match sv.frame {
        Frame::EarthInertial => sv.position,
        Frame::MoonInertial => vadd(sv.position, moon_pos_eci),
        Frame::StableMember => return sv, // should never happen
    };

    let dist_from_moon = norm(vsub(pos_eci, moon_pos_eci));

    match sv.frame {
        Frame::EarthInertial if dist_from_moon < R_SOI_MOON => {
            // Entering Moon SOI: convert ECI → MCI.
            StateVector {
                position: vsub(sv.position, moon_pos_eci),
                velocity: vsub(sv.velocity, moon_vel_eci),
                epoch: sv.epoch,
                frame: Frame::MoonInertial,
            }
        }
        Frame::MoonInertial if dist_from_moon > R_SOI_MOON => {
            // Leaving Moon SOI: convert MCI → ECI.
            StateVector {
                position: vadd(sv.position, moon_pos_eci),
                velocity: vadd(sv.velocity, moon_vel_eci),
                epoch: sv.epoch,
                frame: Frame::EarthInertial,
            }
        }
        _ => sv,
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::gravity::{earth_gravity, third_body_perturbation, MU_EARTH, MU_MOON};
    use crate::navigation::state_vector::Frame;
    use crate::types::Met;

    fn assert_near(a: f64, b: f64, tol: f64, label: &str) {
        assert!(
            (a - b).abs() < tol,
            "{label}: got {a}, expected {b}, tolerance {tol}"
        );
    }

    // Simplified Moon position used across tests.
    const MOON_POS: Vec3 = [3.844e8, 0.0, 0.0];
    // Zero Moon velocity (placeholder — adequate for tests not checking SOI velocity)
    const MOON_VEL: Vec3 = [0.0, 1022.0, 0.0];

    // ── TC-INT-7: total_gravity composition in ECI frame ──────────────────────

    /// TC-INT-7: total_gravity in EarthInertial equals earth_gravity + third_body_perturbation.
    #[test]
    fn tc_int_7_total_gravity_eci_composition() {
        use crate::math::linalg::vadd;

        let position: Vec3 = [7_000_000.0, 0.0, 0.0];
        let moon_pos: Vec3 = MOON_POS;

        let tg = total_gravity(position, Frame::EarthInertial, moon_pos);
        let g_earth = earth_gravity(position);
        let g_moon = third_body_perturbation(position, moon_pos, MU_MOON);
        let expected = vadd(g_earth, g_moon);

        for i in 0..3 {
            assert_near(tg[i], expected[i], 1e-12, &format!("component {i}"));
        }

        // Sanity: dominant term is along -x, magnitude ~8.147 m/s²
        assert!(
            tg[0] < 0.0,
            "TC-INT-7: gravity must be negative (toward Earth)"
        );
        assert!(
            tg[0].abs() > 8.0 && tg[0].abs() < 9.0,
            "TC-INT-7: |g[0]| = {} out of expected range [8, 9]",
            tg[0].abs()
        );
    }

    // ── TC-INT-1: Circular orbit — radius conservation ────────────────────────

    /// TC-INT-1: average_g_step preserves orbital radius for a circular LEO
    /// over 100 steps (200 s) with zero delta-V.
    ///
    /// The second-order Average-G scheme accumulates drift over many steps.
    /// After 200 s (~3.5% of a LEO orbital period), the drift must stay within
    /// 0.01% of the orbital radius (~680 m) and 0.01% of circular velocity.
    #[test]
    fn tc_int_1_circular_orbit_radius_conservation() {
        let r0 = 6_778_000.0_f64;
        let v_circ = libm::sqrt(MU_EARTH / r0);

        let mut sv = StateVector {
            position: [r0, 0.0, 0.0],
            velocity: [0.0, v_circ, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };
        let dv = [0.0_f64; 3];
        let dt = 2.0_f64;

        for _ in 0..100 {
            sv = average_g_step(sv, dv, dt, MOON_POS);
        }

        let r_final = norm(sv.position);
        let v_final = norm(sv.velocity);
        let r_drift = (r_final - r0).abs();
        let v_drift = (v_final - v_circ).abs();

        // 0.01% of orbital radius ≈ 678 m; 0.01% of v_circ ≈ 0.77 m/s
        assert!(
            r_drift < r0 * 1e-4,
            "TC-INT-1: radius drift = {r_drift} m after 200 s, exceeds 0.01% of r0"
        );
        assert!(
            v_drift < v_circ * 1e-4,
            "TC-INT-1: speed drift = {v_drift} m/s after 200 s, exceeds 0.01% of v_circ"
        );
    }

    // ── TC-INT-2: Free-fall — matches gravity extrapolation ───────────────────

    /// TC-INT-2: average_g_step with zero velocity and zero delta-V produces
    /// position and velocity consistent with direct gravity integration.
    #[test]
    fn tc_int_2_free_fall_matches_gravity_extrapolation() {
        let sv = StateVector {
            position: [7_000_000.0, 0.0, 0.0],
            velocity: [0.0, 0.0, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };
        let dv = [0.0_f64; 3];
        let dt = 2.0_f64;
        let moon_pos = MOON_POS;

        let result = average_g_step(sv, dv, dt, moon_pos);

        // Expected values from hand-calculation (pure x-axis, z=0, J2 zero on equator):
        //   g0 ≈ -MU_EARTH / 7e6² ≈ -8.1429 m/s² (point-mass; J2 term also purely along x)
        //   new_velocity[0] ≈ (g0_x + g1_x) * 1.0 (average over dt=2s, using dt/2=1)
        //   In practice g0 and g1 are very close since position barely changes.
        let v_x = result.velocity[0];
        assert!(
            (v_x - (-16.294)).abs() < 0.05,
            "TC-INT-2: velocity[0] = {v_x}, expected ≈ -16.294 m/s"
        );

        let p_x = result.position[0];
        assert!(
            (p_x - 6_999_983.7).abs() < 0.5,
            "TC-INT-2: position[0] = {p_x}, expected ≈ 6_999_983.7 m"
        );

        // Y and Z must remain zero by symmetry.
        assert_near(result.velocity[1], 0.0, 1e-10, "TC-INT-2 velocity[1]");
        assert_near(result.velocity[2], 0.0, 1e-10, "TC-INT-2 velocity[2]");
    }

    // ── TC-INT-3: Thrust delta-V applied correctly ────────────────────────────

    /// TC-INT-3: A 10 m/s prograde delta-V is added exactly to the velocity and
    /// the gravity contribution to the velocity is independent of the thrust.
    #[test]
    fn tc_int_3_thrust_delta_v_applied_correctly() {
        let sv = StateVector {
            position: [6_781_000.0, 0.0, 0.0],
            velocity: [0.0, 7_660.0, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };
        let dv: Vec3 = [0.0, 10.0, 0.0]; // 10 m/s prograde
        let dt = 2.0_f64;
        let moon_pos = MOON_POS;

        let result = average_g_step(sv, dv, dt, moon_pos);

        // Compare with a no-thrust step to isolate the delta-V contribution.
        let result_no_dv = average_g_step(sv, [0.0; 3], dt, moon_pos);
        let dv_y_applied = result.velocity[1] - result_no_dv.velocity[1];
        assert!(
            (dv_y_applied - 10.0).abs() < 1e-3,
            "TC-INT-3: delta-V Y applied = {dv_y_applied}, expected 10.0 m/s"
        );

        // velocity[0] is the gravity contribution alone (delta-V was along Y).
        // At this radius, gravity ≈ -8.681 m/s², over dt=2s → ≈ -17.362 m/s.
        assert!(
            (result.velocity[0] - (-17.362)).abs() < 0.05,
            "TC-INT-3: velocity[0] = {}, expected ≈ -17.362 m/s",
            result.velocity[0]
        );
    }

    // ── TC-INT-4: Energy conservation during coast propagation ────────────────

    /// TC-INT-4: propagate_coast conserves specific orbital energy for a circular
    /// LEO over one full orbit period (≈ 5400 s).
    ///
    /// Acceptance: relative energy error < 1e-4 (0.01%), radius error < 5 km.
    #[test]
    fn tc_int_4_energy_conservation_coast_propagation() {
        let r = 6_778_000.0_f64;
        let v_circ = libm::sqrt(MU_EARTH / r);

        let sv = StateVector {
            position: [r, 0.0, 0.0],
            velocity: [0.0, v_circ, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };
        let moon_pos = MOON_POS;

        // One approximate LEO orbit period
        let dt = 5400.0_f64;
        let result = propagate_coast(sv, dt, moon_pos);

        // Specific orbital energy E = 0.5*v² - MU/r
        let e0 = 0.5 * v_circ * v_circ - MU_EARTH / r;
        let v_f = norm(result.velocity);
        let r_f = norm(result.position);
        let e_f = 0.5 * v_f * v_f - MU_EARTH / r_f;

        let rel_err = (e_f - e0).abs() / e0.abs();
        assert!(
            rel_err < 1e-4,
            "TC-INT-4: relative energy error = {rel_err:.2e}, exceeds 1e-4"
        );

        assert!(
            (r_f - r).abs() < 5000.0,
            "TC-INT-4: radius error = {} m, exceeds 5 km",
            (r_f - r).abs()
        );
    }

    // ── TC-INT-SOI: SOI transition ECI → MCI ─────────────────────────────────

    /// Verify that soi_check converts ECI → MCI when the spacecraft is inside
    /// the Moon's SOI, and returns unchanged when well outside.
    #[test]
    fn tc_int_soi_transition_eci_to_mci() {
        let moon_pos_eci: Vec3 = [3.844e8, 0.0, 0.0];
        let moon_vel_eci: Vec3 = MOON_VEL;

        // Spacecraft just inside the SOI (distance from Moon < R_SOI_MOON).
        let dist_inside = R_SOI_MOON - 1000.0;
        let sv_eci = StateVector {
            position: [moon_pos_eci[0] - dist_inside, 0.0, 0.0],
            velocity: [0.0, 900.0, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        let result = soi_check(sv_eci, moon_pos_eci, moon_vel_eci);
        assert_eq!(
            result.frame,
            Frame::MoonInertial,
            "TC-INT-SOI: should have converted to MoonInertial"
        );
        assert_near(
            result.position[0],
            -dist_inside,
            1.0,
            "TC-INT-SOI: MCI position[0]",
        );
        assert_near(
            result.velocity[1],
            900.0 - moon_vel_eci[1],
            1e-9,
            "TC-INT-SOI: MCI velocity[1]",
        );
        assert_eq!(
            result.epoch, sv_eci.epoch,
            "TC-INT-SOI: epoch must not change"
        );

        // Spacecraft well outside the SOI — should be unchanged.
        let sv_outside = StateVector {
            position: [1.0e8, 0.0, 0.0], // 100,000 km from Earth, far from Moon SOI
            velocity: [0.0, 3000.0, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };
        let unchanged = soi_check(sv_outside, moon_pos_eci, moon_vel_eci);
        assert_eq!(
            unchanged.frame,
            Frame::EarthInertial,
            "TC-INT-SOI: should remain EarthInertial"
        );
    }
}
