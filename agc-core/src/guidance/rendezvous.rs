//! Rendezvous navigation primitives (P20–P23, P31–P34).
//!
//! This module provides stateless, pure-math functions that answer the four
//! fundamental questions of rendezvous navigation: relative position, range,
//! range-rate, line-of-sight direction, and time to closest approach.
//!
//! # Coordinate frame
//!
//! All functions that work in the LVLH frame use the **rendezvous LVLH** (Hill/
//! Clohessy–Wiltshire) convention:
//!
//! | Axis | Direction |
//! |------|-----------|
//! | x    | Along-track (prograde for circular orbits) |
//! | y    | Out-of-plane, **opposite** to angular momentum |
//! | z    | Radial-inward (toward gravitating body) |
//!
//! This differs from the RSW frame used in `guidance::targeting`:
//! `targeting::lvlh_to_inertial` uses R = outward radial (used for ΔV uplinks),
//! whereas this module's `lvlh_matrix` is the Hill frame (z = inward radial, used
//! for relative-motion display and R32–R35 crew displays).  Module-path
//! qualification at call sites disambiguates: `rendezvous::lvlh_matrix` vs.
//! `targeting::lvlh_to_inertial`.
//!
//! See spec §4 for the full reconciliation table.
//!
//! # References
//! - Spec: `specs/rendezvous-spec.md`
//! - O'Brien, *The Apollo Guidance Computer*, Chapter 11, pp. 303–340
//! - AGC source: `Comanche055/R32,R33,R34,R35.agc`

use crate::math::linalg::{cross, dot, mxv, norm, unit, vsub};
use crate::math::trig::atan2;
use crate::types::{Mat3x3, Vec3};

// ── Types ─────────────────────────────────────────────────────────────────────

/// Relative state of the active vehicle with respect to the target vehicle,
/// expressed in the rendezvous LVLH frame of the target vehicle.
///
/// All components are in SI units (m for position, m/s for velocity).
/// Axis conventions: x = along-track (velocity direction for circular orbits),
/// y = out-of-plane (opposite angular momentum), z = radial-inward (toward body).
///
/// Spec: rendezvous-spec.md §5.1
#[derive(Clone, Copy, Debug)]
pub struct LvlhState {
    /// Relative position vector in the LVLH frame (m).
    /// rho_lvlh = M_I2L * (r_active - r_target)
    pub rho: Vec3,
    /// Relative velocity vector in the LVLH frame (m/s).
    /// rho_dot_lvlh = M_I2L * (v_active - v_target)
    pub rho_dot: Vec3,
}

/// Line-of-sight angles to the target expressed in the rendezvous LVLH frame.
///
/// Elevation: angle above the local horizontal plane (range [-π/2, +π/2]).
/// Azimuth: angle in the local horizontal plane measured from +x (along-track),
///          positive toward +y (out-of-plane), range (-π, +π].
///
/// Spec: rendezvous-spec.md §5.2
pub struct LosAngles {
    /// Elevation angle (rad).  Positive = target is above local horizontal.
    pub elevation: f64,
    /// Azimuth angle (rad).  0 = directly ahead along-track; π/2 = out-of-plane.
    pub azimuth: f64,
}

// ── Public API ────────────────────────────────────────────────────────────────

/// Compute the rotation matrix from the inertial frame to the rendezvous LVLH
/// frame of a target vehicle.
///
/// # Preconditions
/// - `r_target` must be non-zero (panics if `norm(r_target) == 0.0`).
/// - `r_target` and `v_target` must not be parallel (degenerate: radial flight;
///   see §8 Edge Cases).  Panics if `norm(cross(r_target, v_target)) == 0.0`.
///
/// # Postconditions
/// - The returned matrix is orthonormal: `M * M^T = I` (up to f64 rounding).
/// - Row 0 = x_hat (along-track), Row 1 = y_hat (out-of-plane), Row 2 = z_hat (radial-inward).
///
/// # Invariant
/// Given `r_t` on a circular orbit, `mxv(result, v_target)` has zero z-component
/// and positive x-component.
///
/// Spec: rendezvous-spec.md §6.1
pub fn lvlh_matrix(r_target: Vec3, v_target: Vec3) -> Mat3x3 {
    // Step 1: angular momentum vector (not normalised).
    let h = cross(r_target, v_target);

    // Step 2: LVLH basis unit vectors.
    // z_hat: radial-inward; panics if r_target is the zero vector.
    assert!(
        norm(r_target) != 0.0,
        "lvlh_matrix: r_target is the zero vector; LVLH frame undefined"
    );
    let z_hat = unit([
        -r_target[0],
        -r_target[1],
        -r_target[2],
    ]);

    // y_hat: out-of-plane (opposite angular momentum); panics for radial flight.
    assert!(
        norm(h) != 0.0,
        "lvlh_matrix: r_target and v_target are parallel (radial flight); LVLH frame undefined"
    );
    let y_hat = unit([-h[0], -h[1], -h[2]]);

    // x_hat: along-track; guaranteed unit because y_hat and z_hat are
    // orthonormal unit vectors.
    let x_hat = cross(y_hat, z_hat);

    // Step 3: assemble rotation matrix with basis vectors as rows.
    [x_hat, y_hat, z_hat]
}

/// Convert two inertial state vectors into a relative state expressed in the
/// rendezvous LVLH frame of the target vehicle.
///
/// # Preconditions
/// - All same-frame preconditions as `lvlh_matrix` apply to the target state.
/// - Both state vectors must be expressed in the same inertial frame (ECI or MCI).
///   The caller is responsible for frame consistency; this function does not check.
///
/// # Postconditions
/// - `result.rho`     == M_I2L * (r_active - r_target)
/// - `result.rho_dot` == M_I2L * (v_active - v_target)
///
/// # Note
/// The LVLH frame rotates with the target's orbit; consequently `rho_dot` is NOT
/// the time derivative of `rho` in an inertial sense. It is the velocity of the
/// active vehicle relative to the target in the LVLH frame at the instant of
/// evaluation. Coriolis and centrifugal terms appear only when propagating `rho`
/// forward (Hill equations); this function evaluates the instantaneous snapshot only.
///
/// Spec: rendezvous-spec.md §6.2
pub fn relative_state_lvlh(
    r_active: Vec3,
    v_active: Vec3,
    r_target: Vec3,
    v_target: Vec3,
) -> LvlhState {
    // Step 1: LVLH rotation matrix.
    let m = lvlh_matrix(r_target, v_target);

    // Step 2–3: inertial relative position and velocity.
    let rho_inertial = vsub(r_active, r_target);
    let rhodot_inertial = vsub(v_active, v_target);

    // Step 4: rotate into LVLH frame.
    let rho = mxv(m, rho_inertial);
    let rho_dot = mxv(m, rhodot_inertial);

    LvlhState { rho, rho_dot }
}

/// Scalar range (distance) between the active and target vehicles.
///
/// # Definition
///   range = |r_active - r_target|  (m)
///
/// # Preconditions
/// - None beyond finite, non-NaN inputs.
///
/// # Postconditions
/// - result >= 0.0
/// - result == 0.0 only when both vehicles are at the same inertial position.
///
/// # Note
/// This function does NOT require the LVLH frame to be constructed; it operates
/// directly on inertial vectors and is faster than extracting `norm(rho)` from
/// `relative_state_lvlh`. Callers that need only range should prefer this function.
///
/// Spec: rendezvous-spec.md §6.3
pub fn range(r_active: Vec3, r_target: Vec3) -> f64 {
    norm(vsub(r_active, r_target))
}

/// Time derivative of range: the scalar rate at which the active vehicle is moving
/// away from (or toward) the target vehicle.
///
/// # Definition
///   range_rate = dot(rho_vec, rho_dot_vec) / |rho_vec|   (m/s)
///
///   where rho_vec     = r_active - r_target   (inertial relative position)
///         rho_dot_vec = v_active - v_target   (inertial relative velocity)
///
/// # Sign convention
///   Positive  => vehicles are separating (range increasing).
///   Negative  => vehicles are closing    (range decreasing).
///
/// # Preconditions
/// - `range(r_active, r_target)` must be > 0.0.  Panics if zero (see §8).
///
/// # Postconditions
/// - result == 0.0 when rho_dot is perpendicular to rho (instantaneous closest
///   approach or instantaneous farthest point).
/// - |result| <= norm(rho_dot_vec)  (Cauchy–Schwarz).
///
/// # AGC correspondence
/// This computation maps to the `RDOT` variable maintained by the R32 routine
/// (O'Brien p. 316).  The AGC stored `RDOT` at scale B+7 m/s in erasable.
///
/// Spec: rendezvous-spec.md §6.4
pub fn range_rate(r_active: Vec3, v_active: Vec3, r_target: Vec3, v_target: Vec3) -> f64 {
    let rho = vsub(r_active, r_target);
    let rho_dot = vsub(v_active, v_target);
    let rng = norm(rho);
    assert!(
        rng != 0.0,
        "range_rate: vehicles are at the same inertial position (range == 0)"
    );
    dot(rho, rho_dot) / rng
}

/// Compute the line-of-sight elevation and azimuth from the active vehicle to
/// the target, expressed in the rendezvous LVLH frame of the target.
///
/// # Definition
///   Let rho_lvlh = [x, y, z] be the relative position in the LVLH frame.
///   horizontal_range = sqrt(x^2 + y^2)
///   elevation = atan2(-z_lvlh, horizontal_range)
///               (positive when target is above local horizontal, i.e. z_lvlh < 0)
///   azimuth   = atan2(y_lvlh, x_lvlh)
///               (0 when target is directly ahead along-track)
///
/// # Preconditions
/// - `lvlh.rho` must be non-zero; panics if `norm(lvlh.rho) == 0.0` (zero range).
///
/// # Postconditions
/// - elevation ∈ [-π/2, +π/2]
/// - azimuth   ∈ (-π,   +π]
///
/// # Degenerate case
/// When the target is directly overhead (x == 0, y == 0, z < 0) elevation = +π/2,
/// azimuth = 0 (defined by convention; azimuth is undefined but 0 is returned).
/// When the target is directly below (x == 0, y == 0, z > 0) elevation = -π/2,
/// azimuth = 0.
///
/// # AGC correspondence
/// Maps to the LOS angle computation in R33 (O'Brien pp. 329–330).  The AGC
/// display used shaft and trunnion angles of the CM optics; this function returns
/// the equivalent geometric angles without the optics CDU encoding.
///
/// Spec: rendezvous-spec.md §6.5
pub fn los_angles_lvlh(lvlh: &LvlhState) -> LosAngles {
    let [x, y, z] = lvlh.rho;
    assert!(
        norm(lvlh.rho) != 0.0,
        "los_angles_lvlh: relative position (rho) is zero; LOS angles undefined"
    );
    let horizontal_range = libm::sqrt(x * x + y * y);
    let elevation = atan2(-z, horizontal_range);
    let azimuth = atan2(y, x);
    LosAngles { elevation, azimuth }
}

/// Approximate time to closest approach (TCA) by linear extrapolation of the
/// current relative position and velocity.
///
/// # Definition
///   TCA = -dot(rho_vec, rho_dot_vec) / dot(rho_dot_vec, rho_dot_vec)   (s)
///
///   where rho_vec and rho_dot_vec are the inertial relative position and velocity.
///   This is the exact solution for constant-velocity (unforced) relative motion
///   and the first-order approximation for orbital relative motion.
///
/// # Sign convention
///   TCA > 0  => closest approach is in the future.
///   TCA < 0  => closest approach was in the past (vehicles are already diverging).
///   TCA = 0  => currently at closest approach (range_rate == 0).
///
/// # Preconditions
/// - `dot(rho_dot_vec, rho_dot_vec)` must be > 0.0; i.e. relative velocity is
///   non-zero.  Panics if relative velocity is zero (see §8).
///
/// # Postconditions
/// - When `range_rate(r_active, v_active, r_target, v_target) == 0.0`,
///   `time_to_closest_approach` returns 0.0.
/// - When the active vehicle is on a pure closing trajectory (range_rate < 0) and
///   no orbital curvature, TCA > 0.
///
/// # Limitation
/// The linear approximation degrades for TCA values longer than roughly one orbital
/// period. It is accurate to within ~1 % for typical Apollo rendezvous geometries
/// (range < 50 km, TCA < 30 min).  Callers requiring high-fidelity TCA for
/// longer horizons should propagate the state vectors with `math::kepler::kepler_step`
/// and iterate.
///
/// # AGC correspondence
/// The R34 display routine computed a "time of intercept" using this same linear
/// formula on the relative velocity vector (O'Brien p. 332).
///
/// Spec: rendezvous-spec.md §6.6
pub fn time_to_closest_approach(
    r_active: Vec3,
    v_active: Vec3,
    r_target: Vec3,
    v_target: Vec3,
) -> f64 {
    let rho = vsub(r_active, r_target);
    let rho_dot = vsub(v_active, v_target);
    let speed2 = dot(rho_dot, rho_dot);
    assert!(
        speed2 != 0.0,
        "time_to_closest_approach: relative velocity is zero; TCA is undefined"
    );
    -dot(rho, rho_dot) / speed2
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::linalg;

    /// Earth gravitational parameter (m³/s²), matching AGC and existing test conventions.
    const MU_EARTH: f64 = 3.986_004_418e14;

    /// Mean Earth radius used for altitude-based test setups (m).
    const R_E: f64 = 6_371_000.0;

    // ── Helper: assert two Vec3 values are component-wise within `eps` ────────

    fn assert_vec_near(a: Vec3, b: Vec3, eps: f64, label: &str) {
        for i in 0..3 {
            assert!(
                (a[i] - b[i]).abs() < eps,
                "{label} component {i}: got {}, expected {} (eps={eps})",
                a[i],
                b[i]
            );
        }
    }

    // ── Helper: assert Mat3x3 component-wise within `eps` ────────────────────

    fn assert_mat_near(m: Mat3x3, expected: Mat3x3, eps: f64, label: &str) {
        for r in 0..3 {
            for c in 0..3 {
                assert!(
                    (m[r][c] - expected[r][c]).abs() < eps,
                    "{label} [{r}][{c}]: got {}, expected {} (eps={eps})",
                    m[r][c],
                    expected[r][c]
                );
            }
        }
    }

    // ── TC-REND-1: relative_state_lvlh circular orbit baseline ───────────────

    /// TC-REND-1: Active vehicle 1 km radially outside target in circular LEO;
    /// LVLH relative position must be [0, 0, -1000] and velocity [0, 0, 0].
    #[test]
    fn tc_rend_1_relative_state_lvlh_circular_baseline() {
        let r_orbit = R_E + 300e3;
        let v_circ = libm::sqrt(MU_EARTH / r_orbit);

        let r_t: Vec3 = [r_orbit, 0.0, 0.0];
        let v_t: Vec3 = [0.0, v_circ, 0.0];

        // Active vehicle 1 km farther from Earth (radially outside target).
        let r_a: Vec3 = [r_orbit + 1000.0, 0.0, 0.0];
        let v_a: Vec3 = v_t; // same velocity

        let state = relative_state_lvlh(r_a, v_a, r_t, v_t);

        // z_hat points toward Earth (inward), so +1000 m in inertial X (outward)
        // maps to -1000 m in the LVLH z component.
        assert_vec_near(state.rho, [0.0, 0.0, -1000.0], 1.0, "TC-REND-1 rho");
        assert_vec_near(state.rho_dot, [0.0, 0.0, 0.0], 0.001, "TC-REND-1 rho_dot");
    }

    // ── TC-REND-2: range and range_rate for 2 km in-track offset ─────────────

    /// TC-REND-2: 2 km in-track offset from circular LEO target;
    /// range ≈ 2000 m, range_rate == 0 (same velocity).
    #[test]
    fn tc_rend_2_range_and_range_rate_in_track() {
        let r_orbit = R_E + 300e3;
        let v_circ = libm::sqrt(MU_EARTH / r_orbit);

        let r_t: Vec3 = [r_orbit, 0.0, 0.0];
        let v_t: Vec3 = [0.0, v_circ, 0.0];

        // "Better setup" from spec: active vehicle 2 km ahead in the Y direction.
        let r_a: Vec3 = [r_orbit, 2000.0, 0.0];
        let v_a: Vec3 = v_t;

        let rng = range(r_a, r_t);
        let rdot = range_rate(r_a, v_a, r_t, v_t);

        assert!(
            (rng - 2000.0).abs() < 1.0,
            "TC-REND-2: range = {} m, expected ≈ 2000 m",
            rng
        );
        assert!(
            rdot.abs() < 1e-9,
            "TC-REND-2: range_rate = {} m/s, expected 0.0",
            rdot
        );
    }

    // ── TC-REND-3: closing approach — range_rate sign ─────────────────────────

    /// TC-REND-3: Active vehicle 10 km outside target radially with −20 m/s
    /// radial velocity; range_rate must be ≈ −20 m/s (closing).
    #[test]
    fn tc_rend_3_range_rate_closing_sign() {
        let r_t: Vec3 = [7_000_000.0, 0.0, 0.0];
        let v_t: Vec3 = [0.0, 7500.0, 0.0];
        let r_a: Vec3 = [7_010_000.0, 0.0, 0.0]; // 10 km radially outside
        let v_a: Vec3 = [-20.0, 7500.0, 0.0];    // −20 m/s radially inward

        let rdot = range_rate(r_a, v_a, r_t, v_t);

        assert!(
            (rdot - (-20.0)).abs() < 0.01,
            "TC-REND-3: range_rate = {} m/s, expected ≈ -20.0 m/s",
            rdot
        );
    }

    // ── TC-REND-4: LOS angles — target directly ahead (along-track) ──────────

    /// TC-REND-4: Target 5000 m directly ahead (+x LVLH); elevation and azimuth
    /// must both be 0.
    #[test]
    fn tc_rend_4_los_angles_directly_ahead() {
        let lvlh = LvlhState {
            rho: [5000.0, 0.0, 0.0],
            rho_dot: [0.0, 0.0, 0.0],
        };

        let los = los_angles_lvlh(&lvlh);

        assert!(
            los.elevation.abs() < 1e-12,
            "TC-REND-4: elevation = {} rad, expected 0.0",
            los.elevation
        );
        assert!(
            los.azimuth.abs() < 1e-12,
            "TC-REND-4: azimuth = {} rad, expected 0.0",
            los.azimuth
        );
    }

    // ── TC-REND-5: LOS angles — target directly overhead (radial) ────────────

    /// TC-REND-5: Target 3000 m directly overhead (z_lvlh = −3000, i.e. farther
    /// from Earth); elevation must be π/2, azimuth = 0.
    #[test]
    fn tc_rend_5_los_angles_directly_overhead() {
        let lvlh = LvlhState {
            rho: [0.0, 0.0, -3000.0],
            rho_dot: [0.0, 0.0, 0.0],
        };

        let los = los_angles_lvlh(&lvlh);

        let pi_half = core::f64::consts::FRAC_PI_2;
        assert!(
            (los.elevation - pi_half).abs() < 1e-12,
            "TC-REND-5: elevation = {} rad, expected π/2 = {}",
            los.elevation,
            pi_half
        );
        assert!(
            los.azimuth.abs() < 1e-12,
            "TC-REND-5: azimuth = {} rad, expected 0.0",
            los.azimuth
        );
    }

    // ── TC-REND-6: LOS angles — 45° starboard and below ──────────────────────

    /// TC-REND-6: Equal x/y/z LVLH components (z > 0 → target below horizontal);
    /// elevation ≈ −0.6155 rad, azimuth = π/4.
    #[test]
    fn tc_rend_6_los_angles_45_starboard_below() {
        let lvlh = LvlhState {
            rho: [1000.0, 1000.0, 1000.0],
            rho_dot: [0.0, 0.0, 0.0],
        };

        let los = los_angles_lvlh(&lvlh);

        // elevation = atan2(-1000, sqrt(1000^2 + 1000^2))
        let expected_el =
            libm::atan2(-1000.0_f64, libm::sqrt(1000.0_f64 * 1000.0 + 1000.0_f64 * 1000.0));
        // azimuth = atan2(1000, 1000) = π/4
        let expected_az = core::f64::consts::FRAC_PI_4;

        assert!(
            (los.elevation - expected_el).abs() < 1e-9,
            "TC-REND-6: elevation = {} rad, expected {} rad",
            los.elevation,
            expected_el
        );
        assert!(
            (los.azimuth - expected_az).abs() < 1e-9,
            "TC-REND-6: azimuth = {} rad, expected {} rad",
            los.azimuth,
            expected_az
        );
    }

    // ── TC-REND-7: TCA — future intercept ────────────────────────────────────

    /// TC-REND-7: 10 km ahead in-track, closing at 10 m/s; TCA must be 1000 s.
    #[test]
    fn tc_rend_7_tca_future_intercept() {
        let r_t: Vec3 = [7_000_000.0, 0.0, 0.0];
        let v_t: Vec3 = [0.0, 7500.0, 0.0];
        let r_a: Vec3 = [7_000_000.0, 10_000.0, 0.0]; // 10 km ahead in-track
        let v_a: Vec3 = [0.0, 7490.0, 0.0];            // 10 m/s slower → closing

        let tca = time_to_closest_approach(r_a, v_a, r_t, v_t);

        assert!(
            (tca - 1000.0).abs() < 0.01,
            "TC-REND-7: TCA = {} s, expected 1000.0 s",
            tca
        );
    }

    // ── TC-REND-8: TCA — zero relative velocity panics ───────────────────────

    /// TC-REND-8: Identical velocities make relative speed zero; TCA must panic.
    #[test]
    #[should_panic(expected = "relative velocity is zero")]
    fn tc_rend_8_tca_zero_relative_velocity_panics() {
        let r_t: Vec3 = [7_000_000.0, 0.0, 0.0];
        let v_t: Vec3 = [0.0, 7500.0, 0.0];
        let r_a: Vec3 = [r_t[0] + 100.0, 0.0, 0.0]; // displaced, but same velocity
        let v_a: Vec3 = v_t;

        let _ = time_to_closest_approach(r_a, v_a, r_t, v_t);
    }

    // ── TC-REND-9: lvlh_matrix orthonormality ────────────────────────────────

    /// TC-REND-9: M * M^T must equal the identity matrix to within 1e-12.
    #[test]
    fn tc_rend_9_lvlh_matrix_orthonormal() {
        let r_orbit = R_E + 300e3;
        let v_circ = libm::sqrt(MU_EARTH / r_orbit);

        let r_t: Vec3 = [r_orbit, 0.0, 0.0];
        let v_t: Vec3 = [0.0, v_circ, 0.0];

        let m = lvlh_matrix(r_t, v_t);
        let mt = linalg::transpose(m);
        let mmt = linalg::mxm(m, mt);

        assert_mat_near(mmt, linalg::IDENTITY, 1e-12, "TC-REND-9 M*M^T");
    }

    // ── TC-REND-10: norm(rho_lvlh) == range(r_a, r_t) ───────────────────────

    /// TC-REND-10: Rotation preserves magnitude; LVLH rho norm must equal
    /// the direct inertial range to within 1e-6 m.
    #[test]
    fn tc_rend_10_lvlh_range_equals_inertial_range() {
        // Use TC-REND-3 inputs.
        let r_t: Vec3 = [7_000_000.0, 0.0, 0.0];
        let v_t: Vec3 = [0.0, 7500.0, 0.0];
        let r_a: Vec3 = [7_010_000.0, 0.0, 0.0];
        let v_a: Vec3 = [-20.0, 7500.0, 0.0];

        let lvlh_state = relative_state_lvlh(r_a, v_a, r_t, v_t);
        let rho_norm = linalg::norm(lvlh_state.rho);
        let inertial_range = range(r_a, r_t);

        assert!(
            (rho_norm - inertial_range).abs() < 1e-6,
            "TC-REND-10: norm(rho_lvlh) = {} m, range(r_a, r_t) = {} m, diff = {}",
            rho_norm,
            inertial_range,
            (rho_norm - inertial_range).abs()
        );
    }

    // ── TC-REND-I1: lvlh_matrix maps v_target to a pure +x vector ────────────

    /// TC-REND-I1: For a prograde equatorial circular orbit, M * v_target must
    /// have zero y and z components (velocity lies entirely along the x-hat axis).
    #[test]
    fn tc_rend_i1_lvlh_matrix_velocity_maps_to_x() {
        let r_orbit = R_E + 300e3;
        let v_circ = libm::sqrt(MU_EARTH / r_orbit);

        let r_t: Vec3 = [r_orbit, 0.0, 0.0];
        let v_t: Vec3 = [0.0, v_circ, 0.0];

        let m = lvlh_matrix(r_t, v_t);
        let v_in_lvlh = linalg::mxv(m, v_t);

        // x component must be positive (prograde = along-track).
        assert!(
            v_in_lvlh[0] > 0.0,
            "TC-REND-I1: v_lvlh[x] = {} must be positive",
            v_in_lvlh[0]
        );
        // y and z components must be near zero.
        assert!(
            v_in_lvlh[1].abs() < 1e-9,
            "TC-REND-I1: v_lvlh[y] = {} must be ~0",
            v_in_lvlh[1]
        );
        assert!(
            v_in_lvlh[2].abs() < 1e-9,
            "TC-REND-I1: v_lvlh[z] = {} must be ~0",
            v_in_lvlh[2]
        );
    }

    // ── TC-REND-I2: range_rate sign flips when approach direction flips ───────

    /// TC-REND-I2: Reversing the radial component of the active vehicle's
    /// velocity (TC-REND-3 geometry) must flip the sign of range_rate.
    #[test]
    fn tc_rend_i2_range_rate_sign_consistency() {
        let r_t: Vec3 = [7_000_000.0, 0.0, 0.0];
        let v_t: Vec3 = [0.0, 7500.0, 0.0];
        let r_a: Vec3 = [7_010_000.0, 0.0, 0.0]; // 10 km outside

        // TC-REND-3 original: closing at −20 m/s radially.
        let v_a_closing: Vec3 = [-20.0, 7500.0, 0.0];
        // Flipped: separating at +20 m/s radially.
        let v_a_separating: Vec3 = [20.0, 7500.0, 0.0];

        let rdot_closing = range_rate(r_a, v_a_closing, r_t, v_t);
        let rdot_separating = range_rate(r_a, v_a_separating, r_t, v_t);

        assert!(
            rdot_closing < 0.0,
            "TC-REND-I2: closing range_rate = {} must be negative",
            rdot_closing
        );
        assert!(
            rdot_separating > 0.0,
            "TC-REND-I2: separating range_rate = {} must be positive",
            rdot_separating
        );
        assert!(
            (rdot_closing + rdot_separating).abs() < 1e-9,
            "TC-REND-I2: range rates must be exact negatives, got {} and {}",
            rdot_closing,
            rdot_separating
        );
    }
}
