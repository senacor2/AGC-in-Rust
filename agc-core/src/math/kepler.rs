//! Kepler equation solvers and universal variable formulation.
//!
//! Implements the conic propagation routines that replace the AGC's
//! KEPSILON interpretive subroutine (Comanche055/CONIC_SUBROUTINES.agc,
//! routines KEPRTN and KEPSILON).
//!
//! Uses the universal-variable (Battin) formulation with Stumpff functions
//! C(z) and S(z), valid for elliptic, parabolic, and hyperbolic orbits
//! without requiring orbit-type classification by the caller.

use crate::math::linalg::{dot, norm, vadd, vscale};
use crate::types::Vec3;

/// Stumpff C function.
///
/// C(z) = (1 − cos(√z)) / z        for z > 0   (elliptic)
/// C(z) = 1/2                       for z = 0   (parabolic)
/// C(z) = (cosh(√(−z)) − 1) / (−z) for z < 0   (hyperbolic)
///
/// Uses a Taylor series for |z| < 1e-6 to avoid catastrophic cancellation.
pub fn stumpff_c(z: f64) -> f64 {
    if z.abs() < 1e-6 {
        // Taylor series: 1/2 − z/24 + z²/720 − z³/40320
        0.5 - z / 24.0 + z * z / 720.0 - z * z * z / 40320.0
    } else if z > 0.0 {
        let sq = libm::sqrt(z);
        (1.0 - libm::cos(sq)) / z
    } else {
        // z < 0: hyperbolic branch
        let sq = libm::sqrt(-z);
        (libm::cosh(sq) - 1.0) / (-z)
    }
}

/// Stumpff S function.
///
/// S(z) = (√z − sin(√z)) / (√z)³              for z > 0   (elliptic)
/// S(z) = 1/6                                   for z = 0   (parabolic)
/// S(z) = (sinh(√(−z)) − √(−z)) / (√(−z))³    for z < 0   (hyperbolic)
///
/// Uses a Taylor series for |z| < 1e-6 to avoid catastrophic cancellation.
pub fn stumpff_s(z: f64) -> f64 {
    if z.abs() < 1e-6 {
        // Taylor series: 1/6 − z/120 + z²/5040 − z³/362880
        1.0 / 6.0 - z / 120.0 + z * z / 5040.0 - z * z * z / 362880.0
    } else if z > 0.0 {
        let sq = libm::sqrt(z);
        (sq - libm::sin(sq)) / (sq * sq * sq)
    } else {
        // z < 0: hyperbolic branch
        let sq = libm::sqrt(-z);
        (libm::sinh(sq) - sq) / (sq * sq * sq)
    }
}

/// Propagate a state vector by time `dt` seconds under a central body
/// with gravitational parameter `mu` (m³/s²).
///
/// Uses the universal variable (Battin) method, valid for all conic sections.
/// Returns `(position_m, velocity_m_s)` at time `t0 + dt`.
///
/// # Parameters
/// - `r0`: Initial position vector (m), inertial frame centred on the central body.
/// - `v0`: Initial velocity vector (m/s), consistent frame with `r0`.
/// - `dt`: Propagation interval (s). Must be positive and finite.
/// - `mu`: Gravitational parameter of the central body (m³/s²).
///
/// # Returns
/// `(r1, v1)` — position (m) and velocity (m/s) at time `t0 + dt`.
///
/// # Panics
/// Panics if the Newton-Raphson iteration does not converge within 50 steps.
/// In debug builds also panics on invalid preconditions (zero radius, non-finite
/// inputs, non-positive mu).
///
/// AGC source: Comanche055/CONIC_SUBROUTINES.agc, routines KEPRTN / KEPSILON.
pub fn kepler_step(r0: Vec3, v0: Vec3, dt: f64, mu: f64) -> (Vec3, Vec3) {
    debug_assert!(dt.is_finite() && dt != 0.0, "kepler_step: dt must be finite and non-zero");
    debug_assert!(mu > 0.0 && mu.is_finite(), "kepler_step: mu must be positive and finite");
    let r0_mag = norm(r0);
    debug_assert!(r0_mag > 0.0 && r0_mag.is_finite(), "kepler_step: r0 must be non-zero and finite");

    // ── Pre-computation ──────────────────────────────────────────────────────
    let v0_sq = dot(v0, v0);
    let r0_dot_v0 = dot(r0, v0);
    let alpha = 2.0 / r0_mag - v0_sq / mu; // = 1/a  (m⁻¹); negative if hyperbolic
    let sqrt_mu = libm::sqrt(mu);

    // ── Initial estimate for χ ───────────────────────────────────────────────
    // χ₀ = √μ · dt / r0   (acceptable for all conic types; see spec §3.4)
    let mut chi = sqrt_mu * dt / r0_mag;

    // ── Newton-Raphson iteration (max 50 iterations) ─────────────────────────
    const MAX_ITER: usize = 50;
    let mut converged = false;

    for _ in 0..MAX_ITER {
        let z = alpha * chi * chi;
        let c = stumpff_c(z);
        let s = stumpff_s(z);

        // Universal Kepler equation residual ψ
        let psi = (r0_dot_v0 / sqrt_mu) * chi * chi * c
            + (1.0 - r0_mag * alpha) * chi * chi * chi * s
            + r0_mag * chi
            - sqrt_mu * dt;

        // dψ/dχ  =  r(χ)  (the instantaneous radius at universal anomaly χ)
        let r_now = (r0_dot_v0 / sqrt_mu) * chi * (1.0 - z * s)
            + (1.0 - r0_mag * alpha) * chi * chi * c
            + r0_mag;

        let dpsi_dchi = r_now;
        let delta = psi / dpsi_dchi;

        chi -= delta;

        // Convergence criterion: |Δχ| < 1e-9·|χ| + 1e-12
        if delta.abs() < 1e-9 * chi.abs() + 1e-12 {
            converged = true;
            break;
        }
    }

    if !converged {
        panic!("kepler_step: Newton-Raphson did not converge");
    }

    // ── Post-iteration: Lagrange coefficients and propagated state ───────────
    let z = alpha * chi * chi;
    let c = stumpff_c(z);
    let s = stumpff_s(z);

    let r1_mag = (r0_dot_v0 / sqrt_mu) * chi * (1.0 - z * s)
        + (1.0 - r0_mag * alpha) * chi * chi * c
        + r0_mag;

    let f = 1.0 - (chi * chi / r0_mag) * c;
    let g = dt - (chi * chi * chi / sqrt_mu) * s;
    let f_dot = (sqrt_mu / (r0_mag * r1_mag)) * chi * (z * s - 1.0);
    let g_dot = 1.0 - (chi * chi / r1_mag) * c;

    let r1 = vadd(vscale(r0, f), vscale(v0, g));
    let v1 = vadd(vscale(r0, f_dot), vscale(v0, g_dot));

    (r1, v1)
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::linalg::{cross, norm};

    const MU_EARTH: f64 = 3.986_004_418e14;
    const MU_MOON: f64 = 4.902_800_118e12;

    // Helper: compute circular orbit speed at radius r.
    fn v_circ(r: f64, mu: f64) -> f64 {
        libm::sqrt(mu / r)
    }

    // Helper: assert two f64 values agree to a relative tolerance.
    fn assert_near_rel(a: f64, b: f64, rel_tol: f64, label: &str) {
        let denom = b.abs().max(1e-30);
        let rel = (a - b).abs() / denom;
        assert!(
            rel < rel_tol,
            "{label}: |{a} − {b}| / {b} = {rel:.3e} exceeds {rel_tol:.3e}"
        );
    }

    // Helper: assert norm of difference vector is below tolerance.
    fn assert_pos_near(actual: Vec3, expected: Vec3, tol_m: f64, label: &str) {
        let diff = [
            actual[0] - expected[0],
            actual[1] - expected[1],
            actual[2] - expected[2],
        ];
        let err = norm(diff);
        assert!(
            err < tol_m,
            "{label}: position error {err:.3e} m exceeds {tol_m:.3e} m"
        );
    }

    fn assert_vel_near(actual: Vec3, expected: Vec3, tol_ms: f64, label: &str) {
        let diff = [
            actual[0] - expected[0],
            actual[1] - expected[1],
            actual[2] - expected[2],
        ];
        let err = norm(diff);
        assert!(
            err < tol_ms,
            "{label}: velocity error {err:.3e} m/s exceeds {tol_ms:.3e} m/s"
        );
    }

    // ── TC-KEP-7 / TC-STUMPFF-*: Stumpff function unit tests ─────────────────

    #[test]
    fn tc_stumpff_1_parabolic_z_zero() {
        // z = 0: C(0) = 0.5, S(0) = 1/6
        let c = stumpff_c(0.0);
        let s = stumpff_s(0.0);
        assert_near_rel(c, 0.5, 1e-14, "C(0)");
        assert_near_rel(s, 1.0 / 6.0, 1e-14, "S(0)");
    }

    #[test]
    fn tc_stumpff_2_elliptic_z_one() {
        // z = 1 (elliptic): C(1) = (1 − cos(1))/1, S(1) = (1 − sin(1))/1
        // Note: S(z>0) = (sqrt(z) − sin(sqrt(z))) / (sqrt(z))^3
        //   with sqrt(1)=1: (1 − sin(1))/1
        let c = stumpff_c(1.0);
        let s = stumpff_s(1.0);
        let expected_c = (1.0 - libm::cos(1.0)) / 1.0;
        let expected_s = (1.0 - libm::sin(1.0)) / 1.0;
        assert_near_rel(c, expected_c, 1e-14, "C(1)");
        assert_near_rel(s, expected_s, 1e-14, "S(1)");
    }

    #[test]
    fn tc_stumpff_3_hyperbolic_z_neg_one() {
        // z = −1 (hyperbolic): C(-1) = (cosh(1)−1)/1, S(-1) = (sinh(1)−1)/1
        let c = stumpff_c(-1.0);
        let s = stumpff_s(-1.0);
        let expected_c = (libm::cosh(1.0) - 1.0) / 1.0;
        let expected_s = (libm::sinh(1.0) - 1.0) / 1.0;
        assert_near_rel(c, expected_c, 1e-14, "C(-1)");
        assert_near_rel(s, expected_s, 1e-14, "S(-1)");
    }

    #[test]
    fn tc_stumpff_4_taylor_branch_positive() {
        // z = 1e-7 (positive Taylor branch)
        let z = 1e-7_f64;
        let c = stumpff_c(z);
        let s = stumpff_s(z);
        let expected_c = 0.5 - z / 24.0 + z * z / 720.0 - z * z * z / 40320.0;
        let expected_s = 1.0 / 6.0 - z / 120.0 + z * z / 5040.0 - z * z * z / 362880.0;
        assert_near_rel(c, expected_c, 1e-14, "C(1e-7) Taylor");
        assert_near_rel(s, expected_s, 1e-14, "S(1e-7) Taylor");
    }

    #[test]
    fn tc_stumpff_5_taylor_branch_negative() {
        // z = -1e-7 (negative Taylor branch)
        let z = -1e-7_f64;
        let c = stumpff_c(z);
        let s = stumpff_s(z);
        let expected_c = 0.5 - z / 24.0 + z * z / 720.0 - z * z * z / 40320.0;
        let expected_s = 1.0 / 6.0 - z / 120.0 + z * z / 5040.0 - z * z * z / 362880.0;
        assert_near_rel(c, expected_c, 1e-14, "C(-1e-7) Taylor");
        assert_near_rel(s, expected_s, 1e-14, "S(-1e-7) Taylor");
    }

    // ── TC-KEP-1: Circular LEO — Quarter Orbit ────────────────────────────────

    #[test]
    fn tc_kep_1_circular_leo_quarter_orbit() {
        let r_circ = 6_571_000.0_f64;
        let vc = v_circ(r_circ, MU_EARTH);
        let r0: Vec3 = [r_circ, 0.0, 0.0];
        let v0: Vec3 = [0.0, vc, 0.0];

        // Orbital period T = 2π √(r³/μ)
        let t_period = 2.0
            * core::f64::consts::PI
            * libm::sqrt(r_circ * r_circ * r_circ / MU_EARTH);
        let dt = t_period / 4.0;

        let (r1, v1) = kepler_step(r0, v0, dt, MU_EARTH);

        // After 90°: position should be [0, r_circ, 0], velocity [-vc, 0, 0]
        assert_pos_near(r1, [0.0, r_circ, 0.0], 1.0, "TC-KEP-1 position");
        assert_vel_near(v1, [-vc, 0.0, 0.0], 1e-3, "TC-KEP-1 velocity");
    }

    // ── TC-KEP-2: Circular LEO — Half Orbit ───────────────────────────────────

    #[test]
    fn tc_kep_2_circular_leo_half_orbit() {
        let r_circ = 6_571_000.0_f64;
        let vc = v_circ(r_circ, MU_EARTH);
        let r0: Vec3 = [r_circ, 0.0, 0.0];
        let v0: Vec3 = [0.0, vc, 0.0];

        let t_period = 2.0
            * core::f64::consts::PI
            * libm::sqrt(r_circ * r_circ * r_circ / MU_EARTH);
        let dt = t_period / 2.0;

        let (r1, v1) = kepler_step(r0, v0, dt, MU_EARTH);

        assert_pos_near(r1, [-r_circ, 0.0, 0.0], 1.0, "TC-KEP-2 position");
        assert_vel_near(v1, [0.0, -vc, 0.0], 1e-3, "TC-KEP-2 velocity");
    }

    // ── TC-KEP-3: Circular LEO — Full Orbit (Round-Trip) + invariants ─────────

    #[test]
    fn tc_kep_3_circular_leo_full_orbit() {
        let r_circ = 6_571_000.0_f64;
        let vc = v_circ(r_circ, MU_EARTH);
        let r0: Vec3 = [r_circ, 0.0, 0.0];
        let v0: Vec3 = [0.0, vc, 0.0];

        let t_period = 2.0
            * core::f64::consts::PI
            * libm::sqrt(r_circ * r_circ * r_circ / MU_EARTH);

        let (r1, v1) = kepler_step(r0, v0, t_period, MU_EARTH);

        // Round-trip: r1 ≈ r0, v1 ≈ v0
        assert_pos_near(r1, r0, 10.0, "TC-KEP-3 position round-trip");
        assert_vel_near(v1, v0, 1e-2, "TC-KEP-3 velocity round-trip");

        // Energy conservation: E = 0.5*|v|² - μ/r
        let e0 = 0.5 * dot(v0, v0) - MU_EARTH / norm(r0);
        let e1 = 0.5 * dot(v1, v1) - MU_EARTH / norm(r1);
        assert_near_rel(e1, e0, 1e-9, "TC-KEP-3 energy conservation");

        // Angular momentum conservation
        let h0 = norm(cross(r0, v0));
        let h1 = norm(cross(r1, v1));
        assert_near_rel(h1, h0, 1e-9, "TC-KEP-3 angular momentum conservation");
    }

    // ── TC-KEP-4: Highly Elliptic Transfer Orbit (Perigee → Apogee) ──────────

    #[test]
    fn tc_kep_4_highly_elliptic_transfer() {
        let r_p = 6_571_000.0_f64; // perigee radius (m)
        let r_a = 42_164_000.0_f64; // apogee radius (m)
        let a = (r_p + r_a) / 2.0; // semi-major axis (m)

        let v_p = libm::sqrt(MU_EARTH * (2.0 / r_p - 1.0 / a));
        let v_a = libm::sqrt(MU_EARTH * (2.0 / r_a - 1.0 / a));

        let r0: Vec3 = [r_p, 0.0, 0.0];
        let v0: Vec3 = [0.0, v_p, 0.0];

        // Half-period = perigee to apogee
        let t_half = core::f64::consts::PI * libm::sqrt(a * a * a / MU_EARTH);

        let (r1, v1) = kepler_step(r0, v0, t_half, MU_EARTH);

        // At apogee: position [-r_a, 0, 0], velocity [0, -v_a, 0]
        assert_pos_near(r1, [-r_a, 0.0, 0.0], 100.0, "TC-KEP-4 position");
        assert_vel_near(v1, [0.0, -v_a, 0.0], 0.01, "TC-KEP-4 velocity");
    }

    // ── TC-KEP-5: Lunar Orbit — Short Propagation Step ────────────────────────

    #[test]
    fn tc_kep_5_lunar_orbit_short_step() {
        let r_lo = 1_837_400.0_f64; // 100 km LLO altitude (m)
        let v_lo = v_circ(r_lo, MU_MOON);

        let r0: Vec3 = [r_lo, 0.0, 0.0];
        let v0: Vec3 = [0.0, v_lo, 0.0];
        let dt = 60.0_f64; // 1-minute step

        let (r1, v1) = kepler_step(r0, v0, dt, MU_MOON);

        // Analytic estimate for short arc of circular orbit
        let delta_angle = v_lo * dt / r_lo;
        let expected_r: Vec3 = [
            r_lo * libm::cos(delta_angle),
            r_lo * libm::sin(delta_angle),
            0.0,
        ];
        let expected_v: Vec3 = [
            -v_lo * libm::sin(delta_angle),
            v_lo * libm::cos(delta_angle),
            0.0,
        ];

        assert_pos_near(r1, expected_r, 0.1, "TC-KEP-5 position");
        assert_vel_near(v1, expected_v, 1e-4, "TC-KEP-5 velocity");
    }

    // ── TC-KEP-6: Short dt (2 s) — agreement with linear approximation ────────

    #[test]
    fn tc_kep_6_short_dt_linear_approx() {
        let r0: Vec3 = [6_571_000.0, 0.0, 0.0];
        let v0: Vec3 = [0.0, 7784.26, 0.0];
        let dt = 2.0_f64;

        let (r1, v1) = kepler_step(r0, v0, dt, MU_EARTH);

        // Linear (second-order) approximation:
        //   r1 ≈ r0 + v0*dt − 0.5*(μ/|r0|³)*r0*dt²
        let r0_mag = norm(r0);
        let accel_scale = MU_EARTH / (r0_mag * r0_mag * r0_mag);
        let expected_r: Vec3 = [
            r0[0] + v0[0] * dt - 0.5 * accel_scale * r0[0] * dt * dt,
            r0[1] + v0[1] * dt - 0.5 * accel_scale * r0[1] * dt * dt,
            r0[2] + v0[2] * dt - 0.5 * accel_scale * r0[2] * dt * dt,
        ];
        // Velocity approximation: v1 ≈ v0 − (μ/|r0|³)*r0*dt
        let expected_v: Vec3 = [
            v0[0] - accel_scale * r0[0] * dt,
            v0[1] - accel_scale * r0[1] * dt,
            v0[2] - accel_scale * r0[2] * dt,
        ];

        // Linear approx is second-order in dt; at dt=2s, the third-order error is
        // ~|v|·(accel_scale·dt²) ≈ 7800·(1.5e-6·4) ≈ 0.05 m position, 0.01 m/s vel.
        assert_pos_near(r1, expected_r, 0.1, "TC-KEP-6 position");
        assert_vel_near(v1, expected_v, 0.1, "TC-KEP-6 velocity");
    }
}
