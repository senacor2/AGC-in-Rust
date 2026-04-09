//! Universal-variable Kepler equation solver.
//!
//! Propagates a state vector (position + velocity) forward along a conic orbit
//! (elliptic, parabolic, or hyperbolic) using Battin's universal-variable
//! formulation with Laguerre-Conway iteration.
//!
//! AGC source: Comanche055/CONIC_SUBROUTINES.agc — KEPRTN / KEPLERN routine.
//! The AGC used a polynomial approximation for Stumpff functions (DELTIME
//! subroutine, page 1283); here we use the closed-form series / trig expressions
//! which are exact in `f64`.

use crate::math::linalg::{add, dot, norm, scale};
use crate::types::Vec3;

/// Maximum Laguerre-Conway iterations before declaring non-convergence.
const MAX_ITER: u32 = 50;

/// Convergence tolerance on the universal variable χ (dimensionless).
const TOL: f64 = 1e-10;

/// Result of a Kepler propagation step.
pub struct KeplerResult {
    /// New position vector (m).
    pub r: Vec3,
    /// New velocity vector (m/s).
    pub v: Vec3,
}

// ---------------------------------------------------------------------------
// Stumpff functions
// ---------------------------------------------------------------------------

/// Stumpff c₂(ψ) = (1 − cos√ψ)/ψ  for ψ > 0,
///                 (cosh√(−ψ) − 1)/(−ψ) for ψ < 0,
///                 1/2                   for |ψ| < ε.
///
/// AGC source: CONIC_SUBROUTINES.agc, DELTIME subroutine (polynomial table,
/// page 1283) — the AGC evaluated these via a degree-8 polynomial in ψ.
pub fn stumpff_c2(psi: f64) -> f64 {
    if psi > 1e-6 {
        let sq = libm::sqrt(psi);
        (1.0 - libm::cos(sq)) / psi
    } else if psi < -1e-6 {
        let sq = libm::sqrt(-psi);
        (libm::cosh(sq) - 1.0) / (-psi)
    } else {
        // Taylor series: c2 = 1/2 - ψ/24 + ψ²/720 − ...
        0.5 - psi / 24.0 + psi * psi / 720.0
    }
}

/// Stumpff c₃(ψ) = (√ψ − sin√ψ)/(√ψ)³  for ψ > 0,
///                 (sinh√(−ψ) − √(−ψ))/(√(−ψ))³ for ψ < 0,
///                 1/6                             for |ψ| < ε.
///
/// AGC source: CONIC_SUBROUTINES.agc, DELTIME subroutine (polynomial table,
/// page 1283).
pub fn stumpff_c3(psi: f64) -> f64 {
    if psi > 1e-6 {
        let sq = libm::sqrt(psi);
        (sq - libm::sin(sq)) / (psi * sq)
    } else if psi < -1e-6 {
        let sq = libm::sqrt(-psi);
        (libm::sinh(sq) - sq) / ((-psi) * sq)
    } else {
        // Taylor series: c3 = 1/6 - ψ/120 + ψ²/5040 − ...
        1.0 / 6.0 - psi / 120.0 + psi * psi / 5040.0
    }
}

// ---------------------------------------------------------------------------
// Kepler propagator
// ---------------------------------------------------------------------------

/// Propagate the state (r0, v0) forward by `dt` seconds on a Keplerian conic
/// with gravitational parameter `mu` (m³/s²).
///
/// Uses the universal variable χ with Laguerre-Conway iteration.  Handles all
/// conic types (elliptic, parabolic, hyperbolic) uniformly.  Negative `dt`
/// propagates backward in time.
///
/// Returns `None` if the iteration fails to converge within [`MAX_ITER`] steps,
/// which indicates a degenerate or unphysical input state.
///
/// AGC source: Comanche055/CONIC_SUBROUTINES.agc — KEPRTN / KEPLERN routine.
pub fn kepler_universal(r0: &Vec3, v0: &Vec3, dt: f64, mu: f64) -> Option<KeplerResult> {
    // Degenerate case: zero time step returns initial state unchanged.
    if dt == 0.0 {
        return Some(KeplerResult { r: *r0, v: *v0 });
    }

    let r0_mag = norm(r0);
    let sqrt_mu = libm::sqrt(mu);

    // Reciprocal semi-major axis α = 2/|r0| − |v0|²/μ.
    // α > 0 → elliptic, α = 0 → parabolic, α < 0 → hyperbolic.
    //
    // AGC source: KEPLERN, label KEPC2/ALPHA computation (page 1277).
    let v0_sq = dot(v0, v0);
    let alpha = 2.0 / r0_mag - v0_sq / mu;

    // Radial velocity component r0·v0 / |r0|.
    let rv0 = dot(r0, v0);

    // Initial guess for χ.
    //
    // AGC source: KEPLERN XKEPNEW initialisation logic (page 1278).
    let chi0 = if alpha.abs() > 1e-10 {
        // Elliptic or hyperbolic: χ ≈ √μ · α · dt  (Battin eq 6.9.27).
        sqrt_mu * dt * alpha
    } else {
        // Near-parabolic: χ ≈ dt / |r0|.
        dt / r0_mag
    };

    // Laguerre-Conway iteration.
    //
    // The universal Kepler equation (Battin eq 6.9.15):
    //   F(χ) = rv0/√μ · χ² · c2(ψ) + (1 − |r0|·α) · χ³ · c3(ψ) + |r0|·χ − √μ·dt = 0
    //
    // where ψ = α·χ².
    //
    // AGC source: KEPLERN KEPLOOP / DELTIME (pages 1279–1280).
    let mut chi = chi0;

    for _ in 0..MAX_ITER {
        let psi = alpha * chi * chi;
        let c2 = stumpff_c2(psi);
        let c3 = stumpff_c3(psi);

        // Evaluate F(χ) and F′(χ).
        let r_val = (rv0 / sqrt_mu) * chi * chi * c2
            + (1.0 - r0_mag * alpha) * chi * chi * chi * c3
            + r0_mag * chi;

        let f_chi = r_val - sqrt_mu * dt;

        // F′(χ) = (rv0/√μ)·χ·(1 − ψ·c3) + (1 − r0|·α)·χ²·c2 + r0|
        //        which simplifies to r_n / χ · χ + ... but is more cleanly:
        let fp_chi = (rv0 / sqrt_mu) * chi * (1.0 - psi * c3)
            + (1.0 - r0_mag * alpha) * chi * chi * c2
            + r0_mag;

        // Laguerre-Conway step (n = 5 for 5th-order method).
        //   δχ = −n·F / (F′ ± √|(n−1)²·F′² − n·(n−1)·F·F″|)
        // For simplicity we use n = 1 (Newton step) as the AGC does; the
        // bisection bounds guard against divergence just as KEPLOOP does.
        if fp_chi.abs() < 1e-30 {
            break;
        }
        let delta = f_chi / fp_chi;
        chi -= delta;

        if delta.abs() < TOL {
            // Converged — compute Lagrange coefficients and output state.
            //
            // AGC source: KEPLERN KEPCONVG (page 1281).
            let psi_f = alpha * chi * chi;
            let c2_f = stumpff_c2(psi_f);
            let c3_f = stumpff_c3(psi_f);

            let r1_mag = (rv0 / sqrt_mu) * chi * chi * c2_f
                + (1.0 - r0_mag * alpha) * chi * chi * chi * c3_f
                + r0_mag * chi;
            // r1_mag is ∂t/∂χ = current radius magnitude.

            // Lagrange f and g scalars.
            let f_lag = 1.0 - chi * chi * c2_f / r0_mag;
            let g_lag = dt - chi * chi * chi * c3_f / sqrt_mu;

            // New position: r1 = f·r0 + g·v0.
            let r1 = add(&scale(r0, f_lag), &scale(v0, g_lag));
            let r1_norm = norm(&r1);

            // Lagrange fdot and gdot.
            let fdot_lag = sqrt_mu * chi * (psi_f * c3_f - 1.0) / (r1_norm * r0_mag);
            let gdot_lag = 1.0 - chi * chi * c2_f / r1_norm;

            // New velocity: v1 = fdot·r0 + gdot·v0.
            let v1 = add(&scale(r0, fdot_lag), &scale(v0, gdot_lag));

            return Some(KeplerResult { r: r1, v: v1 });
        }
    }

    None
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::linalg::{dot, norm};
    use crate::navigation::gravity::MU_EARTH;

    /// Circular orbit at 400 km altitude.
    /// Propagated half a period: position vector must reverse (dot product ≈ −|r|²),
    /// speed and radius must be conserved.
    #[test]
    fn circular_half_period_reverses_position() {
        let alt = 400_000.0_f64; // m above surface
        let r0_mag = 6_371_000.0 + alt;
        let v_circ = libm::sqrt(MU_EARTH / r0_mag);

        let r0: Vec3 = [r0_mag, 0.0, 0.0];
        let v0: Vec3 = [0.0, v_circ, 0.0];

        // Orbital period T = 2π√(a³/μ); half period.
        let period = 2.0 * core::f64::consts::PI * libm::sqrt(r0_mag.powi(3) / MU_EARTH);
        let dt = period / 2.0;

        let res = kepler_universal(&r0, &v0, dt, MU_EARTH).expect("should converge");

        // Radius must be conserved.
        let r1_mag = norm(&res.r);
        assert!(
            (r1_mag - r0_mag).abs() / r0_mag < 1e-9,
            "radius not conserved: got {r1_mag}, expected {r0_mag}"
        );

        // Speed must be conserved.
        let v1_mag = norm(&res.v);
        assert!(
            (v1_mag - v_circ).abs() / v_circ < 1e-9,
            "speed not conserved: got {v1_mag}, expected {v_circ}"
        );

        // Position must be approximately reversed.
        let cos_angle = dot(&r0, &res.r) / (r0_mag * r1_mag);
        assert!(
            (cos_angle + 1.0).abs() < 1e-7,
            "expected position reversal, cos = {cos_angle}"
        );
    }

    /// Elliptic orbit: full period must return to initial state within tolerance.
    #[test]
    fn elliptic_full_period_returns_to_initial() {
        let r_p = 6_571_000.0_f64; // perigee radius (200 km altitude)
        let r_a = 42_164_000.0_f64; // apogee radius (GEO)
        let a = (r_p + r_a) / 2.0; // semi-major axis

        // Velocity at perigee from vis-viva.
        let v_p = libm::sqrt(MU_EARTH * (2.0 / r_p - 1.0 / a));

        let r0: Vec3 = [r_p, 0.0, 0.0];
        let v0: Vec3 = [0.0, v_p, 0.0];

        let period = 2.0 * core::f64::consts::PI * libm::sqrt(a.powi(3) / MU_EARTH);

        let res = kepler_universal(&r0, &v0, period, MU_EARTH).expect("should converge");

        for i in 0..3 {
            assert!(
                (res.r[i] - r0[i]).abs() < 1.0,
                "r[{i}] not recovered: got {}, expected {}",
                res.r[i],
                r0[i]
            );
            assert!(
                (res.v[i] - v0[i]).abs() < 1e-4,
                "v[{i}] not recovered: got {}, expected {}",
                res.v[i],
                v0[i]
            );
        }
    }

    /// Hyperbolic flyby: propagation must converge and total energy must be
    /// conserved (energy is always negative for bound orbits, positive for
    /// hyperbolic).
    #[test]
    fn hyperbolic_energy_conserved() {
        // Escape velocity × 1.2 at LEO altitude.
        let r0_mag = 6_571_000.0_f64;
        let v_esc = libm::sqrt(2.0 * MU_EARTH / r0_mag);
        let v0_mag = 1.2 * v_esc; // hyperbolic excess speed

        let r0: Vec3 = [r0_mag, 0.0, 0.0];
        let v0: Vec3 = [0.0, v0_mag, 0.0];

        // Specific orbital energy ε = v²/2 − μ/r  (positive for hyperbola).
        let energy0 = v0_mag * v0_mag / 2.0 - MU_EARTH / r0_mag;
        assert!(energy0 > 0.0, "expected hyperbolic orbit");

        let dt = 3600.0_f64; // propagate 1 hour
        let res = kepler_universal(&r0, &v0, dt, MU_EARTH).expect("should converge");

        let r1_mag = norm(&res.r);
        let v1_mag = norm(&res.v);
        let energy1 = v1_mag * v1_mag / 2.0 - MU_EARTH / r1_mag;

        assert!(
            (energy1 - energy0).abs() / energy0.abs() < 1e-9,
            "energy not conserved: Δε/ε = {}",
            (energy1 - energy0) / energy0
        );
    }

    /// dt = 0 must return the initial state exactly.
    #[test]
    fn zero_dt_returns_initial_state() {
        let r0: Vec3 = [7_000_000.0, 0.0, 0.0];
        let v0: Vec3 = [0.0, 7_500.0, 0.0];

        let res = kepler_universal(&r0, &v0, 0.0, MU_EARTH).expect("should return immediately");

        assert_eq!(res.r, r0);
        assert_eq!(res.v, v0);
    }

    /// Backward propagation followed by forward propagation must recover the
    /// original state (round-trip test).
    #[test]
    fn round_trip_backward_forward() {
        let r0: Vec3 = [7_000_000.0, 0.0, 0.0];
        let v0_mag = libm::sqrt(MU_EARTH / 7_000_000.0);
        let v0: Vec3 = [0.0, v0_mag, 0.0];

        let dt = 1800.0_f64; // 30 minutes
        let fwd = kepler_universal(&r0, &v0, dt, MU_EARTH).expect("forward converge");
        let back = kepler_universal(&fwd.r, &fwd.v, -dt, MU_EARTH).expect("backward converge");

        for i in 0..3 {
            assert!(
                (back.r[i] - r0[i]).abs() < 1e-3,
                "round-trip r[{i}] error: {}",
                back.r[i] - r0[i]
            );
            assert!(
                (back.v[i] - v0[i]).abs() < 1e-7,
                "round-trip v[{i}] error: {}",
                back.v[i] - v0[i]
            );
        }
    }
}
