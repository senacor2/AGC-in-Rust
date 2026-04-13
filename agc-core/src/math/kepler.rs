//! Universal-variable Kepler propagator (KEPLERN).
//!
//! Propagates a conic state vector by an arbitrary time `dt` using the
//! Battin/Goodyear universal-variable (χ) formulation.  Handles elliptic,
//! parabolic, and hyperbolic trajectories in a single code path.
//!
//! AGC source: Comanche055/CONIC_SUBROUTINES.agc
//!   KEPLERN (page 1277), DELTIME (page 1283), KEPPREP (page 1334).

use crate::math::linalg::{add, dot, norm, norm_sq, scale};
use crate::types::Vec3;

/// Maximum Newton iterations for the universal-variable loop.
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc` `SSP ITERCTR 20D` (page 1277).
pub const MAX_ITERATIONS: u32 = 20;

/// Absolute time convergence tolerance (seconds).
///
/// The AGC used `|TAU| * 2^-22` (BEE22); here we use the tighter of the two.
const KEPLER_TOL: f64 = 1e-12;

/// Result of a universal-variable Kepler propagation step.
///
/// If `converged` is false, `r` and `v` contain the best-available approximation
/// (the state at the last completed iteration) and must NOT be used as navigation
/// truth. The caller must invoke `alarm::raise(AlarmCode::KeplerDiverged)` and
/// enter the appropriate restart path.
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, KEPLERN (page 1277).
pub struct KeplerResult {
    /// Terminal position vector, ECI metres.
    pub r: Vec3,
    /// Terminal velocity vector, ECI m/s.
    pub v: Vec3,
    /// True if the Newton iteration converged within MAX_ITERATIONS.
    pub converged: bool,
    /// Number of Newton iterations actually performed.
    pub iterations: u32,
}

/// Propagate a conic state vector forward by `dt` seconds using the universal
/// variable (Battin/Goodyear) formulation.
///
/// # Arguments
/// - `r0`: initial position in ECI metres.
/// - `v0`: initial velocity in ECI m/s.
/// - `dt`: transfer time in seconds (positive = forward, negative = backward).
/// - `mu`: gravitational parameter in m³/s² (use `MU_EARTH` or `MU_MOON`).
///
/// # Returns
/// `KeplerResult` with terminal state and convergence flag. Never panics.
/// On non-convergence, returns `converged: false` with the last iterate.
///
/// # Invariants
/// - No heap allocation; no `unwrap`; no panic.
/// - Bounded to MAX_ITERATIONS Newton steps.
/// - Handles dt = 0 by returning the input state immediately (zero iterations).
/// - Handles dt > one orbital period via modulo reduction (TMODULO equivalent).
/// - Negative dt is fully supported (backward propagation).
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, KEPLERN routine (page 1277).
/// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc`, KEPPREP (initial guess, page 1334).
pub fn kepler(r0: &Vec3, v0: &Vec3, dt: f64, mu: f64) -> KeplerResult {
    // Singularity guards
    let r0_norm = norm(r0);
    if mu <= 0.0 || r0_norm < 1.0 {
        return KeplerResult {
            r: *r0,
            v: *v0,
            converged: false,
            iterations: 0,
        };
    }

    // dt = 0: identity
    if dt == 0.0 {
        return KeplerResult {
            r: *r0,
            v: *v0,
            converged: true,
            iterations: 0,
        };
    }

    let sqrt_mu = libm::sqrt(mu);

    // KEPLERN initialization
    //
    // sigma0 = dot(r0, v0) / sqrt(mu)  — scaled radial velocity (Battin σ₀)
    // Corresponds to AGC KEPC1 = dot(RTNAV, VNAV) / sqrt(MUE).
    // AGC source: Comanche055/CONIC_SUBROUTINES.agc, KEPLERN (page 1277).
    let sigma0 = dot(r0, v0) / sqrt_mu;

    // Reciprocal semi-major axis: alpha = 2/r0 - |v0|^2/mu = 1/a
    // Positive for elliptic, negative for hyperbolic, ~0 for parabolic.
    // AGC source: KEPC2 = |v0|^2 * r0 / mu - 1; alpha = (1 - KEPC2) / r0.
    let v0_sq = norm_sq(v0);
    let alpha = 2.0 / r0_norm - v0_sq / mu;

    // Time tolerance: max(KEPLER_TOL, |dt| * 2^-22)
    let epsilont = libm::fmax(KEPLER_TOL, libm::fabs(dt) * libm::pow(2.0_f64, -22.0));

    // Determine XMAX for the universal variable bracket.
    // For elliptic (alpha > 0): XMAX = 2π / sqrt(alpha) — one full orbit in χ-space.
    // For hyperbolic (alpha < 0): XMAX = 50 / |alpha|  — AGC constant -50SC (page 1278).
    let (x_max_pos, period_if_elliptic) = if alpha > 1e-20 {
        let period = core::f64::consts::TAU / (sqrt_mu * libm::pow(alpha, 1.5));
        let xmax = core::f64::consts::TAU / libm::sqrt(alpha);
        (xmax, period)
    } else if alpha < -1e-20 {
        let xmax = 50.0 / libm::fabs(alpha);
        (xmax, f64::INFINITY)
    } else {
        // Nearly parabolic: use sqrt(r0) heuristic
        (libm::sqrt(r0_norm) * 5.0, f64::INFINITY)
    };

    // Modulo reduction for long transfers (AGC: PERIODCH loop, page 1278).
    // Reduce dt so |tau| < one orbital period for elliptic orbits.
    let tau = if alpha > 1e-20 && period_if_elliptic.is_finite() && period_if_elliptic > 0.0 {
        let periods = libm::floor(dt / period_if_elliptic);
        dt - periods * period_if_elliptic
    } else {
        dt
    };

    // Sign for negative dt (AGC: STORBNDS negation, page 1278).
    let sign = if tau < 0.0 { -1.0_f64 } else { 1.0_f64 };

    // Initial guess for χ (KEPPREP / XKEPNEW, page 1334).
    // For elliptic: chi₀ ≈ sqrt(mu) × alpha × tau (linearised Kepler equation).
    // For hyperbolic: chi₀ ≈ sqrt(mu) × |alpha| × |tau| — same linearisation,
    //   avoids the x_max/2 default which is far too large for typical hyperbolic cases.
    // For parabolic (alpha ≈ 0): use the energy-based heuristic sqrt(r0).
    // AGC source: Comanche055/ORBITAL_INTEGRATION.agc KEPPREP (page 1334).
    let x_init = if alpha > 1e-20 {
        // Elliptic: linearised Kepler equation
        let guess = sqrt_mu * alpha * tau;
        if !guess.is_finite() || guess.abs() >= x_max_pos {
            sign * x_max_pos * 0.5
        } else {
            guess
        }
    } else if alpha < -1e-20 {
        // Hyperbolic: linearised approximation using |alpha|
        // Much smaller than x_max/2 and converges quickly.
        let guess = sqrt_mu * libm::fabs(alpha) * libm::fabs(tau);
        if !guess.is_finite() || guess <= 0.0 {
            sign * x_max_pos * 0.1
        } else {
            sign * guess
        }
    } else {
        // Nearly parabolic: energy-based heuristic
        sign * libm::sqrt(r0_norm)
    };

    // ── Newton iteration (AGC: KEPLOOP, page 1279) ───────────────────────────
    //
    // Time-of-flight formula (Battin/Goodyear universal variable, Battin p.165):
    //   T(χ) = [ χ³·S(z) + σ₀·χ²·C(z) + r₀·χ·(1 − z·S(z)) ] / √μ
    // where z = α·χ², S and C are Stumpff functions.
    //
    // Orbit radius (dT/dχ = r/√μ):
    //   r(χ) = χ²·C(z) + σ₀·χ·(1 − z·S(z)) + r₀·(1 − α·χ²·C(z))
    //
    // Newton step: Δχ = −(T − τ)·√μ / r.
    //
    // AGC source: Comanche055/CONIC_SUBROUTINES.agc, KEPLOOP (page 1279).
    let mut chi = x_init;
    let mut converged = false;
    let mut iters = 0u32;
    let mut best_r = *r0;
    let mut best_v = *v0;

    for _iter in 0..MAX_ITERATIONS {
        iters += 1;

        let chi2 = chi * chi;
        let chi3 = chi2 * chi;
        let z = alpha * chi2;
        let s_z = stumpff_s(z);
        let c_z = stumpff_c(z);

        // T(χ) — Battin universal-variable time formula
        let one_minus_z_sz = 1.0 - z * s_z;
        let t_of_chi =
            (chi3 * s_z + sigma0 * chi2 * c_z + r0_norm * chi * one_minus_z_sz) / sqrt_mu;

        // r(χ) — dT/dχ = r/√μ, so r(χ) = √μ · dT/dχ
        let r_val =
            chi2 * c_z + sigma0 * chi * one_minus_z_sz + r0_norm * (1.0 - alpha * chi2 * c_z);

        if r_val <= 0.0 || !r_val.is_finite() || !t_of_chi.is_finite() {
            // Guard: reset to midpoint on overflow / non-physical radius
            chi = sign * x_max_pos * 0.5;
            continue;
        }

        // Residual: T(χ) − τ
        let dt_residual = t_of_chi - tau;

        // Lagrange f, g coefficients (Bate-Mueller-White §4.4-17):
        //   f = 1 − χ²·C(z) / r₀
        //   g = τ − χ³·S(z) / √μ      (note: use τ, not T(χ))
        // These map r₀,v₀ → r,v at the current chi estimate.
        let f = 1.0 - chi2 * c_z / r0_norm;
        let g = tau - chi3 * s_z / sqrt_mu;
        let r_new = add(&scale(r0, f), &scale(v0, g));
        let r_new_norm = norm(&r_new);
        if r_new_norm > 0.0 {
            let g_dot = 1.0 - chi2 * c_z / r_new_norm;
            let f_dot = sqrt_mu * chi * (z * s_z - 1.0) / (r0_norm * r_new_norm);
            let v_new = add(&scale(r0, f_dot), &scale(v0, g_dot));
            best_r = r_new;
            best_v = v_new;
        }

        // Convergence check on time residual
        if dt_residual.abs() < epsilont {
            converged = true;
            break;
        }

        // Newton step: Δχ = −(T − τ) · √μ / r
        let delta_chi = -dt_residual * sqrt_mu / r_val;

        // Bracket clamp (AGC: NDXCHNGE / PDXCHNGE bisection fallback, page 1280)
        let new_chi = chi + delta_chi;
        if sign > 0.0 {
            chi = new_chi.clamp(1e-15, x_max_pos);
        } else {
            chi = new_chi.clamp(-x_max_pos, -1e-15);
        }
    }

    KeplerResult {
        r: best_r,
        v: best_v,
        converged,
        iterations: iters,
    }
}

/// Stumpff function S(z).
///
/// Defined as:
///   S(z) = (√z - sin(√z)) / z^(3/2)    for z > 0
///   S(z) = (sinh(√(-z)) - √(-z)) / (-z)^(3/2)  for z < 0
///   S(0) = 1/6
///
/// Implemented as a degree-9 Taylor series (10 terms) matching the AGC DELTIME
/// polynomial (CONIC_SUBROUTINES.agc, page 1283). Series is valid for |z| < ~40.
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, DELTIME (`S(XI)` output).
pub(crate) fn stumpff_s(z: f64) -> f64 {
    // Use closed-form for large |z| to avoid series divergence
    if z > 25.0 {
        let sq = libm::sqrt(z);
        return (sq - libm::sin(sq)) / (z * sq);
    }
    if z < -25.0 {
        let sq = libm::sqrt(-z);
        return (libm::sinh(sq) - sq) / ((-z) * sq);
    }
    // Taylor series: S(z) = sum_{k=0}^{N} (-z)^k / (2k+3)!
    // = 1/6 - z/120 + z^2/5040 - z^3/362880 + ...
    // AGC DELTIME polynomial coefficients (page 1283), scaled by 2 for fixed-point.
    // The Rust implementation uses the standard series directly.
    let mut sum = 0.0_f64;
    let mut term = 1.0_f64 / 6.0; // k=0: 1/3! = 1/6
    let mut factorial_denom = 6.0_f64;
    for k in 0..10u32 {
        if k > 0 {
            // term_k = (-z)^k / (2k+3)!
            // term_k = term_{k-1} * (-z) / ((2k+2) * (2k+3))
            let n1 = (2 * k + 2) as f64;
            let n2 = (2 * k + 3) as f64;
            term *= -z / (n1 * n2);
            let _ = factorial_denom;
            factorial_denom *= n1 * n2;
        }
        sum += term;
        if term.abs() < sum.abs() * 1e-15 {
            break;
        }
    }
    sum
}

/// Stumpff function C(z).
///
/// Defined as:
///   C(z) = (1 - cos(√z)) / z            for z > 0
///   C(z) = (cosh(√(-z)) - 1) / (-z)     for z < 0
///   C(0) = 1/2
///
/// Implemented as a degree-9 Taylor series (10 terms) matching the AGC DELTIME
/// polynomial (CONIC_SUBROUTINES.agc, page 1283-1284).
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, DELTIME.
pub(crate) fn stumpff_c(z: f64) -> f64 {
    // Use closed-form for large |z| to avoid series divergence
    if z > 25.0 {
        return (1.0 - libm::cos(libm::sqrt(z))) / z;
    }
    if z < -25.0 {
        return (libm::cosh(libm::sqrt(-z)) - 1.0) / (-z);
    }
    // Taylor series: C(z) = sum_{k=0}^{N} (-z)^k / (2k+2)!
    // = 1/2 - z/24 + z^2/720 - z^3/40320 + ...
    let mut sum = 0.0_f64;
    let mut term = 0.5_f64; // k=0: 1/2! = 1/2
    for k in 0..10u32 {
        if k > 0 {
            // term_k = term_{k-1} * (-z) / ((2k+1) * (2k+2))
            let n1 = (2 * k + 1) as f64;
            let n2 = (2 * k + 2) as f64;
            term *= -z / (n1 * n2);
        }
        sum += term;
        if term.abs() < sum.abs() * 1e-15 {
            break;
        }
    }
    sum
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::constants::{MU_EARTH, MU_MOON};
    use core::f64::consts::PI;

    /// TC-K-STUMPFF-1: Stumpff functions at z=0.
    ///
    /// S(0) = 1/6, C(0) = 1/2 by definition.
    #[test]
    fn stumpff_at_zero() {
        assert!(
            (stumpff_s(0.0) - 1.0 / 6.0).abs() < 1e-14,
            "S(0) = {}",
            stumpff_s(0.0)
        );
        assert!(
            (stumpff_c(0.0) - 0.5).abs() < 1e-14,
            "C(0) = {}",
            stumpff_c(0.0)
        );
    }

    /// TC-K-STUMPFF-2: Stumpff functions at z=1.
    ///
    /// C(1) = (1 - cos(1)) / 1 = 1 - cos(1) ≈ 0.4597.
    /// S(1) = (1 - sin(1)) / 1 ≈ (1 - 0.8415) / 1 = 0.1585 — no:
    /// S(1) = (sqrt(1) - sin(sqrt(1))) / 1^(3/2) = 1 - sin(1) ≈ 0.1585.
    #[test]
    fn stumpff_at_one() {
        let c1_expected = 1.0 - libm::cos(1.0);
        let s1_expected = 1.0 - libm::sin(1.0);
        assert!(
            (stumpff_c(1.0) - c1_expected).abs() < 1e-12,
            "C(1) = {} expected {}",
            stumpff_c(1.0),
            c1_expected
        );
        assert!(
            (stumpff_s(1.0) - s1_expected).abs() < 1e-12,
            "S(1) = {} expected {}",
            stumpff_s(1.0),
            s1_expected
        );
    }

    /// TC-K-STUMPFF-3: Stumpff functions at z=-1 (hyperbolic case).
    ///
    /// C(-1) = (cosh(1) - 1) / 1 = cosh(1) - 1 ≈ 0.5431.
    /// S(-1) = (sinh(1) - 1) / 1 ≈ 0.1752.
    #[test]
    fn stumpff_at_neg_one() {
        let c_neg1 = (libm::cosh(1.0) - 1.0) / 1.0;
        let s_neg1 = (libm::sinh(1.0) - 1.0) / 1.0;
        assert!(
            (stumpff_c(-1.0) - c_neg1).abs() < 1e-12,
            "C(-1)={} expected {}",
            stumpff_c(-1.0),
            c_neg1
        );
        assert!(
            (stumpff_s(-1.0) - s_neg1).abs() < 1e-12,
            "S(-1)={} expected {}",
            stumpff_s(-1.0),
            s_neg1
        );
    }

    /// TC-K-STUMPFF-4: Stumpff functions at z = pi^2/4.
    #[test]
    fn stumpff_at_pi_sq_over_4() {
        let z = PI * PI / 4.0;
        let sq = libm::sqrt(z); // = pi/2
        let c_expected = (1.0 - libm::cos(sq)) / z;
        let s_expected = (sq - libm::sin(sq)) / (z * sq);
        assert!(
            (stumpff_c(z) - c_expected).abs() < 1e-11,
            "C(pi^2/4)={} expected {}",
            stumpff_c(z),
            c_expected
        );
        assert!(
            (stumpff_s(z) - s_expected).abs() < 1e-11,
            "S(pi^2/4)={} expected {}",
            stumpff_s(z),
            s_expected
        );
    }

    /// TC-K-1: Circular LEO orbit, quarter-period propagation.
    ///
    /// r0 = [6_571_000, 0, 0] m, v0 = [0, 7_784.26, 0] m/s.
    /// dt = T/4 ≈ 1351 s.  Expected: |r| ≈ 6_571_000 m, |v| ≈ 7_784.26 m/s.
    #[test]
    fn circular_leo_quarter_period() {
        let r = 6_571_000.0_f64;
        let v_c = libm::sqrt(MU_EARTH / r);
        let r0 = [r, 0.0, 0.0];
        let v0 = [0.0, v_c, 0.0];

        let period = 2.0 * PI * libm::sqrt(r * r * r / MU_EARTH);
        let dt = period / 4.0;

        let result = kepler(&r0, &v0, dt, MU_EARTH);
        assert!(result.converged, "should converge");

        let r_final = norm(&result.r);
        let v_final = norm(&result.v);
        assert!((r_final - r).abs() < 1.0, "|r| = {r_final} expected {r}");
        assert!(
            (v_final - v_c).abs() < 0.1,
            "|v| = {v_final} expected {v_c}"
        );
    }

    /// TC-K-2: Elliptic orbit, perigee to apogee (Hohmann-like).
    #[test]
    fn elliptic_perigee_to_apogee() {
        let r_per = 6_571_000.0_f64;
        let r_apo = 42_164_000.0_f64;
        let a = (r_per + r_apo) / 2.0;
        let v_per = libm::sqrt(MU_EARTH * (2.0 / r_per - 1.0 / a));
        let v_apo = libm::sqrt(MU_EARTH * (2.0 / r_apo - 1.0 / a));
        let dt = PI * libm::sqrt(a * a * a / MU_EARTH);

        let r0 = [r_per, 0.0, 0.0];
        let v0 = [0.0, v_per, 0.0];

        let result = kepler(&r0, &v0, dt, MU_EARTH);
        assert!(result.converged, "elliptic should converge");

        let r_final = norm(&result.r);
        let v_final = norm(&result.v);
        assert!(
            (r_final - r_apo).abs() < 100.0,
            "|r| = {r_final}, expected {r_apo}"
        );
        assert!(
            (v_final - v_apo).abs() < 1.0,
            "|v| = {v_final}, expected {v_apo}"
        );
    }

    /// TC-K-3: Hyperbolic flyby — energy conservation.
    #[test]
    fn hyperbolic_energy_conservation() {
        let v_inf = 3_000.0_f64;
        let r_per = 6_671_000.0_f64;
        let a_hyp = -MU_EARTH / (v_inf * v_inf);
        let v_per = libm::sqrt(MU_EARTH * (2.0 / r_per - 1.0 / a_hyp));

        let r0 = [r_per, 0.0, 0.0];
        let v0 = [0.0, v_per, 0.0];
        let dt = 3600.0_f64;

        let result = kepler(&r0, &v0, dt, MU_EARTH);
        assert!(result.converged, "hyperbolic should converge");

        // Energy should be conserved
        let e_initial = 0.5 * (v_per * v_per) - MU_EARTH / r_per;
        let r_final = norm(&result.r);
        let v_final = norm(&result.v);
        let e_final = 0.5 * (v_final * v_final) - MU_EARTH / r_final;
        assert!(
            (e_final - e_initial).abs() < 1.0,
            "energy delta = {} J/kg",
            (e_final - e_initial).abs()
        );
        assert!(r_final > r_per, "should move away from periapsis");
    }

    /// TC-K-4: Zero transfer time — identity.
    #[test]
    fn zero_transfer_time_identity() {
        let r0 = [7_000_000.0_f64, 1_000_000.0, 500_000.0];
        let v0 = [100.0_f64, -7_500.0, 200.0];
        let result = kepler(&r0, &v0, 0.0, MU_EARTH);
        assert!(result.converged, "dt=0 must converge");
        assert_eq!(result.iterations, 0);
        for i in 0..3 {
            assert_eq!(result.r[i], r0[i]);
            assert_eq!(result.v[i], v0[i]);
        }
    }

    /// TC-K-5: Near-parabolic edge case (e ≈ 0.9999).
    #[test]
    fn near_parabolic_converges() {
        let r_per = 6_571_000.0_f64;
        let e = 0.9999_f64;
        let a = r_per / (1.0 - e);
        let v_per = libm::sqrt(MU_EARTH * (1.0 + e) / r_per);

        let r0 = [r_per, 0.0, 0.0];
        let v0 = [0.0, v_per, 0.0];

        let result = kepler(&r0, &v0, 600.0, MU_EARTH);
        assert!(
            result.converged,
            "near-parabolic should converge (may use all iterations)"
        );

        let r_final = norm(&result.r);
        assert!(r_final > r_per, "should move away from periapsis");

        // Energy should be nearly zero (small negative, nearly parabolic)
        let e_expected = -MU_EARTH / (2.0 * a);
        let v_final = norm(&result.v);
        let e_final = 0.5 * (v_final * v_final) - MU_EARTH / r_final;
        assert!(
            (e_final - e_expected).abs() < 10.0,
            "energy delta = {} J/kg",
            (e_final - e_expected).abs()
        );
    }

    /// TC-K-6: MU_MOON — verify propagation works with lunar mu.
    #[test]
    fn lunar_circular_orbit() {
        let r_moon = 1_837_400.0_f64; // 100 km altitude above Moon
        let v_c = libm::sqrt(MU_MOON / r_moon);
        let r0 = [r_moon, 0.0, 0.0];
        let v0 = [0.0, v_c, 0.0];
        let period = 2.0 * PI * libm::sqrt(r_moon * r_moon * r_moon / MU_MOON);
        let result = kepler(&r0, &v0, period / 4.0, MU_MOON);
        assert!(result.converged, "lunar circular should converge");
        let r_final = norm(&result.r);
        assert!((r_final - r_moon).abs() < 10.0, "|r| = {r_final}");
    }
}
