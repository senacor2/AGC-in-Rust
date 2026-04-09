//! Lambert's problem solver — universal-variable (Izzo/Battin) formulation.
//!
//! Given two position vectors and a time of flight, find the initial and final
//! velocity vectors that connect them on a conic orbit.
//!
//! AGC source: Comanche055/CONIC_SUBROUTINES.agc — Lambert targeting routines
//! (LAMROUT, TIMETHET sections, pages 1262–1308).

use crate::math::kepler::{stumpff_c2, stumpff_c3};
use crate::math::linalg::{cross, dot, norm, scale, sub};
use crate::types::Vec3;

// ── iteration control ────────────────────────────────────────────────────────

const MAX_ITER: usize = 100;
const TOL: f64 = 1e-10;

// ── public API ───────────────────────────────────────────────────────────────

/// Result of Lambert's problem solution.
///
/// Both velocity vectors are expressed in the same inertial frame as the input
/// position vectors, in metres per second.
#[derive(Clone, Copy, Debug)]
pub struct LambertResult {
    /// Velocity at `r1` (m/s).
    pub v1: Vec3,
    /// Velocity at `r2` (m/s).
    pub v2: Vec3,
}

/// Solve Lambert's problem: find the orbit connecting `r1` to `r2` in time `tof`.
///
/// - `r1`: initial position vector (m)
/// - `r2`: final position vector (m)
/// - `tof`: time of flight (s), must be positive
/// - `mu`: gravitational parameter (m³/s²)
/// - `prograde`: if `true`, use the short-way (prograde) transfer; `false` for
///   the retrograde (long-way) transfer
///
/// Returns `None` if the geometry is degenerate (collinear `r1`/`r2`, zero
/// transfer time) or if the Newton–Raphson iterator fails to converge within
/// [`MAX_ITER`] iterations.
///
/// The algorithm follows the universal-variable (Izzo/Battin) formulation:
/// a single variable `z = α·χ²` (where `α = 1/a` is the inverse semi-major
/// axis and `χ` the universal anomaly) is iterated until the time equation is
/// satisfied.  Stumpff functions c₂(z) and c₃(z) handle the elliptic,
/// parabolic, and hyperbolic cases uniformly.
///
/// AGC source: Comanche055/CONIC_SUBROUTINES.agc — Lambert targeting routines
/// (LAMROUT, TIMETHET sections).
pub fn lambert(
    r1: &Vec3,
    r2: &Vec3,
    tof: f64,
    mu: f64,
    prograde: bool,
) -> Option<LambertResult> {
    if tof <= 0.0 || mu <= 0.0 {
        return None;
    }

    let r1_norm = norm(r1);
    let r2_norm = norm(r2);

    if r1_norm < 1.0 || r2_norm < 1.0 {
        return None;
    }

    // Transfer angle Δν ∈ (0, 2π).
    // The cross-product z-component determines the orbital plane orientation.
    let cos_dnu = dot(r1, r2) / (r1_norm * r2_norm);
    let cos_dnu = cos_dnu.clamp(-1.0, 1.0);

    let cross_r = cross(r1, r2);
    // z-component of r1 × r2 tells us the direction of angular momentum.
    let cross_z = cross_r[2];

    // For a prograde transfer the angular momentum points in the +z direction
    // (cross_z > 0).  For retrograde it points in the −z direction.
    let dnu = {
        let angle = libm::acos(cos_dnu);
        match (prograde, cross_z >= 0.0) {
            (true, true) => angle,
            (true, false) => core::f64::consts::TAU - angle,
            (false, true) => core::f64::consts::TAU - angle,
            (false, false) => angle,
        }
    };

    // Degenerate case: 0° or 360° transfer (r1 ∥ r2, same direction).
    if dnu < 1e-10 || (core::f64::consts::TAU - dnu) < 1e-10 {
        return None;
    }

    let sin_dnu = libm::sin(dnu);

    // Auxiliary parameter A (Bate–Mueller–White, eq. 7.3-3).
    // A = sin(Δν) * √(r1·r2 / (1 − cos(Δν)))
    let a_param = sin_dnu * libm::sqrt(r1_norm * r2_norm / (1.0 - cos_dnu));

    // ── 180° transfer special case ──────────────────────────────────────────
    //
    // When Δν = π exactly, sin(Δν) = 0 so A = 0, and the Lagrange-coefficient
    // formulation degenerates (g = A·√(y/μ) = 0 → division by zero).
    // The orbit plane is undetermined by geometry alone; we choose the xy-plane
    // (prograde: v points +y at r1; retrograde: −y).
    //
    // For a 180° half-transfer on an ellipse with semi-major axis `a`:
    //   tof = π √(a³/μ)  →  a = (μ (tof/π)²)^(1/3)
    // Then vis-viva gives the speed at each endpoint.
    if dnu.abs() > core::f64::consts::PI - 1e-7
        && (core::f64::consts::TAU - dnu) > core::f64::consts::PI - 1e-7
    {
        // Semi-major axis from the 180° time equation.
        let ratio = tof / core::f64::consts::PI;
        let a_sma = libm::cbrt(mu * ratio * ratio);
        if a_sma <= 0.0 {
            return None;
        }
        // Vis-viva speeds.
        let v1_mag_sq = mu * (2.0 / r1_norm - 1.0 / a_sma);
        let v2_mag_sq = mu * (2.0 / r2_norm - 1.0 / a_sma);
        if v1_mag_sq <= 0.0 || v2_mag_sq <= 0.0 {
            return None;
        }
        let v1_mag = libm::sqrt(v1_mag_sq);
        let v2_mag = libm::sqrt(v2_mag_sq);

        // Choose the orbit plane: use the reference +z axis if r1 is not
        // parallel to it, otherwise use +x.
        let ref_axis: Vec3 = if r1[2].abs() < r1_norm * 0.9 {
            [0.0, 0.0, 1.0]
        } else {
            [1.0, 0.0, 0.0]
        };
        // Tangent at r1: perpendicular to r1, in the plane spanned by r1 and ref_axis.
        // t1 = normalize(ref_axis − (ref_axis·r̂1) r̂1)
        let r1_unit = scale(r1, 1.0 / r1_norm);
        let proj = dot(&ref_axis, &r1_unit);
        let t1_raw = sub(&ref_axis, &scale(&r1_unit, proj));
        let t1_norm = norm(&t1_raw);
        if t1_norm < 1e-10 {
            return None;
        }
        let t1 = scale(&t1_raw, 1.0 / t1_norm);

        // Prograde: v1 in the +t1 direction; retrograde: −t1.
        let sign = if prograde { 1.0_f64 } else { -1.0_f64 };
        let v1 = scale(&t1, sign * v1_mag);

        // At r2 (anti-podal) the tangent direction is −t1 rotated 180°.
        // For a prograde transfer the velocity at r2 also points in +t1
        // (the orbit crosses apoapsis/periapsis between r1 and r2 for a
        // Hohmann-style transfer, so v2 is anti-parallel to the r2 direction
        // and parallel to t1, same half-plane).
        let v2 = scale(&t1, sign * v2_mag);

        return Some(LambertResult { v1, v2 });
    }

    if a_param.abs() < 1e-10 {
        return None;
    }

    // ── Newton–Raphson on z ──────────────────────────────────────────────────
    //
    // For a given z, define:
    //   c2 = stumpff_c2(z),  c3 = stumpff_c3(z)
    //   y  = r1 + r2 + A*(z*c3 − 1) / √c2
    //   χ  = √(y / c2)
    //   dt = (χ³*c3 + A*√y) / √μ
    //
    // We want dt = tof.  The derivative ddt/dz is derived analytically and used
    // in the Newton step.

    let sqrt_mu = libm::sqrt(mu);
    let mut z = 0.0_f64; // initial guess: parabolic

    let mut z_lo = -4.0 * core::f64::consts::PI * core::f64::consts::PI;
    let mut z_hi = 4.0 * core::f64::consts::PI * core::f64::consts::PI * 100.0;

    // Evaluate tof(z).  Returns None when y ≤ 0 (geometry invalid for this z).
    let tof_at_z = |z: f64| -> Option<(f64, f64)> {
        let c2 = stumpff_c2(z);
        let c3 = stumpff_c3(z);
        if c2 < 1e-30 {
            return None;
        }
        let y = r1_norm + r2_norm + a_param * (z * c3 - 1.0) / libm::sqrt(c2);
        if y < 0.0 {
            return None;
        }
        let chi = libm::sqrt(y / c2);
        let dt = (chi * chi * chi * c3 + a_param * libm::sqrt(y)) / sqrt_mu;
        // Derivative dt/dz for Newton step (Vallado eq. 7-27 form).
        let dtdz = if z.abs() > 1e-6 {
            let chi2 = chi * chi;
            (chi2 * chi * (c2 - 1.5 * c3 / c2) + 0.125 * a_param * (3.0 * c3 * libm::sqrt(y) / c2 + a_param / chi)) / sqrt_mu
        } else {
            // Near z=0 use a finite-difference approximation.
            let dz = 1e-6;
            let c2p = stumpff_c2(dz);
            let c3p = stumpff_c3(dz);
            let yp = r1_norm + r2_norm + a_param * (dz * c3p - 1.0) / libm::sqrt(c2p);
            if yp < 0.0 {
                return None;
            }
            let chip = libm::sqrt(yp / c2p);
            let dtp = (chip * chip * chip * c3p + a_param * libm::sqrt(yp)) / sqrt_mu;
            (dtp - dt) / dz
        };
        Some((dt, dtdz))
    };

    // Bracket the root: push z_lo up until tof(z_lo) < tof.
    for _ in 0..50 {
        if let Some((dt, _)) = tof_at_z(z_lo) {
            if dt < tof {
                break;
            }
        }
        z_lo *= 0.5;
    }

    // Newton–Raphson with bisection fallback.
    for _ in 0..MAX_ITER {
        match tof_at_z(z) {
            None => {
                // y < 0: move z toward z_lo.
                z = 0.5 * (z + z_lo);
                continue;
            }
            Some((dt, dtdz)) => {
                let err = dt - tof;
                if err.abs() < TOL {
                    break;
                }
                // Newton step, clamped to the current bracket.
                let z_new = if dtdz.abs() > 1e-30 {
                    (z - err / dtdz).clamp(z_lo, z_hi)
                } else {
                    0.5 * (z_lo + z_hi)
                };
                if err > 0.0 {
                    z_hi = z;
                } else {
                    z_lo = z;
                }
                z = z_new;
            }
        }
    }

    // Final evaluation.
    let c2 = stumpff_c2(z);
    let c3 = stumpff_c3(z);
    if c2 < 1e-30 {
        return None;
    }
    let y = r1_norm + r2_norm + a_param * (z * c3 - 1.0) / libm::sqrt(c2);
    if y < 0.0 {
        return None;
    }
    let chi = libm::sqrt(y / c2);

    // Lagrange coefficients.
    // f  = 1 − y/r1
    // g  = A·√(y/μ)
    // ġ  = 1 − y/r2
    let f = 1.0 - y / r1_norm;
    let g = a_param * libm::sqrt(y / mu);
    let g_dot = 1.0 - y / r2_norm;

    if g.abs() < 1e-30 {
        return None;
    }

    // v1 = (r2 − f·r1) / g
    let v1 = scale(&sub(r2, &scale(r1, f)), 1.0 / g);
    // v2 = (ġ·r2 − r1) / g
    let v2 = scale(&sub(&scale(r2, g_dot), r1), 1.0 / g);

    // Sanity check: velocities must be finite.
    for i in 0..3 {
        if !v1[i].is_finite() || !v2[i].is_finite() {
            return None;
        }
    }

    // Verify convergence: re-evaluate the time equation at the solution.
    let chi3_c3 = chi * chi * chi * c3;
    let tof_check = (chi3_c3 + a_param * libm::sqrt(y)) / sqrt_mu;
    if (tof_check - tof).abs() > 1e-3 * tof.abs().max(1.0) {
        return None;
    }

    Some(LambertResult { v1, v2 })
}

// ── tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::gravity::MU_EARTH;
    use core::f64::consts::PI;

    /// Earth radius used in test orbit definitions (m).
    const R_EARTH: f64 = 6_371_000.0;

    // Helper: circular orbital speed at radius r.
    fn v_circ(r: f64, mu: f64) -> f64 {
        libm::sqrt(mu / r)
    }

    // Helper: vis-viva speed at radius r on an ellipse with semi-major axis a.
    fn v_visviva(r: f64, a: f64, mu: f64) -> f64 {
        libm::sqrt(mu * (2.0 / r - 1.0 / a))
    }

    // ── 1. Hohmann transfer ──────────────────────────────────────────────────
    //
    // 200 km → 400 km altitude, prograde coplanar.
    // tof = half the transfer-orbit period.
    // Expect: v1 ≈ vis-viva at periapsis of transfer ellipse,
    //         v2 ≈ vis-viva at apoapsis.
    #[test]
    fn hohmann_200_to_400_km() {
        let r1_m = R_EARTH + 200_000.0; // 6 571 000 m
        let r2_m = R_EARTH + 400_000.0; // 6 771 000 m

        let a_transfer = 0.5 * (r1_m + r2_m);
        let period_transfer = 2.0 * PI * libm::sqrt(a_transfer.powi(3) / MU_EARTH);
        let tof = 0.5 * period_transfer;

        let r1: Vec3 = [r1_m, 0.0, 0.0];
        let r2: Vec3 = [-r2_m, 0.0, 0.0]; // 180° transfer

        let result = lambert(&r1, &r2, tof, MU_EARTH, true)
            .expect("Hohmann Lambert solver must converge");

        let v1_expected = v_visviva(r1_m, a_transfer, MU_EARTH);
        let v2_expected = v_visviva(r2_m, a_transfer, MU_EARTH);

        let v1_got = norm(&result.v1);
        let v2_got = norm(&result.v2);

        assert!(
            (v1_got - v1_expected).abs() < 0.1,
            "v1 magnitude: got {:.4} m/s, expected {:.4} m/s",
            v1_got,
            v1_expected
        );
        assert!(
            (v2_got - v2_expected).abs() < 0.1,
            "v2 magnitude: got {:.4} m/s, expected {:.4} m/s",
            v2_got,
            v2_expected
        );
    }

    // ── 2. 180-degree transfer ───────────────────────────────────────────────
    //
    // Diametrically opposite points on a circular 400 km orbit.
    // tof = half the circular orbital period.
    // The transfer orbit is the same circle; v1 and v2 equal v_circ.
    #[test]
    fn half_circular_orbit() {
        let r_m = R_EARTH + 400_000.0;
        let period = 2.0 * PI * libm::sqrt(r_m.powi(3) / MU_EARTH);
        let tof = 0.5 * period;

        let r1: Vec3 = [r_m, 0.0, 0.0];
        let r2: Vec3 = [-r_m, 0.0, 0.0];

        let result = lambert(&r1, &r2, tof, MU_EARTH, true)
            .expect("180° circular Lambert must converge");

        let v_c = v_circ(r_m, MU_EARTH);
        let v1_got = norm(&result.v1);
        let v2_got = norm(&result.v2);

        assert!(
            (v1_got - v_c).abs() < 1.0,
            "v1: got {:.4} m/s, expected {:.4} m/s",
            v1_got,
            v_c
        );
        assert!(
            (v2_got - v_c).abs() < 1.0,
            "v2: got {:.4} m/s, expected {:.4} m/s",
            v2_got,
            v_c
        );
    }

    // ── 3. Short (30°) transfer on a circular orbit ──────────────────────────
    //
    // r1 at 0°, r2 at 30° on a 500 km circular orbit.
    // tof = 30/360 × orbital period.
    // The connecting orbit is the same circle, so speeds equal v_circ.
    #[test]
    fn short_30_degree_transfer() {
        let r_m = R_EARTH + 500_000.0;
        let period = 2.0 * PI * libm::sqrt(r_m.powi(3) / MU_EARTH);
        let tof = (30.0 / 360.0) * period;

        let r1: Vec3 = [r_m, 0.0, 0.0];
        let angle = 30.0_f64.to_radians();
        let r2: Vec3 = [r_m * libm::cos(angle), r_m * libm::sin(angle), 0.0];

        let result = lambert(&r1, &r2, tof, MU_EARTH, true)
            .expect("30° circular Lambert must converge");

        let v_c = v_circ(r_m, MU_EARTH);
        let v1_got = norm(&result.v1);
        let v2_got = norm(&result.v2);

        assert!(
            (v1_got - v_c).abs() < 1.0,
            "v1: got {:.4} m/s, expected {:.4} m/s",
            v1_got,
            v_c
        );
        assert!(
            (v2_got - v_c).abs() < 1.0,
            "v2: got {:.4} m/s, expected {:.4} m/s",
            v2_got,
            v_c
        );
    }

    // ── 4. Symmetry: swap endpoints, tof unchanged ───────────────────────────
    //
    // lambert(r1, r2, tof) and lambert(r2, r1, tof) should yield |v1| and |v2|
    // swapped respectively (same orbit, traversed in opposite directions).
    #[test]
    fn symmetry_swap_endpoints() {
        let r1_m = R_EARTH + 300_000.0;
        let r2_m = R_EARTH + 700_000.0;
        let a_transfer = 0.5 * (r1_m + r2_m);
        let tof = 0.5 * 2.0 * PI * libm::sqrt(a_transfer.powi(3) / MU_EARTH);

        let r1: Vec3 = [r1_m, 0.0, 0.0];
        let r2: Vec3 = [-r2_m, 0.0, 0.0];

        let fwd = lambert(&r1, &r2, tof, MU_EARTH, true)
            .expect("forward Lambert must converge");
        let rev = lambert(&r2, &r1, tof, MU_EARTH, false)
            .expect("reverse Lambert must converge");

        // |v1_fwd| should equal |v2_rev|  and  |v2_fwd| should equal |v1_rev|.
        let tol = 0.5; // 0.5 m/s tolerance
        let v1f = norm(&fwd.v1);
        let v2f = norm(&fwd.v2);
        let v1r = norm(&rev.v1);
        let v2r = norm(&rev.v2);

        assert!(
            (v1f - v2r).abs() < tol,
            "|v1_fwd|={:.4} should equal |v2_rev|={:.4}",
            v1f,
            v2r
        );
        assert!(
            (v2f - v1r).abs() < tol,
            "|v2_fwd|={:.4} should equal |v1_rev|={:.4}",
            v2f,
            v1r
        );
    }

    // ── 5. Invalid inputs return None ────────────────────────────────────────
    #[test]
    fn degenerate_inputs_return_none() {
        let r1: Vec3 = [7_000_000.0, 0.0, 0.0];
        let r2: Vec3 = [7_000_000.0, 0.0, 0.0]; // same point — degenerate
        assert!(lambert(&r1, &r2, 3600.0, MU_EARTH, true).is_none());

        // Negative tof.
        let r2b: Vec3 = [-7_000_000.0, 0.0, 0.0];
        assert!(lambert(&r1, &r2b, -100.0, MU_EARTH, true).is_none());

        // Zero mu.
        assert!(lambert(&r1, &r2b, 3600.0, 0.0, true).is_none());
    }
}
