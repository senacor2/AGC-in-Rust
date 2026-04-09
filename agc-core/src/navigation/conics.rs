//! Conic orbit determination and utility routines.
//!
//! Provides classical orbital elements from a state vector and supporting
//! geometric utilities for conic-section trajectories (circles, ellipses,
//! parabolas, hyperbolas).
//!
//! AGC source: Comanche055/CONIC_SUBROUTINES.agc — orbit determination and
//! vis-viva computations used by KEPLER, LAMBERT, and CSMCONIC/LEMCONIC.

use crate::math::linalg::{cross, dot, norm, scale, sub, unit};
use crate::types::Vec3;

/// Classical orbital elements derived from a two-body state vector.
///
/// Sign conventions:
/// - `sma` is negative for hyperbolic trajectories.
/// - `ecc` < 1 elliptic, = 1 parabolic (approximately), > 1 hyperbolic.
/// - All angles are in radians, range `[0, 2π)` except `inc` which is `[0, π]`.
#[derive(Clone, Copy, Debug)]
pub struct OrbitalElements {
    /// Semi-major axis (m). Negative for hyperbolic orbits.
    pub sma: f64,
    /// Eccentricity (dimensionless).
    pub ecc: f64,
    /// Inclination (rad), range `[0, π]`.
    pub inc: f64,
    /// Right ascension of the ascending node (rad), range `[0, 2π)`.
    pub raan: f64,
    /// Argument of periapsis (rad), range `[0, 2π)`.
    pub aop: f64,
    /// True anomaly at the epoch (rad), range `[0, 2π)`.
    pub ta: f64,
    /// Gravitational parameter used (m³/s²).
    pub mu: f64,
}

/// Compute classical orbital elements from position and velocity vectors.
///
/// Uses the standard two-body orbit determination algorithm:
/// - Energy → semi-major axis
/// - Angular momentum vector → inclination and RAAN
/// - Eccentricity vector → argument of periapsis
/// - Angle between eccentricity vector and position → true anomaly
///
/// For circular orbits (`ecc < 1e-9`), `aop` is set to zero and `ta` is
/// measured from the ascending-node direction instead.
///
/// AGC source: Comanche055/CONIC_SUBROUTINES.agc — orbit determination
/// (KEPPREP, CSMCONIC input processing).
pub fn rv_to_elements(r: &Vec3, v: &Vec3, mu: f64) -> OrbitalElements {
    let r_mag = norm(r);
    let v_mag = norm(v);

    // Specific orbital energy: ε = v²/2 − μ/r
    let energy = 0.5 * v_mag * v_mag - mu / r_mag;

    // Semi-major axis: a = −μ / (2ε).  Parabolic orbit (energy ≈ 0) is
    // represented as a = f64::INFINITY (handled by the caller).
    let sma = if energy.abs() < 1e-6 {
        f64::INFINITY
    } else {
        -mu / (2.0 * energy)
    };

    // Specific angular momentum h = r × v
    let h = cross(r, v);
    let h_mag = norm(&h);

    // Inclination: cos(i) = h_z / |h|
    let inc = libm::acos((h[2] / h_mag).clamp(-1.0, 1.0));

    // Node vector n = K̂ × h  (K̂ = [0, 0, 1])
    let n = [-h[1], h[0], 0.0_f64];
    let n_mag = norm(&n);

    // RAAN: Ω = acos(n_x / |n|).  If n_y < 0, Ω ∈ (π, 2π).
    let raan = if n_mag < 1e-10 {
        // Equatorial orbit — RAAN is undefined, set to 0.
        0.0
    } else {
        let cos_raan = (n[0] / n_mag).clamp(-1.0, 1.0);
        let angle = libm::acos(cos_raan);
        if n[1] < 0.0 {
            core::f64::consts::TAU - angle
        } else {
            angle
        }
    };

    // Eccentricity vector e = ((v²−μ/r)r − (r·v)v) / μ
    let rdotv = dot(r, v);
    let e_vec = scale(
        &sub(
            &scale(r, v_mag * v_mag - mu / r_mag),
            &scale(v, rdotv),
        ),
        1.0 / mu,
    );
    let ecc = norm(&e_vec);

    // Argument of periapsis ω = acos(n̂ · ê).  If e_z < 0, ω ∈ (π, 2π).
    let aop = if ecc < 1e-9 {
        // Circular — argument of periapsis is undefined, set to 0.
        0.0
    } else if n_mag < 1e-10 {
        // Equatorial — measure ω from the x-axis.
        let cos_aop = (e_vec[0] / ecc).clamp(-1.0, 1.0);
        let angle = libm::acos(cos_aop);
        if e_vec[1] < 0.0 {
            core::f64::consts::TAU - angle
        } else {
            angle
        }
    } else {
        let cos_aop = (dot(&n, &e_vec) / (n_mag * ecc)).clamp(-1.0, 1.0);
        let angle = libm::acos(cos_aop);
        if e_vec[2] < 0.0 {
            core::f64::consts::TAU - angle
        } else {
            angle
        }
    };

    // True anomaly ν = acos(ê · r̂).  If r·v < 0, spacecraft is past
    // periapsis and heading toward apoapsis: ν ∈ (π, 2π).
    let ta = if ecc < 1e-9 {
        // Circular orbit: measure true anomaly from ascending node.
        if n_mag < 1e-10 {
            // Circular equatorial: measure from +x axis.
            let cos_ta = (r[0] / r_mag).clamp(-1.0, 1.0);
            let angle = libm::acos(cos_ta);
            if r[1] < 0.0 {
                core::f64::consts::TAU - angle
            } else {
                angle
            }
        } else {
            let cos_ta = (dot(&n, r) / (n_mag * r_mag)).clamp(-1.0, 1.0);
            let angle = libm::acos(cos_ta);
            if dot(&n, v) < 0.0 {
                core::f64::consts::TAU - angle
            } else {
                angle
            }
        }
    } else {
        let e_hat = unit(&e_vec);
        let r_hat = unit(r);
        let cos_ta = dot(&e_hat, &r_hat).clamp(-1.0, 1.0);
        let angle = libm::acos(cos_ta);
        if rdotv < 0.0 {
            core::f64::consts::TAU - angle
        } else {
            angle
        }
    };

    OrbitalElements { sma, ecc, inc, raan, aop, ta, mu }
}

/// Compute the orbital period for an elliptic orbit.
///
/// Returns `None` if `sma ≤ 0` (hyperbolic, parabolic, or degenerate).
///
/// Formula: T = 2π √(a³ / μ).
///
/// AGC source: Comanche055/CONIC_SUBROUTINES.agc — period used in KEPLER
/// modulo logic (TMODULO computation).
pub fn orbital_period(sma: f64, mu: f64) -> Option<f64> {
    if sma <= 0.0 || !sma.is_finite() {
        return None;
    }
    Some(core::f64::consts::TAU * libm::sqrt(sma * sma * sma / mu))
}

/// Compute the orbital speed at radius `r` on a conic orbit (vis-viva equation).
///
/// v = √(μ (2/r − 1/a))
///
/// For parabolic orbits (`sma` is infinite), simplifies to v = √(2μ/r).
///
/// AGC source: Comanche055/CONIC_SUBROUTINES.agc — vis-viva speed used in
/// KEPPREP and velocity-matching manoeuvres.
pub fn vis_viva_speed(r: f64, sma: f64, mu: f64) -> f64 {
    let term = if sma.is_finite() {
        2.0 / r - 1.0 / sma
    } else {
        2.0 / r
    };
    libm::sqrt(mu * term)
}

/// Compute specific orbital energy ε = v²/2 − μ/|r|.
///
/// Negative for bound (elliptic) orbits, zero for parabolic, positive for
/// hyperbolic.
pub fn specific_energy(r: &Vec3, v: &Vec3, mu: f64) -> f64 {
    let v2 = dot(v, v);
    let r_mag = norm(r);
    0.5 * v2 - mu / r_mag
}

/// Compute specific angular momentum vector h = r × v (m²/s).
///
/// AGC source: Comanche055/CONIC_SUBROUTINES.agc — h vector used throughout
/// orbit determination (inclination, node, and eccentricity derivation).
pub fn angular_momentum(r: &Vec3, v: &Vec3) -> Vec3 {
    cross(r, v)
}

/// Compute the eccentricity vector pointing from the focus toward periapsis.
///
/// e = ((v² − μ/r) r − (r·v) v) / μ
///
/// AGC source: Comanche055/CONIC_SUBROUTINES.agc — eccentricity used in
/// KEPPREP for initial-guess computation and orbit classification.
pub fn eccentricity_vector(r: &Vec3, v: &Vec3, mu: f64) -> Vec3 {
    let r_mag = norm(r);
    let v2 = dot(v, v);
    let rdotv = dot(r, v);
    scale(
        &sub(
            &scale(r, v2 - mu / r_mag),
            &scale(v, rdotv),
        ),
        1.0 / mu,
    )
}

/// Compute the flight path angle γ (rad): angle between the velocity vector
/// and the local horizontal plane.
///
/// γ = asin((r · v) / (|r| |v|))
///
/// Positive γ means the spacecraft is climbing (moving away from the central
/// body).
pub fn flight_path_angle(r: &Vec3, v: &Vec3) -> f64 {
    let r_mag = norm(r);
    let v_mag = norm(v);
    if r_mag < 1e-30 || v_mag < 1e-30 {
        return 0.0;
    }
    let sin_fpa = (dot(r, v) / (r_mag * v_mag)).clamp(-1.0, 1.0);
    libm::asin(sin_fpa)
}

/// Compute the true anomaly ν (rad) at the given position and velocity.
///
/// Equivalent to the `ta` field produced by `rv_to_elements`, but provided as
/// a standalone utility when only the true anomaly is required.
///
/// AGC source: Comanche055/CONIC_SUBROUTINES.agc — true anomaly used in
/// KEPLER initial-guess (XKEPNEW) derivation.
pub fn true_anomaly(r: &Vec3, v: &Vec3, mu: f64) -> f64 {
    rv_to_elements(r, v, mu).ta
}

/// Compute periapsis and apoapsis radii from semi-major axis and eccentricity.
///
/// Returns `(r_periapsis, r_apoapsis)`.
///
/// For hyperbolic orbits (`ecc > 1`) the apoapsis is mathematically negative
/// (no physical apoapsis exists); callers should check `ecc` before
/// interpreting the second return value.
///
/// Formulae:
/// - r_p = a (1 − e)
/// - r_a = a (1 + e)
pub fn apsides(sma: f64, ecc: f64) -> (f64, f64) {
    (sma * (1.0 - ecc), sma * (1.0 + ecc))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::gravity::MU_EARTH;
    use core::f64::consts::{PI, TAU};

    /// Tolerance for angle comparisons (rad).
    const ATOL: f64 = 1e-7;
    /// Tolerance for relative length/speed comparisons.
    const RTOL: f64 = 1e-9;

    // -----------------------------------------------------------------------
    // 1. Circular orbit in the XY plane
    // -----------------------------------------------------------------------

    fn circular_orbit_state() -> ([f64; 3], [f64; 3]) {
        // LEO: r = 6 578 137 m (200 km altitude), circular, equatorial.
        let r_mag = 6_578_137.0_f64;
        let v_mag = libm::sqrt(MU_EARTH / r_mag);
        ([r_mag, 0.0, 0.0], [0.0, v_mag, 0.0])
    }

    #[test]
    fn circular_orbit_eccentricity_near_zero() {
        let (r, v) = circular_orbit_state();
        let el = rv_to_elements(&r, &v, MU_EARTH);
        assert!(el.ecc < 1e-9, "ecc = {}", el.ecc);
    }

    #[test]
    fn circular_orbit_sma_equals_radius() {
        let (r, v) = circular_orbit_state();
        let r_mag = norm(&r);
        let el = rv_to_elements(&r, &v, MU_EARTH);
        let rel_err = (el.sma - r_mag).abs() / r_mag;
        assert!(rel_err < RTOL, "sma = {}, r = {}", el.sma, r_mag);
    }

    #[test]
    fn circular_orbit_period_matches_formula() {
        let (r, v) = circular_orbit_state();
        let el = rv_to_elements(&r, &v, MU_EARTH);
        let expected = TAU * libm::sqrt(el.sma.powi(3) / MU_EARTH);
        let computed = orbital_period(el.sma, MU_EARTH).expect("period must exist");
        let rel_err = (computed - expected).abs() / expected;
        assert!(rel_err < RTOL, "T_computed = {}, T_expected = {}", computed, expected);
    }

    #[test]
    fn circular_orbit_inclination_zero() {
        let (r, v) = circular_orbit_state();
        let el = rv_to_elements(&r, &v, MU_EARTH);
        assert!(el.inc.abs() < ATOL, "inc = {}", el.inc);
    }

    #[test]
    fn angular_momentum_circular_orbit_points_plus_z() {
        let (r, v) = circular_orbit_state();
        let h = angular_momentum(&r, &v);
        assert!(h[0].abs() < 1e-3, "h_x = {}", h[0]);
        assert!(h[1].abs() < 1e-3, "h_y = {}", h[1]);
        assert!(h[2] > 0.0, "h_z should be positive");
    }

    // -----------------------------------------------------------------------
    // 2. Elliptic orbit — GTO-like (200 km × 35 786 km)
    // -----------------------------------------------------------------------

    fn gto_state() -> ([f64; 3], [f64; 3]) {
        // GTO: perigee 200 km, apogee 35 786 km (GEO altitude), equatorial.
        let r_p = 6_578_137.0_f64;           // perigee radius (m)
        let r_a = 6_378_137.0 + 35_786_000.0; // apogee radius (m)
        let sma = 0.5 * (r_p + r_a);
        let ecc = (r_a - r_p) / (r_a + r_p);
        // At periapsis: v = sqrt(mu/a * (1+e)/(1-e))  = sqrt(mu*(1+e)/(a*(1-e)))
        let v_p = libm::sqrt(MU_EARTH * (1.0 + ecc) / (sma * (1.0 - ecc)));
        ([r_p, 0.0, 0.0], [0.0, v_p, 0.0])
    }

    #[test]
    fn gto_elements_eccentricity() {
        let (r, v) = gto_state();
        let r_p = 6_578_137.0_f64;
        let r_a = 6_378_137.0 + 35_786_000.0;
        let expected_ecc = (r_a - r_p) / (r_a + r_p);
        let el = rv_to_elements(&r, &v, MU_EARTH);
        let rel_err = (el.ecc - expected_ecc).abs() / expected_ecc;
        assert!(rel_err < 1e-6, "ecc = {}, expected ≈ {}", el.ecc, expected_ecc);
    }

    #[test]
    fn gto_apsides_match() {
        let (r, v) = gto_state();
        let r_p_expected = 6_578_137.0_f64;
        let r_a_expected = 6_378_137.0 + 35_786_000.0;
        let el = rv_to_elements(&r, &v, MU_EARTH);
        let (r_p, r_a) = apsides(el.sma, el.ecc);
        assert!(
            (r_p - r_p_expected).abs() / r_p_expected < 1e-6,
            "r_p = {}, expected = {}",
            r_p,
            r_p_expected
        );
        assert!(
            (r_a - r_a_expected).abs() / r_a_expected < 1e-6,
            "r_a = {}, expected = {}",
            r_a,
            r_a_expected
        );
    }

    #[test]
    fn gto_true_anomaly_at_periapsis_is_zero() {
        let (r, v) = gto_state();
        let el = rv_to_elements(&r, &v, MU_EARTH);
        // At periapsis ν = 0.  Allow small wrap-around (2π ≈ 0).
        let ta_norm = if el.ta > PI { el.ta - TAU } else { el.ta };
        assert!(ta_norm.abs() < ATOL, "ta = {}", el.ta);
    }

    // -----------------------------------------------------------------------
    // 3. Vis-viva: speed at periapsis and apoapsis
    // -----------------------------------------------------------------------

    #[test]
    fn vis_viva_at_gto_periapsis() {
        let (r_vec, v_vec) = gto_state();
        let el = rv_to_elements(&r_vec, &v_vec, MU_EARTH);
        let (r_p, _) = apsides(el.sma, el.ecc);
        let speed = vis_viva_speed(r_p, el.sma, MU_EARTH);
        let actual = norm(&v_vec);
        let rel_err = (speed - actual).abs() / actual;
        assert!(rel_err < RTOL, "vis-viva v_p = {}, actual = {}", speed, actual);
    }

    #[test]
    fn vis_viva_at_gto_apoapsis() {
        let (r_p_vec, v_p_vec) = gto_state();
        let el = rv_to_elements(&r_p_vec, &v_p_vec, MU_EARTH);
        let (_, r_a) = apsides(el.sma, el.ecc);
        // Speed at apoapsis from vis-viva
        let v_a_vv = vis_viva_speed(r_a, el.sma, MU_EARTH);
        // Independent check: conservation of angular momentum h = r_p * v_p = r_a * v_a
        let r_p = norm(&r_p_vec);
        let v_p = norm(&v_p_vec);
        let v_a_angular = r_p * v_p / r_a;
        let rel_err = (v_a_vv - v_a_angular).abs() / v_a_angular;
        assert!(rel_err < 1e-6, "v_a vis-viva = {}, v_a angular = {}", v_a_vv, v_a_angular);
    }

    // -----------------------------------------------------------------------
    // 4. Energy conservation
    // -----------------------------------------------------------------------

    #[test]
    fn specific_energy_is_conserved_along_gto() {
        let (r_p_vec, v_p_vec) = gto_state();
        let el = rv_to_elements(&r_p_vec, &v_p_vec, MU_EARTH);
        let e_periapsis = specific_energy(&r_p_vec, &v_p_vec, MU_EARTH);

        // Construct state at apoapsis
        let (_, r_a_mag) = apsides(el.sma, el.ecc);
        let v_a_mag = vis_viva_speed(r_a_mag, el.sma, MU_EARTH);
        // Apoapsis lies in the +x direction for our GTO setup, velocity in +y.
        let r_a_vec = [-r_a_mag, 0.0, 0.0];
        let v_a_vec = [0.0, -v_a_mag, 0.0];
        let e_apoapsis = specific_energy(&r_a_vec, &v_a_vec, MU_EARTH);

        let abs_err = (e_periapsis - e_apoapsis).abs();
        // Allow 1 J/kg relative to orbit energy magnitude
        assert!(
            abs_err < 1.0,
            "energy at periapsis = {}, at apoapsis = {}",
            e_periapsis,
            e_apoapsis
        );
    }

    // -----------------------------------------------------------------------
    // 5. Hyperbolic — orbital_period returns None
    // -----------------------------------------------------------------------

    #[test]
    fn hyperbolic_orbit_period_is_none() {
        assert!(orbital_period(-1e9, MU_EARTH).is_none());
    }

    // -----------------------------------------------------------------------
    // 6. Parabolic — orbital_period returns None (sma = infinity)
    // -----------------------------------------------------------------------

    #[test]
    fn parabolic_orbit_period_is_none() {
        assert!(orbital_period(f64::INFINITY, MU_EARTH).is_none());
    }

    // -----------------------------------------------------------------------
    // 7. Flight path angle at periapsis and apoapsis is zero
    // -----------------------------------------------------------------------

    #[test]
    fn flight_path_angle_zero_at_periapsis() {
        let (r, v) = gto_state();
        let fpa = flight_path_angle(&r, &v);
        assert!(fpa.abs() < ATOL, "fpa at periapsis = {}", fpa);
    }

    // -----------------------------------------------------------------------
    // 8. Eccentricity vector points in +x for our periapsis-at-+x setup
    // -----------------------------------------------------------------------

    #[test]
    fn eccentricity_vector_points_toward_periapsis() {
        let (r, v) = gto_state();
        let e_vec = eccentricity_vector(&r, &v, MU_EARTH);
        // For our GTO state (periapsis at +x, velocity in +y), e_vec must be in +x.
        assert!(e_vec[0] > 0.0, "e_x = {}", e_vec[0]);
        assert!(e_vec[1].abs() < 1e-9, "e_y = {}", e_vec[1]);
        assert!(e_vec[2].abs() < 1e-9, "e_z = {}", e_vec[2]);
    }
}
