//! Orbital conic-section element computation (APSIDES, PARAM, GEOM, GETECC).
//!
//! Pure functions computing classical Keplerian orbital elements and derived
//! quantities (period, apsides, vis-viva speed) from a Cartesian state vector.
//! Corresponding to the APSIDES / PARAM / GEOM / GETECC / TIMERAD routines in
//! CONIC_SUBROUTINES.agc, extracted here as named pure functions.
//!
//! AGC source: Comanche055/CONIC_SUBROUTINES.agc
//!   APSIDES (pages 1275-1277, 1303-1304), GETECC (page 1303),
//!   PARAM (pages 1289-1290), GEOM (page 1291), TIMERAD (pages 1301-1303).
//! AGC source: Comanche055/P30-P37.agc
//!   S30.1 (page 639), PARAM30 (page 637).
//! AGC source: Comanche055/POWERED_FLIGHT_SUBROUTINES.agc PERIAPO1 (pages 1365-1372).

use crate::math::linalg::{cross, dot, norm, norm_sq, scale, sub};
use crate::types::Vec3;

/// Minimum radius singularity guard (metres).
///
/// Prevents division-by-zero in element computation.
const MIN_RADIUS: f64 = 1.0;

/// Minimum squared angular momentum guard (m⁴/s²).
///
/// Corresponds to |h| < 1e-6 m²/s — rectilinear trajectory.
const MIN_H_SQ: f64 = 1e-12;

/// Classical Keplerian orbital elements computed from a state vector.
///
/// All angles in radians. Distances in metres. Dimensionless eccentricity.
///
/// For rectilinear (zero angular momentum) trajectories, `elements_from_state`
/// returns `None`. For hyperbolic orbits, `sma < 0` and `ecc > 1`.
///
/// AGC source: elements derived from PARAM, GETECC, and GEOM routines in
/// `Comanche055/CONIC_SUBROUTINES.agc` (pages 1289-1292, 1303).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OrbitalElements {
    /// Semi-major axis in metres. Negative for hyperbolic orbits.
    pub sma: f64,
    /// Eccentricity. 0 = circular, 0 < e < 1 = elliptic, e = 1 parabolic, e > 1 hyperbolic.
    pub ecc: f64,
    /// Inclination in radians, [0, π].
    pub inc: f64,
    /// Right ascension of ascending node (RAAN) in radians, [0, 2π).
    pub raan: f64,
    /// Argument of periapsis in radians, [0, 2π).
    pub argp: f64,
    /// True anomaly in radians, [0, 2π).
    pub true_anom: f64,
}

/// Compute classical orbital elements from a Cartesian state vector.
///
/// Returns `None` for degenerate cases:
/// - Zero or near-zero angular momentum (`|h|² < 1e-12 m⁴/s²`) — rectilinear trajectory.
/// - Zero or near-zero radius (`|r| < 1.0 m`) — singularity.
/// - Zero or negative `mu`.
///
/// For parabolic orbits (ε ≈ 0), `sma` will be very large (positive). The function
/// does not special-case parabolic; it returns a valid struct with large `sma`.
///
/// # Scale factors
/// - Input position: metres.
/// - Input velocity: m/s.
/// - Input mu: m³/s².
/// - Output angles: radians.
/// - Output distances (sma): metres.
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, PARAM (page 1289), GETECC (page 1303),
///             GEOM (page 1291). Inclination / RAAN / argp are not directly in the AGC
///             (which uses the orbit-plane normal UN instead); they are computed here from h.
pub fn elements_from_state(r: &Vec3, v: &Vec3, mu: f64) -> Option<OrbitalElements> {
    if mu <= 0.0 {
        return None;
    }
    let r_norm = norm(r);
    if r_norm < MIN_RADIUS {
        return None;
    }

    // h = r × v  (specific angular momentum vector; AGC: VXV in GEOM)
    let h = cross(r, v);
    let h_sq = norm_sq(&h);
    if h_sq < MIN_H_SQ {
        return None;
    }
    let h_norm = libm::sqrt(h_sq);

    // Specific orbital energy: ε = |v|²/2 - μ/|r|
    let v_sq = norm_sq(v);
    let epsilon = 0.5 * v_sq - mu / r_norm;

    // Semi-major axis: a = -μ / (2ε)
    // For parabolic (ε ≈ 0): a → ±∞; use large finite value
    let sma = if epsilon.abs() > 1e-6 {
        -mu / (2.0 * epsilon)
    } else {
        // Nearly parabolic: return a very large positive value
        1e18_f64
    };

    // Eccentricity vector: e_vec = v × h / μ - r / |r|
    // AGC: from TIMERAD COSF computation via GETECC
    let v_cross_h = cross(v, &h);
    let e_vec = sub(&scale(&v_cross_h, 1.0 / mu), &scale(r, 1.0 / r_norm));
    let ecc = norm(&e_vec);

    // Inclination: i = acos(h[2] / |h|)
    let cos_i = (h[2] / h_norm).clamp(-1.0, 1.0);
    let inc = libm::acos(cos_i);

    // RAAN (Ω): angle of ascending node in the equatorial plane
    // Node vector n = k × h (k = [0,0,1])
    let n_vec = cross(&[0.0, 0.0, 1.0], &h); // = [-h[1], h[0], 0]
    let n_norm = norm(&n_vec);

    let raan = if n_norm < 1e-10 {
        // Equatorial orbit: RAAN undefined, return 0 per AGC convention
        0.0
    } else {
        let raan_raw = libm::acos((n_vec[0] / n_norm).clamp(-1.0, 1.0));
        if n_vec[1] < 0.0 {
            core::f64::consts::TAU - raan_raw
        } else {
            raan_raw
        }
    };

    // Argument of periapsis (ω)
    let argp = if ecc < 1e-7 || n_norm < 1e-10 {
        // Circular or equatorial: argp undefined, return 0 per AGC convention
        0.0
    } else {
        let cos_argp = (dot(&n_vec, &e_vec) / (n_norm * ecc)).clamp(-1.0, 1.0);
        let argp_raw = libm::acos(cos_argp);
        if e_vec[2] < 0.0 {
            core::f64::consts::TAU - argp_raw
        } else {
            argp_raw
        }
    };

    // True anomaly (ν): angle from periapsis to current position
    // AGC: TIMERAD computes COSF = cos(true anomaly)
    let true_anom = if ecc < 1e-7 {
        // Circular: true anomaly undefined, return 0 per AGC convention
        0.0
    } else {
        let cos_nu = (dot(&e_vec, r) / (ecc * r_norm)).clamp(-1.0, 1.0);
        let nu_raw = libm::acos(cos_nu);
        if dot(r, v) < 0.0 {
            core::f64::consts::TAU - nu_raw
        } else {
            nu_raw
        }
    };

    Some(OrbitalElements {
        sma,
        ecc,
        inc,
        raan,
        argp,
        true_anom,
    })
}

/// Orbital speed from vis-viva equation: v = √(μ*(2/r - 1/a)).
///
/// # Arguments
/// - `r`: current radius in metres. Must be > 0.
/// - `a`: semi-major axis in metres. Negative for hyperbolic.
/// - `mu`: gravitational parameter in m³/s².
///
/// Returns 0.0 if `r <= 0.0` or `mu <= 0.0` (rather than NaN/panic).
/// Returns `f64::NAN` if the expression under the sqrt is negative (unphysical inputs).
///
/// AGC source: Implicit in `Comanche055/CONIC_SUBROUTINES.agc`, INITV velocity construction
///             (page 1299) and KEPLERN initialization (KEPC2 = r*v²/μ - 1, page 1277).
pub fn vis_viva(r: f64, a: f64, mu: f64) -> f64 {
    if r <= 0.0 || mu <= 0.0 {
        return 0.0;
    }
    let radicand = mu * (2.0 / r - 1.0 / a);
    if radicand < 0.0 {
        f64::NAN
    } else {
        libm::sqrt(radicand)
    }
}

/// Apoapsis and periapsis radii from orbital elements.
///
/// Returns `(apoapsis_m, periapsis_m)` where:
/// - `apoapsis_m = a * (1 + e)` — `f64::INFINITY` for hyperbolic / parabolic (e ≥ 1)
/// - `periapsis_m = a * (1 - e)`
///
/// # Invariants
/// - Never panics.
/// - If `elements.ecc >= 1.0`, apoapsis is `f64::INFINITY`.
///   Corresponds to AGC INFINAPO branch returning LDPOSMAX (page 1303).
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, APSIDES routine (page 1303-1304).
/// P30 display: `HAPO` / `HPER` erasables, displayed via N42 in PARAM30 (P30-P37.agc, page 637).
pub fn apoapsis_periapsis(elements: &OrbitalElements) -> (f64, f64) {
    let a = elements.sma;
    let e = elements.ecc;
    let periapsis = a * (1.0 - e);
    let apoapsis = if e >= 1.0 {
        f64::INFINITY
    } else {
        a * (1.0 + e)
    };
    (apoapsis, periapsis)
}

/// Orbital period from semi-major axis.
///
/// `T = 2π * √(a³ / μ)` for elliptic orbits (a > 0).
///
/// Returns `f64::INFINITY` for parabolic/hyperbolic orbits (a ≤ 0).
/// Returns `0.0` if `mu <= 0.0`.
///
/// # AGC source
/// Implicit in `Comanche055/CONIC_SUBROUTINES.agc`, KEPLERN modulo reduction
/// (PERIODCH loop, page 1278): `SQRT 2PISC / ALPHA` chain, where
/// `2PISC = 2π * 2^-6` and `ALPHA = 1/a`.
pub fn period(a: f64, mu: f64) -> f64 {
    if mu <= 0.0 {
        return 0.0;
    }
    if a <= 0.0 {
        return f64::INFINITY;
    }
    core::f64::consts::TAU * libm::sqrt(a * a * a / mu)
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::constants::MU_EARTH;
    use core::f64::consts::PI;

    /// TC-C-1: Circular LEO orbit (ecc ≈ 0, inc = 0°).
    ///
    /// r = 6_571_000 m, v_circ = √(mu/r) ≈ 7_784.26 m/s.
    #[test]
    fn circular_leo_equatorial() {
        let r = 6_571_000.0_f64;
        let v_c = libm::sqrt(MU_EARTH / r);
        let r_vec = [r, 0.0, 0.0];
        let v_vec = [0.0, v_c, 0.0];

        let el = elements_from_state(&r_vec, &v_vec, MU_EARTH).expect("should be Some");

        assert!((el.sma - r).abs() < 1.0, "sma={} expected {r}", el.sma);
        assert!(el.ecc < 1e-6, "ecc={}", el.ecc);
        assert!(el.inc < 1e-6, "inc={} (should be ~0 rad)", el.inc);

        let t = period(el.sma, MU_EARTH);
        let expected_period = 2.0 * PI * libm::sqrt(r * r * r / MU_EARTH);
        assert!(
            (t - expected_period).abs() < 1.0,
            "period={t} expected {expected_period}"
        );

        let (apo, per) = apoapsis_periapsis(&el);
        assert!((apo - r).abs() < 10.0, "apo={apo} expected {r}");
        assert!((per - r).abs() < 10.0, "per={per} expected {r}");

        let vv = vis_viva(r, el.sma, MU_EARTH);
        assert!((vv - v_c).abs() < 0.01, "vis_viva={vv} expected {v_c}");
    }

    /// TC-C-2: Geostationary Transfer Orbit (elliptic, ecc ≈ 0.73).
    #[test]
    fn geostationary_transfer_orbit() {
        let r_per = 6_556_370.0_f64;
        let r_apo = 42_164_170.0_f64;
        let a = (r_per + r_apo) / 2.0;
        let e_expected = (r_apo - r_per) / (r_apo + r_per);

        let v_per = libm::sqrt(MU_EARTH * (2.0 / r_per - 1.0 / a));
        let r_vec = [r_per, 0.0, 0.0];
        let v_vec = [0.0, v_per, 0.0];

        let el = elements_from_state(&r_vec, &v_vec, MU_EARTH).expect("should be Some");

        assert!((el.sma - a).abs() < 1_000.0, "sma={} expected {a}", el.sma);
        assert!(
            (el.ecc - e_expected).abs() < 0.001,
            "ecc={} expected {e_expected}",
            el.ecc
        );
        assert!(el.inc < 1e-6, "inc={} (equatorial)", el.inc);

        let (apo, per) = apoapsis_periapsis(&el);
        assert!((apo - r_apo).abs() < 1_000.0, "apo={apo} expected {r_apo}");
        assert!((per - r_per).abs() < 1_000.0, "per={per} expected {r_per}");
    }

    /// TC-C-3: ISS-like inclined orbit (i ≈ 51.6°).
    #[test]
    fn iss_inclined_orbit() {
        let a = 6_778_000.0_f64;
        let inc_expected = 51.6_f64.to_radians();
        let v_c = libm::sqrt(MU_EARTH / a);

        let r_vec = [a, 0.0, 0.0];
        let v_vec = [
            0.0,
            v_c * libm::cos(inc_expected),
            v_c * libm::sin(inc_expected),
        ];

        let el = elements_from_state(&r_vec, &v_vec, MU_EARTH).expect("should be Some");

        assert!((el.sma - a).abs() < 100.0, "sma={} expected {a}", el.sma);
        assert!(el.ecc < 1e-4, "ecc={} (should be ~0)", el.ecc);
        assert!(
            (el.inc - inc_expected).abs() < 0.001,
            "inc={} expected {}",
            el.inc,
            inc_expected
        );
    }

    /// TC-C-4: Vis-viva sanity check at both apsides.
    #[test]
    fn vis_viva_apsides_check() {
        let r_per = 6_671_000.0_f64;
        let r_apo = 42_164_000.0_f64;
        let a = (r_per + r_apo) / 2.0;

        let v_per = vis_viva(r_per, a, MU_EARTH);
        let v_apo = vis_viva(r_apo, a, MU_EARTH);

        assert!((v_per - 10_157.7).abs() < 1.0, "v_per = {v_per}");
        assert!((v_apo - 1_607.1).abs() < 1.0, "v_apo = {v_apo}");

        // Angular momentum conservation: v*r = constant at apsides
        let h_per = v_per * r_per;
        let h_apo = v_apo * r_apo;
        assert!(
            (h_per - h_apo).abs() < 1_000.0,
            "h_per={h_per} h_apo={h_apo}"
        );

        // Degenerate cases
        assert_eq!(vis_viva(0.0, a, MU_EARTH), 0.0);
        assert_eq!(vis_viva(r_per, a, 0.0), 0.0);
    }

    /// TC-C-5: Hyperbolic orbit returns correct ecc > 1 and infinite apoapsis.
    #[test]
    fn hyperbolic_orbit_elements() {
        let v_inf = 2_000.0_f64;
        let r_per = 7_000_000.0_f64;
        let a_hyp = -MU_EARTH / (v_inf * v_inf); // negative
        let v_per = libm::sqrt(MU_EARTH * (2.0 / r_per - 1.0 / a_hyp));

        let r_vec = [r_per, 0.0, 0.0];
        let v_vec = [0.0, v_per, 0.0];

        let el = elements_from_state(&r_vec, &v_vec, MU_EARTH).expect("should be Some");
        assert!(el.sma < 0.0, "hyperbolic sma must be negative");
        assert!(el.ecc > 1.0, "hyperbolic ecc must be > 1");

        let (apo, _per) = apoapsis_periapsis(&el);
        assert_eq!(apo, f64::INFINITY, "hyperbolic apoapsis must be infinity");
    }

    /// TC-C-6: Period returns infinity for hyperbolic and zero for invalid mu.
    #[test]
    fn period_edge_cases() {
        assert_eq!(period(-1e10, MU_EARTH), f64::INFINITY);
        assert_eq!(period(0.0, MU_EARTH), f64::INFINITY);
        assert_eq!(period(7e6, 0.0), 0.0);
    }
}
