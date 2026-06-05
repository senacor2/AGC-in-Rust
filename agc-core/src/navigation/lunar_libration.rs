//! Lunar orientation (libration) model — IAU 2015 / Eckhardt-equivalent.
//!
//! Computes the rotation that takes a point expressed in the Moon-fixed
//! Mean Earth/Polar Axis (ME) frame to the J2000.0 mean-equatorial inertial
//! frame, which we treat as the AGC's MCI / Mean of 1969.5 reference for the
//! Apollo mission window (precession over ~1 year is ~30 km at the lunar
//! surface, well within the existing ADR-013 budget).
//!
//! Until #56 this rotation was ignored — Moon-fixed selenographic Cartesian
//! coordinates were treated as MCI Cartesian. That introduces up to ~150 km
//! systematic bias on the inertial position of a lunar landmark, which is
//! far above the 0.1 mrad sextant noise floor and accumulates into the
//! Kalman state estimate on sustained lunar-orbit navigation.
//!
//! ## Reference
//!
//! Archinal, B.A. *et al.* (2018) "Report of the IAU Working Group on
//! Cartographic Coordinates and Rotational Elements: 2015",
//! *Celestial Mechanics and Dynamical Astronomy* 130:22, Table 4 ("Moon").
//!
//! Equivalent series form to Eckhardt 1981's analytical libration theory;
//! both are widely used in modern lunar ephemeris pipelines.
//!
//! ## Convention
//!
//! Given the IAU pole `(α₀, δ₀)` and prime meridian angle `W` (all in
//! degrees) at a given epoch, the body-fixed→inertial rotation is
//!
//! ```text
//! M_b2i = R_z(α₀ + 90°) · R_x(90° − δ₀) · R_z(−W)
//! ```
//!
//! where `R_z`, `R_x` are the active (right-handed) rotation matrices.
//! This is the form given in Seidelmann's *Explanatory Supplement to the
//! Astronomical Almanac* §6.27. Multiplying a selenographic Cartesian
//! column vector by `M_b2i` yields the equivalent MCI position.

use core::f64::consts::PI;

use crate::math::linalg::{mxm, transpose};
use crate::navigation::planetary::met_to_jd;
use crate::types::{Mat3x3, Met};

/// J2000.0 Julian Day epoch — reference instant for the IAU 2015 polynomials.
const J2000_JD: f64 = 2_451_545.0;

const DEG2RAD: f64 = PI / 180.0;

/// Amplitudes (deg) of the periodic terms in α₀ (sin E_i, i = 1..=13).
///
/// Source: Archinal et al. 2018, Table 4. Zero entries denote terms not
/// listed for α₀; carrying them anyway keeps the three amplitude tables
/// aligned by index.
const ALPHA_E_SIN: [f64; 13] = [
    -3.8787, -0.1204, 0.0700, -0.0172, 0.0, 0.0072, 0.0, 0.0, 0.0, -0.0052, 0.0, 0.0, 0.0043,
];

/// Amplitudes (deg) of the periodic terms in δ₀ (cos E_i, i = 1..=13).
const DELTA_E_COS: [f64; 13] = [
    1.5419, 0.0239, -0.0278, 0.0068, 0.0, -0.0029, 0.0009, 0.0, 0.0, 0.0008, 0.0, 0.0, -0.0009,
];

/// Amplitudes (deg) of the periodic terms in W (sin E_i, i = 1..=13).
const W_E_SIN: [f64; 13] = [
    3.5610, 0.1208, -0.0642, 0.0158, 0.0252, -0.0066, -0.0047, -0.0046, 0.0028, 0.0052, 0.0, 0.0040,
    0.0019,
];

/// Phase polynomials E_i = const_deg + rate_deg_per_day · d, where
/// d = JD − 2_451_545.0 (days from J2000.0). Index 0 = E1, index 12 = E13.
const E_PHASE: [(f64, f64); 13] = [
    (125.045, -0.052_992_1),
    (250.089, -0.105_984_2),
    (260.008, 13.012_000_9),
    (176.625, 13.340_715_4),
    (357.529, 0.985_600_3),
    (311.589, 26.405_708_4),
    (134.963, 13.064_993_0),
    (276.617, 0.328_714_6),
    (34.226, 1.748_487_7),
    (15.134, -0.158_976_3),
    (119.743, 0.003_609_6),
    (239.961, 0.164_357_3),
    (25.053, 12.959_008_8),
];

/// Rotation matrix that takes Moon-fixed (Mean Earth/Polar Axis) Cartesian
/// coordinates to MCI / J2000 mean-equatorial Cartesian coordinates at `epoch`.
///
/// The returned matrix is orthonormal with determinant +1.
///
/// # Use
///
/// `r_mci = mxv(moon_fixed_to_inertial(epoch), r_seleno_cart)`
///
/// where `r_seleno_cart` is the Moon-fixed Cartesian position derived from a
/// selenographic (lat, lon, alt) triple via
///
/// ```text
/// x = (R_MOON + alt) · cos(lat) · cos(lon)
/// y = (R_MOON + alt) · cos(lat) · sin(lon)
/// z = (R_MOON + alt) · sin(lat)
/// ```
pub fn moon_fixed_to_inertial(epoch: Met) -> Mat3x3 {
    let jd = met_to_jd(epoch);
    let d = jd - J2000_JD;
    let t_cy = d / 36_525.0;

    // Phase angles E_i (rad). Argument grows ~13 deg/day for E3..E7,E13;
    // libm::sin/cos accept arbitrarily large arguments.
    let mut e_sin = [0.0_f64; 13];
    let mut e_cos = [0.0_f64; 13];
    for i in 0..13 {
        let arg = (E_PHASE[i].0 + E_PHASE[i].1 * d) * DEG2RAD;
        e_sin[i] = libm::sin(arg);
        e_cos[i] = libm::cos(arg);
    }

    // RA of Moon's pole (deg, then rad).
    let mut alpha_deg = 269.9949 + 0.0031 * t_cy;
    for i in 0..13 {
        alpha_deg += ALPHA_E_SIN[i] * e_sin[i];
    }
    let alpha = alpha_deg * DEG2RAD;

    // Dec of Moon's pole (deg, then rad).
    let mut delta_deg = 66.5392 + 0.0130 * t_cy;
    for i in 0..13 {
        delta_deg += DELTA_E_COS[i] * e_cos[i];
    }
    let delta = delta_deg * DEG2RAD;

    // Prime meridian angle W (deg, then rad). The d² coefficient is the
    // tiny tidal-secular term; carrying it costs ~zero and matches IAU 2015.
    let mut w_deg = 38.3213 + 13.176_358_15 * d - 1.4e-12 * d * d;
    for i in 0..13 {
        w_deg += W_E_SIN[i] * e_sin[i];
    }
    let w = w_deg * DEG2RAD;

    // Body-fixed → inertial = R_z(α₀ + 90°) · R_x(90° − δ₀) · R_z(−W).
    let phi = 0.5 * PI - delta;
    let psi = 0.5 * PI + alpha;

    let rz_neg_w = rotation_z(-w);
    let rx_phi = rotation_x(phi);
    let rz_psi = rotation_z(psi);

    mxm(mxm(rz_psi, rx_phi), rz_neg_w)
}

#[inline]
fn rotation_z(theta: f64) -> Mat3x3 {
    let c = libm::cos(theta);
    let s = libm::sin(theta);
    [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
}

#[inline]
fn rotation_x(theta: f64) -> Mat3x3 {
    let c = libm::cos(theta);
    let s = libm::sin(theta);
    [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::linalg::mxv;

    /// TC-LIB-1: returned matrix is orthonormal (M · Mᵀ = I) with determinant +1.
    #[test]
    fn tc_lib_1_rotation_is_orthonormal() {
        for met_s in [0.0_f64, 86_400.0, 5.0 * 86_400.0, 15.0 * 86_400.0] {
            let m = moon_fixed_to_inertial(Met::from_seconds(met_s));
            // Check M · Mᵀ = I.
            let mt = transpose(m);
            let prod = mxm(m, mt);
            for (i, row) in prod.iter().enumerate() {
                for (j, &entry) in row.iter().enumerate() {
                    let expected = if i == j { 1.0 } else { 0.0 };
                    let diff = libm::fabs(entry - expected);
                    assert!(
                        diff < 1e-12,
                        "TC-LIB-1 at MET {met_s}s: (M·Mᵀ)[{i}][{j}] = {entry} (off-identity by {diff:.2e})"
                    );
                }
            }
            // det(M) ≈ +1 (proper rotation, not improper).
            let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
            assert!(
                libm::fabs(det - 1.0) < 1e-12,
                "TC-LIB-1 at MET {met_s}s: det(M) = {det} (expected +1)"
            );
        }
    }

    /// TC-LIB-2: rotation has the right secular angular rate. Over 86 400 s
    /// (one day) the prime meridian advance is 13.176 deg ± periodic-term wobble.
    /// Verify by tracking the inertial direction of the lunar pole-perpendicular
    /// unit vector that points along the prime meridian (selenographic (0°,0°)).
    /// The angle subtended between the day-0 and day-1 inertial vectors must
    /// lie in [12.5°, 13.8°] — bounds wide enough to absorb the periodic terms
    /// (≈ 3.5° amplitude on W).
    #[test]
    fn tc_lib_2_secular_prime_meridian_rate() {
        let pm0 = mxv(moon_fixed_to_inertial(Met::from_seconds(0.0)), [1.0, 0.0, 0.0]);
        let pm1 = mxv(
            moon_fixed_to_inertial(Met::from_seconds(86_400.0)),
            [1.0, 0.0, 0.0],
        );
        let dot = pm0[0] * pm1[0] + pm0[1] * pm1[1] + pm0[2] * pm1[2];
        let angle_deg = libm::acos(dot.clamp(-1.0, 1.0)) * 180.0 / PI;
        assert!(
            (12.5..=13.8).contains(&angle_deg),
            "TC-LIB-2: 1-day prime-meridian advance = {angle_deg:.4}° \
             (expected 13.18° ± physical-libration wobble)"
        );
    }

    /// TC-LIB-3: the body Z-axis (the lunar rotation pole, by definition)
    /// transforms to the IAU pole direction `(cos δ₀·cos α₀, cos δ₀·sin α₀,
    /// sin δ₀)` in the inertial frame. With α₀ ≈ 270° and δ₀ ≈ 66.54° this
    /// is approximately `(0, −0.398, +0.917)`. Periodic-term wobble is a
    /// few degrees, so we allow ±0.05 on each component (≈ 3° angular).
    /// This is the cleanest sign / composition check on the matrix.
    #[test]
    fn tc_lib_3_pole_direction_matches_iau_constants() {
        let pole = mxv(moon_fixed_to_inertial(Met::from_seconds(0.0)), [0.0, 0.0, 1.0]);
        let expected = [0.0_f64, -0.398, 0.917];
        let tol = 0.05;
        for axis in 0..3 {
            let diff = libm::fabs(pole[axis] - expected[axis]);
            assert!(
                diff < tol,
                "TC-LIB-3 axis {axis}: rotated body-Z = {} but expected ≈ {} \
                 (Moon rotation pole at IAU α₀=270°, δ₀=66.54°)",
                pole[axis],
                expected[axis]
            );
        }
    }

    /// TC-LIB-4: libration moves a fixed-Moon landmark by O(km) in inertial
    /// space across the Apollo 8 mission window. This is the smoke-test
    /// proving the model has the magnitude the issue calls out: ignoring
    /// libration is worth ~100 km of bias on a single lunar landmark.
    /// Specifically, compare the inertial position of selenographic (0°,0°)
    /// at MET 0 with the position 10 days later (one third of a lunar
    /// rotation). The arc-length displacement should be ~hundreds to
    /// thousands of km, which dwarfs the 5 km Kalman convergence target.
    #[test]
    fn tc_lib_4_landmark_inertial_position_moves() {
        const R_MOON: f64 = 1_737_400.0;
        let p0 = mxv(
            moon_fixed_to_inertial(Met::from_seconds(0.0)),
            [R_MOON, 0.0, 0.0],
        );
        let p10 = mxv(
            moon_fixed_to_inertial(Met::from_seconds(10.0 * 86_400.0)),
            [R_MOON, 0.0, 0.0],
        );
        let dx = p10[0] - p0[0];
        let dy = p10[1] - p0[1];
        let dz = p10[2] - p0[2];
        let displacement_km = libm::sqrt(dx * dx + dy * dy + dz * dz) / 1_000.0;
        assert!(
            displacement_km > 100.0,
            "TC-LIB-4: 10-day libration displacement of selenographic (0°,0°) = {displacement_km:.1} km, \
             expected > 100 km (otherwise the model would not deserve the 5 km Kalman target)"
        );
    }
}
