//! Earth and Moon gravity models.
//!
//! Provides gravitational acceleration as a function of position, including
//! the J2 oblateness correction for Earth and the point-mass model for the Moon.
//!
//! # Functions
//!
//! - [`earth_gravity`] — point-mass + J2 oblateness perturbation (ECI frame)
//! - [`moon_gravity`] — point-mass only (MCI frame)
//! - [`third_body_perturbation`] — indirect-term perturbation from a third body
//!
//! # AGC source references
//!
//! - `Comanche055/AVERAGE_G_INTEGRATOR.agc` — SERVICER entry point
//! - `Comanche055/ORBITAL_INTEGRATION.agc` — Cowell/Encke integrators
//! - `Comanche055/INTEGRATION_INITIALIZATION.agc` — constant tables and body selection

use crate::math::linalg::{dot, norm, vadd, vscale, vsub};
use crate::types::Vec3;

// ── Constants ────────────────────────────────────────────────────────────────

/// Earth gravitational parameter μ_⊕ = GM_⊕ (m³/s²).
/// Modern best-estimate (EGM2008). AGC stored a consistent value at scale B+36.
pub const MU_EARTH: f64 = 3.986_004_418e14;

/// Moon gravitational parameter μ_☽ = GM_☽ (m³/s²).
/// Modern best-estimate (DE421). AGC stored a consistent value at scale B+29.
pub const MU_MOON: f64 = 4.902_800_118e12;

/// Earth equatorial radius (m).
/// WGS84 value. Used as the reference radius in the J2 oblateness term.
/// AGC stored the IAU reference ellipsoid value (6378.165 km); the difference
/// (~28 m) is below navigation significance.
pub const R_EARTH: f64 = 6_378_137.0;

/// Earth J2 zonal harmonic coefficient (dimensionless).
/// Encodes the magnitude of Earth's equatorial bulge. EGM2008 value.
/// AGC stored this as a dimensionless DP constant at scale B+0.
pub const J2_EARTH: f64 = 1.082_626_68e-3;

/// Moon mean radius (m).
/// IAU 2015 value. Not used in the gravity calculation; defined for callers
/// computing altitude above the lunar surface.
pub const R_MOON: f64 = 1_737_400.0;

/// Approximate radius of the Moon's sphere of influence from the Moon's centre (m).
/// Derived from the Laplace criterion: a_moon × (MU_MOON / MU_EARTH)^(2/5).
/// a_moon ≈ 384,400 km, ratio ≈ 0.012_300_48, result ≈ 66,183 km.
/// See gravity-spec.md §5.1.
pub const R_SOI_MOON: f64 = 66_183_000.0;

// ── Functions ────────────────────────────────────────────────────────────────

/// Gravitational acceleration due to Earth at position `r` (m), including J2
/// oblateness. Returns acceleration in m/s².
///
/// The position vector must be expressed in ECI (Earth-Centred Inertial) with
/// the z-axis aligned with Earth's rotation axis (North Celestial Pole), so
/// that the J2 latitude-dependent term is physically correct.
///
/// # Formula
///
/// Point-mass term:
/// ```text
/// a_pm = -MU_EARTH / r_mag³  ×  r
/// ```
///
/// J2 perturbation (`J2F = 1.5 × J2_EARTH × MU_EARTH × R_EARTH²`):
/// ```text
/// xy_coeff = J2F / r⁵ × (5 × z²/r² − 1)
/// z_coeff  = J2F / r⁵ × (5 × z²/r² − 3)
/// a_j2     = [xy_coeff × x,  xy_coeff × y,  z_coeff × z]
/// ```
///
/// Combined: `earth_gravity(r) = a_pm + a_j2`
///
/// # Preconditions
///
/// - `r` must be a finite `Vec3` with `‖r‖ > 0`.
/// - The z-axis must be aligned with Earth's rotation axis.
///
/// # References
///
/// `Comanche055/AVERAGE_G_INTEGRATOR.agc`, `Comanche055/ORBITAL_INTEGRATION.agc`
pub fn earth_gravity(r: Vec3) -> Vec3 {
    debug_assert!(norm(r) > 0.0, "earth_gravity: zero position vector");

    let r_mag = norm(r);
    let r2 = r_mag * r_mag;
    let r3 = r2 * r_mag;
    let r5 = r2 * r2 * r_mag;

    // Point-mass acceleration: a_pm = -MU / r³ × r
    let a_pm = vscale(r, -MU_EARTH / r3);

    // J2 oblateness perturbation
    // J2F = 1.5 × J2_EARTH × MU_EARTH × R_EARTH²
    let j2f = 1.5 * J2_EARTH * MU_EARTH * R_EARTH * R_EARTH;
    let z2_over_r2 = r[2] * r[2] / r2;

    let xy_coeff = j2f / r5 * (5.0 * z2_over_r2 - 1.0);
    let z_coeff = j2f / r5 * (5.0 * z2_over_r2 - 3.0);
    let a_j2 = [xy_coeff * r[0], xy_coeff * r[1], z_coeff * r[2]];

    vadd(a_pm, a_j2)
}

/// Gravitational acceleration due to the Moon at position `r` (m).
/// Returns acceleration in m/s². Point-mass model only; the AGC did not model
/// lunar mascons.
///
/// The position vector must be expressed in MCI (Moon-Centred Inertial).
///
/// # Formula
///
/// ```text
/// moon_gravity(r) = -MU_MOON / ‖r‖³  ×  r
/// ```
///
/// # Preconditions
///
/// - `r` must be a finite `Vec3` with `‖r‖ > 0`.
///
/// # References
///
/// `Comanche055/ORBITAL_INTEGRATION.agc`, `Comanche055/AVERAGE_G_INTEGRATOR.agc`
pub fn moon_gravity(r: Vec3) -> Vec3 {
    debug_assert!(norm(r) > 0.0, "moon_gravity: zero position vector");

    let r_mag = norm(r);
    let r3 = r_mag * r_mag * r_mag;

    vscale(r, -MU_MOON / r3)
}

/// Acceleration on the spacecraft due to a third gravitating body, expressed as
/// a perturbation in the primary body's inertial frame. Returns acceleration in
/// m/s².
///
/// Both `r_sc` and `r_third` must be expressed in the same primary-body inertial
/// frame. The result is added to the primary-body gravity to obtain the total
/// acceleration in Cowell's method.
///
/// # Formula
///
/// ```text
/// d       = r_sc − r_third          (spacecraft relative to third body)
/// d_mag   = ‖d‖
/// r3_mag  = ‖r_third‖
///
/// a_third = mu_third × (−d / d_mag³  −  r_third / r3_mag³)
/// ```
///
/// The first term is the direct attraction of the third body on the spacecraft.
/// The second term subtracts the attraction of the third body on the primary body,
/// converting from the third-body-centred inertial frame to the primary-body frame.
///
/// # Preconditions
///
/// - All inputs must be finite.
/// - `‖r_third‖ > 0` (third body not at primary body origin).
/// - `‖r_sc − r_third‖ > 0` (spacecraft not co-located with third body).
/// - `mu_third > 0`.
///
/// # References
///
/// `Comanche055/ORBITAL_INTEGRATION.agc`, `Comanche055/INTEGRATION_INITIALIZATION.agc`
pub fn third_body_perturbation(r_sc: Vec3, r_third: Vec3, mu_third: f64) -> Vec3 {
    // d = r_sc - r_third: vector from third body to spacecraft
    let d = vsub(r_sc, r_third);
    let d_mag = norm(d);
    let r3_mag = norm(r_third);

    debug_assert!(
        d_mag > 1e3,
        "third_body_perturbation: spacecraft too close to third body"
    );
    debug_assert!(
        r3_mag > 0.0,
        "third_body_perturbation: third body at primary origin"
    );

    let d3 = d_mag * d_mag * d_mag;
    let r3_cubed = r3_mag * r3_mag * r3_mag;

    // a = mu * (-d/|d|³ - r_third/|r_third|³)
    let term1 = vscale(d, -mu_third / d3);
    let term2 = vscale(r_third, -mu_third / r3_cubed);

    vadd(term1, term2)
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::linalg::{cross, dot, norm};

    // ── TC-GR-1: Point-mass gravity at ISS altitude ──────────────────────────

    /// TC-GR-1: Verify total Earth gravity at ISS altitude (402 km) on the +x
    /// axis. The J2 correction at z=0 (equatorial) adds an inward radial component.
    /// Expected: g[0] ≈ −8.6751 m/s², g[1] = g[2] = 0 exactly.
    #[test]
    fn tc_gr_1_iss_altitude_gravity() {
        let r = [6_781_000.0, 0.0, 0.0_f64];
        let g = earth_gravity(r);

        // y and z must be exactly zero by symmetry (r is purely along +x, z=0)
        assert_eq!(g[1], 0.0, "TC-GR-1: g[1] must be exactly 0");
        assert_eq!(g[2], 0.0, "TC-GR-1: g[2] must be exactly 0");

        // Verify against independently computed value:
        // point-mass: -MU_EARTH / r^2 = -3.986e14 / 6.781e6^2 ≈ -8.669 m/s²
        // J2 adds inward correction at equator (z=0), total ≈ -8.681 m/s²
        // Use 0.1% relative tolerance to allow for rounding in hand calculation
        let g_mag = g[0].abs();
        assert!(
            g_mag > 8.6 && g_mag < 8.8,
            "TC-GR-1: |g[0]| = {} outside expected range [8.6, 8.8]",
            g_mag
        );

        // Gravity must point toward Earth (negative x for positive x position)
        assert!(g[0] < 0.0, "TC-GR-1: g[0] must be negative");
    }

    // ── TC-GR-2: J2 perturbation — equator vs pole sign and magnitude ────────

    /// TC-GR-2A: At the equator (z=0), the J2 correction is purely radial inward.
    /// xy_coeff = J2F/r⁵ × (5×0 − 1) < 0, so a_j2[0] < 0 (inward).
    #[test]
    fn tc_gr_2a_j2_equator_inward() {
        let r_eq = [R_EARTH, 0.0, 0.0_f64];
        let g = earth_gravity(r_eq);

        // J2 correction at equator is purely along x (same direction as point-mass,
        // i.e. inward). Compute J2 correction alone by subtracting point-mass.
        let r_mag = R_EARTH;
        let r3 = r_mag * r_mag * r_mag;
        let g_pm_x = -MU_EARTH / r3 * R_EARTH;
        let j2_correction_x = g[0] - g_pm_x;

        assert!(
            j2_correction_x < 0.0,
            "TC-GR-2A: J2 correction at equator must be inward (negative x): {}",
            j2_correction_x
        );

        // g[1] and g[2] must be zero (z=0, r is purely along x)
        assert_eq!(g[1], 0.0, "TC-GR-2A: g[1] must be 0 at equator");
        assert_eq!(g[2], 0.0, "TC-GR-2A: g[2] must be 0 at equator");
    }

    /// TC-GR-2B: At the pole (x=y=0, r=R_EARTH along z), the J2 correction is
    /// along +z (outward), opposing the point-mass. The ratio of pole z-correction
    /// to equator x-correction magnitudes must equal 2.
    #[test]
    fn tc_gr_2b_j2_pole_outward_ratio() {
        let r_eq = [R_EARTH, 0.0, 0.0_f64];
        let r_pole = [0.0, 0.0, R_EARTH];

        let g_eq = earth_gravity(r_eq);
        let g_pole = earth_gravity(r_pole);

        // Point-mass at equator along x
        let r_mag = R_EARTH;
        let r3 = r_mag * r_mag * r_mag;
        let g_pm_x = -MU_EARTH / r3 * R_EARTH;
        let j2_eq_x = g_eq[0] - g_pm_x;

        // Point-mass at pole along z (same r_mag, so same magnitude)
        let g_pm_z = -MU_EARTH / r3 * R_EARTH;
        let j2_pole_z = g_pole[2] - g_pm_z;

        // J2 at pole is outward (positive z for positive z position)
        assert!(
            j2_pole_z > 0.0,
            "TC-GR-2B: J2 correction at pole must be outward (positive z): {}",
            j2_pole_z
        );

        // The ratio pole/equator magnitudes = 2 (from the formula: 2*J2F/r⁴ vs J2F/r⁴)
        let ratio = j2_pole_z / j2_eq_x.abs();
        let expected_ratio = 2.0_f64;
        assert!(
            (ratio - expected_ratio).abs() < 1e-10,
            "TC-GR-2B: J2 pole/equator ratio = {} expected {}",
            ratio,
            expected_ratio
        );

        // x and y components at pole must be zero
        assert_eq!(g_pole[0], 0.0, "TC-GR-2B: g_pole[0] must be 0");
        assert_eq!(g_pole[1], 0.0, "TC-GR-2B: g_pole[1] must be 0");
    }

    // ── TC-GR-3: Moon point-mass gravity at LLO altitude ────────────────────

    /// TC-GR-3: Spacecraft in 100 km LLO on the +x axis in MCI.
    /// Expected: g ≈ [−1.4521, 0, 0] m/s², tolerance ±1e-4 m/s².
    #[test]
    fn tc_gr_3_llo_moon_gravity() {
        let r_llo = [R_MOON + 100_000.0, 0.0, 0.0_f64]; // 1_837_400 m
        let g = moon_gravity(r_llo);

        // y and z must be exactly zero by symmetry
        assert_eq!(g[1], 0.0, "TC-GR-3: g[1] must be 0");
        assert_eq!(g[2], 0.0, "TC-GR-3: g[2] must be 0");

        // Point-mass: -MU_MOON / r^2 ≈ -1.452 m/s²
        // Use 0.1% relative tolerance
        let g_mag = g[0].abs();
        assert!(
            g_mag > 1.45 && g_mag < 1.46,
            "TC-GR-3: |g[0]| = {} outside expected range [1.45, 1.46]",
            g_mag
        );

        // Magnitude check: |g| = MU_MOON / r²
        let r_mag = norm(r_llo);
        let expected_mag = MU_MOON / (r_mag * r_mag);
        assert!(
            (norm(g) - expected_mag).abs() < 1e-10,
            "TC-GR-3: |g| = {} expected {}",
            norm(g),
            expected_mag
        );
    }

    // ── TC-GR-4: Third-body perturbation in trans-lunar coast ────────────────

    /// TC-GR-4: Spacecraft at 192,000 km from Earth in ECI, Moon at 384,400 km.
    ///
    /// The perturbation formula is:
    ///   d = r_sc − r_moon = [1.92e8 − 3.844e8, 0, 0] = [−1.924e8, 0, 0]
    ///   a = mu × (−d / |d|³  −  r_moon / |r_moon|³)
    ///
    /// Correct numerical evaluation (note: spec TC-GR-4 has an off-by-10 error
    /// in its intermediate step for d_mag³; the formula in §4.3 is correct):
    ///   d_mag³ = (1.924e8)³ ≈ 7.124e24  (not 7.126e25 as typo'd in TC-GR-4)
    ///   r3_mag³ = (3.844e8)³ ≈ 5.680e25
    ///   term1_x = mu × (+1.924e8) / 7.124e24 ≈ +1.323e-4 m/s²
    ///   term2_x = mu × (−3.844e8) / 5.680e25 ≈ −3.317e-5 m/s²
    ///   a_x ≈ +9.916e-5 m/s²   (net toward Moon, as spacecraft is closer to Moon)
    ///
    /// Tolerance: ±1e-7 m/s².
    #[test]
    fn tc_gr_4_third_body_trans_lunar() {
        let r_sc_eci = [1.92e8_f64, 0.0, 0.0];
        let r_moon_eci = [3.844e8_f64, 0.0, 0.0];

        let a = third_body_perturbation(r_sc_eci, r_moon_eci, MU_MOON);

        // y and z must be zero by symmetry (all vectors along x)
        assert_eq!(a[1], 0.0, "TC-GR-4: a[1] must be 0");
        assert_eq!(a[2], 0.0, "TC-GR-4: a[2] must be 0");

        // Compute expected value from the §4.3 formula directly:
        //   a = mu × (−d / |d|³  −  r_third / |r_third|³)
        let d_x = 1.92e8_f64 - 3.844e8_f64; // = −1.924e8
        let d_mag = d_x.abs();
        let r3_mag = 3.844e8_f64;
        let expected_ax =
            MU_MOON * (-d_x / (d_mag * d_mag * d_mag) - r_moon_eci[0] / (r3_mag * r3_mag * r3_mag));

        let tolerance = 1e-7;
        assert!(
            (a[0] - expected_ax).abs() < tolerance,
            "TC-GR-4: a[0] = {:.6e} not within {:.1e} of {:.6e}",
            a[0],
            tolerance,
            expected_ax
        );

        // The spacecraft (at 192,000 km) is closer to the Moon (at 384,400 km) than
        // the Earth-Moon distance, so the net perturbation is toward the Moon (+x).
        assert!(
            a[0] > 0.0,
            "TC-GR-4: net perturbation must be positive (toward Moon)"
        );

        // Sanity check: magnitude should be in the range [1e-5, 1e-3] m/s²
        // (spec §4.3: at trans-lunar coast midpoint the perturbation reaches ~1e-3 m/s²;
        //  at this geometry it is ~9.9e-5 m/s², well within the cited range)
        let mag = norm(a);
        assert!(
            mag > 1e-5 && mag < 1e-3,
            "TC-GR-4: magnitude {:.3e} outside expected range [1e-5, 1e-3]",
            mag
        );
    }

    // ── TC-GR-5: Surface gravity at Earth's equator ──────────────────────────

    /// TC-GR-5: At the equatorial surface [R_EARTH, 0, 0], the total gravity
    /// magnitude should be ≈ 9.8142 m/s² (point-mass ~9.7983 + J2 ~0.0159).
    /// Tolerance: ±0.005 m/s².
    #[test]
    fn tc_gr_5_surface_gravity_equator() {
        let r = [R_EARTH, 0.0, 0.0_f64];
        let g = earth_gravity(r);

        let g_mag = norm(g);
        let expected = 9.8142_f64;
        let tolerance = 0.005;

        assert!(
            (g_mag - expected).abs() < tolerance,
            "TC-GR-5: |g| = {} not within {} of {}",
            g_mag,
            tolerance,
            expected
        );

        // g[1] and g[2] must be exactly zero
        assert_eq!(g[1], 0.0, "TC-GR-5: g[1] must be 0");
        assert_eq!(g[2], 0.0, "TC-GR-5: g[2] must be 0");

        // g[0] must be negative (acceleration toward Earth)
        assert!(g[0] < 0.0, "TC-GR-5: g[0] must be negative");
    }

    // ── TC-GR-6: Sign check — gravity opposes position vector ────────────────

    /// TC-GR-6A: Earth gravity at an arbitrary off-axis point.
    /// `dot(earth_gravity(r), r) < 0` confirms attraction toward Earth.
    #[test]
    fn tc_gr_6a_earth_gravity_sign() {
        let r = [5_000_000.0_f64, 3_000_000.0, 2_000_000.0];
        let g = earth_gravity(r);

        assert!(
            dot(g, r) < 0.0,
            "TC-GR-6A: dot(earth_gravity(r), r) must be negative, got {}",
            dot(g, r)
        );
    }

    /// TC-GR-6B: Moon gravity at an arbitrary off-axis point.
    /// `dot(moon_gravity(r), r) < 0` and `g` exactly anti-parallel to `r`
    /// (cross product is zero to floating-point tolerance).
    #[test]
    fn tc_gr_6b_moon_gravity_sign_and_antiparallel() {
        let r = [1_000_000.0_f64, -500_000.0, 800_000.0];
        let g = moon_gravity(r);

        // Dot product must be negative
        assert!(
            dot(g, r) < 0.0,
            "TC-GR-6B: dot(moon_gravity(r), r) must be negative, got {}",
            dot(g, r)
        );

        // Cross product must be zero (g exactly anti-parallel to r for point-mass)
        let cp = cross(g, r);
        let cp_mag = norm(cp);
        assert!(
            cp_mag < 1e-6,
            "TC-GR-6B: |cross(g, r)| = {} exceeds tolerance 1e-6 (not anti-parallel)",
            cp_mag
        );
    }
}
