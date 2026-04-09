//! Gravitational acceleration models.
//!
//! AGC source: Comanche055/ORBITAL_INTEGRATION.agc — gravity terms used in
//! the Cowell integrator (ORINIT/INTGRATE).

use crate::types::Vec3;

/// Earth gravitational parameter (m³/s²).
///
/// AGC source: ORBITAL_INTEGRATION.agc — `MUEARTH 3.986032 E10 B-36`.
/// Using the AGC value for fidelity (per CLAUDE.md: "fidelity wins").
/// The modern IAU value is 3.986_004_418e14; the AGC value is ~3.5×10⁻⁶
/// larger, which accumulates to tens of metres over a translunar trajectory.
pub const MU_EARTH: f64 = 3.986_032e14;

/// Moon gravitational parameter (m³/s²).
///
/// AGC source: ORBITAL_INTEGRATION.agc — approximately 4.9027780×10¹² m³/s².
/// Using the AGC value for fidelity.
pub const MU_MOON: f64 = 4.902_778e12;

/// Earth equatorial radius used in the oblateness (J2) model (m).
///
/// AGC source: ORBITAL_INTEGRATION.agc — `RSPHERE` constant, ~6373.338 km.
/// The AGC used a slightly smaller value than WGS84 (6378.137 km); using
/// the AGC value keeps the J2 perturbation consistent with the source.
pub const RE_EARTH: f64 = 6_373_338.0;

/// Earth J2 oblateness coefficient (dimensionless).
///
/// AGC source: ORBITAL_INTEGRATION.agc — J2 term coefficient.
/// Note: the AGC also includes J3 and J4 Legendre terms (ORBITAL_INTEGRATION.agc
/// defines `2J3RE/J2` and `J4REQ/J3`). Those higher-order terms are out of
/// scope for Milestone 2 and will be added in the guidance milestone.
pub const J2_EARTH: f64 = 1.082_626_68e-3;

/// Point-mass gravitational acceleration at position `r` relative to body with GM=`mu`.
///
/// Returns acceleration in m/s². This is the dominant term in the Cowell integrator.
/// Protects against singularity: if |r| < 1.0 m, returns the zero vector.
///
/// AGC source: ORBITAL_INTEGRATION.agc — ORINIT/INTGRATE gravity call.
pub fn point_mass(r: &Vec3, mu: f64) -> Vec3 {
    let r2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
    let r_norm = libm::sqrt(r2);
    if r_norm < 1.0 {
        return [0.0; 3];
    }
    let r3 = r_norm * r2;
    let factor = -mu / r3;
    [r[0] * factor, r[1] * factor, r[2] * factor]
}

/// Earth J2 oblateness perturbation acceleration at ECI position `r`.
///
/// AGC source: ORBITAL_INTEGRATION.agc — J2 perturbation term.
pub fn j2_perturbation(r: &Vec3) -> Vec3 {
    let r2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
    let r_norm = libm::sqrt(r2);
    if r_norm < 1.0 {
        return [0.0; 3];
    }
    let r5 = r_norm * r2 * r2;
    let factor = 1.5 * J2_EARTH * MU_EARTH * RE_EARTH * RE_EARTH / r5;
    let z_over_r2 = (r[2] / r_norm) * (r[2] / r_norm);
    [
        factor * r[0] * (1.0 - 5.0 * z_over_r2),
        factor * r[1] * (1.0 - 5.0 * z_over_r2),
        factor * r[2] * (3.0 - 5.0 * z_over_r2),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn point_mass_at_low_orbit_points_inward() {
        // At [7e6, 0, 0] the acceleration must be in the −x direction.
        let r = [7_000_000.0, 0.0, 0.0];
        let a = point_mass(&r, MU_EARTH);
        assert!(a[0] < 0.0, "acceleration should be in -x direction");
        assert_eq!(a[1], 0.0);
        assert_eq!(a[2], 0.0);
        // Magnitude check: a ≈ μ/r² ≈ 8.15 m/s²
        let mag = libm::sqrt(a[0] * a[0]);
        let expected = MU_EARTH / (7_000_000.0f64 * 7_000_000.0);
        assert!(
            (mag - expected).abs() < 1e-4,
            "magnitude: {} vs {}",
            mag,
            expected
        );
    }

    #[test]
    fn point_mass_zero_radius_returns_zero() {
        let r = [0.0; 3];
        let a = point_mass(&r, MU_EARTH);
        assert_eq!(a, [0.0; 3]);
    }

    #[test]
    fn j2_at_equatorial_point_has_no_z() {
        // On the equatorial plane (z=0), the J2 z-component should be zero.
        let r = [7_000_000.0, 0.0, 0.0];
        let a = j2_perturbation(&r);
        assert_eq!(a[2], 0.0);
        // x-component should be non-zero (J2 stretches equatorial radius)
        assert!(a[0].abs() > 0.0);
    }

    #[test]
    fn j2_at_pole_acceleration_is_radial() {
        // At a polar point [0, 0, r], the x and y components should be zero.
        let r = [0.0, 0.0, 7_000_000.0];
        let a = j2_perturbation(&r);
        assert!(a[0].abs() < 1e-10, "ax = {}", a[0]);
        assert!(a[1].abs() < 1e-10, "ay = {}", a[1]);
    }
}
