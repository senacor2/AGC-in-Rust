//! Gravitational acceleration models: Earth point-mass + J2 oblateness, Moon point-mass.
//!
//! Implements the CALCGRAV routine from SERVICER207.agc (used in the 2-second SERVICER
//! cycle) and the OBLATE routine from ORBITAL_INTEGRATION.agc (J2 perturbation).
//!
//! AGC source: Comanche055/ORBITAL_INTEGRATION.agc
//!   OBLATE (J2 oblateness acceleration, pages 1341-1343),
//!   GAMCOMP (gravity computation subroutine, pages 1338-1340),
//!   ACCOMP  (acceleration component dispatcher, pages 1337-1338),
//!
//! AGC source: Comanche055/SERVICER207.agc
//!   CALCGRAV (point-mass + J2 for SERVICER, pages 835-836),
//!   ITISMOON (Moon-gravity branch in CALCGRAV).
//!
//! AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
//!   PBODY, MOONFLAG (primary body selector).

use crate::math::linalg::{add, dot, norm, scale, sub, unit};
use crate::navigation::constants::{J2_EARTH, MU_EARTH, MU_MOON, RE_EARTH, RSPHERE, R_MIN_GUARD};
use crate::navigation::state_vector::PrimaryBody;
use crate::types::{Met, Vec3, ZERO_VEC3};

/// Earth's rotation unit vector in ECI coordinates.
///
/// For ECI aligned with the Earth's rotation axis, UNITW = [0, 0, 1].
///
/// AGC source: `Comanche055/SERVICER207.agc` CALCGRAV uses `UNITW` erasable register.
/// AGC source: `Comanche055/ERASABLE_ASSIGNMENTS.agc` `UNITW ERASE +5`.
///
/// // TODO(M3): make UNITW a parameter fed from the navigation base frame.
const UNITW: Vec3 = [0.0, 0.0, 1.0];

/// Gravitational acceleration from Earth: point mass + J2 oblateness.
///
/// Computes:
///   a_pm = −(MU_EARTH / |r|³) · r            (point-mass term)
///   a_J2 = J2 correction using J2_EARTH, RE_EARTH
///   return a_pm + a_J2
///
/// The J2 formula in ECI coordinates (derived from OBLATE / CALCGRAV):
///   Let u = r / |r|,  rMag = |r|,  z_hat = [0, 0, 1] (ECI pole = UNITW)
///   k = (3/2) * J2_EARTH * MU_EARTH * RE_EARTH^2 / rMag^5
///   a_J2 = k * [ (5 * (u·z_hat)^2 − 1) * u  −  2 * (u·z_hat) * z_hat ]
///
/// Note: CALCGRAV uses UNITW (Earth rotation unit vector).
/// For ECI, UNITW ≈ z_hat = [0, 0, 1]. The Rust port uses the fixed z_hat.
/// // TODO(M3): pass UNITW dynamically when non-zero x/y components are needed.
///
/// Invariant: never returns NaN or infinity for finite r with |r| > R_MIN_GUARD.
/// Singularity guard: if |r| <= R_MIN_GUARD (= 1.0 m), returns [0.0, 0.0, 0.0].
///
/// Input: r in ECI metres. Output: acceleration in m/s².
///
/// AGC source: `Comanche055/SERVICER207.agc`, CALCGRAV routine (page 835);
///             `Comanche055/ORBITAL_INTEGRATION.agc`, OBLATE routine (page 1341).
/// // TODO: add J3/J4 per ORBITAL_INTEGRATION.agc OBLATE routine
pub fn earth_gravity(r: &Vec3) -> Vec3 {
    let r_mag = norm(r);
    if r_mag <= R_MIN_GUARD {
        return ZERO_VEC3;
    }

    // Point-mass term: a_pm = −(MU_EARTH / r_mag^3) * r
    let r_mag3 = r_mag * r_mag * r_mag;
    let pm_coeff = -MU_EARTH / r_mag3;
    let a_pm = scale(r, pm_coeff);

    // J2 oblateness term
    // u = r / |r| (unit position vector)
    let u = match unit(r) {
        Some(u) => u,
        None => return a_pm, // |r| effectively zero, return point-mass only
    };

    // sin(lat) = u · UNITW (sine of geocentric latitude)
    let sin_lat = dot(&u, &UNITW);

    // Standard ECI J2 acceleration (derived from OBLATE/CALCGRAV):
    //   k = (3/2) * J2 * MU * RE^2 / r^4
    //   a_J2 = k * [(1 - 5*sin_lat^2) * u + 2*sin_lat * z_hat]
    //
    // This matches the AGC source terms (SERVICER207.agc 20J/2J constants):
    //   - At equator (sin_lat=0): a_J2 = k*u (outward), reducing radial gravity
    //   - At pole (sin_lat=1):    a_J2 = k*(-4*u + 2*z_hat) = -2k*u (inward),
    //     increasing polar gravity. Consistent with actual Earth: poles ~9.832 m/s²
    //     vs equator ~9.780 m/s².
    let r_mag4 = r_mag3 * r_mag;
    let k = 1.5 * J2_EARTH * MU_EARTH * RE_EARTH * RE_EARTH / r_mag4;

    // a_J2 = k * [(1 - 5*sin_lat^2)*u + 2*sin_lat*z_hat]
    let coeff_u = k * (1.0 - 5.0 * sin_lat * sin_lat);
    let coeff_w = 2.0 * k * sin_lat;
    let a_j2 = add(&scale(&u, coeff_u), &scale(&UNITW, coeff_w));

    add(&a_pm, &a_j2)
}

/// Moon position stub for Milestone 2.
///
/// Returns a fixed vector at the mean lunar distance in ECI coordinates.
/// A real ephemeris call (LSPOS / LUNPOS in ORBITAL_INTEGRATION.agc) is required for M3.
///
/// Output: ECI position of the Moon's centre of mass, metres.
///
/// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc`, LSPOS / LUNPOS routines.
/// // TODO(M3): replace with LSPOS/LUNPOS ephemeris computation.
fn moon_position_eci(_t: Met) -> Vec3 {
    // Mean lunar distance: 384 400 km = 384_400_000 m
    [384_400_000.0, 0.0, 0.0]
}

/// Gravitational acceleration from the Moon (point-mass only).
///
/// Computes:
///   r_moon_eci = moon_position_eci(t)    (stub for M2; returns fixed mean distance)
///   delta_r = r - r_moon_eci             (vehicle position relative to Moon)
///   a = −(MU_MOON / |delta_r|³) · delta_r
///
/// Singularity guard: if |delta_r| <= R_MIN_GUARD, returns [0.0, 0.0, 0.0].
///
/// Input: r in ECI metres, t is MET (for future ephemeris stub). Output: m/s².
///
/// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc`, ACCOMP routine (line ~130),
///             ITISMOON branch in CALCGRAV (`Comanche055/SERVICER207.agc` page 835).
pub fn moon_gravity(r: &Vec3, t: Met) -> Vec3 {
    let r_moon = moon_position_eci(t);
    // delta_r = vehicle - moon (so gravity accelerates vehicle toward Moon)
    let delta_r = sub(r, &r_moon);
    let delta_mag = norm(&delta_r);
    if delta_mag <= R_MIN_GUARD {
        return ZERO_VEC3;
    }
    let delta_mag3 = delta_mag * delta_mag * delta_mag;
    let coeff = -MU_MOON / delta_mag3;
    scale(&delta_r, coeff)
}

/// Total gravitational acceleration switching on the primary body.
///
/// When `primary = PrimaryBody::Earth`:
///   Returns `earth_gravity(r) + moon_gravity(r, t)` [Earth + lunar perturbation].
///
/// When `primary = PrimaryBody::Moon`:
///   Moon-centred integration with Earth as third body — out of scope for M2.
///   Returns `moon_gravity(r, t)` only.
///   // TODO(M3): add Earth point-mass perturbation for Moon-centred integration.
///
/// The AGC switched primary body at the sphere-of-influence (RSPHERE) boundary
/// via DOSWITCH / ORIGCHNG routines in ORBITAL_INTEGRATION.agc.
///
/// Input: r in ECI metres, t MET centiseconds, primary body selector.
/// Output: total gravitational acceleration, m/s².
///
/// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc`, CHKSWTCH / DOSWITCH / ORIGCHNG
///             (pages 1345-1346); MOONFLAG bit in `ERASABLE_ASSIGNMENTS.agc`.
pub fn total_gravity(r: &Vec3, t: Met, primary: PrimaryBody) -> Vec3 {
    match primary {
        PrimaryBody::Earth => {
            let a_earth = earth_gravity(r);
            let a_moon = moon_gravity(r, t);
            add(&a_earth, &a_moon)
        }
        PrimaryBody::Moon => {
            // TODO(M3): add Earth point-mass perturbation for Moon-centred integration.
            moon_gravity(r, t)
        }
    }
}

/// Returns `true` if the vehicle's ECI distance from Earth exceeds the sphere-of-influence
/// radius (RSPHERE = 64 373 760 m), signalling that Moon-centred integration should be
/// activated.
///
/// AGC: `Comanche055/ORBITAL_INTEGRATION.agc`, CHKSWTCH routine checks |RCV + RCONIC| vs RSPHERE.
///
/// Input: r_eci in metres (ECI position). Output: bool.
pub fn sphere_of_influence_check(r_eci: &Vec3) -> bool {
    norm(r_eci) > RSPHERE
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::constants::{MU_EARTH, RE_EARTH, RSPHERE};
    use crate::navigation::state_vector::PrimaryBody;
    use crate::types::Met;

    /// Test 1 — Earth surface gravity magnitude.
    /// Derived from CALCGRAV routine: MU_EARTH / RE_EARTH^2 ≈ 9.806 m/s².
    #[test]
    fn earth_surface_gravity() {
        let r = [RE_EARTH, 0.0, 0.0];
        let a = earth_gravity(&r);
        // Acceleration points toward Earth (negative x at equator position)
        assert!(a[0] < 0.0, "gravity must point toward Earth, a[0]={}", a[0]);
        assert!(
            (a[0].abs() - 9.80).abs() < 0.02,
            "|g| at surface should be ≈9.80 m/s², got {}",
            a[0].abs()
        );
        assert!(
            a[1].abs() < 1e-10,
            "no transverse y on equator, a[1]={}",
            a[1]
        );
        assert!(
            a[2].abs() < 1e-10,
            "no transverse z on equator, a[2]={}",
            a[2]
        );
    }

    /// Test 2 — GEO altitude (point-mass dominated, J2 << PM).
    #[test]
    fn geo_gravity() {
        let r_geo = [42_164_000.0_f64, 0.0, 0.0];
        let a = earth_gravity(&r_geo);
        // Point-mass: MU_EARTH / r_geo^2
        let r_geo_scalar = 42_164_000.0_f64;
        let a_pm_expected = MU_EARTH / (r_geo_scalar * r_geo_scalar);
        assert!(
            (a[0].abs() - a_pm_expected).abs() / a_pm_expected < 1e-4,
            "GEO gravity: got {}, expected ~{a_pm_expected}",
            a[0].abs()
        );
    }

    /// Test 3 — J2 polar vs equatorial bias.
    /// At the pole, J2 increases |g|; at the equator, J2 reduces it.
    #[test]
    fn j2_polar_equatorial_bias() {
        let r_equator = [RE_EARTH, 0.0, 0.0];
        let r_pole = [0.0, 0.0, RE_EARTH];
        let a_eq = earth_gravity(&r_equator);
        let a_pol = earth_gravity(&r_pole);

        // Polar gravity (along z-axis, pointing toward origin) should be slightly stronger
        assert!(
            a_pol[2].abs() > a_eq[0].abs(),
            "polar |g| = {} should exceed equatorial |g| = {}",
            a_pol[2].abs(),
            a_eq[0].abs()
        );
        let delta = a_pol[2].abs() - a_eq[0].abs();
        assert!(
            delta > 0.01 && delta < 0.1,
            "J2 polar-equatorial delta = {} should be 0.01..0.1 m/s²",
            delta
        );
    }

    /// Test 4 — Sphere-of-influence crossover.
    #[test]
    fn sphere_of_influence() {
        let r_inside = [RSPHERE * 0.99, 0.0, 0.0];
        assert!(
            !sphere_of_influence_check(&r_inside),
            "inside SOI should return false"
        );

        let r_outside = [RSPHERE * 1.01, 0.0, 0.0];
        assert!(
            sphere_of_influence_check(&r_outside),
            "outside SOI should return true"
        );

        // total_gravity with Earth primary at SOI boundary should not panic
        let a = total_gravity(&r_inside, Met(0), PrimaryBody::Earth);
        assert!(a[0] < 0.0, "Earth gravity still dominates inside SOI");
    }

    /// Singularity guard: zero-radius input returns zero.
    #[test]
    fn singularity_guard() {
        let a = earth_gravity(&[0.0, 0.0, 0.0]);
        assert_eq!(a, ZERO_VEC3);
    }
}
