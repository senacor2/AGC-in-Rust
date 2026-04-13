// Spec: specs/navigation-gravity.md §Test Cases
//       docs/agc-reference-constants.md — MU_EARTH, RE_EARTH, J2_EARTH, MU_MOON cited below
//
// Integration tests for gravity model sanity: direction, magnitude, J2 bias,
// GEO altitude, Moon gravity, and sphere-of-influence check.
//
// No global alarm state is touched; parallel execution is safe (no #[serial]).

use agc_core::{
    navigation::{
        constants::{MU_EARTH, MU_MOON, RE_EARTH, RSPHERE},
        gravity::{earth_gravity, moon_gravity, sphere_of_influence_check, total_gravity},
        state_vector::PrimaryBody,
    },
    types::Met,
};

// ── Test 1: Earth surface gravity magnitude ───────────────────────────────────

/// Spec: specs/navigation-gravity.md §Test 1 — Earth surface gravity magnitude
///       docs/agc-reference-constants.md  RE_EARTH = 6_373_338.0 m
///
/// At r = [RE_EARTH, 0, 0] (equatorial surface):
///   Point-mass: MU_EARTH / RE_EARTH² = 3.986032e14 / (6_373_338)² ≈ 9.806 m/s²
///   J2 equatorial term reduces radial component slightly.
///   Expected total |a| ≈ 9.80 m/s² within ±0.1 m/s².
///
/// The AGC uses MU_EARTH = 3.986_032e14 and RE_EARTH = 6_373_338.0.
/// Real surface g is 9.780 m/s² (equatorial geodetic) or 9.798 m/s² (mean);
/// the AGC value (9.80665 reference) differs slightly. We check within 0.1 m/s².
#[test]
fn earth_surface_gravity_magnitude() {
    // Spec: specs/navigation-gravity.md §Test 1
    let r = [RE_EARTH, 0.0_f64, 0.0];
    let a = earth_gravity(&r);

    // Acceleration must point toward Earth (negative x at equator position on +x axis)
    assert!(
        a[0] < 0.0,
        "gravity must point toward Earth (negative x), a[0] = {}",
        a[0]
    );

    // |a| ≈ 9.80 m/s² within 0.1 m/s²
    // Derivation: MU_EARTH / RE_EARTH^2 = 3.986032e14 / (6_373_338)^2 ≈ 9.806 m/s²
    // J2 equatorial term (sin_lat = 0): a_J2 = k * u, which is slightly outward,
    // reducing the net inward acceleration by ~0.026 m/s².
    // Net: ~9.806 - 0.026 = 9.780 m/s², within the ±0.1 m/s² band around 9.80.
    let a_mag = a[0].abs();
    assert!(
        (a_mag - 9.80).abs() < 0.1,
        "|g| at equatorial surface should be ≈ 9.80 m/s² within ±0.1, got {a_mag:.4} m/s²"
    );

    // No transverse components at equator (by symmetry)
    assert!(
        a[1].abs() < 1e-10,
        "no transverse y at equator, a[1] = {}",
        a[1]
    );
    assert!(
        a[2].abs() < 1e-10,
        "no transverse z at equator, a[2] = {}",
        a[2]
    );
}

// ── Test 2: J2 bias — polar vs equatorial gravity ─────────────────────────────

/// Spec: specs/navigation-gravity.md §Test 3 — J2 polar vs equatorial bias
///       docs/agc-reference-constants.md J2_EARTH = 1.082_626_68e-3
///
/// J2 perturbation makes polar gravity stronger than equatorial gravity at the
/// same |r|. At the pole (r along z-axis) the J2 correction adds to the inward
/// radial term; at the equator it subtracts.
///
/// Expected: |a_pole| > |a_equator|, delta ~ 0.01..0.10 m/s²
///
/// The J2 correction also has a non-zero z-component at intermediate latitudes.
/// At the pole (pure z direction), the z-axis force is entirely radial inward.
#[test]
fn j2_polar_vs_equatorial_bias() {
    // Spec: specs/navigation-gravity.md §Test 3
    let r_equator = [RE_EARTH, 0.0_f64, 0.0];
    let r_pole = [0.0_f64, 0.0, RE_EARTH];

    let a_eq = earth_gravity(&r_equator);
    let a_pol = earth_gravity(&r_pole);

    // Polar gravity (along -z, pointing toward origin) must exceed equatorial
    let a_eq_mag = a_eq[0].abs();
    let a_pol_mag = a_pol[2].abs();

    assert!(
        a_pol_mag > a_eq_mag,
        "polar |g| = {a_pol_mag:.4} must exceed equatorial |g| = {a_eq_mag:.4} (J2 increases polar g)"
    );

    let delta = a_pol_mag - a_eq_mag;
    assert!(
        delta > 0.01 && delta < 0.1,
        "J2 polar-equatorial delta = {delta:.4} m/s² should be in (0.01, 0.10)"
    );

    // J2 at the pole: sin_lat = u · z_hat = 1.0
    // a_J2 = k * [(1 - 5*1²)*u + 2*1*z_hat] = k * [-4*u + 2*z_hat]
    // At the pole u = z_hat, so a_J2 = k * (-4 + 2) * z_hat = -2k * z_hat (inward)
    // Verify J2 adds to the inward z-direction at the pole (a_pol[2] < -9.8)
    assert!(
        a_pol[2] < -9.8,
        "polar gravity should be > 9.8 m/s² inward, a_pol[2] = {}",
        a_pol[2]
    );

    // At the equator (sin_lat = 0), J2 adds a small outward u component.
    // The transverse (y,z) components of a_eq should be zero by symmetry.
    assert!(
        a_eq[1].abs() < 1e-10,
        "equatorial a[1] must be zero, got {}",
        a_eq[1]
    );
    assert!(
        a_eq[2].abs() < 1e-10,
        "equatorial a[2] must be zero, got {}",
        a_eq[2]
    );
}

// ── Test 3: GEO altitude — point-mass dominates, J2 negligible ───────────────

/// Spec: specs/navigation-gravity.md §Test 2 — GEO altitude
///       docs/agc-reference-constants.md MU_EARTH = 3.986_032e14 m³/s²
///
/// At GEO (r = 42_164_000 m), the point-mass term dominates:
///   a_pm = MU_EARTH / r² ≈ 0.2242 m/s²
/// J2 is proportional to (RE/r)² which at GEO is ~(6.37e6/4.22e7)² ≈ 2.3e-3,
/// so J2 contribution is ~0.2242 × 1.08e-3 × 2.3e-3 ≈ 5e-7 m/s² (vanishingly small).
///
/// Test: |a| / a_pm_expected within 0.01% (1e-4 relative).
#[test]
fn geo_gravity_point_mass_dominated() {
    // Spec: specs/navigation-gravity.md §Test 2
    let r_geo = 42_164_000.0_f64; // GEO radius, metres
    let r = [r_geo, 0.0_f64, 0.0];
    let a = earth_gravity(&r);

    // Point-mass expected magnitude: MU_EARTH / r²
    let a_pm_expected = MU_EARTH / (r_geo * r_geo);

    // |a| should match point-mass to within 0.01% (J2 is ~2.3e-6 relative)
    let rel_err = (a[0].abs() - a_pm_expected).abs() / a_pm_expected;
    assert!(
        rel_err < 1e-4,
        "GEO gravity: |a| = {:.6e}, expected a_pm = {a_pm_expected:.6e}, rel_err = {rel_err:.3e}",
        a[0].abs()
    );

    // Must point toward Earth (negative x)
    assert!(a[0] < 0.0, "GEO gravity must point toward Earth");
}

// ── Test 4: Moon gravity via total_gravity with PrimaryBody::Moon ─────────────

/// Spec: specs/navigation-gravity.md §Public API — total_gravity, moon_gravity
///       docs/agc-reference-constants.md MU_MOON = 4.902_778e12 m³/s²
///
/// Moon gravity at mean lunar surface (~1737 km from Moon's centre, but the
/// M2 stub returns the Moon at [384_400_000, 0, 0] ECI. We test from
/// r = [384_400_000 + 1_737_000, 0, 0] (one lunar radius above the stub position).
///
/// Expected magnitude: MU_MOON / r_surface² ≈ 1.62 m/s²
/// Tolerance: within 0.5 m/s² (the stub uses fixed mean distance, not ephemeris).
///
/// Additionally verify that total_gravity with PrimaryBody::Moon returns a
/// non-zero, finite acceleration.
#[test]
fn moon_gravity_reasonable_magnitude() {
    // Spec: specs/navigation-gravity.md §moon_gravity, §total_gravity (PrimaryBody::Moon)
    //       MU_MOON = 4.902_778e12 m³/s² (docs/agc-reference-constants.md)
    let lunar_radius = 1_737_000.0_f64; // metres
    let moon_stub_x = 384_400_000.0_f64; // M2 stub fixed position (ECI, metres)

    // Vehicle at lunar surface distance from the stub Moon position
    let r = [moon_stub_x + lunar_radius, 0.0_f64, 0.0];

    let a_moon = moon_gravity(&r, Met(0));

    // moon_gravity = -MU_MOON / |delta_r|^3 * delta_r
    // delta_r = r - moon_stub = [lunar_radius, 0, 0]
    // |a| = MU_MOON / lunar_radius^2
    let expected_mag = MU_MOON / (lunar_radius * lunar_radius);

    assert!(
        a_moon[0] < 0.0,
        "Moon gravity must attract vehicle toward Moon (negative x), a[0] = {}",
        a_moon[0]
    );

    let actual_mag = a_moon[0].abs();
    // Tolerance: 0.5 m/s² — stub uses fixed mean distance so this is approximate
    assert!(
        (actual_mag - expected_mag).abs() < 0.5,
        "Moon surface gravity: got {actual_mag:.4} m/s², expected {expected_mag:.4} m/s²"
    );

    // Verify total_gravity with PrimaryBody::Moon returns finite, non-zero result
    let a_total_moon = total_gravity(&r, Met(0), PrimaryBody::Moon);
    assert!(
        a_total_moon[0].is_finite(),
        "total_gravity(Moon) must be finite, got {:?}",
        a_total_moon
    );
    assert!(
        a_total_moon[0].abs() > 0.0,
        "total_gravity(Moon) must be non-zero"
    );
}

// ── Test 5: Sphere-of-influence crossover ────────────────────────────────────

/// Spec: specs/navigation-gravity.md §Test 4 — Sphere-of-influence crossover
///       docs/agc-reference-constants.md RSPHERE = 64_373_760.0 m
///
/// Just inside SOI: sphere_of_influence_check returns false.
/// Just outside SOI: sphere_of_influence_check returns true.
/// total_gravity at SOI boundary does not panic and returns finite values.
#[test]
fn sphere_of_influence_crossover() {
    // Spec: specs/navigation-gravity.md §Test 4
    // RSPHERE = 64_373_760.0 m (docs/agc-reference-constants.md)
    let r_inside = [RSPHERE * 0.99, 0.0_f64, 0.0];
    let r_outside = [RSPHERE * 1.01, 0.0_f64, 0.0];

    assert!(
        !sphere_of_influence_check(&r_inside),
        "inside SOI should return false (r = {:.0} m < RSPHERE = {RSPHERE:.0} m)",
        r_inside[0]
    );
    assert!(
        sphere_of_influence_check(&r_outside),
        "outside SOI should return true (r = {:.0} m > RSPHERE = {RSPHERE:.0} m)",
        r_outside[0]
    );

    // total_gravity with Earth primary at SOI boundary must not panic and return finite values
    let a = total_gravity(&r_inside, Met(0), PrimaryBody::Earth);
    assert!(
        a[0] < 0.0,
        "Earth gravity still dominates inside SOI (a[0] < 0), got a[0] = {}",
        a[0]
    );
    for &component in a.iter() {
        assert!(
            component.is_finite(),
            "total_gravity component must be finite, got {component}"
        );
    }
}
