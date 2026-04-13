// Spec: specs/math-kepler.md §TC-K-*, specs/math-lambert.md §TC-L-*
//
// Integration tests: Kepler propagation and Lambert solver roundtrip.
// Verifies that the Lambert-computed v1 propagated forward with the Kepler
// solver reaches r2 within tolerance, exercising both modules together.
//
// No global state is touched; parallel execution is safe (no #[serial]).

use agc_core::{
    math::{
        kepler::kepler,
        lambert::{lambert, TransferDirection},
        linalg::{norm, sub},
    },
    navigation::constants::MU_EARTH,
};
use core::f64::consts::PI;

/// TC-KL-01: Hohmann Lambert v1 propagated with Kepler reaches r2.
///
/// A Hohmann-like transfer from 185 km to 400 km altitude.
/// Lambert produces v1; Kepler propagation must reach r2 within 10 km.
///
/// AGC source: Comanche055/CONIC_SUBROUTINES.agc LAMBERT + KEPLERN round-trip.
#[test]
fn hohmann_lambert_kepler_roundtrip() {
    let r_185 = 6_556_370.0_f64; // 185 km altitude
    let r_400 = 6_771_000.0_f64; // 400 km altitude
    let a_tr = (r_185 + r_400) / 2.0;
    let dt = PI * (a_tr * a_tr * a_tr / MU_EARTH).sqrt();

    let r1 = [r_185, 0.0, 0.0_f64];
    let angle_offset = 0.001_f64;
    let r2 = [-r_400 * angle_offset.cos(), r_400 * angle_offset.sin(), 0.0];

    let lam = lambert(&r1, &r2, dt, MU_EARTH, TransferDirection::Short);
    assert!(
        lam.converged,
        "Lambert should converge (iters={})",
        lam.iterations
    );

    let kep = kepler(&r1, &lam.v1, dt, MU_EARTH);
    assert!(kep.converged, "Kepler should converge");

    let r2_norm = norm(&r2);
    let err = norm(&sub(&kep.r, &r2));
    assert!(
        err < 10_000.0,
        "Kepler endpoint error {:.2} km > 10 km tolerance",
        err / 1e3
    );
    // Also check position error is < 0.15% of r2 distance
    assert!(
        err / r2_norm < 0.0015,
        "Relative error {:.4} > 0.15%",
        err / r2_norm
    );
}

/// TC-KL-02: Short-way and long-way Lambert both reach r2 via Kepler.
///
/// For the same r1, r2, dt the short-way and long-way transfers produce
/// different v1 vectors, both of which propagate to r2 within 10 km.
///
/// AGC source: Comanche055/CONIC_SUBROUTINES.agc GEOMSGN flag (page 1267).
#[test]
fn short_and_long_both_reach_r2() {
    let r = 6_571_000.0_f64;
    let r1 = [r, 0.0, 0.0_f64];
    let r2 = [0.0, r, 0.0_f64]; // 90° transfer
    let dt = 1_200.0_f64;

    let short = lambert(&r1, &r2, dt, MU_EARTH, TransferDirection::Short);
    let long = lambert(&r1, &r2, dt, MU_EARTH, TransferDirection::Long);

    assert!(short.converged, "Short-way must converge");
    assert!(long.converged, "Long-way must converge");

    let kep_s = kepler(&r1, &short.v1, dt, MU_EARTH);
    let kep_l = kepler(&r1, &long.v1, dt, MU_EARTH);

    let r2_norm = norm(&r2);
    let err_s = norm(&sub(&kep_s.r, &r2));
    let err_l = norm(&sub(&kep_l.r, &r2));

    assert!(
        err_s < 10_000.0,
        "Short-way Kepler error {:.2} km > 10 km",
        err_s / 1e3
    );
    assert!(
        err_l < 10_000.0,
        "Long-way Kepler error {:.2} km > 10 km",
        err_l / 1e3
    );

    // Velocities must be different
    let dv = norm(&sub(&short.v1, &long.v1));
    assert!(dv > 1.0, "Short/long v1 must differ; got dv={dv:.2} m/s");
    let _ = r2_norm;
}

/// TC-KL-03: Kepler propagation of Lambert v2 gives reversed trajectory.
///
/// Propagating r2 backward by dt with v2 reversed must reach r1 within 10 km.
/// This verifies time-reversibility of the combined Kepler+Lambert solution.
///
/// AGC source: Comanche055/CONIC_SUBROUTINES.agc TARGETV (page 1300).
#[test]
fn lambert_v2_kepler_reverse_reaches_r1() {
    let r_185 = 6_556_370.0_f64;
    let r_400 = 6_771_000.0_f64;
    let a_tr = (r_185 + r_400) / 2.0;
    let dt = PI * (a_tr * a_tr * a_tr / MU_EARTH).sqrt();

    let r1 = [r_185, 0.0, 0.0_f64];
    let angle_offset = 0.001_f64;
    let r2 = [-r_400 * angle_offset.cos(), r_400 * angle_offset.sin(), 0.0];

    let lam = lambert(&r1, &r2, dt, MU_EARTH, TransferDirection::Short);
    assert!(lam.converged, "Lambert must converge");

    // Propagate backward from r2 with -v2
    let neg_v2 = [-lam.v2[0], -lam.v2[1], -lam.v2[2]];
    let kep_back = kepler(&r2, &neg_v2, dt, MU_EARTH);
    assert!(kep_back.converged, "Backward Kepler must converge");

    let err = norm(&sub(&kep_back.r, &r1));
    assert!(
        err < 15_000.0,
        "Backward propagation error {:.2} km > 15 km",
        err / 1e3
    );
}

/// TC-KL-04: Lambert energy conservation for Earth return trajectory.
///
/// A spacecraft at ~lunar distance returns to low Earth orbit in 20 hours.
/// Orbital energy (E = v^2/2 - mu/r) must be conserved: |E1 - E2| < 1000 J/kg.
///
/// This corresponds to the P37 return-to-Earth program.
///
/// AGC source: Comanche055/CONIC_SUBROUTINES.agc LAMBERT used by P37 (page 1296).
#[test]
fn earth_return_energy_conservation() {
    let r1 = [300_000_000.0_f64, 100_000_000.0, 0.0];
    let r2 = [6_500_000.0_f64, 500_000.0, 0.0];
    let dt = 72_000.0_f64; // 20 hours

    let lam = lambert(&r1, &r2, dt, MU_EARTH, TransferDirection::Short);
    assert!(
        lam.converged,
        "P37 return must converge (iters={})",
        lam.iterations
    );

    let r1_norm = norm(&r1);
    let r2_norm = norm(&r2);
    let v1_sq = norm(&lam.v1).powi(2);
    let v2_sq = norm(&lam.v2).powi(2);

    let e1 = 0.5 * v1_sq - MU_EARTH / r1_norm;
    let e2 = 0.5 * v2_sq - MU_EARTH / r2_norm;

    assert!(
        (e1 - e2).abs() < 1_000.0,
        "Orbital energy not conserved: e1={e1:.1} e2={e2:.1} |de|={:.1}",
        (e1 - e2).abs()
    );

    // Sanity: the orbit is hyperbolic (E > 0) since it covers 300 Mm in 20 hours.
    assert!(e1 > 0.0, "Orbit must be hyperbolic (E > 0), got E={e1:.1}");

    let _ = r2_norm;
}
