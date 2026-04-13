// Spec: specs/guidance-targeting.md §TC-T-*
//
// Integration tests for burn targeting (S30.1 / S40.13 path):
//   BurnTarget → burn_duration → predict_vg_at_ignition → VG vector in ECI
//
// Exercises the full targeting pipeline using realistic CSM constants.
// No global state is touched; parallel execution is safe (no #[serial]).

use agc_core::{
    guidance::targeting::{
        burn_duration, predict_vg_at_ignition, sps_constants, BurnTarget, SPS_THRUST_N, SPS_VE_MS,
    },
    math::linalg::norm,
    navigation::constants::MU_EARTH,
    navigation::state_vector::StateVector,
    types::Met,
};

// ── TC-BT-01: SPS burn duration matches Tsiolkovsky equation ─────────────────

/// burn_duration for a 50 m/s SPS burn must match the closed-form Tsiolkovsky
/// rocket equation to within 0.1 s.
///
/// mdot = F / v_e = 91188.544 / 3151.0396 ≈ 28.941 kg/s
/// t = (m0 / mdot) * (1 − exp(−dv / v_e))
///   = (28800 / 28.941) * (1 − exp(−50 / 3151.04)) ≈ 14.4 s
///
/// AGC source: Comanche055/P40-P47.agc, S40.13 TIMEBURN (pages 726-728).
#[test]
fn sps_burn_duration_matches_tsiolkovsky() {
    let (thrust, ve) = sps_constants();
    let mass = 28_800.0_f64;
    let dv = 50.0_f64;

    let t = burn_duration(dv, thrust, ve, mass).expect("valid inputs must return Some");

    // Hand-computed reference
    let mdot = thrust / ve;
    let expected = (mass / mdot) * (1.0 - (-dv / ve).exp());

    assert!(
        (t - expected).abs() < 0.1,
        "burn_duration mismatch: got {t:.3} s, expected {expected:.3} s"
    );
    // Plausibility bounds: a 50 m/s burn with these constants ≈ 14 s
    assert!(
        t > 10.0 && t < 20.0,
        "burn time {t:.3} s out of plausible range"
    );
}

// ── TC-BT-02: Zero dV → zero burn time ────────────────────────────────────────

/// burn_duration(0.0, ...) must return Some(0.0).
///
/// AGC source: Comanche055/P40-P47.agc, S40.13 — degenerate case
/// (no burn required, TGO = 0).
#[test]
fn zero_dv_gives_zero_burn_time() {
    let (thrust, ve) = sps_constants();
    let result = burn_duration(0.0, thrust, ve, 28_800.0);
    assert_eq!(result, Some(0.0), "zero dV must give zero burn time");
}

// ── TC-BT-03: Invalid inputs return None (alarm path) ─────────────────────────

/// burn_duration must return None for any non-positive engine parameter
/// (guard against WEIGHT/G = 0, F = 0, or v_e = 0 alarm conditions).
///
/// AGC source: Comanche055/P40-P47.agc, S40.13 — alarm POODOO on invalid inputs.
#[test]
fn burn_duration_invalid_inputs_return_none() {
    let (thrust, ve) = sps_constants();
    let mass = 28_800.0_f64;

    assert!(
        burn_duration(50.0, thrust, ve, -1.0).is_none(),
        "negative mass must give None"
    );
    assert!(
        burn_duration(50.0, 0.0, ve, mass).is_none(),
        "zero thrust must give None"
    );
    assert!(
        burn_duration(50.0, thrust, 0.0, mass).is_none(),
        "zero exhaust velocity must give None"
    );
    assert!(
        burn_duration(50.0, -thrust, ve, mass).is_none(),
        "negative thrust must give None"
    );
}

// ── TC-BT-04: Prograde LVLH burn rotates correctly into ECI ──────────────────

/// For a circular orbit at 185 km with velocity along +Y (ECI), a prograde
/// LVLH burn (DELVSLV = [0, 50, 0]) must produce a VG vector aligned with +Y.
///
/// LVLH frame at this geometry: r_hat = [1,0,0], v_hat = [0,1,0], h_hat = [0,0,1]
/// DELVSIN = 0*[1,0,0] + 50*[0,1,0] + 0*[0,0,1] = [0, 50, 0].
///
/// AGC source: Comanche055/P30-P37.agc, S30.1 DELVSLV → DELVSIN (page 639).
#[test]
fn prograde_lvlh_burn_gives_correct_eci_vg() {
    let r_185 = 6_556_370.0_f64;
    let v_c = (MU_EARTH / r_185).sqrt(); // circular velocity ≈ 7784 m/s

    // Circular orbit: r along +X, v along +Y
    let r0 = [r_185, 0.0, 0.0_f64];
    let v0 = [0.0, v_c, 0.0_f64];
    let state = StateVector::new(r0, v0, Met::from_centiseconds(0));

    // Prograde burn at TIG = now (same epoch)
    let target = BurnTarget {
        tig: Met::from_centiseconds(0),
        delta_v_lvlh: [0.0, 50.0, 0.0], // prograde
        mass: 28_800.0,
        thrust: SPS_THRUST_N,
        isp: SPS_VE_MS,
    };

    let vg = predict_vg_at_ignition(&state, &target);
    let vg_mag = norm(&vg);

    // Magnitude: 50 m/s
    assert!(
        (vg_mag - 50.0).abs() < 0.1,
        "|VG| = {vg_mag:.3} m/s, expected 50.0"
    );

    // Direction: aligned with +Y (ECI velocity direction for this orbit geometry)
    assert!(
        vg[0].abs() < 0.1,
        "radial component = {:.4} m/s (should be ~0)",
        vg[0]
    );
    assert!(
        (vg[1] - 50.0).abs() < 0.1,
        "velocity component = {:.4} m/s (should be ~50)",
        vg[1]
    );
    assert!(
        vg[2].abs() < 0.1,
        "normal component = {:.4} m/s (should be ~0)",
        vg[2]
    );
}

// ── TC-BT-05: Radial LVLH burn (X-component) produces radial ECI vector ──────

/// For a circular orbit at 185 km (r along +X, v along +Y), a radial
/// LVLH burn (DELVSLV = [30, 0, 0]) must produce a VG vector aligned with +X (r_hat).
///
/// LVLH: r_hat = [1,0,0], so DELVSIN = 30*[1,0,0] = [30, 0, 0].
///
/// AGC source: Comanche055/P30-P37.agc, S30.1 (page 639).
#[test]
fn radial_lvlh_burn_gives_radial_eci_vg() {
    let r_185 = 6_556_370.0_f64;
    let v_c = (MU_EARTH / r_185).sqrt();

    let r0 = [r_185, 0.0, 0.0_f64];
    let v0 = [0.0, v_c, 0.0_f64];
    let state = StateVector::new(r0, v0, Met::from_centiseconds(0));

    let target = BurnTarget {
        tig: Met::from_centiseconds(0),
        delta_v_lvlh: [30.0, 0.0, 0.0], // radial
        mass: 28_800.0,
        thrust: SPS_THRUST_N,
        isp: SPS_VE_MS,
    };

    let vg = predict_vg_at_ignition(&state, &target);
    let vg_mag = norm(&vg);

    assert!(
        (vg_mag - 30.0).abs() < 0.1,
        "|VG| = {vg_mag:.3} m/s, expected 30.0"
    );
    assert!(
        (vg[0] - 30.0).abs() < 0.1,
        "radial component = {:.4} m/s (should be ~30)",
        vg[0]
    );
    assert!(
        vg[1].abs() < 0.1,
        "velocity component = {:.4} m/s (should be ~0)",
        vg[1]
    );
    assert!(
        vg[2].abs() < 0.1,
        "normal component = {:.4} m/s (should be ~0)",
        vg[2]
    );
}

// ── TC-BT-06: SPS constants match AGC source values ──────────────────────────

/// sps_constants() must return the values cited in Comanche055/P40-P47.agc.
///
/// Thrust: 91 188.544 N (FENG, page 689)
/// v_e:   3 151.039_6 m/s (2VEXHUST / 2, page 744)
///
/// AGC source: Comanche055/P40-P47.agc, constants block, pages 689, 744.
#[test]
fn sps_constants_match_agc_source() {
    let (thrust, ve) = sps_constants();
    assert!(
        (thrust - 91_188.544).abs() < 0.001,
        "SPS thrust = {thrust:.4} N (expected 91188.544 N)"
    );
    assert!(
        (ve - 3_151.039_6).abs() < 0.001,
        "SPS v_e = {ve:.4} m/s (expected 3151.0396 m/s)"
    );
}
