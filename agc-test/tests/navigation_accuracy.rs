//! Navigation accuracy tests.
//!
//! Loads fixture files from `agc-test/fixtures/` and verifies that the
//! navigation functions in `agc-core` produce results within the documented
//! tolerances of the analytically computed reference values.
//!
//! Test levels (from `docs/testing.md`):
//!
//! - **Level 1** (this file): math function validation — `earth_gravity`,
//!   `moon_gravity`, `propagate_coast`.
//! - **Level 2** (this file): SERVICER / navigation cycle — `servicer_task`
//!   driven through `AgcState` for scripted PIPA sequences (single-cycle cases),
//!   and `average_g_step` driven directly for multi-cycle orbit conservation.
//!
//! No Docker or VirtualAGC connection is required; the fixtures contain
//! analytically derived reference values committed to source control.

use agc_core::navigation::gravity::{earth_gravity, moon_gravity};
use agc_core::navigation::integration::{average_g_step, propagate_coast};
use agc_core::navigation::state_vector::{Frame, StateVector};
use agc_core::services::average_g::{servicer_task, start_servicer, PipaCalibration};
use agc_core::types::Met;
use agc_core::AgcState;

use agc_test::fixtures::{
    load_gravity_cases, load_orbit_cases, load_servicer_cases, GravityCase, OrbitCase,
    ServicerCase, StateVectorJson,
};

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Convert a JSON `StateVectorJson` struct (from fixtures) into a `StateVector`.
fn sv_from_json(j: &StateVectorJson) -> StateVector {
    let frame = match j.frame.as_str() {
        "EarthInertial" => Frame::EarthInertial,
        "MoonInertial" => Frame::MoonInertial,
        other => panic!("Unknown frame in fixture: {}", other),
    };
    StateVector {
        position: j.position_m,
        velocity: j.velocity_m_s,
        epoch: Met(j.epoch_cs as u32),
        frame,
    }
}

/// Assert that two `f64` values are within `tol` of each other.
fn assert_near(a: f64, b: f64, tol: f64, label: &str) {
    assert!(
        (a - b).abs() <= tol,
        "{label}: got {a:.6e}, expected {b:.6e}, tolerance {tol:.2e}, diff {diff:.2e}",
        diff = (a - b).abs()
    );
}

/// Euclidean norm of a 3-component array.
fn vec3_norm(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

// ═══════════════════════════════════════════════════════════════════════════════
// Level 1: Gravity function tests (fixture-based)
// ═══════════════════════════════════════════════════════════════════════════════

/// Run all gravity fixture cases against the appropriate gravity function.
///
/// For `body = "earth"`, calls `earth_gravity(position)` and checks each
/// acceleration component against `expected_accel_m_s2`.
///
/// For `body = "moon"`, calls `moon_gravity(position)` similarly.
///
/// The tolerance is per-component from the fixture.
#[test]
fn test_gravity_fixtures() {
    let cases = load_gravity_cases();
    assert!(
        !cases.is_empty(),
        "gravity_cases.json must not be empty — check fixtures/"
    );

    for case in &cases {
        run_gravity_case(case);
    }
}

fn run_gravity_case(case: &GravityCase) {
    let pos = case.position_m;
    let expected = case.expected_accel_m_s2;
    let tol = case.tolerance_m_s2;

    let actual = match case.body.as_str() {
        "earth" => earth_gravity(pos),
        "moon" => moon_gravity(pos),
        other => panic!("Unknown body '{}' in gravity case '{}'", other, case.name),
    };

    for i in 0..3 {
        assert_near(
            actual[i],
            expected[i],
            tol,
            &format!("gravity case '{}' component [{}]", case.name, i),
        );
    }
}

/// Gravity fixtures: verify that Earth gravity always points toward the
/// Earth (dot product of acceleration with position must be negative).
#[test]
fn test_gravity_fixtures_direction() {
    let cases = load_gravity_cases();

    for case in cases.iter().filter(|c| c.body == "earth") {
        let pos = case.position_m;
        let g = earth_gravity(pos);
        let dot = g[0] * pos[0] + g[1] * pos[1] + g[2] * pos[2];
        assert!(
            dot < 0.0,
            "earth_gravity fixture '{}': dot(g, r) = {:.4e} must be negative",
            case.name, dot
        );
    }
}

/// Gravity fixtures: verify that Moon gravity points toward the Moon for
/// every Moon case (g exactly anti-parallel to r for point-mass).
#[test]
fn test_moon_gravity_fixtures_antiparallel() {
    let cases = load_gravity_cases();

    for case in cases.iter().filter(|c| c.body == "moon") {
        let pos = case.position_m;
        let g = moon_gravity(pos);

        // Cross product g × pos should be zero for anti-parallel vectors.
        let cp = [
            g[1] * pos[2] - g[2] * pos[1],
            g[2] * pos[0] - g[0] * pos[2],
            g[0] * pos[1] - g[1] * pos[0],
        ];
        let cp_mag = vec3_norm(cp);
        assert!(
            cp_mag < 1e-3,
            "moon_gravity fixture '{}': |cross(g, r)| = {:.4e}, expected < 1e-3 (not anti-parallel)",
            case.name, cp_mag
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Level 2: SERVICER cycle tests (fixture-based)
// ═══════════════════════════════════════════════════════════════════════════════

/// Run all SERVICER fixture cases.
///
/// For single-cycle PIPA conversion cases (`num_cycles == 1`):
/// - Build `AgcState`, inject PIPA counts, call `servicer_task` once.
/// - Verify the delta-V contribution by differencing with a zero-PIPA reference.
///
/// For the multi-cycle zero-PIPA coast case (`num_cycles > 1`, all zeros PIPA):
/// - Drive `average_g_step` directly for the specified number of cycles.
/// - Verify orbital radius and speed conservation within tolerance.
/// - This avoids the Waitlist-filling constraint of the hardware scheduling
///   infrastructure (MAX_WAITLIST_TASKS=8).
#[test]
fn test_servicer_cycle_fixtures() {
    let cases = load_servicer_cases();
    assert!(
        !cases.is_empty(),
        "servicer_cycle_cases.json must not be empty — check fixtures/"
    );

    for case in &cases {
        if case.num_cycles == 1 {
            run_servicer_single_cycle(case);
        } else {
            run_servicer_multi_cycle(case);
        }
    }
}

/// Single-cycle SERVICER test: verifies the full PIPA compensation pipeline
/// (bias correction → scale → misalignment → REFSMMAT rotation) by comparing
/// the actual velocity change against the expected delta-V.
fn run_servicer_single_cycle(case: &ServicerCase) {
    assert_eq!(case.pipa_sequence.len(), 1, "single-cycle case must have exactly 1 PIPA entry");

    // Run the actual servicer task with PIPA counts from fixture.
    let mut state = AgcState::new();
    state.csm_state = sv_from_json(&case.initial_state);
    state.pipa_cal = PipaCalibration {
        scale: case.pipa_cal.scale,
        bias: case.pipa_cal.bias,
        misalignment: case.pipa_cal.misalignment,
    };
    state.refsmmat = case.refsmmat;
    start_servicer(&mut state);
    state.pipa_counts = case.pipa_sequence[0];
    servicer_task(&mut state);

    // Run a reference with zero PIPA to isolate the delta-V contribution.
    let mut ref_state = AgcState::new();
    ref_state.csm_state = sv_from_json(&case.initial_state);
    ref_state.pipa_cal = PipaCalibration {
        scale: case.pipa_cal.scale,
        bias: case.pipa_cal.bias,
        misalignment: case.pipa_cal.misalignment,
    };
    ref_state.refsmmat = case.refsmmat;
    start_servicer(&mut ref_state);
    ref_state.pipa_counts = [0, 0, 0];
    servicer_task(&mut ref_state);

    // Epoch must have advanced by 200 centiseconds.
    let expected_epoch_cs = case.initial_state.epoch_cs + 200;
    assert_eq!(
        state.csm_state.epoch.0 as u64,
        expected_epoch_cs,
        "servicer case '{}': epoch = {} cs, expected {} cs",
        case.name, state.csm_state.epoch.0, expected_epoch_cs
    );

    // Frame must be unchanged.
    let expected_frame = match case.expected_final_state.frame.as_str() {
        "EarthInertial" => Frame::EarthInertial,
        "MoonInertial" => Frame::MoonInertial,
        other => panic!("Unknown expected frame in servicer case '{}': {}", case.name, other),
    };
    assert_eq!(
        state.csm_state.frame, expected_frame,
        "servicer case '{}': frame changed unexpectedly",
        case.name
    );

    // Isolate the applied delta-V by differencing with the zero-PIPA reference.
    let dv_applied = [
        state.csm_state.velocity[0] - ref_state.csm_state.velocity[0],
        state.csm_state.velocity[1] - ref_state.csm_state.velocity[1],
        state.csm_state.velocity[2] - ref_state.csm_state.velocity[2],
    ];

    // Expected delta-V: apply bias → scale → misalignment → REFSMMAT.
    let pipa = case.pipa_sequence[0];
    let bias = case.pipa_cal.bias;
    let scale = case.pipa_cal.scale;
    let m = case.pipa_cal.misalignment;
    let r = case.refsmmat;

    let biased = [
        (pipa[0] as i32 - bias[0] as i32) as f64 * scale,
        (pipa[1] as i32 - bias[1] as i32) as f64 * scale,
        (pipa[2] as i32 - bias[2] as i32) as f64 * scale,
    ];
    let mis_dv = [
        m[0][0]*biased[0] + m[0][1]*biased[1] + m[0][2]*biased[2],
        m[1][0]*biased[0] + m[1][1]*biased[1] + m[1][2]*biased[2],
        m[2][0]*biased[0] + m[2][1]*biased[1] + m[2][2]*biased[2],
    ];
    let expected_dv = [
        r[0][0]*mis_dv[0] + r[0][1]*mis_dv[1] + r[0][2]*mis_dv[2],
        r[1][0]*mis_dv[0] + r[1][1]*mis_dv[1] + r[1][2]*mis_dv[2],
        r[2][0]*mis_dv[0] + r[2][1]*mis_dv[1] + r[2][2]*mis_dv[2],
    ];

    for i in 0..3 {
        assert_near(
            dv_applied[i],
            expected_dv[i],
            case.velocity_tolerance_m_s,
            &format!("servicer case '{}': delta_v[{}]", case.name, i),
        );
    }
}

/// Multi-cycle SERVICER test (zero-PIPA, coast orbit conservation).
///
/// Uses `average_g_step` directly to drive N cycles, bypassing the Waitlist
/// scheduling infrastructure. The moon position is taken from the fixture.
fn run_servicer_multi_cycle(case: &ServicerCase) {
    // Verify all PIPA entries are zero (this test is only for coast/free-fall).
    let all_zero = case.pipa_sequence.iter().all(|p| p == &[0i16, 0, 0]);
    assert!(
        all_zero,
        "run_servicer_multi_cycle expects zero-PIPA sequences; case '{}' has non-zero entries",
        case.name
    );

    let mut sv = sv_from_json(&case.initial_state);
    let moon_pos = case.moon_pos_m;

    // Run num_cycles average_g_step calls with zero delta-V (no thrust).
    let delta_v = [0.0_f64; 3];
    for _ in 0..case.num_cycles {
        sv = average_g_step(sv, delta_v, 2.0, moon_pos);
    }

    // Verify epoch advanced by exactly num_cycles × 200 cs.
    let expected_epoch_cs = case.initial_state.epoch_cs + (case.num_cycles as u64 * 200);
    assert_eq!(
        sv.epoch.0 as u64,
        expected_epoch_cs,
        "servicer multi-cycle case '{}': epoch = {} cs, expected {} cs",
        case.name, sv.epoch.0, expected_epoch_cs
    );

    // Verify frame unchanged.
    let initial_frame = sv_from_json(&case.initial_state).frame;
    assert_eq!(
        sv.frame, initial_frame,
        "servicer multi-cycle case '{}': frame must be unchanged",
        case.name
    );

    // Verify orbital radius and speed conservation.
    let initial_r = vec3_norm(sv_from_json(&case.initial_state).position);
    let initial_v = vec3_norm(sv_from_json(&case.initial_state).velocity);
    let final_r = vec3_norm(sv.position);
    let final_v = vec3_norm(sv.velocity);

    assert!(
        (final_r - initial_r).abs() <= case.position_tolerance_m,
        "servicer multi-cycle case '{}': radius drift = {:.2} m, tolerance = {:.2} m",
        case.name, (final_r - initial_r).abs(), case.position_tolerance_m
    );
    assert!(
        (final_v - initial_v).abs() <= case.velocity_tolerance_m_s,
        "servicer multi-cycle case '{}': speed drift = {:.4} m/s, tolerance = {:.4} m/s",
        case.name, (final_v - initial_v).abs(), case.velocity_tolerance_m_s
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// Level 1 / Level 2: Orbit propagation tests (fixture-based)
// ═══════════════════════════════════════════════════════════════════════════════

/// Run all orbit propagation fixture cases.
///
/// For each case:
/// 1. Call `propagate_coast(initial_state, dt_s, moon_pos)`.
/// 2. Compare the propagated position and velocity against `expected_state`
///    using the fixture tolerance.
/// 3. Verify that the epoch has advanced by exactly `dt_s` seconds.
#[test]
fn test_orbit_propagation_fixtures() {
    let cases = load_orbit_cases();
    assert!(
        !cases.is_empty(),
        "orbit_propagation_cases.json must not be empty — check fixtures/"
    );

    for case in &cases {
        run_orbit_case(case);
    }
}

fn run_orbit_case(case: &OrbitCase) {
    let sv = sv_from_json(&case.initial_state);
    let moon_pos = case.moon_pos_m;
    let dt = case.dt_s;

    let result = propagate_coast(sv, dt, moon_pos);

    let expected = &case.expected_state;
    let pos_tol = case.position_tolerance_m;
    let vel_tol = case.velocity_tolerance_m_s;

    // For the one-orbit case, verify orbital invariants (radius and speed
    // conservation) rather than exact Cartesian coordinates, since the exact
    // return position depends on the precision of the period estimate and J2
    // precession over one orbit.
    let is_full_orbit = case.name.contains("one_orbit");

    if is_full_orbit {
        // Verify orbital radius conservation.
        let r0 = vec3_norm(sv.position);
        let r_final = vec3_norm(result.position);
        assert!(
            (r_final - r0).abs() <= pos_tol,
            "orbit case '{}': radius error = {:.2} m, tolerance = {:.2} m",
            case.name, (r_final - r0).abs(), pos_tol
        );

        // Verify speed conservation.
        let v0 = vec3_norm(sv.velocity);
        let v_final = vec3_norm(result.velocity);
        assert!(
            (v_final - v0).abs() <= vel_tol,
            "orbit case '{}': speed error = {:.4} m/s, tolerance = {:.4} m/s",
            case.name, (v_final - v0).abs(), vel_tol
        );
    } else {
        // Compare each Cartesian component of the propagated state.
        for i in 0..3 {
            assert_near(
                result.position[i],
                expected.position_m[i],
                pos_tol,
                &format!("orbit case '{}' position[{}]", case.name, i),
            );
            assert_near(
                result.velocity[i],
                expected.velocity_m_s[i],
                vel_tol,
                &format!("orbit case '{}' velocity[{}]", case.name, i),
            );
        }
    }

    // Epoch must have advanced by exactly dt_s seconds.
    let expected_epoch_cs = case.initial_state.epoch_cs + (dt * 100.0) as u64;
    assert_eq!(
        result.epoch.0 as u64,
        expected_epoch_cs,
        "orbit case '{}': epoch = {} cs, expected {} cs",
        case.name, result.epoch.0, expected_epoch_cs
    );

    // Frame must be unchanged (no SOI transition for these test cases).
    let expected_frame = match expected.frame.as_str() {
        "EarthInertial" => Frame::EarthInertial,
        "MoonInertial" => Frame::MoonInertial,
        other => panic!("Unknown expected frame in orbit case '{}': {}", case.name, other),
    };
    assert_eq!(
        result.frame, expected_frame,
        "orbit case '{}': frame changed unexpectedly",
        case.name
    );
}

/// Orbit propagation: verify that specific orbital energy is conserved within
/// 0.01% for all Earth-orbit propagation cases.
#[test]
fn test_orbit_energy_conservation() {
    use agc_core::navigation::gravity::MU_EARTH;

    let cases = load_orbit_cases();

    for case in cases.iter().filter(|c| c.initial_state.frame == "EarthInertial") {
        let sv = sv_from_json(&case.initial_state);
        let result = propagate_coast(sv, case.dt_s, case.moon_pos_m);

        let v0 = vec3_norm(sv.velocity);
        let r0 = vec3_norm(sv.position);
        let e0 = 0.5 * v0 * v0 - MU_EARTH / r0;

        let vf = vec3_norm(result.velocity);
        let rf = vec3_norm(result.position);
        let ef = 0.5 * vf * vf - MU_EARTH / rf;

        let rel_err = (ef - e0).abs() / e0.abs();
        assert!(
            rel_err < 1e-4,
            "orbit case '{}': relative energy error = {:.4e} > 1e-4",
            case.name, rel_err
        );
    }
}

/// Orbit propagation: verify that specific orbital energy is conserved within
/// 0.1% for Moon-orbit propagation cases (Earth third-body perturbation causes
/// slightly larger variation than for Earth-orbit cases).
#[test]
fn test_lunar_orbit_energy_conservation() {
    use agc_core::navigation::gravity::MU_MOON;

    let cases = load_orbit_cases();

    for case in cases.iter().filter(|c| c.initial_state.frame == "MoonInertial") {
        let sv = sv_from_json(&case.initial_state);
        let result = propagate_coast(sv, case.dt_s, case.moon_pos_m);

        let v0 = vec3_norm(sv.velocity);
        let r0 = vec3_norm(sv.position);
        let e0 = 0.5 * v0 * v0 - MU_MOON / r0;

        let vf = vec3_norm(result.velocity);
        let rf = vec3_norm(result.position);
        let ef = 0.5 * vf * vf - MU_MOON / rf;

        let rel_err = (ef - e0).abs() / e0.abs();
        // Lunar orbit includes Earth as a third-body perturber, so slightly
        // larger energy variation than for pure Earth-orbit cases is expected.
        assert!(
            rel_err < 1e-3,
            "lunar orbit case '{}': relative energy error = {:.4e} > 1e-3",
            case.name, rel_err
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// VirtualAGC constant validation
// ═══════════════════════════════════════════════════════════════════════════════

/// Validate that our Rust gravity constants are consistent with the actual
/// AGC values extracted from the Comanche055 assembly source via VirtualAGC.
///
/// The AGC used 1960s-era measurements; our Rust implementation uses modern
/// best-estimates. This test documents the known differences and ensures they
/// are within the expected range (< 10 ppm for mu values).
#[test]
fn test_vagc_constant_consistency() {
    use agc_core::navigation::gravity::{MU_EARTH, MU_MOON, R_SOI_MOON, J2_EARTH, R_EARTH};

    // AGC MUEARTH = 3.986032E10 m^3/cs^2 → 3.986032E14 m^3/s^2
    // (AGC stored mu in centisecond units; multiply by 1e4 to get SI)
    let agc_mu_earth_si = 3.986032e14;
    let mu_earth_rel_err = (MU_EARTH - agc_mu_earth_si).abs() / agc_mu_earth_si;
    assert!(
        mu_earth_rel_err < 1e-5,
        "MU_EARTH differs from AGC by {:.1} ppm (expected < 10 ppm)",
        mu_earth_rel_err * 1e6
    );

    // AGC MUMOON = 4.9027780E8 m^3/cs^2 → 4.902778E12 m^3/s^2
    let agc_mu_moon_si = 4.902778e12;
    let mu_moon_rel_err = (MU_MOON - agc_mu_moon_si).abs() / agc_mu_moon_si;
    assert!(
        mu_moon_rel_err < 1e-5,
        "MU_MOON differs from AGC by {:.1} ppm (expected < 10 ppm)",
        mu_moon_rel_err * 1e6
    );

    // AGC J2REQSQ = 1.75501139E21 B-72 is a compound precomputed constant
    // (J2 * R_earth^2 * additional scaling for the integration loop), NOT
    // the simple product J2 * R^2. Direct comparison is not meaningful.
    // Instead, verify that our J2_EARTH and R_EARTH are individually reasonable.
    assert!(
        J2_EARTH > 1.0e-3 && J2_EARTH < 1.1e-3,
        "J2_EARTH = {} outside expected range [1.0e-3, 1.1e-3]",
        J2_EARTH
    );
    assert!(
        R_EARTH > 6.37e6 && R_EARTH < 6.39e6,
        "R_EARTH = {} outside expected range [6.37e6, 6.39e6]",
        R_EARTH
    );

    // AGC RSPHERE = 64373.76 km vs our 66183 km — known 2.7% difference
    let agc_rsphere_m = 64373760.0;
    let rsoi_diff_km = (R_SOI_MOON - agc_rsphere_m).abs() / 1000.0;
    assert!(
        rsoi_diff_km < 2000.0,
        "R_SOI_MOON differs from AGC RSPHERE by {:.0} km (expected < 2000 km)",
        rsoi_diff_km
    );
    // Document the known difference
    eprintln!(
        "INFO: R_SOI_MOON = {:.0} km (modern) vs AGC RSPHERE = {:.0} km (1960s) — {:.1}% difference",
        R_SOI_MOON / 1000.0,
        agc_rsphere_m / 1000.0,
        rsoi_diff_km / (agc_rsphere_m / 1000.0) * 100.0
    );
}
