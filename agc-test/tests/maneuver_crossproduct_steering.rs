// Spec: specs/guidance-maneuver.md §TC-M-*
//
// Integration tests for maneuver execution state:
//   ManeuverState (VG tracking, mass depletion, cutoff detection, steering direction).
//
// Tests the full S40.8 / UPDATEVG / STEERING cycle as described in P40-P47.agc.
// No global state is touched; parallel execution is safe (no #[serial]).

use agc_core::{
    guidance::{
        maneuver::VG_CUTOFF_THRESHOLD,
        new_maneuver,
        targeting::{BurnTarget, SPS_THRUST_N, SPS_VE_MS},
    },
    math::linalg::norm,
    navigation::state_vector::StateVector,
    types::Met,
};

// ── Helpers ───────────────────────────────────────────────────────────────────

fn make_state() -> StateVector {
    // Circular orbit at 185 km altitude: r along +X, v along +Y.
    let r0 = [6_556_370.0_f64, 0.0, 0.0];
    let v0 = [0.0, 7_784.0_f64, 0.0];
    StateVector::new(r0, v0, Met::from_centiseconds(0))
}

fn make_target(dv_lvlh_y: f64) -> BurnTarget {
    BurnTarget {
        tig: Met::from_centiseconds(0),
        delta_v_lvlh: [0.0, dv_lvlh_y, 0.0],
        mass: 28_800.0,
        thrust: SPS_THRUST_N,
        isp: SPS_VE_MS,
    }
}

// ── TC-MX-01: VG decreases monotonically during a prograde burn ───────────────

/// During a sustained prograde burn (5 steps × 3 m/s per step), the VG
/// magnitude must decrease by exactly 3.0 m/s each step.
///
/// Final |VG| ≈ 50 − 15 = 35 m/s.
///
/// AGC source: Comanche055/P40-P47.agc, S40.8 UPDATEVG (pages 699-701).
#[test]
fn vg_decreases_monotonically_during_burn() {
    let state = make_state();
    let target = make_target(50.0);
    let mut ms = new_maneuver(&target, &state);

    // Prograde thrust acceleration at each 2-second cycle
    let thrust_accel = [0.0_f64, 3.0, 0.0];

    for step in 0..5 {
        let vg_before = norm(&ms.vg);
        ms.update(&state, 2.0, &thrust_accel, &target);
        let vg_after = norm(&ms.vg);

        assert!(
            vg_after < vg_before,
            "Step {step}: |VG| must decrease ({vg_after:.4} < {vg_before:.4})"
        );
        assert!(
            (vg_before - vg_after - 3.0).abs() < 0.01,
            "Step {step}: |VG| decrease must be ≈3.0 m/s, got {:.4}",
            vg_before - vg_after
        );
    }

    assert!(
        !ms.cutoff,
        "Should not cut off after 15 m/s burned from 50 m/s VG"
    );
    let final_vg = norm(&ms.vg);
    assert!(
        (final_vg - 35.0).abs() < 0.1,
        "|VG| after 5 steps: {final_vg:.4} m/s (expected ≈35.0)"
    );
}

// ── TC-MX-02: Cutoff triggers when TGO < 4 s (or |VG| < 0.3 m/s) ─────────────

/// A small initial VG (3 m/s) fully depleted by one thrust update (3 m/s step)
/// must trigger the cutoff flag.
///
/// After cutoff, subsequent updates must be no-ops (VG unchanged).
///
/// AGC source: Comanche055/P40-P47.agc, S40.13 IMPULSW check (TGO < 4 s, page 721).
#[test]
fn cutoff_fires_when_vg_depleted() {
    let state = make_state();
    let target = make_target(3.0); // small initial VG so first step triggers cutoff
    let mut ms = new_maneuver(&target, &state);

    assert!(!ms.cutoff, "maneuver must start active");

    let thrust_accel = [0.0_f64, 3.0, 0.0];
    ms.update(&state, 2.0, &thrust_accel, &target);

    assert!(ms.cutoff, "cutoff must fire when VG is fully depleted");

    // Idempotency: further updates change nothing
    let vg_snapshot = ms.vg;
    ms.update(&state, 2.0, &thrust_accel, &target);
    assert_eq!(ms.vg, vg_snapshot, "VG must be frozen after cutoff");
}

// ── TC-MX-03: desired_thrust_direction returns unit vector parallel to VG ──────

/// With VG = [0, 50, 0] (prograde), desired_thrust_direction must return
/// a unit vector approximately [0, 1, 0].
///
/// magnitude must equal 1.0 to within 1e-9.
///
/// AGC source: Comanche055/P40-P47.agc, S40.8 XPRODUCT `UT` unit vector (page 721).
#[test]
fn desired_thrust_direction_is_unit_vector() {
    let state = make_state();
    let target = make_target(50.0);
    let ms = new_maneuver(&target, &state);

    let dir = ms
        .desired_thrust_direction(&state)
        .expect("direction must be Some while active");

    let dir_mag = norm(&dir);
    assert!(
        (dir_mag - 1.0).abs() < 1e-9,
        "|thrust_direction| = {dir_mag:.10} (must be 1.0)"
    );
}

// ── TC-MX-04: desired_thrust_direction returns None after cutoff ───────────────

/// Once the maneuver is cut off, desired_thrust_direction must return None
/// (engine is off; no steering direction is meaningful).
///
/// AGC source: Comanche055/P40-P47.agc, S40.8 IMPULSW/STEERSW cleared path (page 720).
#[test]
fn direction_is_none_after_cutoff() {
    let state = make_state();
    let target = make_target(3.0);
    let mut ms = new_maneuver(&target, &state);

    // Deplete VG to trigger cutoff
    ms.update(&state, 2.0, &[0.0, 3.0, 0.0], &target);
    assert!(ms.cutoff, "cutoff precondition");

    let dir = ms.desired_thrust_direction(&state);
    assert!(
        dir.is_none(),
        "desired_thrust_direction must be None after cutoff"
    );
}

// ── TC-MX-05: Steering reversal detection ─────────────────────────────────────

/// When the engine over-burns and VG reverses direction (dot(VG_new, VG_old) < 0),
/// the maneuver must be cut off immediately.
///
/// VG_initial ≈ [0, 5, 0]; thrust_accel = [0, 10, 0]:
///   VG_new = [0, 5, 0] - [0, 10, 0] = [0, -5, 0]
///   dot([0,-5,0], [0,5,0]) = -25 < 0  → reversal!
///
/// AGC source: Comanche055/P40-P47.agc, S40.8 `BPL INCRSVG`, alarm 01407 (page 720).
#[test]
fn steering_reversal_triggers_cutoff() {
    let state = make_state();
    let target = make_target(5.0); // small VG so overshoot is easy

    let mut ms = new_maneuver(&target, &state);
    assert!(!ms.cutoff, "must start active");

    // Overshooting thrust: VG_new = VG - [0,10,0] will flip sign
    let thrust_overshoot = [0.0_f64, 10.0, 0.0];
    ms.update(&state, 2.0, &thrust_overshoot, &target);

    assert!(ms.cutoff, "steering reversal must set cutoff");
    assert!(
        ms.desired_thrust_direction(&state).is_none(),
        "direction must be None after reversal cutoff"
    );
}

// ── TC-MX-06: Zero dt update is a no-op ──────────────────────────────────────

/// Calling update with dt = 0.0 must leave VG, mass, and cutoff unchanged.
///
/// AGC source: Comanche055/P40-P47.agc, S40.8 — zero-time cycle guard.
#[test]
fn zero_dt_update_is_noop() {
    let state = make_state();
    let target = make_target(40.0);
    let mut ms = new_maneuver(&target, &state);

    let vg_before = ms.vg;
    let mass_before = ms.mass;
    let cutoff_before = ms.cutoff;

    ms.update(&state, 0.0, &[0.0, 3.0, 0.0], &target);

    assert_eq!(ms.vg, vg_before, "VG must not change for dt=0");
    assert_eq!(ms.mass, mass_before, "mass must not change for dt=0");
    assert_eq!(ms.cutoff, cutoff_before, "cutoff must not change for dt=0");
}

// ── TC-MX-07: VG_CUTOFF_THRESHOLD constant is the published 0.3 m/s ──────────

/// The secondary cutoff threshold must be 0.3 m/s as documented.
///
/// AGC source: Comanche055/P40-P47.agc, S40.13 FOURSEC boundary (page 721).
#[test]
fn cutoff_threshold_is_0_3_ms() {
    assert!(
        (VG_CUTOFF_THRESHOLD - 0.3).abs() < 1e-9,
        "VG_CUTOFF_THRESHOLD = {VG_CUTOFF_THRESHOLD} (must be 0.3 m/s)"
    );
}
