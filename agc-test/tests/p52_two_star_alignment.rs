// SPDX-License-Identifier: GPL-3.0-or-later
//! MS-T3 exit-criterion test 1 of 2: P52 two-star IMU realignment.
//!
//! Seeds a perturbed REFSMMAT (~5 arc-min per axis), delivers two
//! `OpticsSighting` events against the identity truth REFSMMAT, and
//! asserts that P52's TRIAD algorithm recovers the truth REFSMMAT to
//! within 1 arc-min per axis (Frobenius-norm tolerance).
//!
//! Stars chosen: 1 (Alpheratz) and 25 (Antares).
//! Angular separation between these two catalog vectors:
//!   dot = 0.875·(-0.786) + 0.026·(-0.522) + 0.484·0.331 ≈ -0.813
//!   angle ≈ 144° — well above the 30° non-degeneracy floor.
//!
//! Exit criterion (GH #26):
//!   "P52 IMU realignment scenario from two star sightings produces a
//!    REFSMMAT close to a known truth matrix."

use agc_core::control::imu_control::ImuAlignmentState;
use agc_core::navigation::state_vector::Frame;
use agc_core::navigation::StateVector;
use agc_core::types::{Mat3x3, Met};
use agc_core::AgcState;
use agc_sim::SimHardware;
use agc_sim::{run_scenario, ScenarioBuilder, SimDuration};

// ── Star IDs used in this test ─────────────────────────────────────────────────

/// Star 1: Alpheratz (α And).  direction ≈ [0.875, 0.026, 0.484].
const STAR_A: u8 = 1;

/// Star 25: Antares (α Sco).  direction ≈ [-0.786, -0.522, 0.331].
/// Angular separation from star 1 ≈ 144°; well above the 30° floor.
const STAR_B: u8 = 25;

// ── Truth and perturbed REFSMMATes ────────────────────────────────────────────

/// Truth REFSMMAT: identity (platform = inertial reference frame).
const TRUTH_REFSMMAT: Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// Tolerance: 1 arc-minute per axis in radians.
///
/// 1 arc-min = π / (180 × 60) ≈ 2.909e-4 rad.
///
/// We test with a Frobenius-norm bound.  For a small rotation by angle θ
/// about a single axis the Frobenius distance to the identity is sqrt(2)·|sin θ|
/// ≈ sqrt(2)·θ.  Three axes contribute √3 · sqrt(2) · (1 arc-min) ≈ 7.1e-4.
/// We set the bound at 1e-9 because the test is noise-free: the sensor
/// simulation uses the exact truth REFSMMAT, so the TRIAD algorithm recovers
/// truth to machine precision.  The generous bound of 1e-9 will catch any
/// gross regression (wrong matrix, sign flip, numerical blow-up) while
/// surviving any future floating-point re-ordering.
const FROB_TOL_RAD: f64 = 1e-9;

// ── Helper ─────────────────────────────────────────────────────────────────────

/// Build a small rotation matrix: Rz(θ) · Ry(θ) · Rx(θ) for a given angle θ.
///
/// For θ ≈ 5 arc-min ≈ 1.45e-3 rad the off-diagonal entries are O(θ) so the
/// resulting matrix differs from the identity by about 1.45e-3 per component —
/// well above machine noise but still a "small" perturbation.
fn small_rotation_xyz(theta: f64) -> Mat3x3 {
    let (s, c) = (theta.sin(), theta.cos());

    // Rx(θ)
    let rx: Mat3x3 = [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]];
    // Ry(θ)
    let ry: Mat3x3 = [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]];
    // Rz(θ)
    let rz: Mat3x3 = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]];

    // Rz · Ry · Rx  (row-major; multiply left-to-right)
    mxm(mxm(rz, ry), rx)
}

/// Multiply two 3×3 matrices.
fn mxm(a: Mat3x3, b: Mat3x3) -> Mat3x3 {
    let mut c = [[0.0_f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    c
}

/// Frobenius norm of (a − b).
fn matrix_frob_diff(a: &Mat3x3, b: &Mat3x3) -> f64 {
    let mut sum = 0.0_f64;
    for i in 0..3 {
        for j in 0..3 {
            let d = a[i][j] - b[i][j];
            sum += d * d;
        }
    }
    sum.sqrt()
}

// ── Test ───────────────────────────────────────────────────────────────────────

/// tc_ms_t3_p52_two_star_alignment_recovers_refsmmat
///
/// Seeds the AGC with an REFSMMAT perturbed by ~5 arc-min per axis.
/// Delivers two `OpticsSighting` events against the identity truth REFSMMAT.
/// Asserts that P52's TRIAD algorithm recovers the truth REFSMMAT to
/// machine precision (Frobenius error < 1e-9 rad in the noise-free case).
///
/// This validates the MS-T3 exit criterion:
///   "P52 IMU realignment from two star sightings produces a REFSMMAT
///    close to a known truth matrix."
#[test]
fn tc_ms_t3_p52_two_star_alignment_recovers_refsmmat() {
    // 5 arc-minutes in radians.
    let theta_5_arcmin = 5.0_f64.to_radians() / 60.0;

    // Perturbed initial REFSMMAT: small rotation about each axis.
    let initial_refsmmat = small_rotation_xyz(theta_5_arcmin);

    // Verify the perturbation is non-trivial (i.e. it differs from identity).
    let perturbation_frob = matrix_frob_diff(&initial_refsmmat, &TRUTH_REFSMMAT);
    assert!(
        perturbation_frob > 1e-4,
        "setup: initial REFSMMAT should differ from truth by > 1e-4 Frobenius; got {perturbation_frob:.6e}"
    );

    let leo_sv = StateVector {
        position: [7_000_000.0, 0.0, 0.0],
        velocity: [0.0, 7546.0, 0.0],
        epoch: Met(0),
        frame: Frame::EarthInertial,
    };

    let scenario = ScenarioBuilder::new("p52_two_star_alignment")
        // Seed AGC state with the perturbed REFSMMAT.
        .seed_state()
        .from_state_vector(leo_sv)
        .met(Met(0))
        .refsmmat(initial_refsmmat)
        .done()
        // Seed the simulator truth REFSMMAT (identity): the "ground truth"
        // that the sextant optics measures against.
        .seed_truth_refsmmat(TRUTH_REFSMMAT)
        // Identity attitude: spacecraft body-frame aligned with platform.
        .command_attitude([1.0, 0.0, 0.0, 0.0])
        .comment("Star A (Alpheratz) — first mark")
        .optics_sighting(STAR_A)
        .comment("Star B (Antares) — second mark; TRIAD fires on this pair")
        .optics_sighting(STAR_B)
        // Tick the executive once so the state settles.
        .advance(SimDuration::seconds(2))
        .build();

    let mut state = AgcState::new();
    let mut hw = SimHardware::new();

    // P52 requires the platform to be at least CoarseAligned; set it here
    // so that p52_mark_align (called inside OpticsSighting) does not alarm.
    // (The scenario runner calls p52_mark_align directly — it does not invoke
    // p52_init, so the alignment-state precondition must be met before the
    // scenario runs.)
    state.imu_alignment_state = ImuAlignmentState::CoarseAligned;

    run_scenario(&scenario, &mut state, &mut hw);

    // ── Assertions ─────────────────────────────────────────────────────────────

    // P52 must have succeeded: no alarm, platform is now FineAligned.
    assert_eq!(
        state.alarm.code(), 0,
        "P52 must not raise an alarm on a valid two-star sighting; \
         alarm code = {:#06x}",
        state.alarm.code()
    );
    assert_eq!(
        state.imu_alignment_state,
        ImuAlignmentState::FineAligned,
        "P52 successful alignment must transition imu_alignment_state to FineAligned"
    );

    // The recovered REFSMMAT must be close to the truth (identity) matrix.
    let frob_err = matrix_frob_diff(&state.refsmmat, &TRUTH_REFSMMAT);
    assert!(
        frob_err < FROB_TOL_RAD,
        "P52 recovered REFSMMAT differs from truth by {frob_err:.3e} rad (Frobenius), \
         exceeds tolerance {FROB_TOL_RAD:.3e} rad.\n\
         Truth REFSMMAT:     {:?}\n\
         Recovered REFSMMAT: {:?}",
        TRUTH_REFSMMAT,
        state.refsmmat
    );
}
