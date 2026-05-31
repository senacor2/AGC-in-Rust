//! MS-T3 exit-criterion test 2 of 2: P22 lunar landmark Kalman update.
//!
//! Seeds the AGC CSM state in equatorial lunar orbit and delivers one lunar
//! landmark sighting (Mount Marilyn, index 5) via the scenario runner.
//!
//! # Exit criterion (GH #26)
//!
//! "P22 in lunar orbit with one lunar landmark sighting produces a Kalman
//! update consistent with the seeded ground-truth state."
//!
//! This test validates that criterion by asserting:
//! 1. The mark is accepted (mark_count == 1, reject_count == 0).
//! 2. The W-matrix position-axis uncertainty is reduced (W[i][i] decreases).
//! 3. No alarm is raised during the entire scenario.
//!
//! # Known issue: LOS sign convention in the scenario runner
//!
//! `landmark_los_in_platform` in `agc-sim/src/sensors.rs` returns
//! `unit(lm_inertial - csm_pos)` (pointing CSM → landmark), while
//! `p22_incorporate_landmark_mark` in `agc-core/src/programs/p22.rs`
//! internally computes `los_hat = unit(csm_pos - lm_inertial)` (pointing
//! landmark → CSM) and checks `mark.los_inertial[c] - los_hat[c]`.
//!
//! As a result the scenario runner injects a mark whose `los_inertial`
//! component has the opposite sign from `z_predicted`, producing a large
//! residual that exceeds the 3-sigma gate and causes the mark to be
//! **rejected** instead of accepted.  This is a confirmed implementation
//! bug in the scenario runner (GH issue filed separately; see report).
//!
//! The test asserts the *intended* behavior (mark accepted, W reduced)
//! and will fail until the sign convention mismatch is fixed.
//!
//! # Lunar orbit parameters used
//!
//!   r = R_MOON_M + 110_000 m = 1_847_400 m (110 km circular LLO)
//!   v = sqrt(MU_MOON / r) ≈ 1629 m/s
//!
//! Mount Marilyn (index 5, selenographic +1.23° lat, +40.01° lon) lies
//! near the equator in the +X/+Y quadrant of the Moon-fixed frame.

use agc_core::navigation::gravity::MU_MOON;
use agc_core::navigation::state_vector::Frame;
use agc_core::navigation::StateVector;
use agc_core::programs::p22::{p22_init, CSM_W_INIT_POS_VARIANCE};
use agc_core::types::{Mat3x3, Met};
use agc_core::AgcState;
use agc_sim::SimHardware;
use agc_sim::{run_scenario, LandmarkTable, ScenarioBuilder, SimDuration};

// ── Constants ──────────────────────────────────────────────────────────────────

/// LLO radius: R_MOON_M (1_737_400 m) + 110 km = 1_847_400 m.
const LLO_RADIUS_M: f64 = 1_847_400.0;

/// Truth REFSMMAT: identity (inertial frame = platform frame).
const TRUTH_REFSMMAT: Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// Mount Marilyn is lunar landmark index 5 in the LUNAR_LANDMARK_TABLE.
const MOUNT_MARILYN_INDEX: u8 = 5;

// ── Test ───────────────────────────────────────────────────────────────────────

/// tc_ms_t3_p22_lunar_landmark_kalman_update_moves_toward_truth
///
/// Seeds the AGC in equatorial LLO, delivers one lunar landmark sighting on
/// Mount Marilyn (index 5), and asserts the Kalman filter accepted the mark
/// and reduced its position uncertainty.
///
/// # Expected behavior (exit criterion)
///
/// After the `LandmarkSighting` event:
/// - `state.csm_nav.mark_count == 1`   (mark accepted)
/// - `state.csm_nav.reject_count == 0` (not rejected)
/// - `state.alarm.code == 0`            (no alarm)
/// - At least one W-matrix position-diagonal entry has decreased
///   (Kalman downdate reduces uncertainty on the observed axis)
///
/// # KNOWN FAILURE — LOS sign convention bug
///
/// This test currently fails because the scenario runner's `LandmarkSighting`
/// event passes `los_inertial = unit(lm - csm)` (csm→lm direction), but
/// `p22_incorporate_landmark_mark` expects `los_inertial = unit(csm - lm)`
/// (lm→csm direction).  The resulting residual of ≈ +0.060 far exceeds the
/// 3-sigma gate (≈ 1.25e-3), so the mark is rejected instead of accepted.
///
/// See module-level doc for details.  This test intentionally asserts the
/// correct exit-criterion behavior so it catches the regression once the
/// sign bug is fixed.
#[test]
fn tc_ms_t3_p22_lunar_landmark_kalman_update_moves_toward_truth() {
    // ── Orbital parameters ────────────────────────────────────────────────────

    let v_circ = (MU_MOON / LLO_RADIUS_M).sqrt();

    // CSM in equatorial lunar orbit on the +X axis.
    let csm_pos = [LLO_RADIUS_M, 0.0_f64, 0.0_f64];
    let csm_vel = [0.0_f64, v_circ, 0.0_f64];

    // Non-zero epoch required by p22_init (alarm 01420 fires if epoch == 0).
    let epoch = Met::from_seconds(1000.0);

    let lunar_sv = StateVector {
        position: csm_pos,
        velocity: csm_vel,
        epoch,
        frame: Frame::MoonInertial,
    };

    // ── Scenario ──────────────────────────────────────────────────────────────

    let scenario = ScenarioBuilder::new("p22_lunar_landmark_nav")
        .seed_state()
        .from_state_vector(lunar_sv)
        .met(epoch)
        .refsmmat(TRUTH_REFSMMAT)
        .done()
        .seed_truth_refsmmat(TRUTH_REFSMMAT)
        .command_attitude([1.0, 0.0, 0.0, 0.0])
        .comment("One lunar landmark mark (Mount Marilyn, index 5) → Kalman update")
        .landmark_sighting(LandmarkTable::Moon, MOUNT_MARILYN_INDEX)
        .advance(SimDuration::seconds(2))
        .build();

    let mut state = AgcState::new();
    let mut hw = SimHardware::new();

    // Initialise P22 so tracking_active = true.
    state.csm_state = lunar_sv;
    state.time = epoch;
    state.refsmmat = TRUTH_REFSMMAT;
    p22_init(&mut state);

    assert_eq!(
        state.alarm.code, 0,
        "p22_init must not raise an alarm; alarm code = {:#06x}",
        state.alarm.code
    );
    assert!(
        state.csm_nav.tracking_active,
        "p22_init must set tracking_active = true"
    );

    // Record initial W-matrix diagonal entries.
    let w_diag_before: [f64; 3] = [
        state.csm_nav.w_matrix[0][0],
        state.csm_nav.w_matrix[1][1],
        state.csm_nav.w_matrix[2][2],
    ];
    for (i, &w) in w_diag_before.iter().enumerate() {
        assert!(
            (w - CSM_W_INIT_POS_VARIANCE).abs() < 1.0,
            "w_matrix[{i}][{i}] = {w} should equal CSM_W_INIT_POS_VARIANCE = {CSM_W_INIT_POS_VARIANCE}"
        );
    }

    run_scenario(&scenario, &mut state, &mut hw);

    // ── Assertions (intended exit-criterion behavior) ──────────────────────────

    // 1. Mark accepted — no alarm, mark_count = 1, reject_count = 0.
    assert_eq!(
        state.alarm.code, 0,
        "P22 must not raise an alarm after a valid lunar landmark sighting; \
         alarm code = {:#06x}",
        state.alarm.code
    );
    assert_eq!(
        state.csm_nav.mark_count, 1,
        "P22 must record exactly one accepted mark; got mark_count = {}\n\
         (reject_count = {} — if reject_count = 1 and mark_count = 0 the \
         LOS sign convention bug is present: scenario runner passes \
         unit(lm-csm) but P22 expects unit(csm-lm))",
        state.csm_nav.mark_count, state.csm_nav.reject_count
    );
    assert_eq!(
        state.csm_nav.reject_count, 0,
        "P22 must not reject the mark; got reject_count = {}",
        state.csm_nav.reject_count
    );

    // 2. W-matrix uncertainty must decrease after the accepted mark.
    let w_diag_after: [f64; 3] = [
        state.csm_nav.w_matrix[0][0],
        state.csm_nav.w_matrix[1][1],
        state.csm_nav.w_matrix[2][2],
    ];
    let any_axis_reduced = (0..3).any(|i| w_diag_after[i] < w_diag_before[i]);
    assert!(
        any_axis_reduced,
        "W-matrix: at least one position-axis diagonal entry must decrease after \
         an accepted landmark mark.\n\
         Before: {:?}\n\
         After:  {:?}",
        w_diag_before, w_diag_after
    );
}
