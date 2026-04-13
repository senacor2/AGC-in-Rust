// Integration tests for P51/P52 IMU alignment programs.
//
// Exercises the full alignment state machine end-to-end using SimHardware.
// Tests cover: P51 fresh alignment, P52 in-flight realignment, IMU fail path.
//
// AGC source: Comanche055/P51-P53.agc PROG52/R51/R52/R55 (pages 737-769).
// AGC source: Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc AXISGEN/CALCGTA.
//
// Tests that touch ALARM_STATE (a global) are marked #[serial] to prevent
// test-parallel state corruption.

use agc_core::{
    programs::p51_imu_align::{
        enter_p51, enter_p52, mark_star_a, mark_star_b, notify_coarse_aligned, tick, AlignPhase,
    },
    types::Vec3,
    AgcState,
};
use agc_sim::SimHardware;
use serial_test::serial;

// ── TC-P51-IT-01: P52 full alignment — PromptRefsmmat → Done ─────────────────

/// Full P52 realignment with orthogonal star pair reaches Done state.
///
/// AGC source: Comanche055/P51-P53.agc PROG52 → R52 → R55 → CALCGTA.
#[test]
fn p52_full_alignment_reaches_done() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new_headless();

    let mut as_state = enter_p52(&mut state, &mut hw);
    assert_eq!(as_state.phase, AlignPhase::PromptRefsmmat);
    assert!(!as_state.is_p51, "P52 must set is_p51 = false");
    assert_eq!(state.modreg, 52);

    // Crew presses PROCEED (option selection).
    hw.dsky.set_proceed(true);
    tick(&mut as_state, &mut state, &mut hw);
    assert_eq!(
        as_state.phase,
        AlignPhase::WaitStarA,
        "must skip WaitCoarseAlign for P52"
    );

    // Mark star A (Arcturus-like direction).
    let star_a_sm: Vec3 = [1.0, 0.0, 0.0];
    let star_a_nb: Vec3 = [1.0, 0.0, 0.0]; // catalog = same as observed (perfect alignment)
    mark_star_a(&mut as_state, star_a_sm, star_a_nb);
    tick(&mut as_state, &mut state, &mut hw);
    assert_eq!(as_state.phase, AlignPhase::WaitStarB);

    // Mark star B (Canopus-like direction, orthogonal to A).
    let star_b_sm: Vec3 = [0.0, 1.0, 0.0];
    let star_b_nb: Vec3 = [0.0, 1.0, 0.0];
    mark_star_b(&mut as_state, star_b_sm, star_b_nb);
    tick(&mut as_state, &mut state, &mut hw); // → Torque (star separation OK)
    assert_eq!(
        as_state.phase,
        AlignPhase::Torque,
        "must be in Torque after valid star pair"
    );

    tick(&mut as_state, &mut state, &mut hw); // → Done (fine alignment math)
    assert_eq!(
        as_state.phase,
        AlignPhase::Done,
        "P52 full alignment must reach Done"
    );
    assert!(
        state.flags.refsmflg,
        "REFSMFLG must be set on alignment completion"
    );
}

// ── TC-P51-IT-02: P51 alignment — includes WaitCoarseAlign step ──────────────

/// P51 (initial alignment) sequences through WaitCoarseAlign before stars.
///
/// AGC source: Comanche055/P51-P53.agc CAL53A → COARFINE (page 762).
#[test]
fn p51_includes_coarse_align_step() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new_headless();

    let mut as_state = enter_p51(&mut state, &mut hw);
    assert_eq!(as_state.phase, AlignPhase::PromptRefsmmat);
    assert!(as_state.is_p51, "P51 must set is_p51 = true");
    assert_eq!(state.modreg, 51);

    // Crew presses PROCEED → goes to WaitCoarseAlign (not directly WaitStarA).
    hw.dsky.set_proceed(true);
    tick(&mut as_state, &mut state, &mut hw);
    assert_eq!(
        as_state.phase,
        AlignPhase::WaitCoarseAlign,
        "P51 must wait for coarse align before stars"
    );

    // Notify coarse alignment complete.
    notify_coarse_aligned(&mut as_state);
    tick(&mut as_state, &mut state, &mut hw);
    assert_eq!(as_state.phase, AlignPhase::WaitStarA);

    // Mark two orthogonal stars and complete alignment.
    let sa: Vec3 = [0.0, 0.0, 1.0];
    let sb: Vec3 = [1.0, 0.0, 0.0];
    mark_star_a(&mut as_state, sa, sa);
    tick(&mut as_state, &mut state, &mut hw); // → WaitStarB
    mark_star_b(&mut as_state, sb, sb);
    tick(&mut as_state, &mut state, &mut hw); // → Torque
    tick(&mut as_state, &mut state, &mut hw); // → Done
    assert_eq!(as_state.phase, AlignPhase::Done);
}

// ── TC-P51-IT-03: IMU fail bit causes immediate Failed on entry ───────────────

/// When IMU status has the fail bit set, P51 entry returns Failed immediately.
///
/// AGC source: Comanche055/P51-P53.agc S61.1 → alarm 01426/01427.
#[test]
#[serial]
fn p51_imu_fail_returns_failed_immediately() {
    use agc_core::services::alarm::AlarmState;
    AlarmState::clear_all();

    let mut state = AgcState::new();
    let mut hw = SimHardware::new_headless();

    // Inject IMU fail bit (CHAN30 bit 13 = bit 12 zero-based).
    hw.imu.inject_status(1 << 12);

    let as_state = enter_p51(&mut state, &mut hw);
    assert_eq!(
        as_state.phase,
        AlignPhase::Failed,
        "IMU fail bit must cause immediate Failed state"
    );
    assert_ne!(as_state.alarm_code, 0, "alarm code must be set on IMU fail");
    assert!(
        AlarmState::most_recent().is_some(),
        "alarm must be raised in AlarmState on IMU fail"
    );

    AlarmState::clear_all();
}
