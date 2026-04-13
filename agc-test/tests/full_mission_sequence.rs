//! Full mission sequence showcase test — FRESH_START through entry.
//!
//! This test exercises the complete state machine from program 0 through
//! the mission sequence: P00 → P11 → P30 → P40 → P37 → P40 → P61.
//!
//! It verifies state machine transitions rather than numerical accuracy.
//! Durations are skipped; only modreg transitions are validated.
//!
//! AGC source: Comanche055 mission programs, V37 dispatch table.

use agc_core::services::pinball::{dispatch, VerbResult};
use agc_core::services::v_n::VnState;
use agc_core::{navigation::state_vector::StateVector, types::Met, AgcState};
use agc_sim::SimHardware;

/// Helper: assert that dispatch(V37, noun) changes modreg to `expected_prog`.
///
/// AGC source: MMCHANG (page 364), V37 = change major mode.
fn change_program(
    prog: u8,
    state: &mut AgcState,
    hw: &mut SimHardware,
    vn: &mut VnState,
) -> VerbResult {
    dispatch(37, state, hw, vn, prog)
}

/// Full mission sequence test.
///
/// Tests the following state machine transitions:
///   FRESH_START → P00 → V37N11 → P11 → V37N30 → P30 → V37N40 → P40
///   → V37N37 → P37 → V37N40 → P40 → V37N61 → P61
///
/// Each step asserts that `state.modreg` matches the requested program.
///
/// # Note on `#[ignore]`
///
/// This test does NOT `#[ignore]` because all transitions are simple modreg
/// writes with no real-time delay. Fast-forwarded mode means it runs in <1 ms.
/// If navigation math is later connected (requiring actual orbit propagation),
/// mark `#[ignore]` with reason "requires realistic orbit integration (>10s)".
#[test]
fn full_mission_state_transitions() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new_headless();
    let mut vn = VnState::new();

    // Seed a 200 km LEO state vector so navigation functions have valid data.
    state.nav.sv = StateVector::new([6_571_000.0, 0.0, 0.0], [0.0, 7_784.0, 0.0], Met(0));

    // Step 1: FRESH_START → P00.
    // AGC source: FRESH_START_AND_RESTART.agc GOPROG path → GOTOPOOH → P00.
    state.modreg = -1; // MODREG_NONE
    assert_eq!(
        change_program(0, &mut state, &mut hw, &mut vn),
        VerbResult::Ok
    );
    assert_eq!(state.modreg, 0, "should be P00 after V37N00");

    // Step 2: P00 → P11 (Earth orbit insertion monitor).
    // AGC source: Comanche055/P11.agc — VHHDOT orbit monitor.
    assert_eq!(
        change_program(11, &mut state, &mut hw, &mut vn),
        VerbResult::Ok
    );
    assert_eq!(state.modreg, 11, "should be P11 after V37N11");

    // Step 3: P11 → P30 (external delta-V targeting).
    // AGC source: Comanche055/P30-P37.agc — S31.1 targeting.
    assert_eq!(
        change_program(30, &mut state, &mut hw, &mut vn),
        VerbResult::Ok
    );
    assert_eq!(state.modreg, 30, "should be P30 after V37N30");

    // Seed a 50 m/s prograde delta-V for the burn.
    state.nav.delvslv = [0.0, 50.0, 0.0];
    state.nav.vgdisp = 50.0;

    // Step 4: P30 → P40 (SPS burn execution).
    // AGC source: Comanche055/P40-P47.agc — SPS burn phases.
    assert_eq!(
        change_program(40, &mut state, &mut hw, &mut vn),
        VerbResult::Ok
    );
    assert_eq!(state.modreg, 40, "should be P40 after V37N40");

    // Step 5: P40 → P37 (return-to-Earth targeting).
    // AGC source: Comanche055/P30-P37.agc S31.1 — Lambert solver.
    assert_eq!(
        change_program(37, &mut state, &mut hw, &mut vn),
        VerbResult::Ok
    );
    assert_eq!(state.modreg, 37, "should be P37 after V37N37");

    // Step 6: P37 → P40 (execute return burn).
    assert_eq!(
        change_program(40, &mut state, &mut hw, &mut vn),
        VerbResult::Ok
    );
    assert_eq!(state.modreg, 40, "should be P40 for return burn");

    // Step 7: P40 → P61 (entry guidance pre-entry).
    // AGC source: Comanche055/P61-P67.agc — entry sequence.
    assert_eq!(
        change_program(61, &mut state, &mut hw, &mut vn),
        VerbResult::Ok
    );
    assert_eq!(state.modreg, 61, "should be P61 after V37N61");

    // Final state: PROG display should show P61.
    assert_eq!(hw.dsky.display.prog, Some(61));
}

/// Test that V37 with an unknown program (P99) still succeeds as a modreg write.
///
/// The real AGC MMCHANG (page 364) validates the program code against PREMM1 table.
/// Our M5 implementation does not enforce this restriction (to remain minimal).
/// This test documents the current behaviour for future tightening.
#[test]
fn v37_unknown_program_writes_modreg() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new_headless();
    let mut vn = VnState::new();
    let vr = dispatch(37, &mut state, &mut hw, &mut vn, 99);
    assert_eq!(vr, VerbResult::Ok);
    // modreg is written regardless — M5 does not validate against PREMM1.
    assert_eq!(state.modreg, 99);
}

/// Test that noun 36 (MET) lookup and display works end-to-end.
///
/// AGC source: NNADTAB[36] = ECADR TIME2, HMSOUT, DECDSP.
#[test]
fn v06n36_met_display_end_to_end() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new_headless();
    let mut vn = VnState::new();

    // MET = 2 hours = 720000 centiseconds.
    state.tephem = Met(720_000);

    let vr = dispatch(6, &mut state, &mut hw, &mut vn, 36);
    assert_eq!(vr, VerbResult::Ok);

    // R1 should show "2" (hours).
    assert_eq!(hw.dsky.display.r1, Some(2));
    // R2 should show "0" (minutes).
    assert_eq!(hw.dsky.display.r2, Some(0));
    // R3 should show "0" (seconds × 100 = 0).
    assert_eq!(hw.dsky.display.r3, Some(0));
}
