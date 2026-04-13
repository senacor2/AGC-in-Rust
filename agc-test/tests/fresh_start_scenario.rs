// AGC behaviour: specs/services-fresh-start.md §1.2 and §4.1
//
// Integration tests for the end-to-end FRESH START and warm RESTART sequences.
// These tests use SimHardware::new_headless() from agc-sim and call
// agc_core::services::fresh_start::{fresh_start, restart} to exercise the
// full cross-module initialisation path.
//
// Unit tests for each individual subsystem already exist in agc-core.
// These tests verify cross-module postconditions only.

use agc_core::{
    hal::{ImuIo, RcsIo},
    services::{
        alarm::{AlarmCode, AlarmState},
        fresh_start::{fresh_start, restart},
    },
    AgcState, MODREG_NONE,
};
use agc_sim::SimHardware;
use serial_test::serial;

// ── Test 1: fresh_start puts every subsystem into the known-safe state ────────

#[test]
#[serial]
fn fresh_start_end_to_end_postconditions() {
    // Spec: specs/services-fresh-start.md §4.1 "Post-fresh_start State"
    // Cross-module: alarm ring → cleared, restart protection → zeroed,
    //               IMU via HAL → coarse-align set, DSKY → blank.

    // Pre-condition: dirty state.
    AlarmState::raise(AlarmCode::PhaseTableError);
    AlarmState::raise(AlarmCode::NoCoreSets);

    let mut state = AgcState::new();
    state.restart.set_phase(3, 7, false, 0);
    state.modreg = 11; // P11 nominally running
    state.redoctr = 42;

    let mut hw = SimHardware::new_headless();

    // AGC behaviour: SLAP1/DOFSTART — zeros alarm history, phase tables, modreg,
    //               IMU to coarse align, DSKY blank, channels cleared.
    fresh_start(&mut state, &mut hw);

    // ── Alarm registers (FAILREG) cleared ────────────────────────────────────
    // Spec: specs/services-fresh-start.md §1.2 "State Zeroed by Fresh Start"
    assert!(
        !AlarmState::prog_light_on(),
        "PROG light must be off after fresh_start"
    );
    assert_eq!(
        AlarmState::most_recent(),
        None,
        "alarm history must be empty"
    );
    let hist = AlarmState::history();
    assert_eq!(hist, [None, None, None], "all 3 FAILREG slots must be None");

    // ── Phase tables cleared (MR.KLEAN) ──────────────────────────────────────
    // Spec: specs/executive-restart.md — MR.KLEAN zeros all 6 group pairs
    assert!(
        state.restart.all_groups_zero(),
        "all 6 restart groups must have phase == 0 after fresh_start"
    );
    assert!(
        state.restart.verify_integrity().is_ok(),
        "phase table integrity must pass after fresh_start"
    );

    // ── MODREG set to NO_PROGRAM sentinel ─────────────────────────────────────
    // Spec: specs/services-fresh-start.md §4.1
    assert_eq!(state.modreg, MODREG_NONE, "modreg must be MODREG_NONE");

    // ── REDOCTR cleared ───────────────────────────────────────────────────────
    // Spec: specs/services-fresh-start.md §1.2 "State Zeroed" table (REDOCTR)
    assert_eq!(state.redoctr, 0, "REDOCTR must be 0 after fresh_start");

    // ── IMU in coarse-align via HAL ───────────────────────────────────────────
    // Spec: specs/services-fresh-start.md §4.1, §4.6 hardware channel side-effects
    assert!(
        hw.imu.coarse_align_active(),
        "IMU must be in coarse-align after fresh_start"
    );

    // ── DSKY display blank ────────────────────────────────────────────────────
    // Spec: specs/services-fresh-start.md §4.1 "DSKY display fields are blank"
    assert!(
        hw.dsky.display.prog.is_none(),
        "DSKY PROG field must be blank"
    );
    assert!(
        hw.dsky.display.verb.is_none(),
        "DSKY VERB field must be blank"
    );
    assert!(
        hw.dsky.display.noun.is_none(),
        "DSKY NOUN field must be blank"
    );
    assert!(
        !hw.dsky.display.prog_light,
        "DSKY PROG alarm light must be off"
    );

    // ── RCS channels cleared ──────────────────────────────────────────────────
    // Spec: specs/services-fresh-start.md §4.6 "hw.rcs().write_channel5(0)"
    assert_eq!(
        hw.rcs.current_command().pitch_yaw,
        0,
        "CHAN5 (pitch/yaw jets) must be 0"
    );
    assert_eq!(
        hw.rcs.current_command().roll,
        0,
        "CHAN6 (roll jets) must be 0"
    );
}

// ── Test 2: restart() increments REDOCTR and preserves FAILREG state ─────────

#[test]
#[serial]
fn restart_increments_redoctr_and_preserves_alarm_history() {
    // Spec: specs/services-fresh-start.md §4.2 "Post-restart State (warm restart)"
    // Cross-module: restart increments REDOCTR (restart.rs) AND does NOT clear
    //               the alarm history that was present before the restart.
    //               The DSKY is re-blanked (STARTSB2) but alarms are preserved.

    AlarmState::clear_all();

    // Raise an alarm before the restart (mirrors a real GOJAM scenario where
    // the alarm is already in FAILREG when GOPROG fires).
    AlarmState::raise(AlarmCode::DeviceConflict);

    let mut state = AgcState::new();

    // Use the public increment API to set restart_count to 5.
    // (restart_count is pub(crate) inside agc-core, so we use the public increment).
    for _ in 0..5 {
        state.restart.increment_restart_count();
    }
    state.redoctr = state.restart.restart_count();

    // Set a valid phase to confirm warm restart path (not fresh-start path).
    state.restart.set_phase(2, 4, false, 0);

    let mut hw = SimHardware::new_headless();
    restart(&mut state, &mut hw);

    // REDOCTR incremented by exactly 1 (5 pre-existing + 1 from restart = 6).
    // Spec: specs/services-fresh-start.md §4.2 "state.redoctr is one greater"
    assert_eq!(
        state.redoctr, 6,
        "REDOCTR must be incremented on warm restart"
    );

    // Alarm history from BEFORE the restart is preserved (GOPROG does NOT
    // call clear_all — only fresh_start/SLAP1 does).
    // Spec: specs/services-alarm.md §1.2 BAILOUT vs ALARM path
    assert_eq!(
        AlarmState::most_recent(),
        Some(AlarmCode::DeviceConflict),
        "pre-restart alarm must still be in FAILREG after warm restart"
    );

    // Phase table content is preserved on warm restart.
    // Spec: specs/services-fresh-start.md §2.6 "State Preserved Through Restart"
    assert_eq!(
        state.restart.current_phase(2),
        4,
        "phase must be preserved through warm restart"
    );

    // No PhaseTableError alarm raised (tables were valid).
    // Spec: specs/executive-restart.md — verify_integrity path
    let hist = AlarmState::history();
    let has_phase_error = hist.iter().any(|e| *e == Some(AlarmCode::PhaseTableError));
    assert!(
        !has_phase_error,
        "no PhaseTableError should be raised for valid phase tables"
    );
}
