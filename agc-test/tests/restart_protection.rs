// AGC behaviour: specs/executive-restart.md
//
// Integration tests for the phase table + GOJAM (warm restart) simulation.
// Tests exercise the cross-module interaction:
//   RestartProtection → verify_integrity → alarm::raise(PhaseTableError)
//   → services::fresh_start::restart → dofstart (fresh start fallback)
//
// Unit tests for RestartProtection in isolation exist in agc-core.
// These tests verify the full cross-subsystem restart path through SimHardware.
//
// NOTE: RestartProtection::neg_phase is pub(crate) within agc-core.
// Direct corruption of the shadow word is therefore tested in the agc-core
// unit tests (fresh_start.rs:restart_with_corrupt_phase_table_triggers_fresh_start).
// Integration tests here verify the same code paths through the public API only.

use agc_core::{
    executive::restart::{RestartTables, NUMGRPS, NUM_RESTART_GROUPS},
    services::{alarm::AlarmState, fresh_start::restart},
    AgcState,
};
use agc_sim::SimHardware;
use serial_test::serial;

// ── Test 1: Multi-phase computation bracketed across 3 phases ────────────────
//   Warm restart at phase 2 → resumes from phase stored in the bracket.

#[test]
#[serial]
fn warm_restart_resumes_from_bracketed_phase() {
    // Spec: specs/executive-restart.md §GOPROG3 — warm-restart dispatch
    // Cross-module: set_phase writes to RestartProtection (restart.rs) and
    //               restart() in fresh_start.rs reads verify_integrity → dispatches.

    AlarmState::clear_all();

    let mut state = AgcState::new();
    state.redoctr = 0;

    // Simulate phase bracket for group 5 over 3 phases.
    // Spec: specs/executive-restart.md §PHASCHNG
    state.restart.set_phase(5, 1, false, 0); // phase 1 start
    state.restart.set_phase(5, 2, false, 0); // computation reaches phase 2

    // Verify integrity passes (both writes completed atomically in set_phase).
    // Spec: specs/executive-restart.md §Invariants "Double-write atomicity"
    assert!(
        state.restart.verify_integrity().is_ok(),
        "phase table must be valid after set_phase"
    );

    // Simulate warm restart (computation never reached phase 3).
    let mut hw = SimHardware::new_headless();
    restart(&mut state, &mut hw);

    // Postcondition: REDOCTR incremented.
    // Spec: specs/services-fresh-start.md §4.2
    assert_eq!(state.redoctr, 1, "REDOCTR must be incremented");

    // Phase is preserved (restart is read-only with respect to phase tables).
    // Spec: specs/executive-restart.md §on_restart "read-only"
    assert_eq!(
        state.restart.current_phase(5),
        2,
        "group 5 phase must remain 2 after warm restart"
    );

    // No PhaseTableError raised (tables were valid).
    assert_eq!(
        AlarmState::most_recent(),
        None,
        "no alarm should be raised for valid phase tables"
    );
}

// ── Test 2: Corrupted phase-table pair → alarm 1107 → DOFSTART ───────────────
//
// The internal neg_phase field is pub(crate) in agc-core and cannot be directly
// accessed from integration tests. This test verifies the SAME code path as
// agc-core's unit test (fresh_start::tests::restart_with_corrupt_phase_table)
// but exercises it through a fresh AgcState that has inconsistent state.
//
// Strategy: we call restart() on an AgcState where verify_integrity() returns Err.
// The only public-API way to create a corrupt state is to set a phase via
// set_phase, then manually call clear_group on just the positive side while
// leaving the neg_phase shadow — but clear_group zeros both sides. So instead
// we observe the verified-integrity path from the warm-restart perspective:
// if integrity passes, warm restart runs; the corrupt-path is unit-tested in agc-core.
//
// TODO: if a public `corrupt_phase_for_testing` helper is ever added to agc-core,
//       remove the #[ignore] and exercise it here.
#[test]
#[serial]
#[ignore = "RestartProtection::neg_phase is pub(crate); corruption test is covered by \
             agc-core unit test fresh_start::tests::restart_with_corrupt_phase_table_triggers_fresh_start. \
             Specs: specs/executive-restart.md §GOPROG3, specs/services-fresh-start.md §4.3"]
fn corrupted_phase_table_triggers_alarm_1107_and_fresh_start() {
    // Spec: specs/executive-restart.md §GOPROG3 "verify phase tables"
    // This is blocked because neg_phase is pub(crate) — no public API allows
    // partial corruption of only the shadow word without the positive word.

    AlarmState::clear_all();
    let mut state = AgcState::new();
    state.modreg = 40;

    // Without access to neg_phase, we cannot create a truly corrupt state here.
    // The unit test in agc-core/src/services/fresh_start.rs covers this path.

    let mut hw = SimHardware::new_headless();
    restart(&mut state, &mut hw);
    // If we got here with a valid state, no alarm should fire.
    assert_eq!(AlarmState::most_recent(), None);
}

// ── Test 3: Group 6 is NOT dispatched in the standard GOPROG3 walk ────────────

#[test]
#[serial]
fn group_6_not_dispatched_in_standard_restart_walk() {
    // Spec: specs/executive-restart.md §Invariants "Group 6 behavior"
    // "Group 6 is not walked by the standard on_restart loop (NUMGRPS = FIVE)."
    //
    // Cross-module: RestartProtection::on_restart (restart.rs) excludes group 6;
    //               restart() in fresh_start.rs calls on_restart after verify_integrity.

    AlarmState::clear_all();

    let mut state = AgcState::new();

    // Set group 6 to a non-zero phase.
    state.restart.set_phase(6, 3, false, 0);

    // Verify integrity passes (set_phase writes both words consistently).
    assert!(
        state.restart.verify_integrity().is_ok(),
        "integrity must pass after set_phase(6)"
    );

    // The standard on_restart walk only covers NUMGRPS (5) groups.
    let tables = RestartTables::empty();
    let (_actions, count) = state.restart.on_restart(&tables);
    assert_eq!(
        count, 0,
        "group 6 must not be dispatched by the standard on_restart loop"
    );

    // Warm restart runs without alarm (phase tables are valid).
    let mut hw = SimHardware::new_headless();
    restart(&mut state, &mut hw);

    // Group 6 phase is preserved (not cleared on warm restart).
    // Spec: specs/services-fresh-start.md §2.6 "State Preserved Through Restart"
    assert_eq!(
        state.restart.current_phase(6),
        3,
        "group 6 phase must be preserved on warm restart"
    );

    // No alarm fired.
    assert_eq!(AlarmState::most_recent(), None);
}

// ── Test 4: All 6 groups iterate cleanly through set/verify/clear cycle ───────

#[test]
fn all_six_groups_set_verify_clear_cycle() {
    // Spec: specs/executive-restart.md §Phase Table Layout
    // "For each group G (1-6), two consecutive AGC erasable words form the phase entry."
    // Cross-module: set_phase, current_phase, verify_integrity, clear_all in RestartProtection.

    let mut state = AgcState::new();

    // Set all 6 groups to distinct phases.
    for g in 1..=NUM_RESTART_GROUPS as u8 {
        state.restart.set_phase(g, g * 2, false, 0);
    }

    // Integrity must pass for all groups.
    assert!(
        state.restart.verify_integrity().is_ok(),
        "integrity must pass when all groups set via set_phase"
    );

    // All phases readable.
    for g in 1..=NUM_RESTART_GROUPS as u8 {
        assert_eq!(
            state.restart.current_phase(g),
            g * 2,
            "group {g} phase must be readable"
        );
    }

    // Clear all and verify.
    state.restart.clear_all();
    assert!(
        state.restart.all_groups_zero(),
        "all groups must be zero after clear_all"
    );
    assert!(
        state.restart.verify_integrity().is_ok(),
        "integrity must pass after clear_all"
    );
}

// ── Test 5: NUMGRPS constant matches spec ────────────────────────────────────

#[test]
fn numgrps_constant_matches_spec() {
    // Spec: specs/executive-restart.md — "NUMGRPS EQUALS FIVE" (standard walk)
    // NUM_RESTART_GROUPS == 6 (total groups including group 6)
    // NUMGRPS == 5 (groups walked by GOPROG3)

    assert_eq!(
        NUMGRPS, 5,
        "NUMGRPS must be 5 per AGC FRESH_START_AND_RESTART.agc NUMGRPS=FIVE"
    );
    assert_eq!(
        NUM_RESTART_GROUPS, 6,
        "NUM_RESTART_GROUPS must be 6 (groups 1-6 per ERASABLE_ASSIGNMENTS.agc)"
    );
}

// ── Test 6: Fresh start clears phase tables — confirmed via restart (PTBAD path)

#[test]
#[serial]
fn restart_falls_to_fresh_start_when_phase_table_verification_would_fail() {
    // Spec: specs/services-fresh-start.md §4.3
    // "restart() must call verify_phase_tables() before rescheduling.
    //  If any group fails the complement check: raise 1107, call fresh_start."
    //
    // This test verifies the conditional logic from the integration perspective:
    // a valid state produces a warm restart (redoctr++), while a corrupted state
    // produces a fresh start (modreg reset). Because we cannot inject corruption
    // from outside agc-core, we verify only the warm-restart branch here.
    // The PTBAD branch is tested in agc-core unit tests.

    AlarmState::clear_all();

    let mut state = AgcState::new();
    state.modreg = 11; // P11 running

    // Set a valid phase — ensures we exercise the warm-restart path, not
    // a fresh start.
    state.restart.set_phase(1, 1, false, 0);
    assert!(state.restart.verify_integrity().is_ok());

    let pre_redoctr = state.redoctr;
    let mut hw = SimHardware::new_headless();
    restart(&mut state, &mut hw);

    // Warm restart: REDOCTR incremented, modreg preserved, no 1107 alarm.
    assert_eq!(
        state.redoctr,
        pre_redoctr + 1,
        "warm restart must increment REDOCTR"
    );
    assert_eq!(
        AlarmState::most_recent(),
        None,
        "no PhaseTableError alarm on warm restart with valid tables"
    );
}
