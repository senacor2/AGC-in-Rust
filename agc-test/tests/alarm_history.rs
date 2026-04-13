// AGC behaviour: specs/services-alarm.md
//
// Integration tests for the FAILREG ring buffer end-to-end path, exercised
// through the public AlarmState API as used by agc-core subsystems.
// SimHardware is used to verify the DSKY prog_light wiring.
//
// Unit tests for AlarmState in isolation exist in agc-core/src/services/alarm.rs.
// These tests focus on cross-module interactions and the ring-wrap behaviour
// at integration level.

use agc_core::services::alarm::{AlarmCode, AlarmSeverity, AlarmState};
use agc_sim::SimHardware;
use serial_test::serial;

// ── Test 1: Ring buffer wraps — 5 distinct codes, only 3 remain ──────────────

#[test]
#[serial]
fn alarm_ring_retains_most_recent_three() {
    // Spec: specs/services-alarm.md §1.1 "3-slot history buffer"
    // "On overflow the oldest slot is overwritten."
    // Cross-module: raise() path from AlarmState global static → history().

    AlarmState::clear_all();

    // Raise 5 distinct alarms.
    AlarmState::raise(AlarmCode::NoVacArea); // slot 0 (will be evicted)
    AlarmState::raise(AlarmCode::WaitlistOverflow); // slot 1 (will be evicted)
    AlarmState::raise(AlarmCode::PhaseTableError); // slot 2 (kept as oldest)
    AlarmState::raise(AlarmCode::DeviceConflict); // evicts NoVacArea
    AlarmState::raise(AlarmCode::MmChangeNotAllowed); // evicts WaitlistOverflow

    let hist = AlarmState::history();

    // Only the 3 most recent must remain.
    // Spec: specs/services-alarm.md §Test 2 "Ring buffer holds three entries"
    assert_eq!(
        hist[0],
        Some(AlarmCode::PhaseTableError),
        "oldest of the surviving 3 must be PhaseTableError"
    );
    assert_eq!(
        hist[1],
        Some(AlarmCode::DeviceConflict),
        "middle slot must be DeviceConflict"
    );
    assert_eq!(
        hist[2],
        Some(AlarmCode::MmChangeNotAllowed),
        "most recent must be MmChangeNotAllowed"
    );

    // most_recent() must agree.
    assert_eq!(
        AlarmState::most_recent(),
        Some(AlarmCode::MmChangeNotAllowed),
        "most_recent must return the last raised alarm"
    );

    // PROG light stays on after overflow.
    // Spec: specs/services-alarm.md §4.4 "PROG Light Consistency"
    assert!(
        AlarmState::prog_light_on(),
        "PROG light must remain on after ring overflow"
    );
}

// ── Test 2: DSKY prog_light wiring after an alarm is raised ──────────────────
//
// The spec (specs/services-alarm.md §6.1) requires that DskyDisplayState.prog_alarm
// is set when an alarm is raised.  However, the wiring from AlarmState::raise()
// to SimDsky::set_prog_light() is NOT automatic in the current implementation:
// raise() updates the in-memory AlarmState global, but the sim's DskyDisplayState
// prog_light field is only set when the flight software explicitly calls
// hw.dsky().set_prog_light(true) — which happens in fresh_start/startsb2 but
// NOT directly from alarm::raise.
//
// TODO: specs/services-alarm.md §6.1 — wire alarm::raise → hw.dsky().set_prog_light(true)
//       so that every alarm automatically illuminates the DSKY PROG lamp.
//       This requires either (a) AlarmState holding a reference to the DSKY HAL
//       (not possible without architectural change) or (b) the Executive loop
//       polling AlarmState::prog_light_on() and forwarding to the DSKY each cycle.
//       Until this is implemented the test is ignored.
#[test]
#[serial]
#[ignore = "TODO: alarm::raise does not yet automatically set DskyDisplayState::prog_light; \
             see specs/services-alarm.md §6.1 — prog_light wiring not hooked up in M1"]
fn alarm_raise_sets_dsky_prog_light() {
    // Spec: specs/services-alarm.md §1.3 PROG Light Mechanism
    // "The PROG alarm indicator cell must be rendered when an alarm is raised."

    AlarmState::clear_all();
    let mut hw = SimHardware::new_headless();

    AlarmState::raise(AlarmCode::NoCoreSets);

    // The AlarmState internal flag is set.
    assert!(AlarmState::prog_light_on());

    // The DSKY SimDsky must also reflect this — currently NOT wired.
    assert!(
        hw.dsky.display.prog_light,
        "DskyDisplayState::prog_light must be set when an alarm is raised"
    );
}

// ── Test 3: Read alarm history without clearing, then clear ──────────────────

#[test]
#[serial]
fn alarm_history_read_without_clear_then_clear() {
    // Spec: specs/services-alarm.md §2.5 history() and clear_all()
    // Cross-module: multiple reads from the ring buffer do not mutate state.

    AlarmState::clear_all();

    AlarmState::raise(AlarmCode::WaitlistOverflow);
    AlarmState::raise(AlarmCode::CcsHole);

    // First read.
    let hist1 = AlarmState::history();
    assert_eq!(hist1[0], Some(AlarmCode::WaitlistOverflow));
    assert_eq!(hist1[1], Some(AlarmCode::CcsHole));
    assert_eq!(hist1[2], None);

    // Second read must return the same values (non-destructive).
    let hist2 = AlarmState::history();
    assert_eq!(
        hist1, hist2,
        "repeated history() calls must return the same values"
    );

    // most_recent is still CcsHole.
    assert_eq!(AlarmState::most_recent(), Some(AlarmCode::CcsHole));

    // Clear via the sim path (equivalent to RSET key → SKIPSIM → clear_all).
    // Spec: specs/services-alarm.md §2.5 clear_all()
    AlarmState::clear_all();

    assert_eq!(
        AlarmState::history(),
        [None, None, None],
        "history must be empty after clear_all"
    );
    assert_eq!(AlarmState::most_recent(), None);
    assert!(!AlarmState::prog_light_on());
}

// ── Test 4: Alarm severity codes are correct per spec ─────────────────────────

#[test]
fn alarm_severity_classification_matches_spec() {
    // Spec: specs/services-alarm.md §1.5 Severity Classification
    // Cross-module: ensures the global AlarmCode::severity() enum matches.

    // Bailout codes (Executive/Waitlist overflow).
    assert_eq!(AlarmCode::NoVacArea.severity(), AlarmSeverity::Bailout);
    assert_eq!(AlarmCode::NoCoreSets.severity(), AlarmSeverity::Bailout);
    assert_eq!(
        AlarmCode::WaitlistOverflow.severity(),
        AlarmSeverity::Bailout
    );

    // SoftRestart codes (POODOO path).
    assert_eq!(
        AlarmCode::WaitlistNegDt.severity(),
        AlarmSeverity::SoftRestart
    );
    assert_eq!(
        AlarmCode::DeviceConflict.severity(),
        AlarmSeverity::SoftRestart
    );
    assert_eq!(AlarmCode::CcsHole.severity(), AlarmSeverity::SoftRestart);

    // Continue codes (ALARM path — record and continue).
    assert_eq!(
        AlarmCode::PhaseTableError.severity(),
        AlarmSeverity::Continue
    );
    assert_eq!(
        AlarmCode::MmChangeNotAllowed.severity(),
        AlarmSeverity::Continue
    );
    assert_eq!(AlarmCode::Curtains.severity(), AlarmSeverity::Continue);
    assert_eq!(
        AlarmCode::RestartNoActiveGroups.severity(),
        AlarmSeverity::Continue
    );
}

// ── Test 5: Alarm octal codes match the AGC source constants ─────────────────

#[test]
fn alarm_codes_have_correct_octal_values() {
    // Spec: specs/services-alarm.md §1.4 Alarm Code Catalogue
    // Cross-module: confirms the enum discriminants match the AGC source exactly.

    assert_eq!(AlarmCode::NoVacArea.value(), 0o1201, "alarm 01201");
    assert_eq!(AlarmCode::NoCoreSets.value(), 0o1202, "alarm 01202");
    assert_eq!(AlarmCode::WaitlistOverflow.value(), 0o1203, "alarm 01203");
    assert_eq!(AlarmCode::WaitlistNegDt.value(), 0o1204, "alarm 01204");
    assert_eq!(AlarmCode::PhaseTableError.value(), 0o1107, "alarm 01107");
    assert_eq!(AlarmCode::DeviceConflict.value(), 0o1210, "alarm 01210");
}
