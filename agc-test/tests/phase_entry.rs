//! MS-T6 — Entry phase end-to-end integration test, scenario-runner-driven.
//!
//! Implements GitHub issue #29. Spec: `specs/ms-t6-phase-entry-spec.md`.
//!
//! Drives the AGC P61 → P67 sequence via the `ScenarioBuilder` API. The
//! scenario runner's ground-truth propagator (`advance_ground_truth`) now
//! supports atmospheric drag + aerodynamic lift, with the bank angle
//! commanded by the AGC's `DapMode::EntryRoll(_)` and the signed L/D
//! fraction from `state.entry.ld_command` (MS-T6 §3).
//!
//! Two tests:
//!
//! * `tc_phase_entry_direct_leo` — direct entry from a 200 km LEO,
//!   V = 7900 m/s, FPA = −6°. Miss-distance threshold 1000 km.
//! * `tc_phase_entry_lunar_return` — translunar-return entry,
//!   V = 11 000 m/s, FPA = −6°. Miss-distance threshold 3000 km.
//!
//! The thresholds match `agc-test/tests/entry_e2e.rs` (the parallel
//! `EntryIntegrator`-driven path). MS-T6 verifies the same closed-loop
//! pipeline through the generic scenario runner — this is what
//! MS-T7 (full Apollo 8 walkthrough) needs to chain the entry phase
//! into one end-to-end scenario.

use agc_core::programs::p61_p67::EntryPhase;
use agc_core::services::average_g::start_servicer;
use agc_core::services::v_n::Key;
use agc_core::AgcState;
use agc_test::entry_scenario::{setup_state_direct_leo, setup_state_lunar_return};
use agc_sim::scenario::SimDuration;
use agc_sim::{run_scenario, ScenarioBuilder, SimHardware};

/// Direct-LEO miss-distance threshold (km) — matches `entry_e2e.rs:44`.
const MISS_DISTANCE_DIRECT_LEO_KM: f64 = 1_000.0;

/// Lunar-return miss-distance threshold (km) — matches `entry_e2e.rs:60`.
const MISS_DISTANCE_LUNAR_RETURN_KM: f64 = 3_000.0;

/// Upper bound on simulated entry duration — same as `entry_scenario.rs:27`.
const MAX_SCENARIO_DURATION_S: u32 = 20 * 60;

/// Common entry-phase driver. Drives V37 E61 E → V37 E62 E → V37 E63 E,
/// starts the SERVICER (the entry-phase exit hook is installed by P63 but
/// the SERVICER itself must be scheduled explicitly — same convention as
/// `agc-test::entry_scenario::simulate_to_drogue`), then advances the
/// scenario until drogue deploy.
fn run_entry_phase(name: &'static str, seed: AgcState, miss_km_tol: f64) {
    let mut state = seed;
    let mut hw = SimHardware::new();

    // ── Phase 1: drive P61 → P62 → P63 via DSKY keystrokes ───────────────────
    //
    // Each init_pNN sets state.entry.phase and installs the next program in
    // its chain. init_p63 hangs the entry_servicer_exit hook on
    // state.servicer_exit; the hook then drives P64/P65/P66/P67 autonomously
    // once the SERVICER starts firing each 200 cs.
    let phase1 = ScenarioBuilder::new(name)
        .comment("entry phase: V37 E61 → V37 E62 → V37 E63")
        .keys(&[
            Key::Verb,
            Key::Digit(3),
            Key::Digit(7),
            Key::Entr,
            Key::Digit(6),
            Key::Digit(1),
            Key::Entr,
        ])
        .expect_major_mode(61)
        .keys(&[
            Key::Verb,
            Key::Digit(3),
            Key::Digit(7),
            Key::Entr,
            Key::Digit(6),
            Key::Digit(2),
            Key::Entr,
        ])
        .expect_major_mode(62)
        .keys(&[
            Key::Verb,
            Key::Digit(3),
            Key::Digit(7),
            Key::Entr,
            Key::Digit(6),
            Key::Digit(3),
            Key::Entr,
        ])
        .expect_major_mode(63)
        .build();
    run_scenario(&phase1, &mut state, &mut hw);

    assert_eq!(
        state.entry.phase,
        EntryPhase::PreEntry,
        "init_p63 must leave entry phase = PreEntry"
    );
    assert!(
        state.servicer_exit.is_some(),
        "init_p63 must install entry_servicer_exit"
    );

    // ── Phase 2: schedule the SERVICER and coast through the entry ───────────
    //
    // Sync the AGC's navigation epoch to mission time so the first SERVICER
    // cycle propagates the state vector by a clean 200 cs, not by 200 cs +
    // the few cs accumulated during the V37 keystrokes. Matches the
    // p40_sps_burn pattern (agc-test/tests/p40_sps_burn.rs:224).
    state.csm_state.epoch = state.time;

    // start_servicer(&mut state) schedules the first 200 cs SERVICER cycle on
    // the waitlist. The scenario runner's coast loop then fires it every
    // cycle, the exit hook updates sensed-g, R-dot, range-to-go, and after
    // 0.05g is crossed the closed-loop guidance (P64 → … → P67) takes over
    // — exactly the same flow as `simulate_to_drogue` but driven through
    // the agc-sim scenario runner instead of a direct integrator loop.
    start_servicer(&mut state);

    let phase2 = ScenarioBuilder::new(name)
        .comment("coast through entry — atmosphere + bank flow on")
        .seed_ground_truth(state.csm_state)
        .enable_atmosphere()
        .advance_coast(SimDuration::seconds(MAX_SCENARIO_DURATION_S))
        .expect_drogue_within(miss_km_tol)
        .build();
    run_scenario(&phase2, &mut state, &mut hw);

    assert_eq!(
        state.entry.phase,
        EntryPhase::Final,
        "entry must end in Final phase"
    );
    assert_eq!(state.alarm.code, 0, "no AGC alarms during entry");
}

#[test]
fn tc_phase_entry_direct_leo() {
    run_entry_phase(
        "phase_entry/direct_leo",
        setup_state_direct_leo(),
        MISS_DISTANCE_DIRECT_LEO_KM,
    );
}

#[test]
fn tc_phase_entry_lunar_return() {
    run_entry_phase(
        "phase_entry/lunar_return",
        setup_state_lunar_return(),
        MISS_DISTANCE_LUNAR_RETURN_KM,
    );
}
