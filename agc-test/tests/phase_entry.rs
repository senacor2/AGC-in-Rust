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

use agc_core::AgcState;
use agc_sim::SimHardware;
use agc_test::entry_scenario::{
    run_entry_phase_scenario, setup_state_direct_leo, setup_state_lunar_return,
};

/// Direct-LEO miss-distance threshold (km) — matches `entry_e2e.rs:44`.
const MISS_DISTANCE_DIRECT_LEO_KM: f64 = 1_000.0;

/// Lunar-return miss-distance threshold (km) — matches `entry_e2e.rs:60`.
const MISS_DISTANCE_LUNAR_RETURN_KM: f64 = 3_000.0;

/// Thin wrapper that owns its `state` / `hw` and delegates to the shared
/// driver in `agc-test::entry_scenario`. MS-T7 chains the entry phase after
/// trans-earth coast by calling the same shared driver directly.
fn run_entry_phase(_name: &'static str, seed: AgcState, miss_km_tol: f64) {
    let mut state = seed;
    let mut hw = SimHardware::new();
    run_entry_phase_scenario(&mut state, &mut hw, miss_km_tol);
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
