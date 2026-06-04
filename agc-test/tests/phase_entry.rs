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

/// Direct-LEO miss-distance threshold (km) — matches `entry_e2e.rs`.
const MISS_DISTANCE_DIRECT_LEO_KM: f64 = 800.0;

/// Lunar-return miss-distance threshold (km) — matches `entry_e2e.rs`.
const MISS_DISTANCE_LUNAR_RETURN_KM: f64 = 200.0;

/// Peak-g acceptance band for `tc_phase_entry_direct_leo` (#83). Direct-LEO
/// FPA = −6° / V = 7900 m/s currently peaks at ~9.1 g; band sized with
/// ~25 % headroom to catch L/D-sign bugs and ballistic regressions.
const PEAK_G_BAND_DIRECT_LEO: (f64, f64) = (7.0, 11.0);

/// Peak-g acceptance band for `tc_phase_entry_lunar_return` (#83).
/// Lunar-return FPA = −6.48° / V = 11 000 m/s currently peaks at ~10.4 g
/// — higher than Apollo 8 actual (6.84 g) because the simulator runs a
/// steeper trajectory shape than the historical one. Band sized around
/// the achieved baseline; tighter bands towards Apollo 8 historical are
/// blocked on trajectory-shape improvements (J2 oblateness, refined
/// atmosphere model).
const PEAK_G_BAND_LUNAR_RETURN: (f64, f64) = (8.5, 12.5);

/// Thin wrapper that owns its `state` / `hw` and delegates to the shared
/// driver in `agc-test::entry_scenario`. MS-T7 chains the entry phase after
/// trans-earth coast by calling the same shared driver directly.
fn run_entry_phase(
    _name: &'static str,
    seed: AgcState,
    miss_km_tol: f64,
    peak_g_band: (f64, f64),
) {
    let mut state = seed;
    let mut hw = SimHardware::new();
    run_entry_phase_scenario(&mut state, &mut hw, miss_km_tol, Some(peak_g_band));
}

#[test]
fn tc_phase_entry_direct_leo() {
    run_entry_phase(
        "phase_entry/direct_leo",
        setup_state_direct_leo(),
        MISS_DISTANCE_DIRECT_LEO_KM,
        PEAK_G_BAND_DIRECT_LEO,
    );
}

#[test]
fn tc_phase_entry_lunar_return() {
    run_entry_phase(
        "phase_entry/lunar_return",
        setup_state_lunar_return(),
        MISS_DISTANCE_LUNAR_RETURN_KM,
        PEAK_G_BAND_LUNAR_RETURN,
    );
}
