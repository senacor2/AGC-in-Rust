// SPDX-License-Identifier: GPL-3.0-or-later
//! M-A.2 — V16 monitor integration tests for the entry-phase display
//! nouns N63 / N64 / N66 / N67 / N68 (issue #125).
//!
//! Each test seeds the relevant `EntryState` (and CSM velocity, where
//! the noun reads it) and drives a `V16 ENTR N## ENTR` keystroke
//! sequence through the scenario runner. The DSKY register expectations
//! are checked with `ExpectDsky` so the assertion lives in the timeline
//! at the MET it was evaluated.
//!
//! Scaling matches the production `noun_display` implementation
//! (SI units, mirroring the N44 convention) — km / m/s / deg / g / s.

use agc_core::navigation::state_vector::{Frame, StateVector};
use agc_core::services::v_n::Key;
use agc_core::types::Met;
use agc_core::AgcState;
use agc_sim::scenario::SimDuration;
use agc_sim::{run_scenario, DskyExpect, ScenarioBuilder, SimHardware};

fn d(n: u8) -> Key {
    Key::Digit(n)
}

/// Build the `V16 N## ENTR` key sequence with the digit decomposition
/// expected by the V/N processor. The `Noun` key separates verb digits
/// from noun digits — V16 is a monitor verb, not a verb-with-major-mode
/// (which would use the `V37 ENTR ## ENTR` two-ENTR pattern).
fn v16_n(noun: u8) -> [Key; 7] {
    let n_tens = noun / 10;
    let n_ones = noun % 10;
    [
        Key::Verb,
        d(1),
        d(6),
        Key::Noun,
        d(n_tens),
        d(n_ones),
        Key::Entr,
    ]
}

fn expect(verb: u8, noun: u8, r0: f32, r1: f32, r2: f32, tol_pct: f32) -> DskyExpect {
    DskyExpect {
        verb: Some(verb),
        noun: Some(noun),
        r0: Some(r0),
        r1: Some(r1),
        r2: Some(r2),
        flashing: None,
        tol_pct,
    }
}

/// Drive `V16 N## ENTR` and assert the resulting DSKY frame.
fn drive(name: &'static str, state: AgcState, noun: u8, want: DskyExpect) {
    let scenario = ScenarioBuilder::new(name)
        .keys(&v16_n(noun))
        // Allow a few ticks for the refresh path to settle.
        .advance(SimDuration::cs(50))
        .expect_dsky(want)
        .build();
    let mut s = state;
    let mut hw = SimHardware::new();
    run_scenario(&scenario, &mut s, &mut hw);
}

#[test]
fn tc_ma2_n63_v16_rtgo_vpred_tfe() {
    let mut state = AgcState::new();
    state.time = Met(0);
    state.entry.target_range_km = 350.0;
    state.entry.vl_predicted_mps = 7_400.0;
    state.entry.time_from_event_s = 42.0;

    drive(
        "ma2/n63",
        state,
        63,
        expect(16, 63, 350.0, 7_400.0, 42.0, 0.0),
    );
}

#[test]
fn tc_ma2_n64_v16_drag_vi_rtsplash() {
    let mut state = AgcState::new();
    state.time = Met(0);
    state.entry.sensed_acceleration_g = 5.6;
    state.csm_state = StateVector {
        position: [6_500_000.0, 0.0, 0.0],
        velocity: [0.0, 7_800.0, 0.0],
        epoch: Met(0),
        frame: Frame::EarthInertial,
    };
    state.entry.target_range_km = 510.0;

    drive(
        "ma2/n64",
        state,
        64,
        expect(16, 64, 5.6, 7_800.0, 510.0, 0.0),
    );
}

#[test]
fn tc_ma2_n66_v16_bank_xrange_drange() {
    let mut state = AgcState::new();
    state.time = Met(0);
    state.entry.roll_command_rad = 30.0_f64.to_radians();
    state.entry.crossrange_km = -8.5;
    state.entry.downrange_error_km = 4.0;

    drive(
        "ma2/n66",
        state,
        66,
        // Bank is converted radians→degrees inside the noun arm; use
        // a small percentage tolerance to absorb f32 rounding.
        expect(16, 66, 30.0, -8.5, 4.0, 0.01),
    );
}

#[test]
fn tc_ma2_n67_v16_rtgo_target_lat_lon() {
    let mut state = AgcState::new();
    state.time = Met(0);
    state.entry.target_range_km = 1_200.0;
    state.entry.target_lat_rad = 0.5_f64; // ≈ 28.6479°
    state.entry.target_lon_rad = 1.0_f64; // ≈ 57.2957°

    drive(
        "ma2/n67",
        state,
        67,
        expect(16, 67, 1_200.0, 28.6479, 57.2957, 0.01),
    );
}

#[test]
fn tc_ma2_n68_v16_bank_vi_rdot() {
    let mut state = AgcState::new();
    state.time = Met(0);
    state.entry.roll_command_rad = -60.0_f64.to_radians();
    state.csm_state = StateVector {
        position: [6_500_000.0, 0.0, 0.0],
        velocity: [0.0, 7_700.0, 0.0],
        epoch: Met(0),
        frame: Frame::EarthInertial,
    };
    state.entry.r_dot_mps = -200.0;

    drive(
        "ma2/n68",
        state,
        68,
        expect(16, 68, -60.0, 7_700.0, -200.0, 0.01),
    );
}
