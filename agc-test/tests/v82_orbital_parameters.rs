// SPDX-License-Identifier: GPL-3.0-or-later
//! M-A.4 — V82 / R30 orbital-parameter display integration scenario
//! (issue #127).
//!
//! Drives `V82 ENTR` through the scenario runner and asserts that the
//! DSKY pages to V06 N44 with the freshly computed apogee / perigee /
//! TFF (Time of Free Fall) triplet. TFF is the time until the
//! spacecraft next descends through 91.44 km above the reference
//! Earth radius — a one-dimensional Kepler propagation backed by
//! `navigation::conics::time_to_radius_descending`.

use agc_core::navigation::conics::{sv_to_elements, time_to_radius_descending};
use agc_core::navigation::gravity::{MU_EARTH, R_EARTH};
use agc_core::navigation::state_vector::{Frame, StateVector};
use agc_core::services::v_n::{Key, TFF_ALTITUDE_M};
use agc_core::types::Met;
use agc_core::AgcState;
use agc_sim::scenario::SimDuration;
use agc_sim::{run_scenario, DskyExpect, ScenarioBuilder, SimHardware};

fn d(n: u8) -> Key {
    Key::Digit(n)
}

/// Build a 200 km × 0 km deorbit ellipse with the spacecraft at apogee.
/// The ellipse intersects the 91.44 km TFF altitude on both the
/// descending and ascending branches, so TFF is well-defined.
fn deorbit_ellipse_at_apogee() -> AgcState {
    let r_apo = R_EARTH + 200_000.0;
    let r_peri = R_EARTH;
    let a = 0.5 * (r_apo + r_peri);
    // Vis-viva at apogee.
    let v_apo = (MU_EARTH * (2.0 / r_apo - 1.0 / a)).sqrt();

    let mut state = AgcState::new();
    state.time = Met(0);
    state.csm_state = StateVector {
        position: [r_apo, 0.0, 0.0],
        velocity: [0.0, v_apo, 0.0],
        epoch: Met(0),
        frame: Frame::EarthInertial,
    };
    state
}

/// TC-MA4-V82-N44: V82 ENTR pages to V06 N44 with apogee / perigee / TFF
/// matching the analytic conic primitive within 1 s.
#[test]
fn tc_ma4_v82_n44_orbital_parameter_display() {
    let state = deorbit_ellipse_at_apogee();

    // Pre-compute the expected TFF from the same primitive the N44 arm
    // calls. Apogee and perigee follow from the fixture geometry.
    let el = sv_to_elements(state.csm_state);
    let r_target = R_EARTH + TFF_ALTITUDE_M;
    let tff_truth_s = time_to_radius_descending(&el, r_target, el.mu())
        .expect("deorbit ellipse must intersect TFF altitude on the descent");

    let want = DskyExpect {
        verb: Some(6),
        noun: Some(44),
        r0: Some(200.0),     // apogee altitude (km)
        r1: Some(0.0),       // perigee altitude (km)
        r2: Some(tff_truth_s as f32),
        flashing: None,
        // 0.5 % absorbs the analytic-vs-display rounding for TFF and
        // the apogee/perigee numerics near zero.
        tol_pct: 0.5,
    };

    let scenario = ScenarioBuilder::new("ma4/v82_n44")
        .keys(&[Key::Verb, d(8), d(2), Key::Entr])
        .advance(SimDuration::cs(20))
        .expect_dsky(want)
        .build();

    let mut s = state;
    let mut hw = SimHardware::new();
    run_scenario(&scenario, &mut s, &mut hw);
}
