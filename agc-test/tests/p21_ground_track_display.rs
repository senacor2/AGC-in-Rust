// SPDX-License-Identifier: GPL-3.0-or-later
//! M-A.5 — P21 ground-track display via the noun pipeline (issue #128).
//!
//! Drives `V37 ENTR 21 ENTR` through the scenario runner against a
//! known equatorial-orbit fixture and asserts the DSKY shows V06 N43
//! with lat / lon / alt computed by `navigation`'s ECI → geodetic
//! conversion. The assertion goes through `decode_dsky` via
//! `ExpectDsky`, not through any direct r-register side-channel.
//!
//! The fixture's epoch and target GET are both zero, so the ground
//! track is just the inertial position projected onto the equator.
//! Spacecraft on the +X axis at 300 km altitude → lat = 0°, lon = 0°,
//! alt = 300 km.

use agc_core::navigation::gravity::R_EARTH;
use agc_core::navigation::state_vector::{Frame, StateVector};
use agc_core::programs::p21::p21_compute_ground_track;
use agc_core::services::v_n::Key;
use agc_core::types::Met;
use agc_core::AgcState;
use agc_sim::{run_scenario, DskyExpect, ScenarioBuilder, SimHardware};

fn d(n: u8) -> Key {
    Key::Digit(n)
}

/// TC-MA5-P21-N43: V37 ENTR 21 ENTR populates V06 N43 with the
/// sub-satellite point via the centralised noun pipeline.
#[test]
fn tc_ma5_p21_n43_ground_track_display() {
    let mut state = AgcState::new();
    state.time = Met(0);
    state.gha_epoch_rad = 0.0; // GHA = 0 at MET = 0 → ECI ≡ ECEF

    // Fixture: a known sub-satellite point (lat = 30° N, lon = 45° E,
    // alt = 200 km) with a circular-orbit velocity tangent so the
    // altitude doesn't drift while the scenario runner ticks.
    let lat = 30.0_f64.to_radians();
    let lon = 45.0_f64.to_radians();
    let r = R_EARTH + 200_000.0;
    let position = [
        r * lat.cos() * lon.cos(),
        r * lat.cos() * lon.sin(),
        r * lat.sin(),
    ];
    // Velocity: circular speed, tangent to the position, in the plane
    // whose normal points along +Z × position (a prograde polar
    // orbit through this point). Perpendicular to `position`, magnitude
    // = √(μ/r), so `kepler_step` propagates along a closed circle.
    use agc_core::navigation::gravity::MU_EARTH;
    let v_circ = (MU_EARTH / r).sqrt();
    let p_xy = (position[0] * position[0] + position[1] * position[1]).sqrt();
    let velocity = [
        -position[1] * (-position[2]) * v_circ / (r * p_xy),
        position[0] * (-position[2]) * v_circ / (r * p_xy),
        p_xy * v_circ / r,
    ];
    state.csm_state = StateVector {
        position,
        velocity,
        // Met(1) — non-zero so the N43 arm's "no CSM SV loaded"
        // short-circuit doesn't fire.
        epoch: Met(1),
        frame: Frame::EarthInertial,
    };

    // Independent ground truth from the same primitive `noun_display`
    // uses. The scenario runner ticks 10 cs per keystroke; the V37 ENTR
    // 21 ENTR sequence sends 7 keys. The *last* keystroke (Entr that
    // triggers dispatch_verb_noun → init_p21 → display_noun) reads
    // state.time BEFORE its own do_tick runs, so the noun arm sees
    // state.time = 6 × 10 cs = 60 cs. The 7th do_tick then bumps the
    // final state.time to 70 cs.
    const NOUN_READ_TIME_CS: u32 = 60;
    const SCENARIO_FINAL_TIME_CS: u32 = 70;
    let expected = p21_compute_ground_track(
        state.csm_state.position,
        state.csm_state.velocity,
        state.csm_state.epoch.to_seconds(),
        Met(NOUN_READ_TIME_CS).to_seconds(),
        state.gha_epoch_rad,
    );
    let want = DskyExpect {
        verb: Some(6),
        noun: Some(43),
        r0: Some(expected.lat_rad.to_degrees() as f32),
        r1: Some(expected.lon_rad.to_degrees() as f32),
        r2: Some((expected.alt_m / 1_000.0) as f32),
        flashing: None,
        // 0.1 % absorbs the f32 rounding through ECI → ECEF and
        // the rad → deg / m → km unit conversions.
        tol_pct: 0.1,
    };

    let scenario = ScenarioBuilder::new("ma5/p21_n43")
        .keys(&[
            Key::Verb,
            d(3),
            d(7),
            Key::Entr,
            d(2),
            d(1),
            Key::Entr,
        ])
        .expect_dsky(want)
        .build();

    let mut hw = SimHardware::new();
    run_scenario(&scenario, &mut state, &mut hw);

    // Pin the scenario runner's tick pattern: if its keystroke ticks
    // ever change cadence, the expected lat/lon/alt above will silently
    // drift. Assert here so the failure mode is "tick count changed",
    // not "lat/lon don't match" (the former is much easier to diagnose).
    assert_eq!(
        state.time,
        Met(SCENARIO_FINAL_TIME_CS),
        "scenario runner keystroke-tick cadence has changed; \
         update SCENARIO_FINAL_TIME_CS to match"
    );
}
