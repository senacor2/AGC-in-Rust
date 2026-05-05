//! Walks through the `docs/p40_burn_demo.md` keystroke sequence and prints
//! the AGC state at each step. Useful when triaging a desk-top demo
//! that looks broken on the `dsky_sim` console — running this binary
//! should finish with `engine=false burn_active=false achieved≈21 m/s`
//! after the SERVICER cuts off the burn at the ΔV target.
//!
//!     cargo run --example repro_demo_flow -p agc-test

use agc_core::services::v_n::{feed_key, Key};
use agc_core::types::Met;
use agc_core::AgcState;
use agc_sim::runtime::{pump_engine_to_hw, pump_pipa_into_state, pump_rcs_to_hw, WaitlistPump};
use agc_sim::SimHardware;

fn d(n: u8) -> Key {
    Key::Digit(n)
}
fn feed(state: &mut AgcState, keys: &[Key]) {
    for &k in keys {
        feed_key(state, k);
    }
}
fn feed_number(state: &mut AgcState, mut n: u32) {
    if n == 0 {
        feed_key(state, d(0));
        return;
    }
    let mut digits = [0u8; 6];
    let mut count = 0;
    while n > 0 {
        digits[count] = (n % 10) as u8;
        n /= 10;
        count += 1;
    }
    for i in (0..count).rev() {
        feed_key(state, d(digits[i]));
    }
}

fn main() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();
    let mut waitlist_pump = WaitlistPump::new();
    state.time = Met(0);

    // Step 1: V71 1 6 +6778 +0 +0 +0 +7669 +0
    feed(&mut state, &[Key::Verb, d(7), d(1), Key::Entr]);
    feed_number(&mut state, 1);
    feed_key(&mut state, Key::Entr);
    feed_number(&mut state, 6);
    feed_key(&mut state, Key::Entr);
    for v in [6778u32, 0, 0, 0, 7669, 0] {
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, v);
        feed_key(&mut state, Key::Entr);
    }
    println!(
        "After V71: pos={:?} vel={:?}",
        state.csm_state.position, state.csm_state.velocity
    );

    // V06 N44 (verification — apogee/perigee in km, half-period in min)
    feed(
        &mut state,
        &[Key::Verb, d(0), d(6), Key::Noun, d(4), d(4), Key::Entr],
    );
    println!(
        "After V06 N44: r=[{:.2} km, {:.2} km, {:.2} min]",
        state.dsky.r[0], state.dsky.r[1], state.dsky.r[2]
    );

    // V37 E30 E, V25 N33 (TIG = 5 min), V25 N81 (ΔV = +21).
    feed(
        &mut state,
        &[Key::Verb, d(3), d(7), Key::Noun, d(3), d(0), Key::Entr],
    );
    feed(
        &mut state,
        &[Key::Verb, d(2), d(5), Key::Noun, d(3), d(3), Key::Entr],
    );
    feed_number(&mut state, 0);
    feed_key(&mut state, Key::Entr);
    feed_number(&mut state, 5);
    feed_key(&mut state, Key::Entr);
    feed_number(&mut state, 0);
    feed_key(&mut state, Key::Entr);
    feed(
        &mut state,
        &[Key::Verb, d(2), d(5), Key::Noun, d(8), d(1), Key::Entr],
    );
    feed_key(&mut state, Key::Plus);
    feed_number(&mut state, 21);
    feed_key(&mut state, Key::Entr);
    feed_key(&mut state, Key::Plus);
    feed_number(&mut state, 0);
    feed_key(&mut state, Key::Entr);
    feed_key(&mut state, Key::Plus);
    feed_number(&mut state, 0);
    feed_key(&mut state, Key::Entr);

    // V37 E40 E
    feed(
        &mut state,
        &[Key::Verb, d(3), d(7), Key::Noun, d(4), d(0), Key::Entr],
    );
    waitlist_pump.tick(&mut state, &mut hw);
    println!(
        "After V37 E40 E: verb={} noun={} burn_active={} pending_v50={}",
        state.dsky.verb,
        state.dsky.noun,
        state.burn.burn_active,
        state.vn.pending_v50.is_some()
    );

    // PRO — arms the burn (engine still off until TIG)
    feed_key(&mut state, Key::Pro);
    println!(
        "After PRO: armed={} engine={} verb={} noun={}",
        state.burn.armed, state.engine_thrusting, state.dsky.verb, state.dsky.noun
    );

    // Run the soft executive forward in 100 ms slices until the burn
    // completes. Skip directly to TIG-1s to avoid walking the 5-minute
    // wait one tick at a time.
    let tig_cs = state.burn.tig.0;
    state.time = Met(tig_cs.saturating_sub(100));
    hw.timers.set_time(state.time.0);
    waitlist_pump.tick(&mut state, &mut hw);
    println!(
        "Approaching TIG (state.time={} cs, burn.tig={} cs): armed={} engine={}",
        state.time.0, state.burn.tig.0, state.burn.armed, state.engine_thrusting
    );

    let mut iters = 0u32;
    let mut ignition_iter = None;
    while state.burn.burn_active && iters < 6_000 {
        state.time = Met(state.time.0 + 10);
        hw.timers.set_time(state.time.0);
        hw.tick(0.1);
        pump_pipa_into_state(&mut state, &mut hw);
        waitlist_pump.tick(&mut state, &mut hw);
        pump_engine_to_hw(&state, &mut hw);
        pump_rcs_to_hw(&mut state, &mut hw);
        if state.engine_thrusting && ignition_iter.is_none() {
            ignition_iter = Some(iters);
        }
        iters += 1;
    }

    let ignition_iter = ignition_iter.unwrap_or(u32::MAX);
    let burn_iters = iters.saturating_sub(ignition_iter);
    let achieved = (state.burn.accumulated_dv_inertial[0].powi(2)
        + state.burn.accumulated_dv_inertial[1].powi(2)
        + state.burn.accumulated_dv_inertial[2].powi(2))
    .sqrt();
    println!(
        "After burn: ignited at iter {ignition_iter}, burned for {} s, achieved ΔV {:.2} m/s, burn_active={} engine={}",
        burn_iters as f64 * 0.1,
        achieved,
        state.burn.burn_active,
        state.engine_thrusting
    );
}
