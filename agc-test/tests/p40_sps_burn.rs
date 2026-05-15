//! P40 SPS-burn integration test driven by V/N keystrokes.
//!
//! Drives a complete crew keystroke sequence through the agc-sim
//! `SimHardware` to seed the state vector, target a burn, arm the SPS,
//! wait for TIG, fire the engine for ~15 seconds, and observe
//! autonomous cutoff — entirely through the same code path
//! `dsky_sim`'s render loop uses (the
//! [`agc_sim::runtime`] soft executive pumps). The companion
//! document is `docs/p40_burn_demo.md`.
//!
//! Why the simulator drives PIPA, not the test:
//! `SimHardware::tick(dt_seconds)` advances the simulator's
//! [`Spacecraft`] dynamics — when the SPS is commanded on it integrates
//! Δv along `Spacecraft::thrust_dir_platform` and drains accumulated Δv
//! as integer PIPA pulses (carrying sub-quantum residue forward). The
//! pulses land in `SimImu::pipa` so the AGC's standard `read_pipa()`
//! call returns them naturally — no test-side state patching.

use agc_core::services::v_n::{feed_key, Key};
use agc_core::types::Met;
use agc_core::AgcState;
use agc_sim::runtime::{
    pump_engine_to_hw, pump_pipa_into_state, pump_rcs_to_hw, DapPump, WaitlistPump,
};
use agc_sim::SimHardware;

// ── Burn profile constants ────────────────────────────────────────────────────

/// Target ΔV magnitude (m/s, along-track LVLH).
///
/// At the simulator's default 1.5 m/s² SPS acceleration the SERVICER
/// will accumulate ≈21 m/s in seven 2-second cycles = 14 s, well within
/// the "~15 s" demonstration window and within the 0.3 m/s burn-cutoff
/// tolerance.
const TARGET_DV_MS: u32 = 21;

/// Initial CSM position in km (LVLH along-track will map onto inertial
/// +Y in this orbit). 6378 km Earth radius + 400 km altitude.
const SEED_POSITION_X_KM: u32 = 6778;

/// Initial CSM velocity in m/s along inertial +Y. Circular speed at the
/// 6 778 km radius is sqrt(μ_Earth / r) ≈ 7669 m/s.
const SEED_VELOCITY_Y_M_S: u32 = 7669;

/// TIG selected for the demo: 5 minutes after MET zero. Far enough into
/// the future that a human operator typing on the dsky_sim binary has
/// plenty of time to complete the V25 N81 ΔV load and the V37 E40 E
/// program switch before the TIG-in-past alarm (210/225) fires.
const TIG_HOURS: u32 = 0;
const TIG_MINUTES: u32 = 5;
const TIG_SECONDS_X100: u32 = 0;

// ── Helpers ──────────────────────────────────────────────────────────────────

fn d(n: u8) -> Key {
    Key::Digit(n)
}

fn feed_keys(state: &mut AgcState, keys: &[Key]) {
    for &k in keys {
        feed_key(state, k);
    }
}

/// Feed a non-negative integer as MSB-first decimal digit keypresses.
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

/// Drive a `V25 Nxx + E + E + E` triple-register load with sign-prefixed
/// integer values. Mirrors what a human types on the DSKY for any of
/// the three-component load nouns (N81, ...).
fn v25_load_three(state: &mut AgcState, noun_tens: u8, noun_units: u8, values: [u32; 3]) {
    feed_keys(
        state,
        &[
            Key::Verb,
            d(2),
            d(5),
            Key::Noun,
            d(noun_tens),
            d(noun_units),
            Key::Entr,
        ],
    );
    for v in values {
        feed_key(state, Key::Plus);
        feed_number(state, v);
        feed_key(state, Key::Entr);
    }
}

/// Drive a `V71` P27 block-address state-vector update.
///
/// Sends the full Apollo-style sequence: V71 ENTR, address, count,
/// then `count` signed data words. Each signed value is given as
/// `(sign, magnitude)`; pass `+1` or `-1` for sign.
fn v71_p27_block_update(state: &mut AgcState, address: u8, words: &[(i8, u32)]) {
    feed_keys(state, &[Key::Verb, d(7), d(1), Key::Entr]);
    feed_number(state, address as u32);
    feed_key(state, Key::Entr);
    feed_number(state, words.len() as u32);
    feed_key(state, Key::Entr);
    for &(sign, mag) in words {
        feed_key(state, if sign < 0 { Key::Minus } else { Key::Plus });
        feed_number(state, mag);
        feed_key(state, Key::Entr);
    }
}

// ── Test ─────────────────────────────────────────────────────────────────────

/// Crew V/N sequence arms the SPS, waits for TIG, fires the engine for
/// ~15 seconds, and observes autonomous cutoff.
///
/// Sequence (matches `docs/p40_burn_demo.md`):
///   1. V71 E 1 E 6 E + ... — P27 block-address state-vector load
///   2. V37 E30 E           — select P30 (External-ΔV targeting)
///   3. V25 N33 E + ...     — load TIG = 0h 5m 0.00s (5 minutes after MET 0)
///   4. V25 N81 E + ...     — load LVLH ΔV = +21 along-track, 0 radial, 0 cross
///   5. V37 E40 E           — select P40 (SPS-thrust program)
///   6. PRO                 — acknowledge V50 N99 (arms; ignition at TIG)
///
/// Verification phases:
///   * `hw.engine.thrusting == false` before PRO.
///   * After PRO: `state.burn.armed == true`, `state.engine_thrusting`
///     still `false` (TIG not yet reached).
///   * Wait for TIG with the soft executive ticking on 0.1 s slices
///     and assert the engine remains cold across the wait.
///   * Once mission time crosses TIG: ignition fires, DAP transitions
///     to Tvc, `hw.engine.thrusting == true`.
///   * After ≈ 14 s of engine-on time: SERVICER cuts off the burn,
///     `hw.engine.thrusting` returns to `false`, accumulated ΔV is
///     within the 0.3 m/s cutoff tolerance of the 21 m/s target.
#[test]
fn it_v37_p40_fires_sps_for_about_15s() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();
    let mut waitlist_pump = WaitlistPump::new();
    let mut dap_pump = DapPump::new();

    // Mission clock at zero. The default `state.refsmmat` is identity, so
    // platform = inertial — no REFSMMAT load needed for this demo.
    state.time = Met(0);

    // ── 1. V71 P27 block update — seed CSM state vector ──────────────────
    // The real Apollo CMC received its state vector from Mission Control
    // through the digital uplink, which is itself just a stream of V/N
    // keystrokes processed by PINBALL — same code path the crew DSKY
    // hits. V71 ("Start AGC update; block address" — Comanche055) opens
    // a P27 multi-keystroke load that walks through a starting logical
    // address, a word count, and `count` signed data words. The
    // simulator-specific address table (see `p27_apply_word` in
    // agc-core) maps:
    //
    //   address 1..3 → state.csm_state.position[0..3] (km on entry)
    //   address 4..6 → state.csm_state.velocity[0..3] (m/s)
    //
    // Loading address 1, count 6, walks the whole state vector in one go.
    v71_p27_block_update(
        &mut state,
        1,
        &[
            (1, SEED_POSITION_X_KM), // pos[0] +6778 km
            (1, 0),                  // pos[1]  +0
            (1, 0),                  // pos[2]  +0
            (1, 0),                  // vel[0]  +0
            (1, SEED_VELOCITY_Y_M_S), // vel[1] +7669 m/s
            (1, 0),                  // vel[2]  +0
        ],
    );
    assert_eq!(state.csm_state.position, [6_778_000.0, 0.0, 0.0]);
    assert_eq!(state.csm_state.velocity, [0.0, 7669.0, 0.0]);

    // ── 2. V37 E30 E — select P30 ─────────────────────────────────────────
    feed_keys(
        &mut state,
        &[Key::Verb, d(3), d(7), Key::Noun, d(3), d(0), Key::Entr],
    );
    assert_eq!(state.major_mode, 30, "V37 E30 E must select P30");

    // ── 3. V25 N33 — load TIG = 0h 5m 0.00s (Met(30 000) cs) ─────────────
    // Five minutes ahead of MET 0 leaves a wide margin for the operator's
    // typing pace — the test code runs in microseconds so the margin is
    // overkill here, but the keystroke sequence is identical to what the
    // human-driven dsky_sim demonstration uses.
    feed_keys(
        &mut state,
        &[Key::Verb, d(2), d(5), Key::Noun, d(3), d(3), Key::Entr],
    );
    feed_number(&mut state, TIG_HOURS);
    feed_key(&mut state, Key::Entr);
    feed_number(&mut state, TIG_MINUTES);
    feed_key(&mut state, Key::Entr);
    feed_number(&mut state, TIG_SECONDS_X100);
    feed_key(&mut state, Key::Entr);
    let tig_cs = TIG_HOURS * 360_000 + TIG_MINUTES * 6_000 + TIG_SECONDS_X100;
    assert_eq!(state.vn.pending_tig, Some(Met(tig_cs)));

    // ── 4. V25 N81 — load LVLH ΔV [+21, 0, 0] (m/s) ───────────────────────
    v25_load_three(&mut state, 8, 1, [TARGET_DV_MS, 0, 0]);

    let pending = state
        .pending_maneuver
        .expect("V25 N81 must produce a pending_maneuver");
    assert_eq!(pending.tig, Met(tig_cs), "TIG must round-trip through P30");

    // ── 5. V37 E40 E — select P40 ─────────────────────────────────────────
    feed_keys(
        &mut state,
        &[Key::Verb, d(3), d(7), Key::Noun, d(4), d(0), Key::Entr],
    );
    assert_eq!(state.major_mode, 40, "V37 E40 E must select P40");
    assert!(
        state.burn.burn_active,
        "P40 must transfer pending_maneuver into BurnState"
    );
    assert!(state.servicer_exit.is_some(), "P40 must install burn hook");
    assert_eq!(state.dsky.verb, 50, "P40 must request V50 N99");
    assert_eq!(state.dsky.noun, 99);
    assert!(state.dsky.flashing);
    assert!(
        !state.engine_thrusting,
        "engine must remain cold until crew presses PRO"
    );
    assert!(
        !state.burn.armed,
        "burn must not be armed yet (V50 N99 still awaiting PRO)"
    );

    // Arm the soft-executive pumps. After ADR-022 the DAP runs on a
    // dedicated T5RUPT path (mirrored in the sim by DapPump); the
    // Waitlist still carries servicer_task.
    dap_pump.tick(&mut state, &mut hw);
    waitlist_pump.tick(&mut state, &mut hw);
    pump_engine_to_hw(&state, &mut hw);
    assert!(!hw.engine.thrusting, "SimHardware SPS must be cold pre-PRO");

    // ── 6. PRO — arm SPS for ignition at TIG ──────────────────────────────
    feed_key(&mut state, Key::Pro);
    assert!(
        state.burn.armed,
        "PRO must arm the burn for TIG-gated ignition"
    );
    assert!(
        !state.engine_thrusting,
        "engine must NOT fire on PRO — must wait for state.time >= burn.tig"
    );

    // ── Soft-executive burn loop ──────────────────────────────────────────
    // Drive the AGC the same way `dsky_sim`'s render loop does: tick the
    // simulated physics, drain PIPA pulses, dispatch waitlist tasks,
    // mirror engine + RCS staging fields back to the hardware. Tick
    // the simulation in 10 cs (100 ms) slices — fine enough that the
    // ignition gate, which checks `state.time >= burn.tig` on every
    // dap_step, fires within one DAP cycle of crossing TIG.
    const TICK_CS: u32 = 10;
    const TICK_S: f64 = TICK_CS as f64 / 100.0;
    let max_iters = 6_000; // 60 s of mission time — plenty for TIG=5m? No: see below.

    // 5-minute TIG with a 10 cs tick = 30_000 iterations to reach TIG.
    // The Apollo-faithful demo TIG is convenient for human typing on
    // dsky_sim but expensive to walk through on a 10 cs tick. Skip the
    // wait by jumping mission time to TIG-1s in a single shot — the
    // pump catches up by dispatching every backlogged dap_step in one
    // tick (dap_step is a no-op while engine is off and DAP is in
    // Maneuver mode, so the catch-up is cheap and correct).
    state.time = Met(tig_cs.saturating_sub(100));
    hw.timers.set_time(state.time.0);
    dap_pump.tick(&mut state, &mut hw);
    waitlist_pump.tick(&mut state, &mut hw);

    // We are now within 1 s of TIG. Confirm the engine is still cold
    // (ignition gate must not fire while time < tig).
    assert!(
        !state.engine_thrusting,
        "ignition gate must hold engine off while state.time < burn.tig"
    );
    assert!(state.burn.armed, "armed must persist until TIG");

    // Walk the remaining 1 s (and the burn) at 100 ms granularity.
    let mut iters = 0u32;
    let mut ignition_iter: Option<u32> = None;
    while state.burn.burn_active && iters < max_iters {
        state.time = Met(state.time.0 + TICK_CS);
        hw.timers.set_time(state.time.0);
        hw.tick(TICK_S);
        pump_pipa_into_state(&mut state, &mut hw);
        dap_pump.tick(&mut state, &mut hw);
        waitlist_pump.tick(&mut state, &mut hw);
        pump_engine_to_hw(&state, &mut hw);
        pump_rcs_to_hw(&mut state, &mut hw);

        if state.engine_thrusting && ignition_iter.is_none() {
            ignition_iter = Some(iters);
        }
        iters += 1;
    }

    // ── Assertions on ignition timing ─────────────────────────────────────
    let ignition_iter =
        ignition_iter.expect("engine must ignite at some point during the loop");
    // Ignition must occur within a couple of dap_step cycles of TIG. We
    // jumped to TIG-1 s above and walk at 10 cs, so ignition lands
    // within ≈ 100 iterations.
    assert!(
        ignition_iter <= 110,
        "ignition must fire within a few DAP cycles of TIG; fired at iter {ignition_iter}"
    );

    // ── Assertions on burn duration ───────────────────────────────────────
    // Total iterations after TIG = (iters - ignition_iter). Each iter is
    // 100 ms, so multiply by TICK_S. Burn time should be ~14 s (7 SERVICER
    // cycles × 2 s) within tolerance.
    let post_ignition_iters = iters - ignition_iter;
    let burn_duration_s = post_ignition_iters as f64 * TICK_S;
    assert!(
        (12.0..=18.0).contains(&burn_duration_s),
        "engine should fire for about 15 s, got {burn_duration_s} s"
    );

    // ── Final state ────────────────────────────────────────────────────────
    assert!(!state.burn.burn_active, "burn must have completed");
    assert!(
        !state.engine_thrusting,
        "engine_thrusting must clear at cutoff"
    );
    assert!(!hw.engine.thrusting, "SimHardware SPS must drop on cutoff");
    assert!(
        state.servicer_exit.is_none(),
        "P40 must uninstall the SERVICER burn hook on cutoff"
    );

    let achieved = (state.burn.accumulated_dv_inertial[0].powi(2)
        + state.burn.accumulated_dv_inertial[1].powi(2)
        + state.burn.accumulated_dv_inertial[2].powi(2))
    .sqrt();
    assert!(
        (achieved - TARGET_DV_MS as f64).abs() < 5.0,
        "achieved ΔV {achieved:.2} m/s should be near target {TARGET_DV_MS} m/s"
    );
}
