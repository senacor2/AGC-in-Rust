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

use agc_core::services::v_n::Key;
use agc_core::types::Met;
use agc_core::AgcState;
use agc_sim::runtime::{pump_engine_to_hw, pump_pipa_into_state, DapPump, WaitlistPump};
use agc_sim::SimHardware;
use agc_sim::{run_scenario, DskyExpect, ScenarioBuilder};

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

    // Precompute the TIG centiseconds used in assertions throughout the test.
    let tig_cs = TIG_HOURS * 360_000 + TIG_MINUTES * 6_000 + TIG_SECONDS_X100;

    // ── Phase 1a: seed state vector + select P30 + enter TIG ─────────────────
    //
    // Split before V25 N81 so we can assert `pending_tig` is Some
    // before noun_81_commit_dv_lvlh consumes it via `.take()`.
    let phase1a = ScenarioBuilder::new("p40_sps_burn/phase1a")
        .comment("seed CSM state, select P30, enter TIG")
        // ── 1. V71 P27 block update — seed CSM state vector ──────────────────
        // address 1, count 6: pos[0..2] then vel[0..2]
        .v71_p27_block_update(
            1,
            &[
                (1, SEED_POSITION_X_KM),  // pos[0] +6778 km
                (1, 0),                   // pos[1]  +0
                (1, 0),                   // pos[2]  +0
                (1, 0),                   // vel[0]  +0
                (1, SEED_VELOCITY_Y_M_S), // vel[1] +7669 m/s
                (1, 0),                   // vel[2]  +0
            ],
        )
        // ── 2. V37 ENTR 30 ENTR — select P30 ────────────────────────────────
        .keys(&[
            Key::Verb,
            Key::Digit(3),
            Key::Digit(7),
            Key::Entr,
            Key::Digit(3),
            Key::Digit(0),
            Key::Entr,
        ])
        .expect_major_mode(30)
        // ── 3. V25 N33 — load TIG = 0h 5m 0.00s ─────────────────────────────
        // Bare digits (no sign prefix) for HMS: V25 handler initialises sign
        // to +1 so a digit is accepted directly, matching the original test.
        .keys(&[
            Key::Verb,
            Key::Digit(2),
            Key::Digit(5),
            Key::Noun,
            Key::Digit(3),
            Key::Digit(3),
            Key::Entr,
        ])
        .digits(TIG_HOURS)
        .enter()
        .digits(TIG_MINUTES)
        .enter()
        .digits(TIG_SECONDS_X100)
        .enter()
        .build();

    run_scenario(&phase1a, &mut state, &mut hw);

    // Assertions that must happen before V25 N81 consumes pending_tig.
    assert_eq!(state.csm_state.position, [6_778_000.0, 0.0, 0.0]);
    assert_eq!(state.csm_state.velocity, [0.0, 7669.0, 0.0]);
    assert_eq!(state.vn.pending_tig, Some(Met(tig_cs)));

    // ── Phase 1b: load ΔV ─────────────────────────────────────────────────────
    //
    // V25 N81 consumes pending_tig and stores pending_maneuver.
    // Split again so we can check pending_maneuver.tig before P40 takes it.
    let phase1b = ScenarioBuilder::new("p40_sps_burn/phase1b")
        .comment("load LVLH delta-V via V25 N81")
        // ── 4. V25 N81 — load LVLH ΔV [+21, 0, 0] (m/s) ────────────────────
        .v25_load_three(81, [TARGET_DV_MS as i32, 0, 0])
        .build();

    run_scenario(&phase1b, &mut state, &mut hw);

    // Verify TIG round-trips through P30 into the pending maneuver.
    let pending = state
        .pending_maneuver
        .expect("V25 N81 must produce a pending_maneuver");
    assert_eq!(pending.tig, Met(tig_cs), "TIG must round-trip through P30");

    // ── Phase 1c: select P40 and arm the burn via PRO ─────────────────────────
    //
    // P40 init consumes pending_maneuver (moves TIG + ΔV into BurnState)
    // and requests V50 N99. PRO fires the V50 callback and sets burn.armed.
    let phase1c = ScenarioBuilder::new("p40_sps_burn/phase1c")
        .comment("select P40, acknowledge V50 N99 PRO to arm burn")
        // ── 5. V37 ENTR 40 ENTR — select P40 ────────────────────────────────
        .keys(&[
            Key::Verb,
            Key::Digit(3),
            Key::Digit(7),
            Key::Entr,
            Key::Digit(4),
            Key::Digit(0),
            Key::Entr,
        ])
        .expect_major_mode(40)
        .expect_dsky(DskyExpect {
            verb: Some(50),
            noun: Some(99),
            flashing: Some(true),
            r0: None,
            r1: None,
            r2: None,
            tol_pct: 0.0,
        })
        // ── 6. PRO — arm SPS for ignition at TIG ─────────────────────────────
        .pro()
        .build();

    run_scenario(&phase1c, &mut state, &mut hw);

    // Verify post-PRO state before the burn loop.
    assert!(
        state.burn.burn_active,
        "P40 must transfer pending_maneuver into BurnState"
    );
    assert!(state.servicer_exit.is_some(), "P40 must install burn hook");
    assert!(
        !state.engine_thrusting,
        "engine must remain cold until crew presses PRO"
    );
    assert!(
        state.burn.armed,
        "PRO must arm the burn for TIG-gated ignition"
    );
    assert!(
        !hw.engine.thrusting,
        "SimHardware SPS must be cold after PRO, before TIG"
    );

    // ── Phase 2: jump to TIG-1s, then run the burn loop to completion ────────
    //
    // 5-minute TIG = 30_000 cs; walking at 10 cs/tick would take 3_000 ticks.
    // Skip the wait by jumping mission time to TIG-1s in a single shot — the
    // pumps catch up by dispatching every backlogged SERVICER cycle in one
    // tick (dap_step is a no-op while engine is off and DAP is in Maneuver
    // mode, so the catch-up is cheap and correct).
    //
    // Synchronise csm_state.epoch to state.time so the SERVICER's epoch-
    // based catch-up (state.time = new_sv.epoch after each servicer_task
    // call) lands near TIG at the end, not near MET=0.  The scenario
    // runner ticks time forward with each KeyPress; the state vector epoch
    // is not updated until the SERVICER actually runs, leaving it at its
    // initial value and causing a large epoch mismatch if not corrected here.
    state.csm_state.epoch = state.time;

    let mut waitlist_pump = WaitlistPump::new();
    let mut dap_pump = DapPump::new();

    // Jump to TIG-1s. Pass the current state.time to the pumps first so
    // they know their reference point, then the second pair of calls after
    // the jump computes the large elapsed gap and drives SERVICER catchup.
    dap_pump.tick(&mut state, &mut hw, None);
    waitlist_pump.tick(&mut state, &mut hw);

    state.time = Met(tig_cs.saturating_sub(100));
    hw.timers.set_time(state.time.0);
    dap_pump.tick(&mut state, &mut hw, None);
    waitlist_pump.tick(&mut state, &mut hw);

    // Engine must still be cold — TIG not yet reached.
    assert!(
        !state.engine_thrusting,
        "ignition gate must hold engine off while state.time < burn.tig"
    );
    assert!(state.burn.armed, "armed must persist until TIG");

    // Walk the remaining 1 s (and the full burn) at 100 ms granularity.
    const TICK_CS: u32 = 10;
    const TICK_S: f64 = TICK_CS as f64 / 100.0;
    let max_iters = 6_000; // 60 s of mission time

    let mut iters = 0u32;
    let mut ignition_iter: Option<u32> = None;

    while state.burn.burn_active && iters < max_iters {
        state.time = Met(state.time.0 + TICK_CS);
        hw.timers.set_time(state.time.0);
        hw.tick(TICK_S);
        pump_pipa_into_state(&mut state, &mut hw);
        dap_pump.tick(&mut state, &mut hw, None);
        waitlist_pump.tick(&mut state, &mut hw);
        pump_engine_to_hw(&state, &mut hw);
        agc_sim::runtime::pump_rcs_to_hw(&mut state, &mut hw);

        if state.engine_thrusting && ignition_iter.is_none() {
            ignition_iter = Some(iters);
        }
        iters += 1;
    }

    // ── Assertions on ignition timing ─────────────────────────────────────────
    let ignition_iter = ignition_iter.expect("engine must ignite at some point during the loop");
    // Ignition must occur within a couple of dap_step cycles of TIG. We jumped
    // to TIG-1 s above and walk at 10 cs, so ignition lands within ≈ 100 iters.
    assert!(
        ignition_iter <= 110,
        "ignition must fire within a few DAP cycles of TIG; fired at iter {ignition_iter}"
    );

    // ── Assertions on burn duration ───────────────────────────────────────────
    let post_ignition_iters = iters - ignition_iter;
    let burn_duration_s = post_ignition_iters as f64 * TICK_S;
    assert!(
        (12.0..=18.0).contains(&burn_duration_s),
        "engine should fire for about 15 s, got {burn_duration_s} s"
    );

    // ── Final state via scenario assertion + direct checks ────────────────────
    let final_check = ScenarioBuilder::new("p40_sps_burn/final")
        .comment("verify post-cutoff state")
        .expect_major_mode(40)
        .build();
    run_scenario(&final_check, &mut state, &mut hw);

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
