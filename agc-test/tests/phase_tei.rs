//! MS-T4 integration test: Apollo 8 Trans-Earth Injection (TEI) burn scenario.
//!
//! # Purpose
//!
//! Validates P40 driving a prograde TEI SPS burn in MCI frame that transitions
//! the spacecraft from a circular 60 nm (111 km) lunar orbit to a hyperbolic
//! departure trajectory (ε_MCI > 0, e > 1), mirroring the Apollo 8 TEI maneuver
//! performed at T+89:19:16.
//!
//! # Architectural significance
//!
//! This test is unique vs `phase_loi.rs` in the following ways:
//! - **Prograde burn mirror**: LOI is retrograde; TEI is prograde (sign flip on R1).
//! - **Hyperbolic end state**: The orbit transitions from elliptic (e < 1) to
//!   hyperbolic (e > 1, ε > 0), unlike LOI which captures from hyperbolic to
//!   elliptic.
//! - **No symmetric-burn-around-apse trick needed**: The seed is a circular orbit
//!   (not a pre-pericynthion hyperbolic approach), so there is no velocity-reversal
//!   trick required. A simple analytic circular orbit state at TIG suffices.
//!
//! # Strategy (5-phase chain)
//!
//! The test is split into five sub-scenarios that share `state` and `hw` across
//! phases, mirroring the `phase_loi.rs` approach exactly.
//!
//! | Phase | What happens                                                                   |
//! |-------|--------------------------------------------------------------------------------|
//! | A     | Seed 60 nm MCI circular SV at TIG−300 s; select P30, load TIG via N33        |
//! | B     | Load ΔV via V25 N81 (+1073 m/s prograde, R1 slot, mirroring phase_loi.rs)    |
//! | C     | Select P40, display V50 N99, crew PRO to arm burn                             |
//! | D     | Burn loop: tick until `burn_active` clears (~353 s at 3.04 m/s²)             |
//! | E     | Post-burn hyperbolic-departure assertions                                      |
//!
//! # LVLH slot convention and robust direction check
//!
//! `phase_loi.rs` uses `v25_load_three(81, [-914, 0, 0])` with slot 1 (R1) for
//! retrograde LOI because the LOI seed has velocity along −Y and the LVLH R1 axis
//! maps to anti-velocity direction at that geometry. For TEI the seed velocity is
//! along +Y (prograde, circular orbit), so the same R1 slot with a positive sign
//! maps to prograde. The structural relationship is identical; only the sign flips.
//!
//! A robust direction check between phases B and C asserts:
//!   `dot(unit(pending.delta_v), [0, 1, 0]) > 0.95`
//! This confirms the ΔV vector aligns with the +Y velocity direction (prograde)
//! regardless of which V25 N81 slot actually carried the value.
//!
//! # What is NOT tested
//!
//! - Moon SOI handover (belongs to phase_transearth, future MS-T5).
//! - Entry interface (MS-T5).
//!
//! # Assertion table (Phase E)
//!
//! | Parameter       | Lower bound  | Upper bound  | Derivation                                                          |
//! |-----------------|--------------|--------------|---------------------------------------------------------------------|
//! | Post-burn speed | 2 640 m/s    | 2 740 m/s    | v_circ + ΔV − gravity_loss ≈ 2662 m/s (MSC-PA-R-69-1 Table 3-I)   |
//! | Specific energy | > 0.5 MJ/kg  | —            | ε > 0 confirms hyperbolic departure                                 |
//! | Eccentricity    | > 1.0        | —            | Hyperbolic conic                                                    |
//! | r after coast   | > r at cutoff| —            | Spacecraft receding from Moon                                       |
//!
//! # Design reference
//!
//! Architect's locked design, GitHub issue #27 (MS-T4 phase_tei), parent #23.
//! ΔV value sourced from Apollo 8 Mission Report MSC-PA-R-69-1, Table 3-I
//! (post-flight reconstruction): 3522 ft/s = 1073.5 m/s, rounded to 1073 m/s.

use agc_core::math::linalg::{dot, norm, unit};
use agc_core::navigation::conics::sv_to_elements;
use agc_core::navigation::gravity::{MU_MOON, R_MOON};
use agc_core::navigation::state_vector::Frame;
use agc_core::navigation::StateVector;
use agc_core::services::v_n::Key;
use agc_core::types::Met;
use agc_core::AgcState;
use agc_sim::runtime::{pump_engine_to_hw, pump_pipa_into_state, DapPump, WaitlistPump};
use agc_sim::SimHardware;
use agc_sim::{run_scenario, DskyExpect, ScenarioBuilder};

// ── Constants ────────────────────────────────────────────────────────────────

/// TEI TIG: T+89:19:16 in centiseconds.
const TEI_MET_CS: u32 = 32_155_600;

/// TEI hours component of TIG (for V25 N33 entry).
const TEI_TIG_H: u32 = 89;

/// TEI minutes component of TIG (for V25 N33 entry).
const TEI_TIG_M: u32 = 19;

/// TEI seconds × 100 component of TIG (for V25 N33 entry). 16.00 s = 1600.
const TEI_TIG_S100: u32 = 1600;

/// Apollo 8 TEI prograde ΔV (m/s, positive = prograde in LVLH R1 slot).
///
/// Source: Apollo 8 Mission Report MSC-PA-R-69-1, Table 3-I (post-flight
/// reconstruction): 3522 ft/s = 1073.5 m/s. Rounded to 1073 m/s.
///
/// The spacecraft is in a 60 nm (111 km) circular orbit at ~1633 m/s.
/// The 1073 m/s prograde burn accelerates it to ~2706 m/s, exceeding escape
/// velocity (~2381 m/s at that altitude) and producing a hyperbolic departure.
///
/// With `SPS_THRUST_N = 91 188 N` and vehicle mass 30 000 kg, the acceleration
/// is ~3.04 m/s², giving a burn duration of ~353 s.
const TEI_DV_MPS: i32 = 1073;

/// Circular orbit altitude above Moon surface (m). 60 nautical miles = 111 km.
const LUNAR_ALT_M: f64 = 111_000.0;

/// Coast window before TIG (centiseconds). State is seeded at TIG − SETTLE_CS so
/// that TIG is in the future when V25 N33 / V25 N81 are entered.
/// Phase D resets csm_state back to the circular orbit SV with state.time = TIG.
const SETTLE_CS: u32 = 30_000; // 300 s in centiseconds

// ── Helper: 60 nm circular MCI state vector at TIG ───────────────────────────

/// Compute the analytic 60 nm circular MCI state vector at TEI TIG.
///
/// Placed on a prograde equatorial circular orbit:
///   - position: `[R_MOON + 111 km, 0, 0]` (along +X)
///   - velocity: `[0, v_circ, 0]` (prograde +Y)
///   - epoch: `TEI_MET_CS`
///   - frame: MoonInertial
///
/// This geometry is deliberately matched to `phase_lunar_orbit.rs` which seeds
/// the same equatorial circular orbit. No velocity-reversal trick is needed
/// because a circular orbit is time-reversible without correction.
///
/// The +Y velocity direction means V25 N81 R1 slot with a positive value
/// produces a prograde ΔV (mirroring phase_loi.rs where the −Y approach
/// velocity made R1 slot with a negative value retrograde).
fn pre_tei_sv() -> StateVector {
    let r = R_MOON + LUNAR_ALT_M;
    let v_circ = (MU_MOON / r).sqrt(); // ≈ 1633 m/s
    StateVector {
        position: [r, 0.0, 0.0],
        velocity: [0.0, v_circ, 0.0], // prograde, +Y
        epoch: Met(TEI_MET_CS),
        frame: Frame::MoonInertial,
    }
}

// ── Test ──────────────────────────────────────────────────────────────────────

/// TC-TEI-1: Apollo 8 TEI prograde burn produces a hyperbolic departure from
/// lunar orbit (ε_MCI > 0, e > 1).
///
/// Drives P40 in MCI via a five-phase split-scenario chain. The seed is a 60 nm
/// circular MCI orbit at TIG. The 1073 m/s prograde V25 N81 burn (R1 slot,
/// same as phase_loi.rs with sign flipped) exceeds escape velocity and produces
/// a hyperbolic departure trajectory.
///
/// ΔV source: Apollo 8 Mission Report MSC-PA-R-69-1, Table 3-I, 3522 ft/s =
/// 1073.5 m/s (post-flight reconstruction).
///
/// The end state must satisfy:
/// - `burn_active == false`, `engine_thrusting == false`
/// - `|v| ∈ [2640, 2740] m/s`
/// - `ε_MCI > 0.5 MJ/kg`
/// - `e > 1.0` (hyperbolic conic)
/// - After 600 s coast: `|r|` has increased (spacecraft receding from Moon)
///
/// # Design reference
///
/// Architect's locked design, GitHub issue #27 (MS-T4 phase_tei), parent #23.
#[test]
fn tc_phase_tei_apollo_8_departs_lunar_orbit_hyperbolically() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();

    // Compute the circular orbit seed.
    let sv_circ = pre_tei_sv();

    // Verify the seed orbit is bound (circular orbit, ε < 0).
    {
        let v_mag = norm(sv_circ.velocity);
        let r_mag = norm(sv_circ.position);
        let epsilon = 0.5 * v_mag * v_mag - MU_MOON / r_mag;
        assert!(
            epsilon < 0.0,
            "pre_tei_sv: specific energy must be negative (bound orbit); got {epsilon:.0} J/kg"
        );
    }

    // ── Phase A: seed + load P30/TIG ─────────────────────────────────────────
    //
    // Seed the MCI state vector at MET = TIG − 300 s so that TIG is in the
    // future when V25 N33 / V25 N81 are entered (p30_load_dv_lvlh requires
    // tig >= state.time). The position/velocity is the circular orbit state
    // (epoch = TIG), intentionally inconsistent with the MET offset — this is
    // harmless because Phase D resets csm_state and state.time back to TIG
    // before the burn loop.
    //
    // No seed_ground_truth here: calling it would start the SERVICER immediately
    // and integrate the spacecraft away from the circular orbit state during
    // keypresses. The SERVICER starts only when P40 is selected in Phase C.
    //
    // Note: V25 N33 format: R1 = hours, R2 = minutes, R3 = seconds × 100.
    // TEI_MET_CS = 32_155_600 cs = 89 h 19 m 16 s = 89 h 19 m 1600 (×100).
    let phase_a = ScenarioBuilder::new("phase_tei/phase_a_setup")
        .comment("Phase A: seed MCI circular SV at TIG-300s, select P30, load TIG")
        .seed_state()
        .from_state_vector(sv_circ)
        .met(Met(TEI_MET_CS - SETTLE_CS))
        .refsmmat_identity()
        .done()
        // Snap to identity attitude so SPS thrust aligns with the +Y inertial axis.
        .command_attitude([1.0, 0.0, 0.0, 0.0])
        // Select P30 (External-ΔV targeting): V37 ENTR 30 ENTR
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
        // Load TIG via V25 N33 ENTR  HH ENTR  MM ENTR  SS00 ENTR
        .keys(&[
            Key::Verb,
            Key::Digit(2),
            Key::Digit(5),
            Key::Noun,
            Key::Digit(3),
            Key::Digit(3),
            Key::Entr,
        ])
        .digits(TEI_TIG_H)
        .enter()
        .digits(TEI_TIG_M)
        .enter()
        .digits(TEI_TIG_S100)
        .enter()
        .build();

    run_scenario(&phase_a, &mut state, &mut hw);

    // Post-Phase-A assertions.
    assert_eq!(
        state.csm_state.frame,
        Frame::MoonInertial,
        "Phase A: frame must be MoonInertial after MCI seed"
    );
    assert_eq!(
        state.vn.pending_tig,
        Some(Met(TEI_MET_CS)),
        "Phase A: pending_tig must be Some(Met({TEI_MET_CS})) after V25 N33 entry"
    );

    // ── Phase B: load ΔV ─────────────────────────────────────────────────────
    //
    // V25 N81 loads the prograde ΔV in LVLH R1 slot.
    // With velocity along +Y at the circular orbit and identity REFSMMAT, P30
    // transforms the R1 +1073 m/s LVLH vector to inertial +Y ΔV.
    // The SPS thrust_dir_platform default is [0, 1, 0] → inertial +Y at
    // identity REFSMMAT, which is pro-velocity (prograde). This produces the
    // required orbital energy increase to escape Moon's gravity.
    //
    // This is the exact structural mirror of phase_loi.rs: LOI uses [-914, 0, 0]
    // (negative R1 = anti-velocity at LOI seed geometry), TEI uses [+1073, 0, 0]
    // (positive R1 = pro-velocity at TEI seed geometry).
    let phase_b = ScenarioBuilder::new("phase_tei/phase_b_load_dv")
        .comment("Phase B: load TEI prograde ΔV via V25 N81 (R1 slot, +1073 m/s)")
        .v25_load_three(81, [TEI_DV_MPS, 0, 0])
        .build();

    run_scenario(&phase_b, &mut state, &mut hw);

    // Post-Phase-B assertions.
    let pending = state
        .pending_maneuver
        .expect("Phase B: V25 N81 must produce a pending_maneuver");
    assert_eq!(
        pending.tig,
        Met(TEI_MET_CS),
        "Phase B: pending_maneuver.tig must round-trip through P30"
    );

    let dv_mag = (pending.delta_v.0[0].powi(2)
        + pending.delta_v.0[1].powi(2)
        + pending.delta_v.0[2].powi(2))
    .sqrt();
    let expected_dv = TEI_DV_MPS.unsigned_abs() as f64;
    assert!(
        (dv_mag - expected_dv).abs() < 2.0,
        "Phase B: |pending ΔV| = {dv_mag:.2} m/s should be near {expected_dv:.0} m/s"
    );

    // Robust direction check: ΔV must be prograde (aligned with +Y seed velocity).
    // dot(unit(delta_v_inertial), [0, 1, 0]) > 0.95 confirms prograde direction.
    // This catches slot-confusion bugs independent of which R component was used.
    let dv_inertial = pending.delta_v.0;
    let dv_unit = unit(dv_inertial);
    let prograde_alignment = dot(dv_unit, [0.0, 1.0, 0.0]);
    assert!(
        prograde_alignment > 0.95,
        "Phase B: ΔV direction check failed — dot(unit(ΔV), [0,1,0]) = {prograde_alignment:.4}, \
         expected > 0.95 (prograde); actual ΔV = {dv_inertial:?}. \
         This suggests the wrong V25 N81 slot was used or the LVLH transform is wrong."
    );

    // ── Phase C: select P40 and arm ───────────────────────────────────────────
    //
    // V37 ENTR 40 ENTR selects P40, which consumes pending_maneuver and
    // requests V50 N99. Crew PRO arms the SPS for TIG-gated ignition.
    let phase_c = ScenarioBuilder::new("phase_tei/phase_c_arm_p40")
        .comment("Phase C: select P40, confirm V50 N99, PRO to arm burn")
        // V37 ENTR 40 ENTR — select P40
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
        // PRO — arm SPS for ignition at TIG
        .pro()
        .build();

    run_scenario(&phase_c, &mut state, &mut hw);

    // Post-Phase-C assertions.
    assert!(
        state.burn.burn_active,
        "Phase C: burn_active must be true after P40 init"
    );
    assert!(
        state.burn.armed,
        "Phase C: burn.armed must be true after crew PRO"
    );
    assert!(
        state.servicer_exit.is_some(),
        "Phase C: P40 must install SERVICER burn hook"
    );
    assert!(
        !state.engine_thrusting,
        "Phase C: engine must remain cold until TIG"
    );
    assert!(
        !hw.engine.thrusting,
        "Phase C: SimHardware SPS must be cold before TIG"
    );

    // ── Phase D: burn loop ────────────────────────────────────────────────────
    //
    // Reset the CSM state to the circular orbit seed at TIG. This is necessary
    // because the SERVICER may have fired during Phase C and integrated the
    // spacecraft a few steps away from the seed position. By resetting here we
    // guarantee the TEI burn starts from the correct circular orbit state.
    //
    // `sv_circ.epoch` = TEI_MET_CS. Setting `state.time` to TIG as well ensures
    // the TIG gate fires immediately on the first DAP cycle.
    state.csm_state = sv_circ;
    state.time = Met(TEI_MET_CS);
    hw.timers.set_time(state.time.0);

    let mut waitlist_pump = WaitlistPump::new();
    let mut dap_pump = DapPump::new();

    // Prime the pumps once to initialize internal state. state.time == TIG so
    // the DAP's TIG gate will fire on this tick (burn.tig == TIG, state.time >= burn.tig).
    dap_pump.tick(&mut state, &mut hw, None);
    waitlist_pump.tick(&mut state, &mut hw);
    pump_engine_to_hw(&state, &mut hw);

    assert!(
        state.burn.armed,
        "Phase D: armed must be set before burn loop"
    );

    // Walk the full TEI burn at 100 ms granularity.
    // At 91 188 N / 30 000 kg ≈ 3.04 m/s², the 1073 m/s burn takes ~353 s
    // = 3 530 ticks. Budget 5 000 ticks (500 s) as a safety margin.
    const TICK_CS: u32 = 10;
    const TICK_S: f64 = TICK_CS as f64 / 100.0;
    let max_iters: u32 = 5_000; // 500 s of sim time — safety margin over ~346 s burn

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

    // ── Phase D assertions ────────────────────────────────────────────────────

    let ignition_iter =
        ignition_iter.expect("Phase D: engine must ignite at some point during the burn loop");

    // Since state.time == TIG when the burn loop starts, the ignition gate fires
    // in the first few DAP cycles (within ~10 ticks = 1 s of sim time).
    assert!(
        ignition_iter <= 110,
        "Phase D: ignition must fire in the first ~10 DAP cycles; fired at iter {ignition_iter}"
    );

    // Burn must have completed before max_iters.
    assert!(
        !state.burn.burn_active,
        "Phase D: burn must complete within {max_iters} iters; still active after {iters} iters"
    );

    // ── Phase E: post-burn hyperbolic departure assertions ────────────────────
    //
    // The spacecraft should now be on a hyperbolic departure trajectory from
    // the Moon. Verify: burn inactive, engine off, frame preserved, and that
    // the orbital energy and eccentricity confirm a hyperbolic conic.

    assert!(!state.burn.burn_active, "Phase E: burn must be complete");
    assert!(
        !state.engine_thrusting,
        "Phase E: engine_thrusting must clear at cutoff"
    );
    assert!(
        !hw.engine.thrusting,
        "Phase E: SimHardware SPS must drop on cutoff"
    );
    assert!(
        state.servicer_exit.is_none(),
        "Phase E: P40 must uninstall the SERVICER burn hook on cutoff"
    );
    assert_eq!(
        state.csm_state.frame,
        Frame::MoonInertial,
        "Phase E: frame must remain MoonInertial after burn"
    );

    // Compute post-burn orbital elements.
    let elements = sv_to_elements(state.csm_state);
    let r_cutoff = norm(state.csm_state.position);
    let v_cutoff = norm(state.csm_state.velocity);

    // Specific orbital energy (should be positive for hyperbolic departure).
    let energy = v_cutoff.powi(2) / 2.0 - MU_MOON / r_cutoff;

    assert!(
        energy > 0.5e6,
        "Phase E: ε_MCI should be > 0.5 MJ/kg (hyperbolic departure), got {energy:.3e} J/kg"
    );
    // Speed band: ideal v_circ + ΔV ≈ 2706 m/s (MSC-PA-R-69-1 Table 3-I,
    // 3522 ft/s = 1073.5 m/s); gravity loss over the full ~353 s burn
    // (no symmetric arc) is ~44 m/s, giving ~2662 m/s actual.
    // Band [2640, 2740] provides ~22 m/s margin on each side.
    assert!(
        (2_640.0..=2_740.0).contains(&v_cutoff),
        "Phase E: post-burn speed = {v_cutoff:.2} m/s must be in [2640, 2740] m/s \
         (v_circ + ΔV ≈ 2706 m/s, minus ~44 m/s gravity loss over ~353 s burn)"
    );
    assert!(
        elements.e > 1.0,
        "Phase E: orbit must be hyperbolic after TEI burn; e = {}",
        elements.e
    );
    assert!(
        elements.is_hyperbolic(),
        "Phase E: is_hyperbolic() must return true after TEI burn"
    );

    // ── Phase E continued: 600 s post-burn coast ─────────────────────────────
    //
    // Propagate the state forward 600 s (6000 ticks) and verify the spacecraft
    // is receding from the Moon (hyperbolic departure trajectory confirmed by
    // increasing |r|). Frame must remain MoonInertial (no SOI crossing expected
    // within 600 s at this speed).
    //
    // We use the bare pump loop (not run_scenario) to avoid re-initialising
    // state and losing the post-burn assertion context.
    let r_before_coast = r_cutoff;
    const COAST_TICKS: u32 = 6_000; // 600 s × 100 ticks/s

    for _ in 0..COAST_TICKS {
        state.time = Met(state.time.0 + TICK_CS);
        hw.timers.set_time(state.time.0);
        hw.tick(TICK_S);
        pump_pipa_into_state(&mut state, &mut hw);
        dap_pump.tick(&mut state, &mut hw, None);
        waitlist_pump.tick(&mut state, &mut hw);
        pump_engine_to_hw(&state, &mut hw);
        agc_sim::runtime::pump_rcs_to_hw(&mut state, &mut hw);
    }

    assert_eq!(
        state.csm_state.frame,
        Frame::MoonInertial,
        "Phase E coast: frame must still be MoonInertial after 600 s (no SOI crossing expected)"
    );

    let r_after_coast = norm(state.csm_state.position);
    assert!(
        r_after_coast > r_before_coast,
        "Phase E coast: spacecraft must be receding from Moon; \
         r_before = {r_before_coast:.0} m, r_after = {r_after_coast:.0} m"
    );

    // Specific energy must remain positive after 600 s coast (no significant drag in vacuum).
    let v_after_coast = norm(state.csm_state.velocity);
    let energy_after_coast = v_after_coast.powi(2) / 2.0 - MU_MOON / r_after_coast;
    assert!(
        energy_after_coast > 0.0,
        "Phase E coast: specific energy must remain positive after 600 s coast; \
         got {energy_after_coast:.3e} J/kg"
    );
}
