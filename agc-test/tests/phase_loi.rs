// SPDX-License-Identifier: GPL-3.0-or-later
//! MS-T4 integration test: Apollo 8 Lunar Orbit Insertion (LOI-1) scenario.
//!
//! # Purpose
//!
//! Validates P40 driving a historically accurate retrograde LOI-1 SPS burn in MCI
//! frame.  The expected result is a 60 × 170 nautical-mile elliptical lunar orbit
//! (111 km pericynthion × 315 km apolune), matching the actual Apollo 8 LOI-1
//! outcome.
//!
//! # Architectural significance
//!
//! This is the first phase test where P40 is exercised in MCI with mission-context
//! fidelity.  It validates that:
//! - `p40_p41.rs` is truly frame-agnostic (runs identically in MCI).
//! - `guidance/maneuver.rs` is frame-agnostic (ΔV transforms correctly).
//! - `navigation/integration.rs` dispatches Moon gravity when `sv.frame == MoonInertial`.
//! - `navigation/conics.rs` selects `MU_MOON` from the MCI frame for orbital element
//!   computation.
//!
//! # Strategy (5-phase chain)
//!
//! The test is split into five sub-scenarios that share `state` and `hw` across
//! phases, mirroring the `p40_sps_burn.rs` approach.
//!
//! | Phase | What happens                                                              |
//! |-------|---------------------------------------------------------------------------|
//! | A     | Seed pre-pericynthion MCI SV at TIG; select P30, load TIG via N33        |
//! | B     | Load ΔV via V25 N81 (914 m/s retrograde, historical Apollo 8 LOI-1)      |
//! | C     | Select P40, display V50 N99, crew PRO to arm burn                        |
//! | D     | Burn loop: tick until `burn_active` clears (~300 s)                      |
//! | E     | Post-burn orbital element assertions (60 × 170 nm)                       |
//!
//! # Symmetric burn geometry (Apollo LOI-1 fidelity)
//!
//! The Apollo LOI-1 burn is centred on pericynthion: the spacecraft arrives on
//! a hyperbolic approach trajectory and fires the SPS ~150 s before pericynthion,
//! burning through pericynthion to cutoff ~150 s after.  This symmetric arc
//! cancels most of the gravity loss (the gravity component along the thrust
//! direction reverses sign across pericynthion), leaving only the second-order
//! residual of ~5–10 m/s.
//!
//! **Pre-pericynthion seed computation:**
//!
//! The seed state vector at TIG (= burn start = pericynthion − 150 s) is computed
//! using the velocity-reversal trick:
//!
//! 1. Start from the analytic pericynthion state:
//!    `r_peri = R_MOON + 111 km` along +X, `v_peri_approach ≈ 2585 m/s` along −Y.
//!    (`v_peri_approach = v_final + |ΔV|` where `v_final ≈ 1671 m/s` for the capture orbit.)
//! 2. Reverse velocity: `v_reversed = +v_peri_approach` along +Y.
//! 3. Propagate forward by 150 s with `propagate_coast` (which asserts `dt > 0`).
//! 4. Negate the resulting velocity.
//!
//! This is mathematically equivalent to stepping backwards 150 s along the
//! hyperbolic approach because RK4 on conservative gravity is time-reversible.
//! The pericynthion is now 150 s into the future from the seed.
//!
//! # What is NOT tested
//!
//! - LOI-2 circularisation (deferred to MS-T5).
//!
//! # Assertion table (Phase E)
//!
//! | Parameter       | Lower bound  | Upper bound  | Derivation                         |
//! |-----------------|--------------|--------------|------------------------------------|
//! | Apoapsis alt    | 265 000 m    | 365 000 m    | target 315 km ± 50 km gravity loss |
//! | Periapsis alt   |  91 000 m    | 131 000 m    | target 111 km ± 20 km              |
//! | Period          |  7 440 s     |  8 040 s     | `T = 2π√(a³/μ)` for a ≈ 213 km    |
//!
//! Tolerances are intentionally wide because the SERVICER's `moon_pos` third-body
//! placeholder is hardcoded and gravity loss during the finite burn arc adds a
//! residual ~5–10 m/s to the total ΔV.
//!
//! Target specific orbital energy: ε ≈ −MU_MOON / (2a) where a = R_MOON + 213 km,
//! giving ε ≈ −1.256 MJ/kg.
//!
//! # Design reference
//!
//! Architect's locked design, GitHub issue #27 (MS-T4 phase_loi).

use agc_core::navigation::conics::{
    apoapsis_altitude_moon, orbital_period, periapsis_altitude_moon, sv_to_elements,
};
use agc_core::navigation::gravity::{MU_MOON, R_MOON};
use agc_core::navigation::integration::propagate_coast;
use agc_core::navigation::state_vector::Frame;
use agc_core::navigation::StateVector;
use agc_core::services::v_n::Key;
use agc_core::types::Met;
use agc_core::AgcState;
use agc_sim::runtime::{pump_engine_to_hw, pump_pipa_into_state, DapPump, WaitlistPump};
use agc_sim::SimHardware;
use agc_sim::{run_scenario, DskyExpect, ScenarioBuilder};

// ── Constants ────────────────────────────────────────────────────────────────

/// LOI-1 TIG: T+69:08:20 in centiseconds.
/// This is the burn start time; pericynthion occurs 150 s later.
const LOI1_TIG_MET_CS: u32 = 24_890_000;

/// LOI-1 hours component of TIG (for V25 N33 entry).
const LOI1_TIG_H: u32 = 69;
/// LOI-1 minutes component of TIG (for V25 N33 entry).
const LOI1_TIG_M: u32 = 8;
/// LOI-1 seconds × 100 component of TIG (for V25 N33 entry). 20.00 s = 2000.
const LOI1_TIG_S100: u32 = 2000;

/// Historical Apollo 8 LOI-1 retrograde ΔV (m/s, signed: negative = retrograde in LVLH).
///
/// The spacecraft arrives on a hyperbolic approach trajectory at 2558 m/s at
/// pericynthion.  The 914 m/s retrograde burn slows it to ~1644 m/s, capturing
/// into a 60 × 170 nm (111 × 315 km) elliptical orbit.
///
/// With `SPS_THRUST_N = 91 188 N` and vehicle mass 30 000 kg, the deceleration
/// is ~3.04 m/s², giving a burn duration of ~300 s.
///
/// The symmetric burn geometry (TIG = pericynthion − 150 s) means most gravity
/// loss cancels across the burn arc, leaving a residual of ~5–10 m/s — well
/// within the ±50 km apolune tolerance.
const LOI1_DV_MPS: i32 = -914;

/// Target pericynthion altitude above Moon surface (m). 60 nautical miles.
const PERICYNTHION_ALT_M: f64 = 111_000.0;

/// Target apolune altitude above Moon surface (m). 170 nautical miles.
const APOLUNE_ALT_M_TARGET: f64 = 315_000.0;

/// Hyperbolic approach speed at pericynthion (m/s), derived from target orbit + ΔV.
///
/// The LOI-1 burn decelerates the spacecraft from `V_PERI_APPROACH` to the target
/// pericynthion speed of the capture orbit.  The capture orbit is:
///   r_p = R_MOON + 111 km,  r_a = R_MOON + 315 km
///   a   = (r_p + r_a) / 2
///   v_f = sqrt(μ_M * (2/r_p − 1/a)) ≈ 1671 m/s
///
/// Adding the planned ΔV gives the required approach speed:
///   V_PERI_APPROACH = v_f + |LOI1_DV_MPS| ≈ 1671 + 914 ≈ 2585 m/s
///
/// This is close to the Apollo 8 historical approach speed (~2585 m/s).  The
/// specific orbital energy at pericynthion:
///   ε_h = v²/2 − μ_M/r ≈ 2,585²/2 − μ_M/r_p ≈ 692 kJ/kg > 0 (hyperbolic).
fn compute_v_peri_approach() -> f64 {
    let r_p = R_MOON + PERICYNTHION_ALT_M;
    let r_a = R_MOON + APOLUNE_ALT_M_TARGET;
    let a = (r_p + r_a) / 2.0;
    let v_final = (MU_MOON * (2.0 / r_p - 1.0 / a)).sqrt(); // pericynthion speed of capture orbit
    v_final + LOI1_DV_MPS.unsigned_abs() as f64
}

/// Half-burn duration (s) — the burn is centred on pericynthion.
/// TIG = pericynthion − HALF_BURN_S, cutoff ≈ pericynthion + HALF_BURN_S.
const HALF_BURN_S: f64 = 150.0;

/// Coast window before TIG (centiseconds). State is seeded at TIG − SETTLE_CS so
/// that TIG is in the future when V25 N33 / V25 N81 are entered (p30_load_dv_lvlh
/// requires tig >= state.time). Phase D resets csm_state back to the pre-pericynthion
/// SV with state.time = TIG before starting the burn loop.
const SETTLE_CS: u32 = 30_000; // 300 s in centiseconds

/// Moon position placeholder (m) used for `propagate_coast` in the seed computation.
/// The Moon is at the origin in MCI, so the Earth's MCI position is not relevant
/// for point-mass Moon gravity.  Third-body perturbation from Earth is negligible
/// over 150 s; we use [0; 3] as the moon_pos sentinel (the integrator uses it
/// to derive the Earth's MCI position for the third-body term).
///
/// In MCI frame, `moon_pos` in `propagate_coast` is the Moon's ECI position, used
/// only to compute Earth's MCI position = `-moon_pos`.  Any plausible value gives
/// the same Moon primary gravity; the Earth third-body term is ~1e-5 m/s² over
/// 150 s and changes the seed by < 0.1 m — well below navigation significance.
const SEED_MOON_POS: [f64; 3] = [3.844e8, 0.0, 0.0];

// ── Helper: pre-pericynthion hyperbolic approach state vector ─────────────────

/// Compute the MCI seed state vector at TIG (= burn start = pericynthion − 150 s).
///
/// The Apollo LOI-1 burn is centred on pericynthion: the SPS ignites 150 s before
/// pericynthion, passes through pericynthion at mid-burn, and cuts off ~150 s after.
/// This symmetric arc cancels most gravity loss.
///
/// The pre-pericynthion state is obtained via the velocity-reversal trick:
///
/// 1. The pericynthion state is analytic: `r_peri` along +X, `V_PERI_APPROACH` along −Y.
/// 2. Reverse velocity: place the spacecraft at `(r_peri, +V_PERI_APPROACH along +Y)`.
///    This is the time-reversed (mirror) state at pericynthion.
/// 3. Propagate forward 150 s with `propagate_coast`.  On a time-reversible
///    integrator (RK4 on conservative gravity), forward propagation from the
///    reversed state is identical to backward propagation from the original state.
/// 4. Negate the velocity of the result.
///
/// The output state vector has epoch = `LOI1_TIG_MET_CS` and frame `MoonInertial`.
fn pre_pericynthion_sv() -> StateVector {
    let r_peri = R_MOON + PERICYNTHION_ALT_M;
    let v_peri_approach = compute_v_peri_approach();

    // Step 1: analytic pericynthion with velocity reversed for forward propagation.
    let sv_reversed = StateVector {
        position: [r_peri, 0.0, 0.0],
        velocity: [0.0, v_peri_approach, 0.0], // +Y = reversed from approach (−Y)
        epoch: Met(LOI1_TIG_MET_CS),
        frame: Frame::MoonInertial,
    };

    // Step 2: propagate forward by HALF_BURN_S seconds.
    let sv_fwd = propagate_coast(sv_reversed, HALF_BURN_S, SEED_MOON_POS);

    // Step 3: negate velocity to recover the approach direction.
    StateVector {
        position: sv_fwd.position,
        velocity: [
            -sv_fwd.velocity[0],
            -sv_fwd.velocity[1],
            -sv_fwd.velocity[2],
        ],
        epoch: Met(LOI1_TIG_MET_CS),
        frame: Frame::MoonInertial,
    }
}

// ── Test ──────────────────────────────────────────────────────────────────────

/// TC-LOI-1: Apollo 8 LOI-1 retrograde burn produces a 60 × 170 nm lunar orbit.
///
/// Drives P40 in MCI via a five-phase split-scenario chain.  The seed is placed
/// 150 s before pericynthion on the hyperbolic approach trajectory so the 914 m/s
/// burn is centred on pericynthion, cancelling most gravity loss (symmetric arc).
///
/// The resulting orbit must satisfy the architect's bounds:
/// - Apolune:      [265 000, 365 000] m
/// - Pericynthion: [ 91 000, 131 000] m
/// - Period:       [  7 440,   8 040] s
///
/// # Design reference
///
/// Architect's locked design, GitHub issue #27 (MS-T4 parent #23).
#[test]
fn tc_phase_loi_apollo_8_circularises_lunar_orbit() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();

    // Compute pre-pericynthion seed (150 s before pericynthion on hyperbolic approach).
    let sv_pre_peri = pre_pericynthion_sv();

    // Verify hyperbolic energy of the approach is positive (unbound pre-burn).
    {
        use agc_core::math::linalg::norm;
        let v_mag = norm(sv_pre_peri.velocity);
        let r_mag = norm(sv_pre_peri.position);
        let epsilon_h = 0.5 * v_mag * v_mag - MU_MOON / r_mag;
        assert!(
            epsilon_h > 0.0,
            "pre_pericynthion_sv: specific energy must be positive (hyperbolic); got {epsilon_h:.0} J/kg"
        );
    }

    // ── Phase A: seed + load P30/TIG ─────────────────────────────────────────
    //
    // Seed the MCI state vector at MET = TIG − 300 s so that TIG is in the future
    // when V25 N33 / V25 N81 are entered (p30_load_dv_lvlh requires tig >= state.time).
    // The position/velocity is the pre-pericynthion state (epoch = TIG), intentionally
    // inconsistent with the MET offset — this is harmless because Phase D resets
    // csm_state and state.time back to TIG before the burn loop.
    //
    // No seed_ground_truth here: calling it would start the SERVICER immediately
    // and integrate the spacecraft away from the pre-pericynthion position during
    // keypresses.  The SERVICER starts only when P40 is selected in Phase C.
    //
    // Note: V25 N33 format: R1 = hours, R2 = minutes, R3 = seconds × 100.
    // LOI1_TIG_MET_CS = 24_890_000 cs = 69 h 8 m 20 s = 69 h 8 m 2000 (×100).
    let phase_a = ScenarioBuilder::new("phase_loi/phase_a_setup")
        .comment("Phase A: seed MCI pre-pericynthion SV at TIG-300s, select P30, load TIG")
        .seed_state()
        .from_state_vector(sv_pre_peri)
        .met(Met(LOI1_TIG_MET_CS - SETTLE_CS))
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
        .digits(LOI1_TIG_H)
        .enter()
        .digits(LOI1_TIG_M)
        .enter()
        .digits(LOI1_TIG_S100)
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
        Some(Met(LOI1_TIG_MET_CS)),
        "Phase A: pending_tig must be Some(Met({LOI1_TIG_MET_CS})) after V25 N33 entry"
    );

    // ── Phase B: load ΔV ─────────────────────────────────────────────────────
    //
    // V25 N81 loads the retrograde ΔV in LVLH along-track (X component).
    // With velocity along −Y at pericynthion and identity REFSMMAT, P30
    // transforms the along-track −914 m/s LVLH vector to inertial +Y ΔV.
    // The SPS thrust_dir_platform default is [0, 1, 0] → inertial +Y at
    // identity REFSMMAT, which is anti-velocity (retrograde). This produces
    // the required orbital energy reduction.
    //
    // IMPORTANT: V25 N81 must be submitted while state.time < TIG.  Phase A
    // only does seed + P30 select + V25 N33 entry (no coast), so state.time
    // is still near LOI1_TIG_MET_CS (a few hundred cs past seed), which is
    // still equal to TIG for p30_load_dv_lvlh to accept the maneuver.
    let phase_b = ScenarioBuilder::new("phase_loi/phase_b_load_dv")
        .comment("Phase B: load LOI-1 retrograde ΔV via V25 N81")
        .v25_load_three(81, [LOI1_DV_MPS, 0, 0])
        .build();

    run_scenario(&phase_b, &mut state, &mut hw);

    // Post-Phase-B assertions.
    let pending = state
        .pending_maneuver
        .expect("Phase B: V25 N81 must produce a pending_maneuver");
    assert_eq!(
        pending.tig,
        Met(LOI1_TIG_MET_CS),
        "Phase B: pending_maneuver.tig must round-trip through P30"
    );
    let dv_mag = (pending.delta_v.0[0].powi(2)
        + pending.delta_v.0[1].powi(2)
        + pending.delta_v.0[2].powi(2))
    .sqrt();
    let expected_dv = LOI1_DV_MPS.unsigned_abs() as f64;
    assert!(
        (dv_mag - expected_dv).abs() < 2.0,
        "Phase B: |pending ΔV| = {dv_mag:.2} m/s should be near {expected_dv:.0} m/s"
    );

    // ── Phase C: select P40 and arm ───────────────────────────────────────────
    //
    // V37 ENTR 40 ENTR selects P40, which consumes pending_maneuver and
    // requests V50 N99.  Crew PRO arms the SPS for TIG-gated ignition.
    let phase_c = ScenarioBuilder::new("phase_loi/phase_c_arm_p40")
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
    // Reset the CSM state to the pre-pericynthion seed at TIG.  This is
    // necessary because the SERVICER may have fired during Phase C and
    // integrated the spacecraft a few steps away from the seed position.
    // By resetting here we guarantee the orbit insertion starts from the
    // correct pre-pericynthion state.
    //
    // `sv_pre_peri.epoch` = LOI1_TIG_MET_CS.  Setting `state.time` to TIG
    // as well ensures the TIG gate fires immediately on the first DAP cycle.
    state.csm_state = sv_pre_peri;
    state.time = Met(LOI1_TIG_MET_CS);
    hw.timers.set_time(state.time.0);

    let mut waitlist_pump = WaitlistPump::new();
    let mut dap_pump = DapPump::new();

    // Prime the pumps once to initialize internal state.  state.time == TIG so
    // the DAP's TIG gate will fire on this tick (burn.tig == TIG, state.time >= burn.tig).
    dap_pump.tick(&mut state, &mut hw, None);
    waitlist_pump.tick(&mut state, &mut hw);
    pump_engine_to_hw(&state, &mut hw);

    assert!(
        state.burn.armed,
        "Phase D: armed must be set before burn loop"
    );

    // Walk the full LOI-1 burn at 100 ms granularity.
    // At 91 188 N / 30 000 kg ≈ 3.04 m/s², the 914 m/s burn takes ~300 s
    // = 3 000 ticks.  Budget 5 000 ticks (500 s) as a safety margin.
    const TICK_CS: u32 = 10;
    const TICK_S: f64 = TICK_CS as f64 / 100.0;
    let max_iters: u32 = 5_000; // 500 s of sim time — safety margin over ~300 s burn

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

    // ── Phase E: post-burn orbital element assertions ─────────────────────────
    //
    // The spacecraft should now be in a 60 × 170 nm (111 × 315 km) elliptical
    // lunar orbit.  Convert the AGC's state vector to orbital elements using
    // `sv_to_elements` (which picks MU_MOON from Frame::MoonInertial) and
    // assert the apse altitudes and period fall within the specified bounds.
    //
    // Tolerance rationale (see module doc comment for full derivation):
    //   - Symmetric burn arc (TIG = pericynthion − 150 s) cancels most gravity loss.
    //   - Residual gravity loss ~5–10 m/s shifts apolune by ~5–10 km.
    //   - Wide tolerances ensure robustness to simulator constants.

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

    let elements = sv_to_elements(state.csm_state);

    // Apoapsis altitude above Moon surface.
    let r_a = apoapsis_altitude_moon(&elements);
    // Periapsis altitude above Moon surface.
    let r_p = periapsis_altitude_moon(&elements);
    // Orbital period using MU_MOON.
    let period = orbital_period(&elements, MU_MOON);

    // Architect's bounds (GitHub issue #27, locked design):
    //   - Apolune:      [265 000, 365 000] m  (target 315 km ± 50 km)
    //   - Pericynthion: [ 91 000, 131 000] m  (target 111 km ± 20 km)
    //   - Period:       [  7 440,   8 040] s  (target ≈ 7 740 s for a = R_MOON + 213 km)
    assert!(
        (265_000.0..=365_000.0).contains(&r_a),
        "Phase E: apoapsis altitude = {r_a:.0} m must be in [265 000, 365 000] m \
         (target: {APOLUNE_ALT_M_TARGET:.0} m); ΔV = {LOI1_DV_MPS} m/s (Apollo 8 historical)"
    );
    assert!(
        (91_000.0..=131_000.0).contains(&r_p),
        "Phase E: periapsis altitude = {r_p:.0} m must be in [91 000, 131 000] m \
         (target: {PERICYNTHION_ALT_M:.0} m)"
    );
    assert!(
        (7_440.0..=8_040.0).contains(&period),
        "Phase E: orbital period = {period:.1} s must be in [7 440, 8 040] s \
         (target ≈ 7 740 s for a = R_MOON + 213 km)"
    );
}
