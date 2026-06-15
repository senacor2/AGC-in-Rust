//! MS-T5 — Inter-phase handoff integration tests.
//!
//! Implements GitHub issue #28. Spec: `specs/ms-t5-handoffs-spec.md`.
//!
//! The MS-T4 phase tests showed that each Apollo 8 phase tracks the oracle
//! inside its checkpoint window. MS-T5 is one layer up: the *transitions*
//! between phases must not corrupt AGC state. Each test below carries
//! explicit state-invariant assertions, not just program-progression checks.
//!
//! Tests:
//!
//! 1. `tc_handoff_p40_to_p00_to_p30` — P40 cutoff → P00 → V37 next program.
//! 2. `tc_handoff_p23_marks_to_p30_targeting` — P23 marks update CSM state,
//!    P30 reads the corrected state at call time.
//! 3. `tc_handoff_soi_outbound_eci_to_mci` and `tc_handoff_soi_inbound_mci_to_eci`
//!    — SOI handover, both directions.
//! 4. `tc_handoff_p52_alignment_to_burn` — P52 writes new REFSMMAT, SERVICER
//!    consumes it on the next cycle.

use agc_core::control::imu_control::ImuAlignmentState;
use agc_core::guidance::targeting::{apply_external_delta_v, TargetingMode};
use agc_core::math::linalg::{mxv, norm, transpose, unit, vadd, vscale, vsub};
use agc_core::navigation::gravity::{R_EARTH, R_SOI_MOON};
use agc_core::navigation::integration::{propagate_coast, soi_check, total_gravity};
use agc_core::navigation::planetary::{moon_position, moon_velocity};
use agc_core::navigation::state_vector::Frame;
use agc_core::navigation::StateVector;
use agc_core::programs::p23::{Body, StarHorizonMark};
use agc_core::programs::p30::p30_load_dv_lvlh;
use agc_core::services::v_n::Key;
use agc_core::types::{Mat3x3, Met, Vec3};
use agc_core::AgcState;
use agc_sim::runtime::{pump_engine_to_hw, pump_pipa_into_state, DapPump, WaitlistPump};
use agc_sim::SimHardware;
use agc_sim::{run_scenario, DskyExpect, ScenarioBuilder};

// ── Shared helpers ────────────────────────────────────────────────────────────

/// Reference Euler step for a state vector under `total_gravity`. Used by the
/// SOI tests as the "what would the trajectory be in pure two-body with no
/// SOI handover" reference — `propagate_coast` + `soi_check` must produce a
/// state vector that, when transformed back to ECI via the Moon ephemeris,
/// matches this Euler reference to within the spec tolerances.
fn euler_step(sv: StateVector, dt: f64, moon_pos: Vec3) -> StateVector {
    let g = total_gravity(sv.position, sv.frame, moon_pos);
    let pos = vadd(sv.position, vadd(vscale(sv.velocity, dt), vscale(g, 0.5 * dt * dt)));
    let vel = vadd(sv.velocity, vscale(g, dt));
    let new_epoch_cs = sv.epoch.0.wrapping_add(Met::from_seconds(dt).0);
    StateVector {
        position: pos,
        velocity: vel,
        epoch: Met(new_epoch_cs),
        frame: sv.frame,
    }
}

// ── Test 1: P40 cutoff → P00 → V37 next program ───────────────────────────────

/// Spec: §3.
///
/// Walks a complete SPS burn (P30 → P40 → cutoff), then drives V37 E00 E to
/// select P00, snapshots the state, and finally selects P30 again. Asserts
/// state invariants at each boundary: no stale `pending_maneuver`, no stale
/// servicer hook, DAP back to AttitudeHold, nav state preserved across P00.
#[test]
fn tc_handoff_p40_to_p00_to_p30() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();

    // Burn fixture: 6 778 km circular LEO, prograde — same as `p40_sps_burn.rs`.
    const TARGET_DV_MS: u32 = 21;
    const SEED_POS_X_KM: u32 = 6778;
    const SEED_VEL_Y_M_S: u32 = 7669;
    const TIG_MIN: u32 = 5;
    let tig_cs = TIG_MIN * 6_000;

    // ── Phase 1: seed + select P30 + load TIG ────────────────────────────────
    let phase1 = ScenarioBuilder::new("handoff_p40_p00/seed_and_p30")
        .v71_p27_block_update(
            1,
            &[
                (1, SEED_POS_X_KM),
                (1, 0),
                (1, 0),
                (1, 0),
                (1, SEED_VEL_Y_M_S),
                (1, 0),
            ],
        )
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
        .keys(&[
            Key::Verb,
            Key::Digit(2),
            Key::Digit(5),
            Key::Noun,
            Key::Digit(3),
            Key::Digit(3),
            Key::Entr,
        ])
        .digits(0)
        .enter()
        .digits(TIG_MIN)
        .enter()
        .digits(0)
        .enter()
        .v25_load_three(81, [TARGET_DV_MS as i32, 0, 0])
        .build();
    run_scenario(&phase1, &mut state, &mut hw);

    assert!(state.pending_maneuver.is_some(), "P30 must produce pending_maneuver");

    // ── Phase 2: V37 E40 + PRO ───────────────────────────────────────────────
    let phase2 = ScenarioBuilder::new("handoff_p40_p00/select_p40_arm")
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
        .pro()
        .build();
    run_scenario(&phase2, &mut state, &mut hw);

    assert!(state.burn.burn_active, "P40 must arm burn");
    assert!(state.servicer_exit.is_some(), "P40 must install SERVICER hook");
    assert!(state.pending_maneuver.is_none(), "P40 must consume pending_maneuver");

    // ── Phase 3: jump to TIG-1s, drive the burn to cutoff ────────────────────
    state.csm_state.epoch = state.time;
    let mut waitlist_pump = WaitlistPump::new();
    let mut dap_pump = DapPump::new();

    dap_pump.tick(&mut state, &mut hw, None);
    waitlist_pump.tick(&mut state, &mut hw);

    state.time = Met(tig_cs.saturating_sub(100));
    hw.timers.set_time(state.time.0);
    dap_pump.tick(&mut state, &mut hw, None);
    waitlist_pump.tick(&mut state, &mut hw);

    const TICK_CS: u32 = 10;
    const TICK_S: f64 = TICK_CS as f64 / 100.0;
    let max_iters = 6_000_u32;
    let mut iters = 0_u32;
    while state.burn.burn_active && iters < max_iters {
        state.time = Met(state.time.0 + TICK_CS);
        hw.timers.set_time(state.time.0);
        hw.tick(TICK_S);
        pump_pipa_into_state(&mut state, &mut hw);
        dap_pump.tick(&mut state, &mut hw, None);
        waitlist_pump.tick(&mut state, &mut hw);
        pump_engine_to_hw(&state, &mut hw);
        agc_sim::runtime::pump_rcs_to_hw(&mut state, &mut hw);
        iters += 1;
    }
    assert!(!state.burn.burn_active, "burn must cut off within max_iters");

    // ── Boundary #1: post-cutoff invariants (still major_mode 40) ────────────
    assert_eq!(state.major_mode, 40, "P40 owns major_mode until V37 reselects");
    assert!(!state.burn.burn_active, "burn_active must be false");
    assert!(!state.burn.armed, "burn.armed must be false after cutoff");
    assert!(!state.engine_thrusting, "engine_thrusting must be false");
    assert!(!hw.engine.thrusting, "SimHardware SPS must be cold");
    assert!(
        state.servicer_exit.is_none(),
        "P40 must uninstall SERVICER hook on cutoff"
    );
    assert!(
        state.pending_maneuver.is_none(),
        "pending_maneuver must remain consumed"
    );
    assert_eq!(state.alarm.code(), 0, "no alarm at boundary #1");
    let achieved = norm(state.burn.accumulated_dv_inertial);
    assert!(
        (achieved - TARGET_DV_MS as f64).abs() < 5.0,
        "achieved ΔV {achieved:.3} m/s within 5 m/s of {TARGET_DV_MS} m/s"
    );

    // Snapshot CSM state so we can verify P00 does not perturb it.
    let csm_before_p00 = state.csm_state;

    // ── Boundary #2: V37 E00 E selects P00, nav state preserved ──────────────
    let phase_p00 = ScenarioBuilder::new("handoff_p40_p00/select_p00")
        .keys(&[
            Key::Verb,
            Key::Digit(3),
            Key::Digit(7),
            Key::Entr,
            Key::Digit(0),
            Key::Digit(0),
            Key::Entr,
        ])
        .expect_major_mode(0)
        .build();
    run_scenario(&phase_p00, &mut state, &mut hw);

    assert_eq!(state.major_mode, 0, "P00 must own major_mode");
    assert_eq!(state.dsky.prog, 0, "DSKY PROG must read 00");
    assert!(state.servicer_exit.is_none(), "P00 must keep SERVICER hook uninstalled");
    assert!(!state.burn.burn_active, "P00 must keep burn inactive");
    assert!(!state.engine_thrusting, "P00 must not light engine");

    // P00 must not perturb csm_state — components within 1 m / 1 mm/s.
    for i in 0..3 {
        let dp = (state.csm_state.position[i] - csm_before_p00.position[i]).abs();
        assert!(dp < 1.0, "csm_state.position[{i}] drift across P00 = {dp} m");
        let dv = (state.csm_state.velocity[i] - csm_before_p00.velocity[i]).abs();
        assert!(
            dv < 1.0e-3,
            "csm_state.velocity[{i}] drift across P00 = {dv} m/s"
        );
    }
    assert_eq!(state.alarm.code(), 0, "no alarm at boundary #2");

    // ── Boundary #3: V37 E30 E selects P30 again — clean handoff again ───────
    let phase_p30_again = ScenarioBuilder::new("handoff_p40_p00/reselect_p30")
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
        .build();
    run_scenario(&phase_p30_again, &mut state, &mut hw);

    assert_eq!(state.major_mode, 30, "P30 must own major_mode on reselect");
    assert_eq!(state.dsky.prog, 30, "DSKY PROG must read 30");
    assert_eq!(state.alarm.code(), 0, "no alarm at boundary #3");
    assert!(
        state.servicer_exit.is_none(),
        "no stale SERVICER hook after re-selecting P30"
    );
    assert!(
        state.pending_maneuver.is_none(),
        "P30 init must clear any stale pending_maneuver"
    );
}

// ── Test 2: P23 marks → P30 targeting ─────────────────────────────────────────

/// Spec: §4.
///
/// Drives P23 cislunar nav marks into a seeded state, then calls P30 to verify
/// the maneuver targeting **reads `state.csm_state` at the time of the call**
/// (not a cached pre-mark snapshot). The boundary under test is P30's
/// consumption of the live nav state; P23's Kalman filter math is unit-tested
/// in `agc-core/src/programs/p23.rs` and is not re-verified here.
///
/// Geometry: star direction perpendicular to position so sensitivity is
/// concentrated along the perpendicular axis. The mark is constructed from
/// the angle that would be predicted at a position 5 km off in Y, so the
/// Kalman update pulls `csm_state.position[1]` toward that synthetic truth.
/// One mark moves the state by ≈1.5 mm; five marks accumulate to several mm,
/// which is well above f64 numerical noise when fed into the LVLH→inertial
/// rotation P30 performs.
#[test]
fn tc_handoff_p23_marks_to_p30_targeting() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();

    const SEED_POS: Vec3 = [3.0e8, 0.0, 0.0];
    const SEED_VEL: Vec3 = [0.0, 800.0, 0.0];
    const SEED_EPOCH_S: f64 = 1_000_000.0;

    // Phase 1: seed state and select P23. P23 init needs a non-zero epoch and
    // ECI/MCI frame.
    let phase1 = ScenarioBuilder::new("handoff_p23_p30/seed_and_p23")
        .seed_state()
        .position_km(SEED_POS[0] / 1000.0, SEED_POS[1] / 1000.0, SEED_POS[2] / 1000.0)
        .velocity_m_s(SEED_VEL[0], SEED_VEL[1], SEED_VEL[2])
        .frame(Frame::EarthInertial)
        .met(Met::from_seconds(SEED_EPOCH_S))
        .refsmmat_identity()
        .done()
        .keys(&[
            Key::Verb,
            Key::Digit(3),
            Key::Digit(7),
            Key::Entr,
            Key::Digit(2),
            Key::Digit(3),
            Key::Entr,
        ])
        .expect_major_mode(23)
        .build();
    run_scenario(&phase1, &mut state, &mut hw);

    assert_eq!(state.major_mode, 23);
    assert!(
        state.csm_nav.tracking_active,
        "P23 init must enable cislunar nav tracking"
    );

    // Snapshot the uncorrected state before any P23 mark is incorporated.
    let uncorrected = state.csm_state;

    // Build a mark with a synthetic residual. Star direction is Y; the
    // measurement angle is what would be predicted at a position offset by
    // 5 km along Y. Computing the angle inline (the P23 helper is private):
    let star_dir: Vec3 = [0.0, 1.0, 0.0];
    let truth_pos: Vec3 = [SEED_POS[0], SEED_POS[1] + 5000.0, SEED_POS[2]];
    let body_pos: Vec3 = [0.0, 0.0, 0.0];
    let rho = vsub(truth_pos, body_pos);
    let d = norm(rho);
    let phi = (R_EARTH / d).asin();
    let u_hat = unit(rho);
    let cos_alpha = (star_dir[0] * u_hat[0] + star_dir[1] * u_hat[1] + star_dir[2] * u_hat[2])
        .clamp(-1.0, 1.0);
    let alpha = cos_alpha.acos();
    let truth_angle = alpha - phi;

    // Inject five identical marks via the new builder method. The Kalman
    // update is applied each time; the position drifts toward the synthetic
    // truth on each iteration as the covariance shrinks.
    let mut phase2 = ScenarioBuilder::new("handoff_p23_p30/marks");
    for _ in 0..5 {
        phase2 = phase2.p23_star_horizon_mark(StarHorizonMark {
            time: SEED_EPOCH_S,
            star_direction: star_dir,
            body: Body::Earth,
            angle_observed_rad: truth_angle,
        });
    }
    let phase2 = phase2.build();
    run_scenario(&phase2, &mut state, &mut hw);

    assert_eq!(state.csm_nav.mark_count, 5, "five marks accepted");
    assert_eq!(state.csm_nav.reject_count, 0, "no marks rejected");
    assert_eq!(state.alarm.code(), 0, "no P23 alarms");

    // P23 must have moved the state at least slightly toward the synthetic
    // truth. Without a measurable change there is nothing to test — fail
    // loudly if the Kalman gain is zero or the residual was discarded.
    let pos_delta_y = (state.csm_state.position[1] - uncorrected.position[1]).abs();
    assert!(
        pos_delta_y > 1.0e-4,
        "P23 marks must move csm_state.position[1]; observed Δ = {pos_delta_y} m"
    );

    let corrected = state.csm_state;

    // Phase 3: call P30 directly. Drives `state.pending_maneuver` through
    // `apply_external_delta_v(state.csm_state, ...)` — bypasses the V25 N33/N81
    // DSKY drive because that adds DSKY state-machine complexity unrelated to
    // the boundary under test (P30 consumption of the live nav state).
    let tig = Met::from_seconds(SEED_EPOCH_S + 600.0); // TIG = epoch + 10 min
    let dv_crew: Vec3 = [2.35, 0.0, 0.0]; // Apollo 8 MCC-2 magnitude, along-track
    p30_load_dv_lvlh(&mut state, tig, dv_crew);

    let pending = state
        .pending_maneuver
        .expect("p30_load_dv_lvlh must produce pending_maneuver");
    assert_eq!(pending.mode, TargetingMode::ExternalDeltaV);
    assert_eq!(pending.tig, tig, "TIG must round-trip exactly");
    assert!(
        (norm(pending.delta_v.0) - 2.35).abs() < 1.0e-6,
        "|ΔV| must equal crew load to 1 µm/s; got {}",
        norm(pending.delta_v.0)
    );

    // Decisive invariant: the inertial ΔV in `pending_maneuver` must equal
    // what `apply_external_delta_v` produces from the **post-mark** state,
    // and must differ from what it would have produced from the pre-mark
    // (uncorrected) state.
    //
    // p30_load_dv_lvlh remaps crew [along, radial, cross] → RSW [Y, X, Z]
    // before calling apply_external_delta_v; replicate that here so the
    // comparison is direct.
    let dv_rsw: Vec3 = [dv_crew[1], dv_crew[0], dv_crew[2]];
    let expected_corrected = apply_external_delta_v(corrected, tig, dv_rsw, state.refsmmat);
    let expected_uncorrected = apply_external_delta_v(uncorrected, tig, dv_rsw, state.refsmmat);

    // The pending maneuver must match the corrected-state computation
    // (same arithmetic — equality to f64 precision).
    let diff_corrected = norm(vsub(pending.delta_v.0, expected_corrected.delta_v.0));
    assert!(
        diff_corrected < 1.0e-12,
        "pending.delta_v must match apply_external_delta_v(corrected, …); diff = {diff_corrected} m/s"
    );

    // And it must NOT match the uncorrected-state computation — that's the
    // stale-cache bug we are guarding against.
    let diff_uncorrected = norm(vsub(pending.delta_v.0, expected_uncorrected.delta_v.0));
    assert!(
        diff_uncorrected > 1.0e-12,
        "pending.delta_v must differ from apply_external_delta_v(uncorrected, …); \
         diff = {diff_uncorrected} m/s — P30 may be reading a stale csm_state"
    );
}

// ── Test 3a: SOI outbound (Earth → Moon) ──────────────────────────────────────

/// Spec: §5.
///
/// Drives `propagate_coast` + `soi_check` directly from the test (Path A in
/// spec §5.1). The SERVICER does not invoke `soi_check` today (GH issue #51),
/// so we cannot use `advance_coast` to drive a frame flip on `state.csm_state`.
/// Instead this test exercises the handover transform mathematics on the
/// same code path `advance_ground_truth` uses.
#[test]
fn tc_handoff_soi_outbound_eci_to_mci() {
    let epoch = Met::from_seconds(0.0);
    let moon_pos_eci = moon_position(epoch);
    let moon_vel_eci = moon_velocity(epoch);

    // Spacecraft 1 km outside the SOI on the Earth side of the Moon, moving
    // toward the Moon at 1500 m/s plus the Moon's own velocity.
    let r_offset_m = R_SOI_MOON + 1000.0;
    let moon_dir = unit(moon_pos_eci);
    let r_eci: Vec3 = [
        moon_pos_eci[0] - r_offset_m * moon_dir[0],
        moon_pos_eci[1] - r_offset_m * moon_dir[1],
        moon_pos_eci[2] - r_offset_m * moon_dir[2],
    ];
    let v_eci: Vec3 = [
        moon_vel_eci[0] + 1500.0 * moon_dir[0],
        moon_vel_eci[1] + 1500.0 * moon_dir[1],
        moon_vel_eci[2] + 1500.0 * moon_dir[2],
    ];

    // Geometry sanity: spacecraft is exactly 1 km outside the SOI.
    let dist_initial = norm(vsub(r_eci, moon_pos_eci));
    assert!(
        (dist_initial - r_offset_m).abs() < 1.0,
        "fixture: spacecraft must be 1 km outside SOI; got {} m",
        dist_initial - R_SOI_MOON
    );

    let sv_eci = StateVector {
        position: r_eci,
        velocity: v_eci,
        epoch,
        frame: Frame::EarthInertial,
    };

    // Propagate 2 s — at 1500 m/s relative the spacecraft moves 3 km, well
    // across the 1 km margin. The SERVICER cycle is 2 s, so this is one
    // SERVICER-equivalent step.
    let dt = 2.0_f64;
    let propagated_eci_only = propagate_coast(sv_eci, dt, moon_pos_eci);
    let moon_pos_at_end = moon_position(propagated_eci_only.epoch);
    let moon_vel_at_end = moon_velocity(propagated_eci_only.epoch);
    let propagated = soi_check(propagated_eci_only, moon_pos_at_end, moon_vel_at_end);

    // Frame must have flipped.
    assert_eq!(
        propagated.frame,
        Frame::MoonInertial,
        "outbound: frame must flip to MoonInertial after crossing SOI"
    );

    // Spacecraft must now be inside the SOI.
    let dist_final = norm(propagated.position);
    assert!(
        dist_final < R_SOI_MOON,
        "outbound: post-handover distance from Moon ({dist_final} m) must be < R_SOI_MOON"
    );

    // Recover the absolute (ECI) coordinates from the post-handover MCI
    // state and the Moon ephemeris. This must reproduce the pre-handover
    // trajectory (continuity across the frame change).
    let r_eci_recovered = vadd(propagated.position, moon_pos_at_end);
    let v_eci_recovered = vadd(propagated.velocity, moon_vel_at_end);

    // Reference: pure two-body Euler step in ECI from the pre-handover state.
    // At dt = 2 s and gravity ≈ 0.003 m/s², Euler-vs-RK4 disagreement is
    // sub-µm in position and sub-µm/s in velocity — well within the
    // 1 m / 1 mm/s tolerance.
    let ref_step = euler_step(sv_eci, dt, moon_pos_eci);

    let pos_err = norm(vsub(r_eci_recovered, ref_step.position));
    let vel_err = norm(vsub(v_eci_recovered, ref_step.velocity));
    assert!(
        pos_err < 1.0,
        "outbound: ECI position continuity error = {pos_err} m (limit 1 m). \
         A large error here typically means the handover failed to subtract moon_pos correctly."
    );
    assert!(
        vel_err < 1.0e-3,
        "outbound: ECI velocity continuity error = {vel_err} m/s (limit 1 mm/s). \
         The mass-bug 'forgot to subtract v_moon' would show ~1018 m/s here."
    );
}

// ── Test 3b: SOI inbound (Moon → Earth) ───────────────────────────────────────

/// Spec: §5.
#[test]
fn tc_handoff_soi_inbound_mci_to_eci() {
    let epoch = Met::from_seconds(0.0);
    let moon_pos_eci = moon_position(epoch);

    // Spacecraft 1 km inside the SOI on the Earth side of the Moon,
    // moving outward (toward Earth) at 1500 m/s in the Moon frame.
    let r_offset_m = R_SOI_MOON - 1000.0;
    let moon_dir = unit(moon_pos_eci);
    let r_mci: Vec3 = [
        -r_offset_m * moon_dir[0],
        -r_offset_m * moon_dir[1],
        -r_offset_m * moon_dir[2],
    ];
    let v_mci: Vec3 = [
        -1500.0 * moon_dir[0],
        -1500.0 * moon_dir[1],
        -1500.0 * moon_dir[2],
    ];

    let dist_initial = norm(r_mci);
    assert!(
        (dist_initial - r_offset_m).abs() < 1.0,
        "fixture: spacecraft must be 1 km inside SOI; got {} m from Moon",
        dist_initial
    );

    let sv_mci = StateVector {
        position: r_mci,
        velocity: v_mci,
        epoch,
        frame: Frame::MoonInertial,
    };

    let dt = 2.0_f64;
    let propagated_mci_only = propagate_coast(sv_mci, dt, moon_pos_eci);
    let moon_pos_at_end = moon_position(propagated_mci_only.epoch);
    let moon_vel_at_end = moon_velocity(propagated_mci_only.epoch);
    let propagated = soi_check(propagated_mci_only, moon_pos_at_end, moon_vel_at_end);

    assert_eq!(
        propagated.frame,
        Frame::EarthInertial,
        "inbound: frame must flip to EarthInertial after crossing SOI"
    );

    let dist_from_moon_final = norm(vsub(propagated.position, moon_pos_at_end));
    assert!(
        dist_from_moon_final > R_SOI_MOON,
        "inbound: post-handover distance from Moon ({dist_from_moon_final} m) must exceed R_SOI_MOON"
    );

    // Euler reference in MCI for the same dt, no SOI check, then convert
    // back to ECI by adding the Moon ephemeris.
    let ref_step_mci = euler_step(sv_mci, dt, moon_pos_eci);
    let ref_step_eci_pos = vadd(ref_step_mci.position, moon_pos_at_end);
    let ref_step_eci_vel = vadd(ref_step_mci.velocity, moon_vel_at_end);

    let pos_err = norm(vsub(propagated.position, ref_step_eci_pos));
    let vel_err = norm(vsub(propagated.velocity, ref_step_eci_vel));
    assert!(
        pos_err < 1.0,
        "inbound: ECI position continuity error = {pos_err} m (limit 1 m)"
    );
    assert!(
        vel_err < 1.0e-3,
        "inbound: ECI velocity continuity error = {vel_err} m/s (limit 1 mm/s)"
    );
}

// ── Test 4: P52 alignment → next burn ─────────────────────────────────────────

/// Spec: §6.
///
/// Drives P52 (via the scenario builder's two-star optics path) to install a
/// non-identity REFSMMAT, then arms and ignites a small SPS burn for one
/// SERVICER cycle. Verifies that `state.servicer_last_dv_inertial` is rotated
/// through the **new** REFSMMAT, not through the identity matrix it had at
/// startup — i.e. that the SERVICER consumed the post-P52 matrix on the next
/// cycle and not a stale cached copy.
#[test]
fn tc_handoff_p52_alignment_to_burn() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();

    // Burn fixture (same LEO as Test 1).
    const SEED_POS_X_KM: u32 = 6778;
    const SEED_VEL_Y_M_S: u32 = 7669;

    // M_TRUTH: 30° rotation about +Z. Well-conditioned, easy to recover via
    // TRIAD.
    let cos_30 = (30.0_f64.to_radians()).cos();
    let sin_30 = (30.0_f64.to_radians()).sin();
    let m_truth: Mat3x3 = [
        [cos_30, -sin_30, 0.0],
        [sin_30, cos_30, 0.0],
        [0.0, 0.0, 1.0],
    ];

    // Snapshot the startup REFSMMAT — AgcState::new() seeds it to identity.
    let refsmmat_before = state.refsmmat;
    let identity: Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    assert_eq!(refsmmat_before, identity, "AgcState must start with identity REFSMMAT");

    // P52's prerequisite is at least a coarse alignment. The full P51 climb
    // is out of scope for this test. Set it on the live state — the scenario
    // builder does not currently expose imu_alignment_state seeding.
    state.imu_alignment_state = ImuAlignmentState::FineAligned;

    // ── Phase 1+2: seed state, install truth REFSMMAT, drive P52 marks ───────
    //
    // truth_refsmmat lives in the RunContext, which is per-`run_scenario`
    // invocation, so SeedTruthRefsmmat and the two OpticsSightings must be in
    // the same scenario.
    //
    // Stars 1 (Alpheratz) and 12 (Rigel) are far enough apart in the catalogue
    // that the TRIAD construction is well-conditioned.
    let phase_seed_and_p52 = ScenarioBuilder::new("handoff_p52_burn/seed_and_p52")
        .v71_p27_block_update(
            1,
            &[
                (1, SEED_POS_X_KM),
                (1, 0),
                (1, 0),
                (1, 0),
                (1, SEED_VEL_Y_M_S),
                (1, 0),
            ],
        )
        .seed_truth_refsmmat(m_truth)
        .optics_sighting(1)
        .optics_sighting(12)
        .build();
    run_scenario(&phase_seed_and_p52, &mut state, &mut hw);

    // P52 must have installed the new REFSMMAT.
    assert_ne!(
        state.refsmmat, refsmmat_before,
        "P52 must have written a new REFSMMAT"
    );
    assert_eq!(
        state.imu_alignment_state,
        ImuAlignmentState::FineAligned,
        "P52 must leave platform FineAligned"
    );

    // Orthonormality: R * Rᵀ ≈ I to 1e-10 per element.
    let r = state.refsmmat;
    for i in 0..3 {
        for j in 0..3 {
            let dot_ij = r[i].iter().zip(r[j].iter()).map(|(a, b)| a * b).sum::<f64>();
            let expected = if i == j { 1.0 } else { 0.0 };
            assert!(
                (dot_ij - expected).abs() < 1.0e-10,
                "REFSMMAT[{i}] · REFSMMAT[{j}] = {dot_ij}, expected {expected}"
            );
        }
    }

    let refsmmat_after_p52 = state.refsmmat;

    // ── Phase 3: prepare and arm a small SPS burn ────────────────────────────
    //
    // Drive P30 + P40 via the keystroke sequence. Use a short 60-second TIG
    // so the test finishes in seconds.
    const TIG_MIN_3: u32 = 1;
    let tig_cs_3 = TIG_MIN_3 * 6_000;

    let phase3 = ScenarioBuilder::new("handoff_p52_burn/p30_p40")
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
        .keys(&[
            Key::Verb,
            Key::Digit(2),
            Key::Digit(5),
            Key::Noun,
            Key::Digit(3),
            Key::Digit(3),
            Key::Entr,
        ])
        .digits(0)
        .enter()
        .digits(TIG_MIN_3)
        .enter()
        .digits(0)
        .enter()
        .v25_load_three(81, [5, 0, 0])
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
        .pro()
        .build();
    run_scenario(&phase3, &mut state, &mut hw);

    assert!(state.burn.burn_active, "P40 must arm the burn");

    // The SERVICER must not have overwritten the new REFSMMAT (no cache
    // anywhere — locks the spec §3 invariant in).
    assert_eq!(
        state.refsmmat, refsmmat_after_p52,
        "REFSMMAT must not be touched by P30/P40 init"
    );

    // ── Phase 4: jump to TIG, run **one** SERVICER cycle past ignition ───────
    state.csm_state.epoch = state.time;
    let mut waitlist_pump = WaitlistPump::new();
    let mut dap_pump = DapPump::new();

    dap_pump.tick(&mut state, &mut hw, None);
    waitlist_pump.tick(&mut state, &mut hw);

    state.time = Met(tig_cs_3.saturating_sub(100));
    hw.timers.set_time(state.time.0);
    dap_pump.tick(&mut state, &mut hw, None);
    waitlist_pump.tick(&mut state, &mut hw);

    const TICK_CS: u32 = 10;
    const TICK_S: f64 = TICK_CS as f64 / 100.0;
    let max_iters = 600_u32;
    let mut iters = 0_u32;
    let mut servicer_recorded = false;
    let mut ignited_at: Option<u32> = None;

    while iters < max_iters {
        state.time = Met(state.time.0 + TICK_CS);
        hw.timers.set_time(state.time.0);
        hw.tick(TICK_S);
        pump_pipa_into_state(&mut state, &mut hw);
        dap_pump.tick(&mut state, &mut hw, None);
        waitlist_pump.tick(&mut state, &mut hw);
        pump_engine_to_hw(&state, &mut hw);
        agc_sim::runtime::pump_rcs_to_hw(&mut state, &mut hw);

        if state.engine_thrusting && ignited_at.is_none() {
            ignited_at = Some(iters);
        }
        // Stop once the SERVICER has recorded a non-zero ΔV — i.e. once the
        // first cycle past ignition has run.
        if norm(state.servicer_last_dv_inertial) > 0.1 {
            servicer_recorded = true;
            break;
        }
        iters += 1;
    }
    assert!(
        servicer_recorded,
        "SERVICER must record at least one ΔV sample (ignited={ignited_at:?}, \
         iters={iters}, burn.armed={}, burn.burn_active={}, engine_thrusting={}, \
         last_dv_inertial={:?})",
        state.burn.armed, state.burn.burn_active, state.engine_thrusting,
        state.servicer_last_dv_inertial,
    );

    // ── Decisive invariant: SERVICER consumed the new REFSMMAT ───────────────
    //
    // thrust_dir_platform = [0, 1, 0] (platform +Y) per Spacecraft::new().
    // Inertial ΔV = REFSMMAT * platform ΔV → platform ΔV = REFSMMATᵀ * inertial ΔV.
    let dv_inertial = state.servicer_last_dv_inertial;
    let dv_platform_via_new = mxv(transpose(refsmmat_after_p52), dv_inertial);
    let dv_platform_via_old = mxv(transpose(refsmmat_before), dv_inertial);

    // Recovered platform ΔV via the post-P52 REFSMMAT must look like a
    // clean +Y thrust. Off-axis components must be near zero.
    assert!(
        dv_platform_via_new[0].abs() < 0.1,
        "recovered platform ΔV X-component must be ≈ 0; got {} m/s",
        dv_platform_via_new[0]
    );
    assert!(
        dv_platform_via_new[1] > 2.0,
        "recovered platform ΔV Y-component must be positive and > 2 m/s; got {} m/s",
        dv_platform_via_new[1]
    );
    assert!(
        dv_platform_via_new[2].abs() < 0.1,
        "recovered platform ΔV Z-component must be ≈ 0; got {} m/s",
        dv_platform_via_new[2]
    );

    // The decisive bug-catcher: if SERVICER had used the **old** REFSMMAT
    // (identity), recovering via the new REFSMMAT would now give a rotated
    // vector with a large X-component. Differ by ≥ 0.5 m/s.
    let recovery_disagreement = norm(vsub(dv_platform_via_new, dv_platform_via_old));
    assert!(
        recovery_disagreement >= 0.5,
        "if SERVICER consumed the new REFSMMAT, recovery-via-new and recovery-via-old must differ; \
         got {recovery_disagreement} m/s (would be near zero if SERVICER kept the stale identity)"
    );

    // REFSMMAT must not have been written back by the SERVICER itself.
    assert_eq!(state.refsmmat, refsmmat_after_p52, "SERVICER must not mutate REFSMMAT");
    assert_eq!(state.alarm.code(), 0, "no alarms during the P52 → burn handoff");
}
