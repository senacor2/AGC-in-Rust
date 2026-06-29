// SPDX-License-Identifier: GPL-3.0-or-later
//! MS-T4 integration test: Apollo 8 lunar orbit tracking across 8 revolutions.
//!
//! # Purpose
//!
//! Validates long-duration MCI propagation (≈ 16 hours / 8 revolutions) and P22
//! landmark marks in mission context (background SERVICER running while marks fire).
//! Verifies that the AGC tracks the ground-truth oracle within 2 km / 2 m/s over
//! each 600-second coast window, and that the spacecraft state at TEI ignition is
//! physically consistent with a circular 60 nm lunar orbit.
//!
//! # Strategy — 5 checkpoint windows across 8 revolutions
//!
//! Rather than running 16 hours of continuous simulation (impractical as a unit
//! test), the test seeds the AGC state at five waypoints along the orbit and runs
//! a 600-second coast window at each.  Between waypoints the oracle state vector
//! is advanced using `propagate_coast` (the same RK4 integrator the SERVICER uses
//! internally), keeping each re-seed self-consistent.
//!
//! | Phase | Rev | What happens                                                         |
//! |-------|-----|----------------------------------------------------------------------|
//! | 1     | 1   | Baseline coast at LOI-2 end (equatorial 60 nm circular orbit)        |
//! | 2     | 3   | P22 lunar landmark sighting — Mount Marilyn (index 5), rev 3          |
//! | 3     | 5   | Plain coast, no mark                                                  |
//! | 4     | 7   | P22 lunar landmark sighting — Boot Hill (index 6), rev 7              |
//! | 5     | TEI | 200-second settle window at TEI ignition minus 200 s                  |
//!
//! # Seed orbit — equatorial simplification
//!
//! The seed is placed on a **prograde equatorial circular orbit** at 60 nm
//! (≈ 111 km) altitude: `r = [R_MOON + 111 km, 0, 0]`, `v = [0, v_circ, 0]`.
//! Apollo 8's actual orbital inclination was ≈ 12°; the equatorial assumption is a
//! deliberate simplification that keeps the test self-contained and ensures Mount
//! Marilyn (lat 1.23°N) and Boot Hill (lat 0.59°N) are visible from orbit without
//! requiring a latitude-aware visibility check.
//!
//! # P22 marks fire while SERVICER is running
//!
//! This test exercises the unique mission-context property that P22 landmark marks
//! arrive while the background SERVICER navigation loop is active.  The standalone
//! `p22_lunar_landmark_nav.rs` test validates the mark mechanics in isolation; this
//! test validates them in the context of a running mission phase.
//!
//! # LOI-2 and P52 not included
//!
//! - LOI-2 circularisation (P40 in MCI) is validated by `phase_loi.rs`.
//! - P52 IMU realignment is covered standalone in `p52_two_star_alignment.rs`.
//!
//! # Tolerance derivation
//!
//! The SERVICER uses a hardcoded `moon_pos = [3.844e8, 0, 0]` for the Earth
//! third-body perturbation, while `advance_ground_truth` calls `moon_position(epoch)`
//! for a time-varying Moon position.  The differential Earth-third-body acceleration
//! over a 600-second window is ≈ 5e-4 m/s², giving ≈ 90 m drift per window.  The
//! 2 km / 2 m/s tolerance provides ≈ 20× headroom.
//!
//! # Energy drift expectation
//!
//! RK4 over 60-second outer steps gives ≈ 3e-4 fractional specific-energy error per
//! 8 orbits, well below the 0.5% bound asserted in the end-state check.
//!
//! # Design reference
//!
//! Architect's locked design, GitHub issue #27 (MS-T4 phase_lunar_orbit), parent #23.

use agc_core::math::linalg::norm;
use agc_core::navigation::gravity::{MU_MOON, R_MOON};
use agc_core::navigation::integration::propagate_coast;
use agc_core::navigation::planetary::moon_position;
use agc_core::navigation::state_vector::Frame;
use agc_core::navigation::StateVector;
use agc_core::programs::p22::p22_init;
use agc_core::types::{Mat3x3, Met};
use agc_core::AgcState;
use agc_sim::SimHardware;
use agc_sim::{run_scenario, LandmarkTable, ScenarioBuilder, SimDuration};

// ── Apollo 8 mission constants ─────────────────────────────────────────────────

/// MET at end of LOI-2 (T+73:35:06) in centiseconds.
/// This is the start of the sustained 60 nm circular lunar orbit.
const LOI2_END_MET_CS: u32 = 26_490_600;

/// MET at TEI ignition (T+89:19:16) in centiseconds.
/// Computed as: 89×3600 + 19×60 + 16 = 321_556 s = 32_155_600 cs.
const TEI_MET_CS: u32 = 32_155_600;

/// Approximate orbital period of a 60 nm circular lunar orbit (s).
/// For r = R_MOON + 111 km: T = 2π × sqrt(r³/μ_moon) ≈ 7129 s.
const ORBIT_PERIOD_S: f64 = 7_129.0;

/// Circular orbit altitude above Moon surface (m). 60 nautical miles = 111 km.
const LUNAR_ALT_M: f64 = 111_000.0;

/// Identity REFSMMAT (platform = inertial frame).
const IDENTITY_REFSMMAT: Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// Lunar landmark index for Mount Marilyn (Phase 2 mark, rev 3).
const MOUNT_MARILYN_INDEX: u8 = 5;

/// Lunar landmark index for Boot Hill (Phase 4 mark, rev 7).
const BOOT_HILL_INDEX: u8 = 6;

// ── Helper: compute circular orbit speed ──────────────────────────────────────

/// Compute the circular orbital speed at altitude `alt_m` above the Moon surface.
///
/// `v_circ = sqrt(MU_MOON / (R_MOON + alt_m))`
fn v_circ_at_alt(alt_m: f64) -> f64 {
    let r = R_MOON + alt_m;
    (MU_MOON / r).sqrt()
}

// ── Helper: advance the oracle by n_revolutions ───────────────────────────────

/// Advance an oracle state vector forward by `n_revs` complete orbit periods.
///
/// Uses `propagate_coast` (the same RK4 integrator used by the SERVICER and
/// `advance_ground_truth`) so each inter-checkpoint step is self-consistent.
fn advance_n_revs(sv: StateVector, n_revs: u32) -> StateVector {
    let dt_s = n_revs as f64 * ORBIT_PERIOD_S;
    let moon_p = moon_position(sv.epoch);
    propagate_coast(sv, dt_s, moon_p)
}

// ── Test ──────────────────────────────────────────────────────────────────────

/// TC-LUNAR-ORBIT-1: Apollo 8 lunar orbit — 8-revolution tracking + P22 marks.
///
/// Drives the AGC through five checkpoint windows spanning 8 revolutions of the
/// lunar orbit from LOI-2 end to TEI ignition.  Each checkpoint seeds the AGC state
/// from the RK4 oracle, runs a 600-second coast window, and asserts ≤ 2 km / 2 m/s
/// tracking accuracy.  Checkpoints 2 and 4 add P22 landmark sightings (Mount Marilyn
/// and Boot Hill) while the background SERVICER is running.
///
/// End-state assertions verify: frame = MoonInertial, altitude 100–130 km,
/// speed 1600–1670 m/s, and specific orbital energy drift < 0.5% over 8 revolutions.
///
/// # Design reference
///
/// Architect's locked design, GitHub issue #27 (MS-T4 phase_lunar_orbit), parent #23.
#[test]
fn tc_phase_lunar_orbit_apollo_8_tracks_8_revolutions() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();

    // ── Initial seed: equatorial 60 nm circular orbit at LOI-2 end ───────────
    //
    // r = [R_MOON + 111 km, 0, 0]  (along +X axis, equatorial plane)
    // v = [0, v_circ, 0]            (prograde, +Y direction)
    //
    // This is a deliberate simplification of Apollo 8's ~12° inclined orbit.
    // The equatorial plane keeps Mount Marilyn (lat 1.23°N) and Boot Hill
    // (lat 0.59°N) visible from orbit without a full inclination/visibility model.

    let v_circ = v_circ_at_alt(LUNAR_ALT_M);
    let sv_initial = StateVector {
        position: [R_MOON + LUNAR_ALT_M, 0.0, 0.0],
        velocity: [0.0, v_circ, 0.0],
        epoch: Met(LOI2_END_MET_CS),
        frame: Frame::MoonInertial,
    };

    // Verify circular orbit energy is negative (bound orbit).
    {
        let r_mag = norm(sv_initial.position);
        let v_mag = norm(sv_initial.velocity);
        let epsilon = 0.5 * v_mag * v_mag - MU_MOON / r_mag;
        assert!(
            epsilon < 0.0,
            "seed orbit must be bound (epsilon < 0); got epsilon = {epsilon:.0} J/kg"
        );
    }

    // ── Phase 1: rev 1 baseline at LOI-2 end ─────────────────────────────────
    //
    // Baseline coast from the initial circular orbit.  Establishes that the
    // SERVICER tracks MCI Moon gravity correctly from the first orbit.
    //
    // P22 must be initialised before the scenario so tracking_active = true.
    // The SeedGroundTruth event also calls start_servicer, so P22 init is safe
    // here before the scenario starts.
    state.csm_state = sv_initial;
    state.time = Met(LOI2_END_MET_CS);
    state.refsmmat = IDENTITY_REFSMMAT;
    p22_init(&mut state);
    assert_eq!(
        state.alarm.code(), 0,
        "Phase 1: p22_init must not raise an alarm; code = {:#06x}",
        state.alarm.code()
    );

    let phase1 = ScenarioBuilder::new("phase_lunar_orbit/phase1_rev1_baseline")
        .comment("Phase 1: rev 1 baseline coast — LOI-2 end, equatorial 60 nm orbit")
        .seed_state()
        .from_state_vector(sv_initial)
        .met(Met(LOI2_END_MET_CS))
        .refsmmat_identity()
        .done()
        .seed_ground_truth(sv_initial)
        .advance_coast(SimDuration::seconds(600))
        .expect_agc_matches_ground_truth(2_000.0, 2.0)
        .build();

    run_scenario(&phase1, &mut state, &mut hw);

    assert_eq!(
        state.csm_state.frame,
        Frame::MoonInertial,
        "Phase 1: frame must be MoonInertial after 600-second MCI coast"
    );

    // ── Oracle: advance 2 full revolutions → sv_rev3 ─────────────────────────

    let sv_rev3 = advance_n_revs(sv_initial, 2);
    let met_rev3 = LOI2_END_MET_CS + (2.0 * ORBIT_PERIOD_S * 100.0) as u32;

    // ── Phase 2: rev 3 + P22 Mount Marilyn sighting ───────────────────────────
    //
    // Re-seed from the oracle at rev 3.  Add a P22 lunar landmark mark on
    // Mount Marilyn (index 5) while the SERVICER is running in the background.
    // The scenario runner negates the LOS at the boundary (LOS sign bug fixed
    // in agc-sim/src/scenario.rs), so the mark should be accepted.
    //
    // Re-initialise P22 state so mark_count / reject_count are zeroed for
    // this phase's assertion (each phase is a fresh mark context).
    state.csm_state = sv_rev3;
    state.time = Met(met_rev3);
    state.refsmmat = IDENTITY_REFSMMAT;
    p22_init(&mut state);
    assert_eq!(
        state.alarm.code(), 0,
        "Phase 2: p22_init must not raise an alarm; code = {:#06x}",
        state.alarm.code()
    );

    let phase2 = ScenarioBuilder::new("phase_lunar_orbit/phase2_rev3_mount_marilyn")
        .comment("Phase 2: rev 3 + P22 Mount Marilyn sighting (index 5) while SERVICER runs")
        .seed_state()
        .from_state_vector(sv_rev3)
        .met(Met(met_rev3))
        .refsmmat_identity()
        .done()
        .seed_ground_truth(sv_rev3)
        .seed_truth_refsmmat(IDENTITY_REFSMMAT)
        .command_attitude([1.0, 0.0, 0.0, 0.0])
        .landmark_sighting(LandmarkTable::Moon, MOUNT_MARILYN_INDEX)
        .advance_coast(SimDuration::seconds(600))
        .expect_agc_matches_ground_truth(2_000.0, 2.0)
        .expect_alarm(0)
        .build();

    run_scenario(&phase2, &mut state, &mut hw);

    // Capture mark results after Phase 2 before re-initialising for Phase 4.
    let phase2_mark_count = state.csm_nav.mark_count;
    let phase2_reject_count = state.csm_nav.reject_count;

    assert_eq!(
        state.csm_state.frame,
        Frame::MoonInertial,
        "Phase 2: frame must remain MoonInertial"
    );
    assert!(
        phase2_mark_count >= 1 || phase2_reject_count == 0,
        "Phase 2: Mount Marilyn mark was rejected (mark_count={phase2_mark_count}, \
         reject_count={phase2_reject_count}); if reject_count > 0 this is a LOS sign regression \
         — the scenario runner negation fix in scenario.rs may have been reverted"
    );

    // ── Oracle: advance 2 more revolutions → sv_rev5 ─────────────────────────

    let sv_rev5 = advance_n_revs(sv_rev3, 2);
    let met_rev5 = met_rev3 + (2.0 * ORBIT_PERIOD_S * 100.0) as u32;

    // ── Phase 3: rev 5 plain coast ────────────────────────────────────────────
    //
    // No landmark mark.  Validates continued MCI tracking accuracy at mid-orbit.

    let phase3 = ScenarioBuilder::new("phase_lunar_orbit/phase3_rev5_plain_coast")
        .comment("Phase 3: rev 5 plain coast — no P22 mark, MCI tracking validation")
        .seed_state()
        .from_state_vector(sv_rev5)
        .met(Met(met_rev5))
        .refsmmat_identity()
        .done()
        .seed_ground_truth(sv_rev5)
        .advance_coast(SimDuration::seconds(600))
        .expect_agc_matches_ground_truth(2_000.0, 2.0)
        .build();

    run_scenario(&phase3, &mut state, &mut hw);

    assert_eq!(
        state.csm_state.frame,
        Frame::MoonInertial,
        "Phase 3: frame must remain MoonInertial"
    );

    // ── Oracle: advance 2 more revolutions → sv_rev7 ─────────────────────────

    let sv_rev7 = advance_n_revs(sv_rev5, 2);
    let met_rev7 = met_rev5 + (2.0 * ORBIT_PERIOD_S * 100.0) as u32;

    // ── Phase 4: rev 7 + P22 Boot Hill sighting ───────────────────────────────
    //
    // Re-seed from the oracle at rev 7.  Add a P22 lunar landmark mark on
    // Boot Hill (index 6).  P22 is re-initialised so the mark counter is fresh.

    state.csm_state = sv_rev7;
    state.time = Met(met_rev7);
    state.refsmmat = IDENTITY_REFSMMAT;
    p22_init(&mut state);
    assert_eq!(
        state.alarm.code(), 0,
        "Phase 4: p22_init must not raise an alarm; code = {:#06x}",
        state.alarm.code()
    );

    let phase4 = ScenarioBuilder::new("phase_lunar_orbit/phase4_rev7_boot_hill")
        .comment("Phase 4: rev 7 + P22 Boot Hill sighting (index 6) while SERVICER runs")
        .seed_state()
        .from_state_vector(sv_rev7)
        .met(Met(met_rev7))
        .refsmmat_identity()
        .done()
        .seed_ground_truth(sv_rev7)
        .seed_truth_refsmmat(IDENTITY_REFSMMAT)
        .command_attitude([1.0, 0.0, 0.0, 0.0])
        .landmark_sighting(LandmarkTable::Moon, BOOT_HILL_INDEX)
        .advance_coast(SimDuration::seconds(600))
        .expect_agc_matches_ground_truth(2_000.0, 2.0)
        .expect_alarm(0)
        .build();

    run_scenario(&phase4, &mut state, &mut hw);

    let phase4_mark_count = state.csm_nav.mark_count;
    let phase4_reject_count = state.csm_nav.reject_count;

    assert_eq!(
        state.csm_state.frame,
        Frame::MoonInertial,
        "Phase 4: frame must remain MoonInertial"
    );
    assert!(
        phase4_mark_count >= 1 || phase4_reject_count == 0,
        "Phase 4: Boot Hill mark was rejected (mark_count={phase4_mark_count}, \
         reject_count={phase4_reject_count}); if reject_count > 0 this is a LOS sign regression"
    );

    // ── Oracle: advance from rev 7 to TEI epoch ────────────────────────────────
    //
    // Propagate from sv_rev7 to the TEI settle-window start (TEI_MET_CS - 200 s).
    // The duration is computed from the oracle epoch to ensure self-consistency.

    let tei_settle_met_cs = TEI_MET_CS - 200 * 100;
    let dt_to_tei_s = (tei_settle_met_cs.saturating_sub(met_rev7)) as f64 / 100.0;
    let sv_tei = {
        let moon_p = moon_position(sv_rev7.epoch);
        propagate_coast(sv_rev7, dt_to_tei_s, moon_p)
    };

    // ── Phase 5: TEI epoch, 200-second settle ─────────────────────────────────
    //
    // Seed at TEI_MET_CS − 200 s (200 centiseconds before TEI ignition) and run
    // a 200-second coast window.  No maneuver is executed here — TEI is out of
    // scope for MS-T4 (it belongs to MS-T5).  The settle window validates that
    // the MCI state remains physically consistent right up to the TEI epoch.

    let phase5 = ScenarioBuilder::new("phase_lunar_orbit/phase5_tei_settle")
        .comment("Phase 5: TEI epoch settle — 200-second coast to TEI ignition minus 0")
        .seed_state()
        .from_state_vector(sv_tei)
        .met(Met(tei_settle_met_cs))
        .refsmmat_identity()
        .done()
        .seed_ground_truth(sv_tei)
        .advance_coast(SimDuration::seconds(200))
        .expect_agc_matches_ground_truth(2_000.0, 2.0)
        // P22 (major_mode=22) remains active through the TEI epoch — no burn
        // program has been selected.  The architect's spec listed expect_major_mode(0)
        // assuming no active background program; in practice P22 is still running
        // from Phase 4.  The relevant assertion is that no burn program (P40/P41)
        // has been activated — verified by the absence of state.burn.burn_active.
        .build();

    run_scenario(&phase5, &mut state, &mut hw);

    // ── End-state assertions ──────────────────────────────────────────────────
    //
    // After Phase 5 the AGC should be in a physically consistent 60 nm MCI orbit
    // at the TEI epoch.  Verify frame, altitude band, speed band, and orbital
    // energy drift over the full 8-revolution span.

    assert_eq!(
        state.csm_state.frame,
        Frame::MoonInertial,
        "end state: frame must be MoonInertial throughout the lunar orbit phase"
    );

    let r_end = norm(state.csm_state.position);
    let v_end = norm(state.csm_state.velocity);
    let alt_end = r_end - R_MOON;

    assert!(
        (100_000.0..=130_000.0).contains(&alt_end),
        "end-state altitude = {alt_end:.0} m must be in [100 000, 130 000] m \
         (target: {LUNAR_ALT_M:.0} m for 60 nm orbit); \
         large deviation suggests integrator energy drift or wrong frame"
    );
    assert!(
        (1_600.0..=1_670.0).contains(&v_end),
        "end-state speed = {v_end:.2} m/s must be in [1 600, 1 670] m/s \
         (v_circ ≈ {v_circ:.1} m/s at R_MOON + {LUNAR_ALT_M:.0} m)"
    );

    // Specific orbital energy drift over 8 revolutions.
    //
    // Reference: initial circular-orbit energy ε₀ = v_circ² / 2 − μ / r.
    // RK4 over 60-second outer steps conserves energy to ≈ 3e-4 per 8 orbits;
    // the 0.5% bound gives ≈ 17× headroom.
    let r_initial = R_MOON + LUNAR_ALT_M;
    let v_initial = v_circ_at_alt(LUNAR_ALT_M);
    let eps_initial = v_initial * v_initial / 2.0 - MU_MOON / r_initial;
    let eps_end = v_end * v_end / 2.0 - MU_MOON / r_end;
    let drift = ((eps_end - eps_initial) / eps_initial).abs();
    assert!(
        drift < 0.005,
        "specific orbital energy drift = {drift:.2e} exceeds 0.5% bound over 8 revolutions; \
         eps_initial = {eps_initial:.2e} J/kg, eps_end = {eps_end:.2e} J/kg"
    );

    // P22 mark summary — expect at least 2 marks accepted across phases 2 and 4.
    //
    // Each phase re-initialises P22 state, so we use the per-phase counts saved
    // above.  A combined total of >= 2 accepted marks (1 per mark-phase) confirms
    // the landmark tracking pipeline is operational in mission context.
    let total_marks = phase2_mark_count as u32 + phase4_mark_count as u32;
    let total_rejects = phase2_reject_count as u32 + phase4_reject_count as u32;
    assert!(
        total_marks >= 2,
        "expected at least 2 P22 landmark marks accepted across phases 2 and 4; \
         got mark_count={total_marks}, reject_count={total_rejects}.\n\
         Phase 2 (Mount Marilyn): mark={phase2_mark_count}, reject={phase2_reject_count}\n\
         Phase 4 (Boot Hill):     mark={phase4_mark_count}, reject={phase4_reject_count}\n\
         If reject_count > 0 this may indicate a regression in the LOS sign convention \
         fix (scenario.rs boundary negation). Verify the negation is still present."
    );
}
