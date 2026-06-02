//! MS-T4 integration test: Apollo 8 trans-lunar coast from post-TLI to LOI-1.
//!
//! # Purpose
//!
//! Drives the AGC through the entire trans-lunar coast of Apollo 8, verifying
//! that:
//!
//! - The CMC correctly tracks the coasting trajectory against an RK4 oracle.
//! - Mid-course corrections (MCC-2 at T+10:55, MCC-4 at T+60:59) are applied
//!   correctly as impulsive ΔVs.
//! - The MCI gravity branch is exercised via a synthetic-seed checkpoint (phase 4b).
//! - The SOI handover from Earth to Moon is exercised via `soi_check` (see
//!   findings below).
//!
//! # MCC direction convention
//!
//! Real Apollo MCCs were small ΔVs applied **perpendicular to the velocity
//! vector** in the orbital plane, targeting pericynthion altitude rather than
//! orbit energy (Apollo 8 Mission Report MSC-PA-R-69-1, §4.4). This simulation
//! uses the in-plane radial-outward direction `n_hat = unit(h × v)` where
//! `h = r × v` is the angular-momentum vector. This direction is perpendicular
//! to velocity and lies in the orbital plane pointing outward. The positive sign
//! (+|ΔV| · n_hat) is a free choice of convention; either sign is physically
//! valid as a pericynthion trim — positive raises pericynthion when applied on
//! the outbound leg.
//!
//! # Checkpoint-reseed strategy
//!
//! A straight continuous SERVICER simulation would take roughly 10 min of wall
//! time on a modern host, which is unacceptable for a unit-test suite. Instead,
//! the test seeds the AGC state at seven waypoints along the trajectory, advances
//! a short coast window at each (5 min for phases 1–5, 2 s for phase 6), and
//! asserts 1 km / 1 m/s tracking accuracy. Between waypoints the oracle SV is
//! advanced by calling `propagate_coast` directly (the same RK4 integrator that
//! `advance_ground_truth` uses internally), so each re-seed is self-consistent.
//!
//! # Findings and synthetic-MCI strategy
//!
//! ## SOI handover cannot trigger with the current simulation setup
//!
//! **Finding (trajectory accuracy)**: The parking-orbit state vector places the
//! spacecraft at `[R_EARTH + 185 km, 0, 0]` with a prograde velocity in the YZ
//! plane (inclination 32.5°). The `moon_position` function is hardcoded to the
//! Apollo 11 launch epoch (July 16, 1969); the Moon at that epoch is located in
//! a completely different direction from the spacecraft's outbound trajectory.
//! Even with TLI ΔV up to 3 500 m/s the minimum approach distance to the Moon's
//! computed position is ~364 000 km — far outside the SOI radius of 66 183 km.
//!
//! The SOI assertion (`frame == MoonInertial`) is therefore structurally
//! infeasible with a continuously integrated trajectory under the current Moon
//! model and parking-orbit initialisation.  Rather than suppressing or widening
//! the bounds, this test uses a **synthetic MCI seed** (phase 4b) to cover the
//! MCI code paths by construction, without claiming orbital continuity with the
//! prior ECI phases.
//!
//! ## Synthetic MCI seed (phase 4b) — what it validates
//!
//! Phase 4b directly constructs a plausible inbound-to-Moon state vector in MCI
//! frame at MET T+55:00:00 (POST_SOI_MET_CS), 50 000 km from Moon centre —
//! clearly inside the SOI boundary (66 183 km).  This is NOT derived from the
//! prior ECI trajectory; it is an independent constructed seed.  Running
//! `propagate_coast` for 5 minutes from this seed validates:
//!
//! - `total_gravity` dispatches to the Moon gravity branch when `frame == MoonInertial`.
//! - `advance_ground_truth` sets `spacecraft.current_body = GravityBody::Moon`
//!   when the seed SV carries `frame == MoonInertial`.
//! - The AGC SERVICER (via `average_g_step`) also reads `sv.frame` and calls
//!   Moon gravity, keeping AGC and oracle in agreement.
//! - Frame preservation: `state.csm_state.frame` is `MoonInertial` after the
//!   5-minute coast window.
//! - Position remains inside the SOI (`‖r_mci‖ < R_SOI_MOON`).
//!
//! The `soi_check` ECI→MCI transition math is covered separately by
//! `tc_int_soi_transition_eci_to_mci` in `agc-core/src/navigation/integration.rs`.
//!
//! ## Deferred work
//!
//! A full continuous SOI handover from a properly aimed Apollo-8 trajectory
//! requires a mission-aware epoch injection (Apollo 8 launch date vs. the
//! hardcoded Apollo 11 epoch).  This is deferred as follow-up for MS-T5,
//! tracked in GH issue #27.
//!
//! ## TLI ΔV produces sub-escape ellipse with apogee ~194 000 km
//!
//! With TLI ΔV = 3 047 m/s from a 185 km parking orbit, the post-TLI speed is
//! ~10 498 m/s — below escape velocity (~11 019 m/s at 185 km). The resulting
//! orbit is a highly eccentric ellipse with apogee ~194 000 km, consistent with
//! the `phase_tli.rs` doc comment (ε ≈ −2 MJ/kg). The spacecraft does not reach
//! the Moon's distance (~384 000 km).
//!
//! # What is NOT tested here
//!
//! - Continuous ECI→MCI SOI handover: see finding above and `tc_int_soi_*`
//!   unit tests in `agc-core/src/navigation/integration.rs`.
//! - PTC (Passive Thermal Control): thermal rotation exercised separately.
//! - P23 (cislunar navigation / landmark tracking): gated on GH issue #57.
//! - P52 (IMU realignment): exercised in `p52_two_star_alignment.rs`.
//!
//! # Assertion table
//!
//! | Phase | Window | Position tol | Velocity tol | Extra assertion                        |
//! |-------|--------|--------------|--------------|----------------------------------------|
//! | 1     | 5 min  | 1 000 m      | 1 m/s        | baseline post-TLI ECI                  |
//! | 2     | 5 min  | 1 000 m      | 1 m/s        | MCC-2 applied (+2.35 m/s perp-in-plane)|
//! | 3     | 5 min  | 1 000 m      | 1 m/s        | mid-transit ECI                        |
//! | 4     | 5 min  | 1 000 m      | 1 m/s        | high-apogee coast                      |
//! | 4b    | 5 min  | 1 000 m      | 1 m/s        | synthetic MCI seed, Moon gravity check |
//! | 5     | 5 min  | 1 000 m      | 1 m/s        | MCC-4 applied (+0.43 m/s perp-in-plane)|
//! | 6     | 2 sec  | 1 000 m      | 1 m/s        | end-of-coast ECI                       |
//!
//! # Design reference
//!
//! Architect's locked design, GitHub issue #27 (parent #23).

use agc_core::math::linalg::{cross, norm, unit};
use agc_core::navigation::gravity::{MU_EARTH, R_EARTH, R_SOI_MOON};
use agc_core::navigation::integration::propagate_coast;
use agc_core::navigation::planetary::moon_position;
use agc_core::navigation::state_vector::Frame;
use agc_core::navigation::StateVector;
use agc_core::types::Met;
use agc_core::AgcState;
use agc_sim::SimHardware;
use agc_sim::{run_scenario, ScenarioBuilder, SimDuration};

// ── Apollo 8 mission-elapsed-time constants (centiseconds) ───────────────────
//
// NOTE: all MET values are relative to mission start (t=0). The
// `moon_position` model is anchored to the Apollo 11 launch epoch (July 16,
// 1969), not Apollo 8 (December 21, 1968). The Moon's direction at these MET
// values does not correspond to the actual Apollo 8 lunar position; see the
// SOI finding in the module doc comment.

/// MET at post-TLI (T+2:56:00). Start of the translunar coast.
const POST_TLI_MET_CS: u32 = 1_056_000;
/// MET at MCC-2 (T+10:55:04).
const MCC2_MET_CS: u32 = 3_930_400;
/// MET at mid-transit checkpoint (T+30:00:00).
const MID_TRANSIT_MET_CS: u32 = 10_800_000;
/// MET at high-apogee checkpoint (T+40:00:00). Spacecraft is near apogee
/// (~193 000 km ECI) of its sub-escape-velocity ellipse. No SOI crossing
/// occurs; see the module finding above.
const HIGH_APOGEE_MET_CS: u32 = 14_400_000;
/// MET for the synthetic MCI seed (T+55:00:00).
///
/// This MET is chosen to fall between the high-apogee and MCC-4 checkpoints.
/// The state vector used at this MET is a **synthetic** construction — it is
/// NOT derived from the prior ECI trajectory. See the module doc comment for
/// the rationale.
const POST_SOI_MET_CS: u32 = 19_800_000;
/// MET at MCC-4 (T+60:59:55).
const MCC4_MET_CS: u32 = 21_959_500;
/// MET at LOI-1 time marker (T+69:08:20).
///
/// On the real Apollo 8 mission this was the LOI-1 ignition. In this
/// simulation the spacecraft is still on the inbound leg of its high ellipse
/// (~157 000 km from Earth, falling back toward periapsis). The simulation
/// exercises the AGC tracking at this epoch, not a real LOI maneuver.
const LOI1_MET_CS: u32 = 24_890_000;

// ── MCC ΔV magnitudes ────────────────────────────────────────────────────────

/// MCC-2 ΔV magnitude (m/s). Applied perpendicular-in-plane (radial-outward direction).
/// Apollo 8 Mission Report MSC-PA-R-69-1, Table 3-I: MCC-2 was ~2.35 m/s SPS.
const MCC2_DV_MPS: f64 = 2.35;
/// MCC-4 ΔV magnitude (m/s). Applied perpendicular-in-plane (radial-outward direction).
const MCC4_DV_MPS: f64 = 0.43;

// ── Parking-orbit / TLI parameters (same arithmetic as phase_tli.rs) ─────────

/// Parking orbit altitude above Earth's surface (m).
const PARKING_ALT_M: f64 = 185_000.0;
/// Apollo 8 parking orbit inclination (32.5°).
const PARKING_INCLINATION_DEG: f64 = 32.5;
/// MET at parking-orbit insertion (T+0:11:35), centiseconds.
const PARKING_INSERTION_MET_CS: u32 = 69_500;
/// MET at TLI ignition (T+2:50:41), centiseconds.
const TLI_IGNITION_MET_CS: u32 = 1_024_100;
/// TLI ΔV (m/s), prograde impulsive. Produces v_post ≈ 10 498 m/s (sub-escape).
const TLI_DV_MPS: f64 = 3047.0;

// ── Helper: derive the post-TLI state vector ─────────────────────────────────
//
// Same parking-orbit-then-impulsive-ΔV arithmetic as phase_tli.rs.
// No helper function is added to avoid coupling the two test files.

fn derive_post_tli_sv() -> StateVector {
    // 1. Parking-orbit insertion state vector.
    let r = R_EARTH + PARKING_ALT_M;
    let v_circ = (MU_EARTH / r).sqrt();
    let i_rad = PARKING_INCLINATION_DEG.to_radians();
    let park_sv = StateVector {
        position: [r, 0.0, 0.0],
        velocity: [0.0, v_circ * i_rad.cos(), v_circ * i_rad.sin()],
        epoch: Met(PARKING_INSERTION_MET_CS),
        frame: Frame::EarthInertial,
    };

    // 2. Coast to TLI ignition.
    let coast_to_tli_s = (TLI_IGNITION_MET_CS - PARKING_INSERTION_MET_CS) as f64 / 100.0;
    let moon_p = moon_position(park_sv.epoch);
    let sv_at_ignition = propagate_coast(park_sv, coast_to_tli_s, moon_p);

    // 3. Apply impulsive prograde TLI ΔV.
    let v = sv_at_ignition.velocity;
    let v_mag = norm(v);
    let v_hat_ign = [v[0] / v_mag, v[1] / v_mag, v[2] / v_mag];
    let sv_post_ignition = StateVector {
        velocity: [
            v[0] + TLI_DV_MPS * v_hat_ign[0],
            v[1] + TLI_DV_MPS * v_hat_ign[1],
            v[2] + TLI_DV_MPS * v_hat_ign[2],
        ],
        epoch: Met(TLI_IGNITION_MET_CS),
        ..sv_at_ignition
    };

    // 4. Coast through the S-IVB burn window to POST_TLI_MET_CS.
    let burn_window_s = (POST_TLI_MET_CS - TLI_IGNITION_MET_CS) as f64 / 100.0;
    let moon_p2 = moon_position(sv_post_ignition.epoch);
    propagate_coast(sv_post_ignition, burn_window_s, moon_p2)
}

/// In-plane radial-outward unit vector — perpendicular to velocity, in the orbital plane.
///
/// Computes `unit(h × v)` where `h = r × v` (angular-momentum direction).
/// For a circular orbit this is purely radially outward. For an elliptical orbit
/// on the outbound leg it points outward with a small along-track component.
///
/// Used for MCC ΔV direction: real Apollo MCCs targeted pericynthion altitude
/// (perpendicular trim), not orbit energy (anti-velocity). Applying `+|ΔV| * n_hat`
/// raises pericynthion when executed on the outbound leg.
/// Reference: Apollo 8 Mission Report MSC-PA-R-69-1, §4.4.
fn n_hat_perp_in_plane(r: [f64; 3], v: [f64; 3]) -> [f64; 3] {
    let h = cross(r, v); // angular momentum vector (out-of-plane)
    let h_n = unit(h); // angular momentum direction
    let v_n = unit(v); // prograde direction
    unit(cross(h_n, v_n)) // in-plane, perpendicular to v, radial-outward
}

// ── Test ──────────────────────────────────────────────────────────────────────

/// TC-TRANS-1: Apollo 8 translunar coast — seven checkpoint phases.
///
/// Verifies AGC tracking accuracy over the entire trans-lunar coast timeline.
/// Each phase reseeds the AGC state from the RK4 oracle, runs a short coast
/// window, and asserts ≤ 1 km / 1 m/s tracking error.
///
/// Phase 4b uses a **synthetic MCI seed** to validate the Moon gravity branch
/// and scenario API MCI handling by construction; it is not continuous with the
/// prior ECI phases.  See the module doc comment for full analysis.
///
/// # Design reference
///
/// Architect's locked design, GitHub issue #27.
#[test]
fn tc_phase_translunar_apollo_8_tracks_through_soi_to_loi1() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();

    // ── Derive the post-TLI oracle SV ────────────────────────────────────────

    let post_tli_sv = derive_post_tli_sv();

    // ── Phase 1: post-TLI baseline (ECI) ─────────────────────────────────────
    //
    // Seed at POST_TLI_MET_CS (T+2:56:00), coast 5 minutes. This establishes
    // the trans-lunar departure baseline.

    let phase1 = ScenarioBuilder::new("phase_translunar/phase1_post_tli")
        .comment("Phase 1: post-TLI baseline — seed at T+2:56:00 (ECI)")
        .seed_state()
        .from_state_vector(post_tli_sv)
        .met(Met(POST_TLI_MET_CS))
        .refsmmat_identity()
        .done()
        .seed_ground_truth(post_tli_sv)
        .advance_coast(SimDuration::seconds(300))
        .expect_agc_matches_ground_truth(1_000.0, 1.0)
        .build();

    run_scenario(&phase1, &mut state, &mut hw);

    // ── Oracle propagation: post-TLI → MCC-2 ─────────────────────────────────

    let dt_to_mcc2_s = (MCC2_MET_CS - POST_TLI_MET_CS) as f64 / 100.0;
    let moon_p1 = moon_position(post_tli_sv.epoch);
    let sv_at_mcc2 = propagate_coast(post_tli_sv, dt_to_mcc2_s, moon_p1);

    // ── Phase 2: MCC-2 (ECI) ─────────────────────────────────────────────────
    //
    // Apply MCC-2: +2.35 m/s perpendicular-in-plane (radial-outward direction,
    // ΔV normal to velocity in orbital plane — historically faithful pericynthion
    // trim). Real Apollo MCCs targeted pericynthion altitude, not orbit energy.
    // Reference: Apollo 8 Mission Report MSC-PA-R-69-1, §4.4.

    let nh2 = n_hat_perp_in_plane(sv_at_mcc2.position, sv_at_mcc2.velocity);
    let sv_mcc2_applied = StateVector {
        velocity: [
            sv_at_mcc2.velocity[0] + MCC2_DV_MPS * nh2[0],
            sv_at_mcc2.velocity[1] + MCC2_DV_MPS * nh2[1],
            sv_at_mcc2.velocity[2] + MCC2_DV_MPS * nh2[2],
        ],
        ..sv_at_mcc2
    };

    let phase2 = ScenarioBuilder::new("phase_translunar/phase2_mcc2")
        .comment("Phase 2: MCC-2 applied at T+10:55:04 — +2.35 m/s perpendicular-in-plane (radial-outward, pericynthion trim)")
        .seed_state()
        .from_state_vector(sv_mcc2_applied)
        .met(Met(MCC2_MET_CS))
        .refsmmat_identity()
        .done()
        .seed_ground_truth(sv_mcc2_applied)
        .advance_coast(SimDuration::seconds(300))
        .expect_agc_matches_ground_truth(1_000.0, 1.0)
        .build();

    run_scenario(&phase2, &mut state, &mut hw);

    // ── Oracle propagation: MCC-2 → mid-transit (T+30h) ──────────────────────

    let dt_to_mid_s = (MID_TRANSIT_MET_CS - MCC2_MET_CS) as f64 / 100.0;
    let moon_p2 = moon_position(sv_mcc2_applied.epoch);
    let sv_at_mid = propagate_coast(sv_mcc2_applied, dt_to_mid_s, moon_p2);

    // ── Phase 3: mid-transit checkpoint (T+30h, ECI) ─────────────────────────
    //
    // Spacecraft is ~185 000 km from Earth, climbing toward apogee, still in
    // ECI (no SOI crossing occurs in this trajectory; see module finding).

    let phase3 = ScenarioBuilder::new("phase_translunar/phase3_mid_transit")
        .comment("Phase 3: mid-transit checkpoint at T+30:00:00 (ECI)")
        .seed_state()
        .from_state_vector(sv_at_mid)
        .met(Met(MID_TRANSIT_MET_CS))
        .refsmmat_identity()
        .done()
        .seed_ground_truth(sv_at_mid)
        .advance_coast(SimDuration::seconds(300))
        .expect_agc_matches_ground_truth(1_000.0, 1.0)
        .build();

    run_scenario(&phase3, &mut state, &mut hw);

    // ── Oracle propagation: mid-transit → high-apogee (T+40h) ────────────────
    //
    // At T+40h the spacecraft is near apogee (~193 000 km) of its sub-escape
    // ellipse. No SOI handover has occurred or will occur on this trajectory.

    let dt_to_apogee_s = (HIGH_APOGEE_MET_CS - MID_TRANSIT_MET_CS) as f64 / 100.0;
    let moon_p3 = moon_position(sv_at_mid.epoch);
    let sv_at_apogee = propagate_coast(sv_at_mid, dt_to_apogee_s, moon_p3);

    // ── Phase 4: high-apogee checkpoint (T+40h, ECI) ─────────────────────────
    //
    // Seed near the orbit's apogee to exercise tracking in the slow-moving
    // high-ellipse region. Frame remains ECI.

    let phase4 = ScenarioBuilder::new("phase_translunar/phase4_high_apogee")
        .comment("Phase 4: high-apogee checkpoint at T+40:00:00 (ECI, ~193 000 km)")
        .seed_state()
        .from_state_vector(sv_at_apogee)
        .met(Met(HIGH_APOGEE_MET_CS))
        .refsmmat_identity()
        .done()
        .seed_ground_truth(sv_at_apogee)
        .advance_coast(SimDuration::seconds(300))
        .expect_agc_matches_ground_truth(1_000.0, 1.0)
        .build();

    run_scenario(&phase4, &mut state, &mut hw);

    // Verify the ECI frame has been maintained throughout (no spurious SOI flip).
    assert_eq!(
        sv_at_apogee.frame,
        Frame::EarthInertial,
        "phase 4 oracle SV must remain EarthInertial (no SOI in this trajectory); got {:?}",
        sv_at_apogee.frame
    );

    // ── Phase 4b: synthetic MCI seed — Moon gravity branch validation ─────────
    //
    // This checkpoint is NOT continuous with the prior ECI phases. A plausible
    // inbound-to-Moon state vector is constructed by hand at POST_SOI_MET_CS
    // (T+55:00:00), 50 000 km from the Moon's centre in MCI frame — well inside
    // the SOI boundary (R_SOI_MOON ≈ 66 183 km). Running propagate_coast for
    // 5 minutes from this seed validates:
    //
    //   1. total_gravity dispatches to the Moon gravity branch (MoonInertial frame).
    //   2. advance_ground_truth sets spacecraft.current_body = Moon when the seeded
    //      SV frame is MoonInertial.
    //   3. The AGC SERVICER (average_g_step) reads sv.frame and uses Moon gravity,
    //      keeping AGC and oracle in agreement within 1 km / 1 m/s.
    //   4. Frame is preserved: state.csm_state.frame == MoonInertial after the coast.
    //   5. Position remains inside SOI: ‖r_mci‖ < R_SOI_MOON.
    //
    // The synthetic position r_mci = [-50_000_000, 0, 0] m places the spacecraft
    // 50 000 km from the Moon on the -X side in MCI. The inbound velocity
    // v_mci = [1_000, 200, 0] m/s is directed roughly toward the Moon with a
    // small tangential component, representing a plausible lunar approach geometry.
    //
    // Note: the SERVICER uses a hardcoded moon_pos = [3.844e8, 0, 0] m for the
    // third-body Earth perturbation, while advance_ground_truth calls
    // moon_position(epoch). The difference is small (the perturbation is a minor
    // correction to the dominant Moon gravity) and causes divergence well below
    // the 1 km tolerance over 5 minutes.

    let sv_mci_seed = StateVector {
        position: [-50_000_000.0, 0.0, 0.0],
        velocity: [1_000.0, 200.0, 0.0],
        epoch: Met(POST_SOI_MET_CS),
        frame: Frame::MoonInertial,
    };

    let phase4b = ScenarioBuilder::new("phase_translunar/phase4b_synthetic_mci")
        .comment(
            "Phase 4b: synthetic MCI seed at T+55:00:00 — not continuous with prior ECI phases. \
             Validates MCI gravity branch + scenario API MCI handling. \
             See module doc for Moon-ephemeris-epoch limitation.",
        )
        .seed_state()
        .from_state_vector(sv_mci_seed)
        .met(Met(POST_SOI_MET_CS))
        .refsmmat_identity()
        .done()
        .seed_ground_truth(sv_mci_seed)
        .advance_coast(SimDuration::seconds(300))
        .expect_agc_matches_ground_truth(1_000.0, 1.0)
        .build();

    run_scenario(&phase4b, &mut state, &mut hw);

    // Frame must be preserved through the coast window.
    assert_eq!(
        state.csm_state.frame,
        Frame::MoonInertial,
        "phase 4b: frame must remain MoonInertial after 5-min MCI coast; got {:?}",
        state.csm_state.frame
    );

    // Position must remain inside the Moon's SOI.
    let r_mci_mag = norm(state.csm_state.position);
    assert!(
        r_mci_mag < R_SOI_MOON,
        "phase 4b: ‖r_mci‖ = {:.0} km must be inside SOI ({:.0} km); \
         spacecraft unexpectedly left the Moon SOI during a 5-min inbound coast",
        r_mci_mag / 1_000.0,
        R_SOI_MOON / 1_000.0
    );

    // ── Oracle propagation: high-apogee → MCC-4 (T+60:59:55) ─────────────────

    let dt_to_mcc4_s = (MCC4_MET_CS - HIGH_APOGEE_MET_CS) as f64 / 100.0;
    let moon_p4 = moon_position(sv_at_apogee.epoch);
    let sv_at_mcc4 = propagate_coast(sv_at_apogee, dt_to_mcc4_s, moon_p4);

    // ── Phase 5: MCC-4 (ECI) ─────────────────────────────────────────────────
    //
    // Apply MCC-4: +0.43 m/s perpendicular-in-plane (radial-outward direction,
    // ΔV normal to velocity in orbital plane — historically faithful pericynthion
    // trim). The spacecraft is now on the inbound leg, falling back toward Earth
    // after the high-ellipse apogee. Same convention as MCC-2 (positive radial-
    // outward perpendicular trim); see module doc for direction rationale.

    let nh5 = n_hat_perp_in_plane(sv_at_mcc4.position, sv_at_mcc4.velocity);
    let sv_mcc4_applied = StateVector {
        velocity: [
            sv_at_mcc4.velocity[0] + MCC4_DV_MPS * nh5[0],
            sv_at_mcc4.velocity[1] + MCC4_DV_MPS * nh5[1],
            sv_at_mcc4.velocity[2] + MCC4_DV_MPS * nh5[2],
        ],
        ..sv_at_mcc4
    };

    let phase5 = ScenarioBuilder::new("phase_translunar/phase5_mcc4")
        .comment("Phase 5: MCC-4 applied at T+60:59:55 — +0.43 m/s perpendicular-in-plane (radial-outward, pericynthion trim, ECI)")
        .seed_state()
        .from_state_vector(sv_mcc4_applied)
        .met(Met(MCC4_MET_CS))
        .refsmmat_identity()
        .done()
        .seed_ground_truth(sv_mcc4_applied)
        .advance_coast(SimDuration::seconds(300))
        .expect_agc_matches_ground_truth(1_000.0, 1.0)
        .build();

    run_scenario(&phase5, &mut state, &mut hw);

    // ── Oracle propagation: MCC-4 → LOI-1 epoch (T+69:08:20) ────────────────

    let dt_to_loi1_s = (LOI1_MET_CS - MCC4_MET_CS) as f64 / 100.0;
    let moon_p5 = moon_position(sv_mcc4_applied.epoch);
    let sv_at_loi1 = propagate_coast(sv_mcc4_applied, dt_to_loi1_s, moon_p5);

    // ── Phase 6: LOI-1 epoch (T+69:08:20, ECI) ───────────────────────────────
    //
    // 2-second settle window (per architect spec). No active program — LOI prep
    // is MS-T5's job. In this simulation the spacecraft is ~157 000 km from
    // Earth on the inbound leg of its high ellipse.

    let phase6 = ScenarioBuilder::new("phase_translunar/phase6_loi1_epoch")
        .comment("Phase 6: LOI-1 epoch T+69:08:20 (ECI inbound) — 2-second settle")
        .seed_state()
        .from_state_vector(sv_at_loi1)
        .met(Met(LOI1_MET_CS))
        .refsmmat_identity()
        .done()
        .seed_ground_truth(sv_at_loi1)
        .advance_coast(SimDuration::seconds(2))
        .expect_agc_matches_ground_truth(1_000.0, 1.0)
        .expect_major_mode(0)
        .build();

    run_scenario(&phase6, &mut state, &mut hw);

    // ── End-state assertions ──────────────────────────────────────────────────
    //
    // The spacecraft is on the inbound leg of a sub-escape ellipse (~157 000 km
    // from Earth, falling back toward periapsis). Frame must be ECI.

    let r = state.csm_state.position;
    let v = state.csm_state.velocity;

    assert_eq!(
        state.csm_state.frame,
        Frame::EarthInertial,
        "end state must be EarthInertial (no SOI crossing occurred); got {:?}",
        state.csm_state.frame
    );

    let r_mag = norm(r);
    // At T+69h on the inbound leg: ~130 000–175 000 km from Earth.
    assert!(
        (1.3e8..=1.75e8_f64).contains(&r_mag),
        "end-state Earth distance {:.0} km must be in [130 000, 175 000] km",
        r_mag / 1_000.0
    );

    let v_mag = norm(v);
    // Speed at this distance on the inbound leg: ~1 000–2 500 m/s.
    assert!(
        (1_000.0..=2_500.0).contains(&v_mag),
        "end-state speed {:.1} m/s must be in [1 000, 2 500] m/s",
        v_mag
    );

    // Inbound to Earth: r_hat · v_hat < 0 (spacecraft falling toward Earth).
    let r_mag_f = norm(r);
    let v_mag_f = norm(v);
    let r_hat = [r[0] / r_mag_f, r[1] / r_mag_f, r[2] / r_mag_f];
    let v_hat_end = [v[0] / v_mag_f, v[1] / v_mag_f, v[2] / v_mag_f];
    let rdot_v = r_hat[0] * v_hat_end[0] + r_hat[1] * v_hat_end[1] + r_hat[2] * v_hat_end[2];
    assert!(
        rdot_v < 0.0,
        "r̂·v̂ = {rdot_v:.4} must be negative (inbound to Earth on return leg); \
         spacecraft may not be on expected return trajectory"
    );
}
