//! P30 — External Delta-V Targeting
//!
//! Accepts a ground-uploaded maneuver (TIG + LVLH delta-V) and computes
//! the required burn attitude. The result is stored in
//! `AgcState::pending_maneuver` for consumption by P40/P41.
//!
//! AGC source: Comanche055/P30,P37.agc

use crate::executive::job::JobPriority;
use crate::executive::restart::Phase;
use crate::guidance::targeting::apply_external_delta_v;
use crate::math::linalg::norm;
use crate::programs::MajorMode;
use crate::types::{Met, Vec3};

// ── Constants ─────────────────────────────────────────────────────────────────

/// Major mode number for P30.
pub const P30_MAJOR_MODE: u8 = 30;

/// Job priority for the P30 targeting computation job.
pub const P30_JOB_PRIORITY: JobPriority = 4;

/// DSKY Noun used to display the burn summary (N45).
pub const P30_NOUN_BURN_SUMMARY: u8 = 45;

/// DSKY Noun used to display and load LVLH delta-V components (N81).
pub const P30_NOUN_DV_LVLH: u8 = 81;

/// DSKY Noun used to display and load TIG (N33).
pub const P30_NOUN_TIG: u8 = 33;

/// DSKY Verb for displaying three registers (read-only).
pub const VERB_DISPLAY_OCT: u8 = 6;

// ── Program alarm code for targeting errors ───────────────────────────────────

/// Program alarm: TIG in the past.
const ALARM_TIG_IN_PAST: u16 = 210;

// ── Entry point registered in PROGRAM_TABLE ───────────────────────────────────

/// Entry point registered in `PROGRAM_TABLE[30]`.
/// Delegates to `p30_init`.
pub fn init(state: &mut crate::AgcState) -> JobPriority {
    p30_init(state)
}

// ── Core functions ────────────────────────────────────────────────────────────

/// P30 entry point — sets the major mode, initialises the DSKY display to
/// request crew TIG entry, and clears any stale pending maneuver.
///
/// # Returns
///
/// `JobPriority` for the Executive job associated with P30's targeting
/// computation (`P30_JOB_PRIORITY = 4`).
///
/// # Side effects
///
/// 1. Sets `state.major_mode = 30`.
/// 2. Sets `state.dsky.prog = 30`.
/// 3. Sets `state.dsky.verb = VERB_DISPLAY_OCT` (6).
/// 4. Sets `state.dsky.noun = P30_NOUN_TIG` (33).
/// 5. Sets `state.dsky.flashing = true` — signals crew input required.
/// 6. Clears `state.pending_maneuver = None`.
pub fn p30_init(state: &mut crate::AgcState) -> JobPriority {
    state.major_mode = P30_MAJOR_MODE;
    state.dsky.prog = P30_MAJOR_MODE;
    state.dsky.verb = VERB_DISPLAY_OCT;
    state.dsky.noun = P30_NOUN_TIG;
    state.dsky.flashing = true;
    state.pending_maneuver = None;
    // NOTE: interactive DSKY data-load state machine (V25 N33, V25 N81) is
    // Milestone 5. For now, the test harness calls p30_load_dv_lvlh directly.
    P30_JOB_PRIORITY
}

/// Accept a ground-uploaded maneuver and compute the inertial `Maneuver`.
///
/// `dv_crew` is in crew-entry order: [along-track (X), radial (Y), cross-track (Z)]
/// (LVLH per spec §2.3). Internally re-ordered to RSW (R, S, W) before
/// passing to `apply_external_delta_v` which expects [radial, along-track, cross].
///
/// # Arguments
///
/// * `state`    — Mutable AGC state. Reads `csm_state`, `refsmmat`, `time`.
/// * `tig`      — Time of Ignition as mission elapsed time in centiseconds.
///               Must be >= `state.time`; if not, alarm 210 is raised and
///               `pending_maneuver` is left unchanged.
/// * `dv_crew`  — Delta-V as entered via N81 [X_along, Y_radial, Z_cross] (m/s).
///
/// # Side effects
///
/// On success: stores `Some(maneuver)` in `state.pending_maneuver` and calls
/// `p30_display_summary`. On TIG-in-past error: sets alarm 210 and returns
/// without modifying `pending_maneuver`.
pub fn p30_load_dv_lvlh(state: &mut crate::AgcState, tig: Met, dv_crew: Vec3) {
    // Guard: TIG must not be in the past.
    if tig < state.time {
        state.alarm.code = ALARM_TIG_IN_PAST;
        state.alarm.lit = true;
        return;
    }

    // Re-order crew [X_along, Y_radial, Z_cross] → LVLH [R, S, W]
    // R = radial       = crew Y (index 1)
    // S = along-track  = crew X (index 0)
    // W = cross-track  = crew Z (index 2)
    let dv_rsw: Vec3 = [dv_crew[1], dv_crew[0], dv_crew[2]];

    let maneuver = apply_external_delta_v(
        state.csm_state,
        tig,
        dv_rsw,
        state.refsmmat,
    );
    state.pending_maneuver = Some(maneuver);
    p30_display_summary(state);
}

/// Populate the DSKY N45 burn-summary display with delta-V magnitude,
/// TIG-35min countdown, and TIG.
///
/// Does nothing if `state.pending_maneuver` is `None`.
///
/// # DSKY fields written
///
/// ```text
/// dsky.verb   = 6   (VERB_DISPLAY_OCT)
/// dsky.noun   = 45  (P30_NOUN_BURN_SUMMARY)
/// dsky.r[0]   = |delta_v| as f32   (m/s)
/// dsky.r[1]   = (tig - 35 min) in cs, clamped to 0 if < 35 min from epoch
/// dsky.r[2]   = tig.0 as f32       (centiseconds)
/// dsky.flashing = false
/// ```
pub fn p30_display_summary(state: &mut crate::AgcState) {
    if let Some(m) = state.pending_maneuver {
        let dv_mag = norm(m.delta_v.0);
        // 35 minutes expressed in centiseconds.
        let thirty_five_min_cs: u32 = 35 * 60 * 100;
        let tig_minus_35 = m.tig.0.saturating_sub(thirty_five_min_cs);

        state.dsky.verb = VERB_DISPLAY_OCT;
        state.dsky.noun = P30_NOUN_BURN_SUMMARY;
        state.dsky.r[0] = dv_mag as f32;
        state.dsky.r[1] = tig_minus_35 as f32;
        state.dsky.r[2] = m.tig.0 as f32;
        state.dsky.flashing = false;
    }
}

// ── MajorMode trait implementation ────────────────────────────────────────────

/// Zero-sized type that implements the `MajorMode` trait for P30.
pub struct P30;

impl MajorMode for P30 {
    fn number(&self) -> u8 {
        P30_MAJOR_MODE
    }

    fn start(&self, state: &mut crate::AgcState) -> JobPriority {
        p30_init(state)
    }

    fn handle_display_input(&self, state: &mut crate::AgcState, verb: u8, noun: u8) {
        match (verb, noun) {
            (VERB_DISPLAY_OCT, P30_NOUN_BURN_SUMMARY) => p30_display_summary(state),
            // V25 N33 and V25 N81 are handled by the V/N processor (Milestone 5).
            _ => { /* unsolicited verb/noun; no action */ }
        }
    }

    fn restart_resume(&self, state: &mut crate::AgcState, _phase: Phase) {
        // P30 has no long-running computation requiring mid-phase restart.
        // Re-enter from the top; redisplay if a maneuver exists.
        p30_init(state);
        p30_display_summary(state);
    }

    fn terminate(&self, state: &mut crate::AgcState) {
        // Clear flashing indicator; leave pending_maneuver intact for P40/P41.
        state.dsky.flashing = false;
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AgcState;
    use crate::guidance::targeting::TargetingMode;
    use crate::math::linalg::norm;
    use crate::types::Met;

    /// TC-P30-1: Zero delta-V produces zero inertial delta-V and identity burn attitude.
    ///
    /// Verifies the identity case — a zero LVLH delta-V results in a `Maneuver`
    /// with zero `delta_v` and identity `burn_attitude`.
    #[test]
    fn tc_p30_1_zero_delta_v() {
        let mut state = AgcState::new();

        // ISS-like circular LEO at 400 km altitude, equatorial orbit.
        let r = 6_778_137.0_f64; // metres (R_Earth + 400 km)
        let v_circ = libm::sqrt(3.986_004_418e14_f64 / r); // ~7784 m/s

        state.csm_state.position = [r, 0.0, 0.0];
        state.csm_state.velocity = [0.0, v_circ, 0.0];
        state.time = Met(0);
        state.refsmmat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

        let tig = Met(360_000); // TIG = 1 hour from epoch (centiseconds)
        let dv_crew: Vec3 = [0.0, 0.0, 0.0];

        p30_load_dv_lvlh(&mut state, tig, dv_crew);

        let m = state
            .pending_maneuver
            .expect("pending_maneuver must be Some after p30_load_dv_lvlh");

        assert_eq!(m.tig, tig);
        assert!(
            norm(m.delta_v.0) < 1e-9,
            "delta_v magnitude must be zero, got {}",
            norm(m.delta_v.0)
        );
        assert_eq!(m.mode, TargetingMode::ExternalDeltaV);

        // burn_attitude must be identity for zero delta-V.
        let id = [[1.0_f64, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (m.burn_attitude[i][j] - id[i][j]).abs() < 1e-9,
                    "burn_attitude[{}][{}] must be identity, got {}",
                    i,
                    j,
                    m.burn_attitude[i][j]
                );
            }
        }
    }

    /// TC-P30-2: Prograde-only delta-V maps to inertial +Y direction.
    ///
    /// For a circular equatorial orbit with position along +X and velocity
    /// along +Y, a prograde (crew X = along-track = S-axis) delta-V of
    /// 100 m/s must appear in the inertial +Y direction.
    #[test]
    fn tc_p30_2_prograde_dv_maps_to_velocity_direction() {
        let mut state = AgcState::new();

        let r = 6_778_137.0_f64;
        let v_circ = libm::sqrt(3.986_004_418e14_f64 / r);

        // Position along +X, velocity along +Y (equatorial circular orbit).
        state.csm_state.position = [r, 0.0, 0.0];
        state.csm_state.velocity = [0.0, v_circ, 0.0];
        state.time = Met(0);
        state.refsmmat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

        let tig = Met(360_000);
        // Crew enters [X=100.0, Y=0.0, Z=0.0] — 100 m/s prograde (X = along-track).
        let dv_crew: Vec3 = [100.0, 0.0, 0.0];

        p30_load_dv_lvlh(&mut state, tig, dv_crew);

        let m = state
            .pending_maneuver
            .expect("pending_maneuver must be Some");
        let dv = m.delta_v.0;

        assert!(
            dv[0].abs() < 1e-6,
            "inertial dv[0] (X) must be ~0, got {}",
            dv[0]
        );
        assert!(
            (dv[1] - 100.0).abs() < 1e-6,
            "inertial dv[1] (Y) must be ~100 m/s, got {}",
            dv[1]
        );
        assert!(
            dv[2].abs() < 1e-6,
            "inertial dv[2] (Z) must be ~0, got {}",
            dv[2]
        );

        // Magnitude must be preserved.
        assert!(
            (norm(dv) - 100.0).abs() < 1e-9,
            "delta-V magnitude must be preserved at 100 m/s, got {}",
            norm(dv)
        );
    }

    /// TC-P30-3: `p30_init` sets major_mode = 30 and updates DSKY PROG display.
    #[test]
    fn tc_p30_3_init_sets_major_mode() {
        let mut state = AgcState::new();
        state.major_mode = 0; // start in P00

        let _ = p30_init(&mut state);

        assert_eq!(state.major_mode, 30, "major_mode must be set to 30 by p30_init");
        assert_eq!(state.dsky.prog, 30, "dsky.prog must reflect major mode 30");
        assert_eq!(
            state.dsky.noun, P30_NOUN_TIG,
            "dsky.noun must be 33 (TIG entry cue)"
        );
        assert!(
            state.dsky.flashing,
            "dsky.flashing must be true to signal crew input required"
        );
        assert!(
            state.pending_maneuver.is_none(),
            "pending_maneuver must be cleared on init"
        );
    }

    /// TC-P30-4: `p30_load_dv_lvlh` stores result in `state.pending_maneuver`.
    ///
    /// Verifies the persistence contract — after `p30_load_dv_lvlh` returns,
    /// `pending_maneuver` is `Some` and carries the correct TIG, mode, and
    /// DSKY summary noun/verb.
    #[test]
    fn tc_p30_4_stores_pending_maneuver() {
        let mut state = AgcState::new();

        let r = 6_778_137.0_f64;
        let v_circ = libm::sqrt(3.986_004_418e14_f64 / r);
        state.csm_state.position = [r, 0.0, 0.0];
        state.csm_state.velocity = [0.0, v_circ, 0.0];
        state.time = Met(0);
        state.refsmmat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

        // Verify pending_maneuver starts as None.
        assert!(state.pending_maneuver.is_none());

        let tig = Met(720_000); // 2 hours
        let dv_crew: Vec3 = [50.0, 10.0, -5.0];

        p30_load_dv_lvlh(&mut state, tig, dv_crew);

        let m = state
            .pending_maneuver
            .expect("pending_maneuver must be Some after load");
        assert_eq!(m.tig, tig, "tig must match the input TIG");
        assert_eq!(
            m.mode,
            TargetingMode::ExternalDeltaV,
            "mode must be ExternalDeltaV for P30"
        );

        // Verify DSKY summary was also populated.
        assert_eq!(
            state.dsky.noun, P30_NOUN_BURN_SUMMARY,
            "dsky.noun must be 45 after display_summary"
        );
        assert_eq!(
            state.dsky.verb, VERB_DISPLAY_OCT,
            "dsky.verb must be 6 after display_summary"
        );
    }
}
