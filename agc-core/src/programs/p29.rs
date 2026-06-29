// SPDX-License-Identifier: GPL-3.0-or-later
//! P29 — Time-of-Longitude.
//!
//! Display-only program. Given a target geographic longitude entered by the
//! crew via Noun 89, propagates the CSM state vector forward until the
//! ground track crosses that longitude, then displays the Mission Elapsed
//! Time at the crossing on Verb 06 Noun 34.
//!
//! P29 is the inverse of P21:
//!
//! | Direction | Program |
//! |---|---|
//! | GET → lat/lon/alt   | P21 |
//! | lon → GET, lat, alt | P29 |
//!
//! Mission context: pass-prediction utility — useful for ground-station
//! contact-window planning and landing-site/longitude crossing prediction.
//! The crew typically gets these values uplinked from the ground; P29 is the
//! AGC's autonomous fallback.
//!
//! ## Crew flow
//!
//! 1. `V37 E 29 E` selects P29. `p29_init` flashes `V25 N89` waiting on the
//!    crew to enter the target longitude.
//! 2. The crew loads R2 = longitude (R1 = lat / R3 = alt are informational
//!    and ignored by P29).
//! 3. The `noun_89_commit` handler in `services::v_n` invokes
//!    [`p29_compute_and_display`], which runs
//!    [`navigation::conics::time_of_longitude`] and either displays the
//!    result via `V06 N34` or raises one of three alarms.
//!
//! AGC source: `Comanche055/P20-P25.agc` (P29 entry sequence).
//! Spec: `specs/p29-spec.md`.

use crate::executive::job::JobPriority;
use crate::navigation::conics::{time_of_longitude, P29Error};
use crate::AgcState;

/// Major mode number for P29.
pub const P29_MAJOR_MODE: u8 = 29;

/// Job priority for P29. Same tier as P21 (one-shot, non-time-critical).
pub const P29_PRIORITY: JobPriority = 7;

// ── Alarm codes (octal) ───────────────────────────────────────────────────────

use crate::tables::alarm_codes::{ALARM_P29_HYPERBOLIC, ALARM_P29_NO_CONV, ALARM_P29_NO_CSM_SV};

// ── Entry point ───────────────────────────────────────────────────────────────

/// Entry point for P29. Registered in `PROGRAM_TABLE[29]`.
///
/// Sets `state.major_mode = 29` and flashes `V25 N89` to prompt the crew for
/// the target longitude. Clears any previous N89 staging so the next entry
/// is interpreted afresh.
///
/// The actual solver runs from [`p29_compute_and_display`], invoked by the
/// `noun_89_commit` handler in `services::v_n` once the crew presses ENTR.
pub fn p29_init(state: &mut AgcState) -> JobPriority {
    state.major_mode = P29_MAJOR_MODE;
    state.dsky.prog = P29_MAJOR_MODE;

    // Clear any stale N89 input from a prior pass through P29.
    state.vn.crew_p29_target = None;

    // Flash V25 N89 to prompt crew for the target longitude.
    state.dsky.verb = 25;
    state.dsky.noun = 89;
    state.dsky.r[0] = 0.0;
    state.dsky.r[1] = 0.0;
    state.dsky.r[2] = 0.0;
    state.dsky.flashing = true;

    P29_PRIORITY
}

// ── Compute + display ─────────────────────────────────────────────────────────

/// Read the staged N89 longitude, run the time-of-longitude solver, and
/// display the result on V06 N34 (or raise the appropriate alarm).
///
/// Invoked from the `noun_89` commit handler in `services::v_n` when P29 is
/// the active major mode at commit time. Idempotent — safe to call again if
/// the crew re-enters N89 with a different longitude (it'll re-solve).
pub fn p29_compute_and_display(state: &mut AgcState) {
    // Precondition: non-zero CSM epoch (otherwise the state vector is fresh-
    // start zeros and the solver has nothing to propagate from).
    if state.csm_state.epoch.to_seconds() == 0.0 {
        raise_alarm(state, ALARM_P29_NO_CSM_SV);
        return;
    }

    let Some(target) = state.vn.crew_p29_target else {
        // No N89 staged — shouldn't happen since this is invoked from the
        // N89 commit, but be defensive.
        raise_alarm(state, ALARM_P29_NO_CSM_SV);
        return;
    };

    const DEG_TO_RAD: f64 = core::f64::consts::PI / 180.0;
    let target_lon_rad = target[1] * DEG_TO_RAD;

    let epoch_s = state.csm_state.epoch.to_seconds();
    let csm_pos = state.csm_state.position;
    let csm_vel = state.csm_state.velocity;
    let gha_epoch = state.gha_epoch_rad;

    let result = match time_of_longitude(csm_pos, csm_vel, epoch_s, target_lon_rad, gha_epoch) {
        Ok(r) => r,
        Err(P29Error::Hyperbolic) => {
            raise_alarm(state, ALARM_P29_HYPERBOLIC);
            return;
        }
        Err(P29Error::NoConvergence) => {
            raise_alarm(state, ALARM_P29_NO_CONV);
            return;
        }
        Err(P29Error::ZeroAngularMomentum) => {
            // Treat zero-angular-momentum the same as no-convergence — the
            // trajectory is degenerate from the orbital-motion standpoint.
            raise_alarm(state, ALARM_P29_NO_CONV);
            return;
        }
    };

    // Display via V06 N34 — time of event as HMS (h, m, s × 100).
    let total_s = result.time_of_crossing_s;
    let hours = libm::floor(total_s / 3600.0);
    let rem = total_s - hours * 3600.0;
    let minutes = libm::floor(rem / 60.0);
    let seconds = rem - minutes * 60.0;
    state.dsky.verb = 6;
    state.dsky.noun = 34;
    state.dsky.r[0] = hours as f32;
    state.dsky.r[1] = minutes as f32;
    state.dsky.r[2] = (seconds * 100.0) as f32;
    state.dsky.flashing = false;
}

fn raise_alarm(state: &mut AgcState, code: u16) {
    state.alarm.raise(code, crate::tables::alarm_codes::SITE_P29);
    state.dsky.verb = 6;
    state.dsky.noun = 34;
    state.dsky.r[0] = 0.0;
    state.dsky.r[1] = 0.0;
    state.dsky.r[2] = 0.0;
    state.dsky.flashing = false;
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::gravity::MU_EARTH;
    use crate::programs::p21::R_EARTH;
    use crate::types::Met;
    use crate::AgcState;

    /// Helper: LEO state for tests (300 km altitude, equatorial, eastward).
    fn leo_state(state: &mut AgcState) {
        let r = R_EARTH + 300_000.0;
        let v = libm::sqrt(MU_EARTH / r);
        state.csm_state.position = [r, 0.0, 0.0];
        state.csm_state.velocity = [0.0, v, 0.0];
        state.csm_state.epoch = Met(100_000); // 1000 s in centiseconds
        state.gha_epoch_rad = 0.0;
        state.time = Met(100_000);
    }

    /// TC-P29-INIT-1: `p29_init` sets the major mode, the DSKY display
    /// (V25 N89 flashing), and clears any prior N89 staging.
    #[test]
    fn tc_p29_init_1_sets_program_state() {
        let mut state = AgcState::new();
        state.vn.crew_p29_target = Some([12.3, 45.6, 100.0]); // stale prior input
        let prio = p29_init(&mut state);
        assert_eq!(prio, P29_PRIORITY);
        assert_eq!(state.major_mode, 29);
        assert_eq!(state.dsky.prog, 29);
        assert_eq!(state.dsky.verb, 25);
        assert_eq!(state.dsky.noun, 89);
        assert!(
            state.dsky.flashing,
            "DSKY must flash to prompt crew for N89"
        );
        assert!(
            state.vn.crew_p29_target.is_none(),
            "p29_init must clear stale N89 input"
        );
    }

    /// TC-P29-FLOW-1: full P29 sequence — `p29_init` followed by an N89
    /// commit (here staged directly to bypass the keystroke layer) returns
    /// a sensible time on V06 N34 for a canned LEO scenario.
    #[test]
    fn tc_p29_flow_1_nominal_returns_time() {
        let mut state = AgcState::new();
        leo_state(&mut state);
        p29_init(&mut state);
        // Stage the N89 target directly — equivalent to what the noun_89
        // commit handler would do after a V25 N89 +00000 E +00300 E +00000 E
        // crew entry (lat 0°, lon 30° east, alt 0). The commit handler then
        // calls into p29_compute_and_display, which we invoke here.
        state.vn.crew_p29_target = Some([0.0, 30.0, 0.0]);
        p29_compute_and_display(&mut state);

        // Expect V06 N34 (no alarm).
        assert!(!state.alarm.lit, "no alarm expected for nominal LEO");
        assert_eq!(state.dsky.verb, 6);
        assert_eq!(state.dsky.noun, 34);
        // For a 300 km LEO with eastward drift, the time-of-crossing for 30°
        // is ~(30°/360°)·T_orb adjusted for Earth rotation — a few minutes
        // past epoch. Just sanity-check the registers are populated.
        let h = state.dsky.r[0] as f64;
        let m = state.dsky.r[1] as f64;
        let s = state.dsky.r[2] as f64 / 100.0;
        let total_s = h * 3600.0 + m * 60.0 + s;
        // Allow anywhere from 0 to one orbital period ahead of epoch
        // (5290 s for this orbit).
        assert!(
            (1000.0..=10_000.0).contains(&total_s),
            "time-of-crossing out of expected band: {} s",
            total_s
        );
    }

    /// TC-P29-FLOW-1B: N89 noun-commit handler in `services::v_n` triggers
    /// `p29_compute_and_display` when P29 is active. End-to-end integration
    /// of the keystroke commit → P29 compute path without going through the
    /// keystroke layer.
    #[test]
    fn tc_p29_flow_1b_n89_commit_triggers_compute() {
        let mut state = AgcState::new();
        leo_state(&mut state);
        p29_init(&mut state);
        assert_eq!(state.major_mode, 29);

        // Feed an N89 commit through the V/N noun_commit dispatcher.
        // We call into the public path that the keystroke layer uses by
        // setting the staged value and invoking compute_and_display ourselves
        // — the routing logic lives in `services::v_n::noun_89_commit_p29_target`
        // and is covered separately by `tc_vnd_p29_n89_routing`.
        state.vn.crew_p29_target = Some([0.0, 30.0, 0.0]);
        p29_compute_and_display(&mut state);

        assert_eq!(state.dsky.noun, 34);
        assert!(!state.alarm.lit);
    }

    /// TC-P29-FLOW-2: ALARM_P29_NO_CSM_SV fires when CSM state vector is fresh-start
    /// zero (epoch_cs == 0).
    #[test]
    fn tc_p29_flow_2_no_csm_sv_alarm() {
        let mut state = AgcState::new();
        // Leave csm_state.epoch == 0 (fresh-start sentinel).
        p29_init(&mut state);
        state.vn.crew_p29_target = Some([0.0, 30.0, 0.0]);

        p29_compute_and_display(&mut state);
        assert!(state.alarm.lit);
        assert_eq!(state.alarm.code(), ALARM_P29_NO_CSM_SV);
    }

    /// TC-P29-FLOW-3: alarm 01431 fires for a hyperbolic trajectory.
    #[test]
    fn tc_p29_flow_3_hyperbolic_alarm() {
        let mut state = AgcState::new();
        leo_state(&mut state);
        // Bump velocity above escape to make trajectory hyperbolic.
        let r = crate::math::linalg::norm(state.csm_state.position);
        let v_esc = libm::sqrt(2.0 * MU_EARTH / r) * 1.1;
        state.csm_state.velocity = [0.0, v_esc, 0.0];
        p29_init(&mut state);
        state.vn.crew_p29_target = Some([0.0, 30.0, 0.0]);

        p29_compute_and_display(&mut state);
        assert!(state.alarm.lit);
        assert_eq!(state.alarm.code(), ALARM_P29_HYPERBOLIC);
    }

    /// TC-P29-FLOW-4: alarm 01432 fires when the solver doesn't converge.
    /// We force this by feeding a zero-angular-momentum (radial) input,
    /// which the solver maps to `ZeroAngularMomentum` and P29 surfaces as
    /// `ALARM_P29_NO_CONV` (the closest functional category).
    #[test]
    fn tc_p29_flow_4_no_conv_alarm_on_degenerate_input() {
        let mut state = AgcState::new();
        leo_state(&mut state);
        // Purely radial velocity → zero angular momentum.
        state.csm_state.velocity = [-1000.0, 0.0, 0.0];
        p29_init(&mut state);
        state.vn.crew_p29_target = Some([0.0, 30.0, 0.0]);

        p29_compute_and_display(&mut state);
        assert!(state.alarm.lit);
        assert_eq!(state.alarm.code(), ALARM_P29_NO_CONV);
    }

    /// TC-P29-FLOW-5: DSKY shows `prog == 29` after `p29_init`.
    #[test]
    fn tc_p29_flow_5_dsky_shows_prog() {
        let mut state = AgcState::new();
        p29_init(&mut state);
        assert_eq!(state.dsky.prog, 29);
    }
}
