//! P11 — Earth Orbit Insertion Monitor
//!
//! Passive monitor that continuously refreshes the DSKY V16N44 display
//! (apogee altitude / perigee altitude / half-period) from the current
//! `csm_state`. Installed as a SERVICER exit hook so the display tracks
//! the state vector as it is integrated during powered ascent.
//!
//! AGC source: Comanche055/P11.agc.

use crate::executive::job::JobPriority;
use crate::navigation::conics::{
    apoapsis_altitude_earth, orbital_period, periapsis_altitude_earth, sv_to_elements,
};
use crate::navigation::gravity::MU_EARTH;
use crate::navigation::state_vector::Frame;

// ── Constants ─────────────────────────────────────────────────────────────────

/// Major mode number for P11.
pub const P11_MAJOR_MODE: u8 = 11;

/// Background monitor job priority.
pub const PRIORITY: JobPriority = 6;

/// DSKY verb for continuous monitor display (V16 — updated each cycle).
const VERB_MONITOR: u8 = 16;

/// DSKY noun for apogee/perigee/TFF triplet (N44).
const NOUN_APO_PERI_TFF: u8 = 44;

// ── Program alarms ────────────────────────────────────────────────────────────

const ALARM_HYPERBOLIC_ORBIT: u16 = 229;
const ALARM_WRONG_FRAME: u16 = 230;

// ── Entry point ───────────────────────────────────────────────────────────────

/// Entry point registered in `PROGRAM_TABLE[11]`.
///
/// Validates that `state.csm_state.frame == Frame::EarthInertial`,
/// sets the major mode and DSKY cue, installs the SERVICER exit hook,
/// and runs one immediate update so the display reflects the orbit at
/// program selection. Always returns `PRIORITY` even on alarm (the
/// executive needs a priority to slot the program).
pub fn init(state: &mut crate::AgcState) -> JobPriority {
    if state.csm_state.frame != Frame::EarthInertial {
        state.alarm.code = ALARM_WRONG_FRAME;
        state.alarm.lit = true;
        return PRIORITY;
    }

    state.major_mode = P11_MAJOR_MODE;
    state.dsky.prog = P11_MAJOR_MODE;
    state.dsky.verb = VERB_MONITOR;
    state.dsky.noun = NOUN_APO_PERI_TFF;
    state.dsky.flashing = false;
    state.servicer_exit = Some(p11_servicer_exit);

    p11_update(state);

    PRIORITY
}

/// Recompute the N44 display from the current `csm_state`.
///
/// Converts the state vector to classical orbital elements and writes
/// apogee altitude, perigee altitude, and half the orbital period into
/// `dsky.r[0..3]`. On a hyperbolic orbit (no apoapsis) it raises alarm
/// 229 and leaves the display untouched.
pub fn p11_update(state: &mut crate::AgcState) {
    let elements = sv_to_elements(state.csm_state);

    if elements.is_hyperbolic() {
        state.alarm.code = ALARM_HYPERBOLIC_ORBIT;
        state.alarm.lit = true;
        return;
    }

    let apo_m = apoapsis_altitude_earth(&elements);
    let peri_m = periapsis_altitude_earth(&elements);
    let half_period_s = orbital_period(&elements, MU_EARTH) / 2.0;

    state.dsky.r[0] = apo_m as f32;
    state.dsky.r[1] = peri_m as f32;
    state.dsky.r[2] = half_period_s as f32;
}

/// SERVICER exit hook — refreshes the N44 display every 2-second cycle.
pub fn p11_servicer_exit(state: &mut crate::AgcState) {
    p11_update(state);
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::gravity::R_EARTH;
    use crate::navigation::state_vector::StateVector;
    use crate::types::Met;
    use crate::AgcState;

    /// Build a circular-LEO state vector at `alt_m` above the Earth surface,
    /// positioned on the +X axis with velocity along +Y.
    fn circular_leo(alt_m: f64) -> StateVector {
        let r = R_EARTH + alt_m;
        let v = libm::sqrt(MU_EARTH / r);
        StateVector {
            position: [r, 0.0, 0.0],
            velocity: [0.0, v, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        }
    }

    /// TC-P11-1: `init` on EarthInertial state sets major_mode = 11.
    #[test]
    fn tc_p11_1_init_sets_major_mode() {
        let mut state = AgcState::new();
        state.csm_state = circular_leo(400_000.0);

        let prio = init(&mut state);

        assert_eq!(prio, PRIORITY);
        assert_eq!(state.major_mode, P11_MAJOR_MODE);
        assert_eq!(state.dsky.prog, P11_MAJOR_MODE);
        assert_eq!(state.dsky.verb, VERB_MONITOR);
        assert_eq!(state.dsky.noun, NOUN_APO_PERI_TFF);
        assert!(!state.dsky.flashing);
        assert!(state.servicer_exit.is_some());
        assert_eq!(state.alarm.code, 0);
    }

    /// TC-P11-2: `init` on MoonInertial frame raises alarm 230.
    #[test]
    fn tc_p11_2_wrong_frame_alarm() {
        let mut state = AgcState::new();
        state.csm_state = StateVector {
            position: [2_000_000.0, 0.0, 0.0],
            velocity: [0.0, 1_500.0, 0.0],
            epoch: Met(0),
            frame: Frame::MoonInertial,
        };

        init(&mut state);

        assert_eq!(state.alarm.code, ALARM_WRONG_FRAME);
        assert!(state.alarm.lit);
        assert_ne!(
            state.major_mode, P11_MAJOR_MODE,
            "major_mode must not advance on wrong-frame alarm"
        );
    }

    /// TC-P11-3: `p11_update` on a 400 km circular LEO yields apogee ≈ perigee ≈ 400 km.
    #[test]
    fn tc_p11_3_circular_leo_display() {
        let mut state = AgcState::new();
        state.csm_state = circular_leo(400_000.0);

        p11_update(&mut state);

        // Circular: apogee ≈ perigee ≈ 400 000 m, tolerance 1 m.
        let apo = state.dsky.r[0] as f64;
        let peri = state.dsky.r[1] as f64;
        assert!(
            (apo - 400_000.0).abs() < 10.0,
            "apogee ≈ 400 km, got {apo} m"
        );
        assert!(
            (peri - 400_000.0).abs() < 10.0,
            "perigee ≈ 400 km, got {peri} m"
        );
        assert!(state.dsky.r[2] > 0.0, "half-period must be positive");
    }

    /// TC-P11-4: `p11_update` on an elliptic 400×1200 km orbit at perigee.
    #[test]
    fn tc_p11_4_elliptic_400_1200_at_perigee() {
        let mut state = AgcState::new();

        let r_peri = R_EARTH + 400_000.0;
        let r_apo = R_EARTH + 1_200_000.0;
        let a = 0.5 * (r_peri + r_apo);
        // vis-viva: v² = μ(2/r − 1/a) at perigee.
        let v_peri = libm::sqrt(MU_EARTH * (2.0 / r_peri - 1.0 / a));

        state.csm_state = StateVector {
            position: [r_peri, 0.0, 0.0],
            velocity: [0.0, v_peri, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        p11_update(&mut state);

        let apo = state.dsky.r[0] as f64;
        let peri = state.dsky.r[1] as f64;

        assert!(
            (apo - 1_200_000.0).abs() < 100.0,
            "apogee ≈ 1200 km, got {apo} m"
        );
        assert!(
            (peri - 400_000.0).abs() < 100.0,
            "perigee ≈ 400 km, got {peri} m"
        );
    }

    /// TC-P11-5: hyperbolic trajectory raises alarm 229 and does not overwrite display.
    #[test]
    fn tc_p11_5_hyperbolic_alarm() {
        let mut state = AgcState::new();

        // Pre-seed a known display to verify it is preserved on alarm.
        state.dsky.r = [999.0, 888.0, 777.0];

        let r = R_EARTH + 400_000.0;
        let v_escape = libm::sqrt(2.0 * MU_EARTH / r);
        // 20% above escape velocity — comfortably hyperbolic.
        state.csm_state = StateVector {
            position: [r, 0.0, 0.0],
            velocity: [0.0, 1.2 * v_escape, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        p11_update(&mut state);

        assert_eq!(state.alarm.code, ALARM_HYPERBOLIC_ORBIT);
        assert!(state.alarm.lit);
        // Display must be preserved.
        assert_eq!(state.dsky.r, [999.0, 888.0, 777.0]);
    }

    /// TC-P11-6: `p11_servicer_exit` refreshes the display from csm_state.
    #[test]
    fn tc_p11_6_servicer_exit_refresh() {
        let mut state = AgcState::new();
        state.csm_state = circular_leo(400_000.0);

        let _ = init(&mut state);
        let apo_initial = state.dsky.r[0];

        // Slew the state vector to a higher orbit and re-run the hook.
        state.csm_state = circular_leo(800_000.0);
        p11_servicer_exit(&mut state);

        let apo_new = state.dsky.r[0];
        assert!(
            (apo_new as f64 - 800_000.0).abs() < 10.0,
            "refreshed apogee must reflect new 800 km orbit, got {apo_new} m"
        );
        assert!(
            (apo_new - apo_initial) > 100.0,
            "display must change after SERVICER refresh"
        );
    }
}
