//! P15 — Trans-Lunar Injection (TLI) Monitor.
//!
//! Passive monitor that refreshes the DSKY V16N44 triplet (apogee /
//! perigee / half-period) from the current `csm_state` during the TLI
//! phase. Shares its compute pipeline with P11 — differs only in the
//! major-mode/PROG fields and in flagging the post-TLI hyperbolic
//! trajectory as an alarm (since N44 cannot display an unbounded orbit).
//!
//! AGC source: Comanche055/P11.agc (shared block for P11/P15).

use crate::executive::job::JobPriority;
use crate::navigation::conics::{
    apoapsis_altitude_earth, orbital_period, periapsis_altitude_earth, sv_to_elements,
};
use crate::navigation::gravity::MU_EARTH;
use crate::navigation::state_vector::Frame;

pub const P15_MAJOR_MODE: u8 = 15;

/// Background monitor priority.
pub const PRIORITY: JobPriority = 6;

const VERB_MONITOR: u8 = 16;
const NOUN_APO_PERI: u8 = 44;

const ALARM_WRONG_FRAME: u16 = 236;
const ALARM_HYPERBOLIC: u16 = 237;

/// Entry point registered in `PROGRAM_TABLE[15]`.
pub fn init(state: &mut crate::AgcState) -> JobPriority {
    if state.csm_state.frame != Frame::EarthInertial {
        state.alarm.code = ALARM_WRONG_FRAME;
        state.alarm.lit = true;
        return PRIORITY;
    }

    state.major_mode = P15_MAJOR_MODE;
    state.dsky.prog = P15_MAJOR_MODE;
    state.dsky.verb = VERB_MONITOR;
    state.dsky.noun = NOUN_APO_PERI;
    state.dsky.flashing = false;
    state.servicer_exit = Some(p15_servicer_exit);

    p15_update(state);

    PRIORITY
}

/// Recompute the N44 display from the current `csm_state`.
pub fn p15_update(state: &mut crate::AgcState) {
    let elements = sv_to_elements(state.csm_state);

    if elements.is_hyperbolic() {
        state.alarm.code = ALARM_HYPERBOLIC;
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

/// SERVICER exit hook for continuous refresh.
pub fn p15_servicer_exit(state: &mut crate::AgcState) {
    p15_update(state);
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::gravity::R_EARTH;
    use crate::navigation::state_vector::StateVector;
    use crate::types::Met;
    use crate::AgcState;

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

    /// TC-P15-1: init on a 400 km circular LEO sets major_mode = 15 and
    /// populates the N44 display.
    #[test]
    fn tc_p15_1_circular_leo() {
        let mut state = AgcState::new();
        state.csm_state = circular_leo(400_000.0);

        let prio = init(&mut state);

        assert_eq!(prio, PRIORITY);
        assert_eq!(state.major_mode, P15_MAJOR_MODE);
        assert_eq!(state.dsky.prog, P15_MAJOR_MODE);
        assert_eq!(state.dsky.noun, NOUN_APO_PERI);

        let apo = state.dsky.r[0] as f64;
        let peri = state.dsky.r[1] as f64;
        assert!((apo - 400_000.0).abs() < 10.0);
        assert!((peri - 400_000.0).abs() < 10.0);
        assert!(state.servicer_exit.is_some());
    }

    /// TC-P15-2: MoonInertial frame raises alarm 236.
    #[test]
    fn tc_p15_2_wrong_frame_alarm() {
        let mut state = AgcState::new();
        state.csm_state = StateVector {
            position: [2_000_000.0, 0.0, 0.0],
            velocity: [0.0, 1_500.0, 0.0],
            epoch: Met(0),
            frame: Frame::MoonInertial,
        };

        init(&mut state);

        assert_eq!(state.alarm.code, ALARM_WRONG_FRAME);
        assert_ne!(state.major_mode, P15_MAJOR_MODE);
    }

    /// TC-P15-3: Hyperbolic trajectory raises alarm 237.
    #[test]
    fn tc_p15_3_hyperbolic_alarm() {
        let mut state = AgcState::new();
        let r = R_EARTH + 400_000.0;
        let v_escape = libm::sqrt(2.0 * MU_EARTH / r);
        state.csm_state = StateVector {
            position: [r, 0.0, 0.0],
            velocity: [0.0, 1.2 * v_escape, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        // Pre-seed a known display to verify it survives the alarm.
        state.dsky.r = [111.0, 222.0, 333.0];

        // init sets the prog/noun then calls p15_update which flags the alarm.
        init(&mut state);

        assert_eq!(state.alarm.code, ALARM_HYPERBOLIC);
        // Display must survive the alarm — p15_update returns early.
        assert_eq!(state.dsky.r, [111.0, 222.0, 333.0]);
    }
}
