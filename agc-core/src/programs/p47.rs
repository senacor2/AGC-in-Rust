//! P47 — Thrust Monitor.
//!
//! Passive display-only program that shows the inertial delta-V the
//! SERVICER integrated during the most recent 2-second cycle. Used by
//! the crew to verify or characterise an uncommanded / non-nominal
//! thrust event. P47 never commands any actuator.
//!
//! AGC source: Comanche055/POWERED_FLIGHT_SUBROUTINES.agc — monitoring path.

use crate::executive::job::JobPriority;

pub const P47_MAJOR_MODE: u8 = 47;

/// Background monitor priority.
pub const PRIORITY: JobPriority = 6;

const VERB_MONITOR: u8 = 16;

/// DSKY noun for delta-V component readout (N83).
const NOUN_DV_COMPONENTS: u8 = 83;

/// Entry point registered in `PROGRAM_TABLE[47]`.
pub fn init(state: &mut crate::AgcState) -> JobPriority {
    state.major_mode = P47_MAJOR_MODE;
    state.dsky.prog = P47_MAJOR_MODE;
    state.dsky.verb = VERB_MONITOR;
    state.dsky.noun = NOUN_DV_COMPONENTS;
    state.dsky.flashing = false;
    state.servicer_exit = Some(p47_servicer_exit);

    // Populate the display with the last known delta-V so the crew sees
    // something immediately — the value is whatever the SERVICER staged
    // on its previous cycle (or zero at program start).
    p47_update(state);

    PRIORITY
}

/// Refresh the N83 delta-V-components display from
/// `state.servicer_last_dv_inertial`.
pub fn p47_update(state: &mut crate::AgcState) {
    let dv = state.servicer_last_dv_inertial;
    state.dsky.r[0] = dv[0] as f32;
    state.dsky.r[1] = dv[1] as f32;
    state.dsky.r[2] = dv[2] as f32;
}

/// SERVICER exit hook installed by P47.
pub fn p47_servicer_exit(state: &mut crate::AgcState) {
    p47_update(state);
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AgcState;

    /// TC-P47-1: init sets major_mode = 47, noun = 83, installs hook.
    #[test]
    fn tc_p47_1_init_sets_monitor() {
        let mut state = AgcState::new();
        let prio = init(&mut state);

        assert_eq!(prio, PRIORITY);
        assert_eq!(state.major_mode, P47_MAJOR_MODE);
        assert_eq!(state.dsky.prog, P47_MAJOR_MODE);
        assert_eq!(state.dsky.noun, NOUN_DV_COMPONENTS);
        assert_eq!(state.dsky.verb, VERB_MONITOR);
        assert!(state.servicer_exit.is_some());
    }

    /// TC-P47-2: staging a delta-V vector and invoking the hook populates the DSKY.
    #[test]
    fn tc_p47_2_servicer_exit_displays_dv() {
        let mut state = AgcState::new();
        init(&mut state);

        state.servicer_last_dv_inertial = [1.5, -0.7, 0.3];
        p47_servicer_exit(&mut state);

        assert!((state.dsky.r[0] - 1.5).abs() < 1e-6);
        assert!((state.dsky.r[1] - (-0.7)).abs() < 1e-6);
        assert!((state.dsky.r[2] - 0.3).abs() < 1e-6);
    }
}
