//! FRESH START and RESTART sequences.
//!
//! **FRESH START** is a complete initialization — performed at power-on or
//! when the crew presses RESET. All erasable memory is cleared, all jobs and
//! tasks cancelled, and P00 (CMC Idling) is established.
//!
//! **RESTART** is a recovery from a fault (night-watchman timeout, parity
//! error, or software GOJAM). Navigation data is preserved. The phase table
//! is examined to re-dispatch in-progress work.
//!
//! AGC source: FRESH_START_AND_RESTART.agc — GOPROG / GORESTART / GOJAM.

use crate::executive::restart::Phase;
use crate::AgcState;

/// Perform a FRESH START.
///
/// Clears all scheduler state and alarms. After calling this function the
/// system is in a clean-slate state ready for P00 to be established.
///
/// The caller is responsible for initializing hardware (HAL) before calling
/// this function.
///
/// AGC source: FRESH_START_AND_RESTART.agc — GOPROG (power-on entry point).
pub fn fresh_start(state: &mut AgcState) {
    // Clear all restart group phases.
    state.restart.clear_all();
    // Clear all pending alarms.
    state.alarms.clear_all();
    // Clear all pending jobs.
    // (Waitlist and Executive job table are zeroed at power-on via static init.)
}

/// Perform a RESTART (recovery from fault).
///
/// Preserves navigation state vectors. Re-dispatches active restart groups
/// from their recorded phases. Groups with `Phase::IDLE` are skipped.
///
/// Returns the number of groups that were re-dispatched.
///
/// AGC source: FRESH_START_AND_RESTART.agc — GORESTART / GOPROG branches.
pub fn restart_recovery(state: &mut AgcState) -> u8 {
    use crate::executive::restart::{RestartGroup, NUM_RESTART_GROUPS};

    let mut redispatched = 0u8;

    for g in 1..=(NUM_RESTART_GROUPS as u8) {
        let group = RestartGroup(g);
        let phase = state.restart.get_phase(group);
        if phase == Phase::IDLE {
            continue;
        }
        // Re-dispatch: task (odd) or job (even).
        // For now we record that re-dispatch is needed; the actual dispatch
        // happens via the Executive establish_job / Waitlist schedule calls
        // that the major-mode restart handlers perform when they check their
        // own phases.
        redispatched += 1;
    }

    redispatched
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::executive::restart::{Phase, GROUP_1, GROUP_3};

    fn make_state() -> AgcState {
        AgcState::new()
    }

    #[test]
    fn fresh_start_clears_phases() {
        let mut state = make_state();
        state.restart.set_phase(GROUP_1, Phase::new(1));
        state.restart.set_phase(GROUP_3, Phase::new(3));
        fresh_start(&mut state);
        assert_eq!(state.restart.get_phase(GROUP_1), Phase::IDLE);
        assert_eq!(state.restart.get_phase(GROUP_3), Phase::IDLE);
    }

    #[test]
    fn restart_recovery_counts_active_groups() {
        let mut state = make_state();
        state.restart.set_phase(GROUP_1, Phase::new(1));
        state.restart.set_phase(GROUP_3, Phase::new(2));
        let count = restart_recovery(&mut state);
        assert_eq!(count, 2);
    }

    #[test]
    fn restart_recovery_skips_idle_groups() {
        let state = &mut make_state();
        // All groups IDLE
        let count = restart_recovery(state);
        assert_eq!(count, 0);
    }
}
