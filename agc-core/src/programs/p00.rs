//! P00 — CMC Idle.
//!
//! The lowest-priority background job. Maintains coasting state-vector
//! propagation when no active navigation program is running.

use crate::control::DapMode;
use crate::executive::job::JobPriority;

pub const PRIORITY: JobPriority = 1;

/// P00 — CMC Idle initialisation.
///
/// Called by the V37 program-select handler when the crew keys V37E00E, and
/// by the FRESH START sequence after clearing jobs and Waitlist tasks.
///
/// # Actions performed (in order)
/// 1. Sets `state.major_mode = 0`.
/// 2. Sets `state.burn.burn_active = false` and `state.engine_thrusting = false`
///    to cancel any active SPS burn.
/// 3. Clears `state.servicer_exit` (removes the P40 burn-exit callback if set).
/// 4. Transitions the DAP to `AttitudeHold` mode if it is not already `Off`.
///    Does not call `dap_init` — that would reset the CDU baseline.
/// 5. Sets `state.dsky.prog = 0` so the PROG indicator shows "00".
///
/// # Returns
/// `PRIORITY` (1).
pub fn init(state: &mut crate::AgcState) -> JobPriority {
    // Step 1: set major mode register to 00.
    state.major_mode = 0;

    // Step 2: cancel any active SPS burn.
    state.burn.burn_active = false;
    state.engine_thrusting = false;

    // Step 3: clear the burn-exit servicer hook.
    state.servicer_exit = None;

    // Step 4: transition DAP to AttitudeHold if it is currently running.
    // Do not call dap_init — that would reset the CDU baseline.
    if state.dap_state.mode != DapMode::Off {
        state.dap_state.mode = DapMode::AttitudeHold;
    }

    // Step 5: update the DSKY PROG field.
    state.dsky.prog = 0;

    PRIORITY
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AgcState;

    // TC-P00-1: init sets major_mode to 0.
    #[test]
    fn tc_p00_1_sets_major_mode_zero() {
        let mut state = AgcState::new();
        state.major_mode = 40;
        init(&mut state);
        assert_eq!(state.major_mode, 0);
    }

    // TC-P00-2: init returns PRIORITY (1).
    #[test]
    fn tc_p00_2_returns_low_priority() {
        let mut state = AgcState::new();
        let prio = init(&mut state);
        assert_eq!(prio, PRIORITY);
        assert_eq!(prio, 1);
    }

    // TC-P00-3: init cancels an active burn.
    #[test]
    fn tc_p00_3_cancels_active_burn() {
        fn dummy_exit_fn(_state: &mut AgcState) {}

        let mut state = AgcState::new();
        state.burn.burn_active = true;
        state.engine_thrusting = true;
        state.servicer_exit = Some(dummy_exit_fn);
        init(&mut state);
        assert!(!state.burn.burn_active);
        assert!(!state.engine_thrusting);
        assert!(state.servicer_exit.is_none());
    }

    // TC-P00-4: init leaves navigation state unchanged.
    #[test]
    fn tc_p00_4_leaves_nav_state_unchanged() {
        let mut state = AgcState::new();
        state.csm_state.position = [1.0e6, 2.0e6, 3.0e6];
        state.csm_state.velocity = [100.0, 200.0, 300.0];
        let pos_before = state.csm_state.position;
        let vel_before = state.csm_state.velocity;
        let refsmmat_before = state.refsmmat;
        init(&mut state);
        assert_eq!(state.csm_state.position, pos_before);
        assert_eq!(state.csm_state.velocity, vel_before);
        assert_eq!(state.refsmmat, refsmmat_before);
    }
}
