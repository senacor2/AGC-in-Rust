//! P06 — CMC Power-down (standby mode).
//!
//! Quiesces every active task so the CMC can be placed into a low-power
//! standby state. Cancels the SERVICER, stops the DAP, clears any
//! pending maneuver, clears the burn state, and lights the STBY
//! indicator. The crew returns the CMC to operation with V37 E00 E
//! (P00) which clears STBY and restarts SERVICER as needed.
//!
//! AGC source: Comanche055/FRESH_START_AND_RESTART.agc — standby path.

use crate::control::dap::dap_stop;
use crate::executive::job::JobPriority;
use crate::services::average_g::stop_servicer;

/// Major mode number for P06.
pub const P06_MAJOR_MODE: u8 = 6;

/// Lowest job priority — P06 is passive once quiesced.
pub const PRIORITY: JobPriority = 1;

/// Entry point registered in `PROGRAM_TABLE[6]`.
pub fn init(state: &mut crate::AgcState) -> JobPriority {
    // Cancel the SERVICER cycle and detach any exit hook.
    stop_servicer(state);
    state.servicer_exit = None;

    // Stop the DAP (clears mode, staging fields, jet commands).
    dap_stop(state);

    // Drop any pending targeting result — it is no longer applicable.
    state.pending_maneuver = None;

    // Quench the SPS burn state.
    state.burn.burn_active = false;
    state.engine_thrusting = false;

    // Light the standby indicator.
    state.dsky.stby = true;
    state.dsky.prog = P06_MAJOR_MODE;
    state.dsky.verb = 37; // V37 prompt — the only valid next input
    state.dsky.noun = 0;
    state.dsky.flashing = false;
    state.dsky.comp_acty = false;

    state.major_mode = P06_MAJOR_MODE;

    PRIORITY
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::DapMode;
    use crate::guidance::maneuver::burn_servicer_exit;
    use crate::guidance::targeting::{Maneuver, TargetingMode};
    use crate::types::{DeltaV, Met};
    use crate::AgcState;

    /// TC-P06-1: `init` quiesces SERVICER, DAP, pending_maneuver, burn, and
    /// lights the STBY indicator.
    #[test]
    fn tc_p06_1_quiesces_all_activity() {
        let mut state = AgcState::new();

        // Seed an active configuration.
        state.dap_state.mode = DapMode::Tvc;
        state.servicer_exit = Some(burn_servicer_exit);
        state.pending_maneuver = Some(Maneuver {
            tig: Met(0),
            delta_v: DeltaV([50.0, 0.0, 0.0]),
            burn_attitude: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            mode: TargetingMode::ExternalDeltaV,
        });
        state.burn.burn_active = true;
        state.engine_thrusting = true;

        let prio = init(&mut state);

        assert_eq!(prio, PRIORITY);
        assert_eq!(prio, 1);
        assert_eq!(state.major_mode, P06_MAJOR_MODE);
        assert_eq!(state.dap_state.mode, DapMode::Off);
        assert!(state.servicer_exit.is_none());
        assert!(state.pending_maneuver.is_none());
        assert!(!state.burn.burn_active);
        assert!(!state.engine_thrusting);
        assert!(state.dsky.stby);
    }

    /// TC-P06-2: `init` is idempotent on a clean state.
    #[test]
    fn tc_p06_2_idempotent_on_clean_state() {
        let mut state = AgcState::new();
        init(&mut state);
        // Second call must not change anything unexpectedly.
        init(&mut state);
        assert_eq!(state.major_mode, P06_MAJOR_MODE);
        assert!(state.dsky.stby);
    }
}
