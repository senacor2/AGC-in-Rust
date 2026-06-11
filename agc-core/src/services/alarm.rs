//! Program alarm system — alarm state, POODOO, and GOTOPOOH recovery.
//!
//! Alarms are 4-digit codes displayed on the DSKY PROG register.
//! See `tables::alarm_codes` for the full code list.
//!
//! **POODOO** and **GOTOPOOH** are the AGC's two soft-abort recovery paths.
//! Both abort the active program and return to P00; they differ only in how
//! aggressively they clear erasable state.
//!
//! AGC source: `Comanche055/ALARM_AND_ABORT.agc`

/// Current alarm state.
#[derive(Clone, Copy, Debug, Default)]
pub struct AlarmState {
    /// The most recent alarm code (0 = none).
    pub code: u16,
    /// Secondary alarm code (stores the previous alarm when a new one fires).
    pub code2: u16,
    /// True when the PROG alarm lamp is lit.
    pub lit: bool,
}

impl AlarmState {
    /// Raise an alarm. Saves the current code to `code2` and lights the lamp.
    pub fn raise(&mut self, code: u16) {
        self.code2 = self.code;
        self.code = code;
        self.lit = true;
    }

    /// Clear the alarm lamp (crew pressed RSET).
    pub fn reset(&mut self) {
        self.lit = false;
    }
}

// ── POODOO ────────────────────────────────────────────────────────────────────

/// POODOO — hard soft-abort recovery.
///
/// Raised when a computation fails irrecoverably (imaginary roots, Lambert
/// non-convergence, state-vector out of range).  Sequence:
///
/// 1. Raise `alarm_code` (1410-class program alarm) and light the PROG lamp.
/// 2. Abort the active program: clear the job/task queues that belong to the
///    current major mode (via the flag-then-exit pattern — the scheduler
///    drains stale jobs naturally because `major_mode` changes).
/// 3. Force the display to P00 / idle state; the Executive loop re-enters P00.
/// 4. Preserve navigation state (`csm_state`, `refsmmat`, `time`,
///    `csm_nav`, `gha_epoch_rad`, `liftoff_time`).
///
/// Named after the NASA internal joke — "Plan Of Operation During Disastrous
/// Off-nominal Operations."
///
/// AGC source: `ALARM_AND_ABORT.agc` — `POODOO` label and `GOTOPOOH` branch.
/// Alarm codes: table in `ASSEMBLY_AND_OPERATION_INFORMATION.agc` §8.
pub fn poodoo(state: &mut crate::AgcState, alarm_code: u16) {
    // Step 1: raise the alarm.
    state.alarm.raise(alarm_code);

    // Step 2: abort — return to P00 without a full FRESH START.
    // Preserve navigation state; discard scheduler and guidance state.
    _return_to_p00(state);
}

/// GOTOPOOH — soft soft-abort recovery.
///
/// Raised when a non-fatal anomaly occurs (e.g. crew pressed RSET mid-mark,
/// V33 proceed in a non-input context, soft alarm).  Clears the active major
/// mode but does NOT raise a program alarm or re-initialise guidance state.
///
/// Sequence:
/// 1. Clear `major_mode` and `dsky.prog` to 0 (P00 idle).
/// 2. Clear `dsky.flashing` so the crew sees a stable display.
/// 3. Navigation state is preserved in full.
///
/// AGC source: `ALARM_AND_ABORT.agc` — `GOTOPOOH` label.
pub fn gotopooh(state: &mut crate::AgcState) {
    _return_to_p00(state);
}

// ── Internal helper ───────────────────────────────────────────────────────────

/// Common P00-return path shared by POODOO and GOTOPOOH.
///
/// Clears the scheduler (Executive + Waitlist), resets guidance/control to
/// safe defaults (DAP off, no pending maneuver, no burn), and sets the DSKY
/// to P00 idle.  Navigation state (`csm_state`, `target_state`, `refsmmat`,
/// `time`, `csm_nav`, `rendezvous_nav`, `gha_epoch_rad`, `liftoff_time`) is
/// left untouched.
fn _return_to_p00(state: &mut crate::AgcState) {
    use crate::executive::{Executive, Waitlist};

    // Clear the scheduler.
    state.executive = Executive::new();
    state.waitlist = Waitlist::new();

    // Guidance — clear the pending maneuver and burn state.
    state.pending_maneuver = None;
    state.burn = Default::default();
    state.servicer_exit = None;
    state.engine_thrusting = false;
    state.drogue_deploy_pending = false;
    state.csm_separation_pending = false;

    // DAP — stop attitude control.
    state.dap_state = Default::default();
    state.rcs_commanded_jets = 0;
    state.rcs_commanded_pulse_cs = 0;
    state.sps_gimbal_cmd = (0, 0);

    // DSKY — idle display.
    state.major_mode = 0;
    state.dsky.prog = 0;
    state.dsky.verb = 0;
    state.dsky.noun = 0;
    state.dsky.flashing = false;
    state.dsky.opr_err = false;
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::state_vector::{Frame, StateVector};
    use crate::navigation::gravity::MU_EARTH;
    use crate::types::Met;
    use crate::AgcState;

    fn leo_state() -> StateVector {
        let r = 6_778_137.0_f64;
        let v = libm::sqrt(MU_EARTH / r);
        StateVector {
            position: [r, 0.0, 0.0],
            velocity: [0.0, v, 0.0],
            epoch: Met(36_000_000),
            frame: Frame::EarthInertial,
        }
    }

    /// TC-ALARM-1: AlarmState::raise stacks codes and lights the lamp.
    #[test]
    fn tc_alarm_1_raise_stacks_and_lights() {
        let mut a = AlarmState::default();
        a.raise(0o1202);
        assert_eq!(a.code, 0o1202);
        assert_eq!(a.code2, 0);
        assert!(a.lit);

        a.raise(0o1211);
        assert_eq!(a.code, 0o1211);
        assert_eq!(a.code2, 0o1202);
        assert!(a.lit);
    }

    /// TC-ALARM-2: AlarmState::reset clears the lamp without erasing the code.
    #[test]
    fn tc_alarm_2_reset_clears_lamp_not_code() {
        let mut a = AlarmState::default();
        a.raise(0o1410);
        a.reset();
        assert!(!a.lit);
        assert_eq!(a.code, 0o1410);
    }

    /// TC-ALARM-3: poodoo ends in P00 with restart group consistent with spec.
    ///
    /// Induced alarm in P23 (major_mode = 23) via poodoo must result in:
    /// - major_mode == 0
    /// - alarm.code == the supplied code
    /// - alarm.lit == true
    /// - scheduler empty (Executive and Waitlist cleared)
    /// - navigation state preserved
    #[test]
    fn tc_alarm_3_poodoo_ends_in_p00_preserves_nav() {
        let mut state = AgcState::new();
        state.major_mode = 23;
        state.csm_state = leo_state();
        state.refsmmat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        state.time = Met(36_000_000);

        poodoo(&mut state, 0o1410);

        assert_eq!(state.major_mode, 0, "poodoo must return to P00");
        assert_eq!(state.dsky.prog, 0, "dsky.prog must reflect P00");
        assert_eq!(state.alarm.code, 0o1410, "alarm.code must be set");
        assert!(state.alarm.lit, "alarm.lit must be true");
        assert!(!state.dsky.flashing, "dsky.flashing must be cleared");
        assert!(state.pending_maneuver.is_none(), "pending_maneuver must be cleared");

        // Navigation state must survive.
        assert_eq!(state.csm_state.epoch, Met(36_000_000), "nav epoch must be preserved");
        assert_eq!(state.csm_state.frame, Frame::EarthInertial, "frame must be preserved");
        assert_eq!(state.time, Met(36_000_000), "time must be preserved");
    }

    /// TC-ALARM-4: gotopooh returns to P00 without raising an alarm.
    #[test]
    fn tc_alarm_4_gotopooh_no_alarm() {
        let mut state = AgcState::new();
        state.major_mode = 23;
        state.csm_state = leo_state();
        state.time = Met(36_000_000);

        let alarm_before = state.alarm.code;
        gotopooh(&mut state);

        assert_eq!(state.major_mode, 0, "gotopooh must return to P00");
        assert_eq!(state.alarm.code, alarm_before, "gotopooh must not raise a new alarm");
        assert!(!state.alarm.lit, "alarm.lit must not be set by gotopooh");

        // Navigation state must survive.
        assert_eq!(state.csm_state.epoch, Met(36_000_000), "nav epoch must be preserved");
        assert_eq!(state.time, Met(36_000_000), "time must be preserved");
    }

    /// TC-ALARM-5: poodoo inside a P22/P40 burn ends in P00.
    #[test]
    fn tc_alarm_5_poodoo_from_p40_burn() {
        let mut state = AgcState::new();
        state.major_mode = 40;
        state.engine_thrusting = true;
        state.dsky.flashing = true;
        state.csm_state = leo_state();

        poodoo(&mut state, 0o1411);

        assert_eq!(state.major_mode, 0, "poodoo in P40 must return to P00");
        assert!(!state.engine_thrusting, "engine must be stopped");
        assert_eq!(state.alarm.code, 0o1411, "alarm code must be set");
    }
}
