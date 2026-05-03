//! FRESH START and RESTART sequences.
//!
//! **FRESH START**: Full re-initialisation (power-on or crew-initiated).
//! Zeroes all state including navigation, establishes P00 idle, and enters
//! the Executive scheduling loop.
//!
//! **RESTART**: Recovery after a watchdog timeout, parity error, or
//! software-initiated GOJAM. Preserves navigation state (CSM/target state
//! vectors, REFSMMAT, MET). Clears the scheduler, then re-dispatches active
//! restart groups from their saved phase registers.
//!
//! See `specs/executive-spec.md` §5.4 for the full restart sequence and the
//! FRESH START vs RESTART comparison table.

use crate::executive::restart::{Phase, NUM_RESTART_GROUPS};
use crate::executive::scheduler::Executive;
use crate::executive::waitlist::Waitlist;
use crate::math::linalg::IDENTITY;
use crate::navigation::state_vector::StateVector;
use crate::services::alarm::AlarmState;
use crate::services::display::DskyState;
use crate::AgcState;

/// Perform a FRESH START — complete re-initialisation.
///
/// Zeroes everything: navigation state, scheduler, alarms, display, flags.
/// Sets major mode to P00. The caller must enter the Executive loop
/// (`Executive::run(state, hw)`) after this returns.
pub fn fresh_start(state: &mut AgcState) {
    // Navigation state — zeroed (no valid data after power-on).
    state.csm_state = StateVector::ZERO;
    state.target_state = StateVector::ZERO;
    state.refsmmat = IDENTITY;
    state.time = crate::types::Met(0);

    // Scheduler — clear everything.
    state.executive = Executive::new();
    state.waitlist = Waitlist::new();

    // Restart groups — all idle (nothing to re-dispatch on a fresh start).
    state.restart = crate::executive::RestartProtection::new();

    // Guidance and control — off.
    state.major_mode = 0; // P00
    state.dap_state = Default::default();
    state.tvc_state = Default::default();

    // Display — reset.
    state.dsky = DskyState {
        prog: 0,
        verb: 0,
        noun: 0,
        r: [0.0; 3],
        flashing: false,
        uplink_activity: false,
        no_att: false,
        stby: false,
        key_rel: false,
        opr_err: false,
        restart_flag: false,
        gimbal_lock: false,
        temp: false,
        prog_alarm: false,
        comp_acty: false,
        tracker: false,
        lamp_test_active: false,
    };

    // Alarms — cleared.
    state.alarm = AlarmState {
        code: 0,
        code2: 0,
        lit: false,
    };

    // Flags — all cleared.
    state.flagwords = [0u16; 12];

    // Rendezvous navigation state — reset.
    state.rendezvous_nav = Default::default();

    // Landmark tracking navigation state — reset.
    state.csm_nav = Default::default();

    // gha_epoch_rad is intentionally NOT reset here.
    // It is an uplink value set by Mission Control prior to orbital insertion
    // and must survive FRESH START (Override 2).

    // TPI/TPM arrival epoch — reset on FRESH START (no active rendezvous).
    state.tpi_arrival_epoch = None;
}

/// Restart group dispatch entry: a function pointer + default priority/delay.
///
/// Programs register their restart handlers by populating this table. During
/// development, all entries are `None` (no groups registered). As programs
/// are implemented, they set their group's entry.
pub struct RestartGroupEntry {
    /// Entry point for re-dispatching this group as a job.
    pub job_entry: Option<fn(&mut AgcState)>,
    /// Default priority for re-created jobs.
    pub job_priority: u8,
    /// Entry point for re-dispatching this group as a task.
    pub task_entry: Option<fn(&mut AgcState)>,
    /// Default delay (centiseconds) for re-scheduled tasks.
    pub task_delay: u16,
    /// Major mode that owns this restart group.
    pub major_mode: u8,
}

impl RestartGroupEntry {
    pub const EMPTY: Self = Self {
        job_entry: None,
        job_priority: 0,
        task_entry: None,
        task_delay: 1,
        major_mode: 0,
    };
}

/// Restart group dispatch table. Indexed by GROUP_1..GROUP_6.
/// Programs populate this at startup; the restart sequence reads it.
pub static mut RESTART_GROUP_TABLE: [RestartGroupEntry; NUM_RESTART_GROUPS] = [
    RestartGroupEntry::EMPTY,
    RestartGroupEntry::EMPTY,
    RestartGroupEntry::EMPTY,
    RestartGroupEntry::EMPTY,
    RestartGroupEntry::EMPTY,
    RestartGroupEntry::EMPTY,
];

/// Perform a RESTART — recover from a hardware restart while preserving
/// navigation state.
///
/// Preserves: `csm_state`, `target_state`, `refsmmat`, `time`, `major_mode`.
/// Clears: scheduler (executive + waitlist), then re-dispatches active restart
/// groups based on their phase registers.
///
/// # Safety
/// Reads `RESTART_GROUP_TABLE` which is `static mut`. Safe because the AGC
/// is single-threaded and this function is called only from the reset vector
/// with interrupts disabled.
pub fn restart(state: &mut AgcState) {
    // Navigation state is PRESERVED — do not touch csm_state, target_state,
    // refsmmat, or time.

    // Clear scheduler — all jobs and tasks are lost; restart groups re-create them.
    state.executive = Executive::new();
    state.waitlist = Waitlist::new();

    // Guidance/control state — reset to safe defaults.
    state.dap_state = Default::default();
    state.tvc_state = Default::default();

    // Display — light the RESTART indicator.
    state.dsky.restart_flag = true;
    state.dsky.flashing = false;
    state.dsky.opr_err = false;

    // Alarm — preserve existing code but do not clear.

    // Re-dispatch active restart groups from their saved phases.
    for group in 0..NUM_RESTART_GROUPS {
        let phase = state.restart.phase(group);
        if phase.is_idle() {
            continue;
        }

        // Safety: single-threaded, interrupts disabled during restart.
        let entry = unsafe { &RESTART_GROUP_TABLE[group] };

        if phase.is_job() {
            // Positive even phase → re-create as Executive job.
            if let Some(job_fn) = entry.job_entry {
                state
                    .executive
                    .create_job(entry.job_priority, job_fn, entry.major_mode);
            }
        } else if phase.is_task() {
            // Positive odd phase → re-schedule as Waitlist task.
            if let Some(task_fn) = entry.task_entry {
                state.waitlist.schedule(entry.task_delay, task_fn);
            }
        } else {
            // Negative phase → restart group from top. Use job entry if available,
            // otherwise task entry, to re-enter the computation from scratch.
            if let Some(job_fn) = entry.job_entry {
                state
                    .executive
                    .create_job(entry.job_priority, job_fn, entry.major_mode);
            } else if let Some(task_fn) = entry.task_entry {
                state.waitlist.schedule(entry.task_delay, task_fn);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::executive::restart::{GROUP_1, GROUP_3, GROUP_5};
    use crate::navigation::state_vector::{Frame, StateVector};
    use crate::types::Met;

    // TC-FS-1: fresh_start zeroes navigation state
    #[test]
    fn tc_fs_1_zeroes_nav_state() {
        let mut state = AgcState::new();
        state.csm_state.position = [1e6, 2e6, 3e6];
        state.csm_state.velocity = [100.0, 200.0, 300.0];
        state.time = Met(999999);
        state.major_mode = 40;

        fresh_start(&mut state);

        assert_eq!(state.csm_state.position, [0.0; 3]);
        assert_eq!(state.csm_state.velocity, [0.0; 3]);
        assert_eq!(state.time, Met(0));
        assert_eq!(state.major_mode, 0); // P00
    }

    // TC-FS-2: fresh_start clears all restart phases
    #[test]
    fn tc_fs_2_clears_phases() {
        let mut state = AgcState::new();
        state.restart.set_phase(GROUP_1, Phase::new(4));
        state.restart.set_phase(GROUP_3, Phase::new(3));

        fresh_start(&mut state);

        for i in 0..NUM_RESTART_GROUPS {
            assert!(state.restart.phase(i).is_idle());
        }
    }

    // TC-FS-3: fresh_start clears alarms
    #[test]
    fn tc_fs_3_clears_alarms() {
        let mut state = AgcState::new();
        state.alarm.raise(1202);

        fresh_start(&mut state);

        assert_eq!(state.alarm.code, 0);
        assert!(!state.alarm.lit);
    }

    // TC-RS-1: restart preserves navigation state
    #[test]
    fn tc_rs_1_preserves_nav() {
        let mut state = AgcState::new();
        state.csm_state = StateVector {
            position: [1e6, 2e6, 3e6],
            velocity: [100.0, 200.0, 300.0],
            epoch: Met(50000),
            frame: Frame::EarthInertial,
        };
        state.time = Met(50000);
        state.major_mode = 23;
        let saved_pos = state.csm_state.position;
        let saved_vel = state.csm_state.velocity;
        let saved_time = state.time;
        let saved_mm = state.major_mode;

        restart(&mut state);

        assert_eq!(state.csm_state.position, saved_pos);
        assert_eq!(state.csm_state.velocity, saved_vel);
        assert_eq!(state.time, saved_time);
        assert_eq!(state.major_mode, saved_mm);
    }

    // TC-RS-2: restart lights RESTART indicator
    #[test]
    fn tc_rs_2_restart_indicator() {
        let mut state = AgcState::new();
        restart(&mut state);
        assert!(state.dsky.restart_flag);
    }

    // TC-RS-3: restart clears scheduler
    #[test]
    fn tc_rs_3_clears_scheduler() {
        let mut state = AgcState::new();
        fn dummy(_: &mut AgcState) {}
        state.executive.create_job(10, dummy, 0);
        state.waitlist.schedule(100, dummy);

        restart(&mut state);

        // Executive and waitlist should be empty (before re-dispatch).
        // But re-dispatch may have added entries from phases.
        // With all phases IDLE (default), nothing is re-dispatched.
        assert!(state.waitlist.is_empty());
    }

    // TC-RS-4: restart re-dispatches active groups
    #[test]
    fn tc_rs_4_redispatch_groups() {
        let mut state = AgcState::new();
        fn group3_job(state: &mut AgcState) {
            let _ = state;
        }
        fn group5_task(state: &mut AgcState) {
            let _ = state;
        }

        // Set up restart group entries.
        unsafe {
            RESTART_GROUP_TABLE[GROUP_3] = RestartGroupEntry {
                job_entry: Some(group3_job),
                job_priority: 15,
                task_entry: None,
                task_delay: 1,
                major_mode: 30,
            };
            RESTART_GROUP_TABLE[GROUP_5] = RestartGroupEntry {
                job_entry: None,
                job_priority: 0,
                task_entry: Some(group5_task),
                task_delay: 200,
                major_mode: 40,
            };
        }

        // Set phases: GROUP_3 = positive even (job), GROUP_5 = positive odd (task).
        state.restart.set_phase(GROUP_3, Phase::new(2));
        state.restart.set_phase(GROUP_5, Phase::new(1));

        restart(&mut state);

        // GROUP_3 should have created a job.
        // GROUP_5 should have scheduled a task.
        assert!(!state.waitlist.is_empty()); // task from GROUP_5

        // Clean up static state for other tests.
        unsafe {
            RESTART_GROUP_TABLE[GROUP_3] = RestartGroupEntry::EMPTY;
            RESTART_GROUP_TABLE[GROUP_5] = RestartGroupEntry::EMPTY;
        }
    }
}
