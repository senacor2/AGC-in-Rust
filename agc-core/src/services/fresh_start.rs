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

use crate::executive::restart::NUM_RESTART_GROUPS;
use crate::executive::scheduler::Executive;
use crate::executive::waitlist::Waitlist;
use crate::AgcState;

/// Perform a FRESH START — complete re-initialisation.
///
/// Replaces every field of `state` with the canonical zero defaults from
/// [`AgcState::new`], then re-injects the small set of fields documented to
/// survive FRESH START. The caller must enter the Executive loop
/// (`Executive::run(state, hw)`) after this returns.
///
/// # Survives FRESH START
///
/// The following fields are preserved across a FRESH START because they are
/// uplink values from Mission Control that cannot be reconstructed on board
/// (Override 2 in the spec). All other fields are zeroed unconditionally.
///
/// - `gha_epoch_rad` — Greenwich Hour Angle at the navigation epoch
///   (AGC erasable `GHABASE`).
///
/// When adding a new field to `AgcState` that must also survive FRESH START,
/// extend the save/restore block below AND add it to this list. Anything not
/// listed here is wiped — that is the deliberate auditability property of
/// this function. Field-by-field reset (the previous implementation) had a
/// failure mode where new fields silently leaked stale state across a FRESH
/// START until someone noticed in production.
pub fn fresh_start(state: &mut AgcState) {
    // Step 1: snapshot the survives-FRESH-START fields.
    let saved_gha_epoch_rad = state.gha_epoch_rad;

    // Step 2: replace the entire state with canonical zero defaults. This
    // guarantees no field is forgotten; new fields added to AgcState
    // automatically get scrubbed unless the author adds them to the survive
    // list above.
    *state = AgcState::new();

    // Step 3: re-inject the saved fields.
    state.gha_epoch_rad = saved_gha_epoch_rad;
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
    // Safety: single-threaded, interrupts disabled during restart, and we only
    // take a shared reference for read-only iteration.
    let table: &[RestartGroupEntry; NUM_RESTART_GROUPS] =
        unsafe { &*core::ptr::addr_of!(RESTART_GROUP_TABLE) };
    for (group, entry) in table.iter().enumerate() {
        let phase = state.restart.phase(group);
        if phase.is_idle() {
            continue;
        }

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
    use crate::executive::restart::{Phase, GROUP_1, GROUP_3, GROUP_5};
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

    // TC-FS-4: fresh_start clears burn / engine / TVC staging fields.
    //
    // These were silently leaking before the audit-driven rewrite. A stale
    // `engine_thrusting = true` after FRESH START would leave SPS commanded
    // on; a stale `servicer_exit` would let the previous burn's exit hook
    // fire on the next SERVICER cycle.
    #[test]
    fn tc_fs_4_clears_burn_and_engine_staging() {
        fn dummy_exit(_: &mut AgcState) {}

        let mut state = AgcState::new();
        state.burn.burn_active = true;
        state.burn.target_dv_inertial = [10.0, 20.0, 30.0];
        state.engine_thrusting = true;
        state.servicer_exit = Some(dummy_exit);
        state.sps_gimbal_cmd = (123, -456);
        state.rcs_commanded_jets = 0xDEAD;
        state.rcs_commanded_pulse_cs = 99;

        fresh_start(&mut state);

        assert!(!state.burn.burn_active, "burn_active must be cleared");
        assert_eq!(state.burn.target_dv_inertial, [0.0; 3]);
        assert!(
            !state.engine_thrusting,
            "engine_thrusting must be cleared (otherwise SPS stays armed)"
        );
        assert!(
            state.servicer_exit.is_none(),
            "servicer_exit must be cleared (otherwise stale callback fires)"
        );
        assert_eq!(state.sps_gimbal_cmd, (0, 0));
        assert_eq!(state.rcs_commanded_jets, 0);
        assert_eq!(state.rcs_commanded_pulse_cs, 0);
    }

    // TC-FS-5: fresh_start clears crew-input + entry-phase state.
    //
    // A leftover `pending_v50` would leave a "press PROCEED" prompt armed
    // for an action belonging to a previous mission phase; a leftover
    // entry phase would mis-cue P61–P67.
    #[test]
    fn tc_fs_5_clears_vn_and_entry() {
        use crate::programs::p61_p67::EntryPhase;
        use crate::services::v_n::Pending50;

        fn dummy_proceed(_: &mut AgcState) {}

        let mut state = AgcState::new();
        state.vn.pending_v50 = Some(Pending50 {
            noun: 33,
            on_proceed: dummy_proceed,
        });
        state.entry.phase = EntryPhase::Entry;
        state.entry.drogue_deployed = true;
        state.entry.roll_command_rad = 0.5;

        fresh_start(&mut state);

        assert!(
            state.vn.pending_v50.is_none(),
            "pending_v50 must be cleared (no stale PROCEED prompt)"
        );
        assert_eq!(
            state.entry.phase,
            EntryPhase::Idle,
            "entry phase must reset to Idle"
        );
        assert!(!state.entry.drogue_deployed);
        assert_eq!(state.entry.roll_command_rad, 0.0);
    }

    // TC-FS-6: fresh_start clears IMU alignment + PIPA staging.
    #[test]
    fn tc_fs_6_clears_imu_and_pipa_staging() {
        use crate::control::imu_control::ImuAlignmentState;

        let mut state = AgcState::new();
        state.imu_alignment_state = ImuAlignmentState::FineAligned;
        state.pipa_counts = [123, -456, 789];
        state.servicer_last_dv_inertial = [1.0, 2.0, 3.0];
        state.last_drift_comp_time = Met(50_000);

        fresh_start(&mut state);

        assert_eq!(
            state.imu_alignment_state,
            ImuAlignmentState::Caged,
            "IMU alignment must reset to Caged after FRESH START"
        );
        assert_eq!(state.pipa_counts, [0; 3]);
        assert_eq!(state.servicer_last_dv_inertial, [0.0; 3]);
        assert_eq!(state.last_drift_comp_time, Met(0));
    }

    // TC-FS-7: fresh_start preserves the documented "survives" set.
    //
    // The only field currently documented to survive FRESH START is
    // `gha_epoch_rad` (Greenwich Hour Angle uplink — Mission Control sets
    // it once before orbital insertion). If this list grows in the future,
    // extend both `fresh_start()` and this test together.
    #[test]
    fn tc_fs_7_preserves_gha_epoch_rad() {
        let mut state = AgcState::new();
        state.gha_epoch_rad = 1.234_567_8;
        // Pollute the rest so we can be sure they were scrubbed.
        state.csm_state.position = [9e6, 9e6, 9e6];
        state.major_mode = 40;

        fresh_start(&mut state);

        assert_eq!(
            state.gha_epoch_rad, 1.234_567_8,
            "gha_epoch_rad (uplink value) must survive FRESH START"
        );
        // Sanity: other state was scrubbed.
        assert_eq!(state.csm_state.position, [0.0; 3]);
        assert_eq!(state.major_mode, 0);
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
