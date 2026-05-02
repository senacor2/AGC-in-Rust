use super::job::{JobEntry, JobPriority, MAX_JOBS};
use crate::hal::dsky::Dsky;
use crate::hal::engine::Engine;
use crate::hal::imu::Imu;
use crate::hal::rcs::Rcs;
use crate::hal::timers::Timers;
use crate::hal::AgcHardware;

/// The Executive — cooperative priority-based job scheduler.
///
/// Maintains a table of up to `MAX_JOBS` concurrent jobs. The `run` loop
/// scans for the highest-priority ready job, dispatches it, and repeats.
/// Only interrupts and waitlist tasks can preempt a running job.
pub struct Executive {
    pub(super) jobs: [JobEntry; MAX_JOBS],
    pub(super) current_priority: JobPriority,
}

impl Executive {
    #[allow(clippy::new_without_default)]
    pub const fn new() -> Self {
        Self {
            jobs: [JobEntry::EMPTY; MAX_JOBS],
            current_priority: 0,
        }
    }

    /// Create a new job. Returns `false` if all slots are occupied (alarm 1202;
    /// alarm 1210 if `has_vac` is true).
    ///
    /// Maps to both AGC NOVAC (`has_vac = false`) and FINDVAC (`has_vac = true`).
    pub fn create_job(
        &mut self,
        priority: JobPriority,
        entry: fn(&mut crate::AgcState),
        major_mode: u8,
        has_vac: bool,
    ) -> bool {
        for slot in &mut self.jobs {
            if slot.priority == 0 {
                *slot = JobEntry {
                    priority,
                    entry,
                    major_mode,
                    has_vac,
                };
                return true;
            }
        }
        false
    }

    /// Remove the job with the lowest priority (used during alarm 1202 recovery).
    pub fn drop_lowest_job(&mut self) {
        if let Some(slot) = self
            .jobs
            .iter_mut()
            .filter(|j| j.priority > 0)
            .min_by_key(|j| j.priority)
        {
            *slot = JobEntry::EMPTY;
        }
    }

    /// Main scheduler loop. Never returns.
    ///
    /// Drains the four AGC interrupt flags (T3/T4/T5/T6) in priority order,
    /// dispatches the highest-priority ready job, translates staging fields to
    /// HAL calls, and re-arms T3 lazily. The watchdog is petted at the top of
    /// every iteration so a hung job triggers a hardware restart (ADR-009).
    ///
    /// Signature is a free associated function (not `&mut self`) so the caller
    /// can pass the full `&mut AgcState` — `executive` lives inside `AgcState`,
    /// and a `&mut self` receiver would cause a split-borrow conflict when
    /// dispatching a job that mutates other fields of `state`.
    pub fn run<H: AgcHardware>(state: &mut crate::AgcState, hw: &mut H) -> ! {
        use crate::hal::runtime::{T3_PENDING, T4_PENDING, T5_PENDING, T6_PENDING};
        use core::sync::atomic::Ordering;

        // Arm T3 for any tasks already on the waitlist (e.g. from dap_init).
        if let Some(cs) = state.waitlist.front_delta() {
            hw.timers().arm_t3(cs);
        }

        // Track the last T3 reload value so we skip redundant arm_t3 register
        // writes when the waitlist front hasn't changed since the last arm.
        let mut last_armed_t3: Option<u16> = state.waitlist.front_delta();

        loop {
            hw.pet_watchdog();

            // ── Drain ISR-posted flags in AGC priority order ──────────────────
            // (T6 highest, then T5, T3, T4)

            // T6RUPT: RCS jet quench (highest AGC priority).
            if T6_PENDING.swap(false, Ordering::Acquire) {
                hw.rcs().quench_all();
            }

            // T5RUPT: DAP runs via the waitlist on T3 in this port (ADR-018).
            if T5_PENDING.swap(false, Ordering::Acquire) {
                // No flight code uses T5 directly — placeholder for future use.
            }

            // T3RUPT: pre-read CDU, dispatch one waitlist task, re-arm T3.
            if T3_PENDING.swap(false, Ordering::Acquire) {
                // Fresh CDU snapshot before any waitlist task runs; dap_step reads
                // state.current_cdu rather than calling hw.imu() directly (Strategy D).
                // PIPA is NOT read here — destructive semantics require the 2-second
                // SERVICER accumulation pipeline (next milestone).
                state.current_cdu = hw.imu().read_cdu();

                // pop_task avoids the split-borrow conflict: borrowing
                // state.waitlist mutably while also passing &mut state is
                // rejected by the borrow checker, so the task fn pointer is
                // extracted first and then called separately.
                if let Some((task, next_delta)) = state.waitlist.pop_task() {
                    task(state);
                    if let Some(cs) = next_delta {
                        hw.timers().arm_t3(cs);
                        last_armed_t3 = Some(cs);
                    } else {
                        last_armed_t3 = None;
                    }
                }
            }

            // T4RUPT: advance MET by 12 cs (120 ms) and apply gyro drift.
            if T4_PENDING.swap(false, Ordering::Acquire) {
                state.time = crate::types::Met(state.time.0.wrapping_add(12));

                let dt_cs = state.time.elapsed_since(state.last_drift_comp_time);
                if dt_cs > 0 {
                    let nbd = [
                        state.gyro_comp.nbdx,
                        state.gyro_comp.nbdy,
                        state.gyro_comp.nbdz,
                    ];
                    let pulses = crate::control::imu_control::compute_gyro_drift(dt_cs, nbd);
                    for (axis, &p) in pulses.iter().enumerate() {
                        if p != 0 {
                            hw.imu().torque_gyro(axis, p);
                        }
                    }
                    state.last_drift_comp_time = state.time;
                }
                // DSKY display emission is deferred — the row-encoding design
                // is a separate milestone.
            }

            // ── Drain DSKY keyqueue → V/N state machine ───────────────────────
            while let Some(code) = hw.dsky().read_key() {
                if let Some(key) = crate::services::v_n::Key::from_code(code) {
                    crate::services::v_n::feed_key(state, key);
                }
            }

            // ── Dispatch one Executive job ────────────────────────────────────
            if let Some(idx) = state.executive.find_highest_priority_job() {
                let entry = state.executive.jobs[idx].entry;
                state.executive.current_priority = state.executive.jobs[idx].priority;
                (entry)(state);
                state.executive.jobs[idx] = JobEntry::EMPTY;
                state.executive.current_priority = 0;
            }

            // ── Translate staging fields written by tasks/jobs to HAL calls ───
            process_rcs_staging(state, hw);
            process_engine_staging(state, hw);

            // ── Lazy T3 re-arm: only write timer registers if the front changed.
            let new_front = state.waitlist.front_delta();
            if new_front != last_armed_t3 {
                if let Some(cs) = new_front {
                    hw.timers().arm_t3(cs);
                }
                last_armed_t3 = new_front;
            }
        }
    }

    /// Scan for the highest-priority occupied slot.
    /// Tie-breaking: on equal priority, the lower index wins (spec §5.1).
    pub(crate) fn find_highest_priority_job(&self) -> Option<usize> {
        let mut best: Option<usize> = None;
        let mut best_pri: JobPriority = 0;
        for (i, j) in self.jobs.iter().enumerate() {
            if j.priority > best_pri {
                best_pri = j.priority;
                best = Some(i);
            }
        }
        best
    }

    /// Read the priority of the currently dispatched job (0 if idle).
    pub fn current_priority(&self) -> JobPriority {
        self.current_priority
    }
}

/// Apply any RCS jet command staged by dap_step.
///
/// Converts `rcs_commanded_pulse_cs` (centiseconds) to T6 counts
/// (1 count = 0.625 ms → cs × 10 ms / 0.625 ms = cs × 16) and fires the jets,
/// then arms T6 to quench them after the pulse duration.
fn process_rcs_staging<H: AgcHardware>(state: &mut crate::AgcState, hw: &mut H) {
    if state.rcs_commanded_jets != 0 && state.rcs_commanded_pulse_cs != 0 {
        let jets_a = (state.rcs_commanded_jets & 0xFF) as u8;
        let jets_b = ((state.rcs_commanded_jets >> 8) & 0xFF) as u8;
        hw.rcs().fire_sm_jets(jets_a, jets_b);
        let counts = (state.rcs_commanded_pulse_cs as u32).saturating_mul(16);
        let counts_clamped = counts.min(u16::MAX as u32) as u16;
        hw.timers().arm_t6(counts_clamped);
        state.rcs_commanded_jets = 0;
        state.rcs_commanded_pulse_cs = 0;
    }
}

/// Apply the SPS engine state staged by dap_step / P40.
fn process_engine_staging<H: AgcHardware>(state: &mut crate::AgcState, hw: &mut H) {
    if state.engine_thrusting {
        hw.engine().sps_enable(true);
        let (pitch, yaw) = state.sps_gimbal_cmd;
        hw.engine().sps_gimbal(pitch, yaw);
    } else {
        hw.engine().sps_enable(false);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dummy_job(_state: &mut crate::AgcState) {}
    fn dummy_job_b(_state: &mut crate::AgcState) {}

    // TC-CJ-1: create_job into empty table
    #[test]
    fn tc_cj_1_create_into_empty() {
        let mut exec = Executive::new();
        assert!(exec.create_job(20, dummy_job, 0, false));
        assert_eq!(exec.jobs[0].priority, 20);
    }

    // TC-CJ-2: create_job with 6 occupied
    #[test]
    fn tc_cj_2_create_with_6_occupied() {
        let mut exec = Executive::new();
        for i in 1..=6 {
            assert!(exec.create_job(i, dummy_job, 0, false));
        }
        assert!(exec.create_job(7, dummy_job, 0, false));
    }

    // TC-CJ-3: create_job when full returns false
    #[test]
    fn tc_cj_3_create_when_full() {
        let mut exec = Executive::new();
        for i in 1..=7 {
            assert!(exec.create_job(i, dummy_job, 0, false));
        }
        assert!(!exec.create_job(5, dummy_job, 0, false));
    }

    // TC-CJ-4: create_job picks first empty slot
    #[test]
    fn tc_cj_4_first_empty_slot() {
        let mut exec = Executive::new();
        exec.create_job(1, dummy_job, 0, false);
        exec.create_job(1, dummy_job_b, 0, false);
        assert_eq!(exec.jobs[0].priority, 1);
        assert_eq!(exec.jobs[1].priority, 1);
    }

    // TC-DL-1: drop_lowest_job removes minimum priority
    #[test]
    fn tc_dl_1_drop_minimum() {
        let mut exec = Executive::new();
        exec.create_job(10, dummy_job, 0, false);
        exec.create_job(20, dummy_job, 0, false);
        exec.create_job(5, dummy_job, 0, false);
        exec.create_job(30, dummy_job, 0, false);
        exec.create_job(15, dummy_job, 0, false);
        exec.drop_lowest_job();
        // Slot 2 (priority 5) should be cleared
        assert_eq!(exec.jobs[2].priority, 0);
        assert_eq!(exec.jobs[0].priority, 10);
    }

    // TC-DL-2: drop_lowest_job on empty table is no-op
    #[test]
    fn tc_dl_2_drop_empty() {
        let mut exec = Executive::new();
        exec.drop_lowest_job(); // must not panic
    }

    // TC-DL-3: drop_lowest_job tie-breaking by lowest index
    #[test]
    fn tc_dl_3_drop_tie_lowest_index() {
        let mut exec = Executive::new();
        exec.create_job(10, dummy_job, 0, false);
        exec.create_job(10, dummy_job, 0, false);
        exec.create_job(10, dummy_job, 0, false);
        exec.drop_lowest_job();
        // Should drop slot 0 (lowest index on tie)
        assert_eq!(exec.jobs[0].priority, 0);
        assert_eq!(exec.jobs[1].priority, 10);
        assert_eq!(exec.jobs[2].priority, 10);
    }

    // TC-FIND: find_highest_priority_job tie-breaking by lowest index
    #[test]
    fn find_highest_tiebreak_lowest_index() {
        let mut exec = Executive::new();
        exec.create_job(10, dummy_job, 0, false); // slot 0
        exec.create_job(20, dummy_job, 0, false); // slot 1
        exec.create_job(20, dummy_job_b, 0, false); // slot 2 (same priority)
        assert_eq!(exec.find_highest_priority_job(), Some(1)); // lowest index wins
    }
}
