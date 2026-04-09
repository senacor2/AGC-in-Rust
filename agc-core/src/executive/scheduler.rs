//! Executive scheduler: CORE SET job table and main EXEC loop.
//!
//! The Executive scans the job table on every iteration for the highest-
//! priority ready job and dispatches it. When no jobs are ready it loops
//! ("dummy job"), petting the watchdog each iteration.
//!
//! Priority inversion and the 1202 alarm (Executive overflow) are handled
//! here: if `novac` or `findvac` cannot find an empty slot, alarm 1202 fires.
//!
//! AGC source: EXECUTIVE.agc — EXEC main loop, NOVAC/FINDVAC, CHANG1.

use super::{job::JobEntry, waitlist::Waitlist, MAX_JOBS};
use crate::hal::AgcHardware;
use crate::services::alarm::{AlarmCode, AlarmState};
use crate::AgcState;

/// The Executive: job table + Waitlist.
///
/// AGC source: EXECUTIVE.agc — EXEC loop, JOBSLIST.
pub struct Executive {
    /// CORE SET job table. Slot 0 is the lowest priority reserved dummy.
    jobs: [Option<JobEntry>; MAX_JOBS],
    /// The Waitlist delta-time chain.
    pub waitlist: Waitlist,
}

impl Default for Executive {
    fn default() -> Self {
        Self::new()
    }
}

impl Executive {
    pub const fn new() -> Self {
        Self {
            jobs: [None; MAX_JOBS],
            waitlist: Waitlist::new(),
        }
    }

    /// Establish a new job (NOVAC/FINDVAC).
    ///
    /// Finds an empty slot and inserts the job. Returns the slot index on
    /// success. If no slots are available, raises alarm 1202 and returns
    /// `None`.
    ///
    /// AGC source: EXECUTIVE.agc — NOVAC/FINDVAC routines.
    pub fn establish_job(
        &mut self,
        entry: fn(*mut AgcState),
        priority: u8,
        major_mode: u8,
        alarms: &mut AlarmState,
    ) -> Option<usize> {
        for (i, slot) in self.jobs.iter_mut().enumerate() {
            if slot.is_none() {
                *slot = Some(JobEntry::new(priority, entry, major_mode));
                return Some(i);
            }
        }
        // No empty slot — Executive overflow alarm.
        alarms.raise(AlarmCode::ExecutiveOverflow);
        None
    }

    /// Remove a completed job from the table.
    ///
    /// AGC source: EXECUTIVE.agc — JOBOVER: clear slot on job completion.
    pub fn complete_job(&mut self, slot: usize) {
        if slot < MAX_JOBS {
            self.jobs[slot] = None;
        }
    }

    /// Find the slot index of the highest-priority ready job.
    ///
    /// AGC source: EXECUTIVE.agc — EXEC priority scan.
    fn find_highest_priority_job(&self) -> Option<usize> {
        let mut best: Option<(usize, u8)> = None;
        for (i, slot) in self.jobs.iter().enumerate() {
            if let Some(job) = slot {
                if job.priority > 0 {
                    match best {
                        None => best = Some((i, job.priority)),
                        Some((_, best_pri)) if job.priority > best_pri => {
                            best = Some((i, job.priority))
                        }
                        _ => {}
                    }
                }
            }
        }
        best.map(|(i, _)| i)
    }

    /// Run the Executive main loop. Never returns in normal operation.
    ///
    /// On each iteration: pets the watchdog, finds the highest-priority job,
    /// and dispatches it. If no jobs are ready, loops (the dummy job).
    ///
    /// # Safety
    ///
    /// `state` must be a valid, exclusively-owned pointer to `AgcState` for
    /// the lifetime of this call. The function pointer stored in each job is
    /// responsible for not aliasing other parts of state.
    ///
    /// AGC source: EXECUTIVE.agc — EXEC loop.
    pub fn run(&mut self, state: *mut AgcState, hw: &mut impl AgcHardware) -> ! {
        loop {
            hw.pet_watchdog();
            if let Some(slot) = self.find_highest_priority_job() {
                if let Some(job) = self.jobs[slot] {
                    // SAFETY: caller guarantees state is valid.
                    (job.entry)(state);
                    // Slots are cleared by the job itself via complete_job.
                }
            }
        }
    }

    /// Dispatch the highest-priority job once without looping.
    ///
    /// Returns `true` if a job was dispatched, `false` if no jobs are ready.
    /// Used by tests and by the Executive main loop body.
    ///
    /// AGC source: EXECUTIVE.agc — EXEC loop body (single iteration).
    pub fn dispatch_highest(&mut self, state: *mut AgcState) -> bool {
        if let Some(slot) = self.find_highest_priority_job() {
            if let Some(job) = self.jobs[slot] {
                (job.entry)(state);
                return true;
            }
        }
        false
    }

    /// Number of occupied job slots.
    pub fn active_job_count(&self) -> usize {
        self.jobs.iter().filter(|s| s.is_some()).count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::services::alarm::AlarmState;

    fn noop_job(_: *mut AgcState) {}
    fn noop_job2(_: *mut AgcState) {}

    #[test]
    fn establish_and_complete_job() {
        let mut exec = Executive::new();
        let mut alarms = AlarmState::new();
        let slot = exec.establish_job(noop_job, 3, 0, &mut alarms).unwrap();
        assert_eq!(exec.active_job_count(), 1);
        exec.complete_job(slot);
        assert_eq!(exec.active_job_count(), 0);
    }

    #[test]
    fn highest_priority_selected() {
        let mut exec = Executive::new();
        let mut alarms = AlarmState::new();
        exec.establish_job(noop_job, 1, 0, &mut alarms);
        exec.establish_job(noop_job2, 5, 0, &mut alarms);
        let best = exec.find_highest_priority_job().unwrap();
        assert_eq!(exec.jobs[best].unwrap().priority, 5);
    }

    #[test]
    fn overflow_raises_alarm() {
        let mut exec = Executive::new();
        let mut alarms = AlarmState::new();
        for _ in 0..MAX_JOBS {
            exec.establish_job(noop_job, 1, 0, &mut alarms);
        }
        let result = exec.establish_job(noop_job, 1, 0, &mut alarms);
        assert!(result.is_none());
        assert!(alarms.is_raised(AlarmCode::ExecutiveOverflow));
    }
}
