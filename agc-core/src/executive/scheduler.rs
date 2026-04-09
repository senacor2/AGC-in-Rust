use crate::hal::AgcHardware;
use super::job::{JobEntry, JobPriority, MAX_JOBS};

/// The Executive — cooperative priority-based job scheduler.
///
/// Maintains a table of up to `MAX_JOBS` concurrent jobs. The `run` loop
/// scans for the highest-priority ready job, dispatches it, and repeats.
/// Only interrupts and waitlist tasks can preempt a running job.
pub struct Executive {
    jobs: [JobEntry; MAX_JOBS],
    current_priority: JobPriority,
}

impl Executive {
    pub const fn new() -> Self {
        Self {
            jobs: [JobEntry::EMPTY; MAX_JOBS],
            current_priority: 0,
        }
    }

    /// Create a new job. Returns `false` if all slots are occupied (alarm 1202).
    pub fn create_job(&mut self, priority: JobPriority, entry: fn(&mut crate::AgcState), major_mode: u8) -> bool {
        for slot in &mut self.jobs {
            if slot.priority == 0 {
                *slot = JobEntry { priority, entry, major_mode };
                return true;
            }
        }
        false
    }

    /// Remove the job with the lowest priority (used during alarm 1202 recovery).
    pub fn drop_lowest_job(&mut self) {
        if let Some(slot) = self.jobs.iter_mut().filter(|j| j.priority > 0).min_by_key(|j| j.priority) {
            *slot = JobEntry::EMPTY;
        }
    }

    /// The main scheduling loop. Never returns in normal operation.
    ///
    /// Calls `hw.pet_watchdog()` on every iteration to reset the night-watchman
    /// timer. If no jobs are ready, the loop spins (P00 idle).
    pub fn run(&mut self, state: &mut crate::AgcState, hw: &mut impl AgcHardware) -> ! {
        loop {
            hw.pet_watchdog();

            if let Some(idx) = self.find_highest_priority_job() {
                let entry = self.jobs[idx].entry;
                self.current_priority = self.jobs[idx].priority;
                (entry)(state);
                // Job returned normally — clear the slot.
                self.jobs[idx] = JobEntry::EMPTY;
                self.current_priority = 0;
            }
        }
    }

    fn find_highest_priority_job(&self) -> Option<usize> {
        self.jobs
            .iter()
            .enumerate()
            .filter(|(_, j)| j.priority > 0)
            .max_by_key(|(_, j)| j.priority)
            .map(|(i, _)| i)
    }
}
