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

    /// Scan for the highest-priority occupied slot.
    /// Tie-breaking: on equal priority, the lower index wins (spec §5.1).
    fn find_highest_priority_job(&self) -> Option<usize> {
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

#[cfg(test)]
mod tests {
    use super::*;

    fn dummy_job(_state: &mut crate::AgcState) {}
    fn dummy_job_b(_state: &mut crate::AgcState) {}

    // TC-CJ-1: create_job into empty table
    #[test]
    fn tc_cj_1_create_into_empty() {
        let mut exec = Executive::new();
        assert!(exec.create_job(20, dummy_job, 0));
        assert_eq!(exec.jobs[0].priority, 20);
    }

    // TC-CJ-2: create_job with 6 occupied
    #[test]
    fn tc_cj_2_create_with_6_occupied() {
        let mut exec = Executive::new();
        for i in 1..=6 {
            assert!(exec.create_job(i, dummy_job, 0));
        }
        assert!(exec.create_job(7, dummy_job, 0));
    }

    // TC-CJ-3: create_job when full returns false
    #[test]
    fn tc_cj_3_create_when_full() {
        let mut exec = Executive::new();
        for i in 1..=7 {
            assert!(exec.create_job(i, dummy_job, 0));
        }
        assert!(!exec.create_job(5, dummy_job, 0));
    }

    // TC-CJ-4: create_job picks first empty slot
    #[test]
    fn tc_cj_4_first_empty_slot() {
        let mut exec = Executive::new();
        exec.create_job(1, dummy_job, 0);
        exec.create_job(1, dummy_job_b, 0);
        assert_eq!(exec.jobs[0].priority, 1);
        assert_eq!(exec.jobs[1].priority, 1);
    }

    // TC-DL-1: drop_lowest_job removes minimum priority
    #[test]
    fn tc_dl_1_drop_minimum() {
        let mut exec = Executive::new();
        exec.create_job(10, dummy_job, 0);
        exec.create_job(20, dummy_job, 0);
        exec.create_job(5, dummy_job, 0);
        exec.create_job(30, dummy_job, 0);
        exec.create_job(15, dummy_job, 0);
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
        exec.create_job(10, dummy_job, 0);
        exec.create_job(10, dummy_job, 0);
        exec.create_job(10, dummy_job, 0);
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
        exec.create_job(10, dummy_job, 0);   // slot 0
        exec.create_job(20, dummy_job, 0);   // slot 1
        exec.create_job(20, dummy_job_b, 0); // slot 2 (same priority)
        assert_eq!(exec.find_highest_priority_job(), Some(1)); // lowest index wins
    }
}
