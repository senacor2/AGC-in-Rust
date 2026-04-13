//! AGC Executive — cooperative, non-preemptive job scheduler.
//!
//! AGC source: Comanche055/EXECUTIVE.agc
//! Pages:      1208-1220 (MIT hardcopy pagination)
//! Routines:   NOVAC, FINDVAC, SPVAC, CHANG2, CHANJOB, ENDJOB1, EJSCAN,
//!             ENDOFJOB, DUMMYJOB, ADVAN, NUDIRECT, SUPDXCHZ

use crate::{
    executive::job::{JobEntry, JobFn, Priority, VacIndex},
    services::alarm::{AlarmCode, AlarmState},
};

/// Maximum number of job slots.
///
/// AGC source: Comanche055/EXECUTIVE.agc `NO.CORES DEC 6` (loop counter = 6 → 7 slots).
/// Confirmed: ERASABLE_ASSIGNMENTS.agc line 1627 "SEVEN SETS OF 12 REGISTERS EACH".
pub const MAX_JOBS: usize = 7;

/// Number of VAC areas.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc lines 1727-1736 (VAC1-VAC5).
pub const MAX_VAC_AREAS: usize = 5;

/// The complete Executive scheduler state.
///
/// All fields are statically allocated arrays — no heap.
///
/// AGC source: Comanche055/EXECUTIVE.agc, routines NOVAC, FINDVAC, CHANJOB, EJSCAN.
pub struct Executive {
    /// The 7-slot job table.
    /// Slot 0 is nominally the currently-running job (swapped in by CHANJOB).
    ///
    /// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
    /// "DYNAMICALLY ALLOCATED CORE SETS FOR JOBS (84D)" — 7 × 12 = 84 words.
    pub(crate) jobs: [JobEntry; MAX_JOBS],

    /// VAC area availability: `true` = free, `false` = in use.
    ///
    /// AGC source: VAC1USE-VAC5USE in ERASABLE_ASSIGNMENTS.agc lines 1727-1736.
    pub(crate) vac_free: [bool; MAX_VAC_AREAS],

    /// Index of the job that ran most recently (CHANJOB destination).
    pub(crate) current: usize,
}

impl Executive {
    /// Create a zero-initialised Executive (all slots free, all VAC areas free).
    ///
    /// Called once during FRESH START / STARTSB2.
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc STARTSB2, lines 562-583.
    pub const fn new() -> Self {
        const FREE_JOB: JobEntry = JobEntry::free();
        Self {
            jobs: [FREE_JOB; MAX_JOBS],
            vac_free: [true; MAX_VAC_AREAS],
            current: 0,
        }
    }

    /// Add a job to the scheduler (NOVAC path — no VAC area required).
    ///
    /// Returns `Some(slot_index)` on success.
    /// Returns `None` and raises alarm `NoCoreSets` (1202) if all 7 slots are occupied.
    ///
    /// Must be called from within a critical section (`sync::cs`).
    ///
    /// AGC source: Comanche055/EXECUTIVE.agc NOVAC / NOVAC2 / CORFOUND.
    pub fn add_job(&mut self, priority: Priority, entry: JobFn) -> Option<usize> {
        let slot = self.find_free_slot()?;
        self.jobs[slot].entry = Some(entry);
        self.jobs[slot].priority = priority;
        self.jobs[slot].sleeping = false;
        self.jobs[slot].vac = None;
        Some(slot)
    }

    /// Add a job that requires a VAC area (FINDVAC path).
    ///
    /// First locates a free VAC area; if none is available raises alarm 1201
    /// (`NoVacArea`) and returns `None`.  Then falls through to `add_job` logic.
    ///
    /// AGC source: Comanche055/EXECUTIVE.agc FINDVAC / FINDVAC2 / VACFOUND.
    pub fn add_job_with_vac(&mut self, priority: Priority, entry: JobFn) -> Option<usize> {
        let vac = self.find_free_vac()?;
        self.vac_free[vac as usize] = false;
        let slot = self.find_free_slot()?;
        self.jobs[slot].entry = Some(entry);
        self.jobs[slot].priority = priority;
        self.jobs[slot].sleeping = false;
        self.jobs[slot].vac = Some(vac);
        Some(slot)
    }

    /// Find the highest-priority runnable (non-sleeping, non-None) job.
    ///
    /// Returns its slot index, or `None` if no runnable jobs exist
    /// (triggers DUMMYJOB idle loop in the caller).
    ///
    /// AGC source: Comanche055/EXECUTIVE.agc EJSCAN / EJ1 / EJ2.
    pub fn find_highest_priority(&self) -> Option<usize> {
        let mut best: Option<usize> = None;
        let mut best_prio: Priority = 0;
        for (i, job) in self.jobs.iter().enumerate() {
            if job.is_runnable() && job.priority > best_prio {
                best_prio = job.priority;
                best = Some(i);
            }
        }
        best
    }

    /// Dispatch the next job: swap the winning slot into current position
    /// and return its `JobFn`.
    ///
    /// Returns `None` if no runnable job exists (caller enters idle loop).
    ///
    /// Corresponds to CHANJOB in the AGC source.
    ///
    /// AGC source: Comanche055/EXECUTIVE.agc CHANJOB (page 1213).
    pub fn run_next(&mut self) -> Option<JobFn> {
        let winner = self.find_highest_priority()?;
        // In the AGC, CHANJOB swaps the winner into slot 0.
        // In Rust we track the current slot index directly.
        self.current = winner;
        self.jobs[winner].entry
    }

    /// Mark the currently running job as complete and release its slot (and VAC area if any).
    ///
    /// AGC source: Comanche055/EXECUTIVE.agc ENDOFJOB / ENDJOB1.
    pub fn finish_job(&mut self) {
        let i = self.current;
        if let Some(vac) = self.jobs[i].vac {
            self.vac_free[vac as usize] = true;
        }
        self.jobs[i] = JobEntry::free();
    }

    /// Voluntarily suspend the current job (lower-priority yield).
    ///
    /// The job remains in its slot but `run_next` will re-scan for the highest
    /// priority job.  In the real AGC the yield is accomplished by CHANJOB
    /// which may swap in a higher-priority job.
    ///
    /// AGC source: Comanche055/EXECUTIVE.agc CHANG1 / CHANG2 / CHANJOB.
    pub fn yield_current(&mut self) {
        // No structural change needed — run_next will pick the highest priority.
        // This is the Rust equivalent of "return to Executive and let EJSCAN decide".
    }

    /// Change the priority of the currently running job.
    ///
    /// If the new priority is lower than some other runnable job,
    /// `run_next()` is expected to be called afterward.
    ///
    /// AGC source: Comanche055/EXECUTIVE.agc PRIOCHNG / PRIOCH2.
    pub fn change_priority(&mut self, new_priority: Priority) {
        self.jobs[self.current].priority = new_priority;
    }

    /// Put the current job to sleep, waiting for a JOBWAKE signal.
    ///
    /// AGC source: Comanche055/EXECUTIVE.agc JOBSLEEP / JOBSLP1.
    pub fn sleep_current(&mut self, _wakeup_addr: JobFn) {
        self.jobs[self.current].sleeping = true;
    }

    /// Wake a sleeping job whose `entry` matches `wakeup_fn`.
    ///
    /// Scans all 7 slots for a sleeping job with matching entry.
    /// Returns `true` if a job was woken.
    ///
    /// AGC source: Comanche055/EXECUTIVE.agc JOBWAKE / JOBWAKE2 / WAKETEST.
    pub fn wake_job(&mut self, wakeup_fn: JobFn) -> bool {
        for job in self.jobs.iter_mut() {
            if job.sleeping {
                if let Some(entry) = job.entry {
                    if core::ptr::eq(entry as *const (), wakeup_fn as *const ()) {
                        job.sleeping = false;
                        return true;
                    }
                }
            }
        }
        false
    }

    /// Return a reference to the currently active job entry.
    pub fn current_job(&self) -> Option<&JobEntry> {
        let job = &self.jobs[self.current];
        if job.is_occupied() {
            Some(job)
        } else {
            None
        }
    }

    /// True if no runnable jobs exist (Executive is in idle / DUMMYJOB state).
    ///
    /// AGC source: Comanche055/EXECUTIVE.agc DUMMYJOB idle loop.
    pub fn is_idle(&self) -> bool {
        self.find_highest_priority().is_none()
    }

    /// Count of non-free slots.
    pub fn active_job_count(&self) -> usize {
        self.jobs.iter().filter(|j| j.is_occupied()).count()
    }

    // ── private helpers ───────────────────────────────────────────────────────

    /// Find the first free job slot.
    ///
    /// Returns `None` and raises alarm `NoCoreSets` (1202) if all slots are full.
    fn find_free_slot(&mut self) -> Option<usize> {
        for (i, job) in self.jobs.iter().enumerate() {
            if job.is_free() {
                return Some(i);
            }
        }
        // All 7 slots occupied — raise alarm 1202 and return None.
        // AGC source: EXECUTIVE.agc NOVAC3 `TC BAILOUT / OCT 1202`.
        AlarmState::raise(AlarmCode::NoCoreSets);
        None
    }

    /// Find the first free VAC area.
    ///
    /// Returns `None` and raises alarm `NoVacArea` (1201) if all VAC areas are in use.
    fn find_free_vac(&mut self) -> Option<VacIndex> {
        for (i, &free) in self.vac_free.iter().enumerate() {
            if free {
                return Some(i as VacIndex);
            }
        }
        // All VAC areas in use — raise alarm 1201 and return None.
        // AGC source: EXECUTIVE.agc FINDVAC2 `TC BAILOUT / OCT 1201`.
        AlarmState::raise(AlarmCode::NoVacArea);
        None
    }
}

impl Default for Executive {
    fn default() -> Self {
        Self::new()
    }
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AgcState;

    fn dummy_job(_: &mut AgcState) {}
    fn job_a(_: &mut AgcState) {}
    fn job_b(_: &mut AgcState) {}
    fn job_c(_: &mut AgcState) {}

    #[test]
    fn single_job_dispatch() {
        // Test 1: single job add → find → run → finish
        let mut exec = Executive::new();

        let slot = exec.add_job(1000, dummy_job);
        assert_eq!(slot, Some(0), "first free slot should be 0");
        assert_eq!(exec.find_highest_priority(), Some(0));

        let fn_ptr = exec.run_next();
        assert!(fn_ptr.is_some());

        exec.finish_job();
        assert!(exec.current_job().is_none());
        assert!(exec.is_idle());
    }

    #[test]
    fn priority_ordering() {
        // Test 2: three jobs dispatched in priority order (300 → 200 → 100)
        let mut exec = Executive::new();

        exec.add_job(100, job_a);
        exec.add_job(300, job_b);
        exec.add_job(200, job_c);

        // Highest priority first: 300
        let first = exec.run_next().unwrap();
        assert!((first as *const () as usize) == (job_b as *const () as usize));
        exec.finish_job();

        // Next: 200
        let second = exec.run_next().unwrap();
        assert!((second as *const () as usize) == (job_c as *const () as usize));
        exec.finish_job();

        // Last: 100
        let third = exec.run_next().unwrap();
        assert!((third as *const () as usize) == (job_a as *const () as usize));
        exec.finish_job();

        assert!(exec.run_next().is_none());
    }

    #[test]
    fn table_overflow_raises_1202() {
        // Test 3: fill 7 slots then try 8th → alarm 1202
        let mut exec = Executive::new();

        for i in 0..MAX_JOBS {
            let r = exec.add_job(100 + i as u16, dummy_job);
            assert!(r.is_some(), "slot {i} should be available");
        }

        // 8th call should fail and raise NoCoreSets
        crate::services::alarm::clear_all();
        let r = exec.add_job(200, dummy_job);
        assert!(r.is_none(), "8th add_job should fail");

        let last = crate::services::alarm::most_recent();
        assert_eq!(last, Some(AlarmCode::NoCoreSets));
    }

    #[test]
    fn vac_overflow_raises_1201() {
        // Fill all 5 VAC areas; 6th should raise NoVacArea
        let mut exec = Executive::new();

        for _ in 0..MAX_VAC_AREAS {
            let r = exec.add_job_with_vac(100, dummy_job);
            assert!(r.is_some());
        }

        crate::services::alarm::clear_all();
        let r = exec.add_job_with_vac(100, dummy_job);
        assert!(r.is_none());
        assert_eq!(
            crate::services::alarm::most_recent(),
            Some(AlarmCode::NoVacArea)
        );
    }

    #[test]
    fn sleep_and_wake() {
        let mut exec = Executive::new();
        exec.add_job(100, job_a);
        exec.run_next();
        exec.sleep_current(job_a);
        // While sleeping, not runnable
        assert!(exec.find_highest_priority().is_none());
        // Wake it
        assert!(exec.wake_job(job_a));
        assert!(exec.find_highest_priority().is_some());
    }
}
