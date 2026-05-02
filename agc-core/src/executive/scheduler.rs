use crate::hal::{AgcHardware, rcs::Rcs};
use crate::hal::runtime::{DEMO_HOOK, T3_PENDING, T3_TICK_COUNT, T4_PENDING, T5_PENDING, T6_PENDING};
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

    /// Create a new job. Returns `false` if all slots are occupied (alarm 1202;
    /// alarm 1210 if `has_vac` is true).
    ///
    /// Maps to both AGC NOVAC (`has_vac = false`) and FINDVAC (`has_vac = true`).
    pub fn create_job(&mut self, priority: JobPriority, entry: fn(&mut crate::AgcState), major_mode: u8, has_vac: bool) -> bool {
        for slot in &mut self.jobs {
            if slot.priority == 0 {
                *slot = JobEntry { priority, entry, major_mode, has_vac };
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

    /// Main scheduler loop. Never returns.
    ///
    /// Drains the four AGC interrupt flags (T3/T4/T5/T6) in priority order,
    /// then dispatches the highest-priority ready job. The watchdog is petted
    /// at the top of every iteration so a hung job triggers a hardware restart
    /// (night-watchman, ADR-009).
    ///
    /// Signature is a free associated function (not `&mut self`) so the caller
    /// can pass the full `&mut AgcState` — `executive` lives inside `AgcState`,
    /// and the previous `&mut self` receiver would have caused a split-borrow
    /// conflict when dispatching a job that mutates other fields of `state`.
    pub fn run<H: AgcHardware>(state: &mut crate::AgcState, hw: &mut H) -> ! {
        use core::sync::atomic::Ordering;

        loop {
            hw.pet_watchdog();

            // Drain ISR-posted flags in AGC priority order
            // (lowest Interrupt discriminant = highest priority — see interrupts.rs).

            // T6RUPT: RCS jet quench (highest AGC priority).
            if T6_PENDING.swap(false, Ordering::Acquire) {
                hw.rcs().quench_all();
            }

            // T5RUPT: DAP cycle wiring arrives in the next milestone.
            if T5_PENDING.swap(false, Ordering::Acquire) {
                // Placeholder — DAP/SERVICER wiring deferred.
            }

            // T3RUPT: For the Phase-3 demo, create a low-priority job that
            // invokes the board-registered demo hook. Real Waitlist dispatch
            // replaces this in the T3RUPT milestone.
            if T3_PENDING.swap(false, Ordering::Acquire) {
                T3_TICK_COUNT.fetch_add(1, Ordering::Relaxed);
                let _ = state.executive.create_job(1, __demo_tick, 0, false);
            }

            // T4RUPT: periodic I/O wiring (DSKY refresh, gyro drift) deferred.
            if T4_PENDING.swap(false, Ordering::Acquire) {
                // Placeholder — T4 periodic I/O wiring deferred.
            }

            // Dispatch one job per iteration.
            if let Some(idx) = state.executive.find_highest_priority_job() {
                let entry = state.executive.jobs[idx].entry;
                state.executive.current_priority = state.executive.jobs[idx].priority;
                (entry)(state);
                state.executive.jobs[idx] = JobEntry::EMPTY;
                state.executive.current_priority = 0;
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

/// Phase-3 demo job dispatched by the T3 drain path in `Executive::run`.
///
/// Reads the board-registered `DEMO_HOOK` function pointer and calls it if
/// non-null. This keeps agc-core free of `defmt` and board-specific imports:
/// the logging concern belongs in the board crate, not here.
///
/// Hidden from rustdoc; will be removed when real T3RUPT/Waitlist wiring
/// replaces the demo in the next milestone.
#[doc(hidden)]
pub fn __demo_tick(state: &mut crate::AgcState) {
    use core::sync::atomic::Ordering;
    let raw = DEMO_HOOK.load(Ordering::Acquire);
    if !raw.is_null() {
        // SAFETY: `DEMO_HOOK` is only written by `register_demo_hook`, which
        // requires the caller to pass a `fn(&mut AgcState)`. We transmute back
        // to that type here. Function pointers on Cortex-M live in flash and
        // cannot be invalidated; the pointer remains valid for program lifetime.
        let f: fn(&mut crate::AgcState) = unsafe { core::mem::transmute(raw) };
        f(state);
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
        exec.create_job(10, dummy_job, 0, false);   // slot 0
        exec.create_job(20, dummy_job, 0, false);   // slot 1
        exec.create_job(20, dummy_job_b, 0, false); // slot 2 (same priority)
        assert_eq!(exec.find_highest_priority_job(), Some(1)); // lowest index wins
    }
}
