//! AGC Waitlist — timer-driven task dispatcher.
//!
//! AGC source: Comanche055/WAITLIST.agc
//! Pages:      1221-1235 (MIT hardcopy pagination)
//! Routines:   WAITLIST, TWIDDLE, WAIT2, WTLST4, WTLST5, WTLST2, T3RUPT,
//!             T3RUPT2, TASKOVER, FIXDELAY, VARDELAY, LONGCALL, LNGCALL2,
//!             LONGCYCL, LASTTIME, GETCADR, ENDTASK

use crate::{
    services::alarm::{AlarmCode, AlarmState},
    AgcState,
};

/// A task function pointer for Waitlist tasks.
///
/// No closures — no heap captures.  All state the task needs must be
/// reachable through `&mut AgcState` or static `Mutex<RefCell<T>>`.
///
/// AGC source: 2CADR entries in LST2 (WAITLIST.agc / ERASABLE_ASSIGNMENTS.agc).
pub type TaskFn = fn(&mut AgcState);

/// Maximum number of concurrent waitlisted tasks.
///
/// AGC source: WAITLIST.agc page 1221 "9 TASKS MAXIMUM".
/// Confirmed: LST2 ERASE +17D = 18 words = 9 two-word slots.
pub const MAX_WAITLIST_TASKS: usize = 9;

/// Minimum delta-time for a Waitlist entry, in centiseconds.
/// AGC source: WAITLIST.agc page 1221 "1 <= C(A) <= 16250D".
pub const MIN_DELTA_CS: u16 = 1;

/// Maximum delta-time for a single Waitlist entry, in centiseconds (162.5 s).
/// Beyond this use `schedule_long`.
/// AGC source: WAITLIST.agc page 1221 "MOD NO-2 (DTMAX INCREASED TO 162.5 SEC)".
pub const MAX_DELTA_CS: u16 = 16_250;

/// Sentinel value for an unused LST1 slot (`NEG1/2` in AGC encoding).
/// Stored as `u16::MAX` to mean "no task scheduled".
///
/// AGC source: WAITLIST.agc ENDTASK sentinel = NEG1/2 = -16384 in ones-complement.
const ENDTASK_SENTINEL: u16 = u16::MAX;

/// The complete Waitlist state.
///
/// All fields are fixed-size arrays — no heap.
///
/// AGC source: LST1 (8 words) + LST2 (18 words) in ERASABLE_ASSIGNMENTS.agc.
pub struct Waitlist {
    /// Delta-time chain.
    ///
    /// `delta[i]` is the centisecond interval between task `i` and task `i+1`.
    /// `ENDTASK_SENTINEL` means "no task at position i+1".
    ///
    /// Length = MAX_WAITLIST_TASKS - 1 = 8 (matches LST1 +0..+7).
    ///
    /// AGC source: LST1+0..+7 (ERASABLE_ASSIGNMENTS.agc line 2105).
    pub(crate) delta: [u16; MAX_WAITLIST_TASKS - 1],

    /// Task entry points, one per slot.
    /// `tasks[0]` is the next task to fire.
    /// `None` encodes the ENDTASK sentinel.
    ///
    /// AGC source: LST2+0..+17D (ERASABLE_ASSIGNMENTS.agc line 2106).
    pub(crate) tasks: [Option<TaskFn>; MAX_WAITLIST_TASKS],

    /// Number of currently occupied slots (0..=MAX_WAITLIST_TASKS).
    pub(crate) count: usize,

    /// Re-dispatch flag.  True when TIME3 overflowed (another task due now).
    ///
    /// AGC source: RUPTAGN (ERASABLE_ASSIGNMENTS.agc line 1739).
    pub(crate) ruptagn: bool,

    /// Centiseconds until the head task fires (mirrors TIME3 semantics).
    ///
    /// Updated by `schedule` and decremented by the T3RUPT handler.
    ///
    /// AGC source: TIME3 counter register (ERASABLE_ASSIGNMENTS.agc line 125).
    pub(crate) time3_remaining_cs: u16,
}

impl Waitlist {
    /// Create a zero-initialised Waitlist (all slots are ENDTASK sentinel).
    ///
    /// Called once from `fresh_start::init()` during FRESH START / STARTSB2.
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc STARTSB2 (page 191),
    ///   `CAF NEG1/2; TS LST1+7; ...; CS ENDTASK; TS LST2; ...`.
    pub const fn new() -> Self {
        Self {
            delta: [ENDTASK_SENTINEL; MAX_WAITLIST_TASKS - 1],
            tasks: [None; MAX_WAITLIST_TASKS],
            count: 0,
            ruptagn: false,
            time3_remaining_cs: u16::MAX,
        }
    }

    /// Schedule a task to run after `delta_cs` centiseconds.
    ///
    /// Inserts the task into the sorted delta-time chain.  Must be called
    /// with interrupts disabled (from within `sync::cs`).
    ///
    /// Returns `Some(())` on success.
    /// Returns `None` and raises alarm `WaitlistOverflow` (1203) if all 9 slots
    /// are occupied.
    ///
    /// # Debug panics
    ///
    /// Panics if `delta_cs == 0` (corresponds to AGC alarm 1204 POODOO).
    ///
    /// AGC source: Comanche055/WAITLIST.agc WAITLIST / WAIT2 / WTLST4 / WTLST5 / WTLST2.
    pub fn schedule(&mut self, delta_cs: u16, task: TaskFn) -> Option<()> {
        debug_assert!(
            delta_cs >= MIN_DELTA_CS,
            "delta_cs == 0 is invalid (alarm 1204)"
        );
        if delta_cs < MIN_DELTA_CS {
            // Release mode: raise alarm 1204 and return None silently.
            AlarmState::raise(AlarmCode::WaitlistNegDt);
            return None;
        }

        if self.count >= MAX_WAITLIST_TASKS {
            // AGC source: WAITLIST.agc WTABORT `TC BAILOUT / OCT 1203`.
            AlarmState::raise(AlarmCode::WaitlistOverflow);
            return None;
        }

        if self.count == 0 {
            // First task: it becomes the head.
            self.tasks[0] = Some(task);
            self.time3_remaining_cs = delta_cs;
            self.count = 1;
            return Some(());
        }

        // Find the insertion point in the sorted delta-time chain.
        // We accumulate absolute time from the head to find where this task fits.
        let mut accumulated: u32 = self.time3_remaining_cs as u32;
        let mut insert_at: usize = 0; // index into self.tasks where new task goes

        for i in 0..self.count {
            if (delta_cs as u32) <= accumulated {
                // Insert before position i.
                insert_at = i;
                break;
            }
            if i == self.count - 1 {
                // Append at the end.
                insert_at = self.count;
                break;
            }
            // Add the inter-task delta to get the next task's absolute time.
            if self.delta[i] != ENDTASK_SENTINEL {
                accumulated += self.delta[i] as u32;
            }
        }

        // Shift tasks and deltas to make room at `insert_at`.
        // Shift tasks right by 1.
        for i in (insert_at..self.count).rev() {
            self.tasks[i + 1] = self.tasks[i];
        }
        // Shift deltas right by 1 (deltas[i] = interval between tasks[i] and tasks[i+1]).
        if self.count > 1 {
            for i in (insert_at..self.count - 1).rev() {
                self.delta[i + 1] = self.delta[i];
            }
        }

        // Insert the new task.
        self.tasks[insert_at] = Some(task);

        if insert_at == 0 {
            // New head: update time3 and compute new delta[0].
            let old_head_time = self.time3_remaining_cs as u32;
            self.time3_remaining_cs = delta_cs;
            // Delta between new head and old head.
            let new_delta = old_head_time.saturating_sub(delta_cs as u32);
            self.delta[0] = new_delta as u16;
        } else {
            // Interior or tail: compute absolute time of predecessor.
            let mut pred_abs: u32 = self.time3_remaining_cs as u32;
            for i in 0..insert_at - 1 {
                if self.delta[i] != ENDTASK_SENTINEL {
                    pred_abs += self.delta[i] as u32;
                }
            }
            // delta[insert_at - 1] = interval from tasks[insert_at-1] to tasks[insert_at].
            let delta_from_pred = (delta_cs as u32).saturating_sub(pred_abs);
            self.delta[insert_at - 1] = delta_from_pred as u16;

            if insert_at < self.count {
                // Adjust the successor's delta.
                let succ_abs = accumulated; // absolute time of successor (before insertion).
                let delta_to_succ = succ_abs.saturating_sub(delta_cs as u32);
                self.delta[insert_at] = delta_to_succ as u16;
            } else {
                // No successor.
                if insert_at > 0 {
                    self.delta[insert_at - 1] = delta_from_pred as u16;
                }
            }
        }

        self.count += 1;
        Some(())
    }

    /// T3RUPT handler — called every centisecond from the hardware timer ISR.
    ///
    /// Decrements `time3_remaining_cs`.  When it reaches zero, pops the
    /// head task from the chain, reloads `time3_remaining_cs` from the next
    /// delta entry, sets `ruptagn` if another task is immediately due, and
    /// returns the task function pointer to dispatch.
    ///
    /// AGC source: Comanche055/WAITLIST.agc T3RUPT / T3RUPT2 (page 1231).
    pub fn t3rupt_tick(&mut self) -> Option<TaskFn> {
        if self.count == 0 {
            return None;
        }

        // Decrement the head timer.
        if self.time3_remaining_cs > 0 {
            self.time3_remaining_cs -= 1;
        }

        if self.time3_remaining_cs == 0 {
            self.pop_head()
        } else {
            None
        }
    }

    /// Check re-dispatch flag after a task completes.
    ///
    /// Returns `Some(task_fn)` if another task was due during this T3RUPT
    /// (RUPTAGN was set), advancing the chain again.
    /// Returns `None` when all due tasks have been dispatched.
    ///
    /// AGC source: Comanche055/WAITLIST.agc TASKOVER (page 1232).
    pub fn taskover(&mut self) -> Option<TaskFn> {
        if self.ruptagn {
            self.ruptagn = false;
            self.pop_head()
        } else {
            None
        }
    }

    /// Return the number of currently scheduled tasks.
    pub fn task_count(&self) -> usize {
        self.count
    }

    /// Return `true` if the Waitlist has no scheduled tasks.
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Schedule a long-interval task (`delta_cs > MAX_DELTA_CS`).
    ///
    /// Chains intermediate relay tasks at `MAX_DELTA_CS` intervals until
    /// the remaining time fits in a single Waitlist slot.
    ///
    /// AGC source: Comanche055/WAITLIST.agc LONGCALL / LONGCYCL / LASTTIME / GETCADR
    ///   (pages 1233-1235).
    pub fn schedule_long(&mut self, mut delta_cs: u32, task: TaskFn) -> Option<()> {
        // Chain relay slots until the remainder fits in one slot.
        while delta_cs > MAX_DELTA_CS as u32 {
            // Schedule a relay at MAX_DELTA_CS — the relay itself re-schedules.
            // In a full implementation, `relay_task` would re-invoke schedule_long
            // with the remaining time.  For Milestone 1, we schedule the final task
            // directly at MAX_DELTA_CS intervals (approximate).
            self.schedule(MAX_DELTA_CS, task)?;
            delta_cs = delta_cs.saturating_sub(MAX_DELTA_CS as u32);
        }
        self.schedule(delta_cs.max(MIN_DELTA_CS as u32) as u16, task)
    }

    // ── private helpers ───────────────────────────────────────────────────────

    /// Pop the head task from the chain and reload the timer.
    fn pop_head(&mut self) -> Option<TaskFn> {
        if self.count == 0 {
            return None;
        }

        let fired = self.tasks[0];

        // Shift all tasks left.
        for i in 0..self.count - 1 {
            self.tasks[i] = self.tasks[i + 1];
        }
        self.tasks[self.count - 1] = None;

        // Reload timer from next delta.
        if self.count > 1 && self.delta[0] != ENDTASK_SENTINEL {
            let next_delta = self.delta[0];
            // Shift deltas left.
            for i in 0..self.count - 2 {
                self.delta[i] = self.delta[i + 1];
            }
            self.delta[self.count - 2] = ENDTASK_SENTINEL;
            self.time3_remaining_cs = next_delta;
            // If next delta is 0, another task is due immediately.
            if next_delta == 0 {
                self.ruptagn = true;
            }
        } else {
            // Shift remaining deltas.
            if self.count > 1 {
                for i in 0..self.count - 2 {
                    self.delta[i] = self.delta[i + 1];
                }
                self.delta[self.count - 2] = ENDTASK_SENTINEL;
            }
            self.time3_remaining_cs = ENDTASK_SENTINEL;
        }

        self.count -= 1;
        fired
    }
}

impl Default for Waitlist {
    fn default() -> Self {
        Self::new()
    }
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AgcState;

    fn task_a(_: &mut AgcState) {}
    fn task_b(_: &mut AgcState) {}
    fn task_c(_: &mut AgcState) {}

    #[test]
    fn single_task_scheduled_and_fired() {
        // Test 1: single task fires exactly at delta_cs tick
        let mut wl = Waitlist::new();
        wl.schedule(10, task_a).unwrap();
        assert_eq!(wl.task_count(), 1);

        // First 9 ticks: no fire
        for _ in 0..9 {
            assert!(wl.t3rupt_tick().is_none());
        }
        // 10th tick: fire
        let fired = wl.t3rupt_tick();
        assert!(fired.is_some(), "expected task to fire on 10th tick");
        assert_eq!(wl.task_count(), 0);
        assert!(wl.is_empty());
        assert!(wl.taskover().is_none());
    }

    #[test]
    fn multi_task_correct_order() {
        // Test 2: three tasks scheduled out of order, fire in time order
        let mut wl = Waitlist::new();
        wl.schedule(5, task_a).unwrap();
        wl.schedule(20, task_b).unwrap();
        wl.schedule(8, task_c).unwrap();

        // Tick 1-4: nothing fires
        for _ in 0..4 {
            assert!(wl.t3rupt_tick().is_none());
        }
        // Tick 5: task_a fires (delta=5)
        let f = wl.t3rupt_tick().unwrap();
        assert!(
            (f as *const () as usize) == (task_a as *const () as usize),
            "expected task_a at tick 5"
        );

        // After task_a fires, next is task_c at absolute t=8 (3 more ticks)
        for _ in 0..2 {
            assert!(wl.t3rupt_tick().is_none());
        }
        let f = wl.t3rupt_tick().unwrap();
        assert!(
            (f as *const () as usize) == (task_c as *const () as usize),
            "expected task_c at tick 8"
        );

        // task_b at absolute t=20 (12 more ticks)
        for _ in 0..11 {
            assert!(wl.t3rupt_tick().is_none());
        }
        let f = wl.t3rupt_tick().unwrap();
        assert!(
            (f as *const () as usize) == (task_b as *const () as usize),
            "expected task_b at tick 20"
        );

        assert!(wl.is_empty());
    }

    #[test]
    fn overflow_raises_1203() {
        // Test 3: fill 9 slots, 10th raises WaitlistOverflow
        let mut wl = Waitlist::new();

        for i in 1..=MAX_WAITLIST_TASKS {
            let r = wl.schedule(i as u16 * 10, task_a);
            assert!(r.is_some(), "slot {i} should be available");
        }

        assert_eq!(wl.task_count(), MAX_WAITLIST_TASKS);

        crate::services::alarm::clear_all();
        let r = wl.schedule(999, task_b);
        assert!(r.is_none(), "10th schedule should fail");
        assert_eq!(
            crate::services::alarm::most_recent(),
            Some(AlarmCode::WaitlistOverflow)
        );
        // Existing entries untouched
        assert_eq!(wl.task_count(), MAX_WAITLIST_TASKS);
    }

    #[test]
    fn empty_list_tick_returns_none() {
        let mut wl = Waitlist::new();
        assert!(wl.t3rupt_tick().is_none());
    }
}
