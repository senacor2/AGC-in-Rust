/// Maximum number of concurrently pending waitlist tasks.
pub const MAX_WAITLIST_TASKS: usize = 8;

/// Result of a `Waitlist::schedule` call.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ScheduleResult {
    /// Task inserted. Caller must reload TIME3 with this value
    /// (the new task is the earliest in the list).
    OkReloadT3(u16),
    /// Task inserted. TIME3 does not need reloading (new task is not earliest).
    Ok,
    /// Waitlist full (all 8 slots occupied). Alarm 1211 must be raised.
    Full,
}

/// A single entry in the Waitlist.
#[derive(Clone, Copy)]
pub struct WaitlistEntry {
    /// Centiseconds until this task fires, measured as a delta from the
    /// previous entry in the sorted list (or from "now" for the first entry).
    pub delta_time: u16,
    /// The task function to run when the timer expires.
    /// Tasks are short, run to completion, and must not block.
    pub task: fn(&mut crate::AgcState),
}

/// The Waitlist — a sorted delta-time chain of pending time-triggered tasks.
///
/// Tasks are dispatched by T3RUPT (TIME3 overflow). The list is kept
/// sorted by absolute fire time; `delta_time` is relative to the previous
/// entry. Only the first entry's delta needs to be loaded into TIME3.
pub struct Waitlist {
    entries: [Option<WaitlistEntry>; MAX_WAITLIST_TASKS],
    count: usize,
}

impl Waitlist {
    pub const fn new() -> Self {
        Self {
            entries: [None; MAX_WAITLIST_TASKS],
            count: 0,
        }
    }

    /// Schedule a task to fire `centiseconds` from now.
    ///
    /// Preconditions:
    /// - `centiseconds > 0` (zero delay is undefined).
    /// - `centiseconds <= 16383` (max TIME3 load; use long-waitlist chaining for more).
    ///
    /// Returns `ScheduleResult::OkReloadT3(cs)` if the new task became the earliest
    /// (caller must call `hw.timers().arm_t3(cs)`), `ScheduleResult::Ok` if inserted
    /// but not earliest, or `ScheduleResult::Full` if the waitlist is full.
    pub fn schedule(
        &mut self,
        centiseconds: u16,
        task: fn(&mut crate::AgcState),
    ) -> ScheduleResult {
        if self.count >= MAX_WAITLIST_TASKS {
            return ScheduleResult::Full;
        }

        // Walk the delta chain to find the insertion position.
        let mut insert_pos = self.count; // default: append at end
        let mut running_sum: u16 = 0;
        for i in 0..self.count {
            if let Some(ref entry) = self.entries[i] {
                if centiseconds <= running_sum + entry.delta_time {
                    insert_pos = i;
                    break;
                }
                running_sum += entry.delta_time;
            }
        }

        // Compute the new entry's delta relative to the previous entry.
        let new_delta = centiseconds - running_sum;

        // If inserting before an existing entry, adjust that entry's delta.
        if insert_pos < self.count {
            if let Some(ref mut entry) = self.entries[insert_pos] {
                entry.delta_time -= new_delta;
            }
        }

        // Shift entries right to make room at insert_pos.
        let mut i = self.count;
        while i > insert_pos {
            self.entries[i] = self.entries[i - 1];
            i -= 1;
        }

        // Insert the new entry.
        self.entries[insert_pos] = Some(WaitlistEntry {
            delta_time: new_delta,
            task,
        });
        self.count += 1;

        if insert_pos == 0 {
            ScheduleResult::OkReloadT3(new_delta)
        } else {
            ScheduleResult::Ok
        }
    }

    /// Dispatch the earliest pending task.
    ///
    /// Called by the T3RUPT handler. Fires the task at `entries[0]`, removes it,
    /// and returns the delta time to the next task (for reloading TIME3), or
    /// `None` if the list is now empty.
    pub fn dispatch(&mut self, state: &mut crate::AgcState) -> Option<u16> {
        if self.count == 0 {
            return None;
        }

        // Extract and fire the earliest task.
        let entry = self.entries[0].take().unwrap();

        // Shift remaining entries left.
        for i in 0..self.count - 1 {
            self.entries[i] = self.entries[i + 1];
        }
        self.entries[self.count - 1] = None;
        self.count -= 1;

        // Fire the task.
        (entry.task)(state);

        // Return the next task's delta for TIME3 reload.
        if self.count > 0 {
            self.entries[0].map(|e| e.delta_time)
        } else {
            None
        }
    }

    /// Number of currently scheduled tasks.
    pub fn len(&self) -> usize {
        self.count
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Peek at entry `i` (for testing / debugging).
    pub fn peek(&self, index: usize) -> Option<&WaitlistEntry> {
        if index < self.count {
            self.entries[index].as_ref()
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::{AtomicU8, Ordering};

    static CALL_LOG: AtomicU8 = AtomicU8::new(0);

    fn task_f(state: &mut crate::AgcState) { let _ = state; CALL_LOG.fetch_add(1, Ordering::Relaxed); }
    fn task_g(state: &mut crate::AgcState) { let _ = state; CALL_LOG.fetch_add(10, Ordering::Relaxed); }
    fn task_h(state: &mut crate::AgcState) { let _ = state; CALL_LOG.fetch_add(100, Ordering::Relaxed); }

    fn reset_log() { CALL_LOG.store(0, Ordering::Relaxed); }

    // TC-SC-1: schedule into empty list
    #[test]
    fn tc_sc_1_schedule_empty() {
        let mut wl = Waitlist::new();
        let result = wl.schedule(100, task_f);
        assert_eq!(result, ScheduleResult::OkReloadT3(100));
        assert_eq!(wl.len(), 1);
        assert_eq!(wl.peek(0).unwrap().delta_time, 100);
    }

    // TC-SC-2: schedule earlier task (new earliest)
    #[test]
    fn tc_sc_2_schedule_earlier() {
        let mut wl = Waitlist::new();
        wl.schedule(200, task_f);
        let result = wl.schedule(100, task_g);
        assert_eq!(result, ScheduleResult::OkReloadT3(100));
        assert_eq!(wl.len(), 2);
        assert_eq!(wl.peek(0).unwrap().delta_time, 100); // g at 100
        assert_eq!(wl.peek(1).unwrap().delta_time, 100); // f at 100+100=200
    }

    // TC-SC-3: schedule later task (not earliest)
    #[test]
    fn tc_sc_3_schedule_later() {
        let mut wl = Waitlist::new();
        wl.schedule(50, task_f);
        let result = wl.schedule(100, task_g);
        assert_eq!(result, ScheduleResult::Ok);
        assert_eq!(wl.len(), 2);
        assert_eq!(wl.peek(0).unwrap().delta_time, 50);  // f at 50
        assert_eq!(wl.peek(1).unwrap().delta_time, 50);  // g at 50+50=100
    }

    // TC-SC-4: schedule when full
    #[test]
    fn tc_sc_4_schedule_full() {
        let mut wl = Waitlist::new();
        for i in 1..=8 {
            wl.schedule(i as u16 * 10, task_f);
        }
        assert_eq!(wl.schedule(5, task_g), ScheduleResult::Full);
        assert_eq!(wl.len(), 8);
    }

    // TC-SC-5: insert between two existing entries
    #[test]
    fn tc_sc_5_insert_between() {
        let mut wl = Waitlist::new();
        wl.schedule(100, task_f); // f at 100
        wl.schedule(200, task_g); // g at 200 → deltas: [100, 100]
        let result = wl.schedule(150, task_h); // h at 150 → between f and g
        assert_eq!(result, ScheduleResult::Ok);
        assert_eq!(wl.len(), 3);
        assert_eq!(wl.peek(0).unwrap().delta_time, 100); // f at 100
        assert_eq!(wl.peek(1).unwrap().delta_time, 50);  // h at 100+50=150
        assert_eq!(wl.peek(2).unwrap().delta_time, 50);  // g at 150+50=200
    }

    // TC-DS-1: dispatch single entry
    #[test]
    fn tc_ds_1_dispatch_single() {
        reset_log();
        let mut wl = Waitlist::new();
        wl.schedule(50, task_f);
        let mut state = crate::AgcState::new();
        let next = wl.dispatch(&mut state);
        assert_eq!(next, None); // list empty after dispatch
        assert_eq!(wl.len(), 0);
        assert_eq!(CALL_LOG.load(Ordering::Relaxed), 1); // task_f called
    }

    // TC-DS-2: dispatch with follow-on task
    #[test]
    fn tc_ds_2_dispatch_with_followon() {
        reset_log();
        let mut wl = Waitlist::new();
        wl.schedule(50, task_f);
        wl.schedule(150, task_g); // deltas: [50, 100]
        let mut state = crate::AgcState::new();
        let next = wl.dispatch(&mut state);
        assert_eq!(next, Some(100)); // g's delta for TIME3 reload
        assert_eq!(wl.len(), 1);
        assert_eq!(CALL_LOG.load(Ordering::Relaxed), 1); // task_f called
    }

    // TC-DS-3: dispatch on empty list
    #[test]
    fn tc_ds_3_dispatch_empty() {
        let mut wl = Waitlist::new();
        let mut state = crate::AgcState::new();
        let next = wl.dispatch(&mut state);
        assert_eq!(next, None);
    }
}
