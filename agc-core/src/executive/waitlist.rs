//! Waitlist: delta-time task chain dispatched by T3RUPT.
//!
//! The Waitlist stores up to `MAX_WAITLIST_TASKS` pending tasks, ordered by
//! their scheduled execution time. Each entry stores a *delta time* relative
//! to the previous entry (not an absolute time), so insertion and dispatch
//! are O(N) on the table size, which is bounded at 8.
//!
//! When T3RUPT fires, the front task is dispatched and T3 is re-armed with
//! the next delta-time.
//!
//! AGC source: WAITLIST.agc — WAITLIST / DELAYJOB routines.

use super::MAX_WAITLIST_TASKS;

/// A single entry in the Waitlist delta-time chain.
///
/// AGC source: WAITLIST.agc — TIME3-relative task scheduling.
#[derive(Clone, Copy)]
pub struct WaitlistEntry {
    /// Centiseconds until this task fires relative to the previous entry
    /// (or relative to "now" for the first entry).
    pub delta_cs: u16,
    /// The task function to invoke at the scheduled time.
    pub task: fn(*mut crate::AgcState),
}

/// The Waitlist delta-time chain.
///
/// AGC source: WAITLIST.agc — TASKTEMP / WAITP tables.
pub struct Waitlist {
    entries: [Option<WaitlistEntry>; MAX_WAITLIST_TASKS],
    count: u8,
}

impl Default for Waitlist {
    fn default() -> Self {
        Self::new()
    }
}

impl Waitlist {
    pub const fn new() -> Self {
        Self {
            entries: [None; MAX_WAITLIST_TASKS],
            count: 0,
        }
    }

    /// Schedule `task` to run in `delay_cs` centiseconds from now.
    ///
    /// Inserts the task into the sorted delta-time chain. If the table is
    /// full, the task is silently dropped (overload condition — the alarm
    /// system must handle this at a higher level).
    ///
    /// AGC source: WAITLIST.agc — WAITLIST insertion logic.
    pub fn schedule(&mut self, delay_cs: u16, task: fn(*mut crate::AgcState)) -> bool {
        if self.count as usize >= MAX_WAITLIST_TASKS {
            return false; // table full
        }

        // Find insertion point: walk chain subtracting deltas until remaining
        // time < next entry's delta, then insert.
        let mut remaining = delay_cs;
        let mut insert_at = self.count as usize;

        for i in 0..self.count as usize {
            if let Some(entry) = &self.entries[i] {
                if remaining < entry.delta_cs {
                    insert_at = i;
                    break;
                }
                remaining -= entry.delta_cs;
            }
        }

        // Shift entries right to make room.
        let count = self.count as usize;
        for i in (insert_at..count).rev() {
            self.entries[i + 1] = self.entries[i];
        }

        // Adjust the delta of the entry that now follows the new one.
        if insert_at < count {
            if let Some(next) = &mut self.entries[insert_at + 1] {
                next.delta_cs = next.delta_cs.saturating_sub(remaining);
            }
        }

        self.entries[insert_at] = Some(WaitlistEntry {
            delta_cs: remaining,
            task,
        });
        self.count += 1;
        true
    }

    /// Dispatch and remove the front task, returning it with its delta-time.
    ///
    /// Called by the T3RUPT handler. The caller must re-arm T3 with the
    /// next entry's delta-time (or a minimum interval if the chain is empty).
    ///
    /// AGC source: WAITLIST.agc — T3RUPT dispatch.
    pub fn dispatch_front(&mut self) -> Option<WaitlistEntry> {
        if self.count == 0 {
            return None;
        }
        let front = self.entries[0];
        // Shift entries left.
        for i in 0..self.count as usize - 1 {
            self.entries[i] = self.entries[i + 1];
        }
        self.entries[self.count as usize - 1] = None;
        self.count -= 1;
        front
    }

    /// Return the delta-time of the next scheduled task in centiseconds,
    /// or `None` if the chain is empty.
    pub fn next_delta_cs(&self) -> Option<u16> {
        self.entries[0].map(|e| e.delta_cs)
    }

    /// Number of tasks currently in the chain.
    pub fn len(&self) -> usize {
        self.count as usize
    }

    /// True if no tasks are pending.
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dummy_task(_: *mut crate::AgcState) {}
    fn dummy_task2(_: *mut crate::AgcState) {}
    fn dummy_task3(_: *mut crate::AgcState) {}

    #[test]
    fn schedule_single_task() {
        let mut wl = Waitlist::new();
        assert!(wl.schedule(100, dummy_task));
        assert_eq!(wl.len(), 1);
        assert_eq!(wl.next_delta_cs(), Some(100));
    }

    #[test]
    fn schedule_ordered_insertion() {
        let mut wl = Waitlist::new();
        wl.schedule(200, dummy_task);
        wl.schedule(100, dummy_task2);
        // After insertion of 100-cs task before 200-cs task:
        // chain: [100, 100] (200-100 = 100 remaining for second entry)
        assert_eq!(wl.len(), 2);
        assert_eq!(wl.next_delta_cs(), Some(100));
        let front = wl.dispatch_front().unwrap();
        assert_eq!(front.delta_cs, 100);
        // remaining chain has 1 entry with delta 100
        assert_eq!(wl.next_delta_cs(), Some(100));
    }

    #[test]
    fn schedule_fills_table() {
        let mut wl = Waitlist::new();
        for i in 0..MAX_WAITLIST_TASKS {
            assert!(wl.schedule((i as u16 + 1) * 10, dummy_task));
        }
        // Table full: next insertion fails
        assert!(!wl.schedule(999, dummy_task));
    }

    #[test]
    fn dispatch_empties_table() {
        let mut wl = Waitlist::new();
        wl.schedule(10, dummy_task);
        wl.schedule(20, dummy_task2);
        wl.schedule(5, dummy_task3);
        // Dispatch all
        let a = wl.dispatch_front().unwrap();
        assert_eq!(a.delta_cs, 5);
        let b = wl.dispatch_front().unwrap();
        assert_eq!(b.delta_cs, 5); // 10 - 5
        let c = wl.dispatch_front().unwrap();
        assert_eq!(c.delta_cs, 10); // 20 - 10
        assert!(wl.is_empty());
    }
}
