/// Maximum number of concurrently pending waitlist tasks.
pub const MAX_WAITLIST_TASKS: usize = 8;

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

/// The Waitlist — a sorted table of pending time-triggered tasks.
///
/// Tasks are dispatched by T3RUPT (TIME3 overflow). The list is kept
/// sorted by absolute fire time so only the earliest task's delta needs
/// to be loaded into TIME3.
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

    /// Schedule a task to run `centiseconds` from now.
    /// Returns `false` if the waitlist is full (alarm 1211 should be raised).
    pub fn schedule(&mut self, centiseconds: u16, task: fn(&mut crate::AgcState)) -> bool {
        if self.count >= MAX_WAITLIST_TASKS {
            return false;
        }
        // Insert sorted by delta time. Full implementation in waitlist module.
        let _ = centiseconds;
        let _ = task;
        todo!("insert task into sorted waitlist")
    }

    /// Dispatch the earliest pending task.
    /// Called by the T3RUPT handler. Returns the delta time to the next task
    /// (to reload TIME3), or `None` if the list is now empty.
    pub fn dispatch(&mut self, state: &mut crate::AgcState) -> Option<u16> {
        let _ = state;
        todo!("dispatch earliest task and return next delta")
    }

    /// Number of currently scheduled tasks.
    pub fn len(&self) -> usize {
        self.count
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }
}
