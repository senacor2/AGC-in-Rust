//! Job table entry for the Executive CORE SET.
//!
//! AGC source: EXECUTIVE.agc — CORE SET table layout.

/// A job in the Executive CORE SET table.
///
/// Jobs are longer computations that the Executive schedules cooperatively.
/// Each job has an assigned priority; higher priority preempts lower.
/// Priority 0 means the slot is empty.
///
/// The dummy job (idle loop when nothing else is ready) runs at priority 0
/// and is not stored in the table — it is the implicit fallback.
///
/// AGC source: EXECUTIVE.agc — JOBSLIST / CORE SET table.
#[derive(Clone, Copy)]
pub struct JobEntry {
    /// Priority: 0 = empty slot; 1–7 = active (7 = highest).
    pub priority: u8,
    /// The job function to execute.
    pub entry: fn(*mut crate::AgcState),
    /// The major mode program that owns this job (for restart dispatch).
    pub major_mode: u8,
}

impl JobEntry {
    /// Construct a new job entry.
    pub const fn new(priority: u8, entry: fn(*mut crate::AgcState), major_mode: u8) -> Self {
        Self {
            priority,
            entry,
            major_mode,
        }
    }
}
