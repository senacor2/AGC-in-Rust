/// Job priority. 0 = slot empty / idle. Higher value = higher priority.
/// The original AGC used octal values such as 37 for the autopilot job.
pub type JobPriority = u8;

/// Maximum number of concurrent executive jobs (matches the AGC CORE SET table).
pub const MAX_JOBS: usize = 7;

/// A single entry in the Executive job table.
#[derive(Clone, Copy)]
pub struct JobEntry {
    /// Priority of this job. 0 means the slot is empty.
    pub priority: JobPriority,
    /// The function that implements this job's computation.
    /// The job runs until it returns; preemption happens between invocations.
    pub entry: fn(&mut crate::AgcState),
    /// Major mode (program number) that created this job.
    /// Used by the restart mechanism to re-dispatch after a power-on restart.
    pub major_mode: u8,
}

impl JobEntry {
    /// An empty (unused) job slot.
    pub const EMPTY: Self = Self {
        priority: 0,
        entry: |_| {},
        major_mode: 0,
    };
}
