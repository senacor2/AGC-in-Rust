//! Job entry type for the AGC Executive scheduler.
//!
//! AGC source: Comanche055/EXECUTIVE.agc (core set layout)
//!             Comanche055/ERASABLE_ASSIGNMENTS.agc lines 1618-1627
//!             (MPAC+0..PRIORITY, 12 registers per core set × 7 sets = 84 words)

use crate::AgcState;

/// Priority of a job. Higher value = higher urgency.
///
/// Valid range: 1..=0x7FFF. Zero is reserved for "slot free".
///
/// AGC scale: priority `OCT 10000` ≈ decimal 4096 is low;
/// `OCT 37777` = 16383 is highest representable.
/// AGC source: Comanche055/RESTART_TABLES.agc priority fields.
pub type Priority = u16;

/// A function pointer representing the job entry point.
///
/// Must be a plain `fn` — no closures, no captures, no heap.
/// Each function receives a mutable reference to the entire AGC state.
///
/// AGC source: Comanche055/EXECUTIVE.agc LOC/BANKSET in each core set.
pub type JobFn = fn(&mut AgcState);

/// A Vector Accumulator area index (0-based, 0..=4).
///
/// `None` means the job does not require a VAC area (NOVAC path).
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc VAC1USE-VAC5USE lines 1727-1736.
pub type VacIndex = u8;

/// One entry in the 7-slot job table (one AGC "core set").
///
/// Corresponds to one AGC "core set" (MPAC +0 through PRIORITY).
/// The 12-word core set is collapsed to the fields needed by the Rust scheduler.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc lines 1618-1627.
pub struct JobEntry {
    /// Job entry point. `None` means the slot is free.
    ///
    /// AGC source: LOC + BANKSET registers in the core set.
    pub entry: Option<JobFn>,

    /// Priority. 0 means free (corresponds to `-0` in ones-complement).
    /// Negative (sleeping) is encoded by the `sleeping` flag.
    pub priority: Priority,

    /// True when the job is suspended waiting for a JOBWAKE event.
    ///
    /// AGC source: JOBSLEEP stores `-C(PRIORITY)` to indicate sleep.
    pub sleeping: bool,

    /// Optional VAC area claimed by this job (FINDVAC path).
    ///
    /// AGC source: low 9 bits of the PRIORITY word when VAC is in use.
    pub vac: Option<VacIndex>,
}

impl JobEntry {
    /// Construct a free (vacant) slot.
    pub const fn free() -> Self {
        Self {
            entry: None,
            priority: 0,
            sleeping: false,
            vac: None,
        }
    }

    /// True when this slot is occupied by an active or sleeping job.
    pub fn is_occupied(&self) -> bool {
        self.entry.is_some() && self.priority > 0
    }

    /// True when this slot is free (no job assigned).
    pub fn is_free(&self) -> bool {
        self.entry.is_none() || self.priority == 0
    }

    /// True when the job is runnable (occupied and not sleeping).
    pub fn is_runnable(&self) -> bool {
        self.is_occupied() && !self.sleeping
    }
}
