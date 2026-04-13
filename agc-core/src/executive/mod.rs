//! AGC Executive: cooperative scheduler, Waitlist, and restart protection.
//!
//! AGC source: Comanche055/EXECUTIVE.agc, WAITLIST.agc,
//!             FRESH_START_AND_RESTART.agc, RESTART_TABLES.agc

pub mod job;
pub mod restart;
pub mod scheduler;
pub mod waitlist;

pub use job::{JobEntry, JobFn, Priority, VacIndex};
pub use restart::{
    GroupId, GroupState, Phase, RestartAction, RestartProtection, RestartTables, NUMGRPS,
    NUM_RESTART_GROUPS,
};
pub use scheduler::{Executive, MAX_JOBS, MAX_VAC_AREAS};
pub use waitlist::{TaskFn, Waitlist, MAX_DELTA_CS, MAX_WAITLIST_TASKS, MIN_DELTA_CS};
