//! Executive and Waitlist: the AGC real-time scheduler.
//!
//! The Executive manages two levels of computation:
//!
//! - **Waitlist tasks** (short, time-critical, non-preemptable): dispatched by
//!   T3RUPT at their scheduled time. Run to completion.
//! - **Jobs** (longer, preemptable): queued in the CORE SET table, dispatched
//!   by the Executive main loop in priority order.
//!
//! AGC source: EXECUTIVE.agc, WAITLIST.agc.

pub mod job;
pub mod restart;
pub mod scheduler;
pub mod waitlist;

pub use scheduler::Executive;

/// Maximum number of jobs in the Executive CORE SET table.
///
/// AGC source: EXECUTIVE.agc — the CORE SET table has 7 slots.
pub const MAX_JOBS: usize = 7;

/// Maximum number of pending entries in the Waitlist delta-time chain.
///
/// AGC source: WAITLIST.agc — "9 TASKS MAXIMUM"; LST2 ERASE +17D (9 two-word
/// 2CADR pairs = 18 words). Previously wrong at 8.
pub const MAX_WAITLIST_TASKS: usize = 9;
