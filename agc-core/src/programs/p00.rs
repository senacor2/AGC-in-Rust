//! P00 — CMC Idle.
//!
//! The lowest-priority background job. Maintains coasting state-vector
//! propagation when no active navigation program is running.

use crate::executive::job::JobPriority;

pub const PRIORITY: JobPriority = 1;

pub fn init(state: &mut crate::AgcState) -> JobPriority {
    let _ = state;
    todo!("P00 init")
}
