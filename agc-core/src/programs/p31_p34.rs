//! P33 — Transfer Phase Initiation (TPI).
//! P34 — Transfer Phase Midcourse (TPM).
//!
//! P31 and P32 have been split into their own modules (`programs/p31.rs`,
//! `programs/p32.rs`). This stub file now only holds the P33/P34 entry
//! points until Phase 6 replaces them.

use crate::executive::job::JobPriority;

pub fn init_p33(state: &mut crate::AgcState) -> JobPriority { let _ = state; todo!("P33 init") }
pub fn init_p34(state: &mut crate::AgcState) -> JobPriority { let _ = state; todo!("P34 init") }
