//! SERVICER — the 2-second navigation cycle (average-G integration).
//!
//! Reads PIPA delta-V counts, rotates them to the reference frame via REFSMMAT,
//! adds gravity, and integrates the state vector. Scheduled as a repeating
//! waitlist task by programs that require active navigation (P11, P20, P40, entry).
