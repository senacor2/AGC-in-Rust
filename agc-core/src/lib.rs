//! AGC-in-Rust: Comanche055 Command Module guidance software in idiomatic Rust.
//!
//! This crate is `no_std` and allocates nothing on the heap.
//! All state is statically allocated; the Executive owns the scheduler.

#![no_std]
#![deny(unsafe_op_in_unsafe_fn)]

pub mod executive;
pub mod hal;
pub mod math;
pub mod navigation;
pub mod services;
pub mod types;

#[cfg(test)]
mod tests;

/// Top-level state threaded through every component.
///
/// `AgcState` is the single authoritative view of all mutable navigation and
/// scheduling data. It is passed by mutable reference through every function
/// that needs it. Interrupts that share state with foreground code use the
/// `cortex_m::interrupt::Mutex<RefCell<T>>` pattern.
pub struct AgcState {
    pub executive: executive::Executive,
    pub restart: executive::restart::RestartProtection,
    pub alarms: services::alarm::AlarmState,
}

impl Default for AgcState {
    fn default() -> Self {
        Self::new()
    }
}

impl AgcState {
    pub const fn new() -> Self {
        Self {
            executive: executive::Executive::new(),
            restart: executive::restart::RestartProtection::new(),
            alarms: services::alarm::AlarmState::new(),
        }
    }
}
