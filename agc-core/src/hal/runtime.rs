//! ISR-to-foreground rendezvous flags for the bare-metal Executive.
//!
//! Each AGC timer interrupt has a corresponding `AtomicBool`. The ISR sets
//! the flag (Release ordering); the Executive's main loop drains it with
//! `swap(false, Acquire)` and runs the matching action. On host (tests /
//! sim), the flags simply reside in BSS and remain false unless a test
//! sets them directly — no special handling required.

use core::sync::atomic::AtomicBool;

/// Set by the TIM2 ISR (T3RUPT). Cleared by the Executive drain loop.
pub static T3_PENDING: AtomicBool = AtomicBool::new(false);

/// Set by the TIM3 ISR (T4RUPT). Cleared by the Executive drain loop.
pub static T4_PENDING: AtomicBool = AtomicBool::new(false);

/// Set by the TIM4 ISR (T5RUPT). Cleared by the Executive drain loop.
pub static T5_PENDING: AtomicBool = AtomicBool::new(false);

/// Set by the TIM5 ISR (T6RUPT). Cleared by the Executive drain loop.
pub static T6_PENDING: AtomicBool = AtomicBool::new(false);
