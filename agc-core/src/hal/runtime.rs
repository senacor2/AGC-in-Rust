//! ISR-to-foreground rendezvous flags for the bare-metal Executive.
//!
//! Each AGC timer interrupt has a corresponding `AtomicBool`. The ISR sets
//! the flag (Release ordering); the Executive's main loop drains it with
//! `swap(false, Acquire)` and runs the matching action. On host (tests /
//! sim), the flags simply reside in BSS and remain false unless a test
//! sets them directly — no special handling required.
//!
//! ## Demo hook
//!
//! `DEMO_HOOK` allows the board crate to register a `fn(*mut ())` callback
//! that is called (with a null argument) each time the T3 drain path fires
//! during the Phase-3 demo. This keeps agc-core free of `defmt` and other
//! board-specific dependencies: the board registers a closure-equivalent at
//! boot by atomically writing a function pointer; `__demo_tick` reads and
//! calls it if non-null.

use core::sync::atomic::{AtomicBool, AtomicPtr, Ordering};

/// Set by the TIM2 ISR (T3RUPT). Cleared by the Executive drain loop.
pub static T3_PENDING: AtomicBool = AtomicBool::new(false);

/// Set by the TIM3 ISR (T4RUPT). Cleared by the Executive drain loop.
pub static T4_PENDING: AtomicBool = AtomicBool::new(false);

/// Set by the TIM4 ISR (T5RUPT). Cleared by the Executive drain loop.
pub static T5_PENDING: AtomicBool = AtomicBool::new(false);

/// Set by the TIM5 ISR (T6RUPT). Cleared by the Executive drain loop.
pub static T6_PENDING: AtomicBool = AtomicBool::new(false);

/// Counts how many times the T3 drain path has fired.
/// The board crate's idle/demo path reads this to emit per-tick log lines
/// without importing `defmt` into agc-core.
pub static T3_TICK_COUNT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Optional board-side demo callback.
///
/// The board crate stores a `fn(&mut crate::AgcState)` function pointer here
/// (cast to `*mut ()`) during init. `__demo_tick` reads and calls it when
/// the T3 drain path creates the demo job. Null (default) means no-op.
pub static DEMO_HOOK: AtomicPtr<()> = AtomicPtr::new(core::ptr::null_mut());

/// Register a board-side demo callback. Called once from `bin/agc.rs` init.
///
/// # Safety
/// The function pointer must remain valid for the lifetime of the program.
/// On bare metal this is always true — function pointers point into flash.
pub fn register_demo_hook(f: fn(&mut crate::AgcState)) {
    // SAFETY: `fn` pointer cast to `*mut ()` for atomic storage. The pointer
    // is only ever written here at init (single-threaded, before interrupts
    // are enabled) and read in `__demo_tick` via `load`. Round-trip through
    // `*mut ()` preserves bit representation on all Cortex-M targets.
    DEMO_HOOK.store(f as *mut (), Ordering::Release);
}
