//! Bare-metal entry point for the AGC-Core flight software.
//!
//! This file is compiled only when building for bare-metal targets
//! (`cfg(not(test))`).  Host unit tests link against `lib.rs` directly and
//! do not need an entry point.
//!
//! The entry point mirrors the AGC GOJAM handler entry at ROM address 4000:
//!   1. Initialise hardware (clocks, peripherals).
//!   2. Construct zeroed `AgcState`.
//!   3. Execute FRESH START (SLAP1 / DOFSTART).
//!   4. Enter the Executive idle loop (DUMMYJOB equivalent) — never returns.
//!
//! AGC source: Comanche055/FRESH_START_AND_RESTART.agc SLAP1, DOFSTART (page 183-185).
#![no_main]

// `main` is provided by the BSP / linker script via a `#[entry]` attribute
// from the device PAC.  In Milestone 1 we do not yet depend on a concrete PAC,
// so this file merely documents the intended structure.
//
// The actual bare-metal entry will be wired in a later milestone when the
// target board and PAC are selected.  For now this file exists so that
// `cargo build --target thumbv7em-none-eabihf` can locate it.

use agc_core::{services::fresh_start, AgcState};

/// AGC boot / restart entry point.
///
/// On bare-metal this is invoked by the reset vector.  The function never
/// returns — control stays in the Executive idle loop.
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc SLAP1 (page 183).
#[cfg(not(test))]
#[no_mangle]
pub extern "C" fn agc_main() -> ! {
    let mut state = AgcState::new();

    // FRESH START — mirrors SLAP1/DOFSTART.
    // Hardware handle is a placeholder until the BSP milestone.
    // fresh_start::fresh_start(&mut state, &mut hw);

    // Executive idle loop — mirrors DUMMYJOB.
    loop {
        // executive::scheduler::step(&mut state, &mut hw);
        let _ = &mut state;
        core::hint::spin_loop();
    }
}
