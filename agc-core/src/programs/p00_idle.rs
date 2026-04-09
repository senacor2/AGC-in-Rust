//! P00 — CMC Idling.
//!
//! The simplest program. When no mission program is active the CMC idles,
//! waiting for crew input via Verb 37 on the DSKY.
//!
//! AGC source: Comanche055/FRESH_START_AND_RESTART.agc — GOTOPOOH routine.

use super::ProgramAction;

/// Program number for CMC idle.
pub const PROG_NUMBER: u8 = 0;

/// P00 state — the CMC is idle, waiting for crew input (V37).
///
/// AGC source: FRESH_START_AND_RESTART.agc — GOTOPOOH initialises the DSKY
/// to display program 00 and waits for V37 (Go To Other Program).
#[derive(Clone, Copy, Debug, Default)]
pub struct P00State {
    /// True after fresh start initialisation is complete.
    pub initialized: bool,
}

impl P00State {
    /// Construct a new, uninitialised P00 state.
    pub const fn new() -> Self {
        Self { initialized: false }
    }

    /// Initialise P00 — called after FRESH START.
    ///
    /// Sets the program display to P00 and enables the crew to invoke any
    /// other program via V37.  No navigation computations are performed.
    ///
    /// AGC source: FRESH_START_AND_RESTART.agc — GOTOPOOH.
    pub fn enter(&mut self) {
        self.initialized = true;
    }

    /// P00 cycle function — the CMC does nothing; it waits for V37 crew input.
    ///
    /// Returns [`ProgramAction::Idle`] unconditionally.
    pub fn cycle(&self) -> ProgramAction {
        ProgramAction::Idle
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_state_is_not_initialized() {
        let p00 = P00State::new();
        assert!(!p00.initialized);
    }

    #[test]
    fn enter_sets_initialized() {
        let mut p00 = P00State::new();
        p00.enter();
        assert!(p00.initialized);
    }

    #[test]
    fn cycle_returns_idle() {
        let p00 = P00State::new();
        assert_eq!(p00.cycle(), ProgramAction::Idle);
    }
}
