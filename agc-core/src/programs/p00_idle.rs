//! P00 — CMC Idle Program (POOH / DUMMYJOB).
//!
//! P00 is the resting state of the CMC. The computer returns to P00 whenever
//! no active navigation or guidance program is running. It sets MODREG to 0
//! and displays "00" in the PROG field.
//!
//! AGC source: Comanche055/FRESH_START_AND_RESTART.agc
//!   GOTOPOOH (page 194), POOH (page 200), GOP00FIX (page 195).
//! AGC source: Comanche055/EXECUTIVE.agc
//!   DUMMYJOB (page 1220).

use crate::hal::{AgcHardware, DskyIo};
use crate::{AgcState, MODREG_NONE};

/// Sentinel value for "P00 is active": MODREG = 0 (zero major mode).
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc POOH: `CS ZERO; TS MODREG`.
pub const MODREG_P00: i16 = 0;

/// Enter the P00 idle state.
///
/// Sets `state.modreg` to `MODREG_P00` (zero).
/// Writes "00" to the DSKY PROG display.
/// Sets Group 1 restart phase to protect the GOTOPOOH → GOP00FIX transition.
///
/// Does NOT affect the Executive job table or Waitlist.
/// Does NOT command any hardware (engines, jets, IMU).
/// Is idempotent: calling while already in P00 is a no-op.
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc
///   GOTOPOOH (page 194), POOH (page 200), GOP00FIX (page 195).
pub fn enter<H: AgcHardware>(state: &mut AgcState, hw: &mut H) {
    // Restart protection: TC PHASCHNG / OCT 14 sets Group 1 phase = 4.
    // AGC source: GOTOPOOH calls TC PHASCHNG / OCT 14.
    state.restart.set_phase(1, 4, false, 0);

    // Set MODREG to 0 (P00).
    state.modreg = MODREG_P00;

    // Show "00" in PROG display.
    // AGC source: GOP00FIX calls CLEANDSP which zeros the prog display.
    hw.dsky().write_prog(0);
}

/// Returns `true` when the current major mode register indicates P00 is active.
///
/// Condition: `state.modreg == MODREG_P00` (value 0, the zero major-mode).
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc V37 routine at label
///   `ISITP00`: `CA MMNUMBER / EXTEND / BZF ISSERVON`.
pub fn is_active(state: &AgcState) -> bool {
    state.modreg == MODREG_P00
}

/// Returns `true` when no program is active (MODREG = MODREG_NONE = -1).
///
/// This is the state immediately after power-on / fresh start before P00 has
/// been formally entered.
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc DOFSTART — `CS ZERO; TS MODREG`
///   stores -0 (ones-complement) which maps to `MODREG_NONE = -1` in Rust.
pub fn is_uninitialised(state: &AgcState) -> bool {
    state.modreg == MODREG_NONE
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests::mock_hw::MockHardware;
    type SimHardware = MockHardware;

    /// TC-P00-1: Enter sets modreg to zero.
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc POOH label.
    #[test]
    fn enter_sets_modreg_zero() {
        let mut state = AgcState::new();
        state.modreg = 11; // simulate P11 active
        let mut hw = SimHardware::new();
        enter(&mut state, &mut hw);
        assert_eq!(state.modreg, 0);
        assert!(is_active(&state));
    }

    /// TC-P00-2: is_active is false when another program is active.
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc ISITP00 label.
    #[test]
    fn is_active_false_when_other_program_set() {
        let mut state = AgcState::new();
        state.modreg = 30; // P30 active
        assert!(!is_active(&state));
    }

    /// TC-P00-3: Enter is idempotent.
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc GOTOPOOH — safe to call twice.
    #[test]
    fn enter_is_idempotent() {
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        enter(&mut state, &mut hw);
        enter(&mut state, &mut hw); // second call must not panic or corrupt state
        assert_eq!(state.modreg, 0);
        assert!(is_active(&state));
    }

    /// TC-P00-4: Fresh-start state has MODREG_NONE (-1), not P00.
    #[test]
    fn fresh_state_is_uninitialised() {
        let state = AgcState::new();
        assert!(is_uninitialised(&state));
        assert!(!is_active(&state));
    }
}
