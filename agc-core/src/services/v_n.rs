//! Verb/Noun processor (PINBALL).
//!
//! State machine that assembles crew keystrokes into Verb/Noun commands
//! and dispatches them to the appropriate handler. Driven by
//! `feed_key(state, key)` which is called from the KEYRUPT ISR shim
//! (bare metal) or from the test harness.
//!
//! **Milestone 6 Phase 1 scope**: V37 (program select), V06 / V16
//! (display), V34 (terminate), V35 (lamp test). Data-entry verbs and
//! crew-acknowledgement verbs are later phases.
//!
//! AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc,
//!             Comanche055/PINBALL_NOUN_TABLES.agc,
//!             Comanche055/KEYRUPT,_UPRUPT.agc.

use crate::programs::PROGRAM_TABLE;

// ── Key codes ─────────────────────────────────────────────────────────────────

/// Canonical DSKY keys.
///
/// Code values match the Block 2 AGC KEYTEMP1 table from
/// `PINBALL_GAME_BUTTONS_AND_LIGHTS.agc`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Key {
    Digit(u8), // 0..9
    Verb,
    Noun,
    Plus,
    Minus,
    Clr,
    Pro,
    KeyRel,
    Entr,
    Rset,
}

impl Key {
    /// Convert a raw 5-bit HAL keypress code into a `Key`.
    ///
    /// Returns `None` for unknown codes.
    pub fn from_code(code: u8) -> Option<Self> {
        match code {
            1..=9 => Some(Key::Digit(code)),
            16 => Some(Key::Digit(0)),
            17 => Some(Key::Verb),
            18 => Some(Key::Rset),
            25 => Some(Key::Pro),     // also KeyRel in hardware
            26 => Some(Key::Plus),
            27 => Some(Key::Minus),
            28 => Some(Key::Entr),
            30 => Some(Key::Clr),
            31 => Some(Key::Noun),
            _ => None,
        }
    }
}

// ── Phase and state ───────────────────────────────────────────────────────────

/// Current state of the V/N input state machine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VnPhase {
    /// Nothing in progress — waiting for VERB or a control key.
    Idle,
    /// VERB pressed, accumulating up to two digits.
    EnteringVerb { digits: u8, buf: u8 },
    /// NOUN pressed after verb complete, accumulating up to two digits.
    EnteringNoun { verb: u8, digits: u8, buf: u8 },
    /// Operator error — awaiting RSET.
    OprErr,
}

impl Default for VnPhase {
    fn default() -> Self {
        VnPhase::Idle
    }
}

/// Crew interface Verb/Noun input state.
#[derive(Clone, Copy, Debug, Default)]
pub struct VnState {
    pub phase: VnPhase,
}

impl VnState {
    /// `const` constructor usable inside `AgcState::new`.
    pub const fn new() -> Self {
        Self { phase: VnPhase::Idle }
    }
}

// ── Entry point ───────────────────────────────────────────────────────────────

/// Feed a single keypress into the V/N processor.
///
/// Drives the state machine and, when a complete VERB+NOUN+ENTR (or
/// VERB+ENTR for noun-less verbs) sequence is recognised, dispatches
/// to the appropriate handler.
pub fn feed_key(state: &mut crate::AgcState, key: Key) {
    use VnPhase::*;

    // Global keys that reset regardless of phase.
    if key == Key::Rset {
        state.vn.phase = Idle;
        state.dsky.opr_err = false;
        return;
    }
    if key == Key::Clr {
        state.vn.phase = Idle;
        return;
    }
    // VERB always restarts the entry — matches AGC behaviour.
    if key == Key::Verb {
        state.vn.phase = EnteringVerb { digits: 0, buf: 0 };
        return;
    }

    match state.vn.phase {
        OprErr => {
            // OPR ERR is only cleared by RSET (handled above).
        }

        Idle => {
            // Any non-VERB, non-RSET key in Idle is an error.
            raise_opr_err(state);
        }

        EnteringVerb { digits, buf } => match key {
            Key::Digit(d) => {
                if digits >= 2 {
                    raise_opr_err(state);
                    return;
                }
                let new_buf = buf * 10 + d;
                state.vn.phase = EnteringVerb {
                    digits: digits + 1,
                    buf: new_buf,
                };
            }
            Key::Noun => {
                if digits != 2 {
                    raise_opr_err(state);
                    return;
                }
                state.vn.phase = EnteringNoun {
                    verb: buf,
                    digits: 0,
                    buf: 0,
                };
            }
            Key::Entr => {
                if digits != 2 {
                    raise_opr_err(state);
                    return;
                }
                // Verbs that take no noun: V35 (lamp test), V34 (terminate).
                if verb_takes_no_noun(buf) {
                    dispatch_verb_noun(state, buf, 0);
                    if state.vn.phase != OprErr {
                        state.vn.phase = Idle;
                    }
                } else {
                    raise_opr_err(state);
                }
            }
            _ => raise_opr_err(state),
        },

        EnteringNoun { verb, digits, buf } => match key {
            Key::Digit(d) => {
                if digits >= 2 {
                    raise_opr_err(state);
                    return;
                }
                let new_buf = buf * 10 + d;
                state.vn.phase = EnteringNoun {
                    verb,
                    digits: digits + 1,
                    buf: new_buf,
                };
            }
            Key::Entr => {
                if digits != 2 {
                    raise_opr_err(state);
                    return;
                }
                dispatch_verb_noun(state, verb, buf);
                // Only return to Idle if dispatch did not raise OPR ERR.
                if state.vn.phase != OprErr {
                    state.vn.phase = Idle;
                }
            }
            _ => raise_opr_err(state),
        },
    }
}

// ── Dispatch ──────────────────────────────────────────────────────────────────

/// Returns true for verbs that do not require a noun (V34, V35, etc.).
fn verb_takes_no_noun(verb: u8) -> bool {
    matches!(verb, 34 | 35)
}

/// Dispatch a completed VERB+NOUN (or noun-less VERB) command.
fn dispatch_verb_noun(state: &mut crate::AgcState, verb: u8, noun: u8) {
    match verb {
        6 => v06_display_decimal(state, noun),
        16 => v16_monitor(state, noun),
        34 => v34_terminate(state),
        35 => v35_lamp_test(state),
        37 => v37_program_select(state, noun),
        _ => raise_opr_err(state),
    }
}

// ── Verb handlers ─────────────────────────────────────────────────────────────

/// V06 — Display decimal.
fn v06_display_decimal(state: &mut crate::AgcState, noun: u8) {
    state.dsky.verb = 6;
    state.dsky.noun = noun;
    state.dsky.flashing = false;
}

/// V16 — Continuous monitor display.
fn v16_monitor(state: &mut crate::AgcState, noun: u8) {
    state.dsky.verb = 16;
    state.dsky.noun = noun;
    state.dsky.flashing = false;
}

/// V34 — Terminate active program: return to P00.
fn v34_terminate(state: &mut crate::AgcState) {
    let _ = crate::programs::p00::init(state);
}

/// V35 — Lamp test.
fn v35_lamp_test(state: &mut crate::AgcState) {
    state.dsky.lamp_test_active = true;
}

/// V37 — Select major mode / program.
fn v37_program_select(state: &mut crate::AgcState, noun: u8) {
    let slot = noun as usize;
    if slot >= PROGRAM_TABLE.len() {
        raise_opr_err(state);
        return;
    }
    match PROGRAM_TABLE[slot] {
        Some(init_fn) => {
            let _prio = init_fn(state);
        }
        None => raise_opr_err(state),
    }
}

// ── Error helper ──────────────────────────────────────────────────────────────

/// Raise the OPR ERR indicator and return the V/N state to `OprErr`.
fn raise_opr_err(state: &mut crate::AgcState) {
    state.dsky.opr_err = true;
    state.vn.phase = VnPhase::OprErr;
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AgcState;

    /// Convenience: feed a slice of keys in order.
    fn feed(state: &mut AgcState, keys: &[Key]) {
        for &k in keys {
            feed_key(state, k);
        }
    }

    /// Shorthand: decimal digit.
    fn d(n: u8) -> Key {
        Key::Digit(n)
    }

    // ── TC-VN-1: Key::from_code round trip ────────────────────────────────────

    #[test]
    fn tc_vn_1_key_from_code() {
        assert_eq!(Key::from_code(1), Some(Key::Digit(1)));
        assert_eq!(Key::from_code(9), Some(Key::Digit(9)));
        assert_eq!(Key::from_code(16), Some(Key::Digit(0)));
        assert_eq!(Key::from_code(17), Some(Key::Verb));
        assert_eq!(Key::from_code(28), Some(Key::Entr));
        assert_eq!(Key::from_code(30), Some(Key::Clr));
        assert_eq!(Key::from_code(31), Some(Key::Noun));
        assert_eq!(Key::from_code(255), None);
        assert_eq!(Key::from_code(0), None);
    }

    // ── TC-VN-2: V37E00E selects P00 ──────────────────────────────────────────

    #[test]
    fn tc_vn_2_v37_e00_e_selects_p00() {
        let mut state = AgcState::new();
        state.major_mode = 42; // nonzero starting mode

        feed(
            &mut state,
            &[Key::Verb, d(3), d(7), Key::Noun, d(0), d(0), Key::Entr],
        );

        assert_eq!(state.major_mode, 0, "V37E00E must invoke P00 init");
        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert!(!state.dsky.opr_err);
    }

    // ── TC-VN-3: V37E30E selects P30 ──────────────────────────────────────────

    #[test]
    fn tc_vn_3_v37_e30_e_selects_p30() {
        let mut state = AgcState::new();

        feed(
            &mut state,
            &[Key::Verb, d(3), d(7), Key::Noun, d(3), d(0), Key::Entr],
        );

        assert_eq!(state.major_mode, 30);
        assert_eq!(state.vn.phase, VnPhase::Idle);
    }

    // ── TC-VN-4: V06N40E sets the display ─────────────────────────────────────

    #[test]
    fn tc_vn_4_v06_n40_e_sets_display() {
        let mut state = AgcState::new();

        feed(
            &mut state,
            &[Key::Verb, d(0), d(6), Key::Noun, d(4), d(0), Key::Entr],
        );

        assert_eq!(state.dsky.verb, 6);
        assert_eq!(state.dsky.noun, 40);
        assert!(!state.dsky.flashing);
        assert_eq!(state.vn.phase, VnPhase::Idle);
    }

    // ── TC-VN-5: V34E terminates to P00 ───────────────────────────────────────

    #[test]
    fn tc_vn_5_v34_e_terminates_to_p00() {
        let mut state = AgcState::new();
        state.major_mode = 40;

        feed(&mut state, &[Key::Verb, d(3), d(4), Key::Entr]);

        assert_eq!(state.major_mode, 0);
        assert_eq!(state.vn.phase, VnPhase::Idle);
    }

    // ── TC-VN-6: V35E sets lamp_test_active ───────────────────────────────────

    #[test]
    fn tc_vn_6_v35_e_lamp_test() {
        let mut state = AgcState::new();

        feed(&mut state, &[Key::Verb, d(3), d(5), Key::Entr]);

        assert!(state.dsky.lamp_test_active);
    }

    // ── TC-VN-7: Unknown verb raises OPR ERR ──────────────────────────────────

    #[test]
    fn tc_vn_7_unknown_verb_opr_err() {
        let mut state = AgcState::new();

        feed(
            &mut state,
            &[Key::Verb, d(9), d(9), Key::Noun, d(0), d(0), Key::Entr],
        );

        assert!(state.dsky.opr_err);
        assert_eq!(state.vn.phase, VnPhase::OprErr);
    }

    // ── TC-VN-8: RSET clears OPR ERR ──────────────────────────────────────────

    #[test]
    fn tc_vn_8_rset_clears_opr_err() {
        let mut state = AgcState::new();
        state.dsky.opr_err = true;
        state.vn.phase = VnPhase::OprErr;

        feed_key(&mut state, Key::Rset);

        assert!(!state.dsky.opr_err);
        assert_eq!(state.vn.phase, VnPhase::Idle);
    }

    // ── TC-VN-9: VERB during EnteringNoun restarts the entry ──────────────────

    #[test]
    fn tc_vn_9_verb_during_noun_restarts() {
        let mut state = AgcState::new();

        feed(
            &mut state,
            &[Key::Verb, d(3), d(7), Key::Noun, d(3), Key::Verb],
        );

        assert_eq!(
            state.vn.phase,
            VnPhase::EnteringVerb { digits: 0, buf: 0 }
        );
    }

    // ── TC-VN-10: CLR from EnteringVerb returns to Idle ───────────────────────

    #[test]
    fn tc_vn_10_clr_cancels_entry() {
        let mut state = AgcState::new();

        feed(&mut state, &[Key::Verb, d(3), Key::Clr]);

        assert_eq!(state.vn.phase, VnPhase::Idle);
    }

    // ── TC-VN-11: V37 with unknown program raises OPR ERR ────────────────────

    #[test]
    fn tc_vn_11_v37_unknown_program_opr_err() {
        let mut state = AgcState::new();
        // Slot 99 is None in PROGRAM_TABLE.
        feed(
            &mut state,
            &[Key::Verb, d(3), d(7), Key::Noun, d(9), d(9), Key::Entr],
        );

        assert!(state.dsky.opr_err);
    }

    // ── TC-VN-12: Single-digit verb + NOUN raises OPR ERR ─────────────────────

    #[test]
    fn tc_vn_12_single_digit_verb_then_noun_error() {
        let mut state = AgcState::new();

        feed(&mut state, &[Key::Verb, d(3), Key::Noun]);

        assert_eq!(state.vn.phase, VnPhase::OprErr);
        assert!(state.dsky.opr_err);
    }

    // ── Extra: V37E11E selects P11 and sets major_mode = 11 ──────────────────

    #[test]
    fn tc_vn_13_v37_e11_e_selects_p11() {
        use crate::navigation::gravity::MU_EARTH;
        use crate::navigation::state_vector::{Frame, StateVector};
        use crate::navigation::gravity::R_EARTH;
        use crate::types::Met;

        let mut state = AgcState::new();
        // P11 requires EarthInertial frame — seed a 400 km LEO.
        let r = R_EARTH + 400_000.0;
        let v = libm::sqrt(MU_EARTH / r);
        state.csm_state = StateVector {
            position: [r, 0.0, 0.0],
            velocity: [0.0, v, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        feed(
            &mut state,
            &[Key::Verb, d(3), d(7), Key::Noun, d(1), d(1), Key::Entr],
        );

        assert_eq!(state.major_mode, 11);
        assert_eq!(state.dsky.prog, 11);
        assert!(!state.dsky.opr_err);
    }
}
