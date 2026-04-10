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
use crate::types::{Met, Vec3};

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
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum VnPhase {
    /// Nothing in progress — waiting for VERB or a control key.
    Idle,
    /// VERB pressed, accumulating up to two digits.
    EnteringVerb { digits: u8, buf: u8 },
    /// NOUN pressed after verb complete, accumulating up to two digits.
    EnteringNoun { verb: u8, digits: u8, buf: u8 },
    /// Data entry in progress for a V21/V22/V23/V25 load.
    EnteringData {
        /// Initiating verb (21, 22, 23, or 25).
        verb: u8,
        /// Target noun.
        noun: u8,
        /// Which register (0, 1, or 2) is currently being loaded.
        reg_index: u8,
        /// Total number of registers this verb loads (1 for V21/22/23, 3 for V25).
        total_regs: u8,
        /// Sign of the current accumulator (+1 or -1).
        sign: i8,
        /// Number of digits accumulated in the current component (0..=5).
        digits: u8,
        /// Absolute value of the current accumulator (0..=99_999).
        buf: u32,
        /// Register values committed so far, scaled into target units.
        committed: [f64; 3],
    },
    /// Operator error — awaiting RSET.
    OprErr,
}

impl Default for VnPhase {
    fn default() -> Self {
        VnPhase::Idle
    }
}

/// Crew interface Verb/Noun input state.
#[derive(Clone, Copy, Debug)]
pub struct VnState {
    pub phase: VnPhase,
    /// TIG stashed by V25 N33 while waiting for the delta-V components.
    /// Consumed by V25 N81 to invoke `p30_load_dv_lvlh`.
    pub pending_tig: Option<Met>,
}

impl VnState {
    /// `const` constructor usable inside `AgcState::new`.
    pub const fn new() -> Self {
        Self {
            phase: VnPhase::Idle,
            pending_tig: None,
        }
    }
}

impl Default for VnState {
    fn default() -> Self {
        Self::new()
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
                // Dispatch may transition phase itself (e.g. V25 → EnteringData).
                // Only return to Idle if still in EnteringNoun AND not in OprErr.
                if matches!(state.vn.phase, EnteringNoun { .. }) {
                    state.vn.phase = Idle;
                }
            }
            _ => raise_opr_err(state),
        },

        EnteringData {
            verb,
            noun,
            reg_index,
            total_regs,
            sign,
            digits,
            buf,
            committed,
        } => match key {
            Key::Digit(d) => {
                if digits >= 5 {
                    raise_opr_err(state);
                    return;
                }
                let new_buf = buf * 10 + d as u32;
                state.vn.phase = EnteringData {
                    verb,
                    noun,
                    reg_index,
                    total_regs,
                    sign,
                    digits: digits + 1,
                    buf: new_buf,
                    committed,
                };
            }
            Key::Plus => {
                if digits != 0 {
                    raise_opr_err(state);
                    return;
                }
                state.vn.phase = EnteringData {
                    verb,
                    noun,
                    reg_index,
                    total_regs,
                    sign: 1,
                    digits,
                    buf,
                    committed,
                };
            }
            Key::Minus => {
                if digits != 0 {
                    raise_opr_err(state);
                    return;
                }
                state.vn.phase = EnteringData {
                    verb,
                    noun,
                    reg_index,
                    total_regs,
                    sign: -1,
                    digits,
                    buf,
                    committed,
                };
            }
            Key::Entr => {
                // Commit the current accumulator into the target register.
                let scale = noun_scale(noun);
                let value = sign as f64 * buf as f64 * scale;
                let mut new_committed = committed;
                new_committed[reg_index as usize] = value;

                let next_reg = reg_index + 1;
                if next_reg < total_regs {
                    // More registers to load.
                    state.vn.phase = EnteringData {
                        verb,
                        noun,
                        reg_index: next_reg,
                        total_regs,
                        sign: 1,
                        digits: 0,
                        buf: 0,
                        committed: new_committed,
                    };
                } else {
                    // Load complete — commit and return to Idle.
                    noun_commit(state, verb, noun, new_committed);
                    if state.vn.phase != OprErr {
                        state.vn.phase = Idle;
                    }
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
        21 | 22 | 23 => start_load(state, verb, noun, 1, verb - 21),
        25 => start_load(state, verb, noun, 3, 0),
        34 => v34_terminate(state),
        35 => v35_lamp_test(state),
        37 => v37_program_select(state, noun),
        _ => raise_opr_err(state),
    }
}

/// Transition into `EnteringData` to start a V21/V22/V23/V25 load.
fn start_load(state: &mut crate::AgcState, verb: u8, noun: u8, total_regs: u8, reg_index: u8) {
    state.dsky.verb = verb;
    state.dsky.noun = noun;
    state.dsky.flashing = true; // crew input requested
    state.vn.phase = VnPhase::EnteringData {
        verb,
        noun,
        reg_index,
        total_regs,
        sign: 1,
        digits: 0,
        buf: 0,
        committed: [0.0; 3],
    };
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

// ── Noun scale table and commit handlers ─────────────────────────────────────

/// Program alarm raised when V25 N81 is entered without a prior TIG load.
const ALARM_DV_LOAD_WITHOUT_TIG: u16 = 240;

/// Convert the raw accumulated integer into the noun's target unit.
fn noun_scale(noun: u8) -> f64 {
    match noun {
        33 => 1.0, // TIG — centiseconds, integer
        34 => 1.0, // TFI — centiseconds, integer (placeholder)
        81 => 1.0, // LVLH ΔV — m/s, integer
        _ => 1.0,  // default pass-through
    }
}

/// Commit a completed data load. Called after the final ENTR of a
/// V21/V22/V23/V25 sequence, with the already-scaled register values.
fn noun_commit(state: &mut crate::AgcState, _verb: u8, noun: u8, values: [f64; 3]) {
    match noun {
        33 => noun_33_commit_tig(state, values[0]),
        81 => noun_81_commit_dv_lvlh(state, values),
        _ => {
            // Phase 2: unknown nouns are silently ignored. Future phases
            // will populate the DSKY R registers from `values`.
        }
    }
    // Clear the flashing indicator now the load is done (unless the
    // commit handler itself raised a flash request).
    if state.vn.phase != VnPhase::OprErr {
        state.dsky.flashing = false;
    }
}

/// N33 commit — stash TIG for a later delta-V load (typically V25 N81 after).
fn noun_33_commit_tig(state: &mut crate::AgcState, tig_cs: f64) {
    // Clamp to non-negative before converting to u32.
    let cs = if tig_cs < 0.0 { 0 } else { tig_cs as u32 };
    state.vn.pending_tig = Some(Met(cs));
}

/// N81 commit — consume the pending TIG and call `p30_load_dv_lvlh`.
fn noun_81_commit_dv_lvlh(state: &mut crate::AgcState, values: [f64; 3]) {
    let Some(tig) = state.vn.pending_tig.take() else {
        // No TIG staged — alarm and return without doing anything.
        state.alarm.code = ALARM_DV_LOAD_WITHOUT_TIG;
        state.alarm.lit = true;
        return;
    };
    let dv: Vec3 = [values[0], values[1], values[2]];
    crate::programs::p30::p30_load_dv_lvlh(state, tig, dv);
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

    // ── Phase 2: Data entry verbs ─────────────────────────────────────────────

    /// Helper: feed the digits of a non-negative integer as individual
    /// keypresses (most significant first).
    fn feed_number(state: &mut AgcState, mut n: u32) {
        if n == 0 {
            feed_key(state, Key::Digit(0));
            return;
        }
        // Build the digit list MSB-first.
        let mut digits: [u8; 6] = [0; 6];
        let mut count = 0;
        while n > 0 {
            digits[count] = (n % 10) as u8;
            n /= 10;
            count += 1;
        }
        for i in (0..count).rev() {
            feed_key(state, Key::Digit(digits[i]));
        }
    }

    /// TC-VND-1: V21 N33 E +12345 E stashes TIG = 12_345 cs.
    #[test]
    fn tc_vnd_1_v21_single_register_load() {
        let mut state = AgcState::new();

        feed(&mut state, &[Key::Verb, d(2), d(1), Key::Noun, d(3), d(3), Key::Entr]);
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 12_345);
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert_eq!(state.vn.pending_tig, Some(Met(12_345)));
        assert!(!state.dsky.opr_err);
    }

    /// TC-VND-2: V25 N33 E +50000 E commits pending_tig.
    #[test]
    fn tc_vnd_2_v25_n33_commits_tig() {
        let mut state = AgcState::new();

        feed(&mut state, &[Key::Verb, d(2), d(5), Key::Noun, d(3), d(3), Key::Entr]);
        // V25 N33 loads 3 registers, but noun 33 only reads values[0]
        // for the TIG. We must still feed all three components to finish.
        feed_number(&mut state, 50_000);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert_eq!(state.vn.pending_tig, Some(Met(50_000)));
    }

    /// TC-VND-3: V25 N33 followed by V25 N81 with 100 m/s prograde ΔV
    /// produces a pending_maneuver (end-to-end P30 flow, no init_p30).
    #[test]
    fn tc_vnd_3_full_p30_data_load() {
        let mut state = AgcState::new();
        // Seed a LEO state so apply_external_delta_v has something to work with.
        use crate::navigation::gravity::{MU_EARTH, R_EARTH};
        use crate::navigation::state_vector::{Frame, StateVector};
        let r = R_EARTH + 400_000.0;
        let v = libm::sqrt(MU_EARTH / r);
        state.csm_state = StateVector {
            position: [r, 0.0, 0.0],
            velocity: [0.0, v, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };
        state.time = Met(0);

        // V25 N33 E 50000 E 0 E 0 E — TIG = 500 s (5-digit limit)
        feed(&mut state, &[Key::Verb, d(2), d(5), Key::Noun, d(3), d(3), Key::Entr]);
        feed_number(&mut state, 50_000);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.pending_tig, Some(Met(50_000)));

        // V25 N81 E +100 E +0 E +0 E
        feed(&mut state, &[Key::Verb, d(2), d(5), Key::Noun, d(8), d(1), Key::Entr]);
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 100);
        feed_key(&mut state, Key::Entr);
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert!(state.vn.pending_tig.is_none(), "TIG must be consumed");
        assert!(
            state.pending_maneuver.is_some(),
            "P30 ΔV load must produce a pending_maneuver"
        );
        let m = state.pending_maneuver.unwrap();
        assert_eq!(m.tig, Met(50_000));

        // 100 m/s prograde → delta_v magnitude ≈ 100
        let dv = m.delta_v.0;
        let mag = libm::sqrt(dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]);
        assert!((mag - 100.0).abs() < 1e-6, "ΔV magnitude ≈ 100 m/s, got {mag}");
    }

    /// TC-VND-4: V25 N81 without prior TIG raises alarm 240.
    #[test]
    fn tc_vnd_4_n81_without_tig_alarms() {
        let mut state = AgcState::new();
        state.vn.pending_tig = None;

        feed(&mut state, &[Key::Verb, d(2), d(5), Key::Noun, d(8), d(1), Key::Entr]);
        feed_number(&mut state, 100);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.alarm.code, ALARM_DV_LOAD_WITHOUT_TIG);
        assert!(state.pending_maneuver.is_none());
    }

    /// TC-VND-5: minus sign before first digit yields a negative value.
    #[test]
    fn tc_vnd_5_minus_sign_handling() {
        let mut state = AgcState::new();
        state.vn.pending_tig = Some(Met(100_000));
        state.time = Met(0);
        use crate::navigation::gravity::{MU_EARTH, R_EARTH};
        use crate::navigation::state_vector::{Frame, StateVector};
        let r = R_EARTH + 400_000.0;
        let v = libm::sqrt(MU_EARTH / r);
        state.csm_state = StateVector {
            position: [r, 0.0, 0.0],
            velocity: [0.0, v, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        feed(&mut state, &[Key::Verb, d(2), d(5), Key::Noun, d(8), d(1), Key::Entr]);
        feed_key(&mut state, Key::Minus);
        feed_number(&mut state, 50);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);

        assert!(state.pending_maneuver.is_some());
        let m = state.pending_maneuver.unwrap();
        // First crew component is along-track (reordered into +Y inertial for
        // this geometry). Negative 50 m/s prograde → inertial dv[1] ≈ -50.
        assert!(m.delta_v.0[1] < -49.0 && m.delta_v.0[1] > -51.0);
    }

    /// TC-VND-6: sign after a digit raises OPR ERR.
    #[test]
    fn tc_vnd_6_sign_after_digit_opr_err() {
        let mut state = AgcState::new();

        feed(&mut state, &[Key::Verb, d(2), d(5), Key::Noun, d(3), d(3), Key::Entr]);
        feed_key(&mut state, Key::Digit(1));
        feed_key(&mut state, Key::Plus); // sign after digit

        assert_eq!(state.vn.phase, VnPhase::OprErr);
        assert!(state.dsky.opr_err);
    }

    /// TC-VND-7: six-digit overflow raises OPR ERR.
    #[test]
    fn tc_vnd_7_six_digit_overflow() {
        let mut state = AgcState::new();

        feed(&mut state, &[Key::Verb, d(2), d(5), Key::Noun, d(3), d(3), Key::Entr]);
        // 5 digits are ok; the 6th must error.
        for _ in 0..5 {
            feed_key(&mut state, Key::Digit(1));
        }
        feed_key(&mut state, Key::Digit(1));

        assert_eq!(state.vn.phase, VnPhase::OprErr);
    }

    /// TC-VND-8: CLR during data entry aborts the load.
    #[test]
    fn tc_vnd_8_clr_aborts_load() {
        let mut state = AgcState::new();

        feed(&mut state, &[Key::Verb, d(2), d(5), Key::Noun, d(3), d(3), Key::Entr]);
        feed_key(&mut state, Key::Digit(1));
        feed_key(&mut state, Key::Digit(2));
        feed_key(&mut state, Key::Clr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert_eq!(state.vn.pending_tig, None, "no commit on CLR");
    }

    /// TC-VND-9: V21 loads R1 only and commits immediately.
    #[test]
    fn tc_vnd_9_v21_immediate_commit() {
        let mut state = AgcState::new();

        feed(&mut state, &[Key::Verb, d(2), d(1), Key::Noun, d(3), d(3), Key::Entr]);
        feed_number(&mut state, 99_999);
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert_eq!(state.vn.pending_tig, Some(Met(99_999)));
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
