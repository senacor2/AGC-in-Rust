//! Verb/Noun keyboard input state machine (PINBALL GAME).
//!
//! Processes 5-bit DSKY key codes and accumulates verb/noun/data register input.
//!
//! AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc
//! Routines:   CHARIN, CHARIN2, VERB, NOUN, ENTER, ENTPAS0, ENTPASHI,
//!             CLEAR, VBRELDSP, CHARALRM, FALTON, NUM, 89TEST, GETINREL
//! Pages:      313–329 (BANK 40, SETLOC PINBALL1)

/// Current phase of the keyboard input state machine.
///
/// AGC source: encodes the combined state of DSPCOUNT, DECBRNCH, and REQRET.
/// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, pages 315-325.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum InputMode {
    /// No active entry; DSPCOUNT is negative (blocked).
    Idle,
    /// VERB key was pressed; entering first or second verb digit.
    EnteringVerb,
    /// NOUN key was pressed; entering first or second noun digit.
    EnteringNoun,
    /// A load verb requested data; entering digits for register 1, 2, or 3.
    /// The u8 is the 1-based register index (1 = R1/XREG, 2 = R2/YREG, 3 = R3/ZREG).
    EnteringData(u8),
}

/// Sign mode for the current data register entry.
///
/// AGC source: DECBRNCH erasable register (low 2 bits).
/// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc pages 315-318.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SignMode {
    /// Octal entry (DECBRNCH = +0).
    Octal,
    /// Plus decimal entry (DECBRNCH bit1 set by POSGN).
    PlusDecimal,
    /// Minus decimal entry (DECBRNCH bit2 set by NEGSGN).
    MinusDecimal,
}

/// Result returned by `char_in` after processing one key code.
///
/// AGC source: mirrors the jump targets after CHARIN dispatch.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CharResult {
    /// Key was accepted; state has advanced but no action is needed yet.
    Accepted,
    /// Key was illegal for the current state; OPERATOR ERROR light must be set.
    Rejected,
    /// ENTER was pressed in pass-0 and a complete verb/noun pair is ready.
    /// The caller should invoke `entr_press`.
    Complete(u8 /* verb */, u8 /* noun */),
}

/// All keyboard and display state for PINBALL.
///
/// AGC equivalents: VERBREG, NOUNREG, XREG/YREG/ZREG, XREGLP/YREGLP/ZREGLP,
/// DSPCOUNT, DECBRNCH, REQRET, CLPASS, DSPLOCK, CADRSTOR, MONSAVE.
/// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc pages 309-312 (erasable assignments).
pub struct VnState {
    /// Two-digit verb buffer (0–99). None = not yet entered.
    pub verb_buf: Option<u8>,
    /// Two-digit noun buffer (0–99). None = not yet entered.
    pub noun_buf: Option<u8>,
    /// Data buffers for R1, R2, R3 as scaled fixed-point.
    /// Matches XREG (index 0), YREG (index 1), ZREG (index 2).
    /// None = register not yet loaded.
    pub data_buf: [Option<i32>; 3],
    /// Current input mode; encodes the DSPCOUNT / REQRET state.
    pub mode: InputMode,
    /// Sign mode for the active data entry (mirrors DECBRNCH low 2 bits).
    pub sign_mode: SignMode,
    /// True when the VERB/NOUN lights should flash (set by load verbs and FLASHON).
    pub flash: bool,
    /// Digit count within the current field (0–5 for data, 0–2 for verb/noun).
    /// Mirrors DSPCOUNT decremented form.
    pub digit_count: u8,
    /// Number of successive CLR presses on the current entry (mirrors CLPASS).
    pub clpass: u8,
    /// Accumulator for the current digit field being entered (raw integer).
    digit_acc: i32,
}

impl VnState {
    /// Construct the fresh-start initial state (all registers zeroed/blank).
    ///
    /// AGC source: FRESH_START_AND_RESTART.agc, STARTSUB — initialises VERBREG,
    /// NOUNREG, DSPCOUNT, DECBRNCH, REQRET, CLPASS, DSPLOCK, CADRSTOR to zero.
    pub const fn new() -> Self {
        Self {
            verb_buf: None,
            noun_buf: None,
            data_buf: [None; 3],
            mode: InputMode::Idle,
            sign_mode: SignMode::Octal,
            flash: false,
            digit_count: 0,
            clpass: 0,
            digit_acc: 0,
        }
    }
}

impl Default for VnState {
    fn default() -> Self {
        Self::new()
    }
}

impl VnState {
    /// Process one 5-bit key code from the DSKY keyboard.
    ///
    /// Implements `CHARIN` (page 315) through the full dispatch table.
    /// Key codes: digits 1–9 = codes 1–9, digit 0 = code 16 (decimal 16),
    /// VERB = 17, NOUN = 31, ENTER = 28, CLR = 30, KEY REL = 25,
    /// + = 26, - = 27. All others return `CharResult::Rejected`.
    ///
    /// AGC source: CHARIN, CHARIN2, NUM, 89TEST, VERB, NOUN, POSGN, NEGSGN.
    /// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, pages 315-321.
    pub fn char_in(&mut self, key: u8) -> CharResult {
        match key {
            // ── VERB key (code 17 = 0x11) ─────────────────────────────────────
            // AGC source: VERB routine, page 319.
            17 => {
                self.mode = InputMode::EnteringVerb;
                self.verb_buf = None;
                self.digit_acc = 0;
                self.digit_count = 0;
                self.sign_mode = SignMode::PlusDecimal; // verb always decimal
                CharResult::Accepted
            }

            // ── NOUN key (code 31 = 0x1F) ─────────────────────────────────────
            // AGC source: NOUN routine, page 319.
            31 => {
                self.mode = InputMode::EnteringNoun;
                self.noun_buf = None;
                self.digit_acc = 0;
                self.digit_count = 0;
                self.sign_mode = SignMode::PlusDecimal; // noun always decimal
                CharResult::Accepted
            }

            // ── ENTER key (code 28 = 0x1C) ────────────────────────────────────
            // AGC source: ENTER, ENTPAS0, pages 323-324.
            28 => self.handle_enter(),

            // ── CLR key (code 30 = 0x1E) ──────────────────────────────────────
            // AGC source: CLEAR routine, page 321.
            30 => {
                self.clr_press();
                CharResult::Accepted
            }

            // ── KEY REL (code 25 = 0x19) ──────────────────────────────────────
            // AGC source: VBRELDSP, page 368.
            25 => {
                self.key_rel_press();
                CharResult::Accepted
            }

            // ── Plus sign (code 26 = 0x1A) ────────────────────────────────────
            // AGC source: POSGN, page 318.
            26 => self.handle_sign(true),

            // ── Minus sign (code 27 = 0x1B) ───────────────────────────────────
            // AGC source: NEGSGN, page 318.
            27 => self.handle_sign(false),

            // ── Digit keys ────────────────────────────────────────────────────
            // Codes 1–9 = digits 1–9; code 16 = digit 0.
            // AGC source: NUM, 89TEST, DECTOBIN, pages 316-318.
            1..=9 | 16 => {
                let digit = if key == 16 { 0u8 } else { key };
                self.handle_digit(digit)
            }

            // ── Key code 0 and everything else → CHARALRM ─────────────────────
            // AGC source: CHARALRM / FALTON — illegal key → OPERATOR ERROR.
            // Per spec decision: key code 0 always Rejected.
            _ => CharResult::Rejected,
        }
    }

    /// Handle the ENTER key (pass-0 path, ENTPAS0).
    ///
    /// AGC source: ENTPAS0, page 324.
    fn handle_enter(&mut self) -> CharResult {
        match self.mode {
            InputMode::EnteringVerb => {
                // Finalize verb from accumulator if partial entry.
                if self.digit_count > 0 {
                    self.verb_buf = Some((self.digit_acc as u8).min(99));
                }
                self.mode = InputMode::Idle;
                CharResult::Accepted
            }
            InputMode::EnteringNoun => {
                // Finalize noun from accumulator if partial entry.
                if self.digit_count > 0 {
                    self.noun_buf = Some((self.digit_acc as u8).min(99));
                }
                self.mode = InputMode::Idle;
                CharResult::Accepted
            }
            InputMode::Idle => {
                // ENTPAS0 path: execute current verb/noun.
                let v = match self.verb_buf {
                    Some(v) => v,
                    None => return CharResult::Rejected,
                };
                let n = self.noun_buf.unwrap_or(0);
                CharResult::Complete(v, n)
            }
            InputMode::EnteringData(reg) => {
                // ENTPASHI path: finalize data entry.
                // AGC requires exactly 5 digits; Rust is lenient (any digits accepted).
                let idx = (reg.saturating_sub(1)) as usize;
                if idx < 3 && self.digit_count > 0 {
                    let val = match self.sign_mode {
                        SignMode::MinusDecimal => -self.digit_acc,
                        _ => self.digit_acc,
                    };
                    self.data_buf[idx] = Some(val);
                }
                self.digit_acc = 0;
                self.digit_count = 0;
                self.mode = InputMode::Idle;
                CharResult::Accepted
            }
        }
    }

    /// Handle a sign key (+ or −).
    ///
    /// AGC source: POSGN / NEGSGN, page 318.
    fn handle_sign(&mut self, plus: bool) -> CharResult {
        match self.mode {
            InputMode::EnteringData(_) => {
                // Sign is only meaningful at the start of a data field.
                if self.digit_count == 0 {
                    self.sign_mode = if plus {
                        SignMode::PlusDecimal
                    } else {
                        SignMode::MinusDecimal
                    };
                    CharResult::Accepted
                } else {
                    // Sign after digits → rejected.
                    CharResult::Rejected
                }
            }
            // In verb/noun mode, signs are rejected.
            _ => CharResult::Rejected,
        }
    }

    /// Handle a digit key (0–9).
    ///
    /// AGC source: NUM, 89TEST, DECTOBIN, pages 316-318.
    fn handle_digit(&mut self, digit: u8) -> CharResult {
        match self.mode {
            InputMode::EnteringVerb => {
                // Verb is always 2-digit decimal.
                if self.digit_count >= 2 {
                    return CharResult::Rejected;
                }
                self.digit_acc = self.digit_acc * 10 + digit as i32;
                self.digit_count += 1;
                if self.digit_count == 2 {
                    self.verb_buf = Some(self.digit_acc.min(99) as u8);
                    self.digit_acc = 0;
                    self.digit_count = 0;
                    self.mode = InputMode::Idle;
                }
                CharResult::Accepted
            }

            InputMode::EnteringNoun => {
                // Noun is always 2-digit decimal.
                if self.digit_count >= 2 {
                    return CharResult::Rejected;
                }
                self.digit_acc = self.digit_acc * 10 + digit as i32;
                self.digit_count += 1;
                if self.digit_count == 2 {
                    self.noun_buf = Some(self.digit_acc.min(99) as u8);
                    self.digit_acc = 0;
                    self.digit_count = 0;
                    self.mode = InputMode::Idle;
                }
                CharResult::Accepted
            }

            InputMode::EnteringData(_) => {
                // 5-digit maximum per CRITCON.
                // AGC source: CRITCON check blocks entry beyond 5 digits.
                if self.digit_count >= 5 {
                    return CharResult::Rejected;
                }
                // 89TEST: digits 8 and 9 rejected in octal mode.
                // AGC source: 89TEST, page 316.
                if self.sign_mode == SignMode::Octal && digit >= 8 {
                    return CharResult::Rejected;
                }
                if self.sign_mode == SignMode::Octal {
                    // Octal: accumulate with ×8.
                    self.digit_acc = self.digit_acc * 8 + digit as i32;
                } else {
                    // Decimal: accumulate with ×10 (DECTOBIN).
                    self.digit_acc = self.digit_acc * 10 + digit as i32;
                }
                self.digit_count += 1;
                CharResult::Accepted
            }

            InputMode::Idle => {
                // Digits in Idle mode are silently rejected (no active entry).
                CharResult::Rejected
            }
        }
    }

    /// Handle the CLR key (CLEAR routine, page 321).
    ///
    /// Successive calls blank R3, R2, R1. Illegal when mode is Idle,
    /// EnteringVerb, or EnteringNoun (those call CHARALRM in the AGC).
    ///
    /// AGC source: CLEAR, CLR5, 5BLANK, LEGALTST.
    /// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, pages 321-322.
    pub fn clr_press(&mut self) {
        // In Idle/EnteringVerb/EnteringNoun, CLR in the AGC jumps to CHARALRM.
        // In Rust we treat it as a no-op for those modes.
        if let InputMode::EnteringData(reg) = self.mode {
            let idx = (reg.saturating_sub(1)) as usize;
            if idx < 3 {
                self.data_buf[idx] = None;
            }
            self.digit_acc = 0;
            self.digit_count = 0;
            self.clpass = self.clpass.saturating_add(1);
        }
    }

    /// Handle the KEY REL key (VBRELDSP routine, page 368).
    ///
    /// Releases the display lock, turns off the KEY RELEASE light, and
    /// re-enables monitor operation if one was suspended.
    ///
    /// AGC source: VBRELDSP, RELDSP, RELDSP1.
    /// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, pages 368.
    pub fn key_rel_press(&mut self) {
        // Release flash lock and return to idle.
        self.flash = false;
        if self.mode != InputMode::Idle {
            self.mode = InputMode::Idle;
        }
    }

    /// Enter data-entry mode for the specified register (1-based: 1=R1, 2=R2, 3=R3).
    ///
    /// Called by load verbs (V21/V24/V25) to request crew input.
    /// Sets flash = true (VERB/NOUN display flashes to invite input).
    ///
    /// AGC source: REQDATX, PUTCOM, ABTGO — PINBALL_GAME_BUTTONS_AND_LIGHTS.agc,
    /// pages 343-348.
    pub fn request_data_entry(&mut self, reg: u8) {
        self.mode = InputMode::EnteringData(reg.clamp(1, 3));
        self.digit_acc = 0;
        self.digit_count = 0;
        self.sign_mode = SignMode::PlusDecimal;
        self.flash = true;
    }

    /// Clear flash and return to Idle (called by V34 terminate and V35 lamp test).
    ///
    /// AGC source: FLASHOFF, KILMONON — PINBALL_GAME_BUTTONS_AND_LIGHTS.agc.
    pub fn terminate_entry(&mut self) {
        self.flash = false;
        self.mode = InputMode::Idle;
        self.clpass = 0;
    }
}

// ── tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// TC-VN-1: Verb entry — two digits produce verb_buf == Some(34).
    ///
    /// AGC source: VERB routine (page 319), NUM routine (page 316).
    #[test]
    fn verb_entry_two_digits() {
        let mut vn = VnState::new();
        assert_eq!(vn.char_in(17), CharResult::Accepted); // VERB key
        assert_eq!(vn.mode, InputMode::EnteringVerb);
        assert_eq!(vn.char_in(3), CharResult::Accepted); // digit '3'
        assert_eq!(vn.char_in(4), CharResult::Accepted); // digit '4'
        assert_eq!(vn.verb_buf, Some(34));
        assert_eq!(vn.mode, InputMode::Idle);
    }

    /// TC-VN-2: Noun entry — two digits produce noun_buf == Some(36).
    ///
    /// AGC source: NOUN routine (page 319).
    #[test]
    fn noun_entry_two_digits() {
        let mut vn = VnState::new();
        assert_eq!(vn.char_in(31), CharResult::Accepted); // NOUN key
        assert_eq!(vn.char_in(3), CharResult::Accepted); // digit '3'
        assert_eq!(vn.char_in(6), CharResult::Accepted); // digit '6'
        assert_eq!(vn.noun_buf, Some(36));
        assert_eq!(vn.mode, InputMode::Idle);
    }

    /// TC-VN-3: Data entry with plus sign accumulates 5 decimal digits.
    ///
    /// AGC source: POSGN, DECTOBIN (page 317), CRITCON check.
    #[test]
    fn data_entry_plus_decimal() {
        let mut vn = VnState::new();
        vn.request_data_entry(1); // mode = EnteringData(1)
        assert_eq!(vn.char_in(26), CharResult::Accepted); // '+' sign
        assert_eq!(vn.sign_mode, SignMode::PlusDecimal);
        // digits 1,2,3,4,5
        for d in 1u8..=5 {
            assert_eq!(vn.char_in(d), CharResult::Accepted);
        }
        assert_eq!(vn.digit_count, 5);
        // 6th digit → rejected (CRITCON)
        assert_eq!(vn.char_in(1), CharResult::Rejected);
        // Finalize via ENTER.
        assert_eq!(vn.char_in(28), CharResult::Accepted);
        assert_eq!(vn.data_buf[0], Some(12345));
    }

    /// TC-VN-4: Bad input (key code 0) → Rejected.
    ///
    /// AGC source: CHARALRM / FALTON — illegal key → operator error light.
    #[test]
    fn bad_key_code_zero_rejected() {
        let mut vn = VnState::new();
        assert_eq!(vn.char_in(0), CharResult::Rejected);
        // State must be unchanged.
        assert_eq!(vn.mode, InputMode::Idle);
        assert!(vn.verb_buf.is_none());
    }

    /// TC-VN-5: CLR press clears data register.
    ///
    /// AGC source: CLEAR, CLR5, 5BLANK — zeroes XREG/YREG/ZREG display.
    #[test]
    fn clr_press_clears_data_reg() {
        let mut vn = VnState::new();
        vn.data_buf = [Some(42), Some(7), None];
        vn.mode = InputMode::EnteringData(2); // entering R2
        vn.digit_acc = 7;
        vn.digit_count = 1;
        vn.clr_press();
        assert_eq!(vn.data_buf[1], None); // R2/YREG cleared
        assert!(vn.clpass > 0);
        assert_eq!(vn.digit_count, 0);
    }

    /// TC-VN-6: KEY REL press returns to Idle and clears flash.
    ///
    /// AGC source: VBRELDSP, RELDSP — releases display lock.
    #[test]
    fn key_rel_clears_flash() {
        let mut vn = VnState::new();
        vn.flash = true;
        vn.mode = InputMode::EnteringData(1);
        vn.key_rel_press();
        assert!(!vn.flash);
        assert_eq!(vn.mode, InputMode::Idle);
    }

    /// TC-VN-7: ENTER in Idle with verb/noun ready → Complete.
    ///
    /// AGC source: ENTPAS0 path, page 324.
    #[test]
    fn enter_in_idle_produces_complete() {
        let mut vn = VnState::new();
        vn.verb_buf = Some(6);
        vn.noun_buf = Some(36);
        assert_eq!(vn.char_in(28), CharResult::Complete(6, 36));
    }

    /// TC-VN-8: Octal mode rejects digits 8 and 9 (89TEST).
    ///
    /// AGC source: 89TEST, page 316.
    #[test]
    fn octal_mode_rejects_8_and_9() {
        let mut vn = VnState::new();
        vn.mode = InputMode::EnteringData(1);
        vn.sign_mode = SignMode::Octal;
        assert_eq!(vn.char_in(8), CharResult::Rejected);
        assert_eq!(vn.char_in(9), CharResult::Rejected);
        assert_eq!(vn.char_in(7), CharResult::Accepted); // 7 is ok in octal
    }

    /// TC-VN-9: Minus sign followed by data entry stores negative value.
    ///
    /// AGC source: NEGSGN sets DECBRNCH bit2.
    /// Key code for digit 0 is 16 (not 0). Key code 0 is always rejected.
    #[test]
    fn minus_sign_data_entry() {
        let mut vn = VnState::new();
        vn.request_data_entry(1);
        assert_eq!(vn.char_in(27), CharResult::Accepted); // minus sign
        assert_eq!(vn.sign_mode, SignMode::MinusDecimal);
        // digits 1, 0, 0 — use code 16 for digit 0
        vn.char_in(1); // digit '1'
        vn.char_in(16); // digit '0' (code 16 = decimal 0)
        vn.char_in(16); // digit '0'
        vn.char_in(28); // ENTER
        assert_eq!(vn.data_buf[0], Some(-100));
    }
}
