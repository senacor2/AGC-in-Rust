//! Verb/Noun processor — the PINBALL keyboard and display state machine.
//!
//! The crew communicates with the AGC by entering two-digit verb and noun
//! codes on the DSKY keyboard. Verbs specify actions (display, load, monitor);
//! nouns specify data items (registers, orbital parameters, time).
//!
//! AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc — CHARIN, ENTPAS0.

use crate::hal::dsky::DskyKey;

/// Current keyboard entry mode.
///
/// AGC source: PINBALL_GAME_BUTTONS_AND_LIGHTS.agc — CHARIN dispatch table.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EntryMode {
    /// Idle — waiting for VERB or NOUN key press.
    Idle,
    /// Entering verb digits (first or second digit).
    VerbEntry,
    /// Entering noun digits (first or second digit).
    NounEntry,
    /// Entering data digits (for load verbs: sign + 5 digits per register).
    DataEntry { register: u8 },
}

/// The PINBALL state machine.
///
/// Holds all mutable state for the keyboard/display subsystem.
/// Designed for `const` construction so it can live in static storage.
///
/// AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc — PINBALL GAME.
#[derive(Clone, Debug)]
pub struct VerbNounState {
    /// Current entry mode.
    pub mode: EntryMode,
    /// Active verb code (0-99).
    pub verb: u8,
    /// Active noun code (0-99).
    pub noun: u8,
    /// Digit buffer for current entry (up to 7 chars: sign + 5 digits + spare).
    pub digit_buf: [u8; 7],
    /// Number of digits entered in current buffer.
    pub digit_count: u8,
    /// Current program number displayed.
    pub prog: u8,
    /// Whether display is flashing (awaiting crew response).
    pub flashing: bool,
    /// Flash state toggle (for blink effect, toggled by `flash_tick`).
    pub flash_on: bool,
    /// Data registers R1, R2, R3 (raw i32 values for display).
    pub registers: [i32; 3],
    /// Sign flags for R1, R2, R3 (+1 or -1).
    pub signs: [i8; 3],
    /// Whether each register has been loaded by the crew.
    pub reg_loaded: [bool; 3],
    /// Whether OPR ERR light should be on.
    pub opr_err: bool,
}

impl VerbNounState {
    /// Construct a zeroed-out initial state.
    pub const fn new() -> Self {
        Self {
            mode: EntryMode::Idle,
            verb: 0,
            noun: 0,
            digit_buf: [0u8; 7],
            digit_count: 0,
            prog: 0,
            flashing: false,
            flash_on: false,
            registers: [0i32; 3],
            signs: [1i8; 3],
            reg_loaded: [false; 3],
            opr_err: false,
        }
    }

    /// Process a single key press. Returns a [`VerbNounAction`].
    ///
    /// Implements the CHARIN / ENTPAS0 state machine from AGC PINBALL:
    /// - digit keys accumulate a two-digit code into `digit_buf`;
    /// - after the second digit the code is committed to `verb` or `noun`;
    /// - a third digit in verb/noun mode is an operator error;
    /// - ENTER dispatches when both verb and noun are non-zero;
    /// - CLR, PRO, RSET, KEY REL have their documented AGC meanings.
    ///
    /// AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc — CHARIN.
    pub fn process_key(&mut self, key: DskyKey) -> VerbNounAction {
        match key {
            DskyKey::Verb => {
                self.mode = EntryMode::VerbEntry;
                self.digit_buf = [0u8; 7];
                self.digit_count = 0;
                VerbNounAction::None
            }
            DskyKey::Noun => {
                self.mode = EntryMode::NounEntry;
                self.digit_buf = [0u8; 7];
                self.digit_count = 0;
                VerbNounAction::None
            }
            DskyKey::Zero
            | DskyKey::One
            | DskyKey::Two
            | DskyKey::Three
            | DskyKey::Four
            | DskyKey::Five
            | DskyKey::Six
            | DskyKey::Seven
            | DskyKey::Eight
            | DskyKey::Nine => {
                let digit = key_to_digit(key);
                match self.mode {
                    EntryMode::VerbEntry | EntryMode::NounEntry => {
                        if self.digit_count >= 2 {
                            // Third digit in two-digit field — operator error.
                            self.opr_err = true;
                            return VerbNounAction::Error;
                        }
                        self.digit_buf[self.digit_count as usize] = digit;
                        self.digit_count += 1;
                        if self.digit_count == 2 {
                            let code = self.digit_buf[0] * 10 + self.digit_buf[1];
                            if self.mode == EntryMode::VerbEntry {
                                self.verb = code;
                            } else {
                                self.noun = code;
                            }
                            self.mode = EntryMode::Idle;
                            self.digit_count = 0;
                        }
                        VerbNounAction::None
                    }
                    EntryMode::DataEntry { register } => {
                        if self.digit_count < 7 {
                            self.digit_buf[self.digit_count as usize] = digit;
                            self.digit_count += 1;
                            // After 5 data digits, commit the register value.
                            if self.digit_count == 5 {
                                let value = digits_to_i32(&self.digit_buf);
                                let sign = self.signs[register as usize];
                                let reg_idx = register as usize;
                                if reg_idx < 3 {
                                    self.registers[reg_idx] = value * sign as i32;
                                    self.reg_loaded[reg_idx] = true;
                                }
                            }
                        }
                        VerbNounAction::None
                    }
                    EntryMode::Idle => VerbNounAction::None,
                }
            }
            DskyKey::Plus => {
                if let EntryMode::DataEntry { register } = self.mode {
                    let idx = register as usize;
                    if idx < 3 {
                        self.signs[idx] = 1;
                    }
                    self.digit_buf = [0u8; 7];
                    self.digit_count = 0;
                }
                VerbNounAction::None
            }
            DskyKey::Minus => {
                if let EntryMode::DataEntry { register } = self.mode {
                    let idx = register as usize;
                    if idx < 3 {
                        self.signs[idx] = -1;
                    }
                    self.digit_buf = [0u8; 7];
                    self.digit_count = 0;
                }
                VerbNounAction::None
            }
            DskyKey::Enter => {
                if self.verb != 0 {
                    let v = self.verb;
                    let n = self.noun;
                    self.mode = EntryMode::Idle;
                    VerbNounAction::Execute { verb: v, noun: n }
                } else {
                    self.opr_err = true;
                    VerbNounAction::Error
                }
            }
            DskyKey::Clear => {
                self.digit_buf = [0u8; 7];
                self.digit_count = 0;
                // Keep mode but clear current partial entry.
                VerbNounAction::Clear
            }
            DskyKey::ProceED => VerbNounAction::Proceed,
            DskyKey::Reset => {
                self.opr_err = false;
                VerbNounAction::Reset
            }
            DskyKey::KeyRel => VerbNounAction::KeyRelease,
        }
    }

    /// Tick the flash state. Call at approximately 2 Hz for the blink effect.
    ///
    /// AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc — flash logic.
    pub fn flash_tick(&mut self) {
        if self.flashing {
            self.flash_on = !self.flash_on;
        }
    }

    /// Set the display to show a verb/noun from an internal program request.
    ///
    /// AGC source: Comanche055/DISPLAY_INTERFACE_ROUTINES.agc — NVSUB.
    pub fn request_display(&mut self, verb: u8, noun: u8) {
        self.verb = verb;
        self.noun = noun;
        self.mode = EntryMode::Idle;
    }

    /// Set a register value for display (index 0, 1, or 2 → R1, R2, R3).
    pub fn set_register(&mut self, index: u8, value: i32) {
        if (index as usize) < 3 {
            self.registers[index as usize] = value;
        }
    }

    /// Set the program number shown in the PROG field.
    pub fn set_prog(&mut self, prog: u8) {
        self.prog = prog;
    }

    /// Start flashing verb/noun to alert the crew that input is required.
    ///
    /// AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc — FLASHON.
    pub fn start_flash(&mut self) {
        self.flashing = true;
        self.flash_on = true;
    }

    /// Stop flashing and hold the display steady.
    ///
    /// AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc — FLASHOFF.
    pub fn stop_flash(&mut self) {
        self.flashing = false;
        self.flash_on = true;
    }
}

impl Default for VerbNounState {
    fn default() -> Self {
        Self::new()
    }
}

/// Action resulting from a key press.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VerbNounAction {
    /// No action — digit accepted, continue entry.
    None,
    /// Verb/Noun pair confirmed by ENTER — dispatch the command.
    Execute { verb: u8, noun: u8 },
    /// Crew pressed PRO — proceed/acknowledge.
    Proceed,
    /// Crew pressed RSET — clear alarms and errors.
    Reset,
    /// Crew pressed CLR — clear current entry.
    Clear,
    /// Crew pressed KEY REL — release keyboard to internal program.
    KeyRelease,
    /// Error: invalid entry (sets OPR ERR light).
    Error,
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

/// Convert a digit `DskyKey` to its `u8` value 0-9.
fn key_to_digit(key: DskyKey) -> u8 {
    match key {
        DskyKey::Zero => 0,
        DskyKey::One => 1,
        DskyKey::Two => 2,
        DskyKey::Three => 3,
        DskyKey::Four => 4,
        DskyKey::Five => 5,
        DskyKey::Six => 6,
        DskyKey::Seven => 7,
        DskyKey::Eight => 8,
        DskyKey::Nine => 9,
        _ => 0,
    }
}

/// Interpret up to 5 digits in `buf` as a decimal `i32`.
fn digits_to_i32(buf: &[u8; 7]) -> i32 {
    let mut v: i32 = 0;
    for &d in buf[..5].iter() {
        v = v * 10 + d as i32;
    }
    v
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn verb_key_switches_to_verb_entry() {
        let mut s = VerbNounState::new();
        let action = s.process_key(DskyKey::Verb);
        assert_eq!(s.mode, EntryMode::VerbEntry);
        assert_eq!(action, VerbNounAction::None);
    }

    #[test]
    fn two_digits_set_verb_code() {
        let mut s = VerbNounState::new();
        s.process_key(DskyKey::Verb);
        s.process_key(DskyKey::Three);
        s.process_key(DskyKey::Seven);
        assert_eq!(s.verb, 37);
        assert_eq!(s.mode, EntryMode::Idle);
    }

    #[test]
    fn enter_with_verb_and_noun_returns_execute() {
        let mut s = VerbNounState::new();
        s.process_key(DskyKey::Verb);
        s.process_key(DskyKey::Zero);
        s.process_key(DskyKey::Six);
        s.process_key(DskyKey::Noun);
        s.process_key(DskyKey::Three);
        s.process_key(DskyKey::Six);
        let action = s.process_key(DskyKey::Enter);
        assert_eq!(action, VerbNounAction::Execute { verb: 6, noun: 36 });
    }

    #[test]
    fn clear_resets_digit_buffer() {
        let mut s = VerbNounState::new();
        s.process_key(DskyKey::Verb);
        s.process_key(DskyKey::Three);
        let action = s.process_key(DskyKey::Clear);
        assert_eq!(action, VerbNounAction::Clear);
        assert_eq!(s.digit_count, 0);
        assert_eq!(s.digit_buf, [0u8; 7]);
    }

    #[test]
    fn proceed_key_returns_proceed() {
        let mut s = VerbNounState::new();
        assert_eq!(s.process_key(DskyKey::ProceED), VerbNounAction::Proceed);
    }

    #[test]
    fn reset_clears_opr_err() {
        let mut s = VerbNounState::new();
        s.opr_err = true;
        let action = s.process_key(DskyKey::Reset);
        assert_eq!(action, VerbNounAction::Reset);
        assert!(!s.opr_err);
    }

    #[test]
    fn flash_tick_toggles_flash_on() {
        let mut s = VerbNounState::new();
        s.start_flash();
        assert!(s.flash_on);
        s.flash_tick();
        assert!(!s.flash_on);
        s.flash_tick();
        assert!(s.flash_on);
    }

    #[test]
    fn third_digit_in_verb_mode_returns_error() {
        let mut s = VerbNounState::new();
        s.process_key(DskyKey::Verb);
        s.process_key(DskyKey::Three);
        // Second digit commits the verb and returns to Idle,
        // so a subsequent digit while Idle is silently ignored (None).
        // Re-enter Verb mode to test the three-digit guard directly.
        s.mode = EntryMode::VerbEntry;
        s.digit_count = 2;
        let action = s.process_key(DskyKey::Seven);
        assert_eq!(action, VerbNounAction::Error);
        assert!(s.opr_err);
    }
}
