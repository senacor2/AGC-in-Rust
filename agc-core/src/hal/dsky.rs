//! DSKY (Display/Keyboard) I/O interface.
//!
//! AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc (pages 307-315, 399+)
//!             Comanche055/T4RUPT_PROGRAM.agc (DSPOUT, CDRVE, RELTAB, pages 133-138)
//! Channels:   OUT0 (octal 10) — display relay output
//!             DSALMOUT (octal 11) — alarm lamp bits
//!             MNKEYIN (octal 15) — main keyboard input
//!             NAVKEYIN (octal 16) — nav keyboard input

/// 5-bit DSKY keyboard key code.
///
/// AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, page 313.
/// Input channel: MNKEYIN (octal 15) for main DSKY; NAVKEYIN (octal 16) for nav DSKY.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Key {
    /// Key code octal 20 = decimal 16.
    Zero = 0x10,
    One = 0x01,
    Two = 0x02,
    Three = 0x03,
    Four = 0x04,
    Five = 0x05,
    Six = 0x06,
    Seven = 0x07,
    /// Key code octal 10 = decimal 8.
    Eight = 0x08,
    /// Key code octal 11 = decimal 9.
    Nine = 0x09,
    /// Key code octal 21 = decimal 17.
    Verb = 0x11,
    /// ERROR LIGHT RESET. Key code octal 22 = decimal 18.
    Reset = 0x12,
    /// KEY RELEASE. Key code octal 31 = decimal 25.
    KeyRel = 0x19,
    /// Key code octal 32 = decimal 26.
    Plus = 0x1A,
    /// Key code octal 33 = decimal 27.
    Minus = 0x1B,
    /// Key code octal 34 = decimal 28.
    Enter = 0x1C,
    /// Key code octal 36 = decimal 30.
    Clear = 0x1E,
    /// Key code octal 37 = decimal 31.
    Noun = 0x1F,
}

/// A packed 15-bit word for channel OUT0 (channel 10).
///
/// Format (AGC bit numbering, 1-based from LSB):
/// - Bits 15-12: relay word selector (4 bits)
/// - Bit 11: special relay (sign, lamp override, etc.)
/// - Bits 10-6: 5-bit relay code for left character of selected pair
/// - Bits 5-1: 5-bit relay code for right character of selected pair
///
/// AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc page 313-314.
#[derive(Clone, Copy, Debug, Default)]
pub struct RelayWord(pub u16);

impl RelayWord {
    /// Construct from components.
    pub fn new(relay_selector: u8, special: bool, left_code: u8, right_code: u8) -> Self {
        let word = ((relay_selector as u16 & 0xF) << 11)
            | ((special as u16) << 10)
            | ((left_code as u16 & 0x1F) << 5)
            | (right_code as u16 & 0x1F);
        Self(word)
    }

    /// Return the 4-bit relay word selector (bits 14-11, 0-indexed).
    pub fn relay_selector(self) -> u8 {
        ((self.0 >> 11) & 0xF) as u8
    }

    /// Return the special relay bit (bit 10).
    pub fn special(self) -> bool {
        (self.0 >> 10) & 1 != 0
    }

    /// Return the 5-bit left character relay code (bits 9-5).
    pub fn left_code(self) -> u8 {
        ((self.0 >> 5) & 0x1F) as u8
    }

    /// Return the 5-bit right character relay code (bits 4-0).
    pub fn right_code(self) -> u8 {
        (self.0 & 0x1F) as u8
    }
}

/// A decoded display row (verb, noun, major mode, or R1/R2/R3).
///
/// Each digit is in `[0, 9]` or `0xFF` for blank.
///
/// AGC source: decoded from 5-bit relay codes in PINBALL_GAME_BUTTONS_AND_LIGHTS.agc.
#[derive(Clone, Copy, Debug, Default)]
pub struct DigitRow {
    /// Up to 5 digit positions; unused positions = 0xFF (blank).
    pub digits: [u8; 5],
    /// Positive sign indicator for R1/R2/R3 display rows.
    pub sign_plus: bool,
    /// Negative sign indicator for R1/R2/R3 display rows.
    pub sign_minus: bool,
}

/// DSKY I/O interface.
///
/// Isolates the flight software from the display relay hardware.
///
/// AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc,
///             Comanche055/T4RUPT_PROGRAM.agc (DSPOUT routine).
/// Channels:   OUT0 (octal 10), DSALMOUT (octal 11),
///             MNKEYIN (octal 15), NAVKEYIN (octal 16).
pub trait DskyIo {
    /// Read the next keyboard keypress, or `None` if no key is pending.
    ///
    /// Corresponds to reading MNKEYIN (channel 15) in KEYRUPT1.
    fn read_key(&mut self) -> Option<Key>;

    /// Read from the navigation DSKY keyboard (NAVKEYIN, channel 16).
    fn read_nav_key(&mut self) -> Option<Key>;

    /// Write a relay word to the display (channel OUT0, octal 10).
    ///
    /// Called by T4RUPT DSPOUT once per display scan cycle.
    /// The relay word encodes which character pair to update and the
    /// 5-bit segment codes for each character.
    ///
    /// AGC source: Comanche055/T4RUPT_PROGRAM.agc DSPOUT routine.
    fn write_relay(&mut self, word: RelayWord);

    /// Write the alarm/status lamp bits (channel DSALMOUT, octal 11).
    ///
    /// Bit 5 = KEY RELEASE lamp, Bit 6 = VERB/NOUN FLASH,
    /// Bit 7 = OPERATOR ERROR, Bit 4 = TEMP lamp.
    ///
    /// AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc PROGLARM.
    fn write_lamp_word(&mut self, bits: u16);

    /// Set the PROG (major mode) digits on the DSKY panel.
    ///
    /// Higher-level convenience; implementation writes to DSPTAB[10].
    fn write_prog(&mut self, prog: u8);

    /// Set the VERB digit field.
    fn write_verb(&mut self, verb: u8);

    /// Set the NOUN digit field.
    fn write_noun(&mut self, noun: u8);

    /// Set a register row (R1, R2, or R3).
    ///
    /// `row`: 0 = R1, 1 = R2, 2 = R3.
    fn write_register(&mut self, row: usize, value: &DigitRow);

    /// True if the PROCEED button is currently pressed.
    ///
    /// Corresponds to channel 32 bit 14 (PROCEEDE routine, T4RUPT_PROGRAM.agc).
    fn proceed_pressed(&self) -> bool;

    /// Set the PROG alarm light on or off.
    ///
    /// Corresponds to DSPTAB+11 bit 9 (PROGLARM in ALARM_AND_ABORT.agc, p. 1494).
    /// `OCT40400` = bits 8 and 9 set.
    ///
    /// AGC source: Comanche055/ALARM_AND_ABORT.agc PROGLARM.
    fn set_prog_light(&mut self, on: bool);

    /// Blank all DSKY display fields (PROG, VERB, NOUN).
    ///
    /// Corresponds to the STARTSB2 loop that zeros DSPTAB[0..10]:
    /// relay code 00000 = BLANK (distinct from relay code 10101 = digit "0").
    ///
    /// The default implementation writes relay code 0 to each field via
    /// `write_prog(0)` / `write_verb(0)` / `write_noun(0)`, which is correct
    /// for a hardware DSKY relay driver (relay code 0 drives no segments).
    /// Simulator implementations should override this to set display fields to
    /// `None` so that `prog.is_none()` correctly reports blank rather than "00".
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc STARTSB2 (line 515),
    ///             PINBALL_GAME_BUTTONS_AND_LIGHTS.agc relay table (BLANK = 00000).
    fn blank_display(&mut self) {
        self.write_prog(0);
        self.write_verb(0);
        self.write_noun(0);
    }

    /// Activate all display relays simultaneously (lamp test / VBTSTLTS).
    ///
    /// Lights every segment and indicator lamp on the DSKY panel.
    /// The default implementation sets the OPERATOR ERROR and KEY REL lamps
    /// via `write_lamp_word` with all defined bits set, and writes max relay
    /// codes to all digit positions to simulate full relay activation.
    ///
    /// AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc VBTSTLTS,
    /// page 326 (VERBTAB entry for V35).
    fn lamp_test(&mut self) {
        // Write all lamp bits: TEMP(bit4), KEY REL(bit5), OPER ERR(bit7),
        // COMP ACTY(bit12), UPLINK ACTY(bit15) — plus all available status bits.
        // AGC source: VBTSTLTS drives all output channels high.
        self.write_lamp_word(0b1000_1000_0111_1000);
        // Show "88" in PROG/VERB/NOUN to activate all segments (full test pattern).
        self.write_prog(88);
        self.write_verb(88);
        self.write_noun(88);
    }
}
