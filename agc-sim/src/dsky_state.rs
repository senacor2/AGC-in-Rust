//! DSKY display state model.
//!
//! Decodes raw 14-bit relay words (channel 010 format) into human-readable
//! display fields: PROG/VERB/NOUN digit pairs, three data registers, and the
//! indicator light bitmask.
//!
//! The relay word format follows the AGC channel 010 bit layout documented in
//! T4RUPT_PROGRAM.agc.

/// Sign character for a data register.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Sign {
    Plus,
    Minus,
    Blank,
}

impl Sign {
    pub fn as_char(self) -> char {
        match self {
            Sign::Plus => '+',
            Sign::Minus => '-',
            Sign::Blank => ' ',
        }
    }
}

/// One 5-digit + sign data register (R1, R2, R3).
#[derive(Clone, Copy, Debug)]
pub struct DataRegister {
    pub sign: Sign,
    /// Digits 1–5 (index 0 = leftmost). 0–9 or 0xFF = blank.
    pub digits: [u8; 5],
}

impl DataRegister {
    pub const BLANK: Self = Self {
        sign: Sign::Blank,
        digits: [0xFF; 5],
    };

    /// Format as a display string like `+00000` or `-12345`.
    pub fn to_display_string(self) -> String {
        let sign = self.sign.as_char();
        let digits: String = self
            .digits
            .iter()
            .map(|&d| if d > 9 { ' ' } else { (b'0' + d) as char })
            .collect();
        format!("{}{}", sign, digits)
    }
}

/// All indicator lights on the DSKY panel.
#[derive(Clone, Copy, Default, Debug)]
pub struct Lights {
    pub uplink_acty: bool,
    pub temp: bool,
    pub gimbal_lock: bool,
    pub prog_alarm: bool,
    pub key_rel: bool,
    pub opr_err: bool,
    pub comp_acty: bool,
    pub no_att: bool,
    pub stby: bool,
    pub restart: bool,
    pub tracker: bool,
    pub alt: bool,
    pub vel: bool,
}

/// Complete decoded DSKY display state.
#[derive(Clone, Debug)]
pub struct DskyDisplayState {
    /// Two-digit PROG display (major mode number).
    pub prog: [u8; 2],
    /// Two-digit VERB display.
    pub verb: [u8; 2],
    /// Two-digit NOUN display.
    pub noun: [u8; 2],
    /// Three data registers.
    pub r1: DataRegister,
    pub r2: DataRegister,
    pub r3: DataRegister,
    /// Indicator lights.
    pub lights: Lights,
}

impl Default for DskyDisplayState {
    fn default() -> Self {
        Self {
            prog: [0xFF, 0xFF],
            verb: [0xFF, 0xFF],
            noun: [0xFF, 0xFF],
            r1: DataRegister::BLANK,
            r2: DataRegister::BLANK,
            r3: DataRegister::BLANK,
            lights: Lights::default(),
        }
    }
}

impl DskyDisplayState {
    /// Format a two-digit field. 0xFF = blank.
    pub fn fmt_pair(digits: [u8; 2]) -> String {
        digits
            .iter()
            .map(|&d| if d > 9 { ' ' } else { (b'0' + d) as char })
            .collect()
    }
}
