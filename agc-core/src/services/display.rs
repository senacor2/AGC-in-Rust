//! DSKY display formatting — converts numeric values to relay word format.
//!
//! AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc — DSPDECVN, DSPOCTWD.

// ---------------------------------------------------------------------------
// Numeric formatters
// ---------------------------------------------------------------------------

/// Format an `i32` value as a 5-digit decimal display with sign.
///
/// Returns `(sign_char, [d1, d2, d3, d4, d5])` where `sign_char` is `'+'` or
/// `'-'` and each digit is 0-9.  Values larger than 99 999 are saturated to
/// 99 999 to fit the 5-digit field.
///
/// AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc — DSPDECVN.
pub fn format_decimal(value: i32) -> (char, [u8; 5]) {
    let sign = if value < 0 { '-' } else { '+' };
    let mut mag = value.unsigned_abs();
    if mag > 99_999 {
        mag = 99_999;
    }
    let d5 = (mag % 10) as u8;
    let d4 = (mag / 10 % 10) as u8;
    let d3 = (mag / 100 % 10) as u8;
    let d2 = (mag / 1_000 % 10) as u8;
    let d1 = (mag / 10_000 % 10) as u8;
    (sign, [d1, d2, d3, d4, d5])
}

/// Format a `u16` value as a 5-digit octal display (no sign).
///
/// The maximum representable value is 0o77777 (32 767); values above that are
/// masked to 15 bits.
///
/// AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc — DSPOCTWD.
pub fn format_octal(value: u16) -> [u8; 5] {
    let v = value & 0o77777;
    let d5 = (v & 0o7) as u8;
    let d4 = ((v >> 3) & 0o7) as u8;
    let d3 = ((v >> 6) & 0o7) as u8;
    let d2 = ((v >> 9) & 0o7) as u8;
    let d1 = ((v >> 12) & 0o7) as u8;
    [d1, d2, d3, d4, d5]
}

/// Format a time value (centiseconds) as MM:SS.cs for display.
///
/// Returns `(minutes, seconds, centiseconds)`. Minutes saturate at 99.
pub fn format_time(centiseconds: u32) -> (u8, u8, u8) {
    let total_seconds = centiseconds / 100;
    let cs = (centiseconds % 100) as u8;
    let seconds = (total_seconds % 60) as u8;
    let minutes = (total_seconds / 60).min(99) as u8;
    (minutes, seconds, cs)
}

/// Format an angle in degrees × 100 (e.g., 12345 = 123.45°).
///
/// Delegates to `format_decimal`; the caller is responsible for scale.
pub fn format_angle_deg100(value: i32) -> (char, [u8; 5]) {
    format_decimal(value)
}

// ---------------------------------------------------------------------------
// Relay word encoding
// ---------------------------------------------------------------------------

/// Encode two BCD digits into the relay word format for DSKY channel 010.
///
/// The 14-bit relay word layout from T4RUPT_PROGRAM.agc:
/// - bits 13-11: field selector (0-7 for the 11 digit fields on the DSKY)
/// - bits 10-7:  first digit (0-9, BCD)
/// - bits  6-3:  second digit (0-9, BCD)
/// - bits  2-0:  unused / zero
///
/// `field` is the 3-bit field address (0–7); `d1` and `d2` are BCD digits
/// (0–9 each).
///
/// AGC source: Comanche055/T4RUPT_PROGRAM.agc — relay word bit layout.
pub fn encode_relay_word(field: u8, d1: u8, d2: u8) -> u16 {
    let f = (field & 0x07) as u16;
    let a = (d1 & 0x0F) as u16;
    let b = (d2 & 0x0F) as u16;
    (f << 11) | (a << 7) | (b << 3)
}

// ---------------------------------------------------------------------------
// Noun table
// ---------------------------------------------------------------------------

/// Noun display table entry: describes what to show for a given noun code.
///
/// AGC source: Comanche055/PINBALL_NOUN_TABLES.agc — NNADTAB, NNTYPTAB.
#[derive(Clone, Copy, Debug)]
pub struct NounDef {
    /// Number of registers used (1, 2, or 3).
    pub num_regs: u8,
    /// Scale type for each register (R1, R2, R3).
    pub scale: [ScaleType; 3],
    /// Human-readable label (for simulator display).
    pub label: &'static str,
}

/// Display scale / format type for a register in a noun definition.
///
/// AGC source: Comanche055/PINBALL_NOUN_TABLES.agc — NNTYPTAB.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScaleType {
    /// Signed 5-digit decimal.
    Decimal,
    /// 5-digit octal (unsigned).
    Octal,
    /// Time in MM:SS.cs format.
    TimeMmSs,
    /// Angle in degrees × 100 (signed 5-digit decimal).
    AngleDeg100,
    /// Raw integer displayed as-is.
    Raw,
}

/// Common noun definitions.
///
/// AGC source: Comanche055/PINBALL_NOUN_TABLES.agc — NNADTAB / NNTYPTAB.
pub const NOUN_TABLE: &[(u8, NounDef)] = &[
    (
        6,
        NounDef {
            num_regs: 3,
            scale: [ScaleType::Octal, ScaleType::Octal, ScaleType::Octal],
            label: "OPTION CODE",
        },
    ),
    (
        9,
        NounDef {
            num_regs: 3,
            scale: [ScaleType::Decimal, ScaleType::Decimal, ScaleType::Decimal],
            label: "ALARM CODES",
        },
    ),
    (
        14,
        NounDef {
            num_regs: 3,
            scale: [ScaleType::Decimal, ScaleType::Decimal, ScaleType::Decimal],
            label: "DESIRED REFSMMAT",
        },
    ),
    (
        17,
        NounDef {
            num_regs: 3,
            scale: [ScaleType::Decimal, ScaleType::Decimal, ScaleType::Decimal],
            label: "ASTRONAUT TOTAL ATT",
        },
    ),
    (
        33,
        NounDef {
            num_regs: 1,
            scale: [ScaleType::TimeMmSs, ScaleType::Raw, ScaleType::Raw],
            label: "TIME OF IGN",
        },
    ),
    (
        36,
        NounDef {
            num_regs: 3,
            scale: [ScaleType::TimeMmSs, ScaleType::TimeMmSs, ScaleType::TimeMmSs],
            label: "TIME OF AGC CLOCK",
        },
    ),
    (
        44,
        NounDef {
            num_regs: 2,
            scale: [ScaleType::Decimal, ScaleType::Decimal, ScaleType::Raw],
            label: "APO/PERI ALT",
        },
    ),
    (
        62,
        NounDef {
            num_regs: 3,
            scale: [ScaleType::Decimal, ScaleType::Decimal, ScaleType::Decimal],
            label: "INERTIAL VEL/ALT RATE/ALT",
        },
    ),
    (
        76,
        NounDef {
            num_regs: 2,
            scale: [ScaleType::Decimal, ScaleType::Decimal, ScaleType::Raw],
            label: "DESIRED LAT/LONG",
        },
    ),
    (
        85,
        NounDef {
            num_regs: 3,
            scale: [ScaleType::Decimal, ScaleType::Decimal, ScaleType::Decimal],
            label: "VG BODY X/Y/Z",
        },
    ),
    (
        99,
        NounDef {
            num_regs: 3,
            scale: [ScaleType::Decimal, ScaleType::Decimal, ScaleType::Decimal],
            label: "SYSTEM TEST RESULTS",
        },
    ),
];

/// Look up a noun definition by its two-digit code.
///
/// Returns `None` for codes not present in `NOUN_TABLE`.
pub fn lookup_noun(code: u8) -> Option<&'static NounDef> {
    for (noun_code, def) in NOUN_TABLE {
        if *noun_code == code {
            return Some(def);
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn format_decimal_positive() {
        let (sign, digits) = format_decimal(12345);
        assert_eq!(sign, '+');
        assert_eq!(digits, [1, 2, 3, 4, 5]);
    }

    #[test]
    fn format_decimal_negative() {
        let (sign, digits) = format_decimal(-42);
        assert_eq!(sign, '-');
        assert_eq!(digits, [0, 0, 0, 4, 2]);
    }

    #[test]
    fn format_decimal_zero() {
        let (sign, digits) = format_decimal(0);
        assert_eq!(sign, '+');
        assert_eq!(digits, [0, 0, 0, 0, 0]);
    }

    #[test]
    fn format_octal_max() {
        assert_eq!(format_octal(0o77777), [7, 7, 7, 7, 7]);
    }

    #[test]
    fn format_octal_value() {
        // 0o12345 = 0b001_010_011_100_101
        assert_eq!(format_octal(0o12345), [1, 2, 3, 4, 5]);
    }

    #[test]
    fn format_time_one_minute() {
        // 6042 centiseconds = 60 seconds 42 cs = 1 min 0 sec 42 cs
        let (m, s, cs) = format_time(6042);
        assert_eq!(m, 1);
        assert_eq!(s, 0);
        assert_eq!(cs, 42);
    }

    #[test]
    fn format_time_zero() {
        assert_eq!(format_time(0), (0, 0, 0));
    }

    #[test]
    fn encode_relay_word_field1_digits_1_2() {
        // field=1, d1=1, d2=2 →
        //  bits 13-11: 001
        //  bits 10-7:  0001
        //  bits  6-3:  0010
        //  bits  2-0:  000
        // = 0b_001_0001_0010_000 = 0x0890
        let word = encode_relay_word(1, 1, 2);
        assert_eq!(word, (1u16 << 11) | (1u16 << 7) | (2u16 << 3));
    }

    #[test]
    fn lookup_noun_known() {
        let def = lookup_noun(36).unwrap();
        assert_eq!(def.num_regs, 3);
        assert_eq!(def.scale[0], ScaleType::TimeMmSs);
    }

    #[test]
    fn lookup_noun_unknown() {
        assert!(lookup_noun(100).is_none());
    }
}
