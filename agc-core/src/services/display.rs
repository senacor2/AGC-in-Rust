//! Display formatting utilities — relay-code generation for the DSKY.
//!
//! Converts numeric values to 5-bit relay codes that drive the DSKY
//! seven-segment display panel.
//!
//! AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc
//! Routines:   DSPDECWD, DSPDECNR, DSPDC2NR, DSP2DEC, DSPOCTWO, DSPIN, DSPSIGN,
//!             DSPRND, HMSOUT, M/SOUT, 2BLANK, 5BLANK, DSPDECVN
//! Pages:      359-364 (BANK 40/42, multiple SETLOC)
//!
//! Secondary:
//! AGC source: Comanche055/T4RUPT_PROGRAM.agc
//! Routines:   DSPOUT, DSPOUTSB, DSPLAY, RELTAB, RELTAB11
//! Pages:      133-136 (BANK 12, SETLOC T4RUP / BLOCK 02 FFTAG12)

/// 5-bit relay codes for digits 0–9 and blank, indexed by digit value.
///
/// Index 0-9 are the digit relay codes.
/// Index 10 is the blank relay code (no segments driven).
///
/// Blank (0x00) is distinct from zero (0x15): blank means "no data",
/// zero means "data equals zero". This matches the AGC BLANKCON / DSPDECWD
/// distinction.
///
/// AGC source: PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, page 314.
/// "THE 5-BIT OUTPUT RELAY CODES ARE: BLANK 00000, 0 10101, 1 00011, ..."
pub const RELAY_CODES: [u8; 11] = [
    0b10101, // 0 → 0x15
    0b00011, // 1 → 0x03
    0b11001, // 2 → 0x19
    0b11011, // 3 → 0x1B
    0b01111, // 4 → 0x0F
    0b11110, // 5 → 0x1E
    0b11100, // 6 → 0x1C
    0b10011, // 7 → 0x13
    0b11101, // 8 → 0x1D
    0b11111, // 9 → 0x1F
    0b00000, // blank (index 10) → 0x00
];

/// Index into `RELAY_CODES` for a blank digit.
///
/// Use as a sentinel to distinguish "no data" from "data = 0".
///
/// AGC source: PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, BLANKCON = OCT 4000.
pub const BLANK: u8 = 10;

/// Sign relay bit position within a DSPTAB entry (bit 11, 1-indexed = bit 10, 0-indexed).
///
/// When set: minus sign relay lit.
/// When clear: plus sign relay (or no sign).
///
/// AGC source: PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, page 313-314 (DSPTAB format).
pub const SIGN_BIT: u16 = 1 << 10;

/// A formatted 5-digit display field.
///
/// Each element is an index into `RELAY_CODES` (0–9 for digits, `BLANK` = 10 for blank).
/// Layout: [D1, D2, D3, D4, D5] left-to-right.
pub type DisplayField = [u8; 5];

/// Sign state for a register's leading sign position.
///
/// AGC source: DSPSIGN routine, PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, page 359.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Sign {
    /// Plus sign relay lit.
    Plus,
    /// Minus sign relay lit.
    Minus,
    /// Sign position blank (no sign, as for octal or verb/noun displays).
    Blank,
}

/// Complete formatted state for one R register (sign + 5 digits).
///
/// Each `digits[i]` is an index into `RELAY_CODES`.
///
/// AGC source: DSPDECWD, DSPOCTWO, PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, pages 359-361.
#[derive(Clone, Copy, Debug)]
pub struct RegisterDisplay {
    /// Sign indicator for the register.
    pub sign: Sign,
    /// 5-digit display field (indices into `RELAY_CODES`).
    pub digits: DisplayField,
}

/// A time value formatted across three display registers.
///
/// AGC source: HMSOUT, PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, pages 338-341 (BANK 42).
#[derive(Clone, Copy, Debug)]
pub struct TimeDisplay {
    /// Hours in R1 (0–999 range; 5 digits, format 0HHHH).
    pub r1: RegisterDisplay,
    /// Minutes in R2 (0–59; format 000MM).
    pub r2: RegisterDisplay,
    /// Seconds × 100 in R3 (0–5999; format 0SS.ss with implied decimal after D3).
    pub r3: RegisterDisplay,
}

/// Convert a digit value 0–9 to its 5-bit relay code.
///
/// Returns `RELAY_CODES[BLANK as usize]` (0x00) for any value outside 0–9.
///
/// AGC source: PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, page 314 relay table.
pub const fn digit_to_relay(d: u8) -> u8 {
    if d <= 9 {
        RELAY_CODES[d as usize]
    } else {
        RELAY_CODES[BLANK as usize]
    }
}

/// Return a blank 5-digit register display (all digits = `BLANK` relay code index).
///
/// Blank represents "no data" (relay code 0x00 for each digit position).
/// This is distinct from `format_decimal(0, true)` which shows 00000 (relay 0x15 each).
///
/// AGC source: 5BLANK / 2BLANK write BLANKCON to DSPTAB entries.
/// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, page 322.
pub const fn blank() -> RegisterDisplay {
    RegisterDisplay {
        sign: Sign::Blank,
        digits: [BLANK; 5],
    }
}

/// Format a signed 32-bit integer as a 5-digit decimal display field.
///
/// The value is clamped to the range [-99999, +99999]. Values outside this range
/// saturate: the display shows 99999 with the appropriate sign. This matches the
/// AGC's TESTOFUF alarm-and-recycle for out-of-range inputs, re-mapped to saturation
/// in Rust (no heap, no panic).
///
/// When `with_sign = false`, returns `Sign::Blank` (used for verb/noun fields).
///
/// AGC source: DSPDECWD (rounds, then extracts 5 digits).
/// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, pages 359-360.
pub fn format_decimal(value: i32, with_sign: bool) -> RegisterDisplay {
    let saturated = value.clamp(-99_999, 99_999);
    let sign = if with_sign {
        if saturated < 0 {
            Sign::Minus
        } else {
            Sign::Plus
        }
    } else {
        Sign::Blank
    };

    let magnitude = saturated.unsigned_abs();
    let digits = [
        ((magnitude / 10_000) % 10) as u8,
        ((magnitude / 1_000) % 10) as u8,
        ((magnitude / 100) % 10) as u8,
        ((magnitude / 10) % 10) as u8,
        (magnitude % 10) as u8,
    ];

    RegisterDisplay { sign, digits }
}

/// Format a 15-bit octal value as 5 octal digits (no sign).
///
/// Only the low 15 bits of `value` are used (AGC word is 15 bits + sign).
/// Sign position is left blank.
///
/// AGC source: DSPOCTWO.
/// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, page 361.
pub fn format_octal(value: u16) -> RegisterDisplay {
    let v = value & 0x7FFF; // keep only low 15 bits
    let digits = [
        ((v >> 12) & 0x7) as u8,
        ((v >> 9) & 0x7) as u8,
        ((v >> 6) & 0x7) as u8,
        ((v >> 3) & 0x7) as u8,
        (v & 0x7) as u8,
    ];
    RegisterDisplay {
        sign: Sign::Blank,
        digits,
    }
}

/// Format a mission elapsed time (in centiseconds) across R1/R2/R3.
///
/// Scale: 1 unit = 1 centisecond.
/// R1 = whole hours (0–999), R2 = minutes mod 60 (0–59),
/// R3 = seconds × 100 mod 6000 (0–5999, format SS.ss with implied decimal).
///
/// Rounding: rounds to the nearest centisecond is a no-op (input is already in cs).
/// Hours/minutes/seconds decomposition per integer arithmetic.
///
/// AGC source: HMSOUT.
/// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, pages 338-341 (BANK 42).
///
/// Format note per spec: R3 displays seconds with two implied decimal places
/// (e.g., 1.00 second = value 100 centiseconds → digits [0,1,0,0,0]).
pub fn format_time(centiseconds: u32) -> TimeDisplay {
    // Round to nearest second using (cs + 50) / 100 per spec decision.
    let total_secs = (centiseconds + 50) / 100;
    let hours = total_secs / 3600;
    let minutes = (total_secs % 3600) / 60;
    let secs = total_secs % 60;

    // R3 shows SS.ss (seconds * 100 mod 6000).
    // Since we rounded to whole seconds, the cs part is always .00.
    let r3_val = secs * 100;

    TimeDisplay {
        r1: format_decimal(hours.min(99_999) as i32, true),
        r2: format_decimal(minutes as i32, true),
        r3: format_decimal(r3_val as i32, true),
    }
}

/// Format a minutes:seconds value (in centiseconds) for a single register.
///
/// Used by M/S (minutes/seconds) display format (SF code 01001).
/// Limits to 59:59 (5999 centiseconds) per AGC M/SCON3.
/// Returns: sign=Plus, D1D2 = minutes, D3 = blank (BLANK sentinel), D4D5 = seconds.
///
/// AGC source: M/SOUT.
/// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, pages 339-341 (BANK 42).
pub fn format_min_sec(centiseconds: u32) -> RegisterDisplay {
    // Clamp to 59:59 = 5999 centiseconds.
    let cs = centiseconds.min(5999);
    let total_secs = cs / 100;
    let minutes = (total_secs / 60).min(59) as u8;
    let seconds = (total_secs % 60) as u8;

    let digits = [
        minutes / 10,
        minutes % 10,
        BLANK, // D3 is always blank in M/S format
        seconds / 10,
        seconds % 10,
    ];

    RegisterDisplay {
        sign: Sign::Plus,
        digits,
    }
}

/// Convert a `RegisterDisplay` to two DSPTAB u16 words that DSPOUT would write.
///
/// This is the inverse of the DSPIN / DSPOUT formatting used in T4RUPT.
/// Used by agc-sim to render the display without running the T4 interrupt.
///
/// The two words encode the sign bit and relay codes for up to 4 digits each.
/// `dsptab_hi` encodes sign + D1D2, `dsptab_lo` encodes D3D4D5 (approx).
///
/// AGC source: DSPIN, 11DSPIN, DSPOUT.
/// T4RUPT_PROGRAM.agc, pages 134-136.
pub fn to_dsptab(display: &RegisterDisplay, dsptab_hi: &mut u16, dsptab_lo: &mut u16) {
    let sign_bit: u16 = if display.sign == Sign::Minus {
        SIGN_BIT
    } else {
        0
    };

    // Encode relay codes for digits.
    let d0 = RELAY_CODES[display.digits[0] as usize] as u16;
    let d1 = RELAY_CODES[display.digits[1] as usize] as u16;
    let d2 = RELAY_CODES[display.digits[2] as usize] as u16;
    let d3 = RELAY_CODES[display.digits[3] as usize] as u16;
    let d4 = RELAY_CODES[display.digits[4] as usize] as u16;

    // High word: sign + D1 (5 bits) + D2 (5 bits).
    *dsptab_hi = sign_bit | (d0 << 5) | d1;
    // Low word: D3 (5 bits) + D4 (5 bits) + D5 (5 bits).
    *dsptab_lo = (d2 << 10) | (d3 << 5) | d4;
}

// ── tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// TC-DISP-1: Positive decimal formatting.
    ///
    /// AGC source: DSPDECWD test case from spec.
    #[test]
    fn format_decimal_positive() {
        let rd = format_decimal(12345, true);
        assert_eq!(rd.sign, Sign::Plus);
        assert_eq!(rd.digits, [1, 2, 3, 4, 5]);
        assert_eq!(RELAY_CODES[rd.digits[0] as usize], 0x03);
        assert_eq!(RELAY_CODES[rd.digits[1] as usize], 0x19);
        assert_eq!(RELAY_CODES[rd.digits[2] as usize], 0x1B);
        assert_eq!(RELAY_CODES[rd.digits[3] as usize], 0x0F);
        assert_eq!(RELAY_CODES[rd.digits[4] as usize], 0x1E);
    }

    /// TC-DISP-2: Negative decimal formatting.
    ///
    /// AGC source: DSPDECWD negative value, DSPSIGN sets minus relay.
    #[test]
    fn format_decimal_negative() {
        let rd = format_decimal(-7, true);
        assert_eq!(rd.sign, Sign::Minus);
        assert_eq!(rd.digits, [0, 0, 0, 0, 7]);
        assert_eq!(RELAY_CODES[rd.digits[4] as usize], 0x13);
    }

    /// TC-DISP-3: Zero is distinct from blank.
    ///
    /// AGC source: DSPDECWD(0) → relay 0x15 each digit; 5BLANK → relay 0x00 each.
    #[test]
    fn format_decimal_zero_vs_blank() {
        let zero = format_decimal(0, true);
        assert_eq!(zero.sign, Sign::Plus);
        assert_eq!(zero.digits, [0, 0, 0, 0, 0]);
        // relay code for digit 0 is 0x15
        for &d in &zero.digits {
            assert_eq!(RELAY_CODES[d as usize], 0x15);
        }

        let b = blank();
        assert_eq!(b.sign, Sign::Blank);
        assert_eq!(b.digits, [BLANK; 5]);
        // relay code for blank is 0x00
        for &d in &b.digits {
            assert_eq!(RELAY_CODES[d as usize], 0x00);
        }
    }

    /// TC-DISP-4: Octal formatting.
    ///
    /// AGC source: DSPOCTWO test case from spec (octal 53170 = decimal 22136).
    #[test]
    fn format_octal_value() {
        // 0b101_011_001_111_000 = octal 53170
        let v: u16 = 0b101_011_001_111_000;
        let rd = format_octal(v);
        assert_eq!(rd.sign, Sign::Blank);
        assert_eq!(rd.digits, [5, 3, 1, 7, 0]);
        assert_eq!(RELAY_CODES[rd.digits[0] as usize], 0x1E); // 5
        assert_eq!(RELAY_CODES[rd.digits[1] as usize], 0x1B); // 3
        assert_eq!(RELAY_CODES[rd.digits[2] as usize], 0x03); // 1
        assert_eq!(RELAY_CODES[rd.digits[3] as usize], 0x13); // 7
        assert_eq!(RELAY_CODES[rd.digits[4] as usize], 0x15); // 0
    }

    /// TC-DISP-5: Time formatting.
    ///
    /// AGC source: HMSOUT test case from spec (3661 seconds = 1h01m01s).
    #[test]
    fn format_time_value() {
        // 3661 seconds = 366100 centiseconds.
        let td = format_time(366_100);
        assert_eq!(td.r1.sign, Sign::Plus);
        assert_eq!(td.r1.digits, [0, 0, 0, 0, 1]); // 1 hour
        assert_eq!(td.r2.sign, Sign::Plus);
        assert_eq!(td.r2.digits, [0, 0, 0, 0, 1]); // 1 minute
        assert_eq!(td.r3.sign, Sign::Plus);
        // 1 second = 100 centiseconds → displayed as 00100 (format 0SS.ss)
        assert_eq!(td.r3.digits, [0, 0, 1, 0, 0]);
    }

    /// TC-DISP-6: Blank field.
    ///
    /// AGC source: 5BLANK writes BLANKCON (relay code 0x00) to DSPTAB.
    #[test]
    fn blank_field() {
        let b = blank();
        assert_eq!(b.sign, Sign::Blank);
        assert_eq!(b.digits, [10, 10, 10, 10, 10]);
        for &d in &b.digits {
            assert_eq!(RELAY_CODES[d as usize], 0x00);
        }
    }

    /// TC-DISP-7: Saturation clamp at ±99999.
    #[test]
    fn format_decimal_saturation() {
        let pos = format_decimal(200_000, true);
        assert_eq!(pos.sign, Sign::Plus);
        assert_eq!(pos.digits, [9, 9, 9, 9, 9]);

        let neg = format_decimal(-200_000, true);
        assert_eq!(neg.sign, Sign::Minus);
        assert_eq!(neg.digits, [9, 9, 9, 9, 9]);
    }

    /// TC-DISP-8: digit_to_relay out-of-range returns blank code.
    #[test]
    fn digit_to_relay_oob() {
        assert_eq!(digit_to_relay(10), 0x00);
        assert_eq!(digit_to_relay(255), 0x00);
    }

    /// TC-DISP-9: format_min_sec limits to 59:59 and blanks D3.
    #[test]
    fn format_min_sec_value() {
        // 125 cs = 1 second 25 cs → 1s total → 0 min, 1 sec
        let rd = format_min_sec(125);
        assert_eq!(rd.sign, Sign::Plus);
        assert_eq!(rd.digits[0], 0); // tens of minutes
        assert_eq!(rd.digits[1], 0); // units of minutes
        assert_eq!(rd.digits[2], BLANK); // D3 always blank
        assert_eq!(rd.digits[3], 0); // tens of seconds
        assert_eq!(rd.digits[4], 1); // units of seconds

        // Clamp test: 9999 cs > 5999 max → 59s
        let rd2 = format_min_sec(9999);
        assert_eq!(rd2.digits[4], 9); // 59 seconds, units = 9
    }

    /// TC-DISP-10: without_sign produces Blank sign.
    #[test]
    fn format_decimal_no_sign() {
        let rd = format_decimal(42, false);
        assert_eq!(rd.sign, Sign::Blank);
        assert_eq!(rd.digits[3], 4);
        assert_eq!(rd.digits[4], 2);
    }
}
