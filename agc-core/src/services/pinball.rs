//! DSKY display formatting (PINBALL layer).
//!
//! Pure-computation translator from `DskyState` to a decoded
//! `DskyFrame` ready to be pushed to the hardware. No HAL access —
//! the T4RUPT display ISR shim calls `decode_dsky` and then iterates
//! over the returned frame invoking `hw.dsky().write_row` and
//! `set_lamp` as needed.
//!
//! The name follows the AGC nomenclature (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc).
//!
//! Milestone 6 Phase 3 scope:
//! - `format_register` — f32 → signed 5-digit Register
//! - `format_two_digit` — u8 → TwoDigit (for PROG/VERB/NOUN)
//! - `decode_dsky` — DskyState → DskyFrame
//! - `digit_to_segments` — 0..9 → 7-segment bit pattern

use crate::services::display::DskyState;

// ── Types ─────────────────────────────────────────────────────────────────────

/// One register's decoded display state: sign + five digits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Register {
    /// `-1` (minus), `+1` (plus), or `0` (blank — value was zero).
    pub sign: i8,
    /// Five decimal digits, most-significant first, each `0..=9`.
    pub digits: [u8; 5],
    /// `true` if `|value|` exceeded 99_999 and the display was clamped.
    pub overflow: bool,
}

/// A two-digit display field (PROG/VERB/NOUN).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TwoDigit {
    pub tens: u8,
    pub units: u8,
}

/// Indicator-lamp state (copied verbatim out of `DskyState`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Lamps {
    pub uplink_activity: bool,
    pub no_att: bool,
    pub stby: bool,
    pub key_rel: bool,
    pub opr_err: bool,
    pub restart: bool,
    pub gimbal_lock: bool,
    pub temp: bool,
    pub prog_alarm: bool,
    pub comp_acty: bool,
}

/// A fully decoded DSKY frame ready for hardware write-out.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DskyFrame {
    pub prog: TwoDigit,
    pub verb: TwoDigit,
    pub noun: TwoDigit,
    pub r1: Register,
    pub r2: Register,
    pub r3: Register,
    pub lamps: Lamps,
    /// V35 lamp-test active — the display shim forces every lamp on.
    pub lamp_test: bool,
    /// VERB/NOUN flashing (crew input requested).
    pub flashing: bool,
}

// ── Formatters ────────────────────────────────────────────────────────────────

/// Format a signed f32 register value into a `Register`.
///
/// Rounds to the nearest integer (half-away-from-zero via `libm::round`),
/// extracts the sign, and decomposes the magnitude into five decimal
/// digits. Values with `|round(v)| > 99_999` are clamped to all-9s
/// with `overflow = true`. Non-finite inputs produce a blank display
/// with `overflow = true`.
pub fn format_register(value: f32) -> Register {
    if !value.is_finite() {
        return Register {
            sign: 0,
            digits: [0; 5],
            overflow: true,
        };
    }

    let n = libm::round(value as f64) as i64;

    let (sign, mut mag) = if n == 0 {
        (0i8, 0u64)
    } else if n > 0 {
        (1i8, n as u64)
    } else {
        // Use unsigned_abs-style conversion to avoid overflow on i64::MIN.
        (-1i8, n.unsigned_abs())
    };

    let overflow = mag > 99_999;
    if overflow {
        mag = 99_999;
    }

    let d0 = (mag / 10_000 % 10) as u8;
    let d1 = (mag / 1_000 % 10) as u8;
    let d2 = (mag / 100 % 10) as u8;
    let d3 = (mag / 10 % 10) as u8;
    let d4 = (mag % 10) as u8;

    Register {
        sign,
        digits: [d0, d1, d2, d3, d4],
        overflow,
    }
}

/// Format a `u8` into the two-digit display field.
///
/// Values `>= 100` are reduced modulo 100.
pub fn format_two_digit(n: u8) -> TwoDigit {
    let n = n % 100;
    TwoDigit {
        tens: n / 10,
        units: n % 10,
    }
}

/// Decode a `DskyState` into a ready-to-write `DskyFrame`.
pub fn decode_dsky(state: &DskyState) -> DskyFrame {
    DskyFrame {
        prog: format_two_digit(state.prog),
        verb: format_two_digit(state.verb),
        noun: format_two_digit(state.noun),
        r1: format_register(state.r[0]),
        r2: format_register(state.r[1]),
        r3: format_register(state.r[2]),
        lamps: Lamps {
            uplink_activity: state.uplink_activity,
            no_att: state.no_att,
            stby: state.stby,
            key_rel: state.key_rel,
            opr_err: state.opr_err,
            restart: state.restart_flag,
            gimbal_lock: state.gimbal_lock,
            temp: state.temp,
            prog_alarm: state.prog_alarm,
            comp_acty: state.comp_acty,
        },
        lamp_test: state.lamp_test_active,
        flashing: state.flashing,
    }
}

// ── Seven-segment encoding ────────────────────────────────────────────────────

/// Return the 7-segment bit pattern for a decimal digit.
///
/// Bit layout (common-cathode):
/// ```text
///     a          bit 0 = a (top)
///   f   b        bit 1 = b (top-right)
///     g          bit 2 = c (bottom-right)
///   e   c        bit 3 = d (bottom)
///     d          bit 4 = e (bottom-left)
///                bit 5 = f (top-left)
///                bit 6 = g (middle)
/// ```
///
/// Returns `0` (blank) for any input outside `0..=9`.
pub fn digit_to_segments(digit: u8) -> u8 {
    match digit {
        0 => 0x3F, // 0011 1111
        1 => 0x06, // 0000 0110
        2 => 0x5B, // 0101 1011
        3 => 0x4F, // 0100 1111
        4 => 0x66, // 0110 0110
        5 => 0x6D, // 0110 1101
        6 => 0x7D, // 0111 1101
        7 => 0x07, // 0000 0111
        8 => 0x7F, // 0111 1111
        9 => 0x6F, // 0110 1111
        _ => 0x00,
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// TC-PB-1: zero maps to blank sign, all-zero digits.
    #[test]
    fn tc_pb_1_zero() {
        let r = format_register(0.0);
        assert_eq!(r.sign, 0);
        assert_eq!(r.digits, [0, 0, 0, 0, 0]);
        assert!(!r.overflow);
    }

    /// TC-PB-2: 12345 → +12345.
    #[test]
    fn tc_pb_2_positive_five_digits() {
        let r = format_register(12_345.0);
        assert_eq!(r.sign, 1);
        assert_eq!(r.digits, [1, 2, 3, 4, 5]);
        assert!(!r.overflow);
    }

    /// TC-PB-3: -7 → -00007.
    #[test]
    fn tc_pb_3_negative_small() {
        let r = format_register(-7.0);
        assert_eq!(r.sign, -1);
        assert_eq!(r.digits, [0, 0, 0, 0, 7]);
        assert!(!r.overflow);
    }

    /// TC-PB-4: max non-overflow value.
    #[test]
    fn tc_pb_4_max_positive() {
        let r = format_register(99_999.0);
        assert_eq!(r.sign, 1);
        assert_eq!(r.digits, [9, 9, 9, 9, 9]);
        assert!(!r.overflow);
    }

    /// TC-PB-5: positive overflow.
    #[test]
    fn tc_pb_5_positive_overflow() {
        let r = format_register(100_000.0);
        assert_eq!(r.sign, 1);
        assert_eq!(r.digits, [9, 9, 9, 9, 9]);
        assert!(r.overflow);
    }

    /// TC-PB-6: negative overflow.
    #[test]
    fn tc_pb_6_negative_overflow() {
        let r = format_register(-100_000.0);
        assert_eq!(r.sign, -1);
        assert_eq!(r.digits, [9, 9, 9, 9, 9]);
        assert!(r.overflow);
    }

    /// TC-PB-7: 3.7 rounds to 4.
    #[test]
    fn tc_pb_7_rounds_up() {
        let r = format_register(3.7);
        assert_eq!(r.sign, 1);
        assert_eq!(r.digits, [0, 0, 0, 0, 4]);
    }

    /// TC-PB-8: -2.5 rounds to -3 (libm half-away-from-zero).
    #[test]
    fn tc_pb_8_half_away_from_zero() {
        let r = format_register(-2.5);
        assert_eq!(r.sign, -1);
        assert_eq!(r.digits, [0, 0, 0, 0, 3]);
    }

    /// TC-PB-9: NaN produces a blank display with overflow set.
    #[test]
    fn tc_pb_9_nan() {
        let r = format_register(f32::NAN);
        assert_eq!(r.sign, 0);
        assert_eq!(r.digits, [0; 5]);
        assert!(r.overflow);
    }

    /// TC-PB-9b: +inf produces a blank display with overflow set.
    #[test]
    fn tc_pb_9b_infinity() {
        let r = format_register(f32::INFINITY);
        assert_eq!(r.sign, 0);
        assert!(r.overflow);
    }

    /// TC-PB-10: 37 → { tens: 3, units: 7 }.
    #[test]
    fn tc_pb_10_two_digit_normal() {
        let td = format_two_digit(37);
        assert_eq!(td.tens, 3);
        assert_eq!(td.units, 7);
    }

    /// TC-PB-11: 105 → 05 (reduced mod 100).
    #[test]
    fn tc_pb_11_two_digit_mod() {
        let td = format_two_digit(105);
        assert_eq!(td.tens, 0);
        assert_eq!(td.units, 5);
    }

    /// TC-PB-12: 7-segment encoding table is exact.
    #[test]
    fn tc_pb_12_segment_table() {
        assert_eq!(digit_to_segments(0), 0x3F);
        assert_eq!(digit_to_segments(1), 0x06);
        assert_eq!(digit_to_segments(2), 0x5B);
        assert_eq!(digit_to_segments(3), 0x4F);
        assert_eq!(digit_to_segments(4), 0x66);
        assert_eq!(digit_to_segments(5), 0x6D);
        assert_eq!(digit_to_segments(6), 0x7D);
        assert_eq!(digit_to_segments(7), 0x07);
        assert_eq!(digit_to_segments(8), 0x7F);
        assert_eq!(digit_to_segments(9), 0x6F);
        assert_eq!(digit_to_segments(10), 0x00);
        assert_eq!(digit_to_segments(255), 0x00);
    }

    /// TC-PB-13: end-to-end `decode_dsky` composition.
    #[test]
    fn tc_pb_13_decode_dsky_end_to_end() {
        let mut state = DskyState::default();
        state.prog = 37;
        state.verb = 6;
        state.noun = 40;
        state.r = [100.0, -2.5, 0.0];
        state.opr_err = true;
        state.flashing = true;

        let frame = decode_dsky(&state);

        assert_eq!(frame.prog, TwoDigit { tens: 3, units: 7 });
        assert_eq!(frame.verb, TwoDigit { tens: 0, units: 6 });
        assert_eq!(frame.noun, TwoDigit { tens: 4, units: 0 });

        assert_eq!(
            frame.r1,
            Register { sign: 1, digits: [0, 0, 1, 0, 0], overflow: false }
        );
        // -2.5 rounds to -3.
        assert_eq!(
            frame.r2,
            Register { sign: -1, digits: [0, 0, 0, 0, 3], overflow: false }
        );
        assert_eq!(
            frame.r3,
            Register { sign: 0, digits: [0; 5], overflow: false }
        );

        assert!(frame.lamps.opr_err);
        assert!(frame.flashing);
        assert!(!frame.lamp_test);
    }
}
