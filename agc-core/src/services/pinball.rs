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
    pub tracker: bool,
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
            tracker: state.tracker,
        },
        lamp_test: state.lamp_test_active,
        flashing: state.flashing,
    }
}

// ── Hardware emission ─────────────────────────────────────────────────────────

/// Emit a decoded `DskyFrame` to the hardware DSKY via the row-encoding protocol.
///
/// Writes 21 `write_row` calls covering PROG/VERB/NOUN and the three data
/// registers (sign + 5 digits each), then updates all 10 indicator lamps.
/// The `set_flash` call is NOT made here — the caller (T4 drain) issues it
/// separately so this function remains pure frame → HAL with no extra state.
///
/// Row encoding (ADR-019):
/// - Row 0: PROG  — `(tens << 4) | units`; 0xF = blank digit
/// - Row 1: VERB  — same
/// - Row 2: NOUN  — same
/// - Row 3: R1 sign — 0=blank, 1='+', 2='-'
/// - Rows 4–8: R1 digits 0–4 (most-significant first); 0x0..0x9 or 0xF=blank
/// - Row 9:  R2 sign
/// - Rows 10–14: R2 digits
/// - Row 15: R3 sign
/// - Rows 16–20: R3 digits
///
/// Lamps are mapped to the `Lamp` enum; the `tracker` lamp in `Lamps` has no
/// HAL variant and is silently skipped.
pub fn emit_dsky_to_hw<H: crate::hal::Dsky>(frame: &DskyFrame, dsky: &mut H) {
    #[inline]
    fn pack_two_digit(td: TwoDigit) -> u16 {
        ((td.tens as u16) << 4) | (td.units as u16)
    }

    #[inline]
    fn sign_byte(s: i8) -> u16 {
        match s {
            1 => 1,  // plus
            -1 => 2, // minus
            _ => 0,  // blank
        }
    }

    dsky.write_row(0, pack_two_digit(frame.prog));
    dsky.write_row(1, pack_two_digit(frame.verb));
    dsky.write_row(2, pack_two_digit(frame.noun));

    // R1: rows 3–8
    dsky.write_row(3, sign_byte(frame.r1.sign));
    for (i, &d) in frame.r1.digits.iter().enumerate() {
        dsky.write_row(4 + i as u8, d as u16);
    }

    // R2: rows 9–14
    dsky.write_row(9, sign_byte(frame.r2.sign));
    for (i, &d) in frame.r2.digits.iter().enumerate() {
        dsky.write_row(10 + i as u8, d as u16);
    }

    // R3: rows 15–20
    dsky.write_row(15, sign_byte(frame.r3.sign));
    for (i, &d) in frame.r3.digits.iter().enumerate() {
        dsky.write_row(16 + i as u8, d as u16);
    }

    // Lamps (tracker has no HAL variant — omitted).
    use crate::hal::Lamp;
    let l = &frame.lamps;
    dsky.set_lamp(Lamp::UplinkActivity, l.uplink_activity);
    dsky.set_lamp(Lamp::NoAtt, l.no_att);
    dsky.set_lamp(Lamp::Stby, l.stby);
    dsky.set_lamp(Lamp::KeyRel, l.key_rel);
    dsky.set_lamp(Lamp::OprErr, l.opr_err);
    dsky.set_lamp(Lamp::Restart, l.restart);
    dsky.set_lamp(Lamp::GimbalLock, l.gimbal_lock);
    dsky.set_lamp(Lamp::Temp, l.temp);
    dsky.set_lamp(Lamp::ProgAlarm, l.prog_alarm);
    dsky.set_lamp(Lamp::CompActy, l.comp_acty);
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

    // ── emit_dsky_to_hw tests ─────────────────────────────────────────────────

    /// Minimal mock that records all `write_row` and `set_lamp` calls.
    struct MockDsky {
        rows: [(u8, u16); 32],
        row_count: usize,
        lamps: [(crate::hal::Lamp, bool); 16],
        lamp_count: usize,
        flash: Option<bool>,
    }

    impl MockDsky {
        fn new() -> Self {
            Self {
                rows: [(0, 0); 32],
                row_count: 0,
                lamps: [(crate::hal::Lamp::CompActy, false); 16],
                lamp_count: 0,
                flash: None,
            }
        }

        fn row(&self, r: u8) -> Option<u16> {
            self.rows[..self.row_count]
                .iter()
                .find(|&&(row, _)| row == r)
                .map(|&(_, data)| data)
        }

        fn lamp(&self, l: crate::hal::Lamp) -> Option<bool> {
            self.lamps[..self.lamp_count]
                .iter()
                .find(|&&(lamp, _)| lamp == l)
                .map(|&(_, on)| on)
        }
    }

    impl crate::hal::Dsky for MockDsky {
        fn write_row(&mut self, row: u8, data: u16) {
            if self.row_count < self.rows.len() {
                self.rows[self.row_count] = (row, data);
                self.row_count += 1;
            }
        }
        fn clear_row(&mut self, _row: u8) {}
        fn set_lamp(&mut self, lamp: crate::hal::Lamp, on: bool) {
            if self.lamp_count < self.lamps.len() {
                self.lamps[self.lamp_count] = (lamp, on);
                self.lamp_count += 1;
            }
        }
        fn set_flash(&mut self, on: bool) {
            self.flash = Some(on);
        }
        fn read_key(&mut self) -> Option<u8> {
            None
        }
    }

    fn blank_frame() -> DskyFrame {
        DskyFrame {
            prog: TwoDigit { tens: 0, units: 0 },
            verb: TwoDigit { tens: 0, units: 0 },
            noun: TwoDigit { tens: 0, units: 0 },
            r1: Register {
                sign: 0,
                digits: [0; 5],
                overflow: false,
            },
            r2: Register {
                sign: 0,
                digits: [0; 5],
                overflow: false,
            },
            r3: Register {
                sign: 0,
                digits: [0; 5],
                overflow: false,
            },
            lamps: Lamps {
                uplink_activity: false,
                no_att: false,
                stby: false,
                key_rel: false,
                opr_err: false,
                restart: false,
                gimbal_lock: false,
                temp: false,
                prog_alarm: false,
                comp_acty: false,
                tracker: false,
            },
            lamp_test: false,
            flashing: false,
        }
    }

    /// TC-PB-E1: all-blank frame: every digit field is zero, all lamps off.
    #[test]
    fn tc_pb_e1_all_blank() {
        let frame = blank_frame();
        let mut dsky = MockDsky::new();
        emit_dsky_to_hw(&frame, &mut dsky);

        // PROG row 0: tens=0, units=0 → (0<<4)|0 = 0x00
        assert_eq!(dsky.row(0), Some(0x00), "PROG row");
        // VERB row 1
        assert_eq!(dsky.row(1), Some(0x00), "VERB row");
        // NOUN row 2
        assert_eq!(dsky.row(2), Some(0x00), "NOUN row");
        // R1 sign row 3: blank → 0
        assert_eq!(dsky.row(3), Some(0), "R1 sign blank");
        // R1 digits rows 4–8: all zero
        for r in 4u8..=8 {
            assert_eq!(dsky.row(r), Some(0), "R1 digit row {r}");
        }
        // 21 write_row calls total
        assert_eq!(dsky.row_count, 21, "21 write_row calls");
        // 10 set_lamp calls, all false
        assert_eq!(dsky.lamp_count, 10, "10 set_lamp calls");
        for i in 0..dsky.lamp_count {
            assert!(!dsky.lamps[i].1, "lamp {} should be off", i);
        }
    }

    /// TC-PB-E2: VERB=37 → row 1 data = `(3<<4)|7` = 0x37.
    #[test]
    fn tc_pb_e2_verb_37() {
        let mut frame = blank_frame();
        frame.verb = TwoDigit { tens: 3, units: 7 };
        let mut dsky = MockDsky::new();
        emit_dsky_to_hw(&frame, &mut dsky);
        assert_eq!(dsky.row(1), Some(0x37), "VERB=37 → 0x37");
    }

    /// TC-PB-E3: R1 = -00123 → sign row=2(minus), digits=[0,0,1,2,3].
    #[test]
    fn tc_pb_e3_r1_negative_123() {
        let mut frame = blank_frame();
        frame.r1 = Register {
            sign: -1,
            digits: [0, 0, 1, 2, 3],
            overflow: false,
        };
        let mut dsky = MockDsky::new();
        emit_dsky_to_hw(&frame, &mut dsky);

        assert_eq!(dsky.row(3), Some(2), "R1 sign = minus (2)");
        assert_eq!(dsky.row(4), Some(0), "R1[0] = 0");
        assert_eq!(dsky.row(5), Some(0), "R1[1] = 0");
        assert_eq!(dsky.row(6), Some(1), "R1[2] = 1");
        assert_eq!(dsky.row(7), Some(2), "R1[3] = 2");
        assert_eq!(dsky.row(8), Some(3), "R1[4] = 3");
    }

    /// TC-PB-E4: all 10 lamps on → 10 set_lamp(_, true) calls.
    #[test]
    fn tc_pb_e4_all_lamps_on() {
        use crate::hal::Lamp;
        let mut frame = blank_frame();
        frame.lamps = Lamps {
            uplink_activity: true,
            no_att: true,
            stby: true,
            key_rel: true,
            opr_err: true,
            restart: true,
            gimbal_lock: true,
            temp: true,
            prog_alarm: true,
            comp_acty: true,
            tracker: true, // tracker has no HAL variant; must not crash
        };
        let mut dsky = MockDsky::new();
        emit_dsky_to_hw(&frame, &mut dsky);

        assert_eq!(dsky.lamp_count, 10, "exactly 10 lamps");
        assert_eq!(dsky.lamp(Lamp::UplinkActivity), Some(true));
        assert_eq!(dsky.lamp(Lamp::NoAtt), Some(true));
        assert_eq!(dsky.lamp(Lamp::Stby), Some(true));
        assert_eq!(dsky.lamp(Lamp::KeyRel), Some(true));
        assert_eq!(dsky.lamp(Lamp::OprErr), Some(true));
        assert_eq!(dsky.lamp(Lamp::Restart), Some(true));
        assert_eq!(dsky.lamp(Lamp::GimbalLock), Some(true));
        assert_eq!(dsky.lamp(Lamp::Temp), Some(true));
        assert_eq!(dsky.lamp(Lamp::ProgAlarm), Some(true));
        assert_eq!(dsky.lamp(Lamp::CompActy), Some(true));
    }

    /// TC-PB-13: end-to-end `decode_dsky` composition.
    #[test]
    fn tc_pb_13_decode_dsky_end_to_end() {
        let state = DskyState {
            prog: 37,
            verb: 6,
            noun: 40,
            r: [100.0, -2.5, 0.0],
            opr_err: true,
            flashing: true,
            ..Default::default()
        };

        let frame = decode_dsky(&state);

        assert_eq!(frame.prog, TwoDigit { tens: 3, units: 7 });
        assert_eq!(frame.verb, TwoDigit { tens: 0, units: 6 });
        assert_eq!(frame.noun, TwoDigit { tens: 4, units: 0 });

        assert_eq!(
            frame.r1,
            Register {
                sign: 1,
                digits: [0, 0, 1, 0, 0],
                overflow: false
            }
        );
        // -2.5 rounds to -3.
        assert_eq!(
            frame.r2,
            Register {
                sign: -1,
                digits: [0, 0, 0, 0, 3],
                overflow: false
            }
        );
        assert_eq!(
            frame.r3,
            Register {
                sign: 0,
                digits: [0; 5],
                overflow: false
            }
        );

        assert!(frame.lamps.opr_err);
        assert!(frame.flashing);
        assert!(!frame.lamp_test);
    }
}
