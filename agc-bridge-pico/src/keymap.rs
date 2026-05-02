//! ASCII keystroke → DSKY key code mapping.
//!
//! Key codes are the 5-bit values from the Block 2 AGC KEYTEMP1 table,
//! `Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc`.
//!
//! These integers must stay in sync with `agc_core::services::v_n::Key::from_code`:
//!   1..=9  → Digit(1)..Digit(9)
//!   16     → Digit(0)
//!   17     → Verb
//!   18     → Rset
//!   25     → Pro
//!   26     → Plus
//!   27     → Minus
//!   28     → Entr
//!   30     → Clr
//!   31     → Noun

/// Map an ASCII character to the corresponding DSKY 5-bit key code.
///
/// Returns `None` for unmapped characters.
pub fn ascii_to_dsky(c: u8) -> Option<u8> {
    match c {
        b'1' => Some(1),
        b'2' => Some(2),
        b'3' => Some(3),
        b'4' => Some(4),
        b'5' => Some(5),
        b'6' => Some(6),
        b'7' => Some(7),
        b'8' => Some(8),
        b'9' => Some(9),
        b'0' => Some(16),
        b'v' | b'V' => Some(17),
        b'r' | b'R' => Some(18),
        b'p' | b'P' => Some(25),
        b'+' => Some(26),
        b'-' => Some(27),
        b'\r' | b'\n' => Some(28),
        b'c' | b'C' => Some(30),
        b'n' | b'N' => Some(31),
        b'k' | b'K' => Some(25), // KeyRel shares the Pro line in the hardware
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn digits_0_through_9_all_map() {
        assert_eq!(ascii_to_dsky(b'0'), Some(16));
        for d in 1u8..=9 {
            let ch = b'0' + d;
            assert_eq!(ascii_to_dsky(ch), Some(d));
        }
    }

    #[test]
    fn control_keys_map() {
        assert_eq!(ascii_to_dsky(b'v'), Some(17));
        assert_eq!(ascii_to_dsky(b'V'), Some(17));
        assert_eq!(ascii_to_dsky(b'n'), Some(31));
        assert_eq!(ascii_to_dsky(b'N'), Some(31));
        assert_eq!(ascii_to_dsky(b'+'), Some(26));
        assert_eq!(ascii_to_dsky(b'-'), Some(27));
        assert_eq!(ascii_to_dsky(b'\r'), Some(28));
        assert_eq!(ascii_to_dsky(b'\n'), Some(28));
        assert_eq!(ascii_to_dsky(b'c'), Some(30));
        assert_eq!(ascii_to_dsky(b'C'), Some(30));
        assert_eq!(ascii_to_dsky(b'p'), Some(25));
        assert_eq!(ascii_to_dsky(b'P'), Some(25));
        assert_eq!(ascii_to_dsky(b'r'), Some(18));
        assert_eq!(ascii_to_dsky(b'R'), Some(18));
        assert_eq!(ascii_to_dsky(b'k'), Some(25));
        assert_eq!(ascii_to_dsky(b'K'), Some(25));
    }

    #[test]
    fn unmapped_chars_return_none() {
        assert_eq!(ascii_to_dsky(b'a'), None);
        assert_eq!(ascii_to_dsky(b'z'), None);
        assert_eq!(ascii_to_dsky(b' '), None);
        assert_eq!(ascii_to_dsky(0x1B), None); // ESC
    }

    #[test]
    fn all_mapped_codes_are_nonzero() {
        let mapped: &[u8] = b"0123456789vVnN+-\r\ncCpPrRkK";
        for &ch in mapped {
            let code = ascii_to_dsky(ch);
            assert!(code.is_some(), "char {} should map", ch as char);
            assert_ne!(code.unwrap(), 0, "code 0 is unused in the AGC key table");
        }
    }
}
