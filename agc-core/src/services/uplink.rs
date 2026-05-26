//! UPRUPT path: ground uplink → V/N processor.
//!
//! [`poll_uplink`] drains words from the HAL [`Uplink`] FIFO and feeds them
//! into the V/N state machine via [`v_n::feed_key`]. Each uplink word carries
//! a single post-validated 5-bit DSKY key code in its lower five bits; the
//! Apollo redundancy / complement / "uplink too fast" protocol is the
//! responsibility of the bare-metal driver (`agc-board-nucleo-f767`) and of
//! the simulator HAL impl.
//!
//! AGC source: `Comanche055/KEYRUPT,_UPRUPT.agc` — the UPRUPT ISR routes the
//! validated key code through the same INREAD / NSTRT path that KEYRUPT uses
//! for crew keypresses, which is exactly what `feed_key` already implements.
//!
//! Called from the T4RUPT handler (see [`crate::services::t4rupt`]).

use crate::hal::Uplink;
use crate::services::v_n::{feed_key, Key, VnPhase};
use crate::tables::alarm_codes::UPLINK_TOO_FAST;
use crate::AgcState;

/// Extract a DSKY [`Key`] from a raw uplink word.
///
/// Only the lower 5 bits are inspected. Returns `None` for the all-zero
/// word (idle / no key) and for any code outside the Block 2 KEYTEMP1
/// table.
pub fn key_from_word(word: u16) -> Option<Key> {
    let code = (word & 0x1F) as u8;
    if code == 0 {
        return None;
    }
    Key::from_code(code)
}

/// Drain all pending uplink words and feed them into the V/N state machine.
///
/// Each non-empty word becomes a single keypress. Unknown codes are
/// silently dropped (matching the AGC's UPRUPT behaviour — bad codes
/// never reach NSTRT). Zero words are also dropped (idle line).
///
/// Side effects:
/// - `state.dsky.uplink_activity` is set whenever at least one word was
///   drained, and cleared on the first quiet poll. This drives the
///   DSKY's UPLINK ACTY lamp (T4 polls at 120 ms cadence in the sim, so
///   the lamp blinks at roughly the same rate as the wire traffic).
/// - Any keystroke arriving while `state.vn.phase == OprErr` raises
///   alarm `0o1106` (UPLINK TOO FAST). Ground is expected to send RSET
///   before continuing — see `specs/uplink-plan.md` §6 MS-U5.
pub fn poll_uplink<U: Uplink>(state: &mut AgcState, uplink: &mut U) {
    let mut drained = false;
    while let Some(word) = uplink.read_word() {
        drained = true;
        let key = match key_from_word(word) {
            Some(k) => k,
            None => continue,
        };
        // RSET out of an OprErr is the documented recovery path; let it
        // through. Any other key arriving while the V/N is locked in
        // OprErr is uplink-too-fast.
        if matches!(state.vn.phase, VnPhase::OprErr) && key != Key::Rset {
            state.alarm.raise(UPLINK_TOO_FAST);
            continue;
        }
        feed_key(state, key);
    }
    state.dsky.uplink_activity = drained;
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::services::v_n::VnPhase;
    use std::collections::VecDeque;

    /// Test-only Uplink impl backed by a VecDeque.
    struct VecUplink(VecDeque<u16>);

    impl Uplink for VecUplink {
        fn read_word(&mut self) -> Option<u16> {
            self.0.pop_front()
        }
    }

    fn uplink_of(words: &[u16]) -> VecUplink {
        VecUplink(words.iter().copied().collect())
    }

    /// TC-UPL-1: valid key codes round-trip through `key_from_word`.
    #[test]
    fn tc_upl_1_key_from_word_valid() {
        assert_eq!(key_from_word(17), Some(Key::Verb));
        assert_eq!(key_from_word(28), Some(Key::Entr));
        assert_eq!(key_from_word(16), Some(Key::Digit(0)));
        assert_eq!(key_from_word(1), Some(Key::Digit(1)));
        assert_eq!(key_from_word(9), Some(Key::Digit(9)));
    }

    /// TC-UPL-2: upper bits are ignored — only the low 5 bits matter.
    #[test]
    fn tc_upl_2_upper_bits_masked() {
        // 0xFFE1 = upper 11 bits set + code 1 (Digit(1))
        assert_eq!(key_from_word(0xFFE1), Some(Key::Digit(1)));
        // 0x8011 = high bit + Verb code
        assert_eq!(key_from_word(0x8011), Some(Key::Verb));
    }

    /// TC-UPL-3: the zero word and undefined 5-bit codes return None.
    #[test]
    fn tc_upl_3_unknown_and_zero() {
        assert_eq!(key_from_word(0), None);
        assert_eq!(key_from_word(0xFFE0), None); // low 5 bits = 0
        // 0..31 codes not in KEYTEMP1: 10..15, 19..24, 29.
        assert_eq!(key_from_word(10), None);
        assert_eq!(key_from_word(15), None);
        assert_eq!(key_from_word(29), None);
    }

    /// TC-UPL-4: `poll_uplink` drains a complete V71 ENTR keystroke
    /// sequence and reproduces the same V/N phase a direct `feed_key`
    /// drive would have produced.
    #[test]
    fn tc_upl_4_v71_sequence_drives_phase() {
        let mut state = AgcState::new();
        // V 7 1 ENTR — 17, 7, 1, 28
        let mut uplink = uplink_of(&[17, 7, 1, 28]);

        poll_uplink(&mut state, &mut uplink);

        assert!(
            matches!(state.vn.phase, VnPhase::P27Address { .. }),
            "V71 ENTR must transition to P27Address; got {:?}",
            state.vn.phase
        );
        // The HAL buffer is fully drained.
        assert_eq!(uplink.read_word(), None);
    }

    /// TC-UPL-5: unknown codes embedded in a stream are skipped without
    /// disturbing the surrounding keystrokes.
    #[test]
    fn tc_upl_5_unknown_codes_skipped() {
        let mut state = AgcState::new();
        // V (17), <noise=10>, 0 (16), <noise=15>, 6 (6), N (31), 4 (4),
        // 0 (16), ENTR (28) — full V06 N40 ENTR sequence with noise between
        // valid keystrokes.
        let mut uplink = uplink_of(&[17, 10, 16, 15, 6, 31, 4, 16, 28]);
        poll_uplink(&mut state, &mut uplink);
        assert_eq!(state.dsky.verb, 6);
        assert_eq!(state.dsky.noun, 40);
        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert!(!state.dsky.opr_err, "noise codes must not raise OPR ERR");
    }

    /// TC-UPL-6: poll_uplink on an empty FIFO is a no-op.
    #[test]
    fn tc_upl_6_empty_fifo_noop() {
        let mut state = AgcState::new();
        let initial_phase = state.vn.phase;
        let mut uplink = uplink_of(&[]);
        poll_uplink(&mut state, &mut uplink);
        assert_eq!(state.vn.phase, initial_phase);
    }

    /// TC-UPL-7: `uplink_activity` lights when a word is drained and
    /// clears on the next quiet poll.
    #[test]
    fn tc_upl_7_activity_lamp_toggles() {
        let mut state = AgcState::new();

        // Quiet poll → lamp off.
        let mut uplink = uplink_of(&[]);
        poll_uplink(&mut state, &mut uplink);
        assert!(!state.dsky.uplink_activity);

        // One word arrives → lamp on.
        let mut uplink = uplink_of(&[17]); // VERB
        poll_uplink(&mut state, &mut uplink);
        assert!(state.dsky.uplink_activity, "lamp must light after a drained word");

        // Next quiet poll → lamp off.
        let mut uplink = uplink_of(&[]);
        poll_uplink(&mut state, &mut uplink);
        assert!(!state.dsky.uplink_activity, "lamp must clear on a quiet poll");
    }

    /// TC-UPL-8: a keystroke arriving while the V/N is locked in OprErr
    /// raises alarm 01106 and the key is dropped. RSET still clears
    /// OprErr normally.
    #[test]
    fn tc_upl_8_alarm_01106_on_opr_err_overrun() {
        use crate::services::v_n::VnPhase;

        let mut state = AgcState::new();
        // Force an OPR ERR by sending a bad verb digit count.
        state.vn.phase = VnPhase::OprErr;
        state.dsky.opr_err = true;

        // Ground sends VERB instead of RSET → uplink-too-fast.
        let mut uplink = uplink_of(&[17]); // VERB
        poll_uplink(&mut state, &mut uplink);
        assert_eq!(state.alarm.code, UPLINK_TOO_FAST);
        assert!(state.alarm.lit);
        assert_eq!(
            state.vn.phase,
            VnPhase::OprErr,
            "alarming keystroke must not advance the V/N phase"
        );

        // Now RSET — should clear OprErr without raising 01106 again.
        state.alarm.lit = false;
        state.alarm.code = 0;
        let mut uplink = uplink_of(&[18]); // RSET
        poll_uplink(&mut state, &mut uplink);
        assert_eq!(state.alarm.code, 0);
        assert!(!state.alarm.lit);
        assert_eq!(state.vn.phase, VnPhase::Idle);
    }
}
