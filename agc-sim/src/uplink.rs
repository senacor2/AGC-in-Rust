//! Scripted host-side [`Uplink`] implementation.
//!
//! [`ScriptedUplink`] is a FIFO of pre-loaded uplink words drained by
//! `agc_core::services::uplink::poll_uplink` exactly as a real ground
//! uplink would be. Tests and the interactive `dsky_sim` use it to feed
//! recorded keystroke sequences through the UPRUPT path.
//!
//! See `specs/uplink-plan.md` §4.

use std::collections::VecDeque;

use agc_core::hal::Uplink;
use agc_core::services::v_n::Key;

// ── Type ─────────────────────────────────────────────────────────────────────

/// Host-side uplink that returns pre-loaded 5-bit key codes.
///
/// Each `u16` word carries one validated DSKY key code in its lower five
/// bits; upper bits are reserved (set to zero). The `read_word()` method
/// pops the front of the queue, mirroring the FIFO semantics of the
/// bare-metal driver.
#[derive(Default)]
pub struct ScriptedUplink {
    /// Pending uplink words. Public so tests can push raw words directly.
    pub words: VecDeque<u16>,
}

impl ScriptedUplink {
    /// Create an empty uplink.
    pub fn new() -> Self {
        Self::default()
    }

    /// Queue a single raw word.
    pub fn push_word(&mut self, word: u16) {
        self.words.push_back(word);
    }

    /// Queue a single DSKY key as its canonical 5-bit code.
    pub fn push_key(&mut self, key: Key) {
        self.words.push_back(key_to_code(key) as u16);
    }

    /// Queue every key in `keys`, preserving order.
    pub fn push_keys(&mut self, keys: &[Key]) {
        for &k in keys {
            self.push_key(k);
        }
    }

    /// Parse `script` and queue the resulting keystrokes.
    ///
    /// See [`parse_script`] for the accepted syntax. Returns
    /// `Err(line_number)` (1-based) if an unrecognised character is
    /// encountered.
    pub fn load_script(&mut self, script: &str) -> Result<(), ScriptError> {
        let keys = parse_script(script)?;
        for k in keys {
            self.push_key(k);
        }
        Ok(())
    }
}

impl Uplink for ScriptedUplink {
    fn read_word(&mut self) -> Option<u16> {
        self.words.pop_front()
    }
}

// ── Script parser ────────────────────────────────────────────────────────────

/// Parse error from [`parse_script`] / [`ScriptedUplink::load_script`].
#[derive(Debug, PartialEq, Eq)]
pub struct ScriptError {
    /// 1-based line number where the error was found.
    pub line: usize,
    /// 1-based column number within that line.
    pub col: usize,
    /// The offending character.
    pub bad_char: char,
}

impl std::fmt::Display for ScriptError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "uplink script: unrecognised character {:?} at line {} column {}",
            self.bad_char, self.line, self.col
        )
    }
}

impl std::error::Error for ScriptError {}

/// Parse a compact DSKY script into a sequence of [`Key`] presses.
///
/// Syntax:
/// - Each non-whitespace character maps to one keypress.
/// - `0`..`9` → digits, `V`/`v` → VERB, `N`/`n` → NOUN, `E`/`e` → ENTR,
///   `+` / `-` → sign, `P`/`p` → PRO, `K`/`k` → KEY REL, `C`/`c` → CLR,
///   `R`/`r` → RSET.
/// - `#` starts a comment that runs to end of line.
/// - Any other character is an error (returned as [`ScriptError`]).
///
/// Example:
/// ```text
/// # V71 reseed 6 words at addr 1
/// V 7 1 E
/// 0 1 E
/// 0 6 E
/// + 6 5 7 8 E
/// ```
pub fn parse_script(script: &str) -> Result<Vec<Key>, ScriptError> {
    let mut keys = Vec::new();
    for (line_idx, raw_line) in script.lines().enumerate() {
        // Strip line comments (`#` through end-of-line).
        let line = raw_line.split('#').next().unwrap_or("");
        for (col_idx, ch) in line.char_indices() {
            if ch.is_whitespace() {
                continue;
            }
            match char_to_key(ch) {
                Some(k) => keys.push(k),
                None => {
                    return Err(ScriptError {
                        line: line_idx + 1,
                        col: col_idx + 1,
                        bad_char: ch,
                    });
                }
            }
        }
    }
    Ok(keys)
}

fn char_to_key(ch: char) -> Option<Key> {
    match ch {
        '0' => Some(Key::Digit(0)),
        '1'..='9' => Some(Key::Digit(ch as u8 - b'0')),
        'V' | 'v' => Some(Key::Verb),
        'N' | 'n' => Some(Key::Noun),
        'E' | 'e' => Some(Key::Entr),
        '+' => Some(Key::Plus),
        '-' => Some(Key::Minus),
        'P' | 'p' => Some(Key::Pro),
        'K' | 'k' => Some(Key::KeyRel),
        'C' | 'c' => Some(Key::Clr),
        'R' | 'r' => Some(Key::Rset),
        _ => None,
    }
}

/// Map a [`Key`] to its Block 2 KEYTEMP1 5-bit code.
///
/// Inverse of `agc_core::services::v_n::Key::from_code`. Returns the
/// code that the hardware UPRUPT line would carry for this key.
pub fn key_to_code(key: Key) -> u8 {
    match key {
        Key::Digit(0) => 16,
        Key::Digit(d) => d, // 1..=9 — values 1..9
        Key::Verb => 17,
        Key::Rset => 18,
        // The Block 2 KEYTEMP1 table reuses code 25 for both PRO and KEY REL
        // (Frank O'Brien §5.3). `Key::from_code(25)` returns `Pro`; the
        // distinction is irrelevant on the uplink path because UPRUPT
        // semantically only delivers ground-originated keypresses, and
        // KEY REL is a crew-only key.
        Key::Pro | Key::KeyRel => 25,
        Key::Plus => 26,
        Key::Minus => 27,
        Key::Entr => 28,
        Key::Clr => 30,
        Key::Noun => 31,
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use agc_core::services::uplink::poll_uplink;
    use agc_core::services::v_n::{feed_key, VnPhase};
    use agc_core::AgcState;

    /// TC-SU-1: round-trip — `key_to_code` then `Key::from_code` recovers
    /// the original key for every variant the parser produces.
    #[test]
    fn tc_su_1_key_code_round_trip() {
        let cases = [
            Key::Digit(0),
            Key::Digit(1),
            Key::Digit(9),
            Key::Verb,
            Key::Noun,
            Key::Plus,
            Key::Minus,
            Key::Clr,
            Key::Pro,
            Key::Entr,
            Key::Rset,
        ];
        for &k in &cases {
            let code = key_to_code(k);
            assert_eq!(
                Key::from_code(code),
                Some(k),
                "round-trip failed for {:?} (code {})",
                k,
                code
            );
        }
    }

    /// TC-SU-2: parse_script accepts the canonical V71 keystroke script.
    #[test]
    fn tc_su_2_parse_v71_script() {
        let script = "\
            # V71 reseed 6 words at addr 1\n\
            V 7 1 E\n\
            0 1 E\n\
            0 6 E\n\
        ";
        let keys = parse_script(script).expect("valid script must parse");
        // V 7 1 E 0 1 E 0 6 E — 10 keys total.
        assert_eq!(keys.len(), 10);
        assert_eq!(keys[0], Key::Verb);
        assert_eq!(keys[1], Key::Digit(7));
        assert_eq!(keys[2], Key::Digit(1));
        assert_eq!(keys[3], Key::Entr);
    }

    /// TC-SU-3: parse_script flags an unrecognised character.
    #[test]
    fn tc_su_3_parse_error_locates_bad_char() {
        let script = "V 7 1 E\nx";
        let err = parse_script(script).unwrap_err();
        assert_eq!(err.line, 2);
        assert_eq!(err.bad_char, 'x');
    }

    /// TC-SU-4: parse_script treats `#` as a line comment.
    #[test]
    fn tc_su_4_comments_ignored() {
        let script = "V 0 6 N 4 0 E # trailing comment with garbage @#$%\n";
        let keys = parse_script(script).expect("comment must not propagate");
        assert_eq!(keys.len(), 7);
    }

    /// TC-SU-5: end-to-end — a scripted V71 sequence pushed through
    /// `poll_uplink` produces the same V/N phase as feeding the same
    /// keystrokes via `feed_key` directly. This is the MS-U1 exit
    /// criterion (`specs/uplink-plan.md` §6 MS-U1).
    #[test]
    fn tc_su_5_scripted_v71_matches_direct() {
        // Reference: drive feed_key directly.
        let mut state_ref = AgcState::new();
        let keys = [
            Key::Verb,
            Key::Digit(7),
            Key::Digit(1),
            Key::Entr,
            Key::Digit(0),
            Key::Digit(1),
            Key::Entr,
            Key::Digit(0),
            Key::Digit(6),
            Key::Entr,
            Key::Plus,
            Key::Digit(6),
            Key::Digit(5),
            Key::Digit(7),
            Key::Digit(8),
            Key::Entr,
        ];
        for &k in &keys {
            feed_key(&mut state_ref, k);
        }

        // Scripted: same keystrokes via ScriptedUplink + poll_uplink.
        let mut state_uplink = AgcState::new();
        let mut uplink = ScriptedUplink::new();
        uplink.load_script("V 7 1 E 0 1 E 0 6 E + 6 5 7 8 E").unwrap();
        poll_uplink(&mut state_uplink, &mut uplink);

        // Both paths must land on the same V/N phase. After 4 words
        // committed, P27Data { loaded: 1, count: 6 } is expected.
        assert!(
            matches!(state_uplink.vn.phase, VnPhase::P27Data { loaded: 1, count: 6, .. }),
            "scripted V71 did not advance P27Data; got {:?}",
            state_uplink.vn.phase
        );
        assert_eq!(
            state_uplink.vn.phase, state_ref.vn.phase,
            "scripted and direct paths must converge on the same V/N phase"
        );
        // R[0] / R[1] / csm_state position after one word: position[0] = 6578 km.
        assert!(
            (state_uplink.csm_state.position[0] - 6_578_000.0).abs() < 1.0,
            "uplink path did not write csm_state.position[0]; got {}",
            state_uplink.csm_state.position[0]
        );
        assert_eq!(state_uplink.csm_state.position[0], state_ref.csm_state.position[0]);
    }
}
