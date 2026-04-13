//! DSKY display state model for the AGC simulator.
//!
//! `DskyDisplayState` mirrors the AGC's DSPTAB registers and the DSALMOUT
//! lamp word, providing a Rust-native representation that the TUI renderer
//! can consume directly.
//!
//! AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc (DSPTAB layout,
//!             pages 307-315); Comanche055/ERASABLE_ASSIGNMENTS.agc (DSALMOUT = octal 11).

use std::collections::VecDeque;

use agc_core::hal::dsky::Key;

/// Display state of the DSKY panel.
///
/// Every field corresponds directly to a visible element on the physical DSKY.
/// `None` means the field is blank (no segments driven).
///
/// AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc DSPTAB[0..10].
#[derive(Clone, Debug)]
pub struct DskyDisplayState {
    // ── PROG / VERB / NOUN two-digit fields ──────────────────────────────────
    /// Current program (major mode) number, 0-99.  `None` = blank.
    ///
    /// AGC source: DSPTAB+10 (MODREG display).
    pub prog: Option<u8>,

    /// Current verb number, 0-99.  `None` = blank.
    ///
    /// AGC source: DSPTAB[2..3].
    pub verb: Option<u8>,

    /// Current noun number, 0-99.  `None` = blank.
    ///
    /// AGC source: DSPTAB[4..5].
    pub noun: Option<u8>,

    // ── Register rows R1/R2/R3 (signed 5-digit decimal) ──────────────────────
    /// R1 numeric value.  `None` = blank.
    ///
    /// AGC source: DSPTAB[6..7] (R1 high/low relay words).
    pub r1: Option<i32>,

    /// R2 numeric value.  `None` = blank.
    ///
    /// AGC source: DSPTAB[8..9] (R2 high/low relay words).
    pub r2: Option<i32>,

    /// R3 numeric value.  `None` = blank.
    ///
    /// AGC source: DSPTAB[0..1] (R3 high/low relay words).
    pub r3: Option<i32>,

    // ── Warning/status lamps (DSALMOUT, octal 11) ────────────────────────────
    /// PROG ALARM lamp.
    ///
    /// AGC source: DSPTAB+11 bit 9 (PROGLARM in ALARM_AND_ABORT.agc p. 1494).
    pub prog_light: bool,

    /// COMP ACTY (computer activity) lamp.
    ///
    /// AGC source: DSALMOUT bit 12 (set by ACTIVITY routine in T4RUPT_PROGRAM.agc).
    pub comp_acty: bool,

    /// KEY REL lamp.
    ///
    /// AGC source: DSALMOUT bit 5 (Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc).
    pub key_rel: bool,

    /// OPER ERR (operator error) lamp.
    ///
    /// AGC source: DSALMOUT bit 7.
    pub oprerr: bool,

    /// UPLINK ACTY lamp.
    ///
    /// AGC source: DSALMOUT bit 15 (uplink activity).
    pub uplink_acty: bool,

    /// TEMP (IMU temperature caution) lamp.
    ///
    /// AGC source: DSALMOUT bit 4.
    pub temp: bool,

    /// NO ATT (no attitude reference) lamp.
    ///
    /// AGC source: DSPTAB+11 bit 4 (IMU fail / coarse-align condition).
    pub no_att: bool,

    /// GIMBAL LOCK lamp.
    ///
    /// AGC source: DSPTAB+11 bit 6 (middle gimbal angle > 70°).
    pub gimbal_lock: bool,

    /// TRACKER lamp (rendezvous radar or optics issue).
    ///
    /// AGC source: DSPTAB+11 bit 3.
    pub tracker: bool,

    /// RESTART lamp.
    ///
    /// AGC source: DSPTAB+11 bit 8 (GOPROG path sets restart lamp).
    pub restart: bool,

    // ── FRESH START transition flag (agc-sim only) ───────────────────────────
    /// True briefly during `fresh_start()` execution.
    ///
    /// Used by the TUI to blank PROG/VERB/NOUN during the transition.
    pub in_fresh_start: bool,

    /// Last observed REDOCTR value (mirrors `state.redoctr`).
    pub last_restart_count: u16,

    // ── PINBALL verb/noun display state ──────────────────────────────────────
    /// True when VERB/NOUN display should flash (set by load verbs / FLASHON).
    ///
    /// AGC source: PINBALL_GAME_BUTTONS_AND_LIGHTS.agc FLASHON routine.
    pub flash_vn: bool,

    /// OPERATOR ERROR lamp state (distinct from oprerr which is driven by lamp word).
    ///
    /// Set when `char_in` returns `CharResult::Rejected`.
    /// AGC source: CHARALRM / FALTON sets bit 7 of DSALMOUT.
    pub error_light: bool,

    /// KEY RELEASE lamp state (driven by VnState).
    ///
    /// AGC source: RELDSPON sets bit 5 of DSALMOUT.
    pub key_rel_light: bool,

    // ── Key queue (populated by command_dispatch) ────────────────────────────
    /// Non-blocking queue of pending DSKY key codes.
    ///
    /// `command_dispatch` pushes raw `Key` values here; the AGC `DskyIo::read_key`
    /// implementation pops from the front.
    pub(crate) key_queue: VecDeque<Key>,
}

impl DskyDisplayState {
    /// Construct the post-FRESH-START display state.
    ///
    /// All segments blank, all lamps off.  Matches the STARTSB2 path which
    /// zeros all DSPTAB registers and clears DSALMOUT.
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc STARTSB2 (DSPTAB blank loop).
    pub fn blank() -> Self {
        Self {
            prog: None,
            verb: None,
            noun: None,
            r1: None,
            r2: None,
            r3: None,
            prog_light: false,
            comp_acty: false,
            key_rel: false,
            oprerr: false,
            uplink_acty: false,
            temp: false,
            no_att: false,
            gimbal_lock: false,
            tracker: false,
            restart: false,
            in_fresh_start: false,
            last_restart_count: 0,
            flash_vn: false,
            error_light: false,
            key_rel_light: false,
            key_queue: VecDeque::new(),
        }
    }

    /// Push a key into the non-blocking key queue.
    pub fn enqueue_key(&mut self, key: Key) {
        self.key_queue.push_back(key);
    }

    /// Pop the next pending key, or `None` if the queue is empty.
    pub fn dequeue_key(&mut self) -> Option<Key> {
        self.key_queue.pop_front()
    }

    /// Apply a raw DSALMOUT lamp-word write (channel 11).
    ///
    /// Extracts individual lamp bits from the packed word.
    ///
    /// AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc PROGLARM /
    ///             T4RUPT_PROGRAM.agc lamp-word assignments.
    /// Bit assignments (1-indexed per AGC convention):
    ///   bit 4  = TEMP, bit 5 = KEY REL, bit 7 = OPER ERR,
    ///   bit 12 = COMP ACTY, bit 15 = UPLINK ACTY.
    pub fn apply_lamp_word(&mut self, bits: u16) {
        // AGC channel words use 1-based bit numbering; translate to 0-based.
        self.temp = (bits >> 3) & 1 != 0; // bit 4
        self.key_rel = (bits >> 4) & 1 != 0; // bit 5
        self.oprerr = (bits >> 6) & 1 != 0; // bit 7
        self.comp_acty = (bits >> 11) & 1 != 0; // bit 12
        self.uplink_acty = (bits >> 14) & 1 != 0; // bit 15
    }
}

impl Default for DskyDisplayState {
    fn default() -> Self {
        Self::blank()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blank_state_all_off() {
        let state = DskyDisplayState::blank();
        assert!(state.prog.is_none());
        assert!(state.verb.is_none());
        assert!(state.noun.is_none());
        assert!(state.r1.is_none());
        assert!(!state.prog_light);
        assert!(!state.comp_acty);
        assert!(!state.restart);
        assert!(!state.gimbal_lock);
    }

    #[test]
    fn enqueue_dequeue_key() {
        let mut state = DskyDisplayState::blank();
        state.enqueue_key(Key::Verb);
        state.enqueue_key(Key::One);
        assert_eq!(state.dequeue_key(), Some(Key::Verb));
        assert_eq!(state.dequeue_key(), Some(Key::One));
        assert_eq!(state.dequeue_key(), None);
    }

    #[test]
    fn apply_lamp_word_extracts_bits() {
        let mut state = DskyDisplayState::blank();
        // Set bit 5 (KEY REL) and bit 12 (COMP ACTY).
        // Bit 5 = 1 << 4 = 0x0010; bit 12 = 1 << 11 = 0x0800.
        state.apply_lamp_word(0x0810);
        assert!(state.key_rel);
        assert!(state.comp_acty);
        assert!(!state.oprerr);
        assert!(!state.temp);
    }
}
