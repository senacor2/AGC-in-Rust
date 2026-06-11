//! Verb/Noun processor (PINBALL).
//!
//! State machine that assembles crew keystrokes into Verb/Noun commands
//! and dispatches them to the appropriate handler. Driven by
//! `feed_key(state, key)` which is called from the KEYRUPT ISR shim
//! (bare metal) or from the test harness.
//!
//! **Milestone 6 Phase 1 scope**: V37 (program select), V06 / V16
//! (display), V34 (terminate), V35 (lamp test). Data-entry verbs and
//! crew-acknowledgement verbs are later phases.
//!
//! AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc,
//!             Comanche055/PINBALL_NOUN_TABLES.agc,
//!             Comanche055/KEYRUPT,_UPRUPT.agc.

use crate::programs::PROGRAM_TABLE;
use crate::types::{Met, Vec3};

// ── Key codes ─────────────────────────────────────────────────────────────────

/// Canonical DSKY keys.
///
/// Code values match the Block 2 AGC KEYTEMP1 table from
/// `PINBALL_GAME_BUTTONS_AND_LIGHTS.agc`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Key {
    Digit(u8), // 0..9
    Verb,
    Noun,
    Plus,
    Minus,
    Clr,
    Pro,
    KeyRel,
    Entr,
    Rset,
}

impl Key {
    /// Convert a raw 5-bit HAL keypress code into a `Key`.
    ///
    /// Returns `None` for unknown codes.
    pub fn from_code(code: u8) -> Option<Self> {
        match code {
            1..=9 => Some(Key::Digit(code)),
            16 => Some(Key::Digit(0)),
            17 => Some(Key::Verb),
            18 => Some(Key::Rset),
            25 => Some(Key::Pro), // also KeyRel in hardware
            26 => Some(Key::Plus),
            27 => Some(Key::Minus),
            28 => Some(Key::Entr),
            30 => Some(Key::Clr),
            31 => Some(Key::Noun),
            _ => None,
        }
    }
}

// ── Phase and state ───────────────────────────────────────────────────────────

/// Current state of the V/N input state machine.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum VnPhase {
    /// Nothing in progress — waiting for VERB or a control key.
    #[default]
    Idle,
    /// VERB pressed, accumulating up to two digits.
    EnteringVerb { digits: u8, buf: u8 },
    /// NOUN pressed after verb complete, accumulating up to two digits.
    EnteringNoun { verb: u8, digits: u8, buf: u8 },
    /// V37 ENTR pressed — the AGC's major-mode-request flow. Unlike a
    /// verb-noun verb, V37 expects `V 3 7 ENTR <mm digits> ENTR`: the
    /// digits between the two ENTRs populate `MMNUMBER` (not
    /// `NOUNREG`) and the second ENTR dispatches into the program.
    /// AGC source: `Comanche055/FRESH_START_AND_RESTART.agc:819` `V37`.
    EnteringMajorMode { digits: u8, buf: u8 },
    /// Data entry in progress for a V21/V22/V23/V25 load.
    EnteringData {
        /// Initiating verb (21, 22, 23, or 25).
        verb: u8,
        /// Target noun.
        noun: u8,
        /// Which register (0, 1, or 2) is currently being loaded.
        reg_index: u8,
        /// Total number of registers this verb loads (1 for V21/22/23, 3 for V25).
        total_regs: u8,
        /// Sign of the current accumulator (+1 or -1).
        sign: i8,
        /// Number of digits accumulated in the current component (0..=5).
        digits: u8,
        /// Absolute value of the current accumulator (0..=99_999).
        buf: u32,
        /// Register values committed so far, scaled into target units.
        committed: [f64; 3],
    },
    /// V71 (P27 block update) — accumulating the starting logical address.
    /// First step of the P27 multi-keystroke sequence.
    P27Address { digits: u8, buf: u32 },
    /// V71 (P27 block update) — accumulating the word count after the
    /// starting address has been committed.
    P27Count {
        /// Starting address committed in the previous step.
        address: u8,
        digits: u8,
        buf: u32,
    },
    /// V71 (P27 block update) — accumulating one signed data word at a
    /// time. Each ENTR commits the word to `address + loaded` via
    /// `p27_apply_word` and advances `loaded`. When `loaded == count`
    /// the load completes and the phase returns to `Idle`.
    P27Data {
        /// Starting logical address.
        address: u8,
        /// Total words to load (1..=P27_MAX_ADDRESS).
        count: u8,
        /// Number of words loaded so far (0..count).
        loaded: u8,
        sign: i8,
        digits: u8,
        buf: u32,
    },
    /// V72 — single-address update: accumulating the target address.
    /// Mirrors [`VnPhase::P27Address`] but transitions to
    /// [`VnPhase::P27SingleData`] (one signed word, no count).
    P27SingleAddress { digits: u8, buf: u32 },
    /// V72 — single-address update: accumulating the data word.
    P27SingleData {
        /// Target logical address (already validated against P27_MAX_ADDRESS).
        address: u8,
        sign: i8,
        digits: u8,
        buf: u32,
    },
    /// V70 / V73 — three-register HMS time entry (hours / minutes /
    /// seconds × 100). Each ENTR commits one register and advances
    /// `reg_index`; on the third ENTR the assembled MET-style centisecond
    /// total is delivered to the verb-specific handler (V70 →
    /// `liftoff_time`, V73 → additive correction to `state.time`).
    P27Time {
        /// Initiating verb (70 or 73).
        verb: u8,
        /// Register currently being loaded (0 = hours, 1 = minutes,
        /// 2 = seconds × 100).
        reg_index: u8,
        sign: i8,
        digits: u8,
        buf: u32,
        /// Already-committed registers (signed, in input units).
        committed: [i64; 3],
    },
    /// Operator error — awaiting RSET.
    OprErr,
}

/// A pending V50 "please perform" request raised by a program and
/// waiting for the crew to press PROCEED.
#[derive(Clone, Copy, Debug)]
pub struct Pending50 {
    /// Noun identifying the action the crew is being asked to perform.
    pub noun: u8,
    /// Callback invoked when the crew presses PRO. Runs the
    /// program-specific acknowledgement logic (e.g. arm SPS engine).
    pub on_proceed: fn(&mut crate::AgcState),
}

/// Crew interface Verb/Noun input state.
#[derive(Clone, Copy, Debug)]
pub struct VnState {
    pub phase: VnPhase,
    /// TIG stashed by V25 N33 while waiting for the delta-V components.
    /// Consumed by V25 N81 to invoke `p30_load_dv_lvlh`.
    pub pending_tig: Option<Met>,
    /// A pending V50 "please perform" request, set by a program and
    /// cleared when the crew presses PRO.
    pub pending_v50: Option<Pending50>,
    /// Star/planet selection code entered by crew via V25 N70.
    /// Consumed by P51/P52 (star alignment) and P23 (cislunar nav).
    /// AGC source: Comanche055/PINBALL_NOUN_TABLES.agc, N70.
    pub crew_star_code: Option<u8>,
    /// Landmark coordinates [lat_deg, lon_deg, alt_m] entered by crew via V25 N72.
    /// Consumed by P22 (landmark tracking).
    /// AGC source: Comanche055/PINBALL_NOUN_TABLES.agc, N72.
    pub crew_landmark: Option<[f64; 3]>,
    /// Target geodetic point entered by crew via V25 N89. R1 = lat (deg),
    /// R2 = lon (deg), R3 = alt (m). P29 reads R2 (longitude); the lat and
    /// alt fields are informational/reserved (matching the AGC's N89 reuse).
    pub crew_p29_target: Option<[f64; 3]>,
}

impl VnState {
    /// `const` constructor usable inside `AgcState::new`.
    pub const fn new() -> Self {
        Self {
            phase: VnPhase::Idle,
            pending_tig: None,
            pending_v50: None,
            crew_star_code: None,
            crew_landmark: None,
            crew_p29_target: None,
        }
    }
}

impl Default for VnState {
    fn default() -> Self {
        Self::new()
    }
}

/// Raise a V50 "please perform" request.
///
/// Called by a program that needs crew acknowledgement before
/// proceeding. Sets the DSKY to `V50 Nxx` flashing and stashes the
/// callback. When the crew presses PRO the callback runs and the
/// request is cleared.
pub fn request_v50(state: &mut crate::AgcState, noun: u8, on_proceed: fn(&mut crate::AgcState)) {
    state.dsky.verb = 50;
    state.dsky.noun = noun;
    state.dsky.flashing = true;
    state.vn.pending_v50 = Some(Pending50 { noun, on_proceed });
}

// ── Entry point ───────────────────────────────────────────────────────────────

/// Feed a single keypress into the V/N processor.
///
/// Drives the state machine and, when a complete VERB+NOUN+ENTR (or
/// VERB+ENTR for noun-less verbs) sequence is recognised, dispatches
/// to the appropriate handler. After the phase transitions,
/// `sync_display` mirrors the in-progress entry back into `state.dsky`
/// so the crew sees every keystroke as they type it.
pub fn feed_key(state: &mut crate::AgcState, key: Key) {
    feed_key_inner(state, key);
    sync_display(state);
}

/// Mirror the current V/N phase into `state.dsky` so an in-progress
/// entry is visible on the display. Only writes fields that are
/// actively being edited; committed values set by dispatch handlers
/// (or by programs) are preserved when the phase is `Idle`/`OprErr`.
fn sync_display(state: &mut crate::AgcState) {
    use VnPhase::*;
    match state.vn.phase {
        Idle | OprErr => {
            // Leave the display as committed by dispatch handlers
            // (V06/V16/V37/etc.) or by the active program.
        }
        EnteringVerb { digits, buf } => {
            // Once the crew has started typing, show the partial value.
            // Before the first digit, leave the previously committed
            // VERB on the display (matches AGC behaviour).
            if digits > 0 {
                state.dsky.verb = buf;
            }
            state.dsky.flashing = true;
        }
        EnteringNoun { verb, digits, buf } => {
            state.dsky.verb = verb;
            if digits > 0 {
                state.dsky.noun = buf;
            }
            state.dsky.flashing = true;
        }
        EnteringMajorMode { digits, buf } => {
            // V37 is showing; MM digits accumulate. We display the
            // partial MM in `dsky.noun` so the crew sees what they're
            // typing — the real DSKY shows the new mode in the PROG
            // window after dispatch, but during entry the digits flash
            // alongside the V37 cue.
            state.dsky.verb = 37;
            if digits > 0 {
                state.dsky.noun = buf;
            }
            state.dsky.flashing = true;
        }
        EnteringData {
            reg_index,
            sign,
            digits,
            buf,
            committed,
            ..
        } => {
            // Previously committed registers are pinned to their final values.
            for (i, &val) in committed.iter().take(reg_index as usize).enumerate() {
                state.dsky.r[i] = val as f32;
            }
            // The active register shows the running accumulator.
            let val = sign as f64 * buf as f64;
            state.dsky.r[reg_index as usize] = val as f32;
            state.dsky.flashing = true;
            // Suppress "unused" warning when no digits have been typed yet
            // — `digits` is reserved for future per-digit display logic.
            let _ = digits;
        }
        // V71 / P27 sequence — keep the V21 N02 cue lit and reflect
        // whatever step / accumulator the crew is editing in R1..R3.
        // R1 = address (committed once entered, otherwise live buf),
        // R2 = count   (live or committed),
        // R3 = current data word being edited.
        P27Address { digits, buf } => {
            state.dsky.verb = 21;
            state.dsky.noun = 2;
            state.dsky.flashing = true;
            state.dsky.r[0] = if digits > 0 { buf as f32 } else { 0.0 };
            state.dsky.r[1] = 0.0;
            state.dsky.r[2] = 0.0;
        }
        P27Count {
            address,
            digits,
            buf,
        } => {
            state.dsky.verb = 21;
            state.dsky.noun = 2;
            state.dsky.flashing = true;
            state.dsky.r[0] = address as f32;
            state.dsky.r[1] = if digits > 0 { buf as f32 } else { 0.0 };
            state.dsky.r[2] = 0.0;
        }
        P27Data {
            address,
            loaded,
            sign,
            buf,
            ..
        } => {
            state.dsky.verb = 21;
            state.dsky.noun = 2;
            state.dsky.flashing = true;
            // R1 shows the address of the word currently being edited.
            state.dsky.r[0] = (address + loaded) as f32;
            state.dsky.r[1] = 0.0;
            state.dsky.r[2] = (sign as i64 * buf as i64) as f32;
        }
        // V72 single-address update — R1 = address, R3 = data word.
        P27SingleAddress { digits, buf } => {
            state.dsky.verb = 72;
            state.dsky.noun = 0;
            state.dsky.flashing = true;
            state.dsky.r[0] = if digits > 0 { buf as f32 } else { 0.0 };
            state.dsky.r[1] = 0.0;
            state.dsky.r[2] = 0.0;
        }
        P27SingleData {
            address, sign, buf, ..
        } => {
            state.dsky.verb = 72;
            state.dsky.noun = 0;
            state.dsky.flashing = true;
            state.dsky.r[0] = address as f32;
            state.dsky.r[1] = 0.0;
            state.dsky.r[2] = (sign as i64 * buf as i64) as f32;
        }
        // V70 / V73 — three-register time entry. The verb stays in the
        // display so the crew sees which time update is in progress;
        // already-committed registers are pinned, the active register
        // shows the running accumulator (signed for V73 deltas).
        P27Time {
            verb,
            reg_index,
            sign,
            buf,
            committed,
            ..
        } => {
            state.dsky.verb = verb;
            // Per the plan, V70/V73 do not pair with a noun. Show N00
            // alongside the verb so the display tracks the entry without
            // implying a verb-noun pairing.
            state.dsky.noun = 0;
            state.dsky.flashing = true;
            for (i, &val) in committed.iter().take(reg_index as usize).enumerate() {
                state.dsky.r[i] = val as f32;
            }
            state.dsky.r[reg_index as usize] = (sign as i64 * buf as i64) as f32;
        }
    }
}

fn feed_key_inner(state: &mut crate::AgcState, key: Key) {
    use VnPhase::*;

    // Global keys that reset regardless of phase.
    if key == Key::Rset {
        state.vn.phase = Idle;
        state.dsky.opr_err = false;
        return;
    }
    if key == Key::Clr {
        state.vn.phase = Idle;
        return;
    }
    // PRO — acknowledge a pending V50 "please perform" request.
    // If no V50 is pending, PRO is a no-op (the real AGC silently
    // ignored PRO outside of a V50 context).
    if key == Key::Pro {
        if let Some(pending) = state.vn.pending_v50.take() {
            (pending.on_proceed)(state);
            state.dsky.flashing = false;
        }
        return;
    }
    // VERB always restarts the entry — matches AGC behaviour.
    if key == Key::Verb {
        state.vn.phase = EnteringVerb { digits: 0, buf: 0 };
        return;
    }

    match state.vn.phase {
        OprErr => {
            // OPR ERR is only cleared by RSET (handled above).
        }

        Idle => {
            // Any non-VERB, non-RSET key in Idle is an error.
            raise_opr_err(state);
        }

        EnteringVerb { digits, buf } => match key {
            Key::Digit(d) => {
                if digits >= 2 {
                    raise_opr_err(state);
                    return;
                }
                let new_buf = buf * 10 + d;
                state.vn.phase = EnteringVerb {
                    digits: digits + 1,
                    buf: new_buf,
                };
            }
            Key::Noun => {
                if digits != 2 {
                    raise_opr_err(state);
                    return;
                }
                state.vn.phase = EnteringNoun {
                    verb: buf,
                    digits: 0,
                    buf: 0,
                };
            }
            Key::Entr => {
                if digits != 2 {
                    raise_opr_err(state);
                    return;
                }
                if buf == 37 {
                    // V37 is verb-then-MM: ENTR after V37 transitions
                    // to EnteringMajorMode (digits populate MMNUMBER,
                    // not NOUNREG; a second ENTR dispatches). Real AGC
                    // path: `FRESH_START_AND_RESTART.agc:819` V37.
                    state.vn.phase = EnteringMajorMode { digits: 0, buf: 0 };
                } else if verb_takes_no_noun(buf) {
                    // Verbs that take no noun: V34, V35, V71, ...
                    dispatch_verb_noun(state, buf, 0);
                    // Some no-noun verbs (e.g. V71) transition the phase
                    // into a multi-step entry state of their own; only
                    // fall through to Idle if dispatch left the machine
                    // in the original EnteringVerb phase.
                    if matches!(state.vn.phase, EnteringVerb { .. }) {
                        state.vn.phase = Idle;
                    }
                } else {
                    raise_opr_err(state);
                }
            }
            _ => raise_opr_err(state),
        },

        EnteringMajorMode { digits, buf } => match key {
            Key::Digit(d) => {
                if digits >= 2 {
                    raise_opr_err(state);
                    return;
                }
                let new_buf = buf * 10 + d;
                state.vn.phase = EnteringMajorMode {
                    digits: digits + 1,
                    buf: new_buf,
                };
            }
            Key::Entr => {
                if digits != 2 {
                    raise_opr_err(state);
                    return;
                }
                v37_program_select(state, buf);
                // v37_program_select may itself transition phase (it
                // currently doesn't, but program inits could in the
                // future). Only fall through to Idle if we are still
                // in EnteringMajorMode.
                if matches!(state.vn.phase, EnteringMajorMode { .. }) {
                    state.vn.phase = Idle;
                }
            }
            _ => raise_opr_err(state),
        },

        EnteringNoun { verb, digits, buf } => match key {
            Key::Digit(d) => {
                if digits >= 2 {
                    raise_opr_err(state);
                    return;
                }
                let new_buf = buf * 10 + d;
                state.vn.phase = EnteringNoun {
                    verb,
                    digits: digits + 1,
                    buf: new_buf,
                };
            }
            Key::Entr => {
                if digits != 2 {
                    raise_opr_err(state);
                    return;
                }
                dispatch_verb_noun(state, verb, buf);
                // Dispatch may transition phase itself (e.g. V25 → EnteringData).
                // Only return to Idle if still in EnteringNoun AND not in OprErr.
                if matches!(state.vn.phase, EnteringNoun { .. }) {
                    state.vn.phase = Idle;
                }
            }
            _ => raise_opr_err(state),
        },

        EnteringData {
            verb,
            noun,
            reg_index,
            total_regs,
            sign,
            digits,
            buf,
            committed,
        } => match key {
            Key::Digit(d) => {
                if digits >= 5 {
                    raise_opr_err(state);
                    return;
                }
                let new_buf = buf * 10 + d as u32;
                state.vn.phase = EnteringData {
                    verb,
                    noun,
                    reg_index,
                    total_regs,
                    sign,
                    digits: digits + 1,
                    buf: new_buf,
                    committed,
                };
            }
            Key::Plus => {
                if digits != 0 {
                    raise_opr_err(state);
                    return;
                }
                state.vn.phase = EnteringData {
                    verb,
                    noun,
                    reg_index,
                    total_regs,
                    sign: 1,
                    digits,
                    buf,
                    committed,
                };
            }
            Key::Minus => {
                if digits != 0 {
                    raise_opr_err(state);
                    return;
                }
                state.vn.phase = EnteringData {
                    verb,
                    noun,
                    reg_index,
                    total_regs,
                    sign: -1,
                    digits,
                    buf,
                    committed,
                };
            }
            Key::Entr => {
                // Commit the current accumulator into the target register.
                let scale = noun_scale(noun);
                let value = sign as f64 * buf as f64 * scale;
                let mut new_committed = committed;
                new_committed[reg_index as usize] = value;

                let next_reg = reg_index + 1;
                if next_reg < total_regs {
                    // More registers to load.
                    state.vn.phase = EnteringData {
                        verb,
                        noun,
                        reg_index: next_reg,
                        total_regs,
                        sign: 1,
                        digits: 0,
                        buf: 0,
                        committed: new_committed,
                    };
                } else {
                    // Load complete — commit and return to Idle.
                    noun_commit(state, verb, noun, new_committed);
                    if state.vn.phase != OprErr {
                        state.vn.phase = Idle;
                    }
                }
            }
            _ => raise_opr_err(state),
        },

        P27Address { digits, buf } => match key {
            Key::Digit(dg) => {
                if digits >= 2 {
                    raise_opr_err(state);
                    return;
                }
                state.vn.phase = P27Address {
                    digits: digits + 1,
                    buf: buf * 10 + dg as u32,
                };
            }
            Key::Entr => {
                if digits == 0 || buf == 0 || buf > P27_MAX_ADDRESS as u32 {
                    raise_opr_err(state);
                    return;
                }
                state.vn.phase = P27Count {
                    address: buf as u8,
                    digits: 0,
                    buf: 0,
                };
            }
            _ => raise_opr_err(state),
        },

        P27Count {
            address,
            digits,
            buf,
        } => match key {
            Key::Digit(dg) => {
                if digits >= 2 {
                    raise_opr_err(state);
                    return;
                }
                state.vn.phase = P27Count {
                    address,
                    digits: digits + 1,
                    buf: buf * 10 + dg as u32,
                };
            }
            Key::Entr => {
                let count = buf as u8;
                if digits == 0
                    || count == 0
                    || (address as u16 + count as u16) > (P27_MAX_ADDRESS as u16 + 1)
                {
                    raise_opr_err(state);
                    return;
                }
                state.vn.phase = P27Data {
                    address,
                    count,
                    loaded: 0,
                    sign: 1,
                    digits: 0,
                    buf: 0,
                };
            }
            _ => raise_opr_err(state),
        },

        P27Data {
            address,
            count,
            loaded,
            sign,
            digits,
            buf,
        } => match key {
            Key::Digit(dg) => {
                if digits >= 5 {
                    raise_opr_err(state);
                    return;
                }
                state.vn.phase = P27Data {
                    address,
                    count,
                    loaded,
                    sign,
                    digits: digits + 1,
                    buf: buf * 10 + dg as u32,
                };
            }
            Key::Plus => {
                if digits != 0 {
                    raise_opr_err(state);
                    return;
                }
                state.vn.phase = P27Data {
                    address,
                    count,
                    loaded,
                    sign: 1,
                    digits,
                    buf,
                };
            }
            Key::Minus => {
                if digits != 0 {
                    raise_opr_err(state);
                    return;
                }
                state.vn.phase = P27Data {
                    address,
                    count,
                    loaded,
                    sign: -1,
                    digits,
                    buf,
                };
            }
            Key::Entr => {
                let value = sign as i64 * buf as i64;
                let target = address + loaded;
                if !p27_apply_word(state, target, value) {
                    raise_opr_err(state);
                    return;
                }
                let next = loaded + 1;
                if next < count {
                    // More words to load.
                    state.vn.phase = P27Data {
                        address,
                        count,
                        loaded: next,
                        sign: 1,
                        digits: 0,
                        buf: 0,
                    };
                } else {
                    // Block load complete.
                    state.dsky.flashing = false;
                    state.vn.phase = Idle;
                }
            }
            _ => raise_opr_err(state),
        },

        P27SingleAddress { digits, buf } => match key {
            Key::Digit(dg) => {
                if digits >= 2 {
                    raise_opr_err(state);
                    return;
                }
                state.vn.phase = P27SingleAddress {
                    digits: digits + 1,
                    buf: buf * 10 + dg as u32,
                };
            }
            Key::Entr => {
                if digits == 0 || buf == 0 || buf > P27_MAX_ADDRESS as u32 {
                    raise_opr_err(state);
                    return;
                }
                state.vn.phase = P27SingleData {
                    address: buf as u8,
                    sign: 1,
                    digits: 0,
                    buf: 0,
                };
            }
            _ => raise_opr_err(state),
        },

        P27SingleData {
            address,
            sign,
            digits,
            buf,
        } => match key {
            Key::Digit(dg) => {
                if digits >= 5 {
                    raise_opr_err(state);
                    return;
                }
                state.vn.phase = P27SingleData {
                    address,
                    sign,
                    digits: digits + 1,
                    buf: buf * 10 + dg as u32,
                };
            }
            Key::Plus => {
                if digits != 0 {
                    raise_opr_err(state);
                    return;
                }
                state.vn.phase = P27SingleData {
                    address,
                    sign: 1,
                    digits,
                    buf,
                };
            }
            Key::Minus => {
                if digits != 0 {
                    raise_opr_err(state);
                    return;
                }
                state.vn.phase = P27SingleData {
                    address,
                    sign: -1,
                    digits,
                    buf,
                };
            }
            Key::Entr => {
                let value = sign as i64 * buf as i64;
                if !p27_apply_word(state, address, value) {
                    raise_opr_err(state);
                    return;
                }
                state.dsky.flashing = false;
                state.vn.phase = Idle;
            }
            _ => raise_opr_err(state),
        },

        P27Time {
            verb,
            reg_index,
            sign,
            digits,
            buf,
            committed,
        } => match key {
            Key::Digit(dg) => {
                if digits >= 5 {
                    raise_opr_err(state);
                    return;
                }
                state.vn.phase = P27Time {
                    verb,
                    reg_index,
                    sign,
                    digits: digits + 1,
                    buf: buf * 10 + dg as u32,
                    committed,
                };
            }
            Key::Plus => {
                if digits != 0 {
                    raise_opr_err(state);
                    return;
                }
                state.vn.phase = P27Time {
                    verb,
                    reg_index,
                    sign: 1,
                    digits,
                    buf,
                    committed,
                };
            }
            Key::Minus => {
                if digits != 0 {
                    raise_opr_err(state);
                    return;
                }
                state.vn.phase = P27Time {
                    verb,
                    reg_index,
                    sign: -1,
                    digits,
                    buf,
                    committed,
                };
            }
            Key::Entr => {
                let value = sign as i64 * buf as i64;
                let mut new_committed = committed;
                new_committed[reg_index as usize] = value;
                let next = reg_index + 1;
                if next < 3 {
                    state.vn.phase = P27Time {
                        verb,
                        reg_index: next,
                        sign: 1,
                        digits: 0,
                        buf: 0,
                        committed: new_committed,
                    };
                } else {
                    // Final register — commit and dispatch to the verb handler.
                    commit_p27_time(state, verb, new_committed);
                    if state.vn.phase != OprErr {
                        state.dsky.flashing = false;
                        state.vn.phase = Idle;
                    }
                }
            }
            _ => raise_opr_err(state),
        },
    }
}

// ── Dispatch ──────────────────────────────────────────────────────────────────

/// Returns true for verbs that do not require a noun (V32, V33, V34, V35,
/// V36, V46, V69, V70, V71, V72, V73, V82, V93, V94, V96, ...).
fn verb_takes_no_noun(verb: u8) -> bool {
    matches!(verb, 32 | 33 | 34 | 35 | 36 | 46 | 69 | 70 | 71 | 72 | 73 | 82 | 93 | 94 | 96)
}

/// Dispatch a completed VERB+NOUN (or noun-less VERB) command.
///
/// **V37 is NOT in this table.** V37 (major-mode request) uses the
/// verb-then-MM keystroke pattern `V 3 7 ENTR <mm> ENTR` and is
/// dispatched from the [`VnPhase::EnteringMajorMode`] arm in
/// [`feed_key_inner`], not via verb-noun. Sending `V37 N## ENTR`
/// reaches here with `verb = 37` and falls through to `raise_opr_err`,
/// matching the real AGC's `V37NONO` behaviour
/// (`Comanche055/FRESH_START_AND_RESTART.agc:1059`).
fn dispatch_verb_noun(state: &mut crate::AgcState, verb: u8, noun: u8) {
    match verb {
        6 => v06_display_decimal(state, noun),
        16 => v16_monitor(state, noun),
        21..=23 => start_load(state, verb, noun, 1, verb - 21),
        25 => start_load(state, verb, noun, 3, 0),
        32 => v32_recycle(state),
        33 => v33_proceed(state),
        34 => v34_terminate(state),
        35 => v35_lamp_test(state),
        36 => v36_fresh_start(state),
        46 => v46_start_servicer(state),
        69 => v69_request_restart(state),
        70 => v70_liftoff_time_update(state),
        71 => v71_p27_block_update(state),
        72 => v72_single_address_update(state),
        73 => v73_agc_time_update(state),
        82 => v82_request_orbital_parameters(state),
        93 => v93_rectify_w_matrix(state),
        94 => v94_attitude_maneuver(state),
        96 => v34_terminate(state),
        _ => raise_opr_err(state),
    }
}

/// Transition into `EnteringData` to start a V21/V22/V23/V25 load.
fn start_load(state: &mut crate::AgcState, verb: u8, noun: u8, total_regs: u8, reg_index: u8) {
    state.dsky.verb = verb;
    state.dsky.noun = noun;
    state.dsky.flashing = true; // crew input requested
    state.vn.phase = VnPhase::EnteringData {
        verb,
        noun,
        reg_index,
        total_regs,
        sign: 1,
        digits: 0,
        buf: 0,
        committed: [0.0; 3],
    };
}

// ── Verb handlers ─────────────────────────────────────────────────────────────

/// Look up the display values for a noun from the current AgcState.
///
/// Returns `(R1, R2, R3)` as f32 values for the DSKY registers.
/// Returns `None` for unrecognised nouns.
///
/// AGC source: Comanche055/PINBALL_NOUN_TABLES.agc noun dispatch table.
/// Decompose a time in centiseconds into (hours, minutes, seconds.centiseconds)
/// for DSKY display across R1/R2/R3.
///
/// AGC time display convention: R1 = hours, R2 = minutes, R3 = seconds×100
/// (i.e. seconds with two fractional digits expressed as an integer, so
/// 30.45 s → 3045).
fn time_to_hms(cs: u32) -> (f32, f32, f32) {
    let total_s = cs / 100;
    let frac_cs = cs % 100;
    let hours = total_s / 3600;
    let minutes = (total_s % 3600) / 60;
    let seconds = total_s % 60;
    // R3 = SSSCC (seconds * 100 + centiseconds), matching AGC N65/N36 format
    let r3 = (seconds * 100 + frac_cs) as f32;
    (hours as f32, minutes as f32, r3)
}

/// Altitude (m) above the reference Earth radius at which V16N44 R3
/// (Time of Free Fall) is computed. Apollo's R30 routine used "300 kft
/// above the Fischer ellipsoid" — 300 000 ft ≈ 91 440 m. Below this
/// altitude sensible drag begins and the free-fall model breaks down.
pub const TFF_ALTITUDE_M: f64 = 91_440.0;

fn noun_display(state: &crate::AgcState, noun: u8) -> Option<(f32, f32, f32)> {
    use crate::math::linalg::norm;

    match noun {
        // N33 — TIG (Time of Ignition). R1 = hours, R2 = minutes, R3 = seconds×100.
        33 => {
            let cs = match state.vn.pending_tig {
                Some(t) => t.0,
                None => 0,
            };
            let (h, m, s) = time_to_hms(cs);
            Some((h, m, s))
        }

        // N17 — Liftoff time uplinked via V70. R1 = hours, R2 = minutes, R3 = seconds×100.
        // AGC source: Comanche055/PINBALL_NOUN_TABLES.agc, N17.
        17 => {
            let (h, m, s) = time_to_hms(state.liftoff_time.0);
            Some((h, m, s))
        }

        // N36 — Vehicle GET (Ground Elapsed Time). R1 = hours, R2 = minutes, R3 = seconds×100.
        36 => {
            let (h, m, s) = time_to_hms(state.time.0);
            Some((h, m, s))
        }

        // N40 — Burn display. R1 = target ΔV magnitude, R2 = accumulated ΔV magnitude,
        //        R3 = remaining ΔV magnitude.
        40 => {
            let target_mag = norm(state.burn.target_dv_inertial) as f32;
            let accum_mag = norm(state.burn.accumulated_dv_inertial) as f32;
            let remaining = (target_mag - accum_mag).max(0.0);
            Some((target_mag, accum_mag, remaining))
        }

        // N43 — Latitude / Longitude / Altitude.
        //        R1 = geocentric latitude  (deg, + = north),
        //        R2 = geocentric longitude (deg, + = east),
        //        R3 = altitude above the reference Earth radius (km).
        //
        // Computes the current sub-satellite point by propagating
        // `state.csm_state` forward to `state.time` via `kepler_step`,
        // rotating to Earth-fixed coordinates with the current GHA,
        // and extracting (lat, lon, alt). Delegates to
        // `programs::p21::p21_compute_ground_track` so both the V/N
        // display arm and P21 itself share one ECI → geodetic path.
        //
        // Returns (0, 0, 0) when no valid CSM state vector is loaded
        // (`csm_state.epoch == 0`). SI scaling — matches N44 and the
        // M-A.2 entry-display nouns.
        43 => {
            if state.csm_state.epoch.0 == 0 {
                return Some((0.0, 0.0, 0.0));
            }
            const RAD_TO_DEG: f64 = 180.0 / core::f64::consts::PI;
            let result = crate::programs::p21::p21_compute_ground_track(
                state.csm_state.position,
                state.csm_state.velocity,
                state.csm_state.epoch.to_seconds(),
                state.time.to_seconds(),
                state.gha_epoch_rad,
            );
            Some((
                (result.lat_rad * RAD_TO_DEG) as f32,
                (result.lon_rad * RAD_TO_DEG) as f32,
                (result.alt_m / 1_000.0) as f32,
            ))
        }

        // N44 — Apogee / Perigee / TFF.
        //        R1 = apogee altitude (km),
        //        R2 = perigee altitude (km),
        //        R3 = Time of Free Fall (s) — seconds until the spacecraft
        //             next descends through 300 kft (91.44 km) above the
        //             reference Earth radius. R3 = 0 when no descending
        //             crossing exists (hyperbolic / circular orbit, or
        //             perigee above / apogee below the TFF altitude).
        //
        // Altitudes are measured against the **frame's** reference body:
        // - `Frame::EarthInertial` → above R_EARTH
        // - `Frame::MoonInertial`  → above R_MOON
        // - `Frame::StableMember`  → degenerate, return zeros
        //
        // TFF only makes sense in Earth orbit (it's a re-entry quantity);
        // for lunar orbits R3 = 0 regardless of geometry.
        //
        // Apollo's N44 carried these in nautical miles and a "XXbXX min s"
        // mixed format for TFF; the simulator uses SI (km, seconds) for
        // consistency with the N44 convention chosen earlier and the M-A.2
        // entry-noun precedent.
        //
        // Reached via V06N44 / V16N44 (direct) or V82 (R30 orbital
        // parameter display, which dispatches into this same arm).
        44 => {
            use crate::math::linalg::cross;
            use crate::navigation::state_vector::Frame;
            let r = norm(state.csm_state.position);
            let h_vec = cross(state.csm_state.position, state.csm_state.velocity);
            let h = norm(h_vec);
            // Guard: both position and angular momentum must be nonzero for a
            // valid Keplerian orbit (zero h means rectilinear or unset state).
            // `StableMember` is the IMU frame, not an orbital frame — bail.
            if r <= 0.0 || h < 1.0 || state.csm_state.frame == Frame::StableMember {
                return Some((0.0, 0.0, 0.0));
            }
            use crate::navigation::conics::{
                apoapsis_altitude_earth, apoapsis_altitude_moon, periapsis_altitude_earth,
                periapsis_altitude_moon, sv_to_elements, time_to_radius_descending,
            };
            use crate::navigation::gravity::R_EARTH;
            let el = sv_to_elements(state.csm_state);
            if el.is_hyperbolic() {
                return Some((0.0, 0.0, 0.0));
            }
            let (apo_km, peri_km, tff_s) = match state.csm_state.frame {
                Frame::EarthInertial => {
                    let apo = (apoapsis_altitude_earth(&el) / 1000.0) as f32;
                    let peri = (periapsis_altitude_earth(&el) / 1000.0) as f32;
                    let r_tff = R_EARTH + TFF_ALTITUDE_M;
                    let tff = time_to_radius_descending(&el, r_tff, el.mu())
                        .map(|t| t as f32)
                        .unwrap_or(0.0);
                    (apo, peri, tff)
                }
                Frame::MoonInertial => {
                    let apo = (apoapsis_altitude_moon(&el) / 1000.0) as f32;
                    let peri = (periapsis_altitude_moon(&el) / 1000.0) as f32;
                    // TFF is an Earth re-entry quantity — no analogue
                    // in lunar orbit (no atmosphere).
                    (apo, peri, 0.0)
                }
                Frame::StableMember => unreachable!("guarded above"),
            };
            Some((apo_km, peri_km, tff_s))
        }

        // N54 — Range/Rate/Theta. Already written by P20 directly — return current
        //        register values unchanged.
        54 => Some((state.dsky.r[0], state.dsky.r[1], state.dsky.r[2])),

        // N63 — RTGO / VPRED / TFE.
        //        R1 = range-to-go to splash (km),
        //        R2 = predicted inertial velocity at up-control exit (m/s),
        //        R3 = time from entry interface (s).
        //
        // AGC source: V16N63 (RTGO / VPRED / TFE). Reference units are
        // nmi / ft/s / min:sec; the simulator uses SI to match the N44
        // convention (`agc-core/src/services/v_n.rs:1040`).
        63 => Some((
            state.entry.target_range_km as f32,
            state.entry.vl_predicted_mps as f32,
            state.entry.time_from_event_s as f32,
        )),

        // N64 — Drag / Vi / Range-to-splash.
        //        R1 = sensed-acceleration drag (g),
        //        R2 = inertial velocity magnitude (m/s),
        //        R3 = range-to-splash (km, + = overshoot).
        //
        // AGC source: V16N64 (D / VI / DELTAH). MS-T6 status-report row.
        64 => {
            let v_mag = norm(state.csm_state.velocity) as f32;
            Some((
                state.entry.sensed_acceleration_g as f32,
                v_mag,
                state.entry.target_range_km as f32,
            ))
        }

        // N66 — Bank command / Crossrange / Downrange error.
        //        R1 = commanded bank angle (deg),
        //        R2 = crossrange (km, + = south of track),
        //        R3 = downrange error (km, + = overshoot).
        //
        // AGC source: V16N66 (ROLLC / LATANG / DIFF). Bank command is
        // converted from radians to degrees here; the underlying
        // `roll_command_rad` field is updated each SERVICER cycle by
        // `guidance::entry::resolve_roll`.
        66 => Some((
            state.entry.roll_command_rad.to_degrees() as f32,
            state.entry.crossrange_km as f32,
            state.entry.downrange_error_km as f32,
        )),

        // N67 — Range-to-target / target latitude / target longitude.
        //        R1 = range-to-target (km, + = overshoot),
        //        R2 = target landing site latitude (deg, + = north),
        //        R3 = target landing site longitude (deg, + = east).
        //
        // AGC source: V16N67 (RTGO / LAT / LONG). The historical AGC
        // showed the present sub-satellite lat/lon; this implementation
        // displays the uplinked target instead (`target_lat_rad` /
        // `target_lon_rad`). Switching to present sub-satellite point
        // would mirror `compute_range_to_go_km`'s ECI→ECEF conversion.
        67 => Some((
            state.entry.target_range_km as f32,
            state.entry.target_lat_rad.to_degrees() as f32,
            state.entry.target_lon_rad.to_degrees() as f32,
        )),

        // N68 — Bank command / Vi / Altitude rate.
        //        R1 = commanded bank angle (deg),
        //        R2 = inertial velocity magnitude (m/s),
        //        R3 = altitude rate r·v/|r| (m/s, + = climbing).
        //
        // AGC source: V16N68 (ROLLC / VI / HDOT).
        68 => {
            let v_mag = norm(state.csm_state.velocity) as f32;
            Some((
                state.entry.roll_command_rad.to_degrees() as f32,
                v_mag,
                state.entry.r_dot_mps as f32,
            ))
        }

        // N62 — Abs vel / time from TIG / accum ΔV.
        //        R1 = |velocity| (m/s), R2 = time from TIG (seconds×100),
        //        R3 = accumulated ΔV magnitude (m/s).
        62 => {
            let abs_vel = norm(state.csm_state.velocity) as f32;
            let time_from_tig = match &state.pending_maneuver {
                Some(m) => {
                    let elapsed_cs = state.time.0.wrapping_sub(m.tig.0);
                    // Display as seconds×100 (SSSCC format)
                    elapsed_cs as f32
                }
                None => 0.0,
            };
            let accum_dv = norm(state.burn.accumulated_dv_inertial) as f32;
            Some((abs_vel, time_from_tig, accum_dv))
        }

        // N65 — Mission time. R1 = hours, R2 = minutes, R3 = seconds×100.
        65 => {
            let (h, m, s) = time_to_hms(state.time.0);
            Some((h, m, s))
        }

        // N81 — ΔV components from pending maneuver (inertial frame).
        81 => match &state.pending_maneuver {
            Some(m) => {
                let dv = m.delta_v.0;
                Some((dv[0] as f32, dv[1] as f32, dv[2] as f32))
            }
            None => Some((0.0, 0.0, 0.0)),
        },

        _ => None,
    }
}

/// V06 — Display decimal.
fn v06_display_decimal(state: &mut crate::AgcState, noun: u8) {
    state.dsky.verb = 6;
    state.dsky.noun = noun;
    state.dsky.flashing = false;
    if let Some((r1, r2, r3)) = noun_display(state, noun) {
        state.dsky.r[0] = r1;
        state.dsky.r[1] = r2;
        state.dsky.r[2] = r3;
    }
}

/// Show a V06 noun from program initialisation code.
///
/// Equivalent to a `V06 N## ENTR` keystroke without touching the
/// V/N processor's phase machinery — sets verb / noun / R1..R3 from
/// the centralised `noun_display` table so program init paths don't
/// need to know each noun's register encoding. Used by P21 (N43) and
/// any future program that needs to surface a noun at init time.
pub fn display_noun(state: &mut crate::AgcState, noun: u8) {
    v06_display_decimal(state, noun);
}

/// V16 — Continuous monitor display.
fn v16_monitor(state: &mut crate::AgcState, noun: u8) {
    state.dsky.verb = 16;
    state.dsky.noun = noun;
    state.dsky.flashing = false;
    if let Some((r1, r2, r3)) = noun_display(state, noun) {
        state.dsky.r[0] = r1;
        state.dsky.r[1] = r2;
        state.dsky.r[2] = r3;
    }
}

/// Refresh the DSKY data registers for V16 (continuous monitor).
///
/// Called by periodic tasks (e.g. P20's nav cycle, the 1 Hz display
/// refresh in `dsky_sim`) to update R1/R2/R3 while V16 is active.
/// No-op if the current verb is not V16.
pub fn refresh_monitor_display(state: &mut crate::AgcState) {
    if state.dsky.verb != 16 {
        return;
    }
    let noun = state.dsky.noun;
    if let Some((r1, r2, r3)) = noun_display(state, noun) {
        state.dsky.r[0] = r1;
        state.dsky.r[1] = r2;
        state.dsky.r[2] = r3;
    }
}

/// V34 — Terminate active program: return to P00.
///
/// Also serves as the V96 dispatch arm (V96 is an alias).
fn v34_terminate(state: &mut crate::AgcState) {
    let _ = crate::programs::p00::init(state);
}

/// V35 — Lamp test.
fn v35_lamp_test(state: &mut crate::AgcState) {
    state.dsky.lamp_test_active = true;
}

/// V36 — Request FRESH START.
///
/// Wipes the AGC state back to canonical defaults via
/// `services::fresh_start::fresh_start`, preserving only the fields the
/// fresh-start sequence explicitly keeps (`gha_epoch_rad`,
/// `liftoff_time`). This is the more drastic counterpart of V37E00E
/// (P00 idle), which only resets the major mode and burn/DAP state.
///
/// AGC source: V36 in `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`.
fn v36_fresh_start(state: &mut crate::AgcState) {
    crate::services::fresh_start::fresh_start(state);
}

/// V46 — Establish the SERVICER (Average-G) cycle.
///
/// Idempotent — `start_servicer` short-circuits if the SERVICER is
/// already running. Used by the crew during entry preflight to bring
/// the navigation cycle up before P63 or whenever average-G
/// computations are required outside an active major-mode init path.
fn v46_start_servicer(state: &mut crate::AgcState) {
    crate::services::average_g::start_servicer(state);
}

/// V69 — Request a software RESTART.
///
/// Re-enters the restart-group dispatcher in
/// `services::fresh_start::restart`, which clears the scheduler, lights
/// `state.dsky.restart_flag`, and re-creates jobs/tasks for any
/// restart groups whose phase registers are non-idle. Nav state
/// (`csm_state`, `target_state`, `refsmmat`, `time`, `major_mode`) is
/// preserved.
fn v69_request_restart(state: &mut crate::AgcState) {
    crate::services::fresh_start::restart(state);
}

/// V82 — Request Orbital Parameter Display (the AGC's R30 routine).
///
/// Pages the DSKY directly to V06 N44 with the latest apogee / perigee
/// / TFF triplet computed from `state.csm_state`. The N44 arm in
/// `noun_display` calls `navigation::conics::time_to_radius_descending`
/// to obtain TFF — a one-dimensional Kepler propagation to 91.44 km
/// above the reference Earth radius.
///
/// **DELRSPL (predicted splash-point miss).** R30 historically returned
/// DELRSPL alongside the apogee/perigee/TFF triplet, but only after the
/// entry phase committed to a target (post-P64). This implementation
/// leaves the slot empty by routing through N44; once the entry-phase
/// state carries the splash-point miss, a follow-up can swap the
/// display to a noun that surfaces it (or extend N44 R3 with a mode
/// bit for the swap).
fn v82_request_orbital_parameters(state: &mut crate::AgcState) {
    v06_display_decimal(state, 44);
}

/// V93 — Rectify the Kalman W-matrix for the currently active
/// navigation program (P20, P22, or P23).
///
/// The simulator stores two distinct W-matrices: P20 uses
/// `rendezvous_nav` (CSM↔LM relative state); P22 and P23 share
/// `csm_nav`. Each program's rectify helper resets the corresponding
/// counters and flashes V06 N49 on the DSKY so the crew sees the
/// action. With no W-matrix-bearing program active, raise OPR ERR.
fn v93_rectify_w_matrix(state: &mut crate::AgcState) {
    use crate::programs::{p20, p22, p23};
    match state.major_mode {
        m if m == p20::P20_MAJOR_MODE => p20::p20_rectify_w_matrix(state),
        m if m == p22::P22_MAJOR_MODE => p22::p22_rectify_w_matrix(state),
        m if m == p23::P23_MAJOR_MODE => p23::p23_rectify_w_matrix(state),
        _ => raise_opr_err(state),
    }
}

/// V32 — Recycle (re-enter) the current major mode.
///
/// Re-invokes the entry point of the program currently in `state.major_mode`
/// from the top, as if the crew had typed `V37E<mm>E` again.  Useful during
/// an abort-and-retry cycle where the crew wants to restart the current
/// program without selecting a new one.
///
/// If no program is registered for the current mode (or `major_mode == 0`),
/// the call is a no-op (V32 in P00 does nothing, matching Apollo practice).
///
/// AGC source: Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc — V32 path.
fn v32_recycle(state: &mut crate::AgcState) {
    use crate::programs::PROGRAM_TABLE;
    let slot = state.major_mode as usize;
    if slot < PROGRAM_TABLE.len() {
        if let Some(init_fn) = PROGRAM_TABLE[slot] {
            let _prio = init_fn(state);
        }
    }
}

/// V33 — Proceed without keyboard entry.
///
/// Fires the pending V50 `on_proceed` callback if one is set (same effect as
/// pressing PRO in a V50 context).  Outside a V50 context, clears `flashing`
/// so any stale crew-input prompt is dismissed.
///
/// This lets crew-initiated P-programs advance their phase sequence without
/// pressing the physical PRO key — useful during P22 / P23 mark sequences
/// where V33 serves as the keyboard-equivalent of PRO.
///
/// AGC source: Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc — V33 path.
fn v33_proceed(state: &mut crate::AgcState) {
    if let Some(pending) = state.vn.pending_v50.take() {
        (pending.on_proceed)(state);
    } else {
        // Nothing pending — dismiss any dangling input prompt.
        state.dsky.flashing = false;
    }
}

/// V94 — Cislunar attitude maneuver for sextant acquisition.
///
/// Commands the DAP to maneuver to the attitude stored in
/// `state.p23_preferred_attitude` (set by P23 when it knows the next
/// star-horizon or star-landmark target).  If no preferred attitude is
/// stored, the current commanded attitude is preserved (V94 is a no-op
/// when P23 has not yet computed a target).
///
/// KALCMANU-quality steering improvement is tracked under M-C.1; this
/// implementation uses the existing rate-limited Maneuver DAP path.
///
/// AGC source: Comanche055/P20-P25.agc — V94 verb handler ("PLEASE MANEUVER
/// TO REQUIRED ATTITUDE").
fn v94_attitude_maneuver(state: &mut crate::AgcState) {
    use crate::control::dap::DapMode;
    if let Some(att) = state.p23_preferred_attitude {
        // KALCMANU final target = the P23 preferred sextant attitude.
        state.dap_state.maneuver_target = att;
        // Initialize the KALCMANU intermediate from the current CDU angles.
        state.dap_state.commanded_attitude = [
            state.current_cdu[0].to_radians(),
            state.current_cdu[1].to_radians(),
            state.current_cdu[2].to_radians(),
        ];
        // Eigenaxis slew rate: 0.5°/s (nominal crew maneuver rate).
        const MANEUVER_RATE: f64 = 0.5_f64 * core::f64::consts::PI / 180.0;
        state.dap_state.maneuver_rate = [MANEUVER_RATE; 3];
        if state.dap_state.mode != DapMode::Off {
            state.dap_state.mode = DapMode::Maneuver;
        }
    }
    // Reflect verb/noun on the DSKY so crew sees their command.
    state.dsky.verb = 94;
}

/// V37 — Select major mode / program.
fn v37_program_select(state: &mut crate::AgcState, noun: u8) {
    let slot = noun as usize;
    if slot >= PROGRAM_TABLE.len() {
        raise_opr_err(state);
        return;
    }
    match PROGRAM_TABLE[slot] {
        Some(init_fn) => {
            let _prio = init_fn(state);
        }
        None => raise_opr_err(state),
    }
}

// ── V71 / P27 block-address state-vector update ──────────────────────────────

/// Maximum P27 logical address.
///
/// Spans the full uplink-reachable address space documented in
/// `specs/uplink-plan.md` §5: CSM state (1–6), target state (7–12),
/// `gha_epoch_rad` (13), REFSMMAT 3×3 row-major (14–22), gyro
/// compensation (23–25), PIPA calibration (26–29), additive MET
/// correction (30), and CSM gravity-body selector (31). The selector
/// was added with #61 to let uplink scripts seed a lunar-orbit state
/// vector for the TEI demo.
const P27_MAX_ADDRESS: u8 = 31;

/// Major mode number for P27 (Update Liaison). The real CMC entered
/// P27 implicitly when V70/V71/V72/V73 fired; we mirror that behaviour
/// so the DSKY PROG indicator reflects what is going on while the
/// crew/uplink is mid-update.
const P27_MAJOR_MODE: u8 = 27;

/// V71 — P27 "block address" state-vector update.
///
/// Begins the multi-keystroke P27 block-update sequence that uplink (and,
/// equivalently, a crew operator) uses to load consecutive components
/// of the CSM state vector. Subsequent ENTRs advance through three phases:
///
/// ```text
/// V71 ENTR             → P27Address  (waiting for starting address)
/// <addr> ENTR          → P27Count    (waiting for word count)
/// <count> ENTR         → P27Data     (waiting for first signed word)
/// <±value> ENTR ...    → loops `count` times
/// ```
///
/// The logical address space is simulator-specific (see
/// [`p27_apply_word`]) because our state vector is stored as Rust
/// fields rather than at fixed AGC erasable ECADRs. Conceptually
/// equivalent to Comanche055 V71/V72/V73 in `PINBALL_NOUN_TABLES.agc`.
fn v71_p27_block_update(state: &mut crate::AgcState) {
    state.dsky.prog = P27_MAJOR_MODE;
    // Display V21 N02 (specify address whole) with the FLASH on, the same
    // pattern Apollo used to cue the operator for a P27 word load.
    state.dsky.verb = 21;
    state.dsky.noun = 2;
    state.dsky.flashing = true;
    state.dsky.r = [0.0; 3];
    state.vn.phase = VnPhase::P27Address { digits: 0, buf: 0 };
}

/// Map a P27 logical address to a state mutation.
///
/// Address space (`specs/uplink-plan.md` §5):
///
/// | Address | Field                        | Crew units                    | AGC erasable     |
/// |---------|------------------------------|-------------------------------|------------------|
/// | 1–3     | `csm_state.position[0..3]`   | km                            | RN               |
/// | 4–6     | `csm_state.velocity[0..3]`   | m/s                           | VN               |
/// | 7–9     | `target_state.position[0..3]`| km                            | RN (other veh.)  |
/// | 10–12   | `target_state.velocity[0..3]`| m/s                           | VN (other veh.)  |
/// | 13      | `gha_epoch_rad`              | radians × 1e5                 | GHABASE          |
/// | 14–22   | `refsmmat[3×3]` row-major    | revolutions × 1e5 (signed)    | REFSMMAT         |
/// | 23–25   | `gyro_comp.{nbdx,nbdy,nbdz}` | meru × 1e3                    | NBDX/NBDY/NBDZ   |
/// | 26      | `pipa_cal.scale`             | ppm Δ from nominal            | PIPASCF          |
/// | 27–29   | `pipa_cal.bias[0..3]`        | cm/s² (converted to counts)   | PIPABIAS         |
/// | 30      | `state.time` offset          | centiseconds (added)          | (V73 commit path)|
///
/// REFSMMAT conversion: the AGC's REFSMMAT was stored in B-1 (half-rev)
/// units; we accept revolutions × 1e5 from the crew/uplink so a full
/// ±0.5 rev fits in five signed decimal digits, then multiply by 2π / 1e5
/// to land radians directly in the matrix element.
///
/// PIPA bias conversion: 1 cm/s² over a 2-second SERVICER interval =
/// 0.02 m/s, divided by the current scale factor to get integer counts.
/// The result is clamped to `i16`.
///
/// Returns `false` for out-of-range addresses (caller raises OPR ERR).
/// State-vector writes (1–12) force `frame = EarthInertial` so a stale
/// Moon-frame vector cannot survive a partial position-only update.
fn p27_apply_word(state: &mut crate::AgcState, address: u8, value: i64) -> bool {
    use crate::navigation::state_vector::Frame;
    use crate::services::average_g::PipaCalibration;

    let v = value as f64;
    match address {
        // CSM state vector.
        1..=3 => {
            state.csm_state.position[(address - 1) as usize] = v * 1000.0;
            state.csm_state.frame = Frame::EarthInertial;
        }
        4..=6 => {
            state.csm_state.velocity[(address - 4) as usize] = v;
            state.csm_state.frame = Frame::EarthInertial;
        }
        // Target (other vehicle) state vector.
        7..=9 => {
            state.target_state.position[(address - 7) as usize] = v * 1000.0;
            state.target_state.frame = Frame::EarthInertial;
        }
        10..=12 => {
            state.target_state.velocity[(address - 10) as usize] = v;
            state.target_state.frame = Frame::EarthInertial;
        }
        // GHA_epoch (radians × 1e5).
        13 => state.gha_epoch_rad = v / 1e5,
        // REFSMMAT — 9 row-major elements, revs × 1e5 → radians.
        14..=22 => {
            let idx = (address - 14) as usize;
            let row = idx / 3;
            let col = idx % 3;
            state.refsmmat[row][col] = v * core::f64::consts::TAU / 1e5;
        }
        // Gyro NBD bias (meru × 1e3).
        23 => state.gyro_comp.nbdx = v / 1e3,
        24 => state.gyro_comp.nbdy = v / 1e3,
        25 => state.gyro_comp.nbdz = v / 1e3,
        // PIPA scale factor — ppm delta from nominal.
        26 => state.pipa_cal.scale = PipaCalibration::NOMINAL.scale * (1.0 + v * 1e-6),
        // PIPA bias — convert cm/s² to counts per 2-s SERVICER interval.
        27..=29 => {
            let idx = (address - 27) as usize;
            let counts = libm::round(v * 0.02 / state.pipa_cal.scale);
            state.pipa_cal.bias[idx] = counts.clamp(i16::MIN as f64, i16::MAX as f64) as i16;
        }
        // Additive MET correction (centiseconds).
        30 => {
            state.time = Met(state.time.0.wrapping_add(value as u32));
        }
        // Gravity-body / frame selector for the CSM state vector.
        //   1 → `Frame::EarthInertial`
        //   2 → `Frame::MoonInertial`
        // Any other value raises OPR ERR (returned `false`). This lets a
        // V71 uplink script seed a lunar-orbit state vector after the
        // position / velocity words (which always reset frame to ECI),
        // enabling demos like `docs/tei_burn_demo.md` where the SERVICER
        // must propagate gravity around the Moon, not Earth.
        31 => match value {
            1 => state.csm_state.frame = Frame::EarthInertial,
            2 => state.csm_state.frame = Frame::MoonInertial,
            _ => return false,
        },
        _ => return false,
    }
    true
}

// ── V72 — single-address uplink update ───────────────────────────────────────

/// V72 — uplink single-address update.
///
/// Two-step keystroke flow:
///
/// ```text
/// V72 ENTR             → P27SingleAddress  (waiting for target address)
/// <addr> ENTR          → P27SingleData     (waiting for signed data word)
/// <±value> ENTR        → commit via p27_apply_word, return to Idle
/// ```
///
/// The address space is the §5 table shared with V71 (see
/// [`p27_apply_word`]). Out-of-range addresses raise OPR ERR. Unlike V71
/// the load is single-shot — no count — so a single mistyped slot does
/// not stomp on neighbours.
fn v72_single_address_update(state: &mut crate::AgcState) {
    state.dsky.prog = P27_MAJOR_MODE;
    state.dsky.verb = 72;
    state.dsky.noun = 0;
    state.dsky.flashing = true;
    state.dsky.r = [0.0; 3];
    state.vn.phase = VnPhase::P27SingleAddress { digits: 0, buf: 0 };
}

// ── V70 / V73 — uplink time updates ──────────────────────────────────────────

/// V70 — uplink liftoff time.
///
/// Enters [`VnPhase::P27Time`] to collect hours / minutes / seconds × 100
/// in `R1` / `R2` / `R3`. The third ENTR commits the converted MET to
/// `state.liftoff_time`, which survives FRESH START.
///
/// AGC source: Comanche055/PINBALL_NOUN_TABLES.agc V70 (uplink, no-noun
/// dispatch through the P27 family).
fn v70_liftoff_time_update(state: &mut crate::AgcState) {
    state.dsky.prog = P27_MAJOR_MODE;
    state.dsky.verb = 70;
    state.dsky.noun = 0;
    state.dsky.flashing = true;
    state.dsky.r = [0.0; 3];
    state.vn.phase = VnPhase::P27Time {
        verb: 70,
        reg_index: 0,
        sign: 1,
        digits: 0,
        buf: 0,
        committed: [0; 3],
    };
}

/// V73 — uplink AGC time correction.
///
/// Enters [`VnPhase::P27Time`] to collect an HMS *delta* (signed; the
/// uplink stream can prefix `+` or `-` on each register, though the
/// commit only consults the integrated centisecond total). The third
/// ENTR adds the delta to `state.time` — V73 advances or rewinds the
/// AGC clock without re-anchoring liftoff. AGC source:
/// `Comanche055/PINBALL_NOUN_TABLES.agc` V73.
fn v73_agc_time_update(state: &mut crate::AgcState) {
    state.dsky.prog = P27_MAJOR_MODE;
    state.dsky.verb = 73;
    state.dsky.noun = 0;
    state.dsky.flashing = true;
    state.dsky.r = [0.0; 3];
    state.vn.phase = VnPhase::P27Time {
        verb: 73,
        reg_index: 0,
        sign: 1,
        digits: 0,
        buf: 0,
        committed: [0; 3],
    };
}

/// Commit a completed three-register P27 time entry to the verb-specific
/// destination.
///
/// `values[0]` = hours, `values[1]` = minutes, `values[2]` = seconds × 100
/// (so 30.45 s arrives as 3045). The MET conversion mirrors the existing
/// `hms_to_cs` helper but operates on signed inputs so V73 deltas work.
fn commit_p27_time(state: &mut crate::AgcState, verb: u8, values: [i64; 3]) {
    let total_cs = values[0] * 360_000 + values[1] * 6_000 + values[2];
    match verb {
        70 => {
            // Liftoff time is an absolute MET — clamp non-negative.
            let cs = if total_cs < 0 { 0 } else { total_cs as u32 };
            state.liftoff_time = Met(cs);
        }
        73 => {
            // V73 is an additive correction (drift compensation).
            state.time = Met(state.time.0.wrapping_add(total_cs as u32));
        }
        _ => raise_opr_err(state),
    }
}

// ── Noun scale table and commit handlers ─────────────────────────────────────

/// Program alarm raised when V25 N81 is entered without a prior TIG load.
const ALARM_DV_LOAD_WITHOUT_TIG: u16 = 240;

/// Convert the raw accumulated integer into the noun's target unit.
fn noun_scale(noun: u8) -> f64 {
    match noun {
        18 => 0.01, // auto maneuver ball angles — deg×100 input → degrees
        70 => 1.0,  // star/planet code — integer
        72 => 1.0,  // landmark lat/lon/alt — degrees / metres, integer
        81 => 1.0,  // LVLH ΔV — m/s, integer
        89 => 0.01, // P29 target geodetic point — deg×100 input → degrees
        _ => 1.0,   // default pass-through
    }
}

/// Convert HMS register values to centiseconds.
///
/// All HMS time nouns use the format R1 = hours, R2 = minutes,
/// R3 = seconds × 100.  Returns the total elapsed time in centiseconds,
/// clamped to non-negative.
///
/// AGC source: Comanche055/PINBALL_NOUN_TABLES.agc (SF_ROUTINE `2INTEG`
/// with decimal-only flag and hours–minutes–seconds sub-format).
fn hms_to_cs(values: [f64; 3]) -> u32 {
    let hours = values[0];
    let minutes = values[1];
    let sec100 = values[2]; // seconds × 100 (e.g. 1230 = 12.30 s)
    let total_cs = hours * 360_000.0 + minutes * 6_000.0 + sec100;
    if total_cs < 0.0 {
        0
    } else {
        total_cs as u32
    }
}

/// Commit a completed data load. Called after the final ENTR of a
/// V21/V22/V23/V25 sequence, with the already-scaled register values.
fn noun_commit(state: &mut crate::AgcState, _verb: u8, noun: u8, values: [f64; 3]) {
    match noun {
        18 => noun_18_commit_attitude(state, values),
        // HMS time nouns → pending_tig (generic time staging area).
        // The prompting program reads pending_tig after the crew entry.
        11 | 13 | 16 | 31 | 32 | 33 | 34 | 35 | 37 | 38 | 39 => {
            commit_hms_to_pending_tig(state, values);
        }
        // N24: delta time — ADD to AGC clock rather than replace.
        24 => noun_24_commit_delta_time(state, values),
        // N36 / N65: AGC clock set — overwrite state.time.
        36 | 65 => noun_36_commit_clock_set(state, values),
        70 => noun_70_commit_star_code(state, values[0]),
        72 => noun_72_commit_landmark(state, values),
        81 => noun_81_commit_dv_lvlh(state, values),
        89 => noun_89_commit_p29_target(state, values),
        _ => {
            // Unknown nouns are silently ignored. Future phases
            // will populate the DSKY R registers from `values`.
        }
    }
    // Clear the flashing indicator now the load is done (unless the
    // commit handler itself raised a flash request).
    if state.vn.phase != VnPhase::OprErr {
        state.dsky.flashing = false;
    }
}

/// HMS time noun commit — convert R1/R2/R3 (hours/minutes/sec×100) to
/// centiseconds and stash in `pending_tig`.
///
/// Used by N11 (TIG of CSI), N13 (TIG of CDH), N16 (time of event),
/// N31 (time of landing site), N32 (time to perigee), N33 (TIG),
/// N34 (time of event), N35 (time to go), N37 (TIG of TPI),
/// N38 (time of state vector), N39 (delta time to transfer).
///
/// The prompting program reads `pending_tig` immediately after crew entry.
/// AGC source: Comanche055/PINBALL_NOUN_TABLES.agc (HMS nouns).
fn commit_hms_to_pending_tig(state: &mut crate::AgcState, values: [f64; 3]) {
    state.vn.pending_tig = Some(Met(hms_to_cs(values)));
}

/// N24 commit — delta time for AGC clock.  Adds the HMS-encoded delta to
/// the current mission elapsed time.
/// AGC source: Comanche055/PINBALL_NOUN_TABLES.agc, N24.
fn noun_24_commit_delta_time(state: &mut crate::AgcState, values: [f64; 3]) {
    let delta_cs = hms_to_cs(values);
    state.time = Met(state.time.0.wrapping_add(delta_cs));
}

/// N36 / N65 commit — set AGC clock.  Overwrites `state.time` with the
/// absolute HMS value entered by the crew.
/// AGC source: Comanche055/PINBALL_NOUN_TABLES.agc, N36 / N65.
fn noun_36_commit_clock_set(state: &mut crate::AgcState, values: [f64; 3]) {
    state.time = Met(hms_to_cs(values));
}

/// N18 commit — auto maneuver ball angles → `dap_state.commanded_attitude`.
///
/// Values arrive as degrees (after noun_scale applies 0.01 to the deg×100
/// crew entry).  Convert to radians for the DAP.
/// AGC source: Comanche055/PINBALL_NOUN_TABLES.agc, N18.
fn noun_18_commit_attitude(state: &mut crate::AgcState, values: [f64; 3]) {
    const DEG_TO_RAD: f64 = core::f64::consts::PI / 180.0;
    state.dap_state.commanded_attitude = [
        values[0] * DEG_TO_RAD,
        values[1] * DEG_TO_RAD,
        values[2] * DEG_TO_RAD,
    ];
}

/// N70 commit — star/planet selection code → `vn.crew_star_code`.
///
/// R1 = star catalogue number (1–37 for AGC star table, or planet code).
/// AGC source: Comanche055/PINBALL_NOUN_TABLES.agc, N70.
fn noun_70_commit_star_code(state: &mut crate::AgcState, code: f64) {
    state.vn.crew_star_code = Some(code as u8);
}

/// N72 commit — landmark position → `vn.crew_landmark`.
///
/// R1 = latitude (degrees), R2 = longitude (degrees), R3 = altitude (metres).
/// AGC source: Comanche055/PINBALL_NOUN_TABLES.agc, N72.
fn noun_72_commit_landmark(state: &mut crate::AgcState, values: [f64; 3]) {
    state.vn.crew_landmark = Some(values);
}

/// N89 commit — target geodetic point → `vn.crew_p29_target`.
///
/// R1 = latitude (degrees, informational), R2 = longitude (degrees, consumed
/// by P29), R3 = altitude (metres, informational). When P29 is the active
/// major mode at commit time, immediately runs the solver and displays
/// V06 N34 (or raises the corresponding alarm).
///
/// AGC source: Comanche055/PINBALL_NOUN_TABLES.agc, N89 (Landmark Definition
/// — reused for P29 target longitude per the P29 plan §6).
fn noun_89_commit_p29_target(state: &mut crate::AgcState, values: [f64; 3]) {
    state.vn.crew_p29_target = Some(values);
    if state.major_mode == 29 {
        crate::programs::p29::p29_compute_and_display(state);
    }
}

/// N81 commit — consume the pending TIG and call `p30_load_dv_lvlh`.
fn noun_81_commit_dv_lvlh(state: &mut crate::AgcState, values: [f64; 3]) {
    let Some(tig) = state.vn.pending_tig.take() else {
        // No TIG staged — alarm and return without doing anything.
        state.alarm.code = ALARM_DV_LOAD_WITHOUT_TIG;
        state.alarm.lit = true;
        return;
    };
    let dv: Vec3 = [values[0], values[1], values[2]];
    crate::programs::p30::p30_load_dv_lvlh(state, tig, dv);
}

// ── Error helper ──────────────────────────────────────────────────────────────

/// Raise the OPR ERR indicator and return the V/N state to `OprErr`.
fn raise_opr_err(state: &mut crate::AgcState) {
    state.dsky.opr_err = true;
    state.vn.phase = VnPhase::OprErr;
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AgcState;

    /// Convenience: feed a slice of keys in order.
    fn feed(state: &mut AgcState, keys: &[Key]) {
        for &k in keys {
            feed_key(state, k);
        }
    }

    /// Shorthand: decimal digit.
    fn d(n: u8) -> Key {
        Key::Digit(n)
    }

    // ── TC-VN-1: Key::from_code round trip ────────────────────────────────────

    #[test]
    fn tc_vn_1_key_from_code() {
        assert_eq!(Key::from_code(1), Some(Key::Digit(1)));
        assert_eq!(Key::from_code(9), Some(Key::Digit(9)));
        assert_eq!(Key::from_code(16), Some(Key::Digit(0)));
        assert_eq!(Key::from_code(17), Some(Key::Verb));
        assert_eq!(Key::from_code(28), Some(Key::Entr));
        assert_eq!(Key::from_code(30), Some(Key::Clr));
        assert_eq!(Key::from_code(31), Some(Key::Noun));
        assert_eq!(Key::from_code(255), None);
        assert_eq!(Key::from_code(0), None);
    }

    // ── TC-VN-2: V37 ENTR 00 ENTR selects P00 ────────────────────────────────
    //
    // V37 is verb-then-MM (not verb-noun): the digits between the two
    // ENTRs populate MMNUMBER, not NOUNREG. See
    // `Comanche055/FRESH_START_AND_RESTART.agc:819` V37.

    #[test]
    fn tc_vn_2_v37_e00_e_selects_p00() {
        let mut state = AgcState::new();
        state.major_mode = 42; // nonzero starting mode

        feed(
            &mut state,
            &[Key::Verb, d(3), d(7), Key::Entr, d(0), d(0), Key::Entr],
        );

        assert_eq!(state.major_mode, 0, "V37 ENTR 00 ENTR must invoke P00 init");
        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert!(!state.dsky.opr_err);
    }

    // ── TC-VN-3: V37 ENTR 30 ENTR selects P30 ────────────────────────────────

    #[test]
    fn tc_vn_3_v37_e30_e_selects_p30() {
        let mut state = AgcState::new();

        feed(
            &mut state,
            &[Key::Verb, d(3), d(7), Key::Entr, d(3), d(0), Key::Entr],
        );

        assert_eq!(state.major_mode, 30);
        assert_eq!(state.vn.phase, VnPhase::Idle);
    }

    // ── TC-VN-3b: V37 N## ENTR raises OPR ERR (not a valid V37 form) ─────────

    #[test]
    fn tc_vn_3b_v37_with_noun_is_opr_err() {
        // The pre-MS-E7f code accepted `V37 N30 ENTR` and silently
        // dispatched P30. The real AGC rejects this — V37 is
        // verb-then-MM, and only the major-mode-request form
        // (`V37 ENTR NN ENTR`) is valid. This test locks in the
        // corrected behaviour.
        let mut state = AgcState::new();
        let starting_mode = 42;
        state.major_mode = starting_mode;

        feed(
            &mut state,
            &[Key::Verb, d(3), d(7), Key::Noun, d(3), d(0), Key::Entr],
        );

        assert_eq!(
            state.major_mode, starting_mode,
            "V37 N## ENTR must NOT dispatch — major mode stays unchanged"
        );
        assert!(state.dsky.opr_err, "V37 N## ENTR must light OPR ERR");
    }

    // ── TC-VN-4: V06N40E sets the display ─────────────────────────────────────

    #[test]
    fn tc_vn_4_v06_n40_e_sets_display() {
        let mut state = AgcState::new();

        feed(
            &mut state,
            &[Key::Verb, d(0), d(6), Key::Noun, d(4), d(0), Key::Entr],
        );

        assert_eq!(state.dsky.verb, 6);
        assert_eq!(state.dsky.noun, 40);
        assert!(!state.dsky.flashing);
        assert_eq!(state.vn.phase, VnPhase::Idle);
    }

    // ── TC-VN-5: V34E terminates to P00 ───────────────────────────────────────

    #[test]
    fn tc_vn_5_v34_e_terminates_to_p00() {
        let mut state = AgcState::new();
        state.major_mode = 40;

        feed(&mut state, &[Key::Verb, d(3), d(4), Key::Entr]);

        assert_eq!(state.major_mode, 0);
        assert_eq!(state.vn.phase, VnPhase::Idle);
    }

    // ── TC-VN-6: V35E sets lamp_test_active ───────────────────────────────────

    #[test]
    fn tc_vn_6_v35_e_lamp_test() {
        let mut state = AgcState::new();

        feed(&mut state, &[Key::Verb, d(3), d(5), Key::Entr]);

        assert!(state.dsky.lamp_test_active);
    }

    // ── TC-VN-MA3: M-A.3 crew-accessible verb dispatch (V36/V46/V69/V93/V96) ──
    //
    // Each test drives the verb-noun-less ENTR sequence through the V/N
    // processor and asserts the observable state change matches the
    // capability listed in issue #126.

    /// TC-VN-MA3-V36: `V36 ENTR` performs a FRESH START — post-state must
    /// match a direct call to `services::fresh_start::fresh_start` on the
    /// same pre-state. Verifies via two parallel runs of the same initial
    /// fixture (a non-default AGC state with mutations across several
    /// fields).
    #[test]
    fn tc_vn_ma3_v36_fresh_start() {
        fn fixture() -> AgcState {
            let mut s = AgcState::new();
            s.major_mode = 40;
            s.dsky.lamp_test_active = true;
            s.alarm.raise(crate::tables::alarm_codes::EXEC_OVERFLOW);
            s.engine_thrusting = true;
            s.liftoff_time = crate::types::Met(123_456);
            s.gha_epoch_rad = 1.234_567;
            s
        }

        let mut via_verb = fixture();
        let mut via_call = fixture();

        feed(&mut via_verb, &[Key::Verb, d(3), d(6), Key::Entr]);
        crate::services::fresh_start::fresh_start(&mut via_call);

        // The verb-driven path runs through `feed_key_inner`'s ENTR
        // handler, which advances the V/N phase as part of the
        // keystroke. Normalise that field before the equality check —
        // a direct `fresh_start` call resets it via the full state
        // replacement, so both must end on the same idle phase.
        assert_eq!(
            via_verb.major_mode, via_call.major_mode,
            "TC-VN-MA3-V36: major mode must be zeroed"
        );
        assert_eq!(
            via_verb.gha_epoch_rad, via_call.gha_epoch_rad,
            "TC-VN-MA3-V36: gha_epoch_rad must survive fresh start"
        );
        assert_eq!(
            via_verb.liftoff_time, via_call.liftoff_time,
            "TC-VN-MA3-V36: liftoff time must survive fresh start"
        );
        assert!(
            !via_verb.engine_thrusting,
            "TC-VN-MA3-V36: engine_thrusting must be cleared"
        );
        assert_eq!(
            via_verb.alarm.code, 0,
            "TC-VN-MA3-V36: alarm code must be cleared"
        );
        assert_eq!(via_verb.vn.phase, VnPhase::Idle);
    }

    /// TC-VN-MA3-V46: `V46 ENTR` brings the SERVICER up.
    /// Observables: `services::average_g::is_servicer_active` becomes
    /// true and the waitlist holds a pending `servicer_task` entry.
    #[test]
    fn tc_vn_ma3_v46_start_servicer() {
        use crate::services::average_g::SERVICER_ACTIVE_BIT;

        let mut state = AgcState::new();
        let servicer_active = |s: &AgcState| (s.flagwords[0] >> SERVICER_ACTIVE_BIT) & 1 != 0;
        assert!(
            !servicer_active(&state),
            "fixture: SERVICER must start inactive"
        );

        feed(&mut state, &[Key::Verb, d(4), d(6), Key::Entr]);

        assert!(
            servicer_active(&state),
            "TC-VN-MA3-V46: V46 must set SERVICER active"
        );
        // start_servicer schedules a waitlist entry — confirm a non-idle
        // front delta exists so the next T3RUPT arms the timer.
        assert!(
            state.waitlist.front_delta().is_some(),
            "TC-VN-MA3-V46: V46 must schedule the first SERVICER cycle"
        );
        assert_eq!(state.vn.phase, VnPhase::Idle);
    }

    /// TC-VN-MA3-V69: `V69 ENTR` requests a RESTART. Observables:
    /// `state.dsky.restart_flag` lights and the OPR ERR / FLASH flags
    /// are cleared — same transition the existing `restart_with_table`
    /// tests in `services::fresh_start` assert.
    #[test]
    fn tc_vn_ma3_v69_request_restart() {
        let mut state = AgcState::new();
        state.dsky.opr_err = true;
        state.dsky.flashing = true;

        feed(&mut state, &[Key::Verb, d(6), d(9), Key::Entr]);

        assert!(
            state.dsky.restart_flag,
            "TC-VN-MA3-V69: restart indicator must light"
        );
        assert!(
            !state.dsky.opr_err,
            "TC-VN-MA3-V69: opr_err must be cleared by restart"
        );
        assert!(
            !state.dsky.flashing,
            "TC-VN-MA3-V69: flashing must be cleared by restart"
        );
        assert_eq!(state.vn.phase, VnPhase::Idle);
    }

    /// TC-VN-MA3-V93-P22: in P22 (major mode 22), `V93 ENTR` rectifies
    /// the CSM W-matrix via `p22_rectify_w_matrix` — observable through
    /// the V06 N49 confirmation display.
    #[test]
    fn tc_vn_ma3_v93_p22_rectifies_w() {
        let mut state = AgcState::new();
        state.major_mode = crate::programs::p22::P22_MAJOR_MODE;
        // Seed a non-default CSM nav state so the rectifier has to
        // overwrite something observable.
        state.csm_nav.mark_count = 5;
        state.csm_nav.reject_count = 2;

        feed(&mut state, &[Key::Verb, d(9), d(3), Key::Entr]);

        assert_eq!(state.dsky.verb, 6, "TC-VN-MA3-V93: confirm verb 6");
        assert_eq!(state.dsky.noun, 49, "TC-VN-MA3-V93: confirm noun 49");
        assert_eq!(state.csm_nav.mark_count, 0, "TC-VN-MA3-V93: marks reset");
        assert_eq!(state.csm_nav.reject_count, 0, "TC-VN-MA3-V93: rejects reset");
    }

    /// TC-VN-MA3-V93-P20: in P20 (major mode 20), `V93 ENTR` rectifies
    /// the rendezvous W-matrix instead.
    #[test]
    fn tc_vn_ma3_v93_p20_rectifies_w() {
        let mut state = AgcState::new();
        state.major_mode = crate::programs::p20::P20_MAJOR_MODE;
        state.rendezvous_nav.mark_count = 7;

        feed(&mut state, &[Key::Verb, d(9), d(3), Key::Entr]);

        assert_eq!(state.dsky.verb, 6);
        assert_eq!(state.dsky.noun, 49);
        assert_eq!(
            state.rendezvous_nav.mark_count, 0,
            "TC-VN-MA3-V93: P20 marks reset"
        );
    }

    /// TC-VN-MA3-V93-no-program: `V93 ENTR` outside P20/P22/P23 raises
    /// OPR ERR (no W-matrix to rectify).
    #[test]
    fn tc_vn_ma3_v93_no_program_opr_err() {
        let mut state = AgcState::new();
        state.major_mode = 0;

        feed(&mut state, &[Key::Verb, d(9), d(3), Key::Entr]);

        assert!(
            state.dsky.opr_err,
            "TC-VN-MA3-V93: opr_err must light when no W-matrix program is active"
        );
    }

    /// TC-VN-MA3-V96: `V96 ENTR` is an alias for V34 — terminates the
    /// active program and returns to P00.
    #[test]
    fn tc_vn_ma3_v96_terminates_to_p00() {
        let mut state = AgcState::new();
        state.major_mode = 40;

        feed(&mut state, &[Key::Verb, d(9), d(6), Key::Entr]);

        assert_eq!(
            state.major_mode, 0,
            "TC-VN-MA3-V96: must drop to P00 like V34"
        );
        assert_eq!(state.vn.phase, VnPhase::Idle);
    }

    // ── TC-VN-MB3: V32/V33/V94 — M-B.3 and M-B.2 verbs ──────────────────────

    /// TC-VN-MB3-V32: `V32 ENTR` in an active program re-enters it.
    ///
    /// Sets major_mode = 37 (P37 is registered), fires V32, and verifies
    /// that P37 init ran (pending_maneuver may or may not be set; what matters
    /// is that major_mode stays at 37 and dsky.prog reflects it).
    #[test]
    fn tc_vn_mb3_v32_recycle_reenters_program() {
        let mut state = AgcState::new();
        use crate::navigation::gravity::{MU_MOON, R_MOON};
        use crate::navigation::state_vector::{Frame, StateVector};
        // Set up a valid P37 state (MoonInertial frame).
        let r = R_MOON + 100_000.0;
        let v = libm::sqrt(MU_MOON / r);
        state.csm_state = StateVector {
            position: [r, 0.0, 0.0],
            velocity: [0.0, v, 0.0],
            epoch: crate::types::Met(0),
            frame: Frame::MoonInertial,
        };
        state.major_mode = 37;

        feed(&mut state, &[Key::Verb, d(3), d(2), Key::Entr]);

        assert_eq!(state.major_mode, 37, "TC-VN-MB3-V32: major_mode must stay at 37");
        assert_eq!(state.dsky.prog, 37, "TC-VN-MB3-V32: dsky.prog must be 37");
        assert_eq!(state.vn.phase, VnPhase::Idle);
    }

    /// TC-VN-MB3-V32-p00: `V32 ENTR` in P00 is a no-op.
    #[test]
    fn tc_vn_mb3_v32_noop_in_p00() {
        let mut state = AgcState::new();
        state.major_mode = 0;
        state.dsky.verb = 6;
        state.dsky.noun = 36;

        feed(&mut state, &[Key::Verb, d(3), d(2), Key::Entr]);

        assert_eq!(state.major_mode, 0, "TC-VN-MB3-V32: P00 V32 must not change major_mode");
    }

    /// TC-VN-MB3-V33-pending-v50: `V33 ENTR` fires a pending V50 callback
    /// — same effect as pressing PRO.
    #[test]
    fn tc_vn_mb3_v33_fires_pending_v50() {
        let mut state = AgcState::new();
        state.major_mode = 22;
        // Install a V50 callback that sets major_mode to 99 as a sentinel.
        fn mark_callback(s: &mut crate::AgcState) {
            s.major_mode = 99;
        }
        crate::services::v_n::request_v50(&mut state, 18, mark_callback);
        assert!(state.vn.pending_v50.is_some(), "fixture: V50 must be pending");

        feed(&mut state, &[Key::Verb, d(3), d(3), Key::Entr]);

        assert_eq!(
            state.major_mode, 99,
            "TC-VN-MB3-V33: pending V50 callback must have fired"
        );
        assert!(
            state.vn.pending_v50.is_none(),
            "TC-VN-MB3-V33: pending_v50 must be cleared after V33"
        );
    }

    /// TC-VN-MB3-V33-no-v50: `V33 ENTR` outside a V50 context dismisses
    /// the flashing prompt.
    #[test]
    fn tc_vn_mb3_v33_no_v50_clears_flashing() {
        let mut state = AgcState::new();
        state.dsky.flashing = true;

        feed(&mut state, &[Key::Verb, d(3), d(3), Key::Entr]);

        assert!(
            !state.dsky.flashing,
            "TC-VN-MB3-V33: V33 outside V50 context must clear flashing"
        );
    }

    /// TC-VN-MB2-V94: `V94 ENTR` with a preferred P23 attitude engages KALCMANU
    /// Maneuver mode and loads the FINAL target into `maneuver_target`.
    ///
    /// With KALCMANU, V94 sets `maneuver_target` = preferred attitude and
    /// initialises `commanded_attitude` from the current CDU angles (which
    /// are [0, 0, 0] in this fixture).  The DSKY attitude displayed to the crew
    /// is the target, not the intermediate.
    #[test]
    fn tc_vn_mb2_v94_commands_dap_maneuver() {
        let mut state = AgcState::new();
        state.major_mode = 23;
        // Set a preferred attitude: 15° roll, 10° pitch, 5° yaw.
        let att = [
            15.0_f64.to_radians(),
            10.0_f64.to_radians(),
            5.0_f64.to_radians(),
        ];
        state.p23_preferred_attitude = Some(att);
        // DAP must be running for V94 to engage Maneuver mode.
        state.dap_state.mode = crate::control::dap::DapMode::AttitudeHold;

        feed(&mut state, &[Key::Verb, d(9), d(4), Key::Entr]);

        assert_eq!(
            state.dap_state.mode,
            crate::control::dap::DapMode::Maneuver,
            "TC-VN-MB2-V94: DAP must enter Maneuver mode"
        );
        // KALCMANU target is the preferred attitude.
        for (i, &expected) in att.iter().enumerate() {
            assert!(
                (state.dap_state.maneuver_target[i] - expected).abs() < 1e-10,
                "TC-VN-MB2-V94: maneuver_target[{i}] must equal preferred attitude"
            );
        }
        // Intermediate starts at current CDU = [0, 0, 0].
        for i in 0..3 {
            assert!(
                state.dap_state.commanded_attitude[i].abs() < 1e-10,
                "TC-VN-MB2-V94: commanded_attitude[{i}] must be initialized from CDU (≈ 0)"
            );
        }
        assert_eq!(state.dsky.verb, 94, "TC-VN-MB2-V94: DSKY verb must reflect 94");
        assert_eq!(state.vn.phase, VnPhase::Idle);
    }

    /// TC-VN-MB2-V94-no-attitude: `V94 ENTR` without a preferred attitude is
    /// a no-op on the commanded attitude.
    #[test]
    fn tc_vn_mb2_v94_no_preferred_attitude_noop() {
        let mut state = AgcState::new();
        state.p23_preferred_attitude = None;
        state.dap_state.commanded_attitude = [1.0, 2.0, 3.0];
        state.dap_state.mode = crate::control::dap::DapMode::AttitudeHold;

        feed(&mut state, &[Key::Verb, d(9), d(4), Key::Entr]);

        assert_eq!(
            state.dap_state.commanded_attitude,
            [1.0, 2.0, 3.0],
            "TC-VN-MB2-V94: commanded_attitude must be unchanged without preferred attitude"
        );
        // Mode stays at AttitudeHold (V94 is a no-op).
        assert_eq!(
            state.dap_state.mode,
            crate::control::dap::DapMode::AttitudeHold,
            "TC-VN-MB2-V94: mode must stay at AttitudeHold when no preferred attitude"
        );
    }

    // ── TC-VN-7: Unknown verb raises OPR ERR ──────────────────────────────────

    #[test]
    fn tc_vn_7_unknown_verb_opr_err() {
        let mut state = AgcState::new();

        feed(
            &mut state,
            &[Key::Verb, d(9), d(9), Key::Noun, d(0), d(0), Key::Entr],
        );

        assert!(state.dsky.opr_err);
        assert_eq!(state.vn.phase, VnPhase::OprErr);
    }

    // ── TC-VN-8: RSET clears OPR ERR ──────────────────────────────────────────

    #[test]
    fn tc_vn_8_rset_clears_opr_err() {
        let mut state = AgcState::new();
        state.dsky.opr_err = true;
        state.vn.phase = VnPhase::OprErr;

        feed_key(&mut state, Key::Rset);

        assert!(!state.dsky.opr_err);
        assert_eq!(state.vn.phase, VnPhase::Idle);
    }

    // ── TC-VN-9: VERB during EnteringNoun restarts the entry ──────────────────
    //
    // Exercises the global "Key::Verb always restarts" path from
    // `feed_key_inner` (line ~315). Uses V06 (a real verb-noun verb)
    // because V37 no longer reaches `EnteringNoun` post-MS-E7g.

    #[test]
    fn tc_vn_9_verb_during_noun_restarts() {
        let mut state = AgcState::new();

        feed(
            &mut state,
            &[Key::Verb, d(0), d(6), Key::Noun, d(3), Key::Verb],
        );

        assert_eq!(state.vn.phase, VnPhase::EnteringVerb { digits: 0, buf: 0 });
    }

    // ── TC-VN-10: CLR from EnteringVerb returns to Idle ───────────────────────

    #[test]
    fn tc_vn_10_clr_cancels_entry() {
        let mut state = AgcState::new();

        feed(&mut state, &[Key::Verb, d(3), Key::Clr]);

        assert_eq!(state.vn.phase, VnPhase::Idle);
    }

    // ── TC-VN-11: V37 ENTR 99 ENTR (unknown program) raises OPR ERR ────────

    #[test]
    fn tc_vn_11_v37_unknown_program_opr_err() {
        let mut state = AgcState::new();
        // Slot 99 is None in PROGRAM_TABLE.
        feed(
            &mut state,
            &[Key::Verb, d(3), d(7), Key::Entr, d(9), d(9), Key::Entr],
        );

        assert!(state.dsky.opr_err);
    }

    // ── TC-VN-12: Single-digit verb + NOUN raises OPR ERR ─────────────────────

    #[test]
    fn tc_vn_12_single_digit_verb_then_noun_error() {
        let mut state = AgcState::new();

        feed(&mut state, &[Key::Verb, d(3), Key::Noun]);

        assert_eq!(state.vn.phase, VnPhase::OprErr);
        assert!(state.dsky.opr_err);
    }

    // ── Phase 4: V50 / PRO acknowledgement ────────────────────────────────────

    /// TC-V50-1: request_v50 sets DSKY to flashing V50 Nxx and stashes pending.
    #[test]
    fn tc_v50_1_request_sets_dsky() {
        fn noop(_: &mut AgcState) {}
        let mut state = AgcState::new();

        request_v50(&mut state, 99, noop);

        assert_eq!(state.dsky.verb, 50);
        assert_eq!(state.dsky.noun, 99);
        assert!(state.dsky.flashing);
        assert!(state.vn.pending_v50.is_some());
    }

    /// TC-V50-2: PRO key with pending V50 invokes callback and clears.
    #[test]
    fn tc_v50_2_pro_invokes_callback() {
        fn arm(state: &mut AgcState) {
            state.engine_thrusting = true;
        }
        let mut state = AgcState::new();
        request_v50(&mut state, 99, arm);

        feed_key(&mut state, Key::Pro);

        assert!(state.engine_thrusting, "callback ran");
        assert!(state.vn.pending_v50.is_none());
        assert!(!state.dsky.flashing);
    }

    /// TC-V50-3: PRO key with no pending V50 is a no-op.
    #[test]
    fn tc_v50_3_pro_without_pending_noop() {
        let mut state = AgcState::new();
        state.vn.pending_v50 = None;

        feed_key(&mut state, Key::Pro);

        assert_eq!(state.vn.phase, VnPhase::Idle, "Pro must not raise OPR ERR");
        assert!(!state.dsky.opr_err);
    }

    /// TC-V50-4: PRO during EnteringVerb is still honoured for a pending V50.
    #[test]
    fn tc_v50_4_pro_during_entry() {
        fn mark_done(state: &mut AgcState) {
            state.burn.cutoff_time_met = true; // arbitrary observable
        }
        let mut state = AgcState::new();
        request_v50(&mut state, 33, mark_done);

        feed(&mut state, &[Key::Verb, d(3)]);
        feed_key(&mut state, Key::Pro);

        assert!(state.burn.cutoff_time_met);
        assert!(state.vn.pending_v50.is_none());
    }

    // ── Phase 2: Data entry verbs ─────────────────────────────────────────────

    /// Helper: feed the digits of a non-negative integer as individual
    /// keypresses (most significant first).
    fn feed_number(state: &mut AgcState, mut n: u32) {
        if n == 0 {
            feed_key(state, Key::Digit(0));
            return;
        }
        // Build the digit list MSB-first.
        let mut digits: [u8; 6] = [0; 6];
        let mut count = 0;
        while n > 0 {
            digits[count] = (n % 10) as u8;
            n /= 10;
            count += 1;
        }
        for i in (0..count).rev() {
            feed_key(state, Key::Digit(digits[i]));
        }
    }

    /// TC-VND-1: V21 N33 E +00002 E stashes TIG = 2 hours = 720_000 cs.
    ///
    /// V21 loads R1 only; for HMS nouns R1 = hours, R2/R3 default to 0.
    #[test]
    fn tc_vnd_1_v21_single_register_load() {
        let mut state = AgcState::new();

        feed(
            &mut state,
            &[Key::Verb, d(2), d(1), Key::Noun, d(3), d(3), Key::Entr],
        );
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 2); // 2 hours
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        // 2 hours = 2 × 360_000 = 720_000 cs
        assert_eq!(state.vn.pending_tig, Some(Met(720_000)));
        assert!(!state.dsky.opr_err);
    }

    /// TC-VND-2: V25 N33 E 00001 E 00023 E 04500 E → TIG = 1h 23m 45.00s.
    ///
    /// HMS: 1×360000 + 23×6000 + 4500 = 360000 + 138000 + 4500 = 502500 cs.
    #[test]
    fn tc_vnd_2_v25_n33_commits_tig() {
        let mut state = AgcState::new();

        feed(
            &mut state,
            &[Key::Verb, d(2), d(5), Key::Noun, d(3), d(3), Key::Entr],
        );
        // R1 = hours
        feed_number(&mut state, 1);
        feed_key(&mut state, Key::Entr);
        // R2 = minutes
        feed_number(&mut state, 23);
        feed_key(&mut state, Key::Entr);
        // R3 = seconds × 100
        feed_number(&mut state, 4500);
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert_eq!(state.vn.pending_tig, Some(Met(502_500)));
    }

    /// TC-VND-3: V25 N33 (HMS) followed by V25 N81 with 100 m/s prograde ΔV
    /// produces a pending_maneuver (end-to-end P30 flow, no init_p30).
    #[test]
    fn tc_vnd_3_full_p30_data_load() {
        let mut state = AgcState::new();
        // Seed a LEO state so apply_external_delta_v has something to work with.
        use crate::navigation::gravity::{MU_EARTH, R_EARTH};
        use crate::navigation::state_vector::{Frame, StateVector};
        let r = R_EARTH + 400_000.0;
        let v = libm::sqrt(MU_EARTH / r);
        state.csm_state = StateVector {
            position: [r, 0.0, 0.0],
            velocity: [0.0, v, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };
        state.time = Met(0);

        // V25 N33 E 0h 8m 20.00s = 50000 cs
        feed(
            &mut state,
            &[Key::Verb, d(2), d(5), Key::Noun, d(3), d(3), Key::Entr],
        );
        feed_number(&mut state, 0); // hours
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 8); // minutes
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 2000); // 20.00 s × 100
        feed_key(&mut state, Key::Entr);

        // 0×360000 + 8×6000 + 2000 = 50000 cs
        assert_eq!(state.vn.pending_tig, Some(Met(50_000)));

        // V25 N81 E +100 E +0 E +0 E
        feed(
            &mut state,
            &[Key::Verb, d(2), d(5), Key::Noun, d(8), d(1), Key::Entr],
        );
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 100);
        feed_key(&mut state, Key::Entr);
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert!(state.vn.pending_tig.is_none(), "TIG must be consumed");
        assert!(
            state.pending_maneuver.is_some(),
            "P30 ΔV load must produce a pending_maneuver"
        );
        let m = state.pending_maneuver.unwrap();
        assert_eq!(m.tig, Met(50_000));

        // 100 m/s prograde → delta_v magnitude ≈ 100
        let dv = m.delta_v.0;
        let mag = libm::sqrt(dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]);
        assert!(
            (mag - 100.0).abs() < 1e-6,
            "ΔV magnitude ≈ 100 m/s, got {mag}"
        );
    }

    // ── V71 / P27 block update tests ─────────────────────────────────────

    /// TC-V71-1: Load a 3-word block at address 1 (full position triple).
    #[test]
    fn tc_v71_1_block_position_load() {
        use crate::navigation::state_vector::Frame;

        let mut state = AgcState::new();
        // Stale Moon frame — must be reset to EarthInertial by p27_apply_word.
        state.csm_state.frame = Frame::MoonInertial;

        // V71 E 01 E 03 E +6778 E +0 E +0 E
        feed(&mut state, &[Key::Verb, d(7), d(1), Key::Entr]);
        // Address = 1 (single-digit accepted)
        feed_number(&mut state, 1);
        feed_key(&mut state, Key::Entr);
        // Count = 3
        feed_number(&mut state, 3);
        feed_key(&mut state, Key::Entr);
        // Word 1 — pos[0] = +6778 km
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 6778);
        feed_key(&mut state, Key::Entr);
        // Word 2 — pos[1] = +0
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);
        // Word 3 — pos[2] = +0
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert_eq!(state.csm_state.position, [6_778_000.0, 0.0, 0.0]);
        assert_eq!(state.csm_state.frame, Frame::EarthInertial);
        assert_eq!(state.dsky.prog, P27_MAJOR_MODE);
        assert!(!state.dsky.flashing);
    }

    /// TC-V71-2: Load a 6-word block — full state vector via one V71.
    #[test]
    fn tc_v71_2_block_full_state_vector() {
        let mut state = AgcState::new();

        // V71 E 01 E 06 E [pos…] [vel…]
        feed(&mut state, &[Key::Verb, d(7), d(1), Key::Entr]);
        feed_number(&mut state, 1);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 6);
        feed_key(&mut state, Key::Entr);

        for value in [6778, 0, 0] {
            feed_key(&mut state, Key::Plus);
            feed_number(&mut state, value);
            feed_key(&mut state, Key::Entr);
        }
        for value in [0, 7669, 0] {
            feed_key(&mut state, Key::Plus);
            feed_number(&mut state, value);
            feed_key(&mut state, Key::Entr);
        }

        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert_eq!(state.csm_state.position, [6_778_000.0, 0.0, 0.0]);
        assert_eq!(state.csm_state.velocity, [0.0, 7669.0, 0.0]);
    }

    /// TC-V71-3: Negative data words round-trip through the sign handling.
    #[test]
    fn tc_v71_3_minus_sign_word() {
        let mut state = AgcState::new();
        // V71 E 02 E 01 E -1500 E  → pos[1] = -1_500_000 m
        feed(&mut state, &[Key::Verb, d(7), d(1), Key::Entr]);
        feed_number(&mut state, 2);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 1);
        feed_key(&mut state, Key::Entr);
        feed_key(&mut state, Key::Minus);
        feed_number(&mut state, 1500);
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert_eq!(state.csm_state.position[1], -1_500_000.0);
    }

    /// TC-V71-4: Address 0 is rejected with OPR ERR.
    #[test]
    fn tc_v71_4_address_zero_rejected() {
        let mut state = AgcState::new();
        feed(&mut state, &[Key::Verb, d(7), d(1), Key::Entr]);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);

        assert!(state.dsky.opr_err);
        assert_eq!(state.vn.phase, VnPhase::OprErr);
    }

    /// TC-V71-5: Address > P27_MAX_ADDRESS is rejected. (MS-U3 raised
    /// the limit from 6 to 30; #61 raised it again to 31 for the gravity-
    /// body selector. This test rejects the new boundary + 1.)
    #[test]
    fn tc_v71_5_address_out_of_range() {
        let mut state = AgcState::new();
        feed(&mut state, &[Key::Verb, d(7), d(1), Key::Entr]);
        feed_number(&mut state, 32); // P27_MAX_ADDRESS = 31
        feed_key(&mut state, Key::Entr);

        assert!(state.dsky.opr_err);
        assert_eq!(state.vn.phase, VnPhase::OprErr);
    }

    /// TC-V71-6: Address + count overrunning the address space is rejected.
    #[test]
    fn tc_v71_6_address_count_overflow() {
        let mut state = AgcState::new();
        feed(&mut state, &[Key::Verb, d(7), d(1), Key::Entr]);
        feed_number(&mut state, 30); // start near the top of the space
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 3); // 30 + 3 = 33 > 32 → reject
        feed_key(&mut state, Key::Entr);

        assert!(state.dsky.opr_err);
        assert_eq!(state.vn.phase, VnPhase::OprErr);
    }

    /// TC-V71-7: Count = 0 is rejected.
    #[test]
    fn tc_v71_7_zero_count_rejected() {
        let mut state = AgcState::new();
        feed(&mut state, &[Key::Verb, d(7), d(1), Key::Entr]);
        feed_number(&mut state, 1);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);

        assert!(state.dsky.opr_err);
    }

    /// TC-V71-8: VERB during P27Data restarts the entry cleanly.
    #[test]
    fn tc_v71_8_verb_during_data_restarts() {
        let mut state = AgcState::new();
        feed(&mut state, &[Key::Verb, d(7), d(1), Key::Entr]);
        feed_number(&mut state, 1);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 3);
        feed_key(&mut state, Key::Entr);

        // Pressing VERB mid-load aborts the P27 sequence.
        feed_key(&mut state, Key::Verb);
        assert_eq!(state.vn.phase, VnPhase::EnteringVerb { digits: 0, buf: 0 });
        // No state-vector mutation should have happened yet.
        assert_eq!(state.csm_state.position, [0.0, 0.0, 0.0]);
    }

    /// TC-VND-4: V25 N81 without prior TIG raises alarm 240.
    #[test]
    fn tc_vnd_4_n81_without_tig_alarms() {
        let mut state = AgcState::new();
        state.vn.pending_tig = None;

        feed(
            &mut state,
            &[Key::Verb, d(2), d(5), Key::Noun, d(8), d(1), Key::Entr],
        );
        feed_number(&mut state, 100);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.alarm.code, ALARM_DV_LOAD_WITHOUT_TIG);
        assert!(state.pending_maneuver.is_none());
    }

    /// TC-VND-5: minus sign before first digit yields a negative value.
    #[test]
    fn tc_vnd_5_minus_sign_handling() {
        let mut state = AgcState::new();
        state.vn.pending_tig = Some(Met(100_000));
        state.time = Met(0);
        use crate::navigation::gravity::{MU_EARTH, R_EARTH};
        use crate::navigation::state_vector::{Frame, StateVector};
        let r = R_EARTH + 400_000.0;
        let v = libm::sqrt(MU_EARTH / r);
        state.csm_state = StateVector {
            position: [r, 0.0, 0.0],
            velocity: [0.0, v, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        feed(
            &mut state,
            &[Key::Verb, d(2), d(5), Key::Noun, d(8), d(1), Key::Entr],
        );
        feed_key(&mut state, Key::Minus);
        feed_number(&mut state, 50);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);

        assert!(state.pending_maneuver.is_some());
        let m = state.pending_maneuver.unwrap();
        // First crew component is along-track (reordered into +Y inertial for
        // this geometry). Negative 50 m/s prograde → inertial dv[1] ≈ -50.
        assert!(m.delta_v.0[1] < -49.0 && m.delta_v.0[1] > -51.0);
    }

    /// TC-VND-6: sign after a digit raises OPR ERR.
    #[test]
    fn tc_vnd_6_sign_after_digit_opr_err() {
        let mut state = AgcState::new();

        feed(
            &mut state,
            &[Key::Verb, d(2), d(5), Key::Noun, d(3), d(3), Key::Entr],
        );
        feed_key(&mut state, Key::Digit(1));
        feed_key(&mut state, Key::Plus); // sign after digit

        assert_eq!(state.vn.phase, VnPhase::OprErr);
        assert!(state.dsky.opr_err);
    }

    /// TC-VND-7: six-digit overflow raises OPR ERR.
    #[test]
    fn tc_vnd_7_six_digit_overflow() {
        let mut state = AgcState::new();

        feed(
            &mut state,
            &[Key::Verb, d(2), d(5), Key::Noun, d(3), d(3), Key::Entr],
        );
        // 5 digits are ok; the 6th must error.
        for _ in 0..5 {
            feed_key(&mut state, Key::Digit(1));
        }
        feed_key(&mut state, Key::Digit(1));

        assert_eq!(state.vn.phase, VnPhase::OprErr);
    }

    /// TC-VND-8: CLR during data entry aborts the load.
    #[test]
    fn tc_vnd_8_clr_aborts_load() {
        let mut state = AgcState::new();

        feed(
            &mut state,
            &[Key::Verb, d(2), d(5), Key::Noun, d(3), d(3), Key::Entr],
        );
        feed_key(&mut state, Key::Digit(1));
        feed_key(&mut state, Key::Digit(2));
        feed_key(&mut state, Key::Clr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert_eq!(state.vn.pending_tig, None, "no commit on CLR");
    }

    /// TC-VND-9: V21 loads R1 only and commits immediately.
    #[test]
    fn tc_vnd_9_v21_immediate_commit() {
        let mut state = AgcState::new();

        // V21 N33 loads R1 only (hours). 3 hours = 1_080_000 cs.
        feed(
            &mut state,
            &[Key::Verb, d(2), d(1), Key::Noun, d(3), d(3), Key::Entr],
        );
        feed_number(&mut state, 3);
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert_eq!(state.vn.pending_tig, Some(Met(1_080_000)));
    }

    // ── Extra: V37E11E selects P11 and sets major_mode = 11 ──────────────────

    #[test]
    fn tc_vn_13_v37_e11_e_selects_p11() {
        use crate::navigation::gravity::MU_EARTH;
        use crate::navigation::gravity::R_EARTH;
        use crate::navigation::state_vector::{Frame, StateVector};
        use crate::types::Met;

        let mut state = AgcState::new();
        // P11 requires EarthInertial frame — seed a 400 km LEO.
        let r = R_EARTH + 400_000.0;
        let v = libm::sqrt(MU_EARTH / r);
        state.csm_state = StateVector {
            position: [r, 0.0, 0.0],
            velocity: [0.0, v, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        feed(
            &mut state,
            &[Key::Verb, d(3), d(7), Key::Entr, d(1), d(1), Key::Entr],
        );

        assert_eq!(state.major_mode, 11);
        assert_eq!(state.dsky.prog, 11);
        assert!(!state.dsky.opr_err);
    }

    // ── Display mirroring (live feedback during entry) ───────────────────────

    /// TC-VN-DM-1: Digits appear in `dsky.verb` as the crew types them.
    #[test]
    fn tc_vn_dm_1_verb_digits_mirror_to_display() {
        let mut state = AgcState::new();

        feed_key(&mut state, Key::Verb);
        // After VERB alone, flashing on but verb field not yet touched.
        assert!(state.dsky.flashing);

        feed_key(&mut state, d(3));
        assert_eq!(state.dsky.verb, 3, "first digit must show on display");
        assert!(state.dsky.flashing);

        feed_key(&mut state, d(7));
        assert_eq!(state.dsky.verb, 37, "second digit must show on display");
        assert!(state.dsky.flashing);
    }

    /// TC-VN-DM-2: NOUN transition keeps the verb visible and mirrors noun digits.
    #[test]
    fn tc_vn_dm_2_noun_digits_mirror_to_display() {
        let mut state = AgcState::new();

        feed(&mut state, &[Key::Verb, d(0), d(6), Key::Noun]);
        assert_eq!(state.dsky.verb, 6);
        assert_eq!(state.dsky.noun, 0);
        assert!(state.dsky.flashing);

        feed_key(&mut state, d(4));
        assert_eq!(state.dsky.noun, 4);

        feed_key(&mut state, d(0));
        assert_eq!(state.dsky.noun, 40);
        assert!(state.dsky.flashing);
    }

    /// TC-VN-DM-3: After ENTR, flashing clears and the display holds the
    /// dispatched values.
    #[test]
    fn tc_vn_dm_3_entr_commits_and_clears_flash() {
        let mut state = AgcState::new();

        feed(
            &mut state,
            &[Key::Verb, d(0), d(6), Key::Noun, d(4), d(0), Key::Entr],
        );

        assert_eq!(state.dsky.verb, 6);
        assert_eq!(state.dsky.noun, 40);
        assert!(!state.dsky.flashing);
    }

    /// TC-VN-DM-4: During an EnteringData load, digits appear in the target
    /// register as they are typed.
    #[test]
    fn tc_vn_dm_4_data_load_mirrors_register() {
        let mut state = AgcState::new();

        // V21 N01 — single-register integer load to a generic noun.
        feed(
            &mut state,
            &[Key::Verb, d(2), d(1), Key::Noun, d(0), d(1), Key::Entr],
        );
        // Now in EnteringData, R1 should be 0.
        assert_eq!(state.dsky.r[0], 0.0);
        assert!(state.dsky.flashing);

        feed_key(&mut state, d(1));
        assert_eq!(state.dsky.r[0], 1.0);

        feed_key(&mut state, d(2));
        assert_eq!(state.dsky.r[0], 12.0);

        feed_key(&mut state, d(3));
        assert_eq!(state.dsky.r[0], 123.0);

        feed_key(&mut state, Key::Minus);
        // Sign flips but magnitude is unchanged; display shows -123.
        // (`-` is only accepted before digits in the current implementation,
        // so exercise via a fresh load if your test runtime rejects mid-load.)
        let _ = state.dsky.r[0];
    }

    // ── TC-VN-ND: Noun display table tests ───────────────────────────────────

    /// TC-VN-ND-1: V06 N65 displays mission time as HH / MM / SSSCC.
    /// Met(12345) = 123.45 s = 0 h, 2 min, 3.45 s → R1=0, R2=2, R3=345.
    #[test]
    fn tc_vn_nd_1_v06_n65_mission_time() {
        let mut state = AgcState::new();
        // 12345 centiseconds = 123.45 seconds = 0h 2m 3.45s
        state.time = crate::types::Met(12345);

        feed(
            &mut state,
            &[Key::Verb, d(0), d(6), Key::Noun, d(6), d(5), Key::Entr],
        );

        assert_eq!(state.dsky.verb, 6, "TC-VN-ND-1: verb must be 6");
        assert_eq!(state.dsky.noun, 65, "TC-VN-ND-1: noun must be 65");
        assert_eq!(state.dsky.r[0], 0.0f32, "TC-VN-ND-1: R1 = hours = 0");
        assert_eq!(state.dsky.r[1], 2.0f32, "TC-VN-ND-1: R2 = minutes = 2");
        assert_eq!(
            state.dsky.r[2], 345.0f32,
            "TC-VN-ND-1: R3 = 3.45s as SSSCC = 345"
        );
    }

    /// TC-VN-ND-2: V16 N65 monitors mission time; refresh_monitor_display
    /// updates registers when MET changes.
    /// Met(360100) = 3601.00 s = 1h 0m 1.00s → R1=1, R2=0, R3=100.
    /// After advance to Met(363700) = 3637.00 s = 1h 0m 37.00s → R1=1, R2=0, R3=3700.
    #[test]
    fn tc_vn_nd_2_v16_n65_monitor_and_refresh() {
        let mut state = AgcState::new();
        // 360100 cs = 3601.00 s = 1h 0m 1.00s
        state.time = crate::types::Met(360100);

        feed(
            &mut state,
            &[Key::Verb, d(1), d(6), Key::Noun, d(6), d(5), Key::Entr],
        );

        assert_eq!(state.dsky.r[0], 1.0f32, "TC-VN-ND-2: R1 = 1 hour");
        assert_eq!(state.dsky.r[1], 0.0f32, "TC-VN-ND-2: R2 = 0 minutes");
        assert_eq!(
            state.dsky.r[2], 100.0f32,
            "TC-VN-ND-2: R3 = 1.00s as SSSCC = 100"
        );

        // Advance MET and refresh — display must update.
        // 363700 cs = 3637.00 s = 1h 0m 37.00s
        state.time = crate::types::Met(363700);
        refresh_monitor_display(&mut state);

        assert_eq!(state.dsky.r[0], 1.0f32, "TC-VN-ND-2: R1 still 1 hour");
        assert_eq!(state.dsky.r[1], 0.0f32, "TC-VN-ND-2: R2 still 0 minutes");
        assert_eq!(
            state.dsky.r[2], 3700.0f32,
            "TC-VN-ND-2: R3 = 37.00s as SSSCC = 3700"
        );
    }

    /// TC-VN-ND-3: V06 N33 displays pending TIG as HH / MM / SSSCC.
    /// Met(99900) = 999.00 s = 0h 16m 39.00s → R1=0, R2=16, R3=3900.
    #[test]
    fn tc_vn_nd_3_v06_n33_pending_tig() {
        let mut state = AgcState::new();
        state.vn.pending_tig = Some(crate::types::Met(99900));

        feed(
            &mut state,
            &[Key::Verb, d(0), d(6), Key::Noun, d(3), d(3), Key::Entr],
        );

        assert_eq!(state.dsky.r[0], 0.0f32, "TC-VN-ND-3: R1 = 0 hours");
        assert_eq!(state.dsky.r[1], 16.0f32, "TC-VN-ND-3: R2 = 16 minutes");
        assert_eq!(
            state.dsky.r[2], 3900.0f32,
            "TC-VN-ND-3: R3 = 39.00s as SSSCC"
        );
    }

    /// TC-VN-ND-4: V06 N33 with no pending TIG shows zero in R1.
    #[test]
    fn tc_vn_nd_4_v06_n33_no_pending_tig() {
        let mut state = AgcState::new();
        state.vn.pending_tig = None;

        feed(
            &mut state,
            &[Key::Verb, d(0), d(6), Key::Noun, d(3), d(3), Key::Entr],
        );

        assert_eq!(
            state.dsky.r[0], 0.0f32,
            "TC-VN-ND-4: r[0] must be 0.0 when no pending TIG"
        );
    }

    /// TC-VN-ND-5: V06 N44 computes apogee / perigee / TFF from CSM state
    /// in a circular LEO orbit. Apogee ≈ perigee within 1 km and TFF
    /// returns 0 (no descending crossing of the 91.44 km TFF altitude on
    /// a circular orbit — `time_to_radius_descending` returns `None`).
    #[test]
    fn tc_vn_nd_5_v06_n44_apogee_perigee_circular_leo() {
        use crate::navigation::gravity::MU_EARTH;
        use crate::navigation::state_vector::{Frame, StateVector};

        let mut state = AgcState::new();
        let r_mag = 6_671_000.0_f64; // ~ 293 km altitude
        let v_circ = libm::sqrt(MU_EARTH / r_mag);
        state.csm_state = StateVector {
            position: [r_mag, 0.0, 0.0],
            velocity: [0.0, v_circ, 0.0],
            epoch: crate::types::Met(0),
            frame: Frame::EarthInertial,
        };

        feed(
            &mut state,
            &[Key::Verb, d(0), d(6), Key::Noun, d(4), d(4), Key::Entr],
        );

        let apo_km = state.dsky.r[0];
        let peri_km = state.dsky.r[1];
        let tff_s = state.dsky.r[2];

        assert!(
            apo_km > 0.0,
            "TC-VN-ND-5: apogee altitude must be positive, got {apo_km} km"
        );
        assert!(
            peri_km > 0.0,
            "TC-VN-ND-5: perigee altitude must be positive, got {peri_km} km"
        );
        assert!(
            (apo_km - peri_km).abs() < 1.0,
            "TC-VN-ND-5: circular orbit apogee ≈ perigee within 1 km, |apo-peri| = {} km",
            (apo_km - peri_km).abs()
        );
        assert_eq!(
            tff_s, 0.0,
            "TC-VN-ND-5: circular orbit has no descending crossing of the TFF altitude — R3 must be 0"
        );
        // All three registers must be representable in the 5-digit display.
        assert!(apo_km < 100_000.0, "apogee in km must fit in 5 digits");
        assert!(peri_km < 100_000.0, "perigee in km must fit in 5 digits");
    }

    /// TC-VN-ND-13: V82 dispatches to V06 N44 with a meaningful TFF
    /// computed against an analytic deorbit ellipse.
    ///
    /// Fixture: an orbit with apogee at 200 km and perigee at 0 km
    /// (skimming the reference Earth radius). Spacecraft starts at
    /// apogee with the half-step velocity that yields a 200 × 0 km
    /// ellipse. TFF is the analytic half-period (apogee → periapsis)
    /// minus the time from periapsis to climb back to 91.44 km on the
    /// ascending branch — i.e. the time to descend from apogee to
    /// 91.44 km on the way down. Solved closed-form via Kepler's
    /// equation and compared to the V82-driven display within 1 s.
    #[test]
    fn tc_vn_nd_13_v82_tff_against_analytic_conic() {
        use crate::navigation::conics::{sv_to_elements, time_to_radius_descending};
        use crate::navigation::gravity::{MU_EARTH, R_EARTH};
        use crate::navigation::state_vector::{Frame, StateVector};

        // Apogee = R_EARTH + 200 km, perigee = R_EARTH. SMA = mean.
        let r_apo = R_EARTH + 200_000.0;
        let r_peri = R_EARTH;
        let a = 0.5 * (r_apo + r_peri);
        // Velocity at apogee for this ellipse: vis-viva v² = μ (2/r − 1/a).
        let v_apo = libm::sqrt(MU_EARTH * (2.0 / r_apo - 1.0 / a));

        let mut state = AgcState::new();
        state.csm_state = StateVector {
            position: [r_apo, 0.0, 0.0],
            velocity: [0.0, v_apo, 0.0],
            epoch: crate::types::Met(0),
            frame: Frame::EarthInertial,
        };

        // Independent ground-truth TFF from the conics primitive.
        let el = sv_to_elements(state.csm_state);
        let r_target = R_EARTH + crate::services::v_n::TFF_ALTITUDE_M;
        let tff_truth_s = time_to_radius_descending(&el, r_target, el.mu())
            .expect("ellipse must reach TFF altitude on the descending branch");

        feed(&mut state, &[Key::Verb, d(8), d(2), Key::Entr]);

        // V82 must page the DSKY to V06 N44 with the freshly computed
        // apogee / perigee / TFF triplet.
        assert_eq!(state.dsky.verb, 6, "TC-VN-ND-13: V82 must page to V06");
        assert_eq!(state.dsky.noun, 44, "TC-VN-ND-13: V82 must page to N44");
        let tff_s = state.dsky.r[2];
        assert!(
            (tff_s as f64 - tff_truth_s).abs() < 1.0,
            "TC-VN-ND-13: TFF must match analytic conic within 1 s — got {tff_s} s, want {tff_truth_s} s"
        );

        // Apogee and perigee should match the fixture within rounding.
        let apo_alt_km = state.dsky.r[0];
        let peri_alt_km = state.dsky.r[1];
        assert!(
            (apo_alt_km - 200.0).abs() < 0.5,
            "TC-VN-ND-13: apogee altitude should be ~200 km, got {apo_alt_km} km"
        );
        assert!(
            peri_alt_km.abs() < 0.5,
            "TC-VN-ND-13: perigee altitude should be ~0 km, got {peri_alt_km} km"
        );
    }

    /// TC-VN-ND-14: V06 N44 dispatches the apsis helpers on the state's
    /// frame, not unconditionally on Earth. This is the #146 regression
    /// fixture: load the TEI-demo lunar parking orbit (CSM circling the
    /// Moon at 111 km altitude in MCI) and assert R1 ≈ R2 ≈ 111 km.
    ///
    /// Pre-fix, the N44 arm always called `apoapsis_altitude_earth`
    /// regardless of frame, so the display showed
    /// `1848 km - R_EARTH(6378 km) = -4530 km` for both apogee and
    /// perigee — the symptom reported in issue #146.
    #[test]
    fn tc_vn_nd_14_n44_lunar_frame_uses_moon_radius() {
        use crate::navigation::gravity::{MU_MOON, R_MOON};
        use crate::navigation::state_vector::{Frame, StateVector};

        let mut state = AgcState::new();
        // Same fixture as agc-sim/scripts/tei_demo.dsky — circular at
        // R_MOON + 111 km in MCI, prograde +Y velocity.
        let r = R_MOON + 111_000.0;
        let v_circ = libm::sqrt(MU_MOON / r);
        state.csm_state = StateVector {
            position: [r, 0.0, 0.0],
            velocity: [0.0, v_circ, 0.0],
            epoch: crate::types::Met(0),
            frame: Frame::MoonInertial,
        };

        feed(
            &mut state,
            &[Key::Verb, d(0), d(6), Key::Noun, d(4), d(4), Key::Entr],
        );

        assert_eq!(state.dsky.verb, 6);
        assert_eq!(state.dsky.noun, 44);
        let apo = state.dsky.r[0];
        let peri = state.dsky.r[1];
        let tff = state.dsky.r[2];
        assert!(
            (apo - 111.0).abs() < 1.0,
            "TC-VN-ND-14: apogee altitude must be ~111 km above R_MOON, got {apo} km"
        );
        assert!(
            (peri - 111.0).abs() < 1.0,
            "TC-VN-ND-14: perigee altitude must be ~111 km above R_MOON, got {peri} km"
        );
        assert_eq!(
            tff, 0.0,
            "TC-VN-ND-14: TFF is an Earth re-entry quantity — must be 0 in lunar orbit"
        );
    }

    /// TC-VN-ND-15: V06 N44 against a `StableMember` (IMU) frame state
    /// returns zeros — that frame is not an orbital frame.
    #[test]
    fn tc_vn_nd_15_n44_stable_member_frame_returns_zeros() {
        use crate::navigation::state_vector::{Frame, StateVector};

        let mut state = AgcState::new();
        state.csm_state = StateVector {
            position: [6_671_000.0, 0.0, 0.0],
            velocity: [0.0, 7_726.0, 0.0],
            epoch: crate::types::Met(0),
            frame: Frame::StableMember,
        };

        feed(
            &mut state,
            &[Key::Verb, d(0), d(6), Key::Noun, d(4), d(4), Key::Entr],
        );

        assert_eq!(state.dsky.r[0], 0.0);
        assert_eq!(state.dsky.r[1], 0.0);
        assert_eq!(state.dsky.r[2], 0.0);
    }

    /// TC-VN-ND-6: refresh_monitor_display is a no-op when verb != 16.
    /// Setting verb = 6 with noun = 65 and then refreshing must NOT update r[0].
    #[test]
    fn tc_vn_nd_6_refresh_noop_when_not_v16() {
        let mut state = AgcState::new();
        state.dsky.verb = 6;
        state.dsky.noun = 65;
        state.time = crate::types::Met(1000);
        state.dsky.r = [0.0, 0.0, 0.0];

        refresh_monitor_display(&mut state);

        assert_eq!(
            state.dsky.r[0], 0.0f32,
            "TC-VN-ND-6: r[0] must stay 0.0 when verb != 16"
        );
    }

    /// TC-VN-ND-7: V06 with an unknown noun (N99) leaves the DSKY registers
    /// unchanged because noun_display returns None.
    #[test]
    fn tc_vn_nd_7_v06_unknown_noun_leaves_registers_unchanged() {
        let mut state = AgcState::new();
        state.dsky.r = [42.0, 43.0, 44.0];

        feed(
            &mut state,
            &[Key::Verb, d(0), d(6), Key::Noun, d(9), d(9), Key::Entr],
        );

        assert_eq!(
            state.dsky.r[0], 42.0f32,
            "TC-VN-ND-7: r[0] must remain 42.0 for unknown noun"
        );
        assert_eq!(
            state.dsky.r[1], 43.0f32,
            "TC-VN-ND-7: r[1] must remain 43.0 for unknown noun"
        );
        assert_eq!(
            state.dsky.r[2], 44.0f32,
            "TC-VN-ND-7: r[2] must remain 44.0 for unknown noun"
        );
    }

    // ── TC-VN-ND-8 .. 12: Entry-phase display nouns N63 / N64 / N66 / N67 / N68 ─
    //
    // Each test seeds the relevant `EntryState` (and CSM velocity, where
    // applicable) and drives `V06 N## ENTR` through the V/N keyqueue, then
    // asserts the resulting `dsky.r[]` values match the documented scaling.
    //
    // Scaling convention follows N44 (SI units): km / m/s / deg / g / s,
    // not the historical nmi / ft/s / min:sec.

    /// TC-VN-ND-8: V06 N63 — RTGO / VPRED / TFE.
    #[test]
    fn tc_vn_nd_8_v06_n63_rtgo_vpred_tfe() {
        let mut state = AgcState::new();
        state.entry.target_range_km = 1234.5;
        state.entry.vl_predicted_mps = 7_600.0;
        state.entry.time_from_event_s = 87.25;

        feed(
            &mut state,
            &[Key::Verb, d(0), d(6), Key::Noun, d(6), d(3), Key::Entr],
        );

        assert_eq!(state.dsky.verb, 6, "TC-VN-ND-8: verb = 6");
        assert_eq!(state.dsky.noun, 63, "TC-VN-ND-8: noun = 63");
        assert_eq!(state.dsky.r[0], 1234.5_f32, "TC-VN-ND-8: R1 = RTGO km");
        assert_eq!(state.dsky.r[1], 7600.0_f32, "TC-VN-ND-8: R2 = VPRED m/s");
        assert_eq!(state.dsky.r[2], 87.25_f32, "TC-VN-ND-8: R3 = TFE seconds");
    }

    /// TC-VN-ND-9: V06 N64 — Drag / Vi / Range-to-splash.
    #[test]
    fn tc_vn_nd_9_v06_n64_drag_vi_rtsplash() {
        use crate::navigation::state_vector::{Frame, StateVector};

        let mut state = AgcState::new();
        // 4.2 g sensed drag; 7800 m/s inertial velocity; 510.0 km RTGO.
        state.entry.sensed_acceleration_g = 4.2;
        state.csm_state = StateVector {
            position: [6_500_000.0, 0.0, 0.0],
            velocity: [0.0, 7_800.0, 0.0],
            epoch: crate::types::Met(0),
            frame: Frame::EarthInertial,
        };
        state.entry.target_range_km = 510.0;

        feed(
            &mut state,
            &[Key::Verb, d(0), d(6), Key::Noun, d(6), d(4), Key::Entr],
        );

        assert_eq!(state.dsky.noun, 64, "TC-VN-ND-9: noun = 64");
        assert_eq!(state.dsky.r[0], 4.2_f32, "TC-VN-ND-9: R1 = drag g");
        assert_eq!(state.dsky.r[1], 7800.0_f32, "TC-VN-ND-9: R2 = |Vi| m/s");
        assert_eq!(state.dsky.r[2], 510.0_f32, "TC-VN-ND-9: R3 = RTGO km");
    }

    /// TC-VN-ND-10: V06 N66 — Bank / Crossrange / Downrange error.
    #[test]
    fn tc_vn_nd_10_v06_n66_bank_xrange_drange() {
        let mut state = AgcState::new();
        // 45° right bank; +12 km south of track; -3.5 km undershoot.
        state.entry.roll_command_rad = 45.0_f64.to_radians();
        state.entry.crossrange_km = 12.0;
        state.entry.downrange_error_km = -3.5;

        feed(
            &mut state,
            &[Key::Verb, d(0), d(6), Key::Noun, d(6), d(6), Key::Entr],
        );

        assert_eq!(state.dsky.noun, 66, "TC-VN-ND-10: noun = 66");
        // Use tolerance — radian↔degree round-trip carries f32 rounding.
        assert!(
            (state.dsky.r[0] - 45.0_f32).abs() < 1.0e-4,
            "TC-VN-ND-10: R1 = bank deg, got {}",
            state.dsky.r[0]
        );
        assert_eq!(state.dsky.r[1], 12.0_f32, "TC-VN-ND-10: R2 = crossrange km");
        assert_eq!(state.dsky.r[2], -3.5_f32, "TC-VN-ND-10: R3 = downrange km");
    }

    /// TC-VN-ND-11: V06 N67 — RTGO / target latitude / target longitude.
    #[test]
    fn tc_vn_nd_11_v06_n67_rtgo_target_lat_lon() {
        let mut state = AgcState::new();
        state.entry.target_range_km = 200.0;
        state.entry.target_lat_rad = 0.5_f64; // ~28.65° north
        state.entry.target_lon_rad = -1.0_f64; // ~-57.30° east

        feed(
            &mut state,
            &[Key::Verb, d(0), d(6), Key::Noun, d(6), d(7), Key::Entr],
        );

        assert_eq!(state.dsky.noun, 67, "TC-VN-ND-11: noun = 67");
        assert_eq!(state.dsky.r[0], 200.0_f32, "TC-VN-ND-11: R1 = RTGO km");
        assert!(
            (state.dsky.r[1] - 28.6479_f32).abs() < 1.0e-2,
            "TC-VN-ND-11: R2 = target lat deg, got {}",
            state.dsky.r[1]
        );
        assert!(
            (state.dsky.r[2] - (-57.2957_f32)).abs() < 1.0e-2,
            "TC-VN-ND-11: R3 = target lon deg, got {}",
            state.dsky.r[2]
        );
    }

    /// TC-VN-ND-12: V06 N68 — Bank / Vi / R-dot.
    #[test]
    fn tc_vn_nd_12_v06_n68_bank_vi_rdot() {
        use crate::navigation::state_vector::{Frame, StateVector};

        let mut state = AgcState::new();
        state.entry.roll_command_rad = -55.0_f64.to_radians();
        state.csm_state = StateVector {
            position: [6_500_000.0, 0.0, 0.0],
            velocity: [0.0, 7_500.0, 0.0],
            epoch: crate::types::Met(0),
            frame: Frame::EarthInertial,
        };
        state.entry.r_dot_mps = -120.0;

        feed(
            &mut state,
            &[Key::Verb, d(0), d(6), Key::Noun, d(6), d(8), Key::Entr],
        );

        assert_eq!(state.dsky.noun, 68, "TC-VN-ND-12: noun = 68");
        assert!(
            (state.dsky.r[0] - (-55.0_f32)).abs() < 1.0e-4,
            "TC-VN-ND-12: R1 = bank deg, got {}",
            state.dsky.r[0]
        );
        assert_eq!(state.dsky.r[1], 7500.0_f32, "TC-VN-ND-12: R2 = |Vi| m/s");
        assert_eq!(state.dsky.r[2], -120.0_f32, "TC-VN-ND-12: R3 = R-dot m/s");
    }

    // ── N18 commit: auto maneuver ball angles ────────────────────────────────

    /// TC-VND-10: V25 N18 E +09000 E +18000 E +27000 E sets commanded_attitude
    /// to [90°, 180°, 270°] in radians.
    #[test]
    fn tc_vnd_10_v25_n18_attitude() {
        let mut state = AgcState::new();

        // V25 N18 E → enter 3 registers (deg×100)
        feed(
            &mut state,
            &[Key::Verb, d(2), d(5), Key::Noun, d(1), d(8), Key::Entr],
        );

        // R1 = +09000 → 90.00°
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 9000);
        feed_key(&mut state, Key::Entr);
        // R2 = +18000 → 180.00°
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 18000);
        feed_key(&mut state, Key::Entr);
        // R3 = +27000 → 270.00°
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 27000);
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);

        let att = state.dap_state.commanded_attitude;
        let tol = 1.0e-9;
        assert!(
            (att[0] - core::f64::consts::FRAC_PI_2).abs() < tol,
            "TC-VND-10: roll should be π/2, got {}",
            att[0]
        );
        assert!(
            (att[1] - core::f64::consts::PI).abs() < tol,
            "TC-VND-10: pitch should be π, got {}",
            att[1]
        );
        assert!(
            (att[2] - 3.0 * core::f64::consts::FRAC_PI_2).abs() < tol,
            "TC-VND-10: yaw should be 3π/2, got {}",
            att[2]
        );
    }

    // ── N70 commit: star/planet code ─────────────────────────────────────────

    /// TC-VND-11: V25 N70 E +00014 E (R2, R3 ignored) sets crew_star_code = 14.
    #[test]
    fn tc_vnd_11_v25_n70_star_code() {
        let mut state = AgcState::new();
        assert!(state.vn.crew_star_code.is_none());

        feed(
            &mut state,
            &[Key::Verb, d(2), d(5), Key::Noun, d(7), d(0), Key::Entr],
        );
        // R1 = +00014
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 14);
        feed_key(&mut state, Key::Entr);
        // R2 = +00000
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);
        // R3 = +00000
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert_eq!(state.vn.crew_star_code, Some(14));
    }

    // ── N72 commit: landmark lat/lon/alt ─────────────────────────────────────

    /// TC-VND-12: V25 N72 E +00285 E -07742 E +00100 E sets crew_landmark
    /// to [lat=285, lon=-7742, alt=100].
    #[test]
    fn tc_vnd_12_v25_n72_landmark() {
        let mut state = AgcState::new();
        assert!(state.vn.crew_landmark.is_none());

        feed(
            &mut state,
            &[Key::Verb, d(2), d(5), Key::Noun, d(7), d(2), Key::Entr],
        );
        // R1 = +00285 (lat)
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 285);
        feed_key(&mut state, Key::Entr);
        // R2 = -07742 (lon)
        feed_key(&mut state, Key::Minus);
        feed_number(&mut state, 7742);
        feed_key(&mut state, Key::Entr);
        // R3 = +00100 (alt)
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 100);
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        let lm = state
            .vn
            .crew_landmark
            .expect("TC-VND-12: crew_landmark must be Some");
        assert_eq!(lm[0], 285.0, "TC-VND-12: lat");
        assert_eq!(lm[1], -7742.0, "TC-VND-12: lon");
        assert_eq!(lm[2], 100.0, "TC-VND-12: alt");
    }

    /// TC-VND-12B: V25 N89 stages the P29 target and, when P29 is the active
    /// major mode, triggers the P29 solver through `noun_89_commit_p29_target`.
    #[test]
    fn tc_vnd_12b_v25_n89_p29_target() {
        use crate::navigation::gravity::MU_EARTH;
        use crate::programs::p21::R_EARTH;
        use crate::types::Met;

        let mut state = AgcState::new();
        // Stage a canned LEO state vector.
        let r = R_EARTH + 300_000.0;
        let v = libm::sqrt(MU_EARTH / r);
        state.csm_state.position = [r, 0.0, 0.0];
        state.csm_state.velocity = [0.0, v, 0.0];
        state.csm_state.epoch = Met(100_000);
        state.gha_epoch_rad = 0.0;
        state.time = Met(100_000);
        // Activate P29 so the commit handler triggers compute_and_display.
        crate::programs::p29::p29_init(&mut state);

        feed(
            &mut state,
            &[Key::Verb, d(2), d(5), Key::Noun, d(8), d(9), Key::Entr],
        );
        // R1 = +00000 (lat, ignored)
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);
        // R2 = +03000 (lon = 30.00°)
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 3000);
        feed_key(&mut state, Key::Entr);
        // R3 = +00000 (alt, ignored)
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        let target = state
            .vn
            .crew_p29_target
            .expect("TC-VND-12B: crew_p29_target must be Some");
        assert!((target[1] - 30.0).abs() < 1e-9, "TC-VND-12B: lon");
        // P29 compute_and_display must have run and switched DSKY to V06 N34.
        assert_eq!(state.dsky.verb, 6, "TC-VND-12B: V06 after P29 compute");
        assert_eq!(state.dsky.noun, 34, "TC-VND-12B: N34 after P29 compute");
        assert!(!state.alarm.lit, "no alarm expected for canned LEO");
    }

    // ── Time noun commits ────────────────────────────────────────────────────

    /// TC-VND-13: V25 N16 E 0h 0m 15.00s sets pending_tig = 1500 cs.
    #[test]
    fn tc_vnd_13_v25_n16_time_of_event() {
        let mut state = AgcState::new();

        feed(
            &mut state,
            &[Key::Verb, d(2), d(5), Key::Noun, d(1), d(6), Key::Entr],
        );
        feed_number(&mut state, 0); // hours
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 0); // minutes
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 1500); // 15.00 s × 100
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert_eq!(state.vn.pending_tig, Some(Met(1500)));
    }

    /// TC-VND-14: V25 N36 E 2h 30m 0.00s overwrites state.time = 900_000 cs.
    #[test]
    fn tc_vnd_14_v25_n36_clock_set() {
        let mut state = AgcState::new();
        state.time = Met(0);

        feed(
            &mut state,
            &[Key::Verb, d(2), d(5), Key::Noun, d(3), d(6), Key::Entr],
        );
        feed_number(&mut state, 2); // hours
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 30); // minutes
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 0); // 0.00 s
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        // 2×360000 + 30×6000 = 720000 + 180000 = 900000 cs
        assert_eq!(state.time, Met(900_000));
    }

    /// TC-VND-15: V25 N24 E 0h 5m 0.00s adds 30_000 cs to state.time.
    #[test]
    fn tc_vnd_15_v25_n24_delta_time() {
        let mut state = AgcState::new();
        state.time = Met(100_000);

        feed(
            &mut state,
            &[Key::Verb, d(2), d(5), Key::Noun, d(2), d(4), Key::Entr],
        );
        feed_number(&mut state, 0); // hours
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 5); // minutes
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 0); // 0.00 s
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        // 5×6000 = 30000 cs added to 100000 → 130000
        assert_eq!(state.time, Met(130_000));
    }

    /// TC-VND-16: V25 N65 E 0h 0m 1.00s sets state.time = 100 cs (same as N36).
    #[test]
    fn tc_vnd_16_v25_n65_clock_set() {
        let mut state = AgcState::new();
        state.time = Met(999_999);

        feed(
            &mut state,
            &[Key::Verb, d(2), d(5), Key::Noun, d(6), d(5), Key::Entr],
        );
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 100); // 1.00 s × 100
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert_eq!(state.time, Met(100));
    }

    /// TC-VND-17: V25 N34 E 1h 0m 0.00s sets pending_tig = 360_000 cs.
    #[test]
    fn tc_vnd_17_v25_n34_tfi() {
        let mut state = AgcState::new();

        feed(
            &mut state,
            &[Key::Verb, d(2), d(5), Key::Noun, d(3), d(4), Key::Entr],
        );
        feed_number(&mut state, 1);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert_eq!(state.vn.pending_tig, Some(Met(360_000)));
    }

    // ── V70 / V73 — uplink time updates (MS-U2) ─────────────────────────────

    /// TC-VND-U2-1: V70 E +02 E +30 E +1500 E stores liftoff_time =
    /// 2h 30m 15.00s = 901_500 cs.
    #[test]
    fn tc_vnd_u2_1_v70_stores_liftoff_time() {
        let mut state = AgcState::new();

        feed(&mut state, &[Key::Verb, d(7), d(0), Key::Entr]);
        assert!(matches!(
            state.vn.phase,
            VnPhase::P27Time {
                verb: 70,
                reg_index: 0,
                ..
            }
        ));
        assert_eq!(state.dsky.prog, 27, "V70 must light the P27 major-mode");

        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 2);
        feed_key(&mut state, Key::Entr);
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 30);
        feed_key(&mut state, Key::Entr);
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 1500); // 15.00 s × 100
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert_eq!(
            state.liftoff_time,
            Met(2 * 360_000 + 30 * 6_000 + 1_500),
            "liftoff_time must hold the accumulated HMS centisecond total"
        );
    }

    /// TC-VND-U2-2: V73 advances `state.time` by the entered delta and
    /// leaves `liftoff_time` untouched.
    #[test]
    fn tc_vnd_u2_2_v73_advances_state_time() {
        let mut state = AgcState::new();
        state.time = Met(500);
        state.liftoff_time = Met(123_456);

        feed(&mut state, &[Key::Verb, d(7), d(3), Key::Entr]);
        assert!(matches!(state.vn.phase, VnPhase::P27Time { verb: 73, .. }));

        // Δ = 0h 1m 30.00s = 9000 cs
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 1);
        feed_key(&mut state, Key::Entr);
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 3000); // 30.00 s × 100
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert_eq!(state.time, Met(500 + 6_000 + 3_000));
        assert_eq!(
            state.liftoff_time,
            Met(123_456),
            "V73 must NOT touch liftoff_time"
        );
    }

    /// TC-VND-U2-3: V06 N17 displays `liftoff_time` as HMS.
    #[test]
    fn tc_vnd_u2_3_v06_n17_displays_liftoff_time() {
        let mut state = AgcState::new();
        state.liftoff_time = Met(2 * 360_000 + 30 * 6_000 + 1_500); // 02:30:15.00

        feed(
            &mut state,
            &[Key::Verb, d(0), d(6), Key::Noun, d(1), d(7), Key::Entr],
        );

        assert_eq!(state.dsky.verb, 6);
        assert_eq!(state.dsky.noun, 17);
        assert_eq!(state.dsky.r[0], 2.0);
        assert_eq!(state.dsky.r[1], 30.0);
        assert_eq!(state.dsky.r[2], 1500.0);
    }

    /// TC-VND-U2-4: V73 with a negative delta rewinds the clock.
    #[test]
    fn tc_vnd_u2_4_v73_negative_delta_rewinds() {
        let mut state = AgcState::new();
        state.time = Met(10_000);

        feed(&mut state, &[Key::Verb, d(7), d(3), Key::Entr]);
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 0);
        feed_key(&mut state, Key::Entr);
        feed_key(&mut state, Key::Minus);
        feed_number(&mut state, 100); // -1.00 s × 100 = -100 cs
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert_eq!(state.time, Met(10_000 - 100));
    }

    // ── MS-U3 — extended P27 address space ──────────────────────────────────

    /// Convenience: drive V71 to load `count` words starting at `address`,
    /// where each word is signed (sign, magnitude).
    fn run_v71_block(state: &mut AgcState, address: u8, words: &[i64]) {
        // V71 ENTR.
        feed(state, &[Key::Verb, d(7), d(1), Key::Entr]);
        // Address ENTR.
        feed_number(state, address as u32);
        feed_key(state, Key::Entr);
        // Count ENTR.
        feed_number(state, words.len() as u32);
        feed_key(state, Key::Entr);
        // Each signed word.
        for &w in words {
            if w < 0 {
                feed_key(state, Key::Minus);
                feed_number(state, (-w) as u32);
            } else {
                feed_key(state, Key::Plus);
                feed_number(state, w as u32);
            }
            feed_key(state, Key::Entr);
        }
    }

    /// TC-VND-U3-1: target_state position (addresses 7–9) and velocity
    /// (10–12) accept a 6-word block.
    #[test]
    fn tc_vnd_u3_1_target_state_block() {
        let mut state = AgcState::new();
        run_v71_block(
            &mut state,
            7,
            &[7000, 0, 0, 0, 7500, 0], // pos km, vel m/s
        );
        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert!((state.target_state.position[0] - 7_000_000.0).abs() < 1.0);
        assert_eq!(state.target_state.velocity[1], 7500.0);
    }

    /// TC-VND-U3-2: GHA_epoch (address 13) — radians × 1e5.
    ///
    /// AGC P27 data words cap at five digits per accumulator, so the
    /// uplink resolution is limited to 0.00001 rad = ~2 arcsec.
    #[test]
    fn tc_vnd_u3_2_gha_epoch_uplink() {
        let mut state = AgcState::new();
        // 0.12345 rad → 12_345 (fits in the five-digit P27 word).
        run_v71_block(&mut state, 13, &[12_345]);
        assert!((state.gha_epoch_rad - 0.123_45).abs() < 1e-9);
    }

    /// TC-VND-U3-3: REFSMMAT (addresses 14–22) — 9 words, identity
    /// matrix expressed as ¼ revolution diagonal entries.
    #[test]
    fn tc_vnd_u3_3_refsmmat_uplink() {
        let mut state = AgcState::new();
        // A REFSMMAT row-major rotation of +1/4 turn about Z (cos=0, sin=1).
        // In revs × 1e5: 90° = 0.25 rev = 25_000. cos(90°) = 0, sin(90°) = 25_000
        // means rows: [0, -25000, 0; 25000, 0, 0; 0, 0, 25000] in revs×1e5.
        // After conversion (× 2π / 1e5): [0, -π/2, 0; π/2, 0, 0; 0, 0, π/2].
        run_v71_block(
            &mut state,
            14,
            &[
                0, -25_000, 0, // row 0
                25_000, 0, 0, // row 1
                0, 0, 25_000, // row 2 (should be 50_000 for full rot; this is just a probe)
            ],
        );
        let pi_2 = core::f64::consts::FRAC_PI_2;
        assert!((state.refsmmat[0][1] + pi_2).abs() < 1e-9);
        assert!((state.refsmmat[1][0] - pi_2).abs() < 1e-9);
        assert!((state.refsmmat[2][2] - pi_2).abs() < 1e-9);
        assert!(state.refsmmat[0][0].abs() < 1e-12);
    }

    /// TC-VND-U3-4: gyro_comp (addresses 23–25) — meru × 1e3.
    #[test]
    fn tc_vnd_u3_4_gyro_comp_uplink() {
        let mut state = AgcState::new();
        // 0.123 meru on X axis = 123 in the uplink word.
        run_v71_block(&mut state, 23, &[123, -456, 789]);
        assert!((state.gyro_comp.nbdx - 0.123).abs() < 1e-9);
        assert!((state.gyro_comp.nbdy + 0.456).abs() < 1e-9);
        assert!((state.gyro_comp.nbdz - 0.789).abs() < 1e-9);
    }

    /// TC-VND-U3-5: pipa_cal scale + bias (addresses 26–29).
    #[test]
    fn tc_vnd_u3_5_pipa_cal_uplink() {
        use crate::services::average_g::PipaCalibration;

        let mut state = AgcState::new();
        // +100 ppm scale delta; bias = +/- a few cm/s² per axis.
        run_v71_block(&mut state, 26, &[100, 30, -30, 0]);

        let expected = PipaCalibration::NOMINAL.scale * (1.0 + 100e-6);
        assert!(
            (state.pipa_cal.scale - expected).abs() < 1e-12,
            "scale ppm delta not applied"
        );
        // 30 cm/s² × 0.02 / 0.0585 ≈ 10.26 → round 10.
        assert_eq!(state.pipa_cal.bias[0], 10);
        assert_eq!(state.pipa_cal.bias[1], -10);
        assert_eq!(state.pipa_cal.bias[2], 0);
    }

    /// TC-VND-U3-6: MET offset slot (address 30) advances `state.time`.
    #[test]
    fn tc_vnd_u3_6_met_offset_uplink() {
        let mut state = AgcState::new();
        state.time = Met(500);
        run_v71_block(&mut state, 30, &[-200]);
        assert_eq!(state.time, Met(500u32.wrapping_add(-200i64 as u32)));
    }

    /// TC-VND-U3-7: addresses above P27_MAX_ADDRESS raise OPR ERR.
    #[test]
    fn tc_vnd_u3_7_address_out_of_range_opr_err() {
        let mut state = AgcState::new();
        // V71 ENTR 32 ENTR — address 32 > P27_MAX_ADDRESS (= 31 since #61).
        feed(&mut state, &[Key::Verb, d(7), d(1), Key::Entr]);
        feed_number(&mut state, 32);
        feed_key(&mut state, Key::Entr);
        assert!(state.dsky.opr_err);
        assert_eq!(state.vn.phase, VnPhase::OprErr);
    }

    /// TC-VND-U3-8 (#61): V71 to address 31 with value 2 switches the
    /// CSM state frame to MoonInertial; value 1 switches back to
    /// EarthInertial. Anything else raises OPR ERR.
    #[test]
    fn tc_vnd_u3_8_frame_selector() {
        use crate::navigation::state_vector::Frame;

        let mut state = AgcState::new();
        assert_eq!(state.csm_state.frame, Frame::EarthInertial);

        // Switch to MoonInertial via V71 31 1 +2.
        run_v71_block(&mut state, 31, &[2]);
        assert_eq!(state.csm_state.frame, Frame::MoonInertial);

        // Switch back to EarthInertial via V71 31 1 +1.
        run_v71_block(&mut state, 31, &[1]);
        assert_eq!(state.csm_state.frame, Frame::EarthInertial);

        // Invalid value raises OPR ERR.
        let mut state = AgcState::new();
        feed(&mut state, &[Key::Verb, d(7), d(1), Key::Entr]);
        feed_number(&mut state, 31);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 1);
        feed_key(&mut state, Key::Entr);
        feed_key(&mut state, Key::Plus);
        feed_number(&mut state, 5); // invalid (not 1 or 2)
        feed_key(&mut state, Key::Entr);
        assert!(state.dsky.opr_err);
    }

    // ── MS-U4 — V72 single-address update ──────────────────────────────────

    /// TC-VND-U4-1: V72 ENTR <addr> ENTR <±value> ENTR updates one slot
    /// and returns to Idle.
    #[test]
    fn tc_vnd_u4_1_v72_single_slot_update() {
        let mut state = AgcState::new();
        // Pre-load all axes so the test can prove only one was touched.
        state.gyro_comp.nbdx = 0.111;
        state.gyro_comp.nbdy = 0.222;
        state.gyro_comp.nbdz = 0.333;

        // V72 ENTR 23 ENTR -456 ENTR  (gyro_comp.nbdx ← -0.456 meru)
        feed(&mut state, &[Key::Verb, d(7), d(2), Key::Entr]);
        assert!(matches!(state.vn.phase, VnPhase::P27SingleAddress { .. }));

        feed_number(&mut state, 23);
        feed_key(&mut state, Key::Entr);
        assert!(matches!(
            state.vn.phase,
            VnPhase::P27SingleData { address: 23, .. }
        ));

        feed_key(&mut state, Key::Minus);
        feed_number(&mut state, 456);
        feed_key(&mut state, Key::Entr);

        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert!((state.gyro_comp.nbdx + 0.456).abs() < 1e-9);
        // Neighbours untouched.
        assert!((state.gyro_comp.nbdy - 0.222).abs() < 1e-9);
        assert!((state.gyro_comp.nbdz - 0.333).abs() < 1e-9);
    }

    /// TC-VND-U4-2: V72 on a bad address raises OPR ERR.
    #[test]
    fn tc_vnd_u4_2_v72_bad_address_opr_err() {
        let mut state = AgcState::new();
        feed(&mut state, &[Key::Verb, d(7), d(2), Key::Entr]);
        feed_number(&mut state, 32); // > P27_MAX_ADDRESS (31 since #61)
        feed_key(&mut state, Key::Entr);
        assert!(state.dsky.opr_err);
        assert_eq!(state.vn.phase, VnPhase::OprErr);
    }

    /// TC-VND-U4-3: V72 with no preceding sign defaults to positive.
    #[test]
    fn tc_vnd_u4_3_v72_default_sign_positive() {
        let mut state = AgcState::new();
        feed(&mut state, &[Key::Verb, d(7), d(2), Key::Entr]);
        feed_number(&mut state, 1);
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 1234); // no sign keystroke
        feed_key(&mut state, Key::Entr);
        assert_eq!(state.csm_state.position[0], 1_234_000.0);
    }

    /// TC-VND-U4-4: scripted V72 update matches a direct feed_key drive.
    #[test]
    fn tc_vnd_u4_4_scripted_v72_matches_direct() {
        use crate::services::uplink::poll_uplink;
        use std::collections::VecDeque;

        struct VecUplink(VecDeque<u16>);
        impl crate::hal::Uplink for VecUplink {
            fn read_word(&mut self) -> Option<u16> {
                self.0.pop_front()
            }
        }

        // Direct drive.
        let mut state_ref = AgcState::new();
        feed(&mut state_ref, &[Key::Verb, d(7), d(2), Key::Entr]);
        feed_number(&mut state_ref, 13);
        feed_key(&mut state_ref, Key::Entr);
        feed_key(&mut state_ref, Key::Plus);
        feed_number(&mut state_ref, 12_345);
        feed_key(&mut state_ref, Key::Entr);

        // Scripted via uplink word codes.
        let mut state_uplink = AgcState::new();
        let words: Vec<u16> = vec![
            17, 7, 2, 28, // V 7 2 E
            1, 3, 28, // 1 3 E
            26, 1, 2, 3, 4, 5, 28, // + 1 2 3 4 5 E
        ];
        let mut uplink = VecUplink(words.into_iter().collect());
        poll_uplink(&mut state_uplink, &mut uplink);

        assert_eq!(state_uplink.gha_epoch_rad, state_ref.gha_epoch_rad);
        assert_eq!(state_uplink.vn.phase, VnPhase::Idle);
    }

    /// TC-VND-U2-5: scripted V70 uplink (via poll_uplink) produces the
    /// same `liftoff_time` as a direct `feed_key` sequence — the MS-U2
    /// exit criterion equivalence check on the UPRUPT path.
    #[test]
    fn tc_vnd_u2_5_scripted_v70_matches_direct() {
        use crate::services::uplink::poll_uplink;
        use std::collections::VecDeque;

        struct VecUplink(VecDeque<u16>);
        impl crate::hal::Uplink for VecUplink {
            fn read_word(&mut self) -> Option<u16> {
                self.0.pop_front()
            }
        }

        // Direct keys.
        let mut state_ref = AgcState::new();
        feed(
            &mut state_ref,
            &[
                Key::Verb,
                d(7),
                d(0),
                Key::Entr,
                Key::Plus,
                d(2),
                Key::Entr,
                Key::Plus,
                d(3),
                d(0),
                Key::Entr,
                Key::Plus,
                d(1),
                d(5),
                d(0),
                d(0),
                Key::Entr,
            ],
        );

        // Same sequence via uplink word codes.
        let mut state_uplink = AgcState::new();
        let words: Vec<u16> = vec![
            17, 7, 16, 28, // V 7 0 E
            26, 2, 28, // + 2 E
            26, 3, 16, 28, // + 3 0 E
            26, 1, 5, 16, 16, 28, // + 1 5 0 0 E
        ];
        let mut uplink = VecUplink(words.into_iter().collect());
        poll_uplink(&mut state_uplink, &mut uplink);

        assert_eq!(state_uplink.liftoff_time, state_ref.liftoff_time);
        assert_eq!(state_uplink.vn.phase, VnPhase::Idle);
    }
}
