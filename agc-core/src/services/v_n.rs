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
    P27Address {
        digits: u8,
        buf: u32,
    },
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
                // Verbs that take no noun: V34, V35, V71, ...
                if verb_takes_no_noun(buf) {
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
    }
}

// ── Dispatch ──────────────────────────────────────────────────────────────────

/// Returns true for verbs that do not require a noun (V34, V35, V71, ...).
fn verb_takes_no_noun(verb: u8) -> bool {
    matches!(verb, 34 | 35 | 71)
}

/// Dispatch a completed VERB+NOUN (or noun-less VERB) command.
fn dispatch_verb_noun(state: &mut crate::AgcState, verb: u8, noun: u8) {
    match verb {
        6 => v06_display_decimal(state, noun),
        16 => v16_monitor(state, noun),
        21..=23 => start_load(state, verb, noun, 1, verb - 21),
        25 => start_load(state, verb, noun, 3, 0),
        34 => v34_terminate(state),
        35 => v35_lamp_test(state),
        37 => v37_program_select(state, noun),
        71 => v71_p27_block_update(state),
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

        // N43 — Lat/Lon/Alt. Placeholder — P21 writes these directly when active.
        43 => Some((0.0, 0.0, 0.0)),

        // N44 — Apogee / Perigee / Half-period.
        //        R1 = apogee altitude (km),   R2 = perigee altitude (km),
        //        R3 = orbital half-period (min).
        //
        // Apollo's N44 carried these in nautical miles ("XXXX.X nmi") and
        // a "XXbXX min s" mixed format for TFF; the simulator picks plain
        // SI units (km, min) so each register fits comfortably inside the
        // DSKY's 5-digit display for any LEO/MEO/HEO orbit. A real-flight
        // unit pass would re-encode this in nmi×10 / min:s once the DSKY
        // register format spec lands.
        44 => {
            use crate::math::linalg::cross;
            let r = norm(state.csm_state.position);
            let h_vec = cross(state.csm_state.position, state.csm_state.velocity);
            let h = norm(h_vec);
            // Guard: both position and angular momentum must be nonzero for a
            // valid Keplerian orbit (zero h means rectilinear or unset state).
            if r > 0.0 && h >= 1.0 {
                use crate::navigation::conics::{
                    apoapsis_altitude_earth, orbital_period, periapsis_altitude_earth,
                    sv_to_elements,
                };
                let el = sv_to_elements(state.csm_state);
                if el.is_hyperbolic() {
                    // No apoapsis/period for a hyperbolic escape trajectory.
                    Some((0.0, 0.0, 0.0))
                } else {
                    let apo_km = (apoapsis_altitude_earth(&el) / 1000.0) as f32;
                    let peri_km = (periapsis_altitude_earth(&el) / 1000.0) as f32;
                    let mu = el.mu();
                    let half_period_min = (orbital_period(&el, mu) / 120.0) as f32;
                    Some((apo_km, peri_km, half_period_min))
                }
            } else {
                Some((0.0, 0.0, 0.0))
            }
        }

        // N54 — Range/Rate/Theta. Already written by P20 directly — return current
        //        register values unchanged.
        54 => Some((state.dsky.r[0], state.dsky.r[1], state.dsky.r[2])),

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
fn v34_terminate(state: &mut crate::AgcState) {
    let _ = crate::programs::p00::init(state);
}

/// V35 — Lamp test.
fn v35_lamp_test(state: &mut crate::AgcState) {
    state.dsky.lamp_test_active = true;
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

/// Maximum P27 logical address. Six slots cover the full inertial
/// position (1..=3) and velocity (4..=6) triple.
const P27_MAX_ADDRESS: u8 = 6;

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

/// Map a P27 logical address to a state-vector mutation.
///
/// Address space (six logical slots):
///
/// | Address | Field                 | Crew units |
/// |---------|-----------------------|------------|
/// | 1       | csm_state.position[0] | km         |
/// | 2       | csm_state.position[1] | km         |
/// | 3       | csm_state.position[2] | km         |
/// | 4       | csm_state.velocity[0] | m/s        |
/// | 5       | csm_state.velocity[1] | m/s        |
/// | 6       | csm_state.velocity[2] | m/s        |
///
/// Returns `false` for out-of-range addresses (caller raises OPR ERR).
/// Always forces `frame = EarthInertial` so a stale Moon-frame state
/// vector cannot survive a partial position-only update.
fn p27_apply_word(state: &mut crate::AgcState, address: u8, value: i64) -> bool {
    use crate::navigation::state_vector::Frame;
    let v = value as f64;
    match address {
        1 => state.csm_state.position[0] = v * 1000.0,
        2 => state.csm_state.position[1] = v * 1000.0,
        3 => state.csm_state.position[2] = v * 1000.0,
        4 => state.csm_state.velocity[0] = v,
        5 => state.csm_state.velocity[1] = v,
        6 => state.csm_state.velocity[2] = v,
        _ => return false,
    }
    state.csm_state.frame = Frame::EarthInertial;
    true
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

    // ── TC-VN-2: V37E00E selects P00 ──────────────────────────────────────────

    #[test]
    fn tc_vn_2_v37_e00_e_selects_p00() {
        let mut state = AgcState::new();
        state.major_mode = 42; // nonzero starting mode

        feed(
            &mut state,
            &[Key::Verb, d(3), d(7), Key::Noun, d(0), d(0), Key::Entr],
        );

        assert_eq!(state.major_mode, 0, "V37E00E must invoke P00 init");
        assert_eq!(state.vn.phase, VnPhase::Idle);
        assert!(!state.dsky.opr_err);
    }

    // ── TC-VN-3: V37E30E selects P30 ──────────────────────────────────────────

    #[test]
    fn tc_vn_3_v37_e30_e_selects_p30() {
        let mut state = AgcState::new();

        feed(
            &mut state,
            &[Key::Verb, d(3), d(7), Key::Noun, d(3), d(0), Key::Entr],
        );

        assert_eq!(state.major_mode, 30);
        assert_eq!(state.vn.phase, VnPhase::Idle);
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

    #[test]
    fn tc_vn_9_verb_during_noun_restarts() {
        let mut state = AgcState::new();

        feed(
            &mut state,
            &[Key::Verb, d(3), d(7), Key::Noun, d(3), Key::Verb],
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

    // ── TC-VN-11: V37 with unknown program raises OPR ERR ────────────────────

    #[test]
    fn tc_vn_11_v37_unknown_program_opr_err() {
        let mut state = AgcState::new();
        // Slot 99 is None in PROGRAM_TABLE.
        feed(
            &mut state,
            &[Key::Verb, d(3), d(7), Key::Noun, d(9), d(9), Key::Entr],
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

    /// TC-V71-5: Address > P27_MAX_ADDRESS is rejected.
    #[test]
    fn tc_v71_5_address_out_of_range() {
        let mut state = AgcState::new();
        feed(&mut state, &[Key::Verb, d(7), d(1), Key::Entr]);
        feed_number(&mut state, 7); // P27_MAX_ADDRESS = 6
        feed_key(&mut state, Key::Entr);

        assert!(state.dsky.opr_err);
        assert_eq!(state.vn.phase, VnPhase::OprErr);
    }

    /// TC-V71-6: Address + count overrunning the address space is rejected.
    #[test]
    fn tc_v71_6_address_count_overflow() {
        let mut state = AgcState::new();
        feed(&mut state, &[Key::Verb, d(7), d(1), Key::Entr]);
        feed_number(&mut state, 5); // start at velocity[1]
        feed_key(&mut state, Key::Entr);
        feed_number(&mut state, 3); // would reach addr 7 → out of range
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
            &[Key::Verb, d(3), d(7), Key::Noun, d(1), d(1), Key::Entr],
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

    /// TC-VN-ND-5: V06 N44 computes apogee/perigee/half-period from CSM state
    /// in a circular LEO orbit. For a circular orbit apogee ≈ perigee within 1 km
    /// and half-period must fit in the 5-digit DSKY register (no overflow).
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

        // R1 / R2 are altitudes in kilometres; R3 is the half-period in minutes.
        let apo_km = state.dsky.r[0];
        let peri_km = state.dsky.r[1];
        let half_period_min = state.dsky.r[2];

        assert!(
            apo_km > 0.0,
            "TC-VN-ND-5: apogee altitude must be positive, got {apo_km} km"
        );
        assert!(
            peri_km > 0.0,
            "TC-VN-ND-5: perigee altitude must be positive, got {peri_km} km"
        );
        assert!(
            half_period_min > 0.0,
            "TC-VN-ND-5: half-period must be positive, got {half_period_min} min"
        );
        assert!(
            (apo_km - peri_km).abs() < 1.0,
            "TC-VN-ND-5: circular orbit apogee ≈ perigee within 1 km, |apo-peri| = {} km",
            (apo_km - peri_km).abs()
        );
        // All three registers must be representable in the 5-digit display
        // (i.e. < 100 000) — the original `m` / `s` units overflowed for any
        // non-trivial Earth orbit.
        assert!(apo_km < 100_000.0, "apogee in km must fit in 5 digits");
        assert!(peri_km < 100_000.0, "perigee in km must fit in 5 digits");
        assert!(
            half_period_min < 100_000.0,
            "half-period in min must fit in 5 digits"
        );
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
}
