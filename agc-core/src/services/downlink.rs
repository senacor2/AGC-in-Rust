//! MSFN downlink — encode AGC state into the MSFN telemetry word stream.
//!
//! The AGC generates downlink data every 20 ms (50 Hz) via a DOWNRUPT interrupt.
//! Each interrupt places two 15-bit words into output channels 34 and 35; those
//! words are received by the MSFN ground network.
//!
//! One 2-second "downlist cycle" = 100 downrupts × 2 words = 200 AGC words.
//!
//! ## Buffer architecture (O'Brien §16.2)
//!
//! The AGC does **not** pre-build a 200-word buffer.  Instead:
//!
//! - **DNTMBUFF** = 12 erasable words (6 DP pairs) — the snapshot buffer.
//!   Filled atomically once per snapshot collection downrupt; drained over
//!   the following N-1 downrupts.
//! - **Snapshot sublists** (first entry is a negated 1DNADR): N-1 pairs are
//!   stored in DNTMBUFF and the Nth pair is sent live in the same downrupt.
//!   The subsequent `NDNADR DNTMBUFF` entry in the control list drains the
//!   buffer over N-1 further downrupts.
//! - **Control-list direct entries** and **regular sublists**: read live from
//!   erasable each downrupt — no intermediate buffer.
//!
//! DNTMBUFF is exactly 12 words because the largest snapshot sublist
//! (CMPOWE01/05, 7 entries) stores 6 pairs = 12 words.  See
//! `Comanche055/ERASABLE_ASSIGNMENTS.agc` line 1714: `DNTMBUFF ERASE +11D`.
//!
//! ## CMCSTADL cycle map (100 pairs / 100 downrupts)
//!
//! ```text
//! Pair 0      SENDID — ID + LOWIDCOD (special first-downrupt routine)
//! Pairs 1–7   CMPOWE01 snapshot: 1 collect (RN/+1 live) + 6 buffer drain
//!             (RN+2/+3, RN+4/+5, VN/+1, VN+2/+3, VN+4/+5, PIPTIME/+1)
//! Pairs 8–12  CMPOWE02 snapshot: 1 collect (CDUX/CDUY live) + 4 drain
//!             (CDUZ/CDUT, ADOT/+1, ADOT+2/+3, ADOT+4/+5)
//! Pairs 13–16 CMPOWE03 regular: AK+AK1+AK2+RCSFLAGS, THETADX/Y/Z
//! Pair  17    TIG/+1
//! Pair  18    BESTI/BESTJ
//! Pairs 19–22 MARKDOWN (4 DP)
//! Pairs 23–26 MARK2DWN (4 DP)
//! Pairs 27–28 HAPOX (apogee, perigee)
//! Pair  29    PACTOFF/YACTOFF
//! Pairs 30–32 VGTIG (3 DP)
//! Pairs 33–38 REFSMMAT first 6 DP elements (B-0)
//! Pairs 39–49 CMPOWE04: FLAGWRDS 0-9 + DSPTAB
//! Pair  50    TIME2/TIME1
//! Pairs 51–57 CMPOWE05 snapshot: 1 collect (R-OTHER/+1 live) + 6 drain
//! Pairs 58–62 CMPOWE02 repeat: 1 collect (CDUX/CDUY live) + 4 drain
//! Pairs 63–66 CMPOWE03 repeat (4 pairs)
//! Pairs 67–72 CMPOWE06: RSBBQ, CADRFLSH, FAILREG, CDUS/PIPA (6 pairs)
//! Pairs 73–75 OGC/IGC/MGC (3 DP)
//! Pair  76    FLAGWRDS 10+11
//! Pairs 77–78 TEVENT, LAUNCHAZ
//! Pair  79    OPTMODES
//! Pairs 80–93 CMPOWE07: masses, DAPDATR, ERRORX/Y/Z, WBODY, channels (14)
//! Pairs 94–99 DSPTAB (6 DP)
//! ```
//!
//! AGC source: `Comanche055/DOWN-TELEMETRY_PROGRAM.agc`,
//!             `Comanche055/DOWNLINK_LISTS.agc` (CMCSTADL).

use crate::AgcState;

// ── Constants ─────────────────────────────────────────────────────────────────

/// Number of word-pairs per 2-second downlist cycle.
pub const DOWNLIST_PAIRS: usize = 100;

/// Words per downlist cycle (2 × pairs).
pub const DOWNLIST_WORDS: usize = DOWNLIST_PAIRS * 2;

/// LOWIDCOD — AGC sync word sent as the second word of the ID pair (octal 77340).
/// Value: 7×4096 + 7×512 + 3×64 + 4×8 = 32480 = 0x7EE0.
pub const LOWIDCOD: u16 = 0x7EE0;

/// CMCSTADL erasable-dump ID code (channel 34 of pair 0).
pub const CMCSTADL_ID: u16 = 0x00FE;

/// DNTMBUFF size in words: 12 words = 6 DP pairs, sized for the largest
/// snapshot sublist (CMPOWE01/05 with 7 entries stores 6 pairs = 12 words).
/// Source: `Comanche055/ERASABLE_ASSIGNMENTS.agc` line 1714: `DNTMBUFF ERASE +11D`.
const DNTMBUFF_WORDS: usize = 12;

// ── AGC fixed-point encoding ──────────────────────────────────────────────────

/// Encode a normalised value in `[−1, 1]` as a 15-bit AGC one's-complement word.
///
/// - Positive: word = round(v × 16383), clamped to `[0, 16383]`.
/// - Negative: word = one's-complement of positive magnitude, i.e. `!mag & 0x7FFF`.
/// - Zero: word = 0x0000 (+0 in one's-complement).
pub fn encode_agc15(normalized: f64) -> u16 {
    let n = normalized.clamp(-1.0, 1.0);
    if n >= 0.0 {
        let raw = libm::round(n * 16383.0) as i32;
        raw.clamp(0, 16383) as u16
    } else {
        let mag = libm::round((-n) * 16383.0) as i32;
        let mag = mag.clamp(0, 16383) as u16;
        (!mag) & 0x7FFF
    }
}

/// Encode a physical value at `b_scale` exponent as a single AGC word.
///
/// Full scale = 2^b_scale in physical units.
pub fn encode_sp(value: f64, b_scale: i32) -> u16 {
    let normalized = value / libm::pow(2.0, b_scale as f64);
    encode_agc15(normalized)
}

/// Encode a physical value as a double-precision AGC word-pair `(high, low)`.
///
/// Combined 28-bit value: `normalized × 2^28` split into two 14-bit halves.
/// Each word's sign bit (bit 14) is set for negative values.
pub fn encode_dp(value: f64, b_scale: i32) -> (u16, u16) {
    let scale = libm::pow(2.0, b_scale as f64);
    let normalized = (value / scale).clamp(-1.0 + 1.0 / 268_435_456.0, 1.0 - 1.0 / 268_435_456.0);

    if normalized >= 0.0 {
        let combined = libm::round(normalized * 268_435_455.0) as i32;
        let high = ((combined >> 14) & 0x3FFF) as u16;
        let low  = (combined & 0x3FFF) as u16;
        (high, low)
    } else {
        let combined = libm::round((-normalized) * 268_435_455.0) as i32;
        let high = ((combined >> 14) & 0x3FFF) as u16;
        let low  = (combined & 0x3FFF) as u16;
        ((!high & 0x3FFF) | 0x4000, (!low & 0x3FFF) | 0x4000)
    }
}

/// Encode a MET centisecond counter as (TIME2, TIME1).
///
/// TIME1 = lower 14 bits; TIME2 = upper 14 bits.  Both are positive
/// (the counter never wraps below zero), so bit 14 = 0 in both words.
pub fn encode_time(time_cs: u32) -> (u16, u16) {
    let time1 = (time_cs & 0x3FFF) as u16;
    let time2 = ((time_cs >> 14) & 0x3FFF) as u16;
    (time2, time1)
}

// ── Downlink driver ───────────────────────────────────────────────────────────

/// Downlink driver: state for one 2-second CMCSTADL cycle.
///
/// ## Memory (per O'Brien §16.2)
///
/// Holds only a 12-word `DNTMBUFF` (snapshot buffer) plus a pair counter.
/// The full 200-word cycle is computed on demand — no pre-built cache.
/// Memory footprint: 28 bytes (12 × u16 + 1 × usize) versus the 408 bytes
/// a 200-word cache would require.
///
/// ## State machine
///
/// `downlink_step` advances `pair_index` by 1 each call.  For snapshot
/// collection pairs it fills `snapshot_buf`; for snapshot drain pairs it
/// reads from `snapshot_buf`; all other pairs read live from `AgcState`.
#[derive(Clone, Copy, Debug)]
pub struct DownlinkDriver {
    /// Current pair index within the 2-second cycle (0–99).
    pub pair_index: usize,
    /// DNTMBUFF — snapshot buffer (12 words = 6 DP pairs).
    snapshot_buf: [u16; DNTMBUFF_WORDS],
}

impl DownlinkDriver {
    pub const fn new() -> Self {
        Self {
            pair_index: 0,
            snapshot_buf: [0; DNTMBUFF_WORDS],
        }
    }
}

impl Default for DownlinkDriver {
    fn default() -> Self {
        Self::new()
    }
}

// ── Snapshot helpers ──────────────────────────────────────────────────────────

/// Fill `buf[0..12]` with the CMPOWE01/05 snapshot data and return the
/// "last pair" that is sent live in the collection downrupt.
///
/// CMPOWE01 data order (7 entries, AGC snapshot order):
///   buf[0..1]  = RN+2/+3  (ry, B+29)
///   buf[2..3]  = RN+4/+5  (rz, B+29)
///   buf[4..5]  = VN/+1    (vx, B+7)
///   buf[6..7]  = VN+2/+3  (vy, B+7)
///   buf[8..9]  = VN+4/+5  (vz, B+7)
///   buf[10..11]= PIPTIME/+1 (epoch, B+28)
///   live       = RN/+1    (rx, B+29)
fn snapshot_cmpowe01(buf: &mut [u16; DNTMBUFF_WORDS], state: &AgcState) -> (u16, u16) {
    let [rx, ry, rz] = state.csm_state.position;
    let [vx, vy, vz] = state.csm_state.velocity;
    let (h, l) = encode_dp(ry, 29); buf[0] = h; buf[1] = l;
    let (h, l) = encode_dp(rz, 29); buf[2] = h; buf[3] = l;
    let (h, l) = encode_dp(vx, 7);  buf[4] = h; buf[5] = l;
    let (h, l) = encode_dp(vy, 7);  buf[6] = h; buf[7] = l;
    let (h, l) = encode_dp(vz, 7);  buf[8] = h; buf[9] = l;
    let (h, l) = encode_time(state.csm_state.epoch.0); buf[10] = h; buf[11] = l;
    encode_dp(rx, 29) // live: RN/+1
}

/// Fill `buf[0..8]` with CMPOWE02 snapshot data and return the live pair.
///
/// CMPOWE02 (5 entries):
///   buf[0..1]  = CDUZ,CDUT  (CDU Z angle + CDUT not tracked = 0)
///   buf[2..3]  = ADOT/+1   (zero — not tracked)
///   buf[4..5]  = ADOT+2/+3 (zero)
///   buf[6..7]  = ADOT+4/+5 (zero)
///   live       = CDUX,CDUY
fn snapshot_cmpowe02(buf: &mut [u16; DNTMBUFF_WORDS], state: &AgcState) -> (u16, u16) {
    let cdu_z = state.current_cdu[2].to_radians() / core::f64::consts::PI;
    buf[0] = encode_agc15(cdu_z); buf[1] = 0; // CDUZ, CDUT(=0)
    buf[2] = 0; buf[3] = 0; // ADOT/+1
    buf[4] = 0; buf[5] = 0; // ADOT+2/+3
    buf[6] = 0; buf[7] = 0; // ADOT+4/+5
    // live: CDUX, CDUY
    let cdu_x = state.current_cdu[0].to_radians() / core::f64::consts::PI;
    let cdu_y = state.current_cdu[1].to_radians() / core::f64::consts::PI;
    (encode_agc15(cdu_x), encode_agc15(cdu_y))
}

/// Fill `buf[0..12]` with CMPOWE05 snapshot data and return the live pair.
///
/// CMPOWE05 (7 entries) — R-OTHER/V-OTHER/T-OTHER not tracked in this port
/// (zero is valid downlink filler).
///   live = R-OTHER/+1 (zero)
fn snapshot_cmpowe05(buf: &mut [u16; DNTMBUFF_WORDS], _state: &AgcState) -> (u16, u16) {
    for w in buf.iter_mut() { *w = 0; }
    (0, 0) // R-OTHER/+1 live
}

// ── Per-pair on-demand encoder ────────────────────────────────────────────────

/// Send one DOWNRUPT word-pair, advancing the driver by one step.
///
/// Three cases based on `pair_index`:
/// - **Pair 0 (SENDID):** sends ID+LOWIDCOD; no buffer interaction.
/// - **Snapshot-collect pairs (1, 8, 51, 58):** fills `snapshot_buf`,
///   sends the "last pair" live, then exits.
/// - **Snapshot-drain pairs (2–7, 9–12, 52–57, 59–62):** reads from
///   `snapshot_buf`.
/// - **All other pairs:** reads live from `AgcState`.
///
/// AGC source: `Comanche055/DOWN-TELEMETRY_PROGRAM.agc` — DODOWNTM/DNPHASE2.
pub fn downlink_step<T: crate::hal::Telemetry>(
    driver: &mut DownlinkDriver,
    state: &AgcState,
    telemetry: &mut T,
) {
    let k = driver.pair_index;
    let (w34, w35) = compute_pair(driver, state, k);
    telemetry.send_word(w34);
    telemetry.send_word(w35);
    driver.pair_index = (driver.pair_index + 1) % DOWNLIST_PAIRS;
}

fn compute_pair(driver: &mut DownlinkDriver, state: &AgcState, k: usize) -> (u16, u16) {
    use crate::navigation::conics::{apoapsis_altitude_earth, periapsis_altitude_earth, sv_to_elements};
    use crate::navigation::state_vector::Frame;

    match k {
        // ── Pair 0: SENDID ────────────────────────────────────────────────────
        0 => (CMCSTADL_ID, LOWIDCOD),

        // ── Pairs 1–7: CMPOWE01 snapshot (RN + VN + PIPTIME) ─────────────────
        // Pair 1 = snapshot collect (7 entries; sends RN/+1 live)
        1 => snapshot_cmpowe01(&mut driver.snapshot_buf, state),
        // Pairs 2–7 = drain snapshot_buf[0..11]
        2..=7 => {
            let i = (k - 2) * 2;
            (driver.snapshot_buf[i], driver.snapshot_buf[i + 1])
        }

        // ── Pairs 8–12: CMPOWE02 snapshot (CDU angles + ADOT) ────────────────
        8 => snapshot_cmpowe02(&mut driver.snapshot_buf, state),
        9..=12 => {
            let i = (k - 9) * 2;
            (driver.snapshot_buf[i], driver.snapshot_buf[i + 1])
        }

        // ── Pairs 13–16: CMPOWE03 regular (AK/RCSFLAGS + THETADX/Y/Z) ────────
        13 | 14 => {
            let ex = state.dap_state.attitude_error[0] / core::f64::consts::PI;
            let ey = state.dap_state.attitude_error[1] / core::f64::consts::PI;
            let ez = state.dap_state.attitude_error[2] / core::f64::consts::PI;
            if k == 13 { (encode_agc15(ex), encode_agc15(ey)) }
            else        { (encode_agc15(ez), 0) }
        }
        15 | 16 => (0, 0), // AK/RCSFLAGS, THETADX/Y/Z — not tracked

        // ── Pair 17: TIG/+1 ──────────────────────────────────────────────────
        17 => state.pending_maneuver.map_or((0, 0), |m| encode_time(m.tig.0)),

        // ── Pairs 18–28: BESTI, MARKDOWN, MARK2DWN, HAPOX, PACTOFF ──────────
        18 => (0, 0), // BESTI/BESTJ — not tracked
        19..=22 => (0, 0), // MARKDOWN — not tracked
        23..=26 => (0, 0), // MARK2DWN — not tracked
        27 | 28 => {       // HAPOX — apogee / perigee
            if state.csm_state.epoch.0 != 0 && state.csm_state.frame == Frame::EarthInertial {
                let el = sv_to_elements(state.csm_state);
                if !el.is_hyperbolic() {
                    let alt = if k == 27 {
                        apoapsis_altitude_earth(&el)
                    } else {
                        periapsis_altitude_earth(&el)
                    };
                    return encode_dp(alt, 29);
                }
            }
            (0, 0)
        }
        29 => (0, 0), // PACTOFF/YACTOFF — not tracked

        // ── Pairs 30–32: VGTIG (3 DP, not tracked) ───────────────────────────
        30..=32 => (0, 0),

        // ── Pairs 33–38: REFSMMAT first 6 DP elements (B-0) ──────────────────
        33..=38 => {
            let i = k - 33;
            let m = state.refsmmat;
            let flat = [m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2]];
            encode_dp(flat[i], 0)
        }

        // ── Pairs 39–49: CMPOWE04 — FLAGWRDS 0-9 + DSPTAB ───────────────────
        39..=43 => {
            let i = k - 39;
            let fw0 = state.flagwords[2 * i] & 0x7FFF;
            let fw1 = state.flagwords[2 * i + 1] & 0x7FFF;
            (fw0, fw1)
        }
        44..=49 => (0, 0), // DSPTAB — display tables (zero until P-program populates)

        // ── Pair 50: TIME2/TIME1 ──────────────────────────────────────────────
        50 => encode_time(state.time.0),

        // ── Pairs 51–57: CMPOWE05 snapshot (R-OTHER/V-OTHER/T-OTHER) ─────────
        51 => snapshot_cmpowe05(&mut driver.snapshot_buf, state),
        52..=57 => {
            let i = (k - 52) * 2;
            (driver.snapshot_buf[i], driver.snapshot_buf[i + 1])
        }

        // ── Pairs 58–62: CMPOWE02 repeat ─────────────────────────────────────
        58 => snapshot_cmpowe02(&mut driver.snapshot_buf, state),
        59..=62 => {
            let i = (k - 59) * 2;
            (driver.snapshot_buf[i], driver.snapshot_buf[i + 1])
        }

        // ── Pairs 63–66: CMPOWE03 repeat (live) ──────────────────────────────
        63 | 64 => {
            let ex = state.dap_state.attitude_error[0] / core::f64::consts::PI;
            let ey = state.dap_state.attitude_error[1] / core::f64::consts::PI;
            let ez = state.dap_state.attitude_error[2] / core::f64::consts::PI;
            if k == 63 { (encode_agc15(ex), encode_agc15(ey)) }
            else        { (encode_agc15(ez), 0) }
        }
        65 | 66 => (0, 0),

        // ── Pairs 67–72: CMPOWE06 — RSBBQ/CADRFLSH/FAILREG/CDUS/PIPA (live) ─
        67..=72 => (0, 0), // not tracked in this port

        // ── Pairs 73–75: OGC/IGC/MGC (zero — P02 not yet implemented) ────────
        73..=75 => (0, 0),

        // ── Pair 76: FLAGWRDS 10+11 ───────────────────────────────────────────
        76 => {
            let fw10 = state.flagwords.get(10).copied().unwrap_or(0) & 0x7FFF;
            let fw11 = state.flagwords.get(11).copied().unwrap_or(0) & 0x7FFF;
            (fw10, fw11)
        }

        // ── Pairs 77–78: TEVENT, LAUNCHAZ (zero) ─────────────────────────────
        77 | 78 => (0, 0),

        // ── Pair 79: OPTMODES (zero) ──────────────────────────────────────────
        79 => (0, 0),

        // ── Pairs 80–93: CMPOWE07 — LEMMASS, DAPDATR, ERRORX/Y/Z, WBODY, … ──
        82 => {
            let ex = state.dap_state.attitude_error[0] / core::f64::consts::PI;
            let ey = state.dap_state.attitude_error[1] / core::f64::consts::PI;
            (encode_agc15(ex), encode_agc15(ey))
        }
        83 => {
            let ez = state.dap_state.attitude_error[2] / core::f64::consts::PI;
            (encode_agc15(ez), 0)
        }
        86 => (state.alarm.code & 0x7FFF, 0), // IMODES30/33 → alarm code
        80..=93 => (0, 0),

        // ── Pairs 94–99: DSPTAB (display tables) ─────────────────────────────
        94 => {
            let major = state.dsky.prog as u16 & 0x7F;
            let verb  = state.dsky.verb  as u16 & 0x7F;
            let noun  = state.dsky.noun  as u16 & 0x7F;
            ((major << 7) | verb, noun)
        }
        95 => {
            let lamp_bits: u16 = (state.alarm.lit as u16)
                | ((state.dsky.opr_err     as u16) << 1)
                | ((state.dsky.gimbal_lock as u16) << 2)
                | ((state.dsky.no_att      as u16) << 3);
            (lamp_bits, 0)
        }
        96..=99 => (0, 0),

        _ => (0, 0),
    }
}

// ── Test helper ───────────────────────────────────────────────────────────────

/// A complete 2-second downlist as a flat 200-word array (test / capture use).
pub type DownlistBuffer = [u16; DOWNLIST_WORDS];

/// Build a complete downlist buffer by driving `downlink_step` 100 times.
///
/// Used by integration tests and the `capture_downlink` fixture tool.
/// The real-time path calls `downlink_step` once per DOWNRUPT without a cache.
pub fn build_cmcstadl(state: &AgcState) -> DownlistBuffer {
    struct Collector {
        buf: DownlistBuffer,
        pos: usize,
    }
    impl crate::hal::Telemetry for Collector {
        fn send_word(&mut self, w: u16) {
            if self.pos < DOWNLIST_WORDS {
                self.buf[self.pos] = w;
                self.pos += 1;
            }
        }
    }
    let mut driver = DownlinkDriver::new();
    let mut col = Collector { buf: [0; DOWNLIST_WORDS], pos: 0 };
    for _ in 0..DOWNLIST_PAIRS {
        downlink_step(&mut driver, state, &mut col);
    }
    col.buf
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::Met;

    // ── encode_agc15 ─────────────────────────────────────────────────────────

    /// TC-DL-1: zero encodes to 0x0000 (+0 in one's complement).
    #[test]
    fn tc_dl_1_zero_encodes_to_plus_zero() {
        assert_eq!(encode_agc15(0.0), 0x0000);
    }

    /// TC-DL-2: +1.0 encodes to maximum positive (0x3FFF = +16383).
    #[test]
    fn tc_dl_2_plus_one_clamps_to_max_positive() {
        assert_eq!(encode_agc15(1.0), 0x3FFF);
    }

    /// TC-DL-3: −1.0 encodes to maximum negative (0x4000 in 15-bit OC).
    #[test]
    fn tc_dl_3_minus_one_clamps_to_max_negative() {
        let word = encode_agc15(-1.0);
        assert_eq!(word, 0x4000, "expected 0x4000, got 0x{word:04X}");
    }

    /// TC-DL-4: +0.5 → word = 8192 = 0x2000.
    #[test]
    fn tc_dl_4_half_positive() {
        assert_eq!(encode_agc15(0.5), 8192);
    }

    /// TC-DL-5: encode_sp with B+0 matches encode_agc15.
    #[test]
    fn tc_dl_5_encode_sp_b0() {
        for v in [0.0, 0.25, -0.5, 0.75] {
            assert_eq!(encode_sp(v, 0), encode_agc15(v), "mismatch at {v}");
        }
    }

    /// TC-DL-6: encode_dp zero → both words zero.
    #[test]
    fn tc_dl_6_dp_zero() {
        assert_eq!(encode_dp(0.0, 28), (0, 0));
    }

    /// TC-DL-7: encode_dp 1000 cs at B+28 round-trips within 1 cs.
    #[test]
    fn tc_dl_7_dp_round_trip_b28() {
        let (hi, lo) = encode_dp(1_000.0, 28);
        let combined = ((hi as u32) << 14) | (lo as u32);
        assert!((combined as i64 - 1_000).abs() <= 1);
    }

    /// TC-DL-8: encode_time separates high and low 14-bit halves.
    #[test]
    fn tc_dl_8_encode_time() {
        let (t2, t1) = encode_time(10_000);
        assert_eq!(t1, 10_000);
        assert_eq!(t2, 0);

        let (t2, t1) = encode_time(20_000);
        assert_eq!(t2, 1);
        assert_eq!(t1, 20_000 - 16_384);
    }

    /// TC-DL-9: ID pair (index 0) must be (CMCSTADL_ID, LOWIDCOD).
    #[test]
    fn tc_dl_9_id_pair() {
        let state = crate::AgcState::new();
        let buf = build_cmcstadl(&state);
        assert_eq!(buf[0], CMCSTADL_ID);
        assert_eq!(buf[1], LOWIDCOD);
    }

    /// TC-DL-10: TIME2/TIME1 at pair 50 matches encode_time(state.time.0).
    #[test]
    fn tc_dl_10_time_pair() {
        let mut state = crate::AgcState::new();
        state.time = Met(20_000);
        let buf = build_cmcstadl(&state);
        let (expected_t2, expected_t1) = encode_time(20_000);
        assert_eq!(buf[2 * 50], expected_t2, "TIME2 mismatch");
        assert_eq!(buf[2 * 50 + 1], expected_t1, "TIME1 mismatch");
    }

    /// TC-DL-11: Fresh-start buffer has 200 words.
    #[test]
    fn tc_dl_11_fresh_start_buffer_length() {
        let buf = build_cmcstadl(&crate::AgcState::new());
        assert_eq!(buf.len(), DOWNLIST_WORDS);
    }

    /// TC-DL-12: DownlinkDriver advances pair_index and resets after 100 pairs.
    #[test]
    fn tc_dl_12_driver_pair_index_cycles() {
        struct NullTelemetry;
        impl crate::hal::Telemetry for NullTelemetry { fn send_word(&mut self, _: u16) {} }

        let state = crate::AgcState::new();
        let mut driver = DownlinkDriver::new();
        let mut tel = NullTelemetry;

        for step in 0..DOWNLIST_PAIRS {
            assert_eq!(driver.pair_index, step);
            downlink_step(&mut driver, &state, &mut tel);
        }
        assert_eq!(driver.pair_index, 0);
    }

    /// TC-DL-13: LOWIDCOD constant equals octal 77340.
    #[test]
    fn tc_dl_13_lowidcod_value() {
        let octal_77340: u16 = 7 * 4096 + 7 * 512 + 3 * 64 + 4 * 8;
        assert_eq!(LOWIDCOD, octal_77340);
    }

    /// TC-DL-14: DNTMBUFF is populated during snapshot-collect pair and
    /// correctly drained over the subsequent pairs.
    ///
    /// Checks that after pair 1 (CMPOWE01 collect), pairs 2-7 return
    /// consistent data matching what snapshot_cmpowe01 would produce directly.
    #[test]
    fn tc_dl_14_snapshot_collect_and_drain_consistent() {
        use crate::navigation::gravity::{MU_EARTH, R_EARTH};
        use crate::navigation::state_vector::{Frame, StateVector};

        let mut state = crate::AgcState::new();
        state.csm_state = StateVector {
            position: [R_EARTH + 200_000.0, 1_000_000.0, -500_000.0],
            velocity: [0.0, libm::sqrt(MU_EARTH / (R_EARTH + 200_000.0)), 100.0],
            epoch: Met(3_600_000),
            frame: Frame::EarthInertial,
        };

        let buf = build_cmcstadl(&state);

        // The buffer was built by driving the driver 100 times.
        // Pairs 2-7 should contain the CMPOWE01 snapshot data.
        // Manually compute what snapshot_cmpowe01 produces for this state.
        let mut snap = [0u16; DNTMBUFF_WORDS];
        let live = snapshot_cmpowe01(&mut snap, &state);

        // Pair 1 (collect) should equal the live pair (RN/+1 = rx DP).
        assert_eq!((buf[2], buf[3]), live, "pair 1 must match live RN/+1");

        // Pairs 2-7 (drain) must match snap[0..11].
        for i in 0..6 {
            let k = 2 + i;
            let expected = (snap[2 * i], snap[2 * i + 1]);
            let got = (buf[2 * k], buf[2 * k + 1]);
            assert_eq!(got, expected, "pair {k} drain mismatch");
        }
    }
}
