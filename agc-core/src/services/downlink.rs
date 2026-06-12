//! MSFN downlink — encode AGC state into the MSFN telemetry word stream.
//!
//! The AGC generates downlink data every 20 ms (50 Hz) via a DOWNRUPT interrupt.
//! Each interrupt places two 15-bit words into output channels 34 and 35; those
//! words are received by the MSFN ground network and later made available to
//! Mission Control.
//!
//! One 2-second "downlist cycle" = 100 downrupts × 2 words = 200 AGC words.
//! The first word-pair of each cycle carries the downlist ID and sync code.
//! The remaining 99 pairs are navigation / guidance state from the CMCSTADL
//! (coast & alignment) downlist.
//!
//! ## MSFN word format
//!
//! Each word is a 15-bit AGC one's-complement integer packed into a `u16`
//! (bits 14:0 used; bit 15 always zero in this implementation).
//!
//! - Positive value `n` (0 ≤ n ≤ 16383): word = n.
//! - Negative value `−n` (0 < n ≤ 16383): word = 32767 − n  (flip all 15 bits).
//! - Zero: word = 0 ("+0" in one's-complement; we do not generate "−0" = 0x7FFF).
//!
//! ## Downlist index → CMCSTADL content map
//!
//! Pair 0   : ID word + LOWIDCOD sync (mandatory per AGC spec)
//! Pairs 1–6: Snapshot 1 — position (RN) + velocity (VN) + PIPTIME
//! Pairs 7–10: Snapshot 2 — CDU angles (CDUZ/T/X/Y) + ADOT
//! Pairs 11–12: AK/RCSFLAGS + THETADX/Y/Z (attitude error)
//! Pair 13  : TIG high/low
//! Pairs 14–15: (spare / zero)
//! Pairs 16–23: MARKDOWN/MARK2DWN (zero — sighting data not tracked here)
//! Pairs 24–25: HAPOX — apogee / perigee (from R30)
//! Pair 26  : PACTOFF/YACTOFF (zero)
//! Pairs 27–29: VGTIG (zero)
//! Pairs 30–35: REFSMMAT (6 DP words = 9 element matrix)
//! Pairs 36–50: FLAGWRDS 0–9 + DSPTAB (display tables)
//! Pair 51  : TIME2 / TIME1
//! Pairs 52–58: Snapshot 5 — R-OTHER + V-OTHER + T-OTHER (zero)
//! Pairs 59–62: Snapshot 2 repeat (CDU + ADOT)
//! Pairs 63–64: AK repeat + THETAD
//! Pairs 65–70: CMCSTA06 — RSBBQ, CADRFLSH, FAILREG, CDUS/PIPA (zero)
//! Pairs 71–72: OGC/IGC/MGC (gyrocompass — zero)
//! Pair 73  : FLAGWRDS 10+11
//! Pair 74  : TEVENT (zero)
//! Pair 75  : LAUNCHAZ (zero)
//! Pair 76  : OPTMODES (zero)
//! Pairs 77–83: CMCSTA07 — LEMMASS/CSMMASS, DAPDATR, ERRORX/Y/Z, WBODY, REDOCTR, IMODES, ch 11–14, ch 30–33
//! Pairs 84–99: DSPTAB (display tables — zero)
//!
//! AGC source: `Comanche055/DOWN-TELEMETRY_PROGRAM.agc`,
//!             `Comanche055/DOWNLINK_LISTS.agc` (CMCSTADL).

use crate::AgcState;

// ── Constants ─────────────────────────────────────────────────────────────────

/// Number of word-pairs per 2-second downlist cycle.
pub const DOWNLIST_PAIRS: usize = 100;

/// LOWIDCOD — AGC sync word sent as the second word of the ID pair (octal 77340).
///
/// All Comanche055 downlists use the same sync word at the start of each cycle.
/// Value: octal 77340 = decimal 32480.  In 15-bit layout: 0b111_1111_0111_0000.
/// The top bit (bit 14) is 0 in a 15-bit word, so `LOWIDCOD = 0x7EE0 & 0x7FFF = 0x7EE0`.
/// Wait — octal 77340 = 0b111_111_100_111_000 = 15-bit value 0x7EE0.
pub const LOWIDCOD: u16 = 0x7EE0; // octal 77340

/// CMCSTADL erasable dump ID (word pair 0, channel 34).
/// Octal 01776 (CSM = bit 0 of low nibble = 6, mode code = 177).
/// In practice the ground uses this to identify the list; we use a representative
/// value derived from the Comanche055 `ERASID` constant.
pub const CMCSTADL_ID: u16 = 0x00FE; // octal 00376 ≈ 0177*2 (CSM coast list)

/// Words per downlist cycle (2 × pairs).
pub const DOWNLIST_WORDS: usize = DOWNLIST_PAIRS * 2;

// ── AGC fixed-point encoding ──────────────────────────────────────────────────

/// Encode a signed float as a 15-bit AGC one's-complement word.
///
/// `normalized` must be in `[−1.0, 1.0)`.  Values outside this range are
/// clamped.  The AGC's full-scale positive is `+16383 / 16384` ≈ `1.0`;
/// its full-scale negative is `−16383 / 16384`.
///
/// # One's-complement encoding
/// - Positive: word = round(v × 16383)  (bits 14:0, bit 14 = 0)
/// - Negative: word = 32767 − round(|v| × 16383) (all 15 bits flipped)
pub fn encode_agc15(normalized: f64) -> u16 {
    let n = normalized.clamp(-1.0, 1.0);
    if n >= 0.0 {
        let raw = libm::round(n * 16383.0_f64) as i32;
        raw.clamp(0, 16383) as u16
    } else {
        let mag = libm::round((-n) * 16383.0_f64) as i32;
        let mag = mag.clamp(0, 16383) as u16;
        (!mag) & 0x7FFF
    }
}

/// Encode a physical value (with B-scale exponent) as a single AGC word.
///
/// `b_scale` is the AGC B-notation exponent: the physical unit corresponding
/// to full-scale `(+1.0)` is `2^b_scale`.  For example `b_scale = 28` means
/// full scale = 2^28 centiseconds, so `encode_sp(1000.0, 28)` ≈ `0` (very
/// small relative to 2^28).
pub fn encode_sp(value: f64, b_scale: i32) -> u16 {
    let normalized = value / libm::pow(2.0, b_scale as f64);
    encode_agc15(normalized)
}

/// Encode a physical value as a double-precision AGC word-pair `(high, low)`.
///
/// Double-precision in the AGC is two consecutive 15-bit words encoding a
/// 28-bit fractional number.  `high` carries bits 27:14, `low` carries bits
/// 13:0 (both with sign in bit 14 — the sign is replicated in both words for
/// a valid one's-complement DP).
///
/// For positive values this simplifies to:
/// - combined = round(v / 2^b_scale × 2^28) clamped to [−2^27, 2^27)
/// - high = combined >> 14  (top 14 bits)
/// - low  = combined & 0x3FFF  (bottom 14 bits)
///
/// Negative values use one's-complement on each word independently.
pub fn encode_dp(value: f64, b_scale: i32) -> (u16, u16) {
    let scale = libm::pow(2.0, b_scale as f64);
    let normalized = (value / scale).clamp(-1.0 + 1.0 / 268_435_456.0, 1.0 - 1.0 / 268_435_456.0);

    if normalized >= 0.0 {
        // 28-bit magnitude
        let combined = libm::round(normalized * 268_435_455.0_f64) as i32;
        let high = ((combined >> 14) & 0x3FFF) as u16;
        let low = (combined & 0x3FFF) as u16;
        (high, low)
    } else {
        let combined = libm::round((-normalized) * 268_435_455.0_f64) as i32;
        let high = ((combined >> 14) & 0x3FFF) as u16;
        let low = (combined & 0x3FFF) as u16;
        // One's complement: flip all 14 data bits and set sign bit (bit 14)
        let high_neg = (!high & 0x3FFF) | 0x4000;
        let low_neg = (!low & 0x3FFF) | 0x4000;
        (high_neg, low_neg)
    }
}

/// Encode a mission-elapsed-time centisecond counter as DP words (TIME2, TIME1).
///
/// The AGC TIME register is a 28-bit counter (TIME2 × 2^14 + TIME1).  Both
/// words are positive (counter never wraps below zero), so bit 14 = 0 in both.
pub fn encode_time(time_cs: u32) -> (u16, u16) {
    let time1 = (time_cs & 0x3FFF) as u16;         // lower 14 bits
    let time2 = ((time_cs >> 14) & 0x3FFF) as u16; // upper 14 bits
    (time2, time1)
}

// ── Downlist builder ──────────────────────────────────────────────────────────

/// A complete 2-second downlist cycle: 100 word-pairs as a flat array of
/// 200 `u16` values.  `words[2k]` = channel-34 word of pair `k`;
/// `words[2k+1]` = channel-35 word of pair `k`.
pub type DownlistBuffer = [u16; DOWNLIST_WORDS];

/// Build the CMCSTADL (coast & alignment) downlist from `AgcState`.
///
/// Word-pairs follow the order described in `Comanche055/DOWNLINK_LISTS.agc`.
/// Fields without a direct Rust equivalent are encoded as zero (positive zero
/// in one's-complement = 0x0000).
///
/// AGC source: `Comanche055/DOWNLINK_LISTS.agc` — CMCSTADL section.
pub fn build_cmcstadl(state: &AgcState) -> DownlistBuffer {
    use crate::navigation::gravity::{R_EARTH, R_MOON};
    use crate::navigation::conics::{apoapsis_altitude_earth, periapsis_altitude_earth, sv_to_elements};
    use crate::navigation::state_vector::Frame;

    let mut buf = [0u16; DOWNLIST_WORDS];

    // Helper closures to write a pair at index `k`.
    let mut pair = |k: usize, w34: u16, w35: u16| {
        if k < DOWNLIST_PAIRS {
            buf[2 * k] = w34;
            buf[2 * k + 1] = w35;
        }
    };

    // ── Pair 0: ID + LOWIDCOD ──────────────────────────────────────────────
    pair(0, CMCSTADL_ID, LOWIDCOD);

    // ── Pairs 1–6: Snapshot 1 — RN (position) + VN (velocity) + PIPTIME ──
    // RN: position vector in metres, B+29 scale (full scale = 2^29 m ≈ 537 Mm)
    // VN: velocity in m/s, B+7 scale (full scale = 128 m/s per count — actually
    //     AGC uses B+7 for *centisecond-unit* velocity; we use m/s directly here).
    // Note: AGC DP order is (high, low) = (most, least) significant.
    {
        let [rx, ry, rz] = state.csm_state.position;
        let [vx, vy, vz] = state.csm_state.velocity;

        // RN (B+29): pairs 1-3 = RN+2/+3, RN+4/+5, VN/+1 (snapshot order)
        // Order from CMPOWE01 snapshot: RN+2,+3, RN+4,+5, VN,+1, VN+2,+3, VN+4,+5, PIPTIME/+1, RN,+1
        let (ry_h, ry_l) = encode_dp(ry, 29); pair(1, ry_h, ry_l);
        let (rz_h, rz_l) = encode_dp(rz, 29); pair(2, rz_h, rz_l);
        let (vx_h, vx_l) = encode_dp(vx, 7);  pair(3, vx_h, vx_l);
        let (vy_h, vy_l) = encode_dp(vy, 7);  pair(4, vy_h, vy_l);
        let (vz_h, vz_l) = encode_dp(vz, 7);  pair(5, vz_h, vz_l);
        // PIPTIME (same as state.time, B+28 cs): pair 6
        let (pt_h, pt_l) = encode_time(state.csm_state.epoch.0);
        pair(6, pt_h, pt_l);
        // RN,+1 (position X, B+29): pair from the buffer send — we send it in slot 7
        let (rx_h, rx_l) = encode_dp(rx, 29); pair(7, rx_h, rx_l);
    }

    // ── Pairs 8–11: Snapshot 2 — CDU angles ───────────────────────────────
    // CDUZ/CDUT, ADOT(0-5), CDUX/CDUY
    // CDU angles: B+0 (full scale = 1 revolution, but AGC uses half-revolution scale B-1)
    // AGC CDU: 1 full revolution = 2^15 counts; normalise to half-revolution (π rad) units
    // Our CduAngle: 1 count = 2π/65536 rad; full revolution = 2π rad
    // AGC scale for CDU: B-1 (half-revolutions), so 1.0 = half revolution = π rad
    {
        let cdu_z = state.current_cdu[2].to_radians() / core::f64::consts::PI;
        let cdu_x = state.current_cdu[0].to_radians() / core::f64::consts::PI;
        let cdu_y = state.current_cdu[1].to_radians() / core::f64::consts::PI;
        // CDUZ, CDUT (CDUT not tracked; send 0): pair 8
        pair(8, encode_agc15(cdu_z), 0);
        // ADOT (angular rate): pairs 9-11 — not tracked, send zero
        pair(9, 0, 0);
        pair(10, 0, 0);
        pair(11, 0, 0);
        // CDUX, CDUY: pair 12
        pair(12, encode_agc15(cdu_x), encode_agc15(cdu_y));
    }

    // ── Pairs 13–14: AK/RCSFLAGS, THETADX/Y/Z/GARBAGE ────────────────────
    // Attitude error (ERRORX/Y/Z) in half-revolution scale (B-1 π rad)
    {
        let ex = state.dap_state.attitude_error[0] / core::f64::consts::PI;
        let ey = state.dap_state.attitude_error[1] / core::f64::consts::PI;
        let ez = state.dap_state.attitude_error[2] / core::f64::consts::PI;
        pair(13, encode_agc15(ex), encode_agc15(ey));
        pair(14, encode_agc15(ez), 0);
    }

    // ── Pair 15: TIG (high/low, B+28 centiseconds) ────────────────────────
    if let Some(m) = state.pending_maneuver {
        let (th, tl) = encode_time(m.tig.0);
        pair(15, th, tl);
    }

    // ── Pairs 16–25: MARKDOWN, MARK2DWN, HAPOX, PACTOFF (zero) ────────────
    // HAPOX = apogee + perigee from R30: pairs 24–25
    if state.csm_state.epoch.0 != 0 && state.csm_state.frame == Frame::EarthInertial {
        let elements = sv_to_elements(state.csm_state);
        if !elements.is_hyperbolic() {
            let apo_m = apoapsis_altitude_earth(&elements);
            let peri_m = periapsis_altitude_earth(&elements);
            // B+29 metres (same as position)
            let (apo_h, apo_l) = encode_dp(apo_m, 29);
            let (per_h, per_l) = encode_dp(peri_m, 29);
            pair(24, apo_h, apo_l);
            pair(25, per_h, per_l);
        }
    }

    // ── Pairs 30–35: REFSMMAT (6DNADR = first 6 DP elements) ────────────
    // REFSMMAT is 9 DP elements (rows [0][0..2], [1][0..2], [2][0..2]).
    // 6DNADR sends the first 6 DPs = first 6 elements.  Each DP occupies
    // one word-pair: (high_word, low_word) of the element.
    // Scale: B-0 (direction cosines, range ≈ [−1, 1)).
    {
        let m = state.refsmmat;
        let elements = [
            m[0][0], m[0][1], m[0][2],
            m[1][0], m[1][1], m[1][2],
        ];
        for (i, &e) in elements.iter().enumerate() {
            let (h, l) = encode_dp(e, 0);
            pair(30 + i, h, l);
        }
    }

    // ── Pairs 36–50: FLAGWRDS 0–9 (2 per pair) + display tables ─────────
    for i in 0..6 {
        let fw0 = state.flagwords[2 * i] & 0x7FFF;
        let fw1 = state.flagwords[2 * i + 1] & 0x7FFF;
        pair(36 + i, fw0, fw1);
    }

    // ── Pair 51: TIME2 / TIME1 (mission elapsed time) ─────────────────────
    {
        let (t2, t1) = encode_time(state.time.0);
        pair(51, t2, t1);
    }

    // ── Pair 52–62: R-OTHER/V-OTHER/T-OTHER (zero) ─────────────────────────
    // Target state not encoded in this implementation (zero is valid downlink).

    // ── Pairs 63–64: Repeat AK / THETAD (zero) ────────────────────────────

    // ── Pairs 65–70: CMCSTA06 — RSBBQ/CADRFLSH/FAILREG/CDUS/PIPA (zero) ──

    // ── Pairs 71–72: OGC/IGC/MGC — gyrocompass (zero until P02) ──────────

    // ── Pair 73: FLAGWRDS 10+11 ───────────────────────────────────────────
    {
        let fw10 = state.flagwords.get(10).copied().unwrap_or(0) & 0x7FFF;
        let fw11 = state.flagwords.get(11).copied().unwrap_or(0) & 0x7FFF;
        pair(73, fw10, fw11);
    }

    // ── Pairs 77–83: CMCSTA07 — masses, DAPDATR, errors, channels ─────────
    {
        // ERRORX/Y/Z (half-revolution scale)
        let ex = state.dap_state.attitude_error[0] / core::f64::consts::PI;
        let ey = state.dap_state.attitude_error[1] / core::f64::consts::PI;
        let ez = state.dap_state.attitude_error[2] / core::f64::consts::PI;
        pair(79, encode_agc15(ex), encode_agc15(ey));
        pair(80, encode_agc15(ez), 0);
        // IMODES30 / IMODES33 — alarm codes (SP integer)
        let alarm = state.alarm.code & 0x7FFF;
        pair(83, alarm, 0);
    }

    // ── Pairs 84–99: DSPTAB (display tables — 12 words = 6 pairs) ────────
    // The DSKY display registers as packed digit/lamp patterns.
    // We encode: major mode (verb/noun), alarm code lit flag, and R1..R3 values.
    {
        let major = state.dsky.prog as u16 & 0x7F;
        let verb  = state.dsky.verb  as u16 & 0x7F;
        let noun  = state.dsky.noun  as u16 & 0x7F;
        pair(84, (major << 7) | verb, noun);
        // Alarm / lamp state as a bitmask
        let lamp_bits: u16 = (state.alarm.lit as u16)
            | ((state.dsky.opr_err    as u16) << 1)
            | ((state.dsky.gimbal_lock as u16) << 2)
            | ((state.dsky.no_att    as u16) << 3);
        pair(85, lamp_bits, 0);
    }

    buf
}

// ── Downlink driver ───────────────────────────────────────────────────────────

/// Downlink driver: tracks position within the current 2-second cycle.
///
/// Each call to `downlink_step` sends one word-pair (two calls to
/// `hw.telemetry().send_word()`).  After 100 pairs the cycle resets and
/// a fresh downlist is built for the next 2-second window.
///
/// The driver is intended to be called from `t4rupt_step` every 20 ms
/// (matching the AGC's DOWNRUPT cadence).
#[derive(Clone, Copy, Debug)]
pub struct DownlinkDriver {
    /// Index of the next word-pair to send (0 = ID pair; resets to 0 each cycle).
    pub pair_index: usize,
    /// Pre-built downlist for the current 2-second window.  Rebuilt at the
    /// start of each new cycle (pair_index == 0).
    pub buffer: DownlistBuffer,
}

impl DownlinkDriver {
    pub const fn new() -> Self {
        Self {
            pair_index: 0,
            buffer: [0; DOWNLIST_WORDS],
        }
    }
}

impl Default for DownlinkDriver {
    fn default() -> Self {
        Self::new()
    }
}

/// Send one word-pair via `hw.telemetry()`.
///
/// Call once per 20 ms (DOWNRUPT cadence).  Rebuilds the downlist buffer at
/// the start of each 2-second cycle.
///
/// AGC source: `Comanche055/DOWN-TELEMETRY_PROGRAM.agc` — DODOWNTM handler.
pub fn downlink_step<T: crate::hal::Telemetry>(
    driver: &mut DownlinkDriver,
    state: &AgcState,
    telemetry: &mut T,
) {
    // Rebuild the downlist buffer at the start of each 2-second cycle.
    if driver.pair_index == 0 {
        driver.buffer = build_cmcstadl(state);
    }

    let k = driver.pair_index;
    let w34 = driver.buffer[2 * k];
    let w35 = driver.buffer[2 * k + 1];

    telemetry.send_word(w34);
    telemetry.send_word(w35);

    driver.pair_index = (driver.pair_index + 1) % DOWNLIST_PAIRS;
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
    ///
    /// One's complement of +16383 (0x3FFF): flip all 15 bits → 0x4000.
    #[test]
    fn tc_dl_3_minus_one_clamps_to_max_negative() {
        let word = encode_agc15(-1.0);
        assert_eq!(word, 0x4000, "expected 0x4000, got 0x{word:04X}");
    }

    /// TC-DL-4: +0.5 → word ≈ 0x2000 (8192 = 16383/2 rounded).
    #[test]
    fn tc_dl_4_half_positive() {
        let word = encode_agc15(0.5);
        // 0.5 × 16383 = 8191.5 → 8192 = 0x2000
        assert_eq!(word, 8192, "expected 8192, got {word}");
    }

    /// TC-DL-5: encode_sp with B+0 (unit scale) matches encode_agc15.
    #[test]
    fn tc_dl_5_encode_sp_b0() {
        let values = [0.0, 0.25, -0.5, 0.75];
        for v in values {
            assert_eq!(
                encode_sp(v, 0),
                encode_agc15(v),
                "encode_sp mismatch at {v}"
            );
        }
    }

    /// TC-DL-6: encode_dp zero → both words zero.
    #[test]
    fn tc_dl_6_dp_zero() {
        assert_eq!(encode_dp(0.0, 28), (0, 0));
    }

    /// TC-DL-7: encode_dp small positive value round-trips through decode.
    ///
    /// Encode 100 centiseconds in B+28, then decode and check within 1 cs
    /// tolerance.
    #[test]
    fn tc_dl_7_dp_round_trip_b28() {
        let time_cs = 1_000_u32; // 10 seconds
        let (hi, lo) = encode_dp(time_cs as f64, 28);
        // Decode: combined = (hi × 2^14 + lo) × 2^28 / 2^28
        let combined = ((hi as u32) << 14) | (lo as u32);
        assert!(
            (combined as i64 - time_cs as i64).abs() <= 1,
            "DP round-trip failed: encoded {combined}, expected {time_cs}"
        );
    }

    /// TC-DL-8: encode_time separates high and low 14-bit halves correctly.
    #[test]
    fn tc_dl_8_encode_time() {
        // 100 seconds = 10000 cs = 0x2710
        let t = 10_000_u32;
        let (t2, t1) = encode_time(t);
        // t1 = 10000 & 0x3FFF = 10000 - 8192 = 1808? No: 10000 = 0x2710
        // 0x2710 & 0x3FFF = 0x2710 = 10000 (since 10000 < 16384)
        // t2 = 10000 >> 14 = 0
        assert_eq!(t1, 10_000, "t1 wrong");
        assert_eq!(t2, 0, "t2 wrong");

        // 20000 cs: t2 = 1 (20000 >> 14 = 1), t1 = 20000 - 16384 = 3616
        let t = 20_000_u32;
        let (t2, t1) = encode_time(t);
        assert_eq!(t2, 1, "t2 wrong for 20000 cs");
        assert_eq!(t1, 20_000 - 16_384, "t1 wrong for 20000 cs");
    }

    // ── build_cmcstadl / downlink_step ───────────────────────────────────────

    /// TC-DL-9: ID pair (index 0) must be (CMCSTADL_ID, LOWIDCOD).
    #[test]
    fn tc_dl_9_id_pair() {
        let state = crate::AgcState::new();
        let buf = build_cmcstadl(&state);
        assert_eq!(buf[0], CMCSTADL_ID, "word 0 (channel 34) must be CMCSTADL_ID");
        assert_eq!(buf[1], LOWIDCOD, "word 1 (channel 35) must be LOWIDCOD");
    }

    /// TC-DL-10: TIME2/TIME1 at pair 51 matches encode_time(state.time.0).
    #[test]
    fn tc_dl_10_time_pair() {
        let mut state = crate::AgcState::new();
        state.time = Met(20_000);
        let buf = build_cmcstadl(&state);
        let (expected_t2, expected_t1) = encode_time(20_000);
        assert_eq!(buf[2 * 51], expected_t2, "TIME2 mismatch");
        assert_eq!(buf[2 * 51 + 1], expected_t1, "TIME1 mismatch");
    }

    /// TC-DL-11: Fresh-start state produces 200 words (no panic, correct length).
    #[test]
    fn tc_dl_11_fresh_start_buffer_length() {
        let state = crate::AgcState::new();
        let buf = build_cmcstadl(&state);
        assert_eq!(buf.len(), DOWNLIST_WORDS);
    }

    /// TC-DL-12: DownlinkDriver advances pair_index and resets after 100 pairs.
    #[test]
    fn tc_dl_12_driver_pair_index_cycles() {
        struct NullTelemetry;
        impl crate::hal::Telemetry for NullTelemetry {
            fn send_word(&mut self, _word: u16) {}
        }

        let state = crate::AgcState::new();
        let mut driver = DownlinkDriver::new();
        let mut tel = NullTelemetry;

        for step in 0..DOWNLIST_PAIRS {
            assert_eq!(driver.pair_index, step, "pair_index wrong at step {step}");
            downlink_step(&mut driver, &state, &mut tel);
        }
        // After 100 steps, must reset to 0.
        assert_eq!(driver.pair_index, 0, "pair_index must reset to 0 after full cycle");
    }

    /// TC-DL-13: LOWIDCOD constant equals octal 77340.
    ///
    /// Octal 77340 = 7×8^4 + 7×8^3 + 3×8^2 + 4×8 + 0 = 28672+3584+192+32 = 32480
    /// In 15-bit layout: 32480 & 0x7FFF = 32480 (0x7EE0). The constant must equal this.
    #[test]
    fn tc_dl_13_lowidcod_value() {
        let octal_77340: u16 = 7 * 4096 + 7 * 512 + 3 * 64 + 4 * 8;
        assert_eq!(LOWIDCOD, octal_77340, "LOWIDCOD must equal octal 77340 = {octal_77340}");
    }
}
