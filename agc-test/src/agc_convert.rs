//! AGC fixed-point word to `f64` conversion utilities.
//!
//! The AGC used 15-bit ones-complement words. Navigation quantities were stored
//! as double-precision (DP) pairs of consecutive words, with a documented
//! B-scale exponent (B+N) defining the physical unit per LSB.
//!
//! # Scale factors from Comanche055 / `docs/AGC Symbolic Listing.md`
//!
//! | Quantity        | AGC symbol | B-scale | 1 DP LSB ≈          |
//! |-----------------|------------|---------|---------------------|
//! | Position (m)    | RN         | B+28    | 2²⁸ m ≈ 2.684×10⁸  |
//! | Velocity (m/s)  | VN         | B+7     | 2⁷ m/s = 128 m/s   |
//!
//! Note: those are the full-scale values. The DP LSB is:
//!
//! For a 29-bit combined word (sign + 28 bits), with full-scale = 2^scale:
//!   1 DP LSB = 2^scale / 2^28 = 2^(scale-28)
//!
//! Position LSB: 2^(28-28) = 1 m (1 metre per DP LSB)
//! Velocity LSB: 2^(7-28) = 2^(-21) ≈ 4.77×10⁻⁷ m/s (sub-millimetre per second)
//!
//! In practice the conversion functions below implement the formula from
//! `docs/testing.md §6` directly.

// ── Single-word conversion ────────────────────────────────────────────────────

/// Convert a raw 15-bit AGC word (stored as `u16`, sign-extended via `i16`)
/// to an `f64` physical value.
///
/// The AGC word is a 15-bit ones-complement integer. The most-significant bit
/// (bit 14) is the sign bit. Conversion steps:
///
/// 1. Reinterpret `raw` as `i16` (sign-extending from bit 14 to bit 15).
/// 2. Multiply by `2^scale` to obtain the physical value.
///
/// # Parameters
///
/// - `raw` — the 15-bit AGC word packed in a `u16` (bits 14:0 used, bit 15 ignored).
/// - `scale` — the B-scale exponent; physical value = raw_signed × 2^scale.
///
/// # Example
///
/// ```
/// use agc_test::agc_convert::from_agc_word;
/// // Word value 0x3FFF = 16383 (max positive 15-bit word) with scale = 0
/// assert_eq!(from_agc_word(0x3FFF, 0), 16383.0);
/// // Word 0x4000 interpreted as i16 = -16384 (most-negative ones-complement)
/// assert_eq!(from_agc_word(0x4000, 0), -16384.0);
/// ```
pub fn from_agc_word(raw: u16, scale: i8) -> f64 {
    // Sign-extend from bit 14 (the AGC sign bit) to get a signed i16 value.
    // Bits 14:0 are the ones-complement 15-bit word.
    let masked = (raw & 0x7FFF) as i32; // bits 14:0
    let signed = if masked & 0x4000 != 0 {
        // Bit 14 set → negative in ones-complement
        masked - 0x8000 // sign-extend: 0x4000..0x7FFF → -16384..-1
    } else {
        masked
    };
    (signed as f64) * (2.0_f64).powi(scale as i32)
}

// ── Double-word conversion ────────────────────────────────────────────────────

/// Convert a double-precision AGC word pair (high word, low word) to `f64`.
///
/// The DP format uses two consecutive 15-bit words to form a 29-bit mantissa.
/// The high word contains the 15 most-significant bits (including sign); the
/// low word contributes its lower 14 bits as the least-significant part.
///
/// Combination formula (from `docs/testing.md §6`):
/// ```text
/// combined = (hi as i32) << 14  |  (lo as i32 & 0x3FFF)
/// value    = combined * 2^scale
/// ```
///
/// # Parameters
///
/// - `hi`    — high AGC word (sign + upper 14 magnitude bits in bits 14:0).
/// - `lo`    — low AGC word (lower 14 bits in bits 13:0; sign bit ignored).
/// - `scale` — B-scale exponent for the DP pair.
///
/// # Example
///
/// ```
/// use agc_test::agc_convert::from_agc_dword;
/// // hi = 0x0001, lo = 0x0000 → combined = 1 << 14 = 16384
/// // With scale = 0: value = 16384.0
/// assert_eq!(from_agc_dword(0x0001, 0x0000, 0), 16384.0);
/// ```
pub fn from_agc_dword(hi: u16, lo: u16, scale: i8) -> f64 {
    // Sign-extend hi from bit 14 (AGC sign bit) to i32.
    let hi_masked = (hi & 0x7FFF) as i32;
    let hi_signed = if hi_masked & 0x4000 != 0 {
        hi_masked - 0x8000
    } else {
        hi_masked
    };
    let combined = (hi_signed << 14) | (lo as i32 & 0x3FFF);
    (combined as f64) * (2.0_f64).powi(scale as i32)
}

// ── Inverse (f64 → DP word pair) ─────────────────────────────────────────────

/// Convert a physical `f64` value to an AGC double-precision word pair.
///
/// This is the inverse of [`from_agc_dword`]. The result is the `(hi, lo)`
/// word pair such that `from_agc_dword(hi, lo, scale) ≈ value`.
///
/// The conversion is rounded to the nearest representable DP value. Overflow
/// (magnitude exceeding `2^(scale+14)` full-scale) results in saturation to
/// the maximum or minimum representable value.
///
/// # Parameters
///
/// - `value` — physical value to encode.
/// - `scale` — B-scale exponent (same convention as [`from_agc_dword`]).
///
/// # Returns
///
/// `(hi, lo)` — the high and low 15-bit AGC words (packed in `u16`, bits 14:0).
///
/// # Example
///
/// ```
/// use agc_test::agc_convert::{from_agc_dword, to_agc_dword};
/// let value = 12345.678_f64;
/// let (hi, lo) = to_agc_dword(value, 0);
/// let recovered = from_agc_dword(hi, lo, 0);
/// // Round-trip error is at most 1 DP LSB = 2^scale
/// assert!((recovered - value).abs() < 1.0);
/// ```
pub fn to_agc_dword(value: f64, scale: i8) -> (u16, u16) {
    // Compute the 29-bit integer representation: combined = round(value / 2^scale)
    let scaled = value / (2.0_f64).powi(scale as i32);
    let combined = scaled.round() as i32;

    // Clamp to 29-bit signed range [-2^28, 2^28 - 1]
    let max_val = (1_i32 << 28) - 1;
    let min_val = -(1_i32 << 28);
    let combined = combined.clamp(min_val, max_val);

    // Split into high (bits 28:14) and low (bits 13:0) parts.
    let hi_i = combined >> 14;           // upper 15 bits (includes sign extension)
    let lo_i = combined & 0x3FFF;        // lower 14 bits (always non-negative)

    // Pack into u16. The i32 hi value fits in i16 (15-bit range); cast preserves bits.
    let hi = hi_i as i16 as u16;
    let lo = lo_i as u16;

    (hi, lo)
}

// ── Unit tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── from_agc_word ──────────────────────────────────────────────────────────

    /// Zero input maps to zero regardless of scale.
    #[test]
    fn word_zero() {
        assert_eq!(from_agc_word(0x0000, 0), 0.0);
        assert_eq!(from_agc_word(0x0000, 28), 0.0);
        assert_eq!(from_agc_word(0x0000, -5), 0.0);
    }

    /// Maximum positive 15-bit word (0x3FFF = 16383) with scale 0.
    #[test]
    fn word_max_positive() {
        assert_eq!(from_agc_word(0x3FFF, 0), 16383.0);
    }

    /// Most-negative 15-bit word (0x4000 when reinterpreted as i16 = -16384) with scale 0.
    #[test]
    fn word_most_negative() {
        assert_eq!(from_agc_word(0x4000, 0), -16384.0);
    }

    /// Positive word with B+28 position scale — represents 1 metre (1 DP LSB).
    #[test]
    fn word_position_scale_b28() {
        // raw = 1, scale = 0  →  1 * 2^0 = 1.0
        assert_eq!(from_agc_word(0x0001, 0), 1.0);
        // raw = 1, scale = 28 →  1 * 2^28 = 268435456.0
        assert_eq!(from_agc_word(0x0001, 28), 268_435_456.0);
    }

    /// Negative scale factor (fractional values).
    #[test]
    fn word_negative_scale() {
        // raw = 2, scale = -1  →  2 * 0.5 = 1.0
        assert_eq!(from_agc_word(0x0002, -1), 1.0);
    }

    // ── from_agc_dword ─────────────────────────────────────────────────────────

    /// Zero word pair maps to zero.
    #[test]
    fn dword_zero() {
        assert_eq!(from_agc_dword(0x0000, 0x0000, 0), 0.0);
        assert_eq!(from_agc_dword(0x0000, 0x0000, 28), 0.0);
    }

    /// hi = 1, lo = 0 → combined = 1 << 14 = 16384; with scale 0 → 16384.0.
    #[test]
    fn dword_hi_one_lo_zero() {
        assert_eq!(from_agc_dword(0x0001, 0x0000, 0), 16384.0);
    }

    /// hi = 0, lo = 1 → combined = 1; with scale 0 → 1.0.
    #[test]
    fn dword_hi_zero_lo_one() {
        assert_eq!(from_agc_dword(0x0000, 0x0001, 0), 1.0);
    }

    /// Negative value: hi = 0xFFFF (-1 as i16) → combined = (-1) << 14 | 0 = -16384.
    #[test]
    fn dword_negative_hi() {
        let result = from_agc_dword(0xFFFF, 0x0000, 0);
        assert_eq!(result, -16384.0);
    }

    /// Position encoding round-trip: B+28 scale, 1-metre resolution.
    /// A position of 6_378_137 m (Earth radius) should round-trip within 1 m.
    #[test]
    fn dword_position_roundtrip_earth_radius() {
        let pos = 6_378_137.0_f64;  // R_EARTH in metres
        let (hi, lo) = to_agc_dword(pos, 0);
        let recovered = from_agc_dword(hi, lo, 0);
        assert!(
            (recovered - pos).abs() < 1.0,
            "Round-trip error for R_EARTH: {} - {} = {}",
            recovered, pos, recovered - pos
        );
    }

    // ── to_agc_dword ──────────────────────────────────────────────────────────

    /// Zero encodes as (0, 0).
    #[test]
    fn to_dword_zero() {
        assert_eq!(to_agc_dword(0.0, 0), (0, 0));
    }

    /// Round-trip: encode then decode should recover within 1 LSB.
    #[test]
    fn roundtrip_positive_value() {
        let val = 12345.678_f64;
        let (hi, lo) = to_agc_dword(val, 0);
        let recovered = from_agc_dword(hi, lo, 0);
        assert!(
            (recovered - val).abs() < 1.0,
            "Round-trip failed: {} vs {}",
            recovered, val
        );
    }

    /// Round-trip with negative value.
    #[test]
    fn roundtrip_negative_value() {
        let val = -98765.4_f64;
        let (hi, lo) = to_agc_dword(val, 0);
        let recovered = from_agc_dword(hi, lo, 0);
        assert!(
            (recovered - val).abs() < 1.0,
            "Round-trip failed for negative: {} vs {}",
            recovered, val
        );
    }

    /// Round-trip with B+28 position scale (R_EARTH-level magnitude, 1 m resolution).
    #[test]
    fn roundtrip_position_b28_scale() {
        // With scale=0, values up to 2^28 ≈ 268 Mm are representable with 1 m LSB.
        // GEO altitude position (42.164 Mm) should round-trip within 1 m.
        let pos = 42_164_000.0_f64;
        let (hi, lo) = to_agc_dword(pos, 0);
        let recovered = from_agc_dword(hi, lo, 0);
        assert!(
            (recovered - pos).abs() < 1.0,
            "GEO position round-trip failed: {} vs {}",
            recovered, pos
        );
    }

    /// Round-trip with velocity: typical LEO circular velocity ~7668 m/s, scale B+7.
    /// At scale 7, 1 LSB = 2^7 = 128 m/s, so this is a coarse encoding.
    /// With scale -14, we get sub-mm/s resolution (1 LSB = 2^(-14) ≈ 6.1e-5 m/s).
    #[test]
    fn roundtrip_velocity_fine_scale() {
        let vel = 7668.72_f64;
        let (hi, lo) = to_agc_dword(vel, -14);
        let recovered = from_agc_dword(hi, lo, -14);
        // At scale -14, 1 LSB = 2^(-14) ≈ 6.1e-5 m/s
        let lsb = (2.0_f64).powi(-14);
        assert!(
            (recovered - vel).abs() < lsb,
            "Velocity round-trip failed: {} vs {} (1 LSB = {})",
            recovered, vel, lsb
        );
    }

    /// Overflow clamp: value exceeding max 29-bit range should saturate.
    #[test]
    fn to_dword_overflow_clamps() {
        // Max representable with scale=0 is 2^28 - 1 = 268435455
        let huge = 1.0e15_f64;
        let (hi, lo) = to_agc_dword(huge, 0);
        let recovered = from_agc_dword(hi, lo, 0);
        let max_val = ((1_i32 << 28) - 1) as f64;
        assert_eq!(recovered, max_val, "Overflow should saturate to max");
    }

    /// Signed-zero: -0.0 should encode identically to +0.0.
    #[test]
    fn to_dword_negative_zero() {
        let (hi_pos, lo_pos) = to_agc_dword(0.0_f64, 0);
        let (hi_neg, lo_neg) = to_agc_dword(-0.0_f64, 0);
        assert_eq!((hi_pos, lo_pos), (hi_neg, lo_neg), "±0.0 must encode identically");
    }
}
