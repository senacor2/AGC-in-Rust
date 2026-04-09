//! CDU gimbal angle newtype.
//!
//! AGC source: ERASABLE_ASSIGNMENTS.agc — CDUX, CDUY, CDUZ CDU counters.

/// CDU gimbal angle as raw hardware counts.
///
/// Full revolution = 2^15 = 32768 counts (scale B-1 revolutions).
/// The CDU counts increase in the positive rotation direction; the
/// hardware counter is 15-bit unsigned, wrapping at 32768.
///
/// Conversion: `counts * (2π / 32768)` = radians.
///
/// AGC source: ERASABLE_ASSIGNMENTS.agc, CDU counter cells.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, Default)]
pub struct CduAngle(pub u16);

impl CduAngle {
    pub const ZERO: Self = Self(0);

    /// Convert raw counts to radians.
    ///
    /// Scale factor: 1 count = 2π / 32768 radians.
    #[inline]
    pub fn to_radians(self) -> f64 {
        (self.0 as f64) * (core::f64::consts::TAU / 32768.0)
    }

    /// Construct from radians, wrapping to [0, 2π).
    #[inline]
    pub fn from_radians(radians: f64) -> Self {
        let raw = radians * (32768.0 / core::f64::consts::TAU);
        // Manual rem_euclid: raw - floor(raw/m)*m, no libm needed for integer modulus
        let counts = (raw - libm::floor(raw / 32768.0) * 32768.0) as u16;
        Self(counts)
    }

    /// Signed difference in counts from `other` to `self`, wrapping.
    #[inline]
    pub fn signed_diff(self, other: Self) -> i16 {
        self.0.wrapping_sub(other.0) as i16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_radians() {
        assert_eq!(CduAngle::ZERO.to_radians(), 0.0);
    }

    #[test]
    fn full_revolution() {
        // 32768 wraps to 0 in u16; verify round-trip at half revolution
        let half = CduAngle(16384);
        let diff = (half.to_radians() - core::f64::consts::PI).abs();
        assert!(
            diff < 1e-9,
            "half revolution = π, got {}",
            half.to_radians()
        );
    }

    #[test]
    fn from_radians_round_trip() {
        let angle = CduAngle::from_radians(1.0);
        let diff = (angle.to_radians() - 1.0).abs();
        // Quantization error ≤ half LSB = π/32768 ≈ 9.6e-5 rad
        assert!(diff < 1e-4, "round-trip error: {}", diff);
    }

    #[test]
    fn signed_diff_wraps() {
        let a = CduAngle(1);
        let b = CduAngle(u16::MAX);
        // 1 - 65535 in i16 wrapping = 2 (since u16 is 16 bit, 1 - 65535 mod 65536 = 2)
        // Wait: 1u16.wrapping_sub(65535) = 2; as i16 = 2
        assert_eq!(a.signed_diff(b), 2);
    }
}
