//! CDU angle newtype.
//!
//! AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
//! Registers: CDUX (octal 32), CDUY (octal 33), CDUZ (octal 34),
//!            CDUT (octal 35, optics trunnion), CDUS (octal 36, optics shaft).

use core::{
    fmt,
    ops::{Add, Neg, Sub},
};

/// CDU (Coupling Data Unit) gimbal angle.
///
/// Raw hardware value: `u16`, range 0..=65535.
/// Scale: 65536 counts = one full revolution (2π radians).
///
/// The AGC hardware uses 15-bit ones-complement internally; this newtype
/// stores the sign-extended, two's-complement equivalent (wrapping `u16`).
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
/// Registers: CDUX/Y/Z (octal 32-34), CDUT/S (octal 35-36).
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default, Debug)]
#[repr(transparent)]
pub struct CduAngle(pub u16);

/// Counts per full revolution.
const COUNTS_PER_REV: f64 = 65536.0;
/// 2π constant (full revolution in radians).
const TAU: f64 = core::f64::consts::TAU;

impl CduAngle {
    /// Construct from a raw hardware count word.
    ///
    /// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc CDU register format.
    pub const fn from_counts(raw: u16) -> Self {
        Self(raw)
    }

    /// Construct from radians, rounding to nearest count.
    ///
    /// Input is reduced modulo 2π before conversion.
    /// Scale: `counts = round(rad / 2π × 65536)` wrapping modulo 65536.
    ///
    /// AGC source: conversion used in IMU_MODE_SWITCHING_ROUTINES.agc SETCOARS.
    pub fn from_radians(rad: f64) -> Self {
        // Reduce to [0, 2π)
        let reduced = rad - TAU * libm::floor(rad / TAU);
        let counts = libm::round(reduced / TAU * COUNTS_PER_REV) as u64;
        Self((counts % 65536) as u16)
    }

    /// Convert to radians in the range [0, 2π).
    ///
    /// Scale: `radians = (counts as f64) × (2π / 65536)`.
    pub fn to_radians(self) -> f64 {
        (self.0 as f64) * (TAU / COUNTS_PER_REV)
    }

    /// Return the raw hardware count.
    pub const fn counts(self) -> u16 {
        self.0
    }

    /// Signed difference: `self - rhs` in wrapping i16 arithmetic.
    ///
    /// Returns a value in `[-32768, 32767]`.
    /// Used for CDU error computation (difference between commanded and actual angles).
    ///
    /// AGC source: error-counter logic in IMU_MODE_SWITCHING_ROUTINES.agc IMUCOARS.
    pub fn wrapping_diff(self, rhs: Self) -> i16 {
        self.0.wrapping_sub(rhs.0) as i16
    }

    /// Convert to degrees in the range [0, 360).
    pub fn to_degrees(self) -> f64 {
        (self.0 as f64) * (360.0 / COUNTS_PER_REV)
    }
}

impl Add for CduAngle {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self(self.0.wrapping_add(rhs.0))
    }
}

impl Sub for CduAngle {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self(self.0.wrapping_sub(rhs.0))
    }
}

impl Neg for CduAngle {
    type Output = Self;
    fn neg(self) -> Self {
        Self(self.0.wrapping_neg())
    }
}

impl fmt::Display for CduAngle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Format as degrees to one decimal place.
        let deg = self.to_degrees();
        // Manual formatting to avoid float Display in no_std.
        let deg_int = deg as u32;
        let frac = ((deg - deg_int as f64) * 10.0) as u32;
        write!(f, "{}.{}°", deg_int, frac)
    }
}

// ── tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_conversion() {
        // Test 1: zero count → 0 radians
        assert_eq!(CduAngle::from_counts(0).to_radians(), 0.0);
    }

    #[test]
    fn half_revolution() {
        // Test 2: 32768 counts ≈ π radians (within 1 ULP tolerance)
        let rad = CduAngle::from_counts(32768).to_radians();
        let pi = core::f64::consts::PI;
        assert!((rad - pi).abs() < 1e-4, "half rev = {rad}, expected ≈ {pi}");
    }

    #[test]
    fn wraparound_diff() {
        // Test 3: 0xFFFF.wrapping_diff(1) = (65535 - 1) as i16 = 65534 as i16 = -2
        let a = CduAngle(0xFFFF);
        let b = CduAngle(1);
        assert_eq!(a.wrapping_diff(b), -2);
    }

    #[test]
    fn coarse_align_tolerance() {
        // Test 4: 2° ≈ 364 counts (32768 counts per half-rev, 180°/half-rev)
        let angle = CduAngle::from_radians(2.0_f64.to_radians());
        let counts = angle.counts();
        // 2 × 32768 / 180 ≈ 364.1 → rounds to 364
        assert!(
            (counts as i32 - 364).abs() <= 2,
            "2° = {counts} counts, expected ≈ 364"
        );
    }

    #[test]
    fn round_trip() {
        // from_radians(to_radians(x)) round-trips with error ≤ 1 count
        let angle = CduAngle::from_counts(12345);
        let back = CduAngle::from_radians(angle.to_radians());
        assert!(
            (angle.0 as i32 - back.0 as i32).abs() <= 1,
            "round-trip error: {angle:?} → {back:?}"
        );
    }

    #[test]
    fn add_wrap() {
        let a = CduAngle(0xFFFE);
        let b = CduAngle(3);
        assert_eq!((a + b).0, 1);
    }

    #[test]
    fn neg_zero_is_zero() {
        assert_eq!((-CduAngle(0)).0, 0);
    }
}
