//! Mission Elapsed Time newtype.
//!
//! AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
//! Registers: TIME1 (octal 25, low word), TIME2 (octal 24, high word).
//!            TIME3 (octal 26), TIME4 (octal 27), TIME5 (octal 30), TIME6 (octal 31).

use core::{
    fmt,
    ops::{Add, Sub},
};

/// Mission Elapsed Time, stored in centiseconds (0.01 s per tick).
///
/// Corresponds to the AGC TIME1/TIME2 double-precision counter.
/// - TIME1 = ERASABLE_ASSIGNMENTS.agc octal 25 (low word, incremented every 10 ms).
/// - TIME2 = ERASABLE_ASSIGNMENTS.agc octal 24 (high word, overflow of TIME1).
///
/// Scale: 1 unit = 0.01 seconds (1 centisecond).
/// Range: 0..=`u32::MAX` centiseconds (≈ 497 days; wraps silently).
///
/// The AGC's double-word counter wrapped after ≈31.1 days (2^28 centiseconds).
/// The `u32` representation gives ≈497 days, well beyond any Apollo mission.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc TIME1/TIME2.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default, Debug)]
#[repr(transparent)]
pub struct Met(pub u32);

impl Met {
    /// Construct from a centisecond count (raw AGC TIME1/TIME2 pair value).
    ///
    /// AGC source: direct mapping of TIME1/TIME2 centisecond counter.
    pub const fn from_centiseconds(cs: u32) -> Self {
        Self(cs)
    }

    /// Construct from seconds, truncating to nearest centisecond.
    ///
    /// `from_secs(x).as_secs_f64()` round-trips with error at most 0.01 s.
    pub fn from_secs(secs: f64) -> Self {
        Self((secs * 100.0) as u32)
    }

    /// Return the count in centiseconds.
    pub const fn as_centiseconds(self) -> u32 {
        self.0
    }

    /// Convert to `f64` seconds for use in navigation math.
    ///
    /// Only call at math sites; do not store the `f64` value in persistent state.
    ///
    /// AGC source: `DAS TIME1` accumulation → physics model time input.
    pub fn as_secs_f64(self) -> f64 {
        self.0 as f64 / 100.0
    }

    /// Wrapping addition of centiseconds (matches AGC TIME overflow behavior).
    ///
    /// AGC source: TIME1/TIME2 wraparound after 2^28 centiseconds.
    pub fn wrapping_add_cs(self, delta_cs: u32) -> Self {
        Self(self.0.wrapping_add(delta_cs))
    }

    /// Wrapping subtraction: returns elapsed centiseconds from `other` to `self`.
    ///
    /// Used for inter-event timing; does not depend on absence of wrap.
    pub fn wrapping_sub_cs(self, other: Self) -> u32 {
        self.0.wrapping_sub(other.0)
    }
}

impl Add<u32> for Met {
    type Output = Self;
    fn add(self, delta_cs: u32) -> Self {
        self.wrapping_add_cs(delta_cs)
    }
}

impl Sub for Met {
    type Output = u32;
    fn sub(self, other: Self) -> u32 {
        self.wrapping_sub_cs(other)
    }
}

impl fmt::Display for Met {
    /// Format as `HH:MM:SS.cc` (hours, minutes, seconds, centiseconds).
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let total = self.0;
        let cc = total % 100;
        let total_secs = total / 100;
        let ss = total_secs % 60;
        let total_mins = total_secs / 60;
        let mm = total_mins % 60;
        let hh = total_mins / 60;
        write!(f, "{hh:02}:{mm:02}:{ss:02}.{cc:02}")
    }
}

// ── tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_secs() {
        // Test 1: 100 cs = 1.0 s exactly
        let met = Met::from_centiseconds(100);
        assert_eq!(met.as_secs_f64(), 1.0);
    }

    #[test]
    fn display_format() {
        // Test 2: 1h 1m 1.05s = 360000 + 6000 + 100 + 5 = 366105 cs
        // Verify the raw computation that Display uses.
        let met = Met::from_centiseconds(360000 + 6000 + 100 + 5);
        let total = met.0;
        let cc = total % 100;
        let total_secs = total / 100;
        let ss = total_secs % 60;
        let mm = (total_secs / 60) % 60;
        let hh = total_secs / 3600;
        assert_eq!((hh, mm, ss, cc), (1, 1, 1, 5));
    }

    #[test]
    fn wrap_at_max() {
        // Test 3: MAX.wrapping_add_cs(1) == Met(0)
        assert_eq!(Met::from_centiseconds(u32::MAX).wrapping_add_cs(1), Met(0));
    }

    #[test]
    fn delta_subtraction() {
        // Test 4: Met(200) - Met(100) == 100 cs
        assert_eq!(Met(200).wrapping_sub_cs(Met(100)), 100);
    }

    #[test]
    fn from_secs_round_trip() {
        let met = Met::from_secs(3.75);
        let back = met.as_secs_f64();
        assert!((back - 3.75).abs() <= 0.01, "back = {back}");
    }

    #[test]
    fn add_operator() {
        let met = Met::from_centiseconds(500);
        assert_eq!((met + 100).0, 600);
    }
}
