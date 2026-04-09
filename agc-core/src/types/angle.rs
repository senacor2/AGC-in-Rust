use super::vector::Vec3;

/// CDU (Coupling Data Unit) gimbal angle from the IMU or optics.
/// Stored as a raw u16 twos-complement count; full revolution = 2^15 counts
/// (scale factor B-1 revolutions, i.e. 1 count ≈ 0.0055°).
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct CduAngle(pub u16);

impl CduAngle {
    /// Convert to radians.
    pub fn to_radians(self) -> f64 {
        (self.0 as f64) * (core::f64::consts::TAU / 65536.0)
    }

    /// Convert to degrees.
    pub fn to_degrees(self) -> f64 {
        self.to_radians() * (180.0 / core::f64::consts::PI)
    }
}

impl core::fmt::Debug for CduAngle {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "CduAngle({:.4}°)", self.to_degrees())
    }
}

/// Mission elapsed time in centiseconds (1/100 s).
/// Integer counter; wraps after ~497 days.
/// Convert to f64 seconds only at call sites that need floating-point math.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default, Debug)]
pub struct Met(pub u32);

impl Met {
    /// Convert to seconds as f64 for use in navigation math.
    pub fn to_seconds(self) -> f64 {
        self.0 as f64 / 100.0
    }

    /// Construct from whole seconds (rounded to nearest centisecond).
    pub fn from_seconds(s: f64) -> Self {
        Met((s * 100.0) as u32)
    }

    /// Elapsed centiseconds between two MET values (saturating at u32::MAX).
    pub fn elapsed_since(self, earlier: Met) -> u32 {
        self.0.wrapping_sub(earlier.0)
    }
}

/// Delta-V vector in metres per second.
#[derive(Clone, Copy, Default, Debug)]
pub struct DeltaV(pub Vec3);
