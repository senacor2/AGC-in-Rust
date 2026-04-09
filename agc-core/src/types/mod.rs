//! Physical quantity newtypes and mathematical array aliases.
//!
//! Rules:
//! - Navigation/guidance math uses `f64` in SI units.
//! - Hardware I/O (CDU angles, PIPA counts, channel words) uses `u16`/`i16`.
//! - Newtypes prevent silent unit mix-ups at compile time.

pub mod angle;
pub mod matrix;
pub mod vector;

pub use angle::CduAngle;
pub use matrix::Mat3x3;
pub use vector::{DeltaV, Vec3};

/// Mission elapsed time in centiseconds.
///
/// Integer counter; wraps after ~497 days. Convert to `f64` seconds only at
/// call sites that need it for math: `met.to_secs()`.
///
/// AGC source: ERASABLE_ASSIGNMENTS.agc — TIME1/TIME2 double-word counter.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, Default)]
pub struct Met(pub u32);

impl Met {
    pub const ZERO: Self = Self(0);

    /// Convert to floating-point seconds.
    #[inline]
    pub fn to_secs(self) -> f64 {
        (self.0 as f64) * 0.01
    }

    /// Advance by `centiseconds`, saturating at `u32::MAX`.
    #[inline]
    pub fn advance(self, centiseconds: u32) -> Self {
        Self(self.0.saturating_add(centiseconds))
    }
}
