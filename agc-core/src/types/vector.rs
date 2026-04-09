//! Navigation vectors and delta-V newtype.
//!
//! AGC source: ERASABLE_ASSIGNMENTS.agc — RLS, RATT, VATT double-word vector cells.

/// 3-component vector for navigation math (position m, velocity m/s, etc.)
///
/// All components are `f64` in SI units. Use `math::linalg` functions for
/// dot product, cross product, norm, and rotation.
pub type Vec3 = [f64; 3];

/// Delta-V maneuver vector in meters per second.
///
/// Wraps a `Vec3` to distinguish a maneuver from a generic velocity vector.
///
/// AGC source: ERASABLE_ASSIGNMENTS.agc — DELVEET/DELVSIN delta-V cells.
#[derive(Clone, Copy, Debug, Default)]
pub struct DeltaV(pub Vec3);

impl DeltaV {
    pub const ZERO: Self = Self([0.0; 3]);

    /// Magnitude of the delta-V in m/s.
    #[inline]
    pub fn magnitude(self) -> f64 {
        let [x, y, z] = self.0;
        libm::sqrt(x * x + y * y + z * z)
    }
}
