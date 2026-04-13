//! Vec3 type alias and DeltaV newtype.
//!
//! AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
//! Usage: position (RN, RN1), velocity (VN, VN1), delta-V (DELVEET), PIPA readings.

use core::{
    fmt,
    ops::{Add, Mul, Neg, Sub},
};

/// 3-component double-precision vector.
///
/// Units and coordinate frame are context-dependent:
/// - Position: metres (ECI or body frame, documented at call sites)
/// - Velocity: m/s
/// - Delta-V: m/s
/// - Attitude rate: rad/s
///
/// This is a type alias, not a newtype.  Arithmetic ops live in `math::linalg`.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc (RN, VN, DELVEET registers).
pub type Vec3 = [f64; 3];

/// Construct a `Vec3` from three components.
///
/// AGC source: used throughout navigation math in Comanche055.
pub const fn vec3(x: f64, y: f64, z: f64) -> Vec3 {
    [x, y, z]
}

/// The zero vector.
///
/// AGC source: equivalent to `ZEROVECS` / `DCA ZEROVEC` pattern.
pub const ZERO_VEC3: Vec3 = [0.0, 0.0, 0.0];

/// PIPA scale factor: metres per second per count.
///
/// AGC source: Comanche055/SERVICER207.agc, `KPIP1 5.85 CM/SEC`.
/// 5.85 cm/s = 0.0585 m/s.
pub const PIPA_SCALE: f64 = 0.0585;

// ─────────────────────────────────────────────────────────────────────────────

/// A velocity change vector, in metres per second (ECI or body frame).
///
/// Wraps [`Vec3`] (`[f64; 3]`).  Units: m/s.
/// Used for SPS delta-V targets (DELVEET), accumulated PIPA delta-V, and
/// RCS burn commands.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc (DELVEET registers).
#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub struct DeltaV(pub Vec3);

impl DeltaV {
    /// Construct from three m/s components.
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self([x, y, z])
    }

    /// Construct from a `Vec3` already in m/s.
    pub const fn from_vec3(v: Vec3) -> Self {
        Self(v)
    }

    /// Return the underlying `Vec3`.
    pub const fn as_vec3(self) -> Vec3 {
        self.0
    }

    /// Magnitude (Euclidean norm) in m/s.
    ///
    /// Uses `libm::sqrt` for `no_std` compatibility.
    pub fn magnitude(self) -> f64 {
        let [x, y, z] = self.0;
        libm::sqrt(x * x + y * y + z * z)
    }

    /// Scale by a dimensionless factor (e.g., throttle fraction).
    pub fn scale(self, factor: f64) -> Self {
        let [x, y, z] = self.0;
        Self([x * factor, y * factor, z * factor])
    }

    /// Add two delta-V vectors (superposition).
    ///
    /// Prefer the `+` operator; this free method exists for `const`-context use.
    pub fn vec_add(self, other: Self) -> Self {
        let [ax, ay, az] = self.0;
        let [bx, by, bz] = other.0;
        Self([ax + bx, ay + by, az + bz])
    }
}

impl Add for DeltaV {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        self.vec_add(rhs)
    }
}

impl Sub for DeltaV {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        let [ax, ay, az] = self.0;
        let [bx, by, bz] = rhs.0;
        Self([ax - bx, ay - by, az - bz])
    }
}

impl Mul<f64> for DeltaV {
    type Output = Self;
    fn mul(self, factor: f64) -> Self {
        self.scale(factor)
    }
}

impl Neg for DeltaV {
    type Output = Self;
    fn neg(self) -> Self {
        let [x, y, z] = self.0;
        Self([-x, -y, -z])
    }
}

impl fmt::Display for DeltaV {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let [x, y, z] = self.0;
        write!(f, "[{x:.3}, {y:.3}, {z:.3}] m/s")
    }
}

// ── tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_vector() {
        assert_eq!(vec3(0.0, 0.0, 0.0), ZERO_VEC3);
    }

    #[test]
    fn indexing() {
        assert_eq!(vec3(1.0, 2.0, 3.0)[1], 2.0);
    }

    #[test]
    fn pipa_conversion() {
        // 100 PIPA counts × 0.0585 m/s/count = 5.85 m/s
        let dv = DeltaV::from_vec3([100_f64 * PIPA_SCALE, 0.0, 0.0]);
        let mag = dv.magnitude();
        assert!((mag - 5.85).abs() < 1e-9, "PIPA magnitude = {mag}");
    }

    #[test]
    fn magnitude_3_4_0() {
        // Test 1: magnitude of (3, 4, 0) = 5
        let dv = DeltaV::new(3.0, 4.0, 0.0);
        assert!((dv.magnitude() - 5.0).abs() < 1e-12);
    }

    #[test]
    fn scale_50x() {
        // Test 2: scale (1, 0, 0) by 50 → (50, 0, 0)
        let dv = DeltaV::new(1.0, 0.0, 0.0).scale(50.0);
        assert_eq!(dv, DeltaV::new(50.0, 0.0, 0.0));
    }

    #[test]
    fn pipa_magnitude_approx() {
        // Test 3: DeltaV from 100 PIPA counts in X axis ≈ 5.85 m/s
        let dv = DeltaV::from_vec3([100_f64 * 0.0585, 0.0, 0.0]);
        let mag = dv.magnitude();
        assert!((mag - 5.85).abs() < 1e-6, "mag = {mag}");
    }

    #[test]
    fn add_and_sub() {
        let a = DeltaV::new(1.0, 2.0, 3.0);
        let b = DeltaV::new(10.0, 0.0, -1.0);
        let sum = a + b;
        assert_eq!(sum, DeltaV::new(11.0, 2.0, 2.0));
        let diff = sum - b;
        assert_eq!(diff, a);
    }

    #[test]
    fn neg() {
        let dv = -DeltaV::new(1.0, -2.0, 3.0);
        assert_eq!(dv, DeltaV::new(-1.0, 2.0, -3.0));
    }
}
