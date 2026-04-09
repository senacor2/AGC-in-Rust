//! State vector and reference frame types.
//!
//! AGC source: ERASABLE_ASSIGNMENTS.agc — RLS, RATT, VATT, REFSMMAT.

use crate::types::matrix::{mxv, transpose, IDENTITY};
use crate::types::{Mat3x3, Met, Vec3};

/// Coordinate frame tag.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Frame {
    /// Earth-centered inertial (J2000-like, used for all navigation math).
    Eci,
    /// Body frame (vehicle axes).
    Body,
    /// Stable member (IMU platform) frame.
    StableMember,
}

/// Position + velocity state vector in a named coordinate frame.
///
/// AGC source: ERASABLE_ASSIGNMENTS.agc — RATT (position), VATT (velocity),
/// in units of 2^28 m and 2^7 m/s respectively. Here stored as f64 SI.
#[derive(Clone, Copy, Debug)]
pub struct StateVector {
    pub frame: Frame,
    /// Position in metres.
    pub r: Vec3,
    /// Velocity in m/s.
    pub v: Vec3,
    /// Mission elapsed time at epoch (centiseconds).
    pub t: Met,
}

impl StateVector {
    /// Zero state vector in ECI frame.
    pub const fn zero_eci() -> Self {
        Self {
            frame: Frame::Eci,
            r: [0.0; 3],
            v: [0.0; 3],
            t: Met::ZERO,
        }
    }
}

/// Reference-to-stable-member matrix (REFSMMAT).
///
/// 3×3 rotation matrix mapping ECI to the IMU stable member frame.
///
/// AGC source: ERASABLE_ASSIGNMENTS.agc — REFSMMAT 18-word double-precision block.
#[derive(Clone, Copy, Debug)]
pub struct Refsmmat(pub Mat3x3);

impl Refsmmat {
    pub const IDENTITY: Self = Self(IDENTITY);

    /// Rotate a vector from ECI to stable-member frame.
    #[inline]
    pub fn eci_to_sm(&self, v: &Vec3) -> Vec3 {
        mxv(&self.0, v)
    }

    /// Rotate a vector from stable-member to ECI frame.
    ///
    /// For a rotation matrix, the inverse equals the transpose.
    #[inline]
    pub fn sm_to_eci(&self, v: &Vec3) -> Vec3 {
        mxv(&transpose(&self.0), v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_eci_frame() {
        let sv = StateVector::zero_eci();
        assert_eq!(sv.frame, Frame::Eci);
        assert_eq!(sv.r, [0.0; 3]);
        assert_eq!(sv.v, [0.0; 3]);
        assert_eq!(sv.t, Met::ZERO);
    }

    #[test]
    fn refsmmat_identity_roundtrip() {
        let rsm = Refsmmat::IDENTITY;
        let v = [1.0, 2.0, 3.0];
        let sm = rsm.eci_to_sm(&v);
        let back = rsm.sm_to_eci(&sm);
        for i in 0..3 {
            assert!((back[i] - v[i]).abs() < 1e-14);
        }
    }

    #[test]
    fn refsmmat_rotation_roundtrip() {
        // 90-degree rotation about Z
        let mat: Mat3x3 = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        let rsm = Refsmmat(mat);
        let v = [1.0, 0.0, 0.0];
        let sm = rsm.eci_to_sm(&v);
        let back = rsm.sm_to_eci(&sm);
        for i in 0..3 {
            assert!((back[i] - v[i]).abs() < 1e-14, "component {i}: {}", back[i]);
        }
    }
}
