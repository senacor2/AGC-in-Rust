//! Linear algebra primitives for Vec3 and Mat3x3.
//!
//! These are plain functions, not methods. Call them as `linalg::dot(a, b)`.
//! They replace the AGC interpretive language's vector and matrix opcodes.

use crate::types::{Mat3x3, Vec3};

/// Dot product of two vectors.
#[inline]
pub fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Cross product a × b.
#[inline]
pub fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Euclidean norm (magnitude) of a vector.
#[inline]
pub fn norm(a: Vec3) -> f64 {
    libm::sqrt(dot(a, a))
}

/// Unit vector in the direction of `a`.
/// Panics (→ restart) if `a` is the zero vector.
#[inline]
pub fn unit(a: Vec3) -> Vec3 {
    let n = norm(a);
    [a[0] / n, a[1] / n, a[2] / n]
}

/// Vector addition a + b.
#[inline]
pub fn vadd(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// Vector subtraction a − b.
#[inline]
pub fn vsub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// Scalar multiplication s · a.
#[inline]
pub fn vscale(a: Vec3, s: f64) -> Vec3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

/// Matrix × vector: M · v (returns a column vector).
#[inline]
pub fn mxv(m: Mat3x3, v: Vec3) -> Vec3 {
    [dot(m[0], v), dot(m[1], v), dot(m[2], v)]
}

/// Vector × matrix: vᵀ · M (treats v as a row vector).
#[inline]
pub fn vxm(v: Vec3, m: Mat3x3) -> Vec3 {
    [
        v[0] * m[0][0] + v[1] * m[1][0] + v[2] * m[2][0],
        v[0] * m[0][1] + v[1] * m[1][1] + v[2] * m[2][1],
        v[0] * m[0][2] + v[1] * m[1][2] + v[2] * m[2][2],
    ]
}

/// Transpose of a 3×3 matrix.
#[inline]
pub fn transpose(m: Mat3x3) -> Mat3x3 {
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}

/// Matrix × matrix: A · B.
#[inline]
pub fn mxm(a: Mat3x3, b: Mat3x3) -> Mat3x3 {
    let bt = transpose(b);
    [
        [dot(a[0], bt[0]), dot(a[0], bt[1]), dot(a[0], bt[2])],
        [dot(a[1], bt[0]), dot(a[1], bt[1]), dot(a[1], bt[2])],
        [dot(a[2], bt[0]), dot(a[2], bt[1]), dot(a[2], bt[2])],
    ]
}

/// 3×3 identity matrix.
pub const IDENTITY: Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dot_orthogonal() {
        assert_eq!(dot([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]), 0.0);
    }

    #[test]
    fn cross_unit_vectors() {
        let c = cross([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
        assert!((c[0]).abs() < 1e-15);
        assert!((c[1]).abs() < 1e-15);
        assert!((c[2] - 1.0).abs() < 1e-15);
    }

    #[test]
    fn norm_unit() {
        assert!((norm([3.0, 4.0, 0.0]) - 5.0).abs() < 1e-14);
    }

    #[test]
    fn mxv_identity() {
        let v = [1.0, 2.0, 3.0];
        let r = mxv(IDENTITY, v);
        assert_eq!(r, v);
    }

    #[test]
    fn transpose_involution() {
        let m = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
        let tt = transpose(transpose(m));
        assert_eq!(tt, m);
    }
}
