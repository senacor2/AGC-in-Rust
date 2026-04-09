//! 3×3 rotation matrix type alias.
//!
//! AGC source: ERASABLE_ASSIGNMENTS.agc — REFSMMAT inertial-to-stable-member rotation matrix.

use super::Vec3;

/// 3×3 rotation matrix (row-major: `mat[row][col]`).
///
/// Used for coordinate frame transforms (e.g., REFSMMAT, body-to-inertial).
/// Matrix-vector multiply and other operations live in `math::linalg`.
pub type Mat3x3 = [[f64; 3]; 3];

/// The identity matrix.
pub const IDENTITY: Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// Multiply a 3×3 matrix by a 3-vector (M · v).
///
/// AGC source: Comanche055/INTERPRETER.agc, MXV interpretive operator.
#[inline]
pub fn mxv(m: &Mat3x3, v: &Vec3) -> Vec3 {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

/// Transpose a 3×3 matrix.
#[inline]
pub fn transpose(m: &Mat3x3) -> Mat3x3 {
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_mxv() {
        let v = [1.0, 2.0, 3.0];
        let result = mxv(&IDENTITY, &v);
        assert_eq!(result, v);
    }

    #[test]
    fn transpose_identity() {
        let t = transpose(&IDENTITY);
        assert_eq!(t, IDENTITY);
    }

    #[test]
    fn mxv_known_rotation() {
        // 90-degree rotation about Z: x→y, y→-x, z→z
        let rot_z_90: Mat3x3 = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        let v = [1.0, 0.0, 0.0];
        let result = mxv(&rot_z_90, &v);
        let expected = [0.0, 1.0, 0.0];
        for i in 0..3 {
            assert!((result[i] - expected[i]).abs() < 1e-15);
        }
    }
}
