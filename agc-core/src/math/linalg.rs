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
///
/// Replaces AGC opcode `UNIT`. Panics (→ restart) if `a` is the zero vector,
/// matching the AGC's ERROR-flag behaviour mapped to Rust's restart model.
#[inline]
pub fn unit(a: Vec3) -> Vec3 {
    let n = norm(a);
    assert!(n != 0.0, "unit: zero vector has no direction");
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

    fn assert_vec_near(a: Vec3, b: Vec3, eps: f64) {
        for i in 0..3 {
            assert!((a[i] - b[i]).abs() < eps, "component {i}: {} != {} (eps={eps})", a[i], b[i]);
        }
    }

    fn assert_mat_near(a: Mat3x3, b: Mat3x3, eps: f64) {
        for r in 0..3 {
            for c in 0..3 {
                assert!((a[r][c] - b[r][c]).abs() < eps,
                    "[{r}][{c}]: {} != {} (eps={eps})", a[r][c], b[r][c]);
            }
        }
    }

    // ── dot (TC-DOT-1 through TC-DOT-5) ─────────────────────────────────────

    #[test]
    fn tc_dot_1_orthogonal() {
        assert_eq!(dot([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]), 0.0);
    }

    #[test]
    fn tc_dot_2_parallel() {
        assert_eq!(dot([2.0, 0.0, 0.0], [3.0, 0.0, 0.0]), 6.0);
    }

    #[test]
    fn tc_dot_3_general() {
        assert_eq!(dot([1.0, 2.0, 3.0], [4.0, 5.0, 6.0]), 32.0);
    }

    #[test]
    fn tc_dot_4_self_dot() {
        assert_eq!(dot([3.0, 4.0, 0.0], [3.0, 4.0, 0.0]), 25.0);
    }

    #[test]
    fn tc_dot_5_antiparallel() {
        assert_eq!(dot([1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]), -1.0);
    }

    // ── cross (TC-CROSS-1 through TC-CROSS-5) ──────────────────────────────

    #[test]
    fn tc_cross_1_right_hand_rule() {
        assert_vec_near(cross([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]), [0.0, 0.0, 1.0], 1e-15);
    }

    #[test]
    fn tc_cross_2_anticommutative() {
        let a = [1.0, 2.0, 3.0];
        let b = [4.0, 5.0, 6.0];
        let ab = cross(a, b);
        let ba = cross(b, a);
        assert_vec_near(ab, vscale(ba, -1.0), 1e-15);
    }

    #[test]
    fn tc_cross_3_parallel_zero() {
        assert_vec_near(cross([2.0, 0.0, 0.0], [5.0, 0.0, 0.0]), [0.0, 0.0, 0.0], 1e-15);
    }

    #[test]
    fn tc_cross_4_self_zero() {
        let a = [1.0, 2.0, 3.0];
        assert_vec_near(cross(a, a), [0.0, 0.0, 0.0], 1e-14);
    }

    #[test]
    fn tc_cross_5_perpendicular_magnitude() {
        // |a×b| = |a||b|sin(θ), for perpendicular unit vectors sin=1
        let c = cross([1.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
        assert!((norm(c) - 1.0).abs() < 1e-15);
    }

    // ── norm (TC-NORM-1 through TC-NORM-5) ──────────────────────────────────

    #[test]
    fn tc_norm_1_pythagorean() {
        assert!((norm([3.0, 4.0, 0.0]) - 5.0).abs() < 1e-14);
    }

    #[test]
    fn tc_norm_2_unit_vector() {
        assert!((norm([1.0, 0.0, 0.0]) - 1.0).abs() < 1e-15);
    }

    #[test]
    fn tc_norm_3_zero_vector() {
        assert_eq!(norm([0.0, 0.0, 0.0]), 0.0);
    }

    #[test]
    fn tc_norm_4_diagonal() {
        // |[1,1,1]| = sqrt(3) ≈ 1.7320508
        assert!((norm([1.0, 1.0, 1.0]) - libm::sqrt(3.0)).abs() < 1e-14);
    }

    #[test]
    fn tc_norm_5_leo_velocity() {
        // LEO velocity ≈ 7800 m/s mostly along one axis
        let v = [7700.0, 800.0, 100.0];
        let expected = libm::sqrt(7700.0 * 7700.0 + 800.0 * 800.0 + 100.0 * 100.0);
        assert!((norm(v) - expected).abs() < 1e-9);
    }

    // ── unit (TC-UNIT-1 through TC-UNIT-5) ──────────────────────────────────

    #[test]
    fn tc_unit_1_already_unit() {
        assert_vec_near(unit([1.0, 0.0, 0.0]), [1.0, 0.0, 0.0], 1e-15);
    }

    #[test]
    fn tc_unit_2_pythagorean() {
        assert_vec_near(unit([3.0, 4.0, 0.0]), [0.6, 0.8, 0.0], 1e-15);
    }

    #[test]
    fn tc_unit_3_diagonal() {
        let s = 1.0 / libm::sqrt(3.0);
        assert_vec_near(unit([1.0, 1.0, 1.0]), [s, s, s], 1e-14);
    }

    #[test]
    fn tc_unit_4_large_position() {
        assert_vec_near(unit([6_578_137.0, 0.0, 0.0]), [1.0, 0.0, 0.0], 1e-14);
    }

    #[test]
    #[should_panic(expected = "zero vector")]
    fn tc_unit_5_zero_panics() {
        let _ = unit([0.0, 0.0, 0.0]);
    }

    // ── vadd (TC-VADD-1 through TC-VADD-3) ─────────────────────────────────

    #[test]
    fn tc_vadd_1_zero_identity() {
        assert_eq!(vadd([1.0, 2.0, 3.0], [0.0, 0.0, 0.0]), [1.0, 2.0, 3.0]);
    }

    #[test]
    fn tc_vadd_2_commutative() {
        let a = [1.0, 2.0, 3.0];
        let b = [4.0, 5.0, 6.0];
        assert_eq!(vadd(a, b), vadd(b, a));
    }

    #[test]
    fn tc_vadd_3_general() {
        assert_eq!(vadd([1.0, -1.0, 0.5], [2.0, 3.0, -0.5]), [3.0, 2.0, 0.0]);
    }

    // ── vsub (TC-VSUB-1 through TC-VSUB-3) ─────────────────────────────────

    #[test]
    fn tc_vsub_1_self_zero() {
        let a = [1.0, 2.0, 3.0];
        assert_eq!(vsub(a, a), [0.0, 0.0, 0.0]);
    }

    #[test]
    fn tc_vsub_2_inverse_of_add() {
        let a = [1.0, 2.0, 3.0];
        let b = [4.0, 5.0, 6.0];
        assert_eq!(vsub(vadd(a, b), b), a);
    }

    #[test]
    fn tc_vsub_3_general() {
        assert_eq!(vsub([5.0, 3.0, 1.0], [2.0, 1.0, 4.0]), [3.0, 2.0, -3.0]);
    }

    // ── vscale (TC-VSCALE-1 through TC-VSCALE-3) ───────────────────────────

    #[test]
    fn tc_vscale_1_by_one() {
        assert_eq!(vscale([1.0, 2.0, 3.0], 1.0), [1.0, 2.0, 3.0]);
    }

    #[test]
    fn tc_vscale_2_by_zero() {
        assert_eq!(vscale([1.0, 2.0, 3.0], 0.0), [0.0, 0.0, 0.0]);
    }

    #[test]
    fn tc_vscale_3_negate() {
        assert_eq!(vscale([1.0, -2.0, 3.0], -1.0), [-1.0, 2.0, -3.0]);
    }

    // ── mxv (TC-MXV-1 through TC-MXV-3) ────────────────────────────────────

    #[test]
    fn tc_mxv_1_identity() {
        assert_eq!(mxv(IDENTITY, [1.0, 2.0, 3.0]), [1.0, 2.0, 3.0]);
    }

    #[test]
    fn tc_mxv_2_rotation_z90() {
        // 90° rotation about Z: x̂ → ŷ
        let rz90: Mat3x3 = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        assert_vec_near(mxv(rz90, [1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], 1e-15);
    }

    #[test]
    fn tc_mxv_3_scale_matrix() {
        let s: Mat3x3 = [[2.0, 0.0, 0.0], [0.0, 3.0, 0.0], [0.0, 0.0, 4.0]];
        assert_eq!(mxv(s, [1.0, 1.0, 1.0]), [2.0, 3.0, 4.0]);
    }

    // ── vxm (TC-VXM-1 through TC-VXM-3) ────────────────────────────────────

    #[test]
    fn tc_vxm_1_identity() {
        assert_eq!(vxm([1.0, 2.0, 3.0], IDENTITY), [1.0, 2.0, 3.0]);
    }

    #[test]
    fn tc_vxm_2_transpose_relation() {
        // vᵀ·M = (Mᵀ·v)ᵀ — for row vectors
        let m: Mat3x3 = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
        let v = [1.0, 0.0, 0.0];
        assert_vec_near(vxm(v, m), mxv(transpose(m), v), 1e-14);
    }

    #[test]
    fn tc_vxm_3_rotation() {
        let rz90: Mat3x3 = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        // vxm is effectively multiplying by Rᵀ (the inverse rotation)
        let result = vxm([0.0, 1.0, 0.0], rz90);
        assert_vec_near(result, [1.0, 0.0, 0.0], 1e-15);
    }

    // ── transpose (TC-TRN-1 through TC-TRN-3) ──────────────────────────────

    #[test]
    fn tc_trn_1_identity() {
        assert_eq!(transpose(IDENTITY), IDENTITY);
    }

    #[test]
    fn tc_trn_2_involution() {
        let m = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
        assert_eq!(transpose(transpose(m)), m);
    }

    #[test]
    fn tc_trn_3_swap_check() {
        let m = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
        let t = transpose(m);
        assert_eq!(t, [[1.0, 4.0, 7.0], [2.0, 5.0, 8.0], [3.0, 6.0, 9.0]]);
    }

    // ── mxm (TC-MXM-1 through TC-MXM-5) ────────────────────────────────────

    #[test]
    fn tc_mxm_1_left_identity() {
        let m = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
        assert_mat_near(mxm(IDENTITY, m), m, 1e-15);
    }

    #[test]
    fn tc_mxm_2_right_identity() {
        let m = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
        assert_mat_near(mxm(m, IDENTITY), m, 1e-15);
    }

    #[test]
    fn tc_mxm_3_rotation_compose() {
        // Two 90° Z-rotations = 180° Z-rotation
        let rz90: Mat3x3 = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        let rz180 = mxm(rz90, rz90);
        let expected: Mat3x3 = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]];
        assert_mat_near(rz180, expected, 1e-14);
    }

    #[test]
    fn tc_mxm_4_known_product() {
        let a = [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]; // Rx90
        let b = [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]]; // Ry90
        let result = mxm(a, b);
        // Rx90 * Ry90 expected:
        let expected = [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        assert_mat_near(result, expected, 1e-14);
    }

    #[test]
    fn tc_mxm_5_orthonormality() {
        // R * Rᵀ = I for a rotation matrix
        let rz90: Mat3x3 = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        assert_mat_near(mxm(rz90, transpose(rz90)), IDENTITY, 1e-14);
    }
}
