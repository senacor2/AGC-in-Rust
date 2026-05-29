//! Quaternion helpers for attitude representation.
//!
//! All quaternions use the **scalar-first `[w, x, y, z]`** layout
//! (inertial → body convention, consistent with the REFSMMAT-based
//! representation used throughout `agc-core`).
//!
//! These functions are pure (no side effects, no global state) and
//! allocation-free — safe for `#![no_std]` bare-metal targets.

use crate::types::Mat3x3;

/// Scalar-first quaternion `[w, x, y, z]`.
///
/// Convention: inertial → body.  A unit quaternion encodes a rotation;
/// the zero quaternion is meaningless and rejected by [`quat_normalise`].
pub type Quat = [f64; 4];

/// Normalise a scalar-first quaternion `[w, x, y, z]` to unit length.
///
/// Returns the input unchanged if its norm is already less than `1e-30`
/// (degenerate zero quaternion), preserving any caller-controlled recovery
/// path rather than panicking in flight software.
///
/// In `#[cfg(test)]` builds the degenerate case panics so that test code
/// catches programming errors early (a zero quaternion has no rotation
/// interpretation).
pub fn quat_normalise(q: Quat) -> Quat {
    let n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if n2 < 1e-60 {
        // Degenerate: no meaningful direction. In tests, panic so the
        // programmer knows immediately; in flight code, return unchanged.
        #[cfg(test)]
        panic!("quat_normalise: zero quaternion");
        #[cfg(not(test))]
        return q;
    }
    let n = libm::sqrt(n2);
    [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
}

/// Convert a scalar-first unit quaternion to a 3×3 rotation matrix.
///
/// The returned matrix `M` satisfies `v_body = M · v_inertial`, consistent
/// with the REFSMMAT convention used throughout `agc-core`.
pub fn quat_to_mat3x3(q: Quat) -> Mat3x3 {
    let [w, x, y, z] = q;
    let x2 = x * x;
    let y2 = y * y;
    let z2 = z * z;
    let wx = w * x;
    let wy = w * y;
    let wz = w * z;
    let xy = x * y;
    let xz = x * z;
    let yz = y * z;
    [
        [1.0 - 2.0 * (y2 + z2), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (x2 + z2), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (x2 + y2)],
    ]
}

/// Spherical-linear interpolation between two unit quaternions.
///
/// `alpha = 0.0` returns `q0`; `alpha = 1.0` returns `q1`.
/// Uses the shortest-arc convention (negates `q1` if `dot(q0, q1) < 0`).
pub fn quat_slerp(q0: Quat, q1: Quat, alpha: f64) -> Quat {
    let dot = q0[0] * q1[0] + q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3];
    // Choose the shorter arc.
    let (q1, dot) = if dot < 0.0 {
        ([-q1[0], -q1[1], -q1[2], -q1[3]], -dot)
    } else {
        (q1, dot)
    };
    // Clamp to avoid acos domain errors from floating-point rounding.
    let dot = dot.min(1.0);
    let theta = libm::acos(dot);
    if theta.abs() < 1e-10 {
        // Quaternions are nearly identical; linear interpolation is numerically safe.
        let w = 1.0 - alpha;
        return quat_normalise([
            w * q0[0] + alpha * q1[0],
            w * q0[1] + alpha * q1[1],
            w * q0[2] + alpha * q1[2],
            w * q0[3] + alpha * q1[3],
        ]);
    }
    let sin_theta = libm::sin(theta);
    let s0 = libm::sin((1.0 - alpha) * theta) / sin_theta;
    let s1 = libm::sin(alpha * theta) / sin_theta;
    [
        s0 * q0[0] + s1 * q1[0],
        s0 * q0[1] + s1 * q1[1],
        s0 * q0[2] + s1 * q1[2],
        s0 * q0[3] + s1 * q1[3],
    ]
}

/// Convert a rotation matrix to a scalar-first unit quaternion.
///
/// Uses Shepperd's branched method: picks the largest of
/// `1 + tr(M)`, `1 + 2·m[0][0] - tr(M)`, `1 + 2·m[1][1] - tr(M)`,
/// `1 + 2·m[2][2] - tr(M)` to avoid numerical issues near the
/// trace = -1 singularity (180° rotation).
///
/// The returned quaternion is normalised.  The round-trip invariant
/// `quat_to_mat3x3(quat_from_mat3x3(M)) ≈ M` holds for any
/// valid rotation matrix M.
pub fn quat_from_mat3x3(m: Mat3x3) -> Quat {
    let tr = m[0][0] + m[1][1] + m[2][2];

    // Four candidates: t = 4 × (w² | x² | y² | z²) respectively.
    let t0 = 1.0 + tr;                       // 4w²
    let t1 = 1.0 + 2.0 * m[0][0] - tr;      // 4x²
    let t2 = 1.0 + 2.0 * m[1][1] - tr;      // 4y²
    let t3 = 1.0 + 2.0 * m[2][2] - tr;      // 4z²

    // Pick the branch with the largest t so the extracted component has
    // maximum magnitude and we divide by the largest possible denominator.
    let q = if t0 >= t1 && t0 >= t2 && t0 >= t3 {
        // w is largest
        let s = 0.5 / libm::sqrt(t0);
        [
            0.25 / s,
            (m[2][1] - m[1][2]) * s,
            (m[0][2] - m[2][0]) * s,
            (m[1][0] - m[0][1]) * s,
        ]
    } else if t1 >= t2 && t1 >= t3 {
        // x is largest
        let s = 0.5 / libm::sqrt(t1);
        [
            (m[2][1] - m[1][2]) * s,
            0.25 / s,
            (m[0][1] + m[1][0]) * s,
            (m[0][2] + m[2][0]) * s,
        ]
    } else if t2 >= t3 {
        // y is largest
        let s = 0.5 / libm::sqrt(t2);
        [
            (m[0][2] - m[2][0]) * s,
            (m[0][1] + m[1][0]) * s,
            0.25 / s,
            (m[1][2] + m[2][1]) * s,
        ]
    } else {
        // z is largest
        let s = 0.5 / libm::sqrt(t3);
        [
            (m[1][0] - m[0][1]) * s,
            (m[0][2] + m[2][0]) * s,
            (m[1][2] + m[2][1]) * s,
            0.25 / s,
        ]
    };

    quat_normalise(q)
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_quat_near(a: Quat, b: Quat, eps: f64) {
        for i in 0..4 {
            assert!(
                (a[i] - b[i]).abs() < eps,
                "component {i}: {} vs {} (eps={eps})",
                a[i],
                b[i]
            );
        }
    }

    fn assert_mat_near(a: Mat3x3, b: Mat3x3, eps: f64) {
        for r in 0..3 {
            for c in 0..3 {
                assert!(
                    (a[r][c] - b[r][c]).abs() < eps,
                    "[{r}][{c}]: {} vs {} (eps={eps})",
                    a[r][c],
                    b[r][c]
                );
            }
        }
    }

    // ── quat_normalise ────────────────────────────────────────────────────────

    /// tc_quat_normalise_unit_norm
    ///
    /// `quat_normalise([2.0, 0.0, 0.0, 0.0])` must return `[1.0, 0.0, 0.0, 0.0]`
    /// and the L2 norm of the result must be 1.0 within 1e-15.
    #[test]
    fn tc_quat_normalise_unit_norm() {
        let q = quat_normalise([2.0, 0.0, 0.0, 0.0]);
        assert_eq!(q, [1.0, 0.0, 0.0, 0.0]);
        let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        assert!(
            (norm - 1.0).abs() < 1e-15,
            "L2 norm of normalised quaternion must be 1.0, got {norm}"
        );
    }

    /// tc_quat_normalise_negative_w_canonical
    ///
    /// `quat_normalise([-1.0, 0.0, 0.0, 0.0])` normalises to magnitude 1.0.
    /// The implementation does NOT impose a canonical sign flip (w >= 0);
    /// it divides uniformly by the norm, preserving the negative w.
    #[test]
    fn tc_quat_normalise_negative_w_canonical() {
        let q = quat_normalise([-1.0, 0.0, 0.0, 0.0]);
        assert_eq!(q, [-1.0, 0.0, 0.0, 0.0]);
        let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        assert!(
            (norm - 1.0).abs() < 1e-15,
            "L2 norm must be 1.0 regardless of sign, got {norm}"
        );
    }

    // ── quat_to_mat3x3 ────────────────────────────────────────────────────────

    /// tc_quat_to_mat3x3_identity
    ///
    /// Identity quaternion `[1, 0, 0, 0]` must produce the 3×3 identity matrix.
    #[test]
    fn tc_quat_to_mat3x3_identity() {
        let m = quat_to_mat3x3([1.0, 0.0, 0.0, 0.0]);
        for (i, row) in m.iter().enumerate() {
            for (j, &cell) in row.iter().enumerate() {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (cell - expected).abs() < 1e-15,
                    "identity quat → mat: m[{i}][{j}] should be {expected}, got {cell}",
                );
            }
        }
    }

    /// tc_quat_to_mat3x3_90deg_x_rotation
    ///
    /// Quaternion for 90° about +X: `[cos(45°), sin(45°), 0, 0]`.
    /// Applying M to `[0, 1, 0]` (inertial +Y) must yield `[0, 0, 1]`
    /// (body +Z) under the active rotation convention used by the implementation.
    #[test]
    fn tc_quat_to_mat3x3_90deg_x_rotation() {
        let angle = core::f64::consts::FRAC_PI_4; // 45° = half of 90°
        let q = [angle.cos(), angle.sin(), 0.0, 0.0];
        let m = quat_to_mat3x3(q);
        let v_in = [0.0_f64, 1.0, 0.0];
        let v_out = [
            m[0][0] * v_in[0] + m[0][1] * v_in[1] + m[0][2] * v_in[2],
            m[1][0] * v_in[0] + m[1][1] * v_in[1] + m[1][2] * v_in[2],
            m[2][0] * v_in[0] + m[2][1] * v_in[1] + m[2][2] * v_in[2],
        ];
        assert!(
            (v_out[0]).abs() < 1e-14,
            "X component should be ~0, got {}",
            v_out[0]
        );
        assert!(
            (v_out[1]).abs() < 1e-14,
            "Y component should be ~0, got {}",
            v_out[1]
        );
        assert!(
            (v_out[2] - 1.0).abs() < 1e-14,
            "Z component should be ~1, got {}",
            v_out[2]
        );
    }

    // ── quat_slerp ────────────────────────────────────────────────────────────

    /// tc_quat_slerp_endpoints_unchanged
    ///
    /// `slerp(q1, q2, 0.0) == q1` and `slerp(q1, q2, 1.0) == q2`
    /// within 1e-12 per component.
    #[test]
    fn tc_quat_slerp_endpoints_unchanged() {
        let q1 = [1.0_f64, 0.0, 0.0, 0.0];
        let angle = core::f64::consts::FRAC_PI_4;
        let q2 = [angle.cos(), angle.sin(), 0.0, 0.0];

        let at_0 = quat_slerp(q1, q2, 0.0);
        let at_1 = quat_slerp(q1, q2, 1.0);

        for i in 0..4 {
            assert!(
                (at_0[i] - q1[i]).abs() < 1e-12,
                "slerp(t=0) component {i}: expected {}, got {}",
                q1[i],
                at_0[i]
            );
        }
        for i in 0..4 {
            assert!(
                (at_1[i] - q2[i]).abs() < 1e-12,
                "slerp(t=1) component {i}: expected {}, got {}",
                q2[i],
                at_1[i]
            );
        }
    }

    // ── quat_from_mat3x3 ─────────────────────────────────────────────────────

    /// tc_quat_from_mat3x3_identity_roundtrip
    ///
    /// `quat_from_mat3x3(identity)` must return `[1, 0, 0, 0]` (or its
    /// negative, since both represent the identity rotation). The round-trip
    /// `quat_to_mat3x3(quat_from_mat3x3(I)) ≈ I` must hold within 1e-14.
    #[test]
    fn tc_quat_from_mat3x3_identity_roundtrip() {
        let identity: Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let q = quat_from_mat3x3(identity);
        // Either [1,0,0,0] or [-1,0,0,0] is acceptable (both represent identity).
        assert!(
            (q[0].abs() - 1.0).abs() < 1e-14,
            "w must be ±1 for identity, got w={}",
            q[0]
        );
        assert!(q[1].abs() < 1e-14, "x must be 0, got {}", q[1]);
        assert!(q[2].abs() < 1e-14, "y must be 0, got {}", q[2]);
        assert!(q[3].abs() < 1e-14, "z must be 0, got {}", q[3]);

        let m_rt = quat_to_mat3x3(q);
        assert_mat_near(m_rt, identity, 1e-14);
    }

    /// tc_quat_from_mat3x3_90deg_x_roundtrip
    ///
    /// A 90° rotation about X: `q = [cos45, sin45, 0, 0]`.
    /// `quat_to_mat3x3(q)` → M; `quat_from_mat3x3(M)` must recover q
    /// (up to sign) within 1e-12, and the round-trip matrix must equal M
    /// within 1e-12.
    #[test]
    fn tc_quat_from_mat3x3_90deg_x_roundtrip() {
        let angle = core::f64::consts::FRAC_PI_4;
        let q_ref = [angle.cos(), angle.sin(), 0.0, 0.0];
        let m = quat_to_mat3x3(q_ref);
        let q_rt = quat_from_mat3x3(m);
        // Allow sign flip (both represent the same rotation).
        let sign = if q_rt[0] * q_ref[0] >= 0.0 { 1.0 } else { -1.0 };
        assert_quat_near(
            [sign * q_rt[0], sign * q_rt[1], sign * q_rt[2], sign * q_rt[3]],
            q_ref,
            1e-12,
        );
        let m_rt = quat_to_mat3x3(q_rt);
        assert_mat_near(m_rt, m, 1e-12);
    }

    /// tc_quat_from_mat3x3_180deg_y_roundtrip
    ///
    /// 180° rotation about Y. This is the near-singular case where
    /// `tr(M) = -1`; Shepperd's branching must select the y-dominant branch.
    /// Round-trip matrix must equal M within 1e-12.
    #[test]
    fn tc_quat_from_mat3x3_180deg_y_roundtrip() {
        // Ry(180°): [[-1, 0, 0], [0, 1, 0], [0, 0, -1]]
        let m: Mat3x3 = [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]];
        let q = quat_from_mat3x3(m);
        // Unit quaternion
        let norm_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
        assert!(
            (norm_sq - 1.0).abs() < 1e-12,
            "quaternion must be unit norm, got norm²={norm_sq}"
        );
        let m_rt = quat_to_mat3x3(q);
        assert_mat_near(m_rt, m, 1e-12);
    }

    /// tc_quat_from_mat3x3_sign_convention_canonical
    ///
    /// `quat_from_mat3x3(identity)` must return a quaternion with `w >= 0`.
    /// Shepperd's method picks the `t0 = 1 + tr(I) = 4` branch (w-dominant)
    /// and computes `w = sqrt(t0)/2 = 1.0` — always positive.
    /// This test pins the sign branch against future algorithmic regressions.
    #[test]
    fn tc_quat_from_mat3x3_sign_convention_canonical() {
        let identity: Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let q = quat_from_mat3x3(identity);
        assert!(
            q[0] >= 0.0,
            "quat_from_mat3x3(identity): w must be >= 0 (canonical sign), got w={}",
            q[0]
        );
        assert!((q[0] - 1.0).abs() < 1e-12, "w must be 1.0, got {}", q[0]);
        assert!(q[1].abs() < 1e-12, "x must be 0, got {}", q[1]);
        assert!(q[2].abs() < 1e-12, "y must be 0, got {}", q[2]);
        assert!(q[3].abs() < 1e-12, "z must be 0, got {}", q[3]);
    }

    /// tc_quat_from_mat3x3_arbitrary_rotation
    ///
    /// Arbitrary rotation: q = normalise([0.5, -0.5, 0.3, 0.7]).
    /// M = quat_to_mat3x3(q); quat_from_mat3x3(M) must recover q (up to sign)
    /// and the round-trip matrix must equal M within 1e-12.
    #[test]
    fn tc_quat_from_mat3x3_arbitrary_rotation() {
        let q_ref = quat_normalise([0.5, -0.5, 0.3, 0.7]);
        let m = quat_to_mat3x3(q_ref);
        let q_rt = quat_from_mat3x3(m);
        let sign = if q_rt[0] * q_ref[0] >= 0.0 { 1.0 } else { -1.0 };
        assert_quat_near(
            [sign * q_rt[0], sign * q_rt[1], sign * q_rt[2], sign * q_rt[3]],
            q_ref,
            1e-12,
        );
        let m_rt = quat_to_mat3x3(q_rt);
        assert_mat_near(m_rt, m, 1e-12);
    }
}
