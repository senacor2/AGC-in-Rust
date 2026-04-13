//! Vector and matrix linear algebra primitives.
//!
//! Pure, no-alloc, no_std-safe wrappers replacing AGC interpretive opcodes
//! (DOT, VXV, UNIT, ABVAL, VXSC, VAD, VSU, MXV, MXM3) with plain `f64` functions.
//! All functions operate in SI units; no scale-factor bookkeeping.
//!
//! AGC source: Comanche055/CONIC_SUBROUTINES.agc      (GEOM, KEPLERN, GETX, LAMROUT)
//!             Comanche055/SERVICER207.agc             (CALCGRAV, CALCRVG, NORMLIZE)
//!             Comanche055/ORBITAL_INTEGRATION.agc     (OBLATE, INTGRATE, DIFEQ0)
//!             Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc (CALCGA)

use crate::types::{Mat3x3, Vec3};

/// Dot product of two 3-vectors.
///
/// Input: any units (but both must be in the same units).
/// Output: product of the input units (dimensionless for unit vectors).
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, GEOM (DOT SL1, line ~1191);
///             `Comanche055/SERVICER207.agc`, CALCGRAV (DOT PUSH UNITW, line ~752).
/// AGC opcode: `DOT` (double-precision, result at B-2 from B-1 inputs in original).
/// Rust: plain f64 — no scale factor.
#[inline]
pub fn dot(a: &Vec3, b: &Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Cross product a × b.
///
/// Input: any units. Output: same units as input (squared for unlike units).
/// Returns a zero Vec3 if `a` or `b` is zero.
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, GEOM (VXV VSL1, line ~1197);
///             `Comanche055/ORBITAL_INTEGRATION.agc`, OBLATE (VLOAD VXV, line ~345).
/// AGC opcode: `VXV`.
#[inline]
pub fn cross(a: &Vec3, b: &Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Euclidean norm (magnitude) of a vector.
///
/// Input: any units. Output: same units.
/// Uses `libm::sqrt` for `no_std` compatibility.
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, GEOM via ABVAL opcode (line ~1213);
///             `Comanche055/SERVICER207.agc`, CALCRVG (ABVAL, line ~478).
/// AGC opcode: `ABVAL` / `NORM` (normalise + return scale count).
#[inline]
pub fn norm(v: &Vec3) -> f64 {
    libm::sqrt(norm_sq(v))
}

/// Squared Euclidean norm (avoids sqrt).
///
/// Input: any units. Output: input units squared.
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, KEPLERN (VSQ / DOT self, line ~607).
/// AGC opcode: `VSQ`.
#[inline]
pub fn norm_sq(v: &Vec3) -> f64 {
    v[0] * v[0] + v[1] * v[1] + v[2] * v[2]
}

/// Normalise a vector to unit length.
///
/// Returns `None` if the input vector's norm is less than `f64::EPSILON`
/// (i.e., effectively zero). Never panics, never unwraps.
///
/// The threshold for "zero vector" is `norm_sq(v) < f64::EPSILON * f64::EPSILON`
/// (i.e., `norm < f64::EPSILON ≈ 2.2e-16`). This matches the AGC's
/// `UNIT BOV COLINEAR` pattern.
///
/// The original AGC `UNIT` opcode set the overflow indicator `OVFIND` for a
/// zero-vector input and returned a copy of the zero vector; the Rust port
/// uses `Option` instead to make the degenerate case explicit at compile time.
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, GEOM (UNIT BOV COLINEAR, line ~1203);
///             `Comanche055/SERVICER207.agc`, CALCGRAV (UNIT PUSH, line ~745);
///             `Comanche055/ORBITAL_INTEGRATION.agc`, OBLATE (UNIT, line ~351).
/// AGC opcode: `UNIT`.
#[inline]
pub fn unit(v: &Vec3) -> Option<Vec3> {
    let sq = norm_sq(v);
    if sq < f64::EPSILON * f64::EPSILON {
        return None;
    }
    let n = libm::sqrt(sq);
    Some([v[0] / n, v[1] / n, v[2] / n])
}

/// Scale a vector by a scalar factor.
///
/// Input: any units, scalar dimensionless. Output: same units as `v`.
///
/// AGC source: `Comanche055/SERVICER207.agc`, CALCGRAV (VXSC, line ~762);
///             `Comanche055/ORBITAL_INTEGRATION.agc`, DIFEQ+1 (VXSC, line ~661).
/// AGC opcode: `VXSC`.
#[inline]
pub fn scale(v: &Vec3, s: f64) -> Vec3 {
    [v[0] * s, v[1] * s, v[2] * s]
}

/// Component-wise vector addition.
///
/// Input/output: same units.
///
/// AGC source: `Comanche055/SERVICER207.agc`, CALCGRAV (VAD PUSH, line ~771);
///             `Comanche055/ORBITAL_INTEGRATION.agc`, GETRPSV (VSR VAD, line ~199).
/// AGC opcode: `VAD`.
#[inline]
pub fn add(a: &Vec3, b: &Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// Component-wise vector subtraction (a − b).
///
/// Input/output: same units.
///
/// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc`, DIFEQ0 (VSU XCHX,2, line ~186);
///             `Comanche055/ORBITAL_INTEGRATION.agc`, EARSPH (VSU ABVAL, line ~571).
/// AGC opcode: `VSU` / `BVSU`.
#[inline]
pub fn sub(a: &Vec3, b: &Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// Matrix-vector multiply: M × v.
///
/// Row-major convention: `result[i] = sum_j m[i][j] * v[j]`.
/// Input: `m` dimensionless (rotation), `v` any units. Output: same units as `v`.
///
/// AGC source: `Comanche055/SERVICER207.agc`, CALCRVG (VXM VSL1 REFSMMAT, line ~787).
/// AGC opcode: `MXV` (matrix × column vector); `VXM` is treated as M^T × v in
///             the AGC, equivalent to calling this function with `transpose(m)`.
#[inline]
pub fn mxv(m: &Mat3x3, v: &Vec3) -> Vec3 {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

/// Matrix-matrix multiply: A × B (row-major).
///
/// `result[i][j] = sum_k a[i][k] * b[k][j]`.
///
/// AGC source: `Comanche055/KALCMANU_STEERING.agc`, NEWANGL (AXC,1 AXC,2 MIS DEL; CALL MXM3,
///             line ~49-51) — composition of two 3×3 rotation matrices.
/// AGC opcode: `MXM3`.
#[inline]
pub fn mxm(a: &Mat3x3, b: &Mat3x3) -> Mat3x3 {
    let mut c = [[0.0_f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            c[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    c
}

/// Transpose a 3×3 matrix.
///
/// `result[i][j] = m[j][i]`.
///
/// AGC source: P51-P53.agc alignment routines; used to convert between SM↔NB
///             coordinate frames (TRG*SMNB / TRG*NBSM are transpose-of-rotation
///             applications in POWERED_FLIGHT_SUBROUTINES.agc lines ~162-228).
#[inline]
pub fn transpose(m: &Mat3x3) -> Mat3x3 {
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}

/// Elementary rotation matrix about the X axis by angle `theta` (radians).
///
/// ```text
/// Rx(θ) = | 1     0       0    |
///          | 0  cos θ  -sin θ  |
///          | 0  sin θ   cos θ  |
/// ```
///
/// Input: theta in radians. Output: dimensionless 3×3 rotation matrix.
///
/// AGC source: built implicitly by CDU-to-DCM conversions in AX*SR*T
///             (POWERED_FLIGHT_SUBROUTINES.agc, pages 1369-1371).
/// Uses `libm::sin` / `libm::cos`.
#[inline]
pub fn rotx(theta: f64) -> Mat3x3 {
    let c = libm::cos(theta);
    let s = libm::sin(theta);
    [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]
}

/// Elementary rotation matrix about the Y axis by angle `theta` (radians).
///
/// ```text
/// Ry(θ) = |  cos θ  0  sin θ |
///          |    0    1    0   |
///          | -sin θ  0  cos θ |
/// ```
///
/// Input: theta in radians. Output: dimensionless 3×3 rotation matrix.
///
/// AGC source: built implicitly by CDU-to-DCM conversions in AX*SR*T
///             (POWERED_FLIGHT_SUBROUTINES.agc, pages 1369-1371).
#[inline]
pub fn roty(theta: f64) -> Mat3x3 {
    let c = libm::cos(theta);
    let s = libm::sin(theta);
    [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]
}

/// Elementary rotation matrix about the Z axis by angle `theta` (radians).
///
/// ```text
/// Rz(θ) = | cos θ  -sin θ  0 |
///          | sin θ   cos θ  0 |
///          |   0       0    1 |
/// ```
///
/// Input: theta in radians. Output: dimensionless 3×3 rotation matrix.
///
/// AGC source: built implicitly by CDU-to-DCM conversions in AX*SR*T
///             (POWERED_FLIGHT_SUBROUTINES.agc, pages 1369-1371).
#[inline]
pub fn rotz(theta: f64) -> Mat3x3 {
    let c = libm::cos(theta);
    let s = libm::sin(theta);
    [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::IDENTITY_MAT3;
    use core::f64::consts::PI;

    /// TC-LINALG-01: Orthogonal basis cross products.
    /// Derived from `Comanche055/CONIC_SUBROUTINES.agc` GEOM (VXV VSL1).
    #[test]
    fn cross_basis_vectors() {
        let ex = [1.0_f64, 0.0, 0.0];
        let ey = [0.0_f64, 1.0, 0.0];
        let ez = [0.0_f64, 0.0, 1.0];

        let r = cross(&ex, &ey);
        assert!((r[0] - ez[0]).abs() < 1e-15, "ex×ey x={}", r[0]);
        assert!((r[1] - ez[1]).abs() < 1e-15, "ex×ey y={}", r[1]);
        assert!((r[2] - ez[2]).abs() < 1e-15, "ex×ey z={}", r[2]);

        let r2 = cross(&ey, &ez);
        assert!((r2[0] - ex[0]).abs() < 1e-15);

        let r3 = cross(&ez, &ex);
        assert!((r3[1] - ey[1]).abs() < 1e-15);

        let r4 = cross(&ex, &ex);
        assert!((r4[0]).abs() < 1e-15);
        assert!((r4[1]).abs() < 1e-15);
        assert!((r4[2]).abs() < 1e-15);
    }

    /// TC-LINALG-02: Dot product of unit vectors.
    #[test]
    fn dot_unit_vectors() {
        let ex = [1.0_f64, 0.0, 0.0];
        let ey = [0.0_f64, 1.0, 0.0];
        assert_eq!(dot(&ex, &ey), 0.0);
        assert_eq!(dot(&ex, &ex), 1.0);

        let v = [0.6_f64, 0.8, 0.0];
        let d = dot(&v, &v);
        assert!((d - 1.0).abs() < 1e-15, "dot self = {d}");
    }

    /// TC-LINALG-03: Norm of a 3-4-5 vector.
    #[test]
    fn norm_3_4_5() {
        let v = [3.0_f64, 4.0, 0.0];
        assert!((norm(&v) - 5.0).abs() < 1e-12, "norm={}", norm(&v));
        assert_eq!(norm(&[0.0_f64, 0.0, 0.0]), 0.0);
        assert_eq!(norm_sq(&v), 25.0);
    }

    /// TC-LINALG-04: Zero-vector `unit` returns `None`.
    /// Safety invariant: must not panic on zero input.
    #[test]
    fn unit_zero_vector() {
        assert!(
            unit(&[0.0_f64, 0.0, 0.0]).is_none(),
            "zero vector should be None"
        );
        // Below epsilon threshold
        assert!(unit(&[f64::MIN_POSITIVE * 0.5, 0.0, 0.0]).is_none());
        // Valid unit vector
        let u = unit(&[1.0_f64, 0.0, 0.0]).expect("should be Some for non-zero");
        assert!((u[0] - 1.0).abs() < 1e-15);
    }

    /// TC-LINALG-05: GEOM colinear detection.
    /// Derived from `Comanche055/CONIC_SUBROUTINES.agc` GEOM COLINEAR branch.
    #[test]
    fn colinear_detection() {
        let r1 = [1.0_f64, 0.0, 0.0];
        let u2 = [2.0_f64, 0.0, 0.0]; // parallel to r1
        let normal = cross(&r1, &u2);
        assert_eq!(normal, [0.0, 0.0, 0.0]);
        assert!(
            unit(&normal).is_none(),
            "colinear cross product must give None from unit"
        );
    }

    /// TC-LINALG-06: Matrix-vector multiply with identity.
    #[test]
    fn mxv_identity() {
        let v = [3.0_f64, -1.0, 7.0];
        let result = mxv(&IDENTITY_MAT3, &v);
        assert_eq!(result, v);
    }

    /// TC-LINALG-07: Rotation matrix orthogonality.
    #[test]
    fn rotx_orthogonality() {
        let r = rotx(0.3);
        let rrt = mxm(&r, &transpose(&r));
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (rrt[i][j] - expected).abs() < 1e-14,
                    "rrt[{i}][{j}] = {} != {expected}",
                    rrt[i][j]
                );
            }
        }
    }

    /// TC-LINALG-08: 90° rotation of x-axis gives y-axis.
    #[test]
    fn rotz_ninety_degrees() {
        let rz90 = rotz(PI / 2.0);
        let result = mxv(&rz90, &[1.0_f64, 0.0, 0.0]);
        assert!((result[0]).abs() < 1e-15, "x={}", result[0]);
        assert!((result[1] - 1.0).abs() < 1e-15, "y={}", result[1]);
        assert!((result[2]).abs() < 1e-15, "z={}", result[2]);
    }

    /// TC-LINALG-09: `sub` is the left-inverse of `add`.
    #[test]
    fn add_sub_roundtrip() {
        let a = [1.0_f64, 2.0, 3.0];
        let b = [4.0_f64, -1.0, 0.0];
        let result = sub(&add(&a, &b), &b);
        for i in 0..3 {
            assert!(
                (result[i] - a[i]).abs() < 1e-15,
                "component {i}: {} != {}",
                result[i],
                a[i]
            );
        }
    }

    /// Roty and rotz orthogonality (extra coverage).
    #[test]
    fn roty_rotz_orthogonality() {
        for r in [roty(1.23), rotz(-0.77)] {
            let rrt = mxm(&r, &transpose(&r));
            for i in 0..3 {
                for j in 0..3 {
                    let expected = if i == j { 1.0 } else { 0.0 };
                    assert!(
                        (rrt[i][j] - expected).abs() < 1e-14,
                        "rrt[{i}][{j}] = {}",
                        rrt[i][j]
                    );
                }
            }
        }
    }

    /// Scale correctness.
    #[test]
    fn scale_vector() {
        let v = [2.0_f64, 3.0, -1.0];
        let s = scale(&v, 2.5);
        assert_eq!(s, [5.0, 7.5, -2.5]);
    }
}
