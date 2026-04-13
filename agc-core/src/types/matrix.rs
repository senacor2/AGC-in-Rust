//! 3×3 matrix type alias and constructor helpers.
//!
//! AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
//! Usage: REFSMMAT (Reference Stable Member Matrix), coordinate frame transforms,
//!        W-matrix (state noise covariance in navigation filter).

/// 3×3 double-precision matrix.
///
/// Stored in row-major order: `mat[row][col]`.
/// Used for rotation matrices (REFSMMAT), coordinate-frame transforms,
/// and the navigation W-matrix (covariance).
///
/// All matrix arithmetic lives in `math::linalg`.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc (REFSMMAT registers).
pub type Mat3x3 = [[f64; 3]; 3];

/// 3×3 identity matrix.
///
/// AGC source: used as initial REFSMMAT when no alignment has been performed.
pub const IDENTITY_MAT3: Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// 3×3 zero matrix.
pub const ZERO_MAT3: Mat3x3 = [[0.0; 3]; 3];

/// Construct a `Mat3x3` from three row vectors.
pub const fn mat3(row0: [f64; 3], row1: [f64; 3], row2: [f64; 3]) -> Mat3x3 {
    [row0, row1, row2]
}

/// Matrix-vector multiply: `M × v`.
///
/// AGC source: used by navigation filter and coordinate transforms throughout
/// Comanche055 (e.g., REFSMMAT × ECI vector in SERVICER207.agc).
pub fn mat_vec_mul(m: Mat3x3, v: [f64; 3]) -> [f64; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

/// Transpose a 3×3 matrix.
///
/// AGC source: used in P51/P52 alignment routines (STAR_CATALOG routines).
pub fn transpose(m: Mat3x3) -> Mat3x3 {
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}

/// Matrix-matrix multiply: `A × B`.
///
/// AGC source: used for sequential rotation composition (REFSMMAT updates).
pub fn mat_mat_mul(a: Mat3x3, b: Mat3x3) -> Mat3x3 {
    let mut c = ZERO_MAT3;
    for i in 0..3 {
        for j in 0..3 {
            let mut s = 0.0_f64;
            for k in 0..3 {
                s += a[i][k] * b[k][j];
            }
            c[i][j] = s;
        }
    }
    c
}

// ── tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_transform() {
        // Test 1: I × v = v
        let v = [1.0, 2.0, 3.0];
        let result = mat_vec_mul(IDENTITY_MAT3, v);
        assert_eq!(result, v);
    }

    #[test]
    fn transpose_involution() {
        // Test 2: transpose(transpose(R)) == R
        let r: Mat3x3 = [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        let tt = transpose(transpose(r));
        for i in 0..3 {
            for j in 0..3 {
                assert!((tt[i][j] - r[i][j]).abs() < 1e-15);
            }
        }
    }

    #[test]
    fn orthogonality_identity() {
        // Test 3: R × R^T ≈ I for the identity matrix (trivial case)
        let r = IDENTITY_MAT3;
        let rrt = mat_mat_mul(r, transpose(r));
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (rrt[i][j] - expected).abs() < 1e-10,
                    "rrt[{i}][{j}] = {} ≠ {expected}",
                    rrt[i][j]
                );
            }
        }
    }

    #[test]
    fn mat3_constructor() {
        let m = mat3([1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 3.0]);
        assert_eq!(m[1][1], 2.0);
    }

    #[test]
    fn mat_mat_mul_identity() {
        // I × A == A
        let a: Mat3x3 = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
        let result = mat_mat_mul(IDENTITY_MAT3, a);
        for i in 0..3 {
            for j in 0..3 {
                assert!((result[i][j] - a[i][j]).abs() < 1e-12);
            }
        }
    }
}
