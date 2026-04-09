//! Linear algebra operations for navigation math.
//!
//! AGC source: Comanche055/INTERPRETER.agc — VLOAD, DOT, CROSS, UNIT, ABVAL
//! interpretive operators. Implemented here as plain Rust functions on `Vec3`
//! and `Mat3x3` (no interpreter VM, per ADR-001).

use crate::types::{Mat3x3, Vec3};

/// Dot product of two vectors.
///
/// AGC source: INTERPRETER.agc, DOT operator.
#[inline]
pub fn dot(a: &Vec3, b: &Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Cross product a × b.
///
/// AGC source: INTERPRETER.agc, CROSS operator.
#[inline]
pub fn cross(a: &Vec3, b: &Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Euclidean norm (magnitude).
///
/// AGC source: INTERPRETER.agc, ABVAL operator.
#[inline]
pub fn norm(v: &Vec3) -> f64 {
    libm::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
}

/// Unit vector. Returns the zero vector if the norm is below epsilon.
///
/// AGC source: INTERPRETER.agc, UNIT operator.
#[inline]
pub fn unit(v: &Vec3) -> Vec3 {
    let n = norm(v);
    if n < 1e-30 {
        [0.0; 3]
    } else {
        scale(v, 1.0 / n)
    }
}

/// Add two vectors.
#[inline]
pub fn add(a: &Vec3, b: &Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// Subtract: a − b.
#[inline]
pub fn sub(a: &Vec3, b: &Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// Scale a vector by scalar s.
#[inline]
pub fn scale(v: &Vec3, s: f64) -> Vec3 {
    [v[0] * s, v[1] * s, v[2] * s]
}

/// Multiply matrix by vector: M · v.
///
/// AGC source: INTERPRETER.agc, MXV interpretive operator.
#[inline]
pub fn mxv(m: &Mat3x3, v: &Vec3) -> Vec3 {
    crate::types::matrix::mxv(m, v)
}

/// Multiply two 3×3 matrices: A · B.
#[inline]
pub fn mxm(a: &Mat3x3, b: &Mat3x3) -> Mat3x3 {
    let mut result = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::matrix::IDENTITY;

    #[test]
    fn dot_orthogonal_is_zero() {
        let x = [1.0, 0.0, 0.0];
        let y = [0.0, 1.0, 0.0];
        assert_eq!(dot(&x, &y), 0.0);
    }

    #[test]
    fn dot_parallel_is_magnitude_squared() {
        let v = [3.0, 0.0, 0.0];
        assert_eq!(dot(&v, &v), 9.0);
    }

    #[test]
    fn cross_x_times_y_equals_z() {
        let x = [1.0, 0.0, 0.0];
        let y = [0.0, 1.0, 0.0];
        let z = cross(&x, &y);
        assert!((z[0]).abs() < 1e-15);
        assert!((z[1]).abs() < 1e-15);
        assert!((z[2] - 1.0).abs() < 1e-15);
    }

    #[test]
    fn norm_of_unit_vec_is_one() {
        let v = [1.0, 0.0, 0.0];
        assert!((norm(&v) - 1.0).abs() < 1e-15);
    }

    #[test]
    fn unit_of_zero_is_zero() {
        let z = [0.0; 3];
        assert_eq!(unit(&z), [0.0; 3]);
    }

    #[test]
    fn unit_preserves_direction() {
        let v = [3.0, 4.0, 0.0];
        let u = unit(&v);
        assert!((norm(&u) - 1.0).abs() < 1e-15);
        assert!((u[0] - 0.6).abs() < 1e-15);
        assert!((u[1] - 0.8).abs() < 1e-15);
    }

    #[test]
    fn mxm_identity() {
        let result = mxm(&IDENTITY, &IDENTITY);
        for i in 0..3 {
            for j in 0..3 {
                assert!((result[i][j] - IDENTITY[i][j]).abs() < 1e-15);
            }
        }
    }

    #[test]
    fn add_and_sub() {
        let a = [1.0, 2.0, 3.0];
        let b = [4.0, 5.0, 6.0];
        let s = add(&a, &b);
        assert_eq!(s, [5.0, 7.0, 9.0]);
        let d = sub(&b, &a);
        assert_eq!(d, [3.0, 3.0, 3.0]);
    }
}
