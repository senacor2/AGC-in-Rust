//! Trigonometric wrappers for `no_std` navigation math.
//!
//! AGC source: Comanche055/INTERPRETER.agc — SINE, COSINE, ASIN, ACOS
//! interpretive operators. All angles are in radians.
//!
//! Inputs to `asin`/`acos` are clamped to `[-1.0, 1.0]` to match the AGC
//! behaviour: the interpretive operators were defined for unit inputs; out-of-
//! range values were a program error but must not panic on bare metal.

/// sin(x) — x in radians.
///
/// AGC source: INTERPRETER.agc, SINE operator.
#[inline]
pub fn sin(x: f64) -> f64 {
    libm::sin(x)
}

/// cos(x) — x in radians.
///
/// AGC source: INTERPRETER.agc, COSINE operator.
#[inline]
pub fn cos(x: f64) -> f64 {
    libm::cos(x)
}

/// asin(x) — result in radians, clamped to [−π/2, π/2].
///
/// Input is clamped to [−1.0, 1.0] before calling libm.
///
/// AGC source: INTERPRETER.agc, ASIN operator.
#[inline]
pub fn asin(x: f64) -> f64 {
    libm::asin(x.clamp(-1.0, 1.0))
}

/// acos(x) — result in radians, clamped to [0, π].
///
/// Input is clamped to [−1.0, 1.0] before calling libm.
///
/// AGC source: INTERPRETER.agc, ACOS operator.
#[inline]
pub fn acos(x: f64) -> f64 {
    libm::acos(x.clamp(-1.0, 1.0))
}

/// atan2(y, x) — result in radians [−π, π].
#[inline]
pub fn atan2(y: f64, x: f64) -> f64 {
    libm::atan2(y, x)
}

/// Two-argument trig pair: returns (sin(x), cos(x)).
#[inline]
pub fn sincos(x: f64) -> (f64, f64) {
    (libm::sin(x), libm::cos(x))
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::{FRAC_PI_2, PI};

    #[test]
    fn sin_zero() {
        assert_eq!(sin(0.0), 0.0);
    }

    #[test]
    fn cos_zero() {
        assert_eq!(cos(0.0), 1.0);
    }

    #[test]
    fn asin_one_is_half_pi() {
        assert!((asin(1.0) - FRAC_PI_2).abs() < 1e-15);
    }

    #[test]
    fn acos_one_is_zero() {
        assert!((acos(1.0)).abs() < 1e-15);
    }

    #[test]
    fn acos_minus_one_is_pi() {
        assert!((acos(-1.0) - PI).abs() < 1e-15);
    }

    #[test]
    fn asin_clamped_above_one() {
        assert!((asin(1.5) - FRAC_PI_2).abs() < 1e-15);
    }

    #[test]
    fn acos_clamped_above_one() {
        assert!((acos(2.0)).abs() < 1e-15);
    }

    #[test]
    fn sincos_roundtrip() {
        let angle = 1.234;
        let (s, c) = sincos(angle);
        assert!((s - sin(angle)).abs() < 1e-15);
        assert!((c - cos(angle)).abs() < 1e-15);
        // Pythagorean identity
        assert!((s * s + c * c - 1.0).abs() < 1e-15);
    }
}
