//! Trigonometric helpers.
//!
//! Thin wrappers around `libm` for `no_std` compatibility.
//! Angles are in radians throughout the guidance software.

/// Sine — angle in radians.
#[inline]
pub fn sin(a: f64) -> f64 {
    libm::sin(a)
}

/// Cosine — angle in radians.
#[inline]
pub fn cos(a: f64) -> f64 {
    libm::cos(a)
}

/// Arcsine — returns radians in [−π/2, π/2].
#[inline]
pub fn asin(x: f64) -> f64 {
    libm::asin(x)
}

/// Arccosine — returns radians in [0, π].
#[inline]
pub fn acos(x: f64) -> f64 {
    libm::acos(x)
}

/// Four-quadrant arctangent — returns radians in (−π, π].
#[inline]
pub fn atan2(y: f64, x: f64) -> f64 {
    libm::atan2(y, x)
}
