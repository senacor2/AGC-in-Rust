//! Trigonometric wrappers for the AGC navigation stack.
//!
//! All functions use `libm::*` exclusively — never `core::f64` methods — so
//! that the identical binary is produced for both host tests and the embedded
//! `thumbv7em-none-eabihf` target.
//!
//! `asin_clamped` and `acos_clamped` clamp input to `[-1.0, 1.0]` before
//! calling libm to avoid NaN on ULP overshoot, matching the ESCAPE/ESCAPE2
//! overflow-handling in the Block II AGC interpreter.
//!
//! AGC source: Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc  (ARCTRIG, CALCGA; pages 1357-1364)
//!             Comanche055/LATITUDE_LONGITUDE_SUBROUTINES.agc (ARCTAN, LAT-LONG; pages 1236-1242)
//!             Comanche055/GROUND_TRACKING_DETERMINATION_PROGRAM.agc (DDV ASIN; page ~459)
//!             Comanche055/P30-P37.agc                      (SL1 ARCCOS; page ~428)
//!             Comanche055/P51-P53.agc                      (ASIN DAD; page ~707)
//!             Comanche055/ERASABLE_ASSIGNMENTS.agc          (ESCAPE/ESCAPE2 aliases; lines 1560, 1565)

/// Sine of `theta` (radians).
///
/// Thin wrapper around `libm::sin`. Provided for consistency — all nav/guidance
/// code uses `math::trig::sin` rather than calling `libm` directly.
///
/// Input: theta in radians. Output: dimensionless, range `[-1.0, 1.0]`.
///
/// AGC opcode: `SIN` (B-1 input → B-1 output in AGC; Rust: radians → dimensionless).
/// AGC usage: `Comanche055/LATITUDE_LONGITUDE_SUBROUTINES.agc` LALOTORV (DLOAD SIN LAT, line ~127).
#[inline]
pub fn sin(theta: f64) -> f64 {
    libm::sin(theta)
}

/// Cosine of `theta` (radians).
///
/// Thin wrapper around `libm::cos`.
///
/// Input: theta in radians. Output: dimensionless, range `[-1.0, 1.0]`.
///
/// AGC opcode: `COS` (B-1 input → B-1 output in AGC).
/// AGC usage: `Comanche055/LATITUDE_LONGITUDE_SUBROUTINES.agc` LALOTORV (COS PDDL LONG, line ~132).
#[inline]
pub fn cos(theta: f64) -> f64 {
    libm::cos(theta)
}

/// Tangent of `theta` (radians).
///
/// Thin wrapper around `libm::tan`. Near ±π/2 the result is large but finite
/// (not NaN) for all representable `f64` values of `theta`.
///
/// Input: theta in radians. Output: dimensionless.
///
/// AGC opcode: Not a direct interpretive opcode; synthesised as SIN/COS in AGC code.
/// Provided for callers in `control/` and `guidance/` that need tangent directly.
#[inline]
pub fn tan(theta: f64) -> f64 {
    libm::tan(theta)
}

/// Arcsine with domain clamping.
///
/// Clamps `x` to `[-1.0, 1.0]` before calling `libm::asin`, then returns the
/// result in radians in the range `[-π/2, π/2]`.
///
/// The clamp is mandatory: AGC fixed-point dot products of unit vectors can
/// produce values slightly outside `[-1, 1]` due to ones-complement rounding.
/// In the Rust port, `f64` dot products of normalised vectors can similarly
/// overshoot by one ULP (e.g., 1.0000000000000002). Without clamping,
/// `libm::asin` returns NaN; with clamping it returns π/2 exactly.
///
/// Input: dimensionless (e.g., dot product of unit vectors), clamped to `[-1, 1]`.
/// Output: radians, range `[-π/2, π/2]`.
///
/// AGC source: `Comanche055/LATITUDE_LONGITUDE_SUBROUTINES.agc` ARCTAN (SR1 ASIN, line ~221);
///             `Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc` ARCTRIG / TRIG1 (ASIN, line ~118);
///             `Comanche055/GROUND_TRACKING_DETERMINATION_PROGRAM.agc` (DDV ASIN, line ~162);
///             `Comanche055/P51-P53.agc` (ASIN DAD, line ~707).
///             `Comanche055/ERASABLE_ASSIGNMENTS.agc` ESCAPE / ESCAPE2 (lines 1560, 1565) —
///             erasable switch words that handled domain overflow in the AGC interpreter.
/// AGC opcode: `ASIN` (with ESCAPE/ESCAPE2 overflow protection in interpreter).
#[inline]
pub fn asin_clamped(x: f64) -> f64 {
    // Clamp unconditionally: even exact ±1.0 is safe, and overshoot by ULP is caught.
    // AGC ESCAPE/ESCAPE2: the AGC interpreter saturated ASIN input to ±half-range on overflow.
    libm::asin(x.clamp(-1.0, 1.0))
}

/// Arccosine with domain clamping.
///
/// Clamps `x` to `[-1.0, 1.0]` before calling `libm::acos`, then returns the
/// result in radians in the range `[0, π]`.
///
/// Same domain-protection rationale as `asin_clamped`.
///
/// Input: dimensionless (e.g., dot product of unit vectors), clamped to `[-1, 1]`.
/// Output: radians, range `[0, π]`.
///
/// AGC source: `Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc` ARCTRIG (ACOS SIGN SINTH, line ~111);
///             `Comanche055/P30-P37.agc` (SL1 ARCCOS, line ~428);
///             `Comanche055/P51-P53.agc` comment (ARCCOS(OS1-OS2), line ~1092).
/// AGC opcode: `ACOS` (with ESCAPE/ESCAPE2 overflow protection in interpreter).
#[inline]
pub fn acos_clamped(x: f64) -> f64 {
    // Clamp unconditionally: even exact ±1.0 is safe, and overshoot by ULP is caught.
    libm::acos(x.clamp(-1.0, 1.0))
}

/// Two-argument arctangent: `atan2(y, x)` in radians, range `(-π, π]`.
///
/// Thin wrapper around `libm::atan2`. Handles all quadrants correctly including
/// the x=0 case (returns ±π/2). Returns 0.0 for the degenerate `atan2(0, 0)` case
/// (matching POSIX/IEEE 754 behaviour and the AGC ARCTANXX zero-result path).
///
/// Input: y, x dimensionless (or same units). Output: radians, range `(-π, π]`.
///
/// AGC source: `Comanche055/LATITUDE_LONGITUDE_SUBROUTINES.agc` ARCTAN routine (lines 209-236)
///             implements the equivalent of atan2(SINTH, COSTH), using ASIN with a
///             quadrant correction for the negative-cosine half-plane. The Rust
///             `atan2` wrapper replaces the entire ARCTAN subroutine.
/// AGC opcode: No single opcode; the AGC synthesised atan2 from ASIN + quadrant branches.
#[inline]
pub fn atan2(y: f64, x: f64) -> f64 {
    libm::atan2(y, x)
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::linalg::dot;
    use core::f64::consts::PI;

    /// TC-TRIG-01: Basic sin/cos values.
    #[test]
    fn sin_cos_basic() {
        assert_eq!(sin(0.0), 0.0);
        assert!(
            (sin(PI / 2.0) - 1.0).abs() < 1e-15,
            "sin(π/2)={}",
            sin(PI / 2.0)
        );
        assert!(sin(PI).abs() < 1e-15, "sin(π)={}", sin(PI));
        assert!((sin(3.0 * PI / 2.0) + 1.0).abs() < 1e-15);
        assert_eq!(cos(0.0), 1.0);
        assert!(cos(PI / 2.0).abs() < 1e-15, "cos(π/2)={}", cos(PI / 2.0));
        assert!((cos(PI) + 1.0).abs() < 1e-15, "cos(π)={}", cos(PI));
    }

    /// TC-TRIG-02: Pythagorean identity.
    #[test]
    fn pythagorean_identity() {
        for theta in [1.23456_f64, 5.678_f64] {
            let s = sin(theta);
            let c = cos(theta);
            let id = s * s + c * c;
            assert!((id - 1.0).abs() < 2e-15, "theta={theta}: sin²+cos²={id}");
        }
    }

    /// TC-TRIG-03: `asin_clamped` with exact ±1.0 inputs.
    #[test]
    fn asin_clamped_exact_boundaries() {
        assert!((asin_clamped(1.0) - PI / 2.0).abs() < 1e-15);
        assert!((asin_clamped(-1.0) + PI / 2.0).abs() < 1e-15);
        assert_eq!(asin_clamped(0.0), 0.0);
    }

    /// TC-TRIG-04: `asin_clamped` domain protection — overshoot by 1e-12.
    /// Critical test: without clamping, libm::asin(1.0 + 1e-12) returns NaN.
    #[test]
    fn asin_acos_clamped_overshoot() {
        assert!((asin_clamped(1.0 + 1e-12) - PI / 2.0).abs() < 1e-15);
        assert!((asin_clamped(-1.0 - 1e-12) + PI / 2.0).abs() < 1e-15);
        assert!((acos_clamped(1.0 + 1e-12) - 0.0).abs() < 1e-15);
        assert!((acos_clamped(-1.0 - 1e-12) - PI).abs() < 1e-15);
    }

    /// TC-TRIG-05: `atan2` quadrant correctness.
    #[test]
    fn atan2_quadrants() {
        assert_eq!(atan2(0.0, 1.0), 0.0);
        assert!((atan2(1.0, 0.0) - PI / 2.0).abs() < 1e-15);
        assert!((atan2(0.0, -1.0) - PI).abs() < 1e-15);
        assert!((atan2(-1.0, 0.0) + PI / 2.0).abs() < 1e-15);
        assert_eq!(
            atan2(0.0, 0.0),
            0.0,
            "degenerate atan2(0,0) should be 0 (AGC ARCTANXX)"
        );
    }

    /// TC-TRIG-06: AGC-derived example — flight-path angle.
    /// Derived from `Comanche055/GROUND_TRACKING_DETERMINATION_PROGRAM.agc` (page 459):
    /// `DDV ASIN # U(R).U(V)` computes `asin(dot(unit_r, unit_v))`.
    #[test]
    fn flight_path_angle() {
        // Circular orbit: velocity perpendicular to radius → flight-path angle = 0
        let unit_r = [1.0_f64, 0.0, 0.0];
        let unit_v = [0.0_f64, 1.0, 0.0];
        let gamma = asin_clamped(dot(&unit_r, &unit_v));
        assert!((gamma - 0.0).abs() < 1e-15, "gamma={gamma}");

        // Radial ascent: same direction → flight-path angle = π/2
        let unit_r2 = [1.0_f64, 0.0, 0.0];
        let unit_v2 = [1.0_f64, 0.0, 0.0];
        let gamma2 = asin_clamped(dot(&unit_r2, &unit_v2));
        assert!((gamma2 - PI / 2.0).abs() < 1e-15, "gamma2={gamma2}");
    }

    /// TC-TRIG-07: `tan` and its relation to sin/cos.
    #[test]
    fn tan_values() {
        assert!(
            (tan(PI / 4.0) - 1.0).abs() < 1e-14,
            "tan(π/4)={}",
            tan(PI / 4.0)
        );
        assert_eq!(tan(0.0), 0.0);
        assert!((tan(-PI / 4.0) + 1.0).abs() < 1e-14);
    }
}
