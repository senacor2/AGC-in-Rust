//! Attitude error computation and maneuver steering.
//!
//! Computes the angular error between current and commanded CDU angles and
//! implements the phase-plane switching logic that decides whether to fire
//! RCS jets, and in which direction.
//!
//! AGC source: KALCMANU_STEERING.agc — NEWDELHI, maneuver rotation steering.
//!             RCS-CSM_DIGITAL_AUTOPILOT.agc — phase-plane jet-on/jet-off logic.

use core::f64::consts::PI;
use libm::fmod;

/// Compute per-axis attitude error between current and commanded CDU angles.
///
/// Each element of the returned array is the signed error in radians, wrapped
/// to `[-π, π]`. Wrapping is necessary because CDU counters can roll over.
///
/// AGC source: KALCMANU_STEERING.agc — NEWDELHI attitude difference routine.
pub fn attitude_error(current: &[f64; 3], commanded: &[f64; 3]) -> [f64; 3] {
    let mut err = [0.0f64; 3];
    for i in 0..3 {
        err[i] = wrap_to_pi(commanded[i] - current[i]);
    }
    err
}

/// Phase-plane switching logic: given attitude error and body rate, determine
/// whether to fire jets and in which direction.
///
/// Returns:
/// - `+1`  fire jets to increase the angle (positive torque)
/// - `-1`  fire jets to decrease the angle (negative torque)
/// -  `0`  no jet command (inside deadband / coasting region)
///
/// The switching curve is a simple rate-limited deadband:
/// - If `|error| > deadband` and the rate is not already driving toward zero
///   at `rate_limit`, command a jet fire.
/// - If the rate exceeds `rate_limit` in the wrong direction, override.
///
/// AGC source: RCS-CSM_DIGITAL_AUTOPILOT.agc — phase-plane jet-on boundary.
pub fn phase_plane(error: f64, rate: f64, deadband: f64, rate_limit: f64) -> i8 {
    // Outside the deadband: drive the error toward zero.
    if error > deadband {
        // Positive error — need negative torque unless rate already negative enough.
        if rate > -rate_limit {
            return -1;
        }
    } else if error < -deadband {
        // Negative error — need positive torque unless rate already positive enough.
        if rate < rate_limit {
            return 1;
        }
    }
    // Inside deadband: damp any rate that exceeds the rate limit.
    if error.abs() <= deadband {
        if rate > rate_limit {
            return -1;
        }
        if rate < -rate_limit {
            return 1;
        }
    }
    0
}

/// Wrap angle to `(-π, π]`.
#[inline]
pub(crate) fn wrap_to_pi(mut a: f64) -> f64 {
    // fmod brings into (-2π, 2π).
    a = fmod(a + PI, 2.0 * PI);
    if a < 0.0 {
        a += 2.0 * PI;
    }
    a - PI
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_error_when_aligned() {
        let err = attitude_error(&[1.0, 2.0, 3.0], &[1.0, 2.0, 3.0]);
        for e in err {
            assert!(e.abs() < 1e-12, "expected zero error, got {e}");
        }
    }

    #[test]
    fn error_wraps_positive() {
        // commanded = 0.1 rad past +π (i.e., -π + 0.1 equivalent),
        // current = 0.0 → error should be near +π, not -π.
        let current = [0.0f64; 3];
        let commanded = [PI + 0.1, 0.0, 0.0];
        let err = attitude_error(&current, &commanded);
        // wrap_to_pi(π + 0.1) = π + 0.1 - 2π = -π + 0.1 ≈ -3.04
        assert!(
            err[0].abs() < PI,
            "wrapped error must be in (-π, π], got {}",
            err[0]
        );
    }

    #[test]
    fn error_wraps_negative() {
        let current = [0.0f64; 3];
        let commanded = [-(PI + 0.1), 0.0, 0.0];
        let err = attitude_error(&current, &commanded);
        assert!(
            err[0].abs() < PI,
            "wrapped error must be in (-π, π], got {}",
            err[0]
        );
    }

    #[test]
    fn phase_plane_inside_deadband_zero_rate() {
        // Small error, no rate → no jet.
        assert_eq!(phase_plane(0.01, 0.0, 0.1, 0.035), 0);
    }

    #[test]
    fn phase_plane_positive_error_fires_negative() {
        // Large positive error, no rate → negative torque.
        assert_eq!(phase_plane(0.2, 0.0, 0.1, 0.035), -1);
    }

    #[test]
    fn phase_plane_negative_error_fires_positive() {
        // Large negative error, no rate → positive torque.
        assert_eq!(phase_plane(-0.2, 0.0, 0.1, 0.035), 1);
    }

    #[test]
    fn phase_plane_rate_already_correcting_no_jet() {
        // Large positive error but rate already strongly negative → no jet.
        assert_eq!(phase_plane(0.2, -0.1, 0.1, 0.035), 0);
    }

    #[test]
    fn phase_plane_excessive_rate_damped() {
        // Inside deadband but rate is too fast → damp it.
        assert_eq!(phase_plane(0.0, 0.1, 0.1, 0.035), -1);
        assert_eq!(phase_plane(0.0, -0.1, 0.1, 0.035), 1);
    }
}
