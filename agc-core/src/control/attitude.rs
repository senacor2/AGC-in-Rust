//! Attitude error computation and phase-plane switching logic.
//!
//! Implements the RCSATT rate filter, attitude error computation, and the
//! linear phase-plane switching surface used to decide RCS jet firing direction.
//!
//! AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc
//!   RCSATT, RATEFILT, DRHOLOOP, ADOTLOOP, FREECHK, GETAKS, NEEDLER,
//!   MERUPDAT (pages 1002-1024).
//! AGC source: Comanche055/KALCMANU_STEERING.agc
//!   NEWDELHI, NEWANGL, INCRDCDU, MANUSTAT (pages 414-419).
//! AGC source: Comanche055/JET_SELECTION_LOGIC.agc
//!   PWORD (TAU1/TAU2 sign check, page 1039).

use crate::control::constants::SLOPE;
use crate::types::{CduAngle, Mat3x3};

/// Body-axis attitude error in radians.
///
/// Positive pitch = nose up, positive yaw = nose left (body-frame convention).
/// Corresponds to ERRORX (pitch), ERRORY (yaw), ERRORZ (roll) in AGC erasable.
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc MERUPDAT (p. 1020),
///             scaled 180 degrees full-scale (1 unit = π / 32768 rad).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttitudeError {
    /// Pitch error, radians (body frame). Positive = nose up.
    pub pitch: f64,
    /// Yaw error, radians (body frame). Positive = nose left.
    pub yaw: f64,
    /// Roll error, radians (body frame). Positive = roll right.
    pub roll: f64,
}

impl AttitudeError {
    /// Zero attitude error (no correction needed).
    pub const ZERO: Self = Self {
        pitch: 0.0,
        yaw: 0.0,
        roll: 0.0,
    };
}

/// Per-axis jet firing decision from the phase-plane switching function.
///
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc PWORD (TAU1 sign check, p. 1039);
///             RCS-CSM_DIGITAL_AUTOPILOT.agc T5PHASE2 TAU generation (p. 1017).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JetDecision {
    /// Fire positive-direction jets (+ torque on this axis).
    Positive,
    /// Fire negative-direction jets (− torque on this axis).
    Negative,
    /// Do not fire on this axis (inside deadband or rate already damping).
    None,
}

/// Compute body-axis attitude error from current CDU angles and desired attitude matrix.
///
/// The desired attitude matrix `desired_mat` is the REFSMMAT composed with the
/// target rotation (e.g., from KALCMANU `NEWANGL` / `MIS` matrix).
/// The function extracts the error angles by comparing the matrix-derived Euler
/// angles against the current CDU readings (converted to radians).
///
/// The CDU angles encode the gimbal position as [CDUX, CDUY, CDUZ] = [inner, middle, outer].
/// The attitude error is computed as the difference between the desired orientation
/// (extracted from `desired_mat` as Euler angles) and the current orientation
/// (from `current_cdu` converted to radians).
///
/// Output is in radians; positive pitch = nose up, positive yaw = nose left.
/// All-zero error is returned when CDU angles exactly match the desired matrix.
///
/// # Invariants
/// - Output components are always finite (no NaN/Inf) for finite inputs.
/// - If any CDU angle encodes NaN (hardware error), returns zero error.
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc
///   GETAKS (attitude error display, p. 1023), MERUPDAT (error accumulation, p. 1019),
///   AMGB1/4/5/7/8 matrix elements (p. 1006).
/// Comanche055/KALCMANU_STEERING.agc NEWANGL / DCMTOCDU (p. 414).
///
/// Units:
///   Input: `current_cdu` — raw CDU counts via `CduAngle`, converted internally to radians.
///   Input: `desired_mat` — dimensionless rotation matrix (Mat3x3).
///   Output: `AttitudeError` — radians.
pub fn compute_error(current_cdu: &[CduAngle; 3], desired_mat: &Mat3x3) -> AttitudeError {
    // Extract desired Euler angles from the rotation matrix
    // Using ZYX (yaw-pitch-roll) Euler angle decomposition:
    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    //
    // Pitch:  θ = asin(-R[2][0])
    // Yaw:    ψ = atan2(R[1][0], R[0][0])   (when cos(θ) ≠ 0)
    // Roll:   φ = atan2(R[2][1], R[2][2])
    //
    // This matches the DCMTOCDU decomposition in KALCMANU_STEERING.agc (p. 414).
    let r = desired_mat;

    // Clamp to avoid NaN from asin
    let sin_pitch = (-r[2][0]).clamp(-1.0, 1.0);
    let desired_pitch = libm::asin(sin_pitch);

    let cos_pitch = libm::cos(desired_pitch);
    let (desired_yaw, desired_roll) = if cos_pitch.abs() > 1e-6 {
        let yaw = libm::atan2(r[1][0], r[0][0]);
        let roll = libm::atan2(r[2][1], r[2][2]);
        (yaw, roll)
    } else {
        // Gimbal lock: pitch ≈ ±90°; yaw arbitrary, roll computed from remaining
        let yaw = libm::atan2(-r[0][1], r[1][1]);
        (yaw, 0.0)
    };

    // Convert CDU angles to radians
    // CDU[0] = CDUX (pitch axis), CDU[1] = CDUY (yaw axis), CDU[2] = CDUZ (roll axis)
    let cdu_pitch = current_cdu[0].to_radians();
    let cdu_yaw = current_cdu[1].to_radians();
    let cdu_roll = current_cdu[2].to_radians();

    // Wrap angle differences to [-π, π]
    let wrap = |d: f64| -> f64 {
        let tau = core::f64::consts::TAU;
        let pi = core::f64::consts::PI;
        let mut v = d % tau;
        if v > pi {
            v -= tau;
        } else if v < -pi {
            v += tau;
        }
        v
    };

    let pitch = wrap(desired_pitch - cdu_pitch);
    let yaw = wrap(desired_yaw - cdu_yaw);
    let roll = wrap(desired_roll - cdu_roll);

    // Guard against NaN (hardware fault in CDU)
    let safe = |x: f64| if x.is_finite() { x } else { 0.0 };

    AttitudeError {
        pitch: safe(pitch),
        yaw: safe(yaw),
        roll: safe(roll),
    }
}

/// Evaluate the phase-plane switching function for one body axis.
///
/// Implements the linear switching-surface approximation used throughout the
/// RCS and TVC roll DAPs.  The switching variable is:
///   `s = error + SLOPE × rate`
/// where SLOPE = 0.24 (≈ 0.6/s × sample_period).
///
/// Decision:
///   |s| < deadband/2  →  None
///   s > 0             →  Positive
///   s < 0             →  Negative
///
/// Caller must provide the per-axis deadband (confirm from DBTABLE decoding in S41.2;
/// default attitude-hold deadband ≈ 0.3° = 5.24e-3 rad).
///
/// # Invariants
/// - `deadband > 0.0`: returns `None` for non-positive deadband in release builds;
///   `debug_assert!` enforces positive deadband in debug builds.
/// - `error == 0.0 && rate == 0.0` always returns `None`.
/// - Finite `error` and `rate` always produce a finite (non-NaN) decision.
/// - This function is pure: no side effects, no state.
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc
///   SLOPE initialization (REDAP, line 572: `CAF =.24 → TS SLOPE`);
///   TAU generation at JETS label (p. 1020, referenced from SPNDXCHK).
/// Comanche055/TVCROLLDAP.agc ROLLOGIC (switching parabola / line, pp. 987–988).
///
/// Units:
///   `error`    — radians
///   `rate`     — radians/second
///   `deadband` — radians (must be > 0)
pub fn phase_plane_decision(error: f64, rate: f64, deadband: f64) -> JetDecision {
    debug_assert!(deadband > 0.0, "deadband must be positive");
    if deadband <= 0.0 {
        return JetDecision::None;
    }

    // Switching variable: s = error + SLOPE * rate
    let s = error + SLOPE * rate;

    let half_db = deadband * 0.5;
    if s.abs() < half_db {
        JetDecision::None
    } else if s > 0.0 {
        JetDecision::Positive
    } else {
        JetDecision::Negative
    }
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::constants::DEADBAND_DEFAULT_RAD;
    use crate::types::{CduAngle, IDENTITY_MAT3};

    const DB: f64 = DEADBAND_DEFAULT_RAD; // ≈ 5.236e-3 rad (0.3°)

    /// TC-ATT-1: Zero error and rate returns None.
    #[test]
    fn zero_error_rate_returns_none() {
        assert_eq!(phase_plane_decision(0.0, 0.0, DB), JetDecision::None);
    }

    /// TC-ATT-2: Positive error beyond deadband returns Positive.
    ///
    /// error = 0.01 rad ≈ 0.57°; deadband = 0.3° = 5.24e-3 rad.
    #[test]
    fn positive_error_beyond_deadband() {
        let result = phase_plane_decision(0.01, 0.0, DB);
        assert_eq!(result, JetDecision::Positive);
    }

    /// TC-ATT-3: Phase-plane hysteresis — rate cancels error.
    ///
    /// s = 0.01 + 0.24 × (−0.1) = 0.01 − 0.024 = −0.014 → Negative.
    #[test]
    fn rate_cancels_error_to_negative() {
        let result = phase_plane_decision(0.01, -0.1, DB);
        assert_eq!(result, JetDecision::Negative);
    }

    /// TC-ATT-4: Rate damping inside deadband → None.
    ///
    /// s = 0.003 + 0.24 × (−0.015) = 0.003 − 0.0036 = −0.0006.
    /// |s| = 6e-4 < DB/2 = 2.618e-3 → None.
    #[test]
    fn rate_damping_inside_deadband() {
        let result = phase_plane_decision(0.003, -0.015, DB);
        assert_eq!(result, JetDecision::None);
    }

    /// TC-ATT-5: Negative error beyond deadband returns Negative.
    #[test]
    fn negative_error_beyond_deadband() {
        assert_eq!(phase_plane_decision(-0.01, 0.0, DB), JetDecision::Negative);
    }

    /// TC-ATT-6: compute_error returns zero for identity matrix and zero CDU.
    #[test]
    fn compute_error_identity_zero_cdu() {
        let cdu = [CduAngle(0); 3];
        let err = compute_error(&cdu, &IDENTITY_MAT3);
        assert!(err.pitch.abs() < 1e-9, "pitch={}", err.pitch);
        assert!(err.yaw.abs() < 1e-9, "yaw={}", err.yaw);
        assert!(err.roll.abs() < 1e-9, "roll={}", err.roll);
    }

    /// TC-ATT-7: compute_error with yaw-only mismatch gives non-zero yaw error only.
    ///
    /// Construct a rotation matrix for a pure yaw of 5° and leave pitch/roll CDU at 0.
    #[test]
    fn compute_error_yaw_only() {
        use crate::math::linalg::rotz;
        let yaw_angle = 5.0_f64.to_radians();
        // For a yaw rotation Rz(ψ), the matrix is:
        // | cos ψ  -sin ψ  0 |
        // | sin ψ   cos ψ  0 |
        // |   0       0    1 |
        let desired_mat = rotz(yaw_angle);
        let cdu = [CduAngle(0); 3]; // CDU at zero

        let err = compute_error(&cdu, &desired_mat);

        // Yaw error should be ≈ yaw_angle; pitch and roll should be ≈ 0
        assert!(
            (err.yaw - yaw_angle).abs() < 1e-6,
            "yaw={} expected {}",
            err.yaw,
            yaw_angle
        );
        assert!(err.pitch.abs() < 1e-9, "pitch={}", err.pitch);
        assert!(err.roll.abs() < 1e-9, "roll={}", err.roll);
    }
}
