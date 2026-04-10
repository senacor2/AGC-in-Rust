//! Attitude control — rate damping, attitude hold, and maneuver logic.
//!
//! This module is the computational core of the CSM Coast DAP. It is called by
//! `control::dap` on every T5RUPT cycle (nominally every 100 ms) to produce
//! torque-demand vectors passed downstream to `control::rcs_logic`.
//!
//! All functions are pure (no side effects, no global state). No heap allocation.
//!
//! AGC source references:
//! - `Comanche055/CM_BODY_ATTITUDE.agc` — attitude error and body-rate derivation
//! - `Comanche055/RCS_CSM_DIGITAL_AUTOPILOT.agc` — rate-damping, attitude-hold,
//!   maneuver-rate logic, T5RUPT dispatch
//! - `Comanche055/ERASABLE_ASSIGNMENTS.agc` — CDUX (0033), CDUY (0034), CDUZ (0035)

use core::f64::consts::TAU;

use crate::math::linalg::{mxm, norm, transpose, unit, vscale};
use crate::types::{CduAngle, Mat3x3, Vec3};

// ── AttitudeError ─────────────────────────────────────────────────────────────

/// Three-axis attitude error (roll, pitch, yaw) in radians.
///
/// Positive error means the current attitude is rotated positively about that
/// body axis relative to the desired attitude.
///
/// AGC correspondence: ERRORX / ERRORY / ERRORZ, scaled B-1 half-revolutions.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AttitudeError {
    /// Roll error in radians. Positive = current attitude rotated clockwise
    /// about body X-axis relative to desired (right-wing-down for standard
    /// CM body-axis convention).
    pub roll: f64,
    /// Pitch error in radians. Positive = nose-up rotation about body Y-axis.
    pub pitch: f64,
    /// Yaw error in radians. Positive = nose-right rotation about body Z-axis.
    pub yaw: f64,
}

impl AttitudeError {
    /// Convert to a `Vec3` as `[roll, pitch, yaw]`.
    #[inline]
    pub fn as_vec3(self) -> Vec3 {
        [self.roll, self.pitch, self.yaw]
    }

    /// Construct from a `Vec3` `[roll, pitch, yaw]`.
    #[inline]
    pub fn from_vec3(v: Vec3) -> Self {
        Self {
            roll: v[0],
            pitch: v[1],
            yaw: v[2],
        }
    }
}

// ── compute_body_rates ────────────────────────────────────────────────────────

/// Estimate body angular rates in rad/s from two successive CDU readings.
///
/// Uses two's-complement (wrapping) subtraction on the raw `u16` counts to
/// handle the 0/65536 wrap-around correctly, then converts to rad/s.
///
/// # Preconditions
/// - `dt > 0.0`.  A zero or negative interval is a programming error; the
///   function `debug_assert!`s this and returns `[0.0; 3]` in release builds.
///
/// # CDU axis convention (§2.2)
/// Index 0 = roll (X / outer gimbal), 1 = pitch (Y / inner), 2 = yaw (Z / middle).
///
/// AGC source: `RCS_CSM_DIGITAL_AUTOPILOT.agc` body-rate read section.
pub fn compute_body_rates(cdu_new: [CduAngle; 3], cdu_old: [CduAngle; 3], dt: f64) -> Vec3 {
    debug_assert!(dt > 0.0, "compute_body_rates: dt must be positive");
    if dt <= 0.0 {
        return [0.0, 0.0, 0.0];
    }

    let mut rates = [0.0_f64; 3];
    for i in 0..3 {
        // Two's-complement subtraction: cast to i16 after wrapping_sub gives a
        // signed delta in (-32768, +32767], correctly handling 0/TAU wrap-around.
        let delta_counts = (cdu_new[i].0.wrapping_sub(cdu_old[i].0) as i16) as f64;
        let delta_rad = delta_counts * (TAU / 65536.0);
        rates[i] = delta_rad / dt;
    }
    rates
}

// ── compute_attitude_error ────────────────────────────────────────────────────

/// Compute the three-axis attitude error (roll, pitch, yaw) in radians.
///
/// Converts the current IMU gimbal CDU angles and the stored REFSMMAT into a
/// body-frame error rotation with respect to the commanded attitude matrix
/// `desired`.
///
/// # Algorithm (§4.2)
/// 1. Convert CDU counts to radians.
/// 2. Build M_gimbal = Rx(roll) · Ry(pitch) · Rz(yaw) (CM outer→inner→middle
///    gimbal suspension = Tait-Bryan XYZ applied left-to-right).
/// 3. M_current = refsmmat · M_gimbal
/// 4. M_err = desired^T · M_current
/// 5. Extract small-angle errors from the anti-symmetric part of M_err.
///
/// # Sign convention (CI-10)
/// A positive outer-gimbal rotation (positive roll CDU count) yields a positive
/// `error.roll`.  This is the "current-relative-to-desired" sign required by
/// `attitude_hold_torque`'s restoring-torque convention.
///
/// AGC source: `Comanche055/CM_BODY_ATTITUDE.agc`.
pub fn compute_attitude_error(
    current_cdu: [CduAngle; 3],
    desired: Mat3x3,
    refsmmat: Mat3x3,
) -> AttitudeError {
    // Step 1 — CDU counts to radians
    let theta_x = current_cdu[0].to_radians(); // outer  / roll
    let theta_y = current_cdu[1].to_radians(); // inner  / pitch
    let theta_z = current_cdu[2].to_radians(); // middle / yaw

    // Step 2 — Build M_gimbal = Rx(θx) · Ry(θy) · Rz(θz)
    let rx = rx(theta_x);
    let ry = ry(theta_y);
    let rz = rz(theta_z);
    let m_gimbal = mxm(mxm(rx, ry), rz);

    // Step 3 — Current inertial attitude: M_current = refsmmat · M_gimbal
    let m_current = mxm(refsmmat, m_gimbal);

    // Step 4 — Error matrix: M_err = desired^T · M_current
    let m_err = mxm(transpose(desired), m_current);

    // Step 5 — Extract roll/pitch/yaw from the anti-symmetric part
    let roll = (m_err[2][1] - m_err[1][2]) / 2.0;
    let pitch = (m_err[0][2] - m_err[2][0]) / 2.0;
    let yaw = (m_err[1][0] - m_err[0][1]) / 2.0;

    AttitudeError { roll, pitch, yaw }
}

// ── rate_damping_torque ───────────────────────────────────────────────────────

/// Compute the torque demand required to null the current body rates.
///
/// `torque[i] = -gain[i] * rates[i]`
///
/// The negative sign ensures that a positive rate produces a negative
/// (opposing) torque. The deadband check is the **caller's** responsibility.
///
/// # Preconditions
/// - `gain[i] >= 0.0` for all i (debug-asserted).
///
/// AGC source: `RCS_CSM_DIGITAL_AUTOPILOT.agc` rate-damping section.
pub fn rate_damping_torque(rates: Vec3, gain: Vec3) -> Vec3 {
    debug_assert!(gain[0] >= 0.0, "rate_damping_torque: gain[0] must be non-negative");
    debug_assert!(gain[1] >= 0.0, "rate_damping_torque: gain[1] must be non-negative");
    debug_assert!(gain[2] >= 0.0, "rate_damping_torque: gain[2] must be non-negative");

    [
        -gain[0] * rates[0],
        -gain[1] * rates[1],
        -gain[2] * rates[2],
    ]
}

// ── attitude_hold_torque ──────────────────────────────────────────────────────

/// Compute the PD attitude-hold torque from attitude error and body rates.
///
/// `torque[i] = -(kp * error[i] + kd * rates[i])`
///
/// The negative sign follows the convention that a positive attitude error
/// requires a negative (restoring) torque.  The deadband check is the
/// **caller's** responsibility.
///
/// # Preconditions
/// - `kp >= 0.0`, `kd >= 0.0` (debug-asserted).
///
/// AGC source: `RCS_CSM_DIGITAL_AUTOPILOT.agc` attitude hold / PD section.
pub fn attitude_hold_torque(error: AttitudeError, rates: Vec3, kp: f64, kd: f64) -> Vec3 {
    debug_assert!(kp >= 0.0, "attitude_hold_torque: kp must be non-negative");
    debug_assert!(kd >= 0.0, "attitude_hold_torque: kd must be non-negative");

    [
        -(kp * error.roll + kd * rates[0]),
        -(kp * error.pitch + kd * rates[1]),
        -(kp * error.yaw + kd * rates[2]),
    ]
}

// ── maneuver_rate ─────────────────────────────────────────────────────────────

/// Compute the instantaneous commanded angular rate vector for a large-angle slew.
///
/// Returns a body-frame angular rate vector (rad/s) that will rotate the
/// spacecraft from `current` toward `target` at up to `max_rate`.
///
/// # Algorithm (§4.6)
/// 1. M_err = transpose(target) · current
/// 2. Extract rotation axis (sine-scaled) from anti-symmetric part.
/// 3. Compute angle via libm::atan2(sin_angle, cos_angle).
/// 4. Return zero vector when angle < 1e-9 rad.
/// 5. Otherwise, return unit(axis) scaled by min(angle, max_rate).
///
/// # Preconditions
/// - `current` and `target` are orthonormal rotation matrices.
/// - `max_rate > 0.0` (debug-asserted).
///
/// AGC source: `Comanche055/RCS_CSM_DIGITAL_AUTOPILOT.agc` maneuver rate table.
pub fn maneuver_rate(current: Mat3x3, target: Mat3x3, max_rate: f64) -> Vec3 {
    debug_assert!(max_rate > 0.0, "maneuver_rate: max_rate must be positive");
    if max_rate <= 0.0 {
        return [0.0, 0.0, 0.0];
    }

    // Step 1 — Error rotation matrix from target to current
    let m_err = mxm(transpose(target), current);

    // Step 2 — Extract sine-scaled rotation axis from anti-symmetric part
    let e_x = (m_err[2][1] - m_err[1][2]) / 2.0;
    let e_y = (m_err[0][2] - m_err[2][0]) / 2.0;
    let e_z = (m_err[1][0] - m_err[0][1]) / 2.0;
    let e: Vec3 = [e_x, e_y, e_z];
    let sin_angle = norm(e);

    // Step 3 — True rotation angle using atan2 for numerical stability
    let cos_angle = (m_err[0][0] + m_err[1][1] + m_err[2][2] - 1.0) / 2.0;
    let angle = libm::atan2(sin_angle, cos_angle);

    // Step 4 — Nearly-zero angle: maneuver complete
    if angle < 1e-9 {
        return [0.0, 0.0, 0.0];
    }

    // Step 5 — Unit rotation axis
    let axis = unit(e);

    // Step 6 — Clamp to max_rate
    let rate_magnitude = if angle < max_rate { angle } else { max_rate };

    // Step 7 — Commanded rate vector
    vscale(axis, rate_magnitude)
}

// ── Elementary rotation matrices (right-hand-rule, standard form) ─────────────

/// Rotation matrix about the X-axis by angle θ.
#[inline]
fn rx(theta: f64) -> Mat3x3 {
    let c = libm::cos(theta);
    let s = libm::sin(theta);
    [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]
}

/// Rotation matrix about the Y-axis by angle θ.
#[inline]
fn ry(theta: f64) -> Mat3x3 {
    let c = libm::cos(theta);
    let s = libm::sin(theta);
    [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]
}

/// Rotation matrix about the Z-axis by angle θ.
#[inline]
fn rz(theta: f64) -> Mat3x3 {
    let c = libm::cos(theta);
    let s = libm::sin(theta);
    [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::linalg;
    use core::f64::consts::TAU;

    // ── TC-ATT-01: Zero error produces zero torque ────────────────────────────

    /// TC-ATT-01: All-zero CDU angles with identity desired and refsmmat must
    /// yield zero attitude error and, consequently, zero torque from the PD
    /// controller.
    #[test]
    fn tc_att_01_zero_error_zero_torque() {
        let identity: Mat3x3 = linalg::IDENTITY;
        let cdu = [CduAngle(0), CduAngle(0), CduAngle(0)];

        let error = compute_attitude_error(cdu, identity, identity);

        assert!(error.roll.abs() < 1e-12, "roll error should be zero, got {}", error.roll);
        assert!(error.pitch.abs() < 1e-12, "pitch error should be zero, got {}", error.pitch);
        assert!(error.yaw.abs() < 1e-12, "yaw error should be zero, got {}", error.yaw);

        let rates: Vec3 = [0.0, 0.0, 0.0];
        let torque = attitude_hold_torque(error, rates, 0.5, 1.0);

        assert!(torque[0].abs() < 1e-12, "torque[0] should be zero, got {}", torque[0]);
        assert!(torque[1].abs() < 1e-12, "torque[1] should be zero, got {}", torque[1]);
        assert!(torque[2].abs() < 1e-12, "torque[2] should be zero, got {}", torque[2]);
    }

    // ── TC-ATT-02: Pure roll error ────────────────────────────────────────────

    /// TC-ATT-02: A 5° outer-gimbal rotation (roll) must produce error.roll ≈ +5°
    /// (positive, sign-convention CI-10), with pitch and yaw ≈ 0.
    /// The PD torque must be negative on the roll axis (restoring) and zero elsewhere.
    #[test]
    fn tc_att_02_pure_roll_error() {
        let five_deg_counts = (5.0_f64.to_radians() * 65536.0 / TAU) as u16;
        let cdu = [CduAngle(five_deg_counts), CduAngle(0), CduAngle(0)];

        let error = compute_attitude_error(cdu, linalg::IDENTITY, linalg::IDENTITY);

        // The anti-symmetric part extraction gives sin(angle), not angle.
        // For 5° the difference (small-angle approx) is ~0.13%.
        let five_deg = 5.0_f64.to_radians();
        assert!(
            (error.roll - five_deg).abs() < 2e-4,
            "roll error should be ~5° (sin approx), got {}",
            error.roll
        );
        assert!(error.pitch.abs() < 1e-6, "pitch error should be ~0, got {}", error.pitch);
        assert!(error.yaw.abs() < 1e-6, "yaw error should be ~0, got {}", error.yaw);

        // Sign-convention check (CI-10 postcondition §4.2)
        assert!(
            error.roll > 0.0,
            "Positive outer-gimbal rotation must yield positive roll error (CI-10)"
        );

        // Torque sign: restoring torque must oppose the positive roll error
        let rates: Vec3 = [0.0, 0.0, 0.0];
        let torque = attitude_hold_torque(error, rates, 1.0, 0.0);

        assert!(torque[0] < 0.0, "restoring torque must be negative for positive roll error");
        assert!(
            torque[1].abs() < 1e-12,
            "pitch torque must be zero, got {}",
            torque[1]
        );
        assert!(
            torque[2].abs() < 1e-12,
            "yaw torque must be zero, got {}",
            torque[2]
        );
    }

    // ── TC-ATT-03: Pure rate damping ──────────────────────────────────────────

    /// TC-ATT-03: A 2°/s roll rate with unit gain must produce torque[0] = -(2°/s).
    /// Also verifies the CDU-differencing round-trip (compute_body_rates).
    #[test]
    fn tc_att_03_pure_rate_damping() {
        let omega_roll = 2.0_f64.to_radians(); // 2°/s
        let rates: Vec3 = [omega_roll, 0.0, 0.0];
        let gain: Vec3 = [1.0, 1.0, 1.0];

        let torque = rate_damping_torque(rates, gain);

        assert!(
            (torque[0] + omega_roll).abs() < 1e-15,
            "torque[0] should be -{}, got {}",
            omega_roll,
            torque[0]
        );
        assert!(torque[1].abs() < 1e-15, "torque[1] should be zero, got {}", torque[1]);
        assert!(torque[2].abs() < 1e-15, "torque[2] should be zero, got {}", torque[2]);

        // Round-trip via compute_body_rates
        let dt = 0.1_f64;
        let delta: u16 = ((omega_roll * dt) * 65536.0 / TAU).round() as u16;
        let cdu_old = [CduAngle(0u16), CduAngle(0), CduAngle(0)];
        let cdu_new = [CduAngle(delta), CduAngle(0), CduAngle(0)];
        let estimated = compute_body_rates(cdu_new, cdu_old, dt);

        // Allow ½ count quantisation error
        let quant = TAU / 65536.0 / dt;
        assert!(
            (estimated[0] - omega_roll).abs() < quant,
            "estimated rate {} should be within {} of {}",
            estimated[0],
            quant,
            omega_roll
        );
    }

    // ── TC-ATT-04: Attitude hold with small perturbation ─────────────────────

    /// TC-ATT-04: 1° pitch error + 0.1°/s pitch rate with kp=0.5, kd=1.0 must
    /// produce the exact PD torque on the pitch axis and zero on the others.
    #[test]
    fn tc_att_04_attitude_hold_pd() {
        let pitch_err = 1.0_f64.to_radians();
        let pitch_rate = 0.1_f64.to_radians();
        let error = AttitudeError { roll: 0.0, pitch: pitch_err, yaw: 0.0 };
        let rates: Vec3 = [0.0, pitch_rate, 0.0];
        let kp = 0.5_f64;
        let kd = 1.0_f64;

        let torque = attitude_hold_torque(error, rates, kp, kd);

        let expected_pitch = -(kp * pitch_err + kd * pitch_rate);
        assert!(
            (torque[1] - expected_pitch).abs() < 1e-14,
            "pitch torque should be {}, got {}",
            expected_pitch,
            torque[1]
        );
        assert!(torque[0].abs() < 1e-14, "roll torque should be zero, got {}", torque[0]);
        assert!(torque[2].abs() < 1e-14, "yaw torque should be zero, got {}", torque[2]);
    }

    // ── TC-ATT-05: Maneuver to 90° yaw target ────────────────────────────────

    /// TC-ATT-05: Current = identity, target = Rz(90°). The commanded rate must
    /// lie entirely on the Z-axis, clamped to max_rate.  Also verifies zero rate
    /// for current == target.
    #[test]
    fn tc_att_05_maneuver_90deg_yaw() {
        let current: Mat3x3 = linalg::IDENTITY;
        // Rz(90°)
        let target: Mat3x3 = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        let max_rate = 0.5_f64.to_radians(); // 0.5°/s

        let rate_cmd = maneuver_rate(current, target, max_rate);

        assert!(
            rate_cmd[0].abs() < 1e-12,
            "no roll component expected, got {}",
            rate_cmd[0]
        );
        assert!(
            rate_cmd[1].abs() < 1e-12,
            "no pitch component expected, got {}",
            rate_cmd[1]
        );
        assert!(
            (rate_cmd[2].abs() - max_rate).abs() < 1e-12,
            "|rate_cmd[2]| should equal max_rate {}, got {}",
            max_rate,
            rate_cmd[2].abs()
        );

        // Zero-error case
        let zero_rate = maneuver_rate(current, current, max_rate);
        assert_eq!(zero_rate, [0.0, 0.0, 0.0], "zero rate expected for current == target");
    }

    // ── TC-ATT-SIGN: CI-10 sign-convention validation ─────────────────────────

    /// TC-ATT-SIGN: A +1° positive roll error (body frame rotated +1° CCW about
    /// the roll axis from desired) must yield error.roll ≈ +0.017453 rad (positive).
    /// This validates the CI-10 sign convention for the full attitude error path.
    #[test]
    fn tc_att_sign_ci10_roll_sign_convention() {
        // Encode +1° as CDU counts for the outer (roll) gimbal
        let one_deg_counts = (1.0_f64.to_radians() * 65536.0 / TAU).round() as u16;
        let cdu = [CduAngle(one_deg_counts), CduAngle(0), CduAngle(0)];

        let error = compute_attitude_error(cdu, linalg::IDENTITY, linalg::IDENTITY);

        let expected = 1.0_f64.to_radians(); // ≈ 0.017453 rad
        assert!(
            (error.roll - expected).abs() < 1e-4,
            "error.roll should be ≈ +{:.5} rad (CI-10), got {:.5}",
            expected,
            error.roll
        );
        assert!(
            error.roll > 0.0,
            "CI-10: positive outer-gimbal rotation must produce positive roll error"
        );
    }
}
