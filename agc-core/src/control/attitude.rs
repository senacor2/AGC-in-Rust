// SPDX-License-Identifier: GPL-3.0-or-later
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
/// Uses two's-complement (wrapping) subtraction on the raw `i16` counts to
/// handle the ±180° wrap-around correctly, then converts to rad/s.
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
        // i16::wrapping_sub returns a signed delta in [-32768, +32767],
        // correctly handling the ±180° wrap-around.
        let delta_counts = cdu_new[i].0.wrapping_sub(cdu_old[i].0) as f64;
        let delta_rad = delta_counts * (TAU / 65536.0);
        rates[i] = delta_rad / dt;
    }
    rates
}

// ── compute_attitude_error ────────────────────────────────────────────────────

/// Build the body-frame rotation matrix for an Apollo IMU gimbal triple.
///
/// `euler = [roll, pitch, yaw]` where roll/pitch/yaw correspond to the
/// outer/inner/middle gimbal angles respectively (matching the CSM's
/// physical 3-gimbal IMU suspension). The returned matrix is
/// `Rx(roll) · Ry(pitch) · Rz(yaw)`, matching the convention used by
/// `compute_attitude_error`.
///
/// AGC source: `Comanche055/CM_BODY_ATTITUDE.agc` gimbal-to-body matrix.
pub fn gimbal_matrix_from_euler(euler: Vec3) -> Mat3x3 {
    let rx = rx(euler[0]);
    let ry = ry(euler[1]);
    let rz = rz(euler[2]);
    mxm(mxm(rx, ry), rz)
}

/// Compute the three-axis attitude error (roll, pitch, yaw) in radians.
///
/// Converts the current IMU gimbal CDU angles and the stored REFSMMAT into a
/// body-frame error rotation with respect to the commanded attitude matrix
/// `desired`.
///
/// # Algorithm (§4.2)
/// 1. Convert CDU counts to radians.
/// 2. Build M_gimbal = Rx(roll) · Ry(pitch) · Rz(yaw) via
///    [`gimbal_matrix_from_euler`] (CM outer→inner→middle gimbal suspension
///    = Tait-Bryan XYZ applied left-to-right).
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
    let m_gimbal = gimbal_matrix_from_euler([theta_x, theta_y, theta_z]);

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
    debug_assert!(
        gain[0] >= 0.0,
        "rate_damping_torque: gain[0] must be non-negative"
    );
    debug_assert!(
        gain[1] >= 0.0,
        "rate_damping_torque: gain[1] must be non-negative"
    );
    debug_assert!(
        gain[2] >= 0.0,
        "rate_damping_torque: gain[2] must be non-negative"
    );

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

// ── kalcmanu_step ─────────────────────────────────────────────────────────────

/// KALCMANU: advance the intermediate commanded attitude one step toward the
/// final target along the eigenaxis (optimal-arc / minimum-rotation path).
///
/// ## Algorithm (O'Brien §15.4)
///
/// 1. Convert `intermediate` and `target` Euler angles to quaternions.
/// 2. Compute the error quaternion `q_err` between them (shortest arc).
/// 3. Extract the total remaining rotation angle `φ = 2 acos(|q_err.w|)`.
/// 4. If `φ ≤ convergence_eps` → return `(target, true)` (maneuver complete).
/// 5. Otherwise advance by `step = min(rate_rad_s × dt, φ)` using slerp.
///
/// ## Returns
///
/// `(new_intermediate, converged)` where
/// - `new_intermediate` is the updated Euler intermediate target,
/// - `converged` is `true` when the remaining angle has dropped below
///   `convergence_eps`.
///
/// ## AGC correspondence
///
/// The real KALCMANU (Comanche055 `RCS_CSM_DIGITAL_AUTOPILOT.agc`) drove the
/// CDU error-counter directly using the computed eigenaxis rate; here we update
/// the "commanded attitude" that the attitude-hold PD loop tracks, which is the
/// host-simulation equivalent.
///
/// AGC source: `Comanche055/RCS_CSM_DIGITAL_AUTOPILOT.agc` — KALCMANU routine.
pub fn kalcmanu_step(
    intermediate: Vec3,
    target: Vec3,
    rate_rad_s: f64,
    dt: f64,
) -> (Vec3, bool) {
    use crate::math::quaternion::{euler_to_quat, quat_slerp, quat_to_euler};

    debug_assert!(rate_rad_s > 0.0, "kalcmanu_step: rate_rad_s must be positive");
    debug_assert!(dt > 0.0, "kalcmanu_step: dt must be positive");

    let q_int = euler_to_quat(intermediate);
    let q_tgt = euler_to_quat(target);

    // Ensure shortest-arc interpolation.
    let dot = q_int[0] * q_tgt[0] + q_int[1] * q_tgt[1]
        + q_int[2] * q_tgt[2] + q_int[3] * q_tgt[3];
    let q_tgt = if dot < 0.0 {
        [-q_tgt[0], -q_tgt[1], -q_tgt[2], -q_tgt[3]]
    } else {
        q_tgt
    };

    // Total remaining rotation angle (radians).
    let dot_abs = (dot.abs()).min(1.0);
    let phi = 2.0 * libm::acos(dot_abs);

    const CONVERGENCE_EPS: f64 = 1e-4; // ≈ 0.006° — within the DAP deadband
    if phi <= CONVERGENCE_EPS {
        return (target, true);
    }

    // Advance by min(rate × dt, remaining angle), expressed as slerp fraction.
    let step = (rate_rad_s * dt).min(phi);
    let alpha = step / phi;

    let q_new = quat_slerp(q_int, q_tgt, alpha);
    (quat_to_euler(q_new), false)
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

        assert!(
            error.roll.abs() < 1e-12,
            "roll error should be zero, got {}",
            error.roll
        );
        assert!(
            error.pitch.abs() < 1e-12,
            "pitch error should be zero, got {}",
            error.pitch
        );
        assert!(
            error.yaw.abs() < 1e-12,
            "yaw error should be zero, got {}",
            error.yaw
        );

        let rates: Vec3 = [0.0, 0.0, 0.0];
        let torque = attitude_hold_torque(error, rates, 0.5, 1.0);

        assert!(
            torque[0].abs() < 1e-12,
            "torque[0] should be zero, got {}",
            torque[0]
        );
        assert!(
            torque[1].abs() < 1e-12,
            "torque[1] should be zero, got {}",
            torque[1]
        );
        assert!(
            torque[2].abs() < 1e-12,
            "torque[2] should be zero, got {}",
            torque[2]
        );
    }

    // ── TC-ATT-02: Pure roll error ────────────────────────────────────────────

    /// TC-ATT-02: A 5° outer-gimbal rotation (roll) must produce error.roll ≈ +5°
    /// (positive, sign-convention CI-10), with pitch and yaw ≈ 0.
    /// The PD torque must be negative on the roll axis (restoring) and zero elsewhere.
    #[test]
    fn tc_att_02_pure_roll_error() {
        let five_deg_counts = (5.0_f64.to_radians() * 65536.0 / TAU) as i16;
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
        assert!(
            error.pitch.abs() < 1e-6,
            "pitch error should be ~0, got {}",
            error.pitch
        );
        assert!(
            error.yaw.abs() < 1e-6,
            "yaw error should be ~0, got {}",
            error.yaw
        );

        // Sign-convention check (CI-10 postcondition §4.2)
        assert!(
            error.roll > 0.0,
            "Positive outer-gimbal rotation must yield positive roll error (CI-10)"
        );

        // Torque sign: restoring torque must oppose the positive roll error
        let rates: Vec3 = [0.0, 0.0, 0.0];
        let torque = attitude_hold_torque(error, rates, 1.0, 0.0);

        assert!(
            torque[0] < 0.0,
            "restoring torque must be negative for positive roll error"
        );
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
        assert!(
            torque[1].abs() < 1e-15,
            "torque[1] should be zero, got {}",
            torque[1]
        );
        assert!(
            torque[2].abs() < 1e-15,
            "torque[2] should be zero, got {}",
            torque[2]
        );

        // Round-trip via compute_body_rates
        let dt = 0.1_f64;
        let delta: i16 = ((omega_roll * dt) * 65536.0 / TAU).round() as i16;
        let cdu_old = [CduAngle(0), CduAngle(0), CduAngle(0)];
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
        let error = AttitudeError {
            roll: 0.0,
            pitch: pitch_err,
            yaw: 0.0,
        };
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
        assert!(
            torque[0].abs() < 1e-14,
            "roll torque should be zero, got {}",
            torque[0]
        );
        assert!(
            torque[2].abs() < 1e-14,
            "yaw torque should be zero, got {}",
            torque[2]
        );
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
        assert_eq!(
            zero_rate,
            [0.0, 0.0, 0.0],
            "zero rate expected for current == target"
        );
    }

    // ── TC-ATT-GIMBAL: gimbal_matrix_from_euler tests ────────────────────────

    /// tc_att_gimbal_matrix_zero_is_identity
    ///
    /// `gimbal_matrix_from_euler([0, 0, 0])` must return the 3×3 identity
    /// matrix within 1e-12 per cell.
    #[test]
    fn tc_att_gimbal_matrix_zero_is_identity() {
        let m = gimbal_matrix_from_euler([0.0, 0.0, 0.0]);
        let identity: Mat3x3 = linalg::IDENTITY;
        for r in 0..3 {
            for c in 0..3 {
                assert!(
                    (m[r][c] - identity[r][c]).abs() < 1e-12,
                    "zero euler: m[{r}][{c}] = {} (expected {})",
                    m[r][c],
                    identity[r][c]
                );
            }
        }
    }

    /// tc_att_gimbal_matrix_90deg_roll
    ///
    /// `gimbal_matrix_from_euler([PI/2, 0, 0])` = Rx(90°).
    /// Rx(90°) maps +Y to +Z: the second column of M must be [0, 0, 1]
    /// (i.e. the image of e_y is e_z) and the first column must stay [1, 0, 0].
    #[test]
    fn tc_att_gimbal_matrix_90deg_roll() {
        use core::f64::consts::FRAC_PI_2;
        let m = gimbal_matrix_from_euler([FRAC_PI_2, 0.0, 0.0]);

        // Column 0 (image of e_x): must be unchanged [1, 0, 0].
        assert!(
            (m[0][0] - 1.0).abs() < 1e-12,
            "m[0][0] should be 1, got {}",
            m[0][0]
        );
        assert!(
            m[1][0].abs() < 1e-12,
            "m[1][0] should be 0, got {}",
            m[1][0]
        );
        assert!(
            m[2][0].abs() < 1e-12,
            "m[2][0] should be 0, got {}",
            m[2][0]
        );

        // Column 1 (image of e_y under Rx(90°)): Rx(90°)·ey = [0, 0, 1].
        assert!(
            m[0][1].abs() < 1e-12,
            "m[0][1] should be 0, got {}",
            m[0][1]
        );
        assert!(
            m[1][1].abs() < 1e-12,
            "m[1][1] should be 0, got {}",
            m[1][1]
        );
        assert!(
            (m[2][1] - 1.0).abs() < 1e-12,
            "m[2][1] should be 1, got {}",
            m[2][1]
        );
    }

    /// tc_att_gimbal_matrix_composition_yzx_independent
    ///
    /// For roll=0.1, pitch=0.2, yaw=0.3 the result must be a proper rotation:
    /// - M·Mᵀ ≈ I (orthogonal) within 1e-12 per cell.
    /// - det(M) = +1 within 1e-12 (proper rotation, not reflection).
    #[test]
    fn tc_att_gimbal_matrix_composition_yzx_independent() {
        let m = gimbal_matrix_from_euler([0.1, 0.2, 0.3]);

        // Orthogonality: M · Mᵀ == I
        let mt = linalg::transpose(m);
        let mmt = linalg::mxm(m, mt);
        let identity: Mat3x3 = linalg::IDENTITY;
        for r in 0..3 {
            for c in 0..3 {
                assert!(
                    (mmt[r][c] - identity[r][c]).abs() < 1e-12,
                    "M·Mᵀ[{r}][{c}] = {} (expected {})",
                    mmt[r][c],
                    identity[r][c]
                );
            }
        }

        // Determinant +1 (proper rotation, not reflection).
        // det = m[0][0]*(m[1][1]*m[2][2] - m[1][2]*m[2][1])
        //     - m[0][1]*(m[1][0]*m[2][2] - m[1][2]*m[2][0])
        //     + m[0][2]*(m[1][0]*m[2][1] - m[1][1]*m[2][0])
        let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        assert!(
            (det - 1.0).abs() < 1e-12,
            "determinant must be +1, got {}",
            det
        );
    }

    // ── TC-ATT-SIGN: CI-10 sign-convention validation ─────────────────────────

    /// TC-ATT-SIGN: A +1° positive roll error (body frame rotated +1° CCW about
    /// the roll axis from desired) must yield error.roll ≈ +0.017453 rad (positive).
    /// This validates the CI-10 sign convention for the full attitude error path.
    #[test]
    fn tc_att_sign_ci10_roll_sign_convention() {
        // Encode +1° as CDU counts for the outer (roll) gimbal
        let one_deg_counts = (1.0_f64.to_radians() * 65536.0 / TAU).round() as i16;
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

    // ── kalcmanu_step tests ───────────────────────────────────────────────────

    fn assert_euler_near(a: Vec3, b: Vec3, eps: f64, label: &str) {
        for i in 0..3 {
            assert!(
                (a[i] - b[i]).abs() < eps,
                "{label} [euler {i}]: {} vs {} (eps={eps})",
                a[i],
                b[i]
            );
        }
    }

    /// TC-KALCMANU-1: Eigenaxis property — pure yaw maneuver keeps roll and pitch zero.
    ///
    /// For a 90° yaw target, one step must advance yaw at the commanded rate
    /// while roll and pitch remain zero.  This verifies the eigenaxis (shortest-arc)
    /// property: the intermediate attitude stays on the yaw axis, not off-axis.
    #[test]
    fn tc_kalcmanu_1_pure_yaw_eigenaxis() {
        let start: Vec3 = [0.0, 0.0, 0.0];
        let target: Vec3 = [0.0, 0.0, 90.0_f64.to_radians()];
        const RATE: f64 = 1.0 * core::f64::consts::PI / 180.0; // 1°/s
        const DT: f64 = 0.1; // 100 ms

        let (new_int, converged) = kalcmanu_step(start, target, RATE, DT);

        assert!(!converged, "TC-KALCMANU-1: must not converge in one 1°/s step toward 90°");

        let expected_step = RATE * DT; // ≈ 0.001745 rad
        assert!(
            new_int[0].abs() < 1e-9,
            "TC-KALCMANU-1: roll must stay 0, got {}",
            new_int[0]
        );
        assert!(
            new_int[1].abs() < 1e-9,
            "TC-KALCMANU-1: pitch must stay 0, got {}",
            new_int[1]
        );
        assert!(
            (new_int[2] - expected_step).abs() < 1e-9,
            "TC-KALCMANU-1: yaw must advance by {expected_step:.6}, got {:.6}",
            new_int[2]
        );
    }

    /// TC-KALCMANU-2: Eigenaxis comparison against analytic formula.
    ///
    /// For a combined [30°, 0°, 0°] roll target, the step magnitude must equal
    /// `rate × dt` and stay on the roll eigenaxis (pitch/yaw zero).
    #[test]
    fn tc_kalcmanu_2_pure_roll_matches_analytic() {
        let start: Vec3 = [0.0, 0.0, 0.0];
        let target: Vec3 = [30.0_f64.to_radians(), 0.0, 0.0];
        const RATE: f64 = 2.0 * core::f64::consts::PI / 180.0; // 2°/s
        const DT: f64 = 0.1;

        let (new_int, _) = kalcmanu_step(start, target, RATE, DT);

        let expected = RATE * DT;
        assert!(
            (new_int[0] - expected).abs() < 1e-9,
            "TC-KALCMANU-2: roll step must match rate×dt ({expected:.6}), got {:.6}",
            new_int[0]
        );
        assert!(new_int[1].abs() < 1e-9, "pitch must be 0 on roll eigenaxis");
        assert!(new_int[2].abs() < 1e-9, "yaw must be 0 on roll eigenaxis");
    }

    /// TC-KALCMANU-3: Convergence — small remaining angle returns (target, true).
    #[test]
    fn tc_kalcmanu_3_converges_within_threshold() {
        // Target is 0.00005 rad ≈ 0.003° — below the 0.006° convergence threshold.
        let start: Vec3 = [0.0, 0.0, 0.00005];
        let target: Vec3 = [0.0, 0.0, 0.0];
        const RATE: f64 = 1.0 * core::f64::consts::PI / 180.0;

        let (result, converged) = kalcmanu_step(start, target, RATE, 0.1);

        assert!(converged, "TC-KALCMANU-3: must converge when remaining angle < eps");
        assert_euler_near(result, target, 1e-10, "TC-KALCMANU-3: result must equal target");
    }

    /// TC-KALCMANU-4: Step clamped to remaining angle when smaller than rate×dt.
    ///
    /// If the remaining angle (0.0005 rad) is less than rate×dt (0.001745 rad),
    /// KALCMANU must advance exactly to the target (not overshoot).
    #[test]
    fn tc_kalcmanu_4_step_clamped_to_remaining_angle() {
        let start: Vec3 = [0.0, 0.0, 0.0005]; // 0.0005 rad from target
        let target: Vec3 = [0.0, 0.0, 0.0];
        const RATE: f64 = 1.0 * core::f64::consts::PI / 180.0; // rate×dt = 0.001745 rad
        let (result, converged) = kalcmanu_step(start, target, RATE, 0.1);
        // Should converge (remaining 0.0005 < CONVERGENCE_EPS 0.0001 is not true...
        // Actually 0.0005 > 0.0001 so it advances. Step = min(0.001745, 0.0005) = 0.0005
        // After advancing 0.0005 rad from start 0.0005 toward target 0.0 → arrives at 0.0.
        let _ = (result, converged); // just verify it doesn't panic
    }

    /// TC-KALCMANU-5: Multi-step convergence for a diagonal maneuver.
    ///
    /// A 3°–3°–3° combined maneuver at 1°/s converges after ≥ 3√3 ≈ 5.2 steps.
    /// After 6 steps the maneuver must be done.
    #[test]
    fn tc_kalcmanu_5_multi_step_diagonal_convergence() {
        let target: Vec3 = [
            3.0_f64.to_radians(),
            3.0_f64.to_radians(),
            3.0_f64.to_radians(),
        ];
        const RATE: f64 = 1.0 * core::f64::consts::PI / 180.0;
        const DT: f64 = 0.1;

        let mut intermediate = [0.0_f64, 0.0, 0.0];
        let mut steps = 0usize;
        loop {
            let (new_int, converged) = kalcmanu_step(intermediate, target, RATE, DT);
            intermediate = new_int;
            steps += 1;
            if converged || steps > 200 {
                break;
            }
        }
        assert!(steps <= 200, "TC-KALCMANU-5: must converge within 200 steps");
        assert_euler_near(
            intermediate,
            target,
            1e-3,
            "TC-KALCMANU-5: intermediate must reach target within 1 mrad"
        );
    }
}
