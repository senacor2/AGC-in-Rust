//! IMU coarse/fine alignment, gyro drift compensation, and PIPA compensation.
//!
//! This module owns:
//! - `ImuAlignmentState` — runtime enum tracking the platform lifecycle.
//! - `GyroCompensation`  — gyro drift bias constants (NBD).
//! - Pure functions implementing the PIPA compensation pipeline.
//! - Pure functions for coarse and fine alignment step computations.
//! - The REFSMMAT construction algorithm (TRIAD method) used by P51/P52.
//! - Gimbal lock detection (warning and critical thresholds).
//!
//! **No hardware access** happens inside any function in this module.
//! All HAL I/O is performed by the callers (T4RUPT ISR shim, P51/P52 programs).
//!
//! AGC source references:
//! - `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc`
//! - `Comanche055/IMU_COMPENSATION_PACKAGE.agc`
//! - `Comanche055/AVERAGE_G_INTEGRATOR.agc`

use core::f64::consts::TAU;

use crate::math::linalg::{cross, mxm, mxv, norm, transpose, unit};
use crate::services::average_g::PipaCalibration;
use crate::types::{CduAngle, Mat3x3, Vec3};

// ── Module-level constants ────────────────────────────────────────────────────

/// Maximum CDU count error considered "converged" after coarse align.
/// 2 counts ≈ 0.011° (2/65536 × 360°).
pub const COARSE_ALIGN_THRESHOLD: u16 = 2;

/// Maximum attitude error (radians) for fine alignment completion.
/// 0.1 arcminute ≈ 2.909e-5 rad = (0.1/60) × (π/180).
pub const FINE_ALIGN_THRESHOLD: f64 = 2.909e-5;

/// Minimum cross-product magnitude for a valid REFSMMAT computation.
/// Below this the two star directions are considered collinear.
pub const COLLINEAR_EPSILON: f64 = 1e-6;

/// Gyro torque scale: radians per pulse.
/// 1 pulse = B-15 revolutions = TAU / 32768 rad ≈ 1.9175e-4 rad.
pub const GYRO_PULSE_RAD: f64 = TAU / 32768.0;

/// T4RUPT nominal period in centiseconds (120 ms).
pub const T4RUPT_PERIOD_CS: u32 = 12;

// ── ImuAlignmentState ─────────────────────────────────────────────────────────

/// Tracks the alignment lifecycle of the IMU stable platform.
///
/// Mirrors the bare-metal HAL typestate (`Unaligned`, `CoarseAligned`,
/// `FineAligned` on `ImuImpl<State>`) but lives in erasable memory as a
/// runtime enum so that `AgcState` can record and preserve the alignment
/// status across RESTART.
///
/// AGC source: `IMU_CALIBRATION_AND_ALIGNMENT.agc` — alignment phase flags
/// in erasable (IMODES33, channel 30 monitor bits).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum ImuAlignmentState {
    /// Platform caged (gimbals locked to zero).
    ///
    /// `hal::Imu::is_caged()` returns `true`. No navigation is possible.
    /// Gyro torque commands and fine-align operations are inhibited.
    #[default]
    Caged,

    /// Platform uncaged and coarsely aligned.
    ///
    /// CDU drive commands have completed; platform is within ~1° of target.
    /// PIPA counts are accumulating but SERVICER should not integrate them
    /// until fine alignment is complete.
    CoarseAligned,

    /// Platform fine aligned.
    ///
    /// Gyro torque nulling has reduced residual error to < 0.1 arcminute.
    /// SERVICER may begin integrating PIPA counts. REFSMMAT is valid.
    FineAligned,
}

// ── GyroCompensation ──────────────────────────────────────────────────────────

/// Gyro drift bias compensation constants (NBD — Non-drift acceleration
/// equivalent) for the three IMU gyroscope axes.
///
/// These constants are measured pre-flight and uplinked from Mission Control
/// when updated. Applied during each T4RUPT cycle to null accumulated drift.
///
/// AGC source: `Comanche055/IMU_COMPENSATION_PACKAGE.agc` — NBDX, NBDY, NBDZ
/// erasable assignments and the torquing loop.
#[derive(Clone, Copy, Debug, Default)]
pub struct GyroCompensation {
    /// X-axis gyro drift rate (rad/s).
    /// AGC: NBDX (erasable, E1 bank).
    pub nbdx: f64,
    /// Y-axis gyro drift rate (rad/s).
    /// AGC: NBDY (erasable, E1 bank).
    pub nbdy: f64,
    /// Z-axis gyro drift rate (rad/s).
    /// AGC: NBDZ (erasable, E1 bank).
    pub nbdz: f64,
}

// ── apply_pipa_compensation ───────────────────────────────────────────────────

/// Apply PIPA calibration corrections to raw hardware counts.
///
/// Implements steps 2–4 of the SERVICER PIPA pipeline from
/// `specs/average-g-spec.md` §3.3:
///   Step 2 — subtract bias (NBDX/NBDY/NBDZ in count units)
///   Step 3 — apply scale factor (1/PIPADT, m/s per count)
///   Step 4 — apply misalignment correction matrix (PIPASR)
///
/// The result is delta-V in the stable-member (platform) frame, in m/s.
/// The caller (SERVICER) is responsible for step 5: rotating via REFSMMAT
/// into the inertial frame.
///
/// # Arguments
///
/// * `raw` — raw PIPA pulse counts `[x, y, z]` as returned by
///   `hal::Imu::read_pipa()`.
/// * `cal` — calibration constants from `AgcState::pipa_cal`.
///
/// # Returns
///
/// Delta-V vector in the stable-member frame, SI units (m/s).
///
/// # AGC source
///
/// `Comanche055/AVERAGE_G_INTEGRATOR.agc` — the innermost compensation loop.
pub fn apply_pipa_compensation(raw: [i16; 3], cal: &PipaCalibration) -> Vec3 {
    // Step 2: subtract bias in i32 to prevent overflow
    let compensated: [i32; 3] = [
        raw[0] as i32 - cal.bias[0] as i32,
        raw[1] as i32 - cal.bias[1] as i32,
        raw[2] as i32 - cal.bias[2] as i32,
    ];

    // Step 3: apply scale factor (1/PIPADT, m/s per count)
    let dv_platform: Vec3 = [
        compensated[0] as f64 * cal.scale,
        compensated[1] as f64 * cal.scale,
        compensated[2] as f64 * cal.scale,
    ];

    // Step 4: apply misalignment correction matrix (PIPASR)
    mxv(cal.misalignment, dv_platform)
}

// ── compute_gyro_drift ────────────────────────────────────────────────────────

/// Compute compensating gyro torque pulse counts to apply for drift correction.
///
/// The drift model is a constant-rate bias. The total drift over `dt_cs`
/// centiseconds is the product of the bias rate and the interval. Returns
/// signed pulse counts (negated to oppose drift) ready for `torque_gyro`.
///
/// # Arguments
///
/// * `dt_cs` — elapsed time since the last drift-compensation call, in
///   centiseconds. Nominally 12 (= 120 ms T4RUPT cycle).
/// * `nbd`   — gyro drift bias `[x, y, z]` in rad/s (steady-state drift rate).
///
/// # Returns
///
/// `[i16; 3]` negative pulse counts (opposing drift) for axes X, Y, Z.
/// Values are clamped to `[i16::MIN, i16::MAX]`.
///
/// # Scale factor
///
/// 1 pulse = TAU / 32768 rad (B-15 rev scale), so
/// pulses = radians * 32768 / TAU.
///
/// # AGC source
///
/// `Comanche055/IMU_COMPENSATION_PACKAGE.agc` — gyro torque compensation loop.
pub fn compute_gyro_drift(dt_cs: u32, nbd: [f64; 3]) -> [i16; 3] {
    let dt_seconds = dt_cs as f64 / 100.0;

    let clamp_i16 = |x: f64| {
        if x > i16::MAX as f64 {
            i16::MAX
        } else if x < i16::MIN as f64 {
            i16::MIN
        } else {
            x as i16
        }
    };

    let radians_x = nbd[0] * dt_seconds;
    let radians_y = nbd[1] * dt_seconds;
    let radians_z = nbd[2] * dt_seconds;

    let px = clamp_i16(radians_x * 32768.0 / TAU);
    let py = clamp_i16(radians_y * 32768.0 / TAU);
    let pz = clamp_i16(radians_z * 32768.0 / TAU);

    [-px, -py, -pz]
}

// ── coarse_align_step ─────────────────────────────────────────────────────────

/// Compute CDU drive pulse commands to slew the platform toward target angles.
///
/// The signed difference (target − current) is computed using wrapping u16
/// subtraction interpreted as i16, which correctly handles the 0°/360°
/// boundary (shortest angular path).
///
/// # Arguments
///
/// * `target`  — desired CDU angles `[outer, inner, middle]`.
/// * `current` — current CDU angles from `hal::Imu::read_cdu()`.
///
/// # Returns
///
/// `[i16; 3]` signed pulse counts for `[outer, inner, middle]`.
///
/// # AGC source
///
/// `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc` — coarse-align CDU drive loop.
pub fn coarse_align_step(target: [CduAngle; 3], current: [CduAngle; 3]) -> [i16; 3] {
    [
        target[0].0.wrapping_sub(current[0].0) as i16,
        target[1].0.wrapping_sub(current[1].0) as i16,
        target[2].0.wrapping_sub(current[2].0) as i16,
    ]
}

// ── fine_align_torque ─────────────────────────────────────────────────────────

/// Compute gyro torque pulse commands to drive residual platform error to zero.
///
/// Converts an attitude error vector (radians, small-angle, stable-member frame)
/// into signed gyro pulse counts. The gyro scale factor is 1 pulse = TAU/32768
/// rad (B-15 rev scale).
///
/// # Arguments
///
/// * `attitude_error` — residual platform orientation error `[x, y, z]` in
///   radians.
/// * `dt_s`           — elapsed time in seconds. Nominally 0.12 s (T4RUPT).
///
/// # Returns
///
/// `[i16; 3]` signed torque pulse counts, clamped to i16 range.
///
/// # AGC source
///
/// `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc` — fine-align torquing loop.
pub fn fine_align_torque(attitude_error: Vec3, dt_s: f64) -> [i16; 3] {
    const RAD_TO_PULSES: f64 = 32768.0 / TAU;
    const GAIN: f64 = 1.0;

    let clamp_i16 = |x: f64| {
        if x > i16::MAX as f64 {
            i16::MAX
        } else if x < i16::MIN as f64 {
            i16::MIN
        } else {
            x as i16
        }
    };

    [
        clamp_i16(libm::round(attitude_error[0] * RAD_TO_PULSES * GAIN * dt_s)),
        clamp_i16(libm::round(attitude_error[1] * RAD_TO_PULSES * GAIN * dt_s)),
        clamp_i16(libm::round(attitude_error[2] * RAD_TO_PULSES * GAIN * dt_s)),
    ]
}

// ── refsmmat_from_star_sightings ──────────────────────────────────────────────

/// Construct a REFSMMAT rotation matrix from two star sightings (TRIAD method).
///
/// Builds an orthonormal rotation matrix mapping the stable-member (platform)
/// frame to the inertial reference frame. Used by P51 and P52.
///
/// # Algorithm — TRIAD Method
///
/// Inertial triad:
///   r1 = unit(star1_inertial)
///   r2 = unit(cross(r1, star2_inertial))
///   r3 = cross(r1, r2)
///
/// Platform triad:
///   s1 = unit(star1_platform)
///   s2 = unit(cross(s1, star2_platform))
///   s3 = cross(s1, s2)
///
/// REFSMMAT = transpose(R_inertial_rows) · R_platform_rows
///
/// # Returns
///
/// `Some(Mat3x3)` — the new REFSMMAT, or `None` if the star vectors are
/// collinear (cross product magnitude < `COLLINEAR_EPSILON`).
///
/// # AGC source
///
/// `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc` — PFRAM routine.
pub fn refsmmat_from_star_sightings(
    star1_inertial: Vec3,
    star2_inertial: Vec3,
    star1_platform: Vec3,
    star2_platform: Vec3,
) -> Option<Mat3x3> {
    // Build inertial triad
    let r1 = unit(star1_inertial);
    let r2_raw = cross(r1, star2_inertial);
    if norm(r2_raw) < COLLINEAR_EPSILON {
        return None; // Stars are collinear in inertial frame
    }
    let r2 = unit(r2_raw);
    let r3 = cross(r1, r2);

    // Build platform (measurement) triad
    let s1 = unit(star1_platform);
    let s2_raw = cross(s1, star2_platform);
    if norm(s2_raw) < COLLINEAR_EPSILON {
        return None; // Measurement vectors are collinear
    }
    let s2 = unit(s2_raw);
    let s3 = cross(s1, s2);

    // Row-major matrices: each row is a basis vector
    let r_inertial: Mat3x3 = [r1, r2, r3];
    let r_platform: Mat3x3 = [s1, s2, s3];

    // REFSMMAT = R_inertial^T · R_platform
    Some(mxm(transpose(r_inertial), r_platform))
}

// ── is_gimbal_lock_warning ────────────────────────────────────────────────────

/// Check whether the middle gimbal (CDUZ, yaw axis) is approaching gimbal lock.
///
/// Gimbal lock occurs when the middle gimbal angle approaches ±90°
/// (= 16384 or 49152 CDU counts). Warning fires when within 20° of either
/// singularity (GIMBAL_LOCK_WARNING_BAND = 3641 counts ≈ 20°).
///
/// # Arguments
///
/// * `cdu` — current CDU angles `[outer, inner, middle]`.
///
/// # Returns
///
/// `true` if the middle gimbal (index 2) is within 20° of ±90°.
///
/// # AGC source
///
/// `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc` — IMODES33/IMUMON monitor.
pub fn is_gimbal_lock_warning(cdu: &[CduAngle; 3]) -> bool {
    const GIMBAL_LOCK_WARNING_BAND: u16 = 3641; // ≈ 20° in CDU counts
    const NINETY_DEG: u16 = 16384; // 90° in CDU counts
    const TWO_SEVENTY_DEG: u16 = 49152; // 270° (= -90°) in CDU counts

    let middle = cdu[2].0;

    // Wrapping distance to +90°: min of the two arc directions
    let dist_pos90 = middle
        .wrapping_sub(NINETY_DEG)
        .min(NINETY_DEG.wrapping_sub(middle));
    // Wrapping distance to -90° / 270°
    let dist_neg90 = middle
        .wrapping_sub(TWO_SEVENTY_DEG)
        .min(TWO_SEVENTY_DEG.wrapping_sub(middle));

    dist_pos90 < GIMBAL_LOCK_WARNING_BAND || dist_neg90 < GIMBAL_LOCK_WARNING_BAND
}

// ── is_gimbal_lock_critical ───────────────────────────────────────────────────

/// Check whether the middle gimbal is in the critical zone (within 5° of ±90°).
///
/// This is the high-priority variant of `is_gimbal_lock_warning`. When this
/// returns `true`, the platform is within 5° of gimbal lock and immediate
/// crew action is required.
///
/// Critical band: GIMBAL_LOCK_CRITICAL_BAND = 910 counts ≈ 5°.
///
/// # Returns
///
/// `true` if the middle gimbal is within 5° of ±90°.
///
/// # AGC source
///
/// `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc` — IMUMON phase 3 critical check.
pub fn is_gimbal_lock_critical(cdu: &[CduAngle; 3]) -> bool {
    const GIMBAL_LOCK_CRITICAL_BAND: u16 = 910; // ≈ 5° in CDU counts
    const NINETY_DEG: u16 = 16384;
    const TWO_SEVENTY_DEG: u16 = 49152;

    let middle = cdu[2].0;

    let dist_pos90 = middle
        .wrapping_sub(NINETY_DEG)
        .min(NINETY_DEG.wrapping_sub(middle));
    let dist_neg90 = middle
        .wrapping_sub(TWO_SEVENTY_DEG)
        .min(TWO_SEVENTY_DEG.wrapping_sub(middle));

    dist_pos90 < GIMBAL_LOCK_CRITICAL_BAND || dist_neg90 < GIMBAL_LOCK_CRITICAL_BAND
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::linalg::{mxm, norm, transpose, IDENTITY};

    fn assert_near(a: f64, b: f64, tol: f64, label: &str) {
        assert!(
            (a - b).abs() < tol,
            "{label}: got {a:.9}, expected {b:.9}, tol {tol:.2e}"
        );
    }

    fn assert_vec_near(a: Vec3, b: Vec3, tol: f64, label: &str) {
        for i in 0..3 {
            assert!(
                (a[i] - b[i]).abs() < tol,
                "{label}[{i}]: got {:.9}, expected {:.9}, tol {tol:.2e}",
                a[i],
                b[i]
            );
        }
    }

    // ── TC-IMU-CTRL-1: Zero-bias PIPA compensation ────────────────────────────

    /// TC-IMU-CTRL-1: With zero bias, identity misalignment, and scale = 1.0,
    /// output exactly equals the raw counts.
    #[test]
    fn tc_imu_ctrl_1_pipa_zero_bias() {
        let raw: [i16; 3] = [100, -50, 30];
        let cal = PipaCalibration {
            scale: 1.0,
            bias: [0, 0, 0],
            misalignment: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        };
        let result = apply_pipa_compensation(raw, &cal);
        assert_near(result[0], 100.0, 1e-12, "TC-IMU-CTRL-1 X");
        assert_near(result[1], -50.0, 1e-12, "TC-IMU-CTRL-1 Y");
        assert_near(result[2], 30.0, 1e-12, "TC-IMU-CTRL-1 Z");
    }

    // ── TC-IMU-CTRL-2: Non-zero bias PIPA compensation ────────────────────────

    /// TC-IMU-CTRL-2: When raw counts equal the bias, output is zero.
    #[test]
    fn tc_imu_ctrl_2_pipa_nonzero_bias() {
        let raw: [i16; 3] = [5, -3, 2];
        let cal = PipaCalibration {
            scale: 0.0585,
            bias: [5, -3, 2],
            misalignment: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        };
        let result = apply_pipa_compensation(raw, &cal);
        assert!(
            result[0].abs() < 1e-12,
            "TC-IMU-CTRL-2 X should be zero: {}",
            result[0]
        );
        assert!(
            result[1].abs() < 1e-12,
            "TC-IMU-CTRL-2 Y should be zero: {}",
            result[1]
        );
        assert!(
            result[2].abs() < 1e-12,
            "TC-IMU-CTRL-2 Z should be zero: {}",
            result[2]
        );
    }

    // ── TC-IMU-CTRL-3: Coarse align wrap-around ───────────────────────────────

    /// TC-IMU-CTRL-3: Target = 0, current = 65000 → shortest path wraps
    /// counter-clockwise by 536 counts (not −65000 clockwise).
    #[test]
    fn tc_imu_ctrl_3_coarse_align_wraparound() {
        let target = [CduAngle(0), CduAngle(0), CduAngle(0)];
        let current = [CduAngle(1000), CduAngle(500), CduAngle(65000)];
        let cmds = coarse_align_step(target, current);
        // 0.wrapping_sub(1000) as i16 = -1000
        assert_eq!(cmds[0], -1000_i16, "TC-IMU-CTRL-3 outer axis");
        // 0.wrapping_sub(500) as i16 = -500
        assert_eq!(cmds[1], -500_i16, "TC-IMU-CTRL-3 inner axis");
        // 0u16.wrapping_sub(65000) = 536; as i16 = 536
        assert_eq!(cmds[2], 536_i16, "TC-IMU-CTRL-3 middle axis wrap");
    }

    // ── TC-IMU-CTRL-4: Gyro drift compensation ────────────────────────────────

    /// TC-IMU-CTRL-4: nbdx = 0.001 rad/s over 1000 cs (10 s) → expected pulse count.
    ///
    /// radians_drift = 0.001 * 10.0 = 0.01 rad
    /// pulses = 0.01 * 32768 / TAU ≈ 52.156 → truncated to 52 as i16
    /// returned as -52 (opposing drift)
    #[test]
    fn tc_imu_ctrl_4_gyro_drift_compensation() {
        let nbd = [0.001_f64, 0.0, 0.0];
        let dt_cs = 1000_u32; // 10 seconds
        let pulses = compute_gyro_drift(dt_cs, nbd);

        // Expected: -(0.001 * 10.0 * 32768 / TAU) truncated to i16
        let expected_magnitude = 0.001 * 10.0 * 32768.0 / TAU; // ≈ 52.156
        let expected_px = -(expected_magnitude as i16); // truncation toward zero → -52
        assert_eq!(pulses[0], expected_px, "TC-IMU-CTRL-4 X pulse count");
        assert_eq!(pulses[1], 0_i16, "TC-IMU-CTRL-4 Y should be 0");
        assert_eq!(pulses[2], 0_i16, "TC-IMU-CTRL-4 Z should be 0");
    }

    // ── TC-IMU-CTRL-5: REFSMMAT orthonormality ────────────────────────────────

    /// TC-IMU-CTRL-5: For identical inertial and platform triads, REFSMMAT
    /// should be identity; and the result must be orthonormal.
    #[test]
    fn tc_imu_ctrl_5_refsmmat_orthonormal() {
        let s1_iner: Vec3 = [1.0, 0.0, 0.0];
        let s1_plat: Vec3 = [1.0, 0.0, 0.0];
        let s2_iner: Vec3 = [0.0, 1.0, 0.0];
        let s2_plat: Vec3 = [0.0, 1.0, 0.0];

        let refsmmat = refsmmat_from_star_sightings(s1_iner, s2_iner, s1_plat, s2_plat)
            .expect("Non-collinear stars must produce a valid REFSMMAT");

        // Orthonormality: R · R^T = I
        let product = mxm(refsmmat, transpose(refsmmat));
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (product[i][j] - expected).abs() < 1e-12,
                    "TC-IMU-CTRL-5 orthonormality [{i}][{j}]: got {}, expected {expected}",
                    product[i][j]
                );
            }
        }

        // When platform = inertial, REFSMMAT should be identity
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (refsmmat[i][j] - expected).abs() < 1e-12,
                    "TC-IMU-CTRL-5 identity [{i}][{j}]: got {}, expected {expected}",
                    refsmmat[i][j]
                );
            }
        }
    }

    // ── TC-IMU-CTRL-6: Gimbal lock warning threshold ──────────────────────────

    /// TC-IMU-CTRL-6: Warning fires at 75° (inside 20° band) but not at 65°
    /// (outside 20° band). Critical does not fire at 75°.
    #[test]
    fn tc_imu_ctrl_6_gimbal_lock_warning() {
        // At +90° (16384 counts) — inside both warning and critical zones
        let at_90 = [CduAngle(0), CduAngle(0), CduAngle(16384)];
        assert!(
            is_gimbal_lock_warning(&at_90),
            "TC-IMU-CTRL-6: warning at 90°"
        );
        assert!(
            is_gimbal_lock_critical(&at_90),
            "TC-IMU-CTRL-6: critical at 90°"
        );

        // At 70° (12288 counts): distance to 90° = 4096 > 3641 → outside warning zone
        let at_70 = [CduAngle(0), CduAngle(0), CduAngle(12288)];
        assert!(
            !is_gimbal_lock_warning(&at_70),
            "TC-IMU-CTRL-6: no warning at 70°"
        );
        assert!(
            !is_gimbal_lock_critical(&at_70),
            "TC-IMU-CTRL-6: no critical at 70°"
        );

        // At 75° (13653 counts): distance to 90° = 2731 < 3641 → inside warning zone
        //                         2731 > 910 → outside critical zone
        let at_75 = [CduAngle(0), CduAngle(0), CduAngle(13653)];
        assert!(
            is_gimbal_lock_warning(&at_75),
            "TC-IMU-CTRL-6: warning at 75°"
        );
        assert!(
            !is_gimbal_lock_critical(&at_75),
            "TC-IMU-CTRL-6: no critical at 75°"
        );

        // At 0° — safe (no warning, no critical)
        let at_0 = [CduAngle(0), CduAngle(0), CduAngle(0)];
        assert!(
            !is_gimbal_lock_warning(&at_0),
            "TC-IMU-CTRL-6: no warning at 0°"
        );
        assert!(
            !is_gimbal_lock_critical(&at_0),
            "TC-IMU-CTRL-6: no critical at 0°"
        );
    }

    // ── TC-IMU-CTRL-7: Gimbal lock critical threshold ─────────────────────────

    /// TC-IMU-CTRL-7: Critical fires at 87° (within 5° of 90°) but not at 75°
    /// (warning only). Also checks the -90° (270°) singularity.
    #[test]
    fn tc_imu_ctrl_7_gimbal_lock_critical() {
        // At 86° (15655 counts): distance to 90° = 729 < 910 → inside critical
        let at_86 = [CduAngle(0), CduAngle(0), CduAngle(15655)];
        assert!(
            is_gimbal_lock_warning(&at_86),
            "TC-IMU-CTRL-7: warning at 86°"
        );
        assert!(
            is_gimbal_lock_critical(&at_86),
            "TC-IMU-CTRL-7: critical at 86°"
        );

        // At 75° — warning only, not critical
        let at_75 = [CduAngle(0), CduAngle(0), CduAngle(13653)];
        assert!(
            is_gimbal_lock_warning(&at_75),
            "TC-IMU-CTRL-7: warning at 75°"
        );
        assert!(
            !is_gimbal_lock_critical(&at_75),
            "TC-IMU-CTRL-7: no critical at 75°"
        );

        // At 270° (49152 counts) — exactly at -90°, inside both zones
        let at_270 = [CduAngle(0), CduAngle(0), CduAngle(49152)];
        assert!(
            is_gimbal_lock_warning(&at_270),
            "TC-IMU-CTRL-7: warning at 270°"
        );
        assert!(
            is_gimbal_lock_critical(&at_270),
            "TC-IMU-CTRL-7: critical at 270°"
        );
    }

    // ── Additional: REFSMMAT collinear star rejection ─────────────────────────

    /// Collinear and anti-parallel stars must return None (TC-IMU-CTRL-7 spec).
    #[test]
    fn tc_imu_ctrl_7_collinear_star_rejection() {
        let star: Vec3 = [1.0, 0.0, 0.0];
        let result = refsmmat_from_star_sightings(star, star, star, star);
        assert!(result.is_none(), "Identical stars must return None");

        let anti: Vec3 = [-1.0, 0.0, 0.0];
        let result2 = refsmmat_from_star_sightings(star, anti, star, anti);
        assert!(result2.is_none(), "Anti-parallel stars must return None");
    }

    // ── Additional: fine_align_torque scale factor ────────────────────────────

    /// 10 arcminute error over one T4RUPT cycle → 2 pulse counts on X axis.
    #[test]
    fn tc_imu_ctrl_fine_align_scale() {
        let one_arcmin_rad = core::f64::consts::PI / (180.0 * 60.0);
        let ten_arcmin_rad = 10.0 * one_arcmin_rad;
        let error: Vec3 = [ten_arcmin_rad, 0.0, 0.0];
        let dt_s = 0.12; // one T4RUPT cycle
        let pulses = fine_align_torque(error, dt_s);
        // Expected: 10 arcmin * (32768/TAU) * 0.12 ≈ 1.82 → rounds to 2
        assert_eq!(
            pulses[0], 2_i16,
            "TC fine-align: expected 2 pulses for 10 arcmin"
        );
        assert_eq!(pulses[1], 0_i16);
        assert_eq!(pulses[2], 0_i16);
    }

    // ── Additional: PIPA compensation with misalignment matrix ───────────────

    /// Non-identity misalignment matrix routes X input to Y output.
    #[test]
    fn tc_imu_ctrl_pipa_misalignment() {
        let raw: [i16; 3] = [100, 0, 0];
        let cal = PipaCalibration {
            scale: 1.0,
            bias: [0, 0, 0],
            // Routes X → Y via a 90° swap (misalignment near identity in practice,
            // but this tests the matrix path)
            misalignment: [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
        };
        let result = apply_pipa_compensation(raw, &cal);
        assert_near(result[0], 0.0, 1e-12, "TC pipa misalign X");
        assert_near(result[1], 100.0, 1e-12, "TC pipa misalign Y");
        assert_near(result[2], 0.0, 1e-12, "TC pipa misalign Z");
    }
}
