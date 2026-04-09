//! IMU mode control: coarse align, fine align, operate.
//!
//! AGC source: IMU_MODE_SWITCHING_ROUTINES.agc, IMU_CALIBRATION_AND_ALIGNMENT.agc.

use crate::types::{CduAngle, Mat3x3};

/// Number of fine-align iterations required before declaring alignment complete.
///
/// AGC source: IMU_CALIBRATION_AND_ALIGNMENT.agc — gyro torquing loop.
const FINE_ALIGN_ITERS_REQUIRED: u8 = 10;

/// Maximum CDU error (counts) considered "aligned" for coarse align.
///
/// AGC source: IMU_MODE_SWITCHING_ROUTINES.agc — IMUCOARS coarse-align threshold.
const COARSE_ALIGN_THRESHOLD_COUNTS: i16 = 4;

/// IMU operational mode.
///
/// AGC source: IMU_MODE_SWITCHING_ROUTINES.agc — IMODES33/IMODES30 status words.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImuMode {
    /// IMU is off or not yet initialized.
    Off,
    /// Coarse align: gimbal angles driven to target via CDU counters.
    CoarseAlign,
    /// Fine align: gyro torquing to null platform drift.
    FineAlign,
    /// Operate: normal navigation mode, reading PIPA and CDU.
    Operate,
    /// Caged: gimbals locked (no navigation data valid).
    Caged,
}

/// IMU control state.
///
/// AGC source: IMU_MODE_SWITCHING_ROUTINES.agc — IMUZERO, IMUCOARS, IMUFINE.
#[derive(Clone, Copy, Debug)]
pub struct ImuControl {
    /// Current IMU operating mode.
    pub mode: ImuMode,
    /// Target CDU angles for coarse align \[outer, inner, middle\].
    pub target_cdu: [CduAngle; 3],
    /// Current CDU angles reported by hardware \[outer, inner, middle\].
    pub current_cdu: [CduAngle; 3],
    /// Desired stable member orientation (REFSMMAT).
    ///
    /// AGC source: IMU_CALIBRATION_AND_ALIGNMENT.agc — REFSMMAT storage.
    pub desired_sm: Mat3x3,
    /// Gyro torquing pulses remaining \[x, y, z\].
    ///
    /// AGC source: IMU_CALIBRATION_AND_ALIGNMENT.agc — gyro-torquing loop.
    pub gyro_torque_remaining: [i16; 3],
    /// Fine align iteration count.
    pub fine_align_iters: u8,
}

impl ImuControl {
    /// Construct a new `ImuControl` in the `Off` mode.
    pub const fn new() -> Self {
        Self {
            mode: ImuMode::Off,
            target_cdu: [CduAngle::ZERO; 3],
            current_cdu: [CduAngle::ZERO; 3],
            desired_sm: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            gyro_torque_remaining: [0; 3],
            fine_align_iters: 0,
        }
    }

    /// Begin coarse align to target CDU angles.
    ///
    /// Sets mode to `CoarseAlign` and stores the target gimbal angles.
    /// The hardware will drive CDU counters toward the target each `alignment_step`.
    ///
    /// AGC source: IMU_MODE_SWITCHING_ROUTINES.agc — IMUCOARS routine.
    pub fn start_coarse_align(&mut self, target: [CduAngle; 3]) {
        self.target_cdu = target;
        self.mode = ImuMode::CoarseAlign;
    }

    /// Begin fine align using gyro torquing to null residual platform drift.
    ///
    /// AGC source: IMU_CALIBRATION_AND_ALIGNMENT.agc — FINEALGN, gyro torquing.
    pub fn start_fine_align(&mut self, desired: Mat3x3) {
        self.desired_sm = desired;
        self.fine_align_iters = 0;
        // Compute nominal torque pulses from desired orientation error.
        // In this model we initialise to a non-zero value so each iteration
        // decrements toward zero (each step applies one pulse per axis).
        self.gyro_torque_remaining = [
            FINE_ALIGN_ITERS_REQUIRED as i16,
            FINE_ALIGN_ITERS_REQUIRED as i16,
            FINE_ALIGN_ITERS_REQUIRED as i16,
        ];
        self.mode = ImuMode::FineAlign;
    }

    /// Transition to operate mode.
    ///
    /// AGC source: IMU_MODE_SWITCHING_ROUTINES.agc — IMUOPERA entry.
    pub fn enter_operate(&mut self) {
        self.gyro_torque_remaining = [0; 3];
        self.mode = ImuMode::Operate;
    }

    /// Cage the IMU (lock gimbals).
    ///
    /// AGC source: IMU_MODE_SWITCHING_ROUTINES.agc — CAGETSTJ.
    pub fn cage(&mut self) {
        self.mode = ImuMode::Caged;
    }

    /// Process one alignment step.
    ///
    /// In `CoarseAlign` mode each call steps the simulated CDU counters one
    /// increment closer to the target. Returns `true` when all three CDU
    /// errors are within the coarse-align threshold.
    ///
    /// In `FineAlign` mode each call decrements one gyro-torque pulse per axis
    /// and increments the iteration counter. Returns `true` when the required
    /// number of iterations is reached and all torque pulses are consumed.
    ///
    /// In any other mode returns `false` immediately.
    ///
    /// AGC source: IMU_MODE_SWITCHING_ROUTINES.agc — IMUCOARS / IMUFINE loop;
    /// IMU_CALIBRATION_AND_ALIGNMENT.agc — FINEALGN torquing step.
    pub fn alignment_step(&mut self) -> bool {
        match self.mode {
            ImuMode::CoarseAlign => {
                let mut all_aligned = true;
                for i in 0..3 {
                    let err = self.target_cdu[i].signed_diff(self.current_cdu[i]);
                    if err.abs() > COARSE_ALIGN_THRESHOLD_COUNTS {
                        all_aligned = false;
                        // Step current CDU one count toward target.
                        let step: i16 = if err > 0 { 1 } else { -1 };
                        self.current_cdu[i] =
                            CduAngle(self.current_cdu[i].0.wrapping_add(step as u16));
                    }
                }
                all_aligned
            }
            ImuMode::FineAlign => {
                // Apply one torque pulse per axis.
                for pulse in self.gyro_torque_remaining.iter_mut() {
                    if *pulse > 0 {
                        *pulse -= 1;
                    } else if *pulse < 0 {
                        *pulse += 1;
                    }
                }
                self.fine_align_iters = self.fine_align_iters.saturating_add(1);
                self.fine_align_iters >= FINE_ALIGN_ITERS_REQUIRED
                    && self.gyro_torque_remaining == [0; 3]
            }
            _ => false,
        }
    }
}

impl Default for ImuControl {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_state_is_off_mode() {
        let imu = ImuControl::new();
        assert_eq!(imu.mode, ImuMode::Off);
    }

    #[test]
    fn start_coarse_align_transitions_mode() {
        let mut imu = ImuControl::new();
        let target = [CduAngle(100), CduAngle(200), CduAngle(300)];
        imu.start_coarse_align(target);
        assert_eq!(imu.mode, ImuMode::CoarseAlign);
        assert_eq!(imu.target_cdu, target);
    }

    #[test]
    fn enter_operate_transitions_to_operate() {
        let mut imu = ImuControl::new();
        imu.enter_operate();
        assert_eq!(imu.mode, ImuMode::Operate);
    }

    #[test]
    fn coarse_align_step_converges_and_returns_true() {
        let mut imu = ImuControl::new();
        // Target 10 counts ahead; threshold is 4, so needs at least ~6 steps.
        let target = [CduAngle(10), CduAngle(0), CduAngle(0)];
        imu.start_coarse_align(target);
        // Run until complete (bounded to avoid infinite loop in test).
        let mut done = false;
        for _ in 0..200 {
            if imu.alignment_step() {
                done = true;
                break;
            }
        }
        assert!(done, "coarse align did not complete");
    }

    #[test]
    fn fine_align_step_returns_true_when_complete() {
        let mut imu = ImuControl::new();
        let identity: Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        imu.start_fine_align(identity);
        let mut done = false;
        for _ in 0..200 {
            if imu.alignment_step() {
                done = true;
                break;
            }
        }
        assert!(done, "fine align did not complete");
        assert_eq!(imu.gyro_torque_remaining, [0; 3]);
    }

    #[test]
    fn cage_transitions_to_caged() {
        let mut imu = ImuControl::new();
        imu.enter_operate();
        imu.cage();
        assert_eq!(imu.mode, ImuMode::Caged);
    }

    #[test]
    fn alignment_step_in_operate_returns_false() {
        let mut imu = ImuControl::new();
        imu.enter_operate();
        assert!(!imu.alignment_step());
    }
}
