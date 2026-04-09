//! IMU (Inertial Measurement Unit) sub-trait.
//!
//! AGC source: IMU_MODE_SWITCHING_ROUTINES.agc, IMU_CALIBRATION_AND_ALIGNMENT.agc.

use crate::types::CduAngle;

/// IMU interface: PIPA readings, CDU angle readback, gyro torque commands.
///
/// AGC source: IMU_MODE_SWITCHING_ROUTINES.agc — coarse/fine alignment modes.
pub trait Imu {
    /// Read and clear accumulated PIPA (Pulse Integrating Pendulous Accelerometer)
    /// delta-velocity counts since the last call. Returns [x, y, z] in raw counts.
    ///
    /// Scale: 1 count ≈ 0.0585 m/s (exact value from IMU calibration).
    ///
    /// AGC source: ERASABLE_ASSIGNMENTS.agc — PIPAX/PIPAY/PIPAZ counter cells.
    fn read_pipa(&mut self) -> [i16; 3];

    /// Read the current CDU (Coupling Data Unit) gimbal angles.
    ///
    /// Returns [inner (X), middle (Y), outer (Z)] gimbal angles in raw counts.
    /// Full revolution = 32768 counts.
    ///
    /// AGC source: ERASABLE_ASSIGNMENTS.agc — CDUX/CDUY/CDUZ.
    fn read_cdu(&self) -> [CduAngle; 3];

    /// Command gyro torque pulses for fine IMU alignment.
    ///
    /// `axis` is 0 (X), 1 (Y), or 2 (Z). `pulses` is signed pulse count.
    ///
    /// AGC source: IMU_CALIBRATION_AND_ALIGNMENT.agc — TORQE routine.
    fn torque_gyro(&mut self, axis: usize, pulses: i16);

    /// Enable or disable the IMU cage mode (drives gimbals to zero).
    fn set_caged(&mut self, caged: bool);

    /// Read the current IMU temperature in degrees Celsius (for thermal management).
    fn read_temperature(&self) -> f32;
}
