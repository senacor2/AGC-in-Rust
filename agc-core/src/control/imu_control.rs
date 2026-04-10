//! IMU coarse/fine alignment, gyro drift compensation, and PIPA compensation.

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
