//! Digital Autopilot (DAP) supervisor state.

use crate::types::Vec3;

/// DAP operating mode.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum DapMode {
    /// DAP is off — no attitude control.
    #[default]
    Off,
    /// Rate damping — null body rates with RCS.
    RateDamping,
    /// Attitude hold — maintain a target attitude.
    AttitudeHold,
    /// Attitude maneuver — rotate to a commanded attitude.
    Maneuver,
    /// TVC mode — gimbal control during SPS burn.
    Tvc,
}

/// Digital Autopilot state (T5RUPT / T6RUPT context).
#[derive(Clone, Copy, Debug, Default)]
pub struct DapState {
    pub mode: DapMode,
    /// Attitude error angles (roll, pitch, yaw) in radians.
    pub attitude_error: Vec3,
    /// Estimated body rates (roll, pitch, yaw) in rad/s.
    pub rate_estimate: Vec3,
    /// Attitude deadband in radians.
    pub deadband: f64,
    /// Currently commanded RCS jet bitmask (SM jets).
    pub rcs_jet_flags: u16,
}
