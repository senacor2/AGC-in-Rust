//! Thrust Vector Control state for SPS burns.

/// TVC (Thrust Vector Control) state.
#[derive(Clone, Copy, Debug, Default)]
pub struct TvcState {
    /// Commanded pitch gimbal angle (radians).
    pub gimbal_pitch: f64,
    /// Commanded yaw gimbal angle (radians).
    pub gimbal_yaw: f64,
    /// Pitch trim bias (hardware counts).
    pub trim_pitch: i16,
    /// Yaw trim bias (hardware counts).
    pub trim_yaw: i16,
}
