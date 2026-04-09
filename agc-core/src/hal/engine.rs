//! SPS engine and TVC sub-trait.
//!
//! AGC source: TVCEXECUTIVE.agc — SPS gimbal steering.

use crate::types::CduAngle;

/// SPS (Service Propulsion System) main engine interface.
///
/// Provides SPS enable/disable and TVC (Thrust Vector Control) gimbal commands.
///
/// AGC source: TVCEXECUTIVE.agc — TVC gimbal position commands.
pub trait Engine {
    /// Enable or disable the SPS engine arm relay.
    fn set_engine_arm(&mut self, armed: bool);

    /// Command the SPS to ignition (requires arm first).
    fn ignite(&mut self);

    /// Cut off the SPS engine.
    fn cutoff(&mut self);

    /// Command SPS TVC gimbal position (pitch and yaw axes).
    ///
    /// `pitch` and `yaw` are gimbal drive counts (i16).
    /// Scale: 1 count ≈ 0.01° gimbal deflection (from TVC calibration).
    ///
    /// AGC source: TVCEXECUTIVE.agc — TVCPITCH/TVCYAW channel writes.
    fn command_gimbal(&mut self, pitch: i16, yaw: i16);

    /// Read current TVC gimbal positions (feedback).
    fn read_gimbal(&self) -> [CduAngle; 2];

    /// True if the engine is currently firing (chamber pressure > threshold).
    fn is_firing(&self) -> bool;
}
