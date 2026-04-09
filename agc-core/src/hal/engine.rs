/// SPS (Service Propulsion System) engine interface.
pub trait Engine {
    /// Enable or disable the SPS engine (ignition / cutoff).
    fn sps_enable(&mut self, on: bool);

    /// Command SPS engine gimbal angles for Thrust Vector Control.
    /// `pitch` and `yaw` are signed counts; the scale and polarity are
    /// defined by the TVC module.
    fn sps_gimbal(&mut self, pitch: i16, yaw: i16);

    /// Return true if the SPS engine is currently thrusting
    /// (as reported by the thrust-on discrete).
    fn thrust_on(&self) -> bool;
}
