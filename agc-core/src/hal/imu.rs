use crate::types::CduAngle;

/// Inertial Measurement Unit interface.
///
/// Exposes the three functional aspects of the IMU:
/// - PIPA (Pulse-Integrating Pendulous Accelerometer) delta-V accumulation
/// - CDU (Coupling Data Unit) gimbal angle readout
/// - Gyro torque commands for fine alignment
pub trait Imu {
    /// Read and reset the PIPA delta-V pulse counts accumulated since the last
    /// call (destructive read — counters are zeroed). Returns [x, y, z] counts;
    /// each count ≈ 0.0585 m/s on the real AGC.
    ///
    /// Called by `Executive::run` on every foreground iteration; the counts
    /// are saturating-accumulated into `AgcState::pipa_counts`. The SERVICER
    /// (`services::average_g`) consumes that staging field on its 2-second
    /// cycle and resets it. This pattern handles the destructive-read
    /// semantics correctly without requiring exactly-on-time SERVICER calls.
    fn read_pipa(&mut self) -> [i16; 3];

    /// Read the three IMU CDU gimbal angles (outer, inner, middle).
    fn read_cdu(&self) -> [CduAngle; 3];

    /// Command gyro torque pulses for fine alignment on the given axis (0=X, 1=Y, 2=Z).
    /// `pulses` is signed; positive and negative torque directions as per the
    /// gyro polarity convention in imu_control.
    fn torque_gyro(&mut self, axis: usize, pulses: i16);

    /// Command coarse CDU drive angles for platform slew.
    fn coarse_align(&mut self, commands: [i16; 3]);

    /// Return true if the IMU is powered and the platform is caged.
    fn is_caged(&self) -> bool;
}
