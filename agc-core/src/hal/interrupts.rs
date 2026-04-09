/// AGC program interrupt sources, in priority order (lowest number = highest priority).
///
/// Mapped to hardware timer interrupts on the MCU via the device PAC's
/// `#[interrupt]` attribute. See architecture §4.2.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
#[repr(u8)]
pub enum Interrupt {
    /// TIME6 pulse. RCS jet on/off timing (0.625 ms resolution).
    T6Rupt = 1,
    /// TIME5 overflow. Digital autopilot computation cycle (~100 ms).
    T5Rupt = 2,
    /// TIME3 overflow. Waitlist task dispatch (10 ms base resolution).
    T3Rupt = 3,
    /// TIME4 overflow. Periodic I/O: DSKY update, IMU monitoring (120 ms).
    T4Rupt = 4,
    /// Keyboard input from the main DSKY.
    KeyRupt1 = 5,
    /// Keyboard input from the nav DSKY or optics mark pulse.
    KeyRupt2 = 6,
    /// Uplink word received from the ground.
    UplinkRupt = 7,
    /// Telemetry end pulse — ready for next downlink word pair.
    DownRupt = 8,
    /// Radar data ready.
    RadarRupt = 9,
    /// Hand controller or discrete input change.
    HandRupt = 10,
}
