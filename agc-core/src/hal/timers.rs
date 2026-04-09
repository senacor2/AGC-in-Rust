//! Timer sub-trait: T3, T4, T5, T6 scheduling timers.
//!
//! AGC source: INTERRUPT_LEAD_INS.agc — T3RUPT/T4RUPT/T5RUPT/T6RUPT handlers.

/// Scheduler timer control.
///
/// The AGC has four hardware timers that drive interrupt-based task dispatch.
/// | Timer | Period      | Purpose                              |
/// |-------|-------------|--------------------------------------|
/// | T3    | 10 ms       | Waitlist task dispatch (T3RUPT)      |
/// | T4    | 7.5 ms      | Periodic I/O: DSKY, IMU (T4RUPT)    |
/// | T5    | configurable| Digital autopilot cycle (T5RUPT)     |
/// | T6    | configurable| RCS jet timing (T6RUPT)              |
pub trait Timers {
    /// Load T3 to fire in `centiseconds` from now (10 ms = 1 cs minimum).
    ///
    /// The Waitlist calls this after dispatching the front task to re-arm
    /// the timer for the next task's delta-time.
    fn arm_t3(&mut self, centiseconds: u16);

    /// Return the current TIME3 counter value (centiseconds remaining).
    fn read_t3(&self) -> u16;

    /// Arm T5 to fire in `centiseconds` from now.
    fn arm_t5(&mut self, centiseconds: u16);

    /// Arm T6 to fire in `centiseconds` from now. Used by RCS jet sequencer.
    fn arm_t6(&mut self, centiseconds: u16);

    /// Disable the T6 timer (no pending jet pulse).
    fn disarm_t6(&mut self);
}
