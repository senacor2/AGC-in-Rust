/// Scheduler timer interface.
///
/// The three scheduling timers map to the AGC's TIME3, TIME5, and TIME6.
/// Implementations use MCU hardware timer peripherals and expose only the
/// arm/disarm interface; all register details are hidden.
pub trait Timers {
    /// Arm TIME3 (waitlist dispatch) to fire in `centiseconds` (1–16383 cs).
    /// The T3RUPT interrupt fires when the timer expires.
    /// Precondition: `1 <= centiseconds <= 16383`. Passing 0 or > 16383 is
    /// undefined on bare metal. Use long-waitlist chaining for delays > 163 s.
    fn arm_t3(&mut self, centiseconds: u16);

    /// Arm TIME5 (DAP cycle) to fire in `centiseconds`.
    /// The T5RUPT interrupt fires when the timer expires.
    fn arm_t5(&mut self, centiseconds: u16);

    /// Arm TIME6 (RCS jet pulse) to fire after `counts` × 0.625 ms.
    /// The T6RUPT interrupt fires when the timer decrements to zero.
    fn arm_t6(&mut self, counts: u16);

    /// Disarm TIME6 (cancel a pending jet pulse before it fires).
    fn disarm_t6(&mut self);

    /// Read the current mission elapsed time in centiseconds.
    fn mission_time(&self) -> u32;
}
