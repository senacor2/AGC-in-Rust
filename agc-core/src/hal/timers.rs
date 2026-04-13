//! Hardware timer control interface for AGC interrupt scheduling.
//!
//! AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
//!             TIME3 (octal 26) — Waitlist timer (T3RUPT)
//!             TIME4 (octal 27) — Periodic I/O timer (T4RUPT)
//!             TIME5 (octal 30) — DAP timer (T5RUPT)
//!             TIME6 (octal 31) — Jet timing timer (T6RUPT)
//!             T4RUPT_PROGRAM.agc: SETTIME4, 20MRUPT = OCT 37776 (16382 decimal)
//!             JET_SELECTION_LOGIC.agc: DELTATT3 = 16378 (60 ms), DELATT20 = 16382 (20 ms)

/// T3 POSMAX value — TIME3 loaded to this value on init to postpone first T3RUPT.
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc STARTSUB `CAF POSMAX; TS TIME3`.
pub const POSMAX: u16 = 0o37777; // 16383

/// T4 nominal init value (POSMAX - 2 = 16381).
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc STARTSUB `TIME4 := POSMAX - 2`.
pub const T4_INIT: u16 = POSMAX - 2;

/// T5 nominal init value (POSMAX - 3 = 16380).
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc STARTSUB `TIME5 := POSMAX - 3`.
pub const T5_INIT: u16 = POSMAX - 3;

/// Hardware timer control interface for AGC interrupt scheduling.
///
/// Maps to TIME3/TIME4/TIME5/TIME6 in the AGC hardware.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc (octal 26-31).
///             Comanche055/T4RUPT_PROGRAM.agc (SETTIME4).
///             Comanche055/JET_SELECTION_LOGIC.agc (DELTATT3, DELATT20, =14MS).
pub trait Timers {
    /// Set the T3 (Waitlist) timer period.
    ///
    /// `centiseconds`: time in centiseconds until T3RUPT fires.
    /// Range: 1..=32767 (fitting the AGC TIME3 15-bit register).
    ///
    /// AGC source: WAITLIST.agc `TS TIME3` at end of WAIT2.
    fn set_t3(&mut self, centiseconds: u16);

    /// Set the T4 (periodic I/O) timer period in centiseconds.
    ///
    /// Nominal: 12 cs (120 ms). Reset each T4RUPT cycle.
    ///
    /// AGC source: Comanche055/T4RUPT_PROGRAM.agc SETTIME4.
    fn set_t4(&mut self, centiseconds: u16);

    /// Set the T5 (DAP) timer period in centiseconds.
    ///
    /// DAP uses 2 cs (20 ms) or 6 cs (60 ms) depending on jet activity.
    ///
    /// AGC source: Comanche055/JET_SELECTION_LOGIC.agc DELTATT3 / DELATT20.
    fn set_t5(&mut self, centiseconds: u16);

    /// Set the T6 (jet timing) timer period in centiseconds.
    ///
    /// Minimum: approximately 1-2 cs (14 ms pulse).
    ///
    /// AGC source: Comanche055/JET_SELECTION_LOGIC.agc `=14MS` constant.
    fn set_t6(&mut self, centiseconds: u16);

    /// Read the current T3 count (remaining centiseconds).
    fn read_t3(&self) -> u16;

    /// Read the current T4 count.
    fn read_t4(&self) -> u16;

    /// Read the current T5 count.
    fn read_t5(&self) -> u16;

    /// Read the current T6 count.
    fn read_t6(&self) -> u16;

    /// Disable T3 interrupt (INHINT equivalent for Waitlist timer).
    fn disable_t3(&mut self);

    /// Enable T3 interrupt (RELINT equivalent).
    fn enable_t3(&mut self);
}

/// Bare-metal timer implementation skeleton.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc TIME3-TIME6 registers.
pub struct TimersImpl {
    t3: u16,
    t4: u16,
    t5: u16,
    t6: u16,
    t3_enabled: bool,
}

impl TimersImpl {
    /// Construct with timers at POSMAX (no immediate interrupt).
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc STARTSUB timer init.
    pub const fn new() -> Self {
        Self {
            t3: POSMAX,
            t4: T4_INIT,
            t5: T5_INIT,
            t6: POSMAX,
            t3_enabled: false,
        }
    }

    /// Release the underlying peripheral handles (C-FREE).
    pub fn free(self) -> (u16, u16, u16, u16) {
        (self.t3, self.t4, self.t5, self.t6)
    }
}

impl Default for TimersImpl {
    fn default() -> Self {
        Self::new()
    }
}

impl Timers for TimersImpl {
    fn set_t3(&mut self, centiseconds: u16) {
        self.t3 = centiseconds;
    }

    fn set_t4(&mut self, centiseconds: u16) {
        self.t4 = centiseconds;
    }

    fn set_t5(&mut self, centiseconds: u16) {
        self.t5 = centiseconds;
    }

    fn set_t6(&mut self, centiseconds: u16) {
        self.t6 = centiseconds;
    }

    fn read_t3(&self) -> u16 {
        self.t3
    }

    fn read_t4(&self) -> u16 {
        self.t4
    }

    fn read_t5(&self) -> u16 {
        self.t5
    }

    fn read_t6(&self) -> u16 {
        self.t6
    }

    fn disable_t3(&mut self) {
        self.t3_enabled = false;
    }

    fn enable_t3(&mut self) {
        self.t3_enabled = true;
    }
}
