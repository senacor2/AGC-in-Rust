//! Program alarm system.
//!
//! Alarms are 4-digit codes displayed on the DSKY PROG register.
//! See `tables::alarm_codes` for the full code list.

/// Current alarm state.
#[derive(Clone, Copy, Debug, Default)]
pub struct AlarmState {
    /// The most recent alarm code (0 = none).
    pub code: u16,
    /// Secondary alarm code (stores the previous alarm when a new one fires).
    pub code2: u16,
    /// True when the PROG alarm lamp is lit.
    pub lit: bool,
}

impl AlarmState {
    /// Raise an alarm. Saves the current code to `code2` and lights the lamp.
    pub fn raise(&mut self, code: u16) {
        self.code2 = self.code;
        self.code = code;
        self.lit = true;
    }

    /// Clear the alarm lamp (crew pressed RSET).
    pub fn reset(&mut self) {
        self.lit = false;
    }
}
