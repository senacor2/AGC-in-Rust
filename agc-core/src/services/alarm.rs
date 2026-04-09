//! Program alarm system.
//!
//! The AGC alarm system raises numeric codes to alert the crew and ground to
//! abnormal conditions. Alarms 1202/1210/1211 became famous during Apollo 11
//! (Executive overflow from rendezvous radar) and are explicitly preserved.
//!
//! When an alarm fires:
//! 1. The code is stored in `FAILREG` (last-alarm register).
//! 2. The PROG light on the DSKY is illuminated.
//! 3. If the alarm is recoverable, execution continues.
//! 4. If the alarm is fatal (ABORT), a restart is triggered.
//!
//! AGC source: ALARM_AND_ABORT.agc — ALARM/ABORT routines.

/// Known alarm codes from Comanche055.
///
/// AGC source: ALARM_AND_ABORT.agc — alarm code table.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u16)]
pub enum AlarmCode {
    /// Executive overflow: no empty VAC (Vector Accumulator) area. Crew-visible.
    ///
    /// Triggered by NOVAC when the 7-slot CORE SET job table is full.
    /// Became famous during Apollo 11 when rendezvous radar load caused
    /// repeated 1202 alarms during the LM descent; the computer recovered
    /// by shedding lower-priority work.
    ///
    /// AGC source: ALARM_AND_ABORT.agc, code 01202; EXECUTIVE.agc — NOVAC.
    ExecutiveOverflow = 1202,
    /// Waitlist overflow: no empty task slots.
    ///
    /// AGC source: ALARM_AND_ABORT.agc, code 1210.
    WaitlistOverflow = 1210,
    /// Erasable memory checksum failure.
    ///
    /// AGC source: ALARM_AND_ABORT.agc, code 1211.
    ErasableChecksum = 1211,
    /// IMU failure (CDU angle out of range).
    ImuFailure = 0x0214,
    /// Display failure (DSKY relay timeout).
    DisplayFailure = 0x0224,
    /// Unknown / software error catch-all.
    SoftwareError = 0x01400,
}

/// Severity of an alarm.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AlarmSeverity {
    /// Warn the crew; continue execution.
    Recoverable,
    /// Trigger a restart after preserving state.
    Fatal,
}

impl AlarmCode {
    pub fn severity(self) -> AlarmSeverity {
        match self {
            AlarmCode::ExecutiveOverflow | AlarmCode::WaitlistOverflow => {
                AlarmSeverity::Recoverable
            }
            AlarmCode::ErasableChecksum
            | AlarmCode::ImuFailure
            | AlarmCode::DisplayFailure
            | AlarmCode::SoftwareError => AlarmSeverity::Fatal,
        }
    }
}

/// Alarm state: last-raised code and pending-alarm flag.
///
/// AGC source: ERASABLE_ASSIGNMENTS.agc — FAILREG alarm register.
pub struct AlarmState {
    /// The most recently raised alarm code, if any.
    last_alarm: Option<AlarmCode>,
    /// Set of raised alarm codes (bit-flags over a small fixed array).
    raised: [bool; 6],
}

const ALARM_TABLE: [AlarmCode; 6] = [
    AlarmCode::ExecutiveOverflow,
    AlarmCode::WaitlistOverflow,
    AlarmCode::ErasableChecksum,
    AlarmCode::ImuFailure,
    AlarmCode::DisplayFailure,
    AlarmCode::SoftwareError,
];

impl Default for AlarmState {
    fn default() -> Self {
        Self::new()
    }
}

impl AlarmState {
    pub const fn new() -> Self {
        Self {
            last_alarm: None,
            raised: [false; 6],
        }
    }

    /// Raise an alarm.
    ///
    /// Stores the code in FAILREG and sets the pending flag.
    /// Fatal alarms must trigger a restart at the call site (the alarm system
    /// cannot perform the restart itself as it needs hardware access).
    ///
    /// AGC source: ALARM_AND_ABORT.agc — ALARM routine.
    pub fn raise(&mut self, code: AlarmCode) {
        self.last_alarm = Some(code);
        for (i, &ac) in ALARM_TABLE.iter().enumerate() {
            if ac == code {
                self.raised[i] = true;
                break;
            }
        }
    }

    /// True if the given alarm code has been raised since last clear.
    pub fn is_raised(&self, code: AlarmCode) -> bool {
        for (i, &ac) in ALARM_TABLE.iter().enumerate() {
            if ac == code {
                return self.raised[i];
            }
        }
        false
    }

    /// The most recently raised alarm, if any.
    pub fn last_alarm(&self) -> Option<AlarmCode> {
        self.last_alarm
    }

    /// Clear all raised alarms. Called after crew acknowledgement or restart.
    ///
    /// AGC source: ALARM_AND_ABORT.agc — CLRALARM.
    pub fn clear_all(&mut self) {
        self.last_alarm = None;
        for r in &mut self.raised {
            *r = false;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raise_and_check() {
        let mut state = AlarmState::new();
        state.raise(AlarmCode::ExecutiveOverflow);
        assert!(state.is_raised(AlarmCode::ExecutiveOverflow));
        assert!(!state.is_raised(AlarmCode::WaitlistOverflow));
        assert_eq!(state.last_alarm(), Some(AlarmCode::ExecutiveOverflow));
    }

    #[test]
    fn clear_all_resets() {
        let mut state = AlarmState::new();
        state.raise(AlarmCode::WaitlistOverflow);
        state.clear_all();
        assert!(!state.is_raised(AlarmCode::WaitlistOverflow));
        assert_eq!(state.last_alarm(), None);
    }

    #[test]
    fn multiple_alarms() {
        let mut state = AlarmState::new();
        state.raise(AlarmCode::ExecutiveOverflow);
        state.raise(AlarmCode::WaitlistOverflow);
        assert!(state.is_raised(AlarmCode::ExecutiveOverflow));
        assert!(state.is_raised(AlarmCode::WaitlistOverflow));
        // last_alarm is most recent
        assert_eq!(state.last_alarm(), Some(AlarmCode::WaitlistOverflow));
    }

    #[test]
    fn severity_classification() {
        assert_eq!(
            AlarmCode::ExecutiveOverflow.severity(),
            AlarmSeverity::Recoverable
        );
        assert_eq!(
            AlarmCode::WaitlistOverflow.severity(),
            AlarmSeverity::Recoverable
        );
        assert_eq!(AlarmCode::ErasableChecksum.severity(), AlarmSeverity::Fatal);
    }
}
