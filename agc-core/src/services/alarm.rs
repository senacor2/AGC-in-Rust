//! Program alarm system — FAILREG ring buffer, alarm codes, and PROG light.
//!
//! AGC source: Comanche055/ALARM_AND_ABORT.agc
//! Routines:   ALARM, ALARM2, BAILOUT, POODOO, VARALARM, CURTAINS, PROGLARM, BORTENT
//! Pages:      1493-1496
//!
//! Secondary: EXECUTIVE.agc pp. 1211-1212 (codes 1201, 1202)
//!            WAITLIST.agc pp. 1222-1223 (codes 1203, 1204)
//!            IMU_MODE_SWITCHING_ROUTINES.agc p. 1441-1442 (code 1210)
//!            FRESH_START_AND_RESTART.agc pp. 186-189 (codes 1107, 1110)
//!            ERASABLE_ASSIGNMENTS.agc line 1721 (FAILREG ERASE +2, 3-word ring)

use crate::sync::Mutex;
use core::cell::RefCell;

/// Program alarm code.
///
/// Values are the octal constants from ALARM_AND_ABORT.agc and the files that
/// raise them.  The discriminant is the octal value cast to `u16`.
///
/// AGC source: Comanche055/ALARM_AND_ABORT.agc, EXECUTIVE.agc, WAITLIST.agc,
///             IMU_MODE_SWITCHING_ROUTINES.agc, FRESH_START_AND_RESTART.agc.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u16)]
pub enum AlarmCode {
    /// No VAC area available (Executive overflow).
    ///
    /// AGC source: Comanche055/EXECUTIVE.agc FINDVAC2, `TC BAILOUT / OCT 1201`.
    NoVacArea = 0o1201,

    /// No core set available (Executive overflow).
    ///
    /// AGC source: Comanche055/EXECUTIVE.agc NOVAC3, `TC BAILOUT / OCT 1202`.
    NoCoreSets = 0o1202,

    /// Waitlist overflow — more than 9 tasks.
    ///
    /// AGC source: Comanche055/WAITLIST.agc WTABORT, `TC BAILOUT / OCT 1203`.
    WaitlistOverflow = 0o1203,

    /// Waitlist called with zero or negative delta-T.
    ///
    /// AGC source: Comanche055/WAITLIST.agc WATLST0-, `TC POODOO / OCT 1204`.
    WaitlistNegDt = 0o1204,

    /// Two jobs attempting to sleep on DSKY simultaneously.
    ///
    /// AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc DSPABORT,
    ///             `TC POODOO / OCT 1206`.
    DspDoubleSleep = 0o1206,

    /// Phase table complement mismatch on restart.
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc PTBAD,
    ///             `TC ALARM / OCT 1107`.
    PhaseTableError = 0o1107,

    /// Restart with no active restart groups.
    ///
    /// Documented in FRESH_START_AND_RESTART.agc header (p. 182 and 134).
    /// No explicit `OCT 1110` in source — handled via GOTOPOOH / V50N07 path.
    RestartNoActiveGroups = 0o1110,

    /// Two programs contending for same IMU/attitude device.
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc MODABORT/GOMANUR,
    ///             `TC POODOO / OCT 1210`.
    DeviceConflict = 0o1210,

    /// Erasable memory checksum failure (reserved; NOT raised in Milestone 1).
    ///
    /// Would be raised by ERASCHK/SELFCHK path.  No explicit `OCT 1211` in source.
    ErasableChecksum = 0o1211,

    /// V37 major-mode change attempted while NODOFLAG is set.
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc CANTR00,
    ///             `TC ALARM / OCT 1520`.
    MmChangeNotAllowed = 0o1520,

    /// Internal consistency error (CCSHOLE trap).
    ///
    /// AGC source: Comanche055/ALARM_AND_ABORT.agc CCSHOLE, OCT 1103.
    CcsHole = 0o1103,

    /// IMU CDU zero attempted while in gimbal lock + coarse align.
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc IMUZERO,
    ///             `TC ALARM / OCT 00206`.
    ImuZeroInGimbalLock = 0o0206,

    /// CURTAINS safety alarm.
    ///
    /// AGC source: Comanche055/ALARM_AND_ABORT.agc CURTAINS, OCT 217.
    Curtains = 0o0217,

    /// PIPA accumulator overflow (saturation) in SERVICER.
    ///
    /// Raised when any PIPA axis count exceeds PIPA_MAX_COUNTS (6398) during
    /// a 2-second Average-G cycle. CALCRVG is skipped; state is preserved.
    ///
    /// AGC source: Comanche055/SERVICER207.agc, `TC ALARM / OCT 00205`
    ///             (fallback path when PIPA saturates, before `TC AVERAGEG`).
    PipaOverflow = 0o0205,
}

impl AlarmCode {
    /// Return the raw octal `u16` value of this alarm code.
    pub const fn value(self) -> u16 {
        self as u16
    }

    /// Return the severity classification of this alarm code.
    ///
    /// Determines whether execution continues, a soft restart occurs, or a
    /// bailout (warm restart) is triggered.
    ///
    /// AGC source: Comanche055/ALARM_AND_ABORT.agc ALARM / BAILOUT / POODOO entry logic.
    pub const fn severity(self) -> AlarmSeverity {
        match self {
            AlarmCode::NoVacArea | AlarmCode::NoCoreSets | AlarmCode::WaitlistOverflow => {
                AlarmSeverity::Bailout
            }

            AlarmCode::WaitlistNegDt
            | AlarmCode::DspDoubleSleep
            | AlarmCode::DeviceConflict
            | AlarmCode::ErasableChecksum
            | AlarmCode::CcsHole => AlarmSeverity::SoftRestart,

            AlarmCode::PhaseTableError
            | AlarmCode::RestartNoActiveGroups
            | AlarmCode::MmChangeNotAllowed
            | AlarmCode::ImuZeroInGimbalLock
            | AlarmCode::Curtains
            | AlarmCode::PipaOverflow => AlarmSeverity::Continue,
        }
    }
}

/// Severity of an alarm code, determining the restart response.
///
/// AGC source: Comanche055/ALARM_AND_ABORT.agc alarm dispatch table.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AlarmSeverity {
    /// Record in history and light PROG; execution continues.
    ///
    /// AGC source: `TC ALARM` / `TC VARALARM` path.
    Continue,

    /// Soft restart: clear phase tables, run ENEMA (warm restart).
    ///
    /// AGC source: `TC POODOO` path → WHIMPER → ENEMA.
    SoftRestart,

    /// Bailout: preserve erasables snapshot, run ENEMA (warm restart).
    ///
    /// AGC source: `TC BAILOUT` path → BORTENT → WHIMPER → ENEMA.
    Bailout,
}

// ── AlarmState ────────────────────────────────────────────────────────────────

/// Fixed-size alarm history ring buffer, matching FAILREG (3 words) in AGC erasable memory.
///
/// Capacity is 3, matching the three FAILREG registers.  On the fourth alarm the oldest
/// slot is evicted and `overflow` is set.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc line 1721
/// `FAILREG ERASE +2  # B(3)PRM 3 ALARM CODE REGISTERS`
pub struct AlarmState {
    /// Ring buffer: `entries[head % 3]` is the next write position.
    entries: [Option<AlarmCode>; 3],
    /// Index of next write position (0..=u8::MAX, modulo 3).
    head: u8,
    /// Number of valid entries currently stored (saturates at 3).
    count: u8,
    /// Set true once more than 3 alarms have been raised without a reset.
    ///
    /// Mirrors the AGC BIT15 marker written into FAILREG+2.
    overflow: bool,
    /// True when the PROG alarm light should be illuminated.
    ///
    /// AGC source: Comanche055/ALARM_AND_ABORT.agc PROGLARM routine.
    prog_light: bool,
}

impl AlarmState {
    /// Construct the initial (all-clear) state.
    ///
    /// Called during `fresh_start` to reset all alarms.
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc SKIPSIM block,
    ///   lines 159-163 (FAILREG zeroing).
    pub const fn new() -> Self {
        Self {
            entries: [None; 3],
            head: 0,
            count: 0,
            overflow: false,
            prog_light: false,
        }
    }

    /// Record an alarm and illuminate the PROG light (internal method).
    ///
    /// Stores code in the ring buffer (overwrites oldest on overflow).
    /// Sets `prog_light = true` on the first alarm since reset.
    ///
    /// AGC source: Comanche055/ALARM_AND_ABORT.agc ALARM routine (page 1493).
    pub(crate) fn record(&mut self, code: AlarmCode) -> AlarmSeverity {
        if self.count < 3 {
            self.entries[self.head as usize % 3] = Some(code);
            self.head = self.head.wrapping_add(1);
            self.count += 1;
        } else {
            // Ring buffer full: evict oldest.
            self.overflow = true;
            self.entries[self.head as usize % 3] = Some(code);
            self.head = self.head.wrapping_add(1);
        }
        if !self.prog_light {
            // AGC source: PROGLARM — set PROG light on first alarm.
            self.prog_light = true;
        }
        code.severity()
    }

    /// Clear all alarm registers and extinguish the PROG light.
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc SKIPSIM block, lines 159-163.
    pub(crate) fn reset(&mut self) {
        self.entries = [None; 3];
        self.head = 0;
        self.count = 0;
        self.overflow = false;
        self.prog_light = false;
    }

    /// Return the most recent alarm code, or `None` if history is empty.
    pub(crate) fn most_recent_inner(&self) -> Option<AlarmCode> {
        if self.count == 0 {
            return None;
        }
        // The last written entry is at (head - 1) % 3.
        let last_idx = self.head.wrapping_sub(1) as usize % 3;
        self.entries[last_idx]
    }

    /// Return a copy of the ring buffer contents in oldest-first order.
    ///
    /// Used by the DSKY display and the sim TUI.
    pub(crate) fn history_inner(&self) -> [Option<AlarmCode>; 3] {
        if self.count == 0 {
            return [None; 3];
        }
        // oldest is at (head - count) % 3
        let oldest_idx = self.head.wrapping_sub(self.count) as usize % 3;
        let mut out = [None; 3];
        for (i, slot) in out.iter_mut().enumerate().take(self.count as usize) {
            *slot = self.entries[(oldest_idx + i) % 3];
        }
        out
    }
}

impl Default for AlarmState {
    fn default() -> Self {
        Self::new()
    }
}

// ── Global singleton ──────────────────────────────────────────────────────────

/// Module-level singleton, matching the AGC's single FAILREG bank.
///
/// Access always through `sync::cs` closure.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc FAILREG (3 words).
pub(crate) static ALARM_STATE: Mutex<RefCell<AlarmState>> =
    Mutex::new(RefCell::new(AlarmState::new()));

// ── ISR-safe public API ───────────────────────────────────────────────────────

impl AlarmState {
    /// Record an alarm and illuminate the PROG light.
    ///
    /// - ISR-safe: enters a `sync::cs` critical section.
    /// - Never allocates.
    /// - Stores code in the ring buffer (overwrites oldest on overflow).
    /// - Sets `prog_light = true` on the first alarm since reset.
    ///
    /// Returns the severity of the alarm so the caller can decide whether to
    /// trigger a restart.
    ///
    /// AGC source: Comanche055/ALARM_AND_ABORT.agc ALARM routine (page 1493).
    pub fn raise(code: AlarmCode) -> AlarmSeverity {
        crate::sync::cs(|cs| ALARM_STATE.borrow(cs).borrow_mut().record(code))
    }

    /// Clear all alarm registers and extinguish the PROG light.
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc SKIPSIM block, lines 159-163.
    pub fn clear_all() {
        crate::sync::cs(|cs| {
            ALARM_STATE.borrow(cs).borrow_mut().reset();
        });
    }

    /// Return the most recent alarm code recorded, or `None` if history is empty.
    pub fn most_recent() -> Option<AlarmCode> {
        crate::sync::cs(|cs| ALARM_STATE.borrow(cs).borrow().most_recent_inner())
    }

    /// Return a copy of the current ring buffer contents (oldest first).
    ///
    /// Used by the DSKY display and the sim TUI.
    pub fn history() -> [Option<AlarmCode>; 3] {
        crate::sync::cs(|cs| ALARM_STATE.borrow(cs).borrow().history_inner())
    }

    /// Return whether the PROG alarm light should be on.
    ///
    /// AGC source: Comanche055/ALARM_AND_ABORT.agc PROGLARM bit in DSPTAB+11.
    pub fn prog_light_on() -> bool {
        crate::sync::cs(|cs| ALARM_STATE.borrow(cs).borrow().prog_light)
    }
}

// ── Module-level convenience wrappers ─────────────────────────────────────────

/// Raise a program alarm.  Convenience wrapper around `AlarmState::raise`.
///
/// AGC source: Comanche055/ALARM_AND_ABORT.agc ALARM entry point (page 1493).
pub fn raise(code: AlarmCode) -> AlarmSeverity {
    AlarmState::raise(code)
}

/// Clear all alarms.  Called during fresh_start and restart initialisation.
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc SKIPSIM block, lines 159-163.
pub fn clear_all() {
    AlarmState::clear_all();
}

/// Return the most recent alarm code.
pub fn most_recent() -> Option<AlarmCode> {
    AlarmState::most_recent()
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: start each test with a clean slate.
    fn setup() {
        AlarmState::clear_all();
    }

    #[test]
    fn basic_raise_and_prog_light() {
        // Test 1: raise NoCoreSets → most_recent, prog_light, severity
        setup();
        let sev = AlarmState::raise(AlarmCode::NoCoreSets);
        assert_eq!(sev, AlarmSeverity::Bailout);
        assert_eq!(AlarmState::most_recent(), Some(AlarmCode::NoCoreSets));
        assert!(AlarmState::prog_light_on());
        let h = AlarmState::history();
        assert_eq!(h[0], Some(AlarmCode::NoCoreSets));
        assert_eq!(h[1], None);
        assert_eq!(h[2], None);
    }

    #[test]
    fn ring_buffer_overflow() {
        // Test 2: 4 alarms → overflow, oldest evicted
        setup();
        AlarmState::raise(AlarmCode::WaitlistOverflow); // slot 0
        AlarmState::raise(AlarmCode::WaitlistNegDt); // slot 1
        AlarmState::raise(AlarmCode::DeviceConflict); // slot 2
        AlarmState::raise(AlarmCode::PhaseTableError); // triggers overflow

        let h = AlarmState::history();
        // Oldest (WaitlistOverflow) evicted; remaining 3 in order
        assert_eq!(h[0], Some(AlarmCode::WaitlistNegDt));
        assert_eq!(h[1], Some(AlarmCode::DeviceConflict));
        assert_eq!(h[2], Some(AlarmCode::PhaseTableError));
        assert_eq!(AlarmState::most_recent(), Some(AlarmCode::PhaseTableError));
        assert!(AlarmState::prog_light_on());

        // Verify overflow flag via a subsequent raise not clearing the light
        AlarmState::raise(AlarmCode::NoCoreSets);
        assert!(AlarmState::prog_light_on());
    }

    #[test]
    fn clear_all_resets() {
        // Test 3: clear_all resets every field
        setup();
        AlarmState::raise(AlarmCode::WaitlistOverflow);
        AlarmState::raise(AlarmCode::NoCoreSets);
        AlarmState::clear_all();

        let h = AlarmState::history();
        assert_eq!(h, [None, None, None]);
        assert_eq!(AlarmState::most_recent(), None);
        assert!(!AlarmState::prog_light_on());

        // Verify overflow flag reset by checking a subsequent raise puts us at slot 0
        AlarmState::raise(AlarmCode::NoCoreSets);
        let h = AlarmState::history();
        assert_eq!(h[0], Some(AlarmCode::NoCoreSets));
        assert_eq!(h[1], None);
    }

    #[test]
    fn severity_classification() {
        assert_eq!(AlarmCode::NoVacArea.severity(), AlarmSeverity::Bailout);
        assert_eq!(
            AlarmCode::WaitlistNegDt.severity(),
            AlarmSeverity::SoftRestart
        );
        assert_eq!(
            AlarmCode::PhaseTableError.severity(),
            AlarmSeverity::Continue
        );
        assert_eq!(AlarmCode::Curtains.severity(), AlarmSeverity::Continue);
        assert_eq!(
            AlarmCode::ErasableChecksum.severity(),
            AlarmSeverity::SoftRestart
        );
    }

    #[test]
    fn alarm_codes_have_correct_values() {
        // Spot-check a few octal values match the AGC source constants
        assert_eq!(AlarmCode::WaitlistOverflow.value(), 0o1203);
        assert_eq!(AlarmCode::NoCoreSets.value(), 0o1202);
        assert_eq!(AlarmCode::DeviceConflict.value(), 0o1210);
        assert_eq!(AlarmCode::PhaseTableError.value(), 0o1107);
    }
}
