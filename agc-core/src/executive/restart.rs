//! Restart protection: phase tables and group management.
//!
//! The AGC can restart at any time. The phase table mechanism ensures that
//! long multi-step computations resume from a known checkpoint rather than
//! starting over or producing corrupted results.
//!
//! Usage:
//! ```ignore
//! state.restart.set_phase(GROUP_3, Phase::new(1));
//! // ... step 1 ...
//! state.restart.set_phase(GROUP_3, Phase::new(3));
//! // ... step 2 ...
//! state.restart.set_phase(GROUP_3, Phase::IDLE);
//! ```
//!
//! AGC source: FRESH_START_AND_RESTART.agc, PHASE_TABLE_MAINTENANCE.agc.

/// Number of restart groups in Comanche055.
///
/// AGC source: FRESH_START_AND_RESTART.agc — `NUMGRPS EQUALS FIVE`.
/// Five groups (1–5), not six. Any group index outside this range is a
/// no-op in `set_phase` / `get_phase`.
pub const NUM_RESTART_GROUPS: usize = 5;

/// Index type for restart groups (1-based, as in the original AGC).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct RestartGroup(pub u8);

pub const GROUP_1: RestartGroup = RestartGroup(1);
pub const GROUP_2: RestartGroup = RestartGroup(2);
pub const GROUP_3: RestartGroup = RestartGroup(3);
pub const GROUP_4: RestartGroup = RestartGroup(4);
pub const GROUP_5: RestartGroup = RestartGroup(5);

/// Phase value within a restart group.
///
/// - `Phase::IDLE` (0): group is idle; no re-dispatch on restart.
/// - Odd positive: re-dispatch as a Waitlist task.
/// - Even positive: re-dispatch as an Executive job.
/// - Negative: restart the group from the top of its phase table.
///
/// AGC source: PHASE_TABLE_MAINTENANCE.agc — phase register encoding.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Phase(pub i16);

impl Phase {
    /// Group is idle — no action on restart.
    pub const IDLE: Self = Self(0);

    pub const fn new(v: i16) -> Self {
        Self(v)
    }

    /// True if this phase triggers a task re-dispatch on restart.
    pub fn is_task(self) -> bool {
        self.0 > 0 && (self.0 & 1) != 0
    }

    /// True if this phase triggers a job re-dispatch on restart.
    pub fn is_job(self) -> bool {
        self.0 > 0 && (self.0 & 1) == 0
    }
}

/// Restart protection state — the phase table.
///
/// One `Phase` per restart group. Written before each step of a multi-step
/// computation; read by the RESTART handler to re-dispatch in-progress work.
///
/// AGC source: ERASABLE_ASSIGNMENTS.agc — PHASEWD0/PHASEWD1 phase words.
pub struct RestartProtection {
    phases: [Phase; NUM_RESTART_GROUPS],
}

impl Default for RestartProtection {
    fn default() -> Self {
        Self::new()
    }
}

impl RestartProtection {
    pub const fn new() -> Self {
        Self {
            phases: [Phase::IDLE; NUM_RESTART_GROUPS],
        }
    }

    /// Record the current phase for `group`.
    ///
    /// `group` is 1-based (GROUP_1..GROUP_6). Out-of-range group indices are
    /// silently ignored.
    ///
    /// AGC source: PHASE_TABLE_MAINTENANCE.agc — PHASCHNG routine.
    pub fn set_phase(&mut self, group: RestartGroup, phase: Phase) {
        let idx = group.0 as usize;
        if (1..=NUM_RESTART_GROUPS).contains(&idx) {
            self.phases[idx - 1] = phase;
        }
    }

    /// Read the recorded phase for `group`.
    pub fn get_phase(&self, group: RestartGroup) -> Phase {
        let idx = group.0 as usize;
        if (1..=NUM_RESTART_GROUPS).contains(&idx) {
            self.phases[idx - 1]
        } else {
            Phase::IDLE
        }
    }

    /// Clear all groups to IDLE. Called at the end of FRESH START.
    ///
    /// AGC source: FRESH_START_AND_RESTART.agc — CLEANDSP routine clears phase table.
    pub fn clear_all(&mut self) {
        for p in &mut self.phases {
            *p = Phase::IDLE;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn set_and_get_phase() {
        let mut rp = RestartProtection::new();
        rp.set_phase(GROUP_3, Phase::new(1));
        assert_eq!(rp.get_phase(GROUP_3), Phase::new(1));
        assert_eq!(rp.get_phase(GROUP_1), Phase::IDLE);
    }

    #[test]
    fn clear_all_resets() {
        let mut rp = RestartProtection::new();
        rp.set_phase(GROUP_1, Phase::new(3));
        rp.set_phase(GROUP_5, Phase::new(2));
        rp.clear_all();
        assert_eq!(rp.get_phase(GROUP_1), Phase::IDLE);
        assert_eq!(rp.get_phase(GROUP_5), Phase::IDLE);
    }

    #[test]
    fn phase_dispatch_type() {
        assert!(Phase::new(1).is_task());
        assert!(Phase::new(3).is_task());
        assert!(Phase::new(2).is_job());
        assert!(Phase::new(4).is_job());
        assert!(!Phase::IDLE.is_task());
        assert!(!Phase::IDLE.is_job());
    }

    #[test]
    fn out_of_range_group_is_noop() {
        let mut rp = RestartProtection::new();
        rp.set_phase(RestartGroup(0), Phase::new(99)); // invalid — below range
        rp.set_phase(RestartGroup(6), Phase::new(99)); // invalid — above range (AGC has 5 groups)
        // All groups remain IDLE
        for g in 1..=NUM_RESTART_GROUPS {
            assert_eq!(rp.get_phase(RestartGroup(g as u8)), Phase::IDLE);
        }
    }
}
