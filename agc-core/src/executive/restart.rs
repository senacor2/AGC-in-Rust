/// Number of restart groups (matches Comanche055).
pub const NUM_RESTART_GROUPS: usize = 6;

/// Restart group indices.
pub const GROUP_1: usize = 0;
pub const GROUP_2: usize = 1;
pub const GROUP_3: usize = 2;
pub const GROUP_4: usize = 3;
pub const GROUP_5: usize = 4;
pub const GROUP_6: usize = 5;

/// Phase value for a restart group.
///
/// - 0 (IDLE): group is inactive; nothing to restart.
/// - Positive odd: re-dispatch the group as a waitlist *task*.
/// - Positive even: re-dispatch the group as an executive *job*.
/// - Negative: restart the group from the top of the current phase.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Phase(pub i16);

impl Phase {
    /// Group is idle — no restart action needed.
    pub const IDLE: Phase = Phase(0);

    pub fn new(value: i16) -> Phase {
        Phase(value)
    }

    pub fn is_idle(self) -> bool {
        self.0 == 0
    }

    pub fn is_task(self) -> bool {
        self.0 > 0 && self.0 % 2 != 0
    }

    pub fn is_job(self) -> bool {
        self.0 > 0 && self.0 % 2 == 0
    }
}

/// Restart protection state — the phase registers for all restart groups.
///
/// Before beginning a restartable multi-step computation, call `set_phase`
/// to record which step is in progress. On completion, call `set_phase` with
/// `Phase::IDLE`. After a restart, the `Executive` reads this table to
/// re-dispatch any groups that were active.
pub struct RestartProtection {
    pub phases: [Phase; NUM_RESTART_GROUPS],
}

impl RestartProtection {
    pub const fn new() -> Self {
        Self {
            phases: [Phase::IDLE; NUM_RESTART_GROUPS],
        }
    }

    /// Record that `group` is now at `phase`.
    #[inline]
    pub fn set_phase(&mut self, group: usize, phase: Phase) {
        self.phases[group] = phase;
    }

    /// Read the phase for `group`.
    #[inline]
    pub fn phase(&self, group: usize) -> Phase {
        self.phases[group]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // TC-RP-1: positive even → is_job
    #[test]
    fn tc_rp_1_positive_even_is_job() {
        let p = Phase::new(2);
        assert!(p.is_job());
        assert!(!p.is_task());
        assert!(!p.is_idle());
    }

    // TC-RP-2: positive odd → is_task
    #[test]
    fn tc_rp_2_positive_odd_is_task() {
        let p = Phase::new(3);
        assert!(p.is_task());
        assert!(!p.is_job());
        assert!(!p.is_idle());
    }

    // TC-RP-3: negative → not idle, not job, not task
    #[test]
    fn tc_rp_3_negative() {
        let p = Phase::new(-1);
        assert!(!p.is_idle());
        assert!(!p.is_job());
        assert!(!p.is_task());
    }

    // TC-RP-4: IDLE
    #[test]
    fn tc_rp_4_idle() {
        assert!(Phase::IDLE.is_idle());
    }

    // TC-RP-5: set_phase and phase round-trip
    #[test]
    fn tc_rp_5_set_and_read() {
        let mut rp = RestartProtection::new();
        rp.set_phase(GROUP_2, Phase::new(4));
        rp.set_phase(GROUP_5, Phase::IDLE);
        assert_eq!(rp.phase(GROUP_2), Phase::new(4));
        assert_eq!(rp.phase(GROUP_5), Phase::IDLE);
        // All other groups remain IDLE after new()
        assert_eq!(rp.phase(GROUP_1), Phase::IDLE);
        assert_eq!(rp.phase(GROUP_3), Phase::IDLE);
    }

    // TC-RP-new: all groups start IDLE
    #[test]
    fn new_all_idle() {
        let rp = RestartProtection::new();
        for i in 0..NUM_RESTART_GROUPS {
            assert!(rp.phase(i).is_idle());
        }
    }
}
