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
