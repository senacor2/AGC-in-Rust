//! Restart protection: phase tables and group management.
//!
//! AGC source: Comanche055/FRESH_START_AND_RESTART.agc
//! Pages:      181-221 (MIT hardcopy pagination)
//! Routines:   GOPROG, GOPROG3, NXTRST/PACTIVE/PINACT, MR.KLEAN, STARTSB2,
//!             DOFSTART, NEWPHASE, PHASCHNG, RESTARTS
//!
//! AGC source: Comanche055/RESTART_TABLES.agc
//! Pages:      211-221 (MIT hardcopy pagination)
//! Routines:   PRDTTAB, CADRTAB, SIZETAB, 1.2SPOT-6.2SPOT (even tables),
//!             1.3SPOT-6.3SPOT (odd tables), per-group phase spots

use crate::executive::{job::JobFn, waitlist::TaskFn};

/// Index of a restart group. Valid values: 1..=NUM_RESTART_GROUPS.
/// 0 is not a valid group index.
///
/// AGC source: groups 1-6 in FRESH_START_AND_RESTART.agc and RESTART_TABLES.agc.
pub type GroupId = u8;

/// Phase number within a group. Valid values: 0..=127.
/// 0 means "group inactive" (corresponds to +0 in the AGC phase word).
///
/// AGC source: phase spots 1.2SPOT, 1.3SPOT, ... encoded as octal phase number.
pub type Phase = u8;

/// Number of restart groups.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc -PHASE1..-PHASE6 (12 words = 6 pairs).
/// Note: `NUMGRPS` in the AGC source = FIVE (5) because GOPROG3's loop walks groups 1-5
/// for standard verification.  Group 6 exists and is used (TVC, P27) but is NOT walked
/// by GOPROG3.  The Rust implementation allocates 6 groups.
pub const NUM_RESTART_GROUPS: usize = 6;

/// Number of groups verified by the standard GOPROG3 loop.
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc `NUMGRPS EQUALS FIVE`.
pub const NUMGRPS: usize = 5;

/// One group's phase state.
///
/// The AGC stores this as two consecutive erasable words:
/// - word 0: -phase (ones-complement negative)
/// - word 1: +phase (the phase value)
///
/// Both words must agree (`XOR == -0`) for the table to be valid.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc lines 1759-1770.
#[derive(Clone, Copy, Default)]
pub struct GroupState {
    /// Current phase. 0 means this group is inactive.
    ///
    /// AGC source: PHASE_G register (positive value).
    pub phase: Phase,

    /// Time-base for this group: the negative of TIME1 at the moment this phase was set.
    /// Used by RESTARTS to compute elapsed time since the phase was set.
    ///
    /// AGC source: TBASE1..TBASE6 (ERASABLE_ASSIGNMENTS.agc lines 1837-1847).
    pub tbase: i16,

    /// Per-group restart priority, set by the caller before PHASCHNG.
    ///
    /// AGC source: PHSPRDT1..6 (ERASABLE_ASSIGNMENTS.agc lines 1838-1848).
    pub restart_priority: u16,
}

/// Describes how a group should be re-dispatched on restart.
///
/// Derived from reading the G.P_SPOT entry in RESTART_TABLES.agc.
///
/// AGC source: Comanche055/RESTART_TABLES.agc, PRDTTAB/CADRTAB fields.
pub enum RestartAction {
    /// Re-schedule as a FINDVAC job (positive priority in PRDTTAB).
    ///
    /// AGC source: RESTART_TABLES.agc — positive PRDTTAB, positive 2CADR.
    FindvacJob { priority: u16, entry: JobFn },

    /// Re-schedule as a NOVAC job (negative priority in PRDTTAB).
    ///
    /// AGC source: RESTART_TABLES.agc — negative PRDTTAB, positive 2CADR.
    NovacJob { priority: u16, entry: JobFn },

    /// Re-schedule as a Waitlist task after `delta_cs` centiseconds.
    ///
    /// AGC source: RESTART_TABLES.agc — negative 2CADR, positive PRDTTAB.
    WaitlistTask { delta_cs: u16, task: TaskFn },

    /// Re-schedule as an immediate Waitlist task (PRDTTAB = -0 / OCT 77777).
    ///
    /// AGC source: RESTART_TABLES.agc — negative 2CADR, PRDTTAB = 77777.
    ImmediateTask { task: TaskFn },
}

/// The complete restart protection state for all 6 groups.
///
/// This struct must reside in a memory region that survives hardware restart.
/// On bare metal, mark with `#[link_section = ".noinit"]` and verify integrity
/// via the double-write invariant on every access.
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc; RESTART_TABLES.agc.
pub struct RestartProtection {
    /// Per-group state. Index 0 = group 1, ..., index 5 = group 6.
    pub(crate) groups: [GroupState; NUM_RESTART_GROUPS],

    /// Restart counter: incremented on every GOPROG entry.
    ///
    /// AGC source: REDOCTR (ERASABLE_ASSIGNMENTS.agc line 1915).
    pub(crate) restart_count: u16,

    /// Shadow array holding the ones-complement negatives of each group's phase.
    ///
    /// Written atomically alongside `groups[g].phase` inside a critical section.
    ///
    /// AGC source: -PHASE1..-PHASE6 (ERASABLE_ASSIGNMENTS.agc lines 1759-1770, odd entries).
    pub(crate) neg_phase: [i16; NUM_RESTART_GROUPS],
}

impl RestartProtection {
    /// Create a zero-initialised instance (all groups inactive).
    ///
    /// Used for fresh start only.  On hardware restart the existing contents
    /// must be preserved, not re-initialised.
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc MR.KLEAN (page 185).
    pub const fn new() -> Self {
        Self {
            groups: [GroupState {
                phase: 0,
                tbase: 0,
                restart_priority: 0,
            }; NUM_RESTART_GROUPS],
            restart_count: 0,
            neg_phase: [0; NUM_RESTART_GROUPS],
        }
    }

    /// Set the phase for a group.
    ///
    /// Both the direct and shadow (negative) words are written inside a single
    /// critical section to maintain the double-write invariant.
    ///
    /// `group` must be 1..=NUM_RESTART_GROUPS.
    /// `phase` = 0 clears the group (marks it inactive).
    ///
    /// If `set_tbase` is true, the time-base is updated to `-time1`.
    ///
    /// AGC source: Comanche055/RESTART_TABLES.agc NEWPHASE / PHASCHNG (page 220-221).
    pub fn set_phase(&mut self, group: GroupId, phase: Phase, set_tbase: bool, time1: i16) {
        debug_assert!(
            group >= 1 && group as usize <= NUM_RESTART_GROUPS,
            "group must be 1..=NUM_RESTART_GROUPS"
        );
        let idx = (group as usize)
            .saturating_sub(1)
            .min(NUM_RESTART_GROUPS - 1);

        // AGC source: NEWPHASE writes both words inside INHINT/RELINT.
        // In Rust we rely on the caller being inside sync::cs.
        self.groups[idx].phase = phase;
        // Ones-complement negation of phase as i16.
        // AGC: `-phase` means the bitwise NOT (ones-complement) for the shadow word.
        self.neg_phase[idx] = !(phase as i16);

        if set_tbase {
            self.groups[idx].tbase = -time1;
        }
    }

    /// Read the current phase for a group.
    ///
    /// Returns 0 if the group is inactive.
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc GOPROG3 `CCS PHASE1` branch.
    pub fn current_phase(&self, group: GroupId) -> Phase {
        if group < 1 || group as usize > NUM_RESTART_GROUPS {
            return 0;
        }
        self.groups[(group as usize) - 1].phase
    }

    /// Check phase table integrity for all 6 groups.
    ///
    /// For each group, verifies `(neg_phase[i] as u8) ^ phase[i] == 0xFF`
    /// (XOR of ones-complement pair = -0 = all-ones in ones-complement).
    ///
    /// Returns `Ok(())` if all groups pass.
    /// Returns `Err(group)` identifying the first corrupt group (1-based).
    ///
    /// Note: this function uses `Result` as an explicit spec exception because
    /// returning which group failed is needed for alarm dispatch.
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc GOPROG3 PCLOOP (page 188).
    pub fn verify_integrity(&self) -> Result<(), GroupId> {
        for i in 0..NUM_RESTART_GROUPS {
            let phase = self.groups[i].phase as i16;
            let neg = self.neg_phase[i];
            // In ones-complement: phase XOR (-phase) == -0 == all ones.
            // In two's complement: !(phase) == neg (our encoding).
            // Verify: neg == !phase (i.e., neg XOR phase == -1 in i16, or 0xFFFF as u16).
            let xor = (neg ^ phase) as u16;
            if xor != 0xFFFF {
                // Zero phase means inactive — both phase and neg_phase are 0 (fresh start).
                // Allow (0, 0) as valid: !0_i16 = -1, but for inactive groups we store (0, 0).
                if phase != 0 || neg != 0 {
                    return Err((i as u8) + 1);
                }
            }
        }
        Ok(())
    }

    /// Clear all groups (set all phases to 0).
    ///
    /// Called during fresh start and by V37 major-mode change.
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc MR.KLEAN (page 185).
    pub fn clear_all(&mut self) {
        for i in 0..NUM_RESTART_GROUPS {
            self.groups[i].phase = 0;
            self.neg_phase[i] = 0;
        }
    }

    /// Clear a specific group (set phase to 0).
    ///
    /// AGC source: `DCA NEG0; DXCH -PHASE_G` pattern throughout programs.
    pub fn clear_group(&mut self, group: GroupId) {
        if group < 1 || group as usize > NUM_RESTART_GROUPS {
            return;
        }
        let idx = (group as usize) - 1;
        self.groups[idx].phase = 0;
        self.neg_phase[idx] = 0;
    }

    /// True if all 6 groups have zero phase (all inactive).
    pub fn all_groups_zero(&self) -> bool {
        self.groups.iter().all(|g| g.phase == 0)
    }

    /// On-restart walker: scan groups 1-5 (NUMGRPS) for active phases and
    /// return the list of restart actions to perform.
    ///
    /// Groups are scanned in descending order (5 down to 1), matching the
    /// AGC GOPROG3 loop direction.  Group 6 is intentionally excluded from the
    /// standard walk (matching `NUMGRPS = FIVE`).
    ///
    /// Returns an array of up to NUM_RESTART_GROUPS actions; entries beyond
    /// `count` are `None`.
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc GOPROG3 NXTRST/PACTIVE loop
    ///   (page 189) and RESTARTS subroutine (called via RACTCADR).
    pub fn on_restart(
        &self,
        tables: &RestartTables,
    ) -> ([Option<RestartAction>; NUM_RESTART_GROUPS], usize) {
        let mut actions: [Option<RestartAction>; NUM_RESTART_GROUPS] =
            [None, None, None, None, None, None];
        let mut count = 0;

        // Scan groups 5 down to 1 (NUMGRPS = 5).
        for g in (1..=NUMGRPS as u8).rev() {
            let phase = self.current_phase(g);
            if phase > 0 {
                if let Some(action) = tables.lookup(g, phase) {
                    actions[count] = Some(action);
                    count += 1;
                }
            }
        }

        (actions, count)
    }

    /// Return the current restart counter value.
    ///
    /// AGC source: REDOCTR (ERASABLE_ASSIGNMENTS.agc line 1915).
    pub fn restart_count(&self) -> u16 {
        self.restart_count
    }

    /// Increment the restart counter (called at the start of GOPROG).
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc GOPROG `INCR REDOCTR` (page 186).
    pub fn increment_restart_count(&mut self) {
        self.restart_count = self.restart_count.wrapping_add(1);
    }
}

impl Default for RestartProtection {
    fn default() -> Self {
        Self::new()
    }
}

/// The compile-time restart tables (ROM equivalent of RESTART_TABLES.agc).
///
/// In the AGC, the tables live in fixed ROM.  In Rust they are `const` arrays.
///
/// AGC source: Comanche055/RESTART_TABLES.agc SIZETAB / PRDTTAB / CADRTAB structure.
pub struct RestartTables {
    pub(crate) _marker: core::marker::PhantomData<()>,
}

impl RestartTables {
    /// Construct an empty (no-op) restart table.
    ///
    /// Production code populates entries from RESTART_TABLES.agc; this empty
    /// version is used in tests and Milestone 1 where no programs are running.
    pub const fn empty() -> Self {
        Self {
            _marker: core::marker::PhantomData,
        }
    }

    /// Look up the restart action for (group, phase).
    ///
    /// Returns `None` if no table entry exists for this (group, phase) pair.
    ///
    /// AGC source: Comanche055/RESTART_TABLES.agc SIZETAB / G.P_SPOT entries.
    pub fn lookup(&self, _group: GroupId, _phase: Phase) -> Option<RestartAction> {
        // Milestone 1: no programs running; return None for all lookups.
        // Future milestones populate this from RESTART_TABLES.agc.
        None
    }
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn set_and_read_phase() {
        // Test 1: set group 3 phase 5; verify current_phase and integrity
        let mut rp = RestartProtection::new();
        rp.set_phase(3, 5, false, 0);

        assert_eq!(rp.current_phase(3), 5);
        assert_eq!(rp.current_phase(1), 0);
        assert_eq!(rp.current_phase(2), 0);
        assert_eq!(rp.current_phase(4), 0);
        assert_eq!(rp.current_phase(5), 0);
        assert_eq!(rp.current_phase(6), 0);

        assert!(rp.verify_integrity().is_ok());

        // neg_phase[2] == !5_i16
        assert_eq!(rp.neg_phase[2], !5_i16);
    }

    #[test]
    fn clear_and_verify() {
        // Test 2: set groups 1 and 4, clear group 4, verify
        let mut rp = RestartProtection::new();
        rp.set_phase(1, 3, false, 0);
        rp.set_phase(4, 7, false, 0);
        rp.clear_group(4);

        assert_eq!(rp.current_phase(4), 0);
        assert_eq!(rp.current_phase(1), 3);
        assert!(rp.verify_integrity().is_ok());

        rp.clear_all();
        for g in 1..=6 {
            assert_eq!(rp.current_phase(g), 0);
        }
        assert!(rp.verify_integrity().is_ok());
    }

    #[test]
    fn simulated_restart() {
        // Test 3: simulate mid-computation restart at phase 2 of group 5
        let mut rp = RestartProtection::new();
        rp.set_phase(5, 1, false, 0);
        rp.set_phase(5, 2, false, 0);

        // Verify integrity passes (both writes completed)
        assert!(rp.verify_integrity().is_ok());

        // on_restart with empty tables returns no actions (no programs defined yet)
        let tables = RestartTables::empty();
        let (actions, count) = rp.on_restart(&tables);
        // Empty tables → 0 actions
        assert_eq!(count, 0);
        assert!(actions[0].is_none());

        // Phase is still 2 (on_restart is read-only)
        assert_eq!(rp.current_phase(5), 2);
    }

    #[test]
    fn integrity_fails_on_mismatch() {
        // Corrupt a group manually — should fail integrity check
        let mut rp = RestartProtection::new();
        rp.set_phase(3, 7, false, 0);
        // Manually corrupt the neg_phase
        rp.neg_phase[2] = -6; // should be !7 = -8
        assert!(rp.verify_integrity().is_err());
    }

    #[test]
    fn increment_restart_count() {
        let mut rp = RestartProtection::new();
        assert_eq!(rp.restart_count(), 0);
        rp.increment_restart_count();
        assert_eq!(rp.restart_count(), 1);
        rp.increment_restart_count();
        assert_eq!(rp.restart_count(), 2);
    }

    #[test]
    fn group_6_not_in_standard_walk() {
        // Group 6 is set but should not appear in on_restart output (NUMGRPS=5)
        let mut rp = RestartProtection::new();
        rp.set_phase(6, 3, false, 0);
        let tables = RestartTables::empty();
        let (_actions, count) = rp.on_restart(&tables);
        assert_eq!(
            count, 0,
            "group 6 should not be walked by standard on_restart"
        );
    }
}
