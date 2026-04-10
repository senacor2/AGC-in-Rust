//! Entry guidance programs P61–P67.
//!
//! **Milestone 4 Phase 5 — skeletons only.** These programs wire up the
//! entry phase state machine, major-mode/DSKY sequencing, and the
//! inter-program handoff contract. The real entry-guidance math (roll
//! steering, lift-to-drag modulation, skip targeting, range prediction)
//! is a later milestone.
//!
//! AGC source: Comanche055/P61-P67.agc, Comanche055/REENTRY_CONTROL.agc.

use crate::control::{dap::dap_stop, DapMode};
use crate::executive::job::JobPriority;

// ── Constants ─────────────────────────────────────────────────────────────────

pub const P61_MAJOR_MODE: u8 = 61;
pub const P62_MAJOR_MODE: u8 = 62;
pub const P63_MAJOR_MODE: u8 = 63;
pub const P64_MAJOR_MODE: u8 = 64;
pub const P67_MAJOR_MODE: u8 = 67;

/// Job priority for entry programs — one tier above the background monitors.
pub const PRIORITY: JobPriority = 10;

/// Sensed-acceleration threshold (g units) that marks entry interface.
/// Below this, P63 monitors; at/above, P64 closed-loop guidance may run.
pub const ENTRY_THRESHOLD_G: f64 = 0.05;

const VERB_DISPLAY: u8 = 6;
const VERB_MONITOR: u8 = 16;

// ── Program alarms ────────────────────────────────────────────────────────────

const ALARM_P62_WRONG_PHASE: u16 = 231;
const ALARM_P63_WRONG_PHASE: u16 = 232;
const ALARM_P64_EARLY: u16 = 233;
const ALARM_P67_WRONG_PHASE: u16 = 234;

// ── EntryPhase ────────────────────────────────────────────────────────────────

/// Entry-guidance phase.
///
/// Advances strictly left-to-right in nominal operation:
/// `Idle → Preparation → Separation → PreEntry → Entry → Final`.
/// Out-of-sequence transitions raise soft alarms but are not blocked.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum EntryPhase {
    /// No entry program active.
    #[default]
    Idle,
    /// P61 — entry preparation (pre-separation).
    Preparation,
    /// P62 — CM/SM separation.
    Separation,
    /// P63 — pre-0.05g monitoring.
    PreEntry,
    /// P64 — closed-loop entry guidance.
    Entry,
    /// P67 — final phase / drogue deployed.
    Final,
}

// ── EntryState ────────────────────────────────────────────────────────────────

/// Entry-guidance state block stored on `AgcState`.
#[derive(Clone, Copy, Debug, Default)]
pub struct EntryState {
    /// Current entry-guidance phase.
    pub phase: EntryPhase,
    /// Sensed spacecraft acceleration (g units).
    ///
    /// Populated by the test harness in Phase 5; wired into the
    /// SERVICER pipeline in a later milestone.
    pub sensed_acceleration_g: f64,
    /// Roll command the entry guidance law is holding (radians). Stub.
    pub roll_command_rad: f64,
    /// Range to splashdown target (km). Stub.
    pub target_range_km: f64,
    /// `true` once `p67_deploy_drogue` has run.
    pub drogue_deployed: bool,
}

impl EntryState {
    /// `const` constructor usable inside `AgcState::new`.
    pub const fn new() -> Self {
        Self {
            phase: EntryPhase::Idle,
            sensed_acceleration_g: 0.0,
            roll_command_rad: 0.0,
            target_range_km: 0.0,
            drogue_deployed: false,
        }
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────────

fn raise(state: &mut crate::AgcState, code: u16) {
    state.alarm.code = code;
    state.alarm.lit = true;
}

fn set_display(state: &mut crate::AgcState, prog: u8, verb: u8, noun: u8) {
    state.major_mode = prog;
    state.dsky.prog = prog;
    state.dsky.verb = verb;
    state.dsky.noun = noun;
    state.dsky.flashing = false;
}

/// Write the continuous-monitor entry status triplet
/// (sensed g / roll command / target range) to the DSKY.
fn write_entry_status(state: &mut crate::AgcState) {
    state.dsky.r[0] = state.entry.sensed_acceleration_g as f32;
    state.dsky.r[1] = state.entry.roll_command_rad as f32;
    state.dsky.r[2] = state.entry.target_range_km as f32;
}

// ── P61 ───────────────────────────────────────────────────────────────────────

/// Entry point registered in `PROGRAM_TABLE[61]`.
pub fn init_p61(state: &mut crate::AgcState) -> JobPriority {
    state.entry.phase = EntryPhase::Preparation;
    set_display(state, P61_MAJOR_MODE, VERB_DISPLAY, 61);
    state.dsky.r[0] = state.entry.target_range_km as f32;
    state.dsky.r[1] = 0.0;
    state.dsky.r[2] = 0.0;
    PRIORITY
}

// ── P62 ───────────────────────────────────────────────────────────────────────

/// Entry point registered in `PROGRAM_TABLE[62]` — CM/SM separation.
pub fn init_p62(state: &mut crate::AgcState) -> JobPriority {
    if state.entry.phase != EntryPhase::Preparation {
        raise(state, ALARM_P62_WRONG_PHASE);
    }
    state.entry.phase = EntryPhase::Separation;

    // Any stale targeting ΔV is void post-separation (SM is jettisoned).
    state.pending_maneuver = None;

    // CM-only RCS control from here on. If a burn was active we also
    // have to quench it — dap_stop clears staging fields.
    dap_stop(state);
    state.dap_state.mode = DapMode::AttitudeHold;

    set_display(state, P62_MAJOR_MODE, VERB_DISPLAY, 62);
    state.dsky.r[0] = 0.0;
    state.dsky.r[1] = 0.0;
    state.dsky.r[2] = 0.0;
    PRIORITY
}

// ── P63 ───────────────────────────────────────────────────────────────────────

/// Entry point registered in `PROGRAM_TABLE[63]` — pre-0.05g monitor.
pub fn init_p63(state: &mut crate::AgcState) -> JobPriority {
    if state.entry.phase != EntryPhase::Separation {
        raise(state, ALARM_P63_WRONG_PHASE);
    }
    state.entry.phase = EntryPhase::PreEntry;

    set_display(state, P63_MAJOR_MODE, VERB_MONITOR, 64);
    write_entry_status(state);
    PRIORITY
}

/// Check whether the 0.05g entry-interface threshold has been crossed.
///
/// Call this from the sensed-acceleration update path (test harness for
/// Phase 5; SERVICER exit hook in a later milestone). When the phase is
/// `PreEntry` and `entry.sensed_acceleration_g >= ENTRY_THRESHOLD_G`,
/// advances the phase to `Entry` and returns `true`.
pub fn p63_check_threshold(state: &mut crate::AgcState) -> bool {
    if state.entry.phase == EntryPhase::PreEntry
        && state.entry.sensed_acceleration_g >= ENTRY_THRESHOLD_G
    {
        state.entry.phase = EntryPhase::Entry;
        return true;
    }
    false
}

// ── P64 ───────────────────────────────────────────────────────────────────────

/// Entry point registered in `PROGRAM_TABLE[64]` — closed-loop entry guidance.
pub fn init_p64(state: &mut crate::AgcState) -> JobPriority {
    if state.entry.sensed_acceleration_g < ENTRY_THRESHOLD_G {
        raise(state, ALARM_P64_EARLY);
    }
    state.entry.phase = EntryPhase::Entry;
    state.entry.roll_command_rad = 0.0; // stub — real guidance law lives in later MS

    set_display(state, P64_MAJOR_MODE, VERB_MONITOR, 64);
    write_entry_status(state);
    PRIORITY
}

// ── P67 ───────────────────────────────────────────────────────────────────────

/// Entry point registered in `PROGRAM_TABLE[67]` — final phase / drogue deploy.
pub fn init_p67(state: &mut crate::AgcState) -> JobPriority {
    if state.entry.phase != EntryPhase::Entry {
        raise(state, ALARM_P67_WRONG_PHASE);
    }
    state.entry.phase = EntryPhase::Final;

    p67_deploy_drogue(state);

    set_display(state, P67_MAJOR_MODE, VERB_DISPLAY, 67);
    state.dsky.r[0] = state.entry.target_range_km as f32;
    state.dsky.r[1] = 0.0;
    state.dsky.r[2] = 0.0;
    PRIORITY
}

/// Latch the drogue-deployed flag.
///
/// The real AGC commands the SECS drogue-deployment pyro via a hardware
/// discrete; the HAL interface for that does not yet exist, so this stub
/// only sets the bookkeeping flag.
pub fn p67_deploy_drogue(state: &mut crate::AgcState) {
    state.entry.drogue_deployed = true;
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AgcState;

    // ── P61 ───────────────────────────────────────────────────────────────────

    /// TC-P61-1: `init_p61` sets phase = Preparation and major_mode = 61.
    #[test]
    fn tc_p61_1_sets_preparation_phase() {
        let mut state = AgcState::new();
        let prio = init_p61(&mut state);

        assert_eq!(prio, PRIORITY);
        assert_eq!(state.entry.phase, EntryPhase::Preparation);
        assert_eq!(state.major_mode, P61_MAJOR_MODE);
        assert_eq!(state.dsky.prog, P61_MAJOR_MODE);
        assert_eq!(state.alarm.code, 0);
    }

    // ── P62 ───────────────────────────────────────────────────────────────────

    /// TC-P62-1: `init_p62` from Preparation advances to Separation and
    /// clears pending_maneuver.
    #[test]
    fn tc_p62_1_from_preparation() {
        use crate::guidance::targeting::{Maneuver, TargetingMode};
        use crate::types::{DeltaV, Met};

        let mut state = AgcState::new();
        state.entry.phase = EntryPhase::Preparation;
        state.pending_maneuver = Some(Maneuver {
            tig: Met(0),
            delta_v: DeltaV([10.0, 0.0, 0.0]),
            burn_attitude: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            mode: TargetingMode::ExternalDeltaV,
        });

        init_p62(&mut state);

        assert_eq!(state.entry.phase, EntryPhase::Separation);
        assert_eq!(state.major_mode, P62_MAJOR_MODE);
        assert_eq!(state.alarm.code, 0);
        assert!(state.pending_maneuver.is_none(), "stale ΔV must be cleared");
        assert_eq!(state.dap_state.mode, DapMode::AttitudeHold);
    }

    /// TC-P62-2: `init_p62` from Idle raises alarm 231 but still advances.
    #[test]
    fn tc_p62_2_wrong_phase_alarm() {
        let mut state = AgcState::new();
        // phase is Idle (default)
        init_p62(&mut state);

        assert_eq!(state.alarm.code, ALARM_P62_WRONG_PHASE);
        assert!(state.alarm.lit);
        assert_eq!(
            state.entry.phase,
            EntryPhase::Separation,
            "soft alarm — phase still advances"
        );
    }

    // ── P63 ───────────────────────────────────────────────────────────────────

    /// TC-P63-1: `init_p63` from Separation advances to PreEntry.
    #[test]
    fn tc_p63_1_from_separation() {
        let mut state = AgcState::new();
        state.entry.phase = EntryPhase::Separation;

        init_p63(&mut state);

        assert_eq!(state.entry.phase, EntryPhase::PreEntry);
        assert_eq!(state.major_mode, P63_MAJOR_MODE);
        assert_eq!(state.dsky.verb, VERB_MONITOR);
        assert_eq!(state.alarm.code, 0);
    }

    /// TC-P63-2: `p63_check_threshold` with g = 0.04 stays in PreEntry.
    #[test]
    fn tc_p63_2_below_threshold() {
        let mut state = AgcState::new();
        state.entry.phase = EntryPhase::PreEntry;
        state.entry.sensed_acceleration_g = 0.04;

        let crossed = p63_check_threshold(&mut state);

        assert!(!crossed);
        assert_eq!(state.entry.phase, EntryPhase::PreEntry);
    }

    /// TC-P63-3: `p63_check_threshold` with g = 0.08 advances to Entry.
    #[test]
    fn tc_p63_3_crosses_threshold() {
        let mut state = AgcState::new();
        state.entry.phase = EntryPhase::PreEntry;
        state.entry.sensed_acceleration_g = 0.08;

        let crossed = p63_check_threshold(&mut state);

        assert!(crossed);
        assert_eq!(state.entry.phase, EntryPhase::Entry);
    }

    // ── P64 ───────────────────────────────────────────────────────────────────

    /// TC-P64-1: `init_p64` with g = 0.10 sets phase = Entry and no alarm.
    #[test]
    fn tc_p64_1_nominal_entry() {
        let mut state = AgcState::new();
        state.entry.phase = EntryPhase::PreEntry;
        state.entry.sensed_acceleration_g = 0.10;

        init_p64(&mut state);

        assert_eq!(state.entry.phase, EntryPhase::Entry);
        assert_eq!(state.major_mode, P64_MAJOR_MODE);
        assert_eq!(state.alarm.code, 0);
    }

    /// TC-P64-2: `init_p64` with g = 0.02 raises alarm 233.
    #[test]
    fn tc_p64_2_early_invocation() {
        let mut state = AgcState::new();
        state.entry.phase = EntryPhase::PreEntry;
        state.entry.sensed_acceleration_g = 0.02;

        init_p64(&mut state);

        assert_eq!(state.alarm.code, ALARM_P64_EARLY);
        assert!(state.alarm.lit);
        // Phase still advances — soft alarm
        assert_eq!(state.entry.phase, EntryPhase::Entry);
    }

    // ── P67 ───────────────────────────────────────────────────────────────────

    /// TC-P67-1: `init_p67` from Entry sets phase = Final and drogue_deployed.
    #[test]
    fn tc_p67_1_from_entry_deploys_drogue() {
        let mut state = AgcState::new();
        state.entry.phase = EntryPhase::Entry;

        init_p67(&mut state);

        assert_eq!(state.entry.phase, EntryPhase::Final);
        assert_eq!(state.major_mode, P67_MAJOR_MODE);
        assert!(state.entry.drogue_deployed);
        assert_eq!(state.alarm.code, 0);
    }

    /// TC-P67-2: `init_p67` from Preparation raises alarm 234 but still advances.
    #[test]
    fn tc_p67_2_wrong_phase_alarm() {
        let mut state = AgcState::new();
        state.entry.phase = EntryPhase::Preparation;

        init_p67(&mut state);

        assert_eq!(state.alarm.code, ALARM_P67_WRONG_PHASE);
        assert_eq!(state.entry.phase, EntryPhase::Final);
        assert!(state.entry.drogue_deployed);
    }

    // ── Sequence test ─────────────────────────────────────────────────────────

    /// End-to-end: nominal P61 → P62 → P63 → threshold → P64 → P67 sequence.
    #[test]
    fn tc_entry_nominal_sequence() {
        let mut state = AgcState::new();

        init_p61(&mut state);
        assert_eq!(state.entry.phase, EntryPhase::Preparation);

        init_p62(&mut state);
        assert_eq!(state.entry.phase, EntryPhase::Separation);

        init_p63(&mut state);
        assert_eq!(state.entry.phase, EntryPhase::PreEntry);

        // Simulate sensed-g crossing during pre-entry monitoring.
        state.entry.sensed_acceleration_g = 0.06;
        assert!(p63_check_threshold(&mut state));
        assert_eq!(state.entry.phase, EntryPhase::Entry);

        // P64 can be called cleanly at this point.
        init_p64(&mut state);
        assert_eq!(state.alarm.code, 0);

        init_p67(&mut state);
        assert_eq!(state.entry.phase, EntryPhase::Final);
        assert!(state.entry.drogue_deployed);
    }
}
