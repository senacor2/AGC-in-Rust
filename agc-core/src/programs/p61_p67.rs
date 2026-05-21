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
use crate::services::average_g::SERVICER_PERIOD_S;

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

/// Standard gravity `g_0` (m/s²) used to convert the SERVICER's sensed
/// delta-V into g-loading for `entry.sensed_acceleration_g`. AGC source
/// `REENTRY_CONTROL.agc` uses 32.2 ft/s² (= 9.815 m/s²); the modern SI
/// value 9.806 65 differs by < 1 % and is preferred since downstream
/// thresholds are stated to two significant figures.
pub const G0_MPS2: f64 = 9.806_65;

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

    // Install the entry SERVICER exit hook so the 0.05g threshold check has
    // a live sensed-acceleration value to compare against. Cleared in
    // `init_p67` when the entry sequence completes.
    state.servicer_exit = Some(entry_servicer_exit);

    set_display(state, P63_MAJOR_MODE, VERB_MONITOR, 64);
    write_entry_status(state);
    PRIORITY
}

/// SERVICER exit hook that updates `state.entry.sensed_acceleration_g`.
///
/// Reads the most recent inertial sensed delta-V staged by `servicer_task`
/// (`state.servicer_last_dv_inertial`), divides by the SERVICER period to get
/// the average acceleration over the cycle, then divides by `G0_MPS2` to
/// express the result in g-units that P63 / P64 threshold logic expects.
///
/// Installed by `init_p63`; cleared by `init_p67`. Coexists with — but never
/// runs at the same time as — `guidance::maneuver::burn_servicer_exit`,
/// because the CM is post-separation and no SPS burn is active during entry.
pub fn entry_servicer_exit(state: &mut crate::AgcState) {
    let dv = state.servicer_last_dv_inertial;
    let dv_mag = libm::sqrt(dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]);
    let accel_mps2 = dv_mag / SERVICER_PERIOD_S;
    state.entry.sensed_acceleration_g = accel_mps2 / G0_MPS2;
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

    // Entry sequence is complete; uninstall the SERVICER exit hook so the
    // next program (typically P00) sees a clean slot.
    state.servicer_exit = None;

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

    // ── entry_servicer_exit ───────────────────────────────────────────────────

    /// TC-ESE-1: `entry_servicer_exit` computes g-loading from the staged
    /// inertial delta-V, dividing by the SERVICER period and `G0_MPS2`.
    #[test]
    fn tc_ese_1_g_loading_from_staged_dv() {
        let mut state = AgcState::new();
        // Stage a 19.613 m/s inertial delta-V — corresponds to ~1 g over 2 s.
        state.servicer_last_dv_inertial = [G0_MPS2 * SERVICER_PERIOD_S, 0.0, 0.0];

        entry_servicer_exit(&mut state);

        assert!(
            (state.entry.sensed_acceleration_g - 1.0).abs() < 1e-12,
            "expected 1.0 g, got {}",
            state.entry.sensed_acceleration_g
        );
    }

    /// TC-ESE-2: A zero delta-V produces zero g-loading (idle/coast case).
    #[test]
    fn tc_ese_2_zero_dv_zero_g() {
        let mut state = AgcState::new();
        state.servicer_last_dv_inertial = [0.0; 3];
        state.entry.sensed_acceleration_g = 999.0; // pre-poison

        entry_servicer_exit(&mut state);

        assert_eq!(state.entry.sensed_acceleration_g, 0.0);
    }

    /// TC-ESE-3: `init_p63` installs the SERVICER exit hook so the live
    /// SERVICER → `sensed_acceleration_g` path is in effect during PreEntry.
    #[test]
    fn tc_ese_3_p63_installs_hook() {
        let mut state = AgcState::new();
        state.entry.phase = EntryPhase::Separation;
        assert!(state.servicer_exit.is_none(), "fixture precondition");

        init_p63(&mut state);

        // Exercising the hook is the cleanest way to prove the right function
        // is installed without comparing raw function pointers.
        state.servicer_last_dv_inertial = [G0_MPS2 * SERVICER_PERIOD_S * 0.5, 0.0, 0.0];
        let exit = state.servicer_exit.expect("init_p63 must install the hook");
        exit(&mut state);
        assert!(
            (state.entry.sensed_acceleration_g - 0.5).abs() < 1e-12,
            "expected 0.5 g, got {}",
            state.entry.sensed_acceleration_g
        );
    }

    /// TC-ESE-4: `init_p67` clears the SERVICER exit hook after entry ends.
    #[test]
    fn tc_ese_4_p67_clears_hook() {
        let mut state = AgcState::new();
        state.entry.phase = EntryPhase::Entry;
        state.servicer_exit = Some(entry_servicer_exit);

        init_p67(&mut state);

        assert!(
            state.servicer_exit.is_none(),
            "init_p67 must clear servicer_exit"
        );
    }

    /// TC-ESE-5: end-to-end PIPA → sensed-g via one SERVICER cycle.
    /// Drives `servicer_task` with non-zero PIPA counts after `init_p63` has
    /// installed the hook, and verifies that `sensed_acceleration_g` advances
    /// off zero.
    #[test]
    fn tc_ese_5_pipa_drives_sensed_g() {
        use crate::services::average_g::{servicer_task, start_servicer};

        let mut state = AgcState::new();
        // REFSMMAT and pipa_cal default to identity / nominal in AgcState::new.
        // earth_gravity rejects a zero position vector, so place the CM at a
        // representative entry-interface altitude (122 km).
        state.csm_state.position = [6_500_000.0, 0.0, 0.0];
        state.csm_state.velocity = [0.0, 7_800.0, 0.0];
        state.entry.phase = EntryPhase::Separation;
        init_p63(&mut state);
        start_servicer(&mut state);

        // ~30 cm/s sensed delta-V over 2 s → ~0.015 g. Well below 0.05g
        // threshold, but enough to verify the path is wired.
        state.pipa_counts = [5, 0, 0];

        assert_eq!(state.entry.sensed_acceleration_g, 0.0, "fixture");
        servicer_task(&mut state);

        assert!(
            state.entry.sensed_acceleration_g > 0.0,
            "PIPA counts must drive sensed_acceleration_g positive, got {}",
            state.entry.sensed_acceleration_g
        );
        // PIPA = 5 counts × 0.0585 m/s = 0.2925 m/s. /2 s = 0.146 m/s².
        // /g0 = 0.0149 g. Allow a wide tolerance — exact value depends on
        // pipa_cal.scale.
        assert!(
            (state.entry.sensed_acceleration_g - 0.015).abs() < 0.005,
            "g-loading out of expected range: {}",
            state.entry.sensed_acceleration_g
        );
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
