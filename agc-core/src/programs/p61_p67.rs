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
use crate::navigation::state_vector::inertial_to_earth_fixed;
use crate::navigation::time::met_to_gha;
use crate::programs::p21::R_EARTH;
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
    /// P66 — ballistic hold (entered when closed-loop guidance has diverged).
    ///
    /// Used by MS-E3 as the destination when HUNTEST predicts a range that
    /// differs from the actual range-to-go by more than
    /// `entry_tables::RANGE_ERR_THRESHOLD_KM`. The DAP holds the most recent
    /// roll command; MS-E5 will replace the hold with the actual P66 logic.
    Ballistic,
    /// P65 — up-control / skip-out (UPCONTRL).
    ///
    /// Entered from `EntryPhase::Entry` when HUNTEST converges (range
    /// prediction within `entry_tables::HUNTEST_CONVERGED_KM` of the actual
    /// range-to-go). The SKIPPER feedback law in `guidance::entry::
    /// upcontrol_step` then maintains the converged trajectory by
    /// commanding `ΔL/D` from `(RDOT − RDOTREF)` and `(V − VREF)` errors.
    /// AGC source: `REENTRY_CONTROL.agc:875–1020`.
    Skip,
}

// ── EntryState ────────────────────────────────────────────────────────────────

/// Entry-guidance state block stored on `AgcState`.
#[derive(Clone, Copy, Debug, Default)]
pub struct EntryState {
    /// Current entry-guidance phase.
    pub phase: EntryPhase,
    /// Sensed spacecraft acceleration (g units).
    ///
    /// Updated each SERVICER cycle by `entry_servicer_exit` from the inertial
    /// sensed delta-V `state.servicer_last_dv_inertial`.
    pub sensed_acceleration_g: f64,
    /// Inertial altitude rate `d|r|/dt` (m/s, positive = climbing).
    ///
    /// Updated each SERVICER cycle. Equals `r · v / |r|` evaluated on the
    /// current ECI state vector. AGC correspondence: V16N64 R2 (HDOT).
    pub r_dot_mps: f64,
    /// Roll command the entry guidance law is holding (radians).
    ///
    /// Updated each SERVICER cycle by `guidance::entry::resolve_roll` once the
    /// 0.05g threshold has been crossed (MS-E3). AGC correspondence: `ROLLC`
    /// at `REENTRY_CONTROL.agc:1308`.
    pub roll_command_rad: f64,
    /// Great-circle range from the current sub-satellite point to the target
    /// landing site (km). Updated each SERVICER cycle.
    pub target_range_km: f64,
    /// Predicted total range to target (km) — the sum
    /// `ASKEP + ASP1 + ASPUP + ASP3 + ASPDWN` from HUNTEST.
    ///
    /// Updated each SERVICER cycle in `EntryPhase::Entry` by
    /// `guidance::entry::predict_range`. AGC correspondence: `ASP` (not stored
    /// in AGC erasable; we cache it here for telemetry and the divergence test).
    pub predicted_range_km: f64,
    /// Signed downrange error in km, `target_range_km - predicted_range_km`.
    ///
    /// Drives the HUNTEST L/D update. AGC correspondence: `DIFF` at
    /// `REENTRY_CONTROL.agc:731`.
    pub downrange_error_km: f64,
    /// Signed cross-range distance (km) of the current sub-satellite track
    /// from the great-circle plane through the target. Positive = right of
    /// track. Used by `resolve_roll` to choose bank direction.
    ///
    /// AGC correspondence: `LATANG` at `REENTRY_CONTROL.agc:141` of the
    /// lexicon, scaled by `4 RADIANS`.
    pub crossrange_km: f64,
    /// Last computed vertical L/D command (dimensionless, range `[-LAD, LAD]`).
    ///
    /// Output of `guidance::entry::compute_ld_command`. AGC correspondence:
    /// `L/D` at `REENTRY_CONTROL.agc:1271` (STOREL/D), scaled by `1`.
    pub ld_command: f64,
    /// HUNTEST iterated reference L/D (`LEWD` in REENTRY_CONTROL.agc).
    ///
    /// Initialised to `entry_tables::LEWD_INIT` on first HUNTEST pass
    /// (`FOREHUNT`, AGC line 861). Updated each cycle by
    /// `LEWD += DLEWD`. Never written outside `guidance::entry`.
    pub lewd_ref: f64,
    /// HUNTEST iteration step (`DLEWD` in REENTRY_CONTROL.agc).
    ///
    /// Initialised to `entry_tables::DLEWD_INIT` on first pass. Updated each
    /// cycle by the Newton step
    /// `DLEWD = DLEWD · DIFF / (DIFFOLD − DIFF)` (AGC line 744).
    pub dlewd: f64,
    /// Previous downrange error (km) — `DIFFOLD` in REENTRY_CONTROL.agc.
    ///
    /// Saved at end of each HUNTEST pass for the next cycle's Newton step.
    pub diffold_km: f64,
    /// SKIPPER nonlinear gain — `FACTOR` in REENTRY_CONTROL.agc.
    ///
    /// Updated each `Skip` cycle by UPCONTRL's CONTINU2 block (AGC lines
    /// 955-968) when `D > Q7MIN`; frozen at the previous value otherwise.
    /// `F1 = (A1 - Q7F) / (D - Q7F)` where `A1` is `D` (descending) or
    /// `A0` (climbing), per HUNTEST lines 502 / 535. Stage-A default is
    /// `1.0` (the previous fixed-gain approximation).
    pub factor: f64,
    /// `false` until the first SERVICER cycle in `EntryPhase::Entry`.
    ///
    /// On the first cycle, `lewd_ref` and `dlewd` are initialised from the
    /// `entry_tables` constants (the FOREHUNT block in REENTRY_CONTROL.agc).
    pub hunt_initialized: bool,
    /// Target landing site geodetic latitude (rad, positive north).
    /// Loaded from ground uplink before P61; default 0.0 = equator.
    pub target_lat_rad: f64,
    /// Target landing site longitude (rad, positive east, range `(-π, π]`).
    /// Loaded from ground uplink before P61; default 0.0 = Greenwich.
    pub target_lon_rad: f64,
    /// `true` once `p67_deploy_drogue` has run.
    pub drogue_deployed: bool,
}

impl EntryState {
    /// `const` constructor usable inside `AgcState::new`.
    pub const fn new() -> Self {
        Self {
            phase: EntryPhase::Idle,
            sensed_acceleration_g: 0.0,
            r_dot_mps: 0.0,
            roll_command_rad: 0.0,
            target_range_km: 0.0,
            predicted_range_km: 0.0,
            downrange_error_km: 0.0,
            crossrange_km: 0.0,
            ld_command: 0.0,
            lewd_ref: 0.0,
            dlewd: 0.0,
            diffold_km: 0.0,
            factor: 1.0,
            hunt_initialized: false,
            target_lat_rad: 0.0,
            target_lon_rad: 0.0,
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

/// Write the V16N64 continuous-monitor entry status triplet
/// (sensed g / R-dot / range-to-go) to the DSKY.
fn write_entry_status(state: &mut crate::AgcState) {
    state.dsky.r[0] = state.entry.sensed_acceleration_g as f32;
    state.dsky.r[1] = state.entry.r_dot_mps as f32;
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

/// Inertial altitude rate `r · v / |r|` (m/s) from the current CSM state vector.
///
/// Returns 0 if the position vector is zero (uninitialised fixture).
fn compute_r_dot(state: &crate::AgcState) -> f64 {
    let r = state.csm_state.position;
    let v = state.csm_state.velocity;
    let r_mag2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
    if r_mag2 == 0.0 {
        return 0.0;
    }
    (r[0] * v[0] + r[1] * v[1] + r[2] * v[2]) / libm::sqrt(r_mag2)
}

/// Great-circle range (km) from the current sub-satellite point to the target
/// landing site stored in `EntryState`.
///
/// Computes the haversine distance on the mean spherical Earth. Uses
/// `met_to_gha` + `inertial_to_earth_fixed` to find the current sub-satellite
/// lat/lon from the ECI position. Returns 0 if the position is at the origin.
fn compute_range_to_go_km(state: &crate::AgcState) -> f64 {
    let pos_eci = state.csm_state.position;
    let r_mag2 = pos_eci[0] * pos_eci[0] + pos_eci[1] * pos_eci[1] + pos_eci[2] * pos_eci[2];
    if r_mag2 == 0.0 {
        return 0.0;
    }
    let gha = met_to_gha(state.time, state.gha_epoch_rad);
    let pos_ef = inertial_to_earth_fixed(pos_eci, gha);
    let r_mag = libm::sqrt(r_mag2);
    let lat = libm::asin(pos_ef[2] / r_mag);
    let lon = libm::atan2(pos_ef[1], pos_ef[0]);

    let dlat = state.entry.target_lat_rad - lat;
    let dlon = state.entry.target_lon_rad - lon;
    let sd_lat = libm::sin(dlat * 0.5);
    let sd_lon = libm::sin(dlon * 0.5);
    let a =
        sd_lat * sd_lat + libm::cos(lat) * libm::cos(state.entry.target_lat_rad) * sd_lon * sd_lon;
    let c = 2.0 * libm::atan2(libm::sqrt(a), libm::sqrt((1.0 - a).max(0.0)));
    R_EARTH * c / 1000.0
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

    // Refresh V16N64 display quantities each cycle.
    state.entry.r_dot_mps = compute_r_dot(state);
    state.entry.target_range_km = compute_range_to_go_km(state);
    state.entry.crossrange_km = crate::guidance::entry::crossrange_km(state);

    // SERVICER-driven 0.05g threshold trip — advances PreEntry → Entry and
    // switches the DAP to trim-attitude EntryRoll(0.0). MS-E3 picks it up
    // from there.
    p63_check_threshold(state);

    // Closed-loop guidance once we are past 0.05g. Four flavours:
    //   - EntryPhase::Entry     → MS-E3 HUNTEST Newton iteration.
    //   - EntryPhase::Skip      → MS-E4 UPCONTRL / SKIPPER feedback law.
    //   - EntryPhase::Ballistic → MS-E5 P66 — freeze L/D, hold attitude.
    //   - EntryPhase::Final     → MS-E6 PREDICT3 final-phase law.
    //
    // Order matters: predict_range first (consumes the previous LEWD),
    // then the per-phase L/D update, then resolve_roll, then select_phase.
    if matches!(
        state.entry.phase,
        EntryPhase::Entry | EntryPhase::Skip | EntryPhase::Ballistic | EntryPhase::Final
    ) {
        use crate::guidance::entry;

        state.entry.predicted_range_km = entry::predict_range(state);

        let upd = match state.entry.phase {
            EntryPhase::Entry => entry::compute_ld_command(state),
            EntryPhase::Skip => entry::upcontrol_step(state),
            EntryPhase::Ballistic => entry::ballistic_step(state),
            EntryPhase::Final => entry::final_phase_step(state),
            _ => unreachable!(),
        };
        state.entry.ld_command = upd.ld_command;
        state.entry.lewd_ref = upd.lewd_new;
        state.entry.dlewd = upd.dlewd_new;
        state.entry.diffold_km = upd.diffold_new_km;
        state.entry.downrange_error_km = upd.diffold_new_km;
        state.entry.factor = upd.factor_new;
        state.entry.hunt_initialized = true;

        state.entry.roll_command_rad = entry::resolve_roll(state, state.entry.ld_command);
        state.dap_state.mode = DapMode::EntryRoll(state.entry.roll_command_rad);

        if let Some(next) = entry::select_phase(state) {
            state.entry.phase = next;
        }

        // MS-E6 drogue-deploy trigger: AGC `STEEROFF` at
        // REENTRY_CONTROL.agc:1142 — when V drops below VQUIT (~305 m/s),
        // P67 stops steering and commands the SECS to deploy the drogue.
        if state.entry.phase == EntryPhase::Final && !state.entry.drogue_deployed {
            let v_mag = libm::sqrt(
                state.csm_state.velocity[0] * state.csm_state.velocity[0]
                    + state.csm_state.velocity[1] * state.csm_state.velocity[1]
                    + state.csm_state.velocity[2] * state.csm_state.velocity[2],
            );
            if v_mag < crate::guidance::entry_tables::VQUIT_MPS {
                p67_deploy_drogue(state);
                // Drogue is out; closed-loop entry guidance is done.
                state.servicer_exit = None;
            }
        }
    }
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
        // Trim-attitude (zero-bank) roll hold until the MS-E3 closed loop
        // begins computing real bank commands.
        state.dap_state.mode = DapMode::EntryRoll(0.0);
        return true;
    }
    false
}

// ── P64 ───────────────────────────────────────────────────────────────────────

/// Entry point registered in `PROGRAM_TABLE[64]` — closed-loop entry guidance.
///
/// Advances the phase and DSKY display. The actual bank command is produced
/// by the next SERVICER cycle's call into `guidance::entry` (MS-E3) — this
/// matches the AGC, where the P64 entry point is also a thin handoff into
/// the cyclic guidance task.
pub fn init_p64(state: &mut crate::AgcState) -> JobPriority {
    if state.entry.sensed_acceleration_g < ENTRY_THRESHOLD_G {
        raise(state, ALARM_P64_EARLY);
    }
    state.entry.phase = EntryPhase::Entry;

    set_display(state, P64_MAJOR_MODE, VERB_MONITOR, 64);
    write_entry_status(state);
    PRIORITY
}

// ── P67 ───────────────────────────────────────────────────────────────────────

/// Entry point registered in `PROGRAM_TABLE[67]` — final phase.
///
/// Advances the phase and DSKY display. The actual closed-loop final-phase
/// math (PREDICT3) runs each subsequent SERVICER cycle in
/// `entry_servicer_exit`; the drogue deploy fires when `V` drops below
/// `entry_tables::VQUIT_MPS` (AGC `STEEROFF`).
pub fn init_p67(state: &mut crate::AgcState) -> JobPriority {
    if state.entry.phase != EntryPhase::Entry {
        raise(state, ALARM_P67_WRONG_PHASE);
    }
    state.entry.phase = EntryPhase::Final;

    set_display(state, P67_MAJOR_MODE, VERB_DISPLAY, 67);
    state.dsky.r[0] = state.entry.target_range_km as f32;
    state.dsky.r[1] = 0.0;
    state.dsky.r[2] = 0.0;
    PRIORITY
}

/// Latch the drogue-deployed flag and stage the SECS pyro discrete.
///
/// The flight-software-side bookkeeping flag `entry.drogue_deployed` is
/// set once (life-of-mission), and the one-shot `drogue_deploy_pending`
/// flag is raised so the foreground loop's `process_secs_staging` step
/// invokes `hw.secs().deploy_drogue()` on the next iteration. That call
/// resets `drogue_deploy_pending` to keep the discrete edge-triggered.
pub fn p67_deploy_drogue(state: &mut crate::AgcState) {
    state.entry.drogue_deployed = true;
    state.drogue_deploy_pending = true;
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

    /// TC-P67-1: `init_p67` from Entry sets phase = Final and major mode 67.
    ///
    /// With MS-E6 in place, `init_p67` no longer auto-deploys the drogue —
    /// that fires later when the SERVICER cycle observes `V < VQUIT_MPS`.
    /// The deploy itself is exercised by the MS-E6 SERVICER tests.
    #[test]
    fn tc_p67_1_from_entry_sets_final_phase() {
        let mut state = AgcState::new();
        state.entry.phase = EntryPhase::Entry;

        init_p67(&mut state);

        assert_eq!(state.entry.phase, EntryPhase::Final);
        assert_eq!(state.major_mode, P67_MAJOR_MODE);
        assert!(
            !state.entry.drogue_deployed,
            "drogue must not deploy in init_p67 — wait for V < VQUIT in the SERVICER cycle"
        );
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
        // Drogue does NOT deploy on the wrong-phase soft alarm either — same
        // SERVICER trigger as the nominal path.
        assert!(!state.entry.drogue_deployed);
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

    /// TC-ESE-4: `init_p67` leaves the SERVICER hook installed so PREDICT3
    /// can run each cycle. The hook is cleared *after* the drogue deploys.
    #[test]
    fn tc_ese_4_p67_keeps_hook_until_drogue() {
        let mut state = AgcState::new();
        state.entry.phase = EntryPhase::Entry;
        state.servicer_exit = Some(entry_servicer_exit);

        init_p67(&mut state);

        assert!(
            state.servicer_exit.is_some(),
            "init_p67 must keep servicer_exit installed for PREDICT3"
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

    // ── MS-E2: R-dot, range-to-go, and SERVICER-driven threshold trip ─────────

    /// TC-MSE2-1: R-dot is positive for an outbound CSM (`r · v > 0`).
    #[test]
    fn tc_mse2_1_r_dot_climbing() {
        let mut state = AgcState::new();
        // Position on +X, velocity along +X (pure outbound radial).
        state.csm_state.position = [6_500_000.0, 0.0, 0.0];
        state.csm_state.velocity = [50.0, 0.0, 0.0];

        entry_servicer_exit(&mut state);

        assert!(
            (state.entry.r_dot_mps - 50.0).abs() < 1e-6,
            "R-dot should equal radial velocity 50 m/s, got {}",
            state.entry.r_dot_mps
        );
    }

    /// TC-MSE2-2: R-dot is negative when descending and equals
    /// `(r · v) / |r|` for a non-radial velocity.
    #[test]
    fn tc_mse2_2_r_dot_descending_general() {
        let mut state = AgcState::new();
        // Position on +X at 6.5 Mm, velocity has both radial (-30 m/s) and
        // tangential (+7800 m/s) components.
        state.csm_state.position = [6_500_000.0, 0.0, 0.0];
        state.csm_state.velocity = [-30.0, 7_800.0, 0.0];

        entry_servicer_exit(&mut state);

        // r · v / |r| = (-30 * 6.5e6) / 6.5e6 = -30.0.
        assert!(
            (state.entry.r_dot_mps - -30.0).abs() < 1e-6,
            "R-dot should be -30 m/s, got {}",
            state.entry.r_dot_mps
        );
    }

    /// TC-MSE2-3: range-to-go is zero when the sub-satellite point coincides
    /// with the target landing site.
    #[test]
    fn tc_mse2_3_range_to_target_zero_when_aligned() {
        let mut state = AgcState::new();
        // Sub-satellite point at lat=lon=0 (target default). gha_epoch=0 and
        // MET=0 → GHA=0 → ECEF = ECI. Equatorial position on +X gives lat=0,
        // lon=0.
        state.csm_state.position = [7_000_000.0, 0.0, 0.0];
        state.csm_state.velocity = [0.0, 7_500.0, 0.0];
        // entry.target_lat_rad = 0, target_lon_rad = 0 by default.

        entry_servicer_exit(&mut state);

        assert!(
            state.entry.target_range_km < 1e-6,
            "range-to-go should be ~0 km, got {}",
            state.entry.target_range_km
        );
    }

    /// TC-MSE2-4: range-to-go matches the haversine result for a known offset.
    /// Target 1° east of the sub-satellite point ⇒ ~111.2 km on the spherical
    /// Earth approximation.
    #[test]
    fn tc_mse2_4_range_to_target_one_degree_east() {
        let mut state = AgcState::new();
        state.csm_state.position = [7_000_000.0, 0.0, 0.0]; // lat=0, lon=0
        state.csm_state.velocity = [0.0, 7_500.0, 0.0];
        state.entry.target_lat_rad = 0.0;
        state.entry.target_lon_rad = 1.0_f64.to_radians();

        entry_servicer_exit(&mut state);

        // 1° × π/180 × R_EARTH_km = π/180 × 6371 ≈ 111.195 km.
        let expected_km = R_EARTH * 1.0_f64.to_radians() / 1000.0;
        assert!(
            (state.entry.target_range_km - expected_km).abs() < 1e-3,
            "range-to-go ≈ {expected_km} km, got {}",
            state.entry.target_range_km
        );
    }

    /// TC-MSE2-5: SERVICER-driven 0.05g threshold trip — one cycle whose
    /// inertial delta-V corresponds to ≥ 0.05 g advances the phase out of
    /// `PreEntry` and switches the DAP into `EntryRoll(_)`.
    ///
    /// With MS-E3 in place, the same SERVICER cycle also runs the closed-loop
    /// HUNTEST step, which overwrites the trim-attitude `EntryRoll(0.0)` set
    /// by the threshold check with a computed bank. The MS-E2 assertion is
    /// therefore loosened to "phase != PreEntry" and "DAP is in EntryRoll".
    /// The exact bank value is exercised by the MS-E3 tests in
    /// `guidance::entry::tests`.
    #[test]
    fn tc_mse2_5_servicer_drives_threshold_and_dap() {
        let mut state = AgcState::new();
        state.entry.phase = EntryPhase::Separation;
        init_p63(&mut state);
        assert_eq!(state.entry.phase, EntryPhase::PreEntry, "fixture");

        // Pre-stage a delta-V corresponding to ~0.1 g over the 2 s cycle.
        state.servicer_last_dv_inertial = [G0_MPS2 * SERVICER_PERIOD_S * 0.1, 0.0, 0.0];
        state.csm_state.position = [6_500_000.0, 0.0, 0.0];
        // Velocity above VFINAL1 so select_phase doesn't immediately hand off
        // to Final inside the same cycle. The MS-E2 contract is "the
        // threshold trip works"; the closed-loop math is exercised separately.
        state.csm_state.velocity = [0.0, 9_500.0, 0.0];

        let exit = state.servicer_exit.expect("init_p63 installs the hook");
        exit(&mut state);

        assert_ne!(
            state.entry.phase,
            EntryPhase::PreEntry,
            "0.1 g must trip the 0.05 g threshold and advance the phase"
        );
        assert!(
            matches!(state.dap_state.mode, DapMode::EntryRoll(_)),
            "DAP must be in EntryRoll mode, got {:?}",
            state.dap_state.mode
        );
        // The terminal phase after one cycle depends on MS-E3 select_phase —
        // tested separately in guidance::entry::tests.
    }

    /// TC-MSE2-6: SERVICER below threshold — phase and DAP unchanged.
    #[test]
    fn tc_mse2_6_servicer_below_threshold_no_transition() {
        let mut state = AgcState::new();
        state.entry.phase = EntryPhase::Separation;
        init_p63(&mut state);
        let dap_before = state.dap_state.mode;

        // 0.02 g — below the 0.05 g threshold.
        state.servicer_last_dv_inertial = [G0_MPS2 * SERVICER_PERIOD_S * 0.02, 0.0, 0.0];
        state.csm_state.position = [6_500_000.0, 0.0, 0.0];
        state.csm_state.velocity = [0.0, 7_800.0, 0.0];

        let exit = state.servicer_exit.expect("init_p63 installs the hook");
        exit(&mut state);

        assert_eq!(
            state.entry.phase,
            EntryPhase::PreEntry,
            "phase must stay PreEntry below threshold"
        );
        assert_eq!(
            state.dap_state.mode, dap_before,
            "DAP mode unchanged below threshold"
        );
    }

    /// TC-MSE2-7: V16N64 DSKY triplet shows (sensed g / R-dot / range-to-go).
    #[test]
    fn tc_mse2_7_write_entry_status_triplet() {
        let mut state = AgcState::new();
        state.entry.sensed_acceleration_g = 0.123;
        state.entry.r_dot_mps = -45.6;
        state.entry.target_range_km = 1234.5;

        write_entry_status(&mut state);

        assert!((state.dsky.r[0] - 0.123).abs() < 1e-5);
        assert!((state.dsky.r[1] - -45.6).abs() < 1e-3);
        assert!((state.dsky.r[2] - 1234.5).abs() < 1e-1);
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
        // Drogue deploy is now SERVICER-driven (V < VQUIT). init_p67 only
        // sets the phase and DSKY display.
        assert!(!state.entry.drogue_deployed);
    }

    // ── MS-E6 SERVICER-driven drogue deploy (V < VQUIT) ───────────────────────

    /// TC-MSE6-DR-1: SERVICER cycle in Final phase with V < VQUIT deploys
    /// the drogue and clears the SERVICER hook.
    #[test]
    fn tc_mse6_dr_1_low_v_deploys_drogue() {
        use crate::guidance::entry_tables::VQUIT_MPS;

        let mut state = AgcState::new();
        state.entry.phase = EntryPhase::Final;
        state.servicer_exit = Some(entry_servicer_exit);
        // Velocity well below VQUIT (~305 m/s).
        state.csm_state.position = [6_500_000.0, 0.0, 0.0];
        state.csm_state.velocity = [0.0, VQUIT_MPS * 0.5, 0.0];
        state.entry.sensed_acceleration_g = 0.5; // any drag — drogue trigger is V-based.
        assert!(!state.entry.drogue_deployed, "fixture");

        entry_servicer_exit(&mut state);

        assert!(
            state.entry.drogue_deployed,
            "drogue must deploy when V < VQUIT in Final phase"
        );
        assert!(
            state.drogue_deploy_pending,
            "SECS pyro discrete must be staged when drogue deploys (MS-E6b)"
        );
        assert!(
            state.servicer_exit.is_none(),
            "SERVICER hook must be cleared after drogue deploys"
        );
    }

    /// TC-MSE6B-SECS-1: `p67_deploy_drogue` stages the SECS pyro discrete
    /// for the scheduler to consume on its next foreground iteration.
    #[test]
    fn tc_mse6b_secs_1_stages_pyro_discrete() {
        let mut state = AgcState::new();
        assert!(!state.drogue_deploy_pending, "fixture");
        assert!(!state.entry.drogue_deployed, "fixture");

        p67_deploy_drogue(&mut state);

        assert!(
            state.entry.drogue_deployed,
            "bookkeeping flag must latch (life-of-mission)"
        );
        assert!(
            state.drogue_deploy_pending,
            "SECS pyro discrete must be staged for the foreground loop"
        );
    }

    /// TC-MSE6-DR-2: SERVICER cycle in Final phase with V > VQUIT does NOT
    /// deploy the drogue — closed-loop PREDICT3 stays in charge.
    #[test]
    fn tc_mse6_dr_2_high_v_no_drogue() {
        use crate::guidance::entry_tables::VQUIT_MPS;

        let mut state = AgcState::new();
        state.entry.phase = EntryPhase::Final;
        state.servicer_exit = Some(entry_servicer_exit);
        state.csm_state.position = [6_500_000.0, 0.0, 0.0];
        // Above VQUIT and below VFINAL1 — proper Final-phase coast.
        state.csm_state.velocity = [0.0, VQUIT_MPS * 4.0, 0.0];
        state.entry.sensed_acceleration_g = 0.5;

        entry_servicer_exit(&mut state);

        assert!(
            !state.entry.drogue_deployed,
            "drogue must NOT deploy while V > VQUIT"
        );
        assert!(
            state.servicer_exit.is_some(),
            "SERVICER hook must remain installed while V > VQUIT"
        );
    }

    /// TC-MSE6-DR-3: Final-phase SERVICER cycle produces a roll command via
    /// PREDICT3 (i.e., `ld_command` is updated, not the entry-time zero).
    #[test]
    fn tc_mse6_dr_3_predict3_updates_ld() {
        use crate::guidance::entry_tables::VQUIT_MPS;

        let mut state = AgcState::new();
        state.entry.phase = EntryPhase::Final;
        state.servicer_exit = Some(entry_servicer_exit);
        state.csm_state.position = [6_500_000.0, 0.0, 0.0];
        state.csm_state.velocity = [0.0, VQUIT_MPS * 6.0, 0.0]; // mid-Final
        state.entry.sensed_acceleration_g = 0.5;
        state.entry.target_range_km = 200.0;
        // Pre-poison ld_command so we can detect that PREDICT3 wrote it.
        state.entry.ld_command = -99.0;

        entry_servicer_exit(&mut state);

        assert!(
            state.entry.ld_command.abs() <= 0.30 + 1e-9,
            "PREDICT3 must write a clamped L/D; got {}",
            state.entry.ld_command
        );
        assert_ne!(
            state.entry.ld_command, -99.0,
            "PREDICT3 must overwrite ld_command in Final phase"
        );
    }
}
